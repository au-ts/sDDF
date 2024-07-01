#include "usdhc.h"

#include <microkit.h>
#include <sddf/util/printf.h>

#define DEBUG_DRIVER

#ifdef DEBUG_DRIVER
#define LOG_DRIVER(...) do{ sddf_dprintf("uSDHC DRIVER|INFO: "); sddf_dprintf(__VA_ARGS__); }while(0)
#else
#define LOG_DRIVER(...) do{}while(0)
#endif

#define LOG_DRIVER_ERR(...) do{ sddf_printf("uSDHC DRIVER|ERROR: "); sddf_printf(__VA_ARGS__); usdhc_debug(); }while(0)

volatile imx_usdhc_regs_t *usdhc_regs;
volatile uint32_t *iomuxc_regs;

uint8_t *usdhc_dma_buffer_vaddr;
uintptr_t usdhc_dma_buffer_paddr;

#define USDHC_INT_CHANNEL 1

static struct {
    uint16_t rca;
    bool ccs; /* card capacity status; false = SDSC, true = SDHC/SDXC. TODO: card type*/
} card_info = { .rca = 0 };

void usdhc_debug(void) {
    LOG_DRIVER("PRES_STATE: %u, PROT_CTRL: %u, SYS_CTRL: %u, MIX_CTRL: %u, INT_STATUS: %u, INT_STATUS_EN: %u, INT_SIGNAL_EN: %u, VEND_SPEC: %u, VEND_SPEC2: %u, BLK_ATT: %u\n", usdhc_regs->pres_state, usdhc_regs->prot_ctrl, usdhc_regs->sys_ctrl, usdhc_regs->mix_ctrl, usdhc_regs->int_status, usdhc_regs->int_status_en, usdhc_regs->int_signal_en, usdhc_regs->vend_spec, usdhc_regs->vend_spec2, usdhc_regs->blk_att);
    LOG_DRIVER("CMD_RSP0: %u, CMD_RSP1: %u, CMD_RSP2: %u, CMD_RSP3: %u\n", usdhc_regs->cmd_rsp0, usdhc_regs->cmd_rsp1, usdhc_regs->cmd_rsp2, usdhc_regs->cmd_rsp3);
}

void usdhc_notified(void)
{
    microkit_dbg_puts("usdhc IRQ!\n");
    usdhc_debug();
}

void notified(microkit_channel ch)
{
    switch (ch)
    {
    case USDHC_INT_CHANNEL:
        usdhc_notified();
        break;

    default:
        LOG_DRIVER_ERR("notification on unknown channel: %d\n", ch);
        break;
    }

    microkit_irq_ack(ch);
}

void usdhc_mask_interrupts() {
    usdhc_regs->int_signal_en = 0;
}

void usdhc_unmask_interrupts() {
    // TODO: Reenable lol, atm we just use polling
    // usdhc_regs->int_signal_en = 0xfffffff;
}

uint32_t get_command_xfr_typ(sd_cmd_t cmd) {
    // Set bits 29-24 (CMDINDX).
    uint32_t cmd_xfr_typ = (cmd.cmd_index & 0b111111) << 24;
    response_type_t rtype = cmd.cmd_response_type;

    if (cmd.data_present) {
        LOG_DRIVER("command has data present\n");
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_DPSEL;
        usdhc_regs->mix_ctrl |= USDHC_MIX_CTRL_DMAEN;

        // docs say this is ignored by uSDHC in sinle block mode, but it's not (lol)
        // if (multi-blocks), mix_ctrl |= USDHC_MIX_CTLR_AC12EN;
    }

    /* Ref: Table 10-42.
            R7 not in there but it's basically R1...
    */
    if (rtype == RespType_None) {
        // Index & CRC Checks: Disabled. RSPTYP: 00b.
        cmd_xfr_typ &= ~USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ &= ~USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b00 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else if (rtype == RespType_R2) {
        // Index Check: Disabled, CRC Check: Enabled
        cmd_xfr_typ &= ~USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b01 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else if (rtype == RespType_R3 || rtype == RespType_R4) {
        // Index & CRC Checks: Disabled.
        cmd_xfr_typ &= ~USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ &= ~USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b10 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else if (rtype == RespType_R1 || rtype == RespType_R5 || rtype == RespType_R6 || rtype == RespType_R7) {
        // Index & CRC Checks: Enabled.
        cmd_xfr_typ |= USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b10 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else if (rtype == RespType_R1b) {
        // Index & CRC Checks: Enabled.
        cmd_xfr_typ |= USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b11 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else {
        LOG_DRIVER_ERR("unknown rtype!\n");
    }

    // CMDTYP (23-22): Nothing needs this, as not suspend/resume/abort YET (TODO)
    // cmd_xfr_typ |= (0b00 << 22);

    return cmd_xfr_typ;
}

// SEe 10.3.7.1.9.2 for mapping of the responses (section 4-9 of SD card spec)
// too the bits of the responses

/* Ref: 10.3.4.1 Command send & response receive basic operation.

    cmd_index: These bits are set to the command number that is specified in
               bits 45-40 of the command-format in the SD Memory Card Physical
               Layer Specification and SDIO Card Specification.
 */
bool usdhc_send_command_poll(sd_cmd_t cmd, uint32_t cmd_arg)
{
    LOG_DRIVER("running cmd %u (is app_cmd: %d) with arg %u\n", cmd.cmd_index, cmd.is_app_cmd, cmd_arg);

    /* See description of App-Specific commands in §4.3.9 */
    if (cmd.is_app_cmd) {
        // [31:15] RCA!
        // note this is implicitly zero.first tim
        bool success = usdhc_send_command_poll(SD_CMD55_APP_CMD, (uint32_t)card_info.rca << 16);
        if (!success) {
            LOG_DRIVER_ERR("couldn't send CMD55_APP_CMD\n");
            return false;
        }

        // Check APP_CMD in the card status to ensure was recognised as such
        // 4.10; bit 5 is app_cmd for next command....
        uint32_t card_status = usdhc_regs->cmd_rsp0;
        if (!(card_status & SD_CARD_STATUS_APP_CMD)) {
            LOG_DRIVER_ERR("card is not expecting next command to be an ACMD...\n");
            return false;
        }
    }


    // The host driver checks the Command Inhibit DAT field (PRES_STATE[CDIHB]) and
    // the \Command Inhibit CMD field (PRES_STATE[CIHB]) in the Present State register
    // before writing to this register.
    if (usdhc_regs->pres_state & (USDHC_PRES_STATE_CIHB | USDHC_PRES_STATE_CDIHB)) {
        LOG_DRIVER("waiting for command inhibit fields to clear... pres: %u, int_status: %u\n", usdhc_regs->pres_state, usdhc_regs->int_status);
        while (usdhc_regs->pres_state & (USDHC_PRES_STATE_CIHB | USDHC_PRES_STATE_CDIHB));
    }

    if (usdhc_regs->pres_state & USDHC_PRES_STATE_DLA) {
        LOG_DRIVER("waiting for data line active to clear...\n");
        while (usdhc_regs->pres_state & USDHC_PRES_STATE_DLA);
    }

    uint32_t cmd_xfr_typ = get_command_xfr_typ(cmd);
    LOG_DRIVER("has cmd_xfr_typ: %u\n", cmd_xfr_typ);

    // if (iinternal DMA)
    // if (multi-block transfer)

    // TODO: app specific commands (4.3.9 part 1 physical layer spec)

    usdhc_mask_interrupts();
    usdhc_regs->cmd_arg = cmd_arg;
    usdhc_regs->cmd_xfr_typ = cmd_xfr_typ;
    usdhc_unmask_interrupts();

    // wait for command completion (polling; TODO: interrupt!; also timeout?)
    while(!(usdhc_regs->int_status));

    // TODO: at the moment if we hae an error it's up to the caller to dea with it...

    uint32_t status = usdhc_regs->int_status;
    if (status & USDHC_INT_STATUS_CTOE) {
        /* command timeout error */
        LOG_DRIVER_ERR("command timeout error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CCE) {
        /* command CRC error */
        LOG_DRIVER_ERR("command crc error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CIE) {
        /* command index error */
        LOG_DRIVER_ERR("command index error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CEBE) {
        /* command end bit error */
        LOG_DRIVER_ERR("command end bit error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_DTOE) {
        /* data timeout error */
        LOG_DRIVER_ERR("data timeout error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_AC12E) {
        /* auto cmd 12 error */
        LOG_DRIVER_ERR("auto cmd12 error: %u\n", usdhc_regs->autocmd12_err_status);
        return false;
    }

    /* clear CC bit and all command error bits... */
    /* n.b. writing 1 clears it ! lol.... */
    uint32_t the_status = usdhc_regs->int_status;
    if (the_status == USDHC_INT_STATUS_CC) {
        usdhc_regs->int_status = USDHC_INT_STATUS_CC;
    } else {
        LOG_DRIVER_ERR("unknown status at command end: 0x%x\n", the_status);
        return false;
    }

    if (cmd.cmd_response_type == RespType_R1b) {
        LOG_DRIVER("waiting on DAT[0]...\n");
        // [SD-PHY] 4.9.2 R1b  "The Host shall check for busy at the response"
        //          "... an optional busy signal transmitted on the data line"
        while (!(usdhc_regs->pres_state & USDHC_PRES_STATE_DLSL0));
    }

    return true;
}

/* false -> clock is changing frequency and not stable (poll until is).
   true  -> clock is stable.
   Ref: 10.3.7.1.11.4 */
bool is_usdhc_clock_stable() {
    return usdhc_regs->pres_state & USDHC_PRES_STATE_SDSTB;
}

#define KHZ (1000)
#define MHZ (1000 * KHZ)

typedef enum sd_clock_freq {
    /* [SD-PHY] 4.2.1 Card Reset "The cards are initialized... 400KHz clock frequency" */
    ClockSpeedIdentify_400KHz = 400 * KHZ,

    // TODO: Higher speeds are currently never used.
    // ClockSpeedDefaultSpeed_25MHz = 25 * MHZ,
} sd_clock_freq_t;

void usdhc_setup_clock(sd_clock_freq_t frequency) {
    /* [IMX8MDQLQRM] Section 10.3.6.7 Change clock frequency
       - Clear the FRC_SDCLK_ON when changing SDCLKFS or setting RSTA bit
       - Also, make sure that the SDSTB field is high.
    */

    // TODO(quirks): see header file comment.
    usdhc_regs->vend_spec &= ~USDHC_VEND_SPEC_CKEN;
    // usdhc_regs->vend_spec &= ~USDHC_VEND_SPEC_FRC_SDCLK_ON;
    while (!is_usdhc_clock_stable()); // TODO: ... timeout.

    /* TODO: We currently don't have a clock driver....
        we inherit a 150MHz clock from U-Boot, so let's use that...
        (TODO: Is there a good way we can assert this?)
    */
    uint32_t clock_source = 150 * MHZ;
    /* Described by [IMX8MDQLQRM] SYS_CTRL, page 2755.
       Values here are 1-offset compared to datasheet ones */
    uint16_t sdclkfs = 1;
    uint8_t dvs = 1;

    // TODO: We always assume SDR, not DDR. This affects clock calculations.

    /* This logic is based on code in U-Boot...
        https://github.com/u-boot/u-boot/blob/8937bb265a/drivers/mmc/fsl_esdhc_imx.c#L606-L610
    */
    while ((clock_source / (16 * sdclkfs)) > frequency && sdclkfs < 256)
        sdclkfs *= 2;

    while (clock_source / (dvs * sdclkfs) > frequency && dvs < 16)
        dvs++;

    LOG_DRIVER("Found freq %u for target %u Hz\n", clock_source / (dvs * sdclkfs), frequency);

    /* Remove the offset by 1 */
    sdclkfs >>= 1;
    dvs -= 1;

    uint32_t sys_ctrl = usdhc_regs->sys_ctrl;
    sys_ctrl &= ~(0xffff0); // clear DTOCV,SDCLFS,DVS
    sys_ctrl |= (sdclkfs << 8);
    sys_ctrl |= (dvs << 4);
    sys_ctrl |= ((0b1111) << 16); // Set the DTOCV to max
    LOG_DRIVER("Changing clocks(SYS_CTRL) from 0x%x to 0x%x\n", usdhc_regs->sys_ctrl, sys_ctrl);
    usdhc_regs->sys_ctrl = sys_ctrl;

    while (!is_usdhc_clock_stable()); // TODO: ... timeout

    // TODO(quirks): see header file comment
    usdhc_regs->vend_spec |= USDHC_VEND_SPEC_PEREN | USDHC_VEND_SPEC_CKEN;
    // TODO: does it make sense to force it on?
    // usdhc_regs->vend_spec |= USDHC_VEND_SPEC_FRC_SDCLK_ON;
}

/* Ref: See 10.3.4.2.2 "Reset" */
void usdhc_reset(void)
{
    // Perform software reset of all components
    usdhc_regs->sys_ctrl |= USDHC_SYS_CTRL_RSTA;

    /* 80 clock ticks for power up, self-clearing when done */
    usdhc_regs->sys_ctrl |= USDHC_SYS_CTRL_INITA;
    while (!(usdhc_regs->sys_ctrl & USDHC_SYS_CTRL_INITA));


    usdhc_regs->int_status_en |= USDHC_INT_STATUS_EN_TCSEN | USDHC_INT_STATUS_EN_DINTSEN
                              | USDHC_INT_STATUS_EN_BRRSEN | USDHC_INT_STATUS_EN_CINTSEN
                              | USDHC_INT_STATUS_EN_CTOESEN | USDHC_INT_STATUS_EN_CCESEN
                              | USDHC_INT_STATUS_EN_CEBESEN | USDHC_INT_STATUS_EN_CIESEN
                              | USDHC_INT_STATUS_EN_DTOESEN | USDHC_INT_STATUS_EN_DCSESEN
                              | USDHC_INT_STATUS_EN_DEBESEN;

    while (usdhc_regs->pres_state & (USDHC_PRES_STATE_CIHB | USDHC_PRES_STATE_CDIHB));

    // https://github.com/BarrelfishOS/barrelfish/blob/master/usr/drivers/imx8x/sdhc/sdhc.c#L166-L175
    usdhc_regs->mmc_boot = 0;
    usdhc_regs->mix_ctrl = 0;
    usdhc_regs->clk_tune_ctrl_status = 0;
    usdhc_regs->dll_status = 0;

    // TODO(quirks): see header file comment about uboot; uboot does this...
    usdhc_regs->vend_spec |= USDHC_VEND_SPEC_HCKEN | USDHC_VEND_SPEC_IPGEN;

    usdhc_setup_clock(ClockSpeedIdentify_400KHz);

    if (!usdhc_send_command_poll(SD_CMD0_GO_IDLE_STATE, 0x0)) {
        LOG_DRIVER_ERR("reset failed...\n");
    }
}

bool usdhc_supports_3v3_operation() {
    // it also supporsts 1.8/3.0/3.3 but for laziness:
    uint32_t host_cap = usdhc_regs->host_ctrl_cap;
    return host_cap & USDHC_HOST_CTRL_CAP_VS33;
}

// TODO: Also see 4.8 Card State Transition Table

/* Figure 4-2 Card Initialization and Identification Flow of
   Physical Layer Simplified Specification Ver9.10 20231201 */
void shared_sd_setup() {
    // 0x1AA corresponds to Table 4-18 of the spec, with VHS = 2.7-3.6V
    // When the card is in Idle state, the host shall issue CMD8 before ACMD41. In the argument, 'voltage
    // supplied' is set to the host supply voltage and 'check pattern' is set to any 8-bit pattern
    bool success = usdhc_send_command_poll(SD_CMD8_SEND_IF_COND, 0x1AA);
    if (!success) {
        LOG_DRIVER_ERR("ver 1.x sd not handled\nn");
        return;

        // Ver 1.x Standard Capacity SD Memory Card!!!
            /*
            ???????????
                Exception in ACMD41
                    - The response of ACMD41 does not have APP_CMD status. Sending the response of CMD41 in idle
                    state means the card is accepted as legal ACMD41.
                    - As APP_CMD status is defined as "clear by read", APP_CMD status, which is set by ACMD41, may
                    be indicated in the response of next CMD11 or CMD3. However, as ACMD11 and ACMD3 are not
                    defined, it is not necessary to set APP_CMD status.
                    - Host should ignore APP_CMD status in the response of CMD11 and CMD3.
            */
        // for R3 (ACDMD41), cmdrsp0 has R[39:8] which is the OCR register accorinnd to table 4-31 (and table 4-38)
        // see §5.1 OCR Register
        // sddf_printf("ocr register: %u\n", ocr_register);
    } else {
        uint32_t r7_resp = usdhc_regs->cmd_rsp0;
        // See Table 4-40; R[39:8].
        if ((r7_resp & 0xFFF) != 0x1AA) {
            // echoed check pattern wrong & accepted voltage
            LOG_DRIVER_ERR("check pattern wrong... %u, wanted %u (full: %u)\n", r7_resp & 0xFFF, 0x1AA, r7_resp);
            return;
        }

        uint32_t ocr_register;
        uint32_t voltage_window = 0; // 0 => inquiry at first.
        do {
            // Flowchart: ACMD41 with HCS=0
            // also 4.2.3.1 voltage window is 0 => inquiry
            // voltage window is bits 23-0 (so 24 bits i..e 0xffffff mask)
            // needs SD_OCR_CCS for SDHC otherwise loops
            success = usdhc_send_command_poll(SD_ACMD41_SD_SEND_OP_COND, SD_OCR_CCS | (voltage_window & 0xffffff));
            if (!success) {
                LOG_DRIVER_ERR("Not SD Memory Card...\n");
                return;
            }

            ocr_register = usdhc_regs->cmd_rsp0;
            if (!(ocr_register & SD_OCR_POWER_UP_STATUS)) {
                LOG_DRIVER("still initialising, trying again %u\n", ocr_register);
            }

            if (!(usdhc_supports_3v3_operation() && (ocr_register & (SD_OCR_VDD31_32 | SD_OCR_VDD32_33)))) {
                LOG_DRIVER_ERR("not compatible both with 3v3; might be others shared compat\n");
                return;
            }

            voltage_window = (SD_OCR_VDD31_32 | SD_OCR_VDD32_33); // 3v2->3v3 & 3v3->3v4.

            // TODO: Use timer driver.
            volatile int32_t i = 0xfffffff;
            while (i > 0) {
                i--; // blursed busy loop
            }
            // TODO: At the momoent
        /* Receiving of CMD8 expands the ACMD41 function; HCS in the argument and CCS (Card Capacity
Status) in the response. HCS is ignored by cards, which didn't respond to CMD8. However the host should
set HCS to 0 if the card returns no response to CMD8. Standard Capacity SD Memory Card ignores HCS.
If HCS is set to 0, SDHC and SDXC Cards never return ready status (keep busy bit to 0). The busy bit in
the OCR is used by the card to inform the host whether initialization of ACMD41 is completed. Setting the
busy bit to 0 indicates that the card is still initializing. Setting the busy bit to 1 indicates completion of
initialization. Card initialization shall be completed within 1 second from the first ACMD41. The host
repeatedly issues ACMD41 for at least 1 second or until the busy bit are set to 1.
The card checks the operational conditions and the HCS bit in the OCR only at the*/
        } while (!(ocr_register & SD_OCR_POWER_UP_STATUS));

        if (ocr_register & SD_OCR_CCS) {
            /* CCS=1, Ver2.00 or later hih/extended capciaty*/
            LOG_DRIVER("Ver2.00 or later High Capacity or Extended Capacity SD Memory Card\n");
            card_info.ccs = true;
        } else {
            LOG_DRIVER("Ver2.00 or later Standard Capacity SD Memory Card\n");
            card_info.ccs = false;
        }

        success = usdhc_send_command_poll(SD_CMD2_ALL_SEND_CID, 0x0);
        if (!success) {
            LOG_DRIVER_ERR(":( couldn't get CID nnumbers\n");
            return;
        }

        // print out rsp0->rsp3, but we don't actually care lol...
        usdhc_debug();

        success = usdhc_send_command_poll(SD_CMD3_SEND_RELATIVE_ADDR, 0x0);
        if (!success) {
            LOG_DRIVER_ERR("couldn't set RCA\n");
            return;
        }

        card_info.rca = (usdhc_regs->cmd_rsp0 >> 16);
        LOG_DRIVER("\nCard: got RCA: %u\n\n", card_info.rca);

        // TODO: we could, in theory, repeat CMD2/CMD3 for multiple cards.
    }
}

/* 4.3 of SD Spec, 10.3.4.3.2.1 of ref manual */
void usdhc_read_single_block() {
    bool success;

    // TODO check if data transfer active
    usdhc_regs->mix_ctrl &= ~USDHC_MIX_CTRL_MSBSEL; /* disable multiple blocks */
    usdhc_regs->mix_ctrl |= USDHC_MIX_CTRL_DTDSEL; // for reading...

    /* [31:16] RCA, [15:0] Stuff bits*/
    /* move the card to the transfer state */
    success = usdhc_send_command_poll(SD_CMD7_CARD_SELECT, ((uint32_t)card_info.rca << 16));
    if (!success) {
        LOG_DRIVER_ERR("failed to move card to transfer state\n");
        return;
    }

    // This gives garbage????
    /* [31:0] stuff bits */
    // success = usdhc_send_command_poll(SD_ACMD51_SEND_SCR, 0x0);
    // if (!success) {
    //     sddf_printf("failed to get scr\n");
    //     return;
    // }
    // sddf_printf("scr: %u\n", usdhc_regs->cmd_rsp0);

    // success = usdhc_send_command_poll((sd_cmd_t){.cmd_index = 6})


    uint32_t block_length = 512; /* default, also Table 4-24 says it doesn't change anyway lol */

    success = usdhc_send_command_poll(SD_CMD16_SET_BLOCKLEN, block_length);
    if (!success) {
        LOG_DRIVER_ERR("couldn't set block length\n");
        return;
    }

    /* 3. Set the uSDHC block length register to be the same as the block length set for the card in step 2.*/
    usdhc_regs->blk_att = (usdhc_regs->blk_att & ~USDHC_BLK_ATT_BLKSIZE_MASK) | (block_length << USDHC_BLK_ATT_BLKSIZE_SHIFT);


    /* 5. disable buffer read ready; set DMA, enable DCMA (done in send_command)  */
    usdhc_regs->int_status_en &= ~USDHC_INT_STATUS_EN_BRRSEN;
    // also disable buffer write read, since we're not using the buffer
    usdhc_regs->int_status_en &= ~USDHC_INT_STATUS_EN_BWRSEN;
    // idk why the DINT (DMA interrupt ) happens
    usdhc_regs->int_status_en &= ~USDHC_INT_STATUS_EN_DINTSEN;

    /* SDSC Card (CCS=0) uses byte unit address and SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes
unit). */
    // uint32_t data_address = 0;

    // I wrote 0xdeadbeef to this block via uboot.
    uint32_t data_address = 0xd86000;
    if (!card_info.ccs) {
        data_address *= block_length; /* convert to byte address */
    }

    // TODO: DTW = 00b = 1-bit mode
    usdhc_regs->prot_ctrl = (usdhc_regs->prot_ctrl & ~USDHC_PROT_CTRL_DTW_MASK) | (0b00 << USDHC_PROT_CTRL_DTW_SHIFT);

    usdhc_regs->ds_addr = usdhc_dma_buffer_paddr;
    LOG_DRIVER("dma system addr (phys): 0x%x\n", usdhc_regs->ds_addr);

    assert(usdhc_regs->host_ctrl_cap & USDHC_HOST_CTRL_CAP_DMAS);

    /* 5. send command */
    success = usdhc_send_command_poll(SD_CMD17_READ_SINGLE_BLOCK, data_address);
    if (!success) {
        LOG_DRIVER_ERR("failed to read single block\n");
        return;
    }

    // DMASEL (simple or off)
    assert(((usdhc_regs->prot_ctrl >> USDHC_PROT_CTRL_DMASEL_SHIFT) & USDHC_PROT_CTRL_DMASEL_MASK) == 0b00);

    LOG_DRIVER("waiting for transfer complete...\n");
    usdhc_debug();

    // TODO: Gets stuck here.
    /* 6. Wait for the Transfer Complete interrupt. */
    // while (!(usdhc_regs->int_status & (USDHC_INT_STATUS_TC | USDHC_INT_STATUS_DTOE)));
    while (!usdhc_regs->int_status);

    if (usdhc_regs->int_status & USDHC_INT_STATUS_TC) {
        LOG_DRIVER("read complete?\n");
        usdhc_regs->int_status = USDHC_INT_STATUS_TC;
    } else if (usdhc_regs->int_status & USDHC_INT_STATUS_DTOE) {
        LOG_DRIVER_ERR("data timeout error\n");
        assert(false);
    }

    assert(*(uint32_t*)usdhc_dma_buffer_vaddr == 0xdeadbeef);

    // let us write...
    data_address = 0xd86000 + 1;
    if (!card_info.ccs) {
        data_address *= block_length; /* convert to byte address */
    }

    usdhc_regs->mix_ctrl &= ~USDHC_MIX_CTRL_DTDSEL; // for writing
    // idk if this moves, but to be safe...
    usdhc_regs->ds_addr = usdhc_dma_buffer_paddr;

    success = usdhc_send_command_poll(SD_CMD24_WRITE_SINGLE_BLOCK, data_address);
    if (!success) {
        LOG_DRIVER_ERR("failed to write single block\n");
        return;
    }

    while (!usdhc_regs->int_status);

    if (usdhc_regs->int_status & USDHC_INT_STATUS_TC) {
        LOG_DRIVER("write complete?\n");
        usdhc_regs->int_status = USDHC_INT_STATUS_TC;
    } else if (usdhc_regs->int_status & USDHC_INT_STATUS_DTOE) {
        LOG_DRIVER_ERR("data timeout error\n");
        assert(false);
    }

    usdhc_debug();
}

void init()
{
    LOG_DRIVER("hello from usdhc driver\n");

    usdhc_reset();

    // TODO: This appears to be broken and does not work at all; Linux does not
    //       even notice when the card is inserted/removed....
    // if (usdhc_regs->pres_state & USDHC_PRES_STATE_CINST) {
    //     sddf_printf("card inserted\n");
    // } else {
    //     sddf_printf("card not inserted or power on reset\n");
    // }
    // 10.3.4.2.1 Card detect => card detect seems broken

    usdhc_debug();
    shared_sd_setup();

    // set WR_WML to 128 & RD_WML to 128
    usdhc_regs->wtmk_lvl |= (0x01 << 16) | (0x01);
    usdhc_regs->int_status_en = 0xffffffff;

    // Figure 4-13 : SD Memory Card State Diagram (data transfer mode)
    usdhc_read_single_block();
}
