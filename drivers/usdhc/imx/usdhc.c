#include "usdhc.h"

#include <microkit.h>
#include <sddf/util/printf.h>

imx_usdhc_regs_t *usdhc_regs;
volatile uint32_t *iomuxc_regs;

uint8_t *usdhc_dma_buffer_vaddr;
uintptr_t usdhc_dma_buffer_paddr;

#define USDHC_INT_CHANNEL 1

static struct {
    uint16_t rca;
    bool ccs; /* card capacity status; false = SDSC, true = SDHC/SDXC. TODO: card type*/
} card_info = { .rca = 0 };

void usdhc_debug(void) {
    sddf_printf("uSDHC: PRES_STATE: %u, PROT_CTRL: %u, SYS_CTRL: %u, MIX_CTRL: %u, INT_STATUS: %u, INT_STATUS_EN: %u, INT_SIGNAL_EN: %u, VEND_SPEC: %u, VEND_SPEC2: %u, BLK_ATT: %u\n", usdhc_regs->pres_state, usdhc_regs->prot_ctrl, usdhc_regs->sys_ctrl, usdhc_regs->mix_ctrl, usdhc_regs->int_status, usdhc_regs->int_status_en, usdhc_regs->int_signal_en, usdhc_regs->vend_spec, usdhc_regs->vend_spec2, usdhc_regs->blk_att);
    sddf_printf("uSDHC: CMD_RSP0: %u, CMD_RSP1: %u, CMD_RSP2: %u, CMD_RSP3: %u\n", usdhc_regs->cmd_rsp0, usdhc_regs->cmd_rsp1, usdhc_regs->cmd_rsp2, usdhc_regs->cmd_rsp3);
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
        sddf_printf("notification on unknown channel: %d\n", ch);
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
        sddf_printf("command has data present\n");
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_DPSEL;
        // disable DMA for now...
        usdhc_regs->mix_ctrl |= USDHC_MIX_CTRL_DMAEN | USDHC_MIX_CTLR_AC12EN;
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
    } else if (rtype == RespType_R1b || rtype == RespType_R5b) {
        // Index & CRC Checks: Enabled.
        cmd_xfr_typ |= USHDC_CMD_XFR_TYP_CICEN;
        cmd_xfr_typ |= USDHC_CMD_XFR_TYP_CCCEN;
        cmd_xfr_typ |= (0b11 << USDHC_CMD_XFR_TYP_RSPTYP_SHIFT);
    } else {
        sddf_printf("unknown rtype!\n");
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
    sddf_printf("running cmd %u (is app_cmd: %d) with arg %u\n", cmd.cmd_index, cmd.is_app_cmd, cmd_arg);

    /* See description of App-Specific commands in §4.3.9 */
    if (cmd.is_app_cmd) {
        // [31:15] RCA!
        // note this is implicitly zero.first tim
        bool success = usdhc_send_command_poll(SD_CMD55_APP_CMD, (uint32_t)card_info.rca << 16);
        if (!success) {
            sddf_printf("couldn't send CMD55_APP_CMD\n");
            return false;
        }

        // Check APP_CMD in the card status to ensure was recognised as such
        // 4.10; bit 5 is app_cmd for next command....
        uint32_t card_status = usdhc_regs->cmd_rsp0;
        if (!(card_status & SD_CARD_STATUS_APP_CMD)) {
            sddf_printf("card is not expecting next command to be an ACMD...\n");
            return false;
        }
    }


    // The host driver checks the Command Inhibit DAT field (PRES_STATE[CDIHB]) and
    // the \Command Inhibit CMD field (PRES_STATE[CIHB]) in the Present State register
    // before writing to this register.
    if (usdhc_regs->pres_state & (USDHC_PRES_STATE_CIHB | USDHC_PRES_STATE_CDIHB)) {
        sddf_printf("waiting for command inhibit fields to clear... pres: %u, int_status: %u\n", usdhc_regs->pres_state, usdhc_regs->int_status);
        while (usdhc_regs->pres_state & (USDHC_PRES_STATE_CIHB | USDHC_PRES_STATE_CDIHB));
    }

    if (usdhc_regs->pres_state & USDHC_PRES_STATE_DLA) {
        sddf_printf("waiting for data line active to clear...\n");
        while (usdhc_regs->pres_state & USDHC_PRES_STATE_DLA);
    }

    uint32_t cmd_xfr_typ = get_command_xfr_typ(cmd);
    sddf_printf("has cmd_xfr_typ: %u\n", cmd_xfr_typ);

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
        sddf_printf("command timeout error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CCE) {
        /* command CRC error */
        sddf_printf("command crc error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CIE) {
        /* command index error */
        sddf_printf("command index error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_CEBE) {
        /* command end bit error */
        sddf_printf("command end bit error\n");
        return false;
    } else if (status & USDHC_INT_STATUS_DTOE) {
        /* data timeout error */
        sddf_printf("data timeout error\n");
        usdhc_debug();
        return false;
    }

    // Either of the busy response commands
    // SDIO 4.9.2 R1b spec: The Host shall check for busy at the response.
    if (cmd.cmd_response_type == RespType_R1b || cmd.cmd_response_type == RespType_R5b) {
        sddf_printf("helooooooooooo DAT[0]...\n");
        usdhc_debug();
        while (!(usdhc_regs->pres_state & 0x01000000)); // look at status of DATA[0] line..., wait for go low?
    }

    /* clear CC bit and all command error bits... */
    /* n.b. writing 1 clears it ! lol.... */
    assert(usdhc_regs->int_status == USDHC_INT_STATUS_CC);
    usdhc_regs->int_status = USDHC_INT_STATUS_CC;

    return true;
}

/* false -> clock is changing frequency and not stable (poll until is).
   true  -> clock is stable.
   Ref: 10.3.7.1.11.4 */
bool is_usdhc_clock_stable() {
    return usdhc_regs->pres_state & USDHC_PRES_STATE_SDSTB;
}

// TODO: Why pres_State not ->vend_spec
void usdhc_setup_clock() {
    /* Ref: 10.3.6.7:
       - Clear the FRC_SDCLK_ON when changing SDCLKFS or setting RSTA bit
       - Also, make sure that the SDSTB field is high.
    */
    usdhc_regs->vend_spec &= ~BIT(14); // disable cken
    while (!is_usdhc_clock_stable()); // TODO: ... timeout.

    /*
     TODO: Clock driver! No exist

     NB: Uboot sees to assume 150 MHz base clock, as we have 0x00 and 0b11 prescaler & divisor, so that means => 50Mhz it uses.

     To get 400kHz, divide by 375 ~ 128 * 3, i.e. divisor = 3, prescalar = 0x40
    */

    /* TODO: We assume single data rate mode (DDR_EN of MIX_CTLR = 0) */
    // TODO: do this in a better, generic way.
    sddf_printf("sys_ctrl before: %u, ", usdhc_regs->sys_ctrl);
    uint32_t sys_ctrl = usdhc_regs->sys_ctrl;
    sys_ctrl &= ~(0xffff0); // clear DTOCV,SDCLFS,DVS
    sys_ctrl |= (0x40 << 8);
    sys_ctrl |= (0b0011 << 4);
    sys_ctrl |= ((0b1111) << 16); // Set the DTOCV to max

    usdhc_regs->sys_ctrl = sys_ctrl;
    sddf_printf("after: %u\n", usdhc_regs->sys_ctrl);

    while (!is_usdhc_clock_stable()); // TODO: ... timeout

    usdhc_regs->vend_spec |= BIT(13) | BIT(14); // peren & cken

    // force it on?
    usdhc_regs->vend_spec |= USDHC_VEND_SPEC_FRC_SDCLK_ON;
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

    // https://github.com/BarrelfishOS/barrelfish/blob/master/devices/imx8x/sdhc.dev#L309-L331
    usdhc_regs->vend_spec |= BIT(11) | BIT(12); // hcken & ipgen = 1

    // TODO: This is also broke....
    usdhc_setup_clock(/* 400 kHz */);

    if (!usdhc_send_command_poll(SD_CMD0_GO_IDLE_STATE, 0x0)) {
        sddf_printf("reset failed...\n");
    }
}

bool usdhc_supports_3v3_operation() {
    // it also supporsts 1.8/3.0/3.3 but for laziness:
    uint32_t host_cap = usdhc_regs->host_ctrl_cap;
    return host_cap & BIT(24);
}

#define IOMUX_ALT0 0b000
#define IOMUX_ALT1 0b001
#define IOMUX_ALT5 0b101

// value after reset, sane defaults
#define IOMUX_PAD_CTL_PULLUP   BIT(6) /* defaults to pulldown. */

/* Ref: Section 8.
Also see 10.3.2 for settings? TODO: i have no idea if the pad ctl setitngs are correct
*/
void usdhc_setup_iomuxc() {
    /*
        Page 1316-7 ! -- pad mux registers
    */
    /* 8.2.5.12 */
    *(iomuxc_regs + 0x40) = IOMUX_ALT5; /* USDHC1_CD_B : IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06 */

    /* 8.2.5.36 onwards; PAD mux registers... */
    *(iomuxc_regs + 0xA0) = IOMUX_ALT0; /* USDHC1_CLK     : IOMUXC_SW_MUX_CTL_PAD_SD1_CLK */
    *(iomuxc_regs + 0xA4) = IOMUX_ALT0; /* USDHC1_CMD     : IOMUXC_SW_MUX_CTL_PAD_SD1_CMD */
    *(iomuxc_regs + 0xA8) = IOMUX_ALT0; /* USDHC1_DATA0   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA0 */
    *(iomuxc_regs + 0xAC) = IOMUX_ALT0; /* USDHC1_DATA1   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA1 */
    *(iomuxc_regs + 0xB0) = IOMUX_ALT0; /* USDHC1_DATA2   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA2 */
    *(iomuxc_regs + 0xB4) = IOMUX_ALT0; /* USDHC1_DATA3   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA3 */
    *(iomuxc_regs + 0xB8) = IOMUX_ALT0; /* USDHC1_DATA4   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA4 */
    *(iomuxc_regs + 0xBC) = IOMUX_ALT0; /* USDHC1_DATA5   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA5 */
    *(iomuxc_regs + 0xC0) = IOMUX_ALT0; /* USDHC1_DATA6   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA6 */
    *(iomuxc_regs + 0xC4) = IOMUX_ALT0; /* USDHC1_DATA7   : IOMUXC_SW_MUX_CTL_PAD_SD1_DATA7 */
    *(iomuxc_regs + 0xC8) = IOMUX_ALT0; /* USDHC1_RESET_B : IOMUXC_SW_MUX_CTL_PAD_SD1_RESET_B */
    *(iomuxc_regs + 0xCC) = IOMUX_ALT0; /* USDHC1_STROBE  : IOMUXC_SW_MUX_CTL_PAD_SD1_STROBE */

    /* 8.2.5.9 */
    *(iomuxc_regs + 0x34) = IOMUX_ALT1; /* USDHC1_VSELECT : IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO03 */
    *(iomuxc_regs + 0x44) = IOMUX_ALT5; /* USDHC1_WP      : IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO07 */

    /*
        Pad control registers; pg 1429, 8.2.5.190 onwrads
    */
   // tie low => card attached, otherwise, high = can card detect...
    *(iomuxc_regs + 0x2A8) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_CD_B : IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06 */

    *(iomuxc_regs + 0x308) |= 0; /* USDHC1_CLK     : IOMUXC_SW_PAD_CTL_PAD_SD1_CLK */
    *(iomuxc_regs + 0x30C) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_CMD     : IOMUXC_SW_PAD_CTL_PAD_SD1_CMD */
    *(iomuxc_regs + 0x310) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA0   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA0 */
    *(iomuxc_regs + 0x314) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA1   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA1 */
    *(iomuxc_regs + 0x318) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA2   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA2 */
    *(iomuxc_regs + 0x31C) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA3   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA3 */
    *(iomuxc_regs + 0x320) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA4   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA4 */
    *(iomuxc_regs + 0x324) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA5   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA5 */
    *(iomuxc_regs + 0x328) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA6   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA6 */
    *(iomuxc_regs + 0x32C) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_DATA7   : IOMUXC_SW_PAD_CTL_PAD_SD1_DATA7 */
    *(iomuxc_regs + 0x330) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_RESET_B : IOMUXC_SW_PAD_CTL_PAD_SD1_RESET_B */ /* active low*/
    *(iomuxc_regs + 0x334) |= 0; /* USDHC1_STROBE  : IOMUXC_SW_PAD_CTL_PAD_SD1_STROBE */

    *(iomuxc_regs + 0x29C) |= 0; /* USDHC1_VSELECT : IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03 */
    // tied high for card write protect detect
    *(iomuxc_regs + 0x2AC) |= IOMUX_PAD_CTL_PULLUP; /* USDHC1_WP : IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07 */
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
        sddf_printf("not hanled\nn");
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
            sddf_printf("check pattern wrong... %u, wanted %u (full: %u)\n", r7_resp & 0xFFF, 0x1AA, r7_resp);
            return;
        }

        uint32_t ocr_register;
        uint32_t voltage_window = 0; // 0 => inquiry at first.
        do {
            // Flowchart: ACMD41 with HCS=0
            // also 4.2.3.1 voltage window is 0 => inquiry
            // voltage window is bits 23-0 (so 24 bits i..e 0xffffff mask)
            // needs BIT(30) for SDHC otherwise loops
            success = usdhc_send_command_poll(SD_ACMD41_SD_SEND_OP_COND, BIT(30) | (voltage_window & 0xffffff));
            if (!success) {
                sddf_printf("Not SD Memory Card...\n");
                usdhc_debug();
                return;
            }

            ocr_register = usdhc_regs->cmd_rsp0;
            if (!(ocr_register & BIT(31))) {
                sddf_printf("still initialising, trying again %u\n", ocr_register);
            }

            if (!(usdhc_supports_3v3_operation() && ((ocr_register & BIT(19)) || (ocr_register & BIT(20))))) {
                sddf_printf("not compatible both with 3v3; might be others shared compat\n");
                return;
            }

            voltage_window = BIT(19) | BIT(20); // 3v2->3v3 & 3v3->3v4.

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
        } while (!(ocr_register & BIT(31)));

        if (ocr_register & BIT(30)) {
            /* CCS=1, Ver2.00 or later hih/extended capciaty*/
            sddf_printf("Ver2.00 or later High Capacity or Extended Capacity SD Memory Card\n");
            card_info.ccs = true;
        } else {
            sddf_printf("Ver2.00 or later Standard Capacity SD Memory Card\n");
            card_info.ccs = false;
        }

        success = usdhc_send_command_poll(SD_CMD2_ALL_SEND_CID, 0x0);
        if (!success) {
            sddf_printf(":( couldn't get CID nnumbers\n");
            return;
        }

        // print out rsp0->rsp3, but we don't actually care lol...
        usdhc_debug();

        success = usdhc_send_command_poll(SD_CMD3_SEND_RELATIVE_ADDR, 0x0);
        if (!success) {
            sddf_printf("couldn't set RCA\n");
            return;
        }

        card_info.rca = (usdhc_regs->cmd_rsp0 >> 16);
        sddf_printf("\nCard: got RCA: %u\n\n", card_info.rca);

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
    // while (true) {
        success = usdhc_send_command_poll(SD_CMD7_CARD_SELECT, ((uint32_t)card_info.rca << 16));
        if (!success) {
            sddf_printf("failed to move card to transfer state\n");
            return;
        }

        // success = usdhc_send_command_poll(SD_CMD7_CARD_SELECT, ((uint32_t)card_info.rca << 16));
        // if (!success) {
        //     sddf_printf("failed to deslect card\n");
        //     return;
        // }
    // }

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
        sddf_printf("couldn't set block length\n");
        return;
    }

    /* 3. Set the uSDHC block length register to be the same as the block length set for the card in step 2.*/
    usdhc_regs->blk_att |= (block_length & USDHC_BLK_ATT_BLKSIZE_MASK) << USDHC_BLK_ATT_BLKSIZE_SHIFT;
    assert(usdhc_regs->blk_att & BIT(16));


    /* 5. disable buffer read ready; set DMA, enable DCMA (done in send_command)  */
    usdhc_regs->int_status_en &= ~USDHC_INT_STATUS_EN_BRRSEN;

    /* SDSC Card (CCS=0) uses byte unit address and SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes
unit). */
    uint32_t data_address = 0; /* 1st block (0 is MBR i believe )*/
    if (!card_info.ccs) {
        data_address *= block_length; /* convert to byte address */
    }

    // TODO: set elsewhere.
    usdhc_regs->prot_ctrl &= ~0b110; // clear DTW (2-1)
    // usdhc_regs->prot_ctrl |= 0b010; // set DTW 4 bit mode

    usdhc_regs->ds_addr = usdhc_dma_buffer_paddr;
    sddf_printf("dma system addr (phys): 0x%lx\n", usdhc_dma_buffer_paddr);
    sddf_printf("dma system addr (phys): 0x%x\n", usdhc_regs->ds_addr);
    usdhc_debug();

    assert(usdhc_regs->host_ctrl_cap & BIT(22));

    sddf_printf("waiting for a bit\n");
            volatile int32_t i = 0xfffffff;
            while (i > 0) {
                i--; // blursed busy loop
            }

    sddf_printf("pre-read memory! %u\n", *(uint32_t*)usdhc_dma_buffer_vaddr);

    /* 5. send command */
    success = usdhc_send_command_poll(SD_CMD17_READ_SINGLE_BLOCK, data_address);
    if (!success) {
        sddf_printf("failed to read single block\n");
        return;
    }

    sddf_printf("waiting for transfer complete...\n");
    usdhc_debug();

    // DMASEL (simple or off)
    assert(!(usdhc_regs->prot_ctrl & BIT(9))  && !(usdhc_regs->prot_ctrl & BIT(8)));

    // TODO: Gets stuck here.
    /* 6. Wait for the Transfer Complete interrupt. */
    // while (!(usdhc_regs->int_status & (USDHC_INT_STATUS_TC | USDHC_INT_STATUS_DTOE)));
    while (!usdhc_regs->int_status);

    if (usdhc_regs->int_status & USDHC_INT_STATUS_TC) {
        sddf_printf("transfer complete?\n");
    } else if (usdhc_regs->int_status & USDHC_INT_STATUS_DTOE) {
        sddf_printf("data timeout error\n");
        usdhc_debug();
        assert(false);
    }

    sddf_printf("read memory! %u\n", *(uint32_t*)usdhc_dma_buffer_vaddr);
}

void init()
{
    microkit_dbg_puts("hello from usdhc driver\n");

    // This is all.... very broken... and yet somehow disabling it has permanently
    // made it work again.
    // usdhc_setup_iomuxc();

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
