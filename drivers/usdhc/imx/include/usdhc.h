#pragma once

#include <stdint.h>

#include <sddf/util/util.h>

/* The driver is based on the i.MX8 Quad Applications Processor Reference Manual Rev 3.1, 06/2021.
   The following register descriptions and layout are from section 10.3.7.1.1.

    uSDHC1 base address: 30B4_0000h
    uSDHC2 base address: 30B5_0000h
*/

struct imx_usdhc_regs {
    uint32_t ds_addr; /* 0h DMA System Address (DS_ADDR) RW */
    uint32_t blk_att; /* 4h  Block Attributes (BLK_ATT) RW */
    uint32_t cmd_arg; /* 8h   Command Argument (CMD_ARG) RW */
    uint32_t cmd_xfr_typ; /* Ch   Command Transfer Type (CMD_XFR_TYP) RW */
    uint32_t cmd_rsp0; /* 10h Command Response0 (CMD_RSP0) RO */
    uint32_t cmd_rsp1; /* 14h Command Response1 (CMD_RSP1) RO */
    uint32_t cmd_rsp2; /* 18h Command Response2 (CMD_RSP2) RO */
    uint32_t cmd_rsp3; /* 1Ch Command Response3 (CMD_RSP3) RO */
    uint32_t data_buff_acc_port; /* 20h Data Buffer Access Port (DATA_BUFF_ACC_PORT) RW */
    uint32_t pres_state; /* 24h Present State (PRES_STATE) ROT */
    uint32_t prot_ctrl; /* 28h Protocol Control (PROT_CTRL) RW */
    uint32_t sys_ctrl; /* 2Ch System Control (SYS_CTRL) RW */
    uint32_t int_status; /* 30h Interrupt Status (INT_STATUS) W1C */
    uint32_t int_status_en; /* 34h Interrupt Status Enable (INT_STATUS_EN) RW */
    uint32_t int_signal_en; /* 38h Interrupt Signal Enable (INT_SIGNAL_EN) RW */
    uint32_t autocmd12_err_status; /* 3Ch Auto CMD12 Error Status (AUTOCMD12_ERR_STATUS) RW */
    uint32_t host_ctrl_cap; /* 40h Host Controller Capabilities (HOST_CTRL_CAP) RW */
    uint32_t wtmk_lvl; /* 44h Watermark Level (WTMK_LVL) RW */
    uint32_t mix_ctrl;/* 48h Mixer Control (MIX_CTRL) RW */
    uint32_t force_event; /* 50h Force Event (FORCE_EVENT) WORZ */
    uint32_t adma_err_status; /* 54h ADMA Error Status (ADMA_ERR_STATUS) RO */
    uint32_t adma_sys_addr; /* 58h ADMA System Address (ADMA_SYS_ADDR) RW */
    uint32_t dll_ctrl; /* 60h DLL (Delay Line) Control (DLL_CTRL) RW */
    uint32_t dll_status; /* 64h DLL Status (DLL_STATUS) RO */
    uint32_t clk_tune_ctrl_status; /* 68h CLK Tuning Control and Status (CLK_TUNE_CTRL_STATUS) RW */
    uint32_t strobe_dll_ctrl; /* 70h Strobe DLL control (STROBE_DLL_CTRL) RW */
    uint32_t strobe_dll_status; /* 74h Strobe DLL status (STROBE_DLL_STATUS) RO */
    uint32_t vend_spec; /* C0h Vendor Specific Register (VEND_SPEC) RW */
    uint32_t mmc_boot; /* C4h MMC Boot (MMC_BOOT) RW */
    uint32_t vend_spec2; /* C8h Vendor Specific 2 Register (VEND_SPEC2) RW */
    uint32_t tuning_ctrl; /* CCh Tuning Control (TUNING_CTRL) RW */
    uint32_t cqe; /* 100h Command Queue (CQE) ROZ */
};
typedef volatile struct imx_usdhc_regs imx_usdhc_regs_t;

/* BLK_ATT Bits. See 10.3.7.1.3 */
#define USDHC_BLK_ATT_BLKSIZE_SHIFT 0              /* Transfer block size    */
#define USDHC_BLK_ATT_BLKSIZE_MASK  (BIT(13) - 1)  /* 13 Bits: BLK_ATT[12-0] */

/* CMD_XFR_TYP Bits. See 10.3.7.1.5 */
#define USDHC_CMD_XFR_TYP_CCCEN BIT(19) /* Command CRC check enable */
#define USHDC_CMD_XFR_TYP_CICEN BIT(20) /* Command index check enable */
#define USDHC_CMD_XFR_TYP_DPSEL BIT(21) /* Data present select */

#define USDHC_CMD_XFR_TYP_RSPTYP_SHIFT 16           /* Response type select */
#define USDHC_CMD_XFR_TYP_RSPTYP_MASK  (BIT(2) - 1) /* 2 Bits: XFR_TYP[17-16]*/

/* PRES_STATE Bits. See 10.3.7.1.11 */
#define USDHC_PRES_STATE_CIHB  BIT(0)  /* Command inhibit (CMD) */
#define USDHC_PRES_STATE_CDIHB BIT(1)  /* Command inhibit (DATA) */
#define USDHC_PRES_STATE_DLA   BIT(2)  /* Data line active */
#define USDHC_PRES_STATE_SDSTB BIT(3)  /* SD clock stable */
#define USDHC_PRES_STATE_CINST BIT(16) /* Card inserted. */

/* PROT_CTRL Bits. See 10.3.7.1.12 */
#define USDHC_PROT_CTRL_DMASEL_SHIFT 8             /* DMA select. */
#define USDHC_PROT_CTRL_DMASEL_MASK  (BIT(2) - 1)  /* 2 Bits: PROT_CTLR[9-8] */

/* SYS_CTRL Bits. See 10.3.7.1.13 */
#define USDHC_SYS_CTRL_RSTA  BIT(24)  /* Software reset for all */
#define USDHC_SYS_CTRL_RSTC  BIT(25)  /* Software reset for CMD line */
#define USDHC_SYS_CTRL_RSTD  BIT(26)  /* Software reset for data line */
#define USDHC_SYS_CTRL_INITA BIT(27)  /* Initialization active */
#define USDHC_SYS_CTRL_RSTT  BIT(28)  /* Reset tuning */

/* INT_STATUS Bits. See 10.3.7.1.14 */
#define USDHC_INT_STATUS_CC    BIT(0)  /* Command complete. */
#define USDHC_INT_STATUS_TC    BIT(1)  /* Transfer complete. */
#define USDHC_INT_STATUS_CTOE  BIT(16) /* Command timeout error. */
#define USDHC_INT_STATUS_CCE   BIT(17) /* Command CRC error. */
#define USDHC_INT_STATUS_CEBE  BIT(18) /* Command end bit error */
#define USDHC_INT_STATUS_CIE   BIT(19) /* Command index error. */
#define USDHC_INT_STATUS_DTOE  BIT(20) /* Data timeout error. */
#define USDHC_INT_STATUS_AC12E BIT(24) /* Auto CMD12 error. */

/* INT_STATUS_EN Bits. See 10.3.7.1.15 */
#define USDHC_INT_STATUS_EN_CCSEN    BIT(0)   /* Command complete status enable */
#define USDHC_INT_STATUS_EN_TCSEN    BIT(1)   /* Transfer complete status enable */
#define USDHC_INT_STATUS_EN_BGESEN   BIT(2)   /* Block gap event status enable */
#define USDHC_INT_STATUS_EN_DINTSEN  BIT(3)   /* DMA interrupt status enable */
#define USDHC_INT_STATUS_EN_BWRSEN   BIT(4)   /* Buffer write ready status enable */
#define USDHC_INT_STATUS_EN_BRRSEN   BIT(5)   /* Buffer read ready status enable */
#define USDHC_INT_STATUS_EN_CINSSEN  BIT(6)   /* Card insertion status enable */
#define USDHC_INT_STATUS_EN_CRMSEN   BIT(7)   /* Card removal status enable */
#define USDHC_INT_STATUS_EN_CINTSEN  BIT(8)   /* Card interrupt status enable */
#define USDHC_INT_STATUS_EN_RTESEN   BIT(12)  /* Re-tuning event status enable */
#define USDHC_INT_STATUS_EN_TPSEN    BIT(13)  /* Tuning pass status enable */
#define USDHC_INT_STATUS_EN_CQISEN   BIT(14)  /* Command queuing status enable */
#define USDHC_INT_STATUS_EN_CTOESEN  BIT(16)  /* Command timeout error status enable */
#define USDHC_INT_STATUS_EN_CCESEN   BIT(17)  /* Command CRC error status enable */
#define USDHC_INT_STATUS_EN_CEBESEN  BIT(18)  /* Command end bit error status enable */
#define USDHC_INT_STATUS_EN_CIESEN   BIT(19)  /* Command indx error status enable */
#define USDHC_INT_STATUS_EN_DTOESEN  BIT(20)  /* Data timeout error status enable*/
#define USDHC_INT_STATUS_EN_DCSESEN  BIT(21)  /* Data CRC error status enable */
#define USDHC_INT_STATUS_EN_DEBESEN  BIT(22)  /* Data end bit error status enable */
#define USDHC_INT_STATUS_EN_AC12ESEN BIT(24)  /* Auto CMD12 error status enable */
#define USDHC_INT_STATUS_EN_TNESEN   BIT(26)  /* Tuning error status enable */
#define USDHC_INT_STATUS_EN_DMAESEN  BIT(28)  /* DMA error status enable */

/* INT_SIGNAL_EN Bits. See 10.3.7.1.16 */
#define USDHC_INT_SIGNAL_CCIEN    BIT(0)   /* Command complete interrupt enable */
#define USDHC_INT_SIGNAL_TCIEN    BIT(1)   /* Transfer complete interrupt enable */
#define USDHC_INT_SIGNAL_BGEIEN   BIT(2)   /* Block gap event interrupt enable */
#define USDHC_INT_SIGNAL_DINTIEN  BIT(3)   /* DMA interrupt interrupt enable */
#define USDHC_INT_SIGNAL_BWRIEN   BIT(4)   /* Buffer write ready interrupt enable */
#define USDHC_INT_SIGNAL_BRRIEN   BIT(5)   /* Buffer read ready interrupt enable */
#define USDHC_INT_SIGNAL_CINSIEN  BIT(6)   /* Card insertion interrupt enable */
#define USDHC_INT_SIGNAL_CRMIEN   BIT(7)   /* Card removal interrupt enable */
#define USDHC_INT_SIGNAL_CINTIEN  BIT(8)   /* Card interrupt enable */
#define USDHC_INT_SIGNAL_RTEIEN   BIT(12)  /* Re-tuning event interrupt enable */
#define USDHC_INT_SIGNAL_TPIEN    BIT(13)  /* Tuning pass interrupt enable */
#define USDHC_INT_SIGNAL_CQIIEN   BIT(14)  /* Command queuing interrupt enable */
#define USDHC_INT_SIGNAL_CTOEIEN  BIT(16)  /* Command timeout error interrupt enable */
#define USDHC_INT_SIGNAL_CCEIEN   BIT(17)  /* Command CRC error interrupt enable */
#define USDHC_INT_SIGNAL_CEBEIEN  BIT(18)  /* Command end bit error interrupt enable */
#define USDHC_INT_SIGNAL_CIEIEN   BIT(19)  /* Command indx error interrupt enable */
#define USDHC_INT_SIGNAL_DTOEIEN  BIT(20)  /* Data timeout error interrupt enable*/
#define USDHC_INT_SIGNAL_DCSEIEN  BIT(21)  /* Data CRC error interrupt enable */
#define USDHC_INT_SIGNAL_DEBEIEN  BIT(22)  /* Data end bit error interrupt enable */
#define USDHC_INT_SIGNAL_AC12EIEN BIT(24)  /* Auto CMD12 error interrupt enable */
#define USDHC_INT_SIGNAL_TNEIEN   BIT(26)  /* Tuning error interrupt enable */
#define USDHC_INT_SIGNAL_DMAEIEN  BIT(28)  /* DMA error interrupt enable */

/* MIX_CTLR Bits. See 10.3.7.1.20 */
#define USDHC_MIX_CTRL_DMAEN  BIT(0)  /* DMA enable */
#define USDHC_MIX_CTLR_AC12EN BIT(2)  /* Auto CMD12 enable */
#define USDHC_MIX_CTRL_DTDSEL BIT(4)  /* Data transfer direction select (1 = read) */
#define USDHC_MIX_CTRL_MSBSEL BIT(5)  /* Mult / Single block select */

/* VEND_SPEC Bits. See 10.3.7.1.29 */
#define USDHC_VEND_SPEC_FRC_SDCLK_ON BIT(8) /* Force CLK output active. */

/* Documentation: 4.9 of the SD Spec */
typedef enum {
    RespType_None = 0,
    RespType_R1,
    RespType_R1b,
    RespType_R2,
    RespType_R3,
    RespType_R4,
    RespType_R5,
    RespType_R5b, // TODO: imx8 made this up lol, see note after table 10-42.
    RespType_R6,
    RespType_R7,
} response_type_t;

typedef struct {
    uint8_t cmd_index;
    response_type_t cmd_response_type;
    bool is_app_cmd;
    bool data_present;
} sd_cmd_t;

#define _SD_CMD_DEF(number, rtype, ...)  (sd_cmd_t){.cmd_index = (number), .cmd_response_type = (rtype), .is_app_cmd = false, ##__VA_ARGS__}
#define _SD_ACMD_DEF(number, rtype) (sd_cmd_t){.cmd_index = (number), .cmd_response_type = (rtype), .is_app_cmd = true, .data_present = false}

/* GENERIC? */
#define SD_CMD0_GO_IDLE_STATE       _SD_CMD_DEF(0, RespType_None)
#define SD_CMD2_ALL_SEND_CID        _SD_CMD_DEF(2, RespType_R2)
#define SD_CMD3_SEND_RELATIVE_ADDR  _SD_CMD_DEF(3, RespType_R6)
#define SD_CMD7_CARD_SELECT         _SD_CMD_DEF(7, RespType_R1b)
#define SD_CMD8_SEND_IF_COND        _SD_CMD_DEF(8, RespType_R7)
#define SD_CMD13_SEND_STATUS        _SD_CMD_DEF(13, RespType_R1)
#define SD_CMD16_SET_BLOCKLEN       _SD_CMD_DEF(16, RespType_R1)
#define SD_CMD17_READ_SINGLE_BLOCK  _SD_CMD_DEF(17, RespType_R1, .data_present = true)
#define SD_CMD24_WRITE_SINGLE_BLOCK _SD_CMD_DEF(24, RespType_R1, .data_present = true)
#define SD_CMD55_APP_CMD            _SD_CMD_DEF(55, RespType_R1)

#define SD_ACMD41_SD_SEND_OP_COND   _SD_ACMD_DEF(41, RespType_R3)
#define SD_ACMD51_SEND_SCR          _SD_ACMD_DEF(51, RespType_R1)


/* See Section 4.10.1 / Table 4-42 definitions */
#define SD_CARD_STATUS_APP_CMD  BIT(5)
