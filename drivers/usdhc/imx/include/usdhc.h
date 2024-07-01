#pragma once

#include <stdint.h>

#include <sddf/util/util.h>

/* The driver is based on:

    [IMX8MDQLQRM]: i.MX8 Quad i.MX 8M Dual/8M QuadLite/8M Quad Applications Processors Reference Manual
                   Document Number: IMX8MDQLQRM, Rev 3.1, 06/2021.
                   https://www.nxp.com/webapp/Download?colCode=IMX8MDQLQRM
    [SD-PHY]:      SD Specifications Part 1 Physical Layer Simplified Specification.
                   Version 9.10, Dec. 2023.
                   https://www.sdcard.org/downloads/pls/
*/

/* [IMX8MDQLQRM] Section 10.3.7.1 uSDHC register descriptions */
typedef struct imx_usdhc_regs {
    uint32_t ds_addr;              /* DMA System Address            (RW)   */
    uint32_t blk_att;              /* Block Attributes              (RW)   */
    uint32_t cmd_arg;              /* Command Argument              (RW)   */
    uint32_t cmd_xfr_typ;          /* Command Transfer Type         (RW)   */
    uint32_t cmd_rsp0;             /* Command Response0             (RO)   */
    uint32_t cmd_rsp1;             /* Command Response1             (RO)   */
    uint32_t cmd_rsp2;             /* Command Response2             (RO)   */
    uint32_t cmd_rsp3;             /* Command Response3             (RO)   */
    uint32_t data_buff_acc_port;   /* Data Buffer Access Port       (RW)   */
    uint32_t pres_state;           /* Present State                 (ROT)  */
    uint32_t prot_ctrl;            /* Protocol Control              (RW)   */
    uint32_t sys_ctrl;             /* System Control                (RW)   */
    uint32_t int_status;           /* Interrupt Status              (W1C)  */
    uint32_t int_status_en;        /* Interrupt Status Enable       (RW)   */
    uint32_t int_signal_en;        /* Interrupt Signal Enable       (RW)   */
    uint32_t autocmd12_err_status; /* Auto CMD12 Error Status       (RW)   */
    uint32_t host_ctrl_cap;        /* Host Controller Capabilities  (RW)   */
    uint32_t wtmk_lvl;             /* Watermark Level               (RW)   */
    uint32_t mix_ctrl;             /* Mixer Control                 (RW)   */
    uint32_t force_event;          /* Force Event                   (WORZ) */
    uint32_t adma_err_status;      /* ADMA Error Status             (RO)   */
    uint32_t adma_sys_addr;        /* ADMA System Address           (RW)   */
    uint32_t dll_ctrl;             /* DLL (Delay Line) Control      (RW)   */
    uint32_t dll_status;           /* DLL Status                    (RO)   */
    uint32_t clk_tune_ctrl_status; /* CLK Tuning Control and Status (RW)   */
    uint32_t strobe_dll_ctrl;      /* Strobe DLL control            (RW)   */
    uint32_t strobe_dll_status;    /* Strobe DLL status             (RO)   */
    uint32_t vend_spec;            /* Vendor Specific Register      (RW)   */
    uint32_t mmc_boot;             /* MMC Boot                      (RW)   */
    uint32_t vend_spec2;           /* Vendor Specific 2 Register    (RW)   */
    uint32_t tuning_ctrl;          /* Tuning Control                (RW)   */
    uint32_t cqe;                  /* Command Queue                 (ROZ)  */
} imx_usdhc_regs_t;

#define _LEN(start, end) ((end - start) + 1)
#define _MASK(start, end)  ((BIT(_LEN(start, end)) - 1) << (start))

/* [IMX8MDQLQRM] Section 10.3.7.1.3 Block Attributes */
#define USDHC_BLK_ATT_BLKSIZE_SHIFT 0            /* Transfer block size    */
#define USDHC_BLK_ATT_BLKSIZE_MASK  _MASK(0, 12) /* BLK_ATT[12-0] */

/* [IMX8MDQLQRM] Section 10.3.7.1.5 Command Transfer Type */
#define USDHC_CMD_XFR_TYP_CCCEN BIT(19) /* Command CRC check enable */
#define USHDC_CMD_XFR_TYP_CICEN BIT(20) /* Command index check enable */
#define USDHC_CMD_XFR_TYP_DPSEL BIT(21) /* Data present select */
#define USDHC_CMD_XFR_TYP_RSPTYP_SHIFT 16            /* Response type select */
#define USDHC_CMD_XFR_TYP_RSPTYP_MASK  _MASK(16, 17) /* RSPTYP[17-16] */

/* [IMX8MDQLQRM] Section 10.3.7.1.11 Present State */
#define USDHC_PRES_STATE_CIHB  BIT(0)  /* Command inhibit (CMD) */
#define USDHC_PRES_STATE_CDIHB BIT(1)  /* Command inhibit (DATA) */
#define USDHC_PRES_STATE_DLA   BIT(2)  /* Data line active */
#define USDHC_PRES_STATE_SDSTB BIT(3)  /* SD clock stable */
#define USDHC_PRES_STATE_CINST BIT(16) /* Card inserted. */

/* [IMX8MDQLQRM] Section 10.3.7.1.12 Protocol Control */
#define USDHC_PROT_CTRL_DTW_SHIFT    1             /* Data transfer width.   */
#define USDHC_PROT_CTRL_DTW_MASK     _MASK(1, 2)   /* 2 Bits: PROT_CTRL[2-1] */
#define USDHC_PROT_CTRL_DMASEL_SHIFT 8             /* DMA select.            */
#define USDHC_PROT_CTRL_DMASEL_MASK  _MASK(8, 9)   /* 2 Bits: PROT_CTRL[9-8] */

/* [IMX8MDQLQRM] Section 10.3.7.1.13 System Control*/
#define USDHC_SYS_CTRL_RSTA  BIT(24)  /* Software reset for all */
#define USDHC_SYS_CTRL_RSTC  BIT(25)  /* Software reset for CMD line */
#define USDHC_SYS_CTRL_RSTD  BIT(26)  /* Software reset for data line */
#define USDHC_SYS_CTRL_INITA BIT(27)  /* Initialization active */
#define USDHC_SYS_CTRL_RSTT  BIT(28)  /* Reset tuning */

/* [IMX8MDQLQRM] Section 10.3.7.1.14 Interrupt Status */
#define USDHC_INT_STATUS_CC    BIT(0)  /* Command complete. */
#define USDHC_INT_STATUS_TC    BIT(1)  /* Transfer complete. */
#define USDHC_INT_STATUS_CTOE  BIT(16) /* Command timeout error. */
#define USDHC_INT_STATUS_CCE   BIT(17) /* Command CRC error. */
#define USDHC_INT_STATUS_CEBE  BIT(18) /* Command end bit error */
#define USDHC_INT_STATUS_CIE   BIT(19) /* Command index error. */
#define USDHC_INT_STATUS_DTOE  BIT(20) /* Data timeout error. */
#define USDHC_INT_STATUS_AC12E BIT(24) /* Auto CMD12 error. */

/* [IMX8MDQLQRM] Section 10.3.7.1.15 Interrupt Status Enable */
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

/* [IMX8MDQLQRM] Section 10.3.7.1.16 Interrupt Signal Enable */
#define USDHC_INT_SIGNAL_EN_CCIEN    BIT(0)   /* Command complete interrupt enable */
#define USDHC_INT_SIGNAL_EN_TCIEN    BIT(1)   /* Transfer complete interrupt enable */
#define USDHC_INT_SIGNAL_EN_BGEIEN   BIT(2)   /* Block gap event interrupt enable */
#define USDHC_INT_SIGNAL_EN_DINTIEN  BIT(3)   /* DMA interrupt interrupt enable */
#define USDHC_INT_SIGNAL_EN_BWRIEN   BIT(4)   /* Buffer write ready interrupt enable */
#define USDHC_INT_SIGNAL_EN_BRRIEN   BIT(5)   /* Buffer read ready interrupt enable */
#define USDHC_INT_SIGNAL_EN_CINSIEN  BIT(6)   /* Card insertion interrupt enable */
#define USDHC_INT_SIGNAL_EN_CRMIEN   BIT(7)   /* Card removal interrupt enable */
#define USDHC_INT_SIGNAL_EN_CINTIEN  BIT(8)   /* Card interrupt enable */
#define USDHC_INT_SIGNAL_EN_RTEIEN   BIT(12)  /* Re-tuning event interrupt enable */
#define USDHC_INT_SIGNAL_EN_TPIEN    BIT(13)  /* Tuning pass interrupt enable */
#define USDHC_INT_SIGNAL_EN_CQIIEN   BIT(14)  /* Command queuing interrupt enable */
#define USDHC_INT_SIGNAL_EN_CTOEIEN  BIT(16)  /* Command timeout error interrupt enable */
#define USDHC_INT_SIGNAL_EN_CCEIEN   BIT(17)  /* Command CRC error interrupt enable */
#define USDHC_INT_SIGNAL_EN_CEBEIEN  BIT(18)  /* Command end bit error interrupt enable */
#define USDHC_INT_SIGNAL_EN_CIEIEN   BIT(19)  /* Command indx error interrupt enable */
#define USDHC_INT_SIGNAL_EN_DTOEIEN  BIT(20)  /* Data timeout error interrupt enable*/
#define USDHC_INT_SIGNAL_EN_DCSEIEN  BIT(21)  /* Data CRC error interrupt enable */
#define USDHC_INT_SIGNAL_EN_DEBEIEN  BIT(22)  /* Data end bit error interrupt enable */
#define USDHC_INT_SIGNAL_EN_AC12EIEN BIT(24)  /* Auto CMD12 error interrupt enable */
#define USDHC_INT_SIGNAL_EN_TNEIEN   BIT(26)  /* Tuning error interrupt enable */
#define USDHC_INT_SIGNAL_EN_DMAEIEN  BIT(28)  /* DMA error interrupt enable */

/* [IMX8MDQLQRM] Section 10.3.7.1.18 Host Controller Capabilities */
#define USDHC_HOST_CTRL_CAP_DMAS  BIT(22)  /* DMA Support */
#define USDHC_HOST_CTRL_CAP_VS33  BIT(24)  /* Voltage support 3.3 V */

/* [IMX8MDQLQRM] Section 10.3.7.1.20 Mixer Control */
#define USDHC_MIX_CTRL_DMAEN  BIT(0)  /* DMA enable */
#define USDHC_MIX_CTLR_AC12EN BIT(2)  /* Auto CMD12 enable */
#define USDHC_MIX_CTRL_DTDSEL BIT(4)  /* Data transfer direction select (1 = read) */
#define USDHC_MIX_CTRL_MSBSEL BIT(5)  /* Mult / Single block select */

/* [IMX8MDQLQRM] Section 10.3.7.1.29 Vendor Specific Register */
#define USDHC_VEND_SPEC_FRC_SDCLK_ON BIT(8) /* Force CLK output active. */
/*
    TODO(quirks):
    - U-Boot writes into CKEN/PEREN/HCKEN/IPGEN bits whereas [IMX8MDQLQRM] sees
      those as reserved bits of VEND_SPEC.
    - However, both seem to work....
*/
#define USDHC_VEND_SPEC_CKEN         0x00004000
#define USDHC_VEND_SPEC_PEREN	     0x00002000
#define USDHC_VEND_SPEC_HCKEN	     0x00001000
#define USDHC_VEND_SPEC_IPGEN	     0x00000800


/*
    Below this point is generic non-imx specific SD items from [SD-PHY].
*/

/* [SD-PHY] Section 4.9 Responses */
typedef enum {
    RespType_None = 0,
    RespType_R1,
    RespType_R1b,
    RespType_R2,
    RespType_R3,
    RespType_R4,
    RespType_R5,
    RespType_R6,
    RespType_R7,
} response_type_t;

/* An arbitrary sd_cmd_t type for passing commands */
typedef struct {
    uint8_t cmd_index;
    response_type_t cmd_response_type;
    bool is_app_cmd;
    bool data_present;
} sd_cmd_t;
#define _SD_CMD_DEF(number, rtype, ...)  (sd_cmd_t){.cmd_index = (number), .cmd_response_type = (rtype), .is_app_cmd = false, ##__VA_ARGS__}
#define _SD_ACMD_DEF(number, rtype) (sd_cmd_t){.cmd_index = (number), .cmd_response_type = (rtype), .is_app_cmd = true, .data_present = false}

/* [SD-PHY] Section 4.7.4 Detailed Command Description */
#define SD_CMD0_GO_IDLE_STATE       _SD_CMD_DEF(0, RespType_None)  /* [31:0] stuff bits */
#define SD_CMD2_ALL_SEND_CID        _SD_CMD_DEF(2, RespType_R2)    /* [31:0] stuff bits */
#define SD_CMD3_SEND_RELATIVE_ADDR  _SD_CMD_DEF(3, RespType_R6)    /* [31:0] stuff bits */
#define SD_CMD7_CARD_SELECT         _SD_CMD_DEF(7, RespType_R1b)   /* [31:16] RCA, [15:0] stuff bits */
#define SD_CMD8_SEND_IF_COND        _SD_CMD_DEF(8, RespType_R7)    /* [31:12] zeroed, [11:8] VHS, [7:0] check pattern */
#define SD_CMD13_SEND_STATUS        _SD_CMD_DEF(13, RespType_R1)   /* [31:16] RCA, [15:0] stuff bits */
#define SD_CMD16_SET_BLOCKLEN       _SD_CMD_DEF(16, RespType_R1)   /* [31:0] block length */
#define SD_CMD17_READ_SINGLE_BLOCK  _SD_CMD_DEF(17, RespType_R1, .data_present = true)  /* [31:0] data address */
#define SD_CMD24_WRITE_SINGLE_BLOCK _SD_CMD_DEF(24, RespType_R1, .data_present = true)  /* [31:0] data address */
#define SD_CMD55_APP_CMD            _SD_CMD_DEF(55, RespType_R1)   /* [31:16] RCA, [15:0] stuff bits */

#define SD_ACMD41_SD_SEND_OP_COND   _SD_ACMD_DEF(41, RespType_R3)  /* [31] zero, [30] host capacity status (CCS), [29] eSD reserved , [28] XPC, [27:25] zeroed, [24] S18R, [23:0] Vdd Voltage Window (host) */
#define SD_ACMD51_SEND_SCR          _SD_ACMD_DEF(51, RespType_R1)  /* [31:0] stuff bits */


/* [SD-PHY] Section 4.10.1 Card Status */
#define SD_CARD_STATUS_APP_CMD  BIT(5)   /* The card will expect ACMD */

/* [SD-PHY] Section 5.1 OCR Register */
#define SD_OCR_VDD27_28         BIT(15)  /* Vdd supports 2.7–2.8V */
#define SD_OCR_VDD28_29         BIT(16)  /* Vdd supports 2.8–2.9V */
#define SD_OCR_VDD29_30         BIT(17)  /* Vdd supports 2.9–3.0V */
#define SD_OCR_VDD30_31         BIT(18)  /* Vdd supports 3.0–3.1V */
#define SD_OCR_VDD31_32         BIT(19)  /* Vdd supports 3.1–3.2V */
#define SD_OCR_VDD32_33         BIT(20)  /* Vdd supports 3.2–3.3V */
#define SD_OCR_VDD33_34         BIT(21)  /* Vdd supports 3.3–3.4V */
#define SD_OCR_VDD34_35         BIT(22)  /* Vdd supports 3.4–3.5V */
#define SD_OCR_VDD35_36         BIT(23)  /* Vdd supports 3.5–3.6V */
#define SD_OCR_CCS              BIT(30)  /* Card Capacity Status (CCS) */
#define SD_OCR_POWER_UP_STATUS  BIT(31)  /* Card power up status bit (busy) */
