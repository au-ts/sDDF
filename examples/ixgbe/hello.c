/*
 * Copyright 2021, Breakaway Consulting Pty. Ltd.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <microkit.h>
#include <stdint.h>
#include <stdbool.h>

#include "printf.h"

const uint64_t device_BASE = 0x200000000;
const uint64_t MSIX_BASE = 0x100000000;
const uint64_t HW_RX_RING = 0x3000000;
const uint64_t HW_TX_RING = 0x3002000;
const uint64_t hw_rx_ring_vaddr = 0x3000000;
const uint64_t hw_rx_ring_paddr = 0x100000000;
const uint64_t hw_tx_ring_vaddr = 0x3002000;
const uint64_t hw_tx_ring_paddr = 0x100002000;

const uint64_t IXGBE_CTRL_LNK_RST  = 0x00000008; /* Link Reset. Resets everything. */
const uint64_t IXGBE_CTRL_RST = 0x04000000; /* Reset (SW) */
const uint64_t IXGBE_CTRL_RST_MASK = IXGBE_CTRL_LNK_RST | IXGBE_CTRL_RST;
const uint64_t IXGBE_CTRL_PCIE_MASTER_DISABLE = 1 << 2;

const uint64_t IXGBE_STATUS_PCIE_MASTER_STATUS = 1 << 19;
const uint64_t IXGBE_CTRL_EXT_DRV_LOAD = 1 << 28;

const uint64_t IXGBE_EEC_ARD = 0x00000200; /* EEPROM Auto Read Done */
const uint64_t IXGBE_RDRXCTL_DMAIDONE = 0x00000008; /* DMA init cycle done */

const uint64_t IXGBE_AUTOC_LMS_SHIFT = 13;
const uint64_t IXGBE_AUTOC_LMS_MASK = 0x7 << IXGBE_AUTOC_LMS_SHIFT;
const uint64_t IXGBE_AUTOC_LMS_10G_SERIAL = 0x3 << IXGBE_AUTOC_LMS_SHIFT;
const uint64_t IXGBE_AUTOC_10G_PMA_PMD_MASK = 0x00000180;
const uint64_t IXGBE_AUTOC_10G_PMA_PMD_SHIFT = 7;
const uint64_t IXGBE_AUTOC_10G_XAUI = 0x0 << IXGBE_AUTOC_10G_PMA_PMD_SHIFT;
const uint64_t IXGBE_AUTOC_AN_RESTART = 0x00001000;

const uint64_t IXGBE_RXCTRL_RXEN = 0x00000001; /* Enable Receiver */

const uint64_t IXGBE_RXPBSIZE_128KB = 0x00020000; /* 128KB Packet Buffer */

const uint64_t IXGBE_HLREG0_RXCRCSTRP = 0x00000002; /* bit  1 */
const uint64_t IXGBE_HLREG0_LPBK = 1 << 15;
const uint64_t IXGBE_RDRXCTL_CRCSTRIP = 0x00000002; /* CRC Strip */

const uint64_t IXGBE_FCTRL_BAM = 0x00000400; /* Broadcast Accept Mode */

const uint64_t IXGBE_CTRL_EXT_NS_DIS = 0x00010000; /* No Snoop disable */

const uint64_t IXGBE_HLREG0_TXCRCEN = 0x00000001; /* bit  0 */
const uint64_t IXGBE_HLREG0_TXPADEN = 0x00000400; /* bit 10 */

const uint64_t IXGBE_TXPBSIZE_40KB = 0x0000A000; /* 40KB Packet Buffer */
const uint64_t IXGBE_RTTDCS_ARBDIS = 0x00000040; /* DCB arbiter disable */

const uint64_t IXGBE_DMATXCTL_TE = 0x1; /* Transmit Enable */

const uint64_t IXGBE_RXDCTL_ENABLE = 0x02000000; /* Ena specific Rx Queue */
const uint64_t IXGBE_TXDCTL_ENABLE = 0x02000000; /* Ena specific Tx Queue */

const uint64_t IXGBE_FCTRL_MPE = 0x00000100; /* Multicast Promiscuous Ena*/
const uint64_t IXGBE_FCTRL_UPE = 0x00000200; /* Unicast Promiscuous Ena */

const uint64_t IXGBE_LINKS_UP = 0x40000000;
const uint64_t IXGBE_LINKS_SPEED_82599 = 0x30000000;
const uint64_t IXGBE_LINKS_SPEED_100_82599 = 0x10000000;
const uint64_t IXGBE_LINKS_SPEED_1G_82599 = 0x20000000;
const uint64_t IXGBE_LINKS_SPEED_10G_82599 = 0x30000000;

const uint32_t IXGBE_IVAR_ALLOC_VAL = 0x80; /* Interrupt Allocation valid */
const uint64_t IXGBE_EICR_RTX_QUEUE = 0x0000FFFF; /* RTx Queue Interrupt */

/* Interrupt clear mask */
const uint64_t IXGBE_IRQ_CLEAR_MASK = 0xFFFFFFFF;

const uint64_t IXGBE_GPIE_MSIX_MODE = 0x00000010; /* MSI-X mode */
const uint64_t IXGBE_GPIE_OCD = 0x00000020; /* Other Clear Disable */
const uint64_t IXGBE_GPIE_EIMEN = 0x00000040; /* Immediate Interrupt Enable */
const uint64_t IXGBE_GPIE_EIAME = 0x40000000;
const uint64_t IXGBE_GPIE_PBA_SUPPORT = 0x80000000;

const uint64_t IXGBE_REG_CTRL = 0x00000;
const uint64_t IXGBE_REG_STATUS = 0x00004;
const uint64_t IXGBE_REG_CTRL_EXT = 0x00018;
const uint64_t IXGBE_REG_EEC = 0x10010;
const uint64_t IXGBE_REG_AUTOC = 0x042A0;
const uint64_t IXGBE_REG_GPRC = 0x04074;
const uint64_t IXGBE_REG_GPTC = 0x04080;
const uint64_t IXGBE_REG_GORCL = 0x04088;
const uint64_t IXGBE_REG_GORCH = 0x0408C;
const uint64_t IXGBE_REG_GOTCL = 0x04090;
const uint64_t IXGBE_REG_GOTCH = 0x04094;
const uint64_t IXGBE_REG_HLREG0 = 0x04240;
const uint64_t IXGBE_REG_LINKS = 0x042A4;
const uint64_t IXGBE_REG_FCTRL = 0x05080;
const uint64_t IXGBE_REG_RXCTRL = 0x03000;
const uint64_t IXGBE_REG_RDRXCTL = 0x02F00;
const uint64_t IXGBE_REG_DTXMXSZRQ = 0x08100;
const uint64_t IXGBE_REG_DMATXCTL = 0x04A80;
const uint64_t IXGBE_REG_RTTDCS = 0x04900;
const uint64_t IXGBE_REG_EICR = 0x00800;
const uint64_t IXGBE_REG_EIMS = 0x00880;
const uint64_t IXGBE_REG_EIMC = 0x00888;
const uint64_t IXGBE_REG_EIAC = 0x00810;
const uint64_t IXGBE_REG_GPIE = 0x00898;
const uint64_t IXGBE_REG_TXDGPC = 0x087A0;
const uint64_t IXGBE_REG_TXDGBCL = 0x087A4;
const uint64_t IXGBE_REG_TXDGBCH = 0x087A8;
const uint64_t IXGBE_REG_CRCERRS = 0x04000;
const uint64_t IXGBE_REG_ILLERRC = 0x04004;
const uint64_t IXGBE_REG_ERRBC = 0x04008;
const uint64_t IXGBE_REG_MLFC = 0x04034;
const uint64_t IXGBE_REG_MRFC = 0x04038;
const uint64_t IXGBE_REG_RLEC = 0x04040;
const uint64_t IXGBE_REG_LXONRXCNT = 0x041A4;
const uint64_t IXGBE_REG_LXOFFRXCNT = 0x041A8;
const uint64_t IXGBE_REG_RXDGPC = 0x02F50;
const uint64_t IXGBE_REG_RXDGBCL = 0x02F54;
const uint64_t IXGBE_REG_RXDGBCH = 0x02F58;
const uint64_t IXGBE_REG_BPRC = 0x04078;
const uint64_t IXGBE_REG_MPRC = 0x0407c;
const uint64_t IXGBE_REG_BPTC = 0x040F4;
const uint64_t IXGBE_REG_MPTC = 0x040F0;
const uint64_t IXGBE_REG_RUC = 0x040A4;
const uint64_t IXGBE_REG_RFC = 0x040A8;
const uint64_t IXGBE_REG_ROC = 0x040AC;
const uint64_t IXGBE_REG_RJC = 0x040B0;

const uint64_t IXGBE_SRRCTL_DESCTYPE_MASK  = 0x0E000000;
const uint64_t IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF = 0x02000000;
const uint64_t IXGBE_SRRCTL_DROP_EN = 0x10000000;

const uint32_t IXGBE_RXD_STAT_DD = 0x01; /* Descriptor Done */
const uint32_t IXGBE_RXD_STAT_EOP = 0x02; /* End of Packet */
const uint32_t IXGBE_RXDADV_STAT_DD = IXGBE_RXD_STAT_DD; /* Done */
const uint32_t IXGBE_RXDADV_STAT_EOP = IXGBE_RXD_STAT_EOP; /* End of Packet */

const uint32_t IXGBE_ADVTXD_PAYLEN_SHIFT = 14; /* Adv desc PAYLEN shift */
const uint32_t IXGBE_TXD_CMD_EOP = 0x01000000; /* End of Packet */
const uint32_t IXGBE_ADVTXD_DCMD_EOP = IXGBE_TXD_CMD_EOP; /* End of Packet */
const uint32_t IXGBE_TXD_CMD_RS = 0x08000000; /* Report Status */
const uint32_t IXGBE_ADVTXD_DCMD_RS = IXGBE_TXD_CMD_RS; /* Report Status */
const uint32_t IXGBE_TXD_CMD_IFCS = 0x02000000; /* Insert FCS (Ethernet CRC) */
const uint32_t IXGBE_ADVTXD_DCMD_IFCS = IXGBE_TXD_CMD_IFCS; /* Insert FCS */
const uint32_t IXGBE_TXD_CMD_DEXT = 0x20000000; /* Desc extension (0 = legacy) */
const uint32_t IXGBE_ADVTXD_DTYP_DATA = 0x00300000; /* Adv Data Descriptor */
const uint32_t IXGBE_ADVTXD_DCMD_DEXT = IXGBE_TXD_CMD_DEXT; /* Desc ext 1=Adv */
const uint32_t IXGBE_TXD_STAT_DD = 0x00000001; /* Descriptor Done */
const uint32_t IXGBE_ADVTXD_STAT_DD = IXGBE_TXD_STAT_DD; /* Descriptor Done */

struct array_register {

    uint64_t offset;
    uint64_t num_regs;
    uint64_t multiplier;
};

#define DECLARE_ARRAY_REGISTER(reg_, offset_, num_regs_, multiplier_) \
    const struct array_register IXGBE_ARRAY_REG_##reg_ = (struct array_register ){ \
	.offset = offset_, \
	.num_regs = num_regs_, \
	.multiplier = multiplier_, \
    };

DECLARE_ARRAY_REGISTER(RDBAL, 0x01000, 64, 0x40);
DECLARE_ARRAY_REGISTER(RDBAH, 0x01004, 64, 0x40);
DECLARE_ARRAY_REGISTER(RDLEN, 0x01008, 64, 0x60);
DECLARE_ARRAY_REGISTER(RDH, 0x01010, 64, 0x40);
DECLARE_ARRAY_REGISTER(RDT, 0x01018, 64, 0x40);
DECLARE_ARRAY_REGISTER(SRRCTL, 0x02100, 16, 0x4);
DECLARE_ARRAY_REGISTER(RXPBSIZE, 0x03C00, 8, 0x4);
DECLARE_ARRAY_REGISTER(DCA_RXCTRL, 0x0100C, 64, 0x40);
DECLARE_ARRAY_REGISTER(RXDCTL, 0x01028, 64, 0x40);

DECLARE_ARRAY_REGISTER(TDBAL, 0x06000, 64, 0x40);
DECLARE_ARRAY_REGISTER(TDBAH, 0x06004, 64, 0x40);
DECLARE_ARRAY_REGISTER(TDLEN, 0x06008, 64, 0x40);
DECLARE_ARRAY_REGISTER(TDH, 0x06010, 64, 0x40);
DECLARE_ARRAY_REGISTER(TDT, 0x06018, 64, 0x40);
DECLARE_ARRAY_REGISTER(TXPBSIZE, 0x0CC00, 8, 0x4);
DECLARE_ARRAY_REGISTER(TXPBTHRESH, 0x04950, 8, 0x4);
DECLARE_ARRAY_REGISTER(DCA_TXCTRL, 0x07200, 128, 0x40);
DECLARE_ARRAY_REGISTER(TXDCTL, 0x06028, 64, 0x40);

DECLARE_ARRAY_REGISTER(IVAR, 0x00900, 64, 0x4);
DECLARE_ARRAY_REGISTER(EITR, 0x00820, 24, 0x4);

DECLARE_ARRAY_REGISTER(QPTC, 0x06030, 16, 0x40);
DECLARE_ARRAY_REGISTER(RXMPC, 0x03FA0, 8, 0x4);

DECLARE_ARRAY_REGISTER(RAL, 0x0A200, 128, 0x8);
DECLARE_ARRAY_REGISTER(RAH, 0x0A204, 128, 0x8);

uint64_t *
get_register(uint64_t base, const uint64_t reg)
{
    return (uint64_t *)(base + reg);
}

uint64_t *
get_array_register(uint64_t base, struct array_register reg, uint64_t index)
{
    if (index >= reg.num_regs) {
	printf("bad array register index");
    }
    return (uint64_t *)(base + reg.offset + index * reg.multiplier);
}

#define REGISTER_READ(reg)				\
    (*get_register(device_BASE, IXGBE_REG_##reg))

#define REGISTER_WRITE(reg, value)				\
    *get_register(device_BASE, IXGBE_REG_##reg) = value;

#define REGISTER_WRITE_FLAGS(reg, flags)	\
    REGISTER_READ(reg) |= flags;

#define REGISTER_CLEAR_FLAGS(reg, flags)	\
    REGISTER_READ(reg) &= !(flags);

#define REGISTER_WAIT_CLEAR(reg, value)			\
    while ((REGISTER_READ(reg) & value) != 0) {}
    
#define REGISTER_WAIT_WRITE(reg, value)			\
    while ((REGISTER_READ(reg) & value) != value) {}

#define ARRAY_REGISTER_READ(reg, index)					\
    (*get_array_register(device_BASE, IXGBE_ARRAY_REG_##reg, index))

#define ARRAY_REGISTER_WRITE(reg, index, value)				\
    *get_array_register(device_BASE, IXGBE_ARRAY_REG_##reg, index) = value;

#define ARRAY_REGISTER_WRITE_FLAGS(reg, index, flags)	\
    ARRAY_REGISTER_READ(reg, index) |= flags;

#define ARRAY_REGISTER_CLEAR_FLAGS(reg, index, flags)	\
    ARRAY_REGISTER_READ(reg, index) &= !(flags);

#define ARRAY_REGISTER_WAIT_CLEAR(reg, index, value)		\
    while ((ARRAY_REGISTER_READ(reg, index) & value) != 0) {}
    
#define ARRAY_REGISTER_WAIT_WRITE(reg, index, value)			\
    while ((ARRAY_REGISTER_READ(reg, index) & value) != value) {}


const uint64_t ONE_MS_IN_NS = 1000000;

typedef struct {
    uint64_t pkt_addr; // Packet buffer address
    uint64_t hdr_addr; // Header buffer address
} ixgbe_adv_rx_desc_read_t;

/* Receive Descriptor - Advanced */
typedef struct {
    uint16_t pkt_info; // RSS, Pkt type
    uint16_t hdr_info; // Splithdr, hdrlen
} ixgbe_adv_rx_desc_wb_lower_lo_dword_hs_rss_t;

typedef union {
    uint32_t data;
    ixgbe_adv_rx_desc_wb_lower_lo_dword_hs_rss_t hs_rss;
} ixgbe_adv_rx_desc_wb_lower_lo_dword_t;

typedef struct {
    uint16_t ip_id; // IP id
    uint16_t csum; // Packet Checksum
} ixgbe_adv_rx_desc_wb_lower_hi_dword_csum_ip_t;

typedef union {
    uint32_t rss; // RSS Hash
    ixgbe_adv_rx_desc_wb_lower_hi_dword_csum_ip_t csum_ip;
} ixgbe_adv_rx_desc_wb_lower_hi_dword_t;

typedef struct {
    ixgbe_adv_rx_desc_wb_lower_lo_dword_t lo_dword;
    ixgbe_adv_rx_desc_wb_lower_hi_dword_t hi_dword;
} ixgbe_adv_rx_desc_wb_lower_t;

typedef struct {
    uint32_t status_error; // ext status/error
    uint16_t length; // Packet length
    uint16_t vlan; // VLAN tag
} ixgbe_adv_rx_desc_wb_upper_t;

typedef struct {
    ixgbe_adv_rx_desc_wb_lower_t lower;
    ixgbe_adv_rx_desc_wb_upper_t upper;
} ixgbe_adv_rx_desc_wb_t;

typedef union {
    ixgbe_adv_rx_desc_read_t read;
    ixgbe_adv_rx_desc_wb_t wb; // writeback
} ixgbe_adv_rx_desc_t;

/* Transmit Descriptor - Advanced */
typedef struct {
    uint64_t buffer_addr; // Address of descriptor's data buf
    uint32_t cmd_type_len;
    uint32_t olinfo_status;
} ixgbe_adv_tx_desc_read_t;

typedef struct {
    uint64_t rsvd; // Reserved
    uint32_t nxtseq_seed;
    uint32_t status;
} ixgbe_adv_tx_desc_wb_t;

typedef union {
    ixgbe_adv_tx_desc_read_t read;
    ixgbe_adv_tx_desc_wb_t wb;
} ixgbe_adv_tx_desc_t;

#define NUM_TX_DESCS 512
#define NUM_RX_DESCS 512
#define TX_CLEAN_BATCH 32

struct ixgbe_device {
    volatile ixgbe_adv_rx_desc_t *rx_ring;
    size_t rx_tail;
    size_t rx_last_head;
    volatile ixgbe_adv_tx_desc_t *tx_ring;
    size_t tx_tail;
    size_t tx_last_head;
    bool tx_slot[NUM_TX_DESCS];
    bool rx_slot[NUM_RX_DESCS];
    bool dump;
    bool rx_dump;
} device;

void
_putchar(char character)
{
    microkit_dbg_putc(character);
}

void
busy_sleep(uint64_t ns)
{
    for (size_t i = 0; i < ns * 3; i++) {
	__asm__ __volatile__ ("nop");
    }
}

void
clear_interrupts(void)
{
    REGISTER_WRITE(EIMC, IXGBE_IRQ_CLEAR_MASK);
    uint64_t res = REGISTER_READ(EICR);
    (void)res;
}

void
disable_interrupts(void)
{
    REGISTER_WRITE(EIMC, 0x0);
    clear_interrupts();
}

void
get_mac_addr(uint8_t mac[6])
{
    uint64_t low = ARRAY_REGISTER_READ(RAL, 0);
    uint64_t high = ARRAY_REGISTER_READ(RAH, 0);

    mac[0] = low & 0xff;
    mac[1] = low >> 8 & 0xff;
    mac[2] = low >> 16 & 0xff;
    mac[3] = low >> 24;
    mac[4] = high & 0xff;
    mac[5] = high >> 8 & 0xff;
}

void
init_link(void)
{
    // link auto-configuration register should already be set correctly, we're resetting it anyway
    REGISTER_WRITE(AUTOC, (REGISTER_READ(AUTOC) & !IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL);
    REGISTER_WRITE(AUTOC, (REGISTER_READ(AUTOC) & !IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI);
    // negotiate link 
    REGISTER_WRITE_FLAGS(AUTOC, IXGBE_AUTOC_AN_RESTART);
    // datasheet wants us to wait for the link here, but we can continue and wait afterwards
}

// Resets the stats of this device.
void
reset_stats(void)
{
    (void)REGISTER_READ(GPRC);
    (void)REGISTER_READ(GPTC);
    (void)REGISTER_READ(GORCL);
    (void)REGISTER_READ(GORCH);
    (void)REGISTER_READ(GOTCL);
    (void)REGISTER_READ(GOTCH);
}

// sections 4.6.7
// Initializes the rx queues of this device.
void
init_rx(void)
{
    // disable rx while re-configuring it
    REGISTER_CLEAR_FLAGS(RXCTRL, IXGBE_RXCTRL_RXEN);

    // enable CRC offloading
    REGISTER_WRITE_FLAGS(HLREG0, IXGBE_HLREG0_RXCRCSTRP);
    REGISTER_WRITE_FLAGS(RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

    // accept broadcast packets
    REGISTER_WRITE_FLAGS(FCTRL, IXGBE_FCTRL_BAM);

    {
        const uint64_t i = 0;

        // probably a broken feature, this flag is initialized with 1 but has to be set to 0
        ARRAY_REGISTER_CLEAR_FLAGS(DCA_RXCTRL, i, 1 << 12);

        // section 4.6.11.3.4 - allocate all queues and traffic to PB0
        ARRAY_REGISTER_WRITE(RXPBSIZE, 0, IXGBE_RXPBSIZE_128KB);

        for (uint64_t i = 0; i < 8; i++) {
            ARRAY_REGISTER_WRITE(RXPBSIZE, i, 0);
        }

        ARRAY_REGISTER_WRITE(
            SRRCTL, i,
            (ARRAY_REGISTER_READ(SRRCTL, i) & !IXGBE_SRRCTL_DESCTYPE_MASK)
            | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF
            );

        // let nic drop packets if no rx descriptor is available instead of buffering them
        ARRAY_REGISTER_WRITE(
            SRRCTL, i,
            ARRAY_REGISTER_READ(SRRCTL, i) | IXGBE_SRRCTL_DROP_EN
            );

        // device.receive_ring = (hw_desc *)hw_rx_ring_vaddr;
        // ARRAY_REGISTER_WRITE(RDBAL, i, hw_rx_ring_paddr & 0xffffffff);
        // ARRAY_REGISTER_WRITE(RDBAH, i, hw_rx_ring_paddr >> 32);

        printf("rx ring %lu phys addr: %lu", i, hw_rx_ring_paddr);

        // ARRAY_REGISTER_WRITE(RDLEN, i, NUM_RX_DESCS * sizeof (hw_desc));
    }

    // last sentence of section 4.6.7 - set some magic bits
    REGISTER_WRITE_FLAGS(CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);

    // start rx
    REGISTER_WRITE_FLAGS(RXCTRL, IXGBE_RXCTRL_RXEN);
}

// section 4.6.8
// Initializes the tx queues of this device.
void
init_tx(void)
{
    // crc offload and small packet padding
    REGISTER_WRITE_FLAGS(HLREG0, IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN);

    // required when not using DCB/VTd
    REGISTER_WRITE(DTXMXSZRQ, 0xffff);
    REGISTER_CLEAR_FLAGS(RTTDCS, IXGBE_RTTDCS_ARBDIS);

    // section 7.1.9 - setup descriptor ring
    {
        // configure a single transmit queue/ring
        const uint64_t i = 0;

        // section 4.6.11.3.4 - set default buffer size allocations
        ARRAY_REGISTER_WRITE(TXPBSIZE, 0, IXGBE_TXPBSIZE_40KB);
        for (uint64_t i = 1; i < 8; i++) {
            ARRAY_REGISTER_WRITE(TXPBSIZE, i, 0);
        }

        ARRAY_REGISTER_WRITE(TXPBTHRESH, 0, 0xA0);

        for (uint64_t i = 1; i < 8; i++) {
            ARRAY_REGISTER_WRITE(TXPBTHRESH, i, 0);
        }

        ARRAY_REGISTER_WRITE(TDBAL, i, hw_tx_ring_paddr & 0xffffffff);
        ARRAY_REGISTER_WRITE(TDBAH, i, hw_tx_ring_paddr >> 32);

        printf("tx ring %lu phys addr: %lu", i, hw_tx_ring_paddr);
        // ARRAY_REGISTER_WRITE(TDLEN, i, NUM_TX_DESCS * sizeof (hw_desc));

        // descriptor writeback magic values, important to get good performance and low PCIe overhead
        // see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
        // we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
        //let mut txdctl = self.read_reg_idx(IxgbeArrayRegs::Txdctl, i);
        // there are no defines for this in ixgbe.rs for some reason
        // pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
        //txdctl &= !(0x3F | (0x3F << 8) | (0x3F << 16));
        //txdctl |= 36 | (8 << 8) | (4 << 16);

        uint64_t txdctl = 0;
        txdctl |= 8 << 16;
        txdctl |= (1 << 8) | 32;

        ARRAY_REGISTER_WRITE(TXDCTL, i, txdctl);
    }
    
    // final step: enable DMA
    REGISTER_WRITE(DMATXCTL, IXGBE_DMATXCTL_TE);
}

void
start_rx_queue(uint64_t queue_id)
{
    // enable queue and wait if necessary
    ARRAY_REGISTER_WRITE(RXDCTL, (uint64_t)queue_id, IXGBE_RXDCTL_ENABLE);
    ARRAY_REGISTER_WAIT_WRITE(RXDCTL, queue_id, IXGBE_RXDCTL_ENABLE);

    // rx queue starts out full
    ARRAY_REGISTER_WRITE(RDH, queue_id, 0);
    ARRAY_REGISTER_WRITE(RDT, queue_id, 0);
    device.rx_last_head = 0;
    device.rx_tail = 0;
}

void
start_tx_queue(uint16_t queue_id)
{
    ARRAY_REGISTER_WRITE(TDH, queue_id, 0);
    ARRAY_REGISTER_WRITE(TDT, queue_id, 0);

    // enable queue and wait if necessary
    ARRAY_REGISTER_WRITE(TXDCTL, queue_id, IXGBE_TXDCTL_ENABLE);
    ARRAY_REGISTER_WAIT_WRITE(TXDCTL, queue_id, IXGBE_TXDCTL_ENABLE);
    device.tx_last_head = 0;
    device.tx_tail = 0;
}

// Enables or disables promiscuous mode of this device.
void
set_promiscuous(bool enabled)
{
    if (enabled) {
	REGISTER_WRITE(FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
    } else {
	REGISTER_CLEAR_FLAGS(FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
    }
}

uint64_t
get_link_speed(void)
{
    uint64_t speed = REGISTER_READ(LINKS);
    if ((speed & IXGBE_LINKS_UP) == 0) {
	return 0;
    }
    switch (speed & IXGBE_LINKS_SPEED_82599) {
    case IXGBE_LINKS_SPEED_100_82599:
	return 100;
    case IXGBE_LINKS_SPEED_1G_82599:
	return 1000;
    case IXGBE_LINKS_SPEED_10G_82599:
	return 10000;
    default:
	return 0;
    }
}

// Waits for the link to come up.
void
wait_for_link(void)
{
    printf("   - waiting for link\n");
    uint64_t speed = get_link_speed();
    uint64_t count = 0;
    while (speed == 0 && count < 100) {
            count += 1;
	    busy_sleep(ONE_MS_IN_NS * 100);
            speed = get_link_speed();
        }
    printf("   - link speed is %lu Mbit/s\n", get_link_speed());
}

void
dump_all_registers(void)
{
    printf("Interrupt regs:\n\tEICR: %08x EIMS: %08x EIMC: %08x\n\tGPIE %08x\n",
	   (uint32_t)REGISTER_READ(EICR),
	   (uint32_t)REGISTER_READ(EIMS),
	   (uint32_t)REGISTER_READ(EIMC),
	   (uint32_t)REGISTER_READ(GPIE));

    printf("Control regs:\n\tCTRL %08x CTRL_EXT %08x\n",
	   (uint32_t)REGISTER_READ(CTRL),
	   (uint32_t)REGISTER_READ(CTRL_EXT)); 
    
    printf("EEPROM regs:\n\tEEC_ARD %08x\n",
	   (uint32_t)REGISTER_READ(EEC));

    printf("AUTOC %08x\n",
	   (uint32_t)REGISTER_READ(AUTOC));

    printf("Receive regs:\n\tRDRXCTRL %08x RXCTRL %08x\n\tHLREG0 %08x FCTRL %08x\n",
	   (uint32_t)REGISTER_READ(RDRXCTL),
	   (uint32_t)REGISTER_READ(RXCTRL),
	   (uint32_t)REGISTER_READ(HLREG0),
	   (uint32_t)REGISTER_READ(FCTRL));

    printf("Transmit regs:\n\tDTXMSSZRQ %08x RTTDCS %08x DMATXCTL: %08x\n",
	   (uint32_t)REGISTER_READ(DTXMXSZRQ),
	   (uint32_t)REGISTER_READ(RTTDCS),
	   (uint32_t)REGISTER_READ(DMATXCTL));
    
    printf("Stats regs:\n\tGPRC %08x GPTC %08x\n\tGORCL %08x GORCH %08x\n\tGOTCL %08x GOTCH %08x\n\tTXDGPC %08x TXDGBCH %08x TXDGBCL %08x QPTC(0) %08x\n",
	   (uint32_t)REGISTER_READ(GPRC),
	   (uint32_t)REGISTER_READ(GPTC),
	   (uint32_t)REGISTER_READ(GORCL),
	   (uint32_t)REGISTER_READ(GORCH),
	   (uint32_t)REGISTER_READ(GOTCL),
	   (uint32_t)REGISTER_READ(GOTCH),
	   (uint32_t)REGISTER_READ(TXDGPC),
	   (uint32_t)REGISTER_READ(TXDGBCH),
	   (uint32_t)REGISTER_READ(TXDGBCL),
	   (uint32_t)ARRAY_REGISTER_READ(QPTC, 0));
}

void
init(void)
{
    printf("disable irqs\n");
    disable_interrupts();

    printf("writing regs\n");
    REGISTER_WRITE(CTRL, IXGBE_CTRL_PCIE_MASTER_DISABLE);

    REGISTER_WAIT_CLEAR(STATUS, IXGBE_STATUS_PCIE_MASTER_STATUS);

    // section 4.6.3.2
    REGISTER_WRITE(CTRL, IXGBE_CTRL_RST_MASK);

    printf("waiting for clear\n");
    REGISTER_WAIT_CLEAR(CTRL, IXGBE_CTRL_RST_MASK);
    printf("sleep\n");
    busy_sleep(100 * ONE_MS_IN_NS);

    printf("resume after sleep\n");
    // section 4.6.3.1 - disable interrupts again after reset
    disable_interrupts();

    printf("no snoop disable bit\n");
    // check for no snoop disable bit
    uint64_t ctrl_ext = REGISTER_READ(CTRL_EXT);
    if ((ctrl_ext & IXGBE_CTRL_EXT_NS_DIS) == 0) {
	REGISTER_WRITE(CTRL_EXT, ctrl_ext | IXGBE_CTRL_EXT_NS_DIS);
    }
    REGISTER_WRITE(CTRL_EXT, IXGBE_CTRL_EXT_DRV_LOAD);

    REGISTER_WRITE(CTRL_EXT, IXGBE_CTRL_EXT_DRV_LOAD);

    uint8_t mac[6];
    get_mac_addr(mac);

    printf("initialising device\n");
    printf("   - MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // section 4.6.3 - wait for EEPROM auto read completion
    REGISTER_WAIT_WRITE(EEC, IXGBE_EEC_ARD);

    // section 4.6.3 - wait for dma initialization done
    REGISTER_WAIT_WRITE(RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);

    // section 4.6.4 - initialize link (auto negotiation) 
    init_link();

    // section 4.6.5 - statistical counters
    // reset-on-read registers, just read them once
    reset_stats();

    // section 4.6.7 - init rx
    init_rx();

    // section 4.6.8 - init tx
    init_tx();

    // start a single receive queue/ring
    start_rx_queue(0);

    // start a single transmit queue/ring
    start_tx_queue(0);

    // section 4.6.3.9 - enable interrupts
    //self.enable_msix_interrupt(0);

    // enable promiscuous mode by default to make testing easier
    set_promiscuous(true);

    // wait some time for the link to come up
    wait_for_link();

    dump_all_registers(); 

    // sleep for 10 seconds. Just stabilize the hardware
    // Well. this ugliness costed us two days of debugging.
    printf("sleep for 15 seconds\n");
    busy_sleep(ONE_MS_IN_NS * 1000 * 3);
    printf("resuming sleep\n");

}

// static inline bool hw_tx_ring_empty(void)
// {
//     uint64_t tx_head = ARRAY_REGISTER_READ(TDH, 0);
//     return ((device.tx_tail - tx_head) % NUM_TX_DESCS) == 0;
// }

// static inline bool hw_tx_ring_full(void)
// {
//     uint64_t tx_head = ARRAY_REGISTER_READ(TDH, 0);
//     return ((device.tx_tail - tx_head + 1) % NUM_TX_DESCS) == 0;
// }

// static inline bool hw_rx_ring_empty(void)
// {
//     uint64_t rx_head = ARRAY_REGISTER_READ(RDH, 0);
//     return ((device.rx_tail - rx_head) % NUM_RX_DESCS) == 0;
// }

// static inline bool hw_rx_ring_full(void)
// {
//     uint64_t rx_head = ARRAY_REGISTER_READ(RDH, 0);
//     return ((device.rx_tail - rx_head + 1) % NUM_RX_DESCS) == 0;
// }

// void
// tx_provide(void)
// {
//     bool reprocess = true;
//     while (reprocess) {
// 	bool provided = false;
//         while (!(hw_tx_ring_full()) && !ring_empty(tx_ring.used_ring)) {
//             buff_desc_t buffer;
//             int err __attribute__((unused)) = dequeue_used(&tx_ring, &buffer);
//             assert(!err);

// 	    volatile ixgbe_adv_tx_desc_t *desc = &device.tx_ring[device.tx_tail];
// 	    desc->read.buffer_addr = buffer.phys;
// 	    desc->cmd_type_len = IXGBE_ADVTXD_DCMD_EOP
// 		| IXGBE_ADVTXD_DCMD_RS
// 		| IXGBE_ADVTXD_DCMD_IFCS
// 		| IXGBE_ADVTXD_DCMD_DEXT
// 		| IXGBE_ADVTXD_DTYP_DATA
// 		| (uint32_t)buffer.len;
// 	    desc->olinfo_status = (uint32_t)buffer.len << IXGBE_ADVTXD_PAYLEN_SHIFT;

//             THREAD_MEMORY_RELEASE();

//             device.tx_tail = (device.tx_tail + 1) % device.tx_size;
// 	    provided = true;
//         }
// 	if (provided) {
// 	    ARRAY_REGISTER_WRITE(TDT, 0, device.tx_tail);
// 	}
	
//         request_signal(tx_ring.used_ring);
//         reprocess = false;

//         if (!hw_ring_full(&tx) && !ring_empty(tx_ring.used_ring)) {
//             cancel_signal(tx_ring.used_ring);
//             reprocess = true;
//         }
//     }
// }

// void
// tx_return(void)
// {
//     bool enqueued = false;
//     while (!hw_tx_ring_empty() && !ring_full(tx_ring.free_ring)) {
//         /* Ensure that this buffer has been sent by the device */
// 	uint32_t status = device.tx_ring[device.tx_head].wb.status;
// 	if ((status & IXGBE_ADVTXD_STAT_DD) == 0) break;

//         THREAD_MEMORY_RELEASE();

//         device.tx_head = (device.tx_head + 1) % tx.size;

//         int err __attribute__((unused)) = enqueue_free(&tx_ring, descr_mdata);
//         assert(!err);
//         enqueued = true;
//     }

//     if (enqueued && require_signal(tx_ring.free_ring)) {
//         cancel_signal(tx_ring.free_ring);
//         microkit_notify(TX_CH);
//     }
// }

// void
// rx_provide(void)
// {
//     bool reprocess = true;
//     while (reprocess) {
//         while (!hw_rx_ring_full() && !ring_empty(rx_ring.free_ring)) {
//             buff_desc_t buffer;
//             int err __attribute__((unused)) = dequeue_free(&rx_ring, &buffer);
//             assert(!err);

	    
// 	    ARRAY_REGISTER_WRITE(RDT, queue_id, 0);

//             uint16_t stat = RXD_EMPTY;
//             if (rx.head + 1 == rx.size) stat |= WRAP;
//             rx.descr_mdata[rx.head] = buffer;
//             update_ring_slot(&rx, rx.head, buffer.phys, 0, stat);

//             THREAD_MEMORY_RELEASE();

//             rx.head = (rx.head + 1) % rx.size;
//         }

//         /* Only request a notification from multiplexer if HW ring not full */
//         if (!hw_ring_full(&rx)) request_signal(rx_ring.free_ring);
//         else cancel_signal(rx_ring.free_ring);
//         reprocess = false;

//         if (!ring_empty(rx_ring.free_ring) && !hw_ring_full(&rx)) {
//             cancel_signal(rx_ring.free_ring);
//             reprocess = true;
//         }
//     }

//     if (!(hw_ring_empty(&rx))) {
//         /* Ensure rx IRQs are enabled */
//         eth->rdar = RDAR_RDAR;
//         if (!(irq_mask & NETIRQ_RXF)) enable_irqs(IRQ_MASK);
//     } else {
//         enable_irqs(NETIRQ_TXF | NETIRQ_EBERR);
//     }
// }

// void
// handle_irq(void)
// {
// }

void
notified(microkit_channel ch)
{
    // printf("INTERRUPT ch=%lu\n", ch);
    // switch (ch) {
    // case IRQ_CH:
	// handle_irq();
	// /*
	//  * Delay calling into the kernel to ack the IRQ until the next loop
	//  * in the seL4CP event handler loop.
	//  */
	// microkit_irq_ack_delayed(ch);
	// break;
    // case RX_CH:
	// rx_provide();
	// break;
    // case TX_CH:
	// tx_provide();
	// break;
    // default:
	// printf("ETH|LOG: received notification on unexpected channel: %X\n", ch);
	// break;
    // }
}
