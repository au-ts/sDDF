/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <microkit.h>
#include <sddf/util/util.h>
#include <sddf/serial/queue.h>

#if defined(CONFIG_PLAT_ODROIDC2)
#define UART_REGS_OFFSET (0x4c0)
#elif defined(CONFIG_PLAT_ODROIDC4)
#define UART_REGS_OFFSET (0x0)
#else
#error "Unexpected platform used with UART meson driver"
#endif

/* The driver is based on the Amlogic S905X3 Data Sheet Revision 02.
The following register descriptions and layout are from section 13.5.4.*/

struct meson_uart_regs {
    uint8_t wfifo;                                      /* 0x000 Write Data */
    uint8_t unused0[3];
    uint8_t rfifo;                                      /* 0x040 Read Data */
    uint8_t unused1[3];
    uint32_t cr;                                        /* 0x080 Control Register */
    uint32_t sr;                                        /* 0x0c0 Status Register */
    uint32_t irqc;                                      /* 0x100 IRQ Control Register*/
    uint32_t reg5;                                      /* 0x140 Baud Rate Control */
};
typedef volatile struct meson_uart_regs meson_uart_regs_t;

/* AML_UART_CONTROL bits */
#define AML_UART_OLD_BAUD           (0xfff)             /* Old baud rate. */
#define AML_UART_TX_EN              BIT(12)             /* Transmit enable bit. */
#define AML_UART_RX_EN              BIT(13)             /* Receive enable bit. */
#define AML_UART_TWO_WIRE_EN        BIT(15)             /* Two wire mode. */
#define AML_UART_STOP_BIT_LEN_MASK  (0x03 << 16)        /* Stop bit length mask. */
#define AML_UART_STOP_BIT_1SB       (0x00 << 16)
#define AML_UART_STOP_BIT_2SB       (0x01 << 16)
#define AML_UART_PARITY_TYPE        BIT(18)             /* Parity type used to detect data change during transmission. even = 0, odd = 1. */
#define AML_UART_PARITY_EN          BIT(19)             /* Parity enable. Set to 1 to enable parity. */
#define AML_UART_DATA_LEN_MASK      (0x03 << 20)        /* Character length. */
#define AML_UART_DATA_LEN_8BIT      (0x00 << 20)
#define AML_UART_DATA_LEN_7BIT      (0x01 << 20)
#define AML_UART_DATA_LEN_6BIT      (0x02 << 20)
#define AML_UART_DATA_LEN_5BIT      (0x03 << 20)
#define AML_UART_TX_RST             BIT(22)             /* Reset transmission state machine. */
#define AML_UART_RX_RST             BIT(23)             /* Reset receive state machine. */
#define AML_UART_CLEAR_ERR          BIT(24)             /* Clear error. */
#define AML_UART_RX_INT_EN          BIT(27)             /* Rx byte interrupt. Set to 1 to enable interrupt generation when a byte is written to rx FIFO. */
#define AML_UART_TX_INT_EN          BIT(28)             /* Tx byte interrupt. Set to 1 to enable interrupt generation when a byte is read from tx FIFO. */
#define AML_UART_MASK_ERR           BIT(30)             /* Set to 1 to mask errors. */

/* AML_UART_STATUS bits */
#define AML_UART_RX_FIFO_CNT        (0x7f)              /* rx FIFO count. Number of bytes in the rx FIFO */
#define AML_UART_TX_FIFO_CNT        (0x7f << 8)         /* tx FIFO count. Number of bytes in the tx FIFO */
#define AML_UART_PARITY_ERR         BIT(16)             /* Parity error. Clear by writing bit 24 to control register. */
#define AML_UART_FRAME_ERR          BIT(17)             /* Frame error. Clear by writing bit 24 to control register. */
#define AML_UART_TX_FIFO_WERR       BIT(18)             /* This bit is set if the FIFO is written to when full. */
#define AML_UART_RX_FULL            BIT(19)             /* rx FIFO full. */
#define AML_UART_RX_EMPTY           BIT(20)             /* rx FIFO empty. */
#define AML_UART_TX_FULL            BIT(21)             /* tx FIFO full. */
#define AML_UART_TX_EMPTY           BIT(22)             /* tx FIFO empty. */
#define AML_UART_RX_OVRFLW          BIT(24)             /* rx FIFO overflow. */
#define AML_UART_TX_BUSY            BIT(25)             /* tx state machine is busy. */
#define AML_UART_RX_BUSY            BIT(26)             /* rx state machine is busy. */

#define UART_INTR_ABNORMAL          (AML_UART_PARITY_ERR | AML_UART_FRAME_ERR  | AML_UART_TX_FIFO_WERR)

/* AML_UART_IRQ bits */
#define AML_UART_RECV_IRQ_MASK      0xff
#define AML_UART_RECV_IRQ(c)        ((c) & 0xff)        /* Generate an interrupt after a certain number of bytes have been received by the UART. */
#define AML_UART_XMIT_IRQ_MASK      (0xff << 8)
#define AML_UART_XMIT_IRQ(c)        (((c) & 0xff) << 8) /* Generate an interrupt if the number of bytes in the transmit FIFO drops below this value. */

/* AML_UART_REG5 bits */
#define AML_UART_BAUD_MASK          0x7fffff
#define AML_UART_BAUD_USE           BIT(23)             /* Use the first 23 bits to calculate the baud rate. */
#define AML_UART_BAUD_XTAL          BIT(24)             /* If this bit is set, then the clock for generating the UART Baud rate comes from the crystal pad. This allows the UART to operate independent of clk81. */
#define AML_UART_BAUD_XTAL_DIV3     BIT(26)             /* If this bit is set use 24MHz crystal clock rate. If unset, divide clock rate by 3 (8MHz). */
#define AML_UART_BAUD_XTAL_DIV2     BIT(27)             /* If this bit is not set, see bit 26. If set, divide clock rate by 2 (12MHz). */

#define UART_XTAL_REF_CLK 24000000

struct uart_clock_state {
    bool crystal_clock;                                 /* True if crystal clock is in use. */
    uint64_t reference_clock_frequency;                 /* Frequency of the reference clock. Either crystal clock of system clock. */
    uint16_t crystal_clock_divider;                     /* Divider of the crystal clock if in use. */
    unsigned long baud;                                 /* Configured baud rate. */
    uint32_t reference_ticks_per_symbol;                /* Baud rate in terms of divided reference clock ticks per symbol. */
};