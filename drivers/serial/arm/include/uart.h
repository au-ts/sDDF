/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <sddf/util/util.h>
#include <sddf/serial/queue.h>

/*
 * This UART driver is based on the following specificion:
 * ARM PrimeCell UART (PL011)
 * Revision: r1p5
 */

/* Register layout is based on section 3.2 of the specification. */
struct pl011_uart_regs {
    uint32_t dr;        /* 0x000 Data Register */
    uint32_t rsr_ecr;   /* 0x004 Receive Status Register/Error Clear Register */
    uint32_t res1;      /* 0x008 Reserved */
    uint32_t res2;      /* 0x00c Reserved */
    uint32_t res3;      /* 0x010 Reserved */
    uint32_t res4;      /* 0x014 Reserved */
    uint32_t fr;        /* 0x018 Flag Register */
    uint32_t res5;      /* 0x01c Reserved */
    uint32_t ilpr;      /* 0x020 IrDA Low-Power Counter Register */
    uint32_t ibrd;      /* 0x024 Integer Baud Rate Register */
    uint32_t fbrd;      /* 0x028 Fractional Baud Rate Register */
    uint32_t lcr_h;     /* 0x02c Line Control Register */
    uint32_t tcr;       /* 0x030 Control Register */
    uint32_t ifls;      /* 0x034 Interrupt FIFO Level Select Register */
    uint32_t imsc;      /* 0x038 Interrupt Mask Set/Clear Register */
    uint32_t ris;       /* 0x03C Raw Interrupt Status Register */
    uint32_t mis;       /* 0x040 Masked Interrupt Status Register */
    uint32_t icr;       /* 0x044 Interrupt Clear Register */
    uint32_t dmacr;     /* 0x048 DMA Control Register */
    /* The rest of the registers are either reserved or not used */
};
typedef volatile struct pl011_uart_regs pl011_uart_regs_t;

/* Data Register bits */
#define PL011_DR_DATA_MASK          0xFF            /* Read or write from/to these bits to rx or tx. */

/* Flag Register bits */
#define PL011_FR_TXFE               BIT(7)          /* Transmit FIFO empty. */
#define PL011_FR_RXFF               BIT(6)          /* Receive FIFO full. */
#define PL011_FR_TXFF               BIT(5)          /* Transmit FIFO full. */
#define PL011_FR_RXFE               BIT(4)          /* Receive FIFO empty. */
#define PL011_FR_UART_BUSY          BIT(3)          /* Uart busy transmitting data. */

/* Integer Baud Rate Register */
#define PL011_IBRD_BAUD_INT_MASK    0xFFFF          /* Integer part of baud rate divisor value. */

/* Fractional Baud Rate Register */
#define PL011_FBRD_BAUD_FRAC_MASK   0x3F            /* Fractional part of baud rate divisor value. */

#define PL011_UART_REF_CLOCK        0x16E3600

/* Line Control Register bits */
#define PL011_LCR_WLEN_MASK         0x3             /* Word length. b00 = 5, b01 = 6, b10 = 7, b11 = 8. */
#define PL011_LCR_WLEN_SHFT         5
#define PL011_LCR_FIFO_EN           BIT(4)          /* Enable tx and rx FIFOs. */
#define PL011_LCR_2_STP_BITS        BIT(3)          /* Set this bit to 1 to tx two stop bits. */
#define PL011_LCR_PARTY_EVEN        BIT(2)          /* Even parity select. */
#define PL011_LCR_PARTY_EN          BIT(1)          /* Enable parity checks and addition. */

/* Control Register */
#define PL011_CR_RX_EN              BIT(9)          /* Enable rx. */
#define PL011_CR_TX_EN              BIT(8)          /* Enable tx. */
#define PL011_CR_UART_EN            BIT(0)          /* Enable the uart. */

/* Interrupt FIFO Level Select Register */
#define PL011_IFLS_RX_MASK          0x7             /* Rx interrupt level select. b000 = 1/8, b001 = 1/4, b010 = 1/2, b011 = 3/4, b100 = 7/8. */
#define PL011_IFLS_RX_SHFT          3
#define PL011_IFLS_TX_MASK          0x7             /* Tx interrupt level select. b000 = 1/8, b001 = 1/4, b010 = 1/2, b011 = 3/4, b100 = 7/8. */
#define PL011_IFLS_TX_SHFT          0

/* Interrupt Mask Set/Clear Register */
#define PL011_IMSC_RX_TIMEOUT       BIT(6)          /* Enable rx timeout interrupt. Occurs when the rx FIFO is not empty, and no more data is received during a 32-bit period. */
#define PL011_IMSC_TX_INT           BIT(5)          /* Enable tx interrupt when FIFO drops below programmed threshold. */
#define PL011_IMSC_RX_INT           BIT(4)          /* Enable rx interrupt when FIFO exceeds programmed threshold. */

/* Masked Interrupt Status Register */
#define PL011_IMSC_RX_TIMEOUT       BIT(6)          /* Rx timeout interrupt. Occurs when the rx FIFO is not empty, and no more data is received during a 32-bit period. */
#define PL011_IMSC_TX_INT           BIT(5)          /* Tx interrupt when FIFO drops below programmed threshold. */
#define PL011_IMSC_RX_INT           BIT(4)          /* Rx interrupt when FIFO exceeds programmed threshold. */