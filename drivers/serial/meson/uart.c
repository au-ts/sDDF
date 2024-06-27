/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/util/printf.h>
#include <serial_config.h>
#include "uart.h"

#define IRQ_CH 0
#define TX_CH  1
#define RX_CH  2

serial_queue_t *rx_queue;
serial_queue_t *tx_queue;

char *rx_data;
char *tx_data;

serial_queue_handle_t rx_queue_handle;
serial_queue_handle_t tx_queue_handle;

uintptr_t uart_base;
volatile meson_uart_regs_t *uart_regs;
struct uart_clock_state uart_clock;

/* UART baud register expects baud rate to be expressed in terms of the number of reference
 ticks per symbol change. This function calculates these ticks and modifies the divisor of
 the reference clock accordingly. */
static void set_baud(unsigned long baud)
{
    /* Baud rate must be positive. */
    assert(baud > 0);

    uint64_t ref_clock_freq = uart_clock.reference_clock_frequency;
    uint64_t ref_clock_ticks_per_symbol = ref_clock_freq / baud;

    /* Check if baud rate can be acheived with a less frequent clock tick.
        Note: Linux defaults to use xtal div 3 if the board doesn't implement xtal_div2.
        They hardcode what boards support xtal_div2. This IS implemented on the odroidc4,
        but this may not work for different meson devices. */
    uint16_t clock_div = 1;
    uint32_t baud_register = AML_UART_BAUD_USE;
    if (uart_clock.crystal_clock) {
        baud_register |= AML_UART_BAUD_XTAL;
        if (ref_clock_ticks_per_symbol % 3 == 0) {
            clock_div = 3;
            ref_clock_ticks_per_symbol /= 3;
        } else if (ref_clock_ticks_per_symbol % 2 == 0) {
            clock_div = 2;
            ref_clock_ticks_per_symbol /= 2;
            baud_register |= AML_UART_BAUD_XTAL_DIV2;
        } else {
            baud_register |= AML_UART_BAUD_XTAL_DIV3;
        }
    }

    /* UART does not support baud rates this slow. */
    assert((ref_clock_ticks_per_symbol & ~AML_UART_BAUD_MASK) == 0);

    if (uart_clock.crystal_clock) {
        uart_clock.crystal_clock_divider = clock_div;
    }
    uart_clock.baud = baud;
    uart_clock.reference_ticks_per_symbol = ref_clock_ticks_per_symbol;
    baud_register |= ref_clock_ticks_per_symbol;
    uart_regs->reg5 = baud_register;
}

static void tx_provide(void)
{
    bool reprocess = true;
    bool transferred = false;
    while (reprocess) {
        char c;
        while (!(uart_regs->sr & AML_UART_TX_FULL) && !serial_dequeue(&tx_queue_handle, &tx_queue_handle.queue->head, &c)) {
            uart_regs->wfifo = (uint32_t)c;
            transferred = true;
        }

        serial_request_producer_signal(&tx_queue_handle);
        /* If transmit fifo is full and there is data remaining to be sent, enable interrupt when fifo is no longer full */
        if (uart_regs->sr & AML_UART_TX_FULL && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            uart_regs->cr |= AML_UART_TX_INT_EN;
        } else {
            uart_regs->cr &= ~AML_UART_TX_INT_EN;
        }
        reprocess = false;

        if (!(uart_regs->sr & AML_UART_TX_FULL) && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            serial_cancel_producer_signal(&tx_queue_handle);
            uart_regs->cr &= ~AML_UART_TX_INT_EN;
            reprocess = true;
        }
    }

    if (transferred && serial_require_consumer_signal(&tx_queue_handle)) {
        serial_cancel_consumer_signal(&tx_queue_handle);
        microkit_notify(TX_CH);
    }
}

static void rx_return(void)
{
    bool reprocess = true;
    bool enqueued = false;
    while (reprocess) {
        while (!(uart_regs->sr & AML_UART_RX_EMPTY) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            char c = (char) uart_regs->rfifo;
            serial_enqueue(&rx_queue_handle, &rx_queue_handle.queue->tail, c);
            enqueued = true;
        }

        if (!(uart_regs->sr & AML_UART_RX_EMPTY) && serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            /* Disable rx interrupts until virtualisers queue is no longer empty. */
            uart_regs->cr &= ~AML_UART_RX_INT_EN;
            serial_request_consumer_signal(&rx_queue_handle);
        }
        reprocess = false;

        if (!(uart_regs->sr & AML_UART_RX_EMPTY) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            serial_cancel_consumer_signal(&rx_queue_handle);
            uart_regs->cr |= AML_UART_RX_INT_EN;
            reprocess = true;
        }
    }

    if (enqueued && serial_require_producer_signal(&rx_queue_handle)) {
        serial_cancel_producer_signal(&rx_queue_handle);
        microkit_notify(RX_CH);
    }
}

static void handle_irq(void)
{
    while (uart_regs->sr & UART_INTR_ABNORMAL || !(uart_regs->sr & AML_UART_RX_EMPTY)
           || (uart_regs->cr & AML_UART_TX_INT_EN && !(uart_regs->sr & AML_UART_TX_FULL))) {
        if (!(uart_regs->sr & AML_UART_RX_EMPTY)) {
            rx_return();
        }
        if (uart_regs->cr & AML_UART_TX_INT_EN && !(uart_regs->sr & AML_UART_TX_FULL)) {
            tx_provide();
        }
        if (uart_regs->sr & UART_INTR_ABNORMAL) {
            sddf_dprintf("UART|ERROR: Uart device encountered an error with status register %u\n", uart_regs->sr);
            uart_regs->cr |= AML_UART_CLEAR_ERR;
        }
    }
}

static void uart_setup(void)
{
    uart_regs = (meson_uart_regs_t *)(uart_base + UART_REGS_OFFSET);

    /* Wait until receive and transmit state machines are no longer busy */
    while (uart_regs->sr & (AML_UART_TX_BUSY | AML_UART_RX_BUSY));

    /* Disable transmit and receive */
    uart_regs->cr &= ~(AML_UART_TX_EN | AML_UART_RX_EN);

    /* Reset UART state machine */
    uart_regs->cr |= (AML_UART_TX_RST | AML_UART_RX_RST | AML_UART_CLEAR_ERR);
    uart_regs->cr &= ~(AML_UART_TX_RST | AML_UART_RX_RST | AML_UART_CLEAR_ERR);

    uint32_t cr = uart_regs->cr;
    /* Configure stop bit length to 1 */
    cr &= ~(AML_UART_STOP_BIT_LEN_MASK);
    cr |= AML_UART_STOP_BIT_1SB;

    /* Set data length to 8 */
    cr &= ~AML_UART_DATA_LEN_MASK;
    cr |= AML_UART_DATA_LEN_8BIT;

    /* Configure the reference clock and baud rate */
    uart_clock = (struct uart_clock_state) {
        true, UART_XTAL_REF_CLK, 1, 0, 0
    };
    set_baud(UART_DEFAULT_BAUD);

    uint32_t irqc = uart_regs->irqc;
    /* Enable receive interrupts every byte */
#if !SERIAL_TX_ONLY
    irqc &= ~AML_UART_RECV_IRQ_MASK;
    irqc |= AML_UART_RECV_IRQ(1);
    cr |= (AML_UART_RX_INT_EN | AML_UART_RX_EN);
#endif

    /* Enable transmit interrupts if the write fifo drops below one byte - used when the write fifo becomes full */
    irqc &= ~AML_UART_XMIT_IRQ_MASK;
    irqc |= AML_UART_XMIT_IRQ(1);
    cr |= AML_UART_TX_EN;

    uart_regs->irqc = irqc;
    uart_regs->cr = cr;
}

void init(void)
{
    uart_setup();

    serial_queue_init(&rx_queue_handle, rx_queue, SERIAL_RX_DATA_REGION_SIZE_DRIV, rx_data);
    serial_queue_init(&tx_queue_handle, tx_queue, SERIAL_TX_DATA_REGION_SIZE_DRIV, tx_data);
}

void notified(microkit_channel ch)
{
    switch (ch) {
    case IRQ_CH:
        handle_irq();
        microkit_irq_ack_delayed(ch);
        break;
    case TX_CH:
        tx_provide();
        break;
    case RX_CH:
        uart_regs->cr |= AML_UART_RX_INT_EN;
        rx_return();
        break;
    default:
        sddf_dprintf("UART|LOG: received notification on unexpected channel: %u\n", ch);
        break;
    }
}