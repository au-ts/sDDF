/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/util/printf.h>
#include <serial_config.h>
#include <uart.h>

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
volatile pl011_uart_regs_t *uart_regs;

/*
 * BaudDivInt + BaudDivFrac/64 = (RefFreq/ (16 x BaudRate))
 */
static void set_baud(long bps)
{
    float baud_div = PL011_UART_REF_CLOCK / (16 * bps);
    uint32_t baud_div_int = (uint32_t)baud_div;
    uint32_t baud_div_frac = (uint32_t)((baud_div * 64) + 0.5);

    /* Minimum divide ratio possible is 1 */
    assert(baud_div_int >= 1);

    /* Maximum divide ratio is 0xFFFF */
    assert(baud_div_int < 0xFFFF || (baud_div_int == 0xFFFF && baud_div_frac == 0));

    uart_regs->ibrd = baud_div_int;
    uart_regs->fbrd = baud_div_frac;
}

static void tx_provide(void)
{
    bool reprocess = true;
    bool transferred = false;
    while (reprocess) {
        char c;
        while (!(uart_regs->fr & PL011_FR_TXFF) && !serial_dequeue(&tx_queue_handle, &tx_queue_handle.queue->head, &c)) {
            uart_regs->dr = (uint32_t)c;
            transferred = true;
        }

        serial_request_producer_signal(&tx_queue_handle);
        /* If transmit fifo is full and there is data remaining to be sent, enable interrupt when fifo is no longer full */
        if (uart_regs->fr & PL011_FR_TXFF && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            uart_regs->imsc |= PL011_IMSC_TX_INT;
        } else {
            uart_regs->imsc &= ~PL011_IMSC_TX_INT;
        }
        reprocess = false;

        if (!(uart_regs->fr & PL011_FR_TXFF) && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            serial_cancel_producer_signal(&tx_queue_handle);
            uart_regs->imsc &= ~PL011_IMSC_TX_INT;
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
        while (!(uart_regs->fr & PL011_FR_RXFE) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            char c = (char)(uart_regs->dr & PL011_DR_DATA_MASK);
            serial_enqueue(&rx_queue_handle, &rx_queue_handle.queue->tail, c);
            enqueued = true;
        }

        if (!(uart_regs->fr & PL011_FR_RXFE) && serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            /* Disable rx interrupts until virtualisers queue is no longer empty. */
            uart_regs->imsc &= ~(PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT);
            serial_request_consumer_signal(&rx_queue_handle);
        }
        reprocess = false;

        if (!(uart_regs->fr & PL011_FR_RXFE) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            serial_cancel_consumer_signal(&rx_queue_handle);
            uart_regs->imsc |= (PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT);
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
    while (uart_regs->mis & (PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT) || uart_regs->mis & PL011_IMSC_TX_INT) {
        if (uart_regs->mis & (PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT)) {
            rx_return();
        }
        if (uart_regs->mis & PL011_IMSC_TX_INT) {
            tx_provide();
        }
    }
}

static void uart_setup(void)
{
    uart_regs = (pl011_uart_regs_t *) uart_base;

    /* Wait for UART to finish transmitting. */
    while (uart_regs->fr & PL011_FR_UART_BUSY);

    /* Disable the UART - UART must be disabled before control registers are reprogrammed. */
    uart_regs->tcr &= ~(PL011_CR_RX_EN | PL011_CR_TX_EN | PL011_CR_UART_EN);

    /* Configure stop bit length to 1 */
    uart_regs->lcr_h &= ~(PL011_LCR_2_STP_BITS);

    /* Set data length to 8 */
    uart_regs->lcr_h |= (0b11 < PL011_LCR_WLEN_SHFT);

    /* Configure the reference clock and baud rate. Difficult to use automatic detection here as it requires the next incoming character to be 'a' or 'A'. */
    set_baud(UART_DEFAULT_BAUD);

    /* Enable FIFOs */
    uart_regs->lcr_h |= PL011_LCR_FIFO_EN;

    /* Disable parity checking */
    uart_regs->lcr_h |= PL011_LCR_PARTY_EN;

    /* Enable receive interrupts when FIFO level exceeds 1/8 or after 32 ticks */
#if !SERIAL_TX_ONLY
    uart_regs->ifls &= ~(PL011_IFLS_RX_MASK << PL011_IFLS_RX_SHFT);
    uart_regs->imsc |= (PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT);
#endif

    /* Enable transmit interrupts if the FIFO drops below 1/8 - used when the write fifo becomes full */
    uart_regs->ifls &= ~(PL011_IFLS_TX_MASK << PL011_IFLS_TX_SHFT);
    uart_regs->imsc |= PL011_IMSC_TX_INT;

    /* Enable the UART */
    uart_regs->tcr |= PL011_CR_UART_EN;

    /* Enable transmit */
    uart_regs->tcr |= PL011_CR_TX_EN;

    /* Enable receive */
#if !SERIAL_TX_ONLY
    uart_regs->tcr |= PL011_CR_RX_EN;
#endif
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
        uart_regs->imsc |= (PL011_IMSC_RX_TIMEOUT | PL011_IMSC_RX_INT);
        rx_return();
        break;
    default:
        sddf_dprintf("UART|LOG: received notification on unexpected channel: %u\n", ch);
        break;
    }
}