/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "sddf/util/util.h"
#include <microkit.h>
#include <sddf/util/printf.h>
#include <stdbool.h>
#include <stdint.h>
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
volatile imx_uart_regs_t *uart_regs;

/*
 * BaudRate = RefFreq / (16 * (BMR + 1)/(BIR + 1) )
 * RefFreq = module clock after divider is applied.
 * Set BIR to 15 for integer division.
 * BMR and BIR are 16 bit.
 */
static void set_baud(long bps)
{
    uint32_t bmr, bir, fcr;
    fcr = uart_regs->fcr;
    fcr &= ~UART_FCR_REF_FRQ_DIV_MSK;
    fcr |= UART_FCR_REF_CLK_DIV_2;
    bir = 0xf;
    bmr = (UART_MOD_CLK / 2) / bps - 1;
    uart_regs->fcr = fcr;
    uart_regs->bir = bir;
    uart_regs->bmr = bmr;
}

static void tx_provide(void)
{
    bool reprocess = true;
    bool transferred = false;
    while (reprocess) {
        char c;
        while (!(uart_regs->ts & UART_TST_TX_FIFO_FULL)
               && !serial_dequeue(&tx_queue_handle, &tx_queue_handle.queue->head, &c)) {
            uart_regs->txd = (uint32_t)c;
            transferred = true;
        }

        serial_request_producer_signal(&tx_queue_handle);
        /* If transmit fifo is full and there is data remaining to be sent, enable interrupt when fifo is no longer full */
        if (uart_regs->ts & UART_TST_TX_FIFO_FULL && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            uart_regs->cr1 |= UART_CR1_TX_READY_INT;
        } else {
            uart_regs->cr1 &= ~UART_CR1_TX_READY_INT;
        }
        reprocess = false;

        if (!(uart_regs->ts & UART_TST_TX_FIFO_FULL) && !serial_queue_empty(&tx_queue_handle, tx_queue_handle.queue->head)) {
            serial_cancel_producer_signal(&tx_queue_handle);
            uart_regs->cr1 &= ~UART_CR1_TX_READY_INT;
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
        while (!(uart_regs->ts & UART_TST_RX_FIFO_EMPTY) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            char c = (char) uart_regs->rxd;
            serial_enqueue(&rx_queue_handle, &rx_queue_handle.queue->tail, c);
            enqueued = true;
        }

        if (!(uart_regs->ts & UART_TST_RX_FIFO_EMPTY) && serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            /* Disable rx interrupts until virtualisers queue is no longer empty. */
            uart_regs->cr1 &= ~UART_CR1_RX_READY_INT;
            serial_request_consumer_signal(&rx_queue_handle);
        }
        reprocess = false;

        if (!(uart_regs->ts & UART_TST_RX_FIFO_EMPTY) && !serial_queue_full(&rx_queue_handle, rx_queue_handle.queue->tail)) {
            serial_cancel_consumer_signal(&rx_queue_handle);
            uart_regs->cr1 |= UART_CR1_RX_READY_INT;
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
    while (uart_regs->sr1 & UART_SR1_ABNORMAL || uart_regs->sr1 & UART_SR1_RX_RDY
           || (uart_regs->cr1 & UART_CR1_TX_READY_INT && uart_regs->sr1 & UART_SR1_TX_RDY)) {
        if (uart_regs->sr1 & UART_SR1_RX_RDY) {
            rx_return();
        }
        if (uart_regs->cr1 & UART_CR1_TX_READY_INT && uart_regs->sr1 & UART_SR1_TX_RDY) {
            tx_provide();
        }
        if (uart_regs->sr1 & UART_SR1_ABNORMAL) {
            sddf_dprintf("UART|ERROR: Uart device encountered an error with status register %u\n", uart_regs->sr1);
            uart_regs->sr1 |= UART_SR1_ABNORMAL;
        }
    }
}

static void uart_setup(void)
{
    uart_regs = (imx_uart_regs_t *) uart_base;

    /* Enable the UART */
    uart_regs->cr1 |= UART_CR1_UART_EN;

    /* Enable transmit and receive */
    uart_regs->cr2 |= UART_CR2_TX_EN;
#if !SERIAL_TX_ONLY
    uart_regs->cr2 |= UART_CR2_RX_EN;
#endif

    /* Configure stop bit length to 1 */
    uart_regs->cr2 &= ~(UART_CR2_STOP_BITS);

    /* Set data length to 8 */
    uart_regs->cr2 |= UART_CR2_WORD_SZE;

    /* Configure the reference clock and baud rate. Difficult to use automatic detection here as it requires the next incoming character to be 'a' or 'A'. */
    set_baud(UART_DEFAULT_BAUD);

    /* Disable escape sequence, parity checking and aging rx data interrupts. */
    uart_regs->cr2 &= ~UART_CR2_PARITY_EN;
    uart_regs->cr2 &= ~(UART_CR2_ESCAPE_EN | UART_CR2_ESCAPE_INT);
    uart_regs->cr2 &= ~UART_CR2_AGE_EN;

    uint32_t fcr = uart_regs->fcr;
    /* Enable receive interrupts every byte */
#if !SERIAL_TX_ONLY
    fcr &= ~UART_FCR_RXTL_MASK;
    fcr |= (1 << UART_FCR_RXTL_SHFT);
#endif

    /* Enable transmit interrupts if the write fifo drops below one byte - used when the write fifo becomes full */
    fcr &= ~UART_FCR_TXTL_MASK;
    fcr |= (2 << UART_FCR_TXTL_SHFT);

    uart_regs->fcr = fcr;
#if !SERIAL_TX_ONLY
    uart_regs->cr1 |= UART_CR1_RX_READY_INT;
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
        uart_regs->cr1 |= UART_CR1_RX_READY_INT;
        rx_return();
        break;
    default:
        sddf_dprintf("UART|LOG: received notification on unexpected channel: %u\n", ch);
        break;
    }
}