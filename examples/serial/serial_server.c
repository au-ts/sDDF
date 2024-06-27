/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <ctype.h>
#include <microkit.h>
#include <sddf/serial/queue.h>
#include <sddf/util/printf.h>
#include <serial_config.h>

#define TX_CH 0
#define RX_CH 1

serial_queue_t *rx_queue;
serial_queue_t *tx_queue;

char *rx_data;
char *tx_data;

serial_queue_handle_t rx_queue_handle;
serial_queue_handle_t tx_queue_handle;

uint32_t local_head;

void init(void)
{
    serial_cli_queue_init_sys(microkit_name, &rx_queue_handle, rx_queue, rx_data, &tx_queue_handle, tx_queue, tx_data);
    serial_putchar_init(TX_CH, &tx_queue_handle);
    sddf_printf("Hello world! I am %s.\r\nPlease give me character!\r\n", microkit_name);
}

uint16_t char_count;
void notified(microkit_channel ch)
{
    bool reprocess = true;
    char c;
    while (reprocess) {
        while (!serial_dequeue(&rx_queue_handle, &rx_queue_handle.queue->head, &c)) {
            if (c == '\r') {
                sddf_putchar_unbuffered('\\');
                sddf_putchar_unbuffered('r');
            } else {
                sddf_putchar_unbuffered(c);
            }
            char_count ++;
            if (char_count % 10 == 0) {
                sddf_printf("\r\n%s has received %u characters so far!\r\n", microkit_name, char_count);
            }
        }

        serial_request_producer_signal(&rx_queue_handle);
        reprocess = false;

        if (!serial_queue_empty(&rx_queue_handle, rx_queue_handle.queue->head)) {
            serial_cancel_producer_signal(&rx_queue_handle);
            reprocess = true;
        }
    }
}
