/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <microkit.h>
#include <sddf/serial/queue.h>
#include <sddf/util/string.h>
#include <sddf/util/printf.h>
#include <serial_config.h>
#include "uart.h"

#define DRIVER_CH 0
#define CLIENT_OFFSET 1

serial_queue_t *rx_queue_drv;
uintptr_t rx_queue_cli0;

char *rx_data_drv;
uintptr_t rx_data_cli0;

serial_queue_handle_t rx_queue_handle_drv;
serial_queue_handle_t rx_queue_handle_cli[SERIAL_NUM_CLIENTS];

#define MAX_CLI_BASE_10 4
typedef enum mode {normal, switched, number} mode_t;

mode_t current_mode = normal;
uint32_t current_client = 0;
char next_client[MAX_CLI_BASE_10 + 1];
uint8_t next_client_index;

void reset_state(void)
{
    sddf_memset(next_client, '\0', MAX_CLI_BASE_10 + 1);
    next_client_index = 0;
    current_mode = normal;
}

void rx_return(void)
{
    bool reprocess = true;
    bool transferred = false;
    uint32_t local_tail = rx_queue_handle_cli[current_client].queue->tail;
    char c = '\0';
    while (reprocess) {
        while (!serial_dequeue(&rx_queue_handle_drv, &rx_queue_handle_drv.queue->head, &c)) {
            switch (current_mode) {
            case normal:
                switch (c) {
                case SERIAL_SWITCH_CHAR:
                    current_mode = switched;
                    break;
                default:
                    if (!serial_enqueue(&rx_queue_handle_cli[current_client], &local_tail, c)) {
                        transferred = true;
                    }
                    break;
                }
                break;
            case switched:
                if (sddf_isdigit(c)) {
                    next_client[next_client_index] = c;
                    next_client_index ++;
                    current_mode = number;
                } else {
                    if (c == SERIAL_SWITCH_CHAR) {
                        if (!serial_enqueue(&rx_queue_handle_cli[current_client], &local_tail, c)) {
                            transferred = true;
                        }
                    } else {
                        sddf_dprintf("VIRT_RX|LOG: User entered an invalid digit %c\n", c);
                    }
                    reset_state();
                }
                break;
            default:
                if (c == SERIAL_TERMINATE_NUM) {
                    int input_number = sddf_atoi(next_client);
                    if (input_number >= 0 && input_number < SERIAL_NUM_CLIENTS) {
                        if (transferred && serial_require_producer_signal(&rx_queue_handle_cli[current_client])) {
                            serial_update_visible_tail(&rx_queue_handle_cli[current_client], local_tail);
                            serial_cancel_producer_signal(&rx_queue_handle_cli[current_client]);
                            microkit_notify(current_client + CLIENT_OFFSET);
                        }
                        current_client = (uint32_t)input_number;
                        local_tail = rx_queue_handle_cli[current_client].queue->tail;
                        transferred = false;
                        sddf_dprintf("VIRT_RX|LOG: switching to client %d\n", input_number);
                    } else {
                        sddf_dprintf("VIRT_RX|LOG: User requested to switch to an invalid client %d\n", input_number);
                    }
                    reset_state();
                } else if (next_client_index < MAX_CLI_BASE_10 && sddf_isdigit((int)c)) {
                    next_client[next_client_index] = c;
                    next_client_index ++;
                } else {
                    sddf_dprintf("VIRT_RX|LOG: User entered too many (%u < %u) or invalid digit (%c)\n", next_client_index + 1,
                                 MAX_CLI_BASE_10, c);
                    reset_state();
                }
                break;
            }
        }

        serial_update_visible_tail(&rx_queue_handle_cli[current_client], local_tail);
        serial_request_producer_signal(&rx_queue_handle_drv);
        reprocess = false;

        if (!serial_queue_empty(&rx_queue_handle_drv, rx_queue_handle_drv.queue->head)) {
            serial_cancel_producer_signal(&rx_queue_handle_drv);
            reprocess = true;
        }
    }

    if (!serial_queue_full(&rx_queue_handle_drv, rx_queue_handle_drv.queue->tail)
        && serial_require_consumer_signal(&rx_queue_handle_drv)) {
        serial_cancel_consumer_signal(&rx_queue_handle_drv);
        microkit_notify(DRIVER_CH);
    }

    if (transferred && serial_require_producer_signal(&rx_queue_handle_cli[current_client])) {
        serial_cancel_producer_signal(&rx_queue_handle_cli[current_client]);
        microkit_notify(current_client + CLIENT_OFFSET);
    }
}

void init(void)
{
    serial_queue_init(&rx_queue_handle_drv, rx_queue_drv, SERIAL_RX_DATA_REGION_SIZE_DRIV, rx_data_drv);
    serial_virt_queue_init_sys(microkit_name, rx_queue_handle_cli, rx_queue_cli0, rx_data_cli0);
}

void notified(microkit_channel ch)
{
    switch (ch) {
    case DRIVER_CH:
        rx_return();
        break;
    default:
        sddf_dprintf("VIRT_RX|LOG: received notification on unexpected channel: %u\n", ch);
        break;
    }
}