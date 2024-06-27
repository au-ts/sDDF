/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <microkit.h>
#include <sddf/serial/queue.h>
#include <sddf/util/printf.h>
#include <serial_config.h>

#define DRIVER_CH 0
#define CLIENT_OFFSET 1

serial_queue_t *tx_queue_drv;
uintptr_t tx_queue_cli0;

char *tx_data_drv;
uintptr_t tx_data_cli0;

#if SERIAL_WITH_COLOUR

#define MAX_COLOURS 256
#define MAX_COLOURS_LEN 3

#define COLOUR_START_START "\x1b[38;5;"
#define COLOUR_START_START_LEN 7

#define COLOUR_START_END "m"
#define COLOUR_START_END_LEN 1

#define COLOUR_END "\x1b[0m"

char *clients_colours[SERIAL_NUM_CLIENTS];

#endif

serial_queue_handle_t tx_queue_handle_drv;
serial_queue_handle_t tx_queue_handle_cli[SERIAL_NUM_CLIENTS];

typedef struct tx_pending {
    uint32_t queue[SERIAL_NUM_CLIENTS];
    bool clients_pending[SERIAL_NUM_CLIENTS];
    uint32_t head;
    uint32_t tail;
} tx_pending_t;

tx_pending_t tx_pending;

static uint32_t tx_pending_length(void)
{
    return tx_pending.tail - tx_pending.head;
}

static void tx_pending_push(uint32_t client)
{
    /* Ensure client is not already pending */
    if (tx_pending.clients_pending[client]) {
        return;
    }

    /* Ensure the pending queue is not already full */
    assert(tx_pending_length() < SERIAL_NUM_CLIENTS);

    tx_pending.queue[tx_pending.tail] = client;
    tx_pending.clients_pending[client] = true;
    tx_pending.tail++;
}

static void tx_pending_pop(uint32_t *client)
{
    /* This should only be called when length > 0 */
    assert(tx_pending_length());

    *client = tx_pending.queue[tx_pending.head];
    tx_pending.clients_pending[*client] = false;
    tx_pending.head++;
}

bool process_tx_queue(uint32_t client)
{
    serial_queue_handle_t *handle = &tx_queue_handle_cli[client];

    if (serial_queue_empty(handle, handle->queue->head)) {
        serial_request_producer_signal(handle);
        return false;
    }

    uint32_t length = serial_queue_length(handle);
#if SERIAL_WITH_COLOUR
    length += COLOUR_START_START_LEN + MAX_COLOURS_LEN + COLOUR_START_END_LEN;
#endif

    /* Not enough space to transmit string to virtualiser. Continue later */
    if (length > serial_queue_free(&tx_queue_handle_drv)) {
        tx_pending_push(client);

        /* Request signal from the driver when data has been consumed */
        serial_request_consumer_signal(&tx_queue_handle_drv);

        /* Cancel further signals from this client */
        serial_cancel_producer_signal(handle);
        return false;
    }

#if SERIAL_WITH_COLOUR
    char colour_start_buff[COLOUR_START_START_LEN + MAX_COLOURS_LEN + COLOUR_START_END_LEN + 1];
    sddf_sprintf(colour_start_buff, "%s%u%s", COLOUR_START_START, client % MAX_COLOURS, COLOUR_START_END);
    serial_transfer_all_with_colour(handle, &tx_queue_handle_drv, colour_start_buff, COLOUR_END);
#else
    serial_transfer_all(handle, &tx_queue_handle_drv);
#endif
    serial_request_producer_signal(handle);
    return true;
}

void tx_return(void)
{
    uint32_t num_pending_tx = tx_pending_length();
    if (!num_pending_tx) {
        return;
    }

    uint32_t client;
    bool transferred = false;
    for (uint32_t req = 0; req < num_pending_tx; req++) {
        tx_pending_pop(&client);
        bool reprocess = true;
        bool client_transferred = false;
        while (reprocess) {
            client_transferred |= process_tx_queue(client);
            reprocess = false;

            /* If more data is available, re-process unless it has been pushed to pending transmits */
            if (!serial_queue_empty(&tx_queue_handle_cli[client], tx_queue_handle_cli[client].queue->head)
                && !tx_pending.clients_pending[client]) {
                serial_cancel_producer_signal(&tx_queue_handle_cli[client]);
                reprocess = true;
            }
        }
        transferred |= client_transferred;
    }

    if (transferred && serial_require_producer_signal(&tx_queue_handle_drv)) {
        serial_cancel_producer_signal(&tx_queue_handle_drv);
        microkit_notify_delayed(DRIVER_CH);
    }
}

void tx_provide(microkit_channel ch)
{
    if (ch > SERIAL_NUM_CLIENTS) {
        sddf_dprintf("VIRT_TX|LOG: Received notification from unkown channel %u\n", ch);
        return;
    }

    uint32_t active_client = ch - CLIENT_OFFSET;
    bool transferred = false;
    bool reprocess = true;
    while (reprocess) {
        transferred |= process_tx_queue(active_client);
        reprocess = false;

        /* If more data is available, re-process unless it has been pushed to pending transmits */
        if (!serial_queue_empty(&tx_queue_handle_cli[active_client], tx_queue_handle_cli[active_client].queue->head)
            && !tx_pending.clients_pending[active_client]) {
            serial_cancel_producer_signal(&tx_queue_handle_cli[active_client]);
            reprocess = true;
        }
    }

    if (transferred && serial_require_producer_signal(&tx_queue_handle_drv)) {
        serial_cancel_producer_signal(&tx_queue_handle_drv);
        microkit_notify_delayed(DRIVER_CH);
    }
}

void init(void)
{
    serial_queue_init(&tx_queue_handle_drv, tx_queue_drv, SERIAL_TX_DATA_REGION_SIZE_DRIV, tx_data_drv);
    serial_virt_queue_init_sys(microkit_name, tx_queue_handle_cli, tx_queue_cli0, tx_data_cli0);

#if !SERIAL_TX_ONLY
    /* Print a deterministic string to allow console input to begin */
    sddf_memcpy(tx_queue_handle_drv.data_region, SERIAL_CONSOLE_BEGIN_STRING, SERIAL_CONSOLE_BEGIN_STRING_LEN);
    serial_update_visible_tail(&tx_queue_handle_drv, SERIAL_CONSOLE_BEGIN_STRING_LEN);
    microkit_notify(DRIVER_CH);
#endif

#if SERIAL_WITH_COLOUR
    serial_channel_names_init(clients_colours);
    for (uint32_t i = 0; i < SERIAL_NUM_CLIENTS; i++) {
        sddf_dprintf("%s%u%s%s is client %u%s\n", COLOUR_START_START, i % MAX_COLOURS, COLOUR_START_END, clients_colours[i], i,
                     COLOUR_END);
    }
#endif
}

void notified(microkit_channel ch)
{
    switch (ch) {
    case DRIVER_CH:
        tx_return();
        break;
    default:
        tx_provide(ch);
        break;
    }
}
