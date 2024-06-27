/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdbool.h>
#include <microkit.h>
#include <sddf/network/queue.h>
#include <sddf/util/fence.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>
#include <ethernet_config.h>

#define VIRT_RX_CH 0
#define CLIENT_CH 1

net_queue_handle_t rx_queue_virt;
net_queue_handle_t rx_queue_cli;

uintptr_t rx_free_virt;
uintptr_t rx_active_virt;
uintptr_t rx_free_cli;
uintptr_t rx_active_cli;

uintptr_t virt_buffer_data_region;
uintptr_t cli_buffer_data_region;

void rx_return(void)
{
    bool enqueued = false;
    bool reprocess = true;

    while (reprocess) {
        while (!net_queue_empty_active(&rx_queue_virt) && !net_queue_empty_free(&rx_queue_cli)) {
            net_buff_desc_t cli_buffer, virt_buffer = {0};
            int err = net_dequeue_free(&rx_queue_cli, &cli_buffer);
            assert(!err);

            if (cli_buffer.io_or_offset % NET_BUFFER_SIZE || cli_buffer.io_or_offset >= NET_BUFFER_SIZE * rx_queue_cli.size) {
                sddf_dprintf("COPY|LOG: Client provided offset %lx which is not buffer aligned or outside of buffer region\n",
                             cli_buffer.io_or_offset);
                continue;
            }

            err = net_dequeue_active(&rx_queue_virt, &virt_buffer);
            assert(!err);

            uintptr_t cli_addr = cli_buffer_data_region + cli_buffer.io_or_offset;
            uintptr_t virt_addr = virt_buffer_data_region + virt_buffer.io_or_offset;

            memcpy((void *)cli_addr, (void *)virt_addr, virt_buffer.len);
            cli_buffer.len = virt_buffer.len;
            virt_buffer.len = 0;

            err = net_enqueue_active(&rx_queue_cli, cli_buffer);
            assert(!err);

            err = net_enqueue_free(&rx_queue_virt, virt_buffer);
            assert(!err);

            enqueued = true;
        }

        net_request_signal_active(&rx_queue_virt);

        /* Only request signal from client if incoming packets from multiplexer are awaiting free buffers */
        if (!net_queue_empty_active(&rx_queue_virt)) {
            net_request_signal_free(&rx_queue_cli);
        } else {
            net_cancel_signal_free(&rx_queue_cli);
        }

        reprocess = false;

        if (!net_queue_empty_active(&rx_queue_virt) && !net_queue_empty_free(&rx_queue_cli)) {
            net_cancel_signal_active(&rx_queue_virt);
            net_cancel_signal_free(&rx_queue_cli);
            reprocess = true;
        }
    }

    if (enqueued && net_require_signal_active(&rx_queue_cli)) {
        net_cancel_signal_active(&rx_queue_cli);
        microkit_notify(CLIENT_CH);
    }

    if (enqueued && net_require_signal_free(&rx_queue_virt)) {
        net_cancel_signal_free(&rx_queue_virt);
        microkit_notify_delayed(VIRT_RX_CH);
    }
}

void notified(microkit_channel ch)
{
    rx_return();
}

void init(void)
{
    net_copy_queue_init_sys(microkit_name, &rx_queue_cli, rx_free_cli, rx_active_cli, &rx_queue_virt, rx_free_virt,
                            rx_active_virt);
    net_buffers_init(&rx_queue_cli, 0);
}
