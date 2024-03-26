#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <microkit.h>
#include <sddf/network/queue.h>
#include <sddf/network/constants.h>
#include <sddf/util/fence.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>
#include <sddf/util/cache.h>
#include <ethernet_config.h>

/* Notification channels */
#define DRIVER_CH 0
#define CLIENT_CH 1

/* Queue regions */
uintptr_t rx_free_drv;
uintptr_t rx_active_drv;
uintptr_t rx_free_arp;
uintptr_t rx_active_arp;
uintptr_t rx_free_cli0;
uintptr_t rx_active_cli0;
uintptr_t rx_free_cli1;
uintptr_t rx_active_cli1;

/* Buffer data regions */
uintptr_t buffer_data_vaddr;
uintptr_t buffer_data_paddr;

typedef struct state {
    net_queue_handle_t rx_queue_drv;
    net_queue_handle_t rx_queue_clients[NUM_CLIENTS];
    uint8_t mac_addrs[NUM_CLIENTS][ETH_HWADDR_LEN];
} state_t;

state_t state;

/* Boolean to indicate whether a packet has been enqueued into the driver's free queue during notification handling */
static bool notify_drv;

/* Return the client ID if the Mac address is a match. */
int get_client(struct ethernet_header * buffer)
{
    for (int client = 0; client < NUM_CLIENTS; client++) {
        bool match = true;
        for (int i = 0; (i < ETH_HWADDR_LEN) && match; i++) if (buffer->dest.addr[i] != state.mac_addrs[client][i]) match = false;
        if (match) return client;
    }
    return -1;
}

void rx_return(void)
{
    bool reprocess = true;
    bool notify_clients[NUM_CLIENTS] = {false};
    while (reprocess) {
        while (!net_queue_empty_active(&state.rx_queue_drv)) {
            net_buff_desc_t buffer;
            int err = net_dequeue_active(&state.rx_queue_drv, &buffer);
            assert(!err);

            buffer.io_or_offset = buffer.io_or_offset - buffer_data_paddr;
            uintptr_t buffer_vaddr = buffer.io_or_offset + buffer_data_vaddr;

            // Cache invalidate after DMA write, so we don't read stale data.
            // This is needed even if we invalidate before the DMA write as it
            // could have been speculatively fetched before the DMA write.
            //
            // In terms of how we actually do the invalidate, there are a couple
            // of considerations:
            //
            // 1. Compared to the seL4_ARM_VSpace_CleanInvalidate_Data syscalls,
            //    the cache_* functions do it entirely in userspace via `dc *`
            //    instructions, avoiding the cost of a syscall.
            //
            // 2. On ARM, you cannot only invalidate without cleaning if you're
            //    at EL0 (i.e. userspace) -- see [1]. Additionally, on at least
            //    some ARM chips, just invalidating also causes a clean to
            //    happen if needde -- see [2].
            //
            // Out of the available options, cache_clean_and_invalidate was the
            // fastest. (This explanation also referred to in rx_provide.)
            //
            // [1]: https://developer.arm.com/documentation/ddi0595/2021-06/AArch64-Instructions/DC-IVAC--Data-or-unified-Cache-line-Invalidate-by-VA-to-PoC
            // [2]: https://developer.arm.com/documentation/100236/0002/functional-description/cache-behavior-and-cache-protection/invalidating-or-cleaning-a-cache
            cache_clean_and_invalidate(buffer_vaddr, buffer_vaddr + ROUND_UP(buffer.len, 1 << CONFIG_L1_CACHE_LINE_SIZE_BITS));

            int client = get_client((struct ethernet_header *) buffer_vaddr);
            if (client >= 0) {
                err = net_enqueue_active(&state.rx_queue_clients[client], buffer);
                assert(!err);
                notify_clients[client] = true;
            } else {
                // We are returning buffers to the device for DMA, which
                // normally requires an invalidate (see rx_provide), but not
                // here since we know there aren't any writes to the buffer
                // since we invalidated above.

                buffer.io_or_offset = buffer.io_or_offset + buffer_data_paddr;
                err = net_enqueue_free(&state.rx_queue_drv, buffer);
                assert(!err);
                notify_drv = true;
            }
        }

        net_request_signal_active(&state.rx_queue_drv);
        reprocess = false;

        if (!net_queue_empty_active(&state.rx_queue_drv)) {
            net_cancel_signal_active(&state.rx_queue_drv);
            reprocess = true;
        }
    }

    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (notify_clients[client] && net_require_signal_active(&state.rx_queue_clients[client])) {
            net_cancel_signal_active(&state.rx_queue_clients[client]);
            microkit_notify(client + CLIENT_CH);
        }
    }    
}

void rx_provide(void)
{
    for (int client = 0; client < NUM_CLIENTS; client++) {
        bool reprocess = true;
        while (reprocess) {
            while (!net_queue_empty_free(&state.rx_queue_clients[client])) {
                net_buff_desc_t buffer;
                int err = net_dequeue_free(&state.rx_queue_clients[client], &buffer);
                assert(!err);
                assert(!(buffer.io_or_offset % NET_BUFFER_SIZE) && 
                       (buffer.io_or_offset < NET_BUFFER_SIZE * state.rx_queue_clients[client].size));

                // Cache invalidate before DMA write to discard dirty
                // cachelines, so they don't overwrite received data.
                //
                // We need to invalidate the whole buffer since we don't know
                // the packet length anymore, and also because the client may
                // have written past the packet anyways.
                //
                // See comment in rx_return for explanation of why we're using
                // cache_clean_and_invalidate.
                cache_clean_and_invalidate(buffer.io_or_offset + buffer_data_vaddr, buffer.io_or_offset + buffer_data_vaddr + NET_BUFFER_SIZE);

                buffer.io_or_offset = buffer.io_or_offset + buffer_data_paddr;
                err = net_enqueue_free(&state.rx_queue_drv, buffer);
                assert(!err);
                notify_drv = true;
            }

            net_request_signal_free(&state.rx_queue_clients[client]);
            reprocess = false;

            if (!net_queue_empty_free(&state.rx_queue_clients[client])) {
                net_cancel_signal_free(&state.rx_queue_clients[client]);
                reprocess = true;
            }
        }
    }

    if (notify_drv && net_require_signal_free(&state.rx_queue_drv)) {
        net_cancel_signal_free(&state.rx_queue_drv);
        microkit_notify_delayed(DRIVER_CH);
    }
}

void notified(microkit_channel ch)
{
    rx_return();
    rx_provide();
}

void init(void)
{
    virt_mac_addr_init_sys(microkit_name, (uint8_t *) state.mac_addrs);

    net_queue_init(&state.rx_queue_drv, (net_queue_t *)rx_free_drv, (net_queue_t *)rx_active_drv, RX_QUEUE_SIZE_DRIV);
    virt_queue_init_sys(microkit_name, state.rx_queue_clients, rx_free_arp, rx_active_arp);
    net_buffers_init(&state.rx_queue_drv, buffer_data_paddr);

    if (net_require_signal_free(&state.rx_queue_drv)) {
        net_cancel_signal_free(&state.rx_queue_drv);
        microkit_notify_delayed(DRIVER_CH);
    }
}
