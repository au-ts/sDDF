#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <microkit.h>
#include <sddf/network/queue.h>
#include <sddf/network/constants.h>
#include <sddf/util/fence.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>
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
        while (!net_queue_empty(state.rx_queue_drv.active)) {
            net_buff_desc_t buffer;
            int err = net_dequeue_active(&state.rx_queue_drv, &buffer);
            assert(!err);

            buffer.phys_or_offset = buffer.phys_or_offset - buffer_data_paddr;
            microkit_arm_vspace_data_invalidate(buffer.phys_or_offset + buffer_data_vaddr, buffer.phys_or_offset + buffer_data_vaddr + ROUND_UP(buffer.len, 64));

            int client = get_client((struct ethernet_header *) (buffer.phys_or_offset + buffer_data_vaddr));
            if (client >= 0) {
                err = net_enqueue_active(&state.rx_queue_clients[client], buffer);
                assert(!err);
                notify_clients[client] = true;
            } else {
                buffer.phys_or_offset = buffer.phys_or_offset + buffer_data_paddr;
                err = net_enqueue_free(&state.rx_queue_drv, buffer);
                assert(!err);
                notify_drv = true;
            }
        }

        net_request_signal(state.rx_queue_drv.active);
        reprocess = false;

        if (!net_queue_empty(state.rx_queue_drv.active)) {
            net_cancel_signal(state.rx_queue_drv.active);
            reprocess = true;
        }
    }

    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (notify_clients[client] && net_require_signal(state.rx_queue_clients[client].active)) {
            net_cancel_signal(state.rx_queue_clients[client].active);
            microkit_notify(client + CLIENT_CH);
        }
    }    
}

void rx_provide(void)
{
    for (int client = 0; client < NUM_CLIENTS; client++) {
        bool reprocess = true;
        while (reprocess) {
            while (!net_queue_empty(state.rx_queue_clients[client].free)) {
                net_buff_desc_t buffer;
                int err = net_dequeue_free(&state.rx_queue_clients[client], &buffer);
                assert(!err);
                assert(!(buffer.phys_or_offset % NET_BUFFER_SIZE) && 
                       (buffer.phys_or_offset < NET_BUFFER_SIZE * state.rx_queue_clients[client].free->size));

                buffer.phys_or_offset = buffer.phys_or_offset + buffer_data_paddr;
                err = net_enqueue_free(&state.rx_queue_drv, buffer);
                assert(!err);
                notify_drv = true;
            }

            net_request_signal(state.rx_queue_clients[client].free);
            reprocess = false;

            if (!net_queue_empty(state.rx_queue_clients[client].free)) {
                net_cancel_signal(state.rx_queue_clients[client].free);
                reprocess = true;
            }
        }
    }

    if (notify_drv && net_require_signal(state.rx_queue_drv.free)) {
        net_cancel_signal(state.rx_queue_drv.free);
        microkit_notify(DRIVER_CH);
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
    net_buffers_init((net_queue_t *)rx_free_drv, buffer_data_paddr, RX_QUEUE_SIZE_DRIV);

    if (net_require_signal(state.rx_queue_drv.free)) {
        net_cancel_signal(state.rx_queue_drv.free);
        microkit_notify_delayed(DRIVER_CH);
    }
}
