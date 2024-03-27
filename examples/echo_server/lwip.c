/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>
#include <sddf/network/arp.h>
#include <sddf/network/queue.h>
#include <sddf/timer/client.h>
#include <sddf/benchmark/sel4bench.h>
#include <ethernet_config.h>
#include <string.h>
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"

#include "echo.h"

#define TIMER  1
#define RX_CH  2
#define TX_CH  3
#define ARP    7

#define LWIP_TICK_MS 100
#define NUM_PBUFFS 512

uintptr_t rx_free;
uintptr_t rx_active;
uintptr_t tx_free;
uintptr_t tx_active;
uintptr_t rx_buffer_data_region;
uintptr_t tx_buffer_data_region;
uintptr_t uart_base;

/* Booleans to indicate whether packets have been enqueued during notification handling */
static bool notify_tx;
static bool notify_rx;

/* Wrapper over custom_pbuf structure to keep track of buffer offset */
typedef struct pbuf_custom_offset {
    struct pbuf_custom custom;
    uintptr_t offset;
} pbuf_custom_offset_t;

LWIP_MEMPOOL_DECLARE(
    RX_POOL,
    NUM_PBUFFS * 2,
    sizeof(struct pbuf_custom_offset),
    "Zero-copy RX pool"
);

typedef struct state {
    struct netif netif;
    uint8_t mac[ETH_HWADDR_LEN];
    net_queue_handle_t rx_queue;
    net_queue_handle_t tx_queue;
    struct pbuf *head;
    struct pbuf *tail;
} state_t;

state_t state;

void set_timeout(void)
{
    sddf_timer_set_timeout(TIMER, LWIP_TICK_MS * NS_IN_MS);
}

uint32_t sys_now(void)
{
    return sddf_timer_time_now(TIMER) / NS_IN_MS;
}

/**
 * Free a pbuf. This also returns the underlying buffer to the receive free ring.
 * 
 * @param p pbuf to free.
 */
static void interface_free_buffer(struct pbuf *p)
{
    SYS_ARCH_DECL_PROTECT(old_level);
    pbuf_custom_offset_t *custom_pbuf_offset = (pbuf_custom_offset_t *)p;
    SYS_ARCH_PROTECT(old_level);
    net_buff_desc_t buffer = {custom_pbuf_offset->offset, 0};
    int err = net_enqueue_free(&(state.rx_queue), buffer);
    assert(!err);
    notify_rx = true;
    LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf_offset);
    SYS_ARCH_UNPROTECT(old_level);
}

/**
 * Create a pbuf structure to pass to the network interface.
 *
 * @param state client state data.
 * @param buffer shared buffer containing the data.
 * @param length length of data.
 *
 * @return the newly created pbuf. Can be cast to pbuf_custom.
 */
static struct pbuf *create_interface_buffer(uintptr_t offset, size_t length)
{
    pbuf_custom_offset_t *custom_pbuf_offset = (pbuf_custom_offset_t *) LWIP_MEMPOOL_ALLOC(RX_POOL);
    custom_pbuf_offset->offset = offset;
    custom_pbuf_offset->custom.custom_free_function = interface_free_buffer;

    return pbuf_alloced_custom(
        PBUF_RAW,
        length,
        PBUF_REF,
        &custom_pbuf_offset->custom,
        (void *)(offset + rx_buffer_data_region),
        NET_BUFFER_SIZE 
    );
}

/**
 * Stores a pbuf to be transmitted upon available transmit buffers.
 * 
 * @param p pbuf to be stored.
 */
void enqueue_pbufs(struct pbuf *p)
{
    /* Indicate to the multiplexer that we require transmit free buffers */
    net_request_signal_free(&state.tx_queue);

    if (state.head == NULL) {
        state.head = p;
    } else {
        state.tail->next_chain = p;
    }
    state.tail = p;

    /* Increment refernce count to ensure this pbuf is not freed by lwip */
    pbuf_ref(p);
}

/** 
 * Insert pbuf into transmit active queue. If no free buffers available or transmit active queue is full, 
 * stores pbuf to be sent upon buffers becoming available. 
 * */
static err_t lwip_eth_send(struct netif *netif, struct pbuf *p)
{
    // sddf_dprintf("transmitting packet\n");
    
    if (p->tot_len > NET_BUFFER_SIZE) {
        sddf_dprintf("LWIP|ERROR: attempted to send a packet of size  %u > BUFFER SIZE  %u\n", p->tot_len, NET_BUFFER_SIZE);
        return ERR_MEM;
    }

    if (net_queue_empty_free(&state.tx_queue)) {
        enqueue_pbufs(p);
        return ERR_OK;
    }
    
    net_buff_desc_t buffer;
    int err = net_dequeue_free(&state.tx_queue, &buffer);
    assert(!err);

    unsigned char *frame = (unsigned char *)(buffer.io_or_offset + tx_buffer_data_region);
    unsigned int copied = 0;
    for (struct pbuf *curr = p; curr != NULL; curr = curr->next) {
        memcpy(frame + copied, curr->payload, curr->len);
        copied += curr->len;
    }

    buffer.len = copied;
    err = net_enqueue_active(&state.tx_queue, buffer);
    assert(!err);

    notify_tx = true;

    return ERR_OK;
}

void transmit(void)
{
    // bool reprocess = true;
    // while (reprocess) {
        while(state.head != NULL && !net_queue_empty_free(&state.tx_queue)) {
            err_t err = lwip_eth_send(&state.netif, state.head);
            if (err == ERR_MEM) {
                sddf_dprintf("LWIP|ERROR: attempted to send a packet of size  %u > BUFFER SIZE  %u\n", state.head->tot_len, NET_BUFFER_SIZE);
            }
            else if (err != ERR_OK) {
                sddf_dprintf("LWIP|ERROR: unkown error when trying to send pbuf  %p\n", state.head);
            }
            
            struct pbuf *temp = state.head;
            state.head = temp->next_chain;
            if (state.head == NULL) state.tail = NULL;
            pbuf_free(temp);
        }

        /* Only request a signal if no more pbufs enqueud to send */
        // if (state.head == NULL || !net_queue_empty_free(&state.tx_queue)) net_cancel_signal_free(&state.tx_queue);
        // else net_request_signal_free(&state.tx_queue);
        // reprocess = false;

        // if (state.head != NULL && !net_queue_empty_free(&state.tx_queue)) {
        //     net_cancel_signal_free(&state.tx_queue);
        //     reprocess = true;
        // }
    // }
}

void receive(void)
{
    // sddf_dprintf("lwip: receive\n");
    // bool reprocess = true;
    // while (reprocess) {
        while (!net_queue_empty_active(&state.rx_queue)) {
            // sddf_dprintf("lwip: receive: got buffer\n");
            net_buff_desc_t buffer;
            int err = net_dequeue_active(&state.rx_queue, &buffer);
            assert(!err);

            struct pbuf *p = create_interface_buffer(buffer.io_or_offset, buffer.len);
            if (state.netif.input(p, &state.netif) != ERR_OK) {
                sddf_dprintf("LWIP|ERROR: unkown error inputting pbuf into network stack\n");
                pbuf_free(p);
            }
        }
        
        // net_request_signal_active(&state.rx_queue);
        // reprocess = false;

    //     if (!net_queue_empty_active(&state.rx_queue)) {
    //         net_cancel_signal_active(&state.rx_queue);
    //         reprocess = true;
    //     }
    // }
}

/**
 * Initialise the network interface data structure.
 * 
 * @param netif network interface data structuer.
 */
static err_t ethernet_init(struct netif *netif)
{
    if (netif->state == NULL) return ERR_ARG;
    state_t *data = netif->state;

    netif->hwaddr[0] = data->mac[0];
    netif->hwaddr[1] = data->mac[1];
    netif->hwaddr[2] = data->mac[2];
    netif->hwaddr[3] = data->mac[3];
    netif->hwaddr[4] = data->mac[4];
    netif->hwaddr[5] = data->mac[5];
    netif->mtu = ETHER_MTU;
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->output = etharp_output;
    netif->linkoutput = lwip_eth_send;
    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;
    return ERR_OK;
}

/* Callback function that prints DHCP supplied IP address and registers it with ARP component. */
static void netif_status_callback(struct netif *netif)
{
    sddf_printf("NEITF STATUS CALLBACK: \n");
    if (dhcp_supplied_address(netif)) {
        bool success = arp_register_ipv4(ARP, ip4_addr_get_u32(netif_ip4_addr(netif)), state.mac);
        if (!success) {
            sddf_printf("LWIP|ERR: could not register IP with ARP\n");
        } else {
            sddf_printf("LWIP|NOTICE: DHCP request for %s returned IP address: %s\n", microkit_name, ip4addr_ntoa(netif_ip4_addr(netif)));
        }
    } else {
        sddf_printf("UH OH WHAT DO??????\n");
    }
}

void init(void)
{
    cli_queue_init_sys(microkit_name, &state.rx_queue, rx_free, rx_active, &state.tx_queue, tx_free, tx_active);
    net_buffers_init(&state.tx_queue, 0);

    lwip_init();
    set_timeout();

    LWIP_MEMPOOL_INIT(RX_POOL);

    cli_mac_addr_init_sys(microkit_name, state.mac);

    /* Set dummy IP configuration values to get lwIP bootstrapped  */
    struct ip4_addr netmask, ipaddr, gw, multicast;
    ipaddr_aton("0.0.0.0", &gw);
    ipaddr_aton("0.0.0.0", &ipaddr);
    ipaddr_aton("0.0.0.0", &multicast);
    ipaddr_aton("255.255.255.0", &netmask);

    state.netif.name[0] = 'e';
    state.netif.name[1] = '0';

    if (!netif_add(&(state.netif), &ipaddr, &netmask, &gw, (void *)&state,
              ethernet_init, ethernet_input)) sddf_dprintf("LWIP|ERROR: Netif add returned NULL\n");

    netif_set_default(&(state.netif));
    netif_set_status_callback(&(state.netif), netif_status_callback);
    netif_set_up(&(state.netif));

    if (dhcp_start(&(state.netif))) sddf_dprintf("LWIP|ERROR: failed to start DHCP negotiation\n");

    setup_udp_socket();
    setup_utilization_socket();

    if (notify_rx // && net_require_signal_free(&state.rx_queue)
        ) {
        // net_cancel_signal_free(&state.rx_queue);
        notify_rx = false;
        if (!have_signal) microkit_notify_delayed(RX_CH);
        else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + RX_CH) microkit_notify(RX_CH);
    }

    if (notify_tx // && net_require_signal_active(&state.tx_queue)
        ) {
        // net_cancel_signal_active(&state.tx_queue);
        notify_tx = false;
        if (!have_signal) microkit_notify_delayed(TX_CH);
        else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + TX_CH) microkit_notify(TX_CH);
    }
}

void notified(microkit_channel ch)
{
    switch(ch) {
        case RX_CH:
            receive();
            break;
        case TIMER:
            sys_check_timeouts();
            set_timeout();
            break;
        case TX_CH:
            transmit();
            receive();
            break;
        default:
            sddf_dprintf("LWIP|LOG: received notification on unexpected channel: %u\n", ch);
            break;
    }
    
    if (notify_rx // && net_require_signal_free(&state.rx_queue)
        ) {
        // net_cancel_signal_free(&state.rx_queue);
        notify_rx = false;
        if (!have_signal) microkit_notify_delayed(RX_CH);
        else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + RX_CH) microkit_notify(RX_CH);
    }

    if (notify_tx // && net_require_signal_active(&state.tx_queue)
        ) {
        // net_cancel_signal_active(&state.tx_queue);
        notify_tx = false;
        if (!have_signal) microkit_notify_delayed(TX_CH);
        else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + TX_CH) microkit_notify(TX_CH);
    }
}
