/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <string.h>
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/sys.h"
#include "lwip/dhcp.h"

#include <sddf/network/shared_ringbuffer.h>
#include <sddf/timer/client.h>
#include <sddf/util/cache.h>
#include <sddf/util/fence.h>
#include <sddf/benchmark/sel4bench.h>

#include "echo.h"

#define TIMER  1
#define RX_CH  2
#define TX_CH  3
#define ARP    7

#define LWIP_TICK_MS 100

#define _unused(x) ((void)(x))
#ifndef MAC_ADDRESS
#  define MAC_ADDRESS (0x525401000000ULL)
#endif

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t rx_free;
uintptr_t rx_used;
uintptr_t tx_free;
uintptr_t tx_used;
uintptr_t shared_dma_vaddr_rx;
uintptr_t shared_dma_vaddr_tx;
uintptr_t uart_base;

static bool notify_tx = false;
static bool notify_rx = false;

/* 
 * LWIP mempool declare literally just initialises an array 
 * big enough with the correct alignment 
 */
typedef struct lwip_custom_pbuf {
    struct pbuf_custom custom;
    uintptr_t buffer;
} lwip_custom_pbuf_t;

LWIP_MEMPOOL_DECLARE(
    RX_POOL,
    NUM_BUFFERS * 2,
    sizeof(lwip_custom_pbuf_t),
    "Zero-copy RX pool"
);

typedef struct state {
    struct netif netif;
    /* mac address for this client */
    uint8_t mac[6];

    /* Pointers to shared buffers */
    ring_handle_t rx_ring;
    ring_handle_t tx_ring;

    /* pbufs left to process */
    struct pbuf *head;
    struct pbuf *tail;
    uint32_t num_pbufs;
} state_t;

struct log {
    lwip_custom_pbuf_t *pbuf_addr;
    uintptr_t dma_addr;
    char action[4]; // en for enqueuing, de for dequeing, 
};

state_t state;
struct log logbuffer[NUM_BUFFERS * 2];
int head = 0;

void
set_timeout(void)
{
    sddf_timer_set_timeout(TIMER, LWIP_TICK_MS * US_IN_MS);
}

uint32_t
sys_now(void)
{
    return sddf_timer_time_now(TIMER) * US_IN_MS;
}

static void
dump_mac(uint8_t *mac)
{
    print(microkit_name);
    print(": ");
    for (unsigned i = 0; i < 6; i++) {
        put8((mac[i] >> 4) & 0xf);
        put8(mac[i] & 0xf);
        if (i < 5) {
            putC(':');
        }
    }
    putC('\n');
}

static inline void
request_used_ntfn(ring_handle_t *ring)
{
    ring->used_ring->notify_reader = true;
}

static inline void
cancel_used_ntfn(ring_handle_t *ring)
{
    ring->used_ring->notify_reader = false;
}

static inline void
request_free_ntfn(ring_handle_t *ring)
{
    ring->free_ring->notify_reader = true;
}

static inline void
cancel_free_ntfn(ring_handle_t *ring)
{
    ring->free_ring->notify_reader = false;
}

static inline void return_buffer(uintptr_t addr)
{
    /* As the rx_free ring is the size of the number of buffers we have,
    the ring should never be full. 
    FIXME: This full condition could change... */
    int err = enqueue_free(&(state.rx_ring), addr, BUF_SIZE, NULL);
    assert(!err);
    _unused(err);
    notify_rx = true;
}

/**
 * Free a pbuf. This also returns the underlying buffer to
 * the appropriate place.
 *
 * @param buf pbuf to free.
 *
 */
static void interface_free_buffer(struct pbuf *buf)
{
    SYS_ARCH_DECL_PROTECT(old_level);
    lwip_custom_pbuf_t *custom_pbuf = (lwip_custom_pbuf_t *)buf;
    SYS_ARCH_PROTECT(old_level);
    return_buffer(custom_pbuf->buffer);
    LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
    SYS_ARCH_UNPROTECT(old_level);
}

/**
 * Create a pbuf structure to pass to the network interface.
 *
 * @param state client state data.
 * @param buffer ethernet buffer containing metadata for the actual buffer
 * @param length length of data
 *
 * @return the newly created pbuf.
 */
static struct pbuf *
create_interface_buffer(state_t *state, uintptr_t buffer, size_t length)
{
    lwip_custom_pbuf_t *custom_pbuf = (lwip_custom_pbuf_t *) LWIP_MEMPOOL_ALLOC(RX_POOL);

    custom_pbuf->buffer = buffer;
    custom_pbuf->custom.custom_free_function = interface_free_buffer;

    return pbuf_alloced_custom(
        PBUF_RAW,
        length,
        PBUF_REF,
        &custom_pbuf->custom,
        (void *)buffer,
        BUF_SIZE
    );
}

/**
 * Allocate an empty TX buffer from the empty pool
 *
 * @param state client state data.
 * @param length length of buffer required
 *
 */
static inline uintptr_t
alloc_tx_buffer(size_t length)
{
    if (BUF_SIZE < length) {
        print("Requested buffer size too large.");
        return (uintptr_t) NULL;
    }

    uintptr_t addr;
    unsigned int len;
    void *cookie;

    int err = dequeue_free(&(state.tx_ring), &addr, &len, &cookie);
    if (err) {
        return (uintptr_t) NULL;
    }

    if (!addr) {
        print("LWIP|ERROR: dequeued a null buffer\n");
        return (uintptr_t) NULL;
    }

    return addr;
}

void
enqueue_pbufs(struct pbuf *buff)
{
    request_free_ntfn(&state.tx_ring);
    if (state.head == NULL) {
        state.head = buff;
    } else {
        state.tail->next_chain = buff;
    }
    
    // move the tail pointer to the new tail.
    state.tail = buff;

    // we need to reference the pbufs so they
    // don't get freed (as we are only responsible
    // for freeing pbufs allocated by us - lwip also
    // allocates it's own. )
    pbuf_ref(buff);
    state.num_pbufs++;
}

/* Grab an free TX buffer, copy pbuf data over,
    add to used tx ring, notify server */
static err_t
lwip_eth_send(struct netif *netif, struct pbuf *p)
{
    /* Grab an free TX buffer, copy pbuf data over,
    add to used tx ring, notify server */
    err_t ret = ERR_OK;
    int err;

    if (p->tot_len > BUF_SIZE) {
        print("LWIP|ERROR: lwip_eth_send total length > BUF SIZE\n");
        return ERR_MEM;
    }

    if (ring_full(state.tx_ring.used_ring)) {
        enqueue_pbufs(p);
        return ERR_OK;
    }

    
    uintptr_t buffer = alloc_tx_buffer(p->tot_len);
    if (buffer == (uintptr_t) NULL) {
        enqueue_pbufs(p);
        return ERR_OK;
    }

    unsigned char *frame = (unsigned char *)buffer;
    /* Copy all buffers that need to be copied */
    unsigned int copied = 0;
    for (struct pbuf *curr = p; curr != NULL; curr = curr->next) {
        // this ensures the pbufs get freed properly. 
        unsigned char *buffer_dest = &frame[copied];
        if ((uintptr_t)buffer_dest != (uintptr_t)curr->payload) {
            /* Don't copy memory back into the same location */
            memcpy(buffer_dest, curr->payload, curr->len);
        }
        copied += curr->len;
    }

    cache_clean((unsigned long) frame, (unsigned long) frame + copied);

    /* insert into the used tx queue */
    err = enqueue_used(&(state.tx_ring), (uintptr_t)frame, copied, NULL);
    if (err) {
        assert(!err);
        return ERR_MEM;
    }

    /* Notify the server for next time we recv() */
    notify_tx = true;

    return ret;
}

void
process_tx_queue(void)
{
    int err;
    struct pbuf *current = state.head;
    struct pbuf *temp;
    process_tx_queue_:
    while(current != NULL && !ring_empty(state.tx_ring.free_ring) && !ring_full(state.tx_ring.used_ring)) {
        uintptr_t buffer = alloc_tx_buffer(current->tot_len);
        if (buffer == (uintptr_t) NULL) {
            print("process_tx_queue() could not alloc_tx_buffer\n");
            break;
        }

        unsigned char *frame = (unsigned char *)buffer;
        /* Copy all buffers that need to be copied */
        unsigned int copied = 0;
        for (struct pbuf *curr = current; curr != NULL; curr = curr->next) {
            // this ensures the pbufs get freed properly. 
            unsigned char *buffer_dest = &frame[copied];
            if ((uintptr_t)buffer_dest != (uintptr_t)curr->payload) {
                /* Don't copy memory back into the same location */
                memcpy(buffer_dest, curr->payload, curr->len);
            }
            copied += curr->len;
        }

        /*err = seL4_ARM_VSpace_Clean_Data(3, frame, frame + copied);
        if (err) {
            print("LWIP|ERROR: ARM VSpace clean failed: ");
            puthex64(err);
            print("\n");
        }*/
        cache_clean((unsigned long) frame, (unsigned long) frame + copied);

        /* insert into the used tx queue */
        err = enqueue_used(&(state.tx_ring), buffer, copied, NULL);
        if (err) {
            print("LWIP|ERROR: TX used ring full\n");
            break;
        }

        /* Notify the server for next time we recv() */
        notify_tx = true;

        /* free the pbufs. */
        temp = current;
        current = current->next_chain;
        pbuf_free(temp);
        state.num_pbufs--;
    }

    // if curr != NULL, we need to make sure we don't lose it and can come back
    state.head = current;

    if (current == NULL || !ring_empty(state.tx_ring.free_ring)) {
        cancel_free_ntfn(&state.tx_ring);
    } else {
        request_free_ntfn(&state.tx_ring);

        THREAD_MEMORY_FENCE();

        if (current != NULL && !ring_empty(state.tx_ring.free_ring) && !ring_full(state.tx_ring.used_ring)) {
            cancel_free_ntfn(&state.tx_ring);
            goto process_tx_queue_;
        }

    }
}

void
process_rx_queue(void)
{
    process_rx_queue_:
    while (!ring_empty(state.rx_ring.used_ring)) {
        uintptr_t addr;
        unsigned int len;
        void *cookie;

        dequeue_used(&state.rx_ring, &addr, &len, &cookie);

        struct pbuf *p = create_interface_buffer(&state, addr, len);

        if (state.netif.input(p, &state.netif) != ERR_OK) {
            // If it is successfully received, the receiver controls whether or not it gets freed.
            print("LWIP|ERROR: netif.input() != ERR_OK");
            pbuf_free(p);
        }
    }
    
    request_used_ntfn(&state.rx_ring);

    THREAD_MEMORY_FENCE();

    if (!ring_empty(state.rx_ring.used_ring)) {
        cancel_used_ntfn(&state.rx_ring);
        goto process_rx_queue_;
    }
}

/**
 * Initialise the network interface data structure.
 *
 * @param netif network interface data structuer.
 */
static err_t ethernet_init(struct netif *netif)
{
    if (netif->state == NULL) {
        return ERR_ARG;
    }

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

static void netif_status_callback(struct netif *netif)
{
    if (dhcp_supplied_address(netif)) {
        /* Tell the ARP component so we it can respond to ARP requests. */
        microkit_mr_set(0, ip4_addr_get_u32(netif_ip4_addr(netif)));
        microkit_mr_set(1, (state.mac[0] << 24) | (state.mac[1] << 16) | (state.mac[2] << 8) | (state.mac[3]));
        microkit_mr_set(2, (state.mac[4] << 24) | (state.mac[5] << 16));
        microkit_ppcall(ARP, microkit_msginfo_new(0, 3));

        print("DHCP request finished, IP address for netif ");
        print(netif->name);
        print(" is: ");
        print(ip4addr_ntoa(netif_ip4_addr(netif)));
        print("\n");
    }
}

static void get_mac(void)
{
    /* For now just use a dummy hardcoded mac address.*/
    int i;
    for (i = 5; i >= 0; --i) {
        state.mac[5 - i] = 0xff & (MAC_ADDRESS >> (i*8));
    }

    if (strcmp(microkit_name, "client0")) {
        state.mac[5] += 0x1;
    }
    /* microkit_ppcall(RX_CH, microkit_msginfo_new(0, 0));
    uint32_t palr = microkit_mr_get(0);
    uint32_t paur = microkit_mr_get(1);
    state.mac[0] = palr >> 24;
    state.mac[1] = palr >> 16 & 0xff;
    state.mac[2] = palr >> 8 & 0xff;
    state.mac[3] = palr & 0xff;
    state.mac[4] = paur >> 24;
    state.mac[5] = paur >> 16 & 0xff;*/
}

void dump_log(void)
{
    for (int i = 0; i < NUM_BUFFERS * 2; i++) {
        print(logbuffer[i].action);
        print(",");
        puthex64((uint64_t) logbuffer[i].pbuf_addr);
        print(",");
        puthex64((uint64_t) logbuffer[i].dma_addr);
        print("\n");
    }
}

void init(void)
{
    /* Set up shared memory regions */
    ring_init(&state.rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, 1, NUM_BUFFERS, NUM_BUFFERS);
    ring_init(&state.tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, 0, NUM_BUFFERS, NUM_BUFFERS);

    state.head = NULL;
    state.tail = NULL;
    state.num_pbufs = 0;

    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        uintptr_t addr = shared_dma_vaddr_rx + (BUF_SIZE * i);
        int err = enqueue_free(&state.rx_ring, addr, BUF_SIZE, NULL);
        assert(!err);
        _unused(err);
    }

    lwip_init();
    set_timeout();

    LWIP_MEMPOOL_INIT(RX_POOL);

    get_mac();

    /* Set some dummy IP configuration values to get lwIP bootstrapped  */
    struct ip4_addr netmask, ipaddr, gw, multicast;
    ipaddr_aton("0.0.0.0", &gw);
    ipaddr_aton("0.0.0.0", &ipaddr);
    ipaddr_aton("0.0.0.0", &multicast);
    ipaddr_aton("255.255.255.0", &netmask);

    state.netif.name[0] = 'e';
    state.netif.name[1] = '0';

    if (!netif_add(&(state.netif), &ipaddr, &netmask, &gw, (void *)&state,
              ethernet_init, ethernet_input)) {
        print("Netif add returned NULL\n");
    }

    netif_set_default(&(state.netif));

    netif_set_status_callback(&(state.netif), netif_status_callback);
    netif_set_up(&(state.netif));

    if (dhcp_start(&(state.netif))) {
        print("failed to start DHCP negotiation\n");
    }

    setup_udp_socket();
    setup_utilization_socket();

    request_used_ntfn(&state.rx_ring);
    request_used_ntfn(&state.tx_ring);
    request_free_ntfn(&state.tx_ring);

    if (notify_rx && state.rx_ring.free_ring->notify_reader) {
        notify_rx = false;
        if (!have_signal) {
            microkit_notify_delayed(RX_CH);
        } else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + RX_CH) {
            microkit_notify(RX_CH);
        }
    }

    if (notify_tx && state.tx_ring.used_ring->notify_reader) {
        notify_tx = false;
        if (!have_signal) {
            microkit_notify_delayed(TX_CH);
        } else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + TX_CH) {
            microkit_notify(TX_CH);
        }
    }

    print(microkit_name);
    print(": elf PD init complete\n");
}

void notified(microkit_channel ch)
{
    switch(ch) {
        case RX_CH:
            process_rx_queue();
            break;
        case TIMER:
            // check timeouts.
            sys_check_timeouts();
            // set a new timeout
            set_timeout();
            break;
        case TX_CH:
            process_tx_queue();
            process_rx_queue();
            break;
        default:
            microkit_dbg_puts("lwip: received notification on unexpected channel\n");
            assert(0);
            break;
    }
    
    if (notify_rx && state.rx_ring.free_ring->notify_reader) {
        state.rx_ring.free_ring->notify_reader = false;
        notify_rx = false;
        if (!have_signal) {
            microkit_notify_delayed(RX_CH);
        } else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + RX_CH) {
            microkit_notify(RX_CH);
        }
    }

    if (notify_tx && state.tx_ring.used_ring->notify_reader) {
        state.tx_ring.used_ring->notify_reader = false;
        notify_tx = false;
        if (!have_signal) {
            microkit_notify_delayed(TX_CH);
        } else if (signal_cap != BASE_OUTPUT_NOTIFICATION_CAP + TX_CH) {
            microkit_notify(TX_CH);
        }
    }
}
