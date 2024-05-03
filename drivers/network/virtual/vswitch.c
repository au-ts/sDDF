/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/network/queue.h>
#include <sddf/network/mac802.h>
#include <sddf/network/util.h>
#include <sddf/util/fence.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>

// #include "ethernet.h"
#include "layout.h"

/*
 * package flow:
 * 
 *         TX               RX
 * src ----------VSWITCH----------> dest
 * 
 */

#define TX_COUNT 256
#define RX_COUNT 256
#define MAX_COUNT MAX(RX_COUNT, TX_COUNT)

typedef struct vswitch_client {
    uint8_t mac_addr[6];
    net_queue_handle_t rx_queue;
    net_queue_handle_t tx_queue;
    size_t rx_ch;
    size_t tx_ch;
    // size_t getmac_ch;
} vswitch_client_t;

static vswitch_client_t clients[NUM_CLIENTS];

static void debug_dump_packet(int len, uint8_t *buffer)
{
    struct ether_addr *macaddr = (struct ether_addr *)buffer;
    sddf_dprintf("VIRTIO(NET)|DUMP PACKET:\n");
    sddf_dprintf("dest MAC: "PR_MAC802_ADDR", src MAC: "PR_MAC802_ADDR", type: 0x%02x%02x, size: %d\n",
            PR_MAC802_DEST_ADDR_ARGS(macaddr), PR_MAC802_SRC_ADDR_ARGS(macaddr),
            macaddr->etype[0], macaddr->etype[1], len);
    sddf_dprintf("-------------------- payload begin --------------------\n");
    for (int i = 0; i < len; i++) {
        sddf_dprintf("%02x ", macaddr->payload[i]);
    }
    sddf_dprintf("\n");
    sddf_dprintf("--------------------- payload end ---------------------\n");
    sddf_dprintf("\n");
}

int find_dest_client(int src, uint8_t *dest_macaddr)
{
    int dest = -1;
    for (int c = 0; c < NUM_CLIENTS; c++) {
        if (mac802_addr_eq(clients[c].mac_addr, dest_macaddr)) {
            dest = c;
            break;
        }
    }
    return dest;
}

void tx_if_possible(int src, int dest, net_buff_desc_t *s_buffer)
{
    if (!IS_CONN(src, dest)) {
        return;
    }
    // fetch empty buffer from sddf free queue (rx)
    if (!net_queue_empty_free(&clients[dest].rx_queue)) {
        net_buff_desc_t d_buffer;
        int err = net_dequeue_free(&clients[dest].rx_queue, &d_buffer);
        assert(!err);

        // copy (@jade: any alternative way? Like have a dedicate data region for each conn...)
        memcpy((void *)d_buffer.io_or_offset, (void *)s_buffer->io_or_offset, s_buffer->len);
        d_buffer.len = s_buffer->len;

        // @jade: do we need a memory barrier here?

        // ship to the sddf active queue (rx)
        err = net_enqueue_active(&clients[dest].rx_queue, d_buffer);
        assert(!err);

        // signal rx_ch (delay)
        microkit_notify_delayed(clients[dest].rx_ch);
    }
}

static void handle_tx(microkit_channel ch)
{
    for (int src = 0; src < NUM_CLIENTS; src++) {
        if (ch == clients[src].tx_ch) {
            // fetch packet(s) from the sddf active queue 
            while (!net_queue_empty_active(&clients[src].tx_queue)) {
                net_buff_desc_t buffer;
                int err = net_dequeue_active(&clients[src].tx_queue, &buffer);
                assert(!err);
                
                // find the dest client (if any) and send the packet (if possible)
                struct ether_addr *macaddr = (struct ether_addr *)buffer.io_or_offset;
                int dest;
                if (mac802_addr_eq_bcast(macaddr->ether_dest_addr_octet)) {
                    for (dest = 0; dest < NUM_CLIENTS; dest++) {
                        tx_if_possible(src, dest, &buffer);
                    }
                } else {
                    dest = find_dest_client(src, macaddr->ether_dest_addr_octet);
                    // @jade: is this check necessary? Should it be an assert?
                    // I wonder how the networking multiplexor works, package
                    // that does not have a vswich MAC address shouldn't be sent
                    // to this "driver" in the frist place.
                    if (dest >= 0) {
                        tx_if_possible(src, dest, &buffer);
                    }
                }

                // return buffer to the sddf free queue 
                err = net_enqueue_free(&clients[src].tx_queue, buffer);
                assert(!err);
            }
            return;
        }
    }
    assert(!"unexpected channel\n");
}

void notified(microkit_channel ch)
{
    // we never "receive" packets since this vswitch
    // doesn't connect to the outside world at all
    handle_tx(ch);
}

// @jade: this function should contain all the hard-coded craps, so it will be
// easier to refactor later.
void init(void)
{
    // @jade: should we initialize all the queues or 
    // should the clients initialize their own queues?

    // client #0
    net_queue_init(&clients[0].tx_queue, (net_queue_t *)tx_free_cli0,
                   (net_queue_t *)tx_active_cli0, TX_QUEUE_SIZE_DRIV);
    clients[0].tx_ch = TX_CLI0_CH;

    net_queue_init(&clients[0].rx_queue, (net_queue_t *)rx_free_cli0,
                   (net_queue_t *)rx_active_cli0, RX_QUEUE_SIZE_DRIV);
    clients[0].rx_ch = RX_CLI0_CH;

    // @jade: how do I tell clients their mac addresses?
    mac802_addr_cp(vswitch_mac_base, clients[0].mac_addr, 0);

    // client #1
    net_queue_init(&clients[1].tx_queue, (net_queue_t *)tx_free_cli1,
                   (net_queue_t *)tx_active_cli1, TX_QUEUE_SIZE_DRIV);
    clients[1].tx_ch = TX_CLI1_CH;

    net_queue_init(&clients[1].rx_queue, (net_queue_t *)rx_free_cli1,
                   (net_queue_t *)rx_active_cli1, RX_QUEUE_SIZE_DRIV);
    clients[1].rx_ch = RX_CLI1_CH;

    mac802_addr_cp(vswitch_mac_base, clients[1].mac_addr, 1);
}
