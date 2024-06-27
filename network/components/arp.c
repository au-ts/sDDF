/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <microkit.h>
#include <sddf/network/queue.h>
#include <sddf/network/constants.h>
#include <sddf/network/util.h>
#include <sddf/util/printf.h>
#include <ethernet_config.h>

#define RX_CH 0
#define TX_CH 1
#define CLIENT_CH 2
#define REG_IP 0
#define IPV4_PROTO_LEN 4
#define PADDING_SIZE 10
#define LWIP_IANA_HWTYPE_ETHERNET 1
#define NUM_ARP_CLIENTS (NUM_NETWORK_CLIENTS - 1)

net_queue_handle_t rx_queue;
net_queue_handle_t tx_queue;

uintptr_t rx_free;
uintptr_t rx_active;
uintptr_t tx_free;
uintptr_t tx_active;

uintptr_t rx_buffer_data_region;
uintptr_t tx_buffer_data_region;

uintptr_t uart_base;

uint8_t mac_addrs[NUM_ARP_CLIENTS][ETH_HWADDR_LEN];
uint32_t ipv4_addrs[NUM_ARP_CLIENTS];

struct __attribute__((__packed__)) arp_packet {
    uint8_t ethdst_addr[ETH_HWADDR_LEN];
    uint8_t ethsrc_addr[ETH_HWADDR_LEN];
    uint16_t type;
    uint16_t hwtype;
    uint16_t proto;
    uint8_t hwlen;
    uint8_t protolen;
    uint16_t opcode;
    uint8_t hwsrc_addr[ETH_HWADDR_LEN];
    uint32_t ipsrc_addr;
    uint8_t hwdst_addr[ETH_HWADDR_LEN];
    uint32_t ipdst_addr;
    uint8_t padding[10];
    uint32_t crc;
};

static char *ipaddr_to_string(uint32_t s_addr, char *buf, int buflen)
{
    char inv[3], *rp;
    uint8_t *ap, rem, n, i;
    int len = 0;

    rp = buf;
    ap = (uint8_t *)&s_addr;
    for (n = 0; n < 4; n++) {
        i = 0;
        do {
            rem = *ap % (uint8_t)10;
            *ap /= (uint8_t)10;
            inv[i++] = (char)('0' + rem);
        } while (*ap);
        while (i--) {
            if (len++ >= buflen) {
                return NULL;
            }
            *rp++ = inv[i];
        }
        if (len++ >= buflen) {
            return NULL;
        }
        *rp++ = '.';
        ap++;
    }
    *--rp = 0;
    return buf;
}

static int match_ip_to_client(uint32_t addr)
{
    for (int i = 0; i < NUM_ARP_CLIENTS; i++) {
        if (ipv4_addrs[i] == addr) {
            return i;
        }
    }

    return -1;
}

static int arp_reply(const uint8_t ethsrc_addr[ETH_HWADDR_LEN],
                     const uint8_t ethdst_addr[ETH_HWADDR_LEN],
                     const uint8_t hwsrc_addr[ETH_HWADDR_LEN], const uint32_t ipsrc_addr,
                     const uint8_t hwdst_addr[ETH_HWADDR_LEN], const uint32_t ipdst_addr)
{
    if (net_queue_empty_free(&tx_queue)) {
        sddf_dprintf("ARP|LOG: Transmit free queue empty or transmit active queue full. Dropping reply\n");
        return -1;
    }

    net_buff_desc_t buffer;
    int err = net_dequeue_free(&tx_queue, &buffer);
    assert(!err);

    uintptr_t addr = tx_buffer_data_region + buffer.io_or_offset;

    struct arp_packet *reply = (struct arp_packet *)addr;
    memcpy(&reply->ethdst_addr, ethdst_addr, ETH_HWADDR_LEN);
    memcpy(&reply->ethsrc_addr, ethsrc_addr, ETH_HWADDR_LEN);

    reply->type = HTONS(ETH_TYPE_ARP);
    reply->hwtype = HTONS(LWIP_IANA_HWTYPE_ETHERNET);
    reply->proto = HTONS(ETH_TYPE_IP);
    reply->hwlen = ETH_HWADDR_LEN;
    reply->protolen = IPV4_PROTO_LEN;
    reply->opcode = HTONS(ETHARP_OPCODE_REPLY);

    memcpy(&reply->hwsrc_addr, hwsrc_addr, ETH_HWADDR_LEN);
    reply->ipsrc_addr = ipsrc_addr;
    memcpy(&reply->hwdst_addr, hwdst_addr, ETH_HWADDR_LEN);
    reply->ipdst_addr = ipdst_addr;
    memset(&reply->padding, 0, 10);

    buffer.len = 56;
    err = net_enqueue_active(&tx_queue, buffer);
    assert(!err);

    return 0;
}

void receive(void)
{
    bool transmitted = false;
    bool reprocess = true;
    while (reprocess) {
        while (!net_queue_empty_active(&rx_queue)) {
            net_buff_desc_t buffer;
            int err = net_dequeue_active(&rx_queue, &buffer);
            assert(!err);
            uintptr_t addr = rx_buffer_data_region + buffer.io_or_offset;

            /* Check if packet is an ARP request */
            struct ethernet_header *ethhdr = (struct ethernet_header *)addr;
            if (ethhdr->type == HTONS(ETH_TYPE_ARP)) {
                struct arp_packet *pkt = (struct arp_packet *)addr;
                /* Check if it's a probe, ignore announcements */
                if (pkt->opcode == HTONS(ETHARP_OPCODE_REQUEST)) {
                    /* Check it it's for a client */
                    int client = match_ip_to_client(pkt->ipdst_addr);
                    if (client >= 0) {
                        /* Send a response */
                        if (!arp_reply(mac_addrs[client], pkt->ethsrc_addr, mac_addrs[client], pkt->ipdst_addr,
                                       pkt->hwsrc_addr, pkt->ipsrc_addr)) {
                            transmitted = true;
                        }
                    }
                }
            }

            buffer.len = 0;
            err = net_enqueue_free(&rx_queue, buffer);
            assert(!err);
        }

        net_request_signal_active(&rx_queue);
        reprocess = false;

        if (!net_queue_empty_active(&rx_queue)) {
            net_cancel_signal_active(&rx_queue);
            reprocess = true;
        }
    }

    if (transmitted && net_require_signal_active(&tx_queue)) {
        net_cancel_signal_active(&tx_queue);
        microkit_notify_delayed(TX_CH);
    }
}

void notified(microkit_channel ch)
{
    receive();
}

seL4_MessageInfo_t protected(microkit_channel ch, microkit_msginfo msginfo)
{
    int client = ch - CLIENT_CH;
    if (client >= NUM_ARP_CLIENTS || client < 0) {
        sddf_dprintf("ARP|LOG: PPC from unkown client %d\n", client);
        return microkit_msginfo_new(0, 0);
    }

    uint32_t ip_addr = microkit_mr_get(0);
    uint32_t mac_higher = microkit_mr_get(1);
    uint32_t mac_lower = microkit_mr_get(2);
    uint64_t mac = (((uint64_t) mac_higher) << 32) | mac_lower;

    char buf[16];
    switch (microkit_msginfo_get_label(msginfo)) {
    case REG_IP:
        sddf_printf("ARP|NOTICE: client%d registering ip address: %s with MAC: %02lx:%02lx:%02lx:%02lx:%02lx:%02lx\n",
                    client, ipaddr_to_string(ip_addr, buf, 16), mac >> 40, mac >> 32 & 0xff, mac >> 24 & 0xff,
                    mac >> 16 & 0xff, mac >> 8 & 0xff, mac & 0xff);
        ipv4_addrs[client] = ip_addr;
        break;
    default:
        sddf_dprintf("ARP|LOG: PPC from client%d with unknown message label %lu\n", client,
                     microkit_msginfo_get_label(msginfo));
        break;
    }

    return microkit_msginfo_new(0, 0);
}

void init(void)
{
    net_queue_init(&rx_queue, (net_queue_t *)rx_free, (net_queue_t *)rx_active, ETHERNET_RX_QUEUE_SIZE_ARP);
    net_queue_init(&tx_queue, (net_queue_t *)tx_free, (net_queue_t *)tx_active, ETHERNET_TX_QUEUE_SIZE_ARP);
    net_buffers_init(&tx_queue, 0);

    ethernet_arp_mac_addr_init_sys(microkit_name, (uint8_t *) mac_addrs);
}
