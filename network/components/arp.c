#include <microkit.h>
#include <sddf/network/shared_ringbuffer.h>
#include "util.h"
#include <string.h>
#include "lwip/ip_addr.h"
#include "netif/etharp.h"

#define TX_CH 1
#define REG_IP 0
#define CLIENT_CH_START 2
#define NUM_CLIENTS 2
#define ETH_HWADDR_LEN 6
#define IPV4_PROTO_LEN 4
#define PADDING_SIZE 10
#define LWIP_IANA_HWTYPE_ETHERNET 1
#define BUF_SIZE 2048
#define NUM_BUFFERS 512
#define SHARED_DMA_SIZE (BUF_SIZE * NUM_BUFFERS)

#define _unused(x) ((void)(x))


#define ROUND_DOWN(n, b) (((n) >> (b)) << (b))
#define LINE_START(a) ROUND_DOWN(a, CONFIG_L1_CACHE_LINE_SIZE_BITS)
#define LINE_INDEX(a) (LINE_START(a)>>CONFIG_L1_CACHE_LINE_SIZE_BITS)

static inline void
dsb(void)
{
    asm volatile("dsb sy" ::: "memory");
}

static inline void 
dmb(void)
{
    asm volatile("dmb sy" ::: "memory");
}

static inline void
cleanInvalByVA(unsigned long vaddr)
{
    asm volatile("dc civac, %0" : : "r"(vaddr));
    dsb();
}

static inline void
cleanByVA(unsigned long vaddr)
{
    asm volatile("dc cvac, %0" : : "r"(vaddr));
    dmb();
}

static void
cleanInvalidateCache(unsigned long start, unsigned long end)
{
    unsigned long line;
    unsigned long index;
    /* Clean the L1 range */

    /* Finally clean and invalidate the L1 range. The extra clean is only strictly neccessary
     * in a multiprocessor environment to prevent a write being lost if another core is
     * attempting a store at the same time. As the range should already be clean asking
     * it to clean again should not affect performance */
    for (index = LINE_INDEX(start); index < LINE_INDEX(end) + 1; index++) {
        line = index << CONFIG_L1_CACHE_LINE_SIZE_BITS;
        cleanInvalByVA(line);
    }
}

static void
cleanCache(unsigned long start, unsigned long end)
{
    unsigned long line;
    unsigned long index;

    for (index = LINE_INDEX(start); index < LINE_INDEX(end) + 1; index++) {
        line = index << CONFIG_L1_CACHE_LINE_SIZE_BITS;
        cleanByVA(line);
    }
}

uintptr_t rx_free;
uintptr_t rx_used;

uintptr_t tx_free;
uintptr_t tx_used;

uintptr_t shared_dma_vaddr_rx;
uintptr_t shared_dma_vaddr_tx;
uintptr_t uart_base;

ring_handle_t rx_ring;
ring_handle_t tx_ring;

uint8_t mac_addrs[NUM_CLIENTS][ETH_HWADDR_LEN];
// TODO: Expand this to support multiple ip addresses for a single client.
uint32_t ipv4_addrs[NUM_CLIENTS];

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

static char *
print_ipaddr(uint32_t s_addr, char *buf, int buflen)
{
    char inv[3];
    char *rp;
    u8_t *ap;
    u8_t rem;
    u8_t n;
    u8_t i;
    int len = 0;

    rp = buf;
    ap = (u8_t *)&s_addr;
    for (n = 0; n < 4; n++) {
    i = 0;
    do {
        rem = *ap % (u8_t)10;
        *ap /= (u8_t)10;
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

static int
match_arp_to_client(uint32_t addr)
{
    for (int i = 0; i < NUM_CLIENTS; i++) {
        if (ipv4_addrs[i] == addr) {
            return i;
        }
    }

    return -1;
}

int
arp_reply(const uint8_t ethsrc_addr[ETH_HWADDR_LEN],
          const uint8_t ethdst_addr[ETH_HWADDR_LEN],
          const uint8_t hwsrc_addr[ETH_HWADDR_LEN], const uint32_t ipsrc_addr,
          const uint8_t hwdst_addr[ETH_HWADDR_LEN], const uint32_t ipdst_addr)
{
    int err = 0;
    uintptr_t addr;
    unsigned int len;
    void *cookie = NULL;
    // TODO: Probably need to put continuations in here to ensure
    // used queue is not full and free queue is not empty. 
    err = dequeue_free(&tx_ring, &addr, &len, (void **)&cookie); 
    if (err) {
        print("ARP|ERROR: Dequeue free failed\n");
        return err;
    } else if (len < sizeof(struct arp_packet)) {
        /* this should never happen given we only enqueue 
            max packet sizes */
        print("ARP|ERROR: Dequeued buffer is too small\n");
        return -1;
    }

    // write the packet. 
    struct arp_packet *reply = (struct arp_packet *)addr;
    memcpy(&reply->ethdst_addr, ethdst_addr, ETH_HWADDR_LEN);
    memcpy(&reply->ethsrc_addr, ethsrc_addr, ETH_HWADDR_LEN);

    reply->type = lwip_htons(ETHTYPE_ARP);
    reply->hwtype = PP_HTONS(LWIP_IANA_HWTYPE_ETHERNET);
    reply->proto = PP_HTONS(ETHTYPE_IP);
    reply->hwlen = ETH_HWADDR_LEN;
    reply->protolen = IPV4_PROTO_LEN;
    reply->opcode = lwip_htons(ARP_REPLY);

    memcpy(&reply->hwsrc_addr, hwsrc_addr, ETH_HWADDR_LEN);
    reply->ipsrc_addr = ipsrc_addr;
    memcpy(&reply->hwdst_addr, hwdst_addr, ETH_HWADDR_LEN); 
    reply->ipdst_addr = ipdst_addr;

    // then padding of 10 bytes
    memset(&reply->padding, 0, 10);
    // then CRC (size of the arp packet (28B) + ethernet header (14B))
    // reply->crc = inet_chksum(reply, 42);

    // clean cache
    cleanCache((uintptr_t)reply, (uintptr_t)reply + 64);

    /* insert into the used tx queue */
    err = enqueue_used(&tx_ring, (uintptr_t)reply, 56, cookie);
    if (err) {
        print("ARP|ERROR: TX used ring full\n");
    }

    return err;
}

void
process_rx_complete(void)
{
    uint32_t transmitted = 0;
    process_rx_complete_: 
    while (!ring_empty(rx_ring.used_ring)) {
        int err;
        uintptr_t addr;
        unsigned int len;
        void *cookie = NULL;
        int client = -1;

        err = dequeue_used(&rx_ring, &addr, &len, (void **)&cookie);
        _unused(err);
        assert(!err);

        // Check if it's an ARP request 
        struct eth_hdr *ethhdr = (struct eth_hdr *)addr;
        if (ethhdr->type == PP_HTONS(ETHTYPE_ARP)) {
            struct arp_packet *pkt = (struct arp_packet *)addr;
            // CHeck if it's a probe (we don't care about announcements)
            if (pkt->opcode == PP_HTONS(ARP_REQUEST)) {
                // CHeck if it's for one of our clients.
                client = match_arp_to_client(pkt->ipdst_addr);
                if (client >= 0) {
                    // if so, send a response. 
                    if (!arp_reply(mac_addrs[client],
                                pkt->ethsrc_addr,
                                mac_addrs[client],
                                pkt->ipdst_addr,
                                pkt->hwsrc_addr,
                                pkt->ipsrc_addr))
                    {
                        transmitted++;
                    }
                }
            }
        }

        err = enqueue_free(&rx_ring, addr, BUF_SIZE, cookie);
        assert(!err);
    }

    rx_ring.used_ring->notify_reader = true;

    if (!ring_empty(rx_ring.used_ring)) {
        rx_ring.used_ring->notify_reader = false;
        goto process_rx_complete_;
    }

    if (transmitted) {
        // notify tx mux
        tx_ring.used_ring->notify_reader = false;
        microkit_notify_delayed(TX_CH);
    }
}

void
notified(microkit_channel ch)
{
    /* We have one job. */
    process_rx_complete();
}

static void
dump_mac(uint8_t *mac)
{
    for (unsigned i = 0; i < 6; i++) {
        putC(hexchar((mac[i] >> 4) & 0xf));
        putC(hexchar(mac[i] & 0xf));
        if (i < 5) {
            putC(':');
        }
    }
}

seL4_MessageInfo_t
protected(microkit_channel ch, microkit_msginfo msginfo)
{
    // get the client ID into our data structures.
    int client = ch - CLIENT_CH_START;
    if (client >= NUM_CLIENTS || client < 0) {
        print("Client out of range: ");
        puthex64(client);
        return microkit_msginfo_new(0, 0);
    }

    // label is the protocol:
    // eg change my ip or register a new one.
    // reg 1 is the ip address. 
    uint32_t ip_addr = microkit_mr_get(0);
    uint32_t mac_lower = microkit_mr_get(1);
    uint32_t mac_higher = microkit_mr_get(2);

    uint8_t mac[8];
    mac[0] = mac_lower >> 24;
    mac[1] = mac_lower >> 16 & 0xff;
    mac[2] = mac_lower >> 8 & 0xff;
    mac[3] = mac_lower & 0xff;
    mac[4] = mac_higher >> 24;
    mac[5] = mac_higher >> 16 & 0xff;
    char buf[16];

    switch (microkit_msginfo_get_label(msginfo)) {
        case REG_IP:
            print("Client registering ip address: ");
            print(print_ipaddr(ip_addr, buf, 16));
            print(" with MAC: ");
            dump_mac(mac);
            print(" client: ");
            put8(client);
            print("\n");
            ipv4_addrs[client] = ip_addr;
            break;
        default:
            print("Unknown request to ARP from client ");
            puthex64(ch);
            print("\n");
            break;
    }

    return microkit_msginfo_new(0, 0);
}

void
init(void)
{
    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, 0, NUM_BUFFERS, NUM_BUFFERS);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, 0, NUM_BUFFERS, NUM_BUFFERS);

    rx_ring.used_ring->notify_reader = true;
    tx_ring.free_ring->notify_reader = true;

    /* Set up hardcoded mac addresses */
    mac_addrs[0][0] = 0x52;
    mac_addrs[0][1] = 0x54;
    mac_addrs[0][2] = 0x1;
    mac_addrs[0][3] = 0;
    mac_addrs[0][4] = 0;
    mac_addrs[0][5] = 0;

    mac_addrs[1][0] = 0x52;
    mac_addrs[1][1] = 0x54;
    mac_addrs[1][2] = 0x1;
    mac_addrs[1][3] = 0;
    mac_addrs[1][4] = 0;
    mac_addrs[1][5] = 1;

    return;
}