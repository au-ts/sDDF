#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <string.h>

#include <sddf/network/shared_ringbuffer.h>
#include <sddf/util/util.h>
#include <sddf/util/fence.h>
#include <sddf/network/constants.h>

uintptr_t rx_free_drv;
uintptr_t rx_used_drv;

uintptr_t rx_free_cli0;
uintptr_t rx_used_cli0;
uintptr_t rx_free_cli1;
uintptr_t rx_used_cli1;
uintptr_t rx_free_arp;
uintptr_t rx_used_arp;

uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;
uintptr_t uart_base;

#define NUM_CLIENTS 3
#define DMA_SIZE 0x200000
#define DRIVER_CH 3

#define ETHER_MTU 1500

#define BUF_SIZE 2048
#define NUM_BUFFERS 512
#ifndef MAC_BASE_ADDRESS
#  define MAC_BASE_ADDRESS (0x525401000000ULL)
#endif

#define _unused(x) ((void)(x))

typedef struct state {
    /* Pointers to shared buffers */
    ring_handle_t rx_ring_drv;
    ring_handle_t rx_ring_clients[NUM_CLIENTS];
    uint8_t mac_addrs[NUM_CLIENTS][6];
} state_t;

state_t state;
uint64_t dropped = 0;

static uintptr_t
get_phys_addr(uintptr_t virtual)
{
    uint64_t offset = -1;
    if (virtual >= shared_dma_vaddr && virtual < shared_dma_vaddr + DMA_SIZE) {
        offset = virtual - shared_dma_vaddr;
    }

    if (offset < 0 || offset >= DMA_SIZE) {
        print("get_phys_addr: offset < 0 || offset >= dma size\n");
        return 0;
    }

    return shared_dma_paddr + offset;
}

static uintptr_t
get_virt_addr(uintptr_t phys)
{
    uint64_t offset = -1;
    if (phys >= shared_dma_paddr && phys < shared_dma_paddr + DMA_SIZE) {
        offset = phys - shared_dma_paddr;
    }

    if (offset < 0 || offset >= DMA_SIZE) {
        print("get_phys_addr: offset < 0 || offset >= dma size\n");
        return 0;
    }

    return shared_dma_vaddr + offset;
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
    putC('\n');
}

int compare_mac(uint8_t *mac1, uint8_t *mac2)
{
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return -1;
        }
    }
    return 0;
}

/* Return the client ID if the Mac address is a match. */
int get_client(uintptr_t dma_vaddr) {
    struct ethernet_header *ethhdr = (struct ethernet_header *)dma_vaddr;
    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (compare_mac(ethhdr->dest.addr, state.mac_addrs[client]) == 0) {
            return client;
        }
    }

    /* Packet isn't for us */
    return -1;
}

/*
 * Loop over driver and insert all used rx buffers to appropriate client queues.
 */
void process_rx_complete(void)
{
    bool notify_clients[NUM_CLIENTS] = {false};
    /* To avoid notifying the driver twice, used this global variable to 
        determine whether we need to notify the driver in 
        process_rx_free() as we dropped some packets */
    dropped = 0;
    process_rx_complete_:
    while (!ring_empty(state.rx_ring_drv.used_ring)) {
        uintptr_t addr = 0;
        unsigned int len = 0;
        void *cookie = NULL;

        int err = dequeue_used(&state.rx_ring_drv, &addr, &len, &cookie);
        assert(!err);

        int vaddr = get_virt_addr(addr);
        if (!vaddr) {
            print("MUX RX|ERROR: get_virt_addr returned 0\nPhys: ");
            puthex64(addr);
            print("\n");
        }
        err = seL4_ARM_VSpace_Invalidate_Data(3, vaddr, vaddr + ETHER_MTU);
        if (err) {
            print("MUX RX|ERROR: ARM Vspace invalidate failed\n");
            puthex64(err);
            print("\n");
        }

        // Get MAC address and work out which client it is.
        int client = get_client(vaddr);
        if (client >= 0 && !ring_full(state.rx_ring_clients[client].used_ring)) {
            /* enqueue it. */
            int err = enqueue_used(&state.rx_ring_clients[client], vaddr, len, cookie);
            if (err) {
                print("MUX RX|ERROR: failed to enqueue onto used ring\n");
            }

            state.rx_ring_clients[client].used_ring->notify_reader = true;
            if (state.rx_ring_clients[client].used_ring->notify_reader) {
                notify_clients[client] = true;
            }
        } else {
            // either the packet is not for us, or the client queue is full.
            // return the buffer to the driver.
            err = enqueue_free(&state.rx_ring_drv, addr, len, cookie);
            if (err) {
                print("MUX RX|ERROR: Failed to enqueue free to driver RX ring\n");
            }
            dropped++;
        }
    }

    state.rx_ring_drv.used_ring->notify_reader = true;

    THREAD_MEMORY_FENCE();

    if (!ring_empty(state.rx_ring_drv.used_ring)) {
        state.rx_ring_drv.used_ring->notify_reader = false;
        goto process_rx_complete_;
    }


    /* Loop over bitmap and see who we need to notify. */
    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (notify_clients[client]) {
            state.rx_ring_clients[client].used_ring->notify_reader = false;
            microkit_notify(client);
        }
    }    
}

// Loop over all client rings and return unused rx buffers to the driver
bool process_rx_free(void)
{
    /* If we have enqueued to the driver's free ring and the free
     * ring was empty, we want to notify the driver. We also only want to
     * notify it only once.
     */
    bool enqueued = false;
    for (int i = 0; i < NUM_CLIENTS; i++) {
        process_rx_free_:
        while (!ring_empty(state.rx_ring_clients[i].free_ring) && !ring_full(state.rx_ring_drv.free_ring)) {
            uintptr_t addr = 0;
            unsigned int len = 0;
            void *buffer = NULL;
            int err = dequeue_free(&state.rx_ring_clients[i], &addr, &len, &buffer);
            assert(!err);
            _unused(err);

            int paddr = get_phys_addr(addr);
            if (!paddr) {
                print("MUX RX|ERROR: get_phys_addr returned 0\nvirt: ");
                puthex64(addr);
                print("\n");
            }

            err = enqueue_free(&state.rx_ring_drv, paddr, len, buffer);
            assert(!err);
            enqueued = true;
        }

        state.rx_ring_clients[i].free_ring->notify_reader = true;

        THREAD_MEMORY_FENCE();

        if (!ring_empty(state.rx_ring_clients[i].free_ring) && !ring_full(state.rx_ring_drv.free_ring)) {
            state.rx_ring_clients[i].free_ring->notify_reader = false;
            goto process_rx_free_;
        }
    }

    /* We only want to notify the driver if the queue either was empty, or
       it wasn't empty, but the driver interrupted us above and emptied it, 
       and thus the number of packets we enqueued does not equal the ring_size now 
       (So the driver could have missed an empty to full ntfn)
       
       We also could have enqueued packets into the free ring during 
       process_rx_complete(), so we could have also missed this empty condition.
       */
    if ((enqueued || dropped) && state.rx_ring_drv.free_ring->notify_reader) {
        state.rx_ring_drv.free_ring->notify_reader = false;
        microkit_notify_delayed(DRIVER_CH);
    }

    return enqueued;
}

void notified(microkit_channel ch)
{
    process_rx_complete();
    process_rx_free();
}

void init(void)
{
    // set up client macs FIXME this should be done by the clients
    int i;
    int j;
    for (j = 0; j < 2; j++) {
        for (i = 5; i >= 0; --i) {
            state.mac_addrs[j][5 - i] = 0xff & (MAC_BASE_ADDRESS >> (i*8));
        }
        state.mac_addrs[j][5] += 10 + j;
    }
    // and for broadcast. 
    state.mac_addrs[2][0] = 0xff;
    state.mac_addrs[2][1] = 0xff;
    state.mac_addrs[2][2] = 0xff;
    state.mac_addrs[2][3] = 0xff;
    state.mac_addrs[2][4] = 0xff;
    state.mac_addrs[2][5] = 0xff;
    // This is the legitimate hw address for imx8mm
    // (can be useful when debugging). 
    /*state.mac_addrs[0][0] = 0;
    state.mac_addrs[0][1] = 0x4;
    state.mac_addrs[0][2] = 0x9f;
    state.mac_addrs[0][3] = 0x5;
    state.mac_addrs[0][4] = 0xf8;
    state.mac_addrs[0][5] = 0xcc;*/

    /* Set up shared memory regions */
    ring_init(&state.rx_ring_drv, (ring_buffer_t *)rx_free_drv, (ring_buffer_t *)rx_used_drv, 1, NUM_BUFFERS, NUM_BUFFERS);

    ring_init(&state.rx_ring_clients[0], (ring_buffer_t *)rx_free_cli0, (ring_buffer_t *)rx_used_cli0, 1, NUM_BUFFERS, NUM_BUFFERS);
    ring_init(&state.rx_ring_clients[1], (ring_buffer_t *)rx_free_cli1, (ring_buffer_t *)rx_used_cli1, 1, NUM_BUFFERS, NUM_BUFFERS);
    ring_init(&state.rx_ring_clients[2], (ring_buffer_t *)rx_free_arp, (ring_buffer_t *)rx_used_arp, 1, NUM_BUFFERS, NUM_BUFFERS);

    /* Enqueue free buffers for the driver to access */
    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        uintptr_t addr = shared_dma_paddr + (BUF_SIZE * i);
        int err = enqueue_free(&state.rx_ring_drv, addr, BUF_SIZE, NULL);
        assert(!err);
        _unused(err);
    }
    // ensure we get a notification when a packet comes in
    state.rx_ring_drv.used_ring->notify_reader = true;
    // Notify the driver that we are ready to receive
    microkit_notify(DRIVER_CH);

    return;
}
