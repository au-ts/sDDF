/* The policy of the mux tx is that all clients can always request to transmit */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sel4/sel4.h>
#include "sddf_serial_shared_ringbuffer.h"
#include "util/include/util.h"
#include "util.h"
#include "uart.h"

#define DRIVER_CH 9

#ifndef NUM_CLIENTS
#error "NUM_CLIENTS is expected to be defined for TX serial multiplexor"
#endif

#define COLOUR_START_LEN 5
#define COLOUR_END_LEN 4

#define MAX_NUM_CLIENTS 2

/* This is a simple sanity check that we have defined as many colours
   as the user as specified clients */
#if NUM_CLIENTS > MAX_NUM_CLIENTS && defined(TRANSFER_WITH_COLOUR)
#error "There are more clients then there are colours to differentiate them"
#endif

char *client_colours[MAX_NUM_CLIENTS] = { "\x1b[31m", "\x1b[32m" };
char *client_colour_end = "\x1b[0m";

/* Memory regions as defined in the system file */

// Transmit rings with the driver
uintptr_t tx_free_driver;
uintptr_t tx_used_driver;

// Transmit rings with the client
uintptr_t tx_free_client;
uintptr_t tx_used_client;
uintptr_t tx_free_client2;
uintptr_t tx_used_client2;

uintptr_t tx_data_driver;
uintptr_t tx_data_client;
uintptr_t tx_data_client2;

// Have an array of client rings.
sddf_serial_ring_handle_t tx_ring[NUM_CLIENTS];
sddf_serial_ring_handle_t drv_tx_ring;

size_t copy_with_colour(size_t client, size_t buffer_len, char *driver_buf, char *client_buf) {
    size_t len_copied = 0;
    assert(buffer_len + COLOUR_START_LEN + COLOUR_START_LEN <= SDDF_SERIAL_BUFFER_SIZE);
    /* First copy the colour start bytes */
    memcpy((char *) driver_buf, client_colours[client], COLOUR_START_LEN);
    len_copied += COLOUR_START_LEN;
    /* Copy the actual TX data */
    char *dest_buffer = driver_buf + COLOUR_START_LEN;
    memcpy(dest_buffer, client_buf, buffer_len);
    len_copied += buffer_len;
    /* And then finally the colour end bytes */
    memcpy((char *)(dest_buffer + buffer_len), client_colour_end, COLOUR_END_LEN);
    len_copied += COLOUR_END_LEN;

    return len_copied;
}

size_t copy_normal(size_t buffer_len, char *driver_buf, char *client_buf) {
    /* All we need to do is copy the client buffer into the driver buffer. */
    memcpy(driver_buf, client_buf, buffer_len);

    return buffer_len;
}

int handle_tx(int curr_client) {
    // Copy data from the client ring to the driver rings.
    uintptr_t client_buf = 0;
    unsigned int client_buf_len = 0;
    void *cookie = 0;

    bool was_empty = sddf_serial_ring_empty(drv_tx_ring.used_ring);

    // Loop over all clients here
    for (int client = 0; client < NUM_CLIENTS; client++) {
        // The client can plug their ring. If plugged, we won't process it.
        if (sddf_serial_ring_plugged(tx_ring[client].used_ring)) {
            continue;
        }
        while (!sddf_serial_dequeue_used(&tx_ring[client], &client_buf, &client_buf_len, &cookie)) {
            // We want to enqueue into the drivers used ring
            uintptr_t driver_buf = 0;
            unsigned int driver_buf_len = 0;
            void *drv_cookie = 0;

            int ret = sddf_serial_driver_dequeue(drv_tx_ring.free_ring, &driver_buf, &driver_buf_len, &drv_cookie);
            if (ret != 0) {
                microkit_dbg_puts("Failed to dequeue buffer from drv tx avail ring\n");
                return 1;
            }

            /*
             * Depending on the overall system, we may want to add colour to
             * the transmit data in order to easily identify which client the
             * data belongs to when looking at the serial output. This makes
             * the most sense when you have, for example, multiple client
             * consoles being multiplexed. However, not all systems will act
             * like that even if they have multiple clients, which is why this
             * is configurable behaviour.
             */
#ifdef TRANSFER_WITH_COLOUR
            size_t len_copied = copy_with_colour(client, client_buf_len, (char *)driver_buf, (char *)client_buf);
#else
            size_t len_copied = copy_normal(client_buf_len, (char *)driver_buf, (char *)client_buf);
#endif

            drv_cookie = cookie;

            ret = sddf_serial_enqueue_used(&drv_tx_ring, driver_buf, len_copied, drv_cookie);
            if (ret != 0) {
                microkit_dbg_puts("Failed to enqueue buffer to drv tx used ring\n");
                // Don't know if I should return here, because we need to enqueue a
                // serpeate buffer
            }

            /*
             * Now that we've finished processing the client's used buffer, we
             * can put it back in the free ring. Note that the *whole* buffer is
             * free now, which is why the length we enqueue is the maximum
             * buffer length.
             */
            sddf_serial_enqueue_free(&tx_ring[client], client_buf, SDDF_SERIAL_BUFFER_SIZE, cookie);
        }
    }

    if (was_empty) {
        microkit_notify(DRIVER_CH);
    }

    return 0;
}

void init (void) {
    // We want to init the client rings here. Currently this only inits one client
    sddf_serial_ring_init(&tx_ring[0], (sddf_serial_ring_buffer_t *)tx_free_client, (sddf_serial_ring_buffer_t *)tx_used_client, 0, SDDF_SERIAL_NUM_BUFFERS, SDDF_SERIAL_NUM_BUFFERS);
    sddf_serial_ring_init(&tx_ring[1], (sddf_serial_ring_buffer_t *)tx_free_client2, (sddf_serial_ring_buffer_t *)tx_used_client2, 0, SDDF_SERIAL_NUM_BUFFERS, SDDF_SERIAL_NUM_BUFFERS);
    sddf_serial_ring_init(&drv_tx_ring, (sddf_serial_ring_buffer_t *)tx_free_driver, (sddf_serial_ring_buffer_t *)tx_used_driver, 0, SDDF_SERIAL_NUM_BUFFERS, SDDF_SERIAL_NUM_BUFFERS);

    // Add buffers to the drv tx ring from our shared dma region
    for (int i = 0; i < SDDF_SERIAL_NUM_BUFFERS - 1; i++) {
        // Have to start at the memory region left of by the rx ring
        int ret = sddf_serial_enqueue_free(&drv_tx_ring, tx_data_driver + ((i + SDDF_SERIAL_NUM_BUFFERS) * SDDF_SERIAL_BUFFER_SIZE), SDDF_SERIAL_BUFFER_SIZE, NULL);

        if (ret != 0) {
            microkit_dbg_puts(microkit_name);
            microkit_dbg_puts(": tx buffer population, unable to enqueue buffer\n");
        }
    }
}

void notified(microkit_channel ch) {
    // We should only ever recieve notifications from the client
    // Sanity check the client
    if (ch < 1 || ch > NUM_CLIENTS) {
        microkit_dbg_puts("Received a bad client channel\n");
        return;
    }

    handle_tx(ch);
}
