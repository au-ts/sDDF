/* MUX is currently limited to a max of 9 clients */
#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/serial/shared_ringbuffer.h>
#include <sddf/serial/util.h>
#include "uart.h"
#include "uart_config.h"

#define CLI_CH 1
#define DRV_CH 11

#ifndef SERIAL_NUM_CLIENTS
#error "SERIAL_NUM_CLIENTS is expected to be defined for RX serial multiplexor"
#endif

/* Memory regions as defined in the system file */

// Transmit rings with the driver
uintptr_t rx_free_driver;
uintptr_t rx_used_driver;

// Transmit rings with the client
uintptr_t rx_free_client;
uintptr_t rx_used_client;
uintptr_t rx_free_client2;
uintptr_t rx_used_client2;

uintptr_t rx_data_driver;
// @ivanv: unused
uintptr_t rx_data_client;
uintptr_t rx_data_client2;

// Have an array of client rings.
ring_handle_t rx_ring[SERIAL_NUM_CLIENTS];
ring_handle_t drv_rx_ring;

/* We need to do some processing of the input stream to determine when we need
to change direction. */

/* To switch input direction, type the "@" symbol followed immediately by a number.
Otherwise, can put "\" before "@" to escape this.*/

int mux_state;
int client;
// We want to keep track of each clients requests, so that they can be serviced once we have changed
// input direction
int num_to_get_chars[SERIAL_NUM_CLIENTS];
int multi_client;

int give_multi_char(char * drv_buffer, int drv_buffer_len) {
    for (int curr_client = 0; curr_client < SERIAL_NUM_CLIENTS; curr_client++) {

        if (num_to_get_chars[curr_client] <= 0) {
            return 1;
        }
        // Address that we will pass to dequeue to store the buffer address
        uintptr_t buffer = 0;
        // Integer to store the length of the buffer
        unsigned int buffer_len = 0;

        void *cookie = 0;

        int ret = dequeue_free(&rx_ring[curr_client], &buffer, &buffer_len, &cookie);

        if (ret != 0) {
            microkit_dbg_puts(microkit_name);
            microkit_dbg_puts(": unable to dequeue from the rx free ring\n");
            return 1;
        }

        memcpy((char *) buffer, drv_buffer, drv_buffer_len);
        buffer_len = drv_buffer_len;

        // Now place in the rx used ring
        ret = enqueue_used(&rx_ring[curr_client], buffer, buffer_len, &cookie);

        if (ret != 0) {
            microkit_dbg_puts(microkit_name);
            microkit_dbg_puts(": unable to enqueue to the rx used ring\n");
            return 1;
        }

        num_to_get_chars[curr_client] -= 1;
    }
    return 0;
}

int give_single_char(int curr_client, char * drv_buffer, int drv_buffer_len) {
    if (curr_client < 1 || curr_client > SERIAL_NUM_CLIENTS) {
        return 1;
    }

    // if (num_to_get_chars[curr_client - 1] <= 0) {
    //     return 1;
    // }
    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0;

    void *cookie = 0;

    int ret = dequeue_free(&rx_ring[curr_client - 1], &buffer, &buffer_len, &cookie);

    if (ret != 0) {
        microkit_dbg_puts(microkit_name);
        microkit_dbg_puts(": unable to dequeue from the rx free ring\n");
        return 1;
    }

    memcpy((char *) buffer, drv_buffer, drv_buffer_len);
    buffer_len = drv_buffer_len;

    // Now place in the rx used ring
    ret = enqueue_used(&rx_ring[curr_client - 1], buffer, buffer_len, &cookie);

    if (ret != 0) {
        microkit_dbg_puts(microkit_name);
        microkit_dbg_puts(": unable to enqueue to the rx used ring\n");
        return 1;
    }

    // num_to_get_chars[curr_client - 1] -= 1;

    microkit_notify(curr_client);

    return 0;
}

int give_char(int curr_client, char * drv_buffer, int drv_buffer_len) {
    if (multi_client == 1) {
        give_multi_char(drv_buffer, drv_buffer_len);
    } else {
        give_single_char(curr_client, drv_buffer, drv_buffer_len);
    }

    return 0;
}

/* We will check for escape characters in here, as well as dealing with switching direction*/
void handle_rx() {
    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0;

    void *cookie = 0;

    // We can only be here if we have been notified by the driver
    int ret = dequeue_used(&drv_rx_ring, &buffer, &buffer_len, &cookie) != 0;
    if (ret != 0) {
        microkit_dbg_puts(microkit_name);
        microkit_dbg_puts(": getchar - unable to dequeue used buffer\n");
    }

    // We can either get a single char here, if driver is in RAW mode, or
    // a buffer if driver is in LINE mode.

    char *chars = (char *) buffer;

    // This is for our RAW mode, char by char
    if (UART_MODE == RAW_MODE) {
        // microkit_dbg_puts("In raw mode mux rx\n");
        char got_char = chars[0];

        // We have now gotten a character, deal with the input direction switch
        if (mux_state == 1) {
            // The previous character was an escape character
            give_char(client, &got_char, 1);
            mux_state = 0;
        }  else if (mux_state == 2) {
            // We are now switching input direction

            // Case for simultaneous multi client input
            if (got_char == 'm') {
                multi_client = 1;
                client = -1;
            } else {
                // Ensure that multi client input is off
                multi_client = 0;
                int new_client = atoi(&got_char);
                if (new_client < 1 || new_client > SERIAL_NUM_CLIENTS) {
                    microkit_dbg_puts("MUX|RX: Attempted to switch to invalid client number: ");
                    puthex64(new_client);
                    microkit_dbg_puts("\n");
                } else {
                    microkit_dbg_puts("MUX|RX: Switching to client number: ");
                    puthex64(new_client);
                    microkit_dbg_puts("\n");
                    client = new_client;
                }
            }

            mux_state = 0;
        } else if (mux_state == 0) {
            // No escape character has been set
            if (got_char == '\\') {
                mux_state = 1;
                // The next character is going to be escaped
            } else if (got_char == '@') {
                mux_state = 2;
            } else {
                give_char(client, &got_char, 1);
            }
        }
    } else if (UART_MODE == LINE_MODE) {
        microkit_dbg_puts("In line mode mux rx\n");
        // This is for LINE mode, placing buffers at a time

        /* Check if the first character is an '@'. The following character
            must be a number. Otherwise, give to the client.
        */

       if (chars[0] == '@') {
            if (chars[1] == 'm') {
                // case for multi client input
                multi_client = 1;
                client = -1;
            } else {
                int new_client = atoi(&chars[1]);
                multi_client = 0;
                client = new_client;
            }
       } else {
            // Otherwise, give entire buffer to the client
            give_char(client, chars, buffer_len);
       }
    }

    /* Now that we are finished with the used buffer, we can add it back to the free ring*/

    ret = enqueue_free(&drv_rx_ring, buffer, BUFFER_SIZE, NULL);

    if (ret != 0) {
        microkit_dbg_puts(microkit_name);
        microkit_dbg_puts(": getchar - unable to enqueue used buffer back into free ring\n");
    }
}

void init (void) {
    // We want to init the client rings here. Currently this only inits one client
    ring_init(&rx_ring[0], (ring_buffer_t *)rx_free_client, (ring_buffer_t *)rx_used_client, 0, NUM_BUFFERS, NUM_BUFFERS);
    // @ivanv: terrible temporary hack
#if SERIAL_NUM_CLIENTS > 1
    ring_init(&rx_ring[1], (ring_buffer_t *)rx_free_client2, (ring_buffer_t *)rx_used_client2, 0, NUM_BUFFERS, NUM_BUFFERS);
#endif

    ring_init(&drv_rx_ring, (ring_buffer_t *)rx_free_driver, (ring_buffer_t *)rx_used_driver, 0, NUM_BUFFERS, NUM_BUFFERS);

    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        int ret = enqueue_free(&drv_rx_ring, rx_data_driver + (i * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            microkit_dbg_puts(microkit_name);
            microkit_dbg_puts(": mux rx buffer population, unable to enqueue buffer\n");
            return;
        }
    }

    // We initialise the current client to 1
    client = 1;
    // Set the current escape character to 0, we can't have recieved an escape character yet
    mux_state = 0;
    // Disable simultaneous multi client input
    multi_client = 0;
    // No chars have been requested yet
    for (int i = 0; i < SERIAL_NUM_CLIENTS; i++) {
        num_to_get_chars[i] = 0;
    }
}

void notified(microkit_channel ch) {
    // We should only ever recieve notifications from the client
    // Sanity check the client
    if (ch == DRV_CH) {
        handle_rx();
    } else if (ch < 1 || ch > SERIAL_NUM_CLIENTS) {
        microkit_dbg_puts("Received a bad client channel\n");
        return;
    }  else {
        // This was recieved on a client channel. Index the number of characters to get
        num_to_get_chars[ch - 1] += 1;
    }
}