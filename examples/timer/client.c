/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <microkit.h>
#include <sddf/timer/client.h>

#define TIMER_CHANNEL 1

static void
put64(uint64_t val) {
    char buffer[20];
    unsigned i = 19;
    buffer[19] = 0;
    do {
        uint8_t c = val % 10;
        buffer[--i] = '0' + c;
        val /= 10;
    } while (val);
    microkit_dbg_puts(&buffer[i]);
}

void
notified(microkit_channel ch)
{
    /*
     * In this example we only have a single possible channel,
     * so we know it's a notification from the driver telling
     * us that the timeout we set has gone off.
     */
    microkit_dbg_puts("CLIENT|INFO: Got a timeout!\n");
    /* Get the current time */
    uint64_t time = sddf_timer_time_now(TIMER_CHANNEL);
    microkit_dbg_puts("CLIENT|INFO: Now the time (in nanoseconds) is: ");
    put64(time);
    microkit_dbg_puts("\n");
    /* Set another timeout */
    sddf_timer_set_timeout(TIMER_CHANNEL, NS_IN_S);
}

void
init(void)
{
    // lets get the time!
    uint64_t time = sddf_timer_time_now(TIMER_CHANNEL);
    microkit_dbg_puts("CLIENT|INFO: The time now is: ");
    put64(time);
    microkit_dbg_puts("\n");

    // lets set a timeout
    microkit_dbg_puts("CLIENT|INFO: Setting a time out for 1 second\n");
    sddf_timer_set_timeout(TIMER_CHANNEL, NS_IN_S);
}
