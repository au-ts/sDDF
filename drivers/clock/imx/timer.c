/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * Very basic timer driver. Currently only permtis
 * a maximum of a single timeout per client for simplicity.
 *
 * Interfaces for clients:
 * microkit_ppcall() with label 0 is a request to get the current time.
 * with a 1 is a request to set a timeout.
 */

#include <stdint.h>
#include <microkit.h>
#include <sddf/util/printf.h>

#define GET_TIME 0
#define SET_TIMEOUT 1

#define GPT_STATUS_REGISTER_CLEAR 0x3F
#define CR 0
#define PR 1
#define SR 2
#define IR 3
#define OCR1 4
#define OCR2 5
#define OCR3 6
#define ICR1 7
#define ICR2 8
#define CNT 9

#define MAX_TIMEOUTS 6

#define IRQ_CH 0

#define GPT_FREQ   (12u)
#define NS_IN_US    1000ULL

uintptr_t gpt_regs;
static volatile uint32_t *gpt;

static uint32_t overflow_count;

static uint64_t timeouts[MAX_TIMEOUTS];
static microkit_channel active_channel = -1;
static bool timeout_active = false;
static uint64_t current_timeout;
static uint8_t pending_timeouts;


static uint64_t get_ticks(void) 
{
    uint64_t overflow = overflow_count;
    uint32_t sr1 = gpt[SR];
    uint32_t cnt = gpt[CNT];
    uint32_t sr2 = gpt[SR];
    if ((sr2 & (1 << 5)) && (!(sr1 & (1 << 5)))) {
        /* rolled-over during - 64-bit time must be the overflow */
        cnt = gpt[CNT];
        overflow++;
    }
    return (overflow << 32) | cnt;
}

void irq(microkit_channel ch)
{
    uint32_t sr = gpt[SR];
    gpt[SR] = sr;

    if (sr & (1 << 5)) {
        overflow_count++;
    }

    if (sr & 1) {
        gpt[IR] &= ~1;
        timeout_active = false;
        microkit_channel curr_channel = active_channel;
        timeouts[curr_channel] = 0;
        // notify the client.
        microkit_notify(curr_channel);
    }

    if (pending_timeouts && !timeout_active) {
        uint64_t curr_time = get_ticks();
        /* find next timeout */
        uint64_t next_timeout = UINT64_MAX;
        microkit_channel ch = -1;
        for (unsigned i = 0; i < MAX_TIMEOUTS; i++) {
            if (timeouts[i] != 0 && timeouts[i] <= curr_time) {
                timeouts[i] = 0;
                pending_timeouts--;
                microkit_notify(i);
            } else if (timeouts[i] != 0 && timeouts[i] < next_timeout) {
                next_timeout = timeouts[i];
                ch = i;
            }
        }

        if (ch != -1 && overflow_count == (next_timeout >> 32)) {
            pending_timeouts--;
            gpt[OCR1] = (uint32_t)next_timeout;
            gpt[IR] |= 1;
            timeout_active = true;
            current_timeout = next_timeout;
            active_channel = ch;
        }
    }
}

void notified(microkit_channel ch)
{
    if (ch == IRQ_CH) {
        irq(ch);
        microkit_irq_ack_delayed(ch);
    }
}

seL4_MessageInfo_t protected(microkit_channel ch, microkit_msginfo msginfo)
{
    uint64_t rel_timeout, cur_ticks, abs_timeout;
    switch (microkit_msginfo_get_label(msginfo)) {
        case GET_TIME:
            // Just wants the time.
            cur_ticks = (get_ticks() / (uint64_t)GPT_FREQ);
            // Convert to nanoseconds.
            cur_ticks *= NS_IN_US;
            seL4_SetMR(0, cur_ticks);
            return microkit_msginfo_new(0, 1);
        case SET_TIMEOUT:
            // Request to set a timeout.
            rel_timeout = (seL4_GetMR(0));
            // Convert to microseconds.
            rel_timeout /= NS_IN_US;
            rel_timeout = (uint64_t)(GPT_FREQ) * rel_timeout;
            cur_ticks = get_ticks();
            abs_timeout = cur_ticks + rel_timeout;

            timeouts[ch] = abs_timeout;
            if ((!timeout_active || abs_timeout < current_timeout)
                && (cur_ticks >> 32 == abs_timeout >> 32)) {
                if (timeout_active) {
                    /* there current timeout is now treated as pending */
                    pending_timeouts++;
                }
                gpt[OCR1] = (uint32_t)abs_timeout;
                gpt[IR] |= 1;
                timeout_active = true;
                current_timeout = abs_timeout;
                active_channel = ch;
            } else {
                pending_timeouts++;
            }
            break;
        default:
            sddf_dprintf("TIMER DRIVER|LOG: Unknown request %llu to timer from channel %llu\n", microkit_msginfo_get_label(msginfo), ch);
            break;
    }

    return microkit_msginfo_new(0, 0);
}

void init(void)
{
    gpt = (volatile uint32_t *) gpt_regs;

    /* Disable GPT. */
    gpt[CR] = 0;
    gpt[SR] = GPT_STATUS_REGISTER_CLEAR;

    /* Configure GPT. */
    gpt[CR] = 0 | (1 << 15); /* Reset the GPT */
    /* SWR will be 0 when the reset is done */
    while (gpt[CR] & (1 << 15));

    uint32_t cr = (
        (1 << 9) | // Free run mode
        (1 << 6) | // Peripheral clocks
        (1) // Enable
    );

    gpt[CR] = cr;

    gpt[IR] = (
        (1 << 5) // rollover interrupt
    );

    gpt[PR] = 1; // prescaler. 

    /* Now go passive! */
}