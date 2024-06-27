/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <microkit.h>
#include <sddf/util/printf.h>
#include <sddf/timer/protocol.h>

uintptr_t gpt_regs;

#define IRQ_CH 0
#define MAX_TIMEOUTS 6

#define TIMER_REG_START   0x140    // TIMER_MUX

#define TIMER_A_INPUT_CLK 0
#define TIMER_E_INPUT_CLK 8
#define TIMER_A_EN      (1 << 16)
#define TIMER_A_MODE    (1 << 12)

#define TIMESTAMP_TIMEBASE_SYSTEM   0b000
#define TIMESTAMP_TIMEBASE_1_US     0b001
#define TIMESTAMP_TIMEBASE_10_US    0b010
#define TIMESTAMP_TIMEBASE_100_US   0b011
#define TIMESTAMP_TIMEBASE_1_MS     0b100

#define TIMEOUT_TIMEBASE_1_US   0b00
#define TIMEOUT_TIMEBASE_10_US  0b01
#define TIMEOUT_TIMEBASE_100_US 0b10
#define TIMEOUT_TIMEBASE_1_MS   0b11

struct timer_regs {
    uint32_t mux;
    uint32_t timer_a;
    uint32_t timer_b;
    uint32_t timer_c;
    uint32_t timer_d;
    uint32_t unused[13];
    uint32_t timer_e;
    uint32_t timer_e_hi;
    uint32_t mux1;
    uint32_t timer_f;
    uint32_t timer_g;
    uint32_t timer_h;
    uint32_t timer_i;
};

volatile struct timer_regs *regs;

/* Right now, we only service a single timeout per client.
 * This timeout array indicates when a timeout should occur,
 * indexed by client ID. */
static uint64_t timeouts[MAX_TIMEOUTS];

static uint64_t get_ticks(void)
{
    uint64_t initial_high = regs->timer_e_hi;
    uint64_t low = regs->timer_e;
    uint64_t high = regs->timer_e_hi;
    if (high != initial_high) {
        low = regs->timer_e;
    }

    uint64_t ticks = (high << 32) | low;
    return ticks;
}

static void process_timeouts(uint64_t curr_time)
{
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        if (timeouts[i] <= curr_time) {
            microkit_notify(i);
            timeouts[i] = UINT64_MAX;
        }
    }

    uint64_t next_timeout = UINT64_MAX;
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        if (timeouts[i] < next_timeout) {
            next_timeout = timeouts[i];
        }
    }

    if (next_timeout != UINT64_MAX) {
        regs->mux &= ~TIMER_A_MODE;
        regs->timer_a = next_timeout - curr_time;
        regs->mux |= TIMER_A_EN;
    }
}

void notified(microkit_channel ch)
{
    if (ch != IRQ_CH) {
        sddf_dprintf("TIMER DRIVER|LOG: unexpected notification from channel %u\n", ch);
        return;
    }

    microkit_irq_ack_delayed(ch);
    regs->mux &= ~TIMER_A_EN;

    uint64_t curr_time = get_ticks();
    process_timeouts(curr_time);
}

seL4_MessageInfo_t protected(microkit_channel ch, microkit_msginfo msginfo)
{
    switch (microkit_msginfo_get_label(msginfo)) {
    case SDDF_TIMER_GET_TIME: {
        uint64_t time_ns = get_ticks() * NS_IN_US;
        seL4_SetMR(0, time_ns);
        return microkit_msginfo_new(0, 1);
    }
    case SDDF_TIMER_SET_TIMEOUT: {
        uint64_t curr_time = get_ticks();
        uint64_t offset_us = seL4_GetMR(0) / NS_IN_US;
        timeouts[ch] = curr_time + offset_us;
        process_timeouts(curr_time);
        break;
    }
    default:
        sddf_dprintf("TIMER DRIVER|LOG: Unknown request %lu to timer from channel %u\n", microkit_msginfo_get_label(msginfo),
                     ch);
        break;
    }

    return microkit_msginfo_new(0, 0);
}

void init(void)
{
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        timeouts[i] = UINT64_MAX;
    }

    regs = (void *)(gpt_regs + TIMER_REG_START);

    /* Start timer E acts as a clock, while timer A can be used for timeouts from clients */
    regs->mux = TIMER_A_EN | (TIMESTAMP_TIMEBASE_1_US << TIMER_E_INPUT_CLK) |
                (TIMEOUT_TIMEBASE_1_US << TIMER_A_INPUT_CLK);

    regs->timer_e = 0;
}
