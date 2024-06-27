/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <microkit.h>
#include <sddf/timer/protocol.h>
#include <sddf/util/util.h>
#include <sddf/util/printf.h>

#if !(CONFIG_EXPORT_PCNT_USER && CONFIG_EXPORT_PTMR_USER)
#error "ARM generic timer is not exported by seL4"
#endif

static uint64_t timer_freq;

#define IRQ_CH 0
#define MAX_TIMEOUTS 6

#define GENERIC_TIMER_ENABLE (1 << 0)
#define GENERIC_TIMER_IMASK  (1 << 1)
#define GENERIC_TIMER_STATUS (1 << 2)

#define COPROC_WRITE_WORD(R,W) asm volatile ("msr " R  ", %0" :: "r"(W))
#define COPROC_READ_WORD(R,W)  asm volatile ("mrs %0, " R : "=r" (W))
#define COPROC_WRITE_64(R,W)   COPROC_WRITE_WORD(R,W)
#define COPROC_READ_64(R,W)    COPROC_READ_WORD(R,W)

/* control reigster for the el1 physical timer */
#define CNTP_CTL "cntp_ctl_el0"
/* holds the compare value for the el1 physical timer */
#define CNTP_CVAL "cntp_cval_el0"
/* holds the 64-bit physical count value */
#define CNTPCT "cntpct_el0"
/* frequency of the timer */
#define CNTFRQ "cntfrq_el0"

static inline uint64_t get_ticks(void)
{
    uint64_t time;
    COPROC_READ_64(CNTPCT, time);
    return time;
}

static inline void generic_timer_set_compare(uint64_t ticks)
{
    COPROC_WRITE_64(CNTP_CVAL, ticks);
}

static inline uint32_t generic_timer_get_freq(void)
{
    uintptr_t freq;
    COPROC_READ_WORD(CNTFRQ, freq);
    return (uint32_t) freq;
}

static inline uint32_t generic_timer_read_ctrl(void)
{
    uintptr_t ctrl;
    COPROC_READ_WORD(CNTP_CTL, ctrl);
    return ctrl;
}

static inline void generic_timer_write_ctrl(uintptr_t ctrl)
{
    COPROC_WRITE_WORD(CNTP_CTL, ctrl);
}

static inline void generic_timer_or_ctrl(uintptr_t bits)
{
    uintptr_t ctrl = generic_timer_read_ctrl();
    generic_timer_write_ctrl(ctrl | bits);
}

static inline void generic_timer_enable(void)
{
    generic_timer_or_ctrl(GENERIC_TIMER_ENABLE);
}

static inline void generic_timer_disable(void)
{
    generic_timer_or_ctrl(~GENERIC_TIMER_ENABLE);
}

#define KHZ (1000)
#define MHZ (1000 * KHZ)
#define GHZ (1000 * MHZ)

static inline uint64_t freq_cycles_and_hz_to_ns(uint64_t ncycles, uint64_t hz)
{
    if (hz % GHZ == 0) {
        return ncycles / (hz / GHZ);
    } else if (hz % MHZ == 0) {
        return ncycles * MS_IN_S / (hz / MHZ);
    } else if (hz % KHZ == 0) {
        return ncycles * US_IN_S / (hz / KHZ);
    }

    return (ncycles * NS_IN_S) / hz;
}

static inline uint64_t freq_ns_and_hz_to_cycles(uint64_t ns, uint64_t hz)
{
    return (ns * hz) / NS_IN_S;
}

void set_timeout(uint64_t timeout)
{
    generic_timer_set_compare(freq_ns_and_hz_to_cycles(timeout, timer_freq));
}

static uint64_t timeouts[MAX_TIMEOUTS];

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
        set_timeout(next_timeout);
    }

}

void init()
{
    for (int i = 0; i < MAX_TIMEOUTS; i++) {
        timeouts[i] = UINT64_MAX;
    }

    generic_timer_set_compare(UINT64_MAX);
    generic_timer_enable();
    timer_freq = generic_timer_get_freq();
}

void notified(microkit_channel ch)
{
    assert(ch == IRQ_CH);
    microkit_irq_ack_delayed(ch);

    generic_timer_set_compare(UINT64_MAX);
    uint64_t curr_time = freq_cycles_and_hz_to_ns(get_ticks(), timer_freq);
    process_timeouts(curr_time);
}

seL4_MessageInfo_t protected(microkit_channel ch, microkit_msginfo msginfo)
{
    switch (microkit_msginfo_get_label(msginfo)) {
    case SDDF_TIMER_GET_TIME: {
        uint64_t time_ns = freq_cycles_and_hz_to_ns(get_ticks(), timer_freq);
        seL4_SetMR(0, time_ns);
        return microkit_msginfo_new(0, 1);
    }
    case SDDF_TIMER_SET_TIMEOUT: {
        uint64_t curr_time = freq_cycles_and_hz_to_ns(get_ticks(), timer_freq);
        uint64_t offset_us = (uint64_t)(seL4_GetMR(0));
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
