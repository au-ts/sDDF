/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <microkit.h>
#include <stdint.h>
#include <sddf/timer/protocol.h>

/**
 * Request a timeout via PPC into the passive timer driver.
 * Use the label to indicate this request.
 * @param microkit channel of timer driver.
 * @param timeout relative timeout in nanoseconds.
 */
static inline void sddf_timer_set_timeout(microkit_channel channel, uint64_t timeout)
{
    microkit_mr_set(0, timeout);
    microkit_ppcall(channel, microkit_msginfo_new(SDDF_TIMER_SET_TIMEOUT, 1));
}

/**
 * Request the time since start up via PPC into the passive timer driver.
 * Use the label to indicate this request.
 * @param microkit channel of timer driver.
 * @return the time in nanoseconds since start up.
 */
static inline uint64_t sddf_timer_time_now(microkit_channel channel)
{
    microkit_ppcall(channel, microkit_msginfo_new(SDDF_TIMER_GET_TIME, 0));
    uint64_t time_now = seL4_GetMR(0);
    return time_now;
}
