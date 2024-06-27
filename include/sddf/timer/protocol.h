/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/* Shared functionality/definitions between timer drivers and clients */

#define SDDF_TIMER_GET_TIME 0
#define SDDF_TIMER_SET_TIMEOUT 1

/* Number of nanoseconds in a second */
#define NS_IN_S  1000000000ULL
/* Number of nanoseconds in a millisecond */
#define NS_IN_MS 1000000ULL
/* Number of nanoseconds in a microsecond */
#define NS_IN_US 1000ULL

/* Number of microseconds in a millisecond */
#define US_IN_MS 1000ULL
/* Number of microseconds in a second */
#define US_IN_S  1000000ULL

/* Number of milliseconds in a second */
#define MS_IN_S  1000ULL
