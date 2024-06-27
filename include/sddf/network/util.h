/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sddf/util/util.h>

#if BYTE_ORDER == BIG_ENDIAN
#define HTONS(x) ((uint16_t)(x))
#else
#define HTONS(x) ((uint16_t)((((x) & (uint16_t)0x00ffU) << 8) | (((x) & (uint16_t)0xff00U) >> 8)))
#endif
