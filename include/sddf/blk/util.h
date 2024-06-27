/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stddef.h>

// @ericc: When we have libc implementation replace this
/* We need an externally linked memset definition for
   copying structs */
void *memcpy(void *restrict dest, const void *restrict src, size_t n);
void *memset(void *dest, int c, size_t n);