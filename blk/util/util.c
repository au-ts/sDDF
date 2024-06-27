/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sddf/blk/util.h>

void *memset(void *dest, int c, size_t n)
{
    unsigned char *s = dest;
    for (; n; n--, s++) {
        *s = c;
    }
    return dest;
}

void *memcpy(void *restrict dest, const void *restrict src, size_t n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    for (; n; n--) {
        *d++ = *s++;
    }
    return dest;
}
