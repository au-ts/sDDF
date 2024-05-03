#pragma once

#include <sddf/util/util.h>

#if BYTE_ORDER == BIG_ENDIAN
#define HTONS(x) ((uint16_t)(x))
#else
#define HTONS(x) ((uint16_t)((((x) & (uint16_t)0x00ffU) << 8) | (((x) & (uint16_t)0xff00U) >> 8)))
#endif

// @jade: When we have libc implementation replace this
static void *memcpy(void *restrict dest, const void *restrict src, size_t n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    for (; n; n--) *d++ = *s++;
    return dest;
}
