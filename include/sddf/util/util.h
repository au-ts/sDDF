/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stddef.h>
#include <microkit.h>
#include <sddf/util/printf.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#ifdef __GNUC__
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x)   (!!(x))
#define unlikely(x) (!!(x))
#endif

#ifndef BYTE_ORDER
#if defined(__BYTE_ORDER__)
#  define BYTE_ORDER __BYTE_ORDER__
#elif defined(__BIG_ENDIAN)
#  define BYTE_ORDER BIG_ENDIAN
#elif defined(__LITTLE_ENDIAN)
#  define BYTE_ORDER LITTLE_ENDIAN
#else
#  error Unable to determine system endianness
#endif
#endif

#define ROUND_UP(n,d) (d*(n/d + (n % d == 0 ? 0 : 1)))

static void _assert_fail(const char  *assertion, const char  *file, unsigned int line, const char  *function)
{
    sddf_dprintf("Failed assertion '%s' at %s:%u in function %s\n", assertion, file, line, function);
    while (1) {}
}

#ifndef CONFIG_DEBUG_BUILD
#define _unused(x) ((void)(x))
#define assert(expr) _unused(expr)
#else
#define assert(expr) \
    do { \
        if (!(expr)) { \
            _assert_fail(#expr, __FILE__, __LINE__, __FUNCTION__); \
        } \
    } while(0)
#endif