/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stddef.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif
#define ALIGN(x, align)   (((x) + (align) - 1) & ~((align) - 1))

#define BIT(nr) (1UL << (nr))

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

// Doing this with statement expressions & temporary variables to avoid issues
// with operator precedence and evaluating arguments multiple times.
#define ROUND_UP(n,d) \
    ({ typeof (n) _n = (n); \
       typeof (d) _d = (d); \
       _d * (_n/_d + (_n % _d == 0 ? 0 : 1)); \
    })
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

void _assert_fail(const char  *assertion, const char  *file, unsigned int line, const char  *function);

#ifndef assert
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
#endif

static inline int sddf_isspace(int ch)
{
    return ch == ' ' || ch == '\f' || ch == '\n' || ch == '\r' || ch == '\t' || ch == '\v';
}

static inline int sddf_isdigit(int ch)
{
    return ch >= '0' && ch <= '9';
}

static inline int sddf_atoi(const char *str)
{
    while (sddf_isspace(*str)) {
        str++;
    }

    int sign = 1;
    if (*str == '+') {
        str++;
    } else if (*str == '-') {
        sign = -1;
        str++;
    }

    int result = 0;
    while (sddf_isdigit(*str)) {
        int digit = *str - '0';
        result *= 10;
        result -= digit;
        str++;
    }

    return sign * (-result);
}
