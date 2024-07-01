/*
 * Very simple string.h that punts everything to compiler builtins.
 * This is what usually happens for -O3, but we want it for everything.
 *
 * Copyright 2024 UNSW, Sydney
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once
#include <stddef.h>
#include <stdint.h>

#ifndef __has_builtin
#  define __has_builtin(x) 0
#endif

static inline void *sddf_memset(void *s, int c, size_t n)
{
    unsigned char *p = s;
    while (n-- > 0) {
        *p++ = c;
    }
    return s;
}

static inline void *sddf_memcpy(void *dest, const void *src, size_t n)
{
    unsigned char *to = dest;
    const unsigned char *from = src;
    while (n-- > 0) {
        *to++ = *from++;
    }
    return dest;
}

static inline char *sddf_strncpy(char *dest, const char *restrict src,
                                 size_t dsize)
{
    char *to = dest;
    while (dsize-- && (*to = *src++)) {
        to++;
    }
    while (dsize--) {
        *to++ = '\0';
    }
    return dest;
}

static inline int sddf_strcmp(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0' && *a == *b) {
        a++;
        b++;
    }
    return (int)(*a) - *b;
}

static inline int sddf_strncmp(const char *a, const char *b, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (a[i] == '\0' || b[i] == '\0' || a[i] != b[i]) {
            return (int)a[i] - b[i];
        }
    }
    return 0;
}

static inline char *sddf_strchr(const char *s, int c)
{
    while (*s != '\0') {
        if (*s == c) {
            return (char *)s;
        }
        s++;
    }
    if (c == '\0') {
        return (char *)s;
    }
    return NULL;
}

static inline int sddf_memcmp(const void *a, const void *b, size_t n)
{
    const unsigned char *_a = a;
    const unsigned char *_b = b;
    for (size_t i = 0; i < n; i++) {
        if (_a[i] != _b[i]) {
            return (int)_a[i] - (int)_b[i];
        }
    }
    return 0;
}


static inline size_t sddf_strlen(const char *s)
{
    const char *_s = s;
    while (*_s != '\0') {
        _s++;
    }
    return (size_t)(_s - s);
}


static inline void *sddf_memmove(void *dest, const void *src, size_t n)
{
    if (dest == src) {
        return dest;
    }
    int copy_backwards = dest > src;
    for (size_t i = 0; i < n; i++) {
        if (copy_backwards) {
            ((uint8_t *)dest)[n - 1 - i] = ((uint8_t *)src)[n - 1 - i];
        } else {
            ((uint8_t *)dest)[i] = ((uint8_t *)src)[i];
        }
    }
    return dest;
}
