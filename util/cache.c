/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* This is a small utility library for performing manual cache operations on
 * AArch64 from user-level. The primary use-case is for managing regions of
 * memory that are mapped as cached but are accessible by DMA capable devices.
 */

#include <stdint.h>
#include <sddf/util/cache.h>
#include <sddf/util/util.h>

#ifndef CONFIG_AARCH64_USER_CACHE_ENABLE
#error "CONFIG_AARCH64_USER_CACHE_ENABLE must be enabled"
#error "seL4 must be configured with CONFIG_AARCH64_USER_CACHE_ENABLE"
#endif

#define ROUND_DOWN(n, b) (((n) >> (b)) << (b))
#define LINE_START(a) ROUND_DOWN(a, CONFIG_L1_CACHE_LINE_SIZE_BITS)
#define LINE_INDEX(a) (LINE_START(a)>>CONFIG_L1_CACHE_LINE_SIZE_BITS)

static inline void
dsb(void)
{
    asm volatile("dsb sy" ::: "memory");
}

static inline void 
dmb(void)
{
    asm volatile("dmb sy" ::: "memory");
}

static inline void
clean_and_invalidate_by_va(unsigned long vaddr)
{
    asm volatile("dc civac, %0" : : "r"(vaddr));
    dsb();
}

static inline void
clean_by_va(unsigned long vaddr)
{
    asm volatile("dc cvac, %0" : : "r"(vaddr));
    dmb();
}

// Intentionally no invalidate_by_va or cache_invalidate. The ARM instruction
// for that -- `dc ivac` -- does not work in userspace (EL0). See [1].
//
// [1]: https://developer.arm.com/documentation/ddi0595/2021-06/AArch64-Instructions/DC-IVAC--Data-or-unified-Cache-line-Invalidate-by-VA-to-PoC

void
cache_clean_and_invalidate(unsigned long start, unsigned long end)
{
    unsigned long line;
    unsigned long index;
    /* Clean the L1 range */

    /* Finally clean and invalidate the L1 range. The extra clean is only strictly neccessary
     * in a multiprocessor environment to prevent a write being lost if another core is
     * attempting a store at the same time. As the range should already be clean asking
     * it to clean again should not affect performance */
    for (index = LINE_INDEX(start); index < LINE_INDEX(end) + 1; index++) {
        line = index << CONFIG_L1_CACHE_LINE_SIZE_BITS;
        clean_and_invalidate_by_va(line);
    }
}

void
cache_clean(unsigned long start, unsigned long end)
{
    unsigned long line;
    unsigned long index;

    for (index = LINE_INDEX(start); index < LINE_INDEX(end) + 1; index++) {
        line = index << CONFIG_L1_CACHE_LINE_SIZE_BITS;
        clean_by_va(line);
    }
}
