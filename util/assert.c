/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sddf/util/printf.h>

void _assert_fail(const char  *assertion, const char  *file, unsigned int line, const char  *function)
{
    sddf_dprintf("Failed assertion '%s' at %s:%u in function %s\n", assertion, file, line, function);
    while (1) {}
}