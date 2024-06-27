/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <microkit.h>

/*
 * This source file is intended to be compiled with other code
 * that links with the newlib C library. Newlib expects these
 * function definitions to exist. At the time of writing, we do
 * not depend on any of the OS-level calls and hence leave them
 * unimplemented.
 */

void _close(void)
{
    microkit_dbg_puts("NEWLIB ERROR: calling unimplemented _close()");
}

void _lseek(void)
{
    microkit_dbg_puts("NEWLIB ERROR: calling unimplemented _lseek()");
}

void _read(void)
{
    microkit_dbg_puts("NEWLIB ERROR: calling unimplemented _read()");
}

void _write(void)
{
    microkit_dbg_puts("NEWLIB ERROR: calling unimplemented _write()");
}

void _sbrk(void)
{
    microkit_dbg_puts("NEWLIB ERROR: calling unimplemented _sbrk()");
}
