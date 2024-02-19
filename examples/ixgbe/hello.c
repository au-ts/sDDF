/*
 * Copyright 2021, Breakaway Consulting Pty. Ltd.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdint.h>
#include <microkit.h>

char *r0;
char *r1 = (char *)0x100000000;

void
init(void)
{
    for (int i = 0; i < 0x4000; i++) {
	if (r1[i] != 0) {
	    microkit_dbg_puts("found non-zero\n");
	}
    }
    microkit_dbg_puts("hello, world\n");
}

void
notified(microkit_channel ch)
{
    microkit_dbg_puts("interrupt: ");
    microkit_dbg_putc('0' + ch);
    microkit_dbg_putc('\n');
}
