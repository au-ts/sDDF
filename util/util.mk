# Snippet to build util library
#
# Copyright 2024, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause
#  
# Include this snippet in your project Makefile to build
# sddf_libutil.a

OBJS := cache.o sddf_printf.o newlibc.o


libsddf_util_debug.a: ${OBJS} putchar_debug.o
	ar rv $@ $^

libsddf_util.a: ${OBJS} putchar_serial.o
	ar rv $@ $^

VPATH += ${SDDF}/util

sddf_printf.o: ${SDDF}/util/printf.c
	${CC} ${CFLAGS} -c -o $@ $<

-include ${OBJS:.o=.d} putchar_debug.d putchar_serial.d
