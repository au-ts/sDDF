#
# Copyright 2024, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Include this snippet in your project Makefile to build
# the PL011 UART driver

UART_DRIVER:= ${SDDF}/drivers/serial/arm

uart_driver.elf: uart_driver.o sddf_libutil.a
	$(LD) $(LDFLAGS) $^ $(LIBS) -o $@

uart_driver.o: ${SDDF}/drivers/serial/arm/uart.c
	$(CC) -c $(CFLAGS) -I${UART_DRIVER}/include -o $@ $< 

-include uart_driver.d

include ${SDDF}/util/util.mk
clean::
	rm -f uart_driver.[do]
