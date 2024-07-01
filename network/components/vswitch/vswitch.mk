#
# Copyright 2024, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause
#
# This Makefile snippet builds the network components
# (for example, simple RX and TX virtualisers)
# it should be included into your project Makefile
#
# NOTES:
# Generates vswitch.elf
# Relies on the variable NUM_VSWITCH_CLIENTS to be a flag for
# the C compiler to configure the vswitch
# Requires ${SDDF}/util/util.mk to build the utility library for debug output

ifeq ($(strip $(NUM_VSWITCH_CLIENTS)),)
$(error Specify the number of clients for the vswitch.  Expect -DNUM_VSWITCH_CLIENTS=3 or similar)
endif


VSWITCH_COMPONENTS_DIR := $(abspath $(dir $(lastword ${MAKEFILE_LIST})))
VSWITCH_IMAGES:= vswitch.elf
network/components/vswitch/%.o: ${SDDF}/network/components/%.c
	${CC} ${CFLAGS} -c -o $@ $<

VSWITCH_COMPONENT_OBJ := $(addprefix network/components/vswitch, vswitch.o)

CFLAGS_vswitch += ${NUM_VSWITCH_CLIENTS} -I${SDDF}/include/sddf/util

CHECK_VSWITCH_FLAGS_MD5:=.vswitch_cflags-$(shell echo -- ${CFLAGS} ${CFLAGS_vswitch} | shasum | sed 's/ *-//')

${CHECK_VSWITCH_FLAGS_MD5}:
	-rm -f .vswitch_cflags-*
	touch $@

#vpath %.c ${SDDF}/network/components

${VSWITCH_IMAGES): LIBS := libsddf_util_debug.a ${LIBS}

${VSWITCH_COMPONENT_OBJ}: |network/components/vswitch
${VSWITCH_COMPONENT_OBJ}: ${CHECK_NETWORK_FLAGS_MD5}
${VSWITCH_COMPONENT_OBJ}: CFLAGS+=${CFLAGS_vswitch}

%.elf: network/components/vswitch/%.o
	${LD} ${LDFLAGS} -o $@ $< ${LIBS}

clean::
	rm -f vswitch.[od]

clobber::
	rm -f ${IMAGES}

network/components/vswitch:
	mkdir -p $@

-include ${VSWITCH_COMPONENTS_OBJS:.o=.d}
