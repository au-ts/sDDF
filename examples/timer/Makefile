#
# Copyright 2024, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause
#

ifeq ($(strip $(MICROKIT_SDK)),)
$(error MICROKIT_SDK must be specified)
endif
override MICROKIT_SDK := $(abspath ${MICROKIT_SDK})

BUILD_DIR ?= build
# By default we make a debug build so that the client debug prints can be seen.
MICROKIT_CONFIG ?= debug

ifeq ($(strip $(MICROKIT_BOARD)), odroidc4)
	export TIMER_DRIVER_DIR := meson
	export CPU := cortex-a55
else ifeq ($(strip $(MICROKIT_BOARD)), qemu_arm_virt)
	export TIMER_DRIVER_DIR := arm
	export CPU := cortex-a53
	QEMU := qemu-system-aarch64
else
$(error Unsupported MICROKIT_BOARD given)
endif

CC := clang
LD := ld.lld
export MICROKIT_TOOL ?= $(abspath $(MICROKIT_SDK)/bin/microkit)

export BOARD_DIR := $(abspath $(MICROKIT_SDK)/board/$(MICROKIT_BOARD)/$(MICROKIT_CONFIG))
export SDDF := $(abspath ../..)
export TOP:= $(abspath $(dir ${MAKEFILE_LIST}))
IMAGE_FILE := $(BUILD_DIR)/loader.img
REPORT_FILE := $(BUILD_DIR)/report.txt

all: ${IMAGE_FILE}

qemu ${IMAGE_FILE} ${REPORT_FILE} clean clobber: $(IMAGE_FILE) ${BUILD_DIR}/Makefile FORCE
	${MAKE} -C ${BUILD_DIR} MICROKIT_SDK=${MICROKIT_SDK} $(notdir $@)

${BUILD_DIR}/Makefile: timer.mk
	mkdir -p ${BUILD_DIR}
	cp timer.mk ${BUILD_DIR}/Makefile

FORCE: