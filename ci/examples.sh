#!/bin/bash

# Copyright 2024, UNSW
#
# SPDX-License-Identifier: BSD-2-Clause

set -e

SDK_PATH=$1

# Number of threads/jobs to compile with, default to 1
if [ "$#" -eq 1 ]; then
NUM_JOBS=1
else
NUM_JOBS=$2
fi

CI_BUILD_DIR="ci_build"

[[ -z $SDK_PATH ]] && echo "usage: examples.sh [PATH TO SDK]" && exit 1
[[ ! -d $SDK_PATH ]] && echo "The path to the SDK provided does not exist: '$SDK_PATH'" && exit 1

NETWORK=true
I2C=true
SERIAL=true
TIMER=true

build_network_echo_server() {
    BOARD=$1
    CONFIG=$2
    echo "CI|INFO: building echo server example with board: ${BOARD}, config: ${CONFIG}"
    BUILD_DIR="${PWD}/${CI_BUILD_DIR}/examples/echo_server/${BOARD}/${CONFIG}"
    rm -rf ${BUILD_DIR}
    mkdir -p ${BUILD_DIR}
    make -j${NUM_JOBS} -C examples/echo_server \
        BUILD_DIR=${BUILD_DIR} \
        MICROKIT_CONFIG=${CONFIG} \
        MICROKIT_SDK=${SDK_PATH} \
        MICROKIT_BOARD=${BOARD}
}

build_i2c() {
    BOARD=$1
    CONFIG=$2
    echo "CI|INFO: building I2C example with board: ${BOARD}, config: ${CONFIG}"
    BUILD_DIR="${PWD}/${CI_BUILD_DIR}/examples/i2c/${BOARD}/${CONFIG}"
    rm -rf ${BUILD_DIR}
    mkdir -p ${BUILD_DIR}
    make -j${NUM_JOBS} -C examples/i2c \
        BUILD_DIR=${BUILD_DIR} \
        MICROKIT_CONFIG=${CONFIG} \
        MICROKIT_SDK=${SDK_PATH} \
        MICROKIT_BOARD=${BOARD}
}

build_timer() {
    BOARD=$1
    CONFIG=$2
    echo "CI|INFO: building timer example with board: ${BOARD}, config: ${CONFIG}"
    BUILD_DIR="${PWD}/${CI_BUILD_DIR}/examples/timer/${BOARD}/${CONFIG}"
    rm -rf ${BUILD_DIR}
    mkdir -p ${BUILD_DIR}
    make -j${NUM_JOBS} -C examples/timer \
        BUILD_DIR=${BUILD_DIR} \
        MICROKIT_CONFIG=${CONFIG} \
        MICROKIT_SDK=${SDK_PATH} \
        MICROKIT_BOARD=${BOARD}
}

build_serial() {
    BOARD=$1
    CONFIG=$2
    echo "CI|INFO: building serial example with board: ${BOARD}, config: ${CONFIG}"
    BUILD_DIR="${PWD}/${CI_BUILD_DIR}/examples/serial/${BOARD}/${CONFIG}"
    rm -rf ${BUILD_DIR}
    mkdir -p ${BUILD_DIR}
    make -j${NUM_JOBS} -C examples/serial \
        BUILD_DIR=${BUILD_DIR} \
        MICROKIT_CONFIG=${CONFIG} \
        MICROKIT_SDK=${SDK_PATH} \
        MICROKIT_BOARD=${BOARD}
}

network() {
    BOARDS=("odroidc4" "imx8mm_evk" "maaxboard" "qemu_arm_virt")
    CONFIGS=("debug" "release" "benchmark")
    for BOARD in "${BOARDS[@]}"
    do
      for CONFIG in "${CONFIGS[@]}"
      do
         build_network_echo_server ${BOARD} ${CONFIG}
       done
    done
}

i2c() {
    BOARDS=("odroidc4")
    CONFIGS=("debug" "release")
    for BOARD in "${BOARDS[@]}"
    do
      for CONFIG in "${CONFIGS[@]}"
      do
         build_i2c ${BOARD} ${CONFIG}
       done
    done
}

timer() {
    BOARDS=("odroidc4" "qemu_arm_virt")
    CONFIGS=("debug" "release")
    for BOARD in "${BOARDS[@]}"
    do
      for CONFIG in "${CONFIGS[@]}"
      do
         build_timer ${BOARD} ${CONFIG}
       done
    done
}

serial() {
    BOARDS=("odroidc4" "qemu_arm_virt" "maaxboard")
    CONFIGS=("debug" "release")
    for BOARD in "${BOARDS[@]}"
    do
      for CONFIG in "${CONFIGS[@]}"
      do
         build_serial ${BOARD} ${CONFIG}
       done
    done
}

# Only run the examples that have been enabled
$NETWORK && network
$I2C && i2c
$TIMER && timer
$SERIAL && serial

echo ""
echo "CI|INFO: Passed all sDDF tests"
