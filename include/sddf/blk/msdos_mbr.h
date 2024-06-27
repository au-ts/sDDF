/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>

/**
 * Master Boot Record with MSDOS partition table.
 *
 * https://en.wikipedia.org/wiki/Master_boot_record
*/

#define MSDOS_MBR_SIGNATURE 0xAA55
#define MSDOS_MBR_MAX_PRIMARY_PARTITIONS 4
#define MSDOS_MBR_SECTOR_SIZE 512

struct msdos_mbr_partition {
    uint8_t status;
    uint8_t chs_start[3];
    uint8_t type;
    uint8_t chs_end[3];
    uint32_t lba_start;
    uint32_t sectors;
} __attribute__((packed));

struct msdos_mbr {
    uint8_t bootstrap[446];
    struct msdos_mbr_partition partitions[MSDOS_MBR_MAX_PRIMARY_PARTITIONS];
    uint16_t signature;
} __attribute__((packed));

#define MSDOS_MBR_PARTITION_TYPE_EMPTY 0x00
