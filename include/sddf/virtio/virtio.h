/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <sddf/util/printf.h>

#define VIRTIO_VERSION (0x2)

#define VIRTIO_MMIO_IRQ_VQUEUE  (1 << 0)
#define VIRTIO_MMIO_IRQ_CONFIG  (1 << 1)

/* Note this is little endian */
#define VIRTIO_MMIO_MAGIC_VALUE (0x74726976)

#define VIRTIO_DEVICE_STATUS_ACKNOWLEDGE (0x1)
#define VIRTIO_DEVICE_STATUS_DRIVER (0x2)
#define VIRTIO_DEVICE_STATUS_FAILED (0x80)
#define VIRTIO_DEVICE_STATUS_FEATURES_OK (0x8)
#define VIRTIO_DEVICE_STATUS_DRIVER_OK (0x4)
#define VIRTIO_DEVICE_STATUS_DRIVER_RESET (0x40)

#define VIRTIO_F_INDIRECT_DESC 28
#define VIRTIO_F_EVENT_IDX 29
#define VIRTIO_F_VERSION_1 32
#define VIRTIO_F_ACCESS_PLATFORM 33
#define VIRTIO_F_RING_PACKED 34
#define VIRTIO_F_IN_ORDER 35
#define VIRTIO_F_ORDER_PLATFORM 36
#define VIRTIO_F_SR_IOV 37
#define VIRTIO_F_NOTIFICATION_DATA 38
#define VIRTIO_F_NOTIF_CONFIG_DATA 39
#define VIRTIO_F_RING_RESET 40

typedef enum {
    VIRTIO_DEVICE_ID_NET = 0x1,
} virtio_device_id_t;

typedef volatile struct {
    uint32_t MagicValue;
    uint32_t Version;
    uint32_t DeviceID;
    uint32_t VendorID;
    uint32_t DeviceFeatures;
    uint32_t DeviceFeaturesSel;
    uint32_t _reserved0[2];
    uint32_t DriverFeatures;
    uint32_t DriverFeaturesSel;
    uint32_t _reserved1[2];
    uint32_t QueueSel;
    uint32_t QueueNumMax;
    uint32_t QueueNum;
    uint32_t _reserved2[2];
    uint32_t QueueReady;
    uint32_t _reserved3[2];
    uint32_t QueueNotify;
    uint32_t _reserved4[3];
    uint32_t InterruptStatus;
    uint32_t InterruptACK;
    uint32_t _reserved5[2];
    uint32_t Status;
    uint32_t _reserved6[3];
    uint32_t QueueDescLow;
    uint32_t QueueDescHigh;
    uint32_t _reserved7[2];
    uint32_t QueueDriverLow;
    uint32_t QueueDriverHigh;
    uint32_t _reserved8[2];
    uint32_t QueueDeviceLow;
    uint32_t QueueDeviceHigh;
    uint32_t _reserved9[21];
    uint32_t ConfigGeneration;
    uint32_t Config[0];
} virtio_mmio_regs_t;

bool virtio_mmio_check_magic(virtio_mmio_regs_t *regs)
{
    return regs->MagicValue == 0x74726976;
}

bool virtio_mmio_check_device_id(virtio_mmio_regs_t *regs, virtio_device_id_t id)
{
    return regs->DeviceID == id;
}

uint32_t virtio_mmio_version(virtio_mmio_regs_t *regs)
{
    return regs->Version;
}

void virtio_print_reserved_feature_bits(uint64_t feature)
{
    if (feature & ((uint64_t)1 << VIRTIO_F_INDIRECT_DESC)) {
        sddf_dprintf("    VIRTIO_F_INDIRECT_DESC\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_EVENT_IDX)) {
        sddf_dprintf("    VIRTIO_F_EVENT_IDX\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_VERSION_1)) {
        sddf_dprintf("    VIRTIO_F_VERSION_1\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_ACCESS_PLATFORM)) {
        sddf_dprintf("    VIRTIO_F_ACCESS_PLATFORM\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_RING_PACKED)) {
        sddf_dprintf("    VIRTIO_F_RING_PACKED\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_IN_ORDER)) {
        sddf_dprintf("    VIRTIO_F_IN_ORDER\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_ORDER_PLATFORM)) {
        sddf_dprintf("    VIRTIO_F_ORDER_PLATFORM\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_SR_IOV)) {
        sddf_dprintf("    VIRTIO_F_SR_IOV\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_NOTIFICATION_DATA)) {
        sddf_dprintf("    VIRTIO_F_NOTIFICATION_DATA\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_NOTIF_CONFIG_DATA)) {
        sddf_dprintf("    VIRTIO_F_NOTIF_CONFIG_DATA\n");
    }
    if (feature & ((uint64_t)1 << VIRTIO_F_RING_RESET)) {
        sddf_dprintf("    VIRTIO_F_RING_RESET\n");
    }
}
