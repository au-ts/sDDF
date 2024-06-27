/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <microkit.h>
#include <stdint.h>

#define ETH_TYPE_ARP 0x0806U
#define ETH_TYPE_IP 0x0800U
#define ETH_HWADDR_LEN 6
#define ETHARP_OPCODE_REQUEST 1
#define ETHARP_OPCODE_REPLY 2

#define NET_BUFFER_SIZE 2048

struct ethernet_address {
  uint8_t addr[6];
} __attribute__((packed));

struct ethernet_header {
  struct ethernet_address dest;
  struct ethernet_address src;
  uint16_t type;
} __attribute__((packed));

/*
 * By default we assume that the hardware we are dealing with
 * cannot generate checksums on transmit. We use this macro
 * to know whether to calculate it in the IP stack.
 */
#if defined(CONFIG_PLAT_IMX8MM_EVK) || defined(CONFIG_PLAT_MAAXBOARD)
#define NETWORK_HW_HAS_CHECKSUM
#endif
