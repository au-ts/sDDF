/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <microkit.h>

bool arp_register_ipv4(microkit_channel arp_ch, uint32_t ipv4_addr, uint8_t mac[6])
{
    microkit_mr_set(0, ipv4_addr);
    microkit_mr_set(1, (mac[0] << 8) | mac[1]);
    microkit_mr_set(2, (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5]);
    microkit_ppcall(arp_ch, microkit_msginfo_new(0, 3));

    return true;
}
