/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PN532_I2C_BUS_ADDRESS (0x48 >> 1)

#define PN532_PREAMBLE                (0x00)
#define PN532_STARTCODE1              (0x00)
#define PN532_STARTCODE2              (0xFF)
#define PN532_POSTAMBLE               (0x00)

#define PN532_HOSTTOPN532             (0xD4)
#define PN532_PN532TOHOST             (0xD5)

#define PN532_CMD_GETFIRMWAREVERSION    (0x02)
#define PN532_CMD_SAMCONFIGURATION      (0x14)
#define PN532_CMD_RFCONFIGURATION       (0x32)
#define PN532_CMD_INLISTPASSIVETARGET   (0x4A)

#define PN532_MIFARE_ISO14443A          (0x00)

int8_t pn532_write_command(uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen, size_t retries);
bool pn532_read_response(uint8_t *buffer, uint8_t buffer_len, size_t retries);
