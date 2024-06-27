/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <microkit.h>
#include <libco.h>
#include <sddf/util/printf.h>
#include <sddf/timer/client.h>
#include <sddf/i2c/queue.h>
#include <sddf/i2c/client.h>
#include "pn532.h"
#include "client.h"

// #define DEBUG_CLIENT

#ifdef DEBUG_CLIENT
#define LOG_CLIENT(...) do{ sddf_dprintf("CLIENT|INFO: "); sddf_printf(__VA_ARGS__); }while(0)
#else
#define LOG_CLIENT(...) do{}while(0)
#endif
#define LOG_CLIENT_ERR(...) do{ sddf_printf("CLIENT|ERROR: "); sddf_printf(__VA_ARGS__); }while(0)

uintptr_t data_region;
uintptr_t request_region;
uintptr_t response_region;
i2c_queue_handle_t queue;

cothread_t t_event;
cothread_t t_main;

#define DEFAULT_READ_RESPONSE_RETRIES (256)
#define DEFAULT_READ_ACK_FRAME_RETRIES (20)

#define STACK_SIZE (4096)
static char t_client_main_stack[STACK_SIZE];

bool read_passive_target_id(uint8_t card_baud_rate, uint8_t *uid_buf, uint8_t *uid_buf_length, uint8_t timeout) {
    uint8_t cmd_header[3] = { PN532_CMD_INLISTPASSIVETARGET, 1, card_baud_rate };

    int ret = pn532_write_command(cmd_header, 3, NULL, 0, DEFAULT_READ_ACK_FRAME_RETRIES);
    if (ret < 0) return false;

    uint8_t response[64];
    bool response_ret = pn532_read_response(response, 64, DEFAULT_READ_RESPONSE_RETRIES);
    if (!response_ret) return false;

    if (response[0] != 1) {
        LOG_CLIENT_ERR("tags found has wrong response value (0x%x)!\n", response[0]);
        return false;
    }

    uint16_t sens_res = response[2];
    sens_res <<= 8;
    sens_res |= response[3];

    LOG_CLIENT("ATQA: 0x%lx\n", sens_res);
    LOG_CLIENT("SAK: 0x%lx\n", response[4]);

    /* Card appears to be Mifare Classic */
    *uid_buf_length = response[5];

    for (uint8_t i = 0; i < response[5]; i++) {
        uid_buf[i] = response[6 + i];
    }

    return true;
}

uint8_t big_buf[64];

void client_main(void) {
    LOG_CLIENT("client_main: started\n");
    uint8_t header[1];
    header[0] = PN532_CMD_GETFIRMWAREVERSION;
    int ret = pn532_write_command(header, 1, NULL, 0, DEFAULT_READ_ACK_FRAME_RETRIES);
    if (ret < 0) {
        LOG_CLIENT_ERR("failed to write PN532_CMD_GETFIRMWAREVERSION\n");
        while (1) {};
    }

    uint8_t response_buffer[6];
    bool response = pn532_read_response(response_buffer, 6, DEFAULT_READ_RESPONSE_RETRIES);
    if (!response) {
        LOG_CLIENT_ERR("failed to read response for PN532_CMD_GETFIRMWAREVERSION\n");
        while (1) {};
    }

    LOG_CLIENT("read response!\n");
    for (int i = 0; i < 6; i++) {
        LOG_CLIENT("firmware_version[%d]: 0x%lx\n", i, response_buffer[i]);
    }

    LOG_CLIENT("set passive activation retries\n");
    uint8_t passive_header[5];
    passive_header[0] = PN532_CMD_RFCONFIGURATION;
    passive_header[1] = 5;
    passive_header[2] = 0xFF;
    passive_header[3] = 0x1;
    /* max retries according to the Arduino code */
    passive_header[4] = 0xff;
    ret = pn532_write_command(passive_header, 5, NULL, 0, DEFAULT_READ_ACK_FRAME_RETRIES);
    if (ret < 0) {
        LOG_CLIENT_ERR("failed to write PN532_CMD_RFCONFIGURATION\n");
        while (1) {};
    }
    response = pn532_read_response(big_buf, 64, DEFAULT_READ_RESPONSE_RETRIES);
    if (!response) {
        LOG_CLIENT_ERR("failed to read response for PN532_CMD_RFCONFIGURATION\n");
        while (1) {};
    }

    LOG_CLIENT("SAM configure\n");
    uint8_t sam_config_header[4];
    sam_config_header[0] = PN532_CMD_SAMCONFIGURATION;
    sam_config_header[1] = 0x1;
    sam_config_header[2] = 0x14;
    sam_config_header[3] = 0x01;
    ret = pn532_write_command(sam_config_header, 4, NULL, 0, DEFAULT_READ_ACK_FRAME_RETRIES);
    if (ret < 0) {
        LOG_CLIENT_ERR("failed to write PN532_CMD_SAMCONFIGURATION\n");
        while (1) {};
    }
    response = pn532_read_response(big_buf, 64, DEFAULT_READ_RESPONSE_RETRIES);
    if (!response) {
        LOG_CLIENT_ERR("failed to read response for PN532_CMD_RFCONFIGURATION\n");
        while (1) {};
    }

    LOG_CLIENT("waiting for card!\n");

    while (true) {
        /* Buffer to store the returned UID */
        uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
        /* Length of the UID (4 or 7 bytes depending on ISO14443A card type) */
        uint8_t uid_length;

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uid_length will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        bool success = read_passive_target_id(PN532_MIFARE_ISO14443A, &uid[0], &uid_length, 0);

        if (success) {
            LOG_CLIENT("Found a card!\n");
            sddf_printf("UID Length: %d bytes\n", uid_length);
            sddf_printf("UID Value: ");
            for (int i = 0; i < uid_length; i++) {
                sddf_printf(" 0x%x", uid[i]);
            }
            sddf_printf("\n");
        } else {
            // PN532 probably timed out waiting for a card
            LOG_CLIENT_ERR("Timed out waiting for a card\n");
        }

        delay_ms(1000);
    }
}

bool delay_ms(size_t milliseconds) {
    size_t time_ns = milliseconds * NS_IN_MS;

    /* Detect potential overflow */
    if (milliseconds != 0 && time_ns / milliseconds != NS_IN_MS) {
        LOG_CLIENT_ERR("overflow detected in delay_ms");
        return false;
    }

    sddf_timer_set_timeout(TIMER_CH, time_ns);
    co_switch(t_event);

    return true;
}

void init(void) {
    LOG_CLIENT("init\n");

    queue = i2c_queue_init((i2c_queue_t *) request_region, (i2c_queue_t *) response_region);

    bool claimed = i2c_bus_claim(I2C_VIRTUALISER_CH, PN532_I2C_BUS_ADDRESS);
    if (!claimed) {
        LOG_CLIENT_ERR("failed to claim PN532 bus\n");
        return;
    }

    /* Define the event loop/notified thread as the active co-routine */
    t_event = co_active();

    /* derive main entry point */
    t_main = co_derive((void *)t_client_main_stack, STACK_SIZE, client_main);

    co_switch(t_main);
}

void notified(microkit_channel ch) {
    switch (ch) {
        case I2C_VIRTUALISER_CH:
            co_switch(t_main);
            break;
        case TIMER_CH:
            co_switch(t_main);
            break;
        default:
            LOG_CLIENT_ERR("Unknown channel 0x%x!\n", ch);
    }
}
