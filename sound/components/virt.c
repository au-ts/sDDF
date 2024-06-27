/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <microkit.h>
#include <sddf/sound/queue.h>
#include <sddf/util/util.h>
#include <sddf/util/cache.h>

#define DRIVER_CH 0
#define CLIENT_CH_BEGIN 1
#define NUM_CLIENTS 2

#define NO_OWNER -1
#define MAX_STREAMS 16

uintptr_t drv_cmd_req;
uintptr_t drv_cmd_res;
uintptr_t drv_pcm_req;
uintptr_t drv_pcm_res;

uintptr_t c0_cmd_req;
uintptr_t c0_cmd_res;
uintptr_t c0_pcm_req;
uintptr_t c0_pcm_res;

uintptr_t c1_cmd_req;
uintptr_t c1_cmd_res;
uintptr_t c1_pcm_req;
uintptr_t c1_pcm_res;

uintptr_t data_region_paddr;
uintptr_t data_region_vaddr;

static sound_queues_t clients[NUM_CLIENTS];
static sound_queues_t driver_queues;

static int owners[MAX_STREAMS];

static void respond_to_cmd(sound_queues_t *client_queues,
                           sound_cmd_t *cmd,
                           sound_status_t status)
{
    cmd->status = status;
    if (sound_enqueue_cmd(&client_queues->cmd_res, cmd) != 0) {
        sddf_dprintf("SND VIRT|ERR: failed to respond to command\n");
    }
}

static void respond_to_pcm(sound_queues_t *client_queues,
                           sound_pcm_t *pcm,
                           sound_status_t status)
{
    pcm->status = status;
    pcm->latency_bytes = 0;
    if (sound_enqueue_pcm(&client_queues->pcm_res, pcm) != 0) {
        sddf_dprintf("SND VIRT|ERR: failed to respond to pcm\n");
    }
}

static int notified_by_client(int client)
{
    if (client < 0 || client > NUM_CLIENTS) {
        sddf_dprintf("SND VIRT|ERR: invalid client id %d\n", client);
        return -1;
    }

    bool notify_client = false;
    bool notify_driver = false;
    sound_queues_t *client_queues = &clients[client];
    sound_cmd_t cmd;

    while (sound_dequeue_cmd(&client_queues->cmd_req, &cmd) == 0) {

        if (cmd.stream_id > MAX_STREAMS) {
            sddf_dprintf(
                "SND VIRT|ERR: [client %d] stream id %u too large (max %u)\n",
                client, cmd.stream_id, MAX_STREAMS);
            respond_to_cmd(client_queues, &cmd, SOUND_S_BAD_MSG);
            continue;
        }

        if (owners[cmd.stream_id] == NO_OWNER) {
            if (cmd.code == SOUND_CMD_TAKE) {
                owners[cmd.stream_id] = client;
            } else {
                sddf_dprintf("SND VIRT|ERR: [client %d] client must take first\n",
                             client);
                respond_to_cmd(client_queues, &cmd, SOUND_S_BAD_MSG);
                notify_client = true;
                continue;
            }
        }

        int owner = owners[cmd.stream_id];
        if (owner != client) {
            sddf_dprintf("SND VIRT|ERR: [client %d] stream busy\n", client);
            respond_to_cmd(client_queues, &cmd, SOUND_S_BUSY);
            notify_client = true;
            continue;
        }

        if (sound_enqueue_cmd(&driver_queues.cmd_req, &cmd) != 0) {
            sddf_dprintf(
                "SND VIRT|ERR: [client %d] failed to enqueue command\n",
                client);
            return -1;
        }
        notify_driver = true;
    }

    sound_pcm_t pcm;
    while (sound_dequeue_pcm(&client_queues->pcm_req, &pcm) == 0) {

        if (pcm.stream_id > MAX_STREAMS) {
            sddf_dprintf("SND VIRT|ERR: [client %d] stream id too large\n", client);
            respond_to_pcm(client_queues, &pcm, SOUND_S_BAD_MSG);
            notify_client = true;
            continue;
        }

        int owner = owners[pcm.stream_id];
        if (owner != client) {
            sddf_dprintf("SND VIRT|ERR: [client %d] stream %u not owned by client\n",
                         client, pcm.stream_id);
            respond_to_pcm(client_queues, &pcm, SOUND_S_BAD_MSG);
            notify_client = true;
            continue;
        }

        if (pcm.io_or_offset % SOUND_PCM_BUFFER_SIZE ||
            pcm.io_or_offset >= SOUND_PCM_BUFFER_SIZE * client_queues->pcm_req.size ||
            pcm.len > SOUND_PCM_BUFFER_SIZE) {
            sddf_dprintf("SND VIRT|ERR: [client %d] invalid PCM buffer bounds\n", client);
            respond_to_pcm(client_queues, &pcm, SOUND_S_BAD_MSG);
            notify_client = true;
            continue;
        }

        uintptr_t vaddr = data_region_vaddr + pcm.io_or_offset;
        uintptr_t paddr = data_region_paddr + pcm.io_or_offset;

        // Write PCM data to RAM
        cache_clean(vaddr, vaddr + pcm.len);

        pcm.io_or_offset = paddr;
        if (sound_enqueue_pcm(&driver_queues.pcm_req, &pcm) != 0) {
            sddf_dprintf("SND VIRT|ERR: Failed to enqueue PCM data\n");
            return -1;
        }
        notify_driver = true;
    }

    if (notify_client) {
        microkit_notify(CLIENT_CH_BEGIN + client);
    }
    if (notify_driver) {
        microkit_notify(DRIVER_CH);
    }

    return 0;
}

int notified_by_driver(void)
{
    bool notify[NUM_CLIENTS] = {0};

    sound_cmd_t cmd;
    while (sound_dequeue_cmd(&driver_queues.cmd_res, &cmd) == 0) {

        if (cmd.stream_id > MAX_STREAMS) {
            sddf_dprintf("SND VIRT|ERR: stream id %u too large (max %u)\n",
                         cmd.stream_id, MAX_STREAMS);
            continue;
        }

        int owner = owners[cmd.stream_id];
        if (owner == NO_OWNER) {
            sddf_dprintf("SND VIRT|ERR: stream %u not owned\n", cmd.stream_id);
            continue;
        }

        if (cmd.code == SOUND_CMD_RELEASE ||
            (cmd.code == SOUND_CMD_TAKE && cmd.status != SOUND_S_OK)) {
            owners[cmd.stream_id] = NO_OWNER;
        }

        if (sound_enqueue_cmd(&clients[owner].cmd_res, &cmd) != 0) {
            sddf_dprintf(
                "SND VIRT|ERR: [client %d] failed to enqueue command response\n",
                owner);
            return -1;
        }
        notify[owner] = true;
    }

    sound_pcm_t pcm;
    while (sound_dequeue_pcm(&driver_queues.pcm_res, &pcm) == 0) {

        if (pcm.stream_id > MAX_STREAMS) {
            sddf_dprintf("SND VIRT|ERR: stream id %u too large (max %u)\n",
                         pcm.stream_id, MAX_STREAMS);
            continue;
        }

        int owner = owners[pcm.stream_id];
        if (owner < 0 || owner > NUM_CLIENTS) {
            sddf_dprintf("SND VIRT|ERR: invalid owner id %d\n", owner);
            continue;
        }

        sound_queues_t *client_queues = &clients[owner];
        uintptr_t paddr = pcm.io_or_offset;

        if (paddr < data_region_paddr ||
            paddr >= data_region_paddr + SOUND_PCM_BUFFER_SIZE * client_queues->pcm_res.size ||
            pcm.len > SOUND_PCM_BUFFER_SIZE) {
            sddf_dprintf("SND VIRT|ERR: invalid PCM buffer bounds from driver\n");
            continue;
        }

        uintptr_t offset = paddr - data_region_paddr;
        uintptr_t vaddr = data_region_vaddr + pcm.io_or_offset;

        // Cache is dirty as device may have written to buffer
        microkit_arm_vspace_data_invalidate(vaddr, vaddr + pcm.len);

        pcm.io_or_offset = offset;
        if (sound_enqueue_pcm(&client_queues->pcm_res, &pcm) != 0) {
            sddf_dprintf(
                "SND VIRT|ERR: [client %d] failed to enqueue PCM data\n",
                owner);
            return -1;
        }
        notify[owner] = true;
    }

    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (notify[client]) {
            microkit_notify(CLIENT_CH_BEGIN + client);
        }
    }

    return 0;
}

void init(void)
{
    assert(data_region_paddr);
    assert(data_region_vaddr);

    sound_queues_init(&clients[0],
                      (void *)c0_cmd_req,
                      (void *)c0_cmd_res,
                      (void *)c0_pcm_req,
                      (void *)c0_pcm_res,
                      SOUND_CMD_QUEUE_SIZE,
                      SOUND_PCM_QUEUE_SIZE);

    sound_queues_init(&clients[1],
                      (void *)c1_cmd_req,
                      (void *)c1_cmd_res,
                      (void *)c1_pcm_req,
                      (void *)c1_pcm_res,
                      SOUND_CMD_QUEUE_SIZE,
                      SOUND_PCM_QUEUE_SIZE);

    sound_queues_init(&driver_queues,
                      (void *)drv_cmd_req,
                      (void *)drv_cmd_res,
                      (void *)drv_pcm_req,
                      (void *)drv_pcm_res,
                      SOUND_CMD_QUEUE_SIZE,
                      SOUND_PCM_QUEUE_SIZE);
    sound_queues_init_buffers(&driver_queues);

    for (int i = 0; i < MAX_STREAMS; i++) {
        owners[i] = NO_OWNER;
    }
}

void notified(microkit_channel ch)
{
    if (ch == DRIVER_CH) {
        if (notified_by_driver() != 0) {
            sddf_dprintf("SND VIRT|ERR: failed to handle driver notification\n");
        }
    } else if (ch >= CLIENT_CH_BEGIN && ch < CLIENT_CH_BEGIN + NUM_CLIENTS) {
        int client = ch - CLIENT_CH_BEGIN;
        if (notified_by_client(client) != 0) {
            sddf_dprintf(
                "SND VIRT|ERR: failed to handle notification from client %d\n",
                client);
        }
    } else {
        sddf_dprintf("SND VIRT|ERR: notified by unexpected channel %d\n", ch);
    }
}
