/*
 * Copyright 2023, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sddf/sound/sound.h>

#define SDDF_SND_NUM_BUFFERS 512
#define SDDF_SND_PCM_BUFFER_SIZE 4096

typedef enum {
    SDDF_SND_CMD_PCM_TAKE,
    SDDF_SND_CMD_PCM_PREPARE,
    SDDF_SND_CMD_PCM_RELEASE,
    SDDF_SND_CMD_PCM_START,
    SDDF_SND_CMD_PCM_STOP,
} sddf_snd_command_code_t;

typedef enum {
    SDDF_SND_EVT_PCM_PERIOD_ELAPSED,
    SDDF_SND_EVT_PCM_XRUN,
} sddf_snd_event_code_t;

typedef enum {
    SDDF_SND_S_OK = 0,
    SDDF_SND_S_BAD_MSG,
    SDDF_SND_S_NOT_SUPP,
    SDDF_SND_S_IO_ERR,
    SDDF_SND_S_BUSY,
} sddf_snd_status_code_t;

typedef struct sddf_snd_pcm_set_params {
    uint8_t channels;
    uint8_t format;
    uint8_t rate;
} sddf_snd_pcm_set_params_t;

typedef struct sddf_snd_command {
    sddf_snd_command_code_t code;
    uint32_t cookie;
    uint32_t stream_id;
    sddf_snd_pcm_set_params_t set_params;
} sddf_snd_command_t;

typedef struct sddf_snd_response {
    uint32_t cookie;
    sddf_snd_status_code_t status;
} sddf_snd_response_t;

typedef struct sddf_snd_pcm_data {
    uint32_t cookie;
    uint32_t stream_id;
    uintptr_t addr;
    unsigned int len;
    // Only used in responses.
    sddf_snd_status_code_t status;
    uint32_t latency_bytes;
} sddf_snd_pcm_data_t;

// Eventually this could be moved into its own library
typedef struct sddf_snd_ring_state {
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t size;
} sddf_snd_ring_state_t;

typedef struct sddf_snd_cmd_ring_t {
    sddf_snd_ring_state_t state;
    sddf_snd_command_t buffers[SDDF_SND_NUM_BUFFERS];
} sddf_snd_cmd_ring_t;

typedef struct sddf_snd_response_ring_t {
    sddf_snd_ring_state_t state;
    sddf_snd_response_t buffers[SDDF_SND_NUM_BUFFERS];
} sddf_snd_response_ring_t;

typedef struct sddf_snd_pcm_data_ring {
    sddf_snd_ring_state_t state;
    sddf_snd_pcm_data_t buffers[SDDF_SND_NUM_BUFFERS];
} sddf_snd_pcm_data_ring_t;

typedef struct sddf_snd_rings {
    sddf_snd_cmd_ring_t *commands;
    sddf_snd_response_ring_t *responses;

    sddf_snd_pcm_data_ring_t *tx_req;
    sddf_snd_pcm_data_ring_t *tx_res;

    sddf_snd_pcm_data_ring_t *rx_req;
    sddf_snd_pcm_data_ring_t *rx_res;
} sddf_snd_rings_t;

typedef struct sddf_snd_state {
    sddf_snd_shared_state_t *shared_state;
    sddf_snd_rings_t rings;
} sddf_snd_state_t;

/**
 * Initialise a ring. Only one side needs to call this function.
 *
 * @param ring_state 
 * @param buffer_count number of buffers in the ring
 */
void sddf_snd_ring_init(sddf_snd_ring_state_t *ring_state, uint32_t buffer_count);

/**
 * Initialises all rings to maximum capacity
 */
void sddf_snd_rings_init_default(sddf_snd_rings_t *rings);

/**
 * Check if the command ring buffer is empty.
 *
 * @param ring_handle ring handle containing command ring buffer.
 *
 * @return true indicates the buffer is empty, false otherwise.
 */
bool sddf_snd_ring_empty(sddf_snd_ring_state_t *ring_state);

/**
 * Check if the command ring buffer is full.
 * Leaves a gap of one buffer before we consider the ring full.
 *
 * @param ring_handle ring handle containing command ring buffer.
 *
 * @return true indicates the command ring buffer is full, false otherwise.
 */
bool sddf_snd_ring_full(sddf_snd_ring_state_t *ring_state);

/**
 * Get the number of elements in a command ring buffer.
 *
 * @param ring_handle ring handle containing command and response ring buffers.
 *
 * @return number of elements in the ring buffer.
 */
int sddf_snd_ring_size(sddf_snd_ring_state_t *ring_state);

/**
 * Enqueue a command into the command ring buffer.
 *
 * @param ring Command ring to enqueue to
 * @param command Reference to command to enqueue
 *
 * @return -1 when command ring is full, 0 on success.
 */
int sddf_snd_enqueue_cmd(sddf_snd_cmd_ring_t *ring, const sddf_snd_command_t *command);

/**
 * Enqueue a response into the response ring buffer.
 *
 * @param ring Response ring to enqueue to
 * @param status Status code
 *
 * @return -1 when ring is full, 0 on success.
 */
int sddf_snd_enqueue_response(sddf_snd_response_ring_t *ring, sddf_snd_response_t *response);

/**
 * Enqueue a PCM data element into the PCM data ring buffer.
 *
 * @param ring PCM data ring to enqueue to.
 * @param pcm PCM data to enqueue.
 *
 * @return -1 when ring is full, 0 on success.
 */
int sddf_snd_enqueue_pcm_data(sddf_snd_pcm_data_ring_t *ring, sddf_snd_pcm_data_t *pcm);

/**
 * Dequeue an element from a command ring buffer.
 *
 * @param ring The command ring to dequeue from.
 * @param out Pointer to write command to.
 *
 * @return -1 when command ring is empty, 0 on success.
 */
int sddf_snd_dequeue_cmd(sddf_snd_cmd_ring_t *ring, sddf_snd_command_t *out);

/**
 * Dequeue an element from a response ring buffer.
 *
 * @param ring The response ring to dequeue from.
 * @param out Pointer to write response to.
 *
 * @return -1 when command ring is empty, 0 on success.
 */
int sddf_snd_dequeue_response(sddf_snd_response_ring_t *ring, sddf_snd_response_t *out);

/**
 * Dequeue an element from a pcm data ring buffer.
 *
 * @param ring The pcm data ring to dequeue from.
 * @param out Pointer to write pcm data to.
 *
 * @return -1 when command ring is empty, 0 on success.
 */
int sddf_snd_dequeue_pcm_data(sddf_snd_pcm_data_ring_t *ring, sddf_snd_pcm_data_t *out);

/**
 * Dequeue an element from a ring buffer without getting its data.
 *
 * @param ring The pcm data ring to dequeue from.
 *
 * @return -1 when command ring is empty, 0 on success.
 */
int sddf_snd_ring_dequeue(sddf_snd_ring_state_t *ring);

/**
 * Get a reference to the head of a command ring buffer.
 *
 * @param ring Ring buffer
 *
 * @return Command
 */
sddf_snd_command_t *sddf_snd_cmd_ring_front(sddf_snd_cmd_ring_t *ring);

/**
 * Get a reference to the head of a PCM data queue.
 *
 * @param ring PCM data ring
 *
 * @return PCM data struct
 */
sddf_snd_pcm_data_t *sddf_snd_pcm_data_front(sddf_snd_pcm_data_ring_t *ring);

const char *sddf_snd_command_code_str(sddf_snd_command_code_t code);

const char *sddf_snd_status_code_str(sddf_snd_status_code_t code);

const char *sddf_snd_event_code_str(sddf_snd_event_code_t code);

const char *sddf_snd_pcm_fmt_str(sddf_snd_pcm_fmt_t fmt);
