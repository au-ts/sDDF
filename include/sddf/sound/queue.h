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

#define SOUND_NUM_BUFFERS 512
#define SOUND_PCM_BUFFER_SIZE 4096

typedef enum {
    SOUND_CMD_TAKE,
    SOUND_CMD_PREPARE,
    SOUND_CMD_RELEASE,
    SOUND_CMD_START,
    SOUND_CMD_STOP,
} sound_cmd_code_t;

typedef enum {
    SOUND_S_OK = 0,
    SOUND_S_BAD_MSG,
    SOUND_S_NOT_SUPP,
    SOUND_S_IO_ERR,
    SOUND_S_BUSY,
} sound_status_t;

typedef struct sound_pcm_set_params {
    uint8_t channels;
    uint8_t format;
    uint8_t rate;
} sound_pcm_set_params_t;

typedef struct sound_cmd {
    sound_cmd_code_t code;
    uint32_t cookie;
uint32_t stream_id;
    union {
        // Set on TAKE request
        sound_pcm_set_params_t set_params;
        // Set on all responses
        sound_status_t status;
    };
} sound_cmd_t;

typedef struct sound_pcm {
    uint32_t cookie;
    uint32_t stream_id;
    uintptr_t addr;
    unsigned int len;
    // Only used in responses.
    sound_status_t status;
    uint32_t latency_bytes;
} sound_pcm_t;

// Eventually this could be moved into its own library
typedef struct sound_queue_state {
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t size;
} sound_queue_state_t;

typedef struct sound_cmd_queue_t {
    sound_queue_state_t state;
    sound_cmd_t buffers[SOUND_NUM_BUFFERS];
} sound_cmd_queue_t;

typedef struct sound_pcm_queue {
    sound_queue_state_t state;
    sound_pcm_t buffers[SOUND_NUM_BUFFERS];
} sound_pcm_queue_t;

typedef struct sound_queues {
    sound_cmd_queue_t *cmd_req;
    sound_cmd_queue_t *cmd_res;

    sound_pcm_queue_t *pcm_req;
    sound_pcm_queue_t *pcm_res;
} sound_queues_t;

typedef struct sound_state {
    sound_shared_state_t *shared_state;
    sound_queues_t queues;
} sound_state_t;

/**
 * Initialise a queue. Only one side needs to call this function.
 *
 * @param queue_state 
 * @param buffer_count number of buffers in the queue
 */
void sound_queue_init(sound_queue_state_t *queue_state, uint32_t buffer_count);

/**
 * Initialises all queues to maximum capacity
 */
void sound_queues_init_default(sound_queues_t *queues);

/**
 * Check if the command queue buffer is empty.
 *
 * @param queue_handle queue handle containing command queue buffer.
 *
 * @return true indicates the buffer is empty, false otherwise.
 */
bool sound_queue_empty(sound_queue_state_t *queue_state);

/**
 * Check if the command queue buffer is full.
 * Leaves a gap of one buffer before we consider the queue full.
 *
 * @param queue_handle queue handle containing command queue buffer.
 *
 * @return true indicates the command queue buffer is full, false otherwise.
 */
bool sound_queue_full(sound_queue_state_t *queue_state);

/**
 * Get the number of elements in a command queue buffer.
 *
 * @param queue_handle queue handle containing command and response queue buffers.
 *
 * @return number of elements in the queue buffer.
 */
int sound_queue_size(sound_queue_state_t *queue_state);

/**
 * Enqueue a command into the command queue buffer.
 *
 * @param queue Command queue to enqueue to
 * @param command Reference to command to enqueue
 *
 * @return -1 when command queue is full, 0 on success.
 */
int sound_enqueue_cmd(sound_cmd_queue_t *queue, const sound_cmd_t *command);

/**
 * Enqueue a PCM data element into the PCM data queue buffer.
 *
 * @param queue PCM data queue to enqueue to.
 * @param pcm PCM data to enqueue.
 *
 * @return -1 when queue is full, 0 on success.
 */
int sound_enqueue_pcm(sound_pcm_queue_t *queue, sound_pcm_t *pcm);

/**
 * Dequeue an element from a command queue buffer.
 *
 * @param queue The command queue to dequeue from.
 * @param out Pointer to write command to.
 *
 * @return -1 when command queue is empty, 0 on success.
 */
int sound_dequeue_cmd(sound_cmd_queue_t *queue, sound_cmd_t *out);

/**
 * Dequeue an element from a pcm data queue buffer.
 *
 * @param queue The pcm data queue to dequeue from.
 * @param out Pointer to write pcm data to.
 *
 * @return -1 when command queue is empty, 0 on success.
 */
int sound_dequeue_pcm(sound_pcm_queue_t *queue, sound_pcm_t *out);

/**
 * Dequeue an element from a queue buffer without getting its data.
 *
 * @param queue The pcm data queue to dequeue from.
 *
 * @return -1 when command queue is empty, 0 on success.
 */
int sound_queue_dequeue(sound_queue_state_t *queue);

/**
 * Get a reference to the head of a command queue buffer.
 *
 * @param queue queue buffer
 *
 * @return Command
 */
sound_cmd_t *sound_cmd_queue_front(sound_cmd_queue_t *queue);

/**
 * Get a reference to the head of a PCM data queue.
 *
 * @param queue PCM data queue
 *
 * @return PCM data struct
 */
sound_pcm_t *sound_pcm_front(sound_pcm_queue_t *queue);

const char *sound_command_code_str(sound_cmd_code_t code);

const char *sound_status_code_str(sound_status_t code);

const char *sound_pcm_fmt_str(sound_pcm_fmt_t fmt);
