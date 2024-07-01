/*
 * Copyright 2022, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sddf/util/string.h>
#include <sddf/util/util.h>
#include <sddf/util/fence.h>

typedef struct serial_queue {
    /* index to insert at */
    uint32_t tail;
    /* index to remove from */
    uint32_t head;
    /* flag to indicate whether consumer requires signalling */
    uint32_t consumer_signalled;
    /* flag to indicate whether producer requires signalling */
    uint32_t producer_signalled;
} serial_queue_t;

typedef struct serial_queue_handle {
    serial_queue_t *queue;
    uint32_t size;
    char *data_region;
} serial_queue_handle_t;

/**
 * Check if the queue is empty.
 *
 * @param queue_handle queue to check.
 * @param local_head head which points to the next character to be dequeued.
 *
 * @return true indicates the queue is empty, false otherwise.
 */
static inline int serial_queue_empty(serial_queue_handle_t *queue_handle, uint32_t local_head)
{
    return local_head == queue_handle->queue->tail;
}

/**
 * Check if the queue is full
 *
 * @param queue_handle queue to check.
 * @param local_tail tail which points to the next enqueue slot.
 *
 * @return true indicates the buffer is full, false otherwise.
 */
static inline int serial_queue_full(serial_queue_handle_t *queue_handle, uint32_t local_tail)
{
    return local_tail - queue_handle->queue->head == queue_handle->size;
}

/**
 * Enqueue a char into a queue.
 *
 * @param queue_handle queue to enqueue into.
 * @param local_tail address of the tail to be incremented. This allows for clients to
 *                      enqueue multiple characters before making the changes visible.
 * @param character character to be enqueued.
 *
 * @return -1 when queue is empty, 0 on success.
 */
static inline int serial_enqueue(serial_queue_handle_t *queue_handle, uint32_t *local_tail,
                                 char character)
{
    if (serial_queue_full(queue_handle, *local_tail)) {
        return -1;
    }

    queue_handle->data_region[*local_tail % queue_handle->size] = character;
    (*local_tail)++;

    return 0;
}

/**
 * Dequeue a char from a queue.
 *
 * @param queue_handle queue to dequeue from.
 * @param local_head address of the head to be incremented. This allows for clients to
 *                      dequeue multiple characters before making the changes visible.
 * @param character character to copy into.
 *
 * @return -1 when queue is empty, 0 on success.
 */
static inline int serial_dequeue(serial_queue_handle_t *queue_handle, uint32_t *local_head,
                                 char *character)
{
    if (serial_queue_empty(queue_handle, *local_head)) {
        return -1;
    }

    *character = queue_handle->data_region[*local_head % queue_handle->size];
    (*local_head)++;

    return 0;
}

/**
 * Update the value of the tail in the shared data structure to make
 * locally enqueued data visible.
 *
 * @param queue_handle queue to update.
 * @param local_tail tail which points to the last character enqueued.
 */
static inline void serial_update_visible_tail(serial_queue_handle_t *queue_handle,
                                              uint32_t local_tail)
{
    uint32_t head = queue_handle->queue->head;
    uint32_t tail = queue_handle->queue->tail;
    uint32_t max_tail = head + queue_handle->size;

    /* Ensure updates to tail don't overwrite existing data */
    if (head <= tail) {
        assert(local_tail >= tail || local_tail < head);
    } else {
        /* tail < head */
        assert(local_tail >= tail && local_tail < head);
    }

    /* Ensure updates to tail don't exceed size restraints */
    if (head <= max_tail) {
        assert(local_tail <= max_tail);
    } else {
        /* max_tail < head */
        assert(local_tail >= head || local_tail <= max_tail);
    }

#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif

    queue_handle->queue->tail = local_tail;
}

/**
 * Update the value of the head in the shared data structure to make
 * locally dequeued data visible.
 *
 * @param queue_handle queue to update.
 * @param local_head head which points to the next character to dequeue.
 */
static inline void serial_update_visible_head(serial_queue_handle_t *queue_handle,
                                              uint32_t local_head)
{
    uint32_t head = queue_handle->queue->head;
    uint32_t tail = queue_handle->queue->tail;

    /* Ensure updates to head don't corrupt queue size constraints */
    if (head <= tail) {
        assert(local_head >= head && local_head <= tail);
    } else {
        /* head > tail */
        assert(local_head >= head || local_head <= tail);
    }

#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif

    queue_handle->queue->head = local_head;
}

/**
 * Return the number of bytes of data stored in the queue.
 *
 * @param queue_handle queue containing the data.
 *
 * @return The bytes of data stored in the queue. 
 */
static inline uint32_t serial_queue_length(serial_queue_handle_t *queue_handle)
{
    return queue_handle->queue->tail - queue_handle->queue->head;
}

/**
 * Return the number of bytes of data stored contiguously in the queue from
 * the head index to either the tail index or the end of the data region.
 *
 * @param queue_handle queue containing the data.
 *
 * @return The bytes of data stored contiguously in the queue. 
 */
static inline uint32_t serial_queue_contiguous_length(serial_queue_handle_t *queue_handle)
{
    return MIN(queue_handle->size - (queue_handle->queue->head % queue_handle->size), serial_queue_length(queue_handle));
}

/**
 * Return the number of free bytes remaining in the queue. This is the number of 
 * bytes that can be added to the queue until the queue is full.
 *
 * @param queue_handle queue to be filled with data.
 *
 * @return The amount of free space remaining in the queue.
 */
static inline uint32_t serial_queue_free(serial_queue_handle_t *queue_handle)
{
    return queue_handle->size - serial_queue_length(queue_handle);
}

/**
 * Return the number of bytes that can be copied into the queue contiguously. This
 * is the number of bytes that can be copied into the queue with a single call of memcpy.
 *
 * @param queue_handle queue to be filled with data.
 *
 * @return The amount of contiguous free space remaining in the queue.
 */
static inline uint32_t serial_queue_contiguous_free(serial_queue_handle_t *queue_handle)
{
    return MIN(queue_handle->size - (queue_handle->queue->tail % queue_handle->size), serial_queue_free(queue_handle));
}

/**
 * Transfer all data from consume queue to produce queue. Assumes there
 * is enough free space in the free queue to fit all data in the active
 * queue.
 *
 * @param active_queue_handle queue to remove from.
 * @param free_queue_handle queue to insert into.
 */
static inline void serial_transfer_all(serial_queue_handle_t *active_queue_handle,
                                       serial_queue_handle_t *free_queue_handle)
{
    assert(serial_queue_length(active_queue_handle) <= serial_queue_free(free_queue_handle));

    while (serial_queue_length(active_queue_handle)) {
        /* Copy all contigous data */
        uint32_t active = serial_queue_contiguous_length(active_queue_handle);
        uint32_t free = serial_queue_contiguous_free(free_queue_handle);
        uint32_t to_transfer = (active < free) ? active : free;

        sddf_memcpy(free_queue_handle->data_region + (free_queue_handle->queue->tail % free_queue_handle->size),
                    active_queue_handle->data_region + (active_queue_handle->queue->head %
                                                        active_queue_handle->size), to_transfer);

        /* Make copy visible */
        serial_update_visible_tail(free_queue_handle, free_queue_handle->queue->tail + to_transfer);
        serial_update_visible_head(active_queue_handle, active_queue_handle->queue->head + to_transfer);
    }
}

/**
 * Transfer all data from consume queue to produce queue and add colour start and end codes
 * before and after the data. Assumes there is enough free space in the free queue to fit
 * all data in the active.
 *
 * @param active_queue_handle queue to remove from.
 * @param free_queue_handle queue to insert into.
 * @param colour_start colour string to be printed at the start
 * @param colour_end colour string to be printed at the end
 */
static inline void serial_transfer_all_with_colour(serial_queue_handle_t *active_queue_handle,
                                                   serial_queue_handle_t *free_queue_handle,
                                                   char *colour_start, char *colour_end)
{
    uint16_t colour_start_length = sddf_strlen(colour_start);
    uint16_t colour_end_length = sddf_strlen(colour_end);
    assert(serial_queue_length(active_queue_handle) + colour_start_length + colour_end_length
           <= serial_queue_free(free_queue_handle));

    uint16_t colour_transferred = 0;
    while (colour_transferred < colour_start_length) {
        uint32_t remaining = colour_start_length - colour_transferred;
        uint32_t free = serial_queue_contiguous_free(free_queue_handle);
        uint32_t to_transfer = (remaining < free) ? remaining : free;

        sddf_memcpy(free_queue_handle->data_region + (free_queue_handle->queue->tail % free_queue_handle->size),
                    colour_start + colour_transferred, to_transfer);

        serial_update_visible_tail(free_queue_handle, free_queue_handle->queue->tail + to_transfer);
        colour_transferred += to_transfer;
    }

    while (serial_queue_length(active_queue_handle)) {
        /* Copy all contigous data */
        uint32_t active = serial_queue_contiguous_length(active_queue_handle);
        uint32_t free = serial_queue_contiguous_free(free_queue_handle);
        uint32_t to_transfer = (active < free) ? active : free;

        sddf_memcpy(free_queue_handle->data_region + (free_queue_handle->queue->tail % free_queue_handle->size),
                    active_queue_handle->data_region + (active_queue_handle->queue->head %
                                                        active_queue_handle->size), to_transfer);

        /* Make copy visible */
        serial_update_visible_tail(free_queue_handle, free_queue_handle->queue->tail + to_transfer);
        serial_update_visible_head(active_queue_handle, active_queue_handle->queue->head + to_transfer);
    }

    colour_transferred = 0;
    while (colour_transferred < colour_end_length) {
        uint32_t remaining = colour_end_length - colour_transferred;
        uint32_t free = serial_queue_contiguous_free(free_queue_handle);
        uint32_t to_transfer = (remaining < free) ? remaining : free;

        sddf_memcpy(free_queue_handle->data_region + (free_queue_handle->queue->tail % free_queue_handle->size),
                    colour_end + colour_transferred, to_transfer);

        serial_update_visible_tail(free_queue_handle, free_queue_handle->queue->tail + to_transfer);
        colour_transferred += to_transfer;
    }
}

/**
 * Initialise the queue handle data structure with the shared queue.
 *
 * @param queue_handle queue handle to use.
 * @param queue pointer to queue in shared memory.
 * @param size size of the queue.
 * @param data_region address of the data region.
 */
static inline void serial_queue_init(serial_queue_handle_t *queue_handle,
                                     serial_queue_t *queue, uint32_t size, char *data_region)
{
    queue_handle->queue = queue;
    queue_handle->size = size;
    queue_handle->data_region = data_region;
}

/**
 * Indicate to producer of the queue that consumer requires signalling.
 *
 * @param queue queue handle of queue that requires signalling upon enqueuing.
 */
static inline void serial_request_consumer_signal(serial_queue_handle_t *queue_handle)
{
    queue_handle->queue->consumer_signalled = 0;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to consumer of the queue that producer requires signalling.
 *
 * @param queue queue handle of queue that requires signalling upon enqueuing.
 */
static inline void serial_request_producer_signal(serial_queue_handle_t *queue_handle)
{
    queue_handle->queue->producer_signalled = 0;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to producer of the queue that consumer has been signalled.
 *
 * @param queue queue handle of the queue that has been signalled.
 */
static inline void serial_cancel_consumer_signal(serial_queue_handle_t *queue_handle)
{
    queue_handle->queue->consumer_signalled = 1;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to consumer of the queue that producer has been signalled.
 *
 * @param queue queue handle of the queue that has been signalled.
 */
static inline void serial_cancel_producer_signal(serial_queue_handle_t *queue_handle)
{
    queue_handle->queue->producer_signalled = 1;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Consumer of the queue requires signalling.
 *
 * @param queue queue handle of the queue to check.
 */
static inline bool serial_require_consumer_signal(serial_queue_handle_t *queue_handle)
{
    return !queue_handle->queue->consumer_signalled;
}

/**
 * Producer of the queue requires signalling.
 *
 * @param queue queue handle of the queue to check.
 */
static inline bool serial_require_producer_signal(serial_queue_handle_t *queue_handle)
{
    return !queue_handle->queue->producer_signalled;
}