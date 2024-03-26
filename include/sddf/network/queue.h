/*
 * Copyright 2022, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sddf/network/constants.h>
#include <sddf/util/fence.h>
#include <sddf/util/util.h>

typedef struct net_buff_desc {
    /* offset of buffer within buffer memory region or io address of buffer */
    uint64_t io_or_offset;
    /* length of data inside buffer */
    uint16_t len;
} net_buff_desc_t;

typedef struct net_queue {
    /* index to insert at */
    uint16_t tail;
    /* index to remove from */
    uint16_t head;
    /* flag to indicate whether consumer requires signalling */
    uint32_t consumer_signalled;
    /* buffer descripter array */
    net_buff_desc_t buffers[];
} net_queue_t;

typedef struct net_queue_handle {
     /* available buffers */
    net_queue_t *free;
     /* filled buffers */
    net_queue_t *active;
    /* size of the queues */
    uint32_t size;
} net_queue_handle_t;

/**
 * Check if the free queue is empty.
 *
 * @param queue queue handle to dequeue from.
 *
 * @return true indicates the buffer is empty, false otherwise.
 */
static inline bool net_queue_empty_free(net_queue_handle_t *queue)
{
    return !((queue->free->tail - queue->free->head) % queue->size);
}

/**
 * Check if the active queue is empty.
 *
 * @param queue queue handle to dequeue from.
 *
 * @return true indicates the buffer is empty, false otherwise.
 */
static inline bool net_queue_empty_active(net_queue_handle_t *queue)
{
    return !((queue->active->tail - queue->active->head) % queue->size);
}

/**
 * Check if the free queue is full.
 *
 * @param queue queue handle to dequeue from.
 *
 * @return true indicates the queue is full, false otherwise.
 */
static inline bool net_queue_full_free(net_queue_handle_t *queue)
{
    return !((queue->free->tail + 1 - queue->free->head) % queue->size);
}

/**
 * Check if the active queue is full.
 *
 * @param queue queue handle to dequeue from.
 *
 * @return true indicates the queue is full, false otherwise.
 */
static inline bool net_queue_full_active(net_queue_handle_t *queue)
{
    return !((queue->active->tail + 1 - queue->active->head) % queue->size);
}

/**
 * Enqueue an element into a free queue.
 *
 * @param queue queue to enqueue into.
 * @param buffer buffer descriptor for buffer to be enqueued.
 *
 * @return -1 when queue is full, 0 on success.
 */
static inline int net_enqueue_free(net_queue_handle_t *queue, net_buff_desc_t buffer)
{
    if (net_queue_full_free(queue)) return -1;

    queue->free->buffers[queue->free->tail % queue->size] = buffer;
    #ifdef CONFIG_ENABLE_SMP_SUPPORT
        THREAD_MEMORY_RELEASE();
    #endif
    queue->free->tail++;

    return 0;
}

/**
 * Enqueue an element into an active queue.
 *
 * @param queue queue to enqueue into.
 * @param buffer buffer descriptor for buffer to be enqueued.
 *
 * @return -1 when queue is full, 0 on success.
 */
static inline int net_enqueue_active(net_queue_handle_t *queue, net_buff_desc_t buffer)
{
    if (net_queue_full_active(queue)) return -1;

    queue->active->buffers[queue->active->tail % queue->size] = buffer;
    #ifdef CONFIG_ENABLE_SMP_SUPPORT
        THREAD_MEMORY_RELEASE();
    #endif
    queue->active->tail++;

    return 0;
}

/**
 * Dequeue an element from the free queue.
 *
 * @param queue queue handle to dequeue from.
 * @param buffer pointer to buffer descriptor for buffer to be dequeued.
 *
 * @return -1 when queue is empty, 0 on success.
 */
static inline int net_dequeue_free(net_queue_handle_t *queue, net_buff_desc_t *buffer)
{
    if (net_queue_empty_free(queue)) return -1;

    *buffer = queue->free->buffers[queue->free->head % queue->size];
    #ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
    #endif
    queue->free->head++;

    return 0;
}

/**
 * Dequeue an element from the active queue.
 *
 * @param queue queue handle to dequeue from.
 * @param buffer pointer to buffer descriptor for buffer to be dequeued.
 *
 * @return -1 when queue is empty, 0 on success.
 */
static inline int net_dequeue_active(net_queue_handle_t *queue, net_buff_desc_t *buffer)
{
    if (net_queue_empty_active(queue)) return -1;

    *buffer = queue->active->buffers[queue->active->head % queue->size];
    #ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
    #endif
    queue->active->head++;

    return 0;
}

/**
 * Initialise the shared queue.
 *
 * @param queue queue handle to use.
 * @param free pointer to free queue in shared memory.
 * @param active pointer to active queue in shared memory.
 * @param size size of the free and active queues.
 */
static inline void net_queue_init(net_queue_handle_t *queue, net_queue_t *free, net_queue_t *active, uint32_t size)
{
    queue->free = free;
    queue->active = active;
    queue->size = size;
}

/**
 * Initialise the free queue by filling with all free buffers.
 *
 * @param queue queue handle to use.
 * @param base_addr start of the memory region the offsets are applied to (only used between virt and driver)
 */
static inline void net_buffers_init(net_queue_handle_t *queue, uintptr_t base_addr)
{
    for (uint32_t i = 0; i < queue->size - 1; i++) {
        net_buff_desc_t buffer = {(NET_BUFFER_SIZE * i) + base_addr, 0};
        int err = net_enqueue_free(queue, buffer);
        assert(!err);
    }
}

/**
 * Indicate to producer of the free queue that consumer requires signalling.
 *
 * @param queue queue handle of free queue that requires signalling upon enqueuing.
 */
static inline void net_request_signal_free(net_queue_handle_t *queue)
{
    queue->free->consumer_signalled = 0;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to producer of the active queue that consumer requires signalling.
 *
 * @param queue queue handle of active queue that requires signalling upon enqueuing.
 */
static inline void net_request_signal_active(net_queue_handle_t *queue)
{
    queue->active->consumer_signalled = 0;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to producer of the free queue that consumer has been signalled.
 *
 * @param queue queue handle of the free queue that has been signalled.
 */
static inline void net_cancel_signal_free(net_queue_handle_t *queue)
{
    queue->free->consumer_signalled = 1;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Indicate to producer of the active queue that consumer has been signalled.
 *
 * @param queue queue handle of the active queue that has been signalled.
 */
static inline void net_cancel_signal_active(net_queue_handle_t *queue)
{
    queue->active->consumer_signalled = 1;
#ifdef CONFIG_ENABLE_SMP_SUPPORT
    THREAD_MEMORY_RELEASE();
#endif
}

/**
 * Consumer of the free queue requires signalling.
 *
 * @param queue queue handle of the free queue to check.
 */
static inline bool net_require_signal_free(net_queue_handle_t *queue)
{
    return !queue->free->consumer_signalled;
}

/**
 * Consumer of the active queue requires signalling.
 *
 * @param queue queue handle of the active queue to check.
 */
static inline bool net_require_signal_active(net_queue_handle_t *queue)
{
    return !queue->active->consumer_signalled;
}
