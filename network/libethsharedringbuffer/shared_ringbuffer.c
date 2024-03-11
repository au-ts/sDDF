/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sddf/network/shared_ringbuffer.h>
#include <sddf/util/util.h>
#include <sddf/util/fence.h>

void ring_init(ring_handle_t *ring, ring_buffer_t *free, ring_buffer_t *used, int buffer_init, uint32_t free_size, uint32_t used_size)
{
    ring->free_ring = free;
    ring->used_ring = used;

    if (buffer_init) {
        ring->free_ring->write_idx = 0;
        ring->free_ring->read_idx = 0;
        ring->free_ring->size = free_size;
        ring->free_ring->notify_writer = false;
        ring->free_ring->notify_reader = false;
        ring->used_ring->write_idx = 0;
        ring->used_ring->read_idx = 0;
        ring->used_ring->size = used_size;
        ring->used_ring->notify_writer =false;
        ring->used_ring->notify_reader = false;
    }
}

int ring_empty(ring_buffer_t *ring)
{
    return !((ring->write_idx - ring->read_idx) % ring->size);
}

int ring_full(ring_buffer_t *ring)
{
    assert((ring->write_idx - ring->read_idx) >= 0);
    return !((ring->write_idx - ring->read_idx + 1) % ring->size);
}

int ring_size(ring_buffer_t *ring)
{
    assert((ring->write_idx - ring->read_idx) >= 0);
    return (ring->write_idx - ring->read_idx);
}

int enqueue(ring_buffer_t *ring, uintptr_t buffer, unsigned int len, void *cookie)
{
    assert(buffer != 0);
    if (ring_full(ring)) {
        return -1;
    }

    ring->buffers[ring->write_idx % ring->size].encoded_addr = buffer;
    ring->buffers[ring->write_idx % ring->size].len = len;
    ring->buffers[ring->write_idx % ring->size].cookie = cookie;

    THREAD_MEMORY_RELEASE();
    ring->write_idx++;

    return 0;
}

int dequeue(ring_buffer_t *ring, uintptr_t *addr, unsigned int *len, void **cookie)
{
    if (ring_empty(ring)) {
        return -1;
    }

    assert(ring->buffers[ring->read_idx % ring->size].encoded_addr != 0);

    *addr = ring->buffers[ring->read_idx % ring->size].encoded_addr;
    *len = ring->buffers[ring->read_idx % ring->size].len;
    *cookie = ring->buffers[ring->read_idx % ring->size].cookie;

    THREAD_MEMORY_RELEASE();
    ring->read_idx++;

    return 0;
}

int enqueue_free(ring_handle_t *ring, uintptr_t addr, unsigned int len, void *cookie)
{
    assert(addr);
    return enqueue(ring->free_ring, addr, len, cookie);
}

int enqueue_used(ring_handle_t *ring, uintptr_t addr, unsigned int len, void *cookie)
{
    assert(addr);
    return enqueue(ring->used_ring, addr, len, cookie);
}

int dequeue_free(ring_handle_t *ring, uintptr_t *addr, unsigned int *len, void **cookie)
{
    return dequeue(ring->free_ring, addr, len, cookie);
}

int dequeue_used(ring_handle_t *ring, uintptr_t *addr, unsigned int *len, void **cookie)
{
    return dequeue(ring->used_ring, addr, len, cookie);
}

int driver_dequeue(ring_buffer_t *ring, uintptr_t *addr, unsigned int *len, void **cookie)
{
    if (ring_empty(ring)) {
        return -1;
    }

    *addr = ring->buffers[ring->read_idx % ring->size].encoded_addr;
    *len = ring->buffers[ring->read_idx % ring->size].len;
    *cookie = &ring->buffers[ring->read_idx % ring->size];

    THREAD_MEMORY_RELEASE();
    ring->read_idx++;

    return 0;
}
