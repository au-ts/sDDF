/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * This file provides an "index allocator" implementation that allocates
 * an ID to the caller which can later be to be retrieved and freed.
 * The implementation uses a fixed-size linked list to keep track of free indices.
 */

typedef struct ialloc {
    uint32_t *idxlist; /* linked list pointing to the next free index */
    uint32_t size; /* length of linked list */
    uint32_t head; /* next free index */
    uint32_t tail; /* last free index */
    uint32_t num_free; /* number of free indices */
} ialloc_t;

/**
 * Check if the index list is full.
 *
 * @param ia pointer to the ialloc struct.
 *
 * @return true if index list is full, false otherwise.
 */
static inline bool ialloc_full(ialloc_t *ia)
{
    return ia->num_free == 0;
}

/**
 * Allocate a free index.
 *
 * @param ia pointer to the ialloc struct.
 * @param id pointer to the index allocated.
 *
 * @return 0 on success, -1 if index list is full.
 */
static inline int ialloc_alloc(ialloc_t *ia, uint32_t *id)
{
    if (ialloc_full(ia)) {
        return -1;
    }
    *id = ia->head;
    ia->head = ia->idxlist[ia->head];
    ia->num_free--;
    return 0;
}

/**
 * Free an active index.
 *
 * @param ia pointer to the ialloc struct.
 * @param id active index to be freed.
 *
 * @return 0 on success, -1 if index is invalid.
 */
static inline int ialloc_free(ialloc_t *ia, uint32_t id)
{
    if (id >= ia->size) {
        return -1;
    }
    if (ialloc_full(ia)) {
        // When index list is full, head and tail will point
        // to stale indices, so we have to restore it here.
        ia->head = id;
        ia->tail = id;
    } else {
        ia->idxlist[ia->tail] = id;
        ia->tail = id;
    }
    ia->num_free++;
    return 0;
}

/**
 * Initialise the index allocator.
 *
 * @param ia pointer to the ialloc struct.
 * @param idxlist pointer to the linked list array storing the next free index.
 * @param size number of indices that can be allocated, also length of idxlist.
 */
static void ialloc_init(ialloc_t *ia, uint32_t *idxlist, uint32_t size)
{
    ia->idxlist = idxlist;
    ia->size = size;
    ia->head = 0;
    ia->tail = size - 1;
    ia->num_free = size;
    for (uint32_t i = 0; i < size - 1; i++) {
        ia->idxlist[i] = i + 1;
    }
    ia->idxlist[size - 1] = 0;
}