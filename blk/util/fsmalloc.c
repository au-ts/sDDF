/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <sddf/blk/bitarray.h>
#include <sddf/blk/fsmalloc.h>
#include <sddf/util/util.h>

/**
 * Convert a bit position to the address of the corresponding data cell.
 *
 * @param bitpos bit position of the data cell
 * @return address of the data cell
 */
static inline uintptr_t bitpos_to_addr(fsmalloc_t *fsmalloc, uint64_t bitpos)
{
    return fsmalloc->base_addr + (uintptr_t)(bitpos * fsmalloc->cell_size);
}

/**
 * Convert an address to the bit position of the corresponding data cell.
 *
 * @param addr address of the data cell
 * @return bit position of the data cell
 */
static inline uint64_t addr_to_bitpos(fsmalloc_t *fsmalloc, uintptr_t addr)
{
    return (uint64_t)(addr - fsmalloc->base_addr) / fsmalloc->cell_size;
}

/**
 * Check if count number of cells will overflow the end of the data region.
 *
 * @param count number of cells to check
 * @return true if count number of cells will overflow the end of the data region, false otherwise
 */
static inline bool fsmalloc_overflow(fsmalloc_t *fsmalloc, uint64_t count)
{
    return (fsmalloc->avail_bitpos + count > fsmalloc->num_cells);
}

bool fsmalloc_full(fsmalloc_t *fsmalloc, uint64_t count)
{
    if (count > fsmalloc->num_cells) {
        return true;
    }

    if (count == 0) {
        return false;
    }

    unsigned int start_bitpos = fsmalloc->avail_bitpos;
    if (fsmalloc_overflow(fsmalloc, count)) {
        start_bitpos = 0;
    }

    // Create a bit mask with count many 1's
    bitarray_t bitarr_mask;
    word_t words[roundup_bits2words64(count)];
    bitarray_init(&bitarr_mask, words, roundup_bits2words64(count));
    bitarray_set_region(&bitarr_mask, 0, count);

    if (bitarray_cmp_region(fsmalloc->avail_bitarr, start_bitpos, &bitarr_mask, 0, count)) {
        return false;
    }

    return true;
}

void fsmalloc_free(fsmalloc_t *fsmalloc, uintptr_t addr, uint64_t count)
{
    unsigned int start_bitpos = addr_to_bitpos(fsmalloc, addr);

    // Assert here in case we try to free cells that overflow the data region
    assert(start_bitpos + count <= fsmalloc->num_cells);

    // Set the next count many bits as available
    bitarray_set_region(fsmalloc->avail_bitarr, start_bitpos, count);
}

int fsmalloc_alloc(fsmalloc_t *fsmalloc, uintptr_t *addr, uint64_t count)
{
    if (fsmalloc_full(fsmalloc, count)) {
        return -1;
    }

    if (fsmalloc_overflow(fsmalloc, count)) {
        fsmalloc->avail_bitpos = 0;
    }

    *addr = bitpos_to_addr(fsmalloc, fsmalloc->avail_bitpos);

    // Set the next count many bits as unavailable
    bitarray_clear_region(fsmalloc->avail_bitarr, fsmalloc->avail_bitpos, count);

    // Update the bitpos
    uint64_t new_bitpos = fsmalloc->avail_bitpos + count;
    if (new_bitpos == fsmalloc->num_cells) {
        new_bitpos = 0;
    }
    fsmalloc->avail_bitpos = new_bitpos;

    return 0;
}

void fsmalloc_init(fsmalloc_t *fsmalloc, uintptr_t base_addr, uint64_t cell_size, uint64_t num_cells,
                   bitarray_t *bitarr, word_t *words, word_index_t num_words)
{
    bitarray_init(bitarr, words, num_words);

    fsmalloc->avail_bitpos = 0;
    fsmalloc->avail_bitarr = bitarr;
    fsmalloc->base_addr = base_addr;
    fsmalloc->cell_size = cell_size;
    fsmalloc->num_cells = num_cells;

    /* Set all available bits to 1 to indicate all cells are available */
    bitarray_set_region(bitarr, 0, num_cells);
}