/*
 * Copyright 2024, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <sddf/blk/shared_queue.h>

void blk_queue_init(blk_queue_handle_t *h,
                        blk_req_queue_t *request,
                        blk_resp_queue_t *response)
{
    h->req_queue = request;
    h->resp_queue = response;
}