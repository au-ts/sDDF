#include <microkit.h>
#include <stdint.h>
#include <stdbool.h>
#include <sddf/blk/shared_queue.h>

/* TODO: Currently only works for 1 and 2 clients, need to handle multiple clients */

#define MAX_BLK_NUM_CLIENTS 16

#define DRIVER_CH 1
#define CLIENT_CH_1 3
#define CLIENT_CH_2 4

blk_queue_handle_t drv_h;

int ch2client[MAX_BLK_NUM_CLIENTS];
blk_queue_handle_t hs[MAX_BLK_NUM_CLIENTS];

uintptr_t blk_config_driver;
uintptr_t blk_req_queue_driver;
uintptr_t blk_resp_queue_driver;
uintptr_t blk_data_driver;

uintptr_t blk_config;
uintptr_t blk_config2;
uintptr_t blk_req_queue;
uintptr_t blk_req_queue2; 
uintptr_t blk_resp_queue;
uintptr_t blk_resp_queue2;
uintptr_t blk_data;
uintptr_t blk_data2;

void init(void) {
    blk_queue_init(&drv_h, (blk_req_queue_t *)blk_req_queue_driver, (blk_resp_queue_t *)blk_resp_queue_driver, true, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);

    ch2client[CLIENT_CH_1] = 0;
    blk_queue_init(&hs[0], (blk_req_queue_t *)blk_req_queue, (blk_resp_queue_t *)blk_resp_queue, false, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);
#if BLK_NUM_CLIENTS > 1
    ch2client[CLIENT_CH_2] = 1;
    blk_queue_init(&hs[1], (blk_req_queue_t *)blk_req_queue2, (blk_resp_queue_t *)blk_resp_queue2, false, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);
#endif
}

static void handle_driver() {

}

static void handle_client(int client) {
    blk_queue_handle_t h = hs[client];

    blk_request_code_t req_code;
    uintptr_t req_addr;
    uint32_t req_block_number;
    uint16_t req_count;
    uint32_t req_id;

    while (!blk_req_queue_empty(h)) {
        blk_dequeue_req(&h, &req_code, &req_addr, &req_block_number, &req_count, &req_id);

        blk_response_status_t status = SUCCESS;
        uint16_t success_count = 0;

        // @ericc: These should be the same across all requests - we return what the request gives us
        uintptr_t addr = req_addr;
        uint16_t count = req_count;
        uint32_t id = req_id;
        
        if (blk_resp_queue_full(h)) {
            continue;
        }
    }
}

void notified(microkit_channel ch) {
    if (ch == DRIVER_CH) {
        handle_driver();
    } else {
        handle_client(ch2client[ch]);
    }
}