#include <microkit.h>
#include <stdint.h>
#include <stdbool.h>
#include <sddf/blk/fsmem.h>
#include <sddf/blk/datastore.h>
#include <sddf/blk/shared_queue.h>
#include <printf.h>

/* TODO: Currently only works for 1 and 2 clients, need to handle multiple clients */

#define MAX_BLK_NUM_CLIENTS 16

#define DRIVER_CH 1
#define CLIENT_CH_1 3
#define CLIENT_CH_2 4

// @ericc: For now size of each buffer is set to 512, to be changed to 4096 later.
#define BUFFER_SIZE 512
// @ericc: = memory region size / BUFFER_SIZE
#define DRV_MAX_DATA_BUFFERS 4096

#define DATASTORE_SIZE (BLK_NUM_CLIENTS * BLK_REQ_QUEUE_SIZE)

uint32_t cli2ch[MAX_BLK_NUM_CLIENTS];

blk_queue_handle_t drv_h;
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

/* Fixed size memory allocator */
static fsmem_t fsmem_data;
static bitarray_t fsmem_avail_bitarr;
static word_t fsmem_avail_bitarr_words[roundup_bits2words64(DRV_MAX_DATA_BUFFERS)];

/* Datastore */
static datastore_t ds;
typedef struct ds_data {
    uint32_t cli_id;
    uint32_t cli_req_id;
    uintptr_t cli_addr;
    uintptr_t drv_addr;
    uint16_t count;
    blk_request_code_t code;
} ds_data_t;
static ds_data_t ds_data[DATASTORE_SIZE];
static uint64_t ds_nextfree[DATASTORE_SIZE];
static bool ds_used[DATASTORE_SIZE];

// @ericc: When we have libc implementation replace this
static void *memcpy(void *restrict dest, const void *restrict src, size_t n)
{
    unsigned char *d = dest;
    const unsigned char *s = src;
    for (; n; n--) *d++ = *s++;
    return dest;
}

void init(void) {
    // @ericc: Hack, spin wait for config from driver to be set
    while (!(((blk_storage_info_t *)blk_config_driver)->ready)) asm("");

    // Initialise driver queue handle
    blk_queue_init(&drv_h, (blk_req_queue_t *)blk_req_queue_driver, (blk_resp_queue_t *)blk_resp_queue_driver, true, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);

    // Initialise client queue handles
    blk_queue_init(&hs[0], (blk_req_queue_t *)blk_req_queue, (blk_resp_queue_t *)blk_resp_queue, false, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);
#if BLK_NUM_CLIENTS > 1
    blk_queue_init(&hs[1], (blk_req_queue_t *)blk_req_queue2, (blk_resp_queue_t *)blk_resp_queue2, false, BLK_REQ_QUEUE_SIZE, BLK_RESP_QUEUE_SIZE);
#endif

    // Initialise fixed size memory allocator and datastore
    datastore_init(&ds, ds_data, sizeof(ds_data_t), ds_nextfree, ds_used, DATASTORE_SIZE);
    fsmem_init(&fsmem_data, blk_data_driver, BUFFER_SIZE, DRV_MAX_DATA_BUFFERS, &fsmem_avail_bitarr, fsmem_avail_bitarr_words, roundup_bits2words64(DRV_MAX_DATA_BUFFERS));

    // Initialise client channels
    cli2ch[0] = CLIENT_CH_1;
#if BLK_NUM_CLIENTS > 1
    cli2ch[1] = CLIENT_CH_2;
#endif
    
    // @ericc: determine config values from partitioner, for now hard code here
    // ((blk_storage_info_t *)blk_config)->blocksize = ((blk_storage_info_t *)blk_config_driver)->blocksize;
    ((blk_storage_info_t *)blk_config)->blocksize = 512;
    ((blk_storage_info_t *)blk_config)->size = ((blk_storage_info_t *)blk_config_driver)->size;
    ((blk_storage_info_t *)blk_config)->ready = true;
#if BLK_NUM_CLIENTS > 1
    // Need to initialise these based on partitioner, right now both clients write into the same disk space
    ((blk_storage_info_t *)blk_config2)->blocksize = 512;
    ((blk_storage_info_t *)blk_config2)->size = ((blk_storage_info_t *)blk_config_driver)->size;
    ((blk_storage_info_t *)blk_config2)->ready = true;
#endif
}

static void handle_driver() {
    blk_response_status_t drv_status;
    uintptr_t drv_addr;
    uint16_t drv_count;
    uint16_t drv_success_count;
    uint32_t drv_resp_id;

    while (!blk_resp_queue_empty(&drv_h)) {
        blk_dequeue_resp(&drv_h, &drv_status, &drv_addr, &drv_count, &drv_success_count, &drv_resp_id);
        
        ds_data_t cli_data;
        datastore_retrieve(&ds, drv_resp_id, &cli_data);
        
        // Free bookkeeping data structures regardless of success or failure
        switch(cli_data.code) {
            case WRITE_BLOCKS:
                fsmem_free(&fsmem_data, cli_data.drv_addr, cli_data.count);
                break;
            case READ_BLOCKS:
                fsmem_free(&fsmem_data, cli_data.drv_addr, cli_data.count);
                break;
            case FLUSH:
                break;
            case BARRIER:
                break;
        }

        // Get the corresponding client queue handle
        blk_queue_handle_t h = hs[cli_data.cli_id];

        //@ericc: drop response if client resp queue is full
        if (blk_resp_queue_full(&h)) {
            continue;
        }

        if (drv_status == SUCCESS) {
            switch(cli_data.code) {
                case READ_BLOCKS:
                    // Invalidate cache
                    seL4_ARM_VSpace_Invalidate_Data(3, cli_data.drv_addr, cli_data.drv_addr + (BUFFER_SIZE * cli_data.count));
                    // Copy data buffers from driver to client
                    memcpy((void *)cli_data.cli_addr, (void *)cli_data.drv_addr, BUFFER_SIZE * cli_data.count);
                    blk_enqueue_resp(&h, SUCCESS, cli_data.cli_addr, cli_data.count, drv_success_count, cli_data.cli_req_id);
                    break;
                case WRITE_BLOCKS:
                    blk_enqueue_resp(&h, SUCCESS, cli_data.cli_addr, cli_data.count, drv_success_count, cli_data.cli_req_id);
                    break;
                case FLUSH:
                case BARRIER:
                    blk_enqueue_resp(&h, SUCCESS, cli_data.cli_addr, cli_data.count, drv_success_count, cli_data.cli_req_id);
                    break;
            }
        } else {
            // When more error conditions are added, this will need to be updated to a switch statement
            blk_enqueue_resp(&h, SEEK_ERROR, cli_data.cli_addr, cli_data.count, drv_success_count, cli_data.cli_req_id);
        }
        
        // Notify corresponding client
        microkit_notify(cli2ch[cli_data.cli_id]);
    }
}

static void handle_client(int cli_id) {
    blk_queue_handle_t h = hs[cli_id];

    blk_request_code_t cli_code;
    uintptr_t cli_addr;
    uint32_t cli_block_number;
    uint16_t cli_count;
    uint32_t cli_req_id;

    uintptr_t drv_addr;
    uint32_t drv_block_number;
    uint64_t drv_req_id;
    while (!blk_req_queue_empty(&h)) {
        blk_dequeue_req(&h, &cli_code, &cli_addr, &cli_block_number, &cli_count, &cli_req_id);

        // TODO: this number will change, to be partitioned by virtualiser
        drv_block_number = cli_block_number;

        switch(cli_code) {
            case READ_BLOCKS:
                if (blk_req_queue_full(&drv_h) || datastore_full(&ds) || fsmem_full(&fsmem_data, cli_count)) {
                    // @ericc: drop request
                    continue;
                }
                // Allocate driver data buffers
                fsmem_alloc(&fsmem_data, &drv_addr, cli_count);
                break;
            case WRITE_BLOCKS:
                if (blk_req_queue_full(&drv_h) || datastore_full(&ds) || fsmem_full(&fsmem_data, cli_count)) {
                    // @ericc: drop request
                    continue;
                }
                // Allocate driver data buffers
                fsmem_alloc(&fsmem_data, &drv_addr, cli_count);
                // Copy data buffers from client to driver
                memcpy((void *)drv_addr, (void *)cli_addr, BUFFER_SIZE * cli_count);
                // Flush the cache
                seL4_ARM_VSpace_Clean_Data(3, drv_addr, drv_addr + (BUFFER_SIZE * cli_count));
                break;
            case FLUSH:
            case BARRIER:
                if (blk_req_queue_full(&drv_h) || datastore_full(&ds)) {
                    // @ericc: drop request
                    continue;
                }
                drv_addr = cli_addr;
                break;
        }

        // Bookkeep client request and generate driver req ID        
        ds_data_t cli_data = {cli_id, cli_req_id, cli_addr, drv_addr, cli_count, cli_code};
        datastore_alloc(&ds, &cli_data, &drv_req_id);
        
        blk_enqueue_req(&drv_h, cli_code, drv_addr, drv_block_number, cli_count, drv_req_id);
    }
}

void notified(microkit_channel ch) {
    if (ch == DRIVER_CH) {
        handle_driver();
    } else {
        for (int i = 0; i < BLK_NUM_CLIENTS; i++) {
            handle_client(i);
        }
        microkit_notify(DRIVER_CH);
    }
}