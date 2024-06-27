/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <microkit.h>
#include <stdint.h>
#include <stdbool.h>
#include <sddf/blk/fsmalloc.h>
#include <sddf/blk/queue.h>
#include <sddf/blk/msdos_mbr.h>
#include <sddf/blk/util.h>
#include <sddf/util/cache.h>
#include <sddf/util/ialloc.h>
#include <sddf/util/printf.h>
#include <sddf/util/util.h>

// #define DEBUG_BLK_VIRT

#if defined(DEBUG_BLK_VIRT)
#define LOG_BLK_VIRT(...) do{ sddf_dprintf("BLK_VIRT|INFO: "); sddf_dprintf(__VA_ARGS__); }while(0)
#else
#define LOG_BLK_VIRT(...) do{}while(0)
#endif


#define LOG_BLK_VIRT_ERR(...) do{ sddf_dprintf("BLK_VIRT|ERROR: "); sddf_dprintf(__VA_ARGS__); }while(0)

/* TODO: Currently only works for 1 and 2 clients, need to handle multiple clients */

#define MAX_BLK_NUM_CLIENTS 16

#define DRIVER_CH 1
#define CLIENT_CH_1 3
#define CLIENT_CH_2 4

#define MEM_REGION_SIZE 0x200000 //@ericc: autogen this from microkit xml system file
#define DRV_MAX_DATA_BUFFERS (MEM_REGION_SIZE / BLK_TRANSFER_SIZE)

#define REQBK_SIZE (BLK_NUM_CLIENTS * BLK_QUEUE_SIZE)

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

/* Client specific info */
typedef struct client {
    blk_queue_handle_t queue_h;
    microkit_channel ch;
    uint32_t start_sector;
    uint32_t sectors;
} client_t;
client_t clients[MAX_BLK_NUM_CLIENTS];

blk_queue_handle_t drv_h;

/* Fixed size memory allocator */
static fsmalloc_t fsmalloc;
static bitarray_t fsmalloc_avail_bitarr;
static word_t fsmalloc_avail_bitarr_words[roundup_bits2words64(DRV_MAX_DATA_BUFFERS)];

/* Bookkeeping struct per request */
typedef struct reqbk {
    uint32_t cli_id;
    uint32_t cli_req_id;
    uintptr_t cli_addr;
    uintptr_t drv_addr;
    uint16_t count;
    blk_request_code_t code;
} reqbk_t;
static reqbk_t reqbk[REQBK_SIZE];

/* Index allocator for request bookkeep */
static ialloc_t ialloc;
static uint32_t ialloc_idxlist[REQBK_SIZE];

/* MS-DOS Master boot record */
struct msdos_mbr msdos_mbr;

/* The virtualiser is not initialised until we can read the MBR and populate the block device configuration. */
bool initialised = false;

static void partitions_init()
{
    if (msdos_mbr.signature != MSDOS_MBR_SIGNATURE) {
        LOG_BLK_VIRT_ERR("Invalid MBR signature\n");
        return;
    }

    //@ericc: Figure out a better way to assign partitions to clients
    int client_idx = 0;
    int num_parts = 0;
    for (int i = 0; i < MSDOS_MBR_MAX_PRIMARY_PARTITIONS; i++) {
        if (msdos_mbr.partitions[i].type == MSDOS_MBR_PARTITION_TYPE_EMPTY) {
            continue;
        } else {
            num_parts++;
        }

        if (client_idx < BLK_NUM_CLIENTS) {
            clients[client_idx].start_sector = msdos_mbr.partitions[i].lba_start;
            clients[client_idx].sectors = msdos_mbr.partitions[i].sectors;
            client_idx++;
        }

        if (msdos_mbr.partitions[i].lba_start % (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE) != 0) {
            LOG_BLK_VIRT_ERR("Partition %d start sector %d not aligned to sDDF transfer size\n", i,
                             msdos_mbr.partitions[i].lba_start);
            return;
        }
    }

    if (num_parts < BLK_NUM_CLIENTS) {
        LOG_BLK_VIRT_ERR("Not enough partitions to assign to clients\n");
        return;
    }

    ((blk_storage_info_t *)blk_config)->sector_size = ((blk_storage_info_t *)blk_config_driver)->sector_size;
    ((blk_storage_info_t *)blk_config)->size = clients[0].sectors / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
    ((blk_storage_info_t *)blk_config)->read_only = false;
    ((blk_storage_info_t *)blk_config)->ready = true;
#if BLK_NUM_CLIENTS > 1
    ((blk_storage_info_t *)blk_config2)->sector_size = ((blk_storage_info_t *)blk_config_driver)->sector_size;
    ((blk_storage_info_t *)blk_config2)->size = clients[1].sectors / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
    ((blk_storage_info_t *)blk_config2)->read_only = false;
    ((blk_storage_info_t *)blk_config2)->ready = true;
#endif
}

static void request_mbr()
{
    uintptr_t mbr_addr;
    int err = fsmalloc_alloc(&fsmalloc, &mbr_addr, 1);
    assert(!err);

    uint32_t mbr_req_id;
    reqbk_t mbr_req_data = {0, 0, 0, mbr_addr, 1, 0};
    err = ialloc_alloc(&ialloc, &mbr_req_id);
    assert(!err);
    reqbk[mbr_req_id] = mbr_req_data;

    err = blk_enqueue_req(&drv_h, READ_BLOCKS, mbr_addr, 0, 1, mbr_req_id);
    assert(!err);

    microkit_notify_delayed(DRIVER_CH);
}

static bool handle_mbr_reply()
{
    int err = 0;

    if (blk_resp_queue_empty(&drv_h)) {
        LOG_BLK_VIRT("Notified by driver but queue is empty, expecting a response to READ into sector 0\n");
        return false;
    }

    blk_response_status_t drv_status;
    uint16_t drv_success_count;
    uint32_t drv_resp_id;
    err = blk_dequeue_resp(&drv_h, &drv_status, &drv_success_count, &drv_resp_id);
    assert(!err);

    reqbk_t mbr_req_data = reqbk[drv_resp_id];
    ialloc_free(&ialloc, drv_resp_id);

    if (drv_status != SUCCESS) {
        LOG_BLK_VIRT_ERR("Failed to read sector 0 from driver\n");
        return false;
    }

    microkit_arm_vspace_data_invalidate(mbr_req_data.drv_addr,
                                        mbr_req_data.drv_addr + (BLK_TRANSFER_SIZE * mbr_req_data.count));
    memcpy(&msdos_mbr, (void *)mbr_req_data.drv_addr, sizeof(struct msdos_mbr));
    fsmalloc_free(&fsmalloc, mbr_req_data.drv_addr, mbr_req_data.count);

    return true;
}

void init(void)
{
    // @ericc: Hack, spin wait for config from driver to be set
    while (!(((blk_storage_info_t *)blk_config_driver)->ready)) {
        asm("");
    }

    // Initialise driver queue handle
    blk_queue_init(&drv_h, (blk_req_queue_t *)blk_req_queue_driver, (blk_resp_queue_t *)blk_resp_queue_driver,
                   BLK_QUEUE_SIZE);

    // Initialise client queue handles
    blk_queue_init(&(clients[0].queue_h), (blk_req_queue_t *)blk_req_queue, (blk_resp_queue_t *)blk_resp_queue,
                   BLK_QUEUE_SIZE);
#if BLK_NUM_CLIENTS > 1
    blk_queue_init(&(clients[1].queue_h), (blk_req_queue_t *)blk_req_queue2, (blk_resp_queue_t *)blk_resp_queue2,
                   BLK_QUEUE_SIZE);
#endif

    // Initialise fixed size memory allocator and ialloc
    ialloc_init(&ialloc, ialloc_idxlist, REQBK_SIZE);
    fsmalloc_init(&fsmalloc, blk_data_driver, BLK_TRANSFER_SIZE, DRV_MAX_DATA_BUFFERS, &fsmalloc_avail_bitarr,
                  fsmalloc_avail_bitarr_words, roundup_bits2words64(DRV_MAX_DATA_BUFFERS));

    // Initialise client channels
    clients[0].ch = CLIENT_CH_1;
#if BLK_NUM_CLIENTS > 1
    clients[1].ch = CLIENT_CH_2;
#endif

    request_mbr();
}

static void handle_driver()
{
    blk_response_status_t drv_status;
    uint16_t drv_success_count;
    uint32_t drv_resp_id;

    int err = 0;
    while (!blk_resp_queue_empty(&drv_h)) {
        err = blk_dequeue_resp(&drv_h, &drv_status, &drv_success_count, &drv_resp_id);
        assert(!err);

        reqbk_t cli_data = reqbk[drv_resp_id];
        ialloc_free(&ialloc, drv_resp_id);

        // Free bookkeeping data structures regardless of success or failure
        switch (cli_data.code) {
        case WRITE_BLOCKS:
            fsmalloc_free(&fsmalloc, cli_data.drv_addr, cli_data.count);
            break;
        case READ_BLOCKS:
            fsmalloc_free(&fsmalloc, cli_data.drv_addr, cli_data.count);
            break;
        case FLUSH:
            break;
        case BARRIER:
            break;
        }

        // Get the corresponding client queue handle
        blk_queue_handle_t h = clients[cli_data.cli_id].queue_h;

        // Drop response if client resp queue is full
        if (blk_resp_queue_full(&h)) {
            continue;
        }

        if (drv_status == SUCCESS) {
            switch (cli_data.code) {
            case READ_BLOCKS:
                // Invalidate cache
                microkit_arm_vspace_data_invalidate(cli_data.drv_addr, cli_data.drv_addr + (BLK_TRANSFER_SIZE * cli_data.count));
                // Copy data buffers from driver to client
                memcpy((void *)cli_data.cli_addr, (void *)cli_data.drv_addr, BLK_TRANSFER_SIZE * cli_data.count);
                err = blk_enqueue_resp(&h, SUCCESS, drv_success_count, cli_data.cli_req_id);
                assert(!err);
                break;
            case WRITE_BLOCKS:
                err = blk_enqueue_resp(&h, SUCCESS, drv_success_count, cli_data.cli_req_id);
                assert(!err);
                break;
            case FLUSH:
            case BARRIER:
                err = blk_enqueue_resp(&h, SUCCESS, drv_success_count, cli_data.cli_req_id);
                assert(!err);
                break;
            }
        } else {
            // When more error conditions are added, this will need to be updated to a switch statement
            err = blk_enqueue_resp(&h, SEEK_ERROR, drv_success_count, cli_data.cli_req_id);
            assert(!err);
        }

        // Notify corresponding client
        microkit_notify(clients[cli_data.cli_id].ch);
    }
}

static void handle_client(int cli_id)
{
    blk_queue_handle_t h = clients[cli_id].queue_h;

    blk_request_code_t cli_code;
    uintptr_t cli_addr;
    uint32_t cli_block_number;
    uint16_t cli_count;
    uint32_t cli_req_id;

    uintptr_t drv_addr;
    uint32_t drv_block_number;
    uint32_t drv_req_id;

    int err = 0;
    while (!blk_req_queue_empty(&h)) {
        err = blk_dequeue_req(&h, &cli_code, &cli_addr, &cli_block_number, &cli_count, &cli_req_id);
        assert(!err);

        drv_block_number = cli_block_number + (clients[cli_id].start_sector / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE));

        // Check if client request is within its allocated bounds
        if (cli_code == READ_BLOCKS || cli_code == WRITE_BLOCKS) {
            unsigned long client_sectors = clients[cli_id].sectors / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
            unsigned long client_start_sector = clients[cli_id].start_sector / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
            if (drv_block_number < client_start_sector || drv_block_number + cli_count > client_start_sector + client_sectors) {
                err = blk_enqueue_resp(&h, SEEK_ERROR, 0, cli_req_id);
                assert(!err);
                continue;
            }
        }

        switch (cli_code) {
        case READ_BLOCKS:
            if (blk_req_queue_full(&drv_h) || ialloc_full(&ialloc) || fsmalloc_full(&fsmalloc, cli_count)) {
                continue;
            }
            // Allocate driver data buffers
            fsmalloc_alloc(&fsmalloc, &drv_addr, cli_count);
            break;
        case WRITE_BLOCKS:
            if (blk_req_queue_full(&drv_h) || ialloc_full(&ialloc) || fsmalloc_full(&fsmalloc, cli_count)) {
                continue;
            }
            // Allocate driver data buffers
            fsmalloc_alloc(&fsmalloc, &drv_addr, cli_count);
            // Copy data buffers from client to driver
            memcpy((void *)drv_addr, (void *)cli_addr, BLK_TRANSFER_SIZE * cli_count);
            // Flush the cache
            cache_clean(drv_addr, drv_addr + (BLK_TRANSFER_SIZE * cli_count));
            break;
        case FLUSH:
        case BARRIER:
            if (blk_req_queue_full(&drv_h) || ialloc_full(&ialloc)) {
                continue;
            }
            drv_addr = cli_addr;
            break;
        }

        // Bookkeep client request and generate driver req ID
        reqbk_t cli_data = {cli_id, cli_req_id, cli_addr, drv_addr, cli_count, cli_code};
        err = ialloc_alloc(&ialloc, &drv_req_id);
        assert(!err);
        reqbk[drv_req_id] = cli_data;

        err = blk_enqueue_req(&drv_h, cli_code, drv_addr, drv_block_number, cli_count, drv_req_id);
        assert(!err);
    }
}

void notified(microkit_channel ch)
{
    if (initialised == false) {
        bool success = handle_mbr_reply();
        if (success) {
            partitions_init();
            initialised = true;
        };
        return;
    }

    if (ch == DRIVER_CH) {
        handle_driver();
    } else {
        for (int i = 0; i < BLK_NUM_CLIENTS; i++) {
            handle_client(i);
        }
        microkit_notify_delayed(DRIVER_CH);
    }
}