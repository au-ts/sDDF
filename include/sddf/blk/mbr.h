#include <stdint.h>

#define MBR_SIGNATURE 0xAA55
#define MBR_MAX_PRIMARY_PARTITIONS 4
#define MBR_SECTOR_SIZE 512

struct mbr_partition {
    uint8_t status;
    uint8_t chs_start[3];
    uint8_t type;
    uint8_t chs_end[3];
    uint32_t lba_start;
    uint32_t sectors;
} __attribute__((packed));

struct mbr {
    uint8_t bootstrap[446];
    struct mbr_partition partitions[MBR_MAX_PRIMARY_PARTITIONS];
    uint16_t signature;
} __attribute__((packed));

#define MBR_PARTITION_TYPE_EMPTY 0x00
