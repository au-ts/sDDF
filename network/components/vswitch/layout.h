#pragma once

#include <stdint.h>
#include <sddf/network/queue.h>

/* This is a hard-coded layout while waiting for Ivanv's cool buildsystem tool,
 * basically the vswitch needs to know:
 * - the number of clients
 * - the topology
 * - everyone's MAC address
 * - vswitch's Microkit channel numbers
 * - vswitch's Microkit pointers
 * 
 * the clients needs to know:
 * - the peers it is connected to
 * - the MAC addresses of its peers and itself
 * - its Microkit channel numbers
 * - its Microkit pointers
 * 
 * I am not quite sure how to distributes the MAC address, and who get to
 * decide the MAC address
 * 
 * also, how does the multiplexor know which driver the package should 
 * be sent to? Theoratically it can check the src MAC address to figure out
 * the net interface, but I believe the multiplexor is not designed to be able
 * to read the content of the package.
 */

#define NUM_CLIENTS 2
static uint8_t vswitch_conn_topology[2] = {0b11, 0b11};
#define IS_CONN(s,d)    (vswitch_conn_topology[s] >> (NUM_CLIENTS - d))

static uint8_t vswitch_mac_base[6] = {0x52, 0x54, 0x02, 0x00, 0x00, 0x00};

// @jade: why?
#define TX_QUEUE_SIZE_DRIV                   512
#define RX_QUEUE_SIZE_DRIV                   512

#define TX_CLI0_CH  1
#define RX_CLI0_CH  2

uintptr_t tx_free_cli0;
uintptr_t tx_active_cli0;
uintptr_t rx_free_cli0;
uintptr_t rx_active_cli0;

#define TX_CLI1_CH  3
#define RX_CLI1_CH  4

uintptr_t tx_free_cli1;
uintptr_t tx_active_cli1;
uintptr_t rx_free_cli1;
uintptr_t rx_active_cli1;
