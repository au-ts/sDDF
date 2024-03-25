// COMPONENTS: CLIENT x 2, ARP, MUX_TX, ETH
// SIMPLIFICATIONS: Client passes packets straight from tx free to tx used 
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring if possible, setting flag to 0
// OPTIMISATIONS: CLT only sets flag if free ring is empty
// ALTERATIONS: CLT can send packet wthout notification

// OUTCOME: (secs, depth), comment out unnecessary code
// Individual Clients (HW_LENGTH = CHAN_LENGTH = 3, all priorities = 1)
// - CLIENT = 59.2, 637
// - ARP = 0.19, 236
// Two Clients (priorities: clients = 1, mux = 2, eth = 3)
// - HW_LENGTH = 3 = 0.38, 735
// - HW_LENGTH = 5 = 2.4, 2200
// One Client, one ARP (priorities: clients = 1, mux = 2, eth = 3)
// - HW_LENGTH = 3 = 0.14, 508
// - HW_LENGTH = 5 = 0.52, 1195

// Length of rings between MUX and ETH
#define HW_LENGTH 3
#define CHAN_LENGTH 3
#define CLIENT_NUM 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    bit ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue ARP_TX_free;
mux_queue ARP_TX_used;

mux_queue CLT1_TX_free;
mux_queue CLT1_TX_used;

mux_queue CLT2_TX_free;
mux_queue CLT2_TX_used;

typedef eth_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    short ring[HW_LENGTH];
    unsigned head : 3;
    unsigned tail : 3;
}

eth_queue ETH_TX_free;
eth_queue ETH_TX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
    bit ring[COUNT];
    unsigned head : 2;
    unsigned tail : 2;
    unsigned next_buffer: 2;
}

chan IRQ_TX = [1] of {bit};

dev_queue HW_TX;

// DESCRIPTION + INVARIANTS
// Buffers from tail through to head - 1 inclusive are available to be used
// Write to head position, read from tail position
// Device processes from tail to head, next_buffer is the next buffer the device will process
// If non-empty, queue looks like either:
// 0 <= T <= N <= H < LENGTH
// [ E | E | TP | P | P | NU | U | HE | E | E ]
// OR
// 0 <= H < T <= N < LENGTH
// [ P | P | NU | HE | E | E | E | TP | P | P ]
// NO BUFFERS PROCESSED
// [ E | E | TN | U | U | U | U | HE | E | E ]
// ALL BUFFERS PROCESSED 
// [ P | P | P | HNE | E | E | E | TP | P | P ]
// T = tail, H = head, N = next processed, E = empty, P = processed, U = Unprocessed

// Useful macros
#define EMPTY(q) (q.tail == q.head)

#define FULL(q, length) ((q.head + 1) % length == q.tail)

#define SIZE(q, length) ((q.head - q.tail + length) % length)

#define TRANSFER_PACKET(from_queue, from_length, to_queue, to_length, value) { \
    d_step { \
    to_queue.ring[to_queue.head] = value; \
    from_queue.ring[from_queue.tail] = 0; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    from_queue.tail = (from_queue.tail + 1) % from_length; } \
    }

#define REMOVE_PACKET(from_queue, from_length, stored_count) { \
    d_step { \
    from_queue.ring[from_queue.tail] = 0; \
    from_queue.tail = (from_queue.tail + 1) % from_length; \
    stored_count = stored_count + 1; } \
    }

#define INSERT_PACKET(to_queue, to_length, stored_count, value) { \
    d_step { \
    to_queue.ring[to_queue.head] = value; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    stored_count = stored_count - 1; } \
    }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

// Initialisation
init priority 4 {
    ARP_TX_used.notify_flag = 1;
    ARP_TX_free.ring[0] = 1;
    ARP_TX_free.ring[0] = 1;
    ARP_TX_free.head = 2;

    CLT1_TX_used.notify_flag = 1;
    CLT1_TX_free.ring[0] = 1;
    CLT1_TX_free.ring[0] = 1;
    CLT1_TX_free.head = 2;

    CLT2_TX_used.notify_flag = 1;
    CLT2_TX_free.ring[0] = 1;
    CLT2_TX_free.ring[0] = 1;
    CLT2_TX_free.head = 2;

    ETH_TX_used.notify_flag = 1;
    ETH_TX_free.notify_flag = 1;

    ARP_TX_free.notification ! 1;
    CLT1_TX_free.notification ! 1;
    CLT2_TX_free.notification ! 1;
}

// ARP Component
active proctype ARP() priority 1 {
    do
        // Non determinstically send a packet
        :: !EMPTY(ARP_TX_free) && !FULL(ARP_TX_used, CHAN_LENGTH) -> 
            goto work_arp;

        :: ARP_TX_free.notification ? 1 -> 
            // Send 0-2 packets
            short arp_packets = 2;
            work_arp: ;

            do
                :: arp_packets > 0 && !EMPTY(ARP_TX_free) && !FULL(ARP_TX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(ARP_TX_free, CHAN_LENGTH, ARP_TX_used, CHAN_LENGTH, 1);
                    arp_packets --;
                :: skip -> break;
            od 

            if
                :: arp_packets != 2 && !(ARP_TX_used.notification ?? [1]) -> ARP_TX_used.notification ! 1;
                :: else;
            fi
    od;
}

// Client Component 1
active proctype CLIENT1() priority 1 {
    bit notify_tx;
    do
        :: skip ->
            if  
                // Non determinstically sent a packet
                :: !EMPTY(CLT1_TX_free) && !FULL(CLT1_TX_used, CHAN_LENGTH) -> 
                    goto clt_tx;

                // Packets have been sent
                :: CLT1_TX_free.notification ? 1 ->
                    clt_tx: ;

                    do
                        :: !EMPTY(CLT1_TX_free) && !FULL(CLT1_TX_used, CHAN_LENGTH) ->
                            TRANSFER_PACKET(CLT1_TX_free, CHAN_LENGTH, CLT1_TX_used, CHAN_LENGTH, 1);
                            notify_tx = 1;
                        :: else -> break;
                    od 

                    if
                        :: EMPTY(CLT1_TX_free) ->
                            CLT1_TX_free.notify_flag = 1;
                        :: else ->
                            CLT1_TX_free.notify_flag = 0;
                    fi

                    // Ensure no packets were missed
                    if  
                        :: !EMPTY(CLT1_TX_free) && !FULL(CLT1_TX_used, CHAN_LENGTH) ->
                            CLT1_TX_free.notify_flag = 0;
                            goto clt_tx;
                        :: else;
                    fi;
            fi;

            if
                // If a packet has been added to the tx used ring and notify flag is set, notify MUX_TX
                :: notify_tx && CLT1_TX_used.notify_flag  && !(ARP_TX_used.notification ?? [1]) -> 
                    notify_tx = 0;
                    CLT1_TX_used.notify_flag = 0;
                    ARP_TX_used.notification ! 1;
                :: else;
            fi;
    od;
}

// Client Component 2
active proctype CLIENT2() priority 1 {
    bit notify_tx;
    do
        :: skip ->
            if  
                // Non determinstically sent a packet
                :: !EMPTY(CLT2_TX_free) && !FULL(CLT2_TX_used, CHAN_LENGTH) -> 
                    goto clt_tx;

                // Packets have been sent
                :: CLT2_TX_free.notification ? 1 ->
                    clt_tx: ;

                    do
                        :: !EMPTY(CLT2_TX_free) && !FULL(CLT2_TX_used, CHAN_LENGTH) ->
                            TRANSFER_PACKET(CLT2_TX_free, CHAN_LENGTH, CLT2_TX_used, CHAN_LENGTH, 1);
                            notify_tx = 1;
                        :: else -> break;
                    od 

                    if
                        :: EMPTY(CLT2_TX_free) ->
                            CLT2_TX_free.notify_flag = 1;
                        :: else ->
                            CLT2_TX_free.notify_flag = 0;
                    fi

                    // Ensure no packets were missed
                    if  
                        :: !EMPTY(CLT2_TX_free) && !FULL(CLT2_TX_used, CHAN_LENGTH) ->
                            CLT2_TX_free.notify_flag = 0;
                            goto clt_tx;
                        :: else;
                    fi;
            fi;

            if
                // If a packet has been added to the tx used ring and notify flag is set, notify MUX_TX
                :: notify_tx && CLT2_TX_used.notify_flag  && !(ARP_TX_used.notification ?? [1]) -> 
                    notify_tx = 0;
                    CLT2_TX_used.notify_flag = 0;
                    ARP_TX_used.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Transmit Component
active proctype MUX_TX() priority 1 {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: ETH_TX_free.notification ? 1 -> goto work_tx;
        :: ARP_TX_used.notification ? 1 ->

        // Return free buffers
        work_tx: ;
        bit notify_eth = 0;
        bit notify_client[CLIENT_NUM] = {0};

        mux_tx1: ;
        do
            :: !EMPTY(ETH_TX_free) ->
                short clt = ETH_TX_free.ring[ETH_TX_free.tail];
                if
                    // ARP
                    :: clt == 0 ->
                        TRANSFER_PACKET(ETH_TX_free, HW_LENGTH, ARP_TX_free, CHAN_LENGTH, 1);
                        notify_client[0] = 1;
                    // CLIENT 1
                    :: clt == 1 -> 
                        TRANSFER_PACKET(ETH_TX_free, HW_LENGTH, CLT1_TX_free, CHAN_LENGTH, 1);
                        notify_client[1] = 1;
                    // CLIENT 2
                    :: clt == 2 ->
                        TRANSFER_PACKET(ETH_TX_free, HW_LENGTH, CLT2_TX_free, CHAN_LENGTH, 1);
                        notify_client[2] = 1;
                    // UNKOWN
                    :: else -> 
                        assert(0);
                fi;
            :: else -> break;
        od

        ETH_TX_free.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(ETH_TX_free) ->
                ETH_TX_free.notify_flag = 0;
                goto mux_tx1;
            :: else;
        fi;

        short client_to_notify = 0;
        do
            // Notify CLT if packets have been transferred & flag is set
            :: client_to_notify == 0 ->
                if
                    :: notify_client[client_to_notify] && ARP_TX_free.notify_flag && !(ARP_TX_free.notification ?? [1]) -> 
                        ARP_TX_free.notify_flag = 0;
                        ARP_TX_free.notification ! 1;
                    :: else;
                fi;
                client_to_notify ++;

            :: client_to_notify == 1 ->
                if
                    :: notify_client[client_to_notify] && CLT1_TX_free.notify_flag && !(CLT1_TX_free.notification ?? [1]) -> 
                        CLT1_TX_free.notify_flag = 0;
                        CLT1_TX_free.notification ! 1;
                    :: else;
                fi;
                client_to_notify ++;

            :: client_to_notify == 2 ->
                if
                    :: notify_client[client_to_notify] && CLT2_TX_free.notify_flag && !(CLT2_TX_free.notification ?? [1]) -> 
                        CLT2_TX_free.notify_flag = 0;
                        CLT2_TX_free.notification ! 1;
                    :: else;
                fi;
                client_to_notify ++;

            :: else -> break;
        od

        // Transmit packets
        mux_tx2_0: ;
        do  
            :: !EMPTY(ARP_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                TRANSFER_PACKET(ARP_TX_used, CHAN_LENGTH, ETH_TX_used, HW_LENGTH, 0);
                notify_eth = 1;
            :: else -> break;
        od;

        ARP_TX_used.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(ARP_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                ARP_TX_used.notify_flag = 0;
                goto mux_tx2_0;
            :: else;
        fi;

        mux_tx2_1: ;
        do  
            :: !EMPTY(CLT1_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                TRANSFER_PACKET(CLT1_TX_used, CHAN_LENGTH, ETH_TX_used, HW_LENGTH, 1);
                notify_eth = 1;
            :: else -> break;
        od;

        CLT1_TX_used.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(CLT1_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                CLT1_TX_used.notify_flag = 0;
                goto mux_tx2_1;
            :: else;
        fi;

        mux_tx2_2: ;
        do  
            :: !EMPTY(CLT2_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                TRANSFER_PACKET(CLT2_TX_used, CHAN_LENGTH, ETH_TX_used, HW_LENGTH, 2);
                notify_eth = 1;
            :: else -> break;
        od;

        CLT2_TX_used.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(CLT2_TX_used) && !FULL(ETH_TX_used, HW_LENGTH) ->
                CLT2_TX_used.notify_flag = 0;
                goto mux_tx2_2;
            :: else;
        fi;

        if
            :: notify_eth && ETH_TX_used.notify_flag && !(ETH_TX_used.notification ?? [1])  -> 
                ETH_TX_used.notify_flag = 0;
                ETH_TX_used.notification ! 1;
            :: else;
        fi

    od;
}

#define ETH_STORED_PACKETS (eth_stored_packets_0 + eth_stored_packets_1 + eth_stored_packets_2)

short eth_stored_packets_0 = 0;
short eth_stored_packets_1 = 0;
short eth_stored_packets_2 = 0;

// Ethernet Driver
active proctype ETH() priority 1 {
    do
        // MUX requested a transmission
        :: ETH_TX_used.notification ? 1 ->
            eth_tx: ;
            do
                :: !EMPTY(ETH_TX_used)->
                    short clt = ETH_TX_used.ring[ETH_TX_used.tail];
                    if
                        :: clt == 0 -> REMOVE_PACKET(ETH_TX_used, HW_LENGTH, eth_stored_packets_0);
                        :: clt == 1 -> REMOVE_PACKET(ETH_TX_used, HW_LENGTH, eth_stored_packets_1);
                        :: clt == 2 -> REMOVE_PACKET(ETH_TX_used, HW_LENGTH, eth_stored_packets_2);
                        :: else -> assert(0);
                    fi;
                :: else -> break
            od

            ETH_TX_used.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_TX_used) ->
                    ETH_TX_used.notify_flag = 0;
                    goto eth_tx;
                :: else;
            fi;

        // Device has sent packets
        :: ETH_STORED_PACKETS > 0 ->
            bit buffers_transferred = 0;

            do  
                // Device has buffers in hardware tx ring, some of them have been sent, and tx_free ring is not full
                :: ETH_STORED_PACKETS > 0 && !FULL(ETH_TX_free, CHAN_LENGTH) ->

                    if
                        :: eth_stored_packets_0 > 0 -> INSERT_PACKET(ETH_TX_free, HW_LENGTH, eth_stored_packets_0, 0);
                        :: eth_stored_packets_1 > 0 -> INSERT_PACKET(ETH_TX_free, HW_LENGTH, eth_stored_packets_1, 1);
                        :: eth_stored_packets_2 > 0 -> INSERT_PACKET(ETH_TX_free, HW_LENGTH, eth_stored_packets_2, 2);
                    fi

                    // Buffers have been transferred
                    buffers_transferred = 1;
                :: else -> break;
            od

            if  
                // If flag is set and buffers have been transferred notify MUX_TX (if it doesn't have a pending notification)
                :: buffers_transferred && ETH_TX_free.notify_flag && !(ETH_TX_free.notification ?? [1])-> 
                    ETH_TX_free.notify_flag = 0;
                    ETH_TX_free.notification ! 1;
                :: else;
            fi    
    od
}