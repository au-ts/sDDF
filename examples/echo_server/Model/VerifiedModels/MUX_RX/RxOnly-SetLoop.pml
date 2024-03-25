// COMPONENTS: ARP, COPY1, COPY2, MUX_RX, ETH
// SIMPLIFICATIONS: Neighbours of MUX_RX don't interact with other components, just mimic these interactions
// DEADLOCK SOLUTION: set loop

// OUTCOME: (secs, depth), comment out unnecessary code
// Individual Clients (HW_LENGTH = CHAN_LENGTH = 3, all priorities = 1)
// - CLIENT = ? > 30 mins, stuck around 417
// - ARP = 284, 587
// Two Clients (priorities: clients = 1, mux = 2, eth = 3)
// - HW_LENGTH = 3 = 0.37, 1183
// - HW_LENGTH = 5 = 2.42, 2192
// One Client, one ARP (priorities: clients = 1, mux = 2, eth = 3)
// - HW_LENGTH = 3 = 0.14, 1132
// - HW_LENGTH = 5 = 2, 2137

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

mux_queue ARP_RX_free;
mux_queue ARP_RX_used;

mux_queue CPY1_RX_free;
mux_queue CPY1_RX_used;

mux_queue CPY2_RX_free;
mux_queue CPY2_RX_used;

typedef eth_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    short ring[HW_LENGTH];
    unsigned head : 3;
    unsigned tail : 3;
}

eth_queue ETH_RX_free;
eth_queue ETH_RX_used;

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

// Initialisation
init priority 5 {
    ARP_RX_free.notify_flag = 1;
    ARP_RX_used.ring[0] = 1;
    ARP_RX_used.ring[0] = 1;
    ARP_RX_used.head = 2;

    CPY1_RX_free.notify_flag = 1;
    CPY1_RX_used.ring[0] = 1;
    CPY1_RX_used.ring[0] = 1;
    CPY1_RX_used.head = 2;

    CPY2_RX_free.notify_flag = 1;
    CPY2_RX_used.ring[0] = 1;
    CPY2_RX_used.ring[0] = 1;
    CPY2_RX_used.head = 2;

    ETH_RX_used.notify_flag = 1;
    ETH_RX_free.notify_flag = 1;

    ARP_RX_used.notification ! 1;
    CPY1_RX_used.notification ! 1;
    CPY2_RX_used.notification ! 1;
}

// ARP Component
active proctype ARP() priority 1 {
    do
        :: ARP_RX_used.notification ? 1 ->
            bit received;
            work_arp: ;
            do
                :: !EMPTY(ARP_RX_used) ->
                    TRANSFER_PACKET(ARP_RX_used, CHAN_LENGTH, ARP_RX_free, CHAN_LENGTH, 1);
                    received = 1;
                :: else -> break;
            od

            ARP_RX_used.notify_flag = 1;

            if 
                :: !EMPTY(ARP_RX_used) -> 
                    ARP_RX_used.notify_flag = 0;
                    goto work_arp;
                :: else;
            fi

            if
                :: received && ARP_RX_free.notify_flag && !(ARP_RX_free.notification ?? [1]) ->
                    ARP_RX_free.notify_flag = 0;
                    ARP_RX_free.notification ! 1;
                :: else;
            fi
    od;
}

// Client Component
active proctype COPY1() priority 1 {
    do
        :: CPY1_RX_used.notification ? 1 ->
            short clt_packets = 2;
            bit enqueued;
            work_copy: ;
            do
                :: !EMPTY(CPY1_RX_used) && !FULL(CPY1_RX_free, CHAN_LENGTH) && clt_packets ->
                    TRANSFER_PACKET(CPY1_RX_used, CHAN_LENGTH, CPY1_RX_free, CHAN_LENGTH, 1);
                    clt_packets --;
                    enqueued = 1;
                :: skip -> break;
            od;

            // Always get notified when packets are received
            CPY1_RX_used.notify_flag = 1;

            if 
                :: !EMPTY(CPY1_RX_used) && !FULL(CPY1_RX_free, CHAN_LENGTH) && clt_packets ->
                    CPY1_RX_used.notify_flag = 0;
                    goto work_copy;
                :: else;
            fi

            if 
                :: CPY1_RX_free.notify_flag && enqueued && !(ARP_RX_free.notification ?? [1]) -> 
                    CPY1_RX_free.notify_flag = 0;
                    // Send through on the same notification (but with different flags)
                    ARP_RX_free.notification ! 1;
                :: else;
            fi;
    od;
}

// Client Component
active proctype COPY2() priority 1 {
    do
        :: CPY2_RX_used.notification ? 1 ->
            short clt_packets = 2;
            bit enqueued;
            work_copy: ;
            do
                :: !EMPTY(CPY2_RX_used) && !FULL(CPY2_RX_free, CHAN_LENGTH) && clt_packets ->
                    TRANSFER_PACKET(CPY2_RX_used, CHAN_LENGTH, CPY2_RX_free, CHAN_LENGTH, 1);
                    clt_packets --;
                    enqueued = 1;
                :: skip -> break;
            od;

            // Always get notified when packets are received
            CPY2_RX_used.notify_flag = 1;

            if 
                :: !EMPTY(CPY2_RX_used) && !FULL(CPY2_RX_free, CHAN_LENGTH) && clt_packets ->
                    CPY2_RX_used.notify_flag = 0;
                    goto work_copy;
                :: else;
            fi

            if 
                :: CPY2_RX_free.notify_flag && enqueued && !(ARP_RX_free.notification ?? [1]) -> 
                    CPY2_RX_free.notify_flag = 0;
                    // Send through on the same notification (but with different flags)
                    ARP_RX_free.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    do
        :: ETH_RX_used.notification ? 1 -> goto work_rx;
        :: ARP_RX_free.notification ? 1 ->

            // Transfer received packets from ETH used to CLT used 
            work_rx: ;
            bit notify_client[CLIENT_NUM] = {0};
            bit notify_eth = 0;

            mux_used_used: ;
            do
                :: !EMPTY(ETH_RX_used) ->
                    short clt = ETH_RX_used.ring[ETH_RX_used.tail];

                    if
                        // ARP
                        :: clt == 0 -> 
                            if
                                :: FULL(ARP_RX_used, CHAN_LENGTH) ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, ETH_RX_free, HW_LENGTH, 0);
                                    notify_eth = 1;
                                :: else ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, ARP_RX_used, CHAN_LENGTH, 1);
                                    notify_client[0] = 1;
                            fi

                        // COPY 1
                        :: clt == 1 -> 
                            if
                                :: FULL(CPY1_RX_used, CHAN_LENGTH) ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, ETH_RX_free, HW_LENGTH, 1);
                                    notify_eth = 1;
                                :: else ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, CPY1_RX_used, CHAN_LENGTH, 1);
                                    notify_client[1] = 1;
                            fi

                        // COPY 2
                        :: clt == 2 ->
                            if
                                :: FULL(CPY2_RX_used, CHAN_LENGTH) ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, ETH_RX_free, HW_LENGTH, 2);
                                    notify_eth = 1;
                                :: else ->
                                    TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, CPY2_RX_used, CHAN_LENGTH, 1);
                                    notify_client[2] = 1;
                            fi

                        // UNKOWN
                        :: else -> 
                            TRANSFER_PACKET(ETH_RX_used, HW_LENGTH, ETH_RX_free, HW_LENGTH, 4);
                            notify_eth = 1;
                    fi;

                :: else -> break;
            od

            ETH_RX_used.notify_flag = 1;

            // Ensure no packets were missed
            if
                :: !EMPTY(ETH_RX_used) ->
                    ETH_RX_used.notify_flag = 0;
                    goto mux_used_used;
                :: else;
            fi

            short client_to_notify = 0;
            do
                // Notify CLT if packets have been transferred & flag is set
                :: client_to_notify == 0 ->
                    if
                        :: notify_client[client_to_notify] && ARP_RX_used.notify_flag && !(ARP_RX_used.notification ?? [1]) -> 
                            ARP_RX_used.notify_flag = 0;
                            ARP_RX_used.notification ! 1;
                        :: else;
                    fi;
                    client_to_notify ++;

                :: client_to_notify == 1 ->
                    if
                        :: notify_client[client_to_notify] && CPY1_RX_used.notify_flag && !(CPY1_RX_used.notification ?? [1]) -> 
                            CPY1_RX_used.notify_flag = 0;
                            CPY1_RX_used.notification ! 1;
                        :: else;
                    fi;
                    client_to_notify ++;

                :: client_to_notify == 2 ->
                    if
                        :: notify_client[client_to_notify] && CPY2_RX_used.notify_flag && !(CPY2_RX_used.notification ?? [1]) -> 
                            CPY2_RX_used.notify_flag = 0;
                            CPY2_RX_used.notification ! 1;
                        :: else;
                    fi;
                    client_to_notify ++;

                :: else -> break;
            od
            

            // Transfer free buffers from CLT free to ETH free
            mux_free_free_0: ;
            do
                :: !EMPTY(ARP_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) ->
                    TRANSFER_PACKET(ARP_RX_free, CHAN_LENGTH, ETH_RX_free, HW_LENGTH, 0);
                    notify_eth = 1;
                :: else -> break;
            od

            ARP_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            if
                :: !EMPTY(ARP_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) -> 
                    ARP_RX_free.notify_flag = 0;
                    goto mux_free_free_0;
                :: else;
            fi

            mux_free_free_1: ;
            do
                :: !EMPTY(CPY1_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) ->
                    TRANSFER_PACKET(CPY1_RX_free, CHAN_LENGTH, ETH_RX_free, HW_LENGTH, 1);
                    notify_eth = 1;
                :: else -> break;
            od

            CPY1_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            if
                :: !EMPTY(CPY1_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) -> 
                    CPY1_RX_free.notify_flag = 0;
                    goto mux_free_free_1;
                :: else;
            fi

            mux_free_free_2: ;
            do
                :: !EMPTY(CPY2_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) ->
                    TRANSFER_PACKET(CPY2_RX_free, CHAN_LENGTH, ETH_RX_free, HW_LENGTH, 2);
                    notify_eth = 1;
                :: else -> break;
            od

            CPY2_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            if
                :: !EMPTY(CPY2_RX_free) && !FULL(ETH_RX_free, HW_LENGTH) -> 
                    CPY2_RX_free.notify_flag = 0;
                    goto mux_free_free_2;
                :: else;
            fi

            if
                // Notify ETH if buffers have been transfered & flag is set
                :: notify_eth && ETH_RX_free.notify_flag && !(ETH_RX_free.notification ?? [1]) -> 
                    ETH_RX_free.notify_flag = 0;
                    ETH_RX_free.notification ! 1;
                :: else;
            fi

    od;
}

#define ETH_STORED_PACKETS (stored_packets_0 + stored_packets_1 + stored_packets_2)

// Ethernet Driver
active proctype ETH() priority 1 {
    short stored_packets_0 = 0;
    short stored_packets_1 = 0;
    short stored_packets_2 = 0;
    do
        // MUX has free buffers to receive
        :: ETH_RX_free.notification ? 1 ->

            // Transfer free buffers from ETH free to HW_RX
            eth_free_hw: ;
            do
                :: !EMPTY(ETH_RX_free) && (stored_packets_0 < 2 || stored_packets_1 < 2 || stored_packets_2 < 2) ->
                    short clt = ETH_RX_free.ring[ETH_RX_free.tail];
                    if
                        :: clt == 0 -> REMOVE_PACKET(ETH_RX_free, HW_LENGTH, stored_packets_0);
                        :: clt == 1 -> REMOVE_PACKET(ETH_RX_free, HW_LENGTH, stored_packets_1);
                        :: clt == 2 -> REMOVE_PACKET(ETH_RX_free, HW_LENGTH, stored_packets_2);
                        :: else;
                    fi

                :: else -> break
            od

            // Only set flag to 1 if HW_RX not full
            if
                :: (stored_packets_0 < 2 || stored_packets_1 < 2 || stored_packets_2 < 2) ->
                    ETH_RX_free.notify_flag = 1;
                :: else ->
                    ETH_RX_free.notify_flag = 0;
            fi

            // Ensure no buffers were missed
            if
                :: !EMPTY(ETH_RX_free) && (stored_packets_0 < 2 || stored_packets_1 < 2 || stored_packets_2 < 2) -> 
                    ETH_RX_free.notify_flag = 0;
                    goto eth_free_hw;
                :: else;
            fi

        // Device has received packets
        ::  ETH_STORED_PACKETS > 0 && !FULL(ETH_RX_used, HW_LENGTH) ->
            bit packets_transferred = 0;

            do
                // Transfer packets from HW_RX to ETH used
                :: ETH_STORED_PACKETS > 0 && !FULL(ETH_RX_used, HW_LENGTH) ->

                    if
                        :: stored_packets_0 > 0 -> INSERT_PACKET(ETH_RX_used, HW_LENGTH, stored_packets_0, 0);
                        :: stored_packets_2 > 0 -> INSERT_PACKET(ETH_RX_used, HW_LENGTH, stored_packets_2, 2);
                        :: stored_packets_1 > 0 -> INSERT_PACKET(ETH_RX_used, HW_LENGTH, stored_packets_1, 1);
                    fi
                    packets_transferred = 1;

                :: else -> break;
            od

            if
                // Notify ETH if packets have been transfered & flag is set
                :: packets_transferred && ETH_RX_used.notify_flag && !(ETH_RX_used.notification ?? [1]) ->
                    ETH_RX_used.notify_flag = 0;
                    ETH_RX_used.notification ! 1;
                :: else;
            fi

            // Process free buffers
            goto eth_free_hw;
    od
}