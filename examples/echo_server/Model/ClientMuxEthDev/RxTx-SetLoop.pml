// COMPONENTS: MUX_RX, MUX_TX, ETH, DEV, LINKED_LIST_CLIENT
// SIMPLIFICATIONS:
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring if possible, setting flag to 0
// OUTCOME: Verifies in 0 seconds with priorities. Can't verify in reasonable amount of time with no priorities, but no errors found early.

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    bit ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue ETH_RX_free;
mux_queue ETH_RX_used;
mux_queue ETH_TX_free;
mux_queue ETH_TX_used;

mux_queue CLT_RX_free;
mux_queue CLT_RX_used;
mux_queue CLT_TX_free;
mux_queue CLT_TX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
    bit ring[COUNT];
    unsigned head : 2;
    unsigned tail : 2;
    unsigned next_buffer: 2;
}

chan IRQ_RX = [1] of {bit};
chan IRQ_TX = [1] of {bit};

dev_queue HW_RX;
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
#define CURR_EMPTY(l) (l == 0)

#define EMPTY(q) (q.tail == q.head)

#define FULL(q, length) ((q.head + 1) % length == q.tail)

#define SIZE(q, length) ((q.head - q.tail + length) % length)

#define TRANSFER_PACKET(from_queue, from_length, to_queue, to_length) { \
    to_queue.ring[to_queue.head] = from_queue.ring[from_queue.tail]; \
    from_queue.ring[from_queue.tail] = 0; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    from_queue.tail = (from_queue.tail + 1) % from_length; }

#define STORE_PACKET(from_queue, from_length, linked_list) { \
    from_queue.ring[from_queue.tail] = 0; \
    from_queue.tail = (from_queue.tail + 1) % from_length; \
    linked_list = linked_list + 1; }

#define RETURN_BUFFER(linked_list, to_queue, to_length) { \
    to_queue.ring[to_queue.head] = 1; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    linked_list = linked_list - 1; }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

// Initialisation
init {
    CLT_TX_free.ring[0] = 1;
    CLT_TX_free.ring[1] = 1;
    CLT_TX_free.head = 2;

    CLT_RX_free.notify_flag = 1;
    CLT_RX_used.notify_flag = 1;
    CLT_TX_free.notify_flag = 1;
    CLT_TX_used.notify_flag = 1;
    ETH_RX_free.notify_flag = 1;
    ETH_RX_used.notify_flag = 1;
    ETH_TX_free.notify_flag = 1;
    ETH_TX_used.notify_flag = 1;

    HW_RX.ring[0] = 1;
    HW_RX.ring[1] = 1;
    HW_RX.head = 2;
}

// Client Component
active proctype CLIENT() priority 1 {
    unsigned cur_size : 2;
    bit notify_rx;
    bit notify_tx;
    do
        :: skip ->
            if  
                // Packets have been sent
                :: CLT_TX_free.notification ? 1 ->
                    clt_tx: ;
                    d_step {
                        do
                            // While there are packets to transfer from current
                            :: !CURR_EMPTY(cur_size) && !EMPTY(CLT_TX_free) && !FULL(CLT_TX_used, CHAN_LENGTH) ->
                                // Transfer packet to TX used ring, return buffer to RX free ring
                                TRANSFER_PACKET(CLT_TX_free, CHAN_LENGTH, CLT_TX_used, CHAN_LENGTH);
                                RETURN_BUFFER(cur_size, CLT_RX_free, CHAN_LENGTH);
                                notify_rx = 1;
                                notify_tx = 1;
                            :: else -> break;
                        od 
                    }

                    // // If there are still packets in curr, set notify flag for tx free
                    // if
                    //     :: CURR_EMPTY(cur_size) -> CLT_TX_free.notify_flag = 0;
                    //     :: else -> CLT_TX_free.notify_flag = 1;
                    // fi

                    CLT_TX_free.notify_flag = 1;

                    // Ensure no buffers were missed
                    if  
                        :: !CURR_EMPTY(cur_size) && !EMPTY(CLT_TX_free) && !FULL(CLT_TX_used, CHAN_LENGTH) ->
                            CLT_TX_free.notify_flag = 0;
                            goto clt_tx;
                        :: else;
                    fi;

                    // Process rx used queue
                    goto clt_rx;

                // Packets have been received
                :: CLT_RX_used.notification ? 1 ->
                    clt_rx: ;
                    d_step {
                        do
                            // While there are packets to transfer (linked list never full)
                            :: !EMPTY(CLT_RX_used) ->
                                if
                                    // Transfer packet to TX used ring to resend
                                    :: !FULL(CLT_TX_used, CHAN_LENGTH) && !EMPTY(CLT_TX_free) ->
                                        // CLT_RX_free ring should never be full
                                        TRANSFER_PACKET(CLT_RX_used, CHAN_LENGTH, CLT_RX_free, CHAN_LENGTH);
                                        TRANSFER_PACKET(CLT_TX_free, CHAN_LENGTH, CLT_TX_used, CHAN_LENGTH);
                                        notify_rx = 1;
                                        notify_tx = 1;
                                    // Store packet in current linked list to send later
                                    :: else ->
                                        CLT_TX_free.notify_flag = 1;
                                        STORE_PACKET(CLT_RX_used, CHAN_LENGTH, cur_size);
                                fi
                            :: else -> break;
                        od 
                    }

                    CLT_RX_used.notify_flag = 1;

                    // Ensure no packets were missed
                    if  
                        :: !EMPTY(CLT_RX_used) ->
                            CLT_RX_used.notify_flag = 0;
                            goto clt_rx;
                        :: else;
                    fi;
            fi;

            if
                // If buffers have been added to the rx free ring and notify flag is set, notify MUX_RX
                :: notify_rx && CLT_RX_free.notify_flag && !(CLT_RX_free.notification ?? [1]) -> 
                    notify_rx = 0;
                    CLT_RX_free.notify_flag = 0;
                    CLT_RX_free.notification ! 1;
                :: else;
            fi;

            if
                // If a packet has been added to the tx used ring and notify flag is set, notify MUX_TX
                :: notify_tx && CLT_TX_used.notify_flag && !(CLT_TX_used.notification ?? [1]) -> 
                    notify_tx = 0;
                    CLT_TX_used.notify_flag = 0;
                    CLT_TX_used.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 2 {
    do
        // Packets have been received
        :: ETH_RX_used.notification ? 1 -> goto mux_rx1;
        :: CLT_RX_free.notification ? 1 ->

            // Transfer received packets from ETH used to CLT used 
            mux_rx1: ;
            bit notify_client = 0;
            bit notify_eth = 0;
            bit work_done = 0;

            d_step {
                do
                    :: !EMPTY(ETH_RX_used) ->
                        // If CLT rx used ring is full, return packet to ETH rx free ring
                        if
                            :: FULL(CLT_RX_used, CHAN_LENGTH) ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                                notify_eth = 1;
                            :: else ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, CLT_RX_used, CHAN_LENGTH);
                                notify_client = 1;
                        fi
                        work_done = 1;
                    :: else -> break;
                od
            }

            ETH_RX_used.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_RX_used) ->
                    ETH_RX_used.notify_flag = 0;
                    goto mux_rx1;
                :: else;
            fi;

            if
                // Notify CLT if packets have been transferred & flag is set
                :: notify_client && CLT_RX_used.notify_flag && !(CLT_RX_used.notification ?? [1]) -> 
                    CLT_RX_used.notify_flag = 0;
                    CLT_RX_used.notification ! 1;
                :: else;
            fi

            mux_rx2: ;
            // Transfer free buffers from CLT free to ETH free
            d_step {
                do
                    :: !EMPTY(CLT_RX_free) ->
                        TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                        notify_eth = 1;
                    :: else -> break;
                od
            }

            CLT_RX_free.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(CLT_RX_free) ->
                    CLT_RX_free.notify_flag = 0;
                    goto mux_rx2;
                :: else;
            fi;

            if
                // Notify ETH if buffers have been transfered & flag is set
                :: notify_eth && ETH_RX_free.notify_flag && !(ETH_RX_free.notification ?? [1]) -> 
                    ETH_RX_free.notify_flag = 0;
                    ETH_RX_free.notification ! 1;
                :: else;
            fi

    od;
}

// MUX Transmit Component
active proctype MUX_TX() priority 2 {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: ETH_TX_free.notification ? 1-> goto mux_tx1;
        :: CLT_TX_used.notification ? 1 ->

        // Return free buffers
        mux_tx1: ;
        bit notify_client = 0;

        d_step {
            do  
                :: !EMPTY(ETH_TX_free) ->
                    assert(!FULL(CLT_TX_free, CHAN_LENGTH));
                    TRANSFER_PACKET(ETH_TX_free, CHAN_LENGTH, CLT_TX_free, CHAN_LENGTH);
                    notify_client = 1;
                :: else -> break;
            od;
        }

        // Ensure no buffers were missed
        ETH_TX_free.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(ETH_TX_free) ->
                ETH_TX_free.notify_flag = 0;
                goto mux_tx1;
            :: else;
        fi;

        if
            :: notify_client && CLT_TX_free.notify_flag && !(CLT_TX_free.notification ?? [1]) -> 
                CLT_TX_free.notify_flag = 0;
                CLT_TX_free.notification ! 1;
            :: else;
        fi

        // Transmit packets
        mux_tx2: ;
        d_step {
            do  
                :: !EMPTY(CLT_TX_used) && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(CLT_TX_used, CHAN_LENGTH, ETH_TX_used, CHAN_LENGTH);
                :: else -> break;
            od;
        }

        CLT_TX_used.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(CLT_TX_used) && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                CLT_TX_used.notify_flag = 0;
                goto mux_tx2;
            :: else;
        fi;

        if
            :: ETH_TX_used.notify_flag && !(ETH_TX_used.notification ?? [1])  -> 
                ETH_TX_used.notify_flag = 0;
                ETH_TX_used.notification ! 1;
            :: else;
        fi

    od;
}


// Ethernet Driver
active proctype ETH() priority 3 {
    do
        // MUX requested a transmission
        :: ETH_TX_used.notification ? 1->
        eth_tx: ;

            d_step {
                do
                    :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                        TRANSFER_PACKET(ETH_TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }

            ETH_TX_used.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                    ETH_TX_used.notify_flag = 0;
                    goto eth_tx;
                :: else;
            fi;

        // MUX has free buffers to receive
        :: ETH_RX_free.notification ? 1 ->
            eth_rx: ;

            // Transfer free buffers from ETH free to HW_RX
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            ETH_RX_free.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                    ETH_RX_free.notify_flag = 0;
                    goto eth_rx;
                :: else;
            fi;

        // Device has sent packets
        :: IRQ_TX ? 1 ->
            bit buffers_transferred = 0;
            d_step {
                do  
                    // Device has buffers in hardware tx ring, some of them have been sent, and tx_free ring is not full
                    :: !EMPTY(HW_TX) && BUFFERS_PROCESSED(HW_TX) && !FULL(ETH_TX_free, CHAN_LENGTH) ->
                        TRANSFER_PACKET(HW_TX, COUNT, ETH_TX_free, CHAN_LENGTH);
                        // Buffers have been transferred
                        buffers_transferred = 1;
                    :: else -> break;
                od
            }

            if  
                // If flag is set and buffers have been transferred notify MUX_TX (if it doesn't have a pending notification)
                :: buffers_transferred && ETH_TX_free.notify_flag && !(ETH_TX_free.notification ?? [1])-> 
                    ETH_TX_free.notify_flag = 0;
                    ETH_TX_free.notification ! 1;
                :: else;
            fi

        // Device has received packets
        :: IRQ_RX ? 1 ->
            bit packets_transferred = 0;
            d_step {
                do
                    // Transfer packets from HW_RX to ETH used
                    :: !EMPTY(HW_RX) && BUFFERS_PROCESSED(HW_RX) && !FULL(ETH_RX_used, CHAN_LENGTH) ->
                        TRANSFER_PACKET(HW_RX, COUNT, ETH_RX_used, CHAN_LENGTH);
                        packets_transferred = 1;
                    :: else -> break;
                od
            }

            if
                // Notify ETH if packets have been transfered & flag is set
                :: packets_transferred && ETH_RX_used.notify_flag && !(ETH_RX_used.notification ?? [1]) ->
                    ETH_RX_used.notify_flag = 0;
                    ETH_RX_used.notification ! 1;
                :: else;
            fi

            eth_rx2: ;

            d_step {
                do
                    // Transfer buffer from ETH free to HW_RX
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            ETH_RX_free.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                    ETH_RX_free.notify_flag = 0;
                    goto eth_rx2;
                :: else;
            fi;
    od
}

// Ethernet Device
active proctype DEV() priority 4 {
    do
        // Receive a packet
        :: !EMPTY(HW_RX) && BUFFERS_UNPROCESSED(HW_RX) ->
            PROCESS_BUFFER(HW_RX);
            
            // Notify ETH if currently un-notified
            if
                :: IRQ_RX ?? [1];
                :: else -> IRQ_RX ! 1;
            fi

        // Transmit a packet
        :: !EMPTY(HW_TX) && BUFFERS_UNPROCESSED(HW_TX) ->
            PROCESS_BUFFER(HW_TX);

            // Notify ETH if currently un-notified
            if
                :: IRQ_TX ?? [1];
                :: else -> IRQ_TX ! 1;
            fi
    od
}
