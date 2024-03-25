// COMPONENTS: MUX_RX, MUX_TX, ETH, DEV
// SIMPLIFICATIONS:
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring if possible, setting flag to 0
// - Extra optimisation where ETH only sets ETH_RX_free notify flag if HW_RX is not full
// OUTCOME: 
// - RX path verifies in 0.24 seconds with no priorities
// - TX path verifies in 0.12 with no priorities
// - Impossible to verify both paths simultaneously in exhaustive mode

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
#define EMPTY(q) (q.tail == q.head)

#define FULL(q, length) ((q.head + 1) % length == q.tail)

#define SIZE(q, length) ((q.head - q.tail + length) % length)

#define TRANSFER_PACKET(from_queue, from_length, to_queue, to_length) { \
    d_step { \
    to_queue.ring[to_queue.head] = from_queue.ring[from_queue.tail]; \
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

#define INSERT_PACKET(to_queue, to_length, stored_count) { \
    d_step { \
    to_queue.ring[to_queue.head] = 1; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    stored_count = stored_count - 1; } \
    }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

// Initialisation
init priority 2 {
    ETH_RX_free.notify_flag = 1;
    ETH_RX_used.notify_flag = 1;
    ETH_TX_free.notify_flag = 1;
    ETH_TX_used.notify_flag = 1;

    // HW_RX.ring[0] = 1;
    // HW_RX.ring[1] = 1;
    // HW_RX.head = 2;

    HW_TX.ring[0] = 1;
    HW_TX.ring[1] = 1;
    HW_TX.head = 2;
}

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    unsigned stored_packets : 2 = 0;
    do
        // Packets have been received
        :: stored_packets > 0 -> goto mux_rx1;
        :: ETH_RX_used.notification ? 1 -> 

            // Transfer received packets from ETH used to CLT used 
            mux_rx1: ;
            bit notify_eth = 0;
            do
                :: !EMPTY(ETH_RX_used) ->
                    // If CLT rx used ring is full, return packet to ETH rx free ring
                    if
                        :: skip ->
                            TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                            notify_eth = 1;
                        :: skip ->
                            REMOVE_PACKET(ETH_RX_used, CHAN_LENGTH, stored_packets);
                    fi
                :: else -> break;
            od

            ETH_RX_used.notify_flag = 1;

            // Ensure no packets were missed
            if  
                :: !EMPTY(ETH_RX_used) ->
                    ETH_RX_used.notify_flag = 0;
                    goto mux_rx1;
                :: else;
            fi;

            mux_rx2: ;
            // Transfer free buffers from CLT free to ETH free
            do
                :: stored_packets > 0 ->
                    INSERT_PACKET(ETH_RX_free, CHAN_LENGTH, stored_packets);
                    notify_eth = 1;
                :: else -> break;
            od

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
active proctype MUX_TX() priority 1 {
    unsigned stored_packets : 2 = 0;
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: stored_packets > 0 -> goto mux_tx1;
        :: ETH_TX_free.notification ? 1-> 

        // Return free buffers
        mux_tx1: ;
        do  
            :: !EMPTY(ETH_TX_free) ->
                REMOVE_PACKET(ETH_TX_free, CHAN_LENGTH, stored_packets);
            :: else -> break;
        od;

        // Ensure no buffers were missed
        ETH_TX_free.notify_flag = 1;

        // Ensure no packets were missed
        if  
            :: !EMPTY(ETH_TX_free) ->
                ETH_TX_free.notify_flag = 0;
                goto mux_tx1;
            :: else;
        fi;

        // Transmit packets
        mux_tx2: ;
        do  
            :: stored_packets > 0 && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                INSERT_PACKET(ETH_TX_used, CHAN_LENGTH, stored_packets);
            :: else -> break;
        od;

        if
            :: ETH_TX_used.notify_flag && !(ETH_TX_used.notification ?? [1])  -> 
                ETH_TX_used.notify_flag = 0;
                ETH_TX_used.notification ! 1;
            :: else;
        fi

    od;
}


// Ethernet Driver
active proctype ETH() priority 1 {
    do
        // MUX requested a transmission
        :: ETH_TX_used.notification ? 1->
            eth_tx: ;
            do
                :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                    TRANSFER_PACKET(ETH_TX_used, CHAN_LENGTH, HW_TX, COUNT);
                :: else -> break
            od

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
            do
                :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                    TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                :: else -> break
            od

            // Only get notified about free buffers if HW_RX is not full
            if 
                :: !FULL(HW_RX, COUNT) -> ETH_RX_free.notify_flag = 1;
                :: else -> ETH_RX_free.notify_flag = 0;
            fi

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
            do  
                // Device has buffers in hardware tx ring, some of them have been sent, and tx_free ring is not full
                :: !EMPTY(HW_TX) && BUFFERS_PROCESSED(HW_TX) && !FULL(ETH_TX_free, CHAN_LENGTH) ->
                    TRANSFER_PACKET(HW_TX, COUNT, ETH_TX_free, CHAN_LENGTH);
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

        // Device has received packets
        :: IRQ_RX ? 1 ->
            bit packets_transferred = 0;
            do
                // Transfer packets from HW_RX to ETH used
                :: !EMPTY(HW_RX) && BUFFERS_PROCESSED(HW_RX) && !FULL(ETH_RX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(HW_RX, COUNT, ETH_RX_used, CHAN_LENGTH);
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

            // Transfer free buffers from ETH free to HW_RX
            goto eth_rx;
    od
}

// Ethernet Device
active proctype DEV() priority 1 {
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
