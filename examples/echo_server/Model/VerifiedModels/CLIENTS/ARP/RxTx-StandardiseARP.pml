// COMPONENTS: ARP, MUX_RX (ARP talks directly to MUX_RX - no COPY), MUX_TX
// SIMPLIFICATIONS:
// ASSUMPTIONS:
// - We assume that ARP does not crash if there are no transmit free buffers left - it just does not send a reply (this seems to be true?)
// DEADLOCK SOLUTIONS: 
// - ARP notifies MUX_RX when free buffers are returned to the free ring
// OUTCOME: Verifies in 0.63 seconds

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    bit ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue CLT_RX_free;
mux_queue CLT_RX_used;
mux_queue CLT_TX_free;
mux_queue CLT_TX_used;

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

// Initialisation
init priority 2 {
    CLT_TX_free.ring[0] = 1;
    CLT_TX_free.ring[1] = 1;
    CLT_TX_free.head = 2;

    CLT_TX_used.notify_flag = 1;
    CLT_RX_used.notify_flag = 1;

    CLT_RX_free.notify_flag = 1;
}

// Client Component
active proctype ARP() priority 1 {
    do
        :: CLT_RX_used.notification ? 1 ->
            bit transmitted;
            bit received;
            do
                :: !EMPTY(CLT_RX_used) ->
                    if
                        :: !EMPTY(CLT_TX_free) && !FULL(CLT_TX_used, CHAN_LENGTH) ->
                            TRANSFER_PACKET(CLT_TX_free, CHAN_LENGTH, CLT_TX_used, CHAN_LENGTH);
                            transmitted = 1;
                        :: true;
                    fi;
                    TRANSFER_PACKET(CLT_RX_used, CHAN_LENGTH, CLT_RX_free, CHAN_LENGTH);
                    received = 1;
                :: else -> break;
            od 

            if
                :: transmitted && !(CLT_TX_used.notification ?? [1]) -> CLT_TX_used.notification ! 1;
                :: else;
            fi

            if
                :: received && !(CLT_RX_free.notification ?? [1]) -> CLT_RX_free.notification ! 1;
                :: else;
            fi
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    unsigned stored_packets : 2 = 2;
    do
        // Packets have been "received"
        :: stored_packets > 0 -> goto work_rx;
        :: CLT_RX_free.notification ? 1 ->

            // Transfer received packets to clients
            work_rx: ;
            bit notify_client = 0;
            do
                :: stored_packets > 0 ->
                    INSERT_PACKET(CLT_RX_used, CHAN_LENGTH, stored_packets);
                    notify_client = 1;
                :: else -> break;
            od

            if
                // If client has received a packet and notify flag is set, notify client
                :: notify_client && CLT_RX_used.notify_flag && !(CLT_RX_used.notification ?? [1]) -> CLT_RX_used.notification ! 1;
                :: else;
            fi

            // Transfer buffers to eth
            do
                // While there are packets to transfer
                :: !EMPTY(CLT_RX_free) ->
                    REMOVE_PACKET(CLT_RX_free, CHAN_LENGTH, stored_packets);
                :: else -> break;
            od
    od;
}

// MUX Transmit Component
active proctype MUX_TX() priority 1 {
    unsigned stored_packets : 2 = 0;
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: stored_packets > 0 -> goto work_tx;
        :: CLT_TX_used.notification ? 1 ->

        // Return free buffers
        work_tx: ;
        bit notify_client = 0;
        do  
            :: stored_packets > 0 ->
                assert(!FULL(CLT_TX_free, CHAN_LENGTH));
                INSERT_PACKET(CLT_TX_free, CHAN_LENGTH, stored_packets);
                notify_client = 1;
            :: else -> break;
        od;

        if
            :: notify_client && CLT_TX_free.notify_flag && !(CLT_TX_free.notification ?? [1]) -> CLT_TX_free.notification ! 1;
            :: else;
        fi

        // Transmit packets
        do  
            // Client has buffer to transmit and TX_used ring not full
            :: !EMPTY(CLT_TX_used) ->
                REMOVE_PACKET(CLT_TX_used, CHAN_LENGTH, stored_packets);
            :: else -> break;
        od;
    od;
}
