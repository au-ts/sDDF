// COMPONENTS: CLIENT, COPY, MUX_RX
// SIMPLIFICATIONS: Client passes packets straight from rx used to rx free
// DEADLOCK SOLUTION: 
// OUTCOME: Verifies in 5.33 seconds

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    bit ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue CPY_RX_free;
mux_queue CPY_RX_used;

mux_queue CLT_RX_free;
mux_queue CLT_RX_used;

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
    CLT_RX_used.notify_flag = 1;
    CPY_RX_used.notify_flag = 1;
    CPY_RX_free.notify_flag = 1;

    CLT_RX_free.head = 2;
}

// Client Component
active proctype CLIENT() priority 1 {
    bit notify_rx;
    do
        :: skip ->
            if
                :: CLT_RX_used.notification ? 1 ->
                    clt: ;
                    do
                        :: !EMPTY(CLT_RX_used) ->
                            TRANSFER_PACKET(CLT_RX_used, CHAN_LENGTH, CLT_RX_free, CHAN_LENGTH);
                            notify_rx = 1;
                        :: else -> break;
                    od
            fi;

            CLT_RX_used.notify_flag = 1;

            if 
                :: !EMPTY(CLT_RX_used) ->
                    CLT_RX_used.notify_flag = 0;
                    goto clt;
                :: else;
            fi

            if
                :: notify_rx && CLT_RX_free.notify_flag && !(CLT_RX_free.notification ?? [1]) -> 
                    notify_rx = 0;
                    CLT_RX_free.notify_flag = 0;
                    CLT_RX_free.notification ! 1;
                :: else;
            fi;
    od;
}

// Client Component
active proctype COPY() priority 1 {
    do
        :: CPY_RX_used.notification ? 1 -> goto work_copy_start;
        :: CLT_RX_free.notification ? 1 ->
            work_copy_start: ;
            bit enqueued;
            work_copy: ;
            do
                :: !EMPTY(CPY_RX_used) && !EMPTY(CLT_RX_free) && !FULL(CLT_RX_used, CHAN_LENGTH) && !FULL(CPY_RX_free, CHAN_LENGTH) ->
                    TRANSFER_PACKET(CPY_RX_used, CHAN_LENGTH, CPY_RX_free, CHAN_LENGTH);
                    TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, CLT_RX_used, CHAN_LENGTH);
                    enqueued = 1;
                :: else -> break;
            od;

            // Always get notified when packets are received
            CPY_RX_used.notify_flag = 1;

            if 
                // Only get notified when free buffers are returned when there are packets to transfer
                :: !EMPTY(CPY_RX_used) -> CLT_RX_free.notify_flag = 1;
                :: else -> CLT_RX_free.notify_flag = 0;
            fi;

            if 
                :: !EMPTY(CPY_RX_used) && !EMPTY(CLT_RX_free) && !FULL(CLT_RX_used, CHAN_LENGTH) && !FULL(CPY_RX_free, CHAN_LENGTH) ->
                    CPY_RX_used.notify_flag = 0;
                    CLT_RX_free.notify_flag = 0;
                    goto work_copy;
                :: else;
            fi

            if 
                :: CLT_RX_used.notify_flag && enqueued && !(CLT_RX_used.notification ?? [1]) -> 
                    CLT_RX_used.notify_flag = 0;
                    CLT_RX_used.notification ! 1;
                :: else;
            fi;

            if 
                :: CPY_RX_free.notify_flag && enqueued && !(CPY_RX_free.notification ?? [1]) -> 
                    CPY_RX_free.notify_flag = 0;
                    CPY_RX_free.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    unsigned stored_packets : 2 = 2;
    do
        // Packets have been "received"
        :: stored_packets > 0 -> goto work_rx;
        :: CPY_RX_free.notification ? 1 ->

            // Transfer received packets to clients
            work_rx: ;
            bit notify_client = 0;
            bit notify_eth = 0;
            do
                :: stored_packets > 0 ->
                    INSERT_PACKET(CPY_RX_used, CHAN_LENGTH, stored_packets);
                    notify_client = 1;
                :: else -> break;
            od

            if
                // If client has received a packet and notify flag is set, notify client
                :: notify_client && CPY_RX_used.notify_flag && !(CPY_RX_used.notification ?? [1]) -> 
                    CPY_RX_used.notify_flag = 0;
                    CPY_RX_used.notification ! 1;
                :: else;
            fi

            // Transfer buffers to eth
            mux_rx: ;
            do
                // While there are packets to transfer
                :: !EMPTY(CPY_RX_free) ->
                    REMOVE_PACKET(CPY_RX_free, CHAN_LENGTH, stored_packets);
                :: else -> break;
            od

            CPY_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            if
                :: !EMPTY(CPY_RX_free) -> 
                    CPY_RX_free.notify_flag = 0;
                    goto mux_rx;
                :: else;
            fi
    od;
}
