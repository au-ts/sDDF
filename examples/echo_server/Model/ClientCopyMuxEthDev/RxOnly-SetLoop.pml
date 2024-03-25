// COMPONENTS: CLIENT, COPY, MUX_RX, ETH, DEV
// SIMPLIFICATIONS: Client passes packets straight from rx used to rx free
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring
// if possible. Before looping back, re-sets flag to 0 to avoid unnecessary signalling.
// OPTIMISATIONS: ETH only sets rx free ring flag if HW_RX is not full
// OUTCOME: 

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

mux_queue CPY_RX_free;
mux_queue CPY_RX_used;

mux_queue CLT_RX_free;
mux_queue CLT_RX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
    bit ring[COUNT];
    unsigned head : 2;
    unsigned tail : 2;
    unsigned next_buffer: 2;
}

chan IRQ_RX = [1] of {bit};

dev_queue HW_RX;

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
    to_queue.ring[to_queue.head] = from_queue.ring[from_queue.tail]; \
    from_queue.ring[from_queue.tail] = 0; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    from_queue.tail = (from_queue.tail + 1) % from_length; }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

// Initialisation
init {
    CLT_RX_used.notify_flag = 1;
    CPY_RX_used.notify_flag = 1;
    ETH_RX_used.notify_flag = 1;

    CLT_RX_free.head = 2;

    HW_RX.ring[0] = 1;
    HW_RX.ring[1] = 1;
    HW_RX.head = 2;
}

// Client Component
active proctype CLIENT() priority 1 {
    bit notify_rx;
    do
        :: skip ->
            if
                :: CLT_RX_used.notification ? 1 ->

                    clt_used_free: ;
                    // Transfer packets from used to free
                    d_step {
                        do
                            :: !EMPTY(CLT_RX_used) ->
                                TRANSFER_PACKET(CLT_RX_used, CHAN_LENGTH, CLT_RX_free, CHAN_LENGTH);
                                notify_rx = 1;
                            :: else -> break;
                        od 
                    }

                    CLT_RX_used.notify_flag = 1;

                    // Ensure no packets were missed
                    if  
                        :: !EMPTY(CLT_RX_used) ->
                            CLT_RX_used.notify_flag = 0;
                            goto clt_used_free;
                        :: else;
                    fi;

            fi;

            if
                // Notify MUX_RX if packets have been transferred & flag is set
                :: notify_rx && CLT_RX_free.notify_flag && !(CLT_RX_free.notification ?? [1])-> 
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
        // Packets have been received (always notified) or free buffers available
        :: CPY_RX_used.notification ? 1 -> goto work_copy;
        :: CLT_RX_free.notification ? 1 ->
            work_copy: ;
            bit enqueued;

            copy_used_free_used_free: ;
            d_step{
                do
                    :: !EMPTY(CPY_RX_used) && !EMPTY(CLT_RX_free) && !FULL(CLT_RX_used, CHAN_LENGTH) && !FULL(CPY_RX_free, CHAN_LENGTH) ->
                        TRANSFER_PACKET(CPY_RX_used, CHAN_LENGTH, CPY_RX_free, CHAN_LENGTH);
                        TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, CLT_RX_used, CHAN_LENGTH);
                        enqueued = 1;
                    :: else -> break;
                od;
            }

            CPY_RX_used.notify_flag = 1;
            CLT_RX_free.notify_flag = 1;

            // Ensure no packets were missed
            if
                :: !EMPTY(CPY_RX_used) && !EMPTY(CLT_RX_free) && !FULL(CLT_RX_used, CHAN_LENGTH) && !FULL(CPY_RX_free, CHAN_LENGTH) ->
                    CPY_RX_used.notify_flag = 0;
                    CLT_RX_free.notify_flag = 0;
                    goto copy_used_free_used_free;
                :: else;
            fi

            if 
                :: CLT_RX_used.notify_flag && enqueued -> 
                    CLT_RX_used.notify_flag = 0;
                    CLT_RX_used.notification ! 1;
                :: else;
            fi;

            if 
                :: CPY_RX_free.notify_flag && enqueued -> 
                    CPY_RX_free.notify_flag = 0;
                    CPY_RX_free.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    do
        // Packets have been received
        :: ETH_RX_used.notification ? 1 -> goto work_rx;
        :: CPY_RX_free.notification ? 1 ->

            // Transfer received packets to clients
            work_rx: ;
            bit notify_client = 0;
            bit notify_eth = 0;

            mux_used_used: ;
            d_step {
                do
                    // While there are packets to transfer
                    :: !EMPTY(ETH_RX_used) ->
                        // If client rx used ring is full, return packet to eth rx free ring
                        if
                            :: FULL(CPY_RX_used, CHAN_LENGTH) ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                                notify_eth = 1;
                            :: else ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, CPY_RX_used, CHAN_LENGTH);
                                notify_client = 1;
                        fi
                    :: else -> break;
                od
            }

            ETH_RX_used.notify_flag = 1;

            // Ensure no packets were missed
            if
                :: !EMPTY(ETH_RX_used) ->
                    ETH_RX_used.notify_flag = 0;
                    goto mux_used_used;
                :: else;
            fi

            if
                // Notify COPY if packets have been transferred & flag is set
                :: notify_client && CPY_RX_used.notify_flag && !(CPY_RX_used.notification ?? [1]) -> 
                    CPY_RX_used.notify_flag = 0;
                    CPY_RX_used.notification ! 1;
                :: else;
            fi

            // Transfer free buffers from CLT free to ETH free
            mux_free_free: ;
            d_step {
                do
                    :: !EMPTY(CPY_RX_free) ->
                        TRANSFER_PACKET(CPY_RX_free, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                        notify_eth = 1;
                    :: else -> break;
                od
            }

            CPY_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            if
                :: !EMPTY(CPY_RX_free) -> 
                    CPY_RX_free.notify_flag = 0;
                    goto mux_free_free;
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


// Ethernet Driver
active proctype ETH() priority 1 {
    do
        // MUX has free buffers to receive
        :: ETH_RX_free.notification ? 1 ->

            // Transfer free buffers from ETH free to HW_RX
            eth_free_hw: ;
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            // Only set flag to 1 if HW_RX not full
            if
                :: !FULL(HW_RX, COUNT) ->
                    ETH_RX_free.notify_flag = 1;
                :: else ->
                    ETH_RX_free.notify_flag = 0;
            fi

            // Ensure no buffers were missed
            if
                :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) -> 
                    ETH_RX_free.notify_flag = 0;
                    goto eth_free_hw;
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

            // Transfer free buffers from ETH free to HW_RX
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            // Only set flag to 1 if HW_RX not full
            if
                :: !FULL(HW_RX, COUNT) ->
                    ETH_RX_free.notify_flag = 1;
                :: else ->
                    ETH_RX_free.notify_flag = 0;
            fi

            // Ensure no buffers were missed
            if
                :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) -> 
                    ETH_RX_free.notify_flag = 0;
                    goto eth_free_hw;
                :: else;
            fi
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
    od
}
