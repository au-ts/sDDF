// COMPONENTS: CLIENT, MUX_TX, ETH, DEV
// SIMPLIFICATIONS: Client passes packets straight from tx free to tx used 
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring
// OUTCOME: Verifies in 4.73 seconds

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    bit ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue ETH_TX_free;
mux_queue ETH_TX_used;

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
    CLT_TX_used.ring[0] = 1;
    CLT_TX_used.ring[1] = 1;
    CLT_TX_used.head = 2;

    CLT_TX_free.notify_flag = 1;
    CLT_TX_used.notify_flag = 1;
    ETH_TX_free.notify_flag = 1;
    ETH_TX_used.notify_flag = 1;

    CLT_TX_used.notification ! 1;
}

// Client Component
active proctype CLIENT() priority 1 {
    bit notify_tx;
    do
        :: skip ->
            if
                // Packets have been sent
                :: CLT_TX_free.notification ? 1 ->

                    d_step {
                        do
                            :: !EMPTY(CLT_TX_free) && !FULL(CLT_TX_used, CHAN_LENGTH) ->
                                TRANSFER_PACKET(CLT_TX_free, CHAN_LENGTH, CLT_TX_used, CHAN_LENGTH);
                                notify_tx = 1;
                            :: else -> break;
                        od 
                    }
                    
                    CLT_TX_free.notify_flag = 1;

                    // Ensure no buffers were missed
                    d_step {
                        do
                            :: !EMPTY(CLT_TX_free) && !FULL(CLT_TX_used, CHAN_LENGTH) ->
                                TRANSFER_PACKET(CLT_TX_free, CHAN_LENGTH, CLT_TX_used, CHAN_LENGTH);
                                notify_tx = 1;
                            :: else -> break;
                        od 
                    }
            fi;

            if
                // If a packet has been added to the tx used ring and notify flag is set, notify MUX_TX
                :: notify_tx && CLT_TX_used.notify_flag  && !(CLT_TX_used.notification ?? [1]) -> 
                    notify_tx = 0;
                    CLT_TX_used.notify_flag = 0;
                    CLT_TX_used.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Transmit Component
active proctype MUX_TX() priority 1 {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: ETH_TX_free.notification ? 1-> goto work_tx;
        :: CLT_TX_used.notification ? 1 ->

        // Return free buffers
        work_tx: ;
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

        d_step {
            do  
                :: !EMPTY(ETH_TX_free) ->
                    assert(!FULL(CLT_TX_free, CHAN_LENGTH));
                    TRANSFER_PACKET(ETH_TX_free, CHAN_LENGTH, CLT_TX_free, CHAN_LENGTH);
                    notify_client = 1;
                :: else -> break;
            od;
        }

        if
            :: notify_client && CLT_TX_free.notify_flag && !(CLT_TX_free.notification ?? [1]) -> 
                CLT_TX_free.notify_flag = 0;
                CLT_TX_free.notification ! 1;
            :: else;
        fi

        // Transmit packets
        d_step {
            do  
                :: !EMPTY(CLT_TX_used) && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(CLT_TX_used, CHAN_LENGTH, ETH_TX_used, CHAN_LENGTH);
                :: else -> break;
            od;
        }

        CLT_TX_used.notify_flag = 1;

        // Ensure no buffers were missed
        d_step {
            do  
                :: !EMPTY(CLT_TX_used) && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(CLT_TX_used, CHAN_LENGTH, ETH_TX_used, CHAN_LENGTH);
                :: else -> break;
            od;
        }

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

            d_step {
                do
                    :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                        TRANSFER_PACKET(ETH_TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }

            ETH_TX_used.notify_flag = 1;

            // Ensure no buffers were missed
            d_step {
                do
                    :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                        TRANSFER_PACKET(ETH_TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }

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
    od
}

// Ethernet Device
active proctype DEV() priority 1 {
    do
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
