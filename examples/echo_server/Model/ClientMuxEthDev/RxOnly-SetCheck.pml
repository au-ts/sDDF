// COMPONENTS: CLIENT, MUX_RX, ETH, DEV
// SIMPLIFICATIONS: Client passes packets straight from rx used to rx free
// DEADLOCK SOLUTION: Producer clears flag then signals, consumer processes ring, sets flag, then re-processes ring
// OUTCOME: Verifies in 11.8 seconds

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
    CLT_RX_free.notify_flag = 1;
    CLT_RX_used.notify_flag = 1;
    ETH_RX_free.notify_flag = 1;
    ETH_RX_used.notify_flag = 1;

    HW_RX.ring[0] = 1;
    HW_RX.ring[1] = 1;
    HW_RX.head = 2;
}

// Client Component
active proctype CLIENT() priority 1 {
    bit notify_rx;
    byte no_work_count = 0;
    do
        :: skip ->
            if
                :: CLT_RX_used.notification ? 1 ->
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
                    d_step {
                        do
                            :: !EMPTY(CLT_RX_used) ->
                                TRANSFER_PACKET(CLT_RX_used, CHAN_LENGTH, CLT_RX_free, CHAN_LENGTH);
                                notify_rx = 1;
                            :: else -> break;
                        od 
                    }

                    if
                        :: notify_rx -> no_work_count = 0;
                        :: else -> no_work_count ++;
                    fi
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

// MUX Receive Component
active proctype MUX_RX() priority 1 {
    byte no_work_count = 0;
    do
        :: ETH_RX_used.notification ? 1 -> goto work_rx;
        :: CLT_RX_free.notification ? 1 ->

            // Transfer received packets from ETH used to CLT used 
            work_rx: ;
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
            d_step {
                do
                    :: !EMPTY(ETH_RX_used) ->
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

            if
                // Notify CLT if packets have been transferred & flag is set
                :: notify_client && CLT_RX_used.notify_flag && !(CLT_RX_used.notification ?? [1]) -> 
                    CLT_RX_used.notify_flag = 0;
                    CLT_RX_used.notification ! 1;
                :: else;
            fi

            // Transfer free buffers from CLT free to ETH free
            d_step {
                do
                    :: !EMPTY(CLT_RX_free) ->
                        TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                        notify_eth = 1;
                        work_done = 1;
                    :: else -> break;
                od
            }

            CLT_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            d_step {
                do
                    :: !EMPTY(CLT_RX_free) ->
                        TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                        notify_eth = 1;
                        work_done = 1;
                    :: else -> break;
                od
            }

            if
                // Notify ETH if buffers have been transfered & flag is set
                :: notify_eth && ETH_RX_free.notify_flag && !(ETH_RX_free.notification ?? [1]) -> 
                    ETH_RX_free.notify_flag = 0;
                    ETH_RX_free.notification ! 1;
                :: else;
            fi

            if
                :: work_done -> no_work_count = 0;
                :: else -> no_work_count ++;
            fi
    od;
}

// Ethernet Driver
active proctype ETH() priority 1 {
    byte no_work_count = 0;
    do
        // MUX has free buffers to receive
        :: ETH_RX_free.notification ? 1 ->
            bit work_done = 0;

            // Transfer free buffers from ETH free to HW_RX
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            ETH_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            if
                :: work_done -> no_work_count = 0;
                :: else -> no_work_count ++;
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

            d_step {
                do
                    // Transfer buffer from ETH free to HW_RX
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            ETH_RX_free.notify_flag = 1;

            // Ensure no buffers were missed
            d_step {
                do
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }
    od
}

active proctype DEV() priority 1 {
    do
        // Receive a packet
        :: !EMPTY(HW_RX) && BUFFERS_UNPROCESSED(HW_RX) ->
            PROCESS_BUFFER(HW_RX);
            
            if
                :: IRQ_RX ?? [1];
                :: else -> IRQ_RX ! 1;
            fi
    od
}

ltl CLT1 { [] (CLIENT:no_work_count < 1)};
ltl CLT2 { [] (CLIENT:no_work_count < 2)};

ltl MUX_RX1 { [] (MUX_RX:no_work_count < 1)};
ltl MUX_RX2 { [] (MUX_RX:no_work_count < 2)};

ltl ETH1 { [] (ETH:no_work_count < 1)};
ltl ETH2 { [] (ETH:no_work_count < 2)};