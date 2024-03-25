// COMPONENTS: MUX_TX, ETH, DEV
// SIMPLIFICATIONS: NO RX
// OUTCOME: Verifies quickly (0.07 seconds)

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef ring_entry {
    unsigned entry : 2;
}

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    ring_entry ring[CHAN_LENGTH];
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue TX_free;
mux_queue TX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
    ring_entry ring[COUNT];
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

#define NEXT(q) (q.ring[q.tail].entry)

#define TRANSFER_PACKET(from_queue, from_length, to_queue, to_length) { \
    to_queue.ring[to_queue.head].entry = from_queue.ring[from_queue.tail].entry; \
    from_queue.ring[from_queue.tail].entry = 0; \
    to_queue.head = (to_queue.head + 1) % to_length; \
    from_queue.tail = (from_queue.tail + 1) % from_length; }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

#define TO_PROCESS(q) (q.ring[q.next_buffer].entry)

// Initialisation
init {
    TX_free.notify_flag = 1;
    TX_free.ring[0].entry = 1;
    TX_free.ring[1].entry = 2;
    TX_free.head = 2;
}

// MUX Transmit Component
active proctype MUX_TX() {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent and there are free buffers in TX_free_ring
        :: TX_free.notification ? 1 ->  skip;

        // Client requests a transmission
        :: !EMPTY(TX_free) -> 
            unsigned original_size : 2 = SIZE(TX_used, CHAN_LENGTH);
            unsigned packets_enqueued : 2 = 0;

            d_step {
                do  
                    // Client has buffer to transmit and TX_used ring not full
                    :: !EMPTY(TX_free) && !FULL(TX_used, CHAN_LENGTH) ->
                        printf("MUX SEND: %d\n", NEXT(TX_free));
                        TRANSFER_PACKET(TX_free, CHAN_LENGTH, TX_used, CHAN_LENGTH);

                        // Packet has been enqueued
                        packets_enqueued++;
                    :: else -> break;
                od;
            }

            // If packets have been enqueued AND ring was originally empty OR current size of ring is not equal to 
            // original size + packets enqueued, notify ETH
            unsigned new_size : 2 = SIZE(TX_used, CHAN_LENGTH);
            if
                :: packets_enqueued != 0 && ((original_size==0) || (original_size + packets_enqueued != new_size)) -> TX_used.notification ! 1;
                :: else;
            fi
    od;
}


// Ethernet Driver
active proctype ETH() {
    do
        // MUX requested a transmission
        :: TX_used.notification ? 1->

            d_step {
                do
                    // Packets left to transmit AND hardware tx ring not full
                    :: !EMPTY(TX_used) && !FULL(HW_TX, COUNT) ->
                        printf("ETH to DEV SEND: %d\n", NEXT(TX_used));
                        TRANSFER_PACKET(TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }

        // Device has sent packets
        :: IRQ_TX ? 1 ->
            bit buffers_transferred = 0;
            d_step {
                do  
                    // Device has buffers in hardware tx ring, some of them have been sent, and tx_free ring is not full
                    :: !EMPTY(HW_TX) && BUFFERS_PROCESSED(HW_TX) && !FULL(TX_free, CHAN_LENGTH) ->
                        TRANSFER_PACKET(HW_TX, COUNT, TX_free, CHAN_LENGTH);
                        // Buffers have been transferred
                        buffers_transferred = 1;
                    :: else -> break;
                od
            }

            if  
                // If flag is set and buffers have been transferred notify MUX_TX (if it doesn't have a pending notification)
                :: buffers_transferred && TX_free.notify_flag && !(TX_free.notification ?? [1])-> TX_free.notification ! 1;
                :: else;
            fi
    od
}

// Ethernet Device
active proctype DEV() {
    do
        // Transmit a packet
        :: !EMPTY(HW_TX) && BUFFERS_UNPROCESSED(HW_TX) ->
            printf("DEV SEND: %d\n", TO_PROCESS(HW_TX));
            PROCESS_BUFFER(HW_TX);

            // Notify ETH if currently un-notified
            if
                :: IRQ_TX ?? [1];
                :: else -> IRQ_TX ! 1;
            fi
    od
}
