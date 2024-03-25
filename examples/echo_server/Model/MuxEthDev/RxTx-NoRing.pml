// COMPONENTS: MUX_RX, MUX_TX, ETH, DEV
// SIMPLIFICATIONS: No Rings
// OUTCOME: Could not verify in a reasonable length of time

// Length of rings between MUX and ETH
#define CHAN_LENGTH 3

typedef mux_queue {
    chan notification = [1] of {bit};
    bit notify_flag;
    unsigned head : 2;
    unsigned tail : 2;
}

mux_queue RX_free;
mux_queue RX_used;
mux_queue TX_free;
mux_queue TX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
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
    to_queue.head = (to_queue.head + 1) % to_length; \
    from_queue.tail = (from_queue.tail + 1) % from_length; }

#define BUFFERS_PROCESSED(q) (q.next_buffer != q.tail)

#define BUFFERS_UNPROCESSED(q) (q.next_buffer != q.head)

#define PROCESS_BUFFER(q) q.next_buffer = (q.next_buffer + 1) % COUNT

// Initialisation
init {
    TX_free.notify_flag = 1;
    TX_free.head = 2;

    RX_used.notify_flag = 1;

    HW_RX.head = 2;
}

// MUX Receive Component
active proctype MUX_RX() {
    do
        // Packets have been received
        :: RX_used.notification ? 1 ->

            // Processing packets so no longer need to be notified
            RX_used.notify_flag = 0;

            d_step {
                do
                    // While there are packets to transfer and space to transfer them
                    :: !EMPTY(RX_used) && !FULL(RX_free, CHAN_LENGTH) ->
                        printf("MUX READ: %d\n", RX_used.tail);
                        TRANSFER_PACKET(RX_used, CHAN_LENGTH, RX_free, CHAN_LENGTH);
                    :: else -> break;
                od
            }

            if
                // If notify flag is set for rx ring free, notify ETH
                :: RX_free.notify_flag -> RX_free.notification ! 1;
                :: else;
            fi

            // Will require notification if further packets are received
            RX_used.notify_flag = 1;
    od;
}

// MUX Transmit Component
active proctype MUX_TX() {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent and there are free buffers in TX_free_ring
        :: TX_free.notification ? 1 ->  skip;

        // Client requests a transmission
        :: !EMPTY(TX_free) -> 
            unsigned packets_enqueued : 2 = 0;
            unsigned original_size : 2 = SIZE(TX_used, CHAN_LENGTH);

            d_step {
                do  
                    // Client has buffer to transmit and TX_used ring not full
                    :: !EMPTY(TX_free) && !FULL(TX_used, CHAN_LENGTH) ->
                        printf("MUX SEND: %d\n", TX_free.tail);
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
                        printf("ETH to DEV SEND: %d\n", TX_used.tail);
                        TRANSFER_PACKET(TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }

        // MUX has free buffers to receive
        :: RX_free.notification ? 1 ->

            d_step {
                do
                    // Buffers left to transfer and hardware rx ring not full
                    :: !EMPTY(RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            // If hardware rx is empty, set notify flag to true, otherwise set to false
            if
                :: EMPTY(HW_RX) -> RX_free.notify_flag = 1;
                :: else -> RX_free.notify_flag = 0;
            fi

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

        // Device has received packets
        :: IRQ_RX ? 1 ->
            bit packets_transferred = 0;
            d_step {
                do
                    // Device has received packets and rx_ring used is not full
                    :: !EMPTY(HW_RX) && BUFFERS_PROCESSED(HW_RX) && !FULL(RX_used, CHAN_LENGTH) ->
                        printf("ETH to MUX READ: %d\n", HW_RX.tail);
                        TRANSFER_PACKET(HW_RX, COUNT, RX_used, CHAN_LENGTH);
                        // Packets have been transferred
                        packets_transferred = 1;
                    :: else -> break;
                od
            }

            if
                // If flag is set and packets have been trransferred, notify MUX_RX
                :: packets_transferred && RX_used.notify_flag -> RX_used.notification ! 1;
                :: else;
            fi

            // Attempt to refill HW_RX with free buffers from RX_free
            d_step {
                do
                    // Buffers left to transfer and hardware rx ring not full
                    :: !EMPTY(RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            // If hardware rx is empty, set notify flag to true, otherwise set to false
            if
                :: EMPTY(HW_RX) -> RX_free.notify_flag = 1;
                :: else -> RX_free.notify_flag = 0;
            fi
    od
}

// Ethernet Device
active proctype DEV() {
    do
        // Receive a packet
        :: !EMPTY(HW_RX) && BUFFERS_UNPROCESSED(HW_RX) ->
            printf("DEV READ: %d\n", HW_RX.next_buffer);
            PROCESS_BUFFER(HW_RX);
            
            // Notify ETH if currently un-notified
            if
                :: IRQ_RX ?? [1];
                :: else -> IRQ_RX ! 1;
            fi

        // Transmit a packet
        :: !EMPTY(HW_TX) && BUFFERS_UNPROCESSED(HW_TX) ->
            printf("DEV SEND: %d\n", HW_TX.next_buffer);
            PROCESS_BUFFER(HW_TX);

            // Notify ETH if currently un-notified
            if
                :: IRQ_TX ?? [1];
                :: else -> IRQ_TX ! 1;
            fi
    od
}
