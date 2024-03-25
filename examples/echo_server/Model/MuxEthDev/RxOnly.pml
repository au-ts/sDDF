// COMPONENTS: MUX_RX, ETH, DEV
// SIMPLIFICATIONS: NO TX
// OUTCOMES: Found 2 x Deadlocks quickly! Once deadlocks fixed + notifications set as none blocking, verifies quickly (0.03 seconds)
// FIXES: Set RX_used.notify_flag == 1 constant to remove *real* deadlock, and RX_free.notify_flag == 1 to remove 
//        *false* deadlock (would not occur with real priorities)

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

mux_queue RX_free;
mux_queue RX_used;

// Length of rings between ETH and DEV
#define COUNT 3

typedef dev_queue {
    ring_entry ring[COUNT];
    unsigned head : 2;
    unsigned tail : 2;
    unsigned next_buffer: 2;
}

chan IRQ_RX = [1] of {bit};
chan IRQ_TX = [1] of {bit};

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
    RX_used.notify_flag = 1;
    // Fixed
    RX_free.notify_flag = 1;

    HW_RX.ring[0].entry = 1;
    HW_RX.ring[1].entry = 2;
    HW_RX.head = 2;
}

// MUX Receive Component
active proctype MUX_RX() {
    do
        // Packets have been received
        :: RX_used.notification ? 1 ->

            // Processing packets so no longer need to be notified
            // RX_used.notify_flag = 0;

            d_step {
                do
                    // While there are packets to transfer and space to transfer them
                    :: !EMPTY(RX_used) && !FULL(RX_free, CHAN_LENGTH) ->
                        printf("MUX READ: %d\n", NEXT(RX_used));
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
            // RX_used.notify_flag = 1;
    od;
}

// Ethernet Driver
active proctype ETH() {
    do
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
            // if
            //     :: EMPTY(HW_RX) -> RX_free.notify_flag = 1;
            //     :: else -> RX_free.notify_flag = 0;
            // fi

        // Device has received packets
        :: IRQ_RX ? 1 ->
            bit packets_transferred = 0;
            d_step {
                do
                    // Device has received packets and rx_ring used is not full
                    :: !EMPTY(HW_RX) && BUFFERS_PROCESSED(HW_RX) && !FULL(RX_used, CHAN_LENGTH) ->
                        printf("ETH to MUX READ: %d\n", NEXT(HW_RX));
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
            // if
            //     :: EMPTY(HW_RX) -> RX_free.notify_flag = 1;
            //     :: else -> RX_free.notify_flag = 0;
            // fi
    od
}

// Ethernet Device
active proctype DEV() {
    do
        // Receive a packet
        :: !EMPTY(HW_RX) && BUFFERS_UNPROCESSED(HW_RX) ->
            printf("DEV READ: %d\n", TO_PROCESS(HW_RX));
            PROCESS_BUFFER(HW_RX);
            
            // Notify ETH if currently un-notified
            if
                :: IRQ_RX ?? [1];
                :: else -> IRQ_RX ! 1;
            fi
    od
}
