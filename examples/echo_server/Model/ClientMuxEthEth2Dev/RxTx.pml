// COMPONENTS: LINKED_LIST_CLIENT, MUX_RX, MUX_TX, ETH, ETH2, DEV (SPLIT DRIVER)
// SIMPLIFICATIONS:
// OUTCOME: With priorities, usual deadlock where client does not set TX free notify flag, 
// causing MUX_TX not to set ETH TX free notify flag, causing an eventual TX deadlock.
// This causes client to not return RX free buffers, causing RX deadlock also.

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

                    // If there are still packets in curr, set notify flag for tx free
                    if
                        :: CURR_EMPTY(cur_size) -> CLT_TX_free.notify_flag = 0;
                        :: else -> CLT_TX_free.notify_flag = 1;
                    fi

                    // Process rx used queue
                    goto process_rx;

                // Packets have been received
                :: CLT_RX_used.notification ? 1 ->
                    process_rx: ;
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
            fi;

            if
                // If buffers have been added to the rx free ring and notify flag is set, notify MUX_RX
                :: notify_rx && CLT_RX_free.notify_flag && !(CLT_RX_free.notification ?? [1]) -> 
                    notify_rx = 0;
                    CLT_RX_free.notification ! 1;
                :: else;
            fi;

            if
                // If a packet has been added to the tx used ring and notify flag is set, notify MUX_TX
                :: notify_tx && CLT_TX_used.notify_flag && !(CLT_TX_used.notification ?? [1]) -> 
                    notify_tx = 0;
                    CLT_TX_used.notification ! 1;
                :: else;
            fi;
    od;
}

// MUX Receive Component
active proctype MUX_RX() priority 2 {
    do
        // Packets have been received
        :: ETH_RX_used.notification ? 1 -> goto work_rx;
        :: CLT_RX_free.notification ? 1 ->

            // Transfer received packets to clients
            work_rx: ;
            bit notify_client = 0;
            bit notify_eth = 0;
            d_step {
                do
                    // While there are packets to transfer
                    :: !EMPTY(ETH_RX_used) ->
                        // If client rx used ring is full, return packet to eth rx free ring
                        if
                            :: FULL(CLT_RX_used, CHAN_LENGTH) ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                                notify_eth = 1;
                            :: else ->
                                TRANSFER_PACKET(ETH_RX_used, CHAN_LENGTH, CLT_RX_used, CHAN_LENGTH);
                                notify_client = 1;
                        fi
                    :: else -> break;
                od
            }

            if
                // If client has received a packet and notify flag is set, notify client
                :: notify_client && CLT_RX_used.notify_flag && !(CLT_RX_used.notification ?? [1]) -> CLT_RX_used.notification ! 1;
                :: else;
            fi

            if
                // If ETH rx free ring notify flag is set, set client rx free notify flag
                :: ETH_RX_free.notify_flag -> CLT_RX_free.notify_flag = 1;
                :: else -> CLT_RX_free.notify_flag = 0;
            fi

            // Transfer buffers to eth
            d_step {
                do
                    // While there are packets to transfer
                    :: !EMPTY(CLT_RX_free) ->
                        TRANSFER_PACKET(CLT_RX_free, CHAN_LENGTH, ETH_RX_free, CHAN_LENGTH);
                        notify_eth = 1;
                    :: else -> break;
                od
            }

            if
                // If free packets have been enqueued and notify flag is set, notify client
                :: notify_eth && ETH_RX_free.notify_flag && !(ETH_RX_free.notification ?? [1]) -> ETH_RX_free.notification ! 1;
                :: else;
            fi

    od;
}

// MUX Transmit Component
active proctype MUX_TX() priority 2 {
    do
        // MUX_TX has awoken by ETH indicating packets have been sent OR CLIENT requesting to send packets
        :: ETH_TX_free.notification ? 1-> goto work_tx;
        :: CLT_TX_used.notification ? 1 ->

        // Return free buffers
        work_tx: ;
        bit notify_client = 0;

        d_step {
            do  
                // Driver has buffers to return
                :: !EMPTY(ETH_TX_free) ->
                    assert(!FULL(CLT_TX_free, CHAN_LENGTH));
                    TRANSFER_PACKET(ETH_TX_free, CHAN_LENGTH, CLT_TX_free, CHAN_LENGTH);
                    notify_client = 1;
                :: else -> break;
            od;
        }

        if
            :: notify_client && CLT_TX_free.notify_flag && !(CLT_TX_free.notification ?? [1]) -> CLT_TX_free.notification ! 1;
            :: else;
        fi

        // Transmit packets
        d_step {
            do  
                // Client has buffer to transmit and TX_used ring not full
                :: !EMPTY(CLT_TX_used) && !FULL(ETH_TX_used, CHAN_LENGTH) ->
                    TRANSFER_PACKET(CLT_TX_used, CHAN_LENGTH, ETH_TX_used, CHAN_LENGTH);
                :: else -> break;
            od;
        }

        if
            :: ETH_TX_used.notify_flag && !(ETH_TX_used.notification ?? [1]) -> ETH_TX_used.notification ! 1;
            :: else;
        fi

        // Set tx free notify flag for ETH if client requires notifying
        if
            :: CLT_TX_free.notify_flag -> ETH_TX_free.notify_flag = 1;
            :: else -> ETH_TX_free.notify_flag = 0;
        fi
    od;
}


// Ethernet Driver (handles everying except transmission)
active proctype ETH() priority 3 {
    do
        // MUX has free buffers to receive
        :: ETH_RX_free.notification ? 1 ->
            fill_rx_bufs: ;

            d_step {
                do
                    // Buffers left to transfer and hardware rx ring not full
                    :: !EMPTY(ETH_RX_free) && !FULL(HW_RX, COUNT) ->
                        TRANSFER_PACKET(ETH_RX_free, CHAN_LENGTH, HW_RX, COUNT);
                    :: else -> break
                od
            }

            // If hardware rx is empty, set notify flag to true, otherwise set to false
            if
                :: !EMPTY(HW_RX) -> ETH_RX_free.notify_flag = 0;
                :: else -> 
                    ETH_RX_free.notify_flag = 1;
                    // This was added to ensure the MUX_RX noticed the change of flag to avoid deadlock. May not be necessary now
                    if
                        :: !(ETH_RX_used.notification ?? [1]) -> ETH_RX_used.notification ! 1;
                        :: else;
                    fi
            fi

        // Device has sent packets
        :: IRQ_TX ? 1 ->
            bit buffers_transferred = 0;
            bit hw_was_full = FULL(HW_TX, COUNT);
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
                :: buffers_transferred && ETH_TX_free.notify_flag && !(ETH_TX_free.notification ?? [1])-> ETH_TX_free.notification ! 1;
                :: else;
            fi

            if  
                :: hw_was_full && buffers_transferred && !(ETH_TX_used.notification ?? [1])-> ETH_TX_used.notification ! 1;
                :: else;
            fi            

        // Device has received packets
        :: IRQ_RX ? 1 ->
            bit packets_transferred = 0;
            d_step {
                do
                    // Device has received packets and rx_ring used is not full
                    :: !EMPTY(HW_RX) && BUFFERS_PROCESSED(HW_RX) && !FULL(ETH_RX_used, CHAN_LENGTH) ->
                        TRANSFER_PACKET(HW_RX, COUNT, ETH_RX_used, CHAN_LENGTH);
                        // Packets have been transferred
                        packets_transferred = 1;
                    :: else -> break;
                od
            }

            if
                // If flag is set and packets have been trransferred, notify MUX_RX
                :: packets_transferred && ETH_RX_used.notify_flag && !(ETH_RX_used.notification ?? [1]) -> ETH_RX_used.notification ! 1;
                :: else;
            fi

            goto fill_rx_bufs;
    od
}

// Ethernet Driver Component 2 - handles transmitting messages for the client
active proctype ETH2() priority 4 {
    do
        // MUX requested a transmission
        :: ETH_TX_used.notification ? 1->

            d_step {
                do
                    // Packets left to transmit AND hardware tx ring not full
                    :: !EMPTY(ETH_TX_used) && !FULL(HW_TX, COUNT) ->
                        TRANSFER_PACKET(ETH_TX_used, CHAN_LENGTH, HW_TX, COUNT);
                    :: else -> break
                od
            }
    od
}


// Ethernet Device
active proctype DEV() priority 5 {
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
