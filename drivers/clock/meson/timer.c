#include <stdint.h>
#include <microkit.h>
#include <sddf/util/printf.h>

uintptr_t gpt_regs;

#define IRQ_CH 0
#define GET_TIME 0
#define SET_TIMEOUT 1
#define MAX_TIMEOUTS 6

#define TIMER_REG_START   0x140    // TIMER_MUX

#define TIMER_A_INPUT_CLK 0
#define TIMER_E_INPUT_CLK 8
#define TIMER_A_EN      (1 << 16)
#define TIMER_A_MODE    (1 << 12)

#define TIMESTAMP_TIMEBASE_SYSTEM   0b000
#define TIMESTAMP_TIMEBASE_1_US     0b001
#define TIMESTAMP_TIMEBASE_10_US    0b010
#define TIMESTAMP_TIMEBASE_100_US   0b011
#define TIMESTAMP_TIMEBASE_1_MS     0b100

#define TIMEOUT_TIMEBASE_1_US   0b00
#define TIMEOUT_TIMEBASE_10_US  0b01
#define TIMEOUT_TIMEBASE_100_US 0b10
#define TIMEOUT_TIMEBASE_1_MS   0b11

/* Number of nanoseconds in microseconds */
#define NS_IN_US    1000ULL
/* Number of nanoseconds in milliseconds */
#define NS_IN_MS    1000000ULL

struct timer_regs {
    uint32_t mux;
    uint32_t timer_a;
    uint32_t timer_b;
    uint32_t timer_c;
    uint32_t timer_d;
    uint32_t unused[13];
    uint32_t timer_e;
    uint32_t timer_e_hi;
    uint32_t mux1;
    uint32_t timer_f;
    uint32_t timer_g;
    uint32_t timer_h;
    uint32_t timer_i;
};

volatile struct timer_regs *regs;

/* Right now, we only service a single timeout per client.
 * This timeout array indicates when a timeout should occur,
 * indexed by client ID. */
static uint64_t timeouts[MAX_TIMEOUTS];
static microkit_channel active_channel = -1;
static bool timeout_active = false;
static uint64_t current_timeout;
static uint8_t pending_timeouts;

static uint64_t
get_ticks(void)
{
    uint64_t initial_high = regs->timer_e_hi;
    uint64_t low = regs->timer_e;
    uint64_t high = regs->timer_e_hi;
    if (high != initial_high) {
        low = regs->timer_e;
    }

    uint64_t ticks = (high << 32) | low;
    return ticks * NS_IN_US;
}

static void
set_timeout(uint32_t timeout) {
    regs->mux &= ~TIMER_A_MODE;
    regs->timer_a = timeout;
    regs->mux |= TIMER_A_EN;
}

static void
handle_irq(microkit_channel ch)
{
    if (timeout_active) {
        regs->mux &= ~TIMER_A_EN;
        timeout_active = false;
        microkit_channel curr_channel = active_channel;
        timeouts[curr_channel] = 0;
        // notify the client.
        microkit_notify(curr_channel);
    }

    if (pending_timeouts && !timeout_active) {
        uint64_t curr_time = get_ticks();
        /* find next timeout */
        uint64_t next_timeout = UINT64_MAX;
        microkit_channel ch = -1;

        /* A more efficient solution would be to order these in terms of
         * timeout time, so then we can just take the head as the next timeout.
         * However, this would require a different data structure...
         */
        for (unsigned i = 0; i < MAX_TIMEOUTS; i++) {
            /* Check if any of these timeouts have gone off in the interim */
            if (timeouts[i] != 0 && timeouts[i] <= curr_time) {
                timeouts[i] = 0;
                pending_timeouts--;
                microkit_notify(i);
            } else if (timeouts[i] != 0 && timeouts[i] < next_timeout) {
                next_timeout = timeouts[i];
                ch = i;
            }
        }

        if (ch != -1) {
            pending_timeouts--;
            set_timeout((uint32_t)((next_timeout - curr_time) / NS_IN_MS));
            timeout_active = true;
            current_timeout = next_timeout;
            active_channel = ch;
        }
    }
}

void
notified(microkit_channel ch)
{
    if (ch == IRQ_CH) {
        handle_irq(ch);
        microkit_irq_ack_delayed(ch);
    } else {
        sddf_dprintf("TIMER DRIVER|LOG: unexpected notification %llu\n", ch);
    }
}

seL4_MessageInfo_t
protected(microkit_channel ch, microkit_msginfo msginfo)
{
    uint64_t rel_timeout, cur_ticks, abs_timeout;
    switch (microkit_msginfo_get_label(msginfo)) {
        case GET_TIME:
            // Just wants the time. Return it in nanoseconds.
            cur_ticks = get_ticks();
            seL4_SetMR(0, cur_ticks);
            return microkit_msginfo_new(0, 1);
        case SET_TIMEOUT:
            // Request to set a timeout.
            rel_timeout = (uint64_t)(seL4_GetMR(0));
            cur_ticks = get_ticks();
            abs_timeout = cur_ticks + rel_timeout;

            timeouts[ch] = abs_timeout; // in nanoseconds
            if (!timeout_active || abs_timeout < current_timeout) {
                if (timeout_active) {
                    /* there current timeout is now treated as pending */
                    pending_timeouts++;
                }
                set_timeout((uint32_t)(rel_timeout / NS_IN_MS));
                timeout_active = true;

                /* We need to keep track of how far into the future this is so
                    we can order client requests appropriately. */
                current_timeout = abs_timeout;
                active_channel = ch;
            } else {
                pending_timeouts++;
            }
            break;
        default:
            sddf_dprintf("TIMER DRIVER|LOG: Unknown request %llu to timer from channel %llu\n", microkit_msginfo_get_label(msginfo), ch);
            break;
    }

    return microkit_msginfo_new(0, 0);
}

void
init(void)
{
    regs = (void *)(gpt_regs + TIMER_REG_START);

    /* Start timer E acts as a clock, while timer A can be used for timeouts from clients */
    regs->mux = TIMER_A_EN | (TIMESTAMP_TIMEBASE_1_US << TIMER_E_INPUT_CLK) |
                       (TIMEOUT_TIMEBASE_1_MS << TIMER_A_INPUT_CLK);

    regs->timer_e = 0;
}
