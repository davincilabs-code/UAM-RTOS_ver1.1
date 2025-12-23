#include "event_flags.h"

void rtos_event_flags_init(rtos_event_flags_t* e)
{
    if (!e) return;
    e->flags = 0;
}

void rtos_event_flags_set(rtos_event_flags_t* e, uint32_t bits)
{
    if (!e) return;
    e->flags |= bits;
    /* resched point */
    rtos_yield();
}

void rtos_event_flags_clear(rtos_event_flags_t* e, uint32_t bits)
{
    if (!e) return;
    e->flags &= ~bits;
}

static uint64_t now_tick(void) { return rtos_tick_get(); }

uint32_t rtos_event_flags_wait_any(rtos_event_flags_t* e, uint32_t bits, uint32_t timeout_ms)
{
    if (!e) return 0;

    uint64_t start = now_tick();
    while (1) {
        uint32_t v = e->flags & bits;
        if (v) return v;

        if (timeout_ms == 0) return 0;

        rtos_poll_delay_1tick();

        uint64_t elapsed_ms = rtos_tick_to_ms(now_tick() - start);
        if (elapsed_ms >= timeout_ms) return 0;
    }
}

uint32_t rtos_event_flags_wait_all(rtos_event_flags_t* e, uint32_t bits, uint32_t timeout_ms)
{
    if (!e) return 0;

    uint64_t start = now_tick();
    while (1) {
        uint32_t v = e->flags & bits;
        if (v == bits) return v;

        if (timeout_ms == 0) return 0;

        rtos_poll_delay_1tick();

        uint64_t elapsed_ms = rtos_tick_to_ms(now_tick() - start);
        if (elapsed_ms >= timeout_ms) return 0;
    }
}
