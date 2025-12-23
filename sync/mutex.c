#include "mutex.h"

void rtos_mutex_init(rtos_mutex_t* m)
{
    if (!m) return;
    atomic_store(&m->locked, 0);
}

rtos_status_t rtos_mutex_lock(rtos_mutex_t* m, uint32_t timeout_ms)
{
    if (!m) return RTOS_ERR;

    uint64_t start = rtos_tick_get();
    while (1) {
        int expected = 0;
        if (atomic_compare_exchange_strong(&m->locked, &expected, 1)) {
            return RTOS_OK;
        }

        if (timeout_ms == 0) return RTOS_TIMEOUT;

        rtos_poll_delay_1tick();

        uint64_t elapsed_ms = rtos_tick_to_ms(rtos_tick_get() - start);
        if (elapsed_ms >= timeout_ms) return RTOS_TIMEOUT;
    }
}

void rtos_mutex_unlock(rtos_mutex_t* m)
{
    if (!m) return;
    atomic_store(&m->locked, 0);
    rtos_yield();
}
