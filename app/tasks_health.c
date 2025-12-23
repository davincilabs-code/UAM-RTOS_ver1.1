#include "tasks_health.h"
#include "app_shared.h"
#include "rtos.h"

void Task_Health(void* arg)
{
    (void)arg;

    uint64_t last_hb = rtos_tick_get();
    while (1) {
        uint32_t hb = rtos_event_flags_wait_any(&g_events, EV_HEARTBEAT, 1000);
        if (hb) {
            last_hb = rtos_tick_get();
            rtos_event_flags_clear(&g_events, EV_HEARTBEAT);
        } else {
            /* watchdog timeout */
            uint64_t now = rtos_tick_get();
            uint64_t dt_ms = rtos_tick_to_ms(now - last_hb);
            rtos_log("HEALTH", "watchdog: heartbeat missing for %llums", (unsigned long long)dt_ms);
            last_hb = now;
        }

        rtos_delay_ms(1000);
    }
}
