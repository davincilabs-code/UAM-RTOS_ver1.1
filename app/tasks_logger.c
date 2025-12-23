#include "tasks_logger.h"
#include "app_shared.h"
#include "rtos.h"

void Task_Logger(void* arg)
{
    (void)arg;

    while (1) {
        nav_state_t st;
        int valid = 0;

        if (rtos_mutex_lock(&g_state_mutex, 10) == RTOS_OK) {
            if (g_latest_state_valid) {
                st = g_latest_state;
                valid = 1;
            }
            rtos_mutex_unlock(&g_state_mutex);
        }

        if (valid) {
            rtos_mutex_lock(&g_log_mutex, 10);
            rtos_log("LOGGER", "state: vx=%.1f m/s alt=%.2f m yaw=%.4f (t=%llums)",
                     st.vx, st.alt_m, st.yaw, (unsigned long long)st.t_ms);
            rtos_mutex_unlock(&g_log_mutex);
        } else {
            rtos_log("LOGGER", "state: (no snapshot yet)");
        }

        rtos_delay_ms(500);
    }
}
