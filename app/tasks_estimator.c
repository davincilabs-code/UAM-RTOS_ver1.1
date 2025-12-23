#include "tasks_estimator.h"
#include "app_shared.h"
#include "rtos.h"
#include <string.h>

void Task_Estimator(void* arg)
{
    (void)arg;
    nav_state_t state = {0};

    /* crude estimator: low-pass filter on altitude and yaw rate */
    float alt_f = 300.0f;
    float yaw_f = 0.0f;

    while (1) {
        /* Wait for new sensor sample (up to 100ms) */
        (void)rtos_event_flags_wait_any(&g_events, EV_SENSOR_READY, 100);

        sensor_sample_t s;
        if (rtos_queue_recv(&g_q_sensor, &s, 0) == RTOS_OK) {
            const float alpha = 0.15f;
            alt_f = (1.0f - alpha) * alt_f + alpha * s.alt_m;
            yaw_f = (1.0f - alpha) * yaw_f + alpha * s.gz;

            state.t_ms = s.t_ms;
            state.roll  = 0.0f;
            state.pitch = 0.0f;
            state.yaw   = yaw_f;
            state.p = s.gx;
            state.q = s.gy;
            state.r = s.gz;
            state.vx = s.airspeed_mps;
            state.vy = 0.0f;
            state.vz = 0.0f;
            state.alt_m = alt_f;

            (void)rtos_queue_send(&g_q_state, &state, 50);
            /* publish snapshot */
            if (rtos_mutex_lock(&g_state_mutex, 10) == RTOS_OK) {
                g_latest_state = state;
                g_latest_state_valid = 1;
                rtos_mutex_unlock(&g_state_mutex);
            }
            rtos_event_flags_set(&g_events, EV_STATE_READY);
        }

        rtos_delay_ms(50);
    }
}
