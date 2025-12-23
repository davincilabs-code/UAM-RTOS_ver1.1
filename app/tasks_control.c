#include "tasks_control.h"
#include "app_shared.h"
#include "rtos.h"
#include <math.h>

static float clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Task_Control(void* arg)
{
    (void)arg;

    /* simple altitude hold P controller placeholder */
    const float alt_ref = 300.0f;
    const float Kp_alt = 0.02f;

    while (1) {
        (void)rtos_event_flags_wait_any(&g_events, EV_STATE_READY, 50);

        nav_state_t st;
        if (rtos_queue_recv(&g_q_state, &st, 0) == RTOS_OK) {
            float e_alt = alt_ref - st.alt_m;
            float thrust = 0.5f + Kp_alt * e_alt;
            thrust = clamp(thrust, 0.0f, 1.0f);

            /* very simplified: pitch command ramps for transition */
            float pitch_cmd = 0.0f;
            if (st.vx > 5.0f) {
                pitch_cmd = clamp((st.vx - 5.0f) * 0.02f, 0.0f, 0.3f);
            }

            control_cmd_t cmd = {
                .t_ms = st.t_ms,
                .thrust = thrust,
                .roll_cmd = 0.0f,
                .pitch_cmd = pitch_cmd,
                .yaw_cmd = 0.0f
            };

            (void)rtos_queue_send(&g_q_cmd, &cmd, 10);
            rtos_event_flags_set(&g_events, EV_CMD_READY);
            rtos_event_flags_set(&g_events, EV_HEARTBEAT);
        }

        rtos_delay_ms(20);
    }
}
