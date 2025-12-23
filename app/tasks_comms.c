#include "tasks_comms.h"
#include "app_shared.h"
#include "rtos.h"
#include <stdio.h>

void Task_Comms(void* arg)
{
    (void)arg;

    while (1) {
        (void)rtos_event_flags_wait_any(&g_events, EV_CMD_READY, 100);

        control_cmd_t cmd;
        if (rtos_queue_recv(&g_q_cmd, &cmd, 0) == RTOS_OK) {
            /* telemetry style output */
            rtos_mutex_lock(&g_log_mutex, 10);
            rtos_log("COMMS", "tx: t=%llums thrust=%.3f pitch_cmd=%.3f",
                     (unsigned long long)cmd.t_ms, cmd.thrust, cmd.pitch_cmd);
            rtos_mutex_unlock(&g_log_mutex);
        }

        rtos_delay_ms(100);
    }
}
