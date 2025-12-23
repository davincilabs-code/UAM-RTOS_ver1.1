#include "rtos.h"
#include "app_shared.h"

#include <stdio.h>
#include <string.h>

/* Shared objects */
rtos_queue_t g_q_sensor;
rtos_queue_t g_q_state;
rtos_queue_t g_q_cmd;

rtos_event_flags_t g_events;
rtos_mutex_t g_log_mutex;
rtos_mutex_t g_state_mutex;
nav_state_t g_latest_state;
int g_latest_state_valid = 0;

/* Backing buffers (queues) */
unsigned char g_buf_sensor[sizeof(sensor_sample_t) * 32];
unsigned char g_buf_state [sizeof(nav_state_t)    * 32];
unsigned char g_buf_cmd   [sizeof(control_cmd_t)  * 32];

#include "tasks_sensor.h"
#include "tasks_estimator.h"
#include "tasks_control.h"
#include "tasks_comms.h"
#include "tasks_health.h"
#include "tasks_logger.h"

static void init_ipc(void)
{
    rtos_queue_init(&g_q_sensor, g_buf_sensor, sizeof(sensor_sample_t), 32);
    rtos_queue_init(&g_q_state,  g_buf_state,  sizeof(nav_state_t),    32);
    rtos_queue_init(&g_q_cmd,    g_buf_cmd,    sizeof(control_cmd_t),  32);

    rtos_event_flags_init(&g_events);
    rtos_mutex_init(&g_log_mutex);
    rtos_mutex_init(&g_state_mutex);
    g_latest_state_valid = 0;
}

int main(void)
{
    if (rtos_init(1000) != RTOS_OK) {
        fprintf(stderr, "rtos_init failed\n");
        return 1;
    }

    init_ipc();

    /* Create tasks: stack >= 16KB required by this demo */
    rtos_task_create("sensor",    Task_Sensor,    NULL, 64*1024, 2, NULL);
    rtos_task_create("estimator", Task_Estimator, NULL, 64*1024, 3, NULL);
    rtos_task_create("control",   Task_Control,   NULL, 64*1024, 5, NULL);
    rtos_task_create("comms",     Task_Comms,     NULL, 64*1024, 1, NULL);
    rtos_task_create("logger",    Task_Logger,    NULL, 64*1024, 0, NULL);
    rtos_task_create("health",    Task_Health,    NULL, 64*1024, 4, NULL);

    rtos_log("BOOT", "UAM-RTOS ver1.1 demo starting (tick=%u Hz)", 1000);

    rtos_start();
    return 0;
}
