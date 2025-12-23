#pragma once
#include "queue.h"
#include "event_flags.h"
#include "mutex.h"
#include "uam_messages.h"

/* Event flag bits */
#define EV_SENSOR_READY   (1u << 0)
#define EV_STATE_READY    (1u << 1)
#define EV_CMD_READY      (1u << 2)
#define EV_HEARTBEAT      (1u << 3)

/* Shared objects */
extern rtos_queue_t g_q_sensor;
extern rtos_queue_t g_q_state;
extern rtos_queue_t g_q_cmd;

extern rtos_event_flags_t g_events;

extern rtos_mutex_t g_log_mutex;

/* Backing buffers */
extern unsigned char g_buf_sensor[];
extern unsigned char g_buf_state[];
extern unsigned char g_buf_cmd[];
/* Latest state snapshot (for logging/monitoring) */
extern nav_state_t g_latest_state;
extern int g_latest_state_valid;
extern rtos_mutex_t g_state_mutex;

