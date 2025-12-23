#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_MAX_TASKS        16
#define RTOS_MAX_PRIORITY     8   /* 0..7 */
#define RTOS_NAME_MAX         16

typedef void (*rtos_task_fn)(void*);

typedef enum {
    RTOS_OK = 0,
    RTOS_ERR = -1,
    RTOS_TIMEOUT = -2,
    RTOS_FULL = -3,
    RTOS_EMPTY = -4
} rtos_status_t;

typedef struct rtos_task* rtos_task_handle_t;

/* Init kernel (call once, before creating tasks). */
rtos_status_t rtos_init(uint32_t tick_hz);

/* Create a task. stack_size is bytes. priority: 0 (low) .. 7 (high). */
rtos_status_t rtos_task_create(const char* name,
                               rtos_task_fn fn,
                               void* arg,
                               size_t stack_size,
                               uint8_t priority,
                               rtos_task_handle_t* out_handle);

/* Start scheduler (never returns in normal flow). */
void rtos_start(void);

/* Yield execution to another ready task. */
void rtos_yield(void);

/* Delay current task for ms milliseconds. */
void rtos_delay_ms(uint32_t ms);

/* Current tick count. */
uint64_t rtos_tick_get(void);

/* Tick -> ms helper */
uint64_t rtos_tick_to_ms(uint64_t tick);

/* Logging helper: [tick] tag message */
void rtos_log(const char* tag, const char* fmt, ...);

/* For sync primitives internal usage */
void rtos_poll_delay_1tick(void);

#ifdef __cplusplus
}
#endif
