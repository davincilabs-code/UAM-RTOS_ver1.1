#pragma once
#include <stdint.h>
#include "rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t flags;
} rtos_event_flags_t;

void rtos_event_flags_init(rtos_event_flags_t* e);
void rtos_event_flags_set(rtos_event_flags_t* e, uint32_t bits);
void rtos_event_flags_clear(rtos_event_flags_t* e, uint32_t bits);

/* wait_any: returns bits observed (subset) or 0 on timeout
   wait_all: returns full bits on success or 0 on timeout */
uint32_t rtos_event_flags_wait_any(rtos_event_flags_t* e, uint32_t bits, uint32_t timeout_ms);
uint32_t rtos_event_flags_wait_all(rtos_event_flags_t* e, uint32_t bits, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
