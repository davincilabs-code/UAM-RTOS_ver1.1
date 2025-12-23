#pragma once
#include <stdatomic.h>
#include <stdint.h>
#include "rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    atomic_int locked;
} rtos_mutex_t;

void rtos_mutex_init(rtos_mutex_t* m);

/* timeout_ms=0 => non-blocking trylock */
rtos_status_t rtos_mutex_lock(rtos_mutex_t* m, uint32_t timeout_ms);
void rtos_mutex_unlock(rtos_mutex_t* m);

#ifdef __cplusplus
}
#endif
