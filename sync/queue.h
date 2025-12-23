#pragma once
#include <stdint.h>
#include <stddef.h>
#include "rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t* buf;
    size_t item_size;
    size_t capacity;   /* number of items */
    size_t head;
    size_t tail;
    size_t count;
} rtos_queue_t;

rtos_status_t rtos_queue_init(rtos_queue_t* q, void* backing_buf, size_t item_size, size_t capacity);
rtos_status_t rtos_queue_send(rtos_queue_t* q, const void* item, uint32_t timeout_ms);
rtos_status_t rtos_queue_recv(rtos_queue_t* q, void* out_item, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
