#include "queue.h"
#include <string.h>

rtos_status_t rtos_queue_init(rtos_queue_t* q, void* backing_buf, size_t item_size, size_t capacity)
{
    if (!q || !backing_buf || item_size == 0 || capacity == 0) return RTOS_ERR;
    q->buf = (uint8_t*)backing_buf;
    q->item_size = item_size;
    q->capacity = capacity;
    q->head = q->tail = q->count = 0;
    return RTOS_OK;
}

static uint64_t now_tick(void) { return rtos_tick_get(); }

static uint64_t ms_to_tick(uint32_t ms)
{
    /* rtos_delay_ms uses ceil internally; keep aligned with that */
    return (ms == 0) ? 0 : ((ms + 0) / 1); /* placeholder, actual checked by time via tick_to_ms */
}

rtos_status_t rtos_queue_send(rtos_queue_t* q, const void* item, uint32_t timeout_ms)
{
    if (!q || !item) return RTOS_ERR;

    uint64_t start = now_tick();
    while (1) {
        if (q->count < q->capacity) {
            uint8_t* dst = q->buf + (q->tail * q->item_size);
            memcpy(dst, item, q->item_size);
            q->tail = (q->tail + 1) % q->capacity;
            q->count++;
            return RTOS_OK;
        }

        if (timeout_ms == 0) return RTOS_TIMEOUT;

        /* polling wait */
        rtos_poll_delay_1tick();

        /* timeout check */
        uint64_t elapsed_ms = rtos_tick_to_ms(now_tick() - start);
        if (elapsed_ms >= timeout_ms) return RTOS_TIMEOUT;
    }
}

rtos_status_t rtos_queue_recv(rtos_queue_t* q, void* out_item, uint32_t timeout_ms)
{
    if (!q || !out_item) return RTOS_ERR;

    uint64_t start = now_tick();
    while (1) {
        if (q->count > 0) {
            uint8_t* src = q->buf + (q->head * q->item_size);
            memcpy(out_item, src, q->item_size);
            q->head = (q->head + 1) % q->capacity;
            q->count--;
            return RTOS_OK;
        }

        if (timeout_ms == 0) return RTOS_TIMEOUT;

        rtos_poll_delay_1tick();

        uint64_t elapsed_ms = rtos_tick_to_ms(now_tick() - start);
        if (elapsed_ms >= timeout_ms) return RTOS_TIMEOUT;
    }
}
