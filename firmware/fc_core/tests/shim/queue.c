/* Host FreeRTOS queue fake for target-glue tests: copies, never blocks. SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>

#include "queue.h"

QueueHandle_t xQueueCreateStatic(UBaseType_t len, UBaseType_t size, uint8_t *buf, StaticQueue_t *q)
{
    *q = (StaticQueue_t){.buf = buf, .len = len, .size = size};
    return q;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait)
{
    (void)wait;
    if (q->count == q->len) return pdFALSE;
    memcpy(q->buf + ((q->head + q->count) % q->len) * q->size, item, q->size);
    q->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait)
{
    (void)wait;
    if (!q->count) return pdFALSE;
    memcpy(item, q->buf + q->head * q->size, q->size);
    q->head = (q->head + 1) % q->len;
    q->count--;
    return pdTRUE;
}
