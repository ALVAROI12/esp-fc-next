/* Host stand-in for FreeRTOS queue.h, only for testing target glue (test_runtime.c implements it). */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"

#ifndef pdTRUE
#define pdTRUE 1
#define pdFALSE 0
#endif

typedef struct {
    uint8_t *buf;
    size_t len, size, head, count;
} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;

QueueHandle_t xQueueCreateStatic(UBaseType_t len, UBaseType_t size, uint8_t *buf, StaticQueue_t *q);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait);
