/* Host stand-in for FreeRTOS semphr.h (fc_hl.c): the SITL is single-threaded, so the mutex is a no-op. */
#pragma once

#include "FreeRTOS.h"

typedef struct { int unused; } StaticSemaphore_t;
typedef StaticSemaphore_t *xSemaphoreHandle;
#define portMAX_DELAY 0xffffffffu

#define xSemaphoreCreateMutexStatic(buffer) (buffer)
#define xSemaphoreTake(sem, ticks) ((void)(sem), (void)(ticks), 1)
#define xSemaphoreGive(sem) ((void)(sem), 1)
