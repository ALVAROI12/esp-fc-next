/* Host stand-in for FreeRTOS task.h, only for the SITL high-level commander (fc_hl.c): its task never runs. */
#pragma once

#include "FreeRTOS.h"

typedef uint32_t StackType_t;
typedef struct { int unused; } StaticTask_t;
typedef void *TaskHandle_t;
#define xTaskCreateStatic(fn, name, depth, prm, prio, stack, tcb) ((void)(fn), (void)(stack), (void)(tcb), (TaskHandle_t)0)
