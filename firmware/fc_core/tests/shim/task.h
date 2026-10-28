/* Host stand-in for FreeRTOS task.h, only for testing target glue (test_runtime.c implements it). */
#pragma once

#include "FreeRTOS.h"

TickType_t xTaskGetTickCount(void);
