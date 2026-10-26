/*
 * Host stand-in for FreeRTOS.h. fc_core algorithm code must not call the RTOS;
 * this header only provides the types and tick constants some headers reference.
 */
#pragma once

#include <stdint.h>

typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;

#define configTICK_RATE_HZ 1000
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
