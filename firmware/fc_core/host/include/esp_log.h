/* Host stand-in for ESP-IDF logging: routes ESP_LOG* macros to fc_host_log(). */
#pragma once

#include "fc_host.h"

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE,
} esp_log_level_t;

#define ESP_LOG_LEVEL_LOCAL(level, tag, fmt, ...) fc_host_log((int)(level), (tag), (fmt), ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)
