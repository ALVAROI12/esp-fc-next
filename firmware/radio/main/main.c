/*
 * ESP-FC Next radio co-processor (ESP32-C5).
 *
 * Placeholder entry point. The P4 <-> C5 SPI link, ESP-NOW swarm transport and
 * Wi-Fi/UDP CRTP bridge are added in Phase 1 task 1.7 (see docs/).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "esp_app_desc.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "fc_link.h"

static const char *TAG = "radio";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "ESP-FC Next radio %s started", esp_app_get_description()->version);

    /* Boot self-check of the shared link codec on the target toolchain. */
    uint8_t frame[FC_LINK_FRAME_SIZE];
    fc_link_frame_t idle = {.type = FC_LINK_IDLE}, back;
    bool ok = fc_link_encode(&idle, frame) && fc_link_decode(frame, sizeof frame, &back) == FC_LINK_OK;
    ESP_LOGI(TAG, "link codec self-check %s", ok ? "ok" : "FAILED");
}
