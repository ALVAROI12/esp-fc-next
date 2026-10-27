/*
 * ESP-FC Next radio co-processor (ESP32-C5).
 *
 * Placeholder entry point. The P4 <-> C5 SPI link, ESP-NOW swarm transport and
 * Wi-Fi/UDP CRTP bridge are added in Phase 1 task 1.7 (see docs/).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "fc_link.h"
#include "fc_link_aead.h"

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

    /* Boot self-check of the AES-CCM envelope with a throwaway key (real keys: provisioning, not done yet). */
    static fc_link_aead_t aead; /* ~3 KB of replay windows: keep off the task stack */
    static const uint8_t testKey[16] = {0};
    uint8_t env[FC_LINK_ENV_HEADER_SIZE + FC_LINK_FRAME_SIZE + FC_LINK_ENV_TAG_SIZE], plain[FC_LINK_ENV_MAX_PLAIN];
    uint8_t sender = 0;
    size_t n = fc_link_aead_init(&aead, testKey, 1) ? fc_link_seal(&aead, 1, 1, frame, 32, env, sizeof env) : 0;
    ok = n && fc_link_open(&aead, env, n, &sender, plain) == 32 && memcmp(plain, frame, 32) == 0 &&
         fc_link_open(&aead, env, n, &sender, plain) < 0; /* replay rejected */
    fc_link_aead_free(&aead);
    ESP_LOGI(TAG, "link envelope self-check %s", ok ? "ok" : "FAILED");
}
