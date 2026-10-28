/*
 * ESP-FC Next radio co-processor (ESP32-C5).
 *
 * Bridges sealed ESP-NOW broadcasts (docs/protocols.md section 3) to the 64-byte frame (section 1) through
 * fc_radio (section 5). On a drone the P4 is SPI master and polls at a fixed rate; every transaction carries one
 * frame each way (IDLE when there is nothing to deliver). The ground station (radio id 0) streams the same frames
 * to and from the PC over USB Serial/JTAG instead.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>

#include "driver/spi_slave.h"
#include "driver/usb_serial_jtag.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "fc_link.h"
#include "fc_link_aead.h"
#include "fc_radio.h"

static const char *TAG = "radio";
static const uint8_t BROADCAST[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

typedef struct {
    uint8_t len;
    uint8_t data[FC_RADIO_ENV_MAX];
} air_msg_t;

static QueueHandle_t airQueue;
static fc_radio_t radio; /* ~3 KB of replay windows; used only by spi_task after init */
static nvs_handle_t store;

static void self_check(void)
{
    uint8_t frame[FC_LINK_FRAME_SIZE];
    fc_link_frame_t idle = {.type = FC_LINK_IDLE}, back;
    bool ok = fc_link_encode(&idle, frame) && fc_link_decode(frame, sizeof frame, &back) == FC_LINK_OK;
    ESP_LOGI(TAG, "link codec self-check %s", ok ? "ok" : "FAILED");

    static fc_link_aead_t aead; /* throwaway key: checks the toolchain's mbedtls, not provisioning */
    static const uint8_t testKey[16] = {0};
    uint8_t env[FC_LINK_ENV_HEADER_SIZE + FC_LINK_FRAME_SIZE + FC_LINK_ENV_TAG_SIZE], plain[FC_LINK_ENV_MAX_PLAIN];
    uint8_t sender = 0;
    size_t n = fc_link_aead_init(&aead, testKey, 1) ? fc_link_seal(&aead, 1, 1, frame, 32, env, sizeof env) : 0;
    ok = n && fc_link_open(&aead, env, n, &sender, plain) == 32 && memcmp(plain, frame, 32) == 0 &&
         fc_link_open(&aead, env, n, &sender, plain) < 0; /* replay rejected */
    fc_link_aead_free(&aead);
    ESP_LOGI(TAG, "link envelope self-check %s", ok ? "ok" : "FAILED");
}

/* Provisioned with nvs_partition_gen (namespace "fclink": key blob[16], net u32, id u8). There is no default
 * key: without one the radio stays off, the P4 sees no valid frames and its command watchdog lands. */
static bool load_radio(void)
{
    uint8_t key[16], id;
    uint32_t net, limit = 0;
    size_t n = sizeof key;
    if (nvs_open("fclink", NVS_READWRITE, &store) != ESP_OK) return false;
    nvs_get_u32(store, "ctr", &limit); /* missing on first boot: start at 0 */
    bool ok = nvs_get_blob(store, "key", key, &n) == ESP_OK && n == sizeof key &&
              nvs_get_u32(store, "net", &net) == ESP_OK && nvs_get_u8(store, "id", &id) == ESP_OK &&
              fc_radio_init(&radio, key, net, id, limit);
    memset(key, 0, sizeof key);
    return ok;
}

/* ponytail: the NVS commit blocks this task (and so the P4 link) for a flash write once per
 * FC_RADIO_COUNTER_BLOCK sent envelopes; move it to a low-priority task if the P4 sees the gap. */
static void reserve_counters(void)
{
    uint32_t limit = fc_radio_reserve_due(&radio);
    if (limit && nvs_set_u32(store, "ctr", limit) == ESP_OK && nvs_commit(store) == ESP_OK) {
        fc_radio_reserved(&radio, limit);
    }
}

/* Wi-Fi task context: copy and hand off; all envelope work happens in spi_task. */
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    air_msg_t m;
    if (len <= 0 || len > (int)sizeof m.data) return;
    m.len = (uint8_t)len;
    memcpy(m.data, data, (size_t)len);
    xQueueSend(airQueue, &m, 0); /* full: drop; commands and status are periodic */
}

static void spi_task(void *arg)
{
    (void)arg;
    WORD_ALIGNED_ATTR static uint8_t tx[FC_LINK_FRAME_SIZE], rx[FC_LINK_FRAME_SIZE];
    for (;;) {
        reserve_counters();
        air_msg_t m;
        bool have = false;
        while (!have && xQueueReceive(airQueue, &m, 0) == pdTRUE) have = fc_radio_from_air(&radio, m.data, m.len, tx);
        if (!have) {
            fc_link_frame_t idle = {.type = FC_LINK_IDLE, .seq = radio.spiSeq++};
            fc_link_encode(&idle, tx);
        }
        spi_slave_transaction_t t = {.length = FC_LINK_FRAME_SIZE * 8, .tx_buffer = tx, .rx_buffer = rx};
        if (spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY) != ESP_OK) continue;
        uint8_t env[FC_RADIO_ENV_MAX];
        size_t n = fc_radio_to_air(&radio, rx, t.trans_len / 8, env);
        if (n) esp_now_send(BROADCAST, env, n);
    }
}

/* Ground station: frames from the PC go to air, frames from air go to the PC. Console text on the same port is
 * skipped by the PC's resync (fc_link_stream). */
static void serial_task(void *arg)
{
    (void)arg;
    static fc_link_stream_t stream;
    uint8_t in[64], frame[FC_LINK_FRAME_SIZE], env[FC_RADIO_ENV_MAX];
    for (;;) {
        reserve_counters();
        int n = usb_serial_jtag_read_bytes(in, sizeof in, pdMS_TO_TICKS(2));
        for (int i = 0; i < n; i++) {
            if (!fc_link_stream_push(&stream, in[i], frame)) continue;
            size_t m = fc_radio_to_air(&radio, frame, sizeof frame, env);
            if (m) esp_now_send(BROADCAST, env, m);
        }
        air_msg_t msg;
        while (xQueueReceive(airQueue, &msg, 0) == pdTRUE) {
            if (fc_radio_from_air(&radio, msg.data, msg.len, frame)) {
                usb_serial_jtag_write_bytes(frame, sizeof frame, pdMS_TO_TICKS(10));
            }
        }
    }
}

static void start_link(void)
{
    if (radio.id == FC_RADIO_GROUND_ID) {
        usb_serial_jtag_driver_config_t usb = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
        esp_log_level_set("*", ESP_LOG_WARN); /* keep the frame stream mostly clean */
        xTaskCreate(serial_task, "pc_link", 4096, NULL, 10, NULL);
        return;
    }
    spi_bus_config_t bus = {.mosi_io_num = CONFIG_ESPFC_RADIO_SPI_MOSI, .miso_io_num = CONFIG_ESPFC_RADIO_SPI_MISO,
                            .sclk_io_num = CONFIG_ESPFC_RADIO_SPI_SCLK, .quadwp_io_num = -1, .quadhd_io_num = -1};
    spi_slave_interface_config_t slave = {.mode = 0, .spics_io_num = CONFIG_ESPFC_RADIO_SPI_CS, .queue_size = 1};
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &bus, &slave, SPI_DMA_CH_AUTO));
    xTaskCreate(spi_task, "spi_link", 4096, NULL, 10, NULL);
}

static void start_radio(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPFC_RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {.ifidx = WIFI_IF_STA, .encrypt = false}; /* sealed in software (section 3) */
    memcpy(peer.peer_addr, BROADCAST, sizeof BROADCAST);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));


    airQueue = xQueueCreate(16, sizeof(air_msg_t));
    configASSERT(airQueue);
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    start_link();
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "ESP-FC Next radio %s started", esp_app_get_description()->version);
    self_check();
    if (!load_radio()) {
        ESP_LOGE(TAG, "no link key provisioned (NVS namespace fclink): radio off");
        return;
    }
    ESP_LOGI(TAG, "radio id %u (%s) on channel %d", radio.id, radio.id == FC_RADIO_GROUND_ID ? "ground" : "drone",
             CONFIG_ESPFC_RADIO_CHANNEL);
    start_radio();
}
