/*
 * ESP-FC Next: CRTP over the P4 <-> C5 SPI link. Spec: docs/protocols.md sections 1 and 5.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "c5_link.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define C5_LINK_QUEUE 16
#define C5_LINK_ACTIVITY_MS 1000 /* isConnected: a CRTP packet arrived this recently */

static StaticQueue_t inStorage, outStorage;
static uint8_t inBuf[C5_LINK_QUEUE * sizeof(CRTPPacket)], outBuf[C5_LINK_QUEUE * sizeof(CRTPPacket)];
static QueueHandle_t inQueue, outQueue;
static uint16_t txSeq, rxSeq;
static bool haveRxSeq, isInit;
static TickType_t lastCrtpTick;
static c5LinkStats_t stats;

void c5LinkExchange(const uint8_t rx[FC_LINK_FRAME_SIZE], uint8_t tx[FC_LINK_FRAME_SIZE])
{
    fc_link_frame_t f;
    if (fc_link_decode(rx, FC_LINK_FRAME_SIZE, &f) == FC_LINK_OK) {
        stats.rxFrames++;
        if (haveRxSeq && f.seq != (uint16_t)(rxSeq + 1u)) stats.rxGaps++;
        rxSeq = f.seq;
        haveRxSeq = true;
        if (f.type == FC_LINK_CRTP && f.len >= 1) {
            CRTPPacket p = {.size = (uint8_t)(f.len - 1)};
            memcpy(p.raw, f.payload, f.len);
            lastCrtpTick = xTaskGetTickCount();
            if (xQueueSend(inQueue, &p, 0) != pdTRUE) stats.rxDropped++;
        }
    } else {
        stats.rxErrors++; /* no C5, bus noise, or CRC error: never reaches CRTP */
    }

    fc_link_frame_t o = {.type = FC_LINK_IDLE, .seq = txSeq++};
    CRTPPacket p;
    if (xQueueReceive(outQueue, &p, 0) == pdTRUE && p.size <= CRTP_MAX_DATA_SIZE) {
        o.type = FC_LINK_CRTP;
        o.len = (uint8_t)(p.size + 1);
        memcpy(o.payload, p.raw, o.len);
        stats.txCrtp++;
    }
    fc_link_encode(&o, tx);
}

static int c5LinkSetEnable(bool enable)
{
    (void)enable;
    return 0;
}

/* crtpTxTask retries while this returns false (0). */
static int c5LinkSendPacket(CRTPPacket *p) { return xQueueSend(outQueue, p, pdMS_TO_TICKS(10)) == pdTRUE; }

/* crtpRxTask: 0 = packet delivered. */
static int c5LinkReceivePacket(CRTPPacket *p) { return xQueueReceive(inQueue, p, pdMS_TO_TICKS(100)) == pdTRUE ? 0 : -1; }

static bool c5LinkIsConnected(void)
{
    return lastCrtpTick && (TickType_t)(xTaskGetTickCount() - lastCrtpTick) < pdMS_TO_TICKS(C5_LINK_ACTIVITY_MS);
}

static struct crtpLinkOperations c5LinkOp = {
    .setEnable = c5LinkSetEnable,
    .sendPacket = c5LinkSendPacket,
    .receivePacket = c5LinkReceivePacket,
    .isConnected = c5LinkIsConnected,
};

struct crtpLinkOperations *c5LinkGetLink(void) { return &c5LinkOp; }
bool c5LinkTest(void) { return isInit; }
c5LinkStats_t c5LinkGetStats(void) { return stats; }

#ifndef FC_HOST_BUILD
#include "driver/spi_master.h"
#include "sdkconfig.h"

#define C5_LINK_PERIOD_MS 2 /* 500 Hz: well above 100 Hz setpoints and logs */

static void c5LinkTask(void *arg)
{
    spi_device_handle_t dev = arg;
    WORD_ALIGNED_ATTR static uint8_t tx[FC_LINK_FRAME_SIZE], rx[FC_LINK_FRAME_SIZE];
    memset(rx, 0, sizeof rx);
    c5LinkExchange(rx, tx); /* first tx frame; the zero rx counts as one error */
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        spi_transaction_t t = {.length = FC_LINK_FRAME_SIZE * 8, .tx_buffer = tx, .rx_buffer = rx};
        if (spi_device_polling_transmit(dev, &t) != ESP_OK) memset(rx, 0, sizeof rx);
        c5LinkExchange(rx, tx);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(C5_LINK_PERIOD_MS));
    }
}
#endif

void c5LinkInit(void)
{
    if (isInit) return;
    inQueue = xQueueCreateStatic(C5_LINK_QUEUE, sizeof(CRTPPacket), inBuf, &inStorage);
    outQueue = xQueueCreateStatic(C5_LINK_QUEUE, sizeof(CRTPPacket), outBuf, &outStorage);
#ifndef FC_HOST_BUILD
    spi_bus_config_t bus = {.mosi_io_num = CONFIG_ESPFC_C5_SPI_MOSI, .miso_io_num = CONFIG_ESPFC_C5_SPI_MISO,
                            .sclk_io_num = CONFIG_ESPFC_C5_SPI_SCLK, .quadwp_io_num = -1, .quadhd_io_num = -1,
                            .max_transfer_sz = FC_LINK_FRAME_SIZE};
    spi_device_interface_config_t devcfg = {.mode = 0, .clock_speed_hz = 10 * 1000 * 1000,
                                            .spics_io_num = CONFIG_ESPFC_C5_SPI_CS, .queue_size = 1};
    spi_device_handle_t dev;
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK ||
        spi_bus_add_device(SPI2_HOST, &devcfg, &dev) != ESP_OK ||
        xTaskCreate(c5LinkTask, "c5_link", 3072, dev, 3, NULL) != pdPASS) {
        return; /* isInit stays false: commTest fails and the link stays silent */
    }
#endif
    isInit = true;
}
