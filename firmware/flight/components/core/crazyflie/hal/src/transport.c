#include "transport.h"

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(CONFIG_TRANSPORT_NATIVE_WIFI) || defined(CONFIG_TRANSPORT_REMOTE_WIFI)
#include "wifi_esp32.h"
#include "wifilink.h"
#endif
#ifdef CONFIG_TRANSPORT_C5_SPI
#include "c5_link.h" /* ESP-FC Next: CRTP over the P4 <-> C5 SPI link */
#endif

static bool isInit;

#ifdef CONFIG_TRANSPORT_NONE
static int noneSetEnable(bool enable)
{
    (void)enable;
    return 0;
}

static int noneSendPacket(CRTPPacket *packet)
{
    (void)packet;
    return -1;
}

static int noneReceivePacket(CRTPPacket *packet)
{
    (void)packet;
    vTaskDelay(pdMS_TO_TICKS(100));
    return -1;
}

static bool noneIsConnected(void)
{
    return false;
}

static struct crtpLinkOperations noneLink = {
    .setEnable = noneSetEnable,
    .sendPacket = noneSendPacket,
    .receivePacket = noneReceivePacket,
    .isConnected = noneIsConnected,
};
#endif

void transportInit(void)
{
    if (isInit) {
        return;
    }

#if defined(CONFIG_TRANSPORT_NATIVE_WIFI) || defined(CONFIG_TRANSPORT_REMOTE_WIFI)
    wifiInit();
    wifilinkInit();
#elif defined(CONFIG_TRANSPORT_C5_SPI)
    c5LinkInit();
#endif
    isInit = true;
}

bool transportTest(void)
{
    if (!isInit) {
        return false;
    }

#if defined(CONFIG_TRANSPORT_NATIVE_WIFI) || defined(CONFIG_TRANSPORT_REMOTE_WIFI)
    return wifiTest() && wifilinkTest();
#elif defined(CONFIG_TRANSPORT_C5_SPI)
    return c5LinkTest();
#else
    return true;
#endif
}

struct crtpLinkOperations *transportGetLink(void)
{
#if defined(CONFIG_TRANSPORT_NATIVE_WIFI) || defined(CONFIG_TRANSPORT_REMOTE_WIFI)
    return wifilinkGetLink();
#elif defined(CONFIG_TRANSPORT_C5_SPI)
    return c5LinkGetLink();
#else
    return &noneLink;
#endif
}

const char *transportGetName(void)
{
#if defined(CONFIG_TRANSPORT_NATIVE_WIFI)
    return "native-wifi";
#elif defined(CONFIG_TRANSPORT_REMOTE_WIFI)
    return "remote-wifi";
#elif defined(CONFIG_TRANSPORT_C5_SPI)
    return "c5-spi";
#else
    return "none";
#endif
}
