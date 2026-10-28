/* P4 side of the C5 SPI link (target/c5_link.c) on the host: frame <-> CRTP and link health.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>

#include "c5_link.h"
#include "task.h"
#include "unity.h"

static TickType_t rtosTick;
TickType_t xTaskGetTickCount(void) { return rtosTick; }

static struct crtpLinkOperations *link;
static uint16_t c5Seq;
static uint8_t tx[FC_LINK_FRAME_SIZE];

/* What the C5 clocks back in one transaction. */
static void from_c5(uint8_t type, const uint8_t *payload, uint8_t len)
{
    fc_link_frame_t f = {.type = type, .seq = c5Seq++, .len = len};
    memcpy(f.payload, payload, len);
    uint8_t rx[FC_LINK_FRAME_SIZE];
    TEST_ASSERT_TRUE(fc_link_encode(&f, rx));
    c5LinkExchange(rx, tx);
}

static fc_link_frame_t sent(void)
{
    fc_link_frame_t f;
    TEST_ASSERT_EQUAL(FC_LINK_OK, fc_link_decode(tx, sizeof tx, &f));
    return f;
}

void setUp(void)
{
    c5LinkInit();
    link = c5LinkGetLink();
    CRTPPacket p;
    while (link->receivePacket(&p) == 0) {}
    do from_c5(FC_LINK_IDLE, NULL, 0); while (sent().type != FC_LINK_IDLE);
}
void tearDown(void) {}

static void test_crtp_from_the_c5_reaches_the_crtp_stack_unchanged(void)
{
    static const uint8_t raw[] = {0x7C, 1, 2, 3, 4, 5, 6};
    rtosTick = 5000;
    TEST_ASSERT_FALSE(link->isConnected());
    from_c5(FC_LINK_CRTP, raw, sizeof raw);
    CRTPPacket p;
    TEST_ASSERT_EQUAL(0, link->receivePacket(&p));
    TEST_ASSERT_EQUAL(sizeof raw - 1, p.size);
    TEST_ASSERT_EQUAL_HEX8(0x7C, p.header);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(raw, p.raw, sizeof raw);
    TEST_ASSERT_NOT_EQUAL(0, link->receivePacket(&p)); /* exactly one */
    TEST_ASSERT_TRUE(link->isConnected());
    static const uint8_t status[FC_LINK_STATUS_SIZE] = {3};
    from_c5(FC_LINK_SWARM_STATUS, status, sizeof status); /* status is not CRTP */
    TEST_ASSERT_NOT_EQUAL(0, link->receivePacket(&p));
    rtosTick += 999;
    TEST_ASSERT_TRUE(link->isConnected());
    rtosTick += 1;
    from_c5(FC_LINK_IDLE, NULL, 0); /* the C5 is alive, but no CRTP from the ground: not connected */
    TEST_ASSERT_FALSE(link->isConnected());
}

static void test_outgoing_crtp_rides_the_next_transaction_then_idle(void)
{
    CRTPPacket p = {.size = 3, .header = 0x50, .data = {7, 8, 9}};
    TEST_ASSERT_TRUE(link->sendPacket(&p)); /* non-zero: crtpTxTask must not retry */
    uint16_t seq = sent().seq;
    from_c5(FC_LINK_IDLE, NULL, 0);
    fc_link_frame_t f = sent();
    TEST_ASSERT_EQUAL(FC_LINK_CRTP, f.type);
    TEST_ASSERT_EQUAL((uint16_t)(seq + 1), f.seq);
    TEST_ASSERT_EQUAL(4, f.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(p.raw, f.payload, 4);
    from_c5(FC_LINK_IDLE, NULL, 0);
    TEST_ASSERT_EQUAL(FC_LINK_IDLE, sent().type);
}

static void test_corrupt_or_missing_c5_frames_never_reach_crtp(void)
{
    static const uint8_t raw[] = {0x7C, 1, 2};
    c5LinkStats_t before = c5LinkGetStats();
    fc_link_frame_t f = {.type = FC_LINK_CRTP, .seq = c5Seq++, .len = sizeof raw};
    memcpy(f.payload, raw, sizeof raw);
    uint8_t rx[FC_LINK_FRAME_SIZE];
    TEST_ASSERT_TRUE(fc_link_encode(&f, rx));
    rx[6] ^= 0x10; /* payload bit flip: CRC mismatch */
    c5LinkExchange(rx, tx);
    memset(rx, 0, sizeof rx); /* no C5 on the bus */
    c5LinkExchange(rx, tx);
    memset(rx, 0xFF, sizeof rx); /* floating MISO */
    c5LinkExchange(rx, tx);
    CRTPPacket p;
    TEST_ASSERT_NOT_EQUAL(0, link->receivePacket(&p));
    TEST_ASSERT_EQUAL(before.rxErrors + 3, c5LinkGetStats().rxErrors);
    TEST_ASSERT_EQUAL(FC_LINK_IDLE, sent().type); /* the P4 keeps clocking valid frames */
}

static void test_lost_frames_are_counted_and_a_full_queue_drops_instead_of_blocking(void)
{
    c5LinkStats_t before = c5LinkGetStats();
    c5Seq += 3; /* three C5 frames lost */
    from_c5(FC_LINK_IDLE, NULL, 0);
    TEST_ASSERT_EQUAL(before.rxGaps + 1, c5LinkGetStats().rxGaps);

    static const uint8_t raw[] = {0x7C, 1};
    for (int i = 0; i < 20; i++) from_c5(FC_LINK_CRTP, raw, sizeof raw);
    TEST_ASSERT_EQUAL(before.rxDropped + 4, c5LinkGetStats().rxDropped); /* 16-deep queue */
    CRTPPacket p = {.size = 1};
    int accepted = 0;
    for (int i = 0; i < 20; i++) accepted += link->sendPacket(&p) ? 1 : 0;
    TEST_ASSERT_EQUAL(16, accepted); /* full: false, so crtpTxTask retries later */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crtp_from_the_c5_reaches_the_crtp_stack_unchanged);
    RUN_TEST(test_outgoing_crtp_rides_the_next_transaction_then_idle);
    RUN_TEST(test_corrupt_or_missing_c5_frames_never_reach_crtp);
    RUN_TEST(test_lost_frames_are_counted_and_a_full_queue_drops_instead_of_blocking);
    return UNITY_END();
}
