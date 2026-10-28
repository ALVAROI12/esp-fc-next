/* Unit tests for the C5 envelope <-> SPI frame bridge (Task 1.7). SPDX-License-Identifier: GPL-3.0-or-later */
#include <string.h>

#include "fc_radio.h"
#include "unity.h"

static const uint8_t KEY[16] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6};
static const uint32_t NET = 0x5EEDF00D;
static fc_radio_t ground, drone, other;

static void ready(fc_radio_t *r, uint8_t id, uint32_t persisted)
{
    TEST_ASSERT_TRUE(fc_radio_init(r, KEY, NET, id, persisted));
    uint32_t limit = fc_radio_reserve_due(r);
    TEST_ASSERT_NOT_EQUAL(0, limit);
    fc_radio_reserved(r, limit);
}

void setUp(void)
{
    ready(&ground, FC_RADIO_GROUND_ID, 0);
    ready(&drone, 1, 0);
    ready(&other, 3, 0);
}
void tearDown(void)
{
    fc_radio_free(&ground);
    fc_radio_free(&drone);
    fc_radio_free(&other);
}

/* What the P4 (or the ground bridge) clocks into its C5: one encoded SPI frame. */
static void spi(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t out[FC_LINK_FRAME_SIZE])
{
    fc_link_frame_t f = {.type = type, .len = len};
    memcpy(f.payload, payload, len);
    TEST_ASSERT_TRUE(fc_link_encode(&f, out));
}

/* The PC hands the ground C5 one CRTP packet for dest (a drone id or FC_RADIO_ALL). */
static size_t ground_sends(uint8_t dest, const uint8_t *crtp, uint8_t len, uint8_t env[FC_RADIO_ENV_MAX])
{
    uint8_t peer[1 + 31], in[FC_LINK_FRAME_SIZE];
    peer[0] = dest;
    memcpy(peer + 1, crtp, len);
    spi(FC_LINK_CRTP_PEER, peer, (uint8_t)(1 + len), in);
    return fc_radio_to_air(&ground, in, sizeof in, env);
}

static fc_link_frame_t decoded(const uint8_t frame[FC_LINK_FRAME_SIZE])
{
    fc_link_frame_t f;
    TEST_ASSERT_EQUAL(FC_LINK_OK, fc_link_decode(frame, FC_LINK_FRAME_SIZE, &f));
    return f;
}

static void test_ground_crtp_reaches_only_the_addressed_drone_unchanged(void)
{
    static const uint8_t crtp[] = {0x7C, 1, 2, 3, 4, 5};
    uint8_t env[FC_RADIO_ENV_MAX], out[FC_LINK_FRAME_SIZE];
    for (uint16_t i = 0; i < 3; i++) {
        size_t n = ground_sends(1, crtp, sizeof crtp, env);
        TEST_ASSERT_EQUAL(FC_LINK_ENV_HEADER_SIZE + 2 + sizeof crtp + FC_LINK_ENV_TAG_SIZE, n);
        TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
        fc_link_frame_t f = decoded(out);
        TEST_ASSERT_EQUAL(FC_LINK_CRTP, f.type);
        TEST_ASSERT_EQUAL(i, f.seq); /* C5 -> P4 numbering, not the sender's */
        TEST_ASSERT_EQUAL(sizeof crtp, f.len);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(crtp, f.payload, sizeof crtp);
        TEST_ASSERT_FALSE(fc_radio_from_air(&other, env, n, out)); /* drone 3 is not addressed */
    }
    size_t n = ground_sends(FC_RADIO_ALL, crtp, sizeof crtp, env); /* e.g. Crazyswarm2 broadcast land */
    TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
    TEST_ASSERT_TRUE(fc_radio_from_air(&other, env, n, out));
    TEST_ASSERT_EQUAL(0, drone.rejected + other.rejected); /* another drone's packet is not an error */
}

static void test_drone_telemetry_reaches_the_ground_tagged_with_its_sender(void)
{
    static const uint8_t crtp[] = {0x5C, 9, 8};
    uint8_t in[FC_LINK_FRAME_SIZE], env[FC_RADIO_ENV_MAX], out[FC_LINK_FRAME_SIZE];
    spi(FC_LINK_CRTP, crtp, sizeof crtp, in);
    size_t n = fc_radio_to_air(&other, in, sizeof in, env);
    TEST_ASSERT_TRUE(fc_radio_from_air(&ground, env, n, out));
    fc_link_frame_t f = decoded(out);
    TEST_ASSERT_EQUAL(FC_LINK_CRTP_PEER, f.type);
    TEST_ASSERT_EQUAL(1 + sizeof crtp, f.len);
    TEST_ASSERT_EQUAL(3, f.payload[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(crtp, f.payload + 1, sizeof crtp);
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, env, n, out)); /* addressed to the ground */
    TEST_ASSERT_EQUAL(0, drone.rejected);
}

/* All members share the group key, so a drone could seal a command; drones take CRTP only from id 0. Each
 * end also refuses the other end's CRTP form. Status from peers passes to every drone. */
static void test_only_the_ground_station_may_send_commands(void)
{
    uint8_t plain[] = {FC_LINK_CRTP, 1, 0x7C, 1}, env[FC_RADIO_ENV_MAX], out[FC_LINK_FRAME_SIZE];
    size_t n = fc_link_seal(&other.aead, 3, other.counter++, plain, sizeof plain, env, sizeof env);
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, env, n, out));
    TEST_ASSERT_EQUAL(1, drone.rejected);

    uint8_t in[FC_LINK_FRAME_SIZE], peer[] = {1, 0x7C};
    spi(FC_LINK_CRTP_PEER, peer, sizeof peer, in);
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&other, in, sizeof in, env)); /* a drone cannot address peers */
    spi(FC_LINK_CRTP, peer + 1, 1, in);
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&ground, in, sizeof in, env)); /* the ground must address */
    TEST_ASSERT_EQUAL(1, other.rejected);
    TEST_ASSERT_EQUAL(1, ground.rejected);

    uint8_t status[FC_LINK_STATUS_SIZE] = {3, 0, 1, 2};
    spi(FC_LINK_SWARM_STATUS, status, sizeof status, in);
    n = fc_radio_to_air(&other, in, sizeof in, env);
    TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
    fc_link_frame_t f = decoded(out);
    TEST_ASSERT_EQUAL(FC_LINK_SWARM_STATUS, f.type);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(status, f.payload, sizeof status);
}

static void test_tampered_replayed_or_idle_envelopes_never_reach_the_flight_chip(void)
{
    static const uint8_t crtp[] = {0x7C, 1, 2};
    uint8_t env[FC_RADIO_ENV_MAX], out[FC_LINK_FRAME_SIZE];
    size_t n = ground_sends(1, crtp, sizeof crtp, env), bits = n * 8;
    for (size_t bit = 0; bit < bits; bit++) {
        env[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        TEST_ASSERT_FALSE(fc_radio_from_air(&drone, env, n, out));
        env[bit / 8] ^= (uint8_t)(1u << (bit % 8));
    }
    TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, env, n, out)); /* replay */
    /* Authentic but not a frame the P4 should see: IDLE, unknown type, CRTP without a header byte. */
    uint8_t plain[3] = {FC_LINK_IDLE, 1, 0x7C}, sealed[FC_RADIO_ENV_MAX];
    n = fc_link_seal(&ground.aead, FC_RADIO_GROUND_ID, 1000, plain, 1, sealed, sizeof sealed);
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, sealed, n, out));
    plain[0] = FC_LINK_CRTP_PEER;
    n = fc_link_seal(&ground.aead, FC_RADIO_GROUND_ID, 1001, plain, 3, sealed, sizeof sealed);
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, sealed, n, out));
    plain[0] = FC_LINK_CRTP;
    n = fc_link_seal(&ground.aead, FC_RADIO_GROUND_ID, 1002, plain, 2, sealed, sizeof sealed);
    TEST_ASSERT_FALSE(fc_radio_from_air(&drone, sealed, n, out));
    TEST_ASSERT_EQUAL(bits + 4, drone.rejected);
}

static void test_idle_and_corrupt_spi_frames_are_not_sent(void)
{
    uint8_t in[FC_LINK_FRAME_SIZE], env[FC_RADIO_ENV_MAX];
    spi(FC_LINK_IDLE, NULL, 0, in);
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&drone, in, sizeof in, env));
    TEST_ASSERT_EQUAL(0, drone.rejected);
    static const uint8_t crtp[] = {0x7C};
    spi(FC_LINK_CRTP, crtp, sizeof crtp, in);
    in[5] ^= 1; /* CRC now wrong */
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&drone, in, sizeof in, env));
    TEST_ASSERT_EQUAL(1, drone.rejected);
    TEST_ASSERT_EQUAL(0, drone.counter); /* nothing consumed */
}

/* Counters come from a persisted reservation: nothing is sent before it is stored, a reboot resumes above
 * every counter that could have been used, and a receiver that never rebooted still accepts it. */
static void test_counter_reservation_survives_reboot_without_nonce_reuse(void)
{
    fc_radio_t r;
    TEST_ASSERT_TRUE(fc_radio_init(&r, KEY, NET, 2, 0));
    uint8_t in[FC_LINK_FRAME_SIZE], env[FC_RADIO_ENV_MAX], out[FC_LINK_FRAME_SIZE];
    uint8_t status[FC_LINK_STATUS_SIZE] = {2};
    spi(FC_LINK_SWARM_STATUS, status, sizeof status, in);
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&r, in, sizeof in, env)); /* not reserved yet */
    uint32_t stored = fc_radio_reserve_due(&r);
    TEST_ASSERT_EQUAL(FC_RADIO_COUNTER_BLOCK, stored);
    fc_radio_reserved(&r, stored);
    for (unsigned i = 0; i < FC_RADIO_COUNTER_BLOCK / 2 - 1; i++) {
        TEST_ASSERT_NOT_EQUAL(0, fc_radio_to_air(&r, in, sizeof in, env));
        TEST_ASSERT_EQUAL(0, fc_radio_reserve_due(&r));
    }
    size_t n = fc_radio_to_air(&r, in, sizeof in, env);
    TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
    TEST_ASSERT_EQUAL(2 * FC_RADIO_COUNTER_BLOCK, fc_radio_reserve_due(&r)); /* headroom down to half */
    while (r.counter < r.counterLimit) TEST_ASSERT_NOT_EQUAL(0, fc_radio_to_air(&r, in, sizeof in, env));
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&r, in, sizeof in, env)); /* stalls instead of overrunning */
    fc_radio_free(&r);

    TEST_ASSERT_TRUE(fc_radio_init(&r, KEY, NET, 2, stored)); /* reboot: read back the stored limit */
    fc_radio_reserved(&r, fc_radio_reserve_due(&r));
    TEST_ASSERT_EQUAL(stored, r.counter);
    n = fc_radio_to_air(&r, in, sizeof in, env);
    TEST_ASSERT_TRUE(fc_radio_from_air(&drone, env, n, out));
    fc_radio_free(&r);

    TEST_ASSERT_TRUE(fc_radio_init(&r, KEY, NET, 2, UINT32_MAX - 10)); /* counter space spent: rekey */
    TEST_ASSERT_EQUAL(0, fc_radio_reserve_due(&r));
    TEST_ASSERT_EQUAL(0, fc_radio_to_air(&r, in, sizeof in, env));
    fc_radio_free(&r);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ground_crtp_reaches_only_the_addressed_drone_unchanged);
    RUN_TEST(test_drone_telemetry_reaches_the_ground_tagged_with_its_sender);
    RUN_TEST(test_only_the_ground_station_may_send_commands);
    RUN_TEST(test_tampered_replayed_or_idle_envelopes_never_reach_the_flight_chip);
    RUN_TEST(test_idle_and_corrupt_spi_frames_are_not_sent);
    RUN_TEST(test_counter_reservation_survives_reboot_without_nonce_reuse);
    return UNITY_END();
}
