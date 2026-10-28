/* Unit tests for fc_link wire formats (Task 1.7). SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <string.h>

#include "fc_link.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_crc16_ccitt_false_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x29B1, fc_link_crc16((const uint8_t *)"123456789", 9));
}

static void test_frame_round_trip_and_zero_tail(void)
{
    fc_link_frame_t f = {.type = FC_LINK_CRTP, .seq = 0xBEEF, .len = 31}, g;
    for (int i = 0; i < 31; i++) f.payload[i] = (uint8_t)(i * 7);
    uint8_t buf[FC_LINK_FRAME_SIZE];
    TEST_ASSERT_TRUE(fc_link_encode(&f, buf));
    TEST_ASSERT_EQUAL(FC_LINK_OK, fc_link_decode(buf, sizeof buf, &g));
    TEST_ASSERT_EQUAL(f.type, g.type);
    TEST_ASSERT_EQUAL_HEX16(f.seq, g.seq);
    TEST_ASSERT_EQUAL(31, g.len);
    TEST_ASSERT_EQUAL_MEMORY(f.payload, g.payload, 31);
    for (unsigned i = FC_LINK_HEADER_SIZE + 31 + FC_LINK_CRC_SIZE; i < sizeof buf; i++) TEST_ASSERT_EQUAL(0, buf[i]);
}

static void test_encode_rejects_bad_type_and_oversize(void)
{
    uint8_t buf[FC_LINK_FRAME_SIZE] = {0x11};
    fc_link_frame_t f = {.type = FC_LINK_TYPE_COUNT};
    TEST_ASSERT_FALSE(fc_link_encode(&f, buf));
    f = (fc_link_frame_t){.type = FC_LINK_CRTP, .len = 32};
    TEST_ASSERT_FALSE(fc_link_encode(&f, buf));
    f = (fc_link_frame_t){.type = FC_LINK_IDLE, .len = 1};
    TEST_ASSERT_FALSE(fc_link_encode(&f, buf));
    f = (fc_link_frame_t){.type = FC_LINK_CRTP_PEER, .len = 33}; /* peer id + 31-byte CRTP is the maximum */
    TEST_ASSERT_FALSE(fc_link_encode(&f, buf));
    TEST_ASSERT_EQUAL(0x11, buf[0]); /* nothing written */
    f.len = 32;
    TEST_ASSERT_TRUE(fc_link_encode(&f, buf));
}

static void test_decode_rejects_each_corruption(void)
{
    fc_link_frame_t f = {.type = FC_LINK_SWARM_STATUS, .seq = 3, .len = FC_LINK_STATUS_SIZE}, g;
    uint8_t buf[FC_LINK_FRAME_SIZE], bad[FC_LINK_FRAME_SIZE];
    TEST_ASSERT_TRUE(fc_link_encode(&f, buf));

    TEST_ASSERT_EQUAL(FC_LINK_ERR_SHORT, fc_link_decode(buf, 6, &g));
    TEST_ASSERT_EQUAL(FC_LINK_ERR_SHORT, fc_link_decode(buf, FC_LINK_HEADER_SIZE + f.len + 1, &g));
    memcpy(bad, buf, sizeof bad); bad[0] ^= 1;
    TEST_ASSERT_EQUAL(FC_LINK_ERR_MAGIC, fc_link_decode(bad, sizeof bad, &g));
    memcpy(bad, buf, sizeof bad); bad[1] = FC_LINK_TYPE_COUNT;
    TEST_ASSERT_EQUAL(FC_LINK_ERR_TYPE, fc_link_decode(bad, sizeof bad, &g));
    memcpy(bad, buf, sizeof bad); bad[4] = 255;
    TEST_ASSERT_EQUAL(FC_LINK_ERR_LEN, fc_link_decode(bad, sizeof bad, &g));
    memcpy(bad, buf, sizeof bad); bad[FC_LINK_FRAME_SIZE - 1] = 1; /* spec: the unused tail is zero */
    TEST_ASSERT_EQUAL(FC_LINK_ERR_TAIL, fc_link_decode(bad, sizeof bad, &g));
    for (unsigned i = 1; i < FC_LINK_HEADER_SIZE + f.len + FC_LINK_CRC_SIZE; i++) {
        if (i == 1 || i == 4) continue; /* type/len flips are caught by the checks above */
        memcpy(bad, buf, sizeof bad); bad[i] ^= 0x40;
        TEST_ASSERT_EQUAL_MESSAGE(FC_LINK_ERR_CRC, fc_link_decode(bad, sizeof bad, &g), "single-bit flip");
    }
}

static void test_replay_window_accepts_once_and_reorders(void)
{
    fc_link_replay_t w = {0};
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 100));
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 100));
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 102));
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 101)); /* late but inside window */
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 101));
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 133));
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 103));  /* age 30 */
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 101)); /* age 32: too old */
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 102)); /* age 31, already seen */
    TEST_ASSERT_TRUE(fc_link_replay_accept(&w, 1000)); /* big jump clears window */
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 133));
    TEST_ASSERT_FALSE(fc_link_replay_accept(&w, 1000));
}

static void test_status_round_trip_quantizes_to_mm(void)
{
    fc_link_status_msg_t m = {.id = 4, .state = 2, .pos = {1.2344f, -32.0f, 0.0004f},
                              .vel = {-0.5f, 0.25f, 3.0f}, .battery_mv = 7400, .time_ms = 0xA1B2C3D4}, r;
    uint8_t buf[FC_LINK_STATUS_SIZE];
    TEST_ASSERT_TRUE(fc_link_status_pack(&m, buf));
    fc_link_status_unpack(buf, &r);
    TEST_ASSERT_EQUAL(4, r.id);
    TEST_ASSERT_EQUAL(2, r.state);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.0005f, m.pos[i], r.pos[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.0005f, m.vel[i], r.vel[i]);
    }
    TEST_ASSERT_EQUAL(7400, r.battery_mv);
    TEST_ASSERT_EQUAL_HEX32(0xA1B2C3D4, r.time_ms);
}

static void test_status_pack_rejects_nan_and_out_of_range(void)
{
    fc_link_status_msg_t m = {.pos = {0, 0, 0}};
    uint8_t buf[FC_LINK_STATUS_SIZE];
    m.pos[1] = NAN;
    TEST_ASSERT_FALSE(fc_link_status_pack(&m, buf));
    m.pos[1] = 0; m.vel[2] = INFINITY;
    TEST_ASSERT_FALSE(fc_link_status_pack(&m, buf));
    m.vel[2] = 0; m.pos[0] = 32.8f;
    TEST_ASSERT_FALSE(fc_link_status_pack(&m, buf));
}

/* Ground serial link: frames survive leading console text, a corrupted frame and a lost byte. */
static void test_stream_resynchronizes_after_noise_corruption_and_loss(void)
{
    uint8_t a[FC_LINK_FRAME_SIZE], b[FC_LINK_FRAME_SIZE], c[FC_LINK_FRAME_SIZE], out[FC_LINK_FRAME_SIZE];
    fc_link_frame_t fa = {.type = FC_LINK_CRTP, .seq = 1, .len = 3, .payload = {0x7C, 1, 2}};
    fc_link_frame_t fb = {.type = FC_LINK_CRTP_PEER, .seq = 2, .len = 2, .payload = {4, 0x5C}};
    fc_link_frame_t fc = {.type = FC_LINK_IDLE, .seq = 3};
    TEST_ASSERT_TRUE(fc_link_encode(&fa, a) && fc_link_encode(&fb, b) && fc_link_encode(&fc, c));
    uint8_t wire[16 + 5 * FC_LINK_FRAME_SIZE];
    size_t n = 0;
    memcpy(wire + n, "I (12) radio: \xA5\n", 16); /* console text, including a magic byte */
    n += 16;
    memcpy(wire + n, a, sizeof a);
    n += sizeof a;
    memcpy(wire + n, b, sizeof b);
    wire[n + 6] ^= 1; /* b corrupted in transit */
    n += sizeof b;
    memcpy(wire + n, c + 1, sizeof c - 1); /* c lost its first byte */
    n += sizeof c - 1;
    memcpy(wire + n, c, 40); /* c lost only tail bytes: header and CRC intact */
    n += 40;
    memcpy(wire + n, a, sizeof a);
    n += sizeof a;

    fc_link_stream_t s = {0};
    unsigned got = 0;
    uint16_t seqs[4];
    for (size_t i = 0; i < n; i++) {
        if (!fc_link_stream_push(&s, wire[i], out)) continue;
        fc_link_frame_t f;
        TEST_ASSERT_EQUAL(FC_LINK_OK, fc_link_decode(out, sizeof out, &f));
        TEST_ASSERT_LESS_THAN(4, got);
        seqs[got++] = f.seq;
    }
    TEST_ASSERT_EQUAL(2, got); /* a, then a again: b and both truncated copies of c are skipped */
    TEST_ASSERT_EQUAL(1, seqs[0]);
    TEST_ASSERT_EQUAL(1, seqs[1]);
    TEST_ASSERT_EQUAL(n - 2 * FC_LINK_FRAME_SIZE, s.skipped);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc16_ccitt_false_check_value);
    RUN_TEST(test_frame_round_trip_and_zero_tail);
    RUN_TEST(test_encode_rejects_bad_type_and_oversize);
    RUN_TEST(test_decode_rejects_each_corruption);
    RUN_TEST(test_replay_window_accepts_once_and_reorders);
    RUN_TEST(test_status_round_trip_quantizes_to_mm);
    RUN_TEST(test_status_pack_rejects_nan_and_out_of_range);
    RUN_TEST(test_stream_resynchronizes_after_noise_corruption_and_loss);
    return UNITY_END();
}
