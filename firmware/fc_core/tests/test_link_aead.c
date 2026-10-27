/* Unit tests for the AES-CCM radio envelope (Task 1.7). SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdio.h>
#include <string.h>

#include "fc_link_aead.h"
#include "unity.h"

static const uint8_t KEY[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const uint32_t NET = 0x5EEDF00D;
static const uint8_t msg[] = "swarm status payload";
static fc_link_aead_t tx, rx;

void setUp(void)
{
    TEST_ASSERT_TRUE(fc_link_aead_init(&tx, KEY, NET));
    TEST_ASSERT_TRUE(fc_link_aead_init(&rx, KEY, NET));
}
void tearDown(void)
{
    fc_link_aead_free(&tx);
    fc_link_aead_free(&rx);
}

static size_t seal(uint8_t sender, uint32_t counter, uint8_t *out)
{
    return fc_link_seal(&tx, sender, counter, msg, sizeof msg, out, 128);
}

static void hex(const char *s, uint8_t *out)
{
    for (size_t i = 0; s[2 * i]; i++) {
        unsigned v;
        TEST_ASSERT_EQUAL(1, sscanf(s + 2 * i, "%2x", &v));
        out[i] = (uint8_t)v;
    }
}

/* RFC 3610 packet vector #1 (as in mbedtls tests/suites/test_suite_ccm.data): same 13-byte nonce / 8-byte
 * tag parameters as the envelope, so the bundled mbedtls build is checked against an external reference. */
static void test_bundled_mbedtls_matches_rfc3610_vector(void)
{
    uint8_t key[16], pt[23], iv[13], ad[8], expect[31], out[31];
    hex("C0C1C2C3C4C5C6C7C8C9CACBCCCDCECF", key);
    hex("08090A0B0C0D0E0F101112131415161718191A1B1C1D1E", pt);
    hex("00000003020100A0A1A2A3A4A5", iv);
    hex("0001020304050607", ad);
    hex("588C979A61C663D2F066D0C2C0F989806D5F6B61DAC38417E8D12CFDF926E0", expect);
    mbedtls_ccm_context c;
    mbedtls_ccm_init(&c);
    TEST_ASSERT_EQUAL(0, mbedtls_ccm_setkey(&c, MBEDTLS_CIPHER_ID_AES, key, 128));
    TEST_ASSERT_EQUAL(0, mbedtls_ccm_encrypt_and_tag(&c, 23, iv, 13, ad, 8, pt, out, out + 23, 8));
    mbedtls_ccm_free(&c);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out, 31);
}

/* Wire layout exactly as docs/protocols.md: header, nonce = net(le32) sender counter(le32) 0000, AAD = header. */
static void test_envelope_layout_matches_spec(void)
{
    uint8_t env[128], ref[128];
    size_t n = seal(0x42, 0x01020304, env);
    const uint8_t header[6] = {1, 0x42, 0x04, 0x03, 0x02, 0x01};
    const uint8_t iv[13] = {0x0D, 0xF0, 0xED, 0x5E, 0x42, 0x04, 0x03, 0x02, 0x01, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(header, env, 6);
    mbedtls_ccm_context c;
    mbedtls_ccm_init(&c);
    TEST_ASSERT_EQUAL(0, mbedtls_ccm_setkey(&c, MBEDTLS_CIPHER_ID_AES, KEY, 128));
    TEST_ASSERT_EQUAL(0, mbedtls_ccm_encrypt_and_tag(&c, sizeof msg, iv, 13, header, 6, msg, ref,
                                                     ref + sizeof msg, 8));
    mbedtls_ccm_free(&c);
    TEST_ASSERT_EQUAL(6 + sizeof msg + 8, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ref, env + 6, sizeof msg + 8);
}

static void test_round_trip(void)
{
    uint8_t env[128], plain[FC_LINK_ENV_MAX_PLAIN], sender = 0;
    size_t n = seal(7, 42, env);
    TEST_ASSERT_EQUAL(sizeof msg, fc_link_open(&rx, env, n, &sender, plain));
    TEST_ASSERT_EQUAL(7, sender);
    TEST_ASSERT_EQUAL_MEMORY(msg, plain, sizeof msg);
}

static void test_any_bit_flip_is_rejected_and_does_not_advance_window(void)
{
    uint8_t env[128], bad[128], plain[FC_LINK_ENV_MAX_PLAIN], sender;
    size_t n = seal(3, 10, env);
    for (size_t i = 0; i < n; i++) {
        for (int bit = 0; bit < 8; bit++) {
            memcpy(bad, env, n);
            bad[i] ^= (uint8_t)(1u << bit);
            TEST_ASSERT_EQUAL_MESSAGE(-1, fc_link_open(&rx, bad, n, &sender, plain), "tampered bit");
        }
    }
    TEST_ASSERT_EQUAL(-1, fc_link_open(&rx, env, n - 1, &sender, plain)); /* truncated */
    for (unsigned s = 0; s < 256; s++) TEST_ASSERT_FALSE(rx.windows[s].started); /* no forged counter got in */
    TEST_ASSERT_EQUAL(sizeof msg, fc_link_open(&rx, env, n, &sender, plain));
}

static void test_replay_is_rejected_per_sender(void)
{
    uint8_t a[128], b[128], plain[FC_LINK_ENV_MAX_PLAIN], sender;
    size_t na = seal(1, 5, a), nb = seal(2, 5, b);
    TEST_ASSERT_EQUAL(sizeof msg, fc_link_open(&rx, a, na, &sender, plain));
    TEST_ASSERT_EQUAL(-1, fc_link_open(&rx, a, na, &sender, plain));
    TEST_ASSERT_EQUAL(sizeof msg, fc_link_open(&rx, b, nb, &sender, plain)); /* same counter, other sender */
}

static void test_wrong_key_or_network_is_rejected(void)
{
    uint8_t env[128], plain[FC_LINK_ENV_MAX_PLAIN], sender, key[16];
    size_t n = seal(1, 1, env);
    fc_link_aead_t other;
    memcpy(key, KEY, 16);
    key[0] ^= 0x80;
    TEST_ASSERT_TRUE(fc_link_aead_init(&other, key, NET));
    TEST_ASSERT_EQUAL(-1, fc_link_open(&other, env, n, &sender, plain));
    fc_link_aead_free(&other);
    TEST_ASSERT_TRUE(fc_link_aead_init(&other, KEY, NET + 1));
    TEST_ASSERT_EQUAL(-1, fc_link_open(&other, env, n, &sender, plain));
    fc_link_aead_free(&other);
}

static void test_size_limits(void)
{
    uint8_t big[FC_LINK_ENV_MAX_PLAIN + 1] = {0}, env[128], plain[FC_LINK_ENV_MAX_PLAIN], sender;
    TEST_ASSERT_EQUAL(0, fc_link_seal(&tx, 1, 1, big, sizeof big, env, sizeof env));
    TEST_ASSERT_EQUAL(0, fc_link_seal(&tx, 1, 1, msg, sizeof msg, env, 10));
    size_t n = fc_link_seal(&tx, 1, 1, msg, 0, env, sizeof env);
    TEST_ASSERT_EQUAL(FC_LINK_ENV_HEADER_SIZE + FC_LINK_ENV_TAG_SIZE, n);
    TEST_ASSERT_EQUAL(0, fc_link_open(&rx, env, n, &sender, plain)); /* empty payload is authentic */
    env[0] = 2;
    TEST_ASSERT_EQUAL(-1, fc_link_open(&rx, env, n, &sender, plain)); /* unknown version */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bundled_mbedtls_matches_rfc3610_vector);
    RUN_TEST(test_envelope_layout_matches_spec);
    RUN_TEST(test_round_trip);
    RUN_TEST(test_any_bit_flip_is_rejected_and_does_not_advance_window);
    RUN_TEST(test_replay_is_rejected_per_sender);
    RUN_TEST(test_wrong_key_or_network_is_rejected);
    RUN_TEST(test_size_limits);
    return UNITY_END();
}
