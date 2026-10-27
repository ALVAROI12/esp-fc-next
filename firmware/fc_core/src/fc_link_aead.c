/* AES-128-CCM radio envelope; format in docs/protocols.md section 3. SPDX-License-Identifier: GPL-3.0-or-later */
#include "fc_link_aead.h"

#include <string.h>

static void nonce(uint8_t n[FC_LINK_ENV_NONCE_SIZE], uint32_t net_id, const uint8_t h[FC_LINK_ENV_HEADER_SIZE])
{
    memset(n, 0, FC_LINK_ENV_NONCE_SIZE);
    for (int i = 0; i < 4; i++) n[i] = (uint8_t)(net_id >> (8 * i));
    memcpy(n + 4, h + 1, 5); /* sender + counter */
}

bool fc_link_aead_init(fc_link_aead_t *a, const uint8_t key[16], uint32_t net_id)
{
    memset(a, 0, sizeof(*a));
    mbedtls_ccm_init(&a->ccm);
    a->net_id = net_id;
    return mbedtls_ccm_setkey(&a->ccm, MBEDTLS_CIPHER_ID_AES, key, 128) == 0;
}

void fc_link_aead_free(fc_link_aead_t *a) { mbedtls_ccm_free(&a->ccm); }

size_t fc_link_seal(fc_link_aead_t *a, uint8_t sender, uint32_t counter, const uint8_t *plain, size_t n,
                    uint8_t *out, size_t cap)
{
    size_t total = FC_LINK_ENV_HEADER_SIZE + n + FC_LINK_ENV_TAG_SIZE;
    if (n > FC_LINK_ENV_MAX_PLAIN || cap < total) return 0;
    out[0] = FC_LINK_ENV_VERSION;
    out[1] = sender;
    for (int i = 0; i < 4; i++) out[2 + i] = (uint8_t)(counter >> (8 * i));
    uint8_t iv[FC_LINK_ENV_NONCE_SIZE];
    nonce(iv, a->net_id, out);
    int err = mbedtls_ccm_encrypt_and_tag(&a->ccm, n, iv, sizeof iv, out, FC_LINK_ENV_HEADER_SIZE, plain,
                                          out + FC_LINK_ENV_HEADER_SIZE, out + FC_LINK_ENV_HEADER_SIZE + n,
                                          FC_LINK_ENV_TAG_SIZE);
    return err ? 0 : total;
}

int fc_link_open(fc_link_aead_t *a, const uint8_t *in, size_t n, uint8_t *sender, uint8_t *plain)
{
    if (n < FC_LINK_ENV_HEADER_SIZE + FC_LINK_ENV_TAG_SIZE || in[0] != FC_LINK_ENV_VERSION) return -1;
    size_t len = n - FC_LINK_ENV_HEADER_SIZE - FC_LINK_ENV_TAG_SIZE;
    if (len > FC_LINK_ENV_MAX_PLAIN) return -1;
    uint8_t iv[FC_LINK_ENV_NONCE_SIZE];
    nonce(iv, a->net_id, in);
    if (mbedtls_ccm_auth_decrypt(&a->ccm, len, iv, sizeof iv, in, FC_LINK_ENV_HEADER_SIZE,
                                 in + FC_LINK_ENV_HEADER_SIZE, plain, in + FC_LINK_ENV_HEADER_SIZE + len,
                                 FC_LINK_ENV_TAG_SIZE) != 0) {
        return -1; /* mbedtls zeroes plain on failure */
    }
    uint32_t counter = in[2] | (uint32_t)in[3] << 8 | (uint32_t)in[4] << 16 | (uint32_t)in[5] << 24;
    if (!fc_link_replay_accept(&a->windows[in[1]], counter)) return -1; /* only after authentication */
    *sender = in[1];
    return (int)len;
}
