/*
 * fc_link_aead: AES-128-CCM radio envelope with per-sender anti-replay (docs/protocols.md section 3).
 * Radio-side code (C5), not flight-loop code. Uses mbedtls as bundled with ESP-IDF; the key is set once in
 * fc_link_aead_init (mbedtls allocates there), so seal/open allocate nothing per packet.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fc_link.h"
#include "mbedtls/ccm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_LINK_ENV_VERSION 1u
#define FC_LINK_ENV_HEADER_SIZE 6u /* version, sender, counter (le32) */
#define FC_LINK_ENV_TAG_SIZE 8u
#define FC_LINK_ENV_NONCE_SIZE 13u
#define FC_LINK_ENV_MAX_PLAIN 64u

typedef struct {
    mbedtls_ccm_context ccm;
    uint32_t net_id;
    fc_link_replay_t windows[256]; /* one per sender id */
} fc_link_aead_t;

/* Returns false if the key cannot be set (context is still safe to free). */
bool fc_link_aead_init(fc_link_aead_t *a, const uint8_t key[16], uint32_t net_id);
void fc_link_aead_free(fc_link_aead_t *a);

/* Seals plain[n] (n <= FC_LINK_ENV_MAX_PLAIN) as sender/counter. Returns bytes written, 0 on bad size.
 * The caller must never reuse a counter under the same key (persist it or rekey across reboots). */
size_t fc_link_seal(fc_link_aead_t *a, uint8_t sender, uint32_t counter, const uint8_t *plain, size_t n,
                    uint8_t *out, size_t cap);

/* Authenticates first, then applies the sender's replay window. Returns plaintext length, or -1 when
 * rejected (bad version/size/tag, wrong key or network, replay). plain must hold FC_LINK_ENV_MAX_PLAIN bytes. */
int fc_link_open(fc_link_aead_t *a, const uint8_t *in, size_t n, uint8_t *sender, uint8_t *plain);

#ifdef __cplusplus
}
#endif
