/* libFuzzer target for the AES-CCM envelope (fc_link_aead). Build: make link-fuzz FUZZ_TARGET=fuzz_link_aead.
 * Arbitrary bytes must never open (or crash); sealing the input must open to the same bytes exactly once,
 * and any single-bit change to the sealed packet must be rejected. SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdlib.h>
#include <string.h>

#include "fc_link_aead.h"

static const uint8_t KEY[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    fc_link_aead_t a;
    if (!fc_link_aead_init(&a, KEY, 0x45535046u)) abort();
    uint8_t sender, plain[FC_LINK_ENV_MAX_PLAIN];
    if (fc_link_open(&a, data, size, &sender, plain) != -1) abort(); /* 2^-64 forgery chance: treat as bug */

    if (size >= 6) {
        uint8_t from = data[0];
        uint32_t counter = data[1] | (uint32_t)data[2] << 8 | (uint32_t)data[3] << 16 | (uint32_t)data[4] << 24;
        uint8_t flip = data[5];
        const uint8_t *msg = data + 6;
        size_t n = size - 6;
        uint8_t packet[FC_LINK_ENV_HEADER_SIZE + FC_LINK_ENV_MAX_PLAIN + FC_LINK_ENV_TAG_SIZE];
        size_t len = fc_link_seal(&a, from, counter, msg, n, packet, sizeof packet);
        if ((len == 0) != (n > FC_LINK_ENV_MAX_PLAIN)) abort();
        if (len) {
            uint8_t bad[sizeof packet];
            memcpy(bad, packet, len);
            bad[(flip >> 3) % len] ^= (uint8_t)(1u << (flip & 7));
            if (fc_link_open(&a, bad, len, &sender, plain) != -1) abort();
            if (fc_link_open(&a, packet, len, &sender, plain) != (int)n || sender != from ||
                memcmp(plain, msg, n) != 0) {
                abort();
            }
            if (fc_link_open(&a, packet, len, &sender, plain) != -1) abort(); /* replay */
        }
    }
    fc_link_aead_free(&a);
    return 0;
}
