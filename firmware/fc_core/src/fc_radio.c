/* C5 envelope <-> SPI frame bridge; docs/protocols.md section 5. SPDX-License-Identifier: GPL-3.0-or-later */
#include "fc_radio.h"

#include <string.h>

bool fc_radio_init(fc_radio_t *r, const uint8_t key[16], uint32_t net_id, uint8_t id, uint32_t persisted)
{
    bool ok = fc_link_aead_init(&r->aead, key, net_id); /* zeroes the context */
    r->id = id;
    r->counter = r->counterLimit = persisted;
    r->spiSeq = 0;
    r->rejected = 0;
    return ok;
}

void fc_radio_free(fc_radio_t *r) { fc_link_aead_free(&r->aead); }

uint32_t fc_radio_reserve_due(const fc_radio_t *r)
{
    if (r->counterLimit - r->counter > FC_RADIO_COUNTER_BLOCK / 2) return 0;
    if (r->counterLimit > UINT32_MAX - FC_RADIO_COUNTER_BLOCK) return 0; /* spent: rekey */
    return r->counterLimit + FC_RADIO_COUNTER_BLOCK;
}

void fc_radio_reserved(fc_radio_t *r, uint32_t limit)
{
    if (limit > r->counterLimit) r->counterLimit = limit;
}

bool fc_radio_from_air(fc_radio_t *r, const uint8_t *env, size_t n, uint8_t frame[FC_LINK_FRAME_SIZE])
{
    uint8_t plain[FC_LINK_ENV_MAX_PLAIN], sender;
    int len = fc_link_open(&r->aead, env, n, &sender, plain);
    fc_link_frame_t f = {.seq = r->spiSeq};
    bool ok = len >= 1;
    if (ok) {
        f.type = plain[0];
        f.len = (uint8_t)(len - 1);
        memcpy(f.payload, plain + 1, f.len <= FC_LINK_MAX_PAYLOAD ? f.len : 0);
        /* Group key: any member could seal CRTP, so commands are accepted only from the ground id. */
        ok = f.len <= FC_LINK_MAX_PAYLOAD && (f.type != FC_LINK_CRTP || sender == FC_RADIO_GROUND_ID) &&
             f.type != FC_LINK_IDLE && fc_link_encode(&f, frame);
    }
    if (!ok) {
        r->rejected++;
        return false;
    }
    r->spiSeq++;
    return true;
}

size_t fc_radio_to_air(fc_radio_t *r, const uint8_t *frame, size_t n, uint8_t env[FC_RADIO_ENV_MAX])
{
    fc_link_frame_t f;
    if (fc_link_decode(frame, n, &f) != FC_LINK_OK) {
        r->rejected++;
        return 0;
    }
    if (f.type == FC_LINK_IDLE || r->counter >= r->counterLimit) return 0;
    uint8_t plain[1 + FC_LINK_MAX_PAYLOAD];
    plain[0] = f.type;
    memcpy(plain + 1, f.payload, f.len);
    size_t out = fc_link_seal(&r->aead, r->id, r->counter, plain, 1u + f.len, env, FC_RADIO_ENV_MAX);
    if (out) r->counter++;
    return out;
}
