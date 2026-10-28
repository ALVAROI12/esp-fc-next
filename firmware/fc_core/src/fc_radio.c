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
    bool ground = r->id == FC_RADIO_GROUND_ID, ok = false;
    fc_link_frame_t f = {.seq = r->spiSeq};
    if (len >= 3 && len - 1 <= (int)FC_LINK_MAX_PAYLOAD && plain[0] == FC_LINK_CRTP) {
        if (plain[1] != r->id && plain[1] != FC_RADIO_ALL) return false; /* for another member */
        /* Group key: any member could seal CRTP, so drones take commands only from the ground; the ground
         * hands drone telemetry to the PC tagged with its sender. */
        f.type = ground ? FC_LINK_CRTP_PEER : FC_LINK_CRTP;
        f.len = (uint8_t)(ground ? len - 1 : len - 2);
        if (ground) f.payload[0] = sender;
        memcpy(f.payload + (ground ? 1 : 0), plain + 2, (size_t)len - 2);
        ok = ground ? sender != FC_RADIO_GROUND_ID : sender == FC_RADIO_GROUND_ID;
    } else if (len >= 1 && len - 1 <= (int)FC_LINK_MAX_PAYLOAD && plain[0] == FC_LINK_SWARM_STATUS) {
        f.type = FC_LINK_SWARM_STATUS;
        f.len = (uint8_t)(len - 1);
        memcpy(f.payload, plain + 1, f.len);
        ok = true;
    }
    if (!ok || !fc_link_encode(&f, frame)) { /* forged/IDLE/unknown type, bad size, CRTP from a non-ground */
        r->rejected++;
        return false;
    }
    r->spiSeq++;
    return true;
}

size_t fc_radio_to_air(fc_radio_t *r, const uint8_t *frame, size_t n, uint8_t env[FC_RADIO_ENV_MAX])
{
    fc_link_frame_t f;
    bool ground = r->id == FC_RADIO_GROUND_ID;
    uint8_t plain[2 + FC_LINK_MAX_PAYLOAD];
    size_t m = 0;
    if (fc_link_decode(frame, n, &f) == FC_LINK_OK) {
        if (f.type == FC_LINK_IDLE) return 0;
        plain[0] = f.type == FC_LINK_CRTP_PEER ? FC_LINK_CRTP : f.type;
        if (f.type == FC_LINK_SWARM_STATUS || (f.type == FC_LINK_CRTP_PEER && ground && f.len >= 2)) {
            memcpy(plain + 1, f.payload, f.len); /* status, or dest + CRTP from the PC */
            m = 1u + f.len;
        } else if (f.type == FC_LINK_CRTP && !ground && f.len >= 1) {
            plain[1] = FC_RADIO_GROUND_ID; /* drone telemetry */
            memcpy(plain + 2, f.payload, f.len);
            m = 2u + f.len;
        }
    }
    if (!m) { /* corrupt, or the wrong CRTP form for this end */
        r->rejected++;
        return 0;
    }
    if (r->counter >= r->counterLimit) return 0;
    size_t out = fc_link_seal(&r->aead, r->id, r->counter, plain, m, env, FC_RADIO_ENV_MAX);
    if (out) r->counter++;
    return out;
}
