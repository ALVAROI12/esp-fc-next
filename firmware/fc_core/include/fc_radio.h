/*
 * fc_radio: C5 bridge between the sealed radio envelope (ESP-NOW) and the P4 SPI frame (docs/protocols.md
 * section 5). Radio-side code, not flight-loop code; portable so the same logic runs on host tests and the C5.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fc_link.h"
#include "fc_link_aead.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_RADIO_GROUND_ID 0u /* only this sender may deliver CRTP (commands) to drones */
#define FC_RADIO_ALL 0xFFu      /* CRTP destination: every drone (not a valid radio id) */
/* Envelope counters are reserved in blocks persisted before use, so a reboot never reuses a nonce. */
#define FC_RADIO_COUNTER_BLOCK 4096u
#define FC_RADIO_ENV_MAX (FC_LINK_ENV_HEADER_SIZE + 1u + FC_LINK_MAX_PAYLOAD + FC_LINK_ENV_TAG_SIZE)

typedef struct {
    fc_link_aead_t aead;
    uint8_t id;            /* own sender id */
    uint32_t counter;      /* next envelope counter */
    uint32_t counterLimit; /* persisted reservation: seal refuses at or above it */
    uint16_t spiSeq;       /* C5 -> P4 frame counter */
    uint32_t rejected;     /* envelopes or frames dropped */
} fc_radio_t;

/* persisted = the last limit read back from storage (0 on first boot). Nothing can be sent until
 * fc_radio_reserve_due() is persisted and confirmed with fc_radio_reserved(). */
bool fc_radio_init(fc_radio_t *r, const uint8_t key[16], uint32_t net_id, uint8_t id, uint32_t persisted);
void fc_radio_free(fc_radio_t *r);

/* New limit to persist before sending can continue, or 0 while the reservation still has headroom.
 * Returns 0 forever once the 32-bit counter space is spent (rekey). */
uint32_t fc_radio_reserve_due(const fc_radio_t *r);
/* Call after limit returned by fc_radio_reserve_due() is durably stored. */
void fc_radio_reserved(fc_radio_t *r, uint32_t limit);

/* Air plaintext: SWARM_STATUS = [type][status]; CRTP = [type][dest][raw CRTP], dest = a radio id or FC_RADIO_ALL.
 * A drone's C5 exchanges FC_LINK_CRTP frames with its P4 (telemetry goes to the ground); the ground C5 (id 0)
 * exchanges FC_LINK_CRTP_PEER frames with the PC (dest on the way out, sender on the way in). */

/* Air -> P4/PC: authenticates one envelope and encodes it as one SPI frame. False when it is not for this
 * radio, or rejected (counted). */
bool fc_radio_from_air(fc_radio_t *r, const uint8_t *env, size_t n, uint8_t frame[FC_LINK_FRAME_SIZE]);
/* P4 -> air: seals one SPI frame. Returns envelope length (<= FC_RADIO_ENV_MAX); 0 for IDLE, a rejected
 * frame (counted) or no counter reservation left. */
size_t fc_radio_to_air(fc_radio_t *r, const uint8_t *frame, size_t n, uint8_t env[FC_RADIO_ENV_MAX]);

#ifdef __cplusplus
}
#endif
