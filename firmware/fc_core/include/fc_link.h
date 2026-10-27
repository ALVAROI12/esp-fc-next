/*
 * fc_link: wire formats shared by the P4 flight chip and the C5 radio chip (Task 1.7).
 * Spec: docs/protocols.md. Pure byte-level code: no RTOS, no allocation, no float->struct casts,
 * explicit little-endian, so the same file builds on host, P4 and C5 and can be fuzzed.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- P4 <-> C5 SPI frame: magic, type, seq(le16), len, payload[len], crc16(le) ---- */
#define FC_LINK_MAGIC 0xA5u
#define FC_LINK_HEADER_SIZE 5u
#define FC_LINK_CRC_SIZE 2u
#define FC_LINK_MAX_PAYLOAD 56u
#define FC_LINK_FRAME_SIZE 64u /* fixed SPI transaction; unused tail is zero */

typedef enum {
    FC_LINK_IDLE = 0,       /* keep-alive, empty payload */
    FC_LINK_CRTP = 1,       /* one raw CRTP packet: header byte + up to 30 data bytes */
    FC_LINK_SWARM_STATUS = 2,
    FC_LINK_TYPE_COUNT
} fc_link_type_t;

typedef enum {
    FC_LINK_OK = 0,
    FC_LINK_ERR_SHORT,
    FC_LINK_ERR_MAGIC,
    FC_LINK_ERR_TYPE,
    FC_LINK_ERR_LEN,
    FC_LINK_ERR_CRC,
} fc_link_status_t;

typedef struct {
    uint8_t type;
    uint16_t seq;
    uint8_t len;
    uint8_t payload[FC_LINK_MAX_PAYLOAD];
} fc_link_frame_t;

uint16_t fc_link_crc16(const uint8_t *data, size_t n); /* CRC-16/CCITT-FALSE */

/* Writes FC_LINK_FRAME_SIZE bytes. Returns false (nothing written) for a bad type or length. */
bool fc_link_encode(const fc_link_frame_t *frame, uint8_t out[FC_LINK_FRAME_SIZE]);
/* Accepts any buffer; out is only valid when FC_LINK_OK is returned. */
fc_link_status_t fc_link_decode(const uint8_t *buf, size_t n, fc_link_frame_t *out);

/* ---- Anti-replay window for 32-bit message counters (one per peer) ---- */
typedef struct {
    bool started;
    uint32_t highest;
    uint32_t bitmap; /* bit i = highest - i already accepted */
} fc_link_replay_t;

/* Accepts each counter at most once; rejects counters older than 32 behind the highest seen.
 * Call only after the message authenticated, or a forged counter can advance the window. */
bool fc_link_replay_accept(fc_link_replay_t *w, uint32_t counter);

/* ---- 50 Hz swarm status broadcast (20 bytes) ---- */
#define FC_LINK_STATUS_SIZE 20u

typedef struct {
    uint8_t id;
    uint8_t state;        /* fc_failsafe_state_t */
    float pos[3];         /* m, world frame; encoded as int16 mm (+/-32.767 m) */
    float vel[3];         /* m/s; encoded as int16 mm/s */
    uint16_t battery_mv;
    uint32_t time_ms;
} fc_link_status_msg_t;

/* Returns false for non-finite or out-of-range position/velocity (nothing written). */
bool fc_link_status_pack(const fc_link_status_msg_t *m, uint8_t out[FC_LINK_STATUS_SIZE]);
void fc_link_status_unpack(const uint8_t in[FC_LINK_STATUS_SIZE], fc_link_status_msg_t *m);

#ifdef __cplusplus
}
#endif
