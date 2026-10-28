/* fc_link wire formats; see include/fc_link.h and docs/protocols.md. SPDX-License-Identifier: GPL-3.0-or-later */
#include "fc_link.h"

#include <math.h>
#include <string.h>

static const uint8_t maxLen[FC_LINK_TYPE_COUNT] = {
    [FC_LINK_IDLE] = 0,
    [FC_LINK_CRTP] = 31, /* CRTP raw: header + CRTP_MAX_DATA_SIZE */
    [FC_LINK_SWARM_STATUS] = FC_LINK_STATUS_SIZE,
    [FC_LINK_CRTP_PEER] = 32, /* peer id + CRTP raw */
};

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

uint16_t fc_link_crc16(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

bool fc_link_encode(const fc_link_frame_t *f, uint8_t out[FC_LINK_FRAME_SIZE])
{
    if (f->type >= FC_LINK_TYPE_COUNT || f->len > maxLen[f->type]) return false;
    memset(out, 0, FC_LINK_FRAME_SIZE);
    out[0] = FC_LINK_MAGIC;
    out[1] = f->type;
    put16(out + 2, f->seq);
    out[4] = f->len;
    memcpy(out + FC_LINK_HEADER_SIZE, f->payload, f->len);
    put16(out + FC_LINK_HEADER_SIZE + f->len, fc_link_crc16(out, FC_LINK_HEADER_SIZE + f->len));
    return true;
}

fc_link_status_t fc_link_decode(const uint8_t *buf, size_t n, fc_link_frame_t *out)
{
    if (n < FC_LINK_HEADER_SIZE + FC_LINK_CRC_SIZE) return FC_LINK_ERR_SHORT;
    if (buf[0] != FC_LINK_MAGIC) return FC_LINK_ERR_MAGIC;
    if (buf[1] >= FC_LINK_TYPE_COUNT) return FC_LINK_ERR_TYPE;
    uint8_t len = buf[4];
    if (len > maxLen[buf[1]]) return FC_LINK_ERR_LEN;
    if (n < FC_LINK_HEADER_SIZE + len + FC_LINK_CRC_SIZE) return FC_LINK_ERR_SHORT;
    if (get16(buf + FC_LINK_HEADER_SIZE + len) != fc_link_crc16(buf, FC_LINK_HEADER_SIZE + len)) return FC_LINK_ERR_CRC;
    out->type = buf[1];
    out->seq = get16(buf + 2);
    out->len = len;
    memcpy(out->payload, buf + FC_LINK_HEADER_SIZE, len);
    return FC_LINK_OK;
}

bool fc_link_replay_accept(fc_link_replay_t *w, uint32_t counter)
{
    if (!w->started) {
        *w = (fc_link_replay_t){.started = true, .highest = counter, .bitmap = 1};
        return true;
    }
    if (counter > w->highest) {
        uint32_t shift = counter - w->highest;
        w->bitmap = shift >= 32 ? 1 : (w->bitmap << shift) | 1;
        w->highest = counter;
        return true;
    }
    uint32_t age = w->highest - counter;
    if (age >= 32 || (w->bitmap & (1u << age))) return false;
    w->bitmap |= 1u << age;
    return true;
}

static bool to_i16(float v, float scale, int16_t *out)
{
    float s = v * scale;
    if (!isfinite(s) || s < -32768.0f || s > 32767.0f) return false;
    *out = (int16_t)lrintf(s);
    return true;
}

bool fc_link_status_pack(const fc_link_status_msg_t *m, uint8_t out[FC_LINK_STATUS_SIZE])
{
    int16_t q[6];
    for (int i = 0; i < 3; i++) {
        if (!to_i16(m->pos[i], 1000.0f, &q[i]) || !to_i16(m->vel[i], 1000.0f, &q[3 + i])) return false;
    }
    out[0] = m->id;
    out[1] = m->state;
    for (int i = 0; i < 6; i++) put16(out + 2 + 2 * i, (uint16_t)q[i]);
    put16(out + 14, m->battery_mv);
    put32(out + 16, m->time_ms);
    return true;
}

void fc_link_status_unpack(const uint8_t in[FC_LINK_STATUS_SIZE], fc_link_status_msg_t *m)
{
    m->id = in[0];
    m->state = in[1];
    for (int i = 0; i < 3; i++) {
        m->pos[i] = (int16_t)get16(in + 2 + 2 * i) / 1000.0f;
        m->vel[i] = (int16_t)get16(in + 8 + 2 * i) / 1000.0f;
    }
    m->battery_mv = get16(in + 14);
    m->time_ms = get32(in + 16);
}
