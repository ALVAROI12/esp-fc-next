/* libFuzzer target for fc_link (Task 1.7 "1 h fuzzing without crashes"). Build: make link-fuzz.
 * Any input must decode without memory errors; a decoded frame must re-encode to the same bytes;
 * the replay window must never accept a counter twice. SPDX-License-Identifier: GPL-3.0-or-later */
#include <stdlib.h>
#include <string.h>

#include "fc_link.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    fc_link_frame_t f;
    if (fc_link_decode(data, size, &f) == FC_LINK_OK) {
        uint8_t out[FC_LINK_FRAME_SIZE];
        size_t used = FC_LINK_HEADER_SIZE + f.len + FC_LINK_CRC_SIZE;
        if (!fc_link_encode(&f, out) || memcmp(out, data, used) != 0) abort();
        if (f.type == FC_LINK_SWARM_STATUS && f.len == FC_LINK_STATUS_SIZE) {
            fc_link_status_msg_t m;
            fc_link_status_unpack(f.payload, &m);
            uint8_t again[FC_LINK_STATUS_SIZE];
            if (!fc_link_status_pack(&m, again) || memcmp(again, f.payload, sizeof again) != 0) abort();
        }
    }

    /* Treat the input as a stream of le32 counters; track the last 64 accepted to catch double accepts. */
    fc_link_replay_t w = {0};
    uint32_t seen[64];
    size_t nseen = 0, head = 0;
    for (size_t i = 0; i + 4 <= size; i += 4) {
        uint32_t c = data[i] | (uint32_t)data[i + 1] << 8 | (uint32_t)data[i + 2] << 16 | (uint32_t)data[i + 3] << 24;
        if (!fc_link_replay_accept(&w, c)) continue;
        for (size_t k = 0; k < nseen; k++) {
            if (seen[k] == c) abort();
        }
        seen[head] = c;
        head = (head + 1) % 64;
        if (nseen < 64) nseen++;
    }
    return 0;
}
