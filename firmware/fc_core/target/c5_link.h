/*
 * c5_link: CRTP transport over the P4 <-> C5 SPI frame (docs/protocols.md sections 1 and 5). The P4 is SPI
 * master and polls the C5 at a fixed rate; each transaction carries one frame each way.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "crtp.h"
#include "fc_link.h"

void c5LinkInit(void); /* queues; on target also the SPI bus and the polling task */
bool c5LinkTest(void);
struct crtpLinkOperations *c5LinkGetLink(void);

/* One transaction's work: consume the frame the C5 sent, then fill the next frame to send (IDLE when no
 * CRTP packet is queued). The polling task calls it; exposed for host tests. */
void c5LinkExchange(const uint8_t rx[FC_LINK_FRAME_SIZE], uint8_t tx[FC_LINK_FRAME_SIZE]);

typedef struct {
    uint32_t rxFrames, rxErrors, rxGaps, rxDropped, txCrtp;
} c5LinkStats_t;
c5LinkStats_t c5LinkGetStats(void);
