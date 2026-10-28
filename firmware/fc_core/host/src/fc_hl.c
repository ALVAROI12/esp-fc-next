/*
 * SITL only: the Crazyflie high-level commander (takeoff / goTo / land / trajectories, as Crazyswarm2 sends
 * them on CRTP port 8) compiled in place from the subtree. Packets go straight to its command handler
 * instead of through its task, and its setpoints reach fc_step the way target/fc_runtime.c forwards them.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "crtp_commander_high_level.c" // NOLINT(bugprone-suspicious-include): reaches its static handlers

#include "fc_step.h"

/* Target modules the high-level commander touches; the SITL has no memory port or CRTP stack. */
void memoryRegisterHandler(const MemoryHandlerDef_t *handlerDef) { (void)handlerDef; }
void crtpInitTaskQueue(CRTPPort taskId) { (void)taskId; }
int crtpReceivePacketBlock(CRTPPort taskId, CRTPPacket *p) { (void)taskId; (void)p; return -1; }
int crtpSendPacket(CRTPPacket *p) { (void)p; return 0; }

/* One CRTP port-8 payload (command byte + struct); returns the handler's errno-style result. */
int fc_hl_packet(const uint8_t *data, uint8_t size)
{
    crtpCommanderHighLevelInit();
    if (size < 1 || size > CRTP_MAX_DATA_SIZE) return EINVAL;
    CRTPPacket p = {0};
    memcpy(p.data, data, size);
    return handleCommand(p.data[0], &p.data[1]);
}

/* Call once per tick: the planner's point while it flies (fc_runtime forwards the same mode.z == modeAbs
 * points after the commander's 2 s packet timeout). Returns whether a setpoint was sent. */
bool fc_hl_step(void)
{
    crtpCommanderHighLevelInit();
    setpoint_t sp = {0};
    crtpCommanderHighLevelGetSetpoint(&sp, fc_state());
    if (sp.mode.z != modeAbs) return false;
    fc_set_setpoint(&sp);
    return true;
}
