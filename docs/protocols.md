# ESP-FC Next link protocols (Task 1.7)

Code: `firmware/fc_core/include/fc_link.h`, `firmware/fc_core/src/fc_link.c` (portable C, built on host,
P4 and C5). Radio envelope: `fc_link_aead.h/.c` (section 3). Tests: `tests/test_link.c`. Fuzzing: `make link-fuzz` (libFuzzer + ASan/UBSan).

All multi-byte fields are little-endian. No packed structs cross the wire; every field is written byte by byte.

## 1. P4 <-> C5 SPI frame

Fixed 64-byte SPI transaction. The P4 is master and clocks one frame each way per transaction.

| Offset | Size | Field | Notes |
| --- | --- | --- | --- |
| 0 | 1 | magic | `0xA5` |
| 1 | 1 | type | 0 IDLE, 1 CRTP, 2 SWARM_STATUS |
| 2 | 2 | seq | per-direction counter, wraps; gaps = lost frames (diagnostic only) |
| 4 | 1 | len | IDLE 0, CRTP <= 31, SWARM_STATUS 20 |
| 5 | len | payload | CRTP: raw CRTP packet (header byte + up to 30 data bytes), unchanged for cflib/Crazyswarm2 |
| 5+len | 2 | crc | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 0..4+len |
| ... | | zero | tail is zero |

The decoder rejects short buffers, wrong magic, unknown type, a length above the type's maximum, and a CRC
mismatch, in that order. A rejected frame is dropped and counted; it never reaches the flight loop. CRC-16
detects all single-bit and odd-count errors and all bursts up to 16 bits. The chip-to-chip link is not
authenticated: both chips are on the same board.

## 2. Swarm status (50 Hz broadcast, 20 bytes)

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | drone id |
| 1 | 1 | flight / failsafe state (`fc_failsafe_state_t`) |
| 2 | 6 | position x, y, z: int16 mm (+/-32.767 m) |
| 8 | 6 | velocity x, y, z: int16 mm/s |
| 14 | 2 | battery mV |
| 16 | 4 | sender time ms |

`fc_link_status_pack` refuses NaN/Inf or out-of-range values instead of clamping. A drone that cannot
report a valid position must not broadcast a plausible one.

## 3. Radio envelope (ESP-NOW, AES-CCM)

ESP-NOW's built-in encryption covers only unicast peers and a small peer table. It does not cover
broadcast, so the swarm status and swarm commands use an application-level envelope:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | version (1) |
| 1 | 1 | sender id |
| 2 | 4 | message counter (strictly increasing per sender) |
| 6 | n | AES-128-CCM ciphertext of one SPI frame payload type byte + payload |
| 6+n | 8 | CCM tag |

- Nonce (13 bytes): network id (4) + sender id (1) + counter (4) + zero (4). A nonce never repeats under
  one key while the counter is strictly increasing. The counter must be persisted (or a fresh key used) across reboots.
- AAD: bytes 0..5.
- Receiver order: tag check first, then `fc_link_replay_accept` on that sender's window. A forged counter
  must never advance the window. The window accepts reordering up to 32 messages back and accepts each counter once.
- Rekey before the 32-bit counter wraps (2.7 years at 50 Hz).
- Code: `include/fc_link_aead.h`, `src/fc_link_aead.c`. The key is set once in `fc_link_aead_init`
  (mbedtls allocates there); seal/open allocate nothing per packet. Replay windows (256 senders, ~3 KB) live
  in the context.
- Tests: `tests/test_link_aead.c` runs against the mbedtls that ESP-IDF 5.5 bundles (3.6.7). `make mbedtls-src`
  copies it out of the pinned Docker image. The suite checks:
  - RFC 3610 vector #1, which uses the same 13-byte nonce / 8-byte tag;
  - the wire layout rebuilt byte-by-byte from this section;
  - every single-bit flip, truncation, wrong key/network, replay, size limits.
  The C5 firmware builds it against IDF's `mbedtls` component, and a boot self-check runs seal/open/replay.
  That self-check has not been run on a board.

## 4. Not covered yet

- ESP-NOW / Wi-Fi transport code on the C5, SPI driver code on the P4/C5, link-loss timing on hardware.
- Key provisioning and counter persistence.
- Crazyswarm2 bridge (CRTP over UDP/ESP-NOW to the ground station).
