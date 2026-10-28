# ESP-FC Next link protocols (Task 1.7)

Code: `firmware/fc_core/include/fc_link.h`, `firmware/fc_core/src/fc_link.c` (portable C, built on host,
P4 and C5). Radio envelope: `fc_link_aead.h/.c` (section 3). C5 bridge: `fc_radio.h/.c` (section 5). Tests: `tests/test_link.c`. Fuzzing: `make link-fuzz` (libFuzzer + ASan/UBSan; 1 h: 4.32 B inputs, no crash) and `make link-fuzz FUZZ_TARGET=fuzz_link_aead` (arbitrary bytes never open; sealed input opens once, bit flips rejected; 1 h: 83.5 M inputs, no crash).

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

- Link-loss timing on hardware. Both SPI ends build but have not run on a board.
- ESP-Drone's `crtpRxTask` asserts (motor stop) if a CRTP port queue overflows. That is upstream behaviour for
  every link; a ground station flooding one port can trigger it.
- A provisioning tool (the NVS layout is defined in section 5; `nvs_partition_gen.py` can write it).
- Crazyswarm2 bridge (CRTP over UDP/ESP-NOW to the ground station).
- Group key: every member can seal any message. CRTP is accepted only from sender id 0 (ground), so a buggy
  drone cannot command another, but a compromised drone can impersonate the ground. Per-sender keys fix that.

## 5. C5 bridge (`fc_radio`)

Code: `include/fc_radio.h`, `src/fc_radio.c` (portable, host-tested in `tests/test_radio.c`), wired in
`firmware/radio/main/main.c`.

- Air to P4: `fc_radio_from_air` opens the envelope, then encodes the plaintext (type byte + payload) as one SPI
  frame with the C5's own per-direction `seq`. It drops: failed authentication or replay, IDLE or unknown types,
  oversize payloads, and CRTP from any sender other than id 0 (ground).
- P4 to air: `fc_radio_to_air` decodes one SPI frame and seals non-IDLE frames as a broadcast with the C5's id.
  Corrupt frames are dropped and counted.
- Counter persistence: counters are reserved in blocks of 4096 and stored in NVS before use. A reboot resumes at
  the stored limit, so it never reuses a nonce. Sending stalls instead of overrunning the reservation. At 50 Hz a
  block lasts 80 s, about 1100 NVS writes per day. When the 32-bit space is spent it stops sending (rekey).
- Transport: the P4 is SPI master and polls; each transaction carries one frame each way, IDLE when there is
  nothing to deliver. ESP-NOW receives go through a 16-deep queue (drop when full); all envelope work runs in
  one task. SPI pins and the Wi-Fi channel are Kconfig placeholders until the board exists.
- P4 end: `firmware/fc_core/target/c5_link.c` is ESP-Drone's CRTP transport for `CONFIG_TRANSPORT_C5_SPI`
  (`make flight-c5link`). A 500 Hz SPI-master task calls `c5LinkExchange` per transaction. That step delivers
  CRTP frames to the CRTP stack (16-deep queue, drop when full) and ignores SWARM_STATUS for now. It counts bad
  frames (no C5, CRC) and `seq` gaps, and sends one queued CRTP packet or IDLE. `isConnected` means a CRTP
  packet arrived within 1 s; C5 IDLE frames alone do not count. Host tests: `tests/test_c5_link.c`.
- Provisioning: NVS namespace `fclink`: `key` (16-byte blob), `net` (u32), `id` (u8), `ctr` (u32, written by
  the firmware). There is no default key. Without one the radio stays off; the P4 then sees no valid frames and its
  command watchdog lands.
