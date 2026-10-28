"""PC side of the ground C5 serial link: 64-byte frames (docs/protocols.md sections 1 and 5).

The ground C5 (radio id 0) streams frames over USB Serial/JTAG. To reach drones, send CRTP_PEER frames
(dest id + raw CRTP, dest 255 = every drone). Drone telemetry comes back as CRTP_PEER (sender id + CRTP), and
swarm status as SWARM_STATUS. Console text on the same port is skipped by the resynchronizing reader.
"""
import binascii
import struct

MAGIC = 0xA5
FRAME = 64
IDLE, CRTP, SWARM_STATUS, CRTP_PEER = range(4)
MAX_LEN = {IDLE: 0, CRTP: 31, SWARM_STATUS: 20, CRTP_PEER: 32}
ALL = 0xFF


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)  # CRC-16/CCITT-FALSE


def encode(kind: int, payload: bytes, seq: int) -> bytes:
    if kind not in MAX_LEN or len(payload) > MAX_LEN[kind]:
        raise ValueError(f"bad frame type {kind} or length {len(payload)}")
    head = bytes([MAGIC, kind]) + struct.pack("<HB", seq & 0xFFFF, len(payload)) + payload
    return (head + struct.pack("<H", crc16(head))).ljust(FRAME, b"\0")


def decode(buf: bytes) -> tuple[int, int, bytes] | None:
    """(type, seq, payload), or None for anything the firmware decoder would reject."""
    if len(buf) < FRAME or buf[0] != MAGIC or buf[1] not in MAX_LEN or buf[4] > MAX_LEN[buf[1]]:
        return None
    n = buf[4]
    if struct.unpack_from("<H", buf, 5 + n)[0] != crc16(bytes(buf[:5 + n])) or any(buf[7 + n:FRAME]):
        return None  # bad CRC, or a non-zero tail (a frame that lost tail bytes in the stream)
    return buf[1], struct.unpack_from("<H", buf, 2)[0], bytes(buf[5:5 + n])


class Stream:
    """Resynchronizing reader: slides one byte at a time until magic and CRC check out (as fc_link_stream)."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.skipped = 0

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self.buf += data
        out = []
        while len(self.buf) >= FRAME:
            f = decode(self.buf[:FRAME])
            if f:
                out.append(f)
                del self.buf[:FRAME]
            else:
                del self.buf[0]
                self.skipped += 1
        return out


class GroundLink:
    """CRTP to and from drones through the ground C5. port: anything with read(n) and write(bytes) (pyserial)."""

    def __init__(self, port) -> None:
        self.port = port
        self.seq = 0
        self.stream = Stream()

    def send_crtp(self, dest: int, packet: bytes) -> None:
        if not 1 <= dest <= ALL or not 1 <= len(packet) <= MAX_LEN[CRTP]:
            raise ValueError("dest must be a drone id 1-254 or ALL; packet 1-31 bytes (header + data)")
        self.port.write(encode(CRTP_PEER, bytes([dest]) + packet, self.seq))
        self.seq += 1

    def receive(self, n: int = 4096) -> list[tuple[str, int | None, bytes]]:
        """("crtp", drone id, packet) and ("status", None, 20-byte status) events from what is buffered."""
        events = []
        for kind, _, payload in self.stream.feed(self.port.read(n)):
            if kind == CRTP_PEER and len(payload) >= 2:
                events.append(("crtp", payload[0], payload[1:]))
            elif kind == SWARM_STATUS:
                events.append(("status", None, payload))
        return events
