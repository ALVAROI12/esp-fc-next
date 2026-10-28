"""tools/espfc_link.py (PC side of the ground serial link) against the firmware codec in the SITL library."""

import ctypes as C
import random
import sys
from pathlib import Path

import pytest

from sitl.firmware import library_path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import espfc_link as L  # noqa: E402


class Frame(C.Structure):
    _fields_ = [("type", C.c_uint8), ("seq", C.c_uint16), ("len", C.c_uint8), ("payload", C.c_uint8 * 56)]


@pytest.fixture(scope="module")
def lib():
    lib = C.CDLL(str(library_path()))
    lib.fc_link_crc16.restype = C.c_uint16
    lib.fc_link_crc16.argtypes = [C.c_char_p, C.c_size_t]
    lib.fc_link_encode.restype = C.c_bool
    lib.fc_link_decode.argtypes = [C.c_char_p, C.c_size_t, C.POINTER(Frame)]
    return lib


def test_crc_matches_firmware(lib):
    assert L.crc16(b"123456789") == 0x29B1 == lib.fc_link_crc16(b"123456789", 9)
    rng = random.Random(1)
    for _ in range(200):
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 60)))
        assert L.crc16(data) == lib.fc_link_crc16(data, len(data))


def test_frames_round_trip_both_ways(lib):
    rng = random.Random(2)
    for _ in range(400):
        kind = rng.choice(list(L.MAX_LEN))
        payload = bytes(rng.randrange(256) for _ in range(rng.randrange(L.MAX_LEN[kind] + 1)))
        seq = rng.randrange(1 << 16)
        wire = L.encode(kind, payload, seq)
        f = Frame()
        assert lib.fc_link_decode(wire, len(wire), C.byref(f)) == 0  # FC_LINK_OK
        assert (f.type, f.seq, bytes(f.payload[:f.len])) == (kind, seq, payload)
        out = C.create_string_buffer(L.FRAME)
        assert lib.fc_link_encode(C.byref(f), out)
        assert out.raw == wire and L.decode(out.raw) == (kind, seq, payload)
    with pytest.raises(ValueError):
        L.encode(L.CRTP, bytes(32), 0)


def test_python_decoder_rejects_what_the_firmware_rejects(lib):
    good = L.encode(L.CRTP_PEER, b"\x01\x7c\x02", 7)
    rng = random.Random(3)
    for _ in range(500):
        bad = bytearray(good)
        bad[rng.randrange(L.FRAME)] ^= 1 << rng.randrange(8)
        f = Frame()
        assert (L.decode(bytes(bad)) is None) == (lib.fc_link_decode(bytes(bad), L.FRAME, C.byref(f)) != 0)


class FakePort:
    def __init__(self, incoming: bytes = b"") -> None:
        self.incoming, self.written = incoming, b""

    def read(self, n: int) -> bytes:
        data, self.incoming = self.incoming[:n], self.incoming[n:]
        return data

    def write(self, data: bytes) -> None:
        self.written += data


def test_ground_link_addresses_drones_and_skips_console_text():
    status = bytes(range(20))
    wire = (b"W (812) radio: \xa5 console line\n" + L.encode(L.CRTP_PEER, b"\x03\x5c\x01\x02", 1) +
            L.encode(L.CRTP_PEER, b"\x04\x5c", 2)[:40] +  # truncated in transit
            L.encode(L.SWARM_STATUS, status, 3) + L.encode(L.IDLE, b"", 4))
    link = L.GroundLink(FakePort(wire))
    assert link.receive() == [("crtp", 3, b"\x5c\x01\x02"), ("status", None, status)]
    link.send_crtp(2, b"\x7c\x05")
    link.send_crtp(L.ALL, b"\x7c\x06")
    assert [L.decode(link.port.written[i:i + L.FRAME]) for i in (0, L.FRAME)] == [
        (L.CRTP_PEER, 0, b"\x02\x7c\x05"), (L.CRTP_PEER, 1, b"\xff\x7c\x06")]
    for dest, pk in ((0, b"\x7c"), (256, b"\x7c"), (1, b""), (1, bytes(32))):
        with pytest.raises(ValueError):
            link.send_crtp(dest, pk)
