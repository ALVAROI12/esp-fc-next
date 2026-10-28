"""tools/espfc_cflib.py: the cflib driver over a pyserial loop:// port standing in for the ground C5.

loop:// echoes what the driver writes. A CRTP_PEER frame sent to drone N comes back tagged as from drone N,
which exercises encode -> resync -> route-by-sender without hardware.
"""

import sys
import threading
from pathlib import Path

import pytest

pytest.importorskip("cflib")
pytest.importorskip("serial")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import cflib.crtp  # noqa: E402
from cflib.crtp.crtpstack import CRTPPacket  # noqa: E402

import espfc_cflib  # noqa: E402

DEV = "loop://"


@pytest.fixture(autouse=True)
def only_espfc(monkeypatch):
    monkeypatch.setattr(cflib.crtp, "CLASSES", [])
    monkeypatch.setattr(espfc_cflib, "CLASSES", cflib.crtp.CLASSES)
    espfc_cflib.register()
    espfc_cflib.register()  # idempotent
    assert cflib.crtp.CLASSES == [espfc_cflib.EspfcDriver]
    yield
    assert espfc_cflib._hubs == {}, "a test left a ground port open"


def connect(uri, errors=None):
    d = cflib.crtp.get_link_driver(uri, None, errors.append if errors is not None else None)
    assert isinstance(d, espfc_cflib.EspfcDriver)
    return d


def test_telemetry_is_routed_to_the_link_of_its_sender():
    d3, d4 = connect(f"espfc://3@{DEV}"), connect(f"espfc://4@{DEV}")
    assert d3.hub is d4.hub  # one serial port, one reader
    d3.send_packet(CRTPPacket(0x30, b"ab"))  # param port; the loop answers "from drone 3"
    pk = d3.receive_packet(1)
    assert (pk.port, pk.channel, bytes(pk.data)) == (3, 0, b"ab")
    assert d4.receive_packet(0.1) is None
    d4.send_packet(CRTPPacket(0x5C, bytes(range(30))))  # largest CRTP payload
    assert bytes(d4.receive_packet(1).data) == bytes(range(30))
    assert d3.receive_packet(0) is None
    d3.close()
    d3.close()  # second close is harmless
    d4.close()


def test_broadcast_link_sends_to_all_and_receives_nothing():
    d3, everyone = connect(f"espfc://3@{DEV}"), connect(f"espfc://all@{DEV}")
    assert everyone.id == 0xFF
    everyone.send_packet(CRTPPacket(0x70, b"\x01"))  # echo comes back "from 255": no link owns it
    assert d3.receive_packet(0.2) is None and everyone.receive_packet(0) is None
    everyone.close()
    assert espfc_cflib._hubs  # drone 3 still holds the port
    d3.close()


def test_uri_checks():
    assert cflib.crtp.get_link_driver("radio://0/80/2M", None, None) is None  # not ours: next driver's turn
    for bad in (f"espfc://0@{DEV}", f"espfc://300@{DEV}"):
        with pytest.raises(Exception, match="1-254"):
            cflib.crtp.get_link_driver(bad, None, None)
    d = connect(f"espfc://5@{DEV}")
    with pytest.raises(Exception, match="already connected"):
        cflib.crtp.get_link_driver(f"espfc://5@{DEV}", None, None)
    d.close()
    connect(f"espfc://5@{DEV}").close()  # the port reopens after the last link closed


def test_losing_the_ground_radio_reports_a_link_error():
    errors = []
    d = connect(f"espfc://7@{DEV}", errors)
    done = threading.Event()

    def on_error(msg):  # what cflib does: report, then close the link from the reader thread
        errors.append(msg)
        d.close()
        done.set()
    d.hub.errors[7] = on_error
    d.hub.port.close()  # unplugged
    assert done.wait(2) and "espfc ground radio" in errors[0]
    assert d.hub is None
