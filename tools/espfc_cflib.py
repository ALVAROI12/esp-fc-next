"""cflib link driver for ESP-FC Next drones through the ground C5 (docs/protocols.md section 5).

URI: espfc://<id>@<serial device>, e.g. espfc://3@/dev/cu.usbmodem1101; id 1-254, or "all" for a broadcast link
(Crazyswarm2 takeoff/land to every drone; it receives nothing). Every URI on the same device shares one serial
port and one reader thread, which routes drone telemetry to the right link by sender id.

Use: cflib.crtp.init_drivers(); espfc_cflib.register(). In Crazyswarm2's cflib backend, call register() after
its init_drivers() and set uri: espfc://<id>@<device> in crazyflies.yaml.
"""
import logging
import queue
import re
import threading

import serial
from cflib.crtp import CLASSES
from cflib.crtp.crtpdriver import CRTPDriver
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.exceptions import WrongUriType

from espfc_link import ALL, GroundLink

logger = logging.getLogger(__name__)
URI = re.compile(r"^espfc://(all|\d{1,3})@(.+)$")


class _Hub:
    """One ground C5 serial port shared by every drone link on it."""

    def __init__(self, device: str) -> None:
        self.device = device
        self.port = serial.serial_for_url(device, baudrate=921600, timeout=0.01)  # USB CDC ignores the baud
        self.link = GroundLink(self.port)
        self.queues: dict[int, queue.Queue] = {}
        self.errors: dict[int, object] = {}
        self.users = 0
        self.write_lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._run, name=f"espfc {device}", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            try:
                events = self.link.receive()
            except Exception as e:  # unplugged ground radio: every link on it is gone
                if self.running:
                    for cb in list(self.errors.values()):
                        if cb:
                            cb(f"espfc ground radio {self.device}: {e}")
                return
            for kind, sender, data in events:
                q = self.queues.get(sender) if kind == "crtp" else None
                if q is not None:
                    q.put(CRTPPacket(data[0], data[1:]))

    def send(self, dest: int, raw: bytes) -> None:
        with self.write_lock:
            self.link.send_crtp(dest, raw)


_hubs: dict[str, _Hub] = {}
_hubs_lock = threading.Lock()


class EspfcDriver(CRTPDriver):

    def __init__(self) -> None:
        super().__init__()
        self.needs_resending = True  # ESP-NOW broadcast is unacknowledged: let cflib retry requests
        self.hub = None
        self.id = None
        self.in_queue = queue.Queue()

    def connect(self, uri, radio_link_statistics_callback, link_error_callback):
        m = URI.match(uri or "")
        if not m:
            raise WrongUriType("Not an espfc URI")
        self.id = ALL if m.group(1) == "all" else int(m.group(1))
        if not 1 <= self.id <= ALL:
            raise Exception(f"espfc drone id must be 1-254 or all: {uri}")
        with _hubs_lock:
            hub = _hubs.get(m.group(2)) or _Hub(m.group(2))
            _hubs[hub.device] = hub
            if self.id != ALL:
                if self.id in hub.queues:
                    raise Exception(f"espfc drone {self.id} is already connected on {hub.device}")
                hub.queues[self.id] = self.in_queue
                hub.errors[self.id] = link_error_callback
            hub.users += 1
        self.hub = hub

    def send_packet(self, pk):
        self.hub.send(self.id, bytes([pk.header]) + bytes(pk.data))

    def receive_packet(self, wait=0):
        try:
            if wait == 0:
                return self.in_queue.get(False)
            return self.in_queue.get(True, None if wait < 0 else wait)
        except queue.Empty:
            return None

    def get_status(self):
        return "ESP-FC Next ground radio"

    def get_name(self):
        return "espfc"

    def scan_interface(self, address=None):
        return []  # drones answer only when addressed: list them in the URI

    def get_help(self):
        return "espfc://<id>@<serial device>, id 1-254 or all"

    def close(self):
        hub, self.hub = self.hub, None
        if hub is None:
            return
        with _hubs_lock:
            hub.queues.pop(self.id, None)
            hub.errors.pop(self.id, None)
            hub.users -= 1
            if hub.users == 0:
                hub.running = False
                del _hubs[hub.device]
        if not hub.running:
            if threading.current_thread() is not hub.thread:  # cflib may close from the error callback
                hub.thread.join(timeout=1)
            hub.port.close()


def register() -> None:
    """Add the driver to cflib (after cflib.crtp.init_drivers())."""
    if EspfcDriver not in CLASSES:
        CLASSES.append(EspfcDriver)
