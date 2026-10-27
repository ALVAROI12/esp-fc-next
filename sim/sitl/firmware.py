"""ctypes binding to the real flight firmware (fc_core's fc_step loop) built as a shared library.

Each FlightFirmware instance loads its own copy of the library file, so several drones in one
process have independent firmware globals (the Crazyflie code is written for a single vehicle).
"""

from __future__ import annotations

import ctypes as C
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading

REPO = Path(__file__).resolve().parents[2]
SOURCE_DIRS = [REPO / "firmware/fc_core", REPO / "firmware/flight/components/core/crazyflie"]
BUILD_DIR = REPO / "build/fc_sitl"
LIB_NAME = "libfc_sitl.dylib" if sys.platform == "darwin" else "libfc_sitl.so"

PHASE_CALIBRATING = 0
PHASE_READY = 1

_build_lock = threading.Lock()
_instance_dir = Path(tempfile.mkdtemp(prefix="fc_sitl_"))
_instance_count = 0


def _newest_source_mtime() -> float:
    newest = 0.0
    for d in SOURCE_DIRS:
        for p in d.rglob("*"):
            if p.suffix in (".c", ".h", ".txt") and "tests" not in p.parts:
                newest = max(newest, p.stat().st_mtime)
    return newest


def library_path(rebuild: bool | None = None) -> Path:
    """Path to the SITL library, (re)building it with CMake when missing or older than the sources."""
    lib = BUILD_DIR / LIB_NAME
    with _build_lock:
        stale = not lib.exists() or lib.stat().st_mtime < _newest_source_mtime()
        if rebuild or (rebuild is None and stale):
            generator = ["-G", "Ninja"] if shutil.which("ninja") else []
            subprocess.run(["cmake", "-S", str(REPO / "firmware/fc_core"), "-B", str(BUILD_DIR), *generator,
                            "-DCMAKE_BUILD_TYPE=Release"], check=True, stdout=subprocess.DEVNULL)
            subprocess.run(["cmake", "--build", str(BUILD_DIR), "--target", "fc_sitl"], check=True,
                           stdout=subprocess.DEVNULL)
    return lib


class FirmwareError(RuntimeError):
    pass


class FlightFirmware:
    """One instance of the flight firmware. Call step() once per 1 ms tick."""

    RATE_HZ = 1000

    def __init__(self, log_level: int = 1):
        global _instance_count
        src = library_path()
        _instance_count += 1
        copy = _instance_dir / f"{src.stem}_{os.getpid()}_{_instance_count}{src.suffix}"
        shutil.copy2(src, copy)
        self._lib = C.CDLL(str(copy))
        self._declare()
        self._lib.fc_host_set_log_level(log_level)
        self._gyro = (C.c_float * 3)()
        self._acc = (C.c_float * 3)()
        self._state = (C.c_float * 13)()
        self.init()

    def _declare(self) -> None:
        f, u16, u32, b = C.c_float, C.c_uint16, C.c_uint32, C.c_bool
        sig = {
            "fc_init": (None, []),
            "fc_arm": (None, [b]),
            "fc_is_armed": (b, []),
            "fc_emergency_stop": (None, []),
            "fc_phase": (C.c_int, []),
            "fc_tick": (u32, []),
            "fc_set_position_setpoint": (None, [f, f, f, f]),
            "fc_push_position": (b, [f, f, f, f]),
            "fc_push_tof": (b, [f, f]),
            "fc_push_flow": (b, [f, f, f, f]),
            "fc_step": (None, [C.POINTER(f), C.POINTER(f)]),
            "fc_get_state_array": (None, [C.POINTER(f)]),
            "fc_host_motor_ratio": (u16, [u32]),
            "fc_host_set_time_us": (None, [C.c_uint64]),
            "fc_host_set_log_level": (None, [C.c_int]),
            "fc_host_assert_count": (u32, []),
            "fc_param_get": (C.c_int, [C.c_char_p, C.c_char_p, C.POINTER(C.c_double)]),
            "fc_param_set": (C.c_int, [C.c_char_p, C.c_char_p, C.c_double]),
            "fc_log_get": (C.c_int, [C.c_char_p, C.c_char_p, C.POINTER(C.c_double)]),
            "fc_vars_list": (C.c_size_t, [C.c_int, C.c_char_p, C.c_size_t]),
        }
        for name, (res, args) in sig.items():
            fn = getattr(self._lib, name)
            fn.restype = res
            fn.argtypes = args

    # ---- lifecycle ----
    def init(self) -> None:
        self._lib.fc_init()

    def arm(self, armed: bool = True) -> None:
        self._lib.fc_arm(armed)

    @property
    def armed(self) -> bool:
        return bool(self._lib.fc_is_armed())

    def emergency_stop(self) -> None:
        self._lib.fc_emergency_stop()

    @property
    def ready(self) -> bool:
        return self._lib.fc_phase() == PHASE_READY

    @property
    def tick(self) -> int:
        return int(self._lib.fc_tick())

    # ---- inputs ----
    def set_position_setpoint(self, x: float, y: float, z: float, yaw_deg: float = 0.0) -> None:
        self._lib.fc_set_position_setpoint(x, y, z, yaw_deg)

    def push_position(self, x: float, y: float, z: float, std: float) -> bool:
        return bool(self._lib.fc_push_position(x, y, z, std))

    def push_tof(self, distance: float, std: float) -> bool:
        return bool(self._lib.fc_push_tof(distance, std))

    def push_flow(self, dpx: float, dpy: float, dt: float, std: float) -> bool:
        return bool(self._lib.fc_push_flow(dpx, dpy, dt, std))

    def step(self, gyro_dps, acc_g, time_us: int) -> tuple[int, int, int, int]:
        """Run one 1 kHz tick; returns the four motor ratios (0..65535)."""
        self._gyro[0], self._gyro[1], self._gyro[2] = gyro_dps
        self._acc[0], self._acc[1], self._acc[2] = acc_g
        self._lib.fc_host_set_time_us(time_us)
        self._lib.fc_step(self._gyro, self._acc)
        m = self._lib.fc_host_motor_ratio
        return m(0), m(1), m(2), m(3)

    # ---- outputs ----
    def state(self) -> list[float]:
        """x y z vx vy vz roll pitch yaw(deg) qw qx qy qz, see fc_step.h."""
        self._lib.fc_get_state_array(self._state)
        return list(self._state)

    @property
    def assert_count(self) -> int:
        return int(self._lib.fc_host_assert_count())

    # ---- firmware variables, same names as cfclient ----
    def param(self, name: str) -> float:
        group, var = name.split(".", 1)
        v = C.c_double()
        if self._lib.fc_param_get(group.encode(), var.encode(), C.byref(v)) != 0:
            raise FirmwareError(f"unknown parameter {name}")
        return v.value

    def set_param(self, name: str, value: float) -> None:
        group, var = name.split(".", 1)
        rc = self._lib.fc_param_set(group.encode(), var.encode(), float(value))
        if rc != 0:
            raise FirmwareError(f"cannot set parameter {name} (code {rc})")

    def log(self, name: str) -> float:
        group, var = name.split(".", 1)
        v = C.c_double()
        if self._lib.fc_log_get(group.encode(), var.encode(), C.byref(v)) != 0:
            raise FirmwareError(f"unknown log variable {name}")
        return v.value

    def variables(self, kind: str = "param") -> list[str]:
        k = 0 if kind == "param" else 1
        n = self._lib.fc_vars_list(k, None, 0)
        buf = C.create_string_buffer(n)
        self._lib.fc_vars_list(k, buf, n)
        return buf.value.decode().split()
