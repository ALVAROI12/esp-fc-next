"""Software-in-the-loop: the real flight firmware (fc_core) flying the 6-DOF model (gate G2)."""

from .drone import SitlDrone
from .firmware import FlightFirmware

__all__ = ["FlightFirmware", "SitlDrone"]
