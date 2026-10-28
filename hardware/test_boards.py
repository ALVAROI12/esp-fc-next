"""Scripted netlist checks of boards.py (gate G4) plus the ngspice power-path check."""
import re
from pathlib import Path

import boards
import power_sim
import pytest
from skidl.logger import erc_logger

ROOT = Path(__file__).resolve().parents[1]
FC, RADIO = boards.build("fc_board"), boards.build("radio_board")
SI = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3, "k": 1e3, "M": 1e6}


def value(part):
    """Ohms or farads from a value like 511k, 22p, 10m."""
    v = re.match(r"([\d.]+)([pnumkM]?)", part.value)
    return float(v[1]) * SI.get(v[2], 1)


def chip(circuit, ref="U1"):
    (u,) = [p for p in circuit.parts if p.ref == ref]
    return u


def net(circuit, name):
    (n,) = [n for n in circuit.get_nets() if n.name == name]
    return n


def two_pin(circuit, kind, a, b):
    """Resistors or capacitors ("R"/"C") between nets a and b."""
    return [p for p in circuit.parts if p.name == kind and {p[1].net.name, p[2].net.name} == {a, b}]


def kconfig_defaults(path):
    return {k: int(v) for k, v in re.findall(r"config (\w+)\n.*\n\s+default (\d+)", (ROOT / path).read_text())}


@pytest.mark.parametrize("circuit", [FC, RADIO], ids=["fc", "radio"])
def test_erc_is_clean(circuit):
    erc_logger.error.reset()
    erc_logger.warning.reset()
    circuit.ERC()
    assert (erc_logger.error.count, erc_logger.warning.count) == (0, 0)


def test_p4_supplies():
    p4 = chip(FC)
    rails = {"3V3": ["VDD_LDO", "VDD_DCDCC", "VDD_ANA", "VDD_LP", "VDD_BAT", "VDD_FLASH_IO", "VDD_IO_0", "VDD_IO_4",
                     "VDD_IO_5", "VDD_IO_6"],
             "VDD_HP": ["VDD_HP_0", "VDD_HP_1", "VDD_HP_2", "VDD_HP_3"],
             "VDD_PSRAM": ["VDDO_PSRAM", "VDD_PSRAM_0", "VDD_PSRAM_1"], "GND": ["GND"]}
    for rail, pins in rails.items():
        for pin in pins:
            assert p4[pin].net.name == rail, pin
    for pin in ("VDD_HP_0", "VDD_IO_0", "VDD_LP", "VDD_BAT"):  # 0.1 uF per pin
        assert len([c for c in two_pin(FC, "C", p4[pin].net.name, "GND") if c.value == "100n"]) >= 1


def test_core_buck_feedback_is_populated_and_lands_in_the_hp_range():
    p4 = chip(FC)
    fb = p4["FB_DCDC"].net
    assert fb.name == "HP_FB"
    assert {p.part.name for p in fb.pins} >= {"TLV62569DBV", "ESP32-P4X"}
    (top,), (bottom,) = two_pin(FC, "R", "VDD_HP", "HP_FB"), two_pin(FC, "R", "HP_FB", "GND")
    assert top.value == bottom.value == "499k"
    assert [c.value for c in two_pin(FC, "C", "VDD_HP", "HP_FB")] == ["22p"]
    assert 0.99 <= 0.6 * (1 + value(top) / value(bottom)) <= 1.3  # TLV62569 VFB = 0.6 V
    buck_en = [p for p in p4["EN_DCDC"].net.pins if p.part.name == "TLV62569DBV"]
    assert [p.name for p in buck_en] == ["EN"]


def test_3v3_buck_boost_divider():
    (top,), (bottom,) = two_pin(FC, "R", "3V3", "BB_FB"), two_pin(FC, "R", "BB_FB", "GND")
    assert value(bottom) <= 100e3  # SLVSEU9D: R2 <= 100 kOhm
    assert 0.5 * (1 + value(top) / value(bottom)) == pytest.approx(3.3, rel=0.02)


def test_straps_reset_and_flash_cs():
    p4 = chip(FC)
    for n in (35, 36):
        assert [r.value for r in two_pin(FC, "R", "3V3", boards.gpio(p4, n).net.name)] == ["10k"]
    assert two_pin(FC, "R", boards.gpio(p4, 34).net.name, "GND")  # GPIO34 has no internal pull
    assert "SW_Push" in {p.part.name for p in boards.gpio(p4, 35).net.pins}
    assert [r.value for r in two_pin(FC, "R", "3V3", "CHIP_PU")] == ["10k"]
    assert [c.value for c in two_pin(FC, "C", "CHIP_PU", "GND")] == ["1u"]
    assert two_pin(FC, "R", "3V3", "FLASH_CS")


def test_imu_uses_the_spi2_io_mux_pins():
    p4 = chip(FC)
    for sig, n in (("IMU_CS", 7), ("IMU_MOSI", 8), ("IMU_SCLK", 9), ("IMU_MISO", 10)):
        assert boards.gpio(p4, n).net.name == sig
        assert "ICM-42688-P" in {p.part.name for p in boards.gpio(p4, n).net.pins}


def test_firmware_pin_defaults_match_the_boards():
    flight = kconfig_defaults("firmware/flight/main/Kconfig.projbuild")
    radio = kconfig_defaults("firmware/radio/main/Kconfig.projbuild")
    p4, c5 = chip(FC), chip(RADIO)
    for sig in ("MOSI", "MISO", "SCLK", "CS"):
        assert boards.gpio(p4, flight[f"ESPFC_C5_SPI_{sig}"]).net.name == f"C5_{sig}"
        assert boards.gpio(c5, radio[f"ESPFC_RADIO_SPI_{sig}"]).net.name == f"C5_{sig}"


def test_stack_connector_carries_the_same_signal_on_both_boards():
    for circuit in (FC, RADIO):
        j = chip(circuit, "J_STACK")
        assert [j[i].net.name for i in range(1, 13)] == boards.STACK
    p4, c5 = chip(FC), chip(RADIO)
    assert boards.gpio(p4, boards.P4["P4_TX"]).net.name == boards.gpio(c5, 12).net.name == "P4_TX"  # C5 U0RXD
    assert boards.gpio(p4, boards.P4["P4_RX"]).net.name == boards.gpio(c5, 11).net.name == "P4_RX"  # C5 U0TXD
    assert c5["EN/CHIP_PU"].net.name == "C5_EN" and boards.gpio(c5, 28).net.name == "C5_BOOT"


def test_c5_link_avoids_strapping_pins():
    assert not set(boards.C5.values()) & {2, 3, 7, 25, 26, 27, 28}


def test_power_path_holds_the_3v3_converter_input_during_a_motor_spike():
    # End of flight (3.5 V open circuit), 50 mOhm pack, 16 A from four motors, 1 A on 3.3 V.
    vsys, _ = power_sim.simulate(ocv=3.5, rint=0.05, i_motor=16.0)
    assert vsys >= power_sim.VIN_FULL_LOAD
    (shunt,) = two_pin(FC, "R", "VBAT", "VBAT_SHUNT")
    assert value(shunt) <= 0.01  # what the simulation assumed
