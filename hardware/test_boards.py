"""Scripted netlist checks of boards.py (gate G4) plus the ngspice power-path check."""
import csv
import json
import re
import shutil
import subprocess
from pathlib import Path

import boards
import library
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


KICAD_FP = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints")
LOCAL_FP = {"ESPFC": library.OUT, "PCM_Espressif": Path(boards.HERE, "lib/espressif/Espressif.pretty")}


def pads(footprint):
    """Pad numbers of a "Library:Name" footprint (KiCad's libraries or ours, as fp-lib-table maps them)."""
    lib, name = footprint.split(":")
    text = (LOCAL_FP.get(lib, KICAD_FP / f"{lib}.pretty") / f"{name}.kicad_mod").read_text()
    return set(re.findall(r'\(pad "([^"]*)"', text)) - {""}


def kconfig_defaults(path):
    return {k: int(v) for k, v in re.findall(r"config (\w+)\n.*\n\s+default (\d+)", (ROOT / path).read_text())}


@pytest.mark.parametrize("circuit", [FC, RADIO], ids=["fc", "radio"])
def test_erc_is_clean(circuit):
    erc_logger.error.reset()
    erc_logger.warning.reset()
    circuit.ERC()
    assert (erc_logger.error.count, erc_logger.warning.count) == (0, 0)


@pytest.mark.parametrize("circuit", [FC, RADIO], ids=["fc", "radio"])
def test_every_pin_lands_on_a_pad_of_an_existing_footprint(circuit):
    for part in circuit.parts:
        assert part.footprint, part.ref
        missing = {p.num for p in part.pins} - pads(part.footprint)
        assert not missing, f"{part.ref} {part.name}: pins {missing} have no pad in {part.footprint}"


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


def test_imu_spare_pins_and_footprint_orientation():
    (imu,) = [p for p in FC.parts if p.name == "ICM-42688-P"]
    assert imu["RESV7"].net.name == "GND"
    assert all(not imu[p].is_connected() or imu[p].net.name not in ("GND", "3V3")
               for p in ("RESV2", "RESV3", "RESV10", "RESV11"))  # NC: the sources disagree on GND vs VDDIO
    assert imu["FSYNC"].net.name == "IMU_INT2" and two_pin(FC, "R", "IMU_INT2", "GND")
    # DS-000347 Fig. 4 (top view) and 10.2: pins 1-4 left top to bottom, 5-7 bottom, 8-11 right bottom to top,
    # 12-14 top right to left; lead centres +-1.1625 / +-0.9125, pitch 0.5.
    text = (KICAD_FP / "Package_LGA.pretty" / "LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y.kicad_mod").read_text()
    at = {n: (float(x), float(y)) for n, x, y in re.findall(r'\(pad "(\d+)" smd \w+\s*\(at (\S+) (\S+)\)', text)}
    side = [-0.75, -0.25, 0.25, 0.75]
    want = {**{str(i + 1): (-1.1625, y) for i, y in enumerate(side)},
            **{str(i + 5): (x, 0.9125) for i, x in enumerate([-0.5, 0, 0.5])},
            **{str(i + 8): (1.1625, y) for i, y in enumerate(reversed(side))},
            **{str(i + 12): (x, -0.9125) for i, x in enumerate([0.5, 0, -0.5])}}
    assert at == want


def test_firmware_pin_defaults_match_the_boards():
    flight = kconfig_defaults("firmware/flight/main/Kconfig.projbuild")
    radio = kconfig_defaults("firmware/radio/main/Kconfig.projbuild")
    p4, c5 = chip(FC), chip(RADIO)
    for sig in ("MOSI", "MISO", "SCLK", "CS"):
        assert boards.gpio(p4, flight[f"ESPFC_C5_SPI_{sig}"]).net.name == f"C5_{sig}"
        assert boards.gpio(c5, radio[f"ESPFC_RADIO_SPI_{sig}"]).net.name == f"C5_{sig}"


def test_stack_connector_carries_the_same_signal_on_both_boards():
    for circuit in (FC, RADIO):
        (j,) = [p for p in circuit.parts if p.value == "STACK"]
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
    q = chip(FC, "Q1")  # DMP1022UFDF: 19 mOhm max at -2.5 V, inside the simulated 30 mOhm
    assert {p.num: p.net.name for p in q.pins} == {"1": "VBAT_SHUNT", "2": "VBAT_SHUNT", "3": "VBUS", "4": "VSYS",
                                                   "5": "VBAT_SHUNT", "6": "VBAT_SHUNT", "7": "VBAT_SHUNT",
                                                   "8": "VSYS"}


def test_espfc_symbol_library_is_regenerated():
    assert library.SYM.read_text() == library.symbol_library()


@pytest.mark.parametrize(("name", "symbol", "min_gap"), [
    ("Texas_DLA0010A_VSON-HR-10_3x2mm_P0.5mm", "TPS63802", 0.2),
    ("Bosch_LGA-10_2x2mm_P0.5mm", "BMP581", 0.2)])  # Bosch: >= 200 um between pads
def test_espfc_footprints(name, symbol, min_gap):
    text = (library.OUT / f"{name}.kicad_mod").read_text()
    assert text == library.FOOTPRINTS[name]  # lib/ESPFC.pretty regenerated
    _, footprint, _, pins = library.SYMBOLS[symbol]
    assert footprint == f"ESPFC:{name}"
    pads = re.findall(r'\(pad "(\d+)" smd \w+ \(at (\S+) (\S+)\) \(size (\S+) (\S+)\)', text)
    assert sorted(int(p[0]) for p in pads) == sorted(p[0] for p in pins)
    boxes = [(float(x) - float(w) / 2, float(y) - float(h) / 2, float(x) + float(w) / 2, float(y) + float(h) / 2)
             for _, x, y, w, h in pads]
    for i, a in enumerate(boxes):
        for b in boxes[i + 1:]:
            assert max(b[0] - a[2], a[0] - b[2], b[1] - a[3], a[1] - b[3]) >= min_gap - 1e-9
    (pin1,) = [b for p, b in zip(pads, boxes) if p[0] == "1"]
    assert pin1[0] < 0 and pin1[1] < 0  # pin 1 top left, as both datasheets' top views


KICAD_CLI = shutil.which("kicad-cli") or "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"


def connectivity(netlist):
    """Each net of a KiCad netlist file as a set of (reference, pin); names ignored, single-pin nets dropped."""
    nets = (set(re.findall(r'\(node\s*\(ref "([^"]*)"\)\s*\(pin "([^"]*)"\)', blk))
            for blk in re.split(r"\(net\s", Path(netlist).read_text())[1:])
    return {frozenset(n) for n in nets if len(n) > 1}


@pytest.mark.skipif(not Path(KICAD_CLI).exists(), reason="needs KiCad's kicad-cli")
@pytest.mark.parametrize("name", list(boards.BOARDS))
def test_drawn_schematic_loads_passes_kicad_erc_and_matches_the_netlist(name, tmp_path):
    ckt = boards.build(name)
    ckt.generate_netlist(file_=str(tmp_path / "skidl.net"))
    sch = boards.schematic(ckt, name, tmp_path)
    run = lambda *a: subprocess.run([KICAD_CLI, "sch", *a, str(sch)], capture_output=True, text=True, check=True)
    run("erc", "--format", "json", "--severity-all", "-o", str(tmp_path / "erc.json"))
    # Library-table warnings only say this sandbox's KiCad has no global symbol/footprint tables configured.
    real = [v for s in json.loads((tmp_path / "erc.json").read_text())["sheets"] for v in s["violations"]
            if v["type"] not in ("lib_symbol_issues", "footprint_link_issues")]
    assert real == []
    out = run("export", "netlist", "-o", str(tmp_path / "drawn.net"))
    assert "annotation" not in out.stdout + out.stderr
    assert connectivity(tmp_path / "drawn.net") == connectivity(tmp_path / "skidl.net")


KICAD_PY = Path("/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3")
STACK_PADS = """
import sys, pcbnew
for fp in pcbnew.LoadBoard(sys.argv[1]).GetFootprints():
    if fp.GetValue() == "STACK":
        print("bottom" if fp.IsFlipped() else "top")
        for p in fp.Pads():
            print(round(pcbnew.ToMM(p.GetPosition().x), 2), round(pcbnew.ToMM(p.GetPosition().y), 2), p.GetNetname())
"""


@pytest.mark.skipif(not KICAD_PY.exists(), reason="needs KiCad's Python (pcbnew)")
def test_pcb_placement_fits_both_boards_and_the_stack_pins_line_up(tmp_path):
    pads = {}
    for name in boards.BOARDS:
        boards.build(name).generate_netlist(file_=str(tmp_path / f"{name}.net"))
        pcb = tmp_path / f"{name}.kicad_pcb"
        fit = subprocess.run([str(KICAD_PY), str(boards.HERE / "pcb.py"), str(tmp_path / f"{name}.net"), str(pcb),
                              "28", "20"], capture_output=True, text=True)
        assert fit.returncode == 0, fit.stdout
        side, *rows = subprocess.run([str(KICAD_PY), "-c", STACK_PADS, str(pcb)], capture_output=True, text=True,
                                     check=True).stdout.split("\n")[:-1]
        pads[name] = (side, sorted(rows))
    (fc_side, fc), (radio_side, radio) = pads["fc_board"], pads["radio_board"]
    assert (fc_side, radio_side) == ("top", "bottom")  # header up on the FC, socket down on the radio board
    assert len(fc) == 12 and fc == radio  # same XY carries the same net on both boards


JAR = boards.HERE / "tools/freerouting-2.5.0.jar"


@pytest.mark.skipif(not (KICAD_PY.exists() and JAR.exists() and Path("/opt/homebrew/opt/openjdk/bin/java").exists()),
                    reason="needs KiCad's Python, Java 25+ and the Freerouting jar (DESIGN.md)")
@pytest.mark.parametrize("name", list(boards.BOARDS))
def test_routed_board_is_complete_and_passes_copper_drc(name, tmp_path):
    boards.build(name).generate_netlist(file_=str(tmp_path / f"{name}.net"))
    pcb = tmp_path / f"{name}.kicad_pcb"
    for script, *args in (("pcb.py", tmp_path / f"{name}.net", pcb, "28", "20"), ("route.py", pcb, "80")):
        run = subprocess.run([str(KICAD_PY), str(boards.HERE / script), *map(str, args)], capture_output=True, text=True)
        assert run.returncode == 0, run.stdout[-2000:] + run.stderr[-2000:]
    drc = tmp_path / "drc.json"
    subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "json", "--severity-all", "-o", str(drc),
                    str(tmp_path / f"{name}-routed.kicad_pcb")], capture_output=True, check=True)
    report = json.loads(drc.read_text())
    # Errors only: dangling vias/stubs the router's optimizer leaves are warnings (redundant copper, not opens).
    # Everything JLCPCB's own check would reject (rules in pcb.py; same set as route.COPPER).
    copper = {"clearance", "shorting_items", "hole_clearance", "hole_to_hole", "copper_edge_clearance",
              "annular_width", "track_width", "via_diameter", "drill_too_small", "solder_mask_bridge",
              "starved_thermal", "isolated_copper"}
    assert [v["description"] for v in report["violations"] if v["type"] in copper] == []
    assert report["unconnected_items"] == []
    # JLCPCB assembly files: every placed part (holes and test points excluded) once in the BOM and once in the CPL.
    run = subprocess.run([str(KICAD_PY), str(boards.HERE / "assembly.py"), str(tmp_path / f"{name}-routed.kicad_pcb"),
                          str(tmp_path)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout[-2000:] + run.stderr[-2000:]
    bom = list(csv.DictReader(open(tmp_path / f"{name}-bom.csv")))
    cpl = list(csv.DictReader(open(tmp_path / f"{name}-cpl.csv")))
    placed = sorted(r["Designator"] for r in cpl)
    assert sorted(ref for r in bom for ref in r["Designator"].split(",")) == placed
    assert len(placed) == int(re.search(r"(\d+) parts", run.stdout).group(1)) > 0
    assert not [ref for ref in placed if ref.startswith(("H", "TP"))]
    assert all(r["Comment"] and r["Footprint"] for r in bom) and all(r["Layer"] in ("Top", "Bottom") for r in cpl)
