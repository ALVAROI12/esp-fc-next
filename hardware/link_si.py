"""ngspice signal-integrity check of the P4 <-> C5 SPI link (10 MHz, mode 0) over the routed tracks and the stack
header: overshoot at the receiver and ring-back into the input threshold band, which on SCLK would double-clock.

Driver = ramp behind the pad's output resistance; each board's track = RLC ladder with the extracted length and
resistance (routes/parasitics.json); header pin = series L + shunt C; receiver = pin capacitance. Espressif
publishes no IBIS models for the P4 or C5, so the pad is the datasheet DC drive strength plus an assumed edge.
"""
import subprocess
import tempfile
from pathlib import Path

from power_sim import LAYOUT

VDD = 3.3
VIH, VIL = 0.75 * VDD, 0.25 * VDD  # P4 v0.7 / C5 v1.5 datasheets, Table 5-4
V_PIN_MAX, V_PIN_MIN = VDD + 0.3, -0.3  # same tables: VIH max, VIL min
R_DRIVER = 33.0  # ohms at the default 20 mA strength (PAD_DRIVER 3: 40 mA at 0.66 V drop = 16.5 ohms, halved drive)
EDGE = 1e-9  # s, 10-90 % pad edge: assumed (not in the datasheets); faster rings more
C_PIN, C_HEADER, L_HEADER = 2e-12, 1e-12, 5e-9  # receiver pin (datasheet), 1.27 mm header pin (assumed)
PS_PER_MM = 7.0  # sqrt(Dk 4.4) / c: embedded track, the slow (long) end
SPI_HZ = 10e6  # c5_link.c
Z0 = LAYOUT["assumptions"]["z0_ohm_0.1mm"]
# signal: board whose chip drives it
SIGNALS = {"sclk": "fc_board", "mosi": "fc_board", "cs": "fc_board", "miso": "radio_board"}


def line(name, a, b, row):
    """One extracted track as an RLC ladder, one section per mm (7 ps: far below the edge); the net's branch copper
    is lumped as capacitance at b. ngspice's LTRA line stalls on these short lines."""
    n = max(1, round(row["mm"]))
    td, r = row["mm"] * PS_PER_MM * 1e-12 / n, row["mohm"] / 1000 / n
    nodes = [a] + [f"{name}{i}" for i in range(1, n)] + [b]
    out = "".join(f"R{name}{i} {x} {name}r{i} {r}\nL{name}{i} {name}r{i} {y} {Z0 * td}\nC{name}{i} {y} 0 {td / Z0}\n"
                  for i, (x, y) in enumerate(zip(nodes, nodes[1:])))
    return out + f"Cs{name} {b} 0 {row['stub_mm'] * PS_PER_MM * 1e-12 / Z0 + 1e-15}\n"


def simulate(signal, edge=EDGE, r_driver=R_DRIVER):
    """Receiver waveform [(t, v)] for one rising and one falling edge of a 10 MHz square wave."""
    first = SIGNALS[signal]
    second = "radio_board" if first == "fc_board" else "fc_board"
    half = 0.5 / SPI_HZ
    with tempfile.TemporaryDirectory() as d:
        dat = Path(d) / "rx.dat"
        (Path(d) / "link.cir").write_text(
            f"SPI link {signal}\nVd src 0 PULSE(0 {VDD} 5n {edge / 0.8} {edge / 0.8} {half} {2 * half})\n"
            f"Rd src a {r_driver}\nCd a 0 {C_PIN}\n" + line("a", "a", "h1", LAYOUT[first][f"link_{signal}"])
            + f"Lh h1 h2 {L_HEADER}\nCh h2 0 {C_HEADER}\n" + line("b", "h2", "rx", LAYOUT[second][f"link_{signal}"])
            + f"Crx rx 0 {C_PIN}\n.tran 20p {2 * half + 5e-9}\n.control\nrun\nwrdata {dat} v(rx)\n.endc\n.end\n")
        subprocess.run(["ngspice", "-b", str(Path(d) / "link.cir")], capture_output=True, check=True)
        return [tuple(map(float, row.split())) for row in dat.read_text().splitlines()]


def check(signal, **kw):
    """Overshoot, undershoot and ring-back of one signal at its receiver.
    ringback_high: lowest level after the rising edge first reaches VIH (must stay >= VIH);
    ringback_low: highest level after the falling edge first reaches VIL (must stay <= VIL)."""
    wave = simulate(signal, **kw)
    fall_at = 5e-9 + 0.5 / SPI_HZ
    rise = [v for t, v in wave if t < fall_at]
    fall = [v for t, v in wave if t >= fall_at]
    high = rise[next(i for i, v in enumerate(rise) if v >= VIH):]
    low = fall[next(i for i, v in enumerate(fall) if v <= VIL):]
    delay = next(t for t, v in wave if v >= VDD / 2) - 5e-9 - kw.get("edge", EDGE) / 1.6
    return {"vmax": max(v for _, v in wave), "vmin": min(v for _, v in wave), "ringback_high": min(high),
            "ringback_low": max(low), "delay_ns": delay * 1e9}


def passes(result):
    return (result["vmax"] <= V_PIN_MAX and result["vmin"] >= V_PIN_MIN and result["ringback_high"] >= VIH
            and result["ringback_low"] <= VIL)


if __name__ == "__main__":
    total = max(LAYOUT["fc_board"][f"link_{s}"]["mm"] + LAYOUT["radio_board"][f"link_{s}"]["mm"] for s in SIGNALS)
    print(f"Z0 {Z0} ohm, longest link {total:.0f} mm = {total * PS_PER_MM / 1000:.2f} ns one way, edge {EDGE * 1e9} ns, "
          f"driver {R_DRIVER} ohm; limits: pin {V_PIN_MIN}..{V_PIN_MAX:.1f} V, VIH {VIH:.2f} V, VIL {VIL:.2f} V")
    for sig in SIGNALS:
        for label, kw in (("nominal", {}), ("51 ohm series, 1 ns edge", {"r_driver": R_DRIVER + 51}),
                          ("51 ohm series, 0.3 ns edge", {"edge": 0.3e-9, "r_driver": R_DRIVER + 51})):
            r = check(sig, **kw)
            print(f"{sig:5} {label:28} max {r['vmax']:.2f} V, min {r['vmin']:.2f} V, ring-back high {r['ringback_high']:.2f} V"
                  f" / low {r['ringback_low']:.2f} V, delay {r['delay_ns']:.2f} ns: {'ok' if passes(r) else 'FAIL'}")
