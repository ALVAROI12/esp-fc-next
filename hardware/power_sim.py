"""ngspice check of the 1S power path: does VSYS (TPS63802 input) stay where the 3.3 V rail holds at full load
while the motors pull a current step through the same battery?

Battery = OCV + internal resistance; ESC = current step; FC path = lead, INA226 shunt, load-sharing P-FET,
VSYS bulk caps; buck-boost = averaged constant-power load (P_out / efficiency). Values: DESIGN.md.

The routed copper comes from routes/parasitics.json (parasitics.py): battery_copper() for this path, rail_3v3() for
the 3.3 V rail up to the C5 during a Wi-Fi TX burst.
"""
import json
import re
import subprocess
import tempfile
from pathlib import Path

VIN_FULL_LOAD = 2.3  # TPS63802 SLVSEU9D section 8.5: minimum VIN for 2 A at 3.3 V once started
UVLO_FALLING = 1.25
LAYOUT = json.loads((Path(__file__).resolve().parent / "routes" / "parasitics.json").read_text())

NETLIST = """1S power path
Vocv ocv 0 {ocv}
Rint ocv cell {rint}
Rlead cell vbat 10m
Iesc vbat 0 PULSE(0 {i_motor} 1m 1m 1m 20m 100m)
Rshunt vbat vsh {r_shunt}
Rfet vsh vfet {r_fet}
Rcu vfet vsys {r_copper}
C1 vsys 0 10u
C2 vsys 0 100u ic={vsys0}
Bconv vsys 0 I = V(vsys) > {uvlo} ? {p_out} / ({eff} * max(V(vsys), {uvlo})) : 0
.tran 10u 30m
.meas tran vsys_min MIN v(vsys) from=1m to=30m
.meas tran vbat_min MIN v(vbat) from=1m to=30m
.end
"""


def simulate(ocv=3.5, rint=0.05, i_motor=16.0, r_shunt=0.01, r_fet=0.03, p_out=3.3 * 1.0, eff=0.85, r_copper=0.0):
    """Minimum VSYS and VBAT (volts) during the motor current step; (0, 0) if the converter input collapses."""
    with tempfile.TemporaryDirectory() as d:
        cir = Path(d) / "power.cir"
        cir.write_text(NETLIST.format(ocv=ocv, rint=rint, i_motor=i_motor, p_out=p_out, eff=eff,
                                      uvlo=UVLO_FALLING, vsys0=ocv - 0.2, r_shunt=r_shunt, r_fet=r_fet,
                                      r_copper=max(r_copper, 1e-6)))
        run = subprocess.run(["ngspice", "-b", str(cir)], capture_output=True, text=True, check=False)
    if run.returncode and "timestep too small" in (run.stdout + run.stderr).lower():
        return 0.0, 0.0  # constant-power load ran away: brownout
    run.check_returncode()
    out = run.stdout
    meas = dict(re.findall(r"^(vsys_min|vbat_min)\s*=\s*(\S+)", out, re.MULTILINE))
    return float(meas["vsys_min"]), float(meas["vbat_min"])


def trace(ocv=3.5, rint=0.05, i_motor=16.0, r_shunt=0.01, r_fet=0.03, p_out=3.3 * 1.0, eff=0.85, r_copper=0.0):
    """Waveforms of the same run: rows of (t s, VBAT V, VSYS V, battery current A)."""
    with tempfile.TemporaryDirectory() as d:
        cir, dat = Path(d) / "power.cir", Path(d) / "power.dat"
        cir.write_text(NETLIST.format(ocv=ocv, rint=rint, i_motor=i_motor, p_out=p_out, eff=eff, uvlo=UVLO_FALLING,
                                      vsys0=ocv - 0.2, r_shunt=r_shunt, r_fet=r_fet,
                                      r_copper=max(r_copper, 1e-6)).replace(
            ".end", f".control\nrun\nwrdata {dat} v(vbat) v(vsys) i(Vocv)\n.endc\n.end"))
        subprocess.run(["ngspice", "-b", str(cir)], capture_output=True, check=True)
        rows = [list(map(float, line.split())) for line in dat.read_text().splitlines()]
    return [(r[0], r[1], r[3], -r[5]) for r in rows]  # wrdata repeats the time column before every vector


def battery_copper():
    """Ohms of routed track in series with the battery: ESC connector -> shunt -> P-FET -> TPS63802 VIN."""
    return sum(LAYOUT["fc_board"][k]["mohm"] for k in ("vbat", "vbat_shunt", "vsys")) / 1000


def max_battery_resistance(ocv=3.5, i_motor=16.0, lo=0.0, hi=0.3):
    """Largest pack internal resistance that keeps VSYS at or above VIN_FULL_LOAD (bisection, 1 mOhm)."""
    while hi - lo > 0.001:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if simulate(ocv, mid, i_motor)[0] >= VIN_FULL_LOAD else (lo, mid)
    return lo


# 3.3 V rail. TI's TPS63802 PSpice model (SLVMCX1) does not start under ngspice-47, so the converter is a
# behavioural current source: PI on the output error, one lag for the inductor/loop delay. KP/TAU/TI are fitted to
# the datasheet load step (SLVSEU9D Fig. 10-25) and checked by datasheet_step(). It says nothing about loop stability.
KP, TAU, TI, I_LIMIT = 4.8, 1.0e-6, 15e-6, 4.0  # A/V, s, s, A (boost-mode switch limit, minimum)
MLCC_22U, MLCC_10U, ESR = 9e-6, 5e-6, 0.005  # farads left at 3.3 V bias (GRM188R60J226 DC-bias curve: -59 %), ohms
STACK_PIN_OHM, STACK_PIN_NH = 0.03, 5.0  # per 1.27 mm header contact; two 3V3 pins out, two GND pins back
RAIL = """3V3 rail
Berr 0 xi I = {vset} - V(vout)
Cxi xi 0 1
Bcmd cmd 0 V = {kp} * ({vset} - V(vout)) + {kp} / {ti} * V(xi)
Rlag cmd il {tau}
Clag il 0 1
Bout 0 vout I = min(max(V(il), 0), {i_limit})
{load}
.ic v(vout)={vset} v(il)={i0} v(xi)={xi0} {ic}
.tran 20n {t_stop} uic
{meas}
.end
"""


def _rail(load, i0, meas, vset, t_stop="600u", ic=""):
    cir = RAIL.format(vset=vset, kp=KP, ti=TI, tau=TAU, i_limit=I_LIMIT, load=load, i0=i0, xi0=i0 * TI / KP,
                      ic=ic, t_stop=t_stop, meas="\n".join(f".meas tran {m}" for m in meas))
    with tempfile.TemporaryDirectory() as d:
        (Path(d) / "rail.cir").write_text(cir)
        run = subprocess.run(["ngspice", "-b", str(Path(d) / "rail.cir")], capture_output=True, text=True, check=True)
    return {k: float(v) for k, v in re.findall(r"^(\w+)\s*=\s*(\S+)", run.stdout, re.MULTILINE)}


def datasheet_step():
    """Undershoot (volts, negative) of the converter model in the datasheet set-up: one 22 uF, 100 mA -> 1 A in 1 us."""
    out = _rail(f"C1 vout c1 {MLCC_22U} ic=3.3\nR1 c1 0 {ESR}\nIload vout 0 PULSE(0.1 1 100u 1u 1u 1 2)", 0.1,
                ["vmin MIN v(vout) from=100u to=600u"], vset=3.3)
    return out["vmin"] - 3.3


def rail_3v3(vset=3.31 * 0.99, i_fc=0.45, i_rx=0.110, i_tx=0.408, rise=2e-6):
    """Lowest 3.3 V rail voltages during a C5 TX burst: at the P4 (converter output) and at the C5 supply pin.
    vset: 511k/91k divider at the -1 % feedback limit; i_fc: P4 + sensors (DESIGN.md budget); i_rx/i_tx: C5
    datasheet v1.5 Table 5-8, 5 GHz RX and 802.11a TX peak."""
    fc, radio = LAYOUT["fc_board"], LAYOUT["radio_board"]
    r_stack = fc["3v3_stack"]["mohm"] / 1000 + STACK_PIN_OHM  # two pins out in parallel + two back = one contact
    l_stack = (fc["3v3_stack"]["nh"] + STACK_PIN_NH) * 1e-9
    load = f"""Rco vout co {fc['3v3_cout']['mohm'] / 1000}
Lco co cf {fc['3v3_cout']['nh']}n
Cf cf cfe {2 * MLCC_22U} ic={vset}
Rcf cfe 0 {ESR / 2}
Ifc vout 0 {i_fc}
Rst vout st {r_stack}
Lst st hdr {l_stack}
Rrb hdr rb {radio['3v3']['mohm'] / 1000}
Lrb rb c5 {radio['3v3']['nh']}n
Rci c5 ci {radio['3v3_cin']['mohm'] / 1000}
Lci ci cr {radio['3v3_cin']['nh']}n
Cr cr cre {MLCC_22U + MLCC_10U} ic={vset}
Rcr cre 0 {ESR / 2}
Ic5 c5 0 PULSE({i_rx} {i_tx} 100u {rise} {rise} 300u 1)"""
    v_c5 = vset - i_rx * (r_stack + radio["3v3"]["mohm"] / 1000)
    out = _rail(load, i_fc + i_rx, ["p4_min MIN v(vout) from=50u to=600u", "c5_min MIN v(c5) from=50u to=600u",
                                    "c5_settled FIND v(c5) AT=390u"], vset, ic=f"v(c5)={v_c5} v(hdr)={v_c5}")
    return {"p4_min": out["p4_min"], "c5_min": out["c5_min"], "c5_settled": out["c5_settled"]}


if __name__ == "__main__":
    for ocv, rint, r_shunt, r_fet in ((4.2, 0.05, 0.01, 0.03), (3.5, 0.05, 0.01, 0.03), (3.5, 0.10, 0.01, 0.03),
                                      (3.5, 0.05, 0.02, 0.085)):  # last: first draft, 20 mOhm shunt + AO3401A
        vsys, vbat = simulate(ocv, rint, r_shunt=r_shunt, r_fet=r_fet)
        print(f"OCV {ocv} V, Rint {rint * 1000:.0f} mOhm, path {(r_shunt + r_fet) * 1000:.0f} mOhm, 16 A step: "
              f"VBAT min {vbat:.2f} V, VSYS min {vsys:.2f} V")
    print(f"max pack resistance at 3.5 V, 16 A: {max_battery_resistance() * 1000:.0f} mOhm")
    vsys, _ = simulate(r_copper=battery_copper())
    print(f"with the routed copper ({battery_copper() * 1000:.0f} mOhm at {LAYOUT['assumptions']['temp_c']:.0f} C): "
          f"VSYS min {vsys:.2f} V (limit {VIN_FULL_LOAD} V)")
    for name, row in rail_3v3().items():
        print(f"3V3 rail, C5 5 GHz TX burst: {name} {row:.3f} V")
    print(f"converter model vs TPS63802 Fig. 10-25 (100 mA -> 1 A, -190 mV): {datasheet_step() * 1000:.0f} mV")
