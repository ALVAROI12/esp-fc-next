"""ngspice check of the 1S power path: does VSYS (TPS63802 input) stay where the 3.3 V rail holds at full load
while the motors pull a current step through the same battery?

Battery = OCV + internal resistance; ESC = current step; FC path = lead, INA226 shunt, load-sharing P-FET,
VSYS bulk caps; buck-boost = averaged constant-power load (P_out / efficiency). Values: DESIGN.md.
"""
import re
import subprocess
import tempfile
from pathlib import Path

VIN_FULL_LOAD = 2.3  # TPS63802 SLVSEU9D section 8.5: minimum VIN for 2 A at 3.3 V once started
UVLO_FALLING = 1.25

NETLIST = """1S power path
Vocv ocv 0 {ocv}
Rint ocv cell {rint}
Rlead cell vbat 10m
Iesc vbat 0 PULSE(0 {i_motor} 1m 1m 1m 20m 100m)
Rshunt vbat vsh {r_shunt}
Rfet vsh vsys {r_fet}
C1 vsys 0 10u
C2 vsys 0 100u ic={vsys0}
Bconv vsys 0 I = V(vsys) > {uvlo} ? {p_out} / ({eff} * max(V(vsys), {uvlo})) : 0
.tran 10u 30m
.meas tran vsys_min MIN v(vsys) from=1m to=30m
.meas tran vbat_min MIN v(vbat) from=1m to=30m
.end
"""


def simulate(ocv=3.5, rint=0.05, i_motor=16.0, r_shunt=0.01, r_fet=0.03, p_out=3.3 * 1.0, eff=0.85):
    """Minimum VSYS and VBAT (volts) during the motor current step; (0, 0) if the converter input collapses."""
    with tempfile.TemporaryDirectory() as d:
        cir = Path(d) / "power.cir"
        cir.write_text(NETLIST.format(ocv=ocv, rint=rint, i_motor=i_motor, p_out=p_out, eff=eff,
                                      uvlo=UVLO_FALLING, vsys0=ocv - 0.2, r_shunt=r_shunt, r_fet=r_fet))
        run = subprocess.run(["ngspice", "-b", str(cir)], capture_output=True, text=True, check=False)
    if run.returncode and "timestep too small" in (run.stdout + run.stderr).lower():
        return 0.0, 0.0  # constant-power load ran away: brownout
    run.check_returncode()
    out = run.stdout
    meas = dict(re.findall(r"^(vsys_min|vbat_min)\s*=\s*(\S+)", out, re.MULTILINE))
    return float(meas["vsys_min"]), float(meas["vbat_min"])


def max_battery_resistance(ocv=3.5, i_motor=16.0, lo=0.0, hi=0.3):
    """Largest pack internal resistance that keeps VSYS at or above VIN_FULL_LOAD (bisection, 1 mOhm)."""
    while hi - lo > 0.001:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if simulate(ocv, mid, i_motor)[0] >= VIN_FULL_LOAD else (lo, mid)
    return lo


if __name__ == "__main__":
    for ocv, rint, r_shunt, r_fet in ((4.2, 0.05, 0.01, 0.03), (3.5, 0.05, 0.01, 0.03), (3.5, 0.10, 0.01, 0.03),
                                      (3.5, 0.05, 0.02, 0.085)):  # last: first draft, 20 mOhm shunt + AO3401A
        vsys, vbat = simulate(ocv, rint, r_shunt=r_shunt, r_fet=r_fet)
        print(f"OCV {ocv} V, Rint {rint * 1000:.0f} mOhm, path {(r_shunt + r_fet) * 1000:.0f} mOhm, 16 A step: "
              f"VBAT min {vbat:.2f} V, VSYS min {vsys:.2f} V")
    print(f"max pack resistance at 3.5 V, 16 A: {max_battery_resistance() * 1000:.0f} mOhm")
