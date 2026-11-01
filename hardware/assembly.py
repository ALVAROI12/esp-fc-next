"""JLCPCB assembly files from a routed board: assembly.py out/<board>-routed.kicad_pcb out/assembly/
writes <board>-bom.csv (Comment, Designator, Footprint, LCSC Part #) and <board>-cpl.csv (Designator, Mid X, Mid Y,
Layer, Rotation), the two files JLCPCB's order form takes next to the Gerber zip. Run with KiCad's Python.

Coordinates follow KiCad's own position export (mm, y up, bottom parts not mirrored), which JLCPCB reads as is;
rotations are KiCad's and JLCPCB's part preview is where a rotated polarised part shows up. Parts marked exclude-
from-BOM (mounting holes, test points) are left out. LCSC numbers below were looked up on lcsc.com / jlcpcb.com on
2026-11-01 for the parts that have one listing; passives stay blank and JLCPCB matches them by value and footprint.
"""
import csv
import sys
from pathlib import Path

import pcbnew

# value -> LCSC part number. Blank or missing = let JLCPCB's BOM tool match (passives) or hand-solder (STACK).
LCSC = {
    "ESP32-P4NRW16X": "C9900140808",  # JLCPCB lists it as "ESP32-P4NRW16" (JLCPCB Assembly); confirm on upload
    "ESP32-C5-MINI-1": "",  # no LCSC listing found yet (only ESP32-C5-WROOM-1 C51950749 and the bare C5HF4)
    "ICM-42688-P": "C1850418",
    "BMP581": "C5362283",
    "TPS63802": "C2845237",  # TPS63802DLAR
    "TLV62569DBV": "C141836",  # TLV62569DBVR
    "INA226": "C49851",  # INA226AIDGSR
    "W25Q128JVP": "C190862",  # W25Q128JVPIQ
    "DMP1022UFDF-7": "C155358",
    "2.2u DFE201610E-2R2M": "C337891",  # DFE201610E-2R2M=P2
    "0.47u XFL4020-471ME": "",  # XFL4020 series not stocked; Coilcraft's drop-in is XGL4020-471MEC (no LCSC listing)
    "USB_C_Receptacle_USB2.0_14P": "C3020560",  # GCT USB4105-GF-A
    "Conn_01x08": "C160394",  # JST BM08B-SRSS-TB(LF)(SN)
    "Conn_01x04": "C160390",  # JST BM04B-SRSS-TB(LF)(SN)
    "B5819W": "C64885",  # MDD, SOD-123
    "BOOT": "C116501",  # C&K PTS810SJM250SMTRLFS
    "40MHz 10ppm CL10pF": "C426988",  # JGHC S3240000101040, 3225, 10 pF, +/-10 ppm
}


def main(pcb, out):
    board = pcbnew.LoadBoard(str(pcb))
    out.mkdir(parents=True, exist_ok=True)
    stem = Path(pcb).stem.replace("-routed", "")
    parts = sorted((f for f in board.GetFootprints() if not f.GetAttributes() & pcbnew.FP_EXCLUDE_FROM_BOM),
                   key=lambda f: (f.GetReference().rstrip("0123456789"), int(f.GetReference().lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ"))))
    groups = {}
    for f in parts:
        groups.setdefault((f.GetValue(), str(f.GetFPID().GetLibItemName())), []).append(f.GetReference())
    with open(out / f"{stem}-bom.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (value, fp), refs in groups.items():
            w.writerow([value, ",".join(refs), fp, LCSC.get(value, "")])
    with open(out / f"{stem}-cpl.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for f in parts:
            p = f.GetPosition()
            w.writerow([f.GetReference(), f"{pcbnew.ToMM(p.x):.4f}", f"{-pcbnew.ToMM(p.y):.4f}",
                        "Bottom" if f.GetLayer() == pcbnew.B_Cu else "Top", f"{f.GetOrientationDegrees():.1f}"])
    missing = [v for v, fp in groups if not LCSC.get(v) and not (v[0].isdigit() and fp.split("_")[0] in ("C", "R", "CP"))]
    print(f"{stem}: {len(parts)} parts, {len(groups)} BOM lines; no LCSC number: {missing or 'none'}")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
