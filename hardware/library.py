"""Symbols and footprints KiCad 10 lacks, from the manufacturers' datasheets (DESIGN.md); writes lib/ESPFC.kicad_sym
and lib/ESPFC.pretty (footprints: top view, mm, y down).

TPS63802: TI SLVSEU9D, DLA0010A "Example board layout" (4223750/D). Package 2 x 3 mm.
BMP581: Bosch BST-BMP581-DS004-13 section 8.1.2 (Fig. 30, bottom view) and 8.2: land = package pad + 25 um per
side, no solder mask under the sensor (20 um mask clearance), no vias or traces under it. Package 2 x 2 mm.
"""
from pathlib import Path

LIB = Path(__file__).resolve().parent / "lib"
OUT = LIB / "ESPFC.pretty"
SYM = LIB / "ESPFC.kicad_sym"

# name: (reference prefix, footprint, datasheet, pins). Pins: (number, name, KiCad electrical type, side L/R).
SYMBOLS = {
    "ICM-42688-P": ("U", "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y", "TDK DS-000347 rev 1.2", [
        (8, "VDD", "power_in", "L"), (5, "VDDIO", "power_in", "L"), (12, "CS", "input", "L"),
        (13, "SCLK", "input", "L"), (14, "SDI", "input", "L"), (9, "FSYNC", "input", "L"), (6, "GND", "power_in", "L"),
        (7, "RESV7", "passive", "L"), (1, "SDO", "output", "R"), (4, "INT1", "output", "R"),
        (2, "RESV2", "passive", "R"), (3, "RESV3", "passive", "R"), (10, "RESV10", "passive", "R"),
        (11, "RESV11", "passive", "R")]),
    "BMP581": ("U", "ESPFC:Bosch_LGA-10_2x2mm_P0.5mm", "Bosch BST-BMP581-DS004-13", [
        (10, "VDD", "power_in", "L"), (1, "VDDIO", "power_in", "L"), (6, "CSB", "input", "L"),
        (2, "SCK", "input", "L"), (3, "VSS3", "power_in", "L"), (8, "VSS8", "power_in", "L"),
        (9, "VSS9", "power_in", "L"), (4, "SDI", "bidirectional", "R"), (5, "SDO", "bidirectional", "R"),
        (7, "INT", "output", "R")]),
    # Diodes DS36624 rev 6-2: 19 mOhm max at VGS -2.5 V, 26 at -1.8 V; VGS(th) -0.35 to -0.8 V. Pads as KiCad's
    # footprint: 1, 2, 5, 6 and paddle 7 drain, 3 gate, 4 and pad 8 source.
    "DMP1022UFDF-7": ("Q", "Package_DFN_QFN:Diodes_UDFN2020-6_Type-F", "Diodes DS36624 rev 6-2", [
        (3, "G", "input", "L"), (1, "D", "passive", "R"), (2, "D", "passive", "R"), (5, "D", "passive", "R"),
        (6, "D", "passive", "R"), (7, "D", "passive", "R"), (4, "S", "passive", "L"), (8, "S", "passive", "L")]),
    "TPS63802": ("U", "ESPFC:Texas_DLA0010A_VSON-HR-10_3x2mm_P0.5mm", "TI SLVSEU9D", [
        (10, "VIN", "power_in", "L"), (1, "EN", "input", "L"), (2, "MODE", "input", "L"), (9, "L1", "passive", "L"),
        (3, "AGND", "power_in", "L"), (8, "GND", "power_in", "L"), (6, "VOUT", "power_out", "R"),
        (4, "FB", "input", "R"), (5, "PG", "open_collector", "R"), (7, "L2", "passive", "R")]),
}

# (number, x, y, width, height) of each pad centre.
TPS63802 = [(n, -0.9, -1.0 + 0.5 * (n - 1), 0.6, 0.25) for n in range(1, 6)] + [
    (6, 0.75, 1.0, 0.9, 0.25), (7, 0.75, 0.5, 0.9, 0.25), (8, 0.55, 0.0, 1.3, 0.25),  # pin 8 GND: long pad
    (9, 0.75, -0.5, 0.9, 0.25), (10, 0.75, -1.0, 0.9, 0.25)]
# Fig. 30 is a bottom view; mirrored here. Row pads 0.25 x 0.275 and side pads 0.275 x 0.25, 0.1 in from the edge,
# centres 1.525 apart, pitch 0.5; each grown 25 um per side.
ROW, SIDE, E = (0.3, 0.325), (0.325, 0.3), 0.7625
BMP581 = [(1, -E, -0.25, *SIDE), (2, -E, 0.25, *SIDE), (3, -0.5, E, *ROW), (4, 0.0, E, *ROW), (5, 0.5, E, *ROW),
          (6, E, 0.25, *SIDE), (7, E, -0.25, *SIDE), (8, 0.5, -E, *ROW), (9, 0.0, -E, *ROW), (10, -0.5, -E, *ROW)]


def rect(layer, w, h, width, fill=False):
    x, y = w / 2, h / 2
    return (f'  (fp_rect (start {-x:g} {-y:g}) (end {x:g} {y:g}) (stroke (width {width}) (type solid))'
            f' (fill {"yes" if fill else "no"}) (layer "{layer}"))\n')


def footprint(name, descr, pads, body, mask_margin=None, open_mask=False):
    xs = [abs(x) + w / 2 for _, x, _, w, _ in pads] + [body[0] / 2]
    ys = [abs(y) + h / 2 for _, _, y, _, h in pads] + [body[1] / 2]
    court = (round(2 * max(xs) + 0.5, 2), round(2 * max(ys) + 0.5, 2))  # 0.25 mm courtyard clearance
    bx, by = body[0] / 2, body[1] / 2
    s = (f'(footprint "{name}"\n  (version 20260206)\n  (generator "espfc_library")\n  (layer "F.Cu")\n'
         f'  (descr "{descr}")\n  (attr smd{" allow_soldermask_bridges" if open_mask else ""})\n'
         f'  (property "Reference" "REF**" (at 0 {-court[1] / 2 - 0.6:g} 0) (layer "F.SilkS")'
         ' (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
         f'  (property "Value" "{name}" (at 0 {court[1] / 2 + 0.6:g} 0) (layer "F.Fab")'
         ' (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
         + rect("F.Fab", *body, 0.1) + rect("F.CrtYd", *court, 0.05)
         + (rect("F.Mask", *body, 0, fill=True) if open_mask else "")
         + f'  (fp_circle (center {-bx - 0.25:g} {-by:g}) (end {-bx - 0.15:g} {-by:g}) (stroke (width 0.1)'
           ' (type solid)) (fill yes) (layer "F.SilkS"))\n')  # pin 1 dot
    for n, x, y, w, h in pads:
        margin = f" (solder_mask_margin {mask_margin})" if mask_margin is not None else ""
        s += (f'  (pad "{n}" smd roundrect (at {x:g} {y:g}) (size {w:g} {h:g}) (roundrect_rratio 0.2)'
              f' (layers "F.Cu" "F.Paste" "F.Mask"){margin})\n')
    return s + ")\n"


FOOTPRINTS = {
    "Texas_DLA0010A_VSON-HR-10_3x2mm_P0.5mm": footprint(
        "Texas_DLA0010A_VSON-HR-10_3x2mm_P0.5mm",
        "TI DLA0010A VSON-HR 10 pin, TPS63802 (SLVSEU9D example board layout 4223750/D)", TPS63802, (2.0, 3.0)),
    "Bosch_LGA-10_2x2mm_P0.5mm": footprint(
        "Bosch_LGA-10_2x2mm_P0.5mm",
        "Bosch BMP581 LGA-10 2x2 mm (BST-BMP581-DS004-13 section 8.2), no solder mask under the sensor",
        BMP581, (2.0, 2.0), mask_margin=0.02, open_mask=True),
}

def prop(key, value, y, hide=False):
    return (f'    (property "{key}" "{value}" (at 0 {y:g} 0) (effects (font (size 1.27 1.27))'
            f'{" (hide yes)" if hide else ""}))\n')


def symbol(name, ref, footprint, datasheet, pins):
    """Rectangle with the left-side pins pointing left and the right-side ones pointing right, 2.54 mm apart."""
    rows = max(sum(p[3] == side for p in pins) for side in "LR")
    top = 1.27 * rows + 1.27
    s = (f'  (symbol "{name}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
         + prop("Reference", ref, top + 1.27) + prop("Value", name, -top - 1.27)
         + prop("Footprint", footprint, -top - 3.81, hide=True) + prop("Datasheet", datasheet, -top - 6.35, hide=True)
         + f'    (symbol "{name}_0_1" (rectangle (start -7.62 {top:g}) (end 7.62 {-top:g})'
           ' (stroke (width 0.254) (type default)) (fill (type background))))\n'
         + f'    (symbol "{name}_1_1"\n')
    for side, x, angle in (("L", -10.16, 0), ("R", 10.16, 180)):
        for i, (num, pin, kind, _) in enumerate(p for p in pins if p[3] == side):
            s += (f'      (pin {kind} line (at {x:g} {top - 2.54 - 2.54 * i:g} {angle}) (length 2.54)'
                  f' (name "{pin}" (effects (font (size 1.27 1.27)))) (number "{num}" (effects (font (size 1.27 1.27)))))\n')
    return s + "    )\n  )\n"


def symbol_library():
    return ('(kicad_symbol_lib (version 20251024) (generator "espfc_library")\n'
            + "".join(symbol(name, *spec) for name, spec in SYMBOLS.items()) + ")\n")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for name, text in FOOTPRINTS.items():
        (OUT / f"{name}.kicad_mod").write_text(text)
    SYM.write_text(symbol_library())
