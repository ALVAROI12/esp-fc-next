"""First-pass PCB from the SKiDL netlist: board outline, M2 holes, every footprint with its nets, greedy placement.
Not routed. It answers "does it fit", and gives layout a starting file.

Runs under KiCad's own Python (pcbnew):  <KiCad.app>/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
    pcb.py out/fc_board.net out/fc_board.kicad_pcb 28 20 [4]    (board side mm, hole spacing mm; 0 = no holes; copper layers)
Prints a fit report; parts that did not fit are left beside the board.
"""
import json
import math
import re
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
KICAD_FP = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints")
LIBS = {"ESPFC": HERE / "lib/ESPFC.pretty", "PCM_Espressif": HERE / "lib/espressif/Espressif.pretty"}
RAILS = {"GND", "3V3", "VDD_HP", "VSYS", "VBAT"}  # too common to pull parts towards anything
CENTER = (100.0, 100.0)  # board centre on the KiCad page, mm
GAP, EDGE, STEP = 0.15, 0.3, 0.1  # courtyard spacing, keep-in from the edge, placement grid (mm)
# Ring round the P4 courtyard that only tracks and vias may use: its 0.35 mm pitch pins need two staggered rows of
# 0.3 mm vias (0.4 mm pitch) to escape; at GAP the 0201 caps left M2 and FLASH_CK unroutable. Not counted against
# the board edge (the P4 stays flush with it); 0.6 pushed the P4 right and J3 off the edge, 0.45 keeps J3's spot.
FANOUT = 0.25
BAND = 0.6  # bottom-side keep-out inside the P4's pin rows (the fanout vias), see the frame below
# Fixed spots by value, (x, y, degrees) from the board centre. The stack header must sit at the same place on both
# boards: left edge, between the M2 holes. USB-C on the bottom edge, its "PCB Edge" mark (y=3.1) on the board edge.
FIXED = {"STACK": (-10.5, -7.0, 0), "USB_C_Receptacle_USB2.0_14P": (0, 12.75 - 3.1, 0)}
BOTTOM = {"radio_board": {"STACK"}}  # the radio board sits on top, its stack socket faces down


# Battery, converter and rail nets: up to ~1.5 A (buck-boost input at 2.4 V and full 3.3 V load).
# ponytail: one width for all; 0.3 mm outer 1 oz runs ~20 C over ambient at 1.5 A (IPC-2152), pours if it gets hot.
POWER = ["VBAT", "VBAT_SHUNT", "VSYS", "BB_L1", "BB_L2", "3V3", "VBUS", "HP_SW", "VDD_HP"]
POWER_WIDTH = 0.3


def netclass(name, width, priority):
    return {"name": name, "clearance": 0.09, "track_width": width, "via_diameter": 0.35, "via_drill": 0.15,
            "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.1, "diff_pair_gap": 0.1,
            "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
            "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": priority}


def mm(v):
    return pcbnew.FromMM(v)


def parse(netlist):
    text = Path(netlist).read_text()
    comps = [(re.search(r'\(ref "([^"]*)"', c)[1], re.search(r'\(value "([^"]*)"', c)[1],
              re.search(r'\(footprint "([^"]*)"', c)[1]) for c in re.split(r"\(comp\s", text.split("(nets")[0])[1:]]
    nets = {}
    for blk in re.split(r"\(net\s", text)[1:]:
        name = re.search(r'\(name "([^"]*)"', blk)[1]
        nets[name] = re.findall(r'\(node\s*\(ref "([^"]*)"\)\s*\(pin "([^"]*)"\)', blk)
    return comps, nets


def outline(board, side, holes):
    h = side / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pcbnew.VECTOR2I(mm(CENTER[0] + x1), mm(CENTER[1] + y1)))
        seg.SetEnd(pcbnew.VECTOR2I(mm(CENTER[0] + x2), mm(CENTER[1] + y2)))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(mm(0.1))
        board.Add(seg)
    keepouts = []
    if holes:
        for sx in (-1, 1):
            for sy in (-1, 1):
                fp = pcbnew.FootprintLoad(str(KICAD_FP / "MountingHole.pretty"), "MountingHole_2.2mm_M2")
                fp.SetReference(f"H{len(keepouts) + 1}")
                fp.Reference().SetLayer(pcbnew.F_Fab)
                fp.SetPosition(pcbnew.VECTOR2I(mm(CENTER[0] + sx * holes / 2), mm(CENTER[1] + sy * holes / 2)))
                board.Add(fp)
                r = courtyard(fp)[2]  # the footprint's courtyard: M2 screw head plus margin
                keepouts.append((sx * holes / 2 - r, sy * holes / 2 - r, sx * holes / 2 + r, sy * holes / 2 + r))
    return keepouts


def courtyard(fp):
    """Courtyard box relative to the footprint position, (x1, y1, x2, y2) in mm."""
    box = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd).BBox()
    if box.GetWidth() == 0:
        box = fp.GetBoundingBox(False)
    p = fp.GetPosition()
    return (pcbnew.ToMM(box.GetLeft() - p.x), pcbnew.ToMM(box.GetTop() - p.y),
            pcbnew.ToMM(box.GetRight() - p.x), pcbnew.ToMM(box.GetBottom() - p.y))


def through(fp):
    """Has plated or bare holes, so it occupies both sides."""
    return any(p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) for p in fp.Pads())


def zone_box(fp):
    """Bounding box of the footprint's keep-out zones relative to its position, mm."""
    p, boxes = fp.GetPosition(), [z.GetBoundingBox() for z in fp.Zones()]
    return (pcbnew.ToMM(min(b.GetLeft() for b in boxes) - p.x), pcbnew.ToMM(min(b.GetTop() for b in boxes) - p.y),
            pcbnew.ToMM(max(b.GetRight() for b in boxes) - p.x), pcbnew.ToMM(max(b.GetBottom() for b in boxes) - p.y))


def free(rect, placed, half, edge=EDGE):
    x1, y1, x2, y2 = rect
    if x1 < -half + edge or y1 < -half + edge or x2 > half - edge or y2 > half - edge:
        return False
    return all(x2 + GAP <= a1 or a2 + GAP <= x1 or y2 + GAP <= b1 or b2 + GAP <= y1 for a1, b1, a2, b2 in placed)


def main(netlist, out, side, holes, layers=4):
    comps, nets = parse(netlist)
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(layers)
    keepouts = outline(board, side, holes)
    netinfo = {}
    for name in nets:
        netinfo[name] = pcbnew.NETINFO_ITEM(board, name)
        board.Add(netinfo[name])
    pin_net = {(ref, pin): name for name, nodes in nets.items() for ref, pin in nodes}

    fps = {}
    for ref, value, footprint in comps:
        lib, name = footprint.split(":")
        fp = pcbnew.FootprintLoad(str(LIBS.get(lib, KICAD_FP / f"{lib}.pretty")), name)
        fp.SetReference(ref)
        fp.SetValue(value)
        for pad in fp.Pads():
            n = pin_net.get((ref, pad.GetNumber()))
            if n:
                pad.SetNet(netinfo[n])
        board.Add(fp)
        fps[ref] = fp

    # Neighbours: parts sharing a signal net (not a rail) pull towards each other.
    links = {ref: set() for ref in fps}
    for name, nodes in nets.items():
        if name not in RAILS:
            refs = {r for r, _ in nodes}
            for r in refs:
                links[r] |= refs - {r}

    half = side / 2
    placed = {"top": list(keepouts), "bottom": list(keepouts)}
    where = {}
    unplaced = []

    def put(ref, fp, layer, x, y, rects):
        fp.SetPosition(pcbnew.VECTOR2I(mm(CENTER[0] + x), mm(CENTER[1] + y)))
        for side_ in (["top", "bottom"] if through(fp) else [layer]):
            placed[side_] += rects
        where[ref] = (x, y)

    # Modules with a keep-out (the C5 antenna) go first: the antenna hangs over the top edge, so its keep-out lies
    # beyond the board (flush with the edge it held two M2 holes and the screws next to the antenna).
    for ref, fp in fps.items():
        if fp.Zones():
            angle = min((0, 90, 180, 270), key=lambda a: (fp.SetOrientationDegrees(a), zone_box(fp)[1])[1])
            fp.SetOrientationDegrees(angle)
            zx1, zy1, zx2, zy2 = zone_box(fp)
            cx1, cy1, cx2, cy2 = courtyard(fp)
            y = -half - zy2
            put(ref, fp, "top", -(cx1 + cx2) / 2, y, [(-(cx2 - cx1) / 2, y + cy1, (cx2 - cx1) / 2, y + cy2),
                                                     (-(cx1 + cx2) / 2 + zx1, y + zy1, -(cx1 + cx2) / 2 + zx2, y + zy2)])

    for ref, fp in fps.items():
        if fp.GetValue() in FIXED:
            x, y, angle = FIXED[fp.GetValue()]
            if fp.GetValue() in BOTTOM.get(Path(out).stem, ()):
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
                angle += 180  # KiCad counts bottom-side rotation mirrored: 180 keeps the pins running the same way
            fp.SetOrientationDegrees(angle)
            cx1, cy1, cx2, cy2 = courtyard(fp)
            put(ref, fp, "top", x, y, [(x + cx1, y + cy1, x + cx2, y + cy2)])

    grid = [i * STEP for i in range(-int(half / STEP), int(half / STEP) + 1)]
    first = ["U1"] + [r for r in fps if fps[r].GetValue() == "ICM-42688-P" or r[0] == "Y"]  # top-only parts first

    def area(r):
        x1, y1, x2, y2 = courtyard(fps[r])
        return (x2 - x1) * (y2 - y1)

    def rect_of(r):
        x1, y1, x2, y2 = courtyard(fps[r])
        return (where[r][0] + x1, where[r][1] + y1, where[r][0] + x2, where[r][1] + y2)
    def crystal():
        """Y1 and its load caps beside the P4's XTAL pins (pins 99/100 face right with the P4 at 270 degrees): a few
        mm of F.Cu, no vias. Greedy placement put the crystal 13 mm from the pins (25/42 mm traces, 3 vias each, 21 mm
        of them cutting the In1 plane) and the caps 12 mm away at the top corners."""
        xtal = {p.GetNetname(): p.GetPosition() for p in fps["U1"].Pads() if p.GetNetname().startswith("XTAL_")}
        y = sum(pcbnew.ToMM(p.y) for p in xtal.values()) / len(xtal) - CENTER[1]
        fp = fps["Y1"]
        fp.SetOrientationDegrees(270)  # pad 1 (XTAL_P) top-left towards pin 100, pad 3 (XTAL_N) bottom-right
        cx1, cy1, cx2, cy2 = courtyard(fp)
        x = rect_of("U1")[2] + FANOUT + GAP - cx1
        put("Y1", fp, "top", x, y, [(x + cx1, y + cy1, x + cx2, y + cy2)])
        for net, below in (("XTAL_P", False), ("XTAL_N", True)):
            ref = next(r for r in fps if r not in where and any(p.GetNetname() == net for p in fps[r].Pads()))
            cap = fps[ref]
            cap.SetOrientationDegrees(0)
            kx1, ky1, kx2, ky2 = courtyard(cap)
            cy = y + (cy2 + GAP - ky1 if below else cy1 - GAP - ky2)
            put(ref, cap, "top", x, cy, [(x + kx1, cy + ky1, x + kx2, cy + ky2)])

    def beside(ref, anchor):
        """`ref` next to `anchor` on the same side, at the anchor's edge nearest their shared pins. The buck-boost
        inductor and its TPS63802 share the switch nodes BB_L1/BB_L2: placed apart (inductor on the bottom, IC on
        top) the router squeezed BB_L1 to 0.09 mm for 3.6 mm between other tracks, a fuse at the inductor current."""
        fp, a = fps[ref], fps[anchor]
        if fp.IsFlipped() != a.IsFlipped():
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        layer = "bottom" if a.IsFlipped() else "top"
        ax1, ay1, ax2, ay2 = rect_of(anchor)
        shared = ({p.GetNetname() for p in fp.Pads()} & {p.GetNetname() for p in a.Pads()}) - RAILS - {""}
        best = None
        for angle in (0, 90, 180, 270):
            fp.SetOrientationDegrees(angle)
            cx1, cy1, cx2, cy2 = courtyard(fp)
            for x, y in ((ax1 - GAP - cx2, (ay1 + ay2) / 2), (ax2 + GAP - cx1, (ay1 + ay2) / 2),
                         ((ax1 + ax2) / 2, ay1 - GAP - cy2), ((ax1 + ax2) / 2, ay2 + GAP - cy1)):
                rect = (x + cx1, y + cy1, x + cx2, y + cy2)
                if not all(free(rect, placed[s], half) for s in (["top", "bottom"] if through(fp) else [layer])):
                    continue
                fp.SetPosition(pcbnew.VECTOR2I(mm(CENTER[0] + x), mm(CENTER[1] + y)))
                d = sum(min((p.GetPosition() - q.GetPosition()).EuclideanNorm() for q in a.Pads()
                            if q.GetNetname() == p.GetNetname()) for p in fp.Pads() if p.GetNetname() in shared)
                if best is None or d < best[0]:
                    best = (d, angle, x, y, rect)
        if best:
            _, angle, x, y, rect = best
            fp.SetOrientationDegrees(angle)
            put(ref, fp, layer, x, y, [rect])

    # P4 first, then connectors (they need edge strips), then the other top-only parts, then the big parts (over
    # 8 mm2: inductors, ICs, bulk caps; placed after the 0201 caps the 4 mm inductor found no square left), then
    # the 100n decoupling caps (they fit the gaps next to the ICs), then by size.
    order = sorted((r for r in fps if r not in where),
                   key=lambda r: (r != "U1", r[0] != "J", r not in first, area(r) < 8, fps[r].GetValue() != "100n",
                                  -area(r)))
    decoupled = {}  # rail net -> (x, y) of the decoupling caps placed on it
    frame = []  # bottom-side keep-out round the P4's pin rows, set once U1 is placed
    for ref in order:
        if ref in where:  # the crystal cluster
            continue
        fp = fps[ref]
        near = [where[r] for r in links[ref] if r in where]
        tx, ty = (sum(p[0] for p in near) / len(near), sum(p[1] for p in near) / len(near)) if near else (0, 0)
        rail = None
        if not near and ref[0] == "C":
            # Decoupling cap (rails only, no neighbour): aim at the IC rail pin farthest from the caps already on
            # that rail, so each supply pin of the P4 gets its own cap next to it. Without this the 100n caps
            # were placed last by size and ended up along the board edges, 10 mm from the chip.
            rail = next((p.GetNetname() for p in fp.Pads() if p.GetNetname() not in ("GND", "")), None)
            pins = [(pcbnew.ToMM(p.GetPosition().x) - CENTER[0], pcbnew.ToMM(p.GetPosition().y) - CENTER[1])
                    for r in where if r[0] == "U" for p in fps[r].Pads() if p.GetNetname() == rail]
            if pins:
                tx, ty = max(pins, key=lambda q: min((math.hypot(q[0] - c[0], q[1] - c[1])
                                                      for c in decoupled.get(rail, [])), default=0))
        if ref[0] == "J":  # connectors: as close to an edge as they fit
            spots = sorted(((x, y) for x in grid for y in grid), key=lambda p: (half - max(abs(p[0]), abs(p[1])),
                                                                                 math.hypot(p[0] - tx, p[1] - ty)))
        else:
            spots = sorted(((x, y) for x in grid for y in grid), key=lambda p: math.hypot(p[0] - tx, p[1] - ty))
        # The P4, crystal and IMU stay on top; everything else may go underneath.
        sides = ["top"] if ref in first else ["top", "bottom"]
        best = None  # the first spot that fits on either side, at either angle: (spot rank, layer, angle, x, y, rect)
        for layer in sides:
            if (layer == "bottom") != fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            for angle in ((270,) if ref == "U1" else (0, 90)):  # P4 XTAL pins to the right, see crystal()
                fp.SetOrientationDegrees(angle)
                cx1, cy1, cx2, cy2 = courtyard(fp)
                # No connectors or test points under the P4: a body there blocks the pin fanout vias (TP2 under the
                # P4 left U0RXD unrouted in some runs). Nothing else on the pin rows either: parts there go inside
                # the frame (under the thermal pads) or outside the fanout ring.
                blocked = []
                if layer == "bottom" and "U1" in where:
                    # Small parts only: a 0201 pad on a pin row takes the via's spot, an IC body does not.
                    blocked = [rect_of("U1")] if ref[0] == "J" or ref[:2] == "TP" else frame if area(ref) < 8 else []
                ring = FANOUT if fp.GetValue().startswith("ESP32-P4") else 0
                for i, (x, y) in enumerate(spots):
                    if best and i >= best[0]:
                        break
                    core = (x + cx1, y + cy1, x + cx2, y + cy2)
                    rect = (core[0] - ring, core[1] - ring, core[2] + ring, core[3] + ring)
                    # The ring keeps parts away; the edge and the M2 hole keep-outs see the bare courtyard.
                    if all(free(core, placed[s][:len(keepouts)], half) and
                           free(rect, placed[s][len(keepouts):] + blocked, half, EDGE - ring)
                           for s in (["top", "bottom"] if through(fp) else [layer])):
                        best = (i, layer, angle, x, y, rect)
                        break
        done = best is not None
        if done:
            _, layer, angle, x, y, rect = best
            if (layer == "bottom") != fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetOrientationDegrees(angle)
            put(ref, fp, layer, x, y, [rect])
            if rail:
                decoupled.setdefault(rail, []).append((x, y))
            if ref == "U1":
                crystal()
                # The P4's fanout vias sit within ~1 mm of its two pin rows, on both sides of the board: on the
                # bottom, keep that frame (pin box grown to the fanout ring, less the inner square) free of parts.
                ox1, oy1, ox2, oy2 = rect  # courtyard plus the ring
                px = [pcbnew.ToMM(p.GetPosition().x) - CENTER[0] for p in fp.Pads()]
                py = [pcbnew.ToMM(p.GetPosition().y) - CENTER[1] for p in fp.Pads()]
                ix1, iy1, ix2, iy2 = min(px) + BAND, min(py) + BAND, max(px) - BAND, max(py) - BAND
                frame = [(ox1, oy1, ox2, iy1), (ox1, iy2, ox2, oy2), (ox1, iy1, ix1, iy2), (ix2, iy1, ox2, iy2)]
            if fp.GetValue().startswith("0.47u"):  # the buck-boost inductor: its TPS63802 goes right beside it
                beside(next(r for r in fps if fps[r].GetValue() == "TPS63802"), ref)
        if not done:
            if fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetPosition(pcbnew.VECTOR2I(mm(CENTER[0] + half + 5 + 12 * (len(unplaced) // 6)),
                                           mm(CENTER[1] - half + 5 * (len(unplaced) % 6))))
            unplaced.append(ref)

    for fp in fps.values():  # reference designators on the fab layer: a 25 mm board has no room for them on silk
        fp.Reference().SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)

    # GND: one solid plane on In1, the reference for every signal; each GND pad gets its own via to it when routed.
    # No outer pours: on a board this dense they break into fragments between the tracks. In2 for signals.
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.In1_Cu)
    zone.SetNet(netinfo["GND"])
    zone.SetLocalClearance(mm(0.1))  # 0.2 let the voids round the via field under the P4 merge and split the plane
    zone.SetMinThickness(mm(0.15))
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)  # vias and through-hole pads join the plane solid
    poly = zone.Outline()
    poly.NewOutline()
    for x, y in ((-half, -half), (half, -half), (half, half), (-half, half)):
        poly.Append(mm(CENTER[0] + x), mm(CENTER[1] + y))
    board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    pcbnew.SaveBoard(str(out), board)
    # Rules: JLCPCB 4-layer, 1 oz outer copper (jlcpcb.com/capabilities/pcb-capabilities, 2026-10): 0.09 mm
    # track/space, via 0.15 hole with a 0.35 pad so hole-to-copper is 0.2 at the 0.1 mm routing clearance (0.37 and
    # 0.38 pads left P4 fanout nets unrouted; route.py narrows a track where the router's octagons leave a few um
    # short; a 0.115 clearance for the Power class alone left nets unrouted too), via hole-to-hole 0.2,
    # copper-to-edge 0.2, mask dam 0.1 (mask opening 1:1, so expansion 0), 0.076 mm (JLCPCB: "3 mil in BGA fan-outs") for the
    # router's necks inside the P4 courtyard. Conditional rules in the .kicad_dru: PTH hole to copper 0.28, ring 0.15,
    # hole-to-hole 0.45; 0.15 pad-to-pad gap; the USB-C footprint's own NPTH pegs sit 0.194 from its GND pads.
    Path(out).with_suffix(".kicad_pro").write_text(json.dumps({
        "meta": {"filename": Path(out).with_suffix(".kicad_pro").name, "version": 3},
        "board": {"design_settings": {"rules": {
            "min_clearance": 0.09, "min_track_width": 0.09, "min_via_diameter": 0.25, "min_through_hole_diameter": 0.15,
            "min_hole_clearance": 0.2, "min_via_annular_width": 0.05, "min_copper_edge_clearance": 0.2, "min_hole_to_hole": 0.2,
            "solder_mask_clearance": 0.0, "solder_mask_min_width": 0.1, "solder_mask_to_copper_clearance": 0.0}}},
        "net_settings": {"meta": {"version": 4}, "classes": [
            netclass("Default", 0.1, 2147483647), netclass("Power", POWER_WIDTH, 0)],
            "netclass_patterns": [{"netclass": "Power", "pattern": n} for n in POWER]}}, indent=2))
    Path(out).with_suffix(".kicad_dru").write_text('(version 1)\n'
        '(rule "JLCPCB PTH hole to copper" (constraint hole_clearance (min 0.28mm))'
        ' (condition "A.Pad_Type == \'Through-hole\' || B.Pad_Type == \'Through-hole\'"))\n'  # both ways: the zone filler asks with A = zone
        '(rule "JLCPCB PTH annular ring" (constraint annular_width (min 0.15mm)) (condition "A.Pad_Type == \'Through-hole\'"))\n'
        '(rule "JLCPCB PTH hole to hole" (constraint hole_to_hole (min 0.45mm))'
        ' (condition "A.Pad_Type == \'Through-hole\' || B.Pad_Type == \'Through-hole\'"))\n'
        '(rule "JLCPCB SMD pad to pad" (constraint clearance (min 0.15mm)) (condition "A.Type == \'Pad\' && B.Type == \'Pad\'"))\n'
        '(rule "footprint-internal NPTH" (constraint hole_clearance (min 0.19mm)) (condition "A.Reference == B.Reference"))\n'
        '(rule "JLCPCB 3 mil in BGA fan-outs" (constraint track_width (min 0.076mm)) (condition "A.insideCourtyard(\'U1\')"))\n')
    area = side * side - sum((k[2] - k[0]) * (k[3] - k[1]) for k in keepouts)
    used = {s: sum((r[2] - r[0]) * (r[3] - r[1]) for r in placed[s][len(keepouts):]) for s in placed}
    print(f"{Path(out).name}: {len(fps)} parts, board {side} mm, {layers} layers")
    print(f"  courtyard area used: top {used['top']:.0f} / {area:.0f} mm2, bottom {used['bottom']:.0f} / {area:.0f} mm2")
    print(f"  did not fit ({len(unplaced)}): {' '.join(f'{r}({fps[r].GetValue()})' for r in unplaced) or 'none'}")
    return unplaced


if __name__ == "__main__":
    netlist, out, side, holes = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    sys.exit(1 if main(netlist, out, side, holes, int(sys.argv[5]) if len(sys.argv) > 5 else 4) else 0)
