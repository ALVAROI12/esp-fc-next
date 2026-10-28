"""First-pass PCB from the SKiDL netlist: board outline, M2 holes, every footprint with its nets, greedy placement.
Not routed. It answers "does it fit", and gives layout a starting file.

Runs under KiCad's own Python (pcbnew):  <KiCad.app>/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
    pcb.py out/fc_board.net out/fc_board.kicad_pcb 25.5 20      (board side mm, hole spacing mm; 0 = no holes)
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
GAP, EDGE, STEP = 0.15, 0.3, 0.125 # courtyard spacing, keep-in from the edge, placement grid (mm)
# Fixed spots by value, (x, y, degrees) from the board centre. The stack header must sit at the same place on both
# boards: left edge, between the M2 holes. USB-C on the bottom edge, its "PCB Edge" mark (y=3.1) on the board edge.
FIXED = {"STACK": (-10.5, -7.0, 0), "USB_C_Receptacle_USB2.0_14P": (0, 12.75 - 3.1, 0)}
BOTTOM = {"radio_board": {"STACK"}}  # the radio board sits on top, its stack socket faces down


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


def main(netlist, out, side, holes):
    comps, nets = parse(netlist)
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
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

    # Modules with a keep-out (the C5 antenna) go first, antenna flush with the top edge, keep-out on both sides.
    for ref, fp in fps.items():
        if fp.Zones():
            angle = min((0, 90, 180, 270), key=lambda a: (fp.SetOrientationDegrees(a), zone_box(fp)[1])[1])
            fp.SetOrientationDegrees(angle)
            zx1, zy1, zx2, zy2 = zone_box(fp)
            cx1, cy1, cx2, cy2 = courtyard(fp)
            y = -half - zy1
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
    order = sorted((r for r in fps if r not in where), key=lambda r: (r not in first, -area(r)))
    for ref in order:
        fp = fps[ref]
        near = [where[r] for r in links[ref] if r in where]
        tx, ty = (sum(p[0] for p in near) / len(near), sum(p[1] for p in near) / len(near)) if near else (0, 0)
        if ref[0] == "J":  # connectors: as close to an edge as they fit
            spots = sorted(((x, y) for x in grid for y in grid), key=lambda p: (half - max(abs(p[0]), abs(p[1])),
                                                                                 math.hypot(p[0] - tx, p[1] - ty)))
        else:
            spots = sorted(((x, y) for x in grid for y in grid), key=lambda p: math.hypot(p[0] - tx, p[1] - ty))
        # The P4, crystal and IMU stay on top; everything else may go underneath.
        sides = ["top"] if ref in first else ["top", "bottom"]
        done = False
        for layer in sides:
            if (layer == "bottom") != fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            for angle in (0, 90):
                fp.SetOrientationDegrees(angle)
                cx1, cy1, cx2, cy2 = courtyard(fp)
                for x, y in spots:
                    rect = (x + cx1, y + cy1, x + cx2, y + cy2)
                    if all(free(rect, placed[s], half) for s in (["top", "bottom"] if through(fp) else [layer])):
                        put(ref, fp, layer, x, y, [rect])
                        done = True
                        break
                if done:
                    break
            if done:
                break
        if not done:
            if fp.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetPosition(pcbnew.VECTOR2I(mm(CENTER[0] + half + 5 + 12 * (len(unplaced) // 6)),
                                           mm(CENTER[1] - half + 5 * (len(unplaced) % 6))))
            unplaced.append(ref)

    for fp in fps.values():  # reference designators on the fab layer: a 25 mm board has no room for them on silk
        fp.Reference().SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)

    # GND: solid plane on In1 (reference for every signal on top), pours on the outer layers. In2 left for routing.
    # ponytail: no power planes; split In2 into VSYS/3V3 islands if the router can't carry the currents.
    for layer in (pcbnew.In1_Cu, pcbnew.F_Cu, pcbnew.B_Cu):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetNet(netinfo["GND"])
        zone.SetLocalClearance(mm(0.2))
        zone.SetMinThickness(mm(0.15))
        zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        zone.SetAssignedPriority(0 if layer == pcbnew.In1_Cu else 1)
        poly = zone.Outline()
        poly.NewOutline()
        for x, y in ((-half, -half), (half, -half), (half, half), (-half, half)):
            poly.Append(mm(CENTER[0] + x), mm(CENTER[1] + y))
        board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    pcbnew.SaveBoard(str(out), board)
    # Netclass rules live in the project file. ponytail: generic fine-pitch 4-layer values (0.1 mm track/space,
    # 0.3/0.15 mm vias); replace with the chosen fab's published limits before ordering.
    Path(out).with_suffix(".kicad_pro").write_text(json.dumps({
        "meta": {"filename": Path(out).with_suffix(".kicad_pro").name, "version": 3},
        "board": {"design_settings": {"rules": {
            "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.3, "min_through_hole_diameter": 0.15,
            "min_hole_clearance": 0.15, "min_copper_edge_clearance": 0.2, "min_hole_to_hole": 0.25}}},
        "net_settings": {"meta": {"version": 4}, "classes": [{
            "name": "Default", "clearance": 0.1, "track_width": 0.1, "via_diameter": 0.3, "via_drill": 0.15,
            "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.1, "diff_pair_gap": 0.1,
            "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
            "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647}]}}, indent=2))
    area = side * side - sum((k[2] - k[0]) * (k[3] - k[1]) for k in keepouts)
    used = {s: sum((r[2] - r[0]) * (r[3] - r[1]) for r in placed[s][len(keepouts):]) for s in placed}
    print(f"{Path(out).name}: {len(fps)} parts, board {side} mm, 4 layers")
    print(f"  courtyard area used: top {used['top']:.0f} / {area:.0f} mm2, bottom {used['bottom']:.0f} / {area:.0f} mm2")
    print(f"  did not fit ({len(unplaced)}): {' '.join(f'{r}({fps[r].GetValue()})' for r in unplaced) or 'none'}")
    return unplaced


if __name__ == "__main__":
    netlist, out, side, holes = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    sys.exit(1 if main(netlist, out, side, holes) else 0)
