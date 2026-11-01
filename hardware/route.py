"""Autoroute a placed board with Freerouting: Specctra DSN out, SES back in, zones refilled.

Runs under KiCad's own Python (pcbnew), after pcb.py:
    route.py out/fc_board.kicad_pcb [passes]
Needs Java 25+ (brew install openjdk) and hardware/tools/freerouting-2.5.0.jar (see DESIGN.md).
Writes out/<board>-routed.kicad_pcb; the placed board is left as it is.
"""
import collections
import json
import re
import subprocess
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
JAVA = "/opt/homebrew/opt/openjdk/bin/java"
JAR = HERE / "tools/freerouting-2.5.0.jar"
KICAD_CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"


def pour(board, layer, plane):
    """GND pour over the whole board on `layer`, shaped like the In1 `plane`, unconnected islands removed."""
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNet(plane.GetNet())
    zone.SetLocalClearance(plane.GetLocalClearance())
    zone.SetMinThickness(plane.GetMinThickness())
    zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)  # thermal spokes do not fit between 0201/0.4 mm pads
    zone.SetAssignedPriority(1)
    zone.Outline().Append(plane.Outline())
    board.Add(zone)


def clusters(board, gnd):
    """GND copper as a graph: pads, tracks and vias by KiCad's connectivity, plus every pour island (zone, index)
    joined to the items whose centre or end lies on it. Returns (reachable keys from the biggest In1 island,
    islands as [(key, zone, polys, index)])."""
    conn = board.GetConnectivity()
    items = {it.m_Uuid.AsString(): it for f in board.GetFootprints() for it in f.Pads() if it.GetNetCode() == gnd}
    items.update((t.m_Uuid.AsString(), t) for t in board.GetTracks() if t.GetNetCode() == gnd)
    adj = collections.defaultdict(set)
    for k, it in items.items():
        adj[k] = {n.m_Uuid.AsString() for n in conn.GetConnectedItems(it) if n.m_Uuid.AsString() in items}
    islands = []
    for z in board.Zones():
        if z.GetNetCode() != gnd:
            continue
        poly = z.GetFilledPolysList(z.GetLayer())
        for i in range(poly.OutlineCount()):
            key = (z.m_Uuid.AsString(), i)
            islands.append((key, z, poly, i))
            for k, it in items.items():
                ends = [it.GetPosition()] + ([it.GetEnd()] if it.GetClass() == "PCB_TRACK" else [])
                if it.IsOnLayer(z.GetLayer()) and any(poly.Contains(p, i) for p in ends):
                    adj[k].add(key)
                    adj[key].add(k)
    main = max((x for x in islands if x[1].GetLayer() == pcbnew.In1_Cu), key=lambda x: x[2].Outline(x[3]).Area())[0]
    seen, todo = {main}, [main]
    while todo:
        k = todo.pop()
        for n in adj[k]:
            if n not in seen:
                seen.add(n)
                todo.append(n)
    return seen, islands


def inside(poly, i, pt, r):
    """pt sits in island i of poly with r to spare."""
    return (poly.Contains(pt, i) and poly.Outline(i).Distance(pt, True) >= r
            and all(poly.Hole(i, j).Distance(pt, True) >= r for j in range(poly.HoleCount(i))))


def stitch(board, plane):
    """Freerouting takes the In1 plane for solid copper, but signal tracks on In1 cut the real fill into pieces, so
    GND pads whose via lands on a cut-off piece (and the pour islands round them) end up off the net. For every
    such island, one GND via where it overlaps a connected island on another layer, clear of all other copper.
    Call with the pours filled; refills them."""
    gnd, ref = plane.GetNetCode(), next(t for t in board.GetTracks() if t.GetClass() == "PCB_VIA")
    r = ref.GetWidth(pcbnew.F_Cu) // 2 + pcbnew.FromMM(0.12)  # via pad + clearance + margin
    others = [o for f in board.GetFootprints() for o in f.Pads() if o.GetNetCode() != gnd]
    others += [t for t in board.GetTracks() if t.GetNetCode() != gnd]
    for _ in range(3):  # ponytail: a via can join two strays at once; 3 rounds covered every board so far
        seen, islands = clusters(board, gnd)
        stray = [x for x in islands if x[0] not in seen]
        if not stray:
            return
        for key, zone, poly, i in stray:
            joined = [x for x in islands if x[0] in seen and x[1].GetLayer() != zone.GetLayer()]
            bb, step = poly.Outline(i).BBox(), pcbnew.FromMM(0.1)
            pt = next((p for y in range(bb.GetY(), bb.GetBottom(), step) for x in range(bb.GetX(), bb.GetRight(), step)
                       for p in [pcbnew.VECTOR2I(x, y)]
                       if inside(poly, i, p, r) and any(inside(jp, ji, p, r) for _, _, jp, ji in joined)
                       and not any(o.HitTest(p, r) for o in others)), None)
            if pt is None:  # a via into a neighbouring stray island may still join this one
                continue
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pt)
            via.SetNetCode(gnd)
            via.SetDrill(ref.GetDrill())
            via.SetWidth(ref.GetWidth(pcbnew.F_Cu))
            board.Add(via)
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    seen, islands = clusters(board, gnd)
    for key, zone, poly, i in islands:
        if key not in seen:
            bb = poly.Outline(i).BBox()
            print(f"stitch: no via spot for the GND island on {board.GetLayerName(zone.GetLayer())} at "
                  f"{pcbnew.ToMM(bb.GetX()):.1f}, {pcbnew.ToMM(bb.GetY()):.1f} mm (DRC decides)")


def sexp(text):
    """Parse the DSN into nested lists; tokens keep their quotes."""
    tokens = re.findall(r'"[^"]*"|[()]|[^\s()"]+', text)
    stack, out = [], []
    for t in tokens:
        if t == "(":
            stack.append(out)
            out = []
        elif t == ")":
            done, out = out, stack.pop()
            out.append(done)
        else:
            out.append(t)
    return out[0]


def unsexp(node):
    return "(" + " ".join(unsexp(c) if isinstance(c, list) else c for c in node) + ")" if isinstance(node, list) else node


def canonical(text, variant=0):
    """KiCad's DSN exporter orders components, images, padstacks and nets by their random UUIDs, so the same board
    routes differently from run to run (Freerouting itself is deterministic per DSN: six runs, six identical
    results). Sort everything by name; `variant` reverses nets (1), components (2) or both (3) for a different but
    repeatable routing order on a retry. Through-hole padstacks ([A]) grow by 0.16 mm for the router only: JLCPCB
    wants 0.28 mm from a plated hole to copper and the 1.0 mm header pads have a 0.175 mm ring."""
    mark = "<<quote>>"
    pcb = sexp(text.replace('(string_quote ")', f"(string_quote {mark})"))
    pcb[1] = "board.dsn"  # the path KiCad put here differs between the Makefile and the tests
    name = lambda n: n[1] if isinstance(n, list) and len(n) > 1 else ""
    for section in pcb:
        if not isinstance(section, list):
            continue
        head = section[0]
        if head == "library":
            for node in section[1:]:
                if node[0] == "padstack" and "[A]" in node[1]:
                    for geo in (shape[1] for shape in node[2:] if shape[0] == "shape"):
                        if geo[0] in ("circle", "path"):
                            geo[2] = str(int(float(geo[2])) + 160)
                        elif geo[0] == "rect":
                            geo[2:6] = [str(int(float(v)) + d) for v, d in zip(geo[2:6], (-80, -80, 80, 80))]
        if head in ("placement", "library", "network"):
            section[1:] = sorted(section[1:], key=lambda n: (str(n[0]), name(n)) if isinstance(n, list) else ("", str(n)),
                                 reverse=head == "network" and variant in (1, 3))
        if head == "placement":
            for comp in section[1:]:
                comp[2:] = sorted(comp[2:], key=name, reverse=variant in (2, 3))
        if head == "network":
            for net in section[1:]:
                if net[0] == "net":
                    for pins in net[2:]:
                        if isinstance(pins, list) and pins[0] == "pins":
                            pins[1:] = sorted(pins[1:])
                elif net[0] == "class":
                    nets = [c for c in net[2:] if not isinstance(c, list)]
                    rest = [c for c in net[2:] if isinstance(c, list)]
                    net[2:] = sorted(nets) + rest
    return unsexp(pcb).replace(mark, '"')


def main(pcb, passes):
    pcb = Path(pcb)
    dsn, ses, out = pcb.with_suffix(".dsn"), pcb.with_suffix(".ses"), pcb.with_name(f"{pcb.stem}-routed.kicad_pcb")
    board = pcbnew.LoadBoard(str(pcb))
    plane = next(z for z in board.Zones() if z.GetNetname() == "GND")  # In1: Freerouting fans GND pads out to it
    plane.UnFill()
    plane.SetLocalClearance(pcbnew.FromMM(0.2))  # ponytail: empirical; at 0.1 the router left 2 FC nets unrouted
    if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
        sys.exit("DSN export failed")
    # Route at 0.1 mm against the 0.09 mm rule: Freerouting models oval pad ends as octagons (the P4's 0.1 mm radius
    # ends come out up to 7.6 um short), so it needs the margin for KiCad's exact check to pass.
    # In1 stays a `signal` layer: declared `power` (no signal tracks through the plane) the router left 73 nets
    # unrouted on the FC board.
    # Typed clearances such as `(clearance 115 (type wire_via))` are accepted but leave dozens of FC nets unrouted,
    # and so did 0.115 for the Power class alone; finish() narrows tracks instead where the margin is missing.
    exported = dsn.read_text().replace("(clearance 90)", "(clearance 100)")
    # Each attempt is a different but repeatable element order (see canonical()); an attempt counts only when the
    # finished board passes KiCad's copper DRC (the router's octagons can leave a hole clearance a few um short).
    for attempt in range(4):
        dsn.write_text(canonical(exported, attempt))
        run = subprocess.run([JAVA, "-jar", str(JAR), "-de", str(dsn), "-do", str(ses),
                              f"--router.autorouter.max_passes={passes}", "--router.neck_width_um=90",
                              "--gui.enabled=false", "--usage_and_diagnostic_data.disable_analytics=true"],
                             capture_output=True, text=True, check=True)
        print(run.stdout, run.stderr)
        left = re.search(r"Auto-routing stage completed.*?\((\d+) unrouted", run.stdout + run.stderr)
        if not left or left[1] != "0":
            print(f"attempt {attempt + 1}: {left[1] if left else '?'} unrouted")
            continue
        report = finish(pcbnew.LoadBoard(str(pcb)), ses, out, pcb)  # a fresh copy: the plane at its fill clearance
        bad = [v["description"] for v in report["violations"] if v["type"] in COPPER]
        print(f"attempt {attempt + 1}: routed, {len(bad)} copper DRC errors, {len(report['unconnected_items'])} unconnected")
        if not bad and not report["unconnected_items"]:
            break
        print("\n".join(f"{d} at {v['items'][0]['pos']}" for v, d in
                        ((v, v["description"]) for v in report["violations"] if v["type"] in COPPER)))
    else:
        sys.exit("Freerouting gave no clean board in 4 runs")


# Everything JLCPCB's check would reject (test_boards.py checks the same set).
COPPER = {"clearance", "shorting_items", "hole_clearance", "hole_to_hole", "copper_edge_clearance", "annular_width",
          "track_width", "via_diameter", "drill_too_small", "solder_mask_bridge", "starved_thermal", "isolated_copper"}


def finish(board, ses, out, pcb):
    """Import the routed session, add the pours and stitch vias, widen the necks; save `out` and return its DRC."""
    if not pcbnew.ImportSpecctraSES(board, str(ses)):
        sys.exit("SES import failed")
    plane = next(z for z in board.Zones() if z.GetNetname() == "GND")
    # Freerouting takes the DSN plane for solid copper, but the via field and the signal tracks on In1 cut the
    # real fill into pieces, so GND stubs it counted as "on the plane" end up on nothing. GND pours on the outer
    # layers, added only now (in the DSN they made the router skip GND pads sitting on them), join every stub to
    # GND copper nearby; islands that reach nothing are removed.
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        pour(board, layer, plane)
    necked = set()  # Freerouting's fanout necks into pads at 0.075 mm; widen to the rules' 0.09 minimum where it fits
    tracks = {t.m_Uuid.AsString(): t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"}
    for uuid, t in tracks.items():
        if t.GetWidth() < pcbnew.FromMM(0.1):  # incl. the router's 0.09 retries
            necked.add(uuid)
            t.SetWidth(pcbnew.FromMM(0.09))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    stitch(board, plane)
    pcbnew.SaveBoard(str(out), board)
    for ext in (".kicad_pro", ".kicad_dru"):  # same rules
        out.with_suffix(ext).write_text(pcb.with_suffix(ext).read_text())
    # KiCad's own DRC narrows a track wherever it is a few um too close: the widened necks (back towards JLCPCB's
    # 3 mil, 0.076 mm, allowed in BGA fan-outs) and the router's octagonal corners next to a hole (0.09 floor).
    drc = out.with_suffix(".drc.json")
    for _ in range(6):
        subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "json", "-o", str(drc), str(out)], capture_output=True)
        report = json.loads(drc.read_text())
        fixed = False
        for v in report["violations"]:
            short = re.search(r"clearance ([\d.]+) mm; actual ([\d.]+) mm", v["description"])
            if v["type"] not in ("clearance", "hole_clearance") or not short:
                continue
            for item in v["items"]:
                t = tracks.get(item["uuid"])
                if t is None:
                    continue
                floor = pcbnew.FromMM(0.076 if item["uuid"] in necked else 0.09)
                width = max(floor, t.GetWidth() - 2 * pcbnew.FromMM(float(short[1]) - float(short[2])) - 1000)
                if width < t.GetWidth():
                    t.SetWidth(width)
                    fixed = True
        if not fixed:
            break
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        pcbnew.SaveBoard(str(out), board)
    print(f"{out.name}: {len(tracks)} tracks, {sum(t.GetClass() == 'PCB_VIA' for t in board.GetTracks())} vias, "
          f"{len(report['unconnected_items'])} unconnected")
    return report

if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 20)
