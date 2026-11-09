"""Autoroute a placed board with Freerouting: Specctra DSN out, SES back in, zones refilled.

Runs under KiCad's own Python (pcbnew), after pcb.py:
    route.py out/fc_board.kicad_pcb [passes]
Needs Java 25+ (brew install openjdk) and hardware/tools/freerouting-2.5.0.jar (see DESIGN.md).
Writes out/<board>-routed.kicad_pcb; the placed board is left as it is.
"""
import collections
import concurrent.futures
import json
import os
import random
import re
import shlex
import subprocess
import sys
from pathlib import Path

import pcbnew

from pcb import POWER, POWER_WIDTH

HERE = Path(__file__).resolve().parent
JAVA = "/opt/homebrew/opt/openjdk/bin/java"
JAR = HERE / "tools/freerouting-2.5.0.jar"
KICAD_CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
# Routed first and locked, so the autorouter works round them: short, sensitive or high-current loops that must not
# lose out to whichever net got there first (XTAL_N was among the leftovers when routed with the rest).
CRITICAL = {"XTAL_P", "XTAL_N", "BB_L1", "BB_L2", "HP_SW"}
# An autorouter pass is single-threaded, so parallel runs scale with the core count.
JOBS = int(os.environ.get("ROUTE_JOBS", max(1, (os.cpu_count() or 2) - 2)))
LIMIT = int(os.environ.get("ROUTE_LIMIT", 600))  # seconds per run; slow runs (14 min seen) ended unrouted anyway
RUNS = int(os.environ.get("ROUTE_RUNS", 10))  # element orders to try
MAX_LEFT = 2  # Freerouting leftovers still worth KiCad's check, see main()
REMOTE = os.environ.get("ROUTE_REMOTE")  # "user@host port": run Freerouting there (Java + jar in ~), see DESIGN.md


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


def stitch_pads(board, plane):
    """A GND via within 1.2 mm of every SMD GND pad that has none. Freerouting leaves most GND pads to the outer
    pours, so a decoupling cap's return ran 3-8 mm through the pour to one of 17 GND vias before reaching the In1
    plane. A spot must lie on connected GND copper on the pad's layer and on In1, clear of every pad, other-net
    track and hole by the rules (plus margin), and outside rule areas. Refills the pours."""
    gnd, ref = plane.GetNetCode(), next(t for t in board.GetTracks() if t.GetClass() == "PCB_VIA")
    drill, r = ref.GetDrillValue(), ref.GetWidth(pcbnew.F_Cu) // 2 + pcbnew.FromMM(0.12)  # via pad + clearance + margin
    pads = [p for f in board.GetFootprints() for p in f.Pads()]
    blockers = pads + [t for t in board.GetTracks() if t.GetNetCode() != gnd]  # a via on any pad wicks solder
    # Minimum centre distance to another hole: JLCPCB 0.45 hole-to-hole and 0.28 hole-to-copper for plated holes,
    # 0.2 hole-to-hole between vias.
    holes = [(p.GetPosition(), max(pcbnew.FromMM(0.45) + (p.GetDrillSize().x + drill) // 2,
                                   pcbnew.FromMM(0.33) + drill // 2 + max(p.GetSize().x, p.GetSize().y) // 2))
             for p in pads if p.GetDrillSize().x]
    holes += [(v.GetPosition(), pcbnew.FromMM(0.2) + drill) for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    gnd_pts = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd]
    gnd_pts += [p.GetPosition() for p in pads if p.GetNetCode() == gnd and p.GetDrillSize().x]
    rule_areas = [z for z in board.Zones() if z.GetIsRuleArea()]
    seen, islands = clusters(board, gnd)
    joined = [x for x in islands if x[0] in seen]
    step, reach = pcbnew.FromMM(0.1), pcbnew.FromMM(1.2)
    spots = sorted(((dx, dy) for dx in range(-reach, reach + 1, step) for dy in range(-reach, reach + 1, step)),
                   key=lambda d: d[0] ** 2 + d[1] ** 2)
    added = 0
    for pad in pads:
        if pad.GetNetCode() != gnd or pad.GetDrillSize().x:
            continue
        c = pad.GetPosition()
        if any((c - q).EuclideanNorm() < reach for q in gnd_pts):
            continue
        layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        near = []
        for o in blockers:
            box = o.GetBoundingBox()
            box.Inflate(reach + r)
            if box.Contains(c):
                near.append(o)
        for dx, dy in spots:
            p = pcbnew.VECTOR2I(c.x + dx, c.y + dy)
            if (any(inside(jp, ji, p, r) for _, z, jp, ji in joined if z.GetLayer() == layer)
                    and any(inside(jp, ji, p, r) for _, z, jp, ji in joined if z.GetLayer() == pcbnew.In1_Cu)
                    and not any(o.HitTest(p, r) for o in near)
                    and all((p - h).EuclideanNorm() >= d for h, d in holes)
                    and not any(z.Outline().Contains(p) for z in rule_areas)):
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(p)
                via.SetNetCode(gnd)
                via.SetDrill(drill)
                via.SetWidth(ref.GetWidth(pcbnew.F_Cu))
                board.Add(via)
                gnd_pts.append(p)
                holes.append((p, pcbnew.FromMM(0.2) + drill))
                added += 1
                break
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    print(f"stitch_pads: {added} GND vias added")


def widen(board):
    """Power tracks back towards POWER_WIDTH where the copper round them allows. Freerouting necks a Power net for
    a whole run when it squeezes past pads (BB_L1, the buck-boost switch node, came back at 0.09 mm for 2.7 mm).
    Each narrow segment takes the widest step that keeps 0.11 mm from every other-net pad, track and via on its
    layer (0.13 from plated holes: JLCPCB's 0.28 hole-to-copper) and 0.2 from the edge; the pours refill round it."""
    power, full = set(POWER), pcbnew.FromMM(POWER_WIDTH)
    edge = board.GetBoardEdgesBoundingBox()
    edge.Inflate(-pcbnew.FromMM(0.2))
    items = [p for f in board.GetFootprints() for p in f.Pads()] + list(board.GetTracks())
    for t in board.GetTracks():
        if t.GetClass() != "PCB_TRACK" or t.GetNetname() not in power or t.GetWidth() >= full:
            continue
        box = t.GetBoundingBox()
        box.Inflate(full)
        near = [o for o in items if o.GetNetCode() != t.GetNetCode() and o.IsOnLayer(t.GetLayer())
                and o.GetBoundingBox().Intersects(box)]
        for w in (0.3, 0.25, 0.2, 0.15):
            if pcbnew.FromMM(w) <= t.GetWidth():
                break
            seg = pcbnew.SHAPE_SEGMENT(t.GetStart(), t.GetEnd(), pcbnew.FromMM(w))
            if edge.Contains(seg.BBox()) and not any(
                    seg.Collide(o.GetEffectiveShape(t.GetLayer()), pcbnew.FromMM(0.13 if o.GetClass() == "PAD" and o.GetDrillSize().x
                                                                     else 0.11)) for o in near):
                t.SetWidth(pcbnew.FromMM(w))
                break


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


def canonical(text, variant=0, keep=None):
    """KiCad's DSN exporter orders components, images, padstacks and nets by their random UUIDs, so the same board
    routes differently from run to run (Freerouting itself is deterministic per DSN: six runs, six identical
    results). Sort everything by name; `variant` reverses nets (1), components (2) or both (3) for a different but
    repeatable routing order on a retry. Through-hole padstacks ([A]) grow by 0.16 mm for the router only: JLCPCB
    wants 0.28 mm from a plated hole to copper and the 1.0 mm header pads have a 0.175 mm ring. With `keep`, only
    those nets stay connected; the pads of the others remain as obstacles."""
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
            if variant >= 4 and head != "library":  # retries past the four orders: a seeded, repeatable shuffle
                section[1:] = sorted(section[1:], key=lambda n: random.Random(f"{variant}:{name(n)}").random())
        if head == "placement":
            for comp in section[1:]:
                comp[2:] = sorted(comp[2:], key=name, reverse=variant in (2, 3))
        if head == "network":
            for net in section[1:]:
                if net[0] == "net":
                    for pins in net[2:]:
                        if isinstance(pins, list) and pins[0] == "pins":
                            pins[1:] = sorted(pins[1:]) if keep is None or net[1].strip('"') in keep else []
                elif net[0] == "class":
                    nets = [c for c in net[2:] if not isinstance(c, list)]
                    rest = [c for c in net[2:] if isinstance(c, list)]
                    net[2:] = sorted(nets) + rest
    return unsexp(pcb).replace(mark, '"')


def freeroute(dsn, ses, passes):
    """One Freerouting run, here or on REMOTE. Returns (unrouted count, None if it hit LIMIT or gave no count; log)."""
    args = ["-de", dsn.name, "-do", ses.name, f"--router.autorouter.max_passes={passes}", "--router.neck_width_um=90",
            "--gui.enabled=false", "--usage_and_diagnostic_data.disable_analytics=true"]
    if REMOTE:
        host, port = REMOTE.split()
        opts = ["-i", str(Path.home() / ".runpod/ssh/runpodctl-ssh-key"), "-o", "StrictHostKeyChecking=accept-new"]
        subprocess.run(["scp", "-q", *opts, "-P", port, str(dsn), f"{host}:"], check=True)
        cmd = ["ssh", *opts, "-p", port, host, f"timeout {LIMIT} java -jar {JAR.name} " + shlex.join(args)]
    else:
        cmd = [JAVA, "-jar", str(JAR), *args]
    try:
        run = subprocess.run(cmd, cwd=dsn.parent, capture_output=True, text=True, timeout=LIMIT + 30)
    except subprocess.TimeoutExpired:
        return None, f"{dsn.name}: over {LIMIT} s"
    log = run.stdout + run.stderr
    left = re.search(r"Auto-routing stage completed.*?\((\d+) unrouted", log)
    if REMOTE and left:
        subprocess.run(["scp", "-q", *opts, "-P", port, f"{host}:{ses.name}", str(ses)], check=True)
    return (int(left[1]) if left else None), log


def export(board, dsn):
    """Specctra DSN of `board`, Freerouting's plane and spacing margins applied (the board is not saved after this)."""
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
    return dsn.read_text().replace("(clearance 90)", "(clearance 100)")


def preroute(pcb, dsn, ses, passes):
    """Route CRITICAL alone, lock it and save <board>-pre.kicad_pcb, which the full runs start from. Boards without
    those nets (the radio) get the placed board unchanged."""
    board = pcbnew.LoadBoard(str(pcb))
    pre = pcb.with_name(f"{pcb.stem}-pre.kicad_pcb")
    if CRITICAL & {n.GetNetname() for n in board.GetNetInfo().NetsByName().values()}:
        dsn.write_text(canonical(export(pcbnew.LoadBoard(str(pcb)), dsn), keep=CRITICAL))
        left, log = freeroute(dsn, ses, passes)
        print(log)
        if left != 0:
            sys.exit(f"critical nets: {left} unrouted")
        if not pcbnew.ImportSpecctraSES(board, str(ses)):
            sys.exit("SES import failed")
        for t in board.GetTracks():
            t.SetLocked(True)
        print("preroute: " + ", ".join(f"{n} {pcbnew.ToMM(sum(t.GetLength() for t in board.GetTracks() if t.GetNetname() == n)):.1f} mm"
                                       for n in sorted(CRITICAL)))
    pcbnew.SaveBoard(str(pre), board)
    for ext in (".kicad_pro", ".kicad_dru"):  # without the .kicad_dru the pours fill to the wrong hole clearance
        pre.with_suffix(ext).write_text(pcb.with_suffix(ext).read_text())
    return pre


def save_route(board, path):
    """Every track and via as one JSON row per line, duplicates dropped: the routed copper as a source file."""
    rows = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            row = ["V", t.GetNetname(), p.x, p.y, t.GetWidth(pcbnew.F_Cu), t.GetDrillValue()]
        else:
            a, b = t.GetStart(), t.GetEnd()
            row = ["T", t.GetNetname(), board.GetLayerName(t.GetLayer()), a.x, a.y, b.x, b.y, t.GetWidth()]
        rows[json.dumps(row)] = None
    path.parent.mkdir(exist_ok=True)
    path.write_text("[\n" + ",\n".join(sorted(rows)) + "\n]\n")


def load_route(board, path):
    """Add save_route()'s copper to a placed board."""
    for kind, net, *r in json.loads(path.read_text()):
        if board.FindNet(net) is None:
            sys.exit(f"{path.name}: net {net} is not on the board")
        if kind == "V":
            item = pcbnew.PCB_VIA(board)
            item.SetPosition(pcbnew.VECTOR2I(r[0], r[1]))
            item.SetWidth(r[2])
            item.SetDrill(r[3])
        else:
            item = pcbnew.PCB_TRACK(board)
            item.SetLayer(board.GetLayerID(r[0]))
            item.SetStart(pcbnew.VECTOR2I(r[1], r[2]))
            item.SetEnd(pcbnew.VECTOR2I(r[3], r[4]))
            item.SetWidth(r[5])
        item.SetNet(board.FindNet(net))
        board.Add(item)


def clean(report):
    bad = [v for v in report["violations"] if v["type"] in COPPER]
    print(f"{len(bad)} copper DRC errors, {len(report['unconnected_items'])} unconnected")
    print("\n".join(f"{v['description']} at {v['items'][0]['pos']}" for v in bad))
    return not bad and not report["unconnected_items"]


def main(pcb, passes):
    pcb = Path(pcb)
    dsn, ses, out = pcb.with_suffix(".dsn"), pcb.with_suffix(".ses"), pcb.with_name(f"{pcb.stem}-routed.kicad_pcb")
    # Freerouting's result changes with the JVM and CPU (order 9 of the FC board: 1 unrouted on Linux/Java 25, 5 on
    # macOS/Java 27), so a clean route is kept in routes/ and only the DRC gate below decides whether it still fits.
    # A netlist or placement change fails that gate and routes afresh. Delete the file to force a re-route.
    pin = HERE / "routes" / f"{pcb.stem}.json"
    if pin.exists():
        board = pcbnew.LoadBoard(str(pcb))
        load_route(board, pin)
        if clean(finish(board, out, pcb)):
            return
        print(f"{pin.name} no longer fits, routing afresh")
    pre = preroute(pcb, dsn, ses, passes)
    exported = export(pcbnew.LoadBoard(str(pre)), dsn)
    # Each run is a different but repeatable element order (see canonical()). The lowest clean order wins, so the
    # result does not depend on which job finished first; a run counts only when the finished board passes KiCad's
    # copper DRC (the router's octagons can leave a hole clearance a few um short).
    def attempt(variant):
        d, s = pcb.with_name(f"{pcb.stem}-{variant}.dsn"), pcb.with_name(f"{pcb.stem}-{variant}.ses")
        d.write_text(canonical(exported, variant))
        left, log = freeroute(d, s, passes)
        print(f"order {variant}: {'over the limit' if left is None else f'{left} unrouted'}", flush=True)
        return variant, left, s, log
    with concurrent.futures.ThreadPoolExecutor(JOBS) as pool:
        runs = list(pool.map(attempt, range(RUNS)))
    for variant, left, s, log in runs:
        # Freerouting's count is not the gate: it can report a GND connection open that the In1 plane and the pours
        # added in finish() close (order 9 of the FC board: 1 "unrouted", 0 unconnected in KiCad). KiCad's DRC decides.
        if left is None or left > MAX_LEFT:
            continue
        print(log)
        board = pcbnew.LoadBoard(str(pre))  # a fresh copy: the plane at its fill clearance
        if not pcbnew.ImportSpecctraSES(board, str(s)):
            sys.exit("SES import failed")
        save_route(board, pin.with_suffix(".tmp"))
        print(f"order {variant}: routed, ", end="")
        if clean(finish(board, out, pcb)):
            pin.with_suffix(".tmp").replace(pin)
            return
    pin.with_suffix(".tmp").unlink(missing_ok=True)
    sys.exit(f"Freerouting gave no clean board in {RUNS} runs")


# Everything JLCPCB's check would reject (test_boards.py checks the same set).
COPPER = {"clearance", "shorting_items", "hole_clearance", "hole_to_hole", "copper_edge_clearance", "annular_width",
          "track_width", "via_diameter", "drill_too_small", "solder_mask_bridge", "starved_thermal", "isolated_copper"}


def finish(board, out, pcb):
    """Add the pours and stitch vias to the routed board, widen the necks; save `out` and return its DRC."""
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
    widen(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    stitch(board, plane)
    stitch_pads(board, plane)
    pcbnew.SaveBoard(str(out), board)
    for ext in (".kicad_pro", ".kicad_dru"):  # same rules
        out.with_suffix(ext).write_text(pcb.with_suffix(ext).read_text())
    # KiCad's own DRC narrows a track wherever it is a few um too close: the widened necks (back towards JLCPCB's
    # 3 mil, 0.076 mm, allowed in BGA fan-outs) and the router's octagonal corners next to a hole (0.09 floor).
    drc = out.with_suffix(".drc.json")
    u1 = next(f for f in board.GetFootprints() if f.GetReference() == "U1").GetCourtyard(pcbnew.F_Cu).BBox()
    for _ in range(12):  # the re-widened Power tracks can take a few rounds between fine-pitch pads
        subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "json", "-o", str(drc), str(out)], capture_output=True)
        report = json.loads(drc.read_text())
        fixed, drop = False, set()
        for v in report["violations"]:
            short = re.search(r"clearance ([\d.]+) mm; actual ([\d.]+) mm", v["description"])
            if v["type"] not in ("clearance", "hole_clearance", "copper_edge_clearance") or not short:
                continue
            for item in v["items"]:
                t = tracks.get(item["uuid"])
                if t is None:
                    continue
                fanout = u1.Contains(t.GetStart()) and u1.Contains(t.GetEnd())  # the 0.076 rule holds inside U1 only
                floor = pcbnew.FromMM(0.076 if item["uuid"] in necked and fanout else 0.09)
                width = max(floor, t.GetWidth() - 2 * pcbnew.FromMM(float(short[1]) - float(short[2])) - 1000)
                if width < t.GetWidth():
                    t.SetWidth(width)
                    fixed = True
                elif t.GetNetname() == "GND" and not t.IsLocked():
                    # At the floor and still too close: the pours carry GND now, and the next DRC round reports an
                    # open if this stub was really needed.
                    drop.add(item["uuid"])
        if not fixed and not drop:
            break
        if drop:  # edited in the file: board.Remove() crashed pcbnew's Python on the next GetTracks()
            pcbnew.SaveBoard(str(out), board)
            text = out.read_text()
            for uuid in drop:
                text = re.sub(rf'\(segment\b(?:(?!\(segment\b).)*?\(uuid "{uuid}"\)\s*\)\s*', "", text, count=1, flags=re.S)
            out.write_text(text)
            board = pcbnew.LoadBoard(str(out))
            tracks = {t.m_Uuid.AsString(): t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"}
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        pcbnew.SaveBoard(str(out), board)
    print(f"{out.name}: {len(tracks)} tracks, {sum(t.GetClass() == 'PCB_VIA' for t in board.GetTracks())} vias, "
          f"{len(report['unconnected_items'])} unconnected")
    return report

if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 20)
