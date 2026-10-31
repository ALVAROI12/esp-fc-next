"""Autoroute a placed board with Freerouting: Specctra DSN out, SES back in, zones refilled.

Runs under KiCad's own Python (pcbnew), after pcb.py:
    route.py out/fc_board.kicad_pcb [passes]
Needs Java 25+ (brew install openjdk) and hardware/tools/freerouting-2.5.0.jar (see DESIGN.md).
Writes out/<board>-routed.kicad_pcb; the placed board is left as it is.
"""
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
    repeatable routing order on a retry."""
    mark = "<<quote>>"
    pcb = sexp(text.replace('(string_quote ")', f"(string_quote {mark})"))
    pcb[1] = "board.dsn"  # the path KiCad put here differs between the Makefile and the tests
    name = lambda n: n[1] if isinstance(n, list) and len(n) > 1 else ""
    for section in pcb:
        if not isinstance(section, list):
            continue
        head = section[0]
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
    fill_clearance = plane.GetLocalClearance()
    plane.SetLocalClearance(pcbnew.FromMM(0.2))  # ponytail: empirical; at 0.1 the router left 2 FC nets unrouted
    if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
        sys.exit("DSN export failed")
    # Route at 0.1 mm against the 0.09 mm rule: Freerouting models oval pad ends as octagons (the P4's 0.1 mm radius
    # ends come out up to 7.6 um short), so it needs the margin for KiCad's exact check to pass.
    # In1 stays a `signal` layer: declared `power` (no signal tracks through the plane) the router left 73 nets
    # unrouted on the FC board.
    exported = dsn.read_text().replace("(clearance 90)", "(clearance 100)")
    # Each attempt is a different but repeatable element order (see canonical()).
    for attempt in range(4):
        dsn.write_text(canonical(exported, attempt))
        run = subprocess.run([JAVA, "-jar", str(JAR), "-de", str(dsn), "-do", str(ses),
                              f"--router.autorouter.max_passes={passes}", "--router.neck_width_um=90",
                              "--gui.enabled=false", "--usage_and_diagnostic_data.disable_analytics=true"],
                             capture_output=True, text=True, check=True)
        print(run.stdout, run.stderr)
        left = re.search(r"Auto-routing stage completed.*?\((\d+) unrouted", run.stdout + run.stderr)
        print(f"attempt {attempt + 1}: {left[1] if left else '?'} unrouted")
        if left and left[1] == "0":
            break
    else:
        sys.exit("Freerouting left nets unrouted in 4 runs")
    if not pcbnew.ImportSpecctraSES(board, str(ses)):
        sys.exit("SES import failed")
    plane.SetLocalClearance(fill_clearance)
    # Freerouting takes the DSN plane for solid copper, but the via field and the signal tracks on In1 cut the
    # real fill into pieces, so GND stubs it counted as "on the plane" end up on nothing. GND pours on the outer
    # layers, added only now (in the DSN they made the router skip GND pads sitting on them), join every stub to
    # GND copper nearby; islands that reach nothing are removed.
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        pour(board, layer, plane)
    necked = {}  # Freerouting's fanout necks into pads at 0.075 mm; widen to the rules' 0.09 minimum where it fits
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < pcbnew.FromMM(0.1):  # incl. the router's 0.09 retries
            necked[t.m_Uuid.AsString()] = t
            t.SetWidth(pcbnew.FromMM(0.09))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(out), board)
    out.with_suffix(".kicad_pro").write_text(pcb.with_suffix(".kicad_pro").read_text())  # same rules
    # KiCad's own DRC puts the neck width back wherever the wider track breaks clearance.
    drc = out.with_suffix(".drc.json")
    for _ in range(4):
        subprocess.run([KICAD_CLI, "pcb", "drc", "--format", "json", "-o", str(drc), str(out)], capture_output=True)
        report = json.loads(drc.read_text())
        bad = {i["uuid"] for v in report["violations"] if v["type"] == "clearance" for i in v["items"]} & necked.keys()
        if not bad:
            break
        for uuid in bad:
            necked.pop(uuid).SetWidth(pcbnew.FromMM(0.075))
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        pcbnew.SaveBoard(str(out), board)
    tracks = [t for t in board.GetTracks()]
    print(f"{out.name}: {sum(t.GetClass() == 'PCB_TRACK' for t in tracks)} tracks, "
          f"{sum(t.GetClass() == 'PCB_VIA' for t in tracks)} vias, {len(report['unconnected_items'])} unconnected")


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 20)
