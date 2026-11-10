"""Layout parasitics of the routed copper: DC resistance, inductance and length between pads, for the ngspice
checks (power_sim.py, rail_sim.py, link_si.py). Run with KiCad's Python after routing (make hw-pcb):

    $(KICAD_PY) parasitics.py        # out/<board>-routed.kicad_pcb -> routes/parasitics.json

Each net's tracks become a resistor network (tracks split where another track, via or pad lands on them), solved
between the pads of PATHS. The pads on each side are taken as shorted by the part they belong to.
Not modelled: the GND return (three pours, taken as ideal), current crowding in pads and corners, skin effect,
mutual inductance between parallel segments.
"""
import heapq
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "routes" / "parasitics.json"

# JLCPCB JLC04161H-7628 (jlcpcb.com/impedance): 1 oz outer, 0.5 oz inner, 7628 prepreg 0.2104 mm, Dk 4.4.
COPPER_MM = {"F.Cu": 0.035, "B.Cu": 0.035, "In1.Cu": 0.0152, "In2.Cu": 0.0152}
PREPREG_MM, DK = 0.2104, 4.4
RHO_20C, ALPHA, TEMP_C = 1.72e-8, 0.0039, 60.0  # ohm m, 1/K; copper temperature the resistance is quoted at
VIA_PLATING_MM, BOARD_MM = 0.018, 1.6
VIA_NH = 0.5  # ponytail: flat per-via inductance; a field solver if a via-heavy net ever matters

# name: (net, pads in, pads out). Refs are SKiDL's stable tags; a renumbered board fails loudly on a missing pad.
LINK = {f"link_{s.lower()}": (f"C5_{s}", ["U1.%d" % p], ["J8.%d" % j])
        for s, p, j in (("MOSI", 2, 5), ("MISO", 3, 6), ("SCLK", 4, 7), ("CS", 5, 8))}
PATHS = {
    "fc_board": {
        "vbat": ("VBAT", ["J2.1"], ["R1.1"]),  # ESC connector -> shunt
        "vbat_shunt": ("VBAT_SHUNT", ["R1.2"], ["Q1.1", "Q1.2", "Q1.5", "Q1.6", "Q1.7"]),  # shunt -> P-FET drain
        "vsys": ("VSYS", ["Q1.4", "Q1.8"], ["U3.1", "U3.10"]),  # P-FET source -> TPS63802 VIN
        "3v3_cout": ("3V3", ["U3.6"], ["C3.1", "C4.1"]),  # TPS63802 VOUT -> its two 22 uF
        "3v3_stack": ("3V3", ["U3.6"], ["J8.1", "J8.2"]),  # TPS63802 VOUT -> radio stack header
        **LINK,
    },
    "radio_board": {
        "3v3": ("3V3", ["J1.1", "J1.2"], ["U1.3"]),  # stack header -> C5 module supply pin
        "3v3_cin": ("3V3", ["U1.3"], ["C1.1"]),  # C5 supply pin -> its 22 uF
        "link_mosi": ("C5_MOSI", ["J1.5"], ["U1.20"]), "link_miso": ("C5_MISO", ["J1.6"], ["U1.21"]),
        "link_sclk": ("C5_SCLK", ["J1.7"], ["U1.10"]), "link_cs": ("C5_CS", ["J1.8"], ["U1.22"]),
    },
}


def ohm_per_mm(width_mm, layer):
    return RHO_20C * (1 + ALPHA * (TEMP_C - 20)) / (width_mm * COPPER_MM[layer] * 1e-6) * 1e-3


def nh_per_mm(width_mm, layer):
    """Track over the adjacent GND pour (IPC-2141 microstrip: L' = Z0 * tpd)."""
    return 0.2 * math.log(max(5.98 * PREPREG_MM / (0.8 * width_mm + COPPER_MM[layer]), 1.5))


def z0(width_mm, layer="F.Cu"):
    """IPC-2141 microstrip impedance, ohms; the inner layers are taken the same (one prepreg from a pour)."""
    return 87 / math.sqrt(DK + 1.41) * math.log(5.98 * PREPREG_MM / (0.8 * width_mm + COPPER_MM[layer]))


class Network:
    """Undirected graph of one net: edges carry (ohms, nH, mm)."""

    def __init__(self):
        self.parent, self.edges = {}, []

    def find(self, a):
        self.parent.setdefault(a, a)
        while self.parent[a] != a:
            self.parent[a] = a = self.parent[self.parent[a]]
        return a

    def union(self, a, b):
        self.parent[self.find(a)] = self.find(b)

    def solve(self, src, dst):
        """(ohms, nH, shortest mm) between two node groups; raises if the copper does not join them."""
        for group in (src, dst):
            for n in group[1:]:
                self.union(group[0], n)
        s, d = self.find(src[0]), self.find(dst[0])
        adj = defaultdict(list)
        for a, b, *w in self.edges:
            a, b = self.find(a), self.find(b)
            if a != b:
                adj[a].append((b, w))
                adj[b].append((a, w))
        dist, heap = {s: 0.0}, [(0.0, 0, s)]
        while heap:  # Dijkstra: path length, and the component reachable from the source
            length, _, n = heapq.heappop(heap)
            if length > dist[n]:
                continue
            for m, w in adj[n]:
                if length + w[2] < dist.get(m, math.inf):
                    dist[m] = length + w[2]
                    heapq.heappush(heap, (dist[m], len(dist), m))
        if d not in dist:
            raise ValueError("tracks do not connect the pads")
        index = {n: i for i, n in enumerate(k for k in dist if k != d)}  # dst is the reference node
        out = []
        for k in (0, 1):  # nodal analysis, 1 A in at src: once with ohms, once with nH as the branch "resistance"
            g, rhs = np.zeros((len(index), len(index))), np.zeros(len(index))
            for a in index:
                for b, w in adj[a]:
                    g[index[a], index[a]] += 1 / w[k]
                    if b != d:
                        g[index[a], index[b]] -= 1 / w[k]
            rhs[index[s]] = 1.0
            out.append(float(np.linalg.solve(g, rhs)[index[s]]))
        return out[0], out[1], dist[d]


def network(board, net):
    """Resistor network of one net and its pad nodes {"REF.num": node}."""
    g, segments, points = Network(), [], []  # points: (x, y, layer name, node)
    cu = [board.GetLayerName(layer) for layer in board.GetEnabledLayers().CuStack()]
    for t in board.GetTracks():
        if t.GetNetname() != net:
            continue
        if t.GetClass() == "PCB_VIA":
            x, y = t.GetPosition()
            barrel = (RHO_20C * (1 + ALPHA * (TEMP_C - 20)) * BOARD_MM * 1e-3
                      / (math.pi * pcbnew.ToMM(t.GetDrill()) * VIA_PLATING_MM * 1e-6))
            for layer in cu:  # star: every layer joins the via centre through half a barrel
                g.edges.append((("via", x, y), (x, y, layer), barrel / 2, VIA_NH / 2, BOARD_MM / 2))
                points.append((x, y, layer, (x, y, layer)))
        else:
            layer, (ax, ay), (bx, by) = board.GetLayerName(t.GetLayer()), t.GetStart(), t.GetEnd()
            segments.append((ax, ay, bx, by, layer, pcbnew.ToMM(t.GetWidth())))
            points += [(ax, ay, layer, (ax, ay, layer)), (bx, by, layer, (bx, by, layer))]
    pads = {}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() != net:
                continue
            name = f"{fp.GetReference()}.{pad.GetNumber()}"
            pads[name] = ("pad", name)
            for x, y, layer, node in points:  # a track end or via inside the pad's copper is on the pad
                if pad.IsOnLayer(board.GetLayerID(layer)) and pad.HitTest(pcbnew.VECTOR2I(int(x), int(y))):
                    g.union(node, pads[name])
    for ax, ay, bx, by, layer, width in segments:
        length = math.hypot(bx - ax, by - ay)
        on = []  # (position along the segment 0..1, node) of everything that touches it on this layer
        for x, y, player, node in points:
            if player != layer:
                continue
            t = 0.0 if length == 0 else max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / length**2))
            if math.hypot(ax + t * (bx - ax) - x, ay + t * (by - ay) - y) <= pcbnew.FromMM(width) / 2:
                on.append((t, node))
        on.sort(key=lambda p: p[0])
        for (t0, n0), (t1, n1) in zip(on, on[1:]):
            mm = pcbnew.ToMM(length) * (t1 - t0)
            if mm < 1e-4:
                g.union(n0, n1)
            else:
                g.edges.append((n0, n1, mm * ohm_per_mm(width, layer), mm * nh_per_mm(width, layer), mm))
    total = sum(pcbnew.ToMM(math.hypot(s[2] - s[0], s[3] - s[1])) for s in segments)
    return g, pads, total, min((s[5] for s in segments), default=0.0)


def extract():
    result = {}
    for name, paths in PATHS.items():
        board, rows = pcbnew.LoadBoard(str(HERE / "out" / f"{name}-routed.kicad_pcb")), {}
        for key, (net, src, dst) in paths.items():
            g, pads, total, narrowest = network(board, net)
            ohms, nh, mm = g.solve([pads[p] for p in src], [pads[p] for p in dst])
            # stub_mm: copper on the net that is not on the path (branches to other loads), for the link reflections.
            rows[key] = {"net": net, "from": src, "to": dst, "mohm": round(ohms * 1e3, 2), "nh": round(nh, 2),
                         "mm": round(mm, 2), "stub_mm": round(max(total - mm, 0.0), 2), "min_width_mm": narrowest}
        result[name] = rows
    return result


if __name__ == "__main__":
    import numpy as np
    import pcbnew

    data = {"assumptions": {"copper_mm": COPPER_MM, "prepreg_mm": PREPREG_MM, "dk": DK, "temp_c": TEMP_C,
                            "z0_ohm_0.1mm": round(z0(0.1), 1)}, **extract()}
    OUT.write_text(json.dumps(data, indent=1) + "\n")
    for board in PATHS:
        for key, row in data[board].items():
            print(f"{board:12} {key:11} {row['net']:10} {row['mohm']:7.2f} mOhm {row['nh']:6.2f} nH "
                  f"{row['mm']:6.2f} mm (net stub {row['stub_mm']} mm, narrowest {row['min_width_mm']} mm)")
    print(OUT)
