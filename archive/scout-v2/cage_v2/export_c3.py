"""Cage v3 / v4 (layout C3) print set (v4 17 Oct 22:10: every rod and pin 3.0, wheels R 5; names from VER): plates per material (each part on its bed face), the assembly picture, the rod cut list.
Parts from node_gen.py (STLs in drone coordinates): corner_node x4 (PETG), vtee x2 (TPU, nose + tail, same part), side_node
x2 (TPU, + mirrored), crown x1 (PETG), wheels x8 (PETG, flat). Rods (C): lengths socket bottom to socket bottom.
Run: python3 export_c3.py <node dir> <out dir>"""
import sys, os, json, math, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(HERE))
import node_gen as NG, layout_v3 as L                      # before matplotlib: pymoto switches the backend on import
from print_check import to_bed
ND, OUT = sys.argv[1], sys.argv[2]; os.makedirs(OUT, exist_ok=True)
VER = os.environ.get("VER", "v4")
RD = NG.ROD                                                # rod diameters by role (layout_C3.json)
DENS = dict(PETG=1.27, TPU=1.26)
PARTS = [("corner_node", "+z", "PETG", 4), ("vtee", "+z", "PETG", 2), ("side_node", "-y", "PETG", 1), ("side_node_M", "+y", "PETG", 1),   # all PETG (Spark 17 Oct 10:39)
         ("crown", "+z", "PETG", 1), ("wheel", "+z", "PETG", 8), ("side_spacer", "+z", "PETG", 2)]

def load(name):
    if name == "wheel":                                    # r WR x 3.0, 0.5 edge chamfers (clears the filleted slot), bore pin + 0.4 (3.4 on the 3.0 pin)
        w = NG.mf.Manifold.hull(NG.cyl((0, 0, 0), (0, 0, 3.0), NG.WR - 0.5, seg=128) + NG.cyl((0, 0, 0.5), (0, 0, 2.5), NG.WR, seg=128))
        return NG.tm(w - NG.cyl((0, 0, -1), (0, 0, 4.0), RD["pin"] / 2 + 0.2, seg=64))
    if name == "side_spacer":                              # under the top case side pads (z 22..26): C3 has no side hubs (17 Oct 20:15)
        return NG.tm(NG.cyl((0, 0, 0), (0, 0, 4.0), 4.75, seg=96) - NG.cyl((0, 0, -1), (0, 0, 5.0), 1.65, seg=48))
    if name == "side_node_M":
        m = trimesh.load(f"{ND}/side_node.stl"); m.apply_transform(np.diag([1, -1, 1, 1.0])); return m
    return trimesh.load(f"{ND}/{name}.stl")

def put(m, build):
    m2, _ = to_bed(m, build); b = m2.bounds; m2.apply_translation([-(b[0, 0] + b[1, 0]) / 2, -(b[0, 1] + b[1, 1]) / 2, 0]); return m2

# ---- rods (C): socket bottom to socket bottom
o = NG.OPT; F, ZL, ZU = NG.F_, NG.ZL, NG.ZU
rods = []
for k in range(4):
    rods += [("lower ring %.1f" % RD["lower"], 189.0), ("upper ring %.1f" % RD["upper"], 189.0)]
R0 = np.array([F, 0, ZU])
for s, p in zip((1, -1), sorted(L.SPOKES["nose_V"]["starts"], key=lambda q: -q[1])):
    Pr = R0 + np.array([0, s * 3.2, 0]); a = np.array(p, float); u = NG.unit(a - Pr); rods.append(("spoke %.1f nose" % RD["spoke"], np.linalg.norm(a - (Pr + u * 3.5))))
for s, p in zip((1, -1), sorted(L.SPOKES["tail_V"]["starts"], key=lambda q: -q[1])):
    Pr = np.array([-F, s * 3.2, ZU]); a = np.array(p, float); u = NG.unit(a - Pr); rods.append(("spoke %.1f tail" % RD["spoke"], np.linalg.norm(a - (Pr + u * 3.5))))
st = np.array(L.SPOKES["side_single"]["starts"][0], float)
for sy in (1, -1):
    Pr = np.array([0, sy * F, ZU]); a = st * [1, sy, 1]; u = NG.unit(a - Pr); rods.append(("spoke %.1f side" % RD["spoke"], np.linalg.norm(a - (Pr + u * 3.5))))
q = o["bottom"]["quad"]
for sx in (1, -1):
    for sy in (1, -1):
        root = np.array([sx * q["x"], sy * L.RCV_Y, q["z"]]); Pr = np.array([sx * 3.2, sy * F, ZL]); u = NG.unit(root - Pr)
        rods.append(("skirt %.1f" % RD["skirt"], np.linalg.norm(root - (Pr + u * 3.5))))
rods += [("wheel pin %.1f" % RD["pin"], 2 * (1.5 + 0.4 + 2.1))] * 8       # through both cheeks (cheek 2.1, side gap 0.4)
cut = {}
for n, l in rods: cut.setdefault(n, []).append(round(float(l) - 0.5, 1))       # 0.5 short: epoxy room at the socket bottoms
tot = {}
for n, ls in cut.items(): tot[n.split()[-1] if n.startswith("wheel") else n.split()[1] if n.split()[1][0].isdigit() else n.split()[2]] = 0
print(json.dumps(cut, indent=1))

# ---- plates
rows = {}; info = {}
for name, build, mat, cnt in PARTS:
    m = load(name); rows[name] = put(m, build); info[name] = dict(mat=mat, n=cnt, g=round(abs(m.volume) * DENS[mat] / 1000, 2), build=build)
    rows[name].export(f"{OUT}/s2_cage_{name}_{VER}.stl")
for mat in ("PETG",):                                      # first-fit on a 1 mm grid by footprint (6 mm gaps, 8 mm margin)
    sc = trimesh.Scene(); occ = np.zeros((180, 180), bool); n = 0
    items = [(nm, i["n"]) for nm, i in info.items() if isinstance(i, dict) and i["mat"] == mat]
    for name, cnt in sorted(items, key=lambda it: -rows[it[0]].extents[0] * rows[it[0]].extents[1]):
        for i in range(cnt):
            m = rows[name].copy(); e = np.ceil(m.extents[:2]).astype(int) + 6; done = False
            for yy in range(8, 172 - e[1] + 7):
                for xx in range(8, 172 - e[0] + 7):
                    if not occ[xx:xx + e[0], yy:yy + e[1]].any():
                        occ[xx:xx + e[0], yy:yy + e[1]] = True; done = True; break
                if done: break
            assert done, f"{name} does not fit the {mat} plate"
            b = m.bounds; m.apply_translation([xx - b[0, 0], yy - b[0, 1], 0]); sc.add_geometry(m, node_name=f"{name}_{i + 1}"); n += 1
    lo, hi = sc.bounds; sc.apply_translation([90 - (lo[0] + hi[0]) / 2, 90 - (lo[1] + hi[1]) / 2, 0])
    sc.export(f"{OUT}/s2_cage_{mat}_plate_{VER}.3mf")
    info[f"{mat}_plate_g"] = round(sum(abs(g.volume) for g in sc.dump()) * DENS[mat] / 1000, 2)
carbon = sum(math.pi / 4 * float(n_.split()[2 if n_.startswith("wheel") else 1] if False else 0) for n_ in [])
rod_g = 0.0
for n, ls in cut.items():
    d = float(n.split()[-1]) if n.startswith("wheel") else float(n.split()[2] if n.startswith(("lower", "upper")) else n.split()[1])
    rod_g += sum(ls) * math.pi / 4 * d * d * 1.55e-3
info["rods_g"] = round(rod_g, 2)
info["printed_g"] = round(sum(v["g"] * v["n"] for k, v in info.items() if isinstance(v, dict)), 2)
info["total_g"] = round(info["printed_g"] + rod_g, 1)
json.dump(dict(parts=info, cut=cut), open(f"{OUT}/s2_cage_{VER}_summary.json", "w"), indent=1)
print(json.dumps(info, indent=1))
