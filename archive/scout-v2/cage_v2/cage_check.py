"""Assembly checks for S2 cage v2: prop keep-out, collar face clearance, wheel gaps, top case keep-out, masses, cut list.
17 Oct: prop band z 37.2..45.2 (prop plane 41.2), separate wheels on pins, no skids, TPU for AMS 1.26 g/cm3 (TDS)."""
import json, math, itertools, numpy as np, trimesh
from manifold3d import Manifold, Mesh
import cage_cad as cc
OUT = cc.OUT
def load(n): return trimesh.load(f"{OUT}/s2_cage_v2_{n}.stl")
def rot(k):
    T = np.eye(4); T[:3, :3] = cc.face_rot(k); return T
lay = json.load(open(f"{OUT}/cage_layout.json"))
parts = []
for k in range(4):
    m = load("corner_node").copy(); m.apply_transform(rot(k)); parts.append((f"corner_node@{k}", "corner_node", m))
W0 = load("wheel")
for i, (n, a, b, r) in enumerate(cc.pins()):
    a, b = np.array(a), np.array(b); d = (b - a) / np.linalg.norm(b - a)
    w = W0.copy(); w.apply_translation([0, 0, -cc.WW / 2]); T = trimesh.geometry.align_vectors([0, 0, 1], d); T[:3, 3] = (a + b) / 2
    w.apply_transform(T); parts.append((f"wheel #{i}", "wheel", w))
for k, name, f in ((0, "tee +y", "tee_node"), (1, "tee tail", "tee_node_tail"), (2, "tee -y", "tee_node")):
    m = load(f).copy(); m.apply_transform(rot(k)); parts.append((name, f, m))
parts.append(("side hub +y", "side_hub", load("side_hub")))
m = load("side_hub").copy(); m.apply_transform(np.diag([1, -1, 1, 1.0])); parts.append(("side hub -y", "side_hub", m))
parts.append(("tail hub", "tail_hub", load("tail_hub")))
for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
    m = load("foot_node").copy(); T = np.diag([sx, sy, 1.0, 1.0]); m.apply_transform(T)
    parts.append((f"foot {sx:+d}{sy:+d}", "foot_node", m))
# rods as meshes
for i, (n, a, b, r) in enumerate(lay["rods"]):
    a, b = np.array(a), np.array(b); d = b - a; L = np.linalg.norm(d)
    cyl = trimesh.creation.cylinder(radius=r, height=L, sections=32)
    T = trimesh.geometry.align_vectors([0, 0, 1], d / L); T[:3, 3] = (a + b) / 2; cyl.apply_transform(T)
    parts.append((f"rod {n} #{i}", "rod", cyl))
def disk_dist(P):
    best = np.full(len(P), 1e9)
    for sx, sy in itertools.product((1, -1), (1, -1)):
        rp = np.linalg.norm(P[:, :2] - [sx * 54.1, sy * 54.1], axis=1)
        dr = np.maximum(0, rp - 44.45); dz = np.maximum.reduce([np.zeros(len(P)), cc.PZ - 4.0 - P[:, 2], P[:, 2] - (cc.PZ + 4.0)])
        best = np.minimum(best, np.hypot(dr, dz))
    return best
planes = [cc.face_plane(k) for k in range(4)]
rows, ok = [], True
for name, kind, m in parts:
    V = np.asarray(m.vertices)
    dd = disk_dist(V).min()
    face = max(float((V @ n - d).max()) for n, d in planes)   # >0 = past the face, 0 = touching
    need_face = 0.0 if "wheel" in kind else (-3.0 if kind == "foot_node" else -1.0)
    f_ok = abs(face) < 0.02 if "wheel" in kind else face <= need_face + 0.05
    rows.append((name, round(dd, 2), round(face, 2), dd >= 12.0 - 0.02, f_ok))
    ok &= (dd >= 11.98) and f_ok
bad = [r for r in rows if not (r[3] and r[4])]
print("prop keep-out (min, need 12):", min(r[1] for r in rows), "| face: wheels touch, worst wheel", max(r[2] for r in rows if 'wheel' in r[0]),
      "| worst non-wheel", max(r[2] for r in rows if 'wheel' not in r[0]), "(need <= -1.0)")
for r in bad: print("FAIL", r)
# wheels vs their node (slot clearances), wheels on the face planes
node = cc.to_manifold(load("corner_node"))
for name, kind, m in parts[4:6]:
    print(name, "overlap with corner node mm3", round((node ^ cc.to_manifold(m)).volume(), 4))
# side hubs vs the S2 top case (its pad on the hub pad from z 26 up; parts inside |x| <= 33, |y| <= 37.5 above z 26)
lo, hi = np.array(cc.CASE_BOX[0]), np.array(cc.CASE_BOX[1])
for name, kind, m in parts:
    if kind in ("side_hub", "tail_hub"):
        V = np.asarray(m.vertices); inside = np.all((V > lo + np.array([0, 0, 0.1])) & (V < hi), axis=1)          # the case pad sits on the hub pad top (z 26)
        print(name, "top z", round(float(V[:, 2].max()), 2), "| inside the top case keep-out:", int(inside.sum()), "vertices")
        ok &= not inside.any()
# masses (solid, M): PETG 1.27, TPU 1.21 g/cm3; carbon rods 1.55 g/cm3
dens = dict(corner_node=1.27, wheel=1.27, tee_node=1.26, tee_node_tail=1.26, side_hub=1.27, tail_hub=1.27, foot_node=1.26)
mass = {}
for name, kind, m in parts:
    if kind == "rod": g = abs(m.volume) * 1.55e-3
    else: g = abs(m.volume) * dens[kind] * 1e-3
    key = kind if kind != "rod" else name.split(" #")[0]
    mass[key] = mass.get(key, 0) + g
tot = sum(mass.values())
print({k: round(v, 2) for k, v in mass.items()}, "total", round(tot, 1), "g (M, solid) + epoxy ~1.5 (E)")
lens = {}
for n, a, b, r in lay["rods"]:
    lens.setdefault(n, []).append(round(float(np.linalg.norm(np.array(b) - np.array(a))), 1))
print("rod cut lengths:", lens)
json.dump(dict(mass={k: round(v, 2) for k, v in mass.items()}, total=round(tot, 1), rods=lens,
               prop_min=min(r[1] for r in rows), face_worst_nonwheel=max(r[2] for r in rows if 'wheel' not in r[0]), ok=bool(ok)),
          open(f"{OUT}/cage_check.json", "w"))
# feet: lowest point, pack drop path
feet = [m for n_, k_, m in parts if k_ == "foot_node"]
low = min(float(m.vertices[:, 2].min()) for m in feet)
path = min(float(np.abs(m.vertices[m.vertices[:, 2] < -39.5][:, 0]).min()) for m in feet)
print("feet bottom z", round(low, 2), "| closest foot point to the pack path, |x|", round(path, 2), "(path |x| <= 23.6 at the mouth)")
# top case keep-outs (s2_top_case/geom_top.py): FC diamond L1 26.46 round (1.9, 0) z 22..38, FC USB lane, pack-lead lane
fc_min = 1e9
for n_, k_, m in parts:
    V = np.asarray(m.vertices)
    sel = V[(V[:, 2] > 22.0) & (V[:, 2] < 38.0)]
    if len(sel): fc_min = min(fc_min, float((np.abs(sel[:, 0] - 1.9) + np.abs(sel[:, 1])).min() - 26.46))
    for lo, hi in (((-50, -13, 23), (-20, 3, 33)), ((-54, -8, 22), (-36, 8, 36))):
        inside = np.all((V > np.array(lo)) & (V < np.array(hi)), axis=1)
        if inside.any(): print("FAIL lane", n_, lo, hi); ok = False
print("FC zone clearance (L1, need >= 0):", round(fc_min, 2)); ok &= fc_min >= 0
print("ALL OK" if ok else "FAILURES")
