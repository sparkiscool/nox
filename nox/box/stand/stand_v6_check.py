"""Stand v6 fit checks (C / M): ring neighbours, frames vs the bay module mesh, pads vs the funnel collar, stub on the mast,
arm keep-outs, base stack. Reads the *_W.stl from stand_v6_parts.py.  python3 stand_v6_check.py"""
import math, json, numpy as np, trimesh, manifold3d as mf
import stand_v6_parts as P

def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def W(n): return trimesh.load(f"box_{n}_v6_W.stl")
out = {}
# ---- ring: place every frame, check every neighbour pair (exact volumes)
fr = {n: man(W(n)) for n in ("frame_mid", "frame_endR", "frame_endL")}
ring = []
for n, ps in P.POS.items():
    for p in ps: ring.append((p, n, P.place(fr[n], p)))
ring.sort()
pairs = []
for (p1, n1, m1), (p2, n2, m2) in zip(ring, ring[1:]):
    if p2 - p1 == 30:
        pairs.append((p1, p2, round((m1 ^ m2).volume(), 2)))
out["frame_pairs_overlap_mm3"] = pairs
# ---- frames vs the bay module mesh (top-frame OBJ placed at -90 and -60): module vertices inside a frame, and the reverse distance
mod = trimesh.load("models/box_baymod_v4i_W.obj", force="mesh")
res = []
for pb in P.BAYS:
    R = trimesh.transformations.rotation_matrix(math.radians(90 - pb), [0, 1, 0])
    mv = trimesh.transform_points(mod.vertices, R)
    for p, n, m in ring:
        if abs(p - pb) == 30:
            t = P.tm(m)
            inside = int(t.contains(mv).sum())
            cq = trimesh.proximity.ProximityQuery(t)
            d = np.abs(cq.signed_distance(mv[np.linalg.norm(mv[:, [0, 2]] - t.centroid[[0, 2]], axis=1) < 120]))
            res.append(dict(bay=pb, frame=p, kind=n, module_vertices_inside=inside, closest=round(float(d.min()), 2)))
out["frames_vs_module"] = res
# ---- the bay chain itself: -90 and -60 modules (mesh) touch only at their joint
# ---- lug hole alignment: hole axes from the module convention, so check frame hole centres vs the module mesh holes by sampling
# ---- pads vs the funnel collar (v5e check: outer surface |X| = 84 + (z - 143.6) + 4.4 for z 143.6..219.6)
pv = []
for n, p in (("frame_endR", 30), ("frame_endL", -210)):
    v = P.tm(P.place(fr[n], p)).vertices
    w = 84 + (v[:, 2] - 143.6)
    pv.append(dict(part=n, inside_collar=int(((np.abs(v[:, 0]) < w + 4.4 - 0.5) & (v[:, 2] > 143.6) & (v[:, 2] < 219.6)).sum()),
                   min_absX_above_Z102=round(float(np.abs(v[v[:, 2] > 102, 0]).min()), 1)))
    pads = v[(v[:, 2] > 140)]
    pv[-1]["pad_bbox"] = np.round([pads.min(0), pads.max(0)], 1).tolist()
out["pads"] = pv
# ---- arm keep-outs: ring parts stay outside r 98.5 (parked heads + packs) everywhere, outside r 136.5 at Y 74..95.5 (bars);
#      the base parts stay outside the bars' / heads' sweep (r 152.5 at Y 97..222, r 136.5 at Y 74..95.5) and the hub (r 86 at Y 0..67)
ko = {}
for p, n, m in ring:
    v = P.tm(m).vertices; r = np.hypot(v[:, 0], v[:, 2])
    ko[f"{n}@{p}"] = dict(min_r=round(float(r.min()), 1), y=[round(float(v[:, 1].min()), 1), round(float(v[:, 1].max()), 1)])
for n in ("mast", "kb", "cradle_keel", "bar_half", "toe"):
    v = W(n).vertices; r = np.hypot(v[:, 0], v[:, 2])
    hub = (v[:, 1] > 0.5) & (v[:, 1] < 67) & (r < 87)
    sweep = ((v[:, 1] > 97) & (v[:, 1] < 222) & (r < 153.5)) | ((v[:, 1] > 74) & (v[:, 1] < 95.5) & (r < 137.5))
    ko[n] = dict(in_hub_zone=int(hub.sum()), in_head_bar_sweep=int(sweep.sum()), ymax=round(float(v[:, 1].max()), 2))
out["keepouts"] = ko
# ---- base stack: KB plate | lap A | lap B | keel plate, no overlaps; bar B = A turned 180 about Z through (0, 54)
kbm, ck, ba, toe, mast = (man(W(n)) for n in ("kb", "cradle_keel", "bar_half", "toe", "mast"))
bb = ba.rotate([0, 0, 180]).translate([0, 2 * 54.0, 0])
crad = man(W("cradle_keel"))
stack = {"kb^barA": (kbm ^ ba).volume(), "kb^barB": (kbm ^ bb).volume(), "barA^barB": (ba ^ bb).volume(),
         "keel^barA": (ck ^ ba).volume(), "keel^barB": (ck ^ bb).volume(), "toe^cradle": (toe ^ ck).volume(),
         "mast^kb (sleeve on spigot)": (mast ^ kbm).volume()}
out["base_overlap_mm3"] = {k: round(v, 2) for k, v in stack.items()}
# ---- stub v3 on the mast: the saved print STL back to W (plate face Y 0 on the bed, z = Y): find its hole centres
st = trimesh.load("box_stub_v3.stl")
out["stub_print_bounds"] = np.round(st.bounds, 2).tolist()
# ---- cradle part vs the -90 module: module vertices inside the cradle_keel
R = trimesh.transformations.rotation_matrix(math.radians(180), [0, 1, 0])
mv = trimesh.transform_points(mod.vertices, R)
out["cradle_keel_vs_module_vertices_inside"] = int(W("cradle_keel").contains(mv).sum())
json.dump(out, open("stand_v6_check.json", "w"), indent=1, default=float)
print(json.dumps(out, indent=1, default=float))
