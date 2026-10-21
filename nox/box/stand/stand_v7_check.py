"""Stand v7 fit checks (C / M) + assembly render. Every v7 part placed in W against the v6 parts it meets, the bay module
mesh, the sheets (3.0 slabs) and the funnel collar.  python3 stand_v7_check.py -> stand_v7_check.json, stand_v7_assembly.png"""
import math, json, numpy as np, trimesh, manifold3d as mf
import stand_v6_parts as S, stand_v7_parts as V

def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def W6(n): return man(trimesh.load(f"box_{n}_v6_W.stl"))
import os
V7B = set(os.environ.get("V7B", "").split(","))
TAGS = dict(kv.split("=") for kv in os.environ.get("TAGS", "").split(",") if "=" in kv)   # e.g. TAGS=bracket_e=v7c (wins over V7B)
def W7(n): return man(trimesh.load(f"box_{n}_{TAGS.get(n, 'v7b' if n in V7B else 'v7')}_W.stl"))
def vol(m): return round(m.volume(), 2)
out = {}
X_IN, YB, YF, ZT, T = V.X_IN, V.Y_BACK, V.Y_FRONT, V.Z_TOP, V.T_SHEET
FLOOR = S.FLOOR
# ---- placed parts
parts = {}
parts["mast"] = W6("mast"); parts["cradle_keel"] = W6("cradle_keel")
parts["kb"] = W7("kb"); parts["toe"] = W7("toe")
ba = W7("bar_half"); parts["barA"] = ba; parts["barB"] = ba.rotate([0, 0, 180]).translate([0, 2 * 54.0, 0])
fm = W6("frame_mid")
for p in S.POS["frame_mid"]: parts[f"frame_mid@{p}"] = S.place(fm, p)
parts["frame_endR@30"] = S.place(W7("frame_endR"), 30); parts["frame_endL@-210"] = S.place(W7("frame_endL"), -210)
ps = W7("post_seg")
for c in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
    for k in range(3):
        parts[f"post{c}{k}"] = V.post_place(ps, c).translate([0, 0, FLOOR + k * V.SEG])
parts["bracket_jA@-15"] = W7("bracket_jA"); parts["bracket_jA@165"] = V.bracket_place("bracket_jA", W7("bracket_jA"), "L")
parts["bracket_jB@15"] = W7("bracket_jB"); parts["bracket_jB@195"] = V.bracket_place("bracket_jB", W7("bracket_jB"), "L")
parts["bracket_e@45"] = W7("bracket_e"); parts["bracket_e@135"] = V.bracket_place("bracket_e", W7("bracket_e"), "L")
mod = trimesh.load("models/box_baymod_v4i_W.obj", force="mesh")
mods = {}
for pb in S.BAYS:
    t = mod.copy(); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 - pb), [0, 1, 0])); mods[pb] = t
# sheets as solids (C): side Y -63..334 at X +-(186..189); front X +-183 at Y 337..340; rear at Y -69..-66
sheets = {"side+": S.box(X_IN, X_IN + T, YB + 3, YF - 3, FLOOR, ZT), "side-": S.box(-X_IN - T, -X_IN, YB + 3, YF - 3, FLOOR, ZT),
          "front": S.box(-X_IN + 3, X_IN - 3, YF, YF + T, FLOOR, ZT), "rear": S.box(-X_IN + 3, X_IN - 3, YB - T, YB, FLOOR, ZT)}
# ---- overlaps: every part pair that could touch (bounding boxes overlap), exact volumes
names = list(parts)
def bb(m): return np.array(m.bounding_box()).reshape(2, 3)
ov = {}
for i, a in enumerate(names):
    for b in names[i + 1:]:
        A, B = bb(parts[a]), bb(parts[b])
        if (A[0] < B[1]).all() and (B[0] < A[1]).all():
            v = vol(parts[a] ^ parts[b])
            if v > 0: ov[f"{a} ^ {b}"] = v
out["part_overlaps_mm3"] = ov
sv = {}
for sn, sm in sheets.items():
    for a in names:
        A, B = bb(parts[a]), bb(sm)
        if (A[0] < B[1]).all() and (B[0] < A[1]).all():
            v = vol(parts[a] ^ sm)
            if v > 0: sv[f"{a} ^ {sn}"] = v
out["sheet_overlaps_mm3"] = sv
# ---- brackets and end frames vs the module meshes (vertices inside)
mv = {}
for pb, t in mods.items():
    for a in names:
        if a.startswith("bracket") or a.startswith("frame_end"):
            tt = S.tm(parts[a]); inside = int(tt.contains(t.vertices).sum())
            if inside: mv[f"{a} vs module {pb}"] = inside
out["module_vertices_inside"] = mv
# ---- closest distances: brackets to their frames (should be the 0.3 clearances)
cd = {}
for a, fr in (("bracket_jA@-15", ("frame_mid@-30", "frame_mid@0")), ("bracket_jB@15", ("frame_mid@0", "frame_endR@30")), ("bracket_e@45", ("frame_endR@30",))):
    ta = S.tm(parts[a])
    for f in fr:
        tf = S.tm(parts[f]); q = trimesh.proximity.ProximityQuery(tf)
        d = q.signed_distance(ta.vertices); cd[f"{a} to {f}"] = dict(closest=round(float(np.abs(d).min()), 2), inside=int((d > 0).sum()))
out["bracket_clearance"] = cd
# ---- funnel collar vs brackets / posts (outer surface |X| = 84 + (Z - 143.6) + 4.4, |Y - 177| likewise, Z 143.6..237)
fc = {}
for a in names:
    if a.startswith("bracket") or a.startswith("post"):
        v = S.tm(parts[a]).vertices
        w = 84 + (v[:, 2] - 143.6) + 4.4
        ins = (v[:, 2] > 143.6) & (v[:, 2] < 237.5) & (np.abs(v[:, 0]) < w) & (np.abs(v[:, 1] - 177) < w)
        if ins.sum(): fc[a] = int(ins.sum())
out["funnel_collar_vertices_inside"] = fc
# ---- sheet sizes, post count, masses
rep = json.load(open("stand_v7_prints.json"))
out["sheets_mm"] = {"side x2": [round(YF - YB - 6), round(ZT - FLOOR)], "front / rear x2": [round(2 * X_IN - 6), round(ZT - FLOOR)]}
out["print_g"] = {k: [v["g"], v["qty"]] for k, v in rep.items()}
out["new_print_g_total"] = round(sum(v["g"] * v["qty"] for v in rep.values()), 1)
json.dump(out, open("stand_v7_check.json", "w"), indent=1, default=float)
print(json.dumps(out, indent=1, default=float))

# ---- render
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
NEW, OLD, ARM, WALL, POSTC, FUN, BR = (0.11, 0.44, 0.54), (0.69, 0.55, 0.36), (0.6, 0.63, 0.66), (0.55, 0.8, 0.9), (0.85, 0.35, 0.2), (0.72, 0.72, 0.68), (0.95, 0.6, 0.1)
items = []
for a, m in parts.items():
    col = NEW
    if a.startswith("post"): col = POSTC
    if a.startswith("bracket"): col = BR
    if a in ("mast", "cradle_keel") or a.startswith("frame_mid"): col = (0.35, 0.55, 0.6)
    items.append((S.tm(m), col, 1.0))
for pb, t in mods.items(): items.append((t, OLD, 1.0))
hub = trimesh.creation.annulus(40, 86, 59, sections=48); hub.apply_translation([0, 0, 37.5]); hub.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [1, 0, 0])); items.append((hub, ARM, 1.0))
def pyr(z0, z1, h0, h1):
    YC = 177.0
    v = [[-h0, YC - h0, z0], [h0, YC - h0, z0], [h0, YC + h0, z0], [-h0, YC + h0, z0], [-h1, YC - h1, z1], [h1, YC - h1, z1], [h1, YC + h1, z1], [-h1, YC + h1, z1]]
    f = [[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return trimesh.Trimesh(v, f)
items.append((pyr(161.0, 237.0, 84.0, 160.0), FUN, 0.9))
psu = trimesh.creation.box(extents=[64, 40, 40], transform=trimesh.transformations.translation_matrix([57, -30, FLOOR + 20])); items.append((psu, (0.3, 0.3, 0.3), 1.0))
for sn, sm in sheets.items(): items.append((S.tm(sm), WALL, 0.22))
fig = plt.figure(figsize=(17, 9), dpi=120)
for k_, (el, az, ttl) in enumerate([(18, -52, "front-right"), (24, 214, "rear-left")]):
    ax = fig.add_subplot(1, 2, k_ + 1, projection="3d")
    Lv = np.array([0.4, -0.6, 0.7]); Lv /= np.linalg.norm(Lv)
    for t, col, al in items:
        if len(t.faces) > 4000: t = t.simplify_quadric_decimation(face_count=4000)
        sh = 0.45 + 0.55 * np.clip(t.face_normals @ Lv, 0, 1)
        fc_ = np.clip(np.array(col)[None, :] * sh[:, None] * 1.25, 0, 1)
        ax.add_collection3d(Poly3DCollection(t.triangles, facecolors=fc_, edgecolors="none", alpha=al))
    ax.set_xlim(-220, 220); ax.set_ylim(-110, 360); ax.set_zlim(-205, 255); ax.set_box_aspect((480, 480, 460))
    ax.view_init(elev=el, azim=az); ax.set_axis_off(); ax.text2D(0.04, 0.93, ttl, transform=ax.transAxes, fontsize=11)
fig.suptitle("Nox box stand v7: ring bolted to the side sheets through 6 brackets (gold), 12 corner post segments (orange) holding the 4 sheets,\n"
             "mid cross bar tips, toe and keel bolted to the sheets; mast, cradle_keel, 5 mid frames unchanged from v6", fontsize=10)
fig.subplots_adjust(0, 0, 1, 0.92, 0, 0); fig.savefig(os.environ.get("ASM_PNG", "stand_v7_assembly.png")); print("stand_v7_assembly.png")
