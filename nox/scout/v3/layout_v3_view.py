"""Motion artifact and stills for Scout v3 layout v3 / cage v2 (27 Oct): the drone from runs/scout_layout/v7/parts, the
shared bottom ring drawn bold, the front camera's view (frustum outline), and the Overseer cage v3 on the same datum (ring on
ring) as a thin ghost, so the funnel interchange shows. Masses on the page are the layout model's (each part's note).

python3 nox/scout/v3/layout_v3_view.py      (after layout_v3.py)
"""
import json, math, os, sys
import numpy as np, trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "scout_layout", "v7")
OV = os.path.join(ROOT, "runs", "overseer_layout", "v5")
r = json.load(open(os.path.join(RUN, "result.json")))
sh = json.load(open(os.path.join(RUN, "cage_shell.json"))); N = np.array(sh["nodes"]); E = sh["rods"]; ring = sh["open"][0]["ring"]
so = json.load(open(os.path.join(OV, "cage_shell.json"))); No = np.array(so["nodes"]); Eo = so["rods"]
ov_cart = trimesh.load(os.path.join(OV, "parts", "cartridge.stl"))
Ro = No[so["open"][0]["ring"]]
No = No - [ov_cart.bounds[:, 0].mean(), 0, Ro[:, 2].mean()] + [r["tab_x"], 0, r["z_o"]]      # the Overseer on the Scout's datum
line = lambda a, b, rr=0.7: trimesh.creation.cylinder(rr, np.linalg.norm(np.subtract(b, a)), sections=6, segment=np.vstack([a, b]))

s = Scene("Scout v3 Cage",
          f"Scout v3 layout (PR 94) + triangulated cage of 3 mm rods (half the rods, any length), bottom ring = the Overseer's (funnel interchangeable), one camera outside the cage, "
          f"carbon base kept; {r['extents'][0]:.0f} x {r['extents'][1]:.0f} x {r['extents'][2]:.0f} (node centres), ~{r['mass']:.0f} g (E), "
          f"CG {math.hypot(*r['cg'][:2]):.1f} mm pack in / {math.hypot(*r['cg_pack_out'][:2]):.1f} pack out")
names = []
for p in json.load(open(os.path.join(RUN, "parts", "parts.json"))):
    if p["name"].startswith("prop"):
        continue
    names.append(p["name"])
    s.part(p["name"], trimesh.load(os.path.join(RUN, "parts", p["file"])), group=p["group"], color=p["color"], printed=False,
           note=f"{p['mass_g']:.0f} g ({p['tag']}, layout model)")
rings = []
for mx, my in [(54.1, 54.1), (54.1, -54.1), (-54.1, 54.1), (-54.1, -54.1)]:
    a = trimesh.creation.annulus(43.0, 44.45, 1.0, sections=96); a.apply_translation([mx, my, 41.2]); rings.append(a)
names.append("props (swept)"); s.part("props (swept)", trimesh.util.concatenate(rings), group="bought", color="#d9a400", printed=False)
rr_ = [trimesh.creation.cylinder(4.0, np.linalg.norm(N[a] - N[b]), sections=12, segment=np.vstack([N[a], N[b]]))
       for a, b in E if a in ring and b in ring]
names.append("shared bottom ring (= Overseer's)"); s.part("shared bottom ring (= Overseer's)", trimesh.util.concatenate(rr_), group="cage", color="#e07020", printed=False,
                                                          note="the Overseer cage v3 opening ring: same 8 nodes and tubes, same datum (grip line, 3 under the cartridge flat bottom)")
names.append("Overseer cage (same datum, ghost)")
s.part("Overseer cage (same datum, ghost)", trimesh.util.concatenate([line(No[a], No[b], 0.6) for a, b in Eo]), group="reference", color="#9ab0c8", printed=False,
       note="Overseer cage v3 placed ring on ring: the Scout's lower shell sits inside the Overseer's, so a funnel that takes one takes both")
L_ = np.array(r["cam"]["lens"]); ha, va = math.radians(r["cam"]["hfov"] / 2), math.radians(r["cam"]["vfov"] / 2); Rf = 160.0
ey, ez, ex = np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
cor = [L_ + Rf * (ex + math.tan(ha) * sy * ey + math.tan(va) * sz * ez) / np.linalg.norm(ex + math.tan(ha) * sy * ey + math.tan(va) * sz * ez)
       for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
fr = [line(L_, c) for c in cor] + [line(cor[i], cor[(i + 1) % 4]) for i in range(4)]
names.append("front camera view"); s.part("front camera view", trimesh.util.concatenate(fr), group="views", color="#3fa34d", printed=False)
# the two alternatives weighed (concept rule), side by side: B = the frequency 2 geodesic of cage v1 with every rod 3 mm (123 rods),
# C = that geodesic thinned to ~half the rods (SCOUT_NFREE); each drawn whole, offset in y
ALTS = [("B geodesic, every rod 3 mm", os.path.join(RUN, "alt_geodesic_full"), -460.0),
        ("C geodesic thinned to ~half", os.path.join(RUN, "alt_geodesic_half"), 460.0)]
for tag, d_, off in ALTS:
    if not os.path.exists(os.path.join(d_, "parts", "parts.json")): continue
    ms = [trimesh.load(os.path.join(d_, "parts", p_["file"])).apply_translation([0, off, 0])
          for p_ in json.load(open(os.path.join(d_, "parts", "parts.json"))) if not p_["name"].startswith("prop")]
    ra = json.load(open(os.path.join(d_, "result.json")))
    nm = f"alternative {tag} ({ra['mass']:.0f} g E, {ra['extents'][0]:.0f} x {ra['extents'][1]:.0f} x {ra['extents'][2]:.0f})"
    names.append(nm); s.part(nm, trimesh.util.concatenate(ms), group="alternatives", color="#8a8a8a", printed=False,
                             note="weighed, not picked: runs/scout_layout/v7/summary.md")
spin = lambda t: trimesh.transformations.rotation_matrix(math.radians(360 * t / 20), [0, 0, 1])
s.timeline(lambda t: {n: spin(t) for n in names}, duration_s=20, fps=24)
s.view("iso", (500, -560, 380), (0, 0, 10)); s.view("all three", (0, -1700, 900), (0, 0, 0)); s.view("side", (0, -900, 30), (0, 0, 30)); s.view("front", (900, 0, 40), (0, 0, 30))
s.view("top", (0, 0, 900), (0, 0, 0)); s.view("under", (200, -200, -700), (0, 0, -40))
s.write(os.path.join(RUN, "motion.html"))

# ---------------------------------------------------------------- stills (matplotlib): iso, side, front, top
parts = json.load(open(os.path.join(RUN, "parts", "parts.json")))
fig = plt.figure(figsize=(16, 11), dpi=110)
for k, (el, az, ttl) in enumerate(((22, -55, "iso"), (0, -90, "side (from -y)"), (0, 0, "front (from +x)"), (90, -90, "top"))):
    ax = fig.add_subplot(2, 2, k + 1, projection="3d"); ax.set_proj_type("ortho")
    for p in parts:
        if p["name"].startswith("cage shell") or p["name"].startswith("spokes"):
            continue
        m = trimesh.load(os.path.join(RUN, "parts", p["file"]))
        if len(m.faces) > 6000: m = m.simplify_quadric_decimation(face_count=6000) if hasattr(m, "simplify_quadric_decimation") else m
        pc = Poly3DCollection(m.triangles, facecolor=p["color"], edgecolor="none", alpha=0.35 if p["name"].startswith("prop") else 0.9)
        ax.add_collection3d(pc)
    ax.add_collection3d(Line3DCollection([(No[a], No[b]) for a, b in Eo], colors="#9ab0c8", linewidths=0.6))
    ax.add_collection3d(Line3DCollection([(N[a], N[b]) for a, b in E if not (a in ring and b in ring)], colors="#222222", linewidths=1.6))
    ax.add_collection3d(Line3DCollection([(N[a], N[b]) for a, b in E if a in ring and b in ring], colors="#e07020", linewidths=3.0))
    ax.add_collection3d(Line3DCollection([tuple(map(tuple, v)) for v in r["links"].values()], colors="#666666", linewidths=0.8))
    ax.add_collection3d(Line3DCollection([(L_, c) for c in cor] + [(cor[i], cor[(i + 1) % 4]) for i in range(4)], colors="#3fa34d", linewidths=0.8))
    ax.scatter(*N.T, s=10, c="#7d8a99")
    lo, hi = np.array([-260, -260, -120]), np.array([300, 260, 180])
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo)
    ax.view_init(el, az); ax.set_title(ttl); ax.set_axis_off()
fig.suptitle("Scout v3 layout v3 / cage v2 (3 mm rods, half the rods): orange = the shared bottom ring (the Overseer's), light blue = the Overseer cage on the same datum, "
             "green = camera view", fontsize=11)
fig.tight_layout(); fig.savefig(os.path.join(RUN, "preview.png")); print("wrote", os.path.join(RUN, "preview.png"))

# ---------------------------------------------------------------- the three options side by side (picked A, B, C): top and side
fig = plt.figure(figsize=(18, 9), dpi=100)
opts = [("A picked: free pattern, every rod 3 mm, half the rods", RUN)] + [(f"{t}", d_) for t, d_, _ in ALTS]
for c_, (ttl, d_) in enumerate(opts):
    if not os.path.exists(os.path.join(d_, "cage_shell.json")): continue
    ra = json.load(open(os.path.join(d_, "result.json"))); sa = json.load(open(os.path.join(d_, "cage_shell.json")))
    Na = np.array(sa["nodes"]); Ea = sa["rods"]; rg = sa["open"][0]["ring"]
    pa = json.load(open(os.path.join(d_, "parts", "parts.json")))
    for r_, (el, az, vt) in enumerate(((90, -90, "top"), (0, -90, "side"))):
        ax = fig.add_subplot(2, 3, r_ * 3 + c_ + 1, projection="3d"); ax.set_proj_type("ortho")
        for p in pa:
            if p["name"].startswith(("cage shell", "spokes")): continue
            m = trimesh.load(os.path.join(d_, "parts", p["file"]))
            ax.add_collection3d(Poly3DCollection(m.triangles, facecolor=p["color"], edgecolor="none", alpha=0.35 if p["name"].startswith("prop") else 0.9))
        ax.add_collection3d(Line3DCollection([(Na[a], Na[b]) for a, b in Ea if not (a in rg and b in rg)], colors="#222222", linewidths=1.4))
        ax.add_collection3d(Line3DCollection([(Na[a], Na[b]) for a, b in Ea if a in rg and b in rg], colors="#e07020", linewidths=2.6))
        ax.add_collection3d(Line3DCollection([tuple(map(tuple, v)) for v in ra["links"].values()], colors="#666666", linewidths=0.7))
        lo, hi = np.array([-230, -170, -80]), np.array([230, 170, 160])
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo)
        ax.view_init(el, az); ax.set_axis_off()
        ax.set_title(f"{ttl}\n{ra['mass']:.0f} g E, {ra['extents'][0]:.0f} x {ra['extents'][1]:.0f} x {ra['extents'][2]:.0f} ({vt})" if r_ == 0 else vt, fontsize=9)
fig.suptitle("Scout v3 cage options: orange = bottom ring", fontsize=11); fig.tight_layout()
fig.savefig(os.path.join(RUN, "options.png")); print("wrote", os.path.join(RUN, "options.png"))
