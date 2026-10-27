"""Motion artifact for Overseer layout v5 (26 Oct): the one layout (CG on the thrust centre), its parts from
runs/overseer_layout/v5/parts, plus the L2 field (half-sphere outline ahead of its base plane) and the two camera views
(frustum outline) as thin lines, so it shows that nothing of the drone is in either. Masses on the page are the layout
model's (each part's note), not solid PETG.

python3 nox/overseer/layout_v5_view.py      (after layout_v5.py; V5_VIDEO=1 for the mp4 / gif, not committed)
"""
import json, math, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "overseer_layout", "v5")
r = json.load(open(os.path.join(RUN, "result.json")))
line = lambda a, b, rr=0.7: trimesh.creation.cylinder(rr, np.linalg.norm(np.subtract(b, a)), sections=6, segment=np.vstack([a, b]))
s = Scene("Overseer Disk Cage",
          f"v2 layout + geodesic disk cage (frequency 2, 6 mm tubes), {r['extents'][0]:.0f} x {r['extents'][1]:.0f} x {r['extents'][2]:.0f}, "
          f"~{r['mass']:.0f} g (E), CG {r['cg_off']:.1f} mm off the thrust centre; the L2 and the camera pair (beside it, field maximized) outside the cage, nothing in their views")
names = []
for p in json.load(open(os.path.join(RUN, "parts", "parts.json"))):
    if p["name"].startswith("prop"):
        continue
    names.append(p["name"])
    s.part(p["name"], trimesh.load(os.path.join(RUN, "parts", p["file"])), group=p["group"], color=p["color"], printed=False,
           note=f"{p['mass_g']:.0f} g ({p['tag']}, layout model)")
rings = []
for mx, my in r["motors_xy"]:
    a = trimesh.creation.annulus(124.5, 127.0, 1.0, sections=96); a.apply_translation([mx, my, 82.0]); rings.append(a)
names.append("props (swept)"); s.part("props (swept)", trimesh.util.concatenate(rings), group="bought", color="#d9a400", printed=False)
# L2 field: half-sphere outline (r 220) ahead of its base plane; camera view: frustum edges (220 long)
Ax, B = np.array(r["l2_axis"]), np.array(r["l2_base"]); up = np.array([-Ax[2], 0, Ax[0]]); ey = np.array([0, 1.0, 0]); Rf = 220.0
lines = []
for t in np.linspace(0, math.pi, 7):                                   # meridians
    d = math.cos(t) * up + math.sin(t) * ey
    pts_ = [B + Rf * (math.cos(f) * d + math.sin(f) * Ax) for f in np.linspace(0, math.pi, 25)]
    lines += [line(pts_[i], pts_[i + 1]) for i in range(24)]
rim = [B + Rf * (math.cos(f) * up + math.sin(f) * ey) for f in np.linspace(0, 2 * math.pi, 49)]
lines += [line(rim[i], rim[i + 1]) for i in range(48)]
names.append("L2 field (90 deg mode)"); s.part("L2 field (90 deg mode)", trimesh.util.concatenate(lines), group="views", color="#4a90d9", printed=False)
fr = []
for c in r["cams"]:                                                    # the camera pair: frustum edges (220 long)
    L_, f_, u_, r_ = (np.array(c[k]) for k in ("lens", "f", "u", "r")); ha, va = math.radians(c["ha"]), math.radians(c["hv"])
    cor = [L_ + Rf * (f_ + math.tan(ha) * sy * r_ + math.tan(va) * sz * u_) / np.linalg.norm(f_ + math.tan(ha) * sy * r_ + math.tan(va) * sz * u_)
           for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    fr += [line(L_, c_) for c_ in cor] + [line(cor[i], cor[(i + 1) % 4]) for i in range(4)]
names.append("camera views"); s.part("camera views", trimesh.util.concatenate(fr), group="views", color="#3fa34d", printed=False)
spin = lambda t: trimesh.transformations.rotation_matrix(math.radians(360 * t / 24), [0, 0, 1])
s.timeline(lambda t: {n: spin(t) for n in names}, duration_s=24, fps=24)
s.view("iso", (700, -800, 500), (60, 0, 20)); s.view("side", (60, -1300, 40), (60, 0, 40)); s.view("front", (1400, 0, 60), (0, 0, 40))
s.view("top", (60, 0, 1300), (60, 0, 0)); s.view("under", (300, -300, -900), (0, 0, -50))
s.write(os.path.join(RUN, "motion.html"))
if os.environ.get("V5_VIDEO"):
    try:
        s.video(os.path.join(RUN, "motion.mp4"))
    except Exception as e:
        print("video skipped:", str(e)[:200])
print("wrote", os.path.join(RUN, "motion.html"))
