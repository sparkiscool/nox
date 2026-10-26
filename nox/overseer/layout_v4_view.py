"""Motion artifact for Overseer layout v4 (26 Oct): the chosen layout (runs/overseer_layout/v4) and its two alternatives
beside it by pose (pack on the thrust axis at y +850, frequency 3 at y -850), all from their exported parts; masses on
the page are the layout model's (each part's note), not solid PETG.

python3 nox/overseer/layout_v4_view.py      (after layout_v4.py for all three; V4_VIDEO=1 for the mp4 / gif, not committed)
"""
import json, math, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "overseer_layout", "v4")
SETS = [(RUN, 0.0, "", "chosen"), (os.path.join(RUN, "alt_pack_axis"), 850.0, "axis: ", "alt pack on axis (y +850)"),
        (os.path.join(RUN, "alt_f3"), -850.0, "f3: ", "alt frequency 3 (y -850)")]
res = {t: json.load(open(os.path.join(f, "result.json"))) for f, _, t, _ in SETS}
r, ra, rf = res[""], res["axis: "], res["f3: "]
s = Scene("Overseer Disk Cage",
          f"chosen: v2 layout + geodesic disk cage (frequency 2, 6 mm tubes), {r['extents'][0]:.0f} x {r['extents'][1]:.0f} x {r['extents'][2]:.0f}, "
          f"~{r['mass']:.0f} g (E); y +850: pack on the thrust axis, front arms r {math.hypot(*ra['motors_xy'][0]):.0f}, ~{ra['mass']:.0f} g; "
          f"y -850: frequency 3, ~{rf['mass']:.0f} g")
names, offs = [], {}
for folder, dy, tag, grp in SETS:
    for p in json.load(open(os.path.join(folder, "parts", "parts.json"))):
        if p["name"].startswith("prop"):
            continue
        m = trimesh.load(os.path.join(folder, "parts", p["file"]))
        nm = tag + p["name"]; names.append(nm); offs[nm] = dy
        s.part(nm, m, group=(grp if tag else p["group"]), color=p["color"], printed=False, note=f"{p['mass_g']:.0f} g ({p['tag']}, layout model)")
    rings = []
    for mx, my in res[tag]["motors_xy"]:
        a = trimesh.creation.annulus(124.5, 127.0, 1.0, sections=96); a.apply_translation([mx, my, 82.0]); rings.append(a)
    nm = tag + "props (swept)"; names.append(nm); offs[nm] = dy
    s.part(nm, trimesh.util.concatenate(rings), group=(grp if tag else "bought"), color="#d9a400", printed=False)
spin = lambda t: trimesh.transformations.rotation_matrix(math.radians(360 * t / 24), [0, 0, 1])
T = lambda dy: trimesh.transformations.translation_matrix([0, dy, 0])
s.timeline(lambda t: {n: T(offs[n]) @ spin(t) for n in names}, duration_s=24, fps=24)
s.view("chosen iso", (700, -800, 500), (40, 0, 20)); s.view("all three", (1300, -1900, 1000), (40, 0, 0))
s.view("side", (40, -1500, 40), (40, 0, 40)); s.view("front", (1500, 0, 40), (0, 0, 40)); s.view("top", (40, 0, 1500), (40, 0, 0))
s.view("under", (300, -300, -900), (0, 0, -50))
s.write(os.path.join(RUN, "motion.html"))
if os.environ.get("V4_VIDEO"):
    try:
        s.video(os.path.join(RUN, "motion.mp4"))
    except Exception as e:
        print("video skipped:", str(e)[:200])
print("wrote", os.path.join(RUN, "motion.html"))
