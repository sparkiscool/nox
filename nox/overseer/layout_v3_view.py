"""Motion artifact for Overseer layout v3 (26 Oct): the chosen layout (L2 on top, runs/overseer_layout/v3) with the
alternative beside it (L2 at the nose, runs/overseer_layout/v3/alt_nose, 900 to the left by its pose), both from their exported parts.

python3 nox/overseer/layout_v3_view.py      (V3_VIDEO=1 for the mp4 / gif, not committed; after layout_v3.py for both; writes runs/overseer_layout/v3/motion.html + .png)
"""
import json, math, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "overseer_layout", "v3")
res = json.load(open(os.path.join(RUN, "result.json"))); alt = json.load(open(os.path.join(RUN, "alt_nose", "result.json")))
s = Scene("Overseer Geodesic Cage",
          f"chosen: L2 on top, Ø{2 * res['R']:.0f} NU {res['NU']} sphere, bottom opening clear Ø{res['clear_d']:.0f}, ~{res['mass']:.0f} g (E); "
          f"alternative (y +900): L2 at the nose, bay {alt['delta']:+.0f} aft, opening Ø{alt['clear_d']:.0f}")
names = []
for folder, dy, tag in ((RUN, 0.0, ""), (os.path.join(RUN, "alt_nose"), 900.0, "alt: ")):
    for p in json.load(open(os.path.join(folder, "parts", "parts.json"))):
        if p["name"].startswith("prop"):
            continue
        m = trimesh.load(os.path.join(folder, "parts", p["file"]))   # at its own origin: the page stores int16 at 0.02 mm (±655)
        nm = tag + p["name"]; names.append(nm)
        # printed=False everywhere: these are layout envelopes, the page would weigh them as solid PETG; the layout's own
        # masses (summary.md) are in each part's note
        s.part(nm, m, group=("alt (L2 nose)" if tag else p["group"]), color=p["color"], printed=False,
               note=f"{p['mass_g']:.0f} g ({p['tag']}, layout model)")
    r = res if not tag else alt
    rings = []
    for i, (mx, my) in enumerate(r["motors_xy"]):
        a = trimesh.creation.annulus(124.5, 127.0, 1.0, sections=96); a.apply_translation([mx, my, 82.0]); rings.append(a)
    nm = tag + "props (swept)"; names.append(nm)
    s.part(nm, trimesh.util.concatenate(rings), group=("alt (L2 nose)" if tag else "bought"), color="#d9a400", printed=False)
spin = lambda t: trimesh.transformations.rotation_matrix(math.radians(360 * t / 24), [0, 0, 1])
s.timeline(lambda t: {n: (spin(t) if not n.startswith("alt") else
                          trimesh.transformations.translation_matrix([0, 900, 0]) @ spin(t) @ trimesh.transformations.translation_matrix([0, -900, 0]))
                      for n in names}, duration_s=24, fps=24)
s.view("iso", (1100, -1300, 900), (0, 300, 0)); s.view("chosen iso", (800, -800, 600), (0, 0, 0))
s.view("side", (0, -1800, 80), (0, 450, 80)); s.view("front", (1600, 0, 80), (0, 0, 80)); s.view("under", (300, -300, -1000), (0, 0, -200))
s.view("top", (0, 450, 2000), (0, 450, 0))
s.write(os.path.join(RUN, "motion.html"))
if os.environ.get("V3_VIDEO"):
  try:
    s.video(os.path.join(RUN, "motion.mp4"))
  except Exception as e:
    print("video skipped:", str(e)[:200])
print("wrote", os.path.join(RUN, "motion.html"))
