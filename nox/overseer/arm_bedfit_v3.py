"""Overseer front arm: why one piece cannot print on the A1 mini (180 bed) and how a 2-piece split joined by the carbon
rods fits (27 Oct). Plan of the front v3 mesh (arm frame) with the bed square at its best angle; the split at x = XS.
python3 nox/overseer/arm_bedfit_v3.py <front_v3.stl> -> runs/overseer_arms/v3/bedfit.png"""
import os, sys, math
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
m = trimesh.load(sys.argv[1], force="mesh")
pts = m.vertices[:, :2][::7]
XS = 95.0
def best_square(P):
    best = None
    for t in np.radians(np.arange(0, 90, 0.5)):
        R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]); Q = P @ R.T
        s = max(np.ptp(Q[:, 0]), np.ptp(Q[:, 1]))
        if best is None or s < best[0]: best = (s, t, Q.min(0))
    return best
fig, axs = plt.subplots(1, 2, figsize=(15, 6.5), dpi=130)
for ax, title, groups in ((axs[0], "one piece (front v3): needs a %.0f mm square, the bed is 180", [pts]),
                          (axs[1], "two pieces joined by the 4 carbon rods: root piece %.0f, tip piece %.0f", [pts[pts[:, 0] < XS + 8], pts[pts[:, 0] > XS - 8]])):
    sizes = []
    for k, P in enumerate(groups):
        s, t, lo = best_square(P); sizes.append(s)
        ax.scatter(P[:, 0], P[:, 1], s=0.3, c=["#55606b", "#1e8449"][k % 2])
        R = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
        sq = np.array([[0, 0], [180, 0], [180, 180], [0, 180]]) + lo
        ax.add_patch(Polygon(sq @ R.T, closed=True, fill=False, ec="#c0392b" if s > 180 else "#2e86c1", lw=1.5, ls="--"))
    if len(groups) == 2: ax.axvline(XS, color="#d68910", lw=2); ax.text(XS + 2, 60, "split: rods carry it\n(epoxied in both pieces)", color="#d68910", fontsize=9)
    ax.set_title(title % tuple(sizes), fontsize=11); ax.set_aspect("equal"); ax.axis("off")
fig.suptitle("Overseer front arm and the A1 mini bed (180 x 180, dashed: red too small, blue fits); arm frame plan, root left, motor right", fontsize=11)
fig.tight_layout(); out = os.path.join(ROOT, "runs", "overseer_arms", "v3", "bedfit.png"); os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, facecolor="white"); print(out)
