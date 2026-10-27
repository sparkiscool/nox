"""Overseer arms v2: the root-joint options for the cage's crash loads into the arms (27 Oct), drawn on the real rear arm
geometry (plates, root block, insert holes, spoke node). Writes runs/overseer_arms/v2/joint_options.png.
Numbers: cage frame model arm loads (runs/overseer_frame/v1/arm_loads.json on the cage branch, F), joint arithmetic E.
"""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, FancyArrowPatch
P = os.path.join(ROOT, "runs", "overseer_layout", "v5", "parts")
S2 = 1 / math.sqrt(2)
def uv(u, v, d=135.0):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d)); return (u * c - v * s, u * s + v * c)
def outline(name):
    t = trimesh.load(os.path.join(P, name), force="mesh"); from scipy.spatial import ConvexHull
    p = t.vertices[:, :2]; return p[ConvexHull(p).vertices]
fig, axs = plt.subplots(1, 3, figsize=(16, 6.2), dpi=130)
titles = ["A  through-bolts (cage session's idea)", "B  replaceable fuse node (DEFAULT)", "C  printed hub plates instead of the S500 plates"]
notes = ["top-plate holes v ±15 and bottom-plate holes v ±10 are not\ncoaxial: needs new holes drilled in the PCB plates;\n1.6 mm PCB in bearing ~1.2 kN a hole (E) vs ~1.9 kN needed",
         "spoke node a separate printed part on 2 M3 into the arm,\nsized to let go at ~600 N (E); arm and root sized for 1.5x that:\n~250 N an insert (OK). After a crash: swap the node.\nCage session must stiffen tube 12-19 / core ties",
         "new printed (or carbon) top and bottom plates with\nlonger arm sockets and bigger bolt spacing: carries the\n2.3 kN; the S500 plates retired; FC / Pi mounts move\n(core session): the biggest change"]
cols = ["#c0392b", "#1e8449", "#7d3c98"]
for ax, t, n, c in zip(axs, titles, notes, cols):
    tp, bp = outline("top_plate.stl"), outline("bottom_plate.stl")
    ax.add_patch(Polygon(bp, closed=True, fc="#e5e8ea", ec="#99a3a8", lw=0.8))
    ax.add_patch(Polygon(tp, closed=True, fc="#d5dade", ec="#7b868c", lw=0.8))
    root = [uv(u, v) for u, v in ((58, -21), (75, -21), (75, 21), (58, 21))]
    ax.add_patch(Polygon(root, closed=True, fc="#55606b", ec="none"))
    beam = [uv(75, -18), uv(75, 18), (-53.3 + 8, 119.7 + 8), (-53.3 - 8, 119.7 - 8)]
    ax.add_patch(Polygon(beam, closed=True, fc="#9aa6b0", ec="none", alpha=0.8))
    for (u, v), k in (((63.17, 15), "top"), ((63.17, -15), "top"), ((63.74, 10), "bot"), ((63.74, -10), "bot")):
        x, y = uv(u, v); ax.add_patch(Circle((x, y), 2.2, fc="white" if k == "top" else "#f5b041", ec="k", lw=0.6))
    node = (-53.3, 119.7)
    ax.add_patch(Circle(node, 9 if t[0] != "B" else 11, fc=c if t[0] == "B" else "#55606b", ec="k", lw=0.8))
    ax.add_patch(FancyArrowPatch(node, (node[0] - 25, node[1] + 55), arrowstyle="-|>", mutation_scale=16, color=c, lw=2))
    ax.text(node[0] - 24, node[1] + 60, "2320 N crash 11 (F)" if t[0] != "B" else "≤ 600 N (fuse)", color=c, fontsize=9)
    if t[0] == "A":
        for v in (12.5, -12.5):
            x, y = uv(63.4, v); ax.plot(x, y, marker="x", color=c, ms=9, mew=2)
    if t[0] == "C":
        ax.add_patch(Polygon([uv(40, -45), uv(110, -45), uv(110, 45), uv(40, 45)], closed=True, fc="none", ec=c, lw=2, ls="--"))
    ax.set_xlim(-140, 20); ax.set_ylim(-10, 190); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(t, fontsize=11, color=c); ax.text(-138, -8, n, fontsize=8.5, va="bottom")
fig.suptitle("Overseer arms: the cage's crash load into the rear arm (plan, rear-left arm, real plate outlines; white = top-plate holes, "
             "orange = bottom-plate holes)\nnow: 2320 N at the node, ~174 N·m at the root, ~3.9 kN in plane over the 45 root, ~1 kN per M3 "
             "insert vs ~600 N (E)", fontsize=10)
fig.tight_layout(); out = os.path.join(ROOT, "runs", "overseer_arms", "v2", "joint_options.png"); fig.savefig(out, facecolor="white"); print(out)
