"""Overseer arms v2: where the cage attaches (the spoke node beside each motor, the L2 brace sockets in the front beam),
drawn from the setup's real geometry: drawn bodies + sockets, rods, the spoke and brace tubes to their far ends, the screw
key column under each motor, the design envelopes outlined. Writes runs/overseer_arms/v2/links.png.

NOX_ARM_LINKS=1 ... set by the script itself: python3 nox/overseer/arm_links_view_v2.py
"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
os.environ.setdefault("NOX_ARM_VERSION", "v2"); os.environ.setdefault("NOX_ARM_LINKS", "1")
os.environ.setdefault("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5"))
os.environ.setdefault("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh
import arm_topo_v1 as A
from topo.part import tm
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

J = json.load(open(A.JOINT))
links = json.load(open(os.path.join(A.LAYOUT, "result.json")))["links"]
parts, lines = [], []
for w in ("front", "rear"):
    B, F = A.bodies(w)
    for mir in (False, True):
        parts.append((A.arm_to_drone(tm(B["drawn"]), F, mir), "#55606b", 1.0))
        parts.append((A.arm_to_drone(tm(B["env"]), F, mir), "#7fb3c9", 0.10))
        if w == "front":
            parts.append((A.arm_to_drone(tm(B["rods"]), F, mir), "#111417", 1.0))
    node = np.array(A.SPOKE_NODE[w]); tag = "1" if w == "front" else "2"
    for k, (p, q) in links.items():
        p, q = np.array(p), np.array(q)
        if k.startswith(f"spoke {tag}."):
            far = q if abs(p[1]) == 139.5 else p
            for sy in (1, -1):
                lines.append(((node * [1, sy, 1], far * [1, sy, 1]), "#c0392b"))
    for k, e in A.BRACE_END.get(w, []):
        p, q = map(np.array, links[k]); far = p if p[0] > q[0] else q
        for sy in (1, -1):
            lines.append(((np.array(e) * [1, sy, 1], far * [1, sy, 1]), "#d68910"))
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
fig = plt.figure(figsize=(16, 7), dpi=130)
for k, (title, el, az) in enumerate((("iso", 24, -120), ("plan (nose to the right)", 90, -90), ("side (from -y)", 0, -90))):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    for m, col, al in parts:
        sh = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col))
        fc = np.clip(base[None] * sh[:, None] + 0.05, 0, 1)
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=np.c_[fc, np.full(len(fc), al)], linewidths=0))
    for (a, b), col in lines:
        ax.plot(*zip(a, b), color=col, lw=2.2)
    for mx, my in A.motors_xy():
        ax.plot([mx, mx], [my, my], [-70, 45], color="#2e86c1", lw=1.2, ls="--")
    for pl in ("top_plate.stl", "bottom_plate.stl"):
        t = trimesh.load(os.path.join(A.LAYOUT, "parts", pl), force="mesh")
        ax.add_collection3d(Poly3DCollection(t.vertices[t.faces], facecolors=(0.6, 0.65, 0.7, 0.15), linewidths=0))
    ax.set_xlim(-160, 400); ax.set_ylim(-280, 280); ax.set_zlim(-90, 110); ax.set_box_aspect((560, 560, 200))
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=11)
wn = J["worst_N"]
fig.suptitle("Overseer arms v2: the cage attachments (red: 3 spokes from one node beside each motor, off the screw key column "
             "(blue dashes); orange: 2 L2 braces into the front beam, inside the arm so the top stays flat)\n"
             f"worst summed crash force from the frame model: front tip {wn['arm 1 tip']:.0f} N, rear tip {wn['arm 2 tip']:.0f} N, "
             f"front beam braces {wn['arm 1 beam (L2 braces)']:.0f} N (F)", fontsize=10)
fig.tight_layout()
out = os.path.join(ROOT, "runs", "overseer_arms", "v2", "links.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, facecolor="white"); print(out)
