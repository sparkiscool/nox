"""Overseer arms v2: close-up of the cage nodes drawn into the arms (spoke node beside each motor, the L2 brace sockets
in the front beam), from the setup's real geometry, with the spoke / brace tube stubs and the motor screw key column.
Writes runs/overseer_arms/v2/nodes.png."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for k, v in (("NOX_ARM_VERSION", "v2"), ("NOX_ARM_LINKS", "1"), ("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5")),
             ("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))):
    os.environ.setdefault(k, v)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh
import arm_topo_v1 as A
from topo.part import tm
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
links = json.load(open(os.path.join(A.LAYOUT, "result.json")))["links"]
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
fig = plt.figure(figsize=(16, 6), dpi=130)
views = [("front arm: spoke node beside the motor", "front", "node"), ("front arm: 2 L2 brace sockets in the beam", "front", "brace"),
         ("rear arm: spoke node beside the motor", "rear", "node")]
for k, (title, w, what) in enumerate(views):
    B, F = A.bodies(w)
    drawn = A.arm_to_drone(tm(B["drawn"] - B["cuts"]), F)
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    sh = 0.35 + 0.65 * np.clip(drawn.face_normals @ light, 0, 1)
    ax.add_collection3d(Poly3DCollection(drawn.vertices[drawn.faces], facecolors=np.clip(np.array([0.36, 0.40, 0.45])[None] * sh[:, None] + 0.1, 0, 1), linewidths=0))
    env = A.arm_to_drone(tm(B["env"]), F)
    ax.add_collection3d(Poly3DCollection(env.vertices[env.faces], facecolors=(0.5, 0.7, 0.8, 0.06), linewidths=0))
    if what == "node":
        c = np.array(A.SPOKE_NODE[w]); tag = "1" if w == "front" else "2"
        for kk, (p, q) in links.items():
            if kk.startswith(f"spoke {tag}."):
                far = np.array(q if abs(p[1]) == 139.5 else p); u = (far - c) / np.linalg.norm(far - c)
                ax.plot(*zip(c + 16 * u, c + 60 * u), color="#c0392b", lw=4)
        M = A.motors_xy()[0 if w == "front" else 1]
        ax.plot([M[0]] * 2, [M[1]] * 2, [-40, 45], color="#2e86c1", ls="--", lw=1.5)
    else:
        c = np.array([72.95, 62.11, 34.0])
        for kk, e in A.BRACE_END["front"]:
            p, q = map(np.array, links[kk]); far = p if p[0] > q[0] else q; e = np.array(e); u = (far - e) / np.linalg.norm(far - e)
            ax.plot(*zip(e + 16 * u, e + 60 * u), color="#d68910", lw=4)
    r = 45
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r * 0.6, c[2] + r * 0.6)
    ax.set_box_aspect((1, 1, 0.6)); ax.view_init(22, -130); ax.set_axis_off(); ax.set_title(title, fontsize=10)
fig.suptitle("Overseer arms v2: the cage nodes as drawn into the arms (grey: drawn sockets 6.2 x 15 for the 6 mm tubes, wall 2.5; "
             "red / orange: tube stubs; blue dashes: the motor screw key column, kept clear)\nthe topology grows the arm round "
             "these; one printed part per arm (Spark)", fontsize=10)
fig.tight_layout(); out = os.path.join(ROOT, "runs", "overseer_arms", "v2", "nodes.png"); fig.savefig(out, facecolor="white"); print(out)
