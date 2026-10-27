"""Overseer front arm v6 setup view (arm frame): drawn bodies (grey), the shoulder wedges (blue-grey), the 4 chord tubes
(black), the 12 diagonals (orange) with their insertion lines (dashed, from the through end out of the arm), the envelope
(light blue). Writes NOX_VIEW_OUT (default runs/overseer_arms/v6/layout.png).

python3 nox/overseer/arm_view_v6.py
"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for k, v in (("NOX_ARM_CAP", "1"), ("NOX_ARM_SLEEVE", "1"), ("NOX_ARM_SHFLANGE", "1"), ("NOX_ARM_BRACES", "0"),
             ("NOX_ARM_LINKS", "1"), ("NOX_ARM_ROOT_U1", "84"), ("NOX_ARM_ROOT_SOLID", "1"), ("NOX_ARM_FLANGE_X", "40"),
             ("NOX_ARM_CHORD", "tube6"), ("NOX_ARM_ROD_Z", "38.5,6.5"),
             ("NOX_ARM_JOINT", os.path.join(ROOT, "runs", "overseer_arms", "v5", "arm_loads.json")),
             ("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5")),
             ("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))):
    os.environ.setdefault(k, v)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import arm_topo_v6  # noqa: F401
import numpy as np
import arm_topo_v1 as A
from topo.part import tm
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

B, F = A.bodies("front")
D = A.diagonals("front", F)
chord_rods = A.mf.Manifold()
for p0, p1 in A.chords("front", F):
    chord_rods += A.rod_m(p0, p1, A.ROD_D / 2)
diag_rods = A.mf.Manifold()
for d in D:
    diag_rods += A.rod_m(d["p_blind"], d["p_open"], d["d"] / 2)
wedge = B["shweb"]
other = (B["drawn"] - B["cuts"]) - wedge
parts = [(tm(other), "#8a939c", 0.28), (tm(wedge - B["cuts"]), "#5d7a96", 0.75), (tm(chord_rods), "#111417", 1.0),
         (tm(diag_rods), "#e67e22", 1.0), (tm(B["env"]), "#7fb3c9", 0.07)]
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
wi = os.path.join(ROOT, "runs", "overseer_arm_front", "v6", "whatif_modes.json")
wtxt = ""
if os.path.exists(wi):
    w = json.load(open(wi))
    wtxt = (f"modal what-if on the v5d mesh (4 inserts, h {w['h']}): rods + nodes + wedge "
            f"{w['rods_and_nodes']['modes'][0]['Hz']:.0f} Hz vertical / {w['rods_and_nodes']['modes'][1]['Hz']:.0f} lateral "
            f"(v5d 131 / 162, target 190)")
fig = plt.figure(figsize=(17, 9), dpi=120)
views = (("iso", 28, -125), ("plan (motor to the right)", 90, -90), ("side (from -y)", 0, -90), ("end (from the motor)", 0, 0))
for k, (title, el, az) in enumerate(views):
    ax = fig.add_subplot(2, 2, k + 1, projection="3d")
    for m, col, al in parts:
        sh = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col))
        fc = np.clip(base[None] * sh[:, None] + 0.05, 0, 1)
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=np.c_[fc, np.full(len(fc), al)], linewidths=0))
    for d in D:
        ax.plot(*zip(d["p_open"] - 25.0 * (d["p_blind"] - d["p_open"]) / np.linalg.norm(d["p_blind"] - d["p_open"]), d["p_open"]),
                color="#c0392b", lw=1.0, ls="--")
    ax.plot([F["L"]] * 2, [0, 0], [-10, 60], color="#2e86c1", lw=1.2, ls="--")
    ax.set_xlim(-30, 215); ax.set_ylim(-60, 60); ax.set_zlim(-20, 65); ax.set_box_aspect((245, 120, 85), zoom=1.2)
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=11)
fig.suptitle("Overseer front arm v6 setup: 4 x 6/4 chords (black) + 12 x 3.0 carbon diagonals (orange, Warren, 3 panels a "
             "face, 7 off each face, blind at the motor side, slid in along the red dashes), drawn sleeves and node blocks "
             "(grey), solid shoulder wedges (blue)\n" + wtxt, fontsize=10)
fig.tight_layout()
out = os.environ.get("NOX_VIEW_OUT", os.path.join(ROOT, "runs", "overseer_arms", "v6", "layout.png"))
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, facecolor="white"); print(out)
