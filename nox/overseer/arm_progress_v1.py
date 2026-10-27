"""Overseer arms v1: picture of a run in progress from its committed ck.npz (density > 0.5, smoothed).
python3 nox/overseer/arm_progress_v1.py <dir with prog_front/ck.npz, prog_rear/ck.npz> <out.png>"""
import sys, os, numpy as np, trimesh
sys.path.insert(0, "nox/overseer"); sys.path.insert(0, "nox")
import arm_topo_v1 as A
from topo.part import tm
from scipy import ndimage as ndi
from skimage import measure
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
S = sys.argv[1]
H = {"front": 2.0, "rear": 1.5}
parts, info = [], []
for w in ("front", "rear"):
    p = A.setup_for(w)(H[w])
    skin = p.functional_skin(2.5)
    design = p.design | (p.solid & ~skin)
    d = np.load(f"{S}/prog_{w}/ck.npz", allow_pickle=True)
    x = d["x"]; it = int(d["it"])
    assert design.sum() == len(x), (design.sum(), len(x))
    f = np.zeros(p.nel); f[design] = x; f[skin & ~p.insert_cells] = 1.0
    f3 = ndi.gaussian_filter(f.reshape(p.shape), 0.7)
    v, fc, _, _ = measure.marching_cubes(np.pad(f3, 1), 0.5)
    v = (v - 1) * p.h + np.array(p.origin) + p.h / 2
    m = trimesh.Trimesh(v, fc); B, F = A.bodies(w)
    vol = (f > 0.5).sum() * p.h ** 3 / 1000
    info.append(f"{w}: iteration {it}, ~{vol:.0f} cm³ solid so far")
    for mir in (False, True):
        parts.append((A.arm_to_drone(m, F, mir), "#b9c3cc"))
        if w == "front":
            parts.append((A.arm_to_drone(tm(B["rods"]), F, mir), "#1d2226"))
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
fig = plt.figure(figsize=(16, 6.4), dpi=130)
for k, (title, el, az) in enumerate((("iso", 28, -125), ("plan (nose to the right)", 90, -90), ("side (from -y)", 0, -90))):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    for m, col in parts:
        sh = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col))
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=np.clip(base[None] * sh[:, None] + 0.05, 0, 1), linewidths=0))
    for pl in ("top_plate.stl", "bottom_plate.stl"):
        t = trimesh.load(os.path.join(A.LAYOUT, "parts", pl), force="mesh")
        ax.add_collection3d(Poly3DCollection(t.vertices[t.faces], facecolors=(0.6, 0.65, 0.7, 0.15), linewidths=0))
    ax.set_xlim(-130, 230); ax.set_ylim(-180, 180); ax.set_zlim(-60, 120); ax.set_box_aspect((360, 360, 180))
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=11)
fig.suptitle("Overseer arms v1: topology IN PROGRESS (density > 0.5 at the last checkpoint, still grey and changing; not the result)\n"
             + "   ·   ".join(info) + "   ·   dark = 3.0 carbon rods", fontsize=11)
fig.tight_layout(); fig.savefig(sys.argv[2], facecolor="white"); print(info)
