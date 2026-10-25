"""Overseer system picture: the assembly file's parts in 4 views (iso, side, front, top), coloured by role.

python3 nox/overseer/render_system.py [nox/overseer/overseer.assembly.json] [out.png] ["title"]
RENDER_BOX="x0,x1,y0,y1,z0,z1" crops every part to that box (a close-up, e.g. the nose).
"""
import json, os, sys
import numpy as np, trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
COL = {"cage": "#2b2b2b", "L2": "#d9822b", "front cameras": "#c0392b", "props": "#9ecae1", "motors": "#555555",
       "pack": "#3a7d44", "cartridge": "#7fbf7f", "receiver": "#9aa5b1", "receiver XT60": "#e3c14b", "electronics": "#2f4fbf",
       "down cam + flow": "#c0392b", "arms": "#8f7a66", "plates": "#4a4a4a", "sled + pods + deck": "#b8b8b8",
       "apex + hubs": "#a0896b", "cam brackets": "#a0896b"}
ALPHA = {"props": 0.18, "cage": 1.0}


def col(name):
    return COL.get(name) or (COL["L2"] if name.startswith("L2") else "#999999")   # L2 base, top band and head


def load(asm_path):
    d = json.load(open(asm_path)); out = []
    for p in d["parts"]:
        m = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
        if len(m.faces) > 30000:
            try: m = m.simplify_quadric_decimation(face_count=30000)
            except Exception: pass
        out.append((p["name"], m))
    return d, out


def draw(ax, parts, elev, azim, title):
    light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    allv = np.vstack([m.vertices for _, m in parts])
    for name, m in parts:
        c = np.array(matplotlib.colors.to_rgb(col(name)))
        sh = 0.45 + 0.55 * np.clip(np.abs(m.face_normals @ light), 0, 1)
        fc = np.clip(c[None, :] * sh[:, None], 0, 1)
        pc = Poly3DCollection(m.vertices[m.faces], facecolors=np.c_[fc, np.full(len(fc), ALPHA.get(name, 1.0))], linewidths=0)
        ax.add_collection3d(pc)
    lo, hi = allv.min(0), allv.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=1.4); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=11)


def main():
    asm = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "nox", "overseer", "overseer.assembly.json")
    out = sys.argv[2] if len(sys.argv) > 2 else "overseer_system.png"
    title = sys.argv[3] if len(sys.argv) > 3 else "Overseer system"
    d, parts = load(asm)
    if os.environ.get("RENDER_BOX"):                                  # close-up: every part cut to the box
        b = [float(v) for v in os.environ["RENDER_BOX"].split(",")]; cut = []
        for name, m in parts:
            for ax_ in range(3):
                for sg, v in ((1, b[2 * ax_]), (-1, b[2 * ax_ + 1])):
                    if len(m.faces): n_ = np.zeros(3); n_[ax_] = sg; o_ = np.zeros(3); o_[ax_] = v; m = m.slice_plane(o_, n_)
            if len(m.faces): cut.append((name, m))
        parts = cut
    fig = plt.figure(figsize=(16, 12), dpi=110)
    for i, (el, az, t) in enumerate(((22, -55, "iso, front left"), (0, -90, "side (nose right)"), (0, 0, "front"), (90, -90, "top (nose right)"))):
        draw(fig.add_subplot(2, 2, i + 1, projection="3d"), parts, el, az, t)
    handles = [matplotlib.patches.Patch(color=col(n), label=n) for n, _ in parts]
    fig.legend(handles=handles, loc="lower center", ncol=8, fontsize=9, frameon=False)
    fig.suptitle(title, fontsize=14); fig.subplots_adjust(left=0, right=1, top=0.94, bottom=0.06, wspace=0, hspace=0.02)
    fig.savefig(out); print("wrote", out)


if __name__ == "__main__":
    main()
