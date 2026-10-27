"""Overseer layout v5 stills from the assembly's part meshes (iso, top, from below), so the stills follow every republish
(CLAUDE.md "Cage sessions keep their parts in sync"). Each part decimated for drawing only.

python3 nox/overseer/layout_v5_stills.py -> runs/overseer_layout/v5/preview.png, preview_top.png, preview_under.png
"""
import os, json
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "runs", "overseer_layout", "v5")
ASM = json.load(open(os.path.join(HERE, "overseer.assembly.json")))
ROLE_COL = {"cage": "#3a4a5c", "structure": "#8da2b8", "bought": "#3f8f4f", "protected": "#c7ced6", "sensor": "#222222"}
VIEWS = {"preview.png": (22, -60, "iso"), "preview_top.png": (90, -90, "top"), "preview_under.png": (-90, -90, "from below")}


def parts():
    out = []
    for p in ASM["parts"]:
        f = p.get("mesh") or p.get("file") or p.get("stl")
        if not f: continue
        f = f if os.path.isabs(f) else os.path.join(ROOT, f) if os.path.exists(os.path.join(ROOT, f)) else os.path.join(HERE, f)
        if not os.path.exists(f): continue
        m = trimesh.load(f, force="mesh")
        if len(m.faces) > 4000: m = m.simplify_quadric_decimation(face_count=4000)
        col = ROLE_COL.get(p.get("role"), "#8899aa") if not p["name"].startswith("sensor head") else "#c9a227"
        out.append((p["name"], m, col))
    return out


def main():
    P = parts(); light = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
    allv = np.vstack([m.vertices for _, m, _ in P]); c = (allv.max(0) + allv.min(0)) / 2; R = (allv.max(0) - allv.min(0)).max() / 2
    for fn, (el, az, ttl) in VIEWS.items():
        fig = plt.figure(figsize=(11, 9)); ax = fig.add_subplot(111, projection="3d")
        for name, m, col in P:
            rgb = np.array(matplotlib.colors.to_rgb(col)); sh = np.clip(np.abs(m.face_normals @ light), 0, 1) * 0.55 + 0.4
            pc = Poly3DCollection(m.vertices[m.faces], linewidths=0); pc.set_facecolor(np.c_[np.outer(sh, rgb), np.ones(len(sh))])
            ax.add_collection3d(pc)
        ax.set_xlim(c[0] - R, c[0] + R); ax.set_ylim(c[1] - R, c[1] + R); ax.set_zlim(c[2] - R, c[2] + R); ax.set_box_aspect((1, 1, 1))
        ax.view_init(el, az); ax.set_axis_off()
        ax.set_title(f"Overseer layout v5, {ttl} (assembly {os.path.basename(HERE)}/overseer.assembly.json; saved parts where they exist)", fontsize=9)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, fn), dpi=100); plt.close(fig); print(os.path.join(OUT, fn))


if __name__ == "__main__":
    main()
