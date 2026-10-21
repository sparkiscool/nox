"""Nox print preview: one PNG per part, shown to Spark before any "ready to print".

Usage:
  python render_preview.py part.stl [--mat PETG|TPU] [--prev 31.2] [--out part_preview.png]
  (--prev = previous version's mass in g, so the sheet shows the change)

Mesh is shown as exported: +z up = print orientation, z min = the bed.
Red faces = overhangs steeper than 45 deg (need support or a fix). Blue = bed contact.
Needs: pip install --break-system-packages trimesh matplotlib numpy
"""
import argparse, os
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

DENSITY = {"PETG": 1.27, "TPU": 1.21, "PLA": 1.24}  # g/cm3, solid (E)

def load(path):
    m = trimesh.load(path, force="mesh")
    m.merge_vertices()
    return m

def face_colors(m, light):
    n = m.face_normals
    shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
    base = np.tile([0.80, 0.80, 0.78], (len(n), 1))
    zmin = m.bounds[0][2]
    down = n[:, 2] < -np.cos(np.radians(45))          # steeper than 45 deg overhang
    on_bed = down & (m.triangles_center[:, 2] < zmin + 0.2)
    base[down & ~on_bed] = [0.90, 0.25, 0.20]
    base[on_bed] = [0.25, 0.45, 0.90]
    return np.clip(base * shade[:, None], 0, 1), down & ~on_bed, on_bed

def view(ax, m, elev, azim, title):
    az, el = np.radians(azim), np.radians(elev)
    cam = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    cols, _, _ = face_colors(m, cam / np.linalg.norm(cam))
    pc = Poly3DCollection(m.triangles, facecolors=cols, edgecolors=cols, linewidths=0.2, antialiased=False)
    ax.add_collection3d(pc)
    lo, hi = m.bounds
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=1.45)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=10)
    ax.set_axis_off()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("mesh")
    p.add_argument("--mat", default="PETG")
    p.add_argument("--prev", type=float, help="previous version mass, g")
    p.add_argument("--out")
    a = p.parse_args()

    m = load(a.mesh)
    name = os.path.splitext(os.path.basename(a.mesh))[0]
    vol = abs(m.volume) / 1000.0  # cm3
    mass = vol * DENSITY[a.mat.upper()]
    ext = m.extents
    _, over, bed = face_colors(m, np.array([0, 0, 1.0]))
    over_mm2 = m.area_faces[over].sum()
    bed_mm2 = m.area_faces[bed].sum()
    bodies = len(m.split(only_watertight=False))

    fig = plt.figure(figsize=(12, 9), dpi=110)
    for i, (e, az, t) in enumerate([(25, -60, "iso, as printed"), (-35, -60, "from below (red = overhang, blue = bed)"),
                                    (0, -90, "front (-y)"), (90, -90, "top")]):
        view(fig.add_subplot(2, 2, i + 1, projection="3d"), m, e, az, t)

    dm = f"  ({mass - a.prev:+.1f} g vs prev {a.prev:.1f} g)" if a.prev else ""
    lines = [
        f"{name}   {a.mat.upper()}   {mass:.1f} g solid (M){dm}",
        f"{vol:.2f} cm3   bbox {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm   "
        f"bodies {bodies}   watertight {'yes' if m.is_watertight else 'NO'}",
        f"bed contact {bed_mm2:.0f} mm2 (blue)   overhang >45 deg {over_mm2:.0f} mm2 (red)   "
        f"fits A1 mini 180: {'yes' if ext.max() <= 180 else 'NO'}",
    ]
    fig.suptitle("\n".join(lines), fontsize=11, family="monospace", x=0.02, ha="left")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.88, wspace=0, hspace=0.05)
    out = a.out or f"{name}_preview.png"
    fig.savefig(out)
    print(out)
    print("\n".join(lines))

if __name__ == "__main__":
    main()
