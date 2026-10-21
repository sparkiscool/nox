"""Clean orthographic renders (painter's algorithm on a subdivided mesh) for Nox part and assembly visuals.
Usage: python render_views.py out.png title mesh1.stl[:color[:alpha]] mesh2.stl ...   (W-frame meshes; views: iso, front(-Y), side(+X), top)"""
import sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

COLORS = dict(petg=(0.16, 0.52, 0.49), al=(0.72, 0.74, 0.76), module=(0.86, 0.68, 0.36), ink=(0.2, 0.22, 0.24), ghost=(0.75, 0.78, 0.8))

def load(spec):
    parts = spec.split(":"); m = trimesh.load(parts[0], force="mesh")
    col = COLORS.get(parts[1], COLORS["petg"]) if len(parts) > 1 else COLORS["petg"]
    alpha = float(parts[2]) if len(parts) > 2 else 1.0
    if m.area_faces.max() > 40: m = trimesh.Trimesh(*trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=6.0), process=False)
    return m, col, alpha

def view(ax, meshes, R, light, title):
    polys, cols, depth = [], [], []
    for m, col, alpha in meshes:
        v = m.vertices @ R.T; tri = v[m.faces]
        n = m.face_normals @ R.T
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        c = np.clip(np.array(col)[None, :] * shade[:, None], 0, 1)
        polys.append(tri[:, :, :2]); cols.append(np.c_[c, np.full(len(c), alpha)]); depth.append(tri[:, :, 2].mean(1))
    P = np.concatenate(polys); C = np.concatenate(cols); D = np.concatenate(depth); o = np.argsort(D)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors=C[o], linewidths=0.15))
    ax.autoscale(); ax.set_aspect("equal"); ax.axis("off"); ax.set_title(title, fontsize=9, loc="left", color="#1d1d1f")

def rot(elev, azim):
    a, e = np.radians(azim), np.radians(elev)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    return Rx @ Rz

if __name__ == "__main__":
    out, title, specs = sys.argv[1], sys.argv[2], sys.argv[3:]
    meshes = [load(s) for s in specs]
    # W frame: Z up. Views are built so screen y = up. front: look along +Y; side: look along -X; top: look down -Z.
    base = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)        # screen x = X, screen y = Z, depth toward +Y (viewer at -Y)
    views = [("iso", rot(-25, 0) @ rot(0, 0) @ np.array([[np.cos(np.radians(35)), -np.sin(np.radians(35)), 0], [0, 0, 1], [np.sin(np.radians(35)), np.cos(np.radians(35)), 0]], float)),
             ("front, from -Y", base),
             ("side, from +X", np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)),
             ("top", np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], float))]
    fig, axs = plt.subplots(2, 2, figsize=(12, 9), dpi=120)
    light = np.array([0.3, 0.5, 0.8]); light /= np.linalg.norm(light)
    for ax, (nm, R) in zip(axs.ravel(), views):
        # depth must be toward the viewer: flip so larger = nearer
        R2 = R.copy(); R2[2] *= -1
        view(ax, meshes, R2, light, nm)
    fig.suptitle(title, fontsize=11, x=0.02, ha="left", color="#1d1d1f")
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.92, wspace=0.05, hspace=0.12)
    fig.savefig(out, facecolor="white"); print(out)
