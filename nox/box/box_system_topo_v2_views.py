"""Render the eta 0.5 iso-surface of runs/box_system/v2/topo/density.npz with receivers, catch and sheets ghosted,
and write density statistics for the summary (per-piece analysis helpers). python3 box_system_topo_v2_views.py"""
import os, sys, json
import numpy as np, trimesh
from skimage import measure
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "runs", "box_system", "v2", "topo"); LAY = os.path.join(ROOT, "runs", "box_system", "v2", "layout")


def field():
    d = np.load(os.path.join(OUT, "density.npz"))
    h = float(d["h"]); o = np.asarray(d["origin"]); sh = tuple(d["shape"])
    rho = d["rho_n"].reshape(sh)
    return d, h, o, sh, rho


def iso(rho, h, o):
    r = np.pad(rho, 1)
    v, f, n, _ = measure.marching_cubes(r, 0.5, spacing=(h, h, h))
    v = v + o - h * 0.5 * 0 - h + h * 0.5                                   # cell centres at o + (i + .5) h; pad shifts by -h
    m = trimesh.Trimesh(v, f)
    return m


def ghost(name, simplify=0.1):
    t = trimesh.load(os.path.join(LAY, name), force="mesh")
    if len(t.faces) > 6000:
        try: t = t.simplify_quadric_decimation(face_count=6000)
        except Exception: pass
    return t


def draw(ax, m, color, alpha, light=True, lw=0.0):
    tri = m.vertices[m.faces]
    n = m.face_normals
    c = np.array(matplotlib.colors.to_rgb(color))
    sh = 0.55 + 0.45 * np.clip(n @ np.array([0.3, -0.5, 0.8]) / 1.0, -1, 1)
    cols = np.clip(c[None] * sh[:, None], 0, 1)
    pc = Poly3DCollection(tri, alpha=alpha, linewidths=lw)
    pc.set_facecolor(np.c_[cols, np.full(len(cols), alpha)]); pc.set_edgecolor("none")
    ax.add_collection3d(pc)


def view(parts, elev, azim, path, title, lim):
    fig = plt.figure(figsize=(9, 9)); ax = fig.add_subplot(111, projection="3d")
    for m, col, a in parts: draw(ax, m, col, a)
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2])
    ax.set_box_aspect([lim[k][1] - lim[k][0] for k in range(3)])
    ax.view_init(elev, azim); ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z"); ax.set_title(title)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def main():
    d, h, o, sh, rho = field()
    m = iso(rho, h, o); m.export(os.path.join(OUT, "iso05.stl"))
    L = json.load(open(os.path.join(LAY, "layout.json"))); EN = L["enclosure"]
    rec = ghost("receivers_W.stl"); catch = ghost("catch_W.stl")
    sheets = []
    X, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
    for b in ([-X, X, Y0, Y0 + 3, Z0, Z1], [X - 3, X, Y0, Y1, Z0, Z1], [-X, -X + 3, Y0, Y1, Z0, Z1]):
        sheets.append(trimesh.creation.box(extents=[b[1] - b[0], b[3] - b[2], b[5] - b[4]],
                                           transform=trimesh.transformations.translation([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])))
    lim = [(-X, X), (Y0, Y1), (Z0, Z1)]
    G = lambda: [(m, "#2a6fdb", 1.0), (rec, "#999999", 0.25), (catch, "#e08a1e", 0.22)] + [(s, "#88aacc", 0.05) for s in sheets[:1]]
    names = dict(front=(0, -90), side=(0, 0), top=(90, -90), iso=(25, -55), back_iso=(25, 125))
    for n, (el, az) in names.items():
        view(G(), el, az, os.path.join(OUT, f"view_{n}.png"), f"eta 0.5, {n}", lim)
    print("views done", len(m.faces))


if __name__ == "__main__":
    main()
