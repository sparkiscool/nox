"""Preview images: the foot pad, and the stub spider v1 spec (assembled drawn set, 4 quadrants, PSU ghost, tie tabs).
python render_spec.py -> out/foot_pad_v1.png, out/stub_spider_v1_spec.png"""
import numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import spider_spec as S, pad_v1

BG, INK, MUTE = "#f6f4ef", "#23262b", "#8a8f98"
PETG, TPU, GHOST, KEEP = "#2f6f8f", "#d9822b", "#b9bec7", "#c8553d"
L = np.array([0.35, -0.55, 0.75]); L /= np.linalg.norm(L)


def tmesh(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts))


def draw(ax, t, color, alpha=1.0, edge=3.0, cull=True):
    v, f = trimesh.remesh.subdivide_to_size(t.vertices, t.faces, max_edge=edge)
    t = trimesh.Trimesh(v, f, process=False); tri = v[f]; n = t.face_normals
    if cull and alpha >= 1.0:                                   # back faces off: matplotlib sorts by centroid only
        el, az = np.radians(ax.elev), np.radians(ax.azim)
        eye = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        keep = n @ eye > -0.02; tri = tri[keep]; n = n[keep]
    shade = 0.45 + 0.55 * np.clip(n @ L, 0, 1)
    c = np.array(matplotlib.colors.to_rgb(color))
    fc = np.clip(c[None] * shade[:, None] + (1 - shade[:, None]) * 0.08, 0, 1)
    pc = Poly3DCollection(tri, facecolors=np.c_[fc, np.full(len(fc), alpha)], edgecolors="none", linewidths=0)
    ax.add_collection3d(pc)


def frame(ax, lo, hi, elev, azim):
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_facecolor(BG)


def pad_png():
    import zrender as Z
    t = tmesh(pad_v1.pad())
    fig = plt.figure(figsize=(10, 4.8), facecolor=BG)
    for i, (el, az, ttl) in enumerate(((30, -55, "stem up, as printed (no supports)"), (-32, -55, "contact face, 45° bed chamfer"))):
        ax = fig.add_axes([0.02 + 0.49 * i, 0.04, 0.47, 0.8]); ax.imshow(Z.render([(t, TPU, 1.0)], el, az, 700, bg=BG))
        ax.set_axis_off(); ax.set_title(ttl, color=MUTE, fontsize=10)
    fig.suptitle("Foot pad v1   TPU 95A   4.6 g (M)   30 × 30 × 4 + barb stem   ×4", color=INK, fontsize=13, x=0.05, ha="left")
    fig.savefig("out/foot_pad_v1.png", dpi=150, bbox_inches="tight", facecolor=BG); plt.close(fig)


def spider_png():
    P = S.parts()
    q = tmesh(P["drawn"] - P["cuts_exact"])
    quads = []
    for k in range(4):
        R = np.eye(4); c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k]
        R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, -s, s, c          # (x, z) -> (x c - z s, x s + z c)
        quads.append(q.copy().apply_transform(R))
    psu = trimesh.creation.box(extents=(215, 30, 115)); psu.apply_translation((0, -23, -50))
    skr = trimesh.creation.box(extents=(180, 30, 80)); skr.apply_translation((0, -23, 50))
    floor = trimesh.creation.box(extents=(380, 260, 0.5)); floor.apply_translation((0, 0, -160.25))
    import zrender as Z
    floor = trimesh.creation.box(extents=(360, 200, 1.0)); floor.apply_translation((0, 0, -160.5))
    bnd = np.array([[x, y, z] for x in (-170, 170) for y in (-95, 95) for z in (-165, 165)], float)
    fig = plt.figure(figsize=(14, 7.2), facecolor=BG)
    for i, (el, az, ttl) in enumerate(((20, -125, "from behind (stub side): back feet, electronics bay"),
                                       (16, -40, "from the arm side: flange seat Ø92, tie tabs to the ring at Y 88"))):
        items = [(floor, "#e6e2d9", 1.0)] + [(m, PETG if j == 0 else "#6f9db3", 1.0) for j, m in enumerate(quads)] \
            + [(psu, GHOST, 0.30), (skr, GHOST, 0.20)]
        ax = fig.add_axes([0.01 + 0.495 * i, 0.06, 0.485, 0.82])
        ax.imshow(Z.render(items, el, az, 900, bg=BG, bounds=bnd)); ax.set_axis_off()
        ax.set_title(ttl, color=MUTE, fontsize=10)
    fig.suptitle("Stub spider v1: drawn set only (generation pending)   4 identical quadrants, dark = the one STL",
                 color=INK, fontsize=13, x=0.03, ha="left")
    fig.text(0.03, 0.02, "Grey ghosts: PSU LRS-350-24 (Z −50) and the SKR + chargers carrier (Z +50) on the same 12-boss C4 orbit. "
             "Floor Z −160.", color=MUTE, fontsize=9)
    fig.savefig("out/stub_spider_v1_spec.png", dpi=140, bbox_inches="tight", facecolor=BG); plt.close(fig)


if __name__ == "__main__":
    import os; os.makedirs("out", exist_ok=True)
    pad_png(); spider_png(); print("ok")
