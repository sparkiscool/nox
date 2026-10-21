"""Preview of the generated stub spider: the quadrant STL as printed (bed = X 0 joint face) and the four quadrants
assembled with the pads, PSU ghost and floor. python render_result.py out/stub_spider_v1_q.stl <mass g> <worst>"""
import sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import zrender as Z, pad_v1, spider_spec as S

BG, INK, MUTE, PETG, PETG2, TPU, GHOST = "#f6f4ef", "#23262b", "#8a8f98", "#2f6f8f", "#6f9db3", "#d9822b", "#b9bec7"


def rotk(m, k):
    R = np.eye(4); c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k]
    R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, -s, s, c
    return m.copy().apply_transform(R)


def main(stl, mass, worst):
    q = trimesh.load(stl, force="mesh")
    me = pad_v1.pad().to_mesh(); pad = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts))
    x, y = S.FOOT
    pb = pad.copy(); pb.apply_translation((x, y, S.ZF))           # contact face on the floor, stem up into the seat
    ps = pad.copy(); ps.apply_translation((-x, y, S.ZF))          # quadrant 3's side seat is the other back foot
    psu = trimesh.creation.box(extents=(215, 30, 115)); psu.apply_translation((0, -23, -50))
    floor = trimesh.creation.box(extents=(380, 200, 1.0)); floor.apply_translation((0, 0, -160.5))
    quads = [rotk(q, k) for k in range(4)]
    fig = plt.figure(figsize=(15, 10), facecolor=BG)
    bnd = np.array([[a, b, c] for a in (-170, 170) for b in (-95, 95) for c in (-165, 165)], float)
    views = [((20, -125), "assembled, from behind: back feet, electronics bay"),
             ((16, -40), "assembled, from the arm side: flange seat, ties to the ring")]
    for i, ((el, az), ttl) in enumerate(views):
        items = [(floor, "#e6e2d9", 1.0)] + [(m, PETG if j == 0 else PETG2, 1.0) for j, m in enumerate(quads)] \
            + [(pb, TPU, 1.0), (ps, TPU, 1.0), (psu, GHOST, 0.28)]
        ax = fig.add_axes([0.01 + 0.495 * i, 0.47, 0.485, 0.46]); ax.set_axis_off()
        ax.imshow(Z.render(items, el, az, 900, bg=BG, bounds=bnd)); ax.set_title(ttl, color=MUTE, fontsize=10)
    qb = q.bounds; bq = np.array([[a, b, c] for a in qb[:, 0] for b in qb[:, 1] for c in qb[:, 2]])
    for i, ((el, az), ttl) in enumerate((((24, -50), "one quadrant (the STL), outside"), ((22, 140), "one quadrant, joint faces"))):
        ax = fig.add_axes([0.01 + 0.495 * i, 0.02, 0.485, 0.42]); ax.set_axis_off()
        ax.imshow(Z.render([(q, PETG, 1.0)], el, az, 800, bg=BG, bounds=bq)); ax.set_title(ttl, color=MUTE, fontsize=10)
    fig.suptitle(f"Stub spider v1   PETG   {mass} g per quadrant (M), ×4   worst index {worst}   bed: X 0 joint face, no supports",
                 color=INK, fontsize=14, x=0.02, ha="left")
    fig.savefig("out/stub_spider_v1.png", dpi=110, bbox_inches="tight", facecolor=BG); plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
