"""Top and iso view of the two cage v2 print plates (A1 mini 180 bed)."""
import sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
D = sys.argv[1]; out = sys.argv[2]
fig = plt.figure(figsize=(16, 8.4), dpi=120, facecolor="#f4f2ee")
for i, (mat, col) in enumerate((("PETG", "#d9cdb4"), ("TPU", "#8fa3b8"))):
    sc = trimesh.load(f"{D}/s2_cage_v2_{mat}_plate.3mf", force="scene")
    ax = fig.add_subplot(1, 2, i + 1, projection="3d", facecolor="#f4f2ee", computed_zorder=False)
    bed = [[(0, 0, -0.3), (180, 0, -0.3), (180, 180, -0.3), (0, 180, -0.3)]]
    ax.add_collection3d(Poly3DCollection(bed, facecolors="#2b2b2e", edgecolors="#444", alpha=1.0, zorder=0))
    n = 0; vol = 0.0
    for g in sc.dump():
        n += 1; vol += abs(g.volume)
        if len(g.faces) > 4000: g = g.simplify_quadric_decimation(face_count=4000)
        tri = g.vertices[g.faces]; nn = g.face_normals
        L = np.array([0.35, -0.45, 0.82]); L /= np.linalg.norm(L); sh = 0.42 + 0.58 * np.clip(nn @ L, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col)); ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(base * sh[:, None], 0, 1), edgecolors="none", zorder=2))
    ax.set_xlim(0, 180); ax.set_ylim(0, 180); ax.set_zlim(0, 60); ax.set_box_aspect((180, 180, 60), zoom=1.1); ax.view_init(52, -62); ax.set_axis_off()
    dens = 1.27 if mat == "PETG" else 1.26
    ax.set_title(f"{mat} plate: {n} parts, {vol * dens / 1000:.1f} g solid (M), no supports", fontsize=12, color="#222")
fig.suptitle("S2 cage v2 print set (one drone), A1 mini 180 bed", fontsize=13, x=0.02, ha="left", color="#222")
fig.subplots_adjust(left=0, right=1, bottom=0, top=0.92, wspace=0); fig.savefig(out)
