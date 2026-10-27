"""Overseer core v3 pictures: runs/overseer_core/v3/preview.png (the core with the front hub tower, keels and braces) and
alternatives.png (the L2 braces: A a block on the top plate's front edge vs B the tower on the hub deck; side and plan)."""
import json, trimesh, numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
d = json.load(open("nox/overseer/overseer_core.assembly.json")); P = {p["name"]: p for p in d["parts"]}
L = lambda n: trimesh.load(P[n]["mesh"])
r = json.load(open("runs/overseer_core/v3/result.json"))
lt = np.array([0.3, -0.45, 0.85]); lt /= np.linalg.norm(lt)


def draw(ax, m, col, a=1.0, maxf=20000):
    F, N = (m.faces, m.face_normals)
    if len(F) > maxf:
        idx = np.random.default_rng(0).choice(len(F), maxf, replace=False); F, N = F[idx], N[idx]
    sh = 0.42 + 0.58 * np.clip(np.abs(N @ lt), 0, 1); c = np.array(matplotlib.colors.to_rgb(col))[None] * sh[:, None]
    ax.add_collection3d(Poly3DCollection(m.vertices[F], facecolors=np.c_[c, np.full(len(c), a)], linewidths=0))


keels = [np.array(v) for v in r["keels"].values()]; braces = [np.array(v) for v in r["braces"].values()]
groups = [("bottom plate", "#c9ced6", 0.35), ("top plate", "#c9ced6", 0.25), ("arm 1", "#b8bec7", 0.25), ("arm 4", "#b8bec7", 0.25),
          ("UBEC 5 V", "#6b4fa0", 1), ("buck 12 V", "#a04f8a", 1), ("Pi 5 + Active Cooler", "#2e7d4f", 0.9), ("hub deck", "#e8833a", 1), ("flow clip", "#2f7fd1", 1)]
fig = plt.figure(figsize=(16, 7), facecolor="white")
for i, (el, az, t) in enumerate([(22, -60, "iso, nose right"), (0, -90, "side: keels low, braces high, one tower"), (90, -90, "plan")]):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    for n, c, a in groups:
        if i == 2 and n == "top plate": continue
        draw(ax, L(n), c, a)
    for a, b in keels: e = a + (b - a) * 0.25; ax.plot(*np.c_[a, e], color="#111", lw=3)
    for a, b in braces: e = a + (b - a) * 0.25; ax.plot(*np.c_[a, e], color="#c0392b", lw=2.5)
    ax.set_xlim(-60, 180); ax.set_ylim(-90, 90); ax.set_zlim(-20, 80); ax.set_box_aspect((240, 180, 100)); ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t, fontsize=12)
fig.suptitle("Overseer core v3 concept: the hub deck's front hub becomes a tower (orange) carrying the 2 keels (black) and the 4 L2 braces (red) toward the head;\n"
             "the keels' push and the braces' pull meet in one piece. Top plate, Pi, UBEC, buck, flow clip as v2.", fontsize=11.5)
plt.tight_layout(rect=(0, 0, 1, 0.9)); plt.savefig("runs/overseer_core/v3/preview.png", dpi=105)

# alternatives, side view x-z and the numbers
fig2, axs = plt.subplots(1, 2, figsize=(16, 6.2), facecolor="white")
plates = [(-95.5, 100.5, -2.1, 0.0), (-60.2, 56.5, 45.0, 46.6)]
A = {"lo": (57, 50), "hi": (53, 50)}; B = {"lo": (97, 22), "hi": (97, 32)}
titles = ["A: block on the top plate's front edge (roots x 53..57, z 50)\nSF 1.1, the cage's per-case forces: 765..788 N on each of 4 fasteners\nin the 1.6 top plate (584 shear): not sound in PETG",
          "B (chosen): tower on the hub deck's front hub (roots x 97, z 22 / 32)\n+ struts to 2 pads under the top plate. SF 1.1: forward tip -> 343 N a pad (5.4 MPa, bearing);\nback tip 13.8 N m a side -> ~445 N per front insert (E); 6 feet <= 247 N (rigid)"]
for k, ax in enumerate(axs):
    for x0, x1, z0, z1 in plates: ax.fill([x0, x1, x1, x0], [z0, z0, z1, z1], color="#9aa4b1")
    ax.fill([34, 82, 82, 34], [6.5, 6.5, 15.5, 15.5], color="#6b4fa0", alpha=0.7)
    ax.plot([96, 96 + (379.7 - 96) * 0.25], [8, 8 + (54.2 - 8) * 0.25], color="#111", lw=4)
    roots = A if k == 0 else B
    for lvl, (x, z) in roots.items():
        lz = 54.2 if lvl == "lo" else 67.8; lx = 379.7 if lvl == "lo" else 383.3
        ax.plot([x, x + (lx - x) * 0.25], [z, z + (lz - z) * 0.25], color="#c0392b", lw=3)
    if k == 0:
        ax.fill([22, 62, 62, 22], [46.6, 46.6, 56, 56], color="#e8833a", alpha=0.9)
        ax.fill([-20, 110, 110, -20], [0, 0, 6.5, 6.5], color="#e8833a", alpha=0.5)
    else:
        ax.fill([-20, 110, 110, -20], [0, 0, 6.5, 6.5], color="#e8833a", alpha=0.9)
        ax.fill([86, 112, 112, 86], [0, 0, 38, 38], color="#e8833a", alpha=0.9)
        ax.plot([88, 46], [30, 40.5], color="#e8833a", lw=8, solid_capstyle="round"); ax.fill([42, 50, 50, 42], [41, 41, 45, 45], color="#e8833a")
        ax.annotate("pad bears up on the top plate", (46, 45), (0, 62), arrowprops=dict(arrowstyle="->"), fontsize=9)
    ax.set_xlim(-30, 200); ax.set_ylim(-10, 80); ax.set_aspect("equal"); ax.set_title(titles[k], fontsize=9.5); ax.set_xlabel("x (nose right), side view")
fig2.suptitle("L2 brace roots: alternatives (grey = S500 plates, purple = UBEC, black = keel, red = braces, orange = printed core)", fontsize=12)
plt.tight_layout(rect=(0, 0, 1, 0.92)); plt.savefig("runs/overseer_core/v3/alternatives.png", dpi=100)
