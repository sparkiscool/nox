"""Structure picture of cage v3 layouts from the frame model (F): members by type, props, plate, top case, crown.
Run: python3 layout_render.py out.png '<opt json>' ['<opt json>' ...]   (one column per layout)"""
import sys, json, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import layout_v3 as L, s2_guard_frame as sg

COL = {"skirt": "#2e7d4f", "lower": "#1d1d1f", "upper": "#1d1d1f", "strut": "#1d1d1f", "leg": "#1d1d1f", "spoke": "#c0392b",
       "node": "#b8a47e", "brace": "#b8a47e", "mouth": "#b8a47e", "post": "#b8a47e", "crown": "#d9822b",
       "tee": "#5a7fa8", "case": "#9aa5b1", "foot": "#5a7fa8", "link": "#5a7fa8"}
LW = {"skirt": 2.8, "spoke": 2.6, "crown": 3.4, "case": 1.2}
def box_faces(lo, hi):
    x0, y0, z0 = lo; x1, y1, z1 = hi
    P = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    return [P[[0, 1, 2, 3]], P[[4, 5, 6, 7]], P[[0, 1, 5, 4]], P[[2, 3, 7, 6]], P[[1, 2, 6, 5]], P[[0, 3, 7, 4]]]
def draw(ax, g, elev, azim):
    t = np.linspace(0, 2 * np.pi, 90)
    for mx, my in sg.MOTORS:
        ax.add_collection3d(Poly3DCollection([np.c_[mx + 44.45 * np.cos(t), my + 44.45 * np.sin(t), np.full_like(t, L.PZ)]],
                                             facecolors="#9cc7ea", edgecolors="#4a90c8", alpha=0.25, linewidths=0.8))
    ax.add_collection3d(Poly3DCollection(box_faces((-50, -47, 18.5), (50, 47, 22)), facecolors="#444", alpha=0.35, edgecolors="none"))
    ax.add_collection3d(Poly3DCollection(box_faces((-15, -33, 22), (15, 33, L.BOSS_TOP)), facecolors="#7d8a96", alpha=0.18, edgecolors="#7d8a96", linewidths=0.4))
    for e in g.elems:
        a, b = g.X[e["n"][0]], g.X[e["n"][1]]; tg = e["tag"]
        if tg in ("case", "rcv"): continue
        ax.plot(*np.c_[a, b], color=COL.get(tg, "#333"), lw=LW.get(tg, 1.8 if e["mat"] == "C" else 2.6), solid_capstyle="round")
    if getattr(g, "rcv_fixed", None):
        ax.add_collection3d(Poly3DCollection(box_faces(*L.RCV_BOX), facecolors="#c8b98f", alpha=0.22, edgecolors="#a08850", linewidths=0.5))
        ax.add_collection3d(Poly3DCollection(box_faces((-41.5, -24.0, -44.5), (37.5, 24.0, -4.5)), facecolors="#e07b39", alpha=0.18, edgecolors="none"))
    for i in g.feet: ax.scatter(*g.X[i], s=30, color="#5a7fa8")
    for p, n in g.wheel_pts: ax.scatter(*p, s=18, color="#c9a227")
    ax.set_xlim(-118, 118); ax.set_ylim(-118, 118); ax.set_zlim(-50, 80); ax.set_box_aspect((236, 236, 130), zoom=1.2)
    ax.view_init(elev, azim); ax.set_axis_off()
if __name__ == "__main__":
    opts = [json.loads(a) for a in sys.argv[2:]]
    fig = plt.figure(figsize=(8 * len(opts), 13), dpi=110, facecolor="#f4f2ee")
    for j, o in enumerate(opts):
        P = sg.params_v2(PZ_=L.PZ, upper=("solid", o.get("upper", 2.0))); P.update(opt=o, spoke=("solid", 2.5), skirt=("solid", o.get("skirt_d", 3.0))); g = L.Guard3(P)
        r = o.get("_r", {})
        for k, (el, az) in enumerate(((26, -55), (0, -90))):
            ax = fig.add_subplot(2, len(opts), 1 + j + k * len(opts), projection="3d", facecolor="#f4f2ee"); draw(ax, g, el, az)
            if k == 0: ax.set_title(o.get("_title", ""), fontsize=13, color="#222", pad=-10)
    fig.suptitle("S2 cage v3 layouts (frame model F, prop plane z 41.2): carbon rods black, crown spokes red, crown orange, PETG nodes sand, TPU tees and feet blue, wheels gold, top case grey",
                 fontsize=12, x=0.01, y=0.985, ha="left", color="#222")
    fig.subplots_adjust(left=0, right=1, bottom=0.02, top=0.90, wspace=0, hspace=-0.15); fig.savefig(sys.argv[1])
