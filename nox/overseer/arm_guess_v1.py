"""Overseer arms v1: a hand-drawn GUESS of what the topology runs will grow (27 Oct, Spark asked for a picture before the
runs end). Not a design and not printed: the real drawn bodies and rod lines from `arm_topo_v1.py`, plus the open
truss expected from 40 / 25 cm3 in envelopes of ~520 / ~190 cm3 (8 % / 13 % fill): Warren webs between each side's
top and bottom chord, cross ties at the top and bottom, struts from the root pads out to the shoulders and the pad.

python3 nox/overseer/arm_guess_v1.py  -> runs/overseer_arms/v1/guess.png
"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
import arm_topo_v1 as A
from topo.part import tm, man

R_WEB = 3.6           # web member radius: the gate's 6.25 mm minimum member plus organic blending (E)


def strut(p, q, r=R_WEB):
    p, q = np.asarray(p, float), np.asarray(q, float)
    return A.rod_m(p, q, r, n=20) + mf.Manifold.sphere(r, 16).translate(q.tolist()) + mf.Manifold.sphere(r, 16).translate(p.tolist())


def front_guess():
    B, F = A.bodies("front")
    L = F["L"]
    ch = A.chords("front", F)                                   # (+y top, +y bottom, -y top, -y bottom)
    web = mf.Manifold()
    n = 6
    for side in (0, 2):
        top0, top1 = ch[side]
        bot0, bot1 = ch[side + 1]
        pts_t = [top0 + (top1 - top0) * k / n for k in range(n + 1)]
        pts_b = [bot0 + (bot1 - bot0) * k / n for k in range(n + 1)]
        for k in range(n):                                       # Warren diagonals, side web
            web += strut(pts_t[k], pts_b[k + 1]) if k % 2 == 0 else strut(pts_b[k], pts_t[k + 1])
        web += strut(pts_t[0], pts_b[0])
    for k in (2, 4):                                             # cross ties top and bottom, X in the bottom plane
        t = k / n
        a_t, b_t = ch[0][0] + (ch[0][1] - ch[0][0]) * t, ch[2][0] + (ch[2][1] - ch[2][0]) * t
        web += strut(a_t, b_t, 3.2)
    for i, j in ((1, 3), (3, 1)):
        p = ch[i][0] + (ch[i][1] - ch[i][0]) * 0.2
        q = ch[j][0] + (ch[j][1] - ch[j][0]) * 0.75
        web += strut(p, q, 3.2)
    root_t, root_b = np.array([4.0, 0.0, 40.0]), np.array([4.0, 0.0, 5.0])
    for side in (0, 2):                                          # root pads out to the shoulders (a stiff root knee)
        web += strut(root_t, ch[side][0], 4.2) + strut(root_b, ch[side + 1][0], 4.2) + strut(root_t, ch[side + 1][0], 3.6)
    web += strut(root_t, root_b, 4.5)
    for side in (0, 2):                                          # into the motor pad wall
        web += strut(ch[side][1], np.array([L - 8.0, 0.0, 40.0]), 3.4) + strut(ch[side + 1][1], np.array([L - 12.0, 0.0, 30.0]), 3.4)
    body = (B["drawn"] + web) ^ B["env"]
    return body - B["cuts"], B["rods"], F


def rear_guess():
    B, F = A.bodies("rear")
    L = F["L"]
    web = mf.Manifold()
    for sy in (1, -1):
        y0, y1 = sy * 16.0, sy * 12.0
        t0, t1 = np.array([2.0, y0, 41.0]), np.array([L - 16.0, y1, 41.0])
        b0, b1 = np.array([2.0, y0, 5.0]), np.array([L - 18.0, y1, 26.0])   # bottom chord rises to the pad wall
        web += strut(t0, t1, 4.4) + strut(b0, b1, 4.4)
        n = 3
        pt = [t0 + (t1 - t0) * k / n for k in range(n + 1)]
        pb = [b0 + (b1 - b0) * k / n for k in range(n + 1)]
        for k in range(n):
            web += strut(pt[k], pb[k + 1]) if k % 2 == 0 else strut(pb[k], pt[k + 1])
    web += strut(np.array([L * 0.45, 14.0, 41.0]), np.array([L * 0.45, -14.0, 41.0]), 3.2)
    web += strut(np.array([L * 0.3, 15.0, 10.0]), np.array([L * 0.7, -13.0, 20.0]), 3.2)
    body = (B["drawn"] + web) ^ B["env"]
    return body - B["cuts"], None, F


def render(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    parts = []
    info = []
    for fn, which in ((front_guess, "front"), (rear_guess, "rear")):
        body, rods, F = fn()
        g = body.volume() / 1000 * 1.27
        rg = 0.0 if rods is None else rods.volume() / 1000 * 1.55
        info.append(f"{which}: ~{g:.0f} g PETG" + (f" + {rg:.0f} g rods" if rods is not None else "") +
                    f" ({body.volume() / 1000:.0f} cm³)")
        for mir in (False, True):
            parts.append((A.arm_to_drone(tm(body), F, mir), "#b9c3cc"))
            if rods is not None:
                parts.append((A.arm_to_drone(tm(rods), F, mir), "#1d2226"))
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(16, 6.4), dpi=130)
    views = (("iso", 28, -125), ("plan (nose to the right)", 90, -90), ("side (from -y)", 0, -90))
    for k, (title, el, az) in enumerate(views):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        for m, col in parts:
            tri = m.vertices[m.faces]
            sh = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(col))
            ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(base[None] * sh[:, None] + 0.05, 0, 1), linewidths=0))
        for p in ("top_plate.stl", "bottom_plate.stl"):
            t = trimesh.load(os.path.join(A.LAYOUT, "parts", p), force="mesh")
            ax.add_collection3d(Poly3DCollection(t.vertices[t.faces], facecolors=(0.6, 0.65, 0.7, 0.15), linewidths=0))
        for mx, my in A.motors_xy():
            for sy in (1,):
                th = np.linspace(0, 2 * np.pi, 40)
                ax.plot(mx + 14 * np.cos(th), my + 14 * np.sin(th), 45 + 0 * th, color="#444", lw=0.8)
        ax.set_xlim(-130, 230); ax.set_ylim(-180, 180); ax.set_zlim(-60, 120)
        ax.set_box_aspect((360, 360, 180))
        ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=11)
    fig.suptitle("Overseer arms v1: GUESS of the topology (drawn by hand on the real interfaces, not the run result)\n"
                 + "   ·   ".join(info) + "   ·   dark = 3.0 carbon rods, grey = PETG", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    return info


if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "overseer_arms", "v1", "guess.png")
    print(render(out), out)
