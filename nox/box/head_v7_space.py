"""Heads v7: the free space each head has, from the assembled arm v7 (24 Oct 2026, heads v7 part session).
Every part a head can meet over a swap cycle (arm_kinematics v7: all 9 bay receivers with their seated cartridges, the hub, rails,
motors, detent, the other head and its pack), sampled on its surface and carried into the head's own frame (X tangential, Y along
the axle, u radial from the head's z0 datum; head B mirrored in X like its parts) at every state of the cycle. The union is what
the head body, servo and grabber must stay clear of. Inner bays (two neighbours) are swept as well as the cycle's end bays.
python3 head_v7_space.py [pack:pick:ret ...]  -> runs/head/v7/space/<pack>_<pick>_<ret>.npz + space.png (X-u maps per Y band)
Numbers C (geometry from the parts in pose)."""
import os, sys, math, json
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(ROOT, "runs", "head", "v7", "space"); os.makedirs(OUT, exist_ok=True)
BOX = dict(x=(-70.0, 70.0), y=(60.0, 230.0), u=(-70.0, 80.0))      # the head's region of interest (head frame)
CYCLES = ["scout:-210:30", "scout:-90:-150", "overseer:-180:0", "overseer:-60:-120"]


def surface_points(mesh, per_mm2=0.6, cap=60000):
    n = int(min(cap, max(2000, mesh.area * per_mm2)))
    p, _ = trimesh.sample.sample_surface(mesh, n, seed=1)
    return np.vstack([p, mesh.vertices])


def run(cycle):
    import arm_kinematics as K, arm_layout as L
    import arm_v7_layout as S7
    S7.use()
    pk, pi, re_ = cycle.split(":")
    K.KEYS = K.keys_v7(float(pi), float(re_)); K.T_END = 22.0
    P = K.build_parts_v7(pk, float(pi), float(re_))
    pts = {nm: surface_points(p.mesh) for nm, p in P.items()}
    cart = surface_points(K.tm(K.CART_LOCAL(0.0)))
    shape = tuple(int(BOX[k][1] - BOX[k][0]) for k in ("x", "y", "u"))
    out = {"A": np.zeros(shape, bool), "B": np.zeros(shape, bool)}
    def mark(h, loc):
        i = np.floor(loc - np.array([BOX["x"][0], BOX["y"][0], BOX["u"][0]])).astype(int)
        ok = np.all((i >= 0) & (i < np.array(shape)), axis=1); i = i[ok]
        out[h][i[:, 0], i[:, 1], i[:, 2]] = True
    for t in K.sample_times(0.25, 3.0):
        T, carts, s = K.transforms(t)
        for h in ("A", "B"):
            Tarm = K.rotY(90.0 - s["phi"])
            Th = Tarm @ K.trans(0, 0, s["zA"]) if h == "A" else Tarm @ K.RB @ K.trans(0, 0, s["zB"])
            inv = np.linalg.inv(Th)
            mir = np.diag([-1.0, 1, 1]) if h == "B" else np.eye(3)
            for nm, p in P.items():
                if p.group == f"head{h}": continue
                w = pts[nm] @ T[nm][:3, :3].T + T[nm][:3, 3]
                loc = (w @ inv[:3, :3].T + inv[:3, 3]) @ mir.T
                mark(h, loc)
            for cn, (Tc, sq, own) in carts.items():
                if own.startswith(f"head{h}"): continue                         # its own carried / gripped pack
                w = cart @ Tc[:3, :3].T + Tc[:3, 3]
                loc = (w @ inv[:3, :3].T + inv[:3, 3]) @ mir.T
                mark(h, loc)
    np.savez_compressed(os.path.join(OUT, cycle.replace(":", "_") + ".npz"), **out)
    return out


def plot(results, bands=((73.75, 110.0), (110.0, 150.0), (150.0, 180.0), (180.0, 217.0)), path=None):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, len(bands), figsize=(4.2 * len(bands), 8.6), sharex=True, sharey=True)
    for i, h in enumerate(("A", "B")):
        g = np.any(np.stack([r[h] for r in results.values()]), axis=0)
        for j, (y0, y1) in enumerate(bands):
            ax = axs[i, j]; j0, j1 = int(y0 - BOX["y"][0]), int(y1 - BOX["y"][0])
            occ = g[:, j0:j1, :].any(axis=1).T
            ax.imshow(occ, origin="lower", extent=(*BOX["x"], *BOX["u"]), cmap="Purples", vmin=0, vmax=1.6, interpolation="nearest")
            ax.add_patch(plt.Rectangle((-28.6, 15.4), 57.2, 60, fill=False, ec="#2f6db3", lw=1, ls="--"))   # carried cartridge footprint
            ax.axhline(15.4, c="#2f6db3", lw=0.6); ax.axhline(0.0, c="k", lw=0.4)
            ax.set_title(f"head {h}: Y {y0:g}..{y1:g}", fontsize=10); ax.set_aspect("equal"); ax.grid(lw=0.3)
            ax.set_xlim(-70, 70); ax.set_ylim(-70, 80)
            if i == 1: ax.set_xlabel("X (tangential)")
            if j == 0: ax.set_ylabel("u (radial, head frame)")
    fig.suptitle("Heads v7: obstacles in the head frame over the swap cycles (" + ", ".join(results) + "); blue = the carried cartridge from u 15.4")
    fig.tight_layout(); fig.savefig(path or os.path.join(OUT, "space.png"), dpi=110)


if __name__ == "__main__":
    cyc = [a for a in sys.argv[1:] if not a.startswith("--")] or CYCLES
    R = {}
    for c in cyc:
        f = os.path.join(OUT, c.replace(":", "_") + ".npz")
        R[c] = dict(np.load(f)) if os.path.exists(f) and "--fresh" not in sys.argv else run(c)
        print(c, {h: int(v.sum()) for h, v in R[c].items()})
    plot(R)
    print("->", os.path.join(OUT, "space.png"))
