"""Stand v6 structural check (19 Oct 2026): every stand part as a voxel FE (printopt's PETG material, transversely
isotropic about its print axis, 8-node bricks, pardiso) under its share of the box's load path, misuse class.
Index = printopt.failure (in-layer von Mises / A_xy, across-layer sqrt((s_n/A_z)^2 + (tau/A_s)^2)); > 1 = fails.
Loads (E): arm 2.0 kg at Y 60 on the stub face + 20 N head push at r 150 Y 175 (mast, KB); ring + funnel + Scout
1.7 kg into the cradle lips + 20 N extract pull + the two chain roots' 2.5 N.m from a 13 N rim push (cradle_keel);
the box's full weight (50 N) on one bar tip (bar_half, misuse); 13 N rim push -> chain root couple 2.5 N.m + 6 N thrust
+ 20 N hand on the rim (frames); end frames: 6 N funnel/Scout + 6.5 N rim push per pad pair.
python3 stand_v6_fe.py [part ...]  -> stand_v6_fe.json + stand_v6_fe.png (index maps)"""
import os, sys, json, math, numpy as np, trimesh, scipy.sparse as sp
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../src/nox"))
import printopt as P
import stand_v6_parts as S
import pypardiso

MAT = P.MATERIALS["PETG"]; CLS = "misuse"
A = P.allowables(MAT, CLS)

def voxelize(mesh, h):
    v = mesh.voxelized(h).fill()
    solid = v.matrix.copy(); origin = v.translation - h / 2 if hasattr(v, "translation") else v.transform[:3, 3] - h / 2
    return solid, np.asarray(origin, float)

VER = "v6"
def run(name, h, build_axis, fix_fn, load_fn, cases_names):
    m = trimesh.load(f"box_{name}_{VER}_W.stl")
    solid, org = voxelize(m, h)
    nx, ny, nz = solid.shape
    nn = (nx + 1) * (ny + 1) * (nz + 1)
    ex, ey, ez = [a.ravel() for a in np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij")]
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    corners = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
    enodes = np.stack([nid(ex + a, ey + b, ez + c) for a, b, c in corners], 1)
    act = solid.ravel()
    edof = np.repeat(enodes * 3, 3, axis=1) + np.tile([0, 1, 2], 8)
    C = P.stiffness(MAT, build_axis); KE, B0 = P.hex_KE(h, C); CB = C @ B0
    iK = np.repeat(edof[act], 24, axis=1).ravel(); jK = np.tile(edof[act], (1, 24)).ravel()
    K = sp.coo_matrix((np.tile(KE.ravel(), act.sum()), (iK, jK)), shape=(3 * nn, 3 * nn)).tocsr()
    # node coords (W)
    gi, gj, gk = np.meshgrid(np.arange(nx + 1), np.arange(ny + 1), np.arange(nz + 1), indexing="ij")
    X = np.stack([gi.ravel(), gj.ravel(), gk.ravel()], 1) * h + org
    used = np.zeros(nn, bool); used[np.unique(enodes[act])] = True
    fixn = fix_fn(X) & used
    fixed = np.repeat(np.where(fixn)[0] * 3, 3) + np.tile([0, 1, 2], fixn.sum())
    free = np.setdiff1d(np.where(np.repeat(used, 3))[0], fixed)
    Kf = sp.triu(K[free][:, free].tocsr(), format="csr")
    ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(2)
    res = {}; worst_all = 0; fig_data = []
    for cname in cases_names:
        F = np.zeros(3 * nn); loadn = load_fn(cname, X, used, F)
        if not np.isfinite(F).all() or np.abs(F).sum() == 0: raise RuntimeError(cname)
        U = np.zeros(3 * nn); U[free] = ps.solve(Kf, F[free])
        ue = U[edof[act]]
        s = ue @ CB.T                                             # (nel_act, 6)
        hot = fixn | loadn
        smask = ~hot[enodes[act]].any(1)
        idx = failure(s, A, build_axis)
        idx_m = np.where(smask, idx, 0)
        k = np.argmax(idx_m)
        cent = (np.stack([ex, ey, ez], 1)[act][k] + 0.5) * h + org
        umax = np.abs(U).reshape(-1, 3)[used].max(0)
        res[cname] = dict(worst=round(float(idx_m.max()), 3), at=np.round(cent, 1).tolist(),
                          p99=round(float(np.quantile(idx_m[smask], 0.99)), 3), umax_mm=np.round(umax, 3).tolist())
        fig_data.append((cname, np.stack([ex, ey, ez], 1)[act], idx_m))
        worst_all = max(worst_all, idx_m.max())
        print(f"  {name:12s} {cname:28s} worst {idx_m.max():5.2f} at {np.round(cent,1)}  p99 {res[cname]['p99']:.2f}  umax {np.round(umax,2)}")
    ps.free_memory(everything=True)
    return dict(h=h, voxels=int(act.sum()), cases=res, worst=round(float(worst_all), 3)), fig_data, org, h

def failure(s, A, axis):
    Axy, Az, As = A
    sxx, syy, szz, sxy, syz, szx = s.T
    vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    sn = s[:, axis]; sh = [k for k, (i, j) in P.VOIGT_PAIRS.items() if axis in (i, j)]
    tau = np.sqrt(sum(s[:, k] ** 2 for k in sh))
    across = np.sqrt((np.maximum(sn, 0) / Az) ** 2 + (tau / As) ** 2)
    return np.maximum(vm / Axy, across)

def nodes_near(X, used, fn):
    n = fn(X) & used
    if n.sum() == 0: raise RuntimeError("no load nodes")
    return n
def put(F, n, vec):
    idx = np.where(n)[0]
    if len(idx) == 0: raise RuntimeError('empty load set')
    v = np.asarray(vec, float) / len(idx)
    F[idx * 3] += v[0]; F[idx * 3 + 1] += v[1]; F[idx * 3 + 2] += v[2]

# ---------------------------------------------------------------- load definitions (W)
FLOOR = S.FLOOR
def mast_fix(X):   # sleeve wall over the spigot's engagement (Z -128..-52): bonded to the spigot
    return (X[:, 2] < -52) & (X[:, 2] > -128.5)
def mast_load(c, X, used, F):
    """stub hole rings on the front wall (Y 0). Lean: 19.6 N arm at Y 60 + 20 N head push at Y 175 -> Mx 4.68 N.m:
    the top bolts pull the wall toward +Y, the bottom bolts push it toward -Y (pair spacing 60). Yaw: 20 N at Y 175 ->
    3.5 N.m on the two bolt columns (spacing 20). Side: 20 N tangential at r 150 on the bay -> 20 N in X + 3.0 N.m roll."""
    yf = X[used, 1].max(); ring = lambda x, z: (np.abs(X[:, 1] - yf) < 0.1) & (np.hypot(X[:, 0] - x, X[:, 2] - z) < 7.5) & (np.hypot(X[:, 0] - x, X[:, 2] - z) > 2.6)
    hs = [(10, 30), (-10, 30), (-10, -30), (10, -30)]
    n = np.zeros(len(X), bool)
    for x, z in hs:
        r = ring(x, z) & used; n |= r
        if c == "arm weight + head push (lean)":
            Mx = 19.6 * 0.060 + 20 * 0.175
            put(F, r, [0, (Mx * 1000 / 60 / 2) * (1 if z > 0 else -1), -19.6 / 4])
        elif c == "head push (yaw)":
            put(F, r, [0, (3.5 * 1000 / 20 / 2) * (1 if x > 0 else -1), 0])
        elif c == "side push 20 N at the bay":
            put(F, r, [20 / 4, (3.0 * 1000 / 20 / 2) * (1 if x > 0 else -1) * 0, 0]); put(F, r, [0, 0, (3.0 * 1000 / 20 / 2) * (1 if x > 0 else -1)])
    return n

def kb_fix(X): return X[:, 2] < FLOOR + 0.6
def kb_load(c, X, used, F):
    # the spigot's top 20 mm carries what the sleeve puts on it
    top = (X[:, 2] > -72) & (X[:, 2] < -51.5) & used
    n = top
    if c == "arm lean (moment at the spigot top)":
        My = 19.6 * 0.060 + 20 * 0.175 + 19.6 * 0 # about X: forward lean, N.m, lever to the spigot top (Z -52) adds (0 - -52) x 20 N -> small
        Mx = My + (20 + 19.6 * 0) * 0.052
        # couple: +Y face pushed, -Y face pulled (the mast tips forward): apply as Y-forces on the +Y / -Y halves
        yp = top & (X[:, 1] > -30); ym = top & (X[:, 1] <= -30)
        put(F, yp, [0, 0, -Mx * 1000 / 28 / 2]); put(F, ym, [0, 0, Mx * 1000 / 28 / 2])
        put(F, top, [0, 20, -19.6])
    elif c == "yaw 20 N at the bay":
        xp = top & (X[:, 0] > 0); xm = top & (X[:, 0] <= 0)
        put(F, xp, [0, 3.5 * 1000 / 18 / 2, 0]); put(F, xm, [0, -3.5 * 1000 / 18 / 2, 0])
    return n

def ck_fix(X): return X[:, 2] < FLOOR + 0.6
def ck_load(c, X, used, F):
    lips = (X[:, 2] > -153.5) & (np.abs(X[:, 0]) > 21) & (np.abs(X[:, 0]) < 30.5) & (X[:, 1] > 140) & (X[:, 1] < 197) & used
    n = lips
    if c == "ring + funnel + Scout weight + extract pull":
        put(F, lips, [0, 0, -(17 + 20)])
    elif c == "rim push 13 N (chain roots)":
        # both chains' root couples 2.5 N.m each about Y on the lips, plus 13 N sideways
        xp = lips & (X[:, 0] > 0); xm = lips & (X[:, 0] <= 0)
        put(F, xp, [0, 0, 5.0 * 1000 / 52]); put(F, xm, [0, 0, -5.0 * 1000 / 52]); put(F, lips, [13, 0, 0])
    elif c == "fore-aft push 13 N":
        put(F, lips, [0, 13, 0]); yp = lips & (X[:, 1] > 168); ym = lips & (X[:, 1] <= 168)
        put(F, yp, [0, 0, 13 * 390 / 35 / 2]); put(F, ym, [0, 0, -13 * 390 / 35 / 2])
    return n

def bar_fix(X): return X[:, 0] < 20.5
def bar_load(c, X, used, F):
    tip = (X[:, 0] > 190) & (X[:, 2] < FLOOR + 0.6) & used
    put(F, tip, [0, 0, 50.0])      # the box on one tip: the floor pushes up 50 N (misuse)
    return tip

def frame_fix(X):
    # the -X joint plane: lug faces and the groove block
    t = X[:, 0] * -math.cos(S.H) - X[:, 2] * math.sin(S.H)   # distance toward -X plane normal... use plane: x*cos(H)+z*sin(H) for +X; mirror for -X
    d = -(X[:, 0] * math.cos(S.H)) - X[:, 2] * math.sin(S.H)
    return d > -1.2
def frame_load(c, X, used, F):
    d = X[:, 0] * math.cos(S.H) - X[:, 2] * math.sin(S.H)       # +X joint plane coordinate
    lugs = (d > -8) & (np.hypot(X[:, 0], X[:, 2]) > 164) & used
    n = lugs
    if c == "chain couple 2.5 N.m + thrust 6 N":
        a = lugs & (X[:, 1] < 177); b = lugs & (X[:, 1] >= 177)
        put(F, a, [-39 * math.sin(S.H), 0, -39 * math.cos(S.H)]); put(F, b, [39 * math.sin(S.H), 0, 39 * math.cos(S.H)])
        put(F, lugs, [-6 * math.cos(S.H), 0, 6 * math.sin(S.H)])
    elif c == "hand 20 N on the rim":
        rim = (np.hypot(X[:, 0], X[:, 2]) > 178) & used
        put(F, rim, [0, 0, -20]); n = rim
    elif c == "hang: own weight + neighbour 80 N at the joint":
        put(F, lugs, [0, 0, -80])
    return n

def end_fix(X, side):
    d = side * (X[:, 0] * math.cos(S.H)) - X[:, 2] * math.sin(S.H)
    return d > -1.2
def end_load(c, X, used, F, side):
    pads = (np.hypot(X[:, 0], X[:, 2]) > 183) & used     # pad blocks beyond the ring (r 183+)
    n = pads
    if c == "funnel + Scout 6 N + rim push 6.5 N":
        put(F, pads, [-side * 6.5, 0, -6]);
    elif c == "rim push fore-aft 6.5 N":
        put(F, pads, [0, 6.5, -6]); a = pads & (X[:, 1] < 177); b = pads & (X[:, 1] >= 177)
        put(F, a, [0, 0, 6.5 * 90 / 72]); put(F, b, [0, 0, -6.5 * 90 / 72])
    elif c == "misuse: 50 N on one pad":
        a = pads & (X[:, 1] < 177); put(F, a, [0, 0, -50]); n = a
    return n

PARTS = {
    "mast": (2.0, 2, mast_fix, mast_load, ["arm weight + head push (lean)", "head push (yaw)", "side push 20 N at the bay"]),
    "kb": (2.0, 2, kb_fix, kb_load, ["arm lean (moment at the spigot top)", "yaw 20 N at the bay"]),
    "cradle_keel": (2.0, 1, ck_fix, ck_load, ["ring + funnel + Scout weight + extract pull", "rim push 13 N (chain roots)", "fore-aft push 13 N"]),
    "bar_half": (1.5, 2, bar_fix, bar_load, ["box weight on one tip (misuse)"]),
    "frame_mid": (1.5, 0, frame_fix, frame_load, ["chain couple 2.5 N.m + thrust 6 N", "hand 20 N on the rim", "hang: own weight + neighbour 80 N at the joint"]),
    "frame_endR": (1.5, 0, lambda X: end_fix(X, +1), lambda c, X, u, F: end_load(c, X, u, F, +1), ["funnel + Scout 6 N + rim push 6.5 N", "rim push fore-aft 6.5 N", "misuse: 50 N on one pad"]),
    "frame_endL": (1.5, 0, lambda X: end_fix(X, -1), lambda c, X, u, F: end_load(c, X, u, F, -1), ["funnel + Scout 6 N + rim push 6.5 N", "rim push fore-aft 6.5 N", "misuse: 50 N on one pad"]),
}
# build axis per part in W: mast prints along Z (axis 2), kb along Z, cradle_keel along -Y (axis 1), bar along Z, frames along the joint normal (~X, axis 0)

if __name__ == "__main__":
    names = sys.argv[1:] or list(PARTS)
    out = {}; figs = {}
    for n in names:
        h, ax, fx, ld, cs = PARTS[n]
        print(n)
        r, fd, org, hh = run(n, h, ax, fx, ld, cs)
        out[n] = r; figs[n] = (fd, org, hh)
    json.dump(out, open("stand_v6_fe.json", "w"), indent=1)
    # figure: worst case per part, index map (max over cases), side view projection
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 4, figsize=(18, 9), dpi=120)
    for axp, n in zip(axs.ravel(), names):
        fd, org, hh = figs[n]
        cells = fd[0][1]; idx = np.max(np.stack([f[2] for f in fd]), 0)
        pts = cells * hh + org + hh / 2
        # project onto the plane with the largest extent pair
        ext = pts.max(0) - pts.min(0); drop = int(np.argmin(ext)); keep = [i for i in range(3) if i != drop]
        order = np.argsort(idx)
        sc = axp.scatter(pts[order, keep[0]], pts[order, keep[1]], c=idx[order], s=2, cmap="inferno", vmin=0, vmax=max(1.0, idx.max()))
        axp.set_aspect("equal"); axp.set_title(f"{n}: worst {out[n]['worst']:.2f}", fontsize=10); axp.set_xticks([]); axp.set_yticks([])
        axp.set_xlabel("XYZ"[keep[0]] + " / " + "XYZ"[keep[1]], fontsize=8)
        plt.colorbar(sc, ax=axp, fraction=0.04)
    for axp in axs.ravel()[len(names):]: axp.axis("off")
    fig.suptitle("Stand v6 failure index (printopt PETG, misuse class; max over the load cases; 1.0 = allowable). Load spots are masked.", fontsize=11)
    fig.tight_layout(); fig.savefig("stand_v6_fe.png"); print("stand_v6_fe.png")
