"""Overseer cage v4 (27 Oct): the cage v3 shell (same nodes and tubes, the same connectivity) with its nodes moved so the tubes
come in as few cut lengths as possible. Spark 27 Oct: "make the entire cage use as many of the same length rods as possible so
during building I don't have to cut different lengths for everything"; "you can actually change the base as well".

Held while the nodes move (least squares, hinge penalties): every part with its margin (the layout's target K: prop disks 12
off the tubes, the rest 3) inside every shell face; the L2 base plane (node centres LIM_L2 behind it) and each camera's view
(node centres LIM beyond the view face that clears the cage, so a tube between two nodes is out of view too); the opening ring
flat at the opening plane, its chords clear of the swap zone (receiver + 30 a side); free nodes above the ring; perfect
left-right symmetry (nodes in mirror pairs). The objective: every tube within TOL of one of n cut lengths (the lengths move
too); n scanned, the smallest n whose fit is within TOL wins (DEFAULT).

python3 nox/overseer/cage_equal.py [dm]   -> nox/overseer/cage_v4_shell.json + runs/overseer_layout/cage_v4/summary
"""
import sys, os, json, math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
NFEV = int(os.environ.get("CE_NFEV", 300))
TOL = 2.0                                         # a tube is "that length" within ±2 (the node sockets take the play, DEFAULT 27 Oct)
NS = [int(x) for x in os.environ.get("CE_NS", "3,4,5,6,7,8,10,12").split(",")]
src = open(os.path.join(HERE, "layout_v5.py")).read(); src = src[:src.index("os.makedirs(OUT, exist_ok=True)")]
argv0 = sys.argv; sys.argv = ["layout_v5.py", os.path.join(ROOT, "runs", "overseer_layout", "cage_v4", "_layout")]
G = {"__file__": os.path.join(HERE, "layout_v5.py"), "__name__": "cage_equal"}
exec(compile(src, "layout_v5", "exec"), G); sys.argv = argv0
FZ = G["FZ"]; A = G["A"]; LIM, LIM_L2, NODE_R, SHELL_R = G["LIM"], G["LIM_L2"], G["NODE_R"], G["SHELL_R"]
dm = float(sys.argv[1]) if len(sys.argv) > 1 else json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "result.json")))["dm"]
g = G["drone"](dm); K = g["K"]
N0 = np.array(FZ["nodes"]); E = [tuple(r) for r in FZ["rods"]]; ring = list(FZ["open"][0]["ring"]); nR = len(ring)
assert ring == list(range(nR))
adj = {i: set() for i in range(len(N0))}
for a, b in E: adj[a].add(b); adj[b].add(a)
tris = sorted({tuple(sorted((a, b, c))) for a, b in E for c in adj[a] & adj[b]})
hN = ConvexHull(N0); fs = {tuple(sorted(f)) for f in hN.simplices}
tris = np.array([t for t in tris if t in fs])                     # the shell's faces (3-cliques on its hull)
Kp = K[ConvexHull(K).vertices]; kc = Kp.mean(0)
if len(Kp) > 400:                                                       # thin the target's hull vertices (farthest-point, 400) for memory
    sel = [0]; dmin = np.linalg.norm(Kp - Kp[0], axis=1)
    for _ in range(399):
        k_ = int(dmin.argmax()); sel.append(k_); dmin = np.minimum(dmin, np.linalg.norm(Kp - Kp[k_], axis=1))
    Kp = Kp[sel]
z_o = g["z_o"]; DC = G["DC"]; zx, zy = G["ZONE"]; cx = -DC
corners = np.array([[cx + sx * (zx + SHELL_R + 2), sy * (zy + SHELL_R + 2)] for sx in (1, -1) for sy in (1, -1)])
# camera limits: per camera the view face the cage clears best (node centres beyond it by LIM, or by what v3 has if less)
cams = []
for c in g["cams"]:
    faces = G["cam_faces"](c["f"], c["u"], c["r"], c["ha"], c["hv"])
    sl = [float(((N0 - c["lens"]) @ n).min()) for n in faces]; k = int(np.argmax(sl))
    cams.append((faces[k], c["lens"], min(LIM, sl[k])))
l2_req = min(LIM_L2, float(-((N0 - g["B"]) @ A).max()))
# mirror pairs: representatives y >= 0
from scipy.spatial import cKDTree
_, P = cKDTree(N0).query(N0 * [1, -1, 1])
rep = [i for i in range(len(N0)) if N0[i, 1] > 0.5 or (abs(N0[i, 1]) <= 0.5)]
onp = [i for i in rep if abs(N0[i, 1]) <= 0.5]


def unpack(x, n):
    N = np.zeros_like(N0); k = 0
    for i in rep:
        if i < nR:                                                     # ring: x, y move, z stays the opening plane
            if i in onp: N[i] = (x[k], 0.0, z_o); k += 1
            else: N[i] = (x[k], x[k + 1], z_o); k += 2
        elif i in onp: N[i] = (x[k], 0.0, x[k + 1]); k += 2
        else: N[i] = x[k:k + 3]; k += 3
    for i in range(len(N0)):
        if i not in rep: N[i] = N[P[i]] * [1, -1, 1]
    return N, x[k:k + n]


def pack(N, T):
    x = []
    for i in rep:
        if i < nR: x += [N[i, 0]] if i in onp else [N[i, 0], N[i, 1]]
        elif i in onp: x += [N[i, 0], N[i, 2]]
        else: x += list(N[i])
    return np.array(x + list(T))


W = 30.0                                                               # constraint weight (mm of violation vs mm of length error)


def resid(x, n, cls):
    N, T = unpack(x, n)
    L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1)
    r = [L - T[cls]]
    p0 = N[tris[:, 0]]; nv = np.cross(N[tris[:, 1]] - p0, N[tris[:, 2]] - p0); nv /= np.linalg.norm(nv, axis=1)[:, None]
    nv *= np.sign(((p0 - kc) * nv).sum(1))[:, None]                    # outward
    r.append(W * np.maximum(0, ((Kp[None] - p0[:, None]) * nv[:, None]).sum(2)).ravel())    # K inside every face
    r.append(W * np.maximum(0, (N[nR:] - g["B"]) @ A + l2_req))          # L2 plane
    for n_, l_, lim in cams: r.append(W * np.maximum(0, lim - (N - l_) @ n_))   # camera views
    r.append(W * np.maximum(0, z_o + 1 - N[nR:, 2]))
    R = N[:nR, :2]                                                      # ring chords clear of the swap zone (corners inside)
    ctr = R.mean(0)
    for i in range(nR):
        a_, b_ = R[i], R[(i + 1) % nR]; t_ = b_ - a_; nn = np.array([t_[1], -t_[0]]) / np.linalg.norm(t_)
        if (ctr - a_) @ nn > 0: nn = -nn
        r.append(W * np.maximum(0, (corners - a_) @ nn + 0.0))
    return np.concatenate(r)


def fit(n):
    L0 = np.linalg.norm(N0[[a for a, b in E]] - N0[[b for a, b in E]], axis=1)
    T = np.quantile(L0, (np.arange(n) + 0.5) / n); x = pack(N0, T); best = None
    for it in range(12):
        N, T = unpack(x, n); L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1)
        cls = np.abs(L[:, None] - T[None]).argmin(1)
        res = least_squares(resid, x, args=(n, cls), method="trf", tr_solver="lsmr", max_nfev=NFEV, x_scale=1.0)
        x = res.x
        N, T = unpack(x, n); L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1)
        cls2 = np.abs(L[:, None] - T[None]).argmin(1)
        if (cls2 == cls).all(): break
    rr = resid(x, n, cls2); m = len(E)
    return dict(n=n, N=N, T=T, cls=cls2, L=L, err=float(np.abs(L - T[cls2]).max()), viol=float(rr[m:].max() / W if len(rr) > m else 0))


out = []
for n in NS:
    f = fit(n); out.append(f)
    print(f"n {n:2d}: worst tube {f['err']:.2f} off its cut length, constraints {f['viol']:.2f} mm; lengths "
          + ", ".join(f"{t:.0f} x{int((f['cls'] == i).sum())}" for i, t in sorted(enumerate(f['T']), key=lambda q: q[1])), flush=True)
    if f["err"] <= TOL and f["viol"] <= 0.05: break
ok = [f for f in out if f["err"] <= TOL and f["viol"] <= 0.05]
best = ok[0] if ok else min(out, key=lambda f: (f["viol"] > 0.05, f["err"]))
# merge cut lengths whose tubes all stay within TOL of one centre (fewest lengths to cut, greedy along the sorted lengths)
Ls = np.sort(best["L"]); groups = []; i = 0
while i < len(Ls):
    j = i
    while j + 1 < len(Ls) and Ls[j + 1] - Ls[i] <= 2 * TOL: j += 1
    groups.append(Ls[i:j + 1]); i = j + 1
cuts = [0.5 * (q.min() + q.max()) for q in groups]
best["T"] = np.array(cuts); best["cls"] = np.abs(best["L"][:, None] - best["T"][None]).argmin(1); best["n"] = len(cuts)
best["err"] = float(np.abs(best["L"] - best["T"][best["cls"]]).max())
spec = dict(note=f"Overseer cage v4 (27 Oct, Spark: as few cut lengths as possible): cage v3's nodes and tubes moved by nox/overseer/cage_equal.py; "
                 f"{best['n']} cut lengths, every tube within {best['err']:.2f} of its length.",
            dm=dm, xB=FZ["xB"], nodes=np.round(best["N"], 3).tolist(), rods=[list(map(int, e)) for e in E], open=FZ["open"],
            node_r_mm=NODE_R, rod_r_mm=SHELL_R, cut_lengths=[round(float(t), 1) for t in best["T"]],
            counts=[int((best["cls"] == i).sum()) for i in range(best["n"])], fits=[dict(n=f["n"], err=round(f["err"], 2), viol=round(f["viol"], 2)) for f in out])
json.dump(spec, open(os.path.join(HERE, "cage_v4_shell.json"), "w"))
print("wrote nox/overseer/cage_v4_shell.json:", best["n"], "lengths", spec["cut_lengths"], spec["counts"], "err", round(best["err"], 2), "viol", round(best["viol"], 2))
