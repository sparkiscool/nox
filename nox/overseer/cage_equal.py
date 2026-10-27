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
SRC = json.load(open(os.environ["CE_START"])) if os.environ.get("CE_START") else FZ
N0 = np.array(SRC["nodes"]); E = [tuple(r) for r in SRC["rods"]]; ring = list(SRC["open"][0]["ring"]); nR = len(ring)
assert ring == list(range(nR))
KEEP = int(os.environ.get("CE_KEEP", 0))
REDUCE = os.environ.get("CE_MODE") == "reduce"                         # Spark 27 Oct: "get rid of the number of same length rule just do the rods number reduction"                               # free nodes kept (0 = all; Spark 27 Oct: "divide the number of rods by half")


def sym_tris(N):
    """the hull's faces (the opening ring's own facet left out), a mirror-symmetric triangulation: flat facets merged, a facet
    and its mirror share one triangulation, a centre-plane facet that is not its own mirror is fanned from a node at its centre"""
    from scipy.spatial import cKDTree
    _, P_ = cKDTree(N).query(N * [1, -1, 1]); h = ConvexHull(N); S = h.simplices; par = list(range(len(S)))
    def root(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    for fi in range(len(S)):
        for fj in h.neighbors[fi]:
            opp = [v for v in S[fj] if v not in S[fi]]
            if opp and abs(N[opp[0]] @ h.equations[fi, :3] + h.equations[fi, 3]) < 0.3: par[root(fi)] = root(fj)
    fac = {}
    for fi in range(len(S)): fac.setdefault(root(fi), []).append(fi)
    fac = [fs_ for fs_ in fac.values() if not (np.unique(S[fs_]) < nR).all()]
    key = {frozenset(np.unique(S[fs_]).tolist()): k for k, fs_ in enumerate(fac)}
    T, extra = [], []
    for k, fs_ in enumerate(fac):
        vs = np.unique(S[fs_]); km = key.get(frozenset(P_[vs].tolist())); T_ = [tuple(sorted(int(v) for v in S[fi])) for fi in fs_]
        if km is None: T += T_; continue
        if km != k:
            if N[vs, 1].mean() > 0: T += T_ + [tuple(sorted(int(P_[v]) for v in t)) for t in T_]
            continue
        if {tuple(sorted(int(P_[v]) for v in t)) for t in T_} == set(T_): T += T_; continue
        cnt = {}
        for t in T_:
            for i in range(3): e2 = tuple(sorted((t[i], t[(i + 1) % 3]))); cnt[e2] = cnt.get(e2, 0) + 1
        c = N[vs].mean(0); c[1] = 0.0; ci = len(N) + len(extra); extra.append(c)
        T += [tuple(sorted((ci, a_, b_))) for (a_, b_), n_ in cnt.items() if n_ == 1]
    if extra: N = np.vstack([N, extra])
    return N, np.array(sorted(set(T)))


MERGE = float(os.environ.get("CE_MERGE_MM", 0))                         # Spark 27 Oct: nodes so close they can just become one (tubes under this)
if MERGE:
    N0, tris = sym_tris(N0); E = sorted({tuple(sorted((int(t[i]), int(t[(i + 1) % 3])))) for t in tris for i in range(3)})
    for _ in range(10):
        Ls = [(np.linalg.norm(N0[a] - N0[b]), a, b) for a, b in E]
        short = [(a, b) for L_, a, b in Ls if L_ < MERGE]
        if not short: break
        par_ = list(range(len(N0)))
        def rt(i):
            while par_[i] != i: par_[i] = par_[par_[i]]; i = par_[i]
            return i
        for a, b in short: par_[rt(a)] = rt(b)
        cl = {}
        for i in range(len(N0)): cl.setdefault(rt(i), []).append(i)
        newN, isring = [], []
        for mem in cl.values():
            rg = [i for i in mem if i < nR]
            c = N0[mem].mean(0)
            if rg: c = N0[rg].mean(0); c[2] = N0[rg[0], 2]               # a cluster with base-loop nodes stays on the loop plane
            if abs(N0[mem, 1].mean()) < 1.0 and len(mem) > 1 and np.allclose(sorted(N0[mem, 1]), sorted(-N0[mem, 1]), atol=1.0): c[1] = 0.0
            newN.append(c); isring.append(bool(rg))
        newN = np.array(newN); isring = np.array(isring)
        ctr = newN[isring].mean(0)
        order = sorted(np.where(isring)[0], key=lambda i: math.atan2(newN[i, 1] - ctr[1], newN[i, 0] - ctr[0]))
        N0 = np.vstack([newN[order], newN[~isring]]); nR = len(order)
        print(f"merged tubes under {MERGE:g}: {len(short)} -> {len(N0)} nodes, base loop {nR} nodes", flush=True)
        N0, tris = sym_tris(N0)
        E = sorted({tuple(sorted((int(t[i]), int(t[(i + 1) % 3])))) for t in tris for i in range(3)})
    SRC = dict(SRC, open=[dict(SRC["open"][0], ring=list(range(nR)))])
    KEEP = 0
if MERGE:
    pass
elif KEEP:                                                               # thin the free nodes: mirror pairs and centre nodes, farthest first
    fr = [i for i in range(nR, len(N0)) if N0[i, 1] >= -0.5]
    ctr = N0[nR:].mean(0); scale = np.ptp(N0, 0)
    pick = [max(fr, key=lambda i: np.linalg.norm((N0[i] - ctr) / scale))]
    cnt_ = lambda sel: sum(1 if abs(N0[i, 1]) <= 0.5 else 2 for i in sel)
    while cnt_(pick) < KEEP:
        d_ = {i: min(np.linalg.norm((N0[i] - N0[j]) / scale) for j in pick) for i in fr if i not in pick}
        pick.append(max(d_, key=d_.get))
    keep = sorted(set(pick) | {int(np.argmin(np.linalg.norm(N0 - N0[i] * [1, -1, 1], axis=1))) for i in pick})
    N0 = np.vstack([N0[:nR], N0[keep]])
    N0, tris = sym_tris(N0)
    MIN_ANG = float(os.environ.get("CE_MIN_ANGLE", 20.0))             # no two tubes at a node closer than this (their sockets clash)
    for _ in range(10 if os.environ.get("CE_DROP") else 0):           # dropping nodes for slivers (off: it cascades; the angle hinge below instead)
        E_ = sorted({tuple(sorted((int(t[i]), int(t[(i + 1) % 3])))) for t in tris for i in range(3)})
        worst = None
        for v in range(len(N0)):
            nb = [b_ if a_ == v else a_ for a_, b_ in E_ if v in (a_, b_)]
            for p_ in range(len(nb)):
                for q_ in range(p_ + 1, len(nb)):
                    u1, u2 = N0[nb[p_]] - N0[v], N0[nb[q_]] - N0[v]
                    th = math.degrees(math.acos(np.clip(u1 @ u2 / np.linalg.norm(u1) / np.linalg.norm(u2), -1, 1)))
                    if th < MIN_ANG and (worst is None or th < worst[0]): worst = (th, v, nb[p_], nb[q_])
        if worst is None: break
        th, v, p_, q_ = worst                                          # drop the free node of the sliver's short side (and its mirror)
        cand = [w for w in (p_, q_, v) if w >= nR]
        w = min(cand, key=lambda w: min(np.linalg.norm(N0[w] - N0[o]) for o in (v, p_, q_) if o != w)) if cand else None
        if w is None: break
        mw = int(np.argmin(np.linalg.norm(N0 - N0[w] * [1, -1, 1], axis=1)))
        keep_ = [k_ for k_ in range(len(N0)) if k_ not in (w, mw)]
        print(f"sliver {th:.1f} deg at node {v}: node {w} dropped" + (f" with its mirror {mw}" if mw != w else ""), flush=True)
        N0 = N0[keep_]; N0, tris = sym_tris(N0)
    E = sorted({tuple(sorted((int(t[i]), int(t[(i + 1) % 3])))) for t in tris for i in range(3)})
else:
    adj = {i: set() for i in range(len(N0))}
    for a, b in E: adj[a].add(b); adj[b].add(a)
    tris = sorted({tuple(sorted((a, b, c))) for a, b in E for c in adj[a] & adj[b]})
    if "tris" in SRC: tris = np.array(SRC["tris"])
    elif SRC is FZ:
        hN = ConvexHull(N0); fs = {tuple(sorted(f)) for f in hN.simplices}
        tris = np.array([t for t in tris if t in fs])                 # the shell's faces (3-cliques on its hull)
    else: tris = np.array([t for t in tris if not all(v < nR for v in t)])   # every 3-clique but the ring's own
print(f"start: {len(N0)} nodes, {len(E)} tubes, {len(tris)} faces", flush=True)
MIN_ANG = float(os.environ.get("CE_MIN_ANGLE", 20.0))
PAIRS = []
for v in range(len(N0)):
    nb = sorted({b_ if a_ == v else a_ for a_, b_ in E if v in (a_, b_)})
    PAIRS += [(v, p_, q_) for k_, p_ in enumerate(nb) for q_ in nb[k_ + 1:]]
PAIRS_A = np.array(PAIRS) if PAIRS else None
N_START = N0.copy()
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


MMAX = int(os.environ.get("CE_MMAX", 3))                               # a span may be cut as up to MMAX equal pieces joined in line
SPLIT_PEN = 0.3                                                         # mm: a split only where it lands a piece nearer a cut length


WL = 1.0                                                                # length-residual weights (IRLS: most pieces exact, a few left out)


def merged(PL):
    """the cut lengths after merging pieces within TOL of one centre (greedy along the sorted piece lengths)"""
    Ls = np.sort(PL); out_ = []; i = 0
    while i < len(Ls):
        j = i
        while j + 1 < len(Ls) and Ls[j + 1] - Ls[i] <= 2 * TOL: j += 1
        out_.append(0.5 * (Ls[i] + Ls[j])); i = j + 1
    return np.array(out_)


def assign(L, T):
    """each span: the (cut length, number of pieces) whose piece is nearest a cut length (a split costs SPLIT_PEN)"""
    m_ = np.arange(1, MMAX + 1)
    cost = np.abs(L[:, None, None] / m_[None, :, None] - T[None, None, :]) + SPLIT_PEN * (m_[None, :, None] - 1)
    k = cost.reshape(len(L), -1).argmin(1)
    return k % len(T), m_[k // len(T)]


def resid(x, n, cls, mult=None):
    N, T = unpack(x, n)
    L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1)
    if REDUCE: r = [0.05 * (N - N_START).ravel()]                      # reduce mode: no length goal, stay near the thinned start
    else: r = [WL * (L / (mult if mult is not None else 1) - T[cls])]
    p0 = N[tris[:, 0]]; nv = np.cross(N[tris[:, 1]] - p0, N[tris[:, 2]] - p0); nv /= np.linalg.norm(nv, axis=1)[:, None]
    nv *= np.sign(((p0 - kc) * nv).sum(1))[:, None]                    # outward
    r.append(W * np.maximum(0, ((Kp[None] - p0[:, None]) * nv[:, None]).sum(2)).ravel())    # K inside every face
    r.append(W * np.maximum(0, (N[nR:] - g["B"]) @ A + l2_req))          # L2 plane
    for n_, l_, lim in cams: r.append(W * np.maximum(0, lim - (N - l_) @ n_))   # camera views
    r.append(W * np.maximum(0, z_o + 1 - N[nR:, 2]))
    if PAIRS:                                                           # every two tubes at a node at least MIN_ANG apart
        u1 = N[PAIRS_A[:, 1]] - N[PAIRS_A[:, 0]]; u2 = N[PAIRS_A[:, 2]] - N[PAIRS_A[:, 0]]
        cosv = (u1 * u2).sum(1) / np.linalg.norm(u1, axis=1) / np.linalg.norm(u2, axis=1)
        r.append(W * 50.0 * np.maximum(0, cosv - math.cos(math.radians(MIN_ANG))))
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
        cls, mult = assign(L, T)
        res = least_squares(resid, x, args=(n, cls, mult), method="trf", tr_solver="lsmr", max_nfev=NFEV, x_scale=1.0)
        x = res.x
        N, T = unpack(x, n); L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1)
        cls2, mult2 = assign(L, T)
        if (cls2 == cls).all() and (mult2 == mult).all(): break
    global WL
    for it in range(8):                                                 # IRLS toward L1: pieces near a cut length go exact
        rl = L / mult2 - T[cls2]; WL = 1.0 / np.sqrt(np.sqrt(rl ** 2 + 0.25))
        res = least_squares(resid, x, args=(n, cls2, mult2), method="trf", tr_solver="lsmr", max_nfev=NFEV, x_scale=1.0); x = res.x
        N, T = unpack(x, n); L = np.linalg.norm(N[[a for a, b in E]] - N[[b for a, b in E]], axis=1); cls2, mult2 = assign(L, T)
    WL = 1.0
    rr = resid(x, n, cls2, mult2); m = len(E)
    return dict(n=n, N=N, T=T, cls=cls2, mult=mult2, L=L, ncut=len(merged(L / mult2)), err=float(np.abs(L / mult2 - T[cls2]).max()), viol=float(rr[m:].max() / W if len(rr) > m else 0))


if REDUCE:
    NS = [1]; MMAX = 1
out = []
for n in NS:
    f = fit(n); out.append(f)
    print(f"n {n:2d}: worst tube {f['err']:.2f} off its cut length, constraints {f['viol']:.2f} mm; lengths "
          + ", ".join(f"{t:.0f} x{int(f['mult'][f['cls'] == i].sum())}" for i, t in sorted(enumerate(f['T']), key=lambda q: q[1]))
          + f"; {int(f['mult'].sum())} pieces in {len(E)} spans; {f['ncut']} lengths to cut within ±{TOL:g}", flush=True)
best = min(out, key=lambda f: (f["viol"] > 0.05, f["ncut"], f["err"]))   # the fewest lengths to cut (Spark 27 Oct)
# merge cut lengths whose tubes all stay within TOL of one centre (fewest lengths to cut, greedy along the sorted lengths)
PL = best["L"] / best["mult"]; cuts = list(merged(PL))
best["T"] = np.array(cuts); best["cls"] = np.abs(PL[:, None] - best["T"][None]).argmin(1); best["n"] = len(cuts)
best["err"] = float(np.abs(PL - best["T"][best["cls"]]).max())
used = sorted({v for e in E for v in e})                               # nodes no tube reaches (left inside by a merge) are dropped
if len(used) < len(best["N"]):
    ri = {o: k for k, o in enumerate(used)}
    best["N"] = best["N"][used]; E = [(ri[a], ri[b]) for a, b in E]; tris = np.array([[ri[v] for v in t] for t in tris if all(v in ri for v in t)])
    SRC = dict(SRC, open=[dict(SRC["open"][0], ring=[ri[v] for v in SRC["open"][0]["ring"] if v in ri])])
spec = dict(note=f"Overseer cage v4 (27 Oct, Spark: as few cut lengths as possible): cage v3's nodes and tubes moved by nox/overseer/cage_equal.py; "
                 f"{best['n']} cut lengths, every piece within {best['err']:.2f} of its length; a span of more than one piece is joined in line by a node.",
            dm=dm, xB=FZ["xB"], nodes=np.round(best["N"], 3).tolist(), rods=[list(map(int, e)) for e in E], open=SRC["open"],
            tris=[list(map(int, t)) for t in tris], node_r_mm=NODE_R, rod_r_mm=SHELL_R, cut_lengths=[round(float(t), 1) for t in best["T"]],
            counts=[int(best["mult"][best["cls"] == i].sum()) for i in range(best["n"])], pieces=[int(m_) for m_ in best["mult"]], fits=[dict(n=f["n"], err=round(f["err"], 2), viol=round(f["viol"], 2)) for f in out])
OUTF = os.environ.get("CE_OUT", os.path.join(HERE, "cage_v4_shell.json"))
json.dump(spec, open(OUTF, "w"))
print("wrote", OUTF, ":", best["n"], "lengths", spec["cut_lengths"], spec["counts"], "err", round(best["err"], 2), "viol", round(best["viol"], 2))
