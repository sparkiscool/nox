"""Scout v3 cage nodes (printed PETG, drawn; Spark 27 Oct: every rod 3.0 solid carbon in 3.2 epoxy sockets, perfect mirror symmetry).

One design per mirror pair (the +y node; the -y node is its mirror print) and per centre-plane node: 11 designs, 18 nodes.
Members: runs/scout_layout/v12/cage_shell.json (47 shell rods) + the 8 saddle ties of result.json "links" (their cage ends sit on the
waist pair 5/6 and on nodes 12/13, 14/15). Drone frame (x nose, y left, z up), mm, N.

Each node is drawn, nothing generated (the rule's drawn set: rod sockets with their boss walls, the hub they join in, the bottom nodes'
TPU cap seat, a flat bed face): a socket per member, bore 3.2 x DEPTH 10, a boss wall >= 3.0 round it, a solid floor >= RMIN 3.0 behind
it (each boss is solid from its line's origin near the node centre to the socket mouth), a hub sphere Ø12 (the layout's node) where the
bosses meet. The sockets are set back from the centre only as far as they must so that every two bores keep 3.0 of PETG between them and
every rod leaves its mouth 0.5 clear of the other bosses (per-socket set-backs solved per node, COBYLA, `layout()`).
Option B (DEFAULT, SCN_ECC=3.5): a rod in a tight pair (< 25 deg) may run up to 3.5 off the node centre at that one end (its line
tilts <= 3.5 / L, its other end stays on its node centre); option A (SCN_ECC=0): every socket axis through the node centre.
Bottom ring nodes (0..3, the shared Overseer loop): hub Ø16 down to a flat seat 10.15 under the rod centres for a TPU foot cap Ø16 x 3
(box catch F, the Overseer's caps; plug Ø8 x 2 into a Ø8.3 x 2.2 recess, bonded). Node 16: nothing past its layout sphere toward +x
(x <= +6; the camera mount v1 sits 5.45 off that sphere).

Strength (as-built voxel FE, nox/topo Part + Solver, the engine's PETG transversely isotropic allowables, crash class): each socket
in turn loaded through a carbon insert in its bore (bonded, EA exact), every other rod's stub held; cases: push min(frame axial,
Pcr k 0.7), pull frame axial (runs/scout_frame/v3/node_loads.json, F), and the rod's full bending moment at the mouth (600 MPa x Z of a
3.0 rod = 1590 N mm, as a couple on the stub) in 4 directions plus the frame side force. So the node outlasts what its rods can deliver:
the rod buckles or breaks, or its bond lets go, first (breaking order). A socket whose own cases read over 1.0 gets 0.5 more wall,
the hub 1.0 more radius, until every case reads <= 1.0.

python3 nox/scout/v3/cage_node.py list                     # members, set-backs, envelopes
python3 nox/scout/v3/cage_node.py fe [--nodes ..]          # one FE pass at the drawn minimum
python3 nox/scout/v3/cage_node.py build [out] [--fe] [--h 0.5] [--nodes 0,1,..]   # meshes (+ FE sizing) -> runs/scout_nodes/v1/nodes/<i>.json
python3 nox/scout/v3/cage_node.py merge [out]              # result.json, placed.json, the TPU foot cap
"""
import os, sys, json, math, argparse, time
import numpy as np
import manifold3d as mf
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
LAY = os.path.join(ROOT, "runs", "scout_layout", "v12")
CG = json.load(open(os.path.join(LAY, "cage_shell.json"))); RES = json.load(open(os.path.join(LAY, "result.json")))
N = np.array(CG["nodes"], float)
NL = json.load(open(os.path.join(ROOT, "runs", "scout_frame", "v3", "node_loads.json")))
RING = list(CG["open"][0]["ring"])

BORE, DEPTH, WALL0, RMIN = 3.2, 10.0, 3.0, 3.0      # socket bore (Spark 27 Oct coupons), depth (the Overseer's 3 mm socket), wall (hard rule), floor
SEP_BORE = BORE + 3.0                                 # two bores keep 3.0 of PETG between them (boss wall rule)
ROD_R, ROD_CLEAR = 1.5, 0.5                           # a rod leaving its mouth clears every other boss by 0.5 (C, DEFAULT)
HUB_R = 6.0                                           # hub sphere Ø12 (the layout's printed node, pad rule)
RING_HUB_R = 8.0                                      # bottom nodes: hub and seat column Ø16 (= the foot cap)
PAD_DZ = 10.15                                        # cap seat under the ring rod centres (the Overseer's: node - 10.15, contact - 13.15)
CAP_D, CAP_T, PLUG_D, PLUG_H = 16.0, 3.0, 8.0, 2.0    # TPU foot cap (box catch F) and its plug
RECESS_D, RECESS_H = 8.3, 2.2                         # the plug's recess in the node seat (0.15 a side, 0.2 deep for the bond)
TIGHT_DEG = 25.0                                      # option B: pairs under this may take an eccentric socket
ECC = float(os.environ.get("SCN_ECC", "3.5"))         # option B (DEFAULT): max socket line offset from the node centre; 0 = option A
LIMITS = {16: (np.array([1.0, 0.0, 0.0]), 6.0)}       # node 16: nothing past x + 6 (its layout sphere; camera mount v1 5.45 off it)
E_ROD, SIG_C = 100000.0, 600.0                        # carbon (E, the frame model's), MPa
ROD_A, ROD_Z = math.pi * 1.5 ** 2, math.pi * 3.0 ** 3 / 32.0
M_ROD = SIG_C * ROD_Z                                 # 1590 N mm: a 3.0 rod's bending at its 600 MPa allowable (E)
K_BUCK = 0.7                                          # rod ends epoxied in PETG sockets (between pinned 1.0 and fixed 0.5, E)
BOND_TAU = 6.0                                        # MPa epoxy on roughened PETG, lap shear (E)
RIM_EX = 1.0                                          # FE: cells this close to a bonded rod's rim are left out (singular corner)
STUB = 8.0                                            # rod stub modelled past the mouth (FE)
PETG_RHO = 1.27
FILLET = 2.5                                          # smooth-union blend at every boss / hub junction (fillet ~ r 2.5, DEFAULT 28 Oct)
MC_PITCH = 0.2                                        # the blend's sampling pitch


def is_centre(i): return abs(N[i, 1]) < 0.5
def mirror_of(i):
    t = N[i] * [1, -1, 1]
    return int(np.argmin(np.linalg.norm(N - t, axis=1)))
def designs(): return [i for i in range(len(N)) if N[i, 1] > -0.5]


OFFS = {}            # option B: (node, member) -> its socket line's offset from the node centre (drone axes), solve_offsets()
OWN = {}             # option B: (node, member) sockets free to run off-centre


def mirror_name(nm):
    if nm.startswith("rod "):
        a, b = (mirror_of(int(v)) for v in nm[4:].split("-")); return f"rod {min(a, b)}-{max(a, b)}"
    return nm[:-2] if nm.endswith(" m") else nm + " m"


def members(i):
    """every member ending at node i: name, far end (its far node's centre, or the point its far socket's line runs through when that
    end is the eccentric one, or the tie's saddle end), length"""
    out = []
    for a, b in CG["rods"]:
        if i in (a, b):
            o = b if a == i else a; nm = f"rod {min(a, b)}-{max(a, b)}"
            out.append(dict(name=nm, far=N[o] + OFFS.get((o, nm), np.zeros(3)), far_node=o, tie=False))
    for k, (p, q) in RES["links"].items():
        for e, o in ((p, q), (q, p)):
            if np.linalg.norm(np.array(e) - N[i]) < 0.6:
                out.append(dict(name=k, far=np.array(o, float), far_node=None, tie=True))
    for m in out: m["L"] = float(np.linalg.norm(m["far"] - N[i]))
    return out


# ---------------------------------------------------------------------------------------------------------- socket layout
def segdist(p0, p1, q0, q1):
    d1, d2, r_ = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r_
    c, b = d1 @ r_, d1 @ d2; den = a * e - b * b
    s = float(np.clip((b * f - c * e) / den, 0, 1)) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0: t = 0.0; s = float(np.clip(-c / a, 0, 1))
    elif t > 1: t = 1.0; s = float(np.clip((b - c) / a, 0, 1))
    return float(np.linalg.norm(p0 + d1 * s - q0 - d2 * t))


def _perp(d):
    x = np.cross(d, [0, 0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0, 0]); x /= np.linalg.norm(x)
    return x, np.cross(d, x)


def owned_offsets():
    """option B: which socket of each tight pair may run off-centre (one end of a rod only; no rod crossing the mirror plane;
    no offsets at centre-plane nodes, which stay mirror-symmetric by construction)"""
    if ECC <= 0: return {}
    own = {}
    for i in designs():
        if is_centre(i): continue
        M = members(i)
        for p in range(len(M)):
            for q in range(p + 1, len(M)):
                dp, dq = [(m["far"] - N[i]) / m["L"] for m in (M[p], M[q])]
                if math.degrees(math.acos(np.clip(dp @ dq, -1, 1))) >= TIGHT_DEG: continue
                for m in (M[p], M[q]):
                    fn = m["far_node"]
                    if fn is not None and (fn == mirror_of(i) or (fn, m["name"]) in own): continue
                    own[(i, m["name"])] = True
    return own


def layout(i, walls=None, free=False):
    """per-socket line origin P (offset from the centre, option B), direction d, set-back r (socket floor from P), boss radius R.
    free: the node's own eccentric sockets (OWN) are variables (solve_offsets); else their offsets are the frozen OFFS"""
    M = members(i); n = len(M); walls = walls or {}
    R = np.array([BORE / 2 + walls.get(m["name"], WALL0) for m in M])
    dirs0 = [(m["far"] - N[i]) / m["L"] for m in M]
    ecc = [k for k, m in enumerate(M) if (i, m["name"]) in OWN] if free else []
    basis = {k: _perp(dirs0[k]) for k in ecc}
    lim = LIMITS.get(i)
    P0 = [np.zeros(3) if k in ecc else OFFS.get((i, M[k]["name"]), np.zeros(3)) for k in range(n)]

    def unpack(x):
        r = x[:n]; P = [P0[k].copy() for k in range(n)]
        for j, k in enumerate(ecc):
            u, v = basis[k]; P[k] = u * x[n + 2 * j] + v * x[n + 2 * j + 1]
        d = [((M[k]["far"] - N[i]) - P[k]) / np.linalg.norm((M[k]["far"] - N[i]) - P[k]) for k in range(n)]
        return r, P, d

    def cons(x):
        r, P, d = unpack(x); g = list(r - RMIN)
        for p in range(n):
            for q in range(n):
                if p == q: continue
                if p < q:                                                    # bore to bore: 3.0 of PETG
                    g.append(segdist(P[p] + d[p] * r[p], P[p] + d[p] * (r[p] + DEPTH), P[q] + d[q] * r[q], P[q] + d[q] * (r[q] + DEPTH))
                             - (BORE + max(R[p], R[q]) - BORE / 2))                # the web between two bores is the thicker wall of the two
                # the rod beyond p's mouth clears q's boss (axis from P[q] to its mouth)
                g.append(segdist(P[p] + d[p] * (r[p] + DEPTH), P[p] + d[p] * 300.0, P[q], P[q] + d[q] * (r[q] + DEPTH)) - (ROD_R + R[q] + ROD_CLEAR))
        for j, k in enumerate(ecc):
            g.append(ECC - math.hypot(x[n + 2 * j], x[n + 2 * j + 1]))
        if lim is not None:
            ax, mx = lim
            for k in range(n):
                s_ = math.sqrt(max(0.0, 1 - (d[k] @ ax) ** 2)) * R[k]
                for s in (0.0, r[k] + DEPTH): g.append(mx - (P[k] @ ax + s * (d[k] @ ax) + s_))
        return np.array(g)

    def cost(x):
        r = x[:n]; e = sum(math.hypot(x[n + 2 * j], x[n + 2 * j + 1]) for j in range(len(ecc)))
        return float(r.sum() + 2.0 * r.max() + 3.0 * e)

    from scipy.optimize import minimize
    rng = np.random.default_rng(i); best = None
    for trial in range(12):
        x0 = np.concatenate([RMIN + rng.uniform(0, 15, n), rng.uniform(-ECC, ECC, 2 * len(ecc)) * 0.7])
        for j, k in enumerate(ecc):                                       # warm start from the last pass
            if trial % 2 == 1 and (i, M[k]["name"]) in OFFS:
                u, v = basis[k]; o = OFFS[(i, M[k]["name"])]; x0[n + 2 * j], x0[n + 2 * j + 1] = o @ u, o @ v
        if trial == 0: x0[:n] = 30.0
        res = minimize(cost, x0, method="COBYLA", constraints=[dict(type="ineq", fun=cons)], options=dict(maxiter=4000, rhobeg=3.0, tol=1e-4))
        x = res.x; ok = cons(x).min() > -1e-3
        if ok and (best is None or cost(x) < cost(best)): best = x
    if best is None: raise RuntimeError(f"node {i}: no feasible socket layout")
    r, P, d = unpack(best)
    socks = [dict(name=M[k]["name"], d=d[k], P=P[k], r=float(r[k]), R=float(R[k]), far=M[k]["far"], far_node=M[k]["far_node"],
                  L=M[k]["L"], tie=M[k]["tie"], ecc=float(np.linalg.norm(P[k]))) for k in range(n)]
    return socks


def symmetrise(i, socks):
    """a centre-plane node keeps perfect mirror symmetry: a mirror pair of sockets takes the larger set-back of the two"""
    if not is_centre(i): return socks
    for a in socks:
        for b in socks:
            if a is not b and np.allclose(a["d"] * [1, -1, 1], b["d"], atol=1e-6):
                a["r"] = b["r"] = max(a["r"], b["r"])
    return socks


def solve_offsets(passes=3, log=print, cache=None):
    """option B: the eccentric sockets' offsets, solved node by node and passed on (a rod is off-centre at one end only, its other
    end aims at that point), then frozen; mirror nodes take the mirror offsets. cache: offsets.json of the run (read if there and
    made with this ECC; SCN_RESOLVE=1 solves again)"""
    OFFS.clear(); OWN.clear()
    if ECC <= 0: return
    if cache and os.path.exists(cache) and not os.environ.get("SCN_RESOLVE"):
        c = json.load(open(cache))
        if c["ecc"] == ECC:
            for k, v in c["offsets"].items(): i, nm = k.split(" | "); OFFS[(int(i), nm)] = np.array(v)
            for k in c["own"]: i, nm = k.split(" | "); OWN[(int(i), nm)] = True
            log(f"eccentric sockets from {os.path.relpath(cache, ROOT)}"); return
    OWN.update(owned_offsets())
    for ps in range(passes):
        for i in designs():
            if is_centre(i) or not any(k[0] == i for k in OWN): continue
            for s in layout(i, free=True):
                if (i, s["name"]) in OWN:
                    OFFS[(i, s["name"])] = s["P"].copy(); OFFS[(mirror_of(i), mirror_name(s["name"]))] = s["P"] * [1, -1, 1]
    log("eccentric sockets: " + ", ".join(f"node {i} {nm} {np.linalg.norm(v):.2f}" for (i, nm), v in sorted(OFFS.items()) if N[i, 1] > 0.5))
    if cache:
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        json.dump(dict(ecc=ECC, offsets={f"{i} | {nm}": np.round(v, 6).tolist() for (i, nm), v in OFFS.items()},
                       own=[f"{i} | {nm}" for (i, nm) in OWN]), open(cache, "w"), indent=1)


# ---------------------------------------------------------------------------------------------------------- geometry
def seg(a, b, r, n=40):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; L = float(np.linalg.norm(d))
    z = d / L; x, y = _perp(z)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, a
    return mf.Manifold.cylinder(L, r, r, n).transform(T[:3, :4])


def up_dir(i, socks):
    """print 'up': the bed face is the node's outside; up maximises the lowest member elevation (every boss rises from the bed)"""
    if i in RING: return np.array([0.0, 0, 1.0])
    D = np.array([s["d"] for s in socks])
    g = np.random.default_rng(0).normal(size=(40000, 3)); g /= np.linalg.norm(g, axis=1)[:, None]
    if is_centre(i): g[:, 1] = 0.0; g /= np.linalg.norm(g, axis=1)[:, None]
    sc = (g @ D.T).min(1); u = g[np.argmax(sc)]
    from scipy.optimize import minimize
    f = lambda v: -np.min(D @ (v / np.linalg.norm(v)))
    u = minimize(f, u, method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-8)).x; u /= np.linalg.norm(u)
    if is_centre(i): u[1] = 0.0; u /= np.linalg.norm(u)
    return u


def _sd_cyl(P, a, b, r):
    """exact signed distance to a flat-ended cylinder a..b of radius r"""
    ba = b - a; L = float(np.linalg.norm(ba)); u = ba / L
    v = P - a; t = v @ u; rad = np.linalg.norm(v - t[:, None] * u[None], axis=1)
    dx, dy = rad - r, np.abs(t - L / 2) - L / 2
    return np.minimum(np.maximum(dx, dy), 0.0) + np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))


def _smin(a, b, k):
    h = np.clip(k - np.abs(a - b), 0.0, None) / k
    return np.minimum(a, b) - h * h * k / 4.0


def blend(prims, k=None, pitch=None):
    """the union of the drawn bodies with every concave junction filleted (smooth union, k = FILLET): the crotches between bosses,
    and between a boss and the hub, were the across-layer peaks of the first FE pass (1.8 on node 1, sharp). Returns a manifold."""
    from skimage import measure
    k = k or FILLET; pitch = pitch or MC_PITCH
    lo = np.min([np.minimum(a, b) - r for kind, a, b, r in prims], 0) - k - 2 * pitch
    hi = np.max([np.maximum(a, b) + r for kind, a, b, r in prims], 0) + k + 2 * pitch
    xs, ys, zs = (np.arange(lo[j], hi[j] + pitch, pitch) for j in range(3))
    F = np.empty((len(xs), len(ys), len(zs)), np.float32)
    YY, ZZ = np.meshgrid(ys, zs, indexing="ij")
    for ix, x in enumerate(xs):
        P = np.column_stack([np.full(YY.size, x), YY.ravel(), ZZ.ravel()]); d = None
        for kind, a, b, r in prims:
            e = np.linalg.norm(P - a, axis=1) - r if kind == "sphere" else _sd_cyl(P, a, b, r)
            d = e if d is None else _smin(d, e, k)
        F[ix] = d.reshape(YY.shape)
    F[0], F[-1], F[:, 0], F[:, -1], F[:, :, 0], F[:, :, -1] = 1, 1, 1, 1, 1, 1
    F[np.abs(F) < 1e-4] = 1e-4                                            # no sample exactly on the surface (degenerate MC faces)
    v, f, _, _ = measure.marching_cubes(F, 0.0, spacing=(pitch, pitch, pitch))
    v = v + lo; t = trimesh.Trimesh(v, f[:, ::-1], process=True)
    if t.volume < 0: t.invert()
    try:
        t2 = t.simplify_quadric_decimation(face_count=min(len(t.faces), 30000))
        if t2.is_watertight and abs(t2.volume - t.volume) < 0.003 * abs(t.volume): t = t2
    except Exception:
        pass
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def bodies(i, socks, hub_r=None):
    """node i in its own frame (centre at the origin, drone axes): solid, bores, the bed plane (up, height)"""
    ring = i in RING
    hub_r = hub_r or (RING_HUB_R if ring else HUB_R)
    O = np.zeros(3)
    prims = [("sphere", O, O, hub_r)]
    bores = None
    for s in socks:
        a, d = s["P"], s["d"]
        prims.append(("cyl", a, a + d * (s["r"] + DEPTH), s["R"]))
        b = seg(a + d * s["r"], a + d * (s["r"] + DEPTH + 0.01), BORE / 2) + seg(a + d * (s["r"] + DEPTH), a + d * (s["r"] + DEPTH + 300.0), BORE / 2 + 0.4)
        bores = b if bores is None else bores + b                        # the bore, and the rod's path out of the mouth 0.4 clear
    u = up_dir(i, socks)
    if ring:
        prims.append(("cyl", np.array([0, 0, -PAD_DZ - 2.0]), O, RING_HUB_R))   # seat column, cut flat at the seat below
        recess = seg([0, 0, -PAD_DZ - 1.0], [0, 0, -PAD_DZ + RECESS_H], RECESS_D / 2)
        zb = -PAD_DZ
        cut_extra = recess
    else:
        zb = min([float(s["P"] @ u - s["R"] * math.sqrt(max(0.0, 1 - (s["d"] @ u) ** 2))) for s in socks] + [-hub_r + 0.9])
        prims.append(("cyl", u * (zb - 2.0), O, 4.0))                    # a foot under the hub: the flat bed face, Ø8 at least
        cut_extra = None
    body = blend(prims)
    # cut everything below the bed plane (a half space along -u)
    big = mf.Manifold.cube([400, 400, 400], center=True).translate([0, 0, -200.0])
    T = np.eye(4); z = u; x, y = _perp(z); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z; T[:3, 3] = u * zb
    body = body - big.transform(T[:3, :4])
    cuts = bores if cut_extra is None else bores + cut_extra
    return dict(solid=body - cuts, bores=bores, cuts=cuts, up=u, zb=zb, hub_r=hub_r)


def to_tm(m):
    me = m.to_mesh()
    t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True, validate=True)
    return clean(t)


def clean(t):
    """drop the zero-area face doublets a boolean on a marching-cubes surface leaves (collinear vertices; no volume, no feature)"""
    b = t.split(only_watertight=False)
    if len(b) == 1: return t
    keep = [x for x in b if x.area > 1e-6]                                  # a component of zero area only: never a feature
    return trimesh.util.concatenate(keep) if len(keep) > 1 else keep[0]


def save(t, f):
    """export as STL and make sure the file reads back as one watertight body (float32 rounding can make new zero-area doublets)"""
    t.export(f); r = clean(trimesh.load(f, force="mesh"))
    if len(r.faces) != len(trimesh.load(f, force="mesh").faces): r.export(f); r = trimesh.load(f, force="mesh")
    b = r.split(only_watertight=False)
    if not (r.is_watertight and len(b) == 1): raise RuntimeError(f"{f}: {len(b)} bodies, watertight {r.is_watertight}")
    return r


def print_pose(u, zb):
    """rotation + translation taking the node frame to the print frame: up -> +z, bed plane at z 0"""
    R = trimesh.geometry.align_vectors(u, [0, 0, 1.0])
    R[:3, 3] = [0, 0, -zb]
    return R


def mirror_tm(t):
    m = t.copy(); m.apply_transform(np.diag([1, -1, 1, 1])); m.invert() if m.volume < 0 else None; m.fix_normals(); return m


def cap_mesh():
    """TPU foot cap (box catch F): disc Ø16 x 3 on the bed, plug Ø8 x 2 up into the node seat's recess (bonded)"""
    c = seg([0, 0, 0], [0, 0, CAP_T], CAP_D / 2, 64) + seg([0, 0, CAP_T - 0.01], [0, 0, CAP_T + PLUG_H], PLUG_D / 2, 64)
    return to_tm(c)


# ---------------------------------------------------------------------------------------------------------- loads
def pcr(L):
    I = math.pi * 3.0 ** 4 / 64.0
    return math.pi ** 2 * E_ROD * I / (K_BUCK * L) ** 2


def socket_loads(i, s):
    fl = NL.get(str(i), {}).get(s["name"]) or {}
    ax_f, side_f = float(fl.get("axial", 50.0)), float(fl.get("side", 10.0))
    Lfree = max(s["L"] - 2 * (s["r"] + DEPTH), 20.0)               # free length between the two mouths (this end's set-back both ends, E)
    push = min(ax_f, pcr(Lfree), SIG_C * ROD_A)
    pull = ax_f
    bond = BOND_TAU * math.pi * BORE * DEPTH
    return dict(push=push, pull=pull, side=side_f, moment=M_ROD, bond=bond, pcr=pcr(Lfree), frame_axial=ax_f, Lfree=Lfree)


# ---------------------------------------------------------------------------------------------------------- FE check
def fe_check(i, socks, B, h=0.5, log=print):
    """as-built voxel FE in the print pose (layers along z): per socket 6 cases, the other rods' stubs held. Returns rows + worst per socket"""
    from topo.part import Part, man, tm
    from topo.fem import Solver
    from topo.material import MATERIALS, stiffness, allowables
    Tp = print_pose(B["up"], B["zb"]); Rr = Tp[:3, :3]
    def tf(m): return m.transform(Tp[:3, :4])
    solid = tf(B["solid"]); cuts = tf(B["cuts"])
    stubs, holds, tips, necks, rods = [], [], [], [], []
    for s in socks:
        a = Rr @ s["P"] + Tp[:3, 3]; d = Rr @ s["d"]; m0 = a + d * (s["r"] + DEPTH)
        rods.append(seg(a + d * s["r"], m0 + d * STUB, ROD_R, 24))
        holds.append(seg(m0 + d * 3.0, m0 + d * STUB, ROD_R + 0.3, 24))
        tips.append(seg(m0 + d * (STUB - 2.5), m0 + d * STUB, ROD_R + 0.3, 24))
        necks.append(seg(m0 + d * 0.5, m0 + d * 2.5, ROD_R + 0.3, 24))
    env = solid
    for r_ in rods: env = env + r_
    quiet = lambda s_: None
    p = Part(f"scout_node_{i}", h, solid, cuts, env, build="+z", material="PETG", log=quiet)
    for k, s in enumerate(socks):
        Lr = s["r"] and (DEPTH + STUB)
        p.insert(f"rod {k}", rods[k], E_ROD / MATERIALS["PETG"].E, area_mm2=ROD_A, length_mm=DEPTH + STUB)
    cases = []
    for k, s in enumerate(socks):
        hold = None
        for t, hh in enumerate(holds):
            if t != k: hold = hh if hold is None else hold + hh
        p.support(f"hold {k}", hold)
        Ld = socket_loads(i, s); d = Rr @ s["d"]; x, y = _perp(d)
        Fc = Ld["moment"] / (STUB - 1.25 - 1.5)                       # the couple: tip centre (STUB - 1.25) to neck centre (1.5) past the mouth
        p.load(f"{k} push", tips[k], -Ld["push"] * d, cls="crash", support=f"hold {k}")
        p.load(f"{k} pull", tips[k], Ld["pull"] * d, cls="crash", support=f"hold {k}")
        for nm, v in (("+a", x), ("-a", -x), ("+b", y), ("-b", -y)):
            p.load_multi(f"{k} bend {nm}", [(tips[k], (Fc + Ld["side"]) * v), (necks[k], -Fc * v)], cls="crash", support=f"hold {k}")
    mat = MATERIALS["PETG"]; C = stiffness(mat, 2)
    from topo.part import voxelise
    A = voxelise(to_tm(solid), *p.axes).ravel() & ~p.cut
    ins = p.insert_cells
    act = A | ins
    sol = Solver(p.shape, h, C, p.supports, active=act, backend="auto", log=quiet)
    g = sol.grid; E = np.where(ins, p.stiff, np.where(A, 1.0, 1e-9))
    edof = g.edof(); enodes = edof[:, ::3] // 3; CB = C @ sol.B0
    Mv = np.array([[1, -.5, -.5, 0, 0, 0], [-.5, 1, -.5, 0, 0, 0], [-.5, -.5, 1, 0, 0, 0], [0, 0, 0, 3, 0, 0], [0, 0, 0, 0, 3, 0], [0, 0, 0, 0, 0, 3.]])
    cen = p.cell_centres(); rows = []
    # the bonded rod's two rims (mouth and floor edge of each bore) are bi-material corners: a voxel FE reads them as singular
    # cells that grow with refinement (the real joint has an epoxy fillet there): cells within RIM_EX of a rim circle are left out
    rim = np.zeros(p.nel, bool)
    for s in socks:
        a = Rr @ s["P"] + Tp[:3, 3]; d = Rr @ s["d"]
        for c0 in (a + d * s["r"], a + d * (s["r"] + DEPTH)):
            v = cen - c0; ax_ = v @ d; rad = np.linalg.norm(v - ax_[:, None] * d[None], axis=1)
            rim |= np.hypot(ax_, rad - BORE / 2) < RIM_EX
    # the bond line: PETG cells sharing a node with the rod insert (a voxel bore is a staircase round a 36x stiffer rod, so single
    # cells there read 2..4x their neighbours). The bond itself is checked by hand (socket_loads "bond"); the wall behind it here.
    ins_nodes = np.zeros(p.nn, bool); ins_nodes[enodes[ins].ravel()] = True
    rim |= ins_nodes[enodes].any(1)
    for k in range(len(socks)):
        cs = [c for c in p.cases if c.support == f"hold {k}"]
        F = np.stack([c.F for c in cs], 1)
        U = sol.solve(E, F, [c.support for c in cs])
        hot = np.zeros(p.nn, bool)
        for c in cs: hot[c.nodes] = True
        hot[p.supports[f"hold {k}"].any(1)] = True
        keep = A & ~ins & ~hot[enodes].any(1) & ~rim; idx = np.where(keep)[0]
        Axy, Az, As = allowables(mat, "crash")
        for q, c in enumerate(cs):
            sg = U[:, q][edof[idx]] @ CB.T
            vm = np.sqrt(np.maximum(np.einsum("ei,ij,ej->e", sg, Mv, sg), 0.0))
            sn = np.maximum(sg[:, 2], 0.0); tau2 = sg[:, 4] ** 2 + sg[:, 5] ** 2
            f1 = vm / Axy; f2 = np.sqrt((sn / Az) ** 2 + tau2 / As ** 2); f = np.maximum(f1, f2)
            # report the 99.9th percentile with the max (a voxel corner at a re-entrant edge reads high on one cell)
            j = int(np.argmax(f)); p999 = float(np.percentile(f, 99.9))
            at_w = Rr.T @ (cen[idx[j]] - Tp[:3, 3])                       # back to the node frame
            rows.append(dict(socket=k, name=socks[k]["name"], case=c.name, max=round(float(f[j]), 3), p999=round(p999, 3),
                             inlayer=round(float(f1[j]), 3), across=round(float(f2[j]), 3), at_node=np.round(at_w, 1).tolist(),
                             at_r=round(float(np.linalg.norm(at_w)), 1)))
        del U
    worst = {}
    for r_ in rows:
        w = worst.setdefault(r_["socket"], r_)
        if r_["max"] > w["max"]: worst[r_["socket"]] = r_
    return rows, worst, dict(grid=list(p.shape), h=h, cells=int(A.sum()))


# ---------------------------------------------------------------------------------------------------------- build
def node_name(i): return f"scout_node_{i}"


def design(i, fe=False, h=0.5, log=print):
    walls, hub = {}, None; hist = []
    for it in range(6):
        socks = symmetrise(i, layout(i, walls))
        B = bodies(i, socks, hub)
        if not fe: break
        rows, worst, info = fe_check(i, socks, B, h=h, log=log)
        w = max(worst.values(), key=lambda r: r["max"])
        hist.append(dict(iter=it, walls={s["name"]: round(s["R"] - BORE / 2, 2) for s in socks}, hub_r=B["hub_r"], worst=w,
                         per_socket={socks[k]["name"]: dict(p999=v["p999"], max=v["max"], case=v["case"], at_r=v["at_r"]) for k, v in worst.items()}))
        log(f"  node {i} iter {it}: worst {w['max']} (99.9 % {w['p999']}) in {w['case']} ({w['name']}) at r {w['at_r']}")
        bad = [k for k, v in worst.items() if v["max"] > 1.0]
        if not bad: break
        for k in bad:
            v = worst[k]; s = socks[k]
            if v["at_r"] <= B["hub_r"] + 0.5: hub = B["hub_r"] + 1.0     # at the hub: a larger hub
            else: walls[s["name"]] = walls.get(s["name"], WALL0) + 0.5
        if is_centre(i):                                                  # keep a mirror pair's walls equal
            for a in socks:
                for b in socks:
                    if a is not b and np.allclose(a["d"] * [1, -1, 1], b["d"], atol=1e-6):
                        m = max(walls.get(a["name"], WALL0), walls.get(b["name"], WALL0)); walls[a["name"]] = walls[b["name"]] = m
    return socks, B, hist


def envelope(i, t_local):
    """the node's envelope in its own frame: max radius from the centre, and its extents along the drone axes"""
    v = t_local.vertices
    return dict(r_max=round(float(np.linalg.norm(v, axis=1).max()), 1), lo=np.round(v.min(0), 1).tolist(), hi=np.round(v.max(0), 1).tolist())


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd"); ap.add_argument("out", nargs="?")
    ap.add_argument("--fe", action="store_true"); ap.add_argument("--h", type=float, default=0.5)
    ap.add_argument("--nodes", default=None)
    a = ap.parse_args()
    ids = [int(v) for v in a.nodes.split(",")] if a.nodes else designs()
    out = a.out or os.path.join(ROOT, "runs", "scout_nodes", "v1")
    solve_offsets(cache=os.path.join(out, "offsets.json"))
    if a.cmd == "list":
        tot = 0.0
        for i in ids:
            socks = symmetrise(i, layout(i)); B = bodies(i, socks); t = to_tm(B["solid"])
            g = t.volume / 1000 * PETG_RHO; tot += g * (1 if is_centre(i) else 2)
            print(f"node {i:2d} {np.round(N[i], 1).tolist()}: {len(socks)} sockets, set-backs " +
                  ", ".join(f"{s['name']} {s['r']:.1f}" + (f" (ecc {s['ecc']:.1f})" if s["ecc"] > 0.05 else "") for s in socks) +
                  f"; {g:.2f} g, envelope r {envelope(i, t)['r_max']}" + ("" if is_centre(i) else " (+ its mirror)"), flush=True)
        print(f"all 18 nodes {tot:.1f} g (C)")
        return
    if a.cmd == "fe":                                                     # one FE pass at the drawn minimum (no sizing)
        for i in ids:
            t0 = time.time(); socks = symmetrise(i, layout(i)); B = bodies(i, socks)
            rows, worst, info = fe_check(i, socks, B, h=a.h)
            import resource
            print(f"node {i}: grid {info['grid']} h {info['h']}, {info['cells']} cells, {time.time() - t0:.0f} s, peak "
                  f"{resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6:.2f} GB", flush=True)
            for k, v in worst.items():
                print(f"   {v['name']:16s} worst {v['max']:.2f} (99.9 % {v['p999']:.2f}) in {v['case']} at r {v['at_r']}", flush=True)
        return
    if a.cmd == "merge": merge(out); return
    build_all(ids, out, fe=a.fe, h=a.h)


def build_all(ids, out, fe=False, h=0.5):
    """each node: its print file (+ the mirror print), its placed copies, nodes/<i>.json; then merge()"""
    os.makedirs(os.path.join(out, "placed"), exist_ok=True); os.makedirs(os.path.join(out, "nodes"), exist_ok=True)
    from solid_check import solid_check
    for i in ids:
        t0 = time.time()
        socks, B, hist = design(i, fe=fe, h=h)
        loc = to_tm(B["solid"])                                          # node frame, centre at the origin
        Tp = print_pose(B["up"], B["zb"]); pr = loc.copy(); pr.apply_transform(Tp)
        f = os.path.join(out, f"{node_name(i)}_v1.stl"); pr = save(pr, f)
        ok, info = solid_check(pr)
        g = pr.volume / 1000 * PETG_RHO
        socket_rows = []
        for s in socks:
            Ld = socket_loads(i, s)
            socket_rows.append(dict(name=s["name"], dir=np.round(s["d"], 4).tolist(), line_origin=np.round(s["P"], 2).tolist(), ecc=round(s["ecc"], 2),
                                    setback=round(s["r"], 2), mouth_r=round(s["r"] + DEPTH, 2), wall=round(s["R"] - BORE / 2, 2),
                                    mouth_drone=np.round(N[i] + s["P"] + s["d"] * (s["r"] + DEPTH), 2).tolist(),
                                    floor_drone=np.round(N[i] + s["P"] + s["d"] * s["r"], 2).tolist(),
                                    elev_deg=round(math.degrees(math.asin(np.clip(s["d"] @ B["up"], -1, 1))), 1),
                                    loads={k: round(v, 1) for k, v in Ld.items()}))
        env = envelope(i, loc)
        rec = dict(centre=N[i].tolist(), mirror=None if is_centre(i) else mirror_of(i), file=os.path.relpath(f, ROOT), mass_g=round(g, 2),
                   solid_check=dict(ok=bool(ok), bodies_eroded=info["bodies_eroded"], thin_walls=len(info["thin_walls"]), necks=len(info["necks"])),
                   up=np.round(B["up"], 4).tolist(), bed_below_centre=round(-B["zb"], 2), hub_r=B["hub_r"], envelope=env, sockets=socket_rows,
                   fe=hist, extents_print=np.round(pr.extents, 1).tolist())
        placed = []
        pl = loc.copy(); pl.apply_translation(N[i]); pf = os.path.join(out, "placed", f"node_{i}.stl"); save(pl, pf)
        placed.append(dict(name=f"cage node {i}", mesh=os.path.relpath(pf, ROOT), node=i, mass_g=round(g, 2), tag="M", design=i, mirror=False))
        if not is_centre(i):
            j = mirror_of(i); mt = mirror_tm(loc); mt.apply_translation(N[j])
            mf_ = os.path.join(out, "placed", f"node_{j}.stl"); save(mt, mf_)
            placed.append(dict(name=f"cage node {j}", mesh=os.path.relpath(mf_, ROOT), node=j, mass_g=round(g, 2), tag="M", design=i, mirror=True))
            mp = mirror_tm(pr); save(mp, os.path.join(out, f"{node_name(i)}_v1_mirror.stl"))
        rec["placed"] = placed
        json.dump(rec, open(os.path.join(out, "nodes", f"{i}.json"), "w"), indent=1)
        print(f"node {i}: {g:.2f} g, solid_check {'PASS' if ok else 'FAIL'}, envelope r {env['r_max']}, "
              f"{len(socks)} sockets, {time.time() - t0:.0f} s" + (f", FE worst {hist[-1]['worst']['max']}" if hist else ""), flush=True)


def rod_ends(out):
    """every rod's two socket floors (drone frame) and its cut length: floor to floor less 0.5 (C); ties: node floor to the saddle end
    point (the saddle's own socket depth not taken off)"""
    fl = {}
    for f in os.listdir(os.path.join(out, "nodes")):
        rec = json.load(open(os.path.join(out, "nodes", f))); i = int(f[:-5])
        for s in rec["sockets"]:
            fl[(i, s["name"])] = np.array(s["floor_drone"])
            if rec["mirror"] is not None: fl[(rec["mirror"], mirror_name(s["name"]))] = np.array(s["floor_drone"]) * [1, -1, 1]
    rows = []
    for a, b in CG["rods"]:
        nm = f"rod {min(a, b)}-{max(a, b)}"
        if (a, nm) in fl and (b, nm) in fl:
            pa, pb = fl[(a, nm)], fl[(b, nm)]
            ends = [n_ for n_ in (a, b) if (n_, nm) in OFFS and np.linalg.norm(OFFS[(n_, nm)]) > 0.05]
            rows.append(dict(rod=nm, floor_a=np.round(pa, 2).tolist(), floor_b=np.round(pb, 2).tolist(), cut_mm=round(float(np.linalg.norm(pa - pb)) - 0.5, 1),
                             ecc_end=ends, ecc_mm=round(max([float(np.linalg.norm(OFFS[(n_, nm)])) for n_ in ends] or [0.0]), 2)))
    for k, (p, q) in RES["links"].items():
        for i in range(len(N)):
            for e, o in ((p, q), (q, p)):
                if np.linalg.norm(np.array(e) - N[i]) < 0.6 and (i, k) in fl:
                    rows.append(dict(rod=k, floor_a=np.round(fl[(i, k)], 2).tolist(), floor_b=list(o), cut_mm=round(float(np.linalg.norm(fl[(i, k)] - np.array(o))), 1),
                                     note="node socket floor to the saddle end point; less the saddle socket's depth"))
    return rows


def merge(out):
    res = dict(option="B" if ECC > 0 else "A", ecc_max=ECC, bore=BORE, depth=DEPTH, wall_min=WALL0, floor_min=RMIN, hub_r=HUB_R,
               ring=dict(hub_r=RING_HUB_R, seat_below_centre=PAD_DZ, cap=[CAP_D, CAP_T], plug=[PLUG_D, PLUG_H], recess=[RECESS_D, RECESS_H]),
               moment_Nmm=round(M_ROD, 1), k_buckling=K_BUCK, bond_tau=BOND_TAU, rim_excluded_mm=RIM_EX, nodes={})
    placed = []
    for i in designs():
        f = os.path.join(out, "nodes", f"{i}.json")
        if not os.path.exists(f): continue
        rec = json.load(open(f)); placed += rec.pop("placed"); res["nodes"][str(i)] = rec
    cap = cap_mesh(); save(cap, os.path.join(out, "scout_foot_cap_v1.stl"))
    for j in RING:
        c = cap_mesh(); c.apply_translation(N[j] + [0, 0, -PAD_DZ - CAP_T])         # disc under the seat, plug up into its recess
        cf = os.path.join(out, "placed", f"foot_cap_{j}.stl"); save(c, cf)
        placed.append(dict(name=f"foot cap {j} (TPU)", mesh=os.path.relpath(cf, ROOT), node=j, mass_g=round(cap.volume / 1000 * 1.22, 2), tag="M",
                           material="TPU"))
    res["cap"] = dict(file="scout_foot_cap_v1.stl", mass_g=round(cap.volume / 1000 * 1.22, 2), contact_below_centre=PAD_DZ + CAP_T)
    res["total_nodes_g"] = round(sum(p["mass_g"] for p in placed if p["name"].startswith("cage node")), 1)
    res["eccentric"] = {f"node {i} {nm}": round(float(np.linalg.norm(v)), 2) for (i, nm), v in sorted(OFFS.items()) if np.linalg.norm(v) > 0.05}
    res["rods"] = rod_ends(out)
    json.dump(res, open(os.path.join(out, "result.json"), "w"), indent=1)
    json.dump(sorted(placed, key=lambda p: (p["node"], p["name"])), open(os.path.join(out, "placed.json"), "w"), indent=1)
    print(f"merged {len(res['nodes'])} designs, {len(placed)} placed meshes, nodes {res['total_nodes_g']} g")
    return res


if __name__ == "__main__":
    main()
