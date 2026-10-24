"""Box structure v2: the cut of the round 2 topology field into printable pieces and the system FE on the cut assembly.
(docs/generative.md The rule, 0: system first, then cut; this is the cut and the assembled recheck.)

Stages (python3 box_system_v2_cut.py --stage structure|cut|fe|report|all):
  structure  the 0.5 surface of runs/box_system/v2/topo_r2/density.npz (g3000, cubic upsample x4 -> h 2), the drawn bodies
             (pads, receiver bosses, seat posts, stub, 8 electronics bosses) added exact, the keep-outs (grown 1) cut exact
  cut        partition into pieces by planes / wedges (exact CSG), a bolted half lap at every cut plane (drawn: overlap 16,
             M3 / M5 heat-set inserts in bosses, pads >= O12 with >= 2.0 web), TPU feet at the floor patches
  fe         system FE (h 4) on the cut assembly, piece-labelled voxel model (box_system.Model), unilateral floor on the feet
  report     requirements, assembly file, visual page
Outputs: runs/box_system/v2/cut/. Everything is in W (box-arm.md). Tags: C geometry / drawn, M mesh, E estimate, F this model."""
import os, sys, json, math, time, argparse, pickle, re
HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(NOX, ".."))
for p in (os.path.join(HERE, "stand"), HERE, NOX):
    if p not in sys.path: sys.path.insert(0, p)
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import numpy as np, trimesh, manifold3d as mf
from scipy import ndimage as ndi
from skimage import measure
import scipy.sparse as sp
import box_system as B

OUT = os.path.join(ROOT, "runs", "box_system", "v2", "cut")
FIELD = os.path.join(ROOT, "runs", "box_system", "v2", "topo_r2", "density.npz")
CACHE = os.environ.get("TOPO_CACHE", "/tmp/claude-0/-home-user-nox/48926873-2ab8-5adf-8e03-a2e97c4fb8f1/scratchpad")
os.makedirs(CACHE, exist_ok=True)
RHO = 1.27e-3                                      # PETG g / mm3 (E)


def tm(m):
    g = m.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def union(ms):
    ms = [m for m in ms if m is not None]
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else ms[0]


def log(s, f=[None]):
    print(s, flush=True)
    if f[0] is None:
        os.makedirs(OUT, exist_ok=True); f[0] = open(os.path.join(OUT, "log.txt"), "a")
    f[0].write(s + "\n"); f[0].flush()


# ================================================================================================ structure
def drawn_inputs():
    """the round 2 drawn bodies and keep-outs as manifolds (box_system_topo_v2r2.setup_factory at h 8 = the run's setup)."""
    f = os.path.join(CACHE, "v2cut_inputs.pkl")
    if os.path.exists(f):
        return _load_inputs(pickle.load(open(f, "rb")))
    import box_system_topo_v2r2 as R2
    info = {}
    setup, geom = R2.setup_factory(lambda s: None, info)
    setup(8.0)
    D = dict(drawn=tm(geom["drawn"]), cuts=tm(geom["cuts"]), env=tm(geom["env"]), fun=tm(geom["fun"]),
             posts=[tm(m) for m in geom["posts"]], eboss=[tm(m) for m in geom["eboss"]], stub=tm(geom["stub"]),
             bosses={a: tm(m) for a, m in geom["bosses"].items()}, pads={a: tm(m) for a, m in geom["pads"].items()})
    pickle.dump(D, open(f, "wb"))
    return _load_inputs(D)


def _load_inputs(D):
    out = {}
    for k, v in D.items():
        if isinstance(v, list): out[k] = [man(t) for t in v]
        elif isinstance(v, dict): out[k] = {a: man(t) for a, t in v.items()}
        else: out[k] = man(v)
    return out


def field_mesh(K=4):
    """0.5 surface of the round 2 field (rho_n, drawn cells 1), cubic upsample xK, marching cubes (M)."""
    d = np.load(FIELD)
    h = float(d["h"]); o = np.asarray(d["origin"]); sh = tuple(int(s) for s in d["shape"])
    rho = np.where(d["solid"].reshape(sh), 1.0, d["rho_n"].reshape(sh))
    up = np.clip(ndi.zoom(rho, K, order=3, mode="nearest", grid_mode=True), 0, 1)
    hf = h / K
    v, f, _, _ = measure.marching_cubes(np.pad(up, 1), 0.5, spacing=(hf, hf, hf))
    v = v + o + hf * 0.5 - hf
    m = trimesh.Trimesh(v, f[:, ::-1], process=True)
    trimesh.smoothing.filter_taubin(m, iterations=6)
    return m


def structure(force=False):
    """S = (field surface U drawn) - (keep-outs - drawn), one manifold, specks dropped. Cached as an STL."""
    fn = os.path.join(CACHE, "v2cut_structure.stl")
    if os.path.exists(fn) and not force: return man(trimesh.load(fn, force="mesh"))
    t0 = time.time(); D = drawn_inputs()
    F = man(field_mesh())
    log(f"field surface {F.volume() / 1e3:.0f} cm3 ({time.time() - t0:.0f} s)")
    S = ((F + D["drawn"]) ^ D["env"]) - (D["cuts"] - D["drawn"])
    parts = S.decompose()
    vols = sorted([p.volume() for p in parts], reverse=True)
    log(f"structure {S.volume() / 1e3:.0f} cm3, {len(parts)} bodies, volumes cm3 {[round(v / 1e3, 1) for v in vols[:12]]} ({time.time() - t0:.0f} s)")
    S = union([p for p in parts if p.volume() >= 20000.0])
    S = despeckle(S, log, h=1.0, r_open=1.15, passes=1, exact=False)
    tm(S).export(fn)
    return S


def despeckle(S, log=print, h=1.0, r_open=1.15, tol=0.6, passes=1, exact=False):
    """features thinner than 2 r_open (the sheets and septa the keep-out cuts leave, the solid gate's thin walls) are removed from the
    cut. Voxels h, opening by r_open; the material that opening removes (exact: the gate's own definition, material further than
    r_open from the eroded core) is cut away as a blob one voxel larger. Several passes, until under 2 mm3 is left (M)."""
    from topo.part import voxelise
    tot = 0.0
    for ps in range(passes):
        t = tm(S); lo = np.floor(t.bounds[0]) - 4; hi = np.ceil(t.bounds[1]) + 4
        ax = [lo[k] + (np.arange(int(hi[k] - lo[k])) + 0.5) * h for k in range(3)]
        A = voxelise(t, *ax)
        core = ndi.distance_transform_edt(A) * h > r_open
        near = ndi.distance_transform_edt(~core) * h <= (r_open if exact else r_open + tol)
        thin = A & ~near
        v_thin = thin.sum() * h ** 3
        if v_thin < 2.0: break
        thin_d = ndi.binary_dilation(thin, iterations=1)
        v, f, _, _ = measure.marching_cubes(np.pad(thin_d, 1).astype(np.float32), 0.5, spacing=(h, h, h))
        v = v + lo + 0.5 * h - h
        cut = man(trimesh.Trimesh(v, f[:, ::-1], process=True))
        S2 = S - cut; tot += S.volume() - S2.volume(); S = S2
    if tot > 0: log(f"despeckle: {tot:.0f} mm3 cut away (M)")
    return S


# ================================================================================================ planes and partition
class Plane:
    """cut plane: point p0, unit normal n, in-plane axis u (v = n x u). A is the n < 0 side."""
    def __init__(self, name, n, u, p0):
        self.name = name
        n = np.asarray(n, float); n /= np.linalg.norm(n)
        u = np.asarray(u, float); u = u - (u @ n) * n; u /= np.linalg.norm(u)
        self.n, self.u, self.p0 = n, u, np.asarray(p0, float)
        self.v = np.cross(n, u)
        Rm = np.stack([self.u, self.v, self.n])                                   # world -> local rows
        self.Tl = np.hstack([Rm, (-Rm @ self.p0)[:, None]])
        self.Tw = np.hstack([Rm.T, self.p0[:, None]])

    def loc(self, m):
        return m.transform(self.Tl.tolist())

    def world(self, m):
        return m.transform(self.Tw.tolist())

    def lbox(self, x0, x1, y0, y1, z0, z1):
        """box in local (u, v, n) coordinates, as a world manifold."""
        return self.world(box(x0, x1, y0, y1, z0, z1))

    def to_local(self, p):
        return self.Tl[:, :3] @ np.asarray(p, float) + self.Tl[:, 3]

    def to_world(self, q):
        return self.Tw[:, :3] @ np.asarray(q, float) + self.Tw[:, 3]


BAY_ANG = [30, 0, -30, -60, -90, -120, -150, -180, -210]
BAY_KIND = {30: "S", 0: "O", -30: "S", -60: "O", -90: "S", -120: "O", -150: "S", -180: "O", -210: "S"}   # box_layout_v2.BAYS (S Scout, O Overseer)
BIG = 900.0
Y0_, Y1_ = -90.0, 360.0
PAR = dict(Ym=20.0, XmP=88.0, XmN=-80.0, Ybr=266.0, Yb=60.0, c_r=140.0, Ztop=110.0,
           phi={15: 15.0, -15: -15.0, -45: -45.0, -75: -75.0, -105: -105.0, -135: -135.0, -165: -165.0, -195: -195.0},
           outer=(45.0, -225.0))


def e_r(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def prism_y(poly_xz, y0, y1):
    """polygon in (X, Z) extruded along Y between y0 and y1 (W)."""
    return mf.Manifold.extrude(mf.CrossSection([[tuple(p) for p in poly_xz]]), y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])


def wedge(a0, a1, c_r, ak, y0, y1, R=700.0):
    """sector between the rays a0 < a1 (deg, W: X = cos, Z = sin), cut off at distance c_r along the bay axis ak (planar)."""
    pts = []
    for a in (a0, a1):
        d = e_r(a); k = c_r / max(float(d @ e_r(ak)), 0.2)
        pts.append(d * k)
    P0, P1 = pts
    return prism_y([P0, e_r(a0) * R, e_r(a1) * R, P1], y0, y1)


def radial_plane(name, phi):
    p = math.radians(phi)
    return Plane(name, [-math.sin(p), 0, math.cos(p)], [0, 1, 0], [0, 0, 0])


def regions(P=PAR):
    """ordered (name, region): each takes what the earlier ones left (cuts.py rules, exact planes / wedges). Also returns the planes."""
    R, pl = [], []
    R.append(("mast", box(P["XmN"], P["XmP"], Y0_, P["Ym"], -BIG, BIG)))
    R.append(("rearP", box(P["XmP"], BIG, Y0_, P["Ym"], -BIG, BIG)))
    R.append(("rearN", box(-BIG, P["XmN"], Y0_, P["Ym"], -BIG, BIG)))
    pl += [Plane("Y rear", [0, 1, 0], [1, 0, 0], [0, P["Ym"], 0]), Plane("X mast +", [1, 0, 0], [0, 1, 0], [P["XmP"], 0, 0]),
           Plane("X mast -", [1, 0, 0], [0, 1, 0], [P["XmN"], 0, 0])]
    R.append(("bridge", box(-BIG, BIG, P["Ybr"], Y1_, -BIG, BIG)))
    pl.append(Plane("Y bridge", [0, 1, 0], [1, 0, 0], [0, P["Ybr"], 0]))
    ph = P["phi"]; edges = [P["outer"][0]] + [ph[k] for k in sorted(ph, reverse=True)] + [P["outer"][1]]
    for i, a in enumerate(BAY_ANG):
        hi, lo = edges[i], edges[i + 1]
        R.append((f"bay{a}", wedge(lo, hi, P["c_r"], a, P["Yb"], Y1_)))
    for k in sorted(ph, reverse=True): pl.append(radial_plane(f"R{k}", ph[k]))
    for a in BAY_ANG:
        pl.append(Plane(f"C{a}", [math.cos(math.radians(a)), 0, math.sin(math.radians(a))], [0, 1, 0], np.r_[e_r(a)[0], 0, e_r(a)[1]] * P["c_r"]))
    pl.append(Plane("Y bay", [0, 1, 0], [1, 0, 0], [0, P["Yb"], 0]))
    R.append(("topP", box(0, BIG, Y0_, Y1_, P["Ztop"], BIG)))
    R.append(("topN", box(-BIG, 0, Y0_, Y1_, P["Ztop"], BIG)))
    pl += [Plane("Z top", [0, 0, 1], [1, 0, 0], [0, 0, P["Ztop"]]), Plane("X top", [1, 0, 0], [0, 1, 0], [0, 0, 0])]
    R.append(("hub", box(-BIG, BIG, Y0_, Y1_, -BIG, BIG)))
    return R, pl


def comps(m, vmin=0.0):
    c = sorted(m.decompose(), key=lambda q: -q.volume())
    return [q for q in c if q.volume() >= vmin]


def bbox(m):
    b = m.bounding_box(); return np.array(b).reshape(2, 3)


def partition(S, P=PAR):
    """regions applied to S: name -> manifold; also the plane list."""
    R, pl = regions(P)
    out = {}; left = S
    for name, reg in R:
        out[name] = left ^ reg
        left = left - reg
    return out, pl


def footprint(m, pl, side, th=0.15):
    """2D cross-section (local u, v) of the material of m within th of the plane on one side (side -1: n < 0)."""
    z0, z1 = (-th, 0.0) if side < 0 else (0.0, th)
    s = pl.loc(m) ^ box(-BIG, BIG, -BIG, BIG, z0, z1)
    if s.num_vert() == 0: return None
    return s.project()


def contact_area(A, B, pl):
    """face contact A | B on the plane (mm2): footprint overlap of A's side n < 0 and B's side n > 0 (either order)."""
    best = 0.0
    for a, b in ((A, B), (B, A)):
        fa = footprint(a, pl, -1)
        if fa is None: continue
        fb = footprint(b, pl, +1)
        if fb is None: continue
        best = max(best, (fa ^ fb).area())
    return best


def near_plane(m, pl, tol=1.0):
    b = bbox(m); c = np.array([[x, y, z] for x in b[:, 0] for y in b[:, 1] for z in b[:, 2]])
    sd = (c - pl.p0) @ pl.n
    return sd.min() <= tol and sd.max() >= -tol


def contacts(pieces, planes, amin=40.0):
    """all face contacts between pieces on the cut planes: list of dict(a, b, plane, area mm2, P): a on the n < 0 side, P the
    contact footprint (CrossSection in the plane's local u, v)."""
    out = []
    names = list(pieces)
    for pl in planes:
        cand = [k for k in names if near_plane(pieces[k], pl)]
        fa = {k: footprint(pieces[k], pl, -1) for k in cand}; fb = {k: footprint(pieces[k], pl, +1) for k in cand}
        for x in cand:
            for y in cand:
                if x == y or fa[x] is None or fb[y] is None: continue
                P = fa[x] ^ fb[y]
                if P.area() >= amin: out.append(dict(a=x, b=y, plane=pl, area=P.area(), P=P))
    return out


def fold(parts, planes, vmin=50.0, vfold=8000.0, log=print):
    """parts: name -> manifold. Components under vmin mm3 are dropped (specks); a component other than the largest of its
    piece and under vfold mm3 joins the piece it touches over the largest face area; the rest stay pieces of their own."""
    pieces = {}; small = []
    for rn, m in parts.items():
        cs = comps(m, vmin)
        for i, c in enumerate(cs):
            if i == 0: pieces[rn] = c
            elif c.volume() >= vfold: pieces[f"{rn}~{i}"] = c
            else: small.append((rn, i, c))
    for rn, i, c in sorted(small, key=lambda t: -t[2].volume()):
        best, bk = 0.0, None
        for pl in planes:
            if not near_plane(c, pl): continue
            for k, m in pieces.items():
                if not near_plane(m, pl): continue
                ar = contact_area(c, m, pl)
                if ar > best: best, bk = ar, k
        if bk is None:
            log(f"  isolated component {rn}{i} ({c.volume() / 1e3:.1f} cm3) kept as a piece of its own"); pieces[f"{rn}~{i}"] = c; continue
        pieces[bk] = pieces[bk] + c
        log(f"  fold {rn}{i} ({c.volume() / 1e3:.1f} cm3) into {bk} over {best:.0f} mm2")
    return pieces


def best_offset(X, mk_plane, nominal, window, step=2.0):
    """plane offset along its normal with the smallest section of X (slab volume / 2 mm) inside nominal +- window."""
    best = None
    for c in np.arange(nominal - window, nominal + window + 1e-6, step):
        pl = mk_plane(c)
        a = (pl.loc(X) ^ box(-BIG, BIG, -BIG, BIG, -1.0, 1.0)).volume() / 2.0
        if best is None or a < best[0] - 1e-6 or (abs(a - best[0]) < 1e-6 and abs(c - nominal) < abs(best[1] - nominal)): best = (a, c)
    return best[1], best[0]


AX = {"x": ([1, 0, 0], [0, 1, 0]), "y": ([0, 1, 0], [1, 0, 0]), "z": ([0, 0, 1], [1, 0, 0])}
# size splits (the 180 cube): piece, plane name, axis, nominal offset, window, (low piece, high piece)
SPLITS = [("mast", "Z mast 2", "z", 66.0, 12.0, ("mast_12", "mast_3")),             # the stub plate Z -45..45 stays in one piece (mast_2)
          ("mast_12", "Z mast 1", "z", -80.0, 12.0, ("mast_1", "mast_2")),
          ("rearP", "Z rearP", "z", 28.0, 14.0, ("rearP_1", "rearP_2")),
          ("rearN", "Z rearN", "z", -48.0, 14.0, ("rearN_1", "rearN_2")),
          ("topP", "Y topP", "y", 138.0, 14.0, ("topP_1", "topP_2")),
          ("topN", "Y topN", "y", 138.0, 14.0, ("topN_1", "topN_2")),
          ("bay0", "Y bay0", "y", 128.0, 20.0, ("bay0_a", "bay0_b")),
          ("bay-60", "Y bay-60", "y", 128.0, 20.0, ("bay-60_a", "bay-60_b")),
          ("bay-120", "Y bay-120", "y", 128.0, 20.0, ("bay-120_a", "bay-120_b")),
          ("bay-180", "Y bay-180", "y", 128.0, 20.0, ("bay-180_a", "bay-180_b"))]


def size_splits(pieces, planes, log=print):
    for src, name, ax, nom, win, (lo, hi) in SPLITS:
        if src not in pieces: log(f"  split {src}: no such piece"); continue
        n, u = AX[ax]; ia = "xyz".index(ax)
        mk = lambda c, n=n, u=u, ia=ia, name=name: Plane(name, n, u, [c if k == ia else 0.0 for k in range(3)])
        c, area = best_offset(pieces[src], mk, nom, win)
        pl = mk(c); planes.append(pl)
        X = pieces.pop(src)
        pieces[lo] = X ^ pl.lbox(-BIG, BIG, -BIG, BIG, -BIG, 0.0); pieces[hi] = X ^ pl.lbox(-BIG, BIG, -BIG, BIG, 0.0, BIG)
        log(f"  split {src} on {name} at {ax.upper()} {c:.0f} (nominal {nom:.0f}, section {area / 100:.1f} cm2)")
        split_islands(pieces, (lo, hi), planes, pl, log)
    return pieces


def split_islands(pieces, names, planes, new_plane, log, vfold=12000.0, vdrop=2000.0):
    """after a size split: the components of each half other than its largest. Over vfold mm3 a piece of its own; else they
    join the piece they touch over the largest face on any plane but the new one; with no such neighbour they are dropped
    under vdrop mm3 (specks of a member that only hung across the plane) and kept as a piece above."""
    for nm in names:
        cs = comps(pieces[nm], 50.0)
        pieces[nm] = cs[0]
        for i, c in enumerate(cs[1:], 1):
            if c.volume() >= vfold: pieces[f"{nm}_{i}"] = c; continue
            best, bk = 0.0, None
            for pl in planes:
                if pl is new_plane or not near_plane(c, pl): continue
                for k, m in pieces.items():
                    if not near_plane(m, pl): continue
                    ar = contact_area(c, m, pl)
                    if ar > best: best, bk = ar, k
            if bk is not None and best >= 40.0:
                pieces[bk] = pieces[bk] + c; log(f"    island {nm}{i} ({c.volume() / 1e3:.1f} cm3) into {bk} over {best:.0f} mm2")
            elif c.volume() < vdrop: log(f"    island {nm}{i} ({c.volume() / 1e3:.2f} cm3) dropped (no neighbour)")
            else: pieces[f"{nm}_{i}"] = c; log(f"    island {nm}{i} ({c.volume() / 1e3:.1f} cm3) kept as a piece")


# ================================================================================================ drawn features
FOOT = dict(d=20.0, h=5.0, z0=-236.0)             # TPU foot Ø20 x 5 (E) on the floor plane Z -236
ZB = FOOT["z0"] + FOOT["h"]                       # structure bottom with feet: -231
FPAD = dict(d=16.0, h=6.0, ins_d=4.0, ins_h=6.7)  # foot pad Ø16 x 6 under the patch, M3 heat-set insert (foot screw M3 from below)


def feature_specs():
    """insert axes of the receiver bolts per bay, seat-post axes, foot sites: (point W, unit inward axis W, kind, owner label)."""
    import box_system_topo_v2r2 as R2
    T = R2.T
    sp = []
    for ang, pk in R2.L2.BAYS.items():
        g = R2.META[pk]; f = T.bay_pose(ang)
        for x, zc in R2.BOLTS[pk]:
            # zc is the bolt's z in the cartridge frame (from the closed end), bay_pose maps Y = Y_C - (z - Z_T). Round 2's drawn bosses
            # used zc + Z_T (shifted by Z_T along Y, off the receiver): the cut draws its own bosses at these points.
            p0 = np.array([x, g["Y_M"], zc]); a = f(p0); b = f(p0 + np.array([0, 1.0, 0]))
            sp.append(dict(kind="receiver", bay=ang, point=a.tolist(), axis=(b - a).tolist(), x=x, zc=zc))
    D = drawn_inputs()
    J_ = R2.J
    for i, ld in enumerate(J_["landings"]):
        p = np.array(ld["p"]); n = np.array(ld["n"]); n /= np.linalg.norm(n)
        sp.append(dict(kind="seat", point=p.tolist(), axis=n.tolist(), idx=i))
    return sp


def cyl_at(p, d, r, length, back=0.0, n=32):
    """world cylinder radius r from p - back*d along unit d, length."""
    d = np.asarray(d, float); d = d / np.linalg.norm(d)
    z = np.array([0, 0, 1.0]); v = np.cross(z, d); c = float(z @ d)
    if np.linalg.norm(v) < 1e-9: Rm = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]); Rm = np.eye(3) + vx + vx @ vx / (1 + c)
    T = np.hstack([Rm, (np.asarray(p, float) - back * d)[:, None]])
    return mf.Manifold.cylinder(length, r, r, n).transform(T.tolist())


def foot_sites(S, amin=40.0):
    """floor patches of S in the layer above the foot plane: centroid and area of each patch (Z ZB .. ZB + 4)."""
    sl = S ^ box(-BIG, BIG, -BIG, BIG, ZB, ZB + 4.0)
    out = []
    for c in comps(sl, 0.0):
        pr = c.project(); ar = pr.area()
        if ar < amin: continue
        bb = bbox(c); ctr = ((bb[0] + bb[1]) / 2)[:2]
        # area centroid of the footprint
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        g = unary_union([Polygon(r_).buffer(0) for r_ in pr.to_polygons()])
        out.append(dict(xy=[float(g.centroid.x), float(g.centroid.y)], area=float(ar), bounds=[float(v) for v in g.bounds]))
    return out


def add_features(S, log=print):
    """foot trim + pads, receiver insert holes, seat-post insert holes. Returns S and PROT (hole + wall volumes the laps keep off)."""
    t0 = time.time()
    S = S ^ box(-BIG, BIG, -BIG, BIG, ZB, BIG)
    feet = foot_sites(S)
    holes, prot, info = [], [], dict(feet=feet, receiver=[], seat=[])
    pads = []
    for f in feet:
        x, y = f["xy"]
        pads.append(mf.Manifold.cylinder(FPAD["h"], FPAD["d"] / 2, FPAD["d"] / 2, 48).translate([x, y, ZB]))
        holes.append(cyl_at([x, y, ZB - 0.5], [0, 0, 1], FPAD["ins_d"] / 2, FPAD["ins_h"] + 0.5)); prot.append(cyl_at([x, y, ZB], [0, 0, 1], 5.5, 9.0))
    D = drawn_inputs(); KEEP = D["cuts"] - D["drawn"]
    S = S + (union(pads) - KEEP)
    bosses = []
    for sp in feature_specs():
        p = np.array(sp["point"]); d = np.array(sp["axis"])
        if sp["kind"] == "receiver":
            bosses.append(cyl_at(p, d, 6.0, 11.0, back=0.0))                    # O12 boss on the mount pad, 11 high: 6.7 insert + 4.3 floor
            h_ = cyl_at(p, d, 2.0, 6.7 + 0.5, back=0.5); pr = cyl_at(p, d, 6.0, 11.0, back=0.0)
        else:
            h_ = cyl_at(p, d, 2.0, 6.7 + 0.5, back=0.5); pr = cyl_at(p, d, 5.0, 8.0, back=0.0)
        holes.append(h_); prot.append(pr)
    S = S + (union(bosses) - KEEP)
    S = S - union(holes)
    PROT = union(prot)
    log(f"features: {len(feet)} foot pads, {len(holes) - len(feet)} insert holes ({time.time() - t0:.0f} s)")
    return S, PROT, info


# ================================================================================================ bolted laps
# one lap site = a lug of one piece lying on a recess of the other, over the cut plane (drawn, never generated):
#   lug: width 2 hw, thickness Lt (in the member's outer layer), runs from 8 inside its own piece to ov over the other piece's
#   face (overlap ov >= 12, pads >= O12 with >= 2.0 web: hw = 8 for M3); counterbore, clearance hole through the lug;
#   insert boss under the lug (receiving piece) with the heat-set hole, wall >= 3.
BOLT = {"M3": dict(Lt=6.0, hw=8.0, ov=14.0, root=7.0, hole=3.4, cb_d=6.2, cb_h=3.2, ins_d=4.0, ins_h=6.7, boss=9.5, screw="M3 x 10", insert="M3 x 5.7",
                   cap_N=250.0, shear_N=450.0),
        "M5": dict(Lt=8.0, hw=10.0, ov=18.0, root=9.0, hole=5.5, cb_d=9.4, cb_h=5.3, ins_d=6.4, ins_h=10.5, boss=13.0, screw="M5 x 16", insert="M5 x 9.5",
                   cap_N=650.0, shear_N=1100.0)}      # per-bolt capacities, E: insert pull-out in PETG (tension) and lug bearing / shear, at SF 1


def lcyl(pl, r, p, d, length, n=32):
    """cylinder radius r, from local point p along local unit vector d, length (world manifold)."""
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    z = np.array([0, 0, 1.0]); v = np.cross(z, d); c = float(z @ d)
    if np.linalg.norm(v) < 1e-9: Rm = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]); Rm = np.eye(3) + vx + vx @ vx / (1 + c)
    T = np.hstack([Rm, np.asarray(p, float)[:, None]])
    return pl.world(mf.Manifold.cylinder(length, r, r, n).transform(T.tolist()))


def poly_of(P):
    """shapely geometry of a CrossSection (rings with their orientation: outer positive, holes negative)."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    outs, holes = [], []
    for r in P.to_polygons():
        pg = Polygon(r)
        if not pg.is_valid: pg = pg.buffer(0)
        (outs if Polygon(r).exterior.is_ccw else holes).append(pg)
    g = unary_union(outs)
    if holes: g = g.difference(unary_union(holes))
    return g


def candidates(P, hw, depth, step=3.0, min_fill=None):
    """boundary sites of a contact footprint (plane-local u, v): q the boundary point, theta the outward direction of the
    material boundary there, fill the share of the lug + boss rectangle (2 hw x depth, inward) inside the contact footprint."""
    from shapely.geometry import Polygon, MultiPolygon
    from shapely.geometry.polygon import orient
    g = poly_of(P); out = []
    if min_fill is None: min_fill = 0.55 if g.area > 500.0 else 0.12
    polys = list(g.geoms) if isinstance(g, MultiPolygon) else [g]
    for pg in polys:
        if pg.is_empty or pg.area < 8: continue
        pg = orient(pg, 1.0)
        for ring in [pg.exterior] + list(pg.interiors):
            L = ring.length
            for s in np.arange(0.5 * step, L, step):
                p = np.array(ring.interpolate(s).coords[0]); p1 = np.array(ring.interpolate(min(s + 2.5, L)).coords[0]); p0 = np.array(ring.interpolate(max(s - 2.5, 0)).coords[0])
                t = p1 - p0
                if np.linalg.norm(t) < 1e-6: continue
                t /= np.linalg.norm(t); nu = np.array([t[1], -t[0]])                      # outward from the material = right of travel
                th = math.atan2(nu[1], nu[0]); R2 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
                rect = Polygon([p + R2 @ np.array(c_) for c_ in ((0, -hw), (0, hw), (-depth, hw), (-depth, -hw))])
                fill = g.intersection(rect).area / rect.area
                if fill >= min_fill: out.append(dict(q=p, theta=th, fill=float(fill)))
    return out


def site_T(c):
    """site frame (x' outward from the member, y' across, z' = plane normal) -> plane-local."""
    th = c["theta"]; q = c["q"]
    return np.array([[math.cos(th), -math.sin(th), 0, q[0]], [math.sin(th), math.cos(th), 0, q[1]], [0, 0, 1, 0]])


def n_sites(area):
    if area < 150.0: return 1
    if area < 600.0: return 2
    return int(np.clip(round(math.sqrt(area) / 14.0), 3, 6))


def site_boxes(pl, c, owner_is_a, spec):
    """lug, boss (world manifolds) and bolt point / inward axis (site coordinates mapped to plane-local)."""
    Lt, hw, ov, bs, rt = spec["Lt"], spec["hw"], spec["ov"], spec["boss"], spec["root"]
    T = site_T(c)
    n_lug = (-rt, ov) if owner_is_a else (-ov, rt)
    n_bs = (0.0, ov) if owner_is_a else (-ov, 0.0)
    mk = lambda tr, nr: pl.world(box(tr[0], tr[1], -hw, hw, nr[0], nr[1]).transform(T.tolist()))
    nb = ov / 2.0 if owner_is_a else -ov / 2.0
    pt = T[:, :3] @ np.array([0.0, 0.0, nb]) + T[:, 3]; tdir = T[:, :3] @ np.array([-1.0, 0, 0])
    return mk((-Lt, 0.0), n_lug), mk((-Lt - bs, -Lt), n_bs), pt, tdir


def apply_site(pieces, KEEP, itf, c, owner_is_a, spec, log=print, level=0, PROT=None):
    pl = itf["plane"]; a, b = itf["a"], itf["b"]
    O, Rr = (a, b) if owner_is_a else (b, a)
    lug, boss, pt, tdir = site_boxes(pl, c, owner_is_a, spec)
    Lt = spec["Lt"]
    lug_c = lug - KEEP; boss_c = boss - KEEP
    if PROT is not None and ((lug ^ PROT).volume() > 3.0 or (boss ^ PROT).volume() > 3.0): return None, "insert hole in the way"
    if lug_c.volume() < 0.97 * lug.volume() or boss_c.volume() < 0.97 * boss.volume(): return None, "keep-out clips the pad"
    lb = bbox(lug); bb = bbox(boss)
    tol_o, tol_a = (2.0, 3.0) if level == 0 else (60.0, 60.0)
    trim = {}
    for k, m in pieces.items():                                          # a third piece in the way (level 1: it gives the material up)
        if k in (a, b): continue
        mb = bbox(m)
        for blk, bx in ((lug_c, lb), (boss_c, bb)):
            if (mb[0] < bx[1] + 0.1).all() and (bx[0] < mb[1] + 0.1).all():
                v = (blk ^ m).volume()
                if v > tol_o: return None, f"piece {k} in the way"
                if v > 0.5: trim[k] = trim.get(k, m) - blk
    ptw = pl.to_world(pt); dw = pl.Tw[:, :3] @ tdir
    head = lcyl(pl, 5.0, pt - tdir * 0.3, -tdir, ACCESS)                 # screwdriver and head clear, outward
    if (head ^ KEEP).volume() > tol_a: return None, "screw access blocked by a keep-out"
    for k, m in pieces.items():
        if (head ^ m).volume() > tol_a: return None, f"screw access blocked by {k}"
    hole = lcyl(pl, spec["hole"] / 2, pt + (-tdir) * 1.0, tdir, Lt + 1.0)
    cbore = lcyl(pl, spec["cb_d"] / 2, pt + (-tdir) * 1.0, tdir, spec["cb_h"] + 1.0)
    ins = lcyl(pl, spec["ins_d"] / 2, pt + tdir * (Lt - 0.5), tdir, spec["ins_h"] + 0.5)      # from the lug underside in the receiving piece
    nO = (pieces[O] + lug_c) - hole - cbore
    nR = ((pieces[Rr] - lug) + boss_c) - ins
    for k, mnew, mold in ((O, nO, pieces[O]), (Rr, nR, pieces[Rr])):
        if len(comps(mnew, 50.0)) > len(comps(mold, 50.0)): return None, f"{k} would fall into two bodies"
        eo, en = np.ptp(bbox(mold), axis=0), np.ptp(bbox(mnew), axis=0)
        if (en > np.maximum(MAXEXT, eo) + 1e-6).any(): return None, f"{k} would pass the 180 cube"
    reg = box(*[v for i_ in range(3) for v in (min(lb[0][i_], bb[0][i_]) - 2.0, max(lb[1][i_], bb[1][i_]) + 2.0)])
    if ((nO ^ reg) ^ (nR ^ reg)).volume() > 3.0: return None, "pieces would overlap"
    for k, m in trim.items():
        if len(comps(m, 50.0)) > len(comps(pieces[k], 50.0)): return None, f"{k} would fall into two bodies"
    pieces.update(trim)
    pieces[O], pieces[Rr] = nO, nR
    return dict(owner=O, receiver=Rr, size=spec_name(spec), pos=np.round(ptw, 2).tolist(), axis_in=np.round(dw, 4).tolist(), plane=pl.name,
                lug_box=np.round(bbox(lug), 2).tolist(), boss_box=np.round(bbox(boss), 2).tolist(), screw=spec["screw"], insert=spec["insert"]), None


ACCESS = 10.0                                                            # free length outside the screw head, Ø10 (E: hex driver + head)
MAXEXT = 178.0


def spec_name(spec):
    return "M3" if spec["hole"] < 4 else "M5"


def laps(pieces, planes, KEEP, sizes=None, log=print, amin=60.0, PROT=None):
    """a bolted lap on every contact. sizes: {(a, b, plane): ('M3' | 'M5', n)} from the system FE (default M3, n_sites(area))."""
    C = [c for c in contacts(pieces, planes, amin) if c["area"] >= amin]
    joints = []; rej = {}
    for k, itf in enumerate(C):
        sz, nn = ((sizes or {}).get((itf["a"], itf["b"], itf["plane"].name)) or ("M3", n_sites(itf["area"])))
        spec = BOLT[sz]
        cand = candidates(itf["P"], spec["hw"], spec["Lt"] + spec["boss"])
        got = []; tried = 0; rj = {}
        for level in (0, 1):
          if level == 1 and len(got) >= min(nn, 2): break
          tried = 0
          for c in sorted(cand, key=lambda c: -c["fill"]):
            if len(got) >= nn or tried >= 60: break
            if any(np.linalg.norm(c["q"] - g[0]["q"]) < 26.0 for g in got): continue
            tried += 1
            owner_a = (len(got) % 2 == 0)
            r, why = apply_site(pieces, KEEP, itf, c, owner_a, spec, log, level, PROT)
            if r is None:
                r, why2 = apply_site(pieces, KEEP, itf, c, not owner_a, spec, log, level, PROT)
                if r is None:
                    key = re.sub(r"(blocked by|piece|falls?|would fall|in the way)\s*\S*", "", why2).strip() or why2
                    key = re.sub(r"^\S+ would", "piece would", why2) if "would" in why2 else re.sub(r"(piece|by) \S+", r"\1 X", why2)
                    rej[key] = rej.get(key, 0) + 1; rj[key] = rj.get(key, 0) + 1; continue
            r['relaxed'] = level; got.append((c, r))
        joints.append(dict(a=itf["a"], b=itf["b"], plane=itf["plane"].name, plane_n=itf["plane"].n.tolist(), plane_p0=itf["plane"].p0.tolist(),
                           area=itf["area"], want=nn, size=sz, sites=[r for _, r in got]))
        log(f"  lap {itf['a']} | {itf['b']} on {itf['plane'].name}: {itf['area'] / 100:.1f} cm2, {len(got)}/{nn} {sz} sites ({len(cand)} candidates){' rejected ' + str(rj) if len(got) < nn else ''}")
    log(f"  rejected candidates: {rej}")
    return joints


def rename(pieces):
    """readable final names (the tree names: region, split halves, islands)."""
    m = {"mast_1": "mast_low", "mast_2": "mast_mid", "mast_3": "mast_top", "mast_1_1": "mast_leg", "rearP_1": "rearP_low", "rearP_2": "rearP_up",
         "rearP_2_1": "rearP_up_b", "rearP~1": "epost", "rearN_1": "rearN_low", "rearN_2": "rearN_up", "rearN~1": "floorbar",
         "bay-60~1": "foot_leg_1", "bay-60~2": "foot_leg_2", "bay-120~1": "foot_leg_3", "bay-150~1": "foot_leg_3", "bay-30~1": "foot_leg_1", "bay-30~2": "foot_leg_2"}
    hubs = sorted([k for k in pieces if k == "hub" or k.startswith("hub~") or k.startswith("hub_")], key=lambda k: -pieces[k].volume())
    for i, k in enumerate(hubs): m[k] = f"link_{i + 1}"
    out = {}
    for k, v in pieces.items():
        n = m.get(k, k)
        if n in out: raise RuntimeError(f"name clash {n}")
        out[n] = v
    return out, {k: m.get(k, k) for k in pieces}


def clean(pieces, log=print, vmin=50.0):
    """one body per piece: the largest component, specks under vmin mm3 dropped (the booleans' slivers); what was dropped is logged."""
    for k in list(pieces):
        cs = comps(pieces[k], 0.0)
        drop = [c.volume() for c in cs[1:]]
        if drop and max(drop) >= vmin: log(f"  WARNING {k}: second body of {max(drop) / 1e3:.2f} cm3 dropped")
        pieces[k] = cs[0]
    return pieces


def finish_piece(m, h=0.5, r=1.2, faces=70000, log=print):
    """the cut's finish: 0.5 mm voxels, opening by r 1.2 (no feature under 2.4 mm before the surface is drawn, the gate's 1.8 plus the
    surface's +-0.25), the largest body, smooth iso-surface (Gaussian 0.7 voxel, level 0.5, Taubin 4), decimated. Holes, laps and
    the cut faces keep their place to +-0.25 mm; the sites in cut.json are the drawn positions (M)."""
    from topo.part import voxelise
    from scipy.ndimage import gaussian_filter
    t = tm(m); lo = np.floor(t.bounds[0] / h) * h - 3 * h; hi = np.ceil(t.bounds[1] / h) * h + 3 * h
    ax = [lo[k] + (np.arange(int(round((hi[k] - lo[k]) / h))) + 0.5) * h for k in range(3)]
    A = voxelise(t, *ax)
    v0 = A.sum() * h ** 3
    core = ndi.distance_transform_edt(A) * h > r
    A2 = ndi.distance_transform_edt(~core) * h <= r
    A2 &= A | ndi.binary_dilation(A, iterations=1)
    lab, n = ndi.label(A2, ndi.generate_binary_structure(3, 1))
    if n > 1:
        sz = np.bincount(lab.ravel()); sz[0] = 0; A2 = lab == int(np.argmax(sz))
    f = gaussian_filter(A2.astype(np.float32), 0.7)
    v, fc, _, _ = measure.marching_cubes(np.pad(f, 1), 0.5, spacing=(h, h, h))
    v = v + lo + 0.5 * h - h
    mm = trimesh.Trimesh(v, fc[:, ::-1], process=True)
    trimesh.smoothing.filter_taubin(mm, iterations=4)
    if len(mm.faces) > faces:
        try: mm = mm.simplify_quadric_decimation(face_count=faces)
        except Exception as ex: log(f"  decimation failed {ex}")
    out = man(mm)
    log(f"  finish: {v0 / 1e3:.1f} -> {out.volume() / 1e3:.1f} cm3, {len(mm.faces)} faces")
    return out


def overlaps(pieces):
    out = {}
    ks = list(pieces); bbs = {k: bbox(pieces[k]) for k in ks}
    for i, a in enumerate(ks):
        for b in ks[i + 1:]:
            A, B = bbs[a], bbs[b]
            if not ((A[0] < B[1]).all() and (B[0] < A[1]).all()): continue
            v = (pieces[a] ^ pieces[b]).volume()
            if v > 0.5: out[f"{a} ^ {b}"] = round(v, 1)
    return out


# the 4 sheets (cladding, bought; box_structure_v2.assembly.json): inner plane coordinate along the axis and the outward direction
SHEETS = {"sheet+X": (0, 206.0, 1.0), "sheet-X": (0, -206.0, -1.0), "sheetR": (1, -66.0, -1.0), "sheetF": (1, 335.0, 1.0)}
CLIP = dict(r=5.0, root=8.0, ins_d=4.0, ins_h=6.7, gap=0.2)


def add_clips(P, KEEP, n_per=5, log=print):
    """>= 4 (here 5) clip tabs per sheet on frame members: a Ø10 stalk from the member's surface to 0.2 short of the sheet's inner
    plane (8 into the member), M3 heat-set hole in its end (sheet screw M3 from outside, Ø3.4 in the sheet, E). Drawn tabs, not structure.
    Points: surface vertices facing the sheet, spread by farthest-point sampling over the sheet plane, on as many pieces as possible."""
    clips = {}
    for sn, (ax, c, sg) in SHEETS.items():
        cand = []
        for k, m in P.items():
            t = tm(m); vn = t.vertex_normals; v = t.vertices
            ok = (vn[:, ax] * sg >= 0.8) & (v[:, ax] * sg >= (c * sg) - 60.0) & (v[:, ax] * sg <= c * sg - 0.5 + 1e-6)
            for i in np.where(ok)[0][::7]: cand.append((k, v[i].copy()))
        if not cand: clips[sn] = []; log(f"  {sn}: no member within 60 mm"); continue
        other = [a for a in range(3) if a != ax]
        pts = np.array([p[other] for _, p in cand])
        sel = []
        order = np.argsort(-pts[:, 0] * 0 + np.array([(c - p[ax]) * sg for _, p in cand]))             # nearest to the sheet first
        first = int(order[0]); sel = [first]
        d = np.linalg.norm(pts - pts[first], axis=1)
        while len(sel) < n_per * 4 and len(sel) < len(cand):
            i = int(np.argmax(d)); sel.append(i); d = np.minimum(d, np.linalg.norm(pts - pts[i], axis=1))
        done = []
        for i in sel:
            k, p = cand[i]
            n = np.zeros(3); n[ax] = sg
            L = (c - CLIP["gap"] - p[ax]) * sg + CLIP["root"]
            stalk = cyl_at(p, n, CLIP["r"], L, back=CLIP["root"]) - KEEP
            if stalk.volume() < 0.97 * math.pi * CLIP["r"] ** 2 * L: continue
            bad = False
            for kk, m in P.items():
                if kk == k: continue
                bb_ = bbox(m); sb = bbox(stalk)
                if (bb_[0] < sb[1]).all() and (sb[0] < bb_[1]).all() and (stalk ^ m).volume() > 3.0: bad = True; break
            if bad: continue
            tip = p + n * (L - CLIP["root"]); hole = cyl_at(tip, -n, CLIP["ins_d"] / 2, CLIP["ins_h"] + 0.5, back=-0.5)
            P[k] = (P[k] + stalk) - hole
            done.append(dict(piece=k, at=np.round(p, 1).tolist(), tip=np.round(tip, 1).tolist(), axis_in=np.round(-n, 3).tolist(), stalk_len=round(float(L - CLIP["root"]), 1)))
            if len(done) >= n_per: break
        clips[sn] = done
        log(f"  clips {sn}: {len(done)} on {sorted(set(d_['piece'] for d_ in done))}")
    return clips


def build_cut(sizes=None, log=print, finish=True):
    S = structure(); S, PROT, finfo = add_features(S, log=log)
    out, pl = partition(S)
    P = fold(out, pl, log=log); P = size_splits(P, pl, log=log)
    P, nm = rename(P)
    D = drawn_inputs(); KEEP = D["cuts"] - D["drawn"]
    t0 = time.time(); J = laps(P, pl, KEEP, sizes=sizes, log=log, PROT=PROT); log(f"laps {time.time() - t0:.0f} s")
    finfo["clips"] = add_clips(P, KEEP, log=log)
    P = clean(P, log)
    if finish:
        for k in list(P): P[k] = finish_piece(P[k], log=lambda s_: log(f"  {k}:{s_}"))
    return P, J, pl, finfo, PROT, KEEP


def feet_meshes(finfo):
    """TPU feet Ø20 x 5 under the pads (E), as manifolds in W."""
    out = {}
    for i, f in enumerate(finfo["feet"]):
        x, y = f["xy"]
        out[f"foot_{i + 1}"] = mf.Manifold.cylinder(FOOT["h"], FOOT["d"] / 2, FOOT["d"] / 2, 64).translate([x, y, FOOT["z0"]])
    return out


def export_cut(P, J, finfo, tag=""):
    pdir = os.path.join(OUT, "pieces"); os.makedirs(pdir, exist_ok=True)
    rows = {}
    for k, m in P.items():
        tm(m).export(os.path.join(pdir, f"{k}.stl"))
        b = bbox(m); rows[k] = dict(vol_cm3=round(m.volume() / 1e3, 2), mass_g=round(m.volume() * RHO, 1), lo=np.round(b[0], 1).tolist(), hi=np.round(b[1], 1).tolist())
    for k, m in feet_meshes(finfo).items():
        tm(m).export(os.path.join(pdir, f"{k}.stl"))
    json.dump(dict(pieces=rows, joints=J, feet=finfo["feet"], clips=finfo.get("clips", {})), open(os.path.join(OUT, f"cut{tag}.json"), "w"), indent=1, default=float)
    return rows


# ================================================================================================ system FE on the cut assembly (F)
LAY = os.path.join(ROOT, "runs", "box_system", "v2", "layout")
CTX = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")
G_ = 9.81e-3                                         # N per g (mass in g)
E_STIFF = dict(E=10000.0, nu=0.3)                    # receivers and cartridges as stiff parts (E)
ELEC_G, ARM_G, FUN_G = 2900.0 * 0 + 2000.0, 2900.0, 640.0
DRONES = {"Scout": 520.0, "Overseer": 2330.0}
SF_CLS = {"sustained": 10.0, "flight": 4.0, "misuse": 2.5}
BOLT_ULT = {"M3": dict(T=300.0, V=600.0), "M5": dict(T=700.0, V=1300.0)}   # ultimate per bolt (E): heat-set pull-out in PETG, lug bearing
EPOS = [(33, -179), (166, -179), (33, 54), (166, 54), (-171, -209), (-35, -209), (-171, -20), (-35, -20)]      # electronics bosses (R2.elec_bosses)


def fe_bodies(P, finfo):
    """name -> (trimesh, material key, build axis, g/mm3): pieces (PETG, build axis = the shortest extent: largest flat face down E),
    TPU feet, receivers (PETG) and cartridges (stiff parts, density from the swap mass)."""
    import box_system_topo_v2r2 as R2
    from topo.material import MATERIALS
    bodies = {}
    for k, m in P.items():
        t = tm(m); bodies[k] = (t, "PETG", int(np.argmin(np.ptp(t.bounds, axis=0))), MATERIALS["PETG"].density * 1e-3)
    for k, m in feet_meshes(finfo).items(): bodies[k] = (tm(m), "TPU", 2, MATERIALS["TPU95A"].density * 1e-3)
    for ang, pk in R2.L2.BAYS.items():
        rc = trimesh.load(os.path.join(LAY, "parts", f"bay{ang}_receiver.stl"), force="mesh")
        ct = trimesh.load(os.path.join(LAY, "parts", f"bay{ang}_cartridge.stl"), force="mesh")
        bodies[f"rcv{ang}"] = (rc, "STIFF", 1, MATERIALS["PETG"].density * 1e-3)
        bodies[f"cart{ang}"] = (ct, "STIFF", 1, R2.MASS[pk] / max(abs(ct.volume), 1.0))
    return bodies


def bolt_zone(pos, axis, r, a0, a1):
    pos = np.asarray(pos); axis = np.asarray(axis)

    def f(X):
        d = X - pos; al = d @ axis; rad = np.linalg.norm(d - al[:, None] * axis[None], axis=1)
        return (rad <= r) & (al >= a0) & (al <= a1)
    return f


def joint_defs(J, P, mode="all", h=4.0):
    """joint zones for the FE. mode 'all': every cut face and lug face bonded (contact + clamp), 'bolts': only a disc O12 round each bolt
    at the lug face (bolt areas only, contact elsewhere ignored: the lower bound on stiffness)."""
    defs = []
    for j in J:
        if not j["sites"]: continue
        zs = []
        for s in j["sites"]:
            spec = BOLT[s["size"]]
            r = 6.0 if mode == "bolts" else spec["hw"] + 1.0
            zs.append(bolt_zone(s["pos"], s["axis_in"], r, -1.0 - spec["cb_h"], spec["Lt"] + (3.0 if mode == "bolts" else spec["boss"])))
        if mode == "all":
            n = np.array(j["plane_n"]); p0 = np.array(j["plane_p0"])
            zs.append(lambda X, n=n, p0=p0: np.abs((X - p0) @ n) <= 2.5)
        defs.append(dict(a=j["a"], b=j["b"], zone=lambda X, zs=zs: np.any([z(X) for z in zs], axis=0)))
    return defs


class CutModel(B.Model):
    """box_system.Model with declared joint zones, per-body density and stiffness."""

    def __init__(self, bodies, h, defs, log=print):
        self.defs = defs; self.dens = {k: v[3] for k, v in bodies.items()}
        super().__init__({k: v[:3] for k, v in bodies.items()}, h, log=log)

    def _joints(self):
        ns = self.ns; d = np.array([(a * ns[1] + b) * ns[2] + c for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)])
        from topo.fem import LOCAL
        L = np.array(LOCAL); ea, eb = [], []; self.pairs = {}
        for df in self.defs:
            a, b = df["a"], df["b"]
            if a not in self.cells or b not in self.cells: continue
            sa, sb = self.samp[a], self.samp[b]
            if len(sa) > len(sb): sa, sb = sb, sa
            dil = np.unique((sa[:, None] + d[None]).ravel())
            hit = sb[np.isin(sb, dil, assume_unique=False)]
            if len(hit) == 0: continue
            si = np.stack([hit // (ns[1] * ns[2]), (hit // ns[2]) % ns[1], hit % ns[2]], 1)
            xyz = B.GRID_O + (si + 0.5) * self.hs
            si = si[df["zone"](xyz)]
            if len(si) == 0: continue
            g = np.unique(self.gnode((si // 3)[:, None, :] + L[None]).ravel())
            pa, pb = self.pid(a, g), self.pid(b, g); ok = (pa >= 0) & (pb >= 0)
            if ok.sum() == 0: continue
            ea.append(pa[ok]); eb.append(pb[ok]); self.pairs[(a, b)] = g[ok]
        from scipy.sparse.csgraph import connected_components
        ea = np.concatenate(ea); eb = np.concatenate(eb)
        Gm = sp.coo_matrix((np.ones(len(ea)), (ea, eb)), shape=(self.nP, self.nP))
        self.nN, self.comp = connected_components(Gm, directed=False)
        self.pg = np.empty(self.nP, np.int64)
        for k in self.names: self.pg[self.poff[k]: self.poff[k] + len(self.pnodes[k])] = self.pnodes[k]

    def Cmat(self, k):
        from topo.material import MATERIALS, stiffness
        m = self.mat[k]
        if m == "STIFF": return B.iso_C(E_STIFF["E"], E_STIFF["nu"])
        if m == "TPU": return B.iso_C(MATERIALS["TPU95A"].E, MATERIALS["TPU95A"].nu)
        return stiffness(MATERIALS["PETG"], self.axis[k])

    def masses(self, k):
        return self.frac[k] * self.h ** 3 * self.dens[k]


def cut_cases():
    """round 2's 18 cases (box_system_topo_v2r2._cases) as loads on named regions; the pitch-guide couple is not in them (none in r2)."""
    import box_system_topo_v2r2 as R2
    L2 = R2.L2
    Y_C, Y_FUN = R2.Y_C, R2.Y_FUN
    E = lambda m_g, v=0.15, drop=0.030: (0.5 * m_g * 1e-3 * v * v + m_g * 1e-3 * 9.81 * drop) * 1000 / 3.0
    down = (0, 0, -1); up = np.array([0, 0, 1.0])
    PL = (60.0, Y_FUN + 30.0, 175.0); PLm = (-60.0, Y_FUN - 30.0, 175.0)
    Fc, Fm = E(DRONES["Overseer"]), E(DRONES["Overseer"], 0.15, 0.040)
    tip = lambda ang, r=140.0: np.array([r * math.cos(math.radians(ang)), Y_C, r * math.sin(math.radians(ang))])
    P0 = np.array([0.0, Y_C, 0.0])
    C = {}

    def add(name, cls, acc, drone, loads, support="floor"): C[name] = dict(cls=cls, acc=acc, drone=drone, loads=loads, support=support)
    add("standing, Overseer seated", "sustained", down, "Overseer", [])
    add("Overseer catch +X+Y diag", "flight", down, "Overseer", [("posts", (-0.25 * Fc, -0.25 * Fc, -Fc), None, PL)])
    add("Overseer catch -X-Y diag", "flight", down, "Overseer", [("posts", (0.25 * Fc, 0.25 * Fc, -Fc), None, PLm)])
    add("misuse: Overseer motors cut", "misuse", down, "Overseer", [("posts", (-0.25 * Fm, -0.25 * Fm, -Fm), None, PL)])
    add("XT60 insert at the drone 60 N", "flight", down, "Scout", [("posts", 60 * up, None, tip(90, 144.5)), ("stub", -60 * up, None, tip(90))])
    add("misuse: head stall at the drone 74 N", "misuse", down, "Scout", [("posts", 74 * up, None, tip(90, 144.5)), ("stub", -74 * up, None, tip(90))])
    for ang in (-90, -120):
        pk = L2.BAYS[ang]; d = tip(ang, 1.0) - np.array([0, Y_C, 0])
        xp = trimesh.load(os.path.join(CTX, f"{pk}_xt60e1m_c.stl"), force="mesh").centroid
        import box_system_topo_v2 as T
        P_ = T.bay_pose(ang)(xp)
        add(f"XT60 insert at the {ang} bay 60 N", "flight", down, "Scout", [(f"rcv{ang}", 60 * d, None, P_), ("stub", -60 * d, None, tip(ang))])
        if ang == -90:
            add("XT60 pull at the -90 bay 50 N", "flight", down, "Scout", [(f"rcv{ang}", -50 * d, None, P_), ("stub", 50 * d, None, tip(ang))])
            add("misuse: head stall at the -90 bay 74 N", "misuse", down, "Scout", [(f"rcv{ang}", 74 * d, None, P_), ("stub", -74 * d, None, tip(ang))])
        tdir = np.array([-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))])
        F_ = 7.6 * tdir; M_ = np.array([0, 1050.0, 0]); Mr = -(M_ + np.cross(P_ - P0, F_))
        add(f"centring at the {ang} bay 7.6 N + 1.05 N.m", "flight", down, "Scout", [(f"rcv{ang}", F_, M_, P_), ("stub", -F_, Mr, P0)])
    add("stub tilt: XT60 at the drone 57 N + 5.9 N.m", "flight", down, "Scout",
        [("stub", (0, 0, -57.0), (5900.0, 0, 0), (0, Y_C, 0)), ("posts", (0, 0, 57.0), (-5900.0, 0, 0), (0, Y_C, 144.5))])
    for nm, mode in (("all bays: 60 N insert", "rad"), ("all bays: 43.5 N across", "tan")):
        loads = []; Ft = np.zeros(3); Mt = np.zeros(3)
        for ang in L2.BAYS:
            r_ = np.array([math.cos(math.radians(ang)), 0, math.sin(math.radians(ang))]); tdir = np.array([-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))])
            F_ = (60.0 * r_) if mode == "rad" else (43.5 * tdir)
            import box_system_topo_v2 as T
            cg = T.bay_pose(ang)(np.array(R2.META[L2.BAYS[ang]]["SWAP_CG"]))
            Pp = np.array([cg[0], Y_C, cg[2]])
            loads.append((f"rcv{ang}", F_, None, Pp)); Ft += F_; Mt += np.cross(Pp - P0, F_)
        loads.append(("stub", -Ft, -Mt, P0))
        add(nm, "flight", down, "Scout", loads, support="floor_hold")
    add("lifted from below 2 g", "flight", (0, 0, -2), None, [])
    add("misuse: set down hard 5 g", "misuse", (0, 0, -5), None, [])
    add("misuse: knocked over onto +X 5 g", "misuse", (5, 0, 0), None, [], support="side")
    return C


def sel_nodes(Mo, names, f):
    p = np.concatenate([np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])) for k in names]); X = Mo.xyz_p(p)
    return p[f(X)]


def fe_regions(Mo, P, specs):
    """named node sets: stub (the Ø90 stub plate Y -8..0), posts (the 8 seat discs), eboss (8 Ø14 bosses), rcv<ang> (receiver bodies)."""
    pieces = list(P)
    R = {}
    R["stub"] = sel_nodes(Mo, pieces, lambda X: (X[:, 1] >= -8.5) & (X[:, 1] <= 0.5) & (np.hypot(X[:, 0], X[:, 2]) <= 45.5))
    pf = []
    for sp_ in specs:
        if sp_["kind"] != "seat": continue
        p = np.array(sp_["point"]); n = np.array(sp_["axis"]); n = n / np.linalg.norm(n)
        pf.append(lambda X, p=p, n=n: ((X - p) @ n >= -0.5) & ((X - p) @ n <= 8.5) & (np.linalg.norm((X - p) - ((X - p) @ n)[:, None] * n[None], axis=1) <= 8.5))
    R["posts"] = sel_nodes(Mo, pieces, lambda X: np.any([f(X) for f in pf], axis=0))
    ef = [(lambda X, x=x, z=z: (X[:, 1] >= -66.5) & (X[:, 1] <= -57.5) & (np.hypot(X[:, 0] - x, X[:, 2] - max(z, -227.0)) <= 7.5)) for x, z in EPOS]
    R["eboss"] = sel_nodes(Mo, pieces, lambda X: np.any([f(X) for f in ef], axis=0))
    for k in Mo.names:
        if k.startswith("rcv"): R[k] = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k]))
    for k, v in R.items():
        if len(v) == 0: raise RuntimeError(f"region {k}: no nodes")
    return R


GRID_O2 = np.array([-212.0, -72.0, -236.0]); GRID_MAX2 = np.array([212.0, 340.0, 240.0])


def extra_defs(P, finfo, specs):
    """receiver <-> bay piece at its insert bolts, cartridge <-> receiver (all contact), foot <-> piece (all contact)."""
    import box_system_topo_v2r2 as R2
    defs = []
    bb = {k: bbox(m) for k, m in P.items()}
    near = lambda a, b, pad=5.0: (a[0] < b[1] + pad).all() and (b[0] < a[1] + pad).all()
    for ang in R2.L2.BAYS:
        rc = trimesh.load(os.path.join(LAY, "parts", f"bay{ang}_receiver.stl"), force="mesh"); rb = np.array(rc.bounds)
        zs = [bolt_zone(s["point"], s["axis"] / np.linalg.norm(s["axis"]), 6.0, -4.0, 10.0) for s in
              [dict(point=np.array(q["point"]), axis=np.array(q["axis"])) for q in specs if q["kind"] == "receiver" and q["bay"] == ang]]
        for k, b in bb.items():
            if near(rb, b): defs.append(dict(a=f"rcv{ang}", b=k, zone=lambda X, zs=zs: np.any([z(X) for z in zs], axis=0)))
        defs.append(dict(a=f"rcv{ang}", b=f"cart{ang}", zone=lambda X: np.ones(len(X), bool)))
    for k, m in feet_meshes(finfo).items():
        fb = bbox(m)
        for kk, b in bb.items():
            if near(fb, b, 1.0): defs.append(dict(a=k, b=kk, zone=lambda X: np.ones(len(X), bool)))
    return defs


def rigid_fit(Mo, U, p):
    X = Mo.xyz_p(p); u = U[Mo.comp[p]]; c0 = X.mean(0); r = X - c0
    A = np.zeros((3 * len(X), 6)); A[:, :3] = np.tile(np.eye(3), (len(X), 1))
    for i in range(3):
        e = np.zeros(3); e[i] = 1.0; A[:, 3 + i] = np.cross(e[None], r).ravel()
    x = np.linalg.lstsq(A, u.ravel(), rcond=None)[0]
    return x[:3], x[3:] * 1000.0                      # translation mm, rotation mrad


def group_forces(Xb, F, Mm, c):
    """forces on pins at Xb (n, 3) from the group's force F and moment Mm about c: F / n + w x r, w from the pseudo-inverse of the polar
    inertia with the near-singular directions dropped (a row of collinear bolts does not resist the moment about its own line: the lugs'
    faces and the cut face carry it, the bolts see only the shear and tension that the other directions give)."""
    r = Xb - c; n = len(Xb)
    Mc = Mm
    J = (r * r).sum() * np.eye(3) - r.T @ r
    ev, V = np.linalg.eigh(J)
    keep = ev > max(0.05 * ev.max(), 1.0)
    w = np.zeros(3)
    if keep.any(): w = V[:, keep] @ ((V[:, keep].T @ Mc) / ev[keep])
    return F[None] / n + np.cross(w[None], r)


def bolt_demands(sites, X, f):
    """joint forces f on the piece a at nodes X -> per-bolt tension / shear (N) on the lug owner's bolts: rigid pin group, force and moment about its centroid."""
    Xb = np.array([s["pos"] for s in sites]); F = f.sum(0); c = Xb.mean(0)
    Mm = np.cross(X - c, f).sum(0)
    fb = group_forces(Xb, F, Mm, c)
    out = []
    for s, fv in zip(sites, fb):
        u = np.array(s["axis_in"]); own_is_a = s["owner_is_a"]
        fo = fv if own_is_a else -fv                    # force on the lug owner from the receiving piece
        T_ = max(float(fo @ u), 0.0); V_ = float(np.linalg.norm(fo - (fo @ u) * u))
        out.append((T_, V_))
    return out


def run_cut_fe(P, J, finfo, h=4.0, mode="all", only=None, log=print, tag="", seed=None):
    B.GRID_O, B.GRID_MAX, B.FLOOR = GRID_O2, GRID_MAX2, -236.0
    specs = feature_specs()
    bodies = fe_bodies(P, finfo)
    defs = joint_defs(J, P, mode, h) + extra_defs(P, finfo, specs)
    Mo = CutModel(bodies, h, defs, log=log); Mo.assemble()
    R = fe_regions(Mo, list(P), specs)
    CS = cut_cases()
    if only: CS = {k: v for k, v in CS.items() if any(o in k for o in only)}
    jmap = {(j["a"], j["b"]): j for j in J if j["sites"]}
    pc = np.zeros(Mo.nP)
    for (a, b), g in Mo.pairs.items():
        pc[Mo.pid(a, g)] += 1; pc[Mo.pid(b, g)] += 1
    pieces = list(P)
    feet = [k for k in Mo.names if re.fullmatch(r"foot_\d+", k)]
    cases = {}
    jc = {ab: Mo.xyz_p(Mo.pid(ab[0], g)).mean(0) for ab, g in Mo.pairs.items()}
    for cname, c in CS.items():
        t0 = time.time()
        acc = np.array(c["acc"], float) * G_
        Fp = np.zeros((Mo.nP, 3))
        for k in Mo.names: np.add.at(Fp, Mo.en[k].ravel(), np.repeat(Mo.masses(k)[:, None] * acc[None] / 8.0, 8, 0))
        hot = np.zeros(Mo.nP, bool)
        def addf(rn, F, Pt, M=(0, 0, 0)):
            p = R[rn]; X = Mo.xyz_p(p); Fp[p] += B.wrench(X, F, M, Pt)
        addf("stub", ARM_G * acc, (0.0, 60.0, 0.0))
        import box_system_topo_v2r2 as R2
        addf("posts", FUN_G * acc, (0.0, R2.Y_FUN, 180.0))
        if c["drone"]: addf("posts", DRONES[c["drone"]] * acc, (0.0, R2.Y_FUN, 184.5))
        addf("eboss", ELEC_G * acc, (100.0, -50.0, -60.0))
        for rn, F, M, Pt in c["loads"]:
            addf(rn, np.asarray(F, float), Pt, np.zeros(3) if M is None else np.asarray(M, float)); hot[R[rn]] = True
        # supports: unilateral floor on the feet's bottom nodes, or the +X face
        if c["support"] == "side":
            pn = sel_nodes(Mo, pieces, lambda X: np.ones(len(X), bool)); Xp = Mo.xyz_p(pn)
            fx = pn[Xp[:, 0] >= Xp[:, 0].max() - 1e-6]; nrm = np.array([-1.0, 0, 0])
        else:
            allp = np.arange(Mo.nP); Xa = Mo.xyz_p(allp)
            fx = np.array([q for k in feet for q in range(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k]))]); fx = fx[np.abs(Xa[fx, 2] + 236.0) < 1e-6]; nrm = np.array([0, 0, 1.0])
        if seed is not None and cname in seed: fx = np.intersect1d(fx, seed[cname])
        for it in range(10):
            U = Mo.solve(Fp, fx); fj = Mo.internal(U) - Fp
            if c["support"] == "floor_hold": break                                  # the all-bays envelope cases: feet held (bilateral), tension reported
            Rm = np.zeros((Mo.nN, 3)); np.add.at(Rm, Mo.comp, fj)
            rn_ = Rm[Mo.comp[fx]] @ nrm
            cm = np.zeros(Mo.nN); np.add.at(cm, Mo.comp[fx], 1)
            pull = rn_ / cm[Mo.comp[fx]] < -0.01
            if not pull.any() or (~pull).sum() < 6: break
            fx = fx[~pull]
        hot[fx] = True
        rows = {}
        for k in pieces:
            s = Mo.stress(k, U); idx = B.failure(s, "PETG", c["cls"], Mo.axis[k])
            ok = (Mo.frac[k] >= 0.3) & ~hot[Mo.en[k]].any(1)
            if ok.sum() == 0: ok = Mo.frac[k] >= 0
            iw = np.where(ok, idx, 0); e = int(np.argmax(iw))
            rows[k] = dict(worst=round(float(iw[e]), 3), at=np.round(B.GRID_O + (Mo.cells[k][e] + 0.5) * h, 0).tolist(), p99=round(float(np.quantile(idx[ok], 0.99)), 3))
        joints = {}
        for ab, g in Mo.pairs.items():
            pa = Mo.pid(ab[0], g); X = Mo.xyz_p(pa); f = fj[pa] / np.maximum(pc[pa], 1)[:, None]
            Fa = f.sum(0); Ma = np.cross(X - jc[ab], f).sum(0)
            jd = dict(F=np.round(Fa, 2).tolist(), M_Nmm=np.round(Ma, 0).tolist(), n=int(len(g)))
            if ab in jmap and jmap[ab]["sites"]:
                sites = [dict(s, owner_is_a=(s["owner"] == ab[0])) for s in jmap[ab]["sites"]]
                dem = bolt_demands(sites, X, f)
                sz = [s["size"] for s in sites]
                idxb = [SF_CLS[c["cls"]] * (T_ / BOLT_ULT[z]["T"] + V_ / BOLT_ULT[z]["V"]) for (T_, V_), z in zip(dem, sz)]
                jd.update(bolt_T=[round(d[0], 1) for d in dem], bolt_V=[round(d[1], 1) for d in dem], bolt_idx=[round(v, 3) for v in idxb], bolt_worst=round(max(idxb), 3))
            joints[f"{ab[0]} | {ab[1]}"] = jd
        # floor reactions: the support's reaction per merged node (K u - F summed over its piece nodes), shared over its fixed piece nodes
        reac = {}
        cm = np.zeros(Mo.nN); np.add.at(cm, Mo.comp[fx], 1)
        Rc = np.zeros((Mo.nN, 3)); np.add.at(Rc, Mo.comp, fj)
        sh = Rc[Mo.comp[fx]] / cm[Mo.comp[fx]][:, None]
        for k in feet:
            q = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); sel = np.isin(fx, q)
            reac[k] = round(float(sh[sel][:, 2].sum()), 1) if sel.any() else 0.0
        react_total = np.round(sh.sum(0), 1).tolist(); load_total = np.round(Fp.sum(0), 1).tolist()
        ext = {}
        for k in pieces:
            q = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); Xq = Mo.xyz_p(q); cq = Xq.mean(0)
            ext[k] = dict(F=np.round(Fp[q].sum(0), 2).tolist(), M_Nmm=np.round(np.cross(Xq - cq, Fp[q]).sum(0), 0).tolist(), c=np.round(cq, 1).tolist())
        trs, rot = rigid_fit(Mo, U, R["stub"])
        bayrot = {}
        for ang in R2.L2.BAYS:
            t_, r_ = rigid_fit(Mo, U, R[f"rcv{ang}"]); bayrot[ang] = round(float(r_[1] - rot[1]), 4)
        cases[cname] = dict(cls=c["cls"], rows=rows, joints=joints, foot_N=reac, uplift=[k for k, v in reac.items() if (v < 0.5 if c["support"] == "floor" else v < -0.5 if c["support"] == "floor_hold" else False)],
                            stub_trans_mm=np.round(trs, 4).tolist(), stub_rot_mrad=np.round(rot, 4).tolist(), bay_rot_rel_stub_mrad=bayrot,
                            ext=ext, reaction_sum_N=react_total, applied_sum_N=load_total, contact_iterations=it + 1, umax_mm=round(float(np.abs(U).max()), 3), feet_in_contact=[k for k in feet if reac[k] >= 0.5], support=c["support"],
                            sup_nodes=fx.tolist() if False else None)
        wk = max(rows, key=lambda k: rows[k]["worst"])
        log(f"{cname:50s} worst {rows[wk]['worst']:.2f} {wk}  stub rot {np.round(rot, 2).tolist()} mrad  feet {len(cases[cname]['feet_in_contact'])}  ({time.time() - t0:.0f} s)")
    import resource
    foot_owner = {a: b for (a, b) in Mo.pairs if re.fullmatch(r"foot_\d+", a)}
    res = dict(h=h, mode=mode, foot_owner=foot_owner, peak_rss_GB=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6, 2), cells=int(sum(len(c) for c in Mo.cells.values())), dofs=int(3 * Mo.nN), cases=cases,
               joint_nodes={f"{a} | {b}": int(len(g)) for (a, b), g in Mo.pairs.items()}, centroids={f"{a} | {b}": np.round(v, 1).tolist() for (a, b), v in jc.items()})
    return res, Mo


# ================================================================================================ sizing, assembly, requirements
def size_joints(fe, J, target=0.8, nmax=6):
    """bolt size and count per joint from the FE: the worst bolt index over the cases, scaled by the bolts there are; M3 up to nmax
    bolts, else M5 (about 2.2 x the capacity). -> {(a, b, plane): (size, n)} for laps(); log of what changed."""
    worst = {}
    for cn, c in fe["cases"].items():
        for jk, jd in c["joints"].items():
            if "bolt_worst" in jd: worst[jk] = max(worst.get(jk, 0.0), jd["bolt_worst"])
    sizes = {}; rows = []
    for j in J:
        if not j["sites"]: continue
        jk = f"{j['a']} | {j['b']}"; w = worst.get(jk, 0.0); n0 = len(j["sites"]); s0 = j["size"]
        cap = 1.0 if s0 == "M3" else 2.2
        need = w * n0 * cap                                          # index x bolts, in M3 units
        if need / target <= nmax: size, n = "M3", max(int(math.ceil(need / target)), min(2, n0), 1)
        else: size, n = "M5", min(max(int(math.ceil(need / (target * 2.2))), 2), nmax)
        sizes[(j["a"], j["b"], j["plane"])] = (size, n)
        rows.append(dict(joint=jk, plane=j["plane"], worst_index=round(w, 3), was=f"{n0} x {s0}", now=f"{n} x {size}", over=bool(size == "M5" and need / (target * 2.2) > nmax)))
    return sizes, rows


def mass_cg(P, finfo):
    rows, mt, mc = {}, 0.0, np.zeros(3)
    for k, m in P.items():
        t = tm(m); g = m.volume() * RHO; c = np.array(t.center_mass); rows[k] = round(g, 1); mt += g; mc += g * c
    for k, m in feet_meshes(finfo).items():
        g = m.volume() * 1.22e-3; c = np.array(tm(m).center_mass); rows[k] = round(g, 2); mt += g; mc += g * c
    return dict(rows_g=rows, structure_g=round(mt, 0), cg=np.round(mc / mt, 1).tolist())


def print_poses(P):
    """largest flat face down: for each of the 6 axis directions the bed contact (print_orient's definition, M)."""
    import print_orient as PO
    out = {}
    for k, m in P.items():
        t = tm(m); best = None
        for ax in range(3):
            for sg in (1, -1):
                R_ = PO.rot_to_down(ax, sg); tt = t.copy(); tt.apply_transform(R_)
                n = tt.face_normals; a = tt.area_faces; zc = tt.triangles_center[:, 2]; zmin = tt.bounds[0][2]
                bed = float(a[(n[:, 2] < -0.99) & (zc < zmin + 0.3)].sum())
                if best is None or bed > best[0]: best = (bed, f"{'+' if sg > 0 else '-'}{'XYZ'[ax]} face down", np.ptp(tt.bounds, axis=0))
        out[k] = dict(bed_mm2=round(best[0], 0), face=best[1], extents=np.round(best[2], 1).tolist(), fits_180=bool((best[2] <= 180.0).all()))
    return out


def write_cut_assembly(P, J, finfo, path):
    """runs/box_system/v2/cut/box_structure_v2_cut.assembly.json: the v2 layout system file (receivers, cartridges, arm envelopes, lanes,
    PSU, sheets as cage) with the bay body envelopes and the v1 mast replaced by the cut pieces and the catch by the main catch file."""
    import assembly
    old = json.load(open(os.path.join(HERE, "box_structure_v2.assembly.json")))
    rel = lambda p: os.path.relpath(p, ROOT)
    drop = lambda n: ("body envelope" in n) or n.startswith("mast v6") or n.startswith("catch (panels v7")
    parts = [p for p in old["parts"] if not drop(p["name"])]
    ca = assembly.load(os.path.join(HERE, "box_catch.assembly.json"))
    catch_names = []
    for p in ca.parts:
        if p["name"].startswith("drone"): continue
        q = dict(p); q["name"] = "catch: " + p["name"]
        if "mesh" in q and not os.path.isabs(q["mesh"]):
            cand = [os.path.join(ROOT, q["mesh"]), os.path.join(HERE, q["mesh"])]
            q["mesh"] = rel(next(c for c in cand if os.path.exists(c)))
        q["owner"] = "box catch"; parts.append(q); catch_names.append(q["name"])
    for k, m in P.items():
        parts.append(dict(name=k, role="structure", mesh=rel(os.path.join(OUT, "pieces", f"{k}.stl")), pose=np.eye(4).tolist(), material="PETG", tag="M",
                          owner="box structure", version="v2 cut", source="nox/box/box_system_v2_cut.py", note="cut piece, a candidate for its own topology run"))
    for k in feet_meshes(finfo):
        parts.append(dict(name=k, role="structure", mesh=rel(os.path.join(OUT, "pieces", f"{k}.stl")), pose=np.eye(4).tolist(), material="TPU95A", tag="E",
                          owner="box structure", version="v2 cut", source="nox/box/box_system_v2_cut.py", note="TPU foot O20 x 5 (E)"))
    names = {p["name"] for p in parts}
    pairs = [pr for pr in old["contact_pairs"] if pr[0] in names and pr[1] in names]
    for j in J:
        if j["sites"]: pairs.append([j["a"], j["b"], 5.0])                      # bolted laps: the tab sits on its recess
    mans = {k: m for k, m in P.items()}
    for ang in BAY_ANG:
        rc = man(trimesh.load(os.path.join(LAY, "parts", f"bay{ang}_receiver.stl"), force="mesh"))
        for k, m in mans.items():
            v = (rc ^ m).volume() if (np.array(bbox(rc)[0]) < bbox(m)[1]).all() and (np.array(bbox(m)[0]) < bbox(rc)[1]).all() else 0.0
            if v > 0.0: pairs.append([f"bay {ang} receiver", k, max(5.0, round(v * 1.5, 1))])
    ct = [(n, man(trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh"))) for n, p in ((p["name"], p) for p in parts if p["name"].startswith("catch: ") and "mesh" in p)]
    for n, cm in ct:
        for k, m in mans.items():
            if (np.array(bbox(cm)[0]) < bbox(m)[1]).all() and (np.array(bbox(m)[0]) < bbox(cm)[1]).all():
                v = (cm ^ m).volume()
                if 0 < v < 1500.0: pairs.append([n, k, round(v * 1.5 + 1, 1)])                       # the seat landing (drawn wins, M)
    acc = []
    for a_ in old["access"]:
        if "body out" in a_["name"]: continue
        a2 = dict(a_); a2["ignore"] = [x for x in a_.get("ignore", []) if x in names]; acc.append(a2)
    A = assembly.new("Box structure v2 cut", "box", parts=parts, fit_tol_mm3=0.5, contact_pairs=pairs, access=acc, probes=False, note=(
        "Box structure v2 cut (24 Oct, box_system_v2_cut.py): the round 2 g3000 field cut into the pieces in pieces/ (W), bolted tab laps, "
        "TPU feet, the 4 sheets as cage with 5 clip tabs each on frame members, receivers and cartridges of the v2 layout in pose, the catch from "
        "box_catch.assembly.json (panel v9 plain) on its 8 seat posts. Left out: the arm (its envelopes stay as keep-outs), bolts, the "
        "electronics boards (movable), the bay body service-door access (the bodies are in halves: not checked). Crash probes off (floor-standing box, DEFAULT)."))
    A.save(path)
    return A


def rear_zone(P, planes_y=(-62.0, -50.0, -30.0, -10.0)):
    """frame members in the rear zone (the boards sit between the rear sheet Y -66 and the frame): per piece and slice, the section's
    bounding boxes and area, M. The fixed electronics are the PSU, the charger and the sheet face strip Y -66..-50."""
    out = {}
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    for yv in planes_y:
        rows = []
        for k, m in P.items():
            s = (m ^ box(-BIG, BIG, yv - 0.3, yv + 0.3, -BIG, BIG))
            if s.num_vert() == 0: continue
            cs = s.rotate([-90, 0, 0]).project()
            for r_ in cs.to_polygons():
                pg = Polygon(r_)
                if pg.area < 20 or not pg.exterior.is_ccw: continue
                b = pg.bounds; rows.append(dict(piece=k, x=[round(b[0]), round(b[2])], z=[round(b[1]), round(b[3])], area_mm2=round(pg.area)))
        out[f"Y {yv:g}"] = rows
    return out


def exploded_views(P, finfo, out_prefix, k_exp=0.35):
    """exploded views of the pieces (one colour each) and the assembled iso: PNGs. Pieces decimated to 2500 faces for the plot."""
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    cm = plt.get_cmap("tab20")
    meshes = {}
    for k, m in P.items():
        t = tm(m)
        try: t = t.simplify_quadric_decimation(face_count=2500)
        except Exception: pass
        meshes[k] = t
    for k, m in feet_meshes(finfo).items(): meshes[k] = tm(m)
    cen = np.mean([t.centroid for t in meshes.values()], axis=0)
    names = list(meshes); paths = {}
    views = dict(iso=(22, -55), front=(0, -90), side=(0, 0), top=(90, -90), back=(22, 125))
    for exploded in (True, False):
        for vn, (el, az) in views.items():
            if not exploded and vn != "iso": continue
            fig = plt.figure(figsize=(9, 9)); ax = fig.add_subplot(111, projection="3d")
            for i, k in enumerate(names):
                t = meshes[k]; off = (t.centroid - cen) * k_exp if exploded else np.zeros(3)
                tri = t.vertices[t.faces] + off
                nrm = t.face_normals; col = np.array(cm(i % 20)[:3]); sh = 0.55 + 0.45 * np.clip(nrm @ np.array([0.3, -0.5, 0.8]), -1, 1)
                pc = Poly3DCollection(tri, linewidths=0.0); pc.set_facecolor(np.c_[np.clip(col[None] * sh[:, None], 0, 1), np.ones(len(sh))]); pc.set_edgecolor("none")
                ax.add_collection3d(pc)
            lim = [(-260, 260), (-110, 400), (-300, 280)] if exploded else [(-215, 215), (-70, 340), (-236, 234)]
            ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2]); ax.set_box_aspect([lim[k][1] - lim[k][0] for k in range(3)])
            ax.view_init(el, az); ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.set_title(("exploded " if exploded else "assembled ") + vn)
            fig.tight_layout(); pth = out_prefix + f"_{'exp' if exploded else 'asm'}_{vn}.png"; fig.savefig(pth, dpi=100); plt.close(fig); paths[("exp" if exploded else "asm", vn)] = pth
    return paths


TARGETS = dict(flight=0.5, misuse=1.0, sustained=1.0)
AIM = 0.8
CASE_CLS = lambda cn, fe: fe["cases"][cn]["cls"]


def case_tables(fe):
    """per piece: worst index per class and overall; per case summaries (stub tilt, bay rotation, uplift)."""
    pieces = sorted({k for c in fe["cases"].values() for k in c["rows"]})
    tab = {}
    for k in pieces:
        row = dict(by_case={cn: c["rows"][k]["worst"] for cn, c in fe["cases"].items()}, by_class={})
        for cn, c in fe["cases"].items():
            cl = c["cls"]; row["by_class"][cl] = max(row["by_class"].get(cl, 0.0), c["rows"][k]["worst"])
        wc = max(row["by_case"], key=row["by_case"].get)
        row["worst"], row["worst_case"] = row["by_case"][wc], wc
        row["over"] = {cl: v for cl, v in row["by_class"].items() if v > TARGETS[cl]}
        tab[k] = row
    return tab


def summary_numbers(fe):
    cs = fe["cases"]; out = {}
    tc = cs.get("stub tilt: XT60 at the drone 57 N + 5.9 N.m")
    if tc:
        r = tc["stub_rot_mrad"]; tilt = math.hypot(r[0], r[2]); out["stub_tilt_mrad"] = round(tilt, 3); out["stub_k_Nm_per_mrad"] = round(5.9 / max(tilt, 1e-9), 2)
    out["stub_rot_max_any_case_mrad"] = round(max(max(abs(v) for v in c["stub_rot_mrad"]) for c in cs.values()), 3)
    br = {cn: max(abs(v) for v in c["bay_rot_rel_stub_mrad"].values()) for cn, c in cs.items()}
    cn = max(br, key=br.get); out["bay_rot_worst_mrad"] = round(br[cn], 3); out["bay_rot_worst_case"] = cn
    out["bay_rot_by_case"] = {k: round(v, 3) for k, v in br.items()}
    out["uplift"] = {cn: c["uplift"] for cn, c in cs.items() if c["uplift"]}
    out["foot_N"] = {cn: c["foot_N"] for cn, c in cs.items()}
    return out


def write_requirements(P, J, finfo, fe, fe_b, tab, mc, pp, sc, out_dir):
    """req/<piece>.json and requirements.md: envelope, interfaces (bolts), joint wrenches per case, own worst index, stiffness to keep, mass."""
    os.makedirs(os.path.join(out_dir, "req"), exist_ok=True)
    jby = {}
    for j in J:
        if j["sites"]: jby.setdefault(j["a"], []).append(j); jby.setdefault(j["b"], []).append(j)
    sn = summary_numbers(fe)
    rows_md = ["| Piece | Qty | Envelope X / Y / Z (W, M) | Mass M | Worst index (case, class) | Over target | Joints (partner: n x size) | solid_check |", "|---|---|---|---|---|---|---|---|"]
    for k, m in P.items():
        b = bbox(m); t = tab.get(k, {})
        js = {}
        for j in jby.get(k, []):
            other = j["b"] if j["a"] == k else j["a"]
            jk = f"{j['a']} | {j['b']}"; sg = 1.0 if j["a"] == k else -1.0
            cs = {cn: dict(F_N=np.round(sg * np.array(c["joints"][jk]["F"]), 2).tolist(), M_Nmm=np.round(sg * np.array(c["joints"][jk]["M_Nmm"]), 0).tolist(),
                           bolt_idx_worst=c["joints"][jk].get("bolt_worst")) for cn, c in fe["cases"].items() if jk in c["joints"]}
            js[other] = dict(plane=j["plane"], plane_normal=j["plane_n"], area_cm2=round(j["area"] / 100, 2), centroid=fe["centroids"].get(jk), sites=j["sites"], cases=cs,
                             bolts=f"{len(j['sites'])} x {j['size']}")
        ext = {cn: c["ext"][k] for cn, c in fe["cases"].items() if k in c["ext"]}
        stiff = {}
        if k == "mast_mid" or any("mast" in k for _ in [0]):
            stiff["stub_tilt"] = "<= 10 mrad at 57 N + 5.9 N.m, k >= 0.59 N.m/mrad (arm round 3); this model: %s mrad" % sn.get("stub_tilt_mrad")
        if k.startswith("bay"): stiff["bay_rotation"] = "receiver mount about the axle relative to the stub <= 2.6 mrad (+-0.15 deg datum); this model, worst bay: %s mrad" % sn["bay_rot_worst_mrad"]
        r = dict(piece=k, version="v2 cut", file=rel_p(os.path.join(OUT, "pieces", f"{k}.stl")), frame="W (box-arm.md): X across, Y along the axle, Z up",
                 envelope=dict(lo=np.round(b[0], 1).tolist(), hi=np.round(b[1], 1).tolist(), extents=np.round(b[1] - b[0], 1).tolist(), print_pose=pp.get(k)),
                 mass_g_M=mc["rows_g"].get(k), qty=1, joints=js, external_by_case=ext, own_worst_index=dict(worst=t.get("worst"), case=t.get("worst_case"), by_class=t.get("by_class"),
                 targets=TARGETS, aim=AIM), stiffness_to_keep=stiff, solid_check=sc.get(k, {}).get("ok"),
                 note="forces / moments act ON this piece (N, N.mm about the joint centroid, W); index = the system FE worst cell index at the case class (F, h %g)" % fe["h"])
        json.dump(r, open(os.path.join(out_dir, "req", f"{k}.json"), "w"), indent=0, default=float)
        wj = "; ".join(f"{o}: {len(v['sites'])} x {v['sites'][0]['size']}" for o, v in js.items())
        over = ", ".join(f"{c} {v:.2f}" for c, v in t.get("over", {}).items()) or "-"
        rows_md.append(f"| {k} | 1 | {' x '.join(f'{e:.0f}' for e in b[1] - b[0])} | {mc['rows_g'].get(k)} g | {t.get('worst', float('nan')):.2f} ({t.get('worst_case')}) | {over} | {wj} | {'PASS' if sc.get(k, {}).get('ok') else 'FAIL' if k in sc else 'not run'} |")
    open(os.path.join(out_dir, "requirements.md"), "w").write("\n".join(rows_md) + "\n")


def rel_p(p):
    return os.path.relpath(p, ROOT)


def make_page(P, finfo, tab, mc, pp, sc, sn, imgs, path, title="Box structure v2: the cut and the system FE"):
    import base64, html
    b64 = lambda f: base64.b64encode(open(f, "rb").read()).decode()
    figs = "".join(f'<figure><img alt="{n}" src="data:image/png;base64,{b64(f)}"><figcaption>{n}</figcaption></figure>' for n, f in
                   [("exploded, iso", imgs[("exp", "iso")]), ("exploded, front", imgs[("exp", "front")]), ("exploded, side", imgs[("exp", "side")]),
                    ("exploded, top", imgs[("exp", "top")]), ("exploded, from behind", imgs[("exp", "back")]), ("assembled, iso", imgs[("asm", "iso")])])
    rows = ""
    for k in sorted(tab, key=lambda q: -tab[q]["worst"]):
        t = tab[k]; flag = " class=bad" if t["over"] else ""
        rows += (f"<tr{flag}><td>{html.escape(k)}</td><td>{mc['rows_g'].get(k)}</td><td>{' x '.join(f'{e:.0f}' for e in pp[k]['extents'])}</td><td>{pp[k]['face']} {pp[k]['bed_mm2']:.0f}</td>"
                 f"<td>{t['by_class'].get('flight', 0):.2f}</td><td>{t['by_class'].get('misuse', 0):.2f}</td><td>{t['by_class'].get('sustained', 0):.2f}</td>"
                 f"<td>{t['worst']:.2f}</td><td>{html.escape(t['worst_case'])}</td><td>{'PASS' if sc.get(k, {}).get('ok') else ('FAIL' if k in sc else '-')}</td></tr>")
    page = f"""<title>{title}</title>
<style>
:root {{ --bg:#f4f6f7; --ink:#18232b; --mute:#5d6b75; --rule:#d3dadf; --bad:#a4462d }}
body {{ background:var(--bg); color:var(--ink); font-family:system-ui,sans-serif; line-height:1.45; margin:0; padding:20px }}
main {{ max-width:1200px; margin:0 auto }} h1 {{ font-size:1.5rem; margin:0 0 6px }} h2 {{ font-size:1.1rem; margin:20px 0 6px }}
.views {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(340px,1fr)); gap:12px }}
figure {{ margin:0; background:#fff; border:1px solid var(--rule); border-radius:6px; overflow:hidden }} figure img {{ width:100%; display:block }} figcaption {{ font-size:.8rem; color:var(--mute); padding:5px 8px }}
table {{ border-collapse:collapse; width:100%; font-size:.84rem; background:#fff }} th,td {{ text-align:left; padding:4px 8px; border-bottom:1px solid var(--rule) }}
tr.bad td {{ color:var(--bad) }} .wrap {{ overflow-x:auto }} p {{ max-width:90ch }}
</style>
<main><h1>{title}</h1>
<p>{len(P)} pieces, structure {mc['structure_g'] / 1000:.2f} kg (M, 1.27 g/cm3 PETG, TPU feet 1.22), CG {mc['cg']} (M, W). Stub tilt {sn.get('stub_tilt_mrad')} mrad at 57 N + 5.9 N.m (target 10); worst bay rotation about the axle against the stub {sn['bay_rot_worst_mrad']} mrad (target 2.6) (F, h 4). Colour = piece.</p>
<div class="views">{figs}</div>
<h2>Index per piece (system FE, worst cell, class targets 0.5 functional / 1.0 misuse and sustained, aim 0.8)</h2>
<div class="wrap"><table><tr><th>Piece</th><th>g (M)</th><th>Extents mm</th><th>Bed face, mm2</th><th>functional</th><th>misuse</th><th>sustained</th><th>worst</th><th>case</th><th>solid_check</th></tr>{rows}</table></div>
</main>"""
    open(path, "w").write(page)


def report_stage(h=4.0, tag="", t_start=None, peak_gb=None, log=print):
    """result.json, requirements, assembly file, visual page, summary.md from the saved cut and FE results."""
    import subprocess, resource
    P, J, finfo = load_cut()
    cutd = json.load(open(os.path.join(OUT, "cut.json")))
    fe = json.load(open(os.path.join(OUT, f"fe_h{h:g}_all{tag}.json")))
    fb_path = os.path.join(OUT, f"fe_h{h:g}_bolts{tag}.json")
    fb = json.load(open(fb_path)) if os.path.exists(fb_path) else None
    scj = json.load(open(os.path.join(OUT, "solid_check.json"))) if os.path.exists(os.path.join(OUT, "solid_check.json")) else {}
    tab = case_tables(fe); sn = summary_numbers(fe); mc = mass_cg(P, finfo); pp = print_poses(P)
    snb = summary_numbers(fb) if fb else None
    A = write_cut_assembly(P, J, finfo, os.path.join(OUT, "box_structure_v2_cut.assembly.json"))
    chk = A.check(); system_line = chk.get("summary")
    rz = rear_zone(P); json.dump(rz, open(os.path.join(OUT, "rear_zone.json"), "w"), indent=0)
    write_requirements(P, J, finfo, fe, fb, tab, mc, pp, scj, OUT)
    imgs = exploded_views(P, finfo, os.path.join(OUT, "view"))
    make_page(P, finfo, tab, mc, pp, scj, sn, imgs, os.path.join(OUT, "visual.html"))
    import shutil; shutil.copy(imgs[("exp", "iso")], os.path.join(OUT, "preview.png"))
    # box totals with the lumps (E) for the CG
    lumps = {"arm": (2900.0, (0.0, 60.0, 0.0)), "funnel + latch": (640.0, (0.0, 165.0, 180.0)), "electronics + PSU": (2000.0, (100.0, -50.0, -60.0))}
    import box_system_topo_v2r2 as R2
    import box_system_topo_v2 as T
    for ang, pk in R2.L2.BAYS.items(): lumps[f"cartridge {ang}"] = (R2.MASS[pk], tuple(T.bay_pose(ang)(np.array(R2.META[pk]["SWAP_CG"]))))
    tg = mc["structure_g"]; cg = np.array(mc["cg"]) * tg
    for g_, p_ in lumps.values(): tg += g_; cg = cg + g_ * np.array(p_)
    rec = dict(structure_g=mc["structure_g"], structure_cg=mc["cg"], box_with_lumps_g=round(tg), box_with_lumps_cg=np.round(cg / tg, 1).tolist())
    res = dict(h=h, pieces=len(P), mass_structure_g_M=mc["structure_g"], cg_structure_M=mc["cg"], mass_box_with_lumps_g_E=rec["box_with_lumps_g"], cg_box_with_lumps_E=rec["box_with_lumps_cg"],
               worst_index=max(tab.values(), key=lambda t: t["worst"])["worst"], worst_piece=max(tab, key=lambda k: tab[k]["worst"]),
               worst_case=tab[max(tab, key=lambda k: tab[k]["worst"])]["worst_case"], summary=sn, summary_bolts_only=snb,
               solid_check_pass=bool(scj and all(v["ok"] for v in scj.values()) and len(scj) >= len(P)), solid_check_n=len(scj), solid_check_pass_n=sum(1 for v in scj.values() if v["ok"]),
               system_line=system_line, peak_memory_GB=peak_gb, wall_time_s=None if t_start is None else round(time.time() - t_start))
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
    return dict(P=P, J=J, finfo=finfo, fe=fe, fb=fb, tab=tab, sn=sn, snb=snb, mc=mc, pp=pp, scj=scj, system_line=system_line, rz=rz, rec=rec, res=res, cutd=cutd)


def load_cut(tag=""):
    d = json.load(open(os.path.join(OUT, f"cut{tag}.json")))
    P = {k: man(trimesh.load(os.path.join(OUT, "pieces", f"{k}.stl"), force="mesh")) for k in d["pieces"]}
    return P, d["joints"], dict(feet=d["feet"])


def sha_of(p):
    import hashlib
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:12]


def solid_all(names=None, log=print):
    """nox/solid_check.py on every piece STL (one at a time: 6.5 GB each at pitch 0.25), output kept in solid/<piece>.txt."""
    import subprocess
    pdir = os.path.join(OUT, "pieces"); sdir = os.path.join(OUT, "solid"); os.makedirs(sdir, exist_ok=True)
    res = {}
    fn = os.path.join(OUT, "solid_check.json")
    if os.path.exists(fn): res = json.load(open(fn))
    for f in sorted(os.listdir(pdir)):
        k = f[:-4]
        if not f.endswith(".stl") or (names and k not in names): continue
        if k in res and res[k].get("sha") == sha_of(os.path.join(pdir, f)): continue
        t0 = time.time()
        r = subprocess.run([sys.executable, os.path.join(NOX, "solid_check.py"), os.path.join(pdir, f)], capture_output=True, text=True)
        txt = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-400:]
        open(os.path.join(sdir, f"{k}.txt"), "w").write(r.stdout + r.stderr)
        res[k] = dict(ok=" PASS " in txt, line=txt[:1500], sha=sha_of(os.path.join(pdir, f)), s=round(time.time() - t0))
        log(f"solid_check {k}: {'PASS' if res[k]['ok'] else 'FAIL'} ({res[k]['s']} s) {'' if res[k]['ok'] else txt[txt.find('{'):][:260]}")
        json.dump(res, open(fn, "w"), indent=1)
    return res


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--stage", default="structure"); ap.add_argument("--names", default="")
    ap.add_argument("--h", type=float, default=4.0); ap.add_argument("--mode", default="all"); ap.add_argument("--only", default=""); ap.add_argument("--tag", default="")
    ap.add_argument("--fe", default=""); ap.add_argument("--sizes", default=""); ap.add_argument("--out-sizes", dest="out_sizes", default="sizes.json")
    a = ap.parse_args()
    if a.stage == "fe":
        P, J, finfo = load_cut()
        res, Mo = run_cut_fe(P, J, finfo, h=a.h, mode=a.mode, only=[x for x in a.only.split(",") if x] or None, log=log)
        json.dump(res, open(os.path.join(OUT, f"fe_h{a.h:g}_{a.mode}{a.tag}.json"), "w"), indent=1, default=float)
    if a.stage == "solid":
        solid_all(set(a.names.split(",")) - {""} or None, log=log)
    if a.stage == "sizes":
        fe = json.load(open(os.path.join(OUT, a.fe)))
        cd = json.load(open(os.path.join(OUT, "cut.json")))
        sizes, rows = size_joints(fe, cd["joints"])
        json.dump({f"{k[0]}|{k[1]}|{k[2]}": v for k, v in sizes.items()}, open(os.path.join(OUT, a.out_sizes), "w"), indent=0)
        json.dump(rows, open(os.path.join(OUT, a.out_sizes.replace(".json", "_table.json")), "w"), indent=0)
        log(f"bolt sizing: {sum(1 for r in rows if r['now'] != r['was'])} of {len(rows)} joints change; over target after M5 x 6: {[r['joint'] for r in rows if r['over']]}")
    if a.stage == "report":
        t0 = time.time()
        report_stage(a.h, a.tag, log=log)
        log("report written")
    if a.stage == "export":
        sizes = None
        if a.sizes:
            sizes = {tuple(k.split("|")): tuple(v) for k, v in json.load(open(os.path.join(OUT, a.sizes))).items()}
        P, J, pl, finfo, PROT, KEEP = build_cut(sizes=sizes, log=log)
        rows = export_cut(P, J, finfo)
        log(f"exported {len(rows)} pieces, {sum(r['mass_g'] for r in rows.values()):.0f} g (M, 1.27 g/cm3)")
        log(f"piece overlaps (mm3): {overlaps(P)}")
    if a.stage == "structure":
        S = structure(force=True); log(f"S {S.volume() / 1e3:.0f} cm3")
    if a.stage == "part0":
        S = structure(); out, pl = partition(S)
        P = fold(out, pl, log=log)
        P = size_splits(P, pl, log=log)
        for k, m in P.items():
            b = bbox(m); log(f"{k}: {m.volume() / 1e3:.1f} cm3 ext {np.round(b[1] - b[0]).tolist()} lo {np.round(b[0]).tolist()} comps {[round(c.volume(), 1) for c in comps(m)][:6]}")
        C = contacts(P, pl)
        for c in C: log(f"  contact {c['a']} | {c['b']} on {c['plane'].name}: {c['area'] / 100:.1f} cm2")
        log(f"{len(C)} interfaces")
    if a.stage == "laps":
        P, J, pl, finfo, PROT, KEEP = build_cut(log=log)
        os.makedirs(os.path.join(OUT, "pieces_wip"), exist_ok=True)
        for k, m in P.items():
            b = bbox(m); log(f"{k}: {m.volume() / 1e3:.1f} cm3 ext {np.round(b[1] - b[0]).tolist()} lo {np.round(b[0]).tolist()} comps {[round(c.volume(), 1) for c in comps(m)][:4]}")
            tm(m).export(os.path.join(CACHE, f"wip_{k}.stl"))
        json.dump(J, open(os.path.join(CACHE, "wip_joints.json"), "w"), indent=1, default=float)
        ov = overlaps(P); log(f"piece overlaps (mm3): {ov}")
        import networkx as nx
        G_ = nx.Graph(); G_.add_nodes_from(P)
        for j in J:
            if j["sites"]: G_.add_edge(j["a"], j["b"])
        log(f"bolted graph: {nx.number_connected_components(G_)} components, sizes {sorted([len(c) for c in nx.connected_components(G_)], reverse=True)}")


if __name__ == "__main__":
    main()


