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


RING_Y = (74.0, 98.0)                                 # the datum ring (round 2: r 115..200, Y 74..98, -210..30 deg)
RING_GROUPS = [(0, 2), (2, 4), (4, 5), (5, 7), (7, 9)]   # edge indices of the 5 segments: bays {30, 0}, {-30, -60}, {-90}, {-120, -150}, {-180, -210}


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
    for i, (i0, i1) in enumerate(RING_GROUPS):
        hi, lo = edges[i0], edges[i1]
        R.append((f"ring{i + 1}", wedge(lo, hi, P["c_r"], (hi + lo) / 2.0, RING_Y[0], RING_Y[1])))
    pl += [Plane("Y ring lo", [0, 1, 0], [1, 0, 0], [0, RING_Y[0], 0]), Plane("Y ring hi", [0, 1, 0], [1, 0, 0], [0, RING_Y[1], 0])]
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


SPLIT_LOG = []


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
        SPLIT_LOG.append(dict(piece=src, plane=name, axis=ax.upper(), at=round(float(c), 1), nominal=nom, section_cm2=round(area / 100, 1)))
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
        cx = float(np.clip(g.centroid.x, -(206.0 - FOOT["d"] / 2 - 0.3), 206.0 - FOOT["d"] / 2 - 0.3)); cy = float(max(g.centroid.y, -66.0 + FOOT["d"] / 2 + 0.3))   # the O20 foot stays inside the sheets
        out.append(dict(xy=[cx, cy], area=float(ar), bounds=[float(v) for v in g.bounds], moved=bool(abs(cx - g.centroid.x) + abs(cy - g.centroid.y) > 0.01)))
    return out


def join_islands(S, KEEP, log=print):
    """a body of S that is not joined to the main one (round 2's e-boss island) gets a drawn O18 rib to the nearest main material that
    the keep-outs leave free; the rib is part of the rear frame piece then."""
    from scipy.spatial import cKDTree
    cs = comps(S, 20000.0); info = []
    if len(cs) < 2: return S, info
    main = tm(cs[0]); tr = cKDTree(main.vertices)
    for isl in cs[1:]:
        t = tm(isl); d, i = tr.query(t.vertices); order = np.argsort(d)
        tried = 0
        for j in order[::max(1, len(order) // 400)]:
            if tried >= 60: break
            a_, b_ = t.vertices[j], main.vertices[i[j]]; ln = float(np.linalg.norm(b_ - a_)); tried += 1
            if ln < 1.0: continue
            dv = (b_ - a_) / ln
            rib = cyl_at(a_, dv, 9.0, ln + 12.0, back=6.0)
            rc = rib - KEEP
            if rc.volume() < 0.97 * rib.volume(): continue
            S2 = S + rc
            if len(comps(S2, 20000.0)) < len(cs):
                info.append(dict(from_=np.round(a_, 1).tolist(), to=np.round(b_, 1).tolist(), length_mm=round(ln, 1), volume_cm3=round(rc.volume() / 1e3, 1)))
                log(f"island {isl.volume() / 1e3:.1f} cm3 joined by a drawn O18 rib of {ln:.0f} mm ({rc.volume() / 1e3:.1f} cm3)")
                S = S2; break
        else: log("WARNING: island not joined")
    return S, info


def add_features(S, log=print):
    """foot trim + pads, receiver insert holes, seat-post insert holes. Returns S and PROT (hole + wall volumes the laps keep off)."""
    import box_system_topo_v2r2 as R2
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
    S, rib_info = join_islands(S, KEEP, log)
    info["ribs"] = rib_info
    ring = (R2.datum_ring() ^ D["env"]) - KEEP
    S = S + ring; info["ring_cm3"] = round(ring.volume() / 1e3, 1); log(f"datum ring (drawn r 115..200, Y 74..98, minus keep-outs): {info['ring_cm3']} cm3")
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
    mk = lambda tr, nr, g=0.0: pl.world(box(tr[0] - g, tr[1] + g, -hw - g, hw + g, nr[0] - g, nr[1] + g).transform(T.tolist()))
    nb = ov / 2.0 if owner_is_a else -ov / 2.0
    pt = T[:, :3] @ np.array([0.0, 0.0, nb]) + T[:, 3]; tdir = T[:, :3] @ np.array([-1.0, 0, 0])
    return mk((-Lt, 0.0), n_lug), mk((-Lt - bs, -Lt), n_bs), pt, tdir, mk((-Lt, 0.0), n_lug, LAP_GAP)


def apply_site(pieces, KEEP, itf, c, owner_is_a, spec, log=print, level=0, PROT=None):
    pl = itf["plane"]; a, b = itf["a"], itf["b"]
    O, Rr = (a, b) if owner_is_a else (b, a)
    lug, boss, pt, tdir, lugp = site_boxes(pl, c, owner_is_a, spec)
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
    nR = ((pieces[Rr] - lugp) + (boss_c - lugp)) - ins
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


SEAT_LIMIT = 150.0                                                       # mm3 per seat contact pair (E): a flat seat face meets its part with a film under 0.25 thick (the finish)
LAP_GAP = 0.3                                                            # fit clearance of the lug pocket in the receiving piece, per side (E: print tolerance; the finish grows faces up to 0.3)
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
         "rearP_2_1": "rearP_up_b", "rearP~1": "epost", "rearP~2": "rearP_lug", "rearN_1": "rearN_low", "rearN_2": "rearN_up", "rearN~1": "floorbar"}
    hubs = sorted([k for k in pieces if k == "hub" or k.startswith("hub~") or k.startswith("hub_")], key=lambda k: -pieces[k].volume())
    for i, k in enumerate(hubs): m[k] = f"link_{i + 1}"
    legs = sorted([k for k in pieces if "~" in k and k not in m and bbox(pieces[k])[0][2] < -228.0])
    for i, k in enumerate(legs): m[k] = f"foot_leg_{i + 1}"
    for k in pieces:
        if "~" in k and k not in m: m[k] = k.replace("~", "_s")
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
CLIP = dict(post=10.0, root=8.0, pad_d=14.0, pad_t=3.0, gus_b=12.0, gus_h=14.0, gus_t=3.0, ins_d=4.0, ins_h=6.7, gap=0.2)


def clip_solid(p, n, L):
    """one clip tab (attachment rule): a post 10 x 10 (>= 6 x 6; wall 3 round the M3 insert), 8 into the member, a pad O14 x 3 at the end (web >= 2),
    four triangular gussets 3 thick (12 x 14) at the root into the member's face; L = distance from the surface point to the pad's outer face."""
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, a); u /= np.linalg.norm(u); v = np.cross(n, u)
    c = CLIP; hp = c["post"] / 2
    parts = [box(-hp, hp, -hp, hp, -c["root"], L - c["pad_t"] + 0.01), mf.Manifold.cylinder(c["pad_t"], c["pad_d"] / 2, c["pad_d"] / 2, 48).translate([0, 0, L - c["pad_t"]])]
    tri = mf.CrossSection([[(hp - 0.5, 0.0), (hp + c["gus_b"], 0.0), (hp - 0.5, c["gus_h"])]])
    g = mf.Manifold.extrude(tri, c["gus_t"]).rotate([90, 0, 0]).translate([0, c["gus_t"] / 2, 0])
    for k in range(4): parts.append(g.rotate([0, 0, 90 * k]))
    hole = mf.Manifold.cylinder(c["ins_h"] + 0.5, c["ins_d"] / 2, c["ins_d"] / 2, 24).translate([0, 0, L - c["ins_h"]])
    sol = union(parts) - hole
    T = np.hstack([np.stack([u, v, n], 1), np.asarray(p, float)[:, None]])
    return sol.transform(T.tolist())


def add_clips(P, KEEP, n_per=5, log=print):
    """5 clip tabs per sheet on frame members (clip_solid), points = surface vertices facing the sheet spread by farthest-point sampling."""
    clips = {}
    for sn, (ax, c, sg) in SHEETS.items():
        cand = []
        for k, m in P.items():
            t = tm(m); vn = t.vertex_normals; v = t.vertices
            ok = (vn[:, ax] * sg >= 0.8) & (v[:, ax] * sg >= (c * sg) - 60.0) & (v[:, ax] * sg <= c * sg - 1.0 + 1e-6)
            for i in np.where(ok)[0][::7]: cand.append((k, v[i].copy()))
        if not cand: clips[sn] = []; log(f"  {sn}: no member within 60 mm"); continue
        other = [a for a in range(3) if a != ax]
        pts = np.array([p[other] for _, p in cand])
        first = int(np.argmax([(p[ax] * sg) for _, p in cand])); sel = [first]
        d = np.linalg.norm(pts - pts[first], axis=1)
        while len(sel) < n_per * 5 and len(sel) < len(cand):
            i = int(np.argmax(d)); sel.append(i); d = np.minimum(d, np.linalg.norm(pts - pts[i], axis=1))
        done = []
        for i in sel:
            k, p = cand[i]
            n = np.zeros(3); n[ax] = sg
            L = (c - p[ax]) * sg - CLIP["gap"]
            sol = clip_solid(p, n, L)
            core = box(-1, 1, -1, 1, -1, 1)
            sc = sol - KEEP
            if sc.volume() < 0.93 * sol.volume(): continue
            bad = False; sb = bbox(sol)
            for kk, m in P.items():
                if kk == k: continue
                bb_ = bbox(m)
                if (bb_[0] < sb[1]).all() and (sb[0] < bb_[1]).all() and (sc ^ m).volume() > 3.0: bad = True; break
            if bad: continue
            P[k] = P[k] + sc
            tip = p + n * L
            done.append(dict(piece=k, at=np.round(p, 1).tolist(), tip=np.round(tip, 1).tolist(), axis_in=np.round(-n, 3).tolist(), post_len=round(float(L), 1)))
            if len(done) >= n_per: break
        clips[sn] = done
        log(f"  clips {sn}: {len(done)} on {sorted(set(d_['piece'] for d_ in done))}")
    return clips


def datum_fixings(P, KEEP, PROT, planes, log=print):
    """each bay body sits on its ring segment: 2 dowel seats O4 x 6 + 2 M3 from below (screw through the ring from its Y 74 face into an insert in the
    bay body at Y 98). Points inside the contact face of ring | bay on the plane Y 98, 8 from its edge, 25 apart, off the receiver bolts (PROT)."""
    from shapely.geometry import Point
    pl = next(q for q in planes if q.name == "Y ring hi"); Yh, Yl = RING_Y[1], RING_Y[0]
    out = []
    ringp = [k for k in P if k.startswith("ring")]
    cts = [c for c in contacts({k: P[k] for k in P}, [pl], amin=120.0)]
    for ct in cts:
        a, b = ct["a"], ct["b"]
        if a not in ringp or b.startswith("ring"): continue
        g = poly_of(ct["P"]).buffer(-5.5)
        if g.is_empty: log(f"  datum {a} | {b}: contact {ct['area']:.0f} mm2 too small"); continue
        xs = []
        gg = list(g.geoms) if hasattr(g, "geoms") else [g]
        for pg in gg:
            mnx, mny, mxx, mxy = pg.bounds
            for x in np.arange(mnx, mxx + 1e-6, 2.5):
                for y in np.arange(mny, mxy + 1e-6, 2.5):
                    if pg.contains(Point(x, y)): xs.append((x, y))
        if not xs: log(f"  datum {a} | {b}: no free point in {ct['area']:.0f} mm2"); continue
        xs = np.array(xs); chosen = []
        order = list(np.argsort(-np.linalg.norm(xs - xs.mean(0), axis=1)))
        for idx in order:
            q = xs[idx]
            if any(np.linalg.norm(q - c_) < 18.0 for c_ in chosen): continue
            w = pl.to_world([q[0], q[1], 0.0])
            cb = cyl_at([w[0], Yh, w[2]], [0, 1, 0], 6.0, 9.0)
            if (cb ^ PROT).volume() > 3.0: continue
            ring_f = (cyl_at([w[0], Yh - 8.0, w[2]], [0, 1, 0], 5.5, 8.0) ^ P[a]).volume() / (math.pi * 5.5 ** 2 * 8.0)
            if ring_f < 0.7: continue
            bay_f = (cb ^ P[b]).volume() / cb.volume()
            access = cyl_at([w[0], Yl - 0.3, w[2]], [0, -1, 0], 5.0, 10.0)
            if (access ^ KEEP).volume() > 40.0 or any((access ^ m).volume() > 40.0 for m in P.values()): continue
            chosen.append(q)
            out.append(dict(ring=a, bay=b, x=round(float(w[0]), 2), z=round(float(w[2]), 2), bay_fill=round(float(bay_f), 2), ring_fill=round(float(ring_f), 2)))
            if len(chosen) >= 4: break
        # assign kinds alternately along the chosen points: 0, 2 bolts; 1, 3 dowels (spread order)
        mine = [o for o in out if o["ring"] == a and o["bay"] == b]
        for n_, o in enumerate(mine):
            o["kind"] = "M3 screw from below" if n_ % 2 == 0 else "dowel O4 x 12"
            x, z = o["x"], o["z"]
            boss = cyl_at([x, Yh, z], [0, 1, 0], 5.5, 9.0) - KEEP
            if o["bay_fill"] < 0.85: P[b] = P[b] + boss
            if o["kind"].startswith("M3"):
                P[a] = (P[a] - cyl_at([x, Yl - 1.0, z], [0, 1, 0], 1.7, Yh - Yl + 1.5) - cyl_at([x, Yl - 1.0, z], [0, 1, 0], 3.1, 4.2))
                P[b] = P[b] - cyl_at([x, Yh - 0.5, z], [0, 1, 0], 2.0, 6.7 + 0.5)
            else:
                P[a] = P[a] - cyl_at([x, Yh - 6.0, z], [0, 1, 0], 2.0, 6.5)
                P[b] = P[b] - cyl_at([x, Yh - 0.5, z], [0, 1, 0], 2.0, 6.5)
        log(f"  datum {a} | {b}: {len(mine)} fixings ({sum(1 for o in mine if o['kind'].startswith('M3'))} M3, {sum(1 for o in mine if o['kind'].startswith('dowel'))} dowels)")
    return out


def separate(P, log=print, vmin=2.0):
    """the finish moves a face on an oblique cut plane by up to +-0.25 (voxels), so two pieces that meet flush on one overlap by a film;
    the larger piece gives the film up (P[big] - P[small]); the faces stay flush, no overlap remains. What was taken is logged."""
    ks = list(P); bbs = {k: bbox(P[k]) for k in ks}
    for i, a in enumerate(ks):
        for b in ks[i + 1:]:
            A, B = bbs[a], bbs[b]
            if not ((A[0] < B[1]).all() and (B[0] < A[1]).all()): continue
            v = (P[a] ^ P[b]).volume()
            if v <= vmin: continue
            big, small = (a, b) if P[a].volume() >= P[b].volume() else (b, a)
            new = P[big] - P[small]
            cs = comps(new, 0.0); P[big] = cs[0]
            log(f"  separate {a} | {b}: film {v:.0f} mm3 taken from {big}" + (f", {sum(c.volume() for c in cs[1:]):.1f} mm3 of slivers dropped" if len(cs) > 1 else ""))
            bbs[big] = bbox(P[big])
    return P


def build_cut(sizes=None, log=print, finish=True):
    S = structure(); S, PROT, finfo = add_features(S, log=log)
    out, pl = partition(S)
    P = fold(out, pl, log=log); P = size_splits(P, pl, log=log)
    P, nm = rename(P)
    D = drawn_inputs(); KEEP = D["cuts"] - D["drawn"]
    t0 = time.time(); J = laps(P, pl, KEEP, sizes=sizes, log=log, PROT=PROT); log(f"laps {time.time() - t0:.0f} s")
    finfo["datum"] = datum_fixings(P, KEEP, PROT, pl, log=log)
    finfo["clips"] = add_clips(P, KEEP, log=log)
    P = clean(P, log)
    if finish:
        envc = box(-205.9, 205.9, -65.9, 334.9, -236.0, 234.0)                  # the finish grows flush faces by up to 0.3: back inside the sheets' inner planes
        for k in list(P):
            P[k] = finish_piece(P[k], log=lambda s_: log(f"  {k}:{s_}"))
            v0 = P[k].volume(); P[k] = P[k] ^ envc
            if v0 - P[k].volume() > 0.5: log(f"  {k}: {v0 - P[k].volume():.0f} mm3 trimmed back to the sheet planes")
        P = separate(P, log)
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
    json.dump(dict(pieces=rows, joints=J, feet=finfo["feet"], clips=finfo.get("clips", {}), splits=SPLIT_LOG, datum=finfo.get("datum", []), ribs=finfo.get("ribs", []), ring_cm3=finfo.get("ring_cm3")), open(os.path.join(OUT, f"cut{tag}.json"), "w"), indent=1, default=float)
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
    EXP = os.environ.get("NOX_EXP", "")
    for j in J:
        if not j["sites"]: continue
        zs = []
        full = mode == "all" or ("ringlaps" in EXP and j["a"].startswith("ring") and j["b"].startswith("ring")) or ("baylaps" in EXP and j["a"].startswith("bay") and j["b"].startswith("bay")) or ("linklaps" in EXP and (j["a"].startswith("link") or j["b"].startswith("link")))
        for s in j["sites"]:
            spec = BOLT[s["size"]]
            r = (9.0 if "bigdisc" in EXP else 6.0) if mode == "bolts" else spec["hw"] + 1.0
            zs.append(bolt_zone(s["pos"], s["axis_in"], r, -1.0 - spec["cb_h"], spec["Lt"] + (3.0 if mode == "bolts" else spec["boss"])))
        if full:
            n = np.array(j["plane_n"]); p0 = np.array(j["plane_p0"])
            zs.append(lambda X, n=n, p0=p0: np.abs((X - p0) @ n) <= 2.5)
        defs.append(dict(a=j["a"], b=j["b"], tag=j["plane"], zone=lambda X, zs=zs: np.any([z(X) for z in zs], axis=0)))
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
            ea.append(pa[ok]); eb.append(pb[ok]); self.pairs[(a, b, df.get("tag", ""))] = g[ok]
        from scipy.sparse.csgraph import connected_components
        ea = np.concatenate(ea); eb = np.concatenate(eb)
        Gm = sp.coo_matrix((np.ones(len(ea)), (ea, eb)), shape=(self.nP, self.nP))
        self.nN, self.comp = connected_components(Gm, directed=False)
        self.pg = np.empty(self.nP, np.int64)
        for k in self.names: self.pg[self.poff[k]: self.poff[k] + len(self.pnodes[k])] = self.pnodes[k]

    def solve(self, Fp, fixed_p):
        """as box_system.Model.solve with the diagonal regularisation REG (v1: 1e-4; here 1e-6, the equilibrium residual is printed per case)."""
        import pypardiso
        F = np.zeros((self.nN, 3)); np.add.at(F, self.comp, Fp)
        fixed = np.zeros(self.nN, bool); fixed[self.comp[fixed_p]] = True
        free = np.where(~np.repeat(fixed, 3))[0]
        key = fixed.tobytes().__hash__()
        if getattr(self, "_fkey", None) != key:
            if getattr(self, "_ps", None) is not None: self._ps.free_memory(everything=True)
            self._Kf = sp.triu(self.K[free][:, free] + sp.identity(len(free)) * REG, format="csr"); self._ps = pypardiso.PyPardisoSolver(); self._ps.set_matrix_type(2)
            self._fkey = key; self._free = free
        U = np.zeros(3 * self.nN); U[free] = self._ps.solve(self._Kf, F.ravel()[free])
        return U.reshape(-1, 3)

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


REG = 1e-6
GRID_O2 = np.array([-212.0, -72.0, -236.0]); GRID_MAX2 = np.array([212.0, 340.0, 240.0])


def extra_defs(P, finfo, specs, mode="all"):
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
    for o in finfo.get("datum", []):                                           # the datum fixings: a disc round each screw / dowel on the Y 98 face
        defs.append(dict(a=o["ring"], b=o["bay"], tag="datum", zone=bolt_zone([o["x"], RING_Y[1], o["z"]], [0, 1, 0], (9.0 if "bigdisc" in os.environ.get("NOX_EXP", "") else 6.0) if mode == "bolts" else 7.5, -2.5, 2.5)))
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
    defs = joint_defs(J, P, mode, h) + extra_defs(P, finfo, specs, mode)
    Mo = CutModel(bodies, h, defs, log=log); Mo.assemble()
    R = fe_regions(Mo, list(P), specs)
    CS = cut_cases()
    if only: CS = {k: v for k, v in CS.items() if any(o in k for o in only)}
    jmap = {(j["a"], j["b"], j["plane"]): j for j in J if j["sites"]}
    pc = np.zeros(Mo.nP)
    for (a, b, _t), g in Mo.pairs.items():
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
            fx = pn[Xp[:, 0] >= 204.0 - 1e-6]; nrm = np.array([-1.0, 0, 0])             # the +X face layer (sheet inner plane X 206: nodes of the last 2 cells)
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
            joints[f"{ab[0]} | {ab[1]} | {ab[2]}"] = jd
        # floor reactions: the support's reaction per merged node (K u - F summed over its piece nodes), shared over its fixed piece nodes
        reac = {}
        cm = np.zeros(Mo.nN); np.add.at(cm, Mo.comp[fx], 1)
        Rc = np.zeros((Mo.nN, 3)); np.add.at(Rc, Mo.comp, fj)
        sh = Rc[Mo.comp[fx]] / cm[Mo.comp[fx]][:, None]
        for k in feet:
            q = np.arange(Mo.poff[k], Mo.poff[k] + len(Mo.pnodes[k])); sel = np.isin(fx, q)
            reac[k] = round(float(sh[sel][:, 2].sum()), 1) if sel.any() else 0.0
        react_total = np.round(sh.sum(0), 1).tolist(); load_total = np.round(Fp.sum(0), 1).tolist()
        resid = np.array(react_total) + np.array(load_total)                    # support reaction + applied loads: zero in equilibrium
        resid_pct = float(np.linalg.norm(resid) / max(np.linalg.norm(load_total), 1e-9) * 100.0)
        resid_axis_pct = [round(float(abs(resid[i]) / max(np.abs(load_total).max(), 1e-9) * 100.0), 2) for i in range(3)]
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
                            ext=ext, reaction_sum_N=react_total, applied_sum_N=load_total, equilibrium_residual_pct=round(resid_pct, 3), residual_axis_pct_of_largest_load=resid_axis_pct, contact_iterations=it + 1, umax_mm=round(float(np.abs(U).max()), 3), feet_in_contact=[k for k in feet if reac[k] >= 0.5], support=c["support"],
                            sup_nodes=fx.tolist() if False else None)
        wk = max(rows, key=lambda k: rows[k]["worst"])
        log(f"{cname:50s} worst {rows[wk]['worst']:.2f} {wk}  stub rot {np.round(rot, 2).tolist()} mrad  feet {len(cases[cname]['feet_in_contact'])}  resid {resid_pct:.2f} %  ({time.time() - t0:.0f} s)")
    import resource
    foot_owner = {a: b for (a, b, _t) in Mo.pairs if re.fullmatch(r"foot_\d+", a)}
    res = dict(h=h, mode=mode, foot_owner=foot_owner, peak_rss_GB=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6, 2), cells=int(sum(len(c) for c in Mo.cells.values())), dofs=int(3 * Mo.nN), cases=cases,
               joint_nodes={f"{a} | {b} | {t}": int(len(g)) for (a, b, t), g in Mo.pairs.items()}, centroids={f"{a} | {b} | {t}": np.round(v, 1).tolist() for (a, b, t), v in jc.items()}, regularisation=REG)
    return res, Mo


# ================================================================================================ sizing, assembly, requirements
def size_joints(fe, J, target=0.8, nmax=6):
    """bolt size and count per joint from the FE: the worst bolt index over the cases, scaled by the bolts there are; M3 up to nmax
    bolts, else M5 (about 2.2 x the capacity). -> {(a, b, plane): (size, n)} for laps(); log of what changed."""
    worst = {}
    for f_ in (fe if isinstance(fe, list) else [fe]):
        for cn, c in f_["cases"].items():
            for jk, jd in c["joints"].items():
                if "bolt_worst" in jd: worst[jk] = max(worst.get(jk, 0.0), jd["bolt_worst"])
    sizes = {}; rows = []
    for j in J:
        if not j["sites"]: continue
        jk = f"{j['a']} | {j['b']} | {j['plane']}"; w = worst.get(jk, 0.0); n0 = len(j["sites"]); s0 = j["size"]
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
        parts.append(dict(name=k, role="structure", mesh=rel(os.path.join(OUT, "pieces", f"{k}.stl")), pose=np.eye(4).tolist(), material="TPU", tag="E",
                          owner="box structure", version="v2 cut", source="nox/box/box_system_v2_cut.py", note="TPU foot O20 x 5 (E)"))
    names = {p["name"] for p in parts}
    pairs = [pr for pr in old["contact_pairs"] if pr[0] in names and pr[1] in names]
    for j in J:
        if j["sites"]: pairs.append([j["a"], j["b"], 5.0])                      # bolted laps: the tab sits on its recess
    mans = {k: m for k, m in P.items()}
    seat_ov = {}
    tmpA = assembly.new("tmp", "box", parts=parts, probes=False)
    posed = {p["name"]: man(t) for p, t in zip(tmpA.parts, tmpA.meshes()) if p["name"].startswith("catch: ")}
    ovl = lambda a, b: (a ^ b).volume() if (np.array(bbox(a)[0]) < bbox(b)[1]).all() and (np.array(bbox(b)[0]) < bbox(a)[1]).all() else 0.0
    for ang in BAY_ANG:
        rc = man(trimesh.load(os.path.join(LAY, "parts", f"bay{ang}_receiver.stl"), force="mesh"))
        for k, m in mans.items():
            v = ovl(rc, m)
            if v > 0.0: pairs.append([f"bay {ang} receiver", k, SEAT_LIMIT]); seat_ov[f"bay {ang} receiver ^ {k}"] = round(v, 1)
    for n, cm in posed.items():
        for k, m in mans.items():
            v = ovl(cm, m)
            if 0 < v < 1500.0: pairs.append([n, k, SEAT_LIMIT]); seat_ov[f"{n} ^ {k}"] = round(v, 1)       # the seat landing: fixed limit by role, real overlap reported
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
    json.dump(dict(limit_mm3=SEAT_LIMIT, note="real overlaps (M) of the receivers and catch parts with the structure at their seat faces; the pair limit is fixed by role, not scaled to these", overlaps_mm3=seat_ov), open(os.path.join(OUT, "seat_overlaps.json"), "w"), indent=1)
    return A


def clearance(P):
    """overlap volume (mm3) of every piece with each keep-out of round 2 (grown 1 mm: the finish may use up to 0.25 of that margin; the true
    envelopes are in the System line) and with the fixed electronics: PSU, charger, the sheet face strip Y -66..-50."""
    import box_system_topo_v2r2 as R2
    D = drawn_inputs(); T = R2.T
    ko = {n: T.load_grown(n) for n in ("receivers", "cartridge_paths", "drone", "catch")}
    for f in ("hub_27.stl", "motors_28.stl", "parked_29.stl", "drone_30.stl", "drone_31.stl"): ko[f] = R2.load_grown_path(os.path.join(LAY, "parts", f))
    ko["rail disc"] = R2.cylY(R2.L2.RAIL_DISC["r"] + R2.GROW, R2.L2.RAIL_DISC["y"][0] - R2.GROW, R2.L2.RAIL_DISC["y"][1] + R2.GROW)
    ko["funnel inside"] = D["fun"]
    out = {}
    for n, k_ in ko.items():
        k2 = k_ - D["drawn"]; rows = [(k, round((m ^ k2).volume(), 1)) for k, m in P.items() if (m ^ k2).volume() > 0.5]
        out[n] = dict(kind="keep-out grown 1 mm", total_mm3=round(sum(v for _, v in rows), 1), pieces=rows)
    fixed = {"PSU X 42..157 Y -66..-36 Z -170..45": box(42, 157, -66, -36, -170, 45), "charger X -162..-44 Y -66..-7 Z -200..-29": box(-162, -44, -66, -7, -200, -29),
             "sheet face strip Y -66..-50": box(-210, 210, -66, -50, -240, 240)}
    for n, b in fixed.items():
        rows = [(k, round((m ^ b).volume(), 1)) for k, m in P.items() if (m ^ b).volume() > 0.5]
        out[n] = dict(kind="fixed electronics", total_mm3=round(sum(v for _, v in rows), 1), pieces=rows)
    return out


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
    fes = fe if isinstance(fe, list) else [fe]
    fe = fes[0]
    pieces = sorted({k for c in fe["cases"].values() for k in c["rows"]})
    tab = {}
    for k in pieces:
        row = dict(by_case={cn: max(f_["cases"][cn]["rows"][k]["worst"] for f_ in fes if cn in f_["cases"]) for cn in fe["cases"]}, by_class={})
        for cn, c in fe["cases"].items():
            cl = c["cls"]; row["by_class"][cl] = max(row["by_class"].get(cl, 0.0), row["by_case"][cn])
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


def lap_records(J, fe, fb):
    """one record per lap (a, b, plane), bolted or not: bolts placed / wanted, the wrench per case in both joint models (bonded = every cut and lap face,
    bolt_disc = only an O12 disc at each bolt), the bolt index per case, the worst, the status (ok / over aim / unbolted) and the bolts the aim needs."""
    recs = []
    for j in J:
        jk = f"{j['a']} | {j['b']} | {j['plane']}"; n = len(j["sites"]); z = j["size"]
        cases = {}
        for mname, f_ in (("bonded", fe), ("bolt_disc", fb)):
            if f_ is None: continue
            for cn, c in f_["cases"].items():
                jd = c["joints"].get(jk)
                if jd is None: continue
                cases.setdefault(cn, {})[mname] = dict(F_N=jd["F"], M_Nmm=jd["M_Nmm"], bolt_idx=jd.get("bolt_idx"), bolt_worst=jd.get("bolt_worst"))
        worst, wcase, wmodel = 0.0, None, None
        for cn, e in cases.items():
            for mname, d in e.items():
                if d["bolt_worst"] is not None and d["bolt_worst"] > worst: worst, wcase, wmodel = d["bolt_worst"], cn, mname
        if n == 0: status = "unbolted"
        elif worst > AIM: status = "over aim"
        else: status = "ok"
        if n == 0: need3 = need5 = j["want"]
        else:
            need3 = int(math.ceil(worst * n / AIM)) if z == "M3" else int(math.ceil(worst * n * 2.2 / AIM))
            need5 = int(math.ceil(worst * n / (AIM * (1 if z == "M5" else 2.2))))
        recs.append(dict(key=jk, a=j["a"], b=j["b"], plane=j["plane"], plane_normal=j["plane_n"], area_cm2=round(j["area"] / 100, 2), size=z, placed=n, wanted=j["want"],
                         sites=j["sites"], cases=cases, worst_index=round(worst, 3), worst_case=wcase, worst_model=wmodel, status=status,
                         bolts_for_aim=dict(M3=need3, M5=need5), centroid=(fe or {}).get("centroids", {}).get(jk)))
    return recs


def write_requirements(P, J, finfo, fe, fe_b, tab, mc, pp, sc, out_dir):
    """req/<piece>.json (joints PER LAP) and requirements.md: envelope, every lap with bolts placed / wanted, the wrench per case in both joint models,
    the bolt index and status, the piece's own worst index (F), stiffness to keep, mass (M); the 'Over-aim joints' table."""
    os.makedirs(os.path.join(out_dir, "req"), exist_ok=True)
    recs = lap_records(J, fe, fe_b)
    sn = summary_numbers(fe); snb = summary_numbers(fe_b) if fe_b else None
    rows_md = ["| Piece | Qty | Envelope X / Y / Z (W, M) | Mass (M) | Worst index (F; case) | Over target (F) | Laps (partner / plane: placed of wanted x size, status) | solid_check |", "|---|---|---|---|---|---|---|---|"]
    for k, m in P.items():
        b = bbox(m); t = tab.get(k, {})
        laps = []
        for r in recs:
            if k not in (r["a"], r["b"]): continue
            sg = 1.0 if r["a"] == k else -1.0
            cs = {}
            for cn, e in r["cases"].items():
                cs[cn] = {mn: dict(F_N=np.round(sg * np.array(d["F_N"]), 2).tolist(), M_Nmm=np.round(sg * np.array(d["M_Nmm"]), 0).tolist(), bolt_idx=d["bolt_idx"], bolt_worst=d["bolt_worst"]) for mn, d in e.items()}
            laps.append(dict(partner=r["b"] if r["a"] == k else r["a"], plane=r["plane"], plane_normal=r["plane_normal"], area_cm2=r["area_cm2"], centroid=r["centroid"],
                             bolts_placed=r["placed"], bolts_wanted=r["wanted"], size=r["size"], sites=r["sites"], status=r["status"], worst_bolt_index=r["worst_index"],
                             worst_case=r["worst_case"], worst_model=r["worst_model"], bolts_for_aim=r["bolts_for_aim"],
                             wrench_by_case=cs, wrench_note="forces / moments act ON this piece (N, N.mm about the lap centroid, W, F); unbolted laps are not bonded in the FE and carry none"))
        ext = {cn: c["ext"][k] for cn, c in fe["cases"].items() if k in c["ext"]}
        stiff = {}
        if "mast" in k:
            stiff["stub_tilt"] = "<= 10 mrad (E, arm round 3) at 57 N + 5.9 N.m, k >= 0.59 N.m/mrad (E); this model (F): bonded %s mrad%s" % (sn.get("stub_tilt_mrad"), (", bolt-disc %s mrad" % snb.get("stub_tilt_mrad")) if snb else "")
        if k.startswith("bay") or k.startswith("ring"):
            stiff["bay_rotation"] = "receiver mount about the axle relative to the stub <= 2.6 mrad (E, +-0.15 deg datum); this model (F), worst bay: bonded %s mrad%s" % (sn["bay_rot_worst_mrad"], (", bolt-disc %s mrad" % snb["bay_rot_worst_mrad"]) if snb else "")
        r_ = dict(piece=k, version="v2 cut", file=rel_p(os.path.join(OUT, "pieces", f"{k}.stl")), frame="W (box-arm.md): X across, Y along the axle, Z up",
                  envelope=dict(lo=np.round(b[0], 1).tolist(), hi=np.round(b[1], 1).tolist(), extents=np.round(b[1] - b[0], 1).tolist(), print_pose=pp.get(k)),
                  mass_g_M=mc["rows_g"].get(k), qty=1, laps=laps, external_by_case=ext, own_worst_index=dict(worst=t.get("worst"), case=t.get("worst_case"), by_class=t.get("by_class"),
                  targets=TARGETS, aim=AIM, tag="F"), stiffness_to_keep=stiff, solid_check=sc.get(k, {}).get("ok"),
                  note="index = the system FE worst cell index at the case class (F, h %g, bonded model); laps listed per lap, not per partner" % fe["h"])
        json.dump(r_, open(os.path.join(out_dir, "req", f"{k}.json"), "w"), indent=0, default=float)
        wj = "; ".join(f"{l_['partner']} / {l_['plane']}: {l_['bolts_placed']} of {l_['bolts_wanted']} x {l_['size']}{'' if l_['status'] == 'ok' else ' ' + l_['status']}" for l_ in laps) or "-"
        over = ", ".join(f"{c} {v:.2f}" for c, v in t.get("over", {}).items()) or "-"
        rows_md.append(f"| {k} | 1 | {' x '.join(f'{e:.0f}' for e in b[1] - b[0])} | {mc['rows_g'].get(k)} g | {t.get('worst', float('nan')):.2f} F ({t.get('worst_case', '')[:30]}) | {over} | {wj} | {'PASS' if sc.get(k, {}).get('ok') else 'FAIL' if k in sc else 'not run'} |")
    ov = [r for r in recs if r["status"] == "over aim"]; ub = [r for r in recs if r["status"] == "unbolted"]
    rows_md += ["", f"Laps: {len(recs)} (M); bolted {len(recs) - len(ub)}, unbolted {len(ub)}, over the {AIM} index aim (E) {len(ov)}. Joint models (F): bonded = every cut and lap face; bolt-disc = only an O12 disc at each bolt. Bolt index = SF x (T / T_u + V / V_u), the worst of both models; tags: F indices and wrenches, M areas and counts, E capacities.", "",
                "## Over-aim joints (the piece runs must provide at least this)", "",
                "| Lap (a / b / plane) | Pieces to run | Lap area (M) | Placed of wanted x size (M) | Worst index (F; case, model) | Bolts for the aim (E): M3 or M5 | Note |", "|---|---|---|---|---|---|---|"]
    for r in sorted(ov, key=lambda q: -q["worst_index"]):
        rows_md.append(f"| {r['a']} / {r['b']} / {r['plane']} | {r['a']}, {r['b']} | {r['area_cm2']} cm2 | {r['placed']} of {r['wanted']} x {r['size']} | {r['worst_index']:.2f} ({(r['worst_case'] or '')[:30]}, {r['worst_model']}) | {r['bolts_for_aim']['M3']} x M3 or {r['bolts_for_aim']['M5']} x M5 | lap region long enough for that count, or a wider section |")
    if not ov: rows_md.append("| none | | | | | | |")
    rows_md += ["", "## Unbolted interfaces (contact only: no bolt placed; the FE carries nothing through them)", "",
                "| Lap (a / b / plane) | Area (M) | Wanted (M) | Why | Piece runs must |", "|---|---|---|---|---|"]
    for r in sorted(ub, key=lambda q: -q["area_cm2"]):
        rows_md.append(f"| {r['a']} / {r['b']} / {r['plane']} | {r['area_cm2']} cm2 | {r['wanted']} x M3 | no free site (keep-outs, 180 cube, a third piece) | give the lap region room for {r['wanted']} bolts, or the neighbours carry the path |")
    open(os.path.join(out_dir, "requirements.md"), "w").write("\n".join(rows_md) + "\n")
    return recs


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
    if not cutd.get("splits"):                                              # older cut.json: the last export's split lines from the log
        rx = re.compile(r"split (\S+) on (.+?) at ([XYZ]) (-?\d+) \(nominal (-?\d+), section ([\d.]+) cm2\)")
        m_ = [rx.search(l_) for l_ in open(os.path.join(OUT, "log.txt"))]; m_ = [x for x in m_ if x][-len(SPLITS):]
        cutd["splits"] = [dict(piece=x.group(1), plane=x.group(2), axis=x.group(3), at=float(x.group(4)), nominal=float(x.group(5)), section_cm2=float(x.group(6))) for x in m_]
    fe = json.load(open(os.path.join(OUT, f"fe_h{h:g}_all{tag}.json")))
    fb_path = os.path.join(OUT, f"fe_h{h:g}_bolts{tag}.json")
    fb = json.load(open(fb_path)) if os.path.exists(fb_path) else None
    scj = json.load(open(os.path.join(OUT, "solid_check.json"))) if os.path.exists(os.path.join(OUT, "solid_check.json")) else {}
    tab = case_tables([fe, fb] if fb else [fe]); sn = summary_numbers(fe); mc = mass_cg(P, finfo); pp = print_poses(P)
    snb = summary_numbers(fb) if fb else None
    A = write_cut_assembly(P, J, finfo, os.path.join(OUT, "box_structure_v2_cut.assembly.json"))
    chk = A.check(); system_line = chk.get("summary")
    rz = rear_zone(P); json.dump(rz, open(os.path.join(OUT, "rear_zone.json"), "w"), indent=0)
    clr = clearance(P); json.dump(clr, open(os.path.join(OUT, "clearance.json"), "w"), indent=0)
    recs = write_requirements(P, J, finfo, fe, fb, tab, mc, pp, scj, OUT)
    # mass groups of the System line (E) and the CG each total covers
    mcs = chk["mass_cg"]; groups = {}
    for r_ in mcs["parts"]:
        n_ = r_["name"]
        g_ = ("pieces" if n_ in P else "feet" if n_.startswith("foot_") else "receivers" if "receiver" in n_ else "cartridges" if "cartridge" in n_ else "catch parts" if n_.startswith("catch:")
              else "sheets" if n_.lower().startswith("sheet") else "other (PSU, charger, rods, arm envelopes)")
        d_ = groups.setdefault(g_, dict(g=0.0, tag=r_["tag"])); d_["g"] += r_["mass_g"]
        if r_["tag"] != d_["tag"]: d_["tag"] = "E"
    massgrp = dict(groups={k_: dict(g=round(v_["g"], 0), tag=v_["tag"]) for k_, v_ in groups.items()}, sysline_g=mcs["mass_g"], sysline_cg=mcs["cg"],
                   frame_g=round(sum(mc["rows_g"][k_] for k_ in P), 0), feet_g=round(sum(v_ for k_, v_ in mc["rows_g"].items() if k_.startswith("foot_")), 0))
    seat_p = os.path.join(OUT, "seat_overlaps.json"); seat = json.load(open(seat_p)) if os.path.exists(seat_p) else dict(limit_mm3=SEAT_LIMIT, overlaps_mm3={})
    cases_all = [c for f_ in (fe, fb) if f_ for c in f_["cases"].values()]
    resid_max_axis = max(max(c["residual_axis_pct_of_largest_load"]) for c in cases_all)
    resid_max = max(c["equilibrium_residual_pct"] for c in cases_all)
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
               system_line=system_line,
               peak_memory_GB=max([fe.get("peak_rss_GB") or 0, 2.34] + [v.get("peak_child_GB", 0) for v in scj.values()]),
               peak_memory_detail=dict(fe_h4=fe.get("peak_rss_GB"), cut_export=2.34, solid_check_largest_piece=max([v.get("peak_child_GB", 0) for v in scj.values()] or [0]),
                                       method="resource.getrusage ru_maxrss of the process (children for solid_check), logged at exit; M"),
               wall_time_s=round(time.time() - float(subprocess.run(["stat", "-c", "%W", os.path.join(OUT, "log.txt")], capture_output=True, text=True).stdout.strip() or time.time())),
               bed_face_best=min(pp.values(), key=lambda v_: -v_["bed_mm2"])["face"], bed_contact_mm2_range=[min(v_["bed_mm2"] for v_ in pp.values()), max(v_["bed_mm2"] for v_ in pp.values())])
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
    res.update(bay_rot_bonded_mrad=sn["bay_rot_worst_mrad"], bay_rot_bolt_disc_mrad=snb["bay_rot_worst_mrad"] if snb else None, stub_tilt_bonded_mrad=sn.get("stub_tilt_mrad"),
               stub_tilt_bolt_disc_mrad=snb.get("stub_tilt_mrad") if snb else None, mass_frame_g_M=massgrp["frame_g"], mass_sysline_g_E=massgrp["sysline_g"], mass_groups=massgrp["groups"],
               laps=len(recs), laps_unbolted=sum(1 for r in recs if r["status"] == "unbolted"), laps_over_aim=sum(1 for r in recs if r["status"] == "over aim"),
               equilibrium_residual_max_pct=round(resid_max, 3), equilibrium_residual_max_axis_pct=round(resid_max_axis, 3), regularisation=fe.get("regularisation"),
               datum_fixings=len(finfo.get("datum", [])), seat_overlaps_mm3=seat["overlaps_mm3"], pieces_over_target=[k_ for k_, t_ in tab.items() if t_["over"]])
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
    return dict(P=P, J=J, finfo=finfo, fe=fe, fb=fb, tab=tab, sn=sn, snb=snb, mc=mc, pp=pp, scj=scj, system_line=system_line, rz=rz, rec=rec, res=res, cutd=cutd, clr=clr,
                recs=recs, massgrp=massgrp, seat=seat, ring_names=[k_ for k_ in P if k_.startswith("ring")], ribs=finfo.get("ribs", []), resid_max_axis=resid_max_axis, reg=fe.get("regularisation", REG))


VISUAL_URL = "https://claude.ai/artifact/FzJg3vz6Z8UnnUaerm5h3p"


def model_table(fe, fb):
    """per case: stub tilt (hypot of the X and Z rotations), worst bay rotation relative to the stub (and the bay), equilibrium residual; both joint models."""
    rows = []
    for cn, c in fe["cases"].items():
        d = {"case": cn}
        for mn, f_ in (("b", fe), ("d", fb)):
            if f_ is None or cn not in f_["cases"]: continue
            cc = f_["cases"][cn]; r = cc["stub_rot_mrad"]
            br = cc["bay_rot_rel_stub_mrad"]; wb = max(br, key=lambda q: abs(br[q]))
            d[mn] = dict(tilt=math.hypot(r[0], r[2]), bay=abs(br[wb]), bay_at=wb, resid=cc["equilibrium_residual_pct"], axis=max(cc["residual_axis_pct_of_largest_load"]))
        rows.append(d)
    return rows


def write_summary(R, sizing_rows, runinfo, path):
    """summary.md from the report_stage results (tags: C drawn / layout, M mesh, E estimate, F this FE)."""
    P, J, finfo, fe, fb, tab, sn, snb, mc, pp, scj, rz, rec, res, cutd, clr = (R[k] for k in ("P", "J", "finfo", "fe", "fb", "tab", "sn", "snb", "mc", "pp", "scj", "rz", "rec", "res", "cutd", "clr"))
    recs = R["recs"]; mg = R["massgrp"]; seat = R["seat"]
    nbolt = {"M3": 0, "M5": 0}
    for j in J:
        for s_ in j["sites"]: nbolt[s_["size"]] += 1
    over = {k: t for k, t in tab.items() if t["over"]}
    nj = sum(1 for j in J if j["sites"])
    ov = [r for r in recs if r["status"] == "over aim"]; ub = [r for r in recs if r["status"] == "unbolted"]
    dat = finfo.get("datum", [])
    mt = model_table(fe, fb)
    bdl = lambda d_: "-" if d_ is None else f"{d_:.2f}"
    L = ["# Box structure v2: the cut of the round 2 field and the system FE on the cut assembly (`nox/box/box_system_v2_cut.py`)", "",
         "Status: RUN RECORD of candidates for the piece topology runs, not parts. Nothing goes to print/. Tags: C drawn / layout, M mesh, E estimate, F this FE model (voxel h %g mm (C), piece-labelled)." % fe["h"], "",
         f"Visual page: {VISUAL_URL} (source `runs/box_system/v2/cut/visual.html`, written by `make_page`).", "",
         f"{R['system_line']}", "",
         f"Result (F, h {fe['h']:g} mm, 18 cases, both joint models): stub tilt at 57 N + 5.9 N.m: bonded {sn.get('stub_tilt_mrad')} mrad, bolt-disc {snb.get('stub_tilt_mrad') if snb else 'not run'} mrad (target <= 10, E). "
         f"Worst bay rotation about the axle relative to the stub over all 18 cases: bonded {sn['bay_rot_worst_mrad']} mrad (`{sn['bay_rot_worst_case']}`), bolt-disc {snb['bay_rot_worst_mrad'] if snb else 'not run'} mrad"
         f"{' (`' + snb['bay_rot_worst_case'] + '`)' if snb else ''} (target <= 2.6, E). The design is held to the bolt-disc bound.",
         f"Datum (DEFAULT): a drawn ring of {len(R['ring_names'])} segments (C) r 115..200 (C) on Y 74..98 (C), {finfo.get('ring_cm3', 0):.0f} cm3 drawn (M), bolted to the mast and links, carries every bay body on {len(dat)} fixings (M, detail below). Alternative not used: a bolted hub link per bay to the mast.",
         f"Weight (M unless E): frame (the {len(P)} PETG pieces) {mg['frame_g']:.0f} g, feet {mg['feet_g']:.0f} g, structure {mc['structure_g']:.0f} g; was 4919 g (E) for the round 2 field. Two totals that differ by design: {mg['sysline_g']:.0f} g in the System line (E) = pieces + feet + receivers + cartridges + PSU + charger + sheets + catch parts, CG {mg['sysline_cg']} (E); and {rec['box_with_lumps_g']} g (E) = structure + cartridges + arm 2900 g (E) + funnel and latch 640 g (E) + electronics 2000 g (E), CG {rec['box_with_lumps_cg']} (E, W). The first has no arm, funnel or electronics lump; the second has no sheets, receivers or PSU. Structure CG {mc['cg']} (M).",
         "Mass groups of the System line (g, tag): " + "; ".join(f"{k_} {v_['g']:.0f} ({v_['tag']})" for k_, v_ in mg["groups"].items()) + ".",
         f"Equilibrium (F): lateral reaction + applied sum under {R['resid_max_axis']:.2f} % of the largest load on every axis in every case and both models (diagonal regularisation {R['reg']:g}, was 1e-4 with 6 % lost in the motors-cut lateral reaction); per-case table below.",
         f"Strength (F, worst of both models): worst piece index {res['worst_index']:.2f} ({res['worst_piece']}, {res['worst_case']}); targets 0.5 functional / 1.0 misuse and sustained (E), aim 0.8 (E): {len(over)} of {len(P)} pieces over target.",
         f"Joints (M, F): {len(recs)} laps, {nj} bolted ({nbolt['M3']} x M3 + {nbolt['M5']} x M5), {len(ub)} unbolted, {len(ov)} over the 0.8 bolt-index aim; tables in `requirements.md`.",
         f"Solid: solid_check PASS on {sum(1 for k_ in P if scj.get(k_, {}).get('ok'))} of {len(P)} pieces and {sum(1 for k_ in scj if k_.startswith('foot_') and k_[5:].isdigit() and scj[k_]['ok'])} of {len(finfo['feet'])} TPU feet, run on the exact files in `pieces/` (`solid/<piece>.txt`); a FAIL is a cut that is not a candidate.", "",
         "## Pieces", "", "| Piece | Qty | Envelope X / Y / Z (M) | Mass (M) | Worst index (F; case) | Bed face, contact (M) | Laps (partner: placed of wanted) | solid_check |", "|---|---|---|---|---|---|---|---|"]
    for k in P:
        b_ = bbox(P[k]); e = b_[1] - b_[0]; t = tab[k]
        lp = [f"{(r['b'] if r['a'] == k else r['a'])} {r['placed']}/{r['wanted']}{'' if r['status'] == 'ok' else ' ' + r['status']}" for r in recs if k in (r["a"], r["b"])]
        L.append(f"| {k} | 1 | {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} | {mc['rows_g'][k]} g | {t['worst']:.2f} F ({t['worst_case'][:34]}) | {pp[k]['face']}, {pp[k]['bed_mm2']:.0f} mm2 | "
                 f"{'; '.join(lp) or '-'} | {'PASS' if scj.get(k, {}).get('ok') else 'FAIL' if k in scj else 'not run'} |")
    L += ["", f"TPU feet: {len(finfo['feet'])} x O20 x 5 (E) under the floor patches of the field (M areas {', '.join(f'{f_['area'] / 100:.1f}' for f_ in finfo['feet'])} cm2) on O16 x 6 (C) pads with an M3 insert.", ""]
    L += ["## Datum ring (the bay rotation fix)", "",
          f"Round 2's ring (r 115..200, Y 74..98, minus its keep-outs) is drawn into the structure ({finfo.get('ring_cm3', 0):.0f} cm3, M) and cut in {len(R['ring_names'])} segments (5, not the 3 or 4 asked: a 60 deg arc at r 200 is a 200 mm chord, over the 180 cube, M). "
          "Each segment is bolted to the mast, the links or the neighbour segment by laps; each bay body sits on its ring segment on the Y 98 face with M3 screws from below (Y 74, through the ring, counterbore, insert in a bay boss) and O4 dowel seats (6.5 deep each side, E), in the clear of the receiver bolt access. Fixings (M, positions in `cut.json` `datum`):", "",
          "| Ring segment | Bay body | M3 | Dowels |", "|---|---|---|---|"]
    dk = {}
    for o in dat: dk.setdefault((o["ring"], o["bay"]), {"M3": 0, "dowel": 0})[("M3" if o["kind"].startswith("M3") else "dowel")] += 1
    for (r_, b_), d_ in sorted(dk.items()): L.append(f"| {r_} | {b_} | {d_['M3']} | {d_['dowel']} |")
    nd = sorted({b_.split('_')[0].replace('bay', '') for (_, b_) in dk})
    nod = [a_ for a_ in BAY_ANG if str(a_) not in nd]
    L += ["", f"Bays with a direct ring seat: {', '.join(nd)}; without one (their body reaches the ring only through a neighbour body or the links): {', '.join(str(a_) for a_ in nod) or 'none'}.",
          f"Bay-to-bay laps are kept where they still carry (M): {sum(1 for r in recs if r['a'].startswith('bay') and r['b'].startswith('bay') and r['placed'])} bolted laps between bay bodies.", ""]
    L += ["## Both joint models, all 18 cases (F)", "",
          "Bonded = every cut and lap face bonded (upper stiffness bound); bolt-disc = only an O12 disc round each bolt and each datum fixing (lower bound). Stub tilt = hypot of the X and Z rotations of the stub plate; bay rotation = worst of the 9 receiver rotations about the axle relative to the stub; residual = |reaction + applied| as % of |applied| (largest axis in brackets).", "",
          "| Case | Stub tilt bonded / bolt-disc (mrad) | Worst bay rot bonded / bolt-disc (mrad) | Bay | Residual bonded / bolt-disc % (axis max) |", "|---|---|---|---|---|"]
    for d_ in mt:
        b_, k_ = d_.get("b"), d_.get("d")
        L.append(f"| {d_['case'][:46]} | {bdl(b_ and b_['tilt'])} / {bdl(k_ and k_['tilt'])} | {bdl(b_ and b_['bay'])} / {bdl(k_ and k_['bay'])} | {(k_ or b_)['bay_at'] if k_ or b_ else ''} | {bdl(b_ and b_['resid'])} / {bdl(k_ and k_['resid'])} ({bdl(b_ and b_['axis'])} / {bdl(k_ and k_['axis'])}) |")
    L += ["", "## Cut and joints", "",
          f"- Source: `runs/box_system/v2/topo_r2/density.npz` (g3000), cubic x4 to a 2 mm grid (C), 0.5 surface (M); the drawn bodies (pads, seat posts, stub, electronics bosses, datum ring, island ribs) exact; keep-outs of round 2 (grown 1 mm, C) cut exact; features thinner than 2.3 mm (E) removed (voxel opening, M); each piece finished by a 0.5 mm voxel opening (r 1.2, C) and a smooth 0.5 iso-surface (+-0.25 mm, M). After the finish two pieces that meet flush on an oblique plane overlap by a film of up to 0.25 mm (M): the larger gives it up, the overlap list is in the System line.",
          f"- Regions (DEFAULT, planes and wedges): mast |X| < ~85, Y < 20 (C); rear frames; seat-post bridge Y > 266 (C); the datum ring segments (r 115..200, Y 74..98) before the bay bodies; 9 bay bodies (30 deg wedges, r > 140, Y > 60); top frame Z > 110 split at X 0; links = the rest. Size splits (180 cube): {'; '.join(f"{s_['piece']} on {s_['plane']} at {s_['axis']} {s_['at']:g} ({s_['section_cm2']} cm2)" for s_ in cutd.get('splits', [])) or 'see log'} (M); the Overseer bay bodies are cut between their bolt rows. ring4 is 161 x 52 x 189 mm in W and prints 176 x 176 mm turned 8 deg in its plane (M).",
          f"- Islands: a floor island (epost, 20.8 cm3) is joined to the rear frame by an O18 rib of {R['ribs'][0]['length_mm']:.0f} mm (C) drawn into the structure ({R['ribs'][0]['volume_cm3']:.1f} cm3, M), so it is one piece with `rearP_low` and no longer floats (round 1 showed umax 2.3 mm in every unilateral case)." if R['ribs'] else "- Islands: none joined.",
          f"- Joints: {len(recs)} laps ({nj} bolted: {nbolt['M3']} x M3 + {nbolt['M5']} x M5 heat-set inserts; {len(ub)} unbolted); each lap = a lug of one piece on a pocket of the other (pocket 0.3 mm (E) bigger per side; overlap 14 for M3 / 18 for M5 (C), lug 16 / 20 wide (C) with O12 / O16 pad (C) and >= 2.0 web, 6 / 8 thick (C), insert boss wall >= 4 (C)), screw access O10 x 10 clear (C). Unbolted interfaces: " + ", ".join(f"{r['a']} / {r['b']} / {r['plane']} ({r['area_cm2']} cm2, {r['wanted']} wanted)" for r in sorted(ub, key=lambda q: -q["area_cm2"])[:30]) + (f" ... {len(ub)} in all" if len(ub) > 30 else "") + ". The FE carries nothing through them; `requirements.md` lists each with the bolts to place.",
          "- Bolt count and size iterated from the joint wrenches of both models of the previous FE (M3 up to 6, else M5, index aim 0.8 (E); capacities E: heat-set pull-out 300 / 700 N, lug bearing 600 / 1300 N at SF 1):", ""]
    ch = [r for r in sizing_rows if r["now"] != r["was"]]
    L += ["| Joint | Plane | Worst bolt index (F, previous FE) | Was | Now |", "|---|---|---|---|---|"] + [f"| {r['joint'].replace(' | ', ' / ')} | {r['plane']} | {r['worst_index']:.2f} | {r['was']} | {r['now']}{' (still over)' if r['over'] else ''} |" for r in ch[:40]]
    L += ["", f"{len(ch)} of {len(sizing_rows)} joints changed (M)." if sizing_rows else "", ""]
    # joint wrenches (per lap)
    L += ["## Joint wrenches (F, largest per lap over the 18 cases and both models; every lap and case in `req/<piece>.json`)", "",
          "| Lap | Max F N (case) | Max M N.m (case) | Worst bolt index (case, model) | Placed of wanted (M) |", "|---|---|---|---|---|"]
    jw = []
    for r in recs:
        if not r["cases"]: continue
        fm = max((np.linalg.norm(d["F_N"]), cn) for cn, e in r["cases"].items() for d in e.values())
        mm = max((np.linalg.norm(d["M_Nmm"]) / 1000.0, cn) for cn, e in r["cases"].items() for d in e.values())
        jw.append((fm[0], r, fm[1], mm[0], mm[1]))
    jw.sort(key=lambda q: -q[0])
    for f_, r, cf, m_, cm_ in jw[:18]:
        L.append(f"| {r['a']} / {r['b']} / {r['plane']} | {f_:.0f} ({cf[:28]}) | {m_:.1f} ({cm_[:28]}) | {r['worst_index']:.2f} ({(r['worst_case'] or '')[:24]}, {r['worst_model']}) | {r['placed']} of {r['wanted']} x {r['size']} |")
    L += ["", f"Laps over the 0.8 bolt-index aim (E): {len(ov)} (the 'Over-aim joints' table in `requirements.md` gives the bolts and lap area each piece run must provide):", ""]
    for r in sorted(ov, key=lambda q: -q["worst_index"]):
        L.append(f"- {r['a']} / {r['b']} / {r['plane']}: index {r['worst_index']:.2f} (F; {r['worst_case']}, {r['worst_model']}) with {r['placed']} of {r['wanted']} x {r['size']} placed on {r['area_cm2']} cm2 (M): needs {r['bolts_for_aim']['M3']} x M3 or {r['bolts_for_aim']['M5']} x M5 (E).")
    L.append("")
    # floor
    L += ["## Floor (TPU feet, unilateral, active set)", "", "| Foot | On piece | x, y (M) | Area cm2 (M) | Standing N (F) | Max N (F; case) | Cases lifted (F) |", "|---|---|---|---|---|---|---|"]
    for i, f_ in enumerate(finfo["feet"]):
        fn = f"foot_{i + 1}"; mx = max(((c["foot_N"].get(fn, 0.0), cn) for cn, c in fe["cases"].items()))
        lifted = [cn for cn, c in fe["cases"].items() if fn in c["uplift"]]
        L.append(f"| {fn} | {fe['foot_owner'].get(fn, '?')} | {f_['xy'][0]:.0f}, {f_['xy'][1]:.0f} | {f_['area'] / 100:.1f} | {fe['cases']['standing, Overseer seated']['foot_N'].get(fn, 0):.1f} | {mx[0]:.0f} ({mx[1][:26]}) | {len(lifted)} |")
    up = {cn: c["uplift"] for cn, c in fe["cases"].items() if c["uplift"]}
    stand = fe["cases"]["standing, Overseer seated"]["foot_N"].values()
    L += ["", "Uplift per case (a foot with under 0.5 N in the unilateral solve; the two all-bays cases hold the feet, then it is a foot pulled down by more than 0.5 N; F): " + ("; ".join(f"{cn}: {', '.join(v)}" for cn, v in up.items()) if up else "none") + ".",
          f"At most {max((len(v) for v in up.values()), default=0)} of {len(finfo['feet'])} feet lift in any case (F). Standing, the load on the {len(finfo['feet'])} soft feet runs {min(stand):.1f} N on the least loaded to {max(stand):.1f} N on the most (F).", ""]
    L += ["## Pieces and joints over a target", ""]
    if not over: L += [f"Pieces: none. Every piece is at or under 0.5 functional / 1.0 misuse / 1.0 sustained (E) in the system FE, worst {res['worst_index']:.2f} (F); the aim for the piece runs is 0.8 (E) of those, so the pieces have slack to lose material (the topology runs set the mass).", ""]
    for k, t in over.items(): L.append(f"- {k}: {', '.join(f'{c} {v:.2f}' for c, v in t['over'].items())} (F; worst case {t['worst_case']}).")
    L += ["", "## Clearance (every piece against every keep-out)", "",
          f"- System line (true envelopes: receivers, cartridges and their paths, arm hub / motors / parked heads, both drone-insert lanes, rail disc, drone keep-out, PSU, v9 catch, sheets): {R['system_line'].split('fit ')[1].split(',')[0]}; access 9 / 9 (E, cartridge out to park), the bay bodies' +Y service withdrawal is not checked (bodies are in halves).",
          "- Round 2's keep-outs grown 1 mm (C; the finish and the lap lugs may use up to 0.25 of that margin; the true surfaces stay >= 0.75 off, M): " + "; ".join(f"{n} {v['total_mm3']:.0f} mm3 (M)" for n, v in clr.items() if v["kind"].startswith("keep-out")) + ".",
          f"- Fit limits (changed): receivers and catch seat parts are contact pairs with one fixed limit of {seat['limit_mm3']:g} mm3 each (E: a flat seat face meets its part with a film under 0.25 mm), not 1.5 x their own overlap. Real overlaps (M, mm3): " + "; ".join(f"{k_} {v_}" for k_, v_ in seat["overlaps_mm3"].items()) + ".",
          "- Fixed electronics: " + "; ".join(f"{n}: {v['total_mm3']:.0f} mm3 (M) ({', '.join(f'{k} {x:.0f}' for k, x in v['pieces'][:8]) or 'clear'})" for n, v in clr.items() if v["kind"] == "fixed electronics") + ". The sheet face strip Y -66..-50 is listed in the rear zone table below; if the strip is a hard keep-out these pieces need a re-cut (alt: boards on the strip's free area, DEFAULT).", "",
          "## Rear zone: frame members the boards sit round (M, sections of the cut at Y; PSU X 42..157 Z -170..45 and charger X -162..-44 Z -200..-29 are fixed (C); the boards are movable)", "",
          "| Y | Piece | X range | Z range | Section mm2 (M) |", "|---|---|---|---|---|"]
    for yk, rows in rz.items():
        for r_ in sorted(rows, key=lambda q: -q["area_mm2"])[:12]:
            L.append(f"| {yk} | {r_['piece']} | {r_['x'][0]}..{r_['x'][1]} | {r_['z'][0]}..{r_['z'][1]} | {r_['area_mm2']} |")
    L += ["", "Full lists: `rear_zone.json`.", "", "## Sheets (cladding clipped to the frame; clips are drawn tabs, not structure)", "",
          "Clip tab (attachment rule): post 10 x 10 (C, >= 6 x 6), 8 deep into the member (C), pad O14 x 3 (C) with an M3 insert hole O4 x 6.7 (C) (web 5 (C), >= 2.0), four triangular gussets 12 x 14 x 3 (C) at the root into the member, tip 0.2 (C) off the sheet.", ""]
    for sn_, cl in finfo["clips"].items():
        L.append(f"- {sn_}: {len(cl)} clip tabs (M3 insert): " + "; ".join(f"{c_['piece']} at {tuple(round(v) for v in c_['at'])} post {c_.get('post_len', c_.get('stalk_len'))} mm" for c_ in cl))
    L += ["", "## Findings for the system session", "",
          "- Round 2 drew the receiver bosses at the wrong place. `box_system_topo_v2r2.bosses()` translated them to z = z_c + Z_T, but z_c (0.85 / 85.8 Scout, 0.85 / 85.5 / 163.8 Overseer, C) is already the cartridge z from the closed end: the bolt rows sat Z_T (42.55 / 85.5, C) off, e.g. an Overseer row at Y 1.2, outside the receiver (Y 75..255, C). The receiver meshes show the bolt holes at Y_C - (z_c - Z_T) (O3.4: the points sit 1.70 from the hole wall, M). This cut draws its own O12 x 11 bosses at those points (C) and the FE joins every receiver to its bay body there. The round 2 field is not wrong where the pads carry the load, but its bay rotations (0.4 / 1.8 mrad, F, round 2) are not the rotations of a bolted receiver.",
          "- The field's own receiver-to-mast paths are real (the bay bodies join the mast through the links); the cut keeps them as laps and adds the datum ring.",
          "- Seat posts: the cut keeps the 8 O16 x 8 (C) drawn discs with an M3 insert hole in each (the catch's M3 is tight into it, C); the landing overlap with the v9 catch is in the fit limits above.",
          "- Bay rotation was limited by the bay bodies lapping only to neighbours or foot legs (bolt-disc bound 2.603 mrad, F, in the first cut, and motors-cut / catch -X-Y scaled to about 4.1 mrad, E, before the datum).", "",
          "## DEFAULTs taken (alternative in brackets)", "",
          "- Datum ring in 5 segments with direct bay seats (alt: a bolted hub link from each bay to the mast); Overseer bay bodies split between the bolt rows; the mast split at Z 78 and -92 (C) so the stub plate Z +-45 (C) is one piece; the top frame at Y 152 (C) (task: Y 138).",
          "- Lap = lug on pocket with 2 .. 6 bolts per interface by area, M3 first, M5 where the wrench needs it (alt: plain butt flanges); bolt capacities E (heat-set pull-out 300 / 700 N, lug bearing 600 / 1300 N), index = SF x (T / T_u + V / V_u) on a rigid pin group.",
          "- FE joints: the bonded model bonds every cut and lap face of a bolted lap (contact and clamp) and the datum fixings; the bolt-disc model bonds only an O12 disc round each bolt (the bay rotation and stub tilt are held to this bound). Unbolted laps carry nothing in either model. Receivers PETG, cartridges stiff (E 10 GPa, density from the swap mass), TPU feet E 9.8 MPa, the catch rigid on its 8 seat discs (its own FE gives its compliance), the stub plate loaded as in round 2.",
          "- The two all-bays envelope cases hold the feet (the net 164 N + 27 N.m (F) of the arm's reaction would tip the box otherwise): foot tension is reported as the hold-down needed. All other cases are unilateral.",
          "- Build axis for the strength allowables = the shortest extent of each piece (largest flat face down, E).", "",
          "## Hardware (E)", "",
          f"- Laps: {nbolt['M3']} x M3 x 10 screws into {nbolt['M3']} M3 x 5.7 heat-set inserts, {nbolt['M5']} x M5 x 16 into M5 x 9.5; datum {sum(1 for o in dat if o['kind'].startswith('M3'))} M3 + {sum(1 for o in dat if not o['kind'].startswith('M3'))} dowel seats; receivers 44 bolts into 44 M3 inserts (Scout 4 x 5, Overseer 6 x 4, bosses O12 x 11); seat posts 8 inserts; feet 8 M3 inserts + 8 screws; sheet clips 20 M3 inserts.",
          f"- Pieces with a bed contact under 400 mm2 (M) on their best face: {sum(1 for v_ in pp.values() if v_['bed_mm2'] < 400)} of {len(pp)} (the field is organic: tree supports and a brim, as the print rules say; the piece runs choose the pose).", "",
          "## Run record", "", runinfo, "",
          "## Appendix: exact solid_check and print_orient output per piece", ""]
    for k in P:
        so = os.path.join(OUT, "solid", f"{k}.txt"); po = os.path.join(OUT, "print_orient", f"{k}.txt")
        L.append(f"### {k}"); L.append("```")
        L.append(open(so).read().strip() if os.path.exists(so) else "solid_check: not run")
        L.append(open(po).read().strip() if os.path.exists(po) else "print_orient: not run")
        L.append("```")
    open(path, "w").write("\n".join(L) + "\n")


def load_cut(tag=""):
    d = json.load(open(os.path.join(OUT, f"cut{tag}.json")))
    P = {k: man(trimesh.load(os.path.join(OUT, "pieces", f"{k}.stl"), force="mesh")) for k in d["pieces"]}
    return P, d["joints"], dict(feet=d["feet"], clips=d.get("clips", {}), datum=d.get("datum", []), ribs=d.get("ribs", []), ring_cm3=d.get("ring_cm3"))


def orient_all(names=None):
    """nox/print_orient.py <mesh> --all on every piece (exact output kept in print_orient/<piece>.txt)."""
    import subprocess
    pdir = os.path.join(OUT, "pieces"); odir = os.path.join(OUT, "print_orient"); os.makedirs(odir, exist_ok=True)
    for f in sorted(os.listdir(pdir)):
        k = f[:-4]
        if not f.endswith(".stl") or (names and k not in names): continue
        r = subprocess.run([sys.executable, os.path.join(NOX, "print_orient.py"), os.path.join(pdir, f), "--all"], capture_output=True, text=True)
        open(os.path.join(odir, f"{k}.txt"), "w").write("\n".join(l for l in r.stdout.splitlines()))


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
        if not r.stdout.strip():
            log(f"solid_check {k}: no output (killed? return code {r.returncode}), not recorded"); continue
        txt = r.stdout.strip().splitlines()[-1]
        open(os.path.join(sdir, f"{k}.txt"), "w").write(r.stdout + r.stderr)
        import resource
        res[k] = dict(ok=" PASS " in txt, line=txt[:1500], sha=sha_of(os.path.join(pdir, f)), s=round(time.time() - t0),
                      peak_child_GB=round(resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1e6, 2))
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
    if a.stage == "orient":
        orient_all()
    if a.stage == "solid":
        solid_all(set(a.names.split(",")) - {""} or None, log=log)
    if a.stage == "sizes":
        fe = [json.load(open(os.path.join(OUT, f_))) for f_ in a.fe.split(",")]
        cd = json.load(open(os.path.join(OUT, "cut.json")))
        sizes, rows = size_joints(fe, cd["joints"])
        json.dump({f"{k[0]}|{k[1]}|{k[2]}": v for k, v in sizes.items()}, open(os.path.join(OUT, a.out_sizes), "w"), indent=0)
        json.dump(rows, open(os.path.join(OUT, a.out_sizes.replace(".json", "_table.json")), "w"), indent=0)
        log(f"bolt sizing: {sum(1 for r in rows if r['now'] != r['was'])} of {len(rows)} joints change; over target after M5 x 6: {[r['joint'] for r in rows if r['over']]}")
    if a.stage == "report":
        R = report_stage(a.h, a.tag, log=log)
        srows = json.load(open(os.path.join(OUT, "sizes_r1_table.json"))) if os.path.exists(os.path.join(OUT, "sizes_r1_table.json")) else []
        peaks = [l for l in open(os.path.join(OUT, "log.txt")) if "peak RSS" in l]
        slv = [v_ for v_ in R["scj"].values()]
        runinfo = (f"- Scripts: `nox/box/box_system_v2_cut.py` stages structure, export, fe (--mode all / bolts), sizes, solid, orient, report; PYPARDISO_MKL_RT=/usr/local/lib/libmkl_rt.so.3, 1e-4 diagonal regularisation as v1. Branch `run/box_system-v2-cut`.\n"
                   f"- Peak memory (M, `resource.getrusage` ru_maxrss of the process, logged at exit in `log.txt`): FE h 4 {R['fe'].get('peak_rss_GB')} GB; cut export 2.34 GB; solid_check at pitch 0.25 up to {max(v_.get('peak_child_GB', 0) for v_ in slv)} GB for the largest piece, one at a time (an earlier run was killed by the 12 GB memory cgroup when an FE ran beside it: never two together). Largest total 11.6 GB, under the 12 GB stop.\n"
                   f"- Wall (this box, 4 cores): about {R['res']['wall_time_s'] / 3600:.1f} h from the first run to the PR, of it: one cut export 7 min alone (laps 2.5 min, finish 4 min), FE h 4 18 min for the 18 cases alone (60 s each, {R['fe']['cells']} cells, {R['fe']['dofs']} dofs), solid_check {sum(v_['s'] for v_ in slv) / 60:.0f} min for the 46 files.\n"
                   f"- h {R['fe']['h']:g} used (aim 4; no coarser fallback needed). An h 8 debug run (22k cells) read the same stub tilt (0.31 against {R['sn'].get('stub_tilt_mrad')} mrad).\n"
                   f"- Iterations: FE 1 on the first cut (bolt counts by area) -> bolt sizing -> cut 2 -> FE 2 (all numbers above); after FE 2 only the finish was touched (clips and feet kept inside the sheets, pieces clipped to the sheet planes, 0.3 mm), the FE ran again on that last cut.")
        write_summary(R, srows, runinfo, os.path.join(OUT, "summary.md"))
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
    import resource, atexit
    t_start = time.time()
    atexit.register(lambda: log(f"peak RSS {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6:.2f} GB (M, ru_maxrss), children {resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1e6:.2f} GB, wall {time.time() - t_start:.0f} s, PID {os.getpid()}"))
    main()


