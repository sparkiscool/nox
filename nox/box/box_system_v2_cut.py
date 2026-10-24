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
    tm(S).export(fn)
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
            elif c.volume() >= vfold: pieces[f"{rn}_{i}"] = c
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
            log(f"  isolated component {rn}{i} ({c.volume() / 1e3:.1f} cm3) kept as a piece of its own"); pieces[f"{rn}_{i}"] = c; continue
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
SPLITS = [("mast", "Z mast 2", "z", 16.0, 14.0, ("mast_12", "mast_3")),
          ("mast_12", "Z mast 1", "z", -109.0, 14.0, ("mast_1", "mast_2")),
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


def apply_site(pieces, KEEP, itf, c, owner_is_a, spec, log=print, level=0):
    pl = itf["plane"]; a, b = itf["a"], itf["b"]
    O, Rr = (a, b) if owner_is_a else (b, a)
    lug, boss, pt, tdir = site_boxes(pl, c, owner_is_a, spec)
    Lt = spec["Lt"]
    lug_c = lug - KEEP; boss_c = boss - KEEP
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


def laps(pieces, planes, KEEP, sizes=None, log=print, amin=60.0):
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
            r, why = apply_site(pieces, KEEP, itf, c, owner_a, spec, log, level)
            if r is None:
                r, why2 = apply_site(pieces, KEEP, itf, c, not owner_a, spec, log, level)
                if r is None:
                    key = re.sub(r"(blocked by|piece|falls?|would fall|in the way)\s*\S*", "", why2).strip() or why2
                    key = re.sub(r"^\S+ would", "piece would", why2) if "would" in why2 else re.sub(r"(piece|by) \S+", r"\1 X", why2)
                    rej[key] = rej.get(key, 0) + 1; rj[key] = rj.get(key, 0) + 1; continue
            r['relaxed'] = level; got.append((c, r))
        joints.append(dict(a=itf["a"], b=itf["b"], plane=itf["plane"].name, area=itf["area"], want=nn, size=sz, sites=[r for _, r in got]))
        log(f"  lap {itf['a']} | {itf['b']} on {itf['plane'].name}: {itf['area'] / 100:.1f} cm2, {len(got)}/{nn} {sz} sites ({len(cand)} candidates){' rejected ' + str(rj) if len(got) < nn else ''}")
    log(f"  rejected candidates: {rej}")
    return joints


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--stage", default="structure")
    a = ap.parse_args()
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
        S = structure(); out, pl = partition(S)
        P = fold(out, pl, log=log); P = size_splits(P, pl, log=log)
        D = drawn_inputs(); KEEP = D["cuts"] - D["drawn"]
        t0 = time.time(); J = laps(P, pl, KEEP, log=log); log(f"laps {time.time() - t0:.0f} s")
        os.makedirs(os.path.join(OUT, "pieces_wip"), exist_ok=True)
        for k, m in P.items():
            b = bbox(m); log(f"{k}: {m.volume() / 1e3:.1f} cm3 ext {np.round(b[1] - b[0]).tolist()} lo {np.round(b[0]).tolist()} comps {[round(c.volume(), 1) for c in comps(m)][:4]}")
            tm(m).export(os.path.join(CACHE, f"wip_{k}.stl"))
        json.dump(J, open(os.path.join(CACHE, "wip_joints.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()


