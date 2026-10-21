"""Lantern cage v12 nodes (20 Oct 2026): generative run (nox-generative.md five steps) for one printed node part.
  python node_topo2.py <node> [h]   ->  <node><tag>.stl / .json / _ck.npz
v12 changes over node_topo.py (v11):
  * Socket v2 (Spark 02:42: wider, soaks resin, slides in): 3.6 bore (0.3 epoxy gap a side on the 3.0 rod), 4 centring ribs
    0.8 wide on a 3.3 circle, two 4.0 glue-lock grooves, one 4.0 bleed channel (air and surplus epoxy back out of the mouth),
    45 deg lead-in 3.6 -> 4.8, OD 7.9 (2.0 wall at the 3.9 grooves / bleed, passes solid_check), 10 deep. Rod tip 4 from the node point as before.
  * Merged parts (Spark 02:43: nodes that touch merge, a rod too short to exist goes): corner node + post + elbow = corner
    column; tee + post + elbow = tee column (side / nose / tail); knee + 40 leg + sled nose = nose leg. Camera rails gone: the
    nose column carries the camera pads. Rule (DEFAULT): a rod with < 20 mm bare between its sockets is printed.
  * Keep-outs as cuts on every part (all were missing in v11): funnel faces 1.0 clear above z -0.8 and the 168 opening at
    +-5 deg yaw below; props swept disks + 6; box arm sweep x 42.. below z -9.5; swap head lane; receiver / cartridge; plate;
    camera board and lens window (nose column). Drawn sockets are never cut: their overlap with a keep-out is reported.
  * Supports are the rods the part hangs on (ring sockets, sled rail); the v11 sled pegs are gone (the receiver hangs on the
    sled rails at x 15 / -25, DEFAULT).
Loads: every socket not held pushed 80 N (E) in +-x +-y +-z at its outer 4 mm, class crash. Material PA12-CF (E)."""
import sys, json, copy, math, time
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO, printopt as PO

PO.MATERIALS["PA12CF"] = PO.Material("PA12CF", E=4500, Ez=0.8 * 4500, nu=0.40, Xt=90.0, Zt=36.0, S=36.0, density=1.19)  # E
MAT = PO.MATERIALS["PA12CF"]
# ---- socket v2 (DEFAULT, Spark 20 Oct 02:42) ----
SOCK = dict(L=10.0, valley=3.6, rib_d=3.3, rib_w=0.8, n_ribs=4, groove_d=3.9, groove_w=1.2, groove_at=(3.5, 7.0),
            bleed_w=1.0, bleed_d=3.9, lead_d=4.8, lead_len=0.6, OD=7.9)   # OD 7.9: 2.0 wall at the 3.9 lock rings and bleed (solid check legal minimum); 0.41 to the funnel face at the skid ring
TIP = 4.0
F_HIT = 80.0
FLOOR, CLIP = 2.0, 0.5
PROP_KEEP = 6.0          # generated node material to the swept disks (DEFAULT; rods keep 12)
log = lambda s: print(s[:300], flush=True)

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)
def P(x, y, z): return np.array([x, y, z], float)
def tm(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def box(x0, x1, y0, y1, z0, z1): return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl_z(z0, z1, r, n=48): return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([0, 0, z0])
def cone_z(z0, z1, r0, r1, n=48): return mf.Manifold.cylinder(z1 - z0, r0, r1, n).translate([0, 0, z0])

def orient(m, d, e):
    """local +z -> unit d, then translate to e."""
    d = unit(d); z = np.array([0, 0, 1.0]); v = np.cross(z, d); s = np.linalg.norm(v); c = float(z @ d)
    T = np.eye(4) if s < 1e-9 and c > 0 else trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0]) if s < 1e-9 else \
        trimesh.transformations.rotation_matrix(math.atan2(s, c), v / s)
    T = trimesh.transformations.translation_matrix(e) @ T
    return m.transform(T[:3, :4])

def socket_parts(kind, half=None, S=SOCK):
    """local frame, axis +z. 'end': socket z 0..L, mouth at L. 'through': z -h..h, mouths both ends.
    returns body (solid cylinder), cut (bore profile, ribs left standing, runs out of the mouth(s) to 80), zone (outer 4 mm)."""
    L = S["L"]; r_o = S["OD"] / 2
    if kind == "end":
        z0, z1, mouths = 0.0, L, [L]                 # rod stops on a 2.0 closed floor at z 0 (drawn below)
    else:
        h = half or L; z0, z1, mouths = -h, h, [-h, h]
    body = cyl_z(z0 - (2.0 if kind == "end" else 0.0), z1, r_o)   # end sockets: closed 2.0 floor = rod stop, epoxy sealed, no gap to the node
    rv = S["valley"] / 2
    cut = cyl_z(z0, z1, rv)
    # centring ribs stand in the bore (stop 1.0 short of the lead-in so the rod finds the bore first)
    r_in = S["rib_d"] / 2; r0 = z0 + (0.3 if kind == "end" else S["lead_len"] + 1.0); r1 = z1 - S["lead_len"] - 1.0
    for k in range(S["n_ribs"]):
        cut -= box(r_in, rv + 0.5, -S["rib_w"] / 2, S["rib_w"] / 2, r0, r1).rotate([0, 0, 90 * k])   # keyed 0.5 into the wall
    half_len = (z1 - z0) if kind == "end" else (z1 - z0) / 2
    for g in S["groove_at"]:                         # glue-lock grooves, continuous rings (cut through the ribs)
        if g > half_len - 1.5: continue
        for m in mouths:
            zc = m - g if m > 0 else m + g
            cut += cyl_z(zc - S["groove_w"] / 2, zc + S["groove_w"] / 2, S["groove_d"] / 2)
    cut += box(0, S["bleed_d"] / 2, -S["bleed_w"] / 2, S["bleed_w"] / 2, z0, z1).rotate([0, 0, 45])   # bleed channel
    path = mf.Manifold()
    for m in mouths:                                 # lead-in chamfers; the rod path out of the mouth is returned apart
        if m > 0:
            cut += cone_z(m - S["lead_len"], m + 0.01, rv, S["lead_d"] / 2); path += cyl_z(m, 80, S["lead_d"] / 2)
        else:
            cut += cone_z(m - 0.01, m + S["lead_len"], S["lead_d"] / 2, rv); path += cyl_z(-80, m, S["lead_d"] / 2)
    zone = mf.Manifold()
    for m in mouths:
        zone += cyl_z(m - 4, m, r_o + 0.6) if m > 0 else cyl_z(m, m + 4, r_o + 0.6)
    return body, cut, zone, path

# ---- rods into a part: (point in local coords where the socket starts, direction, kind, half-length for through)
def end(at, to, base=(0, 0, 0), tip=TIP):
    b = P(*base); d = unit(np.asarray(to, float) - (at + b)); return (b + d * tip, d, "end", None)
def ring_end(ex, ey, ez, d): return (P(ex, ey, ez), unit(d), "end", None)
def thru(d, base=(0, 0, 0), half=None): return (P(*base), unit(d), "through", half)

C1, C1T = P(106, 106, 33.2), P(-106, 106, 33.2)
T1, TN, TT = P(0, 112, 33.2), P(112, 0, 33.2), P(-112, 0, 33.2)
DZ = 60 - 33.2
OCT = [(112, 0), (106, 106), (0, 112), (-106, 106), (-112, 0), (-106, -106), (0, -112), (106, -106)]
HUBP = [(40, 0), (28.28, 28.28), (0, 40), (-28.28, 28.28), (-40, 0), (-28.28, -28.28), (0, -40), (28.28, -28.28)]
XN = 37.5                                            # v12 nose x (was 40: leg sockets reached the arm sweep)
KN, KT = P(XN, 32, -7), P(-66, 52, -0.8)   # v12 tail knee: swept inward until its sockets + 1.5 wall clear the 5 deg yawed rim band by 1.0       # v12 tail knee moved in (yawed opening)
SN, ST, RS = P(XN, 32, -47), P(-52, 32, -47), P(-5, 32, -47)     # v12 tail cross x -50 -> -52: a 7.6 socket at -50 reached 0.8 into the swap head body
NODES = {
 "corner_col":   dict(at=C1, rods=[ring_end(-6, 6, 0, [-1, 0, 0]), ring_end(6, -6, 0, [0, -1, 0]), ring_end(-6, 6, DZ, [-1, 0, 0]),
                      ring_end(6, -6, DZ, [0, -1, 0]), end(C1, KN), end(C1, [28.28, 28.28, 85], base=(0, 0, DZ))], held=[0, 1, 2, 3], shear=([0, 1], [2, 3]), build="+x", gen=1.8),
 "corner_col_t": dict(at=C1T, rods=[ring_end(6, 6, 0, [1, 0, 0]), ring_end(-6, -6, 0, [0, -1, 0]), ring_end(6, 6, DZ, [1, 0, 0]),
                      ring_end(-6, -6, DZ, [0, -1, 0]), end(C1T, KT), end(C1T, [-28.28, 28.28, 85], base=(0, 0, DZ))], held=[0, 1, 2, 3], shear=([0, 1], [2, 3]), build="+x", gen=1.8),
 "side_col":     dict(at=T1, rods=[thru([1, 0, 0]), thru([1, 0, 0], base=(0, 0, DZ)), end(T1, RS), end(T1, [2, 50, 24]),
                      end(T1, [0, 40, 85], base=(0, 0, DZ))], held=[0, 1], shear=([0], [1]), build="+x", gen=1.3),
 "nose_col":     dict(at=TN, rods=[thru([0, 1, 0], base=(0, 14, 0), half=6), thru([0, 1, 0], base=(0, -14, 0), half=6),
                      thru([0, 1, 0], base=(0, 0, DZ)), end(TN, [40, 0, 85], base=(0, 0, DZ))], held=[0, 1, 2], shear=([0, 1], [2]), build="+y", gen=1.0, camera=True),
 "tail_col":     dict(at=TT, rods=[thru([0, 1, 0]), thru([0, 1, 0], base=(0, 0, DZ)), end(TT, [-40, 0, 85], base=(0, 0, DZ))], held=[0, 1], shear=([0], [1]), build="+y", gen=0.8),
 "hub":          dict(at=P(0, 0, 85), rods=[(P(hx, hy, 0), unit(P(ox, oy, 60) - P(hx, hy, 85)), "end", None) for (hx, hy), (ox, oy) in zip(HUBP, OCT)],
                      held=[0, 2, 4, 6], gen=4.0, env="annulus", ring=True),
 "nose_leg":     dict(at=KN, rods=[end(KN, C1), end(KN, SN + P(-10, 0, 0), base=tuple(SN - KN))], held=[1], build="+y", gen=2.4),
 "knee_tail":    dict(at=KT, rods=[end(KT, C1T), end(KT, ST)], held=[1], gen=0.8),
 "sled_tail":    dict(at=ST, rods=[end(ST, SN), end(ST, P(-52, -32, -47)), end(ST, KT)], held=[0], gen=0.9),
 "rail_sleeve":  dict(at=RS, rods=[thru([1, 0, 0]), end(RS, T1)], held=[0], gen=0.4),
}
# nose leg: the rail socket sits at the sled end of the leg, opening to -x (toward the tail)
NODES["nose_leg"]["rods"][1] = (SN - KN + P(-TIP, 0, 0), P(-1, 0, 0), "end", None)

# ---- keep-outs in drone coords (manifold), built once
def keepouts(camera=False):
    big = 400.0
    w0 = 84.0 + 0.8 - 1.0 - 0.8                                   # half-width at z -0.8 with 1.0 clearance: 83.0
    top = 300.0
    pyr = mf.Manifold.extrude(mf.CrossSection.square([2 * w0, 2 * w0], True), top + 0.8, scale_top=[(w0 + top + 0.8) / w0] * 2).translate([0, 0, -0.8])
    sq = box(-w0, w0, -w0, w0, -big, 5.0)
    yaw = sq.rotate([0, 0, 5]) ^ sq.rotate([0, 0, -5])             # 168 opening seen by a drone yawed +-5 deg, held up to z +5 (rim band)
    band = box(-big, big, -big, big, -0.8, 5.0); below = box(-big, big, -big, big, -big, -0.8)
    inside = (pyr - band) + (yaw ^ (pyr + below))
    K = box(-big, big, -big, big, -big, big) - inside
    for mx, my in [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]:
        K += cyl_z(37.2 - PROP_KEEP, 45.2 + PROP_KEEP, 44.45 + PROP_KEEP, 96).translate([mx, my, 0])
    K += box(42, 90, -big, big, -big, -9.5)                       # box arm bars / belts / idlers
    K += box(-47, 39, -26, 14, -big, -0.8)                        # swap head body
    K += box(-45, 41, -23, 23, -44.5, 7.0)                        # receiver v4 + cartridge (DEFAULT envelope)
    K += box(-50, 50, -47, 47, 18.5, 22.0)                        # plate
    K += box(-26, 30, -26, 26, 22.0, 34.5)                        # FC stack
    if camera:
        K += box(98.5, 101.0, -19.5, 19.5, 21.5, 60.5)            # camera board
        K += box(100.6, 140, -8, 8, 33.0, 49.0)                   # lens + straight-ahead window (lens E, D18)
    return K

def geometry(nd, pad=4.0):
    at = nd["at"]; drawn = mf.Manifold(); cuts = mf.Manifold(); socks = []; zones = []
    paths = mf.Manifold()
    for e, d, kind, half in nd["rods"]:
        b, c, z, pth = socket_parts(kind, half)
        b, c, z, pth = orient(b, d, e), orient(c, d, e), orient(z, d, e), orient(pth, d, e)
        socks.append(b); zones.append(z); drawn += b; cuts += c; paths += pth
    if nd.get("ring"):
        pts = HUBP + [HUBP[0]]
        for (ax_, ay_), (bx_, by_) in zip(pts[:-1], pts[1:]):
            Lr = math.hypot(bx_ - ax_, by_ - ay_)
            seg = box(0, Lr, -2, 2, -1.5, 1.5).rotate([0, 0, math.degrees(math.atan2(by_ - ay_, bx_ - ax_))]).translate([ax_, ay_, 0])
            for e_, d_, k_, h_ in nd["rods"]:            # keep the ring out of every bore (r 1.9, inside the 1.8..3.95 wall): no coincident sheets
                seg -= orient(cyl_z(-3.0, SOCK["L"] + 1.0, 1.9), d_, e_)
            drawn += seg
    pads = mf.Manifold()
    if nd.get("camera"):                                          # camera pads: 4 lands on the board face, M2 hole each (DEFAULT, D18)
        for y in (14, -14):
            for zg in (27, 55):
                p = box(100.6, 106.0, y - 3, y + 3, zg - 2.5, zg + 2.5).translate(list(-at))
                hole = mf.Manifold.cylinder(8, 0.9, 0.9, 24).rotate([0, 90, 0]).translate([99.0, y, zg]).translate(list(-at))
                pads += p; cuts += hole
        drawn += pads
    cuts += paths - drawn                              # rod clearance out of each mouth, through free space only
    bb = tm(drawn).bounds; lo = bb[0] - pad; hi = bb[1] + pad
    env = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    if nd.get("env") == "annulus":
        ri, ro = 26.0, 54.0; zl, zh = float(lo[2]), float(hi[2])
        env = cyl_z(zl, zh, ro, 64) - cyl_z(zl - 1, zh + 1, ri, 64)
        cuts += box(lo[0] - 1, hi[0] + 1, lo[1] - 1, hi[1] + 1, zl - 1, zh + 1) - env
    K0 = keepouts(nd.get("camera", False))
    if nd.get("mirrored", True) and abs(at[1]) > 1: K0 = K0 + K0.mirror([0, 1, 0])   # part is also used mirrored about y: keep-outs on both sides (the head lane is off-centre)
    K = K0.translate(list(-at)) ^ box(lo[0] - 1, hi[0] + 1, lo[1] - 1, hi[1] + 1, lo[2] - 1, hi[2] + 1)
    drawn_in_keep = (drawn - cuts) ^ K
    cuts_all = cuts + (K - drawn)
    return dict(drawn=drawn, cuts=cuts_all, bores=cuts, env=env, socks=socks, zones=zones, box=(lo, hi), keep=K,
                drawn_in_keep=drawn_in_keep.volume())

def problem(nd, h):
    g = geometry(nd)
    lo, hi = g["box"]; ns = [int(math.ceil((b - a) / h)) for a, b in zip(lo, hi)]; shape = tuple(ns)
    ax = [np.arange(n) * h + h / 2 + a for n, a in zip(ns, lo)]
    vox = lambda m: NO._voxelise(tm(m), *ax)
    solid = vox(g["drawn"] - g["bores"]); void = vox(g["cuts"]) & ~solid
    envm = vox(g["env"]); void |= ~envm & ~solid
    S = [vox(s) for s in g["socks"]]; Z = [vox(z) & ~void for z in g["zones"]]
    nx, ny, nz = shape
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
    def nodes_of(mask):
        el = np.where(mask.ravel())[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
    dofs = lambda n_: (n_[:, None] * 3 + np.arange(3)).ravel()
    held = list(nd["held"])
    def load(mask, vec):
        n_ = nodes_of(mask); F = np.zeros(ndof)
        for a in range(3): F[3 * n_ + a] = vec[a] / len(n_)
        return F
    sups = dict(main=dofs(np.concatenate([nodes_of(S[i] & solid) for i in held])))
    cases = []
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    for i, z in enumerate(Z):
        if i in held or not z.any(): continue
        for k, d in enumerate(dirs):
            cases.append(PO.LoadCase(f"rod{i} hit {k}", load(z, F_HIT * d), cls="crash", support="main"))
    if nd.get("shear"):           # drum shear: shoulder ring sockets pushed 40 N sideways (E: the other half goes to the rings / spoke), skid ring held
        sk, top = nd["shear"]
        sups["shear"] = dofs(np.concatenate([nodes_of(S[i] & solid) for i in sk]))
        zt = np.zeros_like(Z[0])
        for i in top: zt |= Z[i]
        for k, d in enumerate(dirs[:4]):
            cases.append(PO.LoadCase(f"drum shear {k}", load(zt, 40.0 * d), cls="crash", support="shear"))
    pb = PO.Problem(shape=shape, h=h, origin=tuple(float(a[0] - h / 2) for a in ax), solid=solid.ravel(), void=void.ravel(),
                    supports=sups, cases=cases, material=MAT, overhang=False, R=max(2.0, FLOOR / h), build=nd.get("build", "+z"))
    g["ax"] = ax; g["allowed"] = g["env"] - g["cuts"]
    return pb, g

def run(name, h, gen=None, tag="", maxit=160):
    nd = dict(NODES[name]); t0 = time.time()
    if gen is not None: nd["gen"] = gen
    pb, g = problem(nd, h); pb0 = copy.copy(pb)
    design = (~pb.solid & ~pb.void).sum(); vf = nd["gen"] * 1000 / (design * h ** 3)
    log(f"{name}: grid {pb.shape} h {h}, design cells {design}, cases {len(pb.cases)}, volfrac {vf:.3f} (gen {nd['gen']} cm3), drawn in keep-outs {g['drawn_in_keep']:.2f} mm3")
    drawn, cuts, env, allowed = g["drawn"], g["cuts"], g["env"], g["allowed"]
    fin = drawn - cuts
    def build(dens):
        surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, h, pb0.origin, keep_mesh=tm(fin), clip_mesh=tm(allowed),
                                 clip_margin=CLIP, sigma_mm=0.6 * h, floor_mm=FLOOR, log=log, up=3)
        return ((man(surf) + drawn) ^ env) - cuts
    L = NO.Layout(pb, vf, log=log)
    dens = L.run(checkpoint=f"{name}{tag}_ck.npz", resume=f"{name}{tag}_ck.npz", maxit=maxit)
    L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log)
    for attempt in range(3):
        part = build(dens); comps = part.decompose(); n_bodies = len([p for p in comps if p.volume() > 1.0])
        Pmm = max(comps, key=lambda p: p.volume()); me = Pmm.to_mesh()
        Pm = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
        thin = NO.thin_check(Pm, tm(fin), FLOOR, log=log)
        if not thin or attempt == 2: break
        L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin)
    Bm = Pmm
    fc = ((fin - Bm).volume(), (Bm ^ cuts).volume() - 0.0, ((Bm - fin) ^ g["keep"]).volume())
    log(f"function check (exact): drawn missing {fc[0]:.2f} | in cuts {fc[1]:.2f} | generated in keep-outs {fc[2]:.2f} mm3")
    wr = NO.wrinkle(Pm, drawn=tm(fin), allowed=tm(allowed))
    hr = h if h <= 1.5 else 1.5
    pbr, gr = (pb0, g) if hr == h else problem(nd, hr)
    A = NO._voxelise(Pm, *gr["ax"]).ravel().astype(float)
    pbs = copy.copy(pbr); pbs.void = (A < 0.5) & ~pbr.solid; pbs.solid = np.zeros_like(pbr.solid)
    rows, _ = NO.strength_report(pbs, np.where(pbr.solid, 1.0, A))
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"])); worst = max(w["inlayer"], w["across"])
    import solid_check as SCK
    sc_ok, sc = SCK.solid_check(Pm, drawn=[])
    log(f"solid check: {'PASS' if sc_ok else 'FAIL'} {sc}")
    Pm.export(f"{name}{tag}.stl")
    out = dict(node=name, h=h, at=nd["at"].tolist(), bodies=n_bodies, watertight=bool(Pm.is_watertight),
               mass_g=round(Pm.volume / 1000 * MAT.density, 2), drawn_g=round(tm(fin).volume / 1000 * MAT.density, 2),
               function=[round(v, 2) for v in fc], drawn_in_keep=round(g["drawn_in_keep"], 2), thin=len(thin),
               wrinkle=[round(v, 2) for v in wr], worst=round(worst, 2), worst_case=w["case"], cases=len(pb.cases),
               solid=bool(sc_ok), solid_info=dict(bodies_plain=sc["bodies_plain"], bodies_eroded=sc["bodies_eroded"], thin_walls=sc["thin_walls"][:3]),
               gen_cm3=nd["gen"], minutes=round((time.time() - t0) / 60, 1), faces=int(len(Pm.faces)))
    json.dump(out, open(f"{name}{tag}.json", "w"), indent=1)
    print("RESULT", json.dumps(out), flush=True)
    return out

if __name__ == "__main__":
    run(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.25)
