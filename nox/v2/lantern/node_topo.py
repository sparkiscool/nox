"""Lantern cage v11 nodes: generative run (nox-generative.md five steps) for one node type.
  python node_topo.py <node> [h]      -> <node>.stl, <node>.json (mass, worst index, checks), <node>_ck.npz
Node coords: local = global translated to the node point (no rotation). Drawn set = rod sockets only (+ peg on sled
nodes). Cuts = rod bores running out of the envelope. Loads: every non-held socket pushed 80 N (E) in +-x +-y +-z at its
outer 4 mm, class crash; supports = the ring / receiver side. Material PA12-CF (E values, research 19 Oct)."""
import sys, json, copy, math, time
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO, printopt as PO

PO.MATERIALS["PA12CF"] = PO.Material("PA12CF", E=4500, Ez=0.8 * 4500, nu=0.40, Xt=90.0, Zt=36.0, S=36.0, density=1.19)  # E
MAT = PO.MATERIALS["PA12CF"]
ROD, BORE, OD, SOCK = 3.0, 3.15, 6.5, 10.0          # rod, bore, socket OD, socket length (DEFAULT, E)
TIP = 4.0                                           # rod tip stops this far from the node point (DEFAULT)
F_HIT = 80.0                                        # N per socket (E: 0.5 J hit on a 100 span, rod at break, pinned ends)
FLOOR, CLIP = 2.0, 0.5
log = lambda s: print(s[:300], flush=True)

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)
def P(x, y, z): return np.array([x, y, z], float)

# node point, rods: (direction from node point outward, kind, tip distance), held: indices of rods that are the support
def end(at, to, tip=TIP):      # socket for a rod that leaves the node point toward `to`: (end point rel, direction)
    d = unit(np.asarray(to, float) - at); return (d * tip, d, "end")
def ring_end(ex, ey, d):         # ring rod end sitting off the node point (clipped-square chamfer corners)
    return (P(ex, ey, 0), unit(d), "end")
def thru(d): return (P(0, 0, 0), unit(d), "through")

C1, C2, T1, T2 = P(106, 106, 33.2), P(106, 106, 60), P(0, 112, 33.2), P(112, 0, 60)
OCT = [(112, 0), (106, 106), (0, 112), (-106, 106), (-112, 0), (-106, -106), (0, -112), (106, -106)]
HUBP = [(40, 0), (28.28, 28.28), (0, 40), (-28.28, 28.28), (-40, 0), (-28.28, -28.28), (0, -40), (28.28, -28.28)]
NODES = {
 "corner":       dict(at=C1, rods=[ring_end(-6, 6, [-1, 0, 0]), ring_end(6, -6, [0, -1, 0]), end(C1, C1 + [0, 0, 10]), end(C1, [40, 32, -7])], held=[0, 1], gen=1.0),
 "corner_tail":  dict(at=P(-106, 106, 33.2), rods=[ring_end(6, 6, [1, 0, 0]), ring_end(-6, -6, [0, -1, 0]), end(P(-106, 106, 33.2), [-106, 106, 60]), end(P(-106, 106, 33.2), [-75, 64, -0.8])], held=[0, 1], gen=1.0),
 "side_tee":     dict(at=T1, rods=[thru([1, 0, 0]), end(T1, T1 + [0, 0, 10]), end(T1, [-5, 32, -47]), end(T1, [2, 50, 24])], held=[0], gen=1.0),
 "axis_tee":     dict(at=P(112, 0, 33.2), rods=[thru([0, 1, 0]), end(P(112, 0, 33.2), [112, 0, 60])], held=[0], gen=0.5),
 "elbow_corner": dict(at=C2, rods=[ring_end(-6, 6, [-1, 0, 0]), ring_end(6, -6, [0, -1, 0]), end(C2, C2 - [0, 0, 10]), end(C2, [28.28, 28.28, 85])], held=[0, 1], gen=0.8),
 "elbow_tee":    dict(at=T2, rods=[thru([0, 1, 0]), end(T2, T2 - [0, 0, 10]), end(T2, [40, 0, 85])], held=[0], gen=0.6),
 "hub":          dict(at=P(0, 0, 85), rods=[(P(hx, hy, 0), unit(P(ox, oy, 60) - P(hx, hy, 85)), "end") for (hx, hy), (ox, oy) in zip(HUBP, OCT)],
                      held=[0, 2, 4, 6], gen=2.0, env="annulus", ring=True),
 "knee_nose":    dict(at=P(40, 32, -7), rods=[end(P(40, 32, -7), [106, 106, 33.2]), end(P(40, 32, -7), [40, 32, -47])], held=[1], gen=0.5),
 "knee_tail":    dict(at=P(-75, 64, -0.8), rods=[end(P(-75, 64, -0.8), [-106, 106, 33.2]), end(P(-75, 64, -0.8), [-50, 32, -47])], held=[1], gen=0.5),
 "sled_nose":    dict(at=P(40, 32, -47), rods=[end(P(40, 32, -47), [-50, 32, -47]), end(P(40, 32, -47), [40, 32, -7])], held="peg", gen=0.5),
 "sled_tail":    dict(at=P(-50, 32, -47), rods=[end(P(-50, 32, -47), [40, 32, -47]), end(P(-50, 32, -47), [-50, -32, -47]), end(P(-50, 32, -47), [-75, 64, -0.8])], held="peg", gen=0.6),
 "rail_sleeve":  dict(at=P(-5, 32, -47), rods=[thru([1, 0, 0]), end(P(-5, 32, -47), [0, 112, 33.2])], held="peg", gen=0.5),
}

def tm(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def box(x0, x1, y0, y1, z0, z1): return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def tube_along(d, a, b, r):
    """cylinder of radius r along unit direction d from distance a to b (from the origin)."""
    L = b - a; c = mf.Manifold.cylinder(L, r, r, 40).translate([0, 0, a])
    d = np.asarray(d, float); z = np.array([0, 0, 1.0]); v = np.cross(z, d); s = np.linalg.norm(v); cth = float(z @ d)
    if s < 1e-9:
        return c if cth > 0 else c.rotate([180, 0, 0])
    ang = math.degrees(math.atan2(s, cth)); v = v / s
    # rotate about v by ang: use Rodrigues via trimesh for the mesh (manifold rotate is euler); simpler: build via trimesh
    T = trimesh.transformations.rotation_matrix(math.radians(ang), v)
    m = tm(c); m.apply_transform(T); return man(m)

def geometry(nd, pad=4.0):
    """drawn (sockets), cuts (bores), env (box), sockets list (manifold per rod), load zones per rod."""
    drawn = None; cuts = None; socks = []; zones = []; ext = 0.0
    for e, d, kind in nd["rods"]:
        sh = lambda m: m.translate([float(v) for v in e])
        if kind == "through":
            s = sh(tube_along(d, -SOCK, SOCK, OD / 2)); c = sh(tube_along(d, -80, 80, BORE / 2))
            z = sh(tube_along(d, SOCK - 4, SOCK, OD / 2 + 0.6) + tube_along(d, -SOCK, -SOCK + 4, OD / 2 + 0.6))
        else:
            s = sh(tube_along(d, 0, SOCK, OD / 2)); c = sh(tube_along(d, 0, 80, BORE / 2))
            z = sh(tube_along(d, SOCK - 4, SOCK, OD / 2 + 0.6))
        socks.append(s); zones.append(z)
        drawn = s if drawn is None else drawn + s; cuts = c if cuts is None else cuts + c
    if nd.get("ring"):   # hub: drawn octagon ring r 40 (section 4 x 3, DEFAULT) joining the socket mouths; one part by construction
        pts = HUBP + [HUBP[0]]
        for (ax_, ay_), (bx_, by_) in zip(pts[:-1], pts[1:]):
            Lr = math.hypot(bx_ - ax_, by_ - ay_); seg = box(0, Lr, -2, 2, -1.5, 1.5)
            m_ = tm(seg); m_.apply_transform(trimesh.transformations.rotation_matrix(math.atan2(by_ - ay_, bx_ - ax_), [0, 0, 1])); m_.apply_translation([ax_, ay_, 0])
            drawn = drawn + man(m_)
    peg = None
    if nd["held"] == "peg":
        peg = mf.Manifold.cylinder(8.0, 3.0, 3.0, 40).translate([0, 0, 0.0])      # into the receiver sleeve above, DEFAULT
        drawn = drawn + peg
    bb = tm(drawn).bounds; lo = bb[0] - pad; hi = bb[1] + pad
    env = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    if nd.get("env") == "annulus":   # hub: a ring envelope, the middle stays open (DEFAULT r 26..54)
        ri, ro = nd.get("r_in", 26.0), nd.get("r_out", 54.0); zl, zh = float(lo[2]), float(hi[2])
        env = mf.Manifold.cylinder(zh - zl, ro, ro, 64).translate([0, 0, zl]) - mf.Manifold.cylinder(zh - zl + 2, ri, ri, 64).translate([0, 0, zl - 1])
        cuts = cuts + (box(lo[0] - 1, hi[0] + 1, lo[1] - 1, hi[1] + 1, zl - 1, zh + 1) - env)   # outside the ring is void
    return drawn, cuts, env, socks, zones, peg, (lo, hi)

def masks(E, h, drawn, cuts, socks, zones, peg):
    lo, hi = E; ns = [int(math.ceil((b - a) / h)) for a, b in zip(lo, hi)]; shape = tuple(ns)
    ax = [np.arange(n) * h + h / 2 + a for n, a in zip(ns, lo)]
    vox = lambda m: NO._voxelise(tm(m), *ax)
    solid = vox(drawn); void = vox(cuts) & ~solid
    S = [vox(s) for s in socks]; Z = [vox(z) & ~vox(cuts) for z in zones]
    Pg = vox(peg) if peg is not None else None
    return shape, ax, solid, void, S, Z, Pg

def problem(nd, h):
    drawn, cuts, env, socks, zones, peg, E = geometry(nd)
    shape, ax, solid, void, S, Z, Pg = masks(E, h, drawn, cuts, socks, zones, peg)
    nx, ny, nz = shape; nel = nx * ny * nz
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
    def nodes_of(mask):
        el = np.where(mask.ravel())[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
    dofs = lambda nd_: (nd_[:, None] * 3 + np.arange(3)).ravel()
    held = nd["held"]
    if held == "peg": sup = dofs(nodes_of(Pg)); held_idx = []
    else: sup = dofs(np.concatenate([nodes_of(S[i]) for i in held])); held_idx = list(held)
    def load(mask, vec):
        nd_ = nodes_of(mask); F = np.zeros(ndof)
        for a in range(3): F[3 * nd_ + a] = vec[a] / len(nd_)
        return F
    cases = []
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    for i, z in enumerate(Z):
        if i in held_idx: continue
        for k, d in enumerate(dirs):
            cases.append(PO.LoadCase(f"rod{i} hit {k}", load(z, F_HIT * d), cls="crash", support="main"))
    pb = PO.Problem(shape=shape, h=h, origin=tuple(float(a[0] - h / 2) for a in ax), solid=solid.ravel(), void=void.ravel(),
                    supports=dict(main=sup), cases=cases, material=MAT, overhang=False, R=max(2.0, FLOOR / h))
    return pb, dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts, E=E, ax=ax)

def run(name, h, gen=None, tag=""):
    nd = dict(NODES[name]); t0 = time.time()
    if gen is not None: nd["gen"] = gen
    pb, g = problem(nd, h); pb0 = copy.copy(pb)
    design = (~pb.solid & ~pb.void).sum(); vf = nd["gen"] * 1000 / (design * h ** 3)
    log(f"{name}: grid {pb.shape} h {h}, design cells {design}, cases {len(pb.cases)}, volfrac {vf:.3f} (gen {nd['gen']} cm3)")
    drawn, cuts, env, allowed = g["drawn"], g["cuts"], g["env"], g["allowed"]
    def build(dens):
        surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, h, pb0.origin, keep_mesh=tm(drawn), clip_mesh=tm(allowed),
                                 clip_margin=CLIP, sigma_mm=0.6 * h, floor_mm=FLOOR, log=log, up=3)
        return ((man(surf) + drawn) ^ env) - cuts
    L = NO.Layout(pb, vf, log=log)
    dens = L.run(checkpoint=f"{name}{tag}_ck.npz", resume=f"{name}{tag}_ck.npz", maxit=160)
    L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log)
    thin = []
    for attempt in range(3):
        con = NO.connectivity(dens, L.pb0)
        part = build(dens); Pm = tm(part)
        parts = Pm.split(only_watertight=False); n_bodies = len([p for p in parts if abs(p.volume) > 1.0]); Pm = max(parts, key=lambda p: abs(p.volume))
        thin = NO.thin_check(Pm, tm(drawn - cuts), FLOOR, log=log)
        if not thin or attempt == 2: break
        L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin)
    bodies = n_bodies
    Bm = man(Pm); fc = (((drawn - cuts) - Bm).volume(), (Bm ^ cuts).volume(), (Bm - (allowed + drawn)).volume())
    log(f'function check (exact): drawn missing {fc[0]:.2f} | in cuts {fc[1]:.2f} | in keep-outs {fc[2]:.2f} mm3')
    wr = NO.wrinkle(Pm, drawn=tm(drawn - cuts), allowed=tm(allowed))
    A = NO._voxelise(Pm, *g["ax"]).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, _ = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    worst = max(w["inlayer"], w["across"])
    # one reinforce pass if over 1.0: hot cells back into the build
    Pm.export(f"{name}{tag}.stl")
    mass = Pm.volume / 1000 * MAT.density
    out = dict(node=name, h=h, at=nd["at"].tolist(), bodies=bodies, watertight=bool(Pm.is_watertight), mass_g=round(mass, 2),
               drawn_g=round(tm(drawn - cuts).volume / 1000 * MAT.density, 2), function=[round(v, 2) for v in fc], thin=len(thin),
               wrinkle=[round(v, 2) for v in wr], worst=round(worst, 2), worst_case=w["case"], connectivity=con["ok"],
               cases=len(pb.cases), design_cells=int(design), volfrac=round(vf, 4), minutes=round((time.time() - t0) / 60, 1),
               faces=int(len(Pm.faces)))
    out["gen_cm3"] = nd["gen"]; json.dump(out, open(f"{name}{tag}.json", "w"), indent=1)
    print("RESULT", json.dumps(out), flush=True)
    return out

if __name__ == "__main__":
    run(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.25)
