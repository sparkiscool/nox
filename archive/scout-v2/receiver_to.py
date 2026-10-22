"""S2 drone receiver v2: generated (noxopt run 1, 16 Oct 2026, 21:00). Follows nox/noxopt_demo.py and nox/nox-generative.md.
Receiver v1 (hand-drawn, 18.6 g M; 19.7 g M with the 20:08 shoe modules) is replaced by: the functional surfaces drawn,
everything between them generated, mounted on the drone belly plane (y 49.5).

DRAWN (passive solid, exact bodies from s2_cartridge.py, both sides):
  * 4 post plates with 30 deg lead-in feet (x locating faces 0.3 off the guide rail, capture +-3 at y 2)
  * 2 pocket blocks (pocket + 3.0 walls, latch locked 15 Oct) with their lead-in foot
  * 2 end tabs with 30 deg lead-ins (z locating faces, capture +-3 from y 38)
  * 2 shoe modules, REDUCED to the walls round the mouth, chamber, spring slot and lead hole (1.5 wall, the module's
    own lip and back wall); the solid block above the chamber (y 46.8..52) and the deck to y 52 are REMOVED: the
    receiver stops at the belly (y 49.5), the spring leg's drop channel stays open through the top face.
REMOVED from v1: top bars, end bars, spines, pocket-block posts (all generated now).
SUPPORT: the belly plane y 49.5 outside the cartridge column (DEFAULT: epoxy / screws where the generated part lands;
  the bottom layout, current-parts Problem 31, sets the real fixings).
ENVELOPE (DEFAULT): |x| <= 33 (shoe module back face), y 2..49.5, z -4.3..83.3 (end tab backs), minus the cartridge
  column, its capture envelope (+-3 in x below y 7.2, +-3 in z below y 43.2), pockets, shoe cuts.
Coords: cartridge (s2_cartridge.py), seated. Print: belly face down (build -y), no supports.
python receiver_to.py [generated cm3, default 3.0] [layout time limit s, default 3000] [--lean: noxopt_local, direct, staged | --gmg: multigrid, staged]
"""
import sys, copy, os, time, math
import numpy as np, trimesh, manifold3d as mf
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import noxopt as NO, printopt as PO
import s2_cartridge as C

H = 1.0
ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
GEN_CM3 = float(ARGS[0]) if len(ARGS) > 0 else 3.0
TLIM = float(ARGS[1]) if len(ARGS) > 1 else 3000
FLOOR, CLIP = 2.0, 0.5
TAG = "recv2"
log = lambda s: print(s[:300], flush=True)

RF = C.RAIL_FACE                                   # 20.3 locating plane
YB = C.Y_BELLY                                     # 49.5 belly / mount plane
YM = C.Y_MOUTH                                     # 2.0 lowest point
HCAP = C.CAPTURE / math.tan(C.LEAD)                # 5.2 lead-in height
ZE0, ZE1 = C.Z_END0, C.Z_END1                      # -0.3, 79.3 end locating faces
XO = C.X_MOD1                                      # 33.0 envelope half width (DEFAULT)
ZA, ZB = ZE0 - C.ENDTAB_T, ZE1 + C.ENDTAB_T        # -4.3, 83.3

# ---------------- exact bodies (manifold) ----------------
def man(t):
    m = mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)); m.merge()
    return mf.Manifold(m)
def tm(M):
    me = M.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
mbox = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def both(M): return M + M.mirror([1, 0, 0])
def hullp(pts): return mf.Manifold.hull_points(np.asarray(pts, np.float64))

pk_t, pki = C.pocket_block(); pk_cut_t = C._POCKET_CUT
zpk0, zpk1 = pki["z"][0] - C.PK_WALL, pki["z"][1] + C.PK_WALL
posts = None
for z0, z1 in ((3.0, 3.0 + C.POST_W), (C.Z_LEN - 3.0 - C.POST_W, C.Z_LEN - 3.0)):
    p = mbox(RF, RF + C.POST_T, YM + HCAP, YB, z0, z1) + man(C._lead_foot(1, z0, z1, YM))
    posts = p if posts is None else posts + p
pocket = man(pk_t) + mbox(RF, pki["x"][1], YM, pki["y"][0] + 0.01, zpk0, zpk1)
# shoe module, reduced: walls 1.5 round the chamber, the spring slot (leg height only) and the lead hole
y_ch0, y_ch1 = C.Y_SHOE - C.CH_H / 2, C.Y_SHOE + C.CH_H / 2                      # 36.7 .. 45.3
z_ch0, z_ch1 = C.ZM - C.CH_Z / 2, C.ZM + C.CH_Z / 2
WALL_S = 1.5
y_leg0, y_leg1 = C.Y_SHOE - C.PLATE_H / 2 - 0.3, C.Y_SHOE + C.PLATE_H / 2 + 0.3
modbox = mbox(C.X_MOD0, C.X_MOD1, y_ch0 - WALL_S, YB, C.MOD_Z0, C.MOD_Z1)
keep = (mbox(C.X_MOD0, C.X_PLATE + WALL_S, y_ch0 - WALL_S, y_ch1 + WALL_S, C.MOD_Z0, C.MOD_Z1)
        + mbox(C.X_PLATE - WALL_S, C.X_MOD1, y_leg0 - WALL_S, max(y_leg1, y_ch1) + WALL_S,
               C.ZM - C.PLATE_W / 2 - 0.3 - WALL_S, C.ZM + C.PLATE_W / 2 + 0.3 + WALL_S)
        + mbox(C.X_PLATE - 0.5, C.X_MOD1, C.Y_SHOE - C.WIRE_R - 0.2 - 1.2, C.Y_SHOE + C.WIRE_R + 0.2 + 1.2,
               C.ZM + C.WIRE_BACK_DZ - C.WIRE_R - 0.2 - 1.2, min(C.MOD_Z1, C.ZM + C.WIRE_BACK_DZ + C.WIRE_R + 0.2 + 1.2)))
shoe_cut = man(C.shoe_cut()) + mbox(C.X_PLATE, C.X_PLATE + C.PLATE_T + 0.2, y_leg0, YB + 2,      # drop channel through
                                     C.ZM - C.PLATE_W / 2 - 0.3, C.ZM + C.PLATE_W / 2 + 0.3)    # the belly face
shoe_mod = (modbox ^ keep) - shoe_cut
tabs = None
for zf, sg in ((ZE0, -1), (ZE1, 1)):
    pts = [(zf + sg * C.CAPTURE, C.RAIL_Y0), (zf + sg * C.ENDTAB_T, C.RAIL_Y0), (zf + sg * C.ENDTAB_T, YB), (zf, YB),
           (zf, C.RAIL_Y0 + HCAP)]
    t = hullp([[x, y, z] for z, y in pts for x in (-C.ENDTAB_X, C.ENDTAB_X)])
    tabs = t if tabs is None else tabs + t
# cuts: cartridge column, its capture envelope, pockets, shoe cuts
k30 = math.tan(C.LEAD)
cart = mbox(-RF, RF, -5, YB + 5, ZE0, ZE1)
xcap = None
for sgn in (-1, 1):
    poly = [(0, -1), (RF + C.CAPTURE + (YM + 1) * k30, -1), (RF + C.CAPTURE, YM), (RF, YM + HCAP), (0, YM + HCAP)]
    w = hullp([[sgn * x, y, z] for x, y in poly for z in (ZE0 - C.CAPTURE, ZE1 + C.CAPTURE)])
    xcap = w if xcap is None else xcap + w
zcap = None
for zf, sg in ((ZE0, -1), (ZE1, 1)):
    poly = [(zf, -5), (zf + sg * C.CAPTURE, -5), (zf + sg * C.CAPTURE, C.RAIL_Y0), (zf, C.RAIL_Y0 + HCAP)]
    w = hullp([[x, y, z] for z, y in poly for x in (-RF, RF)])
    zcap = w if zcap is None else zcap + w
cuts = cart + xcap + zcap + both(man(pk_cut_t)) + both(shoe_cut)
drawn = (both(posts + pocket + shoe_mod) + tabs) - cuts
env = mbox(-XO, XO, YM, YB, ZA, ZB)
allowed = env - cuts

# ---------------- grid and masks ----------------
XG = math.ceil(XO / H) * H                         # grid half width: whole cells, symmetric about x 0
nx, ny, nz = int(round(2 * XG / H)), int(round((YB - 1.5) / H)), int(math.ceil((ZB + 4.5) / H))
shape = (nx, ny, nz); nel = nx * ny * nz; origin = (-XG, 1.5, -4.5)
xs, ys, zs = [np.arange(n) * H + o + H / 2 for n, o in zip(shape, origin)]
so = NO._voxelise(tm(drawn), xs, ys, zs)
vo = NO._voxelise(tm(cuts), xs, ys, zs) | ~NO._voxelise(tm(env), xs, ys, zs)
so = so & so[::-1]; vo = vo | vo[::-1]; vo &= ~so                                 # exact mirror in x
solid, void = so.ravel(), vo.ravel()
design = ~solid & ~void
I3 = np.stack(np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij"), -1).reshape(-1, 3)
X, Y, Z = (I3 * H + np.asarray(origin) + H / 2).T; AX = np.abs(X)
inb = lambda v, a, b: (v > a) & (v < b)
def ab(x0, x1, y0, y1, z0, z1, ax=False): return inb(AX if ax else X, x0, x1) & inb(Y, y0, y1) & inb(Z, z0, z1)

nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
def nodes_of(mask):
    el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
    return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
dofs = lambda nd: (nd[:, None] * 3 + np.arange(3)).ravel()
def load(mask, vec):
    assert mask.any(), "empty load patch"
    nd = nodes_of(mask); F = np.zeros(ndof)
    for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
    return F
# support: top node layer (y 49.5) over the active top cells outside the cartridge column
top = (I3[:, 1] == ny - 1) & ~void & ~(ab(-RF, RF, 0, 99, ZE0, ZE1))
tn = nodes_of(top); tn = tn[(tn // (nz + 1)) % (ny + 1) == ny]
sup = dict(belly=dofs(tn))

MC = 0.165 + 0.0264                                # pack (P) + cartridge (M), kg
G = MC * 9.81
floorP = ab(RF, 24.0, 5.5, 7.5, pki["z"][0], pki["z"][1]) & solid.astype(bool)    # pocket floor cells, +x
floorM = ab(-24.0, -RF, 5.5, 7.5, pki["z"][0], pki["z"][1]) & solid.astype(bool)
POSTZ = ((3.0, 11.0), (C.Z_LEN - 11.0, C.Z_LEN - 3.0))
faceP = (ab(RF - 0.5, RF + 1.0, 38, 48, *POSTZ[0]) | ab(RF - 0.5, RF + 1.0, 38, 48, *POSTZ[1])) & solid
faceM = (ab(-RF - 1.0, -RF + 0.5, 38, 48, *POSTZ[0]) | ab(-RF - 1.0, -RF + 0.5, 38, 48, *POSTZ[1])) & solid
tabC = ab(-C.ENDTAB_X, C.ENDTAB_X, C.RAIL_Y0 + HCAP, 48, ZE0 - 1.2, ZE0) & solid      # closed-end tab face
tabL = ab(-C.ENDTAB_X, C.ENDTAB_X, C.RAIL_Y0 + HCAP, 48, ZE1, ZE1 + 1.2) & solid
for nm, mk in (("floorP", floorP), ("floorM", floorM), ("faceP", faceP), ("faceM", faceM), ("tabC", tabC), ("tabL", tabL)):
    assert mk.any(), nm
def cart_case(name, d, g, cls):
    """the seated cartridge pushed by g in direction d (what it does to the receiver; up goes into the TPU / belly)."""
    F = np.zeros(ndof); mag = G * g
    if d[1] < 0: F += load(floorP, (0, mag * d[1] / 2, 0)) + load(floorM, (0, mag * d[1] / 2, 0))
    if d[0] > 0: F += load(faceP, (mag * d[0], 0, 0))
    if d[0] < 0: F += load(faceM, (mag * d[0], 0, 0))
    if d[2] < 0: F += load(tabC, (0, 0, mag * d[2]))
    if d[2] > 0: F += load(tabL, (0, 0, mag * d[2]))
    return PO.LoadCase(name, F, cls=cls, support="belly", accel=np.asarray(d, float) * g)
cases = [cart_case("flight 4g down 2g side", np.array([2, -4, 0]) / np.sqrt(20), np.sqrt(20), "flight"),
         cart_case("landing 6g", np.array([0, -1, 0]), 6.0, "misuse")]
dirs = [np.array(v, float) for v in [(1, 0, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
dirs += [np.array((1, -1, s), float) / np.sqrt(3) for s in (1, -1)] + [np.array((1, 0, s), float) / np.sqrt(2) for s in (1, -1)]
dirs += [np.array((0, -1, s), float) / np.sqrt(2) for s in (1, -1)]
for i, d in enumerate(dirs):                                                    # x mirror: +x covers -x
    cases.append(cart_case(f"crash 25g {i}", d, 25.0, "crash"))
cases.append(PO.LoadCase("preload 12N (bolts on the pocket floors)", load(floorP, (0, -6, 0)) + load(floorM, (0, -6, 0)),
                         cls="sustained", support="belly"))
# capture: 30 deg lead-ins at a 15 N push give ~8.7 N sideways (E); each foot alone, +x side (mirror covers -x)
for nm, z0, z1 in (("post foot, closed end", *POSTZ[0]), ("post foot, lead end", *POSTZ[1]), ("pocket foot", zpk0, zpk1)):
    cases.append(PO.LoadCase(f"capture {nm} 9N", load(ab(RF, RF + 4.0, YM, YM + HCAP, z0, z1) & solid, (9, 5, 0)),
                             cls="misuse", support="belly"))
for nm, z0, z1, sz in (("closed", ZA, ZE0, -1), ("lead", ZE1, ZB, 1)):
    cases.append(PO.LoadCase(f"capture end tab {nm} 9N", load(ab(-C.ENDTAB_X, C.ENDTAB_X, C.RAIL_Y0, C.RAIL_Y0 + HCAP, z0, z1)
                                                               & solid, (0, 5, 9 * sz)), cls="misuse", support="belly"))
legz = (C.ZM - C.PLATE_W / 2, C.ZM + C.PLATE_W / 2)
cases.append(PO.LoadCase("shoe springs 2N per side", load(ab(C.X_PLATE + C.PLATE_T, C.X_MOD1, y_leg0, y_leg1, *legz) & solid, (2, 0, 0))
                         + load(ab(-C.X_MOD1, -C.X_PLATE - C.PLATE_T, y_leg0, y_leg1, *legz) & solid, (-2, 0, 0)),
                         cls="sustained", support="belly"))
cases.append(PO.LoadCase("poke shoe module 5N", load(ab(C.X_MOD1 - 1.0, C.X_MOD1, 36, 46, C.MOD_Z0, C.MOD_Z1) & solid, (-5, 0, 0)),
                         cls="misuse", support="belly"))
cases.append(PO.LoadCase("poke post foot 5N", load(ab(RF + 2.0, RF + 4.0, YM, YM + 6, *POSTZ[0]) & solid, (0, 0, 5)),
                         cls="misuse", support="belly"))

pb = PO.Problem(shape=shape, h=H, origin=origin, solid=solid, void=void, supports=sup, cases=cases,
                material=PO.MATERIALS["PETG"], build="-y", overhang=True, R=2.0)
pb0 = copy.copy(pb)
VF = GEN_CM3 * 1000 / (design.sum() * H ** 3)
drawn_g = drawn.volume() * 1.27e-3
log(f"receiver v2 grid h {H}: {shape} = {nel:,} cells, active {int((~void).sum()):,}, design {int(design.sum()):,} "
    f"({design.sum()*H**3/1000:.1f} cm3), drawn {drawn_g:.1f} g exact; cases {len(cases)}, support {len(tn)} nodes; "
    f"generated target {GEN_CM3} cm3 = volfrac {VF:.3f}")

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    half = S ^ mbox(0, 1e3, -1e3, 1e3, -1e3, 1e3)
    S = half + half.mirror([1, 0, 0])
    return ((S + drawn) ^ env) - cuts

if __name__ == "__main__":
    if "--masks" in sys.argv:
        sys.exit(0)
    t0 = time.time()
    # --lean / --gmg: noxopt_local (lean stiffness storage) so h 1.0 fits the cloud box (kill line ~6.1 GB anon, E).
    # --lean = direct solve (33 s vs 97 s multigrid here), --gmg = multigrid everywhere. With either, the run is staged:
    # layout, each prune re-settle and the build each start in a fresh process (the script re-execs itself from
    # recv2_stage.npz), because a direct solve on top of a finished layout's leftovers is over the line.
    # Delete recv2_stage.npz (and the *_ck.npz) to start over.
    LOCAL = "--gmg" in sys.argv or "--lean" in sys.argv
    PTLIM = 1800                                   # prune re-settle time limit, s (DEFAULT)
    VG = 0.3                                       # damped volume-limit refresh (noxopt vol_gain, DEFAULT): 1.0 limit-cycled here

    def finish(part, P, thin, strength):
        bodies = len(part.decompose())
        fc = NO.function_check(P, drawn, cuts, allowed, log=log)
        wr = NO.wrinkle(P, drawn=tm(drawn), allowed=tm(allowed))
        A = NO._voxelise(P, xs, ys, zs).ravel().astype(float)
        pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid); pbs.sym = None
        rows, rep = strength(pbs, np.where(pb0.solid, 1.0, A))
        print(rep)
        w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
        P.export("s2_receiver_v2_partcoords.stl")
        Q = P.copy(); Q.apply_transform(trimesh.transformations.reflection_matrix([0, 0, 0], [0, 1, 0]))   # belly face down
        Q.apply_transform(trimesh.transformations.reflection_matrix([0, 0, 0], [1, 0, 0]))               # (rotation, not a mirror)
        Q.apply_translation(-Q.bounds[0]); Q.export("s2_receiver_v2.stl")
        print(f"RESULT bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g (drawn {drawn_g:.1f}) | "
              f"function check {fc[0]:.1f}/{fc[1]:.1f}/{fc[2]:.1f} | thin {len(thin)} | wrinkle {wr[0]:.2f}/{wr[1]:.2f} | worst "
              f"{max(w['inlayer'], w['across']):.2f} in {w['case']} | {time.time()-t0:.0f} s", flush=True)

    if not LOCAL:                                  # original single-process flow (machines with room for printopt)
        L = NO.Layout(pb, VF, log=log)
        dens = L.run(checkpoint=f"{TAG}_ck.npz", resume=f"{TAG}_ck.npz", time_limit=TLIM, vol_gain=VG)
        L.save(f"{TAG}_density.npz", dens)
        L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, vol_gain=VG)
        L.save(f"{TAG}_density_pruned.npz", dens)
        for attempt in range(3):
            con = NO.connectivity(dens, L.pb0); log(f"connectivity: {con}")
            part = build(dens); P = tm(part)
            thin = NO.thin_check(P, tm(drawn), FLOOR, log=log)
            if not thin or attempt == 2: break
            L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin, vol_gain=VG)
        finish(part, P, thin, lambda p, d: NO.strength_report(p, d))
        sys.exit(0)

    import json, noxopt_local as NL
    BE = "gmg" if "--gmg" in sys.argv else "pardiso"
    ST = f"{TAG}_stage.npz"
    def save_st(**k):
        np.savez(ST + ".tmp.npz", **k); os.replace(ST + ".tmp.npz", ST)
    def again():                                   # fresh process, same arguments
        sys.stdout.flush(); sys.stderr.flush(); os.execv(sys.executable, [sys.executable] + sys.argv)

    if not os.path.exists(ST):                     # stage 1: layout
        L = NL.Layout(pb, VF, log=log, backend=BE)
        dens = L.run(checkpoint=f"{TAG}_ck.npz", resume=f"{TAG}_ck.npz", time_limit=TLIM, vol_gain=VG)
        L.save(f"{TAG}_density.npz", dens)
        save_st(x=L.sx.state, dens=dens, cut=np.zeros(nel, bool), rnd=0, built="[]", attempt=0)
        again()
    S = np.load(ST, allow_pickle=True)
    x, dens, cut = S["x"].copy(), S["dens"].copy(), S["cut"].astype(bool)
    rnd, built, attempt = int(S["rnd"]), json.loads(str(S["built"])), int(S["attempt"])
    pbc = copy.copy(pb0); pbc.void = pb0.void | cut

    if rnd < 3:                                    # stage 2: prune_thin, one round per process (as noxopt.prune_thin)
        cells, info = NO.thin_members(dens, pbc, FLOOR, CLIP + 0.25)
        if built:
            ijk = np.stack(np.unravel_index(np.arange(cells.size), pbc.shape), 1)
            cen = ijk * pbc.h + np.asarray(pbc.origin) + pbc.h / 2
            for b in built:
                cells |= (np.linalg.norm(cen - np.asarray(b["at"]), axis=1) <= b["len_mm"] / 2 + 0.5 * FLOOR) \
                    & (np.asarray(dens) > 0.5) & ~pbc.solid & ~pbc.void
            info = info + [dict(b, built=True) for b in built]
        if info:
            log(f"prune {rnd + 1}: {len(info)} thin member(s) under {FLOOR} mm as built, {int(cells.sum())} cells removed: {info}")
            cut = cut | cells; x[cells] = 0.0; pbc.void = pb0.void | cut
            L2 = NL.Layout(pbc, VF, log=log, backend=BE); L2.sx.state = x
            ck = f"{TAG}_rs{attempt}{rnd + 1}_ck.npz"
            dens = L2.run(beta0=3, checkpoint=ck, resume=ck, time_limit=PTLIM, vol_gain=VG)
            save_st(x=L2.sx.state, dens=dens, cut=cut, rnd=rnd + 1, built="[]", attempt=attempt)
            again()
    else:
        cells, info = NO.thin_members(dens, pbc, FLOOR, CLIP + 0.25)
        if info: log(f"prune: {len(info)} thin member(s) left after 3 passes: {info}")

    np.savez_compressed(f"{TAG}_density_pruned.npz", rho=x, rho_phys=dens, void=pbc.void, solid=pbc.solid)
    con = NO.connectivity(dens, pbc); log(f"connectivity: {con}")      # stage 3: build
    part = build(dens); P = tm(part)
    thin = NO.thin_check(P, tm(drawn), FLOOR, log=log)
    if thin and attempt < 2:
        save_st(x=x, dens=dens, cut=cut, rnd=0, built=json.dumps(thin), attempt=attempt + 1)
        again()
    finish(part, P, thin, lambda p, d: NL.strength_report(p, d, backend="gmg", log=log))   # one solve: multigrid, ~1 GB
