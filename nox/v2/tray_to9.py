"""S2 cartridge tray run 9 (17 Oct 2026, Spark 11:32 "run this through the optimization"). Copy of tray_to5.py (runs 5..8).
Run 9 changes:
  * FLIMSY (Spark 10:45): the band under the floor (|x| < 17, y 0..12, z 2..77, outside the block) is design space
    (one variable per mirrored cell), so the optimizer can close the lower cell; new cases: hand twist 0.2 N.m about
    the pack axis (+T closed end, -T lead end, E) and a 10 N squeeze on both top bands mid-length (E).
  * FEELER BLADES (s2_blade.py, Spark 11:07): two root posts drawn (solid); each blade's swept path over the squeeze
    (+0.1 running clearance) and its drop-in path to the bottom face are keep-outs; new case: the blade root reactions
    at full squeeze (2.93 N at the tip -> 21.3 N / 18.4 N couple on the slot walls, flight class).
  * v9 lead channels (tray_v9_leads.py) cut into the drawn top band.
python tray_to9.py [generated cm3, default 3.6] [layout time limit s, default 4200]
Coords: cartridge (s2_cartridge.py): x slide axis, y up (0 = flat bottom), z along the pack (0 = closed end)."""
import sys, copy, os, time
import numpy as np, trimesh, manifold3d as mf
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import noxopt as NO, printopt as PO
import s2_cartridge as C, s2_blade as SB

H = 1.0
ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
GEN_CM3 = float(ARGS[0]) if len(ARGS) > 0 else 3.6
TLIM = float(ARGS[1]) if len(ARGS) > 1 else 4200
FLOOR, CLIP = 1.2, 0.0
TAG = os.environ.get("TAG", "tray9")
log = lambda s: print(s[:300], flush=True)

XC, XW, XR = C.CAV_X, C.XW_OUT, C.BOSS_X1
YT, ZL = C.Y_TOP, C.Z_LEN
EW = C.END_W
YF0, YF1 = C.Y_FLOOR0, C.Y_FLOOR1
RY0 = C.RAIL_Y0
BOT = 4.0
ZB0, ZB1 = C.Z_BLK0 - C.BLK_CLR, C.Z_BLK1 + C.BLK_CLR

XH = 21.0; nx, ny, nz = int(2 * XH / H), int(YT / H), int(ZL / H)
shape = (nx, ny, nz); nel = nx * ny * nz; origin = (-XH, 0.0, 0.0)
I3 = np.stack(np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij"), -1).reshape(-1, 3)
X, Y, Z = (I3 * H + np.asarray(origin) + H / 2).T; AX = np.abs(X)
inb = lambda v, a, b: (v > a) & (v < b)
def ab(a0, a1, y0, y1, z0, z1): return inb(AX, a0, a1) & inb(Y, y0, y1) & inb(Z, z0, z1)
CEN = np.stack([X, Y, Z], 1)

# ---- exact blade bodies (trimesh) -> cell masks
blade_keep = trimesh.util.concatenate([SB.sweep(1, SB.CL + 0.6), SB.sweep(-1, SB.CL + 0.6)])   # layout keep-out (+0.6 for the 1 mm grid)
drop = []
for s in (1, -1):                                       # drop-in path: the rest blade extruded down to below the bottom face
    b = SB.blade(SB.D0, s, dil=SB.CL + 0.6, tip_exact=False); bb = b.bounds
    drop.append(C.B(bb[0][0], bb[1][0], -2, SB.Y_B0, bb[0][2], bb[1][2]))
blade_keep = trimesh.util.concatenate([blade_keep] + drop)
bladeV = blade_keep.contains(CEN) | blade_keep.contains(CEN * [-1, 1, 1])
posts_m = SB.tray_add()
postV = posts_m.contains(CEN) & posts_m.contains(CEN * [-1, 1, 1])

# ---- the five plates + the under-floor band
longw = ab(XC, XW, 0, YT, 0, ZL)
endw = ab(0, XC, 0, YT, 0, EW) | ab(0, XC, 0, YT, ZL - EW, ZL)
floor = ab(0, XC, YF0, YF1, EW, ZL - EW)
ground = ab(0, XW, 0, C.Y_BLK1, ZB0, ZB1)
under = ab(0, XC, 0, YF0, EW, ZL - EW) & ~ground
topband = ab(XC, XR, RY0, YT, 0, ZL)
endtop = ab(0, XC, RY0, YT, 0, EW) | ab(0, XC, RY0, YT, ZL - EW, ZL)
botband = ab(XC, XW, 0, BOT, 0, ZL) | ab(0, XC, 0, BOT, 0, EW) | ab(0, XC, 0, BOT, ZL - EW, ZL)
posts = ab(XC, XW, 0, YF1 + 2, ZB0 - 3, ZB0) | ab(XC, XW, 0, YF1 + 2, ZB1, ZB1 + 3) | ab(XC, XW, YF1, YF1 + 2, ZB0, ZB1)
PZ = (C.Z_PACK0 + 3.0, C.Z_PACK1 - 3.0)
fpads = np.zeros(nel, bool); wpads = np.zeros(nel, bool)
for zc in PZ:
    fpads |= ab(XC - 6, XC, YF0, YF1, zc - 3, zc + 3)
    wpads |= ab(XC, XW, 26, 32, zc - 3, zc + 3)
epads = ab(XC - 6, XC, 26, 32, 0, EW)
stop = ab(0, XC, YF1, YF1 + 4, C.Z_PACK1, C.Z_PACK1 + 2)
solid = ground | topband | endtop | botband | posts | fpads | wpads | epads | stop | postV
rail_out = ab(XW, XR, RY0, YT, 0, ZL)
strip_rec = ab(XR - C.STRIP_REC, XR, C.Y_BAND0, C.Y_BAND1, C.Z_STRIP0, C.Z_STRIP1)
domain = longw | endw | floor | ground | rail_out | stop | under | postV
void = ~domain | (strip_rec & ~ground) | bladeV
solid &= ~void
design = ~solid & ~void
so, vo = solid.reshape(shape), void.reshape(shape)
assert np.array_equal(so, so[::-1]) and np.array_equal(vo, vo[::-1]), "masks must mirror in x"

gid = -np.ones(nel, np.int64)
i_abs = np.where(X > 0, I3[:, 0], nx - 1 - I3[:, 0])
j, k = I3[:, 1], I3[:, 2]
m = design & longw;  gid[m] = 0 * 10**7 + j[m] * nz + k[m]
m = design & endw & (Z < ZL / 2); gid[m] = 1 * 10**7 + i_abs[m] * ny + j[m]
m = design & endw & (Z > ZL / 2); gid[m] = 2 * 10**7 + i_abs[m] * ny + j[m]
m = design & floor; gid[m] = 3 * 10**7 + i_abs[m] * nz + k[m]
m = design & under; gid[m] = 4 * 10**7 + (i_abs[m] * ny + j[m]) * nz + k[m]     # 3D cells, mirrored only
assert (gid[design] >= 0).all(), "every design cell must sit in a group"
uq, inv = np.unique(gid[design], return_inverse=True)
cnt = np.bincount(inv).astype(float)
didx = np.where(design)[0]
def sym(v):
    v = np.asarray(v, float).copy()
    v[didx] = (np.bincount(inv, weights=v[didx]) / cnt)[inv]
    return v

nid = lambda i, jj, kk: (i * (ny + 1) + jj) * (nz + 1) + kk
ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
def nodes_of(mask):
    el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
    return np.unique(np.concatenate([nid(ex + a, ey + b, ez + cc) for a in (0, 1) for b in (0, 1) for cc in (0, 1)]))
dofs = lambda nd: (nd[:, None] * 3 + np.arange(3)).ravel()
def load(mask, vec):
    nd = nodes_of(mask); F = np.zeros(ndof)
    for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
    return F
def node_xyz(nd):
    i = nd // ((ny + 1) * (nz + 1)); r = nd % ((ny + 1) * (nz + 1)); jj = r // (nz + 1); kk = r % (nz + 1)
    return np.stack([i, jj, kk], 1) * H + np.asarray(origin)
def torque(mask, T, yc=C.Y_FLOOR1 + C.PACK_Y / 2):
    nd = nodes_of(mask); p = node_xyz(nd); dx, dy = p[:, 0], p[:, 1] - yc
    kq = T / (dx ** 2 + dy ** 2).sum(); F = np.zeros(ndof)
    F[3 * nd] = -kq * dy; F[3 * nd + 1] = kq * dx
    return F
bay_hold = ab(XW, XR, RY0, RY0 + 5, 0, ZL) | ab(XC, XW, 0, 1.0, 0, ZL)
sup = dict(block=dofs(nodes_of(ground)), bay=dofs(nodes_of(bay_hold)))
G = 0.165 * 9.81
floor_top = fpads | (posts & inb(Y, YF1, YF1 + 1))
wallP, wallM = wpads & (X > 0), wpads & (X < 0)
lips = ab(XC - 1, XW, YT - 1, YT, 4, C.Z_BOSS0)
def pack_case(name, d, g, cls, support="block"):
    F = np.zeros(ndof); mag = G * g
    if d[0] > 0: F += load(wallP, (mag * d[0], 0, 0))
    if d[0] < 0: F += load(wallM, (mag * d[0], 0, 0))
    if d[1] < 0: F += load(floor_top, (0, mag * d[1], 0))
    if d[1] > 0: F += load(lips, (0, mag * d[1], 0))
    if d[2] > 0: F += load(stop, (0, 0, mag * d[2]))
    if d[2] < 0: F += load(epads, (0, 0, mag * d[2]))
    return PO.LoadCase(name, F, cls=cls, support=support, accel=np.asarray(d, float) * g)
cases = [pack_case("flight 4g down 2g side", np.array([2, -4, 0]) / np.sqrt(20), np.sqrt(20), "flight"),
         pack_case("landing 6g", np.array([0, -1, 0]), 6.0, "misuse"),
         pack_case("inverted 6g (lips)", np.array([0, 1, 0]), 6.0, "misuse")]
dirs = [np.array(v, float) for v in [(1, 0, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
dirs += [np.array((1, -1, sz), float) / np.sqrt(3) for sz in (1, -1)] + [np.array((1, 0, sz), float) / np.sqrt(2) for sz in (1, -1)]
for kk, d in enumerate(dirs):
    cases.append(pack_case(f"crash 25g {kk}", d, 25.0, "crash"))
topface = ab(XC, XR, YT - 1, YT, 0, ZL) | ab(0, XC, YT - 1, YT, 0, EW) | ab(0, XC, YT - 1, YT, ZL - EW, ZL)
cases.append(PO.LoadCase("preload 12N", load(topface, (0, -12.0, 0)), cls="sustained", support="block"))
shoes = lambda s: (X * s > 0) & ab(XR - 1, XR, C.Y_BAND0, C.Y_BAND1, C.Z_STRIP0, C.Z_STRIP1)
cases.append(PO.LoadCase("shoes 2N per side", load(shoes(1), (-2, 0, 0)) + load(shoes(-1), (2, 0, 0)), cls="sustained", support="block"))
cap = lambda z0, z1: (X > 0) & ab(XW, XR, RY0, RY0 + 5, z0, z1)
cases.append(PO.LoadCase("capture side, closed end 9N", load(cap(0, 12), (-9, 0, 0)), cls="misuse", support="block"))
cases.append(PO.LoadCase("capture side, lead end 9N", load(cap(ZL - 12, ZL), (-9, 0, 0)), cls="misuse", support="block"))
cases.append(PO.LoadCase("capture end, closed 9N", load(ab(0, XC, RY0, RY0 + 5, 0, EW), (0, 0, 9)), cls="misuse", support="block"))
cases.append(PO.LoadCase("capture end, lead 9N", load(ab(0, XC, RY0, RY0 + 5, ZL - EW, ZL), (0, 0, -9)), cls="misuse", support="block"))
pinch = lambda s: (X * s > 0) & ab(XW - 1, XW, 20, 30, 45, 55)
cases.append(PO.LoadCase("pinch 20N", load(pinch(1), (-20, 0, 0)) + load(pinch(-1), (20, 0, 0)), cls="misuse", support="block"))
cases.append(PO.LoadCase("poke end 5N", load(ab(0, 8, 18, 30, 0, 1), (0, 0, 5)), cls="misuse", support="block"))
# run 9: torsion, squeeze, blades
endC = solid & ~ground & (Z < 4); endL = solid & ~ground & (Z > ZL - 4)
TW = 200.0                                                                     # N.mm, hand twist (E)
cases.append(PO.LoadCase("hand twist 0.2 N.m", torque(endC, TW) + torque(endL, -TW), cls="misuse", support="block"))
sq = lambda s: (X * s > 0) & ab(XC, XR, RY0 + 5, YT, 29.5, 49.5)
cases.append(PO.LoadCase("squeeze top bands 10N", load(sq(1), (-10, 0, 0)) + load(sq(-1), (10, 0, 0)), cls="misuse", support="block"))
mouth = lambda s: postV & (X * s > 0) & (Z < SB.Z_ROOT + 1.5)
bottom = lambda s: postV & (X * s > 0) & (Z > SB.Z_ROOT + SB.CLAMP - 1.5)
FT = SB.K * (SB.D0 + C.TRAVEL); F2 = -FT * (SB.Z_ROOT - SB.Z_TIP) / (SB.CLAMP - 1.0); F1 = FT - F2   # on the blade
cases.append(PO.LoadCase("blades at full squeeze", load(mouth(1), (-F1, 0, 0)) + load(bottom(1), (-F2, 0, 0))
                         + load(mouth(-1), (F1, 0, 0)) + load(bottom(-1), (F2, 0, 0)), cls="flight", support="block"))
cases.append(PO.LoadCase("bay: head pulls 10N against the lips", load(ground & inb(Y, 0, 2), (0, -10, 0)), cls="misuse", support="bay"))
cases.append(pack_case("bay: pack 6g toward the axle", np.array([0, -1, 0]), 6.0, "misuse", support="bay"))
cases.append(pack_case("bay: pack 6g sideways", np.array([1, 0, 0]), 6.0, "misuse", support="bay"))

pb = PO.Problem(shape=shape, h=H, origin=origin, solid=solid, void=void, supports=sup, cases=cases,
                material=PO.MATERIALS["PETG"], build="+z", overhang=True, R=2.5, comp_factor=None, sym=sym)
pb0 = copy.copy(pb)
VF = GEN_CM3 * 1000 / (design.sum() * H ** 3)
log(f"tray run 9 grid h {H}: {shape} = {nel:,} cells, active {int((~void).sum()):,}, design {int(design.sum()):,} "
    f"({design.sum()*H**3/1000:.1f} cm3; under-floor {int((design & under).sum())}) in {len(uq):,} variables, "
    f"blade keep-out {int(bladeV.sum())} cells, posts {int(postV.sum())} cells, cases {len(cases)}; blade root couple "
    f"{F1:.1f} / {F2:.1f} N; generated target {GEN_CM3} cm3 = volfrac {VF:.3f}")

# ---- exact bodies
mbox = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def both(a0, a1, y0, y1, z0, z1): return mbox(a0, a1, y0, y1, z0, z1) + mbox(-a1, -a0, y0, y1, z0, z1)
def tm(m_):
    me = m_.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def mprism(xy, z0, z1, flip=False):
    pts = np.array([[x, y, z] for x, y in xy for z in (z0, z1)], np.float64)
    if flip: pts[:, 0] *= -1
    return mf.Manifold.hull_points(pts)
cham = mprism([(XR - C.CHAMF_X, YT + 0.01), (XR + 1, YT + 0.01), (XR + 1, YT - C.CHAMF_Y)], -1, ZL + 1)
cham = cham + mprism([(XR - C.CHAMF_X, YT + 0.01), (XR + 1, YT + 0.01), (XR + 1, YT - C.CHAMF_Y)], -1, ZL + 1, flip=True)
RAMP = [(XR - 1.5, C.Y_BAND1), (C.X_STRIP_FACE, C.Y_BAND1), (XR - 0.8, C.Y_BAND1 + 2.0)]
ramp = mprism(RAMP, C.Z_STRIP0, C.Z_STRIP1) + mprism(RAMP, C.Z_STRIP0, C.Z_STRIP1, flip=True)
lipm = mprism([(XC, C.Y_CAV1 - C.LIP), (XC - C.LIP, C.Y_CAV1), (XC - C.LIP, YT), (XC, YT)], 4, C.Z_BOSS0)
lipm = lipm + mprism([(XC, C.Y_CAV1 - C.LIP), (XC - C.LIP, C.Y_CAV1), (XC - C.LIP, YT), (XC, YT)], 4, C.Z_BOSS0, flip=True)
pads_m = None
for zc in PZ:
    for b in (both(XC - 6, XC, YF0, YF1, zc - 3, zc + 3), both(XC, XW, 26, 32, zc - 3, zc + 3)):
        pads_m = b if pads_m is None else pads_m + b
drawn = (both(XC, XR, RY0, YT, 0, ZL)
         + mbox(-XC, XC, RY0, YT, 0, EW) + mbox(-XC, XC, RY0, YT, ZL - EW, ZL)
         + both(XC, XW, 0, BOT, 0, ZL) + mbox(-XC, XC, 0, BOT, 0, EW) + mbox(-XC, XC, 0, BOT, ZL - EW, ZL)
         + both(XC, XW, 0, YF1 + 2, ZB0 - 3, ZB0) + both(XC, XW, 0, YF1 + 2, ZB1, ZB1 + 3) + both(XC, XW, YF1, YF1 + 2, ZB0, ZB1)
         + pads_m + both(XC - 6, XC, 26, 32, 0, EW)
         + mbox(-XC, XC, YF1, YF1 + 4, C.Z_PACK1, C.Z_PACK1 + 2) + lipm + man(posts_m)) - cham + ramp
stopm = mbox(-XC, XC, YF1, YF1 + 4, C.Z_PACK1, C.Z_PACK1 + 2)
# v9 lead channels (tray_v9_leads.py)
CH = dict(x=(16.5, 18.6), y=(43.3, 45.7), z=(C.WIN_Z0, C.Z_LEN - C.END_W))
LINK = dict(x=(16.5, 18.5), y=(C.WIN_Y1, CH["y"][0] + 0.05), z=(C.WIN_Z0, C.WIN_Z1))
leads = both(CH["x"][0], CH["x"][1], *CH["y"], *CH["z"]) + both(LINK["x"][0], LINK["x"][1], *LINK["y"], *LINK["z"])
bl_exact = man(SB.tray_cut())
for s in (1, -1):
    b = SB.blade(SB.D0, s, dil=SB.CL, tip_exact=False); bb = b.bounds
    bl_exact = bl_exact + mbox(bb[0][0], bb[1][0], -2, SB.Y_B0 + 0.5, bb[0][2], bb[1][2])
cuts = ((mbox(-XC, XC, YF1, YT + 5, EW, ZL - EW) - stopm - lipm)
        + mbox(-XW - 2, XW + 2, -1, C.Y_BLK1, ZB0, ZB1)
        + both(XR - C.STRIP_REC, XR + 5, C.Y_BAND0, C.Y_BAND1, C.Z_STRIP0, C.Z_STRIP1)
        + both(XC - 1, XR - C.STRIP_REC + 0.01, C.WIN_Y0, C.WIN_Y1, C.WIN_Z0, C.WIN_Z1)
        + (cham - ramp) + leads + bl_exact)
env = mbox(-XW, XW, 0, YT, 0, ZL) + both(XW - 0.01, XR, RY0, YT, 0, ZL) + ramp
allowed = env - cuts

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn - cuts),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    half = S ^ mbox(0, 1e3, -1e3, 1e3, -1e3, 1e3)
    S = half + half.mirror([1, 0, 0])
    return ((S + drawn) ^ env) - cuts

def print_check(P, pitch=0.25, shift=0.04):
    from scipy import ndimage as ndi
    axp = [np.arange(P.bounds[0][q] - 1 + shift, P.bounds[1][q] + 1, pitch) for q in range(3)]
    Mp = NO._voxelise(P, *axp); Ep = ndi.distance_transform_edt(Mp) * pitch
    return ndi.label(Mp)[1], ndi.label(Ep > FLOOR / 2)[1]

def stiffness_of(A, names=("hand twist 0.2 N.m", "squeeze top bands 10N", "pinch 20N")):
    """compliance of the built part (voxelised density A) for the named cases -> twist angle, squeeze travel."""
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = pb0.solid; pbs.sym = None; pbs.overhang = False
    pbs.cases = [c for c in pb0.cases if c.name in names]; pbs.R = 0.5
    po = PO.PrintOpt(pbs); po.evaluate(np.where(pbs.solid, 1.0, (A > 0.5).astype(float)), 64.0, grads=False, _ref=True)
    out = {}
    for c, val in zip(pbs.cases, po._comp):
        if "twist" in c.name: out["twist end-to-end deg at 0.2 N.m"] = np.degrees(val / TW)
        elif "squeeze" in c.name: out["squeeze travel per side mm at 10 N"] = val / 20.0
        elif "pinch" in c.name: out["pinch travel per side mm at 20 N"] = val / 40.0
    return out

if __name__ == "__main__":
    t0 = time.time()
    if "--setup" in sys.argv: sys.exit(0)
    L = NO.Layout(pb, VF, log=log)
    dens = L.run(checkpoint=f"{TAG}_ck.npz", resume=f"{TAG}_ck.npz", time_limit=TLIM)
    L.save(f"{TAG}_density.npz", dens)
    con = NO.connectivity(dens, pb0); log(f"connectivity: {con}")
    part = build(dens); P = tm(part)
    P.export(f"s2_tray_{TAG}.stl")
    pc = print_check(P)
    log(f"print check: bodies at 0.25 mm {pc[0]}, with everything under {FLOOR} mm removed {pc[1]}  " + ("OK" if pc == (1, 1) else "FAIL"))
    thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
    fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    A = NO._voxelise(P, *[np.arange(n) * H + o + H / 2 for n, o in zip(pb0.shape, origin)]).ravel().astype(float)
    np.save(f"{TAG}_built_vox.npy", A)
    st = stiffness_of(A); log(f"stiffness as built: {st}")
    Au = A.copy(); Au[under & ~pb0.solid] = 0.0
    log(f"stiffness without the under-floor material: {stiffness_of(Au)}")
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid); pbs.sym = None
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
    print(rep)
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    print(f"RESULT print bodies {pc} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g (vol {P.volume/1000:.2f} cm3) | "
          f"function check {fc[0]:.1f}/{fc[1]:.1f}/{fc[2]:.1f} | thin {len(thin)} | worst "
          f"{max(w['inlayer'], w['across']):.2f} in {w['case']} | {time.time()-t0:.0f} s", flush=True)
