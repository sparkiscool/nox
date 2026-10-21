"""S2 receiver v3 (Scout side), 17 Oct 2026. One generated PETG part on the three plate bolts.
Spark 17 Oct 02:23: the Scout-side receiver fits the ring's open (drone) slot like any other charging bay, and mounts
on the three holes in the carbon plate. 02:28: the funnel is lowered so the drone's pack sits where a bay's does.
Base: receiver_to_shoe.py (contacts chat, 17 Oct 01:10). Follows nox/noxopt_demo.py, whole pipeline.

Bay match (C unless tagged):
  * cartridge interface = the bay's: 30 deg lead-ins (capture CAPTURE), locating faces 0.3 off the guide rails and end
    faces, the two latch pockets, the two copper shoe modules (s2_cartridge.py, same functions as the bay modules).
  * seated pack's flat bottom at ring r 100 = R (arm_layout.py), as in every bay. Pack bottom at drone z -44.5 (E):
    the highest pack the lowered funnel allows (funnel back ribs 1 mm off the neighbour bays, E), so seat z 0 = W Z 144.5.
  * the part stays inside the open slot's sector (+-15 deg, modules stop 0.3 short): every point 1.3 (C) inside the
    slot planes (W X = -x, W Z = 100 + y).
Drone placement: pack long side nose-tail (drone x = Z_C - z), drone y = x, drone z = y + DZ. FC is in the top case
now, so nothing else is under the plate except the flow sensor, which this part carries (current-parts Problem 31).
Flow sensor MTF-02P (DEFAULT, placement search 17 Oct): board centred on the axis at the nose under the nose cup,
lens at drone (63, -2, -5), lens to the nose (SENS_FLOW_ROT unchanged); 23 deg view half-angle (42 deg FOV + 2) clears
the pack corner by 8.2 (C). Plug out the tail end into its own pocket.
Coords below are CARTRIDGE coords.   python receiver_v3.py [generated cm3, default 6.0] [layout time limit s]
"""
import sys, copy, os, time, math
import numpy as np, trimesh, manifold3d as mf
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import noxopt as NO, printopt as PO
import s2_cartridge as C

H = 1.5
GEN_CM3 = float(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1][0].isdigit() else 6.0
TLIM = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2][0].isdigit() else 5400
FLOOR, CLIP = 2.0, 0.5
TAG = os.environ.get("TAG", "rcv3")
log = lambda s: print(s[:300], flush=True)

# ---- placement (drone <- cartridge)
ZC = C.Z_C                                   # 37.5: pack centre at drone x 0 (= the funnel centre = ring tab line Y_C)
DZ = -44.5                                   # pack bottom at drone z -44.5 (E, see header)
R_RING = 100.0                               # ring radius = seated cartridge's flat bottom (C, arm_layout.R)
SLOT_HALF = math.radians(15.0)               # open slot sector half angle (pitch 30, arm_layout v4c)
SLOT_CLR = 0.3 + 1.0                         # modules stop 0.3 short of the plane + 1.0 clearance (DEFAULT)
STRIP, DECK_T = 1.5, 2.0
Y_DECK0 = C.Y_TOP + STRIP                    # 49.5 deck underside (TPU strip on the rail tops)
Y_DECK1 = Y_DECK0 + DECK_T                   # 51.5
Y_PLATE = 18.5 - DZ                          # 63.0 plate underside (3.5 carbon, D8)
PAD_R, PAD_T, BOLT_R, NUT_R, NUT_H = 7.5, 2.5, 1.65, 5.5, 6.0   # pad 7.5 > the nyloc + wrench space 5.5 (Ø9.5 pads floated)
BOLTS = [(31.0, ZC - 3.9), (-31.0, ZC - 3.9), (0.0, ZC - 45.0)]   # (x_cart, z_cart): sides (3.9, +-31), nose (45, 0)

# flow sensor (drone -> cart: z = ZC - x_d, y = z_d - DZ, x = y_d)
FS_L, FS_W, FS_T = 22.0, 16.0, 8.0           # board 22 x 16, 8 tall over the lens (P)
FS_LENS_X, FS_LENS_Y, FS_LENS_Z = 63.0, -2.0, -5.0   # lens (drone, DEFAULT): 6 from the nose end, 6 from the right edge
FS_NOSE = FS_LENS_X + 6.0                    # board nose end, drone x 69
FS_Z0 = ZC - FS_NOSE                         # board z -31.5 .. -9.5 (cart)
FS_Z1 = FS_Z0 + FS_L
FS_Y0 = FS_LENS_Z - DZ                       # 39.5 lens face
FS_Y1 = FS_Y0 + FS_T                         # 47.5 board top = tape face
FS_LAND = 2.0
FS_RIM, FS_RIM_H, FS_CLR = 1.2, 3.0, 0.3
FS_HALF_ANG = math.radians(23.0)             # MicoAir FOV 42 deg + 2 per side
LENS = (FS_LENS_Y, FS_Y0, ZC - FS_LENS_X)     # cart

RF = C.RAIL_FACE
CAP, HCAP = C.CAPTURE, C.CAPTURE / math.tan(C.LEAD)
ZL = C.Z_LEN
POSTZ = [(3.0, 11.0), (ZL - 11.0, ZL - 3.0)]
pkm, pki = C.pocket_block()
PKZ = (pki["z"][0] - C.PK_WALL, pki["z"][1] + C.PK_WALL)
pbm = C.shoe_module()

# ---- grid (cartridge coords), mirrored in x, centred on x 0
XH = 39.0; X0, Y0, Z0 = -XH, -1.5, -34.0
nx, ny, nz = 52, int(round((Y_PLATE - Y0) / H)), 80          # x -39..39, y -1.5..63, z -34..86
shape = (nx, ny, nz); nel = nx * ny * nz; origin = (X0, Y0, Z0)
xs = X0 + (np.arange(nx) + 0.5) * H; ys = Y0 + (np.arange(ny) + 0.5) * H; zs = Z0 + (np.arange(nz) + 0.5) * H
I3 = np.stack(np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij"), -1).reshape(-1, 3)
X, Y, Z = (I3 * H + np.asarray(origin) + H / 2).T; AX = np.abs(X)

# ---- exact bodies (manifold)
mbox = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def both(a0, a1, y0, y1, z0, z1): return mbox(a0, a1, y0, y1, z0, z1) + mbox(-a1, -a0, y0, y1, z0, z1)
def tm(m_):
    me = m_.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def hullp(pts): return mf.Manifold.hull_points(np.asarray(pts, np.float64))
def mirror_x(m_): return m_.mirror([1, 0, 0])
def cyl_y(x, z, y0, y1, r): return mf.Manifold.cylinder(y1 - y0, r, r, 32).rotate([-90, 0, 0]).translate([x, y0, z])

def foot(s, z0, z1):
    xf, xo, y0 = RF, RF + 4.0, C.Y_MOUTH
    pts = [(xf + CAP, y0), (xo, y0), (xo, y0 + HCAP + 1.0), (xf, y0 + HCAP + 1.0), (xf, y0 + HCAP)]
    return hullp([[s * x, y, z] for x, y in pts for z in (z0, z1)])
def endtab(zf, sg):
    y0 = C.RAIL_Y0
    pts = [(zf + sg * CAP, y0), (zf + sg * 4.0, y0), (zf + sg * 4.0, Y_DECK1), (zf, Y_DECK1), (zf, y0 + HCAP)]
    return hullp([[x, y, z] for z, y in pts for x in (-10.0, 10.0)])

feet = None
for s in (-1, 1):
    for z0, z1 in POSTZ + [PKZ]:
        f = foot(s, z0, z1); feet = f if feet is None else feet + f
locpads = None
for z0, z1 in POSTZ + [(PKZ[0], PKZ[1])]:
    p = both(RF, RF + 2.0, C.RAIL_Y0, Y_DECK1, z0, z1); locpads = p if locpads is None else locpads + p
pk_m = man(pkm); pb_m = man(pbm)
pockets = pk_m + mirror_x(pk_m)
pinblk = pb_m + mirror_x(pb_m)
tabs = endtab(C.Z_END0, -1) + endtab(C.Z_END1, 1)
deck = (both(16.0, RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END0 - 4.0, C.Z_END1 + 4.0)
        + mbox(-(RF + 2.0), RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END0 - 4.0, C.Z_END0)
        + mbox(-(RF + 2.0), RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END1, C.Z_END1 + 4.0))
pads = None
for bx, bz in BOLTS:
    p = cyl_y(bx, bz, Y_PLATE - PAD_T, Y_PLATE, PAD_R); pads = p if pads is None else pads + p
posts = None
for z0, z1 in POSTZ + [PKZ]:
    if (z0, z1) == PKZ:
        yb = pki["y"][1] - 0.01; z0, z1 = C.Z_C - 4.0, C.Z_C + 4.0
    else:
        yb = C.Y_MOUTH + HCAP
    p = both(RF, RF + 2.0, yb, Y_DECK1, z0, z1); posts = p if posts is None else posts + p
cups = None
for bx, bz in BOLTS:
    cup = cyl_y(bx, bz, Y_DECK0, Y_PLATE, NUT_R + 2.0) - cyl_y(bx, bz, Y_DECK0 - 1, Y_PLATE - PAD_T, NUT_R)
    if bx != 0:
        s_ = 1 if bx > 0 else -1
        x0_, x1_ = sorted((s_ * RF, bx))
        cup = cup + mbox(x0_, x1_, Y_DECK0 - 4.0, Y_DECK1, bz - 5.0, bz + 5.0)
    else:
        cup = cup + mbox(-4.0, 4.0, Y_DECK0, Y_DECK1, bz, C.Z_END0)
    cups = cup if cups is None else cups + cup
# flow sensor seat: tape land on the board top + a 1.2 rim 3 down round three sides (tail end open for the plug)
fs_in = (-FS_W / 2 - FS_CLR, FS_W / 2 + FS_CLR, FS_Z0 - FS_CLR, FS_Z1 + FS_CLR)
fs_land = mbox(fs_in[0] - FS_RIM, fs_in[1] + FS_RIM, FS_Y1, FS_Y1 + FS_LAND, fs_in[2] - FS_RIM, fs_in[3])
fs_rim = (mbox(fs_in[0] - FS_RIM, fs_in[1] + FS_RIM, FS_Y1 - FS_RIM_H, FS_Y1, fs_in[2] - FS_RIM, fs_in[3])
          - mbox(fs_in[0], fs_in[1], FS_Y1 - FS_RIM_H - 1, FS_Y1 + 0.01, fs_in[2], fs_in[3] + 1))
fseat = fs_land + fs_rim
# 17 Oct: the nut cup walls and deck tongues (6.7 g drawn) are not mounting surfaces: removed, the pads stay and the
# layout joins them (the nyloc spaces stay cut). Posts stay drawn (locating guides; without them 16 Oct runs floated).
# 17 Oct 06:40: the generated links from the side pads came out 1.6 to 1.8 thick (built thin check, twice) and the prune
# re-settle OOMs: each side pad gets a drawn half cup (2.0 wall round the nyloc space, inboard half only, so the wrench
# slot stays open) from the pad down to a deck-level tongue. Mounting structure, ~0.6 g each (C).
side_cups = None
for bx, bz in BOLTS[:2]:
    s_ = 1 if bx > 0 else -1
    ring = cyl_y(bx, bz, Y_DECK0, Y_PLATE - PAD_T + 0.01, NUT_R + 2.0) - cyl_y(bx, bz, Y_DECK0 - 1, Y_PLATE, NUT_R)
    a0, a1 = sorted((bx - s_ * 20.0, bx))                     # inboard half
    ring = ring ^ mbox(a0, a1, Y_DECK0 - 1, Y_PLATE + 1, bz - 10, bz + 10)
    x0_, x1_ = sorted((s_ * RF, bx))
    tongue = mbox(x0_, x1_, Y_DECK0, Y_DECK1, bz - 4.0, bz + 4.0)
    c = ring + tongue; side_cups = c if side_cups is None else side_cups + c
# nose pad: same half cup, the tail half (the nyloc wrench comes from the nose); it lands on the deck's end bar
nbx, nbz = BOLTS[2]
nose_cup = (cyl_y(nbx, nbz, Y_DECK0, Y_PLATE - PAD_T + 0.01, NUT_R + 2.0) - cyl_y(nbx, nbz, Y_DECK0 - 1, Y_PLATE, NUT_R)) \
    ^ mbox(-20, 20, Y_DECK0 - 1, Y_PLATE + 1, nbz, nbz + 20)
side_cups = side_cups + nose_cup
drawn = feet + locpads + pockets + pinblk + tabs + deck + pads + posts + fseat + side_cups

# ---- cuts (exact)
def wedge_x():
    out = None
    for s in (-1, 1):
        poly = [(0, C.Y_MOUTH - 50), (RF + CAP, C.Y_MOUTH - 50), (RF + CAP, C.Y_MOUTH), (RF, C.Y_MOUTH + HCAP), (0, C.Y_MOUTH + HCAP)]
        w = hullp([[s * x, y, z] for x, y in poly for z in (C.Z_END0 - CAP, C.Z_END1 + CAP)])
        out = w if out is None else out + w
    return out
def wedge_z():
    out = None
    for zf, sg in ((C.Z_END0, -1), (C.Z_END1, 1)):
        poly = [(ZL / 2, C.RAIL_Y0 - 60), (zf + sg * CAP, C.RAIL_Y0 - 60), (zf + sg * CAP, C.RAIL_Y0), (zf, C.RAIL_Y0 + HCAP), (ZL / 2, C.RAIL_Y0 + HCAP)]
        w = hullp([[x, y, z] for z, y in poly for x in (-RF, RF)])
        out = w if out is None else out + w
    return out
cart_path = mbox(-RF, RF, Y0 - 5, Y_DECK0, C.Z_END0, C.Z_END1) + wedge_x() + wedge_z()
C.pocket_block(); pc = man(C._POCKET_CUT); pocket_cut = pc + mirror_x(pc)
sc = man(C.shoe_cut()); bores = sc + mirror_x(sc)
slotm = both(C.X_PLATE - 0.3, C.X_PLATE + C.PLATE_T + 0.6, C.Y_SHOE, Y_PLATE + 1.0,
             C.ZM - C.PLATE_W / 2 - 0.5, C.ZM + C.PLATE_W / 2 + 0.5)
wiring = slotm + both(C.X_MOD1 - 0.5, C.X_MOD1 + 4.0, C.Y_SHOE - 2.0, Y_PLATE + 1.0, C.ZM + 4.0, C.ZM + 9.0)
bolt_cut = None
for bx, bz in BOLTS:
    yn0, yn1 = Y_PLATE - PAD_T - NUT_H, Y_PLATE - PAD_T
    c1 = cyl_y(bx, bz, yn0 - 1, Y_PLATE + 1, BOLT_R) + cyl_y(bx, bz, yn0, yn1, NUT_R)
    if bx > 0:   c1 = c1 + mbox(bx, XH + 5, yn0, yn1, bz - 4.0, bz + 4.0)
    elif bx < 0: c1 = c1 + mbox(-XH - 5, bx, yn0, yn1, bz - 4.0, bz + 4.0)
    else:        c1 = c1 + mbox(bx - 4.0, bx + 4.0, yn0, yn1, Z0 - 5, bz)
    bolt_cut = c1 if bolt_cut is None else bolt_cut + c1
# flow sensor: board space, plug pocket out the tail end, lead chases up both sides, the view cone (both mirrors)
fs_board = mbox(fs_in[0], fs_in[1], FS_Y0 - 0.5, FS_Y1, fs_in[2], fs_in[3])
fs_plug = mbox(-6.0, 6.0, FS_Y0 - 0.5, FS_Y1, fs_in[3] - 0.01, fs_in[3] + 4.5)
fs_lead = both(6.0, 10.0, FS_Y0, Y_PLATE + 1.0, fs_in[3] + 0.5, fs_in[3] + 4.5)
def cone(apex, ang, depth):
    r = depth * math.tan(ang)
    return mf.Manifold.cylinder(depth, 0.01, r, 48).rotate([90, 0, 0]).translate([apex[0], apex[1], apex[2]])
fs_cone = cone(LENS, FS_HALF_ANG + math.radians(2), 120.0)
fs_cone = fs_cone + mirror_x(fs_cone)
fs_cut = fs_board + fs_plug + fs_lead + fs_cone
cuts = cart_path + pocket_cut + bores + wiring + bolt_cut + fs_cut

# ---- envelope: grid box inside the open slot's sector (W X = -x, W Z = R + y): cos15 (R + y) - sin15 |x| >= SLOT_CLR
env = mbox(-XH, XH, Y0, Y_PLATE, Z0, Z0 + nz * H)
cs, sn = math.cos(SLOT_HALF), math.sin(SLOT_HALF)
for sgn in (-1, 1):
    # inside the slot: distance to its plane sn*(R+y) - cs*|x| >= CLR  <=>  n.p >= d, n = (-sgn*cs, sn, 0), d = CLR - sn*R
    # (17 Oct 06:55: sn and cs were swapped in the first runs, so the slot check passed far too wide)
    env = env.trim_by_plane([-sgn * cs, sn, 0.0], SLOT_CLR - sn * R_RING)
allowed = env - cuts

def vox(m_): return NO._voxelise(tm(m_), xs, ys, zs).ravel()
solid = vox((drawn - cuts) ^ env)
void = (vox(cuts) | ~vox(env)) & ~solid
void |= Y > Y_PLATE
design = ~solid & ~void
so, vo = solid.reshape(shape), void.reshape(shape)
sym_ok = np.array_equal(so, so[::-1]) and np.array_equal(vo, vo[::-1])

# ---- loads (cartridge mass 0.19 kg: pack 165 P + cartridge 21.6 M; latch sized for 35 g)
M_C = 0.19; G = 9.81
nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
def nodes_of(mask):
    el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
    return np.unique(np.concatenate([nid(ex + a, ey + b, ez + cc) for a in (0, 1) for b in (0, 1) for cc in (0, 1)]))
dofs = lambda nd: (nd[:, None] * 3 + np.arange(3)).ravel()
def load(mask, vec):
    nd = nodes_of(mask); F = np.zeros(ndof)
    if len(nd) == 0: raise ValueError("empty load mask")
    for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
    return F
def cells(m_): return vox(m_) & solid
padtop = cells(pads) & (Y > Y_PLATE - H)
sup = dict(bolts=dofs(nodes_of(padtop)))
pock = cells(pockets); floorP = pock & (Y < C.BAR_Y0) & (X > 0); floorM = pock & (Y < C.BAR_Y0) & (X < 0)
deckm = cells(deck); locP = cells(locpads) & (X > 0); footP = cells(feet) & (X > 0)
tab0 = cells(endtab(C.Z_END0, -1)); tab1 = cells(endtab(C.Z_END1, 1))
pinP = cells(pinblk) & (X > 0); pinM = cells(pinblk) & (X < 0)
fsm = cells(fseat)
crash = M_C * G * 35.0
fs_crash = 0.0015 * G * 35.0 + 2.0           # 1.5 g sensor at 35 g + 2 N tie (tape and plug pull)
cases = [
    PO.LoadCase("crash 35g pull-out (pockets)", load(floorP, (0, -crash / 2 - 6, 0)) + load(floorM, (0, -crash / 2 - 6, 0)), cls="crash", support="bolts", accel=np.array([0, -35.0, 0])),
    PO.LoadCase("crash 35g push-in (deck)", load(deckm, (0, crash, 0)), cls="crash", support="bolts", accel=np.array([0, 35.0, 0])),
    PO.LoadCase("crash 35g side (locating faces)", load(locP | footP, (crash, 0, 0)), cls="crash", support="bolts", accel=np.array([35.0, 0, 0])),
    PO.LoadCase("crash 35g nose-ward (closed-end tab)", load(tab0, (0, 0, -crash)), cls="crash", support="bolts", accel=np.array([0, 0, -35.0])),
    PO.LoadCase("crash 35g tail-ward (lead-end tab)", load(tab1, (0, 0, crash)), cls="crash", support="bolts", accel=np.array([0, 0, 35.0])),
    PO.LoadCase("preload 12N (deck up, pockets down)", load(deckm, (0, 12, 0)) + load(floorP, (0, -6, 0)) + load(floorM, (0, -6, 0)), cls="sustained", support="bolts"),
    PO.LoadCase("shoe springs 6N per side outward", load(pinP, (6, 0, 0)) + load(pinM, (-6, 0, 0)), cls="sustained", support="bolts"),
    PO.LoadCase("hover 1g + flight 4g down 2g side", load(floorP, (2 * M_C * G, -2.5 * M_C * G, 0)) + load(floorM, (0, -2.5 * M_C * G, 0)), cls="flight", support="bolts", accel=np.array([2.0, -4.0, 0])),
    PO.LoadCase("insert push 15N (deck)", load(deckm, (0, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: 30 deg foot, +x 8.7 / +y 15", load(footP & (Z < 20), (8.7, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: lead-end foot", load(footP & (Z > 60), (8.7, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: end tab", load(tab0, (0, 15, -8.7)), cls="misuse", support="bolts"),
    PO.LoadCase("pinch 20N on the posts", load(footP, (-20, 0, 0)) + load(cells(feet) & (X < 0), (20, 0, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("flow seat down (35g + tie)", load(fsm, (0, -fs_crash, 0)), cls="crash", support="bolts"),
    PO.LoadCase("flow seat side (35g + tie)", load(fsm, (fs_crash, 0, 0)) + load(fsm, (0, 0, fs_crash)), cls="crash", support="bolts"),
]
pb = PO.Problem(shape=shape, h=H, origin=origin, solid=solid, void=void, supports=sup, cases=cases,
                material=PO.MATERIALS["PETG"], build="-y", overhang=True, R=2.0, comp_factor=None)
pb0 = copy.copy(pb)
VF = GEN_CM3 * 1000 / (design.sum() * H ** 3)
log(f"receiver v3 grid h {H}: {shape} = {nel:,} cells, design {int(design.sum()):,} ({design.sum()*H**3/1000:.1f} cm3), "
    f"drawn {solid.sum()*H**3/1000:.2f} cm3 (exact {drawn.volume()/1000:.2f} = {drawn.volume()*1.27e-3:.1f} g), mirror ok {sym_ok}, "
    f"cases {len(cases)}; generated {GEN_CM3} cm3 = volfrac {VF:.3f}; DZ {DZ}, plate y {Y_PLATE}")

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn - cuts),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    half = S ^ mbox(0, 1e3, -1e3, 1e3, -1e3, 1e3)
    S = half + half.mirror([1, 0, 0])
    return ((S + drawn) ^ env) - cuts

if __name__ == "__main__":
    if "--check" in sys.argv:
        # geometry checks before the run: drawn set inside the slot, flow cone vs the cartridge
        cart = man(C.block()).translate([0, 0, C.Z_BLK0])
        tray_env = mbox(-C.BOSS_X1, C.BOSS_X1, 0, C.Y_TOP, 0, ZL)
        log(f"drawn outside the slot sector: {(drawn - env).volume():.3f} mm3")
        log(f"flow view cone ({math.degrees(FS_HALF_ANG):.0f} deg) vs cartridge box: {(cone(LENS, FS_HALF_ANG, 120) ^ tray_env).volume():.3f} mm3")
        log(f"drawn vs cuts: {((drawn ^ cuts)).volume():.2f} mm3 (removed); drawn pieces {len(drawn.decompose())}")
        sys.exit(0)
    t0 = time.time()
    if "--resume" not in sys.argv and os.path.exists(f"{TAG}_ck.npz"): os.remove(f"{TAG}_ck.npz")
    import noxopt_local as NL                 # lean multigrid backend: the direct solve of 30 cases OOMs the 7 GB box (17 Oct)
    fn_sym, axes, _ = NO.auto_symmetry(pb)
    if axes: pb.sym = fn_sym; log(f"symmetry imposed about {axes}")
    L = NL.Layout(pb, VF, log=log, backend="gmg")
    class _GL(NL.Layout):                    # prune_thin re-settles with NO.Layout (direct solve, OOMs here): use multigrid too
        def __init__(self, pb, volfrac, R=None, eta=(0.7, 0.5, 0.3), log=print, masked=True, **kw):
            super().__init__(pb, volfrac, R=R, eta=eta, log=log, backend="gmg")
    NO.Layout = _GL
    L.sx.state[:] = 0.3                      # dense start: at volfrac ~0.02 a uniform start never builds a path (16 Oct)
    dens = L.run(checkpoint=f"{TAG}_ck.npz", resume=f"{TAG}_ck.npz", time_limit=TLIM, vol_gain=0.3)
    L.save(f"{TAG}_density.npz", dens)
    if os.environ.get("LAYOUT_ONLY"):        # chained cloud runs (2 h per command): layout first, build once settled
        log(f"layout saved, grey {NO.greyness(dens, L.design):.3f}"); sys.exit(0)
    L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, time_limit=900)
    L.save(f"{TAG}_pruned.npz", dens)
    for attempt in range(3):
        con = NO.connectivity(dens, L.pb0); log(f"connectivity: {con}")
        part = build(dens); P = tm(part)
        thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
        if not thin or attempt == 2: break
        L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin, time_limit=900)
    bodies = len(part.decompose())
    fc_ = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
    P.export(f"s2_receiver_v3.stl")          # saved before the recheck (the recheck is the memory peak)
    print(f"BUILD bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g | function "
          f"{fc_[0]:.1f}/{fc_[1]:.1f}/{fc_[2]:.1f} | thin {len(thin)} | wrinkle {wr[0]:.2f}/{wr[1]:.2f}", flush=True)
    if os.environ.get("NO_RECHECK"): sys.exit(0)
    A = NO._voxelise(P, xs, ys, zs).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
    print(rep)
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    real = [r for r in rows if "pinch" not in r["case"] and "capture" not in r["case"] and "insert" not in r["case"]]
    wr_ = max(real, key=lambda r: max(r["inlayer"], r["across"]))
    P.export(f"s2_receiver_v3.stl")
    print(f"RESULT bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g | function "
          f"{fc_[0]:.1f}/{fc_[1]:.1f}/{fc_[2]:.1f} | thin {len(thin)} | wrinkle {wr[0]:.2f}/{wr[1]:.2f} | worst "
          f"{max(w['inlayer'], w['across']):.2f} in {w['case']} | worst non-misuse {max(wr_['inlayer'], wr_['across']):.2f} "
          f"in {wr_['case']} | {time.time()-t0:.0f} s", flush=True)
