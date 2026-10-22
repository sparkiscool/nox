"""SHOE-CONTACT RECEIVER (this file was written by the contacts chat, 17 Oct 01:10; the other receiver v2 script lives at receiver_to.py).
S2 receiver v2 + Scout mount: one generated part on the three plate bolts (16 Oct 2026).
Spark 19:19: make the receiver stronger, mount it to the Scout on the three plate bolts, run topology on the whole receiver.
Follows nox/noxopt_demo.py; functional surfaces drawn (v2 README "Function first"), everything else generated.

Placement (DEFAULT, one number each; the bottom layout, current-parts Problem 31, may move them):
  drone x = ZC - z_cart (pack centre on the CG, closed end to the nose), drone y = x_cart, drone z = y_cart + DZ.
  Receiver deck top 1.0 under the FC's lowest point (drone z 9.5, CHECK), deck 2.0, TPU preload strip 1.5:
  cartridge top at drone z 5.0, bottom at -43. The Radxa and the flow sensor leave the stack (layout).
Coords below are CARTRIDGE coords (x slide axis = drone y, y up = drone z - DZ, z along the pack = ZC - drone x).
python receiver_to.py [generated cm3, default 8.0] [layout time limit s, default 4800]
"""
import sys, copy, os, time, math
import numpy as np, trimesh, manifold3d as mf
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import noxopt as NO, printopt as PO
import s2_cartridge as C

H = 1.5
GEN_CM3 = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
TLIM = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2][0].isdigit() else 4800
FLOOR, CLIP = 1.2, 0.5
log = lambda s: print(s[:300], flush=True)

# ---- placement (drone <- cartridge)
ZC = C.Z_C                                   # 37.5: pack centre at drone x 0
FC_BOT, FC_GAP, DECK_T, STRIP = 9.5, 1.0, 2.0, 1.5
DZ = (FC_BOT - FC_GAP - DECK_T - STRIP) - C.Y_TOP        # cartridge top at drone z 5.0 -> DZ = -43
Y_DECK0 = C.Y_TOP + STRIP                    # 49.5 deck underside (TPU strip on the rail tops)
Y_DECK1 = Y_DECK0 + DECK_T                   # 51.5
Y_PLATE = 18.5 - DZ                          # 61.5 plate underside (3.5 plate, CHECK)
PAD_R, PAD_T, BOLT_R, NUT_R, NUT_H = 4.75, 2.5, 1.65, 5.5, 6.0
BOLTS = [(31.0, ZC - 3.9), (-31.0, ZC - 3.9), (0.0, ZC - 45.0)]   # (x_cart, z_cart): sides (3.9, ±31), nose (45, 0)
FC_HALF = 36 / math.sqrt(2) + 1.0            # FC diamond |dx|+|dy| <= 25.5, + 1 clearance
FC_X = 1.9

RF = C.RAIL_FACE                             # 20.3 locating face
CAP, HCAP = C.CAPTURE, C.CAPTURE / math.tan(C.LEAD)      # 3.3, 5.2
ZL = C.Z_LEN
POSTZ = [(3.0, 11.0), (ZL - 11.0, ZL - 3.0)]
pkm, pki = C.pocket_block()
PKZ = (pki["z"][0] - C.PK_WALL, pki["z"][1] + C.PK_WALL)
pbm = C.shoe_module()                        # copper shoe modules (16 Oct 20:08: AA springs + copper-clad)

# ---- grid (cartridge coords), mirrored in x
XH = 39.0; X0, Y0, Z0 = -XH, -1.5, -16.0
nx, ny, nz = 52, 42, 68                      # x -39..39 (side nut cups reach 38.5), y -1.5..61.5, z -16..86
# The grid's top face is the plate face = the print bed (build -y): with void rows above it the overhang filter starts
# from an empty first layer and kills every generated cell (16 Oct runs: 0 cells > 0.5).
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

def foot(s, z0, z1):                        # lead-in foot: 30 deg face from (RF + CAP, mouth) to (RF, mouth + HCAP)
    xf, xo, y0 = RF, RF + 4.0, C.Y_MOUTH
    pts = [(xf + CAP, y0), (xo, y0), (xo, y0 + HCAP + 1.0), (xf, y0 + HCAP + 1.0), (xf, y0 + HCAP)]
    return hullp([[s * x, y, z] for x, y in pts for z in (z0, z1)])
def endtab(zf, sg):                         # end tab with a 30 deg lead-in at its bottom (y 38)
    y0 = C.RAIL_Y0
    pts = [(zf + sg * CAP, y0), (zf + sg * 4.0, y0), (zf + sg * 4.0, Y_DECK1), (zf, Y_DECK1), (zf, y0 + HCAP)]
    return hullp([[x, y, z] for z, y in pts for x in (-10.0, 10.0)])

feet = None
for s in (-1, 1):
    for z0, z1 in POSTZ + [PKZ]:
        f = foot(s, z0, z1); feet = f if feet is None else feet + f
locpads = None                              # locating pads at rail height, joined to the deck
for z0, z1 in POSTZ + [(PKZ[0], PKZ[1])]:
    p = both(RF, RF + 2.0, C.RAIL_Y0, Y_DECK1, z0, z1); locpads = p if locpads is None else locpads + p
pk_m = man(pkm); pb_m = man(pbm)
pockets = pk_m + mirror_x(pk_m)
pinblk = pb_m + mirror_x(pb_m)               # shoe modules
tabs = endtab(C.Z_END0, -1) + endtab(C.Z_END1, 1)
deck = (both(16.0, RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END0 - 4.0, C.Z_END1 + 4.0)
        + mbox(-(RF + 2.0), RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END0 - 4.0, C.Z_END0)
        + mbox(-(RF + 2.0), RF + 2.0, Y_DECK0, Y_DECK1, C.Z_END1, C.Z_END1 + 4.0))
pads = None
for bx, bz in BOLTS:
    p = cyl_y(bx, bz, Y_PLATE - PAD_T, Y_PLATE, PAD_R); pads = p if pads is None else pads + p
# 16 Oct 22:00: two runs left the feet, pocket blocks and bolt pads floating (nothing generated between y 8 and 38, nor
# around the nut spaces). Function first: the posts are the locating guides, so they are drawn full height (as the hand
# receiver v1); each bolt pad gets a drawn nut cup (wall 2 round the nyloc space, wrench slot cut after) on a deck tongue.
posts = None
for z0, z1 in POSTZ + [PKZ]:
    if (z0, z1) == PKZ:                                                     # pocket station: an 8 wide spine, not the block's width
        yb = pki["y"][1] - 0.01; z0, z1 = C.Z_C - 4.0, C.Z_C + 4.0
    else:
        yb = C.Y_MOUTH + HCAP
    p = both(RF, RF + 2.0, yb, Y_DECK1, z0, z1); posts = p if posts is None else posts + p
cups = None
for bx, bz in BOLTS:
    yn0 = Y_PLATE - PAD_T - NUT_H
    cup = cyl_y(bx, bz, Y_DECK0, Y_PLATE, NUT_R + 2.0) - cyl_y(bx, bz, Y_DECK0 - 1, Y_PLATE - PAD_T, NUT_R)   # ring wall 2.0
    if bx != 0:
        s_ = 1 if bx > 0 else -1
        x0_, x1_ = sorted((s_ * RF, bx))
        cup = cup + mbox(x0_, x1_, Y_DECK0 - 4.0, Y_DECK1, bz - 5.0, bz + 5.0)   # deck tongue out to the cup, 6 deep (run 3: pinch 1.41 here; 10 deep, r 9 cups, full-width tongues tried: 1.11 to 1.41 for +2 to +3.5 g)
    else:
        cup = cup + mbox(-4.0, 4.0, Y_DECK0, Y_DECK1, bz, C.Z_END0)
    cups = cup if cups is None else cups + cup
drawn = feet + locpads + pockets + pinblk + tabs + deck + pads + posts + cups

# ---- cuts (exact): cartridge path + capture zone, pocket cavities, pin bores, bolt + nut-driver columns, FC, wiring
def wedge_x():                              # the lead-in zone the cartridge may occupy (widens to the mouth)
    k = CAP / HCAP; out = None
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
pocket_cut = C._POCKET_CUT if hasattr(C, "_POCKET_CUT") else None
C.pocket_block(); pc = man(C._POCKET_CUT); pocket_cut = pc + mirror_x(pc)
sc = man(C.shoe_cut()); bores = sc + mirror_x(sc)            # mouth, carrier chamber, plate slot, lead hole
slotm = both(C.X_PLATE - 0.3, C.X_PLATE + C.PLATE_T + 0.6, C.Y_SHOE, Y_PLATE + 1.0,
             C.ZM - C.PLATE_W / 2 - 0.5, C.ZM + C.PLATE_W / 2 + 0.5)                # AA plate drop channel up past the deck
wiring = slotm + both(C.X_MOD1 - 0.5, C.X_MOD1 + 4.0, C.Y_SHOE - 2.0, Y_PLATE + 1.0, C.ZM + 4.0, C.ZM + 9.0)   # lead chase up to the bus
bolt_cut = None                              # shank, nyloc space under the pad, open-wrench slot outboard (sides) / to the nose
for bx, bz in BOLTS:
    yn0, yn1 = Y_PLATE - PAD_T - NUT_H, Y_PLATE - PAD_T
    c1 = cyl_y(bx, bz, yn0 - 1, Y_PLATE + 1, BOLT_R) + cyl_y(bx, bz, yn0, yn1, NUT_R)
    if bx > 0:   c1 = c1 + mbox(bx, XH + 5, yn0, yn1, bz - 4.0, bz + 4.0)
    elif bx < 0: c1 = c1 + mbox(-XH - 5, bx, yn0, yn1, bz - 4.0, bz + 4.0)
    else:        c1 = c1 + mbox(bx - 4.0, bx + 4.0, yn0, yn1, Z0 - 5, bz)
    bolt_cut = c1 if bolt_cut is None else bolt_cut + c1
fc_pts = []
for (dx, dy) in ((FC_HALF, 0), (-FC_HALF, 0), (0, FC_HALF), (0, -FC_HALF)):
    for yy in (FC_BOT - FC_GAP - DZ, Y_PLATE + 1):
        fc_pts.append([dy, yy, ZC - (FC_X + dx)])
fc = hullp(fc_pts)
cuts = cart_path + pocket_cut + bores + wiring + bolt_cut + fc
env = mbox(-XH, XH, Y0, Y_PLATE, Z0, Z0 + nz * H)
allowed = env - cuts

# ---- masks from the exact bodies (cell centres; the grid is offset off the bodies' faces)
def vox(m_): return NO._voxelise(tm(m_), xs, ys, zs).ravel()
solid = vox(drawn - cuts)
void = vox(cuts) & ~solid
void |= Y > Y_PLATE                          # above the plate underside
design = ~solid & ~void
so, vo = solid.reshape(shape), void.reshape(shape)
sym_ok = np.array_equal(so, so[::-1]) and np.array_equal(vo, vo[::-1])

# ---- loads (cartridge mass 0.19 kg: pack 165 P + cartridge 26.4 M; latch sized for 35 g)
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
sup = dict(bolts=dofs(np.unique(np.concatenate([nodes_of(padtop)]))))
pock = cells(pockets); floorP = pock & (Y < C.BAR_Y0) & (X > 0); floorM = pock & (Y < C.BAR_Y0) & (X < 0)
deckm = cells(deck); locP = cells(locpads) & (X > 0); footP = cells(feet) & (X > 0)
tab0 = cells(endtab(C.Z_END0, -1)); tab1 = cells(endtab(C.Z_END1, 1))
pinP = cells(pinblk) & (X > 0); pinM = cells(pinblk) & (X < 0)
crash = M_C * G * 35.0
cases = [
    PO.LoadCase("crash 35g pull-out (pockets)", load(floorP, (0, -crash / 2 - 6, 0)) + load(floorM, (0, -crash / 2 - 6, 0)), cls="crash", support="bolts", accel=np.array([0, -35.0, 0])),
    PO.LoadCase("crash 35g push-in (deck)", load(deckm, (0, crash, 0)), cls="crash", support="bolts", accel=np.array([0, 35.0, 0])),
    PO.LoadCase("crash 35g side (locating faces)", load(locP | footP, (crash, 0, 0)), cls="crash", support="bolts", accel=np.array([35.0, 0, 0])),
    PO.LoadCase("crash 35g nose-ward (closed-end tab)", load(tab0, (0, 0, -crash)), cls="crash", support="bolts", accel=np.array([0, 0, -35.0])),
    PO.LoadCase("crash 35g tail-ward (lead-end tab)", load(tab1, (0, 0, crash)), cls="crash", support="bolts", accel=np.array([0, 0, 35.0])),
    PO.LoadCase("preload 12N (deck up, pockets down)", load(deckm, (0, 12, 0)) + load(floorP, (0, -6, 0)) + load(floorM, (0, -6, 0)), cls="sustained", support="bolts"),
    PO.LoadCase("shoe springs 6N per side outward (E 1.8 x margin)", load(pinP, (6, 0, 0)) + load(pinM, (-6, 0, 0)), cls="sustained", support="bolts"),
    PO.LoadCase("hover 1g + flight 4g down 2g side", load(floorP, (2 * M_C * G, -2.5 * M_C * G, 0)) + load(floorM, (0, -2.5 * M_C * G, 0)), cls="flight", support="bolts", accel=np.array([2.0, -4.0, 0])),
    PO.LoadCase("insert push 15N (deck)", load(deckm, (0, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: 30 deg foot, +x 8.7 / +y 15", load(footP & (Z < 20), (8.7, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: lead-end foot", load(footP & (Z > 60), (8.7, 15, 0)), cls="misuse", support="bolts"),
    PO.LoadCase("capture: end tab", load(tab0, (0, 15, -8.7)), cls="misuse", support="bolts"),
    PO.LoadCase("pinch 20N on the posts", load(footP, (-20, 0, 0)) + load(cells(feet) & (X < 0), (20, 0, 0)), cls="misuse", support="bolts"),
]
pb = PO.Problem(shape=shape, h=H, origin=origin, solid=solid, void=void, supports=sup, cases=cases,
                material=PO.MATERIALS["PETG"], build="-y", overhang=True, R=2.0, comp_factor=None)
pb0 = copy.copy(pb)
VF = GEN_CM3 * 1000 / (design.sum() * H ** 3)
log(f"receiver v2 grid h {H}: {shape} = {nel:,} cells, active {int((~void).sum()):,}, design {int(design.sum()):,} "
    f"({design.sum()*H**3/1000:.1f} cm3), drawn {solid.sum()*H**3/1000:.2f} cm3 (exact {drawn.volume()/1000:.2f}), mirror ok {sym_ok}, "
    f"cases {len(cases)}; generated target {GEN_CM3} cm3 = volfrac {VF:.3f}; DZ {DZ:.1f}, plate y {Y_PLATE:.1f}")

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn - cuts),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    half = S ^ mbox(0, 1e3, -1e3, 1e3, -1e3, 1e3)
    S = half + half.mirror([1, 0, 0])
    return ((S + drawn) ^ env) - cuts

def print_check(P, pitch=0.25, jit=0.037):
    from scipy import ndimage as ndi
    axp = [np.arange(P.bounds[0][q] - 1 + jit, P.bounds[1][q] + 1, pitch) for q in range(3)]
    Mp = NO._voxelise(P, *axp); Ep = ndi.distance_transform_edt(Mp) * pitch
    return ndi.label(Mp)[1], ndi.label(Ep > FLOOR / 2)[1]

if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0)
    t0 = time.time()
    L = NO.Layout(pb, VF, log=log, x0=0.3)          # dense start: at volfrac 0.03 a uniform start never builds a path
    dens = L.run(checkpoint="rcv3_ck.npz", resume="rcv3_ck.npz", time_limit=TLIM)
    L.save("rcv3_density.npz", dens)
    con = NO.connectivity(dens, pb0); log(f"connectivity: {con}")
    part = build(dens); P = tm(part); P.export("s2_receiver_v3_raw.stl")
    log(f"exact pieces: {len(part.decompose())}, volume {part.volume()/1000:.2f} cm3")
    pc = print_check(P)
    log(f"print check (shifted grid): bodies {pc[0]}, with everything under {FLOOR} removed {pc[1]}  " + ("OK" if pc == (1, 1) else "FAIL"))
    fc_ = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    A = NO._voxelise(P, xs, ys, zs).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
    print(rep)
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    P.export("s2_receiver_v3.stl")
    print(f"RESULT mass {P.volume / 1000 * 1.27:.1f} g | function {fc_[0]:.1f}/{fc_[1]:.1f}/{fc_[2]:.1f} | worst "
          f"{max(w['inlayer'], w['across']):.2f} in {w['case']} | {time.time()-t0:.0f} s", flush=True)
