"""Scan stick unit v5: the compact layout after the concept review of v4 (24 Oct) as the system model, the coarse system solve
and the piece pass. Placement: scan_stick_system.py v2 (runs/scan_stick_system/v2: the closed cage, the switch height and the
camera on the module plate inside the optimisation; electronics on the back, L2 on the front: 3.25 L, 184 x 91 x 194 mm,
DEFAULT; alternative electronics on the sides 3.23 L, 154 x 108 x 194, the CG further forward). Review fixes:
  - one closed cage of 3 mm carbon rods on TPU nodes (lower ring at the floor, front bar 3 ahead of the L2 dome tip, in the crop
    zone; upper ring over the camera, behind the L2 base plane and out of the camera view; back verticals; front sides up to
    z 52 under the L2, back along z 52, then up the L2 frame plane 3 behind the base plane), held by drawn sleeves on the spine
    (floor and top ring corners) and on the module (rails, lower front), so a bump goes into the spine and the thread boss;
  - the joint: the module's bottom edge hooks behind two lips of a ledge on the spine, 2x M3 x 12 at z 180 from inside the pocket
    (8 under the opening, reachable) into Ruthex inserts in the module plate;
  - the camera lens 1 behind the L2 plane, the top ring front ahead of the lens and outside its view.
Spark 24 Oct (rod in hand): the tip has 25 of thread then a Ø27.5 plain shank, 27 engaged and no more: a Ø28.3 x 2 counterbore
takes the shank (its shoulder seats on the floor), the thread is cut 26 at z 1.5..27.5 (0.5 past the tip), 1 mm entry chamfer; boss Ø40 x 38.
Pieces: spine = base v21 (prints lying on its back on the bed diagonal), front module = head v16.
   python3 unit_topo_v5.py --check | --manifest | --system [h] [gen] | --piece base|head [h] [gen] | --reactions"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.part import man, tm

ARGS = [a for a in sys.argv[1:] if not a.startswith("-")]
G = 9.81
OUT_SYS = os.path.join(ROOT, "runs", "scan_stick_system", "v2")
# ---------------------------------------------------------------- layout (placement v2 + the drawn interfaces, C)
BOSS_R, BOSS_H, THREAD = 20.0, 38.0, 27.0                      # THREAD = engagement: 2 shank + 25 thread (P Spark 24 Oct), C
SHANK_R, SHANK_D = 14.15, 2.0                                    # Ø27.5 shank (P) + 0.4 radial (C), counterbore depth
PACK = (-23.5, 23.5, -25.5, 25.5, 38.0, 188.0)                 # P + clearance
SPINE = (-30.0, 30.0, -31.5, 31.5, 0.0, 196.0)
JX = 30.0
BP = (JX, 40.0, -31.5, 31.5, 36.0, 196.0)                      # module back plate (C)
BOLTS = [(sy * 20.0, 180.0) for sy in (-1, 1)]                 # 2x M3 x 12 at z 180 (C)
LEDGE = (JX, 43.0, -31.5, 31.5, 30.0, 36.0)                    # spine ledge under the module plate (C)
LIPS = [(40.0, 43.0, min(sy * 28.0, sy * 34.0), max(sy * 28.0, sy * 34.0), 36.0, 44.0) for sy in (-1, 1)] + [(JX, 43.0, min(sy * 31.5, sy * 34.0), max(sy * 31.5, sy * 34.0), 36.0, 44.0) for sy in (-1, 1)]   # front lips + side lips: the plate's bottom edge located in x and y (C)   # hook lips in front of the plate's bottom edge (C)
PI_X0, PI_SO = -30.0, 7.0
PI_HOLES = [(sy * 24.5, z) for sy in (-1, 1) for z in (106.5, 164.5)]
PI_KEEP = (-60.0, PI_X0 - PI_SO + 0.5, -29.0, 29.0, 102.0, 190.0)
EL_KEEP = (-46.0, -30.5, -30.0, 30.0, 45.0, 101.0)
TILT = 15.0
O = np.array((46.9, 0.0, 90.8))                                # C (placement v2 46.4 + 0.5 for the lens margin)
s, c = np.sin(np.radians(TILT)), np.cos(np.radians(TILT))
F, U, Y = np.array((c, 0, -s)), np.array((s, 0, c)), np.array((0, 1.0, 0))
L2_HALF, L2_LEN, L2_PCD, L2_BODY, L2_DOME = 37.5, 65.0, 51.0, 39.0, 26.0   # spec / E (box 39 + dome 26)
CAM_Z, CAM_PLATE, POST_L, POST_R, HOLE_R, BOARD, LENS = 163.0, (JX, 36.0), 10.0, 5.5, 0.9, 19.0, 16.0
CAM_XF = CAM_PLATE[1] + POST_L + 1.6                           # 47.6
SWZ = 30.0                                                    # switch row centre (C): on the back, under the UBEC / buck, inside the cage (Spark: nothing in front of the L2)
SW = (-57.0, -51.0, -30.0, 30.0, SWZ - 12.0, SWZ + 12.0)      # panel x -57..-51 facing back, the bodies inside it in x -51..-30
ROD_R, SOCK_R = 1.6, 6.0
YC = 36.0                                                     # cage half width (C): 3 + rod outside the Pi (±29), clear of the joint side lips (34.5); the L2 (±37.5) is Spark's exception (DEFAULT; alt ±44 round the L2, 100 wide)
Z_LOW, Z_TOP = 8.0, 192.0                                     # lower ring just above the pole tip (under the switches), upper ring over the camera (C)
X_BACK = -66.0                                                # back verticals: their 12 nodes clear the Pi (C)
DIAG_END = (10.0, 0.0, Z_TOP)                                 # the back diagonal ends in a drawn node on the top front spine sleeve (C)
Z_MID = 108.0                                                 # mid support of the back verticals and the diagonals: over the UBEC (101), C
CAGE_F = -7.5                                                 # the climbing cage segment 7.5 behind the L2 base plane: 4 mm rail wall in front of the bore, its top nodes behind the plane (concept review 4)
POCKET_PIN = 192.0
KG = dict(pack=0.59, pi=0.08, l2=0.23, cam=0.03, el=0.06)       # E

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def B6(t):
    return box(*t)
def cyl(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])
def cylx(x0, x1, y, z, r, n=48):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])
def cyly(y0, y1, x, z, r, n=32):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])
def loc(m):
    return m.rotate([0, TILT, 0]).translate(O.tolist())
def S(f, y, u):
    return O + f * F + y * Y + u * U
def plane_x(z):
    return O[0] + np.tan(np.radians(TILT)) * (z - O[2])
def frame_x(z):                                               # the climbing cage segment (3 behind the plane, along it)
    return plane_x(z) + CAGE_F / c
l2_pads = [(L2_PCD / 2 * np.sin(np.radians(a)), -L2_PCD / 2 * np.cos(np.radians(a))) for a in (22.5, 112.5, 202.5, 292.5)]
cam_posts = [(sy * 14.0, CAM_Z + sz * 14.0) for sy in (-1, 1) for sz in (-1, 1)]

def cage_rods():
    """(name, p0, p1) of every rod, world (C). Spark 24 Oct: the L2 keeps its full view and sits outside the cage as an
    exception, so nothing of the cage is in front of its base plane: the lower ring is a U (back + sides) whose sides end where
    the frame rods (3 behind the L2 plane) come down to it; the upper ring is closed over the camera."""
    R = []
    for sy in (-1, 1):
        y = sy * YC
        R += [(f"low side {sy:+d}", (X_BACK, y, Z_LOW), (frame_x(Z_LOW), y, Z_LOW)), (f"top side {sy:+d}", (X_BACK, y, Z_TOP), (frame_x(Z_TOP), y, Z_TOP)),
              (f"back vertical {sy:+d}", (X_BACK, y, Z_LOW), (X_BACK, y, Z_TOP)), (f"frame {sy:+d}", (frame_x(Z_LOW), y, Z_LOW), (frame_x(Z_TOP), y, Z_TOP)),
              (f"back diagonal {sy:+d}", (X_BACK, y, Z_LOW), (DIAG_END[0], y, DIAG_END[2]))]   # concept review 2: no racking; ends in a drawn node
    R += [("low back", (X_BACK, -YC, Z_LOW), (X_BACK, YC, Z_LOW)), ("top back", (X_BACK, -YC, Z_TOP), (X_BACK, YC, Z_TOP)),
          ("top front", (frame_x(Z_TOP), -YC, Z_TOP), (frame_x(Z_TOP), YC, Z_TOP))]
    return R

def sleeve_x(x0, x1, y, z):                                    # drawn Ø12 sleeve round a rod along x, Ø3.2 bore
    return cylx(x0, x1, y, z, SOCK_R), cylx(x0 - 1, x1 + 1, y, z, ROD_R)

def spine_bodies():
    x0, x1, y0, y1, _, ztop = SPINE
    drawn = cyl(0, 0, 0, BOSS_H, BOSS_R, 96)
    floor = box(x0, x1, y0, y1, 32.0, PACK[4])
    back = box(x0, PACK[0] - 0.5, y0, y1, 32.0, ztop)
    front = box(PACK[1] + 0.5, x1, y0, y1, 32.0, ztop)
    ring = box(x0, x1, y0, y1, PACK[5] + 1.0, ztop) - box(PACK[0], PACK[1], PACK[2], PACK[3], PACK[5], ztop + 1)
    standoffs = sum((cylx(PI_X0 - PI_SO, PI_X0 + 0.5, y, z, POST_R) for y, z in PI_HOLES), mf.Manifold())
    ledge = B6(LEDGE) + sum((B6(l) for l in LIPS), mf.Manifold())
    sleeves, bores = mf.Manifold(), mf.Manifold()
    for sy in (-1, 1):                                                                         # the cage on the spine: 4 sleeves on boss arms (low), 4 on the top ring (high)
        for xs, z, yin in ((-14.0, Z_LOW, 14.0), (2.0, Z_LOW, 14.0), (-24.0, Z_TOP, y1 - 0.5), (10.0, Z_TOP, y1 - 0.5)):   # >= node r + 1 from every TPU node (review 3)
            sl, bo = sleeve_x(xs - 6, xs + 6, sy * YC, z)
            sleeves += sl + box(xs - 3, xs + 3, min(sy * yin, sy * YC), max(sy * yin, sy * YC), z - 3, z + 3)   # 6 x 6 arm to the boss / the wall
            bores += bo
        y = sy * YC                                                                            # concept review 2: mid-height support of the back vertical
        sleeves += cyl(X_BACK, y, Z_MID - 6, Z_MID + 6, SOCK_R) + box(-27.0, -21.0, min(sy * (y1 - 0.5), y), max(sy * (y1 - 0.5), y), Z_MID - 3, Z_MID + 3)
        sleeves += box(X_BACK, -14.0, y - 3, y + 3, Z_MID - 3, Z_MID + 3)                       # also carries the back diagonal
        dd = np.array(DIAG_END) - np.array((X_BACK, 0, Z_LOW)); dd[1] = 0; dd /= np.linalg.norm(dd)
        tc = (Z_MID - Z_LOW) / dd[2]; pc = np.array((X_BACK, y, Z_LOW)) + tc * dd               # where the diagonal crosses the arm
        sleeves += mf.Manifold.sphere(7.0, 32).translate(pc.tolist())                           # drawn PETG node on the arm: the diagonal passes through it
        bores += mf.Manifold.cylinder(30.0, ROD_R, ROD_R, 24).translate([0, 0, -15.0]).rotate([0, float(np.degrees(np.arctan2(dd[0], dd[2]))), 0]).translate(pc.tolist())
        pe = np.array((DIAG_END[0], y, DIAG_END[2]))                                             # the diagonal ends in a drawn node at the top front sleeve
        sleeves += mf.Manifold.sphere(7.0, 32).translate(pe.tolist())
        bores += mf.Manifold.cylinder(10.0, ROD_R, ROD_R, 24).rotate([0, float(np.degrees(np.arctan2(dd[0], dd[2]))) + 180, 0]).translate(pe.tolist())
        bores += cyl(X_BACK, y, Z_MID - 7, Z_MID + 7, ROD_R, 24)
    panel = box(SW[0], SW[1], SW[2], SW[3], SW[4], SW[5])                                       # rocker y +16, button y -6, LED y -20 (E), facing -x
    drawn += floor + back + front + ring + standoffs + ledge + sleeves + panel
    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    cuts = man(trimesh.load(tc, force="mesh")).translate([0, 0, SHANK_D - 0.5])                  # thread 26 long at z 1.5..27.5: 0.5 into the counterbore (no coincident faces), 0.5 past the tip
    cuts += mf.Manifold.cylinder(SHANK_D + 1.0, SHANK_R, SHANK_R, 96).translate([0, 0, -1.0])      # shank counterbore z 0..2
    cuts += mf.Manifold.cylinder(1.5, 10.9, 9.9, 64).translate([0, 0, SHANK_D - 0.5])              # 1 mm entry chamfer on the thread
    cuts += box(-100, 100, -100, 100, -10, 0)
    cuts += B6(PACK) + box(PACK[0], PACK[1], PACK[2], PACK[3], PACK[5] - 0.01, 420)
    cuts += cyly(-40, 40, 0.0, POCKET_PIN, ROD_R)
    for y, z in BOLTS:
        cuts += cylx(PACK[1] - 1, JX + 0.01, y, z, 1.7) + cylx(PACK[1] - 1, PACK[1] + 0.5 + 3.0, y, z, 3.0)
    for y, z in PI_HOLES:
        cuts += cylx(PI_X0 - PI_SO - 1, PI_X0 - PI_SO + 6, y, z, HOLE_R)
    cuts += B6(PI_KEEP) + B6(EL_KEEP)
    cuts += box(PI_X0 - PI_SO, PI_X0 - 0.01, -29.0, 29.0, 102.0, 190.0) - sum((cylx(PI_X0 - PI_SO - 1, PI_X0 + 1, y, z, POST_R + 0.5) for y, z in PI_HOLES), mf.Manifold())
    cuts += bores
    cuts += box(SW[0] - 1, SW[1] + 1, 16 - 9.7, 16 + 9.7, SWZ - 6.6, SWZ + 6.6) + box(SW[1], PI_X0 - 0.5, 16 - 11, 16 + 11, SWZ - 11, SWZ + 11)
    cuts += cylx(SW[0] - 1, SW[1] + 1, -6, SWZ, 6.15) + cylx(SW[1], PI_X0 - 0.5, -6, SWZ, 7.0)
    cuts += cylx(SW[0] - 1, SW[1] + 1, -20, SWZ, 2.6) + cylx(SW[1], SW[1] + 8, -20, SWZ, 3.5)
    env = box(-70.0, JX, -44.0, 44.0, 0.0, 200.0)                                             # to the cage sleeves (y ±42), not beyond
    return dict(drawn=drawn, cuts=cuts, env=env, floor=floor, back=back, front=front, ring=ring, standoffs=standoffs, sleeves=sleeves, ledge=ledge, panel=panel)

def head_bodies():
    drawn = B6(BP)
    cuts = mf.Manifold()
    for y, z in BOLTS:
        cuts += cylx(JX - 0.01, JX + 7, y, z, 2.0, 32)
    cuts += B6(LEDGE) + sum((B6(l) for l in LIPS), mf.Manifold())                               # the spine's ledge and lips stay clear
    cuts += loc(box(0.0, L2_LEN + 1.0, -L2_HALF - 1.0, L2_HALF + 1.0, -L2_HALF - 1.0, L2_HALF + 1.0))   # the L2 body: the plate front follows its base face
    ring = mf.Manifold.cylinder(6.0, 31.5, 31.5, 96) - mf.Manifold.cylinder(8.0, 19.5, 19.5, 96).translate([0, 0, -1.0])
    drawn += loc(ring.rotate([0, 90, 0]).translate([-6.0, 0, 0]))
    t15 = np.tan(np.radians(TILT))
    l2_reg = mf.Manifold()
    for y, u in l2_pads:
        p6 = S(-6.0, y, u)
        xb = p6[0] - 9.0 * s - 3.0                                                              # boss back face (inside the plate for the lower pads)
        tb = (p6[0] - xb) / c
        pe = p6 - tb * F
        if xb > BP[1]:
            drawn += mf.Manifold.batch_hull([loc(cylx(-6.0, 0.0, y, u, 9.0)), cylx(xb - 0.01, xb, pe[1], pe[2], 9.0)])
        else:
            drawn += loc(cylx(-6.0, 0.0, y, u, 9.0))
        l2_reg += loc(cylx(-6.0, 0.5, y, u, 9.0))
        tj = (p6[0] - JX) / c + 0.5                                                             # the bore runs along the screw to the plate's back face
        cuts += loc(cylx(-7, 1, y, u, 1.7) + cylx(-6.5, -3.0, y, u, 3.0) + cylx(-6.0 - tj, -6.0, y, u, 4.5))   # overlapped 0.5: face-to-face cuts left a 0 mm film (head v16)
    posts = sum((cylx(CAM_PLATE[1] - 0.5, CAM_XF - 1.6, y, z, POST_R) for y, z in cam_posts), mf.Manifold())
    for y, z in cam_posts:
        cuts += cylx(CAM_PLATE[0] + 0.01, CAM_XF + 1, y, z, HOLE_R)
    keep = box(CAM_PLATE[1], CAM_XF + 1, -20, 20, CAM_Z - 20, CAM_Z + 20) + box(CAM_PLATE[1], CAM_XF + 1, -40, 40, CAM_Z - 6, CAM_Z + 6)
    keep -= sum((cylx(CAM_PLATE[1] - 1, CAM_XF - 1.0, y, z, POST_R + 0.5) for y, z in cam_posts), mf.Manifold())
    cuts += keep
    tv, th = np.tan(np.radians(36.5)), np.tan(np.radians(44.5))
    lens = CAM_XF + LENS
    xe = plane_x(CAM_Z + BOARD + 30 * tv) + 1
    for k in range(4):                                                                          # stepped view frustum up to the L2 plane
        a0, a1 = CAM_XF + (xe - CAM_XF) * k / 4, CAM_XF + (xe - CAM_XF) * (k + 1) / 4
        d = max(a1 - lens, 0.0)
        cuts += box(a0, a1, -(BOARD + d * th), BOARD + d * th, CAM_Z - BOARD - d * tv, CAM_Z + BOARD + d * tv)
    drawn += posts
    # the cage on the module: tilted rails on the climbing segments + sleeves on the front runs (z 52)
    rails, bores = mf.Manifold(), mf.Manifold()
    for sy in (-1, 1):
        y = sy * YC
        p0, p1 = np.array((frame_x(80.0), y, 80.0)), np.array((frame_x(140.0), y, 140.0))
        rails += mf.Manifold.batch_hull([cyl(*p0[:2], p0[2], p0[2] + 0.01, SOCK_R), cyl(*p1[:2], p1[2] - 0.01, p1[2], SOCK_R)])
        bores += mf.Manifold.batch_hull([cyl(*(p0 - (p1 - p0) * 0.2)[:2], (p0 - (p1 - p0) * 0.2)[2], (p0 - (p1 - p0) * 0.2)[2] + 0.01, ROD_R),
                                         cyl(*(p1 + (p1 - p0) * 0.2)[:2], (p1 + (p1 - p0) * 0.2)[2] - 0.01, (p1 + (p1 - p0) * 0.2)[2], ROD_R)])
    drawn += rails
    cuts += bores
    for z0 in np.arange(np.floor(O[2] - L2_HALF * c), 240, 2.0):                               # the L2 hemisphere above its lowest base edge
        cuts += box(plane_x(z0 + 2), 160, -90, 90, z0, z0 + 2)
    env = box(JX, 75.0, -50.0, 50.0, 30.0, 200.0)
    reg = dict(l2=l2_reg, cam=sum((cylx(CAM_XF - 4, CAM_XF - 1.0, y, z, POST_R + 0.5) for y, z in cam_posts), mf.Manifold()), rails=rails)
    return dict(drawn=drawn, cuts=cuts, env=env, regions=reg)

DIRS = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
DIAG = [np.array((a, b, d), float) / np.sqrt(3) for a in (1, -1) for b in (1, -1) for d in (1, -1)]
JOINT_C = np.array((JX, 0.0, 110.0))
HEAD_KG, HEAD_CG = 0.09, np.array((45.0, 0.0, 110.0))          # E until head v16 reports

def centroid(m):
    return np.array(m.bounding_box()).reshape(2, 3).mean(0)

def head_cases(Hd):
    reg = Hd["regions"]
    out = []
    def add(name, parts, accel, extra):
        Fv = sum(f for _, f in parts) + sum(f for _, f in extra)
        Mv = sum(np.cross(centroid(r) - JOINT_C, f) for r, f in parts) + sum(np.cross(p - JOINT_C, f) for p, f in extra)
        out.append((name, parts, accel, Fv, Mv))
    for gs, ds in ((25, DIRS + DIAG), (50, DIRS)):
        for k, d in enumerate(ds):
            add(f"unit {gs} g {k}", [(reg["l2"], KG["l2"] * G * gs * d), (reg["cam"], KG["cam"] * G * gs * d)], gs * d, [(HEAD_CG, HEAD_KG * G * gs * d)])
    for nm, v in (("L2 hit front 100 N", -100 * F), ("L2 hit +y 100 N", -100 * Y), ("L2 hit -y 100 N", 100 * Y), ("L2 drop -z 150 N", np.array((0, 0, -150.0)))):
        add(nm, [(reg["l2"], v)], None, [])
    for nm, v in (("camera hit +y 50 N", -50 * Y), ("camera hit above 50 N", np.array((0, 0, -50.0)))):
        add(nm, [(reg["cam"], v)], None, [])
    for nm, v in (("cage bump front 100 N (module share)", np.array((-50.0, 0, 0))), ("cage bump +y 100 N (module share)", np.array((0, -50.0, 0))),
                  ("cage bump top 100 N (module share)", np.array((0, 0, -50.0)))):
        add(nm, [(reg["rails"], v)], None, [])
    return out

# joint (forces ON THE SPINE): 2 M3 at z 180 take any direction; the contacts only push: the plate's bottom edge pushes the
# ledge down (-z), the front lips +x, the side lips outward (+y / -y). Least-norm forces with exact equilibrium (concept review 2).
JOINT_CONTACTS = [("bolt", np.array((JX, y, z)), None) for y, z in BOLTS] + \
                 [("ledge", np.array((35.0, sy * 20.0, 36.0)), np.array((0, 0, -1.0))) for sy in (-1, 1)] + \
                 [("front lip", np.array((41.5, sy * 29.75, 40.0)), np.array((1.0, 0, 0))) for sy in (-1, 1)] + \
                 [("side lip", np.array((36.0, sy * 33.0, 40.0)), np.array((0, sy * 1.0, 0))) for sy in (-1, 1)] + \
                 [("face", np.array((JX, sy * 28.0, z)), np.array((-1.0, 0, 0))) for sy in (-1, 1) for z in (40.0, 190.0)]   # the plate bearing on the spine face (review 3)

def joint_forces(Fv, Mv):
    from scipy.optimize import minimize
    cols, bounds = [], []
    for kind, p, n in JOINT_CONTACTS:
        r = p - JOINT_C
        dirs = np.eye(3) if n is None else [n]
        for d in dirs:
            cols.append(np.r_[d, np.cross(r, d)]); bounds.append((None, None) if n is None else (0.0, None))
    A = np.array(cols).T; b = np.r_[Fv, Mv]
    res = minimize(lambda x: x @ x, np.zeros(A.shape[1]), jac=lambda x: 2 * x, bounds=bounds,
                   constraints=[dict(type="eq", fun=lambda x: A @ x - b, jac=lambda x: A)], method="SLSQP", options=dict(maxiter=500, ftol=1e-10))
    x = res.x; out, k = [], 0
    for kind, p, n in JOINT_CONTACTS:
        if n is None:
            out.append(x[k:k + 3]); k += 3
        else:
            out.append(x[k] * n); k += 1
    return out, float(np.abs(A @ x - b).max())

def joint_regions():
    reg = [cylx(PACK[1] + 0.5, JX + 0.5, y, z, 6.0) - cylx(PACK[1], JX + 1, y, z, 1.7) for y, z in BOLTS]
    reg += [box(30.0, 40.0, sy * 20.0 - 6, sy * 20.0 + 6, 34.0, 36.0) for sy in (-1, 1)]
    reg += [B6(l) for l in LIPS]
    reg += [box(JX - 1.5, JX, sy * 28.0 - 3.5, sy * 28.0 + 3.5, z - 4, z + 4) for sy in (-1, 1) for z in (40.0, 190.0)]
    return reg

def head_contact_regions():
    """where the spine's contacts act on the module (same order as JOINT_CONTACTS after the 2 bolts)"""
    reg = [box(30.0, 40.0, sy * 20.0 - 6, sy * 20.0 + 6, BP[4], BP[4] + 1.5) for sy in (-1, 1)]                 # plate bottom on the ledge
    reg += [box(BP[1] - 1.5, BP[1], min(sy * 28.0, sy * 31.5), max(sy * 28.0, sy * 31.5), BP[4], BP[4] + 8) for sy in (-1, 1)]   # front face bottom on the front lips
    reg += [box(30.0, 40.0, min(sy * 30.0, sy * 31.5), max(sy * 30.0, sy * 31.5), BP[4], BP[4] + 8) for sy in (-1, 1)]       # side faces on the side lips
    reg += [box(JX, JX + 1.5, sy * 28.0 - 3.5, sy * 28.0 + 3.5, z - 4, z + 4) for sy in (-1, 1) for z in (40.0, 190.0)]   # back face on the spine face
    return reg

def setup_piece(piece, h):
    Sb, Hd = spine_bodies(), head_bodies()
    hc = head_cases(Hd)
    if piece == "head":
        p = Part("scan_stick_head", h, Hd["drawn"], Hd["cuts"], Hd["env"], build="+x", material="PETG")
        p.support("default", sum((cylx(JX - 0.5, JX + 9.0, y, z, 5.0) for y, z in BOLTS), mf.Manifold()))   # held at the 2 bolt bosses only (review 3)
        creg = head_contact_regions()
        for name, parts, accel, Fv, Mv in hc:                                                   # + the push-only contact forces the spine puts on it
            jf, err = joint_forces(Fv, Mv)
            contact = [(r, -f) for r, f in zip(creg, jf[2:]) if np.linalg.norm(f) > 1e-6]
            p.load_multi(name, list(parts) + contact, accel=accel)
        return p
    p = Part("scan_stick_base", h, Sb["drawn"], Sb["cuts"], Sb["env"], build="-x", material="PETG")
    p.support("default", cyl(0, 0, 2, THREAD - 2, BOSS_R - 1, 64) - cyl(0, 0, 0, BOSS_H, 11.0, 64))
    pads = joint_regions()
    pack_r = Sb["floor"] + Sb["back"] + Sb["front"] + Sb["ring"]
    el_r = box(PI_X0, PI_X0 + 2, -30, 30, 45, 101)
    for name, parts, accel, Fv, Mv in hc:
        jf, err = joint_forces(Fv, Mv)
        joint = [(r, f) for r, f in zip(pads, jf) if np.linalg.norm(f) > 1e-6]
        if accel is None:
            p.load_multi(name + " (joint)", joint)
        else:
            gs = np.linalg.norm(accel); d = accel / gs
            p.load_multi(name, joint + [(pack_r, KG["pack"] * G * gs * d), (Sb["standoffs"], KG["pi"] * G * gs * d), (el_r, KG["el"] * G * gs * d)], accel=accel)
    for nm, v in (("cage bump back 100 N (spine share)", (50.0, 0, 0)), ("cage bump +y 100 N (spine share)", (0, -50.0, 0)),
                  ("cage bump top 100 N (spine share)", (0, 0, -50.0)), ("cage bump front 100 N (spine share)", (-50.0, 0, 0))):
        p.load(nm, Sb["sleeves"], np.array(v))
    p.load("switch panel push 50 N", Sb["panel"], np.array((50.0, 0, 0)))
    p.load("hand push top ring -z 100 N", Sb["ring"], np.array((0, 0, -100.0)))
    return p

def setup_system(h):
    Sb, Hd = spine_bodies(), head_bodies()
    p = Part("scan_stick_unit", h, Sb["drawn"] + Hd["drawn"], Sb["cuts"] + Hd["cuts"], Sb["env"] + Hd["env"], build="+z", material="PETG")
    p.support("default", cyl(0, 0, 2, THREAD - 2, BOSS_R - 1, 64) - cyl(0, 0, 0, BOSS_H, 11.0, 64))
    reg = dict(pack=Sb["floor"] + Sb["back"] + Sb["front"] + Sb["ring"], pi=Sb["standoffs"], l2=Hd["regions"]["l2"], cam=Hd["regions"]["cam"], el=box(PI_X0, PI_X0 + 2, -30, 30, 45, 101))
    for gs, ds in ((25, DIRS + DIAG), (50, DIRS)):
        for k, d in enumerate(ds):
            p.load_multi(f"unit {gs} g {k}", [(reg[n], KG[n] * G * gs * d) for n in KG], accel=gs * d)
    for name, parts, accel, Fv, Mv in head_cases(Hd):
        if accel is None:
            p.load(name, parts[0][0], parts[0][1])
    for nm, v in (("cage bump back 100 N", (100.0, 0, 0)), ("cage bump +y 100 N", (0, -100.0, 0)), ("cage bump top 100 N", (0, 0, -100.0))):
        p.load_multi(nm, [(Sb["sleeves"], np.array(v) / 2), (Hd["regions"]["rails"], np.array(v) / 2)])
    return p

def manifest():
    import trimesh as T
    os.makedirs(OUT_SYS, exist_ok=True)
    Hd = head_bodies(); tm(Hd["drawn"] - Hd["cuts"]).export(os.path.join(OUT_SYS, "head_drawn.stl"))   # the module's drawn interfaces as built
    Sb = spine_bodies(); tm(Sb["drawn"] - Sb["cuts"]).export(os.path.join(OUT_SYS, "spine_drawn.stl"))
    body = T.creation.box([L2_BODY, 75, 75]); body.apply_translation([L2_BODY / 2, 0, 0])
    dome = T.creation.icosphere(3, 35.5); dome.apply_scale([L2_DOME / 35.5, 1, 1]); dome.apply_translation([L2_BODY, 0, 0])
    dome = dome.slice_plane([L2_BODY, 0, 0], [1, 0, 0], cap=True)
    l2 = T.util.concatenate([body, dome]); M = T.transformations.rotation_matrix(np.radians(TILT), [0, 1, 0]); M[:3, 3] = O; l2.apply_transform(M)
    l2.export(os.path.join(OUT_SYS, "l2.stl"))
    parts = [dict(name="pack", box=list(PACK), role="protected", mass_g=590, tag="P", swap=True),
             dict(name="pi", box=list(PI_KEEP), role="protected", mass_g=80, tag="E"),
             dict(name="ubec_buck", box=list(EL_KEEP), role="protected", mass_g=60, tag="E"),
             dict(name="l2", mesh=os.path.relpath(os.path.join(OUT_SYS, "l2.stl"), ROOT), role="protected", mass_g=230, tag="P"),
             dict(name="camera", box=[CAM_PLATE[1], CAM_XF, -19, 19, CAM_Z - 19, CAM_Z + 19], role="protected", mass_g=25, tag="E"),
             dict(name="lens", cylinder=dict(r=6.0, p0=[CAM_XF, 0, CAM_Z], p1=[CAM_XF + LENS, 0, CAM_Z]), role="protected", mass_g=5, tag="E"),
             dict(name="switches", box=[SW[0], PI_X0 - 0.5, -23.0, 27.0, SW[4], SW[5]], role="protected", mass_g=20, tag="E"),
             dict(name="spine", mesh=os.path.relpath(os.path.join(OUT_SYS, "spine_drawn.stl"), ROOT), role="cage", mass_g=230, tag="E"),   # PETG walls guard the pack and the Pi top
             dict(name="head", mesh=os.path.relpath(os.path.join(OUT_SYS, "head_drawn.stl"), ROOT), role="structure", mass_g=90, tag="E")]
    for name, p0, p1 in cage_rods():
        parts.append(dict(name=name, cylinder=dict(r=1.5, p0=list(p0), p1=list(p1)), role="cage", material="carbon", tag="C"))
    for sy in (-1, 1):                                                                         # 8 TPU nodes, 12 cube (E)
        for nm, x, z in (("low back", X_BACK, Z_LOW), ("top back", X_BACK, Z_TOP), ("frame foot", frame_x(Z_LOW), Z_LOW), ("top front", frame_x(Z_TOP), Z_TOP)):
            parts.append(dict(name=f"node {nm} {sy:+d}", box=[x - 6, x + 6, sy * YC - 6, sy * YC + 6, z - 6, z + 6], role="cage", material="tpu", mass_g=2, tag="E"))
    contacts = [["pack", "spine", 1e9], ["pi", "spine", 1e9], ["ubec_buck", "spine", 1e9], ["head", "spine", 1e9], ["camera", "head", 1e9],
                ["switches", "spine", 1e9], ["camera", "lens", 1e9]]
    names = [n for n, _, _ in cage_rods()] + [p["name"] for p in parts if p["name"].startswith("node")]
    contacts += [[a, b, 600] for i, a in enumerate(names) for b in names[i + 1:]]               # rods meet in the nodes
    m = dict(name="scan stick v5 (compact, closed cage)", parts=parts,
             openings=[dict(part="l2", axis=F.tolist(), half_angle_deg=90.0, min_mm=-80.0),   # Spark 24 Oct: the L2 keeps its full view and sits outside the cage as an exception
                       dict(part="lens", axis=[1, 0, 0], half_angle_deg=44.5), dict(part="camera", axis=[1, 0, 0], half_angle_deg=44.5),
                       dict(part="pack", axis=[0, 0, 1], half_angle_deg=20.0), dict(axis=[0, 0, -1], half_angle_deg=20.0)],   # the pole below
             contact_pairs=contacts, cg_ref=[0, 0, 0], thrust_axis=[0, 0, 1], cg_tol_mm=30.0, cage_margin_mm=3.0)   # hand-held DEFAULT
    path = os.path.join(OUT_SYS, "system_manifest.json"); json.dump(m, open(path, "w"), indent=1)
    return path

E_ROD, I_ROD = 120e3, np.pi * 3.0 ** 4 / 64                     # E: pultruded carbon 120 GPa (E), Ø3 solid

def rod_spans():
    """longest unsupported span of every rod (supports: its ends and the sleeves / rails on it) and its mid-span deflection at 50 N"""
    sup = [(-14.0, None, Z_LOW, 12), (2.0, None, Z_LOW, 12), (-24.0, None, Z_TOP, 12), (10.0, None, Z_TOP, 12), ]   # x-sleeves (x, -, z, length)
    out = []
    for name, p0, p1 in cage_rods():
        p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0); d = (p1 - p0) / L
        ts = [0.0, L]
        for x, _, z, ln in sup:
            for sgn in (-1, 1):
                for e in (x - ln / 2, x + ln / 2):
                    q = np.array((e, sgn * YC, z)); t = (q - p0) @ d
                    if 0 < t < L and np.linalg.norm(p0 + t * d - q) < 0.5:
                        ts.append(t)
        if "back vertical" in name:
            ts += [Z_MID - 6 - Z_LOW, Z_MID + 6 - Z_LOW]
        if "back diagonal" in name:
            ts.append((Z_MID - p0[2]) / d[2])                                                   # the drawn node on the mid-height arm
        if name.startswith("frame"):
            for zz in (80.0, 140.0):
                ts.append((zz - p0[2]) / d[2])
        ts = sorted(ts); span = max(b - a for a, b in zip(ts[:-1], ts[1:]))
        out.append((name, round(L, 1), round(span, 1), round(50.0 * span ** 3 / (48 * E_ROD * I_ROD), 2)))
    return out

def check():
    Sb, Hd = spine_bodies(), head_bodies()
    for n in ("floor", "back", "front", "ring", "standoffs", "sleeves", "ledge"):
        print(f"spine {n} ∩ spine cuts {(Sb[n] ^ Sb['cuts']).volume():.0f} mm3, ∩ head cuts {(Sb[n] ^ Hd['cuts']).volume():.0f}")
    print(f"head drawn ∩ head cuts {(Hd['drawn'] ^ Hd['cuts']).volume():.0f}, ∩ spine cuts {(Hd['drawn'] ^ Sb['cuts']).volume():.0f}, head drawn ∩ spine drawn {(Hd['drawn'] ^ Sb['drawn']).volume():.0f}")
    print("L2 lowest base edge z", round(O[2] - L2_HALF * c, 1), "dome tip x", round(O[0] + (L2_BODY + L2_DOME) * c, 1), "frame foot x", round(frame_x(Z_LOW), 1),
          "plane at the lens", round(plane_x(CAM_Z - 6), 1), "lens tip", CAM_XF + LENS, "top ring front x", round(frame_x(Z_TOP), 1))
    sp = rod_spans()
    print("rods: total length", round(sum(r[1] for r in sp)), "mm; worst 50 N span deflection", max(sp, key=lambda r: r[3]))
    for r in sp:
        print("   ", r)
    for name, parts, accel, Fv, Mv in head_cases(Hd):
        if name in ("unit 50 g 2", "unit 50 g 0", "L2 hit front 100 N", "L2 drop -z 150 N"):
            jf, err = joint_forces(Fv, Mv)
            print(f"joint {name}: F {np.round(Fv, 1).tolist()} N, M {np.round(Mv / 1e3, 2).tolist()} N m; bolts {[np.round(f, 0).tolist() for f in jf[:2]]} N, max contact {max(np.linalg.norm(f) for f in jf[2:]):.0f} N, error {err:.2g}")
    A = Sb["drawn"] + Hd["drawn"]
    print("drawn bbox", np.round(A.bounding_box(), 1), "spine", np.round(Sb["drawn"].bounding_box(), 1), "head", np.round(Hd["drawn"].bounding_box(), 1))

if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--manifest" in sys.argv:
        print(manifest())
    elif "--reactions" in sys.argv:
        for name, parts, accel, Fv, Mv in head_cases(head_bodies()):
            print(f"{name}: F {np.round(Fv, 1).tolist()} N, M {np.round(Mv / 1000, 2).tolist()} N m")
    elif "--system" in sys.argv:
        h = float(ARGS[0]) if ARGS else 3.0
        out = os.path.join(ROOT, "runs", "scan_stick_unit", "v5")
        if os.environ.get("NOX_SETUP_ONLY"):
            print(setup_system(h).summary())
        else:
            optimise(setup_system, h=h, gen_cm3=float(ARGS[1]) if len(ARGS) > 1 else 50.0, out_dir=out, name="scan_stick_unit", settings=Settings(), coarse=False, git=True)
    elif "--piece" in sys.argv:
        piece = ARGS[0]
        h = float(ARGS[1]) if len(ARGS) > 1 else 2.0
        gen = float(ARGS[2]) if len(ARGS) > 2 else (16.0 if piece == "head" else 35.0)
        name, ver = ("scan_stick_head", "v16") if piece == "head" else ("scan_stick_base", "v21")
        out = os.path.join(ROOT, "runs", name, ver)
        if os.environ.get("NOX_SETUP_ONLY"):
            print(setup_piece(piece, h).summary())
        else:
            optimise(lambda hh: setup_piece(piece, hh), h=h, gen_cm3=gen, out_dir=out, name=name, settings=Settings(), git="--no-git" not in sys.argv)

def render():
    """concept artifact: the drawn interfaces of both pieces, the bought parts and the cage (the generated web comes from the runs)"""
    import math, trimesh as T
    from motion import Scene
    Sb, Hd = spine_bodies(), head_bodies()
    def bx(b):
        m = T.creation.box([b[1] - b[0], b[3] - b[2], b[5] - b[4]]); m.apply_translation([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2]); return m
    l2 = T.load(os.path.join(OUT_SYS, "l2.stl"))
    rods = T.util.concatenate([T.creation.cylinder(1.5, segment=[p0, p1], sections=12) for _, p0, p1 in cage_rods()])
    pole = T.creation.cylinder(12.7, 300, sections=48); pole.apply_translation([0, 0, -150])
    s = Scene("Scan stick v5 concept", "compact layout (placement v2, 3.25 L, 182 x 91 x 196 mm with the cage): spine (base v21) + front module (head v16) drawn interfaces, bought parts, closed 3 mm carbon cage; system_check PASS (cage first 4.1 mm, fit 0). The generated web between the drawn parts comes from the piece runs.")
    s.part("spine drawn", tm(Sb["drawn"] - Sb["cuts"]), group="printed", color="#8899aa", printed=True, note="thread boss Ø40 x 38 (32 deep thread), pocket walls, top ring + pin, Pi standoffs, ledge + hook lips, cage sleeves")
    s.part("module drawn", tm(Hd["drawn"] - Hd["cuts"]), group="printed", color="#9aa8b8", printed=True, note="back plate, L2 ring + bosses, camera posts, switch panel, cage rails / sleeves")
    s.part("cage", rods, group="bought", color="#111111", printed=False, note="3 mm carbon rods on TPU nodes")
    for n, b, col in (("pack", PACK, "#2f4fbf"), ("Pi 5", PI_KEEP, "#2e7d32"), ("UBEC + buck", EL_KEEP, "#b07d2b"), ("camera", (CAM_PLATE[1], CAM_XF + LENS, -19, 19, CAM_Z - 19, CAM_Z + 19), "#2d5a2d")):
        s.part(n, bx(b), group="bought", color=col, printed=False)
    s.part("Unitree L2", l2, group="bought", color="#222222", printed=False)
    s.part("pole", pole, group="bought", color="#777777", printed=False)
    names = ["spine drawn", "module drawn", "cage", "pack", "Pi 5", "UBEC + buck", "camera", "Unitree L2", "pole"]
    s.timeline(lambda t: {n: T.transformations.rotation_matrix(2 * math.pi * t / 16, [0, 0, 1]) for n in names}, duration_s=16, fps=30)
    s.view("iso", (420, -480, 300), (20, 0, 100)); s.view("side", (0, -700, 100), (20, 0, 100)); s.view("front", (700, 0, 100), (20, 0, 100))
    s.write(os.path.join(OUT_SYS, "motion.html"))
    try:
        s.video(os.path.join(OUT_SYS, "motion.mp4"))
    except Exception as e:
        print("video skipped:", str(e)[:200])

if __name__ == "__main__" and "--render" in sys.argv:
    render()
