"""Scan stick unit v4: the compact layout (scan_stick_system.py placement optimisation, runs/scan_stick_system/v1: 3.23 L,
187 x 85 x 203 mm with guards, was 7.45 L; Spark 24 Oct "most compact design possible") as the system model, the coarse system
solve, and the piece pass. Two pieces, cut at the joint face x 30 (C):
  spine (base v20; v19 stopped): thread boss on the pole axis, the pack's pocket right above it (the heaviest part on the axis), 6 mm back and
        front walls, a top ring with the pin; Pi 5 on 4 standoffs on the back wall, UBEC + buck taped to the back wall below it;
        the back guard rods. Prints lying on its back on the bed diagonal (194 long, C).
  head (head v15, front module; v14 stopped): back plate on the spine's front wall (4x M3 x 12 from inside the pocket through Ø3.4 + Ø6 x 3
        counterbores into Ruthex inserts in the plate, fitted before the pack), the L2 ring and 4 bosses (L2 forward 15° down,
        Spark), driver channels from each boss to the plate's back face (the L2 goes on before the module), the level camera above
        the L2 on Ø11 posts with M2 x 8, the switch panel low on the front (crop zone), the L2 frame rails.
Guards (DEFAULT): L2 U frame of 3 mm carbon rods in a plane 14 behind the L2 base plane at y ±42.5, top rod over the camera
(TPU corner nodes); 2 back rods 5 behind the Pi; the spine walls are the pack's guard. Every drawn body keeps a 2 mm core at the
gate's erosion. Engine: nox/topo default (organic).
   python3 unit_topo_v4.py --check | --system | --piece base|head [h] [gen] | --reactions | --render"""
import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.part import man, tm

ARGS = [a for a in sys.argv[1:] if not a.startswith("-")]
G = 9.81
# ---------------------------------------------------------------- layout (scan_stick_system.py v1 + the drawn interfaces, C)
BOSS_R, SOCK_H = 20.0, 34.0
PACK = (-23.5, 23.5, -25.5, 25.5, 38.0, 188.0)                 # P + clearance: 148 x 48 x 45 pack, vertical on the axis
WALL = 6.0
SPINE = (-30.0, 30.0, -31.5, 31.5, 0.0, 196.0)                 # C
JX = 30.0                                                     # joint face (spine front wall front = module back plate back)
BP = (JX, 40.0, -31.5, 31.5, 44.0, 196.0)                      # module back plate, 10 thick for the inserts (C)
BOLTS = [(sy * 20.0, z) for sy in (-1, 1) for z in (60.0, 180.0)]   # (y, z), C
PI_X0, PI_SO = -30.0, 7.0                                      # Pi on 7 mm standoffs off the back wall face
PI_HOLES = [(sy * 24.5, z) for sy in (-1, 1) for z in (106.5, 164.5)]   # 58 x 49 pattern (spec), board z 103..188
PI_KEEP = (-60.0, PI_X0 - PI_SO + 0.5, -29.0, 29.0, 102.0, 190.0)       # board + parts 23 out (E)
EL_KEEP = (-46.0, -30.5, -30.0, 30.0, 45.0, 101.0)             # UBEC + buck side by side, taped to the back wall (E)
TILT = 15.0
O = np.array((50.1, 0.0, 124.5))                               # C: L2 base centre (placement v1 + 5.5 for the camera behind the joint)
s, c = np.sin(np.radians(TILT)), np.cos(np.radians(TILT))
F, U, Y = np.array((c, 0, -s)), np.array((s, 0, c)), np.array((0, 1.0, 0))
L2_HALF, L2_LEN, L2_PCD = 37.5, 65.0, 51.0                     # spec
CAM_Z, CAM_PLATE, POST_L, POST_R, HOLE_R, BOARD, LENS = O[2] + 60.0, (JX, 36.0), 10.0, 5.5, 0.9, 19.0, 16.0   # C / E
CAM_XF = CAM_PLATE[1] + POST_L + 1.6                           # board front x 47.6
SW = (61.0, 67.0, -30.0, 30.0, 48.0, 78.0)                    # switch panel x 61..67 (C): the bodies behind it in x 40.5..61 clear the plate (E)
ROD_R, SOCK_R = 1.6, 6.0
CAGE_F, CAGE_Y, CAGE_U = -14.0, 42.5, 92.0                     # C: L2 frame plane, side rods y ±42.5, top rod at u +92 (over the camera)
RAIL_U = (-20.0, 40.0)
BACK_GUARD = (-65.0, 24.0, 41.0, 194.0)                        # x, ±y, z0, z1 (C)
POCKET_PIN = 192.0                                            # z, along y at x 0 (C)
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
l2_pads = [(L2_PCD / 2 * np.sin(np.radians(a)), -L2_PCD / 2 * np.cos(np.radians(a))) for a in (22.5, 112.5, 202.5, 292.5)]
cam_posts = [(sy * 14.0, CAM_Z + sz * 14.0) for sy in (-1, 1) for sz in (-1, 1)]

def spine_bodies():
    x0, x1, y0, y1, _, ztop = SPINE
    drawn = cyl(0, 0, 0, SOCK_H, BOSS_R, 96)
    floor = box(x0, x1, y0, y1, 32.0, PACK[4])                                                 # pocket floor, 6 (C)
    back = box(x0, PACK[0] - 0.5, y0, y1, 32.0, ztop)                                           # back wall: tape land + Pi standoffs
    front = box(PACK[1] + 0.5, x1, y0, y1, 32.0, ztop)                                          # front wall: the joint
    ring = box(x0, x1, y0, y1, PACK[5] + 1.0, ztop) - box(PACK[0], PACK[1], PACK[2], PACK[3], PACK[5], ztop + 1)   # top ring round the opening
    standoffs = sum((cylx(PI_X0 - PI_SO, PI_X0 + 0.5, y, z, POST_R) for y, z in PI_HOLES), mf.Manifold())
    gx, gy, gz0, gz1 = BACK_GUARD
    guard = mf.Manifold()
    for sy in (-1, 1):
        y = sy * gy
        guard += cyl(gx, y, gz0 - 3, gz0 + 8, SOCK_R) + cyl(gx, y, gz1 - 8, gz1 + 2, SOCK_R)
        guard += box(gx, x0 + 0.5, y - 3, y + 3, gz0 - 3, gz0 + 3) + box(gx, x0 + 0.5, y - 3, y + 3, gz1 - 4, gz1 + 2)   # 6 x 6 arms
    drawn += floor + back + front + ring + standoffs + guard
    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    cuts = man(trimesh.load(tc, force="mesh"))
    cuts += box(-100, 100, -100, 100, -10, 0)
    cuts += B6(PACK) + box(PACK[0], PACK[1], PACK[2], PACK[3], PACK[5] - 0.01, 420)             # pack + its lift-out path
    cuts += cyly(-40, 40, 0.0, POCKET_PIN, ROD_R)                                              # pin across the top ring
    for y, z in BOLTS:
        cuts += cylx(PACK[1] - 1, JX + 0.01, y, z, 1.7) + cylx(PACK[1] - 1, PACK[1] + 0.5 + 3.0, y, z, 3.0)   # Ø3.4 + Ø6 x 3 counterbore from the pocket
    for y, z in PI_HOLES:
        cuts += cylx(PI_X0 - PI_SO - 1, PI_X0 - PI_SO + 6, y, z, HOLE_R)                       # Ø1.8 for M2 x 8 (E), 6 deep
    cuts += B6(PI_KEEP) + B6(EL_KEEP)
    cuts += box(PI_X0 - PI_SO, PI_X0 - 0.01, -29.0, 29.0, 102.0, 190.0) - sum((cylx(PI_X0 - PI_SO - 1, PI_X0 + 1, y, z, POST_R + 0.5) for y, z in PI_HOLES), mf.Manifold())   # under the board, between the standoffs
    for sy in (-1, 1):
        cuts += cyl(gx, sy * gy, gz0, gz0 + 8.01, ROD_R, 24) + cyl(gx, sy * gy, gz1 - 8.01, gz1, ROD_R, 24)
    env = box(-68.0, JX, -38.0, 38.0, 0.0, 198.0)
    return dict(drawn=drawn, cuts=cuts, env=env, floor=floor, back=back, front=front, ring=ring, standoffs=standoffs, guard=guard)

def head_bodies():
    drawn = B6(BP)
    cuts = mf.Manifold()
    for y, z in BOLTS:
        cuts += cylx(JX - 0.01, JX + 7, y, z, 2.0, 32)                                         # Ruthex M3 Ø4.0 x 7 from the joint face
    ring = mf.Manifold.cylinder(6.0, 31.5, 31.5, 96) - mf.Manifold.cylinder(8.0, 19.5, 19.5, 96).translate([0, 0, -1.0])
    drawn += loc(ring.rotate([0, 90, 0]).translate([-6.0, 0, 0]))
    t15 = np.tan(np.radians(TILT))
    l2_reg = mf.Manifold()
    for y, u in l2_pads:
        p6 = S(-6.0, y, u)
        xb = max(p6[0] - 9.0 * s - 3.0, BP[1] + 0.01)                                           # boss back face (at least on the plate)
        tb = (p6[0] - xb) / c
        pe = p6 - tb * F
        drawn += mf.Manifold.batch_hull([loc(cylx(-6.0, 0.0, y, u, 9.0)), cylx(xb - 0.01, xb, pe[1], pe[2], 9.0)])
        l2_reg += loc(cylx(-6.0, 0.5, y, u, 9.0))
        cuts += loc(cylx(-7, 1, y, u, 1.7) + cylx(-6.0, -3.0, y, u, 3.0) + cylx(-6.0 - tb - 2.0, -6.0, y, u, 4.5))
        cuts += box(JX + 0.01, xb - 0.5 + 0.01, pe[1] - 4.7, pe[1] + 4.7, pe[2] - 4.7 - (xb - JX) * t15, pe[2] + 4.7)   # driver channel through the plate (the L2 goes on first)
    # camera: Ø11 posts from the plate face x 36, board front x 47.6, 10 behind the board (Spark), open down and to the sides for the plug
    posts = sum((cylx(CAM_PLATE[1] - 0.5, CAM_XF - 1.6, y, z, POST_R) for y, z in cam_posts), mf.Manifold())
    for y, z in cam_posts:
        cuts += cylx(CAM_PLATE[0] + 0.01, CAM_XF + 1, y, z, HOLE_R)
    keep = box(CAM_PLATE[1], CAM_XF + 1, -20, 20, CAM_Z - 20, CAM_Z + 20) + box(CAM_PLATE[1], CAM_XF + 1, -46, 46, CAM_Z - 6, CAM_Z + 6)   # board + plug room, a cable slot out the sides
    keep -= sum((cylx(CAM_PLATE[1] - 1, CAM_XF - 1.0, y, z, POST_R + 0.5) for y, z in cam_posts), mf.Manifold())
    cuts += keep
    tv, th = np.tan(np.radians(36.5)), np.tan(np.radians(44.5))
    xe = plane_x(CAM_Z + BOARD + 30 * tv) + 1
    for k in range(4):                                                                          # stepped view frustum up to the L2 plane
        a0, a1 = CAM_XF + (xe - CAM_XF) * k / 4, CAM_XF + (xe - CAM_XF) * (k + 1) / 4
        d = a1 - CAM_XF
        cuts += box(a0, a1, -(BOARD + d * th), BOARD + d * th, CAM_Z - BOARD - d * tv, CAM_Z + BOARD + d * tv)
    drawn += posts
    # switch panel: rocker 19.4 x 13.2 at y +16, button Ø12.3 at y -6, LED Ø5.2 at y -20 (E), bodies behind it
    panel = box(SW[0], SW[1], SW[2], SW[3], SW[4], SW[5])
    cuts += box(SW[0] - 1, SW[1] + 1, 16 - 9.7, 16 + 9.7, 63 - 6.6, 63 + 6.6) + box(BP[1] + 0.5, SW[0], 16 - 11, 16 + 11, 63 - 11, 63 + 11)
    cuts += cylx(SW[0] - 1, SW[1] + 1, -6, 63, 6.15) + cylx(BP[1] + 0.5, SW[0], -6, 63, 7.0)
    cuts += cylx(SW[0] - 1, SW[1] + 1, -20, 63, 2.6) + cylx(SW[0] - 8, SW[0], -20, 63, 3.5)
    drawn += panel
    # L2 frame rails
    rails = mf.Manifold()
    for sy in (-1, 1):
        y = sy * CAGE_Y
        rails += loc(mf.Manifold.cylinder(RAIL_U[1] - RAIL_U[0], SOCK_R, SOCK_R, 32).translate([CAGE_F, y, RAIL_U[0]]))
        cuts += loc(mf.Manifold.cylinder(RAIL_U[1] - RAIL_U[0] + 80, ROD_R, ROD_R, 24).translate([CAGE_F, y, RAIL_U[0] - 40]))
    drawn += rails
    # the L2 hemisphere: 2 mm stair in front of the base plane
    for z0 in np.arange(np.floor(O[2] - L2_HALF * c), 240, 2.0):                               # from the L2's lowest base edge up (below it: the crop zone)
        cuts += box(plane_x(z0 + 2), 160, -90, 90, z0, z0 + 2)
    env = box(JX, 75.0, -49.0, 49.0, 42.0, 226.0)
    reg = dict(l2=l2_reg, cam=sum((cylx(CAM_XF - 4, CAM_XF - 1.0, y, z, POST_R + 0.5) for y, z in cam_posts), mf.Manifold()),
               rails=sum((loc(mf.Manifold.cylinder(RAIL_U[1] - RAIL_U[0], SOCK_R + 0.8, SOCK_R + 0.8, 32).translate([CAGE_F, sy * CAGE_Y, RAIL_U[0]])) for sy in (-1, 1)), mf.Manifold()),
               panel=panel)
    return dict(drawn=drawn, cuts=cuts, env=env, regions=reg)

DIRS = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
DIAG = [np.array((a, b, d), float) / np.sqrt(3) for a in (1, -1) for b in (1, -1) for d in (1, -1)]
JOINT_C = np.array((JX, 0.0, 120.0))
HEAD_KG, HEAD_CG = 0.08, np.array((45.0, 0.0, 130.0))          # E until head v14 reports

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
    for nm, v in (("camera hit front 50 N", np.array((-50.0, 0, 0))), ("camera hit +y 50 N", -50 * Y), ("camera hit -y 50 N", 50 * Y),
                  ("camera hit above 50 N", np.array((0, 0, -50.0)))):
        add(nm, [(reg["cam"], v)], None, [])
    for nm, v in (("L2 frame bump +y 100 N", -100 * Y), ("L2 frame bump -y 100 N", 100 * Y), ("L2 frame bump top 100 N", np.array((0, 0, -100.0))),
                  ("switch panel push 100 N", np.array((-100.0, 0, 0)))):
        add(nm, [(reg["rails"] if "frame" in nm else reg["panel"], v)], None, [])
    return out

def bolt_forces(Fv, Mv):
    r = [np.array((JX, y, z)) - JOINT_C for y, z in BOLTS]
    A = np.zeros((6, 12))
    for i, ri in enumerate(r):
        A[:3, 3 * i:3 * i + 3] = np.eye(3)
        A[3:, 3 * i:3 * i + 3] = np.array([[0, -ri[2], ri[1]], [ri[2], 0, -ri[0]], [-ri[1], ri[0], 0]])
    f = np.linalg.lstsq(A, np.r_[Fv, Mv], rcond=None)[0]
    return [f[3 * i:3 * i + 3] for i in range(4)]

def spine_cases(Sb):
    pack_r = Sb["floor"] + Sb["back"] + Sb["front"] + Sb["ring"]
    return pack_r

def setup_piece(piece, h):
    Sb, Hd = spine_bodies(), head_bodies()
    hc = head_cases(Hd)
    if piece == "head":
        p = Part("scan_stick_head", h, Hd["drawn"], Hd["cuts"], Hd["env"], build="+x", material="PETG")
        p.support("default", box(JX - 1, JX + 1.5, BP[2], BP[3], BP[4], BP[5]))                     # the plate face on the spine, clamped by 4x M3
        for name, parts, accel, Fv, Mv in hc:
            if accel is None:
                p.load(name, parts[0][0], parts[0][1])
            else:
                p.load_multi(name, parts, accel=accel)
        return p
    p = Part("scan_stick_base", h, Sb["drawn"], Sb["cuts"], Sb["env"], build="-x", material="PETG")
    p.support("default", cyl(0, 0, 2, SOCK_H - 6, 19.0, 64) - cyl(0, 0, 0, SOCK_H, 10.0, 64))
    bosses = [cylx(PACK[1] + 0.5, JX + 0.5, y, z, 6.0) - cylx(PACK[1], JX + 1, y, z, 1.7) for y, z in BOLTS]
    pack_r = spine_cases(Sb)
    pi_r = Sb["standoffs"]
    el_r = box(PI_X0, PI_X0 + 2, -30, 30, 45, 101)                                             # UBEC + buck taped on the back wall
    for name, parts, accel, Fv, Mv in hc:
        joint = list(zip(bosses, bolt_forces(Fv, Mv)))
        if accel is None:
            p.load_multi(name + " (joint)", joint)
        else:
            gs = np.linalg.norm(accel); d = accel / gs
            p.load_multi(name, joint + [(pack_r, KG["pack"] * G * gs * d), (pi_r, KG["pi"] * G * gs * d), (el_r, KG["el"] * G * gs * d)], accel=accel)
    gx, gy, gz0, gz1 = BACK_GUARD
    p.load("back guard bump 100 N", Sb["guard"], np.array((100.0, 0, 0)))
    p.load("back guard bump side 100 N", Sb["guard"], np.array((0, 100.0, 0)))
    p.load("spine side bump 100 N", box(-30, 30, 31.5 - 2, 31.5, 60, 180), np.array((0, -100.0, 0)))
    p.load("hand push top ring -z 100 N", Sb["ring"], np.array((0, 0, -100.0)))
    return p

def setup_system(h):
    Sb, Hd = spine_bodies(), head_bodies()
    p = Part("scan_stick_unit", h, Sb["drawn"] + Hd["drawn"], Sb["cuts"] + Hd["cuts"], Sb["env"] + Hd["env"], build="+z", material="PETG")
    p.support("default", cyl(0, 0, 2, SOCK_H - 6, 19.0, 64) - cyl(0, 0, 0, SOCK_H, 10.0, 64))
    reg = dict(pack=spine_cases(Sb), pi=Sb["standoffs"], l2=Hd["regions"]["l2"], cam=Hd["regions"]["cam"], el=box(PI_X0, PI_X0 + 2, -30, 30, 45, 101))
    for gs, ds in ((25, DIRS + DIAG), (50, DIRS)):
        for k, d in enumerate(ds):
            p.load_multi(f"unit {gs} g {k}", [(reg[n], KG[n] * G * gs * d) for n in KG], accel=gs * d)
    for name, parts, accel, Fv, Mv in head_cases(Hd):
        if accel is None:
            p.load(name, parts[0][0], parts[0][1])
    p.load("back guard bump 100 N", Sb["guard"], np.array((100.0, 0, 0)))
    p.load("spine side bump 100 N", box(-30, 30, 31.5 - 2, 31.5, 60, 180), np.array((0, -100.0, 0)))
    return p

def check():
    Sb, Hd = spine_bodies(), head_bodies()
    for n in ("floor", "back", "front", "ring", "standoffs", "guard"):
        print(f"spine {n} ∩ spine cuts {(Sb[n] ^ Sb['cuts']).volume():.0f} mm3, ∩ head cuts {(Sb[n] ^ Hd['cuts']).volume():.0f}")
    print(f"head drawn ∩ head cuts {(Hd['drawn'] ^ Hd['cuts']).volume():.0f}, ∩ spine cuts {(Hd['drawn'] ^ Sb['cuts']).volume():.0f}, head drawn ∩ spine drawn {(Hd['drawn'] ^ Sb['drawn']).volume():.0f}")
    print("L2 lowest", np.round(S(L2_LEN, 0, -L2_HALF), 1), "L2 bottom back", np.round(S(0, 0, -L2_HALF), 1), "plane at the lens", round(plane_x(CAM_Z - 6), 1), "lens tip", CAM_XF + LENS)
    A = Sb["drawn"] + Hd["drawn"]
    print("drawn bbox", np.round(A.bounding_box(), 1), "spine bbox", np.round(Sb["drawn"].bounding_box(), 1), "head bbox", np.round(Hd["drawn"].bounding_box(), 1))

if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--reactions" in sys.argv:
        for name, parts, accel, Fv, Mv in head_cases(head_bodies()):
            print(f"{name}: F {np.round(Fv, 1).tolist()} N, M {np.round(Mv / 1000, 2).tolist()} N m")
    elif "--system" in sys.argv:
        h = float(ARGS[0]) if ARGS else 4.0
        out = os.path.join(ROOT, "runs", "scan_stick_unit", "v4")
        if os.environ.get("NOX_SETUP_ONLY"):
            print(setup_system(h).summary())
        else:
            optimise(setup_system, h=h, gen_cm3=float(ARGS[1]) if len(ARGS) > 1 else 50.0, out_dir=out, name="scan_stick_unit", settings=Settings(), coarse=False, git=True)
    elif "--piece" in sys.argv:
        piece = ARGS[0]
        h = float(ARGS[1]) if len(ARGS) > 1 else 2.0
        gen = float(ARGS[2]) if len(ARGS) > 2 else (16.0 if piece == "head" else 35.0)
        name, ver = ("scan_stick_head", "v15") if piece == "head" else ("scan_stick_base", "v20")   # v14 / v19 were stopped (superseded layout)
        out = os.path.join(ROOT, "runs", name, ver)
        if os.environ.get("NOX_SETUP_ONLY"):
            print(setup_piece(piece, h).summary())
        else:
            optimise(lambda hh: setup_piece(piece, hh), h=h, gen_cm3=gen, out_dir=out, name=name, settings=Settings(), git="--no-git" not in sys.argv)
