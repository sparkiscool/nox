"""Scan stick unit v1: the whole stick structure as one nox/topo optimisation (Spark 24 Oct: "even though we have different
parts, we should have the whole structure as one optimisation since they will be bolted together"), split after the build
at the bolted joint (z 168, C) into two prints: base v19 (below; v18 was a failed rebuild) and head v14 (above; the camera mount is part of it, the
separate camera bracket is retired).

Spark 24 Oct: (1) the Unitree L2 mounts forward, turned 90° from up-facing, looking the same way as the camera (+X), 15° down
(C); (2) every structure strong enough to take a hit, the camera bracket looked like it could snap; (3) out of M2.5: M2 for
the camera and the Pi 5 (their M2.5 holes take an M2), M3 for the rest.

Sensor frame (f, y, u): f = the L2 axis, forward and 15° down; u = up in the L2 base plane; origin O = the centre of the L2
base face at world (30, 0, 270) (C, DEFAULT; the camera frustum then clears the joint flange). Everything printed above the joint stays behind the L2 base plane (f <= 0), so
nothing of the head is in the lidar's 90° hemisphere (normal mode; the 96° wide mode would see 6° more). The stick below the
joint (flange front, Pi tray) is in the lower part of the hemisphere with a down-tilted L2: crop it in the SLAM settings.
Camera (DEFAULT): directly below the L2, same tilt, board centre at u -60 (C), board front at f -22 (C), so a lens up to 16 long
(E, Q3) stays behind the L2 base plane; the L2 dome then covers about the top 17° of the camera image (E). Alternative not
taken: the camera ahead of the plane (clear view, but in the lidar's FOV).

Drawn (world): the base v17 bodies (thread boss Ø40, tube, Pi tray + Ø8 standoffs with Ø1.8 (E) M2 holes, panel strip, pack
ledge / lip / rails / back plate, UBEC shelf) with a new joint: lower flange 48 x 66 x 10 (C) with 4 M3 heat-set holes Ø4.0 x 7
(Ruthex, spec) at (-14, ±27) and (22, ±27) (C), upper flange 48 x 66 x 6 (C) with Ø3.4 holes, 4x M3 x 12 (C) from above,
an L-key from the side (keyways open to ±y, 24 tall, C); Ø10 cable hole through both flanges into the bore, a 10 x 5 (C) cable
tunnel in the joint face out to +y and a cable lane up the +y side to the L2 notch (the notch faces +y, DEFAULT).
Drawn (sensor frame): L2 ring OD 63 ID 39 x 6 (C) behind the base face joining the 4 L2 pads (Ø51 at 22.5° off the cable
axis, spec), Ø3.4 holes, Ø6 x 3 counterbores from behind, Ø9 driver clearance 39 behind (C); camera back plate 40 x 40 x 5 (C)
with 4 Ø9 x 10 (C) posts on the DECXIN 28 x 28 pattern (P), Ø1.8 (E) through holes for M2 x 8 (C) thread-forming into PETG.
Keep-outs: the L2 body + 2 (75 x 75 x 65, spec), the board and the 10 mm (C) behind it (open to +y for the plug), the camera's
view frustum (89 x 73°, E, 4:3 assumed) up to the L2 base plane, the keyways, the cable lane, the base v17 keep-outs.
Loads (DEFAULT, E): the whole unit knocked at 25 g (14 dirs) and 50 g (6 dirs): pack 0.59, Pi 0.08, L2 0.23, camera 0.03 kg
and the structure's own mass together; hits: 100 N on the L2 from the front and both sides, 150 N drop on the L2, 50 N on the
camera from the front, both sides, above and below; base: 120 N strap, 50 N hand grabs, 100 N on the ledge. Support: the
thread boss wall. The joint is modelled bonded (the M3s clamp it); the split is the plane z 168 through the two flanges only.
   python3 unit_topo_v1.py [h] [gen_cm3] [--no-git]       python3 unit_topo_v1.py --split   (after the run)"""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.part import man, tm

ARGS = [a for a in sys.argv[1:] if not a.startswith("-")]
H = float(ARGS[0]) if len(ARGS) > 0 else 2.0
GEN = float(ARGS[1]) if len(ARGS) > 1 else 55.0             # cm3 generated (DEFAULT, E: base 25 + head 20 for the cantilevered L2 and the hits + camera 5 + 5)
GIT = "--no-git" not in sys.argv
OUT = os.path.join(ROOT, "runs", "scan_stick_unit", "v1")

# ---- base (as v17, C)
BOSS_R, SOCK_H = 20.0, 34.0
FL_Z, TOP = 158.0, 168.0                                    # lower flange z 158..168, the joint face at z 168
UP_T = 6.0                                                  # upper flange 168..174
PI_L, PI_W, PI_HX, PI_HY, PI_OFF, PI_SO = 85.0, 56.0, 58.0, 49.0, 3.5, 7.0
PI_X0, PI_Z0 = 20.0, 10.0
PLATE_T = 5.0
BAT_L, BAT_W, BAT_H = 148.0, 48.0, 45.0                     # P
BAT_X1, BAT_Z0 = -20.0, 10.0
STRAP_W, STRAP_T = 20.0, 3.5
SHELF_W, SHELF_Y = 52.0, (-23.0, -18.0)
PANEL_Z = PI_Z0 + PI_L + 4 + 12                             # 111
FLX, FLY = (-20.0, 28.0), 33.0                              # flange x -20..28, y ±33 (C)
BOLTS = [(-14.0, -27.0), (-14.0, 27.0), (22.0, -27.0), (22.0, 27.0)]   # C: M3, inserts in the lower flange
# ---- sensors
TILT = 15.0                                                 # C: L2 and camera 15° down (Spark 24 Oct)
O = np.array((30.0, 0.0, 270.0))                            # C, DEFAULT: centre of the L2 base face
s, c = np.sin(np.radians(TILT)), np.cos(np.radians(TILT))
F, U, Y = np.array((c, 0, -s)), np.array((s, 0, c)), np.array((0, 1.0, 0))
L2_HALF, L2_LEN, L2_PCD = 37.5, 65.0, 51.0                  # spec
CAM_U, CAM_F, CAM_P, BOARD = -60.0, -22.0, 14.0, 19.0       # C: board centre u -60, board front f -22, 28 x 28 (P), 38 x 38 (P)
POST_L, POST_R, HOLE_R, CPLATE_T = 10.0, 4.5, 0.9, 5.0      # C: posts Ø9 x 10, E: Ø1.8 for M2, C: back plate 5
KG = dict(pack=0.59, pi=0.08, l2=0.23, cam=0.03)            # E (pack Q9, L2 spec 0.23)
G = 9.81

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])
def cylx(x0, x1, y, z, r, n=48):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])
def loc(m):                                                 # sensor frame (x' = f, y, z' = u) -> world
    return m.rotate([0, TILT, 0]).translate(O.tolist())
def S(f, y, u):
    return O + f * F + y * Y + u * U

pi_pads = [(-PI_W / 2 + PI_OFF + iy * PI_HY, PI_Z0 + PI_OFF + ix * PI_HX) for ix in (0, 1) for iy in (0, 1)]
bar_z = [BAT_Z0 + f * BAT_L for f in (0.25, 0.75)]
l2_pads = [(L2_PCD / 2 * np.cos(np.radians(a)), L2_PCD / 2 * np.sin(np.radians(a))) for a in (22.5, 112.5, 202.5, 292.5)]  # (y, u): angle from +y (the cable axis)
cam_posts = [(sy * CAM_P, CAM_U + su * CAM_P) for sy in (-1, 1) for su in (-1, 1)]                                    # (y, u)

def base_bodies(off=0.5):
    """base v17's exact bodies (bodies(0.5, boss_r=20, v13=True)) with the v1 joint and M2 Pi holes"""
    o2 = 4 * off
    drawn = cyl(0, 0, 0, SOCK_H, BOSS_R, 96) + cyl(0, 0, SOCK_H - 4, FL_Z + 2, 18.0, 96)
    lower = box(FLX[0], FLX[1], -FLY, FLY, FL_Z, TOP)
    pads = mf.Manifold()
    for y, z in pi_pads:
        pads += cylx(PI_X0 - 3.5, PI_X0 + PI_SO, y, z, 4.0)
    panel = box(PI_X0 - PLATE_T, PI_X0, -32, 32, PANEL_Z - 12, PANEL_Z + 12)
    panel += box(PI_X0 - PLATE_T, PI_X0, -32, 32, PI_Z0 - 2, PANEL_Z - 12)
    panel += box(PI_X0 - PLATE_T, PI_X0, -44, 44, PANEL_Z - 12, PANEL_Z + 12)
    ledge = box(BAT_X1 - BAT_H - 4, BAT_X1 + 6, -BAT_W / 2 - 2, BAT_W / 2 + 2, 5, 11)
    lip = box(BAT_X1 - BAT_H - 5, BAT_X1 - BAT_H - 1, -BAT_W / 2 - 2, BAT_W / 2 + 2, 5, 20)
    web = box(BAT_X1, BAT_X1 + 8, -16, 16, 11, 70)
    rails = mf.Manifold()
    for sy in (-1, 1):
        y0 = sy * (BAT_W / 2 + 1); rails += box(BAT_X1 - 10, BAT_X1, min(y0, y0 + sy * 14), max(y0, y0 + sy * 14), 5, BAT_Z0 + BAT_L + 2)
    shelf = box(-SHELF_W / 2, SHELF_W / 2, SHELF_Y[0], SHELF_Y[1], 5, 66)
    back = box(BAT_X1, BAT_X1 + 5, -(BAT_W / 2 + 15), BAT_W / 2 + 15, 5, FL_Z + 2)
    drawn += lower + pads + panel + ledge + lip + web + rails + back + shelf

    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    cuts = man(trimesh.load(tc, force="mesh"))
    cuts += cyl(0, 0, SOCK_H + o2, FL_Z - 4, 14.0, 96)                                        # bore Ø28 up to z 154 (4 floor under the flange)
    cuts += cyl(0, 0, FL_Z - 8, TOP + UP_T + 1, 5.0, 48)                                      # Ø10 cable hole through both flanges into the bore
    cuts += box(-5, 5, -19.5, -12.5, 40, 70)                                                 # cable window, shelf side
    cuts += box(-100, 100, -100, 100, -10, 0)
    for x, y in BOLTS:
        cuts += cyl(x, y, TOP - 7, TOP + 0.01, 2.0, 32)                                      # M3 heat-set Ø4.0 x 7 (Ruthex 5.7 long, depth >= L + 1)
    for y, z in pi_pads:
        cuts += cylx(PI_X0 + PI_SO - 6, PI_X0 + PI_SO + 1, y, z, HOLE_R)                    # Ø1.8 for M2 (E), 6 deep
    cuts += box(PI_X0 + PI_SO + off, PI_X0 + PI_SO + 30, -PI_W / 2 - 2, PI_W / 2 + 2, PI_Z0 - 2, PI_Z0 + PI_L + 2)
    cuts += box(PI_X0 - PLATE_T - 1, PI_X0 + 1, 28 - 9.7, 28 + 9.7, PANEL_Z - 6.6, PANEL_Z + 6.6)
    cuts += box(PI_X0 - PLATE_T - 21, PI_X0 - PLATE_T, 28 - 11, 28 + 11, PANEL_Z - 11, PANEL_Z + 11)
    cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, -26, PANEL_Z, 6.15) + cylx(PI_X0 - PLATE_T - 15, PI_X0 - PLATE_T, -26, PANEL_Z, 7.0)
    cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, -39, PANEL_Z, 2.6) + cylx(PI_X0 - PLATE_T - 8, PI_X0 - PLATE_T, -39, PANEL_Z, 3.5)
    cuts += box(BAT_X1 - BAT_H - 1, BAT_X1 - off, -BAT_W / 2 - 1, BAT_W / 2 + 1, BAT_Z0, BAT_Z0 + BAT_L + 1)
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 8)
            cuts += box(BAT_X1 - 11, BAT_X1 + 1, yc - STRAP_T / 2, yc + STRAP_T / 2, z - STRAP_W / 2, z + STRAP_W / 2)
    for sx in (-1, 1):
        for z in (12.0, 32.0, 52.0):
            cuts += box(sx * 12.0 - 1.5, sx * 12.0 + 1.5, SHELF_Y[0] - 1, SHELF_Y[1] + 1, z, z + 8)
    cuts += box(-14.0, 14.0, SHELF_Y[0] - 30, SHELF_Y[0] - off, 4, 68)
    env = box(BAT_X1 - BAT_H - 5, PI_X0 + PI_SO, SHELF_Y[0], 30.0, 0, TOP)
    return dict(drawn=drawn, cuts=cuts, env=env, ledge=ledge, lip=lip, rails=rails, panel=panel, back=back)

def head_bodies():
    upper = box(FLX[0], FLX[1], -FLY, FLY, TOP, TOP + UP_T)
    # L2 ring and pads (sensor frame: x' = f, z' = u)
    ring = mf.Manifold.cylinder(6.0, 31.5, 31.5, 96) - mf.Manifold.cylinder(8.0, 19.5, 19.5, 96).translate([0, 0, -1.0])
    ring = ring.rotate([0, 90, 0]).translate([-6.0, 0, 0])
    cuts = mf.Manifold()
    for y, u in l2_pads:
        cuts += cylx(-7, 1, y, u, 1.7) + cylx(-7, -3, y, u, 3.0) + cylx(-45, -6, y, u, 4.5)   # Ø3.4, Ø6 x 3 counterbore from behind, Ø9 driver clearance
    cplate = box(-22 - 1.6 - POST_L - CPLATE_T, -22 - 1.6 - POST_L, -20, 20, CAM_U - 20, CAM_U + 20)
    posts = mf.Manifold()
    for y, u in cam_posts:
        posts += cylx(-22 - 1.6 - POST_L - 0.5, -22 - 1.6, y, u, POST_R)
        cuts += cylx(-22 - 1.6 - POST_L - CPLATE_T - 6, -22 - 1.6 + 1, y, u, HOLE_R)          # Ø1.8 through post + plate (+6 behind)
    keep = box(-22 - 1.6 - POST_L, -21.0, -20, 50, CAM_U - 20, CAM_U + 20)                    # board + the 10 mm behind it, open to +y (plug)
    for y, u in cam_posts:
        keep -= cylx(-40, -20, y, u, POST_R + 0.5)
    cuts += keep
    hx, hu = BOARD + 22 * np.tan(np.radians(44.5)), BOARD + 22 * np.tan(np.radians(36.5))   # E: view 89 x 73° from the board front to the plane
    cuts += mf.Manifold.batch_hull([box(-22, -21, -BOARD, BOARD, CAM_U - BOARD, CAM_U + BOARD), box(0, 1, -hx, hx, CAM_U - hu, CAM_U + hu)])
    cuts += box(0, L2_LEN + 2, -L2_HALF - 2, L2_HALF + 2, -L2_HALF - 2, L2_HALF + 2)          # L2 body + 2
    drawn_s = ring + cplate + posts
    drawn = upper + loc(drawn_s)
    cuts_w = loc(cuts)
    for x, y in BOLTS:
        cuts_w += cyl(x, y, TOP - 0.01, TOP + UP_T + 1, 1.7, 32)                                # Ø3.4 through the upper flange
        sy = np.sign(y)
        cuts_w += box(x - 6, x + 6, min(sy * 21, sy * 60), max(sy * 21, sy * 60), TOP + UP_T, TOP + UP_T + 24)   # keyway, open to ±y
    cuts_w += box(-5, 5, 0, FLY + 1, TOP - 1, TOP + 4)                                           # cable tunnel in the joint face, out to +y
    notch = S(4.0, L2_HALF + 3, 0.0)
    cuts_w += mf.Manifold.batch_hull([box(-6, 6, FLY, FLY + 12, TOP - 1, TOP + 12),
                                      box(notch[0] - 6, notch[0] + 6, notch[1] - 6, notch[1] + 6, notch[2] - 6, notch[2] + 6)])   # cable lane up the +y side
    env = box(-34, 50, -46, 46, TOP + UP_T - 2, 312) ^ loc(box(-300, 0, -300, 300, -300, 300))   # behind the L2 base plane
    regions = dict(l2=loc(sum((cylx(-6, 0.5, y, u, 6.5) for y, u in l2_pads), mf.Manifold())),
                   cam=loc(sum((cylx(-26.5, -22.5, y, u, 5.0) for y, u in cam_posts), mf.Manifold())))
    return dict(drawn=drawn, cuts=cuts_w, env=env, regions=regions)

def bodies():
    B, Hd = base_bodies(), head_bodies()
    return B, Hd, dict(drawn=B["drawn"] + Hd["drawn"], cuts=B["cuts"] + Hd["cuts"], env=B["env"] + Hd["env"])

def setup(h):
    B, Hd, A = bodies()
    p = Part("scan_stick_unit", h, A["drawn"], A["cuts"], A["env"], build="+z", material="PETG")
    p.support("default", cyl(0, 0, 2, SOCK_H - 6, 19.0, 64) - cyl(0, 0, 0, SOCK_H, 10.0, 64))   # the boss wall round the thread
    pack_r = B["ledge"] + B["lip"] + B["rails"] + B["back"]
    pi_r = mf.Manifold()
    for y, z in pi_pads:
        pi_r += cylx(PI_X0, PI_X0 + PI_SO + 0.5, y, z, 4.0)
    reg = dict(pack=pack_r, pi=pi_r, l2=Hd["regions"]["l2"], cam=Hd["regions"]["cam"])
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for gs, ds in ((25, dirs + diag), (50, dirs)):
        for k, d in enumerate(ds):
            p.load_multi(f"unit {gs} g {k}", [(reg[n], KG[n] * G * gs * d) for n in KG], accel=gs * d)
    for nm, v in (("L2 hit front 100 N", -100 * F), ("L2 hit +y 100 N", -100 * Y), ("L2 hit -y 100 N", 100 * Y),
                  ("L2 drop -z 150 N", np.array((0, 0, -150.0)))):
        p.load(nm, reg["l2"], v)
    for nm, v in (("camera hit front 50 N", -50 * F), ("camera hit +y 50 N", -50 * Y), ("camera hit -y 50 N", 50 * Y),
                  ("camera hit above 50 N", -50 * U), ("camera hit below 50 N", 50 * U)):
        p.load(nm, reg["cam"], v)
    p.load("strap tension -x 120 N", B["rails"], np.array((-120.0, 0, 0)))
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

def check():
    """setup-only geometry report: drawn volume lost to each keep-out (should be 0 except the planned holes)"""
    B, Hd, A = bodies()
    print("drawn", round(A["drawn"].volume() / 1e3, 1), "cm3; drawn ∩ cuts", round((A["drawn"] ^ A["cuts"]).volume(), 1), "mm3")
    print("head drawn ∩ base cuts", round((Hd["drawn"] ^ B["cuts"]).volume(), 1), "base drawn ∩ head cuts", round((B["drawn"] ^ Hd["cuts"]).volume(), 1))
    for n, p in (("O", O), ("camera board centre", S(CAM_F, 0, CAM_U)), ("board bottom edge", S(CAM_F, 0, CAM_U - BOARD)),
                 ("L2 lowest", S(L2_LEN, 0, -L2_HALF)), ("L2 top", S(0, 0, L2_HALF))):
        print(f"{n}: {np.round(p, 1)}")
    print("unit bbox", np.round(A["drawn"].bounding_box(), 1), "env bbox", np.round((A["env"] + A["drawn"]).bounding_box(), 1))

def split():
    m = man(trimesh.load(os.path.join(OUT, "scan_stick_unit.stl"), force="mesh"))
    lo, hi = m.trim_by_plane([0, 0, -1], -TOP), m.trim_by_plane([0, 0, 1], TOP)
    for name, part in (("scan_stick_base_v19", lo), ("scan_stick_head_v14", hi.translate([0, 0, -TOP]))):
        t = tm(part); t.export(os.path.join(OUT, name + ".stl"))
        print(name, round(t.volume / 1e3 * 1.27, 2), "g PETG (1.27, E)", np.round(t.bounds, 1).tolist(), "bodies", len(t.split(only_watertight=False)))

if __name__ == "__main__":
    if "--split" in sys.argv:
        split()
    elif "--check" in sys.argv:
        check()
    elif os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=OUT, name="scan_stick_unit", settings=Settings(finish="drawn"), git=GIT)
