"""Scan stick unit v2: the whole stick as one nox/topo optimisation, split at the bolted joint (z 168, C) into base v19 and
head v14 (the camera mount is part of the head). Spark 24 Oct: L2 forward, looking the same way as the camera (+X), 15° down
(C); everything strong enough to take a hit; one optimisation for the bolted parts; M2 for the camera and the Pi, M3 for the rest.

v1 (`unit_topo_v1.py`, runs/scan_stick_unit/v1) built but failed: worst index 2.00 (F) in the 50 g -y knock at the head's joint,
where v1's L-key keyways opened the head's ±y sides above the flange; and both print files failed the gate on 1.6 mm (M) walls and
specks along the 15° tilted keep-outs (camera, frustum, L2 driver bores, the FOV clip), which a 2 mm grid trims into slivers.
v2: (1) the joint bolts go up from below (4x M3 x 12 (C) through the 6 mm lower flange into M3 inserts in the 10 mm upper flange),
the L-key keyways sit in the base under the flange, the head keeps its full section; (2) every keep-out that bounds generated
material is axis-aligned: the L2 hemisphere is a 2 mm stair of boxes in front of the base plane (generated material stays behind
it, within 0.54 mm, C), the camera is level (DEFAULT; alternative tilted 15° with a finer grid) with box keep-outs, the L2 screws
are drawn Ø15 (C) bosses from the tilted ring back to a vertical face with a box channel behind each for a straight hex driver;
the tilted parts are all drawn (exact, no slivers); (3) a keep-out over the pack (v1 left a 0.5 mm sliver at x -20.5); (4) the L2
cable notch faces down (DEFAULT): the L2 and camera cables run down the front to the Pi, no cable lane through the head.

Sensor frame (f, y, u): f = the L2 axis, forward and 15° down; origin O = the centre of the L2 base face at world (30, 0, 290)
(C, DEFAULT). Camera board vertical, centre (−10.8, 0, 205), lens up to 16 long (E, Q3) forward to x 6, behind the L2 base plane;
the L2 covers the camera image above about 23° up (E). Loads (DEFAULT, E) as v1: whole-unit knocks 25 g (14 dirs) / 50 g (6),
pack 0.59, Pi 0.08, L2 0.23, camera 0.03 kg + the structure; L2 hits 100 N front / ±y, 150 N drop; camera hits 50 N front, ±y,
above, below; base strap 120 N, hand grabs 50 N, ledge 100 N. Support: the thread boss wall. The joint is modelled bonded.
   python3 unit_topo_v2.py [h] [gen_cm3] [--no-git]   (layout, then the build + split in a fresh process)
   python3 unit_topo_v2.py --rebuild | --check"""
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
OUT = os.path.join(ROOT, "runs", "scan_stick_unit", "v2")

# ---- base (as v17, C)
BOSS_R, SOCK_H = 20.0, 34.0
FL_Z, TOP = 162.0, 168.0                                    # v2: lower flange z 162..168 (6), the joint face at z 168
UP_T = 10.0                                                 # v2: upper flange 168..178 (inserts from below, 7 deep + 3)
PI_L, PI_W, PI_HX, PI_HY, PI_OFF, PI_SO = 85.0, 56.0, 58.0, 49.0, 3.5, 7.0
PI_X0, PI_Z0 = 20.0, 10.0
PLATE_T = 5.0
BAT_L, BAT_W, BAT_H = 148.0, 48.0, 45.0                     # P
BAT_X1, BAT_Z0 = -20.0, 10.0
STRAP_W, STRAP_T = 20.0, 3.5
SHELF_W, SHELF_Y = 52.0, (-23.0, -18.0)
PANEL_Z = PI_Z0 + PI_L + 4 + 12                             # 111
FLX, FLY = (-20.0, 28.0), 34.0                              # flange x -20..28, y ±34 (C)
BOLTS = [(-8.0, -28.0), (-8.0, 28.0), (16.0, -28.0), (16.0, 28.0)]     # C: M3 x 12 up from below, inserts in the upper flange
# ---- sensors
TILT = 15.0                                                 # C: L2 and camera 15° down (Spark 24 Oct)
O = np.array((30.0, 0.0, 290.0))                            # C, DEFAULT: centre of the L2 base face
s, c = np.sin(np.radians(TILT)), np.cos(np.radians(TILT))
F, U, Y = np.array((c, 0, -s)), np.array((s, 0, c)), np.array((0, 1.0, 0))
L2_HALF, L2_LEN, L2_PCD = 37.5, 65.0, 51.0                  # spec
CAM_Z, CAM_XF, CAM_P, BOARD, LENS = 205.0, -10.0, 14.0, 19.0, 16.0   # C: level board, centre z 205, front face x -10; 28 x 28 (P), 38 x 38 (P); lens 16 (E)
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
def plane_x(z):                                              # the L2 base plane f = 0 at height z
    return O[0] + np.tan(np.radians(TILT)) * (z - O[2])

pi_pads = [(-PI_W / 2 + PI_OFF + iy * PI_HY, PI_Z0 + PI_OFF + ix * PI_HX) for ix in (0, 1) for iy in (0, 1)]
bar_z = [BAT_Z0 + f * BAT_L for f in (0.25, 0.75)]
l2_pads = [(L2_PCD / 2 * np.sin(np.radians(a)), -L2_PCD / 2 * np.cos(np.radians(a))) for a in (22.5, 112.5, 202.5, 292.5)]  # (y, u): angle from -u (the cable axis, notch down)
cam_posts = [(sy * CAM_P, CAM_Z + sz * CAM_P) for sy in (-1, 1) for sz in (-1, 1)]                                    # (y, z), world

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
    cuts += cyl(0, 0, SOCK_H + o2, 154.0, 14.0, 96)                                          # bore Ø28 up to z 154 (wires)
    cuts += box(-5, 5, -19.5, -12.5, 40, 70)                                                 # cable window, shelf side
    cuts += box(-100, 100, -100, 100, -10, 0)
    for x, y in BOLTS:
        cuts += cyl(x, y, FL_Z - 1, TOP + 0.01, 1.7, 32)                                     # Ø3.4 through the lower flange, the screw head under it
        sy = np.sign(y)
        cuts += box(x - 6, x + 6, min(sy * 22, sy * 60), max(sy * 22, sy * 60), 138.0, FL_Z)  # L-key keyway under the flange, open to ±y
    cuts += box(-80, BAT_X1 - off, -60, 60, BAT_Z0 + BAT_L + 1 - 0.01, TOP)                      # v2: nothing over the pack (v1: a 0.5 mm sliver)
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
    cuts_w = mf.Manifold()
    for x, y in BOLTS:
        cuts_w += cyl(x, y, TOP - 0.01, TOP + 7, 2.0, 32)                                      # M3 heat-set Ø4.0 x 7 from the joint face (Ruthex 5.7, depth >= L + 1)
    # L2 ring (sensor frame, exact) + 4 drawn screw bosses back to a vertical face, a box channel behind each
    ring = mf.Manifold.cylinder(6.0, 31.5, 31.5, 96) - mf.Manifold.cylinder(8.0, 19.5, 19.5, 96).translate([0, 0, -1.0])
    drawn = upper + loc(ring.rotate([0, 90, 0]).translate([-6.0, 0, 0]))
    t15 = np.tan(np.radians(TILT))
    l2_reg = mf.Manifold()
    for y, u in l2_pads:
        p6 = S(-6.0, y, u)
        xb = p6[0] - 7.5 * s - 3.0                                                              # boss back face: 3 behind the disc's rearmost point
        tb = (p6[0] - xb) / c                                                                   # along -f from the ring back to that face
        pe = p6 - tb * F
        drawn += mf.Manifold.batch_hull([loc(cylx(-6.0, 0.0, y, u, 7.5)), cylx(xb - 0.01, xb, pe[1], pe[2], 7.5)])   # Ø15 boss: 3.0 wall round the Ø9 bore
        l2_reg += loc(cylx(-6.0, 0.5, y, u, 7.5))
        cuts_w += loc(cylx(-7, 1, y, u, 1.7) + cylx(-6.0, -3.0, y, u, 3.0) + cylx(-6.0 - tb - 2.0, -6.0, y, u, 4.5))   # Ø3.4, Ø6 x 3 head pocket, Ø9 bore to the face
        cuts_w += box(-40.0, xb - 0.5, pe[1] - 4.7, pe[1] + 4.7, pe[2] - 4.7, pe[2] + 4.7 + (xb + 40.0) * t15)        # straight driver channel out the back
    # camera (level, world frame): back plate, 4 posts, board + the 10 mm behind it open down to the flange top and forward
    xb_board = CAM_XF - 1.6                                                                    # board back face
    cplate = box(xb_board - POST_L - CPLATE_T, xb_board - POST_L, -20, 20, CAM_Z - 20, CAM_Z + 20)
    posts = mf.Manifold()
    for y, z in cam_posts:
        posts += cylx(xb_board - POST_L - 0.5, xb_board, y, z, POST_R)
        cuts_w += cylx(xb_board - POST_L - CPLATE_T - 6, CAM_XF + 1, y, z, HOLE_R)              # Ø1.8 through post + plate for M2 x 8 thread-forming
    keep = box(xb_board - POST_L, CAM_XF + 1, -20, 20, TOP + UP_T + 0.5, CAM_Z + 20)          # board + plug room, open down to the flange top
    keep += box(xb_board - POST_L, plane_x(TOP + UP_T) + 1, -20, 20, TOP + UP_T + 0.5, CAM_Z - BOARD - 1)   # ... and forward under the board (USB cable to the Pi)
    for y, z in cam_posts:
        keep -= cylx(xb_board - POST_L - 1, xb_board + 0.5, y, z, POST_R + 0.5)
    cuts_w += keep
    zt = CAM_Z + BOARD + (plane_x(CAM_Z + 40) - CAM_XF) * np.tan(np.radians(36.5))            # E: view 89 x 73° (4:3 assumed) up to the L2 base plane
    hy = BOARD + (plane_x(zt) - CAM_XF) * np.tan(np.radians(44.5))
    cuts_w += box(CAM_XF, plane_x(zt) + 1, -hy, hy, TOP + UP_T + 0.5, zt)                      # the lens view, a box
    drawn += cplate + posts
    # the L2 hemisphere: a 2 mm stair of boxes in front of the base plane (behind it within 0.54 mm)
    for z0 in np.arange(TOP + UP_T + 2, 334, 2.0):
        cuts_w += box(plane_x(z0 + 2), 140, -80, 80, z0, z0 + 2)
    env = box(-34, 50, -46, 46, TOP + UP_T - 2, 332)
    regions = dict(l2=l2_reg, cam=sum((cylx(CAM_XF - 4, CAM_XF + 0.5, y, z, 5.0) for y, z in cam_posts), mf.Manifold()))
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
    for nm, v in (("camera hit front 50 N", np.array((-50.0, 0, 0))), ("camera hit +y 50 N", -50 * Y), ("camera hit -y 50 N", 50 * Y),
                  ("camera hit above 50 N", np.array((0, 0, -50.0))), ("camera hit below 50 N", np.array((0, 0, 50.0)))):
        p.load(nm, reg["cam"], v)
    p.load("strap tension -x 120 N", B["rails"], np.array((-120.0, 0, 0)))
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

def check():
    """setup-only geometry report"""
    B, Hd, A = bodies()
    print("drawn", round(A["drawn"].volume() / 1e3, 1), "cm3; head drawn ∩ head cuts", round((Hd["drawn"] ^ Hd["cuts"]).volume(), 1),
          "head drawn ∩ base cuts", round((Hd["drawn"] ^ B["cuts"]).volume(), 1), "base drawn ∩ head cuts", round((B["drawn"] ^ Hd["cuts"]).volume(), 1))
    for n, p in (("O", O), ("L2 lowest", S(L2_LEN, 0, -L2_HALF)), ("L2 bottom back", S(0, 0, -L2_HALF)), ("L2 top", S(0, 0, L2_HALF))):
        print(f"{n}: {np.round(p, 1)}")
    print("lens tip x", CAM_XF + LENS, "plane x at the lens", round(plane_x(CAM_Z), 1), "board bottom z", CAM_Z - BOARD)
    print("head drawn bbox", np.round(Hd["drawn"].bounding_box(), 1), "unit env bbox", np.round((A["env"] + A["drawn"]).bounding_box(), 1))

def rebuild_split():
    """build from the finished layout (density.npz) without the whole-unit solid check (a 302 mm unit at the gate's 0.25 mm
    voxels ran out of memory, 14 GB): surface, exact part, function check, as-built strength on the whole bolted unit, STL;
    then the split, and the gate runs on each print file (python3 nox/solid_check.py, one process each)."""
    import json, time
    from dataclasses import replace
    from topo.run import Tee
    from topo.layout import Layout, gate_member_mm
    from topo.build import surface, assemble, function_check, strength, export_view_mesh, render, _json_default
    log = Tee(os.path.join(OUT, "log.txt"))
    t0 = time.time()
    st = Settings(finish="drawn")
    st = replace(st, min_member_mm=round(gate_member_mm(setup(H), st.min_member_mm, log=log), 2))
    part = setup(H)
    L = Layout(part, gen_cm3=GEN, settings=st, log=log)
    d = np.load(os.path.join(OUT, "density.npz"))
    assert tuple(d["shape"]) == tuple(part.shape) and np.array_equal(d["design"], L.design)
    L.x = d["x_full"][L.design]; L.hist = [tuple(r) for r in d["hist"]]
    L.result = dict(x=L.x, xt=d["xt"], rho_n=d["rho_n"], rho_e=d["rho_e"], beta=float(d["beta"]))
    gm = surface(L, log=log)
    P, bod = assemble(L, gm, log=log)
    PT = tm(P)
    fc = function_check(P, part, L)
    log(f"function check: {fc}")
    sr = strength(L, PT, log=log)
    mass = P.volume() / 1000.0 * L.mat.density
    PT.export(os.path.join(OUT, "scan_stick_unit.stl"))
    export_view_mesh(PT, os.path.join(OUT, "scan_stick_unit_view.stl"))
    try:
        render(PT, os.path.join(OUT, "preview.png"), title="scan_stick_unit (nox/topo)", info=f"{mass:.1f} g | worst index {round(sr['worst'], 2)}")
    except Exception as e:
        log(f"render failed: {e}")
    del gm, PT
    res = dict(name="scan_stick_unit", engine="nox/topo", function_check=fc, strength=sr, mass_g=round(mass, 2), bodies=bod,
               volume_mm3=round(P.volume(), 1), layout=L.meta(), grid=part.summary(), gen_cm3=GEN, h=H,
               build_s=round(time.time() - t0, 1), solid="per print file (split)")
    log("RESULT " + json.dumps(dict(name="scan_stick_unit", mass_g=res["mass_g"], worst=sr["worst"], function_ok=fc["ok"], bodies=bod["bodies"]), default=_json_default))
    pieces = split()
    import subprocess
    res["pieces"] = {}
    for name in pieces:
        r = subprocess.run([sys.executable, os.path.join(ROOT, "nox", "solid_check.py"), os.path.join(OUT, name + ".stl")], capture_output=True, text=True)
        log(f"solid_check {name}: {r.stdout.strip()[-400:]}")
        res["pieces"][name] = dict(solid=r.stdout.strip()[-400:], ok=" PASS " in r.stdout)
    with open(os.path.join(OUT, "result.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=_json_default)

def split():
    m = man(trimesh.load(os.path.join(OUT, "scan_stick_unit.stl"), force="mesh"))
    lo, hi = m.trim_by_plane([0, 0, -1], -TOP), m.trim_by_plane([0, 0, 1], TOP)
    for name, part in (("scan_stick_base_v19", lo), ("scan_stick_head_v14", hi.translate([0, 0, -TOP]))):
        t = tm(part); t.export(os.path.join(OUT, name + ".stl"))
        print(name, round(t.volume / 1e3 * 1.27, 2), "g PETG (1.27, E)", np.round(t.bounds, 1).tolist(), "bodies", len(t.split(only_watertight=False)))
    return ["scan_stick_base_v19", "scan_stick_head_v14"]

if __name__ == "__main__":
    if "--split" in sys.argv:
        split()
    elif "--rebuild" in sys.argv:
        rebuild_split()
    elif "--check" in sys.argv:
        check()
    elif os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        import subprocess, topo.run as R                                                       # layout only here; the build runs in a fresh process
        class LayoutDone(Exception):
            pass
        def _stop(*a, **k):
            raise LayoutDone()
        R.build_and_check = _stop
        try:
            R.optimise(setup, h=H, gen_cm3=GEN, out_dir=OUT, name="scan_stick_unit", settings=Settings(finish="drawn"), git=GIT)
        except LayoutDone:
            pass
        sys.exit(subprocess.run([sys.executable, os.path.abspath(__file__), "--rebuild"] + ARGS).returncode)
