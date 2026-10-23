"""Scan stick head v13: v12 + review round 4 of PR 22. v12's 12 thick block (y to 24) put the -X camera insert hole into the Ø9
driver clearance of the L2 pad at 112.5° ((-9.76, 23.56), z 10..27): at z 17 there is no room for a 7 deep hole between the M4
pocket (y ≤ 12.5) and that clearance (y ≥ 19). v13 drops the camera bolt holes to z 6 (C; below the clearances, which start at
z 10, and beside the M4 pockets with a 3.0 wall), block y 12..22.5 (10.5 thick, C) so the holes run y 15.5..22.5 (7 deep), block
x ±17 (C) for a 2.7 mm wall where the 112.5° clearance passes its -X end. The bracket hangs from z 6 (its tab holes), camera
board centre at z -26 (world 142). Masks stay v8's. v12: v11 + review round 3 of PR 22. (1) v11's 7 deep camera insert holes ended at y 13, 0.5 mm (M) from the M4
driver pockets (Ø8 at (±8.5, 8.5), z 4..26, reaching y 12.5): v12 draws the camera block y 12..24 (12 thick, C; exact drawn, masks
v8's) so the holes end at y 17, 4.5 mm (C) from the pockets on a 5 mm floor; the bracket keep-out starts at y 24. (2) The M4 heads
cannot pass the arms above z 26: the screws go in sideways between the arms (18 mm of room, M, for a 16 mm screw) and a Ø5 (C)
key hole is cut from each pocket up through the arms to the seat for the 3 mm key. v11: v10 + review round 2 of PR 22: the camera block is 10 thick (y 10..20, C; exact drawn, masks stay v8's) so
its M3 heat-set inserts (Ruthex OD 4.6 x 5.7, spec; depth ≥ L + 1) sit in 7 deep holes (C) on a 3 mm floor; bracket screws M3 x 10,
L2 screws M3 x 8 (3 under the head + 5 of the L2's 6 deep holes). v10: rebuild of the v8 layout with the v9 cable hole and one review fix (PR 22): the Ø6.2 counterbores under
the L2 pads left a 2.9 mm pad wall; v10 cuts them Ø6.0 (M3 socket head Ø5.5 spec, 0.25 clearance) for a 3.0 wall (rule: boss
wall ≥ 3 round every counterbore). Masks stay v8's.  v9: Scan stick head v9: rebuild of the v8 layout with a Ø10 cable hole through the flange centre (the base's bore carries
the lidar and camera cables up). v8: nox/topo run (v7 failed the gate on one 5 mm3 speck: the camera insert holes went 8 deep into the
8 mm block, a 1.5 mm floor; v8 drills them 6 deep. v6 hung inside Pardiso at coarse iteration 11, the 3 mm coarse grid being too coarse for
5 mm members; v7 runs the 1.5 mm grid from the start. v5 failed the gate on one 1 mm sheet: the camera-bracket keep-out reached z 33 and
cut through the two front lidar pads; v6 stops it at z 25, the bracket tab ends at 24. v4 at 6 cm3 with the organic finish failed the gate: the opened flange and a gusset
did not survive the 2.0 mm drawn-feature erosion, 7 bodies, worst 0.87; v5 keeps the drawn bodies whole, 12 cm3) (`scan_stick.md`). Layout change (Spark 23 Oct): the stick is one bolted unit that
screws onto the pole tip; the head no longer carries the thread. It bolts onto the base's top flange with 4x M4
(heat-set inserts in the base) and carries the Unitree L2 on 4 pads and the camera block.
   python3 head_topo_v13.py (rebuilds from runs/scan_stick/v8) [h] [gen_cm3] [--no-git]
Print frame: flange bottom on the bed (z = 0), build +z, lidar seat at z = 34. Numbers C unless tagged.
Drawn: flange Ø40 x 8 (4x Ø4.3 at (±8.5, ±8.5), a 17 square on a Ø24 circle, counterbored), 4 L2 pads Ø12 x 6 on Ø51 at 22.5° (spec) each with a 6 x 10
gusset toward the axis (rule), camera block 34 x 10.5 x 30 on +Y, bracket holes at z 6 (v13). Cuts: bolt holes + counterbores, L2 holes + driver clearance
below each pad, camera insert holes, camera-bracket keep-out, L2 cable notch on -X.
Loads (DEFAULT, E): 25 g on the 230 g lidar (14 dirs) + self weight, 50 g impact (6 dirs), 100 N side hits, 150 N drop
on the lidar, 20 N hand grab; 30 g camera at 25 / 50 g, 5 N pokes. Support: the bolted flange."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise
from topo.run import rebuild

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 1.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 12.0  # cm3 generated (DEFAULT, E)
GIT = "--no-git" not in sys.argv

FL_R, FL_T, FL_BOLT = 20.0, 8.0, 8.5            # flange Ø40 x 8, M4 at (±8.5, ±8.5) = 17 square, Ø24 bolt circle (counterbore wall 4 to the edge)
SEAT, PAD_T, PLATE_W = 34.0, 6.0, 86.0
BOLT_R, BOLT_ROT, PAD_R = 51.0 / 2, 22.5, 6.0
CAM_X, CAM_Y, CAM_Z = 32.0, (12.0, 20.0), (2.0, 32.0)
CAM_HOLE_Z, CAM_HOLE_X = 17.0, 10.0
LIDAR_KG, CAM_KG, G = 0.230, 0.030, 9.81

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

angles = [BOLT_ROT + a for a in (0, 90, 180, 270)]
pads = [(BOLT_R * np.cos(np.radians(a)), BOLT_R * np.sin(np.radians(a))) for a in angles]
fl_bolts = [(sx * FL_BOLT, sy * FL_BOLT) for sx in (-1, 1) for sy in (-1, 1)]

def bodies(hole=False, cb_r=3.1, v11=False, v12=False, v13=False):
    flange = cyl(0, 0, 0, FL_T, FL_R, 96)
    padb = sum((cyl(x, y, SEAT - PAD_T, SEAT, PAD_R) for x, y in pads), mf.Manifold())
    gussets = mf.Manifold()
    for a in angles:
        gussets += box(8.0, BOLT_R, -3.0, 3.0, SEAT - 10.0, SEAT).rotate([0, 0, a])
    cy1 = CAM_Y[1] + (2.5 if v13 else 4.0 if v12 else 0.0)                                   # v13: block y 12..22.5; v12: y 12..24
    cx = CAM_X / 2 + (1.0 if v13 else 0.0)                                                   # v13: block x ±17
    hz = 6.0 if v13 else CAM_HOLE_Z                                                          # v13: camera bolt holes at z 6
    cam_block = box(-cx, cx, CAM_Y[0] - (2.0 if v11 and not v12 else 0.0), cy1, CAM_Z[0], CAM_Z[1])
    drawn = flange + padb + gussets + cam_block
    cuts = mf.Manifold()
    for x, y in fl_bolts:
        cuts += cyl(x, y, -1, FL_T + 1, 2.15) + cyl(x, y, 4.0, SEAT - PAD_T - 2, 4.0)        # M4 through + head / driver clearance
    for x, y in pads:
        cuts += cyl(x, y, SEAT - PAD_T - 1, SEAT + 1, 1.7) + cyl(x, y, SEAT - PAD_T - 1, SEAT - PAD_T + 3, cb_r)
        cuts += cyl(x, y, FL_T + 2, SEAT - PAD_T - 1, 4.5)                                   # driver reaches the M3 heads
    for sx in (-1, 1):
        dp = 7.0 if v11 else 4.5                                                                # v11: 7 deep (Ruthex M3 5.7 + 1), 3 floor
        cuts += mf.Manifold.cylinder(dp + 1.5, 2.0, 2.0, 32).rotate([-90, 0, 0]).translate([sx * CAM_HOLE_X, cy1 - dp, hz])
    cuts += box(-24, 24, cy1, PLATE_W / 2 + 1, -1, 25.0)                                     # camera bracket sits here (tab top z 24; pads start at 28)
    if v12:
        for x, y in fl_bolts:
            cuts += cyl(x, y, SEAT - PAD_T - 3, SEAT + 1, 2.6)                                   # Ø5 key hole from the pocket up through the arms (3 mm key)
    cuts += box(-PLATE_W / 2 - 1, -PLATE_W / 2 + 12.5, -7.0, 7.0, 18.0, SEAT + 1)             # L2 cable drops here (-X)
    if hole:
        cuts += cyl(0, 0, -1, FL_T + 1, 5.0, 48)                                              # Ø10 cable hole through the flange centre (exact cut only)
    env = box(-PLATE_W / 2, PLATE_W / 2, -PLATE_W / 2, PLATE_W / 2, 0, SEAT)
    return dict(drawn=drawn, cuts=cuts, env=env, flange=flange, cam=cam_block)

def setup(h):
    B = bodies(False)                                                                  # v8 cuts: the layout's grid and masks
    p = Part("scan_stick_head", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    from topo.part import tm
    B9 = bodies(True, cb_r=3.0, v11=True, v12=True, v13=True)
    p.cuts_m = B9["cuts"]; p.cuts_t = tm(B9["cuts"])                                   # exact cuts at the build: + the cable hole
    p.drawn_m = B9["drawn"]; p.drawn_t = tm(B9["drawn"]); p.env_m = B9["env"] + B9["drawn"]; p.env_t = tm(p.env_m)   # v11: 10 thick camera block
    p.support("default", cyl(0, 0, -1, FL_T + 0.5, FL_R, 64) - cyl(0, 0, -1, FL_T + 1, 7.0, 64))
    pad_region = sum((cyl(x, y, SEAT - PAD_T, SEAT, PAD_R + 0.5) for x, y in pads), mf.Manifold())
    cam_region = box(-CAM_X / 2 - 0.5, CAM_X / 2 + 0.5, CAM_Y[0] - 0.5, CAM_Y[1] + 0.5, CAM_Z[0] - 0.5, CAM_Z[1] + 0.5)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs + diag):
        p.load(f"lidar 25 g {k}", pad_region, LIDAR_KG * G * 25 * d, accel=25 * d)
        p.load(f"camera 25 g {k}", cam_region, CAM_KG * G * 25 * d)
    for k, d in enumerate(dirs):
        p.load(f"lidar impact 50 g {k}", pad_region, LIDAR_KG * G * 50 * d, accel=50 * d)
        p.load(f"camera impact 50 g {k}", cam_region, CAM_KG * G * 50 * d)
    for nm, v in (("side hit +x 100 N", (100, 0, 0)), ("side hit -y 100 N", (0, -100, 0)), ("drop on lidar -z 150 N", (0, 0, -150)),
                  ("hand grab +y 20 N", (0, 20, 0))):
        p.load(nm, pad_region, np.array(v, float))
    for nm, v in (("poke cam +y", (0, 5, 0)), ("poke cam -z", (0, 0, -5)), ("poke cam +x", (5, 0, 0))):
        p.load(nm, cam_region, np.array(v, float))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick", "v13")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        rebuild(setup, h=H, gen_cm3=GEN, src_dir=os.path.join(ROOT, "runs", "scan_stick", "v8"), out_dir=out, name="scan_stick_head", settings=Settings(finish="drawn"))
