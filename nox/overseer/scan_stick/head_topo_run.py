"""Scan stick head v2: topology run of the lidar head (nox/overseer/scan_stick/scan_stick.md).
Drawn = the 3/4-5 ACME pole boss, the 4 lidar bolt pads (Unitree L2, Ø51 circle at 22.5°, section M), the camera
bracket block. Everything between them is generated.   python3 head_topo_run.py [h] [gen_cm3]
Frame: pole axis = Z, lidar seat plane z = 6 (pads z 0..6), boss z -34..0, camera block on +Y, cable notch on -X.
Print: lidar seat face (z = 6) on the bed, boss up, so build = "-z"; tree supports (overhang filter off).
Loads (DEFAULT, E): 25 g misuse on the 230 g lidar in 14 directions + part self-weight, 20 N hand grab on the pads,
25 g on a 30 g camera, 5 N pokes on the camera block. Support: the thread bore (the pole)."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "..", "box"), os.path.join(HERE, "..", "..")]
import numpy as np, trimesh, manifold3d as mf
from arm_topo import PartProblem, run_part, box, log
from arm_cad import man

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.75
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0        # generated cm3 (DEFAULT): v1 drawn head is 85.2 g (M)

# ---- numbers (mm). C = scan_stick.scad, spec = nox/dimensions.md section M
BOSS_R, SOCK_H = 16.0, 34.0                     # C: pole boss OD 32, height 34; thread 27 deep from the bottom
PLATE_T, PLATE_W = 6.0, 86.0                    # C: lidar seat at z = PLATE_T; envelope square
BOLT_R, BOLT_ROT = 51.0 / 2, 22.5               # spec: 4x M3 on Ø51, 22.5° off the cable axis (X)
PAD_R = 6.0                                     # C: Ø12 bolt pads (Problem 46)
CAM_X, CAM_Y, CAM_Z = 32.0, (12.0, 20.0), (-32.0, -2.0)   # C: camera block 32 x 8 x 30 on the +Y face of the boss
CAM_HOLE_Z, CAM_HOLE_X = -17.0, 10.0            # C: 2x Ø4 insert holes, 20 apart
NOTCH = (-PLATE_W / 2 - 1, -PLATE_W / 2 + 12.5, -7.0, 7.0)   # C: cable notch x0, x1, y0, y1 (full height)
LIDAR_KG, CAM_KG, G = 0.230, 0.030, 9.81        # spec / E

def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

bolts = [(BOLT_R * np.cos(np.radians(BOLT_ROT + a)), BOLT_R * np.sin(np.radians(BOLT_ROT + a))) for a in (0, 90, 180, 270)]

# ---- drawn
boss = cyl(0, 0, -SOCK_H, 0, BOSS_R, 96)
pads = sum((cyl(x, y, 0, PLATE_T, PAD_R) for x, y in bolts), mf.Manifold())
cam_block = box(-CAM_X / 2, CAM_X / 2, CAM_Y[0], CAM_Y[1], CAM_Z[0], CAM_Z[1])
drawn = boss + pads + cam_block

# ---- cuts: the exact ACME thread (OpenSCAD export, pole enters from the bottom), bolt holes + counterbores,
#      camera insert holes, the cable notch, the camera-bracket keep-out in front of the block, the pole below the boss
tc = os.path.join(HERE, "out", "thread_cut.stl")
if not os.path.exists(tc):                                     # out/ is not in git: render the thread body first
    os.makedirs(os.path.dirname(tc), exist_ok=True)
    os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
thread = man(trimesh.load(tc, force="mesh")).translate([0, 0, -SOCK_H])
cuts = thread
for x, y in bolts:
    cuts += cyl(x, y, -1, PLATE_T + 1, 1.7) + cyl(x, y, -1, 3.0, 3.1)
for sx in (-1, 1):
    cuts += mf.Manifold.cylinder(8.0, 2.0, 2.0, 32).rotate([-90, 0, 0]).translate([sx * CAM_HOLE_X, CAM_Y[1] - 6.5, CAM_HOLE_Z])
cuts += box(NOTCH[0], NOTCH[1], NOTCH[2], NOTCH[3], -SOCK_H - 1, PLATE_T + 1)
cuts += box(-24, 24, CAM_Y[1], PLATE_W / 2 + 1, -SOCK_H - 1, CAM_Z[1] + 1)          # camera bracket sits here
cuts += box(-PLATE_W, PLATE_W, -PLATE_W, PLATE_W, -SOCK_H - 10, -SOCK_H)           # pole / nothing below the boss

env = box(-PLATE_W / 2, PLATE_W / 2, -PLATE_W / 2, PLATE_W / 2, -SOCK_H, PLATE_T)  # nothing above the lidar seat

pp = PartProblem("scan_stick_head", H, drawn, cuts, env, build="-z", overhang=False)
# support: the boss wall round the thread bore (the pole holds it)
pp.support("default", cyl(0, 0, -SOCK_H + 1, -6, 13.0, 64) - cyl(0, 0, -SOCK_H, -5, 10.0, 64))

pad_region = sum((cyl(x, y, 0, PLATE_T, PAD_R + 0.5) for x, y in bolts), mf.Manifold())
cam_region = box(-CAM_X / 2 - 0.5, CAM_X / 2 + 0.5, CAM_Y[0] - 0.5, CAM_Y[1] + 0.5, CAM_Z[0] - 0.5, CAM_Z[1] + 0.5)
dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
dirs += [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
for k, d in enumerate(dirs):                                   # 25 g knocks: lidar on the pads + the part's own weight
    pp.load(f"lidar 25 g {k}", pad_region, LIDAR_KG * G * 25 * d, accel=25 * d)
    pp.load(f"camera 25 g {k}", cam_region, CAM_KG * G * 25 * d)
pp.load("hand grab +x 20 N", pad_region, (20, 0, 0)); pp.load("hand grab -y 20 N", pad_region, (0, -20, 0))
pp.load("hand push -z 30 N", pad_region, (0, 0, -30))
for nm, v in (("poke cam +y", (0, 5, 0)), ("poke cam -z", (0, 0, -5)), ("poke cam +x", (5, 0, 0))):
    pp.load(nm, cam_region, v)

if not os.environ.get("NOX_SETUP_ONLY"):
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    res, mesh = run_part(pp, drawn, cuts, env, GEN, os.path.join(HERE, "out", "scan_stick_head_v2.stl"))
