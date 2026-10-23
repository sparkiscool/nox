"""Scan stick head v3: nox/topo run of the lidar head (`scan_stick.md`). v2 (noxopt, 20 cm3 generated) filled a cone at a
0.03 index and its pads had no drawn gusset; v3 draws the attachments the CLAUDE.md rule asks for and gives the optimiser
little material, so members form.   python3 head_topo_v3.py [h] [gen_cm3] [--no-git]
Drawn (mounting + must-haves): 3/4-5 ACME pole boss (Ø32 x 34, C), 4 L2 bolt pads Ø12 x 6 on Ø51 at 22.5° (spec) each with
a drawn 6 x 8 gusset into the boss (rule: pads >= Ø12, a gusset into the body), camera block 32 x 8 x 30 (C).
Cuts: the exact thread, bolt holes + counterbores, camera insert holes, the L2 cable notch, the camera-bracket keep-out,
the pole below the boss. Envelope 86 x 86 from z -34 to the lidar seat at +6 (nothing above the seat: L2 hemisphere FOV).
Loads (DEFAULT, E): 25 g on the 230 g lidar in 14 directions + self weight, 20 N hand grab, 30 N hand push, 25 g on a 30 g
camera, 5 N pokes. Support: the boss wall round the thread (the pole). Build "-z": seat on the bed."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.part import man

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 1.25
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 5.0   # cm3 generated (DEFAULT, E)
GIT = "--no-git" not in sys.argv

BOSS_R, SOCK_H = 16.0, 34.0
PLATE_T, PLATE_W = 6.0, 86.0
BOLT_R, BOLT_ROT = 51.0 / 2, 22.5
PAD_R = 6.0
GUSSET_W, GUSSET_Z = 6.0, (-4.0, PLATE_T)          # 6 wide, 10 tall, pad -> boss
CAM_X, CAM_Y, CAM_Z = 32.0, (12.0, 20.0), (-32.0, -2.0)
CAM_HOLE_Z, CAM_HOLE_X = -17.0, 10.0
NOTCH = (-PLATE_W / 2 - 1, -PLATE_W / 2 + 12.5, -7.0, 7.0)
LIDAR_KG, CAM_KG, G = 0.230, 0.030, 9.81

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

angles = [BOLT_ROT + a for a in (0, 90, 180, 270)]
bolts = [(BOLT_R * np.cos(np.radians(a)), BOLT_R * np.sin(np.radians(a))) for a in angles]

def bodies():
    boss = cyl(0, 0, -SOCK_H, 0, BOSS_R, 96)
    pads = sum((cyl(x, y, 0, PLATE_T, PAD_R) for x, y in bolts), mf.Manifold())
    gussets = mf.Manifold()
    for a in angles:                                        # a bar from the pad centre to the boss axis, 6 wide, 10 tall
        g = box(10.0, BOLT_R, -GUSSET_W / 2, GUSSET_W / 2, GUSSET_Z[0], GUSSET_Z[1]).rotate([0, 0, a])
        gussets += g
    cam_block = box(-CAM_X / 2, CAM_X / 2, CAM_Y[0], CAM_Y[1], CAM_Z[0], CAM_Z[1])
    drawn = boss + pads + gussets + cam_block
    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    thread = man(trimesh.load(tc, force="mesh")).translate([0, 0, -SOCK_H])
    cuts = thread
    for x, y in bolts:
        cuts += cyl(x, y, -1, PLATE_T + 1, 1.7) + cyl(x, y, -1, 3.0, 3.1)
    for sx in (-1, 1):
        cuts += mf.Manifold.cylinder(8.0, 2.0, 2.0, 32).rotate([-90, 0, 0]).translate([sx * CAM_HOLE_X, CAM_Y[1] - 6.5, CAM_HOLE_Z])
    cuts += box(NOTCH[0], NOTCH[1], NOTCH[2], NOTCH[3], -SOCK_H - 1, PLATE_T + 1)
    cuts += box(-24, 24, CAM_Y[1], PLATE_W / 2 + 1, -SOCK_H - 1, CAM_Z[1] + 1)
    cuts += box(-PLATE_W, PLATE_W, -PLATE_W, PLATE_W, -SOCK_H - 10, -SOCK_H)
    env = box(-PLATE_W / 2, PLATE_W / 2, -PLATE_W / 2, PLATE_W / 2, -SOCK_H, PLATE_T)
    return dict(boss=boss, pads=pads, gussets=gussets, cam=cam_block, drawn=drawn, cuts=cuts, env=env)

def setup(h):
    B = bodies()
    p = Part("scan_stick_head", h, B["drawn"], B["cuts"], B["env"], build="-z", material="PETG")
    p.support("default", cyl(0, 0, -SOCK_H + 1, -6, 13.0, 64) - cyl(0, 0, -SOCK_H, -5, 10.0, 64))
    pad_region = sum((cyl(x, y, 0, PLATE_T, PAD_R + 0.5) for x, y in bolts), mf.Manifold())
    cam_region = box(-CAM_X / 2 - 0.5, CAM_X / 2 + 0.5, CAM_Y[0] - 0.5, CAM_Y[1] + 0.5, CAM_Z[0] - 0.5, CAM_Z[1] + 0.5)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs):
        p.load(f"lidar 25 g {k}", pad_region, LIDAR_KG * G * 25 * d, accel=25 * d)
        p.load(f"camera 25 g {k}", cam_region, CAM_KG * G * 25 * d)
    p.load("hand grab +x 20 N", pad_region, np.array((20, 0, 0), float))
    p.load("hand grab -y 20 N", pad_region, np.array((0, -20, 0), float))
    p.load("hand push -z 30 N", pad_region, np.array((0, 0, -30), float))
    for nm, v in (("poke cam +y", (0, 5, 0)), ("poke cam -z", (0, 0, -5)), ("poke cam +x", (5, 0, 0))):
        p.load(nm, cam_region, np.array(v, float))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick", "v3")
    if os.environ.get("NOX_SETUP_ONLY"):
        p = setup(H); print(p.summary() if hasattr(p, "summary") else p)
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name="scan_stick_head", settings=Settings(), git=GIT)
