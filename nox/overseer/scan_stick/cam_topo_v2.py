"""Scan stick camera bracket v2: nox/topo run (`scan_stick.md`).   python3 cam_topo_v2.py [h] [gen_cm3] [--no-git]
Print frame: plate in XY, thickness Z, bed at z = 0. Tab end at +Y (bolts to the head's camera block, 2x M3 20 apart),
camera board on 4 standoffs on the DECXIN 28 x 28 M2 pattern (section M, P), lens up (+z), USB slot below the board.
Drawn: 2 bolt pads Ø12 x 3 (rule: pads >= Ø12, 2.0 web), 4 standoff pads Ø8 x 3 + Ø6 x 3 posts, a 6 x 6 gusset bar from each
standoff toward the tab (rule: a gusset into the body). Cuts: M3 Ø3.4, M2 Ø1.8, USB slot, the board keep-out above the posts.
Loads (DEFAULT, E): 30 g camera at 25 g knocks (14 dirs) and 50 g impact (6 dirs), 10 N pokes on each post, 20 N pull on the
cable slot. Support: the two bolt pads (clamped to the block)."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 0.75
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 2.5   # cm3 generated (DEFAULT, E)
GIT = "--no-git" not in sys.argv

W, L, T, POST = 44.0, 60.0, 3.0, 3.0            # C: plate 44 x 60 x 3, posts 3 tall
BOLT_Y, BOLT_X = L - 7.0, 10.0                  # C: M3 holes 7 from the tab end, 20 apart
CAM_C = (0.0, L - 14.0 - 25.0)                  # C: camera centre 25 below the tab (cam_cz = -(46/2) - 2 from the holes)
CAM_P = 28.0 / 2                                # spec P: 28 x 28 M2 pattern
CAM_KG, G = 0.030, 9.81

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

posts = [(CAM_C[0] + sx * CAM_P, CAM_C[1] + sy * CAM_P) for sx in (-1, 1) for sy in (-1, 1)]
bolts = [(-BOLT_X, BOLT_Y), (BOLT_X, BOLT_Y)]

def bodies():
    drawn = mf.Manifold()
    for x, y in bolts:
        drawn += cyl(x, y, 0, T, 6.0)
    for x, y in posts:
        drawn += cyl(x, y, 0, T, 4.0) + cyl(x, y, T, T + POST, 3.0)
        drawn += box(x - 3, x + 3, y, y + 8, 0, T)                       # gusset bar toward the tab (+y) / the plate
    cuts = mf.Manifold()
    for x, y in bolts:
        cuts += cyl(x, y, -1, T + POST + 1, 1.7)
    for x, y in posts:
        cuts += cyl(x, y, -1, T + POST + 1, 0.9)
    cuts += box(-8, 8, CAM_C[1] - CAM_P - 8, CAM_C[1] - CAM_P - 4, -1, T + POST + 1)           # USB slot
    cuts += box(-19.5, 19.5, CAM_C[1] - 19.5, CAM_C[1] + 19.5, T + POST, T + POST + 5)        # board sits on the posts
    env = box(-W / 2, W / 2, 0, L, 0, T + POST)
    return dict(drawn=drawn, cuts=cuts, env=env)

def setup(h):
    B = bodies()
    p = Part("scan_stick_cam", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    sup = mf.Manifold()
    for x, y in bolts:
        sup += cyl(x, y, -1, T + 0.5, 6.5)
    p.support("default", sup)
    post_region = mf.Manifold()
    for x, y in posts:
        post_region += cyl(x, y, T, T + POST + 0.5, 3.5)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs + diag):
        p.load(f"camera 25 g {k}", post_region, CAM_KG * G * 25 * d, accel=25 * d)
    for k, d in enumerate(dirs):
        p.load(f"impact 50 g {k}", post_region, CAM_KG * G * 50 * d, accel=50 * d)
    for k, (x, y) in enumerate(posts):
        p.load(f"poke post {k} -z 10 N", cyl(x, y, T, T + POST + 0.5, 3.5), np.array((0, 0, -10.0)))
        p.load(f"poke post {k} +x 10 N", cyl(x, y, T, T + POST + 0.5, 3.5), np.array((10.0, 0, 0)))
    slot_region = box(-9, 9, CAM_C[1] - CAM_P - 9, CAM_C[1] - CAM_P - 3, -0.5, T + 0.5)
    p.load("cable pull -y 20 N", slot_region, np.array((0, -20.0, 0)))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick_cam", "v2")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name="scan_stick_cam", settings=Settings(), git=GIT)
