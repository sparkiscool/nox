"""Scan stick camera bracket v7: v6 stopped at iteration 44 with 75 % grey because its 4 mm plate layer is thinner than the
engine's robust plate (4.53 mm at the gate's 4.99 mm member): the eroded design split into 6 bodies (main 21 %, M) and the cable
pull blew up. v7 draws the plate layer 6 thick (C, DEFAULT; alternative: 4 mm plate with a drawn web), posts still 10 above it,
so the 10 mm under the board stays; bracket screws M3 x 12 (6 pad + 6 in the head's 7 deep insert holes), 4 cm3 generated (E).
v6: v5 wrapped the standoffs in generated web up to z 13 (r 8 on the mesh, M) and filled the space
between them, because the envelope reached the post tops. v6 keeps the envelope in the plate layer (z 0..4) and cuts the board's
footprint above the plate (38 x 38 + 1, z 4..19) as a keep-out round the posts: the standoffs stay Ø9 and the plug and parts on
the board's back have the full 10 mm. v5: nox/topo run (`scan_stick.md`). Spark 23 Oct: the camera sits on 10 mm standoffs (room for the
plug and parts on the back of the board), the standoffs as thin as the gate allows, M2.5 heat-set inserts in them (the board
takes M2.5, Spark; section M said M2 x 4 inserts (P), superseded). v5: posts Ø POST_D x 10 above the plate with Ø3.2 holes through
(E: M2.5 insert OD 3.5 x 5.7, hole 3.2, pressed 7 deep from the top; through so no part of the post is thicker than the plate) on a Ø12 x 4 foot pad in the plate layer; the bolt pads and the plate as v4.
   python3 cam_topo_v7.py [h] [gen_cm3] [--no-git] [--post-d D]
v2 / v3 failed the gate: posts under Ø12 round an M2 hole do not survive the 2.0 mm drawn-feature erosion. v4: Ø12 posts
through both layers (8 tall), 8 wide gussets, 4 + 4 thick.
Print frame: plate in XY, thickness Z, bed at z = 0. Tab end at +Y (bolts to the head's camera block, 2x M3 20 apart),
camera board on 4 standoffs on the DECXIN 28 x 28 M2 pattern (section M, P), lens up (+z), USB slot below the board.
Drawn: 2 bolt pads Ø12 x 4 (rule: pads >= Ø12, 2.0 web), 4 posts Ø12 through plate + standoff (8 tall), an 8 x 10 gusset bar
from each post toward the tab (rule: a gusset into the body). Cuts: M3 Ø3.4, M2 Ø1.8, USB slot, the board keep-out above the posts.
Loads (DEFAULT, E): 30 g camera at 25 g knocks (14 dirs) and 50 g impact (6 dirs), 10 N pokes on each post, 20 N pull on the
cable slot. Support: the two bolt pads (clamped to the block)."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 0.75
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 4.0   # cm3 generated (DEFAULT, E; v7)
GIT = "--no-git" not in sys.argv

W, L, T, POST = 44.0, 60.0, 6.0, 10.0           # C: plate 44 x 60 x 6 (v7), posts 10 above the plate (Spark 23 Oct)
POST_D = float(sys.argv[sys.argv.index('--post-d') + 1]) if '--post-d' in sys.argv else 9.0   # C: post Ø (2.9 wall round the Ø3.2 insert hole)
INS_D = 3.2                                     # E: M2.5 heat-set OD 3.5 x 5.7, hole Ø3.2 through, insert pressed 7 deep
BOLT_Y, BOLT_X = L - 7.0, 10.0                  # C: M3 holes 7 from the tab end, 20 apart
CAM_C = (0.0, L - 14.0 - 25.0)                  # C: camera centre 25 below the tab (cam_cz = -(46/2) - 2 from the holes)
CAM_P = 28.0 / 2                                # P: 28 x 28 pattern, M2.5 (Spark 23 Oct)
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
        drawn += cyl(x, y, 0, T, 6.0) + cyl(x, y, 0, T + POST, POST_D / 2)   # Ø12 foot pad in the plate, the post above it
        drawn += box(x - 4, x + 4, y, y + 10, 0, T)                      # gusset bar toward the tab (+y)
    cuts = mf.Manifold()
    for x, y in bolts:
        cuts += cyl(x, y, -1, T + POST + 1, 1.7)
    for x, y in posts:
        cuts += cyl(x, y, -1, T + POST + 1, INS_D / 2)                  # Ø3.2 through post + plate (a blind hole leaves a solid foot > 4 thick that fails the gate's drawn-core test)
    cuts += box(-8, 8, CAM_C[1] - CAM_P - 8, CAM_C[1] - CAM_P - 4, -1, T + POST + 1)           # USB slot
    keep = box(-19.5, 19.5, CAM_C[1] - 19.5, CAM_C[1] + 19.5, T, T + POST + 5)                # the board's back and the board itself: free above the plate
    for x, y in posts:
        keep -= cyl(x, y, T - 1, T + POST + 6, POST_D / 2 + 0.5)                                # except the standoffs
    cuts += keep
    env = box(-W / 2, W / 2, 0, L, 0, T)                                                       # v6/v7: generated web in the plate layer only (Part adds the drawn posts)
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
        post_region += cyl(x, y, T, T + POST + 0.5, 6.5)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs + diag):
        p.load(f"camera 25 g {k}", post_region, CAM_KG * G * 25 * d, accel=25 * d)
    for k, d in enumerate(dirs):
        p.load(f"impact 50 g {k}", post_region, CAM_KG * G * 50 * d, accel=50 * d)
    for k, (x, y) in enumerate(posts):
        p.load(f"poke post {k} -z 10 N", cyl(x, y, T, T + POST + 0.5, 6.5), np.array((0, 0, -10.0)))
        p.load(f"poke post {k} +x 10 N", cyl(x, y, T, T + POST + 0.5, 6.5), np.array((10.0, 0, 0)))
    slot_region = box(-9, 9, CAM_C[1] - CAM_P - 9, CAM_C[1] - CAM_P - 3, -0.5, T + 0.5)
    p.load("cable pull -y 20 N", slot_region, np.array((0, -20.0, 0)))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick_cam", "v7")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name="scan_stick_cam", settings=Settings(finish="drawn"), git=GIT)
