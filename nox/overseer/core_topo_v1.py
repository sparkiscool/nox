"""Overseer core piece runs v1 (27 Oct, Spark: "begin the runs"): nox/topo setups for the two printed core pieces on the
requirements in `nox/dimensions.md` "System Overseer core" (round 5, PR 123), every position from `runs/overseer_core/v1/result.json`
and the assembly `nox/overseer/overseer_core.assembly.json` (never retyped), the mounting features exactly as the coupons v1 check them.

  hub deck   on the bottom plate, plate face on the bed (build +z). Drawn: 4 foot pads Ø12 x 6.5 (Ø3.4, M3 x 12 down into the
             sled), 4 Pi posts Ø7 to z 8 (Ø2.2 x 4, M2.5 self-tapping), and (v2) the UBEC and buck seat rails 6 x 6.5 with 2 zip-tie tunnels. Envelope: z 0..6.5 over the plate (Pi underside parts at
             z 6.5..8), inside |y| 20 ahead of x 56 (the arm session's front envelopes), the v5 arm meshes + 1 cut.
             Loads: Pi 55 g at its CG through the 4 posts (force + moment, least norm), UBEC 15 g and buck 6 g on their bays,
             35 g crash x 14, flight 4 g / 2 g, landing 6 g, USB plug 20 N at the Pi's aft ports, 50 N press on the posts.
  sled       under the bottom plate, plate face on the bed (build -z: printed top down). Drawn: 4 plate-bolt pads and 6 receiver
             bolt pads Ø12 x 6 (Ø4.0 inserts), the flow pod's pocket walls (MTF-02P 21.6 x 16 x 6.5 + 0.2 a side, lens window),
             2 bosses on the +y rib for the down camera owner's pod. Envelope: the plate layer x -115..58, |y| <= 56, z -8.1..-2.1,
             ribs beside the receiver |y| 34..56 to z -30, the flow pod column to the lens plane z -76.9. Cuts: the receiver and
             swap column, the XT60 pocket and lead slot, Ø9 driver holes under the 8 arm screws (whole key line), the camera
             owner's pod volume, the flow sensor's view cone.
             Loads: swap set + receiver 1000 g at their CG through the 6 receiver bolts (force + moment, least norm), the camera
             pod 27 g, 35 g crash x 14, flight 4 g / 2 g, landing 6 g, swap push 60 N / pull 50 N (L), 20 N poke on the pod tip.

python3 nox/overseer/core_topo_v1.py hub_deck|sled [h] [gen_cm3] [--no-git] [--no-coarse] [--lean] [--ver=v2]     (NOX_SETUP_ONLY=1: setup only)
sled v1 FAILED (6 attempts out of memory at the fine stage, 8.4..13.3 GB: the 5.7 mm filter is 4.6 cells at h 1.25); sled v2: h 1.6,
fine grid only (--no-coarse), flight cases left out (--lean): OOM-killed in 3 min (8.9 GB) and its 6.0 sled layer only routed 3.55 mm
members at h 1.6. Cause: the engine keeps F and U for every case on the full grid (19 loads + 126 tie cases). sled v3: h 1.25, coarse
stage on, --lean, --set tie_moments=0 --set held_ties=0 --set gate_ties=0 (46 cases).
"""
import json, os, sys
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from topo import Part, Settings, optimise
from topo.part import man

PIECE = sys.argv[1] if len(sys.argv) > 1 else "hub_deck"
if PIECE not in ("hub_deck", "sled"):
    sys.exit(__doc__)
args = [a for i, a in enumerate(sys.argv[2:], 2) if not a.startswith("-") and sys.argv[i - 1] != "--set"]
H = float(args[0]) if args else (1.0 if PIECE == "hub_deck" else 1.25)
GEN = float(args[1]) if len(args) > 1 else (10.0 if PIECE == "hub_deck" else 40.0)   # cm3 generated (E: slots 24 / 75 g, minus drawn)
GIT = "--no-git" not in sys.argv
COARSE = "--no-coarse" not in sys.argv          # sled v2: the 3.0 coarse grid went singular at h 1.5 (v1), so run the fine grid only
LEAN = "--lean" in sys.argv                     # sled v2: flight / landing cases left out, the 35 g crash cases cover them
VER = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--ver=")), "v1")
SETS = {}                                       # --set key=value: Settings fields (sled v3: tie_moments=0 held_ties=0 gate_ties=0, the
for i_, a_ in enumerate(sys.argv):              # engine keeps F and U for every case on the full grid: 145 cases x 1.7 M dofs ran out of memory)
    if a_ == "--set":
        k_, v_ = sys.argv[i_ + 1].split("=", 1); SETS[k_] = (v_ not in ("0", "false", "False")) if v_ in ("0", "1", "true", "false", "True", "False") else float(v_)

R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v1", "result.json")))
C = R["coupon"]
A = json.load(open(os.path.join(HERE, "overseer_core.assembly.json")))
M = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in A["parts"]}
PI_NM = "Pi 5 + Active Cooler"
G = 9.81e-3                                     # N per g per g
DIRS14 = [np.array(v, float) / np.linalg.norm(v) for v in
          [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] + [(a, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)]]
FLIGHT = {"flight 4 g": np.array([0, 0, 4.0]), "flight 2 g x": np.array([2.0, 0, 1]), "flight 2 g y": np.array([0, 2.0, 1]), "landing 6 g": np.array([0, 0, 6.0])}


def cyl(x, y, z0, z1, r, n=40):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def U(ms):
    return mf.Manifold.batch_boolean(list(ms), mf.OpType.Add)


def grown(name, d):
    """an assembly part grown by d (its convex pieces' hulls, offset): a keep-out"""
    t = M[name]
    hulls = [b.convex_hull for b in t.split(only_watertight=False) if len(b.faces) > 3] or [t.convex_hull]
    out = []
    for h_ in hulls:
        v = h_.vertices; c = v.mean(0); v2 = c + (v - c) * 1.0
        hh = trimesh.convex.convex_hull(np.vstack([v2 + d * np.array(s_) for s_ in [(1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1), (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)]]))
        out.append(man(hh))
    return U(out)


def rigid_split(pts, F, P):
    """least-norm forces at pts that carry a force F acting at point P (force and moment): the payload on a rigid link"""
    pts = np.asarray(pts, float); c = pts.mean(0); n = len(pts); Am = np.zeros((6, 3 * n))
    for i, q in enumerate(pts - c):
        Am[:3, 3 * i:3 * i + 3] = np.eye(3); Am[3:, 3 * i:3 * i + 3] = [[0, -q[2], q[1]], [q[2], 0, -q[0]], [-q[1], q[0], 0]]
    Mc = np.cross(np.asarray(P, float) - c, F)
    return (np.linalg.pinv(Am) @ np.r_[F, Mc]).reshape(n, 3)


# ================================================================ hub deck
def hub_deck_bodies():
    T = 6.5                                                                 # envelope / pad height: the gate's 6.25 member, under the Pi's parts (z 6.5)
    feet = [tuple(f) for f in R["feet_xy"]]
    ph = [tuple(p) for p in R["pi_holes"]]
    zp = C["z_pi"]
    # bay seats (hub deck v2: v1 stopped at x 48 and left the UBEC 34 mm unsupported): rails under the UBEC and the buck, the
    # contact faces they rest on (z 6.5) with zip-tie slots, drawn as must-haves; bars inside |y| 20 (the arm envelopes)
    ub, bk = M["UBEC 5 V"].bounds, M["buck 12 V"].bounds
    rails = [box(ub[0][0], ub[1][0], y - 3, y + 3, 0, T) for y in (ub[0][1] + 3, ub[1][1] - 3)]
    rails += [box(bk[0][0], bk[1][0], y - 3, y + 3, 0, T) for y in (bk[0][1] + 3, min(bk[1][1] - 3, 17.0))]
    rails += [box(ub[1][0] - 6, ub[1][0], ub[0][1], ub[1][1], 0, T), box(bk[0][0], bk[0][0] + 6, ub[0][1], bk[1][1], 0, T)]
    slots = [box(x - 1.5, x + 1.5, ub[0][1] - 1, bk[1][1] + 1, 2.0, 4.5) for x in (ub[0][0] + 12, ub[1][0] - 14)]   # zip-tie tunnels 3 x 2.5 under the seats
    drawn = U([cyl(x, y, 0, T, 6.0) for x, y in feet] + [cyl(x, y, 0, zp, 3.5) for x, y in ph] + rails)
    cuts = U([cyl(x, y, -1, T + 1, 1.7) for x, y in feet] + [cyl(x, y, zp - 4.0, zp + 1, 1.1) for x, y in ph] + slots
             + [grown(f"arm {i}", 1.0) for i in (1, 2, 3, 4)]
             + [box(56, 100, 20, 60, -1, 20), box(56, 100, -60, -20, -1, 20)])            # the arm session's front envelopes (27 Oct)
    env = box(-76, 84, -42, 42, 0, T)
    return drawn, cuts, env, feet, ph


def hub_deck_setup(h):
    drawn, cuts, env, feet, ph = hub_deck_bodies()
    p = Part("overseer_hub_deck", h, drawn, cuts, env, build="+z", material="PETG")
    p.support("default", U([cyl(x, y, -0.5, 6.0, 6.5) for x, y in feet]))
    pi_c = M[PI_NM].centroid
    posts = [np.array([x, y, C["z_pi"]]) for x, y in ph]
    post_reg = [cyl(x, y, C["z_pi"] - 3.0, C["z_pi"] + 0.5, 4.0) for x, y in ph]
    ub, bk = M["UBEC 5 V"], M["buck 12 V"]
    bay = lambda m: box(m.bounds[0][0], m.bounds[1][0], m.bounds[0][1], m.bounds[1][1], 0, 6.5)

    def payload(name, a, cls):
        f = rigid_split(posts, -55 * G * a, pi_c)
        p.load_multi(name, [(post_reg[i], f[i]) for i in range(4)] +
                     [(bay(ub), -15 * G * a), (bay(bk), -6 * G * a)], cls=cls, accel=-a)
    for k, d in enumerate(DIRS14):
        payload(f"crash 35 g {k}", 35 * d, "crash")
    for nm, a in FLIGHT.items():
        payload(nm, a, "flight")
    usb = rigid_split(posts, np.array([20.0, 0, 0]), (C["pi_x"][0] - 2, 0, C["z_pi"] + 9))           # plug pushed in at the aft ports
    p.load_multi("USB plug 20 N into the Pi's aft ports", [(post_reg[i], usb[i]) for i in range(4)], cls="misuse")
    p.load_multi("press 50 N on the Pi (assembly)", [(post_reg[i], np.array([0, 0, -12.5])) for i in range(4)], cls="misuse")
    return p


# ================================================================ sled
def sled_bodies():
    zt, zb = C["z_plate_bot"], C["z_plate_bot"] - C["sled_t"]               # -2.1 / -8.1
    feet = [tuple(f) for f in R["feet_xy"]]
    rec = [tuple(b) for b in R["receiver_bolts"]]
    fx, fy = C["flow"]
    lens = R["z"]["lens"]
    cx, cy, cz = 21.6 + 0.4, 16.0 + 0.4, 6.5 + 0.3
    cam = [(-26.0, 50.0), (6.0, 50.0)]                                       # camera owner's pod interface on the +y rib (DEFAULT)
    pocket = box(fx - cx / 2 - 2.5, fx + cx / 2 + 2.5, fy - cy / 2 - 2.5, fy + cy / 2 + 2.5, lens, lens + cz + 2.5)
    drawn = U([cyl(x, y, zb, zt, 6.0) for x, y in feet + rec] + [pocket] + [cyl(x, y, zb - 6.0, zb, 6.0) for x, y in cam])
    rings = [tuple(a) for a in R["arm_screws_driver_holes"]]
    cuts = [cyl(x, y, zb - 1, zt + 1, 2.0) for x, y in feet + rec]                                  # Ø4.0 inserts through
    cuts += [cyl(x, y, zb - 7, zb + 0.5, 2.0) for x, y in cam]
    cuts += [cyl(x, y, -120, zt + 1, 4.5) for x, y in rings]                                         # Ø9 driver holes, whole key line
    cuts += [box(fx - cx / 2, fx + cx / 2, fy - cy / 2, fy + cy / 2, lens + 2.0, lens + cz + 2.0),   # sensor cavity (+ room above for the plug)
             box(fx - 8, fx + 8, fy - 6, fy + 6, lens - 1, lens + 2.5),                              # lens + ToF window
             box(fx - cx / 2 - 3.5, fx - cx / 2 + 0.5, fy - 4, fy + 4, lens + 2.0, lens + cz + 2.0)]   # SH1.0 cable slot
    rb = M["receiver"].bounds
    cuts += [box(rb[0][0] - 0.5, rb[1][0] + 0.5, rb[0][1] - 0.5, rb[1][1] + 0.5, -200, zb)]          # receiver + swap column
    xb = M["XT60 pair"].bounds
    cuts += [box(xb[0][0] - 1, xb[1][0] + 1, xb[0][1] - 1, xb[1][1] + 1, zb - 1, xb[1][2] + 0.5),      # XT60 male in its pocket
             box(xb[0][0] - 1, xb[1][0] + 1, -8.0, 8.0, zb - 1, zt + 1)]                             # its leads up past the plate's rear edge
    cuts += [box(-32, 12, 33.5, 80, -78, zb - 6.0)]                                                   # the camera owner's pod
    cuts += [box(fx - 40, fx + 40, fy - 40, fy + 40, -200, lens - 0.01)]                              # the flow sensor's view below the lens
    cuts = U(cuts)
    env = U([box(-115, 58, -56, 56, zb, zt), box(-115, 58, 34, 56, -30, zb), box(-115, 58, -56, -34, -30, zb),
             box(fx - 16, fx + 16, fy - 12.5, fy + 12.5, lens, zb)])
    return drawn, cuts, env, feet, rec, cam


def sled_setup(h):
    drawn, cuts, env, feet, rec, cam = sled_bodies()
    zt, zb = C["z_plate_bot"], C["z_plate_bot"] - C["sled_t"]
    p = Part("overseer_sled", h, drawn, cuts, env, build="-z", material="PETG")
    p.support("default", U([cyl(x, y, zb, zt + 0.5, 6.5) for x, y in feet]))
    rec_pts = [np.array([x, y, zb]) for x, y in rec]
    rec_reg = [cyl(x, y, zb - 0.5, zb + 3.0, 6.5) for x, y in rec]
    cam_reg = [cyl(x, y, zb - 6.5, zb - 3.0, 6.5) for x, y in cam]
    swap_cg = (M["pack"].centroid * 763 + M["receiver"].centroid * 237) / 1000.0
    cam_cg = np.array([-10.0, 55.0, -50.0])

    def payload(name, a, cls):
        f = rigid_split(rec_pts, -1000 * G * a, swap_cg)
        fc = rigid_split([np.array([x, y, zb - 6.0]) for x, y in cam], -27 * G * a, cam_cg)
        p.load_multi(name, [(rec_reg[i], f[i]) for i in range(6)] + [(cam_reg[i], fc[i]) for i in range(2)], cls=cls, accel=-a)
    for k, d in enumerate(DIRS14):
        payload(f"crash 35 g {k}", 35 * d, "crash")
    for nm, a in ({} if LEAN else FLIGHT).items():        # LEAN: 35 g x crash safety 1.1 = 38.5 g-equiv > 6 g x flight 4.0 = 24 (material.SAFETY)
        payload(nm, a, "flight")
    push = rigid_split(rec_pts, np.array([0, 0, 60.0]), swap_cg - np.array([0, 0, 35.0]))           # swap insert push from below (L)
    pull = rigid_split(rec_pts, np.array([0, 0, -50.0]), swap_cg - np.array([0, 0, 35.0]))
    p.load_multi("swap push 60 N", [(rec_reg[i], push[i]) for i in range(6)], cls="misuse")
    p.load_multi("swap pull 50 N", [(rec_reg[i], pull[i]) for i in range(6)], cls="misuse")
    fx, fy = C["flow"]; lens = R["z"]["lens"]
    tip = box(fx - 15, fx + 15, fy - 12, fy + 12, lens, lens + 4)
    for nm, v in (("+x", (20.0, 0, 0)), ("-y", (0, -20.0, 0)), ("+y", (0, 20.0, 0))):
        p.load(f"poke 20 N on the flow pod tip {nm}", tip, np.array(v), cls="misuse")
    return p


if __name__ == "__main__":
    setup = hub_deck_setup if PIECE == "hub_deck" else sled_setup
    out = os.path.join(ROOT, "runs", f"overseer_{PIECE}", VER)
    if os.environ.get("NOX_SETUP_ONLY"):
        p = setup(H)
        print(p.summary() if hasattr(p, "summary") else (p.shape, p.nel, int(p.design.sum()), int(p.solid.sum()), len(p.cases)))
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name=f"overseer_{PIECE}", settings=Settings(**SETS), git=GIT, coarse=COARSE)
