"""Overseer hub deck v12 (core v4, 27 Oct): the cage hangs the L2 head on 3 rods to its own nodes (Spark 27 Oct: "max of 3" rods to
the lidar), so the deck carries no head load: no sockets, no tower (v9..v11 superseded). nox/topo setup on
`runs/overseer_core/v4/result.json` and the assembly `nox/overseer/overseer_core.assembly.json`.

  hub deck   on the bottom plate, plate face on the bed (build +z). Drawn: 6 feet Ø14 x 6.5 with Ø4.0 insert holes through; 4 Pi
             posts Ø7 to z 8 on Ø12 x 6 root webs; UBEC / buck seat pads Ø12, 4 a bay (VHB tape). Envelope: z 0..6.5 over the plate x -76..100,
             |y| <= 42; cuts: the UBEC / buck columns (placed straight down), the arm session's root envelopes and grown arms, the
             the core's wire routes, the top plate.
             Loads: the payload (Pi 55 g at its CG through the posts, UBEC 15, buck 6, the deck wires 20 g on the bays) at 35 g in 14 directions (crash), flight 4 g,
             2 g x / y, landing 6 g; USB plug 20 N; 50 N press on the Pi.

python3 nox/overseer/core_topo_v4.py hub_deck [h] [gen_cm3] [--no-git] [--no-coarse] [--lean] [--ver=v12] [--set key=value]   (NOX_SETUP_ONLY=1: setup only)
Measure first: python3 tools/run_estimate.py runs/overseer_hub_deck/<ver> -- python3 nox/overseer/core_topo_v4.py hub_deck ...
"""
import json, os, sys
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from topo import Part, Settings, optimise
from topo.part import man

PIECE = sys.argv[1] if len(sys.argv) > 1 else "hub_deck"
if PIECE != "hub_deck":
    sys.exit(__doc__)
args = [a for i, a in enumerate(sys.argv[2:], 2) if not a.startswith("-") and sys.argv[i - 1] != "--set"]
H = float(args[0]) if args else 1.0
GEN = float(args[1]) if len(args) > 1 else 11.0   # cm3 generated (v12: payload loads only, worst bolt 13.5 N; min member governs; pads not rails, so 11 (review), E)
GIT = "--no-git" not in sys.argv
COARSE = "--no-coarse" not in sys.argv          # sled v2: the 3.0 coarse grid went singular at h 1.5 (v1), so run the fine grid only
LEAN = "--lean" in sys.argv                     # sled v2: flight / landing cases left out, the 35 g crash cases cover them
VER = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--ver=")), "v12")
SETS = {}                                       # --set key=value: Settings fields (sled v3: tie_moments=0 held_ties=0 gate_ties=0, the
for i_, a_ in enumerate(sys.argv):              # engine keeps F and U for every case on the full grid: 145 cases x 1.7 M dofs ran out of memory)
    if a_ == "--set":
        k_, v_ = sys.argv[i_ + 1].split("=", 1); SETS[k_] = (v_ not in ("0", "false", "False")) if v_ in ("0", "1", "true", "false", "True", "False") else float(v_)

R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v4", "result.json")))
C = R["coupon"]
_CB = os.path.join(ROOT, "runs", "overseer_core", "v4", "assembly_on_cage_branch.json")   # core v4 on the cage branch's meshes (head rods, arms), while it exists
A = json.load(open(os.environ.get("CORE_ASSEMBLY") or (_CB if os.path.exists(_CB) else os.path.join(HERE, "overseer_core.assembly.json"))))   # CORE_ASSEMBLY: core v4's manifest on the cage branch (runs/overseer_core/v4/assembly_on_cage_branch.json)
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
INS = R["insert"]; T = R["foot_h"]
Z_UNDER = 45.0                                                             # the top plate underside (S500 STEP)
FOOT_R = 7.0                                                               # feet Ø14: 5 of wall round the Ø4.0 insert hole (the gate erodes 2.0; v3's Ø12 left 4.0)


def along(a, d, l0, l1, r, n=40):
    """a cylinder of radius r on the axis a + d t, t in l0..l1"""
    q0, q1 = a + d * l0, a + d * l1
    t = trimesh.creation.cylinder(r, l1 - l0, sections=n, segment=np.vstack([q0, q1]))
    return man(t)


def arm_env(deg):
    """the arm session's root envelope along the arm at deg (27 Oct: roots u >= 58 for |v| <= 21); v3 used a box |y| > 20 ahead of
    x 56, which cut the mid feet (65, ±22.5) to a 3.5 sliver and capped every member at 4.44 (the solid gate FAILED)"""
    return box(58.0, 400.0, -21.0, 21.0, -1.0, 20.0).rotate([0, 0, deg])


def hub_deck_bodies():
    feet = [tuple(f) for f in R["feet_xy"]]
    ph = [tuple(p) for p in R["pi_holes"]]
    zp = C["z_pi"]
    # bay seats (v12, concept review 27 Oct: drawn is attachment only): 4 pads Ø12 x 6.5 under each bay's corners for its VHB tape
    # (~450 mm² a bay against ~9 N in a 35 g crash, E); v2..v11's continuous seat rails were load-carrying bars (7.6 cm³ drawn)
    ub, bk = M["UBEC 5 V"].bounds, M["buck 12 V"].bounds
    rails = [cyl(x, y, 0, T, 6.0) for b_ in (ub, bk) for x in (b_[0][0] + 6.0, b_[1][0] - 6.0) for y in (b_[0][1] + 6.0, b_[1][1] - 6.0)]
    slots = []
    # v5 (PR 148 review): a root web Ø12 x 6.0 under every Pi post (a gusset into the body; v4's bare Ø7 posts were 99.8 % in the main
    # body at the gate's 2.0 erosion, under its 99.9 % bar); 6.0 stays under the board's underside parts (z 6.5..8)
    roots = [cyl(x, y, 0, 6.0, 6.0) for x, y in ph]
    drawn = U([cyl(x, y, 0, T, FOOT_R) for x, y in feet] + [cyl(x, y, 0, zp, 3.5) for x, y in ph] + roots + rails)
    wires = [U([man(b) for b in M["wires"].split(only_watertight=False) if len(b.faces) > 3])]   # the core's wire routes, exact tubes (a hull per
    # route would be a slab across the deck); the flow lead crosses it
    plates = [man(M["top plate"])]
    bays = [box(m.bounds[0][0] - 1, m.bounds[1][0] + 1, m.bounds[0][1] - 1, m.bounds[1][1] + 1, T, Z_UNDER) for m in (M["UBEC 5 V"], M["buck 12 V"])]
    cuts = U(plates + bays + [cyl(x, y, -1, T + 1, INS[0] / 2) for x, y in feet] + [cyl(x, y, zp - 4.0, zp + 1, 1.1) for x, y in ph] + slots + wires
             + [grown(f"arm {i}", 1.0) for i in (1, 2, 3, 4)]
             + [arm_env(a_) for a_ in (45, -45, 135, -135)])                                   # the arm session's root envelopes (u >= 58, |v| <= 21)
    env = box(-76, 100, -42, 42, 0, T)
    return drawn, cuts, env, feet, ph


def hub_deck_setup(h):
    drawn, cuts, env, feet, ph = hub_deck_bodies()
    p = Part("overseer_hub_deck", h, drawn, cuts, env, build="+z", material="PETG")
    foot_sup = [cyl(x, y, -0.5, 3.0, FOOT_R + 0.5) for x, y in feet]
    p.support("default", U(foot_sup))
    pi_c = M[PI_NM].centroid
    posts = [np.array([x, y, C["z_pi"]]) for x, y in ph]
    post_reg = [cyl(x, y, C["z_pi"] - 3.0, C["z_pi"] + 0.5, 4.0) for x, y in ph]
    ub, bk = M["UBEC 5 V"], M["buck 12 V"]
    bay = lambda m: U([cyl(x, y, T - 2.0, T + 0.5, 6.0) for x in (m.bounds[0][0] + 6.0, m.bounds[1][0] - 6.0) for y in (m.bounds[0][1] + 6.0, m.bounds[1][1] - 6.0)])   # its 4 pads' tops (VHB)

    def payload(name, a, cls):
        f = rigid_split(posts, -55 * G * a, pi_c)
        reg = [(post_reg[i], f[i]) for i in range(4)] + [(bay(ub), -(15 + 10) * G * a), (bay(bk), -(6 + 10) * G * a)]   # + the deck wires' 20 g, half on each bay (E)
        p.load_multi(name, reg, cls=cls, accel=-a)
    for k, d in enumerate(DIRS14):
        payload(f"crash {k} (35 g)", 35 * d, "crash")
    if not LEAN:
        for nm, a in FLIGHT.items():
            payload(nm, a, "flight")
    usb = rigid_split(posts, np.array([20.0, 0, 0]), (C["pi_x"][0] - 2, 0, C["z_pi"] + 9))           # plug pushed in at the aft ports
    p.load_multi("USB plug 20 N into the Pi's aft ports", [(post_reg[i], usb[i]) for i in range(4)], cls="misuse")
    p.load_multi("press 50 N on the Pi (assembly)", [(post_reg[i], np.array([0, 0, -12.5])) for i in range(4)], cls="misuse")
    return p


if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", f"overseer_{PIECE}", VER)
    if os.environ.get("NOX_SETUP_ONLY"):
        p = hub_deck_setup(H)
        print(p.summary() if hasattr(p, "summary") else (p.shape, p.nel, int(p.design.sum()), int(p.solid.sum()), len(p.cases)))
    else:
        optimise(hub_deck_setup, h=H, gen_cm3=GEN, out_dir=out, name=f"overseer_{PIECE}", settings=Settings(**SETS), git=GIT, coarse=COARSE)
