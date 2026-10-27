"""Overseer core piece runs v2 (27 Oct, core v2: the hub deck with the keel front hub; the flow clip is drawn, no run; no sled,
the battery side is an empty slot): the nox/topo setup on the requirements in `nox/dimensions.md` "System Overseer core" (v2), every
position from `runs/overseer_core/v2/result.json` and the assembly `nox/overseer/overseer_core.assembly.json` (never retyped).

  hub deck   on the bottom plate, plate face on the bed (build +z). Drawn: 6 feet Ø12 x 6.5, each with a Ø4.0 x 5.7 blind insert
             hole from the plate side (M3 up from under the plate); 4 Pi posts Ø7 to z 8 (Ø2.2 x 4, M2.5 self-tapping); the UBEC and
             buck seat rails 6 x 6.5 with 2 zip-tie tunnels; the keel front hub: 2 bosses Ø13 on the keel axes from 2 behind the v5
             roots (96, ±12, 8) to 20 past them, bores 6.2 x 15 from 5 past (epoxied 6/4 tube), seated on the front feet.
             Envelope: z 0..6.5 over the plate, x -76..100, plus the hub room x 80..118, |y| 20, z 0..16 round the bosses (smoke 27 Oct: without it the bosses could not be joined), inside |y| 20 ahead of
             x 56 (the arm session's front envelopes), the v5 arm meshes + 1 and the keel tubes + 0.5 cut.
             Loads: Pi 55 g at its CG through the 4 posts (force + moment, least norm), UBEC 15 g and buck 6 g on their bays,
             35 g crash x 14, flight 4 g / 2 g, landing 6 g, USB plug 20 N at the Pi's aft ports, 50 N press on the posts, and the
             keels on their bores: both push 387 N, both pull 274 N, one each (frame model maxima, F; crash class) with 35 g up.

python3 nox/overseer/core_topo_v2.py hub_deck [h] [gen_cm3] [--no-git] [--no-coarse] [--lean] [--ver=v3] [--set key=value]   (NOX_SETUP_ONLY=1: setup only)
Measure first (CLAUDE.md Measure before quoting): python3 tools/run_estimate.py runs/overseer_hub_deck/<ver> -- python3 nox/overseer/core_topo_v2.py hub_deck ...
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
GEN = float(args[1]) if len(args) > 1 else 12.0   # cm3 generated (E: slot 32 g, minus drawn)
GIT = "--no-git" not in sys.argv
COARSE = "--no-coarse" not in sys.argv          # sled v2: the 3.0 coarse grid went singular at h 1.5 (v1), so run the fine grid only
LEAN = "--lean" in sys.argv                     # sled v2: flight / landing cases left out, the 35 g crash cases cover them
VER = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--ver=")), "v3")
SETS = {}                                       # --set key=value: Settings fields (sled v3: tie_moments=0 held_ties=0 gate_ties=0, the
for i_, a_ in enumerate(sys.argv):              # engine keeps F and U for every case on the full grid: 145 cases x 1.7 M dofs ran out of memory)
    if a_ == "--set":
        k_, v_ = sys.argv[i_ + 1].split("=", 1); SETS[k_] = (v_ not in ("0", "false", "False")) if v_ in ("0", "1", "true", "false", "True", "False") else float(v_)

R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v2", "result.json")))
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
KEELS = {int(k): (np.array(v[0], float), np.array(v[1], float)) for k, v in R["keels"].items()}
SK = R["socket"]; INS = R["insert"]; T = R["foot_h"]
KEEL_N = R["keel_n"]
FOOT_R = 7.0                                                               # feet Ø14: 5 of wall round the Ø4.0 insert hole (the gate erodes 2.0; v3's Ø12 left 4.0)                                                       # max pull / push per keel (F)


def along(a, d, l0, l1, r, n=40):
    """a cylinder of radius r on the axis a + d t, t in l0..l1"""
    q0, q1 = a + d * l0, a + d * l1
    t = trimesh.creation.cylinder(r, l1 - l0, sections=n, segment=np.vstack([q0, q1]))
    return man(t)


def arm_env(deg):
    """the arm session's root envelope along the arm at deg (27 Oct: roots u >= 58 for |v| <= 21); v3 used a box |y| > 20 ahead of
    x 56, which cut the mid feet (65, ±22.5) to a 3.5 sliver and capped every member at 4.44 (the solid gate FAILED)"""
    return box(58.0, 400.0, -21.0, 21.0, -1.0, 20.0).rotate([0, 0, deg])


def keel_axis(sy):
    a, b = KEELS[sy]; return a, (b - a) / np.linalg.norm(b - a)


def hub_deck_bodies():
    feet = [tuple(f) for f in R["feet_xy"]]
    ph = [tuple(p) for p in R["pi_holes"]]
    zp = C["z_pi"]
    # bay seats (hub deck v2): rails under the UBEC and the buck with zip-tie tunnels, inside |y| 20 (the arm envelopes)
    ub, bk = M["UBEC 5 V"].bounds, M["buck 12 V"].bounds
    rails = [box(ub[0][0], ub[1][0], y - 3, y + 3, 0, T) for y in (ub[0][1] + 3, ub[1][1] - 3)]
    rails += [box(bk[0][0], bk[1][0], y - 3, y + 3, 0, T) for y in (bk[0][1] + 3, min(bk[1][1] - 3, 17.0))]
    rails += [box(ub[1][0] - 6, ub[1][0], ub[0][1], ub[1][1], 0, T), box(bk[0][0], bk[0][0] + 6, ub[0][1], bk[1][1], 0, T)]
    # zip-tie channels 3 x 2.5 open on the plate side (the tie runs between deck and plate), 4.0 of deck above: v2's tunnels at z 2..4.5
    # left 2.0 skins across the whole deck, and the smoke (27 Oct) found the front hub cut off from the rest at a 3.2 member
    # v4: no zip-tie channels: the UBEC and the buck sit on the seat rails on 3M VHB foam tape (DEFAULT; ~576 mm² a bay against ~5 N in
    # a 35 g crash). Any channel under the bays crosses the whole deck and leaves < 6.25 of it above, which cut the front hub off from
    # the rest at the solid gate's member (v4 smoke 27 Oct: gate members capped at 4.96)
    slots = []
    # keel front hub: bosses on the keel axes, seated on the front feet (the keels' 387 N go into the plate at (96, ±17))
    bosses = [along(*keel_axis(sy), -2.0, SK["off"] + SK["l"], SK["od"] / 2) for sy in (1, -1)]
    seats = []                                              # v3: the topology grows the seats (the hub's room below)
    drawn = U([cyl(x, y, 0, T, FOOT_R) for x, y in feet] + [cyl(x, y, 0, zp, 3.5) for x, y in ph] + rails + bosses + seats)
    bores = [along(*keel_axis(sy), SK["off"], SK["off"] + SK["l"] + 1.0, SK["d"] / 2) for sy in (1, -1)]
    tubes = [along(*keel_axis(sy), SK["off"] + SK["l"] - 0.5, 400.0, 3.5) for sy in (1, -1)]   # the keel tube past the socket mouth + 0.5 (0.5 lead-in: no end slivers)
    ties = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "result.json")))["links"]   # the cage's front core ties (parked body node)
    tie_cut = [man(trimesh.creation.cylinder(4.0, np.linalg.norm(np.subtract(b, a)), sections=24, segment=np.array([a, b], float)))
               for k, (a, b) in ties.items() if k.startswith("core tie front")]
    hub_env = box(80.0, 118.0, -20.0, 20.0, 0, 16.0)        # the keel front hub's room: gussets from each socket back to the feet and across
    cuts = U([cyl(x, y, -1, T + 1, INS[0] / 2) for x, y in feet] + tie_cut + [cyl(x, y, zp - 4.0, zp + 1, 1.1) for x, y in ph] + slots + bores + tubes
             + [grown(f"arm {i}", 1.0) for i in (1, 2, 3, 4)]
             + [arm_env(a_) for a_ in (45, -45, 135, -135)])                                   # the arm session's root envelopes (u >= 58, |v| <= 21)
    env = U([box(-76, 100, -42, 42, 0, T), hub_env])
    return drawn, cuts, env, feet, ph


def hub_deck_setup(h):
    drawn, cuts, env, feet, ph = hub_deck_bodies()
    p = Part("overseer_hub_deck", h, drawn, cuts, env, build="+z", material="PETG")
    p.support("default", U([cyl(x, y, -0.5, 3.0, FOOT_R + 0.5) for x, y in feet]))
    pi_c = M[PI_NM].centroid
    posts = [np.array([x, y, C["z_pi"]]) for x, y in ph]
    post_reg = [cyl(x, y, C["z_pi"] - 3.0, C["z_pi"] + 0.5, 4.0) for x, y in ph]
    ub, bk = M["UBEC 5 V"], M["buck 12 V"]
    bay = lambda m: box(m.bounds[0][0], m.bounds[1][0], m.bounds[0][1], m.bounds[1][1], 0, 6.5)
    walls = {sy: along(*keel_axis(sy), SK["off"], SK["off"] + SK["l"], SK["d"] / 2 + 1.0) for sy in (1, -1)}   # the bore wall (epoxy)

    def payload(name, a, cls, keel=None):
        f = rigid_split(posts, -55 * G * a, pi_c)
        reg = [(post_reg[i], f[i]) for i in range(4)] + [(bay(ub), -15 * G * a), (bay(bk), -6 * G * a)]
        if keel:
            for sy, sg in zip((1, -1), keel):
                d = keel_axis(sy)[1]
                reg.append((walls[sy], d * (KEEL_N[0] if sg > 0 else KEEL_N[1]) * sg))
        p.load_multi(name, reg, cls=cls, accel=-a)
    for k, d in enumerate(DIRS14):
        payload(f"crash 35 g {k}", 35 * d, "crash")
    if not LEAN:
        for nm, a in FLIGHT.items():
            payload(nm, a, "flight")
    up = np.array([0, 0, 35.0])
    payload("keels push 387 N each (F) + 35 g up", up, "crash", keel=(-1, -1))
    payload("keels pull 274 N each (F) + 35 g up", up, "crash", keel=(1, 1))
    payload("keel +y push, -y pull (F) + 35 g up", up, "crash", keel=(-1, 1))
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
