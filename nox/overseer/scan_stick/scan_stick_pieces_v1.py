"""Scan stick piece pass (docs/generative.md The rule, 0): head v14 and base v19, each its own nox/topo run with the system
pass's interfaces and joint reactions (`unit_topo_v3.py` is the system model and the system solve; dimensions.md "System scan
stick" holds the requirements). The cut is the bolted joint at z 168 (C): the head hangs from it alone, so the joint reaction of
every system case is the resultant of the head's loads about the joint centre, by statics (no guessed load).

  head: drawn and cut as the system model above the joint; support = its flange face on the base (bolted, 4x M3, C); loads = the
        L2, camera and L2-frame cases plus the head's own mass at every knock.
  base: drawn and cut as the system model below the joint; support = the thread boss wall (the pole); loads = the pack, Pi, guard,
        strap and hand cases plus, at every system case, the head's joint reaction as a force + moment on the 4 bolt bosses
        (least-norm bolt forces, sum F and sum r x f exact).
Head mass for the knock reactions: 133 g at (-5, 0, 214) (M, unit v2's head piece; DEFAULT E until the head v14 run reports its own).
   python3 scan_stick_pieces_v1.py head|base [h] [gen_cm3] [--no-git]"""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise
_argv = sys.argv; sys.argv = [sys.argv[0]]
import unit_topo_v3 as U                                                       # the system model (geometry, masses, cases)
sys.argv = _argv

PIECE = sys.argv[1] if len(sys.argv) > 1 else "head"
ARGS = [a for a in sys.argv[2:] if not a.startswith("-")]
H = float(ARGS[0]) if ARGS else 2.0                                         # C: h 2.0 (h 1.5 on the head would need ~10 GB with 33 cases)
GEN = float(ARGS[1]) if len(ARGS) > 1 else (22.0 if PIECE == "head" else 45.0)   # E: the unit's 65 split by the system solve's volume
GIT = "--no-git" not in sys.argv
NAME, VER = ("scan_stick_head", "v14") if PIECE == "head" else ("scan_stick_base", "v19")
OUT = os.path.join(ROOT, "runs", NAME, VER)
HEAD_KG, HEAD_CG = 0.133, np.array((-5.3, -0.6, 213.7))                       # M (unit v2 head piece), DEFAULT E
JOINT_C = np.array((0.5 * (U.FLX[0] + U.FLX[1]), 0.0, U.TOP))                  # joint centre (4, 0, 168), C
G = 9.81
DIRS = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
DIAG = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]

def centroid(m):
    return np.array(m.bounding_box()).reshape(2, 3).mean(0)

def head_cases(Hd):
    """every system case that acts on the head: (name, [(region, F)], accel or None) and the matching joint reaction (F, M)"""
    reg = Hd["regions"]
    l2c, camc, railc = centroid(reg["l2"]), centroid(reg["cam"]), centroid(reg["rails"])
    out = []
    def add(name, parts, accel, extra_pts):
        F = sum(f for _, f in parts) + sum(f for _, f in extra_pts)
        M = sum(np.cross(centroid(r) - JOINT_C, f) for r, f in parts) + sum(np.cross(p - JOINT_C, f) for p, f in extra_pts)
        out.append((name, parts, accel, F, M))
    for gs, ds in ((25, DIRS + DIAG), (50, DIRS)):
        for k, d in enumerate(ds):
            add(f"unit {gs} g {k}", [(reg["l2"], U.KG["l2"] * G * gs * d), (reg["cam"], U.KG["cam"] * G * gs * d)], gs * d,
                [(HEAD_CG, HEAD_KG * G * gs * d)])                            # the head's own mass rides in the reaction
    for nm, v in (("L2 hit front 100 N", -100 * U.F), ("L2 hit +y 100 N", -100 * U.Y), ("L2 hit -y 100 N", 100 * U.Y),
                  ("L2 drop -z 150 N", np.array((0, 0, -150.0)))):
        add(nm, [(reg["l2"], v)], None, [])
    for nm, v in (("camera hit front 50 N", np.array((-50.0, 0, 0))), ("camera hit +y 50 N", -50 * U.Y), ("camera hit -y 50 N", 50 * U.Y),
                  ("camera hit above 50 N", np.array((0, 0, -50.0))), ("camera hit below 50 N", np.array((0, 0, 50.0)))):
        add(nm, [(reg["cam"], v)], None, [])
    for nm, v in (("L2 frame bump +y 100 N", np.array((0, -100.0, 0))), ("L2 frame bump -y 100 N", np.array((0, 100.0, 0))),
                  ("L2 frame bump top 100 N", np.array((0, 0, -100.0))), ("L2 frame bump back 100 N", np.array((100.0, 0, 0)))):
        add(nm, [(reg["rails"], v)], None, [])
    return out

def bolt_forces(F, M):
    """least-norm forces on the 4 bolt bosses with sum f = F and sum (r - c) x f = M"""
    r = [np.array((x, y, U.TOP)) - JOINT_C for x, y in U.BOLTS]
    A = np.zeros((6, 12))
    for i, ri in enumerate(r):
        A[:3, 3 * i:3 * i + 3] = np.eye(3)
        A[3:, 3 * i:3 * i + 3] = np.array([[0, -ri[2], ri[1]], [ri[2], 0, -ri[0]], [-ri[1], ri[0], 0]])
    f = np.linalg.lstsq(A, np.r_[F, M], rcond=None)[0]
    return [f[3 * i:3 * i + 3] for i in range(4)]

def setup(h):
    B, Hd = U.base_bodies(), U.head_bodies()
    hc = head_cases(Hd)
    if PIECE == "head":
        drop = U.box(U.BAT_X1 - U.BAT_H - 1, U.BAT_X1 + 0.5, -U.BAT_W / 2 - 1, U.BAT_W / 2 + 1, U.TOP, 420)   # the pack's drop-in path
        p = Part(NAME, h, Hd["drawn"], Hd["cuts"] + drop, Hd["env"], build="+z", material="PETG")
        p.support("default", U.box(U.FLX[0], U.FLX[1], -U.FLY, U.FLY, U.TOP - 1, U.TOP + 1.5))                 # the flange face on the base, clamped by 4x M3
        for name, parts, accel, F, M in hc:
            if accel is None:
                p.load(name, parts[0][0], parts[0][1])
            else:
                p.load_multi(name, parts, accel=accel)
        return p
    p = Part(NAME, h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    p.support("default", U.cyl(0, 0, 2, U.SOCK_H - 6, 19.0, 64) - U.cyl(0, 0, 0, U.SOCK_H, 10.0, 64))
    bosses = [U.cyl(x, y, U.FL_Z, U.TOP + 0.5, 5.0, 32) - U.cyl(x, y, U.FL_Z - 1, U.TOP + 1, 1.7, 24) for x, y in U.BOLTS]
    pack_r = B["ledge"] + B["lip"] + B["rails"] + B["back"] + B["posts"] + B["ring"]
    pi_r = mf.Manifold()
    for y, z in U.pi_pads:
        pi_r += U.cylx(U.PI_X0, U.PI_X0 + U.PI_SO + 0.5, y, z, 4.0)
    for name, parts, accel, F, M in hc:                                         # every head case reaches the base through the joint
        joint = [(b, f) for b, f in zip(bosses, bolt_forces(F, M))]
        if accel is None:
            p.load_multi(name + " (joint)", joint)
        else:
            d = accel / np.linalg.norm(accel); gs = np.linalg.norm(accel)
            p.load_multi(name, joint + [(pack_r, U.KG["pack"] * G * gs * d), (pi_r, U.KG["pi"] * G * gs * d)], accel=accel)
    p.load("Pi guard bump 100 N", B["piguard"], np.array((-100.0, 0, 0)))
    p.load("Pi guard bump side 100 N", B["piguard"], np.array((0, -100.0, 0)))
    p.load("UBEC guard bump 100 N", B["ubguard"], np.array((0, 100.0, 0)))
    p.load("pack guard bump back 100 N", B["posts"] + B["ring"], np.array((100.0, 0, 0)))
    p.load("pack guard bump side 100 N", B["posts"] + B["ring"], np.array((0, 100.0, 0)))
    p.load("strap tension -x 120 N", B["rails"], np.array((-120.0, 0, 0)))
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

def reactions():
    """the joint reaction table for dimensions.md "System scan stick" (N, N mm about the joint centre)"""
    for name, parts, accel, F, M in head_cases(U.head_bodies()):
        print(f"{name}: F {np.round(F, 1).tolist()} N, M {np.round(M / 1000, 2).tolist()} N m")

if __name__ == "__main__":
    if "--reactions" in sys.argv:
        reactions()
    elif os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=OUT, name=NAME, settings=Settings(), git=GIT)
