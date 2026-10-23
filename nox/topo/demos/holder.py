"""The old engine's benchmark holder (noxopt_bench.holder + noxopt_demo.py bodies), same features and load set:
60 x 40 x 30 mm, 4 bolt bosses (bottom corners), pack pad (top), 2 hook roots (sides), camera pad (front), battery
channel keep-out; supports: bolts / grab (hooks) / pad; 28 cases (14 misuse directions at 25 g with the pack, flight,
landing, hand and retention loads, catcher grab, pad push). Units mm, N.

Changes for the solid gate (23 Oct, organic test part):
  * bolt bosses: 6 x 6 mm squares round the 3.2 mm holes left 1.4 mm walls, which the gate's 0.9 mm erosion cut off
    (topo-v1: 4 boss cores of 3.4 mm3 split off, FAIL; the self-test smoke run never joined one corner boss). Now
    round bosses of 10 mm diameter, 6 mm tall: 3.4 mm walls, the largest round boss the 60 x 40 envelope holds at the
    corner bolts (5 mm from both edges).
  * keep-out under the pack round the pad: starts at z 27 (was 26), 3 mm clearance under the pack instead of 4 (DEFAULT).
    The pad sat on a 4 mm layer between that keep-out and the battery channel (top z 22), so every member out of the
    pad was a plate at most 4 mm thick, which the gate's 2.0 mm erosion removes (organic-v1: the engine could route
    only 4.44 mm members to the pad, gate FAIL; worst index 0.65 at the channel's top edge, in that layer). A 5 mm
    layer hosts the 5.0 mm members the gate needs.
  Load set, supports (the 6 x 6 seats), battery channel and envelope unchanged."""
import numpy as np
import manifold3d as mf
from ..part import Part

BOLTS = [(x, y) for x in (5, 55) for y in (5, 35)]
M_PACK = 0.165                                   # kg on the pad
BOSS_R = 5.0                                     # bolt boss radius: 3.4 mm wall round the 1.6 mm hole
PACK_CLEAR_Z = 27.0                              # keep-out under the pack round the pad starts here (was 26)


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def bodies():
    pad = box(22, 38, 14, 26, 26, 30)
    hookL = box(26, 34, 0, 3, 14, 24)
    hookR = box(26, 34, 37, 40, 14, 24)
    cam = box(56, 60, 15, 25, 8, 18)
    bosses = [cyl(x, y, 0, 6, BOSS_R) for x, y in BOLTS]
    drawn = pad + hookL + hookR + cam
    for b in bosses:
        drawn += b
    cuts = box(10, 50, 8, 32, 6, 22) + (box(8, 52, 6, 34, PACK_CLEAR_Z, 30.5) - box(22, 38, 14, 26, 26, 30.5))
    for x, y in BOLTS:
        cuts += cyl(x, y, -1, 31, 1.6) + cyl(x, y, 6, 31, 3.0)
    env = box(0, 60, 0, 40, 0, 30)
    return dict(pad=pad, hookL=hookL, hookR=hookR, cam=cam, bosses=bosses, drawn=drawn, cuts=cuts, env=env)


def setup(h):
    B = bodies()
    p = Part("holder", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    seats = None
    for x, y in BOLTS:
        s = box(x - 3, x + 3, y - 3, y + 3, -1.0, 1.5 * h)
        seats = s if seats is None else seats + s
    p.support("bolts", seats)
    p.support("grab", B["hookL"] + B["hookR"])
    p.support("pad", B["pad"])
    g = 9.81
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs):
        p.load(f"misuse {k}", B["pad"], M_PACK * g * 25 * d, cls="misuse", support="bolts", accel=25 * d)
    p.load("flight", B["pad"], M_PACK * g * np.array([0, 2.0, -4.0]), cls="flight", support="bolts",
           accel=np.array([0, 2.0, -4.0]))
    p.load("landing 6g", B["pad"], M_PACK * g * np.array([0, 0, -6.0]), cls="misuse", support="bolts",
           accel=np.array([0, 0, -6.0]))
    for nm, m, v in [("pinch L", "hookL", (0, 20, 0)), ("pinch R", "hookR", (0, -20, 0)),
                     ("retain L", "hookL", (0, 0, 20)), ("retain R", "hookR", (0, 0, 20)),
                     ("poke cam x", "cam", (-5, 0, 0)), ("poke cam z", "cam", (0, 0, -5)),
                     ("poke cam y", "cam", (0, 5, 0)), ("preload", "pad", (0, 0, 12))]:
        p.load(nm, B[m], np.array(v, float), cls="misuse", support="bolts")
    for nm, v in [("grab lift", (0, 0, -M_PACK * g * 6)), ("grab side", (0, M_PACK * g * 3, 0))]:
        v = np.array(v, float)
        p.load(nm, B["pad"], v, cls="misuse", support="grab", accel=v / (M_PACK * g))
    for nm, v in [("pad push", (0, 0, -15)), ("pad shear", (10, 0, 0))]:
        p.load(nm, B["bosses"][0] + B["bosses"][3], np.array(v, float), cls="misuse", support="pad")
    return p
