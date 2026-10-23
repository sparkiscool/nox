"""Scan stick base v4: nox/topo run (v3 crashed at the coarse stage: at h 4 the 3 mm support band round the thread grabbed a
neighbouring drawn body and a tie load sat on fixed nodes; v4 has a wider band and no coarse stage. v2 was stopped minutes in: the organic finish opens the drawn plates and failed the
head and cam gates; v3 keeps the drawn bodies whole) (`scan_stick.md`). Layout (Spark 23 Oct): one bolted unit that screws onto the pole
tip. The base carries the 3/4-5 ACME socket at the bottom (near the hand), the Pi 5 tray on +X, the 4S 6000 pack on -X,
the UBEC / buck shelf on -Y, and a top flange at z 160 that the head bolts onto (4x M4 heat-set inserts). The column
between them is generated.   python3 base_topo_v4.py [h] [gen_cm3] [--no-git]
Print frame: bottom (thread boss + ledge feet) on the bed, build +z, 168 tall (A1 mini 180). Numbers C unless tagged;
pack 148 x 48 x 45 (P, section O). Drawn: thread boss Ø32 x 34, top flange Ø40 x 8, 4 Pi standoff pads, panel strip
(rocker / button / LED), pack ledge + lip + side rails + 4 strap bars, shelf plate. Cuts: the exact thread, flange insert
holes, Pi screw holes, strap slots, panel cutouts, zip slots, pack / Pi / UBEC keep-outs, the pole below the boss.
Loads (DEFAULT, E): pack 0.59 kg (Q9) at 25 g (14 dirs) and 50 g (6 dirs) on ledge / lip / rails / bars, Pi 0.08 kg at
25 / 50 g, head + lidar 0.30 kg on the flange at 25 / 50 g, 100 N side hit and 150 N drop on the flange, 120 N strap
tension, 50 N hand grabs, 100 N push on the ledge. Support: the thread boss wall (the pole)."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.part import man

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 35.0  # cm3 generated (DEFAULT, E)
GIT = "--no-git" not in sys.argv

BOSS_R, SOCK_H = 16.0, 34.0
FL_Z, FL_T, FL_R, FL_BOLT = 160.0, 8.0, 20.0, 8.5
TOP = FL_Z + FL_T                                  # 168
PI_L, PI_W, PI_HX, PI_HY, PI_OFF, PI_SO = 85.0, 56.0, 58.0, 49.0, 3.5, 7.0
PI_X0, PI_Z0 = 20.0, 10.0                          # tray face at x 20, Pi bottom edge at z 10
PLATE_T = 3.5
BAT_L, BAT_W, BAT_H = 148.0, 48.0, 45.0            # P
BAT_X1, BAT_Z0 = -20.0, 10.0                       # pack face at x -20, pack bottom at z 10
STRAP_W, STRAP_T = 20.0, 3.5
SHELF_W, SHELF_Y = 52.0, (-21.5, -18.0)
PANEL_Z = PI_Z0 + PI_L + 4 + 12                    # 111
PACK_KG, PI_KG, HEAD_KG, G = 0.59, 0.08, 0.30, 9.81

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])
def cylx(x0, x1, y, z, r, n=48):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])

pi_pads = [(-PI_W / 2 + PI_OFF + iy * PI_HY, PI_Z0 + PI_OFF + ix * PI_HX) for ix in (0, 1) for iy in (0, 1)]   # (y, z)
bar_z = [BAT_Z0 + f * BAT_L for f in (0.25, 0.75)]
fl_bolts = [(sx * FL_BOLT, sy * FL_BOLT) for sx in (-1, 1) for sy in (-1, 1)]

def bodies():
    boss = cyl(0, 0, 0, SOCK_H, BOSS_R, 96)
    flange = cyl(0, 0, FL_Z, TOP, FL_R, 96)
    pads = mf.Manifold()
    for y, z in pi_pads:
        pads += cylx(PI_X0 - 3.5, PI_X0 + PI_SO, y, z, 3.5)
    panel = box(PI_X0 - PLATE_T, PI_X0, -32, 32, PANEL_Z - 12, PANEL_Z + 12)
    ledge = box(BAT_X1 - BAT_H - 3, BAT_X1 + 4, -BAT_W / 2 - 2, BAT_W / 2 + 2, 6, 10)
    lip = box(BAT_X1 - BAT_H - 3, BAT_X1 - BAT_H, -BAT_W / 2 - 2, BAT_W / 2 + 2, 6, 18)
    rails = mf.Manifold()
    for sy in (-1, 1):
        y0 = sy * (BAT_W / 2 + 2); rails += box(BAT_X1 - 10, BAT_X1, min(y0, y0 + sy * 3), max(y0, y0 + sy * 3), 6, BAT_Z0 + BAT_L + 2)
    bars = mf.Manifold()
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 4); bars += box(BAT_X1 - PLATE_T, BAT_X1, yc - 5, yc + 5, z - STRAP_W / 2 - 4, z + STRAP_W / 2 + 4)
    shelf = box(-SHELF_W / 2, SHELF_W / 2, SHELF_Y[0], SHELF_Y[1], 6, 66)
    drawn = boss + flange + pads + panel + ledge + lip + rails + bars + shelf

    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    cuts = man(trimesh.load(tc, force="mesh"))                                               # thread from z 0 up 27
    cuts += box(-100, 100, -100, 100, -10, 0)                                                 # the pole below
    for x, y in fl_bolts:
        cuts += cyl(x, y, FL_Z, TOP + 1, 2.8)                                                 # M4 heat-set insert holes from the top
    for y, z in pi_pads:
        cuts += cylx(PI_X0 + PI_SO - 6, PI_X0 + PI_SO + 1, y, z, 1.15)
    cuts += box(PI_X0 + PI_SO, PI_X0 + PI_SO + 30, -PI_W / 2 - 2, PI_W / 2 + 2, PI_Z0 - 2, PI_Z0 + PI_L + 2)   # Pi + parts
    cuts += box(PI_X0 - PLATE_T - 1, PI_X0 + 1, -14 - 9.7, -14 + 9.7, PANEL_Z - 6.6, PANEL_Z + 6.6)           # rocker
    cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, 12, PANEL_Z, 6.15) + cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, 24, PANEL_Z, 2.6)
    cuts += box(BAT_X1 - BAT_H - 1, BAT_X1, -BAT_W / 2 - 1, BAT_W / 2 + 1, BAT_Z0, BAT_Z0 + BAT_L + 1)       # pack
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 4)
            cuts += box(BAT_X1 - PLATE_T - 1, BAT_X1 + 1, yc - STRAP_T / 2, yc + STRAP_T / 2, z - STRAP_W / 2, z + STRAP_W / 2)
    for sx in (-1, 1):
        for z in (12.0, 32.0, 52.0):
            cuts += box(sx * 18 - 1.5, sx * 18 + 1.5, SHELF_Y[0] - 1, SHELF_Y[1] + 1, z, z + 8)
    cuts += box(-SHELF_W / 2 - 1, SHELF_W / 2 + 1, SHELF_Y[0] - 30, SHELF_Y[0], 4, 68)                       # UBEC / buck
    env = box(BAT_X1 - BAT_H - 3, PI_X0 + PI_SO, SHELF_Y[0], 30.0, 0, TOP)
    return dict(drawn=drawn, cuts=cuts, env=env, ledge=ledge, lip=lip, rails=rails, bars=bars, panel=panel, flange=flange)

def setup(h):
    B = bodies()
    p = Part("scan_stick_base", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    p.support("default", cyl(0, 0, 2, SOCK_H - 6, 15.0, 64) - cyl(0, 0, 0, SOCK_H, 10.0, 64))   # the boss wall round the thread
    pack_region = B["ledge"] + B["lip"] + B["rails"] + B["bars"]
    pad_region = mf.Manifold()
    for y, z in pi_pads:
        pad_region += cylx(PI_X0, PI_X0 + PI_SO + 0.5, y, z, 4.0)
    fl_region = cyl(0, 0, FL_Z, TOP + 0.5, FL_R + 0.5) - cyl(0, 0, FL_Z - 1, TOP + 1, 6.0)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs + diag):
        p.load(f"pack 25 g {k}", pack_region, PACK_KG * G * 25 * d, accel=25 * d)
        p.load(f"pi 25 g {k}", pad_region, PI_KG * G * 25 * d)
        p.load(f"head 25 g {k}", fl_region, HEAD_KG * G * 25 * d)
    for k, d in enumerate(dirs):
        p.load(f"pack impact 50 g {k}", pack_region, PACK_KG * G * 50 * d, accel=50 * d)
        p.load(f"pi impact 50 g {k}", pad_region, PI_KG * G * 50 * d)
        p.load(f"head impact 50 g {k}", fl_region, HEAD_KG * G * 50 * d)
    for nm, v in (("flange side hit +x 100 N", (100, 0, 0)), ("flange side hit -y 100 N", (0, -100, 0)), ("drop on head -z 150 N", (0, 0, -150))):
        p.load(nm, fl_region, np.array(v, float))
    p.load("strap tension -x 120 N", B["bars"], np.array((-120.0, 0, 0)))
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick_base", "v4")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name="scan_stick_base", settings=Settings(finish="drawn"), coarse=False, git=GIT)
