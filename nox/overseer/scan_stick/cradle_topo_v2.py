"""Scan stick cradle v2: nox/topo run (`scan_stick.md`).   python3 cradle_topo_v2.py [h] [gen_cm3] [--no-git]
Print frame = scan_stick.scad frame: pole axis Z, hub z 0..60, bed at z = 0 (bottom face), Pi tray on +X, pack on -X,
clamp ears on +Y, UBEC / buck shelf on -Y. Numbers C from scan_stick.scad / section S, pack 148 x 48 x 45 (P, section O).
Drawn (mounting + must-haves): clamp hub with ears, 4 Pi standoff pads (Ø7, M2.5), the panel strip (rocker / button / LED),
pack ledge + lip + side rails, 4 strap-slot bars, the shelf plate (zip slots). Everything between them is generated.
Cuts: bore, slit, clamp bolts + nut / head pockets, Pi screw holes, strap slots, panel cutouts, zip slots, the pack and Pi
keep-outs. Support: the hub bore (the pole).
Loads (DEFAULT, E): pack 0.59 kg (Q9) at 25 g knocks on ledge / rails / strap bars (14 dirs) and 50 g impact (6 dirs),
Pi 0.08 kg at 25 g + 50 g, 50 N hand grab on the panel strip and on the tray edge, 30 N strap tension on each bar."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, manifold3d as mf
from topo import Part, Settings, optimise

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 25.0  # cm3 generated (DEFAULT, E)
GIT = "--no-git" not in sys.argv

ROD_D, CLR, WALL = 25.4, 0.5, 5.0               # C (rod_d E until measured)
BORE = ROD_D + CLR; HUB_R = BORE / 2 + WALL; HUB_H = 60.0
EAR_T, GAP = 7.0, 2.5
PI_L, PI_W, PI_HX, PI_HY, PI_OFF, PI_SO = 85.0, 56.0, 58.0, 49.0, 3.5, 7.0
PLATE_T = 3.5
BAT_L, BAT_W, BAT_H = 148.0, 48.0, 45.0           # P
STRAP_W, STRAP_T = 20.0, 3.5
SHELF_W = 52.0
PANEL_Z = 4 + PI_L + 4 + 12                       # 105: rocker / button / LED centre height
PACK_KG, PI_KG, G = 0.59, 0.08, 9.81              # Q9 E, E

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])
def cylx(x0, x1, y, z, r, n=48):                 # cylinder along X
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])

pi_pads = [(-PI_W / 2 + PI_OFF + iy * PI_HY, 4 + PI_OFF + ix * PI_HX) for ix in (0, 1) for iy in (0, 1)]   # (y, z)
bar_z = [4 + f * BAT_L for f in (0.25, 0.75)]

def bodies():
    hub = cyl(0, 0, 0, HUB_H, HUB_R, 96) + box(-(EAR_T + GAP / 2), EAR_T + GAP / 2, HUB_R - 3, HUB_R + 9, 0, HUB_H)
    pads = mf.Manifold()
    for y, z in pi_pads:
        pads += cylx(HUB_R - 3.5, HUB_R + PI_SO, y, z, 3.5)
    panel = box(HUB_R - PLATE_T, HUB_R, -32, 32, PANEL_Z - 12, PANEL_Z + 12)
    ledge = box(-HUB_R - BAT_H - 3, -HUB_R + 3.5, -BAT_W / 2 - 2, BAT_W / 2 + 2, 0, 4)
    lip = box(-HUB_R - BAT_H - 3, -HUB_R - BAT_H, -BAT_W / 2 - 2, BAT_W / 2 + 2, 0, 12)
    rails = mf.Manifold()
    for sy in (-1, 1):
        rails += box(-HUB_R - 10, -HUB_R, sy * (BAT_W / 2 + 2) + (0 if sy > 0 else -3), sy * (BAT_W / 2 + 2) + (3 if sy > 0 else 0), 0, BAT_L + 8)
    bars = mf.Manifold()
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 4)
            bars += box(-HUB_R - PLATE_T, -HUB_R, yc - 5, yc + 5, z - STRAP_W / 2 - 4, z + STRAP_W / 2 + 4)
    shelf = box(-SHELF_W / 2, SHELF_W / 2, -HUB_R - PLATE_T + 1, -HUB_R + 1, 0, HUB_H)
    drawn = hub + pads + panel + ledge + lip + rails + bars + shelf

    cuts = cyl(0, 0, -1, HUB_H + 1, BORE / 2, 96)                                              # bore
    cuts += box(-GAP / 2, GAP / 2, 0, HUB_R + 20, -1, HUB_H + 1)                              # slit
    for z in (12.0, HUB_H - 12.0):                                                            # clamp bolts
        cuts += cylx(-30, 30, HUB_R + 3.5, z, 2.15)
        cuts += cylx(-(GAP / 2 + EAR_T) - 1, -(GAP / 2 + EAR_T) + 3.4, HUB_R + 3.5, z, 4.2)   # nut pocket (-X ear)
        cuts += cylx(GAP / 2 + EAR_T - 3, GAP / 2 + EAR_T + 1, HUB_R + 3.5, z, 3.8)           # head pocket (+X ear)
    for y, z in pi_pads:
        cuts += cylx(HUB_R + PI_SO - 6, HUB_R + PI_SO + 1, y, z, 1.15)                         # M2.5 self-tap
    cuts += box(HUB_R + PI_SO, HUB_R + PI_SO + 28, -PI_W / 2 - 2, PI_W / 2 + 2, 2, 4 + PI_L + 2)   # Pi + components keep-out
    cuts += box(HUB_R - PLATE_T - 1, HUB_R + 1, -14 - 9.7, -14 + 9.7, PANEL_Z - 6.6, PANEL_Z + 6.6)   # rocker
    cuts += cylx(HUB_R - PLATE_T - 1, HUB_R + 1, 12, PANEL_Z, 6.15)                           # button
    cuts += cylx(HUB_R - PLATE_T - 1, HUB_R + 1, 24, PANEL_Z, 2.6)                            # LED
    cuts += box(-HUB_R - BAT_H - 1, -HUB_R + 0.0, -BAT_W / 2 - 1, BAT_W / 2 + 1, 4, 4 + BAT_L + 1)   # pack keep-out
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 4)
            cuts += box(-HUB_R - PLATE_T - 1, -HUB_R + 1, yc - STRAP_T / 2, yc + STRAP_T / 2, z - STRAP_W / 2, z + STRAP_W / 2)
    for sx in (-1, 1):
        for z in (8.0, 28.0, 48.0):
            cuts += box(sx * 18 - 1.5, sx * 18 + 1.5, -HUB_R - PLATE_T - 1, -HUB_R + 2, z, z + 8)   # zip slots
    cuts += box(-SHELF_W / 2 - 1, SHELF_W / 2 + 1, -HUB_R - PLATE_T - 30, -HUB_R - PLATE_T + 1, 0, HUB_H + 1)  # UBEC / buck sit here
    env = box(-HUB_R - BAT_H - 3, HUB_R + PI_SO, -HUB_R - PLATE_T, HUB_R + 9, 0, BAT_L + 8)
    return dict(drawn=drawn, cuts=cuts, env=env, ledge=ledge, lip=lip, rails=rails, bars=bars, pads=pads, panel=panel)

def setup(h):
    B = bodies()
    p = Part("scan_stick_cradle", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    p.support("default", cyl(0, 0, 1, HUB_H - 1, BORE / 2 + 3.0, 64) - cyl(0, 0, 0, HUB_H, BORE / 2 - 1, 64))
    pack_region = B["ledge"] + B["lip"] + B["rails"] + B["bars"]
    pad_region = mf.Manifold()
    for y, z in pi_pads:
        pad_region += cylx(HUB_R, HUB_R + PI_SO + 0.5, y, z, 4.0)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    diag = [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs + diag):
        p.load(f"pack 25 g {k}", pack_region, PACK_KG * G * 25 * d, accel=25 * d)
        p.load(f"pi 25 g {k}", pad_region, PI_KG * G * 25 * d)
    for k, d in enumerate(dirs):
        p.load(f"pack impact 50 g {k}", pack_region, PACK_KG * G * 50 * d, accel=50 * d)
        p.load(f"pi impact 50 g {k}", pad_region, PI_KG * G * 50 * d)
    p.load("strap tension -x 120 N", B["bars"], np.array((-120.0, 0, 0)))                    # 4 bars, 30 N each
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick_cradle", "v2")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        optimise(setup, h=H, gen_cm3=GEN, out_dir=out, name="scan_stick_cradle", settings=Settings(), git=GIT)
