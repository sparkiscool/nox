"""Scan stick base v18: v17 with M2 screws for the Pi 5 (Spark 24 Oct: out of M2.5, use M2 and M3). The Pi's Ø2.7 holes take an
M2 screw; the 4 standoff holes go from Ø2.3 to Ø1.8 (E, M2 machine screws thread into the PETG, 7 deep C; DEFAULT, alternative
M2 heat-set inserts). Pi screws M2 x 8 (C: 1.6 board + 6.4 in the standoff). Only the exact cut changes; the v7 layout masks stay.
v17: v16 + review round 3 of PR 22: the button (Ø12.3 at y -26) and LED (Ø5.2 at y -36) holes left 1.25 and
1.4 mm (M) of strip; v17 runs the strip to y ±44 (C) and puts the LED at y -39 (C): 4.25 mm between the holes, 2.4 to the end,
the Ø14 button body and the Ø7 LED body 2.5 apart (C). v16: v15 + review round 2 of PR 22. (1) The rocker and button cuts at y -14 / 12 ran into the Ø36 tube that
sits right behind the panel strip for |y| < 18: 0.7 mm (M) of tube wall left and no room for the switch bodies. v16 widens the
panel strip to y ±40 (C) and puts the rocker at y +28, the button at y -26 and the LED at y -36 (C), all outside the tube,
with body keep-outs behind them (rocker 22 x 22 x 21 deep, button Ø14 x 15, LED Ø7 x 8, E). (2) M4 inserts are Ruthex
RX-M4x8.1 (OD 5.6 x 8.1, spec): holes 9.5 deep from z 158.5 (C, depth ≥ L + 1), the bore top lowered to z 154 (C) for a 4.5 mm
floor, the cable hole from z 150; M4 x 12 (4 mm clamp + 8 in the insert). v15: v13 + the exact envelope at the build = the layout envelope + the exact drawn bodies (`Part` widens the
envelope by the drawn bodies at construction; v14 swapped in a bare box and lost the rails and the back plate beyond y ±32,
31 cm3). v14: v13 + the exact envelope widened to y ±32 at the build. v13's Pi tray (y ±32) and the -Y standoffs were
clipped by the v7 envelope (y -23..30): the build intersects (field + drawn) with the envelope, and only drawn bodies that were
in the layout's drawn mask survive outside it. v14 swaps the exact envelope in after the Part is built (like the cuts), the
layout masks stay v7's. v13: rebuild of the v7 layout, review fixes on v12 (PR 22): (1) the 4 Pi standoffs had no drawn tray or
gusset (rule: attachments drawn with a gusset into the body) and the lower -Y one was cut to a sliver by the UBEC keep-out;
v13 draws a 5 mm Pi tray plate (x 15..20, y ±32, z 8..99, one plate with the panel strip above it, joined to the tube) with
Ø8 standoffs on it. (2) The UBEC keep-out (x ±27) also cut the -Y rail below z 68 to 3 mm and the back plate to y -23.5:
v13 keeps it at x ±14 (the UBEC and buck stand on end in the 28 wide bay, stacked in y, zip ties through the ±12 slots).
(3) The M4 insert holes start at z 162 (6 deep, 4 mm floor over the bore; v12 ran them through to the 2 mm floor at z 160).
The v7 masks are untouched; the exact drawn / cut bodies carry the fixes.  v12: rebuild of the v7 layout. v10 / v11 failed the exact-file gate on one 0.02 mm3 voxel: a 0.14 mm
feather of the generated surface hugging the thread boss at r 18, z 9.6 (ray crossings 9.55 / 9.70). v12 draws the boss
at Ø40 (the flange size; thread wall 10 mm) as the exact drawn body, which swallows the feather zone (r 16..20, z 0..34);
the layout masks stay v7's. v11: v10 passed every gate test but a single stray voxel (0.02 mm3,
a tangent ray crossing on the generated surface at (2.6, 17.9, 9.6)) that the plain body count rejects; a slightly different
volume target moves the iso-surface everywhere. v10: v8 / v9 still failed the gate on three 0.5 mm slivers: the zip-tie slots at
x ±18 cut into the back plate behind the shelf. v10 moves the slots to x ±12 (only the shelf plate behind them) and keeps the
v8 cut offsets and the v9 cable openings. v9: v8 passed the body count but left the tube bore a sealed 75 cm3 cavity; v9
opens it: a Ø10 hole through the flange centre (lidar + camera cables up into the head) and a 10 x 30 window in the tube
wall on the shelf side at z 40..70 (power wires to the UBEC / buck and the Pi). v8: v7 passed every gate test but the plain body count (5): cut faces flush with
drawn faces (tube bore top on the flange bottom, pack / Pi / UBEC keep-outs on the plate faces) left stray voxels in the
gate's grid. v8 moves those cut faces 0.5-2 mm off the drawn faces and rebuilds from v7's density (no re-optimisation).
v6 (drawn tube, 30 cm3) solved the head loads (flange joint off the list) but read
worst 2.2 at the ledge root under the 50 g sideways pack impacts, the rails being fins held only at the bottom, and failed
the gate on 3.25 mm slot cheeks (neck test). v7: a drawn 5 mm back plate behind the pack spanning both rails and the tube
over the full height (sideways pack load in plate shear), rails 14 wide (5 mm cheeks at the slots), 25 cm3 generated.
v5 (60 cm3) read worst 2.3 at the flange-to-column joint under the head impact and failed
the gate on the strap bars: the generated column is not reliable at 16 Nm of bending, so v6 draws the column as a Ø36 x 4
tube (bending stress ~6 MPa at 16 Nm, E; wires run inside) and the optimiser only braces the plates to it; strap slots go
through 10 mm rails, no separate bars; the lip sits outside the pack keep-out. v4 at 35 cm3 failed the gate (strap bars inside the pack keep-out cut to 1.3 mm, 3.5 mm
plates gone at the 2.0 mm drawn erosion) and read worst 5.5 at the ledge root under the 50 g pack impact: v5 has 5-6 mm
drawn plates, a drawn web from the ledge up the boss, bars outside the keep-out, 60 cm3 generated. v3 crashed at the coarse stage: at h 4 the 3 mm support band round the thread grabbed a
neighbouring drawn body and a tie load sat on fixed nodes; v4 has a wider band and no coarse stage. v2 was stopped minutes in: the organic finish opens the drawn plates and failed the
head and cam gates; v3 keeps the drawn bodies whole) (`scan_stick.md`). Layout (Spark 23 Oct): one bolted unit that screws onto the pole
tip. The base carries the 3/4-5 ACME socket at the bottom (near the hand), the Pi 5 tray on +X, the 4S 6000 pack on -X,
the UBEC / buck shelf on -Y, and a top flange at z 160 that the head bolts onto (4x M4 heat-set inserts). The column
between them is generated.   python3 base_topo_v17.py [h] [gen_cm3] (rebuilds from runs/scan_stick_base/v7) [h] [gen_cm3] [--no-git]
Print frame: bottom (thread boss + ledge feet) on the bed, build +z, 168 tall (A1 mini 180). Numbers C unless tagged;
pack 148 x 48 x 45 (P, section O). Drawn: thread boss Ø40 x 34, Ø36 tube, top flange Ø40 x 8, Pi tray + 4 standoffs, panel strip
(rocker / button / LED), pack ledge + lip + side rails + back plate, shelf plate. Cuts: the exact thread, flange insert
holes, Pi screw holes, strap slots, panel cutouts, zip slots, pack / Pi / UBEC keep-outs, the pole below the boss.
Loads (DEFAULT, E): pack 0.59 kg (Q9) at 25 g (14 dirs) and 50 g (6 dirs) on ledge / lip / rails / bars, Pi 0.08 kg at
25 / 50 g, head + lidar 0.30 kg on the flange at 25 / 50 g, 100 N side hit and 150 N drop on the flange, 120 N strap
tension, 50 N hand grabs, 100 N push on the ledge. Support: the thread boss wall (the pole)."""
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
from topo import Part, Settings, optimise
from topo.run import rebuild
from topo.part import man

H = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 2.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else 25.0  # cm3 generated (DEFAULT, E; the v7 layout's value)
GIT = "--no-git" not in sys.argv

BOSS_R, SOCK_H = 16.0, 34.0
FL_Z, FL_T, FL_R, FL_BOLT = 160.0, 8.0, 20.0, 8.5
TOP = FL_Z + FL_T                                  # 168
PI_L, PI_W, PI_HX, PI_HY, PI_OFF, PI_SO = 85.0, 56.0, 58.0, 49.0, 3.5, 7.0
PI_X0, PI_Z0 = 20.0, 10.0                          # tray face at x 20, Pi bottom edge at z 10
PLATE_T = 5.0
BAT_L, BAT_W, BAT_H = 148.0, 48.0, 45.0            # P
BAT_X1, BAT_Z0 = -20.0, 10.0                       # pack face at x -20, pack bottom at z 10
STRAP_W, STRAP_T = 20.0, 3.5
SHELF_W, SHELF_Y = 52.0, (-23.0, -18.0)
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

def bodies(off=0.5, boss_r=BOSS_R, v13=False):
    """off = how far the keep-out / bore cut faces sit off the drawn faces: 0 = v7 (the layout's masks), 0.5 = the exact v8 cuts"""
    o2 = 4 * off                                                                       # bore ends: 2 mm at off 0.5
    boss = cyl(0, 0, 0, SOCK_H, boss_r, 96)
    tube = cyl(0, 0, SOCK_H - 4, FL_Z + 2, 18.0, 96)                      # drawn column Ø36, bore Ø28 cut below
    flange = cyl(0, 0, FL_Z, TOP, FL_R, 96)
    pads = mf.Manifold()
    for y, z in pi_pads:
        pads += cylx(PI_X0 - 3.5, PI_X0 + PI_SO, y, z, 4.0 if v13 else 3.5)                  # v13: Ø8 standoffs on the tray
    panel = box(PI_X0 - PLATE_T, PI_X0, -32, 32, PANEL_Z - 12, PANEL_Z + 12)
    if v13:
        panel += box(PI_X0 - PLATE_T, PI_X0, -32, 32, PI_Z0 - 2, PANEL_Z - 12)                 # v13: the Pi tray plate, 5 thick, up into the panel strip
        panel += box(PI_X0 - PLATE_T, PI_X0, -44, 44, PANEL_Z - 12, PANEL_Z + 12)              # v17: the strip reaches y ±44, the switches sit outside the tube
    ledge = box(BAT_X1 - BAT_H - 4, BAT_X1 + 6, -BAT_W / 2 - 2, BAT_W / 2 + 2, 5, 11)                    # 6 thick
    lip = box(BAT_X1 - BAT_H - 5, BAT_X1 - BAT_H - 1, -BAT_W / 2 - 2, BAT_W / 2 + 2, 5, 20)                # 4 thick, outside the keep-out
    web = box(BAT_X1, BAT_X1 + 8, -16, 16, 11, 70)                                                        # drawn web: ledge -> tube (rule: gusset into the body)
    rails = mf.Manifold()
    for sy in (-1, 1):
        y0 = sy * (BAT_W / 2 + 1); rails += box(BAT_X1 - 10, BAT_X1, min(y0, y0 + sy * 14), max(y0, y0 + sy * 14), 5, BAT_Z0 + BAT_L + 2)   # 14 wide, strap slots through them (5 mm cheeks)
    shelf = box(-SHELF_W / 2, SHELF_W / 2, SHELF_Y[0], SHELF_Y[1], 5, 66)
    back = box(BAT_X1, BAT_X1 + 5, -(BAT_W / 2 + 15), BAT_W / 2 + 15, 5, FL_Z)                              # 5 mm back plate: rails <-> tube, full height
    drawn = boss + tube + flange + pads + panel + ledge + lip + web + rails + back + shelf

    tc = os.path.join(HERE, "out", "thread_cut.stl")
    if not os.path.exists(tc):
        os.makedirs(os.path.dirname(tc), exist_ok=True)
        os.system(f"openscad -o {tc} {os.path.join(HERE, 'thread_cut.scad')} >/dev/null 2>&1")
    cuts = man(trimesh.load(tc, force="mesh"))                                               # thread from z 0 up 27
    cuts += cyl(0, 0, SOCK_H + o2, FL_Z - (6 if v13 else o2), 14.0, 96)                      # tube bore Ø28 (wires), off the boss top; top at z 154 (v16: insert floor)
    if off > 0:                                                                            # openings only in the exact cuts (the layout masks stay v7's)
        cuts += cyl(0, 0, FL_Z - 10, TOP + 1, 5.0, 48)                                       # Ø10 cable hole through the flange centre, down into the bore
        cuts += box(-5, 5, -19.5, -12.5, 40, 70)                                             # cable window through the tube wall, shelf side
    cuts += box(-100, 100, -100, 100, -10, 0)                                                 # the pole below
    for x, y in fl_bolts:
        cuts += cyl(x, y, TOP - 9.5 if v13 else FL_Z, TOP + 1, 2.8)                           # M4 heat-set insert holes from the top (v16: Ruthex M4 x 8.1, 9.5 deep, 4.5 mm floor)
    for y, z in pi_pads:
        cuts += cylx(PI_X0 + PI_SO - 6, PI_X0 + PI_SO + 1, y, z, 0.9 if v13 else 1.15)          # v18: Ø1.8 for M2 (v17 Ø2.3 for M2.5); the layout's masks keep Ø2.3
    cuts += box(PI_X0 + PI_SO + off, PI_X0 + PI_SO + 30, -PI_W / 2 - 2, PI_W / 2 + 2, PI_Z0 - 2, PI_Z0 + PI_L + 2)   # Pi + parts, 0.5 off the pad ends
    if v13:                                                                                # v16: switches outside the tube, bodies behind the strip cut free
        cuts += box(PI_X0 - PLATE_T - 1, PI_X0 + 1, 28 - 9.7, 28 + 9.7, PANEL_Z - 6.6, PANEL_Z + 6.6)            # rocker opening 19.4 x 13.2 at y +28
        cuts += box(PI_X0 - PLATE_T - 21, PI_X0 - PLATE_T, 28 - 11, 28 + 11, PANEL_Z - 11, PANEL_Z + 11)         # rocker body 22 x 22 x 21 deep (E)
        cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, -26, PANEL_Z, 6.15) + cylx(PI_X0 - PLATE_T - 15, PI_X0 - PLATE_T, -26, PANEL_Z, 7.0)   # button Ø12.3, body Ø14 x 15 (E)
        cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, -39, PANEL_Z, 2.6) + cylx(PI_X0 - PLATE_T - 8, PI_X0 - PLATE_T, -39, PANEL_Z, 3.5)     # LED Ø5.2 at y -39, body Ø7 x 8 (E)
    else:
        cuts += box(PI_X0 - PLATE_T - 1, PI_X0 + 1, -14 - 9.7, -14 + 9.7, PANEL_Z - 6.6, PANEL_Z + 6.6)       # rocker (v7 masks)
        cuts += cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, 12, PANEL_Z, 6.15) + cylx(PI_X0 - PLATE_T - 1, PI_X0 + 1, 24, PANEL_Z, 2.6)
    cuts += box(BAT_X1 - BAT_H - 1, BAT_X1 - off, -BAT_W / 2 - 1, BAT_W / 2 + 1, BAT_Z0, BAT_Z0 + BAT_L + 1)   # pack, 0.5 off the back plate
    for z in bar_z:
        for sy in (-1, 1):
            yc = sy * (BAT_W / 2 + 8)                                                         # slot through rail + back plate, 5 mm cheeks
            cuts += box(BAT_X1 - 11, BAT_X1 + 1, yc - STRAP_T / 2, yc + STRAP_T / 2, z - STRAP_W / 2, z + STRAP_W / 2)
    zx = 18.0 if off == 0 else 12.0                                                        # v7 masks at ±18; exact cuts at ±12 (clear of the back plate and the Pi pads)
    for sx in (-1, 1):
        for z in (12.0, 32.0, 52.0):
            cuts += box(sx * zx - 1.5, sx * zx + 1.5, SHELF_Y[0] - 1, SHELF_Y[1] + 1, z, z + 8)
    ux = 14.0 if v13 else SHELF_W / 2 + 1                                                   # v13: x ±14, clear of the tray, the standoffs, the back plate and the rails
    cuts += box(-ux, ux, SHELF_Y[0] - 30, SHELF_Y[0] - off, 4, 68)                             # UBEC / buck, 0.5 off the shelf
    env = box(BAT_X1 - BAT_H - 5, PI_X0 + PI_SO, SHELF_Y[0], 30.0, 0, TOP)
    return dict(drawn=drawn, cuts=cuts, env=env, ledge=ledge, lip=lip, rails=rails, panel=panel, flange=flange, web=web, tube=tube, back=back)

def setup(h):
    B = bodies(0.0)                                                                    # v7 cuts: the layout's grid and masks
    p = Part("scan_stick_base", h, B["drawn"], B["cuts"], B["env"], build="+z", material="PETG")
    from topo.part import tm
    B8 = bodies(0.5, boss_r=20.0, v13=True)
    p.cuts_m = B8["cuts"]; p.cuts_t = tm(B8["cuts"])                                   # the exact cuts applied at the build: faces off the drawn faces
    p.drawn_m = B8["drawn"]; p.drawn_t = tm(B8["drawn"])
    p.env_m = B8["env"] + B8["drawn"]; p.env_t = tm(p.env_m)                          # v15: envelope + the exact drawn bodies (the tray), as Part does; masks stay v7's                               # exact drawn at the build: Ø40 thread boss (v12), Pi tray + Ø8 standoffs (v13)
    p.support("default", cyl(0, 0, 2, SOCK_H - 6, 15.0, 64) - cyl(0, 0, 0, SOCK_H, 10.0, 64))   # the boss wall round the thread
    pack_region = B["ledge"] + B["lip"] + B["rails"] + B["back"]
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
    p.load("strap tension -x 120 N", B["rails"], np.array((-120.0, 0, 0)))                   # straps pull the rails toward the pack
    p.load("hand grab panel +x 50 N", B["panel"], np.array((50.0, 0, 0)))
    p.load("hand grab panel -y 50 N", B["panel"], np.array((0, -50.0, 0)))
    p.load("hand push ledge -z 100 N", B["ledge"], np.array((0, 0, -100.0)))
    return p

if __name__ == "__main__":
    out = os.path.join(ROOT, "runs", "scan_stick_base", "v18")
    if os.environ.get("NOX_SETUP_ONLY"):
        print(setup(H).summary())
    else:
        rebuild(setup, h=H, gen_cm3=GEN, src_dir=os.path.join(ROOT, "runs", "scan_stick_base", "v7"), out_dir=out, name="scan_stick_base", settings=Settings(finish="drawn"))
