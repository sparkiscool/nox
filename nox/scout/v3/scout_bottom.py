"""Scout v3 bottom layout v1 (24 Oct 2026): receiver v4 + cartridge v4 + pogo pads / KW11 target + underside stack.

Spark 23 Oct: XT60 back, the contact pads are retired. The swap push now mates the pack's own XT60 on the install axis
and the pull unmates it (dimensions I "XT60 mate": insert 60 N, pull-out 50 N, E). This script draws, places and checks
the whole Scout underside in one sweep:
  * cartridge v4 (Scout): the v3 box (38 x 48, rails, flat bottom, sunken tabs, squeeze-release sliders) with the pack's
    XT60 fixed at the lead end facing the install axis (+y_c), lead slack and balance lead stowed under the plug shelf;
  * receiver v4 (drone side, PETG, on the 3 plate bolts): the cartridge interface every bay copies (30 deg lead-ins,
    locating faces, both latch pockets), the XT60E1-M float pocket + its lid, two TPU 95A stop strips, the pogo pad /
    KW11 target carrier, the Radxa standoffs and the flow sensor seat. No contact shoes, no steel plate;
  * placement sweep for the pad carrier, the Radxa and the flow sensor (keep-outs: cartridge and head path, the arm at
    the drone station, the funnel planes at +-5 deg yaw / +-2 deg tilt, the landing sweep of the belly post, FC, plate).
Numbers: nox/dimensions.md A, C, E, G, H, I, J, M. Doc: nox/scout/v3/bottom.md. Tags in the comments: P measured, C CAD,
E estimate, spec maker data. DEFAULT = chosen without Spark (log line in nox/log.md).

Frames (C). Drone frame d: x nose, y left, z up, CG at x 0, plate underside z 18.5. Seat frame (dimensions G) = d.
Cartridge frame c: x_c across the pack = -y, y_c up toward the drone (0 = flat bottom, at drone z -44.5), z_c along the
pack (0 = closed-end outer face, at drone x 37.5). c is a proper rotation of d (dimensions E wrote x_c = +y, a mirror;
harmless while everything was symmetric). Box W (dimensions I): X = -x_c = +y, Y = 175 - x (Y_C 175 at the tab line),
Z = z + 144.5.

python3 nox/scout/v3/scout_bottom.py            sweep, parts, checks, STLs, result.json  (runs/scout_bottom/v1/)
python3 nox/scout/v3/scout_bottom.py --fe       + FE recheck of the receiver and the tray (printopt, ~3 GB)
python3 nox/scout/v3/scout_bottom.py --motion   + motion.html / .mp4 / .gif / .png  (--motion-only: from the saved layout)
"""
import json, math, os, sys, copy, time
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "nox"))
OUT = os.path.join(ROOT, "runs", "scout_bottom", "v1"); os.makedirs(OUT, exist_ok=True)
PRINT = os.path.join(ROOT, "print")
S2 = math.sqrt(2.0)
RHO_PETG, RHO_TPU = 1.27e-3, 1.22e-3                       # g / mm3 (E, printopt)

# ======================================================================== frames (C)
T_CD = np.array([[0, -1, 0, 0], [0, 0, 1, 44.5], [-1, 0, 0, 37.5]], float)   # c <- d
T_DC = np.array([[0, 0, -1, 37.5], [-1, 0, 0, 0], [0, 1, 0, -44.5]], float)  # d <- c
def c2d(p): p = np.atleast_2d(p); return p @ T_DC[:, :3].T + T_DC[:, 3]
def d2c(p): p = np.atleast_2d(p); return p @ T_CD[:, :3].T + T_CD[:, 3]
def T4(A): M = np.eye(4); M[:3] = A; return M

# ======================================================================== helpers
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(axis, a, b, r, t0, t1, n=40):
    """cylinder along axis ('x', 'y', 'z') through the point (a, b) of the other two axes (in x, y, z order), t0..t1."""
    c = mf.Manifold.cylinder(t1 - t0, r, r, n)
    if axis == "z": return c.translate([a, b, t0])
    if axis == "x": return c.rotate([0, 90, 0]).translate([t0, a, b])
    return c.rotate([-90, 0, 0]).translate([a, t0, b])
def hullp(pts): return mf.Manifold.hull_points(np.asarray(pts, np.float64))
def prism_yz(poly, x0, x1): return hullp([[x, y, z] for (y, z) in poly for x in (x0, x1)])
def prism_xy(poly, z0, z1): return hullp([[x, y, z] for (x, y) in poly for z in (z0, z1)])
def prism_zy(poly, x0, x1): return hullp([[x, y, z] for (z, y) in poly for x in (x0, x1)])
def U(*ms):
    ms = [m for m in ms if m is not None and not m.is_empty()]
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else (ms[0] if ms else mf.Manifold())
def D2C(m): return m.transform(T_CD)
def C2D(m): return m.transform(T_DC)
def tm(m, T=None):
    me = m.to_mesh(); t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    if T is not None: t.apply_transform(T)
    return t
def vol(m): return 0.0 if m.is_empty() else m.volume()
def ov(a, b): return round(vol(a ^ b), 3)
def mirx(m): return m.mirror([1, 0, 0])

# ======================================================================== cartridge v4 (c coords, C unless tagged)
PACK = (33.0, 32.0, 70.0)          # pack x (width) / y (height) / z (length), 70 x 32 x 33 (P, dimensions A)
X_OUT, X_IN = 19.0, 17.0            # long walls 2.0 (v3 interface)
X_RAIL, RAIL_Y0, Y_TOP = 20.0, 38.0, 48.0      # guide rails x +-19..20, y 38..48 = the x datum; top 48 (pack top r 148)
RAIL_CH = 1.0                       # 45 deg chamfer on the rail's top outer edge (v4, no shoes to ride a 5 x 2 any more)
L_C = 93.0                          # v4 length (v3 79): pack 2..73, end stop 73..75, lead loop 75..79.6, cross wall, plug 81.6..90, end wall 90..93
Z_P0, Z_P1 = 2.0, 73.0              # pack cavity (70 + 0.5 a side)
Y_FL0, Y_FL1 = 12.35, 14.4          # floor 2.05 (over the recess ceiling 12.35, 2.0 rule)
Y_LIP, LIP = 47.4, 0.6              # pack keeper lips 0.6 at the top of the long walls (pack 32 tall sits 1.0 under them)
RIM = 4.0                           # flat bottom band y 0..4 all round
WINDOWS = [(18.0, 36.0, 7.0, 26.0), (18.0, 36.0, 50.0, 68.0)]   # long-wall windows (y0, y1, z0, z1), DEFAULT (-4.5 g)
# latch (dimensions J; v4: bar 9 wide in z with 2.0 skins, tabs 14 tall up to the pack's underside)
Z_BLK0, Z_BLK1 = 30.5, 44.5         # latch zone, tab line z 37.5 = drone x 0 = W Y_C
Z_TAB = 37.5
REC_X = 14.5                        # recess half width (open blade x 11..14 + 0.5)
CH_Y0, CH_Y1, CH_HW = 7.65, 12.35, 5.0          # slider channel (bar + 0.35 in y, + 0.5 in z)
BAR_Y0, BAR_Y1, BAR_HW = 8.0, 12.0, 4.5         # bar 4 x 9
TAB_IN, TAB_OUT, TAB_Y1 = 4.0, 10.0, 14.0       # tab faces at rest (0 / 6 squeezed: the tabs meet at x 0 = the stop)
RF = 20.3                           # receiver locating face, 0.3 off the rail
X_TIP = RF + 3.0                    # bolt tip, 3.0 overlap into the pocket
UNDER = math.radians(10.0)          # bolt underside undercut over the overlap
GP_YC, GP_H, GP_W, GP_D, GP_TAPER = 4.0, 4.0, 8.0, 3.0, math.radians(5.0)   # finger grip pocket (J)
SPR_D, SPR_Y, SPR_DEPTH = 3.2, 10.0, 4.0        # pen spring seats (J: o 3 spring, free ~17); y 10 keeps 2.0 walls
TRAVEL = 4.0
CH_DROP_X, CH_DROP_Y = 15.8, 7.1    # channel floor drop for the retracted undercut wedge (tip drops 0.53)
# pack XT60 (battery side, female: CHECK D17 which half the packs carry; DEFAULT the usual battery-side plug)
XF_W, XF_D, XF_H = 15.5, 8.1, 16.0  # housing x / z / y (spec face 15.5 x 8.1; length 16 E)
XF_KEY = 1.5                        # the D-shape's two chamfered corners (E), at x_c + (keys the pack plug's polarity)
Z_X = 85.8                          # plug axis z_c
Y_XF_FACE = 46.0                    # plug face at seat (C): 1.5 short of the XT60E1-M flange, so over-travel never reaches the housing
Y_XF_REAR = Y_XF_FACE - XF_H        # 30.0
P_CLR = 0.15                        # plug in its pocket, per side
Y_SH0, Y_SH1 = 27.5, Y_XF_REAR      # plug shelf (takes the 60 N push)
Z_CW0, Z_CW1 = 79.6, 81.6           # cross wall (pack side of the plug pocket)
Z_STOP = 76.0                       # pack end stop bar z 73..76, y 14.4..19.4 (the leads leave the end face above it, E)
Z_EW0 = 90.0                        # end wall 90..93
Y_PW1 = 40.0                        # plug pocket walls up to y 40
Y_ZONE_TOP = 44.2                   # cartridge top inside the plug zone (|x| <= 16.85): 1.3 under the receiver lips at 45.5
SS_Y, SS_X, SS_HOLE = 32.0, 4.0, 2.5            # 2 x M3 x 5 cup-point set screws along -z, self-tapped in o 2.5 (E)
SS_BOSS = (8.25, 27.75, 36.25, 3.0)             # boss block x +-8.25, y 27.75..36.25, 3.0 proud of the end face

def slider(rest=True, side=1):
    """one slider (print part, the same for both sides: symmetric in z about the tab line). side +1 = x_c > 0."""
    tab = box(TAB_IN, TAB_OUT, 0.0, TAB_Y1, Z_BLK0, Z_BLK1)
    bar = box(TAB_IN, X_TIP, BAR_Y0, BAR_Y1, Z_TAB - BAR_HW, Z_TAB + BAR_HW)
    drop = 3.0 * math.tan(UNDER)
    # undercut: bearing face = underside, sloping down 10 deg from the locating face to the tip (pull-out drives it outward)
    under = prism_xy([(RF, BAR_Y0), (X_TIP, BAR_Y0 - drop), (X_TIP, BAR_Y0)], Z_TAB - BAR_HW, Z_TAB + BAR_HW)
    cham = prism_xy([(X_TIP - 1.0, BAR_Y1 + 0.01), (X_TIP + 0.01, BAR_Y1 + 0.01), (X_TIP + 0.01, BAR_Y1 - 1.0)],
                    Z_TAB - BAR_HW - 1, Z_TAB + BAR_HW + 1)
    s = U(tab, bar, under) - cham
    # grip pocket on the tab's outer face: 8 (z) x 4 (y) mouth, 10 deg included taper, 3 deep, centred y 4
    h1 = GP_H - 2 * GP_D * math.tan(GP_TAPER); w1 = GP_W - 2 * GP_D * math.tan(GP_TAPER)
    gp = hullp([[TAB_OUT + 0.01, GP_YC + sy * GP_H / 2, Z_TAB + sz * GP_W / 2] for sy in (-1, 1) for sz in (-1, 1)] +
               [[TAB_OUT - GP_D, GP_YC + sy * h1 / 2, Z_TAB + sz * w1 / 2] for sy in (-1, 1) for sz in (-1, 1)])
    s = s - gp - cyl("x", SPR_Y, Z_TAB, SPR_D / 2, TAB_IN - 0.01, TAB_IN + SPR_DEPTH)
    if not rest: s = s.translate([-TRAVEL, 0, 0])
    return s if side > 0 else mirx(s)

def tray():
    """cartridge v4 tray with the latch frames built in (one print, closed end down)."""
    outer = box(-X_OUT, X_OUT, 0, Y_TOP, 0, L_C)
    rails = box(-X_RAIL, -X_OUT + 0.01, RAIL_Y0, Y_TOP, 0, L_C) + box(X_OUT - 0.01, X_RAIL, RAIL_Y0, Y_TOP, 0, L_C)
    t = outer + rails
    for s in (-1, 1):                                                       # rail top chamfer
        t = t - prism_xy([(s * (X_RAIL - RAIL_CH), Y_TOP + 0.01), (s * (X_RAIL + 0.01), Y_TOP + 0.01),
                          (s * (X_RAIL + 0.01), Y_TOP - RAIL_CH)], -1, L_C + 1)
    t = t - box(-X_IN, X_IN, Y_FL1, Y_TOP + 1, Z_P0, Z_P1)                   # pack cavity, open top
    t = t - box(-X_IN, X_IN, -1, Y_FL0, Z_P0, Z_P1)                          # open under the floor
    t = t - box(-X_IN + RIM - 2.0, X_IN - RIM + 2.0, -1, RIM + 0.01, Z_P0, L_C - RIM)   # (rim stays at the walls)
    # latch frames x 14.5..19 in the latch zone, recess between them (open bottom, up to the pack's underside)
    t = t + box(-X_OUT, -REC_X, 0, Y_FL1, Z_BLK0, Z_BLK1) + box(REC_X, X_OUT, 0, Y_FL1, Z_BLK0, Z_BLK1)
    t = t - box(-REC_X, REC_X, -1, Y_FL1 + 0.01, Z_BLK0, Z_BLK1)
    for s in (-1, 1):
        t = t - prism_xy([(s * (REC_X - 0.01), -0.01), (s * (REC_X + 1.0), -0.01), (s * (REC_X - 0.01), 1.0)], Z_BLK0 - 1, Z_BLK1 + 1)
        a0, a1 = sorted((s * (REC_X - 1), s * (X_OUT + 1)))
        t = t - box(a0, a1, CH_Y0, CH_Y1, Z_TAB - CH_HW, Z_TAB + CH_HW)      # channel through frame and wall
        a0, a1 = sorted((s * CH_DROP_X, s * (X_OUT + 1)))
        t = t - box(a0, a1, CH_DROP_Y, CH_Y0 + 0.01, Z_TAB - CH_HW, Z_TAB + CH_HW)
    # pack keeper lips
    for s in (-1, 1):
        t = t + prism_xy([(s * X_IN, Y_LIP - LIP), (s * (X_IN - LIP), Y_LIP), (s * (X_IN - LIP), Y_TOP), (s * X_IN, Y_TOP)], 6.0, 69.0)
    # long-wall windows
    for (y0, y1, z0, z1) in WINDOWS:
        t = t - box(-X_OUT - 2, -X_IN + 0.01, y0, y1, z0, z1) - box(X_IN - 0.01, X_OUT + 2, y0, y1, z0, z1)
    # ---- lead end: end stop, lead loop, cross wall, plug pocket, shelf, end wall, set screws
    zone = box(-16.85, 16.85, Y_ZONE_TOP, Y_TOP + 1, Z_P1, L_C + 1)         # plug zone held under the receiver lips
    t = t - zone
    t = t - box(-X_IN, X_IN, RIM, Y_ZONE_TOP + 0.01, Z_P1, Z_EW0)            # lead room (rebuilt below)
    t = t - box(-X_IN, X_IN, -1, RIM + 0.01, Z_P1, L_C - RIM)
    t = t + box(-X_IN - 0.01, X_IN + 0.01, Y_FL1 - 0.01, Y_FL1 + 5.0, Z_P1 - 0.01, Z_STOP)        # pack end stop
    t = t + box(-X_IN - 0.01, X_IN + 0.01, Y_SH0, Y_PW1, Z_CW0, Z_CW1)                     # cross wall (wires pass under it)
    t = t + box(-X_IN - 0.01, X_IN + 0.01, Y_SH0, Y_SH1, Z_CW0, Z_EW0)                     # shelf
    px, pz = XF_W / 2 + P_CLR, XF_D / 2 + P_CLR
    t = t + box(-px - 2.0, px + 2.0, Y_SH1, Y_PW1, Z_CW1 - 0.01, Z_EW0 + 0.01)   # pocket walls
    t = t - box(-px, px, Y_SH1, Y_PW1 + 1, Z_X - pz, Z_X + pz)               # plug pocket
    for sz in (-1, 1):                                                        # D-key: the two corners at x_c + filled 1.5 x 45
        z_a = Z_X + sz * pz
        t = t + hullp([[px - XF_KEY, y, z_a] for y in (Y_SH1, Y_PW1)] + [[px + 0.01, y, z_a] for y in (Y_SH1, Y_PW1)] +
                      [[px + 0.01, y, z_a - sz * XF_KEY] for y in (Y_SH1, Y_PW1)])
    t = t - box(-6.0, 6.0, Y_SH0 - 1, Y_SH1 + 0.01, Z_X - 2.6, Z_X + 2.6)    # wire slot (2 cups at 7.2, heat shrink o ~4)
    t = t + box(-X_IN - 0.01, X_IN + 0.01, 0, Y_ZONE_TOP, Z_EW0, L_C)                       # end wall (z datum face at 91)
    bx, by0, by1, bt = SS_BOSS
    t = t + box(-bx, bx, by0, by1, L_C - 0.01, L_C + bt)
    for sx in (-1, 1):
        t = t - cyl("z", sx * SS_X, SS_Y, SS_HOLE / 2, Z_EW0 - 0.5, L_C + bt + 1, 24)
    # flat bottom band (rim) at the ends
    t = t + box(-X_OUT, X_OUT, 0, RIM, 0, Z_P0 + RIM - 2.0) + box(-X_OUT, X_OUT, 0, RIM, L_C - RIM, L_C)
    t = t + box(-X_OUT, -X_IN + RIM - 2.0, 0, RIM, 0, L_C) + box(X_IN - RIM + 2.0, X_OUT, 0, RIM, 0, L_C)
    return t ^ D2C(funnel_env())                                             # bottom end edges chamfered 45 deg (collar planes)

# ======================================================================== receiver v4 interface (c coords)
SP1 = RF + 2.0                      # side plates x 20.3..22.3
FOOT_X = RF + 4.0                   # feet x 20.3..24.3 (lead-in foot, 4 thick)
Y_MOUTH, CAP = 2.0, 3.0             # receiver bottom 2 above the flat bottom; capture +-3.0 (E, v3)
LEAD = math.radians(30.0)
HCAP = CAP / math.tan(LEAD)         # 5.20
FOOT_Y1 = Y_MOUTH + HCAP + 2.0      # 9.2
Y_DK0, Y_DK1 = 49.5, 52.5           # deck (TPU strip seat on its underside)
Z_E0, Z_E1, END_T = -0.3, L_C + 0.3, 4.0      # end plate locating faces, 4 thick
Y_END0 = RAIL_Y0                    # end plates from y 38 (lead-in capture 3.0 at y 38)
XW = FOOT_X                         # receiver half width 24.3
DECK_WIN = (14.5, 4.0, 74.0)        # deck window under the FC: x +-14.5, z 4..74 (strip seats at |x| 15.4..20.3 stay)
# latch pockets (J, v4 bar 9 wide): mouth y 7.75..12.3 at x 20.3, 3.3 deep, floor 10 deg down going in, 0.8 a side in z
PK_Y0, PK_Y1, PK_D, PK_ZC = 7.75, 12.3, 3.3, 0.8
PK_WALL = 3.0
# XT60E1-M (device side, pins; spec flange 27.1 x 12.1 x 3.0, screws 20.5 apart; insert 13.3 x 5.9 x 7.5 and rear 15.5 x 8.1 x 6.2 E)
XM_FW, XM_FD, XM_FT = 27.1, 12.1, 3.0
XM_IW, XM_ID, XM_IL = 13.3, 5.9, 7.5
XM_RW, XM_RD, XM_RL = 15.5, 8.1, 6.2
Y_XM = Y_XF_FACE + 1.5              # flange front face 47.5 (C)
FLOAT, AX_PLAY = 1.0, 0.5           # +-1.0 lateral by clearance; 0.5 axial play = +-1.06 deg free tilt over the flange (C)
PK_XW = XM_FW / 2 + FLOAT           # pocket x +-14.55
PK_ZW = XM_FD / 2 + FLOAT           # pocket z +-7.05
Y_PCEIL = Y_XM + XM_FT + AX_PLAY    # 51.0 push face (lid key)
LIP_X0, LIP_Y0 = 9.5, 45.5          # pull lips under the flange ends, x +-9.5..14.55, y 45.5..47.5
XP_WALL = 2.0
XF_PATH = (8.5, 4.8)                # female / male path kept void below the flange: x +-8.5, z +-4.8 (located +-0.3 + 0.15 + float)
LID_X, LID_Z0, LID_Z1, LID_T = 31.0, Z_X - 10.0, Z_X + 10.0, 4.0
LID_BOLT_X, LID_INS_D, LID_INS_DEPTH = 25.6, 4.0, 6.7          # 2 x M3 x 10 into M3 x 5.7 heat-set (hole o 4.0, M)
LID_WIN = (9.25, 5.55)              # window for the male's rear body + wires (x +-9.25, z +-5.55)
# TPU 95A stop strips (2): in a 0.9 recess on the deck underside over the cartridge's top band, face at y 48.0 (zero gap at seat)
STRIP_X0, STRIP_X1, STRIP_Z0, STRIP_Z1, STRIP_T, STRIP_REC = 15.4, 20.3, 1.0, 72.0, 2.4, 0.9
# plate bolts (dimensions E): sides (3.9, +-31) and nose (45, 0), M3 from the top into M3 x 5.7 heat-set inserts (DEFAULT, was nylocs)
Y_PLATE = 18.5 + 44.5               # plate underside, y_c 63.0
BOSS_R = 6.0                        # pad o 12 (pad rule), insert hole o 4.0 x 6.7: wall 4.0
SIDE_BOSS_D = (3.9, 32.0)           # side boss centred 1.0 outboard of the bolt (3.9, +-31): 0.83 clear of the FC corner (C)
NOSE_BOSS_D = (45.0, 0.0)
INS_D, INS_DEPTH = 4.0, 6.7

def pocket_cut():
    zc0, zc1 = Z_TAB - BAR_HW - PK_ZC, Z_TAB + BAR_HW + PK_ZC
    yb = PK_Y0 - PK_D * math.tan(UNDER)
    pk = hullp([[RF - 0.02, y, z] for y in (PK_Y0, PK_Y1) for z in (zc0, zc1)] +
               [[RF + PK_D, y, z] for y in (yb, PK_Y1) for z in (zc0, zc1)])
    ch = prism_xy([(RF - 0.02, PK_Y1 + 1.0), (RF - 0.02, PK_Y1 - 0.01), (RF + 1.0, PK_Y1 - 0.01)], zc0, zc1)
    return pk + ch, (zc0, zc1, yb)

def pocket_block():
    pk, (zc0, zc1, yb) = pocket_cut()
    body = box(RF, RF + PK_D + PK_WALL, yb - PK_WALL, PK_Y1 + PK_WALL, zc0 - PK_WALL, zc1 + PK_WALL)
    return body, pk

def interface_parts():
    """the cartridge interface every bay copies (c coords, both sides): side plates, feet, end plates, pockets, deck."""
    parts = []
    z0, z1 = Z_E0 - END_T, Z_E1 + END_T
    for s in (-1, 1):
        a0, a1 = sorted((s * RF, s * SP1))
        parts.append(box(a0, a1, Y_MOUTH + HCAP, Y_DK1, z0, z1))
        poly = [(RF + CAP, Y_MOUTH), (FOOT_X, Y_MOUTH), (FOOT_X, FOOT_Y1), (RF, FOOT_Y1), (RF, Y_MOUTH + HCAP)]
        parts.append(prism_xy([(s * x, y) for x, y in poly], z0, z1))
        b, _ = pocket_block()
        parts.append(b if s > 0 else mirx(b))
    for zf, sg in ((Z_E0, -1), (Z_E1, 1)):                                   # end plates with 30 deg lead-ins
        pts = [(zf + sg * CAP, Y_END0), (zf + sg * END_T, Y_END0), (zf + sg * END_T, Y_DK1), (zf, Y_DK1), (zf, Y_END0 + HCAP)]
        parts.append(prism_zy(pts, -XW, XW))
    parts.append(box(-XW, XW, Y_DK0, Y_DK1, z0, z1))                         # deck
    return U(*parts)

def interface_cuts():
    """what the cartridge interface keeps empty (c coords): cartridge path with its lead-ins, pockets, strip recesses."""
    cuts = []
    pk, _ = pocket_cut(); cuts += [pk, mirx(pk)]
    for s in (-1, 1):
        a0, a1 = sorted((s * STRIP_X0, s * STRIP_X1))
        cuts.append(box(a0, a1, Y_DK0 - 1, Y_DK0 + STRIP_REC, STRIP_Z0, STRIP_Z1))
    cuts.append(box(-DECK_WIN[0], DECK_WIN[0], Y_DK0 - 1, Y_DK1 + 1, DECK_WIN[1], DECK_WIN[2]))
    return U(*cuts)

def cart_path():
    """the space the cartridge (and its 3.0 capture) sweeps on the way in; nothing of the receiver below the faces."""
    c = box(-RF, RF, -60, Y_DK0, Z_E0, Z_E1)
    k = math.tan(LEAD)
    for s in (-1, 1):                                                        # x lead-in wedge below the feet
        poly = [(0, -60), (RF + CAP + 62 * k, -60), (RF + CAP, Y_MOUTH), (RF, Y_MOUTH + HCAP), (0, Y_MOUTH + HCAP)]
        c = c + prism_xy([(s * x, y) for x, y in poly], Z_E0 - CAP - 1, Z_E1 + CAP + 1)
    for zf, sg in ((Z_E0, -1), (Z_E1, 1)):                                   # z lead-in wedge under the end plates
        poly = [(L_C / 2, -60), (zf + sg * CAP, -60), (zf + sg * CAP, Y_END0), (zf, Y_END0 + HCAP), (L_C / 2, Y_END0 + HCAP)]
        c = c + prism_zy(poly, -RF, RF)
    return c

def xt60_pocket():
    """drawn XT60E1-M float pocket (c coords): body, void; lid insert bosses."""
    zc = Z_X
    body = box(-PK_XW - XP_WALL, PK_XW + XP_WALL, LIP_Y0, Y_DK1, zc - PK_ZW - XP_WALL, zc + PK_ZW + XP_WALL)
    void = box(-PK_XW, PK_XW, Y_XM, Y_DK1 + 10, zc - PK_ZW, zc + PK_ZW)       # flange space, open to the top for the lid
    path = box(-XF_PATH[0], XF_PATH[0], 20.0, Y_XM + 0.01, zc - XF_PATH[1], zc + XF_PATH[1])
    bosses = U(*[cyl("y", s * LID_BOLT_X, zc, 5.0, Y_DK1 - LID_INS_DEPTH - 2.0, Y_DK1) for s in (-1, 1)])
    holes = U(*[cyl("y", s * LID_BOLT_X, zc, LID_INS_D / 2, Y_DK1 - LID_INS_DEPTH, Y_DK1 + 1) for s in (-1, 1)])
    return body + bosses, void + path + holes

def lid():
    zc = Z_X
    l = box(-LID_X, LID_X, Y_DK1, Y_DK1 + LID_T, LID_Z0, LID_Z1)
    for s in (-1, 1):                                                        # keys down onto the flange ends (push face y 51.0)
        a0, a1 = sorted((s * (LID_WIN[0] + 0.01), s * (PK_XW - 0.25)))
        l = l + box(a0, a1, Y_PCEIL, Y_DK1 + 0.01, zc - PK_ZW + 0.25, zc + PK_ZW - 0.25)
    l = l - box(-LID_WIN[0], LID_WIN[0], Y_PCEIL - 1, Y_DK1 + LID_T + 1, zc - LID_WIN[1], zc + LID_WIN[1])
    for s in (-1, 1):
        l = l - cyl("y", s * LID_BOLT_X, zc, 1.7, Y_DK1 - 1, Y_DK1 + LID_T + 1, 24)
    return l

def tpu_strip():
    return box(0, STRIP_X1 - STRIP_X0 - 0.1, 0, STRIP_T, 0, STRIP_Z1 - STRIP_Z0 - 0.4)

def xt60_female(dy=0.0):
    """pack plug (bought, c coords at seat + dy): housing + two lead stubs."""
    h = box(-XF_W / 2, XF_W / 2, Y_XF_REAR, Y_XF_FACE, Z_X - XF_D / 2, Z_X + XF_D / 2)
    for sz in (-1, 1):
        z_a = Z_X + sz * XF_D / 2
        h = h - hullp([[XF_W / 2 - XF_KEY, y, z_a + sz * 0.01] for y in (Y_XF_REAR - 1, Y_XF_FACE + 1)] +
                      [[XF_W / 2 + 0.01, y, z_a + sz * 0.01] for y in (Y_XF_REAR - 1, Y_XF_FACE + 1)] +
                      [[XF_W / 2 + 0.01, y, z_a - sz * XF_KEY] for y in (Y_XF_REAR - 1, Y_XF_FACE + 1)])
    h = h - box(-6.65, 6.65, Y_XF_FACE - XM_IL, Y_XF_FACE + 1, Z_X - 2.95, Z_X + 2.95)   # receptacle for the male insert
    w = U(*[cyl("y", sx * 3.6, Z_X, 1.8, Y_XF_REAR - 8.0, Y_XF_REAR + 0.5) for sx in (-1, 1)])
    return (h + w).translate([0, dy, 0])

def xt60_male():
    """XT60E1-M at its nominal float centre (bought, c coords)."""
    f = box(-XM_FW / 2, XM_FW / 2, Y_XM, Y_XM + XM_FT, Z_X - XM_FD / 2, Z_X + XM_FD / 2)
    f = f - U(*[cyl("y", s * 10.25, Z_X, 1.6, Y_XM - 1, Y_XM + XM_FT + 1, 20) for s in (-1, 1)])
    i = box(-XM_IW / 2, XM_IW / 2, Y_XM - XM_IL, Y_XM, Z_X - XM_ID / 2, Z_X + XM_ID / 2)
    r = box(-XM_RW / 2, XM_RW / 2, Y_XM + XM_FT, Y_XM + XM_FT + XM_RL, Z_X - XM_RD / 2, Z_X + XM_RD / 2)
    w = U(*[cyl("y", sx * 3.6, Z_X, 1.8, Y_XM + XM_FT + XM_RL - 0.5, Y_XM + XM_FT + XM_RL + 5) for sx in (-1, 1)])
    return f + i + r + w

# ======================================================================== drone context (d coords, P / E)
FC_C, FC_HALF, FC_Z = (1.9, 0.0), 18.0, (9.5, 17.5)     # MicoAir743 AIO 36 x 36 diamond at (1.9, 0), lowest z 9.5 (H, P / spec)
def fc_body():
    b = box(-FC_HALF, FC_HALF, -FC_HALF, FC_HALF, FC_Z[0], FC_Z[1]).rotate([0, 0, 45]).translate([FC_C[0], FC_C[1], 0])
    return b + box(FC_C[0] - 3, FC_C[0] + 3, -3, 3, FC_Z[1], 18.5)
def plate():
    p = box(-50, 55, -47, 47, 18.5, 22.0)                                   # cut-down Bee35 plate (C / E: tail x -50 CHECK)
    for sx in (1, -1):
        for sy in (1, -1):
            p = p + hullp([[0, sy * 20, z] for z in (18.5, 22.0)] + [[sx * 54.1, sy * 54.1 + d, z] for d in (-5, 5) for z in (18.5, 22.0)])
    return p
CG = np.array([0.0, 0.0, 4.2])                          # E (dimensions H flow row)

# Radxa Zero 3W (H, C from Radxa's STEP): board frame X along the 65, Y along the 30, Z 0 = PCB top (component side)
RX_L, RX_W, RX_PCB = 65.0, 30.0, 1.6
RX_HOLES = [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]
RX_USB = [(37.1, 46.0), (49.6, 58.5)]; RX_HDMI = (9.2, 15.7)
def radxa_pose(x0, z_top, s):
    """board X -> drone -x (SD end X 0 at x0, the nose end; the card goes in from the nose), Y -> drone -z (ports edge
    Y 0 on top at z_top), Z -> drone -y (component side outward, the flat underside toward the side plate, its PCB face
    at drone y = -(SP1 + s)). Returns the 4x4 d <- board (proper rotation)."""
    M = np.eye(4); M[:3, 0] = [-1, 0, 0]; M[:3, 1] = [0, 0, -1]; M[:3, 2] = [0, -1, 0]
    M[:3, 3] = [x0, -(SP1 + s) - RX_PCB, z_top]
    return M
def radxa_board():
    keep = U(*[cyl("z", hx, hy, 3.8, -RX_PCB - 1.6, 1.7, 32) for hx, hy in RX_HOLES])   # hole pads o 7.6 free of parts (E, CHECK)
    b = box(0, RX_L, 0, RX_W, -RX_PCB, 0) + ((box(1.0, RX_L - 1.0, 1.0, RX_W - 1.0, 0, 1.6) + box(1.0, RX_L - 1.0, 1.0, RX_W - 1.0, -RX_PCB - 1.55, -RX_PCB)) - keep)
    for a, c in RX_USB: b = b + box(a, c, -1.5, 7.0, 0, 3.16)
    b = b + box(RX_HDMI[0], RX_HDMI[1], -1.25, 7.0, 0, 3.03) + box(0, 4.9, 7.8, 22.3, -RX_PCB - 2.86, -RX_PCB)
    for (hx, hy) in RX_HOLES:                       # mounting-hole keep-out o 9.6 on both sides (E, CHECK on the board)
        b = b - cyl("z", hx, hy, 1.41, -6, 5, 16)
    return b
def radxa_plugs():
    """90 deg USB-C plug envelopes (E: 12.4 x 6.5, 8 up from the port; the cable leaves along the board edge), board frame."""
    return U(*[box((a + c) / 2 - 6.2, (a + c) / 2 + 6.2, -9.5, -1.5, -1.67, 4.83) for a, c in RX_USB])

# MTF-02P (A, P / spec): board 22 (x) x 16 (y) x 8 tall; lens 6 from the nose edge and 6 from the right (-y) edge, on its underside
FL_L, FL_W, FL_T = 22.0, 16.0, 8.0
FL_HALF = math.radians(23.0)                            # 42 deg FOV + 2 (H)
def flow_board(xl, yl, zl): return box(xl - 16.0, xl + 6.0, yl - 6.0, yl + 10.0, zl, zl + FL_T)

# keep-outs (d coords). K1 cartridge path (brief: W X +-23 over the pack's length below the receiver; inside it the
# exact path with its lead-ins, cart_path()), K2 head at the drone station
# (dimensions I head rules, Y_C 175: |X| <= 19 up to r 105, else r <= 98.5), K3 carriage / rail at the drone (I, E).
def keepouts():
    k = {}
    k["K1 cartridge path below the receiver"] = box(-60.0, 41.0, -23.0, 23.0, -120.0, -42.0)   # above: cart_path() (exact)
    k["K2 head |y|<=19 to r105"] = box(-60.0, 77.0, -19.0, 19.0, -120.0, -39.5)
    k["K2 head to r98.5"] = box(-60.0, 77.0, -50.0, 50.0, -120.0, -46.0)
    k["K3 carriage plate"] = box(77.0, 81.0, -28.0, 28.0, -120.0, -8.9)
    k["K3 carriage wings"] = box(77.0, 81.0, -46.5, 46.5, -120.0, -24.9)
    k["K3 rail + bar"] = box(81.0, 101.0, -20.0, 20.0, -120.0, -10.5)
    k["camera (v3 mount, E)"] = box(52.0, 75.0, -21.0, 21.0, -14.0, 26.0)     # from x 52: the nose boss is the camera's mount too
    k["FC"] = fc_body()
    return k

# ---- funnel planes (dimensions G seat frame): contact faces max(|x|, |y|) = 84.8 + z; margin = distance to the plane
def face_margin(P, yaw=5.0, tilt=2.0):
    worst = 1e9
    for a in (-yaw, 0.0, yaw):
        for tx in (-tilt, 0.0, tilt):
            for ty in (-tilt, 0.0, tilt):
                R = trimesh.transformations.euler_matrix(math.radians(tx), math.radians(ty), math.radians(a), "sxyz")[:3, :3]
                Q = (P - CG) @ R.T + CG
                m = (84.8 + Q[:, 2] - np.maximum(np.abs(Q[:, 0]), np.abs(Q[:, 1]))) / S2
                worst = min(worst, float(m.min()))
    return worst

FUNNEL_INSET = 5.0                  # C: inset of the clip solid so the exact +-5 deg / +-2 deg check reads >= 1.0 (set by the run)
def funnel_env(inset=FUNNEL_INSET):
    """seat-frame solid inside the collar planes extended below the opening, inset (d coords): max(|x|,|y|) <= 84.8 + z - inset."""
    z0, z1 = inset - 84.8 + 0.5, 60.0
    w0, w1 = 0.5, 84.8 + z1 - inset
    return hullp([[sx * w, sy * w, z] for (w, z) in ((w0, z0), (w1, z1)) for sx in (-1, 1) for sy in (-1, 1)])

# ---- belly post (box side, context for the checks; the belly-post worker builds it): pins 2 per pad, travel >= 8
PIN_FREE = 8.0                      # free pin tip above the seated pad face (box electronics v1: travel >= 8, E)
POST_DROP = 2.0                     # post body top under the pad face (E)
def post_model(xp, yp, zpad, n=None):
    z_face = zpad - 0.6
    body = box(xp - 21.0, xp + 21.0, yp - 5.0, yp + 5.0, z_face - POST_DROP - 10.0, z_face - POST_DROP)
    tips = [(xp + sx * 12.0 + dx, yp, z_face + PIN_FREE) for sx in (-1, 1) for dx in (-2.0, 2.0)]
    tips.append((xp, yp, z_face + 3.0))                                     # KW11 roller, free 3 above the target (E)
    return body, np.array(tips)

def cone_hits(Q, P, rmax=40.0):
    """drone points P inside the downward 45 deg cone (lateral <= depth, lateral <= rmax) below any post point Q."""
    hits = 0; worst = 1e9
    for q in Q:
        lat = np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]); dep = q[2] - P[:, 2]
        m = (lat > 0) & (lat <= rmax) & (dep > 0)
        if m.any():
            gap = (lat[m] - dep[m]) / S2
            worst = min(worst, float(gap.min())); hits += int((gap < 0).sum())
    return hits, worst

def sample(m, pitch=1.5):
    """surface + interior-ish sample points of a manifold (vertices + face centres + a coarse surface sample)."""
    t = tm(m)
    pts = [t.vertices, t.triangles_center]
    try: pts.append(trimesh.sample.sample_surface_even(t, int(max(200, t.area / pitch ** 2)))[0])
    except Exception: pass
    return np.vstack(pts)

# ======================================================================== drone-specific receiver features (d coords)
PAD_W, PAD_L, PAD_T, PAD_REC = 6.0, 10.0, 1.6, 1.0      # pogo pads: copper-clad FR4 6 x 10 x 1.6 (E) in a 1.0 recess, 0.6 proud
PAD_DX = 12.0                       # B+ at x_p + 12 (nose side), B- at x_p - 12, KW11 target at x_p (DEFAULT)
CAR_HALF = 20.5                     # carrier length +-20.5 along x, 45 deg chamfers 3 on the bottom edges (pins slide in)
Y_IN = SP1 - 1.0                    # carrier / standoffs start 1.0 inside the side plate (drone |y| 21.3)
RX_BOSS_R, RX_INS_D, RX_INS_DEPTH = 3.6, 3.2, 6.7      # M2.5 heat-set (o 3.5 x 5.7 E, hole 3.2): boss o 7.2, wall 2.0 (clears the SD slot by 0.6)

def pad_carrier(xp, yp, zp):
    """belly pad carrier on the +y side plate: bottom face = KW11 target plane z_p; pads recessed in it."""
    top = zp + 4.0 + (yp + 7.5 - Y_IN)
    poly = [(Y_IN, zp), (yp + 3.5, zp), (yp + 7.5, zp + 4.0), (Y_IN, top)]
    c = hullp([[x, y, z] for (y, z) in poly for x in (xp - CAR_HALF, xp + CAR_HALF)])
    for s in (-1, 1):                                                       # 45 deg chamfers on the bottom x-edges
        x0 = xp + s * (CAR_HALF - 3.0)
        c = c - hullp([[x0, y, zp - 0.01] for y in (Y_IN - 1, yp + 10)] + [[x0 + s * 20, y, zp - 0.01] for y in (Y_IN - 1, yp + 10)] +
                      [[x0 + s * 20, y, zp + 20] for y in (Y_IN - 1, yp + 10)])
    cut = mf.Manifold()
    for sx in (-1, 1):
        px = xp + sx * PAD_DX
        cut = cut + box(px - PAD_L / 2 - 0.2, px + PAD_L / 2 + 0.2, yp - PAD_W / 2 - 0.2, yp + PAD_W / 2 + 0.2, zp - 1, zp + PAD_REC)
        cut = cut + cyl("z", px, yp, 1.2, zp - 1, top + 5, 20)              # lead hole up to the carrier top
    return c, cut

def pads_model(xp, yp, zp):
    return U(*[box(xp + sx * PAD_DX - PAD_L / 2, xp + sx * PAD_DX + PAD_L / 2, yp - PAD_W / 2, yp + PAD_W / 2,
                   zp + PAD_REC - PAD_T, zp + PAD_REC) for sx in (-1, 1)])

def radxa_holes_d(x0, z_top, s):
    M = radxa_pose(x0, z_top, s)
    return [(M @ np.array([hx, hy, 0, 1.0]))[:3] for hx, hy in RX_HOLES]

def radxa_standoffs(x0, z_top, s):
    b, cut = mf.Manifold(), mf.Manifold()
    yt = -(SP1 + s)
    for p in radxa_holes_d(x0, z_top, s):
        b = b + cyl("y", p[0], p[2], RX_BOSS_R, yt, -Y_IN)
        cut = cut + cyl("y", p[0], p[2], RX_INS_D / 2, yt - 0.01, yt + RX_INS_DEPTH, 24)
    return b, cut

def flow_seat(xl, yl, zl):
    """flow sensor seat on the +y side plate (d coords): tape land over the board, 2.0 rims on the nose and both long
    sides (tail end open for the plug), the land runs into the side plate, two 45 deg gussets over it."""
    clr, rim, land = 0.3, 2.0, 2.0
    x0, x1, y0, y1 = xl - 16.0 - clr, xl + 6.0 + clr, yl - 6.0 - clr, yl + 10.0 + clr
    s = box(x0 - rim, x1 + rim, Y_IN, y1 + rim, zl + FL_T, zl + FL_T + land)
    s = s + box(x1, x1 + rim, y0 - rim, y1 + rim, zl + 5.0, zl + FL_T + 0.01)              # nose rim
    for (a, b) in ((max(Y_IN, y0 - rim), y0), (y1, y1 + rim)):
        s = s + box(x0 - rim, x1 + rim, a, b, zl + 5.0, zl + FL_T + 0.01)                  # side rims (tail open: plug)
    zt = zl + FL_T + land
    for xg in (x0 - rim, x1):                                                               # gussets into the side plate
        poly = [(Y_IN, zt - 0.01), (y1 + rim, zt - 0.01), (Y_IN, zt + (y1 + rim - Y_IN))]
        s = s + hullp([[x, y, z] for (y, z) in poly for x in (xg, xg + rim)])
    cut = box(x0, x1, y0, y1, zl - 1.0, zl + FL_T)
    return s, cut

def flow_cone(xl, yl, zl, depth=120.0):
    r = depth * math.tan(FL_HALF)
    return mf.Manifold.cylinder(depth, r, 0.01, 48).translate([xl, yl, zl - depth])

# ======================================================================== receiver assembly
def bosses_c():
    """plate bosses (c coords): side pads o 12 on deck wings with gussets, nose pad on a deck tongue."""
    parts, holes = [], []
    for sy in (1, -1):
        xd, yd = SIDE_BOSS_D
        p = d2c([xd, sy * yd, 0])[0]; bx, bz = p[0], p[2]
        bolt_x = d2c([3.9, sy * 31.0, 0])[0][0]
        parts.append(cyl("y", bx, bz, BOSS_R, Y_DK0, Y_PLATE))
        a0, a1 = sorted((np.sign(bx) * (XW - 0.5), bx + np.sign(bx) * 6.0))
        parts.append(box(a0, a1, Y_DK0, Y_DK1, bz - 10.0, bz + 10.0))                     # deck wing
        parts.append(prism_zy([(bz - 10.0, Y_DK1 - 0.01), (bz + 10.0, Y_DK1 - 0.01), (bz + 5.5, Y_DK1 + 6.5), (bz - 5.5, Y_DK1 + 6.5)],
                              bx - 1.25, bx + 1.25))                                         # gusset along z
        g0, g1 = sorted((np.sign(bx) * (XW - 0.5), bx))
        parts.append(prism_xy([(g0, Y_DK1 - 0.01), (g1, Y_DK1 - 0.01), (g1 if bx > 0 else g0, Y_DK1 + 3.5)], bz - 1.25, bz + 1.25))
        # post 6 x 6 on the side plate's outer face from the latch pocket block up to the wing: the side boss joins thick
        # material and the pull-out goes straight from the pockets to the plate bolt (attachment rule: posts >= 6 x 6)
        sg = np.sign(bx)
        a0, a1 = sorted((sg * (SP1 - 0.5), sg * (SP1 + 6.0)))
        parts.append(box(a0, a1, PK_Y1 + PK_WALL - 0.5, Y_DK1, bz - 3.0, bz + 3.0))
        parts.append(prism_xy([(sg * (SP1 + 5.9), Y_DK0 + 0.01), (sg * (abs(bx) + 5.0), Y_DK0 + 0.01), (sg * (SP1 + 5.9), Y_DK0 - 8.0)],
                              bz - 1.25, bz + 1.25))                                         # gusset post -> boss under the wing
        holes.append(cyl("y", bolt_x, bz, INS_D / 2, Y_PLATE - INS_DEPTH, Y_PLATE + 1, 24))
        holes.append(cyl("y", bolt_x, bz, 1.7, Y_DK0 - 1.0, Y_PLATE, 24))                # bolt relief through: a tube, no solid core
    p = d2c([NOSE_BOSS_D[0], NOSE_BOSS_D[1], 0])[0]; bx, bz = p[0], p[2]
    parts.append(cyl("y", bx, bz, BOSS_R, Y_DK0, Y_PLATE))
    parts.append(box(-8.0, 8.0, Y_DK0, Y_DK1, bz - 6.0, Z_E0 - 0.5))                       # nose tongue
    for s in (-1, 1):
        parts.append(prism_xy([(s * 5.5, Y_DK1 - 0.01), (s * 8.0, Y_DK1 - 0.01), (s * 5.5, Y_DK1 + 3.0)], bz - 1.25, bz + 1.25))
    holes.append(cyl("y", bx, bz, INS_D / 2, Y_PLATE - INS_DEPTH, Y_PLATE + 1, 24))
    holes.append(cyl("y", bx, bz, 1.7, Y_DK0 - 1.0, Y_PLATE, 24))
    return U(*parts), U(*holes)

SIDE_WIN = [(16.0, 42.0, 12.0, 27.0), (16.0, 42.0, 48.0, 64.0), (16.0, 42.0, 68.0, 86.0)]   # side plate windows (y_c, z_c), skipped where a feature sits, DEFAULT

def receiver(L, with_layout=True):
    """receiver v4 (c coords). L = layout dict from the sweep."""
    body = interface_parts()
    xb, xv = xt60_pocket()
    bb, bh = bosses_c()
    body = U(body, xb, bb)
    feat, cuts = mf.Manifold(), [interface_cuts(), xv, bh, cart_path()]
    if with_layout:
        xp, yp, zp = L["pads"]; c, cc = pad_carrier(xp, yp, zp); feat = feat + D2C(c); cuts.append(D2C(cc))
        x0, zt, s = L["radxa"]; b, bc = radxa_standoffs(x0, zt, s); feat = feat + D2C(b); cuts.append(D2C(bc))
        xl, yl, zl = L["flow"]; f, fc = flow_seat(xl, yl, zl); feat = feat + D2C(f); cuts.append(D2C(fc))
    body = (body + feat) ^ D2C(funnel_env())                                # 45 deg end chamfers: inside the collar planes
    for (y0, y1, z0, z1) in SIDE_WIN:
        for sgn in (-1, 1):
            a0, a1 = sorted((sgn * (RF - 0.5), sgn * (SP1 + 0.5)))
            w = box(a0, a1, y0, y1, z0, z1)
            if vol(w ^ U(feat, bb)) == 0.0:                                 # keep the plate where a feature sits on it
                cuts.append(w)
    return body - U(*cuts)

# ======================================================================== placement sweep
OVERSEER_Y = 31.0                   # Overseer cartridge path |y| <= 30 (~54 wide + 3 capture, E) + 1: the shared pads stay outside
FACE_MIN = 1.0                      # dimensions G: nothing of the drone within 1.0 of a face (planes extended below the opening)

def drone_points_base():
    """drone points the belly post must not meet in a landing (d coords): interface, XT60 pocket, bosses, cartridge."""
    xb, _ = xt60_pocket(); bb, _ = bosses_c()
    ifc = C2D(U(interface_parts(), xb, bb))
    cart = C2D(U(tray(), slider(True, 1), slider(True, -1)))
    return np.vstack([sample(ifc, 2.0), sample(cart, 2.0)]), ifc, cart

def z_r(y, r):                      # drone z at which W radius r is reached at lateral y (W X = y, Z = z + 144.5)
    return math.sqrt(max(r * r - y * y, 0.0)) - 144.5

def sweep_pads(P_base, log=print):
    rows = []
    for zp in np.arange(-44.0, -33.9, 0.5):
        for yp in np.arange(OVERSEER_Y + PAD_W / 2, 40.01, 0.5):
            for xp in (-10.0, -5.0, 0.0, 5.0, 10.0):
                c, _ = pad_carrier(xp, yp, zp)
                Pc = tm(c).vertices
                fm = face_margin(Pc)
                if fm < FACE_MIN: continue
                body, tips = post_model(xp, yp, zp)
                Pb = tm(body).vertices
                rmin = float(np.hypot(Pb[:, 1], Pb[:, 2] + 144.5).min())
                if rmin < 101.0: continue                                   # heads sweep r <= 98.5 (dimensions I), 2.5 margin
                h1, g1 = cone_hits(tips, P_base); h2, g2 = cone_hits(Pb, P_base)
                if h1 or h2: continue
                reach = yp + 7.5 - Y_IN
                post_len = (zp - 0.6 - POST_DROP) - z_r(yp, 101.0)
                score = reach + 0.6 * post_len + 0.1 * abs(xp)
                rows.append(dict(xp=float(xp), yp=float(yp), zp=float(zp), face=round(fm, 2), cone_tips=round(g1, 2),
                                 cone_post=round(g2, 2), r_min=round(rmin, 1), reach=round(reach, 1), post_len=round(post_len, 1),
                                 score=round(score, 2)))
    rows.sort(key=lambda r: r["score"])
    log(f"pads: {len(rows)} feasible; best {rows[0] if rows else None}")
    return rows

def sweep_radxa(fixed, log=print):
    """fixed: manifold (d) the board, its plugs and standoffs must clear (receiver body + pad carrier + context)."""
    rows = []
    board, plugs = radxa_board(), radxa_plugs()
    ko = keepouts(); kom = U(*[ko[k] for k in ko if not k.startswith("camera")])
    pk, _ = pocket_cut(); pkd = C2D(U(pk, mirx(pk)))
    for s in (6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 10.0):
        for x0 in np.arange(8.0, 41.01, 1.0):
            for zt in np.arange(-8.0, 6.01, 1.0):
                M = radxa_pose(x0, zt, s)
                B = (board + plugs).transform(M[:3])
                if vol(B ^ fixed) > 0.0 or vol(B ^ kom) > 0.0: continue
                sb, sc = radxa_standoffs(x0, zt, s)
                hs = radxa_holes_d(x0, zt, s)
                if any(not (-37.3 + RX_BOSS_R <= p[2] <= 8.0 - RX_BOSS_R and -57.8 + RX_BOSS_R <= p[0] <= 41.8 - RX_BOSS_R) for p in hs):
                    continue                                                # standoffs on the side plate
                if vol(sb ^ pkd) > 0.0 or vol(sc ^ pkd) > 0.0: continue     # off the latch pockets
                fm = face_margin(tm(B).vertices)
                if fm < FACE_MIN: continue
                xc = x0 - RX_L / 2; zc = zt - RX_W / 2
                score = s + 0.1 * abs(xc) + 0.05 * abs(zc + 12.0)
                rows.append(dict(x0=float(x0), z_top=float(zt), s=float(s), face=round(fm, 2), score=round(score, 2)))
    rows.sort(key=lambda r: r["score"])
    log(f"radxa: {len(rows)} feasible; best {rows[0] if rows else None}")
    return rows

def sweep_flow(fixed, fixed_seat, log=print):
    """MTF-02P on the +y side plate beside the pad carrier (the nose is out: under the camera keep-out a seat at x > 56
    sits up to 12 outside the funnel's x planes, C), lens to the nose (SENS_FLOW_ROT unchanged)."""
    rows = []
    ko = keepouts(); kom = U(*[ko[k] for k in ko if not k.startswith("camera")])
    cp = C2D(cart_path())
    for zl in np.arange(-40.0, -27.9, 0.5):
        for xl in list(np.arange(-44.0, -23.9, 1.0)) + list(np.arange(40.0, 56.01, 1.0)):
            for yl in (29.0, 30.0, 31.0, 32.0):
                s, c = flow_seat(xl, yl, zl)
                if vol(s ^ kom) > 0.0 or vol(s ^ cp) > 0.0 or vol(flow_board(xl, yl, zl) ^ kom) > 0.0: continue
                if vol(s ^ fixed_seat) > 0.0 or vol(flow_board(xl, yl, zl) ^ fixed) > 0.0: continue
                fm = face_margin(np.vstack([tm(s).vertices, tm(flow_board(xl, yl, zl)).vertices]))
                if fm < FACE_MIN: continue
                if vol(flow_cone(xl, yl, zl) ^ fixed) > 0.0: continue
                lever = math.hypot(xl - CG[0], yl - CG[1])
                score = vol(s) * RHO_PETG + 0.02 * lever - 0.05 * min(fm, 3.0)   # ties: more room to the collar planes
                rows.append(dict(xl=float(xl), yl=float(yl), zl=float(zl), face=round(fm, 2), g=round(vol(s) * RHO_PETG, 2),
                                 lever=round(lever, 1), score=round(score, 3)))
    rows.sort(key=lambda r: r["score"])
    log(f"flow: {len(rows)} feasible; best {rows[0] if rows else None}")
    return rows

def run_sweep(log=print):
    P_base, ifc, cart = drone_points_base()
    pr = sweep_pads(P_base, log); best_p = pr[0]
    Lp = (best_p["xp"], best_p["yp"], best_p["zp"])
    c, cc = pad_carrier(*Lp)
    fixed = U(ifc, cart, c - cc, fc_body(), plate())
    rr = sweep_radxa(fixed, log); best_r = rr[0]
    Lr = (best_r["x0"], best_r["z_top"], best_r["s"])
    sb, _ = radxa_standoffs(*Lr)
    rad = (radxa_board() + radxa_plugs()).transform(radxa_pose(*Lr)[:3])
    fixed2 = U(ifc - cc, cart, c, sb, rad)
    fr = sweep_flow(fixed2, U(cart, c, sb, rad), log); best_f = fr[0]
    L = dict(pads=Lp, radxa=Lr, flow=(best_f["xl"], best_f["yl"], best_f["zl"]))
    return L, dict(pads=pr[:12], pads_n=len(pr), radxa=rr[:12], radxa_n=len(rr), flow=fr[:12], flow_n=len(fr))

# ======================================================================== checks
def insertion_sweep(R, lid_m, male):
    """cartridge (sliders retracted, pack plug in) rising from 50 below seat to the seat: overlaps with the receiver,
    the lid and the XT60E1-M (0 = clear; the plug's receptacle takes the male's insert)."""
    worst, rows = 0.0, []
    cart_in = U(tray(), slider(False, 1), slider(False, -1))
    fem = xt60_female()
    for dy in list(np.arange(-50.0, 0.01, 1.0)) + [-0.5, -0.25]:
        a = cart_in.translate([0, dy, 0]); f = fem.translate([0, dy, 0])
        o = vol(a ^ R) + vol(a ^ lid_m) + vol(a ^ male) + vol(f ^ R) + vol(f ^ male)
        rows.append((round(float(dy), 2), round(o, 3))); worst = max(worst, o)
    return worst, rows

def float_check(R, lid_m):
    """the XT60E1-M at its float extremes (+-1.0 x, +-1.0 z, +0.5 axial) vs the receiver and the lid (0 = free)."""
    m = xt60_male(); out = {}
    for dx, dz, dy in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 0.5), (0.95, 0.95, 0.25), (-0.95, -0.95, 0.25)):
        mm = m.translate([dx * 0.98, dy * 0.98, dz * 0.98])
        out[f"dx {dx} dz {dz} dy {dy}"] = round(vol(mm ^ R) + vol(mm ^ lid_m), 3)
    return out

def bolt_checks(R):
    s1, s2 = slider(True, 1), slider(True, -1)
    retr = X_TIP - TRAVEL
    return dict(bolts_at_rest_vs_receiver_mm3=round(vol(s1 ^ R) + vol(s2 ^ R), 3),
                bolt_after_0p35_drop_bears_mm3=round(vol(s1.translate([0, -0.35, 0]) ^ R), 3),
                retracted_tip_x=round(retr, 2), retracted_clears_locating_face_by=round(RF - retr, 2),
                overlap_into_pocket=round(X_TIP - RF, 2))

def solid(mesh, drawn=()):
    import solid_check as SC
    return SC.solid_check(mesh, drawn=drawn)

def layout_parts(L):
    xp, yp, zp = L["pads"]; x0, zt, s = L["radxa"]; xl, yl, zl = L["flow"]
    M = radxa_pose(x0, zt, s)[:3]
    return dict(radxa=radxa_board().transform(M), radxa_plugs=radxa_plugs().transform(M), flow_board=flow_board(xl, yl, zl),
                pads=pads_model(xp, yp, zp))

def checks(L, R, lid_m, T):
    out = {}
    male = xt60_male()
    w, rows = insertion_sweep(R, lid_m, male)
    out["insertion_sweep_worst_mm3"] = round(w, 3); out["insertion_rows_nonzero"] = [r for r in rows if r[1] > 0][:10]
    out["float_extremes_mm3"] = float_check(R, lid_m)
    out["bolts"] = bolt_checks(R)
    eng = XM_IL - (Y_XM - Y_XF_FACE)
    out["xt60"] = dict(face_below_flange_at_seat=round(Y_XM - Y_XF_FACE, 2), engagement_at_seat=round(eng, 2),
                       noses_meet_below_seat=round(eng, 2), end_plates_locate_from_below_seat=round(Y_TOP - Y_END0 - HCAP, 2),
                       float_lateral=FLOAT, free_tilt_deg=round(math.degrees(math.atan(2 * AX_PLAY / XM_FW)), 2),
                       female_vs_male_at_seat_mm3=ov(xt60_female(), male), female_vs_tray_mm3=ov(xt60_female(), T))
    Rd, Ld = C2D(R), C2D(lid_m)
    Td = C2D(U(T, slider(True, 1), slider(True, -1)))
    lp = layout_parts(L)
    ctx = dict(FC=fc_body(), plate=plate(), xt60_male=C2D(male), xt60_female=C2D(xt60_female()), cartridge=Td, **lp)
    out["receiver_vs"] = {k: ov(Rd, v) for k, v in ctx.items()}
    out["lid_vs"] = {k: ov(Ld, v) for k, v in ctx.items() if k != "xt60_male"}
    out["radxa_vs"] = {k: ov(lp["radxa"] + lp["radxa_plugs"], v) for k, v in dict(FC=fc_body(), plate=plate(), flow=lp["flow_board"]).items()}
    ko = keepouts()
    out["keepouts"] = {k: dict(receiver=ov(Rd, v), cartridge=ov(Td, v), lid=ov(Ld, v), radxa=ov(lp["radxa"] + lp["radxa_plugs"], v),
                               flow=ov(lp["flow_board"], v)) for k, v in ko.items() if k != "FC"}
    out["cartridge_vs_K1_note"] = "the cartridge is the K1 path; K1 below the receiver must hold only the cartridge"
    out["side_boss_to_FC_corner"] = round(math.hypot(SIDE_BOSS_D[0] - FC_C[0], SIDE_BOSS_D[1] - FC_HALF * S2) - BOSS_R, 2)
    out["funnel_plane_margin"] = {k: round(face_margin(sample(m, 3.0)), 2) for k, m in
                                  dict(receiver=Rd, cartridge=Td, lid=Ld, radxa=lp["radxa"] + lp["radxa_plugs"],
                                       flow=lp["flow_board"], pads=lp["pads"]).items()}
    xp, yp, zp = L["pads"]
    c, cc = pad_carrier(xp, yp, zp)
    rest = Rd - (c - cc)
    P = np.vstack([sample(rest, 2.0), sample(Td, 2.0), sample(lp["radxa"] + lp["radxa_plugs"], 2.0), sample(lp["flow_board"], 2.0)])
    body, tips = post_model(xp, yp, zp); Pb = tm(body).vertices
    h1, g1 = cone_hits(tips, P); h2, g2 = cone_hits(Pb, P)
    out["landing_sweep"] = dict(pin_tip_hits=h1, pin_tip_gap=round(g1, 2), post_hits=h2, post_gap=round(g2, 2),
                                post_r_min=round(float(np.hypot(Pb[:, 1], Pb[:, 2] + 144.5).min()), 1))
    xl, yl, zl = L["flow"]
    out["flow_cone_vs_drone_mm3"] = ov(flow_cone(xl, yl, zl), U(rest, Td, lp["radxa"], lp["radxa_plugs"], Ld))
    return out

# ======================================================================== FE recheck (printopt, as built, layer-aware)
def fe(mesh_c, h, build, sups, cases_def, title, log=print):
    import printopt as PO, noxopt as NO
    bb = np.array(mesh_c.bounding_box()); lo = np.floor(bb[:3] - h); hi = np.ceil(bb[3:] + h)
    shape = tuple(int(np.ceil((hi[k] - lo[k]) / h)) for k in range(3))
    xs, ys, zs = [lo[k] + (np.arange(shape[k]) + 0.5) * h for k in range(3)]
    A = NO._voxelise(tm(mesh_c), xs, ys, zs).ravel().astype(bool)
    I3 = np.stack(np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), np.arange(shape[2]), indexing="ij"), -1).reshape(-1, 3)
    X = I3 * h + lo + h / 2
    ndof = 3 * (shape[0] + 1) * (shape[1] + 1) * (shape[2] + 1)
    nid = lambda i, j, k: (i * (shape[1] + 1) + j) * (shape[2] + 1) + k
    def cells(b0, b1): return A & np.all((X >= b0) & (X <= b1), axis=1)
    def nodes(mask):
        el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
    def load(b0, b1, vec):
        nd = nodes(cells(np.array(b0, float), np.array(b1, float))); F = np.zeros(ndof)
        if len(nd) == 0: raise ValueError("empty load region")
        for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
        return F
    S = {k: (nodes(cells(np.array(a, float), np.array(b, float)))[:, None] * 3 + np.arange(3)).ravel() for k, (a, b) in sups.items()}
    cases = [PO.LoadCase(n, load(b0, b1, v), cls=c, support=s) for (n, b0, b1, v, c, s) in cases_def]
    pb = PO.Problem(shape=shape, h=h, origin=tuple(lo), solid=A.copy(), void=~A, supports=S, cases=cases,
                    material=PO.MATERIALS["PETG"], build=build, overhang=False, R=0.5, comp_factor=None)
    po = PO.PrintOpt(pb); rp = po.physical(A.astype(float), 64.0)
    Eel = po.EMIN + rp ** po.PEN * (1 - po.EMIN); K = po.assemble(Eel)
    groups = {}
    for i, lc in enumerate(pb.cases): groups.setdefault(lc.support, []).append(i)
    rows = []
    log(f"FE {title}: grid {shape} h {h}, {int(A.sum()):,} solid cells, build {build}")
    for sup, idx in groups.items():
        Fs = [po.force(i, rp) for i in idx]
        sol = po.solve(Eel, {sup: np.stack(Fs, 1)}, K=K)[sup]; Uu = sol[0]
        for j, i in enumerate(idx):
            lc = pb.cases[i]; ue = Uu[:, j][po.edof]; st = po.stresses(ue, rp); Al = PO.allowables(pb.material, lc.cls)
            f1, _, f2, _ = po.failure(st, Al); sm = po.smask
            r = dict(case=lc.name, cls=lc.cls, inlayer=round(float(f1[sm].max()), 2), across=round(float(f2[sm].max()), 2),
                     at_inlayer=np.round(po.ecent[sm][np.argmax(f1[sm])], 0).tolist(), at_across=np.round(po.ecent[sm][np.argmax(f2[sm])], 0).tolist(),
                     max_disp_mm=round(float(np.abs(Uu[:, j]).max()), 3))
            rows.append(r); log(f"  {r['case']:46s} [{r['cls']:9s}] in-layer {r['inlayer']:.2f} at {r['at_inlayer']}  across {r['across']:.2f} at {r['at_across']}  u {r['max_disp_mm']}")
        try: sol[1].free_memory(everything=True)
        except Exception: pass
    return rows

def fe_all(L, R, T, log=print):
    big = 1e3
    m_c = 0.165 + (vol(T) + 2 * vol(slider())) * RHO_PETG / 1000      # pack (P) + tray + sliders (M), kg
    crash = m_c * 9.81 * 35.0
    pkz = (Z_TAB - BAR_HW - PK_ZC, Z_TAB + BAR_HW + PK_ZC)
    xp, yp, zp = L["pads"]; x0, zt, s = L["radxa"]; xl, yl, zl = L["flow"]
    car = d2c([[xp - CAR_HALF, Y_IN, zp], [xp + CAR_HALF, yp + 7.5, zp + 1.3]]); c0, c1 = car.min(0), car.max(0)
    rb = d2c([[x0, -(SP1 + s) - 0.1, zt - RX_W], [x0 + RX_L, -(SP1 + s) + 1.3, zt]]); r0, r1 = rb.min(0), rb.max(0)
    fb = d2c([[xl - 18.3, Y_IN, zl + FL_T + 0.5], [xl + 8.3, yl + 12.3, zl + FL_T + 2.0]]); f0, f1 = fb.min(0), fb.max(0)
    sups = {"plate bolts": ((-big, Y_PLATE - 1.5, -big), (big, big, big))}
    cases = [
        ("XT60 insert 60 N (lid bosses, up)", (-31, Y_DK1 - 2.0, Z_X - 6), (31, Y_DK1 + 0.1, Z_X + 6), (0, 60.0, 0), "flight", "plate bolts"),
        ("XT60 pull-out 50 N (lips, down)", (-15, LIP_Y0 - 0.1, Z_X - 9), (15, LIP_Y0 + 1.3, Z_X + 9), (0, -50.0, 0), "flight", "plate bolts"),
        ("head stall 75 N on the strip seats (up)", (-21, Y_DK0 - 0.1, 1), (21, Y_DK0 + 1.3, 72), (0, 75.0, 0), "misuse", "plate bolts"),
        (f"crash 35 g pull-out {crash:.0f} N (pocket floors)", (-27, PK_Y0 - 1.5, pkz[0]), (27, PK_Y0 + 0.3, pkz[1]), (0, -crash, 0), "crash", "plate bolts"),
        (f"crash 35 g side {crash:.0f} N (+x face)", (RF - 0.1, 2, -5), (RF + 1.3, 50, 97), (crash, 0, 0), "crash", "plate bolts"),
        (f"crash 35 g tail-ward {crash:.0f} N (lead-end plate)", (-25, 38, Z_E1 - 0.1), (25, Y_DK1, Z_E1 + 1.3), (0, 0, crash), "crash", "plate bolts"),
        ("capture 60 N push on a 30 deg foot", (RF - 0.1, 2, 2), (FOOT_X, 9.5, 14), (35.0, 60.0, 0), "misuse", "plate bolts"),
        ("pogo 4 pins + KW11 12 N (carrier, up)", tuple(c0), tuple(c1), (0, 12.0, 0), "sustained", "plate bolts"),
        ("Radxa 15 g at 35 g + 5 N (standoffs, off)", tuple(r0), tuple(r1), (10.0, -5.0, 0), "crash", "plate bolts"),
        ("flow seat 10 N down + 10 N out", tuple(f0), tuple(f1), (-10.0, -10.0, 0), "flight", "plate bolts"),
    ]
    rr = fe(R, 1.25, "-z", sups, cases, "receiver v4", log)
    tsup = {"channel floors (fingers pull)": ((-X_OUT - 1, CH_DROP_Y - 1.5, Z_TAB - CH_HW), (X_OUT + 1, CH_Y0 + 0.01, Z_TAB + CH_HW)),
            "flat bottom rim (head pushes)": ((-big, -1, -big), (big, 1.3, big)),
            "channel ceilings (bolts hold)": ((-X_OUT - 1, CH_Y1 - 0.01, Z_TAB - CH_HW), (X_OUT + 1, CH_Y1 + 1.3, Z_TAB + CH_HW))}
    px = XF_W / 2 + P_CLR
    tcases = [
        ("XT60 insert 60 N on the plug shelf (down)", (-px, Y_SH1 - 1.3, Z_X - 4.2), (px, Y_SH1 + 0.1, Z_X + 4.2), (0, -60.0, 0), "flight", "flat bottom rim (head pushes)"),
        ("XT60 pull-out 50 N at the plug (up)", (-px - 2, Y_SH1, Z_CW1 - 0.5), (px + 2, Y_PW1, Z_EW0 + 3.5), (0, 50.0, 0), "flight", "channel floors (fingers pull)"),
        (f"crash 35 g pack on the floor {crash:.0f} N", (-X_IN, Y_FL1 - 1.3, Z_P0), (X_IN, Y_FL1 + 0.1, Z_P1), (0, -crash, 0), "crash", "channel ceilings (bolts hold)"),
        (f"crash 35 g pack on the end stop {crash:.0f} N", (-X_IN, Y_FL1, Z_P1 - 0.1), (X_IN, Y_FL1 + 5.0, Z_P1 + 1.3), (0, 0, crash), "crash", "channel ceilings (bolts hold)"),
        ("pinch 20 N across the long walls", (X_OUT - 1.3, 20, 30), (X_OUT + 0.1, 40, 46), (-20.0, 0, 0), "misuse", "channel ceilings (bolts hold)"),
    ]
    tr = fe(T, 1.0, "+z", tsup, tcases, "cartridge tray v4", log)
    return dict(receiver=rr, tray=tr, crash_N=round(crash, 1))

# ======================================================================== print poses (4x4 on c coords)
P_RECEIVER = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, Z_E1 + END_T], [0, 0, 0, 1.0]])   # lead-end face on the bed
P_TRAY = np.eye(4)                                                                                 # closed end on the bed
P_SLIDER = np.array([[0, 0, -1, Z_BLK1], [0, 1, 0, 0], [1, 0, 0, -TAB_IN], [0, 0, 0, 1.0]])      # inner tab face on the bed
P_LID = np.array([[1, 0, 0, 0], [0, 0, 1, -Z_X], [0, -1, 0, Y_DK1 + LID_T], [0, 0, 0, 1.0]])   # top face on the bed
P_STRIP = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])                 # flat

def export(m, M, path):
    t = tm(m, M); t.apply_translation([0, 0, -t.bounds[0][2]]); t.export(path); return t

# ======================================================================== motion artifact
def motion(L, R, lid_m, T, path_html, path_mp4, log=print):
    from motion import Scene
    sc = Scene("Scout v3 bottom: cartridge v4 into receiver v4", "head pushes the cartridge up: lead-ins, XT60 mates, bolts snap into the pockets")
    D = T4(T_DC)
    lp = layout_parts(L)
    xp, yp, zp = L["pads"]
    body, _ = post_model(xp, yp, zp)
    sc.part("receiver v4", tm(R, D), group="drone", color="#8899aa")
    sc.part("XT60 lid", tm(lid_m, D), group="drone", color="#7788aa")
    sc.part("XT60E1-M", tm(xt60_male(), D), group="drone", color="#d4b13a", printed=False)
    sc.part("FC", tm(fc_body()), group="drone", color="#2d5a2d", printed=False)
    sc.part("plate", tm(plate()), group="drone", color="#333333", printed=False)
    sc.part("Radxa", tm(lp["radxa"]), group="drone", color="#2d5a2d", printed=False)
    sc.part("MTF-02P", tm(lp["flow_board"]), group="drone", color="#2d5a2d", printed=False)
    sc.part("pogo pads", tm(lp["pads"]), group="drone", color="#c87533", printed=False)
    sc.part("belly post (box, E)", tm(body), group="box", color="#666666", printed=False)
    sc.part("cartridge v4", tm(T, D), group="cartridge", color="#cc6644")
    sc.part("slider +", tm(slider(True, 1), D), group="cartridge", color="#4477cc")
    sc.part("slider -", tm(slider(True, -1), D), group="cartridge", color="#4477cc")
    sc.part("pack XT60", tm(xt60_female(), D), group="cartridge", color="#d4b13a", printed=False)
    pack = box(-PACK[0] / 2, PACK[0] / 2, Y_FL1, Y_FL1 + PACK[1], 2.5, 2.5 + PACK[2])
    sc.part("pack 4S 1500", tm(pack, D), group="cartridge", color="#444466", printed=False)
    head = box(-19.0, 19.0, -12.0, -0.1, -2.0, L_C + 2.0)
    sc.part("head (box, E)", tm(head, D), group="box", color="#999999", printed=False)
    dur = 10.0
    def ease(u): u = min(max(u, 0.0), 1.0); return u * u * (3 - 2 * u)
    def dy_of(t):
        if t < 1.0: return -60.0
        if t < 5.0: return -60.0 + 48.0 * ease((t - 1.0) / 4.0)
        if t < 7.5: return -12.0 + 12.0 * ease((t - 5.0) / 2.5)      # last 12 slow (dimensions I: <= 20 mm/s)
        return 0.0
    def sl_of(t):                                                     # sliders squeezed until the seat, then spring out
        return -TRAVEL if t < 7.8 else -TRAVEL + TRAVEL * ease((t - 7.8) / 0.4)
    def poses(t):
        dz = dy_of(t); ds = sl_of(t)
        P = {k: (0, 0, dz, 0, 0, 0) for k in ("cartridge v4", "pack XT60", "pack 4S 1500", "head (box, E)")}
        P["slider +"] = (0, -ds, dz, 0, 0, 0)                          # x_c + side = drone -y: retract = +y
        P["slider -"] = (0, ds, dz, 0, 0, 0)
        return P
    sc.timeline(poses, dur, fps=30)
    ts = [i * dur / 300 for i in range(300)]
    eng = XM_IL - (Y_XM - Y_XF_FACE)
    sc.series("XT60 engagement", [max(0.0, eng + dy_of(t)) for t in ts], unit="mm")
    sc.series("cartridge below seat", [-dy_of(t) for t in ts], unit="mm")
    sc.view("side", (0, -260, -20), (0, 0, -20)); sc.view("tail", (-260, 0, -10), (-40, 0, -10))
    sc.write(path_html)
    try: sc.video(path_mp4, view="iso")
    except Exception as e: log(f"video skipped: {e}")

# ======================================================================== main
def main():
    t0 = time.time(); log = lambda s: print(s, flush=True)
    if "--motion-only" in sys.argv:                                          # reuse the saved layout (no sweep)
        L = {k: tuple(v) for k, v in json.load(open(os.path.join(OUT, "result.json")))["layout"].items()}
        motion(L, receiver(L), lid(), tray(), os.path.join(OUT, "motion.html"), os.path.join(OUT, "motion.mp4"), log); return
    L, SW = run_sweep(log)
    R = receiver(L); T = tray(); lid_m = lid(); sl = slider(True, 1); st = tpu_strip()
    log(f"layout {L}")
    res = dict(layout=L, sweep=SW)
    res["mass_g"] = dict(receiver=round(vol(R) * RHO_PETG, 1), lid=round(vol(lid_m) * RHO_PETG, 1), tray=round(vol(T) * RHO_PETG, 1),
                         slider=round(vol(sl) * RHO_PETG, 2), tpu_strip=round(vol(st) * RHO_TPU, 2))
    res["mass_g"]["cartridge_total"] = round(res["mass_g"]["tray"] + 2 * res["mass_g"]["slider"], 1)
    res["mass_g"]["drone_side_total"] = round(res["mass_g"]["receiver"] + res["mass_g"]["lid"] + 2 * res["mass_g"]["tpu_strip"], 1)
    log(f"mass {res['mass_g']}")
    res["checks"] = checks(L, R, lid_m, T)
    log(json.dumps(res["checks"], default=float)[:3000])
    files = {}
    for name, m, M, mat in (("scout_receiver_v4", R, P_RECEIVER, "PETG"), ("scout_receiver_lid_v4", lid_m, P_LID, "PETG"),
                            ("scout_cartridge_v4", T, P_TRAY, "PETG"), ("scout_cartridge_slider_v4", sl, P_SLIDER, "PETG"),
                            ("scout_receiver_strip_v4", st, P_STRIP, "TPU")):
        p = os.path.join(PRINT, name + ".stl"); t = export(m, M, p)
        ok, info = solid(t)
        files[name] = dict(file=f"print/{name}.stl", material=mat, solid_check="PASS" if ok else "FAIL", info=info,
                           extents=np.round(t.extents, 1).tolist(), watertight=bool(t.is_watertight))
        log(f"{name}: solid_check {'PASS' if ok else 'FAIL'} {info}")
    res["files"] = files
    # design-pose copies for the bay worker and the assembly (c coords / d coords)
    tm(R).export(os.path.join(OUT, "receiver_v4_cart_coords.stl")); tm(T).export(os.path.join(OUT, "cartridge_v4_cart_coords.stl"))
    tm(interface_parts() - interface_cuts() - cart_path()).export(os.path.join(OUT, "interface_v4_cart_coords.stl"))
    if "--fe" in sys.argv:
        res["fe"] = fe_all(L, R, T, log)
    if "--motion" in sys.argv:
        motion(L, R, lid_m, T, os.path.join(OUT, "motion.html"), os.path.join(OUT, "motion.mp4"), log)
    res["wall_s"] = round(time.time() - t0, 1)
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))
    log(f"done {res['wall_s']} s")

if __name__ == "__main__":
    main()
