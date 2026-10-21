"""S2 cartridge v3 (fitted, flat bottom, guided): tray + latch block with SUNKEN tabs, strip pads, drone receiver.
15 Oct 01:17, Spark: the grip notches must be sunken in; the whole unit a rectangle, or at least a flat bottom.
16 Oct 17:54, Spark: centering and error correction; functional surfaces are drawn, topology only between them.
Latch = v2/s2-bottom-latch.md (squeeze-release sliders), contacts = v2/s2-contacts.md (copper-clad strips + sprung copper shoes on AA springs, 16 Oct 20:08).

Cartridge coords: x = slide axis (0 at centre, across the pack width), y = up toward the drone (0 = flat bottom face),
z = along the pack length (0 = closed-end outer face). Drone-side parts in the same coords, seated at bearing.
All mm. Tags: P pack, C CAD, E estimate. Output: STLs in ./out, each in its print orientation.

Parts
  tray   : PETG box round the pack: 36.2 x 48 x 78, flat bottom, open top (pack snaps in under lips), pad bosses,
           strip recesses, solder windows; prints standing on its closed-end wall (z up), no supports.
  block  : latch block 36.2 x 14 x 14 glued into the tray's bottom band (its top is the tray floor). Both sliders
           print in place, flat (z = print Z). The two tabs sit in one recess open to the bottom face (29 x 14, 7.65
           deep); 3 mm finger blades reach up beside the tabs and squeeze them in 4.0 (bolts retract). Return = one
           Ø3 pen-type coil spring between the tabs (DEFAULT; seats are Ø3.2 blind holes in the tab inner faces).
  pocket : drone pocket block, bare (pocket + 3.0 walls). Print 2, same part both sides (turn 180 deg about y).
  shoe   : drone shoe module (lip, chamber, AA plate slot) + printed carrier; copper-clad shoe glued on. 2 per drone.
"""
import os, math
import numpy as np
import trimesh
from trimesh.creation import box

OUT = "out"; os.makedirs(OUT, exist_ok=True)

# ---------------- pack and tray ----------------
PACK_X, PACK_Y, PACK_Z = 33.0, 32.0, 70.0   # P (70 x 32 x 33; 32 taken as the height, DEFAULT)
CLR = 0.5                                   # pack clearance per side (E); 0.5 puts every wall face on a whole mm (grid mirrors)
CAV_X = PACK_X / 2 + CLR                    # 17.0
WALL = 2.0                                  # wall plate thickness (topology picks the holes in its plane)
XW_OUT = CAV_X + WALL                       # 19.0
END_W = 2.0
LEAD_ROOM = 6.0                             # E
Z_PACK0 = END_W                             # 1.2
Z_PACK1 = Z_PACK0 + PACK_Z + 2 * CLR        # 72.0
Z_LEN = Z_PACK1 + LEAD_ROOM                 # 78.0
Z_C = Z_PACK0 + CLR + PACK_Z / 2            # 36.6 pack centre = bolt plane

# ---------------- latch block (block-local y 0 = flat bottom; same y in the tray) ----------------
T = 14.0                   # block thickness in z = print height of the flat print
CL = 0.35
TRAVEL = 4.0
UNDER = math.radians(10)
TAPER = math.radians(5)
Y_BLK1 = 14.0              # block top = tray floor top
BAR_Y0, BAR_Y1 = 8.0, 12.0
BAR_Z0, BAR_Z1 = 0.0, T - 1.0               # bar on the plate, 0.5 gap, 0.5 skin
CH_Y0, CH_Y1 = BAR_Y0 - CL, BAR_Y1 + CL     # 7.65 .. 12.35 channel
TAB_IN, TAB_OUT = 4.0, 10.0                 # tab faces at rest (0 and 6 squeezed: the tabs meet at x 0 = the stop)
REC_X = 14.5                                # recess half width: open finger blade (x 11..14) + 0.5
FINGER_T = 3.0                              # finger blade thickness (box side, E)
POCK_YC, POCK_H, POCK_W, POCK_D = 4.0, 4.0, 8.0, 3.0   # grip pocket on the tab outer face
SPR_D, SPR_Y, SPR_DEPTH = 3.2, 9.0, 4.0     # spring seat holes (Ø3 pen spring, free length ~17, DEFAULT)
RAIL_FACE = XW_OUT + 1.0 + 0.3              # 20.3: receiver locating face, 0.3 off the 1.0 guide rail
OVER = 3.0
X_TIP = RAIL_FACE + OVER                    # 22.4
Z_BLK0 = Z_C - T / 2                        # 29.6
Z_BLK1 = Z_C + T / 2                        # 43.6
BLK_CLR = 0.1

# ---------------- tray bands ----------------
Y_FLOOR0, Y_FLOOR1 = Y_BLK1 - 2.0, Y_BLK1   # 12 .. 14 tray floor (2.0)
Y_CAV1 = Y_FLOOR1 + 33.2                    # 47.2: fits the pack either way up (32 or 33 tall)
Y_TOP = Y_CAV1 + 0.8                        # 48.0
LIP = 0.6
RAIL_TOP = 3.0
POST = 3.0
FLOOR_STR = 5.0
CROSS = 3.0
BOT_T = 2.0                                 # flat bottom plate (lower band), windowed, rim 4
BOT_RIM = 4.0

# ---------------- pads and sprung shoes (16 Oct 20:08, Spark: no new parts; only the copper-clad board + AA springs) ----
# Current never passes through the steel spring: copper strip (cartridge) on copper shoe (drone), each with its lead
# soldered on the copper face through a hole and filed flat. The spring only pushes the shoe's printed carrier.
BOSS = 1.0
BOSS_X1 = XW_OUT + BOSS                     # 20.0 guide rail face
CU_T = 1.6                                  # copper-clad: MCIGICM FR-4, single sided, 70 x 100 x 1.6 (P listing B01MCVLDDZ)
STRIP_PROUD = 0.2                           # strip face proud of the rail: the flat shoe lands on copper, never on PETG
STRIP_H, STRIP_L = 4.0, 16.0
STRIP_REC = CU_T - STRIP_PROUD + 0.1        # 1.5 deep incl. 0.1 glue
CHAMF_Y, CHAMF_X = 5.0, 2.0                 # rail top lead-in for the shoes (21.8 deg from vertical), full length
RAIL_Y0 = 38.0                              # guide rail y 38..48 full length: the cartridge's x datum (receiver + bay)
Y_BAND1 = Y_TOP - CHAMF_Y                   # 43.0
Y_BAND0 = Y_BAND1 - STRIP_H                 # 39.0
Y_SHOE = (Y_BAND0 + Y_BAND1) / 2            # 41.0
X_STRIP_FACE = BOSS_X1 + STRIP_PROUD        # 20.2
# shoe module: z between the latch pocket block (wall ends 47.3) and the end post (68); same for the bay
POST_W = 8.0                                # receiver post width (z), posts at z 3..11 and Z_LEN-11..Z_LEN-3
MOD_Z0, MOD_Z1 = Z_BLK0 + (BAR_Z1 - BAR_Z0) / 2 + (BAR_Z1 - BAR_Z0) / 2 + 0.8 + 3.0, Z_LEN - 3.0 - POST_W   # 47.3, 68.0
MOD_WZ = 1.2                                # chamber end walls (z)
ZM = (MOD_Z0 + MOD_Z1) / 2                  # 57.65 module / strip centre
SH_CLR = 0.3                                # carrier clearance per side (print in place)
CH_Z = MOD_Z1 - MOD_Z0 - 2 * MOD_WZ         # 18.3 chamber length
FL_Z = CH_Z - 2 * SH_CLR                    # 17.7 carrier flange
MOUTH_Z = FL_Z - 2.0                        # 15.7 (lip overlaps the flange 1.0 per side)
SHOE_Z = MOUTH_Z - 2 * SH_CLR               # 15.1 shoe = carrier boss
SHOE_H = 3.6                                # shoe height (y), inside the 4.0 strip (clear of the ramp)
MOUTH_H = 5.1                               # mouth height: boss 3.6 + 0.75 per side (lip still overlaps the flange 1.45)
# Spring = AA negative leaf contact as bought (Amazon B0GTVNBSB5, = MPD BK-209 drawing): 0.3 (0.012") nickel-plated spring
# steel, 7.92 wide x 11.40 tall, 6.35 deep free (back of the mounting leg to the leaf crest), mounting leg 1.57 over its
# lanced tab (P listing). Turned so its 11.4 runs along z; the leg drops into a slot in the back wall from the deck side.
SPR_FREE, SPR_SEAT = 6.35, 3.85             # free depth (P listing); seated depth = 2.5 deflection (E: AA contacts work ~2 to 3)
SPR_W = 7.92                                # leaf width, along y here (P listing)
PLATE_W, PLATE_H, PLATE_T = 11.40, SPR_W, 1.57   # mounting leg: along z, along y, thickness over the tab (P listing)
SPR_BASE = SPR_W                            # what the chamber must clear in y
FL_H = max(MOUTH_H + 2.0, SPR_BASE)         # 8.0 carrier flange height (y)
CH_H = FL_H + 2 * SH_CLR                    # 8.6 chamber height
COMP = 1.5                                  # shoe stroke onto the strip (free face 1.5 inside the strip face)
X_MOD0 = RAIL_FACE + 0.5                    # 20.8 module front: 0.5 behind the locating plane, clear of a proud strip at +0.3 shift
LIP_T, FL_T = 1.5, 2.0
X_LIP1 = X_MOD0 + LIP_T                     # 22.3 lip back = flange front at free
X_SHOE_FREE = X_STRIP_FACE - COMP           # 18.7 shoe face, no cartridge
X_PLATE = X_LIP1 + COMP + FL_T + SPR_SEAT   # 29.65 front face of the spring's mounting leg
X_MOD1 = X_PLATE + PLATE_T + 0.2 + 1.5      # 33.0 back wall outer face
WIRE_DZ, WIRE_R = 6.3, 1.15                 # 16 AWG lead: through the shoe and the carrier's notch at z +6.3 (insulated 2.2)
WIRE_BACK_DZ = 7.6                          # ...and out the back wall at z +7.6, outside the spring leg (±5.7) and its slot
WIRE_STRIP_R = 0.7                          # hole in the copper for the stripped conductor (1.3)
Z_STRIP0, Z_STRIP1 = ZM - STRIP_L / 2, ZM + STRIP_L / 2      # 49.65 .. 65.65
Z_BOSS0 = Z_STRIP0 - 1.5
WIN_Z0, WIN_Z1, WIN_Y0, WIN_Y1 = ZM + 3.5, ZM + 6.5, Y_SHOE - 1.2, Y_SHOE + 1.2   # strip lead hole through the wall


def B(x0, x1, y0, y1, z0, z1):
    m = box(extents=[x1 - x0, y1 - y0, z1 - z0])
    m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return m

def U(*ms):
    r = ms[0]
    for m in ms[1:]:
        r = r.union(m, engine="manifold")
    return r

def D(a, b):
    return a.difference(b, engine="manifold")

def hull(pts):
    return trimesh.convex.convex_hull(np.array(pts, float))

def prism(xy, z0, z1):
    return hull([[x, y, z] for x, y in xy for z in (z0, z1)])

def mirror_x(m):
    r = m.copy(); r.apply_transform(trimesh.transformations.reflection_matrix([0, 0, 0], [1, 0, 0])); return r

def cyl_x(x0, x1, yc, zc, r):
    c = trimesh.creation.cylinder(radius=r, height=x1 - x0, sections=32)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    c.apply_translation([(x0 + x1) / 2, yc, zc]); return c

def frustum_x(x0, x1, yc, zc, h0, w0, h1, w1):
    p = []
    for x, h, w in ((x0, h0, w0), (x1, h1, w1)):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p.append([x, yc + sy * h / 2, zc + sz * w / 2])
    return hull(p)


# =============== latch block (block-local z 0..T; tray z = Z_BLK0 + z) ===============
def block(parts=False):
    frame = B(-XW_OUT, XW_OUT, 0, Y_BLK1, 0, T)
    frame = D(frame, B(-REC_X, REC_X, -1, CH_Y1, -1, T + 1))                  # recess open to the bottom, up to the ceiling skin
    frame = D(frame, B(-XW_OUT - 1, XW_OUT + 1, CH_Y0, CH_Y1, -1, BAR_Z1 + 0.5))   # channel through both walls
    for s in (-1, 1):                                                         # channel floor 0.5 deeper outboard of x 15.3: the
        xa, xb = sorted((s * (REC_X + 0.8), s * (XW_OUT + 1)))                   # retracted wedge tip (drops 0.48) sinks into it
        frame = D(frame, B(xa, xb, CH_Y0 - 0.5, CH_Y0 + 0.01, -1, BAR_Z1 + 0.5))
    for s in (-1, 1):                                                         # 0.5 skin over the bar outboard of the recess
        xa, xb = sorted((s * (REC_X - 0.01), s * XW_OUT))
        frame = U(frame, B(xa, xb, CH_Y0 - 0.01, CH_Y1 + 0.01, T - 0.5, T))
    # 45 deg lead-in on the recess mouth, both long edges (finger self-centres), 1.0
    for s in (-1, 1):
        frame = D(frame, prism([(s * (REC_X - 0.01), -0.01), (s * (REC_X + 1.0), -0.01), (s * (REC_X - 0.01), 1.0)], -1, T + 1))

    def slider(s):
        bar = B(TAB_IN, X_TIP, BAR_Y0, BAR_Y1, BAR_Z0, BAR_Z1)
        wedge = hull([[RAIL_FACE, BAR_Y0, BAR_Z0], [RAIL_FACE, BAR_Y0, BAR_Z1],
                      [X_TIP, BAR_Y0, BAR_Z0], [X_TIP, BAR_Y0, BAR_Z1],
                      [X_TIP, BAR_Y0 - OVER * math.tan(UNDER), BAR_Z0],
                      [X_TIP, BAR_Y0 - OVER * math.tan(UNDER), BAR_Z1]])
        cham = hull([[X_TIP - 1, BAR_Y1 + 0.01, BAR_Z0 - 1], [X_TIP - 1, BAR_Y1 + 0.01, BAR_Z1 + 1],
                     [X_TIP + 0.01, BAR_Y1 - 1, BAR_Z0 - 1], [X_TIP + 0.01, BAR_Y1 - 1, BAR_Z1 + 1],
                     [X_TIP + 0.01, BAR_Y1 + 0.01, BAR_Z0 - 1], [X_TIP + 0.01, BAR_Y1 + 0.01, BAR_Z1 + 1]])
        tab = B(TAB_IN, TAB_OUT, 0.0, BAR_Y1, 0, T)
        body = D(U(bar, wedge, tab), cham)
        pocket = frustum_x(TAB_OUT + 0.01, TAB_OUT - POCK_D, POCK_YC, T / 2, POCK_H, POCK_W,
                           POCK_H - 2 * POCK_D * math.tan(TAPER), POCK_W - 2 * POCK_D * math.tan(TAPER))
        body = D(body, pocket)
        body = D(body, cyl_x(TAB_IN - 0.01, TAB_IN + SPR_DEPTH, SPR_Y, T / 2, SPR_D / 2))   # spring seat
        return mirror_x(body) if s < 0 else body

    if parts:
        return frame, slider(1), slider(-1)
    return U(frame, slider(1), slider(-1))


# =============== tray ===============
def tray():
    parts = []
    # floor ladder y 12.35..14 (block top is flush at 14)
    parts += [B(-CAV_X, -CAV_X + FLOOR_STR, Y_FLOOR0, Y_FLOOR1, 0, Z_LEN),
              B(CAV_X - FLOOR_STR, CAV_X, Y_FLOOR0, Y_FLOOR1, 0, Z_LEN),
              B(-CAV_X, CAV_X, Y_FLOOR0, Y_FLOOR1, 0, END_W + CROSS),
              B(-CAV_X, CAV_X, Y_FLOOR0, Y_FLOOR1, Z_BLK0 - BLK_CLR - CROSS, Z_BLK0 - BLK_CLR),
              B(-CAV_X, CAV_X, Y_FLOOR0, Y_FLOOR1, Z_BLK1 + BLK_CLR, Z_BLK1 + BLK_CLR + CROSS),
              B(-CAV_X, CAV_X, Y_FLOOR0, Y_FLOOR1, Z_PACK1 - CROSS, Z_PACK1 + 2.0)]
    # flat bottom plate y 0..1.2, rim 4, windowed; the block's own bottom fills z 29.5..43.7
    for za, zb in ((0, Z_BLK0 - BLK_CLR), (Z_BLK1 + BLK_CLR, Z_LEN)):
        p = B(-XW_OUT, XW_OUT, 0, BOT_T, za, zb)
        p = D(p, B(-XW_OUT + BOT_RIM, XW_OUT - BOT_RIM, -1, BOT_T + 1, za + BOT_RIM, zb - BOT_RIM))
        parts.append(p)
    parts.append(B(-CAV_X, CAV_X, Y_FLOOR1, Y_FLOOR1 + 4.0, Z_PACK1, Z_PACK1 + 2.0))       # pack end stop (bridged)
    end = D(B(-XW_OUT, XW_OUT, 0, Y_TOP, 0, END_W),
            B(-CAV_X + 3, CAV_X - 3, Y_FLOOR1 + 3, Y_TOP - 3, -1, END_W + 1))
    end = D(end, B(-CAV_X + 3, CAV_X - 3, BOT_T + 2, Y_FLOOR0 - 2, -1, END_W + 1))         # lower-band window
    parts.append(end)
    for s in (-1, 1):
        def X(a, b):
            return (min(s * a, s * b), max(s * a, s * b))
        xi, xo = X(CAV_X, XW_OUT)
        wall = B(xi, xo, 0, Y_TOP, 0, Z_LEN)
        # windows above the floor (between posts, under the top rail) and in the lower band
        for za, zb, ya, yb in ((END_W + POST, Z_BLK0 - BLK_CLR - POST, Y_FLOOR1 + 3, Y_TOP - RAIL_TOP),
                               (Z_BLK1 + BLK_CLR + POST, Z_BOSS0 - POST, Y_FLOOR1 + 3, Y_TOP - RAIL_TOP),
                               (Z_BOSS0, Z_LEN - POST, Y_FLOOR1 + 3, Y_BAND0 - 3.0),
                               (END_W + POST, Z_BLK0 - BLK_CLR - POST, BOT_T + 2, Y_FLOOR0 - 2),
                               (Z_BLK1 + BLK_CLR + POST, Z_LEN - POST, BOT_T + 2, Y_FLOOR0 - 2)):
            if zb - za > 1.0 and yb - ya > 1.0:
                wall = D(wall, B(xi - 1, xo + 1, ya, yb, za, zb))
        # block notch: y 0..14 over the block z range (walls above bear on the block top: pull-out is bearing)
        wall = D(wall, B(xi - 1, xo + 1, -1, Y_FLOOR1, Z_BLK0 - BLK_CLR, Z_BLK1 + BLK_CLR))
        lip = prism([(s * CAV_X, Y_CAV1 - LIP), (s * (CAV_X - LIP), Y_CAV1), (s * (CAV_X - LIP), Y_TOP),
                     (s * CAV_X, Y_TOP)], END_W + POST, Z_BOSS0 - 2)
        bx0, bx1 = X(XW_OUT - 0.01, BOSS_X1)
        boss = B(bx0, bx1, RAIL_Y0, Y_TOP, 0, Z_LEN)                         # guide rail, full length
        cham = prism([(s * (BOSS_X1 - CHAMF_X), Y_TOP + 0.01), (s * (BOSS_X1 + 1), Y_TOP + 0.01),
                      (s * (BOSS_X1 + 1), Y_TOP - CHAMF_Y)], -1, Z_LEN + 1)
        wall = D(U(wall, lip, boss), cham)
        ba, bb = X(CAV_X, BOSS_X1)                                           # solid backing behind the strip
        wall = U(wall, B(ba, bb, Y_BAND0 - 1.0, Y_BAND1 + 0.5, Z_STRIP0 - 1.5, Z_STRIP1 + 1.5))
        wall = D(wall, cham)
        wall = U(wall, prism([(s * (BOSS_X1 - 1.5), Y_BAND1), (s * X_STRIP_FACE, Y_BAND1),         # ramp: chamfer -> proud strip,
                              (s * (BOSS_X1 - 0.8), Y_BAND1 + 2.0)], Z_STRIP0, Z_STRIP1))         # no step for the shoe edge
        ra, rb = X(BOSS_X1 - STRIP_REC, BOSS_X1 + 1)
        wall = D(wall, B(ra, rb, Y_BAND0, Y_BAND1, Z_STRIP0, Z_STRIP1))
        wa, wb = X(CAV_X - 1, BOSS_X1 - STRIP_REC + 0.01)                     # lead hole from the cavity to the strip
        wall = D(wall, B(wa, wb, WIN_Y0, WIN_Y1, WIN_Z0, WIN_Z1))
        parts.append(wall)
    t = U(*parts)
    return D(t, B(-XW_OUT - 1, XW_OUT + 1, -1, Y_FLOOR1, Z_BLK0 - BLK_CLR, Z_BLK1 + BLK_CLR))   # the block's place


# =============== drone pocket block (bare) ===============
POCK_Z_CLR, PK_WALL = 0.8, 3.0             # pocket 14.6 wide (bolt 13): the receiver locates z, the pocket only holds
def pocket_block():
    zc = Z_BLK0 + (BAR_Z0 + BAR_Z1) / 2
    zh = (BAR_Z1 - BAR_Z0) / 2 + POCK_Z_CLR
    x0, x1 = RAIL_FACE, RAIL_FACE + OVER + 0.3
    yc = BAR_Y1 + 0.3
    yf0 = BAR_Y0 - 0.25
    yf1 = BAR_Y0 - (x1 - x0) * math.tan(UNDER) - 0.25
    ybot, ytop = yf1 - PK_WALL, yc + PK_WALL
    body = B(x0, x1 + PK_WALL, ybot, ytop, zc - zh - PK_WALL, zc + zh + PK_WALL)
    pk = hull([[x0 - 0.01, yf0, zc - zh], [x0 - 0.01, yf0, zc + zh], [x0 - 0.01, yc, zc - zh], [x0 - 0.01, yc, zc + zh],
               [x1, yf1, zc - zh], [x1, yf1, zc + zh], [x1, yc, zc - zh], [x1, yc, zc + zh]])
    ch = hull([[x0 - 0.01, yc + 1.0, zc - zh], [x0 - 0.01, yc + 1.0, zc + zh],
               [x0 - 0.01, yc - 0.01, zc - zh], [x0 - 0.01, yc - 0.01, zc + zh],
               [x0 + 1.0, yc - 0.01, zc - zh], [x0 + 1.0, yc - 0.01, zc + zh]])
    global _POCKET_CUT
    _POCKET_CUT = U(pk, ch)
    return D(body, _POCKET_CUT), dict(mouth=(yf0, yc), floor_back=yf1, z=(zc - zh, zc + zh), x=(x0, x1 + PK_WALL), y=(ybot, ytop))


# =============== drone shoe module (+x side; mirror for B-) ===============
# Order, outboard: mouth lip (x 20.8..22.3) | chamber: carrier flange + spring | plate slot (x 29.3..30.0, open to the
# top: the AA plate drops in from the deck side, epoxy dab) | back wall. The carrier prints in place in the chamber.
SLOT_TOP = Y_TOP + 1.5 + 2.0 + 0.5           # plate slot open through the deck top (receiver deck 49.5..51.5)
def shoe_cut():
    """everything the module must keep empty: mouth, chamber, plate slot + its drop channel, lead path."""
    y0, y1 = Y_SHOE - CH_H / 2, Y_SHOE + CH_H / 2
    z0, z1 = ZM - CH_Z / 2, ZM + CH_Z / 2
    mouth = B(X_MOD0 - 1.0, X_LIP1 + 0.01, Y_SHOE - MOUTH_H / 2, Y_SHOE + MOUTH_H / 2, ZM - MOUTH_Z / 2, ZM + MOUTH_Z / 2)
    cham = B(X_LIP1, X_PLATE + 0.01, y0, y1, z0, z1)
    slot = B(X_PLATE, X_PLATE + PLATE_T + 0.2, Y_SHOE - PLATE_H / 2 - 0.3, SLOT_TOP + 1.0,
             ZM - PLATE_W / 2 - 0.3, ZM + PLATE_W / 2 + 0.3)
    lead = cyl_x(X_PLATE - 0.5, X_MOD1 + 1.0, Y_SHOE, ZM + WIRE_BACK_DZ, WIRE_R + 0.2)   # out the back, clear of the spring leg
    return U(mouth, cham, slot, lead)

def shoe_module():
    """walls only (1.5) round the mouth lip, chamber and leg slot, plus the slot's drop column up to the deck; the
    generated structure joins it to the deck and posts."""
    yc0, yc1 = Y_SHOE - CH_H / 2 - 1.5, Y_SHOE + CH_H / 2 + 1.5
    shell = B(X_MOD0, X_MOD1, yc0, yc1, MOD_Z0, MOD_Z1)
    col = B(X_PLATE - 1.5, X_MOD1, yc1 - 0.01, SLOT_TOP, ZM - PLATE_W / 2 - 0.3 - 1.5, ZM + PLATE_W / 2 + 0.3 + 1.5)
    return D(U(shell, col), shoe_cut())

def shoe_carrier(seated=False):
    """printed carrier (PETG), drawn at its free position; the copper shoe (SHOE_Z x SHOE_H x CU_T) glues on its boss."""
    dx = COMP if seated else 0.0
    xb0 = X_SHOE_FREE + CU_T + dx           # boss front = shoe back
    boss = B(xb0, X_LIP1 + dx, Y_SHOE - SHOE_H / 2, Y_SHOE + SHOE_H / 2, ZM - SHOE_Z / 2, ZM + SHOE_Z / 2)
    fl = B(X_LIP1 + dx, X_LIP1 + FL_T + dx, Y_SHOE - FL_H / 2, Y_SHOE + FL_H / 2, ZM - FL_Z / 2, ZM + FL_Z / 2)
    c = U(boss, fl)
    c = D(c, U(cyl_x(xb0 - 1, X_LIP1 + FL_T + dx + 1, Y_SHOE, ZM + WIRE_DZ, WIRE_R),          # lead, notch open to +z
               B(xb0 - 1, X_LIP1 + FL_T + dx + 1, Y_SHOE - WIRE_R, Y_SHOE + WIRE_R, ZM + WIRE_DZ, ZM + FL_Z)))
    return c

def shoe(seated=False):
    dx = COMP if seated else 0.0
    s_ = B(X_SHOE_FREE + dx, X_SHOE_FREE + CU_T + dx, Y_SHOE - SHOE_H / 2, Y_SHOE + SHOE_H / 2, ZM - SHOE_Z / 2, ZM + SHOE_Z / 2)
    return D(s_, cyl_x(X_SHOE_FREE + dx - 1, X_SHOE_FREE + CU_T + dx + 1, Y_SHOE, ZM + WIRE_DZ, WIRE_STRIP_R))


# =============== drone receiver (16 Oct 17:54: centering) ===============
# One ring part hanging from the drone belly (y = Y_TOP + 1.5, the TPU preload strip). It captures the cartridge
# ±CAPTURE in x and z with 30 deg lead-ins, then locates it with 0.3 per side before the pins touch the rail and before
# the bolts reach pocket height. Order on insert: capture (mouths) -> locate (posts on the guide rails, end tabs on the
# end faces) -> pins ride the rail chamfer -> bolts spring into the pockets -> pins drop onto gold.
CAPTURE = 3.0                               # E: funnel seat + wheel index + belt + gripper play ~±1..2, margin to 3
LEAD = math.radians(30)                     # lead-in face angle from the insertion axis (lateral force = push x 0.58)
Y_BELLY = Y_TOP + 1.5                       # 49.5: drone belly (TPU preload strip 1.5)
Y_MOUTH = 2.0                               # lowest point of the receiver, 2 above the cartridge's flat bottom when seated
POST_W, POST_T, FOOT_T = 8.0, 2.0, 4.0      # post width (z), plate thickness, foot thickness at the lead-in
END_GAP = 0.3
Z_END0, Z_END1 = -END_GAP, Z_LEN + END_GAP  # end-tab locating faces
ENDTAB_X, ENDTAB_T = 10.0, 4.0              # end tab half width, thickness (z)
BAR_T = 3.0                                 # top bars and end bars (y)

def _lead_foot(s, z0, z1, y0):
    """foot with a 30 deg lead-in on its inner bottom edge; x from the locating face outward."""
    h = CAPTURE / math.tan(LEAD)            # 5.2
    xf, xo = RAIL_FACE, RAIL_FACE + FOOT_T
    pts = [(xf + CAPTURE, y0), (xo, y0), (xo, y0 + h + 1.0), (xf, y0 + h + 1.0), (xf, y0 + h)]
    f = hull([[s * x, y, z] for x, y in pts for z in (z0, z1)])
    return f

def receiver():
    parts = []
    hcap = CAPTURE / math.tan(LEAD)
    pk, pki = pocket_block()
    sm = shoe_module()
    zpk0, zpk1 = pki["z"][0] - PK_WALL, pki["z"][1] + PK_WALL
    for s in (-1, 1):
        X = lambda a, b: (min(s * a, s * b), max(s * a, s * b))
        xa, xb = X(RAIL_FACE, RAIL_FACE + POST_T)
        posts = [(3.0, 3.0 + POST_W), (Z_LEN - 3.0 - POST_W, Z_LEN - 3.0)]
        for z0, z1 in posts:                                                   # end posts, belly to mouth
            parts.append(B(xa, xb, Y_MOUTH + hcap, Y_BELLY, z0, z1))
            parts.append(_lead_foot(s, z0, z1, Y_MOUTH))
        # pocket block (mirrored for s < 0), its own lead-in foot under it, a spine up to the bar
        p = pk.copy() if s > 0 else mirror_x(pk)
        parts.append(p)
        px0, px1 = X(RAIL_FACE, pki["x"][1])
        parts.append(B(px0, px1, Y_MOUTH, pki["y"][0] + 0.01, zpk0, zpk1))      # pocket block down to the mouth; lead-in cut below
        parts.append(B(xa, xb, pki["y"][1] - 0.01, Y_BELLY, Z_C - 4.0, Z_C + 4.0))
        # shoe module (mirrored)
        parts.append(sm.copy() if s > 0 else mirror_x(sm))
        # top bar along the belly, the full length plus the end bars' depth
        ta, tb = X(RAIL_FACE, RAIL_FACE + POST_T + 2.0)
        parts.append(B(ta, tb, Y_BELLY - BAR_T, Y_BELLY, Z_END0 - ENDTAB_T, Z_END1 + ENDTAB_T))
    # end bars and end tabs (z guides) with 30 deg lead-ins
    xw = RAIL_FACE + POST_T + 2.0
    for zf, sg in ((Z_END0, -1), (Z_END1, 1)):
        za, zb = sorted((zf, zf + sg * ENDTAB_T))
        parts.append(B(-xw, xw, Y_BELLY - BAR_T, Y_BELLY, za, zb))
        y0 = RAIL_Y0
        pts = [(zf + sg * CAPTURE, y0), (zf + sg * ENDTAB_T, y0), (zf + sg * ENDTAB_T, Y_BELLY), (zf, Y_BELLY), (zf, y0 + hcap)]
        parts.append(hull([[x, y, z] for z, y in pts for x in (-ENDTAB_X, ENDTAB_X)]))
    r = U(*parts)
    # capture keep-out: below the end of the lead-in nothing may stand inside the 30 deg line (no flat ledges)
    for sgn in (-1, 1):
        k = math.tan(LEAD)
        poly = [(0, -1), (RAIL_FACE + CAPTURE + (Y_MOUTH + 1) * k, -1), (RAIL_FACE + CAPTURE, Y_MOUTH),
                (RAIL_FACE, Y_MOUTH + hcap), (0, Y_MOUTH + hcap)]
        r = D(r, hull([[sgn * x, y, z] for x, y in poly for z in (Z_END0 - 10, Z_END1 + 10)]))
    # pockets re-cut after the union
    pocket_block()
    r = D(D(r, _POCKET_CUT), mirror_x(_POCKET_CUT))
    # shoe chambers must stay open after the union
    sc = shoe_cut()
    return D(D(r, sc), mirror_x(sc))


def report(name, m):
    print("%-28s watertight %s bodies %d vol %6.0f mm3 PETG %5.1f g bbox %s" % (
        name, m.is_watertight, len(m.split(only_watertight=False)), m.volume, m.volume * 1.27e-3,
        np.round(m.bounds, 1).tolist()))


if __name__ == "__main__":
    tr = tray(); report("tray", tr)
    frame, sl1, sl2 = block(parts=True)
    bl = U(frame, sl1, sl2); report("block (frame+2 sliders)", bl)
    report("  block frame", frame); report("  slider", sl1)
    pk, pkinfo = pocket_block(); report("pocket block", pk)
    pb = shoe_module(); report("shoe module", pb)
    ca = shoe_carrier(); report("shoe carrier", ca)
    print("pocket:", {k: (np.round(v, 2).tolist() if not isinstance(v, float) else round(v, 2)) for k, v in pkinfo.items()})
    print("shoe: z %.2f..%.2f y %.2f..%.2f, face free %.2f seated %.2f; module x %.1f..%.1f z %.1f..%.1f; spring seat %.1f (free %.1f)"
          % (ZM - SHOE_Z / 2, ZM + SHOE_Z / 2, Y_SHOE - SHOE_H / 2, Y_SHOE + SHOE_H / 2, X_SHOE_FREE, X_STRIP_FACE,
             X_MOD0, X_MOD1, MOD_Z0, MOD_Z1, SPR_SEAT, SPR_SEAT + COMP))
    print("cartridge: %.1f x %.1f x %.1f (%.1f over the bosses); recess %.0f x %.0f, %.2f deep; block z %.1f..%.1f; strips z %.1f..%.1f y %.0f..%.0f"
          % (2 * XW_OUT, Y_TOP, Z_LEN, 2 * BOSS_X1, 2 * REC_X, T, CH_Y0, Z_BLK0, Z_BLK1, Z_STRIP0, Z_STRIP1, Y_BAND0, Y_BAND1))
    worst = 0
    for d in np.linspace(0, TRAVEL, 9):
        s1 = sl1.copy(); s1.apply_translation([-d, 0, 0]); s2 = sl2.copy(); s2.apply_translation([d, 0, 0])
        worst = max(worst, s1.intersection(frame, engine="manifold").volume + s2.intersection(frame, engine="manifold").volume
                    + s1.intersection(s2, engine="manifold").volume)
    print("sliders vs frame and each other over the travel, worst %.3f mm3 (0 = clear; tabs meet at 4.0)" % worst)
    print("bolt tip at rest x %.2f (rail face %.1f, overlap %.1f); retracted tip x %.2f (clears rail by %.2f)"
          % (X_TIP, RAIL_FACE, X_TIP - RAIL_FACE, X_TIP - TRAVEL, RAIL_FACE - (X_TIP - TRAVEL)))
    s1t = sl1.copy(); s1t.apply_translation([0, 0, Z_BLK0])
    print("latched bolt vs pocket block (should be 0): %.3f mm3" % s1t.intersection(pk, engine="manifold").volume)
    s1d = s1t.copy(); s1d.apply_translation([0, -0.35, 0])
    print("bolt after a 0.35 drop vs pocket block (> 0 = bears after 0.25): %.3f mm3" % s1d.intersection(pk, engine="manifold").volume)
    print("tray vs pocket block: %.3f mm3, tray vs shoe module: %.3f mm3, tray vs block: %.3f mm3" % (
        tr.intersection(pk, engine="manifold").volume, tr.intersection(pb, engine="manifold").volume,
        tr.intersection(bl.copy().apply_translation([0, 0, Z_BLK0]), engine="manifold").volume))
    # finger blade check: open blade x 11..14 at y 0..7.5 clears the tab and the recess wall
    fing = B(TAB_OUT + 1.0, TAB_OUT + 1.0 + FINGER_T, -2, CH_Y0 - 0.5, (T - 12) / 2, (T + 12) / 2)
    print("open finger blade vs frame %.3f, vs slider %.3f mm3 (0 = clear)" % (
        fing.intersection(frame, engine="manifold").volume, fing.intersection(sl1, engine="manifold").volume))
    tr.export(os.path.join(OUT, "s2_cartridge_tray_v3.stl"))
    bl.export(os.path.join(OUT, "s2_cartridge_block_v3.stl"))
    pk2 = pk.copy(); pk2.apply_translation(-pk.bounds[0]); pk2.export(os.path.join(OUT, "s2_pocket_v3.stl"))
    # shoe checks: seated shoe vs tray (should touch only the strip face), carrier free in its chamber
    tr_s = tr.intersection(shoe(seated=True), engine="manifold").volume
    print("seated shoe vs tray: %.3f mm3 (0: lands on the strip face at x %.2f)" % (tr_s, X_STRIP_FACE))
    for sd in (False, True):
        print("carrier (%s) vs module: %.3f mm3" % ("seated" if sd else "free", shoe_carrier(sd).intersection(pb, engine="manifold").volume))
    ca2 = ca.copy(); ca2.apply_translation(-ca2.bounds[0]); ca2.export(os.path.join(OUT, "s2_shoe_carrier_v4.stl"))
    print("wrote", sorted(os.listdir(OUT)))
