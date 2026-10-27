"""X-axis flip swap v4 (27 Oct 2026, Spark's latch review via Alex): the latch moves under the pack.

What changed from v3 (only the latching and unlatching; XT60, guides, funnel sides, keys, holder deck are v3's):
- Shell: a tray under the pack floor holds two GATE BARS, one per end. Each bar is a rigid PETG lever on a vertical
  steel dowel, pushed latched by a steel spring. Its tip comes out of the shell's end face and sits in a pocket in a
  holder END LEG, on a 12 mm deep ledge. The shell is no wider than v3; it is 14.8 mm taller.
- Holder: the rocker lugs are gone; two end legs (one print with the deck) carry the pockets, ledges and release
  channels. Same length as v2 / v3 (178.6).
- Pincer: narrower than the battery (x +-22). Two round finger posts rise from below through T-slot keyholes in the
  shell bottom into deep slots in the bars, near the pivots: 1 mm at the finger = 4.8 mm at the tip.
- The two bars are the same print and point-symmetric about (x 0, y Q_Y): bar A at the +y (XT60) end releases toward
  -x, bar B at the -y (tail) end releases toward +x.

Frame as v1-v3: x across, y along (+y XT60 end), z up, z 0 = shell top at seat. All numbers C unless tagged E.
"""
import math
import numpy as np
import manifold3d as mf
import xswap as V3                      # helpers + XT60, posts, sockets, keys, gears (unchanged pieces)
from xswap import M, box, cyl, hull, U, tm, overlap, SEG

OUT = V3.os.path.join(V3.ROOT, "runs", "xswap", "v4")

# ============================================================================== shared numbers (v3)
SX, CAV_X, WALL = V3.SX, V3.CAV_X, V3.WALL
SY0, SY1, CAV_Y0, CAV_Y1 = V3.SY0, V3.SY1, V3.CAV_Y0, V3.CAV_Y1
CAV_Z0, CAV_Z1 = V3.CAV_Z0, V3.CAV_Z1            # pack cavity (floor top -52.5, deck underside -3)
Y_X, HEAD_Y1 = V3.Y_X, V3.HEAD_Y1
HC, SK_T, SK_D, DK_T = V3.HC, V3.SK_T, V3.SK_D, V3.DK_T
PETG = V3.PETG
Q_Y = 6.5                                        # centre of the latch symmetry (between the two ends: (SY0 + SY1) / 2)

# ============================================================================== tray (shell underside)
FLOOR_T = 3.0
Z_FL0 = CAV_Z0 - FLOOR_T                         # -55.5 floor plate underside
BAND_D, BAND_H = 16.0, 5.0                       # end band under the floor: 16 deep (y), 5 tall (to -60.5)
Z_BAND0 = Z_FL0 - BAND_H                         # -60.5
END_W = 4.0                                      # tray end walls below the band (y)
G = 0.3                                          # running clearance
TIP_H = 6.7
Z_TIP1 = Z_BAND0 - G                             # -60.8 tip top
Z_TIP0 = Z_TIP1 - TIP_H                          # -67.5 tip bottom = load face
Z_BP1 = Z_TIP0 - G                               # -67.8 bottom plate top
BP_T = 2.5                                       # bottom plate = T-slot lips
SZ0 = Z_BP1 - BP_T                               # -70.3 shell bottom
Z_BODY0, Z_BODY1 = -65.0, Z_FL0 - G              # bar body (between pivot and band): -65.0 .. -55.8
TRAY_X = CAV_X                                   # tray inner half width (24.25)

# ============================================================================== gate bar A (+y end); B = A turned 180 deg about (0, Q_Y)
E_A = SY1                                        # 89: A's end face
TIP_OUT = 4.5                                    # tip past the end face (0.3 gap + 4.2 in the leg)
TIP_Y1 = E_A + TIP_OUT                           # 93.5
TIP_W = 10.0                                     # tip width (x), latched at x 0
X_LAT = 0.0
PIV_Y = 33.5                                     # pivot (x 0)
FIN_A = 12.0                                     # finger slot centre from the pivot
FIN_Y = PIV_Y + FIN_A                            # 45.5
L_TIP = (E_A + TIP_Y1) / 2 - PIV_Y               # 57.75: pivot to the tip's bearing centre
RATIO = L_TIP / FIN_A                            # 4.81
BODY_W = 8.0
BOSS_R, PIN_R = 6.0, 2.075                       # pivot boss, dowel 4 mm (0.075 radial clearance in the bar)
SLOT_R_POST = 3.5                                # finger post radius (round, never square on square)
SLOT_CLR = 0.8                                   # post to slot wall, each side (x)
SLOT_W = 2 * (SLOT_R_POST + SLOT_CLR)            # 8.6
SLOT_L = 12.0                                    # along y (post 7 + 2.5 a side: arc + y tolerance)
SLOT_WALL = 3.0
BAND_Y0 = E_A - BAND_D                           # 73: the band's inner edge (tip region starts under it)
SPR_D = 22.0                                     # spring from the pivot
SPR_Y = PIV_Y + SPR_D                            # 55.5
SPR_Z = 0.5 * (Z_BODY0 + Z_BODY1)                # -60.4
SPR_R, SPR_STUB_R, SPR_STUB_L = 3.0, 2.3, 2.0    # spring OD 6, stub on the bar
SPR_BOSS_X0 = -TRAY_X                            # boss on the -x wall
SPR_BOSS_X1 = -18.5
SPR_POCKET = 2.5
SPR_FREE, SPR_RATE = 21.0, 0.8                   # E: 6 OD x 0.6 wire x 21 free, ~0.8 N/mm (buy to suit)
LEDGE_CLR = 0.4                                  # tip to the pocket's far wall (latched) and to the ledge edge
REL_SWING = TIP_W + LEDGE_CLR + G                # 10.7: tip's +x edge from 5.0 to -5.7, over the channel
STOP_X = X_LAT + TIP_W / 2                       # 5.0: end-wall slot end = the bar's latched stop

def bar_local():
    """bar A latched, world coords. Body (tall, between pivot and the band), tip (low, under the band and out of the
    end face), pivot boss, finger slot, spring stub. Every edge a finger, spring or ledge touches is round or chamfered."""
    zb0, zb1 = Z_BODY0, Z_BODY1
    body = box(-BODY_W / 2, BODY_W / 2, PIV_Y, BAND_Y0 - 0.5, zb0, zb1)
    boss = cyl("z", 0, PIV_Y, BOSS_R, zb0, zb1)
    fin = box(-(SLOT_W / 2 + SLOT_WALL), SLOT_W / 2 + SLOT_WALL, FIN_Y - SLOT_L / 2 - SLOT_WALL, FIN_Y + SLOT_L / 2 + SLOT_WALL, zb0, zb1)
    tip = box(-TIP_W / 2, TIP_W / 2, BAND_Y0 - 2.5, TIP_Y1, Z_TIP0, Z_TIP1)
    step = box(-BODY_W / 2, BODY_W / 2, BAND_Y0 - 2.5, BAND_Y0 - 0.5, Z_TIP0, zb1)       # body-to-tip knee
    b = U(body, boss, fin, tip, step)
    # tip end: 1.0 chamfer on the four end edges; 0.6 on the long bottom edges (ledge entry)
    c = 1.0
    for sx in (-1, 1):
        b = b - hull([[sx * (TIP_W / 2 + 0.01), TIP_Y1 - c, z] for z in (Z_TIP0 - 1, Z_TIP1 + 1)] +
                     [[sx * (TIP_W / 2 - c), TIP_Y1 + 0.01, z] for z in (Z_TIP0 - 1, Z_TIP1 + 1)] +
                     [[sx * (TIP_W / 2 + 0.01), TIP_Y1 + 0.01, z] for z in (Z_TIP0 - 1, Z_TIP1 + 1)])
    for zz, sz in ((Z_TIP0, -1), (Z_TIP1, 1)):
        b = b - hull([[x, TIP_Y1 - c, zz + sz * 0.01] for x in (-TIP_W, TIP_W)] + [[x, TIP_Y1 + 0.01, zz - sz * c] for x in (-TIP_W, TIP_W)] +
                     [[x, TIP_Y1 + 0.01, zz + sz * 0.01] for x in (-TIP_W, TIP_W)])
    # pivot hole
    b = b - cyl("z", 0, PIV_Y, PIN_R, zb0 - 1, zb1 + 1)
    # finger slot: obround, through, 1.0 x 45 deg mouth chamfer underneath
    sl = M.batch_hull([cyl("z", 0, FIN_Y + sy * (SLOT_L / 2 - SLOT_W / 2), SLOT_W / 2, zb0 - 1, zb1 + 1) for sy in (-1, 1)])
    mouth = M.batch_hull([M.cylinder(1.01, SLOT_W / 2 + 1.0, SLOT_W / 2, SEG).translate([0, FIN_Y + sy * (SLOT_L / 2 - SLOT_W / 2), zb0 - 0.01])
                          for sy in (-1, 1)])
    b = b - sl - mouth
    # spring stub on the -x face
    b = b + cyl("x", SPR_Y, SPR_Z, SPR_STUB_R, -BODY_W / 2 - SPR_STUB_L, -BODY_W / 2 + 0.01)
    return b

def rot_z(m, deg, cx, cy):
    return m.translate([-cx, -cy, 0]).rotate([0, 0, deg]).translate([cx, cy, 0])

def bar(theta=0.0, which="A"):
    """bar at swing angle theta (deg, + = toward release). A turns about +z by +theta (tip toward -x)."""
    a = rot_z(bar_local(), theta, 0.0, PIV_Y)
    return a if which == "A" else rot_z(a, 180, 0.0, Q_Y)

def theta_for_tip(dx):
    return math.degrees(math.asin(dx / L_TIP))

TH_REL = theta_for_tip(REL_SWING)                # ~10.7 deg

# ============================================================================== finger post + T-slot (A; B by symmetry)
FL_HX, FL_HY, FL_T = 3.5, 7.0, 2.0               # flange plate half sizes (x, y), thickness
Z_FL_0 = Z_BP1 + G                               # -67.5 flange bottom
Z_FL_1 = Z_FL_0 + FL_T                           # -65.5 (0.5 under the bar body)
Z_POST_TOP = -57.3                               # post tip (7.7 into the bar body: deep slot)
DELTA = 0.5                                      # pincer-to-shell x error the drive is sized for (E)
STROKE = SLOT_CLR + DELTA + REL_SWING / RATIO + 0.25    # ~3.78 per finger
ENTRY_CLR = 0.5

def tslot_cuts():
    """bottom-plate cuts for finger A: entry hole (flange passes at the open position), narrow slot (post passes over
    the whole stroke), 45 deg mouth chamfers. The lips left under the flanges hold the shell to the pincer."""
    ex0, ex1 = -FL_HX - ENTRY_CLR - DELTA, FL_HX + ENTRY_CLR + DELTA
    ey = FL_HY + SLOT_CLR
    entry = box(ex0, ex1, FIN_Y - ey, FIN_Y + ey, V3.SZ0 * 0 + SZ0 - 1, Z_BP1 + 1)
    nx0 = -STROKE - SLOT_R_POST - ENTRY_CLR - DELTA
    ny = SLOT_R_POST + SLOT_CLR
    narrow = box(nx0, ex1, FIN_Y - ny, FIN_Y + ny, SZ0 - 1, Z_BP1 + 1)
    ch = 1.2
    cham = hull([[x, FIN_Y + sy * (ey + ch), SZ0 - 0.01] for x in (ex0 - ch, ex1 + ch) for sy in (-1, 1)] +
                [[x, FIN_Y + sy * ey, SZ0 + ch] for x in (ex0, ex1) for sy in (-1, 1)])
    chn = hull([[x, FIN_Y + sy * (ny + ch), SZ0 - 0.01] for x in (nx0 - ch, ex1 + ch) for sy in (-1, 1)] +
               [[x, FIN_Y + sy * ny, SZ0 + ch] for x in (nx0, ex1) for sy in (-1, 1)])
    return U(entry, narrow, cham, chn)

# ============================================================================== shell v4
def tray_cavity():
    """cavity under the floor: bar body zone (full height to the floor) between the bands, tip zone under the bands."""
    y0, y1 = SY0 + END_W, SY1 - END_W
    low = box(-TRAY_X, TRAY_X, y0, y1, Z_BP1, Z_BAND0)
    high = box(-TRAY_X, TRAY_X, SY0 + BAND_D, SY1 - BAND_D, Z_BAND0 - 0.01, Z_FL0)
    return low + high

def end_slot_A():
    """end-wall slot for A's tip (below the band): from the release side to the latched stop (STOP_X)."""
    x_min = X_LAT - TIP_W / 2 - (STROKE - 0.0) * RATIO - 0.6           # max swing at a +-DELTA pincer
    return box(max(x_min, -TRAY_X + 2.0), STOP_X, SY1 - END_W - 1, SY1 + 1, Z_BP1, Z_BAND0)

def tray_features_A():
    """bosses inside the tray for A: pivot spacer under the bar, spring boss on the -x wall."""
    spacer = cyl("z", 0, PIV_Y, 5.0, Z_BP1 - 0.01, Z_BODY0 - G)
    sb = box(SPR_BOSS_X0 - 0.01, SPR_BOSS_X1, SPR_Y - 5.0, SPR_Y + 5.0, Z_BP1 - 0.01, Z_FL0 + 0.01)
    return spacer + sb

def tray_cuts_A():
    pin = cyl("z", 0, PIV_Y, 1.975, SZ0 - 1, Z_BP1 + 0.01) + cyl("z", 0, PIV_Y, 2.05, Z_BP1, Z_FL0 + 1.5)  # press below, blind 1.5 above
    pocket = cyl("x", SPR_Y, SPR_Z, SPR_R + 0.2, SPR_BOSS_X1 - SPR_POCKET, SPR_BOSS_X1 + 0.01)
    return U(pin, pocket, tslot_cuts(), end_slot_A())

def sym(m):
    return rot_z(m, 180, 0.0, Q_Y)

def shell_body():
    """v3 shell (pack sleeve, XT60 block, tail bolts, guide sockets, side windows, deck windows) with the rockers
    removed and the tray added below the floor."""
    outer = box(-SX, SX, SY0, SY1, SZ0, 0.0)
    cav = box(-CAV_X, CAV_X, SY0 - 1, CAV_Y1, CAV_Z0, CAV_Z1)
    head = box(-CAV_X, CAV_X, CAV_Y1 - 0.01, HEAD_Y1, CAV_Z0, CAV_Z1)         # wire bay: closed below now
    s = outer - cav - head
    blk = box(-20.0, 20.0, Y_X - 6.6, HEAD_Y1, -15.5, 0.0)
    s = s + blk
    s = s - box(-(V3.XF_W + 0.3) / 2, (V3.XF_W + 0.3) / 2, Y_X - (V3.XF_D + 0.3) / 2, Y_X + (V3.XF_D + 0.3) / 2, -16.5, 1)
    for sx in (-1, 1):
        s = s - cyl("z", sx * 15.0, Y_X, V3.INS_D / 2, -16.5, -15.5 + 6.5)
    for z in V3.TAIL_BOLT_Z:
        for sx in (-1, 1):
            s = s + cyl("x", V3.TAIL_BOLT_Y, z, 4.0, sx * SX - (2.0 if sx < 0 else 0), sx * SX + (2.0 if sx > 0 else 0))   # r 4.0: flush with the tail face (end leg 0.3 away)
        s = s - cyl("x", V3.TAIL_BOLT_Y, z, 1.7, -SX - 5, SX + 5)
    s = s - V3.shell_windows(V3.Y_C)
    sk = V3.socket(V3.Y_C + V3.G_OFF); s = s + sk[0]; s = s - sk[1]
    sk = V3.socket(V3.Y_C - V3.G_OFF); s = s + sk[0].mirror([1, 0, 0]); s = s - sk[1].mirror([1, 0, 0])
    # tray
    s = s - tray_cavity()
    f = tray_features_A()
    s = s + f + sym(f)
    c = tray_cuts_A()
    s = s - c - sym(c)
    return s

def shell(theta=0.0):
    return shell_body() + bar(theta, "A") + bar(theta, "B")

def pack():
    return V3.pack()

# ============================================================================== holder v4
LEG_T, GROOVE_D = 6.5, 4.5
LEG_CH = 6.0                                     # 30 deg lead-in on the leg's inner bottom edge (y)
LEG_Z0 = Z_TIP0 - G - 12.0 - LEG_CH / math.tan(math.radians(30))   # -90.2: a full 12 mm ledge above the lead-in
POCKET_X0, POCKET_X1 = -(TIP_W / 2 + LEDGE_CLR), TIP_W / 2 + LEDGE_CLR    # -5.4 .. 5.4 (A)
POCKET_Z0, POCKET_Z1 = Z_TIP0 - G, Z_TIP1 + G    # -67.8 (ledge top) .. -60.5
CHAN_X0 = X_LAT - TIP_W / 2 - STROKE * RATIO - 1.0   # channel to past the deepest swing
LEG_INS = [(sx * 15.0, SY1 + HC + 0.5) for sx in (-1, 1)]   # M3 inserts over leg A (B by symmetry)

def leg_A():
    """end leg at +y: from the deck down to LEG_Z0. Groove in the inner face: pocket over the ledge (latched tip) and the
    release channel beside it, open at the bottom with lead-ins; 30 deg chamfer on the inner bottom edge."""
    yi = SY1 + HC
    yo = yi + LEG_T
    xo = SX + HC + SK_T
    leg = box(-xo, xo, yi, yo, LEG_Z0, DK_T)
    yg = yi + GROOVE_D
    pocket = box(POCKET_X0, POCKET_X1, yi - 1, yg, POCKET_Z0, POCKET_Z1)
    chan = box(CHAN_X0, POCKET_X0 + 0.01, yi - 1, yg, LEG_Z0 - 1, POCKET_Z1)
    # channel mouth: 45 deg flare 3.0 in x (both sides) and y, over the bottom 3.0
    f = 3.0
    mouth = hull([[x, y, LEG_Z0 - 0.01] for x in (CHAN_X0 - f, POCKET_X0 + f) for y in (yi - 1, yg + f)] +
                 [[x, y, LEG_Z0 + f] for x in (CHAN_X0, POCKET_X0) for y in (yi - 1, yg)])
    # ledge's free bottom corner (channel side): 2 x 45 deg
    lc = hull([[POCKET_X0 - 0.01, y, LEG_Z0 - 0.01] for y in (yi - 1, yg)] + [[POCKET_X0 + 2.0, y, LEG_Z0 - 0.01] for y in (yi - 1, yg)] +
              [[POCKET_X0 - 0.01, y, LEG_Z0 + 2.0] for y in (yi - 1, yg)])
    # inner bottom edge lead-in for the shell's end face: 30 deg from vertical, LEG_CH deep
    lead = hull([[x, yi - 0.01, LEG_Z0 + LEG_CH / math.tan(math.radians(30))] for x in (-xo - 1, xo + 1)] +
                [[x, yi + LEG_CH, LEG_Z0 - 0.01] for x in (-xo - 1, xo + 1)] + [[x, yi - 0.01, LEG_Z0 - 0.01] for x in (-xo - 1, xo + 1)])
    leg = leg - pocket - chan - mouth - lc - lead
    # pocket top and channel top edges: 0.6 chamfer is skipped (inside the groove, nothing slides on them)
    return leg

def holder():
    """v3 holder deck + side skirt + side funnel + guide posts (lengthened) + keys; lugs and end flares gone; end legs."""
    ix, iy0, iy1 = SX + HC, SY0 - HC, SY1 + HC
    ox = ix + SK_T
    deck = box(-ox, ox, iy0, iy1, 0.0, DK_T)
    cells = []
    nx, ny = 2, 7
    xs = np.linspace(-ox + SK_T, ox - SK_T, nx + 1); ys = np.linspace(iy0 + SK_T, iy1 - SK_T, ny + 1)
    for i in range(nx):
        for j in range(ny):
            cells.append(box(xs[i] + V3.RIB / 2, xs[i + 1] - V3.RIB / 2, ys[j] + V3.RIB / 2, ys[j + 1] - V3.RIB / 2, V3.SKIN, DK_T + 1))
    ins = LEG_INS + [(-x, 2 * Q_Y - y) for x, y in LEG_INS]
    solid = U(box(-24.5, 24.5, Y_X - 9.5, Y_X + 9.5, -1, DK_T + 1),
              *[cyl("z", x, y, 6.5, -1, DK_T + 1) for x, y in V3.MOUNT + ins],
              box(-ox - 1, ox + 1, iy1 - 8.0, iy1 + 1, -1, DK_T + 1), box(-ox - 1, ox + 1, iy0 - 1, iy0 + 8.0, -1, DK_T + 1))
    deck = deck - (U(*cells) - solid)
    skirt = U(box(-ox, -ix, iy0, iy1, -SK_D, 0.0), box(ix, ox, iy0, iy1, -SK_D, 0.0))
    lead = U(*[hull([[sx * ix, y, -SK_D + V3.LEAD_V] for y in (iy0 - 1, iy1 + 1)] + [[sx * (ix + V3.LEAD_H), y, -SK_D - 0.01] for y in (iy0 - 1, iy1 + 1)] +
                    [[sx * (ix - 0.01), y, -SK_D - 0.01] for y in (iy0 - 1, iy1 + 1)]) for sx in (-1, 1)])
    skirt = skirt - lead
    # side funnel (x flare only; the legs guide the ends)
    z1, z0 = -SK_D + 0.01, -SK_D - V3.FLARE_W
    fun = []
    for sx in (-1, 1):
        o = hull([[sx * (ix + SK_T), y, z1] for y in (iy0, iy1)] + [[sx * (ix + SK_T + V3.FLARE_H), y, z0] for y in (iy0, iy1)] +
                 [[sx * ix, y, z1] for y in (iy0, iy1)] + [[sx * (ix + V3.FLARE_H), y, z0] for y in (iy0, iy1)])
        fun.append(o)
    h = deck + skirt + U(*fun)
    p = post(V3.Y_C + V3.G_OFF)
    h = h + p + p.translate([0, -(2 * V3.G_OFF), 0]).mirror([1, 0, 0])
    h = h + V3.keys()
    la = leg_A()
    h = h + la + sym(la)
    h = h - box(-(V3.XF_W + 0.5) / 2, (V3.XF_W + 0.5) / 2, Y_X - (V3.XF_D + 0.5) / 2, Y_X + (V3.XF_D + 0.5) / 2, -1, V3.XM_FU + 0.01)
    h = h - box(-(V3.XM_FW + 0.6) / 2, (V3.XM_FW + 0.6) / 2, Y_X - (V3.XM_FD + 0.6) / 2, Y_X + (V3.XM_FD + 0.6) / 2, V3.XM_FU, DK_T + 1)
    for sx in (-1, 1):
        h = h - cyl("z", sx * 19.0, Y_X, V3.INS_D / 2, DK_T - 6.5, DK_T + 1)
    for x, y in V3.MOUNT + ins:
        h = h - cyl("z", x, y, V3.INS_D / 2, DK_T - V3.INS_L, DK_T + 1)
    return h

POST_Z0, POST_TIP = -53.0, -62.0                 # guide posts lengthened (v3 -40 / -49) so they lead the bar tips

def post(gy):
    ox = SX + HC + SK_T
    pad = box(18.0, V3.GX + V3.POST_R + 2.0, gy - V3.POST_R - 2.0, gy + V3.POST_R + 2.0, 0.0, DK_T)
    web = box(SX + HC + 0.5, V3.GX, gy - 4.0, gy + 4.0, -SK_D, 0.01)
    fil = M.cylinder(5.0, V3.POST_R, V3.POST_R + 3.0, SEG).translate([V3.GX, gy, -5.0])
    shank = cyl("z", V3.GX, gy, V3.POST_R, POST_Z0, 0.01)
    tip = M.cylinder(POST_Z0 - POST_TIP, V3.POST_TIP_R, V3.POST_R, SEG).translate([V3.GX, gy, POST_TIP])
    return U(pad, web, fil, shank, tip)

def xt60_cap(): return V3.xt60_cap()
def xt60_male(): return V3.xt60_male()
def xt60_female(): return V3.xt60_female()
def xt60_keeper(): return V3.xt60_keeper()

# ============================================================================== pincer v4 (narrower than the battery)
JW = 22.0                                        # jaw half width (battery 23.5)
Y_P = Q_Y                                        # pinion centre y (the symmetry point)
M_G, Z_G, R_P = V3.M_G, V3.Z_G, V3.R_P
Z_JT = SZ0 - G                                   # -70.6 jaw top at the grip height
ARM_H, ARM_W = 4.4, 8.0
Z_ARM1 = Z_JT - 0.3                              # arm top -70.9 (0.3 under the jaw top)
Z_ARM0 = Z_ARM1 - ARM_H                          # -75.3
RK_Z1 = Z_ARM0; RK_Z0 = RK_Z1 - 10.0             # rack under the arm: -85.3 .. -75.3
RK_L = 30.0
SV_L, SV_W, SV_H = V3.SV_L, V3.SV_W, V3.SV_H
SV_EAR_L, SV_EAR_T, SV_EAR_DZ, SV_HOLE, SV_SHAFT = V3.SV_EAR_L, V3.SV_EAR_T, V3.SV_EAR_DZ, V3.SV_HOLE, V3.SV_SHAFT
SV_TOP = RK_Z0 - 2.8 - 3.5
Z_AX = SV_TOP - SV_H / 2                         # flip axis: blind 8 mm shaft bores in the x faces, beside the servo
JB_Y0, JB_Y1 = -2 * 0 + (2 * Y_P - (FIN_Y + 6.0)), FIN_Y + 6.0

def finger_A(u):
    """finger A at drive offset u (0 open .. STROKE shut; A moves toward -x): rack + arm + round post with the flange."""
    x = -u
    rk_y0 = Y_P + R_P - M_G
    p = math.pi * M_G
    x_start0 = -RK_L / 2                         # rack x span at u 0: -15 .. 15
    shift = (-(x_start0 + p / 2)) % p
    rack = V3.rack_2d(RK_L, M_G, shift=shift).extrude(RK_Z1 - RK_Z0).translate([x_start0, Y_P + R_P, RK_Z0])
    rack = rack.translate([x, 0, 0])
    arm = box(x - ARM_W / 2, x + ARM_W / 2, Y_P + R_P + M_G + 0.6, FIN_Y + SLOT_R_POST + 1.5, Z_ARM0, Z_ARM1)   # starts past the pinion's tips
    neck = cyl("z", x, FIN_Y, SLOT_R_POST, Z_ARM1 - 0.01, Z_FL_0 + 0.01)
    flange = box(x - FL_HX, x + FL_HX, FIN_Y - FL_HY, FIN_Y + FL_HY, Z_FL_0, Z_FL_1)
    fl_ch = 0.6
    flange = flange - U(*[hull([[x + sx * (FL_HX + 0.01), y, z] for y in (FIN_Y - FL_HY - 1, FIN_Y + FL_HY + 1) for z in (Z_FL_1 - fl_ch, Z_FL_1 + 0.01)] +
                               [[x + sx * (FL_HX - fl_ch), y, Z_FL_1 + 0.01] for y in (FIN_Y - FL_HY - 1, FIN_Y + FL_HY + 1)]) for sx in (-1, 1)])
    post_ = cyl("z", x, FIN_Y, SLOT_R_POST, Z_FL_1 - 0.01, Z_POST_TOP - 1.5)
    bullet = M.cylinder(1.5, SLOT_R_POST, SLOT_R_POST - 1.5, SEG).translate([x, FIN_Y, Z_POST_TOP - 1.5])
    return U(rack, arm, neck, flange, post_, bullet)

def finger_B(u):
    return sym(finger_A(u))

def pinion():
    p = V3.involute_gear_2d(M_G, Z_G).extrude(RK_Z1 - RK_Z0 + 0.31).translate([0, Y_P, RK_Z0 - 0.3])   # teeth run 0.3 under the racks
    p = p + cyl("z", 0, Y_P, 14.5, RK_Z0 - 2.8, RK_Z0 - 0.29)
    p = p - cyl("z", 0, Y_P, 10.9, RK_Z0 - 3.3, RK_Z0 - 0.3)
    p = p - cyl("z", 0, Y_P, 1.7, RK_Z0 - 3, RK_Z1 + 1) - cyl("z", 0, Y_P, 3.3, RK_Z1 - 3.0, RK_Z1 + 1)
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        p = p - cyl("z", 7.75 * math.cos(a), Y_P + 7.75 * math.sin(a), 1.0, RK_Z0 - 3, RK_Z1 + 1, 16)
    return p

def jaw_body():
    sy_c = Y_P - (SV_L / 2 - SV_SHAFT)
    y0 = min(2 * Y_P - (FIN_Y + 6.0), sy_c - SV_EAR_L / 2 - 4.0)
    y1 = max(FIN_Y + 6.0, sy_c + SV_EAR_L / 2 + 4.0)
    zb = SV_TOP - SV_H
    b = box(-JW, JW, y0, y1, zb, Z_JT)
    # arm channels (open top) over the stroke, + rack tunnels; A then B by symmetry
    def cuts_A():
        c = box(-STROKE - ARM_W / 2 - 0.6, ARM_W / 2 + 0.6, Y_P + R_P - 1.6, FIN_Y + SLOT_R_POST + 2.1, Z_ARM0 - 0.3, Z_JT + 1)
        c = c + box(-RK_L / 2 - STROKE - 0.6, RK_L / 2 + 0.6, Y_P + R_P - M_G - 0.3, Y_P + R_P + 1.25 * M_G + 7.3, RK_Z0 - 0.3, Z_ARM0 + 0.01)
        return c
    ca = cuts_A()
    b = b - ca - sym(ca)
    b = b - cyl("z", 0, Y_P, R_P + M_G + 0.6, RK_Z0 - 0.3, Z_ARM0 + 0.01) - cyl("z", 0, Y_P, 15.1, RK_Z0 - 3.3, RK_Z0 + 0.01)
    # servo pocket, spline boss, ear slot, ear inserts
    ear_top = SV_TOP - SV_EAR_DZ
    b = b - box(-SV_W / 2 - 0.25, SV_W / 2 + 0.25, sy_c - SV_L / 2 - 0.25, sy_c + SV_L / 2 + 0.25, zb - 1, SV_TOP + 0.01)
    b = b - cyl("z", 0, Y_P, 7.0, SV_TOP - 0.1, RK_Z0 - 3.2)
    b = b - box(-SV_W / 2 - 0.25, SV_W / 2 + 0.25, sy_c - SV_EAR_L / 2 - 0.5, sy_c + SV_EAR_L / 2 + 0.5, zb - 1, ear_top)
    for sy in (-1, 1):
        for sx in (-1, 1):
            b = b - cyl("z", sx * 5.0, sy_c + sy * SV_HOLE, V3.INS_D / 2, ear_top - 0.01, ear_top + 6.5)
    # flip shaft bores (8 mm, blind, 9 deep) in both x faces + set-screw inserts from below
    for sx in (-1, 1):
        bore = cyl("x", Y_P, Z_AX, 4.1, (JW - 9.0) if sx > 0 else -JW - 1, (JW + 1) if sx > 0 else -(JW - 9.0))
        sset = cyl("z", sx * (JW - 4.5), Y_P, V3.INS_D / 2, zb - 1, zb + 6.0) + cyl("z", sx * (JW - 4.5), Y_P, 1.7, zb - 1, Z_AX)
        b = b - bore - sset
    # lightening cores either side of the servo, open below
    for sx in (-1, 1):
        x0, x1 = (SV_W / 2 + 3.0, JW - 3.0) if sx > 0 else (-(JW - 3.0), -(SV_W / 2 + 3.0))
        for (ya, yb) in ((y0 + 3.0, Y_P - 9.0), (Y_P + 9.0, y1 - 3.0)):
            b = b - box(x0, x1, ya, yb, zb - 1, RK_Z0 - 4.0)
    return b

def servo_dummy():
    sy_c = Y_P - (SV_L / 2 - SV_SHAFT)
    s = box(-SV_W / 2, SV_W / 2, sy_c - SV_L / 2, sy_c + SV_L / 2, SV_TOP - SV_H, SV_TOP)
    s = s + box(-SV_W / 2, SV_W / 2, sy_c - SV_EAR_L / 2, sy_c + SV_EAR_L / 2, SV_TOP - SV_EAR_DZ - SV_EAR_T, SV_TOP - SV_EAR_DZ)
    s = s + cyl("z", 0, Y_P, 6.5, SV_TOP, RK_Z0 - 2.8) + cyl("z", 0, Y_P, 10.75, RK_Z0 - 2.8, RK_Z0 - 0.3)
    return s

def pincer(u, ang=None):
    """jaw + fingers at drive offset u; pinion turned to match (deg)."""
    a = math.degrees(u / R_P) if ang is None else ang
    pin = rot_z(pinion(), a, 0.0, Y_P)
    return dict(jaw=jaw_body(), finger_a=finger_A(u), finger_b=finger_B(u), pinion=pin, servo=servo_dummy())

# ============================================================================== swing the bar follows from a finger offset
def theta_from_u(u, delta=0.0):
    """bar swing for finger offset u with the pincer delta off in x (toward the drive side +): contact after
    SLOT_CLR - delta of free travel; then the post pushes the slot wall at radius FIN_A."""
    d = u - (SLOT_CLR - delta)
    return 0.0 if d <= 0 else math.degrees(math.asin(min(1.0, d / FIN_A)))

# ============================================================================== mass / CG
def swap_cg():
    s = tm(shell_body()); b = tm(bar_local()); k = tm(xt60_keeper())
    ms, mb, mk = s.volume * PETG, b.volume * PETG, k.volume * PETG
    parts = [(V3.PACK_G, np.array([0, 0, 0.5 * (CAV_Z0 + CAV_Z1)])), (ms, s.center_mass), (mk, k.center_mass),
             (mb, b.center_mass), (mb, np.array([-b.center_mass[0], 2 * Q_Y - b.center_mass[1], b.center_mass[2]])),
             (12.0, np.array([0, Y_X, -8.0])), (6.0, np.array([0, -72.0, -31.0])), (2 * 1.6, np.array([0, Q_Y, -62.0]))]
    m = sum(p[0] for p in parts)
    return m, sum(p[0] * p[1] for p in parts) / m, ms, mb
