"""X-axis flip swap v1 (Alex, 25 Oct 2026): battery shell, battery holder (drone and charging bay, one part), pincer.

Concept (Alex): the drone sits on top with its pack hanging below it in the holder. A pincer rides a vertical rail (the X
axis of the swap). It rises under the pack, pinches the shell's two clip pads (that one squeeze unlatches the shell AND
grips it), moves down (pulls the XT60 apart), flips the shell 180 deg and moves down again into a bay holder that is the
same print turned upside down (opens up). Releasing the pinch lets the clips snap under the bay holder's teeth.

Frame (one frame for shell, holder and pincer, drone pose, shell seated), mm:
  x  across the pack = the pinch axis = the flip axis direction
  y  along the pack, +y = XT60 (lead) end
  z  up = toward the drone = the install axis; z 0 = shell top face = holder deck underside
Pack: Zeee 4S 5200 120C hard case 138 x 47 x 48 (Zeee listing, +-2, E: caliper yours and set PACK_*), 470 g (listing, E).
Tags in comments: C CAD, E estimate, listing = maker's page.

python3 xswap.py            -> builds every part, runs the fit / motion checks, writes runs/xswap/v1/
"""
import json, math, os, sys
import numpy as np, trimesh, manifold3d as mf

M = mf.Manifold
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.join(ROOT, "runs", "xswap", "v1")
SEG = 48

# ============================================================================== helpers
def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl(axis, a, b, r, l0, l1, seg=SEG):
    """cylinder along axis ('x','y','z'), centred at (a, b) in the other two axes (in order), from l0 to l1."""
    c = M.cylinder(l1 - l0, r, r, seg)          # along z, 0..h
    if axis == "z": return c.translate([a, b, l0])
    if axis == "x": return c.rotate([0, 90, 0]).translate([l0, a, b])   # z -> x
    if axis == "y": return c.rotate([-90, 0, 0]).translate([a, l0, b])  # z -> y

def hull(pts):
    return M.hull_points([list(map(float, p)) for p in pts])

def U(*ms):
    ms = [m for m in ms if m is not None]
    return M.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else ms[0]

def tm(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)

def rot_x180(m, yc, zc):
    """180 deg about the line x-axis through (y=yc, z=zc): y -> 2yc - y, z -> 2zc - z."""
    return m.translate([0, -yc, -zc]).rotate([180, 0, 0]).translate([0, yc, zc])

def overlap(a, b):
    return (a ^ b).volume()

# ============================================================================== numbers
# pack (E: listing 138 x 47 x 48 +-2)
PACK_L, PACK_W, PACK_H, PACK_G = 138.0, 47.0, 48.0, 470.0
CLR = 0.75                      # per side around the pack (C); foam tape takes the listing's +-2
WALL, DECK, FLOOR = 2.4, 3.0, 2.5
CAV_X = PACK_W / 2 + CLR                         # 24.25
CAV_Y0, CAV_Y1 = -(PACK_L / 2 + CLR), PACK_L / 2 + CLR   # -69.75 .. 69.75
CAV_Z1 = -DECK                                   # -3.0
CAV_Z0 = CAV_Z1 - (PACK_H + 2 * CLR)             # -52.5
SX = CAV_X + WALL                                # 26.65 shell side face
SZ0 = CAV_Z0 - FLOOR                             # -55.0 shell bottom
SY0 = -76.0                                      # tail end (open; 2 cross bolts hold the pack)
TAIL_BOLT_Y, TAIL_BOLT_Z = -72.0, (-20.0, -42.0)
Y_C = 3.9                                        # clip / pinch line y = swap CG (C, swap_cg)
Y_X = 80.0                                       # XT60 centre (y)
HEAD_Y1 = 86.6                                   # XT60 block +y face
SY1 = HEAD_Y1 + WALL                             # 89.0 head end wall outer face

# XT60 (bought: repo nox/box/battery_latch.py v4 numbers, spec / E)
XF_W, XF_D, XF_H = 15.5, 8.1, 16.0               # female housing (pack side) x, y, z
XF_TOP = 0.5                                     # female face 0.5 proud of the shell top (C): 6.0 engaged at seat
XM_FW, XM_FD, XM_FT = 27.1, 12.1, 3.0            # XT60E1-M flange x, y, thickness
XM_IW, XM_ID, XM_IL = 13.3, 5.9, 7.5             # male insert (below the flange)
XM_RW, XM_RD, XM_RL = 15.5, 8.1, 6.2             # male rear (above the flange)
XM_FU = 2.0                                      # flange underside (holder floor 0..2)
ENGAGE = XF_TOP - (XM_FU - XM_IL)                # 6.0 (C)

# clip beam (on the shell, both sides, at y = Y_C)
GAP_B, T_B, W_B = 4.2, 2.2, 16.0                 # beam gap to the wall, thickness, width (y)
BX0 = SX + GAP_B                                 # 29.65 beam inner face
BX1 = BX0 + T_B                                  # 32.05 beam outer face
B_ROOT_Z = -2.5                                  # root block z 0..-2.5, beam free below
B_TIP_Z = -40.0
PAD_Z0, PAD_Z1, PAD_T = -39.0, -29.0, 2.0        # finger pad (outer face BX1 + PAD_T = 34.05)
Z_P = -34.0                                      # pinch line = pad centre = flip axis height
POC_Y, POC_Z, POC_D = 6.0, 3.5, 1.5              # finger pocket half sizes at the face, depth
POC_YA, POC_ZA = 45.0, 20.0                      # pocket wall angles from the pinch axis (y walls centre, z walls carry)
NOSE_P = 1.3                                     # latch nose protrusion past BX1
NOSE_Z0, NOSE_Z1 = -24.0, -21.0                  # nose catch face (bottom) and ramp top
PINCH = 3.1                                      # pad travel at the pocket floor, set by the servo (C); hard stop: the
                                                 # beam's lower edge meets the wall at ~3.4 (GAP_B / 1.24)

# holder
HC = 0.3                                         # holder clearance to the shell (C)
SK_T, SK_D = 3.0, 12.0                           # skirt thickness, depth
LEAD_H, LEAD_V = 2.0, 3.46                       # 30 deg lead-in on the skirt's inner rim
DK_T = 10.0                                      # deck: 3.0 plate + open rib grid to z 10; rib tops = flat mount face
SKIN, RIB = 3.0, 2.4
ARM_IN = BX1 + NOSE_P + 0.4                      # 33.75 arm pocket inner face (nose clears 0.4)
ARM_T = 4.0
TOOTH_TIP = BX1 + 0.3                            # 32.35 tooth tip (0.3 off the beam): engagement NOSE_P - 0.3 = 1.0
TOOTH_Z1 = NOSE_Z0 - 0.2                         # -24.2 catch face (0.2 drop from deck to catch)
TOOTH_Z0 = TOOTH_Z1 - 3.0                        # -27.2 arm bottom
MOUNT = [(-20.0, -58.0), (20.0, -58.0), (-20.0, 58.0), (20.0, 58.0)]   # M3 heat-set inserts from the mount face (C)
INS_D, INS_L = 4.0, 7.0                          # M3 x 5.7 heat-set insert hole (E, check your inserts)

# pincer
M_G, Z_G = 1.5, 18                               # gear module, pinion teeth
R_P = M_G * Z_G / 2                              # 10.5 pitch radius
NOSE_H = 3.0                                     # finger nose height off the blade face (C): only the nose touches
FACE_SHUT = BX1 + PAD_T - POC_D - PINCH + NOSE_H  # 33.15 blade face, shut: nose tip on the pocket floor
FACE_OPEN = math.ceil(BX1 + PAD_T + 2.5 + NOSE_H) # 41: nose tip 2.6 clear of the pad face when open
STROKE = FACE_OPEN - FACE_SHUT                   # 6.65 per finger
F_T = 6.0                                        # finger blade thickness (x)
RK_Z0, RK_Z1 = -70.0, -60.0                      # rack z
JB_TOP = -57.5                                   # jaw body top (2.5 under the shell bottom)
RK_L = 50.0
RISER_X0, RISER_T = 49.0, 12.0                   # flip risers both sides, clear of the open finger (x 47)
HUB_X1 = RISER_X0 + RISER_T                      # 61
HUB_L, HUB_R = 16.0, 13.0                        # a hub on each riser along x at (Y_C, Z_P), 8 mm D stub shafts
HUB_X0 = HUB_X1 + HUB_L                          # 77 (outer face)
SHAFT_R, SHAFT_FLAT = 4.1, 3.5

# ============================================================================== shell
def shell_body(yc):
    """battery shell: closed tube round the pack (top deck, sides, floor), open tail with 2 M3 cross bolts,
    XT60 block at the lead end, two clip beams at y = yc."""
    outer = box(-SX, SX, SY0, SY1, SZ0, 0.0)
    cav = box(-CAV_X, CAV_X, SY0 - 1, CAV_Y1, CAV_Z0, CAV_Z1)                    # pack slides in from the tail
    head = box(-CAV_X, CAV_X, CAV_Y1 - 0.01, HEAD_Y1, SZ0 - 1, CAV_Z1)          # wire bay, open below
    s = outer - cav - head
    # XT60 block hanging from the deck (pocket + keeper inserts)
    blk = box(-20.0, 20.0, Y_X - 6.6, HEAD_Y1, -18.5 + 3.0, 0.0)               # z -15.5 .. 0
    s = s + blk
    s = s - box(-(XF_W + 0.3) / 2, (XF_W + 0.3) / 2, Y_X - (XF_D + 0.3) / 2, Y_X + (XF_D + 0.3) / 2, -15.5 - 1, 1)
    for sx in (-1, 1):
        s = s - cyl("z", sx * 15.0, Y_X, INS_D / 2, -15.5 - 1, -15.5 + 6.5)    # keeper inserts from below
    # tail cross bolts: bosses outside the walls (below the skirt) and through holes
    for z in TAIL_BOLT_Z:
        for sx in (-1, 1):
            s = s + cyl("x", TAIL_BOLT_Y, z, 4.5, sx * SX - (2.0 if sx < 0 else 0), sx * SX + (2.0 if sx > 0 else 0))
        s = s - cyl("x", TAIL_BOLT_Y, z, 1.7, -SX - 5, SX + 5)
    s = s - shell_windows(yc)
    # clip beams
    for sx in (-1, 1):
        s = s + beam(yc, sx, 0.0)
    return s

def hexwin(axis_n, y0, y1, a0, a1, n0, n1):
    """window through a wall whose normal is axis_n ('x' or 'z'); y0..y1 along y (pointed 45 deg ends), a0..a1 across."""
    h = (a1 - a0) / 2; ac = (a0 + a1) / 2
    pts2 = [(y0, ac), (y0 + h, a1), (y1 - h, a1), (y1, ac), (y1 - h, a0), (y0 + h, a0)]
    if axis_n == "x": pts = [[n, y, a] for y, a in pts2 for n in (n0, n1)]
    else: pts = [[a, y, n] for y, a in pts2 for n in (n0, n1)]
    return hull(pts)

WIN_Y = [(-66.0, -18.0), (24.0, 66.0)]              # clear of the clip zone (Y_C +-12) and the tail bosses
def shell_windows(yc):
    w = []
    for y0, y1 in WIN_Y:
        w.append(hexwin("x", y0, y1, -48.0, -15.0, -SX - 1, SX + 1))       # both side walls, under the skirt band
        w.append(hexwin("z", y0, y1, -17.0, 17.0, SZ0 - 1, CAV_Z0 + 0.5))  # floor
        w.append(hexwin("z", y0, y1, -16.0, 16.0, CAV_Z1 - 0.5, 1.0))      # deck (rim + head bear on the holder)
    return U(*w)

def beam(yc, sx, d_pad):
    """one clip beam (+x drawn, mirrored for sx -1). d_pad = inward pad deflection (0 relaxed, PINCH squeezed):
    the beam is cut in z slices, each moved by the cantilever curve of a point load at the pad (C)."""
    y0, y1 = yc - W_B / 2, yc + W_B / 2
    root = box(SX - 0.01, BX1, y0, y1, B_ROOT_Z, 0.0)
    a = B_ROOT_Z - Z_P                                    # 31.5 root to pad
    def defl(z):
        x = B_ROOT_Z - z                                  # distance below the root
        if x <= 0: return 0.0
        if x <= a: return d_pad * (x * x * (3 * a - x)) / (2 * a ** 3)
        return d_pad * (1 + 1.5 * (x - a) / a)            # straight past the load point
    def piece(z0, z1):
        g = box(BX0, BX1, y0, y1, z0, z1)
        if z1 > PAD_Z0 and z0 < PAD_Z1:                   # pad slice
            p = box(BX1 - 0.01, BX1 + PAD_T, y0, y1, max(z0, PAD_Z0), min(z1, PAD_Z1))
            g = g + p
        if z1 > NOSE_Z0 and z0 < NOSE_Z1:                 # latch nose: flat catch below, 45 deg ramp on top
            n = hull([[BX1 - 0.01, y, NOSE_Z0] for y in (y0, y1)] + [[BX1 + NOSE_P, y, NOSE_Z0] for y in (y0, y1)] +
                     [[BX1 + NOSE_P, y, NOSE_Z0 + 0.4] for y in (y0, y1)] + [[BX1 - 0.01, y, NOSE_Z1] for y in (y0, y1)])
            g = g + (n ^ box(BX1 - 1, BX1 + 3, y0, y1, z0, z1))
        return g.translate([-defl(0.5 * (z0 + z1)), 0, 0])
    cuts = [B_ROOT_Z, NOSE_Z1, NOSE_Z0, PAD_Z1, PAD_Z0, B_TIP_Z]
    zs = sorted(set([B_ROOT_Z - i * (B_ROOT_Z - B_TIP_Z) / 16 for i in range(17)] + cuts), reverse=True)
    parts = [root] + [piece(zs[i + 1], zs[i]) for i in range(len(zs) - 1)]
    b = U(*parts)
    # finger pocket in the pad (moves with the pad)
    b = b - pocket(yc).translate([-defl(Z_P), 0, 0])
    return b if sx > 0 else b.mirror([1, 0, 0])

def pocket(yc, shrink=0.0, depth=POC_D, extra=1.0):
    """the pad's finger pocket (+x side, relaxed): opening POC_Y x POC_Z at the pad face, floor at POC_D."""
    xf = BX1 + PAD_T
    hy0, hz0 = POC_Y - shrink, POC_Z - shrink
    hy1 = hy0 - depth * math.tan(math.radians(POC_YA)); hz1 = hz0 - depth * math.tan(math.radians(POC_ZA))
    pts = [[xf, yc + sy * hy0, Z_P + sz * hz0] for sy in (-1, 1) for sz in (-1, 1)] + \
          [[xf - depth, yc + sy * hy1, Z_P + sz * hz1] for sy in (-1, 1) for sz in (-1, 1)]
    mouth = box(xf - 0.01, xf + extra, yc - hy0, yc + hy0, Z_P - hz0, Z_P + hz0)   # straight run-out past the face (cut only)
    return hull(pts) + mouth

def shell(yc, d_pad=0.0):
    s = shell_body(yc)
    if d_pad:
        # rebuild with squeezed beams: body without beams + squeezed beams
        s = shell_body_nobeam(yc) + beam(yc, 1, d_pad) + beam(yc, -1, d_pad)
    return s

def shell_body_nobeam(yc):
    full = shell_body(yc)
    keep = full - U(*[box(BX0 - 0.02, BX1 + PAD_T + 2, yc - W_B / 2 - 0.01, yc + W_B / 2 + 0.01, B_TIP_Z - 1, B_ROOT_Z - 0.001)
                     .mirror([1, 0, 0]) if sx < 0 else
                     box(BX0 - 0.02, BX1 + PAD_T + 2, yc - W_B / 2 - 0.01, yc + W_B / 2 + 0.01, B_TIP_Z - 1, B_ROOT_Z - 0.001)
                     for sx in (-1, 1)])
    return keep

def xt60_keeper():
    k = box(-20.0, 20.0, Y_X - 6.6, HEAD_Y1, -18.5, -15.5)
    k = k - box(-6.2, 6.2, Y_X - 2.8, Y_X + 2.8, -20, -10)                        # solder cups / wires
    for sx in (-1, 1):
        k = k - cyl("z", sx * 15.0, Y_X, 1.7, -20, -10)
    return k

def xt60_female():
    return box(-XF_W / 2, XF_W / 2, Y_X - XF_D / 2, Y_X + XF_D / 2, XF_TOP - XF_H, XF_TOP) - \
           box(-XM_IW / 2 - 0.2, XM_IW / 2 + 0.2, Y_X - XM_ID / 2 - 0.2, Y_X + XM_ID / 2 + 0.2, XF_TOP - XM_IL - 1.5, XF_TOP + 1)

def pack():
    return box(-PACK_W / 2, PACK_W / 2, -PACK_L / 2, PACK_L / 2, CAV_Z0 + CLR, CAV_Z1 - CLR)

# ============================================================================== holder
def holder(yc):
    """battery holder: one print for the drone (opens down) and every bay (same print, upside down).
    Sandwich deck z 0..12 (flat top = mount face), skirt with 30 deg lead-ins, two latch arms with teeth at y = yc,
    XT60E1-M flange pocket under a screwed cap, 4 M3 heat-set inserts in the mount face."""
    ix, iy0, iy1 = SX + HC, SY0 - HC, SY1 + HC
    ox, oy0, oy1 = ix + SK_T, iy0 - SK_T, iy1 + SK_T
    deck = box(-ox, ox, oy0, oy1, 0.0, DK_T)
    # sandwich cells (closed: skins SKIN, ribs RIB), solid round the XT60 and the bosses
    cells = []
    nx, ny = 2, 7
    xs = np.linspace(-ox + SK_T, ox - SK_T, nx + 1); ys = np.linspace(oy0 + SK_T, oy1 - SK_T, ny + 1)
    for i in range(nx):
        for j in range(ny):
            cells.append(box(xs[i] + RIB / 2, xs[i + 1] - RIB / 2, ys[j] + RIB / 2, ys[j + 1] - RIB / 2, SKIN, DK_T + 1))
    solid = U(box(-24.5, 24.5, Y_X - 9.5, Y_X + 9.5, -1, DK_T + 1),
              *[cyl("z", x, y, 6.5, -1, DK_T + 1) for x, y in MOUNT])
    deck = deck - (U(*cells) - solid)
    skirt = box(-ox, ox, oy0, oy1, -SK_D, 0.0) - box(-ix, ix, iy0, iy1, -SK_D - 1, 0.1)
    lead = U(*[hull([[sx * ix, y, -SK_D + LEAD_V] for y in (oy0 - 1, oy1 + 1)] + [[sx * (ix + LEAD_H), y, -SK_D - 0.01] for y in (oy0 - 1, oy1 + 1)] +
                    [[sx * (ix - 0.01), y, -SK_D - 0.01] for y in (oy0 - 1, oy1 + 1)]) for sx in (-1, 1)] +
              [hull([[x, sy_v, -SK_D + LEAD_V] for x in (-ox - 1, ox + 1)] + [[x, sy_v + (LEAD_H if sy_v > 0 else -LEAD_H), -SK_D - 0.01] for x in (-ox - 1, ox + 1)] +
                    [[x, sy_v - (0.01 if sy_v > 0 else -0.01), -SK_D - 0.01] for x in (-ox - 1, ox + 1)]) for sy_v in (iy0, iy1)])
    skirt = skirt - lead
    h = deck + skirt
    # latch arms (+x drawn, mirrored)
    for sx in (-1, 1):
        a = latch_arm(yc)
        h = (h - arm_clear(yc)) + a if sx > 0 else (h - arm_clear(yc).mirror([1, 0, 0])) + a.mirror([1, 0, 0])
    # XT60E1-M: floor hole (female housing clearance), flange pocket from the top, cap inserts
    h = h - box(-(XF_W + 0.5) / 2, (XF_W + 0.5) / 2, Y_X - (XF_D + 0.5) / 2, Y_X + (XF_D + 0.5) / 2, -1, XM_FU + 0.01)
    h = h - box(-(XM_FW + 0.6) / 2, (XM_FW + 0.6) / 2, Y_X - (XM_FD + 0.6) / 2, Y_X + (XM_FD + 0.6) / 2, XM_FU, DK_T + 1)
    for sx in (-1, 1):
        h = h - cyl("z", sx * 19.0, Y_X, INS_D / 2, DK_T - 6.5, DK_T + 1)
    for x, y in MOUNT:
        h = h - cyl("z", x, y, INS_D / 2, DK_T - INS_L, DK_T + 1)
    return h

def arm_clear(yc):
    """room in the skirt for the beam root and the beam (+x)."""
    return box(SX - 0.5, ARM_IN, yc - W_B / 2 - 0.5, yc + W_B / 2 + 0.5, TOOTH_Z0 - 1, -0.001) + \
           box(SX + HC, ARM_IN, yc - W_B / 2 - 0.5, yc + W_B / 2 + 0.5, TOOTH_Z0 - 1, 0.35)     # root block clearance into the deck skin

def latch_arm(yc):
    hy = W_B / 2 + 0.5
    outer = box(SX + HC, ARM_IN + ARM_T, yc - hy - ARM_T, yc + hy + ARM_T, TOOTH_Z0, DK_T)
    inner = box(SX - 1, ARM_IN, yc - hy, yc + hy, TOOTH_Z1, 0.35)
    a = outer - inner
    a = a - box(SX - 1, ARM_IN, yc - hy, yc + hy, TOOTH_Z0 - 1, TOOTH_Z1) + box(TOOTH_TIP, ARM_IN + 0.01, yc - hy, yc + hy, TOOTH_Z0, TOOTH_Z1)
    return a

def xt60_cap():
    c = box(-24.5, 24.5, Y_X - 9.5, Y_X + 9.5, DK_T, DK_T + 3.0)
    c = c + box(-(XM_FW + 0.3) / 2, (XM_FW + 0.3) / 2, Y_X - (XM_FD + 0.3) / 2, Y_X + (XM_FD + 0.3) / 2, XM_FU + XM_FT, DK_T + 0.01)
    c = c - box(-(XM_RW + 0.6) / 2, (XM_RW + 0.6) / 2, Y_X - (XM_RD + 0.6) / 2, Y_X + (XM_RD + 0.6) / 2, 0, DK_T + 5)
    for sx in (-1, 1):
        c = c - cyl("z", sx * 19.0, Y_X, 1.7, 0, DK_T + 5)                     # M3 pan / socket head on the plate
    return c

def xt60_male():
    f = box(-XM_FW / 2, XM_FW / 2, Y_X - XM_FD / 2, Y_X + XM_FD / 2, XM_FU, XM_FU + XM_FT)
    i = box(-XM_IW / 2, XM_IW / 2, Y_X - XM_ID / 2, Y_X + XM_ID / 2, XM_FU - XM_IL, XM_FU)
    r = box(-XM_RW / 2, XM_RW / 2, Y_X - XM_RD / 2, Y_X + XM_RD / 2, XM_FU + XM_FT, XM_FU + XM_FT + XM_RL)
    return f + i + r

# ============================================================================== pincer
def involute_gear_2d(m, z, backlash=0.3, pa=20.0, seg=8):
    rp = m * z / 2; rb = rp * math.cos(math.radians(pa)); ra = rp + m; rf = rp - 1.25 * m
    def inv(a): return math.tan(a) - a
    a_p = math.acos(rb / rp)
    half = math.pi / (2 * z) - backlash / (2 * rp) + inv(a_p)       # half tooth angle at the base circle reference
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        side = []
        for t in np.linspace(0, 1, seg):
            r = max(rb, rf) + t * (ra - max(rb, rf))
            a = math.acos(min(1.0, rb / r)); th = half - inv(a)
            side.append((r, th))
        flank_l = [(r * math.cos(c - th), r * math.sin(c - th)) for r, th in side]
        flank_r = [(r * math.cos(c + th), r * math.sin(c + th)) for r, th in reversed(side)]
        half_gap = math.pi / z
        pts += [(rf * math.cos(c - half_gap), rf * math.sin(c - half_gap))]
        pts += [(rf * math.cos(c - side[0][1]), rf * math.sin(c - side[0][1]))] + flank_l + flank_r + \
               [(rf * math.cos(c + side[0][1]), rf * math.sin(c + side[0][1]))]
    return mf.CrossSection([pts])

def pinion(yc):
    g = involute_gear_2d(M_G, Z_G).extrude(RK_Z1 - RK_Z0).translate([0, yc, RK_Z0])
    g = g + involute_gear_2d(M_G, Z_G).extrude(0.31).translate([0, yc, RK_Z0 - 0.3])     # teeth run 0.3 under the racks
    collar = cyl("z", 0, yc, 14.5, RK_Z0 - 2.8, RK_Z0 - 0.29)                       # 0.3 under the racks
    p = g + collar
    p = p - cyl("z", 0, yc, 10.9, RK_Z0 - 3.3, RK_Z0 - 0.3)                       # round servo horn recess (E, CHECK Ø21.5)
    p = p - cyl("z", 0, yc, 1.7, RK_Z0 - 3, RK_Z1 + 1) - cyl("z", 0, yc, 3.3, RK_Z1 - 3.0, RK_Z1 + 1)
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        p = p - cyl("z", 7.75 * math.cos(a), yc + 7.75 * math.sin(a), 1.0, RK_Z0 - 3, RK_Z1 + 1, 16)   # horn screws (E)
    return p

def rack_2d(length, m, backlash=0.3, pa=20.0, shift=0.0):
    """rack teeth along x, pitch line at y 0, tips toward -y (addendum m), body to +y."""
    p = math.pi * m; t = math.tan(math.radians(pa))
    n = int(length / p) + 2
    hw_tip = p / 4 - backlash / 2 - m * t; hw_root = p / 4 - backlash / 2 + 1.25 * m * t
    seq = [(-p, 1.25 * m)]
    for k in range(-1, n):
        c = k * p + p / 2 + shift
        seq += [(c - hw_root, 1.25 * m), (c - hw_tip, -m), (c + hw_tip, -m), (c + hw_root, 1.25 * m)]
    top = 1.25 * m + 7.0
    seq += [(n * p, 1.25 * m), (n * p, top), (-p, top)]
    cs = mf.CrossSection([seq])
    return cs ^ mf.CrossSection.square([length, top + m + 1]).translate([0, -m - 0.5])

def finger(yc, face):
    """+x finger at finger-face x = face: blade with the shaped nose, rack along x at y yc + R_P (teeth toward the pinion).
    The -x finger is the same print turned 180 deg about z through (0, yc)."""
    y_lo = yc - W_B / 2
    rk_y0 = yc + R_P - M_G
    rk_y1 = yc + R_P + 1.25 * M_G + 7.0
    blade = box(face, face + F_T, y_lo, rk_y1, RK_Z0, PAD_Z1)
    blade = blade - box(face - 1, face + F_T + 1, yc + W_B / 2, rk_y1 + 1, PAD_Z1 - 14, PAD_Z1 + 1)    # narrow top (beside the pad only)
    # shaped nose: frustum that seats in the pad pocket (0.2 per side under it, 1.3 tall vs the 1.5 pocket)
    hy0, hz0 = POC_Y - 0.25, POC_Z - 0.5; d = POC_D              # frustum part in the pocket: 0.25 a side in y, 0.5 in z
                                                                  # (the pad tilts ~8 deg as the beam bends)
    hy1 = hy0 - d * math.tan(math.radians(POC_YA)); hz1 = hz0 - d * math.tan(math.radians(POC_ZA))
    tip = face - NOSE_H
    nose = hull([[tip + d, yc + sy * hy0, Z_P + sz * hz0] for sy in (-1, 1) for sz in (-1, 1)] +
                [[tip, yc + sy * hy1, Z_P + sz * hz1] for sy in (-1, 1) for sz in (-1, 1)])
    nose = nose + box(tip + d - 0.01, face + 0.01, yc - hy0, yc + hy0, Z_P - hz0, Z_P + hz0)   # neck to the blade
    p = math.pi * M_G
    x_start_shut = FACE_SHUT + F_T - RK_L
    shift = (-(x_start_shut + p / 2)) % p                # rack tooth at x 0 when shut (pinion gap faces +-y at angle 0)
    rack = rack_2d(RK_L, M_G, shift=shift).extrude(RK_Z1 - RK_Z0)      # x 0..L, y -m..(1.25m+7), z 0..10
    rack = rack.translate([face + F_T - RK_L, yc + R_P, RK_Z0])
    return blade + nose + rack

def finger_pair(yc, face):
    f = finger(yc, face)
    g = f.translate([0, -yc, 0]).rotate([0, 0, 180]).translate([0, yc, 0])
    return f, g

# servo (MG996R / DS3218 class, E: CHECK yours): case 40.7 (y) x 19.7 (x) x 37 (z), ears 54.6 x 2.5 at 9.5 below the case
SV_L, SV_W, SV_H = 40.7, 19.7, 37.0
SV_EAR_L, SV_EAR_T, SV_EAR_DZ, SV_HOLE = 54.6, 2.5, 9.5, 24.75
SV_SHAFT = 10.2                                  # shaft centre to the near (+y) case end (E)
SV_TOP = RK_Z0 - 2.8 - 3.5                       # case top -76 (horn 2.5 in the collar recess, spline boss 3.5)

def jaw_body(yc):
    sy_c = yc - (SV_L / 2 - SV_SHAFT)                # servo case centre y
    y_lo, y_hi = sy_c - SV_EAR_L / 2 - 4.0, yc + R_P + 1.25 * M_G + 7.3 + 3.5
    zb = SV_TOP - SV_H                               # -113
    b = box(-30.5, 30.5, y_lo, y_hi, zb, JB_TOP)
    # flip yoke: a low beam under both fingers, a 12 thick riser and a hub on each side (a fork on the rail carriage
    # holds both stub shafts in bearings, so the swap loads pass straight through and the shafts see shear only)
    for sx in (-1, 1):
        base = box(28.0, HUB_X1, yc - 20, yc + 20, zb, RK_Z0 - 4.0)
        riser = box(RISER_X0, HUB_X1, yc - 20, yc + 20, RK_Z0 - 4.0, Z_P + HUB_R)
        hub = cyl("x", yc, Z_P, HUB_R, HUB_X1 - 1, HUB_X0)
        core = box(32.0, RISER_X0 - 4.0, yc - 15, yc + 15, zb - 1, RK_Z0 - 10.0)
        y = U(base, riser, hub) - core
        b = b + (y if sx > 0 else y.mirror([1, 0, 0]))
    for sx in (-1, 1):
        bore = cyl("x", yc, Z_P, SHAFT_R, HUB_X1 - 4, HUB_X0 + 1) - box(HUB_X1 - 5, HUB_X0 + 2, yc - 5, yc + 5, Z_P + SHAFT_FLAT + 0.1, Z_P + 6)
        sets = U(*[cyl("z", x, yc, INS_D / 2, Z_P + HUB_R - 5.7, Z_P + HUB_R + 1) + cyl("z", x, yc, 1.7, Z_P, Z_P + HUB_R)
                   for x in (HUB_X1 + 5, HUB_X1 + 11)])
        c = bore + sets
        b = b - (c if sx > 0 else c.mirror([1, 0, 0]))
    # cores beside the servo (open below)
    for sx in (-1, 1):
        b = b - box(sx * 14.5 - (11.5 if sx < 0 else 0), sx * 14.5 + (11.5 if sx > 0 else 0), y_lo + 5, y_hi - 5, zb - 1, RK_Z0 - 8.0)
    # rack tunnels (0.3 clear) and the pinion cavity
    rk0, rk1 = R_P - M_G - 0.3, R_P + 1.25 * M_G + 7.0 + 0.3
    b = b - box(-22.0, 31.0, yc + rk0, yc + rk1, RK_Z0 - 0.3, RK_Z1 + 0.3)
    b = b - box(-31.0, 22.0, yc - rk1, yc - rk0, RK_Z0 - 0.3, RK_Z1 + 0.3)
    b = b - cyl("z", 0, yc, R_P + M_G + 0.6, RK_Z0 - 0.3, JB_TOP + 1) - cyl("z", 0, yc, 15.1, RK_Z0 - 3.3, RK_Z0 + 0.01)   # pinion well, open on top
    # servo: case pocket (+0.25), spline boss hole, ear slot from below, ear inserts
    ear_top = SV_TOP - SV_EAR_DZ
    b = b - box(-SV_W / 2 - 0.25, SV_W / 2 + 0.25, sy_c - SV_L / 2 - 0.25, sy_c + SV_L / 2 + 0.25, zb - 1, SV_TOP + 0.01)
    b = b - cyl("z", 0, yc, 7.0, SV_TOP - 0.1, RK_Z0 - 3.2)
    b = b - box(-SV_W / 2 - 0.25, SV_W / 2 + 0.25, sy_c - SV_EAR_L / 2 - 0.5, sy_c + SV_EAR_L / 2 + 0.5, zb - 1, ear_top)
    for sy in (-1, 1):
        for sx in (-1, 1):
            b = b - cyl("z", sx * 5.0, sy_c + sy * SV_HOLE, INS_D / 2, ear_top - 0.01, ear_top + 6.5)
    return b

def servo_dummy(yc):
    sy_c = yc - (SV_L / 2 - SV_SHAFT)
    s = box(-SV_W / 2, SV_W / 2, sy_c - SV_L / 2, sy_c + SV_L / 2, SV_TOP - SV_H, SV_TOP)
    s = s + box(-SV_W / 2, SV_W / 2, sy_c - SV_EAR_L / 2, sy_c + SV_EAR_L / 2, SV_TOP - SV_EAR_DZ - SV_EAR_T, SV_TOP - SV_EAR_DZ)
    s = s + cyl("z", 0, yc, 6.5, SV_TOP, RK_Z0 - 2.8) + cyl("z", 0, yc, 10.75, RK_Z0 - 2.8, RK_Z0 - 0.3)
    return s

def pincer(yc, face):
    f, g = finger_pair(yc, face)
    return dict(jaw=jaw_body(yc), finger_p=f, finger_m=g, pinion=pinion(yc), servo=servo_dummy(yc))

# ============================================================================== mass / CG
PETG = 1.27e-3   # g / mm3 solid (E; ~0.8 of it at 6 walls + 40 % gyroid, reported as solid)

def swap_cg(yc):
    s = tm(shell(yc)); k = tm(xt60_keeper())
    ms, mk = s.volume * PETG, k.volume * PETG
    parts = [(PACK_G, np.array([0, 0, 0.5 * (CAV_Z0 + CAV_Z1)])),
             (ms, s.center_mass), (mk, k.center_mass),
             (12.0, np.array([0, Y_X, -8.0])),                  # XT60 + 10 cm of 12 AWG (E)
             (6.0, np.array([0, -72.0, -31.0]))]               # 2 M3 x 60 + nuts (E)
    m = sum(p[0] for p in parts)
    cg = sum(p[0] * p[1] for p in parts) / m
    return m, cg, ms, mk
