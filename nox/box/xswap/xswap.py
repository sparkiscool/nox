"""X-axis flip swap v3 (Alex, 26 Oct 2026; v2 = guided capture + crash latch, v3 = cradle pincer): battery shell, battery holder (drone and charging bay, one part), pincer.

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
OUT = os.path.join(ROOT, "runs", "xswap", "v3")
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
PACK_L, PACK_W, PACK_H, PACK_G = 138.0, 47.0, 48.0, 480.0     # Alex 26 Oct: 470-480 g, design at 480
CLR = 0.75                      # per side around the pack (C); foam tape takes the listing's +-2
WALL, DECK, FLOOR = 2.4, 3.0, 3.0                # floor solid 3.0: it carries the pack into the latch in a crash
CAV_X = PACK_W / 2 + CLR                         # 24.25
CAV_Y0, CAV_Y1 = -(PACK_L / 2 + CLR), PACK_L / 2 + CLR   # -69.75 .. 69.75
CAV_Z1 = -DECK                                   # -3.0
CAV_Z0 = CAV_Z1 - (PACK_H + 2 * CLR)             # -52.5
SX = CAV_X + WALL                                # 26.65 shell side face
SZ0 = CAV_Z0 - FLOOR                             # -55.0 shell bottom
SY0 = -76.0                                      # tail end (open; 2 cross bolts hold the pack)
TAIL_BOLT_Y, TAIL_BOLT_Z = -72.0, (-25.0, -44.0)     # below the funnel's inner face (C)
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

# latch v2: a rigid rocker on an M4 pin (one per side at y = Y_C), hook under a lug on the holder, spring-closed.
# Self-locking: the pin sits inboard of the hook contact, so a pull-out load turns the hook further in (C).
PX, PZ = 31.5, -30.0                             # pivot (pin axis along y)
PIN_R = 2.15                                     # M4 clearance
ROCK_HY = 14.0                                   # rocker half width (y)
HOOK_X0, RB_X0, RB_X1 = 33.4, 36.3, 41.3         # hook inner tip, rocker body inner / outer face
HOOK_Z1, HOOK_Z0 = -12.7, -16.7                  # hook top / bottom (catch face: it rests on the holder's ledge)
RB_Z0 = -47.0                                    # rocker body bottom
PAD_X1, PAD_Z0, PAD_Z1 = 43.3, -45.0, -35.0      # pad on the body's outer face
Z_P = -40.0                                      # pad centre (rest)
SLOT_R, SLOT_DX = 3.2, 1.2                       # finger slot: round groove r 3.2, axis 1.2 outside the pad face (2.0 deep)
SLOT_HY = 6.8
TH_OPEN, TH_STOP = 13.0, 15.0                    # rocker opening for the release / hard stop (deg)
EAR_HY0, EAR_HY1 = ROCK_HY + 0.4, ROCK_HY + 6.4  # pin ears on the shell wall
# guide posts (holder) and sockets (shell): one per side, diagonal
GX, G_OFF = 41.5, 52.0                           # post x, y offset from Y_C (+x post at Y_C + 52, -x post at Y_C - 52)
POST_R, POST_TIP_R, POST_Z0, POST_TIP = 5.0, 1.5, -40.0, -49.0
SOCK_R, SOCK_WALL, SOCK_Z1, SOCK_Z0, SOCK_CH = 5.5, 2.5, -24.0, -50.0, 7.2   # 0.5 radial: posts locate, keys take crash loads   # bore, wall, top, bottom, entry chamfer r

# holder
HC = 0.3                                         # holder clearance to the shell (C)
SK_T, SK_D = 3.0, 12.0                           # skirt thickness, depth
LEAD_H, LEAD_V = 2.0, 3.46                       # 30 deg lead-in on the skirt's inner rim
DK_T = 10.0                                      # deck: 3.0 plate + open rib grid to z 10; rib tops = flat mount face
SKIN, RIB = 3.0, 2.4
LUG_X1, LUG_HY = 35.9, 14.5                      # holder lug: ledge outer face, half width
POCKET_X0, POCKET_Z1 = 33.0, -12.3               # hook pocket in the lug (inner face, top)
LEDGE_Z1, LEDGE_Z0 = HOOK_Z0 - 0.3, HOOK_Z0 - 4.3  # ledge top = catch face (0.3 drop) / bottom
SHELF_X1 = 41.5                                  # lug shelf over the rocker top (insert bosses)
FLARE_H, FLARE_W = 6.0, 10.4                     # funnel below the skirt: 6 per side over 10.4 (30 deg)
FLARE_HY = 3.5                                   # at the two ends (the holder must stay under the 180 bed)
MOUNT = [(-20.0, -58.0), (20.0, -58.0), (-20.0, 58.0), (20.0, 58.0)]   # M3 heat-set inserts from the mount face (C)
MOUNT_LUG = [(sx * 35.0, sy * 9.0) for sx in (-1, 1) for sy in (-1, 1)]  # + 4 over the lugs (y rel. Y_C): crash load straight up
INS_D, INS_L = 4.0, 7.0                          # M3 x 5.7 heat-set insert hole (E, check your inserts)

# pincer
M_G, Z_G = 1.5, 18                               # gear module, pinion teeth
R_P = M_G * Z_G / 2                              # 13.5 pitch radius
NOSE_R, NOSE_HY, NECK = 3.0, 6.0, 4.5            # finger nose: round bar r 3 along y, centre 4.5 in from the blade face
def _rk(theta_deg, lx, lz):
    """rocker local point (relative to the pivot) -> world at opening angle theta (rotation about +y)."""
    t = math.radians(theta_deg)
    return PX + lx * math.cos(t) + lz * math.sin(t), PZ - lx * math.sin(t) + lz * math.cos(t)
SLOT_LX = PAD_X1 + SLOT_DX - PX                  # 12.6 slot axis, local x
X_NS, Z_N = _rk(TH_OPEN, SLOT_LX, Z_P - PZ)      # nose centre, shut (41.53, -42.58): the finger line
FACE_SHUT = X_NS + NECK                          # 46.03
FACE_OPEN = 53.0                                 # nose 2.2 clear of the pad face when open
STROKE = FACE_OPEN - FACE_SHUT
F_T = 5.0
JB_TOP = SZ0 - 4.0                               # jaw top 4.0 under the shell (the shell may hang 2.6 in the slot)
RK_Z1 = JB_TOP - 2.5; RK_Z0 = RK_Z1 - 10.0      # rack z
RK_L = 50.0
RISER_X0, RISER_T = FACE_OPEN + F_T + 2.0, 12.0  # flip risers both sides, clear of the open finger
HUB_X1 = RISER_X0 + RISER_T
HUB_L, HUB_R = 14.0, 13.0                        # a hub on each riser along x on the finger line, 8 mm D stub shafts
HUB_X0 = HUB_X1 + HUB_L
Z_AX = Z_N                                       # flip axis = finger line
SHAFT_R, SHAFT_FLAT = 4.1, 3.5

# ============================================================================== shell
def shell_body(yc):
    """battery shell: closed tube round the pack (top deck, sides, floor), open tail with 2 M3 cross bolts, XT60 block
    at the lead end, rocker ears + stops + spring bosses at y = yc, one guide socket per side."""
    outer = box(-SX, SX, SY0, SY1, SZ0, 0.0)
    cav = box(-CAV_X, CAV_X, SY0 - 1, CAV_Y1, CAV_Z0, CAV_Z1)                    # pack slides in from the tail
    head = box(-CAV_X, CAV_X, CAV_Y1 - 0.01, HEAD_Y1, SZ0 - 1, CAV_Z1)          # wire bay, open below
    s = outer - cav - head
    blk = box(-20.0, 20.0, Y_X - 6.6, HEAD_Y1, -18.5 + 3.0, 0.0)               # XT60 block z -15.5 .. 0
    s = s + blk
    s = s - box(-(XF_W + 0.3) / 2, (XF_W + 0.3) / 2, Y_X - (XF_D + 0.3) / 2, Y_X + (XF_D + 0.3) / 2, -15.5 - 1, 1)
    for sx in (-1, 1):
        s = s - cyl("z", sx * 15.0, Y_X, INS_D / 2, -15.5 - 1, -15.5 + 6.5)    # keeper inserts from below
    for z in TAIL_BOLT_Z:
        for sx in (-1, 1):
            s = s + cyl("x", TAIL_BOLT_Y, z, 4.5, sx * SX - (2.0 if sx < 0 else 0), sx * SX + (2.0 if sx > 0 else 0))
        s = s - cyl("x", TAIL_BOLT_Y, z, 1.7, -SX - 5, SX + 5)
    s = s - shell_windows(yc)
    for sx in (-1, 1):
        f = side_features(yc)
        s = s + (f if sx > 0 else f.mirror([1, 0, 0]))
        c = side_cuts(yc)
        s = s - (c if sx > 0 else c.mirror([1, 0, 0]))
    # guide sockets: +x at yc + G_OFF, -x at yc - G_OFF
    sk = socket(yc + G_OFF)
    s = s + sk[0]; s = s - sk[1]
    sk = socket(yc - G_OFF)
    s = s + sk[0].mirror([1, 0, 0]); s = s - sk[1].mirror([1, 0, 0])
    return s

def side_features(yc):
    """+x: pin ears, rest stop, hard stop, spring boss (on the wall)."""
    ears = U(box(SX - 0.01, 37.0, yc + EAR_HY0, yc + EAR_HY1, PZ - 7.0, PZ + 7.0),
             box(SX - 0.01, 37.0, yc - EAR_HY1, yc - EAR_HY0, PZ - 7.0, PZ + 7.0))
    rest = box(SX - 0.01, RB_X0 - 0.05, yc - 8, yc + 8, -24.8, -22.0)
    x_stop = _rk(TH_STOP, RB_X0 - PX, -45.25 - PZ)[0]
    hard = box(SX - 0.01, x_stop, yc - 8, yc + 8, -47.0, -43.5)
    spring = cyl("x", yc, -40.0, 5.0, SX - 0.01, 29.0)
    return U(ears, rest, hard, spring)

def side_cuts(yc):
    pin = cyl("y", PX, PZ, PIN_R, yc - EAR_HY1 - 1, yc + EAR_HY1 + 1)
    spring = cyl("x", yc, -40.0, 2.65, 27.0, 29.5)
    return pin + spring

def socket(gy):
    """+x guide socket at y gy: tube (bore SOCK_R, 30 deg entry cone) on a web from the wall; teardrop roof toward +y
    (the print's up) so the bore prints without support. Returns (add, cut)."""
    ro = SOCK_R + SOCK_WALL
    tube = cyl("z", GX, gy, ro, SOCK_Z0, SOCK_Z1) + box(SX - 0.01, GX, gy - 4.0, gy + 4.0, SOCK_Z0, SOCK_Z1)
    tube = tube + hull([[SX - 0.01, gy + sy * 4.0, SOCK_Z0] for sy in (-1, 1)] + [[SX - 0.01, gy + sy * 4.0, SOCK_Z0 - 4] for sy in (-1, 1)] +
                       [[GX - ro, gy + sy * 4.0, SOCK_Z0] for sy in (-1, 1)])              # gusset under the web
    bore = cyl("z", GX, gy, SOCK_R, SOCK_Z0 - 5, SOCK_Z1 + 1)
    k = SOCK_R / math.sqrt(2)
    bore = bore + hull([[GX + sx * k, gy + k, z] for sx in (-1, 1) for z in (SOCK_Z0 - 5, SOCK_Z1 + 1)] +
                       [[GX, gy + SOCK_R * math.sqrt(2), z] for z in (SOCK_Z0 - 5, SOCK_Z1 + 1)])
    cone = M.cylinder(SOCK_CH - SOCK_R, SOCK_R, SOCK_CH, SEG).translate([GX, gy, SOCK_Z1 - (SOCK_CH - SOCK_R)])
    return tube, bore + cone + cyl("z", GX, gy, SOCK_CH, SOCK_Z1 - 0.01, SOCK_Z1 + 2)

def hexwin(axis_n, y0, y1, a0, a1, n0, n1):
    """window through a wall whose normal is axis_n ('x' or 'z'); y0..y1 along y (pointed 45 deg ends), a0..a1 across."""
    h = (a1 - a0) / 2; ac = (a0 + a1) / 2
    pts2 = [(y0, ac), (y0 + h, a1), (y1 - h, a1), (y1, ac), (y1 - h, a0), (y0 + h, a0)]
    if axis_n == "x": pts = [[n, y, a] for y, a in pts2 for n in (n0, n1)]
    else: pts = [[a, y, n] for y, a in pts2 for n in (n0, n1)]
    return hull(pts)

WIN_FD = [(-66.0, -18.0), (24.0, 66.0)]             # floor and deck windows
WIN_SIDE = [(-38.0, -21.0), (29.0, 45.0)]           # side walls, clear of the ears, sockets and tail bosses
def shell_windows(yc):
    w = []
    for y0, y1 in WIN_SIDE:
        w.append(hexwin("x", y0, y1, -50.0, -16.0, -SX - 1, SX + 1))
    for y0, y1 in WIN_FD:                            # deck only (the keys drop in); the floor stays solid
        w.append(hexwin("z", y0, y1, -16.0, 16.0, CAV_Z1 - 0.5, 1.0))
    return U(*w)

def rocker_local(yc):
    """+x rocker at rest (theta 0), world coords. Hook on top under the holder lug, pivot boss inboard, pad + finger
    slot on the outer face, spring seat on the inner face."""
    hy = ROCK_HY
    body = box(RB_X0, RB_X1, yc - hy, yc + hy, RB_Z0, HOOK_Z1)
    hook = box(HOOK_X0, RB_X0 + 0.01, yc - hy, yc + hy, HOOK_Z0, HOOK_Z1)
    boss = cyl("y", PX, PZ, 4.5, yc - hy, yc + hy)
    web = box(PX, RB_X0 + 0.01, yc - hy, yc + hy, PZ - 4.0, PZ + 4.0)
    pad = box(RB_X1 - 0.01, PAD_X1, yc - hy, yc + hy, PAD_Z0, PAD_Z1)
    r = U(body, hook, boss, web, pad)
    r = r - cyl("y", PX, PZ, PIN_R, yc - hy - 1, yc + hy + 1)
    r = r - cyl("x", yc, -40.0, 2.65, RB_X0 - 1, RB_X0 + 1.5)                                # spring seat
    r = r - finger_slot(yc)
    return r

def finger_slot(yc):
    """round groove along y (r SLOT_R): from where the nose enters at rest (z Z_N) up to the pad centre, where it sits
    when pinched (the nose stays at world z Z_N while the rocker turns, so in the rocker it climbs 2.6 straight up)."""
    x = PAD_X1 + SLOT_DX
    z_top, z_bot = Z_P + 0.3, Z_N - 0.3
    run = SLOT_HY - SLOT_R * 0.5
    core = M.batch_hull([cyl("y", x, z, SLOT_R, yc - run, yc + run) for z in (z_top, z_bot)])
    ends = U(*[M.batch_hull([M.cylinder(SLOT_R * 0.5 + 0.01, SLOT_R, 0.01, SEG).rotate([-90 if sy > 0 else 90, 0, 0])
                             .translate([x, yc + sy * run, z]) for z in (z_top, z_bot)]) for sy in (-1, 1)])
    return core + ends

def rocker(yc, theta=0.0, side=1):
    r = rocker_local(yc).translate([-PX, 0, -PZ]).rotate([0, theta, 0]).translate([PX, 0, PZ])
    return r if side > 0 else r.mirror([1, 0, 0])

def shell(yc, theta=0.0):
    """shell body + both rockers at opening angle theta (0 latched, TH_OPEN pinched)."""
    return shell_body(yc) + rocker(yc, theta, 1) + rocker(yc, theta, -1)

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
    Deck z 0..10 (plate + open rib grid; rib tops = mount face), skirt with lead-ins and a 30 deg funnel below it,
    two lugs (rocker catches) at y = yc, two guide posts, XT60E1-M under a screwed cap, 8 M3 heat-set inserts."""
    ix, iy0, iy1 = SX + HC, SY0 - HC, SY1 + HC
    ox, oy0, oy1 = ix + SK_T, iy0 - SK_T, iy1 + SK_T
    deck = box(-ox, ox, oy0, oy1, 0.0, DK_T)
    cells = []
    nx, ny = 2, 7
    xs = np.linspace(-ox + SK_T, ox - SK_T, nx + 1); ys = np.linspace(oy0 + SK_T, oy1 - SK_T, ny + 1)
    for i in range(nx):
        for j in range(ny):
            cells.append(box(xs[i] + RIB / 2, xs[i + 1] - RIB / 2, ys[j] + RIB / 2, ys[j + 1] - RIB / 2, SKIN, DK_T + 1))
    solid = U(box(-24.5, 24.5, Y_X - 9.5, Y_X + 9.5, -1, DK_T + 1),
              *[cyl("z", x, y, 6.5, -1, DK_T + 1) for x, y in MOUNT],
              box(-ox - 1, ox + 1, yc - LUG_HY, yc + LUG_HY, -1, DK_T + 1))
    deck = deck - (U(*cells) - solid)
    skirt = box(-ox, ox, oy0, oy1, -SK_D, 0.0) - box(-ix, ix, iy0, iy1, -SK_D - 1, 0.1)
    lead = U(*[hull([[sx * ix, y, -SK_D + LEAD_V] for y in (oy0 - 1, oy1 + 1)] + [[sx * (ix + LEAD_H), y, -SK_D - 0.01] for y in (oy0 - 1, oy1 + 1)] +
                    [[sx * (ix - 0.01), y, -SK_D - 0.01] for y in (oy0 - 1, oy1 + 1)]) for sx in (-1, 1)] +
              [hull([[x, sy_v, -SK_D + LEAD_V] for x in (-ox - 1, ox + 1)] + [[x, sy_v + (LEAD_H if sy_v > 0 else -LEAD_H), -SK_D - 0.01] for x in (-ox - 1, ox + 1)] +
                    [[x, sy_v - (0.01 if sy_v > 0 else -0.01), -SK_D - 0.01] for x in (-ox - 1, ox + 1)]) for sy_v in (iy0, iy1)])
    skirt = skirt - lead
    h = deck + skirt + funnel(yc)
    for sx in (-1, 1):
        g = lug(yc)
        h = h + (g if sx > 0 else g.mirror([1, 0, 0]))
    p = post(yc + G_OFF)
    h = h + p + p.translate([0, -(2 * G_OFF), 0]).mirror([1, 0, 0])
    h = h + keys()
    h = h - funnel_core()
    h = h - box(-(XF_W + 0.5) / 2, (XF_W + 0.5) / 2, Y_X - (XF_D + 0.5) / 2, Y_X + (XF_D + 0.5) / 2, -1, XM_FU + 0.01)
    h = h - box(-(XM_FW + 0.6) / 2, (XM_FW + 0.6) / 2, Y_X - (XM_FD + 0.6) / 2, Y_X + (XM_FD + 0.6) / 2, XM_FU, DK_T + 1)
    for sx in (-1, 1):
        h = h - cyl("z", sx * 19.0, Y_X, INS_D / 2, DK_T - 6.5, DK_T + 1)
    for x, y in MOUNT + [(x, yc + y) for x, y in MOUNT_LUG]:
        h = h - cyl("z", x, y, INS_D / 2, DK_T - INS_L, DK_T + 1)
    return h

def funnel_core():
    """the empty cone inside the funnel (z -12 .. -22.4): nothing of the holder may sit in it (lug undersides follow it)."""
    ix, iy0, iy1 = SX + HC, SY0 - HC, SY1 + HC
    z1, z0 = -SK_D + 0.01, -SK_D - FLARE_W
    return hull([[sx * ix, y, z1] for sx in (-1, 1) for y in (iy0, iy1)] +
                [[sx * (ix + FLARE_H), y, z0] for sx in (-1, 1) for y in (iy0 - FLARE_HY, iy1 + FLARE_HY)])

def funnel(yc):
    """30 deg flare below the skirt: 6 per side over 10.4, cut away beside the rockers (|y - yc| < 22, |x| > 20)."""
    ix, iy0, iy1 = SX + HC, SY0 - HC, SY1 + HC
    z1, z0 = -SK_D + 0.01, -SK_D - FLARE_W
    def frust(off):
        return hull([[sx * (ix + off), y, z1] for sx in (-1, 1) for y in (iy0 - off, iy1 + off)] +
                    [[sx * (ix + off + FLARE_H), y, z0] for sx in (-1, 1) for y in (iy0 - off - FLARE_HY, iy1 + off + FLARE_HY)])
    f = frust(SK_T) - frust(0.0)
    f = f - box(20.0, 200, yc - 22.0, yc + 22.0, z0 - 1, z1 + 1) - box(-200, -20.0, yc - 22.0, yc + 22.0, z0 - 1, z1 + 1)
    return f

KEY_C, KEY_D = 0.4, 2.4                          # keys: 0.4 a side in the shell's deck windows, 2.4 deep
def keys():
    """locating keys under the deck: they drop into the shell's two deck windows and take side and end crash loads."""
    k = []
    for y0, y1 in WIN_FD:
        top = hexwin("z", y0 + KEY_C, y1 - KEY_C, -16.0 + KEY_C, 16.0 - KEY_C, -1.6, 0.01)
        tip = hexwin("z", y0 + KEY_C + 0.8, y1 - KEY_C - 0.8, -16.0 + KEY_C + 0.8, 16.0 - KEY_C - 0.8, -KEY_D, -KEY_D + 0.01)
        k.append(M.batch_hull([top, tip]))
    return U(*k)

def lug(yc):
    """+x rocker catch: a solid block from the mount face down to a ledge; the hook drops into a pocket above the ledge
    (pocket open outward), and a shelf runs over the rocker top for the insert bosses."""
    ox = SX + HC + SK_T
    core = box(ox - 0.5, LUG_X1, yc - LUG_HY, yc + LUG_HY, LEDGE_Z0, DK_T)
    core = core - box(POCKET_X0, LUG_X1 + 1, yc - LUG_HY - 1, yc + LUG_HY + 1, LEDGE_Z1, POCKET_Z1)
    shelf = box(LUG_X1 - 0.01, SHELF_X1, yc - LUG_HY, yc + LUG_HY, POCKET_Z1, DK_T)
    return core + shelf

def post(gy):
    """+x guide post at y gy: root pad and fillet on the deck, r 5 shank, bullet tip (r 1.5 at POST_TIP)."""
    ox = SX + HC + SK_T
    pad = box(18.0, GX + POST_R + 2.0, gy - POST_R - 2.0, gy + POST_R + 2.0, 0.0, DK_T)        # runs 12 into the deck
    web = box(SX + HC + 0.5, GX, gy - 4.0, gy + 4.0, -SK_D, 0.01)                            # onto the skirt, full height
    fil = M.cylinder(5.0, POST_R, POST_R + 3.0, SEG).translate([GX, gy, -5.0])
    shank = cyl("z", GX, gy, POST_R, POST_Z0, 0.01)
    tip = M.cylinder(POST_Z0 - POST_TIP, POST_TIP_R, POST_R, SEG).translate([GX, gy, POST_TIP])
    return U(pad, web, fil, shank, tip)

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
    """+x finger at blade-face x = face: blade, rack along x at y yc + R_P (teeth toward the pinion), and the shaped nose:
    a round bar r 3 along y (45 deg ends) on a neck, on the finger line Z_N. It sits in the rocker's round slot, so the
    rocker can turn round it. The -x finger is the same print turned 180 deg about z through (0, yc)."""
    y_lo = yc - 8.0
    rk_y1 = yc + R_P + 1.25 * M_G + 7.0
    top = Z_N + 6.0
    blade = box(face, face + F_T, y_lo, rk_y1, RK_Z0, top)
    blade = blade - box(face - 1, face + F_T + 1, yc + 8.0, rk_y1 + 1, top - 14, top + 1)
    xc = face - NECK
    run = NOSE_HY - 1.5
    bar = cyl("y", xc, Z_N, NOSE_R, yc - run, yc + run)
    ends = U(*[M.cylinder(1.5, NOSE_R, NOSE_R - 1.5, SEG).rotate([-90 if sy > 0 else 90, 0, 0]).translate([xc, yc + sy * run, Z_N])
               for sy in (-1, 1)])
    neck = box(xc, face + 0.01, yc - NOSE_HY + 1.0, yc + NOSE_HY - 1.0, Z_N - 2.3, Z_N + 2.3)
    nose = U(bar, ends, neck)
    p = math.pi * M_G
    x_start_shut = FACE_SHUT + F_T - RK_L
    shift = (-(x_start_shut + p / 2)) % p
    rack = rack_2d(RK_L, M_G, shift=shift).extrude(RK_Z1 - RK_Z0)
    rack = rack.translate([face + F_T - RK_L, yc + R_P, RK_Z0])
    return blade + nose + rack

# cradle (v3 pincer): a lip under the shell floor along its whole length, a box spine under the lip, and end tabs
# past both shell ends. The lips carry the pack's weight and the insert push; the tabs hold it end to end in the flip.
LIP_Z1 = SZ0 - 0.3                               # lip top, 0.3 under the shell floor
LIP_T = 2.5
LIP_Z0 = LIP_Z1 - LIP_T
LIP_IN = 34.0                                    # lip inner edge at face - 34 (x 12.0 shut)
SPINE_IN, SPINE_Z0, SPINE_W = 13.0, -74.0, 3.0   # box spine: inner web at face - 13, bottom at -74 (2 over the jaw base)
TAB_T, TAB_Z1, TAB_X1 = 3.5, -35.0, 19.6         # end tabs: thickness, top, outer edge at face - 19.6 (x 26.4 shut)
TAIL_Y = SY0 - 0.5                               # shell's tail end incl. the cross-bolt bosses (-76.5)
def cradle(yc, face):
    """+x cradle in absolute y (the -x one is its mirror in x)."""
    y0, y1 = TAIL_Y - 0.4 - TAB_T, SY1 + 0.4 + TAB_T
    x_in, x_out = face - LIP_IN, face + F_T
    lip = box(x_in, x_out, y0, y1, LIP_Z0, LIP_Z1)
    spine = U(box(face - SPINE_IN, face - SPINE_IN + SPINE_W, y0, y1, SPINE_Z0, LIP_Z0 + 0.01),
              box(x_out - SPINE_W, x_out, y0, y1, SPINE_Z0, LIP_Z0 + 0.01),
              box(face - SPINE_IN, x_out, y0, y1, SPINE_Z0, SPINE_Z0 + 2.5))
    tabs = U(box(x_in, face - TAB_X1, y0, TAIL_Y - 0.4, LIP_Z0, TAB_Z1),
             box(x_in, face - TAB_X1, SY1 + 0.4, y1, LIP_Z0, TAB_Z1))
    gus = U(*[hull([[x_in, y, LIP_Z1] for y in (ya, yb)] + [[x_in + 10.0, y, LIP_Z1] for y in (ya, yb)] +
                   [[x_in, y, LIP_Z1 + 8.0] for y in (ya, yb)]) for ya, yb in ((y0, TAIL_Y - 0.4), (SY1 + 0.4, y1))])
    # the tab gussets run 10 in along the lip below the shell's end faces only (they sit beyond the shell in y)
    return U(lip, spine, tabs, gus)

def finger_full(yc, face, side=1):
    """finger A (+x) / B (-x): the v2 nose, blade and rack (turned 180 deg about z for -x) + the cradle (mirrored)."""
    core = finger(yc, face)
    if side > 0:
        return core + cradle(yc, face)
    return core.translate([0, -yc, 0]).rotate([0, 0, 180]).translate([0, yc, 0]) + cradle(yc, face).mirror([1, 0, 0])

def finger_pair(yc, face):
    return finger_full(yc, face, 1), finger_full(yc, face, -1)

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
        riser = box(RISER_X0, HUB_X1, yc - 20, yc + 20, RK_Z0 - 4.0, Z_AX + HUB_R)
        hub = cyl("x", yc, Z_AX, HUB_R, HUB_X1 - 1, HUB_X0)
        core = box(32.0, RISER_X0 - 4.0, yc - 15, yc + 15, zb - 1, RK_Z0 - 10.0)
        y = U(base, riser, hub) - core
        b = b + (y if sx > 0 else y.mirror([1, 0, 0]))
    for sx in (-1, 1):
        bore = cyl("x", yc, Z_AX, SHAFT_R, HUB_X1 - 4, HUB_X0 + 1) - box(HUB_X1 - 5, HUB_X0 + 2, yc - 5, yc + 5, Z_AX + SHAFT_FLAT + 0.1, Z_AX + 6)
        sets = U(*[cyl("z", x, yc, INS_D / 2, Z_AX + HUB_R - 5.7, Z_AX + HUB_R + 1) + cyl("z", x, yc, 1.7, Z_AX, Z_AX + HUB_R)
                   for x in (HUB_X1 + 4, HUB_X1 + 10)])
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
