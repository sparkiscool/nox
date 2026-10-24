"""Scout v3 bottom layout v1 (24 Oct 2026): the whole drone laid out to the latest box (hook latch, XT60 swap, R 5 wheels),
before any topology run. Places the pack (CG solve), the cartridge with the pack's own XT60 at the lead end, the receiver
envelope with the XT60E1-M float carrier, the pogo pads, the electronics (placement search), the flow sensor, the camera,
the feet and a rough cage, then checks every keep-out of the box and the funnel.

python3 nox/scout/v3/layout_v1.py [out_dir]          (~2 min; --novideo skips the mp4)

Drone frame: x nose, y left, z up, CG at x 0, plate top z 22 (`nox/dimensions.md` C). Seat frame (S) = dimensions G:
funnel faces max(|x|, |y|) = 84.8 + z_s, bottom edge z_s -0.8, top 75.1. Numbers: `nox/dimensions.md` V (this layout),
A, C, G, I, J, M; every number below is tagged there. Rough shapes: the receiver and the cage are envelopes for the
topology runs that follow, not parts.
"""
import io, json, math, os, sys, itertools
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene
from topo.part import man

OUT = next((a for a in sys.argv[1:] if not a.startswith("--")), os.path.join(ROOT, "runs", "scout_layout", "v1"))
VIDEO = "--novideo" not in sys.argv
S2 = math.sqrt(2.0)
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


# ---------------------------------------------------------------- airframe (A, C; P / listing / E as tagged there)
PLATE_BOT, PLATE_TOP = 18.5, 22.0
MOTOR, PROP_R, PROP_Z, SWEEP = 54.1, 44.45, 41.2, 4.0
BELL_R, BELL_H = 12.4, 15.0                      # EMAX 2004 bell (E, Q6)
SIDE_BOLT, NOSE_BOLT = (3.9, 31.0), (45.0, 0.0)  # plate bolts (E row "Plate bolts")
PLATE_TAIL = -50.0                               # plate tail edge (E, CHECK), XT60 cut-out there
XT_CUT = (10.0, 18.0)                            # plate XT60 cut-out at the tail edge, x depth / y width (E, CHECK on the plate)
FC_C, FC_Z = (1.9, 0.0), (9.5, 17.5)             # FC diamond 36 x 36 x 8, lowest 9.5 (H, CHECK)

# ---------------------------------------------------------------- catch (G): rings, wheels, seat
RING_LO, RING_HI = 33.2, 49.2                    # prop plane +-8 (cage doc C3)
ROD_R = 1.5                                      # 3.0 carbon
WHEEL_R, WHEEL_W = 5.0, 3.0                      # R 5 set (hook latch laid out on it)
FACE_K, FACE_Z0, FACE_Z1 = 84.8, -0.8, 75.1      # S: face |x| = 84.8 + z, bottom edge, top
SEAT_RING = (112 - FACE_K) + 4.0 * S2            # lower-ring rod centre 4.0 off the face at the outline: z_s 32.86 (C)
DZ = SEAT_RING - RING_LO                         # z_s = z + DZ (-0.34, C)
OPEN_HALF = 84.0

# ---------------------------------------------------------------- box under the seat (I, J; C / E)
CART_BOT_S = -44.5                               # cartridge flat bottom at ring R 100 = W Z 100 (G Funnel, E)
CART_W, CART_H = 38.0, 48.0                      # cartridge v3 section (E row Cartridge, C)
PACK = (70.0, 32.0, 33.0)                        # 4S 1500 (P)
LEAD_BAY = 8.0                                   # lead loop from the pack end face into the XT60 cups (DEFAULT, E)
XT_F = (8.1, 15.5, 16.0)                         # XT60 housing x, y, z (M XT60 row: face 15.5 x 8.1; length E)
XT_NOSE = 7.5                                    # engagement: female nose into the male shroud (E, pins touch in the last ~9)
CART_L = 2.0 + 0.5 + PACK[0] + 0.5 + LEAD_BAY + XT_F[0] + 0.4 + 2.0   # 91.5 -> 92 (C)
CART_L = math.ceil(CART_L)
TAB_FROM_CLOSED = 37.5                           # tab line = pack centre from the closed end (J, unchanged)
XTM_FLANGE = (12.1, 27.1, 3.0)                   # XT60E1-M flange (x, y, t) (M, spec)
XTM_SHROUD, XTM_REAR = 9.0, 11.0                 # shroud below / body + cups above the flange (E, Q19)
CARRIER_FLOAT = 1.0                              # +-1.0 lateral, +-2 deg (I XT60 mate)
TPU_STRIP = 1.5                                  # preload strip on the receiver ceiling (J)
CEIL_T = 2.0                                     # receiver ceiling
HEAD_HALF_Y = 47.0                               # head envelope under the seated pack, |y| (E, box-arm.md gripper + servo)
ARM_GAP, ARM_Z_S = 2.5, -2.2                     # arm sweep at the drone (Y_C 118, C2): carriage corner r 138.5 from 2.5 past the
ARM_CORNER_S, ARM_RIM_GAP = -6.0, 7.5            # closed end (z_s -6.0), wheel rims r 141.3 from 7.5 past it (z_s -3.2; kept at -2.2) (C / E)
POGO_TRAVEL = 8.0                                # box pogo pins (I Box electronics: >= 8)
POGO_X, POGO_Y = 10.0, 63.0                      # pads 10 x 10 at (+-10, +-63) in the catch frame (DEFAULT, shared G (6))


# ---------------------------------------------------------------- mesh helpers
def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b


def cyl(r, z0, z1, x=0.0, y=0.0, n=40):
    c = trimesh.creation.cylinder(r, z1 - z0, sections=n); c.apply_translation([x, y, (z0 + z1) / 2]); return c


def rod(p0, p1, r=ROD_R, n=12):
    return trimesh.creation.cylinder(r, segment=[p0, p1], sections=n)


def rotz(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def union(ms):
    out = man(ms[0])
    for m in ms[1:]:
        out = out + man(m)
    mm = out.to_mesh(); return trimesh.Trimesh(mm.vert_properties[:, :3], mm.tri_verts)


def overlap(a, b):
    return (man(a) ^ man(b)).volume()


def pts(m, n=4000, seed=1):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=seed)[0]])


def seg_pts(p0, p1, n=60):
    t = np.linspace(0, 1, n)[:, None]; return np.asarray(p0) * (1 - t) + np.asarray(p1) * t


# ---------------------------------------------------------------- fixed airframe
def plate():
    body = union([box(PLATE_TAIL, 51, -30, 30, PLATE_BOT, PLATE_TOP), box(-12, 20, -47, 47, PLATE_BOT, PLATE_TOP)])
    body = man(body) - man(box(PLATE_TAIL - 1, PLATE_TAIL + XT_CUT[0], -XT_CUT[1] / 2, XT_CUT[1] / 2, PLATE_BOT - 1, PLATE_TOP + 1))
    body = body.to_mesh(); body = trimesh.Trimesh(body.vert_properties[:, :3], body.tri_verts)
    arms = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        a = box(0, MOTOR * S2, -7, 7, PLATE_BOT, PLATE_TOP); a.apply_transform(rotz(math.degrees(math.atan2(sy, sx))))
        arms += [a, cyl(11, PLATE_BOT, PLATE_TOP, sx * MOTOR, sy * MOTOR)]
    return union([body] + arms)


PLATE = plate()
MOTORS_XY = [(sx * MOTOR, sy * MOTOR) for sx in (1, -1) for sy in (1, -1)]
BELLS = [cyl(BELL_R, PLATE_TOP, PLATE_TOP + BELL_H, x, y) for x, y in MOTORS_XY]
FC = union([box(-18, 18, -15, 15, *FC_Z), box(-15, 18 - 3, -18, 18, *FC_Z).apply_translation([1.5, 0, 0]), box(-15, 15, -18, 18, *FC_Z)]
           + [cyl(3.0, *FC_Z, sx * 15, sy * 15) for sx in (1, -1) for sy in (1, -1)])       # 36 x 36, corners r 3 (E)
FC.apply_transform(rotz(45)); FC.apply_translation([FC_C[0], FC_C[1], 0])


def prop_dist(p):
    """distance of points to the swept prop disks (r 44.45, z 41.2 +- 4)"""
    d = np.full(len(p), np.inf)
    for x, y in MOTORS_XY:
        rho = np.hypot(p[:, 0] - x, p[:, 1] - y)
        d = np.minimum(d, np.hypot(np.maximum(rho - PROP_R, 0), np.maximum(np.abs(p[:, 2] - PROP_Z) - SWEEP, 0)))
    return d


# ---------------------------------------------------------------- layout from (x_ce, slot choices)
CART_BOT = CART_BOT_S - DZ                       # drone z of the cartridge bottom (-44.16, C)
CART_TOP = CART_BOT + CART_H                     # 3.84 (C)


def layout(x_ce, radxa="top", ubec="wing-", pogo_side=1):
    """x_ce: cartridge closed end (nose side, toward the swap arm). Returns dict of meshes + metadata."""
    g = {"x_ce": x_ce}
    x0 = x_ce - CART_L                                                # lead end (tail)
    g["cart"] = box(x0, x_ce, -CART_W / 2, CART_W / 2, CART_BOT, CART_TOP)
    px1 = x_ce - 2.5; px0 = px1 - PACK[0]
    g["pack"] = box(px0, px1, -PACK[1] / 2, PACK[1] / 2, CART_BOT + 14, CART_BOT + 14 + PACK[2])
    g["tab_x"] = x_ce - TAB_FROM_CLOSED
    xt = px0 - 0.5 - LEAD_BAY - XT_F[0] / 2                           # XT60 centre along x
    g["xt_x"] = xt
    # pack's own XT60 (female), nose flush with the cartridge top, shoulder XT_NOSE under it (DEFAULT: protected)
    g["xt_f"] = box(xt - XT_F[0] / 2, xt + XT_F[0] / 2, -XT_F[1] / 2, XT_F[1] / 2, CART_TOP - XT_F[2], CART_TOP)
    # drone's XT60E1-M (male) on the float carrier: shroud down into the cartridge recess, flange, rear body + cups up
    zf = CART_TOP - XT_NOSE + XTM_SHROUD                              # flange underside when mated
    g["xt_m"] = union([box(xt - 4.5, xt + 4.5, -8.4, 8.4, CART_TOP - XT_NOSE, zf),
                       box(xt - XTM_FLANGE[0] / 2, xt + XTM_FLANGE[0] / 2, -XTM_FLANGE[1] / 2, XTM_FLANGE[1] / 2, zf, zf + XTM_FLANGE[2]),
                       box(xt - 4.05, xt + 4.05, -7.75, 7.75, zf + XTM_FLANGE[2], zf + XTM_FLANGE[2] + XTM_REAR)])
    g["xt_top"] = zf + XTM_FLANGE[2] + XTM_REAR
    # receiver envelope (topology later): walls round the cartridge with the latch pockets, ceiling, end stops,
    # pads on the 3 plate bolts, the float carrier at the tail, the flow land at the nose
    zc0 = CART_TOP + TPU_STRIP; zc1 = zc0 + CEIL_T
    mouth = CART_BOT + 2.0
    xw1 = x_ce + ARM_GAP - 1.0                                        # walls stop 1.0 short of the arm carriage (C)
    walls = [box(x0 - 3.3, xw1, 20.3, 26.6, mouth, zc1), box(x0 - 3.3, xw1, -26.6, -20.3, mouth, zc1)]
    ceil = box(x0 - 3.3, x_ce + 2.3, -26.6, 26.6, zc0, zc1)
    nose_stop = ARM_CORNER_S + 1.0 - DZ                              # nose end stop above the carriage corner (DEFAULT)
    ends = [box(x_ce + 0.3, x_ce + 2.3, -26.6, 26.6, nose_stop, zc1), box(x0 - 3.3, x0 - 0.3, -26.6, 26.6, CART_TOP - 10, zc1)]
    pads = [cyl(6.0, zc1, PLATE_BOT, SIDE_BOLT[0], s * SIDE_BOLT[1]) for s in (1, -1)] + [cyl(7.5, zc1, PLATE_BOT, *NOSE_BOLT)]
    # float carrier: a plate over the flange (takes the insert push) with the rear body through it; the flange hangs on
    # 2 screws in holes 2.0 over size with TPU washers (I XT60 mate); the ceiling is cut round the shroud
    carrier = box(xt - 9, xt + 9, -17, 17, zf + XTM_FLANGE[2] + 0.5, zf + XTM_FLANGE[2] + 3.5)
    # flow land: MTF-02P on a tongue forward of the closed end, above the arm sweep (z_s >= -2.2 + 1.0)
    fz = ARM_Z_S + 1.0 - DZ
    flow_x = x_ce + 3.3 + 6 + 11
    g["flow"] = box(flow_x - 11, flow_x + 11, -8, 8, fz, fz + 8)
    g["flow_lens"] = np.array([flow_x + 5, -2.0, fz])               # lens 6 from the nose edge, 6 from the right edge (A)
    land = box(flow_x - 13, flow_x + 13, -10, 10, fz + 8, fz + 10.5)
    tongue = union([box(x_ce + 3.3, flow_x + 13, -10, 10, fz + 8, zc1), box(x_ce + 0.3, x_ce + 14, -10, 10, fz + 8, PLATE_BOT)])
    # pogo pads: 2 pads 10 x 10 face down on an outrigger, both on one side (one post), symmetric about the catch centre
    py = pogo_side * POGO_Y
    pz = -6.0 - DZ                                                    # pad face z_s -6 (DEFAULT, post in the opening wedge)
    g["pogo"] = [box(x - 5, x + 5, py - 5, py + 5, pz, pz + 1.6) for x in (-POGO_X, POGO_X)]
    g["pogo_xy"] = [(-POGO_X, py), (POGO_X, py)]; g["pogo_z"] = pz
    out = box(-POGO_X - 6, POGO_X + 6, min(py, pogo_side * 26.6) - (0 if pogo_side > 0 else 6), max(py, pogo_side * 26.6) + (6 if pogo_side > 0 else 0), pz + 1.6, pz + 6.6)
    g["receiver"] = union(walls + [ceil] + ends + pads + [carrier, land, tongue, out])
    # feet: 4 TPU pads R 5 x 3, 3 under the cartridge bottom, outside the head (|y| > 47 + 3 + 5) and behind the arm sweep
    fzb = CART_BOT - 3.0
    fx = (x_ce - 8.0, x0 + 8.0)
    g["feet_xy"] = [(x, s * 55.0) for x in fx for s in (1, -1)]
    g["feet"] = [cyl(5.0, fzb, fzb + 3.0, x, y) for x, y in g["feet_xy"]]
    # legs (3.0 rods): a vertical post from each foot to z_s -36 (above the head zone), then a V of two rods into the
    # receiver wall 12 higher (the receiver carries the feet; the topology run decides the final shape)
    g["legs"] = []
    zp = -36.0 - DZ
    for x, y in g["feet_xy"]:
        s = 1 if y > 0 else -1
        top = np.array([x, y, zp])
        g["legs"].append((np.array([x, y, fzb + 3.0]), top))
        for dx in (-10.0, 10.0):
            xa = min(max(x + dx, x0 - 3.3), x_ce + ARM_GAP - 2.5)        # 1.0 + rod r short of the arm carriage
            g["legs"].append((top, np.array([xa, s * 26.6, zp + 12.0])))
    # electronics
    g["radxa_slot"], g["ubec_slot"] = radxa, ubec
    if radxa == "top":
        g["radxa"] = box(-32.5, 32.5, -15, 15, PLATE_TOP + 2.0 - 1.55, PLATE_TOP + 2.0 + 1.6 + 3.16)
    else:
        s = 1 if radxa == "wing+" else -1
        g["radxa"] = box(-32.5, 32.5, s * 39.0 if s > 0 else -45.3, 45.3 if s > 0 else -39.0, PLATE_BOT - 1.0 - 30.0, PLATE_BOT - 1.0)
    if ubec == "top-stack":                                           # over the Radxa on its standoffs
        g["ubec"] = box(-22.5, 22.5, -10, 10, PLATE_TOP + 2.0 + 1.6 + 3.16 + 1.5, PLATE_TOP + 2.0 + 1.6 + 3.16 + 9.5)
    elif ubec == "top-tail":
        g["ubec"] = box(-33, -5, -22.5, 22.5, PLATE_TOP + 0.5, PLATE_TOP + 8.5)
    elif ubec in ("wing+", "wing-"):
        s = 1 if ubec == "wing+" else -1
        g["ubec"] = box(-50, -5, 27.5 if s > 0 else -47.5, 47.5 if s > 0 else -27.5, PLATE_BOT - 9.0, PLATE_BOT - 1.0)
    g["xr2"] = box(PLATE_TAIL + 2, PLATE_TAIL + 14, 10, 26, PLATE_TOP + 0.5, PLATE_TOP + 6.5)        # rear, beside the tail strut (DEFAULT)
    # camera: board centre (95, 0, mid-gap), level, lens front x 108 behind the ring line (rods out of view)
    cz = (RING_LO + RING_HI) / 2
    g["cam_board"] = box(94, 96, -19, 19, cz - 19, cz + 19)
    g["cam_lens"] = cyl(7.0, 0, 12, 0, 0); g["cam_lens"].apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    g["cam_lens"].apply_translation([96, 0, cz]); g["lens_front"] = np.array([108.0, 0, cz])
    return g


# ---------------------------------------------------------------- cage (rough; part 2 draws it)
def outline(z):
    pts_ = [(112, -100), (112, 100), (100, 112), (-100, 112), (-112, 100), (-112, -100), (-100, -112), (100, -112)]
    return [np.array([x, y, z], float) for x, y in pts_]


def cage_rods(g):
    rods = []
    for z in (RING_LO, RING_HI):
        o = outline(z)
        rods += [(o[i], o[(i + 1) % 8]) for i in range(8)]
    # columns: at the x-face wheel nodes (+-112, +-100) only (G (7): none straight up from the +-y-face wheels),
    # side columns at (0, +-112), tail (-112, 0), nose pair at (112, +-22) beside the camera
    HZ = 62.0
    for x, y in [(112, 100), (112, -100), (-112, 100), (-112, -100), (0, 112), (0, -112), (-112, 0)]:
        rods.append((np.array([x, y, RING_LO]), np.array([x, y, RING_HI])))
    # nose pair beside the camera runs up to the crown and carries a cross bar there: the nose spoke stays over the board
    rods += [(np.array([112, s * 22.0, RING_LO]), np.array([112, s * 22.0, HZ])) for s in (1, -1)]
    rods.append((np.array([112, -22.0, HZ]), np.array([112, 22.0, HZ])))
    # crown: 4 spokes along the axes to a hub r 20 at z 62 (the |x|, |y| <= 9.6 strips are never under a disk)
    for x, y in [(112, 0), (-112, 0), (0, 112), (0, -112)]:
        u = np.array([x, y, 0.0]) / 112.0
        rods.append((np.array([x, y, HZ if x > 0 else RING_HI]), np.array([20 * u[0], 20 * u[1], HZ])))
    for i in range(4):
        a, b = math.radians(90 * i), math.radians(90 * (i + 1))
        rods.append((np.array([20 * math.cos(a), 20 * math.sin(a), HZ]), np.array([20 * math.cos(b), 20 * math.sin(b), HZ])))
    # core joints: side struts from the side hubs (0, +-58, 25) to the side columns, tail strut from the plate top
    rods += [(np.array([0, s * 58.0, 25.0]), np.array([0, s * 112.0, RING_LO])) for s in (1, -1)]
    rods.append((np.array([-38.0, 0, PLATE_TOP + 2.0]), np.array([-112.0, 0, RING_LO])))   # tail hub on the plate top (CHECK holes)
    # nose: camera mount bar from the nose pair to the board (printed, below)
    return rods


def wheels():
    """8 wheels R 5 at the catch points, centre 5.0 off the face (rod centre 4.0 + 1.0 inward), axle along the face"""
    out = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        for px, py, face in ((112 * sx, 100 * sy, "x"), (100 * sx, 112 * sy, "y")):
            w = trimesh.creation.cylinder(WHEEL_R, WHEEL_W, sections=32)
            if face == "x":
                w.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))   # axle along y
                c = np.array([px - sx * 1.0 / S2, py, RING_LO + 1.0 / S2])
            else:
                w.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))   # axle along x
                c = np.array([px, py - sy * 1.0 / S2, RING_LO + 1.0 / S2])
            w.apply_translation(c); out.append(w)
    return out


WHEELS = wheels()


def nodes():
    """corner nodes (rough): cheek blocks round each wheel pin, set 2.5 inward of the rod so they stay >= 1.0 off the faces"""
    out = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        a = box(-3, 3, -3, 3, -3, 3).apply_translation([sx * (112 - 2.5 / S2), sy * 100, RING_LO + 2.5 / S2])
        b = box(-3, 3, -3, 3, -3, 3).apply_translation([sx * 100, sy * (112 - 2.5 / S2), RING_LO + 2.5 / S2])
        out.append(trimesh.util.concatenate([a, b]).convex_hull)
    return out


NODES = nodes()


# ---------------------------------------------------------------- checks
def funnel_check(p):
    """S checks on drone points p (drone frame): closest to a face above the bottom edge, and |extent| below it at +-5 yaw"""
    q = p + [0, 0, DZ]
    d_face = np.full(len(q), np.inf)
    for ax in (0, 1):
        for sg in (1, -1):
            x = sg * q[:, ax]; z = q[:, 2]
            on = (z > FACE_Z0) & (z < FACE_Z1) & (np.abs(q[:, 1 - ax]) < FACE_K + z)
            d_face = np.where(on, np.minimum(d_face, (z + FACE_K - x) / S2), d_face)
    low = q[q[:, 2] < FACE_Z0]; ext = 0.0
    if len(low):
        for yaw in np.radians(np.linspace(-5, 5, 11)):
            c, s = math.cos(yaw), math.sin(yaw); r = low[:, :2] @ np.array([[c, -s], [s, c]]).T; ext = max(ext, float(np.abs(r).max()))
    return float(d_face.min()), ext


def plunger_clear(p):
    """G (7): extended plunger uphill of each +-y-face wheel, 12 wide at x +-100, slope 50.1..58.1, 12 off the face"""
    q = p + [0, 0, DZ]; best = np.inf
    for sy in (1, -1):
        y = sy * q[:, 1]; z = q[:, 2]
        s_up = ((y - OPEN_HALF) + (z - FACE_Z0)) / S2; off = (z + FACE_K - y) / S2
        for sx in (1, -1):
            dx = np.maximum(np.abs(q[:, 0] - sx * 100.0) - 6.0, 0)
            ds = np.maximum(np.maximum(50.1 - s_up, s_up - 58.1), 0); do = np.maximum(np.maximum(-off, off - 12.0), 0)
            best = min(best, float(np.sqrt(dx ** 2 + ds ** 2 + do ** 2).min()))
    return best


def box_zone_clear(p, g):
    """arm sweep (x >= x_ce + 2.5, |y| <= 50, z_s <= -2.2), head under the pack (|y| <= 47, z_s < -44.5 over the cartridge
    span), cartridge path (footprint + 3.3 straight down from the receiver mouth). Returns min distance to each (0 = inside)."""
    q = p + [0, 0, DZ]; x_ce = g["x_ce"]; x0 = x_ce - CART_L
    def dist_box(lo, hi):
        d = np.maximum(np.maximum(np.array(lo) - q, q - np.array(hi)), 0); return float(np.sqrt((d ** 2).sum(1)).min())
    arm = min(dist_box([x_ce + ARM_GAP, -50, -400], [x_ce + 400, 50, ARM_CORNER_S]),
              dist_box([x_ce + ARM_RIM_GAP, -50, -400], [x_ce + 400, 50, ARM_Z_S]))
    head = dist_box([x0 - 3.3, -HEAD_HALF_Y, -400], [x_ce + 3.3, HEAD_HALF_Y, CART_BOT_S])
    path = dist_box([x0 - 3.3, -CART_W / 2 - 3.3, -400], [x_ce + 3.3, CART_W / 2 + 3.3, CART_BOT_S])
    return arm, head, path


def cone_clear(lens, half_deg, p, down=True):
    d = p - lens; zz = -d[:, 2] if down else d[:, 0]
    rad = np.hypot(d[:, 0], d[:, 1]) if down else np.hypot(d[:, 1], d[:, 2])
    ang = np.degrees(np.arctan2(rad, np.maximum(zz, 1e-6)))
    inside = (zz > 1.0) & (ang < half_deg)
    return int(inside.sum()), float((ang[zz > 1.0] - half_deg).min()) if (zz > 1.0).any() else 90.0


# ---------------------------------------------------------------- masses (g; E unless tagged) and CG
def cg(g, rods):
    items = [("pack", 165, g["pack"]), ("cartridge + latch + XT60 (pack)", 30, g["cart"]), ("XT60E1-M + carrier", 8, g["xt_m"]),
             ("FC", 10, FC), ("Radxa", 10, g["radxa"]), ("UBEC", 15, g["ubec"]), ("XR2", 1, g["xr2"]), ("flow", 1.5, g["flow"]),
             ("camera", 10, g["cam_board"]), ("plate", 28, PLATE), ("wires", 20, FC), ("pogo pads", 2, g["pogo"][0])]
    items += [(f"motor {i}", 18 + 1.8, b) for i, b in enumerate(BELLS)]
    tot, mom = 0.0, np.zeros(3)
    for _, m, mesh in items:
        tot += m; mom += m * mesh.bounding_box.centroid
    rec = abs(g["receiver"].volume) * 1.27e-3 * 0.40                 # receiver envelope at 40 % fill (E, topology sets it)
    tot += rec; mom += rec * g["receiver"].centroid
    for a, b in rods:                                                 # rods 11.0 g/m (G)
        m = np.linalg.norm(b - a) * 0.011; tot += m; mom += m * (a + b) / 2
    for a, b in g["legs"]:
        m = np.linalg.norm(b - a) * 0.011; tot += m; mom += m * (a + b) / 2
    nodes_g = 8 * 3.5 + 9 * 2.5 + 4 * 0.9 + 8 * 0.2 + 6.0            # corner nodes, column ends, feet, wheels, hub (E)
    tot += nodes_g; mom += nodes_g * np.array([0, 0, 40.0])
    return tot, mom / tot, rec


# ---------------------------------------------------------------- search: pack position (CG) x electronics slots
RADXA = ["top", "wing+", "wing-"]; UBEC = ["top-stack", "top-tail", "wing+", "wing-"]; POGO = [1, -1]
results = []
for rs, us, ps in itertools.product(RADXA, UBEC, POGO):
    x_ce = 30.0
    for _ in range(6):                                                # CG on the thrust centre (x 0)
        g = layout(x_ce, rs, us, ps); rods = cage_rods(g)
        tot, c, _ = cg(g, rods)
        if abs(c[0]) < 0.05: break
        x_ce -= c[0] * tot / 200.0                                     # pack + cartridge ~200 g move together
    g = layout(x_ce, rs, us, ps); rods = cage_rods(g); tot, c, _ = cg(g, rods)
    ele = {"radxa": g["radxa"], "ubec": g["ubec"], "xr2": g["xr2"]}
    fixed = {"FC": FC, "receiver": g["receiver"], "plate": PLATE, "xt_m": g["xt_m"], "camera": g["cam_board"],
             **{f"bell{i}": b for i, b in enumerate(BELLS)}}
    hits = []
    for a, b in itertools.combinations(ele, 2):
        if overlap(ele[a], ele[b]) > 0.5: hits.append(f"{a}x{b}")
    for a in ele:
        for b in fixed:
            if overlap(ele[a], fixed[b]) > 0.5: hits.append(f"{a}x{b}")
    legs = trimesh.util.concatenate([rod(a, b) for a, b in g["legs"]])
    for a in ele:
        if overlap(ele[a], legs) > 0.5: hits.append(f"{a}xlegs")
    pp = np.vstack([pts(m, 800) for m in ele.values()])
    prop = float(prop_dist(pp).min())
    fd, ext = funnel_check(pp)
    arm, head, path = box_zone_clear(pp, g)
    # pogo post path: a 12 x 12 column under each pad free of the drone down to z_s -50 (E, post size)
    drone_pts = np.vstack([pts(g["receiver"], 6000), pts(legs, 3000), pts(trimesh.util.concatenate(g["feet"]), 600), pp, pts(g["cart"], 2000)])
    post = np.inf
    for x, y in g["pogo_xy"]:
        sel = (drone_pts[:, 2] < g["pogo_z"] - 0.1) & (drone_pts[:, 2] > -50 - DZ)
        q = drone_pts[sel]
        if len(q):
            post = min(post, float(np.hypot(np.maximum(np.abs(q[:, 0] - x) - 6, 0), np.maximum(np.abs(q[:, 1] - y) - 6, 0)).min()))
    # Overseer belly (layout v1, catch frame): down camera pod y 29..72 |x| <= 23, flow pod y -55..-29 |x| <= 15 (C): pads clear?
    ovs = all(not ((29 - 6 <= y <= 72 + 6 and abs(x) <= 23 + 6) or (-55 - 6 <= y <= -29 + 6 and abs(x) <= 15 + 6)) for x, y in g["pogo_xy"])
    # wire (mm): Radxa UART -> FC, camera USB -> Radxa, UBEC -> Radxa 5 V, UBEC <- XT60 battery side
    rc = g["radxa"].bounding_box.centroid; uc = g["ubec"].bounding_box.centroid; fcc = np.array([FC_C[0], 0, 13.5])
    cam = np.array([95.0, 0, (RING_LO + RING_HI) / 2]); xtc = np.array([g["xt_x"], 0, g["xt_top"]])
    wire = np.linalg.norm(rc - fcc) + np.linalg.norm(rc - cam) + np.linalg.norm(uc - rc) + np.linalg.norm(uc - xtc)
    ok = not hits and prop >= 6.0 and fd >= 3.0 and ext <= OPEN_HALF - 1.0 and arm > 1.0 and head > 1.0 and path > 1.0 and post > 0 and ovs
    score = wire + 10.0 * abs(c[1]) + 0.5 * max(c[2] - 25.0, 0)
    results.append(dict(radxa=rs, ubec=us, pogo=ps, x_ce=round(x_ce, 2), mass=round(tot, 1), cg=[round(v, 2) for v in c], wire=round(wire, 1),
                        prop=round(prop, 1), face=round(fd, 1), opening=round(ext, 1), arm=round(arm, 1), head=round(head, 1),
                        path=round(path, 1), post=round(post, 1), overseer_ok=ovs, hits=hits, ok=ok, score=round(score, 1)))
say("placement search (Radxa slot x UBEC slot x pogo side; pack x solved for CG x 0):")
say(f"{'radxa':7s} {'ubec':9s} {'pogo':4s} {'x_ce':>6s} {'g':>6s} {'cg y':>6s} {'cg z':>6s} {'wire':>6s} {'prop':>5s} {'face':>5s} {'open':>5s} {'arm':>5s} {'head':>5s} {'post':>5s} ok score  hits")
for r in sorted(results, key=lambda r: (not r["ok"], r["score"])):
    say(f"{r['radxa']:7s} {r['ubec']:9s} {r['pogo']:+4d} {r['x_ce']:6.1f} {r['mass']:6.0f} {r['cg'][1]:6.2f} {r['cg'][2]:6.1f} {r['wire']:6.0f} "
        f"{r['prop']:5.1f} {r['face']:5.1f} {r['opening']:5.1f} {r['arm']:5.1f} {r['head']:5.1f} {r['post']:5.1f} {'Y' if r['ok'] else '-'} {r['score']:6.1f}  {','.join(r['hits'])}"
        + ("" if r["overseer_ok"] else " overseer-pod"))
ok = [r for r in results if r["ok"]]
best = min(ok, key=lambda r: r["score"])
say(f"CG x after the solve: {max(abs(r['cg'][0]) for r in results):.2f} worst")
say(f"\nchosen: Radxa {best['radxa']}, UBEC {best['ubec']}, pogo side {'+y' if best['pogo'] > 0 else '-y'}")

# ---------------------------------------------------------------- chosen layout: every check
g = layout(best["x_ce"], best["radxa"], best["ubec"], best["pogo"]); rods = cage_rods(g)
tot, c, rec_g = cg(g, rods)
x_ce, x0 = g["x_ce"], g["x_ce"] - CART_L
R = {}
say(f"cartridge v4 (Scout): {CART_W:.0f} x {CART_H:.0f} x {CART_L} (was 79: +{CART_L - 79} at the lead end for the XT60), "
    f"x {x0:.1f}..{x_ce:.1f}, z {CART_BOT:.2f}..{CART_TOP:.2f} (z_s {CART_BOT_S}..{CART_BOT_S + CART_H}); tab line x {g['tab_x']:.1f}")
say(f"pack x {x_ce - 2.5 - PACK[0]:.1f}..{x_ce - 2.5:.1f}; XT60 at x {g['xt_x']:.1f} (lead end, facing up), mate face z {CART_TOP:.2f}, "
    f"male rear + cups to z {g['xt_top']:.1f} ({'behind the plate tail' if g['xt_x'] + 4.05 < PLATE_TAIL else 'through the plate XT60 cut-out'}: "
    f"plate tail x {PLATE_TAIL}, CHECK)")
say(f"mass ~{tot:.0f} g (E; receiver envelope at 40 % {rec_g:.0f} g), CG ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.1f})")
R.update(x_ce=x_ce, x_lead=x0, cart_L=CART_L, cart_z=[CART_BOT, CART_TOP], tab_x=g["tab_x"], xt_x=g["xt_x"], mass=tot, cg=c.tolist())

# props: rods >= 12, printed / bought >= 6
rp = np.vstack([seg_pts(a, b) for a, b in rods])
rod_prop = float((prop_dist(rp) - ROD_R).min())
others = {"receiver": g["receiver"], "Radxa": g["radxa"], "UBEC": g["ubec"], "XR2": g["xr2"], "camera": g["cam_board"],
          "wheels+nodes": trimesh.util.concatenate(WHEELS + NODES), "XT60E1-M": g["xt_m"]}
say(f"props (swept r {PROP_R} z {PROP_Z} +-{SWEEP}): rods {rod_prop:.1f} (rule 12); " +
    ", ".join(f"{k} {float(prop_dist(pts(m, 3000)).min()):.1f}" for k, m in others.items()) + " (rule 6)")
R["rod_prop"] = rod_prop
# funnel: everything but the wheels >= 1.0 off the faces (underside >= 3), inside the 168 opening at +-5 yaw below z_s -0.8
legs_m = trimesh.util.concatenate([rod(a, b) for a, b in g["legs"]])
rods_m = trimesh.util.concatenate([rod(a, b) for a, b in rods])
drone = {"cage rods": rods_m, "nodes": trimesh.util.concatenate(NODES), "receiver": g["receiver"], "cartridge": g["cart"], "legs": legs_m,
         "feet": trimesh.util.concatenate(g["feet"]), "camera": trimesh.util.concatenate([g["cam_board"], g["cam_lens"]]),
         "Radxa": g["radxa"], "UBEC": g["ubec"], "plate": PLATE, "FC": FC, "bells": trimesh.util.concatenate(BELLS), "pogo": trimesh.util.concatenate(g["pogo"])}
fl = {k: funnel_check(pts(m, 6000)) for k, m in drone.items()}
wf, _ = funnel_check(pts(trimesh.util.concatenate(WHEELS), 4000))
say("funnel seated, closest to a face (rule 1.0, underside 3.0): " + ", ".join(f"{k} {v[0]:.1f}" for k, v in fl.items() if v[0] < 40)
    + f"; wheels {wf:.2f} (rim on the face = 0)")
say(f"below z_s -0.8 at +-5 deg yaw: widest {max(v[1] for v in fl.values()):.1f} of {OPEN_HALF} (rule <= 83)")
R["face_min"] = min(v[0] for k, v in fl.items()); R["opening"] = max(v[1] for v in fl.values())
pl = {k: plunger_clear(pts(m, 6000)) for k, m in drone.items()}
say(f"hook latch plungers (G (7), rule 1.0): closest {min(pl, key=pl.get)} {min(pl.values()):.1f}")
R["plunger"] = min(pl.values())
# box zones
bz = {k: box_zone_clear(pts(m, 5000), g) for k, m in drone.items() if k not in ("cartridge",)}
for i, nm in enumerate(("arm sweep", "head under the pack", "cartridge path")):
    k = min(bz, key=lambda k: bz[k][i]); say(f"{nm}: closest {k} {bz[k][i]:.1f} (rule > 1.0)")
R["arm"] = min(v[0] for v in bz.values()); R["head"] = min(v[1] for v in bz.values()); R["path"] = min(v[2] for v in bz.values())
# flow view: 23 deg half (42 FOV + 2), nothing of the drone in it
allp = np.vstack([pts(m, 5000) for k, m in drone.items()])
n_in, marg = cone_clear(g["flow_lens"], 23.0, allp)
say(f"flow view cone 23 deg from ({g['flow_lens'][0]:.1f}, {g['flow_lens'][1]:.1f}, {g['flow_lens'][2]:.2f}): {n_in} drone points inside, margin {marg:.1f} deg")
R["flow_cone_pts"] = n_in; R["flow_lens"] = g["flow_lens"].tolist()
# camera view: 102 deg diagonal, 4:3 -> 86 x 70 (E); forward, level
lens = g["lens_front"]; d = np.vstack([pts(m, 4000) for k, m in drone.items() if k != "camera"] + [rp]) - lens
fwd = d[:, 0] > 0.5
az = np.degrees(np.arctan2(np.abs(d[:, 1]), np.maximum(d[:, 0], 1e-6))); el = np.degrees(np.arctan2(np.abs(d[:, 2]), np.maximum(d[:, 0], 1e-6)))
cam_in = int((fwd & (az < 43) & (el < 35)).sum())
say(f"camera view (86 x 70 deg, lens front x {lens[0]:.0f}, z {lens[2]:.1f}): {cam_in} drone points in view")
R["cam_in"] = cam_in
# overlaps between drone bodies (bolted joints excepted)
pairs = [("receiver", "FC"), ("receiver", "cartridge"), ("receiver", "Radxa"), ("receiver", "UBEC"), ("cartridge", "legs"), ("receiver", "camera"),
         ("Radxa", "plate"), ("UBEC", "plate"), ("cage rods", "camera"), ("cage rods", "plate"), ("cage rods", "bells"), ("legs", "UBEC"),
         ("legs", "Radxa"), ("pogo", "legs")]
ov = [(a, b, overlap(drone[a], drone[b])) for a, b in pairs]
ov += [("XT60E1-M", "plate", overlap(g["xt_m"], PLATE)), ("XT60E1-M", "cage rods", overlap(g["xt_m"], rods_m)), ("XR2", "cage rods", overlap(g["xr2"], rods_m))]
bad = [f"{a} x {b} {v:.0f} mm3" for a, b, v in ov if v > 0.5]
say("overlaps (M, > 0.5 mm3): " + ("none" if not bad else "; ".join(bad)) + "  (receiver x cartridge = 0 means the cartridge seats free)")
R["overlaps"] = bad
# FC to ceiling gap, pogo
zc1 = CART_TOP + TPU_STRIP + CEIL_T
say(f"receiver ceiling z {CART_TOP + TPU_STRIP:.2f}..{zc1:.2f}; FC lowest 9.5 -> {9.5 - zc1:.2f} clear (CHECK the FC's lowest point)")
say(f"pogo pads: ({g['pogo_xy'][0][0]:.0f}, {g['pogo_xy'][0][1]:.0f}) and ({g['pogo_xy'][1][0]:.0f}, {g['pogo_xy'][1][1]:.0f}), "
    f"face z {g['pogo_z']:.2f} (z_s -6.0), down; post column 12 x 12 below each free; axle angle "
    f"{math.degrees(math.atan2(144.5 - 6.0, abs(g['pogo_xy'][0][1]))):.0f} deg (opening wedge 45..135); travel >= {POGO_TRAVEL} (box)")
st = [np.array(xy) for xy in g["feet_xy"]]
say(f"feet: {', '.join(f'({x:.0f}, {y:.0f})' for x, y in g['feet_xy'])}, bottoms z {CART_BOT - 3:.2f} (3 under the cartridge); "
    f"stance {np.ptp([p[0] for p in st]):.0f} x {np.ptp([p[1] for p in st]):.0f}; tip-over side "
    f"{math.degrees(math.atan2(55 - 5, c[2] - (CART_BOT - 3))):.0f} deg, nose/tail {math.degrees(math.atan2(min(abs(st[0][0] - c[0]), abs(st[2][0] - c[0])), c[2] - (CART_BOT - 3))):.0f} deg")
R["feet"] = g["feet_xy"]; R["pogo"] = g["pogo_xy"]; R["pogo_z"] = g["pogo_z"]
FLOW_POS = (g["flow_lens"] - c) * [1, -1, -1] / 1000
say(f"PX4 flow position from the CG (FRD, m): x {FLOW_POS[0]:+.3f}, y {FLOW_POS[1]:+.3f}, z {FLOW_POS[2]:+.3f}; IMU (FC centre) from the CG: "
    f"x {(FC_C[0] - c[0]) / 1000:+.3f}, y 0, z {-(13.5 - c[2]) / 1000:+.3f}")
R["flow_px4"] = FLOW_POS.tolist()
rod_len = sum(np.linalg.norm(b - a) for a, b in rods) + sum(np.linalg.norm(b - a) for a, b in g["legs"])
say(f"rods: {len(rods) + len(g['legs'])} x 3.0, {rod_len / 1000:.2f} m axis (C, rough cage)")
R["search"] = results; R["chosen"] = {k: best[k] for k in ("radxa", "ubec", "pogo")}
R.update(date="2026-10-24T20:00", status="DONE", mass_g=round(tot, 0), mass_tag="E", note="layout, rough shapes; no part saved")

# ---------------------------------------------------------------- motion artifact: land, latch, swap the pack, take off
os.makedirs(OUT, exist_ok=True)
s = Scene("Scout v3 layout v1", "whole drone before the topology runs: lands in the funnel, the latch holds it, the box swaps the pack through the XT60, it leaves")
P = lambda n, m, grp, col, pr, note="": s.part(n, m, group=grp, color=col, printed=pr, note=note)


def funnel_mesh():
    faces = []
    for ax in (0, 1):
        for sg in (1, -1):
            pl = []
            for z, half in ((FACE_Z0, FACE_K + FACE_Z0), (FACE_Z1, FACE_K + FACE_Z1)):
                for lat in (-half, half):
                    for off in (0.0, 1.6 * S2):
                        p = [0.0, 0.0, z - DZ]; p[ax] = sg * (half + off); p[1 - ax] = lat; pl.append(p)
            faces.append(trimesh.Trimesh(np.array(pl)).convex_hull)
    return trimesh.util.concatenate(faces)


def plungers():
    out = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        t = 54.1; yf, zf = OPEN_HALF + t / S2, FACE_Z0 + t / S2
        b = box(-6, 6, -4, 4, -32.55, 0.0)                         # local: x along the edge, y up the slope, z out of the face
        T = np.eye(4); T[:3, 0] = [1, 0, 0]; T[:3, 1] = [0, sy / S2, 1 / S2]; T[:3, 2] = [0, -sy / S2, 1 / S2]
        T[:3, 3] = [sx * 100, sy * yf, zf - DZ]; b.apply_transform(T); out.append((b, T[:3, 2].copy()))
    return out


PL = plungers()
cam_m = trimesh.util.concatenate([g["cam_board"], g["cam_lens"]])
cam_mount = union([box(96, 108.5, -24, 24, RING_LO - 1, RING_LO + 1), box(96, 108.5, -24, 24, RING_HI - 1, RING_HI + 1)] +
                  [box(96, 113.5, sy * 22 - 3, sy * 22 + 3, RING_LO - 1, RING_HI + 1) for sy in (1, -1)])
P("plate", PLATE, "frame", "#3a3a3a", False, "Bee35 cut down, 3.5 carbon")
P("motors", trimesh.util.concatenate(BELLS), "frame", "#b0b0b0", False)
props_v = []
for i, (x, y) in enumerate(MOTORS_XY):
    ring = man(cyl(PROP_R, PROP_Z - 0.4, PROP_Z + 0.4, x, y, 72)) - man(cyl(PROP_R - 2.0, PROP_Z - 1, PROP_Z + 1, x, y, 72))
    ring = ring.to_mesh(); props_v.append(trimesh.Trimesh(ring.vert_properties[:, :3], ring.tri_verts))
    bl = box(-PROP_R, PROP_R, -6, 6, PROP_Z - 1, PROP_Z + 1); bl.apply_transform(rotz(30 + 45 * i)); bl.apply_translation([x, y, 0]); props_v.append(bl)
P("props (swept)", trimesh.util.concatenate(props_v), "frame", "#d9a400", False, "Gemfan 3520, plane z 41.2")
P("FC", FC, "electronics", "#1b5e20", False, "under the plate, as built")
P("Radxa", g["radxa"], "electronics", "#2e7d32", False, f"slot {best['radxa']}")
P("UBEC", g["ubec"], "electronics", "#444488", False, f"slot {best['ubec']}")
P("XR2", g["xr2"], "electronics", "#6a1b9a", False, "rear, as built")
P("camera", cam_m, "electronics", "#2d5a2d", False, "DECXIN, level, lens behind the ring line")
P("flow MTF-02P", g["flow"], "electronics", "#2d5a2d", False)
P("receiver (envelope)", g["receiver"], "printed", "#8899aa", True, "topology design space: walls, pockets, ceiling, carrier, flow land, pogo arm")
P("camera mount (rough)", cam_mount, "printed", "#9aa8b8", True, "part 3")
P("cage rods", rods_m, "cage", "#202020", False, "3.0 carbon, rough (part 2)")
P("corner nodes", trimesh.util.concatenate(NODES), "cage", "#aab4c0", True, "rough")
P("catch wheels", trimesh.util.concatenate(WHEELS), "cage", "#cc6644", True, "R 5, the box's interface")
P("legs", legs_m, "cage", "#202020", False)
P("feet", trimesh.util.concatenate(g["feet"]), "cage", "#555555", True, "TPU")
P("pogo pads", trimesh.util.concatenate(g["pogo"]), "receiver", "#c9a227", False, "keep-alive, face down")
P("XT60E1-M (drone)", g["xt_m"], "receiver", "#e0c020", False, "on the float carrier")
P("cartridge v4", g["cart"], "pack", "#7d8a99", True, "92 long, XT60 at the lead end")
P("4S 1500", g["pack"], "pack", "#2f4fbf", False)
P("XT60 (pack)", g["xt_f"], "pack", "#e0c020", False)
P("funnel faces", funnel_mesh(), "box", "#c8d6e5", False, "collar v4, seat frame")
for i, (b, _) in enumerate(PL):
    P(f"plunger {i + 1}", b, "box", "#e07a5f", False, "hook latch v1")
drone_names = [p["name"] for p in s.parts if p["group"] not in ("box", "pack")]
pack_names = ["cartridge v4", "4S 1500", "XT60 (pack)"]
LIFT = 2.12                                                          # the XT60 push lifts the drone onto the plungers (G, C)


def smooth(t, a, b):
    u = min(max((t - a) / (b - a), 0.0), 1.0); return u * u * (3 - 2 * u)


def pose(t):
    # 0..3 descend from 140 up and 30 off into the seat; 4..4.5 plungers out; 5..7 pack out (XT60 pulled, 70 down);
    # 7.5..9.5 pack back in; 9.5..10 the push lifts the drone 2.1 onto the plungers; 10.5..11 drone settles, plungers in;
    # 11.5..14 take off
    dz = 140 * (1 - smooth(t, 0, 3)) + LIFT * (smooth(t, 9.5, 10) - smooth(t, 10.5, 11)) + 160 * smooth(t, 11.5, 14)
    dx = 30 * (1 - smooth(t, 0, 3))
    Td = np.eye(4); Td[:3, 3] = [dx, 0, dz]
    pz = -70 * (smooth(t, 5, 7) - smooth(t, 7.5, 9.5))
    Tp = Td.copy(); Tp[2, 3] += pz
    out = {n: Td for n in drone_names}; out.update({n: Tp for n in pack_names})
    out_frac = smooth(t, 4, 4.5) - smooth(t, 10.5, 11)
    for i, (_, w) in enumerate(PL):
        T = np.eye(4); T[:3, 3] = w * 12.0 * out_frac; out[f"plunger {i + 1}"] = T
    return out


s.timeline(pose, duration_s=14, fps=30)
s.view("iso", (330, -380, 260), (0, 0, 0)); s.view("side", (0, -560, 10), (0, 0, 0)); s.view("front", (560, 0, 10), (0, 0, 0))
s.view("top", (0, 0, 620), (0, 0, 0)); s.view("under", (180, -220, -300), (0, 0, -20))
s.write(os.path.join(OUT, "motion.html"))
if VIDEO:
    try:
        s.video(os.path.join(OUT, "motion.mp4"))
    except Exception as e:
        say("video skipped:", str(e)[:200])
for nm, m in (("receiver_envelope", g["receiver"]), ("cartridge_v4_envelope", g["cart"])):
    m.export(os.path.join(OUT, f"{nm}.stl"))
json.dump(R, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
print("wrote", OUT)
