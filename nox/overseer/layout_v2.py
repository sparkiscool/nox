"""Overseer layout v2 (rough design, 24 Oct): the cage (Spark 24 Oct: "the lidar should be mounted on the cage and not
extend out from the body to save weight, lets make the cage first, it must fit in the funnel"). v1 with the nose boom
replaced by a carbon rod cage: two octagon guard rings round the four prop disks (12 off, like the Scout's), carried on
spokes from the arm tips; a lower wheel ring at the catch points with 4 corner nodes holding the 8 wheels (Scout-style)
on struts up to the arms; the L2 sits on the cage's front apex, axis 15 deg below +x, every rod behind its field; the
front camera on the same apex node. Balance as v1: the printed arms carry the motor square forward, the catch sits aft.

python3 nox/overseer/layout_v2.py [out_dir]      (~25 min: catch-offset and wheel-height search, balance, artifact;
V2_CASES="30,-30" reruns only that case)

Drone frame: x forward, y left, z up; z 0 = bottom plate top face; plate centre at x 0. Numbers from `nox/dimensions.md`
O / M / G (tagged there); E where marked here. v0 / v1 kept as they were.
"""
import math, os, sys, io, contextlib
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.join(HERE, "s500")); sys.path.insert(0, HERE)
from motion import Scene
import s500_cad, arm_reach
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_layout", "v2")
S2 = 1 / math.sqrt(2)
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


# ---------------------------------------------------------------- numbers
HALF = (254 + 25) / 2                    # 139.5: motor square half side at a 25 tip gap (C, O Frame)
ROOT_H, TOP_T = 45.0, 1.6                # arm root / plate gap (DEFAULT), top plate (derived)
H_TIP_REAR, H_TIP_FRONT = 16.0, 38.0     # arm depth under the motor (E); the longer front pair deeper and wider (34 -> 28)
PAD_Z = ROOT_H
MOTOR_H = 26.0                           # X2216 body (E)
PROP_Z = PAD_Z + 37.0                    # prop hub mid plane (E)
PROP_R, BLADE = 127.0, 5.0
L2_TILT = 105.0                          # L2 axis turned 105 deg from +z toward +x: 15 deg below the x axis (Spark 24 Oct)
L2_NEG = 0.0                             # normal mode (90 deg hemisphere) kept clear (DEFAULT): the 96 deg negative-angle mode would
                                         # see the far ends of the cage front rods unless the L2 moves ~20 further out
A = np.array([math.sin(math.radians(L2_TILT)), 0.0, math.cos(math.radians(L2_TILT))])   # (0.966, 0, -0.259)
CAM_HFOV, CAM_VFOV = 73.0, 89.0          # front camera portrait, 4:3 from 102 diag (E)
DOWN_HFOV, DOWN_VFOV = 89.0, 73.0
FLOW_HALF = 23.0
PACK = (148.0, 48.0, 45.0)               # 4S 6000 (P)
CART = (157.0, 54.0)                     # cartridge envelope (E, G)
WHEEL_R, WHEEL_W = 5.0, 3.0               # R 5 (v4 set): the hook latch is laid out on it (G Hook latch v1)
OPEN_HALF, FACE_TOP = 84.0, 75.1         # funnel (G)
WHEEL_SEAT = 112 - (OPEN_HALF + 0.8) + WHEEL_R * math.sqrt(2)   # seated wheel centre, seat z 32.9 (C)
CART_BOT_SEAT = (-7.0 - 45.0 - 2.0) + (WHEEL_SEAT + 12.0)       # cartridge bottom kept at v0's seat z -9.1: the swap wheel's
                                                                # R 100 sets it for both drones (box-arm.md Ring); matching it
                                                                # to the Scout's is the cartridge part's job (CHECK)


def set_wheel(wz):
    """wheel centres at drone z wz: the seat height follows, the cartridge hangs to keep its seat z"""
    global WHEEL_Z, SEAT_DZ, FUNNEL_TOP_D, PACK_TOP
    WHEEL_Z = wz
    SEAT_DZ = WHEEL_SEAT - wz                    # bottom plate top in the seat frame
    FUNNEL_TOP_D = FACE_TOP - SEAT_DZ            # funnel top in the drone frame: the L2 and the arms over the rim stay above it
    PACK_TOP = CART_BOT_SEAT - SEAT_DZ + 2.0 + PACK[2]


set_wheel(-12.0)
MOTOR_DEG = (45, 135, 225, 315)
CAM_BACK = 30.0                          # front camera board behind the L2 base plane
ROD_R = 1.5                              # 3.0 carbon rods everywhere (G Carbon rods), 11.0 g/m (C)
ROD_GPM = 11.0e-3
GUARD_GAP = 12.0                         # guard rods 12 off the swept disks in plan, like the Scout's (G Guard outline)
RING_LO, RING_HI = PROP_Z - 14.0, PROP_Z + 10.0   # guard ring heights (DEFAULT): the lower one low enough that the arm-tip
                                                  # spokes pass 12 under the swept band where they run under a disk


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b


def cyl(r, z0, z1, x=0, y=0, sections=48):
    c = trimesh.creation.cylinder(r, z1 - z0, sections=sections); c.apply_translation([x, y, (z0 + z1) / 2]); return c


def rotz(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def hull(*meshes):
    return trimesh.util.concatenate(meshes).convex_hull


def union(meshes):
    from topo.part import man
    out = man(meshes[0])
    for m in meshes[1:]:
        out = out + man(m)
    mm = out.to_mesh()
    return trimesh.Trimesh(mm.vert_properties[:, :3], mm.tri_verts)


def overlap(a, b):
    from topo.part import man
    return (man(a) ^ man(b)).volume()


def rod(p0, p1, r=ROD_R):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    m = trimesh.creation.cylinder(r, np.linalg.norm(p1 - p0), sections=12, segment=np.vstack([p0, p1]))
    return m


def pts(m, n=3000, seed=1):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=seed)[0]])


# ---------------------------------------------------------------- stock plates (Holybro S500 STEP, derived)
with contextlib.redirect_stdout(io.StringIO()):
    parts = s500_cad.load(s500_cad.STEP)
CAD2DRONE = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)


def cad_mesh(key, dz):
    import tempfile
    shp = s500_cad.find(parts, key)
    BRepMesh_IncrementalMesh(shp, 0.1, False, 0.3, True)
    with tempfile.TemporaryDirectory() as tmp:
        StlAPI_Writer().Write(shp, os.path.join(tmp, "p.stl")); m = trimesh.load(os.path.join(tmp, "p.stl"))
    m.apply_transform(CAD2DRONE); m.apply_translation([0, 0, dz]); return m


BOTTOM = cad_mesh("BOTTOM-PLATE-S500", -0.10)
TOP = cad_mesh("TOP-PLATE-S500", ROOT_H - 18.04)

# ---------------------------------------------------------------- hub (between the plates), fixed to the plates
FC = box(-18, 18, -18, 18, 1.0, 18.5)                        # FC (+ stack) 36 x 36 x 17.5 (spec)
DECK = union([box(-44.5, 44.5, -30, 30, 20.0, 22.0)] + [box(sx * 40 - 2, sx * 40 + 2, sy * 25 - 2, sy * 25 + 2, 0, 20) for sx in (1, -1) for sy in (1, -1)])
PI = box(-42.5, 42.5, -28, 28, 22.5, 42.0)                   # Pi 5 with cooler (E height)
PDB = box(-95, -45, -25, 25, 0.5, 6.5)                       # FCHUB-12S on the rear wing (spec)
UBEC = box(-95, -47, 0, 24, 7.5, 17.5)                       # stacked over the PDB on the rear wing (v1: the front wing carries the boom)
BUCK = box(-80, -62, -24, -6, 7.5, 13.5)
RX = box(-95, -83, -22, -10, 7.5, 13.5)
FRONT_HUB = box(86, 99, -18, 18, 0, 12)                       # keel rod hub on the bottom plate front wing (DEFAULT)


# ---------------------------------------------------------------- geometry that moves with the balance
def l2_mesh(B):
    """L2 in its own frame (base at z 0, body +z, dome on top), turned L2_TILT about y, base centre at B."""
    body = box(-37.5, 37.5, -37.5, 37.5, 0, 39)
    dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([1, 1, 26 / 35.5]); dome.apply_translation([0, 0, 39])
    dome = dome.slice_plane([0, 0, 39], [0, 0, 1], cap=True)
    m = trimesh.util.concatenate([body, dome])
    T = trimesh.transformations.rotation_matrix(math.radians(L2_TILT), [0, 1, 0]); T[:3, 3] = B
    m.apply_transform(T); return m


def build(dm, dc, xB, zB, cam_up=0.0, cam_h=None):
    """dm: motor square ahead of the plate centre; dc: catch centre aft of it; (xB, zB): L2 base centre."""
    g = {}
    motors_xy = [(dm + HALF * (1 if d in (45, 315) else -1), HALF * (1 if d in (45, 135) else -1)) for d in MOTOR_DEG]
    arms, axes = [], []
    for d, (mx, my) in zip(MOTOR_DEG, motors_xy):
        front = mx > dm
        ht = H_TIP_FRONT if front else H_TIP_REAR
        c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
        root = box(58, 75, -21, 21, 0, ROOT_H); root.apply_transform(rotz(d))
        r0 = np.array([66.5 * c, 66.5 * s])
        hw = 17 if front else 13                                   # half width at the root end (front 34, rear 26)
        root_end = box(-6, 6, -hw, hw, 0, ROOT_H); ang = math.degrees(math.atan2(my - r0[1], mx - r0[0]))
        root_end.apply_transform(rotz(ang)); root_end.apply_translation([r0[0], r0[1], 0])
        tip = cyl(14 if front else 11, ROOT_H - ht, ROOT_H, mx, my)
        pad = cyl(18, ROOT_H - 4, ROOT_H, mx, my)
        arms.append(union([root, hull(root_end, tip), pad])); axes.append((r0, np.array([mx, my]), ht))
    g["arms"] = arms; g["axes"] = axes
    g["motors"] = [cyl(13.85, PAD_Z, PAD_Z + MOTOR_H, mx, my) for mx, my in motors_xy]
    g["props"] = [cyl(PROP_R, PROP_Z - 0.5, PROP_Z + 0.5, mx, my, 96) for mx, my in motors_xy]
    g["motors_xy"] = motors_xy

    # catch, centred at x = -dc: 4 corner nodes (each spans its chamfer and holds 2 wheels, Scout-style), a lower ring of
    # 3.0 rods along the wheel lines, one strut per corner node up to the nearest arm (rises from the x-face end of the
    # node: the hook latch plungers sit uphill of the ±y-face wheels, G (7))
    wheels, nodes, rods = [], [], {}
    for sx in (1, -1):
        for sy in (1, -1):
            blks = []
            for (px, py, axis) in ((112 * sx, 100 * sy, "y"), (100 * sx, 112 * sy, "x")):
                cx, cy = px - dc, py
                w = trimesh.creation.cylinder(WHEEL_R, WHEEL_W, sections=32)
                w.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0] if axis == "y" else [0, 1, 0]))
                w.apply_translation([cx, cy, WHEEL_Z]); wheels.append(w)
                if axis == "y":   # rolls on an x face: axle along y, cheek beside it toward the drone axis
                    ox = (cx - 3, cx + 1) if sx > 0 else (cx - 1, cx + 3)
                    oy = (cy - 5.5, cy - 1.6) if sy > 0 else (cy + 1.6, cy + 5.5)
                else:
                    oy = (cy - 3, cy + 1) if sy > 0 else (cy - 1, cy + 3)
                    ox = (cx - 5.5, cx - 1.6) if sx > 0 else (cx + 1.6, cx + 5.5)
                blks.append(box(ox[0], ox[1], oy[0], oy[1], WHEEL_Z - 2, WHEEL_Z + 4))
            nodes.append(hull(*blks))
            # strut root on the x-face wheel's cheek, inboard, on top of the node
            root_p = np.array([112 * sx - dc - 2 * sx, 100 * sy - 6 * sy, WHEEL_Z + 4])
            best = None
            for r0, m1, ht in axes:
                v = m1 - r0; t = np.clip(np.dot(root_p[:2] - r0, v) / np.dot(v, v), 0, 1); q = r0 + t * v
                dd = np.hypot(*(q - root_p[:2]))
                if best is None or dd < best[0]: best = (dd, q, t, ht)
            _, q, t, ht = best
            top_p = np.array([q[0], q[1], t * (ROOT_H - ht)])
            rods[f"strut {sx:+d}{sy:+d}"] = (root_p, top_p)
    for sx in (1, -1):   # lower ring: x-face rods along y, y-face rods along x, at the wheel centre line (rims 3.5 proud)
        rods[f"ring x{sx:+d}"] = ((112 * sx - dc, -94.5, WHEEL_Z), (112 * sx - dc, 94.5, WHEEL_Z))
    for sy in (1, -1):
        rods[f"ring y{sy:+d}"] = ((-94.5 - dc, 112 * sy, WHEEL_Z), (94.5 - dc, 112 * sy, WHEEL_Z))
    g["wheels"], g["nodes"] = wheels, nodes
    PB = PACK_TOP - PACK[2]
    g["pack"] = box(-dc - PACK[0] / 2, -dc + PACK[0] / 2, -PACK[1] / 2, PACK[1] / 2, PB, PACK_TOP)
    g["cart"] = box(-dc - CART[0] / 2, -dc + CART[0] / 2, -CART[1] / 2, CART[1] / 2, PB - 2, -2.11)
    g["sled"] = box(-dc - 60, -dc + 60, -72, 72, -5.1, -2.1)
    if PACK_TOP < -7.5:                                                        # hanger walls from the sled to the cartridge rails
        g["sled"] = union([g["sled"], box(-dc - 60, -dc + 60, 27.5, 31, PACK_TOP, -5.0), box(-dc - 60, -dc + 60, -31, -27.5, PACK_TOP, -5.0)])
    g["cam_pod"] = box(-dc - 23, -dc + 23, 29, 72, PB + 4, -5.1)
    g["flow_pod"] = box(-dc - 15, -dc + 15, -55, -29, PB + 4, -5.1)
    g["down_cam"] = trimesh.util.concatenate([box(-dc - 19, -dc + 19, 30, 68, PB + 2.4, PB + 4), cyl(6, PB - 6, PB + 2.4, -dc, 49)])
    g["down_lens"] = np.array([-dc, 49.0, PB - 6]); g["flow"] = box(-dc - 10.8, -dc + 10.8, -50, -34, PB - 2.5, PB + 4)
    g["flow_lens"] = np.array([-dc, -42.0, PB - 2.5])
    g["feet"] = [cyl(5, PB - 9, PB + 4, -dc + sx * 68, sy * 34) for sx in (1, -1) for sy in (1, -1)]

    # guard: two octagon rings round the motor square, 12 off the swept disks; the front flat is a V to the L2 apex
    B = np.array([xB, 0.0, zB]); g["B"] = B
    g["l2"] = l2_mesh(B)
    up_b = np.array([-A[2], 0, A[0]])                            # in the base plane, toward +z: (0.259, 0, 0.966)
    seat = cyl(37.5, -5, 0, 0, 0); T = trimesh.transformations.rotation_matrix(math.radians(L2_TILT), [0, 1, 0]); T[:3, 3] = B
    seat.apply_transform(T)
    back = box(-22, 22, -16, 16, -11, -5); back.apply_transform(T)               # apex block behind the seat (L2 local z -11..-5)
    # apex rod sockets on side lugs beside the L2 body (y ±46), just behind the base plane, so the V never bends back
    # inside the front flat (the guard keeps 12 off the front disks)
    sock = {(lvl, sy): B - A * 5 + up_b * (16 if lvl == "hi" else 2) + np.array([0, 46.0 * sy, 0]) for lvl in ("hi", "lo") for sy in (1, -1)}
    lugs = []
    for sy in (1, -1):
        lug = box(-22, 4, 36 if sy > 0 else -50, 50 if sy > 0 else -36, -8, -2); lug.apply_transform(T); lugs.append(lug)   # local x -22..4 = up_b -4..22
    F = HALF + PROP_R + GUARD_GAP + ROD_R                          # 280: flats from the motor-square centre
    Cf = 2 * HALF + math.sqrt(2) * (PROP_R + GUARD_GAP + ROD_R)    # chamfer line x + y = 477.7
    V = [(F, Cf - F), (Cf - F, F), (-(Cf - F), F), (-F, Cf - F), (-F, -(Cf - F)), (-(Cf - F), -F), (Cf - F, -F), (F, -(Cf - F))]
    V = [(dm + vx, vy) for vx, vy in V]
    g["ring_v"] = V
    ring_nodes = []
    for vx, vy in V:
        ring_nodes.append(box(vx - 5, vx + 5, vy - 5, vy + 5, RING_LO - 6, RING_HI + 6))
    for k in range(8):
        a, b = V[k], V[(k + 1) % 8]
        if k == 7:                                                   # front flat -> V to the apex
            for zr, lvl in ((RING_LO, "lo"), (RING_HI, "hi")):
                rods[f"V L{zr:.0f}"] = ((V[0][0], V[0][1], zr), sock[(lvl, 1)]); rods[f"V R{zr:.0f}"] = ((V[7][0], V[7][1], zr), sock[(lvl, -1)])
            continue
        for zr in (RING_LO, RING_HI):
            rods[f"ring {k} z{zr:.0f}"] = ((a[0], a[1], zr), (b[0], b[1], zr))
    for i_m, (mx, my) in enumerate(motors_xy):                    # 2 spokes per motor: arm tip underside to the 2 nearest vertices
        ht = axes[i_m][2]
        near = sorted(V, key=lambda v: math.hypot(v[0] - mx, v[1] - my))[:2]
        for n_, (vx, vy) in enumerate(near):
            rods[f"spoke {i_m}{n_}"] = ((mx, my, ROOT_H - ht), (vx, vy, RING_LO - 5))
    for sy in (1, -1):                                             # 2 keels from the front hub to the apex
        rods[f"keel {sy:+d}"] = ((96, 12 * sy, 8), sock[("lo", sy)])
    g["rods"] = rods
    g["rod_meshes"] = {k: rod(*v) for k, v in rods.items()}
    g["ring_nodes"] = ring_nodes
    g["apex"] = union([seat, back, FRONT_HUB] + lugs)
    low_keys = [k for k in rods if k.startswith(("strut", "ring x", "ring y"))]
    g["cage_low"] = trimesh.util.concatenate(nodes + [g["rod_meshes"][k] for k in low_keys])
    g["cage_up"] = trimesh.util.concatenate(ring_nodes + [g["rod_meshes"][k] for k in rods if k not in low_keys])
    # front camera: board centre CAM on the base plane's back side, aimed cam_up above the horizon
    fwd = np.array([math.cos(math.radians(cam_up)), 0, math.sin(math.radians(cam_up))])
    cup = np.array([-math.sin(math.radians(cam_up)), 0, math.cos(math.radians(cam_up))])
    h = cam_h if cam_h is not None else 60.0
    C = B + up_b * h - A * CAM_BACK                               # h along the base plane, CAM_BACK behind it
    g["cam_c"], g["cam_fwd"], g["cam_up_v"] = C, fwd, cup
    Rc = np.eye(4); Rc[:3, 0] = fwd; Rc[:3, 1] = [0, 1, 0]; Rc[:3, 2] = cup
    board = box(-1, 1, -19, 19, -19, 19); lens = trimesh.creation.cylinder(6, 8, sections=32)
    lens.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); lens.apply_translation([5, 0, 0])
    cam = trimesh.util.concatenate([board, lens]); Rc[:3, 3] = C; cam.apply_transform(Rc); g["cam"] = cam
    g["cam_lens"] = C + fwd * 9
    # camera bracket: a 3 thick printed plate behind the board on three 3.0 rods down to the apex block (Spark 24 Oct:
    # 3 mm rods wherever they do the job)
    plate = box(-4.5, -1.5, -20, 20, -20, 20).apply_transform(Rc)
    to_w = lambda v: (Rc @ np.array([*v, 1.0]))[:3]
    cam_rods = [(to_w((-3.0, sy * 16.0, -19.0)), B - A * 8 + up_b * 20 + np.array([0, sy * 16.0, 0])) for sy in (1, -1)]
    cam_rods.append((to_w((-3.0, 0.0, -19.0)), B - A * 11 + up_b * 20))
    g["cam_rods"] = cam_rods
    g["cam_plate"] = plate
    g["cam_bracket"] = trimesh.util.concatenate([plate] + [rod(a, b) for a, b in cam_rods])
    return g


def l2_clear_margin(g, B, solid_items):
    """smallest margin (mm) of every drone point to the L2 field: points must lie behind the base plane by
    sin(6 deg) x distance (negative-angle mode); the seat and the block behind it are the mount and skipped."""
    worst = (None, np.inf)
    for name, m in solid_items:
        p = pts(m, 2500)
        if name == "props":
            p = np.vstack([p + [0, 0, BLADE], p - [0, 0, BLADE]])
        d = p - B; depth = d @ A; dist = np.linalg.norm(d, axis=1)
        if name == "apex":
            radial = np.linalg.norm(d - np.outer(depth, A), axis=1)
            keep = ~((depth > -5.5) & (radial < 40.0)); d, depth, dist = d[keep], depth[keep], dist[keep]   # the seat itself
        margin = -depth - math.sin(math.radians(L2_NEG)) * dist
        i = int(np.argmin(margin))
        if margin[i] < worst[1]: worst = (name, margin[i])
    return worst


def cg_of(g):
    items = [("L2", 230, g["l2"]), ("front camera", 15, g["cam"]), ("down camera", 15, g["down_cam"]), ("Pi 5 + cooler", 55, PI),
             ("FC", 10, FC), ("PDB", 21, PDB), ("UBEC", 15, UBEC), ("buck", 6, BUCK), ("RX", 2, RX), ("pack", 650, g["pack"]),
             ("flow", 2, g["flow"]), ("top plate", 23, TOP), ("bottom plate", 61, BOTTOM), ("wires", 80, FC)]
    items += [(f"motor + prop {i}", 81 + 10, m) for i, m in enumerate(g["motors"])]
    items += [(f"ESC {i}", 25, g["arms"][i]) for i in range(4)]                  # on the arms (model open, E)
    items += [(f"wheel {i}", 0.25, w) for i, w in enumerate(g["wheels"])]
    for k, (a, b) in list(g["rods"].items()) + [(f"cam rod {i}", r) for i, r in enumerate(g["cam_rods"])]:   # 3.0 carbon, 11.0 g/m (C)
        L = float(np.linalg.norm(np.subtract(b, a))); items.append((k, L * ROD_GPM, trimesh.points.PointCloud([np.add(a, b) / 2])))
    printed = [("arms", g["arms"], 0.45), ("corner nodes", g["nodes"], 0.6), ("ring nodes", g["ring_nodes"], 0.5), ("apex", [g["apex"]], 0.45),
               ("cam plate", [g["cam_plate"]], 0.5),
               ("deck", [DECK], 0.5), ("sled + pods", [g["sled"], g["cam_pod"], g["flow_pod"]], 0.3), ("feet", g["feet"], 0.5),
               ("cartridge", [g["cart"]], 0.04)]
    tot, mom = 0.0, np.zeros(3)
    for _, gm, m in items:
        c = m.vertices.mean(axis=0) if isinstance(m, trimesh.points.PointCloud) else m.bounding_box.centroid
        tot += gm; mom += gm * c
    for _, ms, fill in printed:
        for m in ms:
            gm = abs(m.volume) * 1.27e-3 * fill; tot += gm; mom += gm * m.centroid
    return tot, mom / tot


def l2_low_zB():
    """lowest base-centre height that keeps the L2 body 1.0 above the funnel top (seated)"""
    m = l2_mesh(np.zeros(3)); return FUNNEL_TOP_D + 1.0 - m.bounds[0][2]


def guard_v_gap(g):
    """plan gap of the V rods to the front disks (≥ GUARD_GAP)"""
    worst = np.inf
    for k, (a, b) in g["rods"].items():
        if not k.startswith("V "): continue
        t = np.linspace(0, 1, 200)[:, None]; q = np.asarray(a, float) + t * (np.subtract(b, a))
        for mx, my in g["motors_xy"]:
            worst = min(worst, float((np.hypot(q[:, 0] - mx, q[:, 1] - my) - PROP_R - ROD_R).min()))
    return worst


def solve_xB(g_fn, zB):
    """smallest base x with the drone behind the L2 field (bisection on x)"""
    lo, hi = 150.0, 500.0
    for _ in range(12):
        mid = (lo + hi) / 2; g = g_fn(mid)
        items = [("props", trimesh.util.concatenate(g["props"])), ("motors", trimesh.util.concatenate(g["motors"])),
                 ("arms", trimesh.util.concatenate(g["arms"])), ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g["apex"]),
                 ("cage up", g["cage_up"]), ("cage low", g["cage_low"]), ("wheels", trimesh.util.concatenate(g["wheels"]))]
        _, mg = l2_clear_margin(g, g["B"], items)
        if mg >= 1.0 and guard_v_gap(g) >= GUARD_GAP - 0.05: hi = mid
        else: lo = mid
    return hi


def funnel(g, dc):
    others = trimesh.util.concatenate(g["arms"] + g["motors"] + [g["cage_low"], g["cage_up"], TOP, BOTTOM, FC, PI, DECK, PDB, UBEC, BUCK, RX, g["apex"], g["l2"], g["cam"],
                                       g["cam_bracket"], g["cart"], g["sled"], g["cam_pod"], g["flow_pod"]] + g["feet"])
    p = pts(others, 30000) + [dc, 0, SEAT_DZ]                     # funnel centre at the catch centre
    d_face = np.full(len(p), np.inf)
    for ax in (0, 1):
        for sg in (1, -1):
            x = sg * p[:, ax]; z = p[:, 2]; in_z = (z > -0.8) & (z < FACE_TOP) & (np.abs(p[:, 1 - ax]) < OPEN_HALF + 0.8 + z)
            d_face = np.where(in_z, np.minimum(d_face, (z + OPEN_HALF + 0.8 - x) / math.sqrt(2)), d_face)
    i = int(np.argmin(d_face))
    # hook latch (G (7)): extended plunger uphill of each ±y-face wheel, 12 wide at x ±100, slope 50.1..58.1 from the
    # bottom edge, 12 off the face; nothing of the drone within 1.0 of it
    global PLUNGER_CLEAR
    PLUNGER_CLEAR = np.inf
    for sy in (1, -1):
        y = sy * p[:, 1]; z = p[:, 2]
        s_up = ((y - OPEN_HALF) + (z + 0.8)) / math.sqrt(2); off = (z + OPEN_HALF + 0.8 - y) / math.sqrt(2)
        for sx in (1, -1):
            dx = np.maximum(np.abs(p[:, 0] - sx * 100.0) - 6.0, 0)
            ds = np.maximum(np.maximum(50.1 - s_up, s_up - 58.1), 0)
            do = np.maximum(np.maximum(-off, off - 12.0), 0)
            PLUNGER_CLEAR = min(PLUNGER_CLEAR, float(np.sqrt(dx ** 2 + ds ** 2 + do ** 2).min()))
    low = p[p[:, 2] < -0.8]; worst = 0.0
    for yaw in np.radians(np.linspace(-5, 5, 11)):
        c, s = math.cos(yaw), math.sin(yaw); q = low[:, :2] @ np.array([[c, -s], [s, c]]).T; worst = max(worst, np.abs(q).max())
    return d_face[i], p[i], worst


# ---------------------------------------------------------------- balance: dm (motors forward), dc (catch aft), wheel height
def leg_reach(g):
    return max(float(np.linalg.norm(np.subtract(b, a))) for k, (a, b) in g["rods"].items() if k.startswith("strut"))


def balance(dc, dm0):
    zB = math.ceil(l2_low_zB()); dm = dm0
    for _ in range(8):
        xB = solve_xB(lambda x: build(dm, dc, x, zB), zB)
        tot, cg = cg_of(build(dm, dc, xB, zB))
        if abs(cg[0] - dm) < 0.3: dm = cg[0]; break
        dm = cg[0]                                                # thrust centre (motor square centre) onto the CG
    xB = solve_xB(lambda x: build(dm, dc, x, zB), zB); g = build(dm, dc, xB, zB); tot, cg = cg_of(g)
    return dm, xB, zB, g, tot, cg


results = []
CASES = [(float(a), float(b)) for a, b in (c.split(",") for c in os.environ["V2_CASES"].split(";"))] if os.environ.get("V2_CASES") \
    else [(dc_, -12.0) for dc_ in (0.0, 10.0, 20.0, 30.0)]      # V2_CASES="30,-30" reruns one case from that wheel height
for dc, wz0 in CASES:
    dm = 60.0
    for wz in np.arange(wz0, -52.0, -2.0):
        set_wheel(wz)
        dm, xB, zB, g, tot, cg = balance(dc, dm)
        fd, fp, low = funnel(g, dc)
        if fd >= 1.0 and low <= OPEN_HALF - 1.0 and PLUNGER_CLEAR >= 1.0: break
    reach = leg_reach(g)
    results.append(dict(dc=dc, wz=wz, dm=dm, xB=xB, zB=zB, fd=fd, low=low, tot=tot, cg=cg, reach=reach, ok=fd >= 1.0 and PLUNGER_CLEAR >= 1.0))
    say(f"catch {dc:4.1f} aft: wheels z {wz:5.1f} (seat dz {SEAT_DZ:5.1f}), motors {dm:5.1f} ahead, L2 base ({xB:5.1f}, {zB:4.0f}), "
        f"funnel {fd:5.1f}, plunger {PLUNGER_CLEAR:5.1f}, strut {reach:4.0f}, {tot:4.0f} g")
ok = [r for r in results if r["ok"]]
best = min(ok, key=lambda r: r["dm"])                              # shortest front arms that pass the funnel (DEFAULT; the legs grow)
dc, dm, xB, zB = best["dc"], best["dm"], best["xB"], best["zB"]
set_wheel(best["wz"])

# camera: lowest height on the L2 back plane where the L2 leaves the view (lower edge + 2 deg), aimed at the horizon
CAM_UP = 0.0
for h in np.arange(30, 160, 2.0):
    g = build(dm, dc, xB, zB, CAM_UP, h)
    d = pts(g["l2"], 4000) - g["cam_lens"]; z = d @ g["cam_fwd"]
    elev = np.degrees(np.arctan2(d @ g["cam_up_v"], np.maximum(z, 1e-6))); azim = np.degrees(np.arctan2(d[:, 1], np.maximum(z, 1e-6)))
    inview = (z > 0) & (elev > -CAM_VFOV / 2 - 2.0) & (np.abs(azim) < CAM_HFOV / 2 + 2.0)
    if not inview.any(): CAM_H = h; break
g = build(dm, dc, xB, zB, CAM_UP, CAM_H)
tot, cg = cg_of(g)

# ---------------------------------------------------------------- checks on the chosen layout
mx_front = g["motors_xy"][0]
r_front = math.hypot(*mx_front); r_rear = math.hypot(*g["motors_xy"][1])
say(f"\nchosen: catch {dc:.0f} aft of the plate centre, motor square {dm:.1f} ahead; motors at ({dm + HALF:.1f}, ±{HALF}) and ({dm - HALF:.1f}, ±{HALF})")
say(f"arms: front r {r_front:.0f} (stock 243), rear r {r_rear:.0f}; tip gaps 25 all round (square kept)")
with contextlib.redirect_stdout(io.StringIO()):
    arm_reach.H_TIP, arm_reach.W_ROOT, arm_reach.W_TIP = H_TIP_FRONT, 34.0, 28.0; ff, _, mf = arm_reach.first_bending_hz(2 * r_front, ROOT_H)
    arm_reach.H_TIP, arm_reach.W_ROOT, arm_reach.W_TIP = H_TIP_REAR, 26.0, 22.0; fr, _, mr = arm_reach.first_bending_hz(2 * r_rear, ROOT_H)
say(f"first vertical bending (E): front {ff:.0f} Hz ({H_TIP_FRONT:.0f} deep at the motor), rear {fr:.0f} Hz; band 1P 87..158, 2P 174..317")
say(f"L2: base centre ({xB:.1f}, 0, {zB:.0f}), axis 15 deg below +x; body lowest z {g['l2'].bounds[0][2]:.1f} vs funnel top {FUNNEL_TOP_D:.1f} (drone frame)")
allitems = [("props", trimesh.util.concatenate(g["props"])), ("motors", trimesh.util.concatenate(g["motors"])), ("arms", trimesh.util.concatenate(g["arms"])),
            ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g["apex"]), ("cage up", g["cage_up"]), ("cage low", g["cage_low"]),
            ("wheels", trimesh.util.concatenate(g["wheels"])), ("front camera", g["cam"]), ("cam bracket", g["cam_bracket"]),
            ("cartridge", g["cart"]), ("pods", trimesh.util.concatenate([g["cam_pod"], g["flow_pod"]])), ("feet", trimesh.util.concatenate(g["feet"])),
            ("Pi", PI), ("rear wing", trimesh.util.concatenate([PDB, UBEC, BUCK, RX]))]
nm, mg = l2_clear_margin(g, g["B"], allitems)
say(f"L2 field (96 deg, negative-angle mode, kept clear in both modes): closest {nm} {mg:.1f} behind it (C, > 0 = clear)")
nose = max(g["l2"].bounds[1][0], g["cam"].bounds[1][0])
say(f"length: rear prop tips x {dm - HALF - PROP_R:.0f} to the nose x {nose:.0f} ({nose - (dm - HALF - PROP_R):.0f}); width 533; "
    f"height {max(g['cam_bracket'].bounds[1][2], g['cam'].bounds[1][2], PROP_Z) - g['feet'][0].bounds[0][2]:.0f}")
say(f"front camera: horizon, portrait, lens at ({g['cam_lens'][0]:.0f}, 0, {g['cam_lens'][2]:.0f}), {CAM_H:.0f} up the L2 back plane: "
    f"the L2 leaves its view 2 deg under the lower edge")
for name, m in (("props", trimesh.util.concatenate(g["props"])), ("arms", trimesh.util.concatenate(g["arms"])), ("motors", trimesh.util.concatenate(g["motors"]))):
    d = pts(m) - g["cam_lens"]; z = d @ g["cam_fwd"]
    el = np.degrees(np.arctan2(d @ g["cam_up_v"], np.maximum(z, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(z, 1e-6)))
    n = ((z > 0) & (abs(el) < CAM_VFOV / 2) & (abs(az) < CAM_HFOV / 2)).sum()
    if n: say(f"   {name} in the front camera view: {n} points")
say("front camera view: checked (lines above list anything in view)")
dn = np.array([0, 0, -1.0])
for name, m in (("feet", trimesh.util.concatenate(g["feet"])), ("cartridge", g["cart"]), ("wheels", trimesh.util.concatenate(g["wheels"])),
                ("cage low", g["cage_low"]), ("arms", trimesh.util.concatenate(g["arms"]))):
    for lens, half_h, half_v, label in ((g["down_lens"], DOWN_HFOV / 2, DOWN_VFOV / 2, "down camera"), (g["flow_lens"], FLOW_HALF, FLOW_HALF, "flow")):
        d = pts(m) - lens; zz = -d[:, 2]
        h = np.degrees(np.arctan2(d[:, 0], np.maximum(zz, 1e-6))); v = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
        n = ((zz > 0) & (abs(h) < half_h) & (abs(v) < half_v)).sum()
        if n: say(f"   {name} in the {label} view: {n} points")
say("down camera and flow views: checked (lines above list anything in view)")
bad = []
hubp = {"FC": FC, "Pi deck": DECK, "Pi 5": PI, "PDB": PDB, "UBEC": UBEC, "buck": BUCK, "RX": RX, "apex": g["apex"]}
plates = {"top plate": TOP, "bottom plate": BOTTOM, **{f"arm {i}": a for i, a in enumerate(g["arms"])}}
names = list(hubp)
for i, a in enumerate(names):
    for b in names[i + 1:] + list(plates):
        if a == "apex" and b in ("top plate", "bottom plate"): continue   # the front hub bolts to them
        v = overlap(hubp[a], {**hubp, **plates}[b])
        if v > 0.5: bad.append((a, b, v))
for p_, nmp in ((g["props"][i], f"prop {i}") for i in range(4)):
    for b, m in (("apex", g["apex"]), ("cage up", g["cage_up"]), ("cage low", g["cage_low"]), ("L2", g["l2"]), ("front camera", g["cam"]), ("cam bracket", g["cam_bracket"])):
        v = overlap(p_, m)
        if v > 0.5: bad.append((nmp, b, v))
say("overlaps (M, > 0.5 mm3):", "none" if not bad else "; ".join(f"{a} x {b} {v:.0f}" for a, b, v in bad))
# guard: ring and V rods 12 off the swept disks in plan; spokes and keels under a disk 12 below the swept band
worst_ring, worst_under = np.inf, np.inf
for k, (a, b) in g["rods"].items():
    t = np.linspace(0, 1, 200)[:, None]; q = np.asarray(a, float) + t * (np.subtract(b, a))
    for mx, my in g["motors_xy"]:
        rr = np.hypot(q[:, 0] - mx, q[:, 1] - my)
        if k.startswith(("ring ", "V ")) and not k.startswith(("ring x", "ring y")):
            worst_ring = min(worst_ring, float((rr - PROP_R - ROD_R).min()))
        else:
            under = rr < PROP_R + ROD_R + GUARD_GAP
            if under.any(): worst_under = min(worst_under, float((PROP_Z - BLADE - (q[under, 2] + ROD_R)).min()))
say(f"guard: ring and V rods {worst_ring:.1f} off the disks in plan (≥ {GUARD_GAP:.0f}); spokes, struts and keels under a disk "
    f"{worst_under:.1f} below the swept band (≥ {GUARD_GAP:.0f})")
rod_len = {k: float(np.linalg.norm(np.subtract(b, a))) for k, (a, b) in g["rods"].items()}
cage_g = sum(rod_len.values()) * ROD_GPM + 8 * 0.25 + sum(abs(m.volume) for m in g["nodes"]) * 1.27e-3 * 0.6 \
    + sum(abs(m.volume) for m in g["ring_nodes"]) * 1.27e-3 * 0.5
say(f"cage: {len(rod_len)} rods, {sum(rod_len.values()) / 1000:.2f} m of 3.0 carbon ({sum(rod_len.values()) * ROD_GPM:.0f} g C), "
    f"longest {max(rod_len.values()):.0f} ({max(rod_len, key=rod_len.get)}); 4 corner nodes + 8 wheels + 8 ring nodes; ~{cage_g:.0f} g (E) without the apex")
fd, fp, low = funnel(g, dc)
say(f"funnel seated (catch centre on the funnel axis, bottom plate top at seat z {SEAT_DZ:.1f}): closest non-wheel point {fd:.1f} "
    f"at seat ({fp[0]:.0f}, {fp[1]:.0f}, {fp[2]:.0f}) (G ≥ 1.0); hook latch plungers {PLUNGER_CLEAR:.1f} (G (7) ≥ 1.0); "
    f"below seat z -0.8 (cartridge incl.) {low:.1f} at ±5 deg yaw vs {OPEN_HALF:.0f}")
say(f"mass ~{tot:.0f} g (E: FC 10 g alone + 4 ESCs on the arms, no steel plate; record 2.0 kg P was the stock frame and gear), "
    f"CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}) vs thrust centre ({dm:.1f}, 0): {cg[0] - dm:+.1f} in x")
FLOW_POS = (g["flow_lens"] - cg) * [1, -1, -1] / 1000
say(f"PX4 flow position from the CG (FRD, m): x {FLOW_POS[0]:+.3f}, y {FLOW_POS[1]:+.3f}, z {FLOW_POS[2]:+.3f}; "
    f"IMU (FC centre) from the CG: x {(0 - cg[0]) / 1000:+.3f}, z {-(9.75 - cg[2]) / 1000:+.3f}")

# ---------------------------------------------------------------- motion artifact + run record
s = Scene("Overseer cage v0", f"carbon rod cage round the props, wheel ring in the funnel, L2 on the cage apex 15 deg below forward; motors {dm:.0f} ahead, catch {dc:.0f} aft (rough, E)")
P = lambda n, m, grp, c, pr, note="": s.part(n, m, group=grp, color=c, printed=pr, note=note)


def prop_view(i, mx, my):
    from topo.part import man
    ring = man(cyl(PROP_R, PROP_Z - 0.5, PROP_Z + 0.5, mx, my, 96)) - man(cyl(PROP_R - 2.5, PROP_Z - 1, PROP_Z + 1, mx, my, 96))
    ring = ring.to_mesh(); ring = trimesh.Trimesh(ring.vert_properties[:, :3], ring.tri_verts)
    blade = box(-PROP_R, PROP_R, -9, 9, PROP_Z - 1.5, PROP_Z + 1.5); blade.apply_transform(rotz(30 + 60 * i)); blade.apply_translation([mx, my, 0])
    return trimesh.util.concatenate([ring, blade])


P("bottom plate", BOTTOM, "frame", "#cfcfcf", False, "S500 stock (Holybro STEP)")
P("top plate", TOP, "frame", "#d8d8d8", False, "S500 stock")
for i, a in enumerate(g["arms"]): P(f"arm {i + 1}", a, "printed", "#8899aa", True, "rough")
P("guard rings + spokes + V + keels", trimesh.util.concatenate([g["rod_meshes"][k] for k in g["rods"] if not k.startswith(("strut", "ring x", "ring y"))]), "cage", "#333333", False, "3.0 carbon")
P("wheel ring + struts", trimesh.util.concatenate([g["rod_meshes"][k] for k in g["rods"] if k.startswith(("strut", "ring x", "ring y"))]), "cage", "#444444", False, "3.0 carbon")
P("ring nodes", trimesh.util.concatenate(g["ring_nodes"]), "cage", "#7d8a99", True, "rough")
P("corner nodes", trimesh.util.concatenate(g["nodes"]), "cage", "#9aa8b8", True, "2 wheels each, Scout-style")
P("L2 apex + front hub", g["apex"], "cage", "#aab4c0", True, "rough")
P("cam bracket", g["cam_bracket"], "printed", "#aab4c0", True)
P("Pi deck", DECK, "printed", "#b8c0c8", True)
P("belly sled + pods", union([g["sled"], g["cam_pod"], g["flow_pod"]]), "printed", "#99a6b3", True)
P("feet", trimesh.util.concatenate(g["feet"]), "printed", "#555555", True, "TPU")
P("catch wheels", trimesh.util.concatenate(g["wheels"]), "cage", "#cc6644", True, "R 5")
P("Unitree L2", g["l2"], "bought", "#222222", False, "axis 15 deg below forward")
P("front camera", g["cam"], "bought", "#2d5a2d", False, "horizon, portrait")
P("down camera", g["down_cam"], "bought", "#2d5a2d", False)
P("flow MTF-02P", g["flow"], "bought", "#2d5a2d", False)
P("Pi 5", PI, "bought", "#2e7d32", False)
P("FC", FC, "bought", "#1b5e20", False)
P("PDB + UBEC + buck + RX", trimesh.util.concatenate([PDB, UBEC, BUCK, RX]), "bought", "#444488", False, "rear wing")
P("motors", trimesh.util.concatenate(g["motors"]), "bought", "#b0b0b0", False)
P("props", trimesh.util.concatenate([prop_view(i, *xy) for i, xy in enumerate(g["motors_xy"])]), "bought", "#d9a400", False)
P("4S 6000", g["pack"], "bought", "#2f4fbf", False)
names = [p["name"] for p in s.parts]
s.timeline(lambda t: {n: rotz(360 * t / 20) for n in names}, duration_s=20, fps=30)
s.view("iso", (600, -660, 520), (60, 0, 30)); s.view("side", (60, -1000, 60), (60, 0, 40)); s.view("front", (1000, 0, 60), (0, 0, 40))
s.view("top", (60, 0, 1100), (60, 0, 0)); s.view("under", (300, -300, -500), (0, 0, -20))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
try:
    s.video(os.path.join(OUT, "motion.mp4"))
except Exception as e:
    say("video skipped:", str(e)[:200])
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
