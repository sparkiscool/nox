"""Overseer system model (system pass, 24 Oct; CLAUDE.md "System first, then cut" and "Every drone design"): every part and
bought part in pose round the optimized full cage (`cage_opt2.py`), the pack on the thrust axis, the balance solved, the
wheel height searched for the funnel, then every rule check on the assembled model: cage first, fit, balance pack in and
out, compact (nox/system_check.py), the L2 field (props out, thin rods in it reported as a blocked share), the 12 mm prop
rules, the funnel and hook latch, landing tip-over, camera views and the frame model on the final geometry.

python3 nox/overseer/overseer_system.py [out_dir]   (SYS_CAGE=<cage_opt2.json> SYS_LEVEL=100Hz_35g; ~10 min)

Drone frame: x forward, y left, z up; z 0 = bottom plate top face; plate centre at x 0.
"""
import math, os, sys, io, contextlib
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.join(HERE, "s500")); sys.path.insert(0, HERE)
from motion import Scene
import s500_cad, arm_reach
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_system", "v1")
import json
import cage_opt2 as co
from system_check import run as system_check_run
_CJ = json.load(open(os.environ.get("SYS_CAGE", os.path.join(ROOT, "runs", "overseer_cage", "v2", "cage_opt2.json"))))
_LV = _CJ["levels"][os.environ.get("SYS_LEVEL", "100Hz_35g")]
CAGE_TOPO, CAGE_SIZES = _LV["topo"], _LV["sizes"]
SYS_PICK = os.environ.get("SYS_PICK", "mid_spokes")        # DEFAULT: +10 g over the lightest for a 178 Hz cage (Spark: "way too flimsy")
if SYS_PICK:
    want = set(SYS_PICK.split(",")) if SYS_PICK != "none" else set()
    for row in _LV["table"]:
        if row["feasible"] and {k for k, v in row["topo"].items() if v and k not in co.FIXED_ON} == want:
            CAGE_TOPO, CAGE_SIZES = row["topo"], row["sizes"]; break
TIP_OFF = float(os.environ.get("SYS_TIP_OFF", "0"))     # pack / catch centre behind the thrust axis: 0 = the balance rule (CG ≤ 3 mm pack in and out)
NOSE_GROUPS = ("nose", "nose_x", "nose_mount")                      # thin rods allowed in the L2 field (blocked share reported)


def sec_of(name, group):
    return CAGE_SIZES.get(name) or CAGE_SIZES.get(group) or "r3"
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
CAM_UP_DEFAULT = 30.0                    # front camera 30 deg up (the 23 Oct choice; a lower bracket than the horizon one, compact)
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
FC = box(4, 40, -18, 18, 1.0, 18.5)                          # FC (+ stack) 36 x 36 x 17.5 (spec), front half of the hub
DECK = union([box(-44.5, 44.5, -30, 30, 20.0, 22.0)] + [box(sx * 41 - 2, sx * 41 + 2, sy * 28 - 2, sy * 28 + 2, 0, 20) for sx in (1, -1) for sy in (1, -1)])
PI = box(-42.5, 42.5, -28, 28, 22.5, 42.0)                   # Pi 5 with cooler (E height)
PDB = box(-55, 0, -24.5, 24.5, 0.5, 6.5)                     # FCHUB-12S under the Pi deck, rear half (spec): inside the cage (system pass)
UBEC = box(-52, -4, 1, 25, 7.5, 17.5)                        # on the PDB, under the deck
BUCK = box(-40, -22, -24, -6, 7.5, 13.5)
RX = box(-52, -40, -22, -10, 7.5, 13.5)
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
    g["feet"] = [cyl(5, PB - 9, PB + 4, -dc + sx * 72, sy * 66) for sx in (1, -1) for sy in (1, -1)]   # widest inside the opening at ±5 deg yaw

    # guard: two octagon rings round the motor square, 12 off the swept disks; the front flat is a V to the L2 apex
    B = np.array([xB, 0.0, zB]); g["B"] = B
    g["l2"] = l2_mesh(B)
    up_b = np.array([-A[2], 0, A[0]])                            # in the base plane, toward +z: (0.259, 0, 0.966)
    seat = cyl(37.5, -5, 0, 0, 0); T = trimesh.transformations.rotation_matrix(math.radians(L2_TILT), [0, 1, 0]); T[:3, 3] = B
    seat.apply_transform(T)
    back = box(-22, 22, -16, 16, -11, -5); back.apply_transform(T)               # apex block behind the seat (L2 local z -11..-5)
    # apex rod sockets on side lugs beside the L2 body (y ±46), just behind the base plane, so the V never bends back
    # inside the front flat (the guard keeps 12 off the front disks)
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
    cam_rods = [(to_w((-3.5, sy * 16.0, -21.5)), B - A * 8 + up_b * 20 + np.array([0, sy * 16.0, 0])) for sy in (1, -1)]
    cam_rods.append((to_w((-3.5, 0.0, -21.5)), B - A * 11 + up_b * 20))
    g["cam_rods"] = cam_rods
    g["cam_plate"] = plate
    g["cam_bracket"] = trimesh.util.concatenate([plate] + [rod(a, b) for a, b in cam_rods])
    sock = co.sockets(xB, zB)
    lugs = []
    for sy in (1, -1):                                             # lugs spanning the socket spread (local x = -up_b)
        h = co.APEX_SPAN / 2 + 6
        lug = box(-h, h, 36 if sy > 0 else -50, 50 if sy > 0 else -36, -8, -2); lug.apply_transform(T); lugs.append(lug)
    cb = g["cam"].bounds
    P = dict(dm=dm, xc=-dc, wz=WHEEL_Z, xB=xB, zB=zB, cam_top=float(max(cb[1][2], g["cam_bracket"].bounds[1][2])), cam_front=float(cb[1][0]),
             m_drone_g=3000.0, rear_x=-100.0, pyramid="rear", x_top=dm, x_tower=-100.0)   # crash mass: the A system is ~2.95 kg (E), no cap
    g["P"] = P
    Ng, Mg, fixed_g, _, info = co.geometry(P, CAGE_TOPO, CAGE_SIZES)
    for name, a, b, group, kind in Mg:
        if kind == "rod":
            sc = sec_of(name, group); rods[name] = (Ng[a], Ng[b], co.SECTIONS[sc]["od"], group, sc)
    ring_nodes = []
    for nm in info["pnodes"]:
        for lvl in ("lo", "hi"):
            q = Ng[f"{nm}{lvl}"]; ring_nodes.append(box(q[0] - 6, q[0] + 6, q[1] - 6, q[1] + 6, q[2] - 6, q[2] + 6))
    for n in [n_ for n_ in Ng if n_ == "TR" or n_.startswith(("KV", "KP"))]:   # rear tower and skirt nodes (cage up)
        q = Ng[n]; ring_nodes.append(box(q[0] - 6, q[0] + 6, q[1] - 6, q[1] + 6, q[2] - 6, q[2] + 6))
    for n in ("NT+1", "NT-1", "NB+1", "NB-1", "TN"):
        q = Ng[n]; ring_nodes.append(box(q[0] - 6, q[0] + 6, q[1] - 6, q[1] + 6, q[2] - 6, q[2] + 6))
    g["ring_v"] = [(r[1], r[2]) for r in info["ring"] if r[3]]
    hubs = [box(Ng[n][0] - 7, Ng[n][0] + 7, Ng[n][1] - 7, Ng[n][1] + 7, Ng[n][2] - 4, Ng[n][2] + 4)
            for n in Ng if n.startswith(("fhubB", "fhubT", "chin"))]
    g["hubs"] = hubs
    g["cage_N"], g["cage_info"] = Ng, info
    g["rods"] = rods
    g["rod_meshes"] = {k: rod(v[0], v[1], v[2] / 2) for k, v in rods.items()}
    g["ring_nodes"] = ring_nodes
    g["apex"] = union([seat, back] + lugs + hubs)
    low_keys = [k for k, v in rods.items() if v[3] in ("struts", "strut2", "wheel_ring")]
    nose_keys = [k for k, v in rods.items() if v[3] in NOSE_GROUPS or k in ("spider nose+", "spider nose-")]
    g["cage_low"] = trimesh.util.concatenate(nodes + [g["rod_meshes"][k] for k in low_keys])
    g["cage_up"] = trimesh.util.concatenate([n_ for n_ in ring_nodes[:-5]] + [g["rod_meshes"][k] for k in rods if k not in low_keys and k not in nose_keys])
    g["cage_nose"] = trimesh.util.concatenate(ring_nodes[-5:] + [g["rod_meshes"][k] for k in nose_keys])
    g["nose_keys"] = nose_keys
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
    for k, v in g["rods"].items():                                             # carbon per its section (C)
        a, b, od, grp, sc = v; gpm = co.SECTIONS[sc]["gpm"] * 1e-3
        L = float(np.linalg.norm(np.subtract(b, a))); items.append((k, L * gpm, trimesh.points.PointCloud([np.add(a, b) / 2])))
    for i_, r_ in enumerate(g["cam_rods"]):
        a, b = r_; L = float(np.linalg.norm(np.subtract(b, a))); items.append((f"cam rod {i_}", L * ROD_GPM, trimesh.points.PointCloud([np.add(a, b) / 2])))
    if CAGE_TOPO.get("arm_spars"):                                             # 4 x 3.0 spars in each arm box (C)
        for i_, (mx, my) in enumerate(g["motors_xy"]):
            r0 = np.array([75 * math.cos(math.radians(MOTOR_DEG[i_])), 75 * math.sin(math.radians(MOTOR_DEG[i_]))])
            L = float(np.hypot(mx - r0[0], my - r0[1])); items.append((f"spars {i_}", 4 * L * ROD_GPM, trimesh.points.PointCloud([[(mx + r0[0]) / 2, (my + r0[1]) / 2, 22]])))
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
    """lowest base-centre height that keeps the nose box bottom rods (L2 lowest - NOSE_GAP - rod r) 1.0 over the funnel top"""
    m = l2_mesh(np.zeros(3)); return FUNNEL_TOP_D + 1.0 + 2.5 + co.NOSE_GAP - m.bounds[0][2]


def guard_v_gap(g):
    """plan gap of the V rods to the front disks (≥ GUARD_GAP)"""
    worst = np.inf
    for k, v in g["rods"].items():
        if v[3] != "V": continue
        a, b = v[0], v[1]
        t = np.linspace(0, 1, 200)[:, None]; q = np.asarray(a, float) + t * (np.subtract(b, a))
        for mx, my in g["motors_xy"]:
            worst = min(worst, float((np.hypot(q[:, 0] - mx, q[:, 1] - my) - PROP_R - v[2] / 2).min()))
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
    return max(float(np.linalg.norm(np.subtract(v[1], v[0]))) for k, v in g["rods"].items() if v[3] in ("struts", "strut2"))


def balance(dc_unused, dm0):
    zB = math.ceil(l2_low_zB()); dm = dm0
    for _ in range(10):
        dc = TIP_OFF - dm
        xB = solve_xB(lambda x: build(dm, dc, x, zB), zB)
        tot, cg = cg_of(build(dm, dc, xB, zB))
        if abs(cg[0] - dm) < 0.3: dm = cg[0]; break
        dm = cg[0]                                                # thrust centre (motor square centre) onto the CG
    dc = TIP_OFF - dm
    xB = solve_xB(lambda x: build(dm, dc, x, zB), zB); g = build(dm, dc, xB, zB); tot, cg = cg_of(g)
    return dm, xB, zB, g, tot, cg


say(f"cage (cage_opt level {os.environ.get('V3_LEVEL', '160')} Hz): " + " ".join(k for k, v in CAGE_TOPO.items() if v)
    + " | " + " ".join(f"{k}:{v}" for k, v in CAGE_SIZES.items()))
dm = 80.0
for wz in np.arange(float(os.environ.get("V3_WZ0", "-30")), -60.0, -2.0):
    set_wheel(wz)
    dm, xB, zB, g, tot, cg = balance(None, dm)
    dc = TIP_OFF - dm
    fd, fp, low = funnel(g, dc)
    say(f"wheels z {wz:5.1f}: motors {dm:5.1f} ahead, catch centre x {-dc:5.1f}, L2 base ({xB:5.1f}, {zB:4.0f}), funnel {fd:5.1f}, "
        f"plungers {PLUNGER_CLEAR:4.1f}, below-opening {low:4.1f}, {tot:4.0f} g")
    if fd >= 1.0 and low <= OPEN_HALF - 1.0 and PLUNGER_CLEAR >= 1.0: break
dc = TIP_OFF - dm

# camera: lowest height on the L2 back plane where the L2 leaves the view (lower edge + 2 deg), aimed at the horizon
CAM_UP = CAM_UP_DEFAULT
for h in np.arange(20, 160, 2.0):
    g = build(dm, dc, xB, zB, CAM_UP, h)
    d = pts(g["l2"], 4000) - g["cam_lens"]; z = d @ g["cam_fwd"]
    elev = np.degrees(np.arctan2(d @ g["cam_up_v"], np.maximum(z, 1e-6))); azim = np.degrees(np.arctan2(d[:, 1], np.maximum(z, 1e-6)))
    inview = (z > 0) & (elev > -CAM_VFOV / 2 - 2.0) & (np.abs(azim) < CAM_HFOV / 2 + 2.0)
    if not inview.any(): CAM_H = h; break
g = build(dm, dc, xB, zB, CAM_UP, CAM_H)
tot, cg = cg_of(g)
CAM_DESC = f"{CAM_UP:.0f} deg up, portrait"

# ---------------------------------------------------------------- checks on the chosen layout
mx_front = g["motors_xy"][0]
r_front = math.hypot(*mx_front); r_rear = math.hypot(*g["motors_xy"][1])
say(f"\nchosen: catch centre x {-dc:.1f} ({TIP_OFF:.0f} behind the thrust centre), motor square {dm:.1f} ahead of the plate centre; motors at ({dm + HALF:.1f}, ±{HALF}) and ({dm - HALF:.1f}, ±{HALF})")
say(f"arms: front r {r_front:.0f} (stock 243), rear r {r_rear:.0f}; tip gaps 25 all round (square kept)")
P_FINAL = g["P"]
FRAME = co.evaluate(P_FINAL, CAGE_TOPO, CAGE_SIZES)
cs = FRAME["cases"]
def worst_of(prefix, key):
    sel = {k: v for k, v in cs.items() if k.startswith(prefix)}
    k = max(sel, key=lambda k: sel[k][key]); return k, sel[k][key]
cr = {k: v for k, v in cs.items() if k.startswith("crash")}
wdef = max(cr, key=lambda k: cr[k]["defl"])
say(f"frame model on the final geometry (F, dimensions L cases): L2 mode {FRAME['f_l2']:.0f} Hz (≥ {co.F_L2_MIN:.0f}), cage first mode {FRAME['f_first']:.0f} Hz; "
    f"crash {co.CRASH_G:.0f} g x {P_FINAL['m_drone_g']:.0f} g in 14 directions: worst rod {max(v['rod'] for v in cr.values()):.2f}, buckling {max(v['buckle'] for v in cr.values()):.2f}, "
    f"sockets {max(v['socket'] for v in cr.values()):.2f}, arm {max(v['arm'] for v in cr.values()):.2f} (≤ 1.0), touching node moves ≤ {cr[wdef]['defl']:.1f} (≤ 9); "
    f"L2 at {co.CRASH_G:.0f} g: worst {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('L2 crash')):.2f}; "
    f"handling 50 N: {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('handling')):.2f}; "
    f"latch {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('latch')):.2f}; violation {FRAME['violation']:.3f}; "
    f"breaking order: node {FRAME['breaking_order']['node']:.2f} vs member {FRAME['breaking_order']['member']:.2f}")
say(f"L2: base centre ({xB:.1f}, 0, {zB:.0f}), axis 15 deg below +x; body lowest z {g['l2'].bounds[0][2]:.1f} vs funnel top {FUNNEL_TOP_D:.1f} (drone frame)")
allitems = [("props", trimesh.util.concatenate(g["props"])), ("motors", trimesh.util.concatenate(g["motors"])), ("arms", trimesh.util.concatenate(g["arms"])),
            ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g["apex"]), ("cage up", g["cage_up"]), ("cage low", g["cage_low"]),
            ("wheels", trimesh.util.concatenate(g["wheels"])), ("front camera", g["cam"]), ("cam bracket", g["cam_bracket"]),
            ("cartridge", g["cart"]), ("pods", trimesh.util.concatenate([g["cam_pod"], g["flow_pod"]])), ("feet", trimesh.util.concatenate(g["feet"])),
            ("Pi", PI), ("rear wing", trimesh.util.concatenate([PDB, UBEC, BUCK, RX]))]
nm, mg = l2_clear_margin(g, g["B"], allitems)
say(f"L2 field ({90 + 2 * L2_NEG:.0f} deg, {'negative-angle' if L2_NEG else 'normal'} mode kept clear): closest {nm} {mg:.1f} behind it (C, > 0 = clear)")
# blocked share of the L2 field by the thin nose rods (allowed: CLAUDE.md "clear view": thin rods only, share reported)
rng = np.random.default_rng(1); dirs = rng.normal(size=(40000, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
dirs = dirs[dirs @ A > 0]; origin = g["B"] + A * 30                               # field hemisphere about the L2 axis, from its head (E)
blocked = np.zeros(len(dirs), bool)
for k in g["nose_keys"]:
    a_, b_, od_, _, _ = g["rods"][k]; a_, b_ = np.asarray(a_, float), np.asarray(b_, float)
    u_ = b_ - a_; w0 = origin - a_; aa = dirs @ dirs.T[:, :1].T if False else None
    # closest distance between each ray (origin + t d, t > 0) and the segment
    uu = u_ @ u_; ud = dirs @ u_; dw = dirs @ w0; uw = u_ @ w0
    den = uu - ud ** 2; sc_ = np.clip((ud * dw - uw) / np.maximum(den, 1e-9), 0, 1); tc = np.maximum(sc_ * ud - dw, 0)
    dist = np.linalg.norm((w0[None, :] + tc[:, None] * dirs) - sc_[:, None] * u_[None, :], axis=1)
    blocked |= dist < od_ / 2
say(f"L2 field blocked by the thin nose rods: {blocked.mean() * 100:.1f} % of the hemisphere (C, rays from the L2 head)")
nose = max(g["l2"].bounds[1][0], g["cam"].bounds[1][0], g["cage_nose"].bounds[1][0])
say(f"length: rear prop tips x {dm - HALF - PROP_R:.0f} to the nose x {nose:.0f} ({nose - (dm - HALF - PROP_R):.0f}); width 533; "
    f"height {max(g['cam_bracket'].bounds[1][2], g['cam'].bounds[1][2], PROP_Z) - g['feet'][0].bounds[0][2]:.0f}")
say(f"front camera: {CAM_DESC}, lens at ({g['cam_lens'][0]:.0f}, 0, {g['cam_lens'][2]:.0f}), {CAM_H:.0f} up the L2 back plane: "
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
for k, v in g["rods"].items():
    a, b, od, grp, sc = v
    t = np.linspace(0, 1, 200)[:, None]; q = np.asarray(a, float) + t * (np.subtract(b, a))
    for mx, my in g["motors_xy"]:
        rr = np.hypot(q[:, 0] - mx, q[:, 1] - my)
        if grp in ("chords", "V", "ring_diag", "posts"):
            worst_ring = min(worst_ring, float((rr - PROP_R - od / 2).min()))
        else:
            under = rr < PROP_R + od / 2 + GUARD_GAP
            if under.any():                                  # clear below the swept band or above it
                zq = q[under, 2]
                worst_under = min(worst_under, float(np.maximum(PROP_Z - BLADE - (zq + od / 2), zq - od / 2 - PROP_Z - BLADE).min()))
say(f"guard: ring and V rods {worst_ring:.1f} off the disks in plan (≥ {GUARD_GAP:.0f}); rods under a disk "
    f"{worst_under:.1f} off the swept band (≥ {GUARD_GAP:.0f})")
by_sec = {}
for k, v in g["rods"].items():
    sec = v[4]; L = float(np.linalg.norm(np.subtract(v[1], v[0])))
    n_, Ls = by_sec.get(sec, (0, 0.0)); by_sec[sec] = (n_ + 1, Ls + L)
rod_g = sum(Ls * co.SECTIONS[sec]["gpm"] / 1000 for sec, (n_, Ls) in by_sec.items())
spar_g = 0.0
if CAGE_TOPO.get("arm_spars"):
    for i_, (mx, my) in enumerate(g["motors_xy"]):
        r0 = np.array([75 * math.cos(math.radians(MOTOR_DEG[i_])), 75 * math.sin(math.radians(MOTOR_DEG[i_]))]); spar_g += 4 * float(np.hypot(mx - r0[0], my - r0[1])) * ROD_GPM
cage_g = rod_g + spar_g + 8 * 0.25 + sum(abs(m.volume) for m in g["nodes"]) * 1.27e-3 * 0.6 + sum(abs(m.volume) for m in g["ring_nodes"]) * 1.27e-3 * 0.5
say("cage rods (C): " + "; ".join(f"{co.SECTIONS[sec]['label']} {n_} x {Ls / 1000:.2f} m" for sec, (n_, Ls) in sorted(by_sec.items(), key=lambda x: co.SECTIONS[x[0]]['gpm']))
    + f"; {rod_g:.0f} g + arm spars {spar_g:.0f} g; {len(g['ring_nodes'])} ring nodes, 4 corner nodes, 8 wheels; ~{cage_g:.0f} g (E) without the apex")
# landing: tip-over angle on the 4 feet (the lowest points), CG over the stance
fz = min(f.bounds[0][2] for f in g["feet"]); fxy = np.array([f.bounds.mean(axis=0)[:2] for f in g["feet"]])
x0, x1, y0, y1 = fxy[:, 0].min() - 5, fxy[:, 0].max() + 5, fxy[:, 1].min() - 5, fxy[:, 1].max() + 5
hcg = cg[2] - fz
edges = {"forward": x1 - cg[0], "back": cg[0] - x0, "left": y1 - cg[1], "right": cg[1] - y0}
tips = {k: math.degrees(math.atan2(v, hcg)) for k, v in edges.items()}
say(f"landing: feet at z {fz:.0f} (lowest points), stance x {x0:.0f}..{x1:.0f}, y {y0:.0f}..{y1:.0f}; CG {hcg:.0f} above the floor; "
    f"tip-over " + ", ".join(f"{k} {v:.1f} deg" for k, v in tips.items()) + f" (worst {min(tips.values()):.1f}, DEFAULT ≥ 30)")
fd, fp, low = funnel(g, dc)
say(f"funnel seated (catch centre on the funnel axis, bottom plate top at seat z {SEAT_DZ:.1f}): closest non-wheel point {fd:.1f} "
    f"at seat ({fp[0]:.0f}, {fp[1]:.0f}, {fp[2]:.0f}) (G ≥ 1.0); hook latch plungers {PLUNGER_CLEAR:.1f} (G (7) ≥ 1.0); "
    f"below seat z -0.8 (cartridge incl.) {low:.1f} at ±5 deg yaw vs {OPEN_HALF:.0f}")
say(f"mass ~{tot:.0f} g (E: FC 10 g alone + 4 ESCs on the arms, no steel plate; record 2.0 kg P was the stock frame and gear), "
    f"CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}) vs thrust centre ({dm:.1f}, 0): {cg[0] - dm:+.1f} in x")
# balance pack in and out (CLAUDE.md "Balanced"): the pack and cartridge on the thrust axis
pk = g["pack"].bounding_box.centroid; cart_g = abs(g["cart"].volume) * 1.27e-3 * 0.04
cg_out = (cg * tot - pk * 650 - g["cart"].centroid * cart_g) / (tot - 650 - cart_g)
say(f"balance: CG {np.hypot(cg[0] - dm, cg[1]):.1f} off the thrust axis with the pack, {np.hypot(cg_out[0] - dm, cg_out[1]):.1f} without (≤ 3)")
# system_check manifest (cage first, fit, compact, mass; grouped meshes)
def tm(m): return m.copy()
cage_parts = [g["cage_up"], g["cage_low"], g["cage_nose"], trimesh.util.concatenate(g["wheels"]), trimesh.util.concatenate(g["feet"])]
man = dict(name="overseer_system_v1", cg_ref=[dm, 0.0, 0.0], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0, n_dirs=1500, parts=[
    dict(name="cage", trimesh=trimesh.util.concatenate(cage_parts), role="cage", mass_g=cage_g, tag="E"),
    dict(name="props", trimesh=trimesh.util.concatenate([cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, *xy, 64) for xy in g["motors_xy"]]), role="protected", mass_g=40, tag="E"),
    dict(name="motors", trimesh=trimesh.util.concatenate(g["motors"]), role="protected", mass_g=324, tag="P"),
    dict(name="L2", trimesh=g["l2"], role="protected", mass_g=230, tag="P"),
    dict(name="front camera", trimesh=g["cam"], role="protected", mass_g=17, tag="E"),
    dict(name="pack", trimesh=g["pack"], role="protected", mass_g=650, tag="E", swap=True),
    dict(name="electronics", trimesh=trimesh.util.concatenate([FC, PI, PDB, UBEC, BUCK, RX]), role="protected", mass_g=110 + 80, tag="E"),
    dict(name="down cam + flow", trimesh=trimesh.util.concatenate([g["down_cam"], g["flow"]]), role="protected", mass_g=17, tag="E"),
    dict(name="arms", trimesh=trimesh.util.concatenate(g["arms"]), role="structure", mass_g=sum(abs(a.volume) for a in g["arms"]) * 1.27e-3 * 0.45 + 100, tag="E"),
    dict(name="plates", trimesh=trimesh.util.concatenate([TOP, BOTTOM]), role="structure", mass_g=84, tag="C"),
    dict(name="sled + pods + deck", trimesh=trimesh.util.concatenate([g["sled"], g["cam_pod"], g["flow_pod"], DECK]), role="structure",
         mass_g=(abs(g["sled"].volume) + abs(g["cam_pod"].volume) + abs(g["flow_pod"].volume)) * 1.27e-3 * 0.3 + abs(DECK.volume) * 1.27e-3 * 0.5, tag="E"),
    dict(name="apex + hubs", trimesh=g["apex"], role="structure", mass_g=abs(g["apex"].volume) * 1.27e-3 * 0.45, tag="E"),
    dict(name="cam bracket rods", trimesh=g["cam_bracket"], role="structure", mass_g=6, tag="E")])
try:
    with contextlib.redirect_stdout(io.StringIO()):
        SYS = system_check_run(man)
    say("system_check: " + SYS["summary"].replace("\n", " | "))
except Exception as e:
    SYS = None; say(f"system_check failed to run: {str(e)[:200]}")
FLOW_POS = (g["flow_lens"] - cg) * [1, -1, -1] / 1000
say(f"PX4 flow position from the CG (FRD, m): x {FLOW_POS[0]:+.3f}, y {FLOW_POS[1]:+.3f}, z {FLOW_POS[2]:+.3f}; "
    f"IMU (FC centre) from the CG: x {(0 - cg[0]) / 1000:+.3f}, z {-(9.75 - cg[2]) / 1000:+.3f}")

# ---------------------------------------------------------------- motion artifact + run record
s = Scene("Overseer system v1", f"full cage (frame model): L2 mode {FRAME['f_l2']:.0f} Hz, crash {co.CRASH_G:.0f} g, tip-over ≥ {min(tips.values()):.0f} deg; pack {TIP_OFF:.0f} behind the thrust axis, motors {dm:.0f} ahead of the plates")
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
SEC_COL = {"r3": "#3a3a3a", "r4": "#2b4f7a", "t6": "#3f7a3f", "r5": "#7a5a2b", "t8": "#8a3f8a", "t10": "#a33a3a"}
for sec in sorted(set(v[4] for v in g["rods"].values()), key=lambda s_: co.SECTIONS[s_]["gpm"]):
    ks = [k for k, v in g["rods"].items() if v[4] == sec]
    if ks: P(f"rods {co.SECTIONS[sec]['label']}", trimesh.util.concatenate([g["rod_meshes"][k] for k in ks]), "cage", SEC_COL[sec], False,
             ", ".join(sorted(set(g["rods"][k][3] for k in ks))))
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
json.dump({k: float(v) for k, v in P_FINAL.items()}, open(os.path.join(OUT, "params.json"), "w"), indent=1)
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
