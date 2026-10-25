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
import cam_cover
from system_check import run as system_check_run
_CJ = json.load(open(os.environ.get("SYS_CAGE", os.path.join(ROOT, "runs", "overseer_cage", "v3", "cage_opt2.json"))))
_LV = _CJ["levels"][os.environ.get("SYS_LEVEL", "100Hz_35g")]
CAGE_TOPO, CAGE_SIZES = _LV["topo"], _LV["sizes"]
SYS_PICK = os.environ.get("SYS_PICK", "")                 # DEFAULT: the lightest feasible layout of the level (v3: all ~105 Hz); a name list picks that row
if SYS_PICK:
    want = set(SYS_PICK.split(",")) if SYS_PICK != "none" else set()
    for row in _LV["table"]:
        if row["feasible"] and {k for k, v in row["topo"].items() if v and k not in co.FIXED_ON} == want:
            CAGE_TOPO, CAGE_SIZES = row["topo"], row["sizes"]; break
if os.environ.get("SYS_TOPO"):                                      # start from a named topology (fixed options always on); the loop re-sizes it
    CAGE_TOPO = {k: (k in co.FIXED_ON or k in os.environ["SYS_TOPO"].split(",")) for k in co.TOPO_KEYS}
SWAP_BACK = float(os.environ.get("SYS_SWAP_BACK", "30"))   # layout B (Spark 24 Oct): the swap set's CG 30 behind the thrust axis
MAST_X_MIN = 105.0 + 13.0 + 1.0                                # the mast node block (26 for a 20 tube) clears the front hub block (x 105) and the plate stack:
                                                              # the mast stands there when the thrust centre is further back
M_DRONE = 3000.0                                            # crash mass, updated to the assembled mass every balance (g)
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
L2_NEG = 6.0                             # scan kept clear in the 96 deg negative-angle mode (Spark 24 Oct: "account for the 15 degrees and the scan range")
L2_BASE_H = 25.0                         # the L2 base (Spark 24 Oct): held and caged; everything above it is exposed and nothing enters the scan
GUARD_TOP, GUARD_IN, GUARD_W = 17.0, 38.5, 2.5   # printed base shroud (cage, TPU): a square sleeve 1 off the base's 75 square faces, 2.5 wall, from the
                                                 # base plane to 17 up: its corners (r 58) are the highest it can stand outside the 96 deg scan (1.5 clear);
                                                 # the 8 of base above it is exposed with the head (DEFAULT; a ring left a gap a sharp edge drops into)
A = np.array([math.sin(math.radians(L2_TILT)), 0.0, math.cos(math.radians(L2_TILT))])   # (0.966, 0, -0.259)
CAM_HFOV, CAM_VFOV = 73.0, 89.0          # front camera portrait, 4:3 from 102 diag (E)
DOWN_HFOV, DOWN_VFOV = 89.0, 73.0
FLOW_HALF = 23.0
PACK = (148.0, 48.0, 45.0)               # 4S 6000 (P)
CART = (157.0, 54.0)                     # cartridge envelope (E, G)
WHEEL_R, WHEEL_W = 5.0, 3.0               # R 5 (v4 set): the hook latch is laid out on it (G Hook latch v1)
OPEN_HALF, FACE_TOP = 84.0, 75.1         # funnel (G): y opening half 84
FACE_C = 82.7                            # every face plane: half = 82.7 + seat z (v7 y edge 84 at z 1.3, v9 plain x edge 95.5 at z 12.8, G)
X_EDGE_Z, Y_EDGE_Z, OPEN_X = 12.8, 1.3, 95.5
WHEEL_SEAT = 112 - FACE_C + WHEEL_R * math.sqrt(2)   # seated wheel centre, seat z 36.4 (C)
PROBE_Z, PAD_OFF, G8_OFF = 33.0, 2.0, 9.0           # keep-alive probes at seat z 33 on the ±x faces, drone pad 2.0 off the face, G (8) band 9.0
# latch v1 set (PR 69 @ 258becf via runs/box_system/v2/latch_ctx, M): cartridge coords (x across, y up from the flat bottom,
# z from the closed end) -> drone: x = catch centre + (z - Z_T) (XT60 end forward: the pack offset points away from the
# box hub, B), y = x_c, seat z = y_c - 42.3 (R 113, Z_SEAT 155.3, C)
LATCH_DIR = os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx")
LATCH_META = json.load(open(os.path.join(LATCH_DIR, "meta.json")))["overseer"]
LATCH_MESH = {k: trimesh.load(os.path.join(LATCH_DIR, f"overseer_{k}_c.stl")) for k in ("cartridge", "pack", "receiver", "receiver_lid", "xt60e1m")}
LATCH_SEAT_Z0 = 113.0 - 155.3
SWAP_G, RECEIVER_G = LATCH_META["SWAP_M"], 190.0                    # cartridge + pack 748 g (M), receiver set 190.0 g (latch PR 69, E)
SWAP_FWD = LATCH_META["Z_T"] - LATCH_META["SWAP_CG"][2]              # swap CG 5.18 behind the set centre (the catch centre)
TIP_OFF = SWAP_BACK - SWAP_FWD                                      # catch centre (the set centre) 24.8 behind the thrust axis (B)


def set_wheel(wz):
    """wheel centres at drone z wz: the seat height follows, the cartridge hangs to keep its seat z"""
    global WHEEL_Z, SEAT_DZ, FUNNEL_TOP_D, CART_BOT
    WHEEL_Z = wz
    SEAT_DZ = WHEEL_SEAT - wz                    # bottom plate top in the seat frame
    FUNNEL_TOP_D = FACE_TOP - SEAT_DZ            # funnel top in the drone frame: the L2 and the arms over the rim stay above it
    CART_BOT = LATCH_SEAT_Z0 - SEAT_DZ           # cartridge flat bottom in the drone frame (the box sets it)


set_wheel(-12.0)
MOTOR_DEG = (45, 135, 225, 315)
CAM_BACK = 30.0                          # (v1 single camera, kept for the old layouts' numbers)
CAM_UP_DEFAULT = 30.0
_CC = os.path.join(ROOT, "runs", "overseer_cams", "v1", "cam_cover.json")
_PAIR = json.load(open(_CC))["pairs"][0] if os.path.exists(_CC) else None
_MIR = [r_ for r_ in json.load(open(_CC))["pairs"] if r_["kind"] == "mirror" and not r_["portrait"]][0] if _PAIR else None
CAM_YAW, CAM_PITCH = (abs(_MIR["x"][0]), _MIR["x"][1]) if _MIR else (46.8, -13.6)   # mirror landscape pair (DEFAULT; ties the ceiling 54.8 %)
CAM_Y, CAM_H2 = 69.0, 0.0                # camera centres beside the L2 (clear of the socket lugs), at its mid height (DEFAULT)
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


def plate_bolts(which):
    """arm-to-plate M3 bolt axes in plan (dimensions O Arms and hub): top 2 per arm at u 63.17, v ±15; bottom at u 63.74, v ±10"""
    u_, v_ = (63.17, 15.0) if which == "top" else (63.74, 10.0)
    out = []
    for d_ in MOTOR_DEG:
        c_, s_ = math.cos(math.radians(d_)), math.sin(math.radians(d_))
        out += [(u_ * c_ - sv * v_ * s_, u_ * s_ + sv * v_ * c_) for sv in (1, -1)]
    return out


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
    def latch_part(k):
        m = LATCH_MESH[k].copy(); v = m.vertices
        m.vertices = np.stack([-dc + (v[:, 2] - LATCH_META["Z_T"]), v[:, 0], v[:, 1] + CART_BOT], 1); return m
    g["pack"], g["cart"] = latch_part("pack"), latch_part("cartridge")
    g["receiver"] = trimesh.util.concatenate([latch_part("receiver"), latch_part("receiver_lid")])   # printed latch frame (structure, like the sled)
    g["xt60"] = latch_part("xt60e1m")                                          # the contact inside it (protected)
    g["swap_cg"] = np.array([-dc - SWAP_FWD, LATCH_META["SWAP_CG"][0], LATCH_META["SWAP_CG"][1] + CART_BOT])
    PB = CART_BOT                                                              # the set's flat bottom (was the pack bottom)
    RT = CART_BOT + LATCH_META["Y_M"]                                         # receiver mount plane (its top)
    rb = g["receiver"].bounds
    # sled: printed PETG plate on the receiver's mount plane (the 6 M3 come up from below into its inserts), 4 thick,
    # on the belly frame tabs of the 4 corner nodes; the pods hang from it beside the receiver
    g["sled"] = box(-dc - 89.65, -dc + 89.65, -76, 76, RT, RT + 4)          # over the receiver body (179.3), its bolts at x -84.7 / 0 / +78.3; 13.8 off the ±x faces (G (8) ≥ 9)
    xb_ = g["xt60"].bounds
    g["sled"] = g["sled"].difference(box(xb_[0][0] - 1, xb_[1][0] + 1, xb_[0][1] - 1, xb_[1][1] + 1, RT - 1, RT + 5))   # XT60 lead window (its top stands 3.5 over the mount plane)
    g["sled"] = g["sled"].difference(union([cyl(4.5, RT - 1, RT + 5, *xy_) for xy_ in plate_bolts("bottom")]))   # driver holes under the bottom plate bolts (as the pan)
    g["cam_pod"] = box(-dc - 23, -dc + 23, 33.6, 72.5, PB + 16, RT)
    g["flow_pod"] = box(-dc - 15, -dc + 15, -62, -35, PB + 16, RT)
    g["down_cam"] = trimesh.util.concatenate([box(-dc - 19, -dc + 19, 34, 72, PB + 14.4, PB + 16), cyl(6, PB + 6, PB + 14.4, -dc, 53)])
    g["down_lens"] = np.array([-dc, 53.0, PB + 6]); g["flow"] = box(-dc - 10.8, -dc + 10.8, -58, -36, PB + 9.5, PB + 16)
    g["flow_lens"] = np.array([-dc, -47.0, PB + 9.5])
    # skid runners: printed PETG sleeves on the skid rails (cage), the lowest points with the feet: a side edge reaching
    # under the feet meets them before the lenses; 2.4 under the down camera board, inside the opening at ±5 deg yaw
    # hub belly pan: printed PETG skid plate under the bottom plate (cage): the electronics' shield from below; the hub
    # blocks sit on it; its underside stays over the box top deck seated (the rear overhangs the funnel mouth)
    g["pan"] = box(-95.0, 100.0, -45.0, 45.0, -4.7, -2.2)
    g["pan"] = g["pan"].difference(union([cyl(4.5, -6.0, -1.0, *xy_) for xy_ in plate_bolts("bottom")]))   # driver holes under the bottom plate bolts
    g["runners"] = [union([box(-dc - 83.5, -dc + 83.5, 62 if sy > 0 else -74, 74 if sy > 0 else -62, PB - 17, PB + 12),     # sleeve on the skid rail, to ±83.5: past the set's ends (±86 cartridge) inside the opening at ±5 deg yaw
                           box(-dc - 83.5, -dc + 83.5, 72.5 if sy > 0 else -74, 74 if sy > 0 else -72.5, PB + 12, PB + 26)])     # outer lip: shields the camera board / flow window edge (to 10 over the board: a side edge 15..60 deg up meets it first)
                    for sy in (1, -1)]
    g["feet"] = [cyl(5, PB - 17, PB - 6, -dc + sx * 77, sy * 66) for sx in (1, -1) for sy in (1, -1)]   # TPU pads under the skid ends, inside the opening at ±5 deg yaw
    # keep-alive pads (G (6)): one per ±x lower-ring face, 2.0 off the face at the probe height, over the probe pair
    # (+x: y +12 / +22 = B+, -x: y -12 / -22 = B-), on the wheel ring rod; 3 thick
    pads = []
    for sx in (1, -1):
        zc = PROBE_Z - SEAT_DZ; xf = FACE_C + PROBE_Z                          # face point at the probe height (seat x)
        nrm = np.array([-sx, 0, 1]) / math.sqrt(2); c0 = np.array([-dc + sx * xf, 17.0 * sx, zc]) + nrm * (PAD_OFF + 1.5)
        pd = box(-1.5, 1.5, -9, 9, -6, 6); R = np.eye(4); R[:3, 0] = nrm; R[:3, 1] = [0, 1, 0]; R[:3, 2] = np.cross(nrm, [0, 1, 0]); R[:3, 3] = c0
        pd.apply_transform(R); pads.append(pd)
    g["pads"] = pads
    g["PB"] = PB; g["RT"] = RT

    # guard: two octagon rings round the motor square, 12 off the swept disks; the front flat is a V to the L2 apex
    B = np.array([xB, 0.0, zB]); g["B"] = B
    g["l2"] = l2_mesh(B)
    up_b = np.array([-A[2], 0, A[0]])                            # in the base plane, toward +z: (0.259, 0, 0.966)
    seat = cyl(37.5, -5, 0, 0, 0); T = trimesh.transformations.rotation_matrix(math.radians(L2_TILT), [0, 1, 0]); T[:3, 3] = B
    seat.apply_transform(T)
    back = box(-22, 22, -16, 16, -11, -5); back.apply_transform(T)               # apex block behind the seat (L2 local z -11..-5)
    # apex rod sockets on side lugs beside the L2 body (y ±46), just behind the base plane, so the V never bends back
    # inside the front flat (the guard keeps 12 off the front disks)
    # two front cameras (Spark 24 Oct: both forward, angled to see as much of the lidar's view as possible): the mirror
    # pair from cam_cover.py (yaw +-CAM_YAW, pitch CAM_PITCH, landscape) beside the L2 at its mid height, each set back
    # until it is out of the L2 field (negative-angle mode + 1.5), on a printed arm from its socket lug
    cams, cam_meshes, plates, arms_ = [], [], [], []
    for sy in (1, -1):
        Rc = np.eye(4); Rc[:3, :3] = cam_cover.R_cam(sy * CAM_YAW, CAM_PITCH, 0.0)
        board = box(-1, 1, -19, 19, -19, 19); lens = trimesh.creation.cylinder(6, 8, sections=32)
        lens.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); lens.apply_translation([5, 0, 0])
        cam0 = trimesh.util.concatenate([board, lens]); plate0 = box(-4.5, -1.5, -20, 20, -20, 20).difference(box(-5, -1, -6, 6, 17, 21))   # USB-C plug notch at the board top edge
        back_ = 0.0
        for _ in range(3):                                         # set back along -A until clear of the field
            C = B + up_b * CAM_H2 + np.array([0, sy * CAM_Y, 0]) - A * back_
            Rc[:3, 3] = C; q = np.vstack([cam0.copy().apply_transform(Rc).vertices, plate0.copy().apply_transform(Rc).vertices])
            d = q - B; mg_ = -(d @ A) - math.sin(math.radians(L2_NEG)) * np.linalg.norm(d, axis=1)
            back_ += max(0.0, 1.5 - float(mg_.min()))
        cm = cam0.copy().apply_transform(Rc); pl = plate0.copy().apply_transform(Rc)
        fwd = Rc[:3, 0].copy(); cup = Rc[:3, 2].copy()
        cams.append(dict(c=C, fwd=fwd, up=cup, left=Rc[:3, 1].copy(), lens=C + fwd * 9, back=back_, sy=sy))
        cam_meshes.append(cm); plates.append(pl)
        lugp = B - A * (co.SOCK_BACK + 3) + np.array([0, sy * 46.0, 0])            # printed arm from the socket lug to the plate back
        ax_ = C - lugp; ax_ /= np.linalg.norm(ax_); sphi = float(np.linalg.norm(np.cross(ax_, fwd)))
        arms_.append(rod(lugp, C - fwd * (2.0 + 6.0 * sphi), 6.0))  # printed with the apex block: its end disc stops inside the 3 plate
        g.setdefault("cam_arm_len", []).append(float(np.linalg.norm(C - fwd * (2.0 + 6.0 * sphi) - lugp)))
    g["cams"] = cams
    g["cam"] = trimesh.util.concatenate(cam_meshes)
    g["cam_lens"] = cams[0]["lens"]; g["cam_fwd"] = cams[0]["fwd"]; g["cam_up_v"] = cams[0]["up"]
    g["cam_rods"] = []
    g["cam_plate"] = trimesh.util.concatenate(plates); g["cam_arms"] = arms_; g["cam_plates"] = plates
    g["cam_joint_mm3"] = None                                     # each arm bonded into its plate: exact volume set on the final layout
    g["cam_bracket"] = g["cam_plate"]
    sock = co.sockets(xB, zB)
    lugs = []
    for sy in (1, -1):                                             # lugs spanning the socket spread (local x = -up_b)
        h = co.APEX_SPAN / 2 + 6
        lug = box(-h, h, 36 if sy > 0 else -50, 50 if sy > 0 else -36, -co.SOCK_BACK - 6, -2); lug.apply_transform(T); lugs.append(lug)
    cb = g["cam"].bounds
    P = dict(dm=dm, xc=-dc, wz=WHEEL_Z, xB=xB, zB=zB, cam_top=float(max(cb[1][2], g["cam_bracket"].bounds[1][2])), cam_front=float(cb[1][0]),
             cam_bot=float(min(cb[0][2], g["cam_bracket"].bounds[0][2])), nose_y=float(max(co.NOSE_Y, np.abs(cb[:, 1]).max() + co.NOSE_GAP)),
             cam_hu=float(np.abs((np.vstack([g["cam"].vertices, g["cam_bracket"].vertices]) - B) @ co.UP_B).max() + co.NOSE_GAP),
             obstacles=[dict(c=[float(v) for v in cm_["c"] + cm_["fwd"] * xc_], fwd=[float(v) for v in cm_["fwd"]], left=[float(v) for v in cm_["left"]],
                             half=hf_) for cm_ in g["cams"] for xc_, hf_ in ((-1.75, [2.75, 20.0, 20.0]), (5.0, [4.0, 6.0, 6.0]))]
                       + [dict(c=[2.5, 0.0, 22.25], fwd=[1.0, 0.0, 0.0], left=[0.0, 1.0, 0.0], half=[98.0, 60.2, 24.4], skip_fixed=True, skip_nodes=["arm"]),   # cameras (plate + board, lens) and the plate sandwich (C)
                          dict(c=[float(v) for v in B + A * 19.5], fwd=[float(v) for v in A], left=[0.0, 1.0, 0.0], half=[19.5, 37.5, 37.5], skip_nodes=["Slo", "Shi", "SP", "apexM"]),   # the L2 body (75 x 75 x 39, M)
                          dict(pts=np.round(g["l2"].vertices[(g["l2"].vertices - B) @ A > 39.1], 2).tolist(), skip_nodes=["Slo", "Shi", "SP", "apexM"]),   # and its dome (vertices)
                          dict(field=[[float(v) for v in B + A * L2_BASE_H], [float(v) for v in A], L2_NEG])],   # the L2 scan: no rod in it (Spark 24 Oct)
             m_drone_g=M_DRONE, rear_x=-102.0, pyramid="mast", x_top=max(dm, MAST_X_MIN), x_mast=max(dm, MAST_X_MIN), z_mast=float(WHEEL_Z),
             feet=(77.0, 66.0, float(g["PB"]) - 3.5), belly=(77.0, 66.0), swap_g=float(SWAP_G + RECEIVER_G), cap_groups={"skid_belly": "t10", "wheel_ring": "t6", "struts": "t10", "skirt_ties": "t8", "skirt": "t10", "nose": "t10", "nose_mount": "t10"}, seat_dz=float(SEAT_DZ),
             z_rhub=-7.0, z_skirt=-4.0, z_fhub=12.0, x_rhub=-68.0)   # rear hub block reaches x -68: its skirt ties 39 long (carbon from 33, CLAUDE.md)                      # front hub block between the plates: the keels pass over the sled and the cartridge                                   # rear hub nodes and skirt 3.5 under the bottom plate (6 tube ties clear it)   # funnel caps: skid rails clear the down camera board and the legs the opening; wheel ring and struts keep the hook latch plungers 2.0 (G (7))   # crash mass ~3 kg (E, no cap); skid rails under the down camera board (10 tube top 1.9 under it)
    g["P"] = P
    Ng, Mg, fixed_g, _, info = co.geometry(P, CAGE_TOPO, CAGE_SIZES)
    sock = hsock = 0.0                                            # rod ends epoxied in printed sockets on the arms / hubs / apex, 20 deep (E)
    for name, a, b, group, kind in Mg:
        if kind == "rod":
            sc = sec_of(name, group); rods[name] = (Ng[a], Ng[b], co.SECTIONS[sc]["od"], group, sc); g.setdefault("rod_ends", {})[name] = (a, b)
            sock += sum(math.pi * (co.SECTIONS[sc]["od"] / 2) ** 2 * 20.0 for n_ in (a, b) if n_.startswith(("tipb", "tipt", "armbot", "arm", "root")))
            hsock += sum(math.pi * (co.SECTIONS[sc]["od"] / 2) ** 2 * 20.0 for n_ in (a, b) if n_.startswith(("fhubB", "fhubT", "chin", "rhubB", "Shi", "Slo", "hubc")))
    g["arm_socket_mm3"] = sock; g["hub_socket_mm3"] = hsock
    g["pod_clamp_mm3"] = (sum(math.pi * (v[2] / 2) ** 2 * 46.0 for k, v in rods.items() if k.startswith("belly bar"))   # pods clamp on the belly bars (E)
                          + sum(overlap(rod(v[0], v[1], v[2] / 2), g["sled"]) for k, v in rods.items() if k.startswith("skid leg")) * 1.05 + 1.0)   # skid legs through the sled bosses
    od_n = {}                                                     # printed nodes drawn at their size: a block round the largest socket
    for name, a, b, group, kind in Mg:
        if kind == "rod":
            for n_ in (a, b): od_n[n_] = max(od_n.get(n_, 0.0), co.SECTIONS[sec_of(name, group)]["od"])
    def nbox(n_):
        h_ = co.node_size(od_n.get(n_, 3.0)) / 2; q = Ng[n_]
        return box(q[0] - h_, q[0] + h_, q[1] - h_, q[1] + h_, q[2] - h_, q[2] + h_)
    ring_nodes = [nbox(f"{nm}{lvl}") for nm in info["pnodes"] for lvl in ("lo", "hi")]
    ring_nodes += [nbox(n_) for n_ in Ng if n_ == "TR" or n_.startswith(("KV", "KP"))]   # rear tower and skirt nodes (cage up)
    mx_ = [n_ for n_ in Ng if n_.startswith("MX")]
    if mx_:                                                       # mast node block: the spine crossings and the short mast links in one print
        h_ = max(co.node_size(od_n.get(n_, 3.0)) for n_ in mx_) / 2; q_ = np.array([Ng[n_] for n_ in mx_])
        ring_nodes.append(box(q_[:, 0].min() - h_, q_[:, 0].max() + h_, -h_, h_, q_[:, 2].min() - h_, q_[:, 2].max() + h_))
    Tg_ = trimesh.transformations.rotation_matrix(math.radians(L2_TILT), [0, 1, 0]); Tg_[:3, 3] = B   # the L2 frame (base at z 0)
    go_ = GUARD_IN + GUARD_W
    sh_ = box(-go_, go_, -go_, go_, -1.8, GUARD_TOP).difference(box(-GUARD_IN, GUARD_IN, -GUARD_IN, GUARD_IN, -3.0, GUARD_TOP + 1.0))   # 1.8 under the base
                                                                                  # plane (to the lug tops): the base's square corners overhang the round seat
    sh_ = sh_.difference(box(-5.0, 5.0, GUARD_IN - 1.0, go_ + 1.0, 1.5, 12.5))                    # L2 cable window on the left wall (local -X turned to +y), closed
                                                                                                  # round: a sharp edge can't reach in (an open slot let one)
    guard_ = [sh_.apply_transform(Tg_)]
    for n_ in ("NT+1", "NT-1", "NB+1", "NB-1"):                                                   # posts from the shroud corners to the four collar corner nodes
        q_ = Ng[n_] - B; cu_, cy_ = float(q_ @ co.UP_B), float(q_[1])
        c_ = B + co.UP_B * math.copysign(go_ - GUARD_W / 2, cu_) + np.array([0.0, math.copysign(go_ - GUARD_W / 2, cy_), 0.0]) + A * 3.0
        guard_.append(rod(c_, Ng[n_], 3.0))
    g["base_guard"] = trimesh.util.concatenate(guard_)
    ring_nodes += guard_                                                                                         # (cage up: printed with the nodes)
    ring_nodes += [nbox(n_) for n_ in ("NR+1", "NR-1", "NT+1", "NT-1", "NB+1", "NB-1", "TN")]   # nose nodes (the last 7)
    g["ring_v"] = [(r[1], r[2]) for r in info["ring"] if r[3]]
    hubs = [box(Ng[n][0] - 7, Ng[n][0] + 7, Ng[n][1] - 7, Ng[n][1] + 7, Ng[n][2] - 4, Ng[n][2] + 4)
            for n in Ng if n.startswith(("fhubB", "fhubT", "chin", "rhubB"))]
    g["hubs"] = hubs
    g["hub_socket_mm3"] = g.get("hub_socket_mm3", 0.0) + sum(overlap(h_, g["pan"]) for h_ in hubs)   # hub blocks seated on the belly pan
    g["hub_plate_mm3"] = sum(overlap(h_, pl_) for h_ in hubs for pl_ in (TOP, BOTTOM)) + 1.0            # and clamped on the plates
    g["hub_arm_mm3"] = sum(overlap(h_, a_) for h_ in hubs for a_ in g["arms"]) * 1.5 + 1.0              # front hub blocks seat on the front arm roots
    g["cage_N"], g["cage_info"] = Ng, info
    g["rods"] = rods
    g["rod_meshes"] = {k: rod(v[0], v[1], v[2] / 2) for k, v in rods.items()}
    g["ring_nodes"] = ring_nodes
    g["apex"] = union([seat, back] + lugs + hubs + g["cam_arms"])
    low_keys = [k for k, v in rods.items() if v[3] in ("struts", "strut2", "wheel_ring", "skid_belly")]
    nose_keys = [k for k, v in rods.items() if v[3] in NOSE_GROUPS or k in ("spider nose+", "spider nose-")]
    g["cage_low"] = trimesh.util.concatenate(nodes + [g["rod_meshes"][k] for k in low_keys])
    g["cage_up"] = trimesh.util.concatenate([n_ for n_ in ring_nodes[:-7]] + [g["rod_meshes"][k] for k in rods if k not in low_keys and k not in nose_keys])
    g["cage_nose"] = trimesh.util.concatenate(ring_nodes[-7:] + [g["rod_meshes"][k] for k in nose_keys])
    g["nose_keys"] = nose_keys
    return g


def l2_clear_margin(g, B, solid_items):
    """smallest margin (mm) of every drone point to the L2 scan: outside the cone of 90 + L2_NEG deg about the axis from
    the top of the base (Spark 24 Oct: the 25 base is the mount, the scan starts above it); the seat is the mount and skipped."""
    worst = (None, np.inf); O_ = B + A * L2_BASE_H; ng_ = math.radians(L2_NEG)
    for name, m in solid_items:
        p = pts(m, 2500)
        if name == "props":
            p = np.vstack([p + [0, 0, BLADE], p - [0, 0, BLADE]])
        d = p - O_; depth = d @ A
        if name == "apex":
            radial = np.linalg.norm(d - np.outer(depth, A), axis=1)
            keep = ~((depth > -L2_BASE_H - 5.5) & (radial < 40.0)); d, depth = d[keep], depth[keep]   # the seat itself
        radial = np.sqrt(np.maximum((d ** 2).sum(1) - depth ** 2, 0))
        margin = -depth * math.cos(ng_) - radial * math.sin(ng_)
        i = int(np.argmin(margin))
        if margin[i] < worst[1]: worst = (name, margin[i])
    return worst


def cg_of(g):
    items = [("L2", 230, g["l2"]), ("front cameras", 30, g["cam"]), ("down camera", 15, g["down_cam"]), ("Pi 5 + cooler", 55, PI),
             ("FC", 10, FC), ("PDB", 21, PDB), ("UBEC", 15, UBEC), ("buck", 6, BUCK), ("RX", 2, RX),
             ("pack + cartridge (swap set)", SWAP_G, trimesh.points.PointCloud([g["swap_cg"]])), ("receiver set", RECEIVER_G, g["receiver"]),
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
               ("deck", [DECK], 0.5), ("sled + pods", [g["sled"], g["cam_pod"], g["flow_pod"]], 0.3), ("feet", g["feet"], 0.5), ("skid runners", g["runners"], 0.4), ("belly pan", [g["pan"]], 0.5),
               ("keep-alive pads", g["pads"], 0.6)]
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
    lo, hi = 150.0, 620.0
    for _ in range(12):
        mid = (lo + hi) / 2; g = g_fn(mid)
        items = [("props", trimesh.util.concatenate(g["props"])), ("motors", trimesh.util.concatenate(g["motors"])),
                 ("arms", trimesh.util.concatenate(g["arms"])), ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g["apex"]),
                 ("cage up", g["cage_up"]), ("cage low", g["cage_low"]), ("cage nose", g["cage_nose"]), ("wheels", trimesh.util.concatenate(g["wheels"])), ("front cameras", g["cam"]), ("cam bracket", g["cam_bracket"])]
        _, mg = l2_clear_margin(g, g["B"], items)
        if mg >= 1.0 and guard_v_gap(g) >= GUARD_GAP - 0.05: hi = mid
        else: lo = mid
    return hi


def funnel(g, dc):
    """seated funnel checks (G): every non-wheel point 1.0 off the faces (planes half = 82.7 + seat z: the ±x faces from
    z 12.8, the ±y faces from z 1.3, to the top 75.1); the hook latch plunger zone (G (7)); the keep-alive band (G (8):
    nothing within 9.0 of the ±x faces over y -35..+69 (+x) / -69..+35 (-x) between the face edge and the probes, the
    pads excepted); below the face edges everything inside 191 (x) x 168 (y) at ±5 deg yaw, 3 a side (G (2))"""
    others = trimesh.util.concatenate(g["arms"] + g["motors"] + [g["cage_low"], g["cage_up"], TOP, BOTTOM, FC, PI, DECK, PDB, UBEC, BUCK, RX, g["apex"], g["l2"], g["cam"],
                                       g["cam_bracket"], g["sled"], g["cam_pod"], g["flow_pod"]] + g["feet"] + g["runners"] + [g["pan"]])
    latch = trimesh.util.concatenate([g["cart"], g["receiver"]])   # the battery latch's set: its own session's check (3.42 a side to the panels)
    po_, pl_ = pts(others, 40000), pts(latch, 8000)
    p = np.vstack([po_, pl_]) + [dc, 0, SEAT_DZ]                   # funnel centre at the catch centre
    own = np.arange(len(p)) < len(po_)
    d_face = np.full(len(p), np.inf); z = p[:, 2]
    for ax, z0 in ((0, X_EDGE_Z), (1, Y_EDGE_Z)):
        for sg in (1, -1):
            x = sg * p[:, ax]; in_z = (z > z0) & (z < FACE_TOP) & (np.abs(p[:, 1 - ax]) < FACE_C + z)
            d_face = np.where(in_z, np.minimum(d_face, (z + FACE_C - x) / math.sqrt(2)), d_face)
    i = int(np.argmin(d_face))
    global PLUNGER_CLEAR, G8_CLEAR
    PLUNGER_CLEAR = np.inf
    for sy in (1, -1):
        y = sy * p[:, 1]
        s_up = ((y - OPEN_HALF) + (z - Y_EDGE_Z)) / math.sqrt(2); off = (z + FACE_C - y) / math.sqrt(2)
        for sx in (1, -1):
            dx = np.maximum(np.abs(p[:, 0] - sx * 100.0) - 6.0, 0)
            ds = np.maximum(np.maximum(50.1 - s_up, s_up - 58.1), 0)
            do = np.maximum(np.maximum(-off, off - 12.0), 0)
            PLUNGER_CLEAR = min(PLUNGER_CLEAR, float(np.sqrt(dx ** 2 + ds ** 2 + do ** 2).min()))
    global LATCH_G8, LATCH_LOW
    def g8(mask):
        c_ = np.inf
        for sx, (y0, y1) in ((1, (-35.0, 69.0)), (-1, (-69.0, 35.0))):
            sel = mask & (z > X_EDGE_Z) & (z < PROBE_Z) & (p[:, 1] > y0) & (p[:, 1] < y1) & (sx * p[:, 0] > 0)
            if sel.any(): c_ = min(c_, float(((z[sel] + FACE_C - sx * p[sel, 0]) / math.sqrt(2)).min()))
        return c_
    def below(mask):
        w_ = -np.inf
        for yaw in np.radians(np.linspace(-5, 5, 11)):
            c, s_ = math.cos(yaw), math.sin(yaw); q = p[:, :2] @ np.array([[c, -s_], [s_, c]]).T
            lx = q[mask & (z < X_EDGE_Z)]; ly = q[mask & (z < Y_EDGE_Z)]
            if len(lx): w_ = max(w_, np.abs(lx[:, 0]).max() - (OPEN_X - 3.0))
            if len(ly): w_ = max(w_, np.abs(ly[:, 1]).max() - (OPEN_HALF - 3.0))
        return w_
    G8_CLEAR, LATCH_G8 = g8(own), g8(~own)
    worst, LATCH_LOW = below(own), below(~own)
    return d_face[i], p[i], worst


# ---------------------------------------------------------------- balance: dm (motors forward), dc (catch aft), wheel height
def leg_reach(g):
    return max(float(np.linalg.norm(np.subtract(v[1], v[0]))) for k, v in g["rods"].items() if v[3] in ("struts", "strut2"))


def balance(dc_unused, dm0):
    global M_DRONE
    zB = math.ceil(l2_low_zB()); dm = dm0
    for _ in range(10):
        dc = TIP_OFF - dm
        xB = solve_xB(lambda x: build(dm, dc, x, zB), zB)
        tot, cg = cg_of(build(dm, dc, xB, zB))
        if abs(cg[0] - dm) < 0.3: dm = cg[0]; break
        dm = cg[0]                                                # thrust centre (motor square centre) onto the CG
    dc = TIP_OFF - dm
    xB = solve_xB(lambda x: build(dm, dc, x, zB), zB); g = build(dm, dc, xB, zB); tot, cg = cg_of(g)
    M_DRONE = float(tot)
    return dm, xB, zB, g, tot, cg


SCR_DBG = os.environ.get("SYS_DEBUG_DIR", "/tmp")
if os.environ.get("SYS_DEBUG_XB"):                                 # why the L2 base sits where it does
    set_wheel(-48.0); zB_ = math.ceil(l2_low_zB())
    for x_ in [float(v) for v in os.environ["SYS_DEBUG_XB"].split(",")]:
        dmd_ = float(os.environ.get("SYS_DEBUG_DM", "197.6")); g_ = build(dmd_, TIP_OFF - dmd_, x_, zB_)
        items = [("props", trimesh.util.concatenate(g_["props"])), ("motors", trimesh.util.concatenate(g_["motors"])),
                 ("arms", trimesh.util.concatenate(g_["arms"])), ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g_["apex"]),
                 ("cage up", g_["cage_up"]), ("cage low", g_["cage_low"]), ("cage nose", g_["cage_nose"]), ("wheels", trimesh.util.concatenate(g_["wheels"])),
                 ("front cameras", g_["cam"]), ("cam bracket", g_["cam_bracket"])]
        print(x_, zB_, l2_clear_margin(g_, g_["B"], items), [(nm_, round(l2_clear_margin(g_, g_["B"], [(nm_, m_)])[1], 1)) for nm_, m_ in items], guard_v_gap(g_))
    json.dump(g_["P"], open(os.path.join(SCR_DBG, "P_debug.json"), "w"), indent=1, default=float)
    for nm__, m__ in (("cage low", g_["cage_low"]), ("cage up", g_["cage_up"]), ("feet", trimesh.util.concatenate(g_["feet"])), ("runners", trimesh.util.concatenate(g_["runners"])),
                      ("receiver", g_["receiver"]), ("cart", g_["cart"]), ("sled", g_["sled"]), ("pods", trimesh.util.concatenate([g_["cam_pod"], g_["flow_pod"]])),
                      ("arms", trimesh.util.concatenate(g_["arms"])), ("wheels", trimesh.util.concatenate(g_["wheels"])), ("nodes", trimesh.util.concatenate(g_["nodes"]))):
        q__ = pts(m__, 20000) + [TIP_OFF - dmd_, 0, SEAT_DZ]; z__ = q__[:, 2]; worst__ = -np.inf; g8__ = np.inf
        for yaw in np.radians(np.linspace(-5, 5, 11)):
            c_, s_ = math.cos(yaw), math.sin(yaw); qq = q__[:, :2] @ np.array([[c_, -s_], [s_, c_]]).T
            if (z__ < X_EDGE_Z).any(): worst__ = max(worst__, np.abs(qq[z__ < X_EDGE_Z, 0]).max() - (OPEN_X - 3))
            if (z__ < Y_EDGE_Z).any(): worst__ = max(worst__, np.abs(qq[z__ < Y_EDGE_Z, 1]).max() - (OPEN_HALF - 3))
        for sx, (y0, y1) in ((1, (-35.0, 69.0)), (-1, (-69.0, 35.0))):
            sel = (z__ > X_EDGE_Z) & (z__ < PROBE_Z) & (q__[:, 1] > y0) & (q__[:, 1] < y1) & (sx * q__[:, 0] > 0)
            if sel.any(): g8__ = min(g8__, float(((z__[sel] + FACE_C - sx * q__[sel, 0]) / math.sqrt(2)).min()))
        fd__ = np.inf
        for ax, z0 in ((0, X_EDGE_Z), (1, Y_EDGE_Z)):
            for sg in (1, -1):
                x__ = sg * q__[:, ax]; in_ = (z__ > z0) & (z__ < FACE_TOP) & (np.abs(q__[:, 1 - ax]) < FACE_C + z__)
                if in_.any():
                    v__ = (z__ + FACE_C - x__) / math.sqrt(2); v__[~in_] = np.inf; k__ = int(np.argmin(v__))
                    if v__[k__] < fd__: fd__ = float(v__[k__]); at__ = q__[k__] - [TIP_OFF - dmd_, 0, SEAT_DZ]
        print(f"  {nm__}: below the edges {worst__:+.1f}, G8 {g8__:.1f}, faces {fd__:.1f}" + (f" at drone {np.round(at__, 1).tolist()}" if np.isfinite(fd__) else ""))
    print("camera x arms", [round(overlap(g_["cam"], a__), 1) for a__ in g_["cam_arms"]], "camera x apex", round(overlap(g_["cam"], g_["apex"]), 1),
          "camera x lugs+seat", round(overlap(g_["cam"], union([c__ for c__ in [g_["apex"]]])), 1))
    sys.exit(0)
say(f"cage (cage_opt level {os.environ.get('V3_LEVEL', '160')} Hz): " + " ".join(k for k, v in CAGE_TOPO.items() if v)
    + " | " + " ".join(f"{k}:{v}" for k, v in CAGE_SIZES.items()))
dm = 80.0
CAM_UP = CAM_UP_DEFAULT
SCR = OUT; os.makedirs(SCR, exist_ok=True)
FINAL_PASS = False
set_wheel(float(os.environ.get("V3_WZ0", "-30")))                     # (a frozen recheck with SYS_RESIZE=0 keeps this wheel height)
for it_sz in range(int(os.environ.get("SYS_RESIZE", "7"))):     # layout <-> cage sizing until the sizes stop changing
    for wz in np.arange(float(os.environ.get("V3_WZ0", "-30")), -60.0, -2.0):
        set_wheel(wz)
        dm, xB, zB, g, tot, cg = balance(None, dm)
        dc = TIP_OFF - dm
        fd, fp, low = funnel(g, dc)
        say(f"wheels z {wz:5.1f}: motors {dm:5.1f} ahead, catch centre x {-dc:5.1f}, L2 base ({xB:5.1f}, {zB:4.0f}), funnel {fd:5.1f}, "
            f"plungers {PLUNGER_CLEAR:4.1f}, G (8) {G8_CLEAR:4.1f}, below the edges {low:+5.1f} (≤ 0), {tot:4.0f} g")
        if fd >= 1.0 and low <= 0.0 and PLUNGER_CLEAR >= 1.0 and G8_CLEAR >= G8_OFF: break
    else:
        say("NO FUNNEL FIT in the wheel z range: the last height is kept and the funnel line below FAILS")
    dc = TIP_OFF - dm
    g = build(dm, dc, xB, zB)
    tot, cg = cg_of(g)
    if FINAL_PASS: break                                              # the layout drawn with the settled sizes
    new_sizes, r_sz, ok_sz = co.size_fsd(g["P"], CAGE_TOPO)   # re-size the cage on this geometry (F), cold start (a warm start ratchets the nose up)
    if not ok_sz and os.environ.get("SYS_SEARCH", "1") == "1":         # the layout no longer passes here: search every topology again
        from multiprocessing import Pool
        with Pool(4) as pool:
            table_, best_, abl_ = co.search(g["P"], co.F_L2_MIN, pool)
        nf = sum(t["feasible"] for t in table_)
        json.dump(dict(params=g["P"], table=[{k: v for k, v in t.items() if k != "sizes"} for t in table_], ablation=abl_,
                       topo=best_[0] if best_ else None, sizes=best_[1] if best_ else None),
                  open(os.path.join(SCR, f"cage_search_pass{it_sz + 1}.json"), "w"), indent=1, default=float)
        say(f"cage search pass {it_sz + 1}: {nf} of {len(table_)} topologies feasible on this geometry"
            + (f"; best {best_[2]['mass_g']:.0f} g, L2 mode {best_[2]['f_l2']:.0f} Hz: " + " ".join(k for k, v in best_[0].items() if v and k not in co.FIXED_ON) if best_ else ""))
        if best_:
            CAGE_TOPO, new_sizes, r_sz, ok_sz = best_[0], best_[1], best_[2], True
    say(f"cage sizing pass {it_sz + 1}: {r_sz['mass_g']:.0f} g, L2 mode {r_sz['f_l2']:.0f} Hz, feasible {ok_sz}, "
        f"{sum(new_sizes.get(k) != v for k, v in CAGE_SIZES.items())} rods changed")
    nchg = sum(new_sizes.get(k) != v for k, v in CAGE_SIZES.items())
    if new_sizes == CAGE_SIZES and ok_sz: break
    CAGE_SIZES = new_sizes
    if ok_sz and nchg <= 3: FINAL_PASS = True                         # settled: one more layout pass with these sizes
# final: the motor square position with the cage sized on its own geometry. Moving the square moves the cage, the L2 and the
# set with it; only the plates and their electronics (~0.3 kg) stay, so a 1.5 mm shift of the moving parts' CG moves the
# balance point ~16 mm and a plain re-balance / re-size loop swings. Bracket instead: e(dm) = CG x - dm with the cage sized
# cold at dm, bisect until |e| <= BAL_TOL and the frame passes (the balance gate is 3, DEFAULT L).
BAL_TOL = 1.5
def at_dm(dm_):
    global CAGE_SIZES, M_DRONE
    zB_ = math.ceil(l2_low_zB()); dc_ = TIP_OFF - dm_
    xB_ = solve_xB(lambda x: build(dm_, dc_, x, zB_), zB_)
    g_ = build(dm_, dc_, xB_, zB_); s_ = None
    for _ in range(4):                                                # the crash load follows the drone mass: size, weigh, size again until it holds
        s_, r_, ok_ = co.size_fsd(g_["P"], CAGE_TOPO, init=s_)
        CAGE_SIZES = s_
        m0_ = M_DRONE; g_ = build(dm_, dc_, xB_, zB_); tot_, cg_ = cg_of(g_); M_DRONE = float(tot_)
        if abs(M_DRONE - m0_) < 5.0: break
        g_ = build(dm_, dc_, xB_, zB_)
    say(f"final: motors {dm_:.1f} ahead, cage {r_['mass_g']:.0f} g sized here, CG {cg_[0] - dm_:+.1f} in x, frame violation {r_['violation']:.3f}")
    return dict(e=float(cg_[0] - dm_), ok=ok_, dm=dm_, sizes=s_, xB=xB_, zB=zB_)
if os.environ.get("SYS_FINAL_SIZES"):                                 # a settled design to recheck: its topology and sizes at SYS_FINAL_DM, no re-sizing
    fz_ = json.load(open(os.environ["SYS_FINAL_SIZES"])); CAGE_TOPO, CAGE_SIZES = fz_["topo"], fz_["sizes"]
    dm = float(os.environ.get("SYS_FINAL_DM", dm)); zB_ = math.ceil(l2_low_zB()); dc_ = TIP_OFF - dm
    for _ in range(2):                                                # the crash load follows the drone mass: build, weigh, build again
        xB_ = solve_xB(lambda x: build(dm, dc_, x, zB_), zB_); g_ = build(dm, dc_, xB_, zB_); tot_, cg_ = cg_of(g_); M_DRONE = float(tot_)
    g_ = build(dm, dc_, xB_, zB_)
    fr_ = co.evaluate(g_["P"], CAGE_TOPO, CAGE_SIZES)
    say(f"final: motors {dm:.1f} ahead, cage {fr_['mass_g']:.0f} g from {os.environ['SYS_FINAL_SIZES']}, CG {cg_[0] - dm:+.1f} in x, frame violation {fr_['violation']:.3f}"
        + "".join(f"; {k} {v:+.3f}" for k, v in fr_["g"].items() if v > 0))
    json.dump(g_["P"], open(os.path.join(OUT, "params_frozen.json"), "w"), indent=1, default=float)
    tries = [dict(e=float(cg_[0] - dm), ok=True, dm=dm, sizes=CAGE_SIZES, xB=xB_, zB=zB_)]   # rechecked as it is: the lines below report it
else:
    tries = [at_dm(float(os.environ.get("SYS_FINAL_DM", dm)))]          # SYS_FINAL_DM: start at a balanced point found before
dm = tries[0]["dm"]
if not (tries[0]["ok"] and abs(tries[0]["e"]) <= BAL_TOL):
    dm_b = balance(None, dm)[0]                                       # exact balance with these sizes: the far side of the bracket
    tries.append(at_dm(dm_b))
    a_, b_ = sorted(tries[:2], key=lambda t: t["dm"])
    for _ in range(7):
        if any(t["ok"] and abs(t["e"]) <= BAL_TOL for t in tries): break
        if (a_["e"] > 0) == (b_["e"] > 0):                            # no sign change: step past the end with the smaller |e|
            t_ = min((a_, b_), key=lambda t: abs(t["e"])); dm_n = t_["dm"] + (8.0 if t_["e"] > 0 else -8.0)
        else:
            dm_n = (a_["dm"] + b_["dm"]) / 2
        tn = at_dm(dm_n); tries.append(tn)
        if (a_["e"] > 0) != (b_["e"] > 0):
            if (tn["e"] > 0) == (a_["e"] > 0): a_ = tn
            else: b_ = tn
        else:
            a_, b_ = sorted([min((a_, b_), key=lambda t: abs(t["e"])), tn], key=lambda t: t["dm"])
best = min(tries, key=lambda t: (not t["ok"], abs(t["e"]) > BAL_TOL, abs(t["e"])))
dm, xB, zB, CAGE_SIZES = best["dm"], best["xB"], best["zB"], best["sizes"]; dc = TIP_OFF - dm
g = build(dm, dc, xB, zB); tot, cg = cg_of(g); M_DRONE = float(tot)
g = build(dm, dc, xB, zB)                                             # with the drone mass of this layout in its crash cases
if not best["ok"] and os.environ.get("SYS_SEARCH", "1") == "1":       # no passing sizing at any point tried: every topology here
    from multiprocessing import Pool
    with Pool(4) as pool:
        table_, best_, abl_ = co.search(g["P"], co.F_L2_MIN, pool)
    say(f"   final search: {sum(t['feasible'] for t in table_)} of {len(table_)} topologies feasible"
        + (f"; best {best_[2]['mass_g']:.0f} g: " + " ".join(k for k, v in best_[0].items() if v and k not in co.FIXED_ON) if best_ else ""))
    if best_: CAGE_TOPO, CAGE_SIZES = best_[0], best_[1]; g = build(dm, dc, xB, zB)
say(f"final choice: motors {dm:.1f} ahead, CG {cg_of(g)[1][0] - dm:+.1f} in x ({len(tries)} points tried)")
tot, cg = cg_of(g)                                                    # the layout drawn with the final sizes (the checks below read this one)
CAM_DESC = f"two, yaw ±{CAM_YAW:.1f}, pitch {CAM_PITCH:.1f}, landscape"

# ---------------------------------------------------------------- checks on the chosen layout
mx_front = g["motors_xy"][0]
r_front = math.hypot(*mx_front); r_rear = math.hypot(*g["motors_xy"][1])
say(f"\nchosen: catch centre x {-dc:.1f} ({TIP_OFF:.1f} behind the thrust centre, swap CG {SWAP_BACK:.0f} behind, layout B), motor square {dm:.1f} ahead of the plate centre; motors at ({dm + HALF:.1f}, ±{HALF}) and ({dm - HALF:.1f}, ±{HALF})")
say(f"arms: front r {r_front:.0f} (stock 243), rear r {r_rear:.0f}; tip gaps 25 all round (square kept)")
P_FINAL = g["P"]
FRAME = co.evaluate(P_FINAL, CAGE_TOPO, CAGE_SIZES)
cs = FRAME["cases"]
def worst_of(prefix, key):
    sel = {k: v for k, v in cs.items() if k.startswith(prefix)}
    k = max(sel, key=lambda k: sel[k][key]); return k, sel[k][key]
cr = {k: v for k, v in cs.items() if k.startswith("crash")}
wdef = max(cr, key=lambda k: cr[k]["defl"])
say(f"frame model on the final geometry (F, dimensions L cases): L2 mode {FRAME['f_l2']:.0f} Hz (≥ {co.F_L2_MIN:.0f}), cage first mode {FRAME['f_first']:.0f} Hz (moves "
    + ", ".join(f"{n_} {v_:.0%}" for n_, v_ in FRAME["first_mode_where"]) + "); "
    + (f"crash {co.crash_force(P_FINAL['m_drone_g'] * 1e-6):.0f} N (rigid {co.CRASH_G:g} g x {P_FINAL['m_drone_g']:.0f} g, L)" if co.CRASH_MODE == "rigid" else
       f"crash {co.crash_force(P_FINAL['m_drone_g'] * 1e-6):.0f} N (energy: {co.V_HIT} m/s x {P_FINAL['m_drone_g']:.0f} g over a {co.STROKE:.0f} stroke)")
    + f" in 14 directions on {sum(1 for k_ in cs if k_.startswith('crash'))} hit points (members ≤ {1 / co.NODE_FUSE:.2f}, the node is the fuse): worst rod {max(v['rod'] for v in cr.values()):.2f}, buckling {max(v['buckle'] for v in cr.values()):.2f}, "
    f"sockets {max(v['socket'] for v in cr.values()):.2f}, arm {max(v['arm'] for v in cr.values()):.2f} (≤ 1.0), touching node moves ≤ {cr[wdef]['defl']:.1f} (≤ 9); "
    f"L2 at {co.CRASH_G:.0f} g: worst {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('L2 crash')):.2f}; "
    f"handling 50 N: {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('handling')):.2f}; "
    f"latch {max(max(v['rod'], v['socket'], v['buckle']) for k, v in cs.items() if k.startswith('latch')):.2f}; violation {FRAME['violation']:.3f}; "
    f"breaking order (L): worst cage node socket {FRAME['breaking_order']['node']:.2f}, arms at {FRAME['breaking_order']['arm_vs_node']:.2f} of it (≤ 0.67) "
    f"{'PASS' if FRAME['breaking_order']['node_first'] else 'FAIL'}; held sockets rated (≤ 1 / 1.5 of the node index): "
    + ", ".join(f"{k} {v:.0f} N" for k, v in FRAME['breaking_order']['joint_req_N'].items())
    + f"; worst rod {FRAME['breaking_order']['rod']:.2f}")
say(f"L2: base centre ({xB:.1f}, 0, {zB:.0f}), axis 15 deg below +x; body lowest z {g['l2'].bounds[0][2]:.1f} vs funnel top {FUNNEL_TOP_D:.1f} (drone frame)")
allitems = [("props", trimesh.util.concatenate(g["props"])), ("motors", trimesh.util.concatenate(g["motors"])), ("arms", trimesh.util.concatenate(g["arms"])),
            ("top plate", TOP), ("bottom plate", BOTTOM), ("apex", g["apex"]), ("cage up", g["cage_up"]), ("cage low", g["cage_low"]),
            ("wheels", trimesh.util.concatenate(g["wheels"])), ("front camera", g["cam"]), ("cam bracket", g["cam_bracket"]),
            ("cartridge", g["cart"]), ("pods", trimesh.util.concatenate([g["cam_pod"], g["flow_pod"]])), ("feet", trimesh.util.concatenate(g["feet"])),
            ("Pi", PI), ("rear wing", trimesh.util.concatenate([PDB, UBEC, BUCK, RX]))]
nm, mg = l2_clear_margin(g, g["B"], allitems)
say(f"L2 scan ({90 + L2_NEG:.0f} deg from the axis, from the top of its {L2_BASE_H:.0f} base; Spark 24 Oct: the head exposed, nothing in the scan): closest {nm} {mg:.1f} outside it (C, > 0 = clear)")
# blocked share of the L2 scan by any cage rod (must be 0: Spark 24 Oct)
rng = np.random.default_rng(1); dirs = rng.normal(size=(40000, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
dirs = dirs[dirs @ A > -math.sin(math.radians(L2_NEG))]; origin = g["B"] + A * (L2_BASE_H + 15)   # the scan from the head (E)
blocked = np.zeros(len(dirs), bool)
for k in g["rods"]:
    a_, b_, od_, _, _ = g["rods"][k]; a_, b_ = np.asarray(a_, float), np.asarray(b_, float)
    u_ = b_ - a_; w0 = origin - a_; aa = dirs @ dirs.T[:, :1].T if False else None
    # closest distance between each ray (origin + t d, t > 0) and the segment
    uu = u_ @ u_; ud = dirs @ u_; dw = dirs @ w0; uw = u_ @ w0
    den = uu - ud ** 2; sc_ = np.clip((ud * dw - uw) / np.maximum(den, 1e-9), 0, 1); tc = np.maximum(sc_ * ud - dw, 0)
    dist = np.linalg.norm((w0[None, :] + tc[:, None] * dirs) - sc_[:, None] * u_[None, :], axis=1)
    blocked |= dist < od_ / 2
say(f"L2 scan blocked by cage rods: {blocked.mean() * 100:.2f} % (C, rays from the L2 head; 0 required)")
nose = max(g["l2"].bounds[1][0], g["cam"].bounds[1][0], g["cage_nose"].bounds[1][0])
say(f"length: rear prop tips x {dm - HALF - PROP_R:.0f} to the nose x {nose:.0f} ({nose - (dm - HALF - PROP_R):.0f}); width 533; "
    f"height {max(g['cam_bracket'].bounds[1][2], g['cam'].bounds[1][2], PROP_Z) - g['feet'][0].bounds[0][2]:.0f}")
FOV_H, FOV_V = 89.0, 73.0                                           # landscape (E, 102 diag 4:3)
def in_view(cm, P_):
    d = P_ - cm["lens"]; x = d @ cm["fwd"]; y = d @ cm["left"]; z = d @ cm["up"]
    return (x > 0) & (np.abs(y) <= x * math.tan(math.radians(FOV_H / 2))) & (np.abs(z) <= x * math.tan(math.radians(FOV_V / 2)))
cov = cam_cover.cover([(sy * CAM_YAW, CAM_PITCH, 0.0, False) for sy in (1, -1)])
for cm in g["cams"]:
    say(f"front camera {'left' if cm['sy'] > 0 else 'right'}: {CAM_DESC}, lens at ({cm['lens'][0]:.0f}, {cm['lens'][1]:.0f}, {cm['lens'][2]:.0f}), "
        f"{cm['back']:.1f} behind the L2 base plane")
    for name, m in (("props", trimesh.util.concatenate(g["props"])), ("arms", trimesh.util.concatenate(g["arms"])), ("motors", trimesh.util.concatenate(g["motors"])),
                    ("L2", g["l2"]), ("apex", g["apex"]), ("other camera", g["cam"])):
        n = int(in_view(cm, pts(m)).sum())
        if n and name != "other camera": say(f"   {name} in its view: {n} points")
    # blocked share of its image by cage rods (thin rods allowed in camera fields, share reported)
    rr = np.random.default_rng(2); u = rr.uniform(-1, 1, size=(20000, 2))
    dirs = (cm["fwd"][None, :] + u[:, :1] * math.tan(math.radians(FOV_H / 2)) * cm["left"][None, :] + u[:, 1:] * math.tan(math.radians(FOV_V / 2)) * cm["up"][None, :])
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]; bl = np.zeros(len(dirs), bool); who = {}
    for k, (a_, b_, od_, _, _) in g["rods"].items():
        a_, b_ = np.asarray(a_, float), np.asarray(b_, float); u_ = b_ - a_; w0 = cm["lens"] - a_
        uu = u_ @ u_; ud = dirs @ u_; dw = dirs @ w0; uw = u_ @ w0
        den = uu - ud ** 2; sc_ = np.clip((ud * dw - uw) / np.maximum(den, 1e-9), 0, 1); tc = np.maximum(sc_ * ud - dw, 0)
        hit = np.linalg.norm((w0[None, :] + tc[:, None] * dirs) - sc_[:, None] * u_[None, :], axis=1) < od_ / 2
        if hit.any(): who[k] = hit.mean(); bl |= hit
    say(f"   cage rods block {100 * bl.mean():.1f} % of its image (C): " + ", ".join(f"{k} {100 * v:.1f}" for k, v in sorted(who.items(), key=lambda t: -t[1])[:4]))
say(f"front cameras: together {100 * cov:.1f} % of the L2 field in view (C, cam_cover.py; one camera at best 27.6 %)")
# camera arm first mode (L Sensor mounts): a PETG rod r 6 cantilevered from the apex lug, camera 15 g + its plate at the tip
_I = math.pi * 6.0 ** 4 / 4; _mt = (15.0 + abs(g["cam_plates"][0].volume) * 1.27e-3 * 0.5) * 1e-6
_fa = [math.sqrt(3 * 2780.0 * _I / L_ ** 3 / (_mt + 0.24 * math.pi * 36 * L_ * 1.27e-9)) / (2 * math.pi) for L_ in g["cam_arm_len"]]
say(f"camera arms: {min(g['cam_arm_len']):.0f}..{max(g['cam_arm_len']):.0f} long, first bending mode {min(_fa):.0f} Hz (≥ 100, E: rigid lug, PETG E 2780)")
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
cage_g = (rod_g + spar_g + 8 * 0.25 + sum(abs(m.volume) for m in g["nodes"]) * 1.27e-3 * 0.6 + sum(abs(m.volume) for m in g["ring_nodes"]) * 1.27e-3 * 0.5
          + sum(abs(m.volume) for m in g["runners"]) * 1.27e-3 * 0.4 + abs(g["pan"].volume) * 1.27e-3 * 0.5)
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
    f"keep-alive band G (8) {G8_CLEAR:.1f} (≥ {G8_OFF:.0f}); below the face edges {low:+.1f} past 191 x 168 less 3 a side at ±5 deg yaw (≤ 0); "
    f"the latch set (receiver + cartridge, the battery latch's: its check vs the real panels 3.42 a side): G (8) {LATCH_G8:.1f}, past the 3 a side box {LATCH_LOW:+.1f} (receiver guide posts 183.6 long here vs 179.3 in its check)")
say(f"mass ~{tot:.0f} g (E: FC 10 g alone + 4 ESCs on the arms, no steel plate; record 2.0 kg P was the stock frame and gear), "
    f"CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}) vs thrust centre ({dm:.1f}, 0): {cg[0] - dm:+.1f} in x")
# balance pack in and out (CLAUDE.md "Balanced"; layout B, Spark 24 Oct: the pack-out shift is an accepted exception)
cg_out = (cg * tot - g["swap_cg"] * SWAP_G) / (tot - SWAP_G)
say(f"balance: CG {np.hypot(cg[0] - dm, cg[1]):.1f} off the thrust axis with the pack (≤ 3), {np.hypot(cg_out[0] - dm, cg_out[1]):.1f} without "
    f"(layout B, Spark 24 Oct: accepted, reported)")
# system_check manifest (cage first, fit, compact, mass; grouped meshes)
def tm(m): return m.copy()
# cage CG from the rod model (tubes by g/m, not solid mesh volume) + printed cage pieces by volume
_cm, _cw = np.zeros(3), 0.0
for k_, v_ in g["rods"].items():
    w_ = float(np.linalg.norm(np.subtract(v_[1], v_[0]))) * co.SECTIONS[v_[4]]["gpm"] / 1000; _cm += w_ * (np.asarray(v_[0]) + np.asarray(v_[1])) / 2; _cw += w_
for ms_, f_ in ((g["nodes"], 0.6), (g["ring_nodes"], 0.5), (g["runners"], 0.4), ([g["pan"]], 0.5), (g["wheels"], 0.5), (g["feet"], 0.5)):
    for m_ in ms_:
        w_ = abs(m_.volume) * 1.27e-3 * f_; _cm += w_ * m_.bounding_box.centroid; _cw += w_
cage_cg = _cm / _cw
cage_parts = [g["cage_up"], g["cage_low"], g["cage_nose"], trimesh.util.concatenate(g["wheels"]), trimesh.util.concatenate(g["feet"])] + g["runners"] + [g["pan"]]
# contact allowances joint by joint: each rod end's own overlap with the piece it is designed to sit in (its socket), summed
# per part pair (x 1.05 + 1 for mesh noise); any other overlap shows in the fit check
J_ARM = ("tipb", "tipt", "armbot", "arm", "root"); J_HUB = ("fhubB", "fhubT", "chin", "rhubB", "Shi", "Slo", "hubc", "SP")
JOINTS = {"arms": [], "apex": [], "sled": []}
for k_, (a_, b_) in g["rod_ends"].items():
    rm_ = g["rod_meshes"][k_]
    if any(n_.startswith(J_ARM) for n_ in (a_, b_)): JOINTS["arms"].append((k_, sum(overlap(rm_, am_) for am_ in g["arms"])))
    if any(n_.startswith(J_HUB) for n_ in (a_, b_)): JOINTS["apex"].append((k_, overlap(rm_, g["apex"])))
    if k_.startswith(("skid leg", "belly bar")): JOINTS["sled"].append((k_, overlap(rm_, union([g["sled"], g["cam_pod"], g["flow_pod"]]))))
JOINTS["apex"] += [("hub on the pan", sum(overlap(h_, g["pan"]) for h_ in g["hubs"]))]
J_ALLOW = {k_: round(sum(v_ for _, v_ in js_) * 1.05 + 1.0, 1) for k_, js_ in JOINTS.items()}
os.makedirs(OUT, exist_ok=True); json.dump({k_: [[n_, round(float(v_), 1)] for n_, v_ in js_] for k_, js_ in JOINTS.items()}, open(os.path.join(OUT, "joints.json"), "w"), indent=0)
say(f"contact allowances (per joint, M): cage/arms {len(JOINTS['arms'])} sockets {J_ALLOW['arms']} mm3, cage/apex + hubs {len(JOINTS['apex'])} sockets {J_ALLOW['apex']} mm3, "
    f"cage/sled {len(JOINTS['sled'])} clamps {J_ALLOW['sled']} mm3")
# what must be reached without taking a part off (concept check): connectors, the SD card, the arm bolts (DEFAULT connector sides where unmeasured)
_l2lx = np.array([0.0, -1.0, 0.0])            # L2 local x in the drone frame: rolled 90 deg about its axis so the cable notch faces left, where the
                                               # collar is 93 wide (44.5 up and down, where the nose bars sit; DEFAULT, the scan is round)
_tz, _bz = float(TOP.bounds[1][2]), float(BOTTOM.bounds[0][2])
ACCESS = [dict(name="FC USB-C (right edge 8 behind the FC centre, DEFAULT: clear of the front arm)", part="electronics", **{"from": [14.0, -18.0, 10.0]}, dir=[0, -1, 0], length_mm=45, tool={"type": "box", "w": 12, "h": 7, "w_axis": [1, 0, 0]}),
          dict(name="Pi 5 SD card (front end)", part="electronics", **{"from": [42.5, 0.0, 24.0]}, dir=[1, 0, 0], length_mm=25, tool={"type": "box", "w": 12, "h": 3, "w_axis": [0, 1, 0]}),
          dict(name="Pi 5 USB / Ethernet end (aft, DEFAULT: the spine hub sits ahead)", part="electronics", **{"from": [-42.5, 0.0, 32.0]}, dir=[-1, 0, 0], length_mm=35, tool={"type": "box", "w": 16, "h": 14}),
          dict(name="L2 cable (base notch 7 up the base, local -X: left, through the shroud window)", part="L2 base", **{"from": [float(v) for v in g["B"] - 37.5 * _l2lx + A * 7.0]},
               dir=[float(v) for v in -_l2lx], length_mm=30, tool={"type": "cylinder", "d": 8}),
          dict(name="XT60 lead up through the sled", part="receiver XT60", **{"from": [float(v) for v in (g["xt60"].bounds.mean(0) * [1, 1, 0] + [0, 0, g["xt60"].bounds[1][2]])]}, dir=[0, 0, 1], length_mm=30, tool={"type": "cylinder", "d": 9})]
ACCESS += [dict(name=f"front camera {'left' if c_['sy'] > 0 else 'right'} USB (board edge, up)", part="front cameras", **{"from": [float(v) for v in c_["c"] + c_["up"] * 19]},
                dir=[float(v) for v in c_["up"]], length_mm=30, tool={"type": "box", "w": 10, "h": 6}) for c_ in g["cams"]]
ACCESS += [dict(name=f"arm bolt {wh_} {i_}", part="plates", **{"from": [xy_[0], xy_[1], _tz if wh_ == "top" else _bz]}, dir=[0, 0, 1 if wh_ == "top" else -1], length_mm=60,
                tool={"type": "cylinder", "d": 8}, **({"ignore": ["props"]} if wh_ == "top" else {}))   # 60 hex driver (DEFAULT); a prop over a bolt is turned aside by hand, never taken off
           for wh_ in ("top", "bottom") for i_, xy_ in enumerate(plate_bolts(wh_))]
if os.environ.get("SYS_DEBUG_FIT"):                                # what overlaps the apex and hubs beyond the rod-end sockets
    own_ = dict(JOINTS["apex"])
    rows_ = sorted(((overlap(rm_, g["apex"]) - own_.get(k_, 0.0), k_) for k_, rm_ in g["rod_meshes"].items()), reverse=True)[:8]
    print("rods x apex beyond own socket:", [(k_, round(v_, 1)) for v_, k_ in rows_])
    print("nodes x apex:", round(sum(overlap(n_, g["apex"]) for n_ in g["nodes"]), 1), "ring nodes x apex:", [(i_, np.round(n_.bounding_box.centroid, 1).tolist(), round(overlap(n_, g["apex"]), 1)) for i_, n_ in enumerate(g["ring_nodes"]) if overlap(n_, g["apex"]) > 1])
    print("cage parts x apex:", [(nm_, round(overlap(m_, g["apex"]), 1)) for nm_, m_ in (("wheels", trimesh.util.concatenate(g["wheels"])), ("feet", trimesh.util.concatenate(g["feet"])), ("runners", trimesh.util.concatenate(g["runners"])), ("pan", g["pan"]))])
    for nm_ in ("cage_up", "cage_low", "cage_nose"): print(nm_, "x apex", round(overlap(g[nm_], g["apex"]), 1))
    print("sum own", round(sum(own_.values()), 1))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); import system_check as sc_
    named_ = dict(g["rod_meshes"]); named_.update({f"node {i_}": n_ for i_, n_ in enumerate(g["nodes"])})
    named_.update({k_: g[k_] for k_ in ("sled", "cam_pod", "flow_pod", "pan", "apex", "receiver") if k_ in g})
    named_.update({f"prop {i_}": p_ for i_, p_ in enumerate(g["props"])}); named_.update({f"hub {i_}": h_ for i_, h_ in enumerate(g["hubs"])})
    named_.update({f"cam bracket": g["cam_bracket"]}); named_.update({f"runner {i_}": r_ for i_, r_ in enumerate(g["runners"])})
    for ac_ in ACCESS:
        tm_ = sc_.sweep_tool(ac_, None, None)[0]
        hit_ = sorted(((overlap(tm_, m_), k_) for k_, m_ in named_.items() if not (k_.startswith("prop") and "props" in ac_.get("ignore", []))), reverse=True)[:4]
        print("access", ac_["name"], [(k_, round(v_, 1)) for v_, k_ in hit_ if v_ > 0.5])
    sys.exit(0)
man = dict(name="overseer_system_v3", cg_ref=[dm, 0.0, 0.0], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cg_tol_no_swap_mm=15.0, cage_margin_mm=3.0, n_dirs=1500,
           contact_pairs=[["cage", "arms", J_ALLOW["arms"]],                         # designed joints, summed per joint (above): rod ends in the arm sockets,
                          ["cage", "sled + pods + deck", J_ALLOW["sled"]],                 # the skid legs and belly bars through the sled and pod clamps,
                          ["cage", "apex + hubs", J_ALLOW["apex"]],                        # rod ends in the hub and apex sockets, hub blocks on the pan,
                          ["plates", "apex + hubs", round(g["hub_plate_mm3"], 1)],         # hub blocks clamped on the plates,
                          ["arms", "apex + hubs", round(g["hub_arm_mm3"], 1)],             # front hub blocks seated on the front arm roots,
                          ["pack", "cartridge", 0.5], ["cartridge", "receiver", 0.5], ["receiver", "receiver XT60", round(abs(g["receiver"].intersection(g["xt60"]).volume) * 1.05 + 1.0, 1)],   # the XT60 panel mount in its receiver pocket
                          ["receiver", "sled + pods + deck", round(abs(g["receiver"].intersection(g["sled"]).volume) * 1.05 + 1.0, 1)],   # bolted to the sled underside
                          ["apex + hubs", "cam brackets", round(abs(g["apex"].intersection(g["cam_plate"]).volume) * 1.05 + 1.0, 1)]],  # camera plates printed onto their arms and socket lugs
           openings=[dict(part="L2 base", axis=[float(v) for v in co.A], half_angle_deg=90.0, min_mm=-1000.0, skip_probes=True),   # the head side: the exposed band and head (Spark 24 Oct)
                                                                                                    # sit on the base's whole top face, the shroud holds its sides up to them
                     *[dict(part=pt_, axis=[0, 0, -1], half_angle_deg=90.0, min_mm=0.0, skip_probes=True) for pt_ in ("pack", "cartridge", "receiver XT60")],   # the swap path: the set fills the opening under the wheels, where nothing else may go; the cartridge and receiver walls take a side hit there (DEFAULT, alt end bumpers with a wider funnel)
                     *[dict(part=pt_, axis=[sx_, 0, 0], half_angle_deg=45.0, min_mm=0.0, skip_probes=True) for pt_ in ("pack", "cartridge", "receiver XT60") for sx_ in (1, -1)],   # the set's ends face the funnel pit walls (0.7 to the opening): a member shielding them up to 45 deg would sit in the wall (DEFAULT, alt a shorter set, latch session)
                     dict(part="down cam + flow", axis=[0, 0, -1], half_angle_deg=90.0, min_mm=0.0, skip_probes=True)],         # under the belly: the swap path must stay open (as the pack); the runners, belly frame and cartridge shield the sides (DEFAULT, alt a removable guard bar)
           access=[dict(name="pack swap", part="cartridge", dir=[0, 0, -1], length_mm=80, tool="part", ignore=["pack", "receiver", "receiver XT60"])] + ACCESS, parts=[
    dict(name="cage", trimesh=trimesh.util.concatenate(cage_parts), role="cage", mass_g=cage_g, cg=[float(v) for v in cage_cg], tag="E"),
    dict(name="props", trimesh=trimesh.util.concatenate([cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, *xy, 64) for xy in g["motors_xy"]]), role="protected", mass_g=40, tag="E"),
    dict(name="motors", trimesh=trimesh.util.concatenate(g["motors"]), role="protected", mass_g=324, tag="P"),
    dict(name="L2 base", trimesh=g["l2"].slice_plane(g["B"] + A * GUARD_TOP, -A, cap=True), role="protected", mass_g=70, tag="E"),   # the base inside the shroud: caged
    dict(name="L2 base top band", trimesh=g["l2"].slice_plane(g["B"] + A * GUARD_TOP, A, cap=True).slice_plane(g["B"] + A * L2_BASE_H, -A, cap=True),
         role="bought", mass_g=30, tag="E"),                                                                                                  # 8 under the head: a guard there would sit in the scan (exposed with the head)
    dict(name="L2 head", trimesh=g["l2"].slice_plane(g["B"] + A * L2_BASE_H, A, cap=True), role="bought", mass_g=130, tag="E"),        # exposed (Spark 24 Oct: nothing in the scan)
    dict(name="front cameras", trimesh=g["cam"], role="protected", mass_g=34, tag="E"),
    dict(name="pack", trimesh=g["pack"], role="protected", mass_g=650, tag="P", swap=True),
    dict(name="cartridge", trimesh=g["cart"], role="protected", mass_g=round(SWAP_G - 650, 1), tag="M", swap=True),
    dict(name="receiver", trimesh=g["receiver"], role="structure", mass_g=RECEIVER_G - 10, tag="E"),   # printed latch frame: its ends sit at the pit walls (0.7 to the opening), no cage member fits past them (DEFAULT, alt protected: needs a shorter receiver, latch session)
    dict(name="receiver XT60", trimesh=g["xt60"], role="protected", mass_g=10, tag="E"),
    dict(name="electronics", trimesh=trimesh.util.concatenate([FC, PI, PDB, UBEC, BUCK, RX]), role="protected", mass_g=110 + 80, tag="E"),
    dict(name="down cam + flow", trimesh=trimesh.util.concatenate([g["down_cam"], g["flow"]]), role="protected", mass_g=17, tag="E"),
    dict(name="arms", trimesh=trimesh.util.concatenate(g["arms"]), role="structure", mass_g=sum(abs(a.volume) for a in g["arms"]) * 1.27e-3 * 0.45 + 100, tag="E"),
    dict(name="plates", trimesh=trimesh.util.concatenate([TOP, BOTTOM]), role="structure", mass_g=84, tag="C"),
    dict(name="sled + pods + deck", trimesh=trimesh.util.concatenate([g["sled"], g["cam_pod"], g["flow_pod"], DECK]), role="structure",
         mass_g=(abs(g["sled"].volume) + abs(g["cam_pod"].volume) + abs(g["flow_pod"].volume)) * 1.27e-3 * 0.3 + abs(DECK.volume) * 1.27e-3 * 0.5, tag="E"),
    dict(name="apex + hubs", trimesh=g["apex"], role="structure", mass_g=abs(g["apex"].volume) * 1.27e-3 * 0.45, tag="E"),
    dict(name="cam brackets", trimesh=g["cam_bracket"], role="structure", mass_g=round(abs(g["cam_bracket"].volume) * 1.27e-3 * 0.5, 1), tag="E")])
try:
    with contextlib.redirect_stdout(io.StringIO()):
        SYS = system_check_run(man)
    say("system_check: " + SYS["summary"].replace("\n", " | "))
    for k in ("probes", "access", "fit"):
        w = (SYS.get(k) or {}).get("worst")
        if w: say(f"  {k} worst: " + json.dumps(w, default=str)[:600])
    for p_ in (SYS.get("access") or {}).get("paths", []):                # every reach path, one line each
        say(f"  access {p_['name']}: {'clear' if p_['pass_'] else 'BLOCKED by ' + ', '.join(f'{b_}' for b_ in p_.get('blocked_by', []))} ({p_['overlap_mm3']} mm3)")
    wf = (SYS.get("fit") or {}).get("worst")
    if wf and not wf["pass_"] and "cage" in (wf["a"], wf["b"]):          # which rods make the cage's overlap
        other = [pt["trimesh"] for pt in man["parts"] if pt["name"] == (wf["b"] if wf["a"] == "cage" else wf["a"])][0]
        ov = []
        for k_, rm in g["rod_meshes"].items():
            lo_ = np.maximum(rm.bounds[0], other.bounds[0]); hi_ = np.minimum(rm.bounds[1], other.bounds[1])
            if np.any(hi_ <= lo_): continue
            try: v_ = abs(rm.intersection(other).volume)
            except Exception: v_ = -1
            if v_ > 0.5: ov.append((round(v_), k_))
        say("  cage rods in that pair: " + ", ".join(f"{k_} {v_}" for v_, k_ in sorted(ov, reverse=True)[:8]))
except Exception as e:
    SYS = None; say(f"system_check failed to run: {str(e)[:200]}")
# the system's assembly file (nox/ASSEMBLY.md: this script is its only writer); SYS_WRITE_ASM=1 on the final run
if os.environ.get("SYS_WRITE_ASM", "0") == "1":
    import re, assembly
    ASM_VER = os.environ.get("SYS_ASM_VER", "v1")
    ASM_DIR = os.path.join(ROOT, "runs", "overseer_assembly", ASM_VER); os.makedirs(ASM_DIR, exist_ok=True)
    aparts = []
    for pt in man["parts"]:
        slug = re.sub(r"[^a-z0-9]+", "_", pt["name"].lower()).strip("_")
        mp = os.path.join(ASM_DIR, slug + ".stl"); pt["trimesh"].export(mp)
        d = {k: v for k, v in pt.items() if k != "trimesh"}
        d.update(mesh=os.path.relpath(mp, ROOT), owner="overseer", version=f"system {ASM_VER} (layout B)", source="nox/overseer/overseer_system.py")
        aparts.append(d)
    asm = assembly.new("Overseer", "overseer", aparts,
        note=(f"System {ASM_VER} from nox/overseer/overseer_system.py (layout B, Spark 24 Oct: swap CG {SWAP_BACK:.0f} behind the thrust axis, no tail, "
              f"motors {dm:.1f} ahead of the plate centre, L2 base ({xB:.1f}, 0, {zB:.0f}) 15 deg below +x, wheels z {WHEEL_Z:.0f}; cage sized on this geometry, "
              f"runs/overseer_system/{os.path.basename(OUT.rstrip('/'))}). "
              "Drone frame: x fwd, y left, z up, z 0 bottom plate top. Parts are grouped as the system pass draws them. "
              "Battery latch set v1 (cartridge, pack, receiver frame, XT60) from runs/box_system/v2/latch_ctx. "
              "Left out (no pose yet): ESCs as bodies (25 g each in the arm masses), wheel pins, keep-alive pads as bodies, "
              "scan stick mount (open: its chat sets the mount; loads in dimensions O System Overseer)."),
        cg_ref=[float(dm), 0.0, 0.0], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cg_tol_no_swap_mm=man["cg_tol_no_swap_mm"], cage_margin_mm=3.0, n_dirs=1500,
        openings=man["openings"], access=man["access"], contact_pairs=man["contact_pairs"],
        guard_offsets={"guard rods to prop disks, plan (G)": float(co.GUARD_GAP), "rods under / over the swept band (G)": float(co.GUARD_GAP)})
    asm.save(os.path.join(HERE, "overseer.assembly.json"))
    say(f"assembly: nox/overseer/overseer.assembly.json, {len(aparts)} parts, meshes in {os.path.relpath(ASM_DIR, ROOT)}")
FLOW_POS = (g["flow_lens"] - cg) * [1, -1, -1] / 1000
say(f"PX4 flow position from the CG (FRD, m): x {FLOW_POS[0]:+.3f}, y {FLOW_POS[1]:+.3f}, z {FLOW_POS[2]:+.3f}; "
    f"IMU (FC centre) from the CG: x {(0 - cg[0]) / 1000:+.3f}, z {-(9.75 - cg[2]) / 1000:+.3f}")

# ---------------------------------------------------------------- motion artifact + run record
s = Scene("Overseer system v2", f"full cage (frame model): L2 mode {FRAME['f_l2']:.0f} Hz, crash {co.crash_force(M_DRONE * 1e-6):.0f} N, tip-over ≥ {min(tips.values()):.0f} deg; pack {TIP_OFF:.0f} behind the thrust axis, motors {dm:.0f} ahead of the plates")
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
SEC_COL = {"r3": "#3a3a3a", "r4": "#2b4f7a", "t6": "#3f7a3f", "r5": "#7a5a2b", "t8": "#8a3f8a", "t10": "#a33a3a", "t12": "#c0662b", "t14": "#b8a02b", "t16": "#2b8a8a", "t20": "#5a2b8a", "t6h": "#5f9a5f", "t8h": "#b066b0", "t10h": "#d04a4a", "r10": "#701c1c"}
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
P("front cameras", g["cam"], "bought", "#2d5a2d", False, CAM_DESC)
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
json.dump(P_FINAL, open(os.path.join(OUT, "params.json"), "w"), indent=1, default=float)
json.dump(dict(topo=CAGE_TOPO, sizes=CAGE_SIZES, mass_g=FRAME["mass_g"], f_l2=FRAME["f_l2"], f_first=FRAME["f_first"], violation=FRAME["violation"]),
          open(os.path.join(OUT, "cage_final.json"), "w"), indent=1, default=float)
if os.environ.get("SYS_VIDEO", "0") == "1":                     # iterations skip the video (png still useful: SYS_VIDEO=1 on the final run)
    try:
        s.video(os.path.join(OUT, "motion.mp4"), gif=False)
    except Exception as e:
        say("video skipped:", str(e)[:200])
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
