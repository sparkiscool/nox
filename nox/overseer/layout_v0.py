"""Overseer layout v0 (rough design, 24 Oct): stock S500 plates, printed arms at 395 with the catch wheels hung under them,
everything else between the plates, Unitree L2 on a mast on the top plate, battery centred under the bottom plate,
down camera and flow sensor in pods beside the battery. Boxes for the bought parts, rough printed shapes; numbers from
`nox/dimensions.md` O / M / G (tagged there), E where marked here. Checks every clearance and writes the motion artifact.

python3 nox/overseer/layout_v0.py [out_dir]      (~2 min: reads the S500 STEP)

Drone frame: x forward, y left, z up; z 0 = bottom plate top face; arms on the diagonals (quad X).
"""
import math, os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.join(HERE, "s500"))
from motion import Scene
import s500_cad
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_layout", "v0")
S2 = 1 / math.sqrt(2)

# ---------------------------------------------------------------- numbers
WB = math.sqrt(2) * (254 + 25)          # 394.6, 25 tip gap (C, dimensions O Frame)
RM = WB / 2                              # motor radius 197.3
ROOT_H = 45.0                            # arm root / plate gap (DEFAULT, O Arms and hub)
TOP_T = 1.6                              # top plate (derived)
H_TIP = 16.0                             # arm depth under the motor (E)
PAD_Z = ROOT_H                           # motor pad top, flush with the arm top
MOTOR_H = 26.0                           # X2216 body (E)
PROP_Z = PAD_Z + 37.0                    # prop hub mid plane (E: body 26 + adapter collar 6 + half hub 5)
PROP_R, BLADE = 127.0, 5.0               # 1045; blade flex / thickness band ±5 (E)
L2_W, L2_H = 75.0, 65.0                  # spec
L2_DOWN = 6.0                            # negative-angle mode sees 6 deg below the base plane (spec)
CAM_UP = 30.0                            # front camera tilt (DEFAULT, O Camera)
CAM_HFOV, CAM_VFOV = 73.0, 89.0          # portrait, 4:3 assumed from 102 diag (E, O Camera)
DOWN_HFOV, DOWN_VFOV = 89.0, 73.0        # down camera, landscape (E)
FLOW_HALF = 23.0                         # flow view half angle incl. 2 margin (spec + 2)
PACK = (148.0, 48.0, 45.0)               # 4S 6000 (P)
PACK_TOP = -7.0                          # 5 under the bottom plate for the cartridge latch block (E)
WHEEL_R, WHEEL_W = 4.0, 3.0              # catch wheels (G)
WHEEL_Z = -12.0                          # wheel centres in the drone frame (DEFAULT)
OPEN_HALF, FACE_TOP = 84.0, 75.1         # funnel: opening 168 at seat z -0.8, top z 75.1, 45 deg faces (G)


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


# ---------------------------------------------------------------- stock plates (Holybro S500 STEP, derived)
parts = s500_cad.load(s500_cad.STEP)
CAD2DRONE = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # (X, Y, Z) -> (X, -Z, Y)


def cad_mesh(key, dz):
    import tempfile
    shp = s500_cad.find(parts, key)
    BRepMesh_IncrementalMesh(shp, 0.1, False, 0.3, True)
    with tempfile.TemporaryDirectory() as tmp:
        StlAPI_Writer().Write(shp, os.path.join(tmp, "p.stl")); m = trimesh.load(os.path.join(tmp, "p.stl"))
    m.apply_transform(CAD2DRONE); m.apply_translation([0, 0, dz]); return m


bottom = cad_mesh("BOTTOM-PLATE-S500", -0.10)                 # top face CAD Y 0.10 -> z 0
top = cad_mesh("TOP-PLATE-S500", ROOT_H - 18.04)              # underside CAD Y 18.04 -> z 45


# ---------------------------------------------------------------- printed arms (rough), arm frame u along, v across
def arm_bottom(u):
    return 0.0 if u <= 75 else (u - 75) / (RM - 75) * (ROOT_H - H_TIP)


U_LEG = (112 + 100) * S2                                      # 149.9: wheel pair under the arm (G points)
V_WHEEL = (112 - 100) * S2                                    # 8.49 either side of the arm axis
root = box(58, 75, -21, 21, 0, ROOT_H)                        # inner face u 58 (O Arms and hub)
beam = hull(box(74.9, 75, -13, 13, 0, ROOT_H), box(RM, RM + 0.1, -11, 11, ROOT_H - H_TIP, ROOT_H))
pad = cyl(18, ROOT_H - 4, ROOT_H, RM, 0)
leg = hull(box(U_LEG - 5, U_LEG + 5, -5, 5, arm_bottom(U_LEG) + 1, arm_bottom(U_LEG) + 2),
           box(U_LEG - 3, U_LEG + 3, -5, 5, WHEEL_Z - 1.5, WHEEL_Z + 1.5))
arm0 = union([root, beam, pad, leg])
ARM_DEG = (45, 135, 225, 315)
arms = []                                                     # arm0 is built along +x (u); place it on the diagonals
for d in ARM_DEG:
    a = arm0.copy(); a.apply_transform(rotz(d)); arms.append(a)

wheels, wheel_axes = [], []
for sx in (1, -1):
    for sy in (1, -1):
        for (px, py, axis) in ((112 * sx, 100 * sy, "y"), (100 * sx, 112 * sy, "x")):
            w = trimesh.creation.cylinder(WHEEL_R, WHEEL_W, sections=32)
            w.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0] if axis == "y" else [0, 1, 0]))
            w.apply_translation([px, py, WHEEL_Z]); wheels.append(w)

motors = [cyl(13.85, PAD_Z, PAD_Z + MOTOR_H, RM * math.cos(math.radians(d)), RM * math.sin(math.radians(d))) for d in ARM_DEG]
props = [cyl(PROP_R, PROP_Z - 0.5, PROP_Z + 0.5, RM * math.cos(math.radians(d)), RM * math.sin(math.radians(d)), 96) for d in ARM_DEG]

# ---------------------------------------------------------------- inside the hub (between the plates)
fc = box(-18, 18, -18, 18, 1.0, 18.5)                         # MicoAir743-Lite + ESC stack 17.5 (spec), on 1.0 standoffs
deck = box(-46, 46, -31, 31, 20.0, 22.0)                      # printed Pi deck on 4 posts (DEFAULT)
posts = [box(sx * 40 - 2, sx * 40 + 2, sy * 25 - 2, sy * 25 + 2, 0, 20) for sx in (1, -1) for sy in (1, -1)]
pi = box(-42.5, 42.5, -28, 28, 22.5, 42.0)                    # Pi 5 85 x 56, with Active Cooler and ports ~19.5 over its feet (E)
pdb = box(-95, -45, -25, 25, 0.5, 6.5)                        # Matek FCHUB-12S 55 x 50 x 6 on the rear wing (spec)
ubec = box(50, 98, 0, 24, 0.5, 10.5)                          # 5 V 5 A UBEC, Q4 envelope (E)
buck = box(58, 76, -24, -6, 0.5, 6.5)                         # 12 V buck, Pololu D24V22F12 class (E)
rx = box(80, 96, -22, -10, 0.5, 6.5)                          # CRSF receiver (E)
escs = []
for d in ARM_DEG:                                             # 4 ESCs under the arms (ESC model open, E 45 x 24 x 8)
    e = box(92, 137, -12, 12, arm_bottom(92) - 8.5, arm_bottom(92) - 0.5)    # hangs under the arm, off the plate
    e.apply_transform(rotz(d)); escs.append(e)

# ---------------------------------------------------------------- L2 mast on the top plate (printed, rough)
TOP_Z = ROOT_H + TOP_T


def l2_base_needed():
    """lowest L2 base so the -6 deg cone from the L2 axis passes over every prop tip (+ blade band) and motor top"""
    far = RM + PROP_R                                         # farthest prop tip from the centre axis
    return PROP_Z + BLADE + far * math.tan(math.radians(L2_DOWN)) + 3.0   # 3 margin (DEFAULT)


L2_Z = math.ceil(l2_base_needed())
spider = union([box(-8, 8, -8, 8, TOP_Z, TOP_Z + 5)] + [box(0, 72, -9, 9, TOP_Z, TOP_Z + 5).apply_transform(rotz(d)) for d in ARM_DEG])
column = box(-22, 22, -22, 22, TOP_Z, L2_Z - 5)
seat = cyl(37.5, L2_Z - 5, L2_Z)
mast = union([spider, column, seat])
lidar = box(-L2_W / 2, L2_W / 2, -L2_W / 2, L2_W / 2, L2_Z, L2_Z + 39)
dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([1, 1, 26 / 35.5]); dome.apply_translation([0, 0, L2_Z + 39])
dome = dome.slice_plane([0, 0, L2_Z + 39], [0, 0, 1], cap=True)
CAM_C = np.array([22 + 4 + 2, 0.0, L2_Z - 5 - 25.0])          # front camera board centre on the column's front face, 25 under the seat (E)
cam_t = trimesh.transformations.rotation_matrix(math.radians(-CAM_UP), [0, 1, 0])   # optical axis +x tilted up 30
cam_board = box(-1, 1, -19, 19, -19, 19); lens_f = trimesh.creation.cylinder(6, 8, sections=32)
lens_f.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); lens_f.apply_translation([5, 0, 0])
front_cam = trimesh.util.concatenate([cam_board, lens_f]); front_cam.apply_transform(cam_t); front_cam.apply_translation(CAM_C)
cam_bracket = hull(box(21.9, 22, -12, 12, CAM_C[2] - 14, CAM_C[2] + 14), box(CAM_C[0] - 4, CAM_C[0] - 3, -12, 12, CAM_C[2] - 12, CAM_C[2] + 12))

# ---------------------------------------------------------------- under the bottom plate
BOT_Z = -2.1
pack = box(-PACK[0] / 2, PACK[0] / 2, -PACK[1] / 2, PACK[1] / 2, PACK_TOP - PACK[2], PACK_TOP)
PACK_BOT = PACK_TOP - PACK[2]
cart = box(-78.5, 78.5, -27, 27, PACK_BOT - 2, BOT_Z - 0.01)  # cartridge envelope ~157 x 54 (E, G), shown as a ghost
sled = box(-60, 60, -72, 72, BOT_Z - 3, BOT_Z)                 # printed belly sled on the bottom plate (DEFAULT)
CAM_DY, FLOW_DY = 49.0, -42.0
cam_pod = box(-23, 23, 29, 72, PACK_BOT + 4, BOT_Z - 3)
flow_pod = box(-15, 15, -55, -29, PACK_BOT + 4, BOT_Z - 3)
down_board = box(-19, 19, CAM_DY - 19, CAM_DY + 19, PACK_BOT + 2.4, PACK_BOT + 4)
down_lens = cyl(6, PACK_BOT - 6, PACK_BOT + 2.4, 0, CAM_DY)
LENS_DOWN = np.array([0, CAM_DY, PACK_BOT - 6])
flow = box(-10.8, 10.8, FLOW_DY - 8, FLOW_DY + 8, PACK_BOT - 2.5, PACK_BOT + 4)
FLOW_LENS = np.array([0, FLOW_DY, PACK_BOT - 2.5])
feet = [cyl(5, PACK_BOT - 9, PACK_BOT + 4, sx * 68, sy * 34) for sx in (1, -1) for sy in (1, -1)]
FEET_BOT = PACK_BOT - 9

# ---------------------------------------------------------------- checks
print(f"wheelbase {WB:.1f}, motor r {RM:.1f}, prop plane z {PROP_Z:.0f} (E), L2 base z {L2_Z:.0f} "
      f"(needs {l2_base_needed():.1f}: props z {PROP_Z + BLADE:.0f} at r {RM + PROP_R:.0f} under the -6 deg cone + 3)")
print(f"overall: span {2 * (RM * S2 + PROP_R):.0f} tip to tip, height {L2_Z + 65 - FEET_BOT:.0f} "
      f"(feet z {FEET_BOT:.0f} to L2 top z {L2_Z + 65:.0f})")

solid = {"bottom plate": bottom, "top plate": top, "FC stack": fc, "Pi deck": deck, "Pi 5": pi, "PDB": pdb, "UBEC": ubec,
         "12 V buck": buck, "RX": rx, "pack": pack, "sled": sled, "cam pod": cam_pod, "flow pod": flow_pod, "mast": mast}
for i, a in enumerate(arms): solid[f"arm {i}"] = a
for i, e in enumerate(escs): solid[f"ESC {i}"] = e
hub_pairs = [(a, b) for a in ("FC stack", "Pi deck", "Pi 5", "PDB", "UBEC", "12 V buck", "RX")
             for b in ("bottom plate", "top plate", "arm 0", "arm 1", "arm 2", "arm 3", "FC stack", "Pi deck", "Pi 5", "PDB", "UBEC", "12 V buck", "RX") if a < b or b in ("bottom plate", "top plate") or b.startswith("arm")]
bad = []
for a, b in hub_pairs:
    if a == b: continue
    v = overlap(solid[a], solid[b])
    if v > 0.5: bad.append((a, b, v))
for a, b in (("pack", "sled"), ("cam pod", "pack"), ("flow pod", "pack"), ("mast", "top plate"), ("ESC 0", "arm 0"), ("ESC 0", "bottom plate")):
    v = overlap(solid[a], solid[b]); bad.append((a, b, v)) if v > 0.5 else None
print("overlaps (M, > 0.5 mm3):", "none" if not bad else "; ".join(f"{a} x {b} {v:.0f}" for a, b, v in bad))
print(f"Pi 5 top z 42.0 under the top plate z {ROOT_H:.0f}: {ROOT_H - 42.0:.1f} clear (E Pi height)")
rects = {"Pi 5": (42.5, 28), "Pi deck": (46, 31)}
for n, (a, b) in rects.items():
    print(f"{n}: corner u {(a + b) * S2:.1f} vs root inner face u 58: {58 - (a + b) * S2:.1f} clear")


def pts(m, n=4000):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n)[0]])


def in_cone(p, apex, axis, half_deg):
    d = p - apex; ln = np.linalg.norm(d, axis=1); ln[ln == 0] = 1e-9
    return np.degrees(np.arccos(np.clip(d @ axis / ln, -1, 1))) < half_deg


def in_frustum(p, apex, fwd, up, hfov, vfov):
    d = p - apex; right = np.cross(fwd, up)
    z = d @ fwd; ok = z > 1e-6
    h = np.degrees(np.arctan2(d @ right, np.maximum(z, 1e-9))); v = np.degrees(np.arctan2(d @ up, np.maximum(z, 1e-9)))
    return ok & (abs(h) < hfov / 2) & (abs(v) < vfov / 2)


# L2 field: nothing above the -6 deg cone from its base centre (except the L2 itself)
apex = np.array([0, 0, L2_Z])
worst = None
for name, m in list(solid.items()) + [("props", trimesh.util.concatenate(props)), ("motors", trimesh.util.concatenate(motors)),
                                       ("front camera", front_cam), ("cam bracket", cam_bracket)]:
    if name == "mast": m = m.slice_plane([0, 0, L2_Z - 5.01], [0, 0, -1], cap=True)   # the seat under the L2 is its mount
    p = pts(m); r = np.hypot(p[:, 0], p[:, 1]); margin = (L2_Z - r * math.tan(math.radians(L2_DOWN))) - p[:, 2]
    if name == "props": margin -= BLADE
    i = int(np.argmin(margin))
    if worst is None or margin[i] < worst[1]: worst = (name, margin[i])
print(f"L2 field (-6 deg): closest {worst[0]} {worst[1]:.1f} under the cone (C, > 0 = clear)")

# front camera: props in its view?
fwd = cam_t[:3, :3] @ np.array([1, 0, 0]); up = cam_t[:3, :3] @ np.array([0, 0, 1])
lens_c = CAM_C + fwd * 9
pp = np.vstack([pts(p, 6000) for p in props])
seen = in_frustum(pp, lens_c, fwd, up, CAM_HFOV, CAM_VFOV)
print(f"front camera ({CAM_UP:.0f} deg up, {CAM_HFOV:.0f} x {CAM_VFOV:.0f} portrait, E): prop disk points in view {seen.mean() * 100:.1f} %"
      + ("" if not seen.any() else f", lowest corners only (elev {np.degrees(np.arcsin((pp[seen] - lens_c) @ np.array([0, 0, 1]) / np.linalg.norm(pp[seen] - lens_c, axis=1))).max():.0f} deg at most)"))
for name, m in (("mast", mast), ("lidar", lidar), ("arms", trimesh.util.concatenate(arms)), ("top plate", top)):
    s = in_frustum(pts(m), lens_c, fwd, up, CAM_HFOV, CAM_VFOV)
    if s.any(): print(f"   {name} in the front camera view: {s.mean() * 100:.1f} % of its points")

# down camera and flow: their views clear of the drone
dn = np.array([0, 0, -1.0]); fwd_up = np.array([1.0, 0, 0])
for name, m in (("feet", trimesh.util.concatenate(feet)), ("pack", pack), ("wheels", trimesh.util.concatenate(wheels)),
                ("flow pod", flow_pod), ("cam pod", cam_pod), ("arms", trimesh.util.concatenate(arms))):
    s1 = in_frustum(pts(m), LENS_DOWN, dn, fwd_up, DOWN_HFOV, DOWN_VFOV).sum()
    s2 = in_cone(pts(m), FLOW_LENS, dn, FLOW_HALF).sum()
    if s1 or s2: print(f"   {name}: {s1} points in the down camera view, {s2} in the flow cone")
print("down camera / flow views: checked (lines above list anything in view; none = clear)")

# funnel, seated: wheel rims on the 45 deg faces
SEAT_DZ = (112 - (OPEN_HALF + 0.8) + WHEEL_R * math.sqrt(2)) - WHEEL_Z   # wheel centre R off the x face, inside
others = trimesh.util.concatenate([m for k, m in solid.items()] + motors + [front_cam, cam_bracket, lidar] + feet + [cam_pod, flow_pod])
p = pts(others, 40000); p = p + [0, 0, SEAT_DZ]
d_face = np.full(len(p), np.inf)
for ax in (0, 1):
    for sg in (1, -1):
        x = sg * p[:, ax]; z = p[:, 2]; in_z = (z > -0.8) & (z < FACE_TOP)
        dist = (z + OPEN_HALF + 0.8 - x) / math.sqrt(2)
        d_face = np.where(in_z, np.minimum(d_face, dist), d_face)
i = int(np.argmin(d_face))
print(f"funnel seated (bottom plate top at seat z {SEAT_DZ:.1f}): closest non-wheel point to a face {d_face[i]:.1f} "
      f"at ({p[i, 0]:.0f}, {p[i, 1]:.0f}, {p[i, 2]:.0f}) (G rule ≥ 1.0)")
low = p[p[:, 2] < -0.8]
worst_low = 0.0
for yaw in np.radians(np.linspace(-5, 5, 11)):
    c, s = math.cos(yaw), math.sin(yaw)
    q = low[:, :2] @ np.array([[c, -s], [s, c]]).T
    worst_low = max(worst_low, np.abs(q).max())
print(f"below seat z -0.8: max |x| or |y| at ±5 deg yaw {worst_low:.1f} vs {OPEN_HALF:.0f} (G)")
print(f"props at seat z {PROP_Z + SEAT_DZ:.0f}, funnel top {FACE_TOP}; motor r {RM:.0f} vs funnel valley r at the arm tip "
      f"{math.sqrt(2) * (OPEN_HALF + 0.8 + arm_bottom(RM) + SEAT_DZ):.0f}")

# mass and CG (E unless the row is tagged)
items = [("L2", 230, lidar), ("front camera", 15, front_cam), ("down camera", 15, down_board), ("Pi 5 + cooler", 55, pi), ("FC + ESC stack", 34, fc),
         ("PDB", 21, pdb), ("UBEC", 15, ubec), ("buck", 6, buck), ("RX", 2, rx), ("pack", 650, pack), ("flow", 2, flow)]
items += [(f"motor+prop {i}", 81 + 10, m) for i, m in enumerate(motors)]
items += [(f"ESC {i}", 25, e) for i, e in enumerate(escs)]
items += [("top plate", 23, top), ("bottom plate", 61, bottom), ("wires", 80, fc)]
printed = {"arms": (trimesh.util.concatenate(arms), 0.45), "mast": (mast, 0.35), "sled + pods": (trimesh.util.concatenate([sled, cam_pod, flow_pod]), 0.3),
           "Pi deck": (deck, 0.5), "feet": (trimesh.util.concatenate(feet), 0.5)}
tot, mom = 0.0, np.zeros(3)
for nm, g, m in items:
    tot += g; mom += g * m.bounding_box.centroid
for nm, (m, fill) in printed.items():
    g = abs(m.volume) * 1.27e-3 * fill; tot += g; mom += g * m.centroid    # PETG at a fill fraction (E)
cg = mom / tot
print(f"mass ~{tot:.0f} g (E; record says 2.0 kg with the stock frame), CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}): "
      f"{PROP_Z - cg[2]:.0f} under the prop plane")
FLOW_POS = (FLOW_LENS - cg) * [1, -1, -1] / 1000
print(f"PX4 flow position from the CG (FRD, m): x {FLOW_POS[0]:+.3f}, y {FLOW_POS[1]:+.3f}, z {FLOW_POS[2]:+.3f}")

# ---------------------------------------------------------------- motion artifact
s = Scene("Overseer layout v0", f"S500 plates, printed arms {WB:.0f}, 45 root, everything between the plates; L2 on a mast, pack centred under, "
          "down camera + flow beside it; rough shapes (E)")
P = lambda n, m, g, c, pr, note="": s.part(n, m, group=g, color=c, printed=pr, note=note)
P("bottom plate", bottom, "frame", "#cfcfcf", False, "S500 stock, 196 x 118 x 2.0 (Holybro STEP)")
P("top plate", top, "frame", "#d8d8d8", False, "S500 stock, 120.5 sq x 1.6")
for i, a in enumerate(arms): P(f"arm {i + 1}", a, "printed", "#8899aa", True, "rough: root 45, taper to 16, wheel leg")
P("L2 mast", mast, "printed", "#9aa8b8", True, "rough: spider on the arm-top screws")
P("cam bracket", cam_bracket, "printed", "#aab4c0", True)
P("Pi deck", union([deck] + posts), "printed", "#b8c0c8", True)
P("belly sled + pods", union([sled, cam_pod, flow_pod]), "printed", "#99a6b3", True, "rough")
P("feet", trimesh.util.concatenate(feet), "printed", "#555555", True, "TPU")
P("catch wheels", trimesh.util.concatenate(wheels), "printed", "#cc6644", True, "8 x R4 at the Scout points")
P("Unitree L2", trimesh.util.concatenate([lidar, dome]), "bought", "#222222", False, "75 x 75 x 65, 230 g")
P("front camera", front_cam, "bought", "#2d5a2d", False, "30 deg up, portrait")
P("down camera", trimesh.util.concatenate([down_board, down_lens]), "bought", "#2d5a2d", False)
P("flow MTF-02P", flow, "bought", "#2d5a2d", False)
P("Pi 5", pi, "bought", "#2e7d32", False)
P("FC stack", fc, "bought", "#1b5e20", False)
P("PDB", pdb, "bought", "#a0522d", False)
P("UBEC + buck + RX", trimesh.util.concatenate([ubec, buck, rx]), "bought", "#444488", False)
P("ESCs", trimesh.util.concatenate(escs), "bought", "#333366", False, "model open, E size")
P("motors", trimesh.util.concatenate(motors), "bought", "#b0b0b0", False, "X2216 II 880 KV")
def prop_view(i, d):
    from topo.part import man
    cx, cy = RM * math.cos(math.radians(d)), RM * math.sin(math.radians(d))
    ring = man(cyl(PROP_R, PROP_Z - 0.5, PROP_Z + 0.5, cx, cy, 96)) - man(cyl(PROP_R - 2.5, PROP_Z - 1, PROP_Z + 1, cx, cy, 96))
    ring = ring.to_mesh(); ring = trimesh.Trimesh(ring.vert_properties[:, :3], ring.tri_verts)
    blade = box(-PROP_R, PROP_R, -9, 9, PROP_Z - 1.5, PROP_Z + 1.5); blade.apply_transform(rotz(30 + 60 * i)); blade.apply_translation([cx, cy, 0])
    return trimesh.util.concatenate([ring, blade])


P("props", trimesh.util.concatenate([prop_view(i, d) for i, d in enumerate(ARM_DEG)]), "bought", "#d9a400", False, "1045: blades + tip circle")
P("4S 6000", pack, "bought", "#2f4fbf", False, "148 x 48 x 45")
names = [p["name"] for p in s.parts] if hasattr(s, "parts") else None
s.timeline(lambda t: {n: rotz(360 * t / 20) for n in names}, duration_s=20, fps=30)
s.view("iso", (560, -620, 520), (0, 0, 30)); s.view("front", (900, 0, 60), (0, 0, 40)); s.view("side", (0, -900, 60), (0, 0, 40))
s.view("top", (1, 0, 1000), (0, 0, 0)); s.view("under", (300, -300, -500), (0, 0, -20))
os.makedirs(OUT, exist_ok=True)
s.write(os.path.join(OUT, "motion.html"))
try:
    s.video(os.path.join(OUT, "motion.mp4"))
except Exception as e:
    print("video skipped:", str(e)[:200])
print("wrote", OUT)
