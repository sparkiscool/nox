"""Scout v2.1 assembled CAD: rods + generated nodes + sleeves/strips/grommets + receiver + core + electronics -> GLB.
Drone coords (x nose, y left, z up, mm). Node STLs come from engine/node_topo.py (local coords, node point at origin).
Tags: C for drawn geometry, M for generated node masses (from the node json)."""
import json, math, os, sys, numpy as np, trimesh
from trimesh.transformations import rotation_matrix, translation_matrix, scale_matrix
ENG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "engine")
TIP, ROD_R = 4.0, 1.5
L = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "s21_lantern.json")))
NODE_PTS = {tuple(p) for v in L["nodes"].values() for p in v} | {(0, 0, 85)}
HUB_PTS = [(40, 0, 85), (28.2842712474619, 28.2842712474619, 85), (0, 40, 85), (-28.2842712474619, 28.2842712474619, 85),
           (-40, 0, 85), (-28.2842712474619, -28.2842712474619, 85), (0, -40, 85), (28.2842712474619, -28.2842712474619, 85)]
scene = trimesh.Scene()
NAMES = {}
def add(mesh, name):
    NAMES[name] = NAMES.get(name, 0) + 1
    if NAMES[name] > 1: name = f"{name}_{NAMES[name]}"
    scene.add_geometry(mesh, node_name=name, geom_name=name)
def cyl(a, b, r, n=18): return trimesh.creation.cylinder(radius=r, segment=[a, b], sections=n)
def box(x0, x1, y0, y1, z0, z1):
    m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return m
def near(p, q, tol=0.6): return math.dist(p, q) < tol

# ---- rods (3.0 carbon). Ends on a node point stop TIP short (inside the socket); ring ends are already at the socket mouths.
rod_mm = 0
for i, (a, b, tag) in enumerate(L["rods"]):
    a = np.array(a, float); b = np.array(b, float); d = (b - a) / np.linalg.norm(b - a)
    sa = any(near(a, p) for p in NODE_PTS) or any(near(a, p) for p in HUB_PTS)
    sb = any(near(b, p) for p in NODE_PTS) or any(near(b, p) for p in HUB_PTS)
    if tag == "camera rail": sa = sb = False
    A = a + d * TIP if sa else a; B = b - d * TIP if sb else b
    rod_mm += np.linalg.norm(B - A)
    add(cyl(A, B, ROD_R), f"rod:{tag}:{i}")

# ---- TPU sleeves on both rings (OD 5.0, between the nodes: flats 200 less 2 x 12 under the tee/elbow sleeve) C
for z in (33.2, 60.0):
    for (a, b, tag) in L["rods"]:
        if tag in ("skid ring", "shoulder ring") and abs(a[2] - z) < .01:
            a = np.array(a, float); b = np.array(b, float); d = (b - a) / np.linalg.norm(b - a); mid = (a + b) / 2
            for s in (1, -1):   # two halves, each from 12 off the tee to the ring end
                p = mid + d * s * 12; q = (a if s < 0 else b)
                add(cyl(p, q, 2.5, 16), f"tpu:sleeve:{z}:{s}")

# ---- UHMW skid strips 90 x 5 x 2.5 on the skid ring flats either side of each tee, outer face 1.0 inside the collar face C
for sgn in (1, -1):
    for c in (55, -55):
        add(box(sgn * 113.0, sgn * 115.5, c - 45, c + 45, 30.7, 35.7) if sgn > 0 else box(-115.5, -113.0, c - 45, c + 45, 30.7, 35.7), f"strip:x{sgn}:{c}")
        add(box(c - 45, c + 45, 113.0, 115.5, 30.7, 35.7) if sgn > 0 else box(c - 45, c + 45, -115.5, -113.0, 30.7, 35.7), f"strip:y{sgn}:{c}")

# ---- generated nodes (M) placed by mirror transforms of the generated quadrant
def load_node(name):
    p = os.path.join(ENG, f"{name}.stl")
    if not os.path.exists(p): return None, None
    m = trimesh.load(p, force="mesh"); j = json.load(open(os.path.join(ENG, f"{name}.json"))); return m, j
mirror_x = np.diag([-1, 1, 1, 1.0]); mirror_y = np.diag([1, -1, 1, 1.0]); rot180 = np.diag([-1, -1, 1, 1.0]); I4 = np.eye(4)
PLACE = {  # node type -> list of (global position, transform of the local mesh)
    "corner":       [((106, 106, 33.2), I4), ((106, -106, 33.2), mirror_y)],
    "corner_tail":  [((-106, 106, 33.2), I4), ((-106, -106, 33.2), mirror_y)],
    "side_tee":     [((0, 112, 33.2), I4), ((0, -112, 33.2), mirror_y)],
    "axis_tee":     [((112, 0, 33.2), I4), ((-112, 0, 33.2), mirror_x)],
    "elbow_corner": [((106, 106, 60), I4), ((-106, 106, 60), mirror_x), ((-106, -106, 60), rot180), ((106, -106, 60), mirror_y)],
    "elbow_tee":    [((112, 0, 60), I4), ((-112, 0, 60), mirror_x)],
    "elbow_tee_y":  [((0, 112, 60), None), ((0, -112, 60), None)],      # elbow_tee rotated 90 deg about z
    "hub":          [((0, 0, 85), I4)],
    "knee_nose":    [((40, 32, -7), I4), ((40, -32, -7), mirror_y)],
    "knee_tail":    [((-75, 64, -0.8), I4), ((-75, -64, -0.8), mirror_y)],
    "sled_nose":    [((40, 32, -47), I4), ((40, -32, -47), mirror_y)],
    "sled_tail":    [((-50, 32, -47), I4), ((-50, -32, -47), mirror_y)],
    "rail_sleeve":  [((-5, 32, -47), I4), ((-5, -32, -47), mirror_y)],
}
node_mass = 0.0; missing = []; node_rows = []
for typ, places in PLACE.items():
    src = "elbow_tee" if typ == "elbow_tee_y" else typ
    m, j = load_node(src)
    if m is None: missing.append(typ); continue
    for k, (pos, T) in enumerate(places):
        mm = m.copy()
        if typ == "elbow_tee_y":
            T = rotation_matrix(math.radians(90), [0, 0, 1]) if pos[1] > 0 else rotation_matrix(math.radians(-90), [0, 0, 1])
        mm.apply_transform(T); mm.apply_translation(pos)
        add(mm, f"node:{typ}:{k}"); node_mass += j["mass_g"]
    node_rows.append((typ, len(places), j["mass_g"], j["worst"], j.get("gen_cm3"), j["wrinkle"]))

# ---- camera rail clips on the shoulder ring (112, +-14, 60) and camera clips at z 24 (drawn, PA12-CF) C
for y in (14, -14):
    add(cyl([112, y - 6, 60], [112, y + 6, 60], 3.25, 16), f"node:railclip:{y}")
    add(box(105, 114, y - 3.5, y + 3.5, 55, 60), f"node:railclip:b{y}")
    add(box(100, 108, y - 3, y + 3, 24, 30), f"node:camclip:{y}")
# side hubs (v2 part, M 1.86 g) with TPU grommets
for y in (50, -50):
    add(cyl([2, y - 4, 24], [2, y + 4, 24], 5), f"node:sidehub:{y}")
    add(trimesh.creation.torus(5.5, 1.5).apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0])).apply_translation([2, y, 24]), f"tpu:grommet:side{y}")

# ---- airframe: plate, arms, motors, props (C)
add(box(-50, 50, -47, 47, 18.5, 22), "frame:plate")
for (x, y) in [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]:
    a = np.array([x * 0.55, y * 0.55, 20.25]); b = np.array([x, y, 20.25]); arm = box(-0.5 * math.hypot(x - a[0], y - a[1]), 0.5 * math.hypot(x - a[0], y - a[1]), -7, 7, -1.75, 1.75)
    arm.apply_transform(rotation_matrix(math.atan2(y - a[1], x - a[0]), [0, 0, 1])); arm.apply_translation((a + b) / 2); add(arm, f"frame:arm:{x}{y}")
    add(cyl([x, y, 22], [x, y, 36], 13, 24), f"frame:motor:{x}{y}")
    add(cyl([x, y, 36], [x, y, 38], 3.5, 12), f"frame:shaft:{x}{y}")
    add(trimesh.creation.annulus(43.5, 44.45, 1.0, sections=64).apply_translation([x, y, 41.2]), f"prop:ring:{x}{y}")
    add(cyl([x, y, 40.7], [x, y, 41.7], 44.45, 64), f"prop:disk:{x}{y}")

# ---- electronics (C from the sweep)
fc = box(-18, 18, -18, 18, 26, 34); fc.apply_transform(rotation_matrix(math.pi / 4, [0, 0, 1])); fc.apply_translation([1.9, 0, 0]); add(fc, "fc:board")
add(box(-32.5, 32.5, -15, 15, 10.4, 12.0), "radxa:pcb"); add(box(-20, 20, 5, 15, 12.0, 15.2), "radxa:usb")
add(box(-15, 15, -40, -24, 8, 18), "ubec:box"); add(box(-32, -17, -37.5, -26.5, 8, 11), "ubec:rx")
add(box(99, 100.6, -19, 19, 22, 60), "cam:board"); add(cyl([100.6, 0, 41], [114, 0, 41], 6, 20), "cam:lens")
add(box(63, 85, -10, 6, 2, 10), "flow:board")
# pins at the receiver sides
for y in (21.5, -21.5):
    add(box(-37, -21, y - 1.5, y + 1.5, -5, -1), f"recv:pins:{y}")

# ---- cartridge v3 and receiver v4 (simplified shell, C) with 4 TPU-grommeted sled sleeves
add(box(-41.5, 37.5, -19, 19, -44.5, 3.5), "pack:cartridge")
add(box(-45, 41, -23, 23, 3.5, 7.0), "recv:ceiling")
for s in (1, -1):
    add(box(-45, 41, s * 21, s * 23, -44.5, 7.0), f"recv:wall:{s}")
    for x in (40, -5):
        add(box(x - 6, x + 6, min(s * 23, s * 36), max(s * 23, s * 36), -41, -36), f"recv:lug:{x}{s}")
        add(cyl([x, s * 32, -45], [x, s * 32, -36], 5.0, 20), f"recv:sleeve:{x}{s}")
        add(trimesh.creation.torus(4.0, 1.6).apply_translation([x, s * 32, -44]), f"tpu:grommet:{x}{s}")

glb = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scout_v21.glb")
scene.export(glb)
tot_faces = sum(len(g.faces) for g in scene.geometry.values())
print(f"GLB {os.path.getsize(glb)/1e6:.2f} MB, {len(scene.geometry)} parts, {tot_faces} faces; rods {rod_mm/1000:.2f} m; nodes placed {node_mass:.1f} g (M); missing {missing}")
for r in node_rows: print("  ", r)
json.dump(dict(rod_m=round(rod_mm / 1000, 3), node_g=round(node_mass, 1), missing=missing, nodes=node_rows), open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "assembly.json"), "w"), indent=1)
