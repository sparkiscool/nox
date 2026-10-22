"""Scout v2.1 cage v12 assembled CAD -> scout_v21.json (glTF, embedded buffer) + clearance check of every part against the
hard keep-outs, measured on the real meshes. Drone coords (x nose, y left, z up, mm).
Node parts from engine/v12/<name>.stl (local coords, node point at origin; node_topo2.py). Tags: C drawn, M mesh masses."""
import json, math, os, base64, numpy as np, trimesh
from trimesh.transformations import rotation_matrix
D = os.path.dirname(os.path.abspath(__file__)); ENG = os.path.join(D, "..", "engine", "v12")
TIP, ROD_R = 4.0, 1.5
L = json.load(open(os.path.join(D, "s21_lantern12.json")))
HUB_PTS = [(40, 0, 85), (28.28, 28.28, 85), (0, 40, 85), (-28.28, 28.28, 85), (-40, 0, 85), (-28.28, -28.28, 85), (0, -40, 85), (28.28, -28.28, 85)]
NODE_PTS = [tuple(p) for k, v in L["nodes"].items() for p in v if k != "hub"]
scene = trimesh.Scene(); NAMES = {}; PARTS = []        # PARTS: (name, mesh) for the clearance check
def add(mesh, name, check=True):
    NAMES[name] = NAMES.get(name, 0) + 1
    if NAMES[name] > 1: name = f"{name}_{NAMES[name]}"
    scene.add_geometry(mesh, node_name=name, geom_name=name)
    if check: PARTS.append((name, mesh))
def cyl(a, b, r, n=18): return trimesh.creation.cylinder(radius=r, segment=[a, b], sections=n)
def box(x0, x1, y0, y1, z0, z1):
    m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return m
def near(p, pts, tol=0.6): return any(math.dist(p, q) < tol for q in pts)

# ---- rods. Ends on a node point stop TIP short (the socket bottom); ring ends and hub ring points are socket bottoms already
rod_mm = 0
for i, (a, b, tag) in enumerate(L["rods"]):
    a = np.array(a, float); b = np.array(b, float); d = (b - a) / np.linalg.norm(b - a)
    A = a + d * TIP if near(a, NODE_PTS) else a; B = b - d * TIP if near(b, NODE_PTS) else b
    rod_mm += np.linalg.norm(B - A); add(cyl(A, B, ROD_R), f"rod:{tag}:{i}")

# ---- TPU sleeves on both rings, between the sockets: from 11 off a tee sleeve to 11 off the corner socket bottom (mouth 10 + 1)
for (a, b, tag) in L["rods"]:
    if tag not in ("skid ring", "shoulder ring"): continue
    a = np.array(a, float); b = np.array(b, float); d = (b - a) / np.linalg.norm(b - a); mid = (a + b) / 2
    nose_skid = tag == "skid ring" and abs(a[0] - 112) < .1 and abs(b[0] - 112) < .1
    t0 = 21.0 if nose_skid else 11.0                   # nose skid flat: split sleeves at y +-14 (8..20), camera window between
    for s in (1, -1):
        p = mid + d * s * t0; q = (b - d * 11) if s > 0 else (a + d * 11)
        add(cyl(p, q, 2.0, 16), f"tpu:sleeve:{tag}:{i}:{s}")

# ---- UHMW skid strips 76 x 5 x 2.5 on the skid ring flats, centred +-50 (x / y 12..88): clear of the tee sleeves and corner
#      sockets (v11's 90 long strips at +-55 ran into the corner sockets). Outer face 115.5 = on the collar face at z 30.7
for sgn in (1, -1):
    for c in (50, -50):
        x0, x1 = (113.0, 115.5) if sgn > 0 else (-115.5, -113.0)
        add(box(x0, x1, c - 38, c + 38, 30.7, 35.7), f"strip:x{sgn}:{c}", check=False)
        add(box(c - 38, c + 38, x0, x1, 30.7, 35.7), f"strip:y{sgn}:{c}", check=False)

# ---- generated parts
my = np.diag([1, -1, 1, 1.0]); I4 = np.eye(4)
PLACE = {"corner_col": [((106, 106, 33.2), I4), ((106, -106, 33.2), my)], "corner_col_t": [((-106, 106, 33.2), I4), ((-106, -106, 33.2), my)],
         "side_col": [((0, 112, 33.2), I4), ((0, -112, 33.2), my)], "nose_col": [((112, 0, 33.2), I4)], "tail_col": [((-112, 0, 33.2), I4)],
         "hub": [((0, 0, 85), I4)], "nose_leg": [((37.5, 32, -7), I4), ((37.5, -32, -7), my)],
         "knee_tail": [((-66, 52, -0.8), I4), ((-66, -52, -0.8), my)], "sled_tail": [((-52, 32, -47), I4), ((-52, -32, -47), my)],
         "rail_sleeve": [((-5, 32, -47), I4), ((-5, -32, -47), my)]}
rows = []; node_g = 0.0; missing = []
for typ, places in PLACE.items():
    p = os.path.join(ENG, f"{typ}.stl")
    if not os.path.exists(p): missing.append(typ); continue
    m = trimesh.load(p, force="mesh"); j = json.load(open(os.path.join(ENG, f"{typ}.json")))
    for k, (pos, T) in enumerate(places):
        mm = m.copy(); mm.apply_transform(T); mm.apply_translation(pos); add(mm, f"node:{typ}:{k}")
    node_g += j["mass_g"] * len(places); rows.append((typ, len(places), j["mass_g"], j["worst"], j["gen_cm3"]))
for y in (50, -50):                                    # v2 side hubs (M 1.86 g) + TPU grommet
    add(cyl([2, y - 4, 24], [2, y + 4, 24], 5), f"node:sidehub:{y}", check=False)
    add(trimesh.creation.torus(5.5, 1.5).apply_transform(rotation_matrix(math.pi / 2, [1, 0, 0])).apply_translation([2, y, 24]), f"tpu:grommet:side{y}", check=False)

# ---- airframe, electronics, cartridge, receiver (C; not checked against themselves)
add(box(-50, 50, -47, 47, 18.5, 22), "frame:plate", False)
for (x, y) in [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]:
    a = np.array([x * 0.55, y * 0.55, 20.25]); b = np.array([x, y, 20.25]); Lh = 0.5 * math.hypot(x - a[0], y - a[1])
    arm = box(-Lh, Lh, -7, 7, -1.75, 1.75); arm.apply_transform(rotation_matrix(math.atan2(y - a[1], x - a[0]), [0, 0, 1])); arm.apply_translation((a + b) / 2)
    add(arm, f"frame:arm:{x}{y}", False); add(cyl([x, y, 22], [x, y, 36], 13, 24), f"frame:motor:{x}{y}", False)
    add(cyl([x, y, 36], [x, y, 38], 3.5, 12), f"frame:shaft:{x}{y}", False)
    add(trimesh.creation.annulus(43.5, 44.45, 1.0, sections=64).apply_translation([x, y, 41.2]), f"prop:ring:{x}{y}", False)
    add(cyl([x, y, 40.7], [x, y, 41.7], 44.45, 64), f"prop:disk:{x}{y}", False)
fc = box(-18, 18, -18, 18, 26, 34); fc.apply_transform(rotation_matrix(math.pi / 4, [0, 0, 1])); fc.apply_translation([1.9, 0, 0]); add(fc, "fc:board", False)
add(box(-32.5, 32.5, -15, 15, 10.4, 12.0), "radxa:pcb", False); add(box(-20, 20, 5, 15, 12.0, 15.2), "radxa:usb", False)
add(box(-15, 15, -40, -24, 8, 18), "ubec:box", False); add(box(-32, -17, -37.5, -26.5, 8, 11), "ubec:rx", False)
add(box(99, 100.6, -19, 19, 22, 60), "cam:board", False); add(cyl([100.6, 0, 41], [114, 0, 41], 6, 20), "cam:lens", False)
add(box(63, 85, -10, 6, 2, 10), "flow:board", False)
for y in (21.5, -21.5): add(box(-37, -21, y - 1.5, y + 1.5, -5, -1), f"recv:pins:{y}", False)
add(box(-41.5, 37.5, -19, 19, -44.5, 3.5), "pack:cartridge", False)
add(box(-45, 41, -23, 23, 3.5, 7.0), "recv:ceiling", False)
for s in (1, -1):
    add(box(-45, 41, s * 21 - 1, s * 21 + 1, -44.5, 7.0) if s > 0 else box(-45, 41, -23, -21, -44.5, 7.0), f"recv:wall:{s}", False)
    for x in (15, -25):                                  # receiver v4 outrigger sleeves on the sled rails (DEFAULT x 15 / -25)
        add(box(x - 5, x + 5, min(s * 22, s * 29), max(s * 22, s * 29), -44, -40), f"recv:lug:{x}{s}", False)
        add(cyl([x - 5, s * 32, -47], [x + 5, s * 32, -47], 4.0, 20), f"recv:sleeve:{x}{s}", False)

# ---- clearance check on the real meshes (every rod, sleeve and node part)
c5, s5 = math.cos(math.radians(5)), math.sin(math.radians(5))
def clear(V):
    x, y, z = V[:, 0], V[:, 1], V[:, 2]; out = {}
    up = z > -0.8
    out["collar face"] = float(((84 + z + 0.8) - np.maximum(abs(x), abs(y)))[up].min()) if up.any() else 99
    lim = 84 + np.maximum(z + 0.8, 0)                     # 168 opening below the rim, the 45 deg face above it; drone yawed +-5 deg
    w = np.minimum(lim - np.maximum(abs(x * c5 - y * s5), abs(x * s5 + y * c5)), lim - np.maximum(abs(x * c5 + y * s5), abs(-x * s5 + y * c5)))
    rb = z <= 5.0                                           # the opening and the 6 mm rim band; above it the drum strips clock the yaw
    if rb.any(): out["opening + rim, yawed 5"] = float(w[rb].min())
    pr = 99
    for mx, my_ in [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]:
        dpl = np.hypot(x - mx, y - my_) - 44.45; dz = np.where((z >= 37.2) & (z <= 45.2), 0, np.minimum(abs(z - 37.2), abs(z - 45.2)))
        pr = min(pr, float(np.where(dpl < 0, dz, np.hypot(np.maximum(dpl, 0), dz)).min()))
    out["props"] = pr
    arm = (z < -9.5); out["arm sweep (x <= 42)"] = float((42 - x[arm]).min()) if arm.any() else 99
    inx = (x >= 100.6) & (x <= 114.0)                       # camera lens (100.6..114, 0, 41) r 6 (lens E, D18)
    if inx.any(): out["camera lens"] = float((np.hypot(y[inx], z[inx] - 41) - 6.0).min())
    hl = (z < -0.8) & (x > -47) & (x < 39)
    if hl.any():
        yy = y[hl]; d = np.where(yy >= 14, yy - 14, np.where(yy <= -26, -26 - yy, -np.minimum(yy + 26, 14 - yy)))
        out["head lane (y outside -26..14)"] = float(d.min())
    return out
worst = {}
for name, m in PARTS:
    V = m.sample(4000) if len(m.vertices) < 400 else np.asarray(m.vertices)
    V = np.vstack([V, np.asarray(m.vertices)])
    for k, v in clear(V).items():
        if k not in worst or v < worst[k][0]: worst[k] = (v, name)

out_gltf = trimesh.exchange.gltf.export_gltf(scene, merge_buffers=True)
g = json.loads(out_gltf["model.gltf"]); binname = [k for k in out_gltf if k.endswith(".bin")][0]
for b in g["buffers"]: b["uri"] = "data:application/octet-stream;base64," + base64.b64encode(out_gltf[binname]).decode()
open(os.path.join(D, "scout_v21.json"), "w").write(json.dumps(g))
faces = sum(len(m.faces) for m in scene.geometry.values())
print(f"glTF {os.path.getsize(os.path.join(D, 'scout_v21.json')) / 1e6:.1f} MB, {len(scene.geometry)} parts, {faces} faces; rods {rod_mm / 1000:.2f} m; generated parts {node_g:.1f} g M; missing {missing}")
for r in rows: print("  ", r)
print("clearances (mm, worst part):"); [print(f"   {k:30s} {v:6.2f}  {n}") for k, (v, n) in worst.items()]
json.dump(dict(rod_m=round(rod_mm / 1000, 3), node_g=round(node_g, 1), missing=missing, nodes=rows, clear={k: [round(v, 2), n] for k, (v, n) in worst.items()}),
          open(os.path.join(D, "assembly12.json"), "w"), indent=1)
