"""Stand v6 assembly render (W): all printed stand parts + ring frames, bay modules at -90/-60 (mesh), stub v3 on the mast."""
import math, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import manifold3d as mf, stand_v6_parts as P
def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
parts = []
NEW, OLD, ARM = (0.11, 0.44, 0.54), (0.69, 0.55, 0.36), (0.6, 0.63, 0.66)
for n in ("mast", "kb", "cradle_keel", "toe", "bar_half"):
    parts.append((trimesh.load(f"box_{n}_v6_W.stl"), NEW))
b = man(trimesh.load("box_bar_half_v6_W.stl")).rotate([0, 0, 180]).translate([0, 108.0, 0]); parts.append((P.tm(b), NEW))
for n, ps in P.POS.items():
    m = man(trimesh.load(f"box_{n}_v6_W.stl"))
    for p in ps: parts.append((P.tm(P.place(m, p)), NEW))
mod = trimesh.load("models/box_baymod_v4i_W.obj", force="mesh")
for pb in P.BAYS:
    t = mod.copy(); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 - pb), [0, 1, 0])); parts.append((t, OLD))
st = trimesh.load("box_stub_v3.stl")                      # print pose: plate face on the bed, z = Y; x, y = X, -Z offsets
c = st.bounds; st.apply_translation([-(c[0][0] + c[1][0]) / 2, -(c[0][1] + c[1][1]) / 2, 0])
st.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])); parts.append((st, ARM))
hub = trimesh.creation.annulus(40, 86, 59, sections=64); hub.apply_translation([0, 0, 37.5]); hub.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [1, 0, 0])); parts.append((hub, ARM))
fig = plt.figure(figsize=(16, 8), dpi=120)
for k, (el, az, ttl) in enumerate([(18, -55, "front-right"), (12, 200, "rear-left")]):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    L = np.array([0.4, -0.6, 0.7]); L /= np.linalg.norm(L)
    for t, col in parts:
        if len(t.faces) > 6000: t = t.simplify_quadric_decimation(face_count=6000) if hasattr(t, "simplify_quadric_decimation") else t
        tri = t.triangles[:, :, [0, 1, 2]]
        sh = 0.45 + 0.55 * np.clip(t.face_normals @ L, 0, 1)
        fc = np.clip(np.array(col)[None, :] * sh[:, None] * 1.25, 0, 1)
        pc = Poly3DCollection(tri[:, :, [0, 1, 2]], facecolors=fc, edgecolors="none"); ax.add_collection3d(pc)
    ax.set_xlim(-230, 230); ax.set_ylim(-140, 280); ax.set_zlim(-205, 215); ax.set_box_aspect((460, 420, 420))
    ax.view_init(elev=el, azim=az); ax.set_axis_off(); ax.text2D(0.05, 0.9, ttl, transform=ax.transAxes, fontsize=11)
fig.suptitle("Nox box stand v6, all printed: teal = new prints, tan = bay modules v4i, grey = stub v3 + hub envelope (funnel not shown)", fontsize=11)
fig.subplots_adjust(0, 0, 1, 0.95, 0, 0); fig.savefig("stand_v6_assembly.png"); print("ok")
