"""Cage v3 (layout C3) status picture: carbon rods from the frame model (F), generated nodes that exist (M), pending nodes as
their frame stand-ins (ghost). python3 status_render.py out.png"""
import sys, os, json, numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(HERE))
import layout_v3 as L, s2_guard_frame as sg
N = "/home/claude/cage/nodes"
o = json.load(open(f"{HERE}/layout_C3.json"))
P = sg.params_v2(PZ_=L.PZ, upper=("solid", o["upper"])); P.update(opt=o, spoke=("solid", 2.5), skirt=("solid", o["skirt_d"])); g = L.Guard3(P)
BG, PETG, TPU, ROD, SKIRT, SPOKE, GHOST = "#f4f2ee", "#d6c7a5", "#56677a", "#232427", "#2e7d4f", "#b0412e", "#e3a25b"
def dec(m, n=6000):
    if len(m.faces) > n:
        try: m = m.simplify_quadric_decimation(face_count=n)
        except Exception: pass
    return m
def Rz(k):
    T = np.eye(4); c, s = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2); T[:2, :2] = [[c, -s], [s, c]]; return T
parts = []
corner = dec(trimesh.load(sys.argv[2] if len(sys.argv) > 2 else f"{N}/corner_node.stl"))
for k in range(4):
    m = corner.copy(); m.apply_transform(Rz(k)); parts.append((m, PETG))
vt = dec(trimesh.load(f"{N}/vtee.stl"))
for k in (0, 2):
    m = vt.copy(); m.apply_transform(Rz(k)); parts.append((m, PETG))
have_side, have_crown = os.path.exists(f"{N}/side_node.stl"), os.path.exists(f"{N}/crown.stl")
if have_side:
    sn = dec(trimesh.load(f"{N}/side_node.stl"))
    for sy in (1, -1):
        m = sn.copy(); m.apply_transform(np.diag([1, sy, 1, 1.0])); parts.append((m, PETG))
if have_crown: parts.append((dec(trimesh.load(f"{N}/crown.stl"), 9000), PETG))
col = {"lower": ROD, "upper": ROD, "spoke": SPOKE, "skirt": SKIRT}
for e in g.elems:
    a, b = g.X[e["n"][0]], g.X[e["n"][1]]; tg = e["tag"]
    if tg in col:
        r = 1.25 if tg != "upper" else o["upper"] / 2
        d = b - a; Ln = np.linalg.norm(d)
        if Ln < 1e-6: continue
        cy = trimesh.creation.cylinder(radius=r, height=Ln, sections=10); T = trimesh.geometry.align_vectors([0, 0, 1], d / Ln); T[:3, 3] = (a + b) / 2
        cy.apply_transform(T); parts.append((cy, col[tg]))
    elif (tg == "crown" and not have_crown) or (not have_side and tg not in ("case", "rcv") and e["mat"] != "C" and abs(a[0]) < 20 and abs(a[1]) > 95):
        d = b - a; Ln = np.linalg.norm(d)
        if Ln < 1e-6: continue
        cy = trimesh.creation.cylinder(radius=2.2, height=Ln, sections=8); T = trimesh.geometry.align_vectors([0, 0, 1], d / Ln); T[:3, 3] = (a + b) / 2
        cy.apply_transform(T); parts.append((cy, GHOST))
def box(lo, hi, c):
    m = trimesh.creation.box(extents=np.subtract(hi, lo)); m.apply_translation((np.add(lo, hi)) / 2); return (m, c)
ctx = [box((-50, -47, 18.5), (50, 47, 22), "#5b5f66"), box((-15, -33, 22), (15, 33, L.BOSS_TOP), "#9aa5b1"), box(*L.RCV_BOX, "#cbbd96")]
def draw(ax, elev, azim, lim=None, title=""):
    e_, a_ = np.radians(elev), np.radians(azim)
    light = np.array([np.cos(e_) * np.cos(a_), np.cos(e_) * np.sin(a_), np.sin(e_)]) * 0.6 + np.array([0.3, -0.3, 0.8]); light /= np.linalg.norm(light)
    for (m, c), al in [(p, 0.22) for p in ctx] + [(p, 1.0) for p in parts]:
        tri = m.vertices[m.faces]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        sh = 0.42 + 0.58 * np.clip(np.abs(n @ light), 0, 1); base = np.array(matplotlib.colors.to_rgb(c))
        fc = np.clip(base[None] * sh[:, None] + (0.10 if c == ROD else 0), 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, edgecolors="none", alpha=al if c != GHOST else 0.55))
    t = np.linspace(0, 2 * np.pi, 90)
    for mx, my in sg.MOTORS:
        ax.add_collection3d(Poly3DCollection([np.c_[mx + 44.45 * np.cos(t), my + 44.45 * np.sin(t), np.full_like(t, L.PZ)]],
                                             facecolors="#9cc7ea", edgecolors="#4a90c8", alpha=0.18, linewidths=0.6))
    if lim is None: lim = ((-118, 118), (-118, 118), (-15, 82))
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2]); ax.set_box_aspect([h - l for l, h in lim], zoom=1.15 if lim[0][1] - lim[0][0] > 100 else 0.85)
    ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=12, color="#333", pad=-6)
fig = plt.figure(figsize=(20, 11), dpi=110, facecolor=BG)
draw(fig.add_axes((0.0, 0.10, 0.50, 0.80), projection="3d", facecolor=BG), 24, -55, title="iso, nose to the lower right")
draw(fig.add_axes((0.47, 0.42, 0.34, 0.52), projection="3d", facecolor=BG), 0, -90, title="side, from the right")
draw(fig.add_axes((0.47, 0.0, 0.30, 0.46), projection="3d", facecolor=BG), 28, -35, lim=((66, 128), (66, 128), (20, 60)), title="corner node close-up (+x +y)")
txt = ["PARTS (all PETG)", "corner node x4", "  4.42 g, index 1.13 F (noted)", "vtee x2", "  2.76 g, index 0.08 F", "side node L + R", "  4.73 g, index 0.28 F",
       "crown x1", "  7.89 g, index 0.81 F", "wheels x8, 0.16 g", "", "plate 41.9 g, rods 18.7 g", "total about 60.5 g (M)", "",
       "CARBON RODS", "rings 2.5 lower, %.1f upper" % o["upper"], "spokes 2.5 to crown (red)", "skirt 2.5 to receiver (green)"]
fig.text(0.80, 0.86, "\n".join(txt), fontsize=11.5, va="top", color="#222", family="monospace", linespacing=1.55)
fig.suptitle("S2 cage v3 (C3), props on top at z 41.2   |   print set, 17 Oct 20:00", fontsize=15, x=0.02, y=0.975, ha="left", color="#1d1f1c", weight="bold")
fig.savefig(sys.argv[1], facecolor=BG)
