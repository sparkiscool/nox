"""Assembly picture of S2 cage v2 (drone frame): printed parts, rods, prop disks and plate for context."""
import json, sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cage_cad as cc
OUT = cc.OUT
def L(n, target=5000):
    m = trimesh.load(f"{OUT}/s2_cage_v2_{n}.stl")
    if len(m.faces) > target:
        try: m = m.simplify_quadric_decimation(face_count=target)
        except Exception: pass
    return m
def rot(k): T = np.eye(4); T[:3, :3] = cc.face_rot(k); return T
parts = []
PETG, TPU, ROD, WHEEL = "#d9cdb4", "#5a6470", "#1d1d1f", "#c9a227"
for k in range(4):
    m = L("corner_node"); m.apply_transform(rot(k)); parts.append((m, PETG))
for k, f in ((0, "tee_node"), (1, "tee_node_tail"), (2, "tee_node")):
    m = L(f); m.apply_transform(rot(k)); parts.append((m, TPU))
for sy in (1, -1):
    m = L("side_hub"); m.apply_transform(np.diag([1, sy, 1, 1.0])); parts.append((m, PETG))
parts.append((L("tail_hub"), PETG))
for sx in (1, -1):
    for sy in (1, -1):
        m = L("foot_node"); m.apply_transform(np.diag([sx, sy, 1, 1.0])); parts.append((m, TPU))
w0 = L("wheel")
for n, a, b, r in cc.pins():
    a, b = np.array(a), np.array(b); c = (a + b) / 2; d = (b - a) / np.linalg.norm(b - a)
    w = w0.copy(); w.apply_translation([0, 0, -cc.WW / 2]); T = trimesh.geometry.align_vectors([0, 0, 1], d); T[:3, 3] = c; w.apply_transform(T); parts.append((w, WHEEL))
lay = json.load(open(f"{OUT}/cage_layout.json"))
for n, a, b, r in lay["rods"]:
    a, b = np.array(a), np.array(b); d = b - a; Ln = np.linalg.norm(d)
    cy = trimesh.creation.cylinder(radius=r, height=Ln, sections=12)
    T = trimesh.geometry.align_vectors([0, 0, 1], d / Ln); T[:3, 3] = (a + b) / 2; cy.apply_transform(T); parts.append((cy, ROD))
def draw(ax, elev, azim, title, ctx=True):
    for m, col in parts:
        tri = m.vertices[m.faces]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        e, a_ = np.radians(elev), np.radians(azim); light = np.array([np.cos(e) * np.cos(a_), np.cos(e) * np.sin(a_), np.sin(e)]) * 0.6 + np.array([0.3, -0.3, 0.7])
        light /= np.linalg.norm(light); sh = 0.45 + 0.55 * np.clip(np.abs(n @ light), 0, 1)
        base = np.array(matplotlib.colors.to_rgb(col)); fc = np.clip(base[None] * sh[:, None] + (0.08 if col == ROD else 0), 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, edgecolors="none"))
    if ctx:
        t = np.linspace(0, 2 * np.pi, 80)
        for sx in (1, -1):
            for sy in (1, -1):
                ax.plot(sx * 54.1 + 44.45 * np.cos(t), sy * 54.1 + 44.45 * np.sin(t), cc.PZ, color="#6aa9d8", lw=1.0, alpha=0.8)
        X, Y = np.meshgrid([-50, 50], [-47, 47]); ax.plot_surface(X, Y, np.full_like(X, 22.0), color="#555555", alpha=0.25)
    ax.set_xlim(-118, 118); ax.set_ylim(-118, 118); ax.set_zlim(-50, 60); ax.set_box_aspect((236, 236, 110), zoom=1.02)
    ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=11, color="#333")
fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#f4f2ee")
draw(fig.add_subplot(1, 2, 1, projection="3d", facecolor="#f4f2ee"), 24, -55, "iso (nose to the lower right)")
draw(fig.add_subplot(1, 2, 2, projection="3d", facecolor="#f4f2ee"), 0, -90, "side (from the right, -y)")
chk = json.load(open(f"{OUT}/cage_check.json")) if len(sys.argv) > 2 else None
fig.suptitle("S2 cage v2, props on top (prop plane z 41.2, blue)  |  PETG nodes and hubs (sand), TPU tees and feet (slate), carbon rods (black), wheels (gold)",
             fontsize=11, x=0.02, ha="left", color="#222")
fig.subplots_adjust(left=0, right=1, bottom=0, top=0.93, wspace=0)
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else "/home/claude/cage/look/cage_assembly.png")
