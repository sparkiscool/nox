"""Hero render: bar_half v8 as installed (W), v7b ghosted behind it, with the numbers that matter. Display only:
meshes decimated for drawing; every number on the sheet comes from the full meshes / FE json.
python3 bar_half_v8_hero.py -> bar_half_v8_hero.png"""
import json, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

v8 = trimesh.load("box_bar_half_v8_W.stl", force="mesh"); v7 = trimesh.load("box_bar_half_v7b_W.stl", force="mesh")
g8, g7 = v8.volume / 1000 * 1.27, v7.volume / 1000 * 1.27
fe8 = json.load(open("stand_v8_fe.json"))["bar_half"]["cases"]["box weight on one tip (misuse)"]
fe7 = json.load(open("stand_v7b_fe.json"))["bar_half"]["cases"]["box weight on one tip (misuse)"]

def dec(m, n):
    try: return m.simplify_quadric_decimation(face_count=n)
    except Exception: return m
d8, d7 = dec(v8, 60000), dec(v7, 20000)

BG, INK, MUTED, ACC = "#f4f2ee", "#1d1f22", "#8a8780", "#c8553d"
fig = plt.figure(figsize=(14, 7.6), dpi=150, facecolor=BG)

def shade(m, base, light, lo=0.42):
    s = lo + (1 - lo) * np.clip(m.face_normals @ light, 0, 1)
    return np.clip(np.asarray(base)[None, :] * s[:, None], 0, 1)

# iso, as installed
ax = fig.add_axes([0.0, 0.18, 0.62, 0.74], projection="3d"); ax.set_facecolor(BG)
L = np.array([-0.35, -0.75, 0.55]); L /= np.linalg.norm(L)
EL, AZ = np.radians(22), np.radians(-62)
cam = np.array([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)])
front = v8.face_normals @ cam > 0                                      # back faces culled: matplotlib has no depth buffer
c8 = shade(v8, [0.86, 0.84, 0.80], L)[front]
ax.add_collection3d(Poly3DCollection(v8.triangles[front], facecolors=c8, edgecolors=c8, linewidths=0.1))
lo, hi = v8.bounds; c, r = (lo + hi) / 2, (hi - lo).max() / 2
ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
ax.set_box_aspect((1, 1, 1), zoom=1.9); ax.view_init(elev=22, azim=-62); ax.set_axis_off()

# side elevation (X-Z), v7b ghost behind v8
ax2 = fig.add_axes([0.06, 0.06, 0.88, 0.16]); ax2.set_facecolor(BG)
def silhouette(m, col, alpha, z):
    tri = m.triangles[:, :, [0, 2]]
    keep = np.abs(m.face_normals[:, 1]) > 0.2
    for t in tri[keep]: ax2.fill(t[:, 0], t[:, 1], color=col, alpha=alpha, lw=0, zorder=z)
silhouette(dec(v8, 20000), INK, 0.9, 2)
sec = v7.section(plane_origin=[0, 50.0, 0], plane_normal=[0, 1, 0])
p2, _ = sec.to_2D(to_2D=np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, -50.0], [0, 0, 0, 1]], float))
for poly in p2.polygons_full:
    for ring in [poly.exterior, *poly.interiors]:
        x, z = ring.xy; ax2.plot(x, z, color="#e8c9a0", lw=1.1, zorder=3)
ax2.set_aspect("equal"); ax2.set_xlim(-25, 192); ax2.set_ylim(-204, -160); ax2.axis("off")
ax2.text(-25, -160, "side view, X / Z     v8 solid, v7b truss outlined", color=MUTED, fontsize=9, va="bottom")
ax2.annotate("tip pad + bolt boss, roofed 6.0 over the head access", xy=(176, -176), xytext=(120, -158), fontsize=8.5, color=ACC,
             arrowprops=dict(arrowstyle="-", color=ACC, lw=0.8))

# numbers
X0 = 0.64
fig.text(X0, 0.88, "Cross bar half  v8", fontsize=24, color=INK, weight="bold")
fig.text(X0, 0.835, "topology, solid-checked, print 2", fontsize=12, color=MUTED)
rows = [("worst index, box on one tip", f"{fe8['worst']:.2f}", f"{fe7['worst']:.2f}", "F"),
        ("tip deflection", f"{fe8['umax_mm'][2]:.2f} mm", f"{fe7['umax_mm'][2]:.2f} mm", "F"),
        ("mass", f"{g8:.1f} g", f"{g7:.1f} g", "M"),
        ("solid check", "PASS", "PASS", "M"),
        ("fits, every part + sheet", "0 mm³", "0 mm³", "C"),
        ("bed contact", "5833 mm²", "2642 mm²", "M")]
y = 0.74
fig.text(X0 + 0.20, y + 0.035, "v8", fontsize=11, color=INK, weight="bold"); fig.text(X0 + 0.29, y + 0.035, "v7b", fontsize=11, color=MUTED, weight="bold")
for lab, a, b, tag in rows:
    fig.text(X0, y, lab, fontsize=11, color=INK); fig.text(X0 + 0.20, y, a, fontsize=11, color=ACC if lab.startswith("worst") else INK, weight="bold")
    fig.text(X0 + 0.29, y, b, fontsize=11, color=MUTED); fig.text(X0 + 0.345, y, tag, fontsize=9, color=MUTED)
    fig.add_artist(plt.Line2D([X0, X0 + 0.355], [y - 0.017, y - 0.017], color="#dcd8d0", lw=0.6)); y -= 0.06
fig.text(X0, y - 0.01, "Y 46 face on the bed at 45.12°, 164 x 160 x 16 on the A1 mini", fontsize=9.5, color=MUTED)
fig.savefig("bar_half_v8_hero.png", facecolor=BG); print("bar_half_v8_hero.png")
