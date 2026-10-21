"""Section figure for belt clamp v2: python belt_clamp_v2_figure.py -> out/belt_clamp_v2_section.png"""
import numpy as np, manifold3d as mf, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
import belt_clamp_v2 as B

INK, MUTE, BG = "#1f2328", "#6b665c", "#f6f5f1"
PART, PART_E, BELT, PIN_C, RED = "#d9d6cf", "#8f8a80", "#2b2f36", "#9aa0a7", "#c0392b"
SLOT = 2.0
c = B.coupon(SLOT)
zc = B.WALL_Z + B.CH_W / 2
fig, axes = plt.subplots(1, 2, figsize=(12.5, 7.2), dpi=170, gridspec_kw=dict(width_ratios=[1.15, 1]))
fig.patch.set_facecolor(BG)
for ax in axes: ax.set_facecolor(BG); ax.set_aspect("equal"); ax.axis("off")

# --- left: section at the belt centre, loaded ---
ax = axes[0]
for poly in c.slice(zc).to_polygons():
    p = np.asarray(poly); ax.add_patch(Polygon(p, closed=True, fc=PART, ec=PART_E, lw=0.8, joinstyle="round"))
# belt centre-line path: leg A up the -x half of the slot, half turn round the pin, leg B down the +x half
r_c = B.PIN_D / 2 + B.TOTAL / 2          # wrap centre-line radius
yc = B.POCKET_YC; xa, xb = -SLOT / 4, SLOT / 4
def leg(x0, x1, y0, y1, n=60):
    t = np.linspace(0, 1, n); s = 3 * t ** 2 - 2 * t ** 3
    return np.c_[x0 + (x1 - x0) * s, y0 + (y1 - y0) * t]
th = np.linspace(np.pi, 0, 80)
path = np.vstack([np.c_[np.full(40, xa), np.linspace(-14, B.SLOT_L, 40)],
                  leg(xa, -r_c, B.SLOT_L, yc)[1:],
                  np.c_[r_c * np.cos(th), yc + r_c * np.sin(th)][1:],
                  leg(r_c, xb, yc, B.SLOT_L)[1:],
                  np.c_[np.full(40, xb), np.linspace(B.SLOT_L, -9, 40)][1:]])
ax.plot(path[:, 0], path[:, 1], color=BELT, lw=4.2, solid_capstyle="butt", zorder=4)
# teeth: short ticks on the inner side (toward the loop centre / the other leg), 2 mm pitch along the path
d = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
for s in np.arange(1.0, d[-1], 2.0):
    i = np.searchsorted(d, s); i = min(max(i, 1), len(path) - 1)
    tng = path[i] - path[i - 1]; tng /= np.linalg.norm(tng); nrm = np.array([tng[1], -tng[0]])
    ax.plot(*np.c_[path[i], path[i] + nrm * 0.62], color=BELT, lw=2.2, zorder=4, solid_capstyle="butt")
ax.add_patch(Circle((0, yc), B.PIN_D / 2, fc=PIN_C, ec="#5f656c", lw=0.8, zorder=5))
ax.annotate("", xy=(xa, -19.5), xytext=(xa, -13.5), arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.8))
kw = dict(fontsize=9.5, color=INK)
ax.text(xa - 0.6, -21.5, "load 98 N (10 kg)", ha="right", color=RED, fontsize=9.5)
ax.text(xb + 0.8, -8.5, "tail", ha="left", **kw)
ax.text(7.2, B.SLOT_L / 2, "slot 2.0\nlegs mesh tooth into tooth\n6 teeth (12)", va="center", **kw)
ax.text(7.2, yc, "M3 screw = pin\nthrough the loop", va="center", **kw)
yt = B.POCKET_YC + B.POCKET_R + B.WALL_X + B.TAB_L - B.X_HALF
ax.text(7.2, yt, "hang hole Ø6.5", va="center", **kw)
for y in (B.SLOT_L / 2, yc, yt):
    ax.plot([5.9, 7.0], [y, y], color=MUTE, lw=0.6)
ax.set_xlim(-9, 19); ax.set_ylim(-23, 40)
ax.set_title("Section through the belt centre, as loaded", fontsize=11, color=INK, loc="left")

# --- right: the three printed variants, top view with slot width ---
ax = axes[1]
for i, s in enumerate((1.8, 2.0, 2.2)):
    ci = B.coupon(s).translate([i * 15.0, 0, 0])
    for poly in ci.slice(zc).to_polygons():
        p = np.asarray(poly); ax.add_patch(Polygon(p, closed=True, fc=PART, ec=PART_E, lw=0.7))
    ax.text(i * 15.0, -3.2, f"slot {s:.1f}", ha="center", fontsize=10, color=INK)
    ax.text(i * 15.0, -5.6, ["1 notch", "2 notches", "3 notches"][i], ha="center", fontsize=8.5, color=MUTE)
ax.set_xlim(-8, 38); ax.set_ylim(-9, 40)
ax.set_title("Coupon set: one plate, three slot widths", fontsize=11, color=INK, loc="left")
fig.suptitle("Belt clamp v2: loop and pin (GT2 6 mm)", fontsize=14, color=INK, x=0.03, ha="left", y=0.985)
fig.text(0.03, 0.02, "Meshed pair 2.01 (E): the hang test picks the slot. Pull slides the legs against each other; the meshed teeth stop it, "
         "the slot walls keep them meshed, the pin stops the loop. No printed teeth.", fontsize=8.5, color=MUTE)
plt.savefig("out/belt_clamp_v2_section.png", facecolor=BG, bbox_inches="tight")
print("ok")
