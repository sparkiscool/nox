"""Section + layout figure for the head belt clamp (belt_clamp.py). python belt_clamp_figure.py -> out/belt_clamp_section.png"""
import numpy as np, manifold3d as mf, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrowPatch
import belt_clamp as B

def section(m, w=3.0):
    # rotate so w -> z, slice at z = w; returns list of polygons in (p, u)
    r = m.transform([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0]])   # (p, w, u) -> (p, u, -w)
    cs = r.slice(-w)
    return [np.asarray(p) for p in cs.to_polygons()]

u0 = -18.6
bed = B.bed(u0); pl = B.plate(u0); belt = B.belt_ends(u0)
uc = u0 + B.END_L + B.LAND_L / 2
screw = (B.cyl_p(3.0, uc, B.P_FL[0] + B.HEAD_H, B.P_RIDGE + 5.6, 1.5) + B.cyl_p(3.0, uc, B.P_FL[0], B.P_FL[0] + B.HEAD_H - 0.01, 2.75))
insert = B.cyl_p(3.0, uc, B.P_RIDGE, B.P_RIDGE + B.INSERT_L, B.INSERT_D / 2) - B.cyl_p(3.0, uc, B.P_RIDGE - 1, B.P_RIDGE + 7, 1.5)

INK = "#1f2328"; BG = "#fbfaf7"
cols = dict(bed=("#d9d6cf", "#8f8a80"), plate=("#7aa6c2", "#3c6a87"), belt=("#2b2f36", "#2b2f36"),
            screw=("#b8bcc2", "#6b7078"), insert=("#c9a24a", "#8a6a22"))
fig, ax = plt.subplots(figsize=(13, 5.2), dpi=170)
fig.patch.set_facecolor(BG); ax.set_facecolor(BG)
for name, m in (("bed", bed), ("insert", insert), ("plate", pl), ("belt", belt), ("screw", screw)):
    fc, ec = cols[name]
    for poly in section(m):
        ax.add_patch(Polygon(np.c_[poly[:, 1], -poly[:, 0]], closed=True, fc=fc, ec=ec, lw=0.6, joinstyle="round"))
ax.set_xlim(u0 - 14, u0 + B.CLAMP_L + 14); ax.set_ylim(-9.5, 9.8); ax.set_aspect("equal"); ax.axis("off")
kw = dict(fontsize=9.5, color=INK, family="DejaVu Sans")
ax.annotate("", xy=(u0 - 12, 2.2), xytext=(u0 - 2, 2.2), arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=1.6))
ax.text(u0 - 12.5, 3.0, "to motor 20T", **kw)
ax.annotate("", xy=(u0 + B.CLAMP_L + 12, 2.2), xytext=(u0 + B.CLAMP_L + 2, 2.2), arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=1.6))
ax.text(u0 + B.CLAMP_L + 3.5, 3.0, "to idler", **kw)
ax.text(u0 + 8, -8.6, "8 teeth (16)", ha="center", **kw)
ax.text(uc, -8.6, "land 8", ha="center", **kw)
ax.text(u0 + B.CLAMP_L - 8, -8.6, "8 teeth (16)", ha="center", **kw)
ax.text(uc, 8.2, "1 x %s into an M3 heat-set insert" % B.SCREW, ha="center", **kw)
ax.text(u0 + 8, -4.6, "toothed bed (printed with the head)", ha="center", fontsize=9, color="#5c574d")
ax.text(u0 + 8, 4.2, "cover plate (PETG, 1 per head)", ha="center", fontsize=9, color="#ffffff")
ax.set_title("Head belt clamp v1  —  section through the belt centre (GT2 6 mm, both ends of the open belt)", fontsize=12, color=INK, loc="center", pad=6)
fig.text(0.012, 0.03, "Teeth carry the pull; the plate only stops them lifting. Plate boss bottoms on the land = 0.10 squeeze on the belt backs. "
         "Head A: bed at u -18.6..21.4  ·  Head B: u -15.6..24.4  (C)", fontsize=8.5, color="#6b665c")
plt.savefig("out/belt_clamp_section.png", facecolor=BG, bbox_inches="tight")
print("ok")
