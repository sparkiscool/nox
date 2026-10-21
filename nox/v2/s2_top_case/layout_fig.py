import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
import geom_top as G
fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2), gridspec_kw=dict(width_ratios=[1, 1.15]))
# top view
for mx, my in G.MOTORS:
    a.add_patch(Circle((mx, my), G.PROP_R, fc="#dde6f2", ec="#6a87b0", lw=1))
    a.add_patch(Circle((mx, my), G.PROP_R + G.PROP_KEEP, fc="none", ec="#6a87b0", lw=0.8, ls="--"))
s = G.FC_L1 - 1
a.add_patch(Polygon([(G.FC_C[0] + s, 0), (G.FC_C[0], s), (G.FC_C[0] - s, 0), (G.FC_C[0], -s)], fc="#cfcfcf", ec="k", lw=1, label="FC on grommets z 23-33"))
def R(d, **k): return Rectangle((d["x"][0], d["y"][0]), d["x"][1] - d["x"][0], d["y"][1] - d["y"][0], **k)
a.add_patch(R(G.UBEC, fc="#f2b880", ec="k", lw=1, label="UBEC on edge z 38-56"))
a.add_patch(R(G.XR2, fc="#a8d5a2", ec="k", lw=1, label="XR2 z 40-54"))
a.add_patch(Rectangle((-15, -32.5), 30, 65, fc="#7aa6d855", ec="#1f4e8c", lw=1.5, label="Radxa z 59-66 (parts down)"))
for hx, hy in G.R_HOLES: a.add_patch(Circle((hx, hy), G.BOSS_R, fc="#1f4e8c"))
for bx, by in G.bolts(): a.add_patch(Circle((bx, by), G.PAD_R, fc="#555", ec="k"))
a.annotate("ports + camera plug", (16.5, -9), (24, -40), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
a.annotate("SD", (0, 33), (-20, 46), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
a.set_xlim(-75, 75); a.set_ylim(-75, 75); a.set_aspect("equal"); a.set_title("Top view (nose +x right, left +y up)", fontsize=10)
a.legend(fontsize=7, loc="lower left"); a.grid(alpha=0.2)
# side view from the right (x horizontal, z up)
b.axhspan(G.PROP_Z - G.SWEEP, G.PROP_Z + G.SWEEP, color="#dde6f2", label="prop sweep z 36.5-44.5 (E)")
b.axhline(G.PLATE_TOP, color="k", lw=2); b.text(-70, G.PLATE_TOP - 4, "plate top z 22", fontsize=8)
for mx in (54.1, -54.1):
    b.add_patch(Rectangle((mx - G.PROP_R, G.PROP_Z - 0.6), 2 * G.PROP_R, 1.2, fc="#6a87b0"))
b.add_patch(Rectangle((G.FC_C[0] - 25.46, 23), 50.9, 10, fc="#cfcfcf", ec="k", label="FC"))
b.add_patch(Rectangle((G.FC_C[0] - 25.46, 33), 50.9, 5, fc="none", ec="#999", ls=":", label="FC plug room"))
b.add_patch(Rectangle((G.UBEC["x"][0], G.UBEC["z"][0]), 10, 18, fc="#f2b880", ec="k", label="UBEC"))
b.add_patch(Rectangle((G.LAND["x"][0], G.LAND["z"][0]), 2, 14, fc="#444", label="tape land"))
b.add_patch(Rectangle((G.XR2["x"][0], G.XR2["z"][0]), 5, 13, fc="#a8d5a2", ec="k", label="XR2"))
b.add_patch(Rectangle((-15, G.R_LOW), 31.75, G.R_BACK_PARTS - G.R_LOW, fc="#7aa6d855", ec="#1f4e8c", lw=1.5, label="Radxa"))
for hx in (11.45, -11.45): b.add_patch(Rectangle((hx - 3.5, G.R_BACK), 7, G.BOSS_TOP - G.R_BACK, fc="#1f4e8c"))
b.axhline(G.BOSS_TOP, color="#1f4e8c", lw=0.8, ls="--"); b.text(20, G.BOSS_TOP + 1, f"top z {G.BOSS_TOP:.1f}", fontsize=8)
b.set_xlim(-75, 95); b.set_ylim(15, 80); b.set_aspect("equal"); b.set_title("Side view (from the right)", fontsize=10)
b.legend(fontsize=7, loc="upper right", ncol=1, framealpha=0.9); b.grid(alpha=0.2)
plt.tight_layout(); plt.savefig("s2t_layout.png", dpi=150)
