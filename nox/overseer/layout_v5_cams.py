"""Overseer camera pair (27 Oct): the chosen orientation and the two alternatives as fields on a direction map, over the L2
field. python3 nox/overseer/layout_v5_cams.py -> runs/overseer_layout/v5/cameras_alt.png (orientation only; the mounts and
the cage checks are layout_v5.py's)."""
import math, os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); RUN = os.path.join(HERE, "..", "..", "runs", "overseer_layout", "v5")
HF, VF = 73.0, 89.0                                   # DECXIN 2633V2, 102 diag (E)
A = np.array(json.load(open(os.path.join(RUN, "result.json")))["l2_axis"])
CASES = [("chosen: landscape, 45° out, 10° down", "landscape", 45.0, -10.0),
         ("alt: portrait, 50° out, 15° down (most L2 field)", "portrait", 50.0, -15.0),
         ("alt: portrait, 37.5° out, level", "portrait", 37.5, 0.0)]


def axes(psi, th, sy):
    p, t = math.radians(psi), math.radians(th)
    f = np.array([math.cos(t) * math.cos(p), sy * math.cos(t) * math.sin(p), math.sin(t)])
    u = np.array([0, 0, 1.0]) - f[2] * f; u /= np.linalg.norm(u)
    return f, u, np.cross(u, f)


az, el = np.meshgrid(np.linspace(-180, 180, 721), np.linspace(-90, 90, 361))
D = np.stack([np.cos(np.radians(el)) * np.cos(np.radians(az)), np.cos(np.radians(el)) * np.sin(np.radians(az)), np.sin(np.radians(el))], -1)
fig, axs = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
for ax, (title, fmt, psi, th) in zip(axs, CASES):
    ha, hv = (VF / 2, HF / 2) if fmt == "landscape" else (HF / 2, VF / 2)
    cov = np.zeros(az.shape, bool)
    for sy in (1, -1):
        f, u, r = axes(psi, th, sy); z = D @ f; zz = np.maximum(z, 1e-6)
        cov |= (z > 0) & (np.abs(np.degrees(np.arctan2(D @ r, zz))) < ha) & (np.abs(np.degrees(np.arctan2(D @ u, zz))) < hv)
    l2 = D @ A >= 0
    U = (cov * np.cos(np.radians(el))).sum() / np.cos(np.radians(el)).sum() * 4 * math.pi
    S = (cov & l2).astype(float)[...].__mul__(np.cos(np.radians(el))).sum() / (l2 * np.cos(np.radians(el))).sum()
    h = np.abs(az[180]) < 40; row = cov[180]; gap = 0.0 if row[np.abs(az[180]) < 0.3].all() else 2 * np.abs(az[180][row & h]).min()
    ax.contourf(az, el, cov.astype(float), levels=[0.5, 1.5], colors=["#2a78d6"], alpha=0.35)
    ax.contour(az, el, cov.astype(float), levels=[0.5], colors=["#2a78d6"], linewidths=1.5)
    ax.contour(az, el, (D @ A), levels=[0.0], colors=["#eb6834"], linewidths=2.0)
    ax.axvline(0, color="#52514e", lw=0.6, ls=":")
    ax.set_title(f"{title}\n{U:.2f} sr, nose gap {gap:.1f}°, {100 * S:.0f} % of the L2 field", fontsize=10, color="#0b0b0b")
    ax.set_xlim(180, -180); ax.set_xlabel("azimuth, ° (left +, nose 0)", color="#52514e"); ax.grid(color="#e4e3df", lw=0.5)
    for s_ in ax.spines.values(): s_.set_color("#c3c2b7")
axs[0].set_ylabel("elevation, °", color="#52514e")
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
fig.legend(handles=[Patch(facecolor="#2a78d6", alpha=0.35, edgecolor="#2a78d6", label="camera fields (the pair)"),
                    Line2D([0], [0], color="#eb6834", lw=2, label="L2 field edge (its 90° half-sphere is the side holding the nose)")],
           loc="lower center", ncol=2, frameon=False)
fig.suptitle("Overseer camera pair beside the L2: where the two cameras look (no overlap in all three; the tie-break is the gap at the nose)", fontsize=11)
fig.tight_layout(rect=(0, 0.07, 1, 0.95)); fig.savefig(os.path.join(RUN, "cameras_alt.png"), dpi=110, facecolor="#fcfcfb")
print("wrote", os.path.join(RUN, "cameras_alt.png"))
