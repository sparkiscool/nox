"""Visual of the Overseer core mounting coupons v1 in place (27 Oct, Spark: "i dont see how thats gonna fit everything show me"):
the two coupons in their drone pose with every part they hold sitting on them (Pi 5, UBEC, buck, receiver, cartridge, pack, flow
sensor), the plates, the arm roots and the FC stack on the top plate as context. motion.html assembles them in order;
fit.png is the labelled top view, side view and the view from below.

python3 nox/overseer/core_coupons_view.py      (after core_v1.py and core_coupons_v1.py)
"""
import json, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion import Scene
import core_coupons_v1 as CC

OUT = os.path.join(ROOT, "runs", "overseer_core_coupons", "v1")
man = json.load(open(os.path.join(ROOT, "nox", "overseer", "overseer_core.assembly.json")))
M = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in man["parts"]}
STACK_NM, PI_NM = "FC stack (MicoAir743-Lite on 55A 4-in-1)", "Pi 5 + Active Cooler"


def tm(m):
    mm = m.to_mesh(); return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts))


deck_c = tm(CC.hub_deck_coupon())                                         # built in the drone frame (z 0 = bottom plate top)
sled_c = tm(CC.sled_coupon())                                             # built flipped about x, bed = plate face
sled_c.apply_transform(np.diag([1, -1, -1, 1.0])); sled_c.apply_translation([0, 0, CC.C["z_plate_bot"]])

HELD = {  # name: (colour, step, fly-in)
    "hub deck coupon": ("#e8834c", 1, (0, 0, 1)), PI_NM: ("#43a047", 2, (0, 0, 1)), "UBEC 5 V": ("#9ccc65", 3, (1, 0, 0)),
    "buck 12 V": ("#c5e1a5", 3, (1, 0, 0)), "sled coupon": ("#4c9be8", 0, (0, 0, -1)), "flow MTF-02P": ("#26a69a", 4, (0, -1, -1)),
    "receiver": ("#8899aa", 5, (0, 0, -1)), "cartridge": ("#b0bec5", 6, (0, 0, -1)), "pack": ("#37474f", 6, (0, 0, -1)), "XT60 pair": ("#fbc02d", 5, (0, 0, -1)),
    STACK_NM: ("#2e7d32", 7, (0, 0, 1)), "RX XR2": ("#a5d6a7", 7, (0, 0, 1)),
}
mesh = {**{k: M[k] for k in HELD if k in M}, "hub deck coupon": deck_c, "sled coupon": sled_c}
s = Scene("Core Coupons Fit",
          "The two mounting check coupons in place (orange: hub deck coupon on the bottom plate; blue: sled coupon under it, flipped as "
          "fitted) with every part they hold sitting on them: Pi 5 on its 4 posts, UBEC and buck on the front rails, the receiver and pack "
          "under the sled, the flow sensor in its pocket. Grey: plates and arms; the FC stack on the top plate (no coupon, it bolts to the plate).")
for n in ("bottom plate", "top plate", "arm 1", "arm 2", "arm 3", "arm 4"):
    s.part(n, M[n], group="plates and arms", color="#cfcfcf" if "plate" in n else "#bdbdbd", printed=False)
for n, (c, k, d) in HELD.items():
    s.part(n, mesh[n], group="coupons" if "coupon" in n else "held parts", color=c, printed="coupon" in n)
STEP = 0.8


def poses(t):
    out = {}
    for n, (_, k, d) in HELD.items():
        u = np.clip((t - k * STEP) / STEP, 0, 1); u = 1 - (1 - u) ** 3
        T = np.eye(4); T[:3, 3] = np.asarray(d, float) / np.linalg.norm(d) * 150.0 * (1 - u); out[n] = T
    return out


s.timeline(poses, 8 * STEP + 2.0)
s.view("top", (0, 0, 700), (-10, 0, 0)); s.view("side", (0, -700, 10), (-10, 0, 0)); s.view("below", (0, 0, -700), (-10, 0, -20))
s.write(os.path.join(OUT, "motion.html"))

# ---------------------------------------------------------------- labelled figure
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Poly
from shapely.geometry import Polygon
from shapely.ops import unary_union


def proj(m, i, j):
    tris = m.vertices[m.faces][:, :, [i, j]]
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]) / 2
    return unary_union([Polygon(t) for t in tris[area > 1e-3]]).buffer(0)


def draw(ax, geom, color, alpha=0.85, lw=0.5, ec="k", hatch=None):
    for g in getattr(geom, "geoms", [geom]):
        if g.is_empty or g.geom_type != "Polygon": continue
        ax.add_patch(Poly(np.asarray(g.exterior.coords), closed=True, fc=color, ec=ec, lw=lw, alpha=alpha, hatch=hatch))
        for h in g.interiors:
            ax.add_patch(Poly(np.asarray(h.coords), closed=True, fc="white", ec=ec, lw=lw * 0.5))


def label(ax, txt, xy, xytext):
    ax.annotate(txt, xy, xytext, fontsize=9, arrowprops=dict(arrowstyle="-", lw=0.7), bbox=dict(fc="white", ec="none", alpha=0.8))


fig, axs = plt.subplots(1, 3, figsize=(22, 8.5))
# 1. top: the hub deck coupon and what sits on it
ax = axs[0]
for n in ("bottom plate", "arm 1", "arm 2", "arm 3", "arm 4"):
    draw(ax, proj(M[n], 0, 1), "#dddddd", 0.7, 0.3, "#999")
draw(ax, proj(deck_c, 0, 1), HELD["hub deck coupon"][0], 0.95)
for n in ("UBEC 5 V", "buck 12 V", PI_NM):
    draw(ax, proj(M[n], 0, 1), HELD[n][0], 0.45, 1.0, HELD[n][0])
label(ax, "Pi 5 (outline) on the 4 posts\nports face aft, plugs go in here", (-30, 10), (-130, 75))
label(ax, "UBEC on the front rails\n(zip ties)", (55, -10), (70, -85))
label(ax, "buck", (45, 13), (90, 60))
label(ax, "6 feet: M3 through the plate holes", (24, 36.5), (10, 85))
label(ax, "arm roots (grey)", (-70, 60), (-135, 95))
ax.set_title("1. Hub deck coupon on the bottom plate (top view)", fontsize=12); ax.set_aspect("equal"); ax.set_xlim(-140, 140); ax.set_ylim(-100, 100); ax.grid(alpha=0.2)
# 2. side: both plates, the 45 gap, everything in it
ax = axs[1]
for n in ("bottom plate", "top plate", "arm 2", "arm 1"):
    draw(ax, proj(M[n], 0, 2), "#dddddd", 0.7, 0.3, "#999")
for n, m, c in (("sled coupon", sled_c, HELD["sled coupon"][0]), ("receiver", M["receiver"], "#8899aa"), ("pack", M["pack"], "#37474f"),
                ("flow", M["flow MTF-02P"], "#26a69a"), ("deck", deck_c, HELD["hub deck coupon"][0]), ("UBEC", M["UBEC 5 V"], "#9ccc65"),
                ("Pi", M[PI_NM], "#43a047"), ("stack", M[STACK_NM], "#2e7d32"), ("RX", M["RX XR2"], "#a5d6a7")):
    draw(ax, proj(m, 0, 2), c, 0.85)
ax.axhline(45, color="#555", lw=0.6, ls=":"); ax.axhline(0, color="#555", lw=0.6, ls=":")
label(ax, "top plate (z 45): FC + ESC bolt on top, no coupon", (0, 47), (-150, 105))
label(ax, "Pi 5 + cooler between the plates\n(21 mm of air over the cooler)", (-20, 18), (-150, 60))
label(ax, "hub deck coupon (8 tall)", (-30, 4), (40, 30))
label(ax, "UBEC", (60, 9), (110, 20))
label(ax, "sled coupon under the plate", (-90, -5), (40, -25))
label(ax, "battery receiver + pack hang from the sled", (-60, -45), (20, -60))
label(ax, "flow sensor pocket", (-10, -6), (60, -90))
ax.set_title("2. Side view: the 45 mm gap between the plates", fontsize=12); ax.set_aspect("equal"); ax.set_xlim(-160, 140); ax.set_ylim(-100, 125); ax.grid(alpha=0.2)
# 3. from below: the sled coupon, the receiver bolt pads, the arm screw rings, the flow pocket
ax = axs[2]
for n in ("bottom plate", "arm 1", "arm 2", "arm 3", "arm 4"):
    draw(ax, proj(M[n], 0, 1), "#dddddd", 0.7, 0.3, "#999")
draw(ax, proj(M["receiver"], 0, 1), "#8899aa", 0.35, 0.8, "#556")
draw(ax, proj(sled_c, 0, 1), HELD["sled coupon"][0], 0.95)
draw(ax, proj(M["flow MTF-02P"], 0, 1), "#26a69a", 0.9)
label(ax, "receiver footprint (bolts to the 6 pads)", (-30, 0), (-135, 85))
label(ax, "Ø9 rings over the 8 arm screws", (38, 52), (40, 90))
label(ax, "flow sensor pocket", (-10, -55), (20, -90))
label(ax, "6 pads under the hub deck feet", (-65, -22.5), (-140, -90))
ax.set_title("3. Sled coupon under the plate (seen from above, through the plate)", fontsize=12); ax.set_aspect("equal"); ax.set_xlim(-140, 140); ax.set_ylim(-100, 100); ax.grid(alpha=0.2)
fig.suptitle("Overseer core coupons in place: what sits where", fontsize=14)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "fit.png"), dpi=105)
print("wrote", os.path.join(OUT, "motion.html"), os.path.join(OUT, "fit.png"))

# ---------------------------------------------------------------- the sled coupon's holes, colour-coded (Spark: "are all those holes to bolt on the plate")
R_ = CC.R
fig, ax = plt.subplots(figsize=(13, 8.5))
for n in ("bottom plate",):
    draw(ax, proj(M[n], 0, 1), "#e6e6e6", 0.8, 0.3, "#999")
draw(ax, proj(sled_c, 0, 1), HELD["sled coupon"][0], 0.9)
for x, y in R_["feet_xy"]:
    ax.add_patch(plt.Circle((x, y), 4.5, fc="none", ec="#c62828", lw=2.5))
for x, y in R_["receiver_bolts"]:
    ax.add_patch(plt.Rectangle((x - 4.5, y - 4.5), 9, 9, fc="none", ec="#1a237e", lw=2.5))
for x, y in R_["arm_screws_driver_holes"]:
    ax.add_patch(plt.Circle((x, y), 8.5, fc="none", ec="#2e7d32", lw=2.5, ls="--"))
draw(ax, proj(M["flow MTF-02P"], 0, 1), "#26a69a", 0.9)
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([], [], marker="o", ms=14, mfc="none", mec="#c62828", mew=2.5, ls="", label="6 x M3 through the bottom plate (the only bolts to the plate; the hub deck sits on top of the same 6)"),
                   Line2D([], [], marker="s", ms=14, mfc="none", mec="#1a237e", mew=2.5, ls="", label="6 x M3 from the battery receiver, up into the sled (not into the plate)"),
                   Line2D([], [], marker="o", ms=20, mfc="none", mec="#2e7d32", mew=2.5, ls="", label="8 x Ø9 clearance rings: the arm screw heads pass, a screwdriver reaches them (no bolt)"),
                   Line2D([], [], marker="s", ms=14, mfc="#26a69a", mec="k", ls="", label="flow sensor pocket (no bolt)")],
          loc="lower center", bbox_to_anchor=(0.5, -0.32), fontsize=10, frameon=False)
ax.set_title("Sled coupon (blue), seen from above through the bottom plate: what each hole is for", fontsize=12)
ax.set_aspect("equal"); ax.set_xlim(-130, 70); ax.set_ylim(-80, 75); ax.grid(alpha=0.2)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "sled_holes.png"), dpi=110)
print("wrote", os.path.join(OUT, "sled_holes.png"))
