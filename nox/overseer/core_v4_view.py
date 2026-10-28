"""Overseer core v4 pictures: runs/overseer_core/v4/preview.png (the core with no head load: hub deck concept on 6 feet, Pi, bays,
RX beside arm 2's top screw line) and alternatives.png (A topology deck on the plate's 6 holes, B no deck: Pi on standoffs through
the carbon plate, C v9's deck with empty sockets), plan views with the numbers.
python3 nox/overseer/core_v4_view.py [assembly.json]   (default: runs/overseer_core/v4/assembly_on_cage_branch.json, else the shared one)
"""
import json, os, sys, trimesh, numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUT = "runs/overseer_core/v4"
MAN = sys.argv[1] if len(sys.argv) > 1 else next(p for p in (f"{OUT}/assembly_on_cage_branch.json", "nox/overseer/overseer_core.assembly.json") if os.path.exists(p))
d = json.load(open(MAN)); P = {p["name"]: p for p in d["parts"]}
L = lambda n: trimesh.load(P[n]["mesh"])
r = json.load(open(f"{OUT}/result.json"))
lt = np.array([0.3, -0.45, 0.85]); lt /= np.linalg.norm(lt)


def draw(ax, m, col, a=1.0, maxf=20000):
    F, N = (m.faces, m.face_normals)
    if len(F) > maxf:
        idx = np.random.default_rng(0).choice(len(F), maxf, replace=False); F, N = F[idx], N[idx]
    sh = 0.42 + 0.58 * np.clip(np.abs(N @ lt), 0, 1); c = np.array(matplotlib.colors.to_rgb(col))[None] * sh[:, None]
    ax.add_collection3d(Poly3DCollection(m.vertices[F], facecolors=np.c_[c, np.full(len(c), a)], linewidths=0))


groups = [("bottom plate", "#c9ced6", 0.35), ("top plate", "#c9ced6", 0.25), ("arm 1", "#b8bec7", 0.3), ("arm 2", "#b8bec7", 0.3),
          ("arm 3", "#b8bec7", 0.3), ("arm 4", "#b8bec7", 0.3), ("UBEC 5 V", "#6b4fa0", 1), ("buck 12 V", "#a04f8a", 1),
          ("Pi 5 + Active Cooler", "#2e7d4f", 0.9), ("hub deck", "#e8833a", 1), ("flow clip", "#2f7fd1", 1), ("RX XR2", "#d62728", 1),
          ("FC stack (MicoAir743-Lite on 55A 4-in-1)", "#333a45", 0.9)]
zt = r["z"]["top_plate_top"]
fig = plt.figure(figsize=(16, 7), facecolor="white")
for i, (el, az, t) in enumerate([(24, -60, "iso, nose right"), (0, -90, "side: nothing of the head lands on the deck"), (90, -90, "plan: RX 5.5 off arm 2's top screw line")]):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    for n, c, a in groups:
        if i == 2 and n == "top plate": continue
        draw(ax, L(n), c, a)
    for x, y in r["arm_top_screws"]:
        ax.plot([x, x], [y, y], [zt, zt + 60], color="#d62728" if (x < 0 < y) else "#888", lw=2.5, ls="--")
    ax.set_xlim(-80, 120); ax.set_ylim(-90, 90); ax.set_zlim(-15, 85); ax.set_box_aspect((200, 180, 100)); ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t, fontsize=12)
fig.suptitle("Overseer core v4: the cage hangs the L2 head on 3 rods to its own nodes, so the hub deck (orange, concept body; v12 topology next) carries only\n"
             "the Pi (green), UBEC and buck (purple) on 6 feet: no tower, no sockets. RX (red) moved 6 to +y; dashed: the 4 arm top screw driver lines.", fontsize=11.5)
plt.tight_layout(rect=(0, 0, 1, 0.9)); plt.savefig(f"{OUT}/preview.png", dpi=105)

# alternatives: plan sketches with the numbers
fig2, axs = plt.subplots(1, 3, figsize=(17, 6.6), facecolor="white")
feet = r["feet_xy"]; ph = r["pi_holes"]; c = r["coupon"]; ub, bk = c["ubec_bay"], c["buck_bay"]
slot = r["slot"]
T = ["A (chosen, DEFAULT): one topology hub deck on the plate's own 6 holes\n~29 g (E: drawn 11.8 + gen 11 cm³); feet + inserts, M3 up from below;\nno holes drilled, every head clear of the slot; Pi mount >= 100 Hz expected",
     "B: no deck. Pi on 4 nylon M2.5 standoffs through 4 holes DRILLED in the\ncarbon bottom plate, UBEC / buck on VHB, clip on 2 nyloc nuts. ~4 g (E),\nPi ~2 kHz (E); but 4 heads in the receiver's reserved slot top 6 (red)",
     "C: keep v9's deck, sockets left empty\n68.0 g (M): ~40 g of tower carrying nothing; HOLD by Spark's look\n(\"material at the surface, not the back\")"]
for k, ax in enumerate(axs):
    ax.add_patch(Rectangle((-95.5, -60), 196, 120, fc="#eef0f3", ec="#9aa3ad"))
    ax.add_patch(Rectangle((slot["x"][0], -slot["y"]), slot["x"][1] - slot["x"][0], 2 * slot["y"], fc="none", ec="#7f8c8d", ls=":", lw=1.5))
    ax.text(-90, -55, "battery slot below (dotted)", fontsize=8, color="#7f8c8d")
    ax.add_patch(Rectangle((c["pi_x"][0], -28), c["pi_x"][1] - c["pi_x"][0], 56, fc="#2e7d4f", alpha=0.25, ec="#2e7d4f"))
    ax.add_patch(Rectangle((ub[0], ub[2]), 48, ub[3] - ub[2], fc="#6b4fa0", alpha=0.35)); ax.add_patch(Rectangle((bk[0], bk[2]), bk[1] - bk[0], bk[3] - bk[2], fc="#a04f8a", alpha=0.35))
    if k == 0:
        for x, y in feet: ax.add_patch(Circle((x, y), 7, fc="#e8833a"))
        for x, y in ph: ax.add_patch(Circle((x, y), 6, fc="#e8833a", ec="#2e7d4f"))
        ax.plot([-24, -30, 28, 65, 96, 96, 65, 28, -30, -24], [36.5, 24.5, 24.5, 22.5, 17, -17, -22.5, -24.5, -24.5, -36.5], color="#e8833a", lw=4, alpha=0.6)
        ax.text(-60, 48, "topology joins feet, posts and pads (sketch)", fontsize=9, color="#b35a1a")
    elif k == 1:
        for x, y in ph: ax.add_patch(Circle((x, y), 3.5, fc="#d62728"))
        for x, y in feet[4:]: ax.add_patch(Circle((x, y), 4, fc="#2f7fd1"))
        ax.text(-60, 48, "red: new holes in the carbon, heads below in the slot", fontsize=9, color="#d62728")
    else:
        for x, y in feet: ax.add_patch(Circle((x, y), 7, fc="#e8833a"))
        ax.add_patch(Rectangle((60, -34), 44, 68, fc="#e8833a", alpha=0.75)); ax.text(62, 0, "tower +\n6 empty\nsockets", fontsize=9)
    ax.set_xlim(-100, 110); ax.set_ylim(-65, 65); ax.set_aspect("equal"); ax.set_title(T[k], fontsize=10, loc="left"); ax.set_xticks([]); ax.set_yticks([])
fig2.suptitle("Overseer core v4, the hub deck now that no head load lands on it: three ways (plan, nose right)", fontsize=13)
plt.tight_layout(rect=(0, 0, 1, 0.93)); plt.savefig(f"{OUT}/alternatives.png", dpi=100)
print("written", f"{OUT}/preview.png", f"{OUT}/alternatives.png")
