"""Visuals for Overseer core v1 (27 Oct): the assembly sequence of everything inside the cage (motion.html, core pieces in
colour, the v5 cage, arms, plates and sensor head in grey as context) and a labelled side / top / section figure
(preview.png) drawn from the same meshes. Masses on the page are the system model's (E slots for the pieces).

python3 nox/overseer/core_v1_view.py      (after core_v1.py)
"""
import json, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene

RUN = os.path.join(ROOT, "runs", "overseer_core", "v1")
man = json.load(open(os.path.join(ROOT, "nox", "overseer", "overseer_core.assembly.json")))
r = json.load(open(os.path.join(RUN, "result.json")))
M = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in man["parts"]}
part = {p["name"]: p for p in man["parts"]}

CORE = {  # name: (colour, step, fly-in direction)
    "sled": ("#4c9be8", 0, (0, 0, -1)), "hub frame": ("#e8834c", 1, (0, 0, 1)), "FC MicoAir743-Lite": ("#2e7d32", 2, (0, 0, 1)),
    "Pi 5 + active cooler": ("#43a047", 3, (0, 0, 1)), "power tray": ("#e8b04c", 4, (-1, 0, 0)), "PDB FCHUB-12S": ("#7cb342", 5, (-1, 0, 0)),
    "UBEC 5 V": ("#9ccc65", 5, (-1, 0, 0)), "buck 12 V": ("#9ccc65", 5, (-1, 0, 0)), "RX XR2": ("#c5e1a5", 5, (-1, 0, 0)),
    "skid L (down camera pod)": ("#7e57c2", 6, (0, 1, 0)), "skid R (flow pod)": ("#7e57c2", 6, (0, -1, 0)),
    "down camera": ("#26a69a", 7, (0, 1, -1)), "flow MTF-02P": ("#26a69a", 7, (0, -1, -1)), "TPU feet": ("#424242", 8, (0, 0, -1)),
    "wires": ("#d84315", 9, (0, 0, 1)), "receiver": ("#8899aa", 10, (0, 0, -1)), "XT60 pair": ("#fbc02d", 10, (0, 0, -1)),
    "cartridge": ("#b0bec5", 11, (0, 0, -1)), "pack": ("#37474f", 11, (0, 0, -1)),
}
s = Scene("Overseer Core",
          f"Everything inside the cage but the arms (core v1, system pass): hub frame, power tray, sled, two skids with the camera and "
          f"flow pods, TPU feet; the real latch set in pose (mechanism not designed). Drone {r['mass_g']:.0f} g (E), belly to z "
          f"{r['z']['floor']:.1f}. Grey: v5 cage, arms, plates, sensor head (context).")
for n, m in M.items():
    if n.startswith("prop"):
        continue
    if n in CORE:
        s.part(n, m, group="core" if part[n]["role"] == "structure" else ("bought" if part[n]["role"] == "bought" else "latch"),
               color=CORE[n][0], printed=part[n]["role"] == "structure", note=f"{part[n].get('mass_g', 0):.0f} g ({part[n]['tag']})")
    else:
        s.part(n, m, group="context (v5)", color="#c8c8c8" if "cage" not in n else "#dddddd", printed=False)

STEP_S, N_STEP, FLY = 0.8, 12, 160.0


def poses(t):
    out = {}
    for n, (_, k, d) in CORE.items():
        u = np.clip((t - k * STEP_S) / STEP_S, 0, 1); u = 1 - (1 - u) ** 3
        T = np.eye(4); T[:3, 3] = np.asarray(d, float) / np.linalg.norm(d) * FLY * (1 - u); out[n] = T
    return out


s.timeline(poses, N_STEP * STEP_S + 1.5)
s.view("side", (60, -900, 0), (40, 0, -10)); s.view("below", (40, 0, -900), (40, 0, -20)); s.view("rear", (-900, 0, 20), (0, 0, -10))
s.write(os.path.join(RUN, "motion.html"))

# ---------------------------------------------------------------- labelled figure: side elevation, top view, section at x -10
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Poly
from shapely.geometry import MultiPoint
from shapely.ops import unary_union


def proj(m, i, j, n=4000):
    """projected silhouette: union of the projected triangles"""
    from shapely.geometry import Polygon
    tris = m.vertices[m.faces][:, :, [i, j]]
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]) / 2
    return unary_union([Polygon(t) for t in tris[area > 1e-3]]).buffer(0)


def section_x(m, x0):
    sec = m.section([1, 0, 0], [x0, 0, 0])
    if sec is None: return None
    p2, _ = sec.to_planar(to_2D=np.array([[0, 1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, x0], [0, 0, 0, 1.0]]), check=False)
    return p2.polygons_full


def draw(ax, geom, color, alpha=0.85, lw=0.4, ec="k"):
    gs = getattr(geom, "geoms", [geom])
    for g in gs:
        if g.is_empty or g.geom_type != "Polygon": continue
        ax.add_patch(Poly(np.asarray(g.exterior.coords), closed=True, fc=color, ec=ec, lw=lw, alpha=alpha))
        for hole in g.interiors:
            ax.add_patch(Poly(np.asarray(hole.coords), closed=True, fc="white", ec=ec, lw=lw * 0.5))


fig, axs = plt.subplots(1, 3, figsize=(21, 7.2), gridspec_kw={"width_ratios": [1.45, 1.45, 0.95]})
ctx_side = ["top plate", "bottom plate", "arm 1", "arm 2", "apex + cam bracket", "Unitree L2", "motor 1", "motor 2"]
order = ["pack", "cartridge", "receiver", "XT60 pair", "sled", "skid L (down camera pod)", "hub frame", "FC MicoAir743-Lite", "Pi 5 + active cooler",
         "power tray", "PDB FCHUB-12S", "UBEC 5 V", "buck 12 V", "RX XR2", "down camera", "TPU feet", "wires"]
for ax, (i, j, title) in zip(axs[:2], ((0, 2, "Side (x forward, z up), from -y"), (0, 1, "Top (x forward, y left), cage and plates hidden"))):
    for n in (ctx_side if i == 0 and j == 2 else ["arm 1", "arm 2", "arm 3", "arm 4", "bottom plate"]):
        draw(ax, proj(M[n], i, j), "#d0d0d0", 0.6, 0.3, "#999")
    for n in (order if j == 2 else [o for o in order if o not in ("pack", "cartridge", "wires")][::-1]):
        if n in ("skid R (flow pod)",) and j == 2: continue
        draw(ax, proj(M[n], i, j), CORE[n][0], 0.9 if part[n]["role"] == "structure" else 0.75)
    if j == 1:
        draw(ax, proj(M["skid R (flow pod)"], i, j), CORE["skid R (flow pod)"][0], 0.9)
        draw(ax, proj(M["flow MTF-02P"], i, j), CORE["flow MTF-02P"][0], 0.9)
        for x, y in r["arm_screws_driver_holes"]:
            ax.add_patch(plt.Circle((x, y), 4.5, fc="white", ec="#c62828", lw=1.0))
        for x, y in r["receiver_bolts"]:
            ax.plot(x, y, "s", ms=4, color="#0d47a1")
        for x, y in r["plate_bolts"]:
            ax.plot(x, y, "o", ms=4, color="#e65100")
    ax.set_title(title, fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
cg = r["cg"]
axs[0].plot(cg[0], cg[2], "x", color="#c62828", ms=12, mew=2); axs[0].annotate("CG", (cg[0], cg[2]), (cg[0] + 8, cg[2] + 8), color="#c62828")
axs[0].axhline(r["z"]["floor"], color="#555", lw=0.8, ls="--"); axs[0].text(-140, r["z"]["floor"] - 9, f"floor z {r['z']['floor']:.1f}", fontsize=8)
axs[0].set_xlim(-160, 460); axs[0].set_ylim(-100, 160)
axs[1].set_xlim(-160, 160); axs[1].set_ylim(-100, 100)
lab = [("hub frame", (40, 30, 34)), ("Pi 5 + active cooler", (-9, 0, 34)), ("power tray", (-80, 0, 4)), ("sled", (-60, 0, -5)),
       ("receiver", (-30, 0, -45)), ("skid L (down camera pod)", (100, 0, -65))]
for n, (x, y, z) in lab:
    axs[0].annotate(n.split(" (")[0], (x, z), (x + 30, z + 25 if z > 0 else z - 22), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
axs[1].text(-155, -95, "white rings: Ø9 driver holes under the 8 arm bottom screws; squares: receiver bolts; dots: plate bolts", fontsize=7.5)
# section at the pods
ax = axs[2]; x0 = -10.0
for n in ["bottom plate", "top plate", "arm 2", "arm 3"] + order + ["skid R (flow pod)", "flow MTF-02P"]:
    if n not in M: continue
    for pg in (section_x(M[n], x0) or []):
        draw(ax, pg, CORE[n][0] if n in CORE else "#d0d0d0", 0.9)
ax.axhline(r["z"]["floor"], color="#555", lw=0.8, ls="--")
ax.set_title(f"Section at x {x0:.0f} (the pods), looking forward", fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
ax.set_xlim(-95, 95); ax.set_ylim(-100, 60); ax.set_xlabel("y (left)"); ax.invert_xaxis()
fig.suptitle(f"Overseer core v1 (system pass): CG {r['cg'][0]:.1f}, {r['cg'][1]:.1f}, {r['cg'][2]:.1f}; tip-over fwd {r['tip_deg']['forward']:.0f}°, "
             f"back {r['tip_deg']['back']:.0f}°, side {r['tip_deg']['side']:.0f}°; pack top z {r['z']['pack_top']:.1f}, cartridge bottom z "
             f"{r['z']['cartridge_bottom']:.1f} (E)", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(RUN, "preview.png"), dpi=110)
print("wrote", os.path.join(RUN, "motion.html"), os.path.join(RUN, "preview.png"))
