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

STACK_NM, PI_NM = "FC stack (MicoAir743-Lite on 55A 4-in-1)", "Pi 5 + Active Cooler"
CORE = {  # name: (colour, assembly step, fly-in direction)
    "sled": ("#4c9be8", 0, (0, 0, -1)), "hub deck": ("#e8834c", 1, (0, 0, 1)), STACK_NM: ("#2e7d32", 2, (0, 0, 1)), "ESC cap": ("#9ccc65", 2, (0, 0, 1)),
    "UBEC 5 V": ("#9ccc65", 3, (-1, 0, 0)), "buck 12 V": ("#9ccc65", 3, (-1, 0, 0)), "RX XR2": ("#c5e1a5", 3, (1, 0, 0)),
    PI_NM: ("#43a047", 4, (0, 0, 1)), "flow MTF-02P": ("#26a69a", 5, (0, -1, -1)), "wires": ("#d84315", 6, (0, 0, 1)),
    "receiver": ("#8899aa", 7, (0, 0, -1)), "XT60 pair": ("#fbc02d", 7, (0, 0, -1)), "cartridge": ("#b0bec5", 8, (0, 0, -1)), "pack": ("#37474f", 8, (0, 0, -1)),
    "TPU feet (cage session, placeholders)": ("#424242", 9, (0, 0, -1)),
}
s = Scene("Overseer Core",
          f"Everything inside the cage but the arms (core v1 round 3): two printed pieces, the hub deck (FC stack, Pi 5, UBEC, buck, RX) and "
          f"the sled (receiver face, flow pod); every screw straight down with a screwdriver in this order. Real latch set in pose (mechanism "
          f"not designed). Drone {r['mass_g']:.0f} g (E). Grey: v5 cage, arms, plates, sensor head (context).")
for n, m in M.items():
    if n.startswith("prop"):
        continue
    if n in CORE:
        s.part(n, m, group="core" if part[n]["role"] == "structure" else ("bought" if part[n]["role"] == "bought" else "latch"),
               color=CORE[n][0], printed=part[n]["role"] == "structure", note=f"{part[n].get('mass_g', 0):.0f} g ({part[n]['tag']})")
    else:
        s.part(n, m, group="context (v5)", color="#c8c8c8" if "cage" not in n else "#dddddd", printed=False)

STEP_S, N_STEP, FLY = 0.9, 10, 160.0


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
order = ["pack", "cartridge", "receiver", "XT60 pair", "sled", "hub deck", STACK_NM, "ESC cap", PI_NM, "UBEC 5 V", "buck 12 V", "RX XR2", "flow MTF-02P", "wires"]
for ax, (i, j, title) in zip(axs[:2], ((0, 2, "Side (x forward, z up), from -y"), (0, 1, "Top (x forward, y left), cage and top plate hidden"))):
    for n in (ctx_side if j == 2 else ["arm 1", "arm 2", "arm 3", "arm 4", "bottom plate"]):
        draw(ax, proj(M[n], i, j), "#d0d0d0", 0.6, 0.3, "#999")
    for n in (order if j == 2 else [o for o in order if o not in ("pack", "cartridge", "wires", "receiver", "XT60 pair", "sled")] + []):
        draw(ax, proj(M[n], i, j), CORE[n][0], 0.9 if part[n]["role"] == "structure" else 0.75)
    ax.set_title(title, fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
for x, y in r["feet_xy"]:
    axs[1].add_patch(plt.Circle((x, y), 2.5, fc="white", ec="#c62828", lw=1.2))
for x, y in r["pi_holes"]:
    axs[1].add_patch(plt.Circle((x, y), 1.8, fc="white", ec="#1565c0", lw=1.2))
axs[1].axvline(-60.2, color="#555", lw=0.8, ls="--"); axs[1].text(-58, 90, "top plate edge", fontsize=8)
axs[1].text(-155, -95, "red rings: 6 M3 hub deck + sled bolts (straight down); blue: 4 Pi screws (reachable with the top plate on)", fontsize=7.5)
cg = r["cg"]
axs[0].plot(cg[0], cg[2], "x", color="#c62828", ms=12, mew=2); axs[0].annotate("CG", (cg[0], cg[2]), (cg[0] + 8, cg[2] + 8), color="#c62828")
axs[0].set_xlim(-170, 460); axs[0].set_ylim(-100, 160)
axs[1].set_xlim(-170, 160); axs[1].set_ylim(-100, 100)
for n, (x, z, dx, dz) in {"hub deck": (-30, 2, 10, -40), PI_NM: (-100, 30, -40, 55), STACK_NM: (0, 18, 40, 50), "sled": (-60, -5, 80, -40), "receiver": (-30, -45, 120, -30), "flow MTF-02P": (-10, -74, 150, -10)}.items():
    axs[0].annotate(n.split(" (")[0].replace(" + Active Cooler", ""), (x, z), (x + dx, z + dz), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
ax = axs[2]; x0 = -100.0
for n in ["bottom plate", "arm 2", "arm 3"] + order:
    if n not in M: continue
    for pg in (section_x(M[n], x0) or []):
        draw(ax, pg, CORE[n][0] if n in CORE else "#d0d0d0", 0.9)
ax.set_title(f"Section at x {x0:.0f} (Pi deck, UBEC, lead channel), looking forward", fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
ax.set_xlim(-60, 60); ax.set_ylim(-85, 50); ax.set_xlabel("y (left)"); ax.invert_xaxis()
fig.suptitle(f"Overseer core v1 round 3: 2 printed pieces; CG {cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}; {r['mass_g']:.0f} g, T/W {r['tw']} (E); "
             f"stack top z {r['z']['stack_top']}, Pi cooler top z {r['z']['cooler_top']:.1f} under the 45 gap", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(RUN, "preview.png"), dpi=110)
print("wrote", os.path.join(RUN, "motion.html"), os.path.join(RUN, "preview.png"))
