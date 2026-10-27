"""Visuals for Overseer core v1 (27 Oct, round 4): the assembly sequence of everything inside the cage (motion.html: core pieces
and parts in colour, the v5 cage, arms, plates and sensor head in grey as context; the two alternatives not taken beside it,
each a copy of the belly offset in y), a labelled side / top / section figure (preview.png) and the A / B / C alternatives
side by side with their numbers (alternatives.png), all drawn from the same meshes. Masses are the system model's (E slots).

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
FEET_NM = "TPU feet (cage session, placeholders)"
CORE = {  # name: (colour, assembly step, fly-in direction)
    "sled": ("#4c9be8", 0, (0, 0, -1)), "hub deck": ("#e8834c", 1, (0, 0, 1)), PI_NM: ("#43a047", 2, (0, 0, 1)),
    "UBEC 5 V": ("#9ccc65", 3, (1, 0, 0)), "buck 12 V": ("#9ccc65", 3, (1, 0, 0)),
    STACK_NM: ("#2e7d32", 5, (0, 0, 1)), "ESC cap": ("#9ccc65", 5, (0, 0, 1)), "RX XR2": ("#c5e1a5", 5, (0, 0, 1)),
    "flow MTF-02P": ("#26a69a", 6, (0, -1, -1)), "wires": ("#d84315", 7, (0, 0, 1)),
    "receiver": ("#8899aa", 8, (0, 0, -1)), "XT60 pair": ("#fbc02d", 8, (0, 0, -1)), "cartridge": ("#b0bec5", 9, (0, 0, -1)), "pack": ("#37474f", 9, (0, 0, -1)),
    FEET_NM: ("#424242", 10, (0, 0, -1)),
}
s = Scene("Overseer Core",
          f"Everything inside the cage but the arms (core v1 round 5): two printed pieces (the FC + 4-in-1 stack and the RX bolt straight to the carbon top plate), "
          f"the hub deck (Pi 5, UBEC, buck) and the sled (receiver face, flow pod); every screw straight down with a screwdriver in this order. "
          f"Real latch set in pose (mechanism not designed). Drone {r['mass_g']:.0f} g (E). Grey: v5 cage, arms, plates, sensor head. "
          f"Beside it: alternative A (skids, +y) and C (receiver on the plate, -y), not taken.")
for n, m in M.items():
    if n.startswith("prop"):
        continue
    if n in CORE:
        s.part(n, m, group="core" if part[n]["role"] == "structure" else ("bought" if part[n]["role"] == "bought" else "latch"),
               color=CORE[n][0], printed=part[n]["role"] == "structure", note=f"{part[n].get('mass_g', 0):.0f} g ({part[n]['tag']})")
    else:
        s.part(n, m, group="context (v5)", color="#c8c8c8" if "cage" not in n else "#dddddd", printed=False)
# alternatives not taken, each a copy of the belly offset in y
A_OFF, C_OFF = np.array([0, 520.0, 0]), np.array([0, -520.0, 0])
for n in ["bottom plate", "receiver", "cartridge", "flow MTF-02P", "arm 1", "arm 2", "arm 3", "arm 4"]:
    s.part(f"A: {n}", M[n].copy().apply_translation(A_OFF), group="alternative A (not taken)", color="#bbbbbb", printed=False)
for f, nm in (("skid_l_down_camera_pod.stl", "skid L"), ("skid_r_flow_pod.stl", "skid R"), ("tpu_feet.stl", "TPU feet on the skids")):
    s.part(f"A: {nm}", trimesh.load(os.path.join(RUN, "alt_a", f)).apply_translation(A_OFF), group="alternative A (not taken)", color="#7e57c2", printed=False,
           note="round 1: front feet 100 ahead of the last joint, ~1.6 kN a joint bolt, rails in the prop wash")
for n in ["bottom plate", "arm 1", "arm 2", "arm 3", "arm 4"]:
    s.part(f"C: {n}", M[n].copy().apply_translation(C_OFF), group="alternative C (not taken)", color="#bbbbbb", printed=False)
for n in ("receiver", "cartridge"):
    s.part(f"C: {n}", M[n].copy().apply_translation(C_OFF + [0, 0, 6.0]), group="alternative C (not taken)", color="#8899aa", printed=False,
           note="no sled: 6 shallower; the front receiver bolts land in the front arm roots, the 2 XT60-end bolts 13 past the plate")
for x, y in r["receiver_bolts"]:
    s.part(f"C: bolt {x:.0f} {y:.0f}", trimesh.creation.cylinder(1.6, 30, sections=12).apply_translation([x, y + C_OFF[1], 6.0]),
           group="alternative C (not taken)", color="#c62828", printed=False)

STEP_S, N_STEP, FLY = 0.8, 11, 160.0


def poses(t):
    out = {}
    for n, (_, k, d) in CORE.items():
        u = np.clip((t - k * STEP_S) / STEP_S, 0, 1); u = 1 - (1 - u) ** 3
        T = np.eye(4); T[:3, 3] = np.asarray(d, float) / np.linalg.norm(d) * FLY * (1 - u); out[n] = T
    return out


s.timeline(poses, N_STEP * STEP_S + 1.5)
s.view("side", (60, -900, 0), (20, 0, 0)); s.view("below", (20, 0, -900), (20, 0, -20)); s.view("rear", (-900, 0, 40), (0, 0, 10))
s.write(os.path.join(RUN, "motion.html"))

# ---------------------------------------------------------------- figures
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


def section_x(m, x0):
    sec = m.section([1, 0, 0], [x0, 0, 0])
    if sec is None: return None
    p2, _ = sec.to_2D(to_2D=np.array([[0, 1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, x0], [0, 0, 0, 1.0]]), check=False)
    return p2.polygons_full


def draw(ax, geom, color, alpha=0.85, lw=0.4, ec="k"):
    for g in getattr(geom, "geoms", [geom]):
        if g.is_empty or g.geom_type != "Polygon": continue
        ax.add_patch(Poly(np.asarray(g.exterior.coords), closed=True, fc=color, ec=ec, lw=lw, alpha=alpha))
        for hole in g.interiors:
            ax.add_patch(Poly(np.asarray(hole.coords), closed=True, fc="white", ec=ec, lw=lw * 0.5))


fig, axs = plt.subplots(1, 3, figsize=(21, 7.2), gridspec_kw={"width_ratios": [1.45, 1.45, 0.95]})
ctx_side = ["top plate", "bottom plate", "arm 1", "arm 2", "apex + cam bracket", "Unitree L2", "motor 1", "motor 2"]
order = ["pack", "cartridge", "receiver", "XT60 pair", "sled", "hub deck", PI_NM, "UBEC 5 V", "buck 12 V", STACK_NM, "ESC cap", "RX XR2", "flow MTF-02P", "wires"]
for ax, (i, j, title) in zip(axs[:2], ((0, 2, "Side (x forward, z up), from -y"), (0, 1, "Top (x forward, y left): top plate parts drawn over the hub"))):
    for n in (ctx_side if j == 2 else ["arm 1", "arm 2", "arm 3", "arm 4", "bottom plate"]):
        draw(ax, proj(M[n], i, j), "#d0d0d0", 0.6, 0.3, "#999")
    for n in (order if j == 2 else ["hub deck", PI_NM, "UBEC 5 V", "buck 12 V", STACK_NM, "ESC cap", "RX XR2"]):
        draw(ax, proj(M[n], i, j), CORE[n][0], 0.9 if part[n]["role"] == "structure" else 0.7)
    ax.set_title(title, fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
for x, y in r["feet_xy"]:
    axs[1].add_patch(plt.Circle((x, y), 2.5, fc="white", ec="#c62828", lw=1.2))
axs[1].text(-135, -95, "red: 6 M3 hub deck + sled bolts; the FC stack bolts through the top plate's own 30.5 holes", fontsize=7.5)
cg = r["cg"]
axs[0].plot(cg[0], cg[2], "x", color="#c62828", ms=12, mew=2); axs[0].annotate("CG", (cg[0], cg[2]), (cg[0] + 8, cg[2] + 8), color="#c62828")
axs[0].set_xlim(-140, 460); axs[0].set_ylim(-100, 160); axs[1].set_xlim(-140, 140); axs[1].set_ylim(-100, 100)
for n, (x, z, dx, dz) in {"hub deck": (60, 2, 40, -35), PI_NM: (-30, 18, -60, 60), STACK_NM: (0, 60, 40, 50),
                          "sled": (-60, -5, 80, -40), "receiver": (-30, -45, 120, -30), "UBEC 5 V": (60, 9, 70, 20)}.items():
    axs[0].annotate(n.split(" (")[0].replace(" + Active Cooler", ""), (x, z), (x + dx, z + dz), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
ax = axs[2]; x0 = 0.0
for n in ["bottom plate", "top plate"] + order:
    for pg in (section_x(M[n], x0) or []):
        draw(ax, pg, CORE[n][0] if n in CORE else "#d0d0d0", 0.9)
ax.set_title(f"Section at x {x0:.0f}, looking forward", fontsize=11); ax.set_aspect("equal"); ax.grid(alpha=0.25)
ax.set_xlim(-70, 70); ax.set_ylim(-85, 80); ax.set_xlabel("y (left)"); ax.invert_xaxis()
fig.suptitle(f"Overseer core v1 round 5: 2 printed pieces; CG {cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}; {r['mass_g']:.0f} g, T/W {r['tw']} (E); "
             f"FC top z {r['z']['stack_top']:.1f} on the top plate, Pi cooler top z {r['z']['cooler_top']:.1f} under the 45 gap", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(RUN, "preview.png"), dpi=110)

# alternatives A / B / C, side views of the belly with their numbers
fig, axs = plt.subplots(1, 3, figsize=(21, 6.0))
alt_a = {nm: trimesh.load(os.path.join(RUN, "alt_a", f)) for f, nm in (("skid_l_down_camera_pod.stl", "skid"), ("tpu_feet.stl", "feet"))}
cases = [("A: skids carry the pods and the feet (round 1, not taken)", [("receiver", M["receiver"], "#8899aa"), ("sled", M["sled"], "#4c9be8"),
                                                                       ("skid", alt_a["skid"], "#7e57c2"), ("feet", alt_a["feet"], "#424242")],
          "front feet 100 ahead of the last joint: ~1.6 kN a joint bolt in a one-side -Z crash (review recompute, E)\n2 rails 19 x 55 in the rear prop wash; +2 x 45 g; tip 28° forward"),
         ("B: feet on the cage's opening ring nodes (chosen)", [("receiver", M["receiver"], "#8899aa"), ("sled", M["sled"], "#4c9be8"),
                                                               ("feet", M[FEET_NM], "#424242"), ("flow", M["flow MTF-02P"], "#26a69a")],
          "floor load into the cage (sized for 35 g through the node that touches first, L)\ncore keeps 2 pieces (slots 99 g); shell reaches z -82.9 (+6 on what it needs)"),
         ("C: receiver straight on the bottom plate, no sled (not taken)", [("receiver", M["receiver"].copy().apply_translation([0, 0, 6.0]), "#8899aa")],
          "6 shallower, sled 75 g -> ~20 g bracket; but the front receiver bolts (56.85, ±27.8) land in\nthe front arm roots (u 59.9) and the 2 XT60-end bolts sit 13 past the plate edge")]
for ax, (title, parts_, note) in zip(axs, cases):
    for n in ("bottom plate", "arm 1", "arm 2"):
        draw(ax, proj(M[n], 0, 2), "#d0d0d0", 0.6, 0.3, "#999")
    for nm, m, c in parts_:
        draw(ax, proj(m, 0, 2), c, 0.85)
    if title.startswith("C"):
        for x, y in r["receiver_bolts"]:
            ax.plot([x, x], [-2.1, 4], color="#c62828", lw=2)
    ax.plot(cg[0], cg[2], "x", color="#c62828", ms=10, mew=2)
    ax.set_title(title, fontsize=10.5); ax.text(-150, -118, note, fontsize=8, va="top")
    ax.set_aspect("equal"); ax.set_xlim(-160, 200); ax.set_ylim(-125, 60); ax.grid(alpha=0.25)
fig.suptitle("Overseer core: three ways to carry the belly and stand on the floor (side views, x forward; red x = CG)", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(RUN, "alternatives.png"), dpi=110)
print("wrote", os.path.join(RUN, "motion.html"), "preview.png", "alternatives.png")
