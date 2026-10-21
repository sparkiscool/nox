"""Hub v4 section drawing: the fixed sun post through the hub, cut on the plane through the axle and head A's bar (X = 0),
plus a view along the axle at the motor plate (Y 52). python3 hub_section.py -> out/hub_v4_section.png"""
import os, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import arm_layout as L, arm_cad as A
from arm_cad import tm, cylY

hub_p = "out/box_hub_v4.stl"
hub = trimesh.load(hub_p) if os.path.exists(hub_p) else tm(A.hub_parts()["drawn"] - A.hub_parts()["cuts"])
S = A.axle_sun_bodies()
brg = lambda y0: tm(cylY(0, 0, y0, y0 + L.BRG_W, L.BRG_OD / 2) - cylY(0, 0, y0 - 1, y0 + L.BRG_W + 1, L.BRG_ID / 2))
M = L.MOTORS["axle"]
parts = [  # name, mesh, face colour, edge colour
    ("stub (fixed)", tm(A.stub()), "#c9ced6", "#5d6673"),
    ("6010 x2", tm(cylY(0, 0, 8, 24, 40) - cylY(0, 0, 7, 25, 25)) + brg(24.0), "#e8e3d8", "#8a8170"),
    ("sun post (fixed)", tm(A.sun_post()), "#3d5a80", "#22344d"),
    ("hub v4 (turns)", hub, "#ee9b5c", "#9c5a26"),
    ("80T sun (fixed)", tm(S["p80"]), "#e0c25c", "#8a7426"),
    ("122T loop", tm(S["loop"]), "#2b2b2b", "#111111"),
    ("axle 20T (turns)", tm(S["p20"]), "#d4a017", "#7a5c00"),
    ("bars", tm(A.bar_2040(*L.BAR_A)) + tm(A.bar_2040(*L.BAR_B).rotate([0, -90, 0])), "#b8bec6", "#6b7480"),
]

def draw(ax, origin, normal, to2d, title, xlim, ylim):
    for name, m, fc, ec in parts:
        sec = m.section(plane_origin=origin, plane_normal=normal)
        if sec is None: continue
        from shapely.geometry import Polygon as SP
        from matplotlib.path import Path
        from matplotlib.patches import PathPatch
        g = None
        for poly in sec.discrete:                                   # even-odd: holes stay holes
            q = SP(to2d(poly)).buffer(0)
            g = q if g is None else g.symmetric_difference(q)
        for q in getattr(g, "geoms", [g]):
            if q.is_empty or q.geom_type != "Polygon": continue
            rings = [np.asarray(q.exterior.coords)] + [np.asarray(i.coords) for i in q.interiors]
            verts = np.concatenate(rings); codes = np.concatenate([[Path.MOVETO] + [Path.LINETO] * (len(r) - 2) + [Path.CLOSEPOLY] for r in rings])
            ax.add_patch(PathPatch(Path(verts, codes), fc=fc, ec=ec, lw=0.6, alpha=0.95))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal"); ax.set_title(title, fontsize=10, color="#333", loc="left")
    ax.tick_params(labelsize=7, colors="#777"); [s.set_color("#ccc") for s in ax.spines.values()]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.2), dpi=170, gridspec_kw=dict(width_ratios=[1, 1.15]))
fig.patch.set_facecolor("#fbfaf7")
for a in (a1, a2): a.set_facecolor("#fbfaf7")
draw(a1, [0, 0, 0], [1, 0, 0], lambda p: np.c_[p[:, 1], p[:, 2]], "Section X = 0 (along head A's bar): Y axial, Z radial", (-2, 98), (-48, 52))
a1.set_xlabel("Y (mm)", fontsize=8, color="#555"); a1.set_ylabel("Z (mm)", fontsize=8, color="#555")
draw(a2, [0, 59.0, 0], [0, 1, 0], lambda p: np.c_[p[:, 0], p[:, 2]], "Section Y 59 (belt plane), looking along the axle", (-100, 100), (-100, 100))
a2.set_xlabel("X (mm)", fontsize=8, color="#555"); a2.set_ylabel("Z (mm)", fontsize=8, color="#555")
handles = [plt.Rectangle((0, 0), 1, 1, fc=fc, ec=ec) for _, _, fc, ec in parts]
fig.legend(handles, [p[0] for p in parts], loc="lower center", ncol=len(parts), frameon=False, fontsize=8)
fig.suptitle("Nox box arm: hub v4 on the fixed sun post", fontsize=13, color="#222", x=0.06, ha="left")
plt.tight_layout(rect=(0, 0.05, 1, 0.95))
plt.savefig("out/hub_v4_section.png", facecolor=fig.get_facecolor())
print("out/hub_v4_section.png")
