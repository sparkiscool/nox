"""Detent renders (next arm version): look down the axle from the bars' side and a close iso. python3 detent_render.py"""
import math, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import arm_layout as L, arm_cad as A, detent as DT
from arm_cad import tm, cylY, rsquare
from hub_render import render, colr, BG

c = DT.CHOSEN; D = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); S = DT.arm_solids(D, np.array(c["Q"]))
Sb = A.axle_sun_bodies()
hub = trimesh.load("out/box_hub_v4.stl")
col = dict(hub=colr("#e8914f"), tower=colr("#d9783a"), ring=colr("#3d5a80"), p80=colr("#d8b84a"), loop=colr("#2b2b2b"),
           roller=colr("#c9ced6"), arm=colr("#f2b27a"), spring=colr("#c0392b"), pin=colr("#8a929c"), motor=colr("#2f3640"))
parts = [(hub, col["hub"]), (tm(S["tower"]), col["tower"]), (tm(DT.ring(c["th"])), col["ring"]), (tm(Sb["p80"]), col["p80"]),
         (tm(Sb["loop"]), col["loop"]), (tm(Sb["p20"]), col["p80"]), (tm(S["roller"] + S["nut"]), col["roller"]),
         (tm(S["arm"] + S["spacer"]), col["arm"]), (tm(S["springs"][0]), col["spring"]), (tm(S["hook"] + S["anchor"]), col["pin"]),
         (trimesh.util.concatenate([tm(rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)) for M in L.MOTORS.values()]), col["motor"])]
# close-up crop: keep what is near the detent
box = trimesh.creation.box(extents=[110, 70, 110]); box.apply_translation([-5, 62, -40])
near = []
for m, cc in parts:
    try: mm = m.slice_plane(box.bounds[0], [1, 0, 0], cap=True).slice_plane(box.bounds[0], [0, 1, 0], cap=True).slice_plane(box.bounds[0], [0, 0, 1], cap=True) \
        .slice_plane(box.bounds[1], [-1, 0, 0], cap=True).slice_plane(box.bounds[1], [0, -1, 0], cap=True).slice_plane(box.bounds[1], [0, 0, -1], cap=True)
    except Exception: continue
    if mm is not None and len(mm.faces): near.append((mm, cc))
fig, ax = plt.subplots(1, 2, figsize=(16, 7.8), dpi=150); fig.patch.set_facecolor(BG)
ax[0].imshow(render(near, 0, 89, 1000, 900)); ax[0].set_title("Down the axle from the bars' side (bars hidden)", fontsize=12, loc="left"); ax[0].axis("off")
ax[1].imshow(render(near, 200, 32, 1000, 900)); ax[1].set_title("Close iso: roller in a notch, springs on the hub v5 tower", fontsize=12, loc="left"); ax[1].axis("off")
fig.legend([Patch(fc=col[k]) for k in ("ring", "roller", "arm", "spring", "tower", "hub", "p80", "loop")],
           ["detent ring (fixed)", "683ZZ roller", "roller arm", "springs (2nd optional)", "hub v5 tower", "hub", "80T sun", "122T loop"],
           loc="lower center", ncol=8, frameon=False, fontsize=9)
fig.suptitle("Nox box arm: auto-centering detent (next arm version)", x=0.02, ha="left", fontsize=14, color="#222")
plt.tight_layout(rect=(0, 0.05, 1, 0.93)); plt.savefig("out/detent_v1.png", facecolor=BG)
print("out/detent_v1.png")
