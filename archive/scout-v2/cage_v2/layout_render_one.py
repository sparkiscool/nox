"""One cage v3 layout, 4 views + member list (frame model F). python3 layout_render_one.py out.png '<opt json>' 'title'"""
import sys, json, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import layout_v3 as L, s2_guard_frame as sg
from layout_render import draw
o = json.loads(sys.argv[2]); title = sys.argv[3]
P = sg.params_v2(PZ_=L.PZ, upper=("solid", o.get("upper", 2.0))); P.update(opt=o, spoke=("solid", 2.5), skirt=("solid", o.get("skirt_d", 3.0))); g = L.Guard3(P)
fig = plt.figure(figsize=(22, 13), dpi=110, facecolor="#f4f2ee")
views = [((26, -55), "iso, nose to the lower right"), ((90, -90), "top (nose to the right)"), ((0, -90), "side, from the right"), ((0, 0), "front, from the nose")]
pos = [(0.0, 0.47, 0.42, 0.50), (0.40, 0.47, 0.36, 0.50), (0.0, 0.0, 0.42, 0.47), (0.40, 0.0, 0.36, 0.47)]
for (ve, t), p in zip(views, pos):
    ax = fig.add_axes(p, projection="3d", facecolor="#f4f2ee"); draw(ax, g, *ve); ax.set_title(t, fontsize=12, color="#444", pad=-8)
# member list
rows = []
for nm, a, b in [(f"spoke {k}", *s) for k, s in enumerate([])]: pass
sp = L.spokes_of(o); names = {0: "nose", 1: "left", 2: "tail", 3: "right"}
for axis, a, Rp, ring in sp:
    rows.append((f"spoke to {names[axis]} tee", "2.5 carbon", f"{np.linalg.norm(Rp - a) - 8.5:.0f}"))
rows += [("lower ring, 1 per side x4", "2.5 carbon", "189"), ("upper ring, 1 per side x4", f"{o['upper']:.1f} carbon", "189")]
if o.get("bottom"):
    q = o["bottom"]["quad"]; import numpy as _n
    for t_ in q["targets"]:
        T = L.skirt_target(tuple(t_), 1, 1, L.PZ - 8.0); R0 = _n.array([q["x"], L.RCV_Y, q["z"]])
        rows.append(("skirt, receiver to ring x4", f"{o.get('skirt_d', 3.0):.1f} carbon", f"{_n.linalg.norm(T - R0) - 8.5:.0f}"))
else:
    rows.append(("legs to the feet x4", "3.0 carbon", "65"))
if o["side_strut"]: rows.append(("side struts x2", "3.0 carbon", "60"))
if o["tail_strut"]: rows.append(("tail strut", "3.0 carbon", "64"))
printed = ["crown on the 4 Radxa boss tops (PETG, generated)", "4 corner nodes with 8 wheels (PETG)", "4 upper tee sleeves for the spokes"]
if o.get("bottom"):
    printed += ["4 lower tee sleeves for the skirt", "receiver v3: 4 skirt sockets + 4 foot pads", "  (receiver chat adds them)"]
else:
    printed += ["2 side hubs on the side plate bolts", "4 feet (TPU)"]
ax = fig.add_axes((0.76, 0.05, 0.23, 0.9)); ax.set_axis_off()
txt = title + "\n\nRODS (C, cut length mm)\n" + "\n".join(f"  {a:30s} {b:11s} {c:>4s}" for a, b, c in rows)
txt += "\n\nPRINTED\n" + "\n".join("  " + p for p in printed)
txt += ("\n\nMOUNTS\n  top: crown on the top case\n  bottom: skirt into the receiver\n  no hubs on the plate" if o.get("bottom") else "\n\nNOT IN C\n  side struts, tail strut, tail hub")
txt += "\n\nKEY\n  black  carbon rods      red     crown spokes\n  orange crown            sand    PETG nodes\n  blue   TPU tees, feet   gold    wheels\n  grey   top case box, plate      blue discs props (z 41.2)\n  green  skirt rods       tan box receiver, orange box pack"
ax.text(0, 1, txt, va="top", ha="left", family="monospace", fontsize=10.5, color="#222")
fig.savefig(sys.argv[1])
