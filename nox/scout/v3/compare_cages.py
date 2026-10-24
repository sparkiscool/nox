"""Before / after picture of two Scout v3 cages (rod reduction pass, 24 Oct 2026).

python3 nox/scout/v3/compare_cages.py <before result.json> <after result.json> <out dir>
Writes compare.png (iso, side, top for each) and compare.html (the picture and the counts).
"""
import base64, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

A, B, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
CORE = [((-46.85, -30.6, 9.6), (54.75, 30.6, 18.5), "#8899aa"),          # mount
        ((-50, -30, -44.5), (50, 30, 9.6), "#7d8a99"),                     # receiver + cartridge envelope (E)
        ((-75, -75, 18.5), (75, 75, 22.0), "#3a3a3a")]                     # plate (E, picture only)
MOTORS = [(54.1, 54.1), (54.1, -54.1), (-54.1, 54.1), (-54.1, -54.1)]


def box_edges(lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return [(c[i], c[j]) for i in range(8) for j in range(i + 1, 8) if np.sum(c[i] != c[j]) == 1]


def draw(ax, R, view):
    for lo, hi, col in CORE:
        for p, q in box_edges(lo, hi): ax.plot(*zip(p, q), color=col, lw=0.8)
    t = np.linspace(0, 2 * np.pi, 60)
    for x, y in MOTORS: ax.plot(x + 44.45 * np.cos(t), y + 44.45 * np.sin(t), np.full_like(t, 41.2), color="#d9a400", lw=0.8)
    for a, b in R["rods"]: ax.plot(*zip(a, b), color="#202020", lw=1.3)
    ends = np.array([p for ab in R["rods"] for p in ab])
    ax.scatter(*ends.T, s=6, color="#4477aa")
    ax.view_init(*view); ax.set_box_aspect((1, 1, 0.55)); ax.set_axis_off()
    ax.set_xlim(-120, 120); ax.set_ylim(-120, 120); ax.set_zlim(-60, 75)


Ra, Rb = json.load(open(A)), json.load(open(B))
fig = plt.figure(figsize=(15, 10), dpi=110)
views = [("iso", (24, -55)), ("side", (0, -90)), ("top", (90, -90))]
for r, (R, lab) in enumerate(((Ra, "before"), (Rb, "after"))):
    for c, (vn, v) in enumerate(views):
        ax = fig.add_subplot(2, 3, 3 * r + c + 1, projection="3d"); draw(ax, R, v)
        ax.set_title(f"{lab}: {vn}", fontsize=11)
fig.tight_layout(); png = os.path.join(OUT, "compare.png"); fig.savefig(png); plt.close(fig)


def row(R):
    n = len(R["rods"]); L = sum(np.linalg.norm(np.subtract(b, a)) for a, b in R["rods"]) / 1000
    return n, R.get("pieces", "-"), R.get("printed_nodes", "-"), round(R["cage_mass"]), round(L, 2), round(R["contact_first"], 2), round(R["worst_index"], 2)


ra, rb = row(Ra), row(Rb)
img = base64.b64encode(open(png, "rb").read()).decode()
hdr = ["rods", "rod pieces", "printed nodes", "cage g (E)", "rod length m", "cage first mm", "worst index"]
trs = "".join(f"<tr><td>{h}</td><td>{a}</td><td>{b}</td></tr>" for h, a, b in zip(hdr, ra, rb))
html = f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Scout cage reduction</title><style>
:root{{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd}}
@media (prefers-color-scheme: dark){{:root:not([data-theme="light"]){{--bg:#151515;--fg:#eee;--mut:#aaa;--line:#333}}}}
:root[data-theme="dark"]{{--bg:#151515;--fg:#eee;--mut:#aaa;--line:#333}}
body{{background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif;margin:0;padding:16px;max-width:1100px}}
img{{width:100%;height:auto;background:#fff;border-radius:6px}} table{{border-collapse:collapse;margin:12px 0}}
td,th{{border-bottom:1px solid var(--line);padding:4px 12px;text-align:left}} p{{color:var(--mut)}}</style></head><body>
<h1>Scout v3 cage: before and after the rod reduction</h1>
<p>Same drone, same checks (every load case, cage first 3.0 mm, balance, edge probes no worse). Top row before, bottom row after.
Black = 3.0 carbon rods, blue dots = rod ends (printed nodes or core joints), grey = mount and receiver envelope, yellow = prop disks.</p>
<table><tr><th></th><th>before</th><th>after</th></tr>{trs}</table>
<img src="data:image/png;base64,{img}" alt="before and after cage, iso side top"></body></html>"""
open(os.path.join(OUT, "compare.html"), "w").write(html)
print("wrote", OUT, ra, rb)
