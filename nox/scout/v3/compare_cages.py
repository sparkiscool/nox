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
    lc = R.get("l_carbon", 0.0)
    for a, b in R["rods"]:
        pr = np.linalg.norm(np.subtract(b, a)) < lc
        ax.plot(*zip(a, b), color="#9aa6b2" if pr else "#202020", lw=3.2 if pr else 1.3)
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


def row(R, over=None):
    L = [np.linalg.norm(np.subtract(b, a)) for a, b in R["rods"]]
    lc = R.get("l_carbon", 0.0); carb = [x for x in L if x >= lc or not R.get("l_carbon")]
    pc, pp = over if over else (R.get("pieces", "-"), R.get("printed_nodes", "-"))
    return [len(L), len(carb), round(min(carb), 1), len(L) - len(carb), pc, pp, round(R["cage_mass"]), round(sum(L) / 1000, 2),
            round(R["contact_first"], 2), round(R["worst_index"], 2)]


over = tuple(int(x) for x in sys.argv[4].split(",")) if len(sys.argv) > 4 else None   # before: (carbon pieces, printed parts)
ra, rb = row(Ra, over), row(Rb)
img = base64.b64encode(open(png, "rb").read()).decode()
hdr = ["members", "carbon rods", "shortest carbon, mm", "printed struts / webs", "carbon pieces to cut (straight runs = 1)",
       "separate printed parts", "cage g (E)", "member length, m", "cage first, mm (F)", "worst load index (F)"]
trs = "".join(f"<tr><th scope=\"row\">{h}</th><td>{a}</td><td>{b}</td></tr>" for h, a, b in zip(hdr, ra, rb))
html = f"""<title>Scout cage reduction</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
/* one column: the counts, then the six views; carbon black, printed struts grey */
:root{{--bg:#f6f7f8;--fg:#16191d;--mut:#5b6470;--line:#d9dde2;--card:#ffffff;--acc:#2f6f9f;
 --body:"IBM Plex Sans",system-ui,sans-serif;--mono:"IBM Plex Mono",ui-monospace,monospace}}
@media (prefers-color-scheme: dark){{:root:not([data-theme="light"]){{--bg:#121417;--fg:#e8eaed;--mut:#9aa3ad;--line:#2b3036;--card:#1a1d21;--acc:#7fb3dc;color-scheme:dark}}}}
:root[data-theme="dark"]{{--bg:#121417;--fg:#e8eaed;--mut:#9aa3ad;--line:#2b3036;--card:#1a1d21;--acc:#7fb3dc;color-scheme:dark}}
body{{background:var(--bg);color:var(--fg);font:15px/1.5 var(--body);padding-block:20px 32px;
 padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px))}}
main{{max-width:1100px;margin:0 auto;display:grid;gap:16px}}
h1{{font-size:1.5rem;margin:0;text-wrap:balance}} p{{color:var(--mut);margin:0;max-width:68ch}}
.t{{overflow-x:auto}} table{{border-collapse:collapse;font-variant-numeric:tabular-nums;min-width:420px}}
th,td{{border-bottom:1px solid var(--line);padding:6px 14px;text-align:left}} td{{font-family:var(--mono)}}
thead th{{color:var(--mut);font-size:.8rem;letter-spacing:.06em;text-transform:uppercase}}
tbody th{{font-weight:500}} td:last-child{{color:var(--acc);font-weight:600}}
figure{{margin:0;background:#fff;border:1px solid var(--line);border-radius:6px;padding:8px}} img{{width:100%;height:auto;display:block}}
figcaption{{color:#555;font-size:.85rem;padding-top:6px}}
</style>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<main>
<h1>Scout v3 cage, before and after the rod reduction</h1>
<p>Spark 24 Oct: carbon only where its length earns it (33 mm break-even, DEFAULT), then fewest pieces. Same drone, same checks: every load case,
cage first 3.0 mm, balance, edge probes no worse. Members under 33 mm are now printed 6 x 6 PETG struts or 3 x 12 webs, part of their nodes.</p>
<div class="t"><table><thead><tr><th></th><th>before</th><th>after</th></tr></thead><tbody>{trs}</tbody></table></div>
<figure><img src="data:image/png;base64,{img}" alt="before and after cage, iso, side and top views">
<figcaption>Top row before, bottom row after. Black = 3.0 carbon rods, thick grey = printed struts, blue dots = joints, grey boxes = mount and receiver envelope, yellow = prop disks.</figcaption></figure>
</main>"""
open(os.path.join(OUT, "compare.html"), "w").write(html)
print("wrote", OUT, ra, rb)
