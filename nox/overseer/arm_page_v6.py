"""Overseer front arm v6 concept page: v6 (diagonals + nodes + solid shoulder wedge), the lighter tri10 option and v5d
as it is, side by side with the modal what-if numbers and masses (images inlined; style from arm_page_v5d.py).

python3 nox/overseer/arm_page_v6.py  -> runs/overseer_arms/v6/arms.html
"""
import base64, json, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
F = os.path.join(ROOT, "runs", "overseer_arm_front")
OUT = os.path.join(ROOT, "runs", "overseer_arms", "v6", "arms.html")
src = open(os.path.join(HERE, "arm_page_v5d.py")).read()
STYLE = src[src.index("<style>"):src.index("</style>") + 8].replace("{{", "{").replace("}}", "}")


def img(path, alt):
    return f'<img src="data:image/png;base64,{base64.b64encode(open(path, "rb").read()).decode()}" alt="{alt}" loading="lazy">'


w6 = json.load(open(os.path.join(F, "v6", "whatif_modes.json")))
wt = json.load(open(os.path.join(F, "v6", "tri10", "whatif_modes.json")))
m5 = json.load(open(os.path.join(F, "v6", "whatif_modes.json")))["rods_only"]
opts = [("v6 chosen: diagonals + nodes + solid shoulder wedge", w6["rods_and_nodes"], os.path.join(ROOT, "runs", "overseer_arms", "v6", "layout.png")),
        ("tri10: diagonals + nodes + 10 mm triangle walls", wt["rods_and_nodes"], os.path.join(ROOT, "runs", "overseer_arms", "v6", "layout_tri10.png")),
        ("v5d as it is (4 tubes, no diagonals)", {"modes": [{"Hz": 131.2}, {"Hz": 162.3}], "arm_g_grid": 242.4}, os.path.join(F, "v5d", "preview.png"))]
rows, figs = "", ""
for name, r, pic in opts:
    v, l = r["modes"][0]["Hz"], r["modes"][1]["Hz"]
    st = "ok" if min(v, l) >= 190 else "bad"
    rows += (f'<tr><td>{name}</td><td class="num">{v:.0f}</td><td class="num">{l:.0f}</td><td class="num">{r["arm_g_grid"]:.0f}</td>'
             f'<td><span class="pill {st}">{"meets 190" if st == "ok" else "under 190"}</span></td></tr>')
    figs += f'<figure>{img(pic, name)}<figcaption>{name}</figcaption></figure>'
html = f"""<title>Overseer Front Arm v6</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;600&family=IBM+Plex+Mono:wght@400;500&family=Archivo+Narrow:wght@600&display=swap">
{STYLE}
<main>
<header>
<h1>Overseer front arm v6</h1>
<p class="lede">Front v5d (4 x 6/4 carbon tubes) passed solid and strength but read 131 Hz vertical / 162 Hz lateral against 190 Hz: the tubes bend almost on their own. v6 ties them into a carbon truss: 12 x 3.0 diagonals, Warren, 3 panels on every face, printed nodes, and a solid shoulder wedge where the vertical shear crosses into the root.</p>
</header>
<section>
<h2>Options, modal what-if on the v5d arm (4 inserts, h 1.5, E)</h2>
<div class="tablewrap"><table>
<thead><tr><th>Option</th><th>Vertical Hz</th><th>Lateral Hz</th><th>Grid g</th><th>Target</th></tr></thead>
<tbody>{rows}</tbody>
</table></div>
<p class="note">Rods only, no new PETG: {m5['modes'][0]['Hz']:.0f} / {m5['modes'][1]['Hz']:.0f} Hz. The shoulder band (x 5..25) is the soft spot: stiffening it alone gave 260 Hz. Grid g is PETG + carbon on the FE grid, before the topology rerun moves material. The solid wedge costs about 9 g over tri10 for 5.5 Hz more margin (DEFAULT). Every rod is 50 to 71 mm (C), slides in from its root-side end (rodcheck on the setup: 16/16 clear).</p>
</section>
<section>
<h2>Pictures</h2>
<div class="pair">{figs}</div>
</section>
</main>
"""
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w").write(html)
print(OUT, len(html) // 1024, "KB")
