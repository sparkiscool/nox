"""Overseer arms v5d page: the rear arm as saved (pictures rendered from the exact print files), the v5c/v5d setup
(drawn bodies, tube chords, envelopes) and the check numbers, one self-contained HTML (images inlined).

python3 nox/overseer/arm_page_v5d.py  -> runs/overseer_arms/v5d/arms.html (published as the arms artifact)
"""
import base64, json, os
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
R = os.path.join(ROOT, "runs", "overseer_arm_rear", "v5d")
OUT = os.path.join(ROOT, "runs", "overseer_arms", "v5d", "arms.html")


def img(path, alt):
    b = base64.b64encode(open(path, "rb").read()).decode()
    return f'<img src="data:image/png;base64,{b}" alt="{alt}" loading="lazy">'


res = json.load(open(os.path.join(R, "result.json")))
band = json.load(open(os.path.join(R, "strength_band.json")))["worst"]
band45 = json.load(open(os.path.join(R, "strength_band_4.5.json")))["worst"]
modes = json.load(open(os.path.join(R, "modes.json")))
by = {}
for x in res["strength"]["rows"]:
    k = "cage crash" if x["case"].startswith("cage") else x["case"].split(" ")[0]
    by[k] = max(by.get(k, 0.0), max(x["inlayer"], x["across"]))
names = {"max": ("Thrust 21 N + yaw, 2 g", 0.25), "landing": ("Landing 6 g", 0.25), "crash": ("Own crash 35 g", 0.5),
         "handling": ("Handling 100 N", 0.5), "blow": ("Blow 150 N on the motor", 0.5), "cage crash": ("Cage crash (2354 N)", 0.5)}
rows = ""
for k, (label, tgt) in names.items():
    v = by.get(k, 0.0)
    st = "ok" if v <= tgt else ("warn" if v <= 1.0 else "bad")
    rows += f'<tr><td>{label}</td><td class="num">{v:.2f}</td><td class="num">{tgt:.2f}</td><td><span class="pill {st}">{"within" if st == "ok" else ("over target, under 1" if st == "warn" else "over 1")}</span></td></tr>'
f_ins = modes["inserts"]["modes"][0]["Hz"]

html = f"""<title>Overseer Arms v5d</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;600&family=IBM+Plex+Mono:wght@400;500&family=Archivo+Narrow:wght@600&display=swap">
<style>
/* layout: one column of plates, a spec strip, then the check table; carbon-and-PETG shop sheet */
:root {{ --bg: #f3f5f6; --panel: #ffffff; --fg: #1b2328; --muted: #5b6871; --line: #d5dce0; --accent: #0f6f8a;
  --ok: #2f7d4a; --warn: #a86b0c; --bad: #b3362b;
  --display: "Archivo Narrow", "Arial Narrow", sans-serif; --body: "IBM Plex Sans", system-ui, sans-serif; --mono: "IBM Plex Mono", ui-monospace, monospace; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg: #121719; --panel: #1b2226; --fg: #e3e9ec; --muted: #9aa8b0; --line: #2d373c; --accent: #5fb7d1; --ok: #6cc28a; --warn: #e0a64a; --bad: #ef7d70; color-scheme: dark }} }}
:root[data-theme="dark"] {{ --bg: #121719; --panel: #1b2226; --fg: #e3e9ec; --muted: #9aa8b0; --line: #2d373c; --accent: #5fb7d1; --ok: #6cc28a; --warn: #e0a64a; --bad: #ef7d70; color-scheme: dark }}
body {{ background: var(--bg); color: var(--fg); font-family: var(--body); font-size: 15px; line-height: 1.5; }}
main {{ max-width: 980px; margin: 0 auto; padding-block: 28px 48px; padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); display: grid; gap: 28px; }}
h1 {{ font-family: var(--display); font-size: 2.1rem; letter-spacing: .01em; margin: 0; text-wrap: balance; }}
h2 {{ font-family: var(--display); font-size: 1.3rem; margin: 0 0 10px; }}
.lede {{ color: var(--muted); max-width: 65ch; margin: 6px 0 0; }}
.spec {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 1px; background: var(--line); border: 1px solid var(--line); }}
.spec div {{ background: var(--panel); padding: 10px 12px; min-width: 0; }}
.spec dt {{ font-size: .72rem; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); }}
.spec dd {{ margin: 2px 0 0; font-family: var(--mono); font-variant-numeric: tabular-nums; }}
figure {{ margin: 0; background: var(--panel); border: 1px solid var(--line); }}
figure img {{ display: block; width: 100%; height: auto; background: #fff; }}
figcaption {{ padding: 8px 12px; font-size: .85rem; color: var(--muted); }}
.pair {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px; }}
.tablewrap {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; background: var(--panel); border: 1px solid var(--line); }}
th, td {{ text-align: left; padding: 8px 12px; border-bottom: 1px solid var(--line); }}
th {{ font-size: .72rem; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); font-weight: 600; }}
.num {{ font-family: var(--mono); font-variant-numeric: tabular-nums; }}
.pill {{ font-size: .78rem; padding: 2px 8px; border: 1px solid currentColor; white-space: nowrap; }}
.pill.ok {{ color: var(--ok); }} .pill.warn {{ color: var(--warn); }} .pill.bad {{ color: var(--bad); }}
.note {{ max-width: 65ch; color: var(--muted); margin: 10px 0 0; }}
.front {{ border-left: 3px solid var(--accent); padding: 4px 0 4px 14px; max-width: 65ch; }}
</style>
<main>
<header>
<h1>Overseer arms v5d</h1>
<p class="lede">Rear arm (arm 2, mirror arm 3) saved 27 Oct: one PETG print between the S500 plates, four 6/4 carbon tube chords in 6.2 bores, the cage spoke node beside the motor. The front arm goes back to its system pass (modes).</p>
</header>
<dl class="spec">
<div><dt>Mass</dt><dd>{res['mass_g']:.1f} g PETG (M)</dd></div>
<div><dt>Tubes</dt><dd>4 x 6/4, 63.1 mm</dd></div>
<div><dt>Solid check</dt><dd>PASS, both files</dd></div>
<div><dt>First mode</dt><dd>{f_ins:.0f} Hz on inserts</dd></div>
<div><dt>Rods</dt><dd>install check OK</dd></div>
<div><dt>Print</dt><dd>top face down, 3376 mm²</dd></div>
</dl>
<section>
<h2>Print files</h2>
<div class="pair">
<figure>{img(os.path.join(R, 'preview_overseer_arm_rear_v5d.png'), 'Rear arm v5d as printed')}<figcaption>print/overseer_arm_rear_v5d.stl (arm 2)</figcaption></figure>
<figure>{img(os.path.join(R, 'preview_overseer_arm_rear_v5d_mirror.png'), 'Rear arm v5d mirror as printed')}<figcaption>print/overseer_arm_rear_v5d_mirror.stl (arm 3)</figcaption></figure>
</div>
</section>
<section>
<h2>Setup in the drone</h2>
<figure>{img(os.path.join(ROOT, 'runs', 'overseer_arms', 'v5c', 'layout.png'), 'Arm setup v5c/v5d in the drone frame')}<figcaption>Drawn bodies and carbon chords of all four arms on the plates: full-length tube sleeves, flanges from the root to the tube shoulders, motor pads. v5d adds a 2.5 wall closing each tube's blind tip end.</figcaption></figure>
</section>
<section>
<h2>Strength, worst index per case</h2>
<div class="tablewrap"><table>
<thead><tr><th>Case</th><th>Index</th><th>Target</th><th>Status</th></tr></thead>
<tbody>{rows}</tbody>
</table></div>
<p class="note">The cage crash worst ({band['arm crash']['index']:.2f}) sits 3.0 to 3.4 mm from the top plate's corner, the root joint Spark said to assume is good; beyond 4.5 mm of the plates the arm's worst is {band45['arm crash']['index']:.2f} at the shoulder socket. Every case stays under 1.</p>
</section>
<section class="front">
<h2>Front arm</h2>
<p>v5d: 228 g, solid PASS, strength 0.62, but first modes 131 Hz vertical and 162 Hz lateral against 190 Hz. Next: a carbon space frame, 12 diagonal 3.0 rods tying the four chords (modal what-if 193 to 200 Hz).</p>
</section>
</main>
"""
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w").write(html)
print(OUT, len(html) // 1024, "KB")
