"""Visual for the box structure v2 coarse system topology (runs/box_system/v2/topo/): the five rendered views of the
0.5 density surface, the per-case compliance and stub tilt from result.json, and the cut proposal. One self-contained
page (views embedded):  python3 nox/box/box_system_topo_v2_page.py -> runs/box_system/v2/topo/topo_views.html"""
import base64, html, json, os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
D = os.path.join(ROOT, "runs", "box_system", "v2", "topo")
VIEWS = [("view_iso", "Iso, from the front right"), ("view_front", "Front (looking along −Y): ring, mast, top frame, feet"),
         ("view_side", "Side (looking along −X)"), ("view_top", "Top"), ("view_back_iso", "Iso from behind: the mast on the rear sheet")]
CUTS = [  # piece, qty, envelope X / Y / Z (W mm), joint, mass (E)
    ("Mast lower", 1, "−64..44 / −56..−4 / −113..0", "bolted to the rear sheet; lap to the upper at Z −16..0", "0.38 kg"),
    ("Mast upper + stub face", 1, "−64..44 / −56..−4 (+ Ø90 plate Y −8..0) / −10..163", "top lands on the top frame", "0.65 kg"),
    ("Hub arch", 1, "−44..116 / 4..68 / 107..179", "to the mast top and the +X rail", "0.31 kg"),
    ("Top front beam", 3, "−196..−66, −70..70, 66..196 / 280..312 / 95..151", "laps at X ±66..70", "0.31 kg total"),
    ("Top side rail +X", 2, "60..200 / 160..312 and 55..175 / 110..180", "joint Y 160..176", "0.95 kg"),
    ("Top side rail −X", 2, "−200..−60 / same", "joint Y 144..160", "0.41 kg"),
    ("Overseer bay bodies 0, −180", 2, "±(108..194) / 71..259 / −50..50", "end wall to the side sheet; out +Y", "1.2 kg each"),
    ("Overseer bay bodies −60, −120", 2, "e.g. 29..137 / 71..259 / −187..−79", "out +Y", "0.9 kg each"),
    ("Scout bay bodies 30, −30, −90, −150, −210", 5, "e.g. 30: 79..173 / 106..216 / 29..126", "30 and −210 carry the seat posts; out +Y", "0.35 kg each"),
    ("Ring links", 8, "arc r 150..200, Y 60..270", "lap tabs at ±15°", "0.1..0.18 kg"),
    ("Base: bottom beam + 2 feet", 1, "−148..100 / 76..252 / −229..−157, split at X 0", "to the bay −90 body; TPU pads", "0.45 kg"),
    ("Side-sheet tabs", 4, "X ±(192..200)", "U-clip", "0.05 kg each"),
    ("Datum ring (added, not from the field)", 3, "r 115..200, Y 74..98, 3 x 80°", "bolted to the mast upper + hub arch; 9 locating seats", "0.6 kg"),
]


def main():
    r = json.load(open(os.path.join(D, "result.json")))
    imgs = "".join(f'<figure><img alt="{html.escape(t)}" src="data:image/png;base64,{base64.b64encode(open(os.path.join(D, v + ".png"), "rb").read()).decode()}">'
                   f'<figcaption>{html.escape(t)}</figcaption></figure>' for v, t in VIEWS)
    rows = "".join(f"<tr><td>{html.escape(k)}</td><td>{r['ratio_to_full_solid'][k]:.1f}</td>"
                   f"<td>{max(abs(x) for x in r['stub_rotation'][k]['rot_mrad']):.2f}</td></tr>" for k in r["ratio_to_full_solid"])
    cuts = "".join(f"<tr><td>{html.escape(a)}</td><td>{b}</td><td>{html.escape(c)}</td><td>{html.escape(d)}</td><td>{html.escape(e)}</td></tr>" for a, b, c, d, e in CUTS)
    page = f'''<title>Box v2 Topology</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
/* layout: a run report, views in a grid, then the two tables */
:root {{ --bg:#f4f6f7; --paper:#fff; --ink:#18232b; --mute:#5d6b75; --rule:#d3dadf; --acc:#1f4e8c; --warn:#a4462d;
  --f-head:"Archivo",system-ui,sans-serif; --f-body:"IBM Plex Sans",system-ui,sans-serif; --f-mono:"IBM Plex Mono",ui-monospace,monospace }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#11171b; --paper:#182126; --ink:#e4eaee; --mute:#9aa8b1; --rule:#2c3840; --acc:#7fb0f0; --warn:#e88a6c; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#11171b; --paper:#182126; --ink:#e4eaee; --mute:#9aa8b1; --rule:#2c3840; --acc:#7fb0f0; --warn:#e88a6c; color-scheme:dark }}
body {{ background:var(--bg); color:var(--ink); font-family:var(--f-body); line-height:1.5; padding-block:24px 48px;
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)) }}
main {{ max-width:1180px; margin:0 auto; display:grid; gap:22px }}
h1 {{ font-family:var(--f-head); font-size:1.7rem; margin:0; text-wrap:balance }} h2 {{ font-family:var(--f-head); font-size:1.15rem; margin:0 0 8px }}
p {{ max-width:72ch; margin:4px 0 }} .mute {{ color:var(--mute) }} .warn {{ color:var(--warn) }}
.views {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(280px, 1fr)); gap:14px }}
figure {{ margin:0; background:#fff; border:1px solid var(--rule); border-radius:6px; overflow:hidden; min-width:0 }}
figure img {{ width:100%; height:auto; display:block }} figcaption {{ font-size:.8rem; color:#5d6b75; padding:6px 8px }}
section {{ background:var(--paper); border:1px solid var(--rule); border-radius:6px; padding:16px; min-width:0 }}
.wrap {{ overflow-x:auto }} table {{ border-collapse:collapse; width:100%; font-size:.86rem }}
th, td {{ text-align:left; padding:5px 8px; border-bottom:1px solid var(--rule); vertical-align:top }}
td:nth-child(n+2) {{ font-family:var(--f-mono); font-variant-numeric:tabular-nums }}
ul {{ margin:4px 0; padding-left:20px }}
</style>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@600;700&family=IBM+Plex+Mono&family=IBM+Plex+Sans:wght@400;500&display=swap">
<main>
<header><h1>Box structure v2: coarse system topology</h1>
<p class="mute">One solve over the whole box (h {r["h"]:g}, min member {r["min_member_mm"]:g} mm, {r["gen_cm3"]:g} cm³ target, {r["generated_cm3"]:.0f} cm³ generated, {r["mass_g"]/1000:.2f} kg PETG for the generated material alone, E), 14 load cases, {r["iterations"]} iterations. Blue = density 0.5 surface; orange = the catch; grey = the receivers. At 1500 cm³ only bays 30 and −210 joined the main body (M), so 3000 cm³ is the record.</p>
<p class="warn">Round 1. The sheets were modelled as 8 mm PETG held rigid at their bottom edge, so they carry much of the box: 12 cm² of floor contact against 576 cm² on the sheets (M). Real sheets are 3 mm acrylic. Round 2 reruns on the final layout with the sheets as cladding before any piece is cut.</p></header>
<div class="views">{imgs}</div>
<section><h2>Per case: compliance (x the full solid) and stub rotation</h2><div class="wrap"><table><tr><th>Case</th><th>Compliance / solid</th><th>Stub rot. max mrad</th></tr>{rows}</table></div>
<p class="mute">Stub tilt at 57 N + 5.9 N·m: 0.37 mrad against the 10 mrad target (M on the h 8 model).</p></section>
<section><h2>Cut proposal (from the density field; envelopes W mm, M on the h 8 field; mass E)</h2><div class="wrap"><table><tr><th>Piece</th><th>Qty</th><th>Envelope X / Y / Z</th><th>Joint</th><th>Mass</th></tr>{cuts}</table></div>
<ul><li>The Overseer bay bodies are 188 long (Y 71..259, M), over the 180 cube: diagonal print or a split.</li>
<li>The datum ring is added to meet the ±0.15° bay-to-mast datum; the field gives no direct path.</li></ul></section>
</main>'''
    out = os.path.join(D, "topo_views.html"); open(out, "w").write(page); print(out, f"{os.path.getsize(out)/1e6:.2f} MB")


if __name__ == "__main__":
    main()
