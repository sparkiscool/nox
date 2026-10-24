"""Visual for the box structure v2 coarse system topology (runs/box_system/v2/topo_r2/): the five rendered views of the
0.5 density surface, the per-case compliance and stub tilt from result.json, and the cut proposal. One self-contained
page (views embedded):  python3 nox/box/box_system_topo_v2r2_page.py -> runs/box_system/v2/topo_r2/topo_views.html"""
import base64, html, json, os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
D = os.path.join(ROOT, "runs", "box_system", "v2", "topo_r2")
VIEWS = [("view_iso", "Iso, from the front right"), ("view_front", "Front (looking along −Y): ring, mast, top frame, feet"),
         ("view_side", "Side (looking along −X)"), ("view_top", "Top"), ("view_back_iso", "Iso from behind: the mast on the rear sheet")]
CUTS = [  # piece, qty, envelope X / Y / Z (W mm), joint, mass (E, field cm3 x 1.27 g/cm3, 3000 cm3 run)
    ("Mast + stub face (Ø90 plate Y -8..0)", 3, "−72..72 / −70..10 / −233..127, split in Z", "lap joints at the Z cuts; rides Y −25..10 over the board stack", "0.93 kg total"),
    ("Rear frame +X / −X", 4, "72..208 and −192..−72 / −70..10 / −233..127, 2 each in Z", "laps; electronics bosses Ø14 at the PSU / charger corners", "0.43 kg total"),
    ("Top frame halves +X / −X", 4, "0..136 and −128..0 / 10..266 / 111..175, 2 each in Y", "lap at Y 138; 8 funnel seat posts Ø16", "0.84 kg total"),
    ("Seat-post bridge", 1, "−56..72 / 266..290 / 111..175", "bolted to the top frame", "0.10 kg"),
    ("Scout bay bodies 30, −30, −90, −150, −210", 5, "e.g. 30: 104..200 / 58..218 / 39..135 (all ≤ 160 long)", "4 M3 inserts in Ø12 bosses; out +Y", "0.17..0.36 kg each"),
    ("Overseer bay bodies 0, −60, −120, −180", 8, "e.g. 0: 144..208 / 58..258 / −49..55 (200 long: over the 180 cube), 2 halves each split in Y at 158", "6 M3 inserts; halves lap at Y 158", "0.21..0.31 kg per body"),
    ("Hub links (ring links, arch bits)", 6, "−184..200 / 10..218 / −153..111", "to the mast and bay bodies", "0.37 kg total"),
    ("Base feet", 3, "−96..96 / 10..58 / −177..−153", "a low tie; the 7 floor patches (9 cm², M) sit on the bay −60 / −120 / −90 bodies and the mast: TPU pads there", "0.02 kg"),
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
<header><h1>Box structure v2: coarse system topology, round 2</h1>
<p class="mute">One solve over the whole box (h {r["h"]:g}, min member {r["min_member_mm"]:g} mm, {r["gen_cm3"]:g} cm³ target, {r["generated_cm3"]*1.27/1000:.2f} kg PETG for the generated material at 1.27 g/cm³, {r["mass_g_PETG"]/1000:.2f} kg with the drawn bodies, E), 18 load cases, {r["iterations"]} iterations. Blue = density 0.5 surface; orange = the catch; grey = the receivers. Sheets are cladding (not in the model); the floor is the only support. 3000 cm³ chosen: 2000 cm³ is far too soft (up to 1180 × the full solid), 4000 cm³ is about 2 × stiffer for 1.2 kg more.</p>
<p class="warn">Round 2: the structure stands on its own on about 9 cm² of floor (M) and keeps the Overseer bay bodies at 200 long (over the 180 cube).</p></header>
<div class="views">{imgs}</div>
<section><h2>Per case: compliance (x the full solid) and stub rotation</h2><div class="wrap"><table><tr><th>Case</th><th>Compliance / solid</th><th>Stub rot. max mrad</th></tr>{rows}</table></div>
<p class="mute">Stub tilt at 57 N + 5.9 N·m: 0.41 mrad against the 10 mrad target (C on the h 8 model).</p></section>
<section><h2>Cut proposal (from the density field; envelopes W mm, M on the h 8 field; mass E)</h2><div class="wrap"><table><tr><th>Piece</th><th>Qty</th><th>Envelope X / Y / Z</th><th>Joint</th><th>Mass</th></tr>{cuts}</table></div>
<ul><li>The Overseer bay bodies are 200 long (Y 58..258), over the 180 cube: split in Y or diagonal print.</li>
<li>The field builds its own bay-to-mast paths (all 9 bays joined to the stub body without the floor). The datum ring (+1.1 kg drawn) cut the worst bay rotation about the axle from 1.8 to 1.3 mrad (C), under the ±0.15° = 2.6 mrad datum either way, so it is not added.</li></ul></section>
</main>'''
    out = os.path.join(D, "topo_views.html"); open(out, "w").write(page); print(out, f"{os.path.getsize(out)/1e6:.2f} MB")


if __name__ == "__main__":
    main()
