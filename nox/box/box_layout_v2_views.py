"""Box structure v2 layout views: the four layout variants side by side, drawn from their concept meshes.

Two sections per variant, to scale in W (box-arm.md: X across, Y along the axle from the stub face, Z up):
  - across the axle at Y = Y_C (the grip line): the ring of bays, the arm, the catch, the side sheets;
  - along the axle at X = 0: stub to front sheet, the drone insert lanes under the -Y funnel panel.
Each part is sliced with trimesh; boxes (sheets, PSU) are cut as rectangles. The System line of each variant
comes from its concept_check.json (system_check). Run after box_layout_v2.py:

  python3 nox/box/box_layout_v2_views.py  -> runs/box_system/v2/layout_views.html
"""
import html, json, os, sys

import numpy as np
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
from box_layout_v2 import Y_C  # noqa: E402

V2 = os.path.join(ROOT, "runs", "box_system", "v2")
VARIANTS = [  # dir, name, what
    ("layout", "A: alternating at R 113 (DEFAULT)", "Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210"),
    ("layout_clip_bottom", "B: Overseer at the bottom, clipped", "4 Overseer bays together at the bottom, receivers clipped"),
    ("layout_bottom", "B0: Overseer at the bottom, unclipped", "the same without the clip: neighbouring receivers overlap"),
    ("layout_R128_bottom", "C: bottom slots at R 128", "Overseer at the bottom on a larger radius; the arm strokes change"),
]
GROUPS = [  # (css class, label, test on the part name)
    ("env", "bay body envelope (design space)", lambda n: "body envelope" in n),
    ("rec", "receiver (latch)", lambda n: n.endswith("receiver")),
    ("car", "cartridge (pack)", lambda n: n.endswith("cartridge")),
    ("arm", "arm: hub, motors, parked heads", lambda n: n.startswith("arm:") and "lane" not in n),
    ("lane", "arm: drone insert lanes", lambda n: "lane" in n),
    ("cat", "catch (funnel panels, hook latch)", lambda n: n.startswith("catch")),
    ("dro", "drone receivers + path (keep-out)", lambda n: n.startswith("drone")),
    ("ctx", "mast (v1), PSU", lambda n: n.startswith(("mast", "PSU"))),
    ("cage", "clear sheets", lambda n: n.startswith("sheet")),
]


def group(name):
    for cls, _, f in GROUPS:
        if f(name):
            return cls
    return "ctx"


def cut(part, axis, at):
    """polylines (in the plane's 2D coords) of one part cut at coordinate `at` on `axis` (0 = X, 1 = Y)."""
    keep = [i for i in range(3) if i != axis]
    if part.get("box"):
        b = part["box"]
        lo, hi = b[2 * axis], b[2 * axis + 1]
        if not lo <= at <= hi:
            return []
        (a0, a1), (c0, c1) = (b[2 * keep[0]], b[2 * keep[0] + 1]), (b[2 * keep[1]], b[2 * keep[1] + 1])
        return [np.array([[a0, c0], [a1, c0], [a1, c1], [a0, c1], [a0, c0]])]
    m = trimesh.load(os.path.join(ROOT, part["mesh"]), force="mesh")
    n = np.zeros(3); n[axis] = 1.0
    o = np.zeros(3); o[axis] = at
    s = m.section(plane_origin=o, plane_normal=n)
    if s is None:
        return []
    return [d[:, keep] for d in s.discrete]


def svg(parts, axis, at, frame, title):
    (x0, x1), (z0, z1) = frame
    pad = 14.0
    w, h = x1 - x0 + 2 * pad, z1 - z0 + 2 * pad
    out = [f'<svg viewBox="{x0 - pad:.0f} {-(z1 + pad):.0f} {w:.0f} {h:.0f}" role="img" aria-label="{html.escape(title)}">']
    order = [g[0] for g in GROUPS]
    for cls in order:
        for p in parts:
            if group(p["name"]) != cls:
                continue
            for d in cut(p, axis, at):
                pts = " ".join(f"{a:.1f},{-c:.1f}" for a, c in d)
                out.append(f'<polygon class="{cls}" points="{pts}"><title>{html.escape(p["name"])}</title></polygon>')
    # axle and scale bar
    if axis == 1:
        out.append(f'<circle class="axle" cx="0" cy="0" r="4"/>')
    else:
        out.append(f'<line class="axle" x1="0" y1="0" x2="{x1:.0f}" y2="0"/>')
        out.append(f'<line class="grip" x1="{Y_C:.0f}" y1="{-z1:.0f}" x2="{Y_C:.0f}" y2="{-z0:.0f}"/>')
    sx, sz = x0 + 6, z0 + 4
    out.append(f'<line class="bar" x1="{sx:.0f}" y1="{-sz:.0f}" x2="{sx + 100:.0f}" y2="{-sz:.0f}"/>')
    out.append(f'<text class="lbl" x="{sx:.0f}" y="{-sz - 5:.0f}">100 mm</text>')
    out.append("</svg>")
    return "".join(out)


def main(out=os.path.join(V2, "layout_views.html")):
    cards = []
    for d, name, what in VARIANTS:
        A = json.load(open(os.path.join(V2, d, "concept.assembly.json")))
        chk = json.load(open(os.path.join(V2, d, "concept_check.json")))
        parts = A["parts"]
        across = svg(parts, 1, Y_C, ((-215, 215), (-245, 245)), f"{name}: section across the axle at Y {Y_C:g}")
        along = svg(parts, 0, 0.0, ((-75, 355), (-245, 245)), f"{name}: section along the axle at X 0")
        fit = chk.get("fit", {})
        pairs = [f"{html.escape(p['a'])} / {html.escape(p['b'])}: {p['overlap_mm3']:.1f} mm³"
                 for p in fit.get("pairs", []) if not p.get("pass_", True)]
        summ = chk["summary"]
        ext = summ.split("extents ")[1].split(" mm")[0] if "extents " in summ else "?"
        vol = summ.split("bbox ")[1].split(" L")[0] if "bbox " in summ else "?"
        acc = "access " + summ.split("access ")[1] if "access " in summ else ""
        li = "".join(f"<li>{p}</li>" for p in pairs) or "<li>none</li>"
        cards.append(f'''<section class="var" id="{d.replace('_', '-')}">
<h2>{html.escape(name)}</h2><p class="what">{html.escape(what)}</p>
<dl><div><dt>Box</dt><dd>{ext} mm, {vol} L (E)</dd></div><div><dt>Swap paths</dt><dd>{html.escape(acc)}</dd></div></dl>
<div class="views"><figure>{across}<figcaption>Across the axle at Y {Y_C:g} (grip line), looking from the front</figcaption></figure>
<figure>{along}<figcaption>Along the axle at X 0, stub face at Y 0 on the left; dashed line = grip line</figcaption></figure></div>
<h3>Fit clashes</h3><ul class="clash">{li}</ul></section>''')
    legend = "".join(f'<li><span class="sw {c}"></span>{html.escape(l)}</li>' for c, l, _ in GROUPS)
    page = f'''<title>Box v2 Layouts</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
/* layout: one column of variant sheets, each two to-scale sections side by side, like a drawing set */
:root {{ --bg:#f3f5f6; --paper:#ffffff; --ink:#18232b; --mute:#5d6b75; --rule:#d3dadf;
  --env:#c9d8c3; --rec:#2f6f9f; --car:#e0a43a; --arm:#7d7f8a; --lane:#c2453a; --cat:#6a4fa0; --dro:#d98aa8; --ctx:#9aa7ae; --cage:#7fb7c9;
  --f-head:"Archivo",system-ui,sans-serif; --f-body:"IBM Plex Sans",system-ui,sans-serif; --f-mono:"IBM Plex Mono",ui-monospace,monospace; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#11171b; --paper:#182126; --ink:#e4eaee; --mute:#9aa8b1; --rule:#2c3840;
  --env:#3f5a3a; --rec:#5ea3d6; --car:#e8b55a; --arm:#8d909c; --lane:#e0675c; --cat:#9b80d1; --dro:#d98aa8; --ctx:#66747c; --cage:#4f8fa3; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#11171b; --paper:#182126; --ink:#e4eaee; --mute:#9aa8b1; --rule:#2c3840;
  --env:#3f5a3a; --rec:#5ea3d6; --car:#e8b55a; --arm:#8d909c; --lane:#e0675c; --cat:#9b80d1; --dro:#d98aa8; --ctx:#66747c; --cage:#4f8fa3; color-scheme:dark }}
body {{ background:var(--bg); color:var(--ink); font-family:var(--f-body); line-height:1.5;
  padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:24px 48px }}
main {{ max-width:1180px; margin:0 auto; display:grid; gap:28px }}
h1 {{ font-family:var(--f-head); font-size:1.7rem; margin:0; text-wrap:balance }}
h2 {{ font-family:var(--f-head); font-size:1.2rem; margin:0 }}
h3 {{ font-size:.8rem; text-transform:uppercase; letter-spacing:.06em; color:var(--mute); margin:12px 0 4px }}
.lede {{ max-width:68ch; margin:6px 0 0; color:var(--mute) }}
.legend {{ list-style:none; padding:0; margin:0; display:flex; flex-wrap:wrap; gap:6px 18px; font-size:.85rem }}
.legend li {{ display:flex; align-items:center; gap:6px }}
.sw {{ width:14px; height:14px; border-radius:2px; border:1px solid var(--rule); display:inline-block }}
.sw.env {{ background:var(--env) }} .sw.rec {{ background:var(--rec) }} .sw.car {{ background:var(--car) }} .sw.arm {{ background:var(--arm) }}
.sw.lane {{ background:var(--lane) }} .sw.cat {{ background:var(--cat) }} .sw.dro {{ background:var(--dro) }} .sw.ctx {{ background:var(--ctx) }} .sw.cage {{ background:var(--cage) }}
.var {{ background:var(--paper); border:1px solid var(--rule); border-radius:6px; padding:18px; min-width:0 }}
.what {{ margin:2px 0 10px; color:var(--mute) }}
dl {{ display:flex; flex-wrap:wrap; gap:6px 28px; margin:0 0 10px; font-family:var(--f-mono); font-size:.85rem; font-variant-numeric:tabular-nums }}
dl div {{ display:flex; gap:8px }} dt {{ color:var(--mute) }} dd {{ margin:0 }}
.views {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(300px, 1fr)); gap:16px }}
figure {{ margin:0; min-width:0 }}
figure svg {{ width:100%; height:auto; display:block; background:var(--bg); border-radius:4px }}
figcaption {{ font-size:.8rem; color:var(--mute); margin-top:4px }}
svg polygon {{ stroke-width:.8; stroke-linejoin:round }}
.env {{ fill:var(--env); fill-opacity:.55; stroke:var(--env) }} .rec {{ fill:var(--rec); stroke:var(--rec) }} .car {{ fill:var(--car); fill-opacity:.85; stroke:var(--car) }}
.arm {{ fill:var(--arm); fill-opacity:.12; stroke:var(--arm); stroke-dasharray:2 2 }} .lane {{ fill:var(--lane); fill-opacity:.25; stroke:var(--lane); stroke-dasharray:3 2 }}
.cat {{ fill:var(--cat); fill-opacity:.8; stroke:var(--cat) }} .dro {{ fill:var(--dro); fill-opacity:.3; stroke:var(--dro) }}
.ctx {{ fill:var(--ctx); fill-opacity:.6; stroke:var(--ctx) }} .cage {{ fill:var(--cage); stroke:var(--cage) }}
.axle {{ fill:var(--ink); stroke:var(--ink); stroke-width:.8; stroke-dasharray:6 3 }}
.grip {{ stroke:var(--mute); stroke-width:.8; stroke-dasharray:4 3; fill:none }}
.bar {{ stroke:var(--ink); stroke-width:2 }} .lbl {{ fill:var(--ink); font:9px var(--f-mono) }}
.clash {{ margin:0; padding-left:18px; font-family:var(--f-mono); font-size:.82rem; overflow-wrap:anywhere }}
</style>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@600;700&family=IBM+Plex+Mono&family=IBM+Plex+Sans:wght@400;500&display=swap">
<main>
<header><h1>Box structure v2: four layouts</h1>
<p class="lede">Nine bays (4 Overseer, 5 Scout) on the swap arm's ring, grip line Y {Y_C:g}, each variant cut through its own concept meshes (to scale, mm, W frame). Bay bodies are design spaces for the topology run, not final shapes. A is the DEFAULT. The clash lists come from system_check; the catch's −Y panel notch (panel v9, hub) and funnel v8 clear the catch pairs.</p></header>
<ul class="legend">{legend}</ul>
{"".join(cards)}
</main>'''
    open(out, "w").write(page)
    print(out, f"{os.path.getsize(out) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
