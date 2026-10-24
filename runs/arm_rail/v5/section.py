"""Rail v4 vs v5 section at the bolt (r 40, head A frame: X across, Y up) with the folded 5-conductor ribbon and the M5 head.
python3 runs/arm_rail/v5/section.py -> runs/arm_rail/v5/section.html (24 Oct 2026, swap arm system worker)"""
import os, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
R = 40.0; S = 9.0                                   # section radius, px per mm
CASES = [("v4", "runs/arm_rail/v4/arm_rail_A_v4_W.stl", dict(head=(61.0, 66.0), chan=(4.0, 3.2), bolt="M5 x 12")),
         ("v5", "runs/arm_rail/v5/arm_rail_A_v5_W.stl", dict(head=(57.75, 62.75), chan=(6.5, 5.0), bolt="M5 x 10"))]
RIB = dict(w=6.1, t=1.3, y0=62.75)                  # 5 conductors (V, G, S, H, 3.3 V), E; folded: two layers on the floor
X0, Y0, Y1 = -14.0, 44.0, 70.0                       # view window (mm)


def P(x, y, ox):
    return f"{ox + (x - X0) * S:.1f},{(Y1 - y) * S + 30:.1f}"


def panel(name, path, c, ox):
    m = trimesh.load(os.path.join(ROOT, path), force="mesh")
    sec = m.section(plane_origin=[0, 0, R], plane_normal=[0, 0, 1])
    out = [f'<text x="{ox}" y="18" class="h">Rail {name} at r {R:g} (bolt)</text>']
    for e in sec.entities:
        pts = sec.vertices[e.points]
        out.append('<polygon class="rail" points="' + " ".join(P(p[0], p[1], ox) for p in pts) + '"/>')
    out.append(f'<rect class="hub" x="{ox + (-11 - X0) * S:.1f}" y="{(Y1 - 54) * S + 30:.1f}" width="{22 * S:.1f}" height="{(54 - Y0) * S:.1f}"/>')
    out.append(f'<text x="{ox + (8 - X0) * S:.1f}" y="{(Y1 - 49) * S + 30:.1f}" class="s">hub v6 pad</text>')
    h0, h1 = c["head"]
    out.append(f'<rect class="bolt" x="{ox + (-4.25 - X0) * S:.1f}" y="{(Y1 - h1) * S + 30:.1f}" width="{8.5 * S:.1f}" height="{(h1 - h0) * S:.1f}"/>')
    tip = h0 - (10.0 if name == "v5" else 12.0)
    out.append(f'<rect class="bolt" x="{ox + (-2.5 - X0) * S:.1f}" y="{(Y1 - h0) * S + 30:.1f}" width="{5 * S:.1f}" height="{(h0 - tip) * S:.1f}"/>')
    for k in range(2):
        y = RIB["y0"] + k * RIB["t"]
        if name == "v4": y = max(y, h1 + k * RIB["t"])         # in v4 the ribbon rests on the head
        out.append(f'<rect class="rib" x="{ox + (-RIB["w"] / 2 - X0) * S:.1f}" y="{(Y1 - y - RIB["t"]) * S + 30:.1f}" width="{RIB["w"] * S:.1f}" height="{RIB["t"] * S:.1f}"/>')
    top = (h1 + 2 * RIB["t"]) if name == "v4" else (RIB["y0"] + 2 * RIB["t"])
    note = ([f"head Y {h0:g}..{h1:g} stands in the channel:", f"the folded ribbon tops at Y {top:.2f}, {top - 66.75:.2f} past the rail top,", "and 6.1 wide does not fit the 4.0 floor"]
            if name == "v4" else [f"head Y {h0:g}..{h1:g} under the channel floor:", f"the folded ribbon tops at Y {top:.2f}, {66.75 - top:.2f} under the rail top;", "3.75 PETG under the head (C)"])
    out.append(f'<text x="{ox}" y="{(Y1 - Y0) * S + 52:.1f}" class="s">{c["bolt"]}, channel {c["chan"][0]:g} floor / {c["chan"][1]:g} mouth (C)</text>')
    for k, t in enumerate(note):
        out.append(f'<text x="{ox}" y="{(Y1 - Y0) * S + 68 + 15 * k:.1f}" class="s">{t}</text>')
    return "".join(out)


W = (28 * S) + 40
svg = panel(*CASES[0], 10) + panel(*CASES[1], 10 + W + 60)
html = f"""<title>Rail v5 channel section</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
:root {{ --bg:#f6f5f1; --fg:#1d232a; --muted:#5d6670; --line:#c9ccd0; --card:#fff; --rail:#7d8a96; --hub:#c4ccd3; --bolt:#c47a1d; --rib:#2f6db3 }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --rail:#6b7884; --hub:#3b444d; --bolt:#e3a24a; --rib:#6aa5ec; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --rail:#6b7884; --hub:#3b444d; --bolt:#e3a24a; --rib:#6aa5ec; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:20px }}
main {{ max-width:1100px; margin:0 auto; display:grid; gap:16px }} h1 {{ font-size:1.4rem; margin:0; text-wrap:balance }} p {{ margin:0; color:var(--muted); max-width:70ch }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; min-width:0 }} .scroll {{ overflow-x:auto }}
svg .h {{ font-weight:600; font-size:14px; fill:var(--fg) }} svg .s {{ font-size:11px; fill:var(--muted) }}
.rail {{ fill:var(--rail) }} .hub {{ fill:var(--hub) }} .bolt {{ fill:var(--bolt); opacity:.9 }} .rib {{ fill:var(--rib); stroke:var(--card); stroke-width:1 }}
</style>
<main><header><h1>Rail v5 channel section</h1><p>Nox swap arm rails, section across the rail at the r 40 bolt, to scale ({S:g} px per mm). Blue: the head ribbon (5 conductors, ~6.1 x 1.3, E) folded on itself. Orange: the M5 socket head and shank. Grey below: the hub v6 pad.</p></header>
<section class="card"><div class="scroll"><svg viewBox="0 0 {2 * W + 80:.0f} {(Y1 - Y0) * S + 110:.0f}" width="{2 * W + 80:.0f}" height="{(Y1 - Y0) * S + 110:.0f}" role="img" aria-label="rail v4 and v5 sections">{svg}</svg></div></section>
<section class="card"><p>Source: <code>runs/arm_rail/v5/section.py</code>; meshes <code>runs/arm_rail/v4/</code>, <code>runs/arm_rail/v5/</code>; numbers <code>nox/dimensions.md</code> "System box swap: arm", Rails A / B v5.</p></section></main>
"""
open(os.path.join(HERE, "section.html"), "w").write(html)
print("->", os.path.join(HERE, "section.html"))
