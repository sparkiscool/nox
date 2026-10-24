"""Hub v7 system pass, the detent tower options in plan at the axle belt (Y 56..62), to scale: run 3's single column (with its flare),
the rejected legs under P and A, and the chosen column inside the loop; plus the swap-vs-volume line from runs v6r2 / v6r3.
python3 runs/hub/v7-concept/section.py -> runs/hub/v7-concept/section.html (24 Oct 2026, swap arm worker)"""
import os, sys, math, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox", "box")); sys.path.insert(0, os.path.join(ROOT, "nox"))
os.environ.setdefault("NOX_ARM", "v7")
import arm_v7_layout as S7; S7.use()
import hub_v6 as HV, arm_v5_layout as V, detent as DT
H = HV.hub_v6_parts(); D, _ = H["detent"]; Q = DT.chosen()["Q"]; ax = HV.MOTORS6["axle"]
STR = [poly for _, poly in V.loop_strands(ax["xc"], ax["zc"])]
S = 6.0; X0, Z0, X1, Z1 = -8, -66, 72, -2                  # window (mm): X right, Z down


def circle(c, r, n=40): return [(c[0] + r * math.cos(2 * math.pi * k / n), c[1] + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def panel(ox, title, solids, verdict, bad=()):
    t = lambda x, z: f"{ox + (x - X0) * S:.1f},{40 + (z - Z0) * S:.1f}"
    W, Hh = (X1 - X0) * S, (Z1 - Z0) * S
    cid = f"p{int(ox)}"
    o = [f'<text x="{ox}" y="18" class="h">{title}</text>', f'<text x="{ox}" y="32" class="s">{verdict}</text>',
         f'<clipPath id="{cid}"><rect x="{ox}" y="40" width="{W:.0f}" height="{Hh:.0f}"/></clipPath><g clip-path="url(#{cid})">',
         f'<rect x="{ox}" y="40" width="{W:.0f}" height="{Hh:.0f}" class="frame"/>']
    o.append('<polygon class="p80" points="' + " ".join(t(*p) for p in circle((0, 0), 27.5, 96)) + '"/>')
    o.append('<polygon class="p16" points="' + " ".join(t(*p) for p in circle((ax["xc"], ax["zc"]), 6.5)) + '"/>')
    for poly in STR: o.append('<polygon class="belt" points="' + " ".join(t(*p) for p in poly) + '"/>')
    for k, poly in enumerate(solids):
        o.append(f'<polygon class="{"bad" if k in bad else "body"}" points="' + " ".join(t(*p) for p in poly) + '"/>')
    for k, lab in (("Q", "Q"), ("P", "pivot P"), ("A", "anchor A")):
        x, z = Q if k == "Q" else D[k]
        o.append(f'<circle cx="{ox + (x - X0) * S:.1f}" cy="{40 + (z - Z0) * S:.1f}" r="2.2" class="pt"/><text x="{ox + (x - X0) * S + 5:.1f}" '
                 f'y="{40 + (z - Z0) * S - 5:.1f}" class="s">{lab}</text>')
    o.append(f'<text x="{ox + 6}" y="{40 + Hh - 8:.0f}" class="s">80T sun (r 27.5)</text></g>')
    return "".join(o), W, Hh


col_r3 = circle(Q, 7.5 - 1.5 * 3 / 6)                        # run 3's flare at belt mid-height Y 59: r 7.5 -> 5 over Y 54..60
legs = [circle(Q, 5.0), circle(D["P"], 4.0), circle(D["A"], 4.0)]
p1, W, Hh = panel(10, "Run 3: one Ø10 column at Q, flared", [col_r3], "flare touches the strand (0.0 gap); sustained 1.08 above it (F)", bad=(0,))
p2, _, _ = panel(10 + W + 30, "Rejected: legs under P and A", legs, "legs close a window round the strand: the spliced loop cannot go on (review FAIL)", bad=(1, 2))
p3, _, _ = panel(10 + 2 * (W + 30), "Chosen: 14 x 14 column inside the loop", [HV.column_foot(HV.COLUMN["len"], HV.COLUMN["w"])],
                 "1.05 off the tooth tips, loop slides on past the free web ends; sustained ≤ 0.5 (E)")
TW, TH = 3 * W + 80, 40 + Hh + 20
runs = [("v6 run 2", 40, 184.4, 0.596), ("v6 run 3", 52, 199.3, 0.523)]
tab = "".join(f"<tr><td>{n}</td><td>{g:g} cm³</td><td>{m:g} g (M)</td><td>{i:g} (F)</td></tr>" for n, g, m, i in runs)
tab += "<tr><td><b>v7 (chosen)</b></td><td>62 cm³</td><td>~212 g (E)</td><td>~0.46 (E, the run 2 → 3 slope; 0.5 at ~56)</td></tr>"
html = f"""<title>Hub v7 system pass</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
:root {{ --bg:#f6f5f1; --fg:#1d232a; --muted:#5d6670; --line:#c9ccd0; --card:#fff; --body:#5f7f9a; --bad:#c0392b; --belt:#2b2b2b; --p80:#d9d4c7; --pt:#c47a1d }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --body:#7fa3c2; --bad:#e06a5a; --belt:#cfd6dc; --p80:#3b3f45; --pt:#e3a24a; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --body:#7fa3c2; --bad:#e06a5a; --belt:#cfd6dc; --p80:#3b3f45; --pt:#e3a24a; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:20px }}
main {{ max-width:1500px; margin:0 auto; display:grid; gap:16px }} h1 {{ font-size:1.4rem; margin:0; text-wrap:balance }} p {{ margin:0; color:var(--muted); max-width:75ch }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; min-width:0 }} .scroll {{ overflow-x:auto }}
table {{ border-collapse:collapse; font-size:14px }} th, td {{ text-align:left; padding:6px 10px; border-top:1px solid var(--line) }}
svg .h {{ font-weight:600; font-size:13px; fill:var(--fg) }} svg .s {{ font-size:11px; fill:var(--muted) }}
.body {{ fill:var(--body); opacity:.9 }} .bad {{ fill:var(--bad); opacity:.85 }} .belt {{ fill:var(--belt) }} .p80 {{ fill:var(--p80) }} .p16 {{ fill:var(--p80); stroke:var(--belt) }}
.frame {{ fill:none; stroke:var(--line) }} .pt {{ fill:var(--pt) }}
</style>
<main><header><h1>Hub v7 system pass</h1><p>Nox swap arm hub. Three hub v6 runs missed a check, so the hub went back to its system pass. Plan at the axle belt (Y 56..62), to scale ({S:g} px per mm): the GT2 loop runs from the fixed 80T sun to the axle motor's 16T, and the detent tower's column stands inside it. The loop is lap-spliced, so it goes on by sliding under the tower's free web ends.</p></header>
<section class="card"><h2 style="margin:0 0 8px;font-size:1.05rem">Detent tower: three options</h2><div class="scroll"><svg viewBox="0 0 {TW:.0f} {TH:.0f}" width="{TW:.0f}" height="{TH:.0f}" role="img" aria-label="tower options in plan">{p1}{p2}{p3}</svg></div></section>
<section class="card"><h2 style="margin:0 0 8px;font-size:1.05rem">Swap 0.5 at a stated mass</h2><div class="scroll"><table><tr><th>Run</th><th>Generated</th><th>Mass</th><th>Swap worst</th></tr>{tab}</table></div>
<p style="margin-top:8px">Alternatives to more volume: a drawn skirt at the sleeve corner (a structural member, not mounting: rejected, The rule 1), a stiffer plate (no room: the wheel lanes and motor seats). Mass is soft (budget 215 g DEFAULT, hub v5 214 g M).</p></section>
<section class="card"><p>Source <code>runs/hub/v7-concept/section.py</code>; numbers <code>nox/dimensions.md</code> "System box swap: arm", Hub row.</p></section></main>
"""
open(os.path.join(HERE, "section.html"), "w").write(html)
print("->", os.path.join(HERE, "section.html"))
