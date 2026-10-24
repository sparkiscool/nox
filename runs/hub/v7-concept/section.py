"""Hub v6 (run 3) vs hub v7 (system pass) drawn sections: plan at Y 60 round the detent tower, radial cut through the sleeve-to-plate
corner at Z 0 (X 30..70). python3 runs/hub/v7-concept/section.py -> runs/hub/v7-concept/section.html (24 Oct 2026, swap arm worker)"""
import os, sys, json, subprocess
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
BOX = os.path.join(ROOT, "nox", "box")
SNIP = r'''
import sys, json; sys.path.insert(0, %r)
import arm_v7_layout as S7; S7.use()
import hub_v6 as HV, numpy as np
from arm_cad import tm
P = HV.hub_v6_parts(); D, S = P["detent"]
m = tm(P["drawn"] - P["cuts"])
def sec(o, n, keep):
    s = m.section(plane_origin=o, plane_normal=n)
    out = []
    for e in (s.entities if s is not None else []):
        pts = s.vertices[e.points][:, keep]; out.append(np.round(pts, 2).tolist())
    return out
mov = []
for k in ("arm", "roller", "springs"):
    v = S[k] if k != "springs" else S["springs"][0]
    ms = tm(v).section(plane_origin=[0, 60, 0], plane_normal=[0, 1, 0])
print(json.dumps(dict(plan=sec([0, 60, 0], [0, 1, 0], [0, 2]), radial=sec([0, 0, 0], [0, 0, 1], [0, 1]),
                      P=np.round(D["P"], 2).tolist(), A=np.round(D["A"], 2).tolist(), Q=list(HV.DT.chosen()["Q"]))))
''' % BOX


def grab(env):
    e = dict(os.environ, NOX_ARM="v7", **env)
    out = subprocess.run([sys.executable, "-c", SNIP], cwd=BOX, env=e, capture_output=True, text=True, check=True).stdout
    return json.loads(out.strip().splitlines()[-1])


V6 = grab(dict(NOX_TOWER_LEGS="0", NOX_SKIRT="0")); V7 = grab({})
S = 7.0                                                  # px per mm


def plan(d, ox, title):
    x0, z0, x1, z1 = -5, -62, 52, -14                    # window round the tower (X, Z), mm
    W, H = (x1 - x0) * S, (z1 - z0) * S
    t = lambda x, z: f"{ox + (x - x0) * S:.1f},{30 + (z - z0) * S:.1f}"
    out = [f'<text x="{ox}" y="18" class="h">{title}</text>', f'<rect x="{ox}" y="30" width="{W:.0f}" height="{H:.0f}" class="frame"/>']
    for poly in d["plan"]:
        if all(x0 - 20 < p[0] < x1 + 20 and z0 - 20 < p[1] < z1 + 20 for p in poly):
            out.append('<polygon class="body" points="' + " ".join(t(*p) for p in poly) + '"/>')
    for k, lab in (("Q", "column Q"), ("P", "pivot P"), ("A", "anchor A")):
        x, z = d[k]; out.append(f'<circle cx="{ox + (x - x0) * S:.1f}" cy="{30 + (z - z0) * S:.1f}" r="2.5" class="pt"/>'
                                f'<text x="{ox + (x - x0) * S + 6:.1f}" y="{30 + (z - z0) * S - 6:.1f}" class="s">{lab}</text>')
    return "".join(out), W, H


def radial(d, ox, oy, title):
    x0, y0, x1, y1 = 28, 36, 72, 58
    W, H = (x1 - x0) * S, (y1 - y0) * S
    t = lambda x, y: f"{ox + (x - x0) * S:.1f},{oy + (y1 - y) * S:.1f}"
    cid = f"c{int(ox)}"
    out = [f'<text x="{ox}" y="{oy - 12}" class="h">{title}</text>', f'<rect x="{ox}" y="{oy}" width="{W:.0f}" height="{H:.0f}" class="frame"/>',
           f'<clipPath id="{cid}"><rect x="{ox}" y="{oy}" width="{W:.0f}" height="{H:.0f}"/></clipPath><g clip-path="url(#{cid})">']
    for poly in d["radial"]:
        pts = [p for p in poly]
        if any(x0 - 5 < p[0] < x1 + 5 and y0 - 5 < p[1] < y1 + 5 for p in pts):
            out.append('<polygon class="body" points="' + " ".join(t(*p) for p in pts) + '"/>')
    out.append('</g>')
    out.append(f'<text x="{ox + 4}" y="{oy + H + 16:.0f}" class="s">X 28..72 at Z 0, Y 36..58 (sleeve r 43, plate Y 50..54)</text>')
    return "".join(out), W, H


p6, W, H = plan(V6, 10, "Hub v6 run 3: one column at Q (bent, sustained 1.08)")
p7, _, _ = plan(V7, 10 + W + 40, "Hub v7: legs under P and A on plate pads (a frame)")
r6, RW, RH = radial(V6, 10, 30 + H + 60, "v6: the sleeve meets the plate square (swap 0.52 there)")
r7, _, _ = radial(V7, 10 + W + 40, 30 + H + 60, "v7: 45 deg skirt r 43 to 49, 2.5 rim")
TW, TH = 2 * W + 60, 30 + H + 60 + RH + 40
html = f"""<title>Hub v7 system pass</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
:root {{ --bg:#f6f5f1; --fg:#1d232a; --muted:#5d6670; --line:#c9ccd0; --card:#fff; --body:#7d8a96; --pt:#c47a1d }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --body:#8795a3; --pt:#e3a24a; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --body:#8795a3; --pt:#e3a24a; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:20px }}
main {{ max-width:1100px; margin:0 auto; display:grid; gap:16px }} h1 {{ font-size:1.4rem; margin:0; text-wrap:balance }} p {{ margin:0; color:var(--muted); max-width:70ch }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; min-width:0 }} .scroll {{ overflow-x:auto }}
svg .h {{ font-weight:600; font-size:13px; fill:var(--fg) }} svg .s {{ font-size:11px; fill:var(--muted) }}
.body {{ fill:var(--body); fill-rule:evenodd }} .frame {{ fill:none; stroke:var(--line) }} .pt {{ fill:var(--pt) }}
</style>
<main><header><h1>Hub v7 system pass</h1><p>Nox swap arm hub, drawn bodies only (the topology fills below the plate). Three hub v6 runs missed a check, so the hub went back to its system pass: the detent tower becomes a frame and the sleeve gets a skirt where the plate meets it. To scale, {S:g} px per mm.</p></header>
<section class="card"><div class="scroll"><svg viewBox="0 0 {TW:.0f} {TH:.0f}" width="{TW:.0f}" height="{TH:.0f}" role="img" aria-label="hub v6 and v7 sections">{p6}{p7}{r6}{r7}</svg></div></section>
<section class="card"><p>Top row: plan at Y 60 round the tower (X right, Z down). Bottom row: radial section at Z 0. Source <code>runs/hub/v7-concept/section.py</code>; numbers <code>nox/dimensions.md</code> "System box swap: arm", Hub row.</p></section></main>
"""
open(os.path.join(HERE, "section.html"), "w").write(html)
print("->", os.path.join(HERE, "section.html"))
