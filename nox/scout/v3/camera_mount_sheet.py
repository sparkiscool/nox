"""Scout camera mount v1: the concept sheet (one HTML page: a to-scale side section, the two alternatives weighed, the checks,
the breaking order, stills). Numbers come from runs/scout_camera_mount/v1/result.json.

python3 nox/scout/v3/camera_mount_sheet.py   -> runs/scout_camera_mount/v1/concept.html
"""
import os, json, math, base64, html

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
RUN = os.path.join(ROOT, "runs", "scout_camera_mount", "v1")
R = json.load(open(os.path.join(RUN, "result.json")))
MOTION_URL = "https://claude.ai/artifact/82LPH8p6h9bXxAq6h3kYPE"
P = R["params"]; S = R["snap"]; F = R["frame"]; V = R["view"]; FIT = R["fit"]


def img(name):
    return "data:image/png;base64," + base64.b64encode(open(os.path.join(RUN, name), "rb").read()).decode()


# ---------------------------------------------------------------- side section (x-z), to scale
X0, X1, Z0, Z1, K = 100.0, 162.0, 2.0, 76.0, 8.0
W, H = (X1 - X0) * K, (Z1 - Z0) * K
px = lambda x: (x - X0) * K
pz = lambda z: (Z1 - z) * K
pt = lambda x, z: f"{px(x):.1f},{pz(z):.1f}"
r_b = 3.1; r_in = r_b + P["T_F"]; nh = r_in * math.tan(math.radians(22.5)); u0 = r_in - 0.5
xt, zt, xb, zb = 110.5, 61.1, 122.6, 11.5
zm = 42.0


def clip(x, z):
    a0 = math.degrees(math.asin((3.0 - P["SNAP"]) / r_b))              # mouth half-angle at the bore
    R_ = (r_b + r_in) / 2; s, e = 180 - a0, -(180 - a0)
    p0 = (x + R_ * math.cos(math.radians(s)), z + R_ * math.sin(math.radians(s)))
    p1 = (x + R_ * math.cos(math.radians(e)), z + R_ * math.sin(math.radians(e)))
    return (f'<path class="clip" d="M{pt(*p0)} A{R_ * K:.1f},{R_ * K:.1f} 0 1,1 {pt(*p1)}"/>'
            f'<circle class="tube" cx="{px(x):.1f}" cy="{pz(z):.1f}" r="{3.0 * K:.1f}"/>')


def poly(pts, cls):
    return f'<polygon class="{cls}" points="{" ".join(pt(*p) for p in pts)}"/>'


tanv = math.tan(math.radians(73 / 2 + 2))
svg = [f'<svg viewBox="0 0 {W:.0f} {H:.0f}" role="img" aria-label="Side section of the camera mount on the two tubes, to scale">']
for g in range(int(X0 // 10 * 10 + 10), int(X1) - 5, 10):
    svg.append(f'<line class="grid" x1="{px(g):.1f}" y1="0" x2="{px(g):.1f}" y2="{H:.0f}"/><text class="tick" x="{px(g) + 3:.1f}" y="{H - 6:.0f}">x {g}</text>')
for g in range(10, int(Z1), 10):
    svg.append(f'<line class="grid" x1="0" y1="{pz(g):.1f}" x2="{W:.0f}" y2="{pz(g):.1f}"/><text class="tick" x="4" y="{pz(g) - 3:.1f}">z {g}</text>')
# view cone (vertical field + 2 deg)
svg.append(poly([(149, 42), (X1, 42 + (X1 - 149) * tanv), (X1, 42 - (X1 - 149) * tanv)], "cone"))
svg.append(f'<text class="lab cone-lab" x="{px(146):.1f}" y="{pz(29):.1f}">field ±38.5°</text>')
# the cage near the nose (projected)
svg.append(f'<line class="rod" x1="{px(xt):.1f}" y1="{pz(zt):.1f}" x2="{px(116.5):.1f}" y2="{pz(36.3):.1f}"/>')
svg.append(f'<line class="rod thin" x1="{px(xb):.1f}" y1="{pz(zb):.1f}" x2="{px(116.5):.1f}" y2="{pz(36.3):.1f}"/>')
svg.append(f'<line class="rod thin" x1="{px(xb):.1f}" y1="{pz(zb):.1f}" x2="{px(106.7):.1f}" y2="{pz(-19):.1f}"/>')
svg.append(f'<circle class="node" cx="{px(116.5):.1f}" cy="{pz(36.3):.1f}" r="{6 * K:.1f}"/><text class="lab" x="{px(116.5) - 26:.1f}" y="{pz(36.3) + 4:.1f}">16</text>')
# mount: back frame, walls, hood, truss arms, knee
svg.append(poly([(128, 17), (131, 17), (131, 67), (128, 67)], "mount"))
svg.append(poly([(128, 64), (153, 64), (153, 67 - 1.2), (151.8, 67), (128, 67)], "mount"))
svg.append(poly([(128, 17), (151.8, 17), (153, 18.2), (153, 20), (128, 20)], "mount"))
svg.append(poly([(147, 17), (153, 17), (153, 67), (147, 67)], "hood"))
T1, T2, T3, T4 = (xt + u0, zt - nh), (131, zm + 2), (131, 66), (xt + u0, zt + nh)
svg.append(poly([T1, T2, T3, T4], "web"))
K1, K2, K3, K4, K5 = (xb + u0, zb - nh), (140, 17), (140, 20), (129.5, 20), (xb + u0, zb + nh)
svg.append(poly([K1, K2, K3, K4, K5], "web"))
svg.append(clip(xt, zt) + clip(xb, zb))
# camera and lens
svg.append(poly([(135, 23), (149, 23), (149, 61), (135, 61)], "cam"))
svg.append(f'<circle class="lens" cx="{px(149):.1f}" cy="{pz(42):.1f}" r="4"/>')
# dimensions and notes
svg.append(f'<line class="dim" x1="{px(149):.1f}" y1="{pz(70.5):.1f}" x2="{px(153):.1f}" y2="{pz(70.5):.1f}"/>'
           f'<text class="lab acc" x="{px(132):.1f}" y="{pz(72):.1f}">hood {V["hood_lead_mm"]:.1f} ahead of lens</text>')
svg.append(f'<line class="push" x1="{px(160):.1f}" y1="{pz(5):.1f}" x2="{px(141):.1f}" y2="{pz(5):.1f}" marker-end="url(#arr)"/>'
           f'<text class="lab" x="{px(141):.1f}" y="{pz(7):.1f}">pushed on along −x</text>')
svg.append(f'<text class="lab" x="{px(101):.1f}" y="{pz(zt + 9):.1f}">rod 7-8</text><text class="lab" x="{px(101):.1f}" y="{pz(zb - 8):.1f}">rod 14-15</text>')
svg.append(f'<text class="lab" x="{px(136):.1f}" y="{pz(52):.1f}">camera</text>')
svg.insert(1, '<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="arrowhead"/></marker></defs>')
svg.append("</svg>")
SVG = "\n".join(svg)

bo = F["breaking_order"]
def first(k): a = bo[k][0]; return f"{a[1]} at {a[0]:.0f} N"
cases = F["cases"]
fc = cases["frontal hit 250 N on the hood centre (-x)"]; fk = cases["frontal hit 250 N on the hood top corner (-x)"]
sys_txt = open(os.path.join(RUN, "system_check.txt")).read().strip().splitlines()[-1]

rows_alt = [
    ("Parts", "1 print + 2 stop collars", "5 prints + 4 M2.5 screws, 4 inserts, 2 collars"),
    ("Mass, mount + clips", f"{R['mass_mount_g']:.1f} g (M)", f"about {R['mass_mount_g'] + 6:.0f} g (E: lugs, pads, hardware)"),
    ("Clip mouths", "all face into the cage (−x); one push fits all four", "top down, bottom up; each clip snapped on alone"),
    ("Frontal hit on the hood centre, 250 N", "carried: PETG " + f"{fc['petg_sigma_MPa']:.0f} MPa, tubes {fc['carbon_ratio']:.2f} of allowable (F)", "carried (E)"),
    ("Frontal hit on a hood corner", f"far-side clips pop at about {250 * S['pop_N'] / max(fk['clip_pull_N'], 1e-6):.0f} N (F on E)", "carried; pops only on vertical hits (E)"),
    ("Cage build tolerance", "about ±0.3 at the snap (E)", "±0.5 in oversize holes, clips slide in y (E)"),
    ("A broken clip", "reprint the mount (21 g, one print)", "reprint one clip"),
]
alt_html = "".join(f"<tr><th scope=\"row\">{html.escape(a)}</th><td>{html.escape(b)}</td><td>{html.escape(c)}</td></tr>" for a, b, c in rows_alt)
order_html = "".join(f"<tr><th scope=\"row\">{html.escape(k)}</th><td>{html.escape(v[0][1])}</td><td class=\"num\">{v[0][0]:.0f} N</td><td>{html.escape(v[1][1]) if len(v) > 1 else ''}</td><td class=\"num\">{(f'{v[1][0]:.0f} N' if len(v) > 1 else '')}</td></tr>" for k, v in bo.items())

page = f"""<title>Scout Camera Mount v1</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600;700&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* layout: a hardware spec sheet; one column of plates, a to-scale section as the centrepiece, data in mono */
:root {{
  --bg: #eef1f4; --panel: #ffffff; --ink: #161b22; --muted: #5b6673; --line: #c9d1da;
  --accent: #c8692a; --steel: #8a96a3; --cam: #2f6b3f; --ok: #2f7d4f; --warn: #b07a12;
  --f-display: "Barlow Condensed", "Arial Narrow", sans-serif; --f-body: "IBM Plex Sans", system-ui, sans-serif; --f-mono: "IBM Plex Mono", ui-monospace, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #0f1318; --panel: #171d24; --ink: #e6ebf0; --muted: #98a3af; --line: #2c3540;
  --accent: #e08a4c; --steel: #6f7b88; --cam: #4f9a63; --ok: #5bbf85; --warn: #e0b04c; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #0f1318; --panel: #171d24; --ink: #e6ebf0; --muted: #98a3af; --line: #2c3540;
  --accent: #e08a4c; --steel: #6f7b88; --cam: #4f9a63; --ok: #5bbf85; --warn: #e0b04c; color-scheme: dark }}
body {{ background: var(--bg); color: var(--ink); font-family: var(--f-body); font-size: 15px; line-height: 1.55; }}
.wrap {{ max-width: 1040px; margin: 0 auto; padding-block: 28px 56px; padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); display: grid; gap: 22px; }}
header {{ display: grid; gap: 6px; }}
.eyebrow {{ font-family: var(--f-mono); font-size: 12px; letter-spacing: .08em; text-transform: uppercase; color: var(--muted); }}
h1 {{ font-family: var(--f-display); font-weight: 700; font-size: clamp(34px, 6vw, 54px); line-height: 1; letter-spacing: .01em; margin: 0; text-wrap: balance; }}
h2 {{ font-family: var(--f-display); font-weight: 600; font-size: 24px; letter-spacing: .02em; margin: 0 0 10px; text-transform: uppercase; }}
p {{ margin: 0; max-width: 68ch; }}
.lede {{ color: var(--muted); }}
.chips {{ display: flex; flex-wrap: wrap; gap: 8px; }}
.chip {{ font-family: var(--f-mono); font-size: 12px; padding: 3px 8px; border: 1px solid var(--line); border-radius: 3px; background: var(--panel); }}
.chip.ok {{ border-color: var(--ok); color: var(--ok); }}
.plate {{ background: var(--panel); border: 1px solid var(--line); padding: 18px; display: grid; gap: 12px; min-width: 0; clip-path: polygon(0 0, calc(100% - 14px) 0, 100% 14px, 100% 100%, 14px 100%, 0 calc(100% - 14px)); }}
.spec {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 1px; background: var(--line); border: 1px solid var(--line); }}
.spec div {{ background: var(--panel); padding: 10px 12px; display: grid; gap: 2px; }}
.spec b {{ font-family: var(--f-mono); font-weight: 500; font-size: 20px; font-variant-numeric: tabular-nums; }}
.spec span {{ font-size: 12px; color: var(--muted); }}
.hero {{ display: grid; grid-template-columns: 1.25fr 1fr; gap: 18px; align-items: start; }}
.hero img, .stills img {{ width: 100%; height: auto; border: 1px solid var(--line); display: block; background: #fff; }}
.stills {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 12px; }}
figure {{ margin: 0; display: grid; gap: 6px; min-width: 0; }}
figcaption {{ font-size: 12px; color: var(--muted); }}
a {{ color: var(--accent); }}
a:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 2px; }}
.svgbox {{ overflow-x: auto; }}
.svgbox svg {{ width: 100%; max-width: 560px; min-width: 360px; height: auto; display: block; margin: 0 auto; }}
svg .grid {{ stroke: var(--line); stroke-width: 1; }}
svg .tick {{ fill: var(--muted); font: 11px var(--f-mono); }}
svg .lab {{ fill: var(--ink); font: 13px var(--f-mono); }}
svg .acc {{ fill: var(--accent); }}
svg .cone {{ fill: var(--accent); fill-opacity: .12; stroke: var(--accent); stroke-dasharray: 5 4; }}
svg .cone-lab {{ fill: var(--accent); font-size: 12px; }}
svg .rod {{ stroke: var(--steel); stroke-width: {6 * K:.0f}; stroke-linecap: round; opacity: .55; }}
svg .rod.thin {{ stroke-width: {3 * K:.0f}; }}
svg .node {{ fill: var(--steel); fill-opacity: .5; }}
svg .tube {{ fill: var(--steel); }}
svg .clip {{ fill: none; stroke: var(--ink); stroke-width: {P["T_F"] * K:.0f}; }}
svg .mount {{ fill: var(--ink); fill-opacity: .18; stroke: var(--ink); stroke-width: 1.5; }}
svg .hood {{ fill: var(--accent); fill-opacity: .35; stroke: var(--accent); stroke-width: 1.5; }}
svg .web {{ fill: var(--ink); fill-opacity: .1; stroke: var(--ink); stroke-width: 2; }}
svg .cam {{ fill: var(--cam); fill-opacity: .35; stroke: var(--cam); stroke-width: 1.5; }}
svg .lens {{ fill: var(--ink); }}
svg .dim {{ stroke: var(--accent); stroke-width: 2; }}
svg .push {{ stroke: var(--ink); stroke-width: 2; }}
svg .arrowhead {{ fill: var(--ink); }}
.tbl {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; }}
th, td {{ text-align: left; padding: 7px 10px; border-bottom: 1px solid var(--line); vertical-align: top; }}
thead th {{ font-family: var(--f-mono); font-size: 12px; font-weight: 500; letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }}
tbody th {{ font-weight: 500; }}
td.num {{ font-family: var(--f-mono); font-variant-numeric: tabular-nums; white-space: nowrap; }}
.pick {{ color: var(--accent); }}
ul {{ margin: 0; padding-left: 18px; display: grid; gap: 4px; max-width: 75ch; }}
.mono {{ font-family: var(--f-mono); font-size: 12.5px; color: var(--muted); overflow-wrap: anywhere; }}
@media (max-width: 760px) {{ .hero {{ grid-template-columns: 1fr; }} }}
</style>
<div class="wrap">
<header>
  <div class="eyebrow">Scout v3 · camera mount · v1 · state DESIGN · 27 Oct 2026</div>
  <h1>Scout Camera Mount v1</h1>
  <p class="lede">One PETG print holds the DECXIN 2633V2 at the nose. It pushes onto the two 6/4 carbon tubes of the sized cage after the cage is built, and a hood stands {V['hood_lead_mm']:.1f} mm ahead of the lens, outside its field.</p>
  <div class="chips"><span class="chip ok">solid_check PASS</span><span class="chip ok">nothing in the camera view</span><span class="chip ok">lowest mode {F['modes_hz'][0]:.0f} Hz ≥ 100</span><span class="chip ok">system_check PASS</span><span class="chip">A1 mini, side down</span></div>
</header>

<section class="spec" aria-label="Key numbers">
  <div><b>{R['mass_mount_g']:.1f} g</b><span>mount, PETG (M); collars 2 × {R['mass_collar_g']:.2f} g</span></div>
  <div><b>{V['hood_lead_mm']:.1f} mm</b><span>hood ahead of the lens front (C, gate 3)</span></div>
  <div><b>{V['cone_margin_deg_point']:.0f}°</b><span>closest part outside the field +2° (C)</span></div>
  <div><b>{F['modes_hz'][0]:.1f} Hz</b><span>lowest mode, a cage pitch mode the camera rides (F)</span></div>
  <div><b>{S['install_N']:.0f} N</b><span>push-on and pop per clip (E); 4 clips at once</span></div>
  <div><b>{FIT['rod_clearance_min_mm']:.1f} mm</b><span>closest other rod (sized tubes, C)</span></div>
</section>

<section class="hero">
  <figure><img src="{img('still_closeup.png')}" alt="Render of the mount on the cage nose: a chamfered box frame round the camera with chevron truss windows, truss arms back to the top tube, a knee down to the bottom tube, orange stop collars."><figcaption>Seated on the v12 cage (6/4 tubes dark, 3.0 rods light). The 3D view and the push-on sequence are in the <a href="{MOTION_URL}">motion artifact</a>.</figcaption></figure>
  <div class="plate">
    <h2>What it is</h2>
    <ul>
      <li>Chamfered box frame round the camera, 0.5 clear at the sides, lips 6 above and below, all behind the lens. The board sits on 4 M2.5 heat-set bosses (28 × 28, Ø12 pads with a 2.0 web) on bought M2.5 nylon washers, 1.0 thick (E).</li>
      <li>Lens hood: the frame runs on to x 153, {V['hood_lead_mm']:.1f} ahead of the lens. A flat hit lands on its front land and goes back through the cradle.</li>
      <li>Faces are chevron trusses, the back is an X frame through the bosses, arms and knees are flanged webs in the cheek plane. Every visible face is planar and chamfered: drawn for the look Spark asked for, a topology pass on the hidden web is the v2 option.</li>
      <li>4 octagon C-clips, bore {R['bore']:.1f}, mouth {S['mouth_mm']:.1f}, fingers {S['t_f']:.1f}, {S['L']:.0f} long, mouths facing into the cage. Grip ribs hold the tube against rattle and slide. Two CA'd stop collars on rod 14-15 fix y.</li>
    </ul>
  </div>
</section>

<section class="plate">
  <h2>Side section, to scale</h2>
  <div class="svgbox">{SVG}</div>
  <p class="mono">Grid 10 mm. Cage rods projected onto the y = 18 plane. The hood and lips stay outside the camera's vertical field; the horizontal field is wider and clears the cheeks by more.</p>
</section>

<section class="plate">
  <h2>Two ways to clip on</h2>
  <div class="tbl"><table>
    <thead><tr><th></th><th class="pick">A · one piece (picked, DEFAULT)</th><th>B · cradle + 4 bolted clips</th></tr></thead>
    <tbody>{alt_html}</tbody>
  </table></div>
  <p>A is the simplest that meets every gate: one print, no hardware, push it on after the cage is built. Its fuse is soft. It lets go at about the force it took to push on (about {4 * S['pop_N']:.0f} N straight, E), so an off-centre bump on the hood pops it. B keeps the camera on through frontal hits but adds 4 prints, 8 pieces of hardware and about 6 mm of lug height. Spark picks; B is drawn only on this page.</p>
</section>

<section class="plate">
  <h2>What lets go first</h2>
  <div class="tbl"><table>
    <thead><tr><th>Load on the hood centre</th><th>First</th><th></th><th>Then</th><th></th></tr></thead>
    <tbody>{order_html}</tbody>
  </table></div>
  <p>The clips are the fuse wherever a load pulls them out. A centred frontal push sits on the clips' closed backs, so it reaches the cage; the cage's own fuse (the saddle ties) goes at {bo['push back -x (frontal)'][0][0]:.0f} N, and the lens stays {V['hood_lead_mm']:.1f} behind the hood throughout. Frame model on the v12 cage with the sized tubes (F on E properties).</p>
</section>

<section class="stills" aria-label="Stills">
  <figure><img src="{img('still_side.png')}" alt="Side view of the mount on the nose of the cage."><figcaption>Side: the hood stands past the cage nose; the truss arm runs back to the top tube.</figcaption></figure>
  <figure><img src="{img('still_inside.png')}" alt="View from inside the cage toward the mount's back frame and clips."><figcaption>From inside the cage: the X back frame and the clip mouths facing in.</figcaption></figure>
  <figure><img src="{img('preview_scout_camera_mount_v1.png')}" alt="Print preview sheet of the mount as saved, side face down."><figcaption>Print file as saved: side face down, 811 mm² on the bed, clip bores vertical.</figcaption></figure>
</section>

<section class="plate">
  <h2>Open</h2>
  <ul>
    <li>Fit the clip on a coupon first: snap {P['SNAP']:.2f} per side and grip ribs {P['GRIP']:.2f} are estimates. Print one clip pair on a 6/4 tube offcut before the full mount.</li>
    <li>Camera lens and board back are not measured (Q3). The field check takes the lens as a disk of r {P['APERTURE_R']:.0f}; it still clears by {V['cone_margin_deg_aperture']:.0f}°.</li>
    <li>For the cage: the frame model v2 put the bottom clips on 3.0 rods 14-16 / 15-16 (its v11 clip points) and weighed the 6/4 tubes at 3.0 g/m. With the real points and tube mass the lowest mode is {F['modes_hz'][0]:.1f} Hz, a cage pitch mode.</li>
  </ul>
  <p class="mono">{html.escape(sys_txt)}</p>
</section>
</div>
"""
open(os.path.join(RUN, "concept.html"), "w").write(page)
print("concept.html", round(len(page) / 1e6, 2), "MB")
