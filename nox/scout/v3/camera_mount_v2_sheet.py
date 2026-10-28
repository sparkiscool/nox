"""Scout camera mount v2: the concept sheet (one HTML page). To-scale side, plan and front views of the assembled mount on the v12
cage (every rod 3.0) with the camera field, the plate, the props' disks and the install sweep; the system pass (first mode per
concept, the 100 Hz gate); the breaking order and the checks. Built from runs/scout_camera_mount/v2/{concept,checks,result}.json.

python3 nox/scout/v3/camera_mount_v2_sheet.py   -> runs/scout_camera_mount/v2/concept.html (+ views.png)
"""
import os, sys, json, math, base64, io, html
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import camera_mount_v2 as cm

RUN = os.path.join(ROOT, "runs", "scout_camera_mount", "v2"); N = cm.N
MOUNT, CAGE, CAM, FIELD, PLATE, PROP, CLIP = "#7d8a99", "#c0563b", "#3f7a52", "#3f7a52", "#5b6470", "#8a6fb3", "#d99a2b"


def views(mesh):
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.4), gridspec_kw=dict(width_ratios=[1.45, 1.45, 0.75]))
    fig.patch.set_alpha(0)
    m = mesh.copy()
    if len(m.faces) > 60000:
        try: m = m.simplify_quadric_decimation(face_count=60000)
        except Exception: pass
    T = m.vertices[m.faces]
    for ax, (i, j), title in ((axs[0], (0, 2), "side (x, z), looking from +y"), (axs[1], (0, 1), "plan (x, y), from above"), (axs[2], (1, 2), "front (y, z), from the nose")):
        ax.set_facecolor("none"); ax.set_aspect("equal"); ax.set_title(title, color="#888", fontsize=10, loc="left")
        for sp in ax.spines.values(): sp.set_color("#999")
        ax.tick_params(colors="#888", labelsize=8); ax.grid(alpha=0.15)
        ax.add_collection(PolyCollection(T[:, :, [i, j]], facecolor=MOUNT, edgecolor="none", alpha=0.55, zorder=3))
        for a, b in cm.CG["rods"]:
            ax.plot([N[a][i], N[b][i]], [N[a][j], N[b][j]], color=CAGE, lw=1.0, alpha=0.75, zorder=2)
        for p, q in cm.RES["links"].values():
            ax.plot([p[i], q[i]], [p[j], q[j]], color=CAGE, lw=0.8, ls=":", alpha=0.7, zorder=2)
        for k in (14, 15, 16, 17, 7, 8):
            ax.add_patch(plt.Circle((N[k][i], N[k][j]), 6.0, color=CAGE, alpha=0.35, zorder=2))
            ax.annotate(str(k), (N[k][i], N[k][j]), color=CAGE, fontsize=8, ha="center", va="center", zorder=6)
        (x0, x1), (y0, y1), (z0, z1) = cm.CAM_BOX
        lo = {0: x0, 1: y0, 2: z0}; hi = {0: x1, 1: y1, 2: z1}
        ax.add_patch(plt.Rectangle((lo[i], lo[j]), hi[i] - lo[i], hi[j] - lo[j], fill=False, ec=CAM, lw=1.5, zorder=4))
        for s in (1, -1):
            for pt in [(cm.FOOT_BOLTS[0][0], s * 13.0)]:
                pass
        if (i, j) in ((0, 2), (0, 1)):
            half = cm.VFOV if j == 2 else cm.HFOV
            t = math.tan(math.radians(half / 2 + cm.VIEW_MARGIN)); L = 30
            c = cm.LENS[j]
            ax.fill([cm.LENS[0], cm.LENS[0] + L, cm.LENS[0] + L], [c, c + cm.APERTURE_R + L * t, c - cm.APERTURE_R - L * t], color=FIELD, alpha=0.12, zorder=1)
            ax.text(cm.LENS[0] + 6, c, "field + 2°", color=FIELD, fontsize=8, va="center")
        if (i, j) == (0, 2):
            pl = cm.plate_outline(); ax.fill_between([pl[:, 0].min(), 51.9], 18.5, 22.0, color=PLATE, alpha=0.6, zorder=1)
            ax.text(0, 13, "Bee35 plate", color=PLATE, fontsize=8)
            ax.fill_between([9.6, 98.6], 37.2, 45.2, color=PROP, alpha=0.18, zorder=1); ax.text(30, 47, "prop disk", color=PROP, fontsize=8)
            ax.annotate("install: pushed on along −x, 110 mm", (128, 2), (160, 2), color="#888", fontsize=8, ha="right",
                        arrowprops=dict(arrowstyle="<-", color="#888"))
            ax.set_xlim(15, 165); ax.set_ylim(-25, 75)
        if (i, j) == (0, 1):
            for (mx, my) in cm.MOTORS:
                ax.add_patch(plt.Circle((mx, my), cm.PROP_R, color=PROP, alpha=0.10, zorder=0))
            pl = cm.plate_outline(); ax.fill(pl[:, 0], pl[:, 1], color=PLATE, alpha=0.25, zorder=0)
            for (bx, by) in cm.FOOT_BOLTS:
                ax.add_patch(plt.Circle((bx, by), 1.7, color="#222", zorder=6)); ax.annotate(f"M3 ({bx:.0f}, {by:+.0f})", (bx, by), (bx - 8, by + (9 if by > 0 else -12)), color="#888", fontsize=8)
            for s in (1, -1):
                ax.add_patch(plt.Circle((N[14][0], s * cm.CLIP_Y), 3.0, color=CLIP, zorder=6))
            ax.annotate(f"clips y ±{cm.CLIP_Y}", (N[14][0], cm.CLIP_Y), (135, 62), color=CLIP, fontsize=8, arrowprops=dict(arrowstyle="-", color=CLIP))
            ax.set_xlim(15, 165); ax.set_ylim(-75, 75)
        if (i, j) == (1, 2):
            ax.set_xlim(-75, 75); ax.set_ylim(-25, 75)
            for s in (1, -1):
                ax.add_patch(plt.Circle((s * cm.CLIP_Y, N[14][2]), 3.0, color=CLIP, zorder=6))
    fig.tight_layout()
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=110, transparent=True); fig.savefig(os.path.join(RUN, "views.png"), dpi=110); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def concept_svg(con):
    rows = [(k, v["first_mode_hz"], v.get("first_camera_mode_hz")) for k, v in con.items() if isinstance(v, dict) and "first_mode_hz" in v]
    W, rh, left, right = 760, 26, 330, 40; H = rh * len(rows) + 46; xmax = 220.0
    sx = lambda v: left + (W - left - right) * min(v, xmax) / xmax
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="first mode per concept" class="chart">']
    for t in (0, 50, 100, 150, 200):
        out.append(f'<line x1="{sx(t):.1f}" y1="8" x2="{sx(t):.1f}" y2="{H - 26}" class="grid"/><text x="{sx(t):.1f}" y="{H - 10}" class="tick" text-anchor="middle">{t} Hz</text>')
    out.append(f'<line x1="{sx(100):.1f}" y1="4" x2="{sx(100):.1f}" y2="{H - 26}" class="gate"/>')
    for n, (k, v, c) in enumerate(rows):
        y = 12 + n * rh; ok = v >= 100; pick = "picked" in k
        out.append(f'<text x="{left - 10}" y="{y + 13}" class="lab{" pick" if pick else ""}" text-anchor="end">{html.escape(k[:52])}</text>')
        out.append(f'<rect x="{left}" y="{y + 2}" width="{sx(v) - left:.1f}" height="{rh - 9}" class="{"ok" if ok else "bad"}{" pickbar" if pick else ""}"/>')
        out.append(f'<text x="{sx(v) + 5:.1f}" y="{y + 13}" class="val">{v:.0f}</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    con = json.load(open(os.path.join(RUN, "concept.json"))); chk = json.load(open(os.path.join(RUN, "checks.json")))
    res = json.load(open(os.path.join(RUN, "result.json"))) if os.path.exists(os.path.join(RUN, "result.json")) else {}
    mesh = trimesh.load(os.path.join(RUN, "camera_mount_assembled.stl"), force="mesh")
    png = views(mesh)
    fr, sn, vw, st, ft = chk["frame"], chk["snap"], chk["view"], chk["stroke"], chk["fit"]
    order_rows = "".join(f"<tr><td>{html.escape(k)}</td><td>{html.escape(v[0][1])}</td><td class='num'>{v[0][0]:.0f} N</td><td>{html.escape(v[1][1]) if len(v) > 1 else ''}</td><td class='num'>{(str(int(v[1][0])) + ' N') if len(v) > 1 else ''}</td></tr>"
                         for k, v in fr["breaking_order"].items())
    worst = res.get("worst_index") or res.get("worst") or ""
    checks = [("Solid check (print file)", chk["print"]["solid_check"]), ("Mass", f"{chk['mass_g']:.1f} g (M)"),
              ("First mode, assembled", f"{fr['first_mode_hz']:.0f} Hz (F), camera {fr['camera_share'][0]:.0%} of its energy"),
              ("First camera mode", f"{fr['first_camera_mode_hz']:.0f} Hz (F)"), ("Cage alone (no mount)", f"{fr['cage_alone_modes_hz'][0]:.0f} Hz (F)"),
              ("In the camera's field", f"{vw['in_view_points']} points, {vw['in_view_aperture']} with the lens as r 6; {vw['cone_margin_deg_aperture']}° clear"),
              ("Hood ahead of the lens", f"{vw['hood_lead_mm']:.1f} mm (gate 3)"),
              ("Install stroke 110 mm", f"other members ≥ {st['other_members_min_clear_mm']:.2f} mm; rod 14-15 only by the snap ({st['clip_rod_max_interference_mm']:.2f} mm)"),
              ("Clips (2, slit fork)", f"push-on {sn['install_N']:.0f} N, pop {sn['pop_N']:.0f} N each, prong {sn['prong_sigma_MPa']:.0f} MPa (E)"),
              ("Closest cage member", f"{ft['member_clearance_worst'][0][0]} {ft['member_clearance_min_mm']:.2f} mm")]
    crow = "".join(f"<tr><th>{html.escape(a)}</th><td>{html.escape(str(b))}</td></tr>" for a, b in checks)
    page = f"""<title>Scout camera mount v2</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:wght@400;500&family=IBM+Plex+Mono:wght@400&display=swap">
<style>
/* layout: a single drawing-sheet column; the to-scale views lead, the system pass and the numbers follow */
:root {{ --bg:#f4f5f2; --panel:#ffffff; --fg:#1d2228; --mute:#5d6670; --line:#d5d9dc; --accent:#c0563b; --ok:#3f7a52; --bad:#b4443a; --clip:#d99a2b;
  --f-disp:"IBM Plex Sans Condensed", "Arial Narrow", sans-serif; --f-body:"IBM Plex Sans", system-ui, sans-serif; --f-mono:"IBM Plex Mono", ui-monospace, monospace; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#15191d; --panel:#1d2227; --fg:#e4e7ea; --mute:#99a2ab; --line:#333a41; --accent:#e07a5f; --ok:#6fb184; --bad:#e0766b; --clip:#e6b04d; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#15191d; --panel:#1d2227; --fg:#e4e7ea; --mute:#99a2ab; --line:#333a41; --accent:#e07a5f; --ok:#6fb184; --bad:#e0766b; --clip:#e6b04d; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font-family:var(--f-body); line-height:1.5; }}
main {{ max-width:1180px; margin:0 auto; padding-block:28px 48px; padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); display:grid; gap:28px; }}
h1 {{ font-family:var(--f-disp); font-weight:600; font-size:2rem; margin:0; letter-spacing:.01em; text-wrap:balance; }}
h2 {{ font-family:var(--f-disp); font-weight:600; font-size:1.15rem; margin:0 0 10px; text-transform:uppercase; letter-spacing:.06em; color:var(--mute); }}
.lede {{ max-width:72ch; margin:6px 0 0; color:var(--mute); }}
.sheet {{ background:var(--panel); border:1px solid var(--line); padding:14px; overflow-x:auto; }}
.sheet img {{ width:100%; min-width:760px; display:block; }}
.two {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(320px, 1fr)); gap:28px; }}
.two > * {{ min-width:0; }}
p, li {{ max-width:70ch; }}
.chartwrap {{ overflow-x:auto; }}
.chart {{ width:100%; min-width:600px; font-family:var(--f-body); }}
.chart .grid {{ stroke:var(--line); stroke-width:1; }} .chart .gate {{ stroke:var(--accent); stroke-width:2; stroke-dasharray:4 3; }}
.chart .tick, .chart .val {{ fill:var(--mute); font-size:11px; font-family:var(--f-mono); }} .chart .lab {{ fill:var(--fg); font-size:12px; }} .chart .lab.pick {{ font-weight:600; }}
.chart .ok {{ fill:var(--ok); }} .chart .bad {{ fill:var(--bad); opacity:.75; }} .chart .pickbar {{ stroke:var(--fg); stroke-width:1.2; }}
table {{ border-collapse:collapse; width:100%; font-size:.92rem; }}
th, td {{ text-align:left; padding:6px 8px; border-bottom:1px solid var(--line); vertical-align:top; }}
th {{ font-weight:500; color:var(--mute); width:38%; }}
td.num {{ font-family:var(--f-mono); font-variant-numeric:tabular-nums; white-space:nowrap; }}
.tablewrap {{ overflow-x:auto; }}
.key {{ display:flex; flex-wrap:wrap; gap:14px; font-size:.85rem; color:var(--mute); }}
.key span::before {{ content:""; display:inline-block; width:10px; height:10px; margin-right:6px; vertical-align:-1px; background:var(--c); }}
code {{ font-family:var(--f-mono); font-size:.88em; }}
</style>
<main>
<header>
<h1>Scout camera mount v2</h1>
<p class="lede">Topology-generated, one PETG print of {chk['mass_g']:.1f} g. Every Scout rod is now 3 mm, and the 3 mm cage alone rings at {fr['cage_alone_modes_hz'][0]:.0f} Hz, so the camera hangs from the core: a tie through the bottom-front cell to a foot on the plate, held by the belly mount's front 2 bolts. Two snap clips on rod 14-15 next to nodes 14 and 15 pin the cage nose to the mount. First mode {fr['first_mode_hz']:.0f} Hz (F), gate 100.</p>
</header>
<section>
<h2>To scale, assembled</h2>
<div class="key"><span style="--c:{MOUNT}">mount v2</span><span style="--c:{CAGE}">3.0 rods and nodes</span><span style="--c:{CAM}">camera and field</span><span style="--c:{CLIP}">clips</span><span style="--c:{PROP}">prop disks</span><span style="--c:{PLATE}">plate</span></div>
<div class="sheet"><img alt="Side, plan and front views of the camera mount on the Scout cage, to scale" src="data:image/png;base64,{png}"></div>
</section>
<section class="two">
<div>
<h2>System pass: where the camera can hang</h2>
<div class="chartwrap">{concept_svg(con)}</div>
<p>First mode of the assembled Scout per concept, cage frame model with every rod 3.0 (F on E). Clips alone, mid-span or next to nodes, stay under 51 Hz because the cage itself is soft. The core tie alone holds the camera but leaves the cage nose at 58 Hz. Tie plus 2 near-node clips passes and stiffens the cage too.</p>
</div>
<div>
<h2>Checks</h2>
<div class="tablewrap"><table>{crow}</table></div>
</div>
</section>
<section>
<h2>Breaking order, a load on the hood centre</h2>
<div class="tablewrap"><table><tr><th>Direction</th><th>Lets go first</th><th>at</th><th>Then</th><th>at</th></tr>{order_rows}</table></div>
<p>E strengths: clip pop from the fork model, foot friction 0.3 on 2 x 300 N preload, belly mount inserts 600 N, PETG 43 MPa, carbon 600 MPa or Euler.</p>
</section>
<section class="two">
<div>
<h2>Fitting</h2>
<ol>
<li>Take out the belly mount's 2 front plate bolts at (30, ±13).</li>
<li>Camera onto the 4 bosses: M2.5 x 8 through the board, 1.0 nylon washers under it, into heat-set inserts.</li>
<li>Push the mount on from the nose along −x: the tie passes the bottom-front cell, the foot slides onto the plate top, both clips snap onto rod 14-15.</li>
<li>2 M3 x 12 with washers down through the foot's slots into the belly mount's inserts (key from above, through the lane between the props).</li>
</ol>
</div>
<div>
<h2>Why these choices</h2>
<ul>
<li>Clip points: rods 14-16 and 14-17 crowd node 14, so the clips sit 23.7 from its centre (y ±{cm.CLIP_Y}), the closest that clears both.</li>
<li>Slit-fork clips: a C clip on a 3.2 bore needs a 0.12 snap to stay under 30 MPa. Long prongs take a 0.25 snap at {sn['prong_sigma_MPa']:.0f} MPa.</li>
<li>Foot slots ±1.5 in x: the clips set x, so the cage's build tolerance never preloads the rod.</li>
<li>Only the bottom-front cell reaches the plate: node 16 sits on the centre line, and every member's install sweep is a keep-out.</li>
</ul>
</div>
</section>
</main>
"""
    open(os.path.join(RUN, "concept.html"), "w").write(page)
    print("wrote", os.path.join(RUN, "concept.html"), f"{len(page) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
