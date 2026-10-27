"""Overseer arms v1: the page (plan of the drone with the four arms, side view of each arm in its own frame, cases,
alternatives, results when the runs are built). Drawn from the real geometry of `arm_topo_v1.py` and layout v5.

python3 nox/overseer/arm_view_v1.py [front_mesh rear_mesh]   -> runs/overseer_arms/v1/arms.html
"""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf
import arm_topo_v1 as A
from topo.part import tm, man

OUT = os.path.join(ROOT, "runs", "overseer_arms", "v1")
P = os.path.join(A.LAYOUT, "parts")


def polys_plan(m):
    return [np.asarray(p) for p in m.project().to_polygons()]


def polys_side(m):
    """x-z silhouette of an arm-frame manifold (rotate -90 about x: z -> y)"""
    return [np.asarray(p) for p in m.rotate([-90.0, 0.0, 0.0]).project().to_polygons()]


def path(polys, tf):
    d = []
    for p in polys:
        q = [tf(x, y) for x, y in p]
        d.append("M" + " L".join(f"{a:.1f},{b:.1f}" for a, b in q) + " Z")
    return " ".join(d)


def plan_svg(meshes):
    W, H, S = 720, 640, 1.0                       # 1 px per mm; drone x up the page, y to the left
    cx, cy = 360, 400
    tf = lambda x, y: (cx - y * S, cy - x * S)
    out = [f'<svg viewBox="0 0 {W} {H}" class="dwg" role="img" aria-label="Plan of the Overseer with the four arms">']
    for nm, cls in (("prop_1", "prop"), ("prop_2", "prop"), ("prop_3", "prop"), ("prop_4", "prop")):
        t = trimesh.load(os.path.join(P, nm + ".stl"), force="mesh")
        c = t.bounds.mean(0); r = (t.bounds[1, 0] - t.bounds[0, 0]) / 2
        x, y = tf(c[0], c[1])
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" class="{cls}"/>')
    for nm, cls in (("bottom_plate", "plate"), ("top_plate", "plate"), ("hub_electronics__fc__pi_5___deck__pdb__u", "elec"),
                    ("apex___cam_bracket", "elec")):
        t = trimesh.load(os.path.join(P, nm + ".stl"), force="mesh")
        if nm.startswith("apex"):
            t = [b for b in t.split(only_watertight=False) if b.bounds[1, 0] < 300][0]
        out.append(f'<path d="{path(polys_plan(man(t)), tf)}" class="{cls}"/>')
    for m, cls in meshes:
        out.append(f'<path d="{path(polys_plan(m), tf)}" class="{cls}"/>')
    for k, (mx, my) in enumerate(A.motors_xy()):
        x, y = tf(mx, my)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="13.9" class="motor"/>'
                   f'<text x="{x:.1f}" y="{y + 4:.1f}" class="lbl c">{k + 1}</text>')
    x0, y0 = tf(0, 0)
    out.append(f'<path d="M{x0:.1f},{y0 + 40:.1f} L{x0:.1f},{y0 - 230:.1f}" class="axis"/>'
               f'<text x="{x0 + 6:.1f}" y="{y0 - 222:.1f}" class="lbl">+x nose</text>')
    sx = 40
    out.append(f'<path d="M{sx},{H - 24} h100" class="scale"/><text x="{sx}" y="{H - 30}" class="lbl">100 mm</text>')
    out.append("</svg>")
    return "".join(out)


def side_svg(which, B, F, built=None):
    S = 2.2
    x_lo, x_hi = -30, F["L"] + 30
    W = (x_hi - x_lo) * S + 20
    H = 75 * S + 70
    z0 = 30 + 62 * S
    tf = lambda x, z: (10 + (x - x_lo) * S, z0 - z * S)
    out = [f'<svg viewBox="0 0 {W:.0f} {H:.0f}" class="dwg" role="img" aria-label="Side view of the {which} arm">']
    out.append(f'<path d="{path(polys_side(B["env"]), tf)}" class="env"/>')
    if built is not None:
        out.append(f'<path d="{path(polys_side(built), tf)}" class="built"/>')
    out.append(f'<path d="{path(polys_side(B["drawn"]), tf)}" class="drawn"/>')
    out.append(f'<path d="{path(polys_side(B["cuts"] ^ B["env"].hull()), tf)}" class="cut"/>')
    out.append(f'<path d="{path(polys_side(B["sup"]), tf)}" class="sup"/>')
    if not B["rods"].is_empty():
        out.append(f'<path d="{path(polys_side(B["rods"]), tf)}" class="rod"/>')
    # motor and prop
    a, b = tf(F["L"] - 13.85, 71); c, d = tf(F["L"] + 13.85, 45)
    out.append(f'<rect x="{a:.1f}" y="{b:.1f}" width="{c - a:.1f}" height="{d - b:.1f}" class="motorside"/>')
    xt, zt = tf(F["L"], 45)
    out.append(f'<path d="M{xt:.1f},{zt + 60:.1f} L{xt:.1f},{zt + 4:.1f}" class="arrow" marker-end="url(#ah)"/>'
               f'<text x="{xt - 8:.1f}" y="{zt + 50:.1f}" class="lbl" text-anchor="end">thrust 21 N, yaw ±0.5 N·m</text>')
    for z in (0, 45):
        p, q = tf(x_lo + 2, z)
        out.append(f'<text x="{p:.1f}" y="{q + (14 if z == 0 else -4):.1f}" class="lbl">z {z}</text>')
    p, q = tf(0, -6); r_, s_ = tf(F["L"], -6)
    out.append(f'<path d="M{p:.1f},{q:.1f} H{r_:.1f}" class="dim"/><text x="{(p + r_) / 2:.1f}" y="{q + 14:.1f}" '
               f'class="lbl c">{F["L"]:.1f} root centre to motor</text>')
    out.append('<defs><marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
               '<path d="M0,0 L10,5 L0,10 Z" class="ahead"/></marker></defs></svg>')
    return "".join(out)


CSS = """
:root{--bg:#f6f7f8;--fg:#1b2026;--mut:#5c6670;--line:#c9d0d6;--panel:#ffffff;--acc:#1f6f8b;--warn:#a5531b;
--env:rgba(31,111,139,.10);--envs:#1f6f8b;--drawn:#3b4652;--built:#9aa9b6;--cut:#f6f7f8;--sup:#2f8f4e;--prop:rgba(120,130,140,.10);
--elec:rgba(165,83,27,.22);--rod:#111417;--plate:rgba(60,70,80,.10);
--f-body:"IBM Plex Sans",system-ui,sans-serif;--f-mono:"IBM Plex Mono",ui-monospace,monospace}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#121518;--fg:#e3e7ea;--mut:#98a3ad;--line:#2d343a;--panel:#191d21;
--acc:#5fb3cf;--warn:#e08a4f;--env:rgba(95,179,207,.13);--envs:#5fb3cf;--drawn:#c4ccd3;--built:#6d7c88;--cut:#121518;--sup:#5cc47f;
--prop:rgba(160,170,180,.08);--elec:rgba(224,138,79,.25);--plate:rgba(200,210,220,.08);--rod:#f2f4f5;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#121518;--fg:#e3e7ea;--mut:#98a3ad;--line:#2d343a;--panel:#191d21;--acc:#5fb3cf;--warn:#e08a4f;
--env:rgba(95,179,207,.13);--envs:#5fb3cf;--drawn:#c4ccd3;--built:#6d7c88;--cut:#121518;--sup:#5cc47f;--prop:rgba(160,170,180,.08);
--elec:rgba(224,138,79,.25);--plate:rgba(200,210,220,.08);--rod:#f2f4f5;color-scheme:dark}
body{background:var(--bg);color:var(--fg);font:15px/1.5 var(--f-body);padding-left:max(16px,env(safe-area-inset-left,0px));
padding-right:max(16px,env(safe-area-inset-right,0px));padding-block:24px 48px}
main{max-width:1060px;margin:0 auto;display:grid;gap:28px}
h1{font-size:26px;margin:0;text-wrap:balance} h2{font-size:18px;margin:0 0 8px;text-wrap:balance}
p{margin:0;max-width:68ch} .mut{color:var(--mut)} .eyebrow{font:12px var(--f-mono);letter-spacing:.06em;text-transform:uppercase;color:var(--acc)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,460px),1fr));gap:20px}
figure{margin:0;background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:12px;min-width:0;overflow-x:auto}
figcaption{font-size:13px;color:var(--mut);margin-top:6px}
.dwg{width:100%;height:auto;display:block}
.prop{fill:var(--prop);stroke:var(--line);stroke-dasharray:4 4}.plate{fill:var(--plate);stroke:var(--mut);stroke-width:.6}
.elec{fill:var(--elec);stroke:var(--warn);stroke-width:.6}.env{fill:var(--env);stroke:var(--envs);stroke-width:1}
.drawn{fill:var(--drawn)}.built{fill:var(--built);stroke:var(--drawn);stroke-width:.5}.cut{fill:var(--cut)}
.sup{fill:var(--sup)}.rod{fill:var(--rod)}.motor{fill:var(--panel);stroke:var(--fg);stroke-width:1}.motorside{fill:none;stroke:var(--fg);stroke-dasharray:3 3}
.axis{stroke:var(--mut);stroke-dasharray:2 4}.scale,.dim{stroke:var(--fg);stroke-width:1.2}.arrow{stroke:var(--acc);stroke-width:2}
.ahead{fill:var(--acc)}.lbl{font:11px var(--f-mono);fill:var(--mut)}.c{text-anchor:middle}
.key{display:flex;flex-wrap:wrap;gap:6px 16px;font-size:13px;color:var(--mut)}.key i{display:inline-block;width:12px;height:12px;
border-radius:2px;margin-right:6px;vertical-align:-1px}
.tw{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:14px;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--line);vertical-align:top}th{font-weight:600;color:var(--mut);font-size:13px}
.pill{font:12px var(--f-mono);padding:2px 8px;border-radius:10px;border:1px solid var(--line)}
.ok{color:var(--sup);border-color:var(--sup)}.no{color:var(--warn);border-color:var(--warn)}
"""


def page(results=None, built=None):
    meshes = []
    Bs = {}
    for which in ("front", "rear"):
        B, F = A.bodies(which)
        Bs[which] = (B, F)
        for mir in (False, True):
            src = built[which] if built else None
            if src is not None:
                meshes.append((man(A.arm_to_drone(tm(src), F, mir)), "built"))
            else:
                meshes.append((man(A.arm_to_drone(tm(B["env"]), F, mir)), "env"))
            meshes.append((man(A.arm_to_drone(tm(B["drawn"]), F, mir)), "drawn"))
            if "rods" in B and not B["rods"].is_empty():
                meshes.append((man(A.arm_to_drone(tm(B["rods"]), F, mir)), "rod"))
    res = ""
    if results:
        rows = "".join(
            f"<tr><td>{w}</td><td>{r['mass_g']:.1f} g</td><td>{r['worst']:.2f} ({r['worst_case']})</td>"
            f"<td>{r['flight']:.2f}</td><td>{r['modes'][0]['Hz']:.0f} Hz {r['modes'][0]['kind']}</td>"
            f"<td><span class='pill {'ok' if r['solid'] else 'no'}'>{'PASS' if r['solid'] else 'FAIL'}</span></td></tr>"
            for w, r in results.items())
        res = ("<section><h2>Results</h2><div class='tw'><table><tr><th>Arm</th><th>Mass (M)</th><th>Worst index (case)</th>"
               "<th>Worst flight / landing</th><th>First mode (E)</th><th>solid_check</th></tr>" + rows + "</table></div></section>")
    cases = [("Max thrust", "21 N up at the motor + yaw ±0.5 N·m", "flight", "≤ 0.25"),
             ("Manoeuvre", "max thrust + 2 g lateral (4 directions) + gyro 0.2 N·m", "flight", "≤ 0.25"),
             ("Landing", "6 g down on the motor, prop and screws (96 g)", "flight", "≤ 0.25"),
             ("Crash", "35 g on the motor, 14 directions (33 N)", "crash", "≤ 0.5"),
             ("Blow", "150 N on the motor end, 6 directions (past the cage)", "crash", "≤ 0.5"),
             ("Handling", "100 N on the motor pad, 6 directions (and on the 4 inserts alone)", "misuse", "≤ 0.5"),
             ("Vibration", "every first mode, motor, prop and rods on, root on the 4 inserts", "", "≥ 190 Hz")]
    crow = "".join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td><td>{d}</td></tr>" for a, b, c, d in cases)
    alts = [("Chosen: front pair a PETG topology arm round 4 x 3.0 carbon chords (a tapered space frame from shoulders at "
             "±35 to ±12 at the motor); rear pair PETG topology only", "front ≥ 190 Hz every mode, ~60 g with rods (E, checked on the "
             "built arm)", "the rods carry the bending both ways, the topology joins them to the plates and the motor; rear is 98 long "
             "and stiff without rods"),
            ("Alt 1: PETG topology only, front too", "~130..190 Hz at ~60 g (concept review hand bound 190 Hz; a plain 30 wide box "
             "read lateral 130 Hz, C)", "the 1P band reaches 158 Hz; the mass to clear it grows the tip inertia with it"),
            ("Alt 2: carbon tube arm (16/14) with a printed root and motor clamp", "~115 Hz (E), ~45 g",
             "inside the 1P band; a ~25 mm tube to reach 190 Hz, and only 6 / 3 mm stock is kept")]
    arow = "".join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in alts)
    sides = "".join(f"<figure>{side_svg(w, *Bs[w], built[w] if built else None)}<figcaption>{w.capitalize()} arm, side view in its "
                    f"own frame (root left, motor right). Arm {'1, mirror 4' if w == 'front' else '2, mirror 3'}.</figcaption></figure>"
                    for w in ("front", "rear"))
    return f"""<title>Overseer Arms v1</title><meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;600&display=swap">
<style>{CSS}</style><main>
<header><div class="eyebrow">Overseer · printed arms · topology v1</div><h1>Overseer arms v1</h1>
<p class="mut">Layout v5 / cage v3 (27 Oct). Front pair r 244, rear pair r 160. Each right arm is the mirror of the left one.
Cage nodes, spokes and L2 brace sockets are left off for now (Spark 27 Oct). Concept review 1 FAIL (front stiffness, supports, mirror holes, spoke path): fixed below. The ESC is a 4-in-1 under the FC; only motor leads run on the arms. Strength generous (Spark 27 Oct): index targets half the drone default, handling 100 N, a 150 N blow on the motor end.</p></header>
{res}
<figure>{plan_svg(meshes)}<div class="key"><span><i style="background:var(--env);border:1px solid var(--envs)"></i>design envelope</span>
<span><i style="background:var(--built)"></i>built arm</span><span><i style="background:var(--drawn)"></i>drawn: root blocks that bolt between the top and bottom plates (M3 inserts), motor pad, rod sleeves</span><span><i style="background:var(--rod)"></i>3.0 carbon rods</span>
<span><i style="background:var(--elec)"></i>hub electronics and L2 keel hub (kept 1 mm clear)</span><span><i style="background:var(--plate)"></i>plates</span>
<span><i style="background:var(--prop)"></i>prop disks</span></div>
<figcaption>Plan, drone frame, nose up. 1 px = 1 mm.</figcaption></figure>
<div class="grid">{sides}</div>
<div class="key"><span><i style="background:var(--sup)"></i>supports: plate contact faces</span>
<span><i style="background:var(--cut);border:1px solid var(--line)"></i>cuts: insert holes, motor holes, screw-head pocket</span></div>
<section><h2>Load cases</h2><div class="tw"><table><tr><th>Case</th><th>Load</th><th>Class</th><th>Target</th></tr>{crow}</table></div>
<p class="mut">Rotor band at 3380 g (E): 1P 113..158 Hz, 2P 227..317 Hz. The 190 Hz target sits above every 1P.</p></section>
<section><h2>Alternatives weighed</h2><div class="tw"><table><tr><th>Option</th><th>First mode, mass</th><th>Why</th></tr>{arow}</table></div></section>
</main>"""


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    results = built = None
    if len(sys.argv) > 2:
        built = {w: man(trimesh.load(sys.argv[1 + i], force="mesh")) for i, w in enumerate(("front", "rear"))}
        results = json.load(open(os.path.join(OUT, "results.json")))
    open(os.path.join(OUT, "arms.html"), "w").write(page(results, built))
    print("wrote", os.path.join(OUT, "arms.html"))
