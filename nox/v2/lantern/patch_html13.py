"""Builds scout_v21_layout.html (cage v13: thick drum parts, flat feet) from the v11 page: v12 drawings, socket v2 + merged parts section, topology table
from engine/v12/*.json, glTF viewer with v12 focus targets, updated panels. Run after assemble12.py."""
import json, os, re
D = os.path.dirname(os.path.abspath(__file__)); ENG = os.path.join(D, "..", "engine")
html = open(os.path.join(D, "scout_v21_layout.v11.html")).read()
asm = json.load(open(os.path.join(D, "assembly13.json"))); lant = json.load(open(os.path.join(D, "s21_lantern12.json")))
def sub(old, new, count=1):
    global html
    assert old in html, old[:80]
    html = html.replace(old, new, count)

ORDER = ["corner_col", "corner_col_t", "side_col", "nose_col", "tail_col", "hub", "nose_leg", "knee_tail", "sled_tail", "rail_sleeve"]
LABEL = dict(corner_col="Nose corner column: 4 ring ends, glove, spoke", corner_col_t="Tail corner column", side_col="Side column: 2 rings through, glove, strut, spoke",
             nose_col="Nose column: split skid sleeves, shoulder through, spoke, 4 camera pads", tail_col="Tail column: 2 rings through, spoke",
             hub="Hub: 8 spokes + drawn ring r 36, 6 × 6", nose_leg="Nose leg: glove, sled rail, flat foot", knee_tail="Tail knee",
             sled_tail="Sled tail: rail, cross, glove, flat foot", rail_sleeve="Rail sleeve: rail through, glove")
COUNT = dict(corner_col=2, corner_col_t=2, side_col=2, nose_col=1, tail_col=1, hub=1, nose_leg=2, knee_tail=2, sled_tail=2, rail_sleeve=2)
rows = []; tot = 0.0; n = 0; worst_all = 0.0; drawn_tot = 0.0; failed = []
for k in ORDER:
    p = os.path.join(ENG, "v13", f"{k}.json")
    ver = "v13"
    if not os.path.exists(p): p = os.path.join(ENG, "v12", f"{k}.json"); ver = "v12"
    if not os.path.exists(p): rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n" colspan="5">not run</td></tr>'); failed.append(k); continue
    j = json.load(open(p))
    if j.get("failed"): rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n" colspan="5">failed</td></tr>'); failed.append(k); continue
    tot += j["mass_g"] * COUNT[k]; drawn_tot += j["drawn_g"] * COUNT[k]; n += COUNT[k]; worst_all = max(worst_all, j["worst"])
    steps = (" → ".join(f'{t["mass"]:.1f} / {t["worst"]:.2f}' + ("" if t["ok"] else " ✗") for t in j.get("trail", []) if "mass" in t) or f'{j["mass_g"]:.1f} / {j["worst"]:.2f}') + ("" if ver == "v13" else " (v12)")
    rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n">{j["gen_cm3"]:.2f}</td><td class="n">{j["mass_g"]:.2f}</td>'
                f'<td class="n">{j["worst"]:.2f}</td><td class="n">{j["drawn_in_keep"]:.1f}</td><td class="t">{steps}</td></tr>')
side_hubs = 2 * 1.86
rod_g = asm["rod_m"] * 11.0
cage_g = rod_g + tot + side_hubs + 5 + 3 + 2 + 2          # sleeves (OD 4) ~5, strips 76 ~3, grommets 2, epoxy 2 (E)
CL = asm["clear"]
def c(k): v = CL.get(k, [99, ""]); return f'{v[0]:.1f}'

# ---- header, concept, drawings
html = re.sub(r'(<header>\s*<h1>Scout v2\.1 layout</h1>\s*<p>)[^<]*', r'\1The structure first: a lantern of rings on one rod size with the drone fitted inside it. v13: thicker printed drum and crown (rods stay 3.0), flat feet on the sled, wider resin sockets, every part generated and checked against the box. mm, drone coords (x nose, y left, z up). Tags: P measured, M mesh, C CAD, E estimate.', html)
sub('<h2>Structure: the lantern</h2>', '<h2>Structure: the lantern (cage v13)</h2>')
sub("z −47. A rectangle of three rods 2.5 under the battery, y ±32, x −50..40. What the drone lands on. Open at the nose: the box's swap head rises there.",
    "z −47. Two rails and a tail cross 2.5 under the battery, y ±32, x −52..37.5, on 4 flat feet (bottoms z −52). What the drone lands on, on the ground or in the funnel. Open at the nose: the box's swap head rises there.")
sub("8 rods from the skid ring down the funnel's own 45° faces to the sled. Sides and tail true to the face; the nose knees in at z −7 because the box arm sweeps below it.",
    "8 rods from the skid ring down the funnel's own 45° faces to the sled. Sides and tail true to the face; at the nose the glove lands on a printed leg (x 37.5, z −7 to −47) because the box arm sweeps below z −9.5.")
sub("Two clipped-square rings, z 33.2 and z 60, 8 vertical posts. The lower ring carries the 8 fixed skid pads the funnel seats on; the drum wraps the props.",
    "Two clipped-square rings, z 33.2 and z 60, held by 8 printed columns (the posts were shorter than their two sockets). The lower ring carries the 8 skid strips the funnel seats on; the drum wraps the props.")
a = html.index('  <div class="views">'); b = html.index('</div></div>\n', html.index('class="topv"')) + len('</div></div>\n')
html = html[:a] + "  " + open(os.path.join(D, "views13.html")).read() + "\n" + html[b:]

# ---- research table: clip-in tees dropped, mass note
sub('<tr><td>Clip-in tees</td><td class="n">the 4 mid-span tees snap onto the rings instead of epoxy: the sacrificial joint</td><td class="t">0 g</td></tr>',
    '<tr><td>Clip-in tees <i>(dropped v12)</i></td><td class="n">the tees are now columns, one part with the elbows, and every joint is epoxied in socket v2; the rod is the fuse</td><td class="t">0 g</td></tr>')
sub('<p class="legend-note">Mass now ~102 g E (rods 42, nodes ~50, sleeves 6, strips 4, grommets 2).',
    f'<p class="legend-note">v13 mass ~{cage_g:.0f} g E (rods {rod_g:.0f}, {n} printed parts {tot:.0f} M, side hubs 4, sleeves 5, strips 3, grommets 2, epoxy 2).')

# ---- v12 section: sockets, merged parts, keep-outs, coupon
sock_svg = open(os.path.join(ENG, "socket_v2.svg")).read()
v12 = f'''
<section class="structure">
  <h2>v12: wider resin sockets, merged parts</h2>
  <div class="concept">
    <div><b>Socket v2</b><span>Bore 3.6 for the 3.0 rod: 0.3 of epoxy all round instead of 0.075. Four thin ribs on a 3.3 circle keep the rod centred and are the only thing that can bind; if a print comes out tight they shave. Two 3.9 lock rings give the epoxy a mechanical key (PA12 bonds poorly on its own). A bleed slot runs from the bottom to the mouth so trapped air and surplus resin come back out instead of pushing the rod back. A closed 2.0 floor is the rod stop and seals the epoxy at the bottom. OD 7.9: 2.0 of wall at the rings, the solid-check minimum.</span></div>
    <div><b>Merged</b><span>Corner node + post + elbow is one corner column; tee + post + elbow one tee column; knee + 40 leg + sled nose one nose leg. Rule (DEFAULT): a rod with under 20 mm bare between its sockets is printed. 27 nodes and 41 rods became {n} parts and {len(lant["rods"])} rods.</span></div>
    <div><b>Keep-outs</b><span>Every part now runs with the funnel faces (1.0 clear), the yawed 168 opening, props + 6, the box arm sweep, the swap head lane, receiver, plate and camera as cuts. v11 had none on the nodes: its nose leg sockets sat 1.2 mm in the arm sweep (nose x 40 → 37.5), and its tail knees left no room for a socket at the yawed opening ((−75, ±64) → (−66, ±52), swept until the sockets clear the 5° yawed rim); the tail cross moved x −50 → −52 so its sockets clear the swap head body.</span></div>
    <div><b>Camera</b><span>The v11 nose post at (112, 0) stood in front of the lens. The nose column splits its skid sleeve to y ±14 and carries the board on 4 pads (M2), so the camera rails are gone.</span></div>
  </div>
  <div class="sock">{sock_svg}</div>
  <p class="legend-note">Coupon before the nodes: <b>s2_socket_v2_coupon_v2</b> in Nox Print Files (7.4 g, 66 × 30 × 12, passes the solid check): bores 3.5 / 3.6 / 3.7 (1 / 2 / 3 notches), each standing and lying, lying mouths at the front edge. A 3.0 rod should slide in by hand without wobble; then epoxy a rod into the standing 3.6 (the plate seals its bottom like the floor does) and pull after 24 h. Sand the rod ends.</p>
</section>

<section class="structure">
  <h2>Topology run v13 (noxopt, 20 Oct)</h2>
  <p class="intro">Each part ran the generative protocol (<code>nox-generative.md</code>) on socket v2: drawn set = the sockets (plus the hub ring and the camera pads), cuts = bores and every keep-out, envelope = sockets + 4 mm. Every socket not held is pushed 80 N (E: a 0.5 J hit on a 100 span at the rod's break load) in ±x ±y ±z, class crash; the part hangs on the rods it really hangs on (both rings for a column, the sled rail for the sled parts). Columns add a drum-shear case: the shoulder sockets pushed 40 N sideways with only the skid ring held (E: the rest of a hit goes round the rings and down the spoke). Columns and the nose leg print on their side so bending runs along the layers. PA12-CF E 4500 / Xt 90 / Zt 36 MPa (E). A part that fails (more than one body, a thin member, index over 1.0) reruns once at 1.5 × the material.</p>
  <div class="tw"><table>
    <tr><th>Part</th><th>n</th><th>gen cm³</th><th>each, g (M)</th><th>index</th><th>drawn in keep-out, mm³</th><th>steps (g / index)</th></tr>
    {"".join(rows)}
  </table></div>
  <p class="legend-note">{n} printed parts {tot:.1f} g M ({drawn_tot:.0f} g of it is socket), worst as-built index {worst_all:.2f}. "Drawn in keep-out" is socket wall inside the 1.0 margin of a funnel face (never the face itself) or the camera pads against the board. Measured on the assembly: collar face {c("collar face")} mm, opening + rim at 5° yaw {c("opening + rim, yawed 5")}, camera lens {c("camera lens")}, props {c("props")}, box arm sweep {c("arm sweep (x <= 42)")}, swap head lane {c("head lane (y outside -26..14)")}. Layouts at h 1.25 (hub 2.0); the finer pass (h 1.0) is the print gate.</p>
  <p class="legend-note"><b>v13 (Spark 20 Oct: thicker structure around the prop guard, rods stay 3.0):</b> the drum columns and the hub carry a drawn skeleton (socket-size cores, a root from every socket floor, spines; the nose column's spines run at y ±14 with a crossbar at z 20.3, under the shoulder rod and over the lens window), members 3.0 minimum (side and nose columns 4.2), gen targets up, and no generated material hugs a socket wall away from its node (those lumps hung on thin members and failed the neck test). Hub ring r 36 × 6 × 6 behind the socket floors with rounded corners. Nose legs and sled tails rerun with the flat feet and a landing case (27 N, E: 6 g on one foot, straight and with half sideways). Every v13 part passes the current solid check, neck test included; rows marked (v12) are unchanged.</p>
</section>
'''
sub('<div class="wrap">\n  <div class="stage" id="stage">', v12 + '\n<div class="wrap">\n  <div class="stage" id="stage">')
sub('.bar .sp{flex:1 0 8px}', '.bar .sp{flex:1 0 8px}\n.bar .brk{flex-basis:100%;height:0}\n.note{position:absolute;right:10px;bottom:10px;font:11px var(--mono);color:var(--muted)}\n.intro{margin:0 0 10px;font-size:12.5px;color:var(--muted);max-width:110ch}\n.intro code{font-family:var(--mono);font-size:11.5px}\nth{font-size:11px;color:var(--muted);font-weight:500;border-top:0!important}\n.sock{margin-top:6px;background:var(--bg);border-radius:4px;padding:8px}\n.concept i{font-style:normal;color:var(--muted)}')
sub('''<button id="t-funnel" aria-pressed="false">Box funnel</button>
    </div>''', '''<button id="t-funnel" aria-pressed="false">Box funnel</button><button id="t-nodes" aria-pressed="false">Printed only</button>
      <span class="brk"></span>
      <button data-focus="corner">Corner column</button><button data-focus="side">Side column</button><button data-focus="nose">Nose column</button><button data-focus="hub">Hub</button><button data-focus="leg">Nose leg</button><button data-focus="sled">Sled</button><button data-focus="feet">Feet</button>
    </div>
    <div class="note" id="load-note">loading the model…</div>''')
sub('<button id="t-cage" aria-pressed="true">Cage</button>', '<button id="t-cage" aria-pressed="true">Cage</button><button id="t-body" aria-pressed="true">Core</button>')
sub('<span><i style="background:var(--node)"></i>PETG node</span>', '<span><i style="background:var(--node)"></i>PA12-CF printed part</span>')

# ---- panels
sub('<td class="n">nose window, board x 99, lens (114, 0, 41)</td><td class="t">C · lens length E, D18</td>', '<td class="n">nose column pads, board x 99, lens (114, 0, 41)</td><td class="t">C · lens length E, D18</td>')
a = html.index('<h2>Cage v11 lantern'); b = html.index('</section>', a)
html = html[:a] + f'''<h2>Cage v13 lantern (3.0 solid carbon, PA12-CF parts, no moving parts)</h2>
      <div class="tw"><table>
        <tr><td>Skid ring</td><td class="n">z 33.2, flats ±112, chamfer 100; TPU-sleeved; 8 UHMW strips 76 long at ±50 on the flats</td><td class="t">4 × 200</td></tr>
        <tr><td>Shoulder ring</td><td class="n">z 60, same outline; TPU-sleeved</td><td class="t">4 × 200</td></tr>
        <tr><td>Columns</td><td class="n">4 corner + 4 tee, printed, z 33.2..60, ring sockets both levels</td><td class="t">8 parts</td></tr>
        <tr><td>Spokes</td><td class="n">8 column tops → hub ring</td><td class="t">4 × 113, 4 × 76</td></tr>
        <tr><td>Hub</td><td class="n">printed, drawn octagon r 36 (6 × 6) + 8 sockets, z 85</td><td class="t">1 part</td></tr>
        <tr><td>Glove</td><td class="n">nose corners → nose legs (37.5, ±32, −7); tail corners → knees (−66, ±52, −0.8) → sled; side columns → rail sleeves</td><td class="t">2 × 109, 2 × 62 + 2 × 62, 2 × 113</td></tr>
        <tr><td>Sled</td><td class="n">rails y ±32, x −52 to 37.5, tail cross; receiver hangs on the rails at x 15 / −25 in TPU grommets</td><td class="t">2 × 88, 1 × 64</td></tr>\n        <tr><td>Feet</td><td class="n">4 flat pads on the nose legs and sled tails, bottoms z −52, x 29.5..41 / −59..−45, y ±27..39, 5 thick; stance 100 × 78, tip-over 35° side / 36° nose / 47° tail (E CG)</td><td class="t">in the parts</td></tr>
        <tr><td>Struts</td><td class="n">side columns → plate side hubs on TPU grommets</td><td class="t">2 × 63</td></tr>
      </table></div>
      <div class="mass">
        <div><b>{asm["rod_m"]:.2f} m</b>rod, {rod_g:.0f} g</div><div><b>{tot:.0f} g</b>{n} printed parts, M</div><div><b>~{cage_g:.0f} g</b>cage, E</div><div><b>{len(lant["rods"])}</b>rods</div><div><b>{c("props")}</b>min to props</div><div><b>{c("collar face")}</b>min to collar</div>
      </div>
    ''' + html[b:]
sub('<li><b>Flow sensor v3.3 / v3.4 spot sits in the box arm sweep.</b> Moved to z 2..10, x 63..85.</li>',
    '<li><b>Flow sensor v3.3 / v3.4 spot sits in the box arm sweep.</b> Moved to z 2..10, x 63..85.</li>\n        <li><b>v11 nodes ignored the box.</b> Nose leg sockets were 1.2 mm inside the arm sweep and the nose post stood in front of the lens. v12 runs every part with the keep-outs as cuts; nose x 40 → 37.5, camera on pads.</li>\n        <li><b>PA12 and epoxy.</b> Nylon bonds poorly by adhesion alone: socket v2 keys the epoxy mechanically (lock rings), sand the rod ends, and pull-test the coupon before trusting a joint. No clip-in sacrificial tee any more: the rod is the fuse.</li>\n        <li><b>Wider sockets meet the funnel at the skid ring.</b> The ring axis is 4.24 from the 45° face; an OD 7.9 socket’s lower outer edge is 0.41 from it (v11’s 6.5 had 1.4). The strips still touch first. If you want the 1.0 margin back: strips 3.0 proud instead of 2.5 (seat 0.7 higher, a shared catch number, your call).</li>\n        <li><b>Neck test: two parts left.</b> Every v13 part (columns, hub, nose legs, sled tails) passes the current solid check with the drawn skeleton. The tail knees and rail sleeves are still the v12 prints and still fail it at their socket roots; same fix, next run.</li>\n        <li><b>Feet need the box below them clear.</b> The feet are the drone’s lowest point, z −52 (1.0 under the v12 sled nodes). In the box nothing under the seated drone may rise above z −52 outside the swap head lane: box chat check.</li>')

# ---- viewer
js = open(os.path.join(D, "viewer.js")).read()
FOCUS = dict(corner=[106, 106, 46, 130, 2.2, .3], side=[0, 112, 46, 130, 2.6, .3], nose=[112, 0, 44, 140, .9, .25], hub=[0, 0, 85, 170, .6, .6],
             leg=[37.5, 32, -27, 130, 1.4, .15], sled=[-20, 32, -47, 170, 1.9, -.15], feet=[37.5, 32, -48, 70, 1.1, -.35])
js = js.replace("__PATHS__", json.dumps(lant["paths"])).replace("__FOCUS__", json.dumps(FOCUS))
a = html.index('<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>')
html = html[:a] + '<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>\n<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>\n<script>\n' + js + '</script>\n'
open(os.path.join(D, "scout_v21_layout.html"), "w").write(html)
print("page", len(html), "parts", n, round(tot, 1), "g; cage", round(cage_g), "failed", failed)
