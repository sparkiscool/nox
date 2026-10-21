"""Rewrites scout_v21_layout.html: CAD viewer (GLB) + topology section + node masses (M)."""
import json, os, re
D = os.path.dirname(os.path.abspath(__file__)); ENG = os.path.join(D, "..", "engine")
html = open(os.path.join(D, "scout_v21_layout.v11.html")).read()
asm = json.load(open(os.path.join(D, "assembly.json")))
lant = json.load(open(os.path.join(D, "s21_lantern.json")))
ORDER = ["corner", "corner_tail", "side_tee", "axis_tee", "elbow_corner", "elbow_tee", "hub", "knee_nose", "knee_tail", "sled_nose", "sled_tail", "rail_sleeve"]
LABEL = dict(corner="Nose corner (2 ring ends, post, glove)", corner_tail="Tail corner (2 ring ends, post, glove)", side_tee="Side tee (ring through, post, glove, strut)",
             axis_tee="Nose / tail tee (ring through, post)", elbow_corner="Corner elbow (2 ring ends, post, spoke)", elbow_tee="Tee elbow (ring through, post, spoke)",
             hub="Hub (8 spokes, r 40)", knee_nose="Nose knee (glove, leg)", knee_tail="Tail knee (glove, sled)", sled_nose="Sled nose (rail, leg; peg)",
             sled_tail="Sled tail (rail, cross, glove; peg)", rail_sleeve="Rail sleeve (rail through, glove; peg)")
COUNT = dict(corner=2, corner_tail=2, side_tee=2, axis_tee=2, elbow_corner=4, elbow_tee=4, hub=1, knee_nose=2, knee_tail=2, sled_nose=2, sled_tail=2, rail_sleeve=2)
rows = []; tot = 0.0; n_nodes = 0; worst_all = 0
for k in ORDER:
    p = os.path.join(ENG, f"{k}.json")
    if not os.path.exists(p): rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n" colspan="5">not run</td></tr>'); continue
    j = json.load(open(p))
    if j.get("failed"):
        rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n" colspan="5">failed: {j["trail"]}</td></tr>'); continue
    tot += j["mass_g"] * COUNT[k]; n_nodes += COUNT[k]; worst_all = max(worst_all, j["worst"])
    steps = " → ".join(f'{t["mass"]:.1f} g / {t["worst"]:.2f}' + ("" if t["ok"] else " ✗") for t in j.get("trail", []))
    rows.append(f'<tr><td>{LABEL[k]}</td><td class="n">{COUNT[k]}</td><td class="n">{j["h"]}</td><td class="n">{j.get("gen_cm3", 0):.2f}</td>'
                f'<td class="n">{j["mass_g"]:.2f}</td><td class="n">{j["worst"]:.2f}</td><td class="t">{steps}</td></tr>')
drawn_node_g = 2 * 1.86 + 2 * 1.2 + 2 * 0.5     # side hubs (M), rail clips, camera clips (E)
cage_g = 42 + tot + drawn_node_g + 6 + 4 + 2
topo = f'''
<section class="structure">
  <h2>Topology run (noxopt, 19 Oct): the nodes generated, not drawn</h2>
  <p class="intro">Each node type ran the generative protocol (<code>nox-generative.md</code>): drawn set = rod sockets only (bore 3.15, OD 6.5, 10 deep; a peg on the sled nodes), cuts = the rod bores, envelope = sockets + 4 mm. Every socket not held is pushed 80 N (E: a 0.5 J hit on a 100 span at the rod's break load, pinned ends) in ±x ±y ±z, class crash; the ring rods (or the receiver peg) hold. Material PA12-CF, E 4500 / Xt 90 / Zt 36 MPa (E, the research's pick). The mass target steps down (×1, ×½, ×¼) while the as-built index stays under 1.0; the table keeps the lightest that passed with one body, function check 0 / 0 / 0 and no thin member.</p>
  <div class="tw"><table>
    <tr><th>Node</th><th>n</th><th>h</th><th>gen cm³</th><th>each, g (M)</th><th>index</th><th>mass steps (g / index)</th></tr>
    {"".join(rows)}
  </table></div>
  <p class="legend-note">{n_nodes} generated nodes {tot:.1f} g M (v11 budget ~50 g E), worst as-built index {worst_all:.2f} on the lightest passing step. Cage {cage_g:.0f} g E (rods {asm["rod_m"]:.2f} m 42 g, nodes {tot:.0f} M, side hubs and clips {drawn_node_g:.0f}, sleeves 6, strips 4, grommets 2). Layouts at h 1.25 (hub 2.0); the protocol's finer pass (h 1.0 / 0.5) is the print gate, not this run.</p>
</section>
'''
# ---- insert the topology section before the 3D stage
html = html.replace('<div class="wrap">\n  <div class="stage" id="stage">', topo + '\n<div class="wrap">\n  <div class="stage" id="stage">', 1)
# ---- bar: focus buttons + nodes-only + core toggle; loading note
html = html.replace('''      <button id="t-cage" aria-pressed="true">Cage</button><button id="t-props" aria-pressed="true">Props</button><button id="t-funnel" aria-pressed="false">Box funnel</button>
    </div>''', '''      <button id="t-cage" aria-pressed="true">Cage</button><button id="t-body" aria-pressed="true">Core</button><button id="t-props" aria-pressed="true">Props</button><button id="t-funnel" aria-pressed="false">Box funnel</button><button id="t-nodes" aria-pressed="false">Nodes only</button>
      <span class="brk"></span>
      <button data-focus="corner">Corner</button><button data-focus="tee">Side tee</button><button data-focus="elbow">Elbow</button><button data-focus="hub">Hub</button><button data-focus="knee">Knee</button><button data-focus="sled">Sled</button>
    </div>
    <div class="note" id="load-note">loading the model…</div>''')
html = html.replace('.bar .sp{flex:1 0 8px}', '.bar .sp{flex:1 0 8px}\n.bar .brk{flex-basis:100%;height:0}\n.note{position:absolute;right:10px;bottom:10px;font:11px var(--mono);color:var(--muted)}\n.intro{margin:0 0 10px;font-size:12.5px;color:var(--muted);max-width:110ch}\n.intro code{font-family:var(--mono);font-size:11.5px}\nth{font-size:11px;color:var(--muted);font-weight:500;border-top:0!important}')
# ---- legend
html = html.replace('<span><i style="background:var(--node)"></i>PETG node</span>', '<span><i style="background:var(--node)"></i>PA12-CF node, generated</span>')
# ---- mass block
html = re.sub(r'<div><b>~50 g</b>27 PA12-CF nodes</div><div><b>~102 g</b>cage, E</div>',
              f'<div><b>{tot:.0f} g</b>{n_nodes} generated nodes, M</div><div><b>~{cage_g:.0f} g</b>cage, E</div>', html)
html = html.replace('<p class="legend-note">Mass now ~102 g E (rods 42, nodes ~50, sleeves 6, strips 4, grommets 2).',
                    f'<p class="legend-note">Mass now ~{cage_g:.0f} g E (rods 42, nodes {tot:.0f} M after the topology run below, sleeves 6, strips 4, grommets 2).')
# ---- viewer script
js = open(os.path.join(D, "viewer.js")).read()
FOCUS = dict(corner=[106, 106, 46, 110, 2.2, .35], tee=[0, 112, 33.2, 110, 2.6, .3], elbow=[112, 0, 60, 100, .9, .4], hub=[0, 0, 85, 170, .6, .6],
             knee=[40, 32, -7, 110, 1.6, .25], sled=[-5, 32, -47, 150, 1.9, -.15])
js = js.replace("__PATHS__", json.dumps(lant["paths"])).replace("__FOCUS__", json.dumps(FOCUS))
a = html.index('<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>')
html = html[:a] + '<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>\n<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>\n<script>\n' + js + '</script>\n'
html = html.replace('header p{margin:0;color:var(--muted);max-width:60ch}', 'header p{margin:0;color:var(--muted);max-width:72ch}')
html = re.sub(r'(<header>\s*<h1>Scout v2\.1 layout</h1>\s*<p>)[^<]*', r'\1The structure first: a lantern of four rings on one rod size, the drone fitted inside it, and now every node generated by the topology run and assembled as CAD. mm, drone coords (x nose, y left, z up). Tags: P measured, M mesh, C CAD, E estimate.', html)
open(os.path.join(D, "scout_v21_layout.html"), "w").write(html)
print("patched", len(html), "nodes", n_nodes, round(tot, 1), "g; cage", round(cage_g))
