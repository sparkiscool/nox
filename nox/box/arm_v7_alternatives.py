"""Swap arm v7 round 3c: the alternatives side by side, to scale, for the concept check (24 Oct 2026, swap arm system worker).
A section along the axle (Y across, r up) through head A at the drone insert: the carriage stack, the rail, the Overseer's receiver end and
its cartridge, for (1) the low carriage stack at Y_C 165 (chosen) and (2) round 2's carriage at Y_C 177 (the least Y_C it clears at).
Numbers from runs/arm/v7/system.json (chosen) and runs/arm/v7/alt_roundtwo_carriage_yc177.json (`NOX_ARM_STACK=v6 NOX_YC=177`), the
axle / detent trade from swap_system.power_cut_head_out (5:1 x1.5 chosen; 4:1 x1.8 the hub's first DEFAULT).
python3 arm_v7_alternatives.py -> runs/arm/v7/alternatives.html"""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
ROOT = os.path.normpath(os.path.join(HERE, "..", "..")); RUN = os.path.join(ROOT, "runs", "arm", "v7")
import arm_v6_layout as S6, arm_v7_layout as S7

LOW = S7.LOW
R2 = S6.v6_values()
STACKS = {
    "low": dict(title="1. Low carriage, Y_C 165 (chosen)", Y_C=165.0, rail=(54.0, LOW["RAIL_TOP"]), keel=LOW["KEEL"]["y"], wheel=LOW["WHEEL_Y"],
                plate=LOW["PLATE_Y"], hw_top=LOW["PLATE_Y"][1], nut=LOW["NUT_Y"]),
    "r2": dict(title="2. Round 2 carriage, Y_C 177", Y_C=177.0, rail=R2["RAIL_Y"], keel=None, wheel=R2["WHEEL_Y"], plate=R2["PLATE_Y"],
               hw_top=R2["PLATE_Y"][1] + 7.5, nut=R2["NUT_Y"]),
}
REC_HALF_ARM, CART_ARM, Z0, R_SEAT = 89.65, 85.5, 108.6, 113.0      # Overseer receiver / body from the grip line (C, latch 258becf)


def section(x0, st):
    s = 2.2; Ybase, rbase = 30.0, 20.0
    X = lambda y: x0 + 20 + (y - Ybase) * s; Yp = lambda r: 420 - (r - rbase) * s
    def rect(y0, y1, r0, r1, cls):
        return f'<rect x="{X(y0)}" y="{Yp(r1)}" width="{(y1 - y0) * s}" height="{(r1 - r0) * s}" class="{cls}"/>'
    g = [f'<text x="{x0 + 20}" y="24" class="h">{st["title"]}</text>']
    g.append(rect(Ybase, 54.0, rbase, 88.0, "hub"))
    g.append(rect(st["rail"][0], st["rail"][1], 29.0, 151.0, "rail"))
    if st["keel"]: g.append(rect(st["keel"][0], st["keel"][1], 89.0, 151.0, "rail"))
    g.append(rect(st["wheel"][0], st["wheel"][1], Z0 + 4 - 12, Z0 + 32 + 12, "wheel"))
    g.append(rect(st["plate"][0], st["plate"][1], Z0, Z0 + 40, "plate"))
    if st["hw_top"] > st["plate"][1]: g.append(rect(st["plate"][1], st["hw_top"], Z0 + 10, Z0 + 26, "hw"))
    y_rec = st["Y_C"] - REC_HALF_ARM; y_cart = st["Y_C"] - CART_ARM
    g.append(rect(y_rec, 260.0, R_SEAT - 4.0, R_SEAT + 70.0, "rec"))
    g.append(rect(y_cart, 260.0, R_SEAT, R_SEAT + 58.6, "cart"))
    g.append(f'<line x1="{X(st["Y_C"])}" y1="{Yp(rbase)}" x2="{X(st["Y_C"])}" y2="{Yp(200)}" class="grip"/>')
    g.append(f'<text x="{X(st["Y_C"]) + 4}" y="{Yp(196)}" class="s">grip line Y_C {st["Y_C"]:g}</text>')
    g.append(f'<text x="{X(y_rec)}" y="{Yp(R_SEAT + 74)}" class="s">Overseer receiver from Y {y_rec:.1f}</text>')
    g.append(f'<text x="{X(st["rail"][0])}" y="{Yp(25)}" class="s">rail, carriage, wheels</text>')
    g.append(f'<text x="{X(Ybase) + 4}" y="{Yp(80)}" class="s">hub</text>')
    clear = y_rec - st["hw_top"]
    g.append(f'<text x="{x0 + 20}" y="44" class="s">clearance carriage → Overseer receiver: {clear:.2f} (hardware top Y {st["hw_top"]:g})</text>')
    for yy in (50, 100, 150, 200, 250):
        g.append(f'<text x="{X(yy)}" y="{Yp(rbase) + 14}" class="s" text-anchor="middle">Y {yy}</text>')
    return "\n".join(g)


def main():
    sysj = json.load(open(os.path.join(RUN, "system.json"))); alt = json.load(open(os.path.join(RUN, "alt_roundtwo_carriage_yc177.json")))
    g1, g2 = sysj["xt60_guided"]["packs"], alt["guided"]["packs"]
    ix = sysj["power_cut_head_out"]["index"]["spring_scales"]
    rows1 = [("Grip line Y_C", "165", "177"), ("Box length in Y", "the reference", "+12"),
             ("Carriage → Overseer receiver", f'{165 - REC_HALF_ARM - LOW["PLATE_Y"][1]:.2f}', f'{177 - REC_HALF_ARM - STACKS["r2"]["hw_top"]:.2f}'),
             ("Free tilt at the mate (Scout / Overseer)", f'{g1["scout"]["free_tangent_deg"]:.2f}° / {g1["overseer"]["free_tangent_deg"]:.2f}°',
              f'{g2["scout"]["free_tangent_deg"]:.2f}° / {g2["overseer"]["free_tangent_deg"]:.2f}°'),
             ("Guide moment at stub 5 mrad (Scout / Overseer)", f'{g1["scout"]["receiver_moment_Nm_stub_5"]} / {g1["overseer"]["receiver_moment_Nm_stub_5"]} N·m',
              f'{g2["scout"]["receiver_moment_Nm_stub_5"]} / {g2["overseer"]["receiver_moment_Nm_stub_5"]} N·m'),
             ("XT60 with the latch's guide", f'{g1["scout"]["per_axis_deg"]}° / {g1["overseer"]["per_axis_deg"]}°', f'{g2["scout"]["per_axis_deg"]}° / {g2["overseer"]["per_axis_deg"]}°'),
             ("Parts", "countersunk axles, thin nuts, rail keel", "as drawn in round 2")]
    t1 = "".join(f"<tr><th>{a}</th><td>{b}</td><td>{c}</td></tr>" for a, b, c in rows1)
    t2 = "".join(f"<tr><td>5:1, springs x{r['scale']:g}</td><td>{r['index_margin']}</td><td>{r['parked_margin']}</td><td>{r['head_out_margin']}</td><td>{r['head_out_creep_deg_s']}°/s, {r['impact_N']:g} N</td></tr>" for r in ix)
    t2 += "<tr><td>4:1, springs x1.8 (the hub's first DEFAULT)</td><td>1.04</td><td>1.62</td><td>0.98</td><td>3.9°/s, 13 N</td></tr>"
    svg = section(0, STACKS["low"]) + section(560, STACKS["r2"])
    html = f"""<title>Arm v7 alternatives</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
:root {{ --bg:#f6f5f1; --fg:#1d232a; --muted:#5d6670; --line:#c9ccd0; --card:#fff; --hub:#b9c1c9; --rail:#7d8a96; --wheel:#2f6db3; --plate:#3d4a57; --hw:#c47a1d; --rec:#d9a3a3; --cart:#a8c69a; --grip:#b33a3a; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --hub:#3b444d; --rail:#6b7884; --wheel:#6aa5ec; --plate:#95a5b5; --hw:#e3a24a; --rec:#8a4f4f; --cart:#4f6e45; --grip:#e06a6a; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#14181c; --fg:#e7eaee; --muted:#9aa4ad; --line:#323a42; --card:#1b2026; --hub:#3b444d; --rail:#6b7884; --wheel:#6aa5ec; --plate:#95a5b5; --hw:#e3a24a; --rec:#8a4f4f; --cart:#4f6e45; --grip:#e06a6a; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding-left:max(16px, env(safe-area-inset-left, 0px)); padding-right:max(16px, env(safe-area-inset-right, 0px)); padding-block:20px }}
main {{ max-width:1160px; margin:0 auto; display:grid; gap:20px }} h1 {{ font-size:1.5rem; margin:0 }} h2 {{ font-size:1.1rem; margin:0 0 8px }}
p {{ margin:0; color:var(--muted); max-width:70ch }} .card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; min-width:0 }}
.scroll {{ overflow-x:auto }} table {{ border-collapse:collapse; width:100%; font-size:14px }} th, td {{ text-align:left; padding:6px 8px; border-top:1px solid var(--line) }}
svg .h {{ font-weight:600; font-size:14px; fill:var(--fg) }} svg .s {{ font-size:11px; fill:var(--muted) }}
.hub {{ fill:var(--hub) }} .rail {{ fill:var(--rail) }} .wheel {{ fill:var(--wheel); opacity:.8 }} .plate {{ fill:var(--plate) }} .hw {{ fill:var(--hw) }}
.rec {{ fill:var(--rec); opacity:.7 }} .cart {{ fill:var(--cart); opacity:.9 }} .grip {{ stroke:var(--grip); stroke-dasharray:4 3 }}
</style>
<main><header><h1>Arm v7 alternatives</h1><p>Nox swap arm, round 3c. Section along the axle through head A at the drone insert, to scale (2.2 px per mm): what the Overseer's receiver end must clear, and what each choice costs.</p></header>
<section class="card"><h2>Carriage stack and grip line</h2><div class="scroll"><svg viewBox="0 0 1120 460" width="1120" height="460" role="img" aria-label="two carriage stacks">{svg}</svg></div></section>
<section class="card"><h2>Numbers</h2><div class="scroll"><table><tr><th></th><th>1. Low carriage, Y_C 165</th><th>2. Round 2 carriage, Y_C 177</th></tr>{t1}</table></div></section>
<section class="card"><h2>Axle drive and detent springs (unpowered arm)</h2><div class="scroll"><table><tr><th>Option</th><th>Index past a notch</th><th>Parked hold</th><th>Head out at a horizontal Overseer bay</th><th>Creep with the axle on the brake relay</th></tr>{t2}</table></div><p>Chosen: 5:1, springs x1.5 (index 1.43, parked 1.31).</p></section>
<section class="card"><p>Source: <code>nox/box/arm_v7_alternatives.py</code>; numbers <code>runs/arm/v7/system.json</code>, <code>runs/arm/v7/alt_roundtwo_carriage_yc177.json</code>; requirements <code>nox/dimensions.md</code> "System box swap: arm".</p></section></main>"""
    open(os.path.join(RUN, "alternatives.html"), "w").write(html)
    print("->", os.path.join(RUN, "alternatives.html"))


if __name__ == "__main__":
    main()
