"""Box landing v1 concept page: top view, groove section, recorded drops, capture chart (Spark 27 Oct).

Reads runs/landing/v1-concept/sweep.json + traj.json (landing_sim.py) and the v5 cage; writes
runs/landing/v1-concept/landing.html. Run: python3 nox/box/landing/landing_view.py
"""
import json, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import landing_sim as L

OUT = L.OUT
DEFAULT = ("kin", 30.0, 0.03)          # concept, flank, mu of the chosen coupling (set from the sweep below)


def pick(res, drone, concept, th, mu):
    for r in res:
        k = r["key"]
        if k[0] == drone and k[1] == concept and abs(k[2] - th) < 1e-6 and abs(k[3] - mu) < 1e-6:
            return r
    return None


def top_svg(sw, H):
    """Plan view to scale, ring frame (x forward = right, y left = up)."""
    d = json.load(open(os.path.join(L.V5, "cage_shell.json")))
    N = np.array(d["nodes"]) + L.RING_OFF
    from scipy.spatial import ConvexHull
    hull = N[ConvexHull(N[:, :2]).vertices][:, :2]
    S = 1.0
    X0, X1, Y0, Y1 = -330, 420, -365, 365
    def p(x, y):
        return f"{x * S:.1f},{-y * S:.1f}"
    out = [f'<svg viewBox="{X0} {-Y1} {X1 - X0} {Y1 - Y0}" class="plan" role="img" aria-label="Plan view of the landing interface to scale">']
    # grid every 50 mm
    for gx in range(-300, 421, 50):
        out.append(f'<line x1="{gx}" y1="{-Y1}" x2="{gx}" y2="{-Y0}" class="grid"/>')
    for gy in range(-350, 351, 50):
        out.append(f'<line x1="{X0}" y1="{-gy}" x2="{X1}" y2="{-gy}" class="grid"/>')
    out.append(f'<polygon points="{" ".join(p(*q) for q in hull)}" class="cage"/>')
    out.append('<text x="-300" y="-318" class="lbl mute">Overseer cage v3 in plan (637 x 673)</text>')
    out.append(f'<circle cx="0" cy="0" r="145" class="scout"/><text x="-60" y="-152" class="lbl mute">Scout disk ~290 (E)</text>')
    # V-blocks
    for f, a, c in L.kin_grooves():
        Wd = L.KIN_W
        u0, u1 = -L.KIN_LEN[1], L.KIN_LEN[0]
        corners = [f + a * u + c * v for u, v in ((u0, -Wd), (u1, -Wd), (u1, Wd), (u0, Wd))]
        out.append(f'<polygon points="{" ".join(p(*q) for q in corners)}" class="vblock"/>')
        g0, g1 = f + a * u0, f + a * u1
        out.append(f'<line x1="{g0[0]:.1f}" y1="{-g0[1]:.1f}" x2="{g1[0]:.1f}" y2="{-g1[1]:.1f}" class="groove"/>')
    hx, hy = L.HOLE[0] / 2, L.HOLE[1] / 2
    out.append(f'<rect x="{-hx}" y="{-hy}" width="{2 * hx}" height="{2 * hy}" class="hole"/>')
    rx, ry = L.RECEIVER[0] / 2, L.RECEIVER[1] / 2
    out.append(f'<rect x="{-rx}" y="{-ry}" width="{2 * rx}" height="{2 * ry}" class="rcv"/>')
    out.append(f'<text x="{-hx + 6}" y="{-hy + 14}" class="lbl">swap hole {L.HOLE[0]:.0f} x {L.HOLE[1]:.0f}</text>')
    out.append(f'<text x="{-rx + 6}" y="{ry - 6}" class="lbl mute">receiver {L.RECEIVER[0]} x {L.RECEIVER[1]}</text>')
    R = L.RING
    out.append(f'<polygon points="{" ".join(p(*q) for q in R)}" class="ring"/>')
    for q in R:
        out.append(f'<circle cx="{q[0]:.1f}" cy="{-q[1]:.1f}" r="{L.NODE_R}" class="node"/>')
    for i, (x, y) in enumerate(L.FEET):
        out.append(f'<circle cx="{x}" cy="{-y}" r="{L.FOOT_R}" class="foot"/>')
    out.append(f'<text x="160" y="-80" class="lbl acc">foot F1 (150, 58)</text><text x="160" y="92" class="lbl acc">foot F2 (150, -58)</text>'
               f'<text x="-300" y="26" class="lbl acc">foot F3 (-176, 0)</text>')
    out.append('<text x="215" y="-175" class="lbl">V-block (groove to the centroid)</text>')
    # down camera + flow lenses (v5 pods, ring frame)
    for name, (x, y) in (("down camera", (0, 49)), ("flow", (0, -42))):
        out.append(f'<rect x="{x - 6}" y="{-y - 6}" width="12" height="12" class="cam"/>')
    out.append('<text x="10" y="-52" class="lbl mute">down cam</text><text x="10" y="58" class="lbl mute">flow</text>')
    # CG marks
    out.append('<g class="cg"><circle cx="91" cy="0" r="5"/><line x1="84" y1="0" x2="98" y2="0"/><line x1="91" y1="-7" x2="91" y2="7"/></g>'
               '<text x="66" y="-12" class="lbl">Overseer CG +91</text>')
    # scale bar
    out.append(f'<line x1="250" y1="330" x2="350" y2="330" class="scale"/><text x="270" y="350" class="lbl">100 mm</text>')
    out.append("</svg>")
    return "\n".join(out)


def section_svg(th, H):
    """Section across a groove at the seated foot, drawn at 2:1 (groove bottom = foot bottom = z 0)."""
    tn = math.tan(math.radians(th))
    Wd, k = L.KIN_W, 2.0
    top = Wd * tn
    zn = H + L.NODE_R                     # ring node centre above the foot bottom
    zt = zn + L.NODE_R + 6
    P = lambda x, z: f"{x * k:.1f},{-z * k:.1f}"
    x0, x1, z0 = -(Wd + 20), Wd + 70, -16
    out = [f'<svg viewBox="{x0 * k} {-zt * k} {(x1 - x0) * k} {(zt - z0) * k}" class="sect" role="img" aria-label="Section across a V-groove, to scale">']
    for gx in range(int(x0) // 10 * 10, int(x1) + 1, 10):
        out.append(f'<line x1="{gx * k}" y1="{-zt * k}" x2="{gx * k}" y2="{-z0 * k}" class="grid"/>')
    blk = [(-Wd - 6, top), (-Wd, top), (0, 0), (Wd, top), (Wd + 6, top), (Wd + 6, -10), (-Wd - 6, -10)]
    out.append(f'<polygon points="{" ".join(P(*q) for q in blk)}" class="vblock"/>')
    fr = L.FOOT_R
    out.append(f'<rect x="{-5 * k}" y="{-zn * k:.1f}" width="{10 * k}" height="{(zn - fr) * k:.1f}" class="leg"/>')
    out.append(f'<circle cx="0" cy="{-fr * k:.1f}" r="{fr * k:.1f}" class="foot"/>')
    out.append(f'<circle cx="0" cy="{-zn * k:.1f}" r="{L.NODE_R * k:.1f}" class="node"/>')
    out.append(f'<line x1="{x0 * k}" y1="{-H * k:.1f}" x2="{x1 * k}" y2="{-H * k:.1f}" class="dim"/>')
    out.append(f'<text x="{(Wd + 10) * k}" y="{-(H + 2.5) * k:.1f}" class="lbl">node bottoms = keep-out</text>')
    out.append(f'<text x="{(Wd + 10) * k}" y="{-(top - 4) * k:.1f}" class="lbl">block top {top:.0f}</text>')
    out.append(f'<text x="{(L.NODE_R + 4) * k}" y="{-(zn + 2) * k:.1f}" class="lbl">ring node</text>')
    out.append(f'<text x="{12 * k}" y="{-(H / 2) * k:.1f}" class="lbl acc">foot {H:.0f}, ball tip</text>')
    out.append(f'<text x="{(-Wd - 4) * k}" y="{-(top - 8) * k:.1f}" class="lbl">flanks {th:.0f}°, half-width {Wd:.0f}</text>')
    out.append(f'<text x="{(Wd + 10) * k}" y="{-z0 * k - 6:.1f}" class="lbl mute">2:1, grid 10 mm</text>')
    out.append("</svg>")
    return "\n".join(out)


def chart_data(res):
    series = [("kin", 30.0, 0.03, "3 feet + V-grooves, ball-transfer tips (DEFAULT)"),
              ("kin", 30.0, 0.12, "3 feet + V-grooves, PTFE glide tips"),
              ("tray", 10.0, 0.03, "tray, rolling nodes"),
              ("dish", 10.0, 0.03, "one cone, rolling nodes")]
    out = {}
    for dr in ("overseer", "scout"):
        out[dr] = []
        for c, th, mu, name in series:
            r = pick(res, dr, c, th, mu)
            if r:
                out[dr].append(dict(name=name, pts=[[int(k), v] for k, v in r["rate"].items()]))
    return out


def table_rows(res):
    rows = []
    for r in res:
        dr, c, th, mu, H, W = r["key"]
        rows.append(dict(drone=dr, concept=c, theta=th, mu=mu, H=H, cap=r["capture_mm"], worst=r["worst_err"],
                         rate40=r["rate"].get("40"), clear=r["clear_min"]))
    return rows


def main():
    prov = not os.path.exists(os.path.join(OUT, "sweep.json"))
    if prov:      # sweep still running: drawings and recorded drops only
        tp = os.path.join(OUT, "traj_prov.json")
        if not os.path.exists(tp):
            json.dump(L.record_examples(), open(tp, "w"))
        traj = json.load(open(tp))
        sw = dict(ring_ellipse=L.fit_ellipse(L.RING))
        blank = dict(capture_mm=25, worst_err="…", clear_min="…", rate={})   # 32-drop check, both drones
        res = []
    else:
        sw = json.load(open(os.path.join(OUT, "sweep.json")))
        traj = json.load(open(os.path.join(OUT, "traj.json")))
        res = sw["results"]
    c, th, mu = DEFAULT
    H = L.kin_H(th)
    ov, sc = (blank, blank) if prov else (pick(res, "overseer", c, th, mu), pick(res, "scout", c, th, mu))
    a, b = sw["ring_ellipse"]
    feat = dict(ring=L.RING, feet=L.FEET, node_r=L.NODE_R, foot_r=L.FOOT_R, hole=L.HOLE, ellipse=[a, b],
                grooves=[dict(f=f.tolist(), a=a_.tolist(), c=c_.tolist()) for f, a_, c_ in L.kin_grooves()],
                W=L.KIN_W, LEN=L.KIN_LEN)
    data = dict(traj=traj, chart=chart_data(res), table=table_rows(res), feat=feat,
                summary=dict(ov=dict(cap=ov["capture_mm"], worst=ov["worst_err"], clear=ov["clear_min"], rate=ov["rate"]),
                             sc=dict(cap=sc["capture_mm"], worst=sc["worst_err"], rate=sc["rate"]), H=H, th=th, mu=mu))
    html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "landing_page.html")).read()
    html = html.replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
    html = html.replace("<!--TOP-->", top_svg(sw, H)).replace("<!--SECT-->", section_svg(th, H))
    for k, v in dict(OV_CAP=ov["capture_mm"], SC_CAP=sc["capture_mm"], OV_WORST=ov["worst_err"], SC_WORST=sc["worst_err"],
                     H=f"{H:.0f}", TH=f"{th:.0f}", MU=f"{mu:.2f}", OV_CLEAR=(f'{ov["clear_min"]} mm' if ov["clear_min"] != "…" else "sweep running")).items():
        html = html.replace("{{" + k + "}}", str(v))
    open(os.path.join(OUT, "landing.html"), "w").write(html)
    print("wrote", os.path.join(OUT, "landing.html"), len(html) // 1024, "KB")


if __name__ == "__main__":
    main()
