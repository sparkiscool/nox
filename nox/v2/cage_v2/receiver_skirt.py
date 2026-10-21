"""Cage v3 (layout C3) skirt interface for the S2 receiver, 17 Oct 09:45 (cage chat). The receiver script imports this and
adds three things to its own run: the 4 drawn skirt sockets, their bores (cuts), and the skirt load cases.

Spark 17 Oct 03:52: the cage's bottom rods mount to the drone-side receiver and connect to the prop guard; 09:20 (box-safe):
the sockets sit at deck height so nothing leaves the wheel's top slot, no feet (the drone lands on the cartridge's flat
bottom). Roots (rod ends) at drone (+-30.5, +-26.5, 5) (C, layout_v3.py), rods (cage v4: 3.0 carbon, Spark 17 Oct 21:33) up
to the side node skirt sockets on the lower ring at (+-3.2, +-112, 33.2) (axis aimed there since v4, Problem 48); socket 10
deep, sleeve wall 2.0 (PETG), bore rod + 0.3 (3.3), 0.6 lead-in at the mouth, closed end 1.0.
Loads (F): frame model C3 (layout_v3.py) at the cage's design speed (rods at index 1, 0.71 m/s), the force each skirt rod
puts on the receiver, all four roots together per case; the 6 worst hits ('crash') and the swap push / flip ('misuse').
Forces only (the epoxied rod's end moment is left out, E).
Use in the receiver script (cartridge coords: x_c = drone y, y_c = drone z + 44.5, z_c = 37.5 - drone x):
    import receiver_skirt as RS
    sk_drawn, sk_cut = RS.bodies(mf)              # manifolds, cartridge coords
    drawn = drawn + sk_drawn;  cuts = cuts + sk_cut
    ... after solid / cells() exist:  cases += RS.cases(PO, load, lambda i: cells(RS.socket(mf, i)))
"""
import json, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = json.load(open(os.path.join(HERE, "receiver_skirt.json")))
ZC, DZ = 37.5, -44.5
DEPTH, WALL, GAP, LEAD, CAP = 10.0, 2.0, 0.3, 0.6, 1.0

def to_cart(p):
    p = np.asarray(p, float); return np.array([p[1], p[2] - DZ, ZC - p[0]])
def vec_to_cart(v):
    v = np.asarray(v, float); return np.array([v[1], v[2], -v[0]])

def _cyl(mf, a, b, r, r2=None, seg=40):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; Ln = float(np.linalg.norm(d)); z = d / Ln
    x = np.cross([0, 0, 1], z) if abs(z[2]) < 0.99 else np.cross([1, 0, 0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    M = np.zeros((3, 4)); M[:, 0], M[:, 1], M[:, 2], M[:, 3] = x, y, z, a
    return mf.Manifold.cylinder(Ln, r, r if r2 is None else r2, seg).transform(M)

def socket(mf, i):
    """drawn sleeve of socket i (0..3), cartridge coords"""
    s = DATA["sockets"][i]; a = to_cart(s["root"]); u = vec_to_cart(s["axis"]); d = s["rod_d"]
    return _cyl(mf, a - u * CAP, a + u * DEPTH, d / 2 + GAP / 2 + WALL)

def bodies(mf):
    drawn = cut = None
    for i, s in enumerate(DATA["sockets"]):
        a = to_cart(s["root"]); u = vec_to_cart(s["axis"]); r = s["rod_d"] / 2 + GAP / 2; m = a + u * DEPTH
        c = _cyl(mf, a, m + u * 8.0, r) + _cyl(mf, m - u * LEAD, m + u * 0.01, r, r + LEAD + 0.01) + _cyl(mf, m, m + u * 8.0, r + LEAD + 0.01)
        sl = socket(mf, i)
        drawn = sl if drawn is None else drawn + sl; cut = c if cut is None else cut + c
    return drawn, cut

def cases(PO, load, cells_of):
    """LoadCase list for the receiver: load(mask, vec) is the receiver's own uniform load helper, support 'bolts'"""
    out = []
    for c in DATA["cases"]:
        F = None
        for i, f in enumerate(c["forces"]):
            v = vec_to_cart(f)
            if np.linalg.norm(v) < 1e-6: continue
            Fi = load(cells_of(i), tuple(v)); F = Fi if F is None else F + Fi
        if F is not None: out.append(PO.LoadCase(("skirt " + c["name"])[:60], F, cls=c["cls"], support="bolts"))
    return out

if __name__ == "__main__":                       # writes receiver_skirt.json from the frame model (cage chat)
    import sys; sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
    import layout_v3 as L, s2_guard_frame as sg
    o = json.load(open(os.path.join(HERE, "layout_C3.json")))
    P = sg.params_v2(PZ_=L.PZ, lower=("solid", o.get("lower", 2.5)), upper=("solid", o["upper"])); P.update(opt=o, spoke=("solid", o.get("spoke", 2.5)), skirt=("solid", o["skirt_d"]))
    g = L.Guard3(P); g.carbon_only = True; r, _ = L.evaluate(o); v = r["v_intact"]
    q = o["bottom"]["quad"]; roots = [np.array([sx * q["x"], sy * L.RCV_Y, q["z"]]) for sx in (1, -1) for sy in (1, -1)]
    socks = []
    for R0 in roots:
        T = L.skirt_target(tuple(q["targets"][0]), np.sign(R0[0]), np.sign(R0[1]), L.PZ - 8.0)
        T = T + np.array([np.sign(R0[0]) * 3.2, 0, 0])     # the side node skirt socket sits 3.2 off the flat centre (Problem 48, v4)
        socks.append(dict(root=R0.round(3).tolist(), axis=((T - R0) / np.linalg.norm(T - R0)).round(5).tolist(), rod_d=o["skirt_d"]))
    rid = [g.node(R0) for R0 in roots]
    def root_forces(u):
        out = []
        for i in rid:
            f = np.zeros(3)
            for e in g.elems:
                if e["tag"] == "skirt" and i in e["n"]:
                    k = e["n"].index(i); a, b = e["n"]; dof = np.r_[6 * a:6 * a + 6, 6 * b:6 * b + 6]
                    fg = e["T"].T @ (e["kl"] @ (e["T"] @ u[dof])); f += -fg[6 * k:6 * k + 3]
            out.append(f)
        return out
    hits = sg.hit_cases_v2(g); U1, _ = g.indent_batch(hits, sg.energy(v))
    rows = []
    for j, h in enumerate(hits):
        fs = root_forces(U1[:, j]); rows.append((max(np.linalg.norm(f) for f in fs), f"hit {h[0]} {h[1]}", "crash", fs))
    W = sg.MIN_KG * sg.G0
    def apply(loads):
        Fv = np.zeros(6 * g.N)
        for i, f in loads: Fv[6 * i:6 * i + 3] += f
        return g.solve(Fv)
    stat = []
    for sgn, ax in ((1, 0), (-1, 0), (1, 1), (-1, 1)):
        wp = [(p, n) for p, n in g.wheel_pts if n[ax] * sgn > 0.5]
        fs = root_forces(apply([(g.node_id(p), -n * 7.5) for p, n in wp])); stat.append((max(np.linalg.norm(f) for f in fs), f"swap push 15N face {'+' if sgn > 0 else '-'}{'xy'[ax]}", "misuse", fs))
    fs = root_forces(apply([(i, np.array([0, 0, -6 * W / 4])) for i in g.crown])); stat.append((max(np.linalg.norm(f) for f in fs), "flip 6g on the crown", "misuse", fs))
    keep = sorted(rows, key=lambda t: -t[0])[:6] + sorted(stat, key=lambda t: -t[0])[:3]
    out = dict(layout="layout_C3.json", v_intact=round(v, 3), sockets=socks,
               cases=[dict(name=n, cls=c, peak_N=round(m, 1), forces=[f.round(2).tolist() for f in fs]) for m, n, c, fs in keep])
    json.dump(out, open(os.path.join(HERE, "receiver_skirt.json"), "w"), indent=1)
    print(json.dumps(dict(v=out["v_intact"], sockets=socks, peaks=[(c["name"], c["peak_N"]) for c in out["cases"]]), indent=1))
