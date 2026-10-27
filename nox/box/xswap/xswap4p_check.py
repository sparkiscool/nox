"""Fit, motion, capture and strength checks for the four v4 pincer versions -> runs/xswap/v4p/result.json"""
import numpy as np, math, json, os
import xswap4 as X, xswap4p as P
ov = X.overlap
bars = lambda th: X.U(X.bar(th, "A"), X.bar(th, "B"))
res = {}
for k in P.VARIANTS:
    shell = P.shell_body(k); r = {}
    p0 = P.pincer(k, 0.0)
    trk, internal = [], []
    for s in (-X.DELTA, 0.0, X.DELTA):
        for u in np.linspace(0, X.STROKE, 7):
            p = P.pincer(k, u)
            fa, fb = p["finger_a"].translate([s, 0, 0]), p["finger_b"].translate([s, 0, 0])
            trk.append(ov(fa, X.bar(X.theta_from_u(u, -s), "A")) + ov(fb, X.bar(X.theta_from_u(u, s), "B")) + ov(X.U(fa, fb), shell))
            if s == 0.0:
                F = X.U(p["finger_a"], p["finger_b"])
                internal.append(ov(F, p["base"]) + ov(F, p["lid"]) + ov(F, p["pinion"]) + ov(p["pinion"], p["base"]) + ov(p["lid"], p["base"]))
    r["fingers vs bars + shell over the stroke, pincer x error -0.5/0/+0.5 (mm3)"] = round(max(trk), 3)
    r["fingers vs base / lid / pinion over the stroke (mm3)"] = round(max(internal), 3)
    pa = X.U(*p0.values())
    r["approach 0..100 below grip vs shell, bars, holder (mm3)"] = round(max(ov(pa.translate([0, 0, -t]), X.U(shell, bars(0.0), X.holder())) for t in np.arange(0, 100.1, 2.0)), 3)
    ps = P.pincer(k, X.STROKE)
    F = X.U(ps["finger_a"], ps["finger_b"]); F0 = X.U(p0["finger_a"], p0["finger_b"])
    r["shut fingers pulled down 1 mm (mm3, >0 = shell captured)"] = round(ov(F.translate([0, 0, -1.0]), shell), 1)
    r["open fingers pulled down 1 mm (mm3, 0 = free to leave)"] = round(ov(F0.translate([0, 0, -1.0]), shell), 3)
    r["shut fingers lifted 0.5 into the lid (mm3, >0 = lid takes the pull)"] = round(ov(F.translate([0, 0, 0.5]), ps["lid"]), 1)
    r["both bars past release at full stroke"] = bool(min(X.theta_from_u(X.STROKE, d) for d in (-X.DELTA, X.DELTA)) >= X.TH_REL)
    mass = {n: round(m.volume() * X.PETG, 1) for n, m in p0.items() if n not in ("servo", "steel")}
    r["mass_g"] = mass; r["pincer_printed_g"] = round(sum(mass.values()), 1)
    bb = pa.bounding_box(); r["size_mm"] = [round(bb[3] - bb[0], 1), round(bb[4] - bb[1], 1), round(bb[5] - bb[2], 1)]
    r["one body each"] = all(len([b for b in m.decompose() if b.volume() > 0.01]) == 1 for n, m in p0.items() if n not in ("steel",))
    r["shell still one body"] = len([b for b in shell.decompose() if b.volume() > 0.01]) == 1
    r["safety"] = P.safety(k)
    res[k] = r
    print(k, {a: b for a, b in r.items() if a not in ("safety", "mass_g")}, flush=True)
res["v4"] = {"safety": P.safety("v4"), "pincer_printed_g": round((X.jaw_body().volume() + 2 * X.finger_A(0).volume() + X.pinion().volume()) * X.PETG, 1)}
os.makedirs(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p"), exist_ok=True)
json.dump(res, open(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "result.json"), "w"), indent=1)
for k in ["v4"] + list(P.VARIANTS):
    s = res[k]["safety"]
    print(k, {c: min((v["sf"], m) for m, v in s[c].items()) for c in s})
