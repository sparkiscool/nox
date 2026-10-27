"""Checks for xswap v4 (latch under the pack). python3 xswap4_check.py -> runs/xswap/v4/result.json, check.txt"""
import json, math, os
import numpy as np
import xswap4 as X

R = {}
def say(k, v):
    R[k] = v; print(f"{k}: {v}", flush=True)
ov = X.overlap
dn = lambda m, t: m.translate([0, 0, -t])
sx_ = lambda m, s: m.translate([s, 0, 0])

body = X.shell_body()
female, keeper, male, cap = X.xt60_female(), X.xt60_keeper(), X.xt60_male(), X.xt60_cap()
holder = X.holder()
H = X.U(holder, cap)
Hm = X.U(H, male)
th_max = X.theta_from_u(X.STROKE, X.DELTA)
th_nom = X.theta_from_u(X.STROKE, 0.0)
th_min = X.theta_from_u(X.STROKE, -X.DELTA)

# ---- geometry summary
say("lever ratio (tip / finger) / release swing at the tip mm / release angle deg", [round(X.RATIO, 2), round(X.REL_SWING, 2), round(X.TH_REL, 2)])
say("finger stroke mm (clearance 0.8 + pincer error 0.5 + drive + 0.25)", round(X.STROKE, 2))
say("bar swing at full stroke deg: pincer off -0.5 / centred / +0.5 (must all be >= release)", [round(th_min, 2), round(th_nom, 2), round(th_max, 2)])
say("tip into the leg / ledge under the tip / tip bearing area mm2", [round(X.TIP_OUT - X.G, 2), 12.0, round(X.TIP_W * (X.TIP_OUT - X.G), 1)])

# ---- seat and bar motion
Sl = X.U(X.shell(0.0), keeper)
say("seat: latched shell vs holder + cap (mm3)", round(ov(Sl, H), 3))
say("seat: XT60 female vs male / male vs shell (mm3)", [round(ov(female, male), 3), round(ov(male, Sl), 3)])
mot = [round(ov(X.bar(th, "A"), body) + ov(X.bar(th, "B"), body), 2) for th in np.linspace(0, th_max, 9)]
say("bars vs shell body over 0..max swing (mm3, must be 0)", mot)
say("bar A vs bar B at max swing (mm3)", round(ov(X.bar(th_max, "A"), X.bar(th_max, "B")), 3))
lock = [round(ov(dn(Sl, t), H), 1) for t in (0.3, 0.6, 1.0, 3.0)]
say("latched shell pulled down 0.3 / 0.6 / 1 / 3 (mm3, must collide: ledges)", lock)

# ---- insert / pull sweeps with the bars held released
Sr = X.U(X.shell(th_min), keeper, female)
worst, stuck = 0.0, []
for t in np.arange(0.0, 95.01, 0.5):
    v = ov(dn(Sr, t), Hm); worst = max(worst, v)
    if v > 0.01: stuck.append(round(float(t), 1))
say("insert / pull sweep 0..95, bars released at the least swing (mm3)", round(worst, 3))
say("insert / pull stuck steps", stuck[:8])
Sr2 = X.U(X.shell(th_max), keeper, female)
say("same at the largest swing (mm3)", round(max(ov(dn(Sr2, t), Hm) for t in np.arange(0.0, 95.01, 1.0)), 3))

# ---- pincer: approach, tracking, error, capture
P = lambda p: X.U(*p.values())
po = X.pincer(0.0)
worst = 0.0
for t in np.arange(0.0, 100.01, 1.0):
    worst = max(worst, ov(dn(P(po), t), X.U(Sl, H, female)))
say("pincer open approach 0..100 below grip (mm3)", round(worst, 3))
trk = {}
for s in (-X.DELTA, 0.0, X.DELTA):
    rows = []
    for u in np.linspace(0.0, X.STROKE, 9):
        fa, fb = sx_(X.finger_A(u), s), sx_(X.finger_B(u), s)
        ta, tb = X.theta_from_u(u, -s), X.theta_from_u(u, s)
        rows.append(round(ov(fa, X.bar(ta, "A")) + ov(fb, X.bar(tb, "B")) + ov(X.U(fa, fb), body), 2))
    trk[s] = rows
say("fingers in the bar slots over the stroke, pincer off -0.5 / 0 / +0.5 in x (mm3, ~0)", [trk[-X.DELTA], trk[0.0], trk[X.DELTA]])
say("both bars past release at full stroke for pincer x error -0.5..+0.5", bool(min(X.theta_from_u(X.STROKE, d) for d in (-X.DELTA, X.DELTA)) >= X.TH_REL))
ps = X.pincer(X.STROKE)
fl = X.U(ps["finger_a"], ps["finger_b"])
cap_lip = [round(ov(dn(sx_(fl, s), 1.0).translate([0, 0, 0]), body), 1) for s in (-X.DELTA, 0.0, X.DELTA)]
say("shut fingers pulled 1.0 down into the lips (mm3, > 0 = the shell comes with the pincer), x error -0.5/0/+0.5", cap_lip)
say("open fingers pulled 1.0 down (mm3, 0 = free to leave)", round(ov(dn(X.U(po["finger_a"], po["finger_b"]), 1.0), body), 3))
say("pincer shut vs holder / vs shell body (mm3)", [round(ov(P(ps), H), 3), round(ov(P(ps), body), 3)])
mesh = []
for u in np.linspace(0.0, X.STROKE, 6):
    p = X.pincer(u)
    mesh.append(round(ov(p["pinion"], X.U(p["finger_a"], p["finger_b"])) + ov(X.U(p["finger_a"], p["finger_b"]), p["jaw"]), 2))
say("pinion vs racks + fingers vs jaw over the stroke (mm3)", mesh)
say("pincer width x / battery width (mm)", [round(2 * X.JW, 1), 47.0])

# ---- flip and bay (bars at the least swing, pack in)
carry = X.U(X.shell(th_min), keeper, female, P(ps))
def rot(m, a, D):
    return dn(m, D).translate([0, -X.Y_P, -(X.Z_AX - D)]).rotate([a, 0, 0]).translate([0, X.Y_P, X.Z_AX - D])
def clear(D, fixed):
    return max(ov(rot(carry, a, D), fixed) for a in range(0, 181, 10))
D1 = next(d for d in range(80, 400, 5) if clear(d, Hm) < 0.01)
say("rail travel before the flip (min, step 5, C)", D1)
bay = lambda m, D2: m.translate([0, -X.Y_P, -X.Z_AX]).rotate([180, 0, 0]).translate([0, X.Y_P, X.Z_AX - D1 - D2])
D2 = next(d for d in range(40, 400, 5) if clear(D1, bay(Hm, d)) < 0.01)
say("rail travel after the flip into the bay (min, step 5, C)", D2)
bH = bay(Hm, D2)
fl_ = lambda m: m.translate([0, -X.Y_P, -X.Z_AX]).rotate([180, 0, 0]).translate([0, X.Y_P, X.Z_AX - D1])
say("bay insert sweep, released (mm3)", round(max(ov(dn(fl_(X.U(X.shell(th_min), female)), t), bH) for t in np.arange(0.0, D2 + 0.01, 0.5)), 3))
say("bay seat latched (mm3)", round(ov(dn(fl_(X.shell(0.0)), D2), bH), 3))
pts = X.tm(carry).vertices
say("flip sweep radius about the axis (C)", round(float(np.max(np.hypot(pts[:, 1] - X.Y_P, pts[:, 2] - X.Z_AX))), 1))

# ---- capture under rail error: what touches first, a slope or a flat face?
flats = []
for which in ("A", "B"):
    led = X.box(X.POCKET_X0, X.POCKET_X1, X.SY1 + X.HC, X.SY1 + X.HC + X.GROOVE_D, X.POCKET_Z0 - 0.3, X.POCKET_Z0)
    flats.append(led if which == "A" else X.sym(led))
flat = X.U(*flats, cap, male)
rows = []
S0 = X.U(X.shell(th_nom), female)
for ex, ey, rx, ry in [(3, 0, 0, 0), (-3, 0, 0, 0), (0, 3, 0, 0), (0, -3, 0, 0), (5, 0, 0, 0), (-5, 0, 0, 0), (0, 5, 0, 0), (0, -5, 0, 0),
                       (4, 4, 0, 0), (-4, -4, 0, 0), (0, 0, 2, 0), (0, 0, 0, 2), (2, 2, 2, 2)]:
    S = S0.translate([0, -X.Y_P, -40]).rotate([rx, ry, 0]).translate([ex, X.Y_P + ey, 40])
    fa = ff = None
    for t in np.arange(110.0, -0.01, -0.5):
        m = dn(S, t)
        if fa is None and ov(m, holder) > 0.01: fa = round(float(t), 1)
        if ff is None and ov(m, flat) > 0.01: ff = round(float(t), 1)
        if fa is not None and ff is not None: break
    ok = fa is not None and (ff is None or fa > ff)
    rows.append(dict(offset_xy_tilt=[ex, ey, rx, ry], first_touch_mm=fa, first_flat_touch_mm=ff, slope_first=ok))
say("capture (x, y offset mm, tilt deg) -> first touch below seat / first flat-face touch; slope must lead",
    [(r["offset_xy_tilt"], r["first_touch_mm"], r["first_flat_touch_mm"], "slope first" if r["slope_first"] else "FAIL") for r in rows])

# ---- mass / CG
m, cg, ms, mb = X.swap_cg()
say("swap mass g (pack 480 + shell + 2 bars + keeper + XT60 + bolts + dowels, E)", round(m, 1))
say("swap CG (C/E)", [round(float(v), 2) for v in cg])
R["mass_g_solid_petg"] = {k: round(v.volume() * X.PETG, 1) for k, v in dict(
    shell=body, bar=X.bar_local(), holder=holder, jaw=po["jaw"], finger=po["finger_a"], pinion=po["pinion"]).items()}
print("mass", R["mass_g_solid_petg"])

# ---- loads (E). 150 g pull-out (10 mph stopped in ~7 mm), shared by the two tips
G_ = 150.0
F = m / 1000 * 9.81 * G_; Ft = F / 2
say("crash: 150 g on the swap mass, total / per tip N (E)", [round(F), round(Ft)])
arm = X.G + 0.5 * (X.TIP_OUT - X.G)
say("tip: shear at the end face / bending MPa (bar printed flat: in-layer, PETG ~45)",
    [round(Ft / (X.TIP_W * X.TIP_H), 1), round(6 * Ft * arm / (X.TIP_W * X.TIP_H ** 2), 1)])
say("tip on the ledge: bearing MPa", round(Ft / (X.TIP_W * (X.TIP_OUT - X.G)), 1))
Lb = 2 * X.TRAY_X
Zb = X.BAND_D * (X.BAND_H + X.FLOOR_T) ** 2 / 6
say("shell end band over the tip: fixed-fixed bending MPa (in-layer, the shell prints tail down)", round(Ft * Lb / 8 / Zb, 1))
arm_l = X.POCKET_X1 - X.X_LAT
# ledge section in (y, z) at its root: the leg below the pocket minus the 30 deg lead-in triangle
import shapely.geometry as sg
yi, yo = X.SY1 + X.HC, X.SY1 + X.HC + X.LEG_T
zt = X.LEG_Z0 + X.LEG_CH / math.tan(math.radians(30))
sec = sg.box(yi, X.LEG_Z0, yo, X.POCKET_Z0).difference(sg.Polygon([(yi, X.LEG_Z0), (yi + X.LEG_CH, X.LEG_Z0), (yi, zt)]))
cz = sec.centroid.y
xs_, zs_ = np.meshgrid(np.linspace(yi, yo, 200), np.linspace(X.LEG_Z0, X.POCKET_Z0, 400))
inside = np.array([sec.contains(sg.Point(a_, b_)) for a_, b_ in zip(xs_.ravel(), zs_.ravel())])
dA = (yo - yi) / 200 * (X.POCKET_Z0 - X.LEG_Z0) / 400
I_ = float(np.sum(((zs_.ravel() - cz) ** 2)[inside]) * dA)
c_ = max(X.POCKET_Z0 - cz, cz - X.LEG_Z0)
say("holder ledge (real section with the lead-in): area mm2 / I mm4 / bending MPa (in-layer)", [round(sec.area), round(I_), round(Ft * arm_l * c_ / I_, 1)])
leg_a = (2 * (X.SX + X.HC + X.SK_T) - (X.POCKET_X1 - X.CHAN_X0)) * X.LEG_T + (X.POCKET_X1 - X.CHAN_X0) * (X.LEG_T - X.GROOVE_D)
say("holder leg tension across layers MPa (PETG across layers ~15-20)", round(Ft / leg_a, 1))
say("per M3 insert over each leg N (2 per leg)", round(Ft / 2))
say("friction holding a loaded tip shut N (mu 0.3) vs spring preload N", [round(0.3 * Ft), round((X.SPR_FREE - 17.0) * X.SPR_RATE, 1)])
# spring, finger force, servo
x_lat = 17.0
d_rel = X.SPR_D * math.sin(math.radians(X.TH_REL)); d_max = X.SPR_D * math.sin(math.radians(th_max))
f_lat, f_rel, f_max = [round((X.SPR_FREE - (x_lat - d)) * X.SPR_RATE, 1) for d in (0.0, d_rel, d_max)]
say("spring force latched / at release / at max swing N (6 OD x 21 free, 0.8 N/mm, E)", [f_lat, f_rel, f_max])
say("spring length at max swing mm (solid ~6.6, E)", round(x_lat - d_max, 1))
F_f = f_max * X.SPR_D / X.FIN_A
say("finger push at max swing N / servo torque N m (2 fingers, R 13.5)", [round(F_f, 1), round(2 * F_f * X.R_P / 1000, 2)])
inert = 6.5e-3 * 9.81 * G_ * 30.0
say("side hit 150 g: bar inertia torque N mm vs spring preload torque (opposite-handed bars: one opens, one closes)",
    [round(inert), round(f_lat * X.SPR_D)])
say("pull to unplug XT60 + 1 g, per finger: flange bearing on the lips MPa", round((50 + m / 1000 * 9.81) / 2 / (2 * 3.0 * 2.3), 1))
say("finger post bending at full push MPa (post printed upright: across layers)", round(F_f * 8.0 / (math.pi * X.SLOT_R_POST ** 3 / 4), 1))

os.makedirs(X.OUT, exist_ok=True)
R["capture"] = rows
json.dump(R, open(os.path.join(X.OUT, "result.json"), "w"), indent=1, default=float)
