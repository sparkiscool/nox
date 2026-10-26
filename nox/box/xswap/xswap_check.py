"""Checks for xswap v2: seat, rocker motion, latch lock, pinched insert / pull sweep, capture under offsets, pincer
approach / grip / mesh, flip and bay, masses, crash hand calcs. Writes runs/xswap/v2/result.json. python3 xswap_check.py"""
import json, math, os
import numpy as np
import xswap as X

YC, ZA = X.Y_C, X.Z_AX
R = {}
def say(k, v):
    R[k] = v; print(f"{k}: {v}")
ov = X.overlap
dn = lambda m, t: m.translate([0, 0, -t])

body = X.shell_body(YC)
shell_l, shell_p = X.shell(YC, 0.0), X.shell(YC, X.TH_OPEN)
female, keeper, male = X.xt60_female(), X.xt60_keeper(), X.xt60_male()
holder, cap = X.holder(YC), X.xt60_cap()
posts = X.U(X.post(YC + X.G_OFF), X.post(YC + X.G_OFF).translate([0, -2 * X.G_OFF, 0]).mirror([1, 0, 0]))
H = X.U(holder, cap)
Sl, Sp = X.U(shell_l, keeper), X.U(shell_p, keeper)

# ---- seat and rocker motion
say("seat: shell (latched) vs holder + cap (mm3)", round(ov(Sl, H), 3))
say("seat: XT60 female vs male / male vs shell (mm3)", [round(ov(female, male), 3), round(ov(male, Sl), 3)])
say("XT60 engaged at seat (C)", X.ENGAGE)
mot = []
for th in np.arange(0, X.TH_OPEN + 0.01, 1.0):
    mot.append(round(ov(X.rocker(YC, th, 1), body), 2))
say("rocker vs shell body, 0..13 deg (mm3, must be 0)", mot)
say("rocker vs hard stop at 15.5 deg (mm3, > 0 = the stop)", round(ov(X.rocker(YC, 15.5, 1), body), 2))
say("rocker at rest vs holder (mm3)", round(ov(X.rocker(YC, 0, 1), H), 3))
say("hook: x engagement over the ledge / drop to the catch / opened hook tip clear of the ledge (mm, C)",
    [round(X.LUG_X1 - X.HOOK_X0, 2), round(X.HOOK_Z0 - X.LEDGE_Z1, 2),
     round(X._rk(X.TH_OPEN, X.HOOK_X0 - X.PX, X.HOOK_Z0 - X.PZ)[0] - X.LUG_X1, 2)])
say("self-locking: pivot x / hook contact x range (C; pivot inboard = a pull closes the hook)", [X.PX, [X.HOOK_X0, X.LUG_X1]])
lock = [round(ov(dn(Sl, t), H), 1) for t in (0.3, 0.6, 1.0, 3.0)]
say("latched shell pulled down 0.3 / 0.6 / 1 / 3 (mm3, must collide)", lock)

# ---- pinched insert / pull sweep
worst, stuck = 0.0, []
for t in np.arange(0.0, 55.01, 0.5):
    v = ov(dn(X.U(Sp, female), t), X.U(H, male)); worst = max(worst, v)
    if v > 0.01: stuck.append(round(float(t), 1))
say("insert / pull sweep, pinched, 0..55 every 0.5: worst overlap (mm3)", round(worst, 3))
say("insert / pull stuck steps", stuck[:8])

# ---- capture: shell offset (ex, ey), pinched, rising from 45 below; which part touches first
lugs = X.U(X.lug(YC), X.lug(YC).mirror([1, 0, 0]))
ledges = X.U(*[X.box(X.POCKET_X0, X.LUG_X1, YC - X.LUG_HY, YC + X.LUG_HY, X.LEDGE_Z0, X.LEDGE_Z1).mirror([1, 0, 0]) if sx < 0 else
               X.box(X.POCKET_X0, X.LUG_X1, YC - X.LUG_HY, YC + X.LUG_HY, X.LEDGE_Z0, X.LEDGE_Z1) for sx in (-1, 1)])
keyflat = X.U(*[X.hexwin("z", y0 + X.KEY_C + 0.8, y1 - X.KEY_C - 0.8, -16.0 + X.KEY_C + 0.8, 16.0 - X.KEY_C - 0.8, -X.KEY_D, -X.KEY_D + 0.3)
                for y0, y1 in X.WIN_FD])
nonguide = X.U(ledges, keyflat, cap, male)          # flat faces that must never be the first touch (everything else is a slope)
cap_rows = []
for ex, ey in [(3, 0), (-3, 0), (0, 3), (0, -3), (5, 0), (-5, 0), (0, 5), (0, -5), (4, 4), (-4, -4), (4, -4), (-4, 4)]:
    S = X.U(Sp, female).translate([ex, ey, 0])
    first_post = first_other = None
    for t in np.arange(45.0, -0.01, -0.25):
        m = dn(S, t)
        if first_post is None and ov(m, posts) > 0.01: first_post = round(float(t), 2)
        if first_other is None and ov(m, nonguide) > 0.01: first_other = round(float(t), 2)
        if first_post is not None and first_other is not None: break
    ok = first_post is not None and (first_other is None or first_post > first_other)
    cap_rows.append(dict(offset=[ex, ey], post_touch_mm_below_seat=first_post, other_touch=first_other, post_first=ok))
say("capture (offset mm -> first touch below seat: post / a non-guide part (lug, key, XT60); guides must lead)",
    [(r["offset"], r["post_touch_mm_below_seat"], r["other_touch"], "post first" if r["post_first"] else "FAIL") for r in cap_rows])
say("capture range (C): post tip r / socket entry r / funnel per side, sides / ends",
    [X.POST_TIP_R, X.SOCK_CH, X.FLARE_H, X.FLARE_HY])

# ---- pincer
P = lambda p: X.U(p["jaw"], p["finger_p"], p["finger_m"], p["pinion"], p["servo"])
pin_o, pin_s = X.pincer(YC, X.FACE_OPEN), X.pincer(YC, X.FACE_SHUT)
worst = 0.0
for t in np.arange(0.0, 90.01, 1.0):
    worst = max(worst, ov(dn(P(pin_o), t), X.U(Sl, H, female)))
say("pincer open approach 0..90 below grip (mm3)", round(worst, 3))
w = X.Z_N - X.PZ
trk = []
for th in np.arange(0.0, X.TH_OPEN + 0.01, 1.0):
    t = math.radians(th); u = (X.SLOT_LX + w * math.sin(t)) / math.cos(t)
    face = X.PX + u + X.NECK
    f = X.finger(YC, face)
    trk.append(round(ov(f, X.rocker(YC, th, 1)) + ov(f, body), 2))
say("finger nose tracking the rocker 0..13 deg (mm3, ~0)", trk)
say("finger shut vs pinched shell / jaw+servo+pinion vs shell (mm3)",
    [round(ov(X.U(pin_s["finger_p"], pin_s["finger_m"]), shell_p), 3), round(ov(X.U(pin_s["jaw"], pin_s["servo"], pin_s["pinion"]), shell_p), 3)])
say("pincer shut vs holder (mm3)", round(ov(P(pin_s), H), 3))
mesh = []
for face in np.linspace(X.FACE_SHUT, X.FACE_OPEN, 6):
    p = X.pincer(YC, face)
    pin = p["pinion"].translate([0, -YC, 0]).rotate([0, 0, -math.degrees((face - X.FACE_SHUT) / X.R_P)]).translate([0, YC, 0])
    mesh.append(round(ov(pin, X.U(p["finger_p"], p["finger_m"])), 2) + round(ov(X.U(p["finger_p"], p["finger_m"]), p["jaw"]), 2))
say("pinion vs racks + fingers vs jaw over the stroke (mm3)", mesh)

# ---- flip and bay (shell nominal, and hanging 2.6 in the slots)
def carry(hang):
    return X.U(Sp.translate([0, 0, -hang]), female.translate([0, 0, -hang]), P(pin_s))
def rot(m, a, D):
    return dn(m, D).translate([0, -YC, -(ZA - D)]).rotate([a, 0, 0]).translate([0, YC, ZA - D])
def flip_clear(D1, fixed):
    w = 0.0
    for hang in (0.0, 2.6):
        c = carry(hang)
        for a in range(0, 181, 10):
            w = max(w, ov(rot(c, a, D1), fixed))
    return w
D1 = next(d for d in range(60, 220, 5) if flip_clear(d, X.U(H, male)) < 0.01)
say("rail travel before the flip (min, step 5, C)", D1)
bayf = lambda m, D2: m.translate([0, -YC, -ZA]).rotate([180, 0, 0]).translate([0, YC, ZA - D1 - D2])
D2 = next(d for d in range(int(D1) - 30, 240, 5) if flip_clear(D1, bayf(X.U(H, male), d)) < 0.01)
say("rail travel after the flip into the bay (min, step 5, C)", D2)
bH = bayf(X.U(H, male), D2)
fl = lambda m: m.translate([0, -YC, -ZA]).rotate([180, 0, 0]).translate([0, YC, ZA - D1])
worst = max(ov(dn(fl(X.U(Sp, female)), t), bH) for t in np.arange(0.0, D2 + 0.01, 0.5))
say("bay insert sweep, pinched (mm3)", round(worst, 3))
say("bay seat latched (mm3)", round(ov(dn(fl(Sl), D2), bH), 3))
pts = X.tm(carry(0.0)).vertices
say("flip sweep radius about the axis (C)", round(float(np.max(np.hypot(pts[:, 1] - YC, pts[:, 2] - ZA))), 1))

# ---- mass / CG
m, cg, ms, mk = X.swap_cg(YC)
say("swap mass g (pack 470 + shell + rockers + keeper + XT60 + bolts, E)", round(m, 1))
say("swap CG (C/E)", [round(float(v), 2) for v in cg])
R["mass_g_solid_petg"] = {k: round(v.volume() * X.PETG, 1) for k, v in dict(
    shell=body, rocker=X.rocker_local(YC), xt60_keeper=keeper, holder=holder, xt60_cap=cap,
    jaw=pin_o["jaw"], finger=pin_o["finger_p"], pinion=pin_o["pinion"]).items()}
print("mass", R["mass_g_solid_petg"])

# ---- crash hand calcs (E). 10 mph = 4.47 m/s; 150 g needs ~6.8 mm of stopping, 100 g ~10 mm.
G = 150.0
F = m / 1000 * 9.81 * G
Fr = F / 2
say("crash: design case 150 g on the swap mass, pull-out direction, total / per rocker N (E)", [round(F), round(Fr)])
hook_l = X.RB_X0 - 0.5 * (X.HOOK_X0 + X.LUG_X1)
sig_hook = 6 * Fr * hook_l / (2 * X.ROCK_HY * (X.HOOK_Z1 - X.HOOK_Z0) ** 2)
say("crash: hook root bending MPa (rocker printed on its side: in-layer, PETG ~45)", round(sig_hook, 1))
say("crash: hook bearing on the lug MPa", round(Fr / (2 * X.ROCK_HY * (X.LUG_X1 - X.HOOK_X0)), 1))
say("crash: rocker boss on the M4 pin MPa", round(Fr / (4.0 * 2 * X.ROCK_HY), 1))
say("crash: M4 pin double shear MPa (steel 8.8 ~ 400)", round(Fr / (2 * math.pi * 2.0 ** 2), 1))
ear_h = 14.0; ear_t = X.EAR_HY1 - X.EAR_HY0
say("crash: ear bearing / ear root bending MPa (in-layer)", [round(Fr / 2 / (4.0 * ear_t), 1),
                                                           round(6 * (Fr / 2) * (X.PX - X.SX) / (ear_t * ear_h ** 2), 1)])
lug_a = (X.POCKET_X0 - (X.SX + X.HC + X.SK_T - 0.5)) * 2 * X.LUG_HY
say("crash: lug spine tension across layers MPa (PETG across layers ~15-20)", round(Fr / lug_a, 1))
led_l = 0.5 * (X.LUG_X1 - X.POCKET_X0)
say("crash: ledge bending MPa (in-layer)", round(6 * Fr * led_l / (2 * X.LUG_HY * (X.LEDGE_Z1 - X.LEDGE_Z0) ** 2), 1))
say("crash: per M3 insert over the lugs N (2 per lug; M3 heat-set pull-out in PETG ~500+ N, E)", round(Fr / 2))
q = (X.PACK_G / 1000 * 9.81 * G) / ((X.CAV_Y1 - X.CAV_Y0) * 2 * X.CAV_X)
Mf = q * (2 * X.CAV_X) ** 2 / 8
say("crash: floor bending under the pack MPa (in-layer, 3.0 solid)", round(6 * Mf / X.FLOOR ** 2, 1))
self_lock = -(0.5 * (X.HOOK_X0 + X.LUG_X1) - X.PX) * Fr
rock_inertia = (9.0 / 1000 * 9.81 * G) * (38.0 - X.PX)
say("crash: closing torque from the hook load / opening torque from the rocker's own inertia N mm (E)", [round(-self_lock), round(rock_inertia)])
key_w = 32.0 - 2 * X.KEY_C
say("crash: side / end hit, deck window bearing on the keys MPa (2 keys, 2.4 deep)", round(F / (2 * key_w * X.KEY_D * 0.5), 1))
# drone side: pinch servo
Fz = 60.0 / 2
T = 2 * (6.0 + Fz * math.tan(math.radians(22))) * X.R_P / 1000
say("pinch servo torque to hold the 60 N push N m (E)", round(T, 2))
os.makedirs(X.OUT, exist_ok=True)
R["capture"] = cap_rows
json.dump(R, open(os.path.join(X.OUT, "result.json"), "w"), indent=1)
