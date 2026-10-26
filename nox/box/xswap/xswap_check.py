"""Checks for xswap v1: seat fit, latch, insertion and pull-out sweeps, pincer approach / grip, flip clearance, bay seat,
rack mesh, hand calcs. Writes runs/xswap/v1/result.json and the STL files. python3 xswap_check.py"""
import json, math, os, sys
import numpy as np
import xswap as X
from xswap import M, box

YC, ZP = X.Y_C, X.Z_P
R = {}
def say(k, v):
    R[k] = v; print(f"{k}: {v}")

shell_r = X.shell(YC)                    # relaxed
shell_p = X.shell(YC, X.PINCH)           # pads squeezed to the stops
female, keeper = X.xt60_female(), X.xt60_keeper()
holder, cap, male = X.holder(YC), X.xt60_cap(), X.xt60_male()
H = X.U(holder, cap)                     # fixed side (XT60 male checked on its own: it mates into the female)
Sp = X.U(shell_p, keeper)
Sr = X.U(shell_r, keeper)
pin_open = X.pincer(YC, X.FACE_OPEN)
pin_shut = X.pincer(YC, X.FACE_SHUT)
def P(p): return X.U(p["jaw"], p["finger_p"], p["finger_m"], p["pinion"], p["servo"])
Po, Ps = P(pin_open), P(pin_shut)
dn = lambda m, t: m.translate([0, 0, -t])
ov = X.overlap

# ---- seat
say("seat: shell vs holder+cap (mm3)", round(ov(Sr, H), 3))
say("seat: female vs male (mm3, insert in the socket)", round(ov(female, male), 3))
say("seat: male vs shell (mm3)", round(ov(male, Sr), 3))
say("XT60 engaged at seat (C)", X.ENGAGE)
say("latch: nose over tooth (C, x engagement / drop)", [round(X.NOSE_P - 0.3, 2), round(X.NOSE_Z0 - X.TOOTH_Z1, 2)])

# ---- pinched insertion / pull-out (shell + female + keeper vs holder + cap + male)
worst = 0.0; stuck = []
for t in np.arange(0.0, 45.01, 0.5):
    v = ov(dn(X.U(Sp, female), t), X.U(H, male)); worst = max(worst, v)
    if v > 0.01: stuck.append(round(float(t), 1))
say("insert/pull sweep, pinched, 0..45 every 0.5: worst overlap (mm3)", round(worst, 3))
say("insert/pull sweep stuck steps", stuck[:10])
hold = [round(ov(dn(Sr, t), H), 2) for t in (0.3, 0.5, 1.0, 2.0)]
say("relaxed shell pulled down 0.3/0.5/1/2 (must collide = latched, mm3)", hold)

# ---- pincer approach (open) from below, shell relaxed and seated
worst = 0.0
for t in np.arange(0.0, 90.01, 1.0):
    worst = max(worst, ov(dn(Po, t), X.U(Sr, H, female)))
say("pincer open approach 0..90 below grip: worst overlap (mm3)", round(worst, 3))
say("pincer shut at seat vs holder+cap (mm3)", round(ov(Ps, H), 3))
fing = X.U(pin_shut["finger_p"], pin_shut["finger_m"])
say("finger shut: blade face / nose tip x (C)", [round(X.FACE_SHUT, 2), round(X.FACE_SHUT - X.NOSE_H, 2)])
say("fingers shut vs squeezed shell (mm3, nose in pocket)", round(ov(fing, shell_p), 3))
say("jaw/servo/pinion vs squeezed shell (mm3)", round(ov(X.U(pin_shut["jaw"], pin_shut["servo"], pin_shut["pinion"]), shell_p), 3))
# rack mesh: pinion vs racks over the stroke (pinion turned with the racks)
mesh = []
for face in np.linspace(X.FACE_SHUT, X.FACE_OPEN, 8):
    p = X.pincer(YC, face)
    ang = -math.degrees((face - X.FACE_SHUT) / X.R_P)
    pin = p["pinion"].translate([0, -YC, 0]).rotate([0, 0, ang]).translate([0, YC, 0])
    mesh.append(round(ov(pin, X.U(p["finger_p"], p["finger_m"])), 2))
say("pinion vs racks over the stroke (mm3, ~0 = meshing)", mesh)
say("fingers vs jaw over the stroke (mm3)", [round(ov(X.U(X.pincer(YC, f)["finger_p"], X.pincer(YC, f)["finger_m"]), X.pincer(YC, f)["jaw"]), 2)
                                           for f in np.linspace(X.FACE_SHUT, X.FACE_OPEN, 5)])

# ---- flip: carry down D1, rotate about (YC, ZP - D1) 0..180, vs the drone holder
carry = X.U(Sp, female, Ps)
def flip_ok(D1, step=10):
    w = 0.0
    for a in range(0, 181, step):
        m = dn(carry, D1).translate([0, -YC, -(ZP - D1)]).rotate([a, 0, 0]).translate([0, YC, ZP - D1])
        w = max(w, ov(m, X.U(H, male)))
    return w
D1 = None
for d in range(60, 200, 5):
    if flip_ok(d) < 0.01: D1 = d; break
say("rail travel before the flip (min, step 5, C)", D1)
sw = tm_b = X.tm(carry).bounds
r_sweep = float(np.max(np.hypot(X.tm(carry).vertices[:, 1] - YC, X.tm(carry).vertices[:, 2] - ZP)))
say("flip sweep radius about the axis (C)", round(r_sweep, 1))
say("flip sweep x extent (C)", [round(float(v), 1) for v in (sw[0][0], sw[1][0])])

# ---- bay: the same holder turned 180 deg about the flip axis, D1 + D_INS lower
def flip_bay_ok(D1, D2, step=10):
    bH = X.U(H, male).translate([0, -YC, -ZP]).rotate([180, 0, 0]).translate([0, YC, ZP - D1 - D2])
    w = 0.0
    for a_ in range(0, 181, step):
        m_ = dn(carry, D1).translate([0, -YC, -(ZP - D1)]).rotate([a_, 0, 0]).translate([0, YC, ZP - D1])
        w = max(w, ov(m_, bH))
    return w
D_INS = None
for d in range(int(D1), 200, 5):
    if flip_bay_ok(D1, d) < 0.01: D_INS = d; break
say("rail travel after the flip into the bay (min, step 5, C)", D_INS)
bay = lambda m: m.translate([0, -YC, -ZP]).rotate([180, 0, 0]).translate([0, YC, ZP - D1 - D_INS])
bay_H, bay_male = bay(H), bay(male)
flipped = lambda m: m.translate([0, -YC, -ZP]).rotate([180, 0, 0]).translate([0, YC, ZP - D1])
worst = 0.0
for t in np.arange(0.0, D_INS + 0.01, 0.5):
    worst = max(worst, ov(dn(flipped(X.U(Sp, female)), t), X.U(bay_H, bay_male)))
say("bay insert sweep, pinched: worst overlap (mm3)", round(worst, 3))
say("bay seat relaxed vs bay holder (mm3)", round(ov(dn(flipped(Sr), D_INS), bay_H), 3))
say("bay holder deck underside z / mount face z, drone frame (C)", [round(2 * ZP - D1 - D_INS, 1), round(2 * ZP - D1 - D_INS - X.DK_T, 1)])
w = 0.0
for a_ in range(0, 181, 10):
    m_ = dn(carry, D1).translate([0, -YC, -(ZP - D1)]).rotate([a_, 0, 0]).translate([0, YC, ZP - D1])
    w = max(w, ov(m_, X.U(bay_H, bay_male)))
say("flip vs bay holder: worst overlap (mm3)", round(w, 3))

# ---- masses (solid PETG, E) and CG
m, cg, ms, mk = X.swap_cg(YC)
say("swap mass g (pack 470 + shell + keeper + XT60 + bolts, E)", round(m, 1))
say("swap CG (C/E)", [round(float(v), 2) for v in cg])
say("clip / pinch line y vs CG y (mm)", round(YC - float(cg[1]), 2))
for k, v in dict(shell=shell_r, xt60_keeper=keeper, holder=holder, xt60_cap=cap, jaw=pin_open["jaw"],
                 finger=pin_open["finger_p"], pinion=pin_open["pinion"]).items():
    R.setdefault("mass_g_solid_petg", {})[k] = round(v.volume() * X.PETG, 1)
print("mass", R["mass_g_solid_petg"])

# ---- hand calcs (E)
E_PETG = 2000.0                                          # MPa (E)
I_b = X.W_B * X.T_B ** 3 / 12
a = X.B_ROOT_Z - ZP
F_b = 3 * E_PETG * I_b * X.PINCH / a ** 3
eps = 3 * X.T_B * X.PINCH / (2 * a ** 2)
x_n = X.B_ROOT_Z - 0.5 * (X.NOSE_Z0 + X.NOSE_Z1)
d_n = X.PINCH * x_n ** 2 * (3 * a - x_n) / (2 * a ** 3)
say("beam: force per pad to the stop N (E)", round(F_b, 1))
say("beam: root strain % at full pinch (E, PETG fatigue ~1 %)", round(100 * eps, 2))
say("beam: nose retracts / engagement / clear of the tooth (mm, C)", [round(d_n, 2), round(X.NOSE_P - 0.3, 2), round(d_n - (X.NOSE_P - 0.3), 2)])
F_hold = 0.592 * 9.81 * 10 / 2
say("latch: 10 g drop per nose N / nose shear MPa (E)", [round(F_hold, 1), round(F_hold / (X.W_B * 3.0), 2)])
Fz = 60.0 / 2
F_out = Fz * math.tan(math.radians(X.POC_ZA))
T_pinch = 2 * (F_b + F_out) * X.R_P / 1000
say("pinch servo torque to hold the 60 N push N m (E)", round(T_pinch, 3))
T_flip = 0.592 * 9.81 * abs(float(cg[2]) - ZP) / 1000 + 0.40 * 9.81 * 0.050
say("flip torque, static N m (E: swap + ~400 g pincer 50 below the axis)", round(T_flip, 2))
say("hub shear per stub shaft at the 60 N push N (C/E, two hubs)", 30.0)

os.makedirs(X.OUT, exist_ok=True)
json.dump(R, open(os.path.join(X.OUT, "result.json"), "w"), indent=1)
