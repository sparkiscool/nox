"""Steeper faces at the same collar height (53.1) and the same 168 opening:
how much capture is left, and how much friction does it tolerate?"""
import math, itertools, json, sys
import s2_collar_settle as s

H = 53.1 * s.mm
EDGE = 84 * s.mm

def set_slope(deg):
    s.A = math.radians(deg); s.TA, s.SA, s.CA = math.tan(s.A), math.sin(s.A), math.cos(s.A)
    s.H_AXIS = s.F + s.R_ROD / s.SA
    s.Z_LO = s.ZS - (s.H_AXIS - EDGE) * s.TA
    s.Z_HI = s.Z_LO + H

def centred_at(m, ref, d):
    ok = n = 0
    for a8, yaw, tilt, dr in itertools.product(range(8), (-5, 0, 5), (0, 2), (1, -1)):
        a = a8 * math.pi / 4
        v = [dr * 0.15 * math.cos(a), dr * 0.15 * math.sin(a), -0.10]
        p, yf, tf, _ = s.drop(m, d * math.cos(a), d * math.sin(a), math.radians(yaw), a + 0.7, math.radians(tilt), v)
        e = (p - ref) * 1e3
        ok += math.hypot(e[0], e[1]) < 0.5 and abs(yf) < 0.3 and tf < 0.3
        n += 1
    return 100 * ok / n

deg, mu = float(sys.argv[1]), float(sys.argv[2])
set_slope(deg)
m = s.build("B", 0.30, mu)
ref = s.drop(m, 0, 0, 0, 0, 0, [0, 0, -0.10])[0]
mouth = 2 * (s.H_AXIS + (s.Z_HI - s.ZS) / s.TA) * 1e3
out = {"slope": deg, "mu": mu, "mouth_mm": round(mouth, 1)}
for dmm in (10, 20, 30, 40, 50):
    out[f"{dmm}"] = round(centred_at(m, ref, dmm * s.mm), 1)
print(json.dumps(out), flush=True)
