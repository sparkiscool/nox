"""Feeler-blade return for the S2 latch sliders: sizing sweep (17 Oct, Spark 11:07: same feeler as Scout v1).
Blade = Precision Brand 19425, 0.012 in x 1/2 in 1095 (0.305 x 12.7, P). Cantilever along z under the tray floor,
root epoxied in a post at the lead end, tip edge bearing in a groove on its own tab's inner face.
Both blades from the lead end (the closed end leaves only ~23 free: stiffer). Tip lands `a` into the tab from its
lead-end face (block z 43.6); the deeper the tip, the deeper the tapered groove has to be."""
import numpy as np, s2_cartridge as C
E, t, b = 205e3, 0.305, 12.7            # MPa E (1095 hardened), P, P
I = b * t**3 / 12
EPS_MAX = 0.0035                        # E: ~half of hardened 1095 yield strain (~0.7 %); V6 ran 0.22 %
F_REST_MIN = 0.3                        # N, E: top of the pen-spring band it replaces (k 0.1-0.3 x ~1 preload)
F_FULL_MAX = 4.0                        # N, E: half of the 8 N gripper finger spring (dimensions J)
TRAVEL = C.TRAVEL
Z_ROOT_MAX = 71.0                       # C: lead-end floor cross starts at z 70; root post z 71..77 (clamp 6) to the end wall
rows = []
for zr in np.arange(60, Z_ROOT_MAX + 1e-6, 0.5):
    for a in np.arange(1.0, 4.01, 0.5):
        zt = C.Z_BLK1 - a; L = zr - zt
        k = 3 * E * I / L**3
        for d0 in np.arange(0.2, 1.51, 0.05):
            Fr, Ff = k * d0, k * (d0 + TRAVEL)
            eps = 1.5 * t * (d0 + TRAVEL) / L**2
            # groove depth at the tab's lead-end face (s = L - a), squeezed state, relative to the tab
            s = L - a; g = s**2 * (3 * L - s) / (2 * L**3)
            depth = 0.5 + (d0 + TRAVEL) * (1 - g)       # 0.5 groove at the tip + what the blade sits proud of it
            ok = Fr >= F_REST_MIN and Ff <= F_FULL_MAX and eps <= EPS_MAX and depth <= 1.5 and a >= 1.5
            rows.append((ok, Ff, zr, a, L, d0, Fr, eps, depth, k))
okr = sorted([r for r in rows if r[0]], key=lambda r: (r[1], -r[3]))
print("feasible %d of %d" % (len(okr), len(rows)))
print(" Ffull  zroot  a    L     d0   Frest  strain  groove  k")
for r in okr[:8]:
    print("%5.2f  %5.1f  %3.1f  %4.1f  %4.2f  %4.2f  %.3f%%  %4.2f  %.3f" % (r[1], r[2], r[3], r[4], r[5], r[6], r[7]*100, r[8], r[9]))
