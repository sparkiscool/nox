"""Box stand: mass model, load configs, static tip margins.

Coordinates W (nox/box-arm.md): X tangential (drone nose), Y along the axle
(+ away from the stub wall), Z radial, up at the drone seat. mm, g.
Tags: P measured, M mesh, C CAD, E estimate.
"""
import itertools
import numpy as np

ZF = -160.0          # floor plane, C: ring outer r 154 + 6 pad (DEFAULT)
G = 9.81

# ---- fixed components: name, g, (X, Y, Z), half-size for inertia, tag
FIXED = [
    ("bay ring",            598, None,               None,           "M"),
    ("stub spider",         120, (0, -3, 0),         (146, 5, 146),  "E"),
    ("stub axle+flange",     90, (0, 15, 0),         (45, 18, 45),   "E"),
    ("2x 6010-2RS",         380, (0, 24, 0),         (40, 16, 40),   "E"),
    ("slip ring",            40, (0, 25, 0),         (11, 10, 11),   "E"),
    ("hub v3",              230, (0, 41, 0),         (80, 33, 80),   "M"),
    ("NEMA17 A",            280, (34, 26, -59),      (21, 24, 21),   "E"),
    ("NEMA17 B",            280, (61, 26, 55),       (21, 24, 21),   "E"),
    ("NEMA17 axle",         280, (-34.7, 26, -60.1), (21, 24, 21),   "E"),
    ("80T, belts, idlers",   80, (0, 60, 0),         (60, 10, 60),   "E"),
    ("PSU LRS-350-24",      750, (0, -23, -55),      (107, 15, 57),  "E"),
    ("SKR, chargers",       400, (0, -30, 40),       (90, 15, 40),   "E"),
    ("wiring",              150, (0, 80, 0),         (100, 60, 100), "E"),
    ("funnel collar v4",    475, (0, 126, 202),      (160, 160, 38), "E"),
    ("seat post, magnet",   150, (0, 126, 140),      (30, 30, 20),   "E"),
]
BAR, HEAD_A, HEAD_B = 60, 56 + 60, 67 + 55          # E bars; M heads + E servos
R_HEAD_PARK, R_HEAD_EXT = 48.0, 107.0               # C (glide 26..70, +58.6)
R_PACK_PARK, R_PACK_EXT = 65.4, 124.0               # C
BAY_ANG = [0, 45, 135, 180, 225, 270, 315]          # C (v3 ring; reorient = E)
R_BAY, Y_BAY = 124.0, 131.0                         # C
PACK = {0: 0.0, 1: 150.0, 2: 650.0}                 # E: Scout, Overseer pack + cartridge
DRONE = {"none": (0.0, None), "scout": (520.0, (0, 126, 200)),
         "overseer": (2000.0, (0, 126, 220))}       # P mass, E CG height


def ring_cg():
    r1, r2 = 99.5, 154.0
    a = np.radians(157.5)                            # half span about -90 deg
    rc = (2 / 3) * (r2**3 - r1**3) / (r2**2 - r1**2) * np.sin(a) / a
    return (0.0, 131.0, -rc)


def fixed_sum():
    m = 0.0; s = np.zeros(3)
    for name, g, p, _, _ in FIXED:
        p = ring_cg() if p is None else p
        m += g; s += g * np.array(p)
    return m, s


def configs(drones=("none", "scout", "overseer"), phi_step=15):
    """All load states: bay occupancy x arm angle x carried pack x drone.
    Returns mass (N,), first moment (N,3) in g*mm, and labels."""
    m0, s0 = fixed_sum()
    occ = [o for o in itertools.product((0, 1, 2), repeat=7)
           if o.count(2) <= 3 and o.count(1) <= 4 and o.count(0) >= 1]
    occ = np.array(occ)
    bay = np.array([[R_BAY * np.cos(np.radians(a)), Y_BAY, R_BAY * np.sin(np.radians(a))]
                    for a in BAY_ANG])
    pm = np.vectorize(PACK.get)(occ).astype(float)           # (O,7)
    bay_m = pm.sum(1); bay_s = pm @ bay                      # (O,), (O,3)
    arm = []
    for phi in np.arange(0, 360, phi_step):
        uA = np.array([np.cos(np.radians(phi)), 0, np.sin(np.radians(phi))])
        uB = np.array([np.cos(np.radians(phi + 90)), 0, np.sin(np.radians(phi + 90))])
        base_m = 2 * BAR + HEAD_A + HEAD_B
        for carry in ("none", "A_park", "A_ext", "B_park", "B_ext"):
            for pk in ((0,) if carry == "none" else (1, 2)):
                rA = R_HEAD_EXT if carry == "A_ext" else R_HEAD_PARK
                rB = R_HEAD_EXT if carry == "B_ext" else R_HEAD_PARK
                s = (BAR * 78 * uA + BAR * 78 * uB + HEAD_A * rA * uA + HEAD_B * rB * uB
                     + np.array([0, 84 * 2 * BAR + 125 * (HEAD_A + HEAD_B), 0]))
                mm = base_m
                if pk:
                    head = uA if carry[0] == "A" else uB
                    rp = R_PACK_EXT if carry.endswith("ext") else R_PACK_PARK
                    s = s + PACK[pk] * (rp * head + np.array([0, 126, 0]))
                    mm += PACK[pk]
                arm.append((phi, carry, pk, mm, s))
    out_m, out_s, lab = [], [], []
    for d in drones:
        dm, dp = DRONE[d]
        ds = np.zeros(3) if dp is None else dm * np.array(dp)
        for phi, carry, pk, am, as_ in arm:
            big_on_arm = 1 if pk == 2 else 0
            ok = (occ == 2).sum(1) + big_on_arm <= 3
            ok &= (occ == 1).sum(1) + (1 if pk == 1 else 0) <= 4
            out_m.append(m0 + am + dm + bay_m[ok])
            out_s.append(s0 + as_ + ds + bay_s[ok])
            lab += [(d, phi, carry, pk, i) for i in np.nonzero(ok)[0]]
    return np.concatenate(out_m), np.concatenate(out_s), lab, occ


def _cr(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def hull(pts):
    pts = sorted(map(tuple, pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and _cr(h[-2], h[-1], p) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + up[:-1])


def edge_margins(feet_xy, cg):
    """Signed distance (mm) of each CG floor projection to each support edge.
    feet_xy: (k,2) in (X,Y); cg: (N,3). Returns d (N,E), outward normals (E,2)."""
    h = hull(feet_xy)
    if len(h) < 3:
        return None, None
    a = h; b = np.roll(h, -1, 0)
    t = b - a; t = t / np.linalg.norm(t, axis=1, keepdims=True)
    n = np.stack([t[:, 1], -t[:, 0]], 1)                      # outward for CCW hull
    p = cg[:, [0, 1]]
    d = -((p[:, None, :] - a[None]) * n[None]).sum(-1)        # + inside
    return d, n


def static_report(feet_xy, m, s, push_N=0.0, arm_T=2.4):
    cg = s / m[:, None]
    d, n = edge_margins(feet_xy, cg)
    h = cg[:, 2] - ZF
    tilt = np.degrees(np.arctan2(d.min(1), h))
    W = m / 1000 * G
    # rim push: horizontal force at the funnel rim (Z 234) to tip about each edge
    h_rim = (234 - ZF) / 1000
    push = (W[:, None] * d / 1000 / h_rim).min(1)
    # arm drive torque about the axle (Y): overturns edges with X-facing normals
    armm = (W[:, None] * d / 1000 - arm_T * np.abs(n[None, :, 0])).min(1)
    return dict(tilt=tilt, push=push, arm_margin=armm, cg=cg, W=W)
