"""Two forward cameras round the Unitree L2 (Spark 24 Oct: "two cameras, both pointing forward, angled so they capture as
much of the lidar's view as possible"). Coverage = share of the L2 field (the hemisphere ahead of its base plane, axis
15 deg below +x; uniform in solid angle) inside the union of the two camera frusta (pinhole, DECXIN 2633V2 89 x 73 deg,
4:3 from 102 diagonal, E). Searched: mirror pair (yaw +-psi, pitch theta, roll +-rho) and stacked pair (pitch theta1 /
theta2, yaw 0), each landscape or portrait; Nelder-Mead from a grid.
python3 nox/overseer/cam_cover.py [out.json]"""
import json, math, sys, itertools
import numpy as np
from scipy.optimize import minimize

H, V = 89.0, 73.0                                     # landscape full angles (E)
A = np.array([math.sin(math.radians(105)), 0.0, math.cos(math.radians(105))])
rng = np.random.default_rng(0)
D = rng.normal(size=(60000, 3)); D /= np.linalg.norm(D, axis=1)[:, None]
D = D[D @ A > 0]                                      # the L2 field, uniform in solid angle


def R_cam(yaw, pitch, roll):
    """camera frame (x forward, y left, z up) in the drone frame: yaw about z, pitch up about -y, roll about forward"""
    y, p, r = (math.radians(a) for a in (yaw, pitch, roll))
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, -math.sin(p)], [0, 1, 0], [math.sin(p), 0, math.cos(p)]])   # +pitch = nose up
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


def seen(R, portrait=False):
    h, v = (V, H) if portrait else (H, V)
    c = D @ R                                         # directions in the camera frame
    return (c[:, 0] > 0) & (np.abs(c[:, 1]) <= c[:, 0] * math.tan(math.radians(h / 2))) & (np.abs(c[:, 2]) <= c[:, 0] * math.tan(math.radians(v / 2)))


def cover(cams):
    m = np.zeros(len(D), bool)
    for yaw, pitch, roll, portrait in cams: m |= seen(R_cam(yaw, pitch, roll), portrait)
    return float(m.mean())


def mirror(x, portrait):
    psi, th, rho = x; return [(psi, th, rho, portrait), (-psi, th, -rho, portrait)]


def stacked(x, portrait):
    t1, t2, rho = x; return [(0.0, t1, rho, portrait), (0.0, t2, -rho, portrait)]


def best(kind, portrait):
    f = mirror if kind == "mirror" else stacked
    grid = itertools.product(range(10, 80, 10), range(-50, 31, 10), (0, 20, 40)) if kind == "mirror" else \
        itertools.product(range(-10, 50, 10), range(-80, -10, 10), (0, 20))
    starts = sorted(grid, key=lambda x: -cover(f(x, portrait)))[:4]
    out = None
    for s in starts:
        r = minimize(lambda x: -cover(f(x, portrait)), s, method="Nelder-Mead", options=dict(xatol=0.5, fatol=1e-4, maxiter=300))
        if out is None or r.fun < out.fun: out = r
    return dict(kind=kind, portrait=portrait, x=[round(float(v), 1) for v in out.x], cover=round(-float(out.fun), 3), cams=f(out.x, portrait))


if __name__ == "__main__":
    one = max((dict(yaw=0.0, pitch=float(t), portrait=pt, cover=cover([(0.0, t, 0.0, pt)])) for t in range(-60, 31, 5) for pt in (False, True)), key=lambda d: d["cover"])
    res = [best(k, pt) for k in ("mirror", "stacked") for pt in (False, True)]
    res.sort(key=lambda r: -r["cover"])
    ideal = 2 * 4 * math.asin(math.sin(math.radians(H / 2)) * math.sin(math.radians(V / 2))) / (2 * math.pi)
    print(f"one camera, best pitch: {one['pitch']:.0f} deg ({'portrait' if one['portrait'] else 'landscape'}): {100 * one['cover']:.1f} % of the L2 field")
    for r in res:
        print(f"{r['kind']:8s} {'portrait ' if r['portrait'] else 'landscape'} x {r['x']}: {100 * r['cover']:.1f} % of the L2 field")
    print(f"ceiling (two frusta, no overlap, fully inside the field): {100 * ideal:.1f} %")
    if len(sys.argv) > 1: json.dump(dict(one=one, pairs=res, ceiling=ideal), open(sys.argv[1], "w"), indent=1, default=float)
