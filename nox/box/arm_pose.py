"""Motor pose search for the 90-degree V arm (16 Oct 2026). Three NEMA17s stand beside the hub in the band Y 2..50
and bolt up to the plate. Head A's motor sits on A's belt line (X 34, below the axle), head B's on B's belt line
(Z 55 in the arm frame, +X side), the axle motor on the C 69.4 circle. Every body: edge toward the axle so its
nearest point stays >= 46.5 from the axis (hub sleeve r 44 + 2.5), bodies >= 3 apart, seats inside the 180 plate
(|x|,|z| <= 88 for the 26.5 seat discs), belt bands and posts clear. Objective: residual unbalance of the arm.
python arm_pose.py -> prints the pose dict for arm_layout.MOTORS"""
import math, itertools

NEMA = 42.3; HALF = NEMA / 2
R_HUB_CLEAR = 46.5
SEAT = 26.5
# fixed geometry (arm frame = head A up)
A_X = 34.0                     # A's belt centre line (runs 27.65 / 40.35)
B_Z = 55.0                     # B's belt centre line in the arm frame (B's X_HBELT_C): runs 48.65 / 61.35
C_AX = 69.4                    # axle belt centre distance
POSTS_A = [(-16, 16, 26, 38), (-16, 16, 70, 82)]        # x0, x1, z0, z1 at Y 52..74 (v3: flat bars, posts X +-16)
POSTS_B = [(-38, -26, -16, 16), (-82, -70, -16, 16)]
# masses (g) and lever arms (mm) of what the motors must balance (E): heads parked, packs in
HEADS = [(300.0, 0.0, 52.0), (300.0, -66.0, 0.0), (58.0, 0.0, 46.0), (47.0, -55.0, 0.0)]
M_MOTOR = 300.0

def square_pts(xc, zc, rot, half, n=12):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    corners = [(sx * half, sz * half) for sx, sz in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    pts = []
    for i in range(4):
        (ax, az), (bx, bz) = corners[i], corners[(i + 1) % 4]
        for k in range(n):
            t = k / n; ex, ez = ax + t * (bx - ax), az + t * (bz - az)
            pts.append((xc + ex * c - ez * s, zc + ex * s + ez * c))
    return pts

def poly(xc, zc, rot, half):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    return [(xc + sx * half * c - sz * half * s, zc + sx * half * s + sz * half * c) for sx, sz in ((1, 1), (1, -1), (-1, -1), (-1, 1))]

def sep_axis(pa, pb):
    """convex polygons: separation distance >= 0 (0 if overlapping) via the SAT on edge normals"""
    best = -1e9
    for P, Q in ((pa, pb), (pb, pa)):
        for i in range(len(P)):
            x0, z0 = P[i]; x1, z1 = P[(i + 1) % len(P)]
            nx, nz = -(z1 - z0), (x1 - x0); ln = math.hypot(nx, nz); nx, nz = nx / ln, nz / ln
            pa_ = [nx * x + nz * z for x, z in P]; pb_ = [nx * x + nz * z for x, z in Q]
            gap = max(min(pb_) - max(pa_), min(pa_) - max(pb_))
            best = max(best, gap)
    return best

def rect_poly(x0, x1, z0, z1): return [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]

def band_poly(ax, az, bx, bz, hw):
    dx, dz = bx - ax, bz - az; ln = math.hypot(dx, dz); nx, nz = -dz / ln * hw, dx / ln * hw
    return [(ax + nx, az + nz), (bx + nx, bz + nz), (bx - nx, bz - nz), (ax - nx, az - nz)]

def evaluate(zA, rotA, xB, rotB, thAx, rotAx):
    xAx, zAx = C_AX * math.cos(math.radians(thAx)), C_AX * math.sin(math.radians(thAx))
    motors = {"A": (A_X, zA, rotA), "B": (xB, B_Z, rotB), "axle": (xAx, zAx, rotAx)}
    polys = {k: poly(*v, HALF) for k, v in motors.items()}
    # hub clearance, plate extent
    for k, (xc, zc, rot) in motors.items():
        if min(math.hypot(x, z) for x, z in square_pts(xc, zc, rot, HALF)) < R_HUB_CLEAR: return None
        if abs(xc) + SEAT > 88 or abs(zc) + SEAT > 88: return None
        if max(math.hypot(x, z) for x, z in square_pts(xc, zc, rot, HALF)) > 150: return None
    for a, b in itertools.combinations(polys, 2):
        if sep_axis(polys[a], polys[b]) < 3.0: return None
    # belts (as 2D bands): the axle belt from the 80T to its 20T, half width 8 (belt + pulley flanges); B's belt band
    axle_band = band_poly(0, 0, xAx, zAx, 8.0)
    b_band = rect_poly(-132, xB, B_Z - 8, B_Z + 8)
    a_band = rect_poly(A_X - 8, A_X + 8, zA, 128)
    for p in POSTS_A + POSTS_B:
        if sep_axis(axle_band, rect_poly(*p)) < 2.0: return None       # axle belt (Y 56..62) vs posts (Y 54..74)
    for p in POSTS_B + POSTS_A:
        if sep_axis(b_band, rect_poly(*p)) < 2.0 and not (p in POSTS_A and p[2] >= 63): return None
    if sep_axis(axle_band, b_band) < 2.0: return None               # same Y plane 56..62
    # the axle 20T (r 8 at Y 56..62) vs B's belt band and B's 20T vs the axle band
    if sep_axis(rect_poly(xAx - 8, xAx + 8, zAx - 8, zAx + 8), b_band) < 2.0: return None
    # pulleys of A (Y 64..74) vs posts of A (Y 54..74): A's 20T at (34, zA) r 8 vs posts X +-8 -> fine; B's 20T vs A's outer post
    if sep_axis(rect_poly(xB - 8, xB + 8, B_Z - 8, B_Z + 8), rect_poly(*POSTS_A[1])) < 2.0: return None
    # unbalance
    ux = sum(m * x for m, x, z in HEADS) + M_MOTOR * sum(v[0] for v in motors.values())
    uz = sum(m * z for m, x, z in HEADS) + M_MOTOR * sum(v[1] for v in motors.values())
    ext = max(max(abs(xc), abs(zc)) + SEAT for xc, zc, _ in motors.values())
    return dict(unbal_gmm=math.hypot(ux, uz), ext=ext, motors=motors)

def search():
    best = None
    for zA in range(-61, -54):
        for rotA in range(0, 90, 5):
            for xB in range(43, 62, 2):
                for rotB in range(0, 90, 5):
                    for th in range(-180, 180, 5):
                        for rotAx in range(0, 90, 10):
                            r = evaluate(zA, rotA, xB, rotB, th, rotAx)
                            if r and (best is None or r["unbal_gmm"] < best["unbal_gmm"]): best = r
    return best

if __name__ == "__main__":
    b = search()
    print(b)
