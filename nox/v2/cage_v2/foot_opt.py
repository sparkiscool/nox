"""Floor feet + skids for the S2 cage v2 (17 Oct 03:27, receiver v3 bottom layout).

Spark 17 Oct 02:23 / 03:27: the pack runs nose-tail, drone x -45.8..41.8, |y| <= 24.3 with the receiver lead-in feet,
bottom at drone z -44.5 (E). Flow sensor on the receiver at the nose: board x 47..69, |y| <= 8, z -5..3; its view cone
(23 deg half angle from the lens at (63, -2, -5), straight down) stays clear. Foot bottoms 3 under the pack (z -47.5).
Skids join each side's front and back foot along x at y +-30.75 (never under the pack or the arm head's path);
legs still go to the side hub roots (strut_route.json "side" bracket_socket_bottom).

Drone frame (x nose, y left, z up, mm). Each foot = TPU pad sphere r 3.2 cut flat at its 45 deg line (centre 2.26 above
the flat bottom) + a 2.5 carbon skid socket along x + the 3.0 leg socket.
Free variable: the foot station x (y is fixed by Spark). Objective: the largest tip-over angle on a flat floor
(levers: x minus the CG x offset, y), CG from the 402 g build-up with the pack lowered (s2_feet_sim.mass_patch, E).
Checks, every point at yaw +-5 deg and tilt +-2 deg about the CG:
  collar : perpendicular gap to the 4 collar face planes extended below the opening, need 3.0 (feet, skids, legs).
  pack   : horizontal gap to the pack + lead-in prism |y| <= 24.3 widening 30 deg below the pack bottom, need 1.0.
  cone   : gap to the flow sensor view cone, need 1.0.
"""
import math, itertools, json, numpy as np
ZL_RING, F_RING, RIM_OFF = 33.2, 112.0, 1.25 + 2.5
S45 = math.sin(math.radians(45))
D0 = F_RING * S45 - ZL_RING * S45 + RIM_OFF * 1.0          # face plane n.p (cage_cad.face_plane): 59.47
MARGIN, PAD_R, SKID_R, LEG_R, SOCK_R = 3.0, 3.2, 1.25, 1.5, 2.9
PACK_BOT, PACK_Y, PACK_X = -44.5, 24.3, (-45.8, 41.8)
FOOT_BOT = PACK_BOT - 3.0                                  # -47.5
ZC = FOOT_BOT + PAD_R * math.cos(math.radians(45))         # pad centre -45.24
Z_SKID = FOOT_BOT + SOCK_R                                 # skid axis: its socket body sits on the floor plane, -44.6
Y_SKID = 30.75                                             # Spark 03:27
LEG_END = np.array([0.0, 50.0, 20.75])                     # side hub root (strut_route.json)
CG = np.array([0.57, 0.0, 2.79])                           # E, s2_feet_sim.mass_patch
LENS, CONE = np.array([63.0, -2.0, -5.0]), math.radians(23.0)
TAN30 = math.tan(math.radians(30))

POSES = []
for yaw, tx, ty in itertools.product((-5, 0, 5), (-2, 0, 2), (-2, 0, 2)):
    a, bx, by = map(math.radians, (yaw, tx, ty))
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, math.cos(bx), -math.sin(bx)], [0, math.sin(bx), math.cos(bx)]])
    Ry = np.array([[math.cos(by), 0, math.sin(by)], [0, 1, 0], [-math.sin(by), 0, math.cos(by)]])
    POSES.append(Rz @ Rx @ Ry)
NORMALS = [np.array([math.cos(k * math.pi / 2) * S45, math.sin(k * math.pi / 2) * S45, -S45]) for k in range(4)]

def collar_gap(P, r):
    """per face: smallest perpendicular gap of points P (n,3) of radius r over the poses"""
    P = np.atleast_2d(P) - CG
    out = []
    for n in NORMALS:
        out.append(min(float((D0 - (P @ R.T + CG) @ n).min()) for R in POSES) - r)
    return out

def seg(a, b, n=40):
    t = np.linspace(0, 1, n)[:, None]; return a + t * (b - a)

def cone_gap(P, r):
    """distance from points to the view cone (apex LENS, axis -z), minus r"""
    d = np.atleast_2d(P) - LENS
    h = -d[:, 2]; rho = np.hypot(d[:, 0], d[:, 1])
    # signed distance to a cone surface: rotate into (rho, h)
    s, c = math.sin(CONE), math.cos(CONE)
    dist = np.where(h * s + rho * c < 0, np.hypot(rho, h), rho * c - h * s)
    return float(dist.min()) - r

def pack_gap(y, z, r):
    """horizontal gap to the pack + lead-in prism, widening 30 deg below the pack bottom"""
    return abs(y) - r - (PACK_Y + max(0.0, PACK_BOT - z) * TAN30)

def evaluate(xf):
    Fc = np.array([xf, Y_SKID, ZC])
    skid = seg(np.array([xf, Y_SKID, Z_SKID]), np.array([-xf, Y_SKID, Z_SKID]))
    legs = np.vstack([seg(np.array([sx * xf, Y_SKID, ZC]), LEG_END) for sx in (1, -1)])
    feet = np.array([[sx * xf, Y_SKID, ZC] for sx in (1, -1)])
    cg_feet = [min(collar_gap(feet * [1, sy, 1], PAD_R)) for sy in (1, -1)]
    g = dict(
        collar_feet=collar_gap(feet, PAD_R), collar_skid=collar_gap(skid, SKID_R),
        collar_legs=collar_gap(legs, LEG_R),
        pack_feet=pack_gap(Y_SKID, ZC, PAD_R), pack_skid=pack_gap(Y_SKID, Z_SKID - SKID_R, SKID_R),
        cone=min(cone_gap(np.vstack([feet * [1, sy, 1], skid * [1, sy, 1], legs * [1, sy, 1]]), PAD_R) for sy in (1, -1)))
    h = CG[2] - FOOT_BOT
    tip = math.degrees(math.atan(min(xf - abs(CG[0]), Y_SKID) / h))
    return tip, g

if __name__ == "__main__":
    rows = []
    for xf in np.arange(10.0, 45.01, 0.25):
        tip, g = evaluate(xf)
        x_ok = min(g["collar_feet"][0], g["collar_feet"][2], g["collar_skid"][0], g["collar_skid"][2],
                   g["collar_legs"][0], g["collar_legs"][2]) >= MARGIN
        ok = x_ok and g["pack_feet"] >= 1.0 and g["pack_skid"] >= 1.0 and g["cone"] >= 1.0
        rows.append((ok, tip, xf, g))
    feas = [r for r in rows if r[0]]
    ok, tip, xf, g = max(feas, key=lambda r: (r[1], -r[2]))
    yface = min(g["collar_feet"][1], g["collar_feet"][3])
    y_ok_up_to = max((r[2] for r in rows if min(r[3]["collar_feet"]) >= MARGIN), default=None)
    out = dict(foot_centre=[round(xf, 2), Y_SKID, round(ZC, 2)], pad_r=PAD_R, foot_bottom=round(FOOT_BOT, 2),
               skid=dict(y=Y_SKID, z=round(Z_SKID, 2), rod=2.5), tip_deg=round(tip, 1),
               collar_gap_x_faces=round(min(g["collar_feet"][0], g["collar_feet"][2]), 2),
               collar_gap_y_faces=round(yface, 2), collar_gap_skid=round(min(g["collar_skid"]), 2),
               collar_gap_legs=round(min(g["collar_legs"]), 2),
               pack_gap=round(min(g["pack_feet"], g["pack_skid"]), 2), cone_gap=round(g["cone"], 2),
               cg=CG.tolist(), feasible=len(feas))
    print(json.dumps(out, indent=1)); json.dump(out, open("foot_opt.json", "w"))
