"""Box swap arm v5 layout (23 Oct 2026, arm-v5 chat): the 2040 bars, belts, idlers, idler ends, T-nuts and head 20Ts are gone
(Spark). Each head rides a PRINTED V-RAIL on a 3-wheel bearing carriage (printed V-wheels on 625ZZ, one eccentric) and is
driven by a RACK AND PINION: an m1 16T printed pinion on the head motor's D-shaft, a 71 rack printed into the head's carriage.
Pure python (math only): the pose search for the motors (balance), every 2D clearance of the new parts over the stroke, the
detent re-placement candidates, the compaction table (Y_C / modules / funnel / enclosure) and the parts delta.
Frames: arm frame (W at phi 90: X tangential, Z radial along head A, Y axial); head frame (X, u along its rail, s = Y - Y_C);
head B's frame maps to the arm frame by arm X = -u_B, arm Z = X_B. Numbers C unless tagged E (estimate) or P.
python3 arm_v5_layout.py  -> report + runs/arm/v5-layout/result.json (the artifact and the docs read it)"""
import math, json, os, itertools
import arm_layout as L

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "runs", "arm", "v5-layout-c"); os.makedirs(OUT, exist_ok=True)
CLR = 1.0                                   # rule: every designed gap >= 1.0 (DEFAULT, = CLR_POST)

# ================================================================== v5 parts (C)
# --- rail (PETG, one per head; A and B differ only in the inner insert row) ---
RAIL_Y = (74.0, 93.0)                       # posts Y 52..74; top 93 (24 Oct, arm-mech: was 94 = 0.0 under the carriage plate, now 1.0)
RAIL_R = (27.0, 134.0)                      # along the head axis: start 27 (24 Oct: was 29; the rail is Y 74..93, the 80T / ring / collar end at
                                            # Y 72, so only the park stop needs it); end 134 (the outer wheel contact at the drone insert is r 127.6:
                                            # 6.4 of ridge past it)
# park hard stop (24 Oct, box electronics v1: homing without endstops): a block on the rail top at the inner end; the carriage plate's inner
# edge (u 0) meets it at z0 31, 1.0 inside park (32). Homing: drive inward at low current to the stall, z0 := 31, go to park. Heads v6 keep
# nothing at u < 0 inside X +-10, Y 93..98 (the plate's own band).
RAIL_STOP = dict(x=(-9.0, 9.0), y=(93.0, 97.0), r=(27.0, 31.0), z0_home=31.0)
RAIL_FOOT_X, RAIL_FOOT_H = 16.0, 6.0        # foot X +-16 on the 32-wide posts, Y 74..80, only at the posts (20 long each)
RAIL_BODY_X = 9.0                           # body X +-9 (18 wide), Y 74..93 (24 Oct: full depth; the hub stays <= Y 73 under it)
RIDGE_X, RIDGE_FLAT_X, RIDGE_Y = 12.0, 11.5, 88.0    # 90 deg V ridge on both faces: flanks 9 -> 12 at Y 85..91, tip truncated at 11.5 (0.5 flat)
RIDGE_HALF_H = 3.0
RAIL_CHAN = dict(x=(-2.0, 2.0), y=(89.0, 93.0), mouth=3.2)   # servo ribbon channel in the rail top, centred (24 Oct: was X -8..-4, a 1.0 outer wall):
                                            # 4 wide at the floor, flanks taper to the 3.2 mouth (ribbon snapped in); the head's entry is at X 0
# rail to post (24 Oct, arm-mech; was 2 x M5 per post at X +-10 from below into rail inserts: the inner posts sit over bearing 1's shoulder,
# no bolt head reaches them from below, and from above at X +-10 the bolt crosses the ridge and the wheel path): ONE M5 x 16 socket head per
# post, down from the rail top at X 0 (counterbore Ø9 to Y 84, through Ø5.5), into an M5 heat-set insert in the post top (hole Ø6.4 x 8, E)
RAIL_BOLTS = dict(x=0.0, r_A=(35.0, 76.0), r_B=(38.75, 76.0), cbore_r=4.5, cbore_y=84.0, hole_r=2.75, bolt="M5 x 16 ISO 4762",
                  post_insert=dict(hole_d=6.4, depth=8.0, insert="M5 x 5.8 heat-set (OD 7.0, E)"))
RAIL_INSERTS = RAIL_BOLTS                   # old name kept for importers; the rail itself has no insert now
# --- carriage (part of the head, printed with it) ---
WHEEL_OD, WHEEL_W, WHEEL_GROOVE_D = 24.0, 10.0, 2.0  # printed V-wheel on 2 x 625ZZ (5 x 16 x 5, P spec): rim r 12, 90 deg groove 2 deep (bottom r 10, 2.0 over the bearing)
WHEEL_X = 22.0                               # axle X: groove bottom at X 12 = the ridge's virtual apex; rim at X 10, 1.0 off the rail body (9)
WHEEL_Y = (83.0, 93.0)                       # wheel over the ridge plane Y 88 +-5
WHEELS = ((22.0, 4.0), (22.0, 32.0), (-22.0, 18.0))    # (X, u): fixed pair on the rack side, the eccentric opposite (printed bush, 0.8 throw)
                                            # 24 Oct: u 10 / 38 / 24 -> 4 / 32 / 18 (wheelbase 28 kept): with the +6 drone push the old set reached r 147.3
                                            # (wheel rim) / 144.3 (plate corner), past the r 143 module keep and the stand's r 145.5 arm lane
NUT_Y, NUT_R = (79.0, 83.0), 4.6             # M5 nyloc under the wheel (8 AF)
TAIL_Y, TAIL_R = (73.0, 79.0), 2.5            # 24 Oct: the M5 x 25 tails below the nylocs (fixed wheels to Y 73, the eccentric's to 75.5): hub v6 stays <= Y 72 under the carriage path, X +-24.5
SPACER_Y = (93.0, 94.0)
PLATE_Y, PLATE_X, PLATE_U = (94.0, 98.0), (-28.0, 28.0), (0.0, 40.0)   # carriage plate: 2 web beside the M5 holes; the head body generates from Y 98
# eccentric (24 Oct, arm-mech, DEFAULT printed bush `arm_eccbush` v1, no bought spacer): shank Ø11 in a Ø11.2 plate bore (Y 94..98), hex flange
# AF 13 x 2.5 on the plate top (Y 98..100.5), M5 bore 0.8 off its axis (wall 2.05); a 5 x 10 x 1 shim at Y 93..94 under it. The plate grows a
# boss r 8.6 round the bush (3.0 wall, the boss rule; to X -30.6). Head keep-outs over every axle: Ø10 x Y 98..106 (socket head + key); over the bush Ø26 x Y 98..101 (13 wrench)
ECC = dict(x=-22.0, u=18.0, shank_r=5.5, bore_in_plate_r=5.6, plate_boss_r=8.6, flange_af=13.0, flange_y=(98.0, 100.5), throw=0.8, bolt_r=2.65)
AXLE_KEEP = dict(r=5.0, y=(98.0, 106.0)); ECC_KEEP = dict(r=13.0, y=(98.0, 101.0))
WING_X, WING_U = (28.0, 46.5), (0.0, 30.0)   # plate wing over the fixed wheels out to the rack web (Y 94..98)
WEB_X, WEB_Y, WEB_U = (40.5, 46.5), (64.0, 94.0), (-47.0, 30.0)   # rack web, 6 thick: teeth on its +X face at Y 64..73, plain above (u -47: was -41, +6 drone push)
RACK_PITCH_X, RACK_TEETH_Y, RACK_LEN = 45.5, (64.0, 73.0), 77.0   # m1 rack: tips at 46.5, pitch 45.5, root 44.25; 77 = stroke 63.6 + 2 x 6 + 1.4
# --- pinion and motors ---
PIN_T, PIN_M = 16, 1.0
PIN_R, PIN_TIP_R, PIN_BOSS_R = 8.0, 9.0, 7.0
PIN_TEETH_Y, PIN_BOSS_Y = (64.5, 72.5), (56.5, 63.0)   # boss ends 1.0 under the rack web (24 Oct; a Ø12 neck to the teeth); on the NEMA D-shaft (Y 50..74, flat from Y 54): M3 grub in the boss at Y 60
PIN_Z = 55.0                                 # pinion axis along the head's travel: pinion u = PIN_Z - z0 stays >= 6 inside the rack for z0 32..89.6
PIN_X = RACK_PITCH_X + PIN_R                 # 53.5
MOTORS_V5 = {"A": dict(xc=PIN_X, zc=PIN_Z, rot=math.degrees(math.atan2(PIN_Z, PIN_X))),          # (53.5, 55), edge toward the axle
             "B": dict(xc=-PIN_Z, zc=-PIN_X, rot=math.degrees(math.atan2(-PIN_X, -PIN_Z)))}     # B mirrored: rack on B's -X = arm -Z: (-55, -53.5)
# --- stroke (unchanged from v4) ---
AXLE_BOLT_L = dict(fixed=25.0, ecc=25.0)   # M5 x 25 on every wheel (v5); arm_v6_layout sets the v6 stack's
RACK_TAIL = None                            # v6: the rack tail stepped up (arm_v6_layout)
DRONE_PUSH = 6.0                            # E (24 Oct, hook latch): the drone rises 2.1..5.7 (C) onto the plungers during the insert, so the insert runs up to 6 further
SLOW_LAST, SLOW_V = 12.0, 20.0              # last 12 of every insert at <= 20 mm/s (E, XT60 mate at 2 A)
Z0S = (L.Z0_A_HOME, L.Z0_B_WAIT, L.Z0_BAY + 1.5, L.Z0_BAY + 5.0, L.Z0_BAY + 5.0 + DRONE_PUSH)      # park 32, wait 81, bay 86.1, drone 89.6, drone insert 95.6
# --- fixed / turning parts kept from v4 (arm frame) ---
P80_FL_R, P80_Y = 27.5, (54.0, 64.0)
COLLAR_R, COLLAR_Y = 15.0, (64.0, 72.0)
RING_R, RING_Y = 23.5, (64.0, 70.0)
POST_Y = (52.0, 74.0)
POSTS_A = [(-16.0, 16.0, r - 6.0, r + 6.0) for r in L.POSTS_A]           # x0 x1 z0 z1
POSTS_B = [(-r - 6.0, -r + 6.0, -16.0, 16.0) for r in L.POSTS_B]
DROP = dict(x=-34.0 * math.cos(math.radians(45)), z=34.0 * math.sin(math.radians(45)), r=4.0, y=(50.0, 86.0))   # harness drop lane, 135 deg, r 34 (sweep chat: r 32 was 0.5 to the 80T flange)
SADDLE = dict(r=5.0, y=(78.0, 86.0))
LOOP_Y, LOOP_HW = (56.0, 62.0), 8.0
AX_P20_R, AX_P20_Y, AX_SHAFT_R, AX_SHAFT_Y = 8.0, (55.0, 63.0), 2.5, (63.0, 74.0)
NEMA, SEAT_R, R_HUB_CLEAR, C_AX = L.NEMA, 26.5, 46.5, 69.4
PLATE_HALF = 88.0

# ================================================================== 2D helpers (XZ plane)
def rect(x0, x1, z0, z1): return [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
def circ(xc, zc, r, n=24): return [(xc + r * math.cos(2 * math.pi * k / n), zc + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
def square(xc, zc, rot, half):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    return [(xc + sx * half * c - sz * half * s, zc + sx * half * s + sz * half * c) for sx, sz in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
def band(ax, az, bx, bz, hw):
    dx, dz = bx - ax, bz - az; ln = math.hypot(dx, dz); nx, nz = -dz / ln * hw, dx / ln * hw
    return [(ax + nx, az + nz), (bx + nx, bz + nz), (bx - nx, bz - nz), (ax - nx, az - nz)]
def sep(pa, pb):
    """convex polygons: separation >= 0 along the best edge normal (SAT); < 0 = overlap depth (approximate)"""
    best = -1e9
    for P, Q in ((pa, pb), (pb, pa)):
        for i in range(len(P)):
            x0, z0 = P[i]; x1, z1 = P[(i + 1) % len(P)]
            nx, nz = -(z1 - z0), (x1 - x0); ln = math.hypot(nx, nz); nx, nz = nx / ln, nz / ln
            pa_ = [nx * x + nz * z for x, z in P]; pb_ = [nx * x + nz * z for x, z in Q]
            best = max(best, min(pb_) - max(pa_), min(pa_) - max(pb_))
    return best
def yov(a, b): return min(a[1], b[1]) - max(a[0], b[0]) > 1e-9
def to_arm(h, poly_head):
    """head frame (X, Z=u+z0 already applied) -> arm frame: A identity; B = A mirrored in X (rack and eccentric on its -X),
    then B's frame into the arm frame: arm X = -Z_B, arm Z = X_B"""
    return poly_head if h == "A" else [(-z, -x) for x, z in poly_head]

# ================================================================== bodies
def head_bodies(h, z0):
    """every v5 carriage part of head h at stroke z0, in the arm frame: (name, poly, (y0, y1))"""
    out = []
    def add(nm, poly_hu, y):               # poly in (X, u)
        out.append((f"{h}:{nm}", to_arm(h, [(x, u + z0) for x, u in poly_hu]), y))
    add("plate", rect(*PLATE_X, *PLATE_U), PLATE_Y)
    add("eccboss", circ(ECC["x"], ECC["u"], ECC["plate_boss_r"]), PLATE_Y)
    add("wing", rect(*WING_X, *WING_U), PLATE_Y)
    add("web", rect(*WEB_X, *WEB_U), WEB_Y)
    for k, (x, u) in enumerate(WHEELS):
        add(f"wheel{k}", circ(x, u, WHEEL_OD / 2), WHEEL_Y)
        add(f"nut{k}", circ(x, u, NUT_R), NUT_Y)
        add(f"tail{k}", circ(x, u, TAIL_R), TAIL_Y)
    return out

def head_envelopes(h):
    """swept over the stroke (for the detent search and the module keep-out), arm frame"""
    z0s, z1s = min(Z0S), max(Z0S)
    env = []
    def add(nm, x0, x1, u0, u1, y): env.append((f"{h}:{nm}", to_arm(h, rect(x0, x1, u0 + z0s, u1 + z1s)), y))
    add("plate+wing", min(PLATE_X[0], ECC["x"] - ECC["plate_boss_r"]), WING_X[1], PLATE_U[0], PLATE_U[1], PLATE_Y)
    add("web", *WEB_X, *WEB_U, WEB_Y)
    add("wheels", -WHEEL_X - WHEEL_OD / 2, WHEEL_X + WHEEL_OD / 2, min(u for _, u in WHEELS) - WHEEL_OD / 2, max(u for _, u in WHEELS) + WHEEL_OD / 2, WHEEL_Y)
    add("nuts", -WHEEL_X - NUT_R, WHEEL_X + NUT_R, min(u for _, u in WHEELS) - NUT_R, max(u for _, u in WHEELS) + NUT_R, NUT_Y)
    add("tails", -WHEEL_X - TAIL_R, WHEEL_X + TAIL_R, min(u for _, u in WHEELS) - TAIL_R, max(u for _, u in WHEELS) + TAIL_R, TAIL_Y)
    return env

def rail_bodies(h):
    out = []
    r0, r1 = RAIL_R
    def add(nm, poly_hu, y): out.append((f"rail{h}:{nm}", to_arm(h, poly_hu), y))
    for r in RAIL_BOLTS["r_" + h]: add(f"foot{r:g}", rect(-RAIL_FOOT_X, RAIL_FOOT_X, max(r - 10.0, r0), r + 10.0), (RAIL_Y[0], RAIL_Y[0] + RAIL_FOOT_H))
    add("body", rect(-RAIL_BODY_X, RAIL_BODY_X, r0, r1), (RAIL_Y[0], RAIL_Y[1]))     # full depth Y 74..93 (24 Oct, arm_v5_cad: 3.1x stiffer than 80..93)
    add("ridges", rect(-RIDGE_FLAT_X, RIDGE_FLAT_X, r0, r1), (RIDGE_Y - RIDGE_HALF_H, RIDGE_Y + RIDGE_HALF_H))
    add("stop", rect(*RAIL_STOP["x"], *RAIL_STOP["r"]), RAIL_STOP["y"])
    return out

def arm_bodies(ax):
    """turning parts that are not the heads: posts, pinions, axle 20T + shaft, loop band, harness lane (arm frame)"""
    out = []
    for p in POSTS_A: out.append(("postA", rect(*p), POST_Y))
    for p in POSTS_B: out.append(("postB", rect(*p), POST_Y))
    for h, m in MOTORS_V5.items():
        out.append((f"pinion{h}", circ(m["xc"], m["zc"], PIN_TIP_R), PIN_TEETH_Y))
        out.append((f"boss{h}", circ(m["xc"], m["zc"], PIN_BOSS_R), PIN_BOSS_Y))
    out.append(("ax20T", circ(ax[0], ax[1], AX_P20_R), AX_P20_Y))
    out.append(("axshaft", circ(ax[0], ax[1], AX_SHAFT_R), AX_SHAFT_Y))
    out.append(("loop", band(0, 0, ax[0], ax[1], LOOP_HW), LOOP_Y))
    out.append(("drop", circ(DROP["x"], DROP["z"], DROP["r"]), DROP["y"]))
    out.append(("saddle", circ(0, 0, SADDLE["r"]), SADDLE["y"]))
    return out

def fixed_bodies():
    return [("80T", circ(0, 0, P80_FL_R, 48), P80_Y), ("collar", circ(0, 0, COLLAR_R), COLLAR_Y), ("ring", circ(0, 0, RING_R, 48), RING_Y)]

# pairs that touch by design (mesh / mount) are skipped
ALLOWED = {("A:web", "pinionA"), ("B:web", "pinionB"), ("A:wheel0", "railA:ridges"), ("A:wheel1", "railA:ridges"), ("A:wheel2", "railA:ridges"),
           ("B:wheel0", "railB:ridges"), ("B:wheel1", "railB:ridges"), ("B:wheel2", "railB:ridges"), ("railA:foot35", "postA"), ("railA:foot76", "postA"),
           ("railB:foot38.75", "postB"), ("railB:foot76", "postB"), ("loop", "80T"), ("ax20T", "loop"), ("collar", "ring")}

def clearance_sweep(ax):
    """min XZ gap of every pair that shares a Y band, over both heads' strokes. Returns the worst gap per pair (name tuple)."""
    worst = {}
    static = rail_bodies("A") + rail_bodies("B") + arm_bodies(ax) + fixed_bodies()
    for zA in Z0S:
        for zB in Z0S:
            moving = head_bodies("A", zA) + head_bodies("B", zB)
            allb = static + moving
            for (na, pa, ya), (nb, pb, yb) in itertools.combinations(allb, 2):
                if na.split(":")[0] == nb.split(":")[0] and ":" in na and ":" in nb: continue   # a head's own parts / a rail's own parts
                if (na, nb) in ALLOWED or (nb, na) in ALLOWED: continue
                if not yov(ya, yb): continue
                g = sep(pa, pb)
                key = (na, nb)
                if key not in worst or g < worst[key]: worst[key] = g
    return worst

# ================================================================== axle motor pose search (balance), v5 motors fixed by the racks
HEADS = [(300.0, 0.0, 52.0), (300.0, -66.0, 0.0), (44.0, 0.0, 81.5), (44.0, -81.5, 0.0)]   # heads + packs parked (E, arm_pose.py), rails 44 g at r 81.5 (E)
M_MOTOR = 300.0

def motor_ok(m, others):
    pts = square(m["xc"], m["zc"], m["rot"], NEMA / 2)
    if min(math.hypot(x, z) for x, z in pts) < R_HUB_CLEAR: return False
    if abs(m["xc"]) + SEAT_R > PLATE_HALF or abs(m["zc"]) + SEAT_R > PLATE_HALF: return False
    for o in others:
        if sep(pts, square(o["xc"], o["zc"], o["rot"], NEMA / 2)) < 3.0: return False
    return True

def axle_search(step=1.0):
    best = None
    mA, mB = MOTORS_V5["A"], MOTORS_V5["B"]
    hx = sum(m * x for m, x, z in HEADS) + M_MOTOR * (mA["xc"] + mB["xc"])
    hz = sum(m * z for m, x, z in HEADS) + M_MOTOR * (mA["zc"] + mB["zc"])
    th = -180.0
    while th < 180.0:
        ax = (C_AX * math.cos(math.radians(th)), C_AX * math.sin(math.radians(th)))
        m = dict(xc=ax[0], zc=ax[1], rot=th)
        if motor_ok(m, [mA, mB]):
            w = clearance_sweep(ax)
            gmin = min(w.values())
            if gmin >= CLR:
                unb = math.hypot(hx + M_MOTOR * ax[0], hz + M_MOTOR * ax[1])
                if best is None or unb < best["unbal_gmm"]: best = dict(th=th, xc=round(ax[0], 2), zc=round(ax[1], 2), unbal_gmm=round(unb, 0), gmin=round(gmin, 2), worst=w)
        th += step
    return best

# ================================================================== detent re-placement candidates (geometry of detent.design(), C)
R_SEAT, SPRING_LS, DET_A, DET_B = 25.5, 38.0, 15.0, 39.0
def detent_pts(th, s):
    t = math.radians(th); u = (math.cos(t), math.sin(t)); tn = (-u[1] * s, u[0] * s)
    C = (R_SEAT * u[0], R_SEAT * u[1]); P = (C[0] + DET_A * tn[0], C[1] + DET_A * tn[1])
    H = (P[0] + DET_B * u[0], P[1] + DET_B * u[1]); Aa = (H[0] - SPRING_LS * tn[0], H[1] - SPRING_LS * tn[1])
    return dict(C=C, P=P, H=H, A=Aa)

def detent_check(ax):
    """which (th, s) keep every detent part >= CLR off the v5 parts (2D, Y bands as detent.py): roller arm hull r 4.5 Y 71.6..74.1,
    pivot boss r 4.5 Y 66.3..71.3, hook pin r 1.5 Y 74..86, anchor post r 4 Y 66.3..74, springs Ø5 Y 75..86. The ring is unchanged
    only for th = -105 + 30 k."""
    obst = [b for b in rail_bodies("A") + rail_bodies("B") + arm_bodies(ax) if not b[0].startswith("post")]
    obst += head_envelopes("A") + head_envelopes("B")
    res = []
    for th in (-45.0, -75.0, -135.0, -165.0):
        for s in (1, -1):
            d = detent_pts(th, s)
            parts = [("arm", [(x, z) for p in (d["C"], d["P"], d["H"]) for x, z in circ(*p, 4.5, 12)], (71.6, 74.1)),
                     ("pivot", circ(*d["P"], 4.5), (66.3, 71.3)), ("hook", circ(*d["H"], 1.5), (74.0, 86.0)),
                     ("anchor", circ(*d["A"], 4.0), (66.3, 74.0)), ("springs", band(*d["H"], *d["A"], 2.5), (75.0, 86.0))]
            parts[0] = ("arm", hull2(parts[0][1]), parts[0][2])
            gmin, at = 1e9, None
            for nm, poly, y in parts:
                for on, op, oy in obst:
                    if not yov(y, oy): continue
                    g = sep(poly, op)
                    if g < gmin: gmin, at = g, (nm, on)
            res.append(dict(th=th, s=s, gmin=round(gmin, 2), at=at, ok=gmin >= CLR, pts={k: (round(v[0], 1), round(v[1], 1)) for k, v in d.items()}))
    return res

def hull2(pts):
    pts = sorted(set(pts))
    if len(pts) < 3: return pts
    def cross(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    up = []
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

# ================================================================== compaction (axial stack -> Y_C -> box length)
def stacks():
    """three stacks. v4 = belts on 2040 (as built); v5 = rails on hub v5's posts, DEFAULT; v5c = rails on the motor plate (hub v6 without
    posts) + Scout-length modules. Box length = funnel mouth (320 on Y_C) far edge + 2 (toe plate) back to the electronics Y -66."""
    fun_half = 160.0; y_back = -66.0; toe = 2.0
    rows = []
    for nm, rail_y, yc, mod, note in (("v4 (belts, universal modules)", (74.0, 94.0), 175.0, (94.0, 255.0), "glide top 97.3, pack from 98.5; universal module (Overseer length), Scout receiver centred"),
                                      ("v5 (rails on the posts, universal modules)", (74.0, 94.0), 175.0, (94.0, 255.0), "carriage plate Y 94..98; head body from 98; the sliced v4i modules stay"),
                                      ("v5 + C1 (Scout-length modules)", (74.0, 94.0), 138.0, (94.0, 184.0), "pack Y 100.5..179.5 (2.5 over the plate); needs a new module version (receiver Y 96..184)"),
                                      ("v5-compact = C1 + C2 (rails on the plate)", (54.0, 74.0), 118.0, (74.0, 164.0), "hub v6 without posts, rail foot on the plate Y 54; pinion plane Y 55..63; carriage plate 74..78; pack Y 80.5..159.5")):
        fy = (yc - fun_half, yc + fun_half)
        rows.append(dict(stack=nm, rail_y=rail_y, Y_C=yc, pack_y=(yc - L.Z_TABC, yc - L.Z_TABC + L.CART_L), modules_y=mod, funnel_y=fy,
                         enclosure_y=(y_back, fy[1] + toe), length=round(fy[1] + toe - y_back, 0), note=note))
    return rows

# ================================================================== parts delta (per arm: both heads)
PARTS_OUT = [("2040 extrusion 112", 2, "bars"), ("idler end A / B v3 (printed)", 2, ""), ("GT2 toothless idler", 2, ""), ("GT2 6 mm belt cut 445 / 457", 2, "+ 2 belt connectors"),
             ("M5 T-nut", 8, "bar posts + idler ends"), ("GT2 20T 5 bore", 2, "head motors (the axle 20T stays)"), ("glide (in heads A / B v5)", 2, "replaced by the carriage")]
PARTS_IN = [("rail A / B v1 (PETG, printed)", 2, "49.7 / 50.0 g M (arm_v5_cad): body 18 x 19 full depth, ridges, feet at the posts, park stop, ribbon channel"),
            ("V-wheel v1 (PETG, printed, OD 24 x 10, 90 deg groove)", 6, "2.8 g each M, on 2 x 625ZZ (press + epoxy)"), ("625ZZ bearing 5 x 16 x 5", 12, "~5 g each E"),
            ("M5 x 25 socket head + nyloc", 6, "wheel axles through the plate"), ("M5 precision shim 5 x 10 x 1", 6, "one under each wheel's inner race (Y 93..94)"),
            ("eccentric bush v1 (PETG, printed, 0.8 throw)", 2, "0.8 g each M, in the plate's eccentric bore"),
            ("pinion 16T m1 v1 (PETG, printed, Ø18 x 16, boss Ø14)", 2, "3.0 g each M, on the NEMA D-shaft, M3 grub"), ("M3 x 4 grub", 2, ""),
            ("M5 x 16 socket head", 4, "rail to post, one per post from the rail top"), ("M5 heat-set insert (OD 7.0 x 5.8, E)", 4, "one per hub v6 post top"),
            ("rack 77 m1 (printed into each head's carriage web)", 2, "in heads v6")]

def main():
    best = axle_search()
    if best is None:
        print("no axle pose clears everything: see clearance_sweep"); return
    print("== v5 layout (arm frame, C) ==")
    print(f"motors: A ({MOTORS_V5['A']['xc']:.1f}, {MOTORS_V5['A']['zc']:.1f}) rot {MOTORS_V5['A']['rot']:.1f}; B ({MOTORS_V5['B']['xc']:.1f}, {MOTORS_V5['B']['zc']:.1f}) rot {MOTORS_V5['B']['rot']:.1f}; "
          f"axle ({best['xc']}, {best['zc']}) at {best['th']:.0f} deg on C {C_AX}; residual unbalance {best['unbal_gmm'] / 1000:.1f} g.m (E; v4 4.4)")
    for h, m in MOTORS_V5.items():
        pts = square(m["xc"], m["zc"], m["rot"], NEMA / 2)
        print(f"  motor {h}: nearest to the axis {min(math.hypot(x, z) for x, z in pts):.1f} (>= 46.5), far corner r {max(math.hypot(x, z) for x, z in pts):.1f}, seat |x|,|z| + 26.5 = "
              f"{max(abs(m['xc']), abs(m['zc'])) + SEAT_R:.1f} (<= 88)")
    ax = (best["xc"], best["zc"])
    pa = square(*ax, best["th"], NEMA / 2)
    print(f"  axle motor: nearest {min(math.hypot(x, z) for x, z in pa):.1f}, far corner r {max(math.hypot(x, z) for x, z in pa):.1f}; loop band at {best['th']:.0f} deg")
    print(f"pinion: {PIN_T}T m{PIN_M:g} r {PIN_R}, axis at |X| {PIN_X}, Z {PIN_Z}; pinion u over the stroke {PIN_Z - max(Z0S):.1f}..{PIN_Z - min(Z0S):.1f} in the rack u {WEB_U[0]:.0f}..{WEB_U[1]:.0f} "
          f"(ends {PIN_Z - max(Z0S) - WEB_U[0]:.1f} / {WEB_U[1] - (PIN_Z - min(Z0S)):.1f} past the pinion centre, need >= 6)")
    st = max(Z0S) - min(Z0S)
    print(f"ratio: {PIN_R * 2 * math.pi:.1f} mm of travel per motor turn; stroke {st:.1f} = {st / (PIN_R * 2 * math.pi):.2f} turns; the last {SLOW_LAST:g} of an insert at {SLOW_V:g} mm/s = {SLOW_LAST / SLOW_V:.1f} s (E)")
    w = best["worst"]
    print(f"clearance sweep ({len(Z0S)}x{len(Z0S)} stroke states, {len(w)} pairs sharing a Y band): min {min(w.values()):.2f}")
    for (a, b), g in sorted(w.items(), key=lambda kv: kv[1])[:14]:
        print(f"  {g:6.2f}  {a} / {b}")
    # bisector rule (park): A's parts at X + z0 + u >= 1.5 (B mirrored)
    z0 = L.Z0_A_HOME
    bis = min([PLATE_X[0] + z0 + PLATE_U[0], ECC["x"] + z0 + ECC["u"] - ECC["plate_boss_r"] * math.sqrt(2)] + [x - WHEEL_OD / 2 + z0 + u - WHEEL_OD / 2 for x, u in WHEELS if x < 0] + [x - NUT_R + z0 + u - NUT_R for x, u in WHEELS if x < 0])
    print(f"park bisector rule: min X + z0 + u = {bis:.1f} (>= {L.BISECT})")
    print(f"axial gaps: rail top Y {RAIL_Y[1]:g} to the carriage plate Y {PLATE_Y[0]:g}: {PLATE_Y[0] - RAIL_Y[1]:.1f}; wheel Y {WHEEL_Y[0]:g} to the rail foot top Y {RAIL_Y[0] + RAIL_FOOT_H:g}: {WHEEL_Y[0] - RAIL_Y[0] - RAIL_FOOT_H:.1f}")
    # reach at the drone and the bay: the carriage's outer corners
    for nm, zz in (("drone insert", max(Z0S)), ("bay", L.Z0_BAY + 1.5)):
        corners = [(PLATE_X[0], PLATE_U[1]), (PLATE_X[1], PLATE_U[1]), (WING_X[1], WING_U[1])]
        rc = max(math.hypot(x, zz + u) for x, u in corners)
        rw = math.hypot(WHEEL_X, zz + WHEELS[1][1] + WHEEL_OD / 2)
        ang = max(math.degrees(math.atan2(x, zz + u)) for x, u in corners)
        need = max(rc, rw, math.hypot(RIDGE_X, RAIL_R[1])) + CLR
        print(f"reach at the {nm}: plate corner r {rc:.1f} (Y 94..98), wheel rim r {rw:.1f} (Y 83..93), {ang:.1f} deg off the head axis; rail tip r {math.hypot(RIDGE_X, RAIL_R[1]):.1f}; "
              + (f"no bay module sits in the drone opening (60..120 deg, the carriage spans +-{ang:.1f} deg round 90); stand arm lane r 145.5: {145.5 - max(rc, rw):.1f} clear"
                 if nm.startswith("drone") else f"bay module keep cylinder (v4i r 136.5 to Y 95.5) needs r >= {need:.1f} to Y {PLATE_Y[1]:g}: r 136.5 holds, Y grows to 98"))
    print(f"wheel on the ridge: outer wheel contact at u {WHEELS[1][1]} -> r {max(Z0S) + WHEELS[1][1]:.1f} at the drone ({RAIL_R[1] - max(Z0S) - WHEELS[1][1]:.1f} of ridge left); inner at park r {min(Z0S) + WHEELS[0][1]:.1f} ({min(Z0S) + WHEELS[0][1] - RAIL_R[0]:.1f} from the rail start)")
    # loads (E)
    F_push = 60.0; lever = (L.Y_C - sum(RACK_TEETH_Y) / 2)                 # the rack takes the push at its teeth (Y 68.5)
    print(f"loads (E): insert push {F_push:.0f} N at Y_C -> couple {F_push * lever / 1000:.2f} N.m on the carriage, {F_push * lever / (WHEELS[1][1] - WHEELS[0][1]):.0f} N axial on each outer fixed wheel "
          f"({F_push * lever / (WHEELS[1][1] - WHEELS[0][1]) * math.sqrt(2):.0f} N normal if one flank takes it; 625ZZ C0 ~670 N); "
          f"pinion tangential {F_push:.0f} N, separating {F_push * math.tan(math.radians(20)):.1f} N; motor stall 0.59 N.m / {PIN_R / 1000} m = {0.59 / (PIN_R / 1000):.0f} N (misuse)")
    I_web = (WEB_Y[1] - WEB_Y[0]) * (WEB_X[1] - WEB_X[0]) ** 3 / 12
    Lt = -WEB_U[0] - 6.0
    Fs = F_push * math.tan(math.radians(20))
    print(f"rack web tail: {WEB_X[1] - WEB_X[0]:.0f} x {WEB_Y[1] - WEB_Y[0]:.0f} x {-WEB_U[0]:.0f} hanging; separating {Fs:.1f} N at the drone insert -> {Fs * Lt ** 3 / (3 * 2000 * I_web):.2f} mm sideways (E, PETG 2 GPa), backlash 0.1 per flank (pinion v1)")
    # detent
    det = detent_check(ax)
    print("detent re-placement (ring v1 unchanged for th = -105 + 30 k; detent.py search() reruns with the v5 obstacles):")
    for d in det:
        print(f"  th {d['th']:6.0f} s {d['s']:+d}: {'OK ' if d['ok'] else 'no '} min gap {d['gmin']:6.2f} {d['at']}  C {d['pts']['C']} P {d['pts']['P']} H {d['pts']['H']} anchor {d['pts']['A']}")
    print("compaction:")
    rows = stacks()
    for r in rows:
        print(f"  {r['stack']:45s} rails Y {r['rail_y'][0]:.0f}..{r['rail_y'][1]:.0f}  Y_C {r['Y_C']:.0f}  pack Y {r['pack_y'][0]:.1f}..{r['pack_y'][1]:.1f}  modules Y {r['modules_y'][0]:.0f}..{r['modules_y'][1]:.0f}  "
              f"funnel Y {r['funnel_y'][0]:.0f}..{r['funnel_y'][1]:.0f}  enclosure Y {r['enclosure_y'][0]:.0f}..{r['enclosure_y'][1]:.0f} = {r['length']:.0f} long")
    print("parts out:", "; ".join(f"{q} x {n}" for n, q, _ in PARTS_OUT))
    print("parts in: ", "; ".join(f"{q} x {n}" for n, q, _ in PARTS_IN))
    data = dict(date="2026-10-24", status="DONE", mass_tag="C",
                note=f"arm v5 layout (pure python): rails + 3-wheel carriages + rack and pinion; min gap {best['gmin']:.2f} over the stroke; axle motor at {best['th']:.0f} deg, unbalance {best['unbal_gmm'] / 1000:.1f} g.m E",
                motors=dict(**MOTORS_V5, axle=dict(xc=best["xc"], zc=best["zc"], rot=best["th"])), unbal_gmm=best["unbal_gmm"], gmin=best["gmin"],
                worst=[dict(pair=list(k), gap=round(v, 2)) for k, v in sorted(w.items(), key=lambda kv: kv[1])[:30]],
                rail=dict(y=RAIL_Y, r=RAIL_R, foot_x=RAIL_FOOT_X, foot_h=RAIL_FOOT_H, body_x=RAIL_BODY_X, ridge_x=RIDGE_X, ridge_y=RIDGE_Y, ridge_half_h=RIDGE_HALF_H, chan=RAIL_CHAN, bolts=RAIL_BOLTS, stop=RAIL_STOP),
                stroke=dict(z0s=Z0S, drone_push=DRONE_PUSH, slow_last=SLOW_LAST, slow_v=SLOW_V), ecc=ECC, axle_keep=AXLE_KEEP, ecc_keep=ECC_KEEP,
                carriage=dict(wheel_od=WHEEL_OD, wheel_w=WHEEL_W, wheel_y=WHEEL_Y, wheels=WHEELS, nut_y=NUT_Y, plate_y=PLATE_Y, plate_x=PLATE_X, plate_u=PLATE_U, wing_x=WING_X, wing_u=WING_U,
                              web_x=WEB_X, web_y=WEB_Y, web_u=WEB_U, rack_pitch_x=RACK_PITCH_X, rack_teeth_y=RACK_TEETH_Y),
                pinion=dict(T=PIN_T, m=PIN_M, r=PIN_R, tip_r=PIN_TIP_R, boss_r=PIN_BOSS_R, teeth_y=PIN_TEETH_Y, boss_y=PIN_BOSS_Y, x=PIN_X, z=PIN_Z),
                detent=det, stacks=rows, parts_out=PARTS_OUT, parts_in=PARTS_IN)
    json.dump(data, open(f"{OUT}/result.json", "w"), indent=1)
    open(f"{OUT}/summary.md", "w").write(
        "arm v5-layout: layout check, no mesh (pure python, C)\n"
        f"Motors: A ({MOTORS_V5['A']['xc']:.1f}, {MOTORS_V5['A']['zc']:.1f}), B ({MOTORS_V5['B']['xc']:.1f}, {MOTORS_V5['B']['zc']:.1f}), axle ({best['xc']}, {best['zc']}) at {best['th']:.0f} deg (C); unbalance {best['unbal_gmm'] / 1000:.1f} g.m (E)\n"
        f"Clearance: min {best['gmin']:.2f} over {len(Z0S)}x{len(Z0S)} stroke states (wheels to the rail body, designed, C); bisector {bis:.1f} (C)\n"
        "Detent: detent.CHOSEN_V5 = roller at -135 deg, pivot side +1, levers 15 / 39, hook along the tangent (psi 315), tower at (18.5, -28.4) (C); "
        "detent.py search(v5=True) / `python3 detent.py v5`: arm parts 1.14, tower 1.0 (C); ring v1 unchanged; the 2D lever-only check above uses v1's radial hook\n"
        f"Compaction: {' / '.join(f'{r['length']:.0f}' for r in rows)} long (C; v4 / v5 / +C1 / compact)\n"
        "Solid: n/a (no part built here; rails / wheels / pinion / eccentric bush: nox/box/arm_v5_cad.py, runs/arm_rail/v1/)\n")
    print(f"-> {OUT}/result.json, summary.md")

if __name__ == "__main__":
    main()
