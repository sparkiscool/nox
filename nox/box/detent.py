"""Arm auto-centering detent (17 Oct 2026, Spark 04:16; goes on the next arm version: hub v5). One detent at the pivot
serves both heads: a printed DETENT RING fixed on top of the 80T sun (clamped by the 80T's 3 M3, bore on its boss), one V-notch
per arm stop; a 683ZZ roller on a short pivoting arm on the hub, pulled into the ring by a 0.5 x 5 x 25 steel extension
spring (a second spring hooks on the same pins, one groove up). The stepper moves close, drops its current (driver off),
the detent centers the arm, then full current again = the lock (Spark 04:17). Homing: one hall switch on the roller tower,
one magnet in the ring at the phi 0 notch.

Frames: W and the arm frame of arm_layout (arm frame = W at phi 90; a point at arm angle th is at W angle th + phi - 90).
python3 detent.py -> stops, notch angles, the placement search, forces, STLs (print orientation) in out/.
All numbers C unless tagged E (estimate) or P (spec / listing)."""
import math, itertools, os
import numpy as np
import manifold3d as mf
import arm_layout as L, arm_cad as A
cross2 = lambda p, q: float(p[0] * q[1] - p[1] * q[0])
from arm_cad import box, cylY, hull

OUT = "out"; os.makedirs(OUT, exist_ok=True)
CLR = 1.0                                   # every fixed-to-turning gap (DEFAULT, = CLR_POST)

# ---------------- bought parts (E unless P) ----------------
ROLLER_R, ROLLER_W, ROLLER_BORE = 3.5, 3.0, 3.0      # 683ZZ 3 x 7 x 3 (P spec)
NUT_R, NUT_T, WASHER_T = 3.2, 1.8, 0.5               # nut Y 65.0..66.8: 1.0 over the 80T top               # M3 thin nut DIN 439 (corners), M3 washer (P spec)
SPRING_FREE, SPRING_F0, SPRING_K = 25.5, 0.9, 0.2    # 0.5 wire x 5 OD x 25 extension spring: hook centres free 25.5, 0.9 N, 0.2 N/mm
SPRING_LMAX = 42.0                                   # 4.2 N at 42 (Spark 04:16, E): keep the working length under it
SPRING_OD = 5.0
HALL = dict(body=(4.0, 3.0, 1.5), magnet=(3.0, 2.0))  # A3144 TO-92S body (E); N52 magnet Ø3 x 2 (E)

# ---------------- ring (fixed) ----------------
RING_Y = (L.Y_P80[1], 69.9)    # on the 80T's top face (Y 54 + P80_W): the base thins if the body is wider, rim stays at Y 67.8..69.9
RIM_Y0 = 67.8                  # notched rim Y 67.8..69.9; below it the ring steps in to r 21 (room for the roller's nut)
RING_R_IN = L.P80_BOSS / 2 + 0.25    # 12.25 on the 80T boss (17 Oct prep: +0.25 for a printed bore, was 12.1)
RING_R_OUT = 23.5              # head A's clamp plate sweeps r >= 24.6 at Y 64.6..73.7 (belt clamp v1): 1.1 off
RING_R_LOW = 21.0              # lower step
NOTCH_HALF = 30.0              # V half angle (60 included): restoring force = N / tan 30 = 1.73 N
ROLL_LIFT = 1.5                # roller centre rises 1.5 between seated and on the land
R_SEAT = RING_R_OUT + ROLLER_R - ROLL_LIFT                   # 25.5 roller centre seated
R_APEX = R_SEAT - ROLLER_R / math.sin(math.radians(NOTCH_HALF))   # 18.5 V apex
BOLT_CB_R = 3.0                # M3 low head (DIN 7984, head Ø5.5 x 2) counterbore Ø6 x 2.2 at the 80T bolts (r 18)
MAGNET_PHI = 0.0               # home: head A at the 0 deg bay (cycle start)
HALL_TH = -60.0                # hall switch at arm angle -60: a land (notches at the roller angle +-30 k), on the tower's web

# ---------------- roller arm (turns) ----------------
ARM_Y = (71.6, 74.1)           # arm plate; roller Y 67.3..70.3 under it on a Ø4.5 spacer (inner race only), washer + thin nut Y 65.0..67.3
ROLLER_Y = (67.3, 70.3)
SPACER_R = 2.25


def stops():
    """arm angles phi where the arm must stop: head A (phi) or head B (phi + 90) on the drone (90) or a bay, inside the
    soft limits. Returns sorted phi list and the notch angles in W (mod 360) for a roller at arm angle th."""
    pos = [90.0] + list(L.BAY_ANGLES)
    lo, hi = -278.0, 143.0                         # firmware soft limits (axle drive v4, C)
    ph = set()
    for p in pos:
        for off in (0.0, -90.0):                   # A on p; B on p (B points at phi + 90)
            for k in range(-2, 3):
                v = p + off + 360 * k
                if lo <= v <= hi: ph.add(round(v, 6))
    return sorted(ph)


def notch_angles(th):
    return sorted(set(round((th + p - 90.0) % 360.0, 6) for p in stops()))


def ring(th):
    """detent ring (PETG, fixed), W coords, for a roller at arm angle th."""
    r = cylY(0, 0, RING_Y[0], RIM_Y0, RING_R_LOW, 128) + cylY(0, 0, RIM_Y0 - 0.01, RING_Y[1], RING_R_OUT, 180)
    r -= cylY(0, 0, RING_Y[0] - 1, RING_Y[1] + 1, RING_R_IN, 96)
    for a in notch_angles(th):
        r -= _notch_prism(a, RIM_Y0 - 0.5, RING_Y[1] + 1)
    for a in L.SUN_BOLT_ANG:                                     # through the 80T's bolts (M3 x 20, low head, E: CHECK the 80T thickness)
        x, z = L.part_angle_pos(L.SUN_BC_R, a)
        r -= cylY(x, z, RING_Y[0] - 1, RING_Y[1] + 1, 1.7) + cylY(x, z, RING_Y[1] - 2.2, RING_Y[1] + 1, BOLT_CB_R)
    t = math.radians(HALL_TH + MAGNET_PHI - 90.0)                # magnet Ø3 x 2 pressed radially into the land the hall sees at home
    u = np.array([math.cos(t), math.sin(t)]); ym = (RIM_Y0 + RING_Y[1]) / 2
    mag = mf.Manifold.cylinder(2.3, HALL["magnet"][0] / 2 + 0.05, HALL["magnet"][0] / 2 + 0.05, 24).rotate([0, 90, 0]) \
        .translate([RING_R_OUT - 2.2, 0, 0]).rotate([0, -math.degrees(t), 0]).translate([0, ym, 0])
    r -= mag
    return r


def _notch_prism(a_deg, y0, y1):
    t = math.radians(a_deg); u = np.array([math.cos(t), math.sin(t)]); n = np.array([-u[1], u[0]])
    k = math.tan(math.radians(NOTCH_HALF)); far = RING_R_OUT + 5.0
    pts2 = [R_APEX * u, far * u + (far - R_APEX) * k * n, far * u - (far - R_APEX) * k * n]
    return hull([[p[0], y, p[1]] for p in pts2 for y in (y0, y1)])


def ring_checks(th):
    """notch count, capture half angle, and the wall left between any notch and a bolt counterbore (min_gap, C)."""
    k = math.tan(math.radians(NOTCH_HALF))
    cb = None
    for b in L.SUN_BOLT_ANG:
        x, z = L.part_angle_pos(L.SUN_BC_R, b); c = cylY(x, z, 0, 1, BOLT_CB_R, 48); cb = c if cb is None else cb + c
    nt = None
    for a in notch_angles(th):
        p = _notch_prism(a, 0, 1); nt = p if nt is None else nt + p
    return dict(n_notches=len(notch_angles(th)), capture_deg=round(math.degrees((RING_R_OUT - R_APEX) * k / RING_R_OUT), 1),
                wall_notch_to_bolt=round(cb.min_gap(nt, 10.0), 2), r_seat=R_SEAT, r_apex=round(R_APEX, 2))


# ---------------- forces (E) ----------------
def disturbance():
    """torque the detent must beat with the driver off (E): gravity of the worst carried state at a horizontal stop,
    2RS seal drag, the stepper's unpowered cogging through the 4:1."""
    g = 9.81
    resid = 4.4e-3 * g                               # arm unbalance 4.4 g.m (arm_pose, E)
    pack = (0.165 + 0.0216) * g * 0.0714            # Scout pack + cartridge, centre r 71.4 when carried (P / M / C)
    head = 0.060 * g * (L.Z0_BAY + 22.6 - L.Z0_A_HOME) * 1e-3   # one head at a bay instead of parked (60 g E)
    seals = 2 * 0.03                                 # 6010-2RS seal drag (E)
    cog = 0.022 * L.AX_SUN_T / L.AX_PIN_T            # NEMA17 detent torque 2.2 N.cm (E) x 4
    parts = dict(gravity=resid + pack + head, seals=seals, cogging=cog)
    parts["total"] = sum(parts.values())
    return {k: round(v, 3) for k, v in parts.items()}


def spring_force(Ls):
    return SPRING_F0 + SPRING_K * (Ls - SPRING_FREE)


# ---------------- placement search ----------------
def obstacles():
    """what the roller arm and its tower must clear, in the arm frame (axisymmetric fixed parts as solids of revolution)."""
    S = A.axle_sun_bodies(); M = L.MOTORS
    fixed = S["p80"] + cylY(0, 0, 40.0, L.POST_Y1, L.POST_R)                 # + the ring itself, per roller angle (ring_fixed)
    turn = S["loop"] + S["p20"]
    turn += cylY(M["A"]["xc"], M["A"]["zc"], 64.0, 74.0, L.P20_FL / 2)                       # head A 20T
    xa = L.X_HBELT_C; turn += box(xa - 7.6, xa + 7.6, L.Y_HBELT[0], L.Y_HBELT[1], M["A"]["zc"], L.Z_IDLER)   # head A belt band
    for z in L.POSTS_A: turn += box(-16, 16, 52, 74, z - 6, z + 6)
    for x in L.POSTS_B: turn += box(-x - 6, -x + 6, 52, 74, -16, 16)
    turn += box(-20, 20, 74, 94, 22, 134) + box(-134, -22, 74, 94, -20, 20)                  # bars
    turn += box(-24.5, 24.5, 73.5, 98, 30, 134) + box(-134, -30, 73.5, 98, -24.5, 24.5)      # glides over their stroke (+0.5)
    turn += box(20.0, 34.0, 63.5, 79, 11.0, 134)                                               # head A clamp + plate, park..drone
    turn += box(-134, -30, 50, 79, 34.0, 58.0)                                                 # head B clamp, park..drone
    return fixed, turn


def design(th, s, a, b, phi_d, psi=0.0):
    """roller seated at arm angle th; pivot a along the tangent (side s); bellcrank: hook b from the pivot at psi deg from
    the tangent (0 = straight lever past the pivot, +90 = radially out); spring from the hook toward phi_d (arm frame) to the
    anchor, working length SPRING_LS. Returns points (XZ) and the force ratio roller N / spring F (negative = wrong way)."""
    t = math.radians(th)
    u = np.array([math.cos(t), math.sin(t)]); tn = np.array([-u[1], u[0]]) * s
    C = R_SEAT * u; P = C + a * tn
    ps = math.radians(psi); H = P + b * (math.cos(ps) * tn + math.sin(ps) * u)
    d = np.array([math.cos(math.radians(phi_d)), math.sin(math.radians(phi_d))])
    Aa = H + SPRING_LS * d
    lever_h = abs(cross2(H - P, d)); lever_c = abs(cross2(C - P, -u))
    sgn = cross2(H - P, d) * cross2(C - P, -u)
    return dict(th=th, C=C, P=P, H=H, A=Aa, u=u, tn=tn, ratio=lever_h / lever_c if sgn > 0 else -1.0)

CHOSEN = dict(th=-105, s=1, a=15.0, b=39.0, phi_d=-105 + 270, psi=90.0, Q=(17.0, -29.4))   # search() result (C), see box-arm.md
SPRING_LS = 38.0               # working length seated (hook centres): 3.4 N; on the land 1.5 x ratio longer, kept <= 42


def arm_solids(D, Q=None):
    """roller arm (printed, turns about P), roller + spacer + washer/nut, hook pin, spring(s), anchor pin; with Q the hub v5
    tower: a Ø10 column at Q from the plate, webs (Y WEB) from it to the pivot boss at P and to the anchor post at A."""
    C, P, H, Aa = D["C"], D["P"], D["H"], D["A"]
    y0, y1 = ARM_Y
    circ = lambda p, r, n=16: [(p[0] + r * math.cos(q), p[1] + r * math.sin(q)) for q in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    arm = hull([[x, y, z] for p in (C, P, H) for (x, z) in circ(p, 4.5) for y in (y0, y1)])
    arm -= cylY(P[0], P[1], y0 - 1, y1 + 1, 1.6) + cylY(C[0], C[1], y0 - 1, y1 + 1, 1.6) + cylY(H[0], H[1], y0 - 1, y1 + 1, 1.45)
    spacer = cylY(C[0], C[1], ROLLER_Y[1], y0 + 0.01, SPACER_R, 32)
    hook = cylY(H[0], H[1], y1 - 0.01, y1 + 12.0, 1.5)
    roller = cylY(C[0], C[1], ROLLER_Y[0], ROLLER_Y[1], ROLLER_R, 48)
    nut = cylY(C[0], C[1], ROLLER_Y[0] - WASHER_T - NUT_T, ROLLER_Y[0], NUT_R, 6)
    springs = [hull([[x, yy, z] for p in (H, Aa) for (x, z) in circ(p, SPRING_OD / 2, 12) for yy in (ys - 2.5, ys + 2.5)])
               for ys in SPRING_Y]
    anchor = cylY(Aa[0], Aa[1], SPRING_Y[0] - 3.6, SPRING_Y[1] + 3.0, 1.5)
    out = dict(arm=arm, spacer=spacer, hook=hook, roller=roller, nut=nut, springs=springs, anchor=anchor, tower=None)
    if Q is not None:
        w0, w1 = WEB_Y
        col = cylY(Q[0], Q[1], L.Y_PLATE1 - 0.5, w1, 5.0, 32)
        web = hull([[x, y, z] for p in (Q, P) for (x, z) in circ(p, 4.0) for y in (w0, w1)])
        web += hull([[x, y, z] for p in (Q, Aa) for (x, z) in circ(p, 4.0) for y in (w0, w1)])
        apost = cylY(Aa[0], Aa[1], w0, SPRING_Y[0] - 3.5, 4.0, 32)          # anchor post up to 1 under spring 1; the pin rises from it
        piv = cylY(P[0], P[1], w0, w1, 4.0, 32)
        t = math.radians(HALL_TH); u = np.array([math.cos(t), math.sin(t)]); n = np.array([-u[1], u[0]])
        hb = hull([[*(p), ] for p in []]) if False else None
        pts = [RING_R_OUT + CLR + k * u * 0 for k in (0,)]
        r0, r1 = RING_R_OUT + CLR, RING_R_OUT + CLR + 5.5
        hall_blk = hull([[ (rr * u + w * n)[0], y, (rr * u + w * n)[1]] for rr in (r0, r1) for w in (-3.0, 3.0) for y in (w0, w1)])
        hall_blk += hull([[x, y, z] for p in (Q, r1 * u) for (x, z) in circ(p, 3.0) for y in (w0, w1)])
        hb_x, hb_y, hb_z = HALL["body"]
        pocket = hull([[ (rr * u + w * n)[0], y, (rr * u + w * n)[1]] for rr in (r0 - 0.1, r0 + hb_z + 0.2) for w in (-hb_x / 2 - 0.2, hb_x / 2 + 0.2)
                       for y in ((RIM_Y0 + RING_Y[1]) / 2 - hb_y / 2 - 0.2, w1 + 0.1)])                 # A3144 dropped in from the top, face 1.0 off the rim
        tw = col + web + apost + piv + hall_blk - pocket
        tw -= cylY(P[0], P[1], w0 - 1, w1 + 1, 1.45) + cylY(Aa[0], Aa[1], SPRING_Y[0] - 10.0, SPRING_Y[0], 1.45)
        out["tower"] = tw
    return out

SPRING_Y = (y_ := ARM_Y[1] + 3.0, y_ + 6.0)   # spring 1 centre Y 77.6, spring 2 (parallel, optional) Y 83.6, both on the same two pins
WEB_Y = (ROLLER_Y[0] - 1.0, ARM_Y[0] - 0.3)   # 66.3..71.3: under the arm, over the 80T (2.3) and beside the ring


def evaluate(D, fixed, turn):
    """min gaps of the turning detent parts: vs fixed (the roller rides the ring; spacer and nut 0.4 / 0.5 allowed, they sit
    in the notch void and under the rim), vs the other turning parts."""
    S = arm_solids(D); rg = _ring_cache(D["th"])
    moving = S["arm"] + S["hook"] + S["springs"][0] + S["springs"][1] + S["anchor"]
    g_fix = min(moving.min_gap(fixed + rg, 3.0), S["roller"].min_gap(fixed, 3.0),
                S["spacer"].min_gap(fixed + rg, 3.0) + 0.6, S["nut"].min_gap(fixed + rg, 3.0) + 0.5)
    g_turn = (moving + S["roller"] + S["spacer"] + S["nut"]).min_gap(turn, 3.0)
    return min(g_fix, g_turn)


_RC = {}
def _ring_cache(th):
    if th not in _RC: _RC[th] = ring(th)
    return _RC[th]


def tower_gap(D, Q, fixed, turn):
    S = arm_solids(D, Q); tw = S["tower"]; fixed = fixed + _ring_cache(D["th"])
    above = turn - box(-300, 300, -10, L.Y_PLATE1, -300, 300)          # the tower grows out of the plate
    return min(tw.min_gap(fixed, 3.0), tw.min_gap(above, 3.0),
               tw.min_gap(S["roller"] + S["nut"] + S["spacer"] + S["springs"][0] + S["springs"][1] + S["hook"], 3.0),
               (tw - cylY(D["P"][0], D["P"][1], 0, 200, 6.0)).min_gap(S["arm"], 3.0) + 0.7)   # arm rides on the pivot boss only


def search():
    fixed, turn = obstacles()
    T = disturbance()["total"]
    cands = []
    for th, s, a, br, phi_d, psi in itertools.product(range(-165, -14, 30), (-1, 1), (9, 11, 13), (2.2, 2.4, 2.6),
                                                      range(0, 360, 15), (0, 45, 90, 135, 180, 225, 270, 315)):
        D = design(th, s, a, a * br, th + phi_d, psi)
        if D["ratio"] <= 0: continue
        N = spring_force(SPRING_LS) * D["ratio"]
        Tmax = N / math.tan(math.radians(NOTCH_HALF)) * R_SEAT * 1e-3
        if Tmax < 0.9 * T or SPRING_LS + ROLL_LIFT * br > SPRING_LMAX: continue
        cands.append((Tmax, th, s, a, br, phi_d, psi, D))
    cands.sort(key=lambda c: -c[0])
    best = None; tried = 0
    for Tmax, th, s, a, br, phi_d, psi, D in cands:
        if D["A"] @ D["A"] > 75.0 ** 2: continue
        g = evaluate(D, fixed, turn)
        if g < CLR: continue
        tried += 1
        mid = (D["P"] + D["A"]) / 2
        for rq in np.arange(32.0, 72.0, 4.0):
            for aq in np.arange(-180.0, 0.1, 7.5):
                Q = rq * np.array([math.cos(math.radians(aq)), math.sin(math.radians(aq))])
                if np.linalg.norm(Q - mid) > 40: continue
                cost = np.linalg.norm(Q - D["P"]) + np.linalg.norm(Q - D["A"])
                if best is not None and Tmax < best[0] - 1e-9 and cost > best[-1]: continue
                gt = tower_gap(D, Q, fixed, turn)
                if gt < CLR: continue
                cand = (Tmax, th, s, a, br, phi_d, psi, round(g, 2), round(gt, 2), Q, cost)
                if best is None or (Tmax, -cost) > (best[0], -best[-1]): best = cand
        if best is not None and tried > 40: break
    return best, dict(T=T, candidates=len(cands), tried=tried)


if __name__ == "__main__":
    print("stops phi", stops())
    print("disturbance (E)", disturbance())
    import sys
    if "search" in sys.argv:
        best, info = search(); print("search", info); print("best", best)
    c = CHOSEN; Dc = design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); Q = np.array(c["Q"])
    fixed, turn = obstacles()
    T = disturbance()["total"]; F1 = spring_force(SPRING_LS); N1 = F1 * Dc["ratio"]
    Tm = N1 / math.tan(math.radians(NOTCH_HALF)) * R_SEAT * 1e-3
    print("chosen", {k: (np.round(v, 1).tolist() if isinstance(v, np.ndarray) else round(v, 3) if isinstance(v, float) else v) for k, v in Dc.items() if k not in ("u", "tn")})
    print("ring", ring_checks(c["th"]))
    print(f"spring {F1:.2f} N seated (L {SPRING_LS}), {spring_force(SPRING_LS + ROLL_LIFT * c['b'] / c['a']):.2f} N on a land (L {SPRING_LS + ROLL_LIFT * c['b'] / c['a']:.1f})")
    print(f"roller N {N1:.1f} N (1 spring); holding torque {Tm:.3f} N.m vs disturbance {T:.3f}: margin 1 spring {Tm / T:.2f}, 2 springs {2 * Tm / T:.2f}")
    print("gaps: arm parts", round(evaluate(Dc, fixed, turn), 2), "tower", round(tower_gap(Dc, Q, fixed, turn), 2))
    S = arm_solids(Dc, Q); rg = ring(c["th"])
    for nm, m in (("ring", rg), ("arm", S["arm"] + S["spacer"]), ("tower (hub v5 drawn)", S["tower"])):
        print(f"  {nm:22s} {m.volume() / 1000 * 1.27:5.1f} g PETG solid")
    import trimesh
    def ex(m, name, rot):
        t = A.tm(m); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot), [1, 0, 0])); t.apply_translation(-t.bounds[0]); t.export(f"{OUT}/{name}")
    ex(rg, "box_detent_ring_v1.stl", 90)                                  # bottom (Y 64 face) on the bed
    ex(S["arm"] + S["spacer"], "box_detent_arm_v1.stl", -90)              # arm top face on the bed, spacer up
