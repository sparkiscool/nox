"""Detent v6 for the swap arm v7 (24 Oct 2026, swap arm system worker; the piece pass after the v7 concept check, springs x1.3 at 5:1):
the round 3d recheck found the 683ZZ (Ø7 x 3) roller on ring v3's 2.1 rim at ~54 MPa Hertz (index ~0.8, every swap rolls it) and arm v2 at
0.68 at its pivot. v6: a 623ZZ roller (Ø10 x 4, bore 3: the 3.0 carbon axle as v2), ring v4 (r_out 25.0, rim Y 65.9..69.9 = the roller's
width, notch apex r 18.5 as v3), arm v3 (plate 3.5, bosses r 6.0). The tower stays at CHOSEN_V7's Q (31.1, -31.1) so hub v6's drawn tower
and its topology run hold; only the levers are re-searched (`search_fixed_q`). Numbers C geometry, E forces.
python3 detent_v6.py [search] -> runs/detent/v6/ (ring v4, arm v3 meshes, checks.json)"""
import os, sys, math, json, itertools
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_v7_layout as S7
S7.use()
import detent as DT
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
RUN = os.path.join(ROOT, "runs", "detent", "v6"); os.makedirs(RUN, exist_ok=True)

V6 = dict(ROLLER_R=5.0, ROLLER_W=4.0, ROLLER_BORE=3.0, RING_R_OUT=25.0, RIM_Y0=65.9, ROLLER_Y=(65.9, 69.9), ARM_Y=(70.5, 74.0), ARM_BOSS_R=6.0)
SPRING_SCALE = S7.DETENT_SPRING_SCALE      # x1.3 (arm_v7_layout)
E_PETG, S_IN = 2000.0, 45.0


def use():
    for k, v in V6.items(): setattr(DT, k, v)
    DT.R_SEAT = DT.RING_R_OUT + DT.ROLLER_R - DT.ROLL_LIFT                          # 28.5
    DT.R_APEX = DT.R_SEAT - DT.ROLLER_R / math.sin(math.radians(DT.NOTCH_HALF))     # 18.5
    DT.SPRING_Y = (DT.ARM_Y[1] + 3.0, DT.ARM_Y[1] + 9.0)
    DT.WEB_Y = (DT.ROLLER_Y[0] - 1.0, DT.ARM_Y[0] - 0.3)
    DT._RC.clear()


def holding(D):
    """per spring, scaled: the roller force on a land and the holding torque seated (E)"""
    F_seat = DT.spring_force(DT.SPRING_LS) * SPRING_SCALE
    F_land = DT.spring_force(DT.SPRING_LS + DT.ROLL_LIFT * D["ratio"]) * SPRING_SCALE
    T = F_seat * D["ratio"] / math.tan(math.radians(DT.NOTCH_HALF)) * DT.R_SEAT * 1e-3
    return F_seat, F_land, T


def stresses(D):
    """two springs on a land (the worst): roller on the rim (Hertz line contact, steel on PETG), the arm at its pivot (in-plane bending)"""
    _, F_land, _ = holding(D); F = 2 * F_land; N = F * D["ratio"]
    Es = E_PETG / (1 - 0.4 ** 2); w = min(DT.ROLLER_W, DT.RING_Y[1] - DT.RIM_Y0)
    p = math.sqrt(N * Es / (math.pi * DT.ROLLER_R * w))
    b = float(np.linalg.norm(D["H"] - D["P"])); M = F * b
    t = DT.ARM_Y[1] - DT.ARM_Y[0]; net = 2 * DT.ARM_BOSS_R - 3.2
    sig = M / (t * net ** 2 / 6)
    return dict(hook_N=round(F, 1), roller_N=round(N, 1), hertz_MPa=round(p, 1), hertz_index=round(p / (1.5 * S_IN), 2), contact_w=w,
                arm_M_Nmm=round(M, 0), arm_MPa=round(sig, 1), arm_index=round(sig / S_IN, 2))


WALLS = {}


def search_fixed_q(Q, target_T):
    fixed, turn = DT.obstacles_v5()
    best = None; n = 0
    for th, s, a, br, phi_d, psi in itertools.product(range(-165, -14, 30), (-1, 1), (9, 11, 13, 15, 17), (2.2, 2.4, 2.6),
                                                      range(0, 360, 15), (0, 45, 90, 135, 180, 225, 270, 315)):
        D = DT.design(th, s, a, a * br, th + phi_d, psi)
        if D["ratio"] <= 0 or DT.SPRING_LS + DT.ROLL_LIFT * br > DT.SPRING_LMAX: continue
        _, _, T = holding(D)
        if 2 * T < target_T: continue
        if D["A"] @ D["A"] > 75.0 ** 2: continue
        mid = (D["P"] + D["A"]) / 2
        if np.linalg.norm(np.asarray(Q) - mid) > 40: continue
        st = stresses(D)
        if st["hertz_index"] > 0.5 or st["arm_index"] > 0.5: continue
        if th not in WALLS: WALLS[th] = min(DT.ring_checks(th)["wall_notch_to_bolt"], DT.ring_walls(th)["pocket_to_notch"])
        if WALLS[th] < 2.0: continue                                     # the 2.0 rule on ring v4 (notches vs the 80T bolt pockets)
        n += 1
        g = DT.evaluate(D, fixed, turn)
        if g < DT.CLR - 1e-6: continue
        gt = DT.tower_gap(D, np.asarray(Q), fixed, turn)
        if gt < DT.CLR - 1e-6: continue
        cost = np.linalg.norm(np.asarray(Q) - D["P"]) + np.linalg.norm(np.asarray(Q) - D["A"])
        cand = (round(2 * T, 3), th, s, a, br, phi_d, psi, round(g, 2), round(gt, 2), round(cost, 1), st)
        if best is None or (-st["hertz_index"] - st["arm_index"], -cost) > (-best[-1]["hertz_index"] - best[-1]["arm_index"], -best[-3]): best = cand
        if n > 400 and best is not None: break
    return best, n


if __name__ == "__main__":
    use()
    Q = DT.CHOSEN_V7["Q"]
    target = 2 * 0.390 * SPRING_SCALE                       # the hold swap_system uses (DET_HOLD, x1.3): 1.014 N.m
    c = DT.CHOSEN_V7; D7 = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"])
    out = dict(v6=V6, R_SEAT=DT.R_SEAT, R_APEX=round(DT.R_APEX, 2), target_hold_Nm=round(target, 3),
               chosen_v7_levers=dict(hold_Nm=round(2 * holding(D7)[2], 3), **stresses(D7)))
    print("CHOSEN_V7 levers on v6 parts:", out["chosen_v7_levers"])
    if "search" in sys.argv:
        best, n = search_fixed_q(Q, target)
        out["search"] = dict(tried=n, best=None if best is None else dict(hold_Nm=best[0], th=best[1], s=best[2], a=best[3], br=best[4], phi_d=best[5],
                                                                          psi=best[6], gap_arm=best[7], gap_tower=best[8], cost=best[9], stresses=best[10]))
        print("search", out["search"])
    json.dump(out, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=str)
