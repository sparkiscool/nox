"""Detent v6 for the swap arm v7 (24 Oct 2026, swap arm system worker; the piece pass after the v7 concept check, springs x1.3 at 5:1):
the round 3d recheck found the 683ZZ (Ø7 x 3) roller on ring v3's 2.1 rim at ~54 MPa Hertz (index ~0.8, every swap rolls it) and arm v2 at
0.68 at its pivot. v6: a 623ZZ roller (Ø10 x 4, bore 3: the 3.0 carbon axle as v2), ring v4 (r_out 26.5: the notch walls to the 80T bolt pockets 2.24 / 2.34 / rule 2.0; rim Y 65.9..69.9 = the roller's
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

V6 = dict(ROLLER_R=5.0, ROLLER_W=4.0, ROLLER_BORE=3.0, RING_R_OUT=26.5, RIM_Y0=65.9, ROLLER_Y=(65.9, 69.9), ARM_Y=(70.9, 74.4), ARM_BOSS_R=6.0)
SPRING_SCALE = S7.DETENT_SPRING_SCALE      # x1.3 (arm_v7_layout)
E_PETG, S_IN = 2000.0, 45.0


def use():
    DT.V6_PARTS.update(V6); DT.use_v6()                                        # the parts live in detent.py (V6_PARTS); V6 here only for a what-if


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


def search_free(target_T, max_tried=120):
    """as search_fixed_q, the tower Q searched too (r 32..72 every 4, -180..0 every 7.5, within 40 of the pivot-anchor midpoint): the
    shortest webs over the lever sets that clear the arm parts by CLR"""
    fixed, turn = DT.obstacles_v5()
    best = None; n = 0
    for th, s, a, br, phi_d, psi in itertools.product(range(-165, -14, 30), (-1, 1), (9, 11, 13, 15, 17), (2.2, 2.4, 2.6),
                                                      range(0, 360, 15), (0, 45, 90, 135, 180, 225, 270, 315)):
        D = DT.design(th, s, a, a * br, th + phi_d, psi)
        if D["ratio"] <= 0 or DT.SPRING_LS + DT.ROLL_LIFT * br > DT.SPRING_LMAX: continue
        _, _, T = holding(D)
        if 2 * T < target_T or D["A"] @ D["A"] > 75.0 ** 2: continue
        st = stresses(D)
        if st["hertz_index"] > 0.5 or st["arm_index"] > 0.5: continue
        if th not in WALLS: WALLS[th] = min(DT.ring_checks(th)["wall_notch_to_bolt"], DT.ring_walls(th)["pocket_to_notch"])
        if WALLS[th] < 2.0: continue
        g = DT.evaluate(D, fixed, turn)
        if g < DT.CLR - 1e-6: continue
        n += 1
        mid = (D["P"] + D["A"]) / 2
        for rq in np.arange(32.0, 72.0, 4.0):
            for aq in np.arange(-180.0, 0.1, 7.5):
                Q = rq * np.array([math.cos(math.radians(aq)), math.sin(math.radians(aq))])
                if np.linalg.norm(Q - mid) > 40: continue
                cost = np.linalg.norm(Q - D["P"]) + np.linalg.norm(Q - D["A"])
                if best is not None and cost >= best[9]: continue
                gt = DT.tower_gap(D, Q, fixed, turn)
                if gt < DT.CLR - 1e-6: continue
                best = (round(2 * T, 3), th, s, a, br, phi_d, psi, round(g, 2), round(gt, 2), round(cost, 1), st, [round(float(Q[0]), 2), round(float(Q[1]), 2)])
        if n >= max_tried and best is not None: break
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
    if "free" in sys.argv:
        best, n = search_free(target)
        out["search_free"] = dict(tried=n, best=None if best is None else dict(hold_Nm=best[0], th=best[1], s=best[2], a=best[3], br=best[4], phi_d=best[5],
                                                                               psi=best[6], gap_arm=best[7], gap_tower=best[8], cost=best[9], stresses=best[10], Q=best[11]))
        print("search_free", out["search_free"])
    if "export" in sys.argv:                                                  # ring v4 and arm v3 at CHOSEN_V8: W meshes, print poses, solid_check
        import trimesh, arm_cad as A
        from solid_check import solid_check
        c = DT.CHOSEN_V8; D = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); S = DT.arm_solids(D, np.asarray(c["Q"]))
        rg = DT.ring_v3(c["th"])
        fixed, turn = DT.obstacles_v5()
        out["chosen_v8"] = dict(hold_Nm=round(2 * holding(D)[2], 3), **stresses(D), gap_arm=round(DT.evaluate(D, fixed, turn), 2),
                                gap_tower=round(DT.tower_gap(D, np.asarray(c["Q"]), fixed, turn), 2), ring=DT.ring_checks(c["th"]), walls=DT.ring_walls(c["th"]))
        for nm, m, rot in (("box_detent_ring_v4", rg, 90), ("box_detent_arm_v3", S["arm"], -90)):
            A.tm(m).export(os.path.join(RUN, nm + "_W.stl"))
            t = A.tm(m); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot), [1, 0, 0])); t.apply_translation(-t.bounds[0])
            path = os.path.join(RUN, nm + ".stl"); t.export(path)
            ok, info = solid_check(trimesh.load(path, force="mesh"), pitch=0.1)
            out[nm] = dict(file=os.path.relpath(path, ROOT), mass_g=round(t.volume / 1000 * 1.27, 2), extents=np.round(t.extents, 1).tolist(),
                           solid_check="PASS" if ok else "FAIL", solid_info={k: info[k] for k in ("bodies_eroded", "necks", "thin_walls") if k in info})
        A.tm(S["tower"] + S["anchor"]).export(os.path.join(RUN, "detent_tower_v6_W.stl"))
        print({k: out[k] for k in ("chosen_v8", "box_detent_ring_v4", "box_detent_arm_v3")})
    json.dump(out, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=str)
