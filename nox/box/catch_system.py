"""Catch system (24 Oct 2026, CLAUDE.md "System first, then cut"): the seated drone, its 8 catch wheels, the collar
(funnel panels), the 4 hook latches and the cartridge push, modelled as one assembly. The hook latch pieces (housing,
plunger) are cut from it: their loads are the joint reactions solved here, not guessed on one piece.

System (seat frame, funnel_panel.py convention; C unless tagged):
  drone       rigid body, weight 4.9 N (Scout ~500 g, dimensions C, E), the catch-interface wheels at (+-100, +-112) and
              (+-112, +-100), rims on the 45 deg faces (hook_latch.py drone model)
  latch       4 plungers at the +-y-face wheels: each takes load only along the slope (a contact on its downhill face)
  collar      4 faces; a wheel on a face pushes only along the face normal (rolling: no friction along the slope)
  push        the XT60 insert, 60 N up (E), anywhere on the cartridge footprint (x -41.5..37.5, y +-19, archive v3 E:
              the Scout v3 bottom layout fixes the point; the worst over the footprint is used)
  probes      4 keep-alive spring probes in the +-x faces (panel v9 plain, Spark 24 Oct), <= 1.0 N each along the face
              normal on the drone's pads: added to every case
Cases:   per drone (Scout pack in 4.9 N / out 3.3 N; Overseer pack in / out at two CG sets, 25 / 36.3 and 29.7 / 41.0):
         seat and catch 6 g with the latch in (faces only: every wheel must stay loaded; pack out with the latch in only
         happens after an abort, reported apart), pull 50 N down (pack in) or insert 60 N up and the seat with the latch
         out (pack out), head stall 74 N up (every insert ends on stall detection: swap class), hand lift 100 N
         at a cage corner (misuse) (dimensions G, L Swap, E). The receiver pitch couple is not a case (the XT60 float takes it).
Solve:   rigid drone on unilateral supports (4 plungers + 8 face normals), equal support stiffness (DEFAULT): least-norm
         non-negative reactions that balance force and moment (scipy nnls on the 6 equilibrium rows).
Load path checked (each joint's reaction from the solve): wheel -> plunger -> housing guide -> 4 M3 into the panel v7
bosses -> panel skin and ribs -> panel seat blocks -> end-frame pads. Pieces: plunger and guide (hook_latch.py
strength(), hand calc), bosses and panel (funnel panel v7's as-built FE and boss hand calc under these reactions,
runs/funnel_panel/v7/result.json), assembled worst index = max over every piece per case.
python3 nox/box/catch_system.py -> runs/box_catch/v9_plain/system.json + system_table.md (v8 plain and before:
runs/hook_latch/v2/system.json)"""
import json, math, os, sys
import numpy as np
from scipy.optimize import nnls

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import hook_latch as H

S2 = math.sqrt(2.0)
OUT = os.path.join(ROOT, "runs", "box_catch", "v9_plain")       # catch system with panels v9 plain (v8 plain: runs/hook_latch/v2/system.json)
W_DRONE = 4.9                                       # N (E), Scout with its pack
W_SCOUT_OUT = W_DRONE - 0.165 * 9.81                # N (E): pack 165 g out (dimensions A, P)
PUSH_IN, PULL_OUT, HAND, CATCH_G = 60.0, 50.0, 100.0, 6.0     # N, N, N, g (E)
STALL = 74.0                                        # N (E): head stall; every insert ends on stall detection at 2 A, so the seat sees
                                                    # 60..74 N (dimensions System box swap: arm "Insert stroke", L Swap, arm PR 79): swap class
FOOT = [(-41.5, -19.0), (-41.5, 19.0), (37.5, -19.0), (37.5, 19.0), (0.0, 0.0)]   # cartridge footprint corners + centre (E)
# keep-alive probes (funnel panel v9 plain, Spark 24 Oct): 2 per plain face, tips at seat (+-115.7, +-12 / +-22, 33), each
# pushing the drone's pad along the face normal into the mouth (up and in), <= 1.0 N each (C on the E spring: 0.59..0.86 N
# over the pad's 7.0..1.0 off the face): added to every case (the drone is seated in all of them)
F_PROBE = 1.0
PROBES = [(np.array([115.7, y, 33.0]), np.array([-1.0, 0.0, 1.0]) / S2) for y in (12.0, 22.0)] + \
         [(np.array([-115.7, -y, 33.0]), np.array([1.0, 0.0, 1.0]) / S2) for y in (12.0, 22.0)]


def supports():
    """(point, unit direction the support can push the drone) for the 8 face wheels and the 4 plungers."""
    R = max(H.R_WHEEL); zc = H.ZB + H.T_C / S2 + R / S2
    S = []
    for k in range(4):                               # faces +y, -x, -y, +x (rotate the +y face by 90 k)
        c, s = math.cos(math.radians(90 * k)), math.sin(math.radians(90 * k))
        rot = lambda v: np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
        for xs in (1, -1):
            p = rot(np.array([xs * H.X_WHEEL, H.K0 + H.ZB + H.T_C / S2 - R / S2, zc]))
            n_face = rot(np.array([0.0, -1.0, 1.0]) / S2)          # face pushes the wheel into the mouth (up + inward)
            S.append(("face", k, xs, p, n_face))
            if k in (0, 2):                                       # the latched faces (drone +-y)
                d_pl = rot(np.array([0.0, -1.0, -1.0]) / S2)      # plunger pushes the wheel down the slope
                S.append(("plunger", k, xs, p, d_pl))
    return S


def solve(F, P, W=None, cg=None, kinds=("face", "plunger"), probes=True):
    """least-norm non-negative support reactions for an external force F at point P on the drone (+ its weight W at cg,
    + the 4 keep-alive probes at F_PROBE each when probes)."""
    S = [s for s in supports() if s[0] in kinds]
    A = np.zeros((6, len(S)))
    for j, (_, _, _, p, d) in enumerate(S):
        A[:3, j] = d; A[3:, j] = np.cross(p, d)
    G = np.array([0, 0, -(W_DRONE if W is None else W)]); cg = np.array([0.0, 0.0, 20.0] if cg is None else cg, float)
    Fx, Mx = F + G, np.cross(P, F) + np.cross(cg, G)
    if probes:
        for pp, d in PROBES:
            Fx = Fx + F_PROBE * d; Mx = Mx + np.cross(pp, F_PROBE * d)
    b = -np.r_[Fx, Mx]
    # nnls minimises |A r - b|; add a small ridge so the indeterminate set takes the least-norm split (equal stiffness)
    lam = 1e-3
    Aa = np.vstack([A, lam * np.eye(len(S))]); ba = np.r_[b, np.zeros(len(S))]
    r, _ = nnls(Aa, ba)
    res = np.linalg.norm(A @ r - b)
    return S, r, res


def _row(S, r, res, p):
    pl = [r[j] for j, s_ in enumerate(S) if s_[0] == "plunger"] or [0.0]
    fc = [r[j] for j, s_ in enumerate(S) if s_[0] == "face"]
    f_lat = max(r[j] for j, s_ in enumerate(S) if s_[0] == "face" and s_[1] in (0, 2))   # +-y faces: panels v7
    f_pln = max(r[j] for j, s_ in enumerate(S) if s_[0] == "face" and s_[1] in (1, 3))   # +-x faces: v9 plain
    return dict(at=[round(v, 1) for v in p], plunger_max_N=round(max(pl), 2), plungers_N=[round(v, 2) for v in pl],
                face_wheel_max_N=round(max(fc), 2), face_wheel_min_N=round(min(fc), 2),
                face_wheel_v7_N=round(f_lat, 2), face_wheel_plain_N=round(f_pln, 2),
                wheels_loaded=int(sum(v > 0.05 for v in fc)), residual_N=round(float(res), 3))


def run_set(W, cg, foot, hand_pts, tag):
    """one drone at one CG: landing with the latch in (faces only), and the latched swap and handling cases, every one
    with the probes; worst point per case (residual, then plunger, then face wheel)."""
    zp = H.ZB - 42.4                                  # cartridge bottom (E): the push acts on the drone through it
    runs = [("seat (latch in)", np.zeros(3), [cg], ("face",)),
            ("catch 6 g (latch in)", np.array([0, 0, -CATCH_G * W]), [cg], ("face",)),
            ("hand lift 100 N", np.array([0, 0, HAND]), hand_pts, ("face", "plunger"))]
    runs += ([("pull 50 N", np.array([0, 0, -PULL_OUT]), [(x, y, zp) for x, y in foot], ("face", "plunger"))] if tag == "pack in"
             else [("insert 60 N", np.array([0, 0, PUSH_IN]), [(x, y, zp) for x, y in foot], ("face", "plunger"))])
    runs += [("stall 74 N", np.array([0, 0, STALL]), [(x, y, zp) for x, y in foot], ("face", "plunger"))]
    if tag == "pack out":                             # the operating pack-out state: the latch is out from the seat until the
        runs += [("seat (latch out)", np.zeros(3), [cg], ("face", "plunger"))]   # new pack is in (pack out + latch in: abort only)
    out = {}
    for nm, F, pts, kinds in runs:
        worst = None
        for p in pts:
            S, r, res = solve(F, np.array(p, float), W=W, cg=cg, kinds=kinds)
            row = _row(S, r, res, p)
            if worst is None or (row["residual_N"], row["plunger_max_N"], row["face_wheel_max_N"]) > \
               (worst["residual_N"], worst["plunger_max_N"], worst["face_wheel_max_N"]):
                worst = row
        out[f"{tag}: {nm}"] = worst
    return out


def scout_cases():
    """the Scout (4.9 N with its pack, 3.3 N without, CG (0, 0, 20), E) on the catch interface points."""
    out = {}
    for tag, W in (("pack in", W_DRONE), ("pack out", W_SCOUT_OUT)):
        out.update(run_set(W, (0.0, 0.0, 20.0), FOOT, [(112, 100, 50), (-112, 100, 50), (0, 112, 50)], tag))
    return out


# Overseer in the catch (hub 24 Oct: Spark chose layout B, pack 30 behind the thrust axis, CG over the feet, ~11 off
# with the pack out, accepted). Numbers from the Overseer system run v1B (runs/overseer_system/v1B/checks.txt, branch of
# the overseer system chat, E): drone x = catch x (latched faces are drone +-y), catch centre 30 behind the thrust centre,
# CG 0.3 behind the thrust centre, y +1.5, 13.6 over the bottom plate top at seat z 78.3; 2850 g (system_check, E).
# Pack out: 41.0 (E). Second set (hub 24 Oct, box catch v9 brief): CG 25 / 36.3 ahead of the catch centre (E); the v1B
# set 29.7 / 41.0 is kept beside it.
OV_M_IN, OV_M_OUT = 2.850, 2.065                              # kg (E)
OV_CG = {"CG 25 / 36.3": ((25.0, 1.5, 91.9), (36.3, 1.5, 91.9)),
         "CG 29.7 / 41.0": ((29.7, 1.5, 91.9), (41.0, 1.5, 91.9))}   # catch frame (E), pack in / pack out
OV_CG_IN, OV_CG_OUT = OV_CG["CG 29.7 / 41.0"]
OV_FOOT = [(78.5, 27.0), (78.5, -27.0), (-78.5, 27.0), (-78.5, -27.0), (0.0, 0.0)]   # cartridge 157 x 54 on the catch centre (E, G)
OV_HAND = [(397.0, 0.0, 120.0), (-236.0, 0.0, 100.0), (0.0, 266.0, 100.0), (300.0, 200.0, 100.0), (-200.0, 200.0, 100.0)]
                                                              # nose, rear prop tips, side, guard corners (v1B extents, E)


def overseer_cases():
    g = 9.81
    out = {}
    for cgn, (cg_in, cg_out) in OV_CG.items():
        for tag, m, cg in (("pack in", OV_M_IN, cg_in), ("pack out", OV_M_OUT, cg_out)):
            for k, v in run_set(m * g, cg, OV_FOOT, OV_HAND, tag).items():
                out[f"{cgn}, {k}"] = v
    return out


def piece_indices(F_pl):
    """latch pieces at one plunger reaction (along the slope), hand calc as hook_latch.strength() (flight / misuse)."""
    lever = H.H_LOAD - H.GUIDE_TOP
    lay = H.servo_layout(13.0, 1, -1); P = H.build(lay)
    eng = H.GUIDE_TOP - P["g_bot"]
    M = F_pl * lever
    s_pl = 6 * M / (H.PL_V * H.PL_U ** 2)
    R_top = F_pl * (lever + eng) / eng
    bw = 2 * P["bore"][1] + H.WALL_VN + H.WALL_VP
    s_w = 6 * R_top * (H.GUIDE_TOP - H.W_FL0) / (bw * H.WALL_U ** 2)
    return s_pl, s_w, R_top


def panel_refs():
    """panel v7 latched FE + boss hand calc at the system loads (runs/funnel_panel/v7/result.json), and the 20 N wheel
    spot presses: v7 (u100 v40), v9 plain (worst of its 4 files at the seated wheel u +-100 v43.6), C."""
    base = os.path.join(ROOT, "runs", "funnel_panel")
    R7 = json.load(open(os.path.join(base, "v7", "result.json")))
    fe = R7["fe_asbuilt_system_loads"]; bh = {r["case"]: r for r in R7["boss_hand_calc"]}
    sk = {"v7": R7["fe_asbuilt"]["per_case"]["press u100 v40"]}
    R9 = json.load(open(os.path.join(base, "v9_plain", "result.json")))
    spots = [(f"{f} {h}", r["wheel_spot_per_20N"]) for f, v in R9["fe"].items() for h, r in v["halves"].items()]
    worst = max(spots, key=lambda t: t[1][1])
    sk["v9 plain"] = worst[1]
    return fe, bh, sk, worst[0]


def assess(rows, fe, bh, sk):
    """per case: latch pieces at the worst plunger (hand calc), the v7 panel FE and bosses scaled linearly from the
    nearest evaluated system load, the panel skins at the seated wheel spot (v7, v9 plain); worst over the pieces."""
    for nm, row in rows.items():
        cls = "misuse" if "hand" in nm else "flight"
        row["cls"] = cls
        row["piece_worst"] = 0.0
        if row["plunger_max_N"] > 0:
            s_pl, s_w, R_top = piece_indices(row["plunger_max_N"])
            allow = H.XT / H.SF[cls]
            row.update(plunger_index=round(s_pl / allow, 3), guide_wall_index=round(s_w / allow, 3),
                       guide_top_N=round(R_top, 1), boss_bolt_shear_N=round(row["plunger_max_N"] / 4, 2))
            ref, F0 = (("hand lift 85.8 N (system)", "hand lift 85.8 N (system, C)"), 85.81) if cls == "misuse" else \
                      (("insert 28.3 N x 2 (system)", "insert 28.3 N (system, C)"), 28.28)
            k = row["plunger_max_N"] / F0
            row.update(panel_fe_util=round(fe[ref[0]]["util"] * k, 3), panel_boss_index=round(bh[ref[1]]["index_across_layers"] * k, 3),
                       panel_boss_defl_mm=round(fe[ref[0]]["boss_defl_mm"] * k, 2))
            row["piece_worst"] = max(row["plunger_index"], row["guide_wall_index"], row["panel_fe_util"], row["panel_boss_index"])
        # allowable: the panel's own design rule, 19.5 MPa in-layer (PETG XY 39 / SF 2, box-funnel.md Loads), every case
        row["panel_wheel_spot"] = {}
        for nm_p, key in (("v7", "face_wheel_v7_N"), ("v9 plain", "face_wheel_plain_N")):
            F = row[key]; st = sk[nm_p][1] * F / 20.0
            row["panel_wheel_spot"][nm_p] = dict(load_N=F, stress_MPa=round(st, 1), index=round(st / 19.5, 3),
                                                 defl_mm=round(sk[nm_p][0] * F / 20.0, 2), cls=cls)
        row["piece_worst"] = round(max([row["piece_worst"]] + [v["index"] for v in row["panel_wheel_spot"].values()]), 3)
        row["seated"] = bool(row["residual_N"] < 0.01)
        row["operating"] = not ("pack out" in nm and "latch in" in nm)   # pack out with the latch in happens only after an abort
        if "latch in" in nm: row["landing_ok"] = bool(row["wheels_loaded"] == 8 and row["residual_N"] < 0.01)
    return rows


def main():
    fe, bh, sk, spot_file = panel_refs()
    sc, ov = assess(scout_cases(), fe, bh, sk), assess(overseer_cases(), fe, bh, sk)
    out = dict(system="catch: seated drone + 8 wheels + collar (v7 on +-y, v9 plain on +-x) + 4 hook latches + 4 keep-alive "
                      "probes + cartridge push",
               probes=dict(F_N=F_PROBE, points_dirs=[[p.tolist(), d.round(4).tolist()] for p, d in PROBES],
                           note="added to every case (the drone is seated in all of them)"),
               wheel_spot_v9_plain_file=spot_file,
               scout=dict(weights_N=dict(pack_in=W_DRONE, pack_out=round(W_SCOUT_OUT, 2)), cases=sc),
               reference_not_added=("receiver pitch couple (dropped: the XT60 float takes it); the box catch session's reference "
                                    "run read plunger 55.5 N, index 0.45, Overseer pack out; not in these cases"))
    out["scout"]["assembled_worst"] = max(((nm, r["piece_worst"]) for nm, r in sc.items()), key=lambda t: t[1])
    out["scout"]["landing_all_8_wheels_loaded"] = all(r["landing_ok"] for nm, r in sc.items() if "latch in" in nm and r["operating"])
    out["scout"]["all_seated"] = all(r["seated"] for r in sc.values() if r["operating"])
    out["scout"]["not_operating"] = {nm: dict(residual_N=r["residual_N"], wheels_loaded=r["wheels_loaded"]) for nm, r in sc.items() if not r["operating"]}
    # compact (CLAUDE.md "Compact by default"): one latch unit's bounding volume and largest extent (housing + servo +
    # plunger out), and the catch assembly (collar + 4 latches) bounding box (C)
    lay = H.servo_layout(13.0, 1, -1); P = H.build(lay)
    unit = P["housing"] + P["servo"] + P["plunger"].translate([0, 0, H.RISE])
    b = np.array(unit.bounding_box()).reshape(2, 3); ext = b[1] - b[0]
    out["compact"] = dict(latch_unit_bbox_mm=[round(float(v), 1) for v in ext], latch_unit_bbox_cm3=round(float(np.prod(ext)) / 1000, 1),
                          latch_unit_largest_extent_mm=round(float(ext.max()), 1),
                          latch_unit_solid_cm3=round(float(unit.volume()) / 1000, 1))
    # cage first (CLAUDE.md, Spark 24 Oct): the collar and the plungers meet only cage members (the catch wheels / nodes);
    # everything else of the drone >= 3 mm off them (hook_latch.py catch-interface drone)
    D = H.drone_parts()
    cl = {}
    for k in ("cage", "plate", "props", "receiver"):
        t = H.to_tm(D[k]); v = t.vertices
        dist = np.min([(np.abs(v[:, i]) * 0 + (H.K0 + v[:, 2] - np.abs(v[:, i])) / S2) for i in (0, 1)], axis=0)  # to the nearer face plane
        cl[k] = round(float(dist.min()), 2)
    out["cage_first_face_clearance_mm"] = cl
    out["cage_first_note"] = ("the node cheeks (part of the cage) sit 1.0 off the faces by the catch interface (G (1)); the rings, "
                              "plate, props and receiver are the 'everything else' and must be >= 3 mm (Scout v3 / Overseer chats own them); "
                              "the keep-alive pads on the lower ring are a designed contact (G (6), (8))")
    sys.path.insert(0, os.path.dirname(HERE))
    import assembly as AS
    a = AS.load(os.path.join(HERE, "box_catch.assembly.json"))
    r_out = a.at(**{n: 1.0 for n in a.names if n.startswith("latch plunger")}).check(); r_in = a.check()
    out["system_check"] = dict(pass_out=bool(r_out["pass"]), pass_in=bool(r_in["pass"]), summary=r_out["summary"],
                               fit_fail_pairs=[(q["a"], q["b"], q["overlap_mm3"]) for r in (r_out, r_in) for q in r["fit"]["pairs"] if not q["pass_"]],
                               compact=r_out["compact"])
    out["overseer"] = dict(source="runs/overseer_system/v1B/checks.txt (overseer system chat, layout B, E) + hub CG 25 / 36.3 (E)",
                           mass_kg=dict(pack_in=OV_M_IN, pack_out=OV_M_OUT),
                           cg_catch_frame={k: dict(pack_in=v[0], pack_out=v[1]) for k, v in OV_CG.items()},
                           cases=ov, assembled_worst=max(((nm, r["piece_worst"]) for nm, r in ov.items()), key=lambda t: t[1]),
                           landing_all_8_wheels_loaded=all(r["landing_ok"] for nm, r in ov.items() if "latch in" in nm and r["operating"]),
                           all_seated=all(r["seated"] for r in ov.values() if r["operating"]),
                           not_operating_all_seated=all(r["seated"] for r in ov.values() if not r["operating"]))
    out["assembled_worst"] = max([out["scout"]["assembled_worst"], out["overseer"]["assembled_worst"]], key=lambda t: t[1])
    os.makedirs(OUT, exist_ok=True)
    json.dump(out, open(os.path.join(OUT, "system.json"), "w"), indent=1)
    table(out)
    return out


def table(out):
    """the per-case table (markdown) -> OUT/system_table.md and stdout."""
    L = ["| Drone, case | Plunger max N | Plunger / guide index (class) | Panel v7 FE / bosses | Face wheel v7 / plain N | Skin v7 / v9 plain index | Wheels loaded | Residual N | Worst |",
         "|---|---|---|---|---|---|---|---|---|"]
    for drone, cases in (("Scout", out["scout"]["cases"]), ("Overseer", out["overseer"]["cases"])):
        for nm, r in cases.items():
            ws = r["panel_wheel_spot"]
            L.append(f"| {drone} {nm} | {r['plunger_max_N']} | {r.get('plunger_index', 0)} / {r.get('guide_wall_index', 0)} ({r['cls']}) | "
                     f"{r.get('panel_fe_util', 0)} / {r.get('panel_boss_index', 0)} | {r['face_wheel_v7_N']} / {r['face_wheel_plain_N']} | "
                     f"{ws['v7']['index']} / {ws['v9 plain']['index']} | {r['wheels_loaded']} | {r['residual_N']} | {r['piece_worst']} |")
    s = "\n".join(L)
    open(os.path.join(OUT, "system_table.md"), "w").write(s + "\n")
    print(s)
    print("assembled worst:", out["assembled_worst"], "| Scout seated", out["scout"]["all_seated"], "landing 8 wheels",
          out["scout"]["landing_all_8_wheels_loaded"], "| Overseer seated", out["overseer"]["all_seated"], "landing 8 wheels",
          out["overseer"]["landing_all_8_wheels_loaded"])


if __name__ == "__main__" and "--assembly-v9" not in sys.argv:
    main()


# ---------------------------------------------------------------- the box catch assembly file (nox/ASSEMBLY.md)
# The file is posed in the box W frame of box structure v2 (hub 24 Oct: the arm v7 sweep needs the funnel in the box
# frame); the analytic loads above stay in the seat frame. Every pose in the file = SEAT_TO_W @ its seat pose; change the
# 4 numbers here (box structure v2, C; Y_C may move once more) and rerun update_assembly() + panels_v7_into_assembly().
Y_C, Y_FUN, Z_SEAT, R_RING = 165.0, 165.0, 155.3, 113.0     # grip line (arm PR 79 DEFAULT, hub 24 Oct), funnel centre = the
                                                            # grip line = each drone's catch centre = its latch set's centre
                                                            # (dimensions L box structure v2, main 24 Oct), seat z 0 in W, ring R (C)
SEAT_TO_W = np.array([[0, -1.0, 0, 0], [1.0, 0, 0, Y_FUN], [0, 0, 1.0, Z_SEAT], [0, 0, 0, 1.0]])   # seat y -> W -X, seat x -> W Y
                                                            # (box_system.py convention: the seated faces +-y face W -+X)
FRAME_NOTE = (f"Frame: box W of box structure v2 (Y_C {Y_C:g}, Y_FUN {Y_FUN:g}, Z_SEAT {Z_SEAT:g}, R_RING {R_RING:g}, C; "
              "nox/box/catch_system.py SEAT_TO_W): every pose = SEAT_TO_W @ the part's pose in the seat frame of "
              "funnel_panel.py / hook_latch.py (drone coordinates of a seated drone, contact face max(|x|, |y|) = 82.7 + z, "
              "bottom edge z 1.3); seat y -> W -X, seat x -> W Y, seat z 0 -> W Z_SEAT. Do not apply SEAT_TO_W again. ")


def _set_frame_note(a):
    n = a.data.get("note", "")
    k = n.find("Funnel panels")
    a.data["note"] = "Box catch, 24 Oct. " + FRAME_NOTE + (n[k:] if k >= 0 else "")
def _print_pose(m, R):
    """4x4 that maps a print file (made by to_tm(m, R) then rezeroed) back to the unit frame of m."""
    t = H.to_tm(m, R); off = -t.bounds[0]
    F = np.eye(4); F[:3, 3] = off
    return np.linalg.inv(F @ R)


def update_assembly(path=None):
    """write hook latch v2 (housings A / B, plungers A / B, print files, posed from the unit frames) into
    nox/box/box_catch.assembly.json; panels and drone context stay as the file has them (the panels move to v7 when PR 37
    is on main). Returns the Assembly."""
    sys.path.insert(0, os.path.dirname(HERE))
    import assembly as AS
    path = path or os.path.join(HERE, "box_catch.assembly.json")
    a = AS.load(path)
    lay = H.servo_layout(13.0, 1, -1); P = H.build(lay)
    RA = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])
    RB = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])
    RP = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
    Mv = np.diag([1.0, -1.0, 1.0, 1.0])
    hA, hB = P["housing"], P["housing"].mirror([0, 1, 0])
    pA, pB = P["plunger"], P["plunger"].mirror([0, 1, 0])
    inv = dict(hA=_print_pose(hA, RA), hB=_print_pose(hB, RB), pA=_print_pose(pA, RP), pB=_print_pose(pB, RP))
    for name, fs, xs in H.UNITS:
        T, mir = H.unit_frame(fs, xs)
        k = "B" if mir else "A"
        Th = (T @ Mv if mir else T) @ inv["h" + k]; Tp = (T @ Mv if mir else T) @ inv["p" + k]
        for nm, mesh, pose in ((f"latch housing {name}", f"print/box_hook_latch_housing_{k}_{H.VER}.stl", Th),
                               (f"latch plunger {name}", f"print/box_hook_latch_plunger_{k}_{H.VER}.stl", Tp)):
            p = a.part(nm)
            p["mesh"] = mesh; p["version"] = H.VER; p["source"] = "nox/box/hook_latch.py + nox/box/catch_system.py:update_assembly"
            a.set_pose(nm, SEAT_TO_W @ pose)                  # box W (the plunger moves function works in any frame)
    a.data["note"] = a.data["note"].replace("hook latch v1", "hook latch v2").replace(
        "B-unit plungers are posed mirrored as hook_latch.py does (one plunger file x4: CHECK the magnet pocket side).",
        "B units use the mirrored housing and plunger files (plunger A / B v2: the plunger is chiral).")
    _set_frame_note(a)
    a.save()
    return a


def panels_v7_into_assembly(path=None):
    """the funnel panels in the catch assembly file: v7 on the seated faces (+-y), v8 plain on the others (+-x, bottom
    edge at seat z 12.8, opening 191 along drone x), seat meshes from funnel_panel_v7.build (print files' geometry) in
    runs/box_catch_assembly/v1/. Run in a fresh process (the v8 switch is read when funnel_panel_v7 is imported)."""
    os.environ["NOX_FUNNEL_PLAIN"] = "1"; os.environ["NOX_FUNNEL_ZBOT"] = "12.8"
    sys.path[:0] = [os.path.dirname(HERE), os.path.join(HERE, "catch_v4")]
    import assembly as AS, funnel_panel_v7 as FV
    assert FV.V8, "funnel_panel_v7 was imported before the v8 switch: run in a fresh process"
    root = os.path.dirname(os.path.dirname(HERE)); od = os.path.join(root, "runs", "box_catch_assembly", "v1"); os.makedirs(od, exist_ok=True)
    files = {}
    for plain in (False, True):
        pa, pb, _, _ = FV.build(plain=plain)
        for k, m in (("A", pa), ("B", pb)):
            nm = f"funnel_panel_{k}_{'v8_plain' if plain else 'v7'}_seat.stl"
            FV.tm(FV.to_seat(m)).export(os.path.join(od, nm)); files[(k, plain)] = f"runs/box_catch_assembly/v1/{nm}"
    a = AS.load(path or os.path.join(HERE, "box_catch.assembly.json"))
    for kq, face in enumerate(("+x", "+y", "-x", "-y")):           # seat pose: the k = 0 copy turned 90 k about z
        plain = face in ("+x", "-x")
        c, s_ = [round(v) for v in (math.cos(math.radians(90 * kq)), math.sin(math.radians(90 * kq)))]
        Rz = np.array([[c, -s_, 0, 0], [s_, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        for k in ("A", "B"):
            p = a.part(f"funnel panel {k} {face}")
            p["mesh"] = files[(k, plain)]; p["version"] = "v8 plain" if plain else "v7"
            p["source"] = "nox/box/catch_v4/funnel_panel_v7.py (seat mesh by nox/box/catch_system.py:panels_v7_into_assembly)"
            a.set_pose(f"funnel panel {k} {face}", SEAT_TO_W @ Rz)
    for nm in ("drone wheels (seated)", "drone cage ring (seated)"):     # seat-frame context meshes
        a.set_pose(nm, SEAT_TO_W)
    a.data["note"] = ("Box catch, 24 Oct. " + FRAME_NOTE + "Funnel panels v7 on the seated faces "
                      "(+-y, latch windows and bosses), v8 plain on +-x (bottom edge at seat z 12.8: opening 191 along x by 168 along y). Hook latch v2: B units use the mirrored housing and "
                      "plunger files. Plunger state via 'moves' (0 retracted, 1 out). Drone wheels and ring: hook_latch.drone_parts "
                      "context (E; the wheels sit in their cheeks, a contact pair). Left out: funnel hub + corner blocks (DESIGN), "
                      "MG90S servos and horns (no mass in dimensions.md), the box frame (box structure system), KW11 + pogo post.")
    a.save()
    return a


def probe_pin(s, part):
    """moves: keep-alive probe pin, s = 0 the file pose (seated: pushed 6.0 from free, tip 2.0 off the face on the drone pad),
    s mm further into the panel along the face normal (s -6.0 free, tip 8.0 off; s +1.5 solid, tip 0.5 off). The pin mesh
    is in the seat frame of panel A (+x face), so the push is N_IN there."""
    T0 = np.asarray(part["pose"], float)
    d = np.array([1.0, 0.0, -1.0]) / S2
    T = T0.copy(); T[:3, 3] += T0[:3, :3] @ d * float(s)
    return T


def panels_v9_into_assembly(path=None):
    """the funnel panels v9 plain on +-x (A px / B px on +x, A nx / B nx on -x: probes in A, arm notch on -x), v7 on +-y
    unchanged, the 2 probe caps and 4 probe pins (posed seated: pin pushed 6.0, pad 2.0 off; moves s -6.0 free .. +1.5 solid),
    seat meshes from funnel_panel_v9.py (runs/box_catch_assembly/v2/, listed in runs/funnel_panel/v9_plain/result.json)."""
    sys.path.insert(0, os.path.dirname(HERE))
    import assembly as AS
    R = json.load(open(os.path.join(ROOT, "runs", "funnel_panel", "v9_plain", "result.json")))
    sm = R["seat_meshes"]
    a = AS.load(path or os.path.join(HERE, "box_catch.assembly.json"))
    rz = {"+x": 0, "+y": 1, "-x": 2, "-y": 3}

    def Rz(face):
        c, s_ = [round(v) for v in (math.cos(math.radians(90 * rz[face])), math.sin(math.radians(90 * rz[face])))]
        return np.array([[c, -s_, 0, 0], [s_, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    src = "nox/box/catch_v4/funnel_panel_v9.py (seat mesh; posed by nox/box/catch_system.py:panels_v9_into_assembly)"
    for face, fk in (("+x", "px"), ("-x", "nx")):
        for k in ("A", "B"):
            p = a.part(f"funnel panel {k} {face}")
            p["mesh"] = sm[f"{k}_{fk}"]; p["version"] = "v9 plain"; p["source"] = src
            a.set_pose(f"funnel panel {k} {face}", SEAT_TO_W @ Rz(face))
    names = set(a.names)
    for face, pole in (("+x", "B+"), ("-x", "B-")):
        nm = f"probe cap {face}"
        if nm not in names:
            a.parts.append(dict(name=nm, role="structure", mesh=sm["cap"], material="PETG", tag="M", owner="box catch"))
        p = a.part(nm); p.update(mesh=sm["cap"], version="v9 plain", source=src,
                                 note=f"keep-alive probe cap ({pole}), M2 x 6 self-tapping into the boss lobe")
        a.set_pose(nm, SEAT_TO_W @ Rz(face))
        for u in (12, 22):
            nm = f"probe pin {face} u{u}"
            if nm not in names:
                a.parts.append(dict(name=nm, role="bought", mesh=sm[f"pin{u}"], material="brass", mass_g=2.4, tag="E", owner="box catch"))
            p = a.part(nm); p.update(mesh=sm[f"pin{u}"], version="v9 plain", source=src, mass_g=2.4, tag="E",
                                     moves="nox/box/catch_system.py:probe_pin",
                                     note=f"keep-alive {pole}: brass o3.0 x 36.5 + collar, spring behind it (E, buy); posed seated (pushed 6.0, "
                                          "tip 2.0 off the face on the drone pad); moves s = further push (-6.0 free, +1.5 solid)")
            a.set_pose(nm, SEAT_TO_W @ Rz(face) @ _push(6.0))
    cp = a.data.setdefault("contact_pairs", [])
    for face in ("+x", "-x"):
        for u in (12, 22):
            for other in (f"funnel panel A {face}", f"probe cap {face}"):
                pr = [f"probe pin {face} u{u}", other]
                if pr not in cp: cp.append(pr)
    a.data["note"] = ("Box catch, 24 Oct. " + FRAME_NOTE + "Funnel panels v7 on the seated faces (+-y, latch windows and bosses), "
                      "v9 plain on +-x (bottom edge at seat z 12.8: opening 191 along x by 168 along y; panel A of each carries the 2 "
                      "keep-alive probes (B+ on +x, B- on -x) and a probe cap; the -x pair has the arm notch: r >= 161 about the axle "
                      "over W X -37..49.5). Probe pins slide in their bores and stop on the bore floor and the cap (contact pairs). "
                      "Hook latch v2: B units use the mirrored housing and plunger files. Plunger state via 'moves' (0 retracted, 1 out). "
                      "Drone wheels and ring: hook_latch.drone_parts context (E; the wheels sit in their cheeks, a contact pair; the "
                      "drone's keep-alive pads are not in the context mesh). Left out: funnel hub + corner blocks (DESIGN), MG90S servos "
                      "and horns (no mass in dimensions.md), probe springs and leads, the box frame (box structure system).")
    a.save()
    return a


def _push(s):
    T = np.eye(4); T[:3, 3] = np.array([1.0, 0.0, -1.0]) / S2 * s
    return T


if __name__ == "__main__" and "--assembly-v9" in sys.argv:
    panels_v9_into_assembly()
