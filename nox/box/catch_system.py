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
Cases:   insert 60 N up (swap), pull 50 N down (swap), hand lift 100 N up at a cage corner (misuse), catch 6 g floor
         landing on the faces (dimensions G, E).
Solve:   rigid drone on unilateral supports (4 plungers + 8 face normals), equal support stiffness (DEFAULT): least-norm
         non-negative reactions that balance force and moment (scipy nnls on the 6 equilibrium rows).
Load path checked (each joint's reaction from the solve): wheel -> plunger -> housing guide -> 4 M3 into the panel v7
bosses -> panel skin and ribs -> panel seat blocks -> end-frame pads. Pieces: plunger and guide (hook_latch.py
strength(), hand calc), bosses and panel (funnel panel v7's as-built FE and boss hand calc under these reactions,
runs/funnel_panel/v7/result.json), assembled worst index = max over every piece per case.
python3 nox/box/catch_system.py -> runs/hook_latch/<VER>/system.json"""
import json, math, os, sys
import numpy as np
from scipy.optimize import nnls

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hook_latch as H

S2 = math.sqrt(2.0)
W_DRONE = 4.9                                       # N (E)
PUSH_IN, PULL_OUT, HAND, CATCH_G = 60.0, 50.0, 100.0, 6.0     # N, N, N, g (E)
FOOT = [(-41.5, -19.0), (-41.5, 19.0), (37.5, -19.0), (37.5, 19.0), (0.0, 0.0)]   # cartridge footprint corners + centre (E)


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


def solve(F, P, W=None, cg=None, kinds=("face", "plunger")):
    """least-norm non-negative support reactions for an external force F at point P on the drone (+ its weight W at cg)."""
    S = [s for s in supports() if s[0] in kinds]
    A = np.zeros((6, len(S)))
    for j, (_, _, _, p, d) in enumerate(S):
        A[:3, j] = d; A[3:, j] = np.cross(p, d)
    G = np.array([0, 0, -(W_DRONE if W is None else W)]); cg = np.array([0.0, 0.0, 20.0] if cg is None else cg, float)
    b = -np.r_[F + G, np.cross(P, F) + np.cross(cg, G)]
    # nnls minimises |A r - b|; add a small ridge so the indeterminate set takes the least-norm split (equal stiffness)
    lam = 1e-3
    Aa = np.vstack([A, lam * np.eye(len(S))]); ba = np.r_[b, np.zeros(len(S))]
    r, _ = nnls(Aa, ba)
    res = np.linalg.norm(A @ r - b)
    return S, r, res


def cases():
    zp = H.ZB - 42.4                                 # cartridge bottom (E): the push acts on the drone through it
    out = {}
    for nm, F, pts in (("insert 60 N", np.array([0, 0, PUSH_IN]), [(x, y, zp) for x, y in FOOT]),
                       ("pull 50 N", np.array([0, 0, -PULL_OUT]), [(x, y, zp) for x, y in FOOT]),
                       ("hand lift 100 N at a corner", np.array([0, 0, HAND]), [(112, 100, 50), (-112, 100, 50), (0, 112, 50)]),
                       ("catch 6 g", np.array([0, 0, -CATCH_G * W_DRONE]), [(0, 0, 20)])):
        worst = None
        for p in pts:
            S, r, res = solve(F, np.array(p, float))
            pl = [r[j] for j, s in enumerate(S) if s[0] == "plunger"]
            fc = [r[j] for j, s in enumerate(S) if s[0] == "face"]
            row = dict(at=[round(v, 1) for v in p], plunger_max_N=round(max(pl), 2), plungers_N=[round(v, 2) for v in pl],
                       face_wheel_max_N=round(max(fc), 2), residual_N=round(float(res), 3))
            if worst is None or row["plunger_max_N"] > worst["plunger_max_N"] or \
               (row["plunger_max_N"] == worst["plunger_max_N"] and row["face_wheel_max_N"] > worst["face_wheel_max_N"]):
                worst = row
        out[nm] = worst
    return out


# Overseer in the catch (hub 24 Oct: Spark chose layout B, pack 30 behind the thrust axis, CG over the feet, ~11 off
# with the pack out, accepted). Numbers from the Overseer system run v1B (runs/overseer_system/v1B/checks.txt, branch of
# the overseer system chat, E): drone x = catch x (latched faces are drone +-y), catch centre 30 behind the thrust centre,
# CG 0.3 behind the thrust centre, y +1.5, 13.6 over the bottom plate top at seat z 78.3; 2850 g (system_check, E).
# Pack out: system_check puts the CG 11.3 off the thrust axis without the swap parts (forward, the pack sits behind):
# 30 + 11.3 = 41.3, taken as 41.0 (E; the run's balance line says 9.2, i.e. 39.2: under 1 N on these loads). The swap
# parts (pack at catch x 0) then weigh 2850 x (41.0 - 29.7) / 41.0 = 785 g (E).
OV_M_IN, OV_M_OUT = 2.850, 2.065                              # kg (E)
OV_CG_IN, OV_CG_OUT = (29.7, 1.5, 91.9), (41.0, 1.5, 91.9)    # catch frame (E)
OV_FOOT = [(78.5, 27.0), (78.5, -27.0), (-78.5, 27.0), (-78.5, -27.0), (0.0, 0.0)]   # cartridge 157 x 54 on the catch centre (E, G)
OV_HAND = [(397.0, 0.0, 120.0), (-236.0, 0.0, 100.0), (0.0, 266.0, 100.0), (300.0, 200.0, 100.0), (-200.0, 200.0, 100.0)]
                                                              # nose, rear prop tips, side, guard corners (v1B extents, E)


def overseer_cases():
    """the catch with the Overseer at both CGs: landing on the faces (latch in), and the latched swap and handling cases.
    Pack in: seat, catch 6 g, pull 50 N (the old pack leaves), hand lift. Pack out: seat, insert 60 N (the new pack goes
    in while the drone carries no pack), hand lift."""
    g = 9.81; zp = H.ZB - 42.4
    out = {}
    for tag, m, cg in (("pack in", OV_M_IN, OV_CG_IN), ("pack out", OV_M_OUT, OV_CG_OUT)):
        W = m * g
        runs = [("seat (latch in)", np.zeros(3), [cg], ("face",)),
                ("catch 6 g (latch in)", np.array([0, 0, -CATCH_G * W]), [cg], ("face",)),
                ("hand lift 100 N", np.array([0, 0, HAND]), OV_HAND, ("face", "plunger"))]
        runs += ([("pull 50 N", np.array([0, 0, -PULL_OUT]), [(x, y, zp) for x, y in OV_FOOT], ("face", "plunger"))] if tag == "pack in"
                 else [("insert 60 N", np.array([0, 0, PUSH_IN]), [(x, y, zp) for x, y in OV_FOOT], ("face", "plunger"))])
        for nm, F, pts, kinds in runs:
            worst = None
            for p in pts:
                S, r, res = solve(F, np.array(p, float), W=W, cg=cg, kinds=kinds)
                pl = [r[j] for j, s_ in enumerate(S) if s_[0] == "plunger"] or [0.0]
                fc = [r[j] for j, s_ in enumerate(S) if s_[0] == "face"]
                f_lat = max(r[j] for j, s_ in enumerate(S) if s_[0] == "face" and s_[1] in (0, 2))   # +-y faces: panels v7
                f_pln = max(r[j] for j, s_ in enumerate(S) if s_[0] == "face" and s_[1] in (1, 3))   # +-x faces: v8 plain
                row = dict(at=[round(v, 1) for v in p], plunger_max_N=round(max(pl), 2), plungers_N=[round(v, 2) for v in pl],
                           face_wheel_max_N=round(max(fc), 2), face_wheel_min_N=round(min(fc), 2),
                           face_wheel_v7_N=round(f_lat, 2), face_wheel_plain_N=round(f_pln, 2),
                           wheels_loaded=int(sum(v > 0.05 for v in fc)), residual_N=round(float(res), 3))
                if worst is None or (row["residual_N"], row["plunger_max_N"], row["face_wheel_max_N"]) > \
                   (worst["residual_N"], worst["plunger_max_N"], worst["face_wheel_max_N"]):
                    worst = row
            out[f"{tag}: {nm}"] = worst
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


def main():
    C = cases()
    out = dict(system="catch: seated drone + 8 wheels + collar + 4 hook latches + cartridge push", cases=C, pieces={})
    for nm, row in C.items():
        cls = "misuse" if "hand" in nm else "flight"
        F = row["plunger_max_N"]
        s_pl, s_w, R_top = piece_indices(F)
        allow = H.XT / H.SF[cls]
        out["pieces"][nm] = dict(plunger_N=F, plunger_index=round(s_pl / allow, 3), guide_wall_index=round(s_w / allow, 3),
                                 guide_top_N=round(R_top, 1), boss_bolt_shear_N=round(F / 4, 2),
                                 face_wheel_max_N=row["face_wheel_max_N"], cls=cls)
    # panel v7 pieces under the same joint reactions (funnel panel v7's as-built plate FE and boss hand calc,
    # runs/funnel_panel/v7/result.json, C): the assembled worst is the max over every piece in the load path
    pr = os.path.join(os.path.dirname(os.path.dirname(HERE)), "runs", "funnel_panel", "v7", "result.json")
    R = json.load(open(pr))
    fe = R["fe_asbuilt_system_loads"]; bh = {r["case"]: r for r in R["boss_hand_calc"]}
    panel = {"insert 60 N": (fe["insert 28.3 N x 2 (system)"]["util"], bh["insert 28.3 N (system, C)"]["index_across_layers"]),
             "hand lift 100 N at a corner": (fe["hand lift 85.8 N (system)"]["util"], bh["hand lift 85.8 N (system, C)"]["index_across_layers"])}
    for nm, v in out["pieces"].items():
        # pull and catch put no plunger load on the panel v7 bosses: not evaluated there (n/a, not 0)
        pu, bi = panel.get(nm, ("n/a", "n/a"))
        v["panel_fe_util"] = pu; v["panel_boss_index"] = bi
        v["piece_worst"] = max(x for x in (v["plunger_index"], v["guide_wall_index"], pu, bi) if x != "n/a")
    # compact (CLAUDE.md "Compact by default"): one latch unit's bounding volume and largest extent (housing + servo +
    # plunger out), and the catch assembly (collar + 4 latches) bounding box (C)
    import manifold3d as mf
    lay = H.servo_layout(13.0, 1, -1); P = H.build(lay)
    unit = P["housing"] + P["servo"] + P["plunger"].translate([0, 0, H.RISE])
    b = np.array(unit.bounding_box()).reshape(2, 3); ext = b[1] - b[0]
    out["compact"] = dict(latch_unit_bbox_mm=[round(float(v), 1) for v in ext], latch_unit_bbox_cm3=round(float(np.prod(ext)) / 1000, 1),
                          latch_unit_largest_extent_mm=round(float(ext.max()), 1),
                          latch_unit_solid_cm3=round(float(unit.volume()) / 1000, 1))
    # cage first (CLAUDE.md, Spark 24 Oct): the collar and the plungers meet only cage members (the catch wheels / nodes);
    # everything else of the drone >= 3 mm off them. Here: the catch-interface drone of hook_latch.py (wheels R 5, node
    # cheeks r 4 round the pin, rings, plate, props, receiver), its distance to the contact face planes and the plungers.
    D = H.drone_parts()
    import trimesh
    cl = {}
    for k in ("cage", "plate", "props", "receiver"):
        t = H.to_tm(D[k]); v = t.vertices
        dist = np.min([(np.abs(v[:, i]) * 0 + (H.K0 + v[:, 2] - np.abs(v[:, i])) / S2) for i in (0, 1)], axis=0)  # to the nearer face plane
        cl[k] = round(float(dist.min()), 2)
    out["cage_first_face_clearance_mm"] = cl
    out["cage_first_note"] = ("the node cheeks (part of the cage) sit 1.0 off the faces by the catch interface (G (1)); the rings, "
                              "plate, props and receiver are the 'everything else' and must be >= 3 mm (Scout v3 / Overseer chats own them)")
    # the assembled check comes from the system's assembly file (nox/ASSEMBLY.md): latch out and in
    sys.path.insert(0, os.path.dirname(HERE))
    import assembly as AS
    a = AS.load(os.path.join(HERE, "box_catch.assembly.json"))
    r_out = a.at(**{n: 1.0 for n in a.names if n.startswith("latch plunger")}).check(); r_in = a.check()
    out["system_check"] = dict(pass_out=bool(r_out["pass"]), pass_in=bool(r_in["pass"]), summary=r_out["summary"],
                               fit_fail_pairs=[(q["a"], q["b"], q["overlap_mm3"]) for r in (r_out, r_in) for q in r["fit"]["pairs"] if not q["pass_"]],
                               compact=r_out["compact"])
    out["assembled_worst"] = max(((nm, v["piece_worst"]) for nm, v in out["pieces"].items()), key=lambda t: t[1])
    # the Overseer at both CGs (pack in / out): landing holds without the latch, latched loads on the same pieces
    ov = overseer_cases()
    for nm, row in ov.items():
        if row["plunger_max_N"] > 0:
            cls = "misuse" if "hand" in nm else "flight"
            s_pl, s_w, R_top = piece_indices(row["plunger_max_N"])
            allow = H.XT / H.SF[cls]
            row.update(cls=cls, plunger_index=round(s_pl / allow, 3), guide_wall_index=round(s_w / allow, 3),
                       guide_top_N=round(R_top, 1), boss_bolt_shear_N=round(row["plunger_max_N"] / 4, 2))
            # panel v7 FE and bosses scaled linearly from the nearest evaluated case (linear FE, C)
            ref, F0 = (("hand lift 85.8 N (system)", "hand lift 85.8 N (system, C)"), 85.81) if cls == "misuse" else \
                      (("insert 28.3 N x 2 (system)", "insert 28.3 N (system, C)"), 28.28)
            k = row["plunger_max_N"] / F0
            row.update(panel_fe_util=round(fe[ref[0]]["util"] * k, 3), panel_boss_index=round(bh[ref[1]]["index_across_layers"] * k, 3),
                       panel_boss_defl_mm=round(fe[ref[0]]["boss_defl_mm"] * k, 2))
            row["piece_worst"] = max(row["plunger_index"], row["guide_wall_index"], row["panel_fe_util"], row["panel_boss_index"])
    # panel skins at the seated wheel spot (20 N press, linear; C): the Overseer puts up to 9 x the Scout's face-wheel load
    # on them (6 g landing, hand lift); latched v7 (runs/funnel_panel/v7/result.json, u 100 v 40), v8 plain on +-x
    # (runs/funnel_panel/v8_plain/result.json, at the seated wheel itself, u 100 v 43.6)
    base = os.path.join(os.path.dirname(os.path.dirname(HERE)), "runs", "funnel_panel")
    sk = {}
    for nm_p, fn, case in (("v7", "v7/result.json", "press u100 v40"), ("v8 plain", "v8_plain/result.json", "press u100 v43.6")):
        pc = json.load(open(os.path.join(base, fn)))["fe_asbuilt"]["per_case"][case]
        sk[nm_p] = pc                                                  # [defl / 1.0 mm, MPa] at 20 N
    # allowable: the panel's own design rule, 19.5 MPa in-layer (PETG XY 39 / SF 2, box-funnel.md Loads), every case; for
    # misuse that is conservative (a ceiling). The seated contact is 43.6 up the slope (T_C); the FE patch is at v 40
    # (interpolating toward v 65 gives the plain panel ~0.60 instead of 0.58, v7 a little lower)
    for nm, row in ov.items():
        cls = "misuse" if "hand" in nm else "flight"
        row["panel_wheel_spot"] = {}
        for nm_p, key in (("v7", "face_wheel_v7_N"), ("v8 plain", "face_wheel_plain_N")):
            F = row[key]; st = sk[nm_p][1] * F / 20.0
            row["panel_wheel_spot"][nm_p] = dict(load_N=F, stress_MPa=round(st, 1), index=round(st / 19.5, 3),
                                                 defl_mm=round(sk[nm_p][0] * F / 20.0, 2), cls=cls)
        row["piece_worst"] = max([row.get("piece_worst", 0.0)] + [v["index"] for v in row["panel_wheel_spot"].values()])
    out["overseer"] = dict(source="runs/overseer_system/v1B/checks.txt (overseer system chat, layout B, E)",
                           mass_kg=dict(pack_in=OV_M_IN, pack_out=OV_M_OUT), cg_catch_frame=dict(pack_in=OV_CG_IN, pack_out=OV_CG_OUT),
                           cases=ov,
                           assembled_worst=max(((nm, r["piece_worst"]) for nm, r in ov.items() if "piece_worst" in r), key=lambda t: t[1]),
                           landing_all_8_wheels_loaded=all(r["wheels_loaded"] == 8 and r["residual_N"] < 0.01 for nm, r in ov.items() if "latch in" in nm))
    os.makedirs(H.OUT, exist_ok=True)
    json.dump(out, open(os.path.join(H.OUT, "system.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
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
