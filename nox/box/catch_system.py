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
strength(), hand calc), bosses and panel (funnel panel v7's as-built FE at its latch cases, read from its result.json
when on main), assembled worst index = max over the pieces per case.
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


def solve(F, P):
    """least-norm non-negative support reactions for an external force F at point P on the drone (+ its weight)."""
    S = supports()
    A = np.zeros((6, len(S)))
    for j, (_, _, _, p, d) in enumerate(S):
        A[:3, j] = d; A[3:, j] = np.cross(p, d)
    G = np.array([0, 0, -W_DRONE]); cg = np.array([0.0, 0.0, 20.0])
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
    # panel v7 (funnel worker's as-built plate FE, when its result.json is on main)
    pr = os.path.join(os.path.dirname(os.path.dirname(HERE)), "runs", "funnel_panel", "v7", "result.json")
    if os.path.exists(pr):
        out["panel_v7_result"] = {k: v for k, v in json.load(open(pr)).items() if any(s in k for s in ("fe", "worst", "boss", "index"))}
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
    # nox/system_check.py on the assembled catch (fit, the catch's cage rule as clearances, compact): drone cage = wheels +
    # node cheeks + rings; the rest of the drone, the collar, 4 latch housings + plungers out (seated pose, not lifted)
    sys.path.insert(0, os.path.join(os.path.dirname(HERE)))
    import system_check as SC
    try:
        import funnel_panel_v7 as FV; collar = FV.collar_world(); cname = "collar v7"
    except Exception:
        sys.path.insert(0, os.path.join(HERE, "catch_v4")); import funnel_panel as F6; collar = F6.collar_world(); cname = "collar v6"
    parts = [dict(name="drone wheels", trimesh=H.to_tm(D["wheels"]), role="cage", material="petg", tag="C"),
             dict(name="drone cage", trimesh=H.to_tm(D["cage"]), role="cage", material="carbon", tag="E"),
             dict(name="drone plate", trimesh=H.to_tm(D["plate"]), role="structure", material="carbon", tag="E"),
             dict(name="drone props", trimesh=H.to_tm(D["props"]), role="structure", mass_g=8.0, tag="E"),
             dict(name="drone receiver", trimesh=H.to_tm(D["receiver"]), role="structure", material="petg", tag="E"),
             dict(name="cartridge", trimesh=H.to_tm(H.cartridge()), role="structure", material="petg", tag="E", swap=True),
             dict(name=cname, trimesh=H.to_tm(collar), role="structure", material="petg", tag="M")]
    for name, fs, xs in H.UNITS:
        T, _ = H.unit_frame(fs, xs)
        parts.append(dict(name=f"latch housing {name}", trimesh=H.to_tm(P["housing"], T), role="structure", material="petg", tag="C"))
        parts.append(dict(name=f"latch plunger {name}", trimesh=H.to_tm(P["plunger"].translate([0, 0, H.RISE]), T), role="structure", material="petg", tag="C"))
        parts.append(dict(name=f"latch servo {name}", trimesh=H.to_tm(P["servo"], T), role="bought", mass_g=13.4, tag="E"))
    man = dict(name="box catch (drone seated, latch out)", parts=parts,
               openings=[dict(axis=[0, 0, -1], half_angle_deg=25.0, min_mm=0.0)],
               # the catch rule (CLAUDE.md "Every drone design": the funnel and the hook latch meet only cage members):
               # collar and plungers >= 3 mm from every non-cage drone part; the drone's own cage-first margins are the
               # drone systems' check (Scout v3 / Overseer), so its parts are "structure" here
               contact_pairs=[["drone wheels", cname, 2.0], ["drone wheels", "drone cage", 1e9]],
               clearance_pairs=[[a, b, 3.0] for a in [cname] + [f"latch plunger {u[0]}" for u in H.UNITS]
                                for b in ("drone plate", "drone props", "drone receiver", "cartridge")],
               cage_margin_mm=3.0, fit_tol_mm3=0.5, n_dirs=1000)
    res = SC.run(man)
    out["system_check"] = dict(pass_=bool(res["pass"]), summary=res["summary"],
                               clearances=[(q["a"], q["b"], q["clearance_mm"], q["ok"]) for q in res["fit"]["clearance"]],
                               fit_fail_pairs=[(q["a"], q["b"], q["overlap_mm3"]) for q in res["fit"]["pairs"] if not q["pass_"]])
    out["system_check_full"] = {k: res[k] for k in ("compact", "fit", "cage_first")}
    out["assembled_worst"] = max(((nm, max(v["plunger_index"], v["guide_wall_index"])) for nm, v in out["pieces"].items()), key=lambda t: t[1])
    os.makedirs(H.OUT, exist_ok=True)
    json.dump(out, open(os.path.join(H.OUT, "system.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------- the box catch assembly file (nox/ASSEMBLY.md)
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
            a.set_pose(nm, pose)
    a.data["note"] = a.data["note"].replace("hook latch v1", "hook latch v2").replace(
        "B-unit plungers are posed mirrored as hook_latch.py does (one plunger file x4: CHECK the magnet pocket side).",
        "B units use the mirrored housing and plunger files (plunger A / B v2: the plunger is chiral).")
    a.save()
    return a
