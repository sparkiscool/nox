"""Box swap arm v6 drawn parts (24 Oct 2026, swap arm system worker): the pieces the v6 system pass (swap_system.py) changes.
  - rail A / B v2: rail v1 (arm_v5_cad.rail) on the v6 stack (arm_v6_layout: rail Y 54..73 on the hub plate, r 29..134, one M5 per
    pad at r 40 / 78 from the top), the inner end stepped over the 80T (no carbon tie sockets: TIES False, rejected alternative);
  - pinion 16T v2: 10.5 face (teeth Y 62..72.5; v1 8), boss Y 56.5..60.5, round bore below Y 59.5 (the NEMA's D-flat is only the last 15,
    dimensions M: v1's D-bore from Y 55.5 met the round shaft), grub on the flat at Y 60;
  - detent ring v3 (detent.ring_v3): no lower step, no bolt through it: keyed by three underside pockets on the 80T's M3 heads, epoxied
    on the boss (ring v1 failed solid_check, ring v2 the 2.0 rule: 1.54 / 1.05 walls, M).
V-wheel v1, eccentric bush v1 and detent arm v2 are unchanged (assembled recheck in swap_system / runs/arm/v6).
python3 arm_v6_cad.py [checks] -> print/box_arm_rail_{A,B}_v2.stl, box_arm_pinion16_v2.stl, box_detent_ring_v3.stl, runs/arm_v6_parts/ (W meshes, checks.json)"""
import os, sys, math, json
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_v6_layout as S6
S6.use("v6")
import arm_v5_layout as V, arm_v5_cad as C5, arm_cad as A, detent as DT, arm_layout as L_
from arm_cad import box, cylY, hull, tm

ROOT = os.path.normpath(os.path.join(HERE, "..", "..")); PRINT = os.path.join(ROOT, "print"); RUN = os.path.join(ROOT, "runs", "arm_v6_parts")
os.makedirs(RUN, exist_ok=True)
T = S6.TIE
TIES = False                     # carbon ties under the rails: rejected (with the receiver guiding, 3 % less receiver moment for 4 rods + 8 sockets)


def cyl_between(p0, p1, r, n=32):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); d = p1 - p0; L_ = float(np.linalg.norm(d)); d /= L_
    m = mf.Manifold.cylinder(L_, r, r, n)                                     # along +Z from the origin
    z = np.array([0, 0, 1.0]); ax = np.cross(z, d); s = np.linalg.norm(ax)
    if s > 1e-9:
        ang = math.degrees(math.atan2(s, float(np.dot(z, d)))); ax /= s
        R = trimesh.transformations.rotation_matrix(math.radians(ang), ax)[:3, :3]
        m = m.transform(np.c_[R, np.zeros(3)])
    return m.translate(p0.tolist())


def tie_geom(h="A"):
    """per side: rod (p0 hub end, p1 rail end), its direction; in head h's own frame for the rail (A frame = arm frame of A)"""
    out = []
    for sd in (-1, 1):
        p0, p1 = S6.tie_rod("A", sd)
        d = (np.asarray(p1) - np.asarray(p0)); d = d / np.linalg.norm(d)
        out.append((np.asarray(p0, float), np.asarray(p1, float), d))
    return out


def rail_v2(h="A"):
    """rail v2 in head A's frame (B = A's rail; arm_kinematics mirrors / turns it): rail v1 on the v6 stack + two Ø3.3 x 10 tie sockets
    rising into its bottom face near the outer end (no lug: the rail still prints bottom down)."""
    m = C5.rail(h)
    # inner end stepped: r 29 beside the 80T (Y 54..65, flange r 27.5 to Y 64), r 27 above it so the park stop (r 27..31) sits on the body
    y_step = L_.Y_P80[1] + 1.0
    m += box(-V.RAIL_BODY_X, V.RAIL_BODY_X, y_step, V.RAIL_Y[1], V.RAIL_STOP["r"][0], V.RAIL_R[0] + 0.01)
    ch = V.RAIL_CHAN
    m -= box(ch["x"][0], ch["x"][1], ch["y"][0], ch["y"][1] + 0.01, V.RAIL_STOP["r"][0] - 1.0, V.RAIL_R[0] + 1.0)   # the ribbon tunnel stays open
    if TIES:                                                                    # rejected alternative (swap_system alternatives): off
        for p0, p1, d in tie_geom(h):
            mouth = p1 - T["depth"] * d
            m -= cyl_between(mouth - 2.0 * d, p1 + 0.01 * d, T["bore_d"] / 2, 32)
    return m


def pinion_v2(xc=0.0, zc=0.0):
    """16T m1 v2: v1's involute (arm_v5_cad.involute_gear_cs) on the 10.5 face; boss Ø14 under it, Ø12 neck; round bore below the flat."""
    cs, g = C5.involute_gear_cs(V.PIN_T, V.PIN_M)
    t0, t1 = V.PIN_TEETH_Y; b0, b1 = V.PIN_BOSS_Y
    ef = 0.3
    gear = mf.Manifold.extrude(cs, t1 - t0 - ef).rotate([-90, 0, 0]).translate([0, t0, 0])
    gear += mf.Manifold.extrude(cs.offset(-0.3, mf.JoinType.Round), ef + 0.01).rotate([-90, 0, 0]).translate([0, t1 - ef - 0.01, 0])
    neck_r = V.PIN_X - V.WEB_X[1] - 1.0                                        # 1.0 off the rack web face
    p = gear + cylY(0, 0, b0, b1, V.PIN_BOSS_R, 96) + cylY(0, 0, b1 - 0.01, t0 + 0.01, neck_r, 96)
    r = C5.SHAFT_D / 2 + C5.BORE_CLR / 2; fd = C5.SHAFT_FLAT - C5.SHAFT_D / 2 + C5.BORE_CLR / 2
    yf = S6.PIN_V2["flat_y0"] + 0.5
    p -= cylY(0, 0, b0 - 1, t1 + 1, r, 48) - (cylY(0, 0, yf, t1 + 2, r + 1, 48) - box(-fd, r + 2, yf - 1, t1 + 3, -r - 2, r + 2))
    yg = S6.PIN_V2["flat_y0"] + 1.0
    p -= A.cylX(yg, 0, -V.PIN_BOSS_R - 1, 0, C5.GRUB_TAP_D / 2, 24)             # grub from -X onto the flat (Y 60)
    return p.translate([xc, 0, zc]), g


def pose_rail(m):
    return C5.pose_rail(m)


PARTS = {  # name -> (builder, pose, material, qty, prev mass g)
    "arm_rail_A_v2": (lambda: rail_v2("A"), pose_rail, "PETG", 1, 49.7),
    "arm_rail_B_v2": (lambda: rail_v2("B"), pose_rail, "PETG", 1, 50.0),
    "arm_pinion16_v2": (lambda: pinion_v2()[0], lambda m: C5.pose_axisY(m, top_down=True), "PETG", 2, 3.0),
    "detent_ring_v3": (lambda: DT.ring_v3(DT.CHOSEN_V6["th"]), lambda m: C5.pose_axisY(m, top_down=False), "PETG", 1, 5.0),
}


def roll_check(n=25):
    pl = C5.carriage("A")["printed"]; base = pinion_v2()[0]; p = math.pi * V.PIN_M; gc = V.WEB_U[0] + p / 2
    ov, gmin = 0.0, 9.0
    for k in range(n):
        z0 = 60.0 + k * p / (n - 1); d = (V.PIN_Z - z0) - gc
        pin = base.rotate([0, -math.degrees(d / V.PIN_R), 0]).translate([V.PIN_X, 0, V.PIN_Z]); pp = pl.translate([0, 0, z0])
        ov = max(ov, (pin ^ pp).volume()); gmin = min(gmin, pin.min_gap(pp, 2.0))
    return round(ov, 4), round(gmin, 3)


def fit():
    """rail v2 + carriage proxy over the v6 stroke; the tie rods vs the rail lug and the hub sockets come from swap_system / the sweep."""
    R = rail_v2("A"); out = []
    body = box(-V.RAIL_BODY_X, V.RAIL_BODY_X, *V.RAIL_Y, *V.RAIL_R)
    Cc = C5.carriage("A")
    rods = None
    for p0, p1, d in tie_geom("A"):
        r_ = cyl_between(p0, p1, T["rod_d"] / 2, 24); rods = r_ if rods is None else rods + r_
    for z0 in V.Z0S:
        mv = lambda m: m.translate([0, 0, z0])
        w, st, pl = mv(Cc["wheels"]), mv(Cc["stack"]), mv(Cc["printed"])
        out.append(dict(z0=z0, wheel_rail_overlap_mm3=round((w ^ R).volume(), 3), wheel_body_gap=round(w.min_gap(body, 3.0), 2),
                        stack_rail_gap=round(st.min_gap(R, 3.0), 2), plate_rail_gap=round(pl.min_gap(R, 3.0), 2),
                        carriage_to_rods=round((w + st + pl).min_gap(rods, 6.0), 2)))
    seat = (R ^ rods).volume()
    return out, round(seat, 2)


def main(checks=False):
    from solid_check import solid_check
    import print_orient as PO
    res = {}
    for nm, (fn, pose, mat, qty, prev) in PARTS.items():
        m = fn()
        tm(m).export(os.path.join(RUN, f"{nm}_W.stl"))
        t = pose(m); path = os.path.join(PRINT, f"box_{nm}.stl"); t.export(path)
        g = t.volume / 1000 * 1.27
        r = dict(file=os.path.relpath(path, ROOT), mass_g=round(g, 2), prev_g=prev, qty=qty, extents=np.round(t.extents, 1).tolist(),
                 watertight=bool(t.is_watertight), bodies=len(m.decompose()))
        if checks:
            tt = trimesh.load(path, force="mesh")
            ok, info = solid_check(tt, pitch=0.25 if "rail" in nm else 0.2)
            r["solid_check"] = "PASS" if ok else "FAIL"; r["solid_info"] = {k: info[k] for k in ("bodies_eroded", "necks", "thin_walls") if k in info}
            n = tt.face_normals; zc = tt.triangles_center[:, 2]
            r["bed_mm2"] = round(float(tt.area_faces[(n[:, 2] < -0.99) & (zc < tt.bounds[0][2] + 0.3)].sum()), 0)
            try:
                first, run = PO.island_check(tt); r["first_layer"] = [first[0], round(float(run), 1)]
            except Exception as e:
                r["first_layer"] = str(e)
        res[nm] = r
        print(nm, {k: v for k, v in r.items() if k != "solid_info"}, r.get("solid_info", ""))
    if checks:
        res["ring_walls"] = DT.ring_walls(DT.CHOSEN_V6["th"]); print("ring v3 walls", res["ring_walls"])
        res["roll"] = roll_check(); print("roll pinion v2 on the rack", res["roll"])
        f, seat = fit(); res["fit"] = f; res["rods_in_rail_mm3"] = seat if TIES else None
        for row in f: print("fit", row)
        if TIES: print("rod in the rail lug (should be 0: rods sit in the bores)", seat)
    json.dump(res, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=str)
    return res


if __name__ == "__main__":
    main(checks="checks" in sys.argv)
