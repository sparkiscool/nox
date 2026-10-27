"""Overseer core mounting check coupons v1 (27 Oct, Spark: "make a fast to print mounting check coupons first, in the orientation
we are going to put them in. make it fast to print"; "make sure bolts are accessible through a screwdriver").

Drawn, least material: only the mounting features of the two core pieces (`nox/overseer/core_v1.py`, numbers read from
`runs/overseer_core/v1/result.json`, never retyped) joined by 6 x 3 bars, each printed the way it sits on the drone:

  hub deck coupon  sits on the bottom plate top, plate face down on the bed. 6 feet (Ø3.4 for M3) on the plate holes
                   (±24, ±36.5) and (-72.5, ±31.75); 4 stack posts Ø9 x 5 on 30.5 (Ø4.0 x 5 through: M3 x 4 short heat-set
                   inserts, or M3 nuts under the plate for the check); the Pi deck rails at their real height with the Pi's
                   4 posts (Ø2.2 x 4 for M2.5 self-tapping) on its 58 x 49, the aft legs at x -109 (the sled's tail in the
                   drone, the bed here). Checks: plate holes, stack posts + grommets + ESC footprint, stack height under the
                   top plate, Pi posts and the Pi's clearance to the arm roots and the top plate's rear edge, a straight
                   screwdriver to every screw.
  sled coupon      sits under the bottom plate; printed plate face down on the bed (its top in the drone), so it bolts on
                   flipped over as it prints. 6 pads Ø12 x 6 with Ø4.0 insert holes under the hub deck feet, 2 under the aft
                   legs, 6 under the receiver v2 bolts, Ø9 driver rings under the 8 arm bottom screws, the MTF-02P pocket
                   (0.2 clearance, lens window, cable slot) at its pod's place. Checks: the bolts line up through the plate,
                   the arm screw heads pass the Ø9 rings, the flow sensor fits its pocket.

python3 nox/overseer/core_coupons_v1.py      (after core_v1.py; writes print/overseer_core_coupon_*_v1.stl)
"""
import json, os, sys
import numpy as np, manifold3d as mf, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v1", "result.json")))
C = R["coupon"]
OUT = os.path.join(ROOT, "runs", "overseer_core_coupons", "v1"); os.makedirs(OUT, exist_ok=True)
PRINT = os.path.join(ROOT, "print")

BAR_W, BAR_T = 6.0, 4.0            # bars (CLAUDE.md: nothing under 2.0; 4 thick: solid_check fails necks under 3.0)
PAD_D = 12.0                       # pads >= Ø12 with >= 2.0 web (CLAUDE.md)
M3_CLR, M3_INS, M25_TAP = 3.4, 4.0, 2.2
FLOW = (21.6, 16.0, 6.5)           # MTF-02P (spec)
FLOW_CLR, WALL = 0.2, 2.0


def cyl(x, y, z0, z1, r, n=40):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1))
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def bar(p, q, z0=0.0, z1=BAR_T, w=BAR_W):
    p, q = np.asarray(p, float), np.asarray(q, float); d = q - p; L = float(np.linalg.norm(d))
    a = float(np.degrees(np.arctan2(d[1], d[0])))
    return mf.Manifold.cube([L, w, z1 - z0]).translate([0, -w / 2, z0]).rotate([0, 0, a]).translate([p[0], p[1], 0])


def union(ms):
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add)


def save(m, name):
    mm = m.to_mesh(); t = trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts))
    t.apply_translation([-t.bounds[0][0], -t.bounds[0][1], -t.bounds[0][2]])
    path = os.path.join(PRINT, name); t.export(path)
    return path, t


# ---------------------------------------------------------------- hub deck coupon (drone frame, z 0 = bottom plate top = bed)
def hub_deck_coupon():
    T = C["floor_t"]; sp = C["stack_p"]; ry = C["rail_y"]
    feet = [(x, y) for x, y in R["feet_xy"]]
    posts = [(sx * sp, sy * sp) for sx in (1, -1) for sy in (1, -1)]
    body = [cyl(x, y, 0, T, PAD_D / 2) for x, y in feet]
    body += [cyl(x, y, 0, C["post_h"], C["post_d"] / 2) for x, y in posts]
    body += [bar((sx * 24, sy * 36.5), (sx * sp, sy * sp)) for sx in (1, -1) for sy in (1, -1)]
    body += [bar((24, 36.5), (24, -36.5)), bar((-24, 36.5), (-24, -36.5))]
    body += [bar((sp, sy * sp), (-sp, sy * sp)) for sy in (1, -1)]
    body += [bar((-24, sy * ry), (-95, sy * ry)) for sy in (1, -1)] + [bar((-72.5, sy * 31.75), (-72.5, sy * ry)) for sy in (1, -1)]
    body += [bar((-95, ry), (-95, -ry))]
    # Pi deck: rails at their height, front legs onto the floor rails, aft legs (sled tail in the drone, bed here), 4 posts
    z0, z1, zp = C["deck_z0"], C["deck_z0"] + C["deck_t"], C["z_pi"]
    x0, x1 = C["pi_x"][0] + 2, C["pi_x"][1] + 1
    for sy in (1, -1):
        body.append(box(x0, x1, sy * 24.5 - 3, sy * 24.5 + 3, z0, z1))                       # rail 6 x 4
        body.append(box(C["front_legs_x"] - 3, C["front_legs_x"] + 3, sy * 24.5 - 3, sy * 24.5 + 3, 0, z0))
        body.append(bar((C["front_legs_x"], sy * 24.5), (C["front_legs_x"], sy * ry)))
        ax_, ay_ = -109.0, sy * 24.5
        body.append(cyl(ax_, ay_, 0, z1, PAD_D / 2))                                          # aft leg Ø12 round its M3
        body.append(box(x0, x0 + 6, sy * 24.5 - 3, sy * 24.5 + 3, 0, z0))                     # coupon only: a leg under the rail's aft end (no supports)
    body += [cyl(x, y, z0, zp, 3.5) for x, y in R["pi_holes"]]
    body.append(bar((x0 + 3, 24.5), (x0 + 3, -24.5)))                                            # ties the two aft legs on the bed
    body.append(bar((-109.0, 24.5), (-109.0, -24.5)))
    body.append(bar((-95, ry), (-109.0, 24.5))); body.append(bar((-95, -ry), (-109.0, -24.5)))
    m = union(body)
    cuts = [cyl(x, y, -1, T + 1, M3_CLR / 2) for x, y in feet]
    cuts += [cyl(x, y, -1, C["post_h"] + 1, M3_INS / 2) for x, y in posts]
    cuts += [cyl(x, y, zp - 4.0, zp + 1, M25_TAP / 2) for x, y in R["pi_holes"]]                # Ø2.2 x 4 self-tapping
    cuts += [cyl(-109.0, sy * 24.5, -1, z1 + 1, M3_CLR / 2) for sy in (1, -1)]                  # aft legs: M3 x 20 into the sled
    return m - union(cuts)


# ---------------------------------------------------------------- sled coupon (drone frame flipped: bed = the sled's top, against the plate)
def sled_coupon():
    T = BAR_T; ST = C["sled_t"]
    bolts = [tuple(f) for f in R["feet_xy"]] + [tuple(a) for a in C["aft_legs"]]
    rec = [tuple(b) for b in R["receiver_bolts"]]
    rings = [tuple(a) for a in R["arm_screws_driver_holes"]]
    body = [cyl(x, -y, 0, ST, PAD_D / 2) for x, y in bolts + rec]                                 # y mirrored: printed flipped about x
    body += [cyl(x, -y, 0, T, 7.5) for x, y in rings]
    # frame: two long bars at y ±27.8 (the receiver bolt lines) + cross bars through every pad and ring
    for sy in (1, -1):
        body.append(bar((-112, sy * 27.8), (58, sy * 27.8)))
    for x, y in bolts + rings:
        body.append(bar((x, -y), (x, -np.sign(y) * 27.8)))
    body.append(bar((-109, 24.5), (-109, -24.5)))
    # flow pocket at its pod's place, cavity open at the bed (the sensor goes in from the pod's inside), lens window on top
    fx, fy = C["flow"][0], -C["flow"][1]
    cx, cy, cz = FLOW[0] + 2 * FLOW_CLR, FLOW[1] + 2 * FLOW_CLR, FLOW[2]
    pocket = box(fx - cx / 2 - WALL, fx + cx / 2 + WALL, fy - cy / 2 - WALL, fy + cy / 2 + WALL, 0, cz + WALL)
    body += [pocket, bar((fx, fy), (fx, 27.8)), bar((fx - 14, 27.8), (fx - 14, fy))]
    m = union(body)
    cuts = [cyl(x, -y, -1, ST + 1, M3_INS / 2) for x, y in bolts + rec]                           # M3 x 5.7 / x 4 inserts (through)
    cuts += [cyl(x, -y, -1, T + 1, 4.5) for x, y in rings]                                       # Ø9 driver holes
    cuts += [box(fx - cx / 2, fx + cx / 2, fy - cy / 2, fy + cy / 2, -1, cz)]                      # sensor cavity
    cuts += [box(fx - 8, fx + 8, fy - 6, fy + 6, cz - 1, cz + WALL + 1)]                          # lens + ToF window 16 x 12
    cuts += [box(fx - cx / 2 - WALL - 1, fx - cx / 2 + 0.5, fy - 4, fy + 4, -1, 4.5)]            # SH1.0 cable slot (short edge)
    return m - union(cuts)


if __name__ == "__main__":
    out = {}
    for name, fn in (("overseer_core_coupon_hub_deck_v1.stl", hub_deck_coupon), ("overseer_core_coupon_sled_v1.stl", sled_coupon)):
        path, t = save(fn(), name)
        g = t.volume / 1000 * 1.27
        out[name] = {"mass_g": round(g, 1), "size_mm": [round(v, 1) for v in t.extents], "watertight": bool(t.is_watertight)}
        print(name, out[name])
    json.dump(out, open(os.path.join(OUT, "coupons.json"), "w"), indent=1)
