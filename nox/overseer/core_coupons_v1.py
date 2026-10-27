"""Overseer core mounting check coupons v1 (27 Oct, Spark: "make a fast to print mounting check coupons first, in the orientation
we are going to put them in. make it fast to print"; "make sure bolts are accessible through a screwdriver"; "the fc mounts to the
top plates lines so just move that there"; "the fc does not need a mount it already goes to the carbon plate": no stack
mount and no coupon for it, the stack bolts through the plate's own holes).

Drawn, least material: only the mounting features of the two core pieces (`nox/overseer/core_v1.py` round 5; every position read
from `runs/overseer_core/v1/result.json`, never retyped) joined by 6 x 4 bars (4 thick: solid_check fails necks under 3.0), each
printed the way it sits on the drone:

  hub deck coupon     on the bottom plate, plate face down. 6 feet Ø12 x 4 (Ø3.4) on the plate's Ø3.1 holes (±24, ±36.5) and
                      (-65, ±22.5) (M3 x 12 down into the sled's inserts, or nuts under the plate for the check); the Pi's 4 posts
                      Ø7 x 8 on its 58 x 49 (Ø2.2 x 4 for M2.5 self-tapping); the UBEC / buck bay rails ahead of it. Checks: the
                      plate holes, the Pi between the arm roots with its plugs aft, the cooler under the top plate, the bays.
  sled coupon         under the bottom plate; printed with its top (the plate face) down, so it bolts on flipped over as it prints
                      (y mirrored here). 6 pads Ø12 x 6 under the hub deck feet and 6 under the receiver v2 bolts (Ø4.0 through:
                      inserts), Ø9 driver rings under the 8 arm bottom screws, the MTF-02P pocket (0.2 a side, 0.3 deep extra,
                      lens window, cable slot). Checks: the bolts line up through the plate, the arm screw heads pass the rings,
                      the flow sensor fits (plan and fit only: the pod's 55 drop to the lens is not in the coupon).

python3 nox/overseer/core_coupons_v1.py      (after core_v1.py; writes print/overseer_core_coupon_*_v1.stl)
"""
import json, os
import numpy as np, manifold3d as mf, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v1", "result.json")))
C = R["coupon"]
OUT = os.path.join(ROOT, "runs", "overseer_core_coupons", "v1"); os.makedirs(OUT, exist_ok=True)
PRINT = os.path.join(ROOT, "print")

BAR_W, BAR_T = 6.0, 4.0            # bars (CLAUDE.md: nothing under 2.0; 4 thick: the solid gate's 3.0 neck test)
PAD_D = 12.0                       # pads >= Ø12 with >= 2.0 web (CLAUDE.md)
M3_CLR, M3_INS, M25_TAP = 3.4, 4.0, 2.2   # M3 clearance; M3 heat-set hole (M, Ruthex); M2.5 self-tapping in PETG (E)
FLOW = (21.6, 16.0, 6.5)           # MTF-02P (spec)
FLOW_CLR, FLOW_ZCLR, WALL = 0.2, 0.3, 2.0


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
    t.apply_translation(-t.bounds[0])
    path = os.path.join(PRINT, name); t.export(path)
    return path, t


def hub_deck_coupon():
    """drone frame, z 0 = bottom plate top = bed"""
    T, ry, zp = C["floor_t"], C["rail_y"], C["z_pi"]
    feet = [tuple(f) for f in R["feet_xy"]]
    ph = [tuple(p) for p in R["pi_holes"]]
    px0, px1 = ph[0][0], ph[2][0]
    body = [cyl(x, y, 0, T, PAD_D / 2) for x, y in feet] + [cyl(x, y, 0, zp, 3.5) for x, y in ph]
    body += [bar((24, sy * 36.5), (px1, sy * 24.5)) for sy in (1, -1)] + [bar((-24, sy * 36.5), (px0, sy * 24.5)) for sy in (1, -1)]
    body += [bar((px0, 24.5), (px0, -24.5)), bar((px1, 24.5), (px1, -24.5))]
    body += [bar((-65.0, sy * 22.5), (px0, sy * ry)) for sy in (1, -1)] + [bar((px0, sy * ry), (px1, sy * ry)) for sy in (1, -1)]
    xb = C["ubec_bay"][1] - 2.0
    body += [bar((px1, sy * ry), (xb, sy * ry)) for sy in (1, -1)] + [bar((xb, ry), (xb, -ry)), bar((57, ry), (57, -ry))]
    cuts = [cyl(x, y, -1, T + 1, M3_CLR / 2) for x, y in feet] + [cyl(x, y, zp - 4.0, zp + 1, M25_TAP / 2) for x, y in ph]
    return union(body) - union(cuts)


def sled_coupon():
    """drone frame flipped about x (bed = the sled's top, against the plate): y mirrored"""
    T, ST = BAR_T, C["sled_t"]
    bolts = [tuple(f) for f in R["feet_xy"]] + [tuple(a) for a in C["aft_legs"]]
    rec = [tuple(b) for b in R["receiver_bolts"]]
    rings = [tuple(a) for a in R["arm_screws_driver_holes"]]
    body = [cyl(x, -y, 0, ST, PAD_D / 2) for x, y in bolts + rec] + [cyl(x, -y, 0, T, 7.5) for x, y in rings]
    for sy in (1, -1):
        body.append(bar((-112, sy * 27.8), (58, sy * 27.8)))
    body += [bar((-109, 27.8), (-109, -27.8)), bar((55, 27.8), (55, -27.8))]
    for x, y in bolts + rings:
        body.append(bar((x, -y), (x, -np.sign(y) * 27.8)))
    fx, fy = C["flow"][0], -C["flow"][1]
    cx, cy, cz = FLOW[0] + 2 * FLOW_CLR, FLOW[1] + 2 * FLOW_CLR, FLOW[2] + FLOW_ZCLR
    body += [box(fx - cx / 2 - WALL, fx + cx / 2 + WALL, fy - cy / 2 - WALL, fy + cy / 2 + WALL, 0, cz + WALL),
             bar((fx, fy), (fx, 27.8)), bar((fx - 14, 27.8), (fx - 14, fy))]
    cuts = [cyl(x, -y, -1, ST + 1, M3_INS / 2) for x, y in bolts + rec] + [cyl(x, -y, -1, T + 1, 4.5) for x, y in rings]
    cuts += [box(fx - cx / 2, fx + cx / 2, fy - cy / 2, fy + cy / 2, -1, cz),                     # sensor cavity, open at the bed (the pod's inside)
             box(fx - 8, fx + 8, fy - 6, fy + 6, cz - 1, cz + WALL + 1),                           # lens + ToF window 16 x 12
             box(fx - cx / 2 - WALL - 1, fx - cx / 2 + 0.5, fy - 4, fy + 4, -1, 4.5)]              # SH1.0 cable slot (short edge)
    return union(body) - union(cuts)


if __name__ == "__main__":
    out = {}
    for name, fn in (("overseer_core_coupon_hub_deck_v1.stl", hub_deck_coupon),
                     ("overseer_core_coupon_sled_v1.stl", sled_coupon)):
        path, t = save(fn(), name)
        out[name] = {"mass_g": round(float(t.volume / 1000 * 1.27), 1), "size_mm": [round(float(v), 1) for v in t.extents], "watertight": bool(t.is_watertight)}
        print(name, out[name])
    json.dump(out, open(os.path.join(OUT, "coupons.json"), "w"), indent=1)
