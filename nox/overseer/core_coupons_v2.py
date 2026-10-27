"""Overseer core v2 prints (27 Oct): the keel front hub mounting coupon and the flow clip v1 (the clip is drawn and small: it is its
own check). Positions from `runs/overseer_core/v2/result.json` (never retyped); each printed the way it sits on the drone.

  keel hub coupon   the hub deck's front: the 4 front feet Ø12 x 6.5 on the plate's holes (96, ±17) Ø3.5 and (65, ±22.5) Ø3.1
                    with Ø4.0 insert holes through, the keel triangles' sides and cross bar, the seats and the 2 keel bosses Ø13 with the 6.2 x 15 bores on the keel axes
                    (aimed at the head lugs (379.7, ±46, 54.2)); plate face down. Checks: the holes, the inserts, a 6 mm tube in
                    each bore and its line to the head, the flow clip under it.
  flow clip v1      under the front wing: the MTF-02P pocket (0.2 a side, 0.3 deep extra) with the sensor's top on the plate,
                    a 3.0 floor and walls with the lens window (12 x 10, E: centred on the board), a cable notch at the nose, a 3.0 flange with
                    Ø3.4 holes at (96, ±17) under the screw heads: the deck's 2 front M3 x 10 come up through it, the plate and into the deck's inserts.
                    Printed floor down (its bottom on the bed, as mounted).

python3 nox/overseer/core_coupons_v2.py      (after core_v2.py; writes print/overseer_core_coupon_keel_hub_v2.stl, print/overseer_flow_clip_v1.stl)
"""
import json, os
import numpy as np, manifold3d as mf, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
R = json.load(open(os.path.join(ROOT, "runs", "overseer_core", "v2", "result.json")))
C = R["coupon"]; SK = R["socket"]; INS = R["insert"]; FH = R["foot_h"]
OUT = os.path.join(ROOT, "runs", "overseer_core_coupons", "v2"); os.makedirs(OUT, exist_ok=True)
PRINT = os.path.join(ROOT, "print")
BAR_W, BAR_T, PAD_D, M3_CLR = 6.0, 4.0, 12.0, 3.4
FLOW = (21.6, 16.0, 6.5)            # MTF-02P (spec), long side along x
FLOW_CLR, FLOW_ZCLR, WALL = 0.2, 0.3, 3.0     # walls and floor 3.0: the solid gate fails necks under 3.0 (2.0 failed, 27 Oct)
WIN = (12.0, 10.0)                  # lens window (E: lens + ToF centred on the board)
FRONT = [tuple(f) for f in R["feet_xy"] if f[0] > 90]          # the keel pair (96, ±17), through the flow clip
MID = [tuple(f) for f in R["feet_xy"] if 60 < f[0] < 90]      # (65, ±22.5): the keel triangles' back corners
FLANGE = 3.0                       # the clip's flange under the screw heads (concept review: only 3 of PETG in the keel joint)


def cyl(x, y, z0, z1, r, n=40):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def along(a, d, l0, l1, r, n=48):
    """a cylinder of radius r on the axis a + d t, t in l0..l1 (manifold's own cylinder, turned onto d)"""
    d = np.asarray(d, float) / np.linalg.norm(d); q0 = np.asarray(a, float) + d * l0
    th, ph = np.degrees(np.arccos(np.clip(d[2], -1, 1))), np.degrees(np.arctan2(d[1], d[0]))
    return mf.Manifold.cylinder(l1 - l0, r, r, n).rotate([0, th, 0]).rotate([0, 0, ph]).translate(q0.tolist())


def union(ms):
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add)


def save(m, name):
    mm = m.to_mesh(); t = trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts))
    t.apply_translation(-t.bounds[0]); path = os.path.join(PRINT, name); t.export(path); return path, t


def bar_(p, q, w=BAR_W, t=BAR_T):
    p, q = np.asarray(p, float), np.asarray(q, float); d = q - p; L = float(np.linalg.norm(d))
    return mf.Manifold.cube([L, w, t]).translate([0, -w / 2, 0]).rotate([0, 0, float(np.degrees(np.arctan2(d[1], d[0])))]).translate([p[0], p[1], 0])


def keel_axis(sy):
    a, b = (np.array(v, float) for v in R["keels"][str(sy)]); return a, (b - a) / np.linalg.norm(b - a)


def keel_hub_coupon():
    body = [cyl(x, y, 0, FH, PAD_D / 2) for x, y in FRONT + MID]
    body += [box(88.0, 100.0, sy * 6.0, sy * 18.0, 0, 8.0) for sy in (1, -1)]                       # seats under the bosses
    body += [box(93.0, 99.0, -18.0, 18.0, 0, BAR_T)]                                                  # cross bar at x 96
    body += [bar_(m, f) for m, f in zip(MID, FRONT if FRONT[0][1] * MID[0][1] > 0 else FRONT[::-1])]  # triangle sides
    body += [along(*keel_axis(sy), -2.0, SK["off"] + SK["l"], SK["od"] / 2) for sy in (1, -1)]
    cut = [cyl(x, y, -1, FH + 1, INS[0] / 2) for x, y in FRONT + MID]                               # insert holes, through
    cut += [along(*keel_axis(sy), SK["off"], SK["off"] + SK["l"] + 1.0, SK["d"] / 2) for sy in (1, -1)]
    cut += [box(80, 130, -50, 50, -20, 0)]                                                            # nothing below the plate face
    return union(body) - union(cut)


def flow_clip():
    fx, fy = C["flow"]; zt = C["z_plate_bot"]
    cx, cy, cz = FLOW[0] + 2 * FLOW_CLR, FLOW[1] + 2 * FLOW_CLR, FLOW[2] + FLOW_ZCLR
    zb = zt - cz - WALL
    body = [box(fx - cx / 2 - WALL, fx + cx / 2 + WALL, fy - cy / 2 - WALL, fy + cy / 2 + WALL, zb, zt)]
    body += [cyl(x, y, zt - FLANGE, zt, PAD_D / 2) for x, y in FRONT]
    body += [box(90.0, 102.0, sy * 7.0, sy * 17.0, zt - FLANGE, zt) for sy in (1, -1)]
    cut = [box(fx - cx / 2, fx + cx / 2, fy - cy / 2, fy + cy / 2, zt - cz, zt + 1)]                  # pocket, open to the plate
    cut += [box(fx - WIN[0] / 2, fx + WIN[0] / 2, fy - WIN[1] / 2, fy + WIN[1] / 2, zb - 1, zt)]     # lens window
    cut += [box(fx + cx / 2 - 1, fx + cx / 2 + WALL + 1, fy - 3.0, fy + 3.0, zt - 4.0, zt + 1)]       # cable notch at the nose
    cut += [cyl(x, y, zb - 1, zt + 1, M3_CLR / 2) for x, y in FRONT]
    return union(body) - union(cut)


if __name__ == "__main__":
    out = {}
    for name, fn in (("overseer_core_coupon_keel_hub_v2.stl", keel_hub_coupon), ("overseer_flow_clip_v1.stl", flow_clip)):
        path, t = save(fn(), name)
        out[name] = {"mass_g": round(float(t.volume / 1000 * 1.27), 1), "size_mm": [round(float(v), 1) for v in t.extents], "watertight": bool(t.is_watertight)}
        print(name, out[name])
    json.dump(out, open(os.path.join(OUT, "coupons.json"), "w"), indent=1)
