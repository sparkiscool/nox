"""Scan stick system model and compact placement (Spark 24 Oct: "needs to be a lot more compact, use the most compact design
possible"; rule: compact by default, docs/generative.md The rule 0). Places every bought part round the pole axis and picks the
arrangement and the sensor positions that minimise the assembled bounding volume (guards included) under the constraints:

  - L2 forward (+X) 15° down (Spark); nothing printed or bought above the L2's lowest base edge in front of its base plane (the
    stick below it is cropped in SLAM, Spark 24 Oct); the L2 body clear of everything.
  - camera level, forward, behind the L2 base plane (out of the lidar's view), its view (89 x 73°, E) not blocked by the L2 by more
    than 2° (E) and clear of everything else; 10 mm behind the board for the plug (Spark 23 Oct).
  - pack 148 x 48 x 45 (P) in a pocket with one open face for the swap (pinned), nothing in front of that face.
  - Pi 5 (85 x 56, 30 deep with standoffs and parts, E), UBEC 50 x 28 x 14 and buck 50 x 28 x 16 (E), switch strip 60 x 24 x 21
    deep (E) each flat on a face, reachable; thread boss Ø40 x 34 (C) on the pole axis under the CG.
  - structure allowance 6 mm (C, the generated walls) between parts and round the pack; guards 5 mm (C) beyond the outermost part
    on every exposed side.
Arrangements (discrete): pack on the axis ("spine") or beside a column (v3, the baseline); electronics on the back, on the sides
or split; L2 on top or on the front face. Continuous: L2 base centre (x, z), camera height above the L2. Scored by bounding volume,
then the largest extent.   python3 scan_stick_system.py [--render]"""
import sys, os, itertools, json
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
import numpy as np
from scipy.optimize import minimize

TILT = np.radians(15.0)                                       # Spark
s, c = np.sin(TILT), np.cos(TILT)
F, U = np.array((c, 0, -s)), np.array((s, 0, c))             # L2 axis and up in its base plane (x, z only: y is plain)
L2_HALF, L2_LEN = 37.5, 65.0                                  # spec
PACK = (47.0, 50.0, 150.0)                                    # P + 1 clearance (x, y, z), vertical
PI = (30.0, 56.0, 85.0)                                       # E: depth, width, height
UBEC, BUCK = (14.0, 28.0, 50.0), (16.0, 28.0, 50.0)           # E
SW = (21.0, 60.0, 24.0)                                       # E: depth, width, height
CAM_D, CAM_H, LENS = 17.6, 38.0, 16.0                         # C: plate 6 + posts 10 + board 1.6 behind the front face; board 38; lens 16 (E)
WALL, GUARD, BOSS_Z = 6.0, 5.0, 34.0                          # C
TV = np.tan(np.radians(36.5))                                 # E: camera half view, vertical

def l2_corners(ox, oz):
    """the L2 body's corners in the x-z plane (front view of its tilted box)"""
    pts = []
    for f in (0.0, L2_LEN):
        for u in (-L2_HALF, L2_HALF):
            pts.append(np.array((ox, oz)) + f * F[[0, 2]] + u * U[[0, 2]])
    return np.array(pts)

def plane_x(ox, oz, z):
    return ox + np.tan(TILT) * (z - oz)

def spine(elec, l2_on, ox, oz, cam_dz, zs=None):
    """pack on the pole axis; returns (boxes, info) or None if infeasible. boxes: name -> (x0, x1, y0, y1, z0, z1)"""
    px, py, pz = PACK
    z0 = BOSS_Z + 4.0
    B = {"pack": (-px / 2, px / 2, -py / 2, py / 2, z0, z0 + pz)}
    sx0, sx1, sy0, sy1, sz1 = -px / 2 - WALL, px / 2 + WALL, -py / 2 - WALL, py / 2 + WALL, z0 + pz + WALL
    B["structure"] = (sx0, sx1, sy0, sy1, 0.0, sz1)
    back, side_p, side_n = sx0, sy1, sy0
    if elec == "back":                                          # Pi above, UBEC + buck side by side below it, all on the back face
        B["pi"] = (sx0 - PI[0], sx0, -PI[1] / 2, PI[1] / 2, sz1 - WALL - PI[2], sz1 - WALL)
        B["ubec"] = (sx0 - UBEC[0], sx0, -UBEC[1] - 1, -1, B["pi"][4] - 4 - UBEC[2], B["pi"][4] - 4)
        B["buck"] = (sx0 - BUCK[0], sx0, 1, 1 + BUCK[1], B["pi"][4] - 4 - BUCK[2], B["pi"][4] - 4)
        if B["ubec"][4] < BOSS_Z:
            return None
        open_face = "top"                                       # Spark 24 Oct: the pack drops in from the top, a 3 mm pin across
    elif elec == "sides":                                       # Pi on +y (its 56 across x sticks out of the 59 face), UBEC + buck on -y
        B["pi"] = (-PI[1] / 2, PI[1] / 2, sy1, sy1 + PI[0], sz1 - WALL - PI[2], sz1 - WALL)
        B["ubec"] = (-UBEC[1] - 1, -1, sy0 - UBEC[0], sy0, z0 + 10, z0 + 10 + UBEC[2])
        B["buck"] = (1, 1 + BUCK[1], sy0 - BUCK[0], sy0, z0 + 10, z0 + 10 + BUCK[2])
        open_face = "top"
    else:
        return None
    if open_face == "+y":
        side_open = True
    # switches on the front face low down, in the stick's crop zone, under the L2
    zs = z0 + 20 if zs is None else zs                          # v2: the switch height is a variable (concept review 24 Oct)
    B["switches"] = (sx1, sx1 + SW[0], -SW[1] / 2, SW[1] / 2, zs, zs + SW[2])
    # L2 and camera
    corners = l2_corners(ox, oz)
    B["l2"] = (corners[:, 0].min(), corners[:, 0].max(), -L2_HALF, L2_HALF, corners[:, 1].min(), corners[:, 1].max())
    cz = oz + cam_dz                                            # camera centre height
    xf = min(plane_x(ox, oz, cz - 6) - 1.0 - LENS, plane_x(ox, oz, cz - CAM_H / 2) - 1.0)   # lens tip and board edge 1 behind the plane
    B["camera"] = (xf - CAM_D, xf, -CAM_H / 2, CAM_H / 2, cz - CAM_H / 2, cz + CAM_H / 2)   # plate + posts + board
    B["lens"] = (xf, xf + LENS, -6, 6, cz - 6, cz + 6)                                         # Ø12 lens (E, Q3)
    info = dict(open_face=open_face, cam_xf=xf, cam_z=cz, sx1=sx1)
    if CAGE:                                                    # v2: the closed rod cage (concept review 24 Oct), 3 mm rods as 3 mm boxes
        yc = L2_HALF + 3.0 + 1.5 + 2.0
        xb = ox + L2_LEN * c + 3.0 + 1.5                        # lower front bumper 3 ahead of the dome tip
        xback = min(B["pi"][0], B["ubec"][0], B["buck"][0]) - 3.0 - 1.5
        ztop = cz + CAM_H / 2 + 3.0 + 1.5                       # upper ring over the camera
        xfr = plane_x(ox, oz, ztop) - 3.0                       # its front, behind the L2 base plane
        B["cage_low"] = (xback - 1.5, xb + 1.5, -yc - 1.5, yc + 1.5, z0 - 1.5, z0 + 1.5)
        B["cage_top"] = (xback - 1.5, xfr + 1.5, -yc - 1.5, yc + 1.5, ztop - 1.5, ztop + 1.5)
        info.update(xb=xb, xfr=xfr, ztop=ztop, yc=yc)
    return B, info

def l2_body_hits(B, ox, oz):
    """volume-free tests: does any other box enter the L2 body (tilted box), or sit in the L2's view above its lowest base edge?"""
    bad = 0.0
    zlow = oz - L2_HALF * c                                     # the L2 base face's lowest edge
    for n, (x0, x1, y0, y1, z0, z1) in B.items():
        if n in ("l2",):
            continue
        if y1 < -L2_HALF - 2 or y0 > L2_HALF + 2:
            continue
        for x in np.linspace(x0, x1, 7):
            for z in np.linspace(z0, z1, 7):
                p = np.array((x - ox, z - oz))
                f, u = p @ F[[0, 2]], p @ U[[0, 2]]
                if n in ("camera", "lens"):
                    if f > -0.5:
                        bad += f + 0.5                          # the camera must stay behind the base plane
                    continue
                if -2 <= f <= L2_LEN + 2 and -L2_HALF - 2 <= u <= L2_HALF + 2:
                    bad += 1.0 + min(f + 2, L2_LEN + 2 - f, u + L2_HALF + 2, L2_HALF + 2 - u)   # inside the body (+ 2 cooling)
                elif f > 0 and z > zlow:
                    bad += f                                    # in the hemisphere above the lowest base edge
    return bad

def camera_hits(B, info, ox, oz):
    """the camera view must clear the L2 (2° allowance, E) and every other box"""
    xf, cz = info["cam_xf"], info["cam_z"]
    lens = np.array((xf + LENS, cz))
    bad = 0.0
    for p in l2_corners(ox, oz):
        d = p - lens
        if d[0] > 0:
            ang = np.degrees(np.arctan2(abs(d[1]), d[0]))
            if ang < 36.5 - 2.0:
                bad += 36.5 - 2.0 - ang
    for n, (x0, x1, y0, y1, z0, z1) in B.items():
        if n in ("camera", "lens", "l2"):
            continue
        if x1 > lens[0]:
            for x in (max(x0, lens[0]), x1):
                for z in (z0, z1):
                    d = np.array((x, z)) - lens
                    if d[0] > 0 and abs(d[1]) < d[0] * TV and not (y1 < -60 or y0 > 60):
                        bad += 1.0
    return bad

def drop_path_hits(B):
    """the pack lifts straight out of the top: nothing over its footprint above the structure (+1 clearance)"""
    x0, x1, y0, y1, z0, z1 = B["pack"]
    bad = 0.0
    for n, (a0, a1, b0, b1, c0, c1) in B.items():
        if n in ("pack", "structure"):
            continue
        ox_ = min(a1, x1 + 1) - max(a0, x0 - 1); oy_ = min(b1, y1 + 1) - max(b0, y0 - 1)
        if ox_ > 0 and oy_ > 0 and c1 > z1:
            bad += ox_ + 0.0 * oy_
    return bad

CAGE = False

def cage_hits(B, info):
    """the upper ring's front must be 3 ahead of the lens (outside the view), the camera on the module plate (behind the joint face)"""
    bad = 0.0
    if CAGE:
        lens_tip = B["lens"][1]
        if info["xfr"] - 1.5 < lens_tip + 3.0:
            bad += lens_tip + 3.0 - (info["xfr"] - 1.5)
    if B["camera"][0] < info["sx1"]:
        bad += info["sx1"] - B["camera"][0]
    return bad

def extents(B, guard_sides=True):
    a = np.array(list(B.values()))
    lo, hi = a[:, [0, 2, 4]].min(0), a[:, [1, 3, 5]].max(0)
    if not CAGE:
        lo[:2] -= GUARD; hi[:2] += GUARD                      # v1: guards as a 5 mm allowance (v2 models the cage)
    lo[2] = 0.0
    return lo, hi

def score(B):
    lo, hi = extents(B)
    e = hi - lo
    return float(np.prod(e)) / 1e6, e

def optimise_arrangement(elec, l2_on):
    best = None
    for cam_dz0 in (45.0, 60.0, 80.0):
        def obj(v):
            ox, oz, cdz, zs = v
            r = spine(elec, l2_on, ox, oz, cdz, zs)
            if r is None:
                return 1e9
            B, info = r
            if l2_on == "top" and B["l2"][4] < B["structure"][5] - 60:
                return 1e9
            pen = l2_body_hits(B, ox, oz) + camera_hits(B, info, ox, oz) + drop_path_hits(B) + cage_hits(B, info) + max(0.0, 2.0 - zs) * 5
            if cdz < L2_HALF * c + CAM_H / 2 + 3:                 # the camera sits above the L2's top edge
                pen += (L2_HALF * c + CAM_H / 2 + 3 - cdz) * 10
            vol, e = score(B)
            return vol + 0.002 * e.max() + 10.0 * pen
        x0 = (30.0, 230.0, cam_dz0, 58.0) if l2_on == "top" else (45.0, 110.0, cam_dz0, 40.0)
        r = minimize(obj, x0, method="Nelder-Mead", options=dict(xatol=0.2, fatol=1e-5, maxiter=4000))
        if best is None or r.fun < best.fun:
            best = r
    ox, oz, cdz, zs = best.x
    B, info = spine(elec, l2_on, ox, oz, cdz, zs)
    pen = l2_body_hits(B, ox, oz) + camera_hits(B, info, ox, oz) + drop_path_hits(B) + cage_hits(B, info)
    vol, e = score(B)
    return dict(elec=elec, l2=l2_on, ox=round(ox, 1), oz=round(oz, 1), cam_dz=round(cdz, 1), zs=round(zs, 1), cage={k: round(float(v), 1) for k, v in info.items() if k in ("xb", "xfr", "ztop", "yc")}, cam_xf=round(info["cam_xf"], 1),
                cam_z=round(info["cam_z"], 1), open_face=info["open_face"], penalty=round(pen, 3), volume_L=round(vol, 2),
                extents=[round(v) for v in e], boxes={k: [round(t, 1) for t in v] for k, v in B.items()})

V3 = dict(volume_L=None, extents=None)

def v3_baseline():
    """unit v3's assembled envelope (guards included), from its drawn bodies and bought parts (C)"""
    lo, hi = np.array((-72.5, -63.0, 0.0)), np.array((101.0, 59.0, 352.0))
    e = hi - lo
    return round(float(np.prod(e)) / 1e6, 2), [round(v) for v in e]

if __name__ == "__main__" and "--render" not in sys.argv:
    CAGE = "--v1" not in sys.argv
    res = []
    for elec, l2_on in itertools.product(("back", "sides"), ("top", "front")):
        r = optimise_arrangement(elec, l2_on)
        res.append(r)
        print(f"{elec:5s} L2 {l2_on:5s}: {r['volume_L']:.2f} L, extents {r['extents']} mm, penalty {r['penalty']}, "
              f"L2 base centre ({r['ox']}, {r['oz']}), camera z {r['cam_z']} front x {r['cam_xf']}, pack opens {r['open_face']}")
    vb, eb = v3_baseline()
    print(f"unit v3 (baseline): {vb} L, extents {eb} mm")
    ok = [r for r in res if r["penalty"] < 1e-6]
    best = min(ok, key=lambda r: (r["volume_L"], max(r["extents"])))
    print("best:", best["elec"], best["l2"], best["volume_L"], "L", best["extents"])
    out = os.path.join(ROOT, "runs", "scan_stick_system", "v1" if not CAGE else "v2"); os.makedirs(out, exist_ok=True)
    json.dump(dict(candidates=res, baseline=dict(volume_L=vb, extents=eb), best=best), open(os.path.join(out, "layout.json"), "w"), indent=1)

def render(best, out):
    """the chosen placement as a motion artifact: bought parts as boxes, the L2 tilted, the structure envelope ghosted"""
    import math, trimesh
    sys.path.insert(0, os.path.join(ROOT, "nox"))
    from motion import Scene
    def bx(b):
        x0, x1, y0, y1, z0, z1 = b
        m = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return m
    B = best["boxes"]
    M = trimesh.transformations.rotation_matrix(TILT, [0, 1, 0]); M[:3, 3] = (best["ox"], 0, best["oz"])
    l2 = bx((0, 39, -37.5, 37.5, -37.5, 37.5))
    dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([26 / 35.5, 1, 1]); dome.apply_translation([39, 0, 0])
    dome = dome.slice_plane([39, 0, 0], [1, 0, 0], cap=True)
    l2 = trimesh.util.concatenate([l2, dome]); l2.apply_transform(M)
    boss = trimesh.creation.cylinder(20, BOSS_Z, sections=48); boss.apply_translation([0, 0, BOSS_Z / 2])
    pole = trimesh.creation.cylinder(12.7, 300, sections=48); pole.apply_translation([0, 0, -150])
    s = Scene("Scan stick compact layout", f"placement optimisation (scan_stick_system.py): {best['volume_L']} L, {best['extents']} mm with guards; unit v3 was 7.45 L, 174 x 122 x 352 mm. Pack on the pole axis, Pi + UBEC + buck on the back, L2 on the front 15° down, camera above it, switches low on the front, pack drops in from the top")
    cols = dict(pack="#2f4fbf", pi="#2e7d32", ubec="#b07d2b", buck="#b05a2b", switches="#888888", camera="#2d5a2d", lens="#2d5a2d")
    s.part("structure envelope", bx(B["structure"]), group="printed", color="#c8d0da", printed=True, note="the generated pocket and frame go here (6 mm walls, C)")
    s.part("thread boss", boss, group="printed", color="#9aa8b8", printed=True)
    for n, col in cols.items():
        s.part(n, bx(B[n]), group="bought", color=col, printed=False)
    s.part("Unitree L2", l2, group="bought", color="#222222", printed=False)
    s.part("pole", pole, group="bought", color="#777777", printed=False)
    names = ["structure envelope", "thread boss"] + list(cols) + ["Unitree L2", "pole"]
    s.timeline(lambda t: {n: trimesh.transformations.rotation_matrix(2 * math.pi * t / 16, [0, 0, 1]) for n in names}, duration_s=16, fps=30)
    s.view("iso", (420, -480, 300), (20, 0, 110)); s.view("side", (0, -700, 110), (20, 0, 110)); s.view("front", (700, 0, 110), (20, 0, 110))
    os.makedirs(out, exist_ok=True)
    s.write(os.path.join(out, "motion.html"))
    try:
        s.video(os.path.join(out, "motion.mp4"))
    except Exception as e:
        print("video skipped:", str(e)[:200])

if __name__ == "__main__" and "--render" in sys.argv:
    out = os.path.join(ROOT, "runs", "scan_stick_system", "v1")
    render(json.load(open(os.path.join(out, "layout.json")))["best"], out)
