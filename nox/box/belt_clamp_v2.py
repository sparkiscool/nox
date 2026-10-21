"""Belt clamp v2 (17 Oct 2026): GT2 loop-and-pin clamp, after the proven printer / robot-kit designs.
The belt end is folded back on itself TEETH IN, so the two legs mesh tooth into tooth, pushed up a narrow slot, and an
M3 screw goes through the loop as a pin. Pull on the run tries to slide the legs apart along the slot: the meshed
teeth stop it, the slot walls keep them meshed, the pin stops the loop pulling through.
Sources of the method: Vampire Bat carriage clips (belt through a thin slot, wrapped round a 4 mm tube, squeezed to
itself), goBILDA / ServoCity GT2 3-1 clamp (belt meshed to itself, one pinch bolt), printables 617391 (fold-in-loop,
teeth against teeth, one part). v1 (printed tooth grooves + cover plate) is retired: Spark 17 Oct 15:35 "didn't work".

Coupon frame = print frame: x across the belt thickness, y along the run (+y away from the load), z across the belt
width (print up). All mm, C unless tagged. Belt GT2 2 mm x 6 (spec E): total 1.38, back 0.63, tooth 0.75.
Meshed pair = 0.63 + 0.75 + 0.63 = 2.01 (E): the coupon set prints the slot at 1.8 / 2.0 / 2.2 and the hang test picks.
python belt_clamp_v2.py -> out/belt_clamp_v2_coupon_set.stl (3 coupons, one plate) + checks.
"""
import math, os
import numpy as np
import manifold3d as mf

BACK, TOOTH_H, TOTAL = 0.63, 0.75, 1.38
BELT_W = 6.0
MESHED = 2 * BACK + TOOTH_H          # 2.01 (E)
CH_W = BELT_W + 0.4                  # channel across the belt width (DEFAULT)
WALL_Z = 2.0                         # floor / roof walls across the width
SLOT_L = 12.0                        # meshed length = 6 teeth, so two ends back to back fit a head's 40 clamp zone: 2 x (12 + 3.6) + 8.5 = 39.7 (C)
PIN_D, PIN_HOLE = 3.0, 3.3           # M3 screw shank as the pin; clearance hole
POCKET_R = PIN_D / 2 + TOTAL + 0.35  # 2.88 wrap + 0.35 = 3.23
POCKET_YC = SLOT_L + 3.6             # pin centre: legs converge from the wrap to the slot over 3.6
WALL_X = 2.4                         # side walls (load the meshed legs), >= 2.0
X_HALF = POCKET_R + WALL_X           # 5.63
TAB_L, HANG_D = 13.0, 6.5            # hang tab beyond the pocket, Ø6.5 hole through z (M6 hang bolt)
LEAD = 0.8                           # 45° lead-in at the slot mouth
WINDOW_W = 4.0                       # sight window over the pocket on +x (see the loop, aim the pin)

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl_z(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

def hull(pts):
    return mf.Manifold.hull_points(np.array(pts, dtype=np.float64))

def coupon(slot):
    z0, z1 = 0.0, CH_W + 2 * WALL_Z
    zc0, zc1 = WALL_Z, WALL_Z + CH_W                     # channel in z
    y_top = POCKET_YC + POCKET_R + WALL_X
    body = box(-X_HALF, X_HALF, 0.0, y_top + TAB_L, z0, z1)
    # round the tab end (beauty + no corner to catch)
    yt = y_top + TAB_L
    body = body - box(-X_HALF - 1, X_HALF + 1, yt - X_HALF, yt + 1, z0 - 1, z1 + 1) + cyl_z(0, yt - X_HALF, z0, z1, X_HALF)
    # slot, lead-in, converging throat, pocket: all through the channel only (floor and roof stay)
    cut = box(-slot / 2, slot / 2, -1.0, SLOT_L + 0.01, zc0, zc1)
    cut += hull([[sx * (slot / 2 + LEAD), -0.01, z] for sx in (-1, 1) for z in (zc0, zc1)] +
                [[sx * slot / 2, LEAD, z] for sx in (-1, 1) for z in (zc0, zc1)])
    cut += hull([[sx * slot / 2, SLOT_L, z] for sx in (-1, 1) for z in (zc0, zc1)] +
                [[sx * POCKET_R, POCKET_YC, z] for sx in (-1, 1) for z in (zc0, zc1)])
    cut += cyl_z(0, POCKET_YC, zc0, zc1, POCKET_R)
    cut += box(0, X_HALF + 1, POCKET_YC - WINDOW_W / 2, POCKET_YC + WINDOW_W / 2, zc0, zc1)   # sight window
    cut += cyl_z(0, POCKET_YC, z0 - 1, z1 + 1, PIN_HOLE / 2)                                 # pin through both walls
    cut += cyl_z(0, yt - X_HALF, z0 - 1, z1 + 1, HANG_D / 2)                                 # hang hole
    body -= cut
    # variant mark: one to three 0.6-deep notches across the tab end face
    k = {1.8: 1, 2.0: 2, 2.2: 3}[round(slot, 1)]
    for i in range(k):
        xm = (i - (k - 1) / 2) * 2.2
        body -= box(xm - 0.4, xm + 0.4, yt - 0.6, yt + 1, z1 - 0.6, z1 + 1)
    return body

def belt_loop(slot):
    """the folded belt as seated (for checks and pictures): two legs in the slot, half-turn round the pin."""
    zc0 = WALL_Z + 0.2
    legs = None
    t = (slot) / 2                                         # legs fill the slot, meshed
    for sx in (-1, 1):
        leg = box(sx * t - (sx > 0) * TOTAL if False else min(0, sx * t), max(0, sx * t), -30.0 if sx < 0 else 2.0,
                  SLOT_L, zc0, zc0 + BELT_W)
        legs = leg if legs is None else legs + leg
    wrap = cyl_z(0, POCKET_YC, zc0, zc0 + BELT_W, PIN_D / 2 + TOTAL) - cyl_z(0, POCKET_YC, zc0 - 1, zc0 + BELT_W + 1, PIN_D / 2)
    wrap = wrap - box(-10, 10, POCKET_YC - 10, POCKET_YC, zc0 - 1, zc0 + BELT_W + 1)
    conv = hull([[sx * t, SLOT_L, z] for sx in (-1, 1) for z in (zc0, zc0 + BELT_W)] +
                [[sx * (PIN_D / 2 + TOTAL), POCKET_YC, z] for sx in (-1, 1) for z in (zc0, zc0 + BELT_W)])
    conv -= cyl_z(0, POCKET_YC, zc0 - 1, zc0 + BELT_W + 1, PIN_D / 2) + hull(
        [[sx * max(t - TOTAL, 0.05), SLOT_L - 0.01, z] for sx in (-1, 1) for z in (zc0 - 1, zc0 + BELT_W + 1)] +
        [[sx * PIN_D / 2, POCKET_YC, z] for sx in (-1, 1) for z in (zc0 - 1, zc0 + BELT_W + 1)])
    return legs + wrap + conv

def pin():
    return cyl_z(0, POCKET_YC, -2.5, CH_W + 2 * WALL_Z + 0.5, PIN_D / 2, 40)

def export(m, path):
    import trimesh
    me = m.to_mesh()
    t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    t.export(path); return t

if __name__ == "__main__":
    os.makedirs("out", exist_ok=True)
    print("meshed pair (E) %.2f; pocket r %.2f; pin centre y %.1f; coupon %.1f x %.1f x %.1f"
          % (MESHED, POCKET_R, POCKET_YC, 2 * X_HALF, POCKET_YC + POCKET_R + WALL_X + TAB_L, CH_W + 2 * WALL_Z))
    plate = None
    for i, s in enumerate((1.8, 2.0, 2.2)):
        c = coupon(s)
        bodies = len(c.decompose())
        thin = min(X_HALF - POCKET_R, X_HALF - s / 2, WALL_Z)
        clr = (pin() ^ c).volume()
        print(f"slot {s}: {c.volume() / 1000 * 1.27:.2f} g (M), bodies {bodies}, thinnest wall {thin:.2f}, pin vs part {clr:.3f} mm3")
        export(c, f"out/belt_clamp_v2_coupon_s{int(s * 10)}.stl")
        c2 = c.translate([i * (2 * X_HALF + 6.0), 0, 0])
        plate = c2 if plate is None else plate + c2
    t = export(plate, "out/belt_clamp_v2_coupon_set.stl")
    print("set", [round(v, 2) for v in t.extents], "watertight", t.is_watertight, "%.2f g" % (plate.volume() / 1000 * 1.27))
