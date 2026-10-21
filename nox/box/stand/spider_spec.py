"""Stub spider v1: drawn set, cuts, envelope and load cases for ONE quadrant (16 Oct 2026).

The spider is 4 identical quadrants (one STL) glued + dowelled on the X = 0 and Z = 0 planes. The assembled spider is
the canonical quadrant Q0 (X >= 0, Z <= 0) rotated by k * 90 deg about the axle (Y). Every feature that one quadrant
needs in ANY of its four positions is drawn in Q0, and every load a quadrant sees in any position is mapped into Q0, so
the one generated part is designed for its worst position.

Coordinates W (nox/box-arm.md), mm. Floor Z -160 (C). Tags: C CAD, F frame model, E estimate, DEFAULT = decided here.
Self-contained (manifold3d only); nox/box/stand/spider_run.py feeds it to the noxopt pipeline (arm_topo.run_part).
python spider_spec.py  -> prints the part summary, writes out/spider_drawn.stl, out/spider_keepout.stl
"""
import math, os
import numpy as np
import manifold3d as mf

# ------------------------------------------------------------------ numbers
ZF = -160.0                       # floor, C (box-stand.md)
E_OUT = 157.0                     # outer face of the foot lands = floor + 3 (pad 4 in a 1-deep pocket), DEFAULT
FOOT = (140.0, -64.0)             # back foot X, Y, F (box-stand.md); in Q0 the bottom seat, its diagonal image the side seat
PAD = 30.0                        # pad 30 x 30 x 4 TPU 95A, E (pad_v1.py)
LAND = 36.0                       # seat land square, DEFAULT (rim 3 round the pad pocket)
POCKET = 30.6                     # pad pocket, 1 deep, DEFAULT
SNAP_D, CAV_D, CAV_H = 6.4, 11.0, 6.0   # pad stem hole, barb cavity behind the 4-mm floor, DEFAULT
R_BOLT, D_M5 = 38.0, 5.5          # stub flange 4 x M5 at r 38 / 45 deg, C (box-arm.md)
SEAT_R, SEAT_T = 46.0, 4.0        # flange seat Ø92, face Y 0; seat Y -2..0 + 2 backing so it voxelises at h 4, DEFAULT
WIRE_R = 12.0                     # slip-ring wire hole Ø24 through the centre, C (stub bore)
Y_BACK = -8.0                     # back face of the plate = PSU bottom face, E (stand_model PSU Y -38..-8)
TIE_R, TIE_ANG = 146.0, -45.0     # tie on the diagonal, F (box-stand.md)
TIE_Y = 88.0                      # ring near-face boss face, C now; follows the ring stack -> parameter
TIE_TAB = (12.0, 4.0, 3.4, 6.5)   # tab Ø, thickness, M3 clearance, head clearance Ø, DEFAULT
PSU_HOLES = [(75.0, -25.0), (75.0, -75.0), (25.0, -75.0)]   # Q0 share of the 12-boss C4 orbit, see below
M4_D, M4_CB = 4.5, 8.0            # PSU M4 x 6 SHCS from the front, head counterbore Y -5..0, DEFAULT
BOSS_D = 12.0                     # M4 boss, wall 2.0 round the counterbore
M5_BOSS_D, M5_CB = 14.0, 9.5      # M5 SHCS from the back, head counterbore Y -8..-3 (under the PSU), DEFAULT
DOWEL_D, DOWEL_L = 3.1, 7.0       # Ø3 x 14 steel dowel, 7 each side, DEFAULT
Y_MIN = FOOT[1] - LAND / 2 - 1.0  # -83
"""PSU LRS-350-24: 215 x 115 x 30, 760 g, 4 x M4 bottom (max 3 deep), C (Mean Well datasheet, case 207A); hole pattern
150 x 50 E, CHECK the drawing. PSU centre moved Z -55 E -> -50 DEFAULT: then the 4 holes (+-75, -25 / -75) and their
rotations form a 12-hole C4 orbit, 3 per quadrant: (+-75, +-75), (+-75, +-25), (+-25, +-75). Bottom pair carries the PSU,
the top pair carries the SKR + chargers carrier (150 x 50 at Z +50), the rest are spare."""

# arm sweep keep-outs (E, from nox/box-arm.md; the arm is being reoriented, CHECK after its next sweep)
SWEEP = [(0.0, 50.0, 99.0),       # motors on the hub plate, Y 2..50, nearest r ~97
         (50.0, 74.0, 145.5),     # head B idler rim r 143.6 at Y 54..64, belt clamps
         (74.0, TIE_Y, 137.5)]    # bars r 135.5 at Y 74..94
ELEC = 109.5                      # electronics bay: C4 union of the PSU (215 x 115 + 2) and its removal path, Y < -8
TERM = (109.5, 129.5, -100.0, 0.0, -50.0, Y_BACK)   # PSU terminal end (+X) wiring room, E DEFAULT
CHANNEL = (5.0, 4.0, 58.0)        # wire channel half width (per quadrant), depth (Y -8..-4), length along each joint


# ------------------------------------------------------------------ primitives
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cylY(x, z, y0, y1, r, n=64):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])

def cylX(y, z, x0, x1, r, n=48):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])

def cylZ(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

def union(ms):
    out = ms[0]
    for m in ms[1:]: out = out + m
    return out

def polar(r, deg):
    a = math.radians(deg); return r * math.cos(a), r * math.sin(a)

Q0 = box(0, E_OUT, Y_MIN, TIE_Y, -E_OUT, 0)             # the quadrant's whole box


def rot_xz(v, k):
    """rotate (x, y, z) by k * 90 deg about Y, (x, z) -> (x cos - z sin, x sin + z cos)"""
    x, y, z = v; c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    return (x * c - z * s, y, x * s + z * c)


# ------------------------------------------------------------------ drawn features (Q0)
def foot_seats():
    """two pad seats on the corner knuckle: bottom (face -Z, the real back foot in position 0) and its diagonal image
    (face +X, the real foot in position 3). Each: land 36 x 36, pocket 30.6 x 1, 4-mm floor with the stem hole, a
    boss round the barb cavity."""
    x, y = FOOT; h = LAND / 2
    bot = box(x - h, E_OUT, y - h, y + h, -E_OUT, -E_OUT + 5) + cylZ(x, y, -E_OUT + 5, -E_OUT + 5 + CAV_H, CAV_D / 2 + 3)
    side = box(E_OUT - 5, E_OUT, y - h, y + h, -E_OUT, -x + h) + cylX(y, -x, E_OUT - 5 - CAV_H, E_OUT - 5, CAV_D / 2 + 3)
    cut_b = box(x - POCKET / 2, x + POCKET / 2, y - POCKET / 2, y + POCKET / 2, -E_OUT - 1, -E_OUT + 1) \
        + cylZ(x, y, -E_OUT, -E_OUT + 5.01, SNAP_D / 2) + cylZ(x, y, -E_OUT + 5, -E_OUT + 5 + CAV_H, CAV_D / 2)
    cut_s = box(E_OUT - 1, E_OUT + 1, y - POCKET / 2, y + POCKET / 2, -x - POCKET / 2, -x + POCKET / 2) \
        + cylX(y, -x, E_OUT - 5.01, E_OUT, SNAP_D / 2) + cylX(y, -x, E_OUT - 5 - CAV_H, E_OUT - 5, CAV_D / 2)
    return bot + side, cut_b + cut_s, dict(bottom=box(x - PAD / 2, x + PAD / 2, y - PAD / 2, y + PAD / 2, -E_OUT, -E_OUT + 5),
                                           side=box(E_OUT - 5, E_OUT, y - PAD / 2, y + PAD / 2, -x - PAD / 2, -x + PAD / 2))


def flange_seat():
    q = box(0, SEAT_R + 1, -SEAT_T, 0, -SEAT_R - 1, 0)
    seat = (cylY(0, 0, -SEAT_T, 0, SEAT_R, 128) ^ q)
    bx, bz = polar(R_BOLT, -45)
    boss = cylY(bx, bz, Y_BACK, 0, M5_BOSS_D / 2)
    cut = cylY(bx, bz, Y_BACK - 1, 1, D_M5 / 2) + cylY(bx, bz, Y_BACK - 1, Y_BACK + 5, M5_CB / 2) \
        + cylY(0, 0, Y_MIN - 1, TIE_Y + 1, WIRE_R)
    return seat, boss, cut, (bx, bz)


def psu_bosses():
    b = [cylY(x, z, Y_BACK, 0, BOSS_D / 2) for x, z in PSU_HOLES]
    c = [cylY(x, z, Y_BACK - 1, 1, M4_D / 2) + cylY(x, z, -5, 1, M4_CB / 2) for x, z in PSU_HOLES]
    return b, union(c)


def joint_lands():
    """glue + dowel lands on the two joint planes, beyond the wire channel (r 60..76), Y -8..0"""
    a = box(0, 10, Y_BACK, 0, -76, -60); b = box(60, 76, Y_BACK, 0, -10, 0)
    ca = cylX(Y_BACK / 2, -68, -1, DOWEL_L, DOWEL_D / 2); cb = cylZ(68, Y_BACK / 2, -DOWEL_L, 1, DOWEL_D / 2)
    return [a, b], ca + cb


def tie_tab(y_face=TIE_Y):
    d, t, hole, head = TIE_TAB
    x, z = polar(TIE_R, TIE_ANG)
    tab = cylY(x, z, y_face - t, y_face, d / 2)
    cut = cylY(x, z, y_face - t - 1, y_face + 1, hole / 2) + cylY(x, z, y_face - t - 8, y_face - t, head / 2)
    return tab, cut, (x, z)


# ------------------------------------------------------------------ keep-outs (Q0)
def keepouts(y_face=TIE_Y):
    k = []
    for y0, y1, r in SWEEP:                                            # the turning arm, Y > 0
        k.append(cylY(0, 0, y0, min(y1, y_face) + (1 if y1 >= y_face else 0), r, 160))
    k.append(box(-1, ELEC, Y_MIN - 1, Y_BACK, -ELEC, 1))                # electronics bay + PSU removal path
    x0, x1, z0, z1, y0, y1 = TERM; k.append(box(x0, x1, y0, y1, z0, z1))
    w, d, L = CHANNEL                                                   # wire channel along both joints
    k.append(box(-1, w, Y_BACK - 0.01, Y_BACK + d, -L, 1)); k.append(box(-1, L, Y_BACK - 0.01, Y_BACK + d, -w, 1))
    return union(k) ^ Q0


def skin(drawn):
    """no-flat-cut-off rule: 2 mm empty skin on the outer envelope faces (not on the joint planes, which are the bed /
    glue faces). Drawn bodies are exempt."""
    s = Q0 - box(-1, E_OUT - 2.13, Y_MIN + 2.13, TIE_Y - 2.13, -(E_OUT - 2.13), 1)   # off the cell centres (ray voxeliser)
    return s - drawn


def parts(y_face=TIE_Y):
    seats, seat_cut, pads = foot_seats()
    fseat, m5, fcut, bolt = flange_seat()
    psu, pcut = psu_bosses()
    lands, dcut = joint_lands()
    tab, tcut, tie = tie_tab(y_face)
    drawn = union([seats, fseat, m5, *psu, *lands, tab])
    cuts = union([seat_cut, fcut, pcut, dcut, tcut])
    ko = keepouts(y_face)
    cuts_all = cuts + (ko - drawn) + skin(drawn)
    regions = dict(pad_bottom=pads["bottom"], pad_side=pads["side"], m5=m5, seat=fseat, tab=tab, lands=lands,
                   psu=[cylY(x, z, Y_BACK, 0, BOSS_D / 2) for x, z in PSU_HOLES])
    return dict(drawn=drawn, cuts=cuts_all, cuts_exact=cuts, keepout=ko, env=Q0, regions=regions, bolt=bolt, tie=tie)


# ------------------------------------------------------------------ loads (box frame -> Q0)
G = 9.81
LAND_N, LAT_N = 250.0, 140.0      # landing peak per back foot: normal 249 N, lateral 140 N, F (stand_footload.py,
                                  # node wheels + funnel contact 0.010 s; rigid 0.004 s gives 697 / 363, E)
HAND_N = 50.0                     # hand on a foot sideways (Spark)
PSU_KG, PSU_G = 0.76, 3.0         # PSU 760 g C, 3 g (Spark)
PSU_DY = 15.0                     # PSU CG 15 off its mounting face, C (30 / 2)
DRIVE_NM = 2.4                    # arm drive through the flange, C
BOLT_PULL, BOLT_SHEAR, SEAT_N, SEAT_SH = 80.0, 25.0, 50.0, 25.0   # stub cantilever (2.7 kg at Y 63, x 3 g), C precedent stub tile v1


def load_cases():
    """[(name, region key, (Fx, Fy, Fz) in Q0, region index or None)]. Single-direction cases are sign-free (linear),
    combined ones are listed with every sign that matters."""
    cases = []
    # position 0: Q0 itself, its bottom seat is a floor foot. position 3 (Q(X-,Z-)): its floor foot is Q0's side seat.
    for k, seat in ((0, "pad_bottom"), (3, "pad_side")):
        kk = (-k) % 4                                                 # box frame -> Q0
        for lat in ((LAT_N, 0, 0), (-LAT_N, 0, 0), (0, LAT_N, 0), (0, -LAT_N, 0)):
            v = rot_xz((lat[0], lat[1], LAND_N), kk)
            cases.append((f"landing p{k} lat {lat[0]:+.0f}/{lat[1]:+.0f}", seat, v, None))
        for v in ((HAND_N, 0, 0), (0, HAND_N, 0)):
            cases.append((f"hand p{k} {'xy'[v[1] > 0]}", seat, rot_xz(v, kk), None))
    # PSU 3 g, position 0 on bosses 0, 1; position 3 on bosses 2, 1 (the SKR carrier in positions 1 / 2 lands on the same
    # bosses with less mass). Half the PSU per quadrant, plus the couple of its CG 15 mm off the face.
    Ft = PSU_KG * G * PSU_G                                            # 22.4 N on the whole PSU, per axis
    ctr = (0.0, -50.0)                                                 # PSU centre (X, Z), DEFAULT
    for k, bos in ((0, (0, 1)), (3, (2, 1))):
        kk = (-k) % 4
        for ax in range(3):
            v = [0.0, 0.0, 0.0]; v[ax] = Ft / 4; vq = rot_xz(v, kk)    # in-plane share per boss (4 bosses)
            for b in bos:
                bx_, _, bz_ = rot_xz((PSU_HOLES[b][0], 0, PSU_HOLES[b][1]), k)   # boss in the box frame
                if ax == 1: fy = 0.0
                else:                                                  # couple of the CG 15 off the face -> +-Y
                    d = (bx_ - ctr[0]) if ax == 0 else (bz_ - ctr[1])
                    fy = -Ft * PSU_DY * d / (4 * (75.0 ** 2 if ax == 0 else 25.0 ** 2))
                cases.append((f"psu 3g p{k} {'xyz'[ax]} b{b}", "psu", (vq[0], vq[1] + fy, vq[2]), b))
    # flange: weight shear (gravity turns with the position: both in-plane axes), bolt pull, drive torque, seat
    bx, bz = polar(R_BOLT, -45); t = (-bz / R_BOLT, 0, bx / R_BOLT)      # tangential unit at the bolt
    cases += [("flange bolt pull", "m5", (0, -BOLT_PULL, 0), None),
              ("flange bolt shear x", "m5", (BOLT_SHEAR, 0, 0), None),
              ("flange bolt shear z", "m5", (0, 0, BOLT_SHEAR), None),
              ("arm drive 2.4 N m", "m5", tuple(DRIVE_NM * 1000 / (4 * R_BOLT) * c for c in t), None),
              ("flange seat push", "seat", (0, SEAT_N, 0), None),
              ("flange seat shear x", "seat", (SEAT_SH, 0, 0), None),
              ("flange seat shear z", "seat", (0, 0, SEAT_SH), None)]
    return cases


def _merge_psu(cases):
    """the two bosses of one PSU case act together: returns [(name, [(region key, idx, vec), ...])]"""
    out = {}
    for name, key, v, idx in cases:
        base = name.rsplit(" b", 1)[0] if key == "psu" else name
        out.setdefault(base, []).append((key, idx, v))
    return list(out.items())


if __name__ == "__main__":
    import trimesh
    P = parts()
    tmesh = lambda m: trimesh.Trimesh(np.asarray(m.to_mesh().vert_properties)[:, :3], np.asarray(m.to_mesh().tri_verts))
    os.makedirs("out", exist_ok=True)
    tmesh(P["drawn"] - P["cuts_exact"]).export("out/spider_drawn.stl"); tmesh(P["keepout"]).export("out/spider_keepout.stl")
    bb = Q0.bounding_box()
    print(f"quadrant box {[round(bb[i + 3] - bb[i], 1) for i in range(3)]} (A1 mini 180 cube)")
    print(f"drawn {(P['drawn'] - P['cuts_exact']).volume() / 1000 * 1.27:.1f} g PETG, bodies {len((P['drawn']).decompose())}")
    print(f"allowed {(Q0 - P['cuts']).volume() / 1000:.0f} cm3 of {Q0.volume() / 1000:.0f}")
    for n, cs in _merge_psu(load_cases()):
        print(f"  {n:28s}", "; ".join(f"{k}{'' if i is None else i} ({v[0]:+.1f}, {v[1]:+.1f}, {v[2]:+.1f})" for k, i, v in cs))
    print("tie tab at", [round(c, 2) for c in P["tie"]], "flange bolt at", [round(c, 2) for c in P["bolt"]])
