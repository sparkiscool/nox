"""Scout v2 guard + box funnel catch layout search (props on top, bottom pack).

Drone axes: x nose, y left, z up; plate top z 22 (dimensions.md C).
Tags in comments: P measured, C CAD, E estimate, F frame model.

Concepts compared:
  A  pyramid: 4 pinwheel rods from the lower ring down a 35 deg face to a foot frame
     (current interface, guard v1 / funnel v3), recomputed with props on top.
  B  collar: the lower ring flats themselves rest on the same 35 deg faces; the funnel
     is only the band of faces from just under the ring seat to the mouth; the bottom
     is open, so pack, stack and both drones' bellies hang through.
Objective: smallest funnel (height, then mouth), then lightest guard + funnel.
"""
import math, itertools, json

A = math.radians(34.99)          # face slope, shared catch interface (C)
TA = math.tan(A)

# ---- drone inputs -------------------------------------------------------------
PROP_Z = 40.5                    # prop plane, E (D12 CHECK)
MOTOR = 54.1                     # motor centres (+-54.1, +-54.1), P
DISK_R = 44.45                   # 3.5 in prop, P
KEEP = 12.0                      # prop keep-out, README
DEF_1MS = 9.9                    # ring deflection in the worst 1 m/s hit (F, guard v1: 14.4 static -> 4.5 left)
R_LOW, R_UP = 1.25, 1.0          # lower ring 2.5, upper 2.0 solid carbon (C)
Z_LOW_RING = PROP_Z - 8          # lower ring centre, same offset to the blades as v1 (C)
Z_UP_RING = PROP_Z + 8
CAPTURE = 40.0                   # hover offset at the gate (E, D7)
YAW = math.radians(5)            # (E)
TILT = math.radians(2)           # (E)
MOUTH_MARGIN = 4.0               # mouth top above the first-touch height (v3: 32 vs 28)
FACE_G_PER_MM2 = 70.9 / 15622    # v3 half panel 70.9 g (M) over its face area (C)

# Scout parts below or around the ring: (name, x0, x1, y0, y1, z0, z1)  drone coords
# Bottom layout is open (Problem 31): DEFAULT cartridge on the axis right under the FC.
CART_TOP = 9.5 - 1.0 - 2.0       # FC lowest z 9.5 (P/CHECK), 1 gap, 2 TPU preload (E)
CART_BOT = CART_TOP - 48.0       # cartridge v2 48 deep in drone z (C)
SCOUT = [
    ("arm plate + motors", -63.6, 63.6, -63.6, 63.6, 18.5, 37.0),   # motor r 9.5 E
    ("FC", -18, 18, -18, 18, 9.5, 18.5),
    ("cartridge (DEFAULT under FC)", -39.4, 39.4, -18.9, 18.9, CART_BOT, CART_TOP),
    ("Radxa + flow beside pack (E)", -32.5, 32.5, -47, 47, CART_BOT + 6, CART_TOP),
    ("camera at the nose (E)", 78, 95, -19, 19, 8, 26),
]
# what must pass down through the funnel opening to the swap wheel
PATH_SCOUT = (88 / 2, 46 / 2)            # box casing pack path X 88 x Y 46 (C)
OVERSEER_PACK = (155 + 10, 48 + 8)       # 4S 6000 + cartridge walls, E (D10 CHECK)
PATH_OVERSEER = (OVERSEER_PACK[0] / 2, OVERSEER_PACK[1] / 2)


def ring_ok(F, c):
    """static clearance rod surface to disk (flats, chamfer) and the 1 m/s deflected margin"""
    flat = F - MOTOR - DISK_R - R_LOW
    cham = (F + c - 2 * MOTOR) / math.sqrt(2) - DISK_R - R_LOW
    s = min(flat, cham)
    return s, s - DEF_1MS


def face_half(z, F, z_seat):
    """face half-width at height z; plane tangent to the lower-ring flats (axis F, z_seat)"""
    # axis sits 1.25 off the plane along its normal: plane passes through the axis shifted
    h_axis = F + R_LOW / math.sin(A)              # plane half-width at the axis height
    return h_axis + (z - z_seat) / TA


def contact_z(z_seat):
    return z_seat - R_LOW * math.cos(A)           # rod-to-face contact line height


def part_margin(part, F, z_seat):
    """smallest perpendicular gap from a part box (yawed 5, tilted 2) to the 4 faces"""
    _, x0, x1, y0, y1, z0, z1 = part
    worst = 1e9
    for x, y, z in itertools.product((x0, x1), (y0, y1), (z0, z1)):
        for sgn in (1, -1):                        # yaw both ways
            xr = x * math.cos(YAW) - sgn * y * math.sin(YAW)
            yr = sgn * x * math.sin(YAW) + y * math.cos(YAW)
            zr = z + max(abs(xr), abs(yr)) * math.sin(TILT)   # tilt lifts nothing, lowers far side
            zr2 = z - max(abs(xr), abs(yr)) * math.sin(TILT)
            for zz in (zr, zr2):
                h = face_half(zz, F, z_seat)
                worst = min(worst, (h - max(abs(xr), abs(yr))) * math.sin(A))
    return worst


def concept_B(F, c, edge_slant):
    z_seat = Z_LOW_RING
    zc = contact_z(z_seat)
    z_edge = zc - edge_slant * math.sin(A)        # lower edge of the collar
    h_edge = face_half(z_edge, F, z_seat)
    z_first = z_seat + CAPTURE * TA               # first touch of a ring 40 off
    z_top = z_first + MOUTH_MARGIN
    h_top = face_half(z_top, F, z_seat)
    H = z_top - z_edge
    slant = H / math.sin(A)
    area = 4 * (h_edge + h_top) * slant           # 4 trapezoids, full widths
    # parts: only those whose z reaches the face band can touch it; everything below
    # the edge must fit the opening (2 deg tilt, 5 deg yaw)
    margins = {}
    for p in SCOUT:
        if p[6] >= z_edge:                        # reaches the face band
            margins[p[0]] = part_margin(p, F, z_seat)
        else:
            ext = max(abs(p[1]), abs(p[2]), abs(p[3]), abs(p[4]))
            margins[p[0]] = h_edge - ext * (math.cos(YAW) + math.sin(YAW))
    for name, (hx, hy) in (("Scout pack path", PATH_SCOUT), ("Overseer pack path", PATH_OVERSEER)):
        ext = max(hx, hy) * math.cos(YAW) + min(hx, hy) * math.sin(YAW)
        margins[name] = h_edge - ext
    return dict(concept="B collar", F=F, c=c, edge_slant=edge_slant, z_seat=z_seat,
                z_edge=round(z_edge, 2), z_top=round(z_top, 2), H=round(H, 1),
                mouth=round(2 * h_top, 1), opening=round(2 * h_edge, 1),
                funnel_g=round(area * FACE_G_PER_MM2, 0),
                guard_rods_removed_g=4.1 + 1.8, margins=margins)


def concept_A(F, c, foot_half):
    z_seat = Z_LOW_RING
    z_foot = z_seat - (F - foot_half) * TA        # rods parallel to their faces
    z_first = z_seat + CAPTURE * TA
    z_top = z_first + MOUTH_MARGIN
    h_top = face_half(z_top, F, z_seat)
    H = z_top - (z_foot - 3)
    slant = H / math.sin(A)
    h_bot = face_half(z_foot - 3, F, z_seat)
    area = 4 * (h_bot + h_top) * slant
    feasible_overseer = foot_half - 3 >= max(PATH_OVERSEER)
    return dict(concept="A pyramid", F=F, c=c, foot_half=foot_half, z_foot=round(z_foot, 1),
                H=round(H, 1), mouth=round(2 * h_top, 1), funnel_g=round(area * FACE_G_PER_MM2, 0),
                pack_hangs_below_foot=round(z_foot - CART_BOT, 1),
                overseer_pack_passes=feasible_overseer)


def search():
    best = None
    rows = []
    for F in [x / 2 for x in range(220, 241)]:          # flats 110 .. 120
        for c in [x / 2 for x in range(150, 171)]:      # chamfer 75 .. 85
            s, d = ring_ok(F, c)
            if s < KEEP or d < 2.0:
                continue
            for e in (3, 4, 5, 6, 8):                   # collar edge below contact, slant
                r = concept_B(F, c, e)
                if min(r["margins"].values()) < 3.0:    # every part >= 3 off the faces / edge
                    continue
                key = (r["H"], r["mouth"], r["funnel_g"])
                rows.append((key, r))
                if best is None or key < best[0]:
                    best = (key, r, s, d)
    return best, len(rows)


if __name__ == "__main__":
    (key, r, s, d), n = search()
    print(f"collar designs passing: {n}")
    print(json.dumps(r, indent=1))
    print(f"ring static clearance {s:.1f}, deflected margin {d:.1f} (E from F)")
    # comparison at the chosen outline
    print("\npyramid (A) at the same outline:")
    for fh in (20, 48, 88):
        print(json.dumps(concept_A(r["F"], r["c"], fh)))
    print("\nv3 funnel for reference: H 103 (z -79..24), mouth 325, panels 567 g + hub 219 + blocks ~300 (M)")
    print(f"cartridge bottom z {CART_BOT:.1f} (E)")
