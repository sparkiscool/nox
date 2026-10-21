"""Box swap arm (v1..v4): drawn bodies for every part (interfaces and must-haves only), keep-outs and envelopes, in wheel
coordinates (arm_layout.py), plus the purely drawn small parts and an assembly collision check.
Generated parts (hub, carriage, bay) get their (drawn, cuts, env) here and are run by arm_run_<part>.py through the
noxopt pipeline.  python arm_cad.py  -> drawn parts to out/, checks printed.
v4 (16 Oct): head_parts / gripper_v4 / fingers_and_racks / pinion_u / servo_v4 (cartridge long side along the axle),
bay_module_parts / bay_strip (universal bay modules bolted long side to long side, v4c 17 Oct; balance leads dropped 01:18). bay_module_parts imports
v2/s2_cartridge.py (receiver()): copy it beside this file. bay_parts / bay_ring_parts / pack_path are v1 / v3 history.
Head belt clamp v2 (17 Oct 15:40): belt_clamp_v2_head.py (loop-and-pin, belt_clamp_v2.py); _glide_clamp imports it. v5 wiring entry + zip anchors in head_parts / gripper_v4.
"""
import os, math
import numpy as np
import manifold3d as mf
import trimesh
import arm_layout as L

OUT = "out"; os.makedirs(OUT, exist_ok=True)
S2 = math.sqrt(2)
SERVO_INSERT_D, SERVO_INSERT_L = 4.0, 5.7   # M3 heat-set (E, CHECK on hand), as the belt clamp's
CLAMP_SLOT = float(os.environ.get("NOX_SLOT", "2.0"))   # belt clamp v2 slot width: the coupon's pick (1.8 / 2.0 / 2.2), DEFAULT 2.0;
                                                         # an exact cut at build time (the layout keeps out of 2.2), so NOX_SLOT=2.2 python3 ... rebuilds
# head v5 wiring (nox/box-wiring.md, 17 Oct): the servo ribbon folds in the bar's top-face T-slot at X -10 (A; B +10) and comes
# up through the glide's top skin at u WIRE_ENTRY_DU in a Ø WIRE_ENTRY_D channel; a zip-tie fin beside the hole anchors it;
# a zip-tie tab on the servo plate's far end holds it at the servo.
WIRE_ENTRY_X, WIRE_ENTRY_DU, WIRE_ENTRY_D = -10.0, 8.0, 4.5
ZIP_SLOT = (3.4, 1.6)                       # zip tie 2.5 x 1.0 (E): slot length x width


# ------------------------------------------------------------------ primitives
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cylY(x, z, y0, y1, r, n=64):
    """cylinder along Y (the axle direction)"""
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])

def cylX(y, z, x0, x1, r, n=48):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])

def cylZ(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])

def hull(pts):
    return mf.Manifold.hull_points(np.asarray(pts, np.float64))

def diamond(xc, zc, half, y0, y1):
    """square of half-diagonal `half` rotated 45 deg in XZ (motor bodies seen along the axle)"""
    return hull([[xc + half, y, zc] for y in (y0, y1)] + [[xc - half, y, zc] for y in (y0, y1)] +
                [[xc, y, zc + half] for y in (y0, y1)] + [[xc, y, zc - half] for y in (y0, y1)])

def rsquare(m, half, y0, y1):
    """square of half side `half` centred (m['xc'], m['zc']) rotated m['rot'] deg in XZ, extruded Y y0..y1"""
    return box(-half, half, y0, y1, -half, half).rotate([0, -m["rot"], 0]).translate([m["xc"], 0, m["zc"]])   # manifold rotates Z toward X: minus to match rot_pt

def rot_pt(m, dx, dz):
    c, s = math.cos(math.radians(m["rot"])), math.sin(math.radians(m["rot"]))
    return m["xc"] + dx * c - dz * s, m["zc"] + dx * s + dz * c

def rotY(m, deg):
    return m.rotate([0, deg, 0])

def tm(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)

def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))

def vol(m): return m.volume()

def report(name, m, dens=1.27):
    bb = m.bounding_box(); ext = [round(bb[i + 3] - bb[i], 1) for i in range(3)]
    print(f"{name:34s} {vol(m) / 1000 * dens:6.1f} g  bodies {m.num_vert() and len(m.decompose())}  bbox {ext}  "
          f"fits180 {'yes' if max(ext) <= 180 else 'NO'}")


# ------------------------------------------------------------------ gear teeth (printed, module 1)
def rack_teeth(y0, y1, x0, x1, u_pitch_line, up, m=1.0, phase_y=None):
    """module-1 trapezoid teeth along Y on a rack: tooth tips at u_pitch_line +- (m) in the `up` direction
    (up=+1: teeth point to +u). Tooth profile 20 deg: base 1.57+, tip ~0.9 wide. Returns a Manifold of the teeth."""
    p = math.pi * m; teeth = None
    yc = (y0 + y1) / 2 if phase_y is None else phase_y  # a GAP is centred on the pinion axis (a pinion tooth points at it)
    y = yc + p / 2 - p * math.ceil((yc + p / 2 - y0) / p)
    while y < y1 - p / 4:
        if y < y0 + p / 4: y += p; continue
        base = p / 2 - 0.2; tip = p / 2 - 0.2 - 2 * m * math.tan(math.radians(20))   # 0.2 backlash for the print
        u0 = u_pitch_line - up * m; u1 = u_pitch_line + up * 0.7 * m
        t = hull([[x, y - base / 2, u0] for x in (x0, x1)] + [[x, y + base / 2, u0] for x in (x0, x1)] +
                 [[x, y - tip / 2, u1] for x in (x0, x1)] + [[x, y + tip / 2, u1] for x in (x0, x1)])
        teeth = t if teeth is None else teeth + t
        y += p
    return teeth

def pinion(n=12, m=1.0, x0=-4.0, x1=2.0, spline_d=5.9):
    """12 T module 1 pinion along X with a 25T-spline-sized bore (printed, press on the servo horn shaft; DEFAULT:
    bore 5.9 for the DS3218 / MG995 25T spline Ø5.8, CHECK)"""
    rp = n * m / 2; ra = rp + 0.7 * m; rr = rp - 1.25 * m
    pts = []
    for k in range(n):
        a0 = 2 * math.pi * k / n; half_t = math.pi / n / 2
        for r, da in ((rr, -2 * half_t + 0.02), (rp, -half_t * 0.75), (ra, -half_t * 0.4), (ra, half_t * 0.4),
                      (rp, half_t * 0.75), (rr, 2 * half_t - 0.02)):
            pts.append([r * math.cos(a0 + da), r * math.sin(a0 + da)])
    poly = mf.CrossSection([pts])
    g = mf.Manifold.extrude(poly, x1 - x0).rotate([0, 90, 0]).translate([x0, 0, 0])   # axis along X
    return g - cylX(0, 0, x0 - 1, x1 + 1, spline_d / 2)


# ================================================================== STUB AXLE + FIXED SUN POST (fixed, drawn)
def stub_v2():
    """v2 (17 Oct), kept for history: Ø90 flange, 4 x M5 on r 38 at 45 deg for the wood upright, solid shaft, Ø24 bore."""
    fl = cylY(0, 0, L.Y_WALL, L.Y_FLANGE1, 45.0)
    for a in (45, 135, 225, 315):
        x, z = L.part_angle_pos(38.0, a); fl -= cylY(x, z, -1, 7, 2.7)          # 4 x M5 to the spider, one per quadrant
    sh = cylY(0, 0, L.Y_FLANGE1, L.Y_FLANGE1 + 2.0, 27.0)                        # shoulder for bearing 1
    ax = cylY(0, 0, L.Y_FLANGE1, L.Y_STUB_END, L.BRG_ID / 2 - 0.05)             # bearing seat Ø49.9 (E: press fit, sand)
    body = fl + sh + ax
    body -= cylY(0, 0, -1, L.Y_STUB_END + 1, 12.0)                               # Ø24 through bore (lead bundle)
    for z in L.POST_BOLT_Z: body -= cylY(0, z, L.Y_STUB_END - 8, L.Y_STUB_END + 1, L.INSERT_R)   # M3 inserts: sun post
    return body


def _xz_section(cs, y0, y1):
    """a 2D outline in (X, Z) extruded along +Y from y0 to y1 (CrossSection v = -Z, as cylY's rotate)."""
    return mf.Manifold.extrude(cs, y1 - y0).rotate([-90, 0, 0]).translate([0, y0, 0])


def _circle(x, z, r, n=96):
    return mf.CrossSection.circle(r, n).translate([x, -z])


def stub_plate_outline(pad_d=None, fillet=None):
    """spider plate outline (X, -Z): the hub disc (Ø54: the shoulder Ø52.8 stands on it, 0.6 step) + one lobe per post hole (pad circle hulled to a circle of the same
    size at r 22 on its ray), concave corners closed with a fillet."""
    pad_r = (pad_d or L.STUB_PAD_D) / 2; f = L.STUB_FILLET if fillet is None else fillet
    cs = _circle(0, 0, L.STUB_SHOULDER_R + 0.6, 192)                            # 0.6 step out round the shoulder
    for x, z in L.STUB_HOLES:
        r = math.hypot(x, z); ux, uz = x / r, z / r
        cs += mf.CrossSection.batch_hull([_circle(x, z, pad_r), _circle(22.0 * ux, 22.0 * uz, pad_r)])
    J = mf.JoinType.Round
    return cs.offset(f, J, circular_segments=96).offset(-f, J, circular_segments=96)


def stub_v3_search():
    """stub v3 sizing (C): the lightest pad that meets Problem 46 (pad >= Ø12, web >= 2.0 round the hole), keeps >= 2.0 of
    wall round the button-head counterbore, seats inside the post's 40 face (X +-20) and stays inside the hub sleeve (r 40.1)
    by 1.0; and the lead channel direction (+X, -X, +Z, -Z) that clears every lobe by >= 2.0 (shortest run out = tie)."""
    hole_r = max(math.hypot(x, z) for x, z in L.STUB_HOLES)
    lo = max(12.0, 2 * L.STUB_HOLE_R + 4.0, L.STUB_CB[0] + 4.2)                 # Problem 46, web 2.0, counterbore wall 2.0 + 0.1 facets
    hi = min(2 * (20.0 - max(abs(x) for x, _ in L.STUB_HOLES)), 2 * (40.1 - 1.0 - hole_r))
    pad = math.ceil(lo * 5 - 1e-9) / 5                                          # 0.2 steps: Ø14.4 (wall 2.1)
    dirs = {}
    for name, (dx, dz) in {"+X": (1, 0), "-X": (-1, 0), "+Z": (0, 1), "-Z": (0, -1)}.items():
        w = L.STUB_LEAD["w"] / 2
        ch = mf.CrossSection.square([60.0, 2 * w]).translate([0, -w])           # along +u from the axis, +-w across
        ang = math.degrees(math.atan2(-dz, dx))                                 # outline v = -Z
        ch = ch.rotate(ang)
        lobes = None
        for x, z in L.STUB_HOLES:
            r = math.hypot(x, z); ux, uz = x / r, z / r
            lb = mf.CrossSection.batch_hull([_circle(x, z, pad / 2), _circle(22.0 * ux, 22.0 * uz, pad / 2)])
            lobes = lb if lobes is None else lobes + lb
        gap = _xz_section(ch, 0, 1).min_gap(_xz_section(lobes - _circle(0, 0, 24.9, 192), 0, 1), 10.0)
        dirs[name] = round(gap, 2)
    return dict(pad_d=pad, pad_range=(lo, hi), lead_gap_to_lobes=dirs)


def stub():
    """v3 (18 Oct 2026, arm chat; stand v5e: one 2040 post, nox/box-stand.md). Same Ø49.9 bearing seats (Y 6..40), shoulder
    for bearing 1 and Ø24 bore through the plate as v2; flange cut to a 4-lobe spider on the post's two slots, 4 x M5 at
    (X +-10, Z +-30) with button heads sunk flush in Ø10.2 counterbores (they sit inside bearing 1's radius: head top Y 6,
    the bearing face Y 8); the shaft hollowed to Ø40 between the plate and the end, two full-height ribs carry the sun post's
    M3 inserts at Z +-21; a 10 x 6 lead channel in the plate's back face runs the bundle out toward -X between the plate and
    the post face. One body. Print: plate face (Y 0) on the bed, no supports (the channel roof is a 10 bridge)."""
    T = L.STUB_PLATE_T
    seat_r = L.BRG_ID / 2 - 0.05                                                 # Ø49.9 (E: press fit, sand)
    plate = _xz_section(stub_plate_outline(), 0.0, T)                           # sharp plate edges: the slicer's elephant-foot compensation
    sh = cylY(0, 0, T - 0.01, T + 2.0, L.STUB_SHOULDER_R, 192)                   # bearing 1 abutment Y 6..8
    lead_in = 0.8
    ax = cylY(0, 0, T - 0.01, L.Y_STUB_END - lead_in, seat_r, 192) + \
        mf.Manifold.cylinder(lead_in, seat_r, seat_r - lead_in, 192).rotate([-90, 0, 0]).translate([0, L.Y_STUB_END - lead_in, 0])
    body = plate + sh + ax
    bore = cylY(0, 0, T, L.Y_STUB_END + 1, L.STUB_BORE_R, 160)                   # Ø40 hollow under the seats
    for z in L.POST_BOLT_Z: bore -= cylY(0, z, T - 1, L.Y_STUB_END + 2, L.STUB_RIB_R, 48)   # insert ribs stay
    body -= bore
    body -= cylY(0, 0, -1, T + 0.5, 12.0, 96)                                    # Ø24 bore through the plate (lead bundle)
    for z in L.POST_BOLT_Z: body -= cylY(0, z, L.Y_STUB_END - 8, L.Y_STUB_END + 1, L.INSERT_R)   # M3 heat-set: sun post
    cbd, cbt = L.STUB_CB
    for x, z in L.STUB_HOLES:
        body -= cylY(x, z, -1, T + 1, L.STUB_HOLE_R, 48)
        body -= cylY(x, z, T - cbt, T + 3, cbd / 2, 64)
    w = L.STUB_LEAD["w"] / 2
    body -= box(-60.0, 0.0, -1.0, T, -w, w) if L.STUB_LEAD["x_dir"] < 0 else box(0.0, 60.0, -1.0, T, -w, w)
    return body


def stub_bolts():
    """the 4 M5 x 8 button heads (ISO 7380: Ø9.5 x 2.75, E) seated in the counterbores, shanks into the post: for the sweep."""
    cbd, cbt = L.STUB_CB; T = L.STUB_PLATE_T; m = None
    for x, z in L.STUB_HOLES:
        b = cylY(x, z, T - cbt, T, 4.75, 32) + cylY(x, z, T - cbt - 8.0, T - cbt, 2.5, 24)
        m = b if m is None else m + b
    return m


def sun_post():
    """fixed sun post (hub v4, 17 Oct): flange on the stub end (clamps bearing 2's inner race, 2 x M3 at Z +-21), a column
    through the hub's Ø45.5 hole to Y 54 carrying the 80T on 3 x M3 heat-set inserts on Ø36, a Ø11.9 spigot locating the
    80T's bore (drilled Ø12), and a Ø8.5 lead bore through it all. Two open flutes down the column reach the stub bolts.
    Print: flange face (Y 40) on the bed, no supports."""
    y0, y1 = L.POST_FL_Y
    c = 0.6                                                                      # edge chamfers (C)
    fl = cylY(0, 0, y0, y1 - c, L.POST_FL_R) + \
        mf.Manifold.cylinder(c, L.POST_FL_R, L.POST_FL_R - c, 96).rotate([-90, 0, 0]).translate([0, y1 - c, 0])
    col = cylY(0, 0, y1 - 0.01, L.POST_Y1 - c, L.POST_R, 96) + \
        mf.Manifold.cylinder(c, L.POST_R, L.POST_R - c, 96).rotate([-90, 0, 0]).translate([0, L.POST_Y1 - c, 0])
    cs = 1.0                                                                     # spigot lead-in (into the 80T bore)
    spg = cylY(0, 0, L.POST_Y1 - 0.01, L.POST_SPIGOT_Y1 - cs, L.POST_SPIGOT_R) + \
        mf.Manifold.cylinder(cs, L.POST_SPIGOT_R, L.POST_SPIGOT_R - 0.6, 48).rotate([-90, 0, 0]).translate([0, L.POST_SPIGOT_Y1 - cs, 0])
    body = fl + col + spg
    body -= cylY(0, 0, y0 - 1, L.POST_SPIGOT_Y1 + 1, L.POST_BORE_R)               # lead bore
    for z in L.POST_BOLT_Z:
        body -= cylY(0, z, y0 - 1, y1 + 1, 1.7)                                  # M3 clearance into the stub
        body -= cylY(0, z, y1, L.POST_Y1 + 1, L.POST_FLUTE_R)                    # access flute (open to the side)
    for a in L.SUN_BOLT_ANG:
        x, z = L.part_angle_pos(L.SUN_BC_R, a)
        body -= cylY(x, z, L.POST_Y1 - 6.0, L.POST_Y1 + 1, L.INSERT_R)           # 80T web: M3 heat-set, 6 deep
    return body

stub_cap = sun_post                                                              # v1..v3 name: the cap is retired, the post clamps the race


def axle_sun_bodies():
    """the fixed sun and the arm-side axle drive bodies the hub must clear (W, arm at phi 90): 80T (flange + boss), the
    122T loop (drawn 1.2 thick outward of the pitch surfaces, as arm_kinematics), the axle 20T."""
    M = L.MOTORS["axle"]; y0, y1 = L.Y_AXBELT
    p80 = cylY(0, 0, L.Y_P80[0], L.Y_P80[1], L.P80_FL / 2) + cylY(0, 0, L.Y_P80[1], 72.0, L.P80_BOSS / 2)
    outer = (cylY(0, 0, y0, y1, L.P80_OD / 2 + 1.2) + cylY(M["xc"], M["zc"], y0, y1, L.P20_OD / 2 + 1.2)).hull()
    inner = (cylY(0, 0, y0 - 1, y1 + 1, L.P80_OD / 2) + cylY(M["xc"], M["zc"], y0 - 1, y1 + 1, L.P20_OD / 2)).hull()
    p20 = cylY(M["xc"], M["zc"], y0 - 2, y1 + 2, L.P20_FL / 2)
    return dict(p80=p80, loop=outer - inner, p20=p20)


def sun_post_search(clr=None, step=0.25):
    """hub v4 clearance search (C): post column r = smallest that carries the 80T's inserts; hole r = post + clr; each
    inner bar post the closest to the axle (best for the bar's root) that stays >= clr off the sun, its boss, the loop
    and the axle 20T. Posts 12 along the bar x 32 across, Y 52..74 (as v3). Returns the numbers and the gaps found."""
    clr = L.CLR_POST if clr is None else clr
    r_p = math.ceil((L.SUN_BC_R + L.INSERT_R + L.INSERT_WALL) / step - 1e-9) * step
    r_h = r_p + clr
    obs = axle_sun_bodies(); ob = obs["p80"] + obs["loop"] + obs["p20"]
    def post(which, r):
        return box(-16, 16, 52.0, 74.0, r - 6, r + 6) if which == "A" else box(-r - 6, -r + 6, 52.0, 74.0, -16, 16)
    out = dict(post_r=r_p, hole_r=r_h, hole_d=2 * r_h)
    for which in ("A", "B"):
        r = 20.0
        while post(which, r).min_gap(ob, 5.0) < clr - 1e-6: r += step
        out[f"post_{which}"] = r
        out[f"gap_{which}"] = dict(sun=round(post(which, r).min_gap(obs["p80"], 10.0), 2), loop=round(post(which, r).min_gap(obs["loop"], 10.0), 2),
                                   p20=round(post(which, r).min_gap(obs["p20"], 30.0), 2))
    P = sun_post()
    out["post_g"] = round(P.volume() / 1000 * 1.27, 1)
    return out


# ================================================================== HUB (rotating, generated)
def hub_parts():
    """hub v4 (17 Oct): the 80T is FIXED on the sun post (planetary axle drive), so the hub carries no pulley: v3's Ø8
    stub, its 3 M3 bosses and the wire-exit tube are gone (leads twist inside the post). Ø45.5 hole through the web and
    motor plate for the post (1.0 radial), the plate held at Y <= 53 inside r 29 (1.0 under the 80T's flange), inner bar
    posts at r 35 (A) / 38.75 (B), the axle motor's seat slotted +-1 toward the axle (tensions the jig-made 122T loop).
    returns dict(drawn, cuts, env, allowed) in W coords. Print: bed at Y_HUB0 (sleeve end down), plate up, tree supports
    under the three motor seats."""
    y0, y1 = L.Y_HUB0, L.Y_HUB1
    # drawn: bearing sleeve (bore Ø80, wall 3), outer-race shoulder, web over the cavity, 3 motor seats, 4 bar posts
    sleeve = cylY(0, 0, y0, L.Y_BRG[1] + L.BRG_W, 43.0) - cylY(0, 0, y0 - 1, 41, L.BRG_OD / 2 + 0.10, 256)
    shoulder = cylY(0, 0, 40.0, 44.0, 43.0) - cylY(0, 0, 39, 45, 36.0)
    plate_c = cylY(0, 0, 48.0, 54.0, 30.0)                                       # web over the cavity (holed for the post below)
    # motor plate, drawn (15 Oct 06:10: at h 2.5 a generated 4 mm plate came out as flakes; the plate is the motors'
    # mounting face, so it is drawn): three 36-wide spokes from the centre web to each seat + the two post footprints
    for M in L.MOTORS.values():
        r = math.hypot(M["xc"], M["zc"]); ux, uz = M["xc"] / r, M["zc"] / r; px, pz = -uz, ux
        plate_c += hull([[s * px * 18.0, y, s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)] +
                        [[M["xc"] + s * px * 18.0, y, M["zc"] + s * pz * 18.0] for s in (-1, 1) for y in (50.0, 54.0)])
    for z in L.POSTS_A:                                                          # A's post footprints (Y 50..52: head B's foot sweeps 2.5 above)
        plate_c += hull([[x, y, zz] for x in (-18.0, 18.0) for y in (50.0, 52.0) for zz in (z - 8.0, z + 8.0)] +
                        [[x, y, zz] for x in (-18.0, 18.0) for y in (50.0, 52.0) for zz in (-12.0, 12.0)])
    for x in L.POSTS_B:                                                          # B's post footprints along the arm's -X
        plate_c += hull([[xx, y, z] for xx in (-x - 8.0, -x + 8.0) for y in (50.0, 52.0) for z in (-18.0, 18.0)] +
                        [[xx, y, z] for xx in (-12.0, 12.0) for y in (50.0, 52.0) for z in (-18.0, 18.0)])
    seats = None; seat_cuts = None
    for nm, M in L.MOTORS.items():
        xc, zc = M["xc"], M["zc"]
        sl = L.AX_SLOT if nm == "axle" else 0.0
        rr = math.hypot(xc, zc); ux, uz = xc / rr, zc / rr                       # motor-to-axle line (slot direction)
        def slot(f):                                                             # f(dx, dz) -> body; hull of the +-sl shifted copies
            return f(0.0, 0.0) if sl == 0 else (f(-sl * ux, -sl * uz) + f(sl * ux, sl * uz)).hull()
        s = cylY(xc, zc, L.Y_PLATE0, L.Y_PLATE1, 26.5)                              # seat: disc covering the 4 bolt bosses
        c = slot(lambda dx, dz: cylY(xc + dx, zc + dz, L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, L.NEMA_BOSS / 2 + 0.5))   # boss hole
        body_k = rsquare(M, L.NEMA / 2 + 1.0, 0.0, L.Y_PLATE0)                     # motor body keep-out (+1 clearance)
        c += body_k if sl == 0 else (body_k.translate([-sl * ux, 0, -sl * uz]) + body_k.translate([sl * ux, 0, sl * uz]))
        for sx in (-1, 1):
            for sz in (-1, 1):                                                   # M3 holes on the 31 square, rotated with the body
                hx, hz = rot_pt(M, sx * L.NEMA_HOLES / 2, sz * L.NEMA_HOLES / 2)
                c += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0 - 1, L.Y_PLATE1 + 1, 1.7))
                s += slot(lambda dx, dz: cylY(hx + dx, hz + dz, L.Y_PLATE0, L.Y_PLATE1, 4.5))
        seats = s if seats is None else seats + s
        seat_cuts = c if seat_cuts is None else seat_cuts + c
    posts = None
    for z in L.POSTS_A:                                                          # A's bar (along +Z): posts 32 x 12, 2 x M5 at X +-10 into the -Y slots
        p = box(-16, 16, 52.0, 74.0, z - 6, z + 6) - cylY(-10, z, 51, 75, 2.7) - cylY(10, z, 51, 75, 2.7)
        posts = p if posts is None else posts + p
    for x in L.POSTS_B:                                                          # B's bar (along -X): same, turned
        p = box(-x - 6, -x + 6, 52.0, 74.0, -16, 16) - cylY(-x, -10, 51, 75, 2.7) - cylY(-x, 10, 51, 75, 2.7)
        posts = posts + p
    # the fixed sun's keep-out: post hole, recess under the 80T flange (Y > 53 inside r 29), the post itself + 1
    post_k = cylY(0, 0, 39.0, 56.0, L.POST_HOLE_R, 128) + cylY(0, 0, L.HUB_RECESS_Y, 80.0, L.HUB_RECESS_R, 128)
    drawn = (sleeve + shoulder + plate_c + seats + posts) - post_k
    cuts = cylY(0, 0, y0 - 1, 40.0, L.BRG_OD / 2 + 0.10, 256) + cylY(0, 0, 39.9, 48.0, 36.0) + seat_cuts + post_k
    if getattr(L, "HUB_DETENT", False):                                          # hub v5: the detent tower (detent.py), drawn
        import detent as DT
        c = DT.CHOSEN; Dc = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); Sd = DT.arm_solids(Dc, np.array(c["Q"]))
        tower = Sd["tower"]
        drawn = drawn + tower
        cuts = cuts + Sd["arm"] + Sd["spacer"] + Sd["nut"] + Sd["roller"] + Sd["springs"][0] + Sd["springs"][1]
    env = cylY(0, 0, y0, L.Y_PLATE0, 110.0) ^ box(-88.0, 88.0, y0 - 1, L.Y_PLATE0 + 1, -88.0, 88.0)
    env -= cylY(0, 0, y0 - 1, 40.0, L.BRG_OD / 2 + 0.10, 256) + cylY(0, 0, 39.9, 48.0, 36.0)
    env -= seat_cuts
    env = (env + posts + plate_c + seats) - post_k                               # drawn bodies above Y 50 are in the env
    if getattr(L, "HUB_DETENT", False): env = env + tower
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts)


# ================================================================== CARRIAGE / HEAD (generated, x2: A and B variants)
def bar_2040(z0, z1):
    """2040 flat in the head's frame: X +-20 (tangential), Y Y_BAR (74..94), along Z from z0 to z1. T-slots as 6.2 x 6
    channels: two on each Y face (X +-10: the posts' and idler ends' T-nuts), one on each X face (Y 84: the glide tongues)."""
    y0, y1 = L.Y_BAR; ys = (y0 + y1) / 2
    b = box(-20, 20, y0, y1, z0, z1)
    for sx in (-1, 1):
        b -= box(min(sx * 20.01, sx * 14.0), max(sx * 20.01, sx * 14.0), ys - 3.1, ys + 3.1, z0 - 1, z1 + 1)
    for x in (-10, 10):
        b -= box(x - 3.1, x + 3.1, y0 - 0.01, y0 + 6.0, z0 - 1, z1 + 1) + box(x - 3.1, x + 3.1, y1 - 6.0, y1 + 0.01, z0 - 1, z1 + 1)
    return b


def mirror_x(m):
    return m.mirror([1, 0, 0])


def _glide_clamp(variant):
    """glide (v1 style, round the flat 2040) and the head belt clamp v2 (belt_clamp_v2_head.py, 17 Oct): the clamp body is
    drawn; its joint to the glide skin is generated inside the clamp env; the cover plate, its screw-driver path and the
    belt line past both mouths are kept out. Returns glide, tongues, clamp dict (drawn, plate, keep, env, ends), xa, ybelt."""
    import belt_clamp_v2_head as BC                                  # v2 loop-and-pin (17 Oct 15:40); v1 belt_clamp.py retired
    yb0, yb1 = L.Y_BAR
    u0, u1 = 0.0, L.GLIDE_L
    gx = L.GLIDE_X
    top = box(-gx, gx, yb1 + 0.3, yb1 + 3.3, u0, u1)
    sides = box(-gx, -20.3, yb0, yb1 + 3.3, u0, u1) + box(20.3, gx, yb0, yb1 + 3.3, u0, u1)
    ys = (yb0 + yb1) / 2
    tongues = box(-20.4, -17.3, ys - 2.7, ys + 2.7, u0, u1) + box(17.3, 20.4, ys - 2.7, ys + 2.7, u0, u1)
    glide = top + sides + tongues
    c = BC.head_clamp(variant, CLAMP_SLOT)
    ybelt = L.Y_HBELT if variant == "A" else L.Y_HBELT_B
    xa = BC.HEADS[variant]["x_pitch"]                                     # the clamped run's pitch line (X)
    return glide, tongues, c, xa, ybelt


def head_env_region(y0, y1, u_lo=-26.0, u_hi=None):
    """v4 envelope rule for head A's parts under the pack (X, u in the head frame), extruded Y y0..y1:
    park bisector X + z0 + u >= BISECT, and r <= ENV_R at the drone (z0 Z0_DRONE) outside |X| <= 19 (the own mouth)."""
    u_hi = L.U_HEAD_TOP if u_hi is None else u_hi
    zc = -L.Z0_DRONE
    circ = cylY(0.0, zc, y0, y1, L.ENV_R, 180) + box(-19.0, 19.0, y0, y1, u_lo, u_hi)
    band = box(-120, 120, y0, y1, u_lo, u_hi)
    z0 = L.Z0_A_HOME
    # half-space X + u >= BISECT - z0 as a big prism
    k = L.BISECT - z0
    half = hull([[x, y, u] for y in (y0, y1) for (x, u) in ((k - (-200.0), -200.0), (400.0, -200.0), (400.0, 200.0), (k - 200.0, 200.0))])
    return circ ^ band ^ half


def gripper_v4():
    """head A's gripper (head B mirrors it in X). Returns drawn (guide block, servo plate), cuts and the bought solids."""
    yc = L.Y_C; p = L.P_SH
    s = lambda a, b: (yc + a, yc + b)
    # guide block: one channel holding both racks and the pinion, a rib between the racks left of the pinion
    gx0, gx1 = L.GUIDE_X; gu0, gu1 = L.GUIDE_U
    guide = box(gx0, gx1, *s(-L.GUIDE_S, L.GUIDE_S), gu0, gu1)
    guide -= box(gx0 - 1, gx1 + 1, *s(-12.2, 12.2), L.U_PIN[0] - 0.2, L.U_PIN[1] + 0.2)
    guide += box(gx0, p - 9.5, *s(-5.6, 5.6), L.U_PIN[0] - 0.21, L.U_PIN[1] + 0.21)
    guide -= cylZ(p, yc, gu0 - 1, L.U_PIN[0] + 0.01, L.SERVO_BOSS_D / 2 + 0.8)          # the boss through the floor
    ss0_, ss1_ = L.S_SERVO
    guide -= box(p - L.SERVO_W / 2 - 0.5, p + L.SERVO_W / 2 + 0.5, *s(ss0_ - 2.0, ss1_ + 1.5), gu0 - 1, L.U_SERVO[1] + 0.8)   # v4d: the case top rises to -11: no floor over the servo
    # servo plate on the ears (window for the body), 4 x M3 heat-set from below
    ss0, ss1 = L.S_SERVO; ex0, ex1 = ss0 - 7.0, ss1 + 7.0
    xs0, xs1 = p - L.SERVO_W / 2, p + L.SERVO_W / 2
    plate = box(xs0 - 3.0, xs1 + 3.0, *s(ex0 - 2.5, ex1 + 2.5), *L.U_PLATE)
    plate -= box(xs0 - 0.5, xs1 + 0.5, *s(ss0 - 0.5, ss1 + 0.5), L.U_PLATE[0] - 1, L.U_PLATE[1] + 1)
    # v5 (17 Oct, wiring): zip-tie tab on the plate's far (+s) end, past the ear inserts: 12 x 6 x the plate's 6, one slot
    # through it along u; the tie loops through the slot and round the tab's end (walls >= 2.0, C)
    zl, zw = ZIP_SLOT
    plate += box(p - 6.0, p + 6.0, *s(ex1 + 2.5 - 0.01, ex1 + 2.5 + 6.0), *L.U_PLATE)
    plate -= box(p - zl / 2, p + zl / 2, *s(ex1 + 2.5 + 2.0, ex1 + 2.5 + 2.0 + zw), L.U_PLATE[0] - 1, L.U_PLATE[1] + 1)
    holes = None
    sm = (ss0 + ss1) / 2
    for ds in (-L.SERVO_HOLES[0] / 2, L.SERVO_HOLES[0] / 2):
        for dx in (-L.SERVO_HOLES[1] / 2, L.SERVO_HOLES[1] / 2):
            # 17 Oct: 4 x M3 heat-set inserts pressed in from the plate's bottom face (box-arm.md); the screws come up through
            # the servo ears. Was M3 slots + nut pockets on top: the guide block and the generated body capped the nuts.
            h = cylZ(p + dx, yc + sm + ds, L.U_PLATE[0] - 1.0, L.U_PLATE[0] + SERVO_INSERT_L, SERVO_INSERT_D / 2, 32)
            holes = h if holes is None else holes + h
    servo = box(xs0, xs1, *s(ss0, ss1), *L.U_SERVO) + box(xs0, xs1, *s(ex0, ex1), *L.U_EARS) + \
        cylZ(p, yc, L.U_BOSS[0] - 0.01, L.U_BOSS[1], L.SERVO_BOSS_D / 2) + cylZ(p, yc, L.U_BOSS[1], L.U_PIN[1], 2.9)
    servo_c = box(xs0 - 0.5, xs1 + 0.5, *s(ss0 - 2.0, ss1 + 1.5), L.U_SERVO[0] - 0.5, L.U_SERVO[1] + 0.5) + \
        box(xs0 - 0.5, xs1 + 0.5, *s(ex0 - 2.0, ex1 + 2.0), L.U_EARS[0] - 3.5, L.U_EARS[1] + 0.01) + \
        cylZ(p, yc, L.U_BOSS[0] - 0.5, L.U_PIN[1] + 0.5, L.SERVO_BOSS_D / 2 + 0.5) + holes
    pin_c = cylZ(p, yc, L.U_PIN[0] - 0.3, L.U_PIN[1] + 0.3, 7.3)
    # rack and blade sweeps (open -1 .. squeezed +4), the carried cartridge and the space over it
    ra0, ra1 = L.RACK_A_X; rb0, rb1 = L.RACK_B_X
    rack_c = box(ra0 - 1.2, ra1 + 4.2, *s(5.1, 12.2), L.U_PIN[0] - 0.2, L.U_PIN[1] + 0.2) + \
        box(rb0 - 4.2, rb1 + 1.2, *s(-12.2, -5.1), L.U_PIN[0] - 0.2, L.U_PIN[1] + 0.2)
    bi, bt = L.BLADE_IN, L.BLADE_THICK
    blade_c = box(-bi - bt - 1.5, -bi + 4.5, *s(-L.BLADE_S - 0.5, 12.2), L.U_PIN[0] - 0.5, L.CART_TOP_DZ + 4) + \
        box(bi - 4.5, bi + bt + 1.5, *s(-12.2, L.BLADE_S + 0.5), L.U_PIN[0] - 0.5, L.CART_TOP_DZ + 4)
    cart_c = box(-L.CART_W / 2 - 1.0, L.CART_W / 2 + 1.0, *s(-L.Z_TABC - 1.0, L.CART_L - L.Z_TABC + 1.0), L.CART_BOT_DZ - 1.0, L.CART_TOP_DZ + 4)
    cuts = servo_c + pin_c + rack_c + blade_c + cart_c
    return dict(guide=guide, plate=plate, drawn=guide + plate, cuts=cuts, servo=servo)


def head_parts(variant="A"):
    """head v4 in its own frame (glide inner end at u = Z = 0, bar along +Z). Glide as v3; belt clamp v1 (belt_clamp.py).
    The gripper group (guide block, servo plate) hangs under the carried pack at Y 97.3..183 and is joined to the glide's
    top skin by the generated body inside head_env_region; the clamp's toothed bed is joined to the glide's side skin by
    the generated body inside the clamp env. Head A as drawn; head B's group mirrored in X (servo on its -X)."""
    glide, tongues, c, xa, ybelt = _glide_clamp(variant)
    yb0, yb1 = L.Y_BAR
    G = gripper_v4()
    grp, grp_c = G["drawn"], G["cuts"]
    genv = head_env_region(yb1 + 3.3 - 0.01, L.Y_C + 47.0)                 # v4c: to 5.5 past the carried Scout cartridge (was Y_FAR_WALL - 1 = 183)
    # v5 wiring entry (nox/box-wiring.md): Ø4.5 channel along Y at (X -10, u 8) from the bar's top slot up through the glide's
    # top skin and 3 beyond (open for the ribbon), plus a zip-tie fin on the skin beside the hole (2 thick, 5 tall, 8 long,
    # slot 3.4 x 1.6 through it along X; walls >= 2.0, C). Head B: mirrored in X with the gripper group.
    ex, du, ed = WIRE_ENTRY_X, WIRE_ENTRY_DU, WIRE_ENTRY_D
    wire_c = cylY(ex, du, yb1 - 7.0, yb1 + 3.3 + 3.0, ed / 2, 32)
    zl, zw = ZIP_SLOT
    fin = box(ex + ed / 2 + 0.75, ex + ed / 2 + 2.75, yb1 + 3.3 - 0.01, yb1 + 3.3 + 5.0, du - 4.0, du + 4.0)
    fin -= box(ex + ed / 2 + 0.25, ex + ed / 2 + 3.25, yb1 + 3.3 + 1.0, yb1 + 3.3 + 1.0 + zw, du - zl / 2, du + zl / 2)
    if variant == "B":
        grp, grp_c, genv = mirror_x(grp), mirror_x(grp_c), mirror_x(genv)
        wire_c, fin = mirror_x(wire_c), mirror_x(fin)
    drawn = glide + c["drawn"] + grp + fin
    cuts = box(-20.3, 20.3, yb0 - 1.0, yb1 + 0.3, -3, L.GLIDE_L + 3) - tongues
    cuts += c["keep"]
    cuts += grp_c
    cuts += wire_c
    env = box(-24.0, 24.0, yb0, yb1 + 3.3, -1.0, L.GLIDE_L + 2.0) + box(-24.0, -20.3, yb0 - 4.0, yb0, -1.0, L.GLIDE_L + 2.0)
    env += genv
    env += c["env"]
    env = env + grp + c["drawn"] + fin
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts, xa=xa, ybelt=ybelt, wire_entry=wire_c, fin=fin,
                clamp_ends=c["ends"], belt_plate=c["plate"],
                guide=G["guide"] if variant == "A" else mirror_x(G["guide"]),
                plate=G["plate"] if variant == "A" else mirror_x(G["plate"]))


def pinion_u(n=12, m=1.0):
    """12T module-1 pinion, axis along u (Z), on the servo spline (bore 5.9), at (P_SH, Y_C), u U_PIN."""
    g = pinion(n, m, 0.0, L.U_PIN[1] - L.U_PIN[0])                         # along X from 0
    return g.rotate([0, -90, 0]).translate([L.P_SH, L.Y_C, L.U_PIN[0]])


def servo_v4(variant="A"):
    sv = gripper_v4()["servo"]
    return sv if variant == "A" else mirror_x(sv)


def _rack_teeth_x(x0, x1, s_pitch, dirn, phase_x):
    """module-1 teeth along X on a rack face at s = s_pitch, pointing dirn (+1: toward +s), width over U_PIN."""
    t = rack_teeth(x0, x1, L.U_PIN[0], L.U_PIN[1], s_pitch, dirn, phase_y=phase_x)   # local: x = u width, y = X, z = s
    return t.transform([[0, 1, 0, 0], [0, 0, 1, L.Y_C], [1, 0, 0, 0]])


def fingers_and_racks(variant="A"):
    """v4: finger A = rack A (s +6..+12) + the -X blade; finger B = rack B (s -12..-6) + the +X blade. Blades 13 wide
    (s +-6.5) in the 14 recess, 3 thick to u 22.9, 5 thick below u 14, inner faces X -+10 on the tabs (-+11 open, -+6
    squeezed); each blade steps over to its rack below u -4 (beside the other rack). Rib 0.8 into the tab pocket at u 19.4.
    Head B: mirrored in X."""
    yc = L.Y_C; p = L.P_SH; u0, u1 = L.U_PIN; bi, bt, bk = L.BLADE_IN, L.BLADE_T, L.BLADE_THICK
    parts = {}
    for name, sgn in (("A", -1), ("B", +1)):
        rs0, rs1 = (L.RACK_S[0], L.RACK_S[1]) if sgn < 0 else (-L.RACK_S[1], -L.RACK_S[0])
        rx0, rx1 = L.RACK_A_X if sgn < 0 else L.RACK_B_X
        rack = box(rx0, rx1, yc + rs0, yc + rs1, u0, u1)
        t0, t1 = L.TEETH_A if sgn < 0 else L.TEETH_B
        sp = 6.0 if sgn < 0 else -6.0; dirn = -1 if sgn < 0 else +1
        rack -= box(t0, t1, yc + sp - 1.0, yc + sp + 1.0, u0 - 1, u1 + 1)
        rack += _rack_teeth_x(t0, t1, sp, dirn, p) ^ box(t0, t1, yc - 20, yc + 20, u0, u1)
        xb = (-bi - bt, -bi) if sgn < 0 else (bi, bi + bt)            # 3 thick in the recess
        xk = (-bi - bk, -bi) if sgn < 0 else (bi, bi + bk)            # 5 thick below u 14
        sb = L.BLADE_S
        blade = box(*xb, yc - sb, yc + sb, L.U_HEAD_TOP, L.FINGER_END_DZ) + box(*xk, yc - sb, yc + sb, u1 + 0.5, L.U_HEAD_TOP + 0.01)
        step = box(*xk, yc - 5.5, yc + rs1, u0, u1 + 0.51) if sgn < 0 else box(*xk, yc + rs0, yc + 5.5, u0, u1 + 0.51)
        d = 0.8; t = math.tan(math.radians(5)); w0, h0 = L.POCK_W - 0.3, L.POCK_H - 0.3
        w1, h1 = w0 - 2 * d * t, h0 - 2 * d * t
        xin = -bi if sgn < 0 else bi; dx = +d if sgn < 0 else -d
        rib = hull([[xin, yc + ss * w0 / 2, L.RIB_DZ + su * h0 / 2] for ss in (-1, 1) for su in (-1, 1)] +
                   [[xin + dx, yc + ss * w1 / 2, L.RIB_DZ + su * h1 / 2] for ss in (-1, 1) for su in (-1, 1)])
        parts[name] = rack + blade + step + rib
    if variant == "B":
        parts = {k: mirror_x(v) for k, v in parts.items()}
    return parts


# ================================================================== IDLER END (drawn, x2: A and B variants)
def idler_end(variant="A"):
    """v3 (flat bars): a plate UNDER the bar's end (2 x M5 T-nuts in the -Y face slots at X +-10, Z 125), a bracket out
    beside the bar to a cheek carrying the idler on a cantilever M5 at (X_HBELT_C, Z_IDLER). A: cheek under the belt
    (Y 57..63.5, idler Y 64..74). B: cheek over the belt (Y 64.5..71, idler Y 54..64, belt 56..62).
    Belt tension (belt_clamp.py v1): loosen the 2 M5, push the idler end out along the bar's slot to the pluck target, tighten."""
    zb = L.BAR_HALF; yb0 = L.Y_BAR[0]
    xa = L.X_HBELT_C if variant == "A" else L.X_HBELT_C_B
    zi = L.Z_IDLER if variant == "A" else L.Z_IDLER_B
    plate = box(-20.0, 20.0, yb0 - 6.3, yb0 - 0.3, zb - 18.0, zb)
    for x in (-10.0, 10.0): plate -= cylY(x, zb - 9.0, yb0 - 7, yb0 + 1, 2.7)                 # M5 into the -Y face slot T-nuts
    yc0, yc1 = (57.0, 63.5) if variant == "A" else (64.5, 71.0)
    bracket = box(20.0, 25.0, min(yc0, yb0 - 6.3), yb0 - 0.3, zb - 18.0, zb)
    cheek = (box(22.0, 25.0, yc0, yc1, zb - 18.0, zb) + cylY(xa, zi, yc0, yc1, 9.0)).hull()
    body = plate + bracket + cheek
    body -= cylY(xa, zi, yc0 - 1, yc1 + 1, 2.6)                                                  # Ø5.2 axle
    if variant == "A": body -= cylY(xa, zi, yc0 - 1, yc0 + 3.0, 4.6)                             # counterbore for the head (below)
    else: body -= cylY(xa, zi, yc1 - 3.0, yc1 + 1, 4.6)                                          # (above)
    return body


# ================================================================== BAY (static, generated)
def bay_parts(angle_deg=None):
    """one Scout bay at the top position (then rotated to its ring angle for the assembly). W coords, radial = Z.
    Drawn: back pad (cartridge top rests on it), 2 side rails (tray long walls, X faces of the pack... no: the tray's
    x faces are the Y faces here), 2 end guides, 2 pin blocks, 2 TPU lip slots, 3 feet on the far wall."""
    yc = L.Y_C
    r_tip, r_top = L.R, L.R_BAY_TOP
    x0c, x1c = -L.Z_TABC, L.CART_L - L.Z_TABC                     # cartridge tangential extent -36.6 .. 41.4
    cav = box(x0c - 0.5, x1c + 0.5, yc - L.CART_X_OUT - 0.4, yc + L.CART_X_OUT + 0.4, r_tip - 1.0, r_top + 0.5)   # tray + 0.4
    cav += box(x0c - 0.5, x1c + 0.5, yc - L.CART_W / 2 - 0.4, yc + L.CART_W / 2 + 0.4, r_top - 10.0, r_top + 0.5)  # bosses
    cav += box(-9.0, 9.0, yc - 16.0, yc + 16.0, r_tip - 12.0, r_tip + 1.0)        # blades under the recess, open
    cav += box(-26.0, 26.0, L.Y_BAR[0] - 1.0, L.Y_GLIDE1 + 1.0, r_tip - 10.0, r_tip + 20.0)   # glide at full extension
    cav += box(-48.0, 48.0, 60.0, 150.0, r_tip - 12.0, r_tip + 2.0)                # head body sweep under the bay mouth
    HA = head_parts()["allowed"]
    for z0 in np.arange(L.Z0_BAY - 6.0, L.Z0_DRONE + 0.1, 2.0):                    # the head over its travel at the bay
        cav += HA.translate([0, 0, float(z0)])
    cartN = box(-L.Z_TABC, L.CART_L - L.Z_TABC, yc - L.CART_W / 2, yc + L.CART_W / 2, L.Z0_BAY - 6.0 + L.CART_BOT_DZ - 1.0, L.Z0_DRONE + L.CART_TOP_DZ + 1.0)
    for d in (-45.0, 45.0):                                                       # a head at the neighbour position, over-travel, with its pack
        cav += rotY(HA.translate([0, 0, L.Z0_DRONE]) + cartN, d)
    # back pad: the cartridge top (y 42 face) rests here; 3 mm, over the pack footprint minus the lead end
    pad = box(x0c, x1c - 6.0, yc - 16.0, yc + 16.0, r_top + 0.5, r_top + 3.5)
    # side rails (guide the tray's long walls at x +-18.1 -> here Y faces), 2.5 thick, over the lower 24 mm of the tray
    rails = box(x0c, x1c, yc - L.CART_X_OUT - 2.9, yc - L.CART_X_OUT - 0.4, r_tip + 1.0, r_tip + 27.0) + \
            box(x0c, x1c, yc + L.CART_X_OUT + 0.4, yc + L.CART_X_OUT + 2.9, r_tip + 1.0, r_tip + 27.0)
    ends = box(x0c - 3.0, x0c - 0.5, yc - 16, yc + 16, r_tip + 1.0, r_top) + box(x1c + 0.5, x1c + 3.0, yc - 16, yc + 16, r_tip + 1.0, r_top)
    # pin blocks: 4 bores each side at R_PIN, pitch 5.4 along X, centred at the strip centre (z 65.5 -> X 28.9)
    xpc = L.Z_PINC - L.Z_TABC
    blocks = None; bores = None
    for s in (-1, 1):
        y0 = yc + s * L.RAIL_FACE; y1 = yc + s * (L.PINBLK_X1)
        b = box(xpc - 12.0, xpc + 12.0, min(y0, y1), max(y0, y1), L.R_PIN - 5.7, L.R_PIN + 5.7)
        blocks = b if blocks is None else blocks + b
        for i in range(L.PIN_N):
            x = xpc + (i - (L.PIN_N - 1) / 2) * L.PIN_PITCH
            c = cylY(x, L.R_PIN, yc + s * (L.X_PIN_TIP - 1.0) if s > 0 else y1 - 1, (y1 + 1) if s > 0 else yc - (L.X_PIN_TIP - 1.0), L.PIN_BORE / 2)
            bores = c if bores is None else bores + c
    # TPU lip slots: dovetail-free pockets 12 x 3 x 6 on each side rail at r_tip + 22 (over the block's bottom edge)
    lips = box(-8.0, 8.0, yc - L.CART_X_OUT - 2.9, yc - L.CART_X_OUT - 0.4 + 1.5, r_tip + 0.0, r_tip + 6.0) + \
           box(-8.0, 8.0, yc + L.CART_X_OUT + 0.4 - 1.5, yc + L.CART_X_OUT + 2.9, r_tip + 0.0, r_tip + 6.0)
    lip_c = box(-6.0, 6.0, yc - L.CART_X_OUT - 2.9 - 0.01, yc - L.CART_X_OUT + 0.1, r_tip + 1.0, r_tip + 5.0) + \
            box(-6.0, 6.0, yc + L.CART_X_OUT - 0.1, yc + L.CART_X_OUT + 2.9 + 0.01, r_tip + 1.0, r_tip + 5.0)
    # feet on the far wall: 3 pads with M3 holes at Y 170..174
    feet = None; feet_c = None
    for (x, r) in ((x0c - 2, r_top - 20), (x1c + 2, r_top - 20), (0.0, r_top - 2)):
        f = box(x - 6, x + 6, L.Y_FAR_WALL - 4.0, L.Y_FAR_WALL, r - 6, r + 6)
        c = cylY(x, r, L.Y_FAR_WALL - 5, L.Y_FAR_WALL + 1, 1.7)
        feet = f if feet is None else feet + f; feet_c = c if feet_c is None else feet_c + c
    drawn = pad + rails + ends + blocks + lips + feet
    cuts = cav + bores + lip_c + feet_c
    env = box(-46.0, 46.0, yc - 38.0, L.Y_FAR_WALL, r_tip - 2.0, L.R_BAY_BACK)
    env -= cav
    env = env + feet + blocks + lips
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts)


def swept_head_env(variant, z_lo, z_hi, dil=1.0):
    """the head's allowed envelope (+ its drawn parts) swept along its bar from z0 = z_lo to z_hi, dilated by `dil`
    (Minkowski sum with a box). In the head frame (bar along +Z)."""
    H = head_parts(variant)
    e = H["allowed"] + (H["drawn"] - H["cuts"])
    k = box(-dil, dil, -dil, dil, z_lo - dil, z_hi + dil)
    try:
        return e.minkowski_sum(k)
    except Exception:
        parts = [e.translate([dx, dy, float(z)]) for z in np.arange(z_lo - dil, z_hi + dil + 0.01, 1.0) for dx in (-dil, dil) for dy in (-dil, dil)]
        return mf.Manifold.batch_boolean(parts, mf.OpType.Add)


def pack_path(z_seat, clr=0.4):
    """the cartridge's path along a position's line from park to its seat (bottom face at r z_seat), with the tray,
    the pad bosses and the end clearances of the bay cavity. Head frame (radial = Z, X tangential, Y along the axle)."""
    yc = L.Y_C; x0c, x1c = -L.Z_TABC, L.CART_L - L.Z_TABC
    r0 = L.Z0_A_HOME + L.CART_BOT_DZ - 1.0
    p = box(x0c - 0.5, x1c + 0.5, yc - L.CART_X_OUT - clr, yc + L.CART_X_OUT + clr, r0, z_seat + L.CART_H + 0.5)
    p += box(55.3 - L.Z_TABC - 1.0, 77.3 - L.Z_TABC + 1.0, yc - L.RAIL_FACE, yc + L.RAIL_FACE, r0, z_seat + 44.5)   # pad bosses (39.8 over) pass the rails
    return p


def bay_ring_parts():
    """v3 (Spark 16 Oct 02:36: the bays are one structure, glued): all 7 Scout bays as ONE ring, generated in one run,
    then split along the 22.5 deg lines into 7 plate-sized segments glued face to face (6 thick joint webs, 3 per side,
    2 x Ø3.2 dowels each). Drawn per bay: back pad, side rails (r 101..127, relieved to RAIL_FACE over the pad bosses),
    end guides, 2 pin blocks with 8 x Ø3.4 bores, 2 TPU lip slots, ONE foot on the far wall at the bay's centre r 146
    (M3). Envelope: annulus r 99.5..154 x Y 88..174 outside the drone's 45 deg sector, minus: the bar sweep (r < 136.5
    for Y < 95.5), both heads' envelopes swept over their travel at all 8 positions (dilated 1), every pack path.
    Returns drawn, cuts, env, allowed, plus feet, pads, lips, blocks (load regions) and joints (plane angles)."""
    yc = L.Y_C; r_tip, r_top = L.R, L.R_BAY_TOP
    x0c, x1c = -L.Z_TABC, L.CART_L - L.Z_TABC
    xpc = L.Z_PINC - L.Z_TABC
    def at(m, a): return m.rotate([0, 90.0 - a, 0])
    drawn = None; cuts = None; feet = None; pads = []; lipsr = []; blocksr = []
    for a in L.BAY_ANGLES:
        pad = box(x0c, x1c - 6.0, yc - 16.0, yc + 16.0, r_top + 0.5, r_top + 3.5)
        rails = box(x0c, x1c, yc - L.CART_X_OUT - 2.9, yc - L.CART_X_OUT - 0.4, r_tip + 1.0, r_tip + 27.0) + \
                box(x0c, x1c, yc + L.CART_X_OUT + 0.4, yc + L.CART_X_OUT + 2.9, r_tip + 1.0, r_tip + 27.0)
        ends = box(x0c - 3.0, x0c - 0.5, yc - 16, yc + 16, r_tip + 1.0, r_top) + box(x1c + 0.5, x1c + 3.0, yc - 16, yc + 16, r_tip + 1.0, r_top)
        for xa_, xb_ in ((x0c - 3.0, x0c + 0.01), (x1c - 0.01, x1c + 3.0)):            # end guides tied to the rail ends (the ring run left the two next to the drone gap loose)
            for s_ in (-1, 1):
                ends += box(xa_, xb_, min(yc + s_ * (L.CART_X_OUT + 0.4), yc + s_ * 16), max(yc + s_ * (L.CART_X_OUT + 2.9), yc + s_ * 16), r_tip + 1.0, r_tip + 27.0)
        blocks = None; bores = None
        for s in (-1, 1):
            y0 = yc + s * L.RAIL_FACE; y1 = yc + s * L.PINBLK_X1
            b = box(xpc - 12.0, xpc + 12.0, min(y0, y1), max(y0, y1), L.R_PIN - 5.7, L.R_PIN + 5.7)
            blocks = b if blocks is None else blocks + b
            for i in range(L.PIN_N):
                x = xpc + (i - (L.PIN_N - 1) / 2) * L.PIN_PITCH
                c = cylY(x, L.R_PIN, yc + s * (L.X_PIN_TIP - 1.0) if s > 0 else y1 - 1, (y1 + 1) if s > 0 else yc - (L.X_PIN_TIP - 1.0), L.PIN_BORE / 2)
                bores = c if bores is None else bores + c
        lips = box(-8.0, 8.0, yc - L.CART_X_OUT - 2.9, yc - L.CART_X_OUT - 0.4 + 1.5, r_tip + 0.0, r_tip + 6.0) + \
               box(-8.0, 8.0, yc + L.CART_X_OUT + 0.4 - 1.5, yc + L.CART_X_OUT + 2.9, r_tip + 0.0, r_tip + 6.0)
        lip_c = box(-6.0, 6.0, yc - L.CART_X_OUT - 2.9 - 0.01, yc - L.CART_X_OUT + 0.1, r_tip + 1.0, r_tip + 5.0) + \
                box(-6.0, 6.0, yc + L.CART_X_OUT - 0.1, yc + L.CART_X_OUT + 2.9 + 0.01, r_tip + 1.0, r_tip + 5.0)
        d = pad + rails + ends + blocks + lips
        c = bores + lip_c
        if a in L.RING_FEET:                                      # 4 feet only: the ring has to carry the other bays round to them
            foot = box(-6.0, 6.0, L.Y_FAR_WALL - 4.0, L.Y_FAR_WALL, r_top - 8.0, r_top + 4.0)
            d += foot; c += cylY(0.0, r_top - 2.0, L.Y_FAR_WALL - 5, L.Y_FAR_WALL + 1, 1.7)
            feet = at(foot, a) if feet is None else feet + at(foot, a)
        drawn = at(d, a) if drawn is None else drawn + at(d, a)
        cuts = at(c, a) if cuts is None else cuts + at(c, a)
        pads.append(at(box(x0c, x1c - 6.0, yc - 16.5, yc + 16.5, r_top - 1.0, r_top + 6.0), a))     # load regions: a few cells deep at h 3
        lipsr.append(at(box(-10.0, 10.0, yc - 27.0, yc + 27.0, r_tip - 1.0, r_tip + 12.0) -          # the rails round the lip slots (the lips themselves are below h)
                        box(-11, 11, yc - L.CART_X_OUT - 0.3, yc + L.CART_X_OUT + 0.3, r_tip - 2, r_tip + 13), a))
        blocksr.append(at(blocks, a))
    # joint webs at the 6 internal boundaries: 4 thick about the plane, r 118..153, Y 96..172; 2 x Ø3.2 dowel holes
    joints = [a - 22.5 for a in L.BAY_ANGLES[:-1]]                      # between consecutive bays (22.5, -22.5, ..., -202.5)
    for j in joints:
        web = box(-2.0, 2.0, 96.0, 172.0, 118.0, 153.0)                 # 4 thick: 2 per side (the floor), glued face to face
        dw = cylX(132.0, 125.0, -3, 3, 1.6) + cylX(160.0, 146.0, -3, 3, 1.6)   # Ø3.2 for 3 mm dowels (alignment)
        drawn += at(web, j); cuts += at(dw, j)
    # envelope
    annulus = cylY(0, 0, 88.0, L.Y_FAR_WALL, L.R_BAY_BACK) - cylY(0, 0, 87.0, L.Y_FAR_WALL + 1, 99.5)
    gap = mf.Manifold.hull_points([[0, 80, 0], [0, 180, 0]] + [[sx * 400 * math.sin(math.radians(22.5)), y, 400 * math.cos(math.radians(22.5))] for sx in (-1, 1) for y in (80, 180)])
    env = annulus - gap
    keep = cylY(0, 0, 70.0, 95.5, 136.5)                               # the bars' sweep (corner r 135.5)
    for p in [90.0] + list(L.BAY_ANGLES):
        zmax = (L.Z0_DRONE if p == 90.0 else L.Z0_BAY) + 0.5
        for v in ("A", "B"):
            keep += at(swept_head_env(v, L.Z0_A_HOME, zmax), p)
        keep += at(pack_path(L.R + (5.0 if p == 90.0 else 0.0)), p)
    cuts = cuts + keep
    env = env - keep + drawn
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts, feet=feet, pads=pads, lips=lipsr, blocks=blocksr, joints=joints)


def ring_segments(ring, joints=None, gap=0.0):
    """split a ring body into the 7 bay segments along the 22.5 deg lines (each wedge = the bay's angle +- 22.5)."""
    segs = []
    for a in L.BAY_ANGLES:
        w = mf.Manifold.hull_points([[0, -10, 0], [0, 300, 0]] + [[400 * math.sin(math.radians(sx * 22.5)), y, 400 * math.cos(math.radians(22.5))] for sx in (-1, 1) for y in (-10, 300)])
        segs.append(ring ^ w.rotate([0, 90.0 - a, 0]))
    return segs


def bay_lip():
    """TPU 95A snap lip: 12 x 1.5 base in the slot, 1.2 proud ramp, prints flat. Hold ~5 N (E) per pair."""
    base = box(-6, 6, 0, 1.5, 0, 4.0)
    bump = hull([[x, 1.5, z] for x in (-6, 6) for z in (0, 4)] + [[x, 2.7, z] for x in (-6, 6) for z in (1.2, 2.8)])
    return base + bump


# ================================================================== assembly check
def assembly_check():
    """every part at its place, heads in the worst states; reports overlaps (mm3) between parts that must not touch."""
    H = hub_parts(); C = head_parts(); F = fingers_and_racks(); B = bay_parts()
    hub_env = H["allowed"]
    bar = box(-20, 20, L.Y_BAR[0], L.Y_BAR[1], -L.BAR_HALF, L.BAR_HALF)
    ys = sum(L.Y_BAR) / 2
    for x0, x1 in ((-20.1, -14), (14, 20.1)): bar -= box(x0, x1, ys - 3, ys + 3, -L.BAR_HALF - 1, L.BAR_HALF + 1)
    for x in (-10, 10):
        for y0, y1 in ((L.Y_BAR[0] - 0.1, L.Y_BAR[0] + 6), (L.Y_BAR[1] - 6, L.Y_BAR[1] + 0.1)): bar -= box(x - 3, x + 3, y0, y1, -L.BAR_HALF - 1, L.BAR_HALF + 1)
    p80 = cylY(0, 0, L.Y_P80[0], L.Y_P80[1], L.P80_FL / 2) + cylY(0, 0, L.Y_P80[1], 72.0, L.P80_BOSS / 2)
    motors = None
    for M in L.MOTORS.values():
        m = rsquare(M, L.NEMA / 2, 2.0, L.Y_PLATE0)
        motors = m if motors is None else motors + m
    stb = stub()
    def cart(z0):   # carried cartridge in head A at glide inner end z0: flat bottom at z0+38, top at z0+86
        yc = L.Y_C
        return box(-L.Z_TABC, L.CART_L - L.Z_TABC, yc - L.CART_W / 2, yc + L.CART_W / 2, z0 + L.CART_BOT_DZ, z0 + L.CART_TOP_DZ)
    def head(z0, rot=0.0):
        m = C["allowed"].translate([0, 0, z0]); f = (F["A"] + F["B"]).translate([0, 0, z0])
        return rotY(m + f, rot)
    bays = None
    for a in L.BAY_ANGLES:
        b = rotY(B["allowed"], 90 - a)                          # env at the top -> ring angle a (rotY turns +Z toward +X)
        bays = b if bays is None else bays + b
    bay_carts = None
    for a in L.BAY_ANGLES:
        c = rotY(cart(L.Z0_BAY), 90 - a); bay_carts = c if bay_carts is None else bay_carts + c
    states = {
        "park: A home carrying, B parked": (head(L.Z0_A_HOME), cart(L.Z0_A_HOME), head(-L.Z0_B_HOME - 0 , 180.0)),
        "A at bay 1 (top), B parked": (head(L.Z0_BAY), cart(L.Z0_BAY), head(-L.Z0_B_HOME, 180.0)),
        "A at drone over-travel": (head(L.Z0_DRONE), cart(L.Z0_DRONE), head(-L.Z0_B_HOME, 180.0)),
    }
    fixed = dict(hub=hub_env, bar=bar, p80=p80, motors=motors, stub=stb, bays=bays, bay_carts=bay_carts)
    print("assembly overlaps (mm3, 0 = clear):")
    for sname, (hA, cA, hB) in states.items():
        mov = dict(headA=hA, cartA=cA, headB=hB)
        for mn, mm in mov.items():
            for fn, fm in fixed.items():
                v = (mm ^ fm).volume()
                flag = "" if v < 0.5 else "   <-- OVERLAP"
                if v >= 0.5 or fn in ("bays", "bay_carts"):
                    print(f"  {sname:34s} {mn:6s} vs {fn:9s} {v:9.1f}{flag}")
        print(f"  {sname:34s} headA vs headB {(hA ^ hB).volume():9.1f}")
        print(f"  {sname:34s} cartA vs headB {(cA ^ hB).volume():9.1f}")
    # sweep: the arm turned every 5 deg, parked state, against the bays and the bay cartridges
    worst = 0.0
    swept = fixed["bar"] + states["park: A home carrying, B parked"][0] + states["park: A home carrying, B parked"][1] + \
        states["park: A home carrying, B parked"][2] + hub_env + motors + p80
    for d in range(0, 360, 5):
        v = (rotY(swept, d) ^ (bays + bay_carts)).volume(); worst = max(worst, v)
    print(f"  arm sweep (parked state, 5 deg steps) vs bays + bay cartridges: worst {worst:.1f} mm3")
    return H, C, F, B


if __name__ == "__main__" and len(__import__("sys").argv) > 1 and __import__("sys").argv[1] == "post":
    # hub v4: python3 arm_cad.py post -> the clearance search, the post and stub STLs (print orientation: flange face down)
    for k, v in sun_post_search().items(): print(f"  {k:8s} {v}")
    t = tm(sun_post()); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0]))
    t.apply_translation(-t.bounds[0]); t.export(os.path.join(OUT, "box_sun_post_v1.stl")); print("post", t.extents.round(1), round(t.volume / 1000 * 1.27, 1), "g")
    t = tm(stub()); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0]))
    t.apply_translation(-t.bounds[0]); t.export(os.path.join(OUT, "box_stub_v3.stl")); print("stub", t.extents.round(1), round(t.volume / 1000 * 1.27, 1), "g")
    raise SystemExit

if __name__ == "__main__":
    stb = stub(); report("stub axle (drawn)", stb); stb.rotate([90, 0, 0]).translate([0, 0, 0]) and None
    cap = sun_post(); report("sun post (was the stub cap)", cap)
    H = hub_parts(); report("hub drawn", H["drawn"]); report("hub env", H["env"]); report("hub allowed", H["allowed"])
    C = head_parts(); report("head drawn", C["drawn"]); report("head env", C["env"])
    F = fingers_and_racks(); report("finger+rack A", F["A"]); report("finger+rack B", F["B"])
    pin = pinion(); report("pinion", pin)
    ie = idler_end(); report("idler end", ie)
    B = bay_parts(); report("bay drawn", B["drawn"]); report("bay env", B["env"])
    lip = bay_lip(); report("bay lip (TPU)", lip, 1.21)
    # exports (print orientation: +Z up on the bed)
    def ex(m, name, rot=None, flip=None):
        t = tm(m)
        if rot is not None: t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot[0]), rot[1]))
        t.apply_translation(-t.bounds[0]); t.export(os.path.join(OUT, name)); return t
    ex(stb, "box_stub_v1.stl", (90, [1, 0, 0]))          # flange down? -> flange at Y 0: rotate so -Y points down
    ex(cap, "box_sun_post_v1.stl", (90, [1, 0, 0]))
    ex(F["A"], "box_finger_A_v1.stl", (0, [1, 0, 0]))
    ex(F["B"], "box_finger_B_v1.stl", (0, [1, 0, 0]))
    ex(pin, "box_pinion_v1.stl", (-90, [0, 1, 0]))
    ex(ie, "box_idler_end_v1.stl", (90, [1, 0, 0]))
    ex(lip, "box_bay_lip_v1.stl")
    assembly_check()


# ================================================================== BAY MODULE v4 (Spark 16 Oct 19:28: bolted, any one swaps out)
def _receiver_W():
    """the drone receiver (v2/s2_cartridge.py receiver(), cartridge v3) as the bay's functional surfaces, in the bay frame:
    X = -x, Y = Y_C + (z - Z_TABC), Z = R + y (cartridge bottom at R)."""
    import s2_cartridge as S
    r = man(S.receiver())
    return r.transform([[-1, 0, 0, 0], [0, 0, 1, L.Y_C - L.Z_TABC], [0, 1, 0, L.R]])


def _cart_to_W(x0, x1, y0, y1, z0, z1):
    """a cartridge-coords box in the bay frame (both x signs if x0 >= 0 is given as a side)."""
    return box(-x1, -x0, L.Y_C - L.Z_TABC + z0, L.Y_C - L.Z_TABC + z1, L.R + y0, L.R + y1)


def wedge(half_deg, gap, y0, y1, rmax=400.0):
    """the bay's angular sector (bay radial = +Z), each side `gap` inside its plane."""
    a = math.radians(half_deg)
    n = np.array([math.cos(a), -math.sin(a)])          # outward normal of the +X plane (X, Z)
    pts = []
    for sx in (-1, 1):
        d = np.array([sx * math.sin(a), math.cos(a)]) * rmax - gap * np.array([sx * n[0], n[1]])
        pts.append(d)
    o = np.array([0.0, 0.0])
    base = [o + np.array([0, -gap / math.sin(a)])] + pts
    return hull([[p[0], y, p[1]] for p in base for y in (y0, y1)])


def _plane_box(side, t0, t1, y0, y1, rho0, rho1):
    """a box in a joint plane's frame: t along the plane's outward normal (t < 0 inside this module), rho along the plane
    (radial), Y along the axle. side +1: the module's +X plane (at +HALF), -1: its -X plane (mirrored)."""
    h = math.radians(L.HALF)
    m = box(t0, t1, y0, y1, rho0, rho1).transform([[math.cos(h), 0, math.sin(h), 0], [0, 1, 0, 0], [-math.sin(h), 0, math.cos(h), 0]])
    return m if side > 0 else m.mirror([1, 0, 0])

def _plane_cyl_t(side, t0, t1, y, rho, r, n=32):
    """a cylinder along the plane normal t (t0..t1) at (Y, rho)."""
    h = math.radians(L.HALF)
    c = mf.Manifold.cylinder(t1 - t0, r, r, n).rotate([0, 90, 0]).translate([t0, y, rho])
    c = c.transform([[math.cos(h), 0, math.sin(h), 0], [0, 1, 0, 0], [-math.sin(h), 0, math.cos(h), 0]])
    return c if side > 0 else c.mirror([1, 0, 0])

def _dovetail(side, grow):
    """v4c dovetail along Y (full module length): profile in the joint plane's (t, rho) frame, neck DT_W0 wide (rho) from
    inside this module to the plane gap, flaring to DT_W1 over DT_D (45 deg flanks) into the neighbour. side +1: the
    tongue on the module's +X plane; side -1 with grow > 0: the groove the neighbour's tongue slides in (its mirror),
    + grow clearance, open at both Y ends."""
    h = math.radians(L.HALF); g = L.JOINT_GAP; rc = L.DT_RHO_C
    w0, w1, d = L.DT_W0 / 2 + grow, L.DT_W1 / 2 + grow, L.DT_D + grow
    y0, y1 = (L.MOD_Y[0], L.MOD_Y[1]) if grow == 0 else (L.MOD_Y[0] - 5.0, L.MOD_Y[1] + 5.0)
    sg = 1.0 if grow == 0 else -1.0
    t_in = -4.0 if grow == 0 else 0.8
    neck = [[sg * t, y, rc + sr * w0] for t in (t_in, g + 0.01) for sr in (-1, 1) for y in (y0, y1)]
    flare = [[sg * t, y, rc + sr * w] for (t, w) in ((g, w0), (g + d, w1)) for sr in (-1, 1) for y in (y0, y1)]
    m = (hull(neck) + hull(flare)).transform([[math.cos(h), 0, math.sin(h), 0], [0, 1, 0, 0], [-math.sin(h), 0, math.cos(h), 0]])
    return m if side > 0 else m.mirror([1, 0, 0])


def bay_module_parts():
    """v4c universal bay module (Spark 17 Oct 02:00..02:02): one design, printed for every bay (11 at 30 deg), drawn at the
    top position. Sized for the Overseer cartridge (OV_*, E; its receiver not drawn), the Scout receiver centred on the
    gripper's tab line Y_C. Drawn: the drone receiver (v2/s2_cartridge.py receiver(): lead-ins, locating faces, end tabs,
    latch pockets, both contact shoe modules), the Scout back plate r 149.5..152.5 (TPU strip under it = the drone's
    preload), and the joints to its neighbours, long side to long side: 2 outer lugs per side (M5: clearance on +X, nut
    pocket on -X, head and nut reached from outside the ring) and one dovetail per side along the full length (tongue on +X,
    groove on -X) that holds the joint shut and lets one module slide out along +Y once its 4 bolts are out. Charging through the main contact shoes
    only (Spark 17 Oct 01:18). Env: its 30 deg sector less 0.3 per side, r MOD_R, Y MOD_Y, minus the bars' sweep, both heads
    swept to the seat (dilated 1) and the Scout cartridge's path."""
    yc = L.Y_C; y_lo, y_hi = yc - 41.8, yc + 45.8
    rec = _receiver_W()
    back = box(-24.3, 24.3, y_lo, y_hi, 149.5, 152.5)
    g = L.JOINT_GAP
    joints = None; joint_c = None
    rc = (L.LUG_RHO[0] + L.LUG_RHO[1]) / 2
    tp, tm_ = L.LUG_T
    hw = L.LUG_W / 2
    for y in L.LUG_Y:
        lug_p = _plane_box(+1, -tp, -0.05, y - hw, y + hw, *L.LUG_RHO)            # clearance lug (bolt head on its inside face)
        lug_m = _plane_box(-1, -tm_, -0.05, y - hw, y + hw, *L.LUG_RHO)           # nut lug
        nut = hull([[-tm_ - 1.0, y + r * math.cos(t), rc + r * math.sin(t)] for r in (L.M5_NUT_AF / math.sqrt(3) + 0.15,)
                    for t in np.linspace(math.pi / 6, 2 * math.pi + math.pi / 6, 6, endpoint=False)] +
                   [[-tm_ + L.M5_NUT_T, y + r * math.cos(t), rc + r * math.sin(t)] for r in (L.M5_NUT_AF / math.sqrt(3) + 0.15,)
                    for t in np.linspace(math.pi / 6, 2 * math.pi + math.pi / 6, 6, endpoint=False)])
        h_ = math.radians(L.HALF)
        nut = nut.transform([[math.cos(h_), 0, math.sin(h_), 0], [0, 1, 0, 0], [-math.sin(h_), 0, math.cos(h_), 0]]).mirror([1, 0, 0])
        cut = _plane_cyl_t(+1, -tp - 1.0, 1.0, y, rc, L.M5_CLR_R) + _plane_cyl_t(-1, -tm_ - 1.0, 1.0, y, rc, L.M5_CLR_R) + nut   # M5 through both; nut pocket on the far face
        j_ = lug_p + lug_m
        joints = j_ if joints is None else joints + j_
        joint_c = cut if joint_c is None else joint_c + cut
    for ys0, ys1 in L.DT_SEG:                                                      # two short engaged segments per side
        seg = box(-500, 500, ys0, ys1, -500, 500)
        tongue = (_dovetail(+1, 0.0) ^ seg) + _plane_box(+1, -tp, -g + 0.01, ys0, ys1, L.DT_RHO[0] + 2.5, L.DT_RHO[1])
        gblock = _plane_box(-1, -8.0, -g, ys0, ys1, *L.DT_RHO)
        joints = joints + tongue + gblock
    joint_c = joint_c + _dovetail(-1, 0.2)
    drawn = rec + back + joints
    # envelope
    ann = cylY(0, 0, L.MOD_Y[0], L.MOD_Y[1], L.MOD_R[1], 180) - cylY(0, 0, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1, L.MOD_R[0], 180)
    env = ann ^ wedge(L.HALF, g, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1)
    keep = cylY(0, 0, 70.0, 95.5, 136.5)
    for v in ("A", "B"):
        keep += swept_head_env(v, L.Z0_A_HOME, L.Z0_BAY + 1.5)
    keep += pack_path_v4(L.R)
    cuts = joint_c + keep
    env = env - keep + drawn
    regions = dict(
        back=box(-18.0, 18.0, yc - 36.0, yc + 40.0, 149.5, 152.5),
        pockets=_cart_to_W(20.3, 29.6, 4.2, 15.3, 29.7, 44.3) + _cart_to_W(-29.6, -20.3, 4.2, 15.3, 29.7, 44.3),
        shoes_px=_cart_to_W(-32.92, -20.8, 35.2, 52.0, 47.3, 68.0),          # X = -x: the cartridge's -x contact shoe module is at +X
        shoes_mx=_cart_to_W(20.8, 32.92, 35.2, 52.0, 47.3, 68.0),
        posts=rec ^ box(-30, 30, y_lo - 1, y_hi + 1, L.R + 2, L.R + 47),
        links=joints)                                                      # the joints to both neighbours: the module's support
    return dict(drawn=drawn, cuts=cuts, env=env, allowed=env - cuts, regions=regions, receiver=rec, joints=joints)


def pack_path_v4(z_seat):
    """the carried cartridge's path along a position's line from park to its seat (bottom at r z_seat): guide rails
    +-20.0 pass the receiver faces at +-20.3; ends 0.3 off the end tabs; top to the TPU strip (y 49.5)."""
    yc = L.Y_C
    r0 = L.Z0_A_HOME + L.CART_BOT_DZ - 1.0
    return box(-20.15, 20.15, yc - L.Z_TABC - 0.15, yc + L.CART_L - L.Z_TABC + 0.15, r0, z_seat + 49.4)


def bay_strip():
    """TPU 95A preload strip under the back plate (the drone's 1.5 strip): 36 x 76 x 1.5 at r 148..149.5."""
    return box(-18.0, 18.0, L.Y_C - 36.0, L.Y_C + 40.0, L.R + 48.0, L.R + 49.5)
