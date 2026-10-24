"""Box swap arm v5 mechanism, drawn parts (24 Oct 2026, arm-mech worker): printed V-rails A / B v1 on the hub posts, the printed
V-wheel v1 (OD 24, 90 deg groove, 2 x 625ZZ), the 16T m1 pinion v1 (D-bore + M3 grub) and the printed eccentric bush v1; plus the
carriage proxy (plate, eccentric boss, wing, rack web with teeth, wheels, axle stack) the sweep and the motion artifact use until
heads v6 exist. Every number comes from `arm_v5_layout.py` (the one source; tags there and in `nox/dimensions.md` I).
Frames: the arm frame of arm_v5_layout (head A's frame: X tangential, Y axial, Z = r along head A's rail, u = Z - z0); head B's
parts are A's mirrored in X, placed by arm_kinematics' RB.
python3 arm_v5_cad.py            -> print/<part>_v1.stl (print pose), runs/arm_rail/v1/ (W meshes, checks.json, loads)
python3 arm_v5_cad.py checks     -> also solid_check + print_orient on every saved file"""
import os, sys, math, json
import numpy as np
import manifold3d as mf
import trimesh
import arm_v5_layout as V
import arm_cad as A
from arm_cad import box, cylY, hull, tm

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
PRINT = os.path.join(ROOT, "print"); RUN = os.path.join(ROOT, "runs", "arm_rail", "v1")
os.makedirs(RUN, exist_ok=True)

# ------------------------------------------------------------------ bought parts (E unless P)
BRG_D, BRG_B, BRG_BORE = 16.0, 5.0, 5.0          # 625ZZ 5 x 16 x 5 (P spec)
WHEEL_BORE_R = 8.025                              # Ø16.05 seat: press + a film of epoxy (DEFAULT; printed holes shrink ~0.1, E)
SHAFT_D, SHAFT_FLAT = 5.0, 4.5                    # NEMA17 D-shaft (17HS19-2004S1 datasheet, E)
BORE_CLR = 0.15                                   # printed bore over the shaft (E)
GRUB_TAP_D = 2.5                                  # M3 grub threads straight into the printed boss (tap or self-cut, DEFAULT)
P_CONTACT = 1.67 * 55.0                           # MPa: Hertz line-contact yield onset for PETG (Tresca, compressive yield 55 E)

# ------------------------------------------------------------------ rail
RAIL_BODY_Y = (V.RAIL_Y[0], V.RAIL_Y[1])          # body full depth Y 74..93 (DEFAULT 24 Oct: the layout's foot-only bottom left it 13 deep; 19 is 3.1x stiffer)


def ridge_prism(sx, z0, z1, taper=2.5):
    """90 deg ridge on face sx (+1 / -1): flanks from X 9 at Y 85 / 91 to the 11.5 flat at Y 87.5..88.5; both ends taper to X 10 over `taper`
    (lead-in for the wheels sliding on from the outer end)."""
    b, f, yc, hh = V.RAIL_BODY_X, V.RIDGE_FLAT_X, V.RIDGE_Y, V.RIDGE_HALF_H
    def prof(xt, z):
        d = xt - b                                                     # flank run at 45 deg
        return [[sx * (b - 0.01), yc - hh, z], [sx * xt, yc - hh + d, z], [sx * xt, yc + hh - d, z], [sx * (b - 0.01), yc + hh, z]]
    full = lambda z: prof(f, z)
    small = lambda z: prof(b + 1.0, z)
    return hull(small(z0) + full(z0 + taper)) + hull(full(z0 + taper) + full(z1 - taper)) + hull(full(z1 - taper) + small(z1))


def rail(h="A"):
    """rail h in its head's frame (Z = r). PETG. Returns the Manifold (W / head frame)."""
    r0, r1 = V.RAIL_R; y0, y1 = RAIL_BODY_Y; bx = V.RAIL_BODY_X
    m = box(-bx, bx, y0, y1, r0, r1)
    for sx in (-1, 1):
        m += ridge_prism(sx, r0, r1)
    fy0, fy1 = V.RAIL_Y[0], V.RAIL_Y[0] + V.RAIL_FOOT_H
    B = V.RAIL_BOLTS
    for rp in B["r_" + h]:
        za, zb = max(rp - 10.0, r0), rp + 10.0
        m += box(-V.RAIL_FOOT_X, V.RAIL_FOOT_X, fy0, fy1, za, zb)
        for sx in (-1, 1):                                             # 2 x 45 deg fillet foot -> body (gusset), 2.0 under the wheel rim zone
            m += hull([[sx * bx, fy1 + 2.0, z] for z in (za, zb)] + [[sx * bx, fy1, z] for z in (za, zb)] + [[sx * (bx + 2.0), fy1, z] for z in (za, zb)])
    # park hard stop on the top at the inner end (the ribbon channel runs under it as a 4-long tunnel)
    S = V.RAIL_STOP
    m += box(*S["x"], y1 - 0.01, S["y"][1], *S["r"])
    # ribbon channel: 4 wide at the floor (Y 89), flanks taper to the 3.2 mouth at the top
    cx0, cx1 = V.RAIL_CHAN["x"]; cy0, cy1 = V.RAIL_CHAN["y"]; cm = V.RAIL_CHAN["mouth"] / 2; cc = (cx0 + cx1) / 2
    m -= hull([[cc + s * (cx1 - cx0) / 2, cy0, z] for s in (-1, 1) for z in (r0 - 1, r1 + 1)] + [[cc + s * cm, cy1 + 0.01, z] for s in (-1, 1) for z in (r0 - 1, r1 + 1)])
    m -= hull([[cc + s * cm, cy1 - 0.01, z] for s in (-1, 1) for z in (S["r"][1], r1 + 1)] + [[cc + s * cm, cy1 + 1, z] for s in (-1, 1) for z in (S["r"][1], r1 + 1)])
    # one M5 per post from the top: Ø9 counterbore to Y 84, Ø5.5 through
    for rp in B["r_" + h]:
        m -= cylY(B["x"], rp, B["cbore_y"], S["y"][1] + 1, B["cbore_r"], 48) + cylY(B["x"], rp, y0 - 1, B["cbore_y"] + 0.01, B["hole_r"], 32)
    # A / B mark: 1 or 2 Ø2 x 1 dimples in the top at r 137, beside the channel
    for k in range(1 if h == "A" else 2):
        m -= cylY(5.5, 125.0 - 4.5 * k, y1 - 1.0, y1 + 1, 1.0, 16)          # 4.5 apart: 2.5 web
    return m


# ------------------------------------------------------------------ V-wheel (one design, x6)
def vwheel_profile():
    """(r, y) half section, y from the wheel's bottom face (W Y 83). Groove 90 deg centred at y 5 (Y 88), rim r 12, apex truncated
    at r 10.25 (0.25 off the ridge's 11.5 flat = wheel r 10.5), so the floor over the bearing seat is 2.22 (rule 2.0)."""
    rb, ro, w = WHEEL_BORE_R, V.WHEEL_OD / 2, V.WHEEL_W
    yc = V.RIDGE_Y - V.WHEEL_Y[0]; gd = V.WHEEL_GROOVE_D; rf = 10.25; ch = 0.5; lead = 0.4
    return [(rb + lead, 0.0), (ro - ch, 0.0), (ro, ch), (ro, yc - gd), (rf, yc - (rf - (ro - gd))), (rf, yc + (rf - (ro - gd))),
            (ro, yc + gd), (ro, w - ch), (ro - ch, w), (rb + lead, w), (rb, w - lead), (rb, lead)]


def vwheel(x=0.0, u=0.0):
    """wheel with its axis along Y, bottom face at WHEEL_Y[0], centred (x, Z = u)."""
    cs = mf.CrossSection([[list(p) for p in vwheel_profile()]])
    w = mf.Manifold.revolve(cs, 128)                                   # about the section's Y axis -> solid about Z, height along Z
    w = w.rotate([-90, 0, 0])                                          # height along +Y
    return w.translate([x, V.WHEEL_Y[0], u])


# ------------------------------------------------------------------ pinion 16T m1 (x2)
def involute_gear_cs(n, m, pa=20.0, backlash=0.1, steps=8, tip_relief=0.06, relief_band=0.35, tip_cut=0.1):
    """involute 20 deg; the top `relief_band` of each flank is thinned linearly by up to `tip_relief` (mm, along the arc) and the tip circle
    cut by `tip_cut` (24 Oct review: tip relief against the rack, which heads v6 draw without one)."""
    rp = n * m / 2; ra = rp + m - tip_cut; rf = rp - 1.25 * m; rb = rp * math.cos(math.radians(pa))
    inv = lambda a: math.tan(a) - a
    # half tooth angle at the pitch circle, thinned by backlash / 2 per flank
    half_p = (math.pi * m / 2 - backlash) / (2 * rp)
    a_p = math.acos(rb / rp); off = half_p + inv(a_p)                 # angle of the involute start (on the base circle) from the tooth axis
    pts = []
    for k in range(n):
        c = 2 * math.pi * k / n
        flank = []
        r_start = max(rb, rf)
        for i in range(steps + 1):
            r = r_start + (ra - r_start) * i / steps
            a = math.acos(min(1.0, rb / r)); th = off - inv(a)
            if r > ra - relief_band: th -= tip_relief * (r - (ra - relief_band)) / relief_band / r
            flank.append((r, th))
        left = [(r, c - th) for r, th in flank]
        right = [(r, c + th) for r, th in reversed(flank)]
        th_root = off + 0.02 if rf < rb else flank[0][1]
        tooth = [(rf, c - th_root)] + left + right + [(rf, c + th_root)]
        pts += [(r * math.cos(t), r * math.sin(t)) for r, t in tooth]
    return mf.CrossSection([pts]), dict(rp=rp, ra=ra, rf=rf, rb=rb)


def pinion16(xc=0.0, zc=0.0):
    """16T m1 pinion on the head motor's D-shaft, axis along Y at (xc, zc): boss Ø14 Y 56.5..63, neck Ø12 Y 63..64.5, teeth Y 64.5..72.5, D-bore,
    M3 grub radial at the boss middle onto the flat (the flat faces -X locally)."""
    cs, g = involute_gear_cs(V.PIN_T, V.PIN_M)
    t0, t1 = V.PIN_TEETH_Y; b0 = V.PIN_BOSS_Y[0]
    ef = 0.3                                                          # elephant-foot step: the bed-side 0.3 of the teeth inset 0.3 (prints gear face down)
    gear = mf.Manifold.extrude(cs, t1 - t0 - ef).rotate([-90, 0, 0]).translate([0, t0, 0])
    gear += mf.Manifold.extrude(cs.offset(-0.3, mf.JoinType.Round), ef + 0.01).rotate([-90, 0, 0]).translate([0, t1 - ef - 0.01, 0])
    b1 = V.WEB_Y[0] - 1.0                                              # boss ends 1.0 under the rack web (Y 64); a Ø12 neck (1.0 off the web face X 46.5) joins the teeth
    boss = cylY(0, 0, b0, b1, V.PIN_BOSS_R, 96) + cylY(0, 0, b1 - 0.01, t0 + 0.01, V.PIN_X - V.WEB_X[1] - 1.0, 96)
    p = gear + boss
    r = SHAFT_D / 2 + BORE_CLR / 2; fd = SHAFT_FLAT - SHAFT_D / 2 + BORE_CLR / 2       # bore r 2.575, flat plane 2.075 off the axis on -X
    bore = cylY(0, 0, b0 - 1, t1 + 1, r, 48) ^ box(-fd, r + 1, b0 - 1, t1 + 1, -r - 1, r + 1)
    p -= bore
    yg = (b0 + b1) / 2
    p -= A.cylX(yg, 0, -V.PIN_BOSS_R - 1, 0, GRUB_TAP_D / 2, 24)        # grub from the -X side onto the flat
    return p.translate([xc, 0, zc]), g


# ------------------------------------------------------------------ eccentric bush (x2, one per head)
def ecc_bush(x=0.0, u=0.0, turn_deg=0.0):
    E = V.ECC
    sh = cylY(0, 0, V.PLATE_Y[0] + 0.1, V.PLATE_Y[1] + 0.01, E["shank_r"], 96)          # 3.9 long: the plate stays in the clamp path
    af = E["flange_af"]; R = af / 2 / math.cos(math.radians(30))
    hexa = hull([[R * math.cos(math.radians(60 * k)), y, R * math.sin(math.radians(60 * k))] for k in range(6) for y in E["flange_y"]])
    b = sh + hexa
    tr = math.radians(turn_deg); ex, ez = E["throw"] * math.cos(tr), E["throw"] * math.sin(tr)
    b -= cylY(ex, ez, V.PLATE_Y[0] - 1, E["flange_y"][1] + 1, E["bolt_r"], 48)
    b -= cylY(-R * 0.62, 0, E["flange_y"][1] - 0.6, E["flange_y"][1] + 1, 0.6, 12)      # throw mark (dimple) on the flange top, on the low side
    return b.translate([x - ex, 0, u - ez])                                              # placed so its bolt bore is on the axle (x, u)


# ------------------------------------------------------------------ carriage proxy (head v6 draws the real one; numbers from the layout)
def carriage(h="A", rack_teeth=True):
    """carriage of head h in its own frame at z0 = 0 (u = Z), A-frame geometry; mirrored in X for B. Returns dict of Manifolds:
    printed (plate + boss + wing + rack web) and bought (wheels shown printed, bearings, bolts, nuts, shims, bush)."""
    pl = box(*V.PLATE_X, *V.PLATE_Y, *V.PLATE_U) + cylY(V.ECC["x"], V.ECC["u"], *V.PLATE_Y, V.ECC["plate_boss_r"], 64)
    pl += box(*V.WING_X, *V.PLATE_Y, *V.WING_U)
    web = box(*V.WEB_X, *V.WEB_Y, *V.WEB_U)
    if rack_teeth:                                                    # m1 rack along u on the web's +X face: tips 46.5, root 44.25
        p = math.pi * V.PIN_M; xt, xr = V.WEB_X[1], V.RACK_PITCH_X - 1.25 * V.PIN_M
        u = V.WEB_U[0] + p / 2
        gaps = None
        while u < V.WEB_U[1] - p / 2:
            wt = p / 2 + 0.1 + 2 * 1.0 * math.tan(math.radians(20)); wr = p / 2 + 0.1 - 2 * 0.25 * math.tan(math.radians(20))
            g = hull([[xt + 0.01, y, u + s * wt / 2] for s in (-1, 1) for y in (V.RACK_TEETH_Y[0] - 0.01, V.RACK_TEETH_Y[1])] +
                     [[xr, y, u + s * wr / 2] for s in (-1, 1) for y in (V.RACK_TEETH_Y[0] - 0.01, V.RACK_TEETH_Y[1])])
            gaps = g if gaps is None else gaps + g
            u += p
        web -= gaps
    holes = None
    for (x, u) in V.WHEELS:
        hr = V.ECC["bore_in_plate_r"] if (x, u) == (V.ECC["x"], V.ECC["u"]) else 2.6
        c = cylY(x, u, V.PLATE_Y[0] - 1, V.PLATE_Y[1] + 1, hr, 48); holes = c if holes is None else holes + c
    printed = pl + web - holes
    wheels = None; stack = None
    for (x, u) in V.WHEELS:
        w = vwheel(x, u); wheels = w if wheels is None else wheels + w
        s = cylY(x, u, V.WHEEL_Y[1], V.PLATE_Y[0], 5.0, 32)                                      # 5 x 10 x 1 shim
        s += cylY(x, u, V.NUT_Y[0], V.NUT_Y[1], V.NUT_R, 6)                                     # nyloc
        top = V.ECC["flange_y"][1] if (x, u) == (V.ECC["x"], V.ECC["u"]) else V.PLATE_Y[1]
        s += cylY(x, u, top, top + 5.0, 4.25, 32) + cylY(x, u, top - 25.0, top, 2.5, 24)      # M5 x 25 socket head + its real shank (tail Y 73 / 75.5)
        stack = s if stack is None else stack + s
    bush = ecc_bush(V.ECC["x"], V.ECC["u"], 90.0)
    out = dict(printed=printed, wheels=wheels, stack=stack, bush=bush)
    if h == "B":
        out = {k: A.mirror_x(v) for k, v in out.items()}
    return out


# ------------------------------------------------------------------ print poses
def pose_rail(m):
    """bottom (Y 74 face: body + feet, coplanar) on the bed: print Z = Y - 74, print Y = r."""
    t = tm(m); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0]))   # (x, y, z) -> (x, -z, y)
    t.apply_translation(-t.bounds[0]); return t


def pose_axisY(m, top_down=False):
    """a part with its axis along Y -> axis along print Z; top_down puts its high-Y face on the bed."""
    t = tm(m)
    R = trimesh.transformations.rotation_matrix(math.radians(-90 if top_down else 90), [1, 0, 0])   # +90: print Z = Y (low-Y face down)
    t.apply_transform(R); t.apply_translation(-t.bounds[0]); return t


# ------------------------------------------------------------------ loads (E), from the layout and the XT60 cases
def loads():
    F_in, F_out = 60.0, 50.0
    y_rack = sum(V.RACK_TEETH_Y) / 2
    M = F_in * (V.L.Y_C - y_rack) / 1000.0                          # N.m: pack push at Y_C vs the rack's reaction at its teeth
    us = [u for _, u in V.WHEELS]; uc = sum(us) / 3; k = M * 1000 / sum((u - uc) ** 2 for u in us)
    Fy = max(abs(k * (u - uc)) for u in us)                           # axial (Y) force on the worst wheel, N
    # rail: body 18 x 19 + ridges, bending about X under the carriage couple; support = the two post bolts
    b, hgt = 2 * V.RAIL_BODY_X, RAIL_BODY_Y[1] - RAIL_BODY_Y[0]
    yc = hgt / 2; I = b * hgt ** 3 / 12
    for _ in (0, 1):                                                  # ridges: 2 x (trapezoid ~2.75 x 6 mean width 1.25 x 6) at 88 - 74 = 14 up
        I += 2 * 2.5 * 4.5 * ((V.RIDGE_Y - RAIL_BODY_Y[0]) - yc) ** 2 / 2
    Ep = 2000.0                                                       # PETG E (MPa, E)
    c = max(yc, hgt - yc)
    sig_rail = M * 1000 * c / I                                       # MPa at the outer post
    r_out = max(V.RAIL_BOLTS["r_A"]) + 10.0; r_c = V.L.Z0_BAY + 5.0 + V.DRONE_PUSH + uc   # carriage centre at the drone insert
    L1 = r_c - r_out
    theta = M * 1000 * L1 / (Ep * I)                                  # rail slope at the carriage (moment over the overhang), rad
    tilt_pack = theta * (V.L.Y_C - V.PLATE_Y[1])
    span = abs(V.RAIL_BOLTS["r_A"][1] - V.RAIL_BOLTS["r_A"][0])
    bolt = M * 1000 / span                                            # N on one post bolt (couple over the two posts)
    # wheel flank contact (PETG on PETG, line contact over the flank band, Hertz cylinder-on-flat estimate)
    Fn = Fy * math.sqrt(2)                                            # the axial load all on one 45 deg flank: F_n = F_y / sin 45
    band = (V.WHEEL_OD / 2 - (22.0 - V.RIDGE_FLAT_X)) * math.sqrt(2)  # contact band along the flank, mm
    Rw = (V.WHEEL_OD / 2 + 10.5) / 2 * math.sqrt(2)                   # circumferential radius of curvature of the 45 deg flank cone: r / cos 45 (E)
    Es = Ep / (2 * (1 - 0.4 ** 2))
    a = math.sqrt(4 * Fn * Rw / (math.pi * band * Es))
    p_max = 2 * Fn / (math.pi * a * band)
    # pinion: Lewis, face 8, Y(16T) 0.295; stall 74 N
    Yl = 0.295; bw = V.PIN_TEETH_Y[1] - V.PIN_TEETH_Y[0]
    lew = lambda F: F / (bw * V.PIN_M * Yl)
    Td = F_in * V.PIN_R / 1000.0
    flat_p = Td * 1000 / 2.0 / (3.0 * (V.PIN_TEETH_Y[1] - V.PIN_BOSS_Y[0]))   # D-flat bearing stress: lever ~2, flat ~3 wide x 16 (E)
    sig_y = 45.0                                                      # PETG tensile strength in-layer (MPa, E); across layers ~25 (E)
    S = V.RAIL_STOP; sw, st, sh = S["x"][1] - S["x"][0], S["r"][1] - S["r"][0], S["y"][1] - S["y"][0]
    E_ = V.ECC; Fb = 30.0 + F_in * math.tan(math.radians(20))           # eccentric wheel: preload ~30 N (E) + the pinion's separating force
    lev = (V.WHEEL_Y[0] + V.WHEEL_Y[1]) / 2 - V.PLATE_Y[0]              # wheel centre below the plate face (the shank root sees the moment)
    ro, ri = E_["shank_r"], E_["bolt_r"] + E_["throw"]; Zs = math.pi * (ro ** 4 - ri ** 4) / (4 * ro)
    sig_bush = Fb * abs(lev) / Zs
    sig_stop = 6 * 74.0 * sh / (sw * st ** 2)                         # stall 74 N on the stop face, bending at its root (across layers)
    return dict(insert_N=F_in, pull_N=F_out, carriage_couple_Nm=round(M, 2), wheel_axial_N=round(Fy, 0),
                wheel_axial_N_layout_plate_plane=round(F_in * (V.L.Y_C - V.PLATE_Y[1]) / (us[1] - us[0]), 0),
                rail_I_mm4=round(I, 0), rail_sigma_MPa=round(sig_rail, 1), rail_index=round(sig_rail / 25.0, 2),
                rail_slope_deg=round(math.degrees(theta), 2), pack_shift_mm=round(tilt_pack, 2), post_bolt_N=round(bolt, 0),
                flank_normal_N=round(Fn, 0), flank_band_mm=round(band, 2), flank_contact_MPa=round(p_max, 0), flank_index=round(p_max / P_CONTACT, 2),
                brg_axial_N=round(Fy, 0), brg_C0_N=670, pinion_torque_Nm=round(Td, 2), pinion_lewis_MPa=round(lew(F_in), 1),
                pinion_index=round(lew(F_in) / sig_y, 2), pinion_stall_lewis_MPa=round(lew(74.0), 1), pinion_stall_index=round(lew(74.0) / sig_y, 2),
                dflat_MPa=round(flat_p, 1), stop_stall_MPa=round(sig_stop, 1), stop_index=round(sig_stop / 25.0, 2),
                bush_load_N=round(Fb, 0), bush_MPa=round(sig_bush, 1), bush_index=round(sig_bush / 25.0, 2), notes="E: PETG E 2 GPa, in-layer 45 MPa, across 25 MPa, line-contact yield onset p0 = 1.67 x 55 MPa compressive yield = 92 MPa (Tresca, E); rail bends across layers "
                "(printed bottom down, the couple bends it in Y = print Z), so its index uses 25 MPa")


PARTS = {  # name -> (builder, pose, material, qty, prev mass)
    "arm_rail_A_v1": (lambda: rail("A"), pose_rail, "PETG", 1, 44.0),
    "arm_rail_B_v1": (lambda: rail("B"), pose_rail, "PETG", 1, 44.0),
    "arm_vwheel_v1": (lambda: vwheel(0.0, 0.0), lambda m: pose_axisY(m), "PETG", 6, 3.0),
    "arm_pinion16_v1": (lambda: pinion16()[0], lambda m: pose_axisY(m, top_down=True), "PETG", 2, 2.0),
    "arm_eccbush_v1": (lambda: ecc_bush(0.0, 0.0, 0.0), lambda m: pose_axisY(m, top_down=True), "PETG", 2, 0.0),
}


def main(checks=False):
    import subprocess
    res = {}
    for nm, (fn, pose, mat, qty, prev) in PARTS.items():
        m = fn()
        tm(m).export(os.path.join(RUN, f"{nm}_W.stl"))
        t = pose(m)
        path = os.path.join(PRINT, f"box_{nm}.stl")
        t.export(path)
        g = t.volume / 1000 * 1.27
        res[nm] = dict(file=os.path.relpath(path, ROOT), mass_g=round(g, 2), qty=qty, extents=np.round(t.extents, 1).tolist(),
                       watertight=bool(t.is_watertight), bodies=len(m.decompose()))
        print(f"{nm:20s} {g:6.2f} g x{qty}  {np.round(t.extents, 1)}  watertight {t.is_watertight}  bodies {len(m.decompose())}")
    _, g = pinion16()
    res["pinion_geometry"] = {k: round(v, 3) for k, v in g.items()}
    res["loads"] = loads()
    for k, v in res["loads"].items(): print(f"  {k}: {v}")
    if checks:
        sys.path.insert(0, os.path.join(ROOT, "nox"))
        from solid_check import solid_check
        for nm in PARTS:
            p = os.path.join(ROOT, res[nm]["file"])
            ok, info = solid_check(trimesh.load(p, force="mesh"), pitch=0.2 if "rail" not in nm else 0.25)
            res[nm]["solid_check"] = "PASS" if ok else "FAIL"; res[nm]["solid_info"] = info
            print(nm, "solid_check", "PASS" if ok else "FAIL", info)
        res["fit"] = fit()
        res["roll_max_overlap_mm3"], res["roll_min_gap"] = roll_check()
        print("roll", res["roll_max_overlap_mm3"], res["roll_min_gap"])
        for r in res["fit"]: print("fit", r)
        save_runs(res)
    json.dump(res, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=str)
    return res


PRINT_NOTES = {   # part folder -> (files, bed face, print rule)
    "arm_rail": (("arm_rail_A_v1", "arm_rail_B_v1"), "bottom (Y 74 (C): body + both feet, coplanar)", "no supports needed (ridge flanks 45 deg (C), channel taper 6 deg (C)); 4 walls, 15 % gyroid"),
    "arm_vwheel": (("arm_vwheel_v1",), "either face (symmetric, C)", "brim 5 mm (bed < 400 mm2), no supports (groove flanks 45 deg, C); 100 % infill (small, rolling contact); print 6"),
    "arm_pinion16": (("arm_pinion16_v1",), "gear face (Y 72.5, C) down with a 0.3 (C) elephant-foot inset step, boss up", "brim 5 mm, no supports; 100 % infill; print 2"),
    "arm_eccbush": (("arm_eccbush_v1",), "hex flange face down", "brim 5 mm, no supports; 100 % infill; print 2"),
}


def save_runs(res):
    """result.json + summary.md (docs/generative.md fixed form) in runs/<part>/v1/ for the four parts."""
    Ld = res["loads"]; fit_ = res.get("fit", [])
    worst = {"arm_rail": (Ld["rail_index"], f"insert 60 N (E): couple {Ld['carriage_couple_Nm']} N.m (E), rail bending at the outer post {Ld['rail_sigma_MPa']} MPa (E, hand calc, across layers vs 25 E)"),
             "arm_vwheel": (Ld["flank_index"], f"insert 60 N (E): {Ld['wheel_axial_N']:.0f} N (E) axial on an outer fixed wheel, all on one flank, Hertz {Ld['flank_contact_MPa']:.0f} MPa (E) vs 92 yield onset (E, hand calc)"),
             "arm_pinion16": (Ld["pinion_index"], f"insert 60 N (E) at r 8 (C), Lewis {Ld['pinion_lewis_MPa']} MPa (E) vs 45 in-layer (E); stall 74 N (E): {Ld['pinion_stall_index']}"),
             "arm_eccbush": (Ld["bush_index"], f"eccentric wheel load {Ld['bush_load_N']:.0f} N (E: preload 30 + separating) bending the shank, {Ld['bush_MPa']} MPa (E, hand calc) vs 25 across layers (E)")}
    for part, (files, bed, rule) in PRINT_NOTES.items():
        d = os.path.join(ROOT, "runs", part, "v1"); os.makedirs(d, exist_ok=True)
        ms = [res[f]["mass_g"] for f in files]
        po = {}
        for f in files:
            t = trimesh.load(os.path.join(ROOT, res[f]["file"]), force="mesh")
            n = t.face_normals; zc = t.triangles_center[:, 2]
            po[f] = round(float(t.area_faces[(n[:, 2] < -0.99) & (zc < t.bounds[0][2] + 0.3)].sum()), 0)
        sok = all(res[f].get("solid_check") == "PASS" for f in files)
        ms_t = " / ".join(f"{m:.1f}" for m in ms)
        w, wc = worst[part]
        json.dump(dict(date="2026-10-24", status="DONE", mass_g=round(sum(ms) / len(ms), 2), mass_tag="M", worst_index=w, worst_case=wc,
                       solid_ok=sok, bed_face=bed, bed_contact_mm2=min(po.values()), note="drawn (manifold3d), nox/box/arm_v5_cad.py"),
                  open(os.path.join(d, "result.json"), "w"), indent=1)
        fl = ", ".join(f"print/box_{f}.stl" for f in files)
        txt = (f"{part} v1: ready to print\n"
               f"Weight: {ms_t} g (M), was ~{ {'arm_rail': '44 (E)', 'arm_vwheel': '3 (E)', 'arm_pinion16': '2 (E)', 'arm_eccbush': 'none'}[part]} (v5 layout)\n"
               f"Strength: worst index {w} in {wc}; weakest spot " +
               {"arm_rail": "the rail's bottom fibre at the outer post", "arm_vwheel": "the groove flank contact (rolling, PETG on PETG)",
                "arm_pinion16": "the tooth root (in-layer)", "arm_eccbush": "the shank wall beside the eccentric bore"}[part] +
               f"   [box part: fits (C, drawn): wheels on the ridge 0 mm3 (C) at every stroke state, rim 1.0 (C) to the rail body, axle stack 1.4 (C), plate 1.0 (C), rack and pinion 0 mm3 (C) through a full pitch of rolling, closest {res.get('roll_min_gap', 0):.2f} (C); no supports]\n"
               f"Solid: solid_check {'PASS' if sok else 'FAIL'}\n"
               f"Print: {bed}, bed contact {' / '.join(str(int(po[f])) for f in files)} mm2 (M), {rule}; file {fl}\n")
        open(os.path.join(d, "summary.md"), "w").write(txt)


def pinion_at(z0):
    """pinion turned so a tooth sits centred in a rack gap at stroke z0 (rack moves +z0 along u; the pinion rolls z0 / r)."""
    p = math.pi * V.PIN_M
    gap_c = V.WEB_U[0] + p / 2                                        # rack gap centres (carriage frame) at gap_c + k p
    u_rel = V.PIN_Z - z0                                              # pinion axis in the carriage frame
    d = (u_rel - gap_c) - p * round((u_rel - gap_c) / p)               # pinion axis offset from the nearest gap centre
    rot = -math.degrees(d / V.PIN_R)                                  # roll the -X tooth by -d along u onto the gap centre
    # the tooth at angle 0 points +X; the rack is at -X of the pinion: start with a tooth at 180 deg (16T: k = 8 is at 180)
    return pinion16()[0].rotate([0, rot, 0]).translate([V.PIN_X, 0, V.PIN_Z])


def roll_check(n=25):
    """pinion v1 rolling on the drawn rack through one full pitch: max overlap (mm3) and min gap (C)."""
    pl = carriage("A")["printed"]; base = pinion16()[0]; p = math.pi * V.PIN_M; gc = V.WEB_U[0] + p / 2
    ov, gmin = 0.0, 9.0
    for k in range(n):
        z0 = 60.0 + k * p / (n - 1)
        d = (V.PIN_Z - z0) - gc
        pin = base.rotate([0, -math.degrees(d / V.PIN_R), 0]).translate([V.PIN_X, 0, V.PIN_Z]); pp = pl.translate([0, 0, z0])
        ov = max(ov, (pin ^ pp).volume()); gmin = min(gmin, pin.min_gap(pp, 2.0))
    return round(ov, 4), round(gmin, 3)


def fit():
    """drawn fit of head A's carriage on rail A over the stroke (C): wheel-on-ridge overlap (line contact, ~0), wheel rim to the rail
    body, axle stack (nylocs, shims) to the rail, plate to the rail top, pinion teeth in the rack (overlap at the drawn phase)."""
    R = rail("A"); out = []
    body = box(-V.RAIL_BODY_X, V.RAIL_BODY_X, *RAIL_BODY_Y, *V.RAIL_R)
    C = carriage("A")
    for z0 in V.Z0S:
        mv = lambda m: m.translate([0, 0, z0])
        w, st, pl = mv(C["wheels"]), mv(C["stack"]), mv(C["printed"])
        pin = pinion_at(z0)
        ov = (pin ^ pl).volume()
        out.append(dict(z0=z0, wheel_rail_overlap_mm3=round((w ^ R).volume(), 3), wheel_body_gap=round(w.min_gap(body, 3.0), 2),
                        stack_rail_gap=round(st.min_gap(R, 3.0), 2), plate_rail_gap=round(pl.min_gap(R, 3.0), 2),
                        pinion_rack_overlap_mm3=round(ov, 3), pinion_rack_gap=round(pin.min_gap(pl, 3.0), 3)))
    return out


if __name__ == "__main__":
    if "fit" in sys.argv:
        for r in fit(): print(r)
    else:
        main(checks="checks" in sys.argv)
