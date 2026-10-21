"""Head belt clamp v1 (17 Oct 2026): how each head's open GT2 belt is connected to its glide.

Replaces the v3 open toothed slot (5 teeth per end, belt could fall out the open side, no tension method).
One printed TOOTHED BED per head (drawn, part of the head) + one printed COVER PLATE + 1 M3 screw + 1 M3 heat-set insert.
  - Both belt ends lie in the bed, teeth down in GT2 grooves, 8 teeth each, butted against a flat centre land.
  - The cover plate (one flat block) is screwed down at the land; its key boss bottoms in a 0.6 recess in the land, which
    locates it and sets the squeeze on the belt backs (0.10, DEFAULT). Grooves stop the belt moving sideways.
  - Nothing sticks out of the bed, so it prints in any orientation with no support.
  - Teeth carry the pull (shear); the plate only stops the teeth lifting out.
Tension: the idler end slides in the bar's T-slot (M5 T-nuts); coarse steps by moving one end a tooth (2 mm loop).

Local frame (every function here): p across the belt thickness from the PITCH LINE, + toward the teeth (toward the
pulley centre); w across the belt width (0..6 = the belt); u along the run (= the head's u).
Head A: X = 27.65 + p, Y = 66 + w.   Head B (own frame): X = 48.65 + p, Y = 56 + w.
All numbers C unless tagged. Belt: GT2 2 mm, 6 wide, 1.38 total, tooth 0.75, PLD 0.254 (spec, E: CHECK on the belt).
python belt_clamp.py  -> coupon + plate STLs in out/, stress and placement report.
"""
import math, os
import numpy as np
import manifold3d as mf

# ---------------- belt (GT2 2 mm, spec) ----------------
PITCH = 2.0
PLD = 0.254          # pitch line to the tooth-side land
BACK = 0.63          # land to back
TOOTH_H = 0.75       # land to tooth tip
TOOTH_W = 1.11       # tooth width (tip radius 0.555)
BELT_W = 6.0
# ---------------- clamp (C / DEFAULT) ----------------
CLR = 0.05           # groove clearance round a tooth (DEFAULT)
SQUEEZE = 0.10       # plate into the belt back (DEFAULT)
TEETH = 8            # per end: stall 93 N (E) x 2 / ~30 N per tooth (E) -> 7, +1 (DEFAULT); proof test decides
END_L = TEETH * PITCH            # 16
LAND_L = 8.0                     # flat centre land: insert + plate boss (Ø6 head, Ø4 insert)
CLAMP_L = 2 * END_L + LAND_L     # 40
P_RIDGE = PLD + 0.05             # 0.304 ridge tops (land of the belt rests here)
P_GROOVE = PLD + TOOTH_H + CLR   # 1.054 groove bottoms
P_PLATE = P_RIDGE - BACK + SQUEEZE   # -0.226 plate face on the belt backs
P_BED1 = P_RIDGE + 7.0           # 7.304 bed back face (A: X 34.95, 4.4 off the return run's teeth)
CH_W = (-0.1, BELT_W + 0.1)      # channel (belt 6.0 in 6.2)
WALL_T = 2.0
P_WALL0 = -2.6                   # belt-line keep-out reaches this far toward the plate (no walls on the bed)
P_FL = (-7.0, None)              # plate back face (the plate is one block from here to P_PLATE)
FL_W = (-1.4, 7.7)               # plate across the width (A: Y 64.6..73.7: 0.6 over the 80T flange at Y 64, 0.3 under the glide skin at Y 74)
KEY_D, KEY_L = 0.6, 7.0          # key recess in the land (depth into the bed, length along u); boss 0.1 smaller each side
INSERT_D, INSERT_L = 4.0, 5.7    # M3 heat-set (E, CHECK on hand), from the land face
SCREW_CLR_D, HEAD_D, HEAD_H = 3.4, 6.0, 3.0   # M3 x 8..10 socket head, sunk flush in the plate
SCREW = "M3 x 8 (x 10 also fits)"

# head placement (from place(); C). The shared numbers live in arm_layout.py (CLAMP_DZ, CLAMP_DZ_B).
try:
    import arm_layout as _L
    _CZ = {"A": tuple(_L.CLAMP_DZ), "B": tuple(_L.CLAMP_DZ_B)}
except ImportError:
    _CZ = {"A": (-18.6, 21.4), "B": (-15.6, 24.4)}
HEADS = {
    "A": dict(x_pitch=27.65, y_belt=66.0, cz=_CZ["A"], bridge_u=(0.0, _CZ["A"][1])),
    "B": dict(x_pitch=48.65, y_belt=56.0, cz=_CZ["B"], bridge_u=(-4.0, _CZ["B"][1])),
}
GLIDE_SKIN_X = 20.3              # glide side skin inner face (bar +-20, 0.3)
Y_BAR0 = 74.0


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_p(w, u, p0, p1, r, n=48, tear=False):
    """cylinder along p (local x) at (w, u); tear=True adds a 45 deg roof toward +w (prints w up with no support)"""
    c = mf.Manifold.cylinder(p1 - p0, r, r, n).rotate([0, 90, 0]).translate([p0, w, u])
    if tear:
        s = r / math.sqrt(2)
        c += mf.Manifold.hull_points(np.array([[p, w + dw, u + du] for p in (p0, p1)
                                               for dw, du in ((s, -s), (s, s), (r * math.sqrt(2), 0.0))], dtype=np.float64))
    return c


def groove(uc, w0, w1):
    """one tooth groove centred at u = uc: straight sides (TOOTH_W + 2 CLR wide) from above the ridge tops down to a
    semicircle bottom at P_GROOVE. Extruded over w0..w1."""
    r = TOOTH_W / 2 + CLR
    pc = P_GROOVE - r
    side = box(P_RIDGE - 0.5, pc, w0, w1, uc - r, uc + r)
    tip = mf.Manifold.cylinder(w1 - w0, r, r, 48).rotate([-90, 0, 0]).translate([pc, w0, uc])
    return side + tip


def bed(u0, with_land_insert=True, ends=("lo", "hi")):
    """the toothed bed, local frame, from u0 (= the clamp's bottom) over CLAMP_L. A plain block with GT2 grooves across
    the belt width and a shallow key recess in the centre land: nothing sticks out of it, so it prints in any
    orientation without support (the head prints far end down, so the clamp's face is vertical)."""
    u1 = u0 + CLAMP_L
    body = box(P_RIDGE, P_BED1, CH_W[0] - WALL_T, CH_W[1] + WALL_T, u0, u1)
    cuts = None
    spans = []
    if "lo" in ends: spans.append(u0)
    if "hi" in ends: spans.append(u0 + END_L + LAND_L)
    for s in spans:
        for k in range(TEETH):
            g = groove(s + PITCH * (k + 0.5), CH_W[0], CH_W[1])
            cuts = g if cuts is None else cuts + g
    uc = u0 + END_L + LAND_L / 2
    key = box(P_RIDGE - 0.5, P_RIDGE + KEY_D, CH_W[0], CH_W[1], uc - KEY_L / 2 - 0.1, uc + KEY_L / 2 + 0.1)   # key recess
    cuts = key if cuts is None else cuts + key
    if with_land_insert:
        cuts += cyl_p(BELT_W / 2, uc, P_RIDGE, P_RIDGE + KEY_D + INSERT_L, INSERT_D / 2)
    body -= cuts
    # lead-in: 0.6 x 45 deg on the groove-side edges at both mouths
    for um, sgn in ((u0, 1), (u1, -1)):
        pts = [[P_RIDGE - 0.01, w, um - sgn * 0.01] for w in (CH_W[0] - WALL_T - 0.01, CH_W[1] + WALL_T + 0.01)] + \
              [[P_RIDGE + 0.6, w, um - sgn * 0.01] for w in (CH_W[0] - WALL_T - 0.01, CH_W[1] + WALL_T + 0.01)] + \
              [[P_RIDGE - 0.01, w, um + sgn * 0.6] for w in (CH_W[0] - WALL_T - 0.01, CH_W[1] + WALL_T + 0.01)]
        body -= mf.Manifold.hull_points(np.array(pts, dtype=np.float64))
    return body


def plate(u0):
    """cover plate, local frame: one flat block over the belt backs (face at P_PLATE = 0.10 into them), its key boss
    seated in the land's recess (locates it in u, w and rotation, and sets the squeeze), M3 head sunk flush."""
    u1 = u0 + CLAMP_L
    uc = u0 + END_L + LAND_L / 2
    pl = box(P_FL[0], P_PLATE, FL_W[0], FL_W[1], u0, u1)
    pl += box(P_PLATE - 0.01, P_RIDGE + KEY_D, CH_W[0] + 0.1, CH_W[1] - 0.1, uc - KEY_L / 2, uc + KEY_L / 2)   # key boss
    pl -= cyl_p(BELT_W / 2, uc, P_FL[0] - 1, P_RIDGE + KEY_D + 1, SCREW_CLR_D / 2)
    pl -= cyl_p(BELT_W / 2, uc, P_FL[0] - 1, P_FL[0] + HEAD_H, HEAD_D / 2)
    return pl


def belt_ends(u0):
    """the two seated belt ends as solids (for checks and pictures): back + land band + teeth."""
    out = None
    for s, e0, e1 in ((u0, u0 - 25.0, u0 + END_L), (u0 + END_L + LAND_L, u0 + END_L + LAND_L, u0 + CLAMP_L + 25.0)):
        b = box(P_RIDGE - BACK, P_RIDGE, 0.0, BELT_W, e0, e1)
        k = 0
        uu = s + PITCH * 0.5 - PITCH * math.ceil((s - e0) / PITCH)
        while uu < e1:
            if uu - TOOTH_W / 2 >= e0 - 1e-6 and uu + TOOTH_W / 2 <= e1 + 1e-6:
                r = TOOTH_W / 2
                t = box(P_RIDGE - 0.01, P_RIDGE + TOOTH_H - r, 0.0, BELT_W, uu - r, uu + r)
                t += mf.Manifold.cylinder(BELT_W, r, r, 32).rotate([-90, 0, 0]).translate([P_RIDGE + TOOTH_H - r, 0.0, uu])
                b += t
            uu += PITCH
        out = b if out is None else out + b
    return out


# ---------------- head integration (frames of arm_cad / arm_layout) ----------------
def to_head(m, h):
    H = HEADS[h]
    return m.translate([H["x_pitch"], H["y_belt"], 0.0])


def head_clamp(h):
    """clamp for head h in its own head frame. drawn = bed + walls only (the interface); the joint to the glide skin is
    GENERATED inside env (README: drawn = mounting + must-haves). Returns dict(drawn, plate, keep, env, ends):
    keep = plate + screw-driver path + belt line past both mouths (never generated into);
    ends = the two toothed spans (load regions: belt pull 93 N along -u on the lower end, +u on the upper, misuse)."""
    H = HEADS[h]
    u0, u1 = H["cz"]
    assert abs(u1 - u0 - CLAMP_L) < 1e-9
    drawn = to_head(bed(u0), h)
    xb1 = H["x_pitch"] + P_BED1
    y_lo = H["y_belt"] + CH_W[0] - WALL_T
    yb_top = H["y_belt"] + CH_W[1] + WALL_T
    bu0, bu1 = H["bridge_u"]
    pl = to_head(plate(u0), h)
    yc = H["y_belt"] + BELT_W / 2
    uc = u0 + END_L + LAND_L / 2
    keep = pl + box(H["x_pitch"] + P_FL[0] - 15.0, H["x_pitch"] + P_FL[0] + 0.01, yc - 4.0, yc + 4.0, uc - 4.0, uc + 4.0)   # driver
    keep += box(H["x_pitch"] + P_WALL0, H["x_pitch"] + P_RIDGE, H["y_belt"] + CH_W[0], H["y_belt"] + CH_W[1], u0 - 30.0, u1 + 30.0)  # belt line
    keep += box(H["x_pitch"] - 1.0, H["x_pitch"] + 1.6, H["y_belt"] - 0.5, H["y_belt"] + BELT_W + 0.5, u0 - 120.0, u0 - 0.01)       # run in
    keep += box(H["x_pitch"] - 1.0, H["x_pitch"] + 1.6, H["y_belt"] - 0.5, H["y_belt"] + BELT_W + 0.5, u1 + 0.01, u1 + 120.0)       # run out
    # env: round the bed (full length, under the bar band) + the joint up to the glide skin (B: only u >= -4 above Y 74,
    # where its parked joint stays 4 past A's glide skin)
    env = box(GLIDE_SKIN_X, xb1 + 0.5, y_lo - 0.5, Y_BAR0, u0, u1)
    env += box(GLIDE_SKIN_X, xb1 + 0.5, Y_BAR0 - 0.01, Y_BAR0 + 4.0, max(u0, bu0), bu1)
    ends = [to_head(box(P_RIDGE, P_BED1, CH_W[0] - WALL_T, CH_W[1] + WALL_T, s, s + END_L), h)
            for s in (u0, u0 + END_L + LAND_L)]
    return dict(drawn=drawn, plate=pl, keep=keep, env=env, ends=ends)


# ---------------- sizing ----------------
def plate_stress(F_end=93.0, lift=math.tan(math.radians(25.0))):
    """cover plate bending, worst section at the land edge (hole and counterbore inside the land): one end's lift
    (F_end x tan 25 deg, E) as a cantilever from the screw, acting at the end's centroid. PETG allowables per printopt
    (Xt 51 x 0.85 / SF): misuse 2.5 -> 17.3 MPa, sustained 10 -> 4.3 MPa (preload 20 N, E)."""
    # section at the land edge: the plate block (P_FL[0]..P_PLATE x FL_W); centroid and I about the w-axis
    rects = [(FL_W[1] - FL_W[0], P_FL[0], P_PLATE)]
    A = sum(b * (p1 - p0) for b, p0, p1 in rects)
    pc = sum(b * (p1 - p0) * (p0 + p1) / 2 for b, p0, p1 in rects) / A
    I = sum(b * (p1 - p0) ** 3 / 12 + b * (p1 - p0) * ((p0 + p1) / 2 - pc) ** 2 for b, p0, p1 in rects)
    c = max(pc - P_FL[0], P_PLATE - pc)
    arm = LAND_L / 2 + END_L / 2 - LAND_L / 2          # land edge to the end's centroid = END_L / 2
    out = {}
    for name, F, allow in (("stall 93 N, misuse", F_end, 51 * 0.85 / 2.5), ("preload 20 N, sustained", 20.0, 51 * 0.85 / 10)):
        M = F * lift * arm
        s = M * c / I
        out[name] = dict(lift_N=round(F * lift, 1), M_Nmm=round(M, 0), stress_MPa=round(s, 2), allow_MPa=round(allow, 1),
                         index=round(s / allow, 2))
    return out


def place():
    """clamp placement per head: the clamp's top is pushed as far out as the full stroke allows (most overlap with the
    44 long glide = stiffest joint), bottom checked against the motor pulley at park. Numbers from arm_layout (C)."""
    Z0_DRONE, Z0_PARK = 89.6, 32.0
    res = {}
    for h, zi, flange_r, zm in (("A", 120.0, 8.0, -59.0), ("B", 124.0, 9.0, -61.0)):
        top_max = zi - flange_r - 1.0 - Z0_DRONE           # idler flange (A) / cheek (B) + 1 at the drone
        bot = top_max - CLAMP_L
        motor_clr = (Z0_PARK + bot) - (zm + 8.0)            # clamp bottom over the 20T flange at park
        glide_overlap = max(0.0, min(44.0, top_max) - max(0.0, bot))
        res[h] = dict(cz=(round(bot, 1), round(top_max, 1)), motor_clear=round(motor_clr, 1), glide_overlap=round(glide_overlap, 1))
    # B's bridge at park must stay 4 past A's glide skin (arm X -24): -(32 + u) <= -28 -> u >= -4
    res["B"]["bridge_u_min"] = -4.0
    return res


def belt_lengths(T0=20.0, mu=0.0076):
    """open-belt cut per head (loop pitch length less the land) and the pluck frequency of the free return run."""
    pd = 2.0 * 20 / math.pi
    out = {}
    for h, zi, zm in (("A", 120.0, -59.0), ("B", 124.0, -61.0)):
        loop = 2 * (zi - zm) + math.pi * pd
        cut = loop - LAND_L
        span = (zi - zm) / 1000.0
        f = math.sqrt(T0 / mu) / (2 * span)
        out[h] = dict(loop=round(loop, 1), cut=round(cut, 0), span_mm=round(zi - zm, 0), pluck_Hz=round(f, 0))
    return out


def export(m, path, rot=None):
    import trimesh
    me = m.to_mesh()
    t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    if rot is not None:
        t.apply_transform(rot)
    t.apply_translation(-t.bounds[0])
    t.export(path)
    return t


def coupon():
    """proof-test coupon: one end (8 teeth) + the land with its insert + a hanging tab (Ø6.5 hole) beyond the land.
    Same profile, walls and plate as the heads. Prints back face down, grooves and walls up: no overhangs."""
    u0 = 0.0
    full = bed(u0, ends=("lo",))
    full = full ^ box(-50, 50, -50, 50, u0, u0 + END_L + LAND_L)          # keep the one end + the land
    tab = box(P_RIDGE, P_BED1, CH_W[0] - WALL_T, CH_W[1] + WALL_T, u0 + END_L + LAND_L - 0.01, u0 + END_L + LAND_L + 16.0)
    tab -= cyl_p(BELT_W / 2, u0 + END_L + LAND_L + 9.0, P_RIDGE - 1, P_BED1 + 1, 3.25)
    cb = full + tab
    pl = plate(u0) ^ box(-50, 50, -50, 50, u0, u0 + END_L + LAND_L)
    return cb, pl


if __name__ == "__main__":
    os.makedirs("out", exist_ok=True)
    print("placement", place())
    print("plate", plate_stress())
    print("belts", belt_lengths())
    for h in ("A", "B"):
        c = head_clamp(h)
        print(h, "clamp drawn %.2f g" % (c["drawn"].volume() / 1000 * 1.27), "plate %.2f g" % (c["plate"].volume() / 1000 * 1.27),
              "bodies", len(c["drawn"].decompose()), "bbox", [round(v, 2) for v in c["drawn"].bounding_box()])
        ov = (c["drawn"] ^ c["plate"]).volume()
        print(h, "plate vs bed overlap mm3", round(ov, 3))
        be = to_head(belt_ends(c and HEADS[h]["cz"][0]), h)
        print(h, "belt vs bed", round((be ^ c["drawn"]).volume(), 3), "belt vs plate", round((be ^ c["plate"]).volume(), 3))
    cb, pl = coupon()
    rx = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], float)   # coupon: bed back face on the bed (-p down), grooves up
    rp = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)   # plate: flange back on the bed (p up)
    export(cb, "out/belt_clamp_coupon_v1.stl", rx)
    export(pl, "out/belt_clamp_coupon_plate_v1.stl", rp)
    export(plate(0.0), "out/belt_clamp_plate_v1.stl", rp)          # one plate design for both heads (print 2)
    print("coupon %.2f g, plate %.2f g" % (cb.volume() / 1000 * 1.27, pl.volume() / 1000 * 1.27))
