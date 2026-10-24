"""Box hook latch v1 (23 Oct 2026, Spark: "a hooked latch, not a steel belly, no magnets").

Holds the seated drone down against the XT60 insert push (dimensions I "XT60 mate": 60 N E) during the swap and lets it
go for takeoff. Replaces the electromagnet and the steel belly plate. Four units, one behind each catch wheel on the two
seated collar faces (the faces over the 45 / 135 deg bays, drone +-y; dimensions G Catch interface): a square plunger
slides out of the face, square to it, just uphill of the seated wheel. The XT60 push lifts the drone, the wheel rolls
up-slope into the plunger's downhill face and stops. That load is across the plunger's travel, so it cannot drive the
plunger back in: the latch holds with the servo unpowered. A micro servo (MG90S class) moves the plunger 7 mm through a
pin in a pocket (scotch yoke). Nothing new on either drone: the wheels are the interface both drones already share.

Frames. Seat frame (S) = drone coordinates of a seated drone, as in catch_v4/funnel_panel.py (contact face
max(|x|, |y|) = 82.7 + z, bottom edge z 1.3, top z 77.2). Unit frame (L) per latch: origin on the contact face at the
plunger axis; u up the slope, w out of the face into the mouth, v along the face toward its centre line (unit A,
right-handed); unit B is A mirrored in v (the other half of the same face). Housing A sits on panel half x > 0 of the
+y face and on x < 0 of the -y face; B on the other two.

python3 nox/box/hook_latch.py            -> placement search, parts, checks, runs/hook_latch/v1/ (STLs, result.json)
python3 nox/box/hook_latch.py --motion   -> + motion.html / .mp4 / .gif / .png
"""
import json, math, os, sys
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(ROOT, "nox"), os.path.join(HERE, "catch_v4")]
OUT = os.path.join(ROOT, "runs", "hook_latch", "v1"); os.makedirs(OUT, exist_ok=True)
S2 = math.sqrt(2.0)

# ---------------------------------------------------------------- collar and wheel (dimensions G; funnel_panel.py, C)
K0, ZB, ZT = 82.7, 1.3, 77.2           # contact face y = K0 + z on the +y face; bottom / top edge z (C, funnel_panel)
T_TOP = (ZT - ZB) * S2                 # slope length bottom edge -> rim, 107.3 (C)
SKIN = 1.6                             # panel skin (C, rib_mask.json ts)
ROD_OUT, ROD_OFF = 112.0, 4.0          # lower-ring rod centre on the guard outline, 4.0 off the face when seated:
                                       # rod r 1.5 + wheel rim 2.5 proud (dimensions G Catch wheels, v4 set, C)
R_WHEEL = (4.0, 5.0)                   # v3.2 printed R 4 / v4 set R 5 (G, C); the latch is set by the larger
X_WHEEL = 100.0                        # wheel points (+-100, +-112) on the +-y faces (G Catch interface (1), C)
T_C = (ROD_OUT + ROD_OFF / S2 - (K0 + ZB)) * S2      # wheel contact point up the slope from the bottom edge, 43.6 (C)
GAP = 1.5                              # plunger downhill face to the R 5 wheel's uphill side (DEFAULT; R 4: 2.5)

# ---------------------------------------------------------------- plunger (C unless tagged)
# Error tolerance (Spark 24 Oct: "a bit more error tolerance"). The wheel may sit off its interface point by e along the
# slope (+ = uphill, toward the plunger) from cage build, panel print and seat scatter. The plunger rises with 0.5
# clearance for e <= GAP - 0.5; a wheel that sits downhill is held higher on the face, and the plunger keeps 1.0 of flat
# face above the wheel centre for e down to -(RISE - LEAD - 1 - R - GAP). A seated drone cannot move down the slope (the
# opposite face holds it), so the 45 deg lead-in on the downhill top edge only helps a drone resting above its seat; a
# plunger that meets a wheel stalls, its hall switch stays off and the box aborts the swap. Sideways the 12 wide plunger
# keeps the whole 3 wide rim on it for +-4.5. v1 as first drawn (8 wide, rise 7, gap 1.0) took e 0..+1 only.
PL_U, PL_V = 8.0, 12.0                 # section: 8 across the load (along the slope) x 12 along the face edge (DEFAULT)
PL = PL_U
RISE = 12.0                            # stroke (DEFAULT, from the tolerance band below)
LEAD = 2.0                             # 45 deg lead-in on the downhill top edge (DEFAULT)
U0 = T_C + max(R_WHEEL) + GAP + PL_U / 2   # plunger axis up the slope, 53.6 (C)
E_UP = GAP - 0.5                       # uphill wheel error with 0.5 rise clearance, 1.0 (C)
E_DOWN = RISE - LEAD - 1.0 - max(R_WHEEL) - GAP   # downhill wheel error taken, 2.5 (C)
E_SIDE = PL_V / 2 - 1.5                # sideways error with the whole rim on the plunger, 4.5 (C)
CL_GUIDE, CL_SKIN = 0.2, 0.5           # plunger in the guide bore / in the panel's skin window, per side (DEFAULT)
GUIDE_TOP = -SKIN - 0.3                # guide block top 0.3 under the skin's back
BOSS_H = 7.0                           # panel v7 bosses (M3 x 5.7 heat-set, hole 4.0 x 7) carry the flange
FL_T = 4.0                             # housing flange
W_FL0 = -SKIN - BOSS_H                 # flange panel-side face, -8.6
W_FL1 = W_FL0 - FL_T                   # flange far face, -12.6
WALL_U = 8.0                           # guide walls across the load (sized by the hand calc)
WALL_VN, WALL_VP = 3.0, 4.0            # guide walls along the load: valley side / servo side (pin window in it)
POCKET_W = 2.3                         # yoke pocket height (M2 pin o 2.0 + 0.3)

# ---------------------------------------------------------------- servo (MG90S class; E, listings disagree: D18 CHECK)
SV_L, SV_W, SV_H = 22.8, 12.2, 22.7    # body length / width / height (bottom to top face)
SV_SHAFT = 5.9                         # shaft axis from one short end
SV_DOME_R, SV_DOME_H = 5.8, 4.0        # gear dome + spline above the top face; horn sits on top of it
SV_EAR_SPAN, SV_EAR_T, SV_EAR_DROP = 32.3, 2.5, 4.0   # ears: span, thickness, top face to ear top
SV_EAR_HOLE = 13.85                    # ear holes from the body centre along its length
HORN_T, HORN_GAP = 1.6, 0.8            # horn thickness; horn to the guide's v+ face
PIN_D = 2.0                            # M2 screw in the horn hole = the yoke pin
THETA_MAX = 30.0                       # deg, servo swing each way from mid stroke (DEFAULT)
POST_V = 8.0                           # ear posts, thickness along v; the M2 slot is open toward the body (no thin web)
BOLT_PAD = 6.0                         # flange pad radius round each M3 (o 12)
BOLT_CLEAR = 3.5                       # M3 axis to any housing wall beside the head / hex key (head o 5.5, key o 3, C)
GUSSET = 8.0                           # ear post gusset: reach along the flange
BOSS_R, BOSS_GUSSET = 6.0, 6.0         # panel v7 bosses o 12 (pad rule) and their 2 gussets, 2.0 thick, reach 6 (C)
MAG_D, MAG_L = 3.0, 2.0                # hall magnet o 3 x 2 in the plunger's valley-side face (E, the A3144 kit's)
HALL_W, HALL_H, HALL_T = 4.4, 3.4, 1.0 # A3144 (TO-92S 4 x 3 x 1.5, E) locating recess in the guide's valley-side face
MAG_UP = 4.0                           # magnet centre over the plunger bottom: 2.45 under the pocket, 2.3 under the recess (C)

# ---------------------------------------------------------------- loads (E) and allowables (printopt PETG)
F_FUNC = 30.0      # N along the slope on one plunger: 55 N net up (60 push - 4.9 Scout), the two plungers on the near
                   # side take 67.5 % with the push 35 off centre (XT60 at the lead end) -> 18.6 N up -> 26.3 along the slope
F_MIS = 100.0      # N misuse on one plunger (someone lifts the latched drone by hand)
H_LOAD = RISE - LEAD - 1.0              # load height above the face: the highest held wheel centre in the band, 8 (C)
XT, ZT_ = 51.0 * 0.85, 25.5 * 0.85     # PETG in-layer / across-layer, MPa, x knockdown (printopt)
SF = {"flight": 4.0, "misuse": 2.5}


def unit_frame(face_sign, x_sign):
    """S <- L transform (4x4) for the latch at lateral x_sign * 100 on the face_sign * y face. Returns (T, mirrored)."""
    eu = np.array([0.0, 1.0, 1.0]) / S2; ew = np.array([0.0, -1.0, 1.0]) / S2
    o = np.array([X_WHEEL, K0 + ZB + U0 / S2, ZB + U0 / S2])
    ev = np.array([-1.0, 0.0, 0.0])                     # toward the face centre line from x = +100
    if x_sign < 0:
        o[0] = -o[0]; ev = -ev
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = eu, ev, ew, o
    mirrored = np.linalg.det(T[:3, :3]) < 0
    if face_sign < 0:
        Rz = np.diag([-1.0, -1.0, 1.0, 1.0]); T = Rz @ T
    return T, mirrored


UNITS = [("+y_x+", 1, 1), ("+y_x-", 1, -1), ("-y_x+", -1, 1), ("-y_x-", -1, -1)]


def face_t(p_s):
    """slope coordinate t (from the bottom edge) and lateral x of seat-frame points on / behind the +y face."""
    y, z = p_s[..., 1], p_s[..., 2]
    return ((y - (K0 + ZB)) + (z - ZB)) / S2


# ---------------------------------------------------------------- manifold helpers
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(r, z0, z1, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([0, 0, z0])


def cyl_v(r, u, w, v0, v1, n=40):      # cylinder along v at (u, w), v0..v1
    return mf.Manifold.cylinder(v1 - v0, r, r, n).rotate([-90, 0, 0]).translate([u, v0, w])


def cyl_w(r, u, v, w0, w1, n=40):      # cylinder along w at (u, v)
    return mf.Manifold.cylinder(w1 - w0, r, r, n).translate([u, v, w0])


def to_tm(m, T=None):
    me = m.to_mesh(); tm = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    if T is not None: tm.apply_transform(T)
    return tm


def vol(m):
    return m.volume() if not m.is_empty() else 0.0


# ---------------------------------------------------------------- placement search (the servo pose)
def servo_layout(r_h, side, body_dir):
    """servo shaft along v. r_h horn hole radius; side = shaft at u = side * r_h (horn points back to the plunger axis);
    body_dir = +1 / -1: the body runs from the shaft along +u / -u. Returns the layout dict or None if it breaks a rule."""
    theta = math.degrees(math.asin(RISE / 2 / r_h))
    if theta > THETA_MAX: return None
    drift = r_h * (1 - math.cos(math.radians(theta)))
    u_s = side * (r_h - drift / 2)                            # pin drift centred on the plunger axis
    v_horn0 = PL_V / 2 + CL_GUIDE + WALL_VP + HORN_GAP        # horn's guide-side face
    v_top = v_horn0 + HORN_T + SV_DOME_H                       # servo top face
    # shaft depth: horn, dome and body all below the flange's far face with 0.8 (C rule of the layout)
    w_s = W_FL1 - 0.8 - max(SV_DOME_R, SV_W / 2, r_h * math.sin(math.radians(theta)) + 3.0)
    if body_dir > 0: b0, b1 = u_s - SV_SHAFT, u_s + SV_L - SV_SHAFT
    else: b0, b1 = u_s - (SV_L - SV_SHAFT), u_s + SV_SHAFT
    bc = (b0 + b1) / 2
    holes = (bc - SV_EAR_HOLE, bc + SV_EAR_HOLE)
    ear_v = (v_top + SV_EAR_DROP, v_top + SV_EAR_DROP + SV_EAR_T)
    return dict(r_h=r_h, side=side, body_dir=body_dir, theta=theta, u_s=u_s, w_s=w_s, v_horn0=v_horn0, v_top=v_top,
                body=(b0, b1), holes=holes, ear_v=ear_v, drift=drift)


def build(lay):
    """housing A (unit frame), plunger (unit frame, retracted), context servo / horn / pin, and the cut / keep-out set."""
    gu, gv = PL_U / 2 + CL_GUIDE, PL_V / 2 + CL_GUIDE
    dr = lay["drift"]
    w_pin = lay["w_s"]                                        # pin depth at mid stroke
    pin_lo, pin_hi = w_pin - RISE / 2, w_pin + RISE / 2       # retracted / extended pin centre
    L_pl = -(pin_lo - POCKET_W / 2 - 3.0)                     # plunger length: 3.0 under the pocket when retracted
    g_bot = -L_pl + RISE                                      # guide bottom = plunger bottom when extended
    # guide block
    gb = box(-gu - WALL_U, gu + WALL_U, -gv - WALL_VN, gv + WALL_VP, g_bot, GUIDE_TOP)
    bore = box(-gu, gu, -gv, gv, g_bot - 1, GUIDE_TOP + 1)
    win = box(-PIN_D / 2 - dr / 2 - 1.5, PIN_D / 2 + dr / 2 + 1.5, gv - 0.5, gv + WALL_VP + 1,
              g_bot - 1, pin_hi + PIN_D / 2 + 1.0)              # pin window in the v+ wall
    # servo, posts, bar
    b0, b1 = lay["body"]; vt = lay["v_top"]; ws = lay["w_s"]
    body_w = (ws - SV_W / 2, ws + SV_W / 2)
    ev0, ev1 = lay["ear_v"]
    posts = []
    post_w0 = body_w[0] - 3.5                                  # posts run past the body to a tie bar under it
    po = (lay["holes"][0] - 4.4, lay["holes"][1] + 4.4)      # posts' outer faces
    for k, hu in enumerate(lay["holes"]):
        if k == 0: pu0, pu1 = po[0], b0 - 0.3
        else: pu0, pu1 = b1 + 0.3, po[1]
        posts.append(box(pu0, pu1, ev1, ev1 + POST_V, post_w0, W_FL1 + 0.01))
    bar = box(po[0], po[1], ev1, ev1 + POST_V, post_w0, body_w[0] - 0.5)
    frame = posts[0] + posts[1] + bar
    for k, sg in ((0, -1), (1, 1)):                            # gussets: post outer face -> flange, on the posts' far v face
        vg = (ev1 + POST_V - 3.0, ev1 + POST_V)
        frame = frame + mf.Manifold.batch_hull([box(po[k] - sg * 0.5, po[k] + sg * GUSSET, *vg, W_FL1 - 0.5, W_FL1 + 0.01) if sg > 0
                                                else box(po[k] + sg * GUSSET, po[k] + 0.5, *vg, W_FL1 - 0.5, W_FL1 + 0.01),
                                                box(min(po[k] - sg * 0.5, po[k]), max(po[k] - sg * 0.5, po[k]) + 0.01, *vg, post_w0, post_w0 + 0.5)])
    # bolts, every axis BOLT_CLEAR from any wall below the flange so the head and the hex key reach it (checked):
    # row 1 on the valley side, just either side of the guide block; row 2 past the posts (v), either side of the servo body
    v_lo = -gv - WALL_VN
    gx = gu + WALL_U + BOSS_R + 0.3                           # the panel boss (r 6, on the skin side) clears the guide
    v_b = (0.0, ev1 + POST_V + BOLT_CLEAR)                    # valley row at v 0: its pads stay off the valley corner
    bolts = [(-gx, v_b[0]), (gx, v_b[0]), (min(-gx, b0 - BOLT_CLEAR), v_b[1]), (max(gx, b1 + BOLT_CLEAR), v_b[1])]
    u_lo = min(min(b[0] for b in bolts) - BOLT_PAD, po[0] - GUSSET)
    u_hi = max(max(b[0] for b in bolts) + BOLT_PAD, po[1] + GUSSET)
    v_hi = v_b[1] + BOLT_PAD
    u_b = sorted(set(b[0] for b in bolts))
    # flange: full width over the guide block, from the valley-row pads' edge (v -6) outside it (valley corner kept clear)
    fl = box(-gu - WALL_U, gu + WALL_U, v_lo, v_hi, W_FL1, W_FL0) + box(u_lo, u_hi, v_b[0] - BOLT_PAD, v_hi, W_FL1, W_FL0)
    # horn sweep keep-out (the arm at every angle + hub) and the servo body keep-out, both cut from the housing
    horn_keep = mf.Manifold()
    for a in np.linspace(-lay["theta"] - 4, lay["theta"] + 4, 13):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        tip = (lay["u_s"] - lay["side"] * (lay["r_h"] + 3.0) * c, ws + (lay["r_h"] + 3.0) * s)
        seg = mf.Manifold.batch_hull([cyl_v(3.4, lay["u_s"], ws, lay["v_horn0"] - 0.6, lay["v_horn0"] + HORN_T + 0.6),
                                      cyl_v(2.6, tip[0], tip[1], lay["v_horn0"] - 0.6, lay["v_horn0"] + HORN_T + 0.6)])
        horn_keep = horn_keep + seg
    servo = box(b0, b1, vt, vt + SV_H, *body_w)
    ears = box((b0 + b1) / 2 - SV_EAR_SPAN / 2, (b0 + b1) / 2 + SV_EAR_SPAN / 2, ev0, ev1, *body_w)
    dome = cyl_v(SV_DOME_R, lay["u_s"], ws, vt - SV_DOME_H, vt + 0.01)
    keep = box(b0 - 0.3, b1 + 0.3, vt - 0.3, vt + SV_H, body_w[0] - 0.3, body_w[1] + 0.3) + \
        cyl_v(SV_DOME_R + 0.5, lay["u_s"], ws, vt - SV_DOME_H - 0.3, vt + 0.1) + \
        box((b0 + b1) / 2 - SV_EAR_SPAN / 2 - 0.3, (b0 + b1) / 2 + SV_EAR_SPAN / 2 + 0.3, ev0 - 4, ev1, body_w[0] - 0.3, body_w[1] + 0.3)
    housing = gb + fl + frame
    housing = housing - bore - win - horn_keep - keep
    w_mag_out = g_bot + MAG_UP                                 # magnet centre when the plunger is out
    housing = housing - box(-HALL_W / 2, HALL_W / 2, v_lo - 1, v_lo + HALL_T, w_mag_out - HALL_H / 2, w_mag_out + HALL_H / 2)
    for (bu, bv) in bolts: housing = housing - cyl_w(1.7, bu, bv, W_FL1 - 1, W_FL0 + 1)      # M3 clearance o 3.4
    # M2 x 12 through each post into a nut on the ear: o 2.4 slot open toward the servo body (the hole sits 2.45 from the
    # body end, a closed hole would leave a 0.95 web)
    housing = housing - box(lay["holes"][0] - 1.2, b0 + 0.5, ev1 - 1, ev1 + POST_V + 1, ws - 1.2, ws + 1.2) \
                      - box(b1 - 0.5, lay["holes"][1] + 1.2, ev1 - 1, ev1 + POST_V + 1, ws - 1.2, ws + 1.2)
    for hu in lay["holes"]: housing = housing - cyl_v(1.2, hu, ws, ev0 - 1, ev1 + POST_V + 1)
    # plunger (retracted: top flush with the face at w 0)
    pl = box(-PL_U / 2, PL_U / 2, -PL_V / 2, PL_V / 2, -L_pl, 0.0)
    pk = PIN_D / 2 + dr / 2 + 0.25                             # pocket half length along u (pin + drift + 0.25: walls >= 2.0)
    pocket = box(-pk, pk, PL_V / 2 - 5.5, PL_V / 2 + 1, pin_lo - POCKET_W / 2, pin_lo + POCKET_W / 2)   # 5.5 deep
    pl = pl - pocket
    pl = pl - mf.Manifold.cylinder(MAG_L + 0.2, MAG_D / 2 + 0.05, MAG_D / 2 + 0.05, 32).rotate([-90, 0, 0]) \
        .translate([0, -PL_V / 2 - 0.01, -L_pl + MAG_UP])       # magnet pocket, valley-side face, MAG_UP over the bottom
    ch = 0.8                                                    # uphill top edge (a wheel rolling down meets it)
    pl = pl - mf.Manifold.batch_hull([box(PL_U / 2 - ch, PL_U / 2 + 1, -PL_V, PL_V, -0.01, 1),
                                      box(PL_U / 2 + 0.5, PL_U / 2 + 1, -PL_V, PL_V, -ch - 0.6, 1)])
    pl = pl - mf.Manifold.batch_hull([box(-PL_U / 2 - 1, -PL_U / 2 + LEAD, -PL_V, PL_V, -0.01, 1),     # downhill lead-in
                                      box(-PL_U / 2 - 1, -PL_U / 2 - 0.5, -PL_V, PL_V, -LEAD - 0.6, 1)])
    # context
    horn = mf.Manifold.batch_hull([cyl_v(3.4, 0, 0, 0, HORN_T), cyl_v(2.2, -lay["side"] * (lay["r_h"] + 3.0), 0, 0, HORN_T)])
    pin = cyl_v(PIN_D / 2, -lay["side"] * lay["r_h"], 0, -(lay["v_horn0"] - (PL_V / 2 - 5.0)), HORN_T + 1.2)
    return dict(housing=housing, plunger=pl, servo=servo + ears + dome, horn=horn, pin=pin, bolts=bolts,
                L_pl=L_pl, g_bot=g_bot, pin_lo=pin_lo, pin_hi=pin_hi, u_range=(u_lo, u_hi), v_range=(v_lo, v_hi),
                bore=(gu, gv), keep=keep, horn_keep=horn_keep, u_b=u_b, v_b=v_b, w_mag_out=w_mag_out)


def check_place(lay, parts):
    """rules the housing must meet in the collar (C): behind the face, inside its own panel half (valley plane |x| = y
    with its 2.4 flange + 1.0), below the rim (t <= T_TOP - 4), clear of the seat block (36 from the centre line, t 14)."""
    T, _ = unit_frame(1, 1)
    tm = to_tm(parts["housing"], T)
    P = tm.vertices
    t = face_t(P); lat = P[:, 0]; depth = (P[:, 1] - (K0 + P[:, 2])) / S2   # > 0 behind the face
    valley = (P[:, 1] - lat).min()                                            # >= 0 inside the +y face's own half
    rim = t.max()
    seat_clear = np.hypot(lat - 36.0, t - 14.0).min()
    # the box round it (W: seated faces = W +-X = drone +-y, W Z = 144.5 + z with the panel frame's +2.1, E): the ring
    # and its brackets reach rho 183.6 (bracket_jA, C), the side sheets' inner faces X +-186 (box-stand.md, C)
    XW = P[:, 1]; ZW = 144.5 + P[:, 2] - 2.1
    ring_gap = float(np.hypot(XW, ZW).min() - 183.6)
    sheet_gap = float(186.0 - np.abs(XW).max())
    return dict(valley_margin=round(float(valley), 2), t_max=round(float(rim), 1), t_min=round(float(t.min()), 1),
                lateral=(round(float(lat.min()), 1), round(float(lat.max()), 1)), seat_block_dist=round(float(seat_clear), 1),
                ring_gap=round(ring_gap, 1), sheet_gap=round(sheet_gap, 1),
                ok=bool(valley >= 2.4 + 1.0 and rim <= T_TOP - 4.0 and seat_clear > 20.0 and ring_gap > 3.0 and sheet_gap > 3.0))


def search():
    rows = []
    for r_h in (11.5, 13.0, 14.5, 16.0, 17.5):
        for side in (1, -1):
            for bd in (1, -1):
                lay = servo_layout(r_h, side, bd)
                if lay is None:
                    rows.append(dict(r_h=r_h, side=side, body_dir=bd, ok=False, why="swing > 30 deg")); continue
                p = build(lay)
                ck = check_place(lay, p)
                g = (vol(p["housing"]) + vol(p["plunger"])) * 1.27e-3
                rows.append(dict(r_h=r_h, side=side, body_dir=bd, theta=round(lay["theta"], 1), g_unit=round(g, 2),
                                 footprint=[round(p["u_range"][1] - p["u_range"][0], 1), round(p["v_range"][1] - p["v_range"][0], 1)],
                                 **ck, why="" if ck["ok"] else "placement rule"))
    good = [r for r in rows if r["ok"]]
    g0 = min(r["g_unit"] for r in good)
    # equal mass within 0.05 g (the housing only shifts the servo along u): the smallest horn radius (DEFAULT: the holes
    # near the hub are on every stock horn; ties broken by mass)
    best = min([r for r in good if r["g_unit"] <= g0 + 0.05], key=lambda r: (r["r_h"], r["g_unit"]))
    return best, rows


# ---------------------------------------------------------------- checks
def strength(parts):
    """hand calcs (E loads, C geometry): plunger bending at the guide top, guide wall cantilever above the flange."""
    out = []
    for name, F, cls in (("functional 30 N", F_FUNC, "flight"), ("misuse 100 N", F_MIS, "misuse")):
        lever = H_LOAD - GUIDE_TOP                            # load on the wheel's equator to the guide top
        eng = GUIDE_TOP - parts["g_bot"]                      # engaged length at full rise = guide length
        M = F * lever
        s_pl = 6 * M / (PL_V * PL_U ** 2)                     # plunger prints lying: in-layer
        R_top = F * (lever + eng) / eng                       # guide top edge reaction
        lev_w = GUIDE_TOP - W_FL0                             # guide top above the flange's panel face
        bw = 2 * parts["bore"][1] + WALL_VN + WALL_VP         # wall width along v
        s_w = 6 * R_top * lev_w / (bw * WALL_U ** 2)          # guide wall root; housing prints on its v- face: in-layer
        allow_in = XT / SF[cls]
        out.append(dict(case=name, plunger_MPa=round(s_pl, 2), plunger_index=round(s_pl / allow_in, 2),
                        guide_top_N=round(R_top, 1), guide_wall_MPa=round(s_w, 2), guide_wall_index=round(s_w / allow_in, 2),
                        guide_wall_index_if_across=round(s_w / (ZT_ / SF[cls]), 2)))
    return out


def clearances(lay, parts):
    """drone vs plunger in the unit frame (C): wheel (R 4 / R 5) and node (cheeks r 4 round the pin) at the seat."""
    res = {}
    for R in R_WHEEL:
        # wheel centre: on the normal through the contact point T_C, R off the face
        wc_u = T_C - U0; wc_w = R
        up_side = wc_u + R                                    # wheel's uphill extreme (u)
        res[f"gap_R{int(R)}"] = round(-PL / 2 - up_side, 2)
    res["plunger_top_proud"] = RISE
    res["drone_lift_mm"] = round(res["gap_R5"] * S2, 2)       # vertical lift until the wheel meets the plunger (R 5)
    res["held_wheel_centre_off_face"] = max(R_WHEEL) + GAP   # a vertical lift moves the wheel off the face too (45 deg)
    res["tolerance_along_slope"] = dict(uphill_with_0p5_clearance=E_UP, uphill_touch=GAP, downhill=E_DOWN,
                                        drone_lift_max=round((GAP + E_DOWN) * S2, 2))
    res["tolerance_sideways_full_rim"] = E_SIDE
    # bolt access: head + key cylinder (r BOLT_CLEAR - 0.3) from the flange's far face 30 deep, vs housing and servo
    acc = []
    for (bu, bv) in parts["bolts"]:
        c = cyl_w(BOLT_CLEAR - 0.3, bu, bv, W_FL1 - 30, W_FL1 - 0.01)
        acc.append(round(vol(c ^ (parts["housing"] + parts["keep"])), 3))
    res["bolt_access_mm3"] = acc
    res["housing_vs_panel_bosses_mm3"] = round(vol(parts["housing"] ^ bosses_local(parts)), 3)
    res["plunger_vs_panel_bosses_mm3"] = round(max(vol(parts["plunger"] ^ bosses_local(parts)),
                                                   vol(parts["plunger"].translate([0, 0, RISE]) ^ bosses_local(parts))), 3)
    # hall: the magnet (out) faces the recess; gap magnet face -> recess floor (C)
    res["hall_gap_mm"] = round((parts["bore"][1] - PL_V / 2) + (WALL_VN - HALL_T), 2)
    res["plunger_engaged_at_rise"] = round(GUIDE_TOP - parts["g_bot"], 1)
    res["swing_deg"] = round(lay["theta"], 1)
    res["pin_drift_u"] = round(lay["drift"], 2)
    # moving parts vs the housing at both ends of the stroke and every 2 deg of horn swing (C, mm3 of overlap)
    pl_ext = parts["plunger"].translate([0, 0, RISE])
    res["plunger_vs_housing_mm3"] = round(max(vol(parts["housing"] ^ parts["plunger"]), vol(parts["housing"] ^ pl_ext)), 3)
    worst_pin = 0.0; worst_pin_pl = 0.0
    for a in np.arange(-lay["theta"], lay["theta"] + 0.01, 2.0):
        c, s_ = math.cos(math.radians(a)), math.sin(math.radians(a))
        Hl = (parts["horn"] + parts["pin"]).rotate([0, math.degrees(math.radians(a)) * lay["side"], 0]).translate([lay["u_s"], lay["v_horn0"], lay["w_s"]])
        worst_pin = max(worst_pin, vol(parts["housing"] ^ Hl))
        frac = (s_ / math.sin(math.radians(lay["theta"])) + 1) / 2          # plunger follows the pin's w
        worst_pin_pl = max(worst_pin_pl, vol(parts["plunger"].translate([0, 0, RISE * frac]) ^ Hl))
    res["horn_pin_vs_housing_mm3"] = round(worst_pin, 3)
    res["horn_pin_vs_plunger_mm3"] = round(worst_pin_pl, 3)
    hk = parts["housing"] ^ parts["horn_keep"]
    res["horn_vs_housing_mm3"] = round(vol(hk), 3)
    res["servo_vs_housing_mm3"] = round(vol(parts["housing"] ^ parts["keep"]), 3)
    return res


def tolerance_sweep(parts):
    """geometric check of the band (C): R 5 wheel seated on the face, moved e along the slope (+ uphill) and d sideways;
    the plunger rises in 0.25 steps. First contact on the lead-in = the wheel is cammed down-slope (OK); on the flat top
    = jam (the servo stalls, the box aborts). No contact: the drone lifts onto the plunger; held wheel centre R + GAP - e
    off the face must stay 1.0 under the top of the plunger's flat face (RISE - LEAD)."""
    R = max(R_WHEEL); u_c = T_C - U0
    out = []
    for e in (-3.0, -2.5, -2.0, -1.0, 0.0, 1.0, 1.5, 2.0):
        for d in (0.0, E_SIDE):
            wheel = mf.Manifold.cylinder(3.0, R, R, 48).rotate([-90, 0, 0]).translate([u_c + e, d - 1.5, R])
            hit = None
            for z in np.arange(0.0, RISE + 0.01, 0.25):
                ov = parts["plunger"].translate([0, 0, z]) ^ wheel
                if vol(ov) > 0.01:
                    b = ov.bounding_box(); hit = (round(float(z), 2), round(float((b[0] + b[3]) / 2), 2)); break
            if hit is None:
                h = R + GAP - e
                state = "held" if h <= RISE - LEAD - 1.0 + 1e-6 else "held too high"
            else:                                               # a seated wheel cannot move down the slope
                state = "stall: hall off, swap aborts"
            out.append(dict(e=e, d=d, state=state, first_contact=hit, held_centre=None if hit else round(R + GAP - e, 2)))
    return out


# ---------------------------------------------------------------- context (motion artifact)
def to_mf(m, T):
    return m.transform(np.asarray(T)[:3, :4].tolist())    # a left-handed T mirrors (housing A -> B), winding kept by manifold


def bosses_local(parts):
    """panel v7 bosses in the unit frame (C): o 12 x 7 from the skin's back, insert hole o 4.0, 2 gussets each 2.0 thick
    pointing away from the guide block along u (full boss height at the boss, 0 at 6 out)."""
    b = mf.Manifold()
    for (bu, bv) in parts["bolts"]:
        sg = 1.0 if bu > 0 else -1.0
        b = b + cyl_w(BOSS_R, bu, bv, W_FL0, -SKIN + 0.01)
        for dv in (-3.0, 3.0):
            b = b + mf.Manifold.batch_hull([box(bu - 0.01 if sg > 0 else bu - BOSS_R - BOSS_GUSSET, bu + BOSS_R + BOSS_GUSSET if sg > 0 else bu + 0.01,
                                                bv + dv - 1.0, bv + dv + 1.0, -SKIN - 0.5, -SKIN + 0.01),
                                            box(bu - 0.5, bu + 0.5, bv + dv - 1.0, bv + dv + 1.0, W_FL0, -SKIN + 0.01)])
        b = b - cyl_w(2.0, bu, bv, W_FL0 - 1, -SKIN - 0.5)
    return b


def bosses(parts, T):
    return to_mf(bosses_local(parts), T)


def drone_parts():
    """catch-interface drone (seat frame, C / E): 8 wheels R 5 on the face planes, lower + upper ring, plate, receiver,
    cartridge, 4 prop disks. Context only."""
    out = {}
    wheels = mf.Manifold(); cheeks = mf.Manifold()
    R = max(R_WHEEL)
    for k in range(4):
        for s in (1, -1):
            # +y face wheel at (s*100, ...): centre on the normal from the contact point, R off the face
            yc = K0 + ZB + T_C / S2 - R / S2; zc = ZB + T_C / S2 + R / S2
            w = mf.Manifold.cylinder(3.0, R, R, 40).rotate([0, 90, 0]).translate([s * X_WHEEL - 1.5, yc, zc])
            c = mf.Manifold.cylinder(7.8, 4.0, 4.0, 32).rotate([0, 90, 0]).translate([s * X_WHEEL - 3.9, yc, zc]) - \
                mf.Manifold.cylinder(3.4, 5.5, 5.5, 32).rotate([0, 90, 0]).translate([s * X_WHEEL - 1.7, yc, zc])
            wheels = wheels + w.rotate([0, 0, 90 * k]); cheeks = cheeks + c.rotate([0, 0, 90 * k])
    out["wheels"] = wheels
    zr = ZB + T_C / S2 + ROD_OFF / S2
    ring = mf.Manifold()
    pts = [(112, 100), (100, 112), (-100, 112), (-112, 100), (-112, -100), (-100, -112), (100, -112), (112, -100)]
    for z in (zr, zr + 16.0):
        for i in range(8):
            a, b = np.array(pts[i], float), np.array(pts[(i + 1) % 8], float)
            ring = ring + mf.Manifold.batch_hull([mf.Manifold.sphere(1.5, 12).translate([a[0], a[1], z]),
                                                  mf.Manifold.sphere(1.5, 12).translate([b[0], b[1], z])])
    for (x, y) in pts:
        # corner posts lean 8 inward over their 16 rise: a post straight up from a wheel node crosses the plunger zone
        # (Catch interface (7)); the v3 cage keeps that zone clear (E, context)
        k = 8.0 / math.hypot(x, y)
        ring = ring + mf.Manifold.batch_hull([mf.Manifold.sphere(1.5, 12).translate([x, y, zr]),
                                              mf.Manifold.sphere(1.5, 12).translate([x * (1 - k), y * (1 - k), zr + 16])])
    out["cage"] = ring + cheeks
    out["plate"] = box(-60, 60, -47, 47, 20.6, 24.1) + box(-54.1 - 4, -54.1 + 4, -54.1 - 4, -54.1 + 4, 24.1, 40) \
        + box(54.1 - 4, 54.1 + 4, -54.1 - 4, -54.1 + 4, 24.1, 40) + box(-54.1 - 4, -54.1 + 4, 54.1 - 4, 54.1 + 4, 24.1, 40) \
        + box(54.1 - 4, 54.1 + 4, 54.1 - 4, 54.1 + 4, 24.1, 40) \
        + box(-54.1, 54.1, -3, 3, 22, 23.5).rotate([0, 0, 45]) + box(-54.1, 54.1, -3, 3, 22, 23.5).rotate([0, 0, -45])
    props = mf.Manifold()
    for (x, y) in ((54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)):
        props = props + cyl(44.5, 42.8, 43.8, 48).translate([x, y, 0])
    out["props"] = props
    out["receiver"] = box(-45, 41, -25, 25, -2, 20.6) - box(-42.5, 38.5, -20.4, 20.4, -3, 18)
    return out


def cartridge():
    return box(-41.5, 37.5, -19, 19, -42.4, 5.6)          # cartridge v3 under the drone (E, archive receiver placement)


def head_block():
    return box(-30, 30, -22, 22, -60, -42.4)               # the swap head's top under the cartridge (context, E)


def main(motion=False):
    best, rows = search()
    lay = servo_layout(best["r_h"], best["side"], best["body_dir"])
    P = build(lay)
    ck = check_place(lay, P)
    st = strength(P)
    cl = clearances(lay, P)
    cl["tolerance_sweep"] = tolerance_sweep(P)
    # print-pose exports: housing on its guide-side v face (the guide walls then bend in-layer), plunger on its downhill face
    hA = P["housing"]; hB = P["housing"].mirror([0, 1, 0])
    # housing A print pose: v- face down -> rotate so +v is +z
    RA = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])   # (u, v, w) -> (u, -w, v)
    tA = to_tm(hA, RA); tA.apply_translation(-tA.bounds[0]); tA.export(os.path.join(OUT, "box_hook_latch_housing_A_v1.stl"))
    RB = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])   # mirrored copy: its v+ face down
    tB = to_tm(hB, RB); tB.apply_translation(-tB.bounds[0]); tB.export(os.path.join(OUT, "box_hook_latch_housing_B_v1.stl"))
    RP = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])   # plunger: u- (downhill, contact) face on the bed
    tP = to_tm(P["plunger"], RP); tP.apply_translation(-tP.bounds[0]); tP.export(os.path.join(OUT, "box_hook_latch_plunger_v1.stl"))
    # seat-frame copies for the fit check
    mass = dict(housing_g=round(vol(hA) * 1.27e-3, 2), plunger_g=round(vol(P["plunger"]) * 1.27e-3, 2))
    mass["set_of_4_g"] = round(4 * (mass["housing_g"] + mass["plunger_g"]), 1)
    # fit: retracted plunger top flush, extended plunger vs drone wheels / cheeks in the seat frame (all 4 units)
    D = drone_parts()
    fit = {}
    for name, fs, xs in UNITS:
        T, mir = unit_frame(fs, xs)
        hm = to_mf(hA, T)                                     # a left-handed frame makes it housing B
        ext = to_mf(P["plunger"].translate([0, 0, RISE]), T)
        fit[name] = dict(plunger_ext_vs_wheels_mm3=round(vol(ext ^ D["wheels"]), 3), plunger_ext_vs_cage_mm3=round(vol(ext ^ D["cage"]), 3),
                         housing_vs_drone_mm3=round(vol(hm ^ (D["wheels"] + D["cage"] + D["plate"])), 3))
    import funnel_panel as F
    collar = F.collar_world()
    for name, fs, xs in UNITS:
        T, mir = unit_frame(fs, xs)
        hm = to_mf(hA, T)
        fit[name]["housing_vs_collar_v6_mm3"] = round(vol(hm ^ collar), 1)   # v6 ribs in the way: the panel v7 window
    res = dict(part="hook_latch", version="v1", best=best, search=rows, layout={k: (round(v, 3) if isinstance(v, float) else v) for k, v in lay.items()},
               plunger=dict(L=round(P["L_pl"], 2), u0_on_slope=round(U0, 2), t_contact=round(T_C, 2), rise=RISE,
                            guide_bottom=round(P["g_bot"], 2), pin_retracted=round(P["pin_lo"], 2), pin_extended=round(P["pin_hi"], 2)),
               bolts_uv=[(round(a, 2), round(b, 2)) for a, b in P["bolts"]], flange=dict(u=[round(x, 2) for x in P["u_range"]], v=[round(x, 2) for x in P["v_range"]], w=[W_FL1, W_FL0]),
               placement=ck, strength=st, clearances=cl, mass=mass, fit=fit)
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(json.dumps({k: res[k] for k in ("best", "layout", "plunger", "placement", "strength", "clearances", "mass", "fit")}, indent=1, default=str))
    if motion:
        make_motion(lay, P, hA, hB, D, collar)
    return res


# ---------------------------------------------------------------- motion artifact
def horn_local(lay, r):
    """L <- horn transform at stroke fraction r (0 retracted, 1 out): shaft at (u_s, v_horn0, w_s), swing -theta..+theta."""
    ang = -lay["theta"] + 2 * lay["theta"] * r
    return trimesh.transformations.translation_matrix([lay["u_s"], lay["v_horn0"], lay["w_s"]]) @ \
        trimesh.transformations.rotation_matrix(math.radians(ang) * lay["side"], [0, 1, 0])


CLOSE_EYE, CLOSE_TGT = (150, 70, 75), (100, 118, 38)       # inside the mouth by the valley, along the +y face to its x+ wheel


def make_motion(lay, P, hA, hB, D, collar):
    from motion import Scene
    sc = Scene("Box hook latch v1", "4 plungers behind the seated faces hold the drone through the XT60 swap")
    # collar with the v7 spec (skin windows + bosses)
    col = collar
    for name, fs, xs in UNITS:
        T, _ = unit_frame(fs, xs)
        win = box(-PL / 2 - CL_SKIN, PL / 2 + CL_SKIN, -PL / 2 - CL_SKIN, PL / 2 + CL_SKIN, -SKIN - 2.0, 0.5)
        col = col - to_mf(win, T)
        col = col + bosses(P, T)
    sc.part("collar (v6 + v7 windows, bosses)", to_tm(col), group="box", color="#b9c2cc", printed=False, faces=40000)
    for name, fs, xs in UNITS:
        T, mir = unit_frame(fs, xs)
        sc.part(f"housing {name}", to_tm(hA, T), group="latch", color="#cc6644", printed=True)
        sc.part(f"plunger {name}", to_tm(P["plunger"], T), group="latch", color="#e8a33c", printed=True)
        sv = to_tm(P["servo"], T); sc.part(f"servo {name}", sv, group="latch", color="#3a6fd8", printed=False)
        sc.part(f"horn {name}", to_tm(P["horn"] + P["pin"], T @ horn_local(lay, 0.0)), group="latch", color="#eeeeee", printed=False)
    for k, m in D.items():
        sc.part(f"drone {k}", to_tm(m), group="drone", color={"wheels": "#44aa66", "cage": "#333333", "plate": "#556677",
                                                             "props": "#9fb8d0", "receiver": "#778899"}[k], printed=False, faces=12000)
    sc.part("cartridge", to_tm(cartridge()), group="swap", color="#9955bb", printed=False)
    sc.part("cartridge (charged)", to_tm(cartridge()), group="swap", color="#44aa99", printed=False)
    sc.part("swap head", to_tm(head_block()), group="swap", color="#aa8833", printed=False)

    lift = GAP * S2                                           # drone rises until the R 5 wheel meets the plunger
    DUR = 16.0
    def ease(a): a = min(max(a, 0.0), 1.0); return a * a * (3 - 2 * a)
    def drone_pose(t):
        # 0..2.5 descend to contact 20 off, 2.5..3.5 roll to the seat; 7.3..7.6 lift by the gap (XT60 push); 9..9.3 back;
        # 11..13 take off
        if t < 2.5:
            a = ease(t / 2.5); return (20.0, 14.0, 34.0 + 90 * (1 - a))
        if t < 3.5:
            a = ease((t - 2.5) / 1.0); return (20.0 * (1 - a), 14.0 * (1 - a), 34.0 * (1 - a))
        z = 0.0
        if 7.2 <= t < 9.0: z = lift * ease((t - 7.2) / 0.3)
        if 9.0 <= t < 9.4: z = lift * (1 - ease((t - 9.0) / 0.4))
        if t >= 11.0: z = 140.0 * ease((t - 11.0) / 2.5)
        return (0.0, 0.0, z)
    def rise(t):
        if t < 3.8: return 0.0
        if t < 4.2: return ease((t - 3.8) / 0.4)
        if t < 10.0: return 1.0
        if t < 10.4: return 1 - ease((t - 10.0) / 0.4)
        return 0.0
    def cart_pose(t):            # dead pack: with the drone until 4.6, pulled down and out
        dx, dy, dz = drone_pose(t)
        if t < 4.6: return (dx, dy, dz)
        return (-260.0 * ease((t - 5.6) / 0.8), 0.0, -90.0 * ease((t - 4.6) / 1.0))      # to a bay (out of frame)
    def new_cart(t):
        if t < 5.8: return (0.0, 0.0, -400.0)
        if t < 6.4: return (0.0, 0.0, -150.0 + 60.0 * ease((t - 5.8) / 0.6))
        if t < 7.6: return (0.0, 0.0, -90.0 + 90.0 * ease((t - 6.4) / 1.2) + drone_pose(t)[2])
        return tuple(np.add((0.0, 0.0, 0.0), drone_pose(t)))
    def head_pose(t):
        if t < 4.0: return (0.0, 0.0, -150.0)
        if t < 4.6: return (0.0, 0.0, -150 + 150 * ease((t - 4.0) / 0.6))
        if t < 5.6: return (0.0, 0.0, -90.0 * ease((t - 4.6) / 1.0))
        if t < 6.4: return (0.0, 0.0, -90.0)
        if t < 7.6: return (0.0, 0.0, -90.0 + 90.0 * ease((t - 6.4) / 1.2) + drone_pose(t)[2])
        if t < 9.0: return (0.0, 0.0, drone_pose(t)[2])
        return (0.0, 0.0, -150.0 * ease((t - 9.0) / 1.0))
    horn_T = {}
    for name, fs, xs in UNITS:
        T, _ = unit_frame(fs, xs)
        horn_T[name] = T
    def poses(t):
        out = {}
        d = drone_pose(t)
        for k in D: out[f"drone {k}"] = (d[0], d[1], d[2], 0, 0, 0)
        c = cart_pose(t); out["cartridge"] = (c[0], c[1], c[2], 0, 0, 0)
        n = new_cart(t); out["cartridge (charged)"] = (n[0], n[1], n[2], 0, 0, 0)
        h = head_pose(t); out["swap head"] = (h[0], h[1], h[2], 0, 0, 0)
        r = rise(t)
        for name, fs, xs in UNITS:
            T = horn_T[name]
            out[f"plunger {name}"] = T @ trimesh.transformations.translation_matrix([0, 0, RISE * r]) @ np.linalg.inv(T)
            # horn swing as a proper rotation in S (the mesh is stored at the retracted pose; a mirrored unit frame cancels)
            out[f"horn {name}"] = T @ horn_local(lay, r) @ np.linalg.inv(T @ horn_local(lay, 0.0))
        return out
    sc.timeline(poses, duration_s=DUR, fps=30)
    ts = np.arange(int(DUR * 30)) / 30.0
    sc.series("plunger out", [RISE * rise(t) for t in ts], unit="mm")
    sc.series("drone lift", [drone_pose(t)[2] if 3.5 <= t < 11 else 0.0 for t in ts], unit="mm")
    sc.series("hold per plunger", [F_FUNC if 7.4 <= t < 9.0 else 0.0 for t in ts], unit="N (E)")
    sc.view("latch close-up", eye=CLOSE_EYE, target=CLOSE_TGT)
    sc.view("behind the face", eye=(175, 230, -40), target=(95, 140, 30))
    sc.view("whole box top", eye=(0, -420, 330), target=(0, 0, 0))
    sc.write(os.path.join(OUT, "motion.html"))
    sc.video(os.path.join(OUT, "motion.mp4"), view="latch close-up")
    return sc


if __name__ == "__main__":
    main(motion="--motion" in sys.argv)
