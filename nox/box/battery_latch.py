"""Battery latch v1 (24 Oct 2026, Spark: "design the new latch system for all batteries so the grabber can follow it").

One latch + cartridge standard for every pack: Scout 4S 1500 (70 x 32 x 33, 165 g P) and Overseer 4S 6000 (148 x 48 x 45 P,
650 g E). Concept (nox/box/battery-latch.md, scored): squeeze latch v5 = the v2 squeeze latch kept (two sliders and one
pen spring in the cartridge, passive pockets in the receiver), made lower, sized for the Overseer and the XT60 pull, and
given one grip interface on both cartridges. The grabber follows it: one finger squeeze grips the tabs and pulls both
bolts in (unlatched, held); opening the fingers lets the spring throw the bolts into the pockets (latched). The insert
stroke alone also latches: the receiver feet's 30 deg lead-ins cam a free bolt in and it snaps out at the seat.

Cartridge coords c (as Scout bottom layout v1, `nox/scout/v3/bottom.md`): x across the pack (= -drone y), y up the install
axis (0 = flat bottom = ring r 100 in a bay), z along the pack (0 = closed end). Tab line z_T = the swap assembly's CG
(pack + cartridge + sliders + plug), so the gripper holds the pack at its CG and the drone puts it on the thrust axis.
Every receiver number is in seated cartridge coords. The Scout drone frame stand-in: d = (z_T - z, -x, y - 44.5).

python3 nox/box/battery_latch.py            -> parts, sweeps, hand calc, system manifest, runs/battery_latch/v1/
from battery_latch import latch_assembly; latch_assembly("scout", "latched") -> {name: manifold3d.Manifold} (c coords)
"""
import json, math, os, sys
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(ROOT, "nox")]
OUT = os.path.join(ROOT, "runs", "battery_latch", "v1")


# ======================================================================== helpers (scout_bottom.py style)
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def cyl(axis, a, b, r, t0, t1, n=40):
    c = mf.Manifold.cylinder(t1 - t0, r, r, n)
    if axis == "z": return c.translate([a, b, t0])
    if axis == "x": return c.rotate([0, 90, 0]).translate([t0, a, b])
    return c.rotate([-90, 0, 0]).translate([a, t0, b])
def hullp(pts): return mf.Manifold.hull_points(np.asarray(pts, np.float64))
def prism_xy(poly, z0, z1): return hullp([[x, y, z] for (x, y) in poly for z in (z0, z1)])
def prism_zy(poly, x0, x1): return hullp([[x, y, z] for (z, y) in poly for x in (x0, x1)])
def U(*ms):
    ms = [m for m in ms if m is not None and not m.is_empty()]
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else (ms[0] if ms else mf.Manifold())
def tm(m, T=None):
    me = m.to_mesh(); t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    if T is not None: t.apply_transform(T)
    return t
def vol(m): return 0.0 if m.is_empty() else m.volume()
def ov(a, b): return round(vol(a ^ b), 3)
def rot_y180(m, zc): return m.translate([0, 0, -zc]).rotate([0, 180, 0]).translate([0, 0, zc])
def mirx(m): return m.mirror([1, 0, 0])
RHO = {"PETG": 1.27, "TPU": 1.21}


# ======================================================================== packs (P / E)
PACKS = {
    # width (x) / height (y) / length (z), mass with its own leads and plug; plug + leads share of that mass (E)
    "scout":    dict(W=33.0, H=32.0, L=70.0, m=165.0, m_tag="P", m_plug=10.0),      # dimensions A (P); 32 tall as v4
    "overseer": dict(W=48.0, H=45.0, L=148.0, m=650.0, m_tag="E", m_plug=15.0),     # dimensions O (P size), L mass 650 E;
}                                                                                    # 45 tall, 48 across (DEFAULT: lower)

# ======================================================================== the common latch module (C; same on every cartridge)
WALL = 2.0
Y_FL1 = 12.0                      # pack bottom (v4 14.4): set by the slider top 11.5 + 0.5
REC_X = 15.5                      # finger recess half width: blades open 11..14 + 1.5 (head offset +-1.0)
REC_Z = 11.5                      # recess half length about z_T (tab +-11 + 0.5)
TAB_IN, TAB_OUT = 4.0, 10.0       # tab faces at rest; squeezed 0 / 6 (the tabs meet at x 0 = the stop): stroke 4.0
TAB_HZ = 11.0                     # tab z +-11 (22 wide: the 13 wide blades + the magnet ear)
TRAVEL = 4.0
BOLT_Y0, BOLT_T, BOLT_HZ = 6.5, 5.0, 9.0      # bar = bolt section: y 6.5..11.5, z +-9 (18 wide, the grip pull lever)
BOLT_Y1 = BOLT_Y0 + BOLT_T                   # 11.5
OVERLAP = 2.5                     # bolt into the pocket past the locating face (v4 3.0; head offset +-1.0 keeps 0.2 to spare)
UNDER = math.radians(10.0)        # bolt underside undercut over the overlap: pull-out drives it outward (v2)
TIP_CH = 1.0                      # 45 deg chamfer on the bolt tip's top outer edge (rides the feet lead-in)
GP_YC, GP_H, GP_W, GP_D, GP_TAPER, GP_CH = 4.0, 4.0, 8.0, 3.0, math.radians(5.0), 0.5   # grip pocket (J, + 0.5 mouth chamfer)
SPR_R, SPR_YC, SPR_DEPTH = 1.6, 9.6, 4.0      # pen spring U-slot (open top, under the pack): o 3.2, 4 deep per tab
MAG_D, MAG_DEPTH, MAG_X, MAG_Z = 2.0, 2.2, 7.0, 8.0   # o 2 x 2 N52 magnet, press-fit up into the tab bottom (head hall reads it)
CH_CLR = 0.35                     # slider in its channel, per side
TAB_CH = 1.0                      # 45 deg chamfer on the tab's bottom outer edge (a blade 1.0 inboard cams it in)
REC_CH = 1.0                      # 45 deg chamfer on the recess mouth's outer edges


def geom(pack):
    """every pack-dependent number (C), cartridge coords."""
    P = PACKS[pack]
    g = dict(pack=pack, **P)
    g["X_IN"] = P["W"] / 2 + 0.5
    g["X_OUT"] = g["X_IN"] + WALL
    g["X_RAIL"] = g["X_OUT"] + 1.0
    g["RF"] = g["X_RAIL"] + 0.3                         # receiver locating face
    g["X_TIP"] = g["RF"] + OVERLAP
    g["Y_LIP"] = Y_FL1 + P["H"] + 1.0                   # keeper lips 0.6, pack 1.0 under them
    g["Y_TOP"] = g["Y_LIP"] + 0.6
    g["RAIL_Y0"] = g["Y_TOP"] - 10.0
    g["Z_P0"] = 2.0
    g["Z_P1"] = g["Z_P0"] + P["L"] + 1.0
    g["Z_STOP"] = g["Z_P1"] + 3.0
    g["Z_CW0"], g["Z_CW1"] = g["Z_P1"] + 6.6, g["Z_P1"] + 8.6
    g["Z_X"] = g["Z_P1"] + 12.8                         # plug axis (v4 offsets from the pack end)
    g["Z_EW0"] = g["Z_P1"] + 17.0
    g["L_C"] = g["Z_P1"] + 20.0
    g["Y_XF_FACE"] = g["Y_TOP"] - 2.0
    g["Y_XF_REAR"] = g["Y_XF_FACE"] - 16.0
    g["Y_SH0"], g["Y_SH1"] = g["Y_XF_REAR"] - 2.5, g["Y_XF_REAR"]
    g["Y_PW1"] = g["Y_XF_REAR"] + 10.0
    g["Y_ZONE"] = g["Y_TOP"] - 3.8                      # cartridge top in the plug zone (under the receiver's XT60 lips)
    g["FLOOR_T"] = 2.0 if P["W"] < 40 else 3.0          # floor plate under the pack (35 g crash, hand calc)
    g["Z_T"] = (g["Z_P0"] + g["Z_P1"]) / 2              # replaced by the CG solve in place_tab_line()
    return g


# ======================================================================== slider (one part, both sides: 180 deg about y)
def slider(g, retract=0.0, side=1):
    """slider at rest (retract 0) or pulled in by `retract`; side +1 = x > 0 (the -x one is the same part turned 180 about y)."""
    zt, tip, rf = g["Z_T"], g["X_TIP"], g["RF"]
    tab = box(TAB_IN, TAB_OUT, 0.0, BOLT_Y1, zt - TAB_HZ, zt + TAB_HZ)
    bar = box(TAB_IN, tip, BOLT_Y0, BOLT_Y1, zt - BOLT_HZ, zt + BOLT_HZ)
    drop = OVERLAP * math.tan(UNDER)
    under = prism_xy([(rf, BOLT_Y0), (tip, BOLT_Y0 - drop), (tip, BOLT_Y0)], zt - BOLT_HZ, zt + BOLT_HZ)
    cham = prism_xy([(tip - TIP_CH, BOLT_Y1 + 0.01), (tip + 0.01, BOLT_Y1 + 0.01), (tip + 0.01, BOLT_Y1 - TIP_CH)],
                    zt - BOLT_HZ - 1, zt + BOLT_HZ + 1)
    s = U(tab, bar, under) - cham
    # tab bottom outer edge chamfer (a blade that lands 1.0 inboard cams the tab in, harmless)
    s = s - prism_xy([(TAB_OUT - TAB_CH, -0.01), (TAB_OUT + 0.01, -0.01), (TAB_OUT + 0.01, TAB_CH)], zt - TAB_HZ - 1, zt + TAB_HZ + 1)
    # grip pocket: 8 (z) x 4 (y) mouth on the outer face, 10 deg included taper, 3 deep, centred y 4, 0.5 mouth chamfer
    h1 = GP_H - 2 * GP_D * math.tan(GP_TAPER); w1 = GP_W - 2 * GP_D * math.tan(GP_TAPER)
    gp = hullp([[TAB_OUT + 0.01, GP_YC + sy * GP_H / 2, zt + sz * GP_W / 2] for sy in (-1, 1) for sz in (-1, 1)] +
               [[TAB_OUT - GP_D, GP_YC + sy * h1 / 2, zt + sz * w1 / 2] for sy in (-1, 1) for sz in (-1, 1)])
    gc = hullp([[TAB_OUT + 0.01, GP_YC + sy * (GP_H / 2 + GP_CH), zt + sz * (GP_W / 2 + GP_CH)] for sy in (-1, 1) for sz in (-1, 1)] +
               [[TAB_OUT - GP_CH, GP_YC + sy * GP_H / 2, zt + sz * GP_W / 2] for sy in (-1, 1) for sz in (-1, 1)])
    s = s - gp - gc
    # pen spring U-slot from the inner face, open to the top (the pack 0.5 above keeps the spring in)
    s = s - cyl("x", SPR_YC, zt, SPR_R, TAB_IN - 0.01, TAB_IN + SPR_DEPTH, 32) \
          - box(TAB_IN - 0.01, TAB_IN + SPR_DEPTH, SPR_YC, BOLT_Y1 + 1, zt - SPR_R, zt + SPR_R)
    # magnet seat in the tab bottom, on the ear beyond the blade (z_T + 8)
    s = s - cyl("y", MAG_X, zt + MAG_Z, MAG_D / 2, -0.01, MAG_DEPTH, 24)
    s = s.translate([-retract, 0, 0])
    return s if side > 0 else rot_y180(s, zt)


# ======================================================================== cartridge v5
def cartridge(g, clip=None):
    """cartridge v5 tray (one print, closed end down): walls, rails, floor, latch frames with the channels, recess, lips,
    windows, lead end with the XT60 plug pocket, shelf, set screw boss. clip: a manifold the tray is intersected with."""
    xi, xo, xr, rf = g["X_IN"], g["X_OUT"], g["X_RAIL"], g["RF"]
    yt, lc, zt = g["Y_TOP"], g["L_C"], g["Z_T"]
    t = box(-xo, xo, 0, yt, 0, lc)
    for s in (-1, 1):                                                        # guide rails (x datum), top chamfer 1.0
        a0, a1 = sorted((s * (xo - 0.01), s * xr))
        t = t + box(a0, a1, g["RAIL_Y0"], yt, 0, lc)
        t = t - prism_xy([(s * (xr - 1.0), yt + 0.01), (s * (xr + 0.01), yt + 0.01), (s * (xr + 0.01), yt - 1.0)], -1, lc + 1)
    fl0 = Y_FL1 - g["FLOOR_T"]
    t = t - box(-xi, xi, Y_FL1, yt + 1, g["Z_P0"], g["Z_P1"])               # pack cavity, open top
    t = t - box(-xi, xi, -1, fl0, g["Z_P0"], g["Z_P1"])                     # open under the floor plate
    t = t - box(-(xi - 2.0), xi - 2.0, -1, 4.01, g["Z_P0"], lc - 4.0)       # rim band 4 wide at the walls stays
    # latch zone: frames x REC_X..X_OUT up to the pack, recess between them open to the bottom
    t = t + box(-xo, -REC_X, 0, Y_FL1, zt - REC_Z, zt + REC_Z) + box(REC_X, xo, 0, Y_FL1, zt - REC_Z, zt + REC_Z)
    t = t - box(-REC_X, REC_X, -1, Y_FL1 + 0.01, zt - REC_Z, zt + REC_Z)
    drop = OVERLAP * math.tan(UNDER)
    for s in (-1, 1):
        t = t - prism_xy([(s * (REC_X - 0.01), -0.01), (s * (REC_X + REC_CH), -0.01), (s * (REC_X - 0.01), REC_CH)],
                         zt - REC_Z - 1, zt + REC_Z + 1)
        # channel: open to the pack inside the walls, a closed hole through the wall (its top edge carries the pull-out)
        a0, a1 = sorted((s * (REC_X - 1), s * (xi + 0.01)))
        t = t - box(a0, a1, BOLT_Y0 - CH_CLR, Y_FL1 + 1, zt - BOLT_HZ - CH_CLR, zt + BOLT_HZ + CH_CLR)
        a0, a1 = sorted((s * (REC_X - 1), s * (xo + 1)))
        t = t - box(a0, a1, BOLT_Y0 - CH_CLR, BOLT_Y1 + CH_CLR, zt - BOLT_HZ - CH_CLR, zt + BOLT_HZ + CH_CLR)
        # floor drop for the retracted undercut (it passes x rf - TRAVEL .. tip - TRAVEL)
        a0, a1 = sorted((s * (rf - TRAVEL - 0.5), s * (xo + 1)))
        t = t - box(a0, a1, BOLT_Y0 - CH_CLR - drop, BOLT_Y0 - CH_CLR + 0.01, zt - BOLT_HZ - CH_CLR, zt + BOLT_HZ + CH_CLR)
    for s in (-1, 1):                                                        # pack keeper lips
        t = t + prism_xy([(s * xi, g["Y_LIP"] - 0.6), (s * (xi - 0.6), g["Y_LIP"]), (s * (xi - 0.6), yt), (s * xi, yt)],
                         g["Z_P0"] + 4, g["Z_P1"] - 4)
    # long-wall windows (DEFAULT, mass): rows under and over the floor, 4 ribs, clear of the latch zone by 4
    for (wy0, wy1) in ((4.0, fl0 - 0.01 if fl0 - 4.0 >= 4 else 4.0), (Y_FL1 + 2.0, g["RAIL_Y0"] - 2.0)):
      if wy1 - wy0 >= 4:
        for (z0, z1) in ((g["Z_P0"] + 5, zt - REC_Z - 4), (zt + REC_Z + 4, g["Z_P1"] - 5)):
            n = max(1, int((z1 - z0) // 30))
            dz = (z1 - z0) / n
            for k in range(n):
                a, b = z0 + k * dz + (4.0 if k else 0), z0 + (k + 1) * dz
                if b - a > 8:
                    t = t - box(-xo - 2, -xi + 0.01, wy0, wy1, a, b) - box(xi - 0.01, xo + 2, wy0, wy1, a, b)
    # ---- lead end: end stop, lead loop, cross wall, plug pocket, shelf, end wall, set screws (v4 layout)
    t = t - box(-16.85, 16.85, g["Y_ZONE"], yt + 1, g["Z_P1"], lc + 1)
    t = t - box(-xi, xi, 4.0, g["Y_ZONE"] + 0.01, g["Z_P1"], g["Z_EW0"])
    t = t - box(-xi, xi, -1, 4.01, g["Z_P1"], lc - 4.0)
    t = t + box(-xi - 0.01, xi + 0.01, fl0, Y_FL1 + 5.0, g["Z_P1"] - 0.01, g["Z_STOP"])
    t = t + box(-xi - 0.01, xi + 0.01, g["Y_SH0"], g["Y_PW1"], g["Z_CW0"], g["Z_CW1"])
    t = t + box(-xi - 0.01, xi + 0.01, g["Y_SH0"], g["Y_SH1"], g["Z_CW0"], g["Z_EW0"])
    px, pz = XF_W / 2 + P_CLR, XF_D / 2 + P_CLR
    t = t + box(-px - 2.0, px + 2.0, g["Y_SH1"], g["Y_PW1"], g["Z_CW1"] - 0.01, g["Z_EW0"] + 0.01)
    t = t - box(-px, px, g["Y_SH1"], g["Y_PW1"] + 1, g["Z_X"] - pz, g["Z_X"] + pz)
    for sz in (-1, 1):                                                       # D-key: the two chamfered corners at x +
        z_a = g["Z_X"] + sz * pz
        t = t + hullp([[px - XF_KEY, y, z_a] for y in (g["Y_SH1"], g["Y_PW1"])] + [[px + 0.01, y, z_a] for y in (g["Y_SH1"], g["Y_PW1"])] +
                      [[px + 0.01, y, z_a - sz * XF_KEY] for y in (g["Y_SH1"], g["Y_PW1"])])
    t = t - box(-6.0, 6.0, g["Y_SH0"] - 1, g["Y_SH1"] + 0.01, g["Z_X"] - 2.6, g["Z_X"] + 2.6)   # wire slot
    t = t + box(-xi - 0.01, xi + 0.01, 0, g["Y_ZONE"], g["Z_EW0"], lc)       # end wall
    ss_y = g["Y_XF_REAR"] + 2.0
    t = t + box(-8.25, 8.25, ss_y - 4.25, ss_y + 4.25, lc - 0.01, lc + 3.0)  # set screw boss, 3.0 proud
    for sx in (-1, 1):
        t = t - cyl("z", sx * 4.0, ss_y, 1.25, g["Z_EW0"] - 0.5, lc + 4.0, 24)
    t = t + box(-xo, xo, 0, 4.0, 0, g["Z_P0"] + 2.0) + box(-xo, xo, 0, 4.0, lc - 4.0, lc)
    t = t + box(-xo, -(xi - 2.0), 0, 4.0, 0, lc) + box(xi - 2.0, xo, 0, 4.0, 0, lc)
    return t if clip is None else t ^ clip


# ======================================================================== XT60 (bought; v4 numbers, spec / E)
XF_W, XF_D, XF_H, XF_KEY, P_CLR = 15.5, 8.1, 16.0, 1.5, 0.15
XM_FW, XM_FD, XM_FT = 27.1, 12.1, 3.0
XM_IW, XM_ID, XM_IL = 13.3, 5.9, 7.5
XM_RW, XM_RD, XM_RL = 15.5, 8.1, 6.2
FLOAT, AX_PLAY = 1.0, 0.5

def xt60_female(g, dy=0.0):
    zx, f, r = g["Z_X"], g["Y_XF_FACE"], g["Y_XF_REAR"]
    h = box(-XF_W / 2, XF_W / 2, r, f, zx - XF_D / 2, zx + XF_D / 2)
    for sz in (-1, 1):
        z_a = zx + sz * XF_D / 2
        h = h - hullp([[XF_W / 2 - XF_KEY, y, z_a + sz * 0.01] for y in (r - 1, f + 1)] + [[XF_W / 2 + 0.01, y, z_a + sz * 0.01] for y in (r - 1, f + 1)] +
                      [[XF_W / 2 + 0.01, y, z_a - sz * XF_KEY] for y in (r - 1, f + 1)])
    h = h - box(-6.65, 6.65, f - XM_IL, f + 1, zx - 2.95, zx + 2.95)
    w = U(*[cyl("y", sx * 3.6, zx, 1.8, r - 8.0, r + 0.5) for sx in (-1, 1)])
    return (h + w).translate([0, dy, 0])

def xt60_male(g):
    zx, ym = g["Z_X"], g["Y_XF_FACE"] + 1.5
    f = box(-XM_FW / 2, XM_FW / 2, ym, ym + XM_FT, zx - XM_FD / 2, zx + XM_FD / 2)
    f = f - U(*[cyl("y", s * 10.25, zx, 1.6, ym - 1, ym + XM_FT + 1, 20) for s in (-1, 1)])
    i = box(-XM_IW / 2, XM_IW / 2, ym - XM_IL, ym, zx - XM_ID / 2, zx + XM_ID / 2)
    r = box(-XM_RW / 2, XM_RW / 2, ym + XM_FT, ym + XM_FT + XM_RL, zx - XM_RD / 2, zx + XM_RD / 2)
    w = U(*[cyl("y", sx * 3.6, zx, 1.8, ym + XM_FT + XM_RL - 0.5, ym + XM_FT + XM_RL + 5) for sx in (-1, 1)])
    return f + i + r + w


# ======================================================================== receiver latch block v1 (one design: drone and every bay)
Y_RB = 0.2                         # block bottom 0.2 over the flat bottom (bays: 1.7 to the heads' r 98.5)
CAP, LEAD = 3.0, math.radians(30.0)
HCAP = CAP / math.tan(LEAD)        # 5.20
FOOT_T, END_T, DECK_T = 4.0, 4.0, 3.0
PLATE_T_BY = {"scout": 2.5, "overseer": 3.0}   # side plate: root bending in the 35 g side hit (hand calc)
PK_GAP = 0.55                      # pocket floor over the lead-in top (v4)
REST_GAP = 0.55                    # bolt underside over the floor at the seat: >= the 10 deg floor's rise over the
                                   # retract path (2.8 tan 10 = 0.49), so a seated bolt slides in free (hard-stop seat)
PK_WALL = 3.0
BOSS_R, INS_D, INS_DEPTH = 5.0, 4.0, 6.7      # M3 x 5.7 heat-set, hole o 4.0 x 6.7, boss o 10 (wall 3.0)
LID_T = 4.0
STRIP_T, STRIP_REC = 2.4, 0.9

def rgeom(g):
    r = dict(g)
    rf = g["RF"]
    r["PLATE_T"] = PLATE_T_BY[g["pack"]]
    r["XW"] = rf + FOOT_T                                   # plates + feet half width
    r["FOOT_Y1"] = Y_RB + HCAP + 2.0
    r["PK_Y0"] = BOLT_Y0 - REST_GAP                         # pocket floor at the mouth (5.95)
    assert r["PK_Y0"] >= Y_RB + HCAP + PK_GAP - 1e-6, "lead-in runs into the pocket floor"
    r["PK_Y1"] = BOLT_Y1 + CH_CLR                           # pocket ceiling
    r["PK_D"] = OVERLAP + 0.3                               # pocket depth: tip 0.3 off the back wall
    r["PK_HZ"] = BOLT_HZ + 0.8
    r["Y_DK0"] = g["Y_TOP"] + STRIP_T - STRIP_REC           # deck underside (strip face = cartridge top)
    r["Y_DK1"] = r["Y_DK0"] + DECK_T
    r["Y_M"] = r["Y_DK1"] + LID_T                           # mounting plane = lid top = boss tops
    r["Z_E0"], r["Z_E1"] = -0.3, g["L_C"] + 0.3
    r["BX"] = rf + 5.3                                      # boss / lid bolt x (v4 25.6 on the Scout)
    r["MOUNT"] = [(sx * r["BX"], z) for sx in (-1, 1) for z in (r["Z_E0"] - END_T + BOSS_R, g["Z_X"])]   # closed end flush
    # with the end face; at the lead end the two lid bolts are the mount bolts (through partner + lid into the deck inserts)
    r["Y_XM"] = g["Y_XF_FACE"] + 1.5
    r["PKX_XW"], r["PKX_ZW"] = XM_FW / 2 + FLOAT, XM_FD / 2 + FLOAT
    r["Y_PCEIL"] = r["Y_XM"] + XM_FT + AX_PLAY
    r["LIP_Y0"] = r["Y_XM"] - 2.0
    return r

def pocket_cut(r):
    zt, rf = r["Z_T"], r["RF"]
    z0, z1 = zt - r["PK_HZ"], zt + r["PK_HZ"]
    yb = r["PK_Y0"] - r["PK_D"] * math.tan(UNDER)
    pk = hullp([[rf - 0.02, y, z] for y in (r["PK_Y0"], r["PK_Y1"]) for z in (z0, z1)] +
               [[rf + r["PK_D"], y, z] for y in (yb, r["PK_Y1"]) for z in (z0, z1)])
    ch = prism_xy([(rf - 0.02, r["PK_Y1"] + 1.0), (rf - 0.02, r["PK_Y1"] - 0.01), (rf + 1.0, r["PK_Y1"] - 0.01)], z0, z1)
    return pk + ch, yb

def receiver(r, clip=None):
    """receiver latch block v1, +x half drawn then mirrored (the XT60 parts are symmetric in x but for nothing)."""
    rf, xw, zt = r["RF"], r["XW"], r["Z_T"]
    z0, z1 = r["Z_E0"] - END_T, r["Z_E1"] + END_T
    parts = []
    for s in (-1, 1):
        def X(a, b): return sorted((s * a, s * b))
        a0, a1 = X(rf, rf + r["PLATE_T"]); parts.append(box(a0, a1, Y_RB + HCAP, r["Y_DK1"], z0, z1))          # side plate
        poly = [(rf + CAP, Y_RB), (xw, Y_RB), (xw, r["FOOT_Y1"]), (rf, r["FOOT_Y1"]), (rf, Y_RB + HCAP)]
        parts.append(prism_xy([(s * x, y) for x, y in poly], z0, z1))                                      # foot, 30 deg lead-in
        pk, yb = pocket_cut(r)
        a0, a1 = X(rf, rf + r["PK_D"] + PK_WALL)
        parts.append(box(a0, a1, Y_RB, r["PK_Y1"] + PK_WALL, zt - r["PK_HZ"] - PK_WALL, zt + r["PK_HZ"] + PK_WALL))
        # leg rib: pocket block to the deck, 6 x 6 post outside the plate at the tab line (pull-out path)
        a0, a1 = X(rf, rf + 6.0); parts.append(box(a0, a1, Y_RB, r["Y_DK1"], zt - 3.0, zt + 3.0))
    for zf, sg in ((r["Z_E0"], -1), (r["Z_E1"], 1)):                                                      # end plates
        y0 = r["RAIL_Y0"]
        pts = [(zf + sg * CAP, y0), (zf + sg * END_T, y0), (zf + sg * END_T, r["Y_DK1"]), (zf, r["Y_DK1"]), (zf, y0 + HCAP)]
        parts.append(prism_zy(pts, -xw, xw))
    parts.append(box(-xw, xw, r["Y_DK0"], r["Y_DK1"], z0, z1))                                             # deck
    # XT60E1-M float pocket body + lid bolt bosses; mount bosses
    zx = r["Z_X"]
    parts.append(box(-r["PKX_XW"] - 2.0, r["PKX_XW"] + 2.0, r["LIP_Y0"], r["Y_DK1"], zx - r["PKX_ZW"] - 2.0, zx + r["PKX_ZW"] + 2.0))
    for (bx, bz) in r["MOUNT"]:
        top = r["Y_DK1"] if abs(bz - zx) < 1e-6 else r["Y_M"]           # lead-end bosses stop under the lid
        bot = r["Y_DK1"] - INS_DEPTH - 2.0
        parts.append(cyl("y", bx, bz, BOSS_R, bot, top))
        parts.append(box(min(bx, np.sign(bx) * rf), max(bx, np.sign(bx) * rf), bot, r["Y_DK1"], bz - 3, bz + 3))
    R = U(*parts)
    # ---- cuts
    cuts = []
    for s in (-1, 1):
        pk, _ = pocket_cut(r); cuts.append(pk if s > 0 else mirx(pk))
        a0, a1 = sorted((s * (r["X_IN"] - 1.6), s * rf))
        cuts.append(box(a0, a1, r["Y_DK0"] - 1, r["Y_DK0"] + STRIP_REC, r["Z_P0"] - 1, r["Z_P1"] - 1))   # strip recesses
    wx = r["X_IN"] - 2.5
    cuts.append(box(-wx, wx, r["Y_DK0"] - 1, r["Y_DK1"] + 1, r["Z_P0"] + 2, r["Z_P1"] - 2))               # deck window
    cuts.append(box(-r["PKX_XW"], r["PKX_XW"], r["Y_XM"], r["Y_DK1"] + 10, zx - r["PKX_ZW"], zx + r["PKX_ZW"]))
    cuts.append(box(-8.5, 8.5, 0.0, r["Y_XM"] + 0.01, zx - 4.8, zx + 4.8))                                   # plug / male path
    for (bx, bz) in r["MOUNT"]:
        top = r["Y_DK1"] if abs(bz - zx) < 1e-6 else r["Y_M"]
        cuts.append(cyl("y", bx, bz, INS_D / 2, top - INS_DEPTH, top + 1))
    # side plate windows (DEFAULT, mass): between the latch leg and the ends, y foot + 3 .. rail - 2
    # side frame (DEFAULT, mass): a top band (rails' locating face) full length and three legs down to the feet per side
    # (both ends and the latch leg); the feet lead-ins at those stations capture the rigid cartridge
    wy0, wy1 = Y_RB - 1.0, r["RAIL_Y0"] - 2.0
    for (za, zb) in ((r["Z_E0"] + 10.0, zt - r["PK_HZ"] - PK_WALL - 2), (zt + r["PK_HZ"] + PK_WALL + 2, r["Z_E1"] - 10.0)):
        if zb - za > 8:
            for s in (-1, 1):
                a0, a1 = sorted((s * (rf - 0.5), s * (xw + 1)))
                cuts.append(box(a0, a1, wy0, wy1, za, zb))
    R = R - U(*cuts) - cart_path(r)
    return R if clip is None else R ^ clip

def lid(r):
    zx, bx = r["Z_X"], r["BX"]
    hz = max(BOSS_R + 1.0, 10.0)                                            # v4 lid: +-10 (3.0 bands neck-fail)
    l = box(-(bx + BOSS_R), bx + BOSS_R, r["Y_DK1"], r["Y_DK1"] + LID_T, zx - hz, zx + hz)
    for s in (-1, 1):
        a0, a1 = sorted((s * 9.26, s * (r["PKX_XW"] - 0.25)))
        l = l + box(a0, a1, r["Y_PCEIL"], r["Y_DK1"] + 0.01, zx - r["PKX_ZW"] + 0.25, zx + r["PKX_ZW"] - 0.25)
    l = l - box(-9.25, 9.25, r["Y_PCEIL"] - 1, r["Y_DK1"] + LID_T + 1, zx - 5.55, zx + 5.55)
    for s in (-1, 1):
        l = l - cyl("y", s * bx, zx, 1.7, r["Y_DK1"] - 1, r["Y_DK1"] + LID_T + 1, 24)
    return l

def strips(r):
    return U(*[box(*sorted((s * (r["X_IN"] - 1.55), s * (r["RF"] - 0.05))), r["Y_TOP"], r["Y_TOP"] + STRIP_T,
                   r["Z_P0"] - 0.8, r["Z_P1"] - 1.2) for s in (-1, 1)])

def cart_path(r):
    """space the cartridge (and its 3.0 capture) sweeps on the way in; nothing of the receiver in it."""
    rf, lc = r["RF"], r["L_C"]
    c = box(-rf, rf, -60, r["Y_DK0"], r["Z_E0"], r["Z_E1"])
    k = math.tan(LEAD)
    for s in (-1, 1):
        poly = [(0, -60), (rf + CAP + 62 * k, -60), (rf + CAP, Y_RB), (rf, Y_RB + HCAP), (0, Y_RB + HCAP)]
        c = c + prism_xy([(s * x, y) for x, y in poly], r["Z_E0"] - CAP - 1, r["Z_E1"] + CAP + 1)
    for zf, sg in ((r["Z_E0"], -1), (r["Z_E1"], 1)):
        poly = [(lc / 2, -60), (zf + sg * CAP, -60), (zf + sg * CAP, r["RAIL_Y0"]), (zf, r["RAIL_Y0"] + HCAP), (lc / 2, r["RAIL_Y0"] + HCAP)]
        c = c + prism_zy(poly, -rf, rf)
    return c


# ======================================================================== Scout drone frame, collar planes (Scout v3 system pass)
COLLAR_K = 84.8                     # contact faces max(|x|, |y|) = 84.8 + z (drone frame; Scout v3 system pass, C)
COLLAR_NEED = 7.0                   # receiver + cartridge >= 7.0 inside below z 20 (cage band 1.0..4.0 + 3.0 lead)
SCOUT_Z0 = -44.5                    # cartridge flat bottom at drone z -44.5 (ring r 100, as v4)
CG_D = np.array([0.0, 0.0, 4.2])    # Scout CG (E, dimensions H flow row)

def T_dc(g):
    """drone <- cartridge (4x4): d = (z_T - z, -x, y + SCOUT_Z0)."""
    return np.array([[0, 0, -1, g["Z_T"]], [-1, 0, 0, 0], [0, 1, 0, SCOUT_Z0], [0, 0, 0, 1.0]])

def collar_margin(P, yaw=5.0, tilt=2.0):
    """min over yaw / tilt about the CG of (84.8 + z) - max(|x|, |y|), points P (drone frame), z < 20 only."""
    P = P[P[:, 2] < 20.0]
    if not len(P): return 99.0
    worst = 99.0
    for a in (-yaw, 0, yaw):
        for tx in (-tilt, 0, tilt):
            for ty in (-tilt, 0, tilt):
                ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
                cx, sx = math.cos(math.radians(tx)), math.sin(math.radians(tx))
                cy, sy = math.cos(math.radians(ty)), math.sin(math.radians(ty))
                Rz = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]]); Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
                Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
                Q = (P - CG_D) @ (Rz @ Ry @ Rx).T + CG_D
                m = (COLLAR_K + Q[:, 2]) - np.maximum(np.abs(Q[:, 0]), np.abs(Q[:, 1]))
                worst = min(worst, float(m.min()))
    return worst

def collar_clip(g, inset):
    """the frustum max(|x|,|y|) <= 84.8 + z - inset (drone frame) in cartridge coords."""
    zlo, zhi = inset - COLLAR_K + 0.5, 40.0           # apex side kept positive (a negative half size flips the hull)
    pts = []
    for zz in (zlo, zhi):
        h = COLLAR_K + zz - inset
        for sx in (-1, 1):
            for sy in (-1, 1): pts.append([sx * h, sy * h, zz])
    Cd = hullp(pts)
    return Cd.transform(np.linalg.inv(T_dc(g))[:3])


# ======================================================================== assembly API (for nox/box/swap_system.py)
_CACHE = {}

def build(pack, clip_inset=None):
    """all meshes for one pack, cartridge coords; the Scout set is clipped to the collar planes (inset from the search)."""
    key = (pack, clip_inset)
    if key in _CACHE: return _CACHE[key]
    g = place_tab_line(pack)
    r = rgeom(g)
    clip = None
    if pack == "scout":
        inset = clip_inset if clip_inset is not None else SCOUT_INSET
        clip = collar_clip(g, inset)
    out = dict(g=g, r=r, clip=clip, cart=cartridge(g, clip), recv=receiver(r, clip), lid=lid(r), strips=strips(r),
               male=xt60_male(g))
    _CACHE[key] = out
    return out

def latch_assembly(pack="scout", state="latched"):
    """meshes in cartridge coordinates (manifold3d.Manifold). state: 'latched' (sliders at rest, bolts in the pockets),
    'gripped' (sliders squeezed 4.0 by the fingers, bolts in), 'free' (cartridge out of the receiver, sliders at rest)."""
    B = build(pack)
    g = B["g"]
    rt = 0.0 if state in ("latched", "free") else TRAVEL
    d = {"cartridge": B["cart"], "slider_px": slider(g, rt, 1), "slider_nx": slider(g, rt, -1), "pack": pack_body(g),
         "xt60_plug": xt60_female(g), "receiver": B["recv"], "receiver_lid": B["lid"], "receiver_strips": B["strips"],
         "xt60e1m": B["male"]}
    if state == "free":
        for k in ("receiver", "receiver_lid", "receiver_strips", "xt60e1m"): d.pop(k)
    return d

def pack_body(g):
    return box(-g["W"] / 2, g["W"] / 2, Y_FL1, Y_FL1 + g["H"], g["Z_P0"] + 0.5, g["Z_P1"] - 0.5)


# ======================================================================== tab line on the swap CG
SPRING_G, MAG_G = 0.3, 0.06          # pen spring, o 2 x 2 magnet (E)

def swap_cg(g):
    """mass (g) and z of the swap assembly (pack + plug + cartridge + 2 sliders + spring + magnets), cartridge coords."""
    items = []
    m_body = g["m"] - g["m_plug"]
    items.append((m_body, np.array([0.0, Y_FL1 + g["H"] / 2, (g["Z_P0"] + g["Z_P1"]) / 2])))
    items.append((g["m_plug"], np.array([0.0, g["Y_XF_REAR"], g["Z_X"]])))
    for nm, m in (("cart", cartridge(g)), ("s1", slider(g, 0, 1)), ("s2", slider(g, 0, -1))):
        t = tm(m)
        items.append((t.volume / 1000 * RHO["PETG"], np.asarray(t.center_mass)))
    items.append((SPRING_G + 2 * MAG_G, np.array([0.0, SPR_YC, g["Z_T"]])))
    M = sum(a for a, _ in items)
    c = sum(a * b for a, b in items) / M
    return M, c, items

def place_tab_line(pack, it=3):
    g = geom(pack)
    for _ in range(it):
        M, c, _ = swap_cg(g)
        g["Z_T"] = round(float(c[2]), 2)
    g["SWAP_M"], g["SWAP_CG"] = M, c
    return g


# ======================================================================== checks
def insertion_sweep(B, side_off=0.0, dz=1.0, depth=50.0):
    """cartridge from `depth` below seat to the seat. Sliders free: at each step the least retraction (bisection) that
    clears the receiver = what the feet lead-ins cam them to. Returns rows (d, retract) and the worst overlap."""
    g, R = B["g"], B["recv"] + B["lid"] + B["strips"] + B["male"]
    C = B["cart"] + xt60_female(g)
    rows, worst_c = [], 0.0
    d = -depth
    while d <= 1e-6:
        Cd = C.translate([side_off, d, 0])
        worst_c = max(worst_c, vol(Cd ^ R))
        rr = []
        for s in (1, -1):
            lo, hi = 0.0, TRAVEL
            if vol(slider(g, 0, s).translate([side_off, d, 0]) ^ R) < 0.01: rr.append(0.0); continue
            if vol(slider(g, hi, s).translate([side_off, d, 0]) ^ R) >= 0.01: rr.append(None); continue
            for _ in range(10):
                mid = (lo + hi) / 2
                if vol(slider(g, mid, s).translate([side_off, d, 0]) ^ R) < 0.01: hi = mid
                else: lo = mid
            rr.append(round(hi, 2))
        rows.append((round(d, 2), rr))
        d += dz
    return rows, round(worst_c, 3)

def gripped_pull_sweep(B, depth=50.0, dz=1.0):
    g, R = B["g"], B["recv"] + B["lid"] + B["strips"] + B["male"]
    A = B["cart"] + xt60_female(g) + slider(g, TRAVEL, 1) + slider(g, TRAVEL, -1)
    worst, d = 0.0, 0.0
    while d >= -depth - 1e-6:
        worst = max(worst, vol(A.translate([0, d, 0]) ^ R)); d -= dz
    return round(worst, 3)

def squeeze_sweep(B, n=9):
    """seated: sliders 0..4 in; vs cartridge, receiver, each other."""
    g = B["g"]; R = B["recv"] + B["lid"] + B["strips"] + B["male"]; C = B["cart"]
    w = dict(vs_cart=0.0, vs_recv=0.0, vs_each=0.0)
    for k in range(n):
        rt = TRAVEL * k / (n - 1) - (0.02 if k == n - 1 else 0.0)
        a, b = slider(g, rt, 1), slider(g, rt, -1)
        w["vs_cart"] = max(w["vs_cart"], vol(a ^ C), vol(b ^ C)); w["vs_recv"] = max(w["vs_recv"], vol(a ^ R), vol(b ^ R))
        w["vs_each"] = max(w["vs_each"], vol(a ^ b))
    return {k: round(v, 3) for k, v in w.items()}

def static_fit(B):
    g = B["g"]
    parts = dict(cart=B["cart"], plug=xt60_female(g), s1=slider(g, 0, 1), s2=slider(g, 0, -1), pack=pack_body(g),
                 recv=B["recv"], lid=B["lid"], strips=B["strips"], male=B["male"])
    names = list(parts); res = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            v = vol(parts[names[i]] ^ parts[names[j]])
            if v > 0.01: res[f"{names[i]}-{names[j]}"] = round(v, 3)
    return res

def bearing_drop(B):
    """sliders at rest: how far the cartridge drops before each bolt bears on its pocket floor (first contact, 0.05 steps)."""
    g = B["g"]; R = B["recv"]
    out = []
    for s in (1, -1):
        d = 0.0
        while d < 2.0 and vol(slider(g, 0, s).translate([0, -d, 0]) ^ R) < 0.01: d += 0.05
        out.append(round(d, 2))
    return out

def finger_model(g, open_=True, off=0.0, reach=6.0):
    """two v4 blades (3 thick, 13 wide in z) at their open (11..14) or squeezed (6..9) inner faces, head offset off in x."""
    xi = 11.0 if open_ else 6.0
    return U(*[box(*sorted((s * xi + off, s * (xi + 3.0) + off)), -20.0, reach, g["Z_T"] - 6.5, g["Z_T"] + 6.5) for s in (-1, 1)])

def grip_checks(B):
    """blades enter the recess open at head offset -1 / 0 / +1 (x), then squeeze; margins C."""
    g = B["g"]; C = B["cart"]
    res = {}
    for off in (-1.0, 0.0, 1.0):
        Fo = finger_model(g, True, off)
        res[f"open_vs_cart_off{off:+.0f}"] = ov(Fo, C)
        res[f"open_vs_tabs_rest_off{off:+.0f}"] = round(vol(Fo ^ (slider(g, 0, 1) + slider(g, 0, -1))), 3)
        # squeezed: the tabs meet at the fingers' centre, so the side with the shorter push retracts TRAVEL - |off|
        res[f"min_retract_off{off:+.0f}"] = TRAVEL - abs(off)
    res["retract_needed"] = OVERLAP + 0.3
    res["retract_margin_at_1mm"] = round(TRAVEL - 1.0 - (OVERLAP + 0.3), 2)
    Fs = finger_model(g, False, 0.0)
    res["squeezed_vs_cart"] = ov(Fs, C)
    res["squeezed_vs_sliders_squeezed"] = round(vol(Fs ^ (slider(g, TRAVEL, 1) + slider(g, TRAVEL, -1))), 3)
    return res


# ======================================================================== hand calc (E loads, dimensions L / I)
G0 = 9.81
ALLOW = dict(functional=10.8, misuse=17.3)    # MPa at index 1.0 (hook latch convention: SF 4 / 2.5 on PETG, E)

def section_ledge(r):
    """ledge + back wall section (x-y plane) under / behind the pocket: (I about its centroid, c_max), numeric strips."""
    rf = r["RF"]; xs = np.linspace(rf, rf + r["PK_D"] + PK_WALL, 241); dx = xs[1] - xs[0]
    A = Sy = 0.0; cells = []
    for x in xs[:-1] + dx / 2:
        lead = Y_RB + max(0.0, (rf + CAP - x)) / math.tan(LEAD) if x < rf + CAP else Y_RB
        if x < rf + r["PK_D"]:
            top = r["PK_Y0"] - (x - rf) * math.tan(UNDER)
        else:
            top = r["PK_Y1"] + PK_WALL
        y0 = min(lead, top); h = max(0.0, top - y0)
        cells.append((y0, top)); A += h * dx; Sy += h * dx * (y0 + top) / 2
    yc = Sy / A; I = 0.0; cmax = 0.0
    for (y0, top) in cells:
        h = top - y0
        I += dx * h ** 3 / 12 + dx * h * ((y0 + top) / 2 - yc) ** 2
        cmax = max(cmax, abs(top - yc), abs(y0 - yc))
    return I, cmax, A

def hand_calc(B):
    g, r = B["g"], B["r"]
    m = g["SWAP_M"] / 1000.0                      # kg, swap assembly
    W1 = m * G0
    out = {"mass_swap_g": round(g["SWAP_M"], 1)}
    lever = (g["RF"] - g["X_OUT"]) + OVERLAP / 2                  # wall hole to the bearing centroid
    Zb = 2 * BOLT_HZ * BOLT_T ** 2 / 6                            # bolt section modulus (18 x 5)
    Il, cl, Al = section_ledge(r)
    span = 2 * r["PK_HZ"]
    cases = {}
    def add(name, Fy_out, cls, Fx=0.0, Fz=0.0):
        """Fy_out: pull-out force (N) on the bolts; Fx lateral."""
        Fb = max(Fy_out, 0.0) / 2.0
        s_bolt = Fb * lever / Zb
        s_ledge = (Fb * span / 12.0) * cl / Il                    # fixed-fixed beam over the pocket span, L section
        idx = max(s_bolt, s_ledge) / ALLOW[cls]
        cases[name] = dict(pull_N=round(Fy_out, 1), per_bolt_N=round(Fb, 1), bolt_MPa=round(s_bolt, 2),
                           ledge_MPa=round(s_ledge, 2), cls=cls, index=round(idx, 3), Fx_N=round(Fx, 1), Fz_N=round(Fz, 1))
    add("hover 1 g", W1, "functional")
    add("flight 4 g + 2 g lateral", 4 * W1, "functional", Fx=2 * W1)
    add("catch 6 g", 6 * W1, "functional")
    add("crash 35 g -y (drop on the belly)", 35 * W1, "misuse")
    add("crash 35 g corner (-y share 35/sqrt3)", 35 * W1 / math.sqrt(3), "misuse", Fx=35 * W1 / math.sqrt(3), Fz=35 * W1 / math.sqrt(3))
    add("crash 35 g +y (top hit, into the deck)", 0.0, "misuse")
    add("crash 35 g side x (into the plates)", 0.0, "misuse", Fx=35 * W1)
    out["cases"] = cases
    # overturning couple in a side hit: top rails bear on the plate at y ~ rail mid, CG at pack centre -> vertical on bolts
    y_rail = g["Y_TOP"] - 5.0; y_cg = float(g["SWAP_CG"][1])
    Fx = 35 * W1
    out["side_hit_couple_per_bolt_N"] = round(Fx * abs(y_rail - y_cg) / (2 * g["RF"]), 1)
    # grip pull while the XT60 unmates: 50 N + 1 g of the swap mass, through the two tabs (bolts in, squeezed)
    Fg = 50.0 + W1; Ft = Fg / 2
    lever_t = REC_X - (TAB_OUT - TRAVEL - GP_D / 2)               # frame edge to the pocket centre, squeezed
    s_tab = Ft * lever_t / Zb
    out["grip_pull"] = dict(total_N=round(Fg, 1), per_tab_N=round(Ft, 1), bar_MPa=round(s_tab, 2),
                            index=round(s_tab / ALLOW["functional"], 3), lever_mm=round(lever_t, 2))
    # floor plate under the pack, 35 g, simply supported strip over the cavity width
    p = (g["m"] - g["m_plug"]) / 1000 * 35 * G0 / (g["W"] * g["L"])
    Mf = p * (2 * g["X_IN"]) ** 2 / 8; s_f = 6 * Mf / g["FLOOR_T"] ** 2
    out["floor_35g"] = dict(MPa=round(s_f, 2), index=round(s_f / ALLOW["misuse"], 3))
    # long walls as beams from the bolt holes, 35 g (half per wall, cantilever each side of the tab line)
    w_wall = 35 * W1 / 2; Lh = max(g["Z_T"], g["L_C"] - g["Z_T"])
    Mw = (w_wall / 2) * Lh / 2; Iw = WALL * g["Y_TOP"] ** 3 / 12 * 0.6                          # 0.6: windows (E)
    s_w = Mw * (g["Y_TOP"] / 2) / Iw
    out["wall_35g"] = dict(MPa=round(s_w, 2), index=round(s_w / ALLOW["misuse"], 3))
    # slider inertia in a 35 g side hit vs the spring and the friction lock of a loaded bolt (corner case)
    m_sl = tm(slider(g, 0, 1)).volume / 1000 * RHO["PETG"] / 1000
    out["slider_inertia_35g_N"] = round(m_sl * 35 * G0, 2)
    out["bolt_friction_lock_corner_N"] = round(0.3 * cases["crash 35 g corner (-y share 35/sqrt3)"]["per_bolt_N"], 1)
    # spring / release
    k_spr, free = 0.17, 20.0
    gap_rest = 2 * TAB_IN + 2 * SPR_DEPTH
    out["spring"] = dict(rate_N_mm=k_spr, free_mm=free, rest_N=round(k_spr * (free - gap_rest), 2),
                         squeezed_N=round(k_spr * (free - (gap_rest - 2 * TRAVEL)), 2), length_squeezed_mm=gap_rest - 2 * TRAVEL)
    fr = 0.3 * W1 / 2 + W1 / 2 * math.tan(UNDER)
    out["release_force_per_finger_N"] = round(out["spring"]["squeezed_N"] + fr, 2)
    # side hit: rails bear on the side plate near the deck; plate root bending per mm of length, and the overturning
    # couple carried by the bolts (vertical) and the deck strips
    q = 35 * W1 / g["L_C"]; Mp = q * (r["Y_DK0"] - (g["Y_TOP"] - 5.0)); s_p = 6 * Mp / r["PLATE_T"] ** 2
    out["plate_side_hit_35g"] = dict(MPa=round(s_p, 2), index=round(s_p / ALLOW["misuse"], 3))
    Fc = out["side_hit_couple_per_bolt_N"]
    s_c = Fc * lever / Zb
    out["bolt_side_hit_couple"] = dict(MPa=round(s_c, 2), index=round(s_c / ALLOW["misuse"], 3))
    out["worst_index"] = max([c["index"] for c in cases.values()] + [out["grip_pull"]["index"], out["floor_35g"]["index"], out["wall_35g"]["index"],
                                                                   out["plate_side_hit_35g"]["index"], out["bolt_side_hit_couple"]["index"]])
    return out


# ======================================================================== joint reactions at the 4 mount bosses (rigid deck, E)
def reactions(B, hc):
    """per case: resultant (N, N mm) at the mount plane centre and per-boss forces (rigid plate, equal shear, axial from
    the moment), cartridge coords. Loads act at the swap CG (inertia) or at the cartridge bottom / XT60 (swap)."""
    g, r = B["g"], B["r"]
    W1 = g["SWAP_M"] / 1000 * G0
    cg = np.asarray(g["SWAP_CG"], float)
    pts = np.array([[bx, r["Y_M"], bz] for (bx, bz) in r["MOUNT"]])
    ctr = pts.mean(0)
    s3 = 1 / math.sqrt(3)
    cases = {
        "hover 1 g": (np.array([0, -1, 0]) * W1, cg),
        "flight 4 g up + 2 g lateral": (np.array([2, -4, 0]) * W1, cg),
        "catch 6 g": (np.array([0, -6, 0]) * W1, cg),
        "crash 35 g -y": (np.array([0, -35, 0]) * W1, cg),
        "crash 35 g +y": (np.array([0, 35, 0]) * W1, cg),
        "crash 35 g +x": (np.array([35, 0, 0]) * W1, cg),
        "crash 35 g +z": (np.array([0, 0, 35]) * W1, cg),
        "crash 35 g corner (+x -y +z)": (np.array([35 * s3, -35 * s3, 35 * s3]) * W1, cg),
        "swap insert 60 N": (np.array([0, 60.0, 0]), np.array([0, 0, g["Z_T"]])),
        "swap pull 50 N (XT60)": (np.array([0, -50.0, 0]), np.array([0, r["Y_XM"], g["Z_X"]])),
    }
    out = {}
    for nm, (F, at) in cases.items():
        M = np.cross(at - ctr, F)
        # boss axial (y) from F_y and the moments about x and z; shear (x, z) equal share + torsion about y
        d = pts - ctr
        Iz = float((d[:, 0] ** 2).sum()); Ix = float((d[:, 2] ** 2).sum())
        Fy = F[1] / 4 + (-M[2] * d[:, 0] / Iz if Iz else 0) + (M[0] * d[:, 2] / Ix if Ix else 0)
        Jt = float((d[:, 0] ** 2 + d[:, 2] ** 2).sum())
        Fx = F[0] / 4 + (-M[1] * d[:, 2] / Jt); Fz = F[2] / 4 + (M[1] * d[:, 0] / Jt)
        out[nm] = dict(F_N=np.round(F, 1).tolist(), M_Nmm=np.round(M, 0).tolist(),
                       boss_axial_N=np.round(Fy, 1).tolist(), boss_shear_N=np.round(np.hypot(Fx, Fz), 1).tolist(),
                       max_tension_N=round(float(-Fy.min()) if Fy.min() < 0 else 0.0, 1))
    return out, pts.tolist(), ctr.tolist()


# ======================================================================== collar inset search (Scout)
SCOUT_INSET = 11.0                # replaced by search_inset() at run time

def search_inset(log=print):
    """smallest clip inset (0.5 steps) that gives every receiver + cartridge point >= 7.0 inside the collar planes."""
    global SCOUT_INSET
    g = place_tab_line("scout"); T = T_dc(g)
    best = None
    for inset in np.arange(8.0, 20.01, 0.5):
        clip = collar_clip(g, inset)
        r = rgeom(g)
        P = np.vstack([tm(cartridge(g, clip), T).vertices, tm(receiver(r, clip), T).vertices,
                       tm(slider(g, 0, 1), T).vertices, tm(slider(g, 0, -1), T).vertices, tm(lid(r), T).vertices])
        mg = collar_margin(P)
        log(f"  inset {inset:.1f}: collar margin {mg:.2f}")
        if mg >= COLLAR_NEED:
            best = (float(inset), mg); break
    SCOUT_INSET = best[0]
    _CACHE.clear()
    return best


# ======================================================================== print files, system manifest
def _rot(axis, deg):
    a = math.radians(deg); c, s_ = math.cos(a), math.sin(a)
    R = {"x": [[1, 0, 0], [0, c, -s_], [0, s_, c]], "y": [[c, 0, s_], [0, 1, 0], [-s_, 0, c]], "z": [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]}[axis]
    T = np.eye(4); T[:3, :3] = R; return T

PIECES = {   # name: (getter, material, print pose (c -> print), count, version, bed face)
    "cartridge": (lambda B: B["cart"], "PETG", np.eye(4), 1, "v5", "closed end (z 0) down"),
    "slider": (lambda B: slider(B["g"], 0, 1), "PETG", _rot("y", -90), 2, "v5", "inner tab face down"),
    "receiver": (lambda B: B["recv"], "PETG", np.eye(4), 1, "v1", "closed-end face down"),
    "receiver_lid": (lambda B: B["lid"], "PETG", _rot("x", -90), 1, "v1", "top face down"),
    "receiver_strip": (lambda B: U(*[box(*sorted((B["r"]["X_IN"] - 1.55, B["r"]["RF"] - 0.05)), B["r"]["Y_TOP"], B["r"]["Y_TOP"] + STRIP_T,
                                         B["r"]["Z_P0"] - 0.8, B["r"]["Z_P1"] - 1.2)]), "TPU", _rot("x", 90), 2, "v1", "flat"),
}

def save_prints(B, pack, log=print):
    """export every printed piece on its bed face to runs/battery_latch/v1/stl/, solid_check, print_orient numbers."""
    from solid_check import solid_check
    import print_orient as PO
    d = os.path.join(OUT, "stl"); os.makedirs(d, exist_ok=True)
    out = {}
    for nm, (get, mat, T, n, ver, face) in PIECES.items():
        t = tm(get(B), T)
        t.apply_translation([-t.bounds[:, 0].mean(), -t.bounds[:, 1].mean(), -t.bounds[0, 2]])
        fn = f"box_battery_{nm}_{pack}_{ver}.stl"
        path = os.path.join(d, fn); t.export(path)
        ok, info = solid_check(t)
        bed, over, run = PO.analyse(t, nm)
        out[nm] = dict(file=fn, material=mat, count=n, version=ver, bed_face=face, mass_g=round(t.volume / 1000 * RHO[mat], 2),
                       extents=np.round(t.extents, 1).tolist(), solid_check="PASS" if ok else "FAIL",
                       solid_info={k: (v if isinstance(v, (int, float, str, bool)) else str(v)) for k, v in (info or {}).items()},
                       bed_mm2=round(float(bed), 0), overhang_mm2=round(float(over), 0), island_run_mm=round(float(run), 1))
        log(f"  {fn}: solid_check {'PASS' if ok else 'FAIL'}, {out[nm]['mass_g']} g, bed {bed:.0f} mm2, overhang {over:.0f} mm2")
    return out

def write_manifest(B, pack):
    """nox/system_check.py manifest (drone frame stand-in). Scout: collar-band cage stand-in, 500 g drone (dimensions L);
    Overseer: 2200 g drone, a box cage stand-in 3.0 + 3.0 outside (its seat height and collar are the Overseer's pass)."""
    g, r = B["g"], B["r"]
    T = T_dc(g)
    d = os.path.join(OUT, "manifest_" + pack); os.makedirs(d, exist_ok=True)
    parts = []
    def add(nm, m, role, mat=None, mass=None, tag="M", swap=False, cg=None, mesh_T=T):
        fn = os.path.join(d, nm + ".stl"); tm(m, mesh_T).export(fn)
        p = dict(name=nm, mesh=os.path.relpath(fn, d), role=role, tag=tag)
        if mat: p["material"] = mat
        if mass is not None: p["mass_g"] = round(float(mass), 2)
        if swap: p["swap"] = True
        if cg is not None: p["cg"] = [round(float(v), 2) for v in cg]
        parts.append(p)
    add("pack", pack_body(g), "protected", mass=g["m"] - g["m_plug"], tag=g["m_tag"], swap=True)
    add("xt60_plug", xt60_female(g), "bought", mass=g["m_plug"], tag="E", swap=True)
    add("cartridge", B["cart"], "protected", mat="PETG", swap=True)
    add("slider_px", slider(g, 0, 1), "protected", mat="PETG", swap=True)
    add("slider_nx", slider(g, 0, -1), "protected", mat="PETG", swap=True)
    add("receiver", B["recv"], "protected", mat="PETG")
    add("receiver_lid", B["lid"], "protected", mat="PETG")
    add("receiver_strips", B["strips"], "structure", mat="TPU")
    add("xt60e1m", B["male"], "bought", mass=6.0, tag="E")
    total = 500.0 if pack == "scout" else 2200.0            # dimensions L mass budgets
    # mass of what is drawn here
    drawn = (g["m"]) + sum(vol(m) / 1000 * RHO[mat] for m, mat in ((B["cart"], "PETG"), (slider(g, 0, 1), "PETG"), (slider(g, 0, -1), "PETG"),
                                                                 (B["recv"], "PETG"), (B["lid"], "PETG"), (B["strips"], "TPU"))) + 6.0
    yb = r["Y_M"] + SCOUT_Z0
    win = tm(box(-9.25, 9.25, r["Y_M"] - 1, r["Y_M"] + 40, g["Z_X"] - 5.55, g["Z_X"] + 5.55), T)    # partner window over the
    wb = win.bounds                                                                                     # XT60E1-M rear + leads
    stand = box(-40, 40, -40, 40, yb, yb + 30) - box(wb[0][0], wb[1][0], wb[0][1], wb[1][1], wb[0][2], wb[1][2])
    add("drone_rest_standin", stand, "structure", mass=total - drawn, tag="E", cg=[0, 0, yb + 15], mesh_T=np.eye(4))
    if pack == "scout":                                      # collar band cage stand-in: outer faces 1.0 inside the planes
        zb, zt_ = -47.5, 20.0
        pts = [[sx * (COLLAR_K + z - 1.0), sy * (COLLAR_K + z - 1.0), z] for z in (zb, zt_) for sx in (-1, 1) for sy in (-1, 1)]
        cage = hullp(pts) - hullp([[sx * (COLLAR_K + z - 4.0), sy * (COLLAR_K + z - 4.0), z] for z in (zb + 3.0, zt_ + 1) for sx in (-1, 1) for sy in (-1, 1)])
    else:
        Ad = tm(U(B["cart"], B["recv"], B["lid"]), T); lo, hi = Ad.bounds
        cage = box(lo[0] - 6, hi[0] + 6, lo[1] - 6, hi[1] + 6, lo[2] - 3.0, hi[2] + 6) - box(lo[0] - 3, hi[0] + 3, lo[1] - 3, hi[1] + 3, lo[2], hi[2] + 3)
    add("cage_standin", cage, "cage", mass=0.0, tag="E", mesh_T=np.eye(4))
    man = dict(name=f"battery_latch_{pack}", parts=parts,
               openings=[dict(axis=[0, 0, -1], half_angle_deg=20.0, min_mm=0.0)],
               contact_pairs=[["cartridge", "receiver_strips", 5.0], ["pack", "cartridge", 1.0], ["xt60_plug", "cartridge", 5.0],
                              ["xt60e1m", "receiver", 5.0], ["xt60e1m", "receiver_lid", 5.0], ["receiver_strips", "receiver", 5.0],
                              ["drone_rest_standin", "receiver_lid", 1.0], ["drone_rest_standin", "receiver", 1.0]],
               cg_ref=[0.0, 0.0, 0.0], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0, fit_tol_mm3=0.5)
    fn = os.path.join(d, "manifest.json")
    with open(fn, "w") as f: json.dump(man, f, indent=1)
    return os.path.relpath(fn, ROOT)


# ======================================================================== Overseer pack bay (nox/overseer/overseer.assembly.json)
OV_ASM = os.path.join(ROOT, "nox", "overseer", "overseer.assembly.json")
OV_Z0 = -80.0                      # cartridge flat bottom at the layout's envelope bottom (drone z, layout v2)

def overseer_fit(x_tab=-30.0, lead_aft=True, log=print):
    """the Overseer set posed in the Overseer layout (its cartridge and pack boxes replaced): flat bottom z -80, tab line
    (= swap CG) at drone x x_tab, lead end aft; system_check fit / balance on the whole drone. Returns the summary."""
    import system_check as sc
    B = build("overseer"); g = B["g"]
    sgn = 1.0 if lead_aft else -1.0
    T = np.array([[0, 0, -sgn, x_tab + sgn * g["Z_T"]], [-sgn, 0, 0, 0], [0, 1, 0, OV_Z0], [0, 0, 0, 1.0]])
    man = json.load(open(OV_ASM))
    base = os.path.dirname(OV_ASM)
    keep = []
    for prt in man["parts"]:
        if prt["name"] in ("cartridge", "pack"): continue
        q = dict(prt)
        if "mesh" in q: q["mesh"] = os.path.join(ROOT, q["mesh"]) if not os.path.isabs(q["mesh"]) else q["mesh"]
        keep.append(q)
    def P(nm, m, role, mat=None, mass=None, swap=False, tag="M"):
        d = dict(name=nm, trimesh=tm(m, T), role=role, tag=tag, geom_tag="C")
        if mat: d["material"] = mat
        if mass is not None: d["mass_g"] = mass
        if swap: d["swap"] = True
        return d
    mine = [P("pack", pack_body(g), "bought", mass=g["m"] - g["m_plug"], swap=True, tag="E"),
            P("xt60_plug", xt60_female(g), "bought", mass=g["m_plug"], swap=True, tag="E"),
            P("cartridge", B["cart"], "protected", "PETG", swap=True), P("slider_px", slider(g, 0, 1), "protected", "PETG", swap=True),
            P("slider_nx", slider(g, 0, -1), "protected", "PETG", swap=True), P("receiver", B["recv"], "protected", "PETG"),
            P("receiver_lid", B["lid"], "protected", "PETG"), P("receiver_strips", B["strips"], "structure", "TPU"),
            P("xt60e1m", B["male"], "bought", mass=6.0, tag="E")]
    for d in mine:
        if d["role"] == "bought" and d["name"] == "pack": d["protect"] = True
    cp = [c for c in man.get("contact_pairs", []) if "cartridge" not in c and "pack" not in c]
    cp += [["cartridge", "receiver_strips", 5.0], ["pack", "cartridge", 1.0], ["xt60_plug", "cartridge", 5.0], ["xt60e1m", "receiver", 5.0],
           ["xt60e1m", "receiver_lid", 5.0], ["receiver_strips", "receiver", 5.0], ["receiver", "bottom plate", 1.0], ["receiver_lid", "bottom plate", 1.0]]
    m2 = dict(man); m2["parts"] = keep + mine; m2["contact_pairs"] = cp
    res = sc.run(m2, n_dirs=600)
    fit_pairs = [pp for pp in res["fit"]["pairs"] if any(k in (pp.get("a"), pp.get("b")) for k in [d["name"] for d in mine])] \
        if isinstance(res.get("fit", {}).get("pairs"), list) else res.get("fit")
    return res, fit_pairs, T


# ======================================================================== main
def solid(mesh):
    from solid_check import solid_check
    ok, info = solid_check(mesh)
    return bool(ok), {k: (v if isinstance(v, (int, float, str, bool, list)) else str(v)) for k, v in (info or {}).items()}

PRINT_POSE = {   # c -> print frame: largest flat face down
    "cartridge": np.diag([1.0, 1, 1, 1]),                                  # replaced: closed end (z 0) down = identity
}

def export(m, path, T=None):
    t = tm(m, T); t.export(path); return t

def main():
    os.makedirs(OUT, exist_ok=True)
    log_lines = []
    def log(s): print(s); log_lines.append(s)
    res = {}
    log("collar inset search (Scout, >= 7.0 inside the planes below z 20, +-5 yaw, +-2 tilt):")
    inset, mg = search_inset(log)
    res["scout_collar"] = dict(inset=inset, margin=round(mg, 2), need=COLLAR_NEED)
    for pack in ("scout", "overseer"):
        B = build(pack); g, r = B["g"], B["r"]
        log(f"== {pack}: z_T {g['Z_T']} (swap CG), swap {g['SWAP_M']:.1f} g, cartridge {g['L_C']:.1f} x {2*g['X_RAIL']:.1f} x {g['Y_TOP']:.1f}")
        R = {}
        R["geom"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in g.items() if isinstance(v, (int, float, str))}
        R["rgeom"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k not in g and isinstance(v, (int, float))}
        R["mount"] = [[round(a, 2) for a in p] for p in r["MOUNT"]]
        R["static_fit_mm3"] = static_fit(B)
        R["squeeze"] = squeeze_sweep(B)
        R["gripped_pull_mm3"] = gripped_pull_sweep(B)
        rows, wc = insertion_sweep(B)
        R["insert_free"] = dict(cart_vs_recv_mm3=wc, stuck=[d for d, rr in rows if None in rr],
                               retract_profile=[(d, rr) for d, rr in rows if any(x for x in rr)])
        rows1, wc1 = insertion_sweep(B, side_off=0.3, dz=2.0, depth=20.0)
        R["insert_free_off0.3"] = dict(cart_vs_recv_mm3=wc1, stuck=[d for d, rr in rows1 if None in rr])
        R["bearing_drop"] = bearing_drop(B)
        R["grip"] = grip_checks(B)
        R["plug_vs_male_seat_mm3"] = ov(xt60_female(g), B["male"])
        hc = hand_calc(B); R["hand_calc"] = hc
        rx, pts, ctr = reactions(B, hc); R["reactions"] = rx; R["mount_pts"] = pts
        # masses and bounding volume
        ms = {}
        for nm, m, mat in (("cartridge", B["cart"], "PETG"), ("slider", slider(g, 0, 1), "PETG"), ("receiver", B["recv"], "PETG"),
                           ("lid", B["lid"], "PETG"), ("strips", B["strips"], "TPU")):
            ms[nm] = round(vol(m) / 1000 * RHO[mat], 2)
        R["mass_g"] = ms
        A = U(B["cart"], B["recv"], B["lid"], slider(g, 0, 1), slider(g, 0, -1))
        bb = tm(A).bounds; ext = bb[1] - bb[0]
        R["bbox"] = dict(extents=np.round(ext, 1).tolist(), volume_L=round(float(np.prod(ext)) / 1e6, 4), largest=round(float(ext.max()), 1))
        if pack == "scout":
            T = T_dc(g)
            R["collar_margin"] = round(collar_margin(np.vstack([tm(A, T).vertices])), 2)
            keel = np.array([[28, -28.5, -47.5], [-28, -28.5, -47.5], [28, 28.5, -47.5]])
            Ad = tm(A, T)
            R["keel_node_clearance"] = [round(float(np.linalg.norm(Ad.vertices - k, axis=1).min()), 2) for k in keel]
            q = trimesh.proximity.ProximityQuery(Ad)
            R["keel_node_surface_dist"] = [round(float(abs(q.signed_distance([k])[0])), 2) for k in keel]
        res[pack] = R
        log(json.dumps({k: R[k] for k in ("static_fit_mm3", "squeeze", "gripped_pull_mm3", "bearing_drop", "mass_g", "bbox")}))
        log(f"  insert free: cart {R['insert_free']['cart_vs_recv_mm3']} mm3, stuck {R['insert_free']['stuck']}; off 0.3 stuck {R['insert_free_off0.3']['stuck']}")
        log(f"  hand calc worst index {hc['worst_index']}; grip pull {hc['grip_pull']}")
        R["prints"] = save_prints(B, pack, log)
        R["system_manifest"] = write_manifest(B, pack)
    # Overseer: pose in the seeded Overseer assembly under its system DEFAULT B (pack / tab line 30 behind the thrust axis)
    try:
        man = json.load(open(OV_ASM)); xt = float(man["cg_ref"][0]) - 30.0
        ores, fp, _ = overseer_fit(x_tab=xt)
        res["overseer"]["overseer_assembly_fit"] = dict(x_tab=xt, summary=ores["summary"],
                                                        pairs=[(p_["a"], p_["b"], p_["overlap_mm3"]) for p_ in fp if not p_["contact"]],
                                                        balance=ores.get("balance"))
        log("overseer in overseer.assembly.json (B): " + ores["summary"])
    except Exception as e:                                                   # the Overseer file is the Overseer chat's
        res["overseer"]["overseer_assembly_fit"] = dict(error=str(e))
    with open(os.path.join(OUT, "result.json"), "w") as f: json.dump(res, f, indent=1, default=float)
    with open(os.path.join(OUT, "log.txt"), "w") as f: f.write("\n".join(log_lines) + "\n")
    return res

if __name__ == "__main__":
    main()
