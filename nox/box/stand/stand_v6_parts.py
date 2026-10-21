"""Box stand v6 printed parts (19 Oct 2026; Spark 19 Oct 17:25 fully printed, 19:09 build all but the servo heads).
Drawn in manifold3d, W coords (box-arm.md: X across, Y along the axle away from the stub face, Z up, axle X 0 Z 0, floor Z -201).
Parts (box_<name>_v6.stl, saved in print pose; *_W.stl in W for checks):
  mast        hollow 40 x 60 sleeve Z -128..44 on KB's spigot; stub v3 on its front face Y 0 (4 x M5 x 12 button + nuts in side slots)
  kb          back keel U-channel Y -110..40 + spigot (Z to -52) + gussets + end plate Y 40..46 (base joint)
  cradle_keel cradle v5e (unchanged interface to the -90 bay module) merged with the front keel Y 62..140 + start plate Y 62..68;
              nut blocks inside the back wall for the toe (replaces cradle v5e + KF)
  bar_half    cross bar half X -20..200 (print 2, identical; the second turned 180 deg about Z), laps at Y 46..54 / 54..62
  toe         L on the cradle's back face, floor foot to Y 260
  frame_mid   ring frame, 30 deg, print 5 (positions -30, 0, -120, -150, -180): the bay module's joints on both sides + truss
  frame_endR  ring end at 30 deg: +X joint (lugs + flat pad, no tongue) + 2 funnel seat pads on the 45 deg end
  frame_endL  ring end at -210 deg: -X joint (nut lugs + groove) + 2 funnel seat pads on the 135 deg end (mirror of endR's pads)
Ring frames are drawn in the module's top-position frame (bay radial = +Z) and placed with rotate([0, 90 - p, 0]).
Tags: C CAD unless noted; E estimate.  Usage: python3 stand_v6_parts.py  -> STLs + stand_v6_prints.json"""
import json, math, sys
import numpy as np, trimesh
import manifold3d as mf
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
import arm_layout as L
from arm_cad import box, cylY, cylX, cylZ, hull, _plane_box, _plane_cyl_t, _dovetail

D = 1.27e-3                                   # g / mm3 PETG solid (E)
FLOOR, KTOP = -201.0, -181.0                  # floor, keel top (C)
W2 = 2.0                                      # min printed wall (README)
KW = 3.0                                      # keel walls / floor
H = math.radians(L.HALF)                      # 15 deg

def tm(m):
    g = m.to_mesh(); return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)
def ccw(poly):
    return [tuple(map(float, p)) for p in orient(Polygon(poly), 1.0).exterior.coords[:-1]]
def prism_xz(poly, y0, y1):
    m = mf.CrossSection([ccw(poly)]).extrude(y1 - y0)
    return m.transform(np.array([[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0]], float))
def prism_yz(poly, x0, x1):
    m = mf.CrossSection([ccw(poly)]).extrude(x1 - x0)
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]], float))
def hexprism_y(x, z, y0, y1, af=8.2):
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(a), z + r * math.sin(a)) for a in np.radians(np.arange(0, 360, 60))]   # flats top / bottom
    return prism_xz(pts, y0, y1)

# ======================================================================= MAST
MAST_X, MAST_Y0, MAST_Y1 = 20.0, -60.0, 0.0       # X +-20, Y -60..0 (C)
MAST_Z = (-128.0, 44.0)                           # 172 tall (fits the 180 bed standing)
HEAD_Z = -44.0                                    # front wall 4.0 from here up
STUB_HOLES = [(10.0, 30.0), (-10.0, 30.0), (-10.0, -30.0), (10.0, -30.0)]
XBOLTS = [(-20.0, -80.0), (-40.0, -112.0)]        # sleeve-to-spigot M5 along X at (Y, Z)
SPIG = dict(x=17.8, y0=-57.8, y1=-2.2, wall=2.0, ztop=-52.0)   # 0.2 inside the sleeve on every side

def mast():
    z0, z1 = MAST_Z
    outer = box(-MAST_X, MAST_X, MAST_Y0, MAST_Y1, z0, z1)
    inner = box(-MAST_X + W2, MAST_X - W2, MAST_Y0 + W2, MAST_Y1 - W2, z0 - 1, z1 + 1)          # open top and bottom
    m = outer - inner
    m += box(-MAST_X + 1, MAST_X - 1, MAST_Y1 - 4.0, MAST_Y1 - 1, HEAD_Z, z1)                 # front wall 4.0 in the head
    m += prism_yz([(MAST_Y1 - 1, HEAD_Z), (MAST_Y1 - 4.0, HEAD_Z), (MAST_Y1 - 1, HEAD_Z - 3)], -MAST_X + 1, MAST_X - 1)   # 45 deg step
    for zc in (30.0, -30.0):                                                                  # nut bars, 45 deg undersides
        poly = [(-4.0, zc + 7), (-12.0, zc + 7), (-12.0, zc - 4), (-4.0, zc - 12)]
        m += prism_yz(poly, -MAST_X + 1, MAST_X - 1)
    for x, z in STUB_HOLES:
        m -= cylY(x, z, -13.0, 1.0, 2.75, 48)                                                # M5 clearance
        s = 1 if x > 0 else -1
        m -= box(min(s * 5.2, s * (MAST_X + 1)), max(s * 5.2, s * (MAST_X + 1)), -8.6, -4.2, z - 4.15, z + 4.15)   # nut slot from the side face
    for y, z in XBOLTS:
        m -= cylX(y, z, -MAST_X - 1, MAST_X + 1, 2.75, 48)
    return m

# ======================================================================= KB: back keel + spigot
KB_Y = (-110.0, 40.0); PLATE_T = 6.0
JOINT_BOLTS = [(12.5, -193.7), (-12.5, -193.7), (6.0, -186.0), (-6.0, -186.0)]   # base joint, along Y (X, Z)
LAP_Y = (46.0, 54.0, 62.0)                                                        # bar half A lap 46..54, B 54..62

def ychannel(y0, y1):
    return box(-20, 20, y0, y1, FLOOR, FLOOR + KW) + box(-20, -20 + KW, y0, y1, FLOOR, KTOP) + box(20 - KW, 20, y0, y1, FLOOR, KTOP)

def joint_holes(y0, y1):
    m = None
    for x, z in JOINT_BOLTS:
        c = cylY(x, z, y0, y1, 2.75, 48); m = c if m is None else m + c
    return m

def kb():
    y0, y1 = KB_Y
    m = ychannel(y0, y1)
    m += box(-20, 20, y1, y1 + PLATE_T, FLOOR, KTOP)                                     # end plate Y 40..46
    s = SPIG
    m += box(-s["x"], s["x"], s["y0"], s["y1"], FLOOR, s["ztop"])
    m -= box(-s["x"] + s["wall"], s["x"] - s["wall"], s["y0"] + s["wall"], s["y1"] - s["wall"], FLOOR + KW, s["ztop"] + 1)
    m += prism_yz([(s["y1"], FLOOR + 1), (s["y1"], -132.0), (y1 + 0.5, FLOOR + 1)], -1.5, 1.5)          # front gusset into the end plate
    m += prism_yz([(s["y0"], FLOOR + 1), (s["y0"], -150.0), (-75.0, FLOOR + 1)], -1.5, 1.5)             # rear gusset (PSU zone Y -110..-75 stays clear)
    for y, z in XBOLTS:
        m -= cylX(y, z, -21, 21, 2.75, 48)
    m -= joint_holes(y1 - 1, y1 + PLATE_T + 1)
    for yz in (-100.0, -85.0):                                                            # zip-tie slots through both walls (PSU)
        m -= box(-21, 21, yz - 4, yz + 4, -192.0, -188.5)
    return m

# ======================================================================= cradle + front keel (cradle v6 = v5e interface + KF merged)
Z_TOP = -152.9; CR_Y = (140.0, 197.0); CR_XW, CR_XL = 20.4, 30.0
CR_BOLTS = [(s * 26.0, y) for s in (1, -1) for y in (154.0, 189.0)]               # up-bolts into the -90 module (v5e, placed on the mesh)
KF_Y0 = LAP_Y[2]                                                                  # start plate Y 62..68

def cradle_keel():
    XW, XL, Y0, Y1 = CR_XW, CR_XL, CR_Y[0], CR_Y[1]; WALL = 2.4
    body = box(-XW, XW, Y0, Y1, FLOOR, Z_TOP) - box(-XW + WALL, XW - WALL, Y0 - 1, Y1 - 3.0, FLOOR + WALL, Z_TOP - WALL)
    for s in (1, -1):
        body += box(min(s * (XW - 1), s * XL), max(s * (XW - 1), s * XL), Y0, Y1, Z_TOP - 6.0, Z_TOP)
    for x, y in CR_BOLTS:
        body -= cylZ(x, y, -170, -140, 2.65)
    for s in (1, -1):
        w = [(190, -190), (172, -190), (172 - 14 * math.tan(math.radians(60)), -176), (172, -162), (190, -162)]
        body -= prism_yz(w, (XW - WALL - 1) if s > 0 else -XW - 1, (XW + 1) if s > 0 else -(XW - WALL - 1))
    # toe nut blocks on the back wall's inside: nut drops in from above (Z), bolt from the toe side
    for s in (1, -1):
        body += box(min(s * 3.5, s * 16.5), max(s * 3.5, s * 16.5), 186.0, 194.5, FLOOR + 2, -183.0)
        body -= cylY(s * 10, -191.0, 184.0, Y1 + 1, 2.75)
        body -= box(s * 10 - 4.15, s * 10 + 4.15, 188.0, 192.6, -196.0, -182.0)
    # front keel, Y 62..140.2, and its start plate
    k = ychannel(KF_Y0 + PLATE_T, Y0 + 0.2) + box(-20, 20, KF_Y0, KF_Y0 + PLATE_T, FLOOR, KTOP)
    # 45 deg hip under the start plate (self-supporting in print: back face down, the plate is the top)
    y_p = KF_Y0 + PLATE_T; d = 17.0
    for s in (1, -1):
        k += hull([[s * 17.0, y_p, FLOOR + KW], [s * 17.0, y_p, KTOP], [s * 17.0, y_p + d, FLOOR + KW], [s * 17.0, y_p + d, KTOP],
                   [0.0, y_p, FLOOR + KW], [0.0, y_p, KTOP]])
    k += hull([[-17, y_p, FLOOR + KW], [17, y_p, FLOOR + KW], [-17, y_p, KTOP], [17, y_p, KTOP],
               [-17, y_p + d, FLOOR + KW], [17, y_p + d, FLOOR + KW]])
    k -= joint_holes(KF_Y0 - 1, y_p + d + 1)
    for x, z in JOINT_BOLTS:                                                              # nut / socket room behind the plate
        k -= cylY(x, z, y_p, y_p + d + 1, 4.9, 48)
    return body + k

# ======================================================================= cross bar half
BAR_X = (-20.0, 200.0)

def bar_half():
    y0, ym, y1 = LAP_Y
    lap = box(-20, 20.5, y0, ym, FLOOR, KTOP)
    P = Polygon([(20, FLOOR), (30, FLOOR), (30, FLOOR + 1), (185, FLOOR + 1), (185, FLOOR), (200, FLOOR),
                 (200, -193.0), (20, KTOP)])                                           # 1 mm relief between the root and the foot
    inner = P.buffer(-4.0, join_style=2)
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    zig = [(24, FLOOR + 1), (52, None), (80, FLOOR + 1), (108, None), (136, FLOOR + 1), (164, None), (186, FLOOR + 1)]
    pts = []
    for x, z in zig:
        ztop = KTOP + (x - 20) / 180 * (-193.0 - KTOP)
        pts.append((x, ztop if z is None else z))
    diag = LineString(pts).buffer(2.0, cap_style=2, join_style=2)
    win = inner.difference(diag)
    geoms = [g for g in getattr(win, "geoms", [win]) if g.area > 25]
    arm = prism_xz(list(P.exterior.coords)[:-1], y0, y1)
    for g in geoms:
        arm -= prism_xz(list(g.buffer(-0.01).exterior.coords)[:-1], y0 - 1, y1 + 1)
    m = lap + arm
    m -= joint_holes(y0 - 1, y1 + 1)
    return m

# ======================================================================= toe
def toe():
    m = box(-20, 20, 197.0, 201.0, FLOOR, KTOP)                                          # upstand on the cradle's back face
    m += box(-8, 8, 201.0, 260.0, FLOOR, FLOOR + 8)                                      # foot bar
    m += box(-15, 15, 250.0, 260.0, FLOOR, FLOOR + 8)                                    # foot pad
    m += prism_yz([(201.0, FLOOR + 7), (201.0, KTOP), (216.0, FLOOR + 7)], -4, 4)         # gusset
    for s in (1, -1):
        m -= cylY(s * 10, -191.0, 196.0, 202.0, 2.75)
    return m

# ======================================================================= ring frames (top-position frame)
RIB_Y = (153.0, 201.0)                 # rib webs beside the lugs (not over them: nut pockets and heads stay reachable)
RIB_T = 6.0
PADS_Y = (141.0, 213.0)                # funnel seat pads (collar inserts)
def pt(side, t, y, rho):
    x = t * math.cos(H) + rho * math.sin(H); z = -t * math.sin(H) + rho * math.cos(H)
    return [side * x, y, z]
def node(side, t, y, rho, a=2.5):
    return [[p[0] + dx, p[1] + dy, p[2] + dz] for p in [pt(side, t, y, rho)] for dx in (-a, a) for dy in (-a, a) for dz in (-a, a)]
def member(a, b):
    return hull(node(*a) + node(*b))
def sector(gp, gm, y0, y1):
    return _plane_box(+1, -600, -gp, y0, y1, -600, 600) ^ _plane_box(-1, -600, -gm, y0, y1, -600, 600)
def annulus(r0, r1, y0, y1):
    return cylY(0, 0, y0, y1, r1, 360) - cylY(0, 0, y0 - 1, y1 + 1, r0, 360)

def dt_prism(side, prof, y0, y1):
    """profile [(t, rho), ...] in the side's plane frame, extruded over Y y0..y1 (convex)."""
    return hull([pt(side, t, y, r) for t, r in prof for y in (y0, y1)])

def tongue_frame(ys0, ys1):
    """frame tongue on the +X plane: the module's dovetail with both flanks moved in 0.2 (h0 2.8 -> tip 5.8 at depth 3.3):
    arm_cad's groove (grow 0.2 on width AND depth) lies ON the nominal tongue's 45 deg flanks, so a nominal tongue has 0 flank
    clearance; this one has 0.14 normal clearance in a module groove and 0.35 in a frame groove."""
    g = L.JOINT_GAP; h0 = L.DT_W0 / 2 - 0.2; rc = L.DT_RHO_C
    neck = dt_prism(+1, [(-4.0, rc - h0), (-4.0, rc + h0), (g + 0.01, rc - h0), (g + 0.01, rc + h0)], ys0, ys1)
    flare = dt_prism(+1, [(g, rc - h0), (g, rc + h0), (g + L.DT_D, rc - h0 - L.DT_D), (g + L.DT_D, rc + h0 + L.DT_D)], ys0, ys1)
    return neck + flare

def groove_frame():
    """frame groove on the -X plane, full length: the nominal profile grown 0.3 in width with 45 deg flanks kept
    (normal clearance 0.21 on the flanks, 0.3 at the neck and the tip); open through the bed-side 0.25."""
    g = L.JOINT_GAP; h = L.DT_W0 / 2 + 0.3; rc = L.DT_RHO_C; y0, y1 = L.MOD_Y[0] - 5, L.MOD_Y[1] + 5
    neck = dt_prism(-1, [(1.0, rc - h), (1.0, rc + h), (-g - 0.01, rc - h), (-g - 0.01, rc + h)], y0, y1)
    flare = dt_prism(-1, [(-g, rc - h), (-g, rc + h), (-g - L.DT_D - 0.3, rc - h - L.DT_D - 0.3), (-g - L.DT_D - 0.3, rc + h + L.DT_D + 0.3)], y0, y1)
    return neck + flare

def nut_pocket_mX(y):
    rc = (L.LUG_RHO[0] + L.LUG_RHO[1]) / 2; tm_ = L.LUG_T[1]; r = L.M5_NUT_AF / math.sqrt(3) + 0.15
    ang = np.linspace(math.pi / 6, 2 * math.pi + math.pi / 6, 6, endpoint=False)
    nut = hull([[-tm_ - 1.0, y + r * math.cos(t), rc + r * math.sin(t)] for t in ang] +
               [[-tm_ + L.M5_NUT_T, y + r * math.cos(t), rc + r * math.sin(t)] for t in ang])
    nut = nut.transform([[math.cos(H), 0, math.sin(H), 0], [0, 1, 0, 0], [-math.sin(H), 0, math.cos(H), 0]])
    return nut.mirror([1, 0, 0])

def free_end(side):
    """station blocks + 2 funnel seat pads beyond the side's plane (side -1 = -X plane: endR, built for the 45 deg end)."""
    m = None
    for ya, yb in ((131.0, 157.0), (197.0, 223.0)):
        b = _plane_box(side, -8.0, -0.3, ya, yb, 147.5, 179.0); m = b if m is None else m + b
    # in W the 45 deg end plane is spanned by U (radial) and Y and its normal is the pad normal: the pad back is
    # parallel to the plane, 22.4 outside it; arms are straight prisms along t. M3 axis at rho 176.5 (C, from the seat).
    T_BACK, T_SEAT = 22.4, 37.4
    for yp in PADS_Y:
        for ya, yb in ((yp - 10, yp - 6), (yp + 6, yp + 10)):
            m += _plane_box(side, -0.31, T_BACK + 0.01, ya, yb, 162.0, 179.0)             # webs; the screw lane between them
        m += _plane_box(side, -0.31, T_BACK + 0.01, yp - 10, yp + 10, 162.0, 170.0)       # floor under the lane
        m += _plane_box(side, T_BACK, T_SEAT, yp - 10, yp + 10, 166.5, 186.5)             # 20 x 20 x 15 pad, seat face 0.3 off the collar
    for yp in PADS_Y:
        m -= _plane_cyl_t(side, T_BACK - 5, T_SEAT + 5, yp, 176.5, 1.7)                   # M3 x 25: head on the pad back, ~10 into the insert
    return m

def frame(kind):
    jp = kind in ("mid", "endR"); jm = kind in ("mid", "endL")
    gp = 0.05 if kind == "endR" else 0.3; gm = 0.3 if kind == "endR" else 0.05            # bed side keeps 0.05 (flat bed face)
    m = None
    for yc in RIB_Y:
        y0, y1 = yc - RIB_T / 2, yc + RIB_T / 2
        sec = sector(gp, gm, y0, y1)
        r = (annulus(148.0, 153.0, y0, y1) ^ sec) + (annulus(174.0, 179.0, y0, y1) ^ sec)
        a, b = ((-1, -8, yc, 151.0), (+1, -8, yc, 176.0)) if yc < 177 else ((-1, -8, yc, 176.0), (+1, -8, yc, 151.0))
        r += member(a, b) ^ box(-300, 300, y0, y1, -300, 300)
        m = r if m is None else m + r
    for rho, tt in ((176.5, -12.0), (150.5, -10.0)):                                       # X-braces on the outer and inner faces
        m += member((-1, tt, RIB_Y[0], rho), (+1, tt, RIB_Y[1], rho))
        m += member((-1, tt, RIB_Y[1], rho), (+1, tt, RIB_Y[0], rho))
    cuts = None
    tp, tm_ = L.LUG_T
    for y in L.LUG_Y:
        if jp:
            m += _plane_box(+1, -tp, -0.05, y - 7, y + 7, *L.LUG_RHO)
            c = _plane_cyl_t(+1, -tp - 1.0, 1.0, y, 172.0, L.M5_CLR_R); cuts = c if cuts is None else cuts + c
        if jm:
            m += _plane_box(-1, -tm_, -0.05, y - 7, y + 7, *L.LUG_RHO)
            c = _plane_cyl_t(-1, -tm_ - 1.0, 1.0, y, 172.0, L.M5_CLR_R) + nut_pocket_mX(y); cuts = c if cuts is None else cuts + c
    for ys0, ys1 in L.DT_SEG:
        if jp:
            if kind == "mid":
                seg = box(-500, 500, ys0, ys1, -500, 500)
                m += tongue_frame(ys0, ys1) + _plane_box(+1, -tp, -L.JOINT_GAP + 0.01, ys0, ys1, L.DT_RHO[0] + 2.5, L.DT_RHO[1])
            else:                                                                           # endR prints on this face: flat pad, no tongue
                m += _plane_box(+1, -tp, -0.05, ys0, ys1, L.DT_RHO[0], L.DT_RHO[1])
        if jm:
            m += _plane_box(-1, -8.0, -0.05, ys0, ys1, *L.DT_RHO)
    if jm:
        cuts += groove_frame()
    if kind == "endR":
        m += free_end(-1)
    if kind == "endL":
        m += free_end(+1)
    # plain end posts where a side has neither a joint nor pads: none (mid has both joints, ends have joint + pads)
    return m - cuts if cuts is not None else m

POS = dict(frame_mid=[-30, 0, -120, -150, -180], frame_endR=[30], frame_endL=[-210])
BAYS = [-90, -60]
def place(m, p):
    return m.rotate([0, 90 - p, 0])

# ======================================================================= print poses
def pose_bed_normal(m, n):
    """rotate so the outward normal n points down, translate to z 0."""
    t = tm(m)
    R = trimesh.geometry.align_vectors(np.asarray(n, float), [0, 0, -1.0])
    t.apply_transform(R); t.apply_translation([-t.bounds[0][0], -t.bounds[0][1], -t.bounds[0][2]])
    return t

def print_pose(name, m):
    if name == "mast":
        t = tm(m); t.apply_translation([0, 0, -MAST_Z[0]])
    elif name == "kb":
        t = tm(m); t.apply_translation([0, 0, -FLOOR])
    elif name == "cradle_keel":
        t = tm(m.transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 197.0]], float)))
    elif name == "bar_half":
        t = tm(m.transform(np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -LAP_Y[0]]], float)))
        t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(45), [0, 0, 1]))
    elif name == "toe":
        t = tm(m); t.apply_translation([0, 0, -FLOOR])                                       # floor face down
    elif name in ("frame_mid", "frame_endL"):
        t = pose_bed_normal(m, [-math.cos(H), 0, -math.sin(H)])                            # -X plane down
    elif name == "frame_endR":
        t = pose_bed_normal(m, [math.cos(H), 0, -math.sin(H)])                             # +X plane down
    t.apply_translation(-np.array([t.bounds[0][0], t.bounds[0][1], t.bounds[0][2]]))
    if name == "bar_half":
        t.apply_translation([5, 5, 0])
    return t

BUILD = dict(mast=mast, kb=kb, cradle_keel=cradle_keel, bar_half=bar_half, toe=toe,
             frame_mid=lambda: frame("mid"), frame_endR=lambda: frame("endR"), frame_endL=lambda: frame("endL"))
QTY = dict(mast=1, kb=1, cradle_keel=1, bar_half=2, toe=1, frame_mid=5, frame_endR=1, frame_endL=1)

if __name__ == "__main__":
    names = sys.argv[1:] or list(BUILD)
    rep = {}
    for n in names:
        m = BUILD[n]()
        tW = tm(m); tW.export(f"box_{n}_v6_W.stl")
        tp = print_pose(n, m); tp.export(f"box_{n}_v6.stl")
        rep[n] = dict(g=round(tp.volume * D, 1), qty=QTY[n], bodies=len(m.decompose()), watertight=bool(tp.is_watertight),
                      bbox=np.round(tp.extents, 1).tolist())
        print(n, rep[n])
    json.dump(rep, open("stand_v6_prints.json", "w"), indent=1)
