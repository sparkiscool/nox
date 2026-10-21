"""Box stand v7 printed parts (19 Oct 2026 21:05; Spark 20:43 bolt the ring to the side walls, box as compact as possible;
21:03 design all of it). Builds on stand_v6_parts (mast, cradle_keel, frame_mid unchanged, still printed from the v6 files).
W coords (box-arm.md). Enclosure: 4 clear 3.0 sheets, inner faces X +-186, Y -66..337, Z -201..240 (441 tall), standing on
the floor in the corner posts' slots. Changed / new parts (box_<name>_v7.stl in print pose, *_W.stl in W):
  bar_half   cross bar half X -20..181 (print 2): v6 truss, web 2.4, + tip block X 170..192 with a sheet slot and an M5 to the sheet
  toe        cradle back -> front wall: foot bar to Y 331 + plate X +-32 at Y 331..337 with 2 M5 nut bosses (front sheet bolts to it)
  kb         back keel Y -66..40: rear plate X +-32 at Y -66..-60 with 2 M5 nut bosses (rear sheet), gusset, PSU zip slots in the +X wall
  frame_endR / frame_endL  v6 end frames with 5.0 pad webs (was 4) and 2 x M5 along Y through the station blocks (end bracket)
  post_seg   corner post segment, 147 tall, print 12 (3 per corner, identical, placed by 90 deg turns): L 9.2 x 21.7, both
             sheets in 3.2 slots 15 deep, 1 x M3 per leg; segments butt, the sheets keep them in line
  bracket_jA ring-to-wall bracket at the -15 deg joint (frame -30 / 0) and, turned 180 about Y, at 165 deg (-180 / endL)
  bracket_jB same at the +15 deg joint (0 / endR) and at 195 deg (-150 / -180): U-clip on the joint's lug pair (the joint
             bolts become M5 x 25 through the clip's cheeks), two arms to a flange on the sheet (2 x M3 through the sheet)
  bracket_e  end bracket at 45 deg (endR's free end; the mirror position 135 deg = the same part turned about the vertical
             axis through Y 177): cheeks on the station blocks' outer Y faces, 2 x M5 x 35 along Y, two arms to a flange on
             the sheet (2 x M3)
No inner bolt pair on the frames (the full-length dovetail groove at rho 150..164 leaves no room; the brackets fix the
joints, ring_chain_fe.py config 'brackets'). Tags: C unless noted. python3 stand_v7_parts.py [part] -> STLs + stand_v7_prints.json"""
import json, math, sys, os
import numpy as np, trimesh
import manifold3d as mf
import arm_layout as L
from arm_cad import box, cylY, cylX, cylZ, hull, _plane_box, _plane_cyl_t
import stand_v6_parts as S
from stand_v6_parts import tm, prism_xz, prism_yz, FLOOR, KTOP, KW, D, H

T_SHEET, SLOT = 3.0, 3.2
BLK_T0, EB_T = -11.0, -5.9        # 20 Oct: station blocks / end-bracket cheeks start at t -11 (was -8) and the M5 sits at t -5.9 (was -4.4): walls >= 2.35 round the hole (were 0.85..1.1)                       # acrylic 3.0 (DEFAULT), slot = sheet + 0.2
X_IN, Y_BACK, Y_FRONT, Z_TOP = 186.0, -66.0, 337.0, 240.0
NUT_AF, NUT_T = L.M5_NUT_AF, L.M5_NUT_T

def tm0(m):
    """64-bit export (to_mesh's float32 collapses hull vertices into degenerate faces), then trimesh's merge."""
    g = m.to_mesh64(); return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)
def hexY(x, z, y0, y1, af=NUT_AF + 0.3):
    """hex pocket along Y with its flats facing X (corners up / down): the side walls of a 16-wide boss stay 3.7 thick."""
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(a), z + r * math.sin(a)) for a in np.radians(np.arange(30, 390, 60))]
    return prism_xz(pts, y0, y1)
def hexX(y, z, x0, x1, af=NUT_AF + 0.3):
    r = af / math.sqrt(3)
    pts = [(y + r * math.cos(a), z + r * math.sin(a)) for a in np.radians(np.arange(0, 360, 60))]
    return prism_yz(pts, x0, x1)

# ======================================================================= cross bar half v7: X -20..181, tip block to the sheet
BAR_X1 = 181.0
def bar_half():
    y0, ym, y1 = S.LAP_Y
    from shapely.geometry import Polygon, LineString
    lap = box(-20, 20.5, y0, ym, FLOOR, KTOP)
    ZT = -193.0
    P = Polygon([(20, FLOOR), (30, FLOOR), (30, FLOOR + 1), (165, FLOOR + 1), (165, FLOOR), (BAR_X1, FLOOR),
                 (BAR_X1, ZT), (20, KTOP)])
    inner = P.buffer(-4.5, join_style=2)
    zig = [(24, FLOOR + 1), (50, None), (76, FLOOR + 1), (102, None), (128, FLOOR + 1), (154, None), (170, FLOOR + 1)]
    pts = []
    for x, z in zig:
        ztop = KTOP + (x - 20) / (BAR_X1 - 20) * (ZT - KTOP)
        pts.append((x, ztop if z is None else z))
    diag = LineString(pts).buffer(2.4, cap_style=2, join_style=2)
    win = inner.difference(diag)
    geoms = [g for g in getattr(win, "geoms", [win]) if g.area > 25]
    arm = prism_xz(list(P.exterior.coords)[:-1], y0, y1)
    for g in geoms:
        arm -= prism_xz(list(g.buffer(-0.01).exterior.coords)[:-1], y0 - 1, y1 + 1)
    # tip pad: X 177..186 against the side sheet's inner face, full bar width, Z floor..-181; M5 x 16 along X at Z -186,
    # head on the pad's -X face (key along X over the bar's foot), nut + washer outside the sheet
    blk = box(177.0, X_IN, y0, y1, FLOOR, -181.0)
    for ry0, ry1 in ((y0, y0 + 3.0), (y1 - 3.0, y1)):                        # 20 Oct: two 3.0 ribs either side of the M5 head
        blk += prism_xz([(165.0, -193.0), (177.0, -193.0), (177.0, -181.0)], ry0, ry1)   # (one gusset sat where the head goes, and the hole left it 1.5)
    blk -= cylX(54.0, -186.0, 176.0, 200.0, 2.75)
    m = lap + arm + blk
    m -= S.joint_holes(y0 - 1, y1 + 1)
    return m

# ======================================================================= toe v7: to the front wall
def toe():
    m = box(-20, 20, 197.0, 201.0, FLOOR, KTOP)                                          # upstand on the cradle's back face
    m += box(-10, 10, 201.0, 331.0, FLOOR, FLOOR + 10)                                   # foot bar
    m += prism_yz([(201.0, FLOOR + 9), (201.0, KTOP), (218.0, FLOOR + 9)], -4, 4)        # gusset at the cradle
    m += box(-36, 36, 331.0, Y_FRONT, FLOOR, KTOP)                                       # front plate on the sheet (X +-36, 20 Oct)
    for s in (1, -1):
        m += box(min(s * 18, s * 34), max(s * 18, s * 34), 325.0, 331.0, FLOOR, -183.0)   # nut bosses behind the plate, 16 wide
        m -= cylY(s * 26, -191.0, 320.0, Y_FRONT + 1, 2.75)
        m -= hexY(s * 26, -191.0, 324.0, 331.0 - 2.0)                                     # nut pocket open toward -Y
        m -= cylY(s * 10, -191.0, 196.0, 202.0, 2.75)                                     # cradle bolts
    m += prism_yz([(331.0, FLOOR + 9), (331.0, KTOP), (314.0, FLOOR + 9)], -4, 4)        # gusset at the plate
    return m

# ======================================================================= kb v7: Y -66..40, rear plate on the sheet
def kb():
    y0, y1 = Y_BACK, S.KB_Y[1]
    m = S.ychannel(y0 + 6.0, y1)
    m += box(-20, 20, y1, y1 + S.PLATE_T, FLOOR, KTOP)                                   # end plate Y 40..46 (base joint)
    s = S.SPIG
    m += box(-s["x"], s["x"], s["y0"], s["y1"], FLOOR, s["ztop"])
    m -= box(-s["x"] + s["wall"], s["x"] - s["wall"], s["y0"] + s["wall"], s["y1"] - s["wall"], FLOOR + KW, s["ztop"] + 1)
    m += prism_yz([(s["y1"], FLOOR + 1), (s["y1"], -132.0), (y1 + 0.5, FLOOR + 1)], -1.5, 1.5)        # front gusset
    m += prism_yz([(s["y0"], FLOOR + 1), (s["y0"], -140.0), (y0 + 6.0, KTOP), (y0 + 6.0, FLOOR + 1)], -1.5, 1.5)   # rear gusset onto the plate
    m += box(-36, 36, y0, y0 + 6.0, FLOOR, KTOP)                                         # rear plate X +-36 on the sheet (20 Oct)
    for sx in (1, -1):
        m += box(min(sx * 18, sx * 34), max(sx * 18, sx * 34), y0 + 6.0, y0 + 12.0, FLOOR, -183.0)   # nut bosses, 16 wide
        m -= cylY(sx * 26, -191.0, y0 - 1, y0 + 20, 2.75)
        m -= hexY(sx * 26, -191.0, y0 + 8.0, y0 + 13.0)                                   # nut pocket open toward +Y
    for y, z in S.XBOLTS:
        m -= cylX(y, z, -21, 21, 2.75, 48)
    m -= S.joint_holes(y1 - 1, y1 + S.PLATE_T + 1)
    for yz in (-40.0, -20.0):                                                             # PSU zip-tie slots, +X wall only
        m -= box(16, 21, yz - 4, yz + 4, -192.0, -188.5)
    return m

# ======================================================================= end frames v7: 5.0 pad webs + bracket bolt holes
PADS_Y = S.PADS_Y
def free_end_v7(side):
    m = None
    for ya, yb in ((131.0, 157.0), (197.0, 223.0)):
        b = _plane_box(side, BLK_T0, -0.3, ya, yb, 147.5, 179.0); m = b if m is None else m + b
    T_BACK, T_SEAT = 22.4, 37.4
    for yp in PADS_Y:
        for ya, yb in ((yp - 11, yp - 6), (yp + 6, yp + 11)):
            m += _plane_box(side, -0.31, T_BACK + 0.01, ya, yb, 162.0, 179.0)             # webs 5.0 (v6: 4.0)
        m += _plane_box(side, -0.31, T_BACK + 0.01, yp - 11, yp + 11, 162.0, 170.0)       # floor under the lane
        m += _plane_box(side, T_BACK, T_SEAT, yp - 11, yp + 11, 166.5, 186.5)             # 22 x 20 x 15 pad
    for yp in PADS_Y:
        m -= _plane_cyl_t(side, T_BACK - 5, T_SEAT + 5, yp, 176.5, 1.7)                   # M3 x 25
    # end bracket bolts: M5 along Y through both station blocks at (t EB_T, rho 170)
    for y0_, y1_ in ((126.0, 162.0), (192.0, 228.0)):
        c = cylY(0, 0, y0_, y1_, 2.75).translate([EB_T, 0, 170.0])
        c = c.transform([[math.cos(H), 0, math.sin(H), 0], [0, 1, 0, 0], [-math.sin(H), 0, math.cos(H), 0]])
        m -= c if side > 0 else c.mirror([1, 0, 0])
    return m

def frame_end(kind):
    old = S.free_end; S.free_end = free_end_v7
    try: return S.frame(kind)
    finally: S.free_end = old

# ======================================================================= corner post segment (right-front corner, X +186 / Y +337)
SEG = 147.0; LEG_T = 3.0 + SLOT + 3.0; LEG_L = 15.0 + SLOT + 3.0
def post_seg():
    cx, cy = X_IN, Y_FRONT                     # inner faces' corner
    z0, z1 = 0.0, SEG                          # segment in its own Z (stacked: Z -201 + k*147)
    A = box(cx - 3.0, cx + SLOT + 3.0, cy - 15.0, cy + SLOT + 3.0, z0, z1)               # leg along -Y, holds the side sheet
    B = box(cx - 15.0, cx + SLOT + 3.0, cy - 3.0, cy + SLOT + 3.0, z0, z1)               # leg along -X, holds the front sheet
    m = A + B
    m -= box(cx, cx + SLOT, cy - 16.0, cy - 3.0, z0 - 1, z1 + 1)                         # side sheet slot, closed at Y 334
    m -= box(cx - 16.0, cx - 3.0, cy, cy + SLOT, z0 - 1, z1 + 1)                         # front sheet slot, closed at X 183
    # the slots stop 3 before the corner: a 9 x 9 solid corner keeps the segment one piece; sheets end 3 short of the corner
    m -= cylX(cy - 8.0, SEG / 2, cx - 4, cx + 7, 1.7)                                    # M3 through leg A + sheet
    m -= cylY(cx - 8.0, SEG / 2, cy - 4, cy + 7, 1.7)                                    # M3 through leg B + sheet
    return m

# ======================================================================= ring-to-wall brackets
TP, TM = L.LUG_T                                # 7 head side (+angle), 9 nut side (-angle)
def jframe(a):
    """(t, Y, rho) -> W for the joint plane at angle a (deg): t along increasing angle."""
    a = math.radians(a)
    return np.array([[-math.sin(a), 0, math.cos(a), 0], [0, 1, 0, 0], [math.cos(a), 0, math.sin(a), 0]], float)
def jbox(a, t0, t1, y0, y1, r0, r1):
    return box(t0, t1, y0, y1, r0, r1).transform(jframe(a))
def jcyl_t(a, t0, t1, y, rho, r):
    return mf.Manifold.cylinder(t1 - t0, r, r, 32).rotate([0, 90, 0]).translate([t0, y, rho]).transform(jframe(a))
def V64(m): return np.asarray(m.to_mesh64().vert_properties)[:, :3].tolist()      # exact corners for hulls (tm's float32 makes slivers)

def bracket_j(a):
    """U-clip on the lug pair at joint angle a (right side), arms to the sheet at X +186."""
    ARMS = ((137.0, 149.7), (204.3, 217.0)); CH = 3.7
    zc = 172.0 * math.sin(math.radians(a))
    fl = box(X_IN - 4.0, X_IN, 137.0, 217.0, zc - 16.0, zc + 16.0)                       # flange on the sheet
    m = fl
    for ya, yb in ARMS:
        saddle = jbox(a, -TM - 0.3 - CH, TP + 0.3 + CH, ya, yb, 179.6, 183.6)
        pad = box(X_IN - 3.5, X_IN - 0.5, ya + 0.2, yb - 0.2, zc - 15.5, zc + 15.5)       # strictly inside the flange (no coincident faces)
        arm = hull(V64(saddle) + V64(pad))
        # (the hull of saddle + pad stays outside r 179.6 by construction: checked in stand_v7_check)
        cheeks = jbox(a, -TM - 0.3 - CH, -TM - 0.3, ya, yb, 165.0, 183.6) + jbox(a, TP + 0.3, TP + 0.3 + CH, ya, yb, 165.0, 183.6)
        m += arm + cheeks
    for y in L.LUG_Y:
        m -= jcyl_t(a, -TM - CH - 2, TP + CH + 2, y, 172.0, L.M5_CLR_R)                  # the joint bolt, M5 x 25
    for y in (160.0, 194.0):
        m -= cylX(y, zc, X_IN - 6, X_IN + 1, 1.7)                                         # M3 to the sheet
    return m

BE_T0, BE_T1, BE_YB, BE_CB, BE_R0 = -18.0, 4.0, 10.0, 5.0, 153.5   # 20 Oct 14:30 (neck check): bracket_e cheek widened on the open side to t -18,
# a 10-long bolt boss outside each cheek (t to +4: the end frame starts at Y 130, so beyond the end plane is free there),
# the M5 head in a r 5 counterbore through it (key access from outside); cheek from r 153.5 (box-arm sweep keep-out); v7b's lower cheek hung on 2.35 walls beside the M5


def be_cheek(a, c0, c1):
    """one end-bracket cheek in W: the clamp plate on the station block face (Y c0..c1) + the bolt boss outside it."""
    out = c0 < 177.0                                                     # cheek on the -Y end: boss toward -Y
    yb0, yb1 = (c0 - BE_YB, c0) if out else (c1, c1 + BE_YB)
    plate = jbox(a, BE_T0, -0.8, c0, c1, BE_R0, 183.6)
    boss = jbox(a, BE_T0, BE_T1, yb0, yb1, BE_R0, 183.6)
    return plate + boss


def be_counterbore(a, c0, c1):
    out = c0 < 177.0
    y0, y1 = (c0 - BE_YB - 30.0, c0) if out else (c1, c1 + BE_YB + 30.0)
    return cylY(0, 0, y0, y1, BE_CB).translate([EB_T, 0, 170.0]).transform(jframe(a))


def bracket_e():
    """end bracket at 45 deg (v7c): cheeks on the station blocks' outer Y faces (Y 126..131 / 223..228, t -18..-0.8,
    rho 153.5..183.6) + a 10-long bolt boss outside each, M5 x 35 along Y through cheek + block, head in a r 5 counterbore,
    arms to the flange on the sheet. Symmetric about Y 177: the 135 deg one is this part turned 180 deg about the vertical
    axis through (X 0, Y 177)."""
    a = 45.0
    zc = 183.6 * math.sin(math.radians(a)) - 2.0
    fl = box(X_IN - 4.0, X_IN, 126.0 - BE_YB, 228.0 + BE_YB, zc - 16.0, zc + 16.0)
    m = fl
    for ya, yb, c0, c1 in ((126.0 - BE_YB, 139.0, 126.0, 131.0), (215.0, 228.0 + BE_YB, 223.0, 228.0)):
        ch = be_cheek(a, c0, c1)
        saddle = jbox(a, BE_T0, -0.8, ya, yb, 179.6, 183.6) + jbox(a, BE_T0, BE_T1, ya if c0 < 177 else c1, c0 if c0 < 177 else yb, 179.6, 183.6)
        pad = box(X_IN - 3.5, X_IN - 0.5, ya + 0.2, yb - 0.2, zc - 15.5, zc + 15.5)       # strictly inside the flange (no coincident faces)
        arm = hull(V64(saddle) + V64(pad))
        m += arm + ch
    for c0, c1 in ((126.0, 131.0), (223.0, 228.0)):
        m -= be_counterbore(a, c0, c1)
    for y0_, y1_ in ((119.0, 140.0), (214.0, 235.0)):
        m -= cylY(0, 0, y0_, y1_, 2.75).translate([EB_T, 0, 170.0]).transform(jframe(a))
    for y in (150.0, 204.0):
        m -= cylX(y, zc, X_IN - 6, X_IN + 1, 1.7)
    return m

# ======================================================================= placements (W) for checks and renders
def post_place(m, corner):
    """corner (sx, sy): turn the right-front post about the vertical axis through (0, (Y_BACK+Y_FRONT)/2)... posts are
    placed by 90 deg turns about the box centre; the box is not square, so turn about Z then shift."""
    sx, sy = corner
    yc = (Y_BACK + Y_FRONT) / 2
    if (sx, sy) == (1, 1): return m
    if (sx, sy) == (-1, -1): return m.translate([0, -yc, 0]).rotate([0, 0, 180]).translate([0, yc, 0])
    if (sx, sy) == (-1, 1):                       # legs toward -Y and +X: corner (-186, 337): rotate +90 about Z around (0, 0) then shift
        r = m.rotate([0, 0, 90])                   # (x,y)->(-y,x): corner (186,337)->(-337,186)
        return r.translate([-X_IN + 337.0, Y_FRONT - 186.0, 0])
    if (sx, sy) == (1, -1):
        r = m.rotate([0, 0, -90])                  # (x,y)->(y,-x): (186,337)->(337,-186)
        return r.translate([X_IN - 337.0, Y_BACK + 186.0, 0])

def bracket_place(name, m, where):
    if where == "R": return m
    if name in ("bracket_jA", "bracket_jB"): return m.rotate([0, 180, 0])
    return m.translate([0, -177.0, 0]).rotate([0, 0, 180]).translate([0, 177.0, 0])

# ======================================================================= print poses
def print_pose(name, m):
    if name == "bar_half":
        t = tm0(m.transform(np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -S.LAP_Y[0]]], float)))
        t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(45), [0, 0, 1]))
    elif name in ("toe", "kb"):
        t = tm0(m); t.apply_translation([0, 0, -FLOOR])
    else:
        n = {"frame_endL": [-math.cos(H), 0, -math.sin(H)], "frame_endR": [math.cos(H), 0, -math.sin(H)]}.get(name, [1, 0, 0])
        t = tm0(m); R = trimesh.geometry.align_vectors(np.asarray(n, float), [0, 0, -1.0]); t.apply_transform(R)   # post: leg A's outer face down; brackets: flange down
    t.apply_translation(-np.array([t.bounds[0][0], t.bounds[0][1], t.bounds[0][2]]))
    if name == "bar_half": t.apply_translation([5, 5, 0])
    return t

BUILD = dict(bar_half=bar_half, toe=toe, kb=kb, frame_endR=lambda: frame_end("endR"), frame_endL=lambda: frame_end("endL"),
             post_seg=post_seg, bracket_jA=lambda: bracket_j(-15.0), bracket_jB=lambda: bracket_j(15.0), bracket_e=bracket_e)
QTY = dict(bar_half=2, toe=1, kb=1, frame_endR=1, frame_endL=1, post_seg=12, bracket_jA=2, bracket_jB=2, bracket_e=2)

if __name__ == "__main__":
    names = sys.argv[1:] or list(BUILD)
    rep = {}
    for n in names:
        m = BUILD[n]()
        tag = os.environ.get("VTAG", "v7")
        tW = tm0(m); tW.export(f"box_{n}_{tag}_W.stl")
        tp = print_pose(n, m); tp.export(f"box_{n}_{tag}.stl")
        from solid_check import solid_check                                        # 20 Oct: no part leaves without the strict check
        ok, info = solid_check(tW); print(n, "solid_check", "PASS" if ok else "FAIL", info)
        rep[n] = dict(g=round(tp.volume * D, 1), qty=QTY[n], bodies=len(m.decompose()), watertight=bool(tp.is_watertight),
                      bbox=np.round(tp.extents, 1).tolist())
        print(n, rep[n])
    json.dump(rep, open(f"stand_{os.environ.get('VTAG', 'v7')}_prints.json", "w"), indent=1)
