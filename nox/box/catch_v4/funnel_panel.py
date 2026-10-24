"""Box collar funnel v6: flat-printed half-face panels for the A1 mini, 17 Oct 2026.
v6 (Spark 17 Oct 00:55): screwed from ABOVE. Each panel carries one seat block behind the skin near its bottom edge,
at 36 from the face centre line (the wheels only run 50..150 from it). An M3 x 12 countersunk screw goes in square to
the 45 deg face from inside the funnel, through the block, into an M3 heat-set insert in the bay post, whose top is cut
at 45 deg to match the block's flat landing. Straight hex driver, every hole in sight. Replaces v5's pads (screw from
below, blind). Faces, skin, ribs, flanges, keys, 45 deg, 168 / 320: unchanged from the ribbed v4 / v5.
Every panel carries a block (8, 4 used on the 45 / 135 deg bays): one A file + one B file; any 90 deg rotation fits.
Spark 21:19: flat prints only (no supports, no tall upright parts); he sands the seams flush.
8 panels = 4 x A + 4 x B (B = A mirrored). Each prints contact face down on the bed: the screw axis is the print z, so
the countersink is a 45 deg cone over the bed and the landing is a flat top.
  centre seam (plane y = 0): back flanges glued back to back, A carries 2 teardrop key pegs, B the holes.
  valley seam (plane x = y): skins bevelled on the valley plane + back flanges glued back to back;
  aligned by standing the funnel mouth-down on a flat table while the glue cures.
World = seat frame (dimensions G, C): faces 45 deg, bottom edge z 1.3 (168 sq), top z 77.2 (320 sq).
python funnel_panel.py -> out/funnel_panel_A_v6.stl, out/funnel_panel_B_v6.stl (print frame), assembly, json"""
import math, json, os
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out"); os.makedirs(OUT, exist_ok=True)
S2 = math.sqrt(2)
ZB, ZT = 1.3, 77.2                     # face bottom / top edge (G, C)
K0 = 84.0 - ZB                         # contact face: max(|x|,|y|) = K0 + z
SLANT = (ZT - ZB) * S2
BED = 180.0
_J = json.load(open(os.path.join(HERE, "rib_mask.json")))   # from panel_opt.py
_OPT = dict(ts=_J["ts"], h=_J["h"], H=_J["H"], ec=np.asarray(_J["ec"]), mask=np.isin(np.arange(len(_J["ec"])), _J["mask"]))
T, RIB_H = float(_OPT["ts"]), float(_OPT["h"])   # skin, rib height behind the skin (optimizer)
FL_T, FL_W = 2.4, 12.0                 # seam + valley flanges: thickness, depth behind the face
PEG_D, PEG_L, CLR = 4.0, 2.2, 0.15     # teardrop key (tip up in the print frame)
PEG_V = (30.0, 80.0)                   # keys up the slope (DEFAULT)
RIB_SMOOTH = 0.9                       # Gaussian smoothing of the rib field, elements (clean, flowing outlines)
RIB_GROW = 1.0                         # outward offset of the smoothed rib outline (set by the as-built FE check)
DRAFT = 3.0                            # deg, on faces that would sit at exactly 45 deg in print
# ---- seat block (v6). Seat frame -> W: collar centred on the bay modules (W Y 94..182 -> 138, DEFAULT), seat z 0 = W Z
# 159.2 (E, ring stack; the collar keeps v5's height). Face frame at the screw: e_u along the edge (+y), e_v up the slope,
# n into the panel (out and down, (1, 0, -1)/sqrt2) = the screw axis = print z.
BAY_YW = (94.0, 182.0)                 # bay module Y (W, C: box-case.md)
ZP = -1.2                              # skin lip under the bottom edge (v4 / v5 height), the collar's lowest point
SEAT_Y = 36.0                          # along the face from its centre line: W Y 102 / 174; wheels run 50..150 (C, G)
SEAT_V = 14.0                          # up the slope from the bottom edge (the flare stays on the face)
CLAMP = 6.0                            # face to landing, along n: PETG the screw clamps
LAND_R, FLARE_R = 8.0, 13.0            # landing disc (post seat o 16) / flare at the skin's back
KEEP_R = 9.5                           # rib-free room round the axis behind the landing, for the post
CSK_D, CSK_SINK, HOLE_D = 6.6, 0.2, 3.4    # ISO 10642 M3 head o 6.0 (6.72 max), 90 deg; sunk 0.2 below the face
SCREW = "M3 x 12 countersunk (ISO 10642)"   # 6 through the block + 6 into a M3 x 5.7 insert in the post
E_U = np.array([0.0, 1.0, 0.0]); E_V = np.array([1.0, 0.0, 1.0]) / S2; N_IN = np.array([1.0, 0.0, -1.0]) / S2
P_FACE = np.array([84.0, 0.0, ZB]) + SEAT_V * E_V + SEAT_Y * E_U      # screw entry on the contact face (panel A)

box = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
def plane_keep(m, n, d):               # keep n . p >= d
    return m.trim_by_plane([float(v) for v in n], float(d))

def frustum(k_off, z0, z1):
    a0, a1 = K0 + k_off + z0, K0 + k_off + z1
    s0 = mf.Manifold.cube([2 * a0, 2 * a0, 0.001]).translate([-a0, -a0, z0])
    s1 = mf.Manifold.cube([2 * a1, 2 * a1, 0.001]).translate([-a1, -a1, z1 - 0.001])
    return mf.Manifold.batch_hull([s0, s1])

def inv_frustum(c):                    # {r + z <= c}: inverted 45 deg frustum, apex z c
    return mf.Manifold.batch_hull([mf.Manifold.cube([2 * (c + 60)] * 2 + [0.001]).translate([-(c + 60), -(c + 60), -60]),
                                   mf.Manifold.cube([0.02, 0.02, 0.001]).translate([-0.01, -0.01, c - 0.01])])

def teardrop(d, L):
    """key along -y (0 .. -L), roof tip toward +z."""
    r = d / 2
    c = mf.CrossSection.circle(r, 48)
    tip = mf.CrossSection([[(-r / S2, r / S2), (0.0, r * S2), (r / S2, r / S2)]])
    return mf.Manifold.extrude(mf.CrossSection.batch_hull([c, tip]), L).rotate([90, 0, 0])

# print frame: rotate about y by -135 deg so the outward-down normal (1,0,-1)/sqrt2 of the +x face becomes +z
ROT = -135.0
def to_print(m): return m.rotate([0, ROT, 0])
def to_print_pt(p):
    th = math.radians(ROT); x, y, z = p
    return (x * math.cos(th) + z * math.sin(th), y, -x * math.sin(th) + z * math.cos(th))

def opening_cut():
    """the 168 opening, kept open below the bottom edge; walls drafted 3 deg out going down (no 45 deg face in print)."""
    a1 = 84.0 + (ZB - ZP + 1) * math.tan(math.radians(DRAFT))
    sq = lambda a, z: mf.Manifold.cube([2 * a, 2 * a, 0.001]).translate([-a, -a, z])
    return mf.Manifold.batch_hull([sq(84.0, ZB - 0.001), sq(a1, ZP - 1)])

def at_seat(m):
    """local (u, v, w = depth into the panel) -> seat frame, at the screw of panel A."""
    M = np.column_stack([E_U, E_V, N_IN, P_FACE])
    return m.transform(M.tolist())

def seat_block():
    """flat landing disc o 16 at depth CLAMP, flaring to o 26 at the skin's back: the post seats on the landing."""
    land = mf.Manifold.cylinder(0.01, LAND_R, LAND_R, 64).translate([0, 0, CLAMP - 0.01])
    base = mf.Manifold.cylinder(0.01, FLARE_R, FLARE_R, 64).translate([0, 0, 0.5])
    return at_seat(mf.Manifold.batch_hull([land, base]))

def screw_hole():
    """countersink (90 deg, o 6.6 sunk 0.2) + o 3.4 clearance, along n from the face."""
    r0 = CSK_D / 2 + CSK_SINK
    cone = mf.Manifold.cylinder(r0 - HOLE_D / 2 + 1.0, r0 + 1.0, HOLE_D / 2, 64).translate([0, 0, -1.0])
    bore = mf.Manifold.cylinder(CLAMP + 40, HOLE_D / 2, HOLE_D / 2, 48).translate([0, 0, -1.0])
    return at_seat(cone + bore)

def post_room():
    """no ribs behind the landing within KEEP_R: the post comes in along +n."""
    return at_seat(mf.Manifold.cylinder(40, KEEP_R, KEEP_R, 64).translate([0, 0, CLAMP]))

def pilot():                           # what every part loses at the seat (kept name: assembly / FE scripts use it)
    return screw_hole()

def panel_world():
    """+x face, y >= 0 half (panel A), world frame, no keys, no ribs."""
    cav = frustum(0.0, -20, 120)
    top = inv_frustum(K0 + 2 * ZT)                                       # top edge cut square to the skin (v <= SLANT)
    opening = opening_cut()
    skin = ((frustum(T * S2, ZP, ZT + 20) - cav) ^ top) - opening       # skin runs down 2.5 under the edge (v4 lip)
    pad = (seat_block() - cav) ^ top
    fl_reg = (frustum(FL_W * S2, ZB, ZT + 20) - cav) ^ top
    r_max = K0 + ZT + (T + RIB_H) * S2
    own = lambda m: plane_keep(plane_keep(m, (0, 1, 0), 0.0), (1, -1, 0), 0.0)     # y >= 0 and x >= y
    body = own(skin + pad)
    seam_fl = own(fl_reg) ^ box(0, 400, 0, FL_T, -50, 200)                           # y in [0, 2.4]
    n = np.array([1, -1, 0]) / S2
    val_fl = plane_keep(plane_keep(own(fl_reg), -n, -FL_T), n, 0.0)                   # 0 <= dist to x = y <= 2.4
    m = (body + seam_fl + val_fl) ^ box(0, r_max, 0, r_max, ZP, 200)        # nothing past the ribs' back, nothing below ZP
    return m - post_room() - screw_hole()

def key_points():
    wm = T + (FL_W - T) / 2
    return [(84.0 + v / S2 + wm / S2, 0.0, ZB + v / S2 - wm / S2) for v in PEG_V]

def rib_section():
    """Optimizer rib mask (u >= 0 half) -> smooth outline in face coordinates (u, v), rounded, slivers dropped."""
    from scipy import ndimage
    from skimage import measure
    ec, mask, Hm = _OPT["ec"], _OPT["mask"], float(_OPT["H"])
    v0 = float(_OPT.get("v0", 0.0))                               # grid start up the slope (v8 plain: its raised bottom edge)
    if not mask.any(): return None
    iu = np.round((ec[:, 0] + 160 - Hm / 2) / Hm).astype(int); iv = np.round((ec[:, 1] - v0 - Hm / 2) / Hm).astype(int)
    img = np.zeros((iu.max() + 3, iv.max() + 3)); img[iu + 1, iv + 1] = mask
    img = ndimage.gaussian_filter(img, RIB_SMOOTH)
    up = 4; fine = ndimage.zoom(img, up, order=1)
    polys = []
    for cnt in measure.find_contours(fine, 0.5):
        u = (cnt[:, 0] / up - 1) * Hm - 160 + Hm / 2
        v = (cnt[:, 1] / up - 1) * Hm + Hm / 2 + v0
        if len(u) > 8: polys.append(list(zip(u.tolist(), v.tolist())))
    cs = mf.CrossSection(polys, mf.FillRule.EvenOdd)
    dom = mf.CrossSection([[(0.0, 0.0), (84.0, 0.0), (84.0 + SLANT / S2, SLANT), (0.0, SLANT)]])
    cs = (cs ^ dom).offset(-0.8, mf.JoinType.Round).offset(0.8 + RIB_GROW, mf.JoinType.Round) ^ dom
    keep = [p for p in cs.decompose() if p.area() > 60.0]          # drop crumbs: every rib is a real member
    if not keep: return None
    out = keep[0]
    for p in keep[1:]: out = out + p
    return out.simplify(0.15)                                     # no near-duplicate points (STL-safe)

def ribs_print():
    cs = rib_section()
    if cs is None: return mf.Manifold()
    p0 = np.array(to_print_pt((84.0, 0.0, ZB))); p1 = np.array(to_print_pt((84.0 + 10 / S2, 0.0, ZB + 10 / S2)))
    dv = (p1 - p0) / 10                                   # print-frame step per mm of v (along x')
    polys = [[(p0[0] + v * dv[0], u) for (u, v) in poly] for poly in cs.to_polygons()]
    cs2 = mf.CrossSection(polys, mf.FillRule.EvenOdd)
    return mf.Manifold.extrude(cs2, RIB_H + 0.01).translate([0, 0, p0[2] + T - 0.01])

def ribs_world():
    """panel A's ribs in the seat frame, nothing below the seat plane, the pilot kept clear."""
    return ribs_print().rotate([0, -ROT, 0]).trim_by_plane([0.0, 0.0, 1.0], ZP) - post_room() - screw_hole()

def build():
    w = panel_world()
    rb = to_print(ribs_world())
    pa = to_print(w) + rb
    pb = to_print(w.mirror([0, 1, 0])) + rb.mirror([0, 1, 0])
    for p in key_points():
        q = to_print_pt(p)
        pa = pa + teardrop(PEG_D, PEG_L).translate([q[0], 0.0, q[2]])                 # A: pegs point -y
        pb = pb - teardrop(PEG_D + 2 * CLR, FL_T + 2).translate([q[0], 1.0, q[2]])    # B: holes through y in [-2.4, 0]
    return pa, pb

def assembly(pa, pb):
    A = pa.rotate([0, -ROT, 0]); B = pb.rotate([0, -ROT, 0])
    return [m.rotate([0, 0, 90 * k]) for k in range(4) for m in (A, B)]

def collar_world(**kw):
    """the glued collar as one body (FE): the panels' solids built whole so no seam faces touch; keys left out."""
    cav = frustum(0.0, -20, 120); top = inv_frustum(K0 + 2 * ZT)
    skin = ((frustum(T * S2, ZP, ZT + 20) - cav) ^ top) - opening_cut()
    fl_reg = (frustum(FL_W * S2, ZB, ZT + 20) - cav) ^ top
    slab = box(-FL_T, FL_T, -400, 400, -50, 200)
    fl = (slab + slab.rotate([0, 0, 90]) + slab.rotate([0, 0, 45]) + slab.rotate([0, 0, -45])) ^ fl_reg
    blk = (seat_block() - cav) ^ top
    rw = ribs_world().translate([0, 0.02, 0])                            # A and B ribs overlap 0.04 across y = 0
    m = skin + fl
    for k in range(4):
        for q in (blk, rw):
            m = m + q.rotate([0, 0, 90 * k]) + q.mirror([0, 1, 0]).rotate([0, 0, 90 * k])
    r_max = K0 + ZT + (T + RIB_H) * S2
    m = m ^ box(-r_max, r_max, -r_max, r_max, ZP, 200)
    for k in range(4):
        for cut in (post_room(), screw_hole()):
            m = m - cut.rotate([0, 0, 90 * k]) - cut.mirror([0, 1, 0]).rotate([0, 0, 90 * k])
    return m

def tm(m):
    me = m.to_mesh()
    idx = np.arange(len(me.vert_properties))
    if len(me.merge_from_vert): idx[np.asarray(me.merge_from_vert)] = np.asarray(me.merge_to_vert)
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], idx[np.asarray(me.tri_verts)], process=False)

def info_of(t):
    b = t.bounds; n = t.face_normals; c = t.triangles_center
    down = (n[:, 2] < -math.cos(math.radians(45.5))) & (c[:, 2] > b[0, 2] + 0.2)
    bed = (n[:, 2] < -0.999) & (c[:, 2] < b[0, 2] + 0.05)
    return dict(cm3=round(t.volume / 1000, 1), g_PETG=round(t.volume / 1000 * 1.27, 1),
                bbox=[round(v, 1) for v in t.extents], watertight=bool(t.is_watertight),
                bodies=len(t.split(only_watertight=False)), fits_bed=bool(t.extents[:2].max() <= BED and t.extents[2] <= BED),
                bed_mm2=round(float(t.area_faces[bed].sum())), overhang_mm2=round(float(t.area_faces[down].sum()), 1))

if __name__ == "__main__":
    pa, pb = build()
    land = P_FACE + CLAMP * N_IN
    res = dict(seat=dict(screw=SCREW, entry=[round(v, 2) for v in P_FACE], landing=[round(v, 2) for v in land], axis=[round(v, 4) for v in N_IN]))
    for name, m in (("A", pa), ("B", pb)):
        t = tm(m)
        t.apply_translation(-np.array([t.bounds[:, 0].mean(), t.bounds[:, 1].mean(), t.bounds[0, 2]]))
        t = trimesh.Trimesh(t.vertices, t.faces, process=True, validate=True)    # drop zero-area slivers (STL-safe)
        t.export(os.path.join(OUT, f"funnel_panel_{name}_v6.stl"))
        res[name] = info_of(t)
    parts = assembly(pa, pb)
    ov = sum((parts[i] ^ parts[j]).volume() for i in range(8) for j in range(i + 1, 8))
    full = parts[0]
    for p in parts[1:]: full = full + p
    ft = tm(full)
    res["assembly"] = dict(overlap_mm3=round(ov, 2), bbox=[round(v, 1) for v in ft.extents],
                           g_PETG=round(ft.volume / 1000 * 1.27, 1), bodies=len(ft.split(only_watertight=False)),
                           z_min=round(float(ft.bounds[0, 2]), 2))
    ft.export(os.path.join(OUT, "funnel_assembly_v6_world.stl"))
    json.dump(res, open(os.path.join(OUT, "funnel_panel_v6.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))
