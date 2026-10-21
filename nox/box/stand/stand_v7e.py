"""Box v7c layout (19 Oct 2026 20:45): the funnel fills the whole enclosure, full-height corner posts, PSU moved beside the mast.
W coords (box-arm.md). All C unless tagged E. Writes stand_v7c.json + stand_v7c.png.

Geometry rule: the catch funnel is ONE 45 deg pyramid (the v4 collar's faces continued outward) from the 168 square opening
at Z Z_OPEN around the drone centre (X 0, Y YC) until it meets the four walls. The walls' inner faces are fixed by what is
inside (mast + keel at the back, toe at the front, bar tips at the sides); where the pyramid meets a wall sets that wall's
required top height; the walls are cut to one height = the tallest (rear), so a drone that drops anywhere inside the top
rectangle lands on a 45 deg face and rolls to the opening. Beyond the printed collar (mouth 320, 8 panels v6) the pyramid is
continued with clear sheet aprons in the same planes (apron = sheet, held in the posts' top caps and under the collar lip)."""
import json, math, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import manifold3d as mf, stand_v6_parts as P

FLOOR = P.FLOOR
YC, Z_OPEN, HALF_OPEN = 177.0, 161.0, 84.0          # drone centre, collar opening (168 square) height and half size (box-funnel v4)
COLLAR_MOUTH, Z_MOUTH = 160.0, 237.0                 # printed collar: mouth 320 at Z 237
T, GAP = 3.0, 0.2                                    # sheet (DEFAULT), slot clearance
# walls' inner faces, set by what is inside (C):
X_IN = 186.0                                         # ring lugs r 179 + 7 for the wall brackets; bar tips shortened to X 181
Y_BACK = -66.0                                       # mast Y -60 + rear cross bar; PSU beside the mast; KB shortened to Y -63
Y_FRONT = 337.0                                      # = the collar mouth's front edge: the pyramid meets the front wall at Z 237 exactly
# pyramid meets each wall where run = rise: Z = Z_OPEN + (distance from the opening edge)
z_meet = dict(front=Z_OPEN + (Y_FRONT - (YC + HALF_OPEN)), back=Z_OPEN + ((YC - HALF_OPEN) - Y_BACK),
              side=Z_OPEN + (X_IN - HALF_OPEN))
Z_TOP = 240.0                                        # Spark 20:37: walls level with the funnel rim (237); no aprons
H_WALL = Z_TOP - FLOOR
# apron sheets (the pyramid beyond the printed collar): trapezoids in each face's plane
def apron(run_lo, run_hi, w_lo, w_hi):
    """a 45 deg face strip from radius run_lo to run_hi (horizontal), widths at each end; slant length = run * sqrt2"""
    return dict(slant=round((run_hi - run_lo) * math.sqrt(2), 1), w_lo=round(w_lo, 1), w_hi=round(w_hi, 1))
aprons = {
    "side (x2)": apron(COLLAR_MOUTH, X_IN, 2 * COLLAR_MOUTH + 0, (YC - Y_BACK) + (Y_FRONT - YC) - 0),   # at the wall the width is the box length
    "rear": apron(COLLAR_MOUTH, YC - Y_BACK, 2 * COLLAR_MOUTH, 2 * X_IN),
    "front": dict(slant=0.0, note="collar mouth touches the front wall"),
}
# the side apron at the wall spans the box length only between where the rear/front faces cut it: the pyramid's corner
# diagonals. At the side wall (run 205 from the axis): the face is bounded by the diagonals at Y = YC +- (205 - 84 + 84) ... use the
# pyramid: on the side face at horizontal run r from the opening edge, the face spans Y in [YC - 84 - r, YC + 84 + r].
r_side = X_IN - HALF_OPEN
aprons["side (x2)"] = dict(slant=round((X_IN - COLLAR_MOUTH) * math.sqrt(2), 1), w_lo=2 * COLLAR_MOUTH,
                            w_hi=round(min(YC + HALF_OPEN + r_side, Y_FRONT) - max(YC - HALF_OPEN - r_side, Y_BACK), 1))
r_back = YC - HALF_OPEN - Y_BACK
aprons["rear"] = dict(slant=round((r_back - (COLLAR_MOUTH - HALF_OPEN)) * math.sqrt(2), 1), w_lo=2 * COLLAR_MOUTH, w_hi=2 * X_IN)
# sheets
sheets = {"side wall x2": [round(Y_FRONT - Y_BACK + 2 * T), round(H_WALL)], "front / back wall x2": [round(2 * X_IN), round(H_WALL)]}
A_walls = 2 * sheets["side wall x2"][0] * sheets["side wall x2"][1] + 2 * sheets["front / back wall x2"][0] * sheets["front / back wall x2"][1]
A_aprons = 0 * 2 * aprons["side (x2)"]["slant"] * (aprons["side (x2)"]["w_lo"] + aprons["side (x2)"]["w_hi"]) / 2 + aprons["rear"]["slant"] * (aprons["rear"]["w_lo"] + aprons["rear"]["w_hi"]) / 2
m_sheet = (A_walls + A_aprons) * 1e-6 * T * 1.19     # kg acrylic (E)
# corner posts: full height, 3 segments (bed 180), 25 x 25 L section with two 3.2 slots 10 deep, wall 4 outside, 3 inside
POST = dict(n=4, segments=3, seg_len=round(H_WALL / 3, 1), section=25.0, slot=T + GAP, slot_depth=10.0, g_each_seg=38.0)  # E
# tip-over (E): base ladder + walls on the floor, footprint = walls' outer faces
m_box = 4.13 + 0.15 + 1.0 - 0.03                     # v6 + 2 cross bars + PSU beside the mast - KB shortening
cg_box = np.array([0.0, 85.0, -14.0])
m_posts = POST["n"] * POST["segments"] * POST["g_each_seg"] / 1000
m_tot = m_box + m_sheet + m_posts
cg = (m_box * cg_box + (m_sheet + m_posts) * np.array([0, (Y_BACK + Y_FRONT) / 2, (FLOOR + Z_TOP) / 2 - 30])) / m_tot
def margin(push, pt):
    worst = 9
    for lever in (X_IN + T - cg[0], X_IN + T + cg[0], Y_FRONT + T - cg[1], cg[1] - (Y_BACK - T)):
        worst = min(worst, m_tot * 9.81 * lever / 1000 / (push * (pt[2] - FLOOR) / 1000))
    return worst
k = margin(13, (0, YC, Z_TOP))
# catch tolerance: any drop inside the top rectangle lands on a 45 deg face
tol = dict(x=round(X_IN - HALF_OPEN), y_back=round(YC - HALF_OPEN - Y_BACK), y_front=round(Y_FRONT - YC - HALF_OPEN),
           v4_collar=round(COLLAR_MOUTH - HALF_OPEN))
out = dict(walls_inner=dict(x=X_IN, y=[Y_BACK, Y_FRONT]), z_meet=z_meet, z_top=Z_TOP, wall_height=H_WALL, sheets=sheets, aprons=aprons,
           sheet_mass_kg=round(m_sheet, 2), posts=POST, posts_mass_kg=round(m_posts, 2), tipover=round(k, 2), total_kg=round(m_tot, 2),
           catch_tolerance_mm=tol, psu="beside the mast, X 25..89, Y -62..-22, on the floor (was behind the keel)")
json.dump(out, open("stand_v7e.json", "w"), indent=1, default=float)
for kk, v in out.items(): print(kk, v)

# ------------------------------------------------------------------ render
def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(extents=[x1-x0, y1-y0, z1-z0], transform=trimesh.transformations.translation_matrix([(x0+x1)/2, (y0+y1)/2, (z0+z1)/2]))
NEW, OLD, ARM, WALL, POSTC, FUN, APR = (0.11, 0.44, 0.54), (0.69, 0.55, 0.36), (0.6, 0.63, 0.66), (0.55, 0.8, 0.9), (0.85, 0.35, 0.2), (0.72, 0.72, 0.68), (0.75, 0.88, 0.95)
parts = []
for n in ("mast", "cradle_keel", "toe"): parts.append((trimesh.load(f"box_{n}_v6_W.stl"), NEW, 1.0))
bh = trimesh.load("box_bar_half_v6_W.stl").slice_plane([181, 0, 0], [-1, 0, 0]); parts.append((bh, NEW, 1.0))
kb = trimesh.load("box_kb_v6_W.stl"); kb = kb.slice_plane([0, Y_BACK + 3, 0], [0, 1, 0]); parts.append((kb, NEW, 1.0))
b = man(bh).rotate([0, 0, 180]).translate([0, 108.0, 0]); parts.append((P.tm(b), NEW, 1.0))
for n, ps in P.POS.items():
    m = man(trimesh.load(f"box_{n}_v6_W.stl"))
    for p in ps: parts.append((P.tm(P.place(m, p)), NEW, 1.0))
mod = trimesh.load("models/box_baymod_v4i_W.obj", force="mesh")
for pb in P.BAYS:
    t = mod.copy(); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 - pb), [0, 1, 0])); parts.append((t, OLD, 1.0))
hub = trimesh.creation.annulus(40, 86, 59, sections=48); hub.apply_translation([0, 0, 37.5]); hub.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [1, 0, 0])); parts.append((hub, ARM, 1.0))
# cross bars front / rear + toe extension + slot blocks
parts += [(box(-181, 181, Y_FRONT - 16, Y_FRONT, FLOOR, FLOOR + 16), NEW, 1.0), (box(-181, 181, Y_BACK, Y_BACK + 16, FLOOR, FLOOR + 16), NEW, 1.0), (box(-8, 8, 260, Y_FRONT - 16, FLOOR, FLOOR + 8), NEW, 1.0)]
# ring-to-wall brackets: at the joint planes -15, 15, 45 deg (right) and their mirrors (left): a block from the joint lug (r 172..179) to the sheet
BR = []
for sx in (1, -1):
    for ang in (-15, 15, 45):
        a = math.radians(ang); x, z = 172 * math.cos(a), 172 * math.sin(a)
        for y in (145, 209):
            BR.append(box(min(sx * x, sx * (X_IN - 0.3)), max(sx * x, sx * (X_IN - 0.3)), y - 9, y + 9, z - 9, z + 9))
for b_ in BR: parts.append((b_, POSTC, 1.0))
psu = box(25, 89, -62, -22, FLOOR, FLOOR + 40); parts.append((psu, (0.3, 0.3, 0.3), 1.0))
# funnel: collar (printed, grey) from the opening to the mouth, aprons (sheet) from the mouth to the walls, clipped by the walls
def pyr(z0, z1, half0, half1):
    v = [[-half0, YC - half0, z0], [half0, YC - half0, z0], [half0, YC + half0, z0], [-half0, YC + half0, z0],
         [-half1, YC - half1, z1], [half1, YC - half1, z1], [half1, YC + half1, z1], [-half1, YC + half1, z1]]
    f = [[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return trimesh.Trimesh(v, f)
collar = pyr(Z_OPEN, Z_MOUTH, HALF_OPEN, COLLAR_MOUTH); parts.append((collar, FUN, 0.9))
big = pyr(Z_MOUTH, Z_TOP + 1, COLLAR_MOUTH, COLLAR_MOUTH + (Z_TOP + 1 - Z_MOUTH))
# clip the aprons to the walls' inner box
clipper = man(box(-X_IN, X_IN, Y_BACK, Y_FRONT, Z_MOUTH - 1, Z_TOP + 2))
apr = P.tm(man(big) ^ clipper) if False else None
# aprons as thin slabs: build each face as a quad clipped to the wall box
def face_quad(side):
    s = 2000
    if side == "side+": pts = [[COLLAR_MOUTH, YC - COLLAR_MOUTH, Z_MOUTH], [COLLAR_MOUTH, YC + COLLAR_MOUTH, Z_MOUTH], [COLLAR_MOUTH + s, YC + COLLAR_MOUTH + s, Z_MOUTH + s], [COLLAR_MOUTH + s, YC - COLLAR_MOUTH - s, Z_MOUTH + s]]
    if side == "side-": pts = [[-COLLAR_MOUTH, YC - COLLAR_MOUTH, Z_MOUTH], [-COLLAR_MOUTH, YC + COLLAR_MOUTH, Z_MOUTH], [-COLLAR_MOUTH - s, YC + COLLAR_MOUTH + s, Z_MOUTH + s], [-COLLAR_MOUTH - s, YC - COLLAR_MOUTH - s, Z_MOUTH + s]]
    if side == "back": pts = [[-COLLAR_MOUTH, YC - COLLAR_MOUTH, Z_MOUTH], [COLLAR_MOUTH, YC - COLLAR_MOUTH, Z_MOUTH], [COLLAR_MOUTH + s, YC - COLLAR_MOUTH - s, Z_MOUTH + s], [-COLLAR_MOUTH - s, YC - COLLAR_MOUTH - s, Z_MOUTH + s]]
    q = trimesh.Trimesh(pts, [[0, 1, 2], [0, 2, 3]])
    n = q.face_normals[0]; slab = q.copy(); slab.apply_translation(-n * 3)
    m = trimesh.util.concatenate([q, slab]); hullm = trimesh.convex.convex_hull(m.vertices)
    return P.tm(man(hullm) ^ clipper)

walls = [box(X_IN, X_IN + T, Y_BACK - T, Y_FRONT + T, FLOOR, Z_TOP), box(-X_IN - T, -X_IN, Y_BACK - T, Y_FRONT + T, FLOOR, Z_TOP),
         box(-X_IN, X_IN, Y_FRONT, Y_FRONT + T, FLOOR, Z_TOP), box(-X_IN, X_IN, Y_BACK - T, Y_BACK, FLOOR, Z_TOP)]
for w in walls: parts.append((w, WALL, 0.25))
for sx in (1, -1):
    for y, sy in ((Y_BACK, -1), (Y_FRONT, 1)):
        x0, x1 = (X_IN - 12, X_IN + T + 7) if sx > 0 else (-X_IN - T - 7, -X_IN + 12)
        y0, y1 = (y - 12, y + T + 7) if sy > 0 else (y - T - 7, y + 12)
        parts.append((box(x0, x1, y0, y1, FLOOR, Z_TOP), POSTC, 1.0))
fig = plt.figure(figsize=(17, 9), dpi=120)
for k_, (el, az, ttl) in enumerate([(18, -52, "front-right"), (24, 214, "rear-left")]):
    ax = fig.add_subplot(1, 2, k_ + 1, projection="3d")
    L = np.array([0.4, -0.6, 0.7]); L /= np.linalg.norm(L)
    for t, col, al in parts:
        if len(t.faces) > 5000: t = t.simplify_quadric_decimation(face_count=5000)
        sh = 0.45 + 0.55 * np.clip(t.face_normals @ L, 0, 1)
        fc = np.clip(np.array(col)[None, :] * sh[:, None] * 1.25, 0, 1)
        ax.add_collection3d(Poly3DCollection(t.triangles, facecolors=fc, edgecolors="none", alpha=al))
    ax.set_xlim(-220, 220); ax.set_ylim(-110, 360); ax.set_zlim(-205, 255); ax.set_box_aspect((480, 480, 460))
    ax.view_init(elev=el, azim=az); ax.set_axis_off(); ax.text2D(0.04, 0.93, ttl, transform=ax.transAxes, fontsize=11)
fig.suptitle(f"Nox box v7e, compact: walls {2*X_IN:.0f} x {Y_FRONT-Y_BACK:.0f} inside, {H_WALL:.0f} tall (level with the funnel rim); the ring bolted to the side walls through 12 brackets at its joint planes (orange blocks);\n"
             f"full-height corner posts, 3-bar base ladder, PSU beside the mast. Tip-over {k:.1f}, no ballast.", fontsize=10)
fig.subplots_adjust(0, 0, 1, 0.92, 0, 0); fig.savefig("stand_v7e.png"); print("stand_v7e.png")
