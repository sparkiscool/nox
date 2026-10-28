"""Scout v3 camera mount v2 (piece of System Scout v3, dimensions.md "System Scout v3": Camera (one), Strength, Rods).

Spark 27 Oct: "the camera mount looks shit lets run it through topology" (v1, drawn facets and trusses, replaced) and "lets
just stick with 3mm rods for the scout" (every Scout rod 3.0 solid carbon: clip bores 3.2). Kept from his first asks: it goes
on after the cage is built (no rod through it), a lens crash hood >= 3 ahead of the lens and outside the 89 x 73 field + 2
deg, a SpaceX / cyber "construction" look (the topology's own struts; nothing drawn for looks), the clips are the fuse.

System pass (runs/scout_camera_mount/v2/concept.json, nox/scout/v3/camera_mount_v2_concept.py): with every rod 3.0 the cage
itself has its first modes at 58 and 71 Hz (F), so a camera clipped only to cage rods rides them (46..51 Hz F wherever the
clips sit, next to nodes included). The camera is carried by the core instead: a tie through the cage's bottom-front cell
(nodes 14-15-16) to a foot on the Bee35 plate top, held by the belly mount's 2 front plate bolts (30, +-13) (longer bolts,
no new hole), slotted +-1.5 in x so the clips set x; plus 2 snap clips on rod 14-15 next to nodes 14 / 15 (y +-38.5), which
pin the cage's nose to the core through the mount (first mode 108 Hz F with a plain 15 x 14 bar; the topology does better).

One PETG print. Drawn (must-haves only): the 4 board bosses (M2.5 heat-set, 28 x 28) with their Ø12 pads, the hood ring
(4 ahead of the lens), the 2 slit-fork snap clips (3.2 seat, 2.0 prongs, 0.8 slits), the foot (contact plate on the plate
top, 2 slotted bolt pads). Everything else is generated (nox/topo, organic finish).

Keep-outs: the camera and its board back, the cable channel, the field cone (aperture r 6 E, + 2 deg), every cage member and
node swept back along -x by the push-on stroke (the mount slides on from the front: whatever passes a cage cell must fit it
the whole way), the props' swept disks as vertical prisms (no material in the prop wash), the plate and everything under it,
the bolt heads and their key paths.

python3 nox/scout/v3/camera_mount_v2.py [out] [--h 1.0] [--gen 12] [--git] [--smoke]
"""
import os, sys, json, math, argparse
import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
LAY = os.environ.get("SCM_LAYOUT") or os.path.join(ROOT, "runs", "scout_layout", "v13")   # the layout run (cage, links, parts): v13 (PR 169), nose as v12
if not os.path.isabs(LAY): LAY = os.path.join(ROOT, LAY)
os.environ.setdefault("SCOUT_FRAME_RUN", LAY)                   # cage_frame.py (the frame models) on the same cage
CG = json.load(open(os.path.join(LAY, "cage_shell.json"))); RES = json.load(open(os.path.join(LAY, "result.json")))
N = np.array(CG["nodes"], float); NR = len(CG["open"][0]["ring"])
mf.set_circular_segments(40)

# ---------------------------------------------------------------- inputs (dimensions.md System Scout v3, layout v12)
ROD_OD, BORE = 3.0, 3.2                                  # every Scout rod 3.0 solid (Spark 27 Oct); seat bore from the coupons (Spark 27 Oct), P
R_T, R_B = ROD_OD / 2, BORE / 2
LENS = np.array([149.0, 0.0, 42.0])                      # lens front, C (layout v12)
HFOV, VFOV, VIEW_MARGIN = 89.0, 73.0, 2.0                # landscape field, E; + 2 deg
APERTURE_R = 6.0                                         # front lens element as a disk, r (E; Q3: lens not measured)
CAM_BOX = ((135.0, 149.0), (-19.0, 19.0), (23.0, 61.0))  # camera 38 x 38 board + lens, 14 deep (P board, E depth)
CAM_G = 10.0                                             # camera, E (Q3)
CAM_CG = np.array([142.0, 0.0, 42.0])                    # E (box centre)
BOARD_HOLES = [(sy * 14.0, 42.0 + sz * 14.0) for sy in (1, -1) for sz in (1, -1)]   # M2.5 on 28 x 28 (y, z), P
INSERT_HOLE = 3.2                                        # M2.5 heat-set insert hole (E, dimensions.md M)
PLATE_TOP, PLATE_BOT = 22.0, 18.5                        # Bee35 plate (P / listing)
FOOT_BOLTS = [(30.0, 13.0), (30.0, -13.0)]               # the belly mount's front plate M3 (inserts below the plate), C (runs/scout_belly_mount/v1)
MOTORS = [(sx * 54.1, sy * 54.1) for sx in (1, -1) for sy in (1, -1)]   # P (Spark 12 Oct)
PROP_R = 44.5                                            # 3.5" prop swept radius, C (layout parts)
CLIP_ROD = (14, 15)                                      # rod 14-15 (124 long, along y at x 122.6, z 11.5)

# ---------------------------------------------------------------- design numbers (C unless tagged)
X_BOSS, BOSS_L, BOSS_D, PAD_D, PAD_T = 134.0, 6.5, 9.2, 12.0, 2.0     # seat face (bought 1.0 washer: board at 135), boss wall 3 round Ø3.2, Ø12 pad 2.0 web
HOOD_X = (148.0, 153.0)                                  # hood ring 5 deep: front 4.0 ahead of the lens front (gate 3, DEFAULT); deep and thick enough to stay in the eroded body
HOOD_OUT, HOOD_IN = (24.0, 15.5, 68.5), (19.5, 20.0, 64.0)   # (y half, z0, z1) outer / inner (camera box + 0.5): wall 4.5 (run 1: 3.0 eroded away at 2.0)
CLIP_Y = None                                            # clip centres on rod 14-15: set below from node 14's saved socket tip (NODE_CLR + 0.5 off it)
CLIP_L, T_P, SLIT, SNAP, FLARE, GRIP = 8.0, 2.0, 0.8, 0.25, 30.0, 0.05   # length along the rod, prong, slit, snap a side, lead-in deg, grip rib (radial, E)
U_ROOT, U_BLOCK = 8.2, 12.0                              # prong root (free length 9.2 to the lip) and the root block's back, local u (+x)
RIB_ANG = (0.0, 95.0, -95.0)                             # grip ribs, deg from the closed back (v1)
FOOT_Z = (PLATE_TOP, PLATE_TOP + 3.0)                    # 3.0 tall: rod 14-16's sweep passes 3.4 over the pads' outer top edge                    # foot contact plate on the plate top
PAD = (15.0, 12.0, 3.0)                                  # bolt pad x, y, z (slot Ø3.4 +-1.5 in x: the clips set x), >= Ø12 web >= 2
SLOT, SLOT_HALF, WASHER_D = 3.4, 1.5, 9.0                # M3 clearance, slot travel, washer seat + clearance (key path Ø9 up)
FOOT_X = (22.5, 51.5)                                    # foot plate x (plate front edge 51.9 at |y| <= 20, M)
ENV_Y = 26.0                                             # design space half width (hood 22.5, the lane's width at the plate; the clip arms below get their own)
NODE_CLR = 2.0                                           # clearance to the saved nodes v2 (parent 28 Oct: >= 2, the v3 nodes on v13 may grow)
STROKE, SWEEP_CLR = 110.0, 1.5                          # push-on stroke along -x (the foot starts in front of the cage face); clearance to swept members
WASH_MARGIN = 1.0                                        # vertical prism of every swept disk, r 44.5 + 1.0
HIT, CRASH, FLIGHT, G = 250.0, 35.0, 4.0, 9.81          # N frontal hit on the hood (35 g of a 735 g drone), g, g (dimensions.md L)
CLIP_BEAR = 40.0                                         # N, a clip's share of the frontal hit (frame model concept: 31 N at 250 N, F on E)
CAM_COUPLE = 10.0                                        # N a seat, the rotation stiffness cases (vibration class)
F_POP_DESIGN = 2.0                                       # the clip arms carry 2 x the pop force (the clips are the fuse)


# ---------------------------------------------------------------- geometry helpers
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def ball(c, r):
    return mf.Manifold.sphere(r, 24).translate(list(map(float, c)))


def cyl_x(y, z, r, x0, x1):
    return mf.Manifold.cylinder(x1 - x0, r, r, 32).transform([[0, 0, 1, x0], [0, 1, 0, y], [-1, 0, 0, z]])


def cyl_z(x, y, r, z0, z1):
    return mf.Manifold.cylinder(z1 - z0, r, r, 32).translate([x, y, z0])


_NS = os.path.join(ROOT, "runs", "scout_camera_mount", "v2", "node_sweeps.json")   # the saved nodes v1 (PR 166) as sweeps: camera_mount_v2_nodes.py
NODE_SWEEPS = json.load(open(_NS)) if os.path.exists(_NS) else None


def rod_ends(name, a, b):
    """a rod's true ends (socket floors, eccentric sockets included) when the saved nodes know it (matched by geometry: both
    floors within 4 of the centre line a-b, so a renumbered layout still finds it), else the node centres"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if NODE_SWEEPS:
        d = b - a; L = float(np.linalg.norm(d)); u = d / L
        for r in NODE_SWEEPS["rods"].values():
            fa, fb = np.array(r["a"], float), np.array(r["b"], float)
            if np.linalg.norm(fa - a) > np.linalg.norm(fb - a): fa, fb = fb, fa
            ok = True
            for f in (fa, fb):
                t = float((f - a) @ u); off = float(np.linalg.norm(f - a - t * u))
                ok &= (-2.0 < t < L + 2.0) and off < 4.0
            if ok and float((fb - fa) @ u) > 0.5 * L: return fa, fb
    return a, b


def node_sweep(i):
    """the saved node at layout node i: (y-z outlines moved to the layout node, x range), or None (no saved node within 6 mm)"""
    if not NODE_SWEEPS: return None
    for nd in NODE_SWEEPS["nodes"].values():
        dl = N[i] - np.asarray(nd["centre"], float)
        if np.linalg.norm(dl) < 6.0:
            return [np.asarray(ol) + dl[1:] for ol in nd["yz"]], (nd["x"][0] + dl[0], nd["x"][1] + dl[0])
    return None


def clip_y():
    """clip centre: the clip's far end NODE_CLR + 0.5 inside the tip of node 14's rod 14-15 socket (lowest y of its outline at the rod's height)"""
    s = node_sweep(CLIP_ROD[0])
    if s is None: return 38.5
    z0 = N[CLIP_ROD[0]][2]; P = np.vstack(s[0]); P = P[np.abs(P[:, 1] - z0) < 6.0]
    return round(float(P[:, 0].min()) - NODE_CLR - 0.5 - CLIP_L / 2, 1)


CLIP_Y = clip_y()


def prism_yz(cs, x0, x1):
    """CrossSection in (y, z) extruded along x from x0 to x1"""
    return cs.extrude(x1 - x0).translate([0, 0, x0]).transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])


def prism_xz(cs, y0, y1):
    """CrossSection in (x, z) extruded along y from y0 to y1"""
    m = cs.extrude(y1 - y0).translate([0, 0, -y1])                 # c in -y1..-y0
    return m.transform([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0]])  # (a, b, c) -> (a, -c, b)


def poly(pts):
    pts = [tuple(map(float, p)) for p in pts]
    if sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])) < 0: pts = pts[::-1]
    return mf.CrossSection([pts])


def swept(points, r, S=STROKE):
    """the convex hull of balls of radius r at the points and at the points moved -S along x (a member and its push-on sweep)"""
    return mf.Manifold.batch_hull([ball(p, r) for p in points] + [ball(np.asarray(p) - [S, 0, 0], r) for p in points])


def lane_half(x, r=PROP_R + WASH_MARGIN):
    """half width at x of the clear lane between the two front swept disks (props at (54.1, +-54.1))"""
    d = r * r - (x - 54.1) ** 2
    return 54.1 - math.sqrt(d) if d > 0 else 99.0


def plate_outline():
    import trimesh
    m = trimesh.load(os.path.join(LAY, "parts", "frame_plate__bee35_carbon.stl"), force="mesh")
    s = m.section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1]); p2, T = s.to_2D()
    P = max(p2.polygons_full, key=lambda q: q.area); xy = np.asarray(P.exterior.coords)[:-1]
    xy = (T[:2, :2] @ xy.T).T + T[:2, 3]                            # back to the drone frame (T is a pure translation here)
    return xy


# ---------------------------------------------------------------- the slit-fork snap clip (local u = x, v = z about the rod)
def clip_profile():
    """seat tongue (back half of the 3.2 bore), two 2.0 prongs from a root block at u = U_ROOT past the rod to lips that leave
    a mouth of 2(R_T - SNAP), 30 deg lead-in, 0.8 slits between tongue and prongs; grip ribs (v1) at 0 and +-95 deg"""
    h = R_T - SNAP; ul = -math.sqrt(R_B ** 2 - h ** 2); vo = R_B + SLIT + T_P
    tb = math.tan(math.radians(FLARE)); D = 3.0
    outer = poly([(U_BLOCK, -vo), (U_BLOCK, vo), (ul - 0.6, vo), (ul - D, h + D * tb + 0.4), (ul - D, -h - D * tb - 0.4), (ul - 0.6, -vo)])
    bore = mf.CrossSection.circle(R_B, 64)
    mouth = poly([(0.0, h), (ul, h), (ul - D - 1, h + (D + 1) * tb), (ul - D - 1, -h - (D + 1) * tb), (ul, -h), (0.0, -h)])
    slits = poly([(0.0, R_B), (U_ROOT, R_B), (U_ROOT, R_B + SLIT), (0.0, R_B + SLIT)]) + poly([(0.0, -R_B), (U_ROOT, -R_B), (U_ROOT, -R_B - SLIT), (0.0, -R_B - SLIT)])
    c = outer - bore - mouth - slits
    for a in RIB_ANG:
        r0 = R_T - GRIP
        rib = mf.CrossSection.square([R_B + 0.3 - r0, 1.2]).translate([r0, -0.6]).rotate(a)
        c = c + (rib - mouth)
    return c, h, ul, vo


def clip_bodies(y_c):
    """(drawn clip, its cuts, the bore ring for loads, the root block region) at y_c (axis along y)"""
    c, h, ul, vo = clip_profile(); x0, z0 = N[CLIP_ROD[0]][0], N[CLIP_ROD[0]][2]
    y0, y1 = y_c - CLIP_L / 2, y_c + CLIP_L / 2
    drawn = prism_xz(c.translate([x0, z0]), y0, y1)
    # cuts: bore + mouth + lead-in + slits (through, 1 past each end), and a finger zone round the prongs' outsides and in front
    # of the clip where nothing generated may touch them (they must flex); only the root block (u >= U_ROOT) takes material
    tb = math.tan(math.radians(FLARE))
    hole = mf.CrossSection.circle(R_B, 64) + poly([(0.0, h), (ul, h), (ul - 6, h + 6 * tb), (ul - 6, -h - 6 * tb), (ul, -h), (0.0, -h)])
    slits = poly([(0.0, R_B), (U_ROOT, R_B), (U_ROOT, R_B + SLIT), (0.0, R_B + SLIT)]) + poly([(0.0, -R_B), (U_ROOT, -R_B), (U_ROOT, -R_B - SLIT), (0.0, -R_B - SLIT)])
    zone = poly([(U_ROOT - 0.3, -vo - 3.0), (U_ROOT - 0.3, vo + 3.0), (ul - 7.0, vo + 3.0), (ul - 7.0, -vo - 3.0)]) - c
    cut = prism_xz((hole + slits).translate([x0, z0]), y0 - 1, y1 + 1) + prism_xz(zone.translate([x0, z0]), y0 - 0.01, y1 + 0.01)
    ends = poly([(U_ROOT - 0.3, -vo - 3.0), (U_ROOT - 0.3, vo + 3.0), (ul - 7.0, vo + 3.0), (ul - 7.0, -vo - 3.0)]).translate([x0, z0])
    cut = cut + prism_xz(ends, y1, y1 + 3.0) + prism_xz(ends, y0 - 3.0, y0)       # the prongs' end faces stay free too
    ring = prism_xz((mf.CrossSection.circle(R_B + 1.2, 48) - mf.CrossSection.circle(R_B, 48)).translate([x0, z0]), y0, y1)
    block = prism_xz(poly([(U_ROOT, -vo), (U_BLOCK, -vo), (U_BLOCK, vo), (U_ROOT, vo)]).translate([x0, z0]), y0, y1)
    seat = prism_xz(((mf.CrossSection.circle(R_B + 1.0, 48) - mf.CrossSection.circle(R_B, 48)) ^ mf.CrossSection.square([R_B + 2, 2 * R_B + 4]).translate([0.2, -R_B - 2])).translate([x0, z0]), y0, y1)
    return drawn, cut, ring, block, (ul, vo), seat


# ---------------------------------------------------------------- all bodies
def bodies():
    drawn = mf.Manifold(); cuts = mf.Manifold(); R = {}
    # --- camera bosses and pads (the camera board seats on 4 bought 1.0 washers at x 134..135)
    bosses = []
    for (y, z) in BOARD_HOLES:
        b = cyl_x(y, z, PAD_D / 2, X_BOSS - BOSS_L, X_BOSS)          # Ø12 the full length: wall 4.4 round the insert (run 1: the Ø9.2 boss eroded away at 2.0)
        drawn += b; bosses.append(b)
        cuts += cyl_x(y, z, INSERT_HOLE / 2, X_BOSS - BOSS_L - 1, X_BOSS + 1)
    R["seats"] = [cyl_x(y, z, PAD_D / 2, X_BOSS - 1.0, X_BOSS) - cyl_x(y, z, INSERT_HOLE / 2 + 0.3, X_BOSS - 2, X_BOSS + 1) for (y, z) in BOARD_HOLES]
    R["bosses"] = bosses
    # --- hood ring
    yo, zo0, zo1 = HOOD_OUT; yi, zi0, zi1 = HOOD_IN
    hood = box(HOOD_X[0], HOOD_X[1], -yo, yo, zo0, zo1) - box(HOOD_X[0] - 1, HOOD_X[1] + 1, -yi, yi, zi0, zi1)
    drawn += hood; R["hood"] = hood
    R["hood_front"] = hood ^ box(HOOD_X[1] - 1.0, HOOD_X[1] + 1, -yo - 1, yo + 1, zo0 - 1, zo1 + 1)
    R["hood_corners"] = {f"{'+' if sy > 0 else '-'}y {'top' if zz > 40 else 'bottom'}": hood ^ ball((HOOD_X[1], sy * (yo - 1.5), zz), 5.0)
                         for sy in (1, -1) for zz in (zo1 - 1.5, zo0 + 1.5)}
    # --- the two clips on rod 14-15
    R["clip_rings"], R["clip_blocks"] = [], []
    for s in (1, -1):
        d, c, ring, blk, (ul, vo), seat = clip_bodies(s * CLIP_Y)
        drawn += d; cuts += c; R["clip_rings"].append(ring); R["clip_blocks"].append(blk); R.setdefault("clip_seats", []).append(seat)
    # --- foot: contact plate on the plate top (lane-limited) + 2 slotted bolt pads
    xs = np.linspace(FOOT_X[0], FOOT_X[1], 30)
    up = [(x, min(lane_half(x) - 0.5, 20.0)) for x in xs]
    foot = poly(up + [(x, -y) for x, y in up[::-1]]).extrude(FOOT_Z[1] - FOOT_Z[0]).translate([0, 0, FOOT_Z[0]])
    pads = mf.Manifold()
    for (x, y) in FOOT_BOLTS:
        pads += box(x - PAD[0] / 2, x + PAD[0] / 2, y - PAD[1] / 2, y + PAD[1] / 2, PLATE_TOP, PLATE_TOP + PAD[2])
    drawn += foot + pads; R["pads"] = pads; R["foot"] = foot
    R["foot_bottom"] = (foot + pads) ^ box(0, 60, -30, 30, PLATE_TOP, PLATE_TOP + 1.0)
    for (x, y) in FOOT_BOLTS:                                          # slot through, washer + head + key path up
        sl = mf.Manifold.hull(cyl_z(x - SLOT_HALF, y, SLOT / 2, PLATE_TOP - 1, PLATE_TOP + PAD[2] + 1) + cyl_z(x + SLOT_HALF, y, SLOT / 2, PLATE_TOP - 1, PLATE_TOP + PAD[2] + 1))
        key = mf.Manifold.hull(cyl_z(x - SLOT_HALF, y, WASHER_D / 2, PLATE_TOP + PAD[2], 90) + cyl_z(x + SLOT_HALF, y, WASHER_D / 2, PLATE_TOP + PAD[2], 90))
        cuts += sl + key
    # --- keep-outs: camera, board back, cable channel, field cone
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = CAM_BOX
    cuts += box(X_BOSS, cx1 + 0.5, cy0 - 0.5, cy1 + 0.5, cz0 - 0.5, cz1 + 0.5)          # camera + the 4 washers (x 134..135)
    back = box(X_BOSS - 3.0, cx0, cy0, cy1, cz0, cz1)
    for (y, z) in BOARD_HOLES: back -= cyl_x(y, z, PAD_D / 2 + 0.01, X_BOSS - 4, cx0 + 0.1)
    cuts += back                                                       # board back parts up to 3 tall (v1: Q3)
    cuts += box(123.5, cx0, -7.0, 7.0, 33.0, 51.0)                    # cable channel from the board back (connector Q3)
    tv = lambda a: math.tan(math.radians(a / 2 + VIEW_MARGIN)); Lc = 80.0
    far = [(LENS[0] + Lc, sy * (APERTURE_R + Lc * tv(HFOV)), LENS[2] + sz * (APERTURE_R + Lc * tv(VFOV))) for sy in (1, -1) for sz in (1, -1)]
    near = [(LENS[0] - 0.5, sy * APERTURE_R, LENS[2] + sz * APERTURE_R) for sy in (1, -1) for sz in (1, -1)]
    cuts += mf.Manifold.hull_points(far + near)
    # --- cage: every member and node with its push-on sweep (clips excepted on rod 14-15)
    rods = [(a, b) for a, b in CG["rods"]]
    for a, b in rods:
        if {a, b} == set(CLIP_ROD):
            continue
        cuts += swept(list(rod_ends(f"rod {a}-{b}", N[a], N[b])), R_T + SWEEP_CLR)
    for k, (p, q) in RES["links"].items():
        cuts += swept(list(rod_ends(k, p, q)), CG.get("link_r_mm", 1.5) + SWEEP_CLR)
    for i, p in enumerate(N):
        ns = node_sweep(i)
        if ns:                                                         # the saved node: its y-z outline, from its back - stroke to its front
            for ol in ns[0]:
                cs = poly(ol).offset(NODE_CLR, mf.JoinType.Round)
                cuts += prism_yz(cs, ns[1][0] - STROKE, ns[1][1] + NODE_CLR)
        else:
            cuts += swept([p], (10.0 if i < NR else CG["node_r_mm"]) + NODE_CLR)
    # rod 14-15: swept everywhere but the clip lengths (there the clip's own mouth / lead-in cut is the path)
    a, b = rod_ends("rod 14-15", N[CLIP_ROD[0]], N[CLIP_ROD[1]])
    sw = swept([a, b], R_T + SWEEP_CLR)
    for s in (1, -1):
        y0, y1 = sorted((s * (CLIP_Y - CLIP_L / 2), s * (CLIP_Y + CLIP_L / 2)))
        sw -= box(a[0] - STROKE - 10, a[0] + 20, y0, y1, a[2] - 10, a[2] + 10)
    cuts += sw
    # --- prop wash: vertical prisms of the swept disks; the plate and everything under it
    wash = mf.Manifold()
    for (mx, my) in MOTORS:
        wash += cyl_z(mx, my, PROP_R + WASH_MARGIN, -80, 120)
    cuts += wash - pads                                                # the pads reach 2.1 into the disk prism, flat on the plate (E)
    pl = plate_outline()
    cuts += poly([tuple(p) for p in pl]).extrude(PLATE_TOP + 80).translate([0, 0, -80])
    cuts += box(-120, 112.0, -80, 80, -80, 9.0)                        # receiver, lid, XT60, belly mount, cartridge (all below the rod 14-15 sweep)
    cuts += box(-120, 112.0, -80, 80, 31.0, 120)                        # behind the face only the bottom-front cell reaches the foot (the upper cell is a dead end)
    # --- envelope: the print box round the camera, the cell and the foot
    env = box(FOOT_X[0] - 0.5, HOOD_X[1], -ENV_Y, ENV_Y, 5.5, HOOD_OUT[2]) + \
        box(112.0, HOOD_X[1], -CLIP_Y - CLIP_L / 2 - 1, CLIP_Y + CLIP_L / 2 + 1, 5.5, 34.0)   # the clips' arms: low, in front of the cage face
    return dict(drawn=drawn, cuts=cuts, env=env, R=R)


def couple(F, at, pts):
    """least-norm forces at pts that carry force F acting at `at` (its moment included); nox/overseer/head_mount.py"""
    c = np.mean(pts, 0); Mm = np.cross(at - c, F); rows = []
    for p in pts:
        S = np.array([[0, -(p - c)[2], (p - c)[1]], [(p - c)[2], 0, -(p - c)[0]], [-(p - c)[1], (p - c)[0], 0]])
        rows.append(np.vstack([np.eye(3), S]))
    x = np.linalg.pinv(np.hstack(rows)) @ np.concatenate([F, Mm])
    return [x[3 * k:3 * k + 3] for k in range(len(pts))]


def fork_snap(t_p=T_P, snap=SNAP, L=CLIP_L, u_root=U_ROOT, mu=0.25, E=2000.0):
    """slit-fork clip, per clip (E): two prongs, cantilevers from u_root to the lip; push-on and pop by a contact sweep of the
    rod past the lip profile (v1's method), root stress at full opening"""
    h = R_T - snap; ul = -math.sqrt(R_B ** 2 - h ** 2); lc = u_root - ul
    I = L * t_p ** 3 / 12; k = 3 * E * I / lc ** 3
    s = np.linspace(0, 4, 300); prof = np.column_stack([ul - s * math.cos(math.radians(FLARE)), h + s * math.sin(math.radians(FLARE))])
    ang = np.linspace(math.pi / 2, math.pi - math.asin(h / R_B), 120); prof = np.vstack([prof, np.column_stack([R_B * np.cos(ang), R_B * np.sin(ang)])])
    def sweep(cus):
        Fm = dm = 0.0
        for cu in cus:
            dx = prof[:, 0] - cu; ok = np.abs(dx) < R_T
            if not ok.any(): continue
            need = np.sqrt(R_T ** 2 - dx[ok] ** 2) - prof[ok, 1]; j = int(np.argmax(need)); d = float(need[j])
            if d <= 0: continue
            p = prof[ok][j] + [0, d]; n = (p - [cu, 0]) / R_T; nu, nv = abs(n[0]), n[1]; den = nv - mu * nu
            F = 2 * k * d * ((nu + mu * nv) / den if den > 1e-6 else 1e9); Fm = max(Fm, F); dm = max(dm, d)
        return Fm, dm
    F_in, d_in = sweep(np.linspace(-6.0, -0.01, 800)); F_pop, d_pop = sweep(np.linspace(-0.01, -6.0, 800))
    d = max(d_in, d_pop); sig = 3 * E * t_p * d / (2 * lc ** 2)
    return dict(t_p=t_p, snap=snap, L=L, free_length=round(lc, 2), k_prong_N_mm=round(k, 1), install_N=round(F_in, 1), pop_N=round(F_pop, 1),
                open_mm=round(d, 3), prong_sigma_MPa=round(sig, 1), mouth_mm=round(2 * h, 2))


def setup(h):
    from topo import Part
    b = bodies(); R = b["R"]
    p = Part("scout_camera_mount", h, b["drawn"], b["cuts"], b["env"], build="+y", material="PETG")
    p.support("foot", R["pads"])                                     # the 2 bolt pads: clamped by the belly mount's front plate bolts
    nd = p.nodes_of(p.elems(R["foot_bottom"], p.solid | p.design, what="foot contact"))
    p.supports["foot"][nd, 2] = True                                 # the foot plate bears on the plate top (preloaded contact, z only)
    # a hit on the hood: the foot plus the clips' closed backs bearing on rod 14-15 at node 14 / 15's socket tips (x only: the rod
    # goes straight into saddle tie 4.1, 3.0 x 88, Euler ~1 kN); run 1 held it at the foot alone and the tie read index 18.6
    p.support("hood", R["pads"]); p.supports["hood"][nd, 2] = True
    for seat in R["clip_seats"]:
        ns = p.nodes_of(p.elems(seat, p.solid | p.design, what="clip seat")); p.supports["hood"][ns, 0] = True
    seats = R["seats"]; at = CAM_CG; pts = [np.array([X_BOSS, y, z]) for (y, z) in BOARD_HOLES]
    dirs = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}
    for nm, d in dirs.items():                                       # camera 35 g at its CG through the 4 seats + the mount's own mass
        d = np.array(d, float); F = CAM_G / 1000 * G * CRASH * d
        p.load_multi(f"crash 35 g {nm}", list(zip(seats, couple(F, at, pts))), cls="misuse", support="foot", accel=CRASH * d)
    # the camera's rotations on the mount (pitch, yaw, roll couples on the 4 seats): the modes rock the camera; run 1 left the
    # upper seats on thin material (0.78 mm/N at the seats against 0.05 mid-tie, 74 Hz F)
    s_ = seats; C_ = CAM_COUPLE
    for nm, parts in (("camera pitch", [(s_[0], (C_, 0, 0)), (s_[2], (C_, 0, 0)), (s_[1], (-C_, 0, 0)), (s_[3], (-C_, 0, 0))]),
                      ("camera yaw", [(s_[0], (C_, 0, 0)), (s_[1], (C_, 0, 0)), (s_[2], (-C_, 0, 0)), (s_[3], (-C_, 0, 0))]),
                      ("camera roll", [(s_[0], (0, 0, C_)), (s_[1], (0, 0, C_)), (s_[2], (0, 0, -C_)), (s_[3], (0, 0, -C_))])):
        p.load_multi(nm, parts, cls="vibration", support="foot")
    fl = np.array([0, 0, -FLIGHT]); p.load_multi("flight 4 g", list(zip(seats, couple(CAM_G / 1000 * G * fl, at, pts))), cls="flight", support="foot", accel=fl)
    p.load("hood hit centre -x", R["hood_front"], (-HIT, 0, 0), cls="misuse", support="hood")
    for nm, reg in R["hood_corners"].items():
        p.load(f"hood hit {nm} corner -x", reg, (-HIT, 0, 0), cls="misuse", support="hood")
    sp = fork_snap(); fp = F_POP_DESIGN * sp["pop_N"]
    for s, ring in zip(("+y", "-y"), R["clip_rings"]):               # the rod on the clip: bearing on the closed back, pull, up, down
        p.load(f"clip {s} bear +x", ring, (CLIP_BEAR, 0, 0), cls="misuse", support="foot")
        p.load(f"clip {s} pull -x", ring, (-fp, 0, 0), cls="misuse", support="foot")
        p.load(f"clip {s} up +z", ring, (0, 0, fp), cls="misuse", support="foot")
        p.load(f"clip {s} down -z", ring, (0, 0, -fp), cls="misuse", support="foot")
    return p


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("out", nargs="?", default=os.path.join(ROOT, "runs", "scout_camera_mount", "v2"))
    ap.add_argument("--h", type=float, default=1.0); ap.add_argument("--gen", type=float, default=16.0); ap.add_argument("--git", action="store_true")
    ap.add_argument("--smoke", action="store_true", help="build the Part once and report, no optimisation")
    ap.add_argument("--no-coarse", action="store_true", help="fine stage only (h 1.5: the 2h coarse grid loses the loads' regions)")
    ap.add_argument("--backend", default="auto", help="topo solver: auto | direct | mg (mg on a shared box: the direct factor is sized on total RAM)")
    a = ap.parse_args()
    if a.smoke:
        print(json.dumps(fork_snap()))
        p = setup(a.h); print("grid", p.shape, "design", int(p.design.sum()), "drawn", int(p.solid.sum()), "cases", len(p.cases)); sys.exit(0)
    from topo import Settings, optimise
    optimise(setup, h=a.h, gen_cm3=a.gen, out_dir=a.out, name="scout_camera_mount", settings=Settings(backend=a.backend), coarse=not a.no_coarse, git=a.git,
             extra=dict(snap=fork_snap(), clip_y=CLIP_Y, stroke=STROKE))
