"""Scout v3 camera mount (piece of System Scout v3, dimensions.md "System Scout v3": Camera (one), Strength, Rods).

One PETG print: the cradle round the DECXIN 2633V2 (board on 4 M2.5 heat-set bosses), its lens hood (Spark 27 Oct: "crash
protection for the lense sticking out ... cool and spacex like cyberware, and construction"), 2 top arms + braces and 2
bottom knees, and 4 snap C-clips that push onto the two 6/4 carbon tubes of the sized cage (rod 7-8 top, rod 14-15 bottom)
after the cage is built, no rod through it. Plus one small print, the stop collar (x2), CA'd on rod 14-15 outboard of the
bottom clips (the y end stop).

Everything is drawn: hard planar facets and chamfers (octagon-chamfered box, octagon bars, octagon clips), chevron truss
windows in the four faces and an X frame at the back (where the load goes: bosses -> X -> corners -> arms -> clips).
The arms and braces are sized on the frame model below (mode, crash, frontal hit on the hood, breaking order).

Checks in this script (numbers to runs/scout_camera_mount/v1/result.json):
  view    nothing of the mount in the camera's field (the layout's in_cam_view point test, +2 deg) and an aperture test
          (the front lens element as a disk of r APERTURE_R, E); the hood's lead ahead of the lens front
  fit     clearance to every cage rod (sized: 6/4 on the frame model's 39, 3.0 on the rest) and node, to the props and
          every v12 part; the install stroke (the mount pushed on along -x from 40 mm out): no rod in the way but the
          two the clips snap onto, and those only by the snap interference
  snap    install and pop force per clip, finger stress, grip (slide) per clip: curved-beam fingers (Castigliano), a
          contact sweep of the tube past the lip profile, friction mu (E)
  frame   the v12 cage with the sized tubes + this mount (arms, braces, knees, clip bodies as PETG beams, the cradle and
          camera one rigid body with their real inertia): lowest modes, frontal hit 250 N on the hood, 35 g crash, 4 g
          flight, and the first thing to let go per direction (the fuse order)

python3 nox/scout/v3/camera_mount.py [out_dir]       (default runs/scout_camera_mount/v1)
env: SCM_ROD=6 (6/4 tube, bore 6.2; 3 -> 3.0 rod, bore 3.2), SCM_TF finger wall, SCM_SNAP snap per side, SCM_ARM arm flats
"""
import os, sys, json, math
import numpy as np
import trimesh
import manifold3d as m3

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from frame_model import Frame, tube

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_camera_mount", "v1")
LAY = os.path.join(ROOT, "runs", "scout_layout", "v12")
SIZED = os.path.join(ROOT, "runs", "scout_frame", "v2", "alt_sized", "up6.json")
m3.set_circular_segments(48)

# ---------------------------------------------------------------- inputs (dimensions.md System Scout v3, layout v12)
ROD_OD = float(os.environ.get("SCM_ROD", 6.0))          # the tube the clips sit on: 6/4 carbon (DEFAULT 27 Oct, frame model sizing), C
BORE = ROD_OD + 0.2                                     # socket bore from the coupon pick (6.2 / 3.2, Spark 27 Oct), P
CLIP_TOP = (110.5, 18.0, 61.1)                          # on rod 7-8 (193 long), +y; mirrored (layout v12), C
CLIP_BOT = (122.6, 18.0, 11.5)                          # on rod 14-15 (124 long), +y; mirrored, C
LENS = np.array([149.0, 0.0, 42.0])                     # lens front, C (layout v12)
HFOV, VFOV, VIEW_MARGIN = 89.0, 73.0, 2.0               # landscape field, E; +2 deg (the layout's in_cam_view)
APERTURE_R = 6.0                                        # front lens element as a disk, r (E; Q3: lens not measured)
CAM_BOX = ((135.0, 149.0), (-19.0, 19.0), (23.0, 61.0)) # camera envelope 38 x 38 board + lens, 14 deep (P board, E depth)
CAM_G = 10.0                                            # camera mass, E (layout; Q3 not measured)
BOARD_HOLES = [(sy * 14.0, 42.0 + sz * 14.0) for sy in (1, -1) for sz in (1, -1)]   # M2.5 on 28 x 28, P
INSERT_HOLE, INSERT_L = 3.2, 5.7                        # M2.5 heat-set insert hole and length, E (dimensions.md M)
WASHER = 1.0                                            # bought M2.5 nylon washer under the board, E (no printed TPU: nothing printed under 2.0)

# ---------------------------------------------------------------- design numbers (C unless tagged)
X_BACK, X_FRAME, X_BOSS, X_HOOD = 128.0, 131.0, 134.0, 153.0   # back face, back frame front, boss face (+ washer = board 135), hood front
HOOD_BACK = 147.0                                       # solid hood ring from here to X_HOOD
Y_OUT, Z_OUT = 22.5, (17.0, 67.0)                       # cradle outline (layout v12 cradle: y +-22.5, z 17..67), C
WALL = 3.0                                              # box walls (lips 6 over / under the camera body incl. 3 air), C
CHAMF = 3.0                                             # long-edge chamfer of the box
HOOD_CH, HOOD_FLARE = 1.2, 0.8                          # hood front outer chamfer, inner flare (a 1.0 land at the front)
LONG = 4.0                                              # longeron strip left at each face edge
RIB = 3.5                                               # truss rib width
XBAR, BORDER = 5.0, 4.0                                 # back frame X bars, border
BOSS_D = INSERT_HOLE + 2 * 3.0                          # boss body: wall 3 round the insert hole (attachments rule)
PAD_D, PAD_T = 12.0, 2.0                                # board pad flange at the seat face: Ø12 across flats, 2.0 web (pad rule)
ARM = float(os.environ.get("SCM_ARM", 7.0))             # arm octagon across flats (>= 6 x 6 post rule)
BRACE = 6.0                                             # (v1 draft: brace octagon; now the truss chords)
WEB, FLANGE, FT = 3.0, 8.0, 3.0                         # truss web thickness, chord flange width and thickness
CLIP_LS = (8.0, 9.0, 10.0, 12.0)                        # clip lengths scanned (layout drew 12)
T_F = float(os.environ.get("SCM_TF", 0.0))              # finger wall (0: picked by the snap scan)
SNAP = float(os.environ.get("SCM_SNAP", 0.0))           # snap interference per side at the mouth (0: picked)
FLARE = 30.0                                            # mouth lead-in face, deg from the push axis
GRIP = 0.05                                             # grip rib interference on the tube, radial (E, set on a coupon)
RIB_ANG = (0.0, 95.0, -95.0)                            # grip ribs, deg from the closed side
COLLAR_L, COLLAR_GAP = 5.0, 0.3                         # stop collar length, gap to the clip end
E_PETG, SY_PETG, SA_PETG, MU = 2000.0, 50.0, 25.0, 0.25  # MPa (E), yield (E), allowable (docs/generative.md, E), PETG on carbon (E)
RHO = 1.27e-3                                           # g/mm3 PETG (E)
INSTALL_MAX, SIGMA_SNAP_MAX, SNAP_MIN = 25.0, 30.0, 0.2  # N per clip (E target), MPa at full opening (0.6 yield, E), mm (print tolerance floor, E)
G, CRASH, FLIGHT, HIT = 9.81, 35.0, 4.0, 250.0          # g, g, N frontal hit (the 35 g crash of a 730 g drone through the hood), dimensions.md L

R_B = BORE / 2; R_T = ROD_OD / 2
OCT = math.tan(math.radians(22.5))


# ---------------------------------------------------------------- geometry helpers (manifold3d)
def tm(man):
    me = man.to_mesh(); m = trimesh.Trimesh(vertices=np.asarray(me.vert_properties)[:, :3], faces=np.asarray(me.tri_verts), process=True)
    if not m.is_watertight: raise SystemExit("mesh not watertight after vertex merge: coincident faces in the construction")
    return m


def prism(poly, plane, c0, c1):
    """2D polygon (list of (a, b)) extruded c0..c1: plane 'yz' (a=y, b=z, along x), 'xz' (a=x, b=z, along y), 'xy' (along z)"""
    if not isinstance(poly, m3.CrossSection):
        pts = [tuple(map(float, p)) for p in poly]
        if sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])) < 0: pts = pts[::-1]   # counter-clockwise
        cs = m3.CrossSection([pts])
    else:
        cs = poly
    if plane == "xz":
        man = cs.extrude(c1 - c0).translate([0, 0, -c1])       # c in -c1..-c0, y = -c
        return man.transform([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0]])
    man = cs.extrude(c1 - c0).translate([0, 0, c0])
    if plane == "yz": return man.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
    return man


def chamfered_rect(a0, a1, b0, b1, c):
    return [(a0 + c, b0), (a1 - c, b0), (a1, b0 + c), (a1, b1 - c), (a1 - c, b1), (a0 + c, b1), (a0, b1 - c), (a0, b0 + c)]


def octagon(r_in, cx=0.0, cy=0.0):
    R = r_in / math.cos(math.radians(22.5))
    return [(cx + R * math.cos(math.radians(22.5 + 45 * k)), cy + R * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]


def bar(p, q, a):
    """octagon bar, a across flats, from p to q, one flat up"""
    p, q = np.asarray(p, float), np.asarray(q, float); d = q - p; L = float(np.linalg.norm(d)); ez = d / L
    up = np.array([0, 0, 1.0]) if abs(ez[2]) < 0.95 else np.array([1.0, 0, 0])
    ex = np.cross(up, ez); ex /= np.linalg.norm(ex); ey = np.cross(ez, ex)
    man = m3.CrossSection([octagon(a / 2)]).extrude(L)
    M = np.column_stack([ex, ey, ez, p])
    return man.transform(M.tolist())


def tri_windows(tris, grow, shrink):
    out = []
    for t in tris:
        cs = m3.CrossSection([[tuple(map(float, v)) for v in t]])
        cs = cs.offset(-shrink, m3.JoinType.Miter)
        if not cs.is_empty(): out.append(cs)
    return out


# ---------------------------------------------------------------- the clip profile (local u = x, v = z about the tube axis)
def clip_profile(t_f, snap, ribs=True):
    """octagon C round the tube, mouth to -u (rearward, into the cage: the mount is pushed on from the front), lead-in flare,
    grip ribs; returns a CrossSection in (u, v)"""
    r_in = R_B + t_f; h = R_T - snap; xl = math.sqrt(R_B ** 2 - h ** 2); D = 12.0; tb = math.tan(math.radians(90 - FLARE))
    outer = m3.CrossSection([octagon(r_in)])
    bore = m3.CrossSection.circle(R_B, 64)
    mouth = m3.CrossSection([[(0.0, h), (-xl, h), (-xl - D, h + D / tb), (-xl - D, -h - D / tb), (-xl, -h), (0.0, -h)]])
    c = outer - bore - mouth
    if ribs:
        for a in RIB_ANG:
            r0 = R_T - GRIP
            rib = m3.CrossSection.square([R_B + 0.4 - r0, 1.2]).translate([r0, -0.6]).rotate(a)
            c = c + (rib - mouth)
    return c


def clip_solid(center, t_f, snap, L):
    """the clip at a world centre, axis along y, length L"""
    cs = clip_profile(t_f, snap).translate([center[0], center[2]])
    return prism(cs, "xz", center[1] - L / 2, center[1] + L / 2)


# ---------------------------------------------------------------- snap mechanics (E: linear curved beam, Castigliano)
def snap_numbers(t_f, snap, L):
    """per clip: finger stiffness, install and pop force, finger stress at full opening, grip (slide) force"""
    r_in = R_B + t_f; h = R_T - snap
    th_t = math.pi - math.asin(h / R_B)                 # finger tip (the lip at the bore)
    th_c = math.radians(22.5)                          # finger root: the arm meets the clip on the octagon's back flat only (the neck)
    phi = np.linspace(th_c, th_t, 400)
    t = r_in / np.cos(((np.degrees(phi) + 22.5) % 45.0 - 22.5) * math.pi / 180) - R_B   # octagon wall
    R = R_B + t / 2; I = L * t ** 3 / 12
    comp_open = np.trapezoid((R * (np.cos(phi) - math.cos(th_t))) ** 2 * R / (E_PETG * I), phi)   # mm/N, tip force along v
    k = 1.0 / comp_open
    # lip profile (upper finger): the lip edge then the flare out to the outer octagon
    xl = math.sqrt(R_B ** 2 - h ** 2)
    s = np.linspace(0, 6, 300); prof = np.column_stack([-xl - s * math.cos(math.radians(FLARE)), h + s * math.sin(math.radians(FLARE))])
    ang = np.linspace(th_c, th_t, 120); prof = np.vstack([prof, np.column_stack([R_B * np.cos(ang), R_B * np.sin(ang)])])   # the bore inside the finger
    def sweep(cu_range):
        Fmax, dmax = 0.0, 0.0
        for cu in cu_range:
            dx = prof[:, 0] - cu; ok = np.abs(dx) < R_T
            if not ok.any(): continue
            need = np.sqrt(R_T ** 2 - dx[ok] ** 2) - prof[ok, 1]
            j = int(np.argmax(need)); d = float(need[j])
            if d <= 0: continue
            p = prof[ok][j] + [0, d]; n = (p - [cu, 0]) / R_T; nu, nv = abs(n[0]), n[1]
            den = nv - MU * nu
            F = 2 * k * d * ((nu + MU * nv) / den if den > 1e-6 else 1e9)
            Fmax = max(Fmax, F); dmax = max(dmax, d)
        return Fmax, dmax
    F_in, d_in = sweep(np.linspace(-9.0, -0.01, 900))
    F_pop, d_pop = sweep(np.linspace(-0.01, -9.0, 900))
    d_open = max(d_in, d_pop)
    sig = k * d_open * float(R[0]) * (math.cos(th_c) - math.cos(th_t)) * (t[0] / 2) / float(I[0])
    # grip: side ribs at +-95 deg pressed out by GRIP (radial), the closed-side rib reacts
    thr = math.radians(abs(RIB_ANG[1]))
    ph2 = phi[phi < thr]; t2 = t[:len(ph2)]; R2 = R_B + t2 / 2; I2 = L * t2 ** 3 / 12
    c_r = np.trapezoid(np.sin(thr - ph2) ** 2 * R2 ** 3 / (E_PETG * I2), ph2)
    Q = GRIP / c_r; Q0 = 2 * Q * abs(math.cos(thr))
    slide = MU * (2 * Q + Q0)
    return dict(t_f=t_f, snap=snap, L=L, k_finger_N_mm=round(k, 1), finger_root_deg=round(math.degrees(th_c), 1), finger_tip_deg=round(math.degrees(th_t), 1),
                install_N=round(F_in, 1), pop_N=round(F_pop, 1), open_mm=round(d_open, 2), finger_sigma_MPa=round(sig, 1),
                grip_rib_N=round(Q, 1), slide_N=round(slide, 1), mouth_mm=round(2 * h, 2))


def pick_snap():
    """fingers 2.0 and up (no flexure exception), snap >= SNAP_MIN; the strongest pop that keeps the push-on force <= 25 N
    a clip (4 clips snap at once: <= 100 N by two thumbs, E) and the finger <= 30 MPa at full opening"""
    rows = [snap_numbers(tf, sn, L) for tf in (2.0, 2.2, 2.4) for sn in (0.2, 0.25, 0.3) for L in CLIP_LS]
    ok = [r for r in rows if r["install_N"] <= INSTALL_MAX and r["finger_sigma_MPa"] <= SIGMA_SNAP_MAX and r["snap"] >= SNAP_MIN]
    best = max(ok, key=lambda r: (r["pop_N"], r["L"])) if ok else min(rows, key=lambda r: r["install_N"])
    return best, rows


# ---------------------------------------------------------------- the mount
def mirror_y(man):
    return man.mirror([0, 1, 0])


def build(t_f, snap, CLIP_L):
    z0, z1 = Z_OUT
    # box shell (octagon-chamfered tube), back frame to hood front
    out_sec = chamfered_rect(-Y_OUT, Y_OUT, z0, z1, CHAMF)
    in_sec = [(-Y_OUT + WALL, z0 + WALL), (Y_OUT - WALL, z0 + WALL), (Y_OUT - WALL, z1 - WALL), (-Y_OUT + WALL, z1 - WALL)]
    outer = prism(out_sec, "yz", X_BACK, X_HOOD)
    # hood front chamfer: the outer solid tapers HOOD_CH over the last HOOD_CH
    tip = chamfered_rect(-Y_OUT + HOOD_CH, Y_OUT - HOOD_CH, z0 + HOOD_CH, z1 - HOOD_CH, CHAMF)
    pts = [(X_BACK, a, b) for a, b in out_sec] + [(X_HOOD - HOOD_CH, a, b) for a, b in out_sec] + [(X_HOOD, a, b) for a, b in tip]
    outer = outer ^ m3.Manifold.hull_points(pts)
    fl = [(-Y_OUT + WALL - HOOD_FLARE, z0 + WALL - HOOD_FLARE), (Y_OUT - WALL + HOOD_FLARE, z0 + WALL - HOOD_FLARE),
          (Y_OUT - WALL + HOOD_FLARE, z1 - WALL + HOOD_FLARE), (-Y_OUT + WALL - HOOD_FLARE, z1 - WALL + HOOD_FLARE)]
    inner = prism(in_sec, "yz", X_FRAME, X_HOOD + 1) + m3.Manifold.hull_points(
        [(X_HOOD - HOOD_FLARE, a, b) for a, b in in_sec] + [(X_HOOD + 0.01, a, b) for a, b in fl])
    body = outer - inner
    # cheek windows (y faces): chevron "<" from the back corners to the front middle
    xa, xb = X_FRAME, HOOD_BACK; za, zb = z0 + LONG, z1 - LONG; zm = (z0 + z1) / 2; g = RIB / 2
    P1, P2, P3, P4, Q = (xa - g, za - g), (xb + g, za - g), (xb + g, zb + g), (xa - g, zb + g), (xb + g, zm)
    for w in tri_windows([[P1, P2, Q], [Q, P3, P4], [P1, Q, P4]], 0, g):
        body = body - prism(w, "xz", Y_OUT - WALL - 0.5, Y_OUT + 0.5) - prism(w, "xz", -Y_OUT - 0.5, -Y_OUT + WALL + 0.5)
    # lip windows (z faces): chevron from the back corners to the front middle
    ya, yb = -Y_OUT + LONG, Y_OUT - LONG
    P1, P2, P3, P4, Q = (xa - g, ya - g), (xb + g, ya - g), (xb + g, yb + g), (xa - g, yb + g), (xb + g, 0.0)
    for w in tri_windows([[P1, P2, Q], [Q, P3, P4], [P1, Q, P4]], 0, g):
        body = body - prism(w, "xy", z0 - 0.5, z0 + WALL + 0.5) - prism(w, "xy", z1 - WALL - 0.5, z1 + 0.5)
    # back frame: border + X, windows between
    plate = prism(out_sec, "yz", X_BACK, X_FRAME)
    yi, zi0, zi1 = Y_OUT - BORDER + XBAR / 2, z0 + BORDER - XBAR / 2, z1 - BORDER + XBAR / 2
    A, B, C, D, O = (-yi, zi0), (yi, zi0), (yi, zi1), (-yi, zi1), (0.0, zm)
    for w in tri_windows([[A, B, O], [B, C, O], [C, D, O], [D, A, O]], 0, XBAR / 2):
        plate = plate - prism(w, "yz", X_BACK - 0.5, X_FRAME + 0.5)
    body = body + plate
    for (y, z) in BOARD_HOLES:
        body = body + prism(octagon(BOSS_D / 2, y, z), "yz", X_BACK, X_BOSS) + prism(octagon(PAD_D / 2, y, z), "yz", X_BOSS - PAD_T, X_BOSS)
    # arms and knees (+y; mirrored): flanged web trusses (a WEB thick plate in the clip's x-z plane, a window, FLANGE wide
    # chords), from the clip's back flat (the neck: the finger roots at +-22.5 deg) into the back frame / the bottom wall
    xt, yt, zt = CLIP_TOP; xb_, yb_, zb_ = CLIP_BOT
    r_in = R_B + t_f; nh = r_in * OCT
    yc, yf = Y_OUT - WEB / 2, Y_OUT - FLANGE                                # the web in the cheek plane, flanges inboard
    def truss(poly, chords):
        w = prism(poly, "xz", yc - WEB / 2, yc + WEB / 2)
        win = m3.CrossSection([poly]).offset(-RIB, m3.JoinType.Miter)
        if not win.is_empty(): w = w - prism(win, "xz", yc - WEB / 2 - 1, yc + WEB / 2 + 1)
        for p_, q_, sgn in chords:                                     # the chord strip (FT inward of the edge), FLANGE wide
            p_, q_ = np.array(p_), np.array(q_); d = (q_ - p_) / np.linalg.norm(q_ - p_); n = np.array([-d[1], d[0]]) * sgn
            strip = [p_, q_, q_ + n * FT, p_ + n * FT]
            w = w + prism([tuple(v) for v in strip], "xz", yf, Y_OUT)
        return w
    u0 = r_in - 0.5
    T1, T2, T3, T4 = (xt + u0, zt - nh), (X_FRAME, zm + 2.0), (X_FRAME, z1 - 1.0), (xt + u0, zt + nh)
    side = truss([T1, T2, T3, T4], [(T4, T3, -1), (T1, T2, 1)])                                      # top arm
    K1, K2, K3, K4, K5 = (xb_ + u0, zb_ - nh), (X_FRAME + 9.0, z0), (X_FRAME + 9.0, z0 + WALL), (X_BACK + 1.5, z0 + WALL), (xb_ + u0, zb_ + nh)
    side = side + truss([K1, K2, K3, K4, K5], [(K1, K2, 1), (K4, K5, 1)])                            # bottom knee
    side = side + prism(chamfered_rect(X_BACK + 0.5, X_FRAME + 10.0, yf, Y_OUT - 0.5, 1.5), "xy", z0 + 0.5, z0 + WALL - 0.5)   # solid wall patch over the knee
    side = side + clip_solid(CLIP_TOP, t_f, snap, CLIP_L) + clip_solid(CLIP_BOT, t_f, snap, CLIP_L)
    # the bores and mouths stay clear of arm material
    for c in (CLIP_TOP, CLIP_BOT):
        hole = m3.CrossSection.circle(R_B, 64) + m3.CrossSection([[(0.0, R_T - snap), (-9.0, R_T - snap + 9.0 * math.tan(math.radians(FLARE)) + 3), (-9.0, -(R_T - snap) - 9.0 * math.tan(math.radians(FLARE)) - 3), (0.0, -(R_T - snap))]])
        cut = prism(hole.translate([c[0], c[2]]), "xz", c[1] - CLIP_L / 2 - 1, c[1] + CLIP_L / 2 + 1)
        side = (side - cut) + clip_solid(c, t_f, snap, CLIP_L)
    body = body + side + mirror_y(side)
    for (y, z) in BOARD_HOLES:
        body = body - prism(m3.CrossSection.circle(INSERT_HOLE / 2, 32).translate([y, z]), "yz", X_BACK - 1, X_BOSS + 1)
    return body


def build_collar(t_f, snap, CLIP_L):
    c = (CLIP_BOT[0], CLIP_BOT[1] + CLIP_L / 2 + COLLAR_GAP + COLLAR_L / 2, CLIP_BOT[2])
    return clip_solid(c, t_f, snap, COLLAR_L), c


# ---------------------------------------------------------------- checks
def in_view(P, lens=LENS, margin=VIEW_MARGIN, ap=0.0):
    d = P - lens; dx = d[:, 0]
    dy = np.maximum(np.abs(d[:, 1]) - ap, 0); dz = np.maximum(np.abs(d[:, 2]) - ap, 0)
    el = np.degrees(np.arctan2(dz, np.maximum(dx, 1e-6))); az = np.degrees(np.arctan2(dy, np.maximum(dx, 1e-6)))
    return (dx > 0) & (el < VFOV / 2 + margin) & (az < HFOV / 2 + margin), el, az


def view_report(mesh):
    P = np.vstack([mesh.vertices, mesh.sample(60000)])
    seen, el, az = in_view(P)
    seen_ap, el2, az2 = in_view(P, ap=APERTURE_R)
    front = P[:, 0] > LENS[0]
    m_pt = float(np.min(np.maximum(el[front] - VFOV / 2, az[front] - HFOV / 2))) if front.any() else 90.0
    m_ap = float(np.min(np.maximum(el2[front] - VFOV / 2, az2[front] - HFOV / 2))) if front.any() else 90.0
    return dict(in_view_points=int(seen.sum()), in_view_aperture=int(seen_ap.sum()), hood_lead_mm=round(float(mesh.bounds[1][0] - LENS[0]), 2),
                cone_margin_deg_point=round(m_pt, 1), cone_margin_deg_aperture=round(m_ap, 1))


def seg_dist(P, a, b):
    ab = b - a; t = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1); return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def cage_rods():
    cg = json.load(open(os.path.join(LAY, "cage_shell.json"))); res = json.load(open(os.path.join(LAY, "result.json")))
    up6 = set(json.load(open(SIZED))); N = np.array(cg["nodes"], float)
    rods = [(f"rod {a}-{b}", N[a], N[b], 3.0 if f"rod {a}-{b}" in up6 else 1.5) for a, b in cg["rods"]]
    rods += [(k, np.array(p, float), np.array(q, float), 3.0 if k in up6 else 1.5) for k, (p, q) in res["links"].items()]
    return cg, res, N, up6, rods


def fit_report(mesh, collar, N, rods, cg):
    P = np.vstack([mesh.vertices, mesh.sample(80000), collar.vertices, collar.sample(5000)])
    clip_rods = {"rod 7-8", "rod 14-15"}
    out = {}
    for nm, a, b, r in rods:
        if nm in clip_rods: continue
        out[nm] = float(seg_dist(P, a, b).min() - r)
    worst = sorted(out.items(), key=lambda kv: kv[1])[:4]
    nr = cg.get("node_r_mm", 6.0); nR = len(cg["open"][0]["ring"])
    node_gap = min(float(np.linalg.norm(P - N[i], axis=1).min() - (10.15 if i < nR else nr)) for i in range(len(N)))
    # every v12 part but the layout's own cradle / arms / clips and the camera (this piece replaces them; the camera is checked by the box)
    parts = {}
    skip = ("camera", "cage_shell")
    for f in sorted(os.listdir(os.path.join(LAY, "parts"))):
        if not f.endswith(".stl") or f.startswith(skip): continue
        q = trimesh.load(os.path.join(LAY, "parts", f), force="mesh")
        d = float(trimesh.proximity.signed_distance(q, P[::7][:20000]).max()) if q.is_watertight else None
        lo, hi = q.bounds; near = ((P > lo - 15) & (P < hi + 15)).all(1).any()
        if near: parts[f] = d
    cam = np.array([[x, y, z] for x in CAM_BOX[0] for y in CAM_BOX[1] for z in CAM_BOX[2]])
    lo, hi = cam.min(0), cam.max(0)
    inside_cam = int(((P > lo + 0.01) & (P < hi - 0.01)).all(1).sum())
    return dict(rod_clearance_min_mm=round(worst[0][1], 2), rod_clearance_worst=[(k, round(v, 2)) for k, v in worst], node_clearance_min_mm=round(node_gap, 2),
                part_penetration_mm={k: (round(v, 2) if v is not None else None) for k, v in parts.items()}, points_in_camera_box=inside_cam)


def stroke_report(mesh, rods, snap):
    """push-on stroke: the mount from +40 x to its seat; every rod but the clip rods clear, the clip rods by <= the snap"""
    P = np.vstack([mesh.vertices, mesh.sample(60000)])
    worst_other, worst_clip = 1e9, 0.0
    for s in np.arange(0.0, 40.01, 0.5):
        Q = P + [s, 0, 0]
        for nm, a, b, r in rods:
            d = seg_dist(Q, a, b) - r
            if nm in ("rod 7-8", "rod 14-15"):
                if s > 0.25: worst_clip = max(worst_clip, float(-d.min()))
            else:
                worst_other = min(worst_other, float(d.min()))
    return dict(other_rods_min_clear_mm=round(worst_other, 2), clip_rod_max_interference_mm=round(worst_clip, 2), allowed_mm=round(snap + 0.05, 2))


def props_report(mesh):
    out = {}
    for k in range(1, 5):
        f = os.path.join(LAY, "parts", f"prop_{k}.stl")
        if os.path.exists(f):
            q = trimesh.load(f, force="mesh"); out[f"prop_{k}"] = round(float(trimesh.proximity.closest_point(q, mesh.vertices[::5])[1].min()), 1)
    return out


# ---------------------------------------------------------------- frame model
PETG = lambda A, I: dict(E=E_PETG, G=E_PETG / 2.7, A=A, I=I, J=2 * I, c=math.sqrt(A) / 2)
RIGID = dict(E=100e6, G=40e6, A=1e3, I=1e5, J=2e5, c=5.0)


def oct_sec(a):
    A = 8 * (a / 2) ** 2 * OCT; I = A ** 2 / (4 * math.pi) * 1.0; s = PETG(A, I); s["c"] = a / 2 / math.cos(math.radians(22.5)); return s


def rigid_body_points(meshes_masses):
    """mass, CG and 6 lumped points with the body's inertia (principal axes)"""
    Ms, Cs, Is = [], [], []
    for (mesh, m) in meshes_masses:
        if isinstance(mesh, trimesh.Trimesh):
            c = mesh.center_mass; I = mesh.moment_inertia / mesh.mass * m                    # about its own CG, scaled to m
        else:
            (x0, x1), (y0, y1), (z0, z1) = mesh; c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); a, b, cc = x1 - x0, y1 - y0, z1 - z0
            I = m / 12 * np.diag([b * b + cc * cc, a * a + cc * cc, a * a + b * b])
        Ms.append(m); Cs.append(np.asarray(c, float)); Is.append(I)
    mtot = sum(Ms); cg = sum(m * c for m, c in zip(Ms, Cs)) / mtot
    It = np.zeros((3, 3))
    for m, c, I in zip(Ms, Cs, Is):
        d = c - cg; It += I + m * ((d @ d) * np.eye(3) - np.outer(d, d))
    w, V = np.linalg.eigh(It)
    a2 = 3 / (2 * mtot) * np.array([w[1] + w[2] - w[0], w[0] + w[2] - w[1], w[0] + w[1] - w[2]])
    pts = []
    for k in range(3):
        r = math.sqrt(max(a2[k], 1e-6))
        for s in (1, -1): pts.append(cg + s * r * V[:, k])
    return mtot, cg, pts


def frame_model(mount_mesh, mount_g, N, rods, cg_json, res, up6, F_pop, F_slide, t_f, CLIP_L, clip_E=E_PETG, chord_I="low"):
    f = Frame(); nid = [f.node(p) for p in N]
    ROD3, T6 = tube(3.0, 0.0), tube(6.0, 4.0)
    tubes = {k: (T6 if k in up6 else ROD3) for k, *_ in rods}
    clips = {"rod 7-8": [np.array(CLIP_TOP) * [1, s, 1] for s in (1, -1)], "rod 14-15": [np.array(CLIP_BOT) * [1, s, 1] for s in (1, -1)]}
    mass = {}
    def addm(i, g_): mass[i] = mass.get(i, 0.0) + g_
    gpm = lambda sec: sec["A"] * 1.55e-3
    for a, b in cg_json["rods"]:
        nm = f"rod {a}-{b}"; pts = [N[a]] + sorted(clips.get(nm, []), key=lambda c: np.linalg.norm(c - N[a])) + [N[b]]
        for p_, q_ in zip(pts[:-1], pts[1:]):
            i, j = f.node(p_), f.node(q_); f.member(i, j, tubes[nm], nm); w = np.linalg.norm(q_ - p_) * gpm(tubes[nm]); addm(i, w / 2); addm(j, w / 2)
    nR = len(cg_json["open"][0]["ring"])
    for i, j in enumerate(nid): addm(j, 4.5 if i < nR else 1.5)
    for k, (p, q) in res["links"].items():
        p, q = np.array(p), np.array(q); c_ = f.node(p); j = f.node(q); f.member(c_, j, tubes[k], k, k=0.7); f.fix(c_)
        w = np.linalg.norm(q - p) * gpm(tubes[k]); addm(j, w / 2)
    # the mount: clip necks, truss chords (PETG beams), the cradle + camera one rigid body. Frame takes one I per member:
    # chord_I "low" = the flange's in-plane I (the truss carries in-plane loads axially), "high" = its out-of-plane I
    xt, _, zt = CLIP_TOP; xb, _, zb = CLIP_BOT; zm = (Z_OUT[0] + Z_OUT[1]) / 2; z0, z1 = Z_OUT
    r_in = R_B + t_f; nh = r_in * OCT; u0 = r_in - 0.5; yc = Y_OUT - WEB / 2
    clip_sec = PETG(CLIP_L * 2 * nh, (2 * nh) ** 3 * CLIP_L / 12); clip_sec["E"] = clip_E; clip_sec["c"] = nh   # the clip's back flat (bending about the tube axis)
    Af = FLANGE * FT + (RIB - FT) * WEB
    chord = PETG(Af, FT ** 3 * FLANGE / 12 if chord_I == "low" else FT * FLANGE ** 3 / 12); chord["c"] = FLANGE / 2 if chord_I == "high" else FT / 2
    edge = PETG(RIB * WEB, min(RIB, WEB) ** 3 * max(RIB, WEB) / 12); edge["c"] = min(RIB, WEB) / 2
    mtot, cg, pts = rigid_body_points([(mount_mesh, mount_g), (CAM_BOX, CAM_G)])
    H = f.node(cg)
    for p in pts:
        i = f.node(p); f.member(H, i, RIGID, "body"); addm(i, mtot / 6)
    for s in (1, -1):
        R_t = f.node((xt, 18 * s, zt)); A_t = f.node((xt + u0, 18 * s, zt))
        T3 = f.node((X_FRAME - 1.5, yc * s, z1 - 2.5)); T2 = f.node((X_FRAME - 1.5, yc * s, zm + 3.5))
        R_b = f.node((xb, 18 * s, zb)); A_b = f.node((xb + u0, 18 * s, zb))
        K2 = f.node((X_FRAME + 7.5, yc * s, z0 + 1.5)); K4 = f.node((X_BACK + 2.0, yc * s, z0 + 1.5))
        f.member(R_t, A_t, clip_sec, f"clip top {s:+d}"); f.member(A_t, T3, chord, f"arm top chord {s:+d}"); f.member(A_t, T2, chord, f"arm lower chord {s:+d}")
        f.member(R_b, A_b, clip_sec, f"clip bottom {s:+d}"); f.member(A_b, K2, chord, f"knee chord {s:+d}"); f.member(A_b, K4, chord, f"knee inner chord {s:+d}")
        for B in (T2, T3, K2, K4): f.member(H, B, RIGID, "body")
    hood = f.node((X_HOOD, 0, 42.0)); f.member(H, hood, RIGID, "body")
    corner = f.node((X_HOOD, Y_OUT - 2, Z_OUT[1] - 2)); f.member(H, corner, RIGID, "body")
    K = f.stiffness()
    hz, U = f.modes(mass, 6)
    body_ids = [f.node(p) for p in pts]
    tot_m = sum(mass.values())
    part, hd = [], []
    for k in range(len(hz)):
        e_body = sum(mass[i] * np.sum(U[6 * i:6 * i + 3, k] ** 2) for i in body_ids); e_all = sum(m_ * np.sum(U[6 * i:6 * i + 3, k] ** 2) for i, m_ in mass.items())
        part.append(e_body / e_all)
        hd.append(max(np.linalg.norm(U[6 * i:6 * i + 3, k]) for i in body_ids) / max(np.linalg.norm(U[6 * i:6 * i + 3, k]) for i in mass))
    def run(loads):
        u = f.solve(loads); mf = f.member_forces(u); out = {}
        for r in mf:
            nm = r["name"]
            if nm == "body": continue
            sec = next(m_["sec"] for m_ in f.M if m_["name"] == nm)
            lim = (SY_PETG if sec["E"] <= E_PETG + 1 else 600.0)
            o = out.setdefault(nm, dict(sigma=0.0, ratio=0.0, buckle=0.0))
            o["sigma"] = max(o["sigma"], r["sigma"]); o["ratio"] = max(o["ratio"], r["sigma"] / lim, r["buckle"]); o["buckle"] = max(o["buckle"], r["buckle"])
        # the clip-rod forces (force from the rod on the clip at the rod node)
        cf = {}
        for m_ in f.M:
            if not m_["name"].startswith("clip"): continue
            kg, T, kl, L = f._k(m_); ue = np.concatenate([u[6 * m_["a"]:6 * m_["a"] + 6], u[6 * m_["b"]:6 * m_["b"] + 6]])
            cf[m_["name"]] = (kg @ ue)[:3]
        return u, out, cf
    cases = {}
    lens_id = f.node(tuple(LENS)); f.member(H, lens_id, RIGID, "body")
    def summarise(name, loads):
        u, out, cf = run(loads)
        petg_ = {k: v for k, v in out.items() if not k.startswith(("rod", "saddle"))}
        carb = {k: v for k, v in out.items() if k.startswith(("rod", "saddle"))}
        wp = max(petg_.items(), key=lambda kv: kv[1]["sigma"]); wc = max(carb.items(), key=lambda kv: kv[1]["ratio"])
        pop = max(max(0.0, -v[0]) for v in cf.values()); sl = abs(sum(v[1] for v in cf.values())) / 4
        cases[name] = dict(petg_worst=wp[0], petg_sigma_MPa=round(wp[1]["sigma"], 2), carbon_worst=wc[0], carbon_ratio=round(wc[1]["ratio"], 3),
                           clip_pull_N=round(pop, 2), net_side_per_clip_N=round(sl, 2), lens_disp_mm=round(float(np.linalg.norm(u[6 * lens_id:6 * lens_id + 3])), 3))
        return u, out, cf
    summarise("frontal hit 250 N on the hood centre (-x)", {hood: [-HIT, 0, 0]})
    summarise("frontal hit 250 N on the hood top corner (-x)", {corner: [-HIT, 0, 0]})
    summarise("flight 4 g down (mount + camera)", {H: [0, 0, -mtot / 1000 * G * FLIGHT]})
    for nm, d in (("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+y", (0, 1, 0)), ("-z", (0, 0, -1)), ("+z", (0, 0, 1))):
        summarise(f"crash 35 g inertial {nm} (mount + camera)", {H: list(np.array(d) * mtot / 1000 * G * CRASH)})
    # breaking order: unit load at the hood centre per direction, the load where each thing first lets go
    order = {}
    for nm, d in (("pull forward +x", (1, 0, 0)), ("push back -x (frontal)", (-1, 0, 0)), ("side +y", (0, 1, 0)), ("up +z", (0, 0, 1)), ("down -z", (0, 0, -1))):
        u, out, cf = run({hood: list(np.array(d, float))})
        cand = []
        for k, v in out.items():
            if v["ratio"] > 1e-9: cand.append((1.0 / v["ratio"], k + (" (PETG yield)" if not k.startswith(("rod", "saddle")) else " (carbon)")))
        pull = max(max(0.0, -v[0]) for v in cf.values()); sl = abs(sum(v[1] for v in cf.values())) / 4
        if pull > 1e-9: cand.append((F_pop / pull, "a clip pops"))
        if sl > 1e-9: cand.append((F_slide / sl, "clips slide (grip), then the stop collars"))
        cand.sort()
        order[nm] = [(round(a, 0), b) for a, b in cand[:3]]
    big = sorted(range(len(f.X)), key=lambda i: -np.linalg.norm(U[6 * i:6 * i + 3, 0]))[:6]
    mode1_nodes = [np.round(f.X[i], 1).tolist() for i in big]
    return dict(mode1_largest_motion_at=mode1_nodes, modes_hz=[round(float(h), 1) for h in hz], camera_share=[round(p, 2) for p in part], camera_disp_ratio=[round(float(x), 2) for x in hd],
                first_camera_mode_hz=round(float(next(h for h, p in zip(hz, part) if p > 0.3)), 1),
                body_mass_g=round(mtot, 1), body_cg=np.round(cg, 1).tolist(), cases=cases, breaking_order=order)


# ---------------------------------------------------------------- main
def main():
    os.makedirs(OUT, exist_ok=True)
    snap_pick, snap_rows = pick_snap()
    t_f = T_F or snap_pick["t_f"]; sn = SNAP or snap_pick["snap"]; CL = snap_pick["L"]; sp = snap_numbers(t_f, sn, CL)
    print("snap:", json.dumps(sp))
    man = build(t_f, sn, CL); mesh = tm(man)
    col, col_c = build_collar(t_f, sn, CL); cmesh = tm(col)
    g_mount = mesh.volume * RHO; g_col = cmesh.volume * RHO
    print(f"mount {g_mount:.2f} g, collar {g_col:.2f} g x2, watertight {mesh.is_watertight}, bodies {len(mesh.split(only_watertight=False))}, bounds {np.round(mesh.bounds, 1).tolist()}")
    mesh.export(os.path.join(OUT, "camera_mount_assembled.stl")); cmesh.export(os.path.join(OUT, "stop_collar_assembled.stl"))
    vr = view_report(mesh); print("view:", vr)
    cg_json, res, N, up6, rods = cage_rods()
    fr = fit_report(mesh, cmesh, N, rods, cg_json); print("fit:", json.dumps(fr))
    st = stroke_report(mesh, rods, sn); print("stroke:", st)
    pr = props_report(mesh); print("props:", pr)
    fm = frame_model(mesh, g_mount, N, rods, cg_json, res, up6, sp["pop_N"], sp["slide_N"], t_f, CL)
    fm_soft = frame_model(mesh, g_mount, N, rods, cg_json, res, up6, sp["pop_N"], sp["slide_N"], t_f, CL, clip_E=E_PETG / 10)
    fm_high = frame_model(mesh, g_mount, N, rods, cg_json, res, up6, sp["pop_N"], sp["slide_N"], t_f, CL, chord_I="high")
    # the old frame-model case for comparison: the 3.0-everywhere cage (Spark may revert)
    print("frame:", json.dumps({k: fm[k] for k in ("mode1_largest_motion_at", "modes_hz", "camera_share", "camera_disp_ratio", "first_camera_mode_hz", "body_mass_g")}))
    for k, v in fm["cases"].items(): print("  ", k, v)
    for k, v in fm["breaking_order"].items(): print("  break", k, v)
    print("soft clips (E/10):", fm_soft["modes_hz"][:3], " chords out-of-plane I:", fm_high["modes_hz"][:3])
    result = dict(part="scout_camera_mount", version="v1", rod_od=ROD_OD, bore=BORE, mass_mount_g=round(g_mount, 2), mass_collar_g=round(g_col, 2),
                  bounds=np.round(mesh.bounds, 2).tolist(), snap=sp, snap_scan=snap_rows, view=vr, fit=fr, stroke=st, props=pr, frame=fm,
                  frame_soft_clips=dict(modes_hz=fm_soft["modes_hz"], camera_share=fm_soft["camera_share"]),
                  frame_chords_high=dict(modes_hz=fm_high["modes_hz"], camera_share=fm_high["camera_share"]), collar_centre=list(col_c),
                  params=dict(ARM=ARM, BRACE=BRACE, WALL=WALL, CHAMF=CHAMF, RIB=RIB, XBAR=XBAR, BORDER=BORDER, CLIP_L=CL, T_F=t_f, SNAP=sn, FLARE=FLARE, GRIP=GRIP,
                              X_BACK=X_BACK, X_HOOD=X_HOOD, APERTURE_R=APERTURE_R, MU=MU))
    # print files: +y face (the cheek plane: cheek, arm and knee webs, the clips' ends) on the bed, clip bores vertical
    def print_pose(m):
        q = m.copy(); q.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
        q.apply_translation([-q.bounds[0][0] - q.extents[0] / 2, -q.bounds[0][1] - q.extents[1] / 2, -q.bounds[0][2]]); return q
    os.makedirs(os.path.join(ROOT, "print"), exist_ok=True)
    pm, pc = print_pose(mesh), print_pose(cmesh)
    pm.export(os.path.join(ROOT, "print", "scout_camera_mount_v1.stl")); pc.export(os.path.join(ROOT, "print", "scout_camera_mount_collar_v1.stl"))
    result["print"] = dict(mount=dict(file="print/scout_camera_mount_v1.stl", extents=np.round(pm.extents, 1).tolist()),
                           collar=dict(file="print/scout_camera_mount_collar_v1.stl", extents=np.round(pc.extents, 1).tolist(), qty=2))
    result["system"] = system_recheck(mesh, cmesh, g_mount, g_col)
    json.dump(result, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
    return result


def system_recheck(mesh, cmesh, g_mount, g_col):
    """the assembled recheck: the v12 Scout manifest with this mount in place of the layout's cradle, arms and clips"""
    import subprocess, copy
    a = json.load(open(os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")))
    old = {"camera cradle (printed)", "camera cradle arms (printed)", "camera cradle clips (printed)"}
    rel = os.path.relpath(OUT, ROOT)
    cm = cmesh.copy(); cm.apply_transform(np.diag([1, -1, 1, 1])); cm.invert()
    cm.export(os.path.join(OUT, "stop_collar_assembled_my.stl"))
    parts = [p for p in a["parts"] if p["name"] not in old]
    parts.append(dict(name="camera mount (printed)", role="cage", mesh=f"{rel}/camera_mount_assembled.stl", material="PETG", mass_g=round(g_mount, 2), tag="M",
                      owner="scout camera mount", version="v1", source="nox/scout/v3/camera_mount.py"))
    for k, f in (("+y", "stop_collar_assembled.stl"), ("-y", "stop_collar_assembled_my.stl")):
        parts.append(dict(name=f"camera mount stop collar {k}", role="cage", mesh=f"{rel}/{f}", material="PETG", mass_g=round(g_col, 2), tag="M",
                          owner="scout camera mount", version="v1", source="nox/scout/v3/camera_mount.py"))
    cp = []
    for x, y, t in a["contact_pairs"]:
        if x in old - {"camera cradle (printed)"} or y in old - {"camera cradle (printed)"}: continue
        cp.append([("camera mount (printed)" if v == "camera cradle (printed)" else v) for v in (x, y)] + [t])
    # no contact pairs for the tubes: the layout draws them 3.0 (r 1.5), inside the 6.2 bores, so the fit check sees any real touch
    a["parts"], a["contact_pairs"] = parts, cp
    a["note"] = "v12 Scout manifest with the camera mount v1 (runs/scout_camera_mount/v1) in place of the layout's cradle, arms and clips. " + a["note"]
    man = os.path.join(OUT, "scout_with_mount.assembly.json"); json.dump(a, open(man, "w"), indent=1)
    r = subprocess.run([sys.executable, os.path.join(ROOT, "nox", "system_check.py"), man, "--report", "--out", os.path.join(OUT, "system_check.json")],
                       capture_output=True, text=True, cwd=ROOT)
    open(os.path.join(OUT, "system_check.txt"), "w").write(r.stdout + r.stderr)
    print(r.stdout[-3000:])
    return dict(manifest=os.path.relpath(man, ROOT), report=f"{rel}/system_check.txt")


if __name__ == "__main__":
    main()
