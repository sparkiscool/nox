"""S2 cage v2 (the guard) CAD: corner nodes, flat-printed wheels, tee sleeves, side and tail hubs, floor feet, skids, rods.

Drone coordinates (x nose, y left, z up, mm). Bodies are smooth capsule unions (SDF + marching cubes, 0.15 mm).
17 Oct (cage v2 rerun, props on top at z 41.2, D12 19.2 E):
  * rings follow the prop plane (dimensions G): lower 2.5 at 33.2, upper 2.0 at 49.2.
  * PRINT CLOSURE, no supports (nox/dubhacks-queue.md rules): every part's bed face comes from a search over the 6 axis
    directions (ridge residual under 15 mm2, a bed face of 50 mm2 or more, lowest build height, biggest bed face); the body is then
    closed for that direction: each layer holds at least the layer above shrunk by one layer height, so every surface
    is 45 deg or steeper (keels under the horizontal members), never through a keep-out (collar faces, plate, top case).
    What is left unsupported are ridge lines between two supported ends (bridges, checked in print_check.py).
    Exact features (bores, slots, pin holes, face cuts) are cut after it.
  * wheels are separate flat prints on a 2.0 carbon pin (print-in-place wheels on a horizontal axle need supports);
    the node keeps a slot open outward to slide the wheel in, the pin is epoxied in the cheeks.
  * lower ring runs through the tees (tee = TPU sleeve + strut socket; frame model pass 1),
    mouth posts 4.0 tie each upper socket mouth to the lower socket (pass 3, and they carry the upper socket in print).
  * 17 Oct 03:27 (receiver v3 bottom layout, Spark): pack nose-tail, bottom z -44.5 (E); foot bottoms z -47.5; a 2.5 carbon
    skid joins each side's front and back foot along x at y +-30.75 (foot_opt.py); legs still run to the side hub roots.
    Leg bores start 5.5 up the foot (clear of the skid bore) and 5.0 out of the hub root (rod ends 0.85 apart, C).
Numbers: v2/dimensions.md G. Placements: strut_route.py + tail_route.py (struts), foot_opt.py (feet, skids),
s2_corner_opt.py (c 100), s2_node_wheels.py (wheels). Frame model: v2/s2_guard_frame.py (v2_main).
Run: python3 cage_cad.py [part ...]  (CAGE_OUT sets the output folder)
"""
import json, math, itertools, os, sys
import numpy as np
from scipy import ndimage
from skimage.measure import marching_cubes
import manifold3d
from manifold3d import Manifold, Mesh
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get("CAGE_OUT", "/home/claude/cage/out")
os.makedirs(OUT, exist_ok=True)

# ---------------- inputs ----------------
PZ = 41.2                     # prop plane (dimensions C: plate top 22 + D12 19.2, E)
F, C = 112.0, 100.0           # ring flats / chamfer start (C)
ZL, ZU = PZ - 8.0, PZ + 8.0   # lower / upper ring centres (C)
R25, R20, R30 = 1.25, 1.0, 1.5   # rod radii: lower 2.5 and skids, upper 2.0, struts and legs 3.0 solid carbon
GAP = 0.3                     # bore over rod (interfaces.py practice, P for 3 mm PETG)
LEAD = 0.6                    # socket lead-in (C)
SLOPE = math.radians(45)      # collar faces (dimensions G)
WR, WW = 4.0, 3.0             # wheel radius, width (C, catch interface)
PIN_R, PIN_HOLE_R, WHEEL_BORE_R = 1.0, 1.1, 1.3   # 2.0 carbon pin; epoxy hole 2.2; wheel bore 2.6 (0.3 radial, E until D14)
WGAP_AX = 0.4                 # wheel side gap (E, coupon D14)
PIN_IN = 1.6                  # pin depth in each cheek (C) -> pin 2.0 x (WW + 2 WGAP_AX + 2 PIN_IN) = 7.0
PROUD = 2.5                   # wheel rim beyond the lower-ring rod surface (C: node walls 1.35 + 1.0 face clearance)
FACE_CLEAR = 1.0              # everything but the wheels stays this far off the faces (C)
MOUTH = 86.0                  # mouth post station along each flat (frame model pass 3)
VOX = 0.15
ROUTE = json.load(open(os.path.join(HERE, "strut_route.json")))
FOOT = json.load(open(os.path.join(HERE, "foot_opt.json")))
LEG_FOOT_BORE = (5.5, 17.5)   # leg bore along the leg from the foot centre (clear of the skid bore, C)
LEG_HUB_BORE = (5.0, 17.0)    # leg bore along the leg from the hub root
SKID_BORE = (1.5, 13.5)       # skid bore along x inboard from the foot station
# S2 top case keep-out for the side hubs (v2/s2-top-case.md): its pads stack on the hub pads (z 26 up at the side bolts)
# and its parts sit inside |x| <= 33, |y| <= 37.5 above z 26 (Radxa 30 x 65 along y, bosses (+-11.45, +-28.9), pads
# r 4.75 round (3.9, +-31), E); the FC zone and lanes are checked in cage_check.py.
CASE_BOX = ((-33.0, -37.5, 26.0), (33.0, 37.5, 90.0))

n_out_x = np.array([math.sin(SLOPE), 0, -math.cos(SLOPE)])   # +x face, toward the face
def face_rot(k):   # 90 deg steps about z
    a = k * math.pi / 2
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
FACES = [face_rot(k) @ n_out_x for k in range(4)]                  # +x, +y, -x, -y
RIM_OFF = R25 + PROUD          # rod axis to face plane, along the normal
def face_plane(k):
    n = FACES[k]; p = face_rot(k) @ np.array([F, 0, ZL]) + n * RIM_OFF
    return n, float(n @ p)

# ---------------- SDF helpers ----------------
def sd_capsule(P, a, b, r):
    a, b = np.asarray(a, np.float32), np.asarray(b, np.float32); ba = b - a; L2 = float(ba @ ba)
    pa = P - a
    if L2 < 1e-9:
        return np.linalg.norm(pa, axis=1) - r
    h = np.clip((pa @ ba) / L2, 0, 1)
    return np.linalg.norm(pa - h[:, None] * ba, axis=1) - r
def sd_zcyl(P, c, r, z0, z1):
    """flat-ended vertical cylinder"""
    d = np.linalg.norm(P[:, :2] - np.asarray(c[:2], np.float32), axis=1) - r
    h = np.maximum(z0 - P[:, 2], P[:, 2] - z1)
    return np.minimum(np.maximum(d, h), 0) + np.hypot(np.maximum(d, 0), np.maximum(h, 0))
def smin(a, b, k=1.2):
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0, 1)
    return b * (1 - h) + a * h - k * h * (1 - h)

AX = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}

class Body:
    """SDF of a part on a 0.15 grid, its keep-outs, and the print closure for a build direction."""
    def __init__(self, caps, cyls=(), keepout=None, pad=6.0, k=1.2):
        pts = np.array([c[0] for c in caps] + [c[1] for c in caps], float)
        rmax = max(c[2] for c in caps)
        lo = pts.min(0) - rmax - pad; hi = pts.max(0) + rmax + pad
        for c_, r_, z0, z1 in cyls:
            lo = np.minimum(lo, [c_[0] - r_ - pad, c_[1] - r_ - pad, z0 - pad]); hi = np.maximum(hi, [c_[0] + r_ + pad, c_[1] + r_ + pad, z1 + pad])
        self.lo = lo
        nx, ny, nz = (np.ceil((hi - lo) / VOX).astype(int) + 1)
        xs, ys, zs = (lo[i] + VOX * np.arange(n) for i, n in enumerate((nx, ny, nz)))
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        vol = np.empty((nx, ny, nz), np.float32); forb = np.zeros((nx, ny, nz), bool)
        for iz, z in enumerate(zs):
            G = np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], 1).astype(np.float32)
            d = None
            for a_, b_, r in caps:
                if z < min(a_[2], b_[2]) - r - 3 * k or z > max(a_[2], b_[2]) + r + 3 * k:
                    continue
                di = sd_capsule(G, a_, b_, r); d = di if d is None else smin(d, di, k)
            for c_, r_, z0, z1 in cyls:
                di = sd_zcyl(G, c_, r_, z0, z1); d = di if d is None else np.minimum(d, di)
            d = d if d is not None else np.full(X.size, 99, np.float32)
            if keepout is not None:
                ko = keepout(G)                      # signed: < 0 inside the keep-out
                d = np.maximum(d, -ko); forb[:, :, iz] = (ko < 0).reshape(nx, ny)
            vol[:, :, iz] = d.reshape(nx, ny)
        self.vol, self.forb = vol, forb

    def closure(self, build, bed_band=0.9):
        """45 deg closure for a build direction: returns (field, added mm3, bed mm2, height, residual mm2).
        bed_band: the lowest 0.9 is the cross-section 0.9 up extruded straight down (flat bed face, no crescents)."""
        ax, sg = AX[build]
        V = np.moveaxis(self.vol, ax, -1); Fb = np.moveaxis(self.forb, ax, -1)
        if sg < 0: V = V[..., ::-1]; Fb = Fb[..., ::-1]
        O = V < 0
        ks = np.where(O.any(axis=(0, 1)))[0]; k0, k1 = ks[0], ks[-1]
        Oc = O.copy(); resid = 0
        for kk in range(k1 - 1, k0 - 1, -1):
            up = Oc[:, :, kk + 1]
            if not up.any(): continue
            dist = ndimage.distance_transform_edt(up)          # voxels to the outside of the layer above
            need = (dist > 1.0 + 1e-6) & ~Fb[:, :, kk]          # the layer above shrunk by one voxel = 45 deg
            Oc[:, :, kk] |= need
        nb = int(round(bed_band / VOX))
        for kk in range(k0, min(k0 + nb, k1)):
            Oc[:, :, kk] |= Oc[:, :, k0 + nb] & ~Fb[:, :, kk]
        for kk in range(k0 + 1, k1 + 1):                      # residual: voxels with nothing within one voxel below
            below = ndimage.binary_dilation(Oc[:, :, kk - 1], structure=np.ones((3, 3), bool))
            resid += int((Oc[:, :, kk] & ~below).sum())
        added = float((Oc & ~O).sum()) * VOX ** 3
        bed = float(Oc[:, :, k0].sum()) * VOX ** 2
        Wf = V.copy()
        addm = Oc & ~O
        for kk in range(k0, k1 + 1):
            if not addm[:, :, kk].any(): continue
            lay = Oc[:, :, kk]
            sd = (ndimage.distance_transform_edt(~lay) - ndimage.distance_transform_edt(lay)) * VOX + 0.5 * VOX
            Wf[:, :, kk] = np.minimum(V[:, :, kk], sd)
        Wf[..., :k0] = np.maximum(Wf[..., :k0], 0.05)          # flat bed at the first layer
        if sg < 0: Wf = Wf[..., ::-1]
        Wf = np.ascontiguousarray(np.moveaxis(Wf, -1, ax))
        return Wf, added, bed, (k1 - k0 + 1) * VOX, resid * VOX ** 2

    def mesh(self, field):
        v, f, _, _ = marching_cubes(field, 0.0, spacing=(VOX, VOX, VOX))
        m = trimesh.Trimesh(v + self.lo, f, process=True)
        if m.volume < 0: m.invert()
        return to_manifold(m)

def best_build(body, dirs=("+z", "-z", "+x", "-x", "+y", "-y"), log=print, name=""):
    """bed face search (queue rules: flat prints, lowest build height): residual ridge area must stay under 15 mm2
    (ridges between supported ends print as bridges; print_check.py verifies), then a bed face of 50 mm2 or more (the queue's 'largest flat face on the plate'), then lowest height (to the mm),
    then the biggest bed face, then least added"""
    rows = []
    for d in dirs:
        _, added, bed, h, res = body.closure(d)
        rows.append((res > 15.0, round(h, 1), round(added, 1), d, round(bed, 1), round(res, 1)))
    rows.sort(key=lambda r: (r[0], r[4] < 50.0, round(r[1]), -r[4], r[2]))   # a real bed face (>= 50 mm2) first, then height
    for r in rows: log(f"  {name} build {r[3]}: height {r[1]}, residual {r[5]} mm2, added {r[2]} mm3, bed {r[4]} mm2")
    return rows[0][3], rows

def to_manifold(tm):
    return Manifold(Mesh(vert_properties=np.asarray(tm.vertices, np.float32), tri_verts=np.asarray(tm.faces, np.uint32)))
def to_trimesh(man):
    mm_ = man.to_mesh(); return trimesh.Trimesh(np.asarray(mm_.vert_properties)[:, :3], np.asarray(mm_.tri_verts))

# ---------------- exact solids ----------------
def frame_to(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; L = np.linalg.norm(d); z = d / L
    x = np.cross([0, 0, 1], z) if abs(z[2]) < 0.99 else np.cross([1, 0, 0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    M = np.zeros((3, 4)); M[:, 0], M[:, 1], M[:, 2], M[:, 3] = x, y, z, a
    return M, L
def cyl(a, b, r, r2=None, seg=48):
    M, L = frame_to(a, b)
    return Manifold.cylinder(L, r, r if r2 is None else r2, seg).transform(M)
def bore(a_bottom, b_mouth, rod_r, extra=4.0, trim_r=3.6):
    """rod bore from bottom to mouth, 0.6 lead-in at the mouth, and the body trimmed flat at the mouth plane"""
    a, b = np.asarray(a_bottom, float), np.asarray(b_mouth, float); u = (b - a) / np.linalg.norm(b - a)
    r = rod_r + GAP / 2
    return (cyl(a, b, r) + cyl(b - u * LEAD, b + u * 0.01, r, r + LEAD + 0.01) + cyl(b, b + u * extra, trim_r))
def through_bore(a, b, rod_r, trim_r=3.6):
    """rod passes through: bore a..b with lead-ins at both mouths, trimmed flat at both ends"""
    a, b = np.asarray(a, float), np.asarray(b, float); u = (b - a) / np.linalg.norm(b - a); r = rod_r + GAP / 2
    return (cyl(a - u * 0.01, b + u * 0.01, r) + cyl(b - u * LEAD, b + u * 0.01, r, r + LEAD + 0.01)
            + cyl(a + u * LEAD, a - u * 0.01, r, r + LEAD + 0.01) + cyl(b, b + u * 4.0, trim_r) + cyl(a, a - u * 4.0, trim_r))
def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Manifold.cube(tuple(hi - lo)).translate(tuple(lo))
def halfspace_keepout(n, d, margin):
    """everything with n.p > d - margin (within margin of the plane or past it)"""
    B = 2000.0
    b = Manifold.cube((B, B, B), center=True).translate((0, 0, B / 2))      # z >= 0
    z = np.asarray(n, float); x = np.cross([0, 0, 1], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    T = np.zeros((3, 4)); T[:, 0], T[:, 1], T[:, 2] = x, y, z; T[:, 3] = z * (d - margin)
    return b.transform(T)
def face_keepout_sdf(G):
    """signed distance to the face keep-out (< 0 inside: within FACE_CLEAR of a collar face or past it)"""
    out = None
    for k in range(4):
        n, d = face_plane(k); s = (d - FACE_CLEAR) - G @ n.astype(np.float32)
        out = s if out is None else np.minimum(out, s)
    return out

# ---------------- corner node (+x +y corner; the other 3 are this part turned 90 deg) ----------------
def wheel_center(node_xy, k):
    n = FACES[k]; p = np.array([*node_xy, ZL])
    return p + n * (RIM_OFF - WR)                       # rim = centre + WR * n touches the plane
def corner_caps():
    PA, PB = (F, C), (C, F)
    cA, cB = wheel_center(PA, 0), wheel_center(PB, 1)
    K = np.array([106.0, 106.0]); caps = []
    for (px, py), cw, ax in ((PA, cA, 1), (PB, cB, 0)):
        def P(along, z, inward=0.0):
            q = np.array([px, py, z], float); q[ax] = along; q[1 - ax] += inward; return q
        ib = -7.0
        caps += [(P(83, ZL), P(98.1, ZL), 2.6),                              # lower socket body, near cheek
                 (P(101.5, cw[2]), P(103.4, cw[2]), 2.6),                     # far cheek
                 (P(95.5, ZL, ib), P(104.0, ZL, ib), 2.0),                    # inboard bridge round the wheel
                 (P(96.5, ZL), P(95.5, ZL, ib), 2.0),
                 (P(103.4, cw[2]), P(104.0, ZL, ib), 2.0),
                 (P(103.4, cw[2]), [*K, ZL], 2.2), (P(104.0, ZL, ib), [*K, ZL], 2.0),
                 (P(85, ZU), P(97, ZU), 2.3), (P(97, ZU), [*K, ZU], 2.2),     # upper socket body, link to the post top
                 (P(96.0, ZL, ib), P(93.0, ZU), 1.8),                         # brace at the rod ends
                 (P(MOUTH, ZL), P(MOUTH, ZU), 2.0)]                           # mouth post (17 Oct)
    caps.append(([*K, ZL], [*K, ZU], 2.4))
    return caps, (PA, cA, 1), (PB, cB, 0)
def corner_node(build=None, log=print):
    caps, A, B = corner_caps()
    body = Body(caps, keepout=face_keepout_sdf)
    if build is None: build, _ = best_build(body, log=log, name="corner")
    field, added, bed, h, res = body.closure(build)
    m = body.mesh(field)
    for (px, py), cw, ax in (A, B):
        def Q(along, z):
            q = np.array([px, py, z], float); q[ax] = along; return q
        axis = np.zeros(3); axis[ax] = 1.0; out = np.zeros(3); out[1 - ax] = 1.0
        m = m - bore(Q(95.0, ZL), Q(83.0, ZL), R25)                           # lower socket 12 deep
        m = m - bore(Q(95.0, ZU), Q(85.0, ZU), R20)                           # upper socket 10 deep
        half = WW / 2 + WGAP_AX
        slot = cyl(cw - axis * half, cw + axis * half, WR + 0.4)
        lo = cw - axis * half - np.array([0, 0, WR + 0.4]); hi = cw + axis * half + out * 20.0 + np.array([0, 0, WR + 0.4])
        slot = slot + box(np.minimum(lo, hi), np.maximum(lo, hi))             # open outward: the wheel slides in
        m = m - slot
        m = m - cyl(cw - axis * (half + PIN_IN), cw + axis * (half + PIN_IN), PIN_HOLE_R)   # pin hole through both cheeks
    for k in range(4):
        n, d = face_plane(k); m = m - halfspace_keepout(n, d, FACE_CLEAR)
    return m, dict(cA=A[1].tolist(), cB=B[1].tolist(), build=build, added=added, bed=bed, height=h, residual=res)
def wheel():
    """flat print, bore vertical; 8 per drone"""
    return cyl((0, 0, 0), (0, 0, WW), WR, seg=96) - cyl((0, 0, -1), (0, 0, WW + 1), WHEEL_BORE_R, seg=48)

# ---------------- tee sleeve (+y side tee at (0, F); the other two are this part turned) ----------------
def strut_dir(name):
    r = ROUTE[name]; Bp = np.array(r["bracket_socket_bottom"], float); R = np.array(r["ring_point"], float)
    R[2] = ZL; u = (R - Bp) / np.linalg.norm(R - Bp); return Bp, R, u
def tee_node(which="side", build=None, log=print):
    Bp, R, u = strut_dir(which)
    if which == "tail":
        T = face_rot(3); Bp, R, u = T @ Bp, T @ R, T @ u
    caps = [((-17, F, ZL), (17, F, ZL), 2.6),
            (tuple(R - 2.0 * u), tuple(R - 15.0 * u), 2.9),
            ((-6, F, ZL), tuple(R - 8.0 * u), 2.2), ((6, F, ZL), tuple(R - 8.0 * u), 2.2)]
    body = Body(caps, keepout=face_keepout_sdf)
    if build is None: build, _ = best_build(body, log=log, name=f"tee {which}")
    field, added, bed, h, res = body.closure(build)
    m = body.mesh(field)
    m = m - through_bore((-17.5, F, ZL), (17.5, F, ZL), R25)                  # lower ring runs through (17 Oct)
    m = m - bore(R - 2.5 * u, R - 14.5 * u, R30)
    for k in range(4):
        n, d = face_plane(k); m = m - halfspace_keepout(n, d, FACE_CLEAR)
    return m, dict(build=build, added=added, bed=bed, height=h, residual=res)

# ---------------- side hub (on the side plate bolt, plate top z 22) ----------------
PLATE = ((-50.2, -47.1, 18.5), (50.2, 47.1, 22.0))   # plate 100 x 93.8 (M) + 0.2 play, 3.5 carbon (D8)
def _box_sdf(G, lo, hi):
    lo, hi = np.asarray(lo, np.float32), np.asarray(hi, np.float32)
    q = np.maximum(lo - G, G - hi)
    return np.minimum(q.max(axis=1), 0) + np.linalg.norm(np.maximum(q, 0), axis=1)
def side_keepout(G):
    bolt = sd_zcyl(G, (3.9, 31.0, 0), 1.7, 10.0, 40.0)                       # M3 lane
    top = 26.0 - G[:, 2]                                                       # flat top: the bed face (prints upside down)
    return np.minimum(np.minimum(np.minimum(_box_sdf(G, *PLATE), _box_sdf(G, *CASE_BOX)), bolt), top)
def leg_end():
    return np.array(ROUTE["side"]["bracket_socket_bottom"], float)
def leg_dirs():
    xf, yf, zc = FOOT["foot_centre"]; E = leg_end(); out = []
    for sx in (1, -1):
        v = np.array([sx * xf, yf, zc]) - E; out.append(v / np.linalg.norm(v))
    return out
def side_hub(build=None, log=print):
    """+y side hub (the -y hub is this part mirrored), PETG, on the side plate bolt (3.9, 31). Pad z 22..26 under the
    top case's own pad (stack, bolt 4 longer); the arm runs on the plate top to its edge and drops past it (that drop
    is also the lip that stops the hub turning on its one bolt) to the strut root just off the plate (strut_route.py,
    17 Oct: nothing above z 26, so the pad top is a flat bed face and the hub prints upside down); side strut socket
    and both leg sockets at the root."""
    Bs, Rs, us = strut_dir("side")
    pad = np.array([3.9, 31.0]); E = leg_end()
    elbow = (0.6, 47.6, 24.2)                                                  # arm stays on the plate top to its edge
    caps = [((*pad, 24.0), elbow, 2.4), (elbow, tuple(Bs), 2.4),
            (tuple(Bs - us * 1.0), tuple(Bs + us * 12.0), 2.9),
            ((0.0, 47.1 + 1.0, 24.5), (0.0, 47.1 + 1.0, 19.0), 2.2)]          # lip post down the plate edge
    for v in leg_dirs():
        caps.append((tuple(E + 1.0 * v), tuple(E + (LEG_HUB_BORE[1] + 0.5) * v), 2.9))
    body = Body(caps, cyls=[((*pad, 0), 4.75, 22.0, 26.0)], keepout=side_keepout)
    if build is None: build, _ = best_build(body, log=log, name="side hub")
    field, added, bed, h, res = body.closure(build)
    m = body.mesh(field)
    m = m - bore(Bs, Bs + us * 12.0, R30)
    for v in leg_dirs():
        m = m - bore(E + LEG_HUB_BORE[0] * v, E + LEG_HUB_BORE[1] * v, R30, extra=8.0)
    m = m - cyl((*pad, 10), (*pad, 40), 1.7)                                    # M3 clearance
    return m, dict(build=build, added=added, bed=bed, height=h, residual=res)

def tail_hub(build=None, log=print):
    """tail strut root under the plate's tail end (tail_route.py), PETG; flat top on the plate underside (z 18.5)."""
    Bt, Rt, ut = strut_dir("tail")
    caps = [(tuple(Bt - ut * 1.0), tuple(Bt + ut * 12.0), 2.9),
            ((Bt[0] - 3.0, 0.0, 17.0), (Bt[0] + 9.0, 0.0, 17.0), 2.4)]
    body = Body(caps, keepout=lambda G: (18.5 - G[:, 2]))                       # nothing above the plate underside
    if build is None: build, _ = best_build(body, log=log, name="tail hub")
    field, added, bed, h, res = body.closure(build)
    m = body.mesh(field)
    m = m - bore(Bt, Bt + ut * 12.0, R30)
    return m, dict(build=build, added=added, bed=bed, height=h, residual=res)

# ---------------- floor foot (pad cut flat at its 45 deg line; 17 Oct 03:27: skid socket along x) ----------------
def leg_dir():
    xf, yf, zc = FOOT["foot_centre"]; v = leg_end() - np.array([xf, yf, zc]); return v / np.linalg.norm(v)
def foot_node(build="+z", log=print):
    """+x +y foot (front left); the other three are this part mirrored in x and / or y. TPU for AMS.
    Flat bottom = lowest point of the drone. Leg socket up to the side hub, skid socket along -x to the back foot."""
    xf, yf, zc = FOOT["foot_centre"]; pr = FOOT["pad_r"]; zs = FOOT["skid"]["z"]
    Fc = np.array([xf, yf, zc]); uL = leg_dir(); zb = FOOT["foot_bottom"]
    S0 = np.array([xf, yf, zs]); ux = np.array([-1.0, 0.0, 0.0])
    caps = [((xf, yf, zc), (xf, yf, zc), pr), (tuple(Fc + 1.0 * uL), tuple(Fc + (LEG_FOOT_BORE[1] + 0.5) * uL), 2.9),
            (tuple(S0), tuple(S0 + (SKID_BORE[1] + 0.5) * ux), 2.9)]
    body = Body(caps, keepout=lambda G: (G[:, 2] - zb))                         # floor
    field, added, bed, h, res = body.closure(build)
    m = body.mesh(field)
    m = m - bore(Fc + LEG_FOOT_BORE[0] * uL, Fc + LEG_FOOT_BORE[1] * uL, R30, extra=8.0)
    m = m - bore(S0 + SKID_BORE[0] * ux, S0 + SKID_BORE[1] * ux, R25, extra=8.0)
    return m, dict(build=build, added=added, bed=bed, height=h, residual=res)
def foot_rods():
    """4 legs (3.0) foot to hub, 2 skids (2.5) front foot to back foot; each rod end 0.5 off its bore bottom"""
    xf, yf, zc = FOOT["foot_centre"]; zs = FOOT["skid"]["z"]; u = leg_dir(); Fc = np.array([xf, yf, zc]); E = leg_end()
    a, b = Fc + (LEG_FOOT_BORE[0] + 0.5) * u, E - (LEG_HUB_BORE[0] + 0.5) * u; out = []
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        m = np.array([sx, sy, 1.0]); out.append(("leg 3.0", list(a * m), list(b * m), R30))
    xs = xf - SKID_BORE[0] - 0.5
    for sy in (1, -1):
        out.append(("skid 2.5", [xs, sy * yf, zs], [-xs, sy * yf, zs], R25))
    return out

# ---------------- rods (cut lengths, C) ----------------
def rods():
    out = []
    for k in range(4):
        Rk = face_rot(k)
        out.append(("lower 2.5", Rk @ [F, -94.5, ZL], Rk @ [F, 94.5, ZL], R25))     # through the tees (17 Oct)
        out.append(("upper 2.0", Rk @ [F, -94.5, ZU], Rk @ [F, 94.5, ZU], R20))
    for name, mir in (("side", 1), ("side", -1), ("tail", 1)):
        Bp, R, u = strut_dir(name)
        if mir == -1: Bp, R, u = Bp * [1, -1, 1], R * [1, -1, 1], u * [1, -1, 1]
        out.append(("strut 3.0", Bp + 0.5 * u, R - 3.0 * u, R30))
    return out
def pins():
    """8 wheel pins, 2.0 carbon, epoxied in the cheeks"""
    L = WW + 2 * WGAP_AX + 2 * PIN_IN; out = []
    for k in range(4):
        Rk = face_rot(k)
        for (px, py), ax in (((F, C), 1), ((C, F), 0)):
            cw = wheel_center((px, py), 0 if ax == 1 else 1); axis = np.zeros(3); axis[ax] = 1
            out.append(("pin 2.0", Rk @ (cw - axis * L / 2), Rk @ (cw + axis * L / 2), PIN_R))
    return out

if __name__ == "__main__":
    def log(s): print(s, flush=True)
    parts, info = {}, {}
    builders = [("corner_node", corner_node), ("tee_node", lambda **kw: tee_node("side", **kw)),
                ("tee_node_tail", lambda **kw: tee_node("tail", **kw)), ("side_hub", side_hub),
                ("tail_hub", tail_hub), ("foot_node", foot_node)]
    only = sys.argv[1:]
    for name, fn in builders:
        if only and name not in only: continue
        m, inf = fn(log=log); parts[name] = m; info[name] = inf
    if not only or "wheel" in only:
        parts["wheel"] = wheel(); info["wheel"] = dict(build="+z")
    for k, m in list(parts.items()):
        p = f"{OUT}/s2_cage_v2_{k}.stl"
        to_trimesh(m).export(p)
        m = to_manifold(trimesh.load(p)).simplify(0.005)
        bits = m.decompose()
        if len(bits) > 1:                                                  # drop zero-volume marching-cube slivers only
            keep = [b for b in bits if abs(b.volume()) >= 0.5]
            log(f"  {k}: dropped {len(bits) - len(keep)} sliver(s) under 0.5 mm3"); m = Manifold.batch_boolean(keep, manifold3d.OpType.Add) if len(keep) > 1 else keep[0]
        t = to_trimesh(m); t.export(p)
        if not trimesh.load(p).is_watertight:                               # marching-cube pinch: merge closer, rebuild
            t2 = trimesh.load(p); t2.merge_vertices(digits_vertex=3); m = to_manifold(t2).simplify(0.01); to_trimesh(m).export(p)
        assert trimesh.load(p).is_watertight, k
        parts[k] = m
        log(f"{k}: vol {m.volume():.1f} mm3, bodies {len(m.decompose())}, genus {m.genus()}, build {info[k].get('build')}")
    prev = {}
    pj = f"{OUT}/cage_layout.json"
    if os.path.exists(pj): prev = json.load(open(pj)).get("info", {})
    prev.update(info)
    json.dump(dict(info=prev, rods=[(n, list(map(float, a)), list(map(float, b)), r) for n, a, b, r in rods() + foot_rods() + pins()]),
              open(pj, "w"))
