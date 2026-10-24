"""Scout v3 layout v2 = the Scout v3 system pass (24 Oct 2026).

Spark 24 Oct: "this design seems very flimsy ... as structurally secure while light as possible ... a full cage, in all
contact points the cage should be touching first" (CLAUDE.md "Cage first, strong and light"), and the rule "System first,
then cut" (docs/generative.md The rule, 0). v1 had a stick cage drawn by hand; v2 lays the cage out by optimisation on the
assembled drone and checks it there:

1. Assembled drone: frame, props, FC, camera, PR 44's receiver interface and cartridge v4 (the control session's latch
   standard from now on; its numbers are DEFAULT until "System box swap" lands), Radxa and flow from PR 44, UBEC and XR2,
   the box's keep-outs (head, carriage, cartridge path), the funnel planes at +-5 deg yaw / +-2 deg tilt, the hook latch
   plungers, the belly post with its landing cones.
2. Cage ground structure: candidate 3.0 carbon rods between candidate nodes (rings, crown, glove down the faces, keel
   under the pack, joints into the core), every rod that breaks a keep-out dropped. Mass-minimum layout over the system
   load cases (hits, top, landing, swap pull, latch hold, handling) by linear programming (plastic truss design: each rod
   carries at most its joint / buckling capacity). Rings are mandatory (prop guard).
3. Frame model (rigid printed nodes, Euler beams) of the chosen cage on the core: every case, every rod's index, the
   deflection at the load, the joint reactions into the core (the requirements for the receiver, plate hubs and nodes).
4. Contact-first: support distance over 2562 directions, cage vs everything else; the cage must lead by >= 3 mm.
5. Pogo pad height is searched with all of the above (the post's pins sweep 45 deg cones the cage must stay out of).

python3 nox/scout/v3/layout_v2.py [--novideo]       (~10 min)
Drone frame = seat frame (PR 44): x nose, y left, z up, plate underside 18.5. Numbers: nox/dimensions.md "System Scout v3".
"""
import io, json, math, os, sys, itertools, time
import numpy as np, trimesh
from scipy.sparse import lil_matrix, csr_matrix, vstack as svstack, hstack as shstack, identity

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from motion import Scene
from topo.part import man

OUT = os.environ.get("LAYOUT_OUT") or os.path.join(ROOT, "runs", "scout_layout", "v3")   # v3: printed short members + rod reduction
LATCH_DIR = os.path.join(ROOT, "runs", "scout_layout", "v2")                  # inputs (latch, pr44) always from the v2 run folder
ESO_SEED = int(os.environ.get("ESO_SEED", "1"))         # multi-start: seed 1 had the fewest pieces of 4 (count objective, 24 Oct)
VIDEO = "--novideo" not in sys.argv
S2 = math.sqrt(2.0)
LOG = io.StringIO()


def say(*a):
    print(*a, flush=True); print(*a, file=LOG)


# ================================================================ numbers (tags in dimensions.md "System Scout v3")
PLATE_BOT, PLATE_TOP, PLATE_TAIL = 18.5, 22.0, -50.0
MOTOR, PROP_R, PROP_Z, SWEEP = 54.1, 44.45, 41.2, 4.0
BELL_R, BELL_H = 12.4, 15.0
FACE_K, FACE_Z0, FACE_Z1 = 84.8, -0.8, 75.1
RING_LO = (112 - FACE_K) + 4.0 * S2          # 32.86: lower-ring rod centre 4.0 off the face at the outline (seat = drone)
RING_HI = 49.2
TOP_Z = float(os.environ.get("TOP_Z", "72.0"))                                # crown nodes (dome over the props; centre strips carry posts to the plate)
BOT_Z = -47.5                                # keel nodes: 3.0 under the cartridge bottom (-44.5)
ROD_R, ROD_A, ROD_I = 1.5, 7.069, 3.976      # 3.0 solid carbon (C)
NROD = int(os.environ.get("NROD", "1"))       # rods per member (a pair = 2 x 3.0 side by side in shared sockets, test only)
ROD_A, ROD_I = ROD_A * NROD, ROD_I * NROD
E_ROD, G_ROD = 100000.0, 38000.0             # MPa (E, pultruded; D21 bend test replaces)
SIG_ROD = 750.0                              # MPa, bending + axial allowable (1500 E / 2)
# carbon where its length earns it (Spark 24 Oct, CLAUDE.md, PR 91): a straight run (one rod through its nodes) shorter than
# L_CARBON is a printed PETG strut, part of the node (or of the mount at a core joint), no sockets. Break-even: 2 sockets 1.14 g = (0.0457 - 0.011) g/mm x L -> 33 mm (C, DEFAULT)
L_CARBON = float(os.environ.get("L_CARBON", "33.0"))
PET_W, PET_A, PET_I, PET_J = 6.0, 36.0, 108.0, 182.5     # 6 x 6 PETG strut (C; posts >= 6 x 6, CLAUDE.md)
E_PET, G_PET, SIG_PET, PET_GPM = 2000.0, 740.0, 25.0, 0.0457   # MPa (E), MPa (E), MPa allowable (E, 50 / 2), g/mm (C, 1.27 g/cm3)
PET_R = PET_W / 2                            # strut half width: the node turns it flat to the nearest keep-out (C)


NOPRINT = set()                              # short members where a 6 x 6 strut misses a keep-out: carbon only as part of a long straight rod


def _mkey(p0, p1):
    return frozenset((tuple(np.round(p0, 2)), tuple(np.round(p1, 2))))


COLLIN, LEN = {}, {}                         # static: member -> collinear partner members through printed nodes; member -> length


def set_collinear(N, kind, members):
    """precompute, once per run, which candidate members continue each other straight through a printed node"""
    COLLIN.clear(); LEN.clear(); inc = {}
    for m in members:
        a, b = m; LEN[m] = float(np.linalg.norm(N[b] - N[a]))
        inc.setdefault(a, []).append((m, (N[b] - N[a]) / LEN[m])); inc.setdefault(b, []).append((m, (N[a] - N[b]) / LEN[m]))
    for i, lst in inc.items():
        if kind[i] == "C": continue
        for x in range(len(lst)):
            for y in range(x + 1, len(lst)):
                if np.dot(lst[x][1], lst[y][1]) < -0.998:
                    COLLIN.setdefault(lst[x][0], []).append(lst[y][0]); COLLIN.setdefault(lst[y][0], []).append(lst[x][0])


def _runs(N, kind, members):
    """straight runs: members meeting collinear at a printed node are one rod through it; -> (root per member, run length per root)"""
    if LEN and all(m in LEN for m in members):                              # fast path (set_collinear)
        idx = {m: e for e, m in enumerate(members)}; par = list(range(len(members)))
        def g(x):
            while par[x] != x: par[x] = par[par[x]]; x = par[x]
            return x
        for e, m in enumerate(members):
            for q in COLLIN.get(m, ()):
                j = idx.get(q)
                if j is not None: par[g(e)] = g(j)
        root = [g(e) for e in range(len(members))]; tot = {}
        for e, r_ in enumerate(root): tot[r_] = tot.get(r_, 0.0) + LEN[members[e]]
        return root, tot
    Ls = [float(np.linalg.norm(N[b] - N[a])) for a, b in members]
    par = list(range(len(members)))
    def f(x):
        while par[x] != x: par[x] = par[par[x]]; x = par[x]
        return x
    inc = {}
    for e, (a, b) in enumerate(members):
        inc.setdefault(a, []).append((e, (N[b] - N[a]) / Ls[e])); inc.setdefault(b, []).append((e, (N[a] - N[b]) / Ls[e]))
    for i, lst in inc.items():
        if kind[i] == "C": continue
        free = list(range(len(lst)))
        while free:
            k = free.pop(0); m = next((q for q in free if np.dot(lst[k][1], lst[q][1]) < -0.998), None)
            if m is not None: free.remove(m); par[f(lst[k][0])] = f(lst[m][0])
    root = [f(e) for e in range(len(members))]; tot = {}
    for e, r_ in enumerate(root): tot[r_] = tot.get(r_, 0.0) + Ls[e]
    return root, tot


def member_types(N, kind, members):
    """{member: "rod" | "pet" | "bad"} (Spark 24 Oct, carbon where its length earns it): carbon when its straight run is >= L_CARBON,
    else a 6 x 6 PETG strut in its node; "bad" = a short run where no strut fits (dropped)"""
    root, tot = _runs(N, kind, members); out = {}
    for e, m in enumerate(members):
        out[m] = "rod" if tot[root[e]] >= L_CARBON else ("bad" if _mkey(N[m[0]], N[m[1]]) in NOPRINT else "pet")
    return out


def drop_bad(N, kind, members):
    cur = list(members)
    while True:
        T = member_types(N, kind, cur); bad = [m for m in cur if T[m] == "bad"]
        if not bad: return cur
        cur = [m for m in cur if T[m] != "bad"]


CAP_JOINT = 150.0 * int(os.environ.get("NROD", "1"))                            # N, epoxied socket pull-out design value (E; socket coupon pull, item 23)
NODE_G, SOCKET_G, ROD_GPM = 1.5, 0.57, 0.011  # printed node body, per socket (cage v2 doc, M), rod g/mm (G)
CG = np.array([0.0, 0.0, 4.2])
CG_TOL = 3.0                                 # mm, balance (dimensions L)
CONTACT_MARGIN = 3.0                         # cage ahead of everything else (CLAUDE.md, DEFAULT)
KEEP = 1.0                                   # every keep-out, plus the rod radius


# ================================================================ helpers
def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b


def cyl(r, z0, z1, x=0.0, y=0.0, n=40):
    c = trimesh.creation.cylinder(r, z1 - z0, sections=n); c.apply_translation([x, y, (z0 + z1) / 2]); return c


def rod(p0, p1, r=ROD_R, n=10):
    return trimesh.creation.cylinder(r, segment=[p0, p1], sections=n)


def strut(p0, p1, w=PET_W):
    """printed PETG strut, w x w square section along p0 -> p1"""
    d = np.asarray(p1, float) - np.asarray(p0, float); L = np.linalg.norm(d); ex = d / L
    ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0, 0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    b = trimesh.creation.box([L, w, w]); T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = ex, ey, ez; T[:3, 3] = (np.asarray(p0) + np.asarray(p1)) / 2
    return b.apply_transform(T)


def member_mesh(p0, p1, typ):
    return rod(p0, p1) if typ == "rod" else strut(p0, p1)


def rotz(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def union(ms):
    out = man(ms[0])
    for m in ms[1:]:
        out = out + man(m)
    mm = out.to_mesh(); return trimesh.Trimesh(mm.vert_properties[:, :3], mm.tri_verts)


def pts(m, n=3000, seed=1):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=seed)[0]])


def in_box(p, lo, hi, pad=0.0):
    return np.all((p >= np.array(lo) - pad) & (p <= np.array(hi) + pad), axis=1)


def box_dist(p, lo, hi):
    d = np.maximum(np.maximum(np.array(lo) - p, p - np.array(hi)), 0); return np.sqrt((d ** 2).sum(1))


# ================================================================ assembled drone (fixed)
def plate():
    body = union([box(PLATE_TAIL, 51, -30, 30, PLATE_BOT, PLATE_TOP), box(-12, 20, -47, 47, PLATE_BOT, PLATE_TOP)])
    arms = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        a = box(0, MOTOR * S2, -7, 7, PLATE_BOT, PLATE_TOP); a.apply_transform(rotz(math.degrees(math.atan2(sy, sx))))
        arms += [a, cyl(11, PLATE_BOT, PLATE_TOP, sx * MOTOR, sy * MOTOR)]
    b = man(union([body] + arms)) - man(box(PLATE_TAIL - 1, PLATE_TAIL + 10, -9, 9, PLATE_BOT - 1, PLATE_TOP + 1))
    b = b.to_mesh(); return trimesh.Trimesh(b.vert_properties[:, :3], b.tri_verts)


PLATE = plate()
MOTORS_XY = [(sx * MOTOR, sy * MOTOR) for sx in (1, -1) for sy in (1, -1)]
BELLS = [cyl(BELL_R, PLATE_TOP, PLATE_TOP + BELL_H, x, y) for x, y in MOTORS_XY]
FC = box(-18, 18, -18, 18, 9.5, 17.5); FC.apply_transform(rotz(45)); FC.apply_translation([1.9, 0, 0])


def c2d(m):
    """PR 44 cartridge coords -> drone: d = (37.5 - z_c, -x_c, y_c - 44.5)"""
    T = np.array([[0, 0, -1, 37.5], [-1, 0, 0, 0], [0, 1, 0, -44.5], [0, 0, 0, 1.0]])
    m = m.copy(); m.apply_transform(T); return m


PR44 = os.path.join(LATCH_DIR, "pr44")
# battery latch standard (control session, branch part/battery-latch @ 258becf, dimensions "System box swap: latch"; interim):
# cartridge v5 93 long with the XT60 at the NOSE, receiver block v1 + lid + 2 TPU strips + XT60E1-M, posed by its T_dc
LATCH = os.path.join(LATCH_DIR, "latch")
LATCH_DX = 0.0                                 # the set stays where the latch's T_dc puts it: its ends are clipped to the funnel planes
                                               # from this catch centre (a 3.7 aft shift cut cage first to 0.83); centring is the latch's
_L = {k: trimesh.load(os.path.join(LATCH, k + ".stl")).apply_translation([LATCH_DX, 0, 0])
      for k in ("cartridge", "slider_px", "slider_nx", "pack", "cartridge_skid_closed", "cartridge_skid_lead", "xt60_plug", "receiver",
                "receiver_lid", "receiver_strips", "xt60e1m")}
IFACE = trimesh.util.concatenate([_L[k] for k in ("receiver", "receiver_lid", "receiver_strips", "xt60e1m")])   # receiver set (M)
CART = trimesh.util.concatenate([_L[k] for k in ("cartridge", "slider_px", "slider_nx", "pack", "xt60_plug")])   # swap set (M)
SKIDS = [_L["cartridge_skid_closed"], _L["cartridge_skid_lead"]]   # the latch's TPU end skids: sacrificial contacts, counted as cage
PACK_PTS = _L["pack"].bounding_box.vertices.copy()
# the Scout's own mount (partner part): receiver bosses (z 9.6) up to the plate (18.5) round the FC and the XT60 lead window,
# legs down outside the receiver plates carrying the suspension joints, the Radxa plate (-y), the flow seat and pogo carrier (+y)
MOUNT = box(-46.85 + LATCH_DX, 54.75 + LATCH_DX, -30.6, 30.6, 9.6, 18.5)
LEADS = box(37.7 + LATCH_DX, 48.8 + LATCH_DX, -9.25, 9.25, 9.6, 18.5)            # keep clear above the lid (latch row)
# legs at the nose only, >= 7.0 inside the funnel planes like the receiver (|x| <= 84.8 + z - 10.5 at their foot); the tail
# joints sit on the mount's corners (a tail leg would stand outside the cage there, pass 4)
LEGS = [box(34 + LATCH_DX, 40 + LATCH_DX, sy * 31.0 if sy > 0 else -35.0, 35.0 if sy > 0 else -31.0, -34.0, 9.6) for sy in (1, -1)]
RADXA = box(-33, 32, -33.5, -30.3, -35, -5)                                       # standing, -y side, parts outward (PR 44, E)
RADXA_STANDOFF = box(-33, 32, -30.3, -25.5, -35, -5)                              # on the mount's -y plate (1.2 off the receiver)
FLOW = box(-11, 11, 27.5, 43.5, -33.5, -25.5); FLOW_LENS = np.array([5.0, 33.5, -33.5])    # +y mid-length on the mount (DEFAULT; the
# tail spot blocked the 4th keel node's rods; lens 6 from the nose edge, SENS_FLOW_ROT unchanged)
UBEC = box(-58, -13, 10.6, 26.0, 10.0, 18.0)                                    # in the mount's tail beside the FC (DEFAULT:
# balance with the pack out, latch refit: the receiver XT60 + lid at the nose put the dry CG 3.2 forward; was the plate top)
XR2 = box(-48, -36, 10, 26, PLATE_TOP + 0.5, PLATE_TOP + 6.5)
XT_M = LEADS                                                                       # XT60E1-M rear + leads (the male itself is in IFACE)
CAM_Z = (RING_LO + RING_HI) / 2
CAM_X = 104.0                                  # lens front, 8 behind the ring line (crash stroke at the nose; DEFAULT)
CAM = union([box(CAM_X - 14, CAM_X - 12, -19, 19, CAM_Z - 19, CAM_Z + 19), box(CAM_X - 12, CAM_X, -7, 7, CAM_Z - 7, CAM_Z + 7)])
LENS = np.array([CAM_X, 0, CAM_Z])

# box keep-outs at the drone station (PR 44 K1..K3, C / E)
KO = {"cartridge path": ([-47, -23.5, -200], [58, 23.5, -42]), "head |y|<=19 to r105": ([-60, -19, -200], [77, -19 + 38, -39.5]),
      "head r<=98.5": ([-60, -50, -200], [77, 50, -46]), "carriage plate": ([77, -28, -200], [81, 28, -8.9]),
      "carriage wings": ([77, -46.5, -200], [81, 46.5, -24.9]), "rail + bar": ([81, -20, -200], [101, 20, -10.5])}


def prop_dist(p):
    d = np.full(len(p), np.inf)
    for x, y in MOTORS_XY:
        rho = np.hypot(p[:, 0] - x, p[:, 1] - y)
        d = np.minimum(d, np.hypot(np.maximum(rho - PROP_R, 0), np.maximum(np.abs(p[:, 2] - PROP_Z) - SWEEP, 0)))
    return d


def face_margin(p, yaw=5.0, tilt=2.0):
    """distance inside the funnel planes extended (max(|x|,|y|) = 84.8 + z), worst over +-yaw / +-tilt about the CG"""
    worst = np.full(len(p), np.inf)
    for a in (-yaw, 0.0, yaw):
        for tx in (-tilt, 0.0, tilt):
            for ty in (-tilt, 0.0, tilt):
                R = trimesh.transformations.euler_matrix(math.radians(tx), math.radians(ty), math.radians(a), "sxyz")[:3, :3]
                q = (p - CG) @ R.T + CG
                worst = np.minimum(worst, (FACE_K + q[:, 2] - np.maximum(np.abs(q[:, 0]), np.abs(q[:, 1]))) / S2)
    return worst


def seated_face(p):
    return (FACE_K + p[:, 2] - np.maximum(np.abs(p[:, 0]), np.abs(p[:, 1]))) / S2


def plunger_clear(p):
    best = np.full(len(p), np.inf)
    for sy in (1, -1):
        y = sy * p[:, 1]; z = p[:, 2]
        s_up = ((y - 84.0) + (z - FACE_Z0)) / S2; off = (z + FACE_K - y) / S2
        for sx in (1, -1):
            dx = np.maximum(np.abs(p[:, 0] - sx * 100.0) - 6.0, 0)
            ds = np.maximum(np.maximum(50.1 - s_up, s_up - 58.1), 0); do = np.maximum(np.maximum(-off, off - 12.0), 0)
            best = np.minimum(best, np.sqrt(dx ** 2 + ds ** 2 + do ** 2))
    return best


# ================================================================ pogo pads + belly post (PR 44 model, C / E)
PIN_FREE, POST_DROP = 8.0, 2.0
PAD_DX = 12.0


def pads(yp, zp):
    """PR 44 model (C): carrier bottom = KW11 target plane zp; pads 10 x 6 at (+-12, yp) in 1.0 recesses, face zp - 0.6;
    carrier section (21.3, zp) -> (yp + 3.5, zp) -> (yp + 7.5, zp + 4) -> (21.3, top), x +-20.5; post body under the pads"""
    top = zp + 4.0 + (yp + 7.5 - 26.0)
    sec = [(26.0, zp), (yp + 3.0, zp), (yp + 7.0, zp + 4.0), (26.0, top)]          # from the mount's +y plate (latch receiver at 24.3);
    # the 45 deg chamfer starts at the pad's outer edge (DEFAULT, +0.35 to the funnel planes; PR 44 kept a 0.5 rim)
    carrier = trimesh.Trimesh(np.array([[x, y, z] for (y, z) in sec for x in (-20.5, 20.5)])).convex_hull
    pad = [box(sx * PAD_DX - 5, sx * PAD_DX + 5, yp - 3, yp + 3, zp + 1.0 - 1.6, zp + 1.0) for sx in (-1, 1)]
    zf = zp - 0.6
    body = box(-21.0, 21.0, yp - 5.0, yp + 5.0, zf - POST_DROP - 10.0, zf - POST_DROP)
    tips = np.array([(sx * PAD_DX + dx, yp, zf + PIN_FREE) for sx in (-1, 1) for dx in (-2.0, 2.0)] + [(0, yp, zf + 3.0)])
    return pad, carrier, body, tips


def cone_gap(Q, P, rmax=40.0):
    """min (lateral - depth)/sqrt2 of drone points P below post points Q (< 0 = the post hits P in a landing slide)"""
    worst = np.inf
    for q in Q:
        lat = np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]); dep = q[2] - P[:, 2]
        m = (dep > 0) & (lat <= rmax)
        if m.any():
            worst = min(worst, float(((lat[m] - dep[m]) / S2).min()))
    return worst


# ================================================================ cage ground structure
RING_T = (0.25, 0.5, 0.75) if os.environ.get("COARSE") is None else (0.5,)   # flat nodes every 50 (more sections, Spark 24 Oct)


# keep-alive (box catch, 24 Oct, DEFAULT): spring probes in the two plain +-x funnel faces touch a pad on the lower ring; G (8):
# within 9.0 of each +-x face over y -35..+69 (+x) / -69..+35 (-x), from the face bottom up to the pad top, only the pad, its
# carrier and round rods >= 1.0 off the face. So the lower ring's x-face flats carry one node each, the pad carrier, at y +17
# (+x, B+) / -17 (-x, B-), and keep their y -50 / +50 node outside the band; the other flat nodes go (DEFAULT).
PAD_T_X = (0.25, 0.585)                       # +x flat: y -50, +17; -x flat (runs +100 -> -100): y +50, -17
RING_EDGES = set()


def ring_nodes(z, lower=False):
    n = [(112, -100), (112, 100), (100, 112), (-100, 112), (-112, 100), (-112, -100), (-100, -112), (100, -112)]
    loop = []
    for i in range(8):
        a, b = np.array(n[i], float), np.array(n[(i + 1) % 8], float)
        loop.append(a)
        if np.linalg.norm(b - a) > 100:                                       # the flats
            ts = PAD_T_X if (lower and abs(a[0]) == 112 and abs(b[0]) == 112) else RING_T
            for t in ts:
                loop.append(a + t * (b - a))
    P = [np.array([p[0], p[1], z]) for p in loop]
    for i in range(len(P)):
        RING_EDGES.add((tuple(np.round(P[i], 3)), tuple(np.round(P[(i + 1) % len(P)], 3))))
    return P


# the keep-alive pads (catch frame: probe line on each +-x face at panel z 33 = drone z 30.9, x +-115.7)
PROBE_Z = 33.0 - 2.1
def keepalive_pads():
    out = []
    for sx in (1, -1):
        u = np.array([sx, 0, 1.0]) / S2; nrm = np.array([-sx, 0, 1.0]) / S2           # up the face / into the funnel
        p0 = np.array([sx * (FACE_K + PROBE_Z), 0, PROBE_Z])
        c = p0 + nrm * (2.0 + 0.8) + u * (-2.0) + np.array([0, sx * 17.0, 0])          # face 2.0 off the plane, 1.6 thick
        b = trimesh.creation.box([12.0, 22.0, 1.6])                                    # along the slope x along y x thick
        T = np.eye(4); T[:3, 0] = u; T[:3, 1] = [0, 1, 0]; T[:3, 2] = nrm; T[:3, 3] = c
        b.apply_transform(T); out.append(b)
    return out


KA_PADS = keepalive_pads()


def candidate_nodes():
    N, kind = [], []
    L = ring_nodes(RING_LO, lower=True); U = ring_nodes(RING_HI)
    N += L; kind += ["L"] * len(L); N += U; kind += ["U"] * len(U)
    for x, y in [(0, 0), (45, 0), (-45, 0), (0, 45), (0, -45), (36, 36), (36, -36), (-36, 36), (-36, -36)]:
        N.append(np.array([x, y, TOP_Z], float)); kind.append("T")
    for y in (-24.0, 0.0, 24.0):                                              # hood over the camera board (its top is at z 60,
        N.append(np.array([100.0, y, 64.0])); kind.append("T")              # 11 over the upper ring; edge probes found it, refit)
    for sx in (1, -1):                                                        # shoulders over the prop tips: dome the corners
        for sy in (1, -1):
            N.append(np.array([sx * 92.0, sy * 92.0, 0.5 * (RING_HI + TOP_Z) + 2.0])); kind.append("T")
    for x, y in [(76, 0), (-76, 0), (0, 76), (0, -76), (66, 66), (66, -66), (-66, 66), (-66, -66)]:
        N.append(np.array([x, y, 0.0])); kind.append("G")                   # glove knees along the faces at z 0
    for x, y, z in [(20, -46, -24), (-20, -46, -24), (20, 46, -24), (-20, 46, -24), (44, 29, -32), (44, -29, -32), (-44, 29, -32),
                    (-44, -29, -32), (30, 62, -12), (30, -62, -12), (-30, 62, -12), (-30, -62, -12), (60, 40, -12), (60, -40, -12),
                    (-60, 40, -12), (-60, -40, -12)]:
        N.append(np.array([x, y, z], float)); kind.append("M")               # mid glove and end nodes on the funnel planes
    for x, y, z in [(60, 0, -14), (60, 30, -14), (60, -30, -14), (-60, 0, -14), (-60, 30, -14), (-60, -30, -14),
                    (30, -40, -32), (-30, -40, -32), (0, -40, -34), (30, 48, -30), (-30, 48, -30), (0, 48, -30)]:
        N.append(np.array([x, y, z], float)); kind.append("M")               # shields: nose / tail bottom, belly sides outside the
                                                                             # Radxa (-y) and the flow board (+y) (probes, review PR 94)
    for x, y in [(28, 28.5), (28, -28.5), (-28, 28.5), (-28, -28.5), (0, -28.5), (0, 28.5)]:
        N.append(np.array([x, y, BOT_Z])); kind.append("B")                   # keel under the pack, outside the path
    core = [(3.9, 47, 24.0), (3.9, -47, 24.0), (49, 18, 24.0), (49, -18, 24.0), (-46, 22, 24.0), (-46, -22, 24.0),
            (3.9, 47, 14.0), (3.9, -47, 14.0), (-46, 22, 14.0), (-46, -22, 14.0),
            (34, 0, 24.0), (-30, 0, 24.0), (0, 26, 24.0), (0, -26, 24.0),
            (35.0, 33, -32.0), (35.0, -33, -32.0), (35.0, 33, 4.0), (35.0, -33, 4.0),   # in the nose legs' face (x 34): 33.3 put rod ends 1.0 off the Radxa (fit 0.4, reduction pass)
           
            (50.3, 31, 14.0), (50.3, -31, 14.0), (-49.7, 31, 14.0), (-49.7, -31, 14.0)]   # moved with the set (LATCH_DX)   # nose legs and the mount's corners
    for c in core:
        N.append(np.array(c, float)); kind.append("C")                        # joints into the core (plate edge, receiver)
    return np.array(N), kind


# ================================================================ feasibility of a rod
BODIES = {"plate": PLATE, "FC": FC, "receiver": IFACE, "cartridge": CART, "mount": MOUNT, **{f"leg{i}": l for i, l in enumerate(LEGS)}, "Radxa": union([RADXA, RADXA_STANDOFF]), "flow": FLOW,
          "UBEC": UBEC, "XR2": XR2, "XT60": XT_M, "camera": CAM, **{f"bell{i}": b for i, b in enumerate(BELLS)}}


def rod_samples(p0, p1):
    n = max(6, int(np.linalg.norm(p1 - p0) / 2.0))
    t = np.linspace(0, 1, n)[:, None]; P = p0 * (1 - t) + p1 * t
    inner = P[(t[:, 0] > 0.04) & (t[:, 0] < 0.96)]                           # rod ends sit in nodes
    return inner if len(inner) else P


def rod_pad_ok(inner, padb, tips, post_pts):
    for nm, m in padb.items():
        lo, hi = m.bounds
        if (box_dist(inner, lo, hi) < KEEP + ROD_R).any(): return nm
    if cone_gap(tips, inner) < ROD_R / S2 + 0.3 or cone_gap(post_pts, inner) < ROD_R / S2 + 0.3: return "post cones"
    return None


STRICT = ("FC", "Radxa", "flow", "UBEC", "XR2", "XT60", "camera")


def rod_ok(p0, p1, kinds, B, is_ring, core_ends=(), r=ROD_R):
    inner = rod_samples(p0, p1)
    body_pts = inner
    for c in core_ends:                                                       # the hub at a core joint is its own body
        body_pts = body_pts[np.linalg.norm(body_pts - c, axis=1) > 7.0]
    if (prop_dist(inner) - r < 12.0 - 0.05).any(): return "props"           # the rings sit at 12.0 (C)
    for nm, (lo, hi) in KO.items():
        if nm == "head r<=98.5":                                               # the heads sweep r <= 98.5 about the axle
            inx = (inner[:, 0] > lo[0]) & (inner[:, 0] < hi[0])
            if (inx & (np.hypot(inner[:, 1], inner[:, 2] + 144.5) < 98.5 + KEEP + r)).any(): return nm
            continue
        if (box_dist(inner, lo, hi) < KEEP + r).any(): return nm
    for nm, m in B.items():
        lo, hi = m.bounds
        bp = np.vstack([p0, p1, inner, rod_samples(p0, p0 + 0.08 * (p1 - p0)), rod_samples(p1, p1 + 0.08 * (p0 - p1))]) if nm in STRICT else body_pts   # electronics: no rod near them, ends included
        near = box_dist(bp, lo, hi) < KEEP + r + 0.01
        if near.any():
            q = bp[near]
            if nm in ("receiver", "cartridge", "plate"):
                d = np.abs(trimesh.proximity.signed_distance(m, q))
                inside = m.contains(q)
                if (inside | (d < KEEP + r)).any(): return nm
            else:
                return nm
    above = inner[:, 2] > FACE_Z0
    if above.any() and not is_ring and (seated_face(inner[above]) < KEEP + r).any(): return "funnel seated"
    low = inner[:, 2] < 20.0
    if low.any() and (face_margin(inner[low]) < KEEP + r).any(): return "funnel planes +-5/+-2"
    if (plunger_clear(inner) < KEEP + r).any(): return "plungers"
    d = inner - FLOW_LENS; dz = -d[:, 2]
    if ((dz > 0) & (np.degrees(np.arctan2(np.hypot(d[:, 0], d[:, 1]), np.maximum(dz, 1e-6))) < 23.0 + 3.0)).any(): return "flow view"
    d = inner - LENS; fx = d[:, 0] > 0.5
    az = np.degrees(np.arctan2(np.abs(d[:, 1]), np.maximum(d[:, 0], 1e-6))); el = np.degrees(np.arctan2(np.abs(d[:, 2]), np.maximum(d[:, 0], 1e-6)))
    if (fx & (az < 45) & (el < 37)).any(): return "camera view"
    return None


# ================================================================ load cases (E): name, {node index: force}, class
MASS_KG = 0.50                                # Scout (dimensions L budget; E)
G0 = 9.81e-3                                  # N per g of mass per g of acceleration


def load_cases(N, kind, live=None):
    """dimensions.md L standard cases (DEFAULT): crash 35 g through the cage node that touches first in each of the 6 face
    and 8 corner directions; landing 6 g at the keel / wheels; handling 50 N on one member node in any direction; swap
    pull-out 50 N on two wheels; latch hold 4 x 30 N and misuse 100 N (G)."""
    cases = []
    idx = {k: [i for i, kk in enumerate(kind) if kk == k and (live is None or i in live)] for k in set(kind)}
    cage = [i for i, k in enumerate(kind) if k != "C" and (live is None or i in live)]
    crash = 1.0                                                          # unit: frame_eval scales to the design hit
    dirs = [np.array(d, float) for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for d in dirs:
        top_ = max(N[i] @ d for i in cage)
        touch = [i for i in cage if N[i] @ d >= top_ - 1.0]                 # a flat wall meets every node within 1.0 at once
        cases.append((f"crash {V_CRASH} m/s {np.round(d, 2).tolist()}", {i: -crash * d / len(touch) for i in touch}, "crash"))
    land = 6 * MASS_KG * 1000 * G0                                        # 29.4 N
    B = idx.get("B", [])
    if B:
        cases.append(("landing 6 g on the keel", {i: np.array([0, 0, land / len(B)]) for i in B}, "flight"))
        for i in sorted(B, key=lambda i: (-N[i][0], -N[i][1]))[:1] + sorted(B, key=lambda i: (N[i][0], -N[i][1]))[:1]:
            cases.append((f"landing 6 g on one keel node {tuple(N[i][:2].astype(int))}", {i: np.array([0.3 * land, 0.3 * land * np.sign(N[i][1] or 1), land])}, "flight"))
    if idx.get("T"):
        cases.append(("upside-down landing 6 g on the crown", {i: np.array([0, 0, -land / len(idx["T"])]) for i in idx["T"]}, "flight"))
    W = {tuple(np.round(N[i][:2]).astype(int)): i for i in [j for j, k in enumerate(kind) if k == "L"]}
    nrm = lambda x, y: (np.array([-np.sign(x), 0, 1.0]) if abs(x) == 112 else np.array([0, -np.sign(y), 1.0])) / S2
    for pair in (((112, 100), (-112, -100)), ((100, 112), (-100, -112))):
        cases.append((f"swap pull 50 N on wheels {pair}", {W[p]: 25 * S2 * nrm(*p) for p in pair}, "flight"))
    yw = [(100, 112), (-100, 112), (100, -112), (-100, -112)]
    down = lambda x, y: np.array([0, -np.sign(y), -1.0]) / S2
    cases.append(("latch hold 4 x 34 N", {W[p]: 34.0 * down(*p) for p in yw}, "flight"))           # G (7), box catch PR 70
    cases.append(("latch misuse 175 N on one wheel", {W[yw[0]]: 175.0 * down(*yw[0])}, "misuse"))
    # handling 50 N: one node, inward / down / up at the side flat, the nose flat, the crown, a keel node
    for ring in ("L", "U"):
        for d in (np.array([0, 1.0, 0]), np.array([1.0, 0, 0])):
            i = max(idx[ring], key=lambda i: N[i] @ d)
            cases.append((f"handling 50 N on {ring} {np.round(N[i][:2]).astype(int).tolist()}", {i: -50.0 * d}, "handling"))
            cases.append((f"handling 50 N down on {ring} {np.round(N[i][:2]).astype(int).tolist()}", {i: np.array([0, 0, -50.0])}, "handling"))
    return cases


def capacity(L):
    """rod capacity (N): tension = the epoxied socket, compression = the socket or 0.7 x Euler buckling (E)"""
    pcr = 0.7 * math.pi ** 2 * E_ROD * ROD_I / L ** 2
    return CAP_JOINT, min(CAP_JOINT, pcr)


# ================================================================ frame FE (Euler beams, rigid printed nodes, core joints fixed)
DEFL_LIM = {"crash": 1e9, "flight": 2.0, "functional": 1.5, "misuse": 3.0, "handling": 2.0}   # mm at the load (E, DEFAULT);
                                              # crash: the stroke limit of each hit direction instead (below)
# TPU 95A joints (Spark 24 Oct: 3.0 rods only, TPU for nodes / mounts; CLAUDE.md "TPU feet and nodes"). Every rod end sits in a
# TPU socket, modelled as a short TPU connector beam (socket depth) between the node centre and the rod.
TPU_E, TPU_G = 26.0, 9.0                      # MPa, 95A (E)
CONN_L = 8.0                                  # socket depth (E)
CONN_A, CONN_I = 42.0, 187.0                  # socket ring OD 7.9 / rod 3.0 section, mm2 / mm4 (C)
MOUNT_SCALE = float(os.environ.get("MOUNT_SCALE", "1.0"))   # cage-to-core TPU mounts relative to a node socket
CAP_TPU = 150.0                               # N, rod in a TPU socket, axial + shear (E; socket coupon)
V_CRASH = float(os.environ.get("V_CRASH", "1.5"))   # m/s, design hit speed (DEFAULT: 1.5x the planned PX4 cap 1.0; Elios 2 sharp-object rating; alt 2.0 needs more top / bottom / nose stroke)
M_DRONE = 0.72                                # kg (E, system pass v2)
E_CRASH = 0.5 * M_DRONE * V_CRASH ** 2 * 1000.0      # N.mm
IDX_LIM = {"crash": 1.0, "flight": 0.5, "functional": 0.5, "misuse": 1.0, "handling": 1.0}   # dimensions L index targets
STROKE = {}                                   # crash direction (tuple) -> allowed stroke, mm (contact margin - 1, set per run)
_ELC = {}


def _beam(p0, p1, E, G, A, I, J):
    key = (tuple(np.round(p0, 4)), tuple(np.round(p1, 4)), E, A)
    if key in _ELC: return _ELC[key]
    d = p1 - p0; L = np.linalg.norm(d); ex = d / L
    ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0, 0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    T = np.kron(np.eye(4), np.vstack([ex, ey, ez]))
    k = np.zeros((12, 12))
    k[0, 0] = k[6, 6] = E * A / L; k[0, 6] = k[6, 0] = -E * A / L
    k[3, 3] = k[9, 9] = G * J / L; k[3, 9] = k[9, 3] = -G * J / L
    c1, c2, c3, c4 = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
    for (v, th, sg) in ((1, 5, 1), (2, 4, -1)):
        k[v, v] = k[v + 6, v + 6] = c1; k[v, v + 6] = k[v + 6, v] = -c1
        k[v, th] = k[th, v] = sg * c2; k[v, th + 6] = k[th + 6, v] = sg * c2
        k[v + 6, th] = k[th, v + 6] = -sg * c2; k[v + 6, th + 6] = k[th + 6, v + 6] = -sg * c2
        k[th, th] = k[th + 6, th + 6] = c3; k[th, th + 6] = k[th + 6, th] = c4
    _ELC[key] = (L, T, k, T.T @ k @ T); return _ELC[key]


TPU_NODES = os.environ.get("TPU_NODES") == "1"     # test: TPU socket joints everywhere (too soft for the swap, 24 Oct sweep)
F_PLATEAU = float(os.environ.get("F_PLATEAU", "120"))   # N, preloaded TPU suspension between cage and core starts to give
                                                      # (DEFAULT: 1.25 x the 96 N vertical of the latch hold (4 x 34 N, G (7)), so the box sees it rigid)


def _members_elems(N, kind, members):
    """each rod: carbon in PETG sockets (rigid joints); with TPU_NODES a TPU socket connector at both ends (C / E)"""
    nn = len(N); X = [N]; els = []
    if not TPU_NODES:
        T = member_types(N, kind, members)
        for e, (a, b) in enumerate(members):
            if T[(a, b)] == "bad": return N, None
            if T[(a, b)] == "pet": els.append((e, "pet", a, b, _beam(N[a], N[b], E_PET, G_PET, PET_A, PET_I, PET_J)))
            else: els.append((e, "rod", a, b, _beam(N[a], N[b], E_ROD, G_ROD, ROD_A, ROD_I, 2 * ROD_I)))
        return N, els
    for e, (a, b) in enumerate(members):
        u = (N[b] - N[a]) / np.linalg.norm(N[b] - N[a])
        pa, pb = N[a] + CONN_L * u, N[b] - CONN_L * u
        ia, ib = nn + 2 * e, nn + 2 * e + 1
        X.append(np.array([pa, pb]))
        for (i, j, p, q, end) in ((a, ia, N[a], pa, a), (ib, b, pb, N[b], b)):
            s_ = MOUNT_SCALE if kind[end] == "C" else 1.0
            els.append((e, "tpu", i, j, _beam(p, q, TPU_E * s_, TPU_G * s_, CONN_A, CONN_I, 2 * CONN_I)))
        els.append((e, "rod", ia, ib, _beam(pa, pb, E_ROD, G_ROD, ROD_A, ROD_I, 2 * ROD_I)))
    return np.vstack(X), els


def frame_eval(N, kind, members, cases):
    """per case: index of every rod (carbon: bending + axial / 750 MPa, tension / socket, compression / buckling; TPU
    socket: axial + shear / 150 N), the deflection at the load, the reactions at the core joints. Crash cases carry a unit
    load: the hit stiffness k gives the peak force of the design hit, F = sqrt(2 E k) (elastic), and the case is scaled
    to it; its stroke F / k is checked against the hit direction's contact margin. None if the frame is singular."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import splu
    X, els = _members_elems(N, kind, members)
    if els is None: return None                                           # a short run with no room for a strut
    nn = len(X); rows, cols, vals = [], [], []
    for _, _, i, j, (L, T, k, kg) in els:
        dofs = np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6]
        rows.append(np.repeat(dofs, 12)); cols.append(np.tile(dofs, 12)); vals.append(kg.ravel())
    K = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(6 * nn, 6 * nn)).tocsc()
    used = sorted(set(i for e in members for i in e))
    act = np.array([d for i in list(used) + list(range(len(N), nn)) if i >= len(N) or kind[i] != "C" for d in range(6 * i, 6 * i + 6)])
    for _, F, _ in cases:
        if any(i not in used for i in F): return None
    try:
        lu = splu(K[act][:, act].tocsc())
    except RuntimeError:
        return None
    out = []
    for name, F, cls in cases:
        f = np.zeros(6 * nn)
        for i, v in F.items(): f[6 * i:6 * i + 3] += v
        u = np.zeros(6 * nn); u[act] = lu.solve(f[act])
        if not np.isfinite(u).all() or np.abs(u).max() > 1e4: return None
        scale, stroke_lim = 1.0, None
        if cls == "crash":
            f0 = sum(F.values()); d = f0 / np.linalg.norm(f0)
            ud = float(np.mean([u[6 * i:6 * i + 3] @ d for i in F]))     # mm per N of total hit force, along the hit
            if ud <= 0: return None
            kc = 1.0 / ud                                                 # cage stiffness along the hit, N/mm
            f_el = math.sqrt(2 * E_CRASH * kc)
            if f_el <= F_PLATEAU:                                         # the suspension never gives: elastic hit
                scale, stroke = f_el, f_el / kc
            else:                                                         # the suspension gives at F_PLATEAU for the rest
                scale = F_PLATEAU
                stroke = F_PLATEAU / kc + (E_CRASH - 0.5 * F_PLATEAU ** 2 / kc) / F_PLATEAU
            stroke_lim = STROKE.get(tuple(np.round(-d, 3)), 9.0)
        if cls == "misuse":                                               # the suspension may stroke: the cage sees <= F_PLATEAU
            tot = np.linalg.norm(sum(F.values()))
            if tot > F_PLATEAU: scale = F_PLATEAU / tot
        u *= scale
        idx = np.zeros(len(members)); reac = {}
        for e, typ, i, j, (L, T, k, kg) in els:
            fl = k @ (T @ np.concatenate([u[6 * i:6 * i + 6], u[6 * j:6 * j + 6]]))
            if typ == "rod":
                Nax = fl[6]; M = max(np.hypot(fl[4], fl[5]), np.hypot(fl[10], fl[11]))
                Tc, Cc = capacity(L + 2 * CONN_L)
                v = max((abs(Nax) / ROD_A + M * ROD_R / ROD_I) / SIG_ROD, Nax / Tc if Nax > 0 else -Nax / Cc)
            elif typ == "pet":                                            # printed strut: stress at the corner, buckling
                Nax = fl[6]; Ms = max(abs(fl[4]) + abs(fl[5]), abs(fl[10]) + abs(fl[11]))
                I_, c_ = PET_I, PET_W / 2
                pcr = 0.7 * math.pi ** 2 * E_PET * I_ / L ** 2
                v = max((abs(Nax) / PET_A + Ms * c_ / I_) / SIG_PET, -Nax / pcr if Nax < 0 else 0.0)
            else:
                v = np.linalg.norm(fl[6:9]) / CAP_TPU
            idx[e] = max(idx[e], v)
            for node, sl in ((i, slice(0, 6)), (j, slice(6, 12))):
                if node < len(N) and kind[node] == "C":
                    reac[node] = reac.get(node, np.zeros(6)) + (T.T @ fl)[sl]
        defl = max(np.linalg.norm(u[6 * i:6 * i + 3]) for i in F) if cls != "crash" else stroke
        out.append(dict(case=name, cls=cls, idx=idx, worst=float(idx.max()), el=int(idx.argmax()), defl=float(defl), reac=reac,
                        peak=scale if cls == "crash" else None, stroke_lim=stroke_lim))
    return out


def feasible(fe):
    return fe is not None and all(r["worst"] <= IDX_LIM[r["cls"]] and r["defl"] <= (r["stroke_lim"] if r["stroke_lim"] is not None else DEFL_LIM[r["cls"]]) for r in fe)


def cage_mass(N, kind, members):
    """rods 11 g/m + every printed node (body + a socket per rod end); a ring node a straight ring rod runs through
    with nothing else on it is no node at all (C / E)"""
    T = member_types(N, kind, members)
    g = sum((PET_GPM if T[(a, b)] == "pet" else ROD_GPM) * np.linalg.norm(N[b] - N[a]) for a, b in members)
    deg, typ = {}, {}
    for a, b in members:
        for i, j in ((a, b), (b, a)): deg.setdefault(i, []).append(j); typ[(i, j)] = T[(a, b)]
    for i, nb in deg.items():
        sock = SOCKET_G * sum(1 for j in nb if typ[(i, j)] == "rod")    # printed struts are part of the node: no socket
        if kind[i] == "C": g += sock; continue                           # sockets in the core hubs
        if len(nb) == 2 and kind[i] in "LU" and sock == 2 * SOCKET_G:
            u = [(N[j] - N[i]) / np.linalg.norm(N[j] - N[i]) for j in nb]
            if np.dot(u[0], u[1]) < -0.999: continue                         # straight through
        g += NODE_G + sock
    return g


OBJ = os.environ.get("OBJ", "count")           # ESO target: "mass" (grams) or "count" (rod pieces + printed nodes, mass as tiebreak; Spark 24 Oct)


def cage_count(N, kind, members):
    """(carbon rods, printed parts): a straight run through nodes is one rod; nodes joined by printed struts are one printed
    part, and a part strutted to a core joint is the mount's (not counted); a node with only one rod running straight
    through it is no part"""
    T = member_types(N, kind, members); root, tot = _runs(N, kind, members)
    carbon = len({root[e] for e, m in enumerate(members) if T[m] == "rod"})
    par = {}
    def f(x):
        while par.setdefault(x, x) != x: x = par[x]
        return x
    deg = {}
    for a, b in members:
        if T[(a, b)] == "pet": par[f(a)] = f(b)
        else:
            for i, j in ((a, b), (b, a)): deg.setdefault(i, []).append(j)
    nodes = set(i for e in members for i in e); comps = {}
    for i in nodes: comps.setdefault(f(i), []).append(i)
    parts = 0
    for c in comps.values():
        if any(kind[i] == "C" for i in c): continue
        if len(c) == 1:
            nb = deg.get(c[0], [])
            if len(nb) == 2 and np.dot(N[nb[0]] - N[c[0]], N[nb[1]] - N[c[0]]) / (np.linalg.norm(N[nb[0]] - N[c[0]]) * np.linalg.norm(N[nb[1]] - N[c[0]])) < -0.998: continue
        parts += 1
    return carbon, parts


def cage_cost(N, kind, members):
    if OBJ == "count":
        pc, nc = cage_count(N, kind, members)
        return pc + nc + 0.02 * cage_mass(N, kind, members)
    return cage_mass(N, kind, members)


def mirror_groups(N, members):
    """pair every rod with its mirror in y (the drone is symmetric but for the pogo side), so removals keep the cage
    symmetric; a rod whose mirror is not feasible stays alone"""
    key = {tuple(np.round(N[i], 2)): i for i in range(len(N))}
    mset = {frozenset(m): m for m in members}; done, groups = set(), []
    for m in members:
        if frozenset(m) in done: continue
        mi = tuple(key.get(tuple(np.round(N[i] * [1, -1, 1], 2)), -1) for i in m)
        g = [m]
        if -1 not in mi and frozenset(mi) in mset and frozenset(mi) != frozenset(m): g.append(mset[frozenset(mi)])
        for x in g: done.add(frozenset(x))
        groups.append(tuple(g))
    return groups


def eso(N, kind, members, mandatory, cases, contact_ok, log=print):
    """greedy mass removal on the frame model, mirror pairs together: repeatedly drop the group that saves the most mass
    per unit of use, while every case stays at index <= 1 inside its deflection limit and the contact-first margin holds"""
    cur = drop_bad(N, kind, members); fe = frame_eval(N, kind, cur, cases)
    groups = [g for g in mirror_groups(N, members) if not any(m in mandatory for m in g)]

    def violation(f):
        if f is None: return np.inf
        return sum(max(r["worst"] / IDX_LIM[r["cls"]] - 1, 0) + max(r["defl"] / (r["stroke_lim"] or DEFL_LIM[r["cls"]]) - 1, 0) for r in f)
    for _ in range(40):                                                    # start: drop rods that attract load past a limit
        if fe is not None and feasible(fe): break
        v0 = violation(fe); best_ = None
        for r in fe or []:
            if r["worst"] <= IDX_LIM[r["cls"]] and r["defl"] <= (r["stroke_lim"] or DEFL_LIM[r["cls"]]): continue
            m = cur[r["el"]]
            g = next((g for g in groups if m in g), None)
            if g is None: continue
            trial = drop_bad(N, kind, [x for x in cur if x not in g]); f2 = frame_eval(N, kind, trial, cases)
            if violation(f2) < v0 and (best_ is None or violation(f2) < best_[0]): best_ = (violation(f2), trial, f2)
        if best_ is None: break
        cur, fe = best_[1], best_[2]
    if not feasible(fe) or not contact_ok(cur):
        return cur, fe, "no feasible start (ground structure, after dropping load-attracting rods)"
    tried, steps = set(), 0
    while True:
        pos = {m: j for j, m in enumerate(cur)}
        util = np.max([r["idx"] for r in fe], axis=0)
        m0 = cage_cost(N, kind, cur)
        gain = []
        for g in groups:
            if g in tried or any(m not in pos for m in g): continue
            rest = [m for m in cur if m not in g]                          # ranking only (the trial drops short runs below)
            gain.append(((m0 - cage_cost(N, kind, rest)) / (max(util[pos[m]] for m in g) + 0.05), g))
        if not gain: break
        if ESO_SEED:                                                       # multi-start: jitter the order (seeded)
            rng_ = np.random.default_rng(ESO_SEED * 100003 + len(cur))
            gain = [(g_ * (1.0 + 0.6 * rng_.random()), grp) for g_, grp in gain]
        gain.sort(key=lambda t: -t[0]); moved = False
        for _, g in gain[:30]:
            trial = drop_bad(N, kind, [m for m in cur if m not in g])
            if any(m in mandatory for m in cur if m not in trial): tried.add(g); continue   # would orphan a ring / keel member
            fe2 = frame_eval(N, kind, trial, cases); steps += 1
            if feasible(fe2) and contact_ok(trial):
                cur, fe, moved = trial, fe2, True; tried.clear(); break
            tried.add(g)
        if not moved and all(g in tried for _, g in gain): break
    return cur, fe, f"{steps} trials"


def frame_solve(N, kind, members, cases):
    return frame_eval(N, kind, members, cases)


# ================================================================ edge / post probes (system_check), repair by adding rods
SWAP_CONE = 40.0                              # deg, the swap opening's edge-probe exemption for the receiver and cartridge (DEFAULT;
                                              # the latch uses 35; an edge at 38 deg reached the receiver's nose-bottom corner where
                                              # no rod fits; alt: a receiver nose skid, latch)


def probe_set():
    """protected parts for the probes, as meshes; the swap set and the receiver are exempt to 35 deg round the swap opening"""
    pv = [cyl(PROP_R, PROP_Z - SWEEP, PROP_Z + SWEEP, x, y, 48) for x, y in MOTORS_XY]
    prot = [("props", trimesh.util.concatenate(pv)), ("motors", trimesh.util.concatenate(BELLS)), ("FC", FC), ("Radxa", RADXA), ("flow", FLOW),
            ("UBEC", UBEC), ("XR2", XR2), ("camera", CAM), ("receiver", IFACE), ("cartridge", CART)]
    openings = [dict(part=n, axis=[0, 0, -1], half_angle_deg=SWAP_CONE, skip_probes=True) for n in ("receiver", "cartridge")]
    return prot, openings


PROBE_PROT, PROBE_OPEN = probe_set()


def cage_mesh(ms):
    T = member_types(N, kind, ms)
    rods_ = [member_mesh(N[a_], N[b_], T[(a_, b_)]) for a_, b_ in ms]
    return trimesh.util.concatenate(rods_ + KA_PADS + SKIDS)


PROBE_CFG = dict(n_dirs=300, n_roll=24, post=True)


def run_probes(ms, cfg=None):
    import system_check as sc_
    parts = [dict(name="cage", role="cage")] + [dict(name=n, role="protected") for n, _ in PROBE_PROT]
    meshes = [cage_mesh(ms)] + [m for _, m in PROBE_PROT]
    return sc_.crash_probes(parts, meshes, cfg or PROBE_CFG, 0.0, PROBE_OPEN)


def probe_deficit(pr):
    """(failing probe x part rows, sum of their margins below 0) -- 0, 0 when every probe passes"""
    rows = [r for r in pr["per_probe_part"] if not r["pass_"]]
    return len(rows), float(sum(min(r["margin_mm"], 0.0) for r in rows))


def _mirror(N, m):
    key = {tuple(np.round(N[i], 2)): i for i in range(len(N))}
    mi = tuple(key.get(tuple(np.round(N[i] * [1, -1, 1], 2)), -1) for i in m)
    return None if -1 in mi or set(mi) == set(m) else mi


def probe_repair(N, kind, chosen, members, cases, contact_ok, fe, max_add=30, n_try=4):
    """every failing probe x part row in turn (worst first): add the lightest feasible member (and its mirror) that crosses
    that probe's line ahead of the part, if it keeps every case feasible and the probe deficit drops; a row no member can
    fix is set aside (stuck) and the next one is tried"""
    cur = list(chosen); added = 0; stuck = set(); pr = run_probes(cur); d0 = probe_deficit(pr)
    mset = set(members)
    while added < max_add:
        rows = [r for r in pr["per_probe_part"] if not r["pass_"] and (r["probe"], r["part"]) not in stuck and r["contact"]]
        if not rows: break
        w = rows[0]; d = np.array(w["dir"]); e = np.array(w["axis"]); cp = np.array(w["contact"]); rr = w["r_mm"]
        u = np.cross(e, d); u /= np.linalg.norm(u)
        cands = []
        for m in members:
            if m in cur: continue
            P = np.vstack([N[m[0]] * (1 - t) + N[m[1]] * t for t in np.linspace(0, 1, 40)])
            lat = np.abs((P - cp) @ u); ahead = (P - cp) @ d
            if ((lat <= rr + 2.5) & (ahead > ROD_R + 0.5)).any(): cands.append((cage_mass(N, kind, [m]), m))
        cands.sort(key=lambda t: t[0]); fixed = False
        for _, m in cands[:n_try]:
            mm = _mirror(N, m); add = [m] + ([mm] if mm is not None and mm in mset and mm not in cur else [])
            trial = drop_bad(N, kind, cur + add)
            if any(a not in trial for a in add): continue
            fe2 = frame_eval(N, kind, trial, cases)
            if not (feasible(fe2) and contact_ok(trial)): continue
            pr2 = run_probes(trial); d2 = probe_deficit(pr2)
            if d2[1] > d0[1] + 0.3 and d2[0] <= d0[0]:
                cur, fe, pr, d0, fixed = trial, fe2, pr2, d2, True; added += len(add); break
        if not fixed: stuck.add((w["probe"], w["part"]))
    note = (f"PASS after {added} added" if pr["pass_"] else f"{d0[0]} rows still failing (deficit {d0[1]:.1f} mm), worst {pr['worst']['probe']} "
            f"{pr['worst']['part']} {pr['worst']['margin_mm']} mm; set aside: " + ", ".join(f"{a} {b}" for a, b in sorted(stuck)) + f"; {added} members added")
    return cur, fe, note


def reduce_pass(N, kind, chosen, mandatory, cases, contact_ok, fe):
    """final pass (Spark 24 Oct: fewer rods at the end): drop any mirror group whose removal keeps every case feasible,
    contact-first and the probe deficit no worse; fewest carbon rods + printed parts first, then grams"""
    cur = list(chosen); d0 = probe_deficit(run_probes(cur))
    c0 = cage_count(N, kind, cur); n_try = 0
    while True:
        groups = [g for g in mirror_groups(N, cur) if not any(m in mandatory for m in g)]
        base = sum(cage_count(N, kind, cur)); m_cur = cage_mass(N, kind, cur)
        def score(g):
            rest = drop_bad(N, kind, [m for m in cur if m not in g])
            return -(base - sum(cage_count(N, kind, rest)) + 0.02 * (m_cur - cage_mass(N, kind, rest)))
        moved = False
        for g in sorted(groups, key=score):
            trial = drop_bad(N, kind, [m for m in cur if m not in g]); n_try += 1
            if any(m in mandatory for m in cur if m not in trial): continue
            f2 = frame_eval(N, kind, trial, cases)
            if not (feasible(f2) and contact_ok(trial)): continue
            d2 = probe_deficit(run_probes(trial))
            if d2[0] > d0[0] or d2[1] < d0[1] - 0.05: continue
            cur, fe, d0, moved = trial, f2, d2, True; break
        if not moved: break
    c1 = cage_count(N, kind, cur)
    return cur, fe, f"{n_try} trials: {len(chosen)} -> {len(cur)} members, carbon rods {c0[0]} -> {c1[0]}, printed parts {c0[1]} -> {c1[1]}"


# ================================================================ contact-first
def fib_sphere(n=2562):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


def prop_points():
    out = []
    for x, y in MOTORS_XY:
        a = np.linspace(0, 2 * math.pi, 180, endpoint=False)
        for z in (PROP_Z - 1.0, PROP_Z + 1.0):
            out.append(np.stack([x + PROP_R * np.cos(a), y + PROP_R * np.sin(a), np.full_like(a, z)], 1))
    return np.vstack(out)


def contact_first(cage_pts, cage_r, others):
    D = fib_sphere()
    sc = (D @ cage_pts.T).max(1) + cage_r
    worst = (np.inf, None, None)
    for nm, p in others.items():
        so = (D @ p.T).max(1)
        k = int(np.argmin(sc - so))
        if sc[k] - so[k] < worst[0]: worst = (float(sc[k] - so[k]), nm, D[k])
    return worst


# ================================================================ wheels (R 5 at the catch points) for the contact set
def wheel_pts():
    out = []
    for sx, sy in itertools.product((1, -1), (1, -1)):
        for px, py, f in ((112 * sx, 100 * sy, "x"), (100 * sx, 112 * sy, "y")):
            c = np.array([px - (sx / S2 if f == "x" else 0), py - (sy / S2 if f == "y" else 0), RING_LO + 1 / S2])
            a = np.linspace(0, 2 * math.pi, 36, endpoint=False)
            if f == "x": ring = np.stack([5 * np.cos(a), np.zeros_like(a), 5 * np.sin(a)], 1)
            else: ring = np.stack([np.zeros_like(a), 5 * np.cos(a), 5 * np.sin(a)], 1)
            for dw in (-1.5, 1.5):
                off = np.array([0, dw, 0]) if f == "x" else np.array([dw, 0, 0])
                out.append(c + ring + off)
    return np.vstack(out)


# ================================================================ run: pad heights x (members -> LP -> frame -> contact)
t0 = time.time()
N, kind = candidate_nodes()
say(f"candidate nodes: {len(N)} ({', '.join(f'{k} {kind.count(k)}' for k in 'LUTGMBC')})")
CASES = load_cases(N, kind)
say(f"load cases: {len(CASES)}")
fixed_pts = {"props": prop_points(), "bells": np.vstack([pts(b, 400) for b in BELLS]), "plate": pts(PLATE, 3000), "FC": pts(FC, 500),
             "receiver": pts(IFACE, 6000), "cartridge": pts(CART, 4000), "Radxa": pts(RADXA, 600), "flow": pts(FLOW, 400),
             "UBEC": pts(UBEC, 400), "XR2": pts(XR2, 200), "XT60": pts(XT_M, 400), "camera": pts(CAM, 800),
             "mount": np.vstack([pts(m_, 300) for m_ in [MOUNT] + LEGS])}
BASE, why0 = [], {}
nn = len(N)
for a in range(nn):
    for b in range(a + 1, nn):
        if kind[a] == "C" and kind[b] == "C": continue
        L = np.linalg.norm(N[b] - N[a])
        if L < 8 or L > 160: continue
        ring = (tuple(np.round(N[a], 3)), tuple(np.round(N[b], 3))) in RING_EDGES or (tuple(np.round(N[b], 3)), tuple(np.round(N[a], 3))) in RING_EDGES
        r = rod_ok(N[a], N[b], kind, BODIES, ring, [N[i] for i in (a, b) if kind[i] == "C"], PET_R if L < L_CARBON else ROD_R)
        if r is not None and L < L_CARBON and rod_ok(N[a], N[b], kind, BODIES, ring, [N[i] for i in (a, b) if kind[i] == "C"]) is None:
            NOPRINT.add(_mkey(N[a], N[b])); r = None                   # no room for a 6 x 6 strut: carbon only inside a long straight rod
        if r is None: BASE.append((a, b, rod_samples(N[a], N[b])))
        else: why0[r] = why0.get(r, 0) + 1
say(f"rods feasible before the pads: {len(BASE)}; dropped: " + ", ".join(f"{k} {v}" for k, v in sorted(why0.items(), key=lambda kv: -kv[1])))
BELLY_POST = os.environ.get("BELLY_POST") == "1"
# Hub DEFAULT 24 Oct (Scout proposal, box catch to confirm): keep-alive through two hook latch plungers onto contact pads on the
# two latched +-y wheel nodes; seat sensing by the plunger halls; belly post, pogo pads and KW11 target dropped (BELLY_POST=1
# brings back the refit's post at (+-12, 40.6): its landing cones leave no room for the 4th keel node, cage first -5.9)
PAD_OPTIONS = [(40.6, -37.0)] if BELLY_POST else [None]
if os.environ.get('PADS'): PAD_OPTIONS = [tuple(map(float, p.split(','))) for p in os.environ['PADS'].split(';')]
runs = []
for opt in PAD_OPTIONS:
    if opt is None:
        yp = zp = None; pad, carrier, body, tips = [], None, None, np.zeros((0, 3)); post_pts = np.zeros((0, 3))
        fm, rmin = None, None; padb = {}
        say("no belly post: keep-alive by probes in the +-x funnel faces onto pads on the lower ring (box catch DEFAULT)")
    else:
        yp, zp = opt
        pp = pads(yp, zp)
        pad, carrier, body, tips = pp
        post_pts = pts(body, 400)
        fm = float(face_margin(pts(union(pad + [carrier]), 600)).min())
        rmin = float(np.hypot(body.vertices[:, 1], body.vertices[:, 2] + 144.5).min())
        drone_base = np.vstack([fixed_pts["receiver"], fixed_pts["cartridge"], fixed_pts["Radxa"], fixed_pts["flow"]])
        cg_base = min(cone_gap(tips, drone_base), cone_gap(post_pts, drone_base))
        if fm < 1.0 or rmin < 101.0 or cg_base < 0:
            say(f"pads ({yp}, {zp}): out (funnel {fm:.2f}, post r {rmin:.1f}, cones vs receiver {cg_base:.2f})"); continue
        say(f"pads ({yp}, {zp}): funnel {fm:.2f}, post r {rmin:.1f}, cones vs receiver {cg_base:.2f}")
        padb = {"pads": union(pad + [carrier]), "post": body}
    members, why = [], {}
    for a_, b_, inner in BASE:
        r = rod_pad_ok(inner, padb, tips, post_pts)
        if r is None: members.append((a_, b_))
        else: why[r] = why.get(r, 0) + 1
    # keep the part of the ground structure connected to the core; cases on live nodes only
    live = set(i for i, k in enumerate(kind) if k == "C"); grow = True
    while grow:
        grow = False
        for a_, b_ in members:
            if (a_ in live) != (b_ in live): live |= {a_, b_}; grow = True
    members = [m for m in members if m[0] in live and m[1] in live]
    set_collinear(N, kind, members)
    CASES = load_cases(N, kind, live)
    ring_members = [j for j, (a, b) in enumerate(members)
                    if (tuple(np.round(N[a], 3)), tuple(np.round(N[b], 3))) in RING_EDGES or (tuple(np.round(N[b], 3)), tuple(np.round(N[a], 3))) in RING_EDGES]
    # keel rails along both sides of the pack are mandatory too (edge probes from below found the slider unshielded, refit)
    ring_members += [j for j, (a, b) in enumerate(members) if kind[a] == kind[b] == "B" and abs(N[a][1] - N[b][1]) < 0.1
                     and np.linalg.norm(N[b] - N[a]) < 31]
    _T0 = member_types(N, kind, drop_bad(N, kind, members)); _lost = [members[j] for j in ring_members if members[j] not in _T0]
    say(f"   short members with no room for a 6 x 6 strut: {len(NOPRINT)}; mandatory ones lost with them: "
        + (", ".join(f"{np.round(N[a]).astype(int).tolist()}-{np.round(N[b]).astype(int).tolist()}" for a, b in _lost) or "none"))
    say(f"   mandatory: {len(ring_members)} ring + keel rail rods; keel rails " +
        ", ".join(f"{N[members[j][0]][:2].astype(int).tolist()}-{N[members[j][1]][:2].astype(int).tolist()}" for j in ring_members if kind[members[j][0]] == "B"))
    say(f"pads ({yp}, {zp}): {len(members)} feasible rods ({len(ring_members)} ring), dropped: " + ", ".join(f"{k} {v}" for k, v in sorted(why.items(), key=lambda kv: -kv[1])))
    others = dict(fixed_pts)
    if pad: others["pads"] = pts(union(pad + [carrier]), 600)
    WP = np.vstack([wheel_pts()] + [p_.vertices for p_ in KA_PADS + SKIDS]); DIRS = fib_sphere(1200)
    so_all = np.max(np.stack([(DIRS @ p.T).max(1) for p in others.values()]), 0)
    def cage_support(ms):
        P_ = np.vstack([np.vstack([N[a_] * (1 - t) + N[b_] * t for t in np.linspace(0, 1, 12)]) for a_, b_ in ms] + [WP])
        return (DIRS @ P_.T).max(1) + ROD_R
    base_margin = cage_support(members) - so_all                      # per direction, the full ground structure
    target = np.minimum(base_margin, CONTACT_MARGIN)
    # crash stroke per hit direction = how far the cage leads everything else there, less 1.0 (full ground structure)
    allo = np.vstack(list(others.values()))
    Pc = np.vstack([np.vstack([N[a_] * (1 - t) + N[b_] * t for t in np.linspace(0, 1, 12)]) for a_, b_ in members] + [WP])
    STROKE.clear()
    for nm_, F_, cls_ in CASES:
        if cls_ != "crash": continue
        f0 = sum(F_.values()); d = -f0 / np.linalg.norm(f0)
        # a hit from below may bring the cartridge floor onto the ground once the keel has given (the latch is designed for
        # the 35 g upright drop, J; Skydio lands on its battery, research 19 Oct): the stroke there runs to the pack (DEFAULT)
        oth = np.vstack([v for k_, v in others.items() if not (k_ in ("cartridge", "receiver") and d[2] < -0.5)] + ([PACK_PTS] if d[2] < -0.5 else []))
        STROKE[tuple(np.round(d, 3))] = max(float((Pc @ d).max() + ROD_R - (oth @ d).max()) - 1.0, 0.5)
    say("   crash stroke limits (mm): " + ", ".join(f"{list(k)}: {v:.1f}" for k, v in STROKE.items()))
    # balance (CLAUDE.md, dimensions L): whole-drone CG within 3 mm of the thrust axis, pack in and out
    fixed_items = [(165, _L["pack"].bounding_box.centroid, True), (39.4 + 4.5, _L["cartridge"].centroid, True), (54.5, IFACE.centroid, False),
                   (25, MOUNT.centroid, False),
                   (10, FC.bounding_box.centroid, False), (10, RADXA.bounding_box.centroid, False), (15, UBEC.bounding_box.centroid, False),
                   (1, XR2.bounding_box.centroid, False), (1.5, FLOW.bounding_box.centroid, False), (16, CAM.bounding_box.centroid, False),
                   (28, PLATE.centroid, False), (20, FC.bounding_box.centroid, False), (8, XT_M.bounding_box.centroid, False)] + \
                  [(19.8, b_.bounding_box.centroid, False) for b_ in BELLS]
    def cg_ok(ms):
        cm, cmom = 0.0, np.zeros(3)
        for a_, b_ in ms:
            g_ = ROD_GPM * np.linalg.norm(N[b_] - N[a_]) + 2 * SOCKET_G; cm += g_; cmom += g_ * (N[a_] + N[b_]) / 2
        for i_ in set(i for e in ms for i in e):
            if kind[i_] != "C": cm += NODE_G; cmom += NODE_G * N[i_]
        for with_swap in (True, False):
            m_, mom_ = cm, cmom.copy()
            for g_, c_, sw in fixed_items:
                if sw and not with_swap: continue
                m_ += g_; mom_ += g_ * c_
            if np.hypot(*(mom_ / m_)[:2]) > CG_TOL: return False
        return True
    contact_ok = lambda ms: bool(((cage_support(ms) - so_all) >= target - 1e-6).all()) and (cg_ok(ms) or not cg_ok(members))
    chosen, fe, note = eso(N, kind, members, set(members[j] for j in ring_members), CASES, contact_ok)
    if fe is None or not feasible(fe):
        bad = [] if fe is None else [f"{r['case']} index {r['worst']:.2f} defl {r['defl']:.1f} at rod {np.round(N[chosen[r['el']][0]]).astype(int).tolist()}-{np.round(N[chosen[r['el']][1]]).astype(int).tolist()}"
                                     for r in fe if r["worst"] > IDX_LIM[r["cls"]] or r["defl"] > (r["stroke_lim"] if r["stroke_lim"] is not None else DEFL_LIM[r["cls"]])]
        say(f"   ESO: {note}; failing cases: {bad[:6]}"); continue
    chosen, fe, pnote = probe_repair(N, kind, chosen, members, CASES, contact_ok, fe)
    say(f"   probe repair: {pnote}")
    if os.environ.get("REDUCE", "1") == "1":
        chosen, fe, rnote = reduce_pass(N, kind, chosen, set(members[j] for j in ring_members), CASES, contact_ok, fe)
        say(f"   reduction: {rnote}")
    used = sorted(set([i for e in chosen for i in e]))
    cage_pts = np.vstack([np.vstack([N[a_] * (1 - t) + N[b_] * t for t in np.linspace(0, 1, 25)]) for a_, b_ in chosen] + [N[[i for i in used if kind[i] != "C"]]])
    cage_all = np.vstack([cage_pts, wheel_pts()])
    cf = contact_first(cage_all, ROD_R, others)
    mass = cage_mass(N, kind, chosen) + 8 * 0.2                                    # + 8 wheels (M, cage v2)
    worst = max(fe, key=lambda r: r["worst"])
    say(f"   ESO {note}: {len(chosen)} rods, cage {mass:.1f} g (E); worst index {worst['worst']:.2f} "
        f"({worst['case']}); max deflection at a load {max(r['defl'] for r in fe):.2f} mm; contact-first {cf[0]:.2f} mm (worst vs {cf[1]})")
    runs.append(dict(yp=yp, zp=zp, chosen=chosen, fe=fe, cf=cf, mass=mass, members=members, pp=(pad, carrier, body, tips), fm=fm, rmin=rmin, why=why))

ok = [r for r in runs if r["cf"][0] >= CONTACT_MARGIN and feasible(r["fe"])]
best = min(ok, key=lambda r: r["mass"]) if ok else max(runs, key=lambda r: (min(r["cf"][0], CONTACT_MARGIN), -r["mass"]))
say(f"\nchosen: {'pads (%s, %s)' % (best['yp'], best['zp']) if best['yp'] is not None else 'no belly post'}{'' if ok else '  (not every rule met: best contact-first margin kept)'}")
say(f"elapsed {time.time() - t0:.0f} s")

# ================================================================ report the chosen system
chosen, fe = best["chosen"], best["fe"]
used = sorted(set([i for e in chosen for i in e]))
PIECES, PNODES = cage_count(N, kind, chosen)
TYP = member_types(N, kind, chosen); _root, _tot = _runs(N, kind, chosen)
CARB = sorted({round(_tot[_root[e]], 1) for e, m in enumerate(chosen) if TYP[m] == "rod"})          # carbon rod (straight run) lengths
NPET = sum(1 for m in chosen if TYP[m] == "pet")
say(f"members: {len(chosen) - NPET} carbon segments in {len(CARB)} distinct rod lengths (shortest rod {CARB[0]:.1f} mm, L_CARBON {L_CARBON:.0f}), "
    f"{NPET} printed 6 x 6 PETG struts (part of their nodes); short members with no room for a strut were dropped ({len(NOPRINT)} candidates)")
say(f"cage: {len(chosen)} members = {PIECES} carbon rod pieces (straight runs through a node are one rod), {PNODES} printed parts (count); {sum(np.linalg.norm(N[b] - N[a]) for a, b in chosen) / 1000:.2f} m, "
    f"{len([i for i in used if kind[i] != 'C'])} printed nodes, {len([i for i in used if kind[i] == 'C'])} joints into the core; ~{best['mass']:.0f} g (E)")
say("cases (frame model, F): worst rod index / deflection at the load")
for r in fe:
    a, b = chosen[r["el"]]
    say(f"   {r['case']:44s} {r['cls']:10s} index {r['worst']:.2f} (rod {tuple(N[a].round(0).astype(int))}-{tuple(N[b].round(0).astype(int))}), "
        f"deflection {r['defl']:.2f} mm" + (f" (stroke limit {r['stroke_lim']:.1f}), peak {r['peak']:.0f} N" if r["cls"] == "crash" else ""))
say(f"contact-first: cage ahead by {best['cf'][0]:.2f} mm at worst (vs {best['cf'][1]}, direction {np.round(best['cf'][2], 2).tolist()})")
REAC = {}
for r in fe:
    for i, f in r["reac"].items():
        F = float(np.linalg.norm(f[:3])); M = float(np.linalg.norm(f[3:]))
        if F > REAC.get(i, (0, 0, ""))[0]: REAC[i] = (F, M, r["case"])
say("joint reactions into the core (max over cases; the receiver and plate hubs carry these):")
for i, (F, M, c) in sorted(REAC.items()):
    say(f"   core joint {tuple(N[i].round(1))}: {F:.0f} N, {M:.0f} N.mm ({c})")
NODE_REQ = {}
for i in used:
    if kind[i] == "C": continue
    deg = sum(1 for e in chosen if i in e)
    NODE_REQ[i] = deg
say("printed nodes (sockets): " + ", ".join(f"{kind[i]}{tuple(N[i].round(0).astype(int))}:{d}" for i, d in sorted(NODE_REQ.items())))

# mass and CG of the whole drone (E)
PACK_BOX = box(-35, 35, -16, 16, -44.5 + 14.4, -44.5 + 14.4 + 33)          # pack centred on the tab line (x 0), PR 44 cavity
items = [("pack + plug", 165, _L["pack"]), ("cartridge v5 + sliders (latch, M)", 39.4, _L["cartridge"]), ("receiver v1 + lid + strips + XT60E1-M (latch, M / E)", 54.5, IFACE),
         ("mount + legs (E)", 25, MOUNT), ("FC", 10, FC),
         ("Radxa", 10, RADXA), ("UBEC", 15, UBEC), ("XR2", 1, XR2), ("flow", 1.5, FLOW), ("camera + mount", 16, CAM), ("plate", 28, PLATE),
         ("wires", 20, FC), ("XT60E1-M", 8, XT_M)] + [(f"motor {i}", 19.8, b) for i, b in enumerate(BELLS)]
tot, mom = 0.0, np.zeros(3)
for _, g_, m in items:
    tot += g_; mom += g_ * m.bounding_box.centroid
cage_cent = np.mean([(N[a] + N[b]) / 2 for a, b in chosen], 0)
tot += best["mass"]; mom += best["mass"] * cage_cent
cg = mom / tot
say(f"drone ~{tot:.0f} g (E), CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f})")
# G (8) band check: printed nodes within 9.0 of a +-x face plane over its y band, from the face bottom (panel z 12.8 = drone
# 10.7) to the pad top, other than the two pad carriers
pad_top = max(p_.bounds[1][2] for p_ in KA_PADS)
bad8 = []
for i in used:
    if kind[i] == "C": continue
    x_, y_, z_ = N[i]
    for sx, ylo, yhi in ((1, -35, 69), (-1, -69, 35)):
        dface = (FACE_K + z_ - sx * x_) / S2
        if sx * x_ > 0 and dface <= 9.0 and ylo <= y_ <= yhi and 10.7 <= z_ <= pad_top and not (abs(abs(y_) - 17) < 0.5 and kind[i] == "L"):
            bad8.append(tuple(np.round(N[i], 1)))
say(f"G (8) keep-alive band: {'clear' if not bad8 else 'nodes in it: ' + str(bad8)}; pads {[np.round(p_.bounds, 1).tolist() for p_ in KA_PADS]}")

R = dict(version="v3 printed short members, fewest pieces", date="2026-10-24T23:59", status="DONE", mass_g=round(tot), mass_tag="E", worst_index=round(max(r["worst"] for r in fe), 2),
         worst_case=max(fe, key=lambda r: r["worst"])["case"], note="system pass: cage by LP + frame model, contact-first",
         pads=(dict(y=best["yp"], z=best["zp"], x=[-PAD_DX, PAD_DX]) if best["yp"] is not None else None), belly_post=BELLY_POST,
         keepalive="latch plungers onto the two latched +-y wheel nodes (hub DEFAULT)" if not BELLY_POST else "belly post", cage_mass=best["mass"], contact_first=best["cf"][0],
         contact_vs=best["cf"][1], pieces=PIECES, printed_nodes=PNODES, carbon_members=len(chosen) - NPET, printed_struts=NPET, shortest_carbon=CARB[0], types=[TYP[m] for m in chosen], l_carbon=L_CARBON, obj=OBJ, eso_seed=ESO_SEED, rods=[[N[a].tolist(), N[b].tolist()] for a, b in chosen],
         cases=[dict(case=r["case"], cls=r["cls"], index=r["worst"], defl=r["defl"]) for r in fe],
         core_reactions=[dict(at=N[i].tolist(), F=F, M=M, case=c) for i, (F, M, c) in REAC.items()],
         options=[dict(y=r["yp"], z=r["zp"], mass=r["mass"], worst=max(x["worst"] for x in r["fe"]), contact=r["cf"][0], vs=r["cf"][1]) for r in runs],
         cg=cg.tolist())
os.makedirs(OUT, exist_ok=True)
json.dump(R, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))

# ================================================================ motion artifact
pad, carrier, body, tips = best["pp"]
s = Scene("Scout v3 system v2", "cage laid out by optimisation on the whole drone; the cage touches first in every direction")
P = lambda n, m, grp, col, pr, note="": s.part(n, m, group=grp, color=col, printed=pr, note=note)
bands = {"rods": [rod(N[a], N[b]) for a, b in chosen if TYP[(a, b)] == "rod"], "struts": [strut(N[a], N[b]) for a, b in chosen if TYP[(a, b)] == "pet"]}
P("plate", PLATE, "frame", "#3a3a3a", False)
P("motors", trimesh.util.concatenate(BELLS), "frame", "#b0b0b0", False)
pv = []
for i, (x, y) in enumerate(MOTORS_XY):
    ring = man(cyl(PROP_R, PROP_Z - 0.4, PROP_Z + 0.4, x, y, 72)) - man(cyl(PROP_R - 2.0, PROP_Z - 1, PROP_Z + 1, x, y, 72))
    ring = ring.to_mesh(); pv.append(trimesh.Trimesh(ring.vert_properties[:, :3], ring.tri_verts))
    bl = box(-PROP_R, PROP_R, -6, 6, PROP_Z - 1, PROP_Z + 1); bl.apply_transform(rotz(30 + 45 * i)); bl.apply_translation([x, y, 0]); pv.append(bl)
P("props (swept)", trimesh.util.concatenate(pv), "frame", "#d9a400", False)
P("FC", FC, "electronics", "#1b5e20", False)
P("Radxa", RADXA, "electronics", "#2e7d32", False)
P("flow", FLOW, "electronics", "#2d5a2d", False)
P("UBEC", UBEC, "electronics", "#444488", False)
P("XR2", XR2, "electronics", "#6a1b9a", False)
P("camera", CAM, "electronics", "#2d5a2d", False)
P("receiver v1 (latch standard)", IFACE, "receiver", "#8899aa", True, "control session, part/battery-latch 258becf")
P("mount + legs (Scout)", union([MOUNT] + LEGS), "receiver", "#9aa8b8", True, "envelope, own topology next")
if pad: P("pogo pads + carrier", union(pad + [carrier]), "receiver", "#c9a227", True)
P("cage rods", trimesh.util.concatenate(bands["rods"]), "cage", "#202020", False, "3.0 carbon, every one >= L_CARBON long")
if bands["struts"]: P("cage printed struts", trimesh.util.concatenate(bands["struts"]), "cage", "#aab4c0", True, "6 x 6 PETG, part of their nodes (short members)")
nodes_m = [trimesh.creation.icosphere(1, 4.0).apply_translation(N[i]) for i in used if kind[i] != "C"]
P("cage nodes", trimesh.util.concatenate(nodes_m), "cage", "#aab4c0", True, "printed PETG, topology per node next")
whe = []
for sx, sy in itertools.product((1, -1), (1, -1)):
    for px, py, f in ((112 * sx, 100 * sy, "x"), (100 * sx, 112 * sy, "y")):
        w = trimesh.creation.cylinder(5.0, 3.0, sections=32)
        w.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0] if f == "x" else [0, 1, 0]))
        w.apply_translation([px - (sx / S2 if f == "x" else 0), py - (sy / S2 if f == "y" else 0), RING_LO + 1 / S2]); whe.append(w)
P("catch wheels", trimesh.util.concatenate(whe), "cage", "#cc6644", True)
P("keep-alive pads (B+ nose, B- tail)", trimesh.util.concatenate(KA_PADS), "cage", "#c9a227", False, "box catch probes in the +-x faces")
P("cartridge v5 + pack (latch)", CART, "pack", "#7d8a99", True)
if body is not None: P("belly post (box)", body, "box", "#e07a5f", False, "PR 44 model")
fun = []
for ax in (0, 1):
    for sg in (1, -1):
        pl = []
        for z, half in ((FACE_Z0, FACE_K + FACE_Z0), (FACE_Z1, FACE_K + FACE_Z1)):
            for lat in (-half, half):
                for off in (0.0, 1.6 * S2):
                    p = [0.0, 0.0, z]; p[ax] = sg * (half + off); p[1 - ax] = lat; pl.append(p)
        fun.append(trimesh.Trimesh(np.array(pl)).convex_hull)
P("funnel faces", trimesh.util.concatenate(fun), "box", "#c8d6e5", False)
drone_names = [p["name"] for p in s.parts if p["group"] not in ("box", "pack")]


def smooth(t, a, b):
    u = min(max((t - a) / (b - a), 0.0), 1.0); return u * u * (3 - 2 * u)


def pose(t):
    dz = 140 * (1 - smooth(t, 0, 3)) + 160 * smooth(t, 9, 12); dx = 30 * (1 - smooth(t, 0, 3))
    Td = np.eye(4); Td[:3, 3] = [dx, 0, dz]
    Tp = Td.copy(); Tp[2, 3] += -70 * (smooth(t, 4, 5.5) - smooth(t, 6.5, 8))
    out = {n: Td for n in drone_names}; out["cartridge v5 + pack (latch)"] = Tp
    return out


s.timeline(pose, duration_s=12, fps=30)
s.view("iso", (330, -380, 260), (0, 0, 0)); s.view("side", (0, -560, 10), (0, 0, 0)); s.view("front", (560, 0, 10), (0, 0, 0))
s.view("top", (0, 0, 620), (0, 0, 0)); s.view("under", (180, -220, -300), (0, 0, -20))
s.write(os.path.join(OUT, "motion.html"))
if VIDEO:
    try:
        s.video(os.path.join(OUT, "motion.mp4"))
    except Exception as e:
        say("video skipped:", str(e)[:200])
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
print("wrote", OUT)
