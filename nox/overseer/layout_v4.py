"""Overseer layout v4 (rough design, 26 Oct): layout v2 with its cage replaced by a triangulated (geodesic-pattern) shell
formed to the drone. Spark 26 Oct: "it needs to follow the format we had initially, with the lidar towards the nose and
the arms compensation and the weight balance, also the cage needs to be in this geometric shape but not necessarily be a
sphere, it needs to form itself to the drone and try to be as compact as possible, probably closer to a disk"; "think of
it as adding the triangle shapes to the original drone layout we had designed, replacing that cage only".

Every v2 part keeps its v2 pose (L2 at the nose axis 15 deg below +x, front camera, hub, plates, belly sled, pods, feet,
cartridge 30 aft), except the belly, which comes up under the plates (Spark 26 Oct: the battery as close to the body as we
can, funnel disregarded): pack top z -7 as layout v0; the v2 cage (guard rings, spokes, V, wheel ring, corner nodes, struts) is replaced by the shell. The
funnel is redesigned round the shell later (Spark 26 Oct), so the wheels go with the v2 cage. The motor square moves onto
the CG again (v2's balance) because the shell's mass sits differently.

Shell: the convex hull of every part with its margin (prop disks 12 off, the rest 3 off the tube surface, the belly
clipped at the opening plane) is covered with ~(240.0 / FREQ) nodes by farthest-point sampling, relaxed to even cells, then pushed
out node by node until every face plane holds the hull: a triangulated convex shell as tight as its cells allow. One
bottom opening: a ring of nodes on an ellipse round the swap zone (receiver + SPARE a side), 3 under the cartridge.

python3 nox/overseer/layout_v4.py [out_dir]      (~2 min; V4_FREQ=3 for the smaller-cell alternative)
Drone frame: x forward, y left, z up; z 0 = bottom plate top; plate centre x 0. Numbers: `nox/dimensions.md` O / L / G.
"""
import math, os, sys, io, json
import numpy as np, trimesh
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import assembly_seed as S

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_layout", "v4")
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


L = S._layout_v2()                       # v2 definitions (v2 itself is not edited)
box, cyl, rod, pts, union, overlap = L["box"], L["cyl"], L["rod"], L["pts"], L["union"], L["overlap"]
A, HALF, PROP_R, PROP_Z, BLADE, ROOT_H = L["A"], L["HALF"], L["PROP_R"], L["PROP_Z"], L["BLADE"], L["ROOT_H"]
TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX = (L[k] for k in ("TOP", "BOTTOM", "FC", "DECK", "PI", "PDB", "UBEC", "BUCK", "RX"))

# ---------------------------------------------------------------- numbers
DC, ZB = 30.0, 51.0                      # v2's chosen case: catch 30 aft, L2 base z 51
PACK_AXIS = os.environ.get("V4_PACK", "aft") == "axis"   # axis: the pack bay follows the thrust centre (pack out balanced too)
WZ = -12.0                               # belly as close to the body as it goes (Spark 26 Oct: "the battery ... as close to the drone
                                         # body as we can, disregard the funnel"): v2's set_wheel(-12) = v0's pack top z -7 (5 for the
                                         # latch block, E); v2 hung it at -33 only for the funnel's wheel height
L["set_wheel"](WZ)
FREQ = int(os.environ.get("V4_FREQ", 2))  # geodesic frequency (Spark 26 Oct: the larger cells; 3 the alternative)
START = float(os.environ.get("V4_START", 0.85))  # start: the geodesic on an ellipsoid of the hull's half extents x START (0 = cast
                                                  # onto the hull itself: tighter, but the nose camera peak tents the top)
SHELL_R = 3.0                            # 6 mm carbon tube (Spark 26 Oct), 6/4 DEFAULT
SHELL_GPM = 1.55e-3 * math.pi / 4 * (6.0 ** 2 - 4.0 ** 2)        # 24.3 g/m (E)
ROD_R, ROD_GPM = 1.5, 11.0e-3            # links: 3.0 solid (G Carbon rods), 11.0 g/m (C)
NODE_R, NODE_G = 10.15, 4.5              # printed node round 6.3 sockets, Ø20 (E); 4.5 g (E)
PROP_GAP, CAGE_GAP = 12.0, 3.0
M_PROP, M_BODY = PROP_GAP + SHELL_R + 1.0, CAGE_GAP + SHELL_R + 1.0   # hull margins to the tube axis (+1 for facet sampling)
SPARE = 30.0                             # room a side round the receiver at the opening (DEFAULT, Spark 26 Oct)
REC = (179.6, 76.2)                      # Overseer receiver v2 footprint (M)
ZONE = (REC[0] / 2 + SPARE, REC[1] / 2 + SPARE)
LATCH_G = 338.5                          # Overseer latch set (M, battery latch v4); v2 counted the cartridge at 4 % fill
BREAK_EVEN = 33.0


def fib(n):
    i = np.arange(n) + 0.5; z = 1 - 2 * i / n; r = np.sqrt(1 - z * z); a = np.pi * (3 - 5 ** 0.5) * i
    return np.stack([r * np.cos(a), r * np.sin(a), z], 1)


DIRS = fib(64)


def geodesic(nu):
    """class-I icosahedral geodesic, unit sphere, a vertex at +z and the xz plane a mirror plane"""
    p = (1 + 5 ** 0.5) / 2
    V = np.array([(-1, p, 0), (1, p, 0), (-1, -p, 0), (1, -p, 0), (0, -1, p), (0, 1, p), (0, -1, -p), (0, 1, -p),
                  (p, 0, -1), (p, 0, 1), (-p, 0, -1), (-p, 0, 1)], float)
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    V /= np.linalg.norm(V, axis=1)[:, None]
    z = V[5]; ax = np.cross(z, [0, 0, 1.0]); sn = np.linalg.norm(ax); cs = z[2]
    Kx = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]) / sn
    V = V @ (np.eye(3) + sn * Kx + (1 - cs) * Kx @ Kx).T
    up = [i for i in range(12) if 0.3 < V[i, 2] < 0.6]                  # the upper pentagon: one vertex onto azimuth 0
    t = -math.atan2(V[up[0], 1], V[up[0], 0]); Rz = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
    V = V @ Rz.T
    out, seen = [], set()
    for a, b, c in F:
        for i in range(nu + 1):
            for j in range(nu + 1 - i):
                q = (i * V[a] + j * V[b] + (nu - i - j) * V[c]) / nu; q /= np.linalg.norm(q)
                k = tuple(np.round(q, 6))
                if k not in seen: seen.add(k); out.append(q)
    return np.array(out)


def hullpts(meshes, n=1500):
    P = np.vstack([pts(m, n) for m in meshes]); return P[ConvexHull(P).vertices]


def seg_band_dist(p0, p1, mxy):
    t = np.linspace(0, 1, 120)[:, None]; q = p0 + t * (p1 - p0); best = np.inf
    for mx, my in mxy:
        dr = np.maximum(np.hypot(q[:, 0] - mx, q[:, 1] - my) - PROP_R, 0); dz = np.maximum(np.abs(q[:, 2] - PROP_Z) - BLADE, 0)
        best = min(best, float(np.hypot(dr, dz).min()))
    return best


# ---------------------------------------------------------------- the v2 drone (everything but its cage)
def drone(dm, xB=None, cam_h=None):
    if xB is None:
        g0 = L["build"](dm, DC, 330.0, ZB)
        non_cage = g0["arms"] + g0["motors"] + [TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX, g0["cart"], g0["sled"], g0["cam_pod"], g0["flow_pod"]] + g0["feet"]
        P = np.vstack([pts(m, 600) for m in non_cage] + [np.vstack([pts(p, 2000) + [0, 0, s * BLADE] for p in g0["props"] for s in (1, -1)])])
        xB = (float((P @ A).max()) + 1.0 + (-A[2]) * ZB) / A[0]          # every non-cage point 1.0 behind the L2 base plane
    if cam_h is None:                                                     # v2's search + the props out of view (CLAUDE.md 8)
        for h in np.arange(30, 200, 2.0):
            g = L["build"](dm, DC, xB, ZB, 0.0, h); clear = True
            for m in [g["l2"]] + g["props"]:
                d = pts(m, 3000) - g["cam_lens"]; zz = d @ g["cam_fwd"]
                el = np.degrees(np.arctan2(d @ g["cam_up_v"], np.maximum(zz, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
                if ((zz > 0) & (np.abs(el) < L["CAM_VFOV"] / 2 + 2.0) & (np.abs(az) < L["CAM_HFOV"] / 2 + 2.0)).any(): clear = False; break
            if clear: cam_h = h; break
    g = L["build"](dm, DC, xB, ZB, 0.0, cam_h)
    g.update(dm=dm, xB=xB, cam_h=cam_h, mxy=g["motors_xy"], keels={k: v for k, v in g["rods"].items() if k.startswith("keel")})
    g["z_o"] = g["cart"].bounds[0][2] - 3.0                            # opening plane: cartridge bottom 3 above it
    return g


# ---------------------------------------------------------------- the shell
def ring_nodes(g):
    """ellipse round the swap zone at the opening plane; grown until node surfaces and chords clear the zone"""
    cx = -DC; n = 8; k = 1.0
    zx, zy = ZONE
    while True:
        a, b = (math.sqrt(2) * zx + NODE_R) * k, (math.sqrt(2) * zy + NODE_R) * k
        t = np.linspace(0, 2 * math.pi, n, endpoint=False) + math.pi / n
        R = np.stack([cx + a * np.cos(t), b * np.sin(t), np.full(n, g["z_o"])], 1)
        ok = True
        for i in range(n):                                               # chord samples outside the zone + tube radius
            q = R[i] + np.linspace(0, 1, 50)[:, None] * (R[(i + 1) % n] - R[i])
            if ((np.abs(q[:, 0] - cx) < zx + SHELL_R + 2) & (np.abs(q[:, 1]) < zy + SHELL_R + 2)).any(): ok = False; break
        if ok: return R
        k *= 1.03


def shell(g, seed=1):
    mxy = g["mxy"]
    props = np.vstack([np.stack([mx + PROP_R * np.cos(a), my + PROP_R * np.sin(a), np.full_like(a, PROP_Z + s * BLADE)], 1)
                       for mx, my in mxy for s in (1, -1) for a in [np.linspace(0, 2 * np.pi, 72, endpoint=False)]])
    body = hullpts(g["arms"] + g["motors"] + [TOP, BOTTOM, PI, DECK, PDB, UBEC, g["l2"], g["cam"], g["cam_bracket"], g["apex"]])
    belly = hullpts([g["cart"], g["sled"], g["cam_pod"], g["flow_pod"], g["down_cam"], g["flow"]] + g["feet"])
    K = np.vstack([(props[:, None, :] + M_PROP * DIRS[None]).reshape(-1, 3), (body[:, None, :] + M_BODY * DIRS[None]).reshape(-1, 3),
                   (belly[:, None, :] + M_BODY * DIRS[None]).reshape(-1, 3)])
    K[:, 2] = np.maximum(K[:, 2], g["z_o"])                            # under the opening plane the opening rule holds
    K = np.vstack([K, K * [1, -1, 1]])                                 # mirror in y: a symmetric shell (balance, looks)
    K = K[ConvexHull(K).vertices]
    Kmesh = trimesh.convex.convex_hull(K)
    R = ring_nodes(g); nR = len(R)
    # free nodes: a class-I icosahedral geodesic (FREQ), a vertex up and a mirror plane on xz; each direction, stretched to the
    # hull's proportions (split top / bottom), is cast from the centre onto the hull, so the pattern stays geodesic and the
    # nodes start on the drone's margin surface; directions inside the opening ring are left to the ring; the convex pass
    # below then pushes nodes out only as far as their chords need
    c0 = 0.5 * (K.min(0) + K.max(0)); c0[2] = 0.5 * (g["z_o"] + K[:, 2].max())
    hx, hy = np.ptp(K[:, 0]) / 2, np.ptp(K[:, 1]) / 2; ct, cb = K[:, 2].max() - c0[2], c0[2] - g["z_o"]
    area = 4 * math.pi * ((((hx * hy) ** 1.6 + (hx * (ct + cb) / 2) ** 1.6 + (hy * (ct + cb) / 2) ** 1.6) / 3) ** (1 / 1.6))
    f = FREQ
    Ug = geodesic(f)
    D = np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1)   # ellipsoid directions
    D /= np.linalg.norm(D, axis=1)[:, None]
    eqK = ConvexHull(K).equations                                       # cast each onto the hull: nodes start on it
    den = D @ eqK[:, :3].T; num = -(c0 @ eqK[:, :3].T + eqK[:, 3])
    tK = np.where(den > 1e-9, num / np.where(den > 1e-9, den, 1), np.inf).min(1)
    Pg = (c0 + tK[:, None] * D) if START <= 0 else c0 + START * np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1)
    ex, ey = np.ptp(R[:, 0]) / 2, np.ptp(R[:, 1]) / 2
    in_open = (((Pg[:, 0] + DC) / (1.25 * ex)) ** 2 + (Pg[:, 1] / (1.25 * ey)) ** 2 < 1.0) & (Ug[:, 2] < 0)
    N = np.vstack([R, Pg[~in_open]])
    N[nR:, 2] = np.maximum(N[nR:, 2], g["z_o"] + 1.0)
    shell.freq = f
    dirs = lambda P: (P - c0) / np.linalg.norm(P - c0, axis=1)[:, None]
    topo = lambda P: ConvexHull(dirs(P)).simplices                     # triangulation by direction: every node stays a vertex
    U = dirs(N); viol = np.array([np.inf])
    # convex pass: the true hull of the nodes must hold every hull point (direction triangles can fold inward at an edge)
    for it in range(2000):
        h = ConvexHull(N)
        lost = np.setdiff1d(np.arange(len(N)), h.vertices)
        if len(lost):
            lf = lost[lost >= nR]
            if len(lf) == 0: viol = np.array([np.inf]); break           # a ring node fell inside: invalid start
            N[lf] += 3.0 * U[lf]; continue
        viol_f = (K @ h.equations[:, :3].T + h.equations[:, 3]).max(0)
        viol = viol_f
        if viol_f.max() <= 0.05: break
        step = np.zeros(len(N))
        for f, v in zip(h.simplices, viol_f):
            if v > 0.05:
                for a in f: step[a] = max(step[a], min(v, 15.0))
        step[:nR] = 0
        N += (0.7 * step)[:, None] * U
    h = ConvexHull(N)
    tris = np.array([f for f in h.simplices if not (f < nR).all()])
    used = sorted(set(tris.ravel())); remap = {o: i for i, o in enumerate(used)}
    nodes = N[used]; tris = np.array([[remap[a] for a in f] for f in tris])
    E = sorted({tuple(sorted((int(f[i]), int(f[(i + 1) % 3])))) for f in tris for i in range(3)})
    ring_ids = [remap[i] for i in range(nR) if i in remap]
    return nodes, E, tris, ring_ids, float(viol.max()), K


def attach(g, nodes, ring_ids):
    mxy = g["mxy"]; extra = {}
    cb_ = g["cart"].bounds + [[-ROD_R - 1, -ROD_R - 1, -ROD_R - 1], [ROD_R + 1, ROD_R + 1, ROD_R + 1]]
    clear_of_cart = lambda p0, p1: not (((p0 + np.linspace(0, 1, 80)[:, None] * (p1 - p0) > cb_[0]) &
                                         (p0 + np.linspace(0, 1, 80)[:, None] * (p1 - p0) < cb_[1])).all(1)).any()
    for i, ((mx, my), (r0, m1, ht)) in enumerate(zip(mxy, g["axes"])):  # 3 spokes per arm tip, 12 off the disks
        tip = np.array([mx, my, ROOT_H - ht]); n_ok = 0
        for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - tip)):
            if seg_band_dist(tip, nodes[k], mxy) - ROD_R >= PROP_GAP:
                extra[f"spoke {i + 1}.{n_ok + 1}"] = (tip, nodes[k]); n_ok += 1
                if n_ok == 3: break
    bb = BOTTOM.bounds                                                  # core ties: bottom plate underside to the shell
    for nm, q0 in (("front", (bb[1][0], 0)), ("rear", (bb[0][0], 0)), ("left", (0, bb[1][1])), ("right", (0, bb[0][1]))):
        q = np.array([q0[0], q0[1], -1.0]); n_ok = 0
        for k in sorted([k for k in range(len(nodes)) if k not in ring_ids], key=lambda k: np.linalg.norm(nodes[k] - q)):
            if seg_band_dist(q, nodes[k], mxy) - ROD_R >= PROP_GAP and clear_of_cart(q, nodes[k]):
                extra[f"core tie {nm}.{n_ok + 1}"] = (q, nodes[k]); n_ok += 1
                if n_ok == 2: break
    B = g["B"]; up_b = np.array([-A[2], 0, A[0]])                       # L2 apex lugs to the 2 nearest nodes behind its plane
    for sy in (1, -1):
        sock = B - A * 5 + up_b * 16 + np.array([0, 46.0 * sy, 0])
        for n_, k in enumerate([k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - sock))
                                if (nodes[k] - B) @ A < -1.0 - NODE_R and seg_band_dist(sock, nodes[k], mxy) - ROD_R >= PROP_GAP][:2]):
            extra[f"L2 tie {sy:+d}.{n_ + 1}"] = (sock, nodes[k])
    sb = g["sled"].bounds                                               # belly sled corners to the 2 nearest ring nodes clear of the cartridge
    for sx in (1, -1):
        for sy in (1, -1):
            cn = np.array([-DC + sx * (sb[1][0] - sb[0][0]) / 2, sy * sb[1][1], sb[0][2]])
            clear = lambda k: clear_of_cart(cn, nodes[k])
            for n_, k in enumerate([k for k in sorted(ring_ids, key=lambda k: np.linalg.norm(nodes[k] - cn)) if clear(k)][:2]):
                extra[f"bay tie {sx:+d}{sy:+d}.{n_ + 1}"] = (cn, nodes[k])
    extra.update(g["keels"])
    return extra


def masses(g, nodes, E, extra):
    it = []; add = lambda n, m, c: it.append((n, float(m), np.asarray(c, float))); cen = lambda m: m.bounding_box.centroid
    add("L2", 230, cen(g["l2"])); add("front camera", 15, cen(g["cam"])); add("down camera", 15, cen(g["down_cam"]))
    for n, gm, m in (("Pi 5", 55, PI), ("FC", 10, FC), ("PDB", 21, PDB), ("UBEC", 15, UBEC), ("buck", 6, BUCK), ("RX", 2, RX),
                     ("top plate", 23, TOP), ("bottom plate", 61, BOTTOM), ("wires", 80, FC)):
        add(n, gm, cen(m))
    add("pack", 650, cen(g["pack"])); add("flow", 2, cen(g["flow"])); add("latch set", LATCH_G, cen(g["cart"]))
    for i, m in enumerate(g["motors"]): add(f"motor + prop {i + 1}", 91, cen(m))
    for i, a in enumerate(g["arms"]): add(f"arm {i + 1}", abs(a.volume) * 1.27e-3 * 0.45 + 25, a.centroid)
    add("apex", abs(g["apex"].volume) * 1.27e-3 * 0.45, g["apex"].centroid)
    add("cam plate", abs(g["cam_plate"].volume) * 1.27e-3 * 0.5, g["cam_plate"].centroid)
    for k, (a, b) in enumerate(g["cam_rods"]): add(f"cam rod {k}", np.linalg.norm(np.subtract(b, a)) * ROD_GPM, np.add(a, b) / 2)
    add("Pi deck", abs(DECK.volume) * 1.27e-3 * 0.5, DECK.centroid)
    for nm in ("sled", "cam_pod", "flow_pod"): add(nm, abs(g[nm].volume) * 1.27e-3 * 0.3, g[nm].centroid)
    for i, f in enumerate(g["feet"]): add(f"foot {i}", abs(f.volume) * 1.21e-3 * 0.5, f.centroid)
    for i, (a, b) in enumerate(E): add(f"shell {i}", np.linalg.norm(nodes[a] - nodes[b]) * SHELL_GPM, (nodes[a] + nodes[b]) / 2)
    for k, (p, q) in extra.items(): add(k, np.linalg.norm(np.subtract(q, p)) * ROD_GPM, (np.asarray(p) + np.asarray(q)) / 2)
    for i, n in enumerate(nodes): add(f"node {i}", NODE_G, n)
    return it


def cg(it):
    m = sum(x for _, x, _ in it); return m, sum(x * c for _, x, c in it) / m


def solve(dm=52.1):
    """v2's balance: the motor square (thrust centre) onto the CG, the shell rebuilt round the drone each time"""
    global DC
    for _ in range(14):
        if PACK_AXIS: DC = -dm
        g = drone(dm)
        nodes, E, tris, ring_ids, viol, K = shell(g)
        extra = attach(g, nodes, ring_ids); it = masses(g, nodes, E, extra); tot, c = cg(it)
        say(f"   motors {dm:5.1f} ahead: CG x {c[0]:5.1f}, {tot:5.0f} g, {len(E)} tubes")
        if abs(c[0] - dm) < 0.3: break
        dm = c[0]
    return g, nodes, E, tris, ring_ids, extra, it, tot, c, viol, K


os.makedirs(OUT, exist_ok=True)
MAX_TUBE = 350.0                         # longest shell tube allowed (DEFAULT: keeps the geodesic pattern; buckling E)
STARTS = [float(x) for x in os.environ.get("V4_STARTS", "0.7,0.85,1.0,0").split(",")] if "V4_START" not in os.environ else [START]
say(f"balance (v2: motor square onto the CG; catch {DC:.0f} aft, L2 base z {ZB:.0f}; geodesic frequency {FREQ}); "
    f"start scan {STARTS} (0 = cast onto the hull), smallest shell volume among valid shells:")
best = None
for START in STARTS:
    r = solve()
    g_, nodes_, E_ = r[0], r[1], r[2]
    tl = max(np.linalg.norm(nodes_[a] - nodes_[b]) for a, b in E_); vol = ConvexHull(nodes_).volume / 1e6
    pg_ = min(seg_band_dist(nodes_[a], nodes_[b], g_["mxy"]) for a, b in E_) - SHELL_R
    ok = r[9] <= 0.05 and len(r[4]) == 8 and tl <= MAX_TUBE and pg_ >= PROP_GAP
    say(f"start {START}: {'valid' if ok else 'invalid'}: shell {vol:.1f} L, longest tube {tl:.0f}, props {pg_:.1f}, residual {r[9]:.2f}, {r[7]:.0f} g")
    if ok and (best is None or vol < best[0]): best = (vol, START, r)
_, START, (g, nodes, E, tris, ring_ids, extra, it, tot, cgv, viol, K) = best
dm = g["dm"]; mxy = g["mxy"]

# ---------------------------------------------------------------- checks
say(f"\nchosen: start {START} (smallest valid shell); motor square {dm:.1f} ahead of the plate centre; motors ({mxy[0][0]:.1f}, ±{HALF}) and ({mxy[1][0]:.1f}, ±{HALF}); "
    f"front arms r {math.hypot(*mxy[0]):.0f}, rear r {math.hypot(*mxy[1]):.0f}")
say(f"L2: base ({g['xB']:.1f}, 0, {ZB:.0f}), axis 15 deg below +x (Spark 24 Oct); front camera horizon, portrait, {g['cam_h']:.0f} up its back plane, "
    f"lens ({g['cam_lens'][0]:.0f}, 0, {g['cam_lens'][2]:.0f}); L2 and props out of its view + 2 deg")
sh = np.array([np.linalg.norm(nodes[a] - nodes[b]) for a, b in E])
ext = nodes.max(0) - nodes.min(0)
say(f"shell: {len(nodes)} nodes ({len(ring_ids)} on the opening ring), {len(E)} tubes, {len(tris)} triangles; tubes {sh.min():.0f} .. {sh.max():.0f} "
    f"(mean {sh.mean():.0f}), {sh.sum() / 1000:.2f} m; node extents {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} (h/w {ext[2] / max(ext[:2]):.2f}); "
    f"hull residual {viol:.2f} (C)")
circ = []
for f in tris:
    a_, b_, c_ = nodes[f]; la, lb, lc = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_); s_ = (la + lb + lc) / 2
    circ.append(2 * math.sqrt(max(s_ * (s_ - la) * (s_ - lb) * (s_ - lc), 1e-9)) / s_)
say(f"largest cell: inscribed circle Ø{max(circ) - 2 * SHELL_R:.0f} between tube surfaces (C)")
short = [(i, l) for i, l in enumerate(sh) if l < BREAK_EVEN]
rl = {k: float(np.linalg.norm(np.subtract(q, p))) for k, (p, q) in extra.items()}
say(f"carbon: shell {len(sh) - len(short)} tubes 6/4 (Spark 26 Oct), shortest {min(l for l in sh if l >= BREAK_EVEN):.0f}; links {sum(v >= BREAK_EVEN for v in rl.values())} rods 3.0 solid, "
    f"{sum(rl.values()) / 1000:.2f} m, shortest {min(rl.values()):.0f} ({min(rl, key=rl.get)}); printed struts (< {BREAK_EVEN:.0f}): "
    + (", ".join(f"shell {i} {l:.0f}" for i, l in short) + ", " if short else "") + (", ".join(f"{k} {v:.0f}" for k, v in rl.items() if v < BREAK_EVEN) or "none"))
pg = min(seg_band_dist(nodes[a], nodes[b], mxy) for a, b in E) - SHELL_R
pn = min(seg_band_dist(n, n, mxy) for n in nodes) - NODE_R
pe_all = {k: seg_band_dist(np.asarray(p, float), np.asarray(q, float), mxy) - ROD_R for k, (p, q) in extra.items()}
say(f"props: shell tubes {pg:.1f} off the swept disks (≥ 12, 3D), nodes {pn:.1f}, links {min(pe_all.values()):.1f} ({min(pe_all, key=pe_all.get)}) (C)")
eqs = ConvexHull(nodes).equations
for nm, ms in (("props (disks)", g["props"]), ("L2 + camera + apex", [g["l2"], g["cam"], g["cam_bracket"], g["apex"]]), ("arms + motors", g["arms"] + g["motors"]),
               ("hub + plates", [TOP, BOTTOM, PI, DECK, PDB]), ("belly", [g["cart"], g["sled"], g["cam_pod"], g["flow_pod"]])):
    p = np.vstack([pts(m, 2000) for m in ms]); p = p[p[:, 2] > g["z_o"]]
    say(f"   inside: {nm} {float((-(p @ eqs[:, :3].T + eqs[:, 3])).min(1).min()):.1f} from the nearest face plane (C)")
rc = ring_nodes(g)
say(f"opening: {len(ring_ids)} nodes on an ellipse {np.ptp(rc[:, 0]):.0f} x {np.ptp(rc[:, 1]):.0f} round the swap zone {2 * ZONE[0]:.0f} x {2 * ZONE[1]:.0f} "
    f"(receiver {REC[0]} x {REC[1]} M + {SPARE:.0f} a side), plane z {g['z_o']:.1f} = cartridge bottom - 3; v2 TPU feet the floor contact (z {min(f.bounds[0][2] for f in g['feet']):.1f})")
zone = box(-DC - ZONE[0], -DC + ZONE[0], -ZONE[1], ZONE[1], g["z_o"] - 200, g["cart"].bounds[0][2])
blk = [(i, overlap(zone, rod(nodes[a], nodes[b], SHELL_R))) for i, (a, b) in enumerate(E)]
blk = [(i, v) for i, v in blk if v > 0.5] + [(k, overlap(zone, rod(p, q))) for k, (p, q) in extra.items() if overlap(zone, rod(p, q)) > 0.5]
say("swap zone (receiver + spare, from the cartridge bottom down): " + ("clear (0 mm³, M)" if not blk else "; ".join(f"{k} {v:.0f} mm³" for k, v in blk)))
Bc = g["B"] + A * 40.0
rng = np.random.default_rng(1); D = rng.normal(size=(20000, 3)); D /= np.linalg.norm(D, axis=1)[:, None]; D = D[D @ A > 0]
hit = np.zeros(len(D), bool)
for p0, p1, rr in [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), ROD_R) for p, q in extra.values()]:
    u = p1 - p0; w0 = Bc - p0; bq = D @ u; cq = float(u @ u); dq = D @ w0; eq_ = float(u @ w0)
    sq = np.clip((eq_ - bq * dq) / np.maximum(cq - bq * bq, 1e-9), 0, 1); tq = -dq + sq * bq
    hit |= (np.linalg.norm(Bc + tq[:, None] * D - (p0 + sq[:, None] * u), axis=1) < rr) & (tq > 0)
for n in nodes:
    v = n - Bc; t = D @ v; hit |= (np.linalg.norm(v - t[:, None] * D, axis=1) < NODE_R) & (t > 0)
say(f"L2 field: {100 * hit.mean():.1f} % of its hemisphere crossed by tubes and nodes (E; thin members allowed, CLAUDE.md 8)")
for lens, hh, hv, nm in ((g["down_lens"], 44.5, 36.5, "down camera"), (g["flow_lens"], 23.0, 23.0, "flow")):
    for k, m in (("shell", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E])), ("cartridge", g["cart"]), ("feet", trimesh.util.concatenate(g["feet"]))):
        d = pts(m, 4000) - lens; zz = -d[:, 2]
        n_ = ((zz > 0) & (np.abs(np.degrees(np.arctan2(d[:, 0], np.maximum(zz, 1e-6)))) < hh) & (np.abs(np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))) < hv)).mean()
        if n_: say(f"   {k} in the {nm} view ({100 * n_:.1f} % of its points)")
say("down camera and flow: views checked (lines above list anything in view)")
shell_g = sh.sum() * SHELL_GPM + NODE_G * len(nodes); link_g = sum(rl.values()) * ROD_GPM
say(f"cage: shell {shell_g:.0f} g (tubes {sh.sum() * SHELL_GPM:.0f} E + nodes {NODE_G * len(nodes):.0f} E), links {link_g:.0f} g (C); "
    f"v0 rings ~96 g, v1 sphere 802 + 98 g (E)")
t2, c2 = cg([x for x in it if x[0] not in ("pack",)] + [("cartridge out", -LATCH_G * 0.45, g["cart"].bounding_box.centroid)])
Iyy = sum(m_ * ((c_[0] - cgv[0]) ** 2 + (c_[2] - cgv[2]) ** 2) for _, m_, c_ in it) * 1e-6
say(f"mass ~{tot:.0f} g (E; latch set {LATCH_G} M), CG ({cgv[0]:.1f}, {cgv[1]:.1f}, {cgv[2]:.1f}) vs thrust centre ({dm:.1f}, 0): {math.hypot(cgv[0] - dm, cgv[1]):.1f} off; "
    f"pack + cartridge out: {math.hypot(c2[0] - dm, c2[1]):.1f} off ({t2:.0f} g)")
say(f"T/W {6600 / tot:.2f} (C on E mass, 1650 gf a motor); CG {PROP_Z - cgv[2]:.0f} under the prop plane; pitch inertia {Iyy:.1f} g m² (E, point masses)")
hullK = ConvexHull(K)
say(f"compact: shell hull {ConvexHull(nodes).volume / 1e6:.1f} L vs the parts-with-margins hull {hullK.volume / 1e6:.1f} L (C)")

SUMMARY = dict(freq=FREQ, start=START, dm=dm, xB=g["xB"], cam_h=g["cam_h"], z_o=g["z_o"], nodes=len(nodes), tubes=len(E), ring=len(ring_ids), shell_m=float(sh.sum() / 1000),
               shell_g=float(shell_g), link_g=float(link_g), mass=tot, cg=cgv.tolist(), cg_pack_out=c2.tolist(), extents=ext.tolist(), prop_gap=pg,
               l2_blocked=float(hit.mean()), motors_xy=[list(map(float, m)) for m in mxy], Iyy_gm2=Iyy, largest_cell=max(circ) - 2 * SHELL_R)
json.dump(SUMMARY, open(os.path.join(OUT, "result.json"), "w"), indent=1)

# ---------------------------------------------------------------- export: parts, assembly file
from assembly import new as new_assembly
PD = os.path.join(OUT, "parts"); os.makedirs(PD, exist_ok=True)
rel = lambda p_: os.path.relpath(p_, ROOT)
node_m = lambda c_: trimesh.creation.icosphere(1, NODE_R).apply_translation(c_)
grp = {}
for n_, gm, _c in it:
    k = ("shell" if n_.startswith("shell") else "nodes" if n_.startswith("node") else "links" if n_.split()[0] in ("spoke", "core", "L2", "bay", "keel") and n_ != "L2"
         else "hub" if n_ in ("Pi 5", "FC", "PDB", "UBEC", "buck", "RX", "Pi deck") else "belly" if n_ in ("sled", "cam_pod", "flow_pod") else
         "feet" if n_.startswith("foot") else "L2 mount" if n_ in ("apex", "cam plate") or n_.startswith("cam rod") else n_)
    grp[k] = grp.get(k, 0.0) + gm
links = trimesh.util.concatenate([rod(p, q) for p, q in extra.values()])
PARTS = [  # name, mesh, role, protect, g, tag, material, group, colour, swap
    ("cage shell tubes", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E]), "cage", False, grp["shell"], "E", "carbon", "cage", "#2b2b2b", False),
    ("cage shell nodes", trimesh.util.concatenate([node_m(n) for n in nodes]), "cage", False, grp["nodes"], "E", "PETG", "cage", "#7d8a99", False),
    ("spokes, ties, keels", links, "structure", False, grp["links"], "C", "carbon", "cage", "#444444", False),
    ("TPU feet", trimesh.util.concatenate(g["feet"]), "cage", False, grp["feet"], "E", "TPU", "cage", "#555555", False),
    ("apex + cam bracket", trimesh.util.concatenate([g["apex"], g["cam_bracket"]]), "structure", False, grp["L2 mount"], "E", "PETG", "printed", "#aab4c0", False),
    ("Unitree L2", g["l2"], "bought", True, 230, "P", None, "bought", "#222222", False),
    ("front camera", g["cam"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False),
    ("top plate", TOP, "bought", False, 23, "E", None, "frame", "#d8d8d8", False),
    ("bottom plate", BOTTOM, "bought", False, 61, "E", None, "frame", "#cfcfcf", False),
    ("hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", trimesh.util.concatenate([FC, PI, DECK, PDB, UBEC, BUCK, RX]), "bought", True, grp["hub"], "E", None, "bought", "#2e7d32", False),
    ("wires", FC, "bought", False, 80, "E", None, "bought", "#1b5e20", False),
    ("pack", g["pack"], "bought", True, 650, "E", None, "bought", "#2f4fbf", True),
    ("cartridge + receiver (latch set)", g["cart"], "protected", True, LATCH_G, "M", "PETG", "printed", "#99a6b3", True),
    ("belly sled + pods", trimesh.util.concatenate([g["sled"], g["cam_pod"], g["flow_pod"]]), "structure", False, grp["belly"], "E", "PETG", "printed", "#99a6b3", False),
    ("down camera", g["down_cam"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False),
    ("flow MTF-02P", g["flow"], "bought", True, 2, "E", None, "bought", "#2d5a2d", False),
]
for i in range(4):
    PARTS += [(f"arm {i + 1}", g["arms"][i], "structure", False, grp[f"arm {i + 1}"], "E", "PETG", "printed", "#8899aa", False),
              (f"motor {i + 1}", g["motors"][i], "bought", True, 81, "E", None, "bought", "#b0b0b0", False),
              (f"prop {i + 1}", cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, *mxy[i], 96), "bought", True, 10, "E", None, "bought", "#d9a400", False)]
fname = lambda n_: "".join(ch if ch.isalnum() else "_" for ch in n_.lower()).strip("_")[:40] + ".stl"
for p_ in PARTS:
    p_[1].export(os.path.join(PD, fname(p_[0])))
json.dump([dict(name=n_, file=fname(n_), role=r_, protect=pr_, mass_g=round(float(gm_), 2), tag=t_, group=gr_, color=co_, swap=sw_)
           for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS], open(os.path.join(PD, "parts.json"), "w"), indent=1)
say(f"assembly mass {sum(p_[4] for p_ in PARTS):.0f} g (E) vs the layout model {tot:.0f} g")
SHELL_SPEC = dict(nodes=np.round(nodes, 3).tolist(), rods=[list(map(int, e)) for e in E],
                  open=[dict(name="bottom opening (pack + swap)", ring=[int(i) for i in np.where(nodes[:, 2] < g["z_o"] + 0.5)[0]])],
                  node_r_mm=NODE_R, rod_r_mm=SHELL_R)
json.dump(SHELL_SPEC, open(os.path.join(OUT, "cage_shell.json"), "w"))
if not any(k in os.environ for k in ("V4_FREQ", "V4_START", "V4_STARTS", "V4_PACK")) and os.path.abspath(OUT) == os.path.join(ROOT, "runs", "overseer_layout", "v4"):
    asm_parts = []
    for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS:
        p_ = dict(name=n_, role=r_, mesh=rel(os.path.join(PD, fname(n_))), mass_g=round(float(gm_), 2), tag=t_, owner="overseer",
                  version="layout v4 / cage v2", source="nox/overseer/layout_v4.py")
        if pr_: p_["protect"] = True
        if mat_: p_["material"] = mat_
        if sw_: p_["swap"] = True
        asm_parts.append(p_)
    hubn = "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)"
    contact = [["wires", hubn], ["pack", "cartridge + receiver (latch set)"], ["cage shell tubes", "cage shell nodes"], ["spokes, ties, keels", "cage shell nodes"],
               ["spokes, ties, keels", "cage shell tubes"], ["spokes, ties, keels", "bottom plate"], ["spokes, ties, keels", "apex + cam bracket"],
               ["spokes, ties, keels", "belly sled + pods"], ["Unitree L2", "apex + cam bracket"], ["front camera", "apex + cam bracket"],
               ["apex + cam bracket", "bottom plate"], ["apex + cam bracket", "top plate"], ["belly sled + pods", "cartridge + receiver (latch set)"],
               ["belly sled + pods", "down camera"], ["belly sled + pods", "flow MTF-02P"], ["belly sled + pods", "TPU feet"], ["belly sled + pods", "bottom plate"],
               [hubn, "bottom plate"], [hubn, "top plate"]]
    for i in range(1, 5):
        contact += [[f"arm {i}", "top plate"], [f"arm {i}", "bottom plate"], [f"motor {i}", f"arm {i}"], ["spokes, ties, keels", f"arm {i}"], [f"arm {i}", hubn]]
    asm = new_assembly("Overseer", "overseer", asm_parts,
                       note=("Layout v4 / cage v2 (26 Oct, Spark: layout v2 with its cage replaced by a triangulated shell formed to the drone; funnel "
                             "redesigned later) from nox/overseer/layout_v4.py; supersedes layout v3 / cage v1 (sphere). Rough layout: C geometry on E sizes. "
                             "Drone frame: x fwd, y left, z up, z 0 bottom plate top. Left out: ESC bodies (25 g each in the arm masses), catch features "
                             "(the new funnel's), scan stick mount, XT60 and leads; the latch set is drawn as v2's cartridge envelope."),
                       cg_ref=[round(dm, 3), 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0,
                       openings=[dict(part=p_, axis=[0, 0, -1], half_angle_deg=40.0, skip_probes=True)
                                 for p_ in ("pack", "cartridge + receiver (latch set)", "down camera", "flow MTF-02P")],
                       contact_pairs=contact, guard_offsets={"shell tubes to prop disks, 3D (G)": round(pg, 1)}, mass_budget_g=2200, cage_shell=SHELL_SPEC,
                       balance_no_swap_gate=False)   # dimensions.md L Balance: Overseer pack-out reported (DEFAULT 26 Oct, Spark's v2 format)
    asm.save(os.path.join(ROOT, "nox", "overseer", "overseer.assembly.json"))
    say("wrote nox/overseer/overseer.assembly.json")
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
