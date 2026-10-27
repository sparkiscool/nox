"""Overseer layout v5 (rough design, 26 Oct): layout v4 (the v2 drone with a geodesic disk cage formed to it) with the L2 and
the front camera outside the cage, nothing in their views. Spark 26 Oct: "The lidar needs to be outside the cage and not be
blocked at all, try to give it enough support and guard is okay as long as it's outside the view of the lidar, also keep
the two camera layout, also needs to not have its view obstructed, also why are there three orientations, I just need the
one with perfect cg". Spark 27 Oct: "get rid of the down camera we don't need it, put the two cameras on either side the
lidar pointing so their field of view is maximized".

The shell target is every part with its margin except the L2, the two cameras and their mounts; every shell node stays
NODE_R + 3 behind the L2 base plane (its 90 deg field) and beyond one view face (+2 deg) of each camera, so the shell's
front is a flat face there. The camera pair: every orientation of the mirrored pair (portrait / landscape, yaw, pitch) is
ranked by the union of their fields, then the share of the L2 field they see; the first one with a mount beside the L2
(behind its base plane, clear of the guard ring) where nothing of the drone is in view and the shell target clears a view
face wins. The L2 base is placed so the target fits behind those limits.
The L2 keeps v2's mount (seat, back block, side lugs, keels to the front hub, ties into the shell behind its plane) and
gets a guard ring 8 behind its base plane (rods a camera would see are left out). The flow sensor looks
through the bottom opening; no down camera (Spark 27 Oct). One layout: the CG on the thrust centre (v2's balance).

Shell method as v4: a class-I geodesic (FREQ, symmetric about xz) laid on an ellipsoid of the target hull, pushed out node by
node until its convex hull holds the target (now also projected back behind the two view limits); start scale and a prop
snap scanned, structure and compactness first, then the least prop shadow.

python3 nox/overseer/layout_v5.py [out_dir]      (~7 min: balance x start / dome / prop-snap scan)
Drone frame: x forward, y left, z up; z 0 = bottom plate top; plate centre x 0. Numbers: `nox/dimensions.md` O / L / G.
"""
import math, os, sys, io, json
import numpy as np, trimesh
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import assembly_seed as S

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_layout", "v5")
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


L = S._layout_v2()                       # v2 definitions (v2 itself is not edited)
box, cyl, rod, pts, union, overlap = L["box"], L["cyl"], L["rod"], L["pts"], L["union"], L["overlap"]
A, HALF, PROP_R, PROP_Z, BLADE, ROOT_H = L["A"], L["HALF"], L["PROP_R"], L["PROP_Z"], L["BLADE"], L["ROOT_H"]
TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX = (L[k] for k in ("TOP", "BOTTOM", "FC", "DECK", "PI", "PDB", "UBEC", "BUCK", "RX"))

# ---------------------------------------------------------------- numbers
DC, ZB = 30.0, 51.0                      # v2's chosen case: catch 30 aft, L2 base z 51
PACK_AXIS = os.environ.get("V5_PACK", "aft") == "axis"   # axis: the pack bay follows the thrust centre (pack out balanced too)
WZ = -12.0                               # belly as close to the body as it goes (Spark 26 Oct: "the battery ... as close to the drone
                                         # body as we can, disregard the funnel"): v2's set_wheel(-12) = v0's pack top z -7 (5 for the
                                         # latch block, E); v2 hung it at -33 only for the funnel's wheel height
L["set_wheel"](WZ)
FREQ = int(os.environ.get("V5_FREQ", 2))  # geodesic frequency (Spark 26 Oct: the larger cells; 3 the alternative)
ROT, SNAP, PEAK = 0.0, False, False                   # geodesic turn about z (0 / 36 keep the xz mirror) and the prop snap, both scanned
START = float(os.environ.get("V5_START", 0.85))  # start: the geodesic on an ellipsoid of the hull's half extents x START (0 = cast
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
POD_DX = float(os.environ.get("V5_POD_DX", 15.0))  # flow pod 15 ahead of v2's (Spark 27 Oct: clear of the rear arm's bottom screw)
POD_UP = float(os.environ.get("V5_POD_UP", 0.0))   # v2's lens height
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
HALO_BACK, HALO_R, HALO_NODE_R = 8.0, 62.0, 5.0   # L2 guard ring: 8 behind its base plane, r 62 round its axis (body half
                                                 # diagonal 53 + 9), printed nodes r 5 (DEFAULT, E)
FROZEN = os.environ.get("V5_FROZEN", os.path.join(HERE, "cage_v5_shell.json"))   # the cage is fixed (Spark 27 Oct: "make sure we are not
FZ = json.load(open(FROZEN)) if FROZEN and os.path.exists(FROZEN) else None        # changing any cage structure"); V5_FROZEN= builds a new one
_LS = os.path.join(HERE, "link_sizes.json")                       # links the frame model sized up to 6 mm tube (cage_frame.py); the rest 3.0
LINK6 = set(json.load(open(_LS))["tube6"]) if os.path.exists(_LS) else set()
lr = lambda k: SHELL_R if k in LINK6 else ROD_R
lgpm = lambda k: SHELL_GPM if k in LINK6 else ROD_GPM
PRINTED_HEAD = os.environ.get("V5_HEAD", "printed") == "printed"
HEAD_ON_CAGE = os.environ.get("V5_HEAD_ON", "core") == "cage"          # alt (V5_HEAD_ON=cage): the head on the cage's front centre node only, no keels,
# braces or wing struts (Spark 27 Oct tried it: ~32 Hz on TPU nodes, F on E, runs/overseer_head_cage/concept); Spark then: "keep the design we have
# right now i need the lidar to have max performance": the keels and L2 braces stay (A, the stiffest: 200 / 119 Hz rigid / TPU)
LINKN = "spokes, ties, keels, braces, head struts" if os.environ.get("V5_HEAD", "printed") == "printed" else "spokes, ties, keels, L2 guard ring"
HEADN = "sensor head mount (printed: drawn seat, lugs, camera pads, guards; topology joins them)"   # Spark 27 Oct: "for the mounts on the lidar and the camera, replace the bunch
CAM_BRACKET_G = 10.0                                               # of little rods with a topology printed mount": the short rods at the head (L2
                                                                   # ties, guard ring and its struts, camera guards and brackets) become one printed
                                                                   # part. Its mass budget is what it replaces (rods, their nodes, the camera
                                                                   # brackets' struts, E 10 g), so the balance stays (DEFAULT 27 Oct)
L2_CONE, CAM_CONE = 75.0, 55.0                  # cage first is gated outside these cones (L2 field 90 from its axis: 75 keeps the
                                                # ring's lateral cover gated; camera diagonal half-angle 51 + 4) (DEFAULT 27 Oct, review)
LIM = NODE_R + 3.0                               # shell node centres this far behind the L2 base plane and the camera's lower view plane
SLACK = float(os.environ.get("V5_SLACK", 5.0))
DOME = float(os.environ.get("V5_DOME", 60.0))    # minimum top semi-axis of the start ellipsoid (scanned 26 Oct)  # room between the shell target and those limits so frequency-2 cells fit (scanned)


def adjust(g):
    for k in ("down_cam", "cam_pod", "down_lens"): g.pop(k, None)       # no down camera (Spark 27 Oct: "get rid of the down camera, we don't need it")
    g["flow"].apply_translation([POD_DX, 0, POD_UP]); g["flow_pod"].apply_translation([POD_DX, 0, 0])
    g["flow_lens"] = g["flow_lens"] + [POD_DX, 0, POD_UP]
    g["z_o"] = g["cart"].bounds[0][2] - 3.0                            # opening plane: cartridge bottom 3 above it
    return g


def make_K(g):
    """the shell's target: every part but the L2, the cameras and their mounts, with its margin (prop disks 12 off,
    the rest 3 off the tube surface, +1), the belly clipped at the opening plane, mirrored in y"""
    props = np.vstack([np.stack([mx + PROP_R * np.cos(a), my + PROP_R * np.sin(a), np.full_like(a, PROP_Z + s_ * BLADE)], 1)
                       for mx, my in g["motors_xy"] for s_ in (1, -1) for a in [np.linspace(0, 2 * np.pi, 72, endpoint=False)]])
    body = hullpts(g["arms"] + g["motors"] + [TOP, BOTTOM, PI, DECK, PDB, UBEC])
    belly = hullpts([g["cart"], g["sled"], g["flow_pod"], g["flow"]] + g["feet"])
    K = np.vstack([(props[:, None, :] + M_PROP * DIRS[None]).reshape(-1, 3), (body[:, None, :] + M_BODY * DIRS[None]).reshape(-1, 3),
                   (belly[:, None, :] + M_BODY * DIRS[None]).reshape(-1, 3)])
    K[:, 2] = np.maximum(K[:, 2], g["z_o"])
    K = np.vstack([K, K * [1, -1, 1]])
    return K[ConvexHull(K).vertices]


def in_cam_view(g, P, extra_deg=2.0):
    """in either camera's view (+ extra_deg)"""
    P = np.atleast_2d(P); v = np.zeros(len(P), bool)
    for c in g["cams"]: v |= in_frustum(c["lens"], c["f"], c["u"], c["r"], c["ha"], c["hv"], P, extra_deg)
    return v


# ---------------------------------------------------------------- the camera pair (Spark 27 Oct: "get rid of the down camera, we
# don't need it; put the two cameras on either side of the lidar pointing so their field of view is maximized")
CAM_FMTS = {"landscape": (L["CAM_VFOV"] / 2, L["CAM_HFOV"] / 2), "portrait": (L["CAM_HFOV"] / 2, L["CAM_VFOV"] / 2)}   # (half across, half up), 102 diag (E)
CAM_W = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0)   # lens off the centre plane (scanned)
CAM_HS = (-110.0, -90.0, -70.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 70.0, 90.0, 110.0)   # lens up the L2 base plane (scanned)
HEAD_ONE = os.environ.get("V5_HEAD_ONE", "1") == "1"   # Spark 27 Oct: "make the camera mounted on the same print as the lidar mount": the
# the arm session's printed nodes (overseer arm, 27 Oct, Spark "add the arm nodes"): the 3 spokes of each arm leave from a node beside
# the motor (offset from the motor axis point, y mirrored), the L2 braces root in the core (BRACE_ROOT below; first in the front beam, 27 Oct)
ARM_NODE = {"front": (-27.03, 7.2, 1.0), "rear": (19.77, -19.8, -4.0)}
KEEL_INSET = 5.0                                   # the keel tube's core end, 5 along the keel from v5's root (96, ±12, 8): the hub deck socket
BRACE_ROOT = ({"lo": 97.0, "hi": 97.0}, {"lo": 26.0, "hi": 14.0}, {"lo": 12.0, "hi": 18.0})   # (x, y, z per level): the hub deck's front tower between the plates beside the keels, low (core v3, 27 Oct: halves the worst root force to 190 N, the keels near idle in the top hit; was z 22 / 32, alt head 166 Hz)
HEAD_STRUTS = [((401.0, 83.0, 151.0), (354.0, 223.0, 128.7)), ((383.0, 83.0, 73.0), (336.5, 219.0, -29.3))]   # = head_mount.HEAD_STRUTS
HEAD_FIT = 175.0                                   # head (seat, guard ring, both cameras and plates) one print: its oriented box in the
                                                   # A1 mini's 180 cube with 2.5 a side (C); off: centre + 2 wings, cameras at ±120
CAM_PSI = np.arange(0.0, 72.5, 2.5)                # yaw out from +x (scanned)
CAM_TH = np.arange(-30.0, 32.5, 5.0)               # pitch (scanned)
CAM_BODY = np.array([[x, y, z] for x in (-13.5, -10.75, -8.0) for y in np.linspace(-20.0, 20.0, 9) for z in np.linspace(-20.0, 20.0, 9)]
                    + [[0.0, 6 * math.cos(t), 6 * math.sin(t)] for t in np.linspace(0, 2 * math.pi, 8, endpoint=False)])   # plate + board + lens, lens frame
SPH = fib(6000)
DCF = fib(500)
CAM_STRUT_R = 3.4                                  # printed bracket strut ~6 x 6 PETG (Ø6.8 round in the model)


def cam_axes(psi, th, sy):
    """forward, up, across of a camera yawed psi out to side sy and pitched th"""
    p_, t_ = math.radians(psi), math.radians(th)
    f = np.array([math.cos(t_) * math.cos(p_), sy * math.cos(t_) * math.sin(p_), math.sin(t_)])
    u = np.array([0, 0, 1.0]) - f[2] * f; u /= np.linalg.norm(u)
    return f, u, np.cross(u, f)


def in_frustum(lens, f, u, r, ha, hv, P, extra=2.0):
    d = np.atleast_2d(P) - lens; z = d @ f; zz = np.maximum(z, 1e-6)
    return (z > 0) & (np.abs(np.degrees(np.arctan2(d @ r, zz))) < ha + extra) & (np.abs(np.degrees(np.arctan2(d @ u, zz))) < hv + extra)


def cam_faces(f, u, r, ha, hv, extra=2.0):
    """outward normals of the 4 view faces (+ extra) and the lens plane; a point beyond one of them is out of view"""
    a_, b_ = math.radians(ha + extra), math.radians(hv + extra)
    return [r * math.cos(a_) - f * math.sin(a_), -r * math.cos(a_) - f * math.sin(a_), u * math.cos(b_) - f * math.sin(b_),
            -u * math.cos(b_) - f * math.sin(b_), -f]


_CAMT = []


def cam_table():
    """every orientation of the mirrored pair, best first: the union of the two fields (steradians, the ask; within 2 % is a
    tie, the sampling noise), then the blind wedge straight ahead on the horizon (the two fields meet at the nose), then the
    share of the L2's field they see (they colour its cloud). V5_CAM=fmt,yaw,pitch forces one orientation (the alternatives)"""
    if _CAMT: return _CAMT
    l2f = SPH @ A >= 0; rows = []
    for fmt, (ha, hv) in CAM_FMTS.items():
        for psi in CAM_PSI:
            for th in CAM_TH:
                v = np.zeros(len(SPH), bool)
                for sy in (1, -1):
                    f, u, r = cam_axes(psi, th, sy); v |= in_frustum(np.zeros(3), f, u, r, ha, hv, SPH, 0.0)
                hz = np.radians(np.arange(-40.0, 40.01, 0.5)); H_ = np.stack([np.cos(hz), np.sin(hz), np.zeros_like(hz)], 1)
                hv_ = np.zeros(len(H_), bool)
                for sy in (1, -1):
                    f, u, r = cam_axes(psi, th, sy); hv_ |= in_frustum(np.zeros(3), f, u, r, ha, hv, H_, 0.0)
                gap = 0.0 if hv_[np.abs(np.degrees(hz)) < 0.3].all() else float(2 * np.min(np.abs(np.degrees(hz[hv_])))) if hv_.any() else 80.0
                rows.append(dict(fmt=fmt, psi=float(psi), th=float(th), U=4 * math.pi * v.mean(), S=float(v[l2f].mean()), gap=gap))
    um = max(r_["U"] for r_ in rows)
    if os.environ.get("V5_CAM"):
        fm, ps, tt = os.environ["V5_CAM"].split(","); rows = [r_ for r_ in rows if r_["fmt"] == fm and abs(r_["psi"] - float(ps)) < 0.01 and abs(r_["th"] - float(tt)) < 0.01]
    _CAMT.extend(sorted(rows, key=lambda r_: (-int(r_["U"] >= 0.98 * um), r_["gap"], -r_["S"])))
    return _CAMT


def fz_clear(Q):
    """least gap from points Q to the frozen cage's node and tube surfaces"""
    SN = np.array(FZ["nodes"]); a_ = SN[[r_[0] for r_ in FZ["rods"]]]; b_ = SN[[r_[1] for r_ in FZ["rods"]]]
    d_ = b_ - a_; t_ = np.clip(np.einsum("qkj,kj->qk", Q[:, None] - a_[None], d_) / (d_ * d_).sum(1), 0, 1)
    seg = np.linalg.norm(Q[:, None] - (a_[None] + t_[..., None] * d_[None]), axis=2).min() - SHELL_R
    return float(min(seg, np.linalg.norm(Q[:, None] - SN[None], axis=2).min() - NODE_R))


def head_fits(B, up_b, Q, f, r, u, lens):
    """the one-piece head (seat Ø80, guard ring r HALO_R + its nodes, both camera bodies and their back plates) inside the
    printer: oriented bounding box (trimesh) with every side <= HEAD_FIT"""
    acr = np.cross(A, up_b)
    ring = [B - A * HALO_BACK + (HALO_R + HALO_NODE_R) * (math.cos(t_) * up_b + math.sin(t_) * acr) for t_ in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
    seat = [B - A * 4 + 40 * (math.cos(t_) * up_b + math.sin(t_) * acr) for t_ in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    plate = [lens - f * 13.5 + r * a_ + u * b_ for a_ in (-20, 20) for b_ in (-20, 20)]
    P = np.vstack([ring, seat, Q, plate, Q * [1, -1, 1], np.array(plate) * [1, -1, 1]])
    return bool(np.max(trimesh.bounds.oriented_bounds(P)[1]) <= HEAD_FIT)


def cam_pick(B, K, Keq, avoid, face=True):
    """the first orientation of the table with a mount beside the L2 (lens off the centre plane CAM_W, up its base plane
    CAM_HS, body behind the base plane by 2, clear of the guard ring) where nothing of the drone is in view and the shell
    target sits LIM + SLACK beyond one of the view faces (that face is then the shell's limit); of those mounts the most
    compact (least behind the L2 plane, nearest the axis)"""
    up_b = np.array([-A[2], 0, A[0]]); KS = (K @ DCF.T).max(0)
    for t in cam_table():
        ha, hv = CAM_FMTS[t["fmt"]]; f, u, r = cam_axes(t["psi"], t["th"], 1)
        body = CAM_BODY @ np.stack([f, r, u]); faces = cam_faces(f, u, r, ha, hv)
        ok_ = []
        for w, h in ((w, h) for w in CAM_W for h in CAM_HS):
            base = B + up_b * h + np.array([0, w, 0])
            bk = max(0.0, float(((base + body - B) @ A).max()) + 2.0)
            if bk > 80: continue
            lens = base - A * bk; Q = lens + body
            dq = (Q - B) - np.outer((Q - B) @ A, A)
            if np.linalg.norm(dq, axis=1).min() < HALO_R + HALO_NODE_R + 3.0: continue
            if (Q @ Keq[:, :3].T + Keq[:, 3]).max(1).min() < 5.0: continue        # the body clear of the shell target
            if FZ is not None and fz_clear(Q) < 3.0: continue            # the body 3 off the frozen cage's tubes and nodes
            if in_frustum(lens, f, u, r, ha, hv, np.vstack([avoid, Q * [1, -1, 1]])).any(): continue
            if HEAD_ONE and not head_fits(B, up_b, Q, f, r, u, lens): continue
            sl = [float(((K - lens) @ n).min()) for n in faces]; k = int(np.argmax(sl))
            if face and sl[k] < LIM + SLACK: continue                    # (a frozen cage: its own points are in avoid instead)
            ok_.append(dict(t, w=w, h=h, bk=bk, face=k, slack=sl[k], lens_rel=lens - B, cost=bk + 0.5 * (w - CAM_W[0]) + 0.5 * abs(h - 10.0)))
        if ok_: return dict(min(ok_, key=lambda q: q["cost"]), mounts=len(ok_))   # the most compact mount (behind the plane, off the axis)
    return None


def place_cams(g, sel):
    """the pair at sel (mirrored): board, lens, a printed plate behind the board on 3 printed struts to its side's lugs and block"""
    B = g["B"]; up_b = np.array([-A[2], 0, A[0]]); ha, hv = CAM_FMTS[sel["fmt"]]
    cams, meshes, plates, rods_ = [], [], [], []
    for sy in (1, -1):
        f, u, r = cam_axes(sel["psi"], sel["th"], sy); lens = B + sel["lens_rel"] * [1, sy, 1]
        Rc = np.eye(4); Rc[:3, 0] = f; Rc[:3, 1] = r; Rc[:3, 2] = u; Rc[:3, 3] = lens - 9 * f
        board = box(-1, 1, -19, 19, -19, 19); lc = trimesh.creation.cylinder(6, 8, sections=32)
        lc.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); lc.apply_translation([5, 0, 0])
        m = trimesh.util.concatenate([board, lc]); m.apply_transform(Rc)
        pl = box(-4.5, -1.5, -20, 20, -20, 20); pl.apply_transform(Rc)
        corners = [lens - 12 * f + r * a_ + u * b_ for a_ in (-16, 16) for b_ in (-16, 16)]
        # printed 3-point bracket (27 Oct review: 2 rods to one corner were a pin; short links between printed parts are printed):
        # struts 6 x 6 PETG from three different plate corners to the low lug, the high lug and the back block of its side
        ring_ = [B - A * HALO_BACK + HALO_R * (math.cos(t_) * up_b + math.sin(t_) * np.array([0, 1.0, 0]))
                 for t_ in np.linspace(0, 2 * math.pi, 8, endpoint=False) + math.pi / 8]
        clear_ = lambda a_, b_: all(np.linalg.norm(np.cross(b_ - a_, q_ - a_)) / np.linalg.norm(b_ - a_) > HALO_NODE_R + CAM_STRUT_R + 1.0
                                    or not 0 < (q_ - a_) @ (b_ - a_) < (b_ - a_) @ (b_ - a_) for q_ in ring_)
        lugs_ = [B - A * 5 + up_b * 2.0 + np.array([0, 46.0 * sy, 0]), B - A * 5 + up_b * 16.0 + np.array([0, 46.0 * sy, 0])]
        thirds = [B - A * 11 + up_b * z_ + np.array([0, y_ * sy, 0]) for z_ in (20.0, 2.0, 30.0) for y_ in (20.0, 0.0, 35.0)]
        best_ = None
        for th_ in thirds:                                             # 3 struts, 3 corners, clear of the ring nodes, the shortest set
            free = list(range(4)); set_ = []
            for so in lugs_ + [th_]:
                k_ = min(free, key=lambda i_: np.linalg.norm(corners[i_] - so)); free.remove(k_); set_.append((corners[k_], so))
            if all(clear_(a_, b_) for a_, b_ in set_):
                L_ = sum(np.linalg.norm(b_ - a_) for a_, b_ in set_)
                if best_ is None or L_ < best_[0]: best_ = (L_, set_)
        rods_ += best_[1] if best_ else set_
        faces = cam_faces(f, u, r, ha, hv)
        lim_n = -faces[sel["face"]] if sy > 0 else cams[0]["lim_n"] * [1, -1, 1]    # the mirror's face is the mirror of the left one
        cams.append(dict(lens=lens, f=f, u=u, r=r, ha=ha, hv=hv, lim_n=lim_n, mesh=m, side=sy))
        meshes.append(m); plates.append(pl)
    g["cams"] = cams; g["cam_sel"] = sel
    if PRINTED_HEAD: rods_ = []                                        # the printed head joins the plates (topology)
    g["cam"] = trimesh.util.concatenate(meshes); g["cam_plate"] = trimesh.util.concatenate(plates); g["cam_rods"] = rods_
    g["cam_bracket"] = trimesh.util.concatenate(plates + [rod(a_, b_, CAM_STRUT_R) for a_, b_ in rods_])
    g["cam_lens"], g["cam_fwd"], g["cam_up_v"] = cams[0]["lens"], cams[0]["f"], cams[0]["u"]
    return g


LIM_L2 = HALO_BACK + HALO_NODE_R + 2.0 + NODE_R   # shell node centres behind the L2 base plane: the guard ring fits in front (25.2)
PHIS = (20.0, 35.0, 50.0, 65.0)                  # tilt of the head plane (from the L2 axis toward +z), scanned
_DRONE = {}


def saved_node(c):
    """the newest saved cage node at c (CLAUDE.md "Cage sessions keep their parts in sync": the artifact shows the saved parts):
    a decimated copy of runs/cage_nodes/v3/node_<i>/cage_node_<i>.stl (run/cage_nodes-v3) in runs/overseer_layout/v5/saved/, for the
    design at c, or its mirror's reflected (y -> -y); None if not saved yet"""
    if FZ is None: return None
    SN = np.array(FZ["nodes"])
    for i, q in enumerate(SN):
        for sy in (1, -1):
            if np.linalg.norm(q * [1, sy, 1] - c) < 0.5:
                f = os.path.join(ROOT, "runs", "overseer_layout", "v5", "saved", f"cage_node_{i}.stl")   # decimated copy of the saved node
                if os.path.exists(f):
                    m = trimesh.load(f, force="mesh")
                    if sy < 0: m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0])); m.invert()
                    return m
    return None


SAVED_ARMS = {"rear": "runs/overseer_layout/v5/saved/overseer_arm_rear_v5d.stl"}   # decimated copies of the arm session's newest saved
                                                   # run meshes (arm frame; rear v5d: claude/pensive-turing-jyxlcc 319ca86a)
ARM_DEG = {"front": 45.0, "rear": 135.0}           # the arm frame: root at r 66.5 on the plate diagonal, x toward the motor (arm_topo_v1)


def saved_arm(mx, my, dm):
    """the newest saved arm at the motor (mx, my), in the drone frame (the arm session's arm_to_drone), or None"""
    which = "front" if mx > dm else "rear"
    f = SAVED_ARMS.get(which)
    if not f or not os.path.exists(os.path.join(ROOT, f)): return None
    d = math.radians(ARM_DEG[which]); R0 = 66.5 * np.array([math.cos(d), math.sin(d)])
    th = math.atan2(abs(my) - R0[1], mx - R0[0])
    m = trimesh.load(os.path.join(ROOT, f), force="mesh"); T = trimesh.transformations.rotation_matrix(th, [0, 0, 1]); T[:2, 3] = R0
    m.apply_transform(T)
    if my < 0: m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0])); m.invert()
    return m


def apex_head(g):
    """the apex without the front hub block (the core's since 27 Oct, core v2 keel sockets on the hub deck): the part behind the L2
    base plane by 45 or less, as head_pts"""
    if "apex_head" not in g:
        g["apex_head"] = g["apex"].slice_plane(g["B"] - A * 45.0, A, cap=True)
    return g["apex_head"]


def head_pts(g):
    """the sensor head behind the L2 base plane: seat, back block, lugs (not the front hub)"""
    P = pts(g["apex"], 3000)                       # the cameras keep the shell out by their own view face and body check
    return P[((P - g["B"]) @ A) > -45.0]


def drone(dm):
    """the v2 drone at motor square dm, its sensor head (L2 + the camera pair) placed outside the shell: the shell target
    (make_K) LIM_L2 + SLACK behind the L2 base plane, LIM + SLACK beyond each camera's limiting view face and behind the head
    plane (the head's own back, tilted PHI from the L2 axis toward +z; the tilt that keeps the L2 furthest back wins)"""
    key = round(dm, 1)
    if key in _DRONE: return _DRONE[key]
    g0 = adjust(L["build"](dm, DC, 330.0, ZB)); K = make_K(g0); Keq = ConvexHull(K).equations
    disks = np.vstack([pts(p, 1500) for p in g0["props"]]); armp = np.vstack([pts(a_, 500) for a_ in g0["arms"]])
    best = None
    if FZ is not None:                                                 # the frozen cage: the L2 where it was, nothing of the cage in a camera view
        SN = np.array(FZ["nodes"]); SP_ = np.vstack([SN + NODE_R * DIRS[i % len(DIRS)] for i in range(4) for _ in [0]] +
                                                    [SN[a_] + t_ * (SN[b_] - SN[a_]) for a_, b_ in FZ["rods"] for t_ in np.linspace(0, 1, 12)])
        SP_ = np.vstack([SP_] + [SN[a_] + t_ * (SN[b_] - SN[a_]) + SHELL_R * d_ for a_, b_ in FZ["rods"] for t_ in (0.25, 0.5, 0.75) for d_ in DIRS[:16]])
        SP_ = np.vstack([SP_, (SN[:, None, :] + NODE_R * DIRS[None]).reshape(-1, 3)])
        g = adjust(L["build"](dm, DC, FZ["xB"], ZB, 0.0, 60.0))
        sel = cam_pick(g["B"], K, Keq, np.vstack([pts(g["l2"], 1500), disks, pts(g["apex"], 1000), armp, SP_]), face=False)
        g = place_cams(g, sel)
        g.update(dm=dm, xB=FZ["xB"], mxy=g["motors_xy"], keels={k: v for k, v in g["rods"].items() if k.startswith("keel")}, K=K, phi=None, lims=[])
        _DRONE[key] = g
        return g
    for phi in PHIS:
        n3 = A * math.cos(math.radians(phi)) + np.array([0, 0, 1.0]) * math.sin(math.radians(phi)); n3 /= np.linalg.norm(n3)
        xB = (float((K @ A).max()) + LIM_L2 + SLACK - ZB * A[2]) / A[0]
        for _ in range(25):
            g = adjust(L["build"](dm, DC, xB, ZB, 0.0, 60.0))
            sel = cam_pick(g["B"], K, Keq, np.vstack([pts(g["l2"], 1500), disks, pts(g["apex"], 1000), armp]))
            if sel is None: xB += 5.0; continue                       # no mount sees clear: the head further out
            g = place_cams(g, sel)
            d_b = float((head_pts(g) @ n3).min())
            viol = float((K @ n3).max()) - (d_b - LIM - SLACK)
            if viol <= 0: break
            xB += viol / n3[0] + 0.5
        if best is None or xB < best[0]: best = (xB, phi, n3, d_b, g)
    xB, phi, n3, d_b, g = best
    g.update(dm=dm, xB=xB, mxy=g["motors_xy"], keels={k: v for k, v in g["rods"].items() if k.startswith("keel")}, K=K, phi=phi)
    g["lims"] = [(A, g["B"], LIM_L2)] + [(c["lim_n"], c["lens"], LIM) for c in g["cams"]] + [(n3, n3 * d_b, LIM)]
    _DRONE[key] = g
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


def shell(g, seed=1, warm=None):
    if FZ is not None:                                                 # the frozen cage (Spark 27 Oct): its nodes and tubes as they are
        nodes = np.array(FZ["nodes"]); E = sorted(tuple(sorted(map(int, r_))) for r_ in FZ["rods"]); ring_ids = list(FZ["open"][0]["ring"])
        h = ConvexHull(nodes); adj = {i: set() for i in range(len(nodes))}
        for a_, b_ in E: adj[a_].add(b_); adj[b_].add(a_)
        tris = np.array(sorted({tuple(sorted((a_, b_, c_))) for a_, b_ in E for c_ in adj[a_] & adj[b_]}))
        viol = float((g["K"] @ h.equations[:, :3].T + h.equations[:, 3]).max())   # the target (parts + margins) still inside it
        shell.lastN = nodes; shell.snapped = 0
        return nodes, E, tris, ring_ids, viol, g["K"]
    mxy = g["mxy"]; K = g["K"]
    Kmesh = trimesh.convex.convex_hull(K)
    R = ring_nodes(g); nR = len(R)
    # free nodes: a class-I icosahedral geodesic (FREQ), a vertex up and a mirror plane on xz; each direction, stretched to the
    # hull's proportions (split top / bottom), is cast from the centre onto the hull, so the pattern stays geodesic and the
    # nodes start on the drone's margin surface; directions inside the opening ring are left to the ring; the convex pass
    # below then pushes nodes out only as far as their chords need
    c0 = 0.5 * (K.min(0) + K.max(0)); c0[2] = PROP_Z                  # equator at the prop plane: the drone's widest (v5: the L2 and
    hx, hy = np.ptp(K[:, 0]) / 2, np.ptp(K[:, 1]) / 2; ct, cb = K[:, 2].max() - c0[2], c0[2] - g["z_o"]   # camera no longer lift the top)
    ct = max(ct, DOME)                                                 # a minimum dome so the top keeps its triangles (no tent)
    area = 4 * math.pi * ((((hx * hy) ** 1.6 + (hx * (ct + cb) / 2) ** 1.6 + (hy * (ct + cb) / 2) ** 1.6) / 3) ** (1 / 1.6))
    f = FREQ
    Ug = geodesic(f)
    tr = math.radians(ROT); Ug = Ug @ np.array([[math.cos(tr), -math.sin(tr), 0], [math.sin(tr), math.cos(tr), 0], [0, 0, 1]]).T
    D = np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1)   # ellipsoid directions
    D /= np.linalg.norm(D, axis=1)[:, None]
    eqK = ConvexHull(K).equations                                       # cast each onto the hull: nodes start on it
    den = D @ eqK[:, :3].T; num = -(c0 @ eqK[:, :3].T + eqK[:, 3])
    tK = np.where(den > 1e-9, num / np.where(den > 1e-9, den, 1), np.inf).min(1)
    Pg = (c0 + tK[:, None] * D) if START <= 0 else c0 + START * np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1)
    ex, ey = np.ptp(R[:, 0]) / 2, np.ptp(R[:, 1]) / 2
    in_open = (((Pg[:, 0] + DC) / (1.25 * ex)) ** 2 + (Pg[:, 1] / (1.25 * ey)) ** 2 < 1.0) & (Ug[:, 2] < 0)
    N = np.vstack([R, Pg[~in_open]])
    if PEAK:                                                           # local nodes on the hull's top front (the nose camera peak)
        Kp = K[(K[:, 0] > c0[0] + 0.4 * hx) & (K[:, 1] >= 0)]
        top2 = Kp[np.argsort(-Kp[:, 2])]; sel = [top2[0]]
        for q in top2[1:]:
            if np.linalg.norm(q - sel[0]) > 60: sel.append(q); break
        add = np.array([q for q0 in sel for q in ([q0] if q0[1] < 10 else [q0, q0 * [1, -1, 1]])])
        keep = [i for i in range(nR, len(N)) if np.min(np.linalg.norm(add - N[i], axis=1)) > 60.0]
        N = np.vstack([N[:nR], N[keep], add])
    # a 3 x 5 grid of nodes on the flat front face (behind the L2 and under the camera, y 0 / ±85 / ±170): without them tubes span
    # the whole face (~400, 26 Oct); they are also the L2 mount's nearest shell nodes
    fr = []
    for zf, yf in [(z_, y_) for z_ in (PROP_Z + 20.0, PROP_Z - 30.0, PROP_Z - 80.0) for y_ in (0.0, 85.0, -85.0, 170.0, -170.0)]:
        q = np.array([K[:, 0].max() + 200.0, yf, zf])
        for n_, p0, lim_ in g["lims"]:                                 # back along -x onto each limit, its offset behind it
            over = (q - p0) @ n_ + lim_
            if over > 0: q = q - np.array([over / n_[0], 0, 0])
        fr.append(q)
    keep = [i for i in range(nR, len(N)) if min(np.linalg.norm(N[i] - q) for q in fr) > 60.0]
    N = np.vstack([N[:nR], N[keep], np.array(fr)])
    N[nR:, 2] = np.maximum(N[nR:, 2], g["z_o"] + 1.0)
    shell.freq = f
    dirs = lambda P: (P - c0) / np.linalg.norm(P - c0, axis=1)[:, None]
    topo = lambda P: ConvexHull(dirs(P)).simplices                     # triangulation by direction: every node stays a vertex
    from scipy.spatial import cKDTree
    mir = np.array([1.0, -1.0, 1.0])

    def partners(N):
        """each node's mirror partner (itself on the centre plane): perfect left-right symmetry (Spark 27 Oct)"""
        d, j = cKDTree(N).query(N * mir)
        assert d.max() < 1.0, f"shell nodes are not mirror pairs ({d.max():.2f})"
        return j

    def sym(N, P):
        M = 0.5 * (N + N[P] * mir); M[P == np.arange(len(N)), 1] = 0.0; return M

    def convex(N, U):
        """the true hull of the nodes must hold every hull point: push the nodes of each crossed face along U (capped);
        every step is taken by a node and its mirror partner alike, so the shell stays symmetric"""
        viol = np.array([np.inf]); stuck = np.zeros(len(N), int); P = partners(N)
        for it in range(2000):
            h = ConvexHull(N)
            lost = np.setdiff1d(np.arange(len(N)), h.vertices)
            lost = np.union1d(lost, P[lost])
            if len(lost):
                lf = lost[lost >= nR]
                stuck[lf] += 1
                drop = lf[stuck[lf] > 15]                                # held inside by the view limits: not a cell corner there
                if len(drop):
                    drop = np.union1d(drop, P[drop])
                    keep = np.setdiff1d(np.arange(len(N)), drop); N, U, stuck = N[keep], U[keep], stuck[keep]; P = partners(N); continue
                if len(lf) == 0:
                    return N, np.array([np.inf]), U                     # a ring node fell inside: invalid start
                N[lf] += 3.0 * U[lf]; N = sym(behind(N), P); continue
            viol = (K @ h.equations[:, :3].T + h.equations[:, 3]).max(0)
            if viol.max() <= 0.05: break
            step = np.zeros(len(N))
            for f, v in zip(h.simplices, viol):
                if v > 0.05:
                    for a in f: step[a] = max(step[a], min(v, 15.0))
            step = np.maximum(step, step[P]); step[:nR] = 0
            N = sym(behind(N + (0.7 * step)[:, None] * U), P)
        return N, viol, U

    def behind(N):
        """every free node behind each view limit by its offset (projected onto it)"""
        N = N.copy(); P0 = partners(N)
        for _ in range(12):                                            # each pass the worst limit per node, then the pair averaged (the camera
            O = np.stack([(N - p0) @ n_ + lim_ for n_, p0, lim_ in g["lims"]], 1); k = O.argmax(1); over = O[np.arange(len(N)), k]
            mv = over > 0.01; mv[:nR] = False                          # limits come in mirror pairs; a centre node stays on the plane)
            if not mv.any(): break
            Ns = np.stack([g["lims"][j][0] for j in k])
            N[mv] -= over[mv][:, None] * Ns[mv]; N = sym(N, P0)
        N[nR:, 2] = np.maximum(N[nR:, 2], g["z_o"] + 1.0)              # free nodes stay above the opening ring (it is the bottom)
        return N
    if warm is not None and len(warm) > nR: N = np.vstack([R, warm[nR:]])   # balance trim: the last shell's pattern, re-held round the moved parts
    U = dirs(N)
    N = sym(N, partners(N)); U = dirs(N)
    N, viol, U = convex(behind(N), U)
    if SNAP and warm is None and np.isfinite(viol.max()):
        # prop snap, a mirror pair at a time (Spark 26 Oct: less structure over and under the props, secondary to structure):
        # a free node in a prop disk column tries the motor axis (the hub, low thrust; one per prop and side) or out past
        # the tip (R + 15), keeping its height, its partner the mirror move; both then move only vertically in a convex pass;
        # the move stays only if the shell still converges, keeps its ring, the props 12 off and its prop shadow drops
        grids = prop_grids(g)
        def stats(Nn):
            h_ = ConvexHull(Nn); F = [f for f in h_.simplices if not (f < nR).all()]
            Ee = sorted({tuple(sorted((int(f[i]), int(f[(i + 1) % 3])))) for f in F for i in range(3)})
            pg_ = min(seg_band_dist(Nn[a_], Nn[b_], mxy) for a_, b_ in Ee) - SHELL_R
            sh_ = sum(sum(seg_shadow(grids, Nn[a_], Nn[b_], SHELL_R)) for a_, b_ in Ee)
            return pg_, sh_, len(h_.vertices) == len(Nn)
        _, sh0, _ = stats(N)
        Ud = U.copy(); used_axis = set(); n_snap = 0
        cand = [i for i in range(nR, len(N)) if N[i, 1] >= 0 and min(math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in mxy) < PROP_R + 15.0]
        for i in sorted(cand, key=lambda i: min(math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in mxy)):
            P = partners(N); j = P[i]
            k = int(np.argmin([math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in mxy])); mx, my = mxy[k]
            r = math.hypot(N[i, 0] - mx, N[i, 1] - my); side = 1 if N[i, 2] > PROP_Z else -1
            T_ = N.copy(); Ut = Ud.copy()
            if r < 0.6 * PROP_R and (k, side) not in used_axis: T_[i, :2] = (mx, my)
            else: T_[i, :2] = np.array([mx, my]) + (N[i, :2] - (mx, my)) / max(r, 1e-6) * (PROP_R + 15.0)
            if j != i: T_[j] = T_[i] * mir
            elif abs(T_[i, 1]) > 0.5: continue                          # a centre-plane node cannot leave the plane
            if np.min(np.linalg.norm(np.delete(T_, [i, j], 0) - T_[i], axis=1)) < 40.0: continue
            Ut[i] = Ut[j] = (0, 0, side)
            T_, v_, Ut = convex(T_, Ut)
            if len(T_) != len(N): continue                             # a snap that drops a node changes the pattern: not taken
            if not np.isfinite(v_.max()) or v_.max() > 0.05: continue
            pg_, sh_, allv = stats(T_)
            if allv and pg_ >= PROP_GAP and sh_ < sh0 - 0.05:
                N, Ud, viol, sh0 = T_, Ut, v_, sh_; n_snap += 1
                if r < 0.6 * PROP_R: used_axis.add((k, side)); used_axis.add((int(np.argmin([math.hypot(T_[j, 0] - a_, T_[j, 1] - b_) for a_, b_ in mxy])), side))
        shell.snapped = n_snap
    # symmetric triangulation (Spark 27 Oct: perfect left-right symmetry): the hull's faces are merged into flat facets
    # (every vertex within 0.3 of the face plane); a facet and its mirror facet take one triangulation, mirrored; a facet
    # on the centre plane keeps its triangles if they are their own mirror, else is fanned from a node at its centre
    # (on the plane and in the facet, so the hull is unchanged)
    h = ConvexHull(N); P = partners(N); S = h.simplices; nS = len(S)
    par = list(range(nS))
    def root(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    for fi in range(nS):
        for fj in h.neighbors[fi]:
            opp = [v for v in S[fj] if v not in S[fi]]
            if opp and abs(N[opp[0]] @ h.equations[fi, :3] + h.equations[fi, 3]) < 0.3: par[root(fi)] = root(fj)
    facets = {}
    for fi in range(nS): facets.setdefault(root(fi), []).append(fi)
    facets = [fs for fs in facets.values() if not (np.unique(S[fs]) < nR).all()]          # the opening ring's own facet is the hole
    key = {frozenset(np.unique(S[fs]).tolist()): k for k, fs in enumerate(facets)}
    tris = []; extraN = []
    for k, fs in enumerate(facets):
        vs = np.unique(S[fs]); km = key.get(frozenset(P[vs].tolist()))
        assert km is not None, "a facet has no mirror facet"
        T_ = [tuple(sorted(int(v) for v in S[fi])) for fi in fs]
        if km != k:
            if N[vs, 1].mean() > 0: tris += T_ + [tuple(sorted(int(P[v]) for v in t)) for t in T_]
            continue
        if {tuple(sorted(int(P[v]) for v in t)) for t in T_} == set(T_): tris += T_; continue
        cnt = {}
        for t in T_:
            for i in range(3): e2 = tuple(sorted((t[i], t[(i + 1) % 3]))); cnt[e2] = cnt.get(e2, 0) + 1
        c = N[vs].mean(0); c[1] = 0.0
        onp = [int(v) for v in vs if abs(N[v, 1]) < 0.5 and np.linalg.norm(N[v] - c) < 60.0]
        if onp: ci = min(onp, key=lambda v: np.linalg.norm(N[v] - c))  # a vertex on the plane near the centre: the fan's apex (no short tube)
        else: ci = len(N) + len(extraN); extraN.append(c)
        tris += [tuple(sorted((ci, a_, b_))) for (a_, b_), n_ in cnt.items() if n_ == 1 and ci not in (a_, b_)]
    shell.lastN = N.copy()
    if extraN: N = np.vstack([N, extraN])
    tris = np.array(sorted(set(tris)))
    used = sorted(set(tris.ravel())); remap = {o: i for i, o in enumerate(used)}
    nodes = N[used]; tris = np.array([[remap[a] for a in f] for f in tris])
    E = sorted({tuple(sorted((int(f[i]), int(f[(i + 1) % 3])))) for f in tris for i in range(3)})
    ring_ids = [remap[i] for i in range(nR) if i in remap]
    return nodes, E, tris, ring_ids, float(viol.max()), K


def prop_grids(g):
    """per prop: annulus points 0.2 .. 1.0 R (plan), weight r x r (area x loading ~ r, E), and the mask of points the arm
    and motor already shadow (a member there costs no extra thrust)"""
    from matplotlib.path import Path
    rr = np.linspace(0.2, 1.0, 24) * PROP_R; th = np.linspace(0, 2 * np.pi, 96, endpoint=False)
    Rg, Tg = np.meshgrid(rr, th); w = (Rg ** 2).ravel()
    arm_paths = [Path(a.vertices[ConvexHull(a.vertices[:, :2]).vertices, :2]) for a in g["arms"]]
    out = []
    for mx, my in g["mxy"]:
        P = np.stack([mx + (Rg * np.cos(Tg)).ravel(), my + (Rg * np.sin(Tg)).ravel()], 1)
        shade = np.zeros(len(P), bool)
        for pth in arm_paths: shade |= pth.contains_points(P)
        out.append((P, w, shade))
    return out


def seg_shadow(grids, p0, p1, rad):
    """weighted share (% of the 4 disks) a member's plan shadow adds above (z > prop plane) and below, outside the arms'"""
    top = bot = 0.0; d = p1[:2] - p0[:2]; L2 = float(d @ d)
    for P, w, shade in grids:
        t = np.clip(((P - p0[:2]) @ d) / L2, 0, 1) if L2 > 1e-9 else np.zeros(len(P))
        hit = (np.linalg.norm(P - (p0[:2] + t[:, None] * d), axis=1) <= rad) & ~shade
        z = p0[2] + t * (p1[2] - p0[2])
        top += (w * (hit & (z > PROP_Z))).sum() / w.sum(); bot += (w * (hit & (z < PROP_Z))).sum() / w.sum()
    return 100 * top / len(grids), 100 * bot / len(grids)


def blockage(g, nodes, E, extra, parts=False):
    """share of the props' thrust shadowed in plan by shell tubes, nodes and links above and below, outside the arm and motor
    shadow: annulus 0.2 .. 1.0 R, thrust per area ~ r (E); (top %, bottom %) over the 4 props (+ shell / links split)"""
    grids = prop_grids(g)
    def mark(segs, pts_):
        bt = [np.zeros(len(P), bool) for P, _, _ in grids]; bb = [np.zeros(len(P), bool) for P, _, _ in grids]
        for j, (P, w, shade) in enumerate(grids):
            for p0, p1, rad in segs:
                d = p1[:2] - p0[:2]; L2 = float(d @ d)
                t = np.clip(((P - p0[:2]) @ d) / L2, 0, 1) if L2 > 1e-9 else np.zeros(len(P))
                hit = (np.linalg.norm(P - (p0[:2] + t[:, None] * d), axis=1) <= rad) & ~shade
                z = p0[2] + t * (p1[2] - p0[2]); bt[j] |= hit & (z > PROP_Z); bb[j] |= hit & (z < PROP_Z)
            for n in pts_:
                hit = (np.linalg.norm(P - n[:2], axis=1) <= NODE_R) & ~shade
                bt[j] |= hit & (n[2] > PROP_Z); bb[j] |= hit & (n[2] < PROP_Z)
        return bt, bb
    pct = lambda m: 100 * np.mean([(w * mm).sum() / w.sum() for (P, w, _), mm in zip(grids, m)])
    sh = [(nodes[a], nodes[b], SHELL_R) for a, b in E]; ln = [(np.asarray(p, float), np.asarray(q, float), lr(k)) for k, (p, q) in extra.items()]
    bt, bb = mark(sh + ln, nodes)
    if not parts: return pct(bt), pct(bb)
    st, sb = mark(sh, nodes); lt, lb = mark(ln, [])
    return pct(bt), pct(bb), dict(shell_top=pct(st), shell_bottom=pct(sb), links_top=pct(lt), links_bottom=pct(lb))


def views_clear(g, p0, p1, rad):
    """a member stays out of the L2 field (behind its base plane) and out of both camera views (+2 deg)"""
    q = p0[None] + np.linspace(0, 1, 60)[:, None] * (p1 - p0)[None]
    return bool(((q - g["B"]) @ A < -(rad + 1.0)).all() and not in_cam_view(g, q).any())


def halo(g, nodes):
    """L2 guard ring: 8 nodes round its axis, HALO_BACK behind its base plane, r HALO_R; the rods a camera would see
    are left out; each kept node 2 struts to the nearest shell nodes (props 12 off, views clear)"""
    up_b = np.array([-A[2], 0, A[0]]); c = g["B"] - A * HALO_BACK; ey = np.array([0, 1.0, 0])
    H = [c + HALO_R * (math.cos(t) * up_b + math.sin(t) * ey) for t in np.linspace(0, 2 * math.pi, 8, endpoint=False) + math.pi / 8]
    rods = {}
    for i in range(8):
        a, b = H[i], H[(i + 1) % 8]
        if views_clear(g, a, b, ROD_R): rods[f"halo {i}"] = (a, b)
    used = sorted({i for k in rods for i in (int(k.split()[1]), (int(k.split()[1]) + 1) % 8)})
    for i in used:
        for n_, k in enumerate([k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - H[i]))
                                if seg_band_dist(H[i], nodes[k], g["mxy"]) - ROD_R >= PROP_GAP and views_clear(g, H[i], nodes[k], ROD_R)][:2]):
            rods[f"halo strut {i}.{n_ + 1}"] = (H[i], nodes[k])
    return rods, [H[i] for i in used]


def attach(g, nodes, ring_ids):
    mxy = g["mxy"]; extra = {}
    pad = [[-SHELL_R - 1] * 3, [SHELL_R + 1] * 3]                       # a link may be sized up to 6 mm tube (link_sizes.json)
    boxes_ = [m_.bounds + pad for m_ in (g["cart"], TOP, FC, PI, DECK, PDB, UBEC, BUCK, RX)]   # links clear of the cartridge, top plate, hub

    keys_ = [(np.array([r_ * math.cos(math.radians(d_)) - v_ * math.sin(math.radians(d_)), r_ * math.sin(math.radians(d_)) + v_ * math.cos(math.radians(d_))]), z_, sg)
             for d_ in (45, 135, 225, 315) for r_, vs_, z_, sg in ((63.74, (10.0, -10.0), -1.7, -1), (63.17, (15.0, -15.0), ROOT_H + 1.6, 1)) for v_ in vs_]

    def clear_of_cart(p0, p1):
        """links clear of the cartridge, top plate and hub boxes and of the arm screws' key lines (d 8 + 1, below and above)"""
        q = p0[None] + np.linspace(0.02, 1, 80)[:, None] * (p1 - p0)[None]
        if any(((q > b_[0]) & (q < b_[1])).all(1).any() for b_ in boxes_): return False
        return not any(((sg * (q[:, 2] - z_) > 0) & (np.linalg.norm(q[:, :2] - c_, axis=1) < 4.0 + 1.0 + ROD_R)).any() for c_, z_, sg in keys_)
    grids = prop_grids(g)

    def pick(p0, cands, n, spread_deg):
        """n links from p0 among the 10 nearest valid candidates: least added prop shadow (Spark 26 Oct, secondary to
        structure: the links still spread at least spread_deg apart), length as the tie-break"""
        cands = sorted(cands, key=lambda k: np.linalg.norm(nodes[k] - p0))[:10]
        cost = {k: sum(seg_shadow(grids, p0, nodes[k], ROD_R)) + 0.002 * np.linalg.norm(nodes[k] - p0) for k in cands}
        out = []
        for k in sorted(cands, key=cost.get):
            u = (nodes[k] - p0) / np.linalg.norm(nodes[k] - p0)
            if all(np.degrees(np.arccos(np.clip(u @ v, -1, 1))) >= spread_deg for v in [(nodes[j] - p0) / np.linalg.norm(nodes[j] - p0) for j in out]):
                out.append(k)
            if len(out) == n: break
        return out
    for i, ((mx, my), (r0, m1, ht)) in enumerate(zip(mxy, g["axes"])):  # 3 spokes per arm tip, 12 off the disks
        tip = np.array([mx, my, ROOT_H - ht])
        an = tip + np.array(ARM_NODE["front" if mx > g["dm"] else "rear"]) * [1, np.sign(my), 1]   # the arm's printed spoke node
        for n_, k in enumerate(pick(an, [k for k in range(len(nodes)) if seg_band_dist(an, nodes[k], mxy) - ROD_R >= PROP_GAP
                                          and views_clear(g, an, nodes[k], ROD_R)], 3, 30.0)):
            extra[f"spoke {i + 1}.{n_ + 1}"] = (an, nodes[k])
    bb = BOTTOM.bounds                                                  # core ties: bottom plate underside to the shell
    for nm, q0 in (("front", (bb[1][0], 0)), ("rear", (bb[0][0], 0)), ("left", (0, bb[1][1])), ("right", (0, bb[0][1]))):
        q = np.array([q0[0], q0[1], -1.0])
        for n_, k in enumerate(pick(q, [k for k in range(len(nodes)) if k not in ring_ids and seg_band_dist(q, nodes[k], mxy) - ROD_R >= PROP_GAP
                                        and clear_of_cart(q, nodes[k]) and views_clear(g, q, nodes[k], ROD_R)], 2, 25.0)):
            extra[f"core tie {nm}.{n_ + 1}"] = (q, nodes[k])
    B = g["B"]; up_b = np.array([-A[2], 0, A[0]])                       # L2 apex lugs to the 2 nearest nodes behind its plane
    for sy in (1, -1):
        sock = B - A * 5 + up_b * 16 + np.array([0, 46.0 * sy, 0])
        for n_, k in enumerate([k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - sock))
                                if (nodes[k] - B) @ A < -1.0 - NODE_R and seg_band_dist(sock, nodes[k], mxy) - ROD_R >= PROP_GAP
                                and views_clear(g, sock, nodes[k], ROD_R)][:2]):
            extra[f"L2 tie {sy:+d}.{n_ + 1}"] = (sock, nodes[k])
    sb = g["sled"].bounds                                               # belly sled corners to the 2 nearest ring nodes clear of the cartridge
    for sx in (1, -1):
        for sy in (1, -1):
            cn = np.array([-DC + sx * (sb[1][0] - sb[0][0]) / 2, sy * sb[1][1], sb[0][2]])
            clear = lambda k: clear_of_cart(cn, nodes[k])
            for n_, k in enumerate([k for k in sorted(ring_ids, key=lambda k: np.linalg.norm(nodes[k] - cn)) if clear(k)][:2]):
                extra[f"bay tie {sx:+d}{sy:+d}.{n_ + 1}"] = (cn, nodes[k])
    # L2 braces (27 Oct review: the 4 ties land on the flat front face and the keels root 24 apart, a stalk): each apex lug,
    # low and high, also to its side's front arm beam just outboard of the plates, so the seat is a braced pyramid on the core
    g["braces"] = []
    for sy in (() if HEAD_ON_CAGE else (1, -1)):
        (mx, my), (r0, m1, ht) = next(((m_, a_) for m_, a_ in zip(mxy, g["axes"]) if m_[0] > g["dm"] and m_[1] * sy > 0))
        u = (np.asarray(m1) - r0) / np.linalg.norm(np.asarray(m1) - r0)
        for out_ in (30.0, 40.0, 50.0, 60.0):
            root = np.array([*(r0 + out_ * u), ROOT_H + ROD_R])
            ok_ = True
            for lvl, upz in (("lo", 2.0), ("hi", 16.0)):
                s_ = B - A * 5 + up_b * upz + np.array([0, 46.0 * sy, 0])
                ok_ &= seg_band_dist(s_, root, mxy) - ROD_R >= PROP_GAP and views_clear(g, s_, root, ROD_R) and clear_of_cart(s_ + 0.05 * (root - s_), root)
            if ok_:
                for lvl, upz in (("lo", 2.0), ("hi", 16.0)):
                    rt = np.array([BRACE_ROOT[0][lvl], BRACE_ROOT[1][lvl] * sy, BRACE_ROOT[2][lvl]]) if BRACE_ROOT else root
                    extra[f"L2 brace {lvl} {sy:+d}"] = (B - A * 5 + up_b * upz + np.array([0, 46.0 * sy, 0]), rt)
                g["braces"].append(round(out_, 0)); break
    if not HEAD_ON_CAGE:                                               # the keels' core ends sit 5 deeper in the hub deck's sockets (core v2, 27 Oct)
        for k_, (a_, b_) in g["keels"].items():
            a_, b_ = np.asarray(a_, float), np.asarray(b_, float); cr, lg = (a_, b_) if a_[0] < b_[0] else (b_, a_)
            extra[k_] = (cr + (lg - cr) / np.linalg.norm(lg - cr) * KEEL_INSET, lg)
    hr, hn = halo(g, nodes); extra.update(hr); g["halo_nodes"] = hn
    cr, cn_ = cam_guards(g, nodes); extra.update(cr); g["halo_nodes"] = hn + cn_

    # perfect left-right symmetry (Spark 27 Oct) for the links too: a link without its mirror gets it
    mir = np.array([1.0, -1.0, 1.0]); have = [(np.asarray(a_), np.asarray(b_)) for a_, b_ in extra.values()]
    for k_, (a_, b_) in list(extra.items()):
        a_, b_ = np.asarray(a_) * mir, np.asarray(b_) * mir
        if not any(min(np.linalg.norm(a_ - p_) + np.linalg.norm(b_ - q_), np.linalg.norm(a_ - q_) + np.linalg.norm(b_ - p_)) < 1.0 for p_, q_ in have):
            extra[k_ + " m"] = (a_, b_); have.append((a_, b_))
    g["head_attach"], g["head_guards"] = [], {}
    if PRINTED_HEAD:                                                   # the head's short rods out: the printed head carries the guards (ring, camera
        head = {k: extra.pop(k) for k in list(extra) if k.startswith(("L2 tie", "halo", "cam guard"))}   # bars) and reaches the cage nodes they reached
        g["head_attach"] = sorted({tuple(np.round(np.asarray(b_, float), 2)) for k, (a_, b_) in head.items() if "strut" in k or "tie" in k})
        if HEAD_ONE and not HEAD_ON_CAGE:                              # the one-piece head: its own node + 4 struts (head_mount.HEAD_STRUTS)
            cn_ = [k_ for k_ in range(len(nodes)) if abs(nodes[k_][1]) < 0.5 and nodes[k_][0] > 300]
            g["head_attach"] = [tuple(np.round(nodes[max(cn_, key=lambda k_: nodes[k_][0])], 2))]
            for w_, q_ in HEAD_STRUTS:
                for sy in (1, -1):
                    k_ = int(np.argmin([np.linalg.norm(nodes[j_] - np.array(q_) * [1, sy, 1]) for j_ in range(len(nodes))]))
                    extra[f"head strut {k_}"] = (np.array(w_) * [1, sy, 1], nodes[k_]); g["head_attach"].append(tuple(np.round(nodes[k_], 2)))
        if HEAD_ON_CAGE:                                               # the front centre node only (the head's own node, printed in it)
            g["head_attach"] = [max(g["head_attach"], key=lambda q_: q_[0] - 10 * abs(q_[1]))]
        g["head_guards"] = {k: v for k, v in head.items() if not ("strut" in k or "tie" in k)}
        g["head_budget"] = (sum(np.linalg.norm(np.subtract(b_, a_)) for a_, b_ in head.values()) * ROD_GPM + 1.5 * len(g["halo_nodes"]) + CAM_BRACKET_G)
        g["head_topo"] = max(0.0, g["head_budget"] - 1.5 * len(g["halo_nodes"])
                             - sum(np.linalg.norm(np.subtract(q_, p_)) for p_, q_ in g["head_guards"].values()) * 36 * 1.27e-3 * 0.8)
    return extra


def cam_guards(g, nodes):
    """a guard over each camera (the cameras stand ahead of the shell at the nose, like the L2): a bar of 2 printed nodes over
    its top, behind its lens (out of both views) and behind the L2 base plane, each node 2 struts to the nearest shell nodes
    (props 12 off, views clear); the left one is built, the right one mirrored"""
    c = g["cams"][0]; rods, T = {}, []
    for a_ in (-16.0, 16.0):
        q = c["lens"] - c["f"] * (HALO_NODE_R + 4.0) + c["u"] * (20.0 + 3.0 + HALO_NODE_R) + c["r"] * a_
        T.append(q)
    if not (views_clear(g, T[0], T[1], ROD_R) and ((np.stack(T) - g["B"]) @ A < -(HALO_NODE_R + 1.0)).all()): return rods, []
    rods["cam guard bar"] = (T[0], T[1])
    for i, t_ in enumerate(T):
        for n_, k in enumerate([k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - t_))
                                if seg_band_dist(t_, nodes[k], g["mxy"]) - ROD_R >= PROP_GAP and views_clear(g, t_, nodes[k], ROD_R)][:2]):
            rods[f"cam guard strut {i}.{n_ + 1}"] = (t_, nodes[k])
    return rods, T + [t_ * [1, -1, 1] for t_ in T]


def masses(g, nodes, E, extra):
    it = []; add = lambda n, m, c: it.append((n, float(m), np.asarray(c, float))); cen = lambda m: m.bounding_box.centroid
    add("L2", 230, cen(g["l2"])); [add(f"camera {'left' if c['side'] > 0 else 'right'}", 15, cen(c["mesh"])) for c in g["cams"]]
    for n, gm, m in (("Pi 5", 55, PI), ("FC", 10, FC), ("PDB", 21, PDB), ("UBEC", 15, UBEC), ("buck", 6, BUCK), ("RX", 2, RX),
                     ("top plate", 23, TOP), ("bottom plate", 61, BOTTOM), ("wires", 80, FC)):
        add(n, gm, cen(m))
    add("pack", 650, cen(g["pack"])); add("flow", 2, cen(g["flow"])); add("latch set", LATCH_G, cen(g["cart"]))
    for i, m in enumerate(g["motors"]): add(f"motor + prop {i + 1}", 91, cen(m))
    for i, a in enumerate(g["arms"]): add(f"arm {i + 1}", abs(a.volume) * 1.27e-3 * 0.45 + 25, a.centroid)
    ah = apex_head(g); add("apex", abs(ah.volume) * 1.27e-3 * 0.45, ah.centroid)   # the front hub block is the core's
    add("cam plate", abs(g["cam_plate"].volume) * 1.27e-3 * 0.5, g["cam_plate"].centroid)
    for k, (a, b) in enumerate(g["cam_rods"]): add(f"cam strut {k}", np.linalg.norm(np.subtract(b, a)) * 36 * 1.27e-3 * 0.8, np.add(a, b) / 2)   # printed (E)
    add("Pi deck", abs(DECK.volume) * 1.27e-3 * 0.5, DECK.centroid)
    for nm in ("sled", "flow_pod"): add(nm, abs(g[nm].volume) * 1.27e-3 * 0.3, g[nm].centroid)
    for i, f in enumerate(g["feet"]): add(f"foot {i}", abs(f.volume) * 1.21e-3 * 0.5, f.centroid)
    for i, (a, b) in enumerate(E): add(f"shell {i}", np.linalg.norm(nodes[a] - nodes[b]) * SHELL_GPM, (nodes[a] + nodes[b]) / 2)
    for k, (p, q) in extra.items(): add(k, np.linalg.norm(np.subtract(q, p)) * lgpm(k), (np.asarray(p) + np.asarray(q)) / 2)
    for i, n in enumerate(nodes): add(f"node {i}", NODE_G, n)
    for i, n in enumerate(g.get("halo_nodes", [])): add(f"halo node {i}", 1.5, n)            # printed, r 5 (E)
    for k, (p, q) in g.get("head_guards", {}).items(): add(f"head guard {k}", np.linalg.norm(np.subtract(q, p)) * 36 * 1.27e-3 * 0.8, (np.asarray(p) + np.asarray(q)) / 2)
    if g.get("head_attach"):                                           # the topology's joining material (E) between the head and the cage nodes
        add("head topology", g["head_topo"], 0.5 * (g["B"] - A * 20) + 0.5 * np.mean(g["head_attach"], 0))
    return it


def cg(it):
    m = sum(x for _, x, _ in it); return m, sum(x * c for _, x, c in it) / m


def solve(dm=52.1):
    """v2's balance: the motor square (thrust centre) onto the CG, the shell rebuilt round the drone each time"""
    global DC
    def run(dm, warm=None):
        global DC
        if PACK_AXIS: DC = -dm
        g = drone(dm)
        nodes, E, tris, ring_ids, viol, K = shell(g, warm=warm); g["shellN"] = shell.lastN
        extra = attach(g, nodes, ring_ids); it = masses(g, nodes, E, extra); tot, c = cg(it)
        say(f"   motors {dm:5.1f} ahead: CG x {c[0]:5.1f}, {tot:5.0f} g, {len(E)} tubes")
        return g, nodes, E, tris, ring_ids, extra, it, tot, c, viol, K
    seen = []                                                   # (dm, result): the shell is discrete, so the CG can jump
    for _ in range(14):
        r = run(dm); seen.append((dm, r)); c = r[8]
        if abs(c[0] - dm) < 0.1: return r
        lo = [d_ for d_, q in seen if q[8][0] > d_]; hi = [d_ for d_, q in seen if q[8][0] < d_]
        if lo and hi and any(abs(d_ - dm) < 1e-6 for d_, _ in seen[:-1]) or len(seen) > 6 and lo and hi:
            break                                               # a 2-cycle: bisect between the shells
        dm = c[0]
    for _ in range(8):
        lo = max(d_ for d_, q in seen if q[8][0] > d_); hi = min(d_ for d_, q in seen if q[8][0] < d_ and d_ > lo) if any(q[8][0] < d_ and d_ > lo for d_, q in seen) else None
        if hi is None or hi - lo < 0.05: break
        dm = 0.5 * (lo + hi); r = run(dm); seen.append((dm, r))
        if abs(r[8][0] - dm) < 0.1: return r
    dm, r = min(seen, key=lambda q: abs(q[1][8][0] - q[0]))
    # the shell pattern jumps with the motor square (a node more or less): keep the closest one's pattern and trim the
    # motor square onto the CG with it (nodes re-pushed round the moved parts, no new pattern)
    for _ in range(10):
        if abs(r[8][0] - dm) < 0.1 or not np.isfinite(r[9]) or FZ is not None: break
        dm = r[8][0]; r2 = run(dm, warm=r[0]["shellN"])
        if not np.isfinite(r2[9]) or (r2[9] > 0.05 and FZ is None): break
        r = r2
    return r


os.makedirs(OUT, exist_ok=True)
# the cage structure is kept (Spark 27 Oct: "don't change the structure"): the 27 Oct pick of the full scan (start 0.85 x dome 60 x
# prop snap) is the default; V5_STARTS=0.7,0.85,1.0 V5_DOMES=40,60,80 V5_SNAPS=0,1 runs the scan again
STARTS = [float(x) for x in os.environ.get("V5_STARTS", "0.85").split(",")] if "V5_START" not in os.environ else [START]
ROTS = [float(x) for x in os.environ.get("V5_ROTS", "0").split(",")]     # 36 was worse in every case (26 Oct scan)
PEAKS = [bool(int(x)) for x in os.environ.get("V5_PEAKS", "0").split(",")]   # 1 (nodes at the nose camera peak): every case over 400 tubes (26 Oct)
SNAPS = [bool(int(x)) for x in os.environ.get("V5_SNAPS", "1").split(",")]
DOMES = [float(x) for x in os.environ.get("V5_DOMES", "60").split(",")] if "V5_DOME" not in os.environ else [DOME]
say(f"balance (v2: motor square onto the CG; catch {DC:.0f} aft, L2 base z {ZB:.0f}; geodesic frequency {FREQ}); scan start {STARTS} x "
    f"prop snap {SNAPS} x dome {DOMES}: structure first (the most compact valid shell sets the bar: volume, mass; no tube-length limit), "
    f"then the least prop blockage among shells within 10 % / 3 % of it:")
cands = []
for START in STARTS:
    for ROT in ROTS:
        for SNAP in SNAPS:
          for PEAK in PEAKS:
           for DOME in DOMES:
            r = solve()
            g_, nodes_, E_ = r[0], r[1], r[2]
            tl = max(np.linalg.norm(nodes_[a] - nodes_[b]) for a, b in E_); vol = ConvexHull(nodes_).volume / 1e6
            pg_ = min(seg_band_dist(nodes_[a], nodes_[b], g_["mxy"]) for a, b in E_) - SHELL_R
            ok = bool((FZ is not None or np.max(r[9]) <= 0.05) and len(r[4]) == (len(FZ["open"][0]["ring"]) if FZ is not None else 8) and pg_ >= PROP_GAP - 0.05)   # no tube-length limit (Spark 26 Oct); a frozen
                                                                                                   # cage is held to the props and system_check instead
            bt, bb = blockage(g_, nodes_, E_, r[5]) if ok else (np.nan, np.nan)
            say(f"start {START} dome {DOME:.0f} snap {int(SNAP)}: {'valid' if ok else 'invalid'}: shell {vol:.1f} L, {len(E_)} tubes, longest {tl:.0f}, "
                f"props {pg_:.1f}, {r[7]:.0f} g, prop blockage top {bt:.1f} % / bottom {bb:.1f} %")
            if ok: cands.append(dict(vol=vol, tl=tl, mass=r[7], block=bt + bb, bt=bt, bb=bb, start=START, rot=ROT, snap=SNAP, peak=PEAK, dome=DOME, r=r))
ref = min(cands, key=lambda c: c["vol"])
fit = [c for c in cands if c["vol"] <= 1.1 * ref["vol"] and c["mass"] <= 1.03 * ref["mass"]]
pick = min(fit, key=lambda c: c["block"])
say(f"structure bar: start {ref['start']} dome {ref['dome']:.0f} snap {int(ref['snap'])} ({ref['vol']:.1f} L, longest {ref['tl']:.0f}, {ref['mass']:.0f} g, "
    f"blockage {ref['bt']:.1f} / {ref['bb']:.1f} %); picked start {pick['start']} dome {pick['dome']:.0f} snap {int(pick['snap'])} "
    f"(blockage {pick['bt']:.1f} / {pick['bb']:.1f} %)")
START, ROT, SNAP, PEAK, DOME = pick["start"], pick["rot"], pick["snap"], pick["peak"], pick["dome"]
best = (pick["vol"], START, pick["r"])
_, START, (g, nodes, E, tris, ring_ids, extra, it, tot, cgv, viol, K) = best
dm = g["dm"]; mxy = g["mxy"]

# ---------------------------------------------------------------- checks
say(f"\nchosen: start {START} (smallest valid shell); motor square {dm:.1f} ahead of the plate centre; motors ({mxy[0][0]:.1f}, ±{HALF}) and ({mxy[1][0]:.1f}, ±{HALF}); "
    f"front arms r {math.hypot(*mxy[0]):.0f}, rear r {math.hypot(*mxy[1]):.0f}")
cs = g["cam_sel"]; ha_, hv_ = CAM_FMTS[cs["fmt"]]
say(f"L2: base ({g['xB']:.1f}, 0, {ZB:.0f}), axis 15 deg below +x (Spark 24 Oct)")
say(f"cameras (Spark 27 Oct: no down camera; the two on either side of the L2, their field maximized): {cs['fmt']} ({2 * ha_:.0f} across x {2 * hv_:.0f} up), "
    f"yawed {cs['psi']:.1f} deg out, pitched {cs['th']:+.0f}; lenses ({g['cams'][0]['lens'][0]:.0f}, ±{g['cams'][0]['lens'][1]:.0f}, {g['cams'][0]['lens'][2]:.0f}), "
    f"{cs['w']:.0f} off the centre plane, {cs['bk']:.0f} behind the L2 base plane (the most compact of {cs['mounts']} clear mounts); combined field "
    f"{cs['U']:.2f} sr ({100 * cs['U'] / (4 * math.pi):.1f} % of the sphere; every pair whose fields do not overlap gets this, within the sampling), "
    f"blind wedge ahead on the horizon {cs['gap']:.1f} deg (the tie-break: the fields meet at the nose), {100 * cs['S']:.0f} % of the L2 field (C); "
    f"printed bracket: a plate on 3 struts 6 x 6 from 3 corners to its side's lugs and block")
um_ = max(t["U"] for t in cam_table())
CAMALT = [f"{t['fmt']} yaw {t['psi']:.1f} pitch {t['th']:+.0f}: {t['U']:.2f} sr, gap {t['gap']:.1f}, {100 * t['S']:.0f} % of the L2 field"
          for t in sorted(cam_table(), key=lambda t: -t["S"])[:1] + [t for t in cam_table() if t["fmt"] != cs["fmt"]][:1]]
say("   camera alternatives: " + "; ".join(CAMALT))
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
say(f"carbon: shell {len(sh) - len(short)} tubes 6/4 (Spark 26 Oct), shortest {min(l for l in sh if l >= BREAK_EVEN):.0f}; links {sum(v >= BREAK_EVEN for v in rl.values())} ({sum(v >= BREAK_EVEN and k in LINK6 for k, v in rl.items())} tube 6/4, the rest rod 3.0), "
    f"{sum(rl.values()) / 1000:.2f} m, shortest {min(rl.values()):.0f} ({min(rl, key=rl.get)}); printed struts (< {BREAK_EVEN:.0f}): "
    + (", ".join(f"shell {i} {l:.0f}" for i, l in short) + ", " if short else "") + (", ".join(f"{k} {v:.0f}" for k, v in rl.items() if v < BREAK_EVEN) or "none"))
pg = min(seg_band_dist(nodes[a], nodes[b], mxy) for a, b in E) - SHELL_R
pn = min(seg_band_dist(n, n, mxy) for n in nodes) - NODE_R
pe_all = {k: seg_band_dist(np.asarray(p, float), np.asarray(q, float), mxy) - ROD_R for k, (p, q) in extra.items()}
say(f"props: shell tubes {pg:.1f} off the swept disks (≥ 12, 3D), nodes {pn:.1f}, links {min(pe_all.values()):.1f} ({min(pe_all, key=pe_all.get)}) (C)")
eqs = ConvexHull(nodes).equations
for nm, ms in (("props (disks)", g["props"]), ("arms + motors", g["arms"] + g["motors"]),
               ("hub + plates", [TOP, BOTTOM, PI, DECK, PDB]), ("belly", [g["cart"], g["sled"], g["flow_pod"]])):
    p = np.vstack([pts(m, 2000) for m in ms]); p = p[p[:, 2] > g["z_o"]]
    say(f"   inside: {nm} {float((-(p @ eqs[:, :3].T + eqs[:, 3])).min(1).min()):.1f} from the nearest face plane (C)")
rc = nodes[ring_ids]
say(f"opening: {len(ring_ids)} nodes, a ring {np.ptp(rc[:, 0]):.0f} x {np.ptp(rc[:, 1]):.0f} round the swap zone {2 * ZONE[0]:.0f} x {2 * ZONE[1]:.0f} "
    f"(receiver {REC[0]} x {REC[1]} M + {SPARE:.0f} a side), plane z {g['z_o']:.1f} = cartridge bottom - 3; v2 TPU feet the floor contact (z {min(f.bounds[0][2] for f in g['feet']):.1f})")
zone = box(-DC - ZONE[0], -DC + ZONE[0], -ZONE[1], ZONE[1], g["z_o"] - 200, g["cart"].bounds[0][2])
blk = [(i, overlap(zone, rod(nodes[a], nodes[b], SHELL_R))) for i, (a, b) in enumerate(E)]
blk = [(i, v) for i, v in blk if v > 0.5] + [(k, overlap(zone, rod(p, q))) for k, (p, q) in extra.items() if overlap(zone, rod(p, q)) > 0.5]
say("swap zone (receiver + spare, from the cartridge bottom down): " + ("clear (0 mm³, M)" if not blk else "; ".join(f"{k} {v:.0f} mm³" for k, v in blk)))
Bc = g["B"] + A * 40.0
rng = np.random.default_rng(1); D = rng.normal(size=(20000, 3)); D /= np.linalg.norm(D, axis=1)[:, None]; D = D[D @ A > 0]
hit = np.zeros(len(D), bool)
for p0, p1, rr in [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), lr(k)) for k, (p, q) in extra.items()]:
    u = p1 - p0; w0 = Bc - p0; bq = D @ u; cq = float(u @ u); dq = D @ w0; eq_ = float(u @ w0)
    sq = np.clip((eq_ - bq * dq) / np.maximum(cq - bq * bq, 1e-9), 0, 1); tq = -dq + sq * bq
    hit |= (np.linalg.norm(Bc + tq[:, None] * D - (p0 + sq[:, None] * u), axis=1) < rr) & (tq > 0)
for n in nodes:
    v = n - Bc; t = D @ v; hit |= (np.linalg.norm(v - t[:, None] * D, axis=1) < NODE_R) & (t > 0)
BT, BB, BS = blockage(g, nodes, E, extra, parts=True)
say(f"prop blockage (Spark 26 Oct, secondary to structure): {BT:.1f} % of the disks' thrust shadowed from above, {BB:.1f} % from below "
    f"(plan shadow of tubes, nodes and links outside the arms' own shadow, annulus 0.2..1 R weighted by r, E); shell {BS['shell_top']:.1f} / {BS['shell_bottom']:.1f} %, "
    f"links {BS['links_top']:.1f} / {BS['links_bottom']:.1f} %")
memb = [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), lr(k)) for k, (p, q) in extra.items()]
ahead = max([float(((np.stack([p0, p1]) - g["B"]) @ A).max()) + rr for p0, p1, rr in memb] + [float(((nodes - g["B"]) @ A).max()) + NODE_R]
            + [float((np.asarray(h_) - g["B"]) @ A) + HALO_NODE_R for h_ in g["halo_nodes"]])
fw = {k: float(((np.stack([np.asarray(p, float), np.asarray(q, float)]) - g["B"]) @ A).max()) + ROD_R for k, (p, q) in extra.items()}
fw["shell tubes"] = max(float(((np.stack([nodes[a], nodes[b]]) - g["B"]) @ A).max()) + SHELL_R for a, b in E)
fw["shell nodes"] = float(((nodes - g["B"]) @ A).max()) + NODE_R
fw.update({"guard ring nodes": max([float((np.asarray(h_) - g["B"]) @ A) + HALO_NODE_R for h_ in g["halo_nodes"]] or [-99])})
say(f"L2 field (Spark 26 Oct: outside the cage, not blocked at all): {100 * hit.mean():.1f} % of its hemisphere crossed; the most forward cage "
    f"member surface is {-ahead:.1f} behind its base plane ({max(fw, key=fw.get)}) (C)")
cam_seen = []
for k, P_ in (("shell tubes", np.vstack([rod(nodes[a], nodes[b], SHELL_R).vertices for a, b in E])), ("shell nodes", np.vstack([n + NODE_R * DIRS for n in nodes])),
              ("links + guard ring", np.vstack([pts(rod(p0, p1, rr), 200) for p0, p1, rr in memb[len(E):]])), ("props", np.vstack([pts(p_, 3000) for p_ in g["props"]])),
              ("L2", pts(g["l2"], 3000)), ("arms", np.vstack([pts(a_, 2000) for a_ in g["arms"]])),
              ("apex + camera brackets", np.vstack([pts(g["apex"], 3000), pts(g["cam_bracket"], 3000)]))):
    n_ = int(in_cam_view(g, P_, 0.0).sum())
    if n_: cam_seen.append(f"{k} {n_} points")
for c in g["cams"]:                                                     # each camera and the other one
    o_ = [d_ for d_ in g["cams"] if d_ is not c][0]
    n_ = int(in_frustum(c["lens"], c["f"], c["u"], c["r"], c["ha"], c["hv"], pts(o_["mesh"], 3000), 0.0).sum())
    if n_: cam_seen.append(f"the other camera {n_} points")
say("cameras: " + ("nothing of the drone in either view (C)" if not cam_seen else "in view: " + "; ".join(cam_seen)))
if FZ is None:
    say(f"sensor head outside the shell: shell node centres {LIM_L2:.1f} behind the L2 base plane (guard ring in front), {LIM:.1f} beyond each camera's "
        f"limiting view face (+2 deg) and behind the head plane (the head's back, tilted {g['phi']:.0f} deg from the L2 axis toward +z: the tilt that keeps "
        f"the L2 furthest back of {list(PHIS)})")
else:
    say(f"sensor head outside the frozen cage ({os.path.basename(FROZEN)}): "
        f"the L2 base where it was ({FZ['xB']:.1f}, 0, {ZB:.0f}); the cameras' bodies {fz_clear(np.vstack([c['mesh'].vertices for c in g['cams']])):.1f} off its tubes "
        f"and nodes, nothing of it in their views (C)")
if PRINTED_HEAD:
    say(f"L2 mount: the printed sensor head mount{' (one piece with both cameras)' if HEAD_ONE else ''}: seat, back block, lugs, the guard ring (8 nodes, r "
        f"{HALO_R:.0f}, {HALO_BACK:.0f} behind the base plane) and the camera guard bars printed in it; the cage's front centre node in it; on "
        f"{sum(k.startswith('keel') for k in extra)} keels to the front hub and {sum(k.startswith('L2 brace') for k in extra)} braces to the core's hub deck tower "
        f"{'(BRACE_ROOT)' if BRACE_ROOT else str(g.get('braces')) + ' out of the arm root'}, {sum(k.startswith('head strut') for k in extra)} struts to the side cage nodes; break order: the apex lugs are the "
        f"printed fuse (requirement: they let go before the L2 base, E)")
else: say(f"L2 guard ring: {sum(k.startswith('halo ') and 'strut' not in k and 'cam' not in k for k in extra)} of 8 rods (r {HALO_R:.0f}, {HALO_BACK:.0f} behind the base plane; "
          f"the others in a camera view), {sum('halo strut' in k for k in extra)} struts into the shell; L2 mount: v2 seat, back block, lugs, 2 keels to the front hub, "
          f"{sum(k.startswith('L2 tie') for k in extra)} ties into the shell, {sum(k.startswith('L2 brace') for k in extra)} braces from the lugs (low and high) to the front "
          f"arm beams {g.get('braces')} out of the root (a braced pyramid on the core, not a stalk); each camera on a printed plate with 2 rods to its side's lugs; break order: the apex lugs are the "
          f"printed fuse (requirement: they let go before the L2 base, E)")
for lens, hh, hv, nm in ((g["flow_lens"], 23.0, 23.0, "flow"),):
    for k, m in (("shell", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E])), ("cartridge", g["cart"]), ("feet", trimesh.util.concatenate(g["feet"])),
                 ("links", trimesh.util.concatenate([rod(p0, p1, rr) for p0, p1, rr in memb[len(E):]])), ("sled + pod", trimesh.util.concatenate([g["sled"], g["flow_pod"]]))):
        d = pts(m, 4000) - lens; zz = -d[:, 2]
        n_ = ((zz > 0) & (np.abs(np.degrees(np.arctan2(d[:, 0], np.maximum(zz, 1e-6)))) < hh) & (np.abs(np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))) < hv)).mean()
        if n_: say(f"   {k} in the {nm} view ({100 * n_:.1f} % of its points)")
say("flow: view checked (lines above list anything in view)")
shell_g = sh.sum() * SHELL_GPM + NODE_G * len(nodes); link_g = sum(v * lgpm(k) for k, v in rl.items())
say(f"cage: shell {shell_g:.0f} g (tubes {sh.sum() * SHELL_GPM:.0f} E + nodes {NODE_G * len(nodes):.0f} E), links {link_g:.0f} g (C); "
    f"v0 rings ~96 g, v1 sphere 802 + 98 g (E)")
t2, c2 = cg([x for x in it if x[0] not in ("pack",)] + [("cartridge out", -LATCH_G * 0.45, g["cart"].bounding_box.centroid)])
Iyy = sum(m_ * ((c_[0] - cgv[0]) ** 2 + (c_[2] - cgv[2]) ** 2) for _, m_, c_ in it) * 1e-6
say(f"mass ~{tot:.0f} g (E; latch set {LATCH_G} M), CG ({cgv[0]:.1f}, {cgv[1]:.1f}, {cgv[2]:.1f}) vs thrust centre ({dm:.1f}, 0): {math.hypot(cgv[0] - dm, cgv[1]):.1f} off; "
    f"pack + cartridge out: {math.hypot(c2[0] - dm, c2[1]):.1f} off ({t2:.0f} g; layout mass model, the gated number is system_check's)")
say(f"T/W {6600 / tot:.2f} (C on E mass, 1650 gf a motor); CG {PROP_Z - cgv[2]:.0f} under the prop plane; pitch inertia {Iyy:.1f} g m² (E, point masses)")
hullK = ConvexHull(K)
say(f"compact: shell hull {ConvexHull(nodes).volume / 1e6:.1f} L vs the parts-with-margins hull {hullK.volume / 1e6:.1f} L (C)")

SUMMARY = dict(cg_off=float(math.hypot(cgv[0] - dm, cgv[1])), l2_axis=A.tolist(), l2_base=g["B"].tolist(), cams=[dict(lens=c["lens"].tolist(), f=c["f"].tolist(),
               u=c["u"].tolist(), r=c["r"].tolist(), ha=c["ha"], hv=c["hv"]) for c in g["cams"]], cam_sel={k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in g["cam_sel"].items()}, freq=FREQ, start=START, dome=DOME, snap=SNAP, block_parts=BS, block_top=BT, block_bottom=BB, dm=dm, xB=g["xB"], z_o=g["z_o"], nodes=len(nodes), tubes=len(E), ring=len(ring_ids), shell_m=float(sh.sum() / 1000),
               shell_g=float(shell_g), link_g=float(link_g), mass=tot, cg=cgv.tolist(), cg_pack_out=c2.tolist(), extents=ext.tolist(), prop_gap=pg,
               l2_blocked=float(hit.mean()), motors_xy=[list(map(float, m)) for m in mxy], Iyy_gm2=Iyy, largest_cell=max(circ) - 2 * SHELL_R)
SUMMARY["links"] = {k: [np.round(np.asarray(a_), 2).tolist(), np.round(np.asarray(b_), 2).tolist()] for k, (a_, b_) in extra.items()}
json.dump(SUMMARY, open(os.path.join(OUT, "result.json"), "w"), indent=1)
json.dump([dict(name=n_, g=round(float(m_), 3), c=np.round(c_, 2).tolist()) for n_, m_, c_ in it], open(os.path.join(OUT, "masses.json"), "w"))

# ---------------------------------------------------------------- export: parts, assembly file
from assembly import new as new_assembly
PD = os.path.join(OUT, "parts"); os.makedirs(PD, exist_ok=True)
cen_ = lambda m_: m_.bounding_box.centroid
rel = lambda p_: os.path.relpath(p_, ROOT)
node_m = lambda c_: trimesh.creation.icosphere(1, NODE_R).apply_translation(c_)
grp, grp_mc = {}, {}
for n_, gm, _c in it:
    k = ("shell" if n_.startswith("shell") else "nodes" if n_.startswith("node") else ("head" if PRINTED_HEAD else "halo nodes") if n_.startswith("halo node") else "head" if n_.startswith("head ") and not n_.startswith("head strut")
         else "links" if n_.startswith("head strut") or n_.split()[0] in ("spoke", "core", "L2", "bay", "keel", "halo", "cam") and n_ != "L2" and not n_.startswith(("cam plate", "cam strut"))
         else "hub" if n_ in ("Pi 5", "FC", "PDB", "UBEC", "buck", "RX", "Pi deck") else "belly" if n_ in ("sled", "flow_pod") else
         "feet" if n_.startswith("foot") else ("head" if PRINTED_HEAD else "L2 mount") if n_ in ("apex", "cam plate") or n_.startswith("cam strut") else n_)
    grp[k] = grp.get(k, 0.0) + gm; grp_mc[k] = grp_mc.get(k, 0.0) + gm * np.asarray(_c, float)
links = trimesh.util.concatenate([rod(p, q, lr(k)) for k, (p, q) in extra.items()])
PARTS = [  # name, mesh, role, protect, g, tag, material, group, colour, swap
    ("cage shell tubes", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E]), "cage", False, grp["shell"], "E", "carbon", "cage", "#2b2b2b", False),
    ("cage shell nodes", trimesh.util.concatenate([saved_node(n) or node_m(n) for n in nodes]), "cage", False, grp["nodes"], "E", "PETG", "cage", "#7d8a99", False),
    (LINKN, links, "cage", False, grp["links"], "C", "carbon", "cage", "#444444", False),
    ("TPU feet", trimesh.util.concatenate(g["feet"]), "cage", False, grp["feet"], "E", "TPU", "cage", "#555555", False),
    *([(HEADN, trimesh.util.concatenate([apex_head(g), g["cam_bracket"]] + [rod(p_, q_, CAM_STRUT_R) for p_, q_ in g["head_guards"].values()]
                                        + [trimesh.creation.icosphere(1, HALO_NODE_R).apply_translation(h_) for h_ in g["halo_nodes"]]),
         "structure", False, grp["head"], "E", "PETG", "printed", "#aab4c0", False)] if PRINTED_HEAD else
      [("L2 guard ring nodes", trimesh.util.concatenate([trimesh.creation.icosphere(1, HALO_NODE_R).apply_translation(h_) for h_ in g["halo_nodes"]]), "cage", False, grp.get("halo nodes", 0.0), "E", "PETG", "cage", "#7d8a99", False),
       ("apex + cam bracket", trimesh.util.concatenate([g["apex"], g["cam_bracket"]]), "structure", False, grp["L2 mount"], "E", "PETG", "printed", "#aab4c0", False)]),
    ("Unitree L2", g["l2"], "bought", True, 230, "P", None, "bought", "#222222", False),
    ("camera left", g["cams"][0]["mesh"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False),
    ("camera right", g["cams"][1]["mesh"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False),
    ("top plate", TOP, "bought", False, 23, "E", None, "frame", "#d8d8d8", False),
    ("bottom plate", BOTTOM, "bought", False, 61, "E", None, "frame", "#cfcfcf", False),
    ("hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", trimesh.util.concatenate([FC, PI, DECK, PDB, UBEC, BUCK, RX]), "bought", True, grp["hub"], "E", None, "bought", "#2e7d32", False),
    ("wires", FC, "bought", False, 80, "E", None, "bought", "#1b5e20", False),
    ("pack", g["pack"], "bought", True, 650, "E", None, "bought", "#2f4fbf", True),
    ("cartridge", g["cart"], "protected", True, LATCH_G * 0.45, "E", "PETG", "printed", "#99a6b3", True),
    ("receiver", g["cart"], "protected", True, LATCH_G * 0.55, "E", "PETG", "printed", "#8899aa", False),   # stays in the drone (latch set split E)
    ("belly sled + pods", trimesh.util.concatenate([g["sled"], g["flow_pod"]]), "structure", False, grp["belly"], "E", "PETG", "printed", "#99a6b3", False),
    ("flow MTF-02P", g["flow"], "bought", True, 2, "E", None, "bought", "#2d5a2d", False),
]
for i in range(4):
    PARTS += [(f"arm {i + 1}", saved_arm(*g["motors_xy"][i], g["dm"]) or g["arms"][i], "structure", False, grp[f"arm {i + 1}"], "E", "PETG", "printed", "#8899aa", False),
              (f"motor {i + 1}", g["motors"][i], "bought", True, 81, "E", None, "bought", "#b0b0b0", False),
              (f"prop {i + 1}", cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, *mxy[i], 96), "bought", True, 10, "E", None, "bought", "#d9a400", False)]
fname = lambda n_: "".join(ch if ch.isalnum() else "_" for ch in n_.lower()).strip("_")[:40] + ".stl"
for p_ in PARTS:
    p_[1].export(os.path.join(PD, fname(p_[0])))
json.dump([dict(name=n_, file=fname(n_), role=r_, protect=pr_, mass_g=round(float(gm_), 2), tag=t_, group=gr_, color=co_, swap=sw_)
           for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS], open(os.path.join(PD, "parts.json"), "w"), indent=1)
for f_ in os.listdir(PD):                                               # parts no longer in the drone (27 Oct review: a stale down camera)
    if f_.endswith(".stl") and f_ not in {fname(p_[0]) for p_ in PARTS}: os.remove(os.path.join(PD, f_))
say(f"assembly mass {sum(p_[4] for p_ in PARTS):.0f} g (E) vs the layout model {tot:.0f} g")
if PRINTED_HEAD:
    # the sensor head mount's requirements for its topology run (system first, then cut): drawn bodies, the cage nodes it holds,
    # the carbon rods it takes, keep-outs, masses and cases (dimensions.md L), print limits
    HEAD_SYS_G = sum(m_ for n_, m_, _ in it if n_.startswith(("apex", "cam plate", "cam strut", "halo node", "head guard", "head topology")))   # one head mass: the system model's
    HS = dict(note="Overseer sensor head mount (printed PETG, topology): replaces the short rods at the L2 and the cameras (Spark 27 Oct). Drone frame, mm.",
              l2=dict(base=g["B"].tolist(), axis=A.tolist(), mass_g=230, cg=cen_(g["l2"]).tolist(), field="half-space ahead of the base plane, nothing of the mount in it (1 behind)"),
              cameras=[dict(side=c["side"], lens=c["lens"].tolist(), f=c["f"].tolist(), u=c["u"].tolist(), r=c["r"].tolist(), half_across=c["ha"], half_up=c["hv"],
                            mass_g=15, cg=cen_(c["mesh"]).tolist(), view="frustum + 2 deg kept clear; board 38 x 38, M2.5 on 28 x 28 behind it") for c in g["cams"]],
              cage_nodes_held=[list(q_) for q_ in g["head_attach"]], cage_node_r=NODE_R,
              carbon_kept={k: [np.round(np.asarray(v[0]), 2).tolist(), np.round(np.asarray(v[1]), 2).tolist()] for k, v in extra.items() if k.startswith(("keel", "L2 brace"))},
              struts={k: [np.round(np.asarray(v[0]), 2).tolist(), np.round(np.asarray(v[1]), 2).tolist()] for k, v in extra.items() if k.startswith("head strut")},
              guards={k: [np.round(np.asarray(v[0]), 2).tolist(), np.round(np.asarray(v[1]), 2).tolist()] for k, v in g["head_guards"].items()},
              guard_nodes=[np.round(np.asarray(h_), 2).tolist() for h_ in g["halo_nodes"]],
              keep_out=["the L2 and its field", "both cameras and their views (+2 deg)", "the frozen cage's tubes (3 off) except the held nodes",
                        "prop disks 12 off", "the keels and braces (their sockets are on the lugs)"],
              mass_budget_g=round(HEAD_SYS_G, 1), cases="dimensions.md L (flight 4 g / 2 g, crash 35 g in 14 directions at each payload's CG, "
              "sensor mounts first mode >= 100 Hz, breaking order: the lugs let go before the L2 base)",
              print_="Bambu A1 mini 180 cube, PETG; cut DEFAULT: centre piece |y| <= 75 (seat, lugs, guard ring, the centre node) and two mirrored wings "
                     "(camera pad, camera guard, the side nodes), bolted M3 on drawn pads",
              drawn_mesh="parts/" + fname(HEADN))
    json.dump(HS, open(os.path.join(OUT, "head_system.json"), "w"), indent=1)
    say(f"sensor head mount (Spark 27 Oct: a topology printed mount instead of the little rods): drawn seat, lugs, camera pads, guard ring and camera "
        f"guard bars; it holds {len(g['head_attach'])} cage nodes (its own node + the struts' nodes){'' if HEAD_ON_CAGE else ' and takes the keels and braces'}; {HEAD_SYS_G:.0f} g in the system "
        f"model (E: seat and lugs, camera plates, guards, joining material {g['head_topo']:.0f} g held to the rods it replaces); requirements in head_system.json")
# ---------------------------------------------------------------- access (rule 2: reachable without taking another part off)
# straight tool lines from each target out through a shell cell, searched (clear of every part by the tool radius + 1),
# then gated by system_check's own sweep
blockers = [(n_, m_) for n_, m_, *_ in PARTS if not n_.startswith(("cage shell", "spokes"))]
segs_all = [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), lr(k)) for k, (p, q) in extra.items()]
pq = {n_: trimesh.proximity.ProximityQuery(m_) for n_, m_ in blockers}


def line_clear(p0, d, L_, rad, ignore=()):
    q = p0 + np.linspace(6, L_, int(L_ / 4))[:, None] * d
    for a, b, rr in segs_all:
        u = b - a; t = np.clip(((q - a) @ u) / (u @ u), 0, 1)
        if (np.linalg.norm(q - (a + t[:, None] * u), axis=1) < rad + rr + 1).any(): return False
    for n in nodes:
        if (np.linalg.norm(q - n, axis=1) < rad + NODE_R + 1).any(): return False
    for n_, m_ in blockers:
        if n_ in ignore: continue
        bb_ = m_.bounds
        near = ((q > bb_[0] - rad - 2) & (q < bb_[1] + rad + 2)).all(1)
        if near.any():
            sd = pq[n_].signed_distance(q[near])
            if (sd > -(rad + 1)).any(): return False
    return True


def search(p0, axis, cone_deg, rad, L_=450.0, ignore=()):
    axis = np.asarray(axis, float) / np.linalg.norm(axis); best = None
    for d in fib(400):
        ang = np.degrees(np.arccos(np.clip(d @ axis, -1, 1)))
        if ang <= cone_deg and (best is None or ang < best[0]) and line_clear(p0, d, L_, rad, ignore): best = (ang, d)
    return best


ACCESS, acc_rep = [], []
ACCESS.append({"name": "pack swap: cartridge + pack down through the opening", "part": "cartridge", "dir": [0, 0, -1], "length_mm": 150.0,
               "tool": "part", "ignore": ["pack", "receiver"]})
hubn_ = "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)"
for nm_, faces in (("FC USB-C", [((18.0, 0, 9.75), (1, 0, 0)), ((0, 18.0, 9.75), (0, 1, 0)), ((0, -18.0, 9.75), (0, -1, 0)), ((-18.0, 0, 9.75), (-1, 0, 0))]),
                   ("Pi 5 USB-C", [((42.5, 0, 30.0), (1, 0, 0)), ((-42.5, 0, 30.0), (-1, 0, 0)), ((0, 28.0, 30.0), (0, 1, 0)), ((0, -28.0, 30.0), (0, -1, 0))])):
    got = None
    for p0, ax in faces:
        b_ = search(np.array(p0, float), ax, 50.0, 7.0, ignore=(hubn_, "wires"))
        if b_ and (got is None or b_[0] < got[0]): got = (b_[0], b_[1], p0)
    if got:
        ACCESS.append({"name": f"{nm_} cable", "part": hubn_, "from": list(got[2]), "dir": np.round(got[1], 4).tolist(), "length_mm": 450.0,
                       "tool": {"type": "box", "w": 12, "h": 7}, "ignore": ["wires"]})
        acc_rep.append(f"{nm_}: clear line {got[0]:.0f} deg off its face normal from {tuple(np.round(got[2], 1))}")
    else:
        acc_rep.append(f"{nm_}: NO clear line within 50 deg of any face")
for i_, d_ in enumerate((45, 135, 225, 315)):                           # arm-top screws (M3 into the root, driver d 8 from above)
    c_, s_ = math.cos(math.radians(d_)), math.sin(math.radians(d_))
    for v_ in (15.0, -15.0):
        p0 = np.array([63.17 * c_ - v_ * s_, 63.17 * s_ + v_ * c_, ROOT_H + 1.6])
        ign_p = ("top plate", f"arm {i_ + 1}") + tuple(f"prop {j}" for j in range(1, 5))
        ok_ = line_clear(p0, np.array([0, 0, 1.0]), 400.0, 4.0, ignore=ign_p)
        if ok_:
            ACCESS.append({"name": f"M3 key, arm {i_ + 1} top screw v {v_:+.0f}", "part": "top plate", "from": np.round(p0, 2).tolist(), "dir": [0, 0, 1],
                           "length_mm": 400.0, "tool": {"type": "cylinder", "d": 8}, "ignore": [f"arm {i_ + 1}"] + [f"prop {j}" for j in range(1, 5)]})
            acc_rep.append(f"arm {i_ + 1} top screw v {v_:+.0f}: clear straight up (a stopped prop turned aside)")
            continue
        # an M3 hex L-key (2.5): short leg 30 straight up (d 8 with the hand clear), long leg out through a shell cell (d 4)
        short_ok = line_clear(p0, np.array([0, 0, 1.0]), 30.0, 4.0, ignore=ign_p)
        top_ = p0 + [0, 0, 30.0]; out_ = np.array([c_, s_, 0.0])
        lg = search(top_, out_, 70.0, 2.0, L_=400.0, ignore=ign_p) if short_ok else None
        if lg:
            ACCESS.append({"name": f"M3 L-key short leg, arm {i_ + 1} top screw v {v_:+.0f}", "part": "top plate", "from": np.round(p0, 2).tolist(),
                           "dir": [0, 0, 1], "length_mm": 30.0, "tool": {"type": "cylinder", "d": 8}, "ignore": [f"arm {i_ + 1}"] + [f"prop {j}" for j in range(1, 5)]})
            ACCESS.append({"name": f"M3 L-key long leg, arm {i_ + 1} top screw v {v_:+.0f}", "part": "top plate", "from": np.round(top_, 2).tolist(),
                           "dir": np.round(lg[1], 4).tolist(), "length_mm": 400.0, "tool": {"type": "cylinder", "d": 4},
                           "ignore": [f"arm {i_ + 1}"] + [f"prop {j}" for j in range(1, 5)]})
        acc_rep.append(f"arm {i_ + 1} top screw v {v_:+.0f}: " + (f"L-key (short leg up 30, long leg {lg[0]:.0f} deg off radial out through a cell)" if lg else "BLOCKED"))
    for v_ in (10.0, -10.0):                                            # bottom screws (driver d 8 from below)
        p0 = np.array([63.74 * c_ - v_ * s_, 63.74 * s_ + v_ * c_, -1.7])
        ok_ = line_clear(p0, np.array([0, 0, -1.0]), 400.0, 4.0, ignore=("bottom plate", f"arm {i_ + 1}"))
        ign = [f"arm {i_ + 1}"]
        if not ok_:
            for extra_ in (["belly sled + pods"], ["belly sled + pods", "flow MTF-02P"]):
                if line_clear(p0, np.array([0, 0, -1.0]), 400.0, 4.0, ignore=("bottom plate", f"arm {i_ + 1}", *extra_)): ign += extra_; break
        ACCESS.append({"name": f"M3 key, arm {i_ + 1} bottom screw v {v_:+.0f}", "part": "bottom plate", "from": np.round(p0, 2).tolist(), "dir": [0, 0, -1],
                       "length_mm": 400.0, "tool": {"type": "cylinder", "d": 8}, "ignore": ign})
        acc_rep.append(f"arm {i_ + 1} bottom screw v {v_:+.0f}: " + ("clear" if ok_ else "BLOCKED" if len(ign) == 1 else
                       "through a Ø9 driver hole in the belly sled (requirement)" if len(ign) == 2 else
                       f"through the sled with the {ign[-1]} board unplugged from its pod (v2 camera format, Spark 27 Oct; requirement)"))
say("access (straight lines through the shell cells, tool radius + 1 clear, C): " + "; ".join(acc_rep))

SHELL_SPEC = dict(nodes=np.round(nodes, 3).tolist(), rods=[list(map(int, e)) for e in E],
                  open=[dict(name="bottom opening (pack + swap)", ring=[int(i) for i in np.where(nodes[:, 2] < g["z_o"] + 0.5)[0]])],
                  node_r_mm=NODE_R, rod_r_mm=SHELL_R)
json.dump(SHELL_SPEC, open(os.path.join(OUT, "cage_shell.json"), "w"))
GRP_OF = {"cage shell tubes": "shell", "cage shell nodes": "nodes", LINKN: "links", "L2 guard ring nodes": "halo nodes",
          "TPU feet": "feet", "apex + cam bracket": "L2 mount", HEADN: "head", "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)": "hub", "belly sled + pods": "belly"}
if ("V5_ASM" in os.environ or not any(k in os.environ for k in ("V5_FREQ", "V5_START", "V5_STARTS", "V5_PACK", "V5_ROTS", "V5_SNAPS", "V5_PEAKS", "V5_DOME", "V5_DOMES", "V5_SLACK")) and os.path.abspath(OUT) == os.path.join(ROOT, "runs", "overseer_layout", "v5")):
    asm_parts = []
    for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS:
        p_ = dict(name=n_, role=r_, mesh=rel(os.path.join(PD, fname(n_))), mass_g=round(float(gm_), 2), tag=t_, owner="overseer",
                  version="layout v5 / cage v5", source="nox/overseer/layout_v5.py")
        gk = GRP_OF.get(n_, n_ if n_ in grp else None)                 # CG from the layout's mass model (multi-piece meshes are not
        if gk and grp.get(gk, 0) > 0: p_["cg"] = np.round(grp_mc[gk] / grp[gk], 2).tolist()   # watertight: system_check would take the box centre)
        if pr_: p_["protect"] = True
        if mat_: p_["material"] = mat_
        if sw_: p_["swap"] = True
        asm_parts.append(p_)
    hubn = "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)"
    contact = [["wires", hubn], ["pack", "cartridge"], ["pack", "receiver"], ["cartridge", "receiver"], ["belly sled + pods", "receiver"], ["cage shell tubes", "cage shell nodes"], [LINKN, "cage shell nodes"], [LINKN, "L2 guard ring nodes"],
               [LINKN, "cage shell tubes"], [LINKN, "bottom plate"], [LINKN, "apex + cam bracket"],
               [LINKN, "belly sled + pods"], ["Unitree L2", "apex + cam bracket"], ["camera left", "apex + cam bracket"], ["camera right", "apex + cam bracket"],
               ["apex + cam bracket", "bottom plate"], ["apex + cam bracket", "top plate"], ["belly sled + pods", "cartridge"],
               ["belly sled + pods", "flow MTF-02P"], ["belly sled + pods", "TPU feet"], ["belly sled + pods", "bottom plate"],
               [hubn, "bottom plate"], [hubn, "top plate"]]
    for i in range(1, 5):
        contact += [[f"arm {i}", "top plate"], [f"arm {i}", "bottom plate"], [f"motor {i}", f"arm {i}"], [LINKN, f"arm {i}"], [f"arm {i}", hubn]]
    if PRINTED_HEAD:                                                   # the two head parts are one printed part now
        contact = [[HEADN if q_ in ("apex + cam bracket", "L2 guard ring nodes") else q_ for q_ in c_] for c_ in contact]
        contact = [c_ for c_ in contact if c_[0] != c_[1]] + [[HEADN, "cage shell nodes"], [HEADN, "cage shell tubes"]]
    asm = new_assembly("Overseer", "overseer", asm_parts,
                       note=("Layout v5 / cage v5 (26-27 Oct, Spark: the v2 drone with a triangulated shell formed to it; the L2 and the two cameras (27 Oct: no down camera, the pair beside the L2) outside "
                             "it with nothing in their views, a guard ring behind the L2; funnel redesigned later) from nox/overseer/layout_v5.py; supersedes "
                             "layout v4 / cage v2. Rough layout: C geometry on E sizes. "
                             "Drone frame: x fwd, y left, z up, z 0 bottom plate top. Left out: ESC bodies (25 g each in the arm masses), catch features "
                             "(the new funnel's), scan stick mount, XT60 and leads; the latch set is drawn as v2's cartridge envelope."),
                       cg_ref=[round(dm, 3), 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0,
                       openings=[dict(part=p_, axis=[0, 0, -1], half_angle_deg=40.0, skip_probes=True)
                                 for p_ in ("pack", "cartridge", "receiver", "flow MTF-02P")]
                                + [dict(part="Unitree L2", axis=[round(float(v), 4) for v in A], half_angle_deg=L2_CONE, field=True,
                                        note="Spark 26 Oct: outside the cage, nothing in its view; guarded only outside its field (cage first gated outside this cone)"),
                                   *[dict(part=f"camera {'left' if c['side'] > 0 else 'right'}", axis=[round(float(v), 4) for v in c["f"]], half_angle_deg=CAM_CONE, field=True,
                                          note="Spark 27 Oct: the pair beside the L2, nothing of the drone in its view (diagonal 51); cage first gated outside this cone")
                                     for c in g["cams"]]],
                       contact_pairs=contact, guard_offsets={"shell tubes to prop disks, 3D (G)": round(pg, 1)}, mass_budget_g=2200, cage_shell=SHELL_SPEC, access=ACCESS,
                       balance_no_swap_gate=False)   # dimensions.md L Balance: Overseer pack-out reported (DEFAULT 26 Oct, Spark's v2 format)
    asm.save(os.environ.get("V5_ASM", os.path.join(ROOT, "nox", "overseer", "overseer.assembly.json")))
    say("wrote nox/overseer/overseer.assembly.json")
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
