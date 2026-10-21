"""S2 cage v3 layout search (17 Oct 03:10, Spark 03:01: nodes up to the electronics above the props, layout from a search).

The cage meets the electronics through a printed CROWN clamped on the 4 Radxa boss tops of the S2 top case (Spark 03:05:
"crown on the top case"). Spokes from the crown come down to the rings only along the 4 axis gaps between the props
(nose +x, left +y, tail -x, right -y). The search picks, per axis, the spoke pattern and the ring it lands on, whether
the plate struts (side hubs, tail hub) stay, and the tee material, from the frame model (F):

  nose / tail / side: none | single | V (two spokes); start points from spoke_route.py (12 off the props, lanes clear)
  landing ring      : upper (z 49.2) only (the lower ring is out of reach past the props, spoke_route.py)
  upper ring        : 2.0 | 2.5 solid
  plate struts      : side pair from the side hubs yes / no; tail strut from the tail hub yes / no
  tee nodes         : TPU for AMS (PETG checked on the short list)

Fixed: rings (clipped square F 112 / C 100, lower 2.5 at prop plane - 8, upper 2.0 at + 8), PETG corner nodes with the
wheels, the 4 feet on 3.0 legs from the side hubs (floor landing without the funnel, Spark 16 Oct 20:54).
Top case stand-in (E): 6 PETG columns d 6 from the boss tops to the plate pads (side bolts; front bosses also to the
nose bolt), clamped at the pads. Its forces are reported as the CASE LOAD for the top case chat, never in the cage index.

Index = carbon rods and their socket bearing only (17 Oct 03:45): the printed parts here are stand-ins; each is generated
for the socket loads this model hands it (node_gen.py). Feasible = every member 12 off the swept props (socket bodies r 2.9 near the ends), clear of the Radxa microSD lane and
the 3 hex-key lanes, 0.5 m/s touches index <= 1 (frame model design rule), statics (6 g landing, 15 N swap push, 6 g
upside-down landing on the crown) index <= 1, swap-push wheel deflection <= 3.0 (DEFAULT). Report: mass, v_intact
(index 1 speed of the worst side / foot hit), ceiling hit, case load.
Run: python3 layout_v3.py search  -> layout_v3.json ; python3 layout_v3.py show <i>
"""
import sys, os, json, itertools, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
import s2_guard_frame as sg
from s2_guard_frame import sec, Guard2, _rot, MIN_KG, G0, energy, ROD_ALLOW, TPU_ALLOW, PETG_XY, PETG_Z, EC_RHO
import strut_route as sr

PZ = 41.2
BX, BY = 11.45, 28.9                 # Radxa boss centres (top case geom_top.py R_HOLES, C)
BOSS_TOP = 68.11                     # printed top case v1 boss tops (C, 17 Oct; cage v4). Was 68.81 (E, case riding the prop plane)
CASE_DZ = BOSS_TOP - 68.11            # 0: the top case geom_top numbers apply as drawn
CR = 2.2                             # crown beam radius (E, DEFAULT)
ZC = BOSS_TOP + CR                   # crown beam axis (frame model)
ZS = 76.0                            # spoke start height (spoke_route.py, 17 Oct 04:20)
PADS = {"L": (3.9, 31.0, 25.0), "R": (3.9, -31.0, 25.0), "N": (45.0, 0.0, 25.0)}
# keep-outs above the plate the spokes must miss (top case geom_top.py, +0.7 for the prop plane, C / E)
SD_LANE = ((-8.3, 32.5, 63.26 + CASE_DZ), (8.2, 47.0, 67.26 + CASE_DZ))   # microSD in / out, left end
KEY_LANES = [(3.9, 31.0), (3.9, -31.0), (45.0, 0.0)]                  # hex key from above, z 25 .. 75
KEY_R = [2.0, 2.0, 3.5]               # lane radius per lane: sides = hex key r 2.0; nose = the bolt head comes out upward
                                      # (button head ~5.7 + 0.65, DEFAULT, cage v4: top case chat flag 17 Oct 22:07)
OPT_C = json.load(open(os.path.join(HERE, "layout_C3.json")))
ROD_D = float(OPT_C.get("lower", 2.5))  # cage rod diameter (v4: 3.0 solid everywhere, Spark 17 Oct 21:33 / 21:45)
R_ROD = ROD_D / 2
R_SOCK = R_ROD + 0.15 + 2.0           # socket sleeve outer radius (bore rod + 0.3, PETG wall 2.0): 3.65 at 3.0 (was 2.9 E)
MASS_E = dict(corner=3.62, wheel=0.171, side_hub=1.66, tail_hub=0.50, tee=1.23, foot=0.49,
              crown_base=2.7, crown_socket=0.3)                         # g; M from the 02:58 set, crown E

# receiver v3 (receiver_v3.py, 17 Oct 02:23): pack nose-tail, bottom z -44.5 (E); outer sides y +-24.3 (RF 20.3 + 4, C);
# body x -46 .. 42 (pack x -41.5 .. 37.5 + walls, E), up to the plate underside 18.5. Skirt sockets on its sides at y +-26.
RCV_Y, RCV_PLATE, FOOT_Z, PACK_BOT = 26.5, 18.5, -47.5, -44.5   # 17 Oct 09:40: root y 26.5, socket 10 deep: 0.48 inside the wheel slot (C)
SKIRT_DEPTH = 10.0
RCV_BOX = ((-46.0, -26.0, -46.0), (42.0, 26.0, 18.5))
PACK_PATH = ((-45.0, -23.4, -200.0), (41.0, 23.4, 3.5))                 # cartridge (x 20 + capture 3.3) + lead-ins, up to its top z 3.5 (C)
FLOW_APEX, FLOW_HALF = np.array([63.0, -2.0, -5.0]), np.radians(25.0)   # receiver flow sensor view cone (C)
D_FACE = 112.0 * np.sin(np.radians(45)) - (PZ - 8.0) * np.cos(np.radians(45)) + R_ROD + 2.5   # collar face n.p, seat (wheel rim 2.5 proud of the lower rod)
COLLAR_EDGE_Z = -0.1 - (R_ROD - 1.25) / np.sin(np.radians(45))          # collar bottom edge in drone z (seat; 3.0 rods sit the drone 0.35 higher)
def skirt_target(t, sx, sy, ZL):
    """t = ("s", a): side flat at x = sx * a; ("e", a): nose / tail flat at y = sy * a (lower ring, a TPU tee sleeve there)"""
    k, a = t
    return np.array((sx * a, sy * 112.0, ZL) if k == "s" else (sx * 112.0, sy * a, ZL), float)
STATIONS = (20.0, 40.0, 60.0, 80.0)
def collar_margin(p):
    """smallest gap to the 4 collar face planes over yaw +-5, tilt +-2 (CG z 4.2)"""
    worst = 1e9; S45 = np.sin(np.radians(45))
    for yaw, tx, ty in itertools.product((-5, 0, 5), (-2, 0, 2), (-2, 0, 2)):
        a, bx, by = map(np.radians, (yaw, tx, ty))
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, np.cos(bx), -np.sin(bx)], [0, np.sin(bx), np.cos(bx)]])
        Ry = np.array([[np.cos(by), 0, np.sin(by)], [0, 1, 0], [-np.sin(by), 0, np.cos(by)]])
        q = Rz @ Rx @ Ry @ (p - [0, 0, 4.2]) + [0, 0, 4.2]
        for k in range(4):
            c, s_ = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)
            worst = min(worst, D_FACE - np.array([c * S45, s_ * S45, -S45]) @ q)
    return worst
def skirt_ok(R0, T, r=None, r_end=None):
    """worst margin of one skirt rod: props 12, collar 1.0 above the collar edge / 3.0 below (last 15 at the ring end
    belong to the node), pack path, receiver body (first 4 belong to the socket), flow cone"""
    r = R_ROD if r is None else r; r_end = R_SOCK if r_end is None else r_end
    L_ = np.linalg.norm(T - R0); worst = sr.clearance(R0, T, r_end=r_end, r_rod=r) - 12.0; why = "props"
    for t in np.linspace(0, 1, 120):
        p = R0 + (T - R0) * t; d = t * L_; rr = r_end if (d < 15 or L_ - d < 15) else r
        if L_ - d > 15:
            m = collar_margin(p) - rr - (1.0 if p[2] > COLLAR_EDGE_Z else 3.0)
            if m < worst: worst, why = m, "collar"
        m = box_dist(p, *PACK_PATH) - rr
        if m < worst: worst, why = m, "pack path"
        if d > 4.0:
            m = box_dist(p, *RCV_BOX) - rr
            if m < worst: worst, why = m, "receiver"
        if p[2] < FLOW_APEX[2]:
            m = np.hypot(*(p[:2] - FLOW_APEX[:2])) - (FLOW_APEX[2] - p[2]) * np.tan(FLOW_HALF) - rr - 1.0
            if m < worst: worst, why = m, "flow cone"
    return worst, why
def box_dist(p, lo, hi):
    q = np.maximum(np.asarray(lo) - p, p - np.asarray(hi)); return float(np.linalg.norm(np.maximum(q, 0)) + min(q.max(), 0))
def lane_clear(a, b, r_rod=None, r_end=None, end_len=15.0):
    r_rod = R_ROD if r_rod is None else r_rod; r_end = R_SOCK if r_end is None else r_end
    a, b = np.asarray(a, float), np.asarray(b, float); L = np.linalg.norm(b - a); worst = 1e9
    for t in np.linspace(0, 1, 200):
        p = a + (b - a) * t; r = r_end if min(t * L, (1 - t) * L) < end_len else r_rod
        d = box_dist(p, *SD_LANE) - r
        for (kx, ky), kr in zip(KEY_LANES, KEY_R):
            if p[2] >= 25.0: d = min(d, np.hypot(p[0] - kx, p[1] - ky) - kr - r)      # the key comes down from above: lane open to the sky
        worst = min(worst, d)
    return worst

SPOKES = json.load(open(os.path.join(HERE, "spoke_route.json")))
def spoke_starts(axis, pattern):
    """crown points for one axis (k 0 nose, 1 +y, 2 tail, 3 -y), from spoke_route.py (side starts are the +y ones)"""
    if pattern == "none": return []
    name = {0: "nose", 1: "side", 2: "tail", 3: "side"}[axis]
    pts = [np.array(p, float) for p in SPOKES[f"{name}_{pattern}"]["starts"]]
    if axis == 3: pts = [p * [1, -1, 1] for p in pts]
    return pts
def ring_point(axis, ring, F=112.0):
    z = PZ + (8.0 if ring == "U" else -8.0)
    return np.array({0: (F, 0, z), 1: (0, F, z), 2: (-F, 0, z), 3: (0, -F, z)}[axis], float)

def spokes_of(opt):
    out = []
    for axis, key in ((0, "nose"), (2, "tail"), (1, "side"), (3, "side")):
        pat, ring = opt[key]
        for a in spoke_starts(axis, pat): out.append((axis, np.array(a, float), ring_point(axis, ring), ring))
    return out

def geom_ok(opt):
    worst_p, worst_l = 1e9, 1e9
    for axis, a, b, ring in spokes_of(opt):
        worst_p = min(worst_p, sr.clearance(a, b, r_end=R_SOCK, r_rod=R_ROD)); worst_l = min(worst_l, lane_clear(a, b))
    return worst_p, worst_l

class Guard3(Guard2):
    def node(s, p):
        p = np.asarray(p, float); key = tuple(np.round(p, 5))
        if not hasattr(s, "_nk"): s._nk = {}
        if key in s._nk: return s._nk[key]
        s.nodes.append(p); s._nk[key] = len(s.nodes) - 1; return len(s.nodes) - 1
    def build(s):
        P = s.P; F, C, ZL, ZU = P["F"], P["C"], P["ZL"], P["ZU"]; e, m = P["rod_end"], P["mouth"]
        S = lambda d: sec(("solid", d)); s.wheel_pts = []; o = P["opt"]
        for k in range(4):                                   # rings, corner nodes (as Guard2, tee points on every flat)
            R = _rot(k); ss = sorted(set([-C, -e, -m, -4.5, 0., 4.5, m, e, C] + [v for a in STATIONS for v in (a, -a)]))
            for z, sc, tag in ((ZL, sec(P["lower"]), "lower"), (ZU, sec(P["upper"]), "upper")):
                pts = [R @ [F, v, z] for v in ss]
                for a, b, va, vb in zip(pts[:-1], pts[1:], ss[:-1], ss[1:]):
                    if abs(va) >= e - 1e-9 and abs(vb) >= e - 1e-9: s.line(a, b, S(5.2 if tag == "lower" else 4.6), "P", "node", n=1)
                    else: s.line(a, b, sc, "C", tag)
            for v in (-e, e): s.line(R @ [F, v, ZL], R @ [F, v, ZU], S(3.6), "P", "brace", n=3)
            for v in (-m, m): s.line(R @ [F, v, ZL], R @ [F, v, ZU], S(P["mouth_d"]), "P", "mouth", n=3)
            for v in (-C, C): s.wheel_pts.append((R @ [F, v, ZL], R @ np.array([np.sin(np.pi / 4), 0, -np.cos(np.pi / 4)])))
        for k in range(4):
            R = _rot(k); K = np.array([106., 106.])
            for z in (ZL, ZU):
                s.line(R @ [F, C, z], R @ [*K, z], S(4.4), "P", "node", n=2); s.line(R @ [*K, z], R @ [C, F, z], S(4.4), "P", "node", n=2)
            s.line(R @ [*K, ZL], R @ [*K, ZU], S(4.8), "P", "post", n=3)
        sp = spokes_of(o); s.crown = []; tm = o["tee_mat"]; s.case_fixed = []
        if sp:                                               # crown + top case stand-in
            for bx in (BX, -BX):
                s.line((bx, -BY, ZC), (bx, 0, ZC), S(2 * CR), "P", "crown", n=3); s.line((bx, 0, ZC), (bx, BY, ZC), S(2 * CR), "P", "crown", n=3)
            s.line((-BX, 0, ZC), (BX, 0, ZC), S(2 * CR), "P", "crown", n=3)
            for bx in (BX, -BX):
                for by in (BY, -BY):
                    pad = PADS["L"] if by > 0 else PADS["R"]
                    ids = s.line((bx, by, ZC), pad, S(6.0), "P", "case", n=4); s.case_fixed.append(ids[-1]); s.fixed.append(ids[-1])
                    if bx > 0:
                        ids = s.line((bx, by, ZC), PADS["N"], S(6.0), "P", "case", n=4); s.case_fixed.append(ids[-1]); s.fixed.append(ids[-1])
            s.crown = [s.node((bx, by, ZC)) for bx in (BX, -BX) for by in (BY, -BY)]
        cn = [np.array(p, float) for p in [(bx, by, ZC) for bx in (BX, -BX) for by in (BY, 0.0, -BY)]]
        for axis, a, Rp, ring in sp:                         # crown arm from each spoke start to its 2 nearest crown points
            near = sorted(cn, key=lambda q: np.linalg.norm(q - a))[:2]
            for q in near:
                if np.linalg.norm(q - a) > 1e-6: s.line(a, q, S(2 * CR), "P", "crown", n=2)
        for axis, a, Rp, ring in sp:
            u = (Rp - a) / np.linalg.norm(Rp - a)
            s.line(a, Rp - 8.5 * u, sec(P["spoke"]), "C", "spoke"); s.line(Rp - 8.5 * u, Rp, S(5.8), tm, "tee", n=1)
        ro = P["route"]; s.hub_fixed = []
        roots = []
        if o["side_strut"]:
            for sy in (1, -1): roots.append((np.array(ro["side"]["bracket_socket_bottom"], float) * [1, sy, 1], 1 if sy > 0 else 3))
        if o["tail_strut"]: roots.append((np.array(ro["tail"]["bracket_socket_bottom"], float), 2))
        for B, axis in roots:
            Rp = ring_point(axis, "L"); u = (Rp - B) / np.linalg.norm(Rp - B)
            ids = s.line(B, Rp - 8.5 * u, sec(P["strut"]), "C", "strut"); s.fixed.append(ids[0]); s.hub_fixed.append(ids[0])
            s.line(Rp - 8.5 * u, Rp, S(5.8), tm, "tee", n=1)
        s.feet = []
        if o.get("bottom"): s.skirt(o["bottom"]); s.fixed = sorted(set(s.fixed)); s.N = len(s.nodes); s.X = np.array(s.nodes); s.assemble(); s.prep(); return
        xf, yf, zc = P["foot"]["foot_centre"]
        for sx in (1, -1):
            for sy in (1, -1):
                B = np.array(ro["side"]["bracket_socket_bottom"], float) * [1, sy, 1]
                ids = s.line(B, (sx * xf, sy * yf, zc), sec(P["leg"]), "C", "leg", n=7)
                s.fixed.append(ids[0]); s.hub_fixed.append(ids[0]); s.feet.append(ids[-1])
        s.fixed = sorted(set(s.fixed)); s.N = len(s.nodes); s.X = np.array(s.nodes); s.assemble(); s.prep()

    def skirt(s, bt):
        """17 Oct 03:55 (Spark): the bottom rods root on the drone-side receiver (receiver v3, on the 3 plate bolts) and run up
        to the guard's lower ring. Receiver stand-in (E): PETG columns d 8 from each root and each foot pad to the two
        nearest plate bolts (under the plate, z 18.5), clamped there; its forces are the RECEIVER LOAD, never in the index.
        Feet = pads on the receiver under the roots, bottoms 3 under the pack (z -47.5)."""
        S = lambda d: sec(("solid", d)); P = s.P; ZL = P["ZL"]
        bolts = [np.array(b, float) for b in ((3.9, 31.0, RCV_PLATE), (3.9, -31.0, RCV_PLATE), (45.0, 0.0, RCV_PLATE))]
        s.rcv_fixed = []
        def to_plate(p):
            for b in sorted(bolts, key=lambda q: np.linalg.norm(q - p))[:2]:
                ids = s.line(p, b, S(8.0), "P", "rcv", n=4); s.fixed.append(ids[-1]); s.rcv_fixed.append(ids[-1])
        for grp, sx in (("front", 1), ("back", -1)):
            gp = bt[grp] if grp in bt else dict(bt["quad"], x=sx * bt["quad"]["x"])     # "quad": one quadrant, mirrored 4x
            for sy in (1, -1):
                R0 = np.array([gp["x"], sy * RCV_Y, gp.get("z", bt.get("z"))]); to_plate(R0)
                if bt.get("feet", True):
                    foot = np.array([gp["x"], sy * RCV_Y, FOOT_Z]); s.line(R0, foot, S(8.0), "P", "rcv", n=2); s.feet.append(s.node(foot))
                for t in gp["targets"]:
                    T = skirt_target(tuple(t), sx, sy, ZL); u = (T - R0) / np.linalg.norm(T - R0)
                    s.line(R0, T - 8.5 * u, sec(P["skirt"]), "C", "skirt")
                    s.line(T - 8.5 * u, T, S(5.8), P["opt"]["tee_mat"], "tee", n=1)
    def prep(s):
        ne = len(s.elems); s.KT = np.zeros((ne, 12, 12)); s.D = np.zeros((ne, 12), int)
        s.A = np.zeros(ne); s.I = np.zeros(ne); s.c = np.zeros(ne); s.d = np.zeros(ne); s.allow = np.zeros(ne); s.isC = np.zeros(ne, bool)
        host = {}
        for e in s.elems:
            if e["mat"] in "TP" and e["tag"] not in ("case", "rcv"):
                for i in e["n"]: host[i] = "T" if (e["mat"] == "T" or host.get(i) == "T") else "P"
        s.bear = np.zeros((ne, 2))
        for i, e in enumerate(s.elems):
            s.KT[i] = e["kl"] @ e["T"]; a, b = e["n"]; s.D[i] = np.r_[6 * a:6 * a + 6, 6 * b:6 * b + 6]
            s.A[i], s.I[i], s.c[i], s.d[i] = e["sec"]["A"], e["sec"]["I"], e["sec"]["c"], e["sec"]["d"]
            if e["tag"] in ("case", "rcv"): s.allow[i] = np.inf
            elif e["mat"] == "C":
                s.allow[i] = ROD_ALLOW; s.isC[i] = True
                for k, ni in enumerate(e["n"]):
                    if ni in host: s.bear[i, k] = 1.0 / (6.0 / (s.d[i] * s.P["sock_L"] ** 2)) * (22.4 if host[ni] == "T" else PETG_XY)
            elif e["mat"] == "T": s.allow[i] = TPU_ALLOW
            else:
                az = abs((s.X[b] - s.X[a])[2]) / e["L"]; s.allow[i] = PETG_XY * (1 - az) + PETG_Z * az
        s.Kc = None

    def index_batch(s, U):
        fl = np.einsum("eij,ejc->eic", s.KT, U[s.D])
        M0 = np.hypot(fl[:, 4], fl[:, 5]); M1 = np.hypot(fl[:, 10], fl[:, 11])
        sgm = np.maximum(np.abs(fl[:, 0]) / s.A[:, None] + M0 * (s.c / s.I)[:, None], np.abs(fl[:, 6]) / s.A[:, None] + M1 * (s.c / s.I)[:, None])
        idx = sgm / s.allow[:, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            b0 = np.where(s.bear[:, :1] > 0, M0 / s.bear[:, :1], 0); b1 = np.where(s.bear[:, 1:] > 0, M1 / s.bear[:, 1:], 0)
        idx = np.maximum(idx, np.maximum(b0, b1))
        if getattr(s, "carbon_only", False):                 # rods + their sockets; printed parts are generated for their loads
            idx = np.where(s.isC[:, None], idx, 0.0)
        return idx.max(0), idx.argmax(0)

    def indent_batch(s, cases, E):
        """rigid flat indenter cases [(kind, where, ids, dirs)] at energy E -> U (ndof, nc), force (nc)"""
        U = np.zeros((6 * s.N, len(cases))); Fo = np.zeros(len(cases))
        if not hasattr(s, "fmap"):
            s.fmap = -np.ones(6 * s.N, int); s.fmap[np.where(s.free)[0]] = np.arange(s.free.sum())
        for j, (kind, where, ids, dirs) in enumerate(cases):
            cols = []
            for i, d in zip(ids, dirs):
                d = -np.asarray(d, float); col = np.zeros(6 * s.N)
                fi = s.fmap[6 * i:6 * i + 3]
                if (fi < 0).any(): cols.append(col); continue
                col[s.free] = s.Kinv[:, fi] @ d; cols.append(col)
            Cm = np.array([[cols[q][6 * ids[p]:6 * ids[p] + 3] @ -np.asarray(dirs[p]) for q in range(len(ids))] for p in range(len(ids))])
            act = list(range(len(ids)))
            while True:
                f = np.linalg.solve(Cm[np.ix_(act, act)], np.ones(len(act)))
                if (f >= -1e-9).all(): break
                act = [a for a, v in zip(act, f) if v > 1e-9]
            k = f.sum(); delta = np.sqrt(2 * E / k); fa = f * delta
            U[:, j] = sum(fa[q] * cols[a] for q, a in enumerate(act)); Fo[j] = k * delta
        return U, Fo

    def clearance_batch(s, U):
        R = sg.node_radius(s); X = s.X[None] + U.reshape(s.N, 6, -1)[:, :3].transpose(2, 0, 1)
        best = np.full(X.shape[0], 1e9)
        dz = np.maximum(0, np.abs(X[..., 2] - PZ) - 4.0)
        for mx, my in sg.MOTORS:
            d = np.hypot(X[..., 0] - mx, X[..., 1] - my) - 44.45
            c = np.where(d > 0, np.hypot(np.maximum(d, 0), dz), dz) - R[None]
            best = np.minimum(best, c.min(1))
        return best

def evaluate(opt, spoke=None):
    spoke = spoke or ("solid", opt.get("spoke", 2.5))
    P = sg.params_v2(PZ_=PZ, lower=("solid", opt.get("lower", 2.5)), upper=("solid", opt.get("upper", 2.0))); P.update(opt=opt, spoke=spoke, skirt=("solid", opt.get("skirt_d", 3.0)))
    g = Guard3(P); W = MIN_KG * G0; g.carbon_only = True
    hits = sg.hit_cases_v2(g)
    U1, F1 = g.indent_batch(hits, energy(1.0)); idx1, arg1 = g.index_batch(U1)
    clr1 = g.clearance_batch(U1)
    j = int(idx1.argmax()); worst_hit = (float(idx1[j]), hits[j][0], g.elems[arg1[j]]["tag"])
    out = dict(opt=opt, v_intact=float(1.0 / idx1.max()), idx1=float(idx1.max()), worst_hit=str(worst_hit),
               touch=float(idx1.max() * 0.5), clr1=float(clr1.min()), peak_g=float(F1.max() / W))
    # statics
    def apply(loads):
        Fv = np.zeros(6 * g.N)
        for i, f in loads: Fv[6 * i:6 * i + 3] += f
        return g.solve(Fv)
    st = {}
    for name, sel in (("land_4", g.feet), ("land_2side", [i for i in g.feet if g.X[i, 1] > 0]), ("land_2front", [i for i in g.feet if g.X[i, 0] > 0])):
        st[name] = apply([(i, np.array([0, 0, 6 * W / len(sel)])) for i in sel])
    wp = [(p, n) for p, n in g.wheel_pts if n[0] > 0.5]
    st["swap_15N"] = apply([(g.node_id(p), -n * 7.5) for p, n in wp])
    if g.crown: st["flip_6g_crown"] = apply([(i, np.array([0, 0, -6 * W / 4])) for i in g.crown])
    Us = np.stack(list(st.values()), 1); idxs, args = g.index_batch(Us)
    out["static"] = {k: round(float(v), 3) for k, v in zip(st, idxs)}
    out["swap_defl"] = float(max(abs(st["swap_15N"][6 * g.node_id(p):6 * g.node_id(p) + 3] @ n) for p, n in wp))
    # ceiling hit at 1 m/s on the top-most nodes, and the case load
    if g.crown:
        top = [i for i in range(g.N) if g.X[i, 2] > g.X[:, 2].max() - 0.05]
        Uc, Fc = g.indent_batch([("ceiling", "top", top, [np.array([0, 0, 1.0])] * len(top))], energy(1.0))
        ic, _ = g.index_batch(Uc); out["ceiling_idx"] = float(ic[0]); out["ceiling_N"] = float(Fc[0])
        def case_share(u, Fapplied):
            r = g.K @ u; return float(max(np.linalg.norm(r[6 * i:6 * i + 3]) for i in g.case_fixed))   # largest pad force
        out["case_N_ceiling"] = case_share(Uc[:, 0], Fc[0])
        out["case_N_flip"] = case_share(st["flip_6g_crown"], 6 * W)
        out["case_N_hit"] = float(max(case_share(U1[:, j], 0) for j in range(U1.shape[1])))
        out["case_N_swap"] = case_share(st["swap_15N"], 15)
    else:
        out.update(ceiling_idx=None, ceiling_N=None, case_N_ceiling=0.0, case_N_flip=0.0, case_N_hit=0.0, case_N_swap=0.0)
    # mass
    m = g.mass() + 4 * MASS_E["corner"] + 8 * MASS_E["wheel"] + 2 * MASS_E["side_hub"] + 4 * MASS_E["foot"]
    sp = spokes_of(opt); tee_axes = {(a, r) for a, _, _, r in sp}
    if opt["side_strut"]: tee_axes |= {(1, "L"), (3, "L")}
    if opt["tail_strut"]: tee_axes |= {(2, "L")}; m += MASS_E["tail_hub"]
    m += len(tee_axes) * MASS_E["tee"]
    if sp: m += MASS_E["crown_base"] + len(sp) * MASS_E["crown_socket"]
    out["mass"] = float(m)
    out["ok"] = bool(out["touch"] <= 1.0 and max(idxs) <= 1.0 and out["swap_defl"] <= 3.0 and out["clr1"] > 0)
    return out, g

def all_opts():
    """17 Oct 03:25: spokes land on the upper ring only (spoke_route.py: the lower ring is out of reach past the props)"""
    for nose in ("none", "single", "V"):
        for tail in ("none", "single", "V"):
            for side in ("none", "single", "V"):
                for ss in (True, False):
                    for ts in (True, False):
                        for up in (2.0, 2.5):
                            yield dict(nose=(nose, "U"), tail=(tail, "U"), side=(side, "U"), side_strut=ss, tail_strut=ts, tee_mat="T", upper=up)

def skirt_opts():
    """one quadrant of the receiver skirt, mirrored to all 4 (17 Oct 04:00); crown spokes as layout C"""
    tg = [("s", a) for a in (0.0,) + STATIONS] + [("e", a) for a in STATIONS]
    sets = [[t] for t in tg] + [[a, b] for a in tg if a[0] == "s" for b in tg if b[0] == "e"]
    for xr in (15.0, 30.5):
        for zr in (-35.0, -25.0, -15.0, -5.0, 5.0):
            for ts in sets:
                ok = all(skirt_ok(np.array([xr, RCV_Y, zr]), skirt_target(t, 1, 1, PZ - 8.0))[0] >= 0 for t in ts)
                if ok: yield dict(nose=("V", "U"), tail=("V", "U"), side=("single", "U"), side_strut=False, tail_strut=False,
                                  tee_mat="T", upper=2.0, bottom=dict(quad=dict(x=xr, z=zr, targets=[list(t) for t in ts])))

if __name__ == "__main__":
    if sys.argv[1] == "skirt":
        part, nparts = int(sys.argv[2]), int(sys.argv[3]); rows = []; t0 = time.time()
        for i, o in enumerate(skirt_opts()):
            if i % nparts != part: continue
            r, _ = evaluate(o); r.update(i=i); rows.append(r)
        json.dump(rows, open(f"skirt_{part}.json", "w")); print("done", len(rows), f"{time.time() - t0:.0f} s")
    elif sys.argv[1] == "search":
        part, nparts = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (0, 1)
        rows = []; t0 = time.time(); skipped = 0
        for i, o in enumerate(all_opts()):
            if i % nparts != part: continue
            gp, gl = geom_ok(o)
            if gp < 12.0 or gl < 0.0: skipped += 1; continue
            r, _ = evaluate(o); r.update(prop_clear=gp, lane_clear=gl, i=i); rows.append(r)
            if len(rows) % 20 == 0: print(f"{len(rows)} done, {skipped} geometry misses, {time.time() - t0:.0f} s", flush=True)
        json.dump(rows, open(f"layout_v3_{part}.json", "w")); print("done", len(rows), "skipped", skipped, f"{time.time() - t0:.0f} s")
    elif sys.argv[1] == "one":
        o = json.loads(sys.argv[2]); print(geom_ok(o)); t = time.time(); r, g = evaluate(o); print(json.dumps(r, indent=1), time.time() - t)
