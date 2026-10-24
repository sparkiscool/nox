"""Overseer cage: geometry, 3D beam frame model and topology + size optimization (24 Oct).

Spark 24 Oct: "this cage is way too flimsy, make sure the lidar is mounted tight and the cage can take impact and the
bottom can land without toppling over, run a serious design optimization"; "use the 3mm rods whenever we can".

    python3 nox/overseer/cage_opt.py <params.json> <out.json>

params.json (from layout_v3.py): dm, xc, wz, xB, zB (drone frame mm, see layout_v3.py). out.json: the chosen topology,
the rod size per group, the frame-model results of every case, and the search table.

Frame model (F): Euler-Bernoulli 12-DOF beams, rigid joints (epoxied sockets in PETG nodes; node compliance not
modelled, E), the hub (plates, root blocks, front hub) fixed. Carbon E 120 GPa, G 5 GPa (pultruded UD, E); PETG
E 2780 MPa (nox/topo material), G 1000 (E). Units mm, N, MPa, t (tonne), s.

Cases and limits (DEFAULT, E unless tagged):
- L2 mount: the lowest mode that moves the L2 (≥ 20 % of the modal kinetic energy on the L2 group) ≥ 200 Hz: over 1P at
  full throttle (158) and 2P at hover (174) with margin (motor band E, dimensions O Arms and hub).
- Guard hits: the whole drone (2.33 kg E) at 1.0 m/s into a rigid wall, square to the guard, at every ring position and
  the V midpoints: peak force from the local stiffness (F = v sqrt(k m)); guard moves ≤ 8 toward the nearest prop
  (12 gap, 4 kept); rod stress ≤ 600 MPa (pultruded carbon flexural ~1200-1500, E); compressed rods ≥ 1.5 x Euler
  (pinned, between structural nodes); socket axial ≤ 600 N per 3.0 rod, scaled with rod OD (epoxy ~7 MPa over a
  10 deep socket, E). Arm (PETG) stress reported (≤ 25 MPa), not sized here. A hit on the L2 nose itself is taken at
  0.5 m/s (DEFAULT: nothing may sit ahead of the L2's field, so the cage cannot guard it; slow indoor bumps), stress,
  sockets and buckling only.
- Latch and swap (G (5), (7)): 100 N misuse down the slope on each of the 4 ±y-face wheels; 50 N into the face on two
  x-face wheels: rod stress and sockets as above.
- Landing on the floor (no tipping) is layout_v3.py's job (stance vs CG); the cage carries no floor load.
Objective: cage mass (rods + printed nodes); 3.0 solid rods wherever they pass, thicker only where a case needs it.
"""
import json, math, sys, itertools
import numpy as np
import scipy.linalg as sla

# ---------------------------------------------------------------- shared geometry numbers (dimensions O, C)
HALF, PROP_R, PROP_Z, BLADE = 139.5, 127.0, 82.0, 5.0
ROOT_H = 45.0
H_TIP = {"front": 38.0, "rear": 16.0}
W_ARM = {"front": (34.0, 28.0), "rear": (26.0, 22.0)}
ARM_WALL = 2.4
GUARD_GAP, ROD_R3 = 12.0, 1.5
RING_LO, RING_HI = PROP_Z - 14.0, PROP_Z + 10.0
F_FLAT = HALF + PROP_R + GUARD_GAP + ROD_R3                 # 280
C_CHAMF = 2 * HALF + math.sqrt(2) * (PROP_R + GUARD_GAP + ROD_R3)   # 477.7
L2_TILT = 105.0
A = np.array([math.sin(math.radians(L2_TILT)), 0.0, math.cos(math.radians(L2_TILT))])
UP_B = np.array([-A[2], 0.0, A[0]])
MOTOR_DEG = (45, 135, 225, 315)

# sections (carbon, C): A mm2, I mm4, J mm4, g/m, OD
SECTIONS = {
    "r3": dict(A=7.07, I=3.98, J=7.95, gpm=11.0, od=3.0, label="3.0 solid"),
    "r4": dict(A=12.57, I=12.57, J=25.1, gpm=19.5, od=4.0, label="4.0 solid"),
    "t6": dict(A=15.71, I=51.05, J=102.1, gpm=24.4, od=6.0, label="6/4 tube"),
    "r5": dict(A=19.63, I=30.68, J=61.4, gpm=30.4, od=5.0, label="5.0 solid"),
    "t8": dict(A=21.99, I=137.4, J=274.9, gpm=34.1, od=8.0, label="8/6 tube"),
    "t10": dict(A=28.27, I=289.8, J=579.6, gpm=43.8, od=10.0, label="10/8 tube"),
}
E_C, G_C, E_P, G_P = 120000.0, 5000.0, 2780.0, 1000.0
SIG_ALLOW, SOCKET_3, BUCKLE_SF = 600.0, 600.0, 1.5
DEFL_ALLOW, F_L2_MIN = 8.0, 200.0
M_DRONE, V_HIT = 2.33e-3, 1000.0                            # t, mm/s
NODE_G, CORNER_G = 2.0, 6.0                                 # printed ring node / corner node (E, g)

TOPO_KEYS = ("mid_spokes", "ring_diag", "l2_braces", "v_diag", "strut2")


def motors(dm):
    return [(dm + HALF * (1 if d in (45, 315) else -1), HALF * (1 if d in (45, 135) else -1)) for d in MOTOR_DEG]


def sockets(xB, zB):
    B = np.array([xB, 0.0, zB])
    return {(lvl, sy): B - A * 5 + UP_B * (16 if lvl == "hi" else 2) + np.array([0, 46.0 * sy, 0]) for lvl in ("hi", "lo") for sy in (1, -1)}


def ring_positions(dm):
    """ordered ring positions (name, x, y, is_vertex, is_mid) from the front-left vertex round to the front-right one"""
    F, c = F_FLAT, C_CHAMF - F_FLAT
    V = [(F, c), (c, F), (-c, F), (-F, c), (-F, -c), (-c, -F), (c, -F), (F, -c)]
    out = []
    for k in range(7):
        a, b = np.array(V[k]), np.array(V[k + 1])
        out.append((f"V{k}", *(a + [dm, 0]), True, False))
        flat = k in (1, 3, 5)
        fr = (0.25, 0.5, 0.75) if flat else (0.5,)
        for f in fr:
            p = a + f * (b - a)
            out.append((f"P{k}_{int(f * 100)}", p[0] + dm, p[1], False, flat and f == 0.5))
    out.append(("V7", V[7][0] + dm, V[7][1], True, False))
    return out


def geometry(p, topo, sizes=None):
    """nodes {name: xyz}, members [(name, a, b, group, kind)], fixed node names, mass {node: t}, extra info.
    kind: 'rod' (group sized), 'arm' (PETG box), 'link' (rigid), 'pnode' (printed node post, PETG 10 x 10)."""
    dm, xc, wz, xB, zB = p["dm"], p["xc"], p["wz"], p["xB"], p["zB"]
    N, M, fixed, mass = {}, [], set(), {}
    def node(n, xyz):
        N[n] = np.asarray(xyz, float); return n
    def mem(name, a, b, group, kind="rod"):
        M.append((name, a, b, group, kind))
    mot = motors(dm)
    # hub (fixed): arm clamps at u 75 on the diagonals, front hub, bottom plate corners
    for k, d in enumerate(MOTOR_DEG):
        c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
        fixed.add(node(f"root{k}", (75 * c, 75 * s, 22.5)))
    for sy in (1, -1):
        fixed.add(node(f"fhub{sy:+d}", (96, 12 * sy, 8)))
    for sx in (1, -1):
        for sy in (1, -1):
            fixed.add(node(f"hubc{sx:+d}{sy:+d}", (88 * sx, 52 * sy, -1)))
    # arms: beam line from the clamp to the motor, split at quarters + strut attachments (filled below)
    arm_pts = {k: [0.25, 0.5, 0.75] for k in range(4)}
    corners = []
    for sx in (1, -1):
        for sy in (1, -1):
            sr = np.array([xc + 110 * sx, 94 * sy, wz + 4])
            best = None
            for k, (mx, my) in enumerate(mot):
                r0 = np.array(N[f"root{k}"][:2]); v = np.array([mx, my]) - r0
                t = float(np.clip(np.dot(sr[:2] - r0, v) / np.dot(v, v), 0.05, 0.95)); q = r0 + t * v
                dd = float(np.hypot(*(q - sr[:2])))
                if best is None or dd < best[0]: best = (dd, k, t)
            corners.append((sx, sy, sr, best[1], best[2])); arm_pts[best[1]].append(best[2])
    for k, (mx, my) in enumerate(mot):
        side = "front" if mx > dm else "rear"; ht = H_TIP[side]
        r0 = N[f"root{k}"]; m1 = np.array([mx, my, ROOT_H - ht / 2])
        ts = sorted(set([0.0] + arm_pts[k] + [1.0]))
        names = []
        for t in ts:
            if t == 0.0: names.append(f"root{k}"); continue
            nm = f"mot{k}" if t == 1.0 else f"arm{k}_{t:.3f}"
            node(nm, r0 + t * (m1 - r0)); names.append(nm)
        for i in range(len(names) - 1):
            mem(f"arm{k}.{i}", names[i], names[i + 1], (side, k, ts[i], ts[i + 1]), "arm")
        node(f"tipb{k}", (mx, my, ROOT_H - ht)); mem(f"tiplink{k}", f"mot{k}", f"tipb{k}", None, "link")
        mass[f"mot{k}"] = mass.get(f"mot{k}", 0) + 91e-6
        mass[f"arm{k}_0.500"] = mass.get(f"arm{k}_0.500", 0) + 25e-6          # ESC on the arm (E)
    # guard rings
    rp = ring_positions(dm)
    diag_on, mid_on = topo["ring_diag"], topo["mid_spokes"]
    pnodes = []
    for i, (nm, x, y, isv, ismid) in enumerate(rp):
        node(f"{nm}lo", (x, y, RING_LO)); node(f"{nm}hi", (x, y, RING_HI))
        if isv or (ismid and mid_on) or diag_on:
            mem(f"post {nm}", f"{nm}lo", f"{nm}hi", None, "pnode"); pnodes.append(nm)
    for i in range(len(rp) - 1):
        a, b = rp[i][0], rp[i + 1][0]
        for lvl in ("lo", "hi"):
            mem(f"chord {a}-{b} {lvl}", f"{a}{lvl}", f"{b}{lvl}", "chords")
        if diag_on:
            lo_first = i % 2 == 0
            mem(f"diag {a}-{b}", f"{a}lo" if lo_first else f"{a}hi", f"{b}hi" if lo_first else f"{b}lo", "ring_diag")
    vx = [r for r in rp if r[3]]
    for k, (mx, my) in enumerate(mot):
        near = sorted(vx, key=lambda r: math.hypot(r[1] - mx, r[2] - my))[:2]
        for r in near:
            mem(f"spoke m{k}-{r[0]}", f"tipb{k}", f"{r[0]}lo", "spokes")
    if mid_on:
        for r in rp:
            if not r[4]: continue
            two = sorted(range(4), key=lambda k: math.hypot(mot[k][0] - r[1], mot[k][1] - r[2]))[:2]
            for k in two:
                mem(f"midspoke m{k}-{r[0]}", f"tipb{k}", f"{r[0]}lo", "mid_spokes")
    # front V to the L2 apex: sockets on the apex lugs, rigid to the L2 group node
    S = sockets(xB, zB)
    apex = node("apexM", np.array([xB, 0, zB]) + A * 30)
    for (lvl, sy), q in S.items():
        node(f"S{lvl}{sy:+d}", q); mem(f"apexlink {lvl}{sy:+d}", "apexM", f"S{lvl}{sy:+d}", None, "link")
    mem("apexlink lohi+", "Slo+1", "Shi+1", None, "link"); mem("apexlink lohi-", "Slo-1", "Shi-1", None, "link")
    mem("apexlink lo+-", "Slo+1", "Slo-1", None, "link")
    mass["apexM"] = 266e-6                                                 # L2 230 + camera 15 + plate, rods, apex ~21 (E)
    for vn, sy in (("V0", 1), ("V7", -1)):
        for lvl in ("lo", "hi"):
            mid = node(f"Vmid {vn}{lvl}", (N[f"{vn}{lvl}"] + N[f"S{lvl}{sy:+d}"]) / 2)
            mem(f"V {vn}{lvl} a", f"{vn}{lvl}", mid, "V"); mem(f"V {vn}{lvl} b", mid, f"S{lvl}{sy:+d}", "V")
        if topo["v_diag"]:
            mem(f"vdiag {vn}", f"{vn}lo", f"Shi{sy:+d}", "v_diag")
    for sy in (1, -1):
        mem(f"keel {sy:+d}", f"fhub{sy:+d}", f"Slo{sy:+d}", "keels")
    if topo["l2_braces"]:
        for k, (mx, my) in enumerate(mot):
            if mx > dm:
                sy = 1 if my > 0 else -1
                mem(f"brace m{k}", f"tipb{k}", f"Slo{sy:+d}", "l2_braces")
    # wheel ring: corner nodes (rigid blocks), ring rods on the wheel lines, struts to the arms (+ optional to the hub)
    for sx, sy, sr, k, t in corners:
        cn = node(f"corner{sx:+d}{sy:+d}", (xc + 106 * sx, 106 * sy, wz))
        node(f"WA{sx:+d}{sy:+d}", (xc + 112 * sx, 100 * sy, wz)); node(f"WB{sx:+d}{sy:+d}", (xc + 100 * sx, 112 * sy, wz))
        node(f"WAi{sx:+d}{sy:+d}", (xc + 112 * sx, 94.5 * sy, wz)); node(f"WBi{sx:+d}{sy:+d}", (xc + 94.5 * sx, 112 * sy, wz))
        node(f"SR{sx:+d}{sy:+d}", sr)
        for w in ("WA", "WB", "WAi", "WBi", "SR"):
            mem(f"cornerlink {w}{sx:+d}{sy:+d}", cn, f"{w}{sx:+d}{sy:+d}", None, "link")
        mass[cn] = CORNER_G * 1e-6 + 0.5e-6
        an = f"arm{k}_{t:.3f}"
        side = "front" if mot[k][0] > dm else "rear"
        bot = node(f"armbot{sx:+d}{sy:+d}", (N[an][0], N[an][1], t * (ROOT_H - H_TIP[side])))
        mem(f"armbotlink{sx:+d}{sy:+d}", an, bot, None, "link")
        mem(f"strut {sx:+d}{sy:+d}", f"SR{sx:+d}{sy:+d}", bot, "struts")
        if topo["strut2"]:
            mem(f"strut2 {sx:+d}{sy:+d}", f"SR{sx:+d}{sy:+d}", f"hubc{sx:+d}{sy:+d}", "strut2")
    for sx in (1, -1):
        mem(f"wring x{sx:+d}", f"WAi{sx:+d}+1", f"WAi{sx:+d}-1", "wheel_ring")
    for sy in (1, -1):
        mem(f"wring y{sy:+d}", f"WBi+1{sy:+d}", f"WBi-1{sy:+d}", "wheel_ring")
    info = dict(ring=rp, pnodes=pnodes, motors=mot, corners=corners, sockets=S)
    return N, M, fixed, mass, info


# ---------------------------------------------------------------- beam FE
def arm_section(side, t0, t1):
    t = (t0 + t1) / 2; ht = H_TIP[side]; b0, b1 = W_ARM[side]
    h = ROOT_H + (ht - ROOT_H) * t; b = b0 + (b1 - b0) * t; w = ARM_WALL
    Iy = (b * h ** 3 - (b - 2 * w) * (h - 2 * w) ** 3) / 12; Iz = (h * b ** 3 - (h - 2 * w) * (b - 2 * w) ** 3) / 12
    Am = (b - w) * (h - w); J = 4 * Am ** 2 * w / (2 * (b - w) + 2 * (h - w)); A_ = b * h - (b - 2 * w) * (h - 2 * w)
    return dict(A=A_, Iy=Iy, Iz=Iz, J=J, E=E_P, G=G_P, c=max(b, h) / 2, rho=1.27e-9 * 0.45 / 0.45, solid_frac=1.0, h=h, b=b)


def elem_props(kind, group, sizes):
    if kind == "arm":
        side = group[0]; s = arm_section(side, group[2], group[3]); s["rho"] = 1.27e-9; return s
    if kind == "link":
        return dict(A=2000.0, Iy=4e5, Iz=4e5, J=8e5, E=E_P * 20, G=G_P * 20, c=20.0, rho=0.0)
    if kind == "pnode":
        return dict(A=100.0, Iy=833.0, Iz=833.0, J=1400.0, E=E_P, G=G_P, c=5.0, rho=0.0)
    s = SECTIONS[sizes[group]]
    return dict(A=s["A"], Iy=s["I"], Iz=s["I"], J=s["J"], E=E_C, G=G_C, c=s["od"] / 2, rho=s["gpm"] * 1e-9, od=s["od"])


def k_local(L, P):
    E, G, A_, Iy, Iz, J = P["E"], P["G"], P["A"], P["Iy"], P["Iz"], P["J"]
    k = np.zeros((12, 12))
    def put(i, j, v): k[i, j] += v; k[j, i] += v if i != j else 0
    ea, gj = E * A_ / L, G * J / L
    k[0, 0] = k[6, 6] = ea; k[0, 6] = k[6, 0] = -ea
    k[3, 3] = k[9, 9] = gj; k[3, 9] = k[9, 3] = -gj
    for (v1, t1, v2, t2, I, sg) in ((1, 5, 7, 11, Iz, 1), (2, 4, 8, 10, Iy, -1)):
        a = 12 * E * I / L ** 3; b = 6 * E * I / L ** 2; c = 4 * E * I / L; d = 2 * E * I / L
        k[v1, v1] = k[v2, v2] = a; k[v1, v2] = k[v2, v1] = -a
        k[t1, t1] = k[t2, t2] = c; k[t1, t2] = k[t2, t1] = d
        k[v1, t1] = k[t1, v1] = sg * b; k[v1, t2] = k[t2, v1] = sg * b
        k[v2, t1] = k[t1, v2] = -sg * b; k[v2, t2] = k[t2, v2] = -sg * b
    return k


def rot(p0, p1):
    ex = (p1 - p0) / np.linalg.norm(p1 - p0)
    ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.95 else np.array([1.0, 0, 0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    R = np.vstack([ex, ey, ez]); T = np.zeros((12, 12))
    for i in range(4): T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R
    return T


class Model:
    def __init__(self, p, topo, sizes):
        self.p, self.topo, self.sizes = p, topo, sizes
        self.N, self.M, self.fixed, self.mass, self.info = geometry(p, topo, sizes)
        self.names = list(self.N); self.idx = {n: i for i, n in enumerate(self.names)}
        nd = 6 * len(self.names)
        K = np.zeros((nd, nd)); Mm = np.zeros(nd)
        self.el = []
        for name, a, b, group, kind in self.M:
            P = elem_props(kind, group, sizes); pa, pb = self.N[a], self.N[b]; L = float(np.linalg.norm(pb - pa))
            T = rot(pa, pb); kl = k_local(L, P); kg = T.T @ kl @ T
            dofs = np.r_[6 * self.idx[a]:6 * self.idx[a] + 6, 6 * self.idx[b]:6 * self.idx[b] + 6]
            K[np.ix_(dofs, dofs)] += kg
            m = P["rho"] * P["A"] * L if kind in ("rod", "arm") else 0.0
            for n_ in (a, b):
                i = 6 * self.idx[n_]; Mm[i:i + 3] += m / 2; Mm[i + 3:i + 6] += m / 2 * (L ** 2) / 24 + 1e-12
            self.el.append(dict(name=name, a=a, b=b, group=group, kind=kind, P=P, L=L, T=T, kl=kl, dofs=dofs))
        for n_, m in self.mass.items():
            i = 6 * self.idx[n_]; Mm[i:i + 3] += m; Mm[i + 3:i + 6] += m * 400.0          # rotary ~ m (20 mm)^2 (E)
        i = 6 * self.idx["apexM"]; Mm[i + 3:i + 6] += 0.2                                  # L2 rotary inertia ~2e-4 kg m2 (E)
        Mm[Mm == 0] = 1e-12
        free = np.ones(nd, bool)
        for n_ in self.fixed: free[6 * self.idx[n_]:6 * self.idx[n_] + 6] = False
        self.free = np.where(free)[0]; self.K, self.Mm = K, Mm
        Kf = K[np.ix_(self.free, self.free)]
        self.cho = sla.cho_factor(Kf + np.eye(len(self.free)) * 1e-9)

    def solve(self, loads):
        """loads: list of {node: force xyz}; returns full displacement vectors"""
        nd = 6 * len(self.names); R = np.zeros((nd, len(loads)))
        for j, ld in enumerate(loads):
            for n_, f in ld.items(): R[6 * self.idx[n_]:6 * self.idx[n_] + 3, j] += f
        U = np.zeros_like(R); U[self.free] = sla.cho_solve(self.cho, R[self.free]); return U

    def modes(self, n=24):
        Kf = self.K[np.ix_(self.free, self.free)]; Mf = self.Mm[self.free]
        w2, V = sla.eigh(Kf, np.diag(Mf), subset_by_index=[0, n - 1])
        f = np.sqrt(np.maximum(w2, 0)) / (2 * math.pi)
        ia = [list(self.free).index(6 * self.idx["apexM"] + j) for j in range(6)]
        frac = np.array([float((V[ia, m] ** 2 * Mf[ia]).sum()) for m in range(V.shape[1])])
        return f, frac

    def member_results(self, U):
        """per element: axial N (tension +), max bending stress, end axial forces"""
        out = []
        for e in self.el:
            ue = U[e["dofs"]]; fl = e["kl"] @ (e["T"] @ ue)
            Nax = -fl[0]; My = max(abs(fl[4]), abs(fl[10])); Mz = max(abs(fl[5]), abs(fl[11]))
            P = e["P"]; sig = abs(Nax) / P["A"] + math.hypot(My, Mz) * P["c"] / max(P["Iy"], 1e-9)
            if e["kind"] == "arm":
                sig = abs(Nax) / P["A"] + My * (P["h"] / 2) / P["Iy"] + Mz * (P["b"] / 2) / P["Iz"]
            out.append((Nax, sig))
        return out

    def rod_mass_g(self):
        g = 0.0
        for e in self.el:
            if e["kind"] == "rod": g += SECTIONS[self.sizes[e["group"]]]["gpm"] * e["L"] / 1000.0
        g += NODE_G * len(self.info["pnodes"]) + CORNER_G * 4
        return g


def structural_members(model):
    """rods between structural nodes: chords and V are split at ring positions / midpoints; buckling uses the run
    between nodes that carry another member (a posted ring node, a spoke, the apex) or the whole rod otherwise"""
    deg = {}
    for e in model.el:
        for n_ in (e["a"], e["b"]): deg[n_] = deg.get(n_, 0) + 1
    runs, used = [], set()
    rods = [i for i, e in enumerate(model.el) if e["kind"] == "rod"]
    by_node = {}
    for i in rods:
        for n_ in (model.el[i]["a"], model.el[i]["b"]): by_node.setdefault(n_, []).append(i)
    def is_joint(n_):
        same = [i for i in by_node.get(n_, [])]
        return deg.get(n_, 0) != 2 or len(same) != 2
    for i in rods:
        if i in used: continue
        chain = [i]; used.add(i)
        for end in ("a", "b"):
            n_ = model.el[i][end]
            while not is_joint(n_):
                nxt = [j for j in by_node[n_] if j not in used]
                if not nxt: break
                j = nxt[0]; used.add(j); chain.append(j)
                n_ = model.el[j]["b"] if model.el[j]["a"] == n_ else model.el[j]["a"]
        runs.append(chain)
    return runs


def evaluate(p, topo, sizes, detail=False):
    m = Model(p, topo, sizes)
    res = dict(mass_g=m.rod_mass_g(), cases={})
    g = {}
    # L2 mode
    f, frac = m.modes()
    l2 = [fi for fi, fr in zip(f, frac) if fr >= 0.2]
    f_l2 = float(l2[0]) if l2 else float(f[-1])
    res["f_first"], res["f_l2"] = float(f[0]), f_l2
    g["L2 mode"] = F_L2_MIN / f_l2 - 1
    # hit and latch cases
    cases = []
    rp = m.info["ring"]
    for nm, x, y, isv, ismid in rp:
        # outward normal: flats along the axes, chamfers / vertices diagonal (from the octagon)
        rx, ry = x - p["dm"], y
        if abs(abs(rx) - F_FLAT) < 1e-6 and abs(ry) <= C_CHAMF - F_FLAT + 1e-6: n = np.array([np.sign(rx), 0, 0])
        elif abs(abs(ry) - F_FLAT) < 1e-6 and abs(rx) <= C_CHAMF - F_FLAT + 1e-6: n = np.array([0, np.sign(ry), 0])
        else: n = np.array([np.sign(rx), np.sign(ry), 0]) / math.sqrt(2)
        cases.append((f"hit {nm}", {f"{nm}lo": -0.5 * n, f"{nm}hi": -0.5 * n}, n, (f"{nm}lo", f"{nm}hi")))
    for vn, sy in (("V0", 1), ("V7", -1)):
        q = m.N[f"Vmid {vn}lo"]; n = np.array([1.0, sy * 0.5, 0]); n /= np.linalg.norm(n)
        cases.append((f"hit V {vn}", {f"Vmid {vn}lo": -0.5 * n, f"Vmid {vn}hi": -0.5 * n}, n, (f"Vmid {vn}lo", f"Vmid {vn}hi")))
    cases.append(("hit L2 nose", {"apexM": np.array([-1.0, 0, 0])}, np.array([1.0, 0, 0]), ("apexM",)))
    loads = [c[1] for c in cases]
    latch = []
    for sx in (1, -1):
        for sy in (1, -1):
            latch.append((f"latch WB{sx:+d}{sy:+d}", {f"WB{sx:+d}{sy:+d}": 100.0 * np.array([0, -sy, -1]) / math.sqrt(2)}))
    latch.append(("swap pull x+", {"WA+1+1": 50.0 * np.array([1, 0, -1]) / math.sqrt(2), "WA+1-1": 50.0 * np.array([1, 0, -1]) / math.sqrt(2)}))
    U = m.solve(loads + [c[1] for c in latch])
    runs = structural_members(m)
    mot_nodes = [f"mot{k}" for k in range(4)]
    worst = {}
    for j, (cname, ld, n, pts) in enumerate(cases):
        u = U[:, j]
        d1 = float(np.mean([-(u[6 * m.idx[q]:6 * m.idx[q] + 3] @ n) for q in pts])) / 1.0     # mm per N, inward
        k = 1.0 / max(d1, 1e-9)                                                             # N/mm
        Fpk = (0.5 if cname == "hit L2 nose" else 1.0) * V_HIT * math.sqrt(k * M_DRONE)
        hit_xy = np.mean([m.N[q] for q in pts], axis=0)
        km = min(range(4), key=lambda i: np.hypot(*(m.N[mot_nodes[i]][:2] - hit_xy[:2])))
        dmot = float(-(u[6 * m.idx[mot_nodes[km]]:6 * m.idx[mot_nodes[km]] + 3] @ n))
        defl = Fpk * (d1 - dmot)
        mr = m.member_results(u * Fpk)
        sig_rod = max([s for (N_, s), e in zip(mr, m.el) if e["kind"] == "rod"] + [0])
        sig_arm = max([s for (N_, s), e in zip(mr, m.el) if e["kind"] == "arm"] + [0])
        sock = max([abs(N_) / (SOCKET_3 * e["P"]["od"] / 3.0) for (N_, s), e in zip(mr, m.el) if e["kind"] == "rod"] + [0])
        buck = 0.0
        for run in runs:
            e0 = m.el[run[0]]; P = e0["P"]; L = sum(m.el[i]["L"] for i in run)
            Ncomp = max(-mr[i][0] for i in run)
            if Ncomp > 0: buck = max(buck, BUCKLE_SF * Ncomp / (math.pi ** 2 * P["E"] * P["Iy"] / L ** 2))
        r = dict(k=k, F=Fpk, defl=defl, sig_rod=sig_rod, sig_arm=sig_arm, socket=sock, buckle=buck)
        res["cases"][cname] = r
        if cname != "hit L2 nose":
            g[f"{cname} defl"] = defl / DEFL_ALLOW - 1
        g[f"{cname} stress"] = sig_rod / SIG_ALLOW - 1
        g[f"{cname} socket"] = sock - 1
        g[f"{cname} buckle"] = buck - 1
    for jj, (cname, ld) in enumerate(latch):
        u = U[:, len(cases) + jj]; mr = m.member_results(u)
        sig_rod = max([s for (N_, s), e in zip(mr, m.el) if e["kind"] == "rod"] + [0])
        sock = max([abs(N_) / (SOCKET_3 * e["P"]["od"] / 3.0) for (N_, s), e in zip(mr, m.el) if e["kind"] == "rod"] + [0])
        dmax = max(float(np.linalg.norm(u[6 * m.idx[n_]:6 * m.idx[n_] + 3])) for n_ in ld)
        res["cases"][cname] = dict(sig_rod=sig_rod, socket=sock, defl=dmax)
        g[f"{cname} stress"] = sig_rod / SIG_ALLOW - 1; g[f"{cname} socket"] = sock - 1
    res["g"] = g
    res["violation"] = float(sum(max(0.0, v) for v in g.values()))
    return (res, m) if detail else res


def groups_of(topo):
    gs = ["chords", "spokes", "V", "keels", "wheel_ring", "struts"]
    for k in TOPO_KEYS:
        if topo[k]: gs.append(k)
    return gs


ORDER = sorted(SECTIONS, key=lambda s: SECTIONS[s]["gpm"])


def size_topology(p, topo, log=None):
    sizes = {gname: "r3" for gname in groups_of(topo)}
    r = evaluate(p, topo, sizes); steps = 0
    while r["violation"] > 0 and steps < 40:
        best = None
        for gname in sizes:                                                # one size up per group, best violation drop per gram
            cur = ORDER.index(sizes[gname])
            for s in ORDER[cur + 1:cur + 3]:
                trial = dict(sizes, **{gname: s}); rt = evaluate(p, topo, trial)
                dmass = rt["mass_g"] - r["mass_g"]
                gain = (r["violation"] - rt["violation"]) / max(dmass, 0.5)
                if best is None or gain > best[0]: best = (gain, trial, rt)
        if best is None or best[0] <= 0: break
        sizes, r = best[1], best[2]; steps += 1
    if r["violation"] > 0:
        return sizes, r, False
    improved = True
    while improved:                                                        # take back what the feasible design does not need
        improved = False
        for gname in list(sizes):
            cur = ORDER.index(sizes[gname])
            for s in ORDER[:cur]:
                trial = dict(sizes, **{gname: s}); rt = evaluate(p, topo, trial)
                if rt["violation"] == 0 and rt["mass_g"] < r["mass_g"] - 0.05:
                    sizes, r, improved = trial, rt, True; break
    return sizes, r, True


def _one(args):
    p, bits = args
    topo = dict(zip(TOPO_KEYS, bits)); sizes, r, ok = size_topology(p, topo); return bits, topo, sizes, r, ok


def main(pin, pout):
    from multiprocessing import Pool
    p = json.load(open(pin))
    table = []
    best = None
    with Pool(4) as pool:
        done = pool.map(_one, [(p, bits) for bits in itertools.product((0, 1), repeat=len(TOPO_KEYS))])
    for bits, topo, sizes, r, ok in done:
        row = dict(topo=topo, sizes=sizes, feasible=ok, mass_g=r["mass_g"], f_l2=r["f_l2"], violation=r["violation"],
                   worst=max(r["g"], key=r["g"].get), worst_g=r["g"][max(r["g"], key=r["g"].get)])
        table.append(row)
        print(f"{''.join(map(str, bits))} {'OK ' if ok else 'no '} {r['mass_g']:6.1f} g  f_L2 {r['f_l2']:5.0f} Hz  worst {row['worst']} {row['worst_g']:+.2f}  "
              + " ".join(f"{k}:{v}" for k, v in sizes.items()), flush=True)
        if ok and (best is None or r["mass_g"] < best[2]["mass_g"]): best = (topo, sizes, r)
    out = dict(params=p, table=table)
    if best:
        topo, sizes, r = best
        out.update(topo=topo, sizes=sizes, result=r)
    json.dump(out, open(pout, "w"), indent=1, default=float)
    print("best:", None if not best else (best[0], best[1], round(best[2]["mass_g"], 1)))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
