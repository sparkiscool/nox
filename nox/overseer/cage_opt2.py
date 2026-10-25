"""Overseer cage v2: the full cage of the Overseer system pass (24 Oct): geometry, 3D beam frame model, topology + size
search. Spark 24 Oct: "this cage is way too flimsy, make sure the lidar is mounted tight and the cage can take impact and
the bottom can land without toppling over, run a serious design optimization"; "use the 3mm rods whenever we can"; and
the CLAUDE.md "Every drone design" list (cage first top, sides and bottom, balanced pack in and out, the L2 mounted for
what it carries, sensor mounts ≥ 100 Hz, breaks in the right order), cases from `dimensions.md` L.

    python3 nox/overseer/cage_opt2.py <params.json> <out.json>

params.json (from the system model `overseer_system.py`): dm (motor square centre ahead of the plate centre), xc (catch and
pack centre, = dm: the pack on the thrust axis), wz (wheel centres), xB, zB (L2 base centre), cam_top, cam_front (front
camera extremes), m_drone_g.

Cage (all 3.0 carbon unless a case needs more): guard rings round the props at z 68 and 104 (the upper ring is the top
perimeter), posts at the vertices, spokes from the arm tips; a top node over the plate centre on a 4-leg pyramid from the
top plate with a spider to the upper ring; a nose box round the L2 and camera, 3 mm + rod ahead of them (thin rods in
the L2 field, blocked share reported by the system model); the L2 lugs tied to the nose box and back to the core by
keels; the wheel ring with 4 corner nodes on struts; TPU feet on the sled (plate-fixed here).

Frame model (F): Euler-Bernoulli 12-DOF beams, rigid joints (node compliance not modelled, E), the hub (plates, root blocks,
front hub, sled) fixed. Carbon E 120 GPa, G 5 GPa (pultruded UD, E); PETG E 2780, G 1000. Units mm, N, MPa, t, s.

Cases (dimensions L, DEFAULT unless tagged) and limits:
- L2 mount: lowest mode with ≥ 20 % of its kinetic energy on the L2 group ≥ 100 Hz (L Sensor mounts).
- Crash, 35 g x the whole drone in 14 directions (6 faces, 8 corners), on the cage node that touches first in that
  direction (largest support) and on every node within CRASH_TIE (3) of it, hub fixed: sockets (600 N per 3.0 rod, scaled
  with OD) ≤ 1.0; rod index σ/600 MPa, buckling 1.5 x Euler and arm PETG σ/51 MPa ≤ 1 / NODE_FUSE (the cage node is the
  fuse, drawn to let go at ≤ 1 / 1.5 of its rod); the touching node moves ≤ 9 toward the drone (12 prop gap, 3 kept).
- L2 crash: 35 g x the L2 group at its CG in 14 directions: the same indices ≤ 1.0.
- Handling: 50 N on every real cage node in ±x, ±y, ±z: indices ≤ 1.0 (DEFAULT: a misuse load; L gives 0.5 to flight and landing).
- Catch landing: 6 g x the drone up through the 8 wheels; latch misuse 100 N down the slope on each ±y-face wheel (≤ 1.0);
  swap pull 50 N into the face on two wheels (≤ 0.5).
- Breaking order (L, reported): in the crash cases the arms stay at <= 1 / 1.5 of the worst cage node socket index; the held
  joints (hub, L2 apex, belly-frame sockets) get a requirement: a rated strength that keeps their index there too.
  The worst rod index is reported beside it.
Objective: cage mass (rods + printed nodes + arm spars); 3.0 solid wherever it passes.
"""
import json, math, os, re, sys, itertools
import numpy as np
import scipy.linalg as sla

# ---------------------------------------------------------------- shared geometry numbers (dimensions O, C)
HALF, PROP_R, PROP_Z, BLADE = 139.5, 127.0, 82.0, 5.0
ROOT_H = 45.0
H_TIP = {"front": 38.0, "rear": 16.0}
W_ARM = {"front": (34.0, 28.0), "rear": (26.0, 22.0)}
ARM_WALL = 2.4
GUARD_GAP, ROD_R3 = 12.0, 1.5
ROD_RMAX = 10.0                                             # guard offset for the largest catalog rod (20/18 tube), so any size keeps 12
RING_LO, RING_HI = PROP_Z - BLADE - 12.0 - ROD_RMAX, PROP_Z + BLADE + 12.0 + ROD_RMAX   # 55 / 109: spokes and spiders keep 12 off the swept band with a 20 tube
Z_TOP = 124.0                                               # top node over the thrust centre (DEFAULT)
NOSE_OD_MAX = 10.0                                          # nose collar rods ≤ 10 od (cap groups nose / nose_mount t10): sets its node blocks
NOSE_Y, NOSE_GAP = 58.0, 3.0 + 4.0                          # nose box half width; ahead of the L2 / camera by 3 + rod (nose plane rods ≤ 8 od: cap group nose_x t8)
F_FLAT = HALF + PROP_R + GUARD_GAP + ROD_RMAX               # 288.5
C_CHAMF = 2 * HALF + math.sqrt(2) * (PROP_R + GUARD_GAP + ROD_RMAX)   # 489.6
L2_TILT = 105.0
A = np.array([math.sin(math.radians(L2_TILT)), 0.0, math.cos(math.radians(L2_TILT))])
UP_B = np.array([-A[2], 0.0, A[0]])
L2_LEN = 65.0                                                 # the L2 from its base plane to the dome tip (body 39 + dome 26, M)
L2_BASE_H = 25.0                                              # the L2 base (Spark 24 Oct): held and caged; the head above it is exposed
L2_NEG = 6.0                                                  # scan kept clear in the 96 deg negative-angle mode (DEFAULT, alt 90 deg normal mode)
MOTOR_DEG = (45, 135, 225, 315)
HELD_NODES = ("Slo", "Shi", "SP", "apexM", "BF")                  # L2 mount sockets and the belly frame corners (pack holder)
SOCK_BACK = 13.0                                            # socket centres behind the L2 base plane: 20 tube radius 10 + 3
APEX_SPAN = 56.0                                            # socket spread on the apex lugs (DEFAULT, inside the L2's 75)

# sections (carbon, C): A mm2, I mm4, J mm4, g/m, OD
SECTIONS = {
    "r3": dict(A=7.07, I=3.98, J=7.95, gpm=11.0, od=3.0, label="3.0 solid"),
    "r4": dict(A=12.57, I=12.57, J=25.1, gpm=19.5, od=4.0, label="4.0 solid"),
    "t6": dict(A=15.71, I=51.05, J=102.1, gpm=24.4, od=6.0, label="6/4 tube"),
    "r5": dict(A=19.63, I=30.68, J=61.4, gpm=30.4, od=5.0, label="5.0 solid"),
    "t8": dict(A=21.99, I=137.4, J=274.9, gpm=34.1, od=8.0, label="8/6 tube"),
    "t10": dict(A=28.27, I=289.8, J=579.6, gpm=43.8, od=10.0, label="10/8 tube"),
    "t12": dict(A=34.56, I=527.0, J=1054.0, gpm=53.6, od=12.0, label="12/10 tube"),   # weight no cap (Spark 24 Oct): stiffer tubes allowed
    "t14": dict(A=40.84, I=867.9, J=1735.8, gpm=63.3, od=14.0, label="14/12 tube"),
    "t16": dict(A=47.12, I=1331.5, J=2663.0, gpm=73.0, od=16.0, label="16/14 tube"),
    "t20": dict(A=59.69, I=2701.0, J=5402.0, gpm=92.5, od=20.0, label="20/18 tube"),
    # thick wall and solid at the capped ODs (prop wash 10, nose, pods): more section where a larger OD may not go (DEFAULT 25 Oct)
    "t6h": dict(A=21.21, I=59.65, J=119.3, gpm=33.0, od=6.0, label="6/3 tube"),
    "t8h": dict(A=37.70, I=188.5, J=377.0, gpm=58.7, od=8.0, label="8/4 tube"),
    "t10h": dict(A=50.27, I=427.3, J=854.6, gpm=78.2, od=10.0, label="10/6 tube"),
    "r10": dict(A=78.54, I=490.9, J=981.7, gpm=122.2, od=10.0, label="10.0 solid"),
}
E_C, G_C, E_P, G_P = 120000.0, 5000.0, 2780.0, 1000.0
SIG_ALLOW, SOCKET_3, BUCKLE_SF = 600.0, 600.0, 1.5
SOCKET_D, SOCKET_D_MAX = 20.0, 30.0                          # socket depth: 600 N per 3.0 rod at 20 deep (E), scaled with OD and depth; a node or hub
                                                              # may draw a socket up to 30 deep where its rod needs it (DEFAULT): the gate, deeper ones listed
SOCKET_CAP = SOCKET_3 * SOCKET_D_MAX / SOCKET_D / 3.0         # N per mm OD at the deepest socket
DEFL_ALLOW, F_L2_MIN = 9.0, 100.0                              # crash deflection, L2 mode (L Sensor mounts)
F_FIRST_MIN = 0.0                                           # cage's own lowest mode floor (set below after the trade)
M_DRONE, V_HIT = 2.33e-3, 1000.0                            # t, mm/s
CRASH_TIE = 3.0                                             # nodes this close to the leading one in a crash direction get their own case (the cage margin, L)
CARBON_MIN = 33.0                                           # carbon break-even (CLAUDE.md): a shorter link is printed into its node
NODE_G, CORNER_G = 2.0, 6.0                                 # (retired: nodes are sized from their rods) / corner node (E, g)
NODE_WALL = 3.0                                             # printed node: socket bore OD + 0.3, wall 3 round it (CLAUDE.md attachments)
PRINTED_NODES = ("NT", "NB", "NR", "TN", "TR", "KV", "KP", "FT", "MX", "XJ", "SM")


def node_size(od):
    """drawn printed node: a block round its largest socket (bore OD + 0.3, NODE_WALL a side), never under 12"""
    return max(12.0, od + 0.3 + 2 * NODE_WALL)


SOCK_LEN = 20.0 + NODE_WALL                                   # a socket boss from the node centre: 20 deep + its 3 bottom wall (deeper ones listed)
NODE_FILL = 0.8                                                # printed walls round the bores, mostly solid perimeters (E)


def node_mass_g(ods):
    """printed PETG node drawn from its sockets: a core round the largest bore and one boss per rod (bore OD + 0.3, NODE_WALL
    round it, SOCK_LEN from the centre; E). ods: the OD of every rod in it (a float = one rod)"""
    ods = [ods] if np.isscalar(ods) else list(ods)
    rc = max(ods) / 2 + 0.15 + NODE_WALL
    v = 4.0 / 3.0 * math.pi * rc ** 3 + sum(math.pi * ((o / 2 + 0.15 + NODE_WALL) ** 2 - (o / 2 + 0.15) ** 2) * (SOCK_LEN - rc) for o in ods if SOCK_LEN > rc)
    return 1.27e-3 * NODE_FILL * v

TOPO_KEYS = ("spine", "mid_spokes", "ring_diag", "l2_braces", "tip_rails", "keels_lo", "keels_x", "chin", "arm_spars", "top_spider")   # no nose cross: it would sit in the L2 scan
# retired from the search (24 Oct, system v3): keels (prop guard), v_diag (the cameras), strut2 (funnel), spokes_hi (motor bells): off


def motors(dm):
    return [(dm + HALF * (1 if d in (45, 315) else -1), HALF * (1 if d in (45, 135) else -1)) for d in MOTOR_DEG]


def sockets(xB, zB):
    B = np.array([xB, 0.0, zB])
    # apex sockets on lugs either side of the L2 body (y ±46), SOCK_BACK behind its base plane (the largest rod stays 3 out
    # of the field), 56 apart along the base plane: V lower to the lower pair; V upper, keels and arm braces to the upper
    # pair, so they climb to the L2 (bounce, pitch)
    return {(lvl, sy): B - A * SOCK_BACK + UP_B * (APEX_SPAN / 2 if lvl == "hi" else -APEX_SPAN / 2) + np.array([0, 46.0 * sy, 0])
            for lvl in ("hi", "lo") for sy in (1, -1)}


def ring_positions(dm, rear_x=None):
    """ordered ring positions (name, x, y, is_vertex, is_mid) from the front-left vertex round to the front-right one"""
    F, c = F_FLAT, C_CHAMF - F_FLAT
    Fr = max(F, dm - rear_x) if rear_x is not None else F           # rear flat moves back to cover the plates (hub aft of the motors)
    V = [(F, c), (c, F), (-c, F), (-Fr, c), (-Fr, -c), (-c, -F), (c, -F), (F, -c)]
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


_GEOM_CACHE = {}


def geometry(p, topo, sizes=None):
    """cached on the layout (positions do not depend on the sizes); returns copies"""
    key = json.dumps([p, topo], sort_keys=True, default=float)
    if key not in _GEOM_CACHE:
        if len(_GEOM_CACHE) > 64: _GEOM_CACHE.clear()
        _GEOM_CACHE[key] = _geometry(p, topo)
    N, M, fixed, mass, info = _GEOM_CACHE[key]
    return {k: v.copy() for k, v in N.items()}, list(M), set(fixed), dict(mass), info


def _geometry(p, topo, sizes=None):
    """nodes {name: xyz}, members [(name, a, b, group, kind)], fixed node names, mass {node: t}, extra info.
    kind: 'rod' (group sized), 'arm' (PETG box), 'link' (rigid), 'pnode' (printed node post, PETG 10 x 10)."""
    dm, xc, wz, xB, zB = p["dm"], p["xc"], p["wz"], p["xB"], p["zB"]
    N, M, fixed, mass = {}, [], set(), {}
    SPINE_X = []
    def node(n, xyz):
        N[n] = np.asarray(xyz, float); return n
    def mem(name, a, b, group, kind="rod"):
        M.append((name, a, b, group, kind))
    mot = motors(dm)
    # hub (fixed): arm clamps at u 75 on the diagonals, front hub, bottom plate corners
    for k, d in enumerate(MOTOR_DEG):
        c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
        fixed.add(node(f"root{k}", (75 * c, 75 * s, 22.5)))
    for sy in (1, -1):                                               # front hub: bottom plate front wing (low) and top plate front edge (high)
        fixed.add(node(f"fhubB{sy:+d}", (98, 12 * sy, p.get("z_fhub", -2.0)))); fixed.add(node(f"fhubT{sy:+d}", (58, 20 * sy, 46)))
        fixed.add(node(f"chin{sy:+d}", (xc + 62, 20 * sy, -30)))   # front of the belly sled (plate-fixed), inside the funnel
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
            t_ = round(best[2], 3)
            t_ = next((q for q in arm_pts[best[1]] if abs(q - t_) < 0.02), t_)   # on a split point already there: share it (no zero / sliver beam)
            corners.append((sx, sy, sr, best[1], t_)); arm_pts[best[1]].append(t_)
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
            mem(f"arm{k}.{i}", names[i], names[i + 1], (side, k, ts[i], ts[i + 1], bool(topo.get("arm_spars"))), "arm")
        node(f"tipb{k}", (mx, my, ROOT_H - ht)); mem(f"tiplink{k}", f"mot{k}", f"tipb{k}", None, "link")
        # arm top just inboard of the motor bell (r 22 from the motor centre, z 45): the upper spokes' socket
        ui = (r0[:2] - np.array([mx, my])); ui /= np.linalg.norm(ui)
        node(f"tipt{k}", (mx + 22 * ui[0], my + 22 * ui[1], ROOT_H)); mem(f"tiptlink{k}", f"mot{k}", f"tipt{k}", None, "link")
        mass[f"mot{k}"] = mass.get(f"mot{k}", 0) + 91e-6
        mass[f"arm{k}_0.500"] = mass.get(f"arm{k}_0.500", 0) + 25e-6          # ESC on the arm (E)
    # guard rings
    rp = ring_positions(dm, p.get("rear_x"))
    diag_on, mid_on = topo["ring_diag"], topo["mid_spokes"]
    pnodes = []
    for i, (nm, x, y, isv, ismid) in enumerate(rp):
        node(f"{nm}lo", (x, y, RING_LO)); node(f"{nm}hi", (x, y, RING_HI))
        if isv or (ismid and mid_on) or diag_on:
            mem(f"post {nm}", f"{nm}lo", f"{nm}hi", "posts"); pnodes.append(nm)       # 3.0 rod post between the two ring nodes
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
    if topo.get("spokes_hi", 0):                                             # second support height: arm top (z 45) to the same ring nodes
        for k, (mx, my) in enumerate(mot):
            near = sorted(vx, key=lambda r: math.hypot(r[1] - mx, r[2] - my))[:2]
            for r in near:
                mem(f"spokehi m{k}-{r[0]}", f"tipt{k}", f"{r[0]}lo", "spokes_hi")
    if topo["tip_rails"]:                                             # rails joining adjacent arm tips (the motor square), under the disks
        for k1, k2 in ((0, 1), (1, 2), (2, 3), (3, 0)):
            mem(f"rail m{k1}-m{k2}", f"tipb{k1}", f"tipb{k2}", "tip_rails")
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
    mass["apexM"] = 285e-6                                                 # L2 230 + both cameras 34 + plates and arms 6 + apex block ~15 (E)
    node("L2H", np.array([xB, 0, zB]) + A * L2_LEN); mem("apexlink head", "apexM", "L2H", None, "link")   # the head's tip: it leads nose-first
    for vn, sy in (("V0", 1), ("V7", -1)):
        for lvl in ("lo", "hi"):
            mid = node(f"Vmid {vn}{lvl}", (N[f"{vn}{lvl}"] + N[f"S{lvl}{sy:+d}"]) / 2)
            mem(f"V {vn}{lvl} a", f"{vn}{lvl}", mid, "V"); mem(f"V {vn}{lvl} b", mid, f"S{lvl}{sy:+d}", "V")
        if topo.get("v_diag", 0):
            mem(f"vdiag {vn}", f"{vn}hi", f"Slo{sy:+d}", "v_diag")
    for sy in (1, -1):
        if topo.get("keels", 0):
            mem(f"keel {sy:+d}", f"fhubB{sy:+d}", f"Shi{sy:+d}", "keels")   # bottom plate front up to the upper socket (16 deg)
        if topo["keels_lo"]:                                              # top plate front down to the lower socket: the keels cross (pitch, bounce)
            mem(f"keello {sy:+d}", f"fhubT{sy:+d}", f"Slo{sy:+d}", "keels_lo")
            mem(f"keeldg {sy:+d}", f"fhubB{sy:+d}", f"Slo{sy:+d}", "keels_lo")   # the diagonal: without it the two keels are a four-bar
        if topo["keels_x"]:                                               # crossed in plan: the hub holds the L2 sideways
            mem(f"keelx {sy:+d}", f"fhubB{sy:+d}", f"Slo{-sy:+d}", "keels_x")
        if topo["chin"]:                                                  # from the sled front, 30 under the plates: 23 deg of climb
            mem(f"chin {sy:+d}", f"chin{sy:+d}", f"Shi{sy:+d}", "chin")
    if topo.get("spine"):                                             # keel spine on the centre line, in the gap between the front disks
        B0 = np.array([xB, 0.0, zB])                                      # (out of the prop wash): top plate front to the upper centre socket,
        sp = {lv: B0 - A * SOCK_BACK + UP_B * (APEX_SPAN / 2 if lv == "hi" else -APEX_SPAN / 2) for lv in ("hi", "lo")}   # hub bottom to the lower
        for lv, q in sp.items():                                          # one and up to the upper one (triangulated in pitch)
            node(f"SP{lv}", q); mem(f"apexlink SP{lv}", "apexM", f"SP{lv}", None, "link")
        fixed.add(node("fhubB0", (98, 0, p.get("z_fhub", -2.0)))); fixed.add(node("fhubT0", (58, 0, 46)))
        for nm_, a_, b_ in (("spine top", "fhubT0", "SPlo"), ("spine lo", "fhubB0", "SPlo"), ("spine dg", "fhubB0", "SPhi")):   # all under the swept band
            xm_ = p.get("x_mast", dm)
            if p.get("pyramid") == "mast" and N[a_][0] < xm_ < N[b_][0]:  # crosses the mast: a shared node (the spine braces the mast)
                f_ = (xm_ - N[a_][0]) / (N[b_][0] - N[a_][0]); xn_ = node(f"MX {nm_}", N[a_] + f_ * (N[b_] - N[a_]))
                mem(f"{nm_} a", a_, xn_, "spine"); mem(f"{nm_} b", xn_, b_, "spine"); SPINE_X.append(xn_)
            else:
                mem(nm_, a_, b_, "spine")
    if topo["l2_braces"]:
        for k, (mx, my) in enumerate(mot):
            if mx > dm:
                sy = 1 if my > 0 else -1
                mem(f"brace m{k}", f"tipb{k}", f"Shi{sy:+d}", "l2_braces")      # arm tip underside up to the upper socket
                mem(f"bracelo m{k}", f"tipt{k}", f"Slo{sy:+d}", "l2_braces")    # arm tip top down to the lower socket (crossed)
                mem(f"bracedg m{k}", f"tipb{k}", f"Slo{sy:+d}", "l2_braces")    # diagonal: no four-bar
    # nose collar round the L2 base (Spark 24 Oct: the 25 base is held and caged; above it the L2 is exposed and nothing
    # may enter the scan, 96 deg negative-angle mode about the axis 15 deg below +x): 4 corner nodes in the L2 frame at
    # local up ±hu (the base and the cameras inside) and y ±ny, set back along the axis until they sit NOSE_GAP behind
    # the scan surface (a cone of 90 + L2_NEG deg from the top of the base); a rear-bottom bar behind the lugs
    Bv = np.array([xB, 0, zB]); O = Bv + A * L2_BASE_H
    ny = p.get("nose_y", NOSE_Y); hu = max(37.5 + NOSE_GAP, p.get("cam_hu", 0.0))
    gap_n = 3.0 + node_size(NOSE_OD_MAX) / 2 * math.sqrt(3)            # the corner node block (rods ≤ NOSE_OD_MAX) stays 3 outside the scan
    r_c = math.hypot(hu, ny); h_n = L2_BASE_H - r_c * math.tan(math.radians(L2_NEG)) - gap_n / math.cos(math.radians(L2_NEG))
    Yv = np.array([0.0, 1.0, 0.0])
    for sy in (1, -1):
        node(f"NT{sy:+d}", Bv + A * h_n + UP_B * hu + Yv * ny * sy); node(f"NB{sy:+d}", Bv + A * h_n - UP_B * hu + Yv * ny * sy)
        mem(f"nose vert {sy:+d}", f"NT{sy:+d}", f"NB{sy:+d}", "nose")
    for sy in (1, -1):
        node(f"NR{sy:+d}", Bv - A * (SOCK_BACK + 6.0) - UP_B * hu + Yv * ny * sy); mem(f"nose rail {sy:+d}", f"NB{sy:+d}", f"NR{sy:+d}", "nose")
        mem(f"nose rear lo {sy:+d}", f"Slo{sy:+d}", f"NR{sy:+d}", "nose")
    mem("nose rear bar", "NR+1", "NR-1", "nose")
    mem("nose top", "NT+1", "NT-1", "nose"); mem("nose bot", "NB+1", "NB-1", "nose")
    xn, ztn, zbn = float(N["NT+1"][0]), float(N["NT+1"][2]), float(N["NB+1"][2])
    for vn, sy in (("V0", 1), ("V7", -1)):
        mem(f"longeron hi {sy:+d}", f"{vn}hi", f"NT{sy:+d}", "nose"); mem(f"longeron lo {sy:+d}", f"{vn}lo", f"NB{sy:+d}", "nose")
        mem(f"nose side dg {sy:+d}", f"{vn}hi", f"NB{sy:+d}", "nose")
        mem(f"mount hi {sy:+d}", f"Shi{sy:+d}", f"NT{sy:+d}", "nose_mount"); mem(f"mount lo {sy:+d}", f"Slo{sy:+d}", f"NB{sy:+d}", "nose_mount")
        mem(f"mount dg {sy:+d}", f"NR{sy:+d}", f"NT{sy:+d}", "nose_mount")   # side panel diagonal (the socket-to-front diagonal would cross the camera)
    # top: node over the thrust centre on a 4-leg pyramid from the top plate, spider to the upper ring vertices
    # pyramid "plate": 4 legs from the top plate corners; "rear": the hub sits under the rear disks (layout A), so the
    # top node stands on a rear tower (top plate rear corners + rear arm roots to TR behind the disks) tied to TN
    pyr = p.get("pyramid", "plate")
    node("TN", (p.get("x_top", dm), 0, Z_TOP))
    if pyr == "plate":
        for sx in (1, -1):
            for sy in (1, -1):
                fixed.add(node(f"tp{sx:+d}{sy:+d}", (40 * sx, 40 * sy, ROOT_H + 1.6)))
                mem(f"pyramid {sx:+d}{sy:+d}", f"tp{sx:+d}{sy:+d}", "TN", "pyramid")
    elif pyr == "rear":
        node("TR", (p.get("x_tower", -100.0), 0, Z_TOP))
        for sy in (1, -1):
            fixed.add(node(f"tp-1{sy:+d}", (-40, 40 * sy, ROOT_H + 1.6)))
            mem(f"pyramid -1{sy:+d}", f"tp-1{sy:+d}", "TR", "pyramid")
        mem("top bar", "TR", "TN", "pyramid")                         # the tower's third leg is the skirt post below
    if p.get("rear_x") is not None:                                   # rear skirt: the rear flat drops to plate level and ties to the rear roots
        zs = p.get("z_skirt", 0.0)
        rv = [r for r in rp if r[0] in ("P3_25", "P3_50", "P3_75")]       # y ±100 and 0: covers the hub (±59); corner hits land on the ring vertices
        for r in rv:
            node(f"K{r[0]}", (r[1], r[2], zs)); mem(f"skirt post {r[0]}", f"{r[0]}lo", f"K{r[0]}", "skirt")
        mem("skirt rail +", "KP3_25", "KP3_50", "skirt"); mem("skirt rail -", "KP3_50", "KP3_75", "skirt")
        mem("skirt dg +", "P3_25lo", "KP3_50", "skirt"); mem("skirt dg -", "P3_75lo", "KP3_50", "skirt")
        for sy in (1, -1):                                             # rear hub block under the bottom plate (printed, like the front sled)
            fixed.add(node(f"rhubB{sy:+d}", (p.get("x_rhub", -75.0), 15.0 * sy, p.get("z_rhub", -2.0))))
            mem(f"skirt tie {sy:+d}", f"rhubB{sy:+d}", "KP3_50", "skirt_ties")
            mem(f"skirt side {sy:+d}", f"rhubB{sy:+d}", "KP3_25" if sy > 0 else "KP3_75", "skirt_ties")
        if "TR" in N:
            mem("tower post", "KP3_50", "TR", "pyramid")
    if topo["top_spider"]:                                           # spider with a mid ring: each leg braced at half length (buckling, 35 g)
        for r in vx:
            node(f"SM{r[0]}", (N["TN"] + N[f"{r[0]}hi"]) / 2)
            mem(f"spider {r[0]}", "TN", f"SM{r[0]}", "top"); mem(f"spider out {r[0]}", f"SM{r[0]}", f"{r[0]}hi", "top")
        for i_, r in enumerate(vx):
            mem(f"spider ring {r[0]}", f"SM{r[0]}", f"SM{vx[(i_ + 1) % len(vx)][0]}", "top")
        for sy in (1, -1):                                            # only where it stays clear of the swept band (a low nose top dives through it)
            if prop_clear(N["TN"], N[f"NT{sy:+d}"], 3.0, mot) >= GUARD_GAP:
                mem(f"spider nose{'+' if sy > 0 else '-'}", "TN", f"NT{sy:+d}", "top")
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
        if topo.get("strut2", 0):
            mem(f"strut2 {sx:+d}{sy:+d}", f"SR{sx:+d}{sy:+d}", f"hubc{sx:+d}{sy:+d}", "strut2")
    for sx in (1, -1):
        mem(f"wring x{sx:+d}", f"WAi{sx:+d}+1", f"WAi{sx:+d}-1", "wheel_ring")
    for sy in (1, -1):
        mem(f"wring y{sy:+d}", f"WBi+1{sy:+d}", f"WBi-1{sy:+d}", "wheel_ring")
    if p.get("feet") is not None:                                     # belly frame + skid (layout A: the catch sits under the motor square)
        fx, fy, fz = p["feet"]; bx, by = p.get("belly", (fx, fy))
        for sx in (1, -1):
            for sy in (1, -1):
                # printed belly frame (the sled under the receiver mount plane) on each corner block at wheel height: it
                # carries the receiver, the swap set and the pods (a quarter each); skid leg inside the opening to the foot
                node(f"BF{sx:+d}{sy:+d}", (xc + bx * sx, by * sy, wz)); mem(f"belly tab {sx:+d}{sy:+d}", f"corner{sx:+d}{sy:+d}", f"BF{sx:+d}{sy:+d}", None, "link")
                mass[f"BF{sx:+d}{sy:+d}"] = p.get("swap_g", 720.0) / 4 * 1e-6
                node(f"FT{sx:+d}{sy:+d}", (xc + fx * sx, fy * sy, fz))
                mem(f"skid leg {sx:+d}{sy:+d}", f"BF{sx:+d}{sy:+d}", f"FT{sx:+d}{sy:+d}", "skid_belly")
        for sy in (1, -1):                                            # side bar + rail close each side; no rail across the ends (the pack drops out the bottom)
            mem(f"belly bar {sy:+d}", f"BF+1{sy:+d}", f"BF-1{sy:+d}", "skid_belly")
            mem(f"skid rail {sy:+d}", f"FT+1{sy:+d}", f"FT-1{sy:+d}", "skid_belly")
        if p.get("pyramid") == "mast":                                # top node on a vertical mast at the thrust centre (70 off every
            node("MB", (p.get("x_mast", dm), 0, p.get("z_mast", wz)))   # disk, out of the prop wash) down to the sled (no tail, Spark 24 Oct)
            for sx in (1, -1):
                for sy in (1, -1):
                    mem(f"sled link {sx:+d}{sy:+d}", "MB", f"BF{sx:+d}{sy:+d}", None, "link")
            chain = ["MB"] + sorted(SPINE_X, key=lambda n_: N[n_][2]) + ["TN"]   # through the spine crossings
            for i_ in range(len(chain) - 1):                          # a link under the carbon break-even is printed into the mast node block
                short_ = np.linalg.norm(N[chain[i_ + 1]] - N[chain[i_]]) < CARBON_MIN
                mem(f"mast {i_}", chain[i_], chain[i_ + 1], "pyramid", "link" if short_ else "rod")
    join_crossings(N, M)
    rod_len = {}                                                      # carbon break-even (CLAUDE.md) per physical rod: a rod runs on through its
    for nm_, a_, b_, grp_, kd_ in M:                                  # cross nodes (the split pieces x1 / x2, j1 / j2, the spine a / b at the mast
        if kd_ == "rod": rod_len[carbon_rod(nm_)] = rod_len.get(carbon_rod(nm_), 0.0) + float(np.linalg.norm(N[b_] - N[a_]))   # block); one under 33
    for i_, (nm_, a_, b_, grp_, kd_) in enumerate(M):                 # between printed parts is printed with them (a strut), never carbon
        if kd_ == "rod" and rod_len[carbon_rod(nm_)] < CARBON_MIN:
            M[i_] = (nm_, a_, b_, grp_, "link")
    info = dict(ring=rp, pnodes=pnodes, motors=mot, corners=corners, sockets=S, nose=(xn, ztn, zbn))
    return N, M, fixed, mass, info


def carbon_rod(name):
    """the physical carbon rod a member belongs to: its name less the cross-node split tags (x1 / x2 / j1 / j2, repeated) and the
    spine's a / b at the mast block"""
    return re.sub(r"( [xj][12])+$", "", re.sub(r"^(spine (?:top|lo|dg)) [ab]", r"\1", name))


JOIN_D = 14.0                                                   # rods (or a rod and a node) closer than this would clash: one printed cross node


def _seg_closest(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f, c, b = d1 @ d1, d2 @ d2, d2 @ r, d1 @ r, d1 @ d2
    den = a * e - b * b
    s = float(np.clip((b * f - c * e) / den, 0, 1)) if den > 1e-9 else 0.0
    t = float(np.clip((b * s + f) / e, 0, 1)); s = float(np.clip((b * t - c) / a, 0, 1))
    return float(np.linalg.norm(p1 + s * d1 - (p2 + t * d2))), s, t


def join_crossings(N, M):
    """a node lying on a rod's run splits the rod there; two rods crossing within JOIN_D share a new printed node "XJ<k>"
    at the midpoint of their closest points (each rod kinks through it); repeated until no clash is left"""
    def split(i, nn, tag):
        name, a, b, group, kind = M[i]
        M[i] = (f"{name} {tag}1", a, nn, group, kind); M.append((f"{name} {tag}2", nn, b, group, kind))
    k_ = 0
    for _ in range(60):
        changed = False
        rods = [i for i, m_ in enumerate(M) if m_[4] == "rod"]
        used = {n_ for m_ in M if m_[4] in ("rod", "arm") for n_ in (m_[1], m_[2])}
        for i in rods:                                                    # a node on a rod
            name, a, b, group, kind = M[i]; pa, pb = N[a], N[b]; u = pb - pa; L2_ = u @ u
            names_ = [n_ for n_ in used if n_ not in (a, b)]
            Q = np.array([N[n_] for n_ in names_]); t = np.clip((Q - pa) @ u / L2_, 0, 1)
            dl = np.linalg.norm(pa + t[:, None] * u - Q, axis=1)
            ok_ = (dl < JOIN_D / 2) & (np.linalg.norm(Q - pa, axis=1) > JOIN_D) & (np.linalg.norm(Q - pb, axis=1) > JOIN_D)
            if ok_.any():
                split(i, names_[int(np.argmin(np.where(ok_, dl, np.inf)))], "j"); changed = True
            if changed: break
        if changed: continue
        if len(rods) > 1:                                                 # two rods crossing (all pairs at once)
            A_ = np.array([N[M[i][1]] for i in rods]); B_ = np.array([N[M[i][2]] for i in rods])
            ii, jj = np.triu_indices(len(rods), 1)
            ends = np.array([[M[i][1], M[i][2]] for i in rods])
            share = (ends[ii, 0] == ends[jj, 0]) | (ends[ii, 0] == ends[jj, 1]) | (ends[ii, 1] == ends[jj, 0]) | (ends[ii, 1] == ends[jj, 1])
            p1, d1, p2, d2 = A_[ii], B_[ii] - A_[ii], A_[jj], B_[jj] - A_[jj]; r_ = p1 - p2
            a_, e_, f_ = (d1 * d1).sum(1), (d2 * d2).sum(1), (d2 * r_).sum(1); c_, b_ = (d1 * r_).sum(1), (d1 * d2).sum(1)
            den = a_ * e_ - b_ * b_
            s_ = np.where(den > 1e-9, np.clip((b_ * f_ - c_ * e_) / np.where(den > 1e-9, den, 1), 0, 1), 0.0)
            t_ = np.clip((b_ * s_ + f_) / e_, 0, 1); s_ = np.clip((b_ * t_ - c_) / a_, 0, 1)
            P1 = p1 + s_[:, None] * d1; P2 = p2 + t_[:, None] * d2; dd = np.linalg.norm(P1 - P2, axis=1)
            Li, Lj = np.sqrt(a_), np.sqrt(e_)
            hit = (~share) & (dd < JOIN_D) & (s_ * Li > JOIN_D) & (s_ * Li < Li - JOIN_D) & (t_ * Lj > JOIN_D) & (t_ * Lj < Lj - JOIN_D)
            if hit.any():
                q = int(np.flatnonzero(hit)[0]); i, j = rods[ii[q]], rods[jj[q]]
                nn = f"XJ{k_}"; k_ += 1; N[nn] = (P1[q] + P2[q]) / 2
                split(i, nn, "x"); split(j, nn, "x"); changed = True
        if not changed: break


# ---------------------------------------------------------------- beam FE
def arm_section(side, t0, t1, spars=False):
    """PETG box arm; spars: 4 x 3.0 carbon rods epoxied in the box corners (centres 3.5 in from the outer faces),
    taken into the section as PETG-equivalent area and inertia (E_C / E_P), torsion from the PETG box only"""
    t = (t0 + t1) / 2; ht = H_TIP[side]; b0, b1 = W_ARM[side]
    h = ROOT_H + (ht - ROOT_H) * t; b = b0 + (b1 - b0) * t; w = ARM_WALL
    Iy = (b * h ** 3 - (b - 2 * w) * (h - 2 * w) ** 3) / 12; Iz = (h * b ** 3 - (h - 2 * w) * (b - 2 * w) ** 3) / 12
    Am = (b - w) * (h - w); J = 4 * Am ** 2 * w / (2 * (b - w) + 2 * (h - w)); A_ = b * h - (b - 2 * w) * (h - 2 * w)
    rho = 1.27e-9
    if spars:
        n = E_C / E_P; r = SECTIONS["r3"]; dz, dy = h / 2 - 3.5, b / 2 - 3.5
        Iy += n * 4 * (r["I"] + r["A"] * dz ** 2); Iz += n * 4 * (r["I"] + r["A"] * dy ** 2); A_ += n * 4 * r["A"]
        rho = (1.27e-9 * (b * h - (b - 2 * w) * (h - 2 * w)) + 4 * r["gpm"] * 1e-9) / A_
    return dict(A=A_, Iy=Iy, Iz=Iz, J=J, E=E_P, G=G_P, c=max(b, h) / 2, rho=rho, h=h, b=b, spars=spars)


def elem_props(kind, group, sizes, name=None):
    if kind == "arm":
        side = group[0]; return arm_section(side, group[2], group[3], spars=group[4])
    if kind == "link":
        return dict(A=2000.0, Iy=4e5, Iz=4e5, J=8e5, E=E_P * 20, G=G_P * 20, c=20.0, rho=0.0)
    if kind == "pnode":
        return dict(A=100.0, Iy=833.0, Iz=833.0, J=1400.0, E=E_P, G=G_P, c=5.0, rho=0.0)
    s = SECTIONS[sizes.get(name) or sizes.get(group) or "r3"]                   # per rod (fully stressed design) or per group
    return dict(A=s["A"], Iy=s["I"], Iz=s["I"], J=s["J"], E=E_C, G=G_C, c=s["od"] / 2, rho=s["gpm"] * 1e-9 / s["A"], od=s["od"], sec=sizes.get(name) or sizes.get(group) or "r3")   # t/mm3


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
            P = elem_props(kind, group, sizes, name); pa, pb = self.N[a], self.N[b]; L = float(np.linalg.norm(pb - pa))
            T = rot(pa, pb); kl = k_local(L, P); kg = T.T @ kl @ T
            dofs = np.r_[6 * self.idx[a]:6 * self.idx[a] + 6, 6 * self.idx[b]:6 * self.idx[b] + 6]
            K[np.ix_(dofs, dofs)] += kg
            m = P["rho"] * P["A"] * L if kind in ("rod", "arm") else 0.0
            for n_ in (a, b):
                i = 6 * self.idx[n_]; Mm[i:i + 3] += m / 2; Mm[i + 3:i + 6] += m / 2 * (L ** 2) / 24 + 1e-12
            self.el.append(dict(name=name, a=a, b=b, group=group, kind=kind, P=P, L=L, T=T, kl=kl, dofs=dofs))
        od_max, ods_at = {}, {}
        for e in self.el:
            if e["kind"] == "rod":
                for n_ in (e["a"], e["b"]): od_max[n_] = max(od_max.get(n_, 0.0), e["P"]["od"]); ods_at.setdefault(n_, []).append(e["P"]["od"])
        self.node_g = {n_: node_mass_g(ods_at.get(n_, [3.0])) for n_ in self.names
                       if n_ not in self.fixed and (n_[:-2] in self.info["pnodes"] or n_.startswith(PRINTED_NODES))}
        self.node_size = {n_: node_size(od_max.get(n_, 3.0)) for n_ in self.node_g}
        for n_, gm in self.node_g.items():                                                # printed nodes at their drawn size (E)
            i = 6 * self.idx[n_]; Mm[i:i + 3] += gm * 1e-6; Mm[i + 3:i + 6] += gm * 1e-6 * self.node_size[n_] ** 2 / 6
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
        """lowest modes from the largest eigenvalues of M^1/2 K^-1 M^1/2 (well conditioned with the near-massless
        rotations); frac = share of each mode's kinetic energy on the L2 group node"""
        Mf = self.Mm[self.free]; mh = np.sqrt(Mf)
        Kinv = sla.cho_solve(self.cho, np.eye(len(self.free)))
        Asym = (mh[:, None] * Kinv) * mh[None, :]; Asym = (Asym + Asym.T) / 2
        nn = len(self.free); lam, Y = sla.eigh(Asym, subset_by_index=[nn - n, nn - 1])
        lam, Y = lam[::-1], Y[:, ::-1]
        f = 1.0 / np.sqrt(np.maximum(lam, 1e-30)) / (2 * math.pi)
        ia = [list(self.free).index(6 * self.idx["apexM"] + j) for j in range(6)]
        frac = np.array([float((Y[ia, m] ** 2).sum()) for m in range(Y.shape[1])])
        self.mode_f, self.mode_Y = f, Y
        return f, frac

    def mode_share(self, names):
        """share of each mode's kinetic energy on the given nodes (translations)"""
        fr = list(self.free); ib = [fr.index(6 * self.idx[n] + j) for n in names for j in range(3)]
        return np.array([float((self.mode_Y[ib, m] ** 2).sum()) for m in range(self.mode_Y.shape[1])])

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
            if e["kind"] == "rod": g += SECTIONS[e["P"]["sec"]]["gpm"] * e["L"] / 1000.0
        g += sum(self.node_g.values()) + CORNER_G * 4                                     # printed nodes at their drawn size, corner blocks
        if self.topo.get("arm_spars"):
            g += sum(4 * SECTIONS["r3"]["gpm"] * e["L"] / 1000.0 for e in self.el if e["kind"] == "arm")
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
        if n_ in model.fixed or deg.get(n_, 0) != 2 or len(same) != 2: return True   # a hub (fixed) node always ends a run
        u_ = [np.subtract(model.N[model.el[i]["b"] if model.el[i]["a"] == n_ else model.el[i]["a"]], model.N[n_]) for i in same]
        return float(-u_[0] @ u_[1]) < math.cos(math.radians(10.0)) * np.linalg.norm(u_[0]) * np.linalg.norm(u_[1])   # a corner (> 10 deg kink) ends a run
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


G_ACC = 9810.0                                                # mm/s2
CRASH_G = 35.0                                                # dimensions L Crash (DEFAULT, E): the rigid case
NODE_FUSE = 1.5                                               # breaking order (CLAUDE.md design 7, L): cage nodes are drawn as fuses that let go at
                                                              # <= 1 / 1.5 of their rod's strength, so rods and buckling size to 1 / 1.5 in the crash cases
CRASH_MODE = os.environ.get("CAGE_CRASH_MODE", "rigid")       # rigid 35 g (L) until a suspension is drawn (review 24 Oct); "energy": as the Scout, a
V_HIT, STROKE, PLATEAU_MIN = 2.0, 10.0, 200.0                 # 2.0 m/s hit through a preloaded TPU suspension, 10 stroke, plateau ≥ 200 N


PLAN_GROUPS = ("chords", "V", "ring_diag", "posts", "skirt")       # guard rods checked in plan (12 off the disks), the rest under / over the band


def prop_clear(a, b, od, motors):
    """smallest prop clearance (mm) of a rod: within 12 (plan) of a swept disk it must sit 12 under or over the band"""
    t = np.linspace(0, 1, 120)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))
    c = np.inf
    for mx, my in motors:
        u = np.hypot(q[:, 0] - mx, q[:, 1] - my) < PROP_R + od / 2 + GUARD_GAP
        if u.any():
            zq = q[u, 2]; c = min(c, float(np.maximum(PROP_Z - BLADE - (zq + od / 2), zq - od / 2 - PROP_Z - BLADE).min()))
    return c


OPEN_HALF, FACE_TOP = 84.0, 75.1                                   # box collar funnel (dimensions G)


FACE_C, X_EDGE_Z, Y_EDGE_Z, OPEN_X = 82.7, 12.8, 1.3, 95.5           # funnel v9 plain / v7 (dimensions G): half = 82.7 + seat z


def funnel_clear(a, b, od, p):
    """smallest clearance (mm) of a rod to the seated funnel (catch centre on the funnel axis, p["seat_dz"] = bottom
    plate top in the seat frame): face planes half = 82.7 + seat z, the ±x faces from z 12.8, the ±y faces from z 1.3,
    to the top 75.1; below the edges the rod stays inside 191 (x) x 168 (y) at ±5 deg yaw, 3 a side (dimensions G (2))"""
    if p.get("seat_dz") is None: return np.inf
    t = np.linspace(0, 1, 80)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))
    q = q + np.array([-p["xc"], 0.0, p["seat_dz"]]); z = q[:, 2]; c = np.inf
    for ax, z0 in ((0, X_EDGE_Z), (1, Y_EDGE_Z)):
        for sg in (1, -1):
            x = sg * q[:, ax]; inz = (z > z0) & (z < FACE_TOP) & (np.abs(q[:, 1 - ax]) < FACE_C + z)
            if inz.any(): c = min(c, float(((z[inz] + FACE_C - x[inz]) / math.sqrt(2) - od / 2).min()))
    for yaw in np.radians((-5.0, 0.0, 5.0)):
        cy, sy_ = math.cos(yaw), math.sin(yaw)
        rx = np.abs(q[:, 0] * cy - q[:, 1] * sy_); ry = np.abs(q[:, 0] * sy_ + q[:, 1] * cy)
        lx, ly = z <= X_EDGE_Z, z <= Y_EDGE_Z
        if lx.any(): c = min(c, float((OPEN_X - 3.0 - rx[lx] - od / 2).min()) + 1.0)   # +1: the 1.0 clearance target is the 3 a side here
        if ly.any(): c = min(c, float((OPEN_HALF - 3.0 - ry[ly] - od / 2).min()) + 1.0)
    return c


MOTOR_R, MOTOR_Z0, MOTOR_ZH = 13.85, ROOT_H, ROOT_H + 26.0           # X2216 bell on the arm pad (E, as the system model)


def motor_clear(a, b, od, motors):
    """smallest clearance (mm) of a rod to the motor bells"""
    t = np.linspace(0, 1, 80)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))
    inz = (q[:, 2] > MOTOR_Z0 - od / 2) & (q[:, 2] < MOTOR_ZH + od / 2); c = np.inf
    if inz.any():
        for mx, my in motors:
            c = min(c, float((np.hypot(q[inz, 0] - mx, q[inz, 1] - my) - MOTOR_R - od / 2).min()))
    return c


def obstacle_clear(a, b, od, obstacles):
    """smallest clearance (mm) of a rod to the obstacle boxes {c, fwd, left, half} (the front cameras in their own frame)
    and point sets {pts} (a curved surface's vertices, the L2 dome)"""
    if not obstacles: return np.inf
    t = np.linspace(0, 1, 100)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float)); c = np.inf
    for o in obstacles:
        if "field" in o:                                              # the L2 scan cone: clearance behind its surface
            O_, A_, ng_ = np.asarray(o["field"][0], float), np.asarray(o["field"][1], float), math.radians(o["field"][2])
            d_ = q - O_; mg_ = -(d_ @ A_) * math.cos(ng_) - np.sqrt(np.maximum((d_ ** 2).sum(1) - (d_ @ A_) ** 2, 0)) * math.sin(ng_)
            c = min(c, float(mg_.min() - od / 2)); continue
        if "pts" in o:
            P_ = np.asarray(o["pts"], float); ab = np.asarray(b, float) - np.asarray(a, float)
            s_ = np.clip((P_ - a) @ ab / max(ab @ ab, 1e-9), 0, 1)
            c = min(c, float(np.linalg.norm(P_ - (np.asarray(a, float) + s_[:, None] * ab), axis=1).min() - od / 2)); continue
        f, l = np.asarray(o["fwd"], float), np.asarray(o["left"], float); u = np.cross(f, l)
        d = q - np.asarray(o["c"], float); loc = np.stack([d @ f, d @ l, d @ u], 1)
        out = np.maximum(np.abs(loc) - np.asarray(o["half"], float), 0)
        c = min(c, float(np.linalg.norm(out, axis=1).min() - od / 2))
    return c


WASH_OD, WASH_LEN = 10.0, 15.0                                    # thin rods in the prop wash (CLAUDE.md design 8, DEFAULT 25 Oct: ≤ 10/8 tube where a rod runs ≥ 15 under / over a disk, alt 8/6 with more bracing: 0 of 256 layouts carry 35 g at 8);
WASH_SHARE = None                                                # each disk's shadowed share reported (a 5 % gate is not reachable with the L2 mount under the front disks)


def wash_share(m):
    """per motor: share of its swept disk shadowed in plan by the cage rods (od x length inside the disk / disk area)"""
    out = []
    for mx, my in m.info["motors"]:
        a_ = 0.0
        for e in m.el:
            if e["kind"] != "rod": continue
            A, B = np.asarray(m.N[e["a"]], float), np.asarray(m.N[e["b"]], float)
            t = np.linspace(0, 1, 60)[:, None]; q = A + t * (B - A)
            ins = np.hypot(q[:, 0] - mx, q[:, 1] - my) < PROP_R
            if ins.any(): a_ += ins.mean() * float(np.hypot(*(B - A)[:2])) * SECTIONS[e["P"]["sec"]]["od"]
        out.append(a_ / (math.pi * PROP_R ** 2))
    return out


def wash_len(a, b, motors):
    """length (mm) of a rod inside the swept disks in plan (in the prop wash, above or below)"""
    t = np.linspace(0, 1, 100)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))
    inside = np.zeros(len(q), bool)
    for mx, my in motors: inside |= np.hypot(q[:, 0] - mx, q[:, 1] - my) < PROP_R
    return float(inside.mean() * np.linalg.norm(np.subtract(b, a)))


def obstacles_for(m, e):
    """the obstacles a rod must clear: all, except a box the rod is attached to: skip_fixed (a rod ending in a hub
    node) and skip_nodes (name prefixes of the nodes on that body: arm roots on the plates, the L2 apex sockets)"""
    ends_fixed = e["a"] in m.fixed or e["b"] in m.fixed
    def attached(o):
        return (o.get("skip_fixed") and ends_fixed) or any(n.startswith(tuple(o.get("skip_nodes", ()))) for n in (e["a"], e["b"]) if o.get("skip_nodes"))
    return [o for o in (m.p.get("obstacles") or []) if not attached(o)]


def guard_caps(m):
    """largest OD per rod that keeps the prop clearance (12), the funnel clearance (1.0), the motors, the obstacles and the
    prop wash cap (every check grows with OD only); -1 = no section does (the layout fails)"""
    caps = {}
    ods = sorted({S["od"] for S in SECTIONS.values()})
    for e in m.el:
        if e["kind"] != "rod": continue
        cap = -1.0
        for od in ods:
            if e["group"] not in PLAN_GROUPS and prop_clear(m.N[e["a"]], m.N[e["b"]], od, m.info["motors"]) < GUARD_GAP: break
            if funnel_clear(m.N[e["a"]], m.N[e["b"]], od, m.p) < 1.0: break
            if motor_clear(m.N[e["a"]], m.N[e["b"]], od, m.info["motors"]) < 1.0: break
            if obstacle_clear(m.N[e["a"]], m.N[e["b"]], od, obstacles_for(m, e)) < 1.0: break
            if od > WASH_OD and wash_len(m.N[e["a"]], m.N[e["b"]], m.info["motors"]) >= WASH_LEN: break
            cap = od
        caps[e["name"]] = cap
    return caps


MODE_SHARE = 0.10                                                  # a mode is the L2's (or the belly's) from 10 % of its kinetic energy there


def mode_energy_runs(m, k, runs):
    """share of mode k's strain energy in each run (the rods to stiffen for that mode)"""
    U = np.zeros(m.K.shape[0]); mh = np.sqrt(m.Mm[m.free]); U[m.free] = m.mode_Y[:, k] / mh
    se = np.zeros(len(m.el))
    for ie, e in enumerate(m.el):
        ue = e["T"] @ U[e["dofs"]]; se[ie] = 0.5 * float(ue @ (e["kl"] @ ue))
    tot = se.sum() or 1.0
    return np.array([se[run].sum() / tot for run in runs])


def crash_force(m_t):
    """touching-node crash force (N) for a drone of m_t (t): rigid CRASH_G, or the suspension plateau that stops a V_HIT
    hit in STROKE (½ m v² / s, never under PLATEAU_MIN, above the 175 N latch misuse)"""
    if CRASH_MODE == "rigid": return CRASH_G * G_ACC * m_t
    return max(0.5 * (m_t * 1e3) * V_HIT ** 2 / (STROKE * 1e-3), PLATEAU_MIN)


def crash_dirs():
    d = [np.array(v, float) for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    d += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    return d


def _indices(m, mr, runs):
    """worst rod stress index, socket index, buckling index, arm index over the elements"""
    rod = sock = arm = 0.0
    for (N_, sg), e in zip(mr, m.el):
        if e["kind"] == "rod":
            rod = max(rod, sg / SIG_ALLOW); sock = max(sock, abs(N_) / (SOCKET_CAP * e["P"]["od"]))
        elif e["kind"] == "arm":
            arm = max(arm, sg / 51.0)
    buck = 0.0
    for run in runs:
        e0 = m.el[run[0]]; P = e0["P"]; L = sum(m.el[i]["L"] for i in run)
        Ncomp = max(-mr[i][0] for i in run)
        if Ncomp > 0: buck = max(buck, BUCKLE_SF * Ncomp / (math.pi ** 2 * P["E"] * P["Iy"] / L ** 2))
    return dict(rod=rod, socket=sock, buckle=buck, arm=arm)


def evaluate(p, topo, sizes, detail=False):
    m = Model(p, topo, sizes)
    res = dict(mass_g=m.rod_mass_g(), cases={})
    g = {}
    f, frac = m.modes()
    l2 = [i for i, fr in enumerate(frac) if fr >= MODE_SHARE]
    f_l2 = float(f[l2[0]]) if l2 else float(f[-1]); res["_l2_mode"] = l2[0] if l2 else len(f) - 1
    res["f_first"], res["f_l2"] = float(f[0]), f_l2
    fr_idx = {d_: k_ for k_, d_ in enumerate(m.free)}; y0 = m.mode_Y[:, 0] ** 2       # where the first mode moves: kinetic-energy share per node
    sh = {n_: sum(y0[fr_idx[6 * m.idx[n_] + j]] for j in range(3) if 6 * m.idx[n_] + j in fr_idx) for n_ in m.names}
    tot_ = sum(sh.values()) or 1.0
    res["first_mode_where"] = [(n_, round(float(v_ / tot_), 2)) for n_, v_ in sorted(sh.items(), key=lambda kv: -kv[1])[:4]]
    g["L2 mode"] = F_L2_MIN / f_l2 - 1
    bf = [n for n in m.names if n.startswith("BF")]
    if bf:                                                            # the down camera and flow ride the belly frame: a sensor mount too
        sh = m.mode_share(bf); bm = [i for i, s_ in enumerate(sh) if s_ >= MODE_SHARE]
        res["f_belly"] = float(f[bm[0]]) if bm else float(f[-1]); res["_belly_mode"] = bm[0] if bm else len(f) - 1
        g["belly mode"] = F_L2_MIN / res["f_belly"] - 1
    m_drone = p.get("m_drone_g", 2200.0) * 1e-6                       # t (g -> t)
    cage_nodes = [n for n in m.names if n not in m.fixed and (n.endswith(("lo", "hi")) and not n.startswith(("S", "Vmid"))
                                                              or n.startswith(("NT", "NB", "NR", "TN", "TR", "corner", "KV", "KP", "FT")))]
    cage_nodes = cage_nodes + ["L2H"]                                # nose-first the exposed head leads: the hit goes in through the L2 and its mount
    P_ = np.array([m.N[n] for n in cage_nodes])
    cases = []                                                       # (name, loads, limit, kind, hit node, direction)
    for d in crash_dirs():                                           # every node within CRASH_TIE of the leading support may touch first
        sup = P_ @ d
        for k in np.flatnonzero(sup >= sup.max() - CRASH_TIE):
            n_ = cage_nodes[k]
            cases.append((f"crash {np.round(d, 2).tolist()} at {n_}", {n_: -crash_force(m_drone) * d}, 1.0, "crash", n_, d))
    for d in crash_dirs():
        cases.append((f"L2 crash {np.round(d, 2).tolist()}", {"apexM": crash_force(m_drone) / m_drone * 266e-6 * d}, 1.0, "l2", None, d))
    real_nodes = [n for n in cage_nodes if n[:-2] in m.info["pnodes"] or n.startswith(("NT", "NB", "NR", "TN", "TR", "corner", "KV", "KP", "FT"))]
    for n_ in real_nodes:
        for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            cases.append((f"handling {n_} {d}", {n_: 50.0 * np.array(d, float)}, 1.0, "handling", None, None))   # misuse-type (DEFAULT)
    wheels = [f"W{a}{sx:+d}{sy:+d}" for a in ("A", "B") for sx in (1, -1) for sy in (1, -1)]
    cases.append(("catch landing 6 g", {w: np.array([0, 0, 6.0 * G_ACC * m_drone / 8]) for w in wheels}, 0.5, "landing", None, None))
    feet = [n for n in m.names if n.startswith("FT")]
    bfs = [n for n in m.names if n.startswith("BF")]
    if bfs:                                                           # receiver on the sled (latch PR 69, E): stall 74 N up, 43.5 N across,
        bx_ = max(abs(m.N[n][0] - p["xc"]) for n in bfs)             # pitch couple 6.1 N m (misuse)
        ld = {n: np.array([0.0, 43.5 / 4, 74.0 / 4 + np.sign(m.N[n][0] - p["xc"]) * 6100.0 / (2 * bx_) / 2]) for n in bfs}
        cases.append(("receiver misuse", ld, 1.0, "swap", None, None))
    if feet:                                                          # floor landing on the 4 feet (dimensions G (5): 6 g)
        cases.append(("floor landing 6 g", {f_: np.array([0, 0, 6.0 * G_ACC * m_drone / len(feet)]) for f_ in feet}, 0.5, "landing", None, None))
    for sx in (1, -1):
        for sy in (1, -1):
            cases.append((f"latch WB{sx:+d}{sy:+d}", {f"WB{sx:+d}{sy:+d}": 175.0 * np.array([0, -sy, -1]) / math.sqrt(2)}, 1.0, "latch", None, None))
    cases.append(("swap pull", {"WA+1+1": 50.0 * np.array([1, 0, -1]) / math.sqrt(2), "WA+1-1": 50.0 * np.array([1, 0, -1]) / math.sqrt(2)}, 0.5, "swap", None, None))
    U = m.solve([c[1] for c in cases])
    runs = structural_members(m)
    # member forces for every case at once: axial N (tension +), bending stress
    nc = U.shape[1]; Nax = np.zeros((len(m.el), nc)); Sg = np.zeros((len(m.el), nc))
    for ie, e in enumerate(m.el):
        fl = e["kl"] @ (e["T"] @ U[e["dofs"]]); P = e["P"]
        Nax[ie] = -fl[0]; My = np.maximum(abs(fl[4]), abs(fl[10])); Mz = np.maximum(abs(fl[5]), abs(fl[11]))
        if e["kind"] == "arm": Sg[ie] = abs(Nax[ie]) / P["A"] + My * (P["h"] / 2) / P["Iy"] + Mz * (P["b"] / 2) / P["Iz"]
        else: Sg[ie] = abs(Nax[ie]) / P["A"] + np.hypot(My, Mz) * P["c"] / max(P["Iy"], 1e-9)
    is_rod = np.array([e["kind"] == "rod" for e in m.el]); is_arm = np.array([e["kind"] == "arm" for e in m.el])
    cap = np.array([SOCKET_CAP * e["P"].get("od", 3.0) for e in m.el])
    rod_ix = (Sg[is_rod] / SIG_ALLOW).max(axis=0); sock_ix = (abs(Nax[is_rod]) / cap[is_rod, None]).max(axis=0)
    arm_ix = (Sg[is_arm] / 51.0).max(axis=0)
    buck_ix = np.zeros(nc)
    for run in runs:
        P = m.el[run[0]]["P"]; L = sum(m.el[i]["L"] for i in run); Pcr = math.pi ** 2 * P["E"] * P["Iy"] / L ** 2
        buck_ix = np.maximum(buck_ix, BUCKLE_SF * np.maximum(-Nax[run], 0).max(axis=0) / Pcr)
    crash_node = crash_member = crash_rod = 0.0
    # breaking order (L): in the crash cases the frame (arms) stays at <= 1 / 1.5 of the worst cage node socket index, and
    # every held joint (hub, L2 apex and belly-frame sockets) is rated so its index stays there too: the socket strength
    # written to those pieces as a requirement (N, F)
    held = np.array([e["kind"] == "rod" and any(n in m.fixed or n.startswith(HELD_NODES) for n in (e["a"], e["b"])) for e in m.el])
    is_held_node = lambda n: n in m.fixed or n.startswith(HELD_NODES)
    cage_rod = np.array([e["kind"] == "rod" and not (is_held_node(e["a"]) and is_held_node(e["b"])) for e in m.el])   # a cage node at one end at least
    node_ix = (abs(Nax[cage_rod]) / cap[cage_rod, None]).max(axis=0) if cage_rod.any() else np.zeros(nc)
    held_req = {g_: 0.0 for g_ in ("hub", "apex", "belly")}
    for j, (cname, ld, lim, kind, hn, d) in enumerate(cases):
        u = U[:, j]
        ix = dict(rod=float(rod_ix[j]), socket=float(sock_ix[j]), buckle=float(buck_ix[j]), arm=float(arm_ix[j]))
        mlim = lim / NODE_FUSE if kind == "crash" else lim               # crash: members at <= 1 / 1.5, the node is the fuse
        r = dict(ix, limit=lim, mlimit=mlim)
        if kind == "crash":
            r["defl"] = float(-(u[6 * m.idx[hn]:6 * m.idx[hn] + 3] @ d))
            g[f"{cname} defl"] = r["defl"] / 9.0 - 1
            crash_node = max(crash_node, float(node_ix[j])); crash_rod = max(crash_rod, ix["rod"], ix["buckle"])
            crash_member = max(crash_member, ix["arm"])
            for i in np.flatnonzero(held):
                e_ = m.el[i]; ends = (e_["a"], e_["b"])
                g_ = "hub" if any(n in m.fixed for n in ends) else "belly" if any(n.startswith("BF") for n in ends) else "apex"
                held_req[g_] = max(held_req[g_], abs(float(Nax[i, j])))
        res["cases"][cname] = r
        for key in ("rod", "socket", "buckle", "arm"):
            g[f"{cname} {key}"] = ix[key] / (lim if key == "socket" else mlim) - 1
    deep = {}                                                        # sockets that need more than SOCKET_D: depth per rod (requirement)
    need = (abs(Nax[is_rod]) / (cap[is_rod, None] * SOCKET_D / SOCKET_D_MAX)).max(axis=1)
    for k_, (i_, v_) in enumerate(zip(np.flatnonzero(is_rod), need)):
        if v_ > 1.0: deep[m.el[i_]["name"]] = round(float(SOCKET_D * v_), 1)
    res["deep_sockets"] = deep
    res["breaking_order"] = dict(node=crash_node, arm_vs_node=crash_member / max(crash_node, 1e-9), rod=crash_rod,
                                 node_first=max(crash_member, crash_rod) * NODE_FUSE <= 1.0 + 1e-9 and crash_node <= 1.0 + 1e-9,   # every case
                                 fuse_N=(crash_force(m_drone), NODE_FUSE * crash_force(m_drone)),   # the touching node: holds the hit, breaks under 1.5 x
                                 joint_req_N={k: 1.5 * v / max(crash_node, 1e-9) for k, v in held_req.items()})   # held socket at <= 1 / 1.5 of the worst node index
    if detail: res["_U"], res["_names"] = U, [c[0] for c in cases]
    worst_pc = np.inf                                                # prop guard (C): every rod under / over the swept band by 12
    for e in m.el:
        if e["kind"] == "rod" and e["group"] not in PLAN_GROUPS:
            worst_pc = min(worst_pc, prop_clear(m.N[e["a"]], m.N[e["b"]], SECTIONS[e["P"]["sec"]]["od"], m.info["motors"]))
    res["prop_clear"] = float(worst_pc)
    g["prop guard"] = (GUARD_GAP - worst_pc) / GUARD_GAP if np.isfinite(worst_pc) else -1.0
    if p.get("seat_dz") is not None:                                 # funnel (G): every rod 1.0 off the faces, inside the opening below it
        wf = min(funnel_clear(m.N[e["a"]], m.N[e["b"]], SECTIONS[e["P"]["sec"]]["od"], p) for e in m.el if e["kind"] == "rod")
        res["funnel_clear"] = float(wf); g["funnel"] = 1.0 - wf if np.isfinite(wf) else -1.0
    wm = min(motor_clear(m.N[e["a"]], m.N[e["b"]], SECTIONS[e["P"]["sec"]]["od"], m.info["motors"]) for e in m.el if e["kind"] == "rod")
    res["motor_clear"] = float(wm); g["motor clear"] = 1.0 - wm if np.isfinite(wm) else -1.0
    ws = wash_share(m); res["wash_share"] = [float(v) for v in ws]
    if WASH_SHARE: g["prop wash"] = max(ws) / WASH_SHARE - 1
    if p.get("obstacles"):
        wo = min(obstacle_clear(m.N[e["a"]], m.N[e["b"]], SECTIONS[e["P"]["sec"]]["od"], obstacles_for(m, e)) for e in m.el if e["kind"] == "rod")
        res["obstacle_clear"] = float(wo); g["obstacles"] = 1.0 - wo
    res["g"] = g
    res["violation"] = float(sum(max(0.0, v) for v in g.values()))
    return (res, m) if detail else res


def groups_of(topo):
    gs = ["chords", "posts", "spokes", "V", "keels", "wheel_ring", "struts", "nose", "nose_mount", "pyramid", "skirt", "skirt_ties"]
    for k in TOPO_KEYS:
        if topo[k] and k != "arm_spars":
            gs.append("top" if k == "top_spider" else k)
    return gs


ORDER = sorted(SECTIONS, key=lambda s: SECTIONS[s]["gpm"])
THICK = ("t6h", "t8h", "t10h", "r10")                              # used only on a run whose OD cap is their OD (a larger thin tube beats them elsewhere)
MENU = ("r3", "t6", "t8", "t10", "t10h", "r10", "t16", "t20")     # carbon stock (DEFAULT 25 Oct): 8 types, 3.0 wherever it passes (CLAUDE.md); 7 did not
                                                                   # carry the skirt, all 14 save 4 g; 10/6 and 10 solid only where the wash cap binds


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


FIXED_ON = ("arm_spars", "top_spider")   # found by diagnosis / needed by cage first; the ablation re-tests each


def size_fsd(p, topo, iters=30, log=False, init=None):
    """fully stressed design per physical rod (a run between joints): each run gets the smallest section whose stress,
    buckling and socket indices pass every case at its limit; then stiffness limits (L2 mode, crash deflection) by
    stepping up the runs that carry them. Returns (sizes per element name, result, feasible)."""
    m0 = Model(p, topo, {}); runs = structural_members(m0)
    caps = guard_caps(m0)
    run_names = [[m0.el[i]["name"] for i in run] for run in runs]
    run_group = [m0.el[run[0]]["group"] for run in runs]
    gcap = {gname: SECTIONS[sec_]["od"] for gname, sec_ in p.get("cap_groups", {}).items()}   # geometric caps (funnel, pods), as OD
    run_od = [min([caps.get(m0.el[i]["name"], 99.0) for i in run] + [gcap.get(m0.el[run[0]]["group"], 99.0)]) for run in runs]
    crod = [carbon_rod(names[0]) for names in run_names]           # a physical rod (its runs through cross nodes) takes one section: the cap of all
    rod_od = {}
    for i, c_ in enumerate(crod): rod_od[c_] = min(rod_od.get(c_, 99.0), run_od[i])
    run_od = [rod_od[c_] for c_ in crod]
    allowed = [[s_ for s_ in ORDER if s_ in MENU and SECTIONS[s_]["od"] <= od_ and (s_ not in THICK or SECTIONS[s_]["od"] == od_)] or ["r3"]
               for od_ in run_od]                                  # sections that fit, lightest first; thick wall only where the cap binds
    def fit(i, s_):
        """s_ if it fits run i, else the lightest fitting section at least as strong, else the stiffest that fits"""
        if s_ in allowed[i]: return s_
        S = SECTIONS[s_]
        for a_ in allowed[i]:
            if SECTIONS[a_]["I"] >= S["I"] and SECTIONS[a_]["A"] >= S["A"]: return a_
        return max(allowed[i], key=lambda a_: (SECTIONS[a_]["I"], SECTIONS[a_]["A"]))
    def capped(req):
        return [fit(i, r_) for i, r_ in enumerate(req)]
    def step(i, s_, d, od_up=False, thin=False):
        """the next fitting section up (d 1) or down (d -1) from s_ in weight; od_up: a larger OD only (a socket needs one);
        thin: no thick wall (the blind last resort)"""
        k = ORDER.index(s_); nxt = [a_ for a_ in allowed[i] if (ORDER.index(a_) - k) * d > 0
                                    and (not od_up or SECTIONS[a_]["od"] > SECTIONS[s_]["od"]) and (not thin or a_ not in THICK)]
        return (nxt[0] if d > 0 else nxt[-1]) if nxt else s_
    sec = [(init or {}).get(names[0], "r3") if (init or {}).get(names[0]) in SECTIONS else "r3" for names in run_names]   # warm start (system loop)
    sec = capped(sec)
    # mirror pairs (y -> -y): both sides of a run take the larger section, so the cage is built symmetric
    def rkey(run, sgn):
        pts = sorted({tuple(np.round(np.asarray(m0.N[m0.el[i][e]], float) * [1, sgn, 1], 1)) for i in run for e in ("a", "b")})
        return tuple(pts)
    kpos = {rkey(r, 1): i for i, r in enumerate(runs)}
    mirror = [kpos.get(rkey(r, -1), i) for i, r in enumerate(runs)]
    def sym(req):
        out = [ORDER[max(ORDER.index(req[i]), ORDER.index(req[mirror[i]]))] for i in range(len(req))]
        big = {}
        for i, c_ in enumerate(crod): big[c_] = max(big.get(c_, 0), ORDER.index(out[i]))
        return [ORDER[big[c_]] for c_ in crod]                       # and one section along each physical rod
    def sizes_of(sec):
        return {n: s_ for names, s_ in zip(run_names, sec) for n in names}
    for it in range(iters):
        sizes = sizes_of(sec); res, m = evaluate(p, topo, sizes, detail=True)
        req = capped(sym(run_required(m, res, runs, allowed)))         # lightest passing section at the current forces, within the caps
        changed = False
        for i in range(len(sec)):
            k, kr = ORDER.index(sec[i]), ORDER.index(req[i])
            if kr > k: sec[i] = req[i]; changed = True                  # up at once
            elif kr < k: sec[i] = step(i, sec[i], -1); changed = True   # down one step per round (no oscillation)
        if log: print(it, round(res["mass_g"], 1), round(res["violation"], 2), flush=True)
        if not changed: break
    # stiffness: L2 mode and crash deflections, step up the runs on those paths
    l2_groups = ("keels", "keels_lo", "keels_x", "chin", "l2_braces", "nose_mount", "V", "nose", "v_diag")
    for it in range(30):
        prev = list(sec)
        sizes = sizes_of(sec); res, m = evaluate(p, topo, sizes, detail=True)
        bad = {k: v for k, v in res["g"].items() if v > 0 and k not in ("prop guard", "funnel", "motor clear", "obstacles", "prop wash")}
        if not bad: break
        stepped = False
        for key, mk in (("L2 mode", "_l2_mode"), ("belly mode", "_belly_mode")):
            if key not in bad: continue
            share = mode_energy_runs(m, res[mk], runs)                     # step the runs holding half the mode's strain energy
            order_ = np.argsort(-share); acc = 0.0
            for i in order_:
                if acc >= 0.5: break
                acc += share[i]
                if step(i, sec[i], 1) != sec[i]: sec[i] = step(i, sec[i], 1); stepped = True
        for k in bad:
            if k.endswith(" defl") or (k not in ("L2 mode", "belly mode") and bad[k] > 0):
                hn = k.split(" at ")[-1].split(" ")[0] if " at " in k else None
                sk = k.endswith(" socket")                               # a socket holds by OD and depth, not by wall
                for i, names in enumerate(run_names):
                    els = [e for e in m.el if e["name"] in names]
                    if hn and any(hn in (e["a"], e["b"]) for e in els) and step(i, sec[i], 1, od_up=sk) != sec[i]:
                        sec[i] = step(i, sec[i], 1, od_up=sk); stepped = True
        sec = capped(sym(sec))
        if not stepped:
            for i in range(len(sec)):                                     # last resort: everything one step (thin wall)
                if step(i, sec[i], 1, thin=True) != sec[i]: sec[i] = step(i, sec[i], 1, thin=True); stepped = True
            sec = capped(sec)
            if not stepped: break
        if sec == prev: break                                             # capped by the prop guard: nothing left to step
    sizes = sizes_of(sec); res = evaluate(p, topo, sizes)
    if res["violation"] == 0:                                             # take back what the stiffness steps did not need, heaviest first
        for i in sorted(range(len(sec)), key=lambda i: -SECTIONS[sec[i]]["gpm"] * sum(m0.el[j]["L"] for j in runs[i])):
            if sec[i] not in THICK and SECTIONS[sec[i]]["od"] < 12: continue
            j = mirror[i]; dn = step(i, sec[i], -1)
            if dn == sec[i]: continue
            trial = list(sec)
            for k in range(len(sec)):                                     # the whole physical rod and its mirror
                if crod[k] in (crod[i], crod[j]) and dn in allowed[k]: trial[k] = dn
            rt = evaluate(p, topo, sizes_of(trial))
            if rt["violation"] == 0: sec, res = trial, rt
    sizes = sizes_of(sec)
    return sizes, res, res["violation"] == 0


def run_required(m, res, runs, allowed=None):
    """per run, the lightest section (ORDER, within allowed[i] when given) whose stress, buckling and socket indices pass
    every case at its limit, holding the run's present axial force and moments; else the stiffest allowed"""
    lim = np.array([v["limit"] for v in res["cases"].values()]); U = res["_U"]
    mlim = np.array([v.get("mlimit", v["limit"]) for v in res["cases"].values()])
    out = []
    for ir, run in enumerate(runs):
        L = sum(m.el[i]["L"] for i in run)
        Nall, Mall = [], []
        for i in run:
            e = m.el[i]; fl = e["kl"] @ (e["T"] @ U[e["dofs"]])
            Nall.append(-fl[0]); Mall.append(np.hypot(np.maximum(abs(fl[4]), abs(fl[10])), np.maximum(abs(fl[5]), abs(fl[11]))))
        Nall, Mall = np.array(Nall), np.array(Mall)
        menu = allowed[ir] if allowed else ORDER
        choice = max(menu, key=lambda a_: (SECTIONS[a_]["I"], SECTIONS[a_]["A"]))
        for sname in menu:
            S = SECTIONS[sname]; cap = SOCKET_CAP * S["od"]; Pcr = math.pi ** 2 * E_C * S["I"] / L ** 2
            sig = (abs(Nall) / S["A"] + Mall * (S["od"] / 2) / S["I"]) / SIG_ALLOW
            ok = (sig / mlim).max() <= 1.0 and (abs(Nall) / cap / lim).max() <= 1.0 and (BUCKLE_SF * np.maximum(-Nall, 0).max(axis=0) / Pcr / mlim).max() <= 1.0
            if ok: choice = sname; break
        out.append(choice)
    return out


def run_need(m, res, runs, p, topo, sizes):
    """per run: the worst (index / case limit) over every case of stress, buckling and sockets"""
    cases_lim = [v["limit"] for v in res["cases"].values()]; cases_mlim = np.array([v.get("mlimit", v["limit"]) for v in res["cases"].values()])
    U, names = res["_U"], res["_names"]
    need = []
    for run in runs:
        worst = 0.0
        L = sum(m.el[i]["L"] for i in run); P = m.el[run[0]]["P"]; Pcr = math.pi ** 2 * P["E"] * P["Iy"] / L ** 2
        cap = SOCKET_CAP * P.get("od", 3.0)
        Nmin = None
        for i in run:
            e = m.el[i]; fl = e["kl"] @ (e["T"] @ U[e["dofs"]])
            N_ = -fl[0]; My = np.maximum(abs(fl[4]), abs(fl[10])); Mz = np.maximum(abs(fl[5]), abs(fl[11]))
            sg = abs(N_) / P["A"] + np.hypot(My, Mz) * P["c"] / P["Iy"]
            lim = np.array(cases_lim)
            worst = max(worst, float((sg / SIG_ALLOW / cases_mlim).max()), float((abs(N_) / cap / lim).max()))
            Nmin = np.minimum(Nmin, N_) if Nmin is not None else N_
        worst = max(worst, float((BUCKLE_SF * np.maximum(-Nmin, 0) / Pcr / cases_mlim).max()))
        need.append(worst)
    return need


def _set_crash(cg):
    global CRASH_G
    CRASH_G = cg


def _one(args):
    p, topo, fl2 = args
    global F_L2_MIN
    F_L2_MIN = fl2
    sizes, r, ok = size_fsd(p, topo); return topo, sizes, r, ok


def search(p, fl2, pool):
    free_keys = [k for k in TOPO_KEYS if k not in FIXED_ON]
    jobs = []
    for bits in itertools.product((0, 1), repeat=len(free_keys)):
        topo = dict(zip(free_keys, bits)); topo.update({k: 1 for k in FIXED_ON}); jobs.append((p, topo, fl2))
    done = pool.map(_one, jobs)
    table, best = [], None
    for topo, sizes, r, ok in done:
        worst = max(r["g"], key=r["g"].get)
        table.append(dict(topo=topo, sizes=sizes, feasible=ok, mass_g=r["mass_g"], f_l2=r["f_l2"], f_first=r["f_first"], worst=worst, worst_g=r["g"][worst]))
        if ok and (best is None or r["mass_g"] < best[2]["mass_g"]): best = (topo, sizes, r)
    abl = []
    if best:                                                          # ablation: each fixed option off on the winner
        for k in FIXED_ON:
            t2 = dict(best[0], **{k: 0}); _, s2, r2, ok2 = _one((p, t2, fl2))
            abl.append(dict(off=k, feasible=ok2, mass_g=r2["mass_g"], f_l2=r2["f_l2"]))
    return table, best, abl


def main(pin, pout, levels):
    from multiprocessing import Pool
    global CRASH_G
    p = json.load(open(pin))
    out = dict(params=p, levels={})
    crash_levels = [float(x) for x in os.environ.get("CAGE_CRASH_G", "35").split(",")]
    for cg in crash_levels:
      CRASH_G = cg
      with Pool(4, initializer=_set_crash, initargs=(cg,)) as pool:
        for fl2 in levels:
            table, best, abl = search(p, fl2, pool)
            ok = [t for t in table if t["feasible"]]
            print(f"crash {cg:.0f} g, L2 mode ≥ {fl2:.0f} Hz: {len(ok)} of {len(table)} topologies feasible", flush=True)
            for t in sorted(ok, key=lambda t: t["mass_g"])[:5]:
                print(f"  {t['mass_g']:6.1f} g  f_L2 {t['f_l2']:5.0f}  f1 {t['f_first']:5.0f}  "
                      + " ".join(k for k, v in t["topo"].items() if v and k not in FIXED_ON) + "  | " + " ".join(f"{a} x{b}" for a, b in __import__("collections").Counter(t["sizes"].values()).items()), flush=True)
            for a in abl:
                print(f"  ablation {a['off']} off: feasible {a['feasible']}, {a['mass_g']:.1f} g, f_L2 {a['f_l2']:.0f}", flush=True)
            lv = dict(table=table, ablation=abl)
            if best: lv.update(topo=best[0], sizes=best[1], result=best[2])
            lv["crash_g"] = cg
            out["levels"][f"{int(fl2)}Hz_{int(cg)}g"] = lv
            json.dump(out, open(pout, "w"), indent=1, default=float)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], [float(x) for x in (sys.argv[3] if len(sys.argv) > 3 else "100").split(",")])
