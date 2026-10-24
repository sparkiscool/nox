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
  direction (largest support), hub fixed: rod index σ/600 MPa, buckling 1.5 x Euler, sockets (600 N per 3.0 rod, scaled
  with OD), arm PETG σ/51 MPa, all ≤ 1.0; the touching node moves ≤ 9 toward the drone (12 prop gap, 3 kept).
- L2 crash: 35 g x the L2 group at its CG in 14 directions: the same indices ≤ 1.0.
- Handling: 50 N on every real cage node in ±x, ±y, ±z: indices ≤ 1.0 (DEFAULT: a misuse load; L gives 0.5 to flight and landing).
- Catch landing: 6 g x the drone up through the 8 wheels; latch misuse 100 N down the slope on each ±y-face wheel (≤ 1.0);
  swap pull 50 N into the face on two wheels (≤ 0.5).
- Breaking order (reported): worst socket index (the node) vs worst rod and arm index in the crash cases.
Objective: cage mass (rods + printed nodes + arm spars); 3.0 solid wherever it passes.
"""
import json, math, os, sys, itertools
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
NOSE_Y, NOSE_GAP = 58.0, 3.0 + 2.0                          # nose box half width; ahead of the L2 / camera by 3 + rod
F_FLAT = HALF + PROP_R + GUARD_GAP + ROD_RMAX               # 288.5
C_CHAMF = 2 * HALF + math.sqrt(2) * (PROP_R + GUARD_GAP + ROD_RMAX)   # 489.6
L2_TILT = 105.0
A = np.array([math.sin(math.radians(L2_TILT)), 0.0, math.cos(math.radians(L2_TILT))])
UP_B = np.array([-A[2], 0.0, A[0]])
MOTOR_DEG = (45, 135, 225, 315)
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
}
E_C, G_C, E_P, G_P = 120000.0, 5000.0, 2780.0, 1000.0
SIG_ALLOW, SOCKET_3, BUCKLE_SF = 600.0, 600.0, 1.5
DEFL_ALLOW, F_L2_MIN = 9.0, 100.0                              # crash deflection, L2 mode (L Sensor mounts)
F_FIRST_MIN = 0.0                                           # cage's own lowest mode floor (set below after the trade)
M_DRONE, V_HIT = 2.33e-3, 1000.0                            # t, mm/s
NODE_G, CORNER_G = 2.0, 6.0                                 # printed ring node / corner node (E, g)

TOPO_KEYS = ("keels", "mid_spokes", "ring_diag", "l2_braces", "v_diag", "strut2", "spokes_hi", "tip_rails", "keels_lo", "keels_x", "chin", "arm_spars", "nose_x", "top_spider")


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
    if topo["spokes_hi"]:                                             # second support height: arm top (z 45) to the same ring nodes
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
    mass["apexM"] = 266e-6                                                 # L2 230 + camera 15 + plate, rods, apex ~21 (E)
    for vn, sy in (("V0", 1), ("V7", -1)):
        for lvl in ("lo", "hi"):
            mid = node(f"Vmid {vn}{lvl}", (N[f"{vn}{lvl}"] + N[f"S{lvl}{sy:+d}"]) / 2)
            mem(f"V {vn}{lvl} a", f"{vn}{lvl}", mid, "V"); mem(f"V {vn}{lvl} b", mid, f"S{lvl}{sy:+d}", "V")
        if topo["v_diag"]:
            mem(f"vdiag {vn}", f"{vn}hi", f"Slo{sy:+d}", "v_diag")
    for sy in (1, -1):
        if topo.get("keels", 1):
            mem(f"keel {sy:+d}", f"fhubB{sy:+d}", f"Shi{sy:+d}", "keels")   # bottom plate front up to the upper socket (16 deg)
        if topo["keels_lo"]:                                              # top plate front down to the lower socket: the keels cross (pitch, bounce)
            mem(f"keello {sy:+d}", f"fhubT{sy:+d}", f"Slo{sy:+d}", "keels_lo")
            mem(f"keeldg {sy:+d}", f"fhubB{sy:+d}", f"Slo{sy:+d}", "keels_lo")   # the diagonal: without it the two keels are a four-bar
        if topo["keels_x"]:                                               # crossed in plan: the hub holds the L2 sideways
            mem(f"keelx {sy:+d}", f"fhubB{sy:+d}", f"Slo{-sy:+d}", "keels_x")
        if topo["chin"]:                                                  # from the sled front, 30 under the plates: 23 deg of climb
            mem(f"chin {sy:+d}", f"chin{sy:+d}", f"Shi{sy:+d}", "chin")
    if topo["l2_braces"]:
        for k, (mx, my) in enumerate(mot):
            if mx > dm:
                sy = 1 if my > 0 else -1
                mem(f"brace m{k}", f"tipb{k}", f"Shi{sy:+d}", "l2_braces")      # arm tip underside up to the upper socket
                mem(f"bracelo m{k}", f"tipt{k}", f"Slo{sy:+d}", "l2_braces")    # arm tip top down to the lower socket (crossed)
                mem(f"bracedg m{k}", f"tipb{k}", f"Slo{sy:+d}", "l2_braces")    # diagonal: no four-bar
    # nose box round the L2 and the front camera: 4 nodes 3 + rod ahead of them, longerons back to the front vertices
    lx = np.array([math.cos(math.radians(L2_TILT)), 0, -math.sin(math.radians(L2_TILT))])
    Bv = np.array([xB, 0, zB])
    corners_l2 = [Bv + lx * a + np.array([0, b, 0]) + A * c for a in (-37.5, 37.5) for b in (-37.5, 37.5) for c in (0.0, 39.0)] + [Bv + A * 65]
    l2x = max(q[0] for q in corners_l2); l2zmax = max(q[2] for q in corners_l2); l2zmin = min(q[2] for q in corners_l2)
    xn = max(l2x, p.get("cam_front", xB)) + NOSE_GAP
    ztn = max(l2zmax, p.get("cam_top", zB + 60)) + NOSE_GAP; zbn = min(l2zmin, p.get("cam_bot", l2zmin)) - NOSE_GAP
    ny = p.get("nose_y", NOSE_Y)                                      # wide enough for the two front cameras beside the L2
    for sy in (1, -1):
        node(f"NT{sy:+d}", (xn, ny * sy, ztn)); node(f"NB{sy:+d}", (xn, ny * sy, zbn))
        mem(f"nose vert {sy:+d}", f"NT{sy:+d}", f"NB{sy:+d}", "nose")
    mem("nose top", "NT+1", "NT-1", "nose"); mem("nose bot", "NB+1", "NB-1", "nose")
    if topo["nose_x"]:
        mem("nose x1", "NT+1", "NB-1", "nose_x"); mem("nose x2", "NT-1", "NB+1", "nose_x")
    for vn, sy in (("V0", 1), ("V7", -1)):
        mem(f"longeron hi {sy:+d}", f"{vn}hi", f"NT{sy:+d}", "nose"); mem(f"longeron lo {sy:+d}", f"{vn}lo", f"NB{sy:+d}", "nose")
        mem(f"nose side dg {sy:+d}", f"{vn}hi", f"NB{sy:+d}", "nose")
        mem(f"mount hi {sy:+d}", f"Shi{sy:+d}", f"NT{sy:+d}", "nose_mount"); mem(f"mount lo {sy:+d}", f"Slo{sy:+d}", f"NB{sy:+d}", "nose_mount")
        mem(f"mount dg {sy:+d}", f"Shi{sy:+d}", f"NB{sy:+d}", "nose_mount")
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
    if topo["top_spider"]:
        for r in vx:
            mem(f"spider {r[0]}", "TN", f"{r[0]}hi", "top")
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
        if topo["strut2"]:
            mem(f"strut2 {sx:+d}{sy:+d}", f"SR{sx:+d}{sy:+d}", f"hubc{sx:+d}{sy:+d}", "strut2")
    for sx in (1, -1):
        mem(f"wring x{sx:+d}", f"WAi{sx:+d}+1", f"WAi{sx:+d}-1", "wheel_ring")
    for sy in (1, -1):
        mem(f"wring y{sy:+d}", f"WBi+1{sy:+d}", f"WBi-1{sy:+d}", "wheel_ring")
    if p.get("feet") is not None:                                     # belly frame + skid (layout A: the catch sits under the motor square)
        fx, fy, fz = p["feet"]
        for sx in (1, -1):
            for sy in (1, -1):
                # printed belly frame tab on each corner block at wheel height: carries the cartridge receiver, the pods
                # and the pack (720 g, E, a quarter each); vertical skid leg inside the 168 opening down to the foot
                node(f"BF{sx:+d}{sy:+d}", (xc + fx * sx, fy * sy, wz)); mem(f"belly tab {sx:+d}{sy:+d}", f"corner{sx:+d}{sy:+d}", f"BF{sx:+d}{sy:+d}", None, "link")
                mass[f"BF{sx:+d}{sy:+d}"] = 180e-6
                node(f"FT{sx:+d}{sy:+d}", (xc + fx * sx, fy * sy, fz))
                mem(f"skid leg {sx:+d}{sy:+d}", f"BF{sx:+d}{sy:+d}", f"FT{sx:+d}{sy:+d}", "skid_belly")
        for sy in (1, -1):                                            # side bar + rail close each side; no rail across the ends (the pack drops out the bottom)
            mem(f"belly bar {sy:+d}", f"BF+1{sy:+d}", f"BF-1{sy:+d}", "skid_belly")
            mem(f"skid rail {sy:+d}", f"FT+1{sy:+d}", f"FT-1{sy:+d}", "skid_belly")
    info = dict(ring=rp, pnodes=pnodes, motors=mot, corners=corners, sockets=S, nose=(xn, ztn, zbn))
    return N, M, fixed, mass, info


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
        g += NODE_G * (2 * len(self.info["pnodes"]) + 4 + 1 + sum(n.startswith(("K", "FT")) or n == "TR" for n in self.N)) + CORNER_G * 4   # ring lo + hi, nose 4, top, skirt, tower
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
        return n_ in model.fixed or deg.get(n_, 0) != 2 or len(same) != 2   # a hub (fixed) node always ends a run
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
CRASH_G = 35.0                                                # dimensions L Crash (DEFAULT, E)


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


def funnel_clear(a, b, od, p):
    """smallest clearance (mm) of a rod to the seated funnel (catch centre on the funnel axis, p["seat_dz"] = bottom
    plate top in the seat frame): 45 deg faces from the 168 opening at seat z -0.8 to the top at 75.1; under the
    opening the rod stays inside it at ±5 deg yaw with 1.0 (dimensions G (2))"""
    if p.get("seat_dz") is None: return np.inf
    t = np.linspace(0, 1, 80)[:, None]; q = np.asarray(a, float) + t * (np.asarray(b, float) - np.asarray(a, float))
    q = q + np.array([-p["xc"], 0.0, p["seat_dz"]]); z = q[:, 2]; c = np.inf
    for ax in (0, 1):
        for sg in (1, -1):
            x = sg * q[:, ax]; inz = (z > -0.8) & (z < FACE_TOP) & (np.abs(q[:, 1 - ax]) < OPEN_HALF + 0.8 + z)
            if inz.any(): c = min(c, float(((z[inz] + OPEN_HALF + 0.8 - x[inz]) / math.sqrt(2) - od / 2).min()))
    low = q[z <= -0.8]
    if len(low):
        for yaw in np.radians((-5.0, 0.0, 5.0)):
            cy, sy_ = math.cos(yaw), math.sin(yaw)
            r = np.abs(np.stack([low[:, 0] * cy - low[:, 1] * sy_, low[:, 0] * sy_ + low[:, 1] * cy], 1)).max()
            c = min(c, float(OPEN_HALF - r - od / 2))
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


def guard_caps(m):
    """largest section index (ORDER) per rod that keeps the prop clearance (12) and the funnel clearance (1.0);
    -1 = no section does (the layout fails)"""
    caps = {}
    for e in m.el:
        if e["kind"] != "rod": continue
        cap = -1
        for i, sec in enumerate(ORDER):
            od = SECTIONS[sec]["od"]
            if e["group"] not in PLAN_GROUPS and prop_clear(m.N[e["a"]], m.N[e["b"]], od, m.info["motors"]) < GUARD_GAP: break
            if funnel_clear(m.N[e["a"]], m.N[e["b"]], od, m.p) < 1.0: break
            if motor_clear(m.N[e["a"]], m.N[e["b"]], od, m.info["motors"]) < 1.0: break
            cap = i
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


def crash_dirs():
    d = [np.array(v, float) for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    d += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    return d


def _indices(m, mr, runs):
    """worst rod stress index, socket index, buckling index, arm index over the elements"""
    rod = sock = arm = 0.0
    for (N_, sg), e in zip(mr, m.el):
        if e["kind"] == "rod":
            rod = max(rod, sg / SIG_ALLOW); sock = max(sock, abs(N_) / (SOCKET_3 * e["P"]["od"] / 3.0))
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
    g["L2 mode"] = F_L2_MIN / f_l2 - 1
    bf = [n for n in m.names if n.startswith("BF")]
    if bf:                                                            # the down camera and flow ride the belly frame: a sensor mount too
        sh = m.mode_share(bf); bm = [i for i, s_ in enumerate(sh) if s_ >= MODE_SHARE]
        res["f_belly"] = float(f[bm[0]]) if bm else float(f[-1]); res["_belly_mode"] = bm[0] if bm else len(f) - 1
        g["belly mode"] = F_L2_MIN / res["f_belly"] - 1
    m_drone = p.get("m_drone_g", 2200.0) * 1e-6                       # t (g -> t)
    cage_nodes = [n for n in m.names if n not in m.fixed and (n.endswith(("lo", "hi")) and not n.startswith(("S", "Vmid"))
                                                              or n.startswith(("NT", "NB", "TN", "TR", "corner", "KV", "KP", "FT")))]
    P_ = np.array([m.N[n] for n in cage_nodes])
    cases = []                                                       # (name, loads, limit, kind, hit node, direction)
    for d in crash_dirs():
        k = int(np.argmax(P_ @ d)); n_ = cage_nodes[k]
        cases.append((f"crash {np.round(d, 2).tolist()} at {n_}", {n_: -CRASH_G * G_ACC * m_drone * d}, 1.0, "crash", n_, d))
    for d in crash_dirs():
        cases.append((f"L2 crash {np.round(d, 2).tolist()}", {"apexM": CRASH_G * G_ACC * 266e-6 * d}, 1.0, "l2", None, d))
    real_nodes = [n for n in cage_nodes if n[:-2] in m.info["pnodes"] or n.startswith(("NT", "NB", "TN", "TR", "corner", "KV", "KP", "FT"))]
    for n_ in real_nodes:
        for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            cases.append((f"handling {n_} {d}", {n_: 50.0 * np.array(d, float)}, 1.0, "handling", None, None))   # misuse-type (DEFAULT)
    wheels = [f"W{a}{sx:+d}{sy:+d}" for a in ("A", "B") for sx in (1, -1) for sy in (1, -1)]
    cases.append(("catch landing 6 g", {w: np.array([0, 0, 6.0 * G_ACC * m_drone / 8]) for w in wheels}, 0.5, "landing", None, None))
    feet = [n for n in m.names if n.startswith("FT")]
    if feet:                                                          # floor landing on the 4 feet (dimensions G (5): 6 g)
        cases.append(("floor landing 6 g", {f_: np.array([0, 0, 6.0 * G_ACC * m_drone / len(feet)]) for f_ in feet}, 0.5, "landing", None, None))
    for sx in (1, -1):
        for sy in (1, -1):
            cases.append((f"latch WB{sx:+d}{sy:+d}", {f"WB{sx:+d}{sy:+d}": 100.0 * np.array([0, -sy, -1]) / math.sqrt(2)}, 1.0, "latch", None, None))
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
    cap = np.array([SOCKET_3 * e["P"].get("od", 3.0) / 3.0 for e in m.el])
    rod_ix = (Sg[is_rod] / SIG_ALLOW).max(axis=0); sock_ix = (abs(Nax[is_rod]) / cap[is_rod, None]).max(axis=0)
    arm_ix = (Sg[is_arm] / 51.0).max(axis=0)
    buck_ix = np.zeros(nc)
    for run in runs:
        P = m.el[run[0]]["P"]; L = sum(m.el[i]["L"] for i in run); Pcr = math.pi ** 2 * P["E"] * P["Iy"] / L ** 2
        buck_ix = np.maximum(buck_ix, BUCKLE_SF * np.maximum(-Nax[run], 0).max(axis=0) / Pcr)
    crash_node = crash_member = 0.0
    for j, (cname, ld, lim, kind, hn, d) in enumerate(cases):
        u = U[:, j]
        ix = dict(rod=float(rod_ix[j]), socket=float(sock_ix[j]), buckle=float(buck_ix[j]), arm=float(arm_ix[j]))
        r = dict(ix, limit=lim)
        if kind == "crash":
            r["defl"] = float(-(u[6 * m.idx[hn]:6 * m.idx[hn] + 3] @ d))
            g[f"{cname} defl"] = r["defl"] / 9.0 - 1
            crash_node = max(crash_node, ix["socket"]); crash_member = max(crash_member, ix["rod"], ix["arm"], ix["buckle"])
        res["cases"][cname] = r
        for key in ("rod", "socket", "buckle", "arm"):
            g[f"{cname} {key}"] = ix[key] / lim - 1
    res["breaking_order"] = dict(node=crash_node, member=crash_member, node_first=crash_node * 1.0 >= crash_member * 1.5)
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


FIXED_ON = ("keels_lo", "keels_x", "arm_spars", "top_spider")   # found by diagnosis / needed by cage first; the ablation re-tests each


def size_fsd(p, topo, iters=30, log=False):
    """fully stressed design per physical rod (a run between joints): each run gets the smallest section whose stress,
    buckling and socket indices pass every case at its limit; then stiffness limits (L2 mode, crash deflection) by
    stepping up the runs that carry them. Returns (sizes per element name, result, feasible)."""
    m0 = Model(p, topo, {}); runs = structural_members(m0)
    caps = guard_caps(m0)
    run_names = [[m0.el[i]["name"] for i in run] for run in runs]
    run_group = [m0.el[run[0]]["group"] for run in runs]
    gcap = {gname: ORDER.index(sec_) for gname, sec_ in p.get("cap_groups", {}).items()}   # geometric caps (funnel, pods)
    run_cap = [min([caps.get(m0.el[i]["name"], len(ORDER) - 1) for i in run] + [gcap.get(m0.el[run[0]]["group"], len(ORDER) - 1)]) for run in runs]
    def capped(req):
        return [ORDER[max(0, min(ORDER.index(r_), run_cap[i]))] for i, r_ in enumerate(req)]
    sec = ["r3"] * len(runs)
    # mirror pairs (y -> -y): both sides of a run take the larger section, so the cage is built symmetric
    def rkey(run, sgn):
        pts = sorted({tuple(np.round(np.asarray(m0.N[m0.el[i][e]], float) * [1, sgn, 1], 1)) for i in run for e in ("a", "b")})
        return tuple(pts)
    kpos = {rkey(r, 1): i for i, r in enumerate(runs)}
    mirror = [kpos.get(rkey(r, -1), i) for i, r in enumerate(runs)]
    def sym(req):
        return [ORDER[max(ORDER.index(req[i]), ORDER.index(req[mirror[i]]))] for i in range(len(req))]
    def sizes_of(sec):
        return {n: s_ for names, s_ in zip(run_names, sec) for n in names}
    for it in range(iters):
        sizes = sizes_of(sec); res, m = evaluate(p, topo, sizes, detail=True)
        req = capped(sym(run_required(m, res, runs)))                  # smallest passing section at the current forces, within the prop guard
        changed = False
        for i in range(len(sec)):
            k, kr = ORDER.index(sec[i]), ORDER.index(req[i])
            if kr > k: sec[i] = req[i]; changed = True                  # up at once
            elif kr < k: sec[i] = ORDER[k - 1]; changed = True          # down one step per round (no oscillation)
        if log: print(it, round(res["mass_g"], 1), round(res["violation"], 2), flush=True)
        if not changed: break
    # stiffness: L2 mode and crash deflections, step up the runs on those paths
    l2_groups = ("keels", "keels_lo", "keels_x", "chin", "l2_braces", "nose_mount", "V", "nose", "v_diag")
    for it in range(30):
        prev = list(sec)
        sizes = sizes_of(sec); res, m = evaluate(p, topo, sizes, detail=True)
        bad = {k: v for k, v in res["g"].items() if v > 0 and k not in ("prop guard", "funnel", "motor clear")}
        if not bad: break
        stepped = False
        for key, mk in (("L2 mode", "_l2_mode"), ("belly mode", "_belly_mode")):
            if key not in bad: continue
            share = mode_energy_runs(m, res[mk], runs)                     # step the runs holding half the mode's strain energy
            order_ = np.argsort(-share); acc = 0.0
            for i in order_:
                if acc >= 0.5: break
                acc += share[i]
                if ORDER.index(sec[i]) < len(ORDER) - 1: sec[i] = ORDER[ORDER.index(sec[i]) + 1]; stepped = True
        for k in bad:
            if k.endswith(" defl") or (k not in ("L2 mode", "belly mode") and bad[k] > 0):
                hn = k.split(" at ")[-1].split(" ")[0] if " at " in k else None
                for i, names in enumerate(run_names):
                    els = [e for e in m.el if e["name"] in names]
                    if hn and any(hn in (e["a"], e["b"]) for e in els) and ORDER.index(sec[i]) < len(ORDER) - 1:
                        sec[i] = ORDER[ORDER.index(sec[i]) + 1]; stepped = True
        sec = capped(sym(sec))
        if not stepped:
            for i in range(len(sec)):                                     # last resort: everything one step
                if ORDER.index(sec[i]) < len(ORDER) - 1: sec[i] = ORDER[ORDER.index(sec[i]) + 1]; stepped = True
            sec = capped(sec)
            if not stepped: break
        if sec == prev: break                                             # capped by the prop guard: nothing left to step
    sizes = sizes_of(sec); res = evaluate(p, topo, sizes)
    return sizes, res, res["violation"] == 0


def run_required(m, res, runs):
    """per run, the smallest section (ORDER) whose stress, buckling and socket indices pass every case at its limit,
    holding the run's present axial force and moments"""
    lim = np.array([v["limit"] for v in res["cases"].values()]); U = res["_U"]
    out = []
    for run in runs:
        L = sum(m.el[i]["L"] for i in run)
        Nall, Mall = [], []
        for i in run:
            e = m.el[i]; fl = e["kl"] @ (e["T"] @ U[e["dofs"]])
            Nall.append(-fl[0]); Mall.append(np.hypot(np.maximum(abs(fl[4]), abs(fl[10])), np.maximum(abs(fl[5]), abs(fl[11]))))
        Nall, Mall = np.array(Nall), np.array(Mall)
        choice = ORDER[-1]
        for sname in ORDER:
            S = SECTIONS[sname]; cap = SOCKET_3 * S["od"] / 3.0; Pcr = math.pi ** 2 * E_C * S["I"] / L ** 2
            sig = (abs(Nall) / S["A"] + Mall * (S["od"] / 2) / S["I"]) / SIG_ALLOW
            ok = (sig / lim).max() <= 1.0 and (abs(Nall) / cap / lim).max() <= 1.0 and (BUCKLE_SF * np.maximum(-Nall, 0).max(axis=0) / Pcr / lim).max() <= 1.0
            if ok: choice = sname; break
        out.append(choice)
    return out


def run_need(m, res, runs, p, topo, sizes):
    """per run: the worst (index / case limit) over every case of stress, buckling and sockets"""
    cases_lim = [v["limit"] for v in res["cases"].values()]
    U, names = res["_U"], res["_names"]
    need = []
    for run in runs:
        worst = 0.0
        L = sum(m.el[i]["L"] for i in run); P = m.el[run[0]]["P"]; Pcr = math.pi ** 2 * P["E"] * P["Iy"] / L ** 2
        cap = SOCKET_3 * P.get("od", 3.0) / 3.0
        Nmin = None
        for i in run:
            e = m.el[i]; fl = e["kl"] @ (e["T"] @ U[e["dofs"]])
            N_ = -fl[0]; My = np.maximum(abs(fl[4]), abs(fl[10])); Mz = np.maximum(abs(fl[5]), abs(fl[11]))
            sg = abs(N_) / P["A"] + np.hypot(My, Mz) * P["c"] / P["Iy"]
            lim = np.array(cases_lim)
            worst = max(worst, float((sg / SIG_ALLOW / lim).max()), float((abs(N_) / cap / lim).max()))
            Nmin = np.minimum(Nmin, N_) if Nmin is not None else N_
        worst = max(worst, float((BUCKLE_SF * np.maximum(-Nmin, 0) / Pcr / np.array(cases_lim)).max()))
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
