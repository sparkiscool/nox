"""Overseer printed arms v1: front and rear pair, nox/topo (organic), from layout v5 / cage v3 (27 Oct).

python3 nox/overseer/arm_topo_v1.py front|rear [--gen CM3] [--h H] [--out DIR] [--git]   topology run
python3 nox/overseer/arm_topo_v1.py front|rear --setup                                    setup summary only
python3 nox/overseer/arm_topo_v1.py front|rear --modes MESH [--out DIR]                    first modes of a built arm (Hz)
python3 nox/overseer/arm_topo_v1.py front|rear --place MESH                               the 2 arms of the pair in the drone frame
python3 nox/overseer/arm_topo_v1.py front --rodcheck MESH [--out DIR]                     rods: bore clear, sockets grip, bonded length
python3 nox/overseer/arm_topo_v1.py front --rodcheck-setup [--out DIR]                    v6: the same + diagonal geometry on drawn - cuts

Requirements: `dimensions.md` "System Overseer: arms" (the system pass for the arms, numbers tagged there). The run is
in the arm's own frame (x from the root centre to the motor, z the drone z, z 0 = bottom plate top), so the grid hugs
the beam; the left arm (1 front, 2 rear) is run and placed, the right arm (4, 3) is its mirror in y: every load case is
mirror-symmetric (yaw torque both signs, lateral both sides), so the mirror is the same optimum.
Cage nodes and spoke / L2 brace sockets are left off this version (Spark 27 Oct: "ignore the cage nodes on the arms for
now"); the cage pass adds them and reruns.

Drawn (mounting only): root contact pads on the top and bottom plates with 2 + 2 M3 heat-set insert bosses (plate holes
from the S500 STEP, dimensions O / M), the motor pad (X2216 16 / 19 M3 cross, Ø10.5 shaft bore) with the screw-head
pocket under it and its 3.5 boss wall. Cuts: insert holes, motor holes, pocket, hub electronics + 1, the L2 keel hub + 1.
Supports: the two plate contact faces (top plate, bottom plate). Build -z: the flat top (plate face, motor face) on the bed.
"""
import argparse, json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import numpy as np, trimesh, manifold3d as mf

# inputs: v1 is pinned to the layout its runs used (runs/overseer_arms/v1/inputs, from layout v5 at af98ac52); arms v2
# (arm_topo_v2.py) sets the current layout and the core session's parts through these
VERSION = os.environ.get("NOX_ARM_VERSION", "v1")
LAYOUT = os.environ.get("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_arms", "v1", "inputs"))
CORE = os.environ.get("NOX_ARM_CORE")               # a parts folder whose meshes (+ CLEAR) are keep-outs, or None
LINKS = os.environ.get("NOX_ARM_LINKS") == "1"      # v2 (Spark 27 Oct: "add the arm nodes"): cage spoke + L2 brace sockets
G = 9.81

# ------------------------------------------------------------------------------------------- numbers (dimensions.md)
ROOT_H = 45.0                         # root between the plates, arm top = top plate underside (O Arms and hub, DEFAULT)
U_ROOT = (58.0, float(os.environ.get("NOX_ARM_ROOT_U1", 75.0)))   # root block along the diagonal (O: inner face u >= 58 for
                                      # |v| <= 20; stock block to 75; v3: to the plate corner, clipped by each plate outline)
V_ROOT = 21.0
TOP_INS = [(63.17, 15.0), (63.17, -15.0)]      # M3 into the top plate (O, derived)
BOT_INS = [(63.74, 10.0), (63.74, -10.0)]      # M3 into the bottom plate (O, derived)
INS_D, INS_DEPTH, BOSS_D = 4.0, 7.0, 12.0      # M3 heat-set insert hole (O), boss Ø12 (attachment rule)
ROOT_PAD_T = 9.0                      # root contact pads 9 thick: the insert depth + 2, and room for the chord bores
PAD_T = 6.0                           # motor pad (README: plates >= 6 for the gate's 2 mm core)
FLANGE_T = 6.0                        # v4 root flanges: the arm's top / bottom face just past the plate edges (rear v3: crash 1.22 there)
MOTOR_R_PAD = 18.0                    # motor pad Ø36 (O v0 pad)
MOTOR_SLOTS = (8.0, 9.5)              # X2216 M3 pairs on 16 and 19 (M): a radial slot r 8 .. 9.5 on each 45° line, so the
MOTOR_SLOT_DEG = (45.0, 135.0, 225.0, 315.0)   # pad takes the motor either hand (the right arms are mirrors) with its leads
SHAFT_D, POCKET_R, POCKET_WALL, POCKET_H = 8.6, 13.5, 3.5, 6.0           # to the root; bore Ø8.6 (shaft 3.175 + clip, 2.0 web to
                                                                         # the slots, DEFAULT, CHECK P; stock Ø10.5); heads at r 9.5 + 2.85
MOTOR_HOLE_D = 3.2
ENV_R_TIP = 20.0                      # plan envelope round the motor (pad 18 + 2)
CLEAR = 1.0                           # off the hub electronics and the keel hub
MOTOR_DEG = {"front": 45.0, "rear": 135.0}     # arm 1 / arm 2 diagonal (left side)

# carbon chords (front pair only; concept review 27 Oct: PETG alone tops out near the mode target on the 179 front arm,
# and a modal check of a plain 30 wide box beam read its first mode lateral at ~130 Hz). A tapered space frame: 4 x 3.0
# carbon rods (hard rule), top and bottom on each side, from two shoulders beside the root (arm frame x 25, y ±35, outside
# both plates) to y ±12 beside the motor pocket; bore 3.2 (coupon pick), open at the shoulder end (the rod slides in from
# there and is epoxied), blind at the motor end inside the pocket wall. Drawn: a socket at each rod end; the topology
# webs, root and pad join them, the rods are stiff inserts in its FE (Part.insert).
ROD_D, ROD_BORE = 3.0, 3.2      # bore 3.2: Spark's pick from the printed coupons (27 Oct, P; OD + 0.3 was loose in PETG)
ROD_ID = 0.0                          # solid rod
CHORD = os.environ.get("NOX_ARM_CHORD", "rod3")
if CHORD == "tube6":                  # system pass after 3 strikes (27 Oct): 6/4 carbon tube chords (the cage's stock), bore 6.2
    ROD_D, ROD_BORE, ROD_ID = 6.0, 6.2, 4.0
SHOULDER = {"front": (25.0, 35.0, 9.0)}              # arm-frame x, |y|, plan radius of the shoulder envelope disc
if CHORD == "tube6":
    SHOULDER["rear"] = tuple(float(v) for v in os.environ.get("NOX_ARM_REAR_SHOULDER", "20,28,9").split(","))
ROD_TIP = (15.0, 12.0)                               # rod end: x = L - 15, |y| 12 (r 19.2 from the motor axis, in the wall)
ROD_Z = tuple(float(v) for v in os.environ.get("NOX_ARM_ROD_Z", "41.0,4.0").split(","))   # top / bottom rod centres
                                                     # (v3: 39.5 / 5.5, front v2 read a 1.2 wall under the bottom bore)
CHORD_SKIN = 2.65 if CHORD == "rod3" else 2.5   # socket wall round the bore (>= skin 2.5)
SOCKET_L = 18.0                       # drawn socket at each rod end (6 x the rod: epoxy bond ~ 2 MPa x pi x 3 x 18 = 340 N, E)
if CHORD == "tube6":
    SOCKET_L = 20.0                   # tube: epoxy ~2 MPa x pi x 6 x 20 = 750 N an end (E)
ROD_AREA = math.pi / 4 * (ROD_D ** 2 - ROD_ID ** 2)   # carbon section carried by each chord
E_CARBON = 120000.0                   # MPa, pultruded 3.0 rod along its axis (E)
RHO_CARBON = 1.55                     # g/cm3 (E)
E_PETG = 2780.0                       # MPa, nox/topo/material.py

# ESC: a 4-in-1 under the FC, the stack on top of the top plate (Spark 27 Oct), nothing on the arms; the motor leads run
# along the arm's flat top to it, zip-tied round the top carbon rods' free spans (front) or the beam (DEFAULT)

# cage links on the arms (v2, Spark 27 Oct; links and forces from the cage frame model, `arm_joint_loads.py`): 6/4 tube
# sockets, bore 6.2 x 15 + 0.6 lead-in (Spark's coupon pick, dimensions System Overseer v5), wall 2.5. The 3 spokes of each arm
# meet in one printed node beside the motor, off its axis (the screw pocket's key column, r 14, stays clear: every spoke line
# >= 17 from the axis in plan, C); the 2 L2 braces end inside the front beam (z 30 / 39) so the top stays flat for the bed.
TUBE_BORE, TUBE_DEPTH, TUBE_LEAD, TUBE_WALL = 6.2, 15.0, 0.6, 2.5
SPOKE_NODE = {"front": (178.9, 146.7, 8.0), "rear": (-53.3, 119.7, 25.0)}   # left arm, drone frame (C, searched)


def _spoke_nodes_from_layout():
    """layout v5 after the cage's 27 Oct rebalance: each arm's spokes start at its node (the end the 3 spokes share);
    older layouts started them at the motor (y 139.5) and keep SPOKE_NODE above"""
    try:
        links = json.load(open(os.path.join(LAYOUT, "result.json")))["links"]
    except (OSError, KeyError, ValueError):
        return
    for which, tag in (("front", "1"), ("rear", "2")):
        ends = [tuple(np.round(e, 2)) for k, v in links.items() if k.startswith(f"spoke {tag}.") for e in v]
        common = [e for e in set(ends) if ends.count(e) >= 3]
        if len(common) == 1 and abs(common[0][1]) != 139.5:
            SPOKE_NODE[which] = tuple(float(c) for c in common[0])


_spoke_nodes_from_layout()
BRACE_END = {"front": [("L2 brace lo +1", (72.95, 62.11, 30.0)), ("L2 brace hi +1", (72.95, 62.11, 39.0))]}
if os.environ.get("NOX_ARM_BRACES", "1") == "0":   # v5b: the L2 braces root on a top-plate block (cage session 27 Oct), not the arm
    BRACE_END = {}
JOINT = os.environ.get("NOX_ARM_JOINT", os.path.join(ROOT, "runs", "overseer_arms", "v2", "joint_loads.json"))

# loads (dimensions.md "System Overseer: arms", E)
THRUST_N = 21.0                       # 1650 gf at 14.8 V (C) x (16.8 / 14.8)^2 on a full pack = 2.13 kgf
YAW_NM = 0.5                          # motor reaction at full thrust on a full pack, both signs (DShot braking)
GYRO_NM = 0.2                         # prop gyroscopic moment, 5.4e-5 kg m2 x 995 rad/s x 3 rad/s + margin
TIP_KG = 0.096                        # motor 81 (M) + prop 10 + screws 5 at the motor CG
TIP_Z = 60.0                          # tip mass CG height (motor body mid 58, prop 82)
LAT_G, LAND_G, CRASH_G = 2.0, 6.0, 35.0              # dimensions L
# Spark 27 Oct: "the strength needs to be really generous because if this arm is flimsy the drone is fucked": handling
# twice L's 50 N, a direct 150 N blow on the motor end (past the cage) in 6 directions, index targets half of L's
HANDLE_N, BLOW_N = 100.0, 150.0


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(x, y, z0, z1, r, n=64):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def from_tm(t):
    from topo.part import man
    return man(t)


def motors_xy():
    return json.load(open(os.path.join(LAYOUT, "result.json")))["motors_xy"]


def frame(which):
    """drone-frame data of the left arm and the drone -> arm-frame map (rotate about z, root centre to the origin)"""
    d = MOTOR_DEG[which]
    M = np.array(motors_xy()[0 if which == "front" else 1], float)
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    R0 = 66.5 * np.array([c, s])
    th = math.degrees(math.atan2(*(M - R0)[::-1]))
    L = float(np.linalg.norm(M - R0))
    return dict(d=d, M=M, R0=R0, th=th, L=L)


def to_arm(m, F):
    return m.translate([-F["R0"][0], -F["R0"][1], 0.0]).rotate([0.0, 0.0, -F["th"]])


def to_arm_pt(p, F):
    c, s_ = math.cos(math.radians(-F["th"])), math.sin(math.radians(-F["th"]))
    x, y = p[0] - F["R0"][0], p[1] - F["R0"][1]
    return np.array([c * x - s_ * y, s_ * x + c * y, p[2]])


def to_arm_vec(v, F):
    c, s_ = math.cos(math.radians(-F["th"])), math.sin(math.radians(-F["th"]))
    return np.array([c * v[0] - s_ * v[1], s_ * v[0] + c * v[1], v[2]])


def diag(m, d):
    return m.rotate([0.0, 0.0, d])


def plate_prism(name, z0, z1):
    t = trimesh.load(os.path.join(LAYOUT, "parts", name), force="mesh")
    from scipy.spatial import ConvexHull
    p = t.vertices[:, :2]
    h = p[ConvexHull(p).vertices]
    cs = mf.CrossSection([h.tolist()])
    return mf.Manifold.extrude(cs, z1 - z0).translate([0, 0, z0])


def keepouts():
    """hub electronics (FC stack, Pi 5 + deck, PDB, UBEC, buck, RX) and the L2 keel hub, + CLEAR, as boxes (v1); with
    CORE, the core session's real part meshes grown by CLEAR (6 shifted copies)"""
    out = mf.Manifold()
    if CORE:
        import glob
        for f in sorted(glob.glob(os.path.join(CORE, "*.stl"))):
            t = trimesh.load(f, force="mesh")
            if t.bounds[1][2] <= 0.0 or t.bounds[0][0] > 300:    # under the bottom plate (sled, pack...) or the nose
                continue
            m = from_tm(t)
            for d in np.vstack([np.eye(3), -np.eye(3)]) * CLEAR:
                out += m.translate(d.tolist())
        return out
    for nm in ("hub_electronics__fc__pi_5___deck__pdb__u.stl", "apex___cam_bracket.stl"):
        t = trimesh.load(os.path.join(LAYOUT, "parts", nm), force="mesh")
        for b in t.split(only_watertight=False):
            lo, hi = b.bounds
            if lo[0] > 300:                           # the L2 head itself is far ahead of the arms
                continue
            out += box(lo[0] - CLEAR, hi[0] + CLEAR, lo[1] - CLEAR, hi[1] + CLEAR, lo[2] - CLEAR, hi[2] + CLEAR)
    return out


def cylx(x0, x1, y, z, r, n=32):
    return mf.Manifold.cylinder(x1 - x0, r, r, n).rotate([0.0, 90.0, 0.0]).translate([x0, y, z])


def chords(which, F):
    """arm-frame rods: (p0, p1) 3D end points, p0 at the shoulder (open end), p1 blind in the motor pocket wall"""
    if which not in SHOULDER:
        return []
    xs, ys, _ = SHOULDER[which]
    xt, yt = F["L"] - ROD_TIP[0], ROD_TIP[1]
    return [(np.array([xs, sy * ys, z]), np.array([xt, sy * yt, z])) for sy in (1, -1) for z in ROD_Z]


# v6 (front, 27 Oct): the chords as a space frame. v5d read 131 / 162 Hz against 190: the 4 tubes worked almost alone, the
# PETG webs did not carry the shear. 12 carbon diagonals, a Warren girder of 3 panels in each of the 4 faces (the 2 side
# faces: top + bottom chord of one side; the top and bottom faces: zigzag between the +y and -y chords). Each diagonal
# lies in a plane DIAG_OFF off its face, inboard (sides) or inside (top / bottom), so it passes beside every chord tube
# and its straight insertion line leaves the arm without crossing one; it ends DIAG_GAP short of its Warren node point
# so the other leg's insertion line passes clear. One end blind, the other a through bore (rod slides in from outside,
# epoxied). Drawn: a full-length sleeve on each diagonal (as the chords) and a printed node block at each end, the hull of
# the socket's last 10 mm and the chord sleeve beside it. Stations along the chords, node parity and the open-end
# direction of each face's chain are set here (searched by diag_geomcheck, C).
DIAG = os.environ.get("NOX_ARM_DIAG") == "1"
DIAG_T = tuple(float(v) for v in os.environ.get("NOX_ARM_DIAG_T", "0.02,0.34,0.66,0.98").split(","))
DIAG_OFF = float(os.environ.get("NOX_ARM_DIAG_OFF", 7.0))   # axis to chord axis: tube bore 3.1 + 2.0 web + rod bore 1.6 = 6.7
DIAG_GAP = float(os.environ.get("NOX_ARM_DIAG_GAP", 6.0))   # blind rod end short of its node point (the other leg's
DIAG_GAP_OPEN = float(os.environ.get("NOX_ARM_DIAG_GAP_OPEN", 0.0))   # insertion line passes it); the through end
DIAG_PAT = os.environ.get("NOX_ARM_DIAG_PAT", "0:1111")     # parity : open-end chain per face (side+, side-, top, bottom)
DIAG_KIND = os.environ.get("NOX_ARM_DIAG_KIND", "rod3")     # rod3 (3.0 rod, bore 3.2) or tube6 (6/4, bore 6.2)
DIAG_NODE_L = float(os.environ.get("NOX_ARM_DIAG_NODE_L", 10.0))      # node block: the socket's last DIAG_NODE_L hulled to
DIAG_NODE_SEG = float(os.environ.get("NOX_ARM_DIAG_NODE_SEG", 9.0))   # the chord sleeve ± DIAG_NODE_SEG beside the node point
DIAG_SOCKET_L = 18.0


def diag_spec():
    if DIAG_KIND == "tube6":
        return dict(d=6.0, bore=6.2, area=math.pi / 4 * (36.0 - 16.0), skin=2.5)
    return dict(d=3.0, bore=3.2, area=math.pi / 4 * 9.0, skin=2.65)


def diagonals(which, F):
    """v6: the 12 diagonals (arm frame): dicts name, face, chords (ia, ib), node points A / B, p_blind, p_open, u (blind
    to open), d, bore, area, skin"""
    if not DIAG or which not in SHOULDER:
        return []
    C = chords(which, F)                       # 0 T+, 1 B+, 2 T-, 3 B-
    par = int(DIAG_PAT.split(":")[0]); chain = [int(c) for c in DIAG_PAT.split(":")[1]]
    ez = np.array([0.0, 0.0, 1.0])

    def P(i, t):
        return C[i][0] + t * (C[i][1] - C[i][0])

    def n_in(i):                               # plan normal to chord i pointing inboard (towards y 0)
        u = C[i][1] - C[i][0]; n = np.array([-u[1], u[0], 0.0]); n /= np.linalg.norm(n)
        return n if n[1] * C[i][0][1] < 0 else -n
    # node parity: side+ has T+ at odd stations, B+ at even (par 0) or the reverse (par 1); the rest follows so each chord
    # station carries one face's node only
    odd, even = (1, 3), (0, 2)
    sp = (odd, even) if par == 0 else (even, odd)          # side+: (T+ stations, B+ stations)
    faces = {
        "side+": ((0, sp[0]), (1, sp[1]), lambda i: n_in(i)),
        "side-": ((2, sp[1]), (3, sp[0]), lambda i: n_in(i)),
        "top": ((0, sp[1]), (2, sp[0]), lambda i: -ez),
        "bottom": ((1, sp[0]), (3, sp[1]), lambda i: ez),
    }
    sp_ = diag_spec()
    out = []
    for fi, (fname, ((ia, sa), (ib, sb), off)) in enumerate(faces.items()):
        nodes = sorted([(s, ia) for s in sa] + [(s, ib) for s in sb])     # station order 0..3, alternating chords
        pts = [P(i, DIAG_T[s]) + DIAG_OFF * off(i) for s, i in nodes]
        for k in range(3):
            A_, B_ = pts[k], pts[k + 1]
            u = (B_ - A_) / np.linalg.norm(B_ - A_)
            open_hi = chain[fi] == 0                       # chain 0: each diagonal open at its higher station
            ga, gb = (DIAG_GAP, DIAG_GAP_OPEN) if open_hi else (DIAG_GAP_OPEN, DIAG_GAP)
            pa, pb = A_ + ga * u, B_ - gb * u
            p_open, p_blind = (pb, pa) if open_hi else (pa, pb)
            uo = (p_open - p_blind) / np.linalg.norm(p_open - p_blind)
            out.append(dict(name=f"diag {fname} {k}", face=fname, chords=(nodes[k][1], nodes[k + 1][1]), A=A_, B=B_,
                            stations=(nodes[k][0], nodes[k + 1][0]), p_blind=p_blind, p_open=p_open, u=uo, **sp_))
    return out


def diag_bodies(which, F, env):
    """v6 drawn: per diagonal a full-length sleeve, the blind end's cap, and a node block at each end (hull of the socket's
    last DIAG_NODE_L and the chord sleeve segment beside it); cuts: the bore, open past the through end to the outside"""
    drawn, cuts, ins = mf.Manifold(), mf.Manifold(), mf.Manifold()
    C = chords(which, F)
    r_ch = ROD_BORE / 2 + CHORD_SKIN
    clip = box(-50, F["L"] + 50, -60, 60, 0.0, ROOT_H)
    for D in diagonals(which, F):
        r = D["bore"] / 2 + D["skin"]
        u, pb, po = D["u"], D["p_blind"], D["p_open"]
        Ld = float(np.linalg.norm(po - pb))
        drawn += rod_m(pb - D["skin"] * u, po, r, n=40) ^ clip
        for end, s_in in ((pb, u), (po, -u)):
            near_a = np.linalg.norm(end - D["A"]) < np.linalg.norm(end - D["B"])
            node, i = (D["A"], D["chords"][0]) if near_a else (D["B"], D["chords"][1])
            c0, c1 = C[i]; uc = (c1 - c0) / np.linalg.norm(c1 - c0)
            q = c0 + float((node - c0) @ uc) * uc          # chord point beside the node
            seg = rod_m(q - DIAG_NODE_SEG * uc, q + DIAG_NODE_SEG * uc, r_ch, n=40)
            sock = rod_m(end, end + DIAG_NODE_L * s_in, r, n=40)
            if end is pb:
                sock = sock + rod_m(end - D["skin"] * u, end, r, n=40)
            drawn += (seg + sock).hull() ^ clip
        cuts += rod_m(pb, po, D["bore"] / 2, n=32) + rod_m(po, po + 80.0 * u, D["bore"] / 2, n=32)
        ins += rod_m(pb, po, D["d"] / 2)
    return drawn ^ env, cuts, ins


def rod_m(p0, p1, r, back=0.0, n=32):
    """cylinder of radius r from p0 - back along the axis to p1"""
    d = p1 - p0
    L = float(np.linalg.norm(d))
    u = d / L
    yaw = math.degrees(math.atan2(u[1], u[0]))
    pitch = math.degrees(math.atan2(-u[2], math.hypot(u[0], u[1])))
    c = mf.Manifold.cylinder(L + back, r, r, n).rotate([0.0, 90.0, 0.0]).translate([-back, 0.0, 0.0])
    return c.rotate([0.0, pitch, 0.0]).rotate([0.0, 0.0, yaw]).translate(p0.tolist())


def bodies(which):
    """everything in the drone frame (left arm), mapped to the arm frame; then the chords, in the arm frame"""
    F = frame(which)
    d, M = F["d"], F["M"]
    u0, u1 = U_ROOT
    root = diag(box(u0, u1, -V_ROOT, V_ROOT, 0.0, ROOT_H), d)
    top_fp = plate_prism("top_plate.stl", -5.0, ROOT_H + 5.0)
    bot_fp = plate_prism("bottom_plate.stl", -5.0, ROOT_H + 5.0)
    ko = keepouts()

    # envelope: hull of the root block and a Ø40 disc at the motor, z 0 .. 45
    tip = cyl(M[0], M[1], 0.0, ROOT_H, ENV_R_TIP, 96)
    env = (root + tip).hull() - ko

    # drawn: plate contact pads with insert bosses (held by the plates), motor pad with the pocket wall
    pad_top = (diag(box(u0, u1, -V_ROOT, V_ROOT, ROOT_H - ROOT_PAD_T, ROOT_H), d) ^ top_fp)
    pad_bot = (diag(box(u0, u1, -V_ROOT, V_ROOT, 0.0, ROOT_PAD_T), d) ^ bot_fp)
    for u, v in TOP_INS:
        pad_top += diag(cyl(u, v, ROOT_H - ROOT_PAD_T, ROOT_H, BOSS_D / 2), d)
    for u, v in BOT_INS:
        pad_bot += diag(cyl(u, v, 0.0, ROOT_PAD_T, BOSS_D / 2), d)
    pad_top -= ko
    pad_bot -= ko
    mpad = cyl(M[0], M[1], ROOT_H - PAD_T, ROOT_H, MOTOR_R_PAD, 96)
    mwall = cyl(M[0], M[1], ROOT_H - PAD_T - POCKET_H, ROOT_H - PAD_T, POCKET_R + POCKET_WALL, 96)

    # cuts
    cuts = ko
    bosses = mf.Manifold()
    for u, v in TOP_INS:
        cuts += diag(cyl(u, v, ROOT_H - INS_DEPTH, ROOT_H + 1, INS_D / 2, 32), d)
        bosses += diag(cyl(u, v, ROOT_H - INS_DEPTH, ROOT_H, INS_D / 2 + 1.5, 32), d)
    for u, v in BOT_INS:
        cuts += diag(cyl(u, v, -1.0, INS_DEPTH, INS_D / 2, 32), d)
        bosses += diag(cyl(u, v, 0.0, INS_DEPTH, INS_D / 2 + 1.5, 32), d)
    lead = math.degrees(math.atan2(*(F["R0"] - M)[::-1]))          # motor leads point back to the root
    for ang in MOTOR_SLOT_DEG:
        aa = math.radians(lead + ang)
        ends = [cyl(M[0] + r * math.cos(aa), M[1] + r * math.sin(aa), ROOT_H - PAD_T - 1, ROOT_H + 1, MOTOR_HOLE_D / 2, 24)
                for r in MOTOR_SLOTS]
        cuts += (ends[0] + ends[1]).hull()
    cuts += cyl(M[0], M[1], ROOT_H - PAD_T - 1, ROOT_H + 1, SHAFT_D / 2, 48)
    cuts += cyl(M[0], M[1], -1.0, ROOT_H - PAD_T, POCKET_R, 96)       # screw heads + key from below, open at the bottom

    # support faces: the plate contact faces of the root pads (inside each plate's outline)
    sup_top = diag(box(u0, u1, -V_ROOT, V_ROOT, ROOT_H - 1.0, ROOT_H + 0.5), d) ^ top_fp
    sup_bot = diag(box(u0, u1, -V_ROOT, V_ROOT, -0.5, 1.0), d) ^ bot_fp
    sup = (sup_top + sup_bot) - ko
    load = mpad - cyl(M[0], M[1], ROOT_H - PAD_T - 1, ROOT_H + 1, SHAFT_D / 2 + 0.5, 48)
    socks, sock_regions = mf.Manifold(), {}
    if LINKS:
        res = json.load(open(os.path.join(LAYOUT, "result.json")))["links"]
        ends = []
        node = np.array(SPOKE_NODE[which])
        tag = "1" if which == "front" else "2"
        for k in sorted(res):
            if k.startswith(f"spoke {tag}."):
                p_, q_ = map(np.array, res[k])
                far = q_ if np.linalg.norm(q_ - node) > np.linalg.norm(p_ - node) else p_   # the end away from the node
                ends.append((k, node, far))
        for k, e in BRACE_END.get(which, []):
            p_, q_ = map(np.array, res[k])
            far = p_ if p_[0] > q_[0] else q_
            ends.append((k, np.array(e), far))
        r_b = TUBE_BORE / 2 + TUBE_WALL
        for k, a, far in ends:
            u = (far - a) / np.linalg.norm(far - a)
            mouth = a + (TUBE_DEPTH + TUBE_LEAD) * u
            boss = rod_m(a - TUBE_WALL * u, mouth, r_b, n=40) + mf.Manifold.sphere(r_b, 32).translate(a.tolist())
            socks += boss
            cuts += rod_m(a, mouth + 30.0 * u, TUBE_BORE / 2, n=32)          # the bore, open past the mouth
            sock_regions[k] = (boss, a, u)
        # the sockets join the envelope after the shoulder hull (below), with the node and the tip hulled together
        link_env = socks + (mf.Manifold.sphere(r_b + 4.0, 32).translate(node.tolist()) + tip).hull()
        if which == "rear":
            link_env = link_env + socks.hull()
        cuts += cyl(M[0], M[1], -80.0, -1.0, 14.0, 64)                   # the key column under the pocket stays clear
        clamp = box(-400, 400, -400, 400, ROOT_H, ROOT_H + 100) + plate_prism("bottom_plate.stl", -100.0, 0.0)
    B = {k: to_arm(v, F) for k, v in dict(env=env, pads=pad_top + pad_bot, mpad=mpad + mwall + socks, cuts=cuts, sup=sup,
                                         bosses=bosses, load=load, ko=ko).items()}
    B["sock"] = {k: (to_arm(m_, F), (np.r_[to_arm_pt(a_, F)]), to_arm_vec(u_, F)) for k, (m_, a_, u_) in sock_regions.items()}
    B["link_env"] = to_arm(link_env, F) if LINKS else None
    B["clamp"] = to_arm(clamp, F) if LINKS else None
    if which in SHOULDER:                       # the plan envelope widens to the two shoulders (arm frame)
        xs, ys, rs = SHOULDER[which]
        sh = cyl(xs, ys, 0.0, ROOT_H, rs) + cyl(xs, -ys, 0.0, ROOT_H, rs)
        B["env"] = (B["env"] + sh).hull() - B["ko"]
    if B["link_env"] is not None:
        B["env"] = B["env"] + B["link_env"] - B["ko"] - B["clamp"]

    # chords (arm frame): the rod is an insert (Part.insert: solid at the carbon's modulus in the layout and the strength
    # FE, never printed); drawn only a socket at each end (review 2, 27 Oct); the optimiser hangs its webs on the free span,
    # and the bore (open past the shoulder) cuts a collar through every web it meets: epoxy at the sockets and collars
    rods = mf.Manifold()
    sockets = mf.Manifold()
    r = ROD_BORE / 2 + CHORD_SKIN
    for p0, p1 in chords(which, F):
        rods += rod_m(p0, p1, ROD_D / 2)
        B["cuts"] += rod_m(p0, p1, ROD_BORE / 2, back=30.0)
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        cap = CHORD_SKIN if os.environ.get("NOX_ARM_CAP") == "1" else 1.0   # v5d: the blind tip end's wall (rear v5c: 1.58)
        for q0, q1 in ((p0, p0 + SOCKET_L * u), (p1 - SOCKET_L * u, p1 + cap * u)):
            sockets += rod_m(q0, q1, r, n=48) ^ box(-50, F["L"] + 50, -60, 60, 0.0, ROOT_H)
        ncol = int(os.environ.get("NOX_ARM_COLLARS", "-1"))   # v8 (Spark 27 Oct: lighter, natural): drawn collar rings on
        if ncol >= 0:                                         # the free span, the rest of a 2.5 ring round the bore kept clear
            col = mf.Manifold()                               # (so a web meets the tube only at a full collar, no thin walls)
            L_ = float(np.linalg.norm(p1 - p0)); a0, a1 = SOCKET_L, L_ - SOCKET_L
            for k_ in range(ncol):
                c_ = a0 + (a1 - a0) * (k_ + 1) / (ncol + 1)
                col += rod_m(p0 + (c_ - 4.0) * u, p0 + (c_ + 4.0) * u, r, n=48)
            sockets += col ^ box(-50, F["L"] + 50, -60, 60, 0.0, ROOT_H)
            B["cuts"] += rod_m(p0 + a0 * u, p0 + a1 * u, r, n=48) - col
        if os.environ.get("NOX_ARM_SLEEVE") == "1":   # v5c: a full-length drawn sleeve (rear v5b: webs meeting the tube
            sockets += rod_m(p0, p1, r, n=48) ^ box(-50, F["L"] + 50, -60, 60, 0.0, ROOT_H)   # left 1.1 collar walls)
    B["cuts_base"] = B["cuts"]
    if DIAG:                                    # v6: the diagonals, their sleeves and node blocks
        d_drawn, d_cuts, d_ins = diag_bodies(which, F, B["env"])
        sockets += d_drawn
        B["cuts"] += d_cuts
        rods += d_ins
        B["diag_drawn"] = d_drawn
    B["rods"] = rods
    B["drawn"] = B["pads"] + B["mpad"] + sockets
    if os.environ.get("NOX_ARM_ROOT_SOLID") == "1":      # front v3: the root block solid between the plates (front v2: its open
        rb = to_arm(diag(box(U_ROOT[0], U_ROOT[1], -V_ROOT, V_ROOT, 0.0, ROOT_H), F["d"]), F) ^ B["env"]   # middle read 1.6 in flight)
        B["drawn"] = B["drawn"] + (rb - B["cuts"])
    fx = os.environ.get("NOX_ARM_FLANGE_X")              # v4: drawn root flanges, top and bottom, out along the arm (arm frame)
    if fx:
        x0, x1 = 0.0, float(fx)
        fl = (box(x0, x1, -20.0, 20.0, 0.0, FLANGE_T) + box(x0, x1, -20.0, 20.0, ROOT_H - FLANGE_T, ROOT_H)) ^ B["env"]
        B["drawn"] = B["drawn"] + (fl - B["cuts"])
    if os.environ.get("NOX_ARM_SHFLANGE") == "1" and which in SHOULDER:   # v5c: the top / bottom flange spans root to the
        xs, ys, _ = SHOULDER[which]                     # shoulder sockets (rear v5b: crash 1.47 where the socket hung off
        rb = to_arm(diag(box(U_ROOT[0], U_ROOT[1], -V_ROOT, V_ROOT, 0.0, ROOT_H), F["d"]), F)   # the flange corner)
        for z0, z1 in ((0.0, FLANGE_T), (ROOT_H - FLANGE_T, ROOT_H)):
            band = box(-400, 400, -400, 400, z0, z1)
            shf = ((rb ^ band) + cyl(xs, ys, z0, z1, r) + cyl(xs, -ys, z0, z1, r)).hull() ^ B["env"]
            B["drawn"] = B["drawn"] + (shf - B["cuts"])
    shw = os.environ.get("NOX_ARM_SHWEB")                # v6: the shoulder closed full depth (the modal what-if: the x 5..25
    B["shweb"] = mf.Manifold()                           # band, root block to the chord ends, was the soft spot)
    if shw and which in SHOULDER:
        xs, ys, _ = SHOULDER[which]
        rb = to_arm(diag(box(U_ROOT[0], U_ROOT[1], -V_ROOT, V_ROOT, 0.0, ROOT_H), F["d"]), F)
        for sy in (1, -1):
            post = cyl(xs, sy * ys, 0.0, ROOT_H, r)      # a post joining the top and bottom chord sockets
            if shw == "solid":                           # the wedge from the root block's half to the post
                w = (post + (rb ^ box(-400, 400, min(0, 400 * sy), max(0, 400 * sy), 0, ROOT_H))).hull()
            else:                                        # walls t thick from the post into the root block: to its nearest
                tri = shw.startswith("tri")              # corner ("<t>"), or to both corners of its near side ("tri<t>")
                t_ = float(shw[3:] if tri else shw)
                v = trimesh_vertices(rb ^ box(-400, 400, min(0, 400 * sy), max(0, 400 * sy), 0, ROOT_H))[:, :2]
                v = np.unique(v.round(3), axis=0)
                dd = np.linalg.norm(v - [xs, sy * ys], axis=1)
                outer = v[np.argsort(-sy * v[:, 1])[:4]]          # the near side's corners (largest |y| on this side)
                cs_ = [v[np.argmin(dd)]] + ([outer[np.argmax(np.linalg.norm(outer - v[np.argmin(dd)], axis=1))]] if tri else [])
                w = post
                for c_ in cs_:
                    e_ = c_ + 8.0 * (c_ - [xs, sy * ys]) / np.linalg.norm(c_ - [xs, sy * ys])   # 8 into the block
                    w = w + (cyl(xs, sy * ys, 0.0, ROOT_H, t_ / 2) + cyl(e_[0], e_[1], 0.0, ROOT_H, t_ / 2)).hull()
            B["shweb"] += (w ^ B["env"]) - B["cuts"]
        B["drawn"] = B["drawn"] + B["shweb"]
    return B, F


def trimesh_vertices(m):
    return m.to_mesh().vert_properties[:, :3]


def rigid_case(p, name, parts, cls="flight", accel=None, support="default"):
    """parts: [(region, point, force, moment)], each carried by its region's nodes as a rigid link: every node gets
    F / n + (J^-1 M_c) x r, so the resultant and the moment about the node centroid are exact"""
    from topo.part import LoadCase
    Fv = np.zeros(p.ndof)
    alln = []
    for region, point, force, moment in parts:
        el = p.elems(region, p.solid | p.design, what=f"load {name}")
        nd = p.nodes_of(el)
        i, j, k = np.unravel_index(nd, p.nshape)
        X = np.stack([p.origin[0] + i * p.h, p.origin[1] + j * p.h, p.origin[2] + k * p.h], 1)
        c = X.mean(0)
        r = X - c
        f = np.asarray(force, float)
        Mc = np.asarray(moment, float) + np.cross(np.asarray(point, float) - c, f)
        J = np.eye(3) * (r * r).sum() - r.T @ r
        w = np.linalg.solve(J, Mc)
        fn = f / len(nd) + np.cross(w, r)
        for a in range(3):
            Fv[3 * nd + a] += fn[:, a]
        alln.append(nd)
    p.cases.append(LoadCase(name, Fv, cls=cls, support=support,
                            accel=None if accel is None else np.asarray(accel, float), nodes=np.unique(np.concatenate(alln))))


def setup_for(which):
    from topo import Part

    def setup(h):
        B, F = bodies(which)
        p = Part(f"overseer_arm_{which}", h, B["drawn"], B["cuts"], B["env"], build="-z", material="PETG")
        for k_, (p0, p1) in enumerate(chords(which, F)):
            rr = max(ROD_BORE / 2, 0.9 * h)              # a fat enough cylinder to stay connected on the coarse grid
            p.insert(f"rod {k_}", rod_m(p0, p1, rr), E_CARBON / E_PETG, area_mm2=ROD_AREA,
                     length_mm=float(np.linalg.norm(p1 - p0)))
        for D in diagonals(which, F):                   # v6: the diagonals, each at its own section
            rr = max(D["bore"] / 2, 0.9 * h)
            p.insert(D["name"], rod_m(D["p_blind"], D["p_open"], rr), E_CARBON / E_PETG, area_mm2=D["area"],
                     length_mm=float(np.linalg.norm(D["p_open"] - D["p_blind"])))
        p.support("default", B["sup"])
        p.support("bosses", B["bosses"])        # review 27 Oct: the joint as 4 inserts only (no face contact)
        tipc = np.array([F["L"], 0.0, TIP_Z])
        W = TIP_KG * G
        ex, ey, ez = np.eye(3)
        Z = np.zeros(3)

        def masses(gvec, extra_f=Z, extra_m=Z):
            return [(B["load"], tipc, W * gvec + extra_f, extra_m)]
        # flight: max thrust with the yaw reaction both ways; with 2 g lateral and the gyro moment in 4 directions
        for sgn, nm in ((1, "+"), (-1, "-")):
            rigid_case(p, f"max thrust {THRUST_N:.0f} N + yaw {nm}{YAW_NM} Nm", masses(Z, THRUST_N * ez, sgn * YAW_NM * ez))
        for v, nm, gax in ((ex, "+x radial", ey), (-ex, "-x radial", -ey), (ey, "+y side", ex), (-ey, "-y side", -ex)):
            rigid_case(p, f"max thrust + {LAT_G:.0f} g {nm} + gyro", masses(LAT_G * v, THRUST_N * ez, GYRO_NM * gax + YAW_NM * ez),
                       accel=LAT_G * v)
        rigid_case(p, f"landing {LAND_G:.0f} g", masses(-LAND_G * ez), accel=-LAND_G * ez)
        # crash: the motor's own inertia at 35 g in 14 directions (the cage stops the drone)
        dirs = [ex, -ex, ey, -ey, ez, -ez] + [np.array((a, b, c_)) / math.sqrt(3) for a in (1, -1) for b in (1, -1)
                                               for c_ in (1, -1)]
        for k_, v in enumerate(dirs):
            rigid_case(p, f"crash {CRASH_G:.0f} g dir {k_}", masses(CRASH_G * v), cls="crash", accel=CRASH_G * v)
        # handling: 50 N on the motor pad, 6 directions
        padc = np.array([F["L"], 0.0, ROOT_H])
        for v, nm in ((ex, "+x"), (-ex, "-x"), (ey, "+y"), (-ey, "-y"), (ez, "+z"), (-ez, "-z")):
            rigid_case(p, f"handling {HANDLE_N:.0f} N {nm}", [(B["load"], padc, HANDLE_N * v, Z)], cls="misuse")
        # direct blow on the motor end (something gets past the cage), 6 directions
        for v, nm in ((ex, "+x"), (-ex, "-x"), (ey, "+y"), (-ey, "-y"), (ez, "+z"), (-ez, "-z")):
            rigid_case(p, f"blow {BLOW_N:.0f} N {nm}", [(B["load"], tipc, BLOW_N * v, Z)], cls="crash")
        # cage joint reactions (v2): the frame model's force on the arm at each link group, per case (crash 35 g x 14,
        # flight 4 g), on the socket bosses; braces split evenly between their 2 sockets (E)
        if LINKS:
            J = json.load(open(JOINT))
            J = J["forces"] if "forces" in J else J["cases"]            # v2 joint_loads.json or the cage's arm_loads.json
            tag = "arm 1" if which == "front" else "arm 2"
            tkey = f"{tag} tip" if f"{tag} tip" in J else f"{tag} spoke node"
            for cn in J[tkey]:
                parts = []
                tipF = to_arm_vec(np.array(J[tkey][cn]), F)
                spokes = [(m_, a_) for k, (m_, a_, u_) in B["sock"].items() if k.startswith("spoke")]
                for m_, a_ in spokes:
                    parts.append((m_, a_, tipF / len(spokes), Z))
                if which == "front" and BRACE_END:
                    bF = to_arm_vec(np.array(J["arm 1 beam (L2 braces)"][cn]), F)
                    for k, (m_, a_, u_) in B["sock"].items():
                        if k.startswith("L2 brace"):
                            parts.append((m_, a_, bF / 2, Z))
                rigid_case(p, f"cage {cn}", parts, cls="flight" if cn.startswith("flight") else "crash")
        # the same joint held by the 4 inserts only (review 27 Oct): the worst-looking cases again
        rigid_case(p, "max thrust + yaw [inserts only]", masses(Z, THRUST_N * ez, YAW_NM * ez), support="bosses")
        for v, nm in ((ez, "+z"), (-ez, "-z"), (ey, "+y"), (-ey, "-y")):
            rigid_case(p, f"handling {HANDLE_N:.0f} N {nm} [inserts only]", [(B["load"], padc, HANDLE_N * v, Z)], cls="misuse",
                       support="bosses")
        return p
    return setup


# ------------------------------------------------------------------------------------------------- placement
def arm_to_drone(t, F, mirror=False):
    t = t.copy()
    T = trimesh.transformations.rotation_matrix(math.radians(F["th"]), [0, 0, 1])
    T[:2, 3] = F["R0"]
    t.apply_transform(T)
    if mirror:
        t.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
    return t


# ------------------------------------------------------------------------------------------------- first modes
def all_rods(which, F):
    """every carbon insert (arm frame): (p0, p1, d, area); v6 adds the diagonals (their own d and area)"""
    out = [(p0, p1, ROD_D, ROD_AREA) for p0, p1 in chords(which, F)]
    out += [(D["p_blind"], D["p_open"], D["d"], D["area"]) for D in diagonals(which, F)]
    return out


def modes(which, mesh_path, h=1.0, n=4, support="faces", log=print, extra=None):
    """first modes (Hz, E) of the built arm, fixed at the two plate faces, with the motor + prop (96 g) as a stiff body
    on the pad (the ESC is in the hub, Spark 27 Oct). Hex8 voxel FE at pitch h, PETG transversely
    isotropic (build z), lumped masses, shift-invert Lanczos."""
    import scipy.sparse as sp
    import scipy.sparse.linalg as sla
    from topo.part import voxelise
    from topo.fem import hex_ke
    from topo.material import MATERIALS, stiffness
    B, F = bodies(which)
    arm = trimesh.load(mesh_path, force="mesh")
    lo, hi = arm.bounds
    lo = np.minimum(lo, [F["L"] - 15, -15, 0]) - h
    hi = np.maximum(hi, [F["L"] + 15, 15, 76]) + h
    ax = [lo[k] + (np.arange(int(math.ceil((hi[k] - lo[k]) / h))) + 0.5) * h for k in range(3)]
    shape = tuple(len(a) for a in ax)
    solid = voxelise(arm, *ax)
    X, Y, Z = np.meshgrid(*ax, indexing="ij")
    motor = ((X - F["L"]) ** 2 + Y ** 2 <= 13.85 ** 2) & (Z > ROOT_H) & (Z < 75.0)
    if extra is not None:                                        # v6 what-if: drawn PETG added to a built arm
        solid |= from_tm_vox(extra, ax, h)
    rod = np.zeros(shape, bool)
    rod_k = np.zeros(shape)                                      # per-cell carbon multiplier (each rod's own E A)
    P = np.stack([X, Y, Z], -1)
    for p0, p1, dd, area in all_rods(which, F):
        d = p1 - p0; Ln = np.linalg.norm(d); u = d / Ln
        t = (P - p0) @ u
        rv = max(dd / 2, 1.0 * h) if abs(u[0]) < 0.999 else dd / 2   # a slanted thin rod: fat enough to stay face-connected
        m_ = (np.linalg.norm(P - p0 - t[..., None] * u, axis=-1) <= rv) & (t >= 0) & (t <= Ln)
        if m_.any():
            rod_k[m_] += area * Ln / (m_.sum() * h ** 3)
        rod |= m_
    solid &= ~rod
    mat = MATERIALS["PETG"]
    C = stiffness(mat, 2)
    KE, _ = hex_ke(h, C)
    from scipy import ndimage as ndi
    on3 = solid | motor | rod
    lab, nlab = ndi.label(on3, np.ones((3, 3, 3), bool))
    big = np.argmax(np.bincount(lab.ravel())[1:]) + 1
    on = (lab == big).ravel()                                    # the connected arm (voxel specks dropped)
    # rods: carbon along x; the voxel cross-section scaled to the real Ø3 area
    area_fix = rod_k.ravel()
    Ee = np.where(motor.ravel(), 50.0, np.where(rod.ravel(), E_CARBON / mat.E * area_fix, 1.0))[on]   # motor + prop stiff
    rho = mat.density * 1e-3 * h ** 3 * 1e-3                    # kg per PETG cell
    me = np.where(motor.ravel(), 0.0, np.where(rod.ravel(), rho * RHO_CARBON / mat.density * area_fix, rho))[on]
    el = np.where(on)[0]
    ex, ey, ez = np.unravel_index(el, shape)
    nyz = (shape[1] + 1) * (shape[2] + 1)
    nid = lambda i, j, k: (i * (shape[1] + 1) + j) * (shape[2] + 1) + k
    en = np.stack([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)], 1)
    used, inv = np.unique(en, return_inverse=True)
    en = inv.reshape(en.shape)
    nn = len(used)
    edof = (3 * en[:, :, None] + np.arange(3)).reshape(len(el), 24)
    rows = np.repeat(edof, 24, 1).ravel(); cols = np.tile(edof, (1, 24)).ravel()
    K = sp.csr_matrix(((KE.ravel()[None, :] * Ee[:, None]).ravel() * 1e3, (rows, cols)), shape=(3 * nn, 3 * nn))  # N/m
    mn = np.zeros(nn)
    np.add.at(mn, en.ravel(), np.repeat(me / 8.0, 8))
    mot_cells = motor.ravel()[on]
    mm = 0.096 / max(mot_cells.sum(), 1)                         # motor + prop + screws on the motor cells (M spec, E)
    np.add.at(mn, en[mot_cells].ravel(), mm / 8.0)
    xm = ax[0][ex]
    # supports: the plate faces (support="faces") or the 4 insert bores only ("inserts"), as cells within 1.5 h
    if support == "faces":
        reg = B["sup"]
        reg = reg + reg.translate([0, 0, h]) + reg.translate([0, 0, -h])
    else:
        reg = B["bosses"]
    sup = from_tm_vox(reg, ax, h)
    fix_el = np.where(sup.ravel()[on])[0]
    fixed = np.zeros(3 * nn, bool)
    for e in fix_el:
        for a in range(8):
            fixed[3 * en[e, a]:3 * en[e, a] + 3] = True
    free = np.where(~fixed)[0]
    M = sp.diags(np.repeat(mn, 3)).tocsr()
    Kf = K[free][:, free]; Mf = M[free][:, free]
    import topo.fem  # noqa: F401  (sets the MKL path for pypardiso)
    import pypardiso
    Kf = sp.csr_matrix(Kf)
    ps = pypardiso.PyPardisoSolver()
    ps.set_matrix_type(11)
    ps.factorize(Kf)
    OPinv = sla.LinearOperator(Kf.shape, matvec=lambda b: ps.solve(Kf, np.asarray(b, float).ravel()), dtype=float)
    w2, V = sla.eigsh(Kf, k=n, M=Mf, sigma=0.0, which="LM", OPinv=OPinv)
    ps.free_memory(everything=True)
    f = np.sqrt(np.abs(w2)) / (2 * math.pi)
    out = []
    for q in np.argsort(f):
        U = np.zeros(3 * nn); U[free] = V[:, q]
        Un = U.reshape(-1, 3)
        dom = np.abs(Un).sum(0)
        kind = ["fore-aft / axial", "lateral (in-plane)", "vertical"][int(np.argmax(dom))]
        out.append(dict(Hz=round(float(f[q]), 1), kind=kind))
    mass = me.sum() * 1000
    log(f"modes {which} ({support}): {[(o['Hz'], o['kind']) for o in out]}; arm + rods {mass:.1f} g on the FE grid (h {h})")
    return dict(modes=out, arm_g_grid=round(float(mass), 1), h=h, support=support)


def seg_dist(a0, a1, b0, b1, step=0.25):
    """min distance between segments a0a1 and b0b1 (sampled along a, exact to b)"""
    n = max(int(np.linalg.norm(a1 - a0) / step), 2)
    P = a0[None] + np.linspace(0, 1, n)[:, None] * (a1 - a0)[None]
    d = b1 - b0; L2 = float(d @ d)
    t = np.clip((P - b0) @ d / L2, 0, 1)
    return float(np.min(np.linalg.norm(P - b0 - t[:, None] * d, axis=1)))


def diag_geomcheck(which, B=None, F=None, log=print, wall=2.0):
    """v6: the diagonals on the setup geometry (C): (1) every rod clear of every other rod and chord tube by a 2.0 web
    (bore to bore); (2) each insertion line, from the through end out to where it leaves the envelope, clear of every
    other rod and tube (2.0 web inside the envelope), of the motor pad, its pocket wall, pocket, slots, shaft bore, the
    spoke node sockets and their bores, the root pads and insert holes (its own bore + 2.0 must not touch them)"""
    if B is None:
        B, F = bodies(which)
    D = diagonals(which, F)
    C = chords(which, F)
    rods = [(d_["name"], d_["p_blind"], d_["p_open"], d_["bore"] / 2) for d_ in D]
    rods += [(f"chord {k}", p0 - 30.0 * (p1 - p0) / np.linalg.norm(p1 - p0), p1, ROD_BORE / 2) for k, (p0, p1) in enumerate(C)]
    from topo.part import tm
    env_t = tm(B["env"])
    hard = B["cuts_base"] - B["ko"]
    hard = hard + B["mpad"] + B["bosses"]
    root_pads = B["pads"]
    out, ok_all = [], True
    for d_ in D:
        r = d_["bore"] / 2
        u, po = d_["u"], d_["p_open"]
        # where the insertion line leaves the envelope (0.5 mm steps)
        ss = np.arange(0.0, 120.0, 0.5)
        inside = env_t.contains(po[None] + ss[:, None] * u[None])
        s_exit = float(ss[inside].max() + 0.5) if inside.any() else 0.0
        p_ex = po + (s_exit + 2.0) * u
        worst_rod = (1e9, None)
        for nm, a0, a1, rr in rods:
            if nm == d_["name"]:
                continue
            c1 = seg_dist(d_["p_blind"], p_ex, a0, a1) - r - rr
            c2 = seg_dist(p_ex, po + 120.0 * u, a0, a1) - r - rr
            c = min(c1 - wall, c2 - 0.2)
            if c < worst_rod[0]:
                worst_rod = (c + wall, nm)
        corr = rod_m(d_["p_blind"] - d_["skin"] * u, p_ex, r + wall, n=24)    # the rod's own bore and its way in
        v_hard = (corr ^ hard).volume()
        v_pads = (corr ^ root_pads).volume()
        ok = worst_rod[0] >= wall and v_hard < 0.5 and v_pads < 0.5
        ok_all &= ok
        out.append(dict(diag=d_["name"], length_mm=round(float(np.linalg.norm(d_["p_open"] - d_["p_blind"])), 1),
                        exit_mm=round(s_exit, 1), web_to_nearest_mm=round(worst_rod[0], 2), nearest=worst_rod[1],
                        hits_pad_pocket_slots_sockets_mm3=round(v_hard, 1), hits_root_pads_mm3=round(v_pads, 1), ok=bool(ok)))
        log(f"{d_['name']}: {out[-1]['length_mm']} long, leaves the envelope {s_exit:.1f} past its open end, web "
            f"{worst_rod[0]:.2f} to {worst_rod[1]}, hits pad/pocket/slots/sockets {v_hard:.1f} mm3, root pads {v_pads:.1f} mm3"
            f" -> {'OK' if ok else 'FAIL'}")
    return dict(ok=bool(ok_all), diagonals=out)


def rodcheck(which, mesh_path, log=print):
    """can the rods go in and are they held, on a built arm (arm frame): (1) the bore is clear for a 3.0 rod from 160
    behind the shoulder to the blind end (slides in straight); (2) both end sockets grip (PETG round >= 80 % of the
    18 mm x 2.65 wall); (3) bonded length along each rod (PETG round the bore at >= 6 of 8 points, r 2.4) and the collars.
    v6: the diagonals too, each from 100 outside its through end along its axis to its blind end (their geometry: also
    diag_geomcheck; on the setup before a run: --rodcheck-setup, the drawn bodies minus the cuts as the mesh)"""
    from topo.part import voxelise
    arm = trimesh.load(mesh_path, force="mesh")
    m = from_tm(arm)
    VP = 0.5                                                   # point test on a 0.5 mm voxel copy (contains() on the
    vlo = arm.bounds[0] - VP                                   # 600k-face mesh ran out of memory)
    ax = [vlo[k] + (np.arange(int((arm.bounds[1][k] - vlo[k]) / VP) + 2) + 0.5) * VP for k in range(3)]
    V = voxelise(arm, *ax)
    out = []
    F = frame(which)
    # (name, open end, blind end, rod d, bore, skin, socket length, run-in behind the open end); v6 adds the diagonals,
    # each slid in along its own axis from outside the arm through its through end
    rl = [(k, p0, p1, ROD_D, ROD_BORE, CHORD_SKIN, SOCKET_L, 160.0) for k, (p0, p1) in enumerate(chords(which, F))]
    rl += [(D["name"], D["p_open"], D["p_blind"], D["d"], D["bore"], D["skin"], DIAG_SOCKET_L, 100.0)
           for D in diagonals(which, F)]
    for k, p0, p1, rd, rbore, rskin, sl, run_in in rl:
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        L = float(np.linalg.norm(p1 - p0))
        block = (m ^ rod_m(p0 - run_in * u, p1, rd / 2)).volume()
        r_in, r_out = rbore / 2, rbore / 2 + rskin
        grip = []
        for q0, q1 in ((p0, p0 + min(sl, L / 2) * u), (p1 - min(sl, L / 2) * u, p1)):
            ann = rod_m(q0, q1, r_out) - rod_m(q0 - u, q1 + u, r_in)
            grip.append(round((m ^ ann).volume() / ann.volume(), 2))
        a = np.cross(u, [0, 0, 1.0]); a /= np.linalg.norm(a); b = np.cross(u, a)
        ang = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        ts = np.arange(0.5, L, 1.0)
        rs = 2.4 if rbore < 4.8 or isinstance(k, int) else rbore / 2 + 0.8      # chords keep the v5 probe radius
        P = (p0[None, None] + ts[:, None, None] * u + rs * (np.cos(ang)[None, :, None] * a + np.sin(ang)[None, :, None] * b))
        idx = np.floor((P.reshape(-1, 3) - vlo) / VP).astype(int)
        okk = np.all((idx >= 0) & (idx < np.array(V.shape)), 1)
        hit = np.zeros(len(idx), bool)
        hit[okk] = V[idx[okk, 0], idx[okk, 1], idx[okk, 2]]
        inside = hit.reshape(len(ts), 8).sum(1) >= 6
        collars = int(np.sum(np.diff(np.r_[0, inside.astype(int)]) == 1))
        ok = block < 1.0 and min(grip) >= 0.8
        out.append(dict(rod=k, length_mm=round(L, 1), bore_blocked_mm3=round(block, 1), socket_grip=grip,
                        bonded_mm=int(inside.sum()), collars=collars, ok=bool(ok)))
        log(f"{'rod ' if isinstance(k, int) else ''}{k}: {L:.1f} long, bore blocked {block:.1f} mm3, socket grip {grip}, bonded {int(inside.sum())} mm in "
            f"{collars} collars -> {'OK' if ok else 'FAIL'}")
    return out


def from_tm_vox(m, ax, h):
    from topo.part import tm, voxelise
    return voxelise(tm(m), *ax)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("which", choices=("front", "rear"))
    ap.add_argument("--gen", type=float, default=None)
    ap.add_argument("--h", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--git", action="store_true")
    ap.add_argument("--setup", action="store_true")
    ap.add_argument("--modes", default=None)
    ap.add_argument("--place", default=None)
    ap.add_argument("--rodcheck", default=None)
    ap.add_argument("--rodcheck-setup", action="store_true", help="v6: rodcheck + diagonal geometry on drawn - cuts")
    a = ap.parse_args()
    h = a.h or (2.0 if a.which == "front" else 1.5)   # ~ min member / 3 (topo README); direct solver (27 Oct: h 1.5 / 1.0 fell to multigrid, ~10 min an iteration)
    gen = a.gen or (40.0 if a.which == "front" else 25.0)   # cm3 generated (E), generous (Spark 27 Oct); front rods carry the chords
    out = a.out or os.path.join(ROOT, "runs", f"overseer_arm_{a.which}", VERSION)
    if a.setup:
        p = setup_for(a.which)(h)
        s = p.summary(); s["cases"] = len(s["cases"]); print(json.dumps(s), frame(a.which))
    elif a.modes:
        r = {sp_: modes(a.which, a.modes, support=sp_) for sp_ in ("faces", "inserts")}
        os.makedirs(out, exist_ok=True)
        json.dump(r, open(os.path.join(out, "modes.json"), "w"), indent=1)
    elif a.rodcheck_setup:
        from topo.part import tm
        B, F = bodies(a.which)
        os.makedirs(out, exist_ok=True)
        proxy = os.path.join(out, "setup_proxy.stl")
        tm(B["drawn"] - B["cuts"]).export(proxy)
        r = dict(rods=rodcheck(a.which, proxy), geometry=diag_geomcheck(a.which, B, F))
        r["ok"] = bool(all(x["ok"] for x in r["rods"]) and r["geometry"]["ok"])
        print("rodcheck on the setup (drawn - cuts):", "PASS" if r["ok"] else "FAIL")
        json.dump(r, open(os.path.join(out, "rodcheck_setup.json"), "w"), indent=1)
        os.remove(proxy)
    elif a.rodcheck:
        r = rodcheck(a.which, a.rodcheck)
        os.makedirs(out, exist_ok=True)
        json.dump(r, open(os.path.join(out, "rodcheck.json"), "w"), indent=1)
    elif a.place:
        F = frame(a.which)
        t = trimesh.load(a.place, force="mesh")
        nums = (1, 4) if a.which == "front" else (2, 3)
        for mir, k in zip((False, True), nums):
            arm_to_drone(t, F, mir).export(os.path.join(out, f"arm_{k}_drone_frame.stl"))
        print("placed", nums, "in", out)
    else:
        from topo import Settings, optimise
        optimise(setup_for(a.which), h=h, gen_cm3=gen, out_dir=out, name=f"overseer_arm_{a.which}",
                 settings=Settings(), git=a.git)
