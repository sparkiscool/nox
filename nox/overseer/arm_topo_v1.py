"""Overseer printed arms v1: front and rear pair, nox/topo (organic), from layout v5 / cage v3 (27 Oct).

python3 nox/overseer/arm_topo_v1.py front|rear [--gen CM3] [--h H] [--out DIR] [--git]   topology run
python3 nox/overseer/arm_topo_v1.py front|rear --setup                                    setup summary only
python3 nox/overseer/arm_topo_v1.py front|rear --modes MESH [--out DIR]                    first modes of a built arm (Hz)
python3 nox/overseer/arm_topo_v1.py front|rear --place MESH                               the 2 arms of the pair in the drone frame

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

LAYOUT = os.path.join(ROOT, "runs", "overseer_layout", "v5")
G = 9.81

# ------------------------------------------------------------------------------------------- numbers (dimensions.md)
ROOT_H = 45.0                         # root between the plates, arm top = top plate underside (O Arms and hub, DEFAULT)
U_ROOT = (58.0, 75.0)                 # root block along the diagonal (O: inner face u >= 58 for |v| <= 20; stock block to 75)
V_ROOT = 21.0
TOP_INS = [(63.17, 15.0), (63.17, -15.0)]      # M3 into the top plate (O, derived)
BOT_INS = [(63.74, 10.0), (63.74, -10.0)]      # M3 into the bottom plate (O, derived)
INS_D, INS_DEPTH, BOSS_D = 4.0, 7.0, 12.0      # M3 heat-set insert hole (O), boss Ø12 (attachment rule)
ROOT_PAD_T = 9.0                      # root contact pads 9 thick: the insert depth + 2, and room for the chord bores
PAD_T = 6.0                           # motor pad (README: plates >= 6 for the gate's 2 mm core)
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
# both plates) to y ±12 beside the motor pocket; bore 3.3 (OD + 0.3), open at the shoulder end (the rod slides in from
# there and is epoxied), blind at the motor end inside the pocket wall. Drawn: a socket at each rod end; the topology
# webs, root and pad join them, the rods are stiff inserts in its FE (Part.insert).
ROD_D, ROD_BORE = 3.0, 3.3
SHOULDER = {"front": (25.0, 35.0, 9.0)}              # arm-frame x, |y|, plan radius of the shoulder envelope disc
ROD_TIP = (15.0, 12.0)                               # rod end: x = L - 15, |y| 12 (r 19.2 from the motor axis, in the wall)
ROD_Z = (41.0, 4.0)                                  # top and bottom rod centres (sleeve flat to the top / bottom face)
CHORD_SKIN = 2.65                     # socket wall round the bore (>= skin 2.5)
SOCKET_L = 18.0                       # drawn socket at each rod end (6 x the rod: epoxy bond ~ 2 MPa x pi x 3 x 18 = 340 N, E)
E_CARBON = 120000.0                   # MPa, pultruded 3.0 rod along its axis (E)
RHO_CARBON = 1.55                     # g/cm3 (E)
E_PETG = 2780.0                       # MPa, nox/topo/material.py

# ESC: a 4-in-1 under the FC, the stack on top of the top plate (Spark 27 Oct), nothing on the arms; the motor leads run
# along the arm's flat top to it, zip-tied round the top carbon rods' free spans (front) or the beam (DEFAULT)

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
    """hub electronics (FC stack, Pi 5 + deck, PDB, UBEC, buck, RX) and the L2 keel hub, + CLEAR, as boxes"""
    out = mf.Manifold()
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
    B = {k: to_arm(v, F) for k, v in dict(env=env, pads=pad_top + pad_bot, mpad=mpad + mwall, cuts=cuts, sup=sup,
                                         bosses=bosses, load=load, ko=ko).items()}
    if which in SHOULDER:                       # the plan envelope widens to the two shoulders (arm frame)
        xs, ys, rs = SHOULDER[which]
        sh = cyl(xs, ys, 0.0, ROOT_H, rs) + cyl(xs, -ys, 0.0, ROOT_H, rs)
        B["env"] = (B["env"] + sh).hull() - B["ko"]

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
        for q0, q1 in ((p0, p0 + SOCKET_L * u), (p1 - SOCKET_L * u, p1 + 1.0 * u)):
            sockets += rod_m(q0, q1, r, n=48) ^ box(-50, F["L"] + 50, -60, 60, 0.0, ROOT_H)
    B["rods"] = rods
    B["drawn"] = B["pads"] + B["mpad"] + sockets
    return B, F


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
            p.insert(f"rod {k_}", rod_m(p0, p1, rr), E_CARBON / E_PETG, area_mm2=math.pi * (ROD_D / 2) ** 2,
                     length_mm=float(np.linalg.norm(p1 - p0)))
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
def modes(which, mesh_path, h=1.0, n=4, support="faces", log=print):
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
    rod = np.zeros(shape, bool)
    P = np.stack([X, Y, Z], -1)
    for p0, p1 in chords(which, F):
        d = p1 - p0; Ln = np.linalg.norm(d); u = d / Ln
        t = (P - p0) @ u
        rod |= (np.linalg.norm(P - p0 - t[..., None] * u, axis=-1) <= ROD_D / 2) & (t >= 0) & (t <= Ln)
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
    nrod = max(rod.sum(), 1)
    rod_len = sum(float(np.linalg.norm(p1 - p0)) for p0, p1 in chords(which, F))
    area_fix = (math.pi * (ROD_D / 2) ** 2 * rod_len) / (nrod * h ** 3) if rod.any() else 1.0
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
    a = ap.parse_args()
    h = a.h or (2.0 if a.which == "front" else 1.5)   # ~ min member / 3 (topo README); direct solver (27 Oct: h 1.5 / 1.0 fell to multigrid, ~10 min an iteration)
    gen = a.gen or (40.0 if a.which == "front" else 25.0)   # cm3 generated (E), generous (Spark 27 Oct); front rods carry the chords
    out = a.out or os.path.join(ROOT, "runs", f"overseer_arm_{a.which}", "v1")
    if a.setup:
        p = setup_for(a.which)(h)
        s = p.summary(); s["cases"] = len(s["cases"]); print(json.dumps(s), frame(a.which))
    elif a.modes:
        r = {sp_: modes(a.which, a.modes, support=sp_) for sp_ in ("faces", "inserts")}
        os.makedirs(out, exist_ok=True)
        json.dump(r, open(os.path.join(out, "modes.json"), "w"), indent=1)
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
