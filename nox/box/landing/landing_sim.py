"""Box landing v1 concept: the box catches each drone by its bottom opening ring (Spark 27 Oct).

Spark 27 Oct: "the bottom is no longer a funnel ... find the best way we can still land the drones
reliably with error tolerance and also the scout needs to be able to land as well"; "just really care
about the bottom opening, since that will likely be the only thing thats the same with the scout and
the overseer"; "the newest design should have two cameras" (front + down: the down camera looks through
the opening, so nothing of the catch may sit in its view in flight, and it can see a tag on the box).

The shared interface is the cage's bottom opening ring (Overseer cage v3: 8 nodes, 6 mm tubes, round the
swap zone = receiver + 30 a side). Three ways to catch it, each dropped with MuJoCo:

  dish  one shallow cone through the ring nodes (rising outward at THETA, falling inward to the swap hole)
  tray  flat floor inside the node contour, walls rising outward at THETA (nodes roll or slide)
  kin   kinematic (Maxwell) coupling: 3 short feet under the ring in 3 V-grooves pointing at the feet centroid
        (flanks THETA, half-width KIN_W, round ends), cut into a flat plate PLATE_GAP under the ring node bottoms,
        so a foot that misses its groove leaves the drone level on the plate; each foot a printed V hung from the
        two ring nodes of its tube; nothing else touches

Drop cases (E): offsets 0..80 in 8 directions, yaw -5 / 0 / +5 deg, drift 0.15 m/s turning with the case, tilt 2 deg,
descent 0.30 m/s; thrust holds the weight to the first touch, then spins down (tau 0.08 s). Pass at
4.0 s: ring centre within 2 mm, yaw within 1 deg, height within 2 mm and tilt within 2 deg of the centred seat (speed is
stored, not gated: a drone still rattling in the seat counts).
Drones from their run assemblies, masses at the part centroids (E masses, M geometry): Overseer layout v5 / cage v3
(3380 g), Scout layout v3 / cage v1 (`runs/scout_layout/v6`, 933 g, the same ring, Spark 27 Oct). Each drone's cage,
belly, pods and cartridge are point-sampled and checked against the box along every drop (`clear_mm`, ring excluded).

Run: python3 nox/box/landing/landing_sim.py [--quick]   -> runs/landing/v1-concept/sweep.json + traj.json
"""
import json, math, os, sys, itertools, time
from multiprocessing import Pool
import numpy as np
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
OUT = os.path.join(ROOT, "runs", "landing", "v1-concept")
V5 = os.path.join(ROOT, "runs", "overseer_layout", "v5")
S6 = os.path.join(ROOT, "runs", "scout_layout", "v6")      # Scout v3 layout v3 / cage v1: the same ring (G Shared bottom ring)
G = 9.81
TUBE_R = 3.0                       # 6 mm shell tube (C, Cage v3)
NODE_R = 10.15                     # shell node (C, cage_shell.json)
RECEIVER = (179.6, 76.2)           # Overseer receiver (M): the larger one sets the shared opening
HOLE_M = 15.0                      # box swap hole = receiver + 15 a side (DEFAULT: grabber zone |y| 47.5 + 2)
HOLE = (RECEIVER[0] + 2 * HOLE_M, RECEIVER[1] + 2 * HOLE_M)
RING_OFF = np.array([30.0, 0.0, 57.0])   # v5 layout frame -> ring frame (catch centre x -30, ring plane z -57)
SRC = {"overseer": (V5, RING_OFF), "scout": (S6, np.array([13.5, 0.0, 47.5]))}   # Scout catch x -13.5, ring plane z -47.5
GRID, PIT = 2.0, -90.0
DRIFT, DESCENT, TILT, TAU, T_END = 0.15, 0.30, 2.0, 0.08, 4.0

# kinematic coupling (DEFAULT): feet on the ring tubes, ball-transfer tips (bought, E rolling mu 0.03; PTFE glide 0.12 jams the Scout at 30 deg)
FEET = [(150.0, 58.4), (150.0, -58.4), (-176.0, 0.0)]    # ring frame; front pair on the front-diagonal tubes
FOOT_R = 8.0                       # glide tip radius (E)
KIN_W = 60.0                       # groove half-width (DEFAULT): catches +-30 with +-5 deg yaw, drift and tilt (sim)
PLATE_GAP = 8.0                    # V-plate top under the ring node bottoms (DEFAULT, >= 3 through the capture)
KIN_LEN = (60.0, 60.0)             # groove length inward / outward of the seated foot (the swap hole cuts the front ones)


# ---------------------------------------------------------------- ring and drones (ring frame, mm, g)
def ring_nodes():
    d = json.load(open(os.path.join(V5, "cage_shell.json")))
    n = np.array(d["nodes"])[d["open"][0]["ring"]]
    return [(float(x + RING_OFF[0]), float(y)) for x, y, _ in n]


RING = ring_nodes()


def fit_ellipse(nodes):
    P = np.array(nodes)
    u, v = np.linalg.lstsq(np.column_stack([P[:, 0] ** 2, P[:, 1] ** 2]), np.ones(len(P)), rcond=None)[0]
    return 1 / math.sqrt(u), 1 / math.sqrt(v)


def drone_parts(drone):
    """Every part of the drone's run assembly: (name, g, centroid, bbox size, mesh), ring frame."""
    import trimesh
    run, off = SRC[drone]
    parts = json.load(open(os.path.join(run, "parts", "parts.json")))
    out = []
    for q in parts:
        m = trimesh.load(os.path.join(run, "parts", q["file"]))
        m.apply_translation(off)
        out.append((q["name"], q["mass_g"], m.centroid, m.bounds[1] - m.bounds[0], m))
    return out


def drone_masses(drone):
    return [(n, mg, c, box) for n, mg, c, box, _ in drone_parts(drone)]


def drone_underside(drone, z_max=60.0):
    """Surface samples of every part below ring-frame z_max (keep-out check), ring frame."""
    import trimesh
    pts = []
    for name, _, _, _, m in drone_parts(drone):
        if name.startswith(("prop", "motor", "top plate", "Unitree", "front camera", "L2 guard", "TPU feet")):
            continue
        p, _ = trimesh.sample.sample_surface(m, 6000, seed=1)
        pts.append(p[p[:, 2] < z_max])
    return np.vstack(pts)


DRONES = {"overseer": lambda: drone_masses("overseer"), "scout": lambda: drone_masses("scout")}


def mass_props(pts):
    M = sum(p[1] for p in pts) / 1000
    cg = sum(p[1] / 1000 * p[2] / 1000 for p in pts) / M
    I = np.zeros((3, 3))
    for _, mg, c, box in pts:
        m, d = mg / 1000, c / 1000 - cg
        I += m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        a, b, cc = box / 1000
        I += np.diag([m * (b * b + cc * cc), m * (a * a + cc * cc), m * (a * a + b * b)]) / 12
    return M, cg, I


# ---------------------------------------------------------------- box surfaces (ring frame at the seat)
GROOVE_DEG = {"front": None, "rear": None}   # groove axes, deg from x (F1 +, F2 mirrored; F3); None = radial to the centroid


def kin_grooves():
    C = np.mean(np.array(FEET), axis=0)
    out = []
    for i, f in enumerate(FEET):
        f = np.array(f)
        a = (C - f) / np.linalg.norm(C - f)            # groove axis, foot -> coupling centroid
        if GROOVE_DEG["front"] is not None and i < 2:
            g = math.radians(GROOVE_DEG["front"])
            a = np.array([-math.cos(g), -math.sin(g) * (1 if f[1] > 0 else -1)])
        if GROOVE_DEG["rear"] is not None and i == 2:
            g = math.radians(GROOVE_DEG["rear"])
            a = np.array([math.cos(g), math.sin(g)])
        out.append((f, a, np.array([-a[1], a[0]])))
    return out


def foot_nodes(i):
    """The two ring nodes whose tube carries foot i (the foot hangs from both as a printed V)."""
    f = np.array(FEET[i])
    P = np.array(RING)
    best = None
    for k in range(len(P)):
        a, b = P[k], P[(k + 1) % len(P)]
        t = np.clip(np.dot(f - a, b - a) / np.dot(b - a, b - a), 0, 1)
        d = np.linalg.norm(f - (a + t * (b - a)))
        if best is None or d < best[0]:
            best = (d, a, b)
    return [best[1], best[2]]


def foot_bottom(H):
    return -NODE_R - H


def box_z(X, Y, concept, theta, H=0.0, W=KIN_W):
    """Box surface height in the ring frame with the drone seated (node centres at z 0)."""
    tn = math.tan(math.radians(theta))
    hx, hy = HOLE[0] / 2, HOLE[1] / 2
    hole = (np.abs(X) <= hx) & (np.abs(Y) <= hy)
    if concept in ("dish", "tray"):
        a, b = fit_ellipse(RING)
        s = None
        for k in range(128):
            f = 2 * math.pi * k / 128
            n = (math.cos(f), math.sin(f))
            v = n[0] * X + n[1] * Y - math.hypot(a * n[0], b * n[1])
            s = v if s is None else np.maximum(s, v)
        s = np.minimum(s, 140.0)
        if concept == "tray":
            s = np.maximum(s, 0.0)
        z = -NODE_R / math.cos(math.radians(theta)) + tn * s
        if concept == "tray":    # the floor dips into the swap hole at 30 deg over 20 (lead-out for a node)
            dh = np.hypot(np.maximum(np.abs(X) - hx, 0), np.maximum(np.abs(Y) - hy, 0))
            z = z - math.tan(math.radians(30)) * np.maximum(20 - dh, 0)
        return np.where(hole, PIT, z)
    # kin: V-grooves cut into a flat plate at the groove-top level; round (conical) groove ends
    zb = foot_bottom(H)
    top = zb + W * tn
    z = np.full(np.shape(X), top)
    for f, a, c in kin_grooves():
        u = (X - f[0]) * a[0] + (Y - f[1]) * a[1]
        v = (X - f[0]) * c[0] + (Y - f[1]) * c[1]
        du = np.maximum(0.0, np.maximum(u - KIN_LEN[0], -KIN_LEN[1] - u))
        z = np.minimum(z, zb + np.hypot(v, du) * tn)
    return np.where(hole, PIT, z)


def hfield(concept, theta, H, W):
    P = np.array(RING)
    rx, ry = float(np.abs(P[:, 0]).max() + 180), float(np.abs(P[:, 1]).max() + 180)
    xs, ys = np.arange(-rx, rx + 1e-6, GRID), np.arange(-ry, ry + 1e-6, GRID)
    X, Y = np.meshgrid(xs, ys)
    Z = box_z(X, Y, concept, theta, H, W)
    zmin, zmax = float(Z.min()), float(Z.max())
    return dict(nrow=len(ys), ncol=len(xs), rx=rx, ry=ry, span=zmax - zmin, zmin=zmin, elev=(Z - zmin) / (zmax - zmin))


# ---------------------------------------------------------------- model and drop
def build_xml(drone, concept, theta, mu, H, W=KIN_W):
    pts = DRONES[drone]()
    M, cg, I = mass_props(pts)
    ev, evec = np.linalg.eigh(I)
    if np.linalg.det(evec) < 0:
        evec[:, 0] *= -1
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, evec.flatten())
    hf = hfield(concept, theta, H, W)
    mm = 1e-3
    f0 = '"0.001 0.001 0.0001"'
    geoms = []
    P = [np.array([x, y, 0.0]) * mm for x, y in RING]
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        geoms.append(f'<geom name="ring{i}" type="capsule" fromto="{a[0]:.5f} {a[1]:.5f} 0 {b[0]:.5f} {b[1]:.5f} 0" size="{TUBE_R*mm}" friction={f0} mass="0"/>')
        geoms.append(f'<geom name="node{i}" type="sphere" pos="{a[0]:.5f} {a[1]:.5f} 0" size="{NODE_R*mm}" friction={f0} mass="0"/>')
    if concept == "kin":
        zb = foot_bottom(H)
        for i, (x, y) in enumerate(FEET):
            tip = np.array([x, y, zb + FOOT_R]) * mm
            geoms.append(f'<geom name="foot{i}" type="sphere" pos="{tip[0]:.5f} {tip[1]:.5f} {tip[2]:.5f}" size="{FOOT_R*mm}" friction={f0} mass="0"/>')
            for j, nd in enumerate(foot_nodes(i)):
                n3 = np.array([nd[0], nd[1], 0.0]) * mm
                geoms.append(f'<geom name="leg{i}{j}" type="capsule" fromto="{tip[0]:.5f} {tip[1]:.5f} {tip[2]+0.006:.5f} {n3[0]:.5f} {n3[1]:.5f} 0" size="{4*mm}" friction={f0} mass="0"/>')
    xml = f"""<mujoco><option timestep="0.0005" gravity="0 0 -{G}" integrator="implicitfast"/>
<default><geom solref="0.004 0.5" solimp="0.95 0.99 0.001" condim="3" contype="2" conaffinity="1"/></default>
<asset><hfield name="box" nrow="{hf['nrow']}" ncol="{hf['ncol']}" size="{hf['rx']*mm:.5f} {hf['ry']*mm:.5f} {hf['span']*mm:.5f} 0.01"
 elevation="{' '.join(f'{v:.4f}' for v in hf['elev'].flatten())}"/></asset>
<worldbody><geom type="hfield" hfield="box" pos="0 0 {hf['zmin']*mm:.5f}" friction="{mu} 0.001 0.0001" contype="1" conaffinity="2"/>
<body name="drone" pos="0 0 0.2"><freejoint/>
<inertial pos="{cg[0]:.6f} {cg[1]:.6f} {cg[2]:.6f}" quat="{' '.join(f'{v:.6f}' for v in quat)}" mass="{M:.5f}" diaginertia="{' '.join(f'{max(v,1e-6):.7f}' for v in ev)}"/>
{''.join(geoms)}</body></worldbody></mujoco>"""
    return xml, M, cg


_CACHE = {}


def model_for(key):
    if key not in _CACHE:
        xml, M, cg = build_xml(*key)
        _CACHE[key] = (mujoco.MjModel.from_xml_string(xml), M, cg)
    return _CACHE[key]


def drop_height(key, dx, dy):
    concept, theta, H, W = key[1], key[2], key[4], key[5]
    pts = np.array([[x + dx, y + dy] for x, y in (FEET if concept == "kin" else RING)])
    z = box_z(pts[:, 0], pts[:, 1], concept, theta, H, W)
    low = foot_bottom(H) if concept == "kin" else -NODE_R
    return float(np.max(z) - low + 20.0)


def drop(key, dx, dy, yaw, k, record=False):
    m, M, cg = model_for(key)
    d = mujoco.MjData(m)
    ang = 2 * math.pi * ((k * 3) % 8) / 8
    tax = 2 * math.pi * ((k * 5) % 8) / 8 + 0.3
    qy, qt, q = np.zeros(4), np.zeros(4), np.zeros(4)
    mujoco.mju_axisAngle2Quat(qy, np.array([0, 0, 1.0]), math.radians(yaw))
    mujoco.mju_axisAngle2Quat(qt, np.array([math.cos(tax), math.sin(tax), 0.0]), math.radians(TILT))
    mujoco.mju_mulQuat(q, qt, qy)
    d.qpos[:3] = [dx * 1e-3, dy * 1e-3, max(0.0, drop_height(key, dx, dy)) * 1e-3]
    d.qpos[3:7] = q
    d.qvel[:3] = [DRIFT * math.cos(ang), DRIFT * math.sin(ang), -DESCENT]
    body = m.body("drone").id
    t_c, traj, every = None, [], 40 if record else 100
    for s in range(int(T_END / m.opt.timestep)):
        if d.ncon and t_c is None:
            t_c = d.time
        T = M * G if t_c is None else M * G * math.exp(-(d.time - t_c) / TAU)
        d.xfrc_applied[body, :3] = d.xmat[body].reshape(3, 3)[:, 2] * T
        mujoco.mj_step(m, d)
        if s % every == 0:
            traj.append([round(d.time, 3)] + [round(float(v), 6) for v in d.qpos[:7]])
    bad = d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0 or d.qpos[2] < -0.05
    R = d.xmat[body].reshape(3, 3)
    return dict(x=d.qpos[0] * 1e3, y=d.qpos[1] * 1e3, z=d.qpos[2] * 1e3, yaw=math.degrees(math.atan2(R[1, 0], R[0, 0])),
                tilt=math.degrees(math.acos(max(-1, min(1, R[2, 2])))), v=float(np.linalg.norm(d.qvel[:3])) * 1e3,
                bad=bool(bad), traj=traj)


# ---------------------------------------------------------------- keep-out along a drop (Overseer v5 as drawn)
_UNDER, _GRID = {}, {}


def clearance(key, traj):
    """Least gap (mm) between the drone's underside (cage, belly, pods, cartridge; not the ring nodes and tubes,
    not the feet) and the box surface over the recorded poses; negative = that part would hit the box."""
    dr = key[0]
    if dr not in _UNDER:
        P = drone_underside(dr)
        rn = np.array(RING)
        seg = []
        for i in range(len(rn)):
            a, b = rn[i], rn[(i + 1) % len(rn)]
            t = np.clip(((P[:, :2] - a) @ (b - a)) / np.dot(b - a, b - a), 0, 1)
            seg.append(np.hypot(*(P[:, :2] - (a + t[:, None] * (b - a))).T))
        on_ring = (np.min(np.array(seg), axis=0) < NODE_R + 1) & (np.abs(P[:, 2]) < NODE_R + 1)
        _UNDER[dr] = P[~on_ring] * 1e-3
    gk = (key[1], key[2], key[4], key[5])
    if gk not in _GRID:
        rx, ry = 700.0, 600.0
        xs, ys = np.arange(-rx, rx + 1e-6, 1.0), np.arange(-ry, ry + 1e-6, 1.0)
        X, Y = np.meshgrid(xs, ys)
        _GRID[gk] = (box_z(X, Y, key[1], key[2], key[4], key[5]), rx, ry)
    Z, rx, ry = _GRID[gk]
    P = _UNDER[dr]
    worst = 1e9
    for row in traj[::2]:
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.array(row[4:8]))
        W = (P @ R.reshape(3, 3).T + np.array(row[1:4])) * 1e3
        ix = np.clip(np.rint(W[:, 0] + rx).astype(int), 0, Z.shape[1] - 1)
        iy = np.clip(np.rint(W[:, 1] + ry).astype(int), 0, Z.shape[0] - 1)
        worst = min(worst, float(np.min(W[:, 2] - Z[iy, ix])))
    return round(worst, 1)


# ---------------------------------------------------------------- sweep
OFFSETS = [0, 10, 20, 30, 40, 50, 60, 70, 80]
YAWS = [-5.0, 0.0, 5.0]


def run_config(key):
    t0 = time.time()
    seat = drop(key, 0.0, 0.0, 0.0, 0)
    rows, k = [], 0
    for r in OFFSETS:
        for j in range(8):
            a = 2 * math.pi * j / 8
            for yaw in YAWS:
                k += 1
                o = drop(key, r * math.cos(a), r * math.sin(a), yaw, k)
                err = math.hypot(o["x"] - seat["x"], o["y"] - seat["y"])
                ok = ((not o["bad"]) and err < 2.0 and abs(o["yaw"] - seat["yaw"]) < 1.0 and abs(o["z"] - seat["z"]) < 2.0
                      and abs(o["tilt"] - seat["tilt"]) < 2.0)
                rows.append(dict(r=r, dir=j, yaw=yaw, ok=bool(ok), err=round(err, 1), dyaw=round(o["yaw"] - seat["yaw"], 2),
                                 dz=round(o["z"] - seat["z"], 1), v=round(o["v"], 1), tilt=round(o["tilt"], 1), fell=o["bad"],
                                 clear_mm=clearance(key, o["traj"])))
    cap = 0
    for r in OFFSETS:
        if all(x["ok"] for x in rows if x["r"] == r):
            cap = r
        else:
            break
    rate = {str(r): round(sum(x["ok"] for x in rows if x["r"] == r) / sum(1 for x in rows if x["r"] == r), 3) for r in OFFSETS}
    cl = [x["clear_mm"] for x in rows if x["clear_mm"] is not None and x["ok"]]
    return dict(key=list(key), seat=dict(x=round(seat["x"], 2), y=round(seat["y"], 2), z=round(seat["z"], 2), yaw=round(seat["yaw"], 2), tilt=round(seat["tilt"], 2)),
                capture_mm=cap, rate=rate, worst_err=max(x["err"] for x in rows), clear_min=min(cl) if cl else None,
                rows=rows, secs=round(time.time() - t0, 1))


def kin_H(th, W=KIN_W):
    return round(W * math.tan(math.radians(th)) + PLATE_GAP, 1)


# drone, concept, theta (deg: dish/tray wall or groove flank), mu, H (foot length below the node bottoms, kin), W
CONFIGS = (
    [(dr, "kin", th, mu, kin_H(th), KIN_W) for dr in ("overseer", "scout")
     for th, mu in ((30.0, 0.03), (20.0, 0.03), (30.0, 0.06), (30.0, 0.12), (40.0, 0.12), (30.0, 0.35))]
    + [(dr, "tray", th, mu, 0.0, 0.0) for dr in ("overseer", "scout") for th, mu in ((10.0, 0.03), (10.0, 0.06), (20.0, 0.12))]
    + [(dr, "dish", 10.0, mu, 0.0, 0.0) for dr in ("overseer", "scout") for mu in (0.03, 0.12)]
)


def record_examples():
    """A few drops per concept for the motion artifact."""
    ex = {}
    for key, cases in [(("overseer", "kin", 30.0, 0.03, kin_H(30.0), KIN_W), [(30, 15, 5), (-20, -25, -5)]),
                       (("scout", "kin", 30.0, 0.03, kin_H(30.0), KIN_W), [(-25, 20, 5)]),
                       (("scout", "kin", 30.0, 0.12, kin_H(30.0), KIN_W), [(40, 0, 5)]),
                       (("overseer", "tray", 10.0, 0.03, 0.0, 0.0), [(60, 0, 0)]),
                       (("overseer", "dish", 10.0, 0.12, 0.0, 0.0), [(40, 30, 0)])]:
        for i, (dx, dy, yw) in enumerate(cases):
            o = drop(key, dx, dy, yw, 3 + i, record=True)
            ex[f"{key[0]}|{key[1]}|{dx},{dy},{yw}"] = dict(key=list(key), start=[dx, dy, yw], traj=o["traj"],
                                                          end=dict(x=round(o["x"], 1), y=round(o["y"], 1), yaw=round(o["yaw"], 2)))
    return ex


def main():
    global OFFSETS, YAWS
    quick = "--quick" in sys.argv
    cfgs = CONFIGS
    if quick:
        OFFSETS, YAWS = [0, 40], [0.0]
        cfgs = [("overseer", "kin", 30.0, 0.03, kin_H(30.0), KIN_W), ("scout", "kin", 30.0, 0.03, kin_H(30.0), KIN_W)]
    os.makedirs(OUT, exist_ok=True)
    with Pool(min(4, os.cpu_count() or 1)) as p:
        res = p.map(run_config, cfgs)
    for r in res:
        dr, cpt, th, mu, H, W = r["key"]
        print(f"{dr:8s} {cpt:4s} theta {th:4.1f} mu {mu:.2f} H {H:5.1f} W {W:4.0f}  capture {r['capture_mm']:3d}  rates {' '.join(f'{k}:{v:.2f}' for k, v in r['rate'].items())}"
              f"  worst {r['worst_err']:6.1f}  clear {r['clear_min']}  seat {r['seat']}  ({r['secs']} s)", flush=True)
    if not quick:
        json.dump(dict(ring=RING, ring_ellipse=fit_ellipse(RING), feet=FEET, foot_r=FOOT_R, kin_w=KIN_W, kin_len=KIN_LEN,
                       grooves=[dict(foot=f.tolist(), axis=a.tolist()) for f, a, _ in kin_grooves()], hole=HOLE,
                       offsets=OFFSETS, yaws=YAWS, cases=dict(drift=DRIFT, descent=DESCENT, tilt=TILT, tau=TAU, t_end=T_END),
                       results=res), open(os.path.join(OUT, "sweep.json"), "w"))
        json.dump(record_examples(), open(os.path.join(OUT, "traj.json"), "w"))


if __name__ == "__main__":
    main()
