"""Collar funnel v4 centering check (MuJoCo rigid contact with Coulomb friction).

Question (Spark 16 Oct 18:37): will the drone always end up centred, and can the node
contact faces be shaped so it does?

Final layout (16 Oct 19:02): c 100, opening 168, 8 PETG domes built into the corner nodes.
Variants (Scout guard v2, props on top, seat frame = drone coords, metres):
  A  as drawn today: rods + TPU nodes that stick out past the rods (tee nodes 238.8 across
     vs 230 flats in guard v1, E: +4.4 on the radius) -> nodes touch, grippy TPU.
  D  nodes trimmed flush to the rod line: only the 2.5 lower-ring flats touch (line contact,
     4 flats = over-constrained).
  B  8 corner nodes carry a hard dome (PETG or POM) that stands 1.0 proud of the rod
     surface toward its face; rods and tees stay 1.0 off the faces -> 8 point contacts,
     2 per face, spread +-78 along each face.
Pass: settles within 0.5 mm, 0.3 deg yaw, 0.3 deg tilt of the centred seat, in 1.2 s.
Inputs tagged: C from s2_catch_opt.py, E estimates.
"""
import math, itertools, json, sys
import numpy as np
import mujoco

A = math.radians(34.99)
TA, SA, CA = math.tan(A), math.sin(A), math.cos(A)
mm = 1e-3
F, C = 112 * mm, 100 * mm              # ring flats / chamfer corners (C; c 100 from s2_corner_opt.py)
ZS = 32.5 * mm                         # lower ring centre (C)
R_ROD = 1.25 * mm
H_AXIS = F + R_ROD / SA                # face half-width at the ring axis height (C)
Z_HI = ZS + 32 * mm                    # mouth: 40 off first touch 28 + 4
EDGE_HALF = 84 * mm                    # opening 168 (largest that centres 100 %, s2_collar_edge)
Z_LO = ZS - (H_AXIS - EDGE_HALF) * TA
MASS = 0.45                            # E (v2 target 400, v1 520 P)
CG_Z = 10 * mm                         # E: stack + pack below the ring


def h(z):
    return H_AXIS + (z - ZS) / TA


def quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def faces_xml(mu_face=0.001):
    out = []
    zm = 0.5 * (Z_LO + Z_HI)
    L = (Z_HI - Z_LO) / SA / 2
    W = h(Z_HI) + 20 * mm
    Ry = np.array([[CA, 0, -SA], [0, 1, 0], [SA, 0, CA]])  # columns: slope dir, y, normal
    Ry = np.array([[CA, 0, -SA], [0, 1, 0], [SA, 0, CA]])
    for k in range(4):
        t = k * math.pi / 2
        Rz = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
        R = Rz @ Ry
        n = R[:, 2]
        p = Rz @ np.array([h(zm), 0, zm]) - n * 2 * mm
        q = quat_from_mat(R)
        out.append(f'<geom type="box" size="{L} {W} {2*mm}" pos="{p[0]} {p[1]} {p[2]}" '
                   f'quat="{q[0]} {q[1]} {q[2]} {q[3]}" friction="{mu_face} 0.001 0.0001" rgba=".8 .3 .3 .4"/>')
    return "\n".join(out)


CORNERS = [(F, -C), (F, C), (C, F), (-C, F), (-F, C), (-F, -C), (-C, -F), (C, -F)]
TEES = [(0, F), (0, -F), (-F, 0)]       # no nose tee (camera)
DOMES = None                            # override dome points (variant B); None = 8 corners


def face_out_normal(x, y):
    """outward-down normal toward the face a ring point leans on"""
    if abs(x) >= abs(y):
        s = math.copysign(1, x)
        return np.array([s * SA, 0, -CA])
    s = math.copysign(1, y)
    return np.array([0, s * SA, -CA])


def drone_xml(variant, mu_rod, mu_node, proud=1.0 * mm, dome_r=3.0 * mm):
    g = []
    off = 0.0 if variant != "B" else proud           # B: rods held 1.0 off the faces
    for i in range(8):
        (x0, y0), (x1, y1) = CORNERS[i], CORNERS[(i + 1) % 8]
        g.append(f'<geom type="capsule" size="{R_ROD}" fromto="{x0} {y0} {ZS} {x1} {y1} {ZS}" '
                 f'friction="{mu_rod} 0.005 0.0001" rgba="0 0 0 1"/>')
        g.append(f'<geom type="capsule" size="{1*mm}" fromto="{x0} {y0} {ZS+16*mm} {x1} {y1} {ZS+16*mm}" '
                 f'friction="{mu_rod} 0.005 0.0001" rgba="0 0 0 1"/>')
    if variant == "A":
        for x, y in CORNERS + TEES:
            g.append(f'<geom type="sphere" size="{4.4*mm}" pos="{x} {y} {ZS}" '
                     f'friction="{mu_node} 0.005 0.0001" rgba=".2 .6 .2 1"/>')
    elif variant == "B":
        for x, y in (DOMES or CORNERS):
            n = face_out_normal(x, y)
            dist = (H_AXIS - max(abs(x), abs(y))) * SA          # ring axis to its face
            c = np.array([x, y, ZS]) + n * (dist + proud - dome_r)
            g.append(f'<geom type="sphere" size="{dome_r}" pos="{c[0]} {c[1]} {c[2]}" '
                     f'friction="{mu_node} 0.005 0.0001" rgba=".2 .2 .8 1"/>')
    return "\n".join(g), off


def build(variant, mu_rod, mu_node):
    g, _ = drone_xml(variant, mu_rod, mu_node)
    xml = f"""
<mujoco>
 <option timestep="0.0002" integrator="implicitfast" cone="elliptic" impratio="10" gravity="0 0 -9.81"/>
 <default><geom solref="0.002 1" solimp="0.95 0.99 0.0005" condim="3"/></default>
 <worldbody>
  {faces_xml()}
  <body name="drone" pos="0 0 0">
   <freejoint/>
   <inertial pos="0 0 {CG_Z}" mass="{MASS}" diaginertia="1.6e-3 1.6e-3 2.8e-3"/>
   {g}
  </body>
 </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def drop(m, x, y, yaw, tilt_ax, tilt, v, T=1.2):
    d = mujoco.MjData(m)
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.array([tilt * math.cos(tilt_ax), tilt * math.sin(tilt_ax), yaw]), "xyz")
    d.qpos[:3] = [x, y, 0.07]
    d.qpos[3:7] = q
    # come down from above until first touch, then back off 3 mm
    for _ in range(400):
        mujoco.mj_forward(m, d)
        if d.ncon > 0:
            break
        d.qpos[2] -= 0.25 * mm
    d.qpos[2] += 3 * mm
    d.qvel[:3] = v
    n = int(T / m.opt.timestep)
    for _ in range(n):
        mujoco.mj_step(m, d)
    pos = d.qpos[:3].copy()
    R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); R = R.reshape(3, 3)
    yaw_f = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    tilt_f = math.degrees(math.acos(max(-1, min(1, R[2, 2]))))
    speed = float(np.linalg.norm(d.qvel[:3]))
    return pos, yaw_f, tilt_f, speed


def run(variant, mu_rod, mu_node):
    m = build(variant, mu_rod, mu_node)
    ref, _, _, _ = drop(m, 0, 0, 0, 0, 0, [0, 0, -0.10])
    offs = [(0, 0)] + [(d * math.cos(a), d * math.sin(a)) for d in (20 * mm, 40 * mm)
                       for a in [k * math.pi / 4 for k in range(8)]]
    res = []
    for (x, y), yaw, tilt, drift in itertools.product(offs, (-5, 0, 5), (0, 2), (1, -1)):
        a = math.atan2(y, x) if (x or y) else 0.0
        v = [drift * 0.15 * math.cos(a), drift * 0.15 * math.sin(a), -0.10]   # +1 drifts outward
        pos, yf, tf, sp = drop(m, x, y, math.radians(yaw), a + 0.7, math.radians(tilt), v)
        e = pos - ref
        res.append(dict(xy=1e3 * math.hypot(e[0], e[1]), z=1e3 * e[2], yaw=abs(yf), tilt=tf, v=sp))
    ok = [r for r in res if r["xy"] < 0.5 and r["yaw"] < 0.3 and r["tilt"] < 0.3 and abs(r["z"]) < 0.5]
    worst = max(res, key=lambda r: r["xy"] + r["yaw"] + r["tilt"])
    return dict(variant=variant, mu_rod=mu_rod, mu_node=mu_node, runs=len(res),
                centred_pct=round(100 * len(ok) / len(res), 1),
                worst_xy_mm=round(max(r["xy"] for r in res), 2),
                worst_yaw_deg=round(max(r["yaw"] for r in res), 2),
                worst_tilt_deg=round(max(r["tilt"] for r in res), 2),
                worst_z_hang_mm=round(max(r["z"] for r in res), 2),
                seat_z_mm=round(1e3 * ref[2], 2))


if __name__ == "__main__":
    cases = [("B", 0.30, 0.15), ("B", 0.30, 0.20), ("B", 0.30, 0.25)]   # final: PETG domes
    if len(sys.argv) > 1:
        cases = [c for c in cases if c[0] in sys.argv[1]]
    for c in cases:
        print(json.dumps(run(*c)), flush=True)
