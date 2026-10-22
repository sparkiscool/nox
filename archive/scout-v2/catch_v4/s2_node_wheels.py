"""All-PETG catch: can the corner nodes centre the drone on bare PETG faces (mu 0.3 to 0.4, E)?

Node options (8 corner nodes, c 100, 168 opening, 35 deg, 53.1 collar; s2_collar_settle.py geometry):
  dome   : fixed PETG dome (R 3, 1.0 proud) sliding at mu
  wheelH : print-in-place PETG wheel, axle up the slope -> rolls HORIZONTALLY along its face
  wheelD : axle horizontal -> rolls DOWN the slope
  wheel45: axle at 45 deg in the face plane
Wheel: R 4, 3 wide, on a pin r 1 (print-in-place, 0.3 gap E). Rolling resistance = axle friction
torque mu * N * r_pin (E: N ~ 0.6 N per wheel -> 2.4e-4 Nm at mu 0.4), modelled as hinge frictionloss.
Sideways (along the axle) the wheel slides at mu.
"""
import math, itertools, json, sys
import numpy as np
import mujoco
import s2_collar_settle as s

mm = 1e-3
R_W, HW_W, R_PIN = 4 * mm, 1.5 * mm, 1 * mm
PROUD = 1.0 * mm


def face_frame(x, y):
    """n_out (toward the face), u (up the slope, outward), t (horizontal along the face)"""
    if abs(x) >= abs(y):
        s_ = math.copysign(1, x)
        return (np.array([s_ * s.SA, 0, -s.CA]), np.array([s_ * s.CA, 0, s.SA]), np.array([0, 1.0, 0]))
    s_ = math.copysign(1, y)
    return (np.array([0, s_ * s.SA, -s.CA]), np.array([0, s_ * s.CA, s.SA]), np.array([1.0, 0, 0]))


def quat_z_to(a):
    """quaternion turning local z onto unit vector a"""
    a = a / np.linalg.norm(a)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, a); c = float(np.dot(z, a))
    if np.linalg.norm(v) < 1e-9:
        return np.array([1.0, 0, 0, 0]) if c > 0 else np.array([0, 1.0, 0, 0])
    q = np.array([1 + c, *v]); return q / np.linalg.norm(q)


def build(node, mu, axle_mu=0.4, wheel_load=0.6):
    rods = []
    for i in range(8):
        (x0, y0), (x1, y1) = s.CORNERS[i], s.CORNERS[(i + 1) % 8]
        for z, r in ((s.ZS, s.R_ROD), (s.ZS + 16 * mm, 1 * mm)):
            rods.append(f'<geom type="capsule" size="{r}" fromto="{x0} {y0} {z} {x1} {y1} {z}" friction="{mu} 0.005 0.0001"/>')
    nodes = []
    frictionloss = axle_mu * wheel_load * R_PIN
    for k, (x, y) in enumerate(s.CORNERS):
        n, u, t = face_frame(x, y)
        dist = (s.H_AXIS - max(abs(x), abs(y))) * s.SA
        p = np.array([x, y, s.ZS])
        if node == "dome":
            c = p + n * (dist + PROUD - 3 * mm)
            nodes.append(f'<geom type="sphere" size="{3*mm}" pos="{c[0]} {c[1]} {c[2]}" friction="{mu} 0.005 0.0001"/>')
            continue
        kind = node if node != "mix" else ("wheelD" if k % 2 == 0 else "wheelH")
        axle = {"wheelH": u, "wheelD": t, "wheel45": (u + t) / math.sqrt(2)}[kind]
        c = p + n * (dist + PROUD - R_W)
        q = quat_z_to(axle)
        nodes.append(f'''<body pos="{c[0]} {c[1]} {c[2]}" quat="{q[0]} {q[1]} {q[2]} {q[3]}">
   <joint type="hinge" axis="0 0 1" frictionloss="{frictionloss}" damping="1e-7"/>
   <geom type="cylinder" size="{R_W} {HW_W}" mass="0.0002" friction="{mu} 0.005 0.0001"/>
  </body>''')
    xml = f"""
<mujoco>
 <option timestep="0.0002" integrator="implicitfast" cone="elliptic" impratio="10"/>
 <default><geom solref="0.002 1" solimp="0.95 0.99 0.0005" condim="3"/></default>
 <worldbody>
  {s.faces_xml()}
  <body name="drone">
   <freejoint/>
   <inertial pos="0 0 {s.CG_Z}" mass="{s.MASS}" diaginertia="1.6e-3 1.6e-3 2.8e-3"/>
   {''.join(rods)}
   {''.join(nodes)}
  </body>
 </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def drop(m, x, y, yaw, tax, tilt, v, T=1.5):
    d = mujoco.MjData(m)
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.array([tilt * math.cos(tax), tilt * math.sin(tax), yaw]), "xyz")
    d.qpos[:3] = [x, y, 0.07]; d.qpos[3:7] = q
    for _ in range(400):
        mujoco.mj_forward(m, d)
        if d.ncon > 0:
            break
        d.qpos[2] -= 0.25 * mm
    d.qpos[2] += 3 * mm
    d.qvel[:3] = v
    for _ in range(int(T / m.opt.timestep)):
        mujoco.mj_step(m, d)
    R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); R = R.reshape(3, 3)
    return d.qpos[:3].copy(), math.degrees(math.atan2(R[1, 0], R[0, 0])), math.degrees(math.acos(min(1, R[2, 2])))


def run(node, mu, offsets=(20, 40)):
    m = build(node, mu)
    ref = drop(m, 0, 0, 0, 0, 0, [0, 0, -0.1])[0]
    res = []
    offs = [(0, 0)] + [(d * mm * math.cos(a), d * mm * math.sin(a)) for d in offsets
                       for a in [k * math.pi / 4 for k in range(8)]]
    for (x, y), yaw, tilt, dr in itertools.product(offs, (-5, 0, 5), (0, 2), (1, -1)):
        a = math.atan2(y, x) if (x or y) else 0.0
        v = [dr * 0.15 * math.cos(a), dr * 0.15 * math.sin(a), -0.10]
        p, yf, tf = drop(m, x, y, math.radians(yaw), a + 0.7, math.radians(tilt), v)
        e = (p - ref) * 1e3
        res.append(math.hypot(e[0], e[1]) < 0.5 and abs(yf) < 0.3 and tf < 0.3)
    return dict(node=node, mu=mu, centred_pct=round(100 * sum(res) / len(res), 1), runs=len(res))


def set_slope(deg, mouth_half=159.9 * mm, edge_half=84 * mm):
    """same mouth and opening, height follows the slope"""
    s.A = math.radians(deg); s.TA, s.SA, s.CA = math.tan(s.A), math.sin(s.A), math.cos(s.A)
    s.H_AXIS = s.F + s.R_ROD / s.SA
    s.Z_LO = s.ZS - (s.H_AXIS - edge_half) * s.TA
    s.Z_HI = s.ZS + (mouth_half - s.H_AXIS) * s.TA
    return (s.Z_HI - s.Z_LO) * 1e3


if __name__ == "__main__":
    node, mu = sys.argv[1], float(sys.argv[2])
    deg = float(sys.argv[3]) if len(sys.argv) > 3 else 34.99
    axle_mu = float(sys.argv[4]) if len(sys.argv) > 4 else 0.4
    H = set_slope(deg)
    m_build = build
    globals()["build"] = lambda n, mu_: m_build(n, mu_, axle_mu=axle_mu)
    r = run(node, mu); r.update(slope=deg, collar_H=round(H, 1), axle_mu=axle_mu)
    print(json.dumps(r), flush=True)
