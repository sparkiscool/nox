"""Full-physics landing in the 45 deg PETG collar (replaces the kinematic approach in s2_catch_record.py).

What is modelled (E unless tagged):
- Mass and inertia from a component build-up (~400 g, the v2 target): pack + cartridge low, rings and motors wide.
- Flight under power: a velocity controller tilts the drone and sets thrust to hold the approach
  (0.10 m/s down plus the drift), from 35 mm above the first touch.
- Motor kill at the first face contact; thrust then decays with the props spinning down (tau 0.08 s, no brake).
- Air drag on the frame (CdA 0.010 m^2).
- Contacts: PETG on PETG mu 0.4, compliant with some bounce (time constant 4 ms, damping ratio 0.5);
  wheels R 4 on a 2 mm pin with axle friction mu 0.4 x load x pin radius.
"""
import math, json, itertools, sys
import numpy as np, mujoco
import s2_node_wheels as w, s2_collar_settle as s

mm, g0 = 1e-3, 9.81
H = w.set_slope(45)

# ---- mass build-up (mm, g) : name, mass, x, y, z, (box dims or None)
PARTS = [
    ("lower ring 2.5 carbon", 6.6, "ring", s.ZS / mm), ("upper ring 2.0 carbon", 4.3, "ring", s.ZS / mm + 16),
    ("8 PETG corner nodes + wheels", 9.6, "corners", s.ZS / mm), ("3 tee nodes + struts", 9.0, "tees", 28.0),
    ("Bee35 arm plate", 28.0, (0, 0, 20.25), (100, 94, 3.5)),
    ("motors x4", 72.0, "motors", 29.0), ("props x4", 16.0, "motors", 40.5),
    ("FC + Radxa + flow", 16.0, (0, 0, 10.0), (40, 40, 15)), ("camera + bracket", 10.0, (88, 0, 18), (20, 38, 20)),
    ("receiver", 18.6, (0, 0, 8.0), (82, 44, 6)), ("cartridge + 4S 1500", 191.4, (0, 0, -17.5), (79, 38, 48)),
    ("wiring, ESC leads", 20.0, (0, 0, 12.0), (60, 60, 10)),
]


def mass_props():
    pts = []  # (m kg, r m, box dims m or None)
    F, C = s.F / mm, s.C / mm
    for name, m, where, zz in PARTS:
        mk = m / 1000
        if where == "ring":
            for i in range(8):
                (x0, y0), (x1, y1) = s.CORNERS[i], s.CORNERS[(i + 1) % 8]
                for t in (0.25, 0.75):
                    pts.append((mk / 16, np.array([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, zz * mm]), None))
        elif where == "corners":
            for (x, y) in s.CORNERS:
                pts.append((mk / 8, np.array([x, y, zz * mm]), None))
        elif where == "tees":
            for (x, y) in s.TEES:
                pts.append((mk / 3, np.array([x * 0.8, y * 0.8, zz * mm]), None))
        elif where == "motors":
            for sx, sy in itertools.product((1, -1), (1, -1)):
                pts.append((mk / 4, np.array([sx * 54.1, sy * 54.1, zz]) * mm, None))
        else:
            pts.append((mk, np.array(where) * mm, np.array(zz) * mm))
    M = sum(p[0] for p in pts)
    cg = sum(p[0] * p[1] for p in pts) / M
    I = np.zeros((3, 3))
    for m_, r, box in pts:
        d = r - cg
        I += m_ * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        if box is not None:
            a, b, c = box
            I += np.diag([m_ * (b * b + c * c), m_ * (a * a + c * c), m_ * (a * a + b * b)]) / 12
    return M, cg, I


M, CG, INERTIA = mass_props()


EXTRA = []          # extra drone geoms (xml strings) to watch for collar contact
LAST_HITS = 0

def model(node="wheelD", mu=0.4, axle_mu=0.4):
    # reuse s2_node_wheels geometry, swap inertial + contact softness
    base = w.build(node, mu, axle_mu=axle_mu)
    import os; xml_path = f"/tmp/_catch_real_{os.getpid()}.xml"
    mujoco.mj_saveLastXML(xml_path, base)
    x = open(xml_path).read()
    import re
    x = re.sub(r'<inertial pos="[^"]*" mass="[^"]*" diaginertia="[^"]*"/>',
               f'<inertial pos="{CG[0]} {CG[1]} {CG[2]}" mass="{M}" fullinertia="{INERTIA[0,0]} {INERTIA[1,1]} {INERTIA[2,2]} {INERTIA[0,1]} {INERTIA[0,2]} {INERTIA[1,2]}"/>', x)
    x = x.replace('solref="0.002 1"', 'solref="0.004 0.5"')
    if EXTRA:
        i = x.index('<inertial', x.index('name="drone"')); j = x.index('/>', i) + 2
        x = x[:j] + ''.join(EXTRA) + x[j:]
    return mujoco.MjModel.from_xml_string(x)


TAU_SPIN = 0.08      # prop spin-down time constant, s (E)
CDA = 0.010          # m^2 (E)
GATE = 10 * mm       # start: at the nominal offset, 10 mm above first contact, flying the approach velocity (E)
APPROACH = 0.0


def fly(m, x, y, yaw_deg, tilt_deg, v_des, T_after=1.2, rec_every=None, watch=None):
    d = mujoco.MjData(m)
    q = np.zeros(4); mujoco.mju_euler2Quat(q, np.array([math.radians(tilt_deg), 0, math.radians(yaw_deg)]), "xyz")
    # find the touch height at the nominal offset, then start APPROACH seconds back along the path
    d.qpos[:3] = [x, y, 0.12]; d.qpos[3:7] = q
    for _ in range(800):
        mujoco.mj_forward(m, d)
        if d.ncon: break
        d.qpos[2] -= 0.25 * mm
    v_des = np.array(v_des)
    d.qpos[:3] = d.qpos[:3] + np.array([0, 0, GATE])   # gate: hovering at the offset, lowest contact GATE above the faces
    d.qvel[:] = 0; d.qvel[:3] = v_des
    mujoco.mj_forward(m, d)
    body = m.body("drone").id
    killed, t_kill, T_kill = False, None, None
    frames, n_after = [], int(T_after / m.opt.timestep)
    step, yaw0 = 0, math.radians(yaw_deg)
    hinge = [m.jnt_qposadr[j] for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    while True:
        R = d.xmat[body].reshape(3, 3); zb = R[:, 2]
        v = d.qvel[:3].copy(); om = d.qvel[3:6].copy()   # world linear, local angular
        drag = -0.5 * 1.2 * CDA * np.linalg.norm(v) * v
        if not killed:
            # velocity loop -> desired thrust vector; attitude loop -> torque (PX4-like, gentle)
            a_cmd = 3.0 * (v_des - v)
            Fv = M * (np.array([0, 0, g0]) + a_cmd) - drag
            T = max(0.0, float(np.dot(Fv, zb)))
            zd = Fv / np.linalg.norm(Fv)
            e_att = R.T @ np.cross(zb, zd)
            yaw = math.atan2(R[1, 0], R[0, 0])
            tau = INERTIA @ (np.array([400, 400, 0]) * e_att + np.array([0, 0, 100 * (yaw0 - yaw)]) - np.array([40, 40, 20]) * om)
            if d.ncon > 0:
                killed, t_kill, T_kill = True, d.time, T
        if killed:
            T = T_kill * math.exp(-(d.time - t_kill) / TAU_SPIN); tau = np.zeros(3)
        d.xfrc_applied[body, :3] = T * zb + drag
        d.xfrc_applied[body, 3:] = R @ tau
        if rec_every and step % rec_every == 0:
            frames.append([round(d.qpos[0] / mm, 2), round(d.qpos[1] / mm, 2), round(d.qpos[2] / mm, 2),
                           *[round(float(a), 5) for a in d.qpos[3:7]],
                           *([round(float(d.qpos[a]), 3) for a in hinge] or [0.0] * 8),
                           0 if killed else 1, round(T / (M * g0), 3)])
        mujoco.mj_step(m, d); step += 1
        if watch is not None:
            for ci in range(d.ncon):
                c = d.contact[ci]
                if c.geom1 in watch or c.geom2 in watch:
                    global LAST_HITS; LAST_HITS += 1
        if killed and d.time - t_kill > T_after: break
        if d.time > APPROACH + 1.0 and not killed: break   # never touched
    R = d.xmat[body].reshape(3, 3)
    return d.qpos[:3].copy(), math.degrees(math.atan2(R[1, 0], R[0, 0])), math.degrees(math.acos(min(1, R[2, 2]))), frames


def sweep(node="wheelD", mu=0.4):
    m = model(node, mu)
    ref = fly(m, 0, 0, 0, 0, [0, 0, -0.10])[0]
    offs = [(0, 0)] + [(d * mm * math.cos(a), d * mm * math.sin(a)) for d in (20, 40) for a in [k * math.pi / 4 for k in range(8)]]
    ok = []
    for (x, y), yaw, tilt, dr in itertools.product(offs, (-5, 0, 5), (0, 2), (1, -1)):
        a = math.atan2(y, x) if (x or y) else 0.0
        v = [dr * 0.15 * math.cos(a), dr * 0.15 * math.sin(a), -0.10]
        p, yf, tf, _ = fly(m, x, y, yaw, tilt, v)
        e = (p - ref) / mm
        ok.append(math.hypot(e[0], e[1]) < 0.5 and abs(yf) < 0.3 and tf < 0.3)
    return dict(node=node, mu=mu, centred_pct=round(100 * sum(ok) / len(ok), 1), runs=len(ok))


if __name__ == "__main__":
    print(f"mass {M*1000:.0f} g, CG z {CG[2]/mm:.1f} mm, I diag (kg m2) {np.diag(INERTIA).round(6)}")
    if len(sys.argv) > 1 and sys.argv[1] == "sweep":
        print(json.dumps(sweep(sys.argv[2], float(sys.argv[3]))))
