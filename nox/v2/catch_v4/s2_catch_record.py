"""Final catch (45 deg collar, 8 down-slope PETG wheels, bare PETG mu 0.4): full check + recorded drops for the viewer."""
import math, json, numpy as np, mujoco
import s2_node_wheels as w, s2_collar_settle as s
mm = 1e-3
H = w.set_slope(45)

def record(node, x, y, yaw, tilt, v, T=1.3, every=50):
    m = w.build(node, 0.4)
    d = mujoco.MjData(m)
    q = np.zeros(4); mujoco.mju_euler2Quat(q, np.array([math.radians(tilt), 0, math.radians(yaw)]), "xyz")
    d.qpos[:3] = [x, y, 0.09]; d.qpos[3:7] = q
    for _ in range(600):
        mujoco.mj_forward(m, d)
        if d.ncon > 0: break
        d.qpos[2] -= 0.25 * mm
    d.qpos[2] += 3 * mm
    z_touch = d.qpos[2]
    frames = []
    # approach under power: 0.35 s straight in at v, kinematic, before the kill
    for i in range(35, 0, -1):
        t = i * 0.01
        frames.append([round((d.qpos[0] - v[0] * t) / mm, 2), round((d.qpos[1] - v[1] * t) / mm, 2),
                       round((z_touch - v[2] * t) / mm, 2), *[round(float(a), 5) for a in d.qpos[3:7]],
                       *[0.0] * 8, 1])
    d.qvel[:3] = v
    hinge = list(range(7, 15)) if node != "dome" else []
    n = int(T / m.opt.timestep)
    for i in range(n):
        if i % every == 0:
            wa = [round(float(d.qpos[j]), 3) for j in hinge] or [0.0] * 8
            frames.append([round(float(d.qpos[0]) / mm, 2), round(float(d.qpos[1]) / mm, 2), round(float(d.qpos[2]) / mm, 2),
                           *[round(float(a), 5) for a in d.qpos[3:7]], *wa, 0])
        mujoco.mj_step(m, d)
    return frames

# reference seat
mref = w.build("wheelD", 0.4); ref = w.drop(mref, 0, 0, 0, 0, 0, [0, 0, -0.1])[0] / mm
cases = [
 ("Centred", "wheelD", 0, 0, 0, 0, [0, 0, -0.10]),
 ("40 off, drifting out", "wheelD", 40*mm, 0, 5, 2, [0.15, 0, -0.10]),
 ("40 off diagonal", "wheelD", 28.3*mm, 28.3*mm, -5, 2, [0.106, 0.106, -0.10]),
 ("20 off, drifting in", "wheelD", 0, -20*mm, 5, 0, [0, 0.15, -0.10]),
 ("Fixed dome, 40 off (old)", "dome", 40*mm, 0, 5, 2, [0.15, 0, -0.10]),
]
out = {"seat": [round(a, 2) for a in ref], "cases": []}
for name, node, x, y, yaw, tilt, v in cases:
    fr = record(node, x, y, yaw, tilt, v)
    f = fr[-1]; R = np.zeros(9); mujoco.mju_quat2Mat(R, np.array(f[3:7])); R = R.reshape(3, 3)
    err = math.hypot(f[0] - ref[0], f[1] - ref[1]); yawf = math.degrees(math.atan2(R[1, 0], R[0, 0])); tiltf = math.degrees(math.acos(min(1, R[2, 2])))
    out["cases"].append(dict(name=name, node=node, dt=0.01, frames=fr,
                             final=dict(xy=round(err, 2), yaw=round(yawf, 2), tilt=round(tiltf, 2))))
    print(name, len(fr), "frames  final xy", round(err, 2), "yaw", round(yawf, 2), "tilt", round(tiltf, 2))
# geometry (mm)
geo = dict(F=s.F/mm, C=s.C/mm, zs=s.ZS/mm, zup=s.ZS/mm+16, prop_z=40.5, motor=54.1, disk=44.45,
           slope=45, z_lo=s.Z_LO/mm, z_hi=s.Z_HI/mm, h_axis=s.H_AXIS/mm,
           cart=dict(x=39.5, y=19, z0=-41.5, z1=6.5), wheel_r=4, wheel_w=3, proud=1.0)
wheels = []
for (x, y) in s.CORNERS:
    n, u, t = w.face_frame(x, y)
    dist = (s.H_AXIS - max(abs(x), abs(y))) * s.SA
    c = np.array([x, y, s.ZS]) + n * (dist + 1 * mm - 4 * mm)
    wheels.append(dict(c=[round(a / mm, 3) for a in c], axle=[round(float(a), 4) for a in t]))
geo["wheels"] = wheels
out["geo"] = geo
json.dump(out, open("catch_frames.json", "w"), separators=(",", ":"))
print("json KB", round(len(json.dumps(out, separators=(",", ":"))) / 1024))
