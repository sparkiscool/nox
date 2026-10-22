"""Record full-physics landings for the viewer (s2_catch_real.py model)."""
import math, json, numpy as np, mujoco
import s2_catch_real as r, s2_collar_settle as s, s2_node_wheels as w
mm = 1e-3
cases = [
 ("Centred", "wheelD", 0, 0, 0, 0, [0, 0, -0.10]),
 ("30 off, drifting out", "wheelD", 30*mm, 0, 5, 2, [0.15, 0, -0.10]),
 ("30 off diagonal", "wheelD", 21.2*mm, 21.2*mm, -5, 2, [0.106, 0.106, -0.10]),
 ("20 off, drifting in", "wheelD", 0, -20*mm, 5, 0, [0, 0.15, -0.10]),
 ("40 off, drifting out (outside gate)", "wheelD", 40*mm, 0, 5, 2, [0.15, 0, -0.10]),
 ("Fixed dome, 30 off (old)", "dome", 30*mm, 0, 5, 2, [0.15, 0, -0.10]),
]
mref = r.model("wheelD", 0.4); seat = r.fly(mref, 0, 0, 0, 0, [0, 0, -0.10])[0] / mm
out = {"seat": [round(a, 2) for a in seat], "cases": [], "mass_g": round(r.M * 1000)}
for name, node, x, y, yaw, tilt, v in cases:
    m = r.model(node, 0.4)
    p, yf, tf, fr = r.fly(m, x, y, yaw, tilt, v, T_after=1.0, rec_every=25)   # 200 Hz
    err = math.hypot(*(p[:2] / mm - seat[:2]))
    out["cases"].append(dict(name=name, node=node, dt=0.005, frames=fr, final=dict(xy=round(err, 2), yaw=round(yf, 2), tilt=round(tf, 2))))
    print(name, len(fr), "final xy", round(err, 2), "yaw", round(yf, 2), "tilt", round(tf, 2))
geo = dict(F=s.F/mm, C=s.C/mm, zs=s.ZS/mm, zup=s.ZS/mm+16, prop_z=40.5, motor=54.1, disk=44.45, slope=45,
           z_lo=s.Z_LO/mm, z_hi=s.Z_HI/mm, h_axis=s.H_AXIS/mm, cart=dict(x=39.5, y=19, z0=-41.5, z1=6.5),
           wheel_r=4, wheel_w=3, proud=1.0)
geo["wheels"] = []
for (x, y) in s.CORNERS:
    n, u, t = w.face_frame(x, y)
    dist = (s.H_AXIS - max(abs(x), abs(y))) * s.SA
    c = np.array([x, y, s.ZS]) + n * (dist + 1 * mm - 4 * mm)
    geo["wheels"].append(dict(c=[round(a / mm, 3) for a in c], axle=[round(float(a), 4) for a in t]))
out["geo"] = geo
json.dump(out, open("catch_frames_real.json", "w"), separators=(",", ":"))
