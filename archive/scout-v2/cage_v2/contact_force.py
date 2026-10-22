"""Peak contact force of each added part on the collar over the sweep (gate from argv), plus where it touches."""
import sys, math, itertools, json, numpy as np, mujoco, s2_feet_sim as fs, s2_catch_real as r
mm = 1e-3; gate = float(sys.argv[1])
m = r.model("wheelD", 0.4)
watch = {i for i in range(m.ngeom) if m.geom(i).name.startswith("x_")}
peak = {}; where = {}; runs = {}; hit = set()
orig = mujoco.mj_step; f6 = np.zeros(6)
def step(mm_, dd):
    orig(mm_, dd)
    for i in range(dd.ncon):
        c = dd.contact[i]
        g = c.geom1 if c.geom1 in watch else (c.geom2 if c.geom2 in watch else None)
        if g is None: continue
        mujoco.mj_contactForce(mm_, dd, i, f6); n = mm_.geom(g).name.rstrip("-1").rstrip("1").rstrip("-") if mm_.geom(g).name.startswith(("x_foot","x_leg","x_skid")) else mm_.geom(g).name
        hit.add(n)
        if abs(f6[0]) > peak.get(n, 0): peak[n] = abs(f6[0]); where[n] = [round(float(v), 1) for v in c.pos / mm]
mujoco.mj_step = step
offs = [(0, 0)] + [(d * mm * math.cos(a), d * mm * math.sin(a)) for d in (10, 20, gate) for a in [k * math.pi / 4 for k in range(8)]]
for (x, y), yaw, tilt, dr in itertools.product(offs, (-5, 0, 5), (0, 2), (1, -1)):
    a = math.atan2(y, x) if (x or y) else 0.0
    hit.clear(); r.fly(m, x, y, yaw, tilt, [dr * .15 * math.cos(a), dr * .15 * math.sin(a), -.1])
    for n in hit: runs[n] = runs.get(n, 0) + 1
print(json.dumps(dict(gate=gate, runs_of_300=runs, peak_N={k: round(v, 2) for k, v in peak.items()}, at_world_mm=where)))
