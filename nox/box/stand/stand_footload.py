"""Peak foot loads during an Overseer landing (for the stub spider v1 load cases).

Uses stand_landing.build() on the box-stand.md footprint and records the contact
force (normal + friction, world frame) on every foot pad every step.
Feet: back pair X +-140 Y -64, front pair X +-160 Y 225 (F, box-stand.md).
Output: out/footload.json, peak per foot and the state it came from.
17 Oct (legs chat): also reports the front feet (the legs' loads).
"""
import json, itertools, os, sys
import numpy as np
import mujoco
from stand_landing import build, _magnet, FUN_Y
from stand_model import ZF

FEET = [(140, -64), (-140, -64), (160, 225), (-160, 225)]          # F
OCC = {"empty": (0,) * 7, "full": (2, 2, 1, 2, 1, 1, 1)}            # E: 3 Overseer + 3 Scout packs


def run(model, dx, dy, h, drift, yaw, v0=0.3, cgz=220.0, T=1.0):
    d = mujoco.MjData(model)
    mujoco.mj_resetData(model, d)
    for _ in range(int(0.15 / model.opt.timestep)):
        d.qpos[7:10] = [0, 0, 5]; d.qvel[6:] = 0
        mujoco.mj_step(model, d)
    d.qpos[7:10] = [dx / 1000, FUN_Y / 1000 + dy / 1000, (cgz - ZF + h) / 1000]
    d.qpos[10:14] = [np.cos(np.radians(yaw) / 2), 0, 0, np.sin(np.radians(yaw) / 2)]
    d.qvel[6:9] = [drift[0], drift[1], -v0]; d.qvel[9:] = 0
    fid = [model.geom(f"foot{i}").id for i in range(len(FEET))]
    seat = np.array([0, FUN_Y / 1000, cgz / 1000]); mag = False
    peak = np.zeros((len(FEET), 3)); peakN = np.zeros(len(FEET)); f6 = np.zeros(6)
    static = None
    for k in range(int(T / model.opt.timestep)):
        mag = _magnet(model, d, seat, mag)
        mujoco.mj_step(model, d)
        F = np.zeros((len(FEET), 3))
        for c in range(d.ncon):
            con = d.contact[c]
            for j, g in enumerate(fid):
                if g in (con.geom1, con.geom2):
                    mujoco.mj_contactForce(model, d, c, f6)
                    fw = con.frame.reshape(3, 3).T @ f6[:3]          # contact frame -> world
                    F[j] += fw if con.geom2 == g else -fw          # force on the foot
        if k == 0: static = F.copy()
        n = np.abs(F[:, 2])
        for j in range(len(FEET)):
            if n[j] > peakN[j]: peakN[j] = n[j]; peak[j] = F[j]
    return peak, peakN, static


if __name__ == "__main__":
    os.makedirs("out", exist_ok=True)
    rows = []
    cases = []
    for (dx, dy), h, dr, occ in itertools.product(
            [(0, 0), (30, 0), (-30, 0), (0, 30), (0, -30), (30, 30), (-30, -30)], (50, 100),
            [(0, 0), (0.15, 0), (0, 0.15), (0, -0.15)], ("empty", "full")):
        cases.append((dx, dy, h, dr, 5.0, occ))
    for dx, dy in [(40, 0), (-40, 0), (0, 40), (0, -40)]:      # abuse case, box-stand.md
        for occ in ("empty", "full"):
            cases.append((dx, dy, 150, (0, 0), 5.0, occ))
    models = {o: build(FEET, occ=OCC[o])[0] for o in OCC}
    for dx, dy, h, dr, yaw, occ in cases:
        pk, pn, st = run(models[occ], dx, dy, h, dr, yaw)
        rows.append(dict(dx=dx, dy=dy, h=h, drift=dr, occ=occ, peak=pk.round(2).tolist(),
                         peakN=pn.round(2).tolist(), static=np.abs(st[:, 2]).round(2).tolist()))
    front = max(rows, key=lambda r: max(r["peakN"][2:]))
    out = dict(cases=len(rows), worst_front=front, front_peak_N=max(max(r["peakN"][2:]) for r in rows),
               front_peak_lateral_N=max(float(np.hypot(*np.array(r["peak"])[j, :2])) for r in rows for j in (2, 3)),
               back_peak_N=max(max(r["peakN"][:2]) for r in rows), rows=rows)
    json.dump(out, open("out/footload.json", "w"), indent=1)
    print("cases", len(rows))
    print("front foot peak normal N", round(out["front_peak_N"], 1), "lateral", round(out["front_peak_lateral_N"], 1),
          "back", round(out["back_peak_N"], 1))
    print("worst front:", {k: front[k] for k in ("dx", "dy", "h", "drift", "occ", "peak", "peakN")})
