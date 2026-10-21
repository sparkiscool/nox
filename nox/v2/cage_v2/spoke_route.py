"""Crown spoke starts (17 Oct, cage v3; v4 22:10: 3.0 rods, socket bodies r 3.65, nose bolt-head lane r 3.5): for each axis and pattern, the start point(s) on the crown for the lightest
spoke that keeps 12 off the swept props (socket bodies r 2.9 near the ends, strut_route.clearance) and clears the
microSD lane and the 3 hex-key lanes (layout_v3.lane_clear). Spokes land on the upper ring at the axis tee (the lower
ring is out of reach: 2.0 to 10.6 off the props, C). Crown footprint = over the 4 Radxa bosses (|x| <= 13.5,
|y| <= 31), start height 71 .. 75 (crown on the boss tops 68.81, E).
Objective: spoke length (start height fixed at 76 since 17 Oct 04:20) (a taller crown raises the drone's top), V spread = the search's own.
Run: python3 spoke_route.py -> spoke_route.json"""
import json, numpy as np, strut_route as sr, layout_v3 as L
out = {}
zs = np.array([76.0])                     # 17 Oct 04:20: crown prints flat on its boss pads, so every socket stays above the pad plane (68.81): start z 76 (C)
for axis, name in ((0, "nose"), (2, "tail"), (1, "side")):
    b = L.ring_point(axis, "U"); best = {}
    along = 0 if axis in (0, 2) else 1; across = 1 - along; sgn = 1 if axis in (0, 1) else -1
    a_vals = np.arange(-13.5, 13.51, 0.5) if along == 0 else np.arange(-31.0, 31.01, 1.0)
    c_vals = np.arange(0.0, 31.01, 1.0) if across == 1 else np.arange(0.0, 13.51, 0.5)
    for pat in ("single", "V"):
        cand = []
        for av in a_vals:
            if sgn * av < 0: continue                           # start on the near half of the crown
            for cv in c_vals:
                if pat == "V" and cv < 4.0: continue
                for z in zs:
                    pts = []
                    for s_ in ((1,) if pat == "single" else (1, -1)):
                        p = np.zeros(3); p[along] = av; p[across] = s_ * cv; p[2] = z; pts.append(p)
                    ok = True; L_ = 0.0; cmin = 1e9; lmin = 1e9
                    for p in pts:
                        c = sr.clearance(p, b, r_end=L.R_SOCK, r_rod=L.R_ROD); l = L.lane_clear(p, b)
                        if c < 12.0 or l < 0.0: ok = False; break
                        L_ += np.linalg.norm(b - p); cmin = min(cmin, c); lmin = min(lmin, l)
                    if ok: cand.append((L_ + 2 * (z - 71.0) * len(pts), [p.round(2).tolist() for p in pts], round(cmin, 2), round(lmin, 2)))
        cand.sort(key=lambda t: t[0])
        out[f"{name}_{pat}"] = dict(starts=cand[0][1], prop_clear=cand[0][2], lane_clear=cand[0][3], length=round(cand[0][0], 1), n_feasible=len(cand)) if cand else None
        print(name, pat, out[f"{name}_{pat}"], flush=True)
json.dump(out, open("spoke_route.json", "w"), indent=1)
