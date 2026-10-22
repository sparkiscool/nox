"""Tail strut root under the plate (16 Oct 21:50): the plate top centre strip is taken by the FC (top case chat:
diamond L1 26.46 round (1.9, 0), z 22..38) and the pack-lead lane (x -54..-36, |y| <= 8, z 22..36), and above them
is inside the prop keep-out, so the tail root goes under the plate's tail end (plate underside z 18.5). The underside
is built around it (Spark 20:56). Same search as strut_route.py: 12 off the swept disks (socket bodies r 2.9)."""
import numpy as np, json, strut_route as sr
best = None
for x in np.arange(-47.0, -30.0, 1.0):                  # under the plate (tail end about x -50, E), for a pad on its underside
    for z in np.arange(6.0, 15.6, 0.25):                      # socket body top <= plate underside 18.5
        B = np.array([x, 0.0, z]); R = np.array([-112.0, 0.0, sr.ZL])
        c = sr.clearance(B, R)
        if c < 12.0: continue
        L = np.linalg.norm(R - B)
        if best is None or L < best[0]: best = (L, x, z, c)
L, x, z, c = best
out = dict(bracket_socket_bottom=[x, 0.0, z], ring_point=[-112.0, 0.0, sr.ZL], centre_line_length=round(L, 1), prop_clearance=round(c, 2))
print(json.dumps(out)); r = json.load(open("strut_route.json")); r["tail"] = out; json.dump(r, open("strut_route.json", "w"))
