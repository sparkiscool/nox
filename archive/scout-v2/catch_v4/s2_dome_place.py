"""Where on the ring should the PETG domes sit so centring survives the highest friction?"""
import json, s2_collar_settle as s
s.Z_LO = s.ZS - (s.H_AXIS - 84 * s.mm) * s.TA          # 168 opening (sim max)
F = s.F
for sl in (0, 30, 55, 78):
    pts = []
    for (x, y) in ((F, 0), (-F, 0), (0, F), (0, -F)):
        if sl == 0:
            pts.append((x, y))
        elif x:
            pts += [(x, sl * s.mm), (x, -sl * s.mm)]
        else:
            pts += [(sl * s.mm, y), (-sl * s.mm, y)]
    s.DOMES = pts
    for mu in (0.20, 0.25):
        r = s.run("B", 0.30, mu); r["dome_offset_mm"] = sl; r["domes"] = len(pts)
        print(json.dumps(r), flush=True)
