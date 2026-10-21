"""Move the corner nodes (chamfer start c) out so the node-built domes spread wider."""
import json, s2_collar_settle as s
s.Z_LO = s.ZS - (s.H_AXIS - 84 * s.mm) * s.TA
F = s.F
for c in (92, 100, 106):
    s.C = c * s.mm
    s.CORNERS = [(F, -s.C), (F, s.C), (s.C, F), (-s.C, F), (-F, s.C), (-F, -s.C), (-s.C, -F), (s.C, -F)]
    s.DOMES = None
    for mu in (0.20, 0.25, 0.30):
        r = s.run("B", 0.30, mu); r["c_mm"] = c; print(json.dumps(r), flush=True)
