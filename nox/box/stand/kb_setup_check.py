"""kb v8 setup audit (20 Oct): every access cut keeps >= 3 of drawn material all round (CLAUDE.md hard rules, attachments)."""
import os; os.environ["NOX_SETUP_ONLY"] = "1"
import json, numpy as np, trimesh
import stand_v7_topo as T, stand_v6_parts as S
from arm_cad import box, cylY, tm
pp, drawn, cuts, env, gen = T.kb_setup(2.0)
D = drawn - cuts; BIG = 1e3; y0, y1 = T.Y_BACK, S.KB_Y[1]; F = S.FLOOR
af = box(-BIG, BIG, -BIG, BIG, F, BIG)
out = {}
for sx in (1, -1):   # rear nut + driver r 5.5: ring 5.5..8.4 over the tube (Y -54..-50.5) must be drawn
    ring = (cylY(sx * 26, -191.0, y0 + 12.05, -50.55, 8.45) - cylY(sx * 26, -191.0, y0 + 11, -49, 5.5)) ^ af
    out[f"rear access {sx:+d}: ring 3 not drawn mm3"] = round((ring - D).volume(), 2)
hc = [cylY(x, z, 28.05, 39.95, 5.0) for x, z in S.JOINT_BOLTS]; pk = (hc[0] + hc[1] + hc[2] + hc[3]).hull()
hs = [cylY(x, z, 28.05, 39.95, 7.95) for x, z in S.JOINT_BOLTS]; sh = (hs[0] + hs[1] + hs[2] + hs[3]).hull()
out["joint heads: shell 3 not drawn mm3"] = round(((sh - pk) ^ af - D).volume(), 2)
for x, z in S.JOINT_BOLTS:  # hole through the end plate: 3 round it
    r = (cylY(x, z, 40.05, 45.95, 5.7) - cylY(x, z, 39, 47, 2.75)) ^ af
    out[f"joint hole ({x},{z}) ring 3 not drawn mm3"] = round((r - D).volume(), 2)
out["design cells"] = int(pp.design.sum()); out["grid"] = pp.shape; out["drawn g"] = round(D.volume() / 1000 * 1.27, 1)
from solid_check import solid_check
ok, info = solid_check(tm(D)); out["drawn-only solid check"] = ["PASS" if ok else "FAIL", info["bodies_plain"], info["necks"], info["thin_walls"][:3]]
print(json.dumps(out, indent=1, default=str))
