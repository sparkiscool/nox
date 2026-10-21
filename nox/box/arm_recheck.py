"""As-built strength recheck of a built mesh with the part's own load cases (the run script's Problem).
python arm_recheck.py <part> <h> <mesh_W.stl>"""
import sys, os, copy, json
import numpy as np, trimesh
os.environ["NOX_SETUP_ONLY"] = "1"
import noxopt as NO
part, H, mesh = sys.argv[1], sys.argv[2], sys.argv[3]
variant = part.split("_")[1] if "_" in part else None                      # head_A / head_B -> arm_run_head with the variant
sys.argv = [sys.argv[0], H] + ([ "10", variant] if variant else [])
R = __import__(f"arm_run_{part.split('_')[0]}")
pp = R.pp; pb0 = pp.problem()
P = trimesh.load(mesh, force="mesh", process=False)
A = NO._voxelise(P, *pp.axes).ravel().astype(float)
pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A), title=part)
print(rep)
w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
print("WORST", round(max(w["inlayer"], w["across"]), 2), w["case"])
json.dump(dict(worst=round(max(w["inlayer"], w["across"]), 2), case=w["case"], rows=[(r["case"], round(r["inlayer"], 2), round(r["across"], 2)) for r in rows]), open(f"{part}_recheck.json", "w"), indent=1)
