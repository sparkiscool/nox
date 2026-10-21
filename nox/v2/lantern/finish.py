"""Re-check a finished node build after dropping zero-volume slivers (boolean artefacts at the bore meeting point)."""
import sys, json, copy, numpy as np, trimesh
import node_topo as NT, noxopt as NO
name, tag = sys.argv[1], sys.argv[2]
j = json.load(open(f"{name}{tag}.json")); nd = dict(NT.NODES[name]); nd["gen"] = j["gen_cm3"]
pb, g = NT.problem(nd, j["h"]); drawn, cuts, allowed = g["drawn"], g["cuts"], g["allowed"]
m = trimesh.load(f"{name}{tag}.stl", force="mesh"); parts = m.split(only_watertight=False)
bodies = len([p for p in parts if abs(p.volume) > 1.0]); Pm = max(parts, key=lambda p: abs(p.volume))
Bm = NT.man(Pm); fc = (((drawn - cuts) - Bm).volume(), (Bm ^ cuts).volume(), (Bm - (allowed + drawn)).volume())
thin = NO.thin_check(Pm, NT.tm(drawn - cuts), NT.FLOOR, log=print)
j.update(bodies=bodies, watertight=bool(Pm.is_watertight), function=[round(v, 2) for v in fc], thin=len(thin), mass_g=round(Pm.volume / 1000 * NT.MAT.density, 2), faces=int(len(Pm.faces)))
Pm.export(f"{name}{tag}.stl"); json.dump(j, open(f"{name}{tag}.json", "w"), indent=1); print("FINISH", name, tag, bodies, j["watertight"], fc, len(thin), j["mass_g"])
