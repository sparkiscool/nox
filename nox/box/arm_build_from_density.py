"""Build a part from its saved layout density without the prune loop (used when prune_thin cannot re-run the layout).
python arm_build_from_density.py <part: bay|head|hub|stub_tile> <h> [gen_cm3]   -> out/<name>.stl + checks, no strength rerun."""
import sys, copy, json, math
import numpy as np, trimesh
import noxopt as NO
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, tm, man, FLOOR, CLIP, log

part = sys.argv[1]; H = float(sys.argv[2])
if part == "bay":
    P = A.bay_parts(); build = "-z"
elif part.startswith("head"):
    P = A.head_parts(part[-1] if part[-1] in "AB" else "A"); build = "-y"
elif part == "hub":
    P = A.hub_parts(); build = "+y"
elif part == "baymod":
    P = A.bay_module_parts(); build = "-y"
elif part == "ring":
    P = A.bay_ring_parts(); build = "-y"
else:
    import arm_casing as CS; P = CS.stub_tile_parts(); build = "-y"
drawn, cuts, env = P["drawn"], P["cuts"], P["env"]; allowed = env - cuts
pp = PartProblem(part, H, drawn, cuts, env, build=build, overhang=False)
d = np.load(f"{part}_density.npz"); dens = d["rho_phys"]
assert dens.size == pp.nel, (dens.size, pp.nel)
pb0 = pp.problem()
con = NO.connectivity(dens, pb0); log(f"connectivity: {con}")
surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn), clip_mesh=tm(allowed),
                         clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
S = man(surf); built = ((S + drawn) ^ env) - cuts
bodies = built.decompose(); main = max(bodies, key=lambda b: b.volume())
loose = sum(b.volume() for b in bodies) - main.volume()
me = main.to_mesh(); Pm = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
log(f"bodies {len(bodies)}, loose generated dropped {loose:.1f} mm3 (specks only; a drawn body loose would be a rerun)")
thin = NO.thin_check(Pm, tm(drawn - cuts), FLOOR, log=log)
fc = NO.function_check(Pm, drawn - cuts, cuts, allowed, log=log)
wr = NO.wrinkle(Pm, drawn=tm(drawn - cuts), allowed=tm(allowed))
out = (f"out/box_{part}_v4_W.stl" if part.startswith("head") or part == "baymod" else f"out/box_{part}_v3_W.stl" if part in ("hub", "ring") else f"out/box_{part}_v1_W.stl") if part != "stub_tile" else "out/case_stub_tile_v1_W.stl"
Pm.export(out)
res = dict(name=part, bodies=len(bodies), loose_mm3=round(loose, 1), mass_g=round(Pm.volume / 1000 * 1.27, 1), thin=len(thin),
           thin_list=thin[:6], function_check=[round(float(x), 2) for x in fc], wrinkle=[round(wr[0], 3), round(wr[1], 3)], stl=out)
log("RESULT " + json.dumps(res)); json.dump(res, open(f"{part}_build_result.json", "w"), indent=1)
