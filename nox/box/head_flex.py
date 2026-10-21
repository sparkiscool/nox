"""Gripper deflection of a built head vs the same envelope filled solid (C/M). python3 head_flex.py A out/box_head_A_v5_W.stl"""
import sys, os, copy, json
os.environ["NOX_SETUP_ONLY"] = "1"
import numpy as np, trimesh
import noxopt as NO, printopt as PO
V, stl = sys.argv[1], sys.argv[2]
sys.argv = ["arm_run_head.py", os.environ.get("HH", "1.75"), "14", V]
import arm_cad as A
if os.environ.get("ULO"): A.head_env_region.__defaults__ = (float(os.environ["ULO"]), None)
import arm_run_head as R
pp = R.pp; pb0 = pp.problem()
P = trimesh.load(stl, force="mesh")
built = NO._voxelise(P, *pp.axes).ravel() & ~pb0.void | pb0.solid
full = pb0.solid | pp.design
ijk = np.stack(np.unravel_index(np.arange(pp.nel), pp.shape), 1)
cen = ijk * pp.h + np.asarray(pp.origin) + pp.h / 2
gi = pp.elems(R.guide, pb0.solid)         # the drawn rack guide block cells
nodes = np.unique(pp.nodes_of(gi))
res = {}
only_full = os.environ.get("ONLY_FULL")
for nm, mask in ((("envelope full", full),) if only_full else (("built", built), ("envelope full", full))):
    pb = copy.copy(pb0); pb.void = ~mask; pb.solid = pb0.solid.copy()
    po = PO.PrintOpt(pb)
    Eel = np.where(mask, 1.0, 0.0)
    K = po.assemble(Eel)
    rows = []
    for i, lc in enumerate(pb.cases):
        sol = po.solve(Eel, {lc.support: lc.F[:, None]}, K=K)[lc.support]
        U = sol[0][:, 0].reshape(-1, 3); sol[1].free_memory(everything=True)
        d = np.linalg.norm(U[nodes], axis=1)
        rows.append((lc.name, round(float(d.max()), 4), round(float(np.abs(U[nodes]).mean(0).max()), 4)))
    res[nm] = dict(g=round(mask.sum() * pp.h ** 3 * 1.27e-3, 1), rows=rows)
    print(nm, res[nm]["g"], "g (voxel)")
    for r in rows: print(f"   {r[0]:34s} max {r[1]:.4f} mm")
json.dump(res, open(f"head_{V}_flex.json", "w"), indent=1)
