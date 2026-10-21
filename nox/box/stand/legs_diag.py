"""legs v1 diagnosis: as-built per-element failure index for one case, hot spots and their distance to the screws.
python legs_diag.py out/legs_v1_shifted.stl [case substring]"""
import os, sys, copy, json
import numpy as np, trimesh
os.environ["NOX_SETUP_ONLY"] = "1"
sys.argv = [sys.argv[0], "2.0", "100"] + sys.argv[1:]
stl = sys.argv[3]; key = sys.argv[4] if len(sys.argv) > 4 else "+X face (heavy)"
import runpy
g = runpy.run_path("legs_v1.py", run_name="diag")
import noxopt as NO, printopt as PO
P = g["parts"](); shift = g["shift"]
pp = g["PartProblem"]("diag", 2.0, shift(P["drawn"]), shift(P["cuts"]), shift(P["env"]), build="+z", overhang=True, pad=0)
sup = None
for s in P["regions"]["screws"]: sup = shift(s) if sup is None else sup + shift(s)
pp.support("default", sup)
LC = json.load(open("legs_loads.json"))
c = [c for c in LC["cases"] if key in c["name"]][0]
F = np.zeros(pp.ndof)
for reg, v in (("foot", c["foot"]), ("tie", c["tie"])):
    el = pp.elems(shift(P["regions"][reg]), pp.solid | pp.design); nd = pp.nodes_of(el)
    for a in range(3): F[3 * nd + a] += v[a] / len(nd)
pp.cases.append(PO.LoadCase(c["name"], F, cls=c["cls"]))
mesh = trimesh.load(stl)
Av = NO._voxelise(mesh, *pp.axes).ravel()
pb = pp.problem(); pb.void = (~Av) & ~pb.solid; pb.solid = np.zeros_like(pb.solid); pb.overhang = False; pb.R = 0.5
po = PO.PrintOpt(pb); rho = np.where(pp.solid, 1.0, Av.astype(float))
vol, gg, _, _, info, rp = po.evaluate(rho, 64.0, grads=False)
K = po.assemble(po.EMIN + rp ** po.PEN * (1 - po.EMIN))
U = po.solve(po.EMIN + rp ** po.PEN * (1 - po.EMIN), {"default": F[:, None]}, K=K)["default"][0][:, 0]
s = po.stresses(U[po.edof], rp)
f1, _, f2, _ = po.failure(s, PO.allowables(pb.material, c["cls"]))
f = np.maximum(f1, f2); f[~po.smask] = 0
cen = po.ecent - np.array([0, 0, 1.0])
scr = np.array([[x, 97.0, z] for x, z in P["screws"]])
d = np.min(np.linalg.norm(cen[:, None, :] - scr[None], axis=2), 1)
print(c["name"], "foot", c["foot"], "tie", c["tie"])
for lim in (0, 4, 8, 12, 20):
    m = d > lim; i = np.argmax(np.where(m, f, 0))
    print(f"  > {lim:2d} mm from a screw: worst {f[i]:.2f} at {cen[i].round(0)} (in {f1[i]:.2f} / across {f2[i]:.2f})")
top = np.argsort(-f)[:15]
for i in top: print(f"   {f[i]:5.2f} at {cen[i].round(0)}  d_screw {d[i]:.1f}")
np.save("out/diag_f.npy", f)
# reactions per screw (N): K u at the fixed dofs, grouped by the nearest screw
Kf = K if not isinstance(K, tuple) else K[0]
R = Kf @ U
fix = pp.supports["default"]; nodes = np.unique(fix // 3)
nx, ny, nz = pp.shape
ii = nodes // ((ny + 1) * (nz + 1)); jj = (nodes // (nz + 1)) % (ny + 1); kk = nodes % (nz + 1)
xyz = np.stack([ii, jj, kk], 1) * pp.h + np.asarray(pp.origin) - np.array([0, 0, 1.0])
scr16 = [(x, 97.0, z) for x, z in P["screws"]] + [(x, 185.8, z) for x, z in P["screws"]]
grp = np.argmin(np.linalg.norm(xyz[:, None, :] - np.array(scr16)[None], axis=2), 1)
Rn = R.reshape(-1, 3)[nodes]
print("screw reactions (N, on the leg):")
for k, s in enumerate(scr16):
    f = Rn[grp == k].sum(0); print(f"  {'near' if k < 8 else 'far '} ({s[0]:6.1f},{s[2]:7.1f}) Fy {f[1]:7.1f}  shear {np.hypot(f[0], f[2]):6.1f}")
