"""Overseer arms: as-built strength split into the ARM and the JOINT band (27 Oct, Spark: "just assume those are good" for
the root joint). Same FE and indices as nox/topo build.strength (every real case, the built mesh voxelised on the part
grid, inserts at their modulus, elements touching a loaded or fixed node left out); the band is every cell within BAND mm
of the plate contact faces (the joint: the plate edges press into the root there). Prints and writes strength_band.json.

python3 nox/overseer/arm_strength_band.py <arm_topo_vN.py> front|rear <mesh.stl> [h] [band_mm]
"""
import os, sys, json, runpy
import numpy as np
script, which, mesh = sys.argv[1], sys.argv[2], sys.argv[3]
h = float(sys.argv[4]) if len(sys.argv) > 4 else (2.0 if which == "front" else 1.5)
BAND = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
src = open(script).read()                                       # the version wrapper sets its env, then load arm_topo_v1
for line in src.splitlines():
    if line.strip().startswith("os.environ.setdefault("):
        exec(line.strip(), {"os": os, "ROOT": ROOT, "HERE": HERE})
import trimesh
import arm_topo_v1 as A
from scipy import ndimage as ndi
from topo.part import voxelise
from topo.fem import Solver
from topo.material import MATERIALS, AXES, stiffness, allowables

p = A.setup_for(which)(h)
mat = MATERIALS[p.material]; ax = AXES[p.build][0]; C = stiffness(mat, ax)
P = trimesh.load(mesh, force="mesh")
ins = p.insert_cells
Av = voxelise(P, *p.axes).ravel() | ins
E = np.where(ins, p.stiff, np.where(Av, 1.0, 1e-9))
sol = Solver(p.shape, p.h, C, p.supports, active=Av | p.solid, backend="auto", log=lambda s_: None)
g = sol.grid
wg = mat.density * 1e-6 * p.h ** 3 * 9.81
F = np.zeros((p.ndof, len(p.cases)))
for q, c in enumerate(p.cases):
    F[:, q] = c.F
    if c.accel is not None:
        fe = np.tile(np.asarray(c.accel, float) * wg / 8.0, 8)
        F[:, q] += g.to_flat(g.scatter((Av.astype(float)[:, None] * fe[None, :])[None]))[:, 0]
U = sol.solve(E, F, [c.support for c in p.cases], keys=[c.name for c in p.cases])
edof = g.edof(); enodes = edof[:, ::3] // 3
hot = np.zeros(p.nn, bool)
for c in p.cases:
    hot[c.nodes] = True
for fx in p.supports.values():
    hot[fx.any(1)] = True
keep = Av & ~ins & ~hot[enodes].any(1)
sup = np.zeros(p.nel, bool); sup[p.support_cells["default"]] = True
band = (ndi.distance_transform_edt(~sup.reshape(p.shape)) * p.h <= BAND).ravel()
CB = C @ sol.B0
lay = [k for k, (i, j) in {3: (0, 1), 4: (1, 2), 5: (2, 0)}.items() if ax in (i, j)]
M = np.array([[1, -.5, -.5, 0, 0, 0], [-.5, 1, -.5, 0, 0, 0], [-.5, -.5, 1, 0, 0, 0], [0, 0, 0, 3, 0, 0], [0, 0, 0, 0, 3, 0], [0, 0, 0, 0, 0, 3.]])
cen = p.cell_centres()
worst = {}
for q, c in enumerate(p.cases):
    Axy, Az, As = allowables(mat, c.cls)
    for zone, mask in (("arm", keep & ~band), ("joint", keep & band)):
        idx = np.where(mask)[0]
        sg = U[:, q][edof[idx]] @ CB.T
        vm = np.sqrt(np.maximum(np.einsum("ei,ij,ej->e", sg, M, sg), 0.0))
        f = np.maximum(vm / Axy, np.sqrt((np.maximum(sg[:, ax], 0) / Az) ** 2 + sum(sg[:, k] ** 2 for k in lay) / As ** 2))
        j = int(np.argmax(f)); key = (zone, c.cls)
        if key not in worst or f[j] > worst[key][0]:
            worst[key] = (float(f[j]), c.name, cen[idx[j]].round(1).tolist())
out = {f"{z} {cl}": dict(index=round(v[0], 3), case=v[1], at=v[2]) for (z, cl), v in sorted(worst.items())}
for k, v in out.items():
    print(f"{k:16s} {v['index']:.3f}  {v['case']}  at {v['at']}")
json.dump(dict(band_mm=BAND, h=h, mesh=mesh, worst=out), open(os.path.join(os.path.dirname(mesh), "strength_band.json"), "w"), indent=1)
