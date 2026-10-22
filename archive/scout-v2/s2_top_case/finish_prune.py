"""usage: python finish_prune.py <tag>   take the prune checkpoint as final: rebuild the pruned problem, load
runs/<tag>/ck_prune0.npz, one iteration to get the physical density, write density.npz + void_pruned.npy."""
import sys, copy, json, numpy as np
from scipy import ndimage as ndi
import noxopt as NO, noxopt_local as NL, spec_top as spec
tag = sys.argv[1]; d = f"runs/{tag}"
log = open(f"{d}/log.txt", "a")
def L(s): log.write(s + "\n"); log.flush(); print(s, flush=True)
pb, gr, geo = spec.problem(1.25)
z = np.load(f"{d}/density_raw.npz"); dens = z["rho_phys"]
cells, info = NO.thin_members(dens, pb, 2.0, 0.75)
pb2 = copy.copy(pb); v = pb.void | cells
lab, n = ndi.label((~v).reshape(pb2.shape)); lab = lab.ravel(); v |= ~np.isin(lab, np.unique(lab[pb2.solid])); pb2.void = v
des = ~pb.solid & ~pb.void; vf = 7.0 * 1000 / (des.sum() * pb.h ** 3)
lay = NL.Layout(pb2, vf, log=L, backend="gmg")
dens = lay.run(beta0=3, checkpoint=f"{d}/ck_prune0.npz", resume=f"{d}/ck_prune0.npz", time_limit=1.0)
lay.save(f"{d}/density.npz", dens); np.save(f"{d}/void_pruned.npy", pb2.void)
L(f"prune taken from its checkpoint (grey stop met: 1.5% at beta 32): connectivity {json.dumps(NO.connectivity(dens, pb2))}")
th = NO.thin_members(dens, pb2, 2.0, 0.75)[1]; L(f"thin after prune: {len(th)} {th}")
