"""usage: python run_prune.py <tag> [h]   prune stage only, from runs/<tag>/density_raw.npz (the layout already ran);
resumes runs/<tag>/ck_prune<r>.npz. Same steps as run_top.py after the layout. Checkpoints every 5 iterations."""
import sys, os, time, json, copy
import numpy as np
from scipy import ndimage as ndi
import noxopt as NO
import noxopt_local as NL
import spec_top as spec
tag = sys.argv[1]; h = float(sys.argv[2]) if len(sys.argv) > 2 else 1.25
d = f"runs/{tag}"
log = open(f"{d}/log.txt", "a")
def L(s): log.write(s + "\n"); log.flush()
pb, gr, geo = spec.problem(h)
z = np.load(f"{d}/density_raw.npz"); dens = z["rho_phys"]
des = ~pb.solid & ~pb.void
vf = 7.0 * 1000 / (des.sum() * pb.h ** 3)
t = time.time(); pb0 = pb; x = np.where(pb.void, 0.0, z["rho"]); R = None; eta = (0.7, 0.5, 0.3)
removed = []; lay = None
for r in range(3):
    cells, info = NO.thin_members(dens, pb0, 2.0, 0.75)
    if not info: break
    L(f"prune {r + 1} (run_prune): {len(info)} thin member(s) under 2.0 as built, {int(cells.sum())} cells")
    removed += info
    pb2 = copy.copy(pb0); v = pb0.void | cells
    lab, n = ndi.label((~v).reshape(pb2.shape)); lab = lab.ravel()
    v |= ~np.isin(lab, np.unique(lab[pb2.solid]))
    pb2.void = v
    x = x.copy(); x[v] = 0.0
    lay = NL.Layout(pb2, vf, R=R, log=L, backend="gmg"); lay.sx.state = x
    dens = lay.run(beta0=3, checkpoint=f"{d}/ck_prune{r}.npz", resume=f"{d}/ck_prune{r}.npz", checkpoint_every=5,
                   time_limit=1.6 * 3600)
    pb0 = lay.pb0; x = lay.sx.state.copy(); R = lay.R
    lay.save(f"{d}/density_prune{r}.npz", dens)
    if lay.stopped == "time":
        L(f"prune {r + 1} stopped on time ({lay.stopped}): resume with the same command"); break
np.savez_compressed(f"{d}/density.npz", **dict(np.load(f"{d}/density_prune{r}.npz") if lay else z))
np.save(f"{d}/void_pruned.npy", pb0.void)
con = NO.connectivity(dens, pb0)
L(f"pruned: {json.dumps(removed)}")
L(f"connectivity: {json.dumps(con)}")
L(f"prune stage done in {time.time()-t:.0f}s")
