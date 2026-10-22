"""usage: python run_top.py <generated cm3> <tag> [h]   layout -> prune thin -> connectivity, in runs/<tag>
Copy of the underside's run_layout.py with the local backend (noxopt_local.Layout)."""
import sys, os, time, json, copy
import numpy as np
from scipy import ndimage as ndi
import noxopt as NO
import noxopt_local as NL
import spec_top as spec
gen, tag = float(sys.argv[1]), sys.argv[2]
h = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
d = f"runs/{tag}"; os.makedirs(d, exist_ok=True)
log = open(f"{d}/log.txt", "a")
def L(s): log.write(s + "\n"); log.flush()
pb, gr, geo = spec.problem(h)
des = ~pb.solid & ~pb.void
vf = gen * 1000 / (des.sum() * pb.h ** 3)
L(f"S2 top case {tag}: generated target {gen} cm3 = volfrac {vf:.4f} of {des.sum()} design cells, h {pb.h}, R {pb.R}, "
  f"build {pb.build}, cases {len(pb.cases)}")
t = time.time()
lay = NL.Layout(pb, vf, log=L, backend="gmg")         # pardiso calibration at 1.15M dof ran out of memory (run a)
WARM = os.environ.get("S2T_WARM")                      # warm start (run d): a finished layout's density, new drawn bits
beta0 = int(os.environ.get("S2T_BETA0", 0))
if WARM and not os.path.exists(f"{d}/ck.npz"):
    zw = np.load(WARM); w = zw["rho"] if "rho" in zw.files else zw["x"]; x0 = np.where(pb.void, 0.0, np.where(pb.solid, 1.0, w))
    lay.sx.state = x0; L(f"warm start from {WARM}, beta0 {beta0}")
dens = lay.run(beta0=beta0, checkpoint=f"{d}/ck.npz", resume=f"{d}/ck.npz", time_limit=5.0 * 3600)
lay.save(f"{d}/density_raw.npz", dens)
L(f"layout done {time.time()-t:.0f}s, stopped {lay.stopped}")
removed = []
for r in range(3):
    cells, info = NO.thin_members(dens, lay.pb0, 2.0, 0.75)
    if not info: break
    L(f"prune {r + 1}: {len(info)} thin member(s) under 2.0 as built, {int(cells.sum())} cells: {info}")
    removed += info
    pb2 = copy.copy(lay.pb0); v = lay.pb0.void | cells
    lab, n = ndi.label((~v).reshape(pb2.shape)); lab = lab.ravel()
    v |= ~np.isin(lab, np.unique(lab[pb2.solid]))
    L(f"  pockets cut off: {int((v & ~(lay.pb0.void | cells)).sum())} cells voided")
    pb2.void = v
    x = lay.sx.state.copy(); x[v] = 0.0
    lay = NL.Layout(pb2, lay.volfrac, R=lay.R, eta=lay.eta, log=L, backend="gmg"); lay.sx.state = x
    dens = lay.run(beta0=3, checkpoint=f"{d}/ck_prune{r}.npz", resume=f"{d}/ck_prune{r}.npz", time_limit=1.5 * 3600)
lay.save(f"{d}/density.npz", dens)
np.save(f"{d}/void_pruned.npy", lay.pb0.void)
con = NO.connectivity(dens, lay.pb0)
L(f"pruned: {json.dumps(removed)}")
L(f"connectivity: {json.dumps(con)}")
L(f"done in {time.time()-t:.0f}s")
