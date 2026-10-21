"""usage: python run_layout.py <generated cm3> <tag> [h]   layout -> prune thin (own loop) -> connectivity, in runs/<tag>
The prune follows noxopt.prune_thin (thin members as built become void, layout continues from sharpness 8), and after
each pass voids any open pocket the pruned cells cut off from the drawn features (else the stiffness matrix is singular)."""
import sys, os, time, json, copy
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
import numpy as np
from scipy import ndimage as ndi
import noxopt as NO
import spec
gen, tag = float(sys.argv[1]), sys.argv[2]
h = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
d = f"runs/{tag}"; os.makedirs(d, exist_ok=True)
log = open(f"{d}/log.txt", "a")
def L(s): log.write(s + "\n"); log.flush()
pb, gr, geo = spec.problem(h)
des = ~pb.solid & ~pb.void
vf = gen * 1000 / (des.sum() * pb.h ** 3)
L(f"S2 underside {tag}: generated target {gen} cm3 = volfrac {vf:.4f} of {des.sum()} design cells, h {pb.h}, R {pb.R}, cases {len(pb.cases)}")
t = time.time()
lay = NO.Layout(pb, vf, log=L)
dens = lay.run(checkpoint=f"{d}/ck.npz", resume=f"{d}/ck.npz", time_limit=5.5 * 3600)
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
    lay = NO.Layout(pb2, lay.volfrac, R=lay.R, eta=lay.eta, log=L); lay.sx.state = x
    dens = lay.run(beta0=3, checkpoint=f"{d}/ck_prune{r}.npz", resume=f"{d}/ck_prune{r}.npz", time_limit=2 * 3600)
lay.save(f"{d}/density.npz", dens)
np.save(f"{d}/void_pruned.npy", lay.pb0.void)
con = NO.connectivity(dens, lay.pb0)
L(f"pruned: {json.dumps(removed)}")
L(f"connectivity: {json.dumps(con)}")
L(f"done in {time.time()-t:.0f}s")
