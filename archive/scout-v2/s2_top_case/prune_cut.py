"""usage: python prune_cut.py <tag>   prune without re-convergence (run f, 17 Oct: the prune restart dropped volume and
cut load paths every time): thin members under 2.0 voided pass by pass until none, then material cut off from the
drawn features voided (as run_layout's prune). Writes runs/<tag>/density.npz + void_pruned.npy."""
import sys, copy, json, numpy as np
from scipy import ndimage as ndi
import spec_top as S, noxopt as NO
tag = sys.argv[1]; d0 = f"runs/{tag}"
pb, gr, geo = S.problem(1.25)
z = dict(np.load(f"{d0}/density_raw.npz")); d = z["rho_phys"]; v = pb.void.copy()
log = open(f"{d0}/log.txt", "a")
def L(s): print(s, flush=True); log.write(s + "\n"); log.flush()
L(f"layout end: {json.dumps(NO.connectivity(d, pb))}")
for r in range(6):
    pb2 = copy.copy(pb); pb2.void = v
    cells, info = NO.thin_members(d, pb2, 2.0, 0.75)
    if not info: break
    v = v | cells; d = np.where(v, 0.0, d)
    L(f"prune_cut pass {r + 1}: {len(info)} thin member(s), {int(cells.sum())} cells")
lab, n = ndi.label((d > 0.5).reshape(pb.shape)); lab = lab.ravel()
keep = np.unique(lab[pb.solid]); keep = keep[keep > 0]
off = (d > 0.5) & ~np.isin(lab, keep); v |= off; d = np.where(v, 0.0, d)
pb2 = copy.copy(pb); pb2.void = v
con = NO.connectivity(d, pb2)
L(f"prune_cut: {int(off.sum())} cut-off cells voided; connectivity {json.dumps(con)}; volume {(d > 0.5).sum() * pb.h ** 3 / 1000:.2f} cm3")
z["rho_phys"] = d; z["void"] = v
np.savez_compressed(f"{d0}/density.npz", **z); np.save(f"{d0}/void_pruned.npy", v)
sys.exit(0 if con["ok"] else 1)
