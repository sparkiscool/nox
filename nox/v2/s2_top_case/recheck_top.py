"""as-built strength recheck at the spec loads (layer-aware index per case), load / support pads out of the stress set.
Copy of the underside's recheck.py on spec_top / geom_top."""
import numpy as np
import spec_top as spec, geom_top as G
import printopt as PO


def recheck(P, h=1.25, log=print):
    pb, gr, geo = spec.problem(h)
    occ = (gr.frac(P) >= 0.5) | pb.solid
    occ &= ~(gr.frac(G.U(geo["cuts"])) >= 0.5)
    hot = np.zeros(gr.nn, bool)
    for lc in pb.cases:
        hot[np.unique(np.where(lc.F != 0)[0] // 3)] = True
    for v in pb.supports.values(): hot[np.unique(v // 3)] = True
    excl = hot[gr.enodes].any(1)
    pb1 = PO.Problem(shape=pb.shape, h=h, origin=pb.origin, solid=occ, void=~occ, supports=pb.supports, cases=pb.cases,
                     material=pb.material, build=pb.build, overhang=False, R=0.5, sym=None, comp_factor=None,
                     stress_exclude=excl)
    po = PO.PrintOpt(pb1)
    _, _, _, _, info, _ = po.evaluate(np.where(occ, 1.0, 0.0), 64.0, grads=False)
    U = po._U; hotpts = []
    for ci, lc in enumerate(pb1.cases):
        if lc.cls is None: continue
        ue = U[:, ci][po.edof]; s = po.stresses(ue, np.where(occ, 1.0, 0.0))
        f1, _, f2, _ = po.failure(s, PO.allowables(pb1.material, lc.cls))
        f = np.maximum(f1, f2); f[~po.smask] = 0
        hotpts += [tuple(p) for p in gr.ecent[f > 1.0].round(2)]
    recheck.hot = sorted(set(hotpts))
    rows = []
    for r in info:
        if r and "inlayer" in r:
            w = r["where_inlayer"] if r["inlayer"] >= r["across"] else r["where_across"]
            rows.append(dict(case=r["case"], cls=r["cls"], idx=float(max(r["inlayer"], r["across"])),
                             inlayer=float(r["inlayer"]), across=float(r["across"]), at=[round(float(v), 1) for v in w]))
    return rows
