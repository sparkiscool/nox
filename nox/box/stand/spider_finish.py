"""Finish a spider run from its saved layout in fresh processes (run 2 was OOM-killed in wrinkle()).
python spider_finish.py prune  -> re-settle after removing the built thin members, saves stub_spider_v1_pruned.npz
python spider_finish.py build  -> smooth build, thin / function check, as-built strength, STL + result json
python spider_finish.py wrinkle -> smoothness only (own process, it is the memory peak)"""
import sys, os, json, copy, time
import numpy as np
os.environ.setdefault("NOX_SETUP_ONLY", "1")
sys.argv = [sys.argv[0], "4", "80"] + sys.argv[1:]
import spider_run as R
import noxopt as NO, arm_topo as AT
from arm_topo import tm, man, log

step = sys.argv[3]
pp, drawn, cuts, env = R.pp, R.drawn, R.cuts, R.env
pb = pp.problem(); H = pp.h; allowed = env - cuts
vf = 80e3 / (pp.design.sum() * H ** 3)

if step == "prune":
    z = np.load("stub_spider_v1_density.npz")
    L = NO.Layout(pb, vf, log=log, x0=1.0); L.sx.state = z["rho"].copy()
    thin = [dict(len_mm=7.6, radius_mm=0.4, at=[5.0, -4.1, -51.8]), dict(len_mm=6.5, radius_mm=0.4, at=[75.4, 1.8, -75.1])]
    L, dens, rem = NO.prune_thin(L, z["rho_phys"], floor_mm=AT.FLOOR, margin_mm=AT.CLIP + 0.25, log=log, built=thin, maxit=60)
    np.savez_compressed("stub_spider_v1_pruned.npz", rho_phys=dens, void=L.pb0.void)
    log(f"pruned {len(rem)}; saved")

elif step == "build":
    z = np.load("stub_spider_v1_pruned.npz"); dens = z["rho_phys"]
    pb0 = copy.copy(pb); pbp = copy.copy(pb); pbp.void = z["void"]
    log(f"connectivity: {NO.connectivity(dens, pbp)}")
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn),
                             clip_mesh=tm(allowed), clip_margin=AT.CLIP, sigma_mm=0.6 * H, floor_mm=AT.FLOOR, log=log)
    part = ((man(surf) + drawn) ^ env) - cuts; P = tm(part)
    thin = NO.thin_check(P, tm(drawn - cuts), AT.FLOOR, log=log)
    fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    P.export("out/stub_spider_v1_q.stl")
    A = NO._voxelise(P, *pp.axes).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A), title=pp.name)
    open("strength_report.txt", "w").write(rep)
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    res = dict(bodies=len(part.decompose()), watertight=bool(P.is_watertight), mass_g=P.volume / 1000 * 1.27,
               function_check=[round(float(x), 2) for x in fc], thin=len(thin), worst=round(max(w["inlayer"], w["across"]), 2),
               worst_case=w["case"], drawn_g=(drawn - cuts).volume() / 1000 * 1.27, bbox=(P.bounds[1] - P.bounds[0]).round(1).tolist(),
               strength_rows=[(r["case"], round(r["inlayer"], 2), round(r["across"], 2)) for r in rows])
    json.dump(res, open("stub_spider_v1_result.json", "w"), indent=1)
    log("RESULT " + json.dumps({k: v for k, v in res.items() if k != "strength_rows"}))

elif step == "wrinkle":
    import trimesh
    P = trimesh.load("out/stub_spider_v1_q.stl", force="mesh")
    wr = NO.wrinkle(P, pitch=0.5, drawn=tm(drawn - cuts), allowed=tm(allowed))
    log(f"WRINKLE p95 / p99 {wr[0]:.3f} / {wr[1]:.3f} (pitch 0.5)")
