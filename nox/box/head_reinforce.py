"""Head v5 as-built strength reinforcement (README "Generative parts" rule 7: a must-hold case over 1.0 gets local
reinforcement, at most 3 passes). Rebuilds from the pruned layout density with smooth_surface(reinforce=...) seeded at the
cells whose index is >= HOT in the failing cases; drawn bodies untouched (reinforcement is generated material only).
python head_reinforce.py <A|B> [h]  (slot pick: NOX_SLOT=<pick> NOX_REINFORCE=rebuild python3 head_reinforce.py B 1.75, then python3 post_v4.py heads) -> out/box_head_<V>_v5.stl (W), head_<V>_reinforce.json, head_<V>_hot.npy (the seeds:
a later NOX_SLOT rebuild passes the same seeds so the slot pick keeps the reinforcement)."""
import os, sys, copy, json
import numpy as np
V = sys.argv[1]; H = float(sys.argv[2]) if len(sys.argv) > 2 else 1.75
os.environ["NOX_SETUP_ONLY"] = "1"; sys.argv = ["x", str(H), "14", V]
import arm_run_head as R, noxopt as NO, printopt as PO
from arm_topo import tm, man, FLOOR, CLIP, log

HOT, R0, DR, PASSES = 0.85, 2.0, 1.0, 3          # DEFAULT: seed threshold, ball radius pass 1, growth per pass, max passes
pp, drawn, cuts, env = R.pp, R.drawn, R.cuts, R.env; allowed = env - cuts
pb0 = pp.problem(); dens = np.load(f"head_{V}_prune0_density.npz")["rho_phys"]
_rec = []
_fail = PO.PrintOpt.failure
def _wrap(self, s, A):
    out = _fail(self, s, A); _rec.append((out[0], out[2])); return out
PO.PrintOpt.failure = _wrap

def strength(P):
    A = NO._voxelise(P, *pp.axes).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    _rec.clear()
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A), title=f"head_{V}")
    pb1 = copy.copy(pbs); pb1.overhang = False; pb1.R = 0.5
    po = PO.PrintOpt(pb1); sm = po.smask; ec = po.ecent
    hot = []
    for r, (f1, f2) in zip(rows, _rec[:len(rows)]):
        if max(r["inlayer"], r["across"]) > 1.0:
            m = np.maximum(f1[sm], f2[sm]) >= HOT; hot.append(ec[sm][m])
    return rows, rep, (np.concatenate(hot) if hot else np.zeros((0, 3)))

seeds = np.zeros((0, 3)); hist = []
P = None
import trimesh
REBUILD = len(os.sys.argv) > 0 and "rebuild" in os.environ.get("NOX_REINFORCE", "")
if REBUILD:                                       # slot pick: one build from the pruned density with the saved seeds (no rerun)
    seeds = np.load(f"head_{V}_hot.npy"); hist_old = json.load(open(f"head_{V}_reinforce.json"))
    worst, hot, PASSES = 9.0, np.zeros((0, 3)), 1; R0 = hist_old[-1]["r"]
    log(f"rebuild: slot {os.environ.get('NOX_SLOT', '2.0')}, {len(seeds)} saved seeds, r {R0}")
else:
    P = trimesh.load(f"out/box_head_{V}_v5.stl", force="mesh")
    rows, rep, hot = strength(P)
    worst = max(max(r["inlayer"], r["across"]) for r in rows)
    hist.append(dict(pass_=0, worst=round(worst, 2), hot=len(hot)))
    log(f"pass 0: worst {worst:.2f}, {len(hot)} hot cells")
for k in range(1, PASSES + 1):
    if worst <= 1.0: break
    if len(hot): seeds = np.unique(np.round(np.concatenate([seeds, hot]), 2), axis=0)
    rr = R0 + DR * (k - 1)
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn), clip_mesh=tm(allowed),
                             clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log, reinforce=seeds, reinforce_r=rr)
    part = ((man(surf) + drawn) ^ env) - cuts; P = tm(part)
    rows, rep, hot = strength(P)
    worst = max(max(r["inlayer"], r["across"]) for r in rows)
    nb = len(part.decompose()); thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
    fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log); wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
    hist.append(dict(pass_=k, r=rr, seeds=len(seeds), worst=round(worst, 2), bodies=nb, thin=len(thin),
                     function_check=[round(float(x), 2) for x in fc], wrinkle=[round(wr[0], 3), round(wr[1], 3)],
                     mass_g=round(P.volume / 1000 * 1.27, 1), watertight=bool(P.is_watertight),
                     rows=[(r["case"], round(r["inlayer"], 2), round(r["across"], 2)) for r in rows]))
    log(f"pass {k}: r {rr}, seeds {len(seeds)}, worst {worst:.2f}, bodies {nb}, thin {len(thin)}, fc {fc}, {P.volume / 1000 * 1.27:.1f} g")
    log(rep)
    P.export(f"out/box_head_{V}_v5.stl"); np.save(f"head_{V}_hot.npy", seeds)
json.dump(hist, open(f"head_{V}_reinforce.json", "w"), indent=1)
log("REINFORCE " + json.dumps(hist[-1] if len(hist) > 1 else hist[0]))
