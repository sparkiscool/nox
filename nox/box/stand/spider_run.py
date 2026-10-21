"""Stub spider v1, one quadrant (all four are this STL). python spider_run.py [h] [gen_cm3]
Bed: the X = 0 joint face, build +x, overhang filter ON (no supports). Support: the tie tab on the ring boss.
Loads: every position's cases mapped into Q0 (spider_spec.load_cases). Pipeline: arm_topo.run_part (= noxopt_demo).
Needs on the path: noxopt, printopt (engine bundle), nox/box/arm_topo.py, nox/box/arm_cad.py, nox/box/arm_layout.py."""
import sys, os
import numpy as np
import printopt as PO
from arm_topo import PartProblem, run_part, log
import spider_spec as S

H = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0          # DEFAULT h 4 (bay ring precedent); h 3 next if lumpy
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0       # generated cm3 per quadrant, E DEFAULT
TIE_Y = float(os.environ.get("TIE_Y", S.TIE_Y))               # ring near face, follows the ring stack

P = S.parts(TIE_Y)
drawn, cuts, env, R = P["drawn"], P["cuts"], P["env"], P["regions"]
pp = PartProblem("stub_spider_v1", H, drawn, cuts, env, build="+x", overhang=True, pad=0)
# v1 run 1 (16 Oct, h 4, 50 cm3): tab-only support -> the back foot is a 165 mm cantilever, worst index 13.4 (landing).
# In the assembly the glued joint lands hand load to the neighbours and all four ties share the frame moment. Model:
#   "default": tab + both joint lands fixed (neighbours as rigid) for the part's own loads;
#   "tie":     tab only, carrying the land reactions the neighbours push back in (solid-domain FE, mapped by rotation).
pp.support("default", R["tab"] + R["lands"][0] + R["lands"][1])
pp.support("tie", R["tab"])


def region(key, idx):
    return R[key][idx] if idx is not None else R[key]


def force_vec(parts):
    F = np.zeros(pp.ndof)
    for key, idx, v in parts:
        el = pp.elems(region(key, idx), pp.solid | pp.design)
        if len(el) == 0: raise ValueError(f"empty region {key}{idx}")
        nd = pp.nodes_of(el)
        for a in range(3): F[3 * nd + a] += v[a] / len(nd)
    return F


own = S._merge_psu(S.load_cases())
for name, parts in own:                                        # several regions per case (the PSU bosses act together)
    pp.cases.append(PO.LoadCase(name, force_vec(parts), cls="misuse", support="default"))

# land reactions on the solid domain (E: the final layout carries them differently; good for size and direction)
po = PO.PrintOpt(PO.Problem(**{**pp.problem().__dict__, "overhang": False, "comp_factor": None}))
x1 = np.where(pp.void, 0.0, 1.0); Eel = po.EMIN + x1 ** po.PEN * (1 - po.EMIN); K = po.assemble(Eel)
Fs = np.stack([c.F for c in pp.cases], 1)
sol = po.solve(Eel, {"default": Fs}, K=K)["default"]; U = sol[0]; sol[1].free_memory(everything=True)
Rall = K @ U - Fs                                              # reactions (nonzero on the fixed dofs only)
lnodes = [pp.nodes_of(pp.elems(L_, pp.solid)) for L_ in R["lands"]]
tnodes = pp.nodes_of(pp.elems(R["tab"], pp.solid))
for i, (name, _) in enumerate(own):
    ra = [Rall[3 * nd[:, None] + np.arange(3), i].sum(0) for nd in lnodes]       # force from neighbour on land A, B
    rt = Rall[3 * tnodes[:, None] + np.arange(3), i].sum(0)
    if not name.startswith(("landing", "hand", "flange bolt pull")): continue
    log(f"  reactions {name:26s} land A {np.round(ra[0]).tolist()} land B {np.round(ra[1]).tolist()} tab {np.round(rt).tolist()}")
    # land A (X 0 plane) mates with land B of the quadrant at -90 (k 3); land B (Z 0 plane) with land A of the one at +90
    va = S.rot_xz(tuple(-ra[0]), 1); vb = S.rot_xz(tuple(-ra[1]), 3)
    pp.cases.append(PO.LoadCase(f"nbr {name[:22]} -> B", force_vec([("lands", 1, va)]), cls="misuse", support="tie"))
    pp.cases.append(PO.LoadCase(f"nbr {name[:22]} -> A", force_vec([("lands", 0, vb)]), cls="misuse", support="tie"))
log(f"{len(pp.cases)} load cases")

import noxopt as NO
_Layout = NO.Layout


class _DenseStart(_Layout):
    """volfrac ~0.02 in a big domain: start dense (noxopt x0 note, receiver 16 Oct) so a load path forms"""
    def __init__(self, pb, vf, **kw):
        kw.setdefault("x0", float(os.environ.get("X0", 0.3))); super().__init__(pb, vf, **kw)


NO.Layout = _DenseStart

if not os.environ.get("NOX_SETUP_ONLY"):
    os.makedirs("out", exist_ok=True)
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/stub_spider_v1_q.stl")
