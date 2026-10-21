"""Bay module v4 (Spark 16 Oct 19:28: each bay bolted to its neighbours, any one swaps out): one Scout bay, generated
alone and printed 4x. Balance leads dropped (Spark 17 Oct 01:18): bays charge through the main contact shoes only. python arm_run_baymod.py [h] [gen_cm3]
v4c..v4i (17 Oct): universal module, Scout receiver centred on the tab line, 9 round the ring at 30 deg (90 deg drone opening).
Supports: its joints to both neighbours (2 M5 lugs + nuts and a dovetail segment pair per side, axial slide-out; no casing). The bay ring chat
adds front feet, funnel seats and spider-tie bosses to the modules afterwards (nox/box-stand.md).
Cases: head insert push on the back plate, extract pull through the latch pockets, the contact shoe springs, a hand on the back plate
while fitting it, the pack at 3 g. Print: near end (Y 133) down, build +Y (post_v4.py), tree supports ~21 g (E)."""
import sys, os
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
B = A.bay_module_parts(); drawn, cuts, env = B["drawn"], B["cuts"], B["env"]; R = B["regions"]
pp = PartProblem("baymod", H, drawn, cuts, env, build="+y", overhang=True, pad=0)     # overhang filter ON (box-case Problem 2); printed near end (Y 133) down; pad 0: the AM filter's layer 0 (the bed) must be a real layer, a void pad layer supports nothing
pp.support("default", R["links"])
pp.load("insert push 20 N (TPU on the back plate)", R["back"], (0, 0, 20))
pp.load("extract pull 20 N (bolts in the pockets)", R["pockets"], (0, 0, -20))
pp.load("contact shoe spring +X 10 N", R["shoes_px"], (10, 0, 0)); pp.load("contact shoe spring -X 10 N", R["shoes_mx"], (-10, 0, 0))
for v, nm in (((0, 0, -20), "-z"), ((20, 0, 0), "+x"), ((0, 20, 0), "+y"), ((0, -20, 0), "-y")):
    pp.load(f"hand on the back plate 20 N {nm}", R["back"], v)
for v, nm in (((6, 0, 0), "+x"), ((0, 6, 0), "+y"), ((0, 0, -6), "-z")):
    pp.load(f"pack 3 g {nm}", R["posts"], v)
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_baymod_v4.stl")
