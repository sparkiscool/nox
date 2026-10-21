"""Bay ring v3 (Spark 16 Oct 02:36: the 7 Scout bays are one structure, glued): one topology run over the whole ring,
then split into 7 plate-sized segments along the 22.5 deg lines (arm_cad.ring_segments). python arm_run_ring.py [h] [gen_cm3]
Supports: the 7 feet on the far wall. Cases: every bay's insert push and extract pull, a hand on two pads, the pin
springs, all packs at 3 g down and sideways, the box dropped 6 g. Print (segments): far-wall face down, build -Y."""
import sys, os, math
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
R = A.bay_ring_parts(); drawn, cuts, env = R["drawn"], R["cuts"], R["env"]
pp = PartProblem("ring", H, drawn, cuts, env, build="-y", overhang=False)
pp.support("default", R["feet"])
def rad(a, f): c, s = math.cos(math.radians(a)), math.sin(math.radians(a)); return (f * c, 0.0, f * s)
for a, pad, lips in zip(L.BAY_ANGLES, R["pads"], R["lips"]):
    pp.load(f"bay {a:.0f} insert push 20 N", pad, rad(a, 20.0))             # head pushes the pack home onto the pad
    pp.load(f"bay {a:.0f} extract 20 N", lips, rad(a, -20.0))               # head pulls the pack out through the lips
for a in (0.0, -90.0):
    k = list(L.BAY_ANGLES).index(a)
    pp.load(f"bay {a:.0f} hand on the pad 50 N", R["pads"][k], rad(a, -50.0))
k0 = list(L.BAY_ANGLES).index(0.0)
pp.load("bay 0 pins spring +y", R["blocks"][k0], (0, 10, 0)); pp.load("bay 0 pins spring -y", R["blocks"][k0], (0, -10, 0))
allpads = R["pads"][0]
for p in R["pads"][1:]: allpads = allpads + p
pp.load("all packs 3 g down", allpads, (0, 0, -5.3 * 7)); pp.load("all packs 3 g +x", allpads, (5.3 * 7, 0, 0))
pp.accel_case("box dropped 6g -z", (0, 0, -6)); pp.accel_case("box dropped 6g +x", (6, 0, 0))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_bay_ring_v3.stl")
