"""Head v4 (cartridge long side along the axle, squeeze tangential): carriage + gripper body, head at z0 = 0.
python arm_run_head.py [h] [gen_cm3] [A|B]. Print: far end (Y 183) down, build -Y, tree supports.
Belt clamp v1 (17 Oct, belt_clamp.py): the toothed bed is drawn; belt pull 93 N (stall, E) on each end on its own."""
import sys, os
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.75
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
V = sys.argv[3] if len(sys.argv) > 3 else "A"
P = A.head_parts(V); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem(f"head_{V}", H, drawn, cuts, env, build="-y", overhang=False)
yb0, yb1 = L.Y_BAR
pp.support("default", box(-24.5, 24.5, yb0 - 0.5, yb1 + 3.8, -0.5, L.GLIDE_L + 0.5) - box(-20.0, 20.0, yb0 - 1, yb1 + 0.2, -1, L.GLIDE_L + 1))
sx = 1 if V == "A" else -1
def xr(x0, x1): return (min(sx * x0, sx * x1), max(sx * x0, sx * x1))
guide = box(*xr(L.GUIDE_X[0] - 0.5, L.GUIDE_X[1] + 0.5), L.Y_C - 16, L.Y_C + 16, L.GUIDE_U[0] - 0.5, L.GUIDE_U[1] + 0.5)
plate = box(*xr(L.P_SH - 13.5, L.P_SH + 13.5), L.Y_C - 21, L.Y_C + 40.5, L.U_PLATE[0] - 0.5, L.U_PLATE[1] + 0.5)
pp.load("insert push 20 N", guide, (0, 0, -20)); pp.load("extract pull 20 N", guide, (0, 0, 20))
for vec, nm in (((8, 0, 0), "+x"), ((-8, 0, 0), "-x"), ((0, 8, 0), "+y"), ((0, -8, 0), "-y")):
    pp.load(f"pack side {nm}", guide, vec)
pp.load("servo torque +x", plate, (16, 0, 0)); pp.load("servo torque -x", plate, (-16, 0, 0))
pp.load("servo weight / whip -z", plate, (0, 0, -6)); pp.load("servo whip +y", plate, (0, 6, 0))
lo, hi = P["clamp_ends"]                                                   # the two toothed spans of the belt clamp
pp.load("belt lower end -z 93 N (stall)", lo, (0, 0, -93)); pp.load("belt upper end +z 93 N (stall)", hi, (0, 0, 93))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, f"out/box_head_{V}_v5.stl")
