"""Head v6 run (24 Oct 2026): carriage (plate, wing, eccentric boss, 3 wheel axles, rack web + teeth) + gripper v4d, generated
between them (head_v6.head_v6_parts). Head A is generated; head B = head A mirrored in X (post_head_v6.py).
python3 arm_run_head_v6.py [h] [gen_cm3]  -> out/box_head_A_v6.stl (head frame) + head_A_v6_result.json
Supports (the statically right path, C): the three wheel axles hold X and Y (the wheels roll along u), the rack teeth at the
pinion hold u (the pinion drives); one support set per pinion position (drone insert z0 95.6, bay z0 86.1).
Loads (E): insert 74 N = the head motor's stall through the 16T (0.59 N.m / 8 mm, misuse; the design push is 60 N), pull-out 50 N,
both through the guide block (fingers -> racks -> guide), with the rack's separating force (tan 20) on the teeth; gripper spring
>= 15 N per finger: the servo opens it, 2 x 15 N at the 12T = 30 N on the servo plate (and the 8.3 N splay at the pull, inside it)."""
import sys, os, math
import numpy as np
import arm_layout as L, arm_cad as A
import head_v6 as HD
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.75
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 22.0
P = HD.head_v6_parts("A"); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem("head_A_v6", H, drawn, cuts, env, build="-y", overhang=False)

def support_dofs(name, parts):
    idx = []
    for region, dofs in parts:
        el = pp.elems(region, pp.solid | pp.design); nd = pp.nodes_of(el)
        idx.append((nd[:, None] * 3 + np.asarray(dofs)).ravel())
        log(f"  support {name}: {len(el)} elements, dofs {dofs}")
    pp.supports[name] = np.unique(np.concatenate(idx))

y0, y1 = HD.PLATE_Y
wheels = [(A.cylY(x, u, y0, y1, 9.0 if abs(x - HD.ECC["x"]) < 1e-6 else 6.5) - A.cylY(x, u, y0 - 1, y1 + 1, 2.7), (0, 1)) for x, u in HD.WHEELS]
xr = HD.RACK_PITCH_X - 1.25
def teeth_at(z0):
    up = L_PIN_Z - z0
    return box(HD.WEB_X[0] + 1.0, HD.WEB_X[1] + 0.2, HD.WEB_Y[0], HD.RACK_TEETH_Y[1], up - 4.0, up + 4.0)
import arm_v5_layout as V
L_PIN_Z = V.PIN_Z
Z_DRONE, Z_BAY = max(HD.Z0S), L.Z0_BAY + 1.5
support_dofs("drone", wheels + [(teeth_at(Z_DRONE), (2,))])
support_dofs("bay", wheels + [(teeth_at(Z_BAY), (2,))])
guide = box(L.GUIDE_X[0] - 0.5, L.GUIDE_X[1] + 0.5, L.Y_C - 16, L.Y_C + 16, L.GUIDE_U[0] - 0.5, L.GUIDE_U[1] + 0.5)
splate = box(L.P_SH - 13.5, L.P_SH + 13.5, L.Y_C - 21, L.Y_C + 40.5, L.U_PLATE[0] - 0.5, L.U_PLATE[1] + 0.5)
t20 = math.tan(math.radians(20))
for nm, z0 in (("drone", Z_DRONE), ("bay", Z_BAY)):
    T = teeth_at(z0)
    for F, what in ((-74.0, "insert stall 74 N"), (50.0, "pull 50 N")):
        el_g = pp.elems(guide, pp.solid | pp.design); el_t = pp.elems(T, pp.solid | pp.design)
        nd_g, nd_t = pp.nodes_of(el_g), pp.nodes_of(el_t)
        Fv = np.zeros(pp.ndof)
        Fv[3 * nd_g + 2] = F / len(nd_g)                                    # pack reaction along u through the fingers and racks
        Fv[3 * nd_t + 0] = -abs(F) * t20 / len(nd_t)                         # separating force: pinion pushes the rack toward -X
        import printopt as PO
        pp.cases.append(PO.LoadCase(f"{what} at the {nm}", Fv, cls="misuse", support=nm))
for vec, nm in (((10, 0, 0), "+x"), ((-10, 0, 0), "-x"), ((0, 10, 0), "+y"), ((0, -10, 0), "-y")):
    pp.load(f"pack side {nm}", guide, vec, support="bay")
pp.load("servo opens the springs +x (2 x 15 N)", splate, (30, 0, 0), support="bay")
pp.load("servo opens the springs -x (2 x 15 N)", splate, (-30, 0, 0), support="bay")
pp.load("servo weight / whip -u", splate, (0, 0, -6), support="bay"); pp.load("servo whip +y", splate, (0, 6, 0), support="bay")
pp.accel_case("arm whip +x 3g", (3, 0, 0), support="bay"); pp.accel_case("arm whip +u 3g", (0, 0, 3), support="bay")
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_head_A_v6.stl")
