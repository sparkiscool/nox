"""Hub run (v4 17 Oct: fixed sun post, no pulley on the hub): bearing housing + motor plate + bar posts, generated.
python arm_run_hub.py [h] [gen_cm3]  -> out/box_hub_v4.stl (W coords) + box_hub_v4_result.json
Print: bed at Y 7 (sleeve end), plate up, tree supports under the motor seats. Build direction +Y.
Axle drive v4 is planetary: the drive torque reaches the hub through the axle motor's seat (belt pull toward the sun
plus the tangential reaction), not through 80T bolts."""
import sys, os, math
import numpy as np
import arm_layout as L, arm_cad as A
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
P = A.hub_parts(); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem("box_hub_v5" if getattr(L, "HUB_DETENT", False) else "box_hub_v4", H, drawn, cuts, env, build="+y", overhang=False)
# support: the bearing seats = sleeve cells next to the bore (Y 8..40)
pp.support("default", A.cylY(0, 0, 8, 40, 42.0) - A.cylY(0, 0, 7, 41, 39.5))
posts = {f"A{z:.0f}": (box(-16, 16, 56, 74, z - 6, z + 6), (0, 0, 60)) for z in L.POSTS_A}
posts.update({f"B{x:.0f}": (box(-x - 6, -x + 6, 56, 74, -16, 16), (-60, 0, 0)) for x in L.POSTS_B})
for nm_, (p, radial) in posts.items():
    for vec, nm in (((50, 0, 0), "+x"), ((-50, 0, 0), "-x"), ((0, 50, 0), "+y"), ((0, -50, 0), "-y")):
        pp.load(f"post {nm_} push {nm}", p, vec)                              # misuse: a hand on the bar end, 5 N·m
    pp.load(f"post {nm_} radial", p, radial)                                  # pull along the bar (pack extraction 20 N x3)
T_AX = 2.43e3 / math.hypot(L.MOTORS["axle"]["xc"], L.MOTORS["axle"]["zc"])   # 2.43 N.m axle torque (E) at the seat: 35 N tangential
for nm, M in L.MOTORS.items():
    xc, zc = M["xc"], M["zc"]
    seat = A.cylY(xc, zc, 50, 54, 15.5) - A.cylY(xc, zc, 49, 55, 11.5)
    pp.load(f"motor {nm} weight", seat, (0, -20, 0))                         # 4 g on a 300 g motor, axial
    if nm == "axle":                                                         # stall: both spans pull the 20T toward the sun (93 N E)
        r = math.hypot(xc, zc); ux, uz = -xc / r, -zc / r; tx, tz = -uz, ux
        for sgn in (1, -1):
            pp.load(f"axle drive {'+' if sgn > 0 else '-'}", seat, (93 * ux + sgn * T_AX * tx, 0, 93 * uz + sgn * T_AX * tz))
        continue
    tx, tz = (0, 1) if nm == "A" else (-1, 0)                                # B's belt runs toward the arm's -X
    pp.load(f"motor {nm} belt", seat, (30 * tx, 0, 30 * tz))                 # belt tension toward the belt's far end
if getattr(L, "HUB_DETENT", False):                                          # hub v5: detent tower, 2 springs (8.4 N) + roller reaction (2 x 11 N), E
    import detent as DT
    c = DT.CHOSEN; Dc = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); Pv, Av = Dc["P"], Dc["A"]
    dA = (Dc["H"] - Av) / np.linalg.norm(Dc["H"] - Av)
    pp.load("detent pivot", A.cylY(Pv[0], Pv[1], 66.3, 71.3, 4.0), (22 * Dc["u"][0], 0, 22 * Dc["u"][1]))
    pp.load("detent anchor", A.cylY(Av[0], Av[1], 66.3, 74.1, 4.0), (8.4 * dA[0], 0, 8.4 * dA[1]))
pp.accel_case("arm whip +x 3g", (3, 0, 0)); pp.accel_case("arm whip +z 3g", (0, 0, 3))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_hub_v5.stl" if getattr(L, "HUB_DETENT", False) else "out/box_hub_v4.stl")