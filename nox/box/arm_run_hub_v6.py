"""Hub v6 run (24 Oct 2026): hub_v6.hub_v6_parts() through the noxopt pipeline (arm_topo.run_part), as hub v5 (h 2.5, overhang off,
build +y: sleeve end on the bed). python3 arm_run_hub_v6.py [h] [gen_cm3] -> out/box_hub_v6.stl (W) + box_hub_v6_result.json
Loads (E), all misuse class like hub v5:
  - per head, insert at the head motor's stall 74 N (0.59 N.m / r 8) and pull-out 50 N: the pinion reaction on the motor seat along
    the rail (-u insert, +u pull) with the rack's separating force (tan 20), and the carriage's couple (74 N between the pack at
    Y_C 175 and the rack at Y 68.5) through the wheels and the rail into its two posts: inner post -Y, outer post +Y (insert);
  - a hand on a rail: 50 N in +-X and +-Y on each head's two posts (misuse, as v5's per-post pushes, now per rail);
  - axle drive stall (93 N belt pull toward the sun + the 2.43 N.m torque at the seat), motor weights, detent pivot 22 N and anchor
    8.4 N (the detent placement of hub_v6.DET6), arm whip 3 g in X and Z."""
import sys, os, math
import numpy as np
import arm_layout as L, arm_cad as A, detent as DT
import hub_v6 as HV
from arm_topo import PartProblem, run_part, box, log

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 55.0
P = HV.hub_v6_parts(); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
pp = PartProblem("box_hub_v6", H, drawn, cuts, env, build="+y", overhang=False)
pp.support("default", A.cylY(0, 0, 8, 40, 42.0) - A.cylY(0, 0, 7, 41, 39.5))          # the bearing seats, as v5
M6 = HV.MOTORS6
F_IN, F_OUT = 74.0, 50.0
Y_PACK, Y_MESH = L.Y_C, 0.5 * (HV.V.PIN_TEETH_Y[0] + HV.V.PIN_TEETH_Y[1])
t20 = math.tan(math.radians(20))
def seat(M): return A.cylY(M["xc"], M["zc"], 50, 54, 15.5) - A.cylY(M["xc"], M["zc"], 49, 55, 11.5)
heads = {}
for h, r, p, fp in P["posts"]:
    heads.setdefault(h, []).append((r, p))
for h, pr in heads.items():
    (r_in, p_in), (r_out, p_out) = sorted(pr, key=lambda t: t[0])
    u = np.array((0.0, 0.0, 1.0)) if h == "A" else np.array((-1.0, 0.0, 0.0))     # along the rail, outward (arm frame)
    xs = np.array((1.0, 0.0, 0.0)) if h == "A" else np.array((0.0, 0.0, 1.0))     # the head's +X (the rack side; B mirrored: arm +Z)
    sx = 1.0 if h == "A" else -1.0                                                  # pinion side of the rack: A +X, B -X_B = arm -Z
    for F, nm in ((F_IN, "insert stall"), (-F_OUT, "pull-out")):
        Fc = abs(F) * (Y_PACK - Y_MESH) / (r_out - r_in) * (1 if F > 0 else -1)
        Fv = np.zeros(pp.ndof)
        def add(region, vec):
            el = pp.elems(region, pp.solid | pp.design); nd = pp.nodes_of(el)
            for a in range(3): Fv[3 * nd + a] += vec[a] / len(nd)
        add(seat(M6[h]), -F * u + sx * abs(F) * t20 * xs)                           # pinion on the motor: reaction along -u, pushed off the rack
        add(p_in, (0, -Fc, 0) - 0.5 * sx * abs(F) * t20 * xs)                       # carriage couple via the rail; the rack's push-off back through the wheels
        add(p_out, (0, Fc, 0) - 0.5 * sx * abs(F) * t20 * xs)
        import printopt as PO
        pp.cases.append(PO.LoadCase(f"head {h} {nm} {abs(F):.0f} N", Fv, cls="misuse"))
    both = p_in + p_out
    for vec, nm in (((50, 0, 0), "+x"), ((-50, 0, 0), "-x"), ((0, 50, 0), "+y"), ((0, -50, 0), "-y")):
        pp.load(f"rail {h} push {nm}", both, vec)
T_AX = 2.43e3 / math.hypot(M6["axle"]["xc"], M6["axle"]["zc"])
for nm, M in M6.items():
    pp.load(f"motor {nm} weight", seat(M), (0, -20, 0))
    if nm == "axle":
        xc, zc = M["xc"], M["zc"]; r = math.hypot(xc, zc); ux, uz = -xc / r, -zc / r; tx, tz = -uz, ux
        for sgn in (1, -1):
            pp.load(f"axle drive {'+' if sgn > 0 else '-'}", seat(M), (93 * ux + sgn * T_AX * tx, 0, 93 * uz + sgn * T_AX * tz))
D, S = P["detent"]
Pv, Av = D["P"], D["A"]; dA = (D["H"] - Av) / np.linalg.norm(D["H"] - Av)
pp.load("detent pivot", A.cylY(Pv[0], Pv[1], 66.3, 71.3, 4.0), (22 * D["u"][0], 0, 22 * D["u"][1]))
pp.load("detent anchor", A.cylY(Av[0], Av[1], 66.3, 74.1, 4.0), (8.4 * dA[0], 0, 8.4 * dA[1]))
pp.accel_case("arm whip +x 3g", (3, 0, 0)); pp.accel_case("arm whip +z 3g", (0, 0, 3))
if not os.environ.get('NOX_SETUP_ONLY'):
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_hub_v6.stl")
