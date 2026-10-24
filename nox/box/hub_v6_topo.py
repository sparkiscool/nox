"""Hub v6 on the topo engine (24 Oct 2026): hub_v6.hub_v6_parts(), build +y (sleeve end on the bed, as hub v5). The hub is cut from the
arm system at its joints: the bearing seats (support), the motor seats, the rail posts and the detent tower (loads). python3 hub_v6_topo.py [h] [gen_cm3] [out_dir] -> <out_dir>/box_hub_v6.stl (W) + result.json (topo engine)
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
from arm_cad import box
from topo import Part, Settings, optimise

H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 55.0

def setup(h):
  P = HV.hub_v6_parts(); drawn, cuts, env0 = P["drawn"], P["cuts"], P["env0"]
  pp = Part("box_hub_v6", h, drawn, cuts, env0, build="+y")
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
          pp.load_multi(f"head {h} {nm} {abs(F):.0f} N", [
              (seat(M6[h]), tuple(-F * u + sx * abs(F) * t20 * xs)),                  # pinion on the motor: reaction along -u, pushed off the rack
              (p_in, tuple(np.array((0, -Fc, 0)) - 0.5 * sx * abs(F) * t20 * xs)),    # carriage couple via the rail; the push-off back through the wheels
              (p_out, tuple(np.array((0, Fc, 0)) - 0.5 * sx * abs(F) * t20 * xs))])
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
  return pp

if __name__ == "__main__":
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HV.REPO, "runs", "hub", "v6")
    if os.environ.get('NOX_SETUP_ONLY'):
        setup(H); sys.exit(0)
    res = optimise(setup, H, GEN, os.path.abspath(out), "box_hub_v6", settings=Settings(), git=False)
    print({k: res.get(k) for k in ("name", "solid_ok", "mass_g", "total_s")}, res["strength"]["worst"])
