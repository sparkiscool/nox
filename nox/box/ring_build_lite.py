import os, sys, json, numpy as np, trimesh
os.environ["NOX_SETUP_ONLY"] = "1"; os.environ["NOX_DROP_ZERO"] = "1"
import noxopt as NO, arm_cad as A, arm_layout as L
from arm_topo import PartProblem, tm, man, FLOOR, CLIP, log
H = 4.0
P = A.bay_ring_parts(); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]; allowed = env - cuts
pp = PartProblem("ring", H, drawn, cuts, env, build="-y", overhang=False)
dens = np.load("ring_density.npz")["rho_phys"]; pb0 = pp.problem()
surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn), clip_mesh=tm(allowed),
                         clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
S = man(surf); built = ((S + drawn) ^ env) - cuts
bodies = sorted(built.decompose(), key=lambda b: -b.volume())
for b in bodies[:15]:
    bb = np.array(b.bounding_box()); c = (bb[:3] + bb[3:]) / 2
    print(round(b.volume(), 1), "centre", c.round(1).tolist(), "angle", round(float(np.degrees(np.arctan2(c[2], c[0]))), 1), "r", round(float(np.hypot(c[0], c[2])), 1))
main = bodies[0]
dm = (drawn - cuts) - main
print("main g", round(main.volume() * 1.27e-3, 1), "drawn missing from main mm3", round(dm.volume(), 1))
tm(built).export("out/box_bay_ring_v3_raw.stl"); tm(main).export("out/box_bay_ring_v3_W.stl")
