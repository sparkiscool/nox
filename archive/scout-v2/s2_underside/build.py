"""Polished build + checks. usage: python build.py <tag> [reinforce passes, default 3]
Writes runs/<tag>/s2_underside.stl (drone coords), s2_underside_print.stl (plate face on the bed), report.json."""
import sys, os, json, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO
import spec, geom as G
import recheck as RC

tag = sys.argv[1]
passes = int(sys.argv[2]) if len(sys.argv) > 2 else 3
d = f"runs/{tag}"
log = open(f"{d}/build.txt", "w")
def L(s): print(s, flush=True); log.write(s + "\n"); log.flush()
pb, gr, geo = spec.problem(1.0)
z = np.load(f"{d}/density.npz"); rho = z["rho_phys"]
pb0 = pb
if os.path.exists(f"{d}/void_pruned.npy"):
    import copy
    pb0 = copy.copy(pb); pb0.void = np.load(f"{d}/void_pruned.npy")
con = NO.connectivity(rho, pb0)
L(f"layout connectivity: bodies {con['bodies']}, detached {con['detached_interfaces']}, loose {con['loose_generated']}")

drawn = G.U(geo["drawn"]); cuts = G.U(geo["cuts"]); keep = G.U(geo["keep"])
env = geo["env"] - G.skin()
allowed = env - keep - cuts
# the plate face (z 18.5) is real: let the polish run 3 mm past it, then trim flat (pads flat on the plate and on the bed)
zp = G.PLATE_UNDER
bb = allowed.bounding_box()
above = G.box((bb[0] - 1, bb[3] + 1), (bb[1] - 1, bb[4] + 1), (zp - 0.01, zp + 3.0))
clip_allowed = allowed + (above - cuts - keep)
trim = G.box((bb[0] - 5, bb[3] + 5), (bb[1] - 5, bb[4] + 5), (bb[2] - 5, zp))
tm = G.to_trimesh
def to_man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))

R_REIN = float(os.environ.get("S2U_R_REIN", 1.5))
def build(reinforce=None):
    gen = NO.smooth_surface(rho, pb.solid, pb0.void, pb.shape, pb.h, pb.origin, keep_mesh=tm(drawn),
                            clip_mesh=tm(clip_allowed), clip_margin=0.5, sigma_mm=0.6 * pb.h, floor_mm=2.0,
                            reinforce=reinforce, reinforce_r=R_REIN, log=L)
    Gm = to_man(gen)
    return (((Gm ^ (allowed + above)) ^ trim) + drawn) - cuts

t0 = time.time()
hot = []
for p in range(passes + 1):
    P = build(np.array(hot) if hot else None)
    rows = RC.recheck(P, 0.75, log=L)
    tm(P).export(f"{d}/s2_underside_pass{p}.stl")
    worst = max(rows, key=lambda r: r["idx"])
    L(f"pass {p}: {P.volume()/1000:.2f} cm3 = {P.volume()/1000*1.27:.2f} g; worst {worst['idx']:.2f} {worst['case']} at {worst['at']}")
    over = [r for r in rows if r["idx"] > 1.0]
    if not over or p == passes: break
    new = [list(q) for q in RC.recheck.hot]
    L(f"  {len(new)} elements over 1.0 -> reinforcement balls r {R_REIN} mm")
    if not new: break
    hot += new
t = tm(P); t.export(f"{d}/s2_underside.stl")
bodies = len(P.decompose())
miss, incut, inkeep = NO.function_check(t, drawn, cuts, allowed + above, log=L)
thin = NO.thin_check(t, tm(drawn - cuts), 2.0, log=L)
try:
    wr = NO.wrinkle(t, drawn=tm(drawn - cuts), allowed=tm(allowed))
except Exception as e:
    wr = (float("nan"), float("nan")); L(f"wrinkle failed: {e}")
# print orientation: plate face down on the bed (turn over about x), z min = 0
tp = t.copy(); tp.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
tp.apply_translation(-tp.bounds[0]); tp.export(f"{d}/s2_underside_print.stl")
rep = dict(tag=tag, mass_g=P.volume() / 1000 * 1.27, vol_cm3=P.volume() / 1000, bodies=bodies,
           drawn_g=(drawn - cuts).volume() / 1000 * 1.27, watertight=bool(t.is_watertight),
           function_check=[miss, incut, inkeep], thin=thin, wrinkle=wr, cases=rows, reinforce_points=hot,
           bbox=t.extents.round(1).tolist(), zmin=float(t.bounds[0][2]), minutes=(time.time() - t0) / 60)
json.dump(rep, open(f"{d}/report.json", "w"), indent=1)
L(f"bodies {bodies}, watertight {t.is_watertight}, wrinkle p95/p99 {wr[0]:.2f}/{wr[1]:.2f}, lowest point z {t.bounds[0][2]:.2f}")
for r in rows:
    L(f"  {r['idx']:5.2f} (in {r['inlayer']:.2f} / across {r['across']:.2f})  {r['case']}  [{r['cls']}] at {r['at']}")
