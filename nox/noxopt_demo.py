"""Reference run (12 Oct 2026): the whole generative pipeline on the benchmark holder, start to finish, the way every
part is run. Copy this file as the starting point of a part's run / build script and swap in the part's own Problem,
drawn bodies, cuts and envelope.   python noxopt_demo.py [h, default 4/3] [mass target, default 0.12]
Steps: layout (symmetry found automatically) -> prune thin members -> connectivity -> polished build, one half mirrored
-> function check, thin check (hits go back into the prune, build again), wrinkle, as-built strength -> STL.
Needs noxopt.py, printopt.py, noxopt_bench.py beside it; then python render_preview.py demo_holder.stl for the image."""
import sys, copy
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO, noxopt_bench as B

H = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0 / 3.0
VF = float(sys.argv[2]) if len(sys.argv) > 2 else 0.12
FLOOR, CLIP = 2.0, 0.5                                   # wall class minimum (README P7), build keep-out margin
log = lambda s: print(s[:300], flush=True)

# ---- problem. The holder mirrors in y; make the voxel masks exact mirrors (a cell centre on a feature edge breaks it)
pb = B.holder(H)
sh = pb.shape; so = pb.solid.reshape(sh); vo = pb.void.reshape(sh)
pb.solid = (so & so[:, ::-1, :]).ravel(); pb.void = (vo | vo[:, ::-1, :]).ravel() & ~pb.solid
pb0 = copy.copy(pb)                                      # original keep-outs, for the build and the strength recheck

# ---- exact bodies (mm): drawn = mounting + must-have only, cuts = holes and keep-outs, env = envelope
box = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
cyl = lambda x, y, z0, z1, r: mf.Manifold.cylinder(z1 - z0, r, r, 48).translate([x, y, z0])
def tm(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
bolts = [(x, y) for x in (5, 55) for y in (5, 35)]
drawn = box(22, 38, 14, 26, 26, 30) + box(26, 34, 0, 3, 14, 24) + box(26, 34, 37, 40, 14, 24) + box(56, 60, 15, 25, 8, 18)
cuts = box(10, 50, 8, 32, 6, 22) + (box(8, 52, 6, 34, 26, 30.5) - box(22, 38, 14, 26, 26, 30.5))
for x, y in bolts:
    drawn += box(x - 3, x + 3, y - 3, y + 3, 0, 6); cuts += cyl(x, y, -1, 31, 1.6) + cyl(x, y, 6, 31, 3.0)
env = box(0, 60, 0, 40, 0, 30); allowed = env - cuts; MIRROR_Y = 20.0      # None for a part with no mirror plane

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    if MIRROR_Y is not None:                             # symmetric part: one half built, mirrored
        half = S ^ box(-1e3, 1e3, -1e3, MIRROR_Y, -1e3, 1e3)
        S = half + half.mirror([0, 1, 0]).translate([0, 2 * MIRROR_Y, 0])
    return ((S + drawn) ^ env) - cuts

# ---- 1 layout, 2 prune
L = NO.Layout(pb, VF, log=log)
dens = L.run(checkpoint="demo_ck.npz")
L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log)
# ---- 3 build, 4 thin check on the built part; hits go back into the prune (at most 2 times)
for attempt in range(3):
    con = NO.connectivity(dens, L.pb0); assert con["ok"], con
    part = build(dens); P = tm(part)
    thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
    if not thin or attempt == 2: break
    L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin)
# ---- 5 checks
bodies = len(part.decompose())
fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
A = NO._voxelise(P, *[np.arange(n) * H + H / 2 for n in pb0.shape]).ravel().astype(float)
pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
rows, _ = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
P.export("demo_holder.stl")
print(f"RESULT bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g | function check "
      f"{fc[0]:.1f}/{fc[1]:.1f}/{fc[2]:.1f} | thin {len(thin)} | wrinkle {wr[0]:.2f}/{wr[1]:.2f} | worst "
      f"{max(w['inlayer'], w['across']):.2f} in {w['case']}")
ok = bodies == 1 and P.is_watertight and max(fc) < 0.5 and not thin
print("PASS" if ok else "FAIL: not ready to print"); sys.exit(0 if ok else 1)
