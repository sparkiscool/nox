"""Run 9 rebuild with the two thin end-wall bars reinforced (r 1.2 balls along them, the run 6 practice), then checks."""
import sys, numpy as np
TAG = sys.argv[1]; PTS = [(-14.4, 7.6, 0.6), (14.4, 7.6, 0.6)]
sys.argv = ["tray_to9.py", "3.6", "--setup"]
import tray_to9 as T
NO, mf = T.NO, T.mf
d = np.load(f"{TAG}_density.npz"); dens = d["rho_phys"]
pts = np.array([[x, y, z] for (x0, y, z0) in PTS for x in np.linspace(x0 - 2.2, x0 + 2.2, 5) for z in (0.6, 1.4)] )
pts = np.vstack([pts, pts * [-1, 1, 1]])
surf = NO.smooth_surface(dens, T.pb0.solid, T.pb0.void, T.pb0.shape, T.H, T.pb0.origin, keep_mesh=T.tm(T.drawn - T.cuts),
                         clip_mesh=T.tm(T.allowed), clip_margin=T.CLIP, sigma_mm=0.6 * T.H, floor_mm=T.FLOOR, log=T.log,
                         reinforce=pts, reinforce_r=1.2)
S = T.man(surf); half = S ^ T.mbox(0, 1e3, -1e3, 1e3, -1e3, 1e3); S = half + half.mirror([1, 0, 0])
part = ((S + T.drawn) ^ T.env) - T.cuts
P = T.tm(part)
print("manifold volume %.3f cm3 = %.2f g, bodies %d, trimesh watertight %s" % (part.volume() / 1000, part.volume() * 1.27e-3, len(part.decompose()), P.is_watertight))
pc = T.print_check(P); print("print check", pc)
thin = NO.thin_check(P, T.tm(T.drawn - T.cuts), T.FLOOR, log=T.log)
fc = NO.function_check(P, T.drawn - T.cuts, T.cuts, T.allowed, log=T.log)
A = NO._voxelise(P, *[np.arange(n) * T.H + o + T.H / 2 for n, o in zip(T.pb0.shape, T.origin)]).ravel().astype(float)
print("under-floor material %.2f cm3 of generated" % ((A > 0.5) & T.under & ~T.pb0.solid).sum() * 1e-3 if False else
      "under-floor material %.2f cm3" % (((A > 0.5) & T.under & ~T.pb0.solid).sum() / 1000))
print("stiffness", T.stiffness_of(A))
import trimesh
out = trimesh.Trimesh(*(lambda me: (np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)))(part.to_mesh()))
out.export(f"s2_tray_{TAG}_fix.stl"); np.save(f"{TAG}_fix_vox.npy", A)
