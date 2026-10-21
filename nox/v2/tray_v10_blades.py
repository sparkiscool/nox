"""Tray v10 = tray v9 (v8 generated run 8 + lead channels) + the two feeler-blade root posts and the blades' swept
clearance (17 Oct 11:07, Spark: latch return = the Scout v1 feeler gauge, 19425 0.305 x 12.7). Exact booleans:
v10 = (v9 + posts) - sweep. Geometry and sizing: s2_blade.py. Run next to s2_tray_v9.stl:
    python tray_v10_blades.py
Prints mass, bodies and the blade-vs-tray interference over the squeeze (must be 0), writes s2_tray_v10.stl."""
import numpy as np, trimesh, manifold3d as mf
import s2_cartridge as C, s2_blade as S
def man(t):
    m = mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)); m.merge(); return mf.Manifold(m)
def tm(M):
    me = M.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts))
if __name__ == "__main__":
    t9 = man(trimesh.load("s2_tray_v9.stl"))
    t10 = (t9 + man(S.tray_add())) - man(S.tray_cut())
    out = tm(t10)
    worst = 0.0
    keep = C.B(-30, 30, -5, 20, -5, S.Z_ROOT - 0.05)            # the free part of each blade (the clamp sits in its slot)
    for q in np.linspace(0, C.TRAVEL, 9):
        for side in (1, -1):
            b = S.blade(S.D0 + q, side).intersection(keep, engine="manifold")
            worst = max(worst, b.intersection(out, engine="manifold").volume)
    print("v9 %.2f g -> v10 %.2f g, bodies %d, blade vs tray worst %.4f mm3 (0 = clear)"
          % (t9.volume() * 1.27e-3, t10.volume() * 1.27e-3, len(t10.decompose()), worst))
    out.export("s2_tray_v10.stl")
