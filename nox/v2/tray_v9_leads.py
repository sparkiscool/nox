"""Tray v9 = tray v8 (generated run 8) + lead channels (17 Oct 02:40). Exact booleans on the DRAWN top band only.
Why: the strip's 16 AWG lead leaves the strip back through the window (z 61.15..64.15, y 39.8..42.2) into the pack
cavity, where the pack sits 0.5 off the wall (C, sections of v8): no path to the pack's leads in the lead room.
Channel: x 16.5..18.6 (cavity face 17 + 1.6 into the band), y 43.3..45.7, z 61.15 -> 77 (lead room), both sides;
link slot x 16.5..18.5, y 42.2..43.35 over the window's z joins the window to the channel. 2.1 x 2.4 for a 16 AWG
silicone lead (OD ~2.3, E: it squeezes). Skins: chamfer face 0.6 above the channel's top corner, recess 0.3 below
its outboard 0.1 (C)."""
import numpy as np, trimesh, manifold3d as mf, s2_cartridge as C
def man(t):
    m=mf.Mesh(vert_properties=np.asarray(t.vertices,np.float32),tri_verts=np.asarray(t.faces,np.uint32)); m.merge(); return mf.Manifold(m)
def tm(M):
    me=M.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:,:3],np.asarray(me.tri_verts))
mbox=lambda x0,x1,y0,y1,z0,z1: mf.Manifold.cube([x1-x0,y1-y0,z1-z0]).translate([x0,y0,z0])
CH = dict(x=(16.5, 18.6), y=(43.3, 45.7), z=(C.WIN_Z0, C.Z_LEN - C.END_W))
LINK = dict(x=(16.5, 18.5), y=(C.WIN_Y1, CH["y"][0] + 0.05), z=(C.WIN_Z0, C.WIN_Z1))
def cut():
    c = None
    for s in (1, -1):
        for d in (CH, LINK):
            a, b = sorted((s * d["x"][0], s * d["x"][1]))
            k = mbox(a, b, *d["y"], *d["z"]); c = k if c is None else c + k
    return c
if __name__ == "__main__":
    t8 = man(trimesh.load("s2_tray_v8.stl")); t9 = t8 - cut()
    print("v8 %.2f g -> v9 %.2f g, bodies %d" % (t8.volume()*1.27e-3, t9.volume()*1.27e-3, len(t9.decompose())))
    tm(t9).export("s2_tray_v9.stl")
