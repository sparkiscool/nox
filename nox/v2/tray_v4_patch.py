"""Tray v4 = optimised tray v3 + copper-clad strip patch (16 Oct 20:08: only parts on hand). Exact booleans."""
import trimesh, numpy as np, manifold3d as mf
import s2_cartridge as C
def man(t):
    m = mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)); m.merge(); return mf.Manifold(m)
def tm(M):
    me = M.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts))
B = lambda *a: man(C.B(*a))
t = man(trimesh.load('s2_tray_v3.stl'))
fill = cut = None
for s in (-1, 1):
    X = lambda a, b: (min(s * a, s * b), max(s * a, s * b))
    cham = man(C.prism([(s * (C.BOSS_X1 - C.CHAMF_X), C.Y_TOP + 0.01), (s * (C.BOSS_X1 + 1), C.Y_TOP + 0.01),
                        (s * (C.BOSS_X1 + 1), C.Y_TOP - C.CHAMF_Y)], -1, C.Z_LEN + 1))
    f = (B(*X(19.0, 20.0), 38.5, 48, 55.5, 77.5) + B(*X(17.0, 19.1), 39.5, 42.5, 71.5, 77.5)        # old recess + window
         + B(*X(C.CAV_X, C.BOSS_X1), C.Y_BAND0 - 1.0, C.Y_BAND1 + 0.5, C.Z_STRIP0 - 1.5, C.Z_STRIP1 + 1.5)) - cham
    f = f + man(C.prism([(s * (C.BOSS_X1 - 1.5), C.Y_BAND1), (s * C.X_STRIP_FACE, C.Y_BAND1),
                         (s * (C.BOSS_X1 - 0.8), C.Y_BAND1 + 2.0)], C.Z_STRIP0, C.Z_STRIP1))
    c = (B(*X(C.BOSS_X1 - C.STRIP_REC, C.BOSS_X1 + 1), C.Y_BAND0, C.Y_BAND1, C.Z_STRIP0, C.Z_STRIP1)
         + B(*X(C.CAV_X - 1, C.BOSS_X1 - C.STRIP_REC + 0.01), C.WIN_Y0, C.WIN_Y1, C.WIN_Z0, C.WIN_Z1))
    fill = f if fill is None else fill + f; cut = c if cut is None else cut + c
t4 = (t + fill) - cut
cav = B(-C.CAV_X + 0.01, C.CAV_X - 0.01, C.Y_FLOOR1 + 0.01, C.Y_CAV1 - 0.01, C.Z_PACK0 + 0.01, C.Z_PACK1 - 0.01)
sh = man(C.shoe(seated=True)); sh2 = sh.translate([-0.05, 0, 0])
print("v3 %.2f g -> v4 %.2f g (+%.2f), pieces %d, in pack cavity %.3f mm3" % (
    t.volume() * 1.27e-3, t4.volume() * 1.27e-3, (t4.volume() - t.volume()) * 1.27e-3, len(t4.decompose()), (t4 ^ cav).volume()))
print("seated shoe vs tray %.3f mm3; shoe pushed 0.05 further: vs PETG %.3f mm3 (0 = only copper under it)" % (
    (t4 ^ sh).volume(), (t4 ^ sh2).volume()))
tm(t4).export('out/s2_cartridge_tray_v4.stl')
