"""Print closure of a finished generated part (17 Oct 04:45): the smooth polish can leave fillets under a rising socket
that face down steeper than the 45 deg limit. Fill each layer to at least the layer above shrunk by 0.8 of a layer step
(walls >= 51 deg off horizontal) on a fine grid in the build frame, never into a cut or keep-out, smooth only the added
material (Gaussian on its distance field), union it back. Result: no face needs support; the polished surface is
untouched wherever it already printed.  close(part_mesh, cut_mesh, build, pitch) -> (mesh, added_mm3)"""
import numpy as np, trimesh, manifold3d as mf
from scipy import ndimage
from skimage.measure import marching_cubes
import noxopt as NO
import print_check as PC

def _man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def _tm(m):
    me = m.to_mesh(); t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    t.merge_vertices(digits_vertex=5); t.update_faces(t.nondegenerate_faces()); t.remove_unreferenced_vertices(); return t

def close(part, cuts, build, pitch=0.12, shrink=1.0, sigma=0.12):
    P, T = PC.to_bed(part, build)                      # bed frame: layers along +z, bed at z 0
    C = cuts.copy(); C.apply_transform(T); C.apply_translation([0, 0, -part.copy().apply_transform(T).bounds[0, 2]])
    lo = P.bounds[0] - 2.0; hi = P.bounds[1] + 2.0; lo[2] = 0.0
    xs, ys, zs = (np.arange(lo[i] + pitch / 2, hi[i], pitch) for i in range(3))
    A = NO._voxelise(P, xs, ys, zs)
    Fb = NO._voxelise(C, xs, ys, zs) if len(C.faces) else np.zeros_like(A)
    O = A.copy(); nz = O.shape[2]; ks = np.where(O.any(axis=(0, 1)))[0]; k0, k1 = ks[0], ks[-1]
    for k in range(k1 - 1, k0 - 1, -1):
        up = O[:, :, k + 1]
        if not up.any(): continue
        need = (ndimage.distance_transform_edt(up) > shrink + 1e-6) & ~Fb[:, :, k]
        O[:, :, k] |= need
    add = O & ~A
    vol = float(add.sum()) * pitch ** 3
    if vol < 0.05: return part, 0.0
    # smooth field of the closed set (only the added region changes): signed distance, Gaussian, marching cubes
    Oa = add | (ndimage.binary_dilation(add, iterations=3) & A)   # the added wedges + 3 cells of overlap into the part
    Op = np.pad(Oa, 2)                                 # closed surface: an empty layer round the grid (bed side too)
    sd = (ndimage.distance_transform_edt(~Op) - ndimage.distance_transform_edt(Op)) * pitch
    sd = ndimage.gaussian_filter(sd, sigma / pitch)
    v, f, _, _ = marching_cubes(sd, 0.0, spacing=(pitch, pitch, pitch)); v += np.array([xs[0], ys[0], zs[0]]) - 2 * pitch
    shell = trimesh.Trimesh(v, f, process=True)
    if shell.volume < 0: shell.invert()
    S = _man(shell)
    if S.status() != mf.Error.NoError or S.is_empty(): raise RuntimeError(f"closure shell not a manifold: {S.status()}")
    out = (_man(P) + S) - _man(C)                      # the polished surface stays wherever nothing was added
    flat = mf.Manifold.cube([1e4, 1e4, 1e4], center=True).translate([0, 0, -5e3])     # nothing below the bed
    out = out - flat
    res = _tm(out)
    Ti = np.linalg.inv(T); res.apply_translation([0, 0, part.copy().apply_transform(T).bounds[0, 2]]); res.apply_transform(Ti)
    return res, vol
