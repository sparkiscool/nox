import sys, numpy as np, trimesh, scipy.ndimage as ndi
sys.path.insert(0, "/home/user/nox/nox")
import importlib.util
spec = importlib.util.spec_from_file_location("sc", "/home/user/nox/nox/solid_check.py"); sc = importlib.util.module_from_spec(spec); spec.loader.exec_module(sc)
from topo.demos import motor_mount as M
m = trimesh.load("/home/user/nox/runs/motor_mount/topo-v2/motor_mount.stl")
B = M.bodies()
o = B["drawn"].to_mesh(); t = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts)
pitch = 0.25
lo = m.bounds[0] - 2 * pitch; hi = m.bounds[1] + 2 * pitch
shape = tuple(int(np.ceil((hi[k] - lo[k]) / pitch)) for k in range(3))
V = sc.voxels(m, pitch, lo, shape); st = ndi.generate_binary_structure(3, 1)
d = ndi.distance_transform_edt(V) * pitch
for j, b in enumerate(t.split(only_watertight=False)):
    Dv = sc.voxels(b, pitch, lo, shape) & V
    ef = float(np.clip(0.8 * d[Dv].max(), 0.9, 2.0))
    Le, ne = ndi.label(d > ef, st); core = Dv & (d > ef)
    ve = np.bincount(Le.ravel())[1:]; mn = int(np.argmax(ve)) + 1
    off = core & (Le != mn)
    labs = np.unique(Le[off])
    print(j, "centre", np.round(b.centroid, 1), "dmax", round(float(d[Dv].max()), 2), "ef", ef, "core", int(core.sum()), "off", int(off.sum()), "ncomp", ne,
          "off comps", [(int(l), int(ve[l-1]) if l else 0) for l in labs])
    if off.sum():
        idx = np.argwhere(off); print("   off at", np.round(lo + (idx.mean(0) + .5) * pitch, 1), "span", np.round((idx.max(0) - idx.min(0) + 1) * pitch, 2))
        # local thickness: max d in the off blob vs ef
        for l in labs:
            if l: print("   blob", l, "max d", round(float(d[Le == l].max()), 2), "vol mm3", round(ve[l-1] * pitch**3, 2))
