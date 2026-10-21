"""bar_half v8 export (20 Oct 2026): the W mesh from stand_v7_topo_build.py -> clean W + print STL, both checked.
Clean-up: corners closer than 1e-3 mm are merged and the faces that collapse are dropped (the build leaves ~270 near-degenerate
slivers, 3.6e-12 mm2 M, that split off as a zero-area shell on a float32 write; volume change 0.0). Print pose = stand_v7_parts
print_pose (Y 46 face down) turned 45.12 deg about the bed normal: at exactly 45 deg one z-ray column of solid_check's voxeliser
lands on an edge and reads a 1-voxel second body that is outside the part at 0.05 / 0.1 pitch (as the arm hub, 0.12 deg turn).
python3 stand_v8_bar_export.py [in_W.stl]  -> box_bar_half_v8_W.stl, box_bar_half_v8.stl (+ out/ copies)"""
import sys, math, shutil, hashlib, numpy as np, trimesh
import stand_v6_parts as S
from solid_check import solid_check

def collapse(m, digits=3):
    m = m.copy(); m.merge_vertices(digits_vertex=digits); F = m.faces
    keep = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2]); m.update_faces(keep); m.remove_unreferenced_vertices()
    return m

src = sys.argv[1] if len(sys.argv) > 1 else "out/box_bar_half_v8_W.stl"
W = collapse(trimesh.load(src, force="mesh", process=False))
t = W.copy(); M = np.eye(4); M[:3, :] = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -S.LAP_Y[0]]], float); t.apply_transform(M)
t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(45.12), [0, 0, 1]))
t.apply_translation(-t.bounds[0]); t.apply_translation([5, 5, 0]); t = collapse(t)
W.export("box_bar_half_v8_W.stl"); t.export("box_bar_half_v8.stl")
for f in ("box_bar_half_v8_W.stl", "box_bar_half_v8.stl"):
    r = trimesh.load(f, force="mesh"); ok, info = solid_check(r)
    print(f, "solid_check", "PASS" if ok else "FAIL", "shells", len(r.split(only_watertight=False)), "watertight", r.is_watertight,
          "g", round(r.volume / 1000 * 1.27, 1), "extents", np.round(r.extents, 1).tolist(), "sha", hashlib.sha256(open(f, "rb").read()).hexdigest()[:12])
    shutil.copy(f, "out/" + f)
