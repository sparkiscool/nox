"""Head print pose from a W mesh (17 Oct 19:40, head A chat): glide legs (Y 74 face) down = +90 deg about X as an exact
axis permutation (x, y, z) -> (x, -z, y), then the mesh is cleaned so it reloads watertight (the run's mesh carries a few
near-coincident vertex pairs that a float32 STL round trip merges into zero-area flaps: merge at 1e-5, drop degenerate and
duplicate faces). Writes <stem>_W.stl (cleaned) and <stem>.stl (print pose) and prints the reload check.
python head_print_pose.py out/box_head_A_v5_W.stl [out/box_head_A_v5.stl]"""
import sys, numpy as np, trimesh

src = sys.argv[1]
dst = sys.argv[2] if len(sys.argv) > 2 else src.replace("_W.stl", ".stl")
m = trimesh.load(src, force="mesh", process=False)
m.merge_vertices(digits_vertex=5); m.update_faces(m.nondegenerate_faces()); m.update_faces(m.unique_faces()); m.remove_unreferenced_vertices()
assert m.is_watertight and len(m.split(only_watertight=False)) == 1, ("W mesh not clean", m.is_watertight)
m.export(src)
v = m.vertices; P = trimesh.Trimesh(np.c_[v[:, 0], -v[:, 2], v[:, 1]], m.faces, process=False); P.apply_translation(-P.bounds[0])
P.export(dst)
for path in (src, dst):
    q = trimesh.load(path, force="mesh")
    print(f"{path}: watertight {q.is_watertight}, bodies {len(q.split(only_watertight=False))}, {q.volume / 1000 * 1.27:.1f} g (M), extents {q.extents.round(1)}")
