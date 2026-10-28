"""Save the Overseer cage nodes drawn first (CLAUDE.md Drawn first, Spark 27 Oct): each node design whose drawn part passed
(runs/cage_nodes/<v>/node_<i>/drawn.json, `cage_node.py <i> --drawn`) is oriented, checked and saved, with its mirror where it has one.

Orientation: every socket mouth on the bed (the socket axis vertical, the bore printed round), scored by print_orient's measures:
no first-layer island under 20 mm2 lasting over 10 mm, then the least overhang. Print files print/overseer_cage_node_<n>_<v>.stl
(n the cage node; a mirror node is its own file, a mirrored part), solid_check on each exact file, a preview, and a decimated copy
in the drone frame in runs/overseer_layout/v5/saved/ for the assembly.

python3 nox/overseer/cage_nodes_save.py [node ...]
"""
import os, sys, json, subprocess
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
sys.argv, _args = sys.argv[:1], sys.argv[1:]
import cage_node as cn
import print_orient as po

VER = os.environ.get("NODE_VER", "v5")
RUN = os.path.join(ROOT, "runs", "cage_nodes", VER)
SAVED = os.path.join(ROOT, "runs", "overseer_layout", "v5", "saved")


def measures(m):
    n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]; zmin = m.bounds[0][2]
    bed = float(a[(n[:, 2] < -0.99) & (zc < zmin + 0.3)].sum())
    over = float(a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < zmin + 0.3))].sum())
    first, run = po.island_check(m)
    return bed, over, run


def orient(m, dirs):
    best = None
    for k, d in enumerate(dirs):
        R = trimesh.geometry.align_vectors(np.asarray(d, float), [0, 0, -1.0])
        mm = m.copy(); mm.apply_transform(R)
        bed, over, run = measures(mm)
        key = (run > 10.0, -round(bed / 10.0), over)                    # no thin-island run first, then the largest bed, then least overhang
        if best is None or key < best[0]: best = (key, k, mm, bed, over, run)
    _, k, mm, bed, over, run = best
    mm.apply_translation([-mm.bounds.mean(0)[0], -mm.bounds.mean(0)[1], -mm.bounds[0][2]])
    return mm, k, bed, over, run


def save(i):
    d = json.load(open(os.path.join(RUN, f"node_{i}", "drawn.json")))
    if not d["pass"]: return None
    src = trimesh.load(os.path.join(RUN, f"node_{i}", f"cage_node_{i}.stl"), force="mesh")
    dirs = [s["d"] for s in cn.bodies(i)["sockets"]]
    out = []
    for n_, mesh, ds in ((i, src, dirs), *(((cn.mirror_of(i), _mirror(src), [np.asarray(v) * [1, -1, 1] for v in dirs]),) if cn.N[i, 1] > 0.5 else ())):
        pm, k, bed, over, run = orient(mesh, ds)
        f = os.path.join(ROOT, "print", f"overseer_cage_node_{n_}_{VER}.stl"); pm.export(f)
        sc = subprocess.run([sys.executable, os.path.join(ROOT, "nox", "solid_check.py"), f], capture_output=True, text=True)
        ok = sc.returncode == 0
        prev = os.path.join(RUN, f"node_{i}", f"preview_{n_}.png")
        subprocess.run([sys.executable, os.path.join(ROOT, "nox", "render_preview.py"), f, "--mat", "PETG", "--out", prev], capture_output=True, text=True)
        dm = mesh.copy()
        if len(dm.faces) > 3000: dm = dm.simplify_quadric_decimation(face_count=3000)
        if n_ == i: dm.export(os.path.join(SAVED, f"cage_node_{i}.stl"))     # the assembly mirrors the design itself (layout_v5.saved_node)
        r = dict(node=int(n_), design=int(i), file=os.path.relpath(f, ROOT), mass_g=round(float(pm.volume) * 1.27e-3, 2), solid_check="PASS" if ok else "FAIL",
                 bed_mm2=round(bed), overhang_mm2=round(over), thin_island_run_mm=round(run, 1), bed_face=f"socket {k} mouth ({ds[k] if not hasattr(ds[k], 'tolist') else None})",
                 extents_mm=np.round(pm.extents, 1).tolist(), worst_index=d["worst"], worst_case=d["worst_case"], preview=os.path.relpath(prev, ROOT))
        r["bed_face"] = f"the mouth of socket {k} ({cn.bodies(i)['sockets'][k]['name']}{', mirrored' if n_ != i else ''})"
        out.append(r); print(json.dumps(r))
    return out


def _mirror(m):
    mm = m.copy(); mm.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))            # trimesh flips the winding of a reflection itself
    if mm.volume < 0: mm.invert()
    return mm


if __name__ == "__main__":
    nodes = [int(a) for a in _args] or cn.designs()
    res = []
    for i in nodes:
        if os.path.exists(os.path.join(RUN, f"node_{i}", "drawn.json")): res += save(i) or []
    old = json.load(open(os.path.join(RUN, "saved.json"))) if os.path.exists(os.path.join(RUN, "saved.json")) else []
    keep = {r["node"]: r for r in old}; keep.update({r["node"]: r for r in res})
    json.dump(sorted(keep.values(), key=lambda r: r["node"]), open(os.path.join(RUN, "saved.json"), "w"), indent=1)
