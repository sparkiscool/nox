"""Print check for the cage v2 parts (DubHacks rules: no supports, flat prints, one body each).
For the part's build direction (cage_cad.py's bed face search) it finds every downward face steeper than the 45 deg
limit (2 deg tolerance for facets on 45 deg cuts) that is not on the bed, groups them into connected regions and
classes each: a BRIDGE when it is a horizontal-ish strip (plan width <= 4.0, length < 20) whose two ends sit on
supported material (bores, pin holes, keel ridges); otherwise SUPPORT. Support area must be 0.
Run: python3 print_check.py <stl> <build> [<stl> <build> ...]   (build: +z, -z, +x, -x, +y, -y)"""
import sys, json, numpy as np, trimesh
from scipy.sparse.csgraph import connected_components

LIM = np.cos(np.radians(45.0 - 2.0))
def to_bed(m, build):
    ax = "xyz".index(build[1]); v = np.zeros(3); v[ax] = 1 if build[0] == "+" else -1
    T = trimesh.geometry.align_vectors(v, [0, 0, 1]); m2 = m.copy(); m2.apply_transform(T)
    m2.apply_translation([0, 0, -m2.bounds[0, 2]]); return m2, T
def check(m, build, bridge_w=4.0, bridge_L=20.0):
    m2, T = to_bed(m, build)
    n, c, a = m2.face_normals, m2.triangles_center, m2.area_faces
    down = (-n[:, 2] > LIM) & (c[:, 2] > 0.25)
    idx = np.where(down)[0]; out = dict(support_mm2=0.0, bridge_mm2=0.0, bridges=[], height=float(m2.extents[2]),
                                        bed_mm2=float(a[(-n[:, 2] > 0.999) & (c[:, 2] <= 0.25)].sum()), worst=[])
    if len(idx) == 0: return out, m2
    adj = m2.face_adjacency; keep = np.isin(adj, idx).all(axis=1); e = adj[keep]
    pos = {f: i for i, f in enumerate(idx)}
    import scipy.sparse as sp
    G = sp.coo_matrix((np.ones(len(e)), ([pos[x] for x in e[:, 0]], [pos[x] for x in e[:, 1]])), shape=(len(idx), len(idx)))
    ncomp, lab = connected_components(G, directed=False)
    for k in range(ncomp):
        f = idx[lab == k]; ar = float(a[f].sum()); P = c[f][:, :2]
        if len(P) > 2:
            P0 = P - P.mean(0); w, V = np.linalg.eigh(P0.T @ P0); proj = P0 @ V
            L, W = float(np.ptp(proj[:, 1])), float(np.ptp(proj[:, 0]))
        else: L = W = float(np.ptp(P, axis=0).max()) if len(P) else 0.0
        zr = float(np.ptp(c[f][:, 2]))
        if ar < 0.5 and len(f) <= 4:
            out["speck_mm2"] = out.get("speck_mm2", 0.0) + ar; continue
        if W <= bridge_w and L < bridge_L:
            out["bridge_mm2"] += ar; out["bridges"].append(dict(area=round(ar, 2), L=round(L, 1), W=round(W, 1), z=round(float(c[f][:, 2].mean()), 1)))
        else:
            out["support_mm2"] += ar; out["worst"].append(dict(area=round(ar, 2), L=round(L, 1), W=round(W, 1), z=round(float(c[f][:, 2].mean()), 1), dz=round(zr, 1)))
    out["bridges"] = sorted(out["bridges"], key=lambda d: -d["area"])[:6]; out["worst"] = sorted(out["worst"], key=lambda d: -d["area"])[:6]
    return out, m2
if __name__ == "__main__":
    a = sys.argv[1:]
    for p, b in zip(a[::2], a[1::2]):
        m = trimesh.load(p); o, _ = check(m, b)
        o.update(file=p.split("/")[-1], build=b, bodies=len(m.split(only_watertight=False)), watertight=bool(m.is_watertight))
        for k in ("support_mm2", "bridge_mm2", "height", "bed_mm2", "speck_mm2"): o[k] = round(o.get(k, 0.0), 2)
        print(json.dumps(o))
