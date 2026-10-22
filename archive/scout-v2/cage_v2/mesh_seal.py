"""Seal the zero-area triangles that degenerate-face cleanup drops (17 Oct 17:45: side node had 3 such holes, 9 open edges).
Every boundary loop of 3 edges gets its face back, wound opposite to its neighbours. Nothing else changes."""
import numpy as np, trimesh
def seal(m):
    m = m.copy(); m.merge_vertices()
    e = m.edges_sorted; u, c = np.unique(e, axis=0, return_counts=True); b = [tuple(x) for x in u[c == 1]]
    if not b: return m, 0
    adj = {}
    for a, d in b: adj.setdefault(a, set()).add(d); adj.setdefault(d, set()).add(a)
    # directed boundary edges as used by existing faces (a face uses a->b; the new face must use b->a)
    used = {}
    for f in m.faces:
        for i in range(3):
            a, d = int(f[i]), int(f[(i + 1) % 3]); used[(a, d)] = True
    new, done = [], set()
    for a, d in b:
        for w in adj[a] & adj[d]:
            tri = tuple(sorted((a, d, w)))
            if tri in done: continue
            done.add(tri)
            x, y, z = tri
            face = [x, y, z] if (y, x) in used else [x, z, y]     # existing face has y->x, so the new one runs x->y
            new.append(face)
    if new: m = trimesh.Trimesh(m.vertices, np.vstack([m.faces, np.array(new)]), process=False)
    return m, len(new)
if __name__ == "__main__":
    import sys
    t = trimesh.load(sys.argv[1]); s, n = seal(t)
    print(sys.argv[1], "faces added", n, "watertight", s.is_watertight, "winding", s.is_winding_consistent, "vol", round(s.volume, 1))
    if len(sys.argv) > 2: s.export(sys.argv[2])
