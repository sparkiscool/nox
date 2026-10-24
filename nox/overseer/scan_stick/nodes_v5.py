"""Scan stick v5 cage corner nodes (TPU, drawn, Spark rule: TPU nodes). One node per rod corner of the v5 cage (unit_topo_v5.cage_rods):
low back, top back, frame foot, top front, each side. A node is the convex hull of a 12 cube on the corner point and one socket
per rod (Ø9.2 = 3 wall round the Ø3.2 bore (C)); each bore starts where it clears the others by a 2.0 wall (two rods 22° apart
start 13.6 out), insertion 6 (C) or less where the socket would run into a printed part; trimmed 1.0 clear of the Pi, UBEC / buck and pack.
   python3 nodes_v5.py [out_dir]"""
import os, sys, json
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
_argv = sys.argv; sys.argv = [sys.argv[0]]
import unit_topo_v5 as U
sys.argv = _argv
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scan_stick_nodes", "v1")
R_BORE, WALL, CUBE, INS, GAP = 1.6, 3.0, 12.0, 6.0, 1.0

def cyl_along(p, d, s0, s1, r, n=32):
    """cylinder of radius r on the line p + s d, s in [s0, s1]"""
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    c = mf.Manifold.cylinder(s1 - s0, r, r, n)
    z = np.array([0, 0, 1.0]); v = np.cross(z, d); s = np.linalg.norm(v); cth = float(z @ d)
    if s < 1e-9:
        M = np.eye(3) if cth > 0 else np.diag([1, -1, -1.0])
    else:
        k = v / s; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); th = np.arctan2(s, cth)
        M = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    p0 = np.asarray(p, float) + s0 * d
    A = np.hstack([M, p0[:, None]])
    return c.transform(A.tolist())

def corners():
    """{corner point: [unit directions of the rods leaving it]} from the cage rods"""
    C = {}
    for name, p0, p1 in U.cage_rods():
        p0, p1 = np.round(np.array(p0, float), 3), np.round(np.array(p1, float), 3)
        d = (p1 - p0) / np.linalg.norm(p1 - p0)
        C.setdefault(tuple(p0), []).append((name, d)); C.setdefault(tuple(p1), []).append((name, -d))
    return C

def node(P, rods, avoid):
    P = np.array(P); body = [U.box(P[0] - CUBE / 2, P[0] + CUBE / 2, P[1] - CUBE / 2, P[1] + CUBE / 2, P[2] - CUBE / 2, P[2] + CUBE / 2)]
    bores, info = [], []
    for i, (nm, d) in enumerate(rods):
        s0 = 2.0
        for j, (_, e) in enumerate(rods):
            if i != j:
                th = np.arccos(np.clip(d @ e, -1, 1)); s0 = max(s0, (R_BORE + WALL / 1.5) / np.sin(th / 2))   # bores 2.0 apart at their start
        L = INS
        while L > 3.0:
            sock = cyl_along(P, d, 0.0, s0 + L, R_BORE + WALL)
            if (sock ^ avoid).volume() < 1e-3 or min((sock ^ a).volume() for a in [avoid]) < 1e-3: break
            L -= 0.5
        body.append(cyl_along(P, d, 0.0, s0 + L, R_BORE + WALL)); bores.append(cyl_along(P, d, s0, s0 + L + 1.0, R_BORE))
        info.append(dict(rod=nm, start_mm=round(float(s0), 1), insertion_mm=L))
    hull = mf.Manifold.batch_hull(body)
    keep = sum((U.box(b[0] - GAP, b[1] + GAP, b[2] - GAP, b[3] + GAP, b[4] - GAP, b[5] + GAP) for b in (U.PI_KEEP, U.EL_KEEP, U.PACK)), mf.Manifold())
    return hull - sum(bores, mf.Manifold()) - keep, info                                       # 1.0 clear of the Pi, the UBEC / buck and the pack

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    Sb, Hd = U.spine_bodies(), U.head_bodies()
    printed = (Sb["drawn"] - Sb["cuts"]) + (Hd["drawn"] - Hd["cuts"])
    avoid = printed
    res = {}
    for P, rods in sorted(corners().items()):
        if len(rods) < 2 or abs(abs(P[1]) - U.YC) > 1e-6: continue                 # rod ends in a drawn PETG node (the diagonal top): not a TPU node
        n, info = node(P, rods, avoid)
        clash = (n ^ printed).volume()
        tag = {U.X_BACK: "back"}.get(P[0], "front")
        kind = ("low" if P[2] < 100 else "top") + "_" + tag + ("_pos" if P[1] > 0 else "_neg")
        t = U.tm(n); path = os.path.join(OUT, f"scan_stick_node_{kind}.stl"); t.export(path)
        res[kind] = dict(at=[float(v) for v in P], rods=info, clash_mm3=round(float(clash), 2), g_tpu=round(t.volume / 1e3 * 1.21, 2), extents=np.round(t.extents, 1).tolist())
        print(kind, res[kind])
    json.dump(res, open(os.path.join(OUT, "nodes.json"), "w"), indent=1)
