"""Scout camera mount v2: the saved Scout nodes v1 (runs/scout_nodes/v1, PR 166) as the mount's push-on keep-outs. The mount slides
on along -x, so what a node sweeps is its y-z projection over its x range and back by the stroke: this writes each placed node's
y-z projection outline (filled, simplified 0.1) and x range, and every rod's true ends (the socket floors: eccentric sockets move
some rods up to 3.5 off the node centres), to runs/scout_camera_mount/v2/node_sweeps.json. camera_mount_v2.py reads it.

python3 nox/scout/v3/camera_mount_v2_nodes.py <dir with node_<i>.stl and result.json> <source note> [cage_shell.json the nodes were placed on]
Each node also gets its centre (from that cage), so the mount matches nodes and rods to any layout run by position, not by index.
"""
import os, sys, json
import numpy as np, trimesh
from shapely.geometry import Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_nodes", "v1", "placed")
NOTE = sys.argv[2] if len(sys.argv) > 2 else "runs/scout_nodes/v1"
OUT = os.path.join(ROOT, "runs", "scout_camera_mount", "v2", "node_sweeps.json")
CAGE = sys.argv[3] if len(sys.argv) > 3 else os.path.join(ROOT, "runs", "scout_layout", "v12", "cage_shell.json")


def projection(m):
    T = m.vertices[m.faces][:, :, 1:]                                     # (n, 3, 2) in (y, z)
    d1, d2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]; ok = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) > 1e-6
    U = unary_union([Polygon(t) for t in T[ok]]).buffer(0.05).buffer(-0.05)
    polys = list(U.geoms) if hasattr(U, "geoms") else [U]
    return [np.round(np.asarray(Polygon(p.exterior).simplify(0.1).exterior.coords)[:-1], 2).tolist() for p in polys if p.area > 0.5]


def main():
    out = dict(source=NOTE, note="y-z projection outlines (filled) and x range of each placed node; rods by their socket-floor ends", nodes={}, rods={})
    C = json.load(open(CAGE))["nodes"]; out["placed_on"] = os.path.relpath(CAGE, ROOT)
    for i in range(len(C)):
        f = os.path.join(SRC, f"node_{i}.stl")
        m = trimesh.load(f, force="mesh")
        out["nodes"][str(i)] = dict(centre=C[i], x=[round(float(m.bounds[0][0]), 2), round(float(m.bounds[1][0]), 2)], yz=projection(m))
        print(i, out["nodes"][str(i)]["x"], len(out["nodes"][str(i)]["yz"]), "outline(s)")
    r = json.load(open(os.path.join(os.path.dirname(SRC.rstrip("/")) if SRC.endswith("placed") else SRC, "result.json")))
    for x in r["rods"]:
        out["rods"][x["rod"]] = dict(a=x["floor_a"], b=x["floor_b"], ecc_mm=x.get("ecc_mm", 0.0))
    os.makedirs(os.path.dirname(OUT), exist_ok=True); json.dump(out, open(OUT, "w"), indent=0)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
