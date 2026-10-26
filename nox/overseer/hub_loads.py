"""Hub joint reactions of the Overseer cage (F, frame model): for each printed hub piece (front sled fhubB / fhubT,
chin, rear hub block rhubB, top plate tower feet tp, arm roots) the largest reaction over every dimensions L case.
python3 nox/overseer/hub_loads.py runs/overseer_system/<ver>   (reads params.json + cage_final.json, writes hub_loads.txt)"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import cage_opt2 as co

d = sys.argv[1]
p = json.load(open(os.path.join(d, "params.json"))); cf = json.load(open(os.path.join(d, "cage_final.json")))
res, m = co.evaluate(p, cf["topo"], cf["sizes"], detail=True)
U, names = res["_U"], res["_names"]
R = m.K @ U                                                       # reactions at the fixed dofs (no loads applied there)
groups = {}
for n in m.fixed:
    key = n.rstrip("+-0123456789") if not n.startswith("root") else "root"
    groups.setdefault(key, []).append(n)
out = []
for key, ns in sorted(groups.items()):
    best = (0.0, None, None, 0.0)
    for n in ns:
        i = m.idx[n]; F = R[6 * i:6 * i + 3]; M = R[6 * i + 3:6 * i + 6]
        fm = np.linalg.norm(F, axis=0); mm = np.linalg.norm(M, axis=0); j = int(np.argmax(fm))
        if fm[j] > best[0]: best = (float(fm[j]), n, names[j], float(mm[j]))
    out.append(f"{key:6s} ({len(ns)} nodes): max reaction {best[0]:7.0f} N at {best[1]} in '{best[2]}', moment there {best[3] / 1000:5.1f} N m")
txt = "\n".join(out); print(txt); open(os.path.join(d, "hub_loads.txt"), "w").write(txt + "\n")
