"""Overseer cage: split every shell tube the frame model finds buckling (runs/overseer_frame/v1/members.json, buckle >= 1) at
mid-span: a node at its middle (5 out from the cage centre, on the centre plane if the tube crosses it), the two faces on the tube
each cut in two. Mirror pairs split alike. Edits nox/overseer/cage_v5_shell.json in place (the frame model's fix, 27 Oct).

python3 nox/overseer/cage_split.py
"""
import os, json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
P = os.path.join(HERE, "cage_v5_shell.json")
d = json.load(open(P)); M = json.load(open(os.path.join(ROOT, "runs", "overseer_frame", "v1", "members.json")))["members"]
N = [np.array(x, float) for x in d["nodes"]]; R = {tuple(sorted(r)) for r in d["rods"]}
T = [tuple(sorted(t)) for t in d["tris"]]
bad = [tuple(sorted(map(int, k.split()[1].split("-")))) for k, v in M.items() if k.startswith("tube") and v["buckle"] >= 1.0]
ctr = np.mean(N, 0); done = []
for e in bad:
    if e not in R: continue
    a, b = e; m = (N[a] + N[b]) / 2; out = m - ctr; m = m + 5.0 * out / np.linalg.norm(out)
    if abs(N[a][1] + N[b][1]) < 1.0: m[1] = 0.0
    k = len(N); N.append(m); R.discard(e); R |= {tuple(sorted((a, k))), tuple(sorted((b, k)))}
    faces = [t for t in T if a in t and b in t]; T = [t for t in T if t not in faces]
    for t in faces:
        c = [v for v in t if v not in (a, b)][0]; R.add(tuple(sorted((c, k)))); T += [tuple(sorted((a, c, k))), tuple(sorted((b, c, k)))]
    done.append((e, round(float(np.linalg.norm(N[a] - N[b])))))
d["nodes"] = [list(map(float, n)) for n in N]; d["rods"] = [list(r) for r in sorted(R)]; d["tris"] = [list(t) for t in T]
if done: d["note"] = d.get("note", "") + f" Frame model splits: {done}."
json.dump(d, open(P, "w"))
print("split", done, "->", len(N), "nodes,", len(R), "tubes")
