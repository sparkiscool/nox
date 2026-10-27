"""Overseer arms v2: the cage's joint reactions on the arms, from the cage frame model (`nox/overseer/cage_frame.py`,
read-only: its build and its cases). For each case, the 3 spokes at each arm tip and the 2 L2 braces at each front arm
beam are summed into one force on the arm (axial N x the link's direction away from the arm; the side forces are
<= 11 N, dimensions System Overseer v5). The left arms (1 front, 2 rear) are written; the right arms are their mirrors.

python3 nox/overseer/arm_joint_loads.py  -> runs/overseer_arms/v2/joint_loads.json
"""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
sys.argv = sys.argv[:1]
import cage_frame as C

OUT = os.path.join(ROOT, "runs", "overseer_arms", "v2")


def cases(f, nid, H):
    """the same cases as cage_frame.main (crash 35 g x 14 through the first-touching member, flight 4 g)"""
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    guard = np.array(C.HS["guard_nodes"])
    F = C.MASS * C.G * C.CRASH
    out = {}
    for k, d in enumerate(dirs):
        sup_c = C.N @ d; i = int(np.argmax(sup_c)); sup_h = (guard @ d).max()
        out[f"crash {k}"] = {H: -d * F} if sup_h > sup_c[i] + 3.0 else {nid[i]: -d * F}
    m = C.node_mass(f, nid)
    fl = {i: np.array([0, 0, -m[i] * C.G * C.FLIGHT]) for i in range(len(m)) if m[i] > 0}
    fl[H] = fl.get(H, 0) + np.array([0, 0, -C.M_HEAD * C.G * C.FLIGHT]); out["flight 4 g"] = fl
    return out


def main():
    f, nid, H, _ = C.build()
    links = C.RES["links"]
    groups = {"arm 1 tip": [k for k in links if k.startswith("spoke 1.")],
              "arm 2 tip": [k for k in links if k.startswith("spoke 2.")],
              "arm 1 beam (L2 braces)": [k for k in links if k.startswith("L2 brace") and k.endswith("+1")]}
    arm_end = {}
    for g, ks in groups.items():
        for k in ks:
            p, q = map(np.array, links[k])
            a = p if (g.startswith("arm 1 beam") and p[0] < 200) or (not g.startswith("arm 1 beam") and abs(p[2]) < 40 and
                                                                      np.linalg.norm(p[:2] - q[:2]) > 0 and abs(p[1]) == 139.5) else q
            if g.startswith("arm 1 beam"):
                a = p if p[0] < q[0] else q
            arm_end[k] = (a, (q if a is p else p))
    res = {}
    for cn, loads in cases(f, nid, H).items():
        mf_ = {r["name"]: r for r in f.member_forces(f.solve(loads))}
        for g, ks in groups.items():
            F = np.zeros(3)
            for k in ks:
                a, b = arm_end[k]
                u = (b - a) / np.linalg.norm(b - a)
                F += mf_[k]["N"] * u                                     # tension pulls the arm end toward the cage
            res.setdefault(g, {})[cn] = np.round(F, 1).tolist()
    pts = {g: np.round(arm_end[ks[0]][0], 2).tolist() for g, ks in groups.items()}
    worst = {g: max(np.linalg.norm(v) for v in d.values()) for g, d in res.items()}
    os.makedirs(OUT, exist_ok=True)
    json.dump(dict(note="force on the arm at the link end (N, drone frame), per case; from cage_frame.py (F)",
                   points=pts, links={g: ks for g, ks in groups.items()}, forces=res,
                   worst_N={g: round(v, 1) for g, v in worst.items()}),
              open(os.path.join(OUT, "joint_loads.json"), "w"), indent=1)
    for g in res:
        print(g, pts[g], "worst", round(worst[g], 1), "N")


if __name__ == "__main__":
    main()
