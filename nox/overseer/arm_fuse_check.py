"""Overseer cage frame with the arm spoke nodes as fuses (overseer arm session, 27 Oct, DEFAULT pending Spark: each arm's spoke node is
a printed part on 2 M3 that lets go at ~600 N resultant). For every crash case of cage_frame.py: solve; where an arm's spoke group
carries more than the cap, that node fuses: (cap) the group is replaced by the cap force along its load direction, or (detach) the
group is removed (the node broke). The cage and the other links then carry the rest: worst buckling per member over all cases.

python3 nox/overseer/arm_fuse_check.py [cap_N]  -> runs/overseer_frame/v1/arm_fuse.md
"""
import os, sys, math, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
CAP = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
sys.argv = sys.argv[:1]
import cage_frame as C


def groups(f):
    out = {}
    for a in (1, 2, 3, 4):
        idx = [j for j, m in enumerate(f.M) if m["name"].startswith(f"spoke {a}.")]
        out[a] = idx
    return out


def cases(f, nid, H):
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    guard = np.array(C.HS["guard_nodes"]); F = C.MASS * C.G * C.CRASH; out = {}
    for k, d in enumerate(dirs):
        sup = C.N @ d; i = int(np.argmax(sup))
        out[f"crash {k} {np.round(d, 2).tolist()}"] = {H: -d * F} if (guard @ d).max() > sup[i] + 3.0 else {nid[i]: -d * F}
    return out


def group_force(f, mf_, idx):
    """the group's summed force on its arm node (vector), and the arm node index"""
    Fv = np.zeros(3); node = None
    for j in idx:
        m = f.M[j]; r = mf_[j]; a, b = f.X[m["a"]], f.X[m["b"]]
        arm_end, far = (m["b"], a) if m["b"] in f.fixed else (m["a"], b)
        node = arm_end; e = f.X[arm_end]; Fv += r["N"] * (far - e) / np.linalg.norm(far - e)
    return Fv, node


def main():
    rows = {}; fused = {}
    for mode in ("as built", "cap", "detach"):
        f, nid, H, _ = C.build(); G = groups(f); worst = {}
        for cn, loads in cases(f, nid, H).items():
            mf_ = f.member_forces(f.solve(loads))
            over = {a: group_force(f, mf_, idx) for a, idx in G.items()}
            over = {a: v for a, v in over.items() if np.linalg.norm(v[0]) > CAP}
            if mode != "as built" and over:
                fused.setdefault(cn, sorted(over))
                keep = list(f.M); saved = {}
                for a in over:
                    for j in G[a]: saved[j] = f.M[j]["sec"]; f.M[j]["sec"] = dict(f.M[j]["sec"], A=f.M[j]["sec"]["A"] * 1e-6, I=f.M[j]["sec"]["I"] * 1e-6, J=f.M[j]["sec"]["J"] * 1e-6)
                L2 = {k: np.array(v, float) for k, v in loads.items()}
                if mode == "cap":                                       # the fused node still pulls / pushes the cage with the cap force
                    for a, (Fv, node) in over.items():
                        idx = G[a]; m = f.M[idx[0]]; cage_end = m["a"] if m["b"] == node else m["b"]
                        share = -Fv / np.linalg.norm(Fv) * CAP / len(idx)
                        for j in idx:
                            mm = f.M[j]; ce = mm["a"] if mm["b"] == node else mm["b"]; L2[ce] = L2.get(ce, 0) + share
                mf_ = f.member_forces(f.solve(L2))
                for j, sec in saved.items(): f.M[j]["sec"] = sec
            for r in mf_:
                if r["name"].startswith("head-") or (mode != "as built" and r["name"].startswith("spoke") and any(r["name"].startswith(f"spoke {a}.") for a in over)): continue
                if r["buckle"] > worst.get(r["name"], (0, ""))[0]: worst[r["name"]] = (r["buckle"], cn)
        rows[mode] = sorted(((round(v[0], 2), k, v[1]) for k, v in worst.items() if v[0] >= 0.8), reverse=True)
    L = [f"# Arm spoke nodes as {CAP:.0f} N fuses (F on E; `nox/overseer/arm_fuse_check.py`)\n",
         f"Crash cases that fuse an arm node: " + "; ".join(f"{c} -> arms {a}" for c, a in fused.items()) + "\n"]
    for mode, rs in rows.items():
        L.append(f"## {mode}: members at or over 0.8 x buckling (k as cage_frame)")
        L += [f"- {k}: {b} ({c})" for b, k, c in rs[:14]] or ["- none"]
        L.append("")
    out = os.path.join(ROOT, "runs", "overseer_frame", "v1", "arm_fuse.md"); open(out, "w").write("\n".join(L)); print("\n".join(L))


if __name__ == "__main__":
    main()
