"""system_check (cage first: flat walls + edge / post probes) on the tail bay electronics of each option in
runs/scout_electronics/v1/result.json, against the race-car cage in ctx/. Run after electronics_tail.py; prints the result."""
import sys, json, os, numpy as np, trimesh
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from system_check import run
R = json.load(open(os.path.join(ROOT, "runs/scout_electronics/v1/result.json")))
ctx = os.path.join(ROOT, "runs/scout_electronics/v1/ctx")
c = json.load(open(ctx + "/ctx.json"))
parts = []
for n, v in c["parts"].items():
    if v["group"] == "cage":
        parts.append(dict(name=n, mesh=os.path.join(ctx, v["file"]), role="cage", mass_g=v["mass"]))
for opt in R["options"]:
    ps = list(parts)
    for n in ("Radxa Zero 3W", "UBEC", "XR2"):
        (lo, hi) = opt["parts"][n]["bounds"]
        ps.append(dict(name=n, box=[lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]], role="protected", mass_g=1))
    m = dict(name="e", parts=ps, cage_margin_mm=3.0, n_dirs=2000, probes=True)
    r = run(m)
    cf = r.get("cage_first") or r.get("results", {}).get("cage_first")
    print(opt["option"])
    for k in r:
        if k in ("cage_first", "probes"):
            v = r[k]
            print(k, json.dumps({kk: vv for kk, vv in v.items() if kk != "per_part"})[:900])
            if "per_part" in v:
                for pn, pv in v["per_part"].items(): print("  ", pn, json.dumps(pv)[:400])
