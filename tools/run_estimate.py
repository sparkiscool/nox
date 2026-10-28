#!/usr/bin/env python3
"""Measured run estimate (Spark 27 Oct: "why are we always so off on timing ... make that the default for all now"): before a
heavy nox/topo run is quoted or started beside another, smoke it here and quote from the measurement, never from an older part.

python3 tools/run_estimate.py runs/<part>/<version> -- python3 <part script> [args]

Runs the part script with nox/topo's optimise() swapped for a smoke version (no engine edit): the setup as the script builds it,
3 iterations of the coarse stage and 3 of the fine stage (fresh, no checkpoint, no build), then writes <dir>/estimate.json and
prints one line:
  s per iteration (coarse, fine, M), peak memory GB (M, the process's max RSS), cases (load + ties: every case is a solve and a
  full-grid F and U column), active dofs per stage, and the projected wall time as a range (E): coarse 40..70 iterations, fine
  50..120, build 5..15 min (the spread of the nox/topo runs to 27 Oct: hub v6/v7, scan stick, overseer hub deck v1).
Room audit (Spark 27 Oct, hub deck v10: loaded sockets drawn outside the design room; CLAUDE.md "Drawn is attachment only"):
every loaded drawn feature (the solid cells at a load case's nodes, with the drawn cells within 8 mm of them) must face the design
room on >= 35 % of its exposed surface, so the topology can carry its load into the part; a smoke that reads FAIL writes
estimate.json with "room_audit": "FAIL", exits 3, and `tools/review_precheck.py` 14 fails the PR.
The quote to Spark is that range in Pacific time. Two runs share a box only when their measured peaks x 1.15 sum to under 75 % of
its RAM (docs/generative.md Heavy runs). The smoke takes a few minutes; it is part of the run, never skipped.
"""
import json, pathlib, resource, runpy, sys, time

argv = sys.argv[1:]
if "--" not in argv or len(argv) < 3:
    sys.exit(__doc__)
cut = argv.index("--")
OUT = pathlib.Path(argv[0]); cmd = argv[cut + 1:]
if cmd[0].startswith("python"):
    cmd = cmd[1:]
OUT.mkdir(parents=True, exist_ok=True)
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "nox"))
import topo, topo.run as TR
from dataclasses import replace
from topo.layout import Layout, gate_member_mm

SMOKE_IT = 3
AUDIT_NEAR_MM, AUDIT_MIN_SHARE = 8.0, 0.35
C_IT, F_IT, BUILD_S = (40, 70), (50, 120), (300, 900)


def peak_gb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6


def room_audit(p, near_mm=AUDIT_NEAR_MM):
    """share of each loaded drawn feature's exposed faces that touch the design room (rest: void, envelope edge, cuts)"""
    from scipy import ndimage as ndi
    import numpy as np
    sh = p.shape; nx, ny, nz = sh
    solid = p.solid.reshape(sh); design = p.design.reshape(sh)
    loaded = np.zeros((nx + 1, ny + 1, nz + 1), bool)
    for c in p.cases:
        if getattr(c, "nodes", None) is not None and len(c.nodes):
            loaded.ravel()[np.asarray(c.nodes, int)] = True
    lc = np.zeros(sh, bool)
    for di in (0, 1):
        for dj in (0, 1):
            for dk in (0, 1):
                lc |= loaded[di:di + nx, dj:dj + ny, dk:dk + nz]
    lc &= solid
    reg, nreg = ndi.label(lc, np.ones((3, 3, 3)))
    rows = []
    for r in range(1, nreg + 1):
        cells = reg == r
        near = solid & (ndi.distance_transform_edt(~cells) * p.h <= near_mm)
        n_des = n_out = 0
        for ax in range(3):
            for sg in (1, -1):
                pad = [(1, 1)] * 3
                des = np.pad(design, pad)[tuple(slice(1 + sg, 1 + sg + sh[k]) if k == ax else slice(1, 1 + sh[k]) for k in range(3))]
                sol = np.pad(solid, pad)[tuple(slice(1 + sg, 1 + sg + sh[k]) if k == ax else slice(1, 1 + sh[k]) for k in range(3))]
                face = near & ~sol
                n_des += int((face & des).sum()); n_out += int((face & ~des).sum())
        c = (np.argwhere(cells).mean(0) + 0.5) * p.h + np.asarray(p.origin)
        rows.append({"share": round(n_des / max(n_des + n_out, 1), 2), "at": [round(float(v), 1) for v in c], "loaded_cells": int(cells.sum())})
    return sorted(rows, key=lambda r_: r_["share"])


def smoke(setup, h, gen_cm3, out_dir, name, settings=None, coarse=True, git=False, log=None, maxit=400, extra=None):
    s = settings or topo.Settings()
    if s.gate_member:
        s = replace(s, min_member_mm=round(gate_member_mm(setup(h), s.min_member_mm, log=print), 2))
    est = {"name": name, "h": h, "gen_cm3": gen_cm3, "coarse": coarse, "smoke_iterations": SMOKE_IT}
    rows = room_audit(setup(h))
    bad = [r_ for r_ in rows if r_["share"] < AUDIT_MIN_SHARE]
    est.update(room_audit="FAIL" if bad else "PASS", room_audit_rows=rows[:12], room_audit_min_share=AUDIT_MIN_SHARE)
    if bad:
        (OUT / "estimate.json").write_text(json.dumps(est, indent=1))
        print(f"room audit FAIL {name}: loaded drawn features with under {AUDIT_MIN_SHARE:.0%} of their surface on the design room: "
              + "; ".join(f"{r_['share']:.0%} at {r_['at']}" for r_ in bad) + ". Open the design room round them (the topology carries the load), "
              "never draw more solid. No run.")
        sys.exit(3)
    stages = ([("coarse", 2.0 * h, replace(s, reach_check="off", geometric=False, max_stage=25))] if coarse else []) + [("fine", h, s)]
    for tag, hh, ss in stages:
        part = setup(hh)
        L = Layout(part, gen_cm3=gen_cm3, settings=ss, log=lambda *a: None)
        t0 = time.time()
        L.run(maxit=SMOKE_IT, beta_start=0 if tag == "coarse" or not coarse else 2)
        dt = (time.time() - t0) / max(len(L.hist), 1)
        est[tag] = {"s_per_it": round(dt, 1), "cells": int(L.nel), "design_cells": int(L.design.sum()),
                    "cases": len(L.cases), "load_cases": int(getattr(L, "n_real", len(L.cases))),
                    "backend": getattr(getattr(L, "fe_solver", None), "backend", None), "peak_gb": round(peak_gb(), 2)}
        del L
    c = est.get("coarse", {"s_per_it": 0.0}); f = est["fine"]
    lo = c["s_per_it"] * (C_IT[0] if coarse else 0) + f["s_per_it"] * F_IT[0] + BUILD_S[0]
    hi = c["s_per_it"] * (C_IT[1] if coarse else 0) + f["s_per_it"] * F_IT[1] + BUILD_S[1]
    est.update(peak_gb=round(peak_gb(), 2), wall_h=[round(lo / 3600, 2), round(hi / 3600, 2)],
               note="s per iteration and peak M (this smoke); the wall range E (iteration counts of past nox/topo runs)")
    (OUT / "estimate.json").write_text(json.dumps(est, indent=1))
    print(f"estimate {name}: coarse {c['s_per_it']} s/it, fine {f['s_per_it']} s/it ({f['cases']} cases, {f['design_cells']:,} design cells), "
          f"peak {est['peak_gb']} GB (M); wall {est['wall_h'][0]}..{est['wall_h'][1]} h (E); written {OUT / 'estimate.json'}")
    sys.exit(0)


topo.optimise = TR.optimise = smoke
sys.argv = cmd
sys.path.insert(0, str(pathlib.Path(cmd[0]).resolve().parent))
runpy.run_path(cmd[0], run_name="__main__")
