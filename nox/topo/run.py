"""Run a part end to end: coarse warm start -> fine layout -> build -> checks -> files, with checkpoints.

    python3 -m topo.run <part> [--h 1.0] [--gen 12] [--min-member 3.2] [--out runs/<part>/topo-v1] [--no-coarse]
                               [--git] [--backend auto|direct|mg] [--time-limit 19800] [--set KEY=VALUE ...]
(run from nox/, i.e. `cd nox && python3 -m topo.run holder`). <part> is a name from topo.demos.PARTS.

Speed plan for a 4 vCPU / 16 GB cloud box (measured numbers go in the run log):
  * stage 1 on a 2h grid (8x fewer cells) for beta 1..4, where most iterations are spent;
  * stage 2 on the h grid from beta 4, warm-started from stage 1;
  * Pardiso for small active systems, multigrid CG beyond (topo.fem.Solver picks by RAM);
  * time-aware continuation: past 75 % of the time limit the beta steps are taken as soon as allowed.
"""
import os
import sys
import json
import time
import argparse
import subprocess
from dataclasses import replace

import numpy as np

from .layout import Layout, Settings, gate_member_mm
from .build import build_and_check


class Tee:
    def __init__(self, path):
        self.f = open(path, "a", buffering=1)

    def __call__(self, s):
        s = str(s)
        print(s[:2000], flush=True)
        self.f.write(s + "\n")


def git_checkpoint(out_dir, log):
    def hook(it):
        try:
            root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                                  cwd=out_dir).stdout.strip()
            if not root:
                return
            subprocess.run(["git", "add", os.path.relpath(out_dir, root)], cwd=root, capture_output=True)
            subprocess.run(["git", "commit", "-q", "-m", f"ckpt {it}"], cwd=root, capture_output=True)
            r = subprocess.run(["git", "push", "-q"], cwd=root, capture_output=True, text=True)
            if r.returncode:
                log(f"git push failed: {r.stderr.strip()[:200]}")
        except Exception as e:                                   # a checkpoint push must never stop the run
            log(f"git checkpoint failed: {e}")
    return hook


def peak_gb():
    try:
        import resource
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
    except Exception:
        return None


DRAWN_MAX = 1.0                    # the drawn part passes at a worst as-built index <= this (a finished part's own bar)


def drawn_check(setup, h, gen_cm3=None, settings=None, stl=None):
    """Drawn first (Spark 27 Oct: "why were they doing [topology] after checks passed ... fix it for the future"): the drawn
    bodies minus the cuts, alone, through the solid gate (nox/solid_check.py) and the as-built strength check on every real
    load case of the part. If they pass, topology can only add mass: the drawn part is the part and no run starts
    (tools/run_estimate.py stops there). Writes the drawn mesh to `stl` when given. Returns a dict (name, mass_g, solid_ok,
    worst, worst_case, pass)."""
    import trimesh
    from .layout import Layout
    from .build import strength
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import solid_check as SC
    s = settings or Settings()
    part = setup(h)
    g = (part.drawn_m - part.cuts_m).to_mesh()
    mesh = trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts))
    if stl: mesh.export(stl)
    ok_solid, info = SC.solid_check(trimesh.load(stl, force="mesh") if stl else mesh)
    L = Layout(part, gen_cm3=gen_cm3, settings=s, log=lambda *a: None)
    try:
        st = strength(L, mesh, log=lambda *a: None); worst, case = st["worst"], st["worst_case"]
    except Exception as e:                                             # a drawn set that is not one body cannot carry the loads
        worst, case = None, f"no solve ({type(e).__name__})"
    ok = bool(ok_solid and worst is not None and worst <= DRAWN_MAX)
    return {"name": part.name, "mass_g": round(float(mesh.volume) * L.mat.density * 1e-3, 2), "solid_ok": bool(ok_solid),
            "bodies_plain": info.get("bodies_plain"), "worst": None if worst is None else round(float(worst), 3), "worst_case": case,
            "max": DRAWN_MAX, "h": h, "pass": ok}


def optimise(setup, h, gen_cm3, out_dir, name, settings=None, coarse=True, git=False, log=None, maxit=400,
             extra=None):
    os.makedirs(out_dir, exist_ok=True)
    log = log or Tee(os.path.join(out_dir, "log.txt"))
    s = settings or Settings()
    t0 = time.time()
    hook = git_checkpoint(out_dir, log) if git else None
    ck = os.path.join(out_dir, "ck.npz")
    log(f"=== {name}: h {h}, generated {gen_cm3} cm3, min member {s.min_member_mm} mm, cores {os.cpu_count()}")
    if s.gate_member:
        s = replace(s, min_member_mm=round(gate_member_mm(setup(h), s.min_member_mm, log=log), 2))
    xh_c = part_c = None
    if coarse and not os.path.exists(ck):
        part_c = setup(2.0 * h)
        sc = replace(s, reach_check="off", geometric=False, max_stage=25)
        Lc = Layout(part_c, gen_cm3=gen_cm3, settings=sc, log=log)
        ckc = os.path.join(out_dir, "coarse_ck.npz")
        Lc.run(maxit=maxit, checkpoint=ckc, resume=ckc, beta_stop=2, on_checkpoint=hook)
        xh_c = Lc.x_hat()
        log(f"coarse stage done: {len(Lc.hist)} iterations, {time.time() - t0:.0f} s, peak {peak_gb()} GB")
        del Lc
    part = setup(h)
    L = Layout(part, gen_cm3=gen_cm3, settings=s, log=log)
    if xh_c is not None:
        L.init_from(xh_c, part_c)
    L.run(maxit=maxit, checkpoint=ck, resume=ck, beta_start=2 if coarse else 0, on_checkpoint=hook)
    L.save(os.path.join(out_dir, "density.npz"))
    t_layout = time.time() - t0
    log(f"layout done: {t_layout:.0f} s, peak {peak_gb()} GB")
    ex = dict(layout_s=round(t_layout), peak_gb=peak_gb(), gen_cm3=gen_cm3, h=h)
    if extra:
        ex.update(extra)
    res, P = build_and_check(L, out_dir, name, log=log, extra=ex)
    res["total_s"] = round(time.time() - t0)
    with open(os.path.join(out_dir, "result.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=str)
    if hook:
        hook("final")
    return res


def rebuild(setup, h, gen_cm3, src_dir, out_dir, name, settings=None, log=None, extra=None):
    """build and check again from a finished layout (src_dir/density.npz), e.g. after a build-stage change. The
    layout is not rerun; result.json names the source run."""
    import shutil
    import numpy as np
    os.makedirs(out_dir, exist_ok=True)
    log = log or Tee(os.path.join(out_dir, "log.txt"))
    s = settings or Settings()
    t0 = time.time()
    log(f"=== {name}: rebuild from {src_dir}/density.npz (layout not rerun)")
    if s.gate_member:
        s = replace(s, min_member_mm=round(gate_member_mm(setup(h), s.min_member_mm, log=log), 2))
    part = setup(h)
    L = Layout(part, gen_cm3=gen_cm3, settings=s, log=log)
    d = np.load(os.path.join(src_dir, "density.npz"))
    if tuple(d["shape"]) != tuple(part.shape) or not np.array_equal(d["design"], L.design):
        raise ValueError("rebuild: the source layout's grid or design mask differs from this setup")
    L.x = d["x_full"][L.design]
    L.hist = [tuple(r) for r in d["hist"]]
    L.result = dict(x=L.x, xt=d["xt"], rho_n=d["rho_n"], rho_e=d["rho_e"], beta=float(d["beta"]))
    shutil.copy(os.path.join(src_dir, "density.npz"), os.path.join(out_dir, "density.npz"))
    src = {}
    try:
        with open(os.path.join(src_dir, "result.json")) as fh:
            r0 = json.load(fh)
        src = dict(layout_s=r0.get("layout_s"), peak_gb=r0.get("peak_gb"))
    except OSError:
        pass
    ex = dict(gen_cm3=gen_cm3, h=h, rebuilt_from=os.path.basename(os.path.normpath(src_dir)), **src)
    if extra:
        ex.update(extra)
    res, P = build_and_check(L, out_dir, name, log=log, extra=ex)
    res["total_s"] = round(time.time() - t0)
    with open(os.path.join(out_dir, "result.json"), "w") as fh:
        json.dump(res, fh, indent=1, default=str)
    return res


def parse_sets(items):
    """['pmean=8', 'eta=0.7,0.5,0.3', 'geometric=true'] -> typed Settings overrides (types from the defaults)."""
    import dataclasses
    names = {f.name for f in dataclasses.fields(Settings)}
    out = {}
    for kv in items:
        if "=" not in kv:
            raise SystemExit(f"--set needs KEY=VALUE, got {kv!r}")
        k, v = kv.split("=", 1)
        k = k.strip().replace("-", "_")
        if k not in names:
            raise SystemExit(f"unknown setting {k!r}; settings: {', '.join(sorted(names))}")
        d = getattr(Settings, k)
        if isinstance(d, bool):
            out[k] = v.strip().lower() in ("1", "true", "yes", "on")
        elif isinstance(d, tuple):
            out[k] = tuple(type(d[0])(float(t)) if d else float(t) for t in v.split(","))
        elif isinstance(d, int):
            out[k] = int(float(v))
        elif isinstance(d, float):
            out[k] = float(v)
        else:
            out[k] = v.strip()
    return out


def main(argv=None):
    from .demos import PARTS
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=sorted(PARTS))
    ap.add_argument("--h", type=float)
    ap.add_argument("--gen", type=float)
    ap.add_argument("--min-member", type=float, default=Settings.min_member_mm)
    ap.add_argument("--out")
    ap.add_argument("--no-coarse", action="store_true")
    ap.add_argument("--git", action="store_true")
    ap.add_argument("--backend", default="auto")
    ap.add_argument("--time-limit", type=float, default=5.5 * 3600)
    ap.add_argument("--maxit", type=int, default=400)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override any Settings field, e.g. --set pmean=8 --set eta=0.7,0.5,0.3 (repeatable)")
    ap.add_argument("--rebuild-from", metavar="RUN_DIR", help="build and check again from RUN_DIR/density.npz")
    a = ap.parse_args(argv)
    spec = PARTS[a.part]()
    h = a.h or spec["h"]
    gen = a.gen or spec["gen_cm3"]
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "runs", a.part, "topo-v1")
    s = Settings(min_member_mm=a.min_member, backend=a.backend, time_limit_s=a.time_limit, **parse_sets(a.set))
    if a.rebuild_from:
        res = rebuild(spec["setup"], h, gen, os.path.abspath(a.rebuild_from), os.path.abspath(out), a.part,
                      settings=s, extra=dict(baseline=spec.get("baseline")))
    else:
        res = optimise(spec["setup"], h, gen, os.path.abspath(out), a.part, settings=s, coarse=not a.no_coarse,
                       git=a.git, maxit=a.maxit, extra=dict(baseline=spec.get("baseline")))
    print(json.dumps(dict(name=res["name"], solid_ok=res["solid_ok"], mass_g=res["mass_g"],
                          worst=res["strength"]["worst"], total_s=res["total_s"]), indent=1))
    return 0 if res["solid_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
