"""Carry a finished nox/topo layout checkpoint onto a setup whose drawn bodies or cuts changed slightly (a bore 3.2 -> 3.6),
so the part is rebuilt from the same layout instead of a new run. The layout's design mask (part.design plus the drawn volume
the organic finish opens) is rebuilt for the old and the new setup; densities move cell by cell, a cell that leaves the design
drops, a cell that joins it starts at 0. Only for small changes: it refuses when more than 1 % of the design cells differ.
    python3 tools/topo_remap.py <module.py> <setup_fn> <piece> <h> <old_ck.npz> <new_ck.npz> NAME=OLD_VALUE [NAME=OLD ...]
NAME=OLD_VALUE sets module globals for the old setup (a Python literal; ROD_R=1.6, or V.ROD_R=1.6 for a module the target imports as V);
the new setup uses the module as it is now."""
import sys, os, ast, importlib.util
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from topo import Settings


def load(path):
    sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    argv = sys.argv; sys.argv = [path]
    spec = importlib.util.spec_from_file_location("remap_target", path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    sys.argv = argv
    return m


def layout_design(part, s=Settings()):
    design, solid = part.design.copy(), part.solid.copy()
    if s.finish == "organic":
        skin = part.functional_skin(s.skin_mm)
        design = design | (solid & ~skin)
    return design


def main():
    mod, fn, piece, h, old_ck, new_ck = sys.argv[1:7]
    sets = dict(a.split("=", 1) for a in sys.argv[7:])
    m = load(os.path.join(ROOT, mod) if not os.path.isabs(mod) else mod)
    setup = getattr(m, fn)
    new = setup(piece, float(h)); dn = layout_design(new)
    def owner(k):                                       # "V.ROD_R": a global of a module the target imports as V
        o = m
        for a in k.split(".")[:-1]: o = getattr(o, a)
        return o, k.split(".")[-1]
    cur = {k: getattr(*owner(k)) for k in sets}
    for k, v in sets.items():
        setattr(*owner(k), ast.literal_eval(v))
    old = setup(piece, float(h)); do = layout_design(old)
    for k, v in cur.items():
        setattr(*owner(k), v)
    assert new.shape == old.shape, "grid changed: run the layout again"
    changed = int((do ^ dn).sum())
    print(f"remap: old design {int(do.sum()):,}, new {int(dn.sum()):,}, cells changed {changed}")
    if changed > 0.01 * do.sum():
        sys.exit("remap: more than 1 % of the design changed: run the layout again")
    d = dict(np.load(old_ck, allow_pickle=True))
    assert d["x"].shape[0] == int(do.sum()), "checkpoint does not match the old setup"
    for k in ("x", "xold1", "xold2", "low", "upp"):
        full = np.zeros(do.shape[0]); full[do] = d[k]
        d[k] = full[dn]
    os.makedirs(os.path.dirname(os.path.abspath(new_ck)), exist_ok=True)
    np.savez(new_ck, **d)
    print("remap: wrote", new_ck)


if __name__ == "__main__":
    main()
