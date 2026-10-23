"""Same-grid strength comparison: the old engine's shipped part against a nox/topo part, on the SAME Part grid and load
cases (topo.build.strength on one Layout), so the worst indices compare directly.

    python3 -m topo.compare <part> --new ../runs/<part>/topo-v2/<part>.stl [--out ../runs/<part>/topo-v2/compare.json]

The old part is the print file (print/box_<part>_v8.stl, print pose). It goes back to W with the inverse of the repo's
own print-pose rotation (nox/box/stand/stand_v7_parts.py print_pose; bar_half: stand_v8_bar_export.py, turned 45.12
deg). print_pose ends with a translation to the bed whose value depends on the W mesh, so the translation is found by
overlaying the drawn bodies: a voxel cross-correlation (drawn inside the part, nothing in the cuts or outside the
envelope) for the integer shift, then Nelder-Mead on the exact manifold volumes. The fit is reported (drawn missing,
in cuts, outside the envelope, mm3) and must read about zero.
"""
import os
import sys
import json
import math
import argparse
import numpy as np
from scipy.signal import fftconvolve
from scipy.optimize import minimize

from .part import tm, man, voxelise
from .layout import Layout, Settings
from .build import strength

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(NOX, ".."))


def _stand_modules():
    from .demos.stand import _module
    _module()
    sys.path.insert(0, os.path.join(NOX, "box", "stand"))
    import stand_v6_parts as S
    return S


def print_rotation(name, angle_deg=45.12):
    """the linear part of W -> print pose (4 x 4), from the repo's print-pose code."""
    import trimesh
    if name == "bar_half":
        S = _stand_modules()
        M = np.eye(4)
        M[:3, :] = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, -S.LAP_Y[0]]], float)
        return trimesh.transformations.rotation_matrix(math.radians(angle_deg), [0, 0, 1]) @ M
    if name in ("toe", "kb"):
        return np.eye(4)
    n = [1.0, 0.0, 0.0]                                                   # brackets: flange down (print_pose)
    return trimesh.geometry.align_vectors(np.asarray(n, float), [0, 0, -1.0])


def _fit(part, mesh_w0, log=print, pitch=1.0):
    """translation t so that mesh_w0 + t sits on the setup: drawn bodies inside, nothing in cuts / outside env."""
    drawn = part.drawn_m - part.cuts_m
    allowed = part.env_m - part.cuts_m
    ab = np.array(allowed.bounding_box(), float)
    oD = ab[:3] - 10.0
    nD = np.ceil((ab[3:] + 10.0 - oD) / pitch).astype(int)
    oO = mesh_w0.bounds[0] - pitch
    nO = np.ceil((mesh_w0.bounds[1] + pitch - oO) / pitch).astype(int)
    axD = [oD[k] + (np.arange(nD[k]) + 0.5) * pitch for k in range(3)]
    axO = [oO[k] + (np.arange(nO[k]) + 0.5) * pitch for k in range(3)]
    D = voxelise(tm(drawn), *axD).astype(np.float32)
    A = voxelise(tm(allowed), *axD).astype(np.float32)
    O = voxelise(mesh_w0, *axO).astype(np.float32)
    # score(s) = sum O(x - s) * (D(x) - 2 (1 - A(x))): drawn covered, nothing outside the allowed space. Shifts that put
    # part of O off the setup grid lose nothing there, so the outside of the grid counts as not allowed: pad below.
    Wt = np.pad(D - 2.0 * (1.0 - A), [(int(n), int(n)) for n in nO], constant_values=-2.0)
    oW = oD - nO * pitch
    sc = fftconvolve(Wt, O[::-1, ::-1, ::-1], mode="valid")
    k = np.array(np.unravel_index(int(np.argmax(sc)), sc.shape))
    s0 = oW - oO + k * pitch
    log(f"compare: voxel overlay shift {s0.round(2).tolist()} mm (score {sc.max():.0f} of {D.sum():.0f} drawn voxels, "
        f"pitch {pitch})")
    M0 = man(mesh_w0)

    def miss(t):
        m = M0.translate(list(map(float, t)))
        return (drawn - m).volume() + (m - allowed).volume()
    r = minimize(miss, s0, method="Nelder-Mead", options=dict(xatol=1e-3, fatol=1e-3, initial_simplex=
                 np.vstack([s0, s0 + [pitch, 0, 0], s0 + [0, pitch, 0], s0 + [0, 0, pitch]]), maxiter=300))
    t = r.x
    m = M0.translate(list(map(float, t)))
    fit = dict(shift=np.round(t, 4).tolist(), drawn_missing_mm3=round((drawn - m).volume(), 2),
               in_cuts_mm3=round((m ^ part.cuts_m).volume(), 2), outside_env_mm3=round((m - part.env_m).volume(), 2),
               evals=int(r.nfev))
    log(f"compare: fit {fit}")
    return t, fit


def old_in_w(name, part, path, log=print):
    import trimesh
    P = trimesh.load(path, force="mesh")
    best = None
    angles = (45.12, 45.0) if name == "bar_half" else (None,)
    for ang in angles:
        R = print_rotation(name, ang) if ang is not None else print_rotation(name)
        W0 = P.copy()
        W0.apply_transform(np.linalg.inv(R))
        t, fit = _fit(part, W0, log=log)
        err = fit["drawn_missing_mm3"] + fit["in_cuts_mm3"] + fit["outside_env_mm3"]
        if best is None or err < best[0]:
            W = W0.copy()
            W.apply_translation(t)
            best = (err, W, dict(fit, angle_deg=ang))
    return best[1], best[2]


def compare(name, new_stl, old_stl=None, h=None, out=None, log=print):
    import trimesh
    from .demos import PARTS
    spec = PARTS[name]()
    h = h or spec["h"]
    part = spec["setup"](h)
    old_stl = old_stl or os.path.join(ROOT, "print", f"box_{name}_v8.stl")
    W_old, fit = old_in_w(name, part, old_stl, log=log)
    W_new = trimesh.load(new_stl, force="mesh")
    s = Settings(reach_check="off", auto_sym=False)
    L = Layout(part, volfrac=0.5, settings=s, log=lambda s_: None)
    rows = {}
    for tag, m in (("old", W_old), ("new", W_new)):
        st = strength(L, m, log=lambda s_: log(f"{tag} {s_}"))
        rows[tag] = dict(mass_g=round(m.volume / 1000.0 * L.mat.density, 2), worst=st["worst"],
                         worst_case=st["worst_case"], worst_at=st["worst_at"], rows=st["rows"],
                         stl=os.path.relpath(old_stl if tag == "old" else new_stl, ROOT))
    res = dict(part=name, grid=part.summary(), h=h, fit=fit, old=rows["old"], new=rows["new"])
    log(f"compare {name} (same grid h {h}): old {rows['old']['mass_g']} g worst {rows['old']['worst']}; "
        f"new {rows['new']['mass_g']} g worst {rows['new']['worst']}")
    if out:
        with open(out, "w") as fh:
            json.dump(res, fh, indent=1, default=str)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("part")
    ap.add_argument("--new", required=True)
    ap.add_argument("--old")
    ap.add_argument("--h", type=float)
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.new)), "compare.json")
    r = compare(a.part, os.path.abspath(a.new), a.old, a.h, out)
    print(json.dumps(dict(old=dict(mass_g=r["old"]["mass_g"], worst=r["old"]["worst"]),
                          new=dict(mass_g=r["new"]["mass_g"], worst=r["new"]["worst"]), fit=r["fit"]), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
