"""Self-test of nox/topo. Run from nox/:  python3 -m topo.test_topo [--quick]
Each test prints PASS / FAIL with its numbers; exit code 1 if any fails. ~2-4 min on a 4 vCPU box (--quick skips the
end-to-end run)."""
import sys
import time
import traceback
import numpy as np
import scipy.sparse as sp

from . import fem
from .material import MATERIALS, stiffness
from .mma import MMA
from .layout import calibration, Layout, Settings

RESULTS = []


def check(name, ok, info=""):
    RESULTS.append((name, bool(ok)))
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}", flush=True)


def _C():
    return stiffness(MATERIALS["PETG"], 2)


def _assembled(shape, h, E, C):
    g = fem.Grid(shape)
    KE, _ = fem.hex_ke(h, C)
    edof = g.edof()
    iK = np.repeat(edof, 24, axis=1).ravel()
    jK = np.tile(edof, (1, 24)).ravel()
    sK = (E[:, None] * KE.ravel()[None, :]).ravel()
    return sp.coo_matrix((sK, (iK, jK)), shape=(g.ndof, g.ndof)).tocsr(), g, KE


def t_ke():
    KE, B0 = fem.hex_ke(1.0, _C())
    w = np.linalg.eigvalsh(KE)
    nz = int((np.abs(w) < 1e-8 * w.max()).sum())
    check("hex KE symmetric, PSD, 6 rigid modes", np.allclose(KE, KE.T) and w.min() > -1e-8 * w.max() and nz == 6,
          f"zero modes {nz}, min eig {w.min():.2e}")


def t_matvec():
    rng = np.random.default_rng(0)
    shape = (6, 4, 5)
    E = rng.uniform(0.1, 1.0, np.prod(shape))
    K, g, KE = _assembled(shape, 1.3, E, _C())
    u = rng.normal(size=(g.ndof, 2))
    U = g.to_field(u)
    r = (g.gather(U).reshape(-1, 24) @ KE).reshape(2, g.nel, 24) * E[None, :, None]
    Y = g.to_flat(g.scatter(r))
    err = np.abs(Y - K @ u).max() / np.abs(K @ u).max()
    check("structured gather / scatter == assembled K", err < 1e-12, f"rel err {err:.1e}")


def t_transfer():
    rng = np.random.default_rng(1)
    xc = rng.normal(size=(1, 4, 3, 5, 3))
    yf = rng.normal(size=(1, 7, 5, 9, 3))
    a = (fem.prolong(xc) * yf).sum()
    b = (xc * fem.restrict(yf)).sum()
    check("restrict = prolong^T", abs(a - b) < 1e-10 * abs(a), f"{a:.6f} vs {b:.6f}")


def t_galerkin():
    rng = np.random.default_rng(2)
    shape = (4, 4, 2)
    E = rng.uniform(0.01, 1.0, np.prod(shape))
    K, g, KE = _assembled(shape, 1.0, E, _C())
    gc = fem.Grid((2, 2, 1))
    P = np.zeros((g.ndof, gc.ndof))
    for j in range(gc.ndof):
        e = np.zeros(gc.ndof)
        e[j] = 1.0
        P[:, j] = fem.prolong(gc.to_field(e)).reshape(-1)
    Kc_true = P.T @ K.toarray() @ P
    P8 = fem.child_prolongation()
    G8 = np.einsum("cai,ab,cbj->cij", P8, KE, P8).reshape(8, 576)
    Ke1 = (fem._octants(E, shape) @ G8).reshape(-1, 24, 24)
    ed = gc.edof()
    Kc = np.zeros((gc.ndof, gc.ndof))
    for e in range(gc.nel):
        Kc[np.ix_(ed[e], ed[e])] += Ke1[e]
    err = np.abs(Kc - Kc_true).max() / np.abs(Kc_true).max()
    check("element-by-element Galerkin == P^T K P", err < 1e-10, f"rel err {err:.1e}")


def _cantilever(shape=(16, 8, 8), void_frac=0.3, seed=3):
    rng = np.random.default_rng(seed)
    nel = int(np.prod(shape))
    act = rng.uniform(size=nel) > void_frac
    E = np.where(act, rng.uniform(0.05, 1.0, nel), 0.0)
    g = fem.Grid(shape)
    fixed = np.zeros((g.nn, 3), bool)
    nodes = np.arange(g.nn).reshape(g.nshape)
    fixed[nodes[0].ravel()] = True
    F = np.zeros((g.ndof, 2))
    tip = nodes[-1].ravel()
    F[3 * tip + 2, 0] = -1.0 / len(tip)
    F[3 * tip + 1, 1] = 1.0 / len(tip)
    return g, E, act, {"default": fixed}, F


def t_mg_vs_direct():
    g, E, act, fixed, F = _cantilever()
    try:
        d = fem.Solver(g.shape, 1.0, _C(), fixed, active=act, backend="direct", log=lambda s: None)
        Ud = d.solve(E, F, ["default", "default"])
        m = fem.Solver(g.shape, 1.0, _C(), fixed, active=act, backend="mg", log=lambda s: None, mg_tol=1e-9,
                       coarse_dofs=500)
        Um = m.solve(np.where(act, E, 0.0), F, ["default", "default"], keys=["a", "b"])
        cd, cm = (F * Ud).sum(0), (F * Um).sum(0)
        err = np.abs(cm - cd).max() / np.abs(cd).max()
        its = m.stats[-1][1]
        check("multigrid CG == Pardiso (compliance, 2 rhs)", err < 1e-4, f"rel err {err:.1e}, {its} CG its, "
              f"{m.levels} levels")
        t0 = time.time()
        Um2 = m.solve(np.where(act, E * 1.01, 0.0), F, ["default", "default"], keys=["a", "b"])
        check("warm start cuts CG iterations", m.stats[-1][1] < its, f"{m.stats[-1][1]} vs {its} its")
    except Exception as e:
        traceback.print_exc()
        check("multigrid CG == Pardiso", False, str(e))


def t_mma():
    n = 50
    rng = np.random.default_rng(4)
    a = rng.uniform(0.2, 0.9, n)
    opt = MMA(n, 1, move=0.2)
    x = np.full(n, 0.5)
    for _ in range(60):
        f0 = ((x - a) ** 2).sum()
        df0 = 2 * (x - a)
        g = x.mean() / 0.4 - 1.0
        dg = np.full(n, 1.0 / (n * 0.4))
        x = opt.step(x, f0, df0, np.array([g]), dg[None])
    # KKT solution: x = clip(a - lam, 0, 1) with mean 0.4
    lo, hi = -1.0, 1.0
    for _ in range(60):
        lam = 0.5 * (lo + hi)
        if np.clip(a - lam, 0, 1).mean() > 0.4:
            lo = lam
        else:
            hi = lam
    xs = np.clip(a - lam, 0, 1)
    err = np.abs(x - xs).max()
    check("MMA solves a constrained quadratic", err < 2e-3, f"max err {err:.1e}, mean x {x.mean():.4f}")


def t_calibration():
    """independent check of layout.calibration: the exact 3D cone kernel voxelised at R = 40 cells, a plate and a rod
    of the calibrated input sizes filtered by brute force, blueprint widths read off at eta_n. (The 1D hat filter gives
    plate = 1.0 R; the 3D cone integrated over a plane has a cubic profile and gives 0.88 R, so 1.0 is not the target.)"""
    from scipy.signal import fftconvolve
    c = calibration()
    R = 40
    q = np.arange(-3 * R, 3 * R + 1)
    # plate: 1D profile of the cone = its integral over the plane, done numerically on a 3D voxel grid
    X3, Y3, Z3 = np.meshgrid(np.arange(-R, R + 1), np.arange(-R, R + 1), np.arange(-R, R + 1),
                             indexing="ij")
    w3 = np.maximum(1.0 - np.sqrt(X3 ** 2 + Y3 ** 2 + Z3 ** 2) / R, 0.0)
    prof1 = w3.sum((1, 2))
    prof1 /= prof1.sum()
    prof2 = w3.sum(2)
    prof2 /= prof2.sum()
    edge = lambda f: (np.argmax(f < 0.5) - 1 + (f[np.argmax(f < 0.5) - 1] - 0.5) /
                      (f[np.argmax(f < 0.5) - 1] - f[np.argmax(f < 0.5)]))      # 0.5 crossing, linear interpolation
    slab = np.clip(c["plate_input"] * R / 2 + 0.5 - np.abs(q), 0.0, 1.0)     # area-weighted edge cells
    f1 = np.convolve(slab, prof1, mode="same")
    plate_bp = 2 * edge(f1[3 * R:]) / R
    X, Y = np.meshgrid(q, q, indexing="ij")
    disc = np.clip(c["rod_input"] * R / 2 + 0.5 - np.sqrt(X ** 2 + Y ** 2), 0.0, 1.0)
    f2 = fftconvolve(disc, prof2, mode="same")
    rod_bp = 2 * edge(f2[3 * R:, 3 * R]) / R
    ok = (abs(f1[3 * R] - 0.75) < 0.02 and abs(f2[3 * R, 3 * R] - 0.75) < 0.02 and abs(plate_bp - c["plate"]) < 0.03
          and abs(rod_bp - c["rod"]) < 0.03 and 0 < c["erode"] < 0.5)
    check("length-scale calibration vs brute-force 3D cone filter", ok,
          f"rod {c['rod']:.3f} R (brute {rod_bp:.3f}), plate {c['plate']:.3f} R (brute {plate_bp:.3f}), centre "
          f"{f1[3 * R]:.3f} / {f2[3 * R, 3 * R]:.3f} (eta_e 0.75), erosion {c['erode']:.3f} R")


def _mini_part(neck=False, h=1.0):
    """bar 24 x 10 x 10 mm: drawn plate at x 0..3 (support), drawn pad at x 20..24 loaded down; design between."""
    import manifold3d as mf
    from .part import Part
    box = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
    base = box(0, 3, 0, 10, 0, 10)
    pad = box(20, 24, 2, 8, 2, 8)
    p = Part("mini", h, base + pad, box(100, 101, 100, 101, 100, 101), box(0, 24, 0, 10, 0, 10), log=lambda s: None)
    p.support("default", box(-1, 1.5 * h, -1, 11, -1, 11))
    p.load("down", pad, (0, 0, -10.0))
    p.load("side", pad, (0, 5.0, 0))
    return p


def t_gradients():
    try:
        p = _mini_part()
        s = Settings(min_member_mm=3.2, backend="direct", reach_check="off", auto_sym=False)
        L = Layout(p, volfrac=0.3, settings=s, log=lambda s_: None)
        rng = np.random.default_rng(5)
        x = rng.uniform(0.2, 0.8, L.nd)
        beta = 4.0

        def J(xx):
            f = L.fields(xx, beta)
            comp, _ = L._compliance(f["rho_e"], need_grad=False)
            return L._objective(comp)[0]
        f = L.fields(x, beta)
        comp, (U, E) = L._compliance(f["rho_e"])
        Jv, dJdc, r = L._objective(comp)
        g = L._grad_rho_e(f["rho_e"], U, dJdc)
        m_e = L.design | L.skin
        dx = L._back(np.where(m_e, g * f["dHe"], 0.0))
        idx = rng.choice(L.nd, 6, replace=False)
        errs = []
        for i in idx:
            e = 1e-5
            xp, xm = x.copy(), x.copy()
            xp[i] += e
            xm[i] -= e
            fd = (J(xp) - J(xm)) / (2 * e)
            errs.append(abs(fd - dx[i]) / max(abs(fd), 1e-3 * np.abs(dx).max()))
        check("objective gradient vs finite differences", max(errs) < 1e-3, f"max rel err {max(errs):.1e}")
        geo = L._geometric(f, 32.0)
        errs = []
        for q in range(2):
            val, dg = geo[q]
            gx = L._back(dg)
            for i in idx[:4]:
                e = 1e-6
                xp, xm = x.copy(), x.copy()
                xp[i] += e
                xm[i] -= e
                vp = L._geometric(L.fields(xp, beta), 32.0)[q][0]
                vm = L._geometric(L.fields(xm, beta), 32.0)[q][0]
                fd = (vp - vm) / (2 * e)
                errs.append(abs(fd - gx[i]) / max(abs(fd), 1e-3 * np.abs(gx).max(), 1e-30))
        check("geometric-constraint gradients vs finite differences", max(errs) < 1e-3, f"max rel err {max(errs):.1e}")
    except Exception as e:
        traceback.print_exc()
        check("gradients", False, str(e))


def t_neck():
    """the reason for anchored erosion: the same pad joined by a 2 mm neck vs a full-width root. In the eroded design
    the neck must open a gap (compliance ratio >> wide root); with frozen pads (old behaviour) it must not."""
    try:
        res = {}
        for anchored in (True, False):
            p = _mini_part()
            s = Settings(min_member_mm=3.2, backend="direct", reach_check="off", auto_sym=False,
                         anchored_erosion=anchored)
            L = Layout(p, volfrac=0.3, settings=s, log=lambda s_: None)
            cen = p.cell_centres()[L.design]
            # member from the base to x = 18, then a neck (2 x 2) or a wide root (6 x 6) to the pad face at x 20
            core = (cen[:, 0] < 18) & (np.abs(cen[:, 1] - 5) < 3) & (np.abs(cen[:, 2] - 5) < 3)
            for kind, half in (("neck", 1.0), ("wide", 3.0)):
                lnk = (cen[:, 0] >= 18) & (np.abs(cen[:, 1] - 5) < half) & (np.abs(cen[:, 2] - 5) < half)
                x = np.where(core | lnk, 1.0, 0.0)
                f = L.fields(x, 32.0)
                comp, _ = L._compliance(f["rho_e"], need_grad=False)
                res[(anchored, kind)] = float(comp[0] / L.c_ref[0])
        ratio_new = res[(True, "neck")] / res[(True, "wide")]
        ratio_old = res[(False, "neck")] / res[(False, "wide")]
        check("anchored erosion punishes a 2 mm neck at a pad", ratio_new > 50 and ratio_new > 10 * ratio_old,
              f"neck / wide compliance: anchored {ratio_new:.3g}, frozen pads {ratio_old:.3g}")
    except Exception as e:
        traceback.print_exc()
        check("anchored erosion neck test", False, str(e))


def t_end_to_end():
    import os
    import tempfile
    from .demos.holder import setup
    from .run import optimise
    try:
        out = tempfile.mkdtemp(prefix="topo_e2e_")
        s = Settings(min_member_mm=4.0, max_stage=12, min_stage=4, backend="auto", time_limit_s=900)
        res = optimise(setup, 2.0, 6.0, out, "holder_smoke", settings=s, coarse=False, maxit=60,
                       log=lambda s_: print(s_[:300], flush=True))
        check("end to end: holder at h 2.0 builds one watertight body", res["bodies"]["bodies"] == 1 and
              res["watertight"], f"{res['mass_g']} g, solid check {'PASS' if res['solid_ok'] else 'FAIL'}, "
                                 f"worst {res['strength']['worst']}, {res['total_s']} s, files in {out}")
    except Exception as e:
        traceback.print_exc()
        check("end to end", False, str(e))


def main():
    quick = "--quick" in sys.argv
    t0 = time.time()
    for t in (t_ke, t_matvec, t_transfer, t_galerkin, t_mg_vs_direct, t_mma, t_calibration, t_gradients, t_neck):
        t()
    if not quick:
        t_end_to_end()
    bad = [n for n, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(bad)} / {len(RESULTS)} passed in {time.time() - t0:.0f} s" +
          (f"; FAILED: {bad}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
