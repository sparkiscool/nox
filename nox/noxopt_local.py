"""noxopt_local: large-grid backend for noxopt (9 Oct 2026). For long runs on Sparks Book.

noxopt.py and printopt.py are NOT changed: this module imports them and swaps the FE solve. Same formulation (robust
eroded / nominal / dilated stiffness layout, tie cases, p-mean, MMA, beta continuation, grey stop), so a part spec
that builds a printopt.Problem runs unchanged: replace NO.Layout(pb, vf) with NL.Layout(pb, vf).

What is different (all inside the compliance evaluation):
  1. Solver: CG preconditioned by a geometric multigrid V-cycle (Amir, Aage, Lazarov 2014; Aage et al. 2015).
     Coarse levels are the exact Galerkin operators P^T K P, built element by element from the 8 children (no global
     triple product), block-Jacobi smoothing, direct (pardiso) solve on the coarsest level. Memory grows ~linearly with
     the cell count, where the direct factorisation grows ~n^(4/3).
     backend="pardiso" keeps a direct solve (exact, fastest on small grids); "auto" picks pardiso below AUTO_DOF.
  2. Load superposition: per support group, every case is a combination of a few basis loads: the independent static
     load vectors plus 3 body-force fields (unit acceleration in x, y, z on the current density). 14 misuse directions
     + flight + landing cost 3 solves, not 16. Exact to the solver tolerance.
  3. Warm start: each iteration starts CG from the previous iteration's displacements.
  4. Lean memory: no global COO index arrays (printopt keeps 2 x 576 int64 per element, ~9 GB at 1M cells); the
     stiffness is stored as 3x3-block BSR with an int32 position map (64 per element). Sensitivities in element chunks.
  5. MKL threaded sparse products when sparse_dot_mkl is installed (pip install sparse_dot_mkl), scipy otherwise.
Checked against the direct solve in test_noxopt_local.py (finite differences, same compliance and sensitivities,
same layout on the self-test problem). Units mm, N, MPa. Index conventions as printopt (C-order (nx, ny, nz), node
id (i*(ny+1)+j)*(nz+1)+k, dof 3*node+axis).
"""
import os
import io
import time
import contextlib
import copy
import numpy as np
import scipy.sparse as sp
import scipy.linalg as sla
import pymoto as pym
import printopt as PO
import noxopt as NO

# ------------------------------------------------------------------------------------------------ sparse products
if "MKL_RT" not in os.environ:                             # where pip's mkl puts mkl_rt (Linux box / Windows venv)
    import sys, glob
    _cand = (glob.glob("/usr/local/lib/libmkl_rt.so*") + glob.glob(os.path.join(sys.prefix, "lib", "libmkl_rt.so*"))
             + glob.glob(os.path.join(sys.prefix, "Library", "bin", "mkl_rt*.dll")))
    if _cand: os.environ["MKL_RT"] = _cand[0]
try:
    from sparse_dot_mkl import dot_product_mkl as _mkl_dot
    HAVE_MKL = True
except Exception:                                          # pragma: no cover
    HAVE_MKL = False                                       # scipy products: same results, ~3x slower multigrid
if os.environ.get("MKL_RT"):                               # pypardiso: skip its ~2 s venv search per new solver
    os.environ.setdefault("PYPARDISO_MKL_RT", os.environ["MKL_RT"])


def spmm(A, X):
    """A (sparse) @ X (dense 2D, C-contiguous)."""
    if HAVE_MKL:
        return _mkl_dot(A, np.ascontiguousarray(X))
    return A @ X


CALIBRATE = (60_000, 700_000)   # auto: free dofs between these: time both solvers once; below: direct; above: multigrid
def _direct_fits(nfree, log=print):
    """would a direct (pardiso) solve fit in the free memory? Estimate from the 9 Oct measurements: 4.8 GB peak at
    453k free dof, growing ~n^1.33 (3D fill-in). Keeps 25 % of the free memory in reserve."""
    est = 4.8 * (nfree / 453e3) ** 1.33
    try:
        import psutil
        free = psutil.virtual_memory().available / 1e9
    except Exception:
        return True
    ok = est < 0.75 * free
    if not ok:
        log(f"  auto: direct solve needs ~{est:.1f} GB, {free:.1f} GB free -> multigrid")
    return ok


CORNERS = np.array([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)])


# ------------------------------------------------------------------------------------------------ lean model
class LitePO(PO.PrintOpt):
    """The parts of printopt.PrintOpt that the layout and the strength check use, without the global COO index
    arrays (built only if a printopt method asks for them). Same attributes and numbering."""

    def __init__(self, pb):
        self.pb = pb; nx, ny, nz = pb.shape; self.nel = nx * ny * nz; h = pb.h
        self.axis = PO.AXES[pb.build][0]
        self.C = PO.stiffness(pb.material, self.axis)
        self.KE, self.B0 = PO.hex_KE(h, self.C)
        self.CB = self.C @ self.B0
        ex, ey, ez = [a.ravel() for a in np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij")]
        self.eijk = np.stack([ex, ey, ez], 1).astype(np.int32)
        nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
        self.enodes = np.stack([nid(ex + a, ey + b, ez + c) for a, b, c in CORNERS], 1)
        self.edof = np.repeat(self.enodes * 3, 3, axis=1) + np.tile([0, 1, 2], 8)
        self.ecent = np.stack([ex, ey, ez], 1) * h + np.asarray(pb.origin) + h / 2
        self.nn = (nx + 1) * (ny + 1) * (nz + 1); self.ndof = 3 * self.nn
        self.act = ~pb.void
        used = np.zeros(self.ndof, bool); used[self.edof[self.act].ravel()] = True
        self.used = used
        self.freedofs = {k: np.setdiff1d(np.where(used)[0], v) for k, v in pb.supports.items()}
        self.design = ~pb.solid & ~pb.void
        wg = pb.material.density * 1e-6 * h ** 3 * 9.81
        self.wg = wg
        self.fe = [None if lc.accel is None else np.tile(np.asarray(lc.accel, float) * wg / 8, 8) for lc in pb.cases]
        lay = [k for k, (i, j) in PO.VOIGT_PAIRS.items() if self.axis in (i, j)]
        self.layer_shear = lay
        self.am = None; self.c_ref = None; self.cmax = np.ones(len(pb.cases)); self.hist = []
        hot = np.zeros(self.nn, bool); mnodes = np.unique(self.enodes[self.act]); self.body_like = []
        for lc in pb.cases:
            ln = np.unique(np.where(lc.F != 0)[0] // 3)
            if len(ln) > self.BODY_FRAC * len(mnodes):
                self.body_like.append(lc.name); continue
            hot[ln] = True
        for v in pb.supports.values(): hot[np.unique(v // 3)] = True
        self.smask = self.act & ~hot[self.enodes].any(1) if pb.stress_exclude is None else self.act & ~pb.stress_exclude

    def __getattr__(self, name):                       # printopt's COO indices, only if a printopt method needs them
        if name in ("iK", "jK"):
            ed = self.edof[self.act]
            self.iK = np.repeat(ed, 24, axis=1).ravel(); self.jK = np.tile(ed, (1, 24)).ravel()
            return self.__dict__[name]
        raise AttributeError(name)


# ------------------------------------------------------------------------------------------------ multigrid
def _prolong_1d(n_f):
    """fine nodes 0..n_f onto coarse nodes 0..ceil(n_f/2) (linear)."""
    n_c = (n_f + 1) // 2
    rows, cols, vals = [], [], []
    for f in range(n_f + 1):
        if f % 2 == 0:
            rows.append(f); cols.append(f // 2); vals.append(1.0)
        else:
            rows += [f, f]; cols += [(f - 1) // 2, (f + 1) // 2]; vals += [0.5, 0.5]
    return sp.csr_matrix((vals, (rows, cols)), shape=(n_f + 1, n_c + 1)), n_c


def _child_Q():
    """Q[c] (24, 24): fine-element dofs of child c (c = 4*di + 2*dj + dk) from its coarse element's dofs."""
    Q = np.zeros((8, 24, 24))
    for c in range(8):
        off = np.array([c // 4, (c // 2) % 2, c % 2])
        Qn = np.zeros((8, 8))
        for a, oa in enumerate(CORNERS):
            t = off + oa                                   # in fine units 0..2 inside the coarse element
            for A, OA in enumerate(CORNERS):
                Qn[a, A] = np.prod(np.where(OA == 1, t / 2.0, 1 - t / 2.0))
        Q[c] = np.kron(Qn, np.eye(3))
    return Q


class Level:
    """one grid level: active elements, node pattern (3x3 block BSR), block positions of every element pair."""

    def __init__(self, shape, act_ijk):
        self.shape = tuple(int(s) for s in shape)
        nx, ny, nz = self.shape
        self.ijk = act_ijk                                  # (n, 3) active element indices
        self.n = len(act_ijk)
        self.nn = (nx + 1) * (ny + 1) * (nz + 1)
        i, j, k = (act_ijk[:, d].astype(np.int64) for d in range(3))
        en = np.stack([((i + a) * (ny + 1) + (j + b)) * (nz + 1) + (k + c) for a, b, c in CORNERS], 1)
        self.enodes = en
        keys = [np.arange(self.nn, dtype=np.int64) * self.nn + np.arange(self.nn)]      # every diagonal block
        for a in range(8):
            for b in range(8):
                keys.append(en[:, a] * self.nn + en[:, b])
        keys = np.unique(np.concatenate(keys)); del_ = None
        self.rows = (keys // self.nn).astype(np.int64)
        self.indices = (keys % self.nn).astype(np.int32)
        self.indptr = np.concatenate([[0], np.cumsum(np.bincount(self.rows, minlength=self.nn))]).astype(np.int64)
        self.nnzb = len(keys)
        self.pos = np.empty((8, 8, self.n), np.int64 if self.nnzb > 2 ** 31 - 1 else np.int32)
        for a in range(8):
            for b in range(8):
                self.pos[a, b] = np.searchsorted(keys, en[:, a] * self.nn + en[:, b])
        self.pdiag = np.searchsorted(keys, np.arange(self.nn, dtype=np.int64) * (self.nn + 1))
        used = np.zeros(self.nn, bool); used[en.ravel()] = True
        self.used = used
        del keys

    def assemble(self, Ke=None, KE=None, Eel=None):
        """block data (nnzb, 3, 3): from per-element matrices Ke (n, 24, 24), or Eel (n,) x KE (24, 24)."""
        data = np.zeros((self.nnzb, 3, 3))
        for a in range(8):
            for b in range(8):
                if Ke is not None:
                    data[self.pos[a, b]] += Ke[:, 3 * a:3 * a + 3, 3 * b:3 * b + 3]
                else:
                    data[self.pos[a, b]] += Eel[:, None, None] * KE[3 * a:3 * a + 3, 3 * b:3 * b + 3][None]
        return data

    def bsr(self, data):
        return sp.bsr_matrix((data, self.indices, self.indptr), shape=(3 * self.nn, 3 * self.nn))


class Hierarchy:
    """geometric multigrid for one model (fixed topology). update(Eel) assembles every level for the current
    moduli; prepare(fixed_dofs) sets one support group's constraints; solve(B, X0) runs PCG."""

    def __init__(self, shape, act, KE, coarse_dof=150000, min_dim=2, sweeps=3, ratio=30.0, gamma=2, coarse_sweeps=6, log=print):
        self.KE = KE; self.sweeps = sweeps; self.csweeps = coarse_sweeps or sweeps; self.ratio = ratio; self.gamma = gamma; self.log = log
        t0 = time.time()
        shape = tuple(shape)
        ijk = np.stack(np.unravel_index(np.where(act)[0], shape), 1).astype(np.int64)
        self.levels = [Level(shape, ijk)]
        self.act_idx = np.where(act)[0]
        self.P, self.child = [], []                        # P[l]: level l nodes <- level l+1 nodes (scalar, CSR)
        self.Q = _child_Q()
        while True:
            L = self.levels[-1]
            if 3 * L.nn <= coarse_dof or min(L.shape) < 2 * min_dim:
                break
            Px, ncx = _prolong_1d(L.shape[0]); Py, ncy = _prolong_1d(L.shape[1]); Pz, ncz = _prolong_1d(L.shape[2])
            P = sp.kron(Px, sp.kron(Py, Pz, format="csr"), format="csr")
            par = L.ijk // 2
            ctype = (L.ijk[:, 0] % 2) * 4 + (L.ijk[:, 1] % 2) * 2 + (L.ijk[:, 2] % 2)
            cshape = (ncx, ncy, ncz)
            pid = np.ravel_multi_index(par.T, cshape)
            upid, inv = np.unique(pid, return_inverse=True)
            cijk = np.stack(np.unravel_index(upid, cshape), 1).astype(np.int64)
            self.P.append(P); self.child.append((inv, ctype))
            self.levels.append(Level(cshape, cijk))
        self.PT = [P.T.tocsr() for P in self.P]
        self.M0 = np.einsum("cia,ij,cjb->cab", self.Q, KE, self.Q).reshape(8, 576)     # level 1 from scalar moduli
        Ls = self.levels
        self.log(f"  multigrid: {len(Ls)} levels, dof " + " / ".join(f"{3 * L.nn:,}" for L in Ls)
                 + f", block nnz {Ls[0].nnzb:,} ({Ls[0].nnzb * 72 / 1e9:.2f} GB fine), setup {time.time() - t0:.1f}s")
        self.A = None; self.data = None

    def update(self, Eel_act):
        """fine level (unconstrained; constraints are applied in the products) for element moduli of the active
        elements. Coarse levels are built per support group in prepare()."""
        Ls = self.levels; self.Eel = Eel_act
        self.data = [Ls[0].assemble(Eel=Eel_act, KE=self.KE)] + [None] * (len(Ls) - 1)
        self.A = [Ls[0].bsr(self.data[0])] + [None] * (len(Ls) - 1)
        if len(Ls) > 1:
            inv, ctype = self.child[0]
            self.Ec = np.zeros((Ls[1].n, 8)); self.Ec[inv, ctype] = Eel_act

    def prepare(self, fixed_dofs):
        """one support group: constrained Galerkin coarse operators (P_c^T K_c P_c with P_c = P with the fixed fine
        dofs zeroed, built element by element), block-Jacobi inverses, coarsest factorisation."""
        Ls = self.levels
        free = np.ones((Ls[0].nn, 3), bool); free.ravel()[fixed_dofs] = False
        free &= Ls[0].used[:, None]
        self.free = [free]
        if len(Ls) > 1:
            L0 = Ls[0]; inv, ctype = self.child[0]
            Ke = (self.Ec @ self.M0).reshape(Ls[1].n, 24, 24)
            dm = free[L0.enodes].reshape(L0.n, 24)                       # free dofs of each fine element
            sp_el = np.where(~dm.all(1))[0]                              # fine elements touching a fixed dof
            if len(sp_el):
                D = dm[sp_el].astype(float)
                dK = self.Eel[sp_el, None, None] * (D[:, :, None] * self.KE[None] * D[:, None, :] - self.KE[None])
                for c in range(8):
                    m = ctype[sp_el] == c
                    if m.any():
                        np.add.at(Ke, inv[sp_el[m]], np.einsum("ia,nij,jb->nab", self.Q[c], dK[m], self.Q[c], optimize=True))
            for l in range(1, len(Ls)):
                if l > 1:
                    inv, ctype = self.child[l - 1]
                    Kc = np.zeros((Ls[l].n, 24, 24))
                    for c in range(8):
                        m = ctype == c
                        if m.any():
                            Kc[inv[m]] += np.einsum("ia,nij,jb->nab", self.Q[c], Ke[m], self.Q[c], optimize=True)
                    Ke = Kc
                self.data[l] = Ls[l].assemble(Ke=Ke)
                self.A[l] = Ls[l].bsr(self.data[l])
                dg = np.einsum("nii->ni", self.data[l][Ls[l].pdiag])
                self.free.append((dg > 1e-10 * dg.max()) & Ls[l].used[:, None])
            del Ke
        # constraints built into each level's matrix: fixed / unused dofs get an identity row and column
        self.Abc, self.Dinv, self.lmax = [], [], []
        I3 = np.eye(3)
        for l, L in enumerate(Ls):
            f = self.free[l]
            if l == 0:
                d = self.data[0].copy(); d *= f[L.rows][:, :, None]; d *= f[L.indices][:, None, :]
            else:
                d = self.data[l]
            d[L.pdiag] += np.einsum("ni,ij->nij", ~f, I3)
            self.Abc.append(L.bsr(d))
            if l < len(Ls) - 1:
                self.Dinv.append(np.linalg.inv(d[L.pdiag]))
                self.lmax.append(self._lmax(l))
        self.data = [self.data[0]] + [None] * (len(Ls) - 1)      # coarse data now lives in Abc
        # coarsest: direct solve
        import pypardiso
        if getattr(self, "_ps", None) is not None:
            self._ps.free_memory(everything=True)
        from pypardiso.pardiso_wrapper import PyPardisoError
        Ac = self.Abc[-1].tocsr()
        for mt in (2, 11):
            self._ps = pypardiso.PyPardisoSolver(); self._ps.set_matrix_type(mt)
            self._Ac = sp.triu(Ac, format="csr") if mt == 2 else Ac
            try:
                self._ps.factorize(self._Ac); break
            except PyPardisoError:                             # near-singular coarse grid: LU with pivoting
                self._ps.free_memory(everything=True)
                if mt == 11: raise
        self.mf = [fl.reshape(-1, 1).astype(float) for fl in self.free]

    def free_memory(self):
        if getattr(self, "_ps", None) is not None:
            self._ps.free_memory(everything=True); self._ps = None
        self.Abc = []

    def _Dinv(self, l, R):
        nn = self.levels[l].nn; k = R.shape[1]
        return np.matmul(self.Dinv[l], R.reshape(nn, 3, k)).reshape(3 * nn, k)

    def _lmax(self, l, its=12):
        """largest eigenvalue of D^-1 A on level l (power iteration)."""
        v = np.random.default_rng(l).standard_normal((3 * self.levels[l].nn, 1)); lam = 1.0
        for _ in range(its):
            w = self._Dinv(l, spmm(self.Abc[l], v)); lam = float(np.linalg.norm(w) / np.linalg.norm(v)); v = w / np.linalg.norm(w)
        return lam

    def _cheb(self, l, X, R, B):
        """Chebyshev smoothing (degree = sweeps) with block-Jacobi on [lmax/ratio, 1.1 lmax]; R = B - A X on entry.
        Returns X and, if asked, the updated residual."""
        A = self.Abc[l]; hi = 1.25 * self.lmax[l]; lo = hi / self.ratio
        th, de = 0.5 * (hi + lo), 0.5 * (hi - lo); sig = th / de; rho = 1.0 / sig
        Dd = self._Dinv(l, R) / th
        X = X + Dd
        for i in range(1, self.sweeps if l == 0 else self.csweeps):
            R = R - spmm(A, Dd)
            rn = 1.0 / (2 * sig - rho)
            Dd = (rn * rho) * Dd + (2 * rn / de) * self._Dinv(l, R)
            rho = rn; X += Dd
        R = R - spmm(A, Dd)
        return X, R

    def vcycle(self, l, B):
        if l == len(self.levels) - 1:
            X = self._ps.solve(self._Ac, np.ascontiguousarray(B))
            return X.reshape(B.shape)
        X, R = self._cheb(l, np.zeros_like(B), B, B)
        nn, nc, k = self.levels[l].nn, self.levels[l + 1].nn, B.shape[1]
        Rc = spmm(self.PT[l], R.reshape(nn, 3 * k)).reshape(3 * nc, k)
        Xc = self.vcycle(l + 1, Rc)
        if self.gamma > 1 and l + 1 < len(self.levels) - 1:      # W-cycle: a second coarse correction
            Xc += self.vcycle(l + 1, Rc - spmm(self.Abc[l + 1], Xc))
        dX = spmm(self.P[l], Xc.reshape(nc, 3 * k)).reshape(3 * nn, k); dX *= self.mf[l]
        X += dX
        X, _ = self._cheb(l, X, B - spmm(self.Abc[l], X), B)
        return X

    def _Amul(self, l, X):
        return spmm(self.Abc[l], X)

    def solve(self, B, X0=None, tol=1e-6, maxit=400, chunk=32):
        """PCG on every column of B (ndof, k); constrained / unused dofs come out 0. Returns X, iterations."""
        m = self.mf[0]; self.converged = True
        X = np.zeros_like(B) if X0 is None else X0 * m
        its = []
        for c0 in range(0, B.shape[1], chunk):
            sl = slice(c0, c0 + chunk)
            Bc = B[:, sl] * m; Xc = np.ascontiguousarray(X[:, sl])
            bn = np.linalg.norm(Bc, axis=0); bn[bn == 0] = 1.0
            R = Bc - self._Amul(0, Xc)
            if (np.linalg.norm(R, axis=0) / bn).max() <= tol:
                X[:, sl] = Xc; its.append(0); continue
            Z = self.vcycle(0, R); Pd = Z.copy(); rz = (R * Z).sum(0)
            it = 0
            for it in range(1, maxit + 1):
                AP = self._Amul(0, Pd)
                pap = (Pd * AP).sum(0); pap[pap == 0] = 1.0
                al = rz / pap
                Xc += al * Pd; R -= al * AP
                res = np.linalg.norm(R, axis=0) / bn
                if res.max() <= tol: break
                Z = self.vcycle(0, R); rzn = (R * Z).sum(0)
                be = rzn / np.where(rz == 0, 1.0, rz); rz = rzn
                Pd = Z + be * Pd
            else:
                self.log(f"  multigrid CG: not converged after {maxit} its (residual {res.max():.2e})")
                self.converged = False
            X[:, sl] = Xc; its.append(it)
        return X, its


# ------------------------------------------------------------------------------------------------ compliance
def _direct(Kfull, fd, B, log):
    """direct solve of K[fd, fd] U = B[fd] (pardiso Cholesky, LU if not positive definite), as printopt.solve."""
    import pypardiso
    from pypardiso.pardiso_wrapper import PyPardisoError
    Kff = Kfull[fd][:, fd].tocsr(); U = np.zeros_like(B)
    for mt in (2, 11):
        Kf = sp.triu(Kff, format="csr") if mt == 2 else Kff
        ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(mt)
        try:
            U[fd] = ps.solve(Kf, np.ascontiguousarray(B[fd])).reshape(len(fd), -1); ps.free_memory(everything=True); break
        except PyPardisoError as e:
            ps.free_memory(everything=True)
            if mt == 11: raise
            log(f"  Cholesky failed ({e}); using LU")
    return U


def _basis(Fcols, rtol=1e-10):
    """independent subset of the static load columns (sparse ndof x ns) and every column's coefficients on it."""
    ns = Fcols.shape[1]
    if ns == 0: return [], np.zeros((0, 0))
    G = (Fcols.T @ Fcols).toarray(); scale = max(np.diag(G).max(), 1e-300)
    sel = []
    for i in range(ns):
        if G[i, i] <= rtol * scale: continue
        if sel:
            Gs = G[np.ix_(sel, sel)]; gi = G[sel, i]
            res = G[i, i] - gi @ np.linalg.solve(Gs, gi)
            if res <= 1e-9 * G[i, i]: continue
        sel.append(i)
    if not sel: return [], np.zeros((0, ns))
    Gs = G[np.ix_(sel, sel)]
    coef = np.linalg.solve(Gs, G[sel, :])                  # (nb, ns)
    return sel, coef


class Compliance(pym.Module):
    """c_i = F_i(x) . u_i for every case: superposed basis solves per support group, multigrid PCG or pardiso.
    Same inputs, outputs and sensitivities as noxopt.Compliance."""

    def __init__(self, po, backend="auto", tol=1e-6, auto_dof=200_000, chunk_el=200_000, log=print, mg_opts=None):
        self.po = po; self.log = log; self.tol = tol; self.chunk_el = chunk_el
        pb = po.pb; nc = len(pb.cases)
        nfree = len(po.freedofs[pb.cases[0].support]) if nc else 0
        self.backend = backend if backend != "auto" else ("gmg" if nfree > auto_dof else "pardiso")
        self.groups = {}
        for i, lc in enumerate(pb.cases):
            self.groups.setdefault(lc.support, []).append(i)
        self.a = po.act; self.ed = po.edof[self.a]
        self.G = {}
        self.nsolve = 0
        for sup, idx in self.groups.items():
            F = sp.csc_matrix(np.stack([pb.cases[i].F for i in idx], 1)) if idx else None
            sel, coef = _basis(F)
            acc = np.array([pb.cases[i].accel if pb.cases[i].accel is not None else np.zeros(3) for i in idx], float)
            has_acc = bool(np.abs(acc).sum() > 0)
            nb = len(sel); m = nb + (3 if has_acc else 0)
            Cf = np.zeros((m, len(idx)))
            if nb: Cf[:nb] = coef
            if has_acc: Cf[nb:] = acc.T                      # body basis: unit g in x, y, z on the current density
            err = 0.0
            if nb:
                rec = F[:, sel] @ coef - F
                err = float(abs(rec).max() / max(abs(F).max(), 1e-300))
            self.G[sup] = dict(idx=idx, Fb=F[:, sel].toarray() if nb else np.zeros((po.ndof, 0)), nb=nb,
                               acc=has_acc, C=Cf, U=None)
            self.nsolve += m
            if err > 1e-8: self.log(f"  superposition warning: support {sup} reconstruction error {err:.1e}")
        self.log(f"  compliance: {nc} cases in {len(self.groups)} support groups -> {self.nsolve} basis solves "
                 f"per evaluation; backend {self.backend}"
                 + ("" if self.backend == "pardiso" else f", sparse products {'MKL' if HAVE_MKL else 'scipy (MKL not found: ~3x slower)'}"))
        self.H = None
        if self.backend == "gmg":
            self.H = Hierarchy(pb.shape, self.a, po.KE, log=log, **(mg_opts or {}))
        self.stats = []

    def __call__(self, x):
        return self.calc(x)

    def _body_basis(self, x):
        po = self.po; a = self.a
        w = x[a] * po.wg / 8.0
        Bb = np.zeros((po.ndof, 3))
        for d in range(3):
            Bb[:, d] = np.bincount(self.ed[:, d::3].ravel(), weights=np.repeat(w, 8), minlength=po.ndof)
        return Bb

    def calc(self, x):
        po = self.po; pb = po.pb; nc = len(pb.cases); t0 = time.time()
        self.x = x
        Eel = po.EMIN + x ** po.PEN * (1 - po.EMIN)
        Bb = self._body_basis(x) if any(g["acc"] for g in self.G.values()) else None
        c = np.zeros(nc); its_all = []
        Kfull = None
        if self.backend == "gmg":
            self.H.update(Eel[self.a])
        else:
            L0 = getattr(self, "_L0", None)
            if L0 is None:
                ijk = np.stack(np.unravel_index(np.where(self.a)[0], pb.shape), 1).astype(np.int64)
                self._L0 = L0 = Level(pb.shape, ijk)
            Kfull = L0.bsr(L0.assemble(Eel=Eel[self.a], KE=po.KE)).tocsr()
        for sup, g in self.G.items():
            B = g["Fb"]
            if g["acc"]: B = np.concatenate([B, Bb], 1)
            g["B"] = B
            if self.backend == "gmg":
                self.H.prepare(pb.supports[sup])
                U, its = self.H.solve(B, X0=g["U"], tol=self.tol)
                its_all += its
                if not self.H.converged:                          # never pass an unconverged solve to MMA
                    self.log(f"  support {sup}: multigrid did not converge, solving this group directly")
                    if Kfull is None: Kfull = self.H.levels[0].bsr(self.H.data[0]).tocsr()
                    U = _direct(Kfull, po.freedofs[sup], B, self.log)
            else:
                U = _direct(Kfull, po.freedofs[sup], B, self.log)
            g["U"] = U
            M = B.T @ U                                           # (m, m)
            Cg = g["C"]
            c[g["idx"]] = np.einsum("ai,ab,bi->i", Cg, M, Cg)
        if self.backend == "gmg": self.H.free_memory(); self.H.data = None
        else: del Kfull
        self.stats.append((time.time() - t0, its_all))
        return c

    def case_displacements(self, i):
        """full displacement of case i (for strength checks)."""
        for g in self.G.values():
            if i in g["idx"]:
                return g["U"] @ g["C"][:, g["idx"].index(i)]

    def _sensitivity(self, dc):
        po = self.po; x = self.x; a = self.a
        xa = x[a]; dE = po.PEN * xa ** (po.PEN - 1) * (1 - po.EMIN)
        acc = np.zeros(a.sum())
        for sup, g in self.G.items():
            w = dc[g["idx"]]
            if not np.any(w): continue
            Cg = g["C"]
            W = (Cg * w) @ Cg.T                                   # sum_i w_i c_i c_i^T
            lam, V = np.linalg.eigh(0.5 * (W + W.T))
            keep = np.abs(lam) > 1e-14 * max(np.abs(lam).max(), 1e-300)
            lam, V = lam[keep], V[:, keep]
            UV = g["U"] @ V                                       # (ndof, r)
            Gb = None
            if g["acc"]:
                acc_i = Cg[g["nb"]:]                              # (3, ncase)
                Gb = (acc_i * w) @ Cg.T                           # (3, m): sum_i w_i a_i c_i^T
            for s in range(0, len(xa), self.chunk_el):
                ed = self.ed[s:s + self.chunk_el]
                ue = UV[ed]                                       # (n, 24, r)
                kue = np.einsum("ij,njr->nir", po.KE, ue)
                q = (ue * kue).sum(1) @ lam                       # sum_r lam_r u^T KE u
                d = -dE[s:s + self.chunk_el] * q
                if Gb is not None:
                    Ue = g["U"][ed]                               # (n, 24, m)
                    S = Ue.reshape(len(ed), 8, 3, -1).sum(1)      # (n, 3, m): sum of the 8 node displacements
                    d = d + 2 * po.wg / 8.0 * np.einsum("ndm,dm->n", S, Gb)
                acc[s:s + self.chunk_el] += d
        out = np.zeros(po.nel); out[a] = acc
        return out


# ------------------------------------------------------------------------------------------------ layout
def add_tie_cases(pb, min_cells=4, log=print):
    """noxopt.add_tie_cases on the lean model (identical cases)."""
    po0 = LitePO(NO._no_comp(pb))
    count = {}
    for lc in pb.cases: count[lc.support] = count.get(lc.support, 0) + 1
    main = max(count, key=count.get)
    fixed_nodes = np.zeros(po0.nn, bool); fixed_nodes[np.unique(pb.supports[main] // 3)] = True
    bodies, small = NO.interface_bodies(pb, min_cells)
    cases, info = [], []
    for k, el in bodies:
        nd = np.unique(po0.enodes[el])
        if fixed_nodes[nd].any():
            info.append((k, len(el), "held by support")); continue
        for ax in range(3):
            F = np.zeros(po0.ndof); F[3 * nd + ax] = 1.0 / len(nd)
            cases.append(PO.LoadCase(name=f"tie b{k} {'xyz'[ax]}", F=F, cls=None, support=main))
        info.append((k, len(el), "tied"))
    log(f"noxopt: {len(bodies)} interface bodies ({sum(1 for i in info if i[2] == 'tied')} tied with 3 cases each, "
        f"{sum(1 for i in info if i[2] != 'tied')} held by support '{main}'); {len(small)} fragments under {min_cells} "
        f"cells not tied")
    pb2 = copy.copy(pb); pb2.cases = list(pb.cases) + cases
    return pb2, len(pb.cases), info


class Layout(NO.Layout):
    """noxopt.Layout with the local compliance backend. Same arguments plus backend / tol / mg_opts; same run(),
    checkpoints and save(). Results agree with noxopt.Layout to the solver tolerance (test_noxopt_local.py)."""

    def __init__(self, pb, volfrac, R=None, eta=(0.7, 0.5, 0.3), P=4.0, tie_weight=0.25, tie_min_cells=4, log=print,
                 backend="auto", tol=1e-6, mg_opts=None, auto_dof=200_000):
        self.log = log; self.pb0 = pb; self.volfrac = volfrac
        self.pb, self.n_real, self.tie_info = add_tie_cases(pb, tie_min_cells, log)
        pb = self.pb
        self.po = LitePO(NO._no_comp(pb))
        po = self.po
        self.design = po.design; self.nel = po.nel
        R = pb.R if R is None else R; self.R = R
        nx, ny, nz = pb.shape
        self.dom = pym.VoxelDomain(nz, ny, nx)
        solid = np.where(pb.solid)[0]; void = np.where(pb.void)[0]
        ax, sgn = PO.AXES[pb.build]
        direction = [0.0, 0.0, 0.0]; direction[2 - ax] = float(sgn)
        solid_x = np.where(pb.void, 0.0, 1.0)
        nfree = len(po.freedofs[pb.cases[0].support])
        if backend == "auto" and CALIBRATE[0] < nfree < CALIBRATE[1] and _direct_fits(nfree, log):
            # between the two limits: time both on the solid-domain reference solve (needed anyway). Multigrid is
            # kept only if clearly faster (< 0.8 x direct): on Sparks Book a 0.85 x reference solve still gave a
            # whole run no faster than direct (9 Oct, 1 mm holder: 19.0 vs 18.7 s/it).
            t = time.time(); cp = Compliance(po, backend="pardiso", tol=tol, log=log)
            c_p = cp.calc(solid_x).copy(); tp = time.time() - t
            t = time.time(); cg = Compliance(po, backend="gmg", tol=tol, log=log, mg_opts=mg_opts)
            c_g = cg.calc(solid_x).copy(); tg = time.time() - t
            comp = cg if tg < 0.8 * tp else cp
            log(f"  auto: direct {tp:.1f}s, multigrid {tg:.1f}s on the reference solve -> {comp.backend}")
            if comp is cp: cg.H = None
            self.c_ref = c_g if comp is cg else c_p
        else:
            be = backend
            if backend == "auto" and CALIBRATE[0] < nfree < CALIBRATE[1]:
                be = "gmg"                                  # direct would not fit in free memory
            comp = Compliance(po, backend=be, tol=tol, log=log, mg_opts=mg_opts, auto_dof=auto_dof)
            self.c_ref = comp.calc(solid_x).copy()
        for g in comp.G.values(): g["U"] = None              # the solid-domain solution is a poor warm start
        bad = [lc.name for lc, c in zip(pb.cases, self.c_ref) if not c > 1e-12 * self.c_ref.max()]
        if bad:
            raise ValueError(f"noxopt: zero compliance in the solid domain for {bad}: the load sits on supported dofs")
        w = np.ones(len(pb.cases)); w[self.n_real:] = tie_weight
        self.sx = pym.Signal("x", state=np.full(self.nel, volfrac))
        self.sbeta = pym.Signal("beta", state=1.0)
        self.eta = eta
        with pym.Network() as fn:
            sx_ = NO.Symmetrise(pb.sym)(self.sx) if getattr(pb, "sym", None) is not None else self.sx
            s0 = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(sx_))
            sf = NO.MaskedFilter(pb.shape, R, pb.void)(s0)
            fields = []
            for e in eta:
                s = pym.MathExpression(NO.HEAVI.format(e))(sf, self.sbeta)
                s = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(s))
                if pb.overhang:
                    with contextlib.redirect_stdout(io.StringIO()):
                        s = pym.OverhangFilter(self.dom, direction=direction)(s)
                    s = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(s))
                fields.append(s)
            self.s_ero, self.s_nom, self.s_dil = fields
            self.comp = comp
            self.s_c = self.comp(self.s_ero)
            self.pmean = NO.PMean(self.c_ref, w, P)
            self.s_obj = self.pmean(self.s_c)
            self.vol = NO.VolFrac(self.design, volfrac)
            self.s_vol = self.vol(self.s_dil)
        self.fn = fn
        self.hist = []


# ------------------------------------------------------------------------------------------------ strength
def strength_report(pb, dens, thr=0.5, backend="auto", tol=1e-8, log=lambda s: None):
    """noxopt.strength_report for large grids: layer-aware failure index per case of the thresholded design.
    Same criteria as printopt.evaluate (in-layer von Mises, across-layer tension + interlayer shear), without
    printopt's global COO matrices. Returns (rows, report text) like noxopt.strength_report."""
    binary = np.where(pb.solid, 1.0, np.where(pb.void, 0.0, (dens > thr).astype(float)))
    po = LitePO(NO._no_comp(pb))
    comp = Compliance(po, backend=backend, tol=tol, log=log)
    comp.calc(binary)
    sm = po.smask; rows = []
    for i, lc in enumerate(pb.cases):
        if lc.cls is None: continue
        u = comp.case_displacements(i)
        A = PO.allowables(pb.material, lc.cls)
        f1 = np.zeros(po.nel); f2 = np.zeros(po.nel)
        for s in range(0, po.nel, 200_000):
            ue = u[po.edof[s:s + 200_000]]
            st = po.stresses(ue, binary[s:s + 200_000])
            a1, _, a2, _ = po.failure(st, A)
            f1[s:s + 200_000] = a1; f2[s:s + 200_000] = a2
        rows.append(dict(case=lc.name, cls=lc.cls, inlayer=float(f1[sm].max()), across=float(f2[sm].max()),
                         where_inlayer=po.ecent[sm][np.argmax(f1[sm])].round(1).tolist(),
                         where_across=po.ecent[sm][np.argmax(f2[sm])].round(1).tolist()))
    text = "\n".join([f"noxopt_local as built: build {pb.build}, {pb.material.name}, solid {binary.sum() * pb.h ** 3 / 1000:.2f} cm3"]
                     + [f"  {r['case']:28s} [{r['cls']:9s}] in-layer {r['inlayer']:5.2f} at {tuple(r['where_inlayer'])}  "
                        f"across-layer {r['across']:5.2f} at {tuple(r['where_across'])}" for r in rows])
    return rows, text
