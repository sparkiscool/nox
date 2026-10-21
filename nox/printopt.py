"""printopt: print-aware topology optimisation for Nox. Every generated part goes through this.

A part is optimised for the way it will actually be printed:
  * BUILD DIRECTION is chosen first (one grid axis: '+z' means layers stack upward along grid z, the first grid
    layer along that axis sits on the bed).
  * MATERIAL is transversely isotropic: softer (Ez) across the layers than along them.
  * STRENGTH is checked per element in two ways, each against its own allowable:
      in-layer:      von Mises / A_xy
      across layers: sqrt((sigma_n+ / A_z)^2 + (tau_n / A_s)^2), sigma_n = stress normal to the layers (tension only),
                     tau_n = shear on the layer plane (the interlayer bond)
    Allowables = strength x 0.85 process knock-down / safety factor of the load class (misuse, flight, vibration,
    sustained). Across-layer strength = 0.5 x in-layer for PLA/PETG (research report), until coupons say otherwise.
  * Formulation: minimise volume subject to one stress constraint per load case (qp-relaxation p=3, q=2.5, p-norm
    aggregation with the adaptive correction of Le et al. 2010) plus optional displacement limits on precision
    features, solved with MMA (mmapy). Replaces compliance-only SIMP + "stress grow".
  * Optional OVERHANG filter (Langelaar 2017, 45 deg, 5-point support stencil) so every member can print without
    supports in the chosen direction. Passive solids are forced back to 1 after it; unsupported ones are reported.
    Its smooth max (p-norm, P = 40) is scaled by the stencil max so void stencils cannot underflow to 0 / NaN gradients.
FE: evaluate() handles one support group at a time (factorise, forward + adjoint solves, free), so peak memory is one
pardiso factorisation, not one per group; SPD Cholesky (MTYPE 2, LU fallback) halves it again (3 Oct 2026).
Filter chain: x -> density filter (radius R) -> Heaviside (beta continuation) -> overhang filter -> physical density.
Inertia: LoadCase.accel (g) = density-weighted body force on the design's own material, with sensitivities; it never
empties the automatic stress set (only supports and surface-load pads are excluded) and leaves no load on removed
material (3 Oct 2026, dry run gap 6). A nodal body force in F still works (taken as a body force by the auto set).
Checkpoint (B4): run(checkpoint="x.npz", checkpoint_every=10) saves the state; run(resume="x.npz") continues it.
Units mm, N, MPa. See research/reports/Generative design protocol for drones.md.
"""
import os
import time
from dataclasses import dataclass, field
import numpy as np
import scipy.sparse as sp
from scipy.ndimage import correlate


# ----------------------------------------------------------------------------------------------------- materials
@dataclass
class Material:
    name: str
    E: float        # in-layer modulus, MPa
    Ez: float       # across-layer modulus
    nu: float
    Xt: float       # in-layer tensile strength (characteristic), MPa
    Zt: float       # across-layer tensile strength (layer adhesion) used for design
    S: float        # interlayer shear strength used for design (no data yet: = Zt, flagged)
    density: float  # g/cm3


# Values: Bambu TDS (research_notes/fdm_constraints.md). Zt = 0.5 x Xt for PLA/PETG and 0.8 x for TPU, Ez = 0.8 x E
# (research report design rule; manufacturer coupons are kinder, independent standing coupons are worse).
# TPU E = 9.8 MPa, the Bambu TDS value (Protocol Manager ruling, cage v8 stage 2, R20, 3 Oct 2026; was 26 MPa, the
# cage chat's estimate). Ez = 0.8 E = 7.84. Allowables come from Xt / Zt / S, not from E, so they are unchanged.
# Linear elasticity is rough for TPU anyway.
MATERIALS = {
    "PLA": Material("PLA", E=2580, Ez=0.8 * 2580, nu=0.35, Xt=36.0, Zt=18.0, S=18.0, density=1.24),
    "PETG": Material("PETG", E=2780, Ez=0.8 * 2780, nu=0.38, Xt=51.0, Zt=25.5, S=25.5, density=1.27),
    "TPU95A": Material("TPU95A", E=9.8, Ez=0.8 * 9.8, nu=0.45, Xt=27.3, Zt=21.8, S=21.8, density=1.22),
}
KNOCKDOWN = 0.85
# safety factor per load class. misuse 2.5 reproduces the report's PLA numbers (12 MPa in-layer, 6 across layers).
SAFETY = {"crash": 1.1, "misuse": 2.5, "flight": 4.0, "vibration": 7.0, "sustained": 10.0}
# crash: survive-once test-phase crash (~25 g inside the cage); permanent damage allowed, nothing breaks or comes loose


def allowables(mat, cls):
    sf = SAFETY[cls]
    return mat.Xt * KNOCKDOWN / sf, mat.Zt * KNOCKDOWN / sf, mat.S * KNOCKDOWN / sf


_ENV_RESUME_USED = False   # NOX_PRINTOPT_RESUME (autorun S7) applies to the first run() call of a process only

AXES = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}
VOIGT_PAIRS = {3: (0, 1), 4: (1, 2), 5: (2, 0)}                   # fe3d order: xx yy zz xy yz zx


def stiffness(mat, axis):
    """6x6 Voigt stiffness, transversely isotropic about grid axis `axis` (0/1/2 = layer normal)."""
    E = np.full(3, mat.E); E[axis] = mat.Ez
    S = np.zeros((6, 6))
    for i in range(3):
        for j in range(3):
            S[i, j] = 1 / E[i] if i == j else -mat.nu / np.sqrt(E[i] * E[j])
    for k, (i, j) in VOIGT_PAIRS.items():
        G = (mat.Ez if axis in (i, j) else mat.E) / (2 * (1 + mat.nu))
        S[k, k] = 1 / G
    return np.linalg.inv(S)


def hex_KE(h, C):
    """8-node brick stiffness for a cube of side h with constitutive matrix C (2x2x2 Gauss), and the centre B."""
    nodes = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], float)
    s = 2 * nodes - 1; g = 1 / np.sqrt(3); J = h / 2

    def Bmat(xi, et, ze):
        dN = np.array([[s[i, 0] * (1 + s[i, 1] * et) * (1 + s[i, 2] * ze) / 8,
                        s[i, 1] * (1 + s[i, 0] * xi) * (1 + s[i, 2] * ze) / 8,
                        s[i, 2] * (1 + s[i, 0] * xi) * (1 + s[i, 1] * et) / 8] for i in range(8)]) / J
        B = np.zeros((6, 24))
        for i in range(8):
            B[0, 3 * i] = dN[i, 0]; B[1, 3 * i + 1] = dN[i, 1]; B[2, 3 * i + 2] = dN[i, 2]
            B[3, 3 * i] = dN[i, 1]; B[3, 3 * i + 1] = dN[i, 0]
            B[4, 3 * i + 1] = dN[i, 2]; B[4, 3 * i + 2] = dN[i, 1]
            B[5, 3 * i] = dN[i, 2]; B[5, 3 * i + 2] = dN[i, 0]
        return B
    KE = sum(Bmat(a, b, c).T @ C @ Bmat(a, b, c) * J ** 3 for a in (-g, g) for b in (-g, g) for c in (-g, g))
    return KE, Bmat(0, 0, 0)


# --------------------------------------------------------------------------------------------- overhang filter
class OverhangFilter:
    """Langelaar (2017) AM filter on a (nx, ny, nz) density array, build direction one of AXES. Layer 0 = bed."""
    P, XI0, EPS = 40.0, 0.5, 1e-4

    def __init__(self, shape, build):
        self.shape, (self.axis, self.sign) = shape, AXES[build]
        self.Q = self.P + np.log(5) / np.log(self.XI0)

    def _to(self, a):
        a = np.moveaxis(a.reshape(self.shape), self.axis, 0)
        return a[::-1] if self.sign < 0 else a

    def _from(self, a):
        a = a[::-1] if self.sign < 0 else a
        return np.moveaxis(a, 0, self.axis).ravel()

    @staticmethod
    def _stencil(prev):
        p = np.pad(prev, 1)
        return np.stack([p[1:-1, 1:-1], p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])   # self + 4 neighbours

    def forward(self, rho):
        r = self._to(rho); xi = np.empty_like(r); self.cache = []
        xi[0] = r[0]
        PQ = self.P / self.Q
        for i in range(1, r.shape[0]):
            raw = self._stencil(xi[i - 1]); st = np.maximum(raw, 1e-12)
            # smooth max (sum st^P)^(1/Q), max-scaled: sum st^P underflows below st ~8e-9 (P = 40), so with
            # m = max st, S = sum (st/m)^P in [1, 5] and smax = m^(P/Q) S^(1/Q): no under/overflow (3 Oct 2026)
            m = st.max(0); sr = st / m; S = (sr ** self.P).sum(0)
            smax = m ** PQ * S ** (1 / self.Q)
            x, y = r[i], smax; d = np.sqrt((x - y) ** 2 + self.EPS)
            xi[i] = 0.5 * (x + y - d + np.sqrt(self.EPS))
            dsdst = (PQ * (smax / m) / S)[None] * sr ** (self.P - 1)              # d smax / d st_k
            dsdst *= raw >= 1e-12                                                 # d st / d raw: 0 under the clamp
            self.cache.append((dsdst, 0.5 * (1 - (x - y) / d), 0.5 * (1 + (x - y) / d)))  # dsmin/dx, dsmin/dy
        return self._from(xi)

    def backward(self, dxi):
        """dF/dxi (physical) -> dF/drho (input), using the cache of the last forward()."""
        g = self._to(dxi).copy(); out = np.zeros_like(g)
        for i in range(g.shape[0] - 1, 0, -1):
            dsdst, dx, dy = self.cache[i - 1]
            out[i] = g[i] * dx
            ds = (g[i] * dy)[None] * dsdst
            acc = np.zeros((g.shape[1] + 2, g.shape[2] + 2))
            acc[1:-1, 1:-1] += ds[0]; acc[:-2, 1:-1] += ds[1]; acc[2:, 1:-1] += ds[2]
            acc[1:-1, :-2] += ds[3]; acc[1:-1, 2:] += ds[4]
            g[i - 1] += acc[1:-1, 1:-1]
        out[0] = g[0]
        return self._from(out)


# ------------------------------------------------------------------------------------------------------ problem
@dataclass
class LoadCase:
    name: str
    F: np.ndarray                 # (ndof,) nodal forces (surface loads: pads, held items, contacts); zeros if none
    cls: str = "misuse"           # allowable class; None = no stress constraint (e.g. a stiffness-only case)
    support: str = "default"      # key into Problem.supports
    accel: np.ndarray = None      # (3,) inertial load in g on the part's OWN material (the direction it is pushed):
                                  # a density-weighted body force, rho_phys x material density x h^3 x 9.81 x accel per
                                  # element (1/8 per node), with its sensitivity. It follows the design (no load left on
                                  # removed material) and never empties the stress set. None = no body force.


@dataclass
class DispLimit:
    case: str
    L: np.ndarray                 # (ndof,) weights: the constrained quantity is L.u (e.g. mean pin tip deflection)
    limit: float                  # mm; constraint (L.u)^2 <= limit^2


@dataclass
class Problem:
    shape: tuple                  # (nx, ny, nz) elements
    h: float                      # element size, mm
    origin: tuple                 # min corner
    solid: np.ndarray             # passive solid elements (bool, nel)
    void: np.ndarray              # passive void
    supports: dict                # name -> fixed dof array
    cases: list                   # LoadCase
    material: Material
    build: str = "+z"
    overhang: bool = True
    disp: list = field(default_factory=list)
    R: float = 2.0                # density filter radius, elements (min feature ~ 2R h)
    sym: object = None            # callable(full element vector) -> symmetrised vector, or None
    stress_exclude: np.ndarray = None   # elements left out of the stress measure (load/support pads); auto if None:
                                        # elements touching a support or a surface-loaded node. Body forces never
                                        # exclude: LoadCase.accel is not in F, and an F that loads more than
                                        # BODY_FRAC of the model's nodes is taken as a nodal body force (warning).
    comp_factor: float = 20.0     # stiffness safeguard: compliance of every case <= comp_factor x that of the solid
                                  # domain. Stops the stress-only problem from 'solving' itself by disconnecting the
                                  # loads (stress is not measured on the singular load/support pads). None = off.


class PrintOpt:
    EMIN, PEN, QREL, PNORM = 1e-6, 3.0, 2.5, 8.0
    TARGET = 0.95      # aim the aggregated stress at 95% of the allowable: the p-norm runs a few % under the true peak
    BODY_FRAC = 0.5    # auto stress set: an F loading more than this share of the model's nodes is a body force

    def __init__(self, pb: Problem):
        self.pb = pb; nx, ny, nz = pb.shape; self.nel = nx * ny * nz; h = pb.h
        self.axis = AXES[pb.build][0]
        self.C = stiffness(pb.material, self.axis)
        self.KE, self.B0 = hex_KE(h, self.C)
        self.CB = self.C @ self.B0                                  # (6, 24): solid stress per element dof
        ex, ey, ez = [a.ravel() for a in np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing="ij")]
        nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
        corners = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
        self.enodes = np.stack([nid(ex + a, ey + b, ez + c) for a, b, c in corners], 1)
        self.edof = np.repeat(self.enodes * 3, 3, axis=1) + np.tile([0, 1, 2], 8)
        self.ecent = np.stack([ex, ey, ez], 1) * h + np.asarray(pb.origin) + h / 2
        self.nn = (nx + 1) * (ny + 1) * (nz + 1); self.ndof = 3 * self.nn
        self.act = ~pb.void                                         # elements in the model at all
        used = np.zeros(self.ndof, bool); used[self.edof[self.act].ravel()] = True
        self.freedofs = {k: np.setdiff1d(np.where(used)[0], v) for k, v in pb.supports.items()}
        self.iK = np.repeat(self.edof[self.act], 24, axis=1).ravel(); self.jK = np.tile(self.edof[self.act], (1, 24)).ravel()
        self.design = ~pb.solid & ~pb.void
        # filter
        k = int(np.ceil(pb.R)); ii = np.arange(-k, k + 1)
        I, J, K = np.meshgrid(ii, ii, ii, indexing="ij"); self.ker = np.maximum(0.0, pb.R - np.sqrt(I * I + J * J + K * K))
        self.Hs = correlate(np.ones(pb.shape), self.ker, mode="constant")
        self.am = OverhangFilter(pb.shape, pb.build) if pb.overhang else None
        # body forces (LoadCase.accel): per-element nodal vector of a fully solid element, N; F_body = sum rp_e fe
        wg = pb.material.density * 1e-6 * h ** 3 * 9.81                # kg/mm3 x mm3 x m/s2 = N per g, one element
        self.fe = [None if lc.accel is None else np.tile(np.asarray(lc.accel, float) * wg / 8, 8) for lc in pb.cases]
        # stress measure set: skip elements touching surface-loaded or fixed nodes (singular), and void. A body force
        # loads every node of the part, so it must not mark nodes (dry run 3 Oct 2026: a region_mapper body force
        # emptied the set -> "zero-size array"): accel is not in F, and an F spread over > BODY_FRAC of the model's
        # nodes is treated as a nodal body force (its nodes are not excluded; supports still are).
        if pb.stress_exclude is None:
            hot = np.zeros(self.nn, bool)
            mnodes = np.unique(self.enodes[self.act]); self.body_like = []
            for lc in pb.cases:
                ln = np.unique(np.where(lc.F != 0)[0] // 3)
                if len(ln) > self.BODY_FRAC * len(mnodes):
                    self.body_like.append(lc.name); continue
                hot[ln] = True
            for v in pb.supports.values(): hot[np.unique(v // 3)] = True
            self.smask = self.act & ~hot[self.enodes].any(1)
            if self.body_like:
                print(f"printopt: case(s) {', '.join(self.body_like)} load > {self.BODY_FRAC:.0%} of the nodes: taken as "
                      "nodal body forces, their nodes stay in the stress set (fixed load on removed material: prefer "
                      "LoadCase.accel, a density-weighted body force)")
        else:
            self.smask = self.act & ~pb.stress_exclude
        if not self.smask.any():
            raise ValueError("printopt: the stress set is empty (every element touches a load or support, or "
                             "stress_exclude covers the model): pass stress_exclude = the pads only")
        lay = [k for k, (i, j) in VOIGT_PAIRS.items() if self.axis in (i, j)]
        self.layer_shear = lay
        self.cmax = np.ones(len(pb.cases)); self.pn_prev = None
        self.hist = []
        self.c_ref = None
        if pb.comp_factor:                                          # compliance of the fully solid design domain
            self.c_ref = np.zeros(len(pb.cases))
            vol, g, *_ = self.evaluate(np.where(pb.void, 0.0, 1.0), 16.0, grads=False, _ref=True)
            self.c_ref = self._comp * pb.comp_factor
            bad = [lc.name for lc, c in zip(pb.cases, self._comp) if not c > 1e-12 * max(self._comp.max(), 1e-30)]
            if bad:
                raise ValueError(f"printopt: zero compliance in the solid domain for {bad}: the load sits on supported dofs "
                                 "(the comp_factor constraint would be 0/0 = NaN); drop the case or move its load")

    # ---- density chain
    def filt(self, v): return (correlate(v.reshape(self.pb.shape), self.ker, mode="constant") / self.Hs).ravel()
    def filt_T(self, v): return correlate(v.reshape(self.pb.shape) / self.Hs, self.ker, mode="constant").ravel()

    def physical(self, rho, beta):
        rt = self.filt(rho)
        t = np.tanh(beta * 0.5); rp = (t + np.tanh(beta * (rt - 0.5))) / (2 * t)
        self._dproj = beta * (1 - np.tanh(beta * (rt - 0.5)) ** 2) / (2 * t)
        rp = np.where(self.pb.solid, 1.0, np.where(self.pb.void, 0.0, rp))
        r = self.am.forward(rp) if self.am else rp
        return np.where(self.pb.solid, 1.0, np.where(self.pb.void, 0.0, r))

    def chain_T(self, d):
        """sensitivity wrt physical density -> wrt design rho (through overhang, projection, filter)."""
        d = np.where(self.pb.solid | self.pb.void, 0.0, d)
        if self.am: d = self.am.backward(d)
        d = np.where(self.pb.solid | self.pb.void, 0.0, d)
        return self.filt_T(d * self._dproj)

    # ---- FE
    # pardiso matrix type: 2 = real SPD (Cholesky on the upper triangle; ~half the memory and time of LU, results
    # equal to 1e-13; falls back to LU if a group is not positive definite), 11 = real nonsymmetric LU (before 3 Oct)
    MTYPE = 2

    def assemble(self, Eel):
        sK = (self.KE.ravel()[None, :] * Eel[self.act][:, None]).ravel()
        return sp.coo_matrix((sK, (self.iK, self.jK)), shape=(self.ndof, self.ndof)).tocsr()

    def solve(self, Eel, rhs_by_support, K=None):
        """rhs_by_support: {support: (ndof, k) array}. Returns {support: (U, solver, Kf, fd)}, one factorisation per
        support, each alive until the caller frees it (solver.free_memory(everything=True)). evaluate() passes one
        support at a time, so only one factorisation exists at once."""
        import pypardiso
        from pypardiso.pardiso_wrapper import PyPardisoError
        K = self.assemble(Eel) if K is None else K
        out = {}
        for sup, B in rhs_by_support.items():
            fd = self.freedofs[sup]; Kfull = K[fd][:, fd].tocsr(); U = np.zeros_like(B)
            for mt in ((2, 11) if self.MTYPE == 2 else (self.MTYPE,)):
                Kf = sp.triu(Kfull, format="csr") if mt == 2 else Kfull
                ps = pypardiso.PyPardisoSolver(); ps.set_matrix_type(mt)
                # pypardiso globs the whole venv for mkl_rt on every new solver (~2 s each); remember the path it found
                os.environ.setdefault("PYPARDISO_MKL_RT", ps.libmkl._name)
                try:
                    U[fd] = ps.solve(Kf, np.ascontiguousarray(B[fd])).reshape(len(fd), -1); break
                except PyPardisoError as e:                # not positive definite: fall back to LU
                    ps.free_memory(everything=True)
                    if mt == 11: raise
                    print(f"printopt: Cholesky failed for support '{sup}' ({e}); using LU (mtype 11)")
            del Kfull
            out[sup] = (U, ps, Kf, fd)
        return out

    def force(self, i, rp):
        """total nodal load of case i at physical density rp: F + density-weighted body force (LoadCase.accel)."""
        lc = self.pb.cases[i]
        if self.fe[i] is None:
            return lc.F
        a = self.act
        w = (rp[a][:, None] * self.fe[i][None, :]).ravel()
        return lc.F + np.bincount(self.edof[a].ravel(), weights=w, minlength=self.ndof)

    def stresses(self, ue, rp):
        """element stresses (nel, 6) at the centre, qp-relaxed."""
        return (ue @ self.CB.T) * (rp ** self.QREL)[:, None]

    def failure(self, s, A):
        """in-layer and across-layer failure indices (nel,) and d(index)/d(sigma) (nel, 6) for each."""
        Axy, Az, As = A
        M = np.array([[1, -.5, -.5, 0, 0, 0], [-.5, 1, -.5, 0, 0, 0], [-.5, -.5, 1, 0, 0, 0],
                      [0, 0, 0, 3, 0, 0], [0, 0, 0, 0, 3, 0], [0, 0, 0, 0, 0, 3.]])
        Ms = s @ M; vm = np.sqrt(np.maximum((Ms * s).sum(1), 1e-30))
        f1 = vm / Axy; d1 = Ms / (vm * Axy)[:, None]
        sn = s[:, self.axis]; e = 0.05 * Az; root = np.sqrt(sn * sn + e * e)
        snp = 0.5 * (sn + root); dsnp = 0.5 * (1 + sn / root)
        tau2 = sum(s[:, k] ** 2 for k in self.layer_shear)
        f2 = np.sqrt(np.maximum((snp / Az) ** 2 + tau2 / As ** 2, 1e-30))
        d2 = np.zeros_like(s)
        d2[:, self.axis] = snp / Az ** 2 * dsnp / f2
        for k in self.layer_shear: d2[:, k] = s[:, k] / As ** 2 / f2
        return f1, d1, f2, d2

    def evaluate(self, rho, beta, grads=True, _ref=False):
        """Volume fraction, constraints, their sensitivities, per-constraint info and the physical density.
        Support groups (cases sharing a support) are processed one at a time: factorise, forward solves, the adjoint
        solves of that group's constraints, then free the factorisation, so peak memory is one factorisation.
        Constraint order (unchanged): stress (cases with a class), compliance safeguard (every case), displacement."""
        pb = self.pb; rp = self.physical(rho, beta); nc = len(pb.cases)
        Eel = self.EMIN + rp ** self.PEN * (1 - self.EMIN)
        dE = self.PEN * rp ** (self.PEN - 1) * (1 - self.EMIN)
        groups = {}
        for i, lc in enumerate(pb.cases): groups.setdefault(lc.support, []).append(i)
        # fixed slots: every constraint depends on exactly one case, so it belongs to that case's support group
        names = [c.name for c in pb.cases]
        s_ci, k = {}, 0
        for i, lc in enumerate(pb.cases):
            if lc.cls is not None: s_ci[i] = k; k += 1
        c0 = k; d0 = c0 + (nc if (self.c_ref is not None and not _ref) else 0)
        ncons = d0 + len(pb.disp)
        cons = np.zeros(ncons); dcons = [None] * ncons
        info = [None] * (nc + (d0 - c0) + len(pb.disp))
        U = np.zeros((self.ndof, nc)); self._comp = np.zeros(nc)
        sm = self.smask; P = self.PNORM
        pn_now = np.zeros(nc); maxf = np.zeros(nc)
        K = self.assemble(Eel)
        Fs = [self.force(i, rp) for i in range(nc)]                     # F + density-weighted body force
        for sup, idx in groups.items():
            sol = self.solve(Eel, {sup: np.stack([Fs[i] for i in idx], 1)}, K=K)[sup]
            try:
                U[:, idx] = sol[0]
                for i in idx: self._comp[i] = Fs[i] @ U[:, i]
                if _ref: continue
                adj_b, adj_meta = [], []                                 # this group's adjoint rhs / (cons, case, expl)
                for i in idx:
                    lc = pb.cases[i]; ue = U[:, i][self.edof]; s = self.stresses(ue, rp)
                    res = dict(case=lc.name, cls=lc.cls)
                    if self.fe[i] is not None:
                        res["body_N"] = float(np.linalg.norm((Fs[i] - lc.F).reshape(-1, 3).sum(0)))
                    if lc.cls is not None:
                        A = allowables(pb.material, lc.cls)
                        f1, d1, f2, d2 = self.failure(s, A)
                        fa = np.concatenate([f1[sm], f2[sm]])
                        pn = (fa ** P).sum() ** (1 / P); pn_now[i] = pn; maxf[i] = fa.max()
                        g = self.cmax[i] * pn / self.TARGET - 1.0
                        res.update(inlayer=f1[sm].max(), across=f2[sm].max(), pnorm=pn, g=g,
                                   where_inlayer=self.ecent[sm][np.argmax(f1[sm])],
                                   where_across=self.ecent[sm][np.argmax(f2[sm])], A=A)
                        cons[s_ci[i]] = g
                        if grads:
                            dpdf = pn ** (1 - P)
                            dS = np.zeros_like(s)
                            dS[sm] = (dpdf * f1[sm] ** (P - 1))[:, None] * d1[sm] + (dpdf * f2[sm] ** (P - 1))[:, None] * d2[sm]
                            dS *= self.cmax[i] / self.TARGET
                            s0 = ue @ self.CB.T
                            expl = self.QREL * rp ** (self.QREL - 1) * (dS * s0).sum(1)
                            due = (dS * (rp ** self.QREL)[:, None]) @ self.CB     # (nel, 24)
                            adj_b.append(np.bincount(self.edof.ravel(), weights=due.ravel(), minlength=self.ndof))
                            adj_meta.append((s_ci[i], i, expl))
                    info[i] = res
                    if self.c_ref is not None:
                        g = self._comp[i] / self.c_ref[i] - 1.0; cons[c0 + i] = g
                        info[nc + i] = dict(case=lc.name, compliance=self._comp[i], c_lim=self.c_ref[i], g=g)
                        if grads:
                            dc = -dE * np.einsum("ij,jk,ik->i", ue, self.KE, ue)
                            if self.fe[i] is not None:                   # c = F(rp).u: + 2 u . dF/drp
                                dc = dc + 2 * ue @ self.fe[i]
                            dcons[c0 + i] = self.chain_T(dc / self.c_ref[i])
                for j, dl in enumerate(pb.disp):
                    i = names.index(dl.case)
                    if i not in idx: continue
                    lu = dl.L @ U[:, i]; g = (lu / dl.limit) ** 2 - 1.0; cons[d0 + j] = g
                    info[nc + (d0 - c0) + j] = dict(case=dl.case, disp=lu, limit=dl.limit, g=g)
                    if grads:
                        adj_b.append(2 * lu / dl.limit ** 2 * dl.L); adj_meta.append((d0 + j, i, np.zeros(self.nel)))
                if grads and adj_b:                                      # adjoints K lam = rhs, same factorisation
                    _, ps, Kf, fd = sol; B = np.stack(adj_b, 1); Lam = np.zeros_like(B)
                    Lam[fd] = ps.solve(Kf, np.ascontiguousarray(B[fd])).reshape(len(fd), -1)
                    for col, (ci, i, expl) in enumerate(adj_meta):
                        ue = U[:, i][self.edof]; le = Lam[:, col][self.edof]
                        d = expl - dE * np.einsum("ij,jk,ik->i", le, self.KE, ue)
                        if self.fe[i] is not None:                       # lam . dF/drp (body force follows rp)
                            d = d + le @ self.fe[i]
                        dcons[ci] = self.chain_T(d)
            finally:
                sol[1].free_memory(everything=True); del sol
        self._U = U
        if _ref:
            return None, None
        self.pn_now, self.maxf = pn_now, maxf
        vol = rp[~pb.void].sum() / self.design.sum() if self.design.any() else 0.0
        if not grads:
            return vol, cons, None, None, info, rp
        dvol = self.chain_T(np.where(pb.void, 0.0, 1.0) / self.design.sum())
        return vol, cons, dvol, np.array(dcons), info, rp

    # ---- optimisation
    def _save_checkpoint(self, path, **state):
        """write the run state to path (.npz) atomically: a crash while saving keeps the previous checkpoint."""
        tmp = str(path) + ".tmp.npz"
        np.savez(tmp, **state)
        os.replace(tmp, str(path))

    def run(self, x0=1.0, iters=160, betas=(1, 2, 4, 8, 16), beta_every=25, move=0.05, log=print, x_init=None,
            checkpoint=None, checkpoint_every=10, resume=None, stall=None):
        """Optimise. Returns the lightest design that met every constraint at the final beta (or the last one, with a
        warning in the log, if none did). Run report() on the result.
        checkpoint: path of a .npz written every checkpoint_every iterations and at the last one (design vector,
          iteration, beta step, MMA state x / xold1 / xold2 / low / upp, p-norm corrections, best design, history);
          None = no checkpoint (nothing else changes).
        resume: path of such a checkpoint: continue from its iteration up to `iters` (the TOTAL count, as in the
          original call; x0 / x_init are ignored). Same problem and settings -> the same result as an unbroken run.
        stall: P0 §5 progress rule: stop when volume and every constraint value are unchanged (volume within 0.5 %,
          constraints within 0.01) over `stall` consecutive checkpoints taken at least beta_every iterations apart
          (a plateau inside one beta step is not a stall); then return as at the end. Needs checkpoint; self.stopped
          says why (None = ran to iters). None = off.
        Under tools\\autorun.py S7 the defaults come from the environment when the caller passes None:
          NOX_PRINTOPT_CKPT (checkpoint path), NOX_PRINTOPT_CKPT_EVERY, NOX_PRINTOPT_STALL, and NOX_PRINTOPT_RESUME
          (resume from that file; first run() call of the process only, and only if the file exists)."""
        import mmapy
        global _ENV_RESUME_USED
        env = os.environ
        if checkpoint is None and env.get("NOX_PRINTOPT_CKPT"):
            checkpoint = env["NOX_PRINTOPT_CKPT"]
            checkpoint_every = int(env.get("NOX_PRINTOPT_CKPT_EVERY") or checkpoint_every)
            if resume is None and env.get("NOX_PRINTOPT_RESUME") and not _ENV_RESUME_USED \
                    and os.path.exists(env["NOX_PRINTOPT_RESUME"]):
                resume = env["NOX_PRINTOPT_RESUME"]
            _ENV_RESUME_USED = True
        if stall is None and env.get("NOX_PRINTOPT_STALL"):
            stall = int(env["NOX_PRINTOPT_STALL"])
        if stall and checkpoint is None:
            raise ValueError("stall needs checkpoint (the progress rule compares checkpoints)")
        self.stopped = None; stall_hist = []                  # (it, vol, g) at the checkpoints the rule compares
        pb = self.pb; dz = self.design
        rho = np.where(pb.solid, 1.0, np.where(pb.void, 0.0, x0)) if x_init is None else x_init.copy()
        n = int(dz.sum())
        m = len(pb.cases) - sum(c.cls is None for c in pb.cases) + len(pb.disp) + (len(pb.cases) if self.c_ref is not None else 0)
        x = rho[dz][:, None]; xold1 = x.copy(); xold2 = x.copy(); low = np.zeros((n, 1)); upp = np.ones((n, 1))
        a0, a, cc, d = 1.0, np.zeros((m, 1)), 1e4 * np.ones((m, 1)), np.zeros((m, 1))
        sym = pb.sym or (lambda v: v)
        beta_i = 0; t_start = time.time(); best = None; it0 = 0
        if resume is not None:
            with np.load(resume) as z:                  # read and close: the same path may be checkpointed again
                ck = {k: z[k] for k in z.files}
            if int(ck["nel"]) != self.nel or int(ck["n"]) != n or int(ck["m"]) != m or \
                    not np.array_equal(ck["design"], dz):
                raise ValueError(f"checkpoint {resume} is for a different problem (nel/design/constraints differ)")
            if list(ck["betas"]) != list(betas) or int(ck["beta_every"]) != beta_every or float(ck["move"]) != move:
                log(f"WARNING: resume {resume}: betas / beta_every / move differ from the saved run (saved "
                    f"{list(ck['betas'])} / {int(ck['beta_every'])} / {float(ck['move'])}); the result will differ")
            rho = ck["rho"].copy(); x = ck["x"].copy(); xold1 = ck["xold1"].copy(); xold2 = ck["xold2"].copy()
            low = ck["low"].copy(); upp = ck["upp"].copy(); beta_i = int(ck["beta_i"]); it0 = int(ck["it"])
            self.cmax = ck["cmax"].copy()
            best = (float(ck["best_vol"]), ck["best_rho"].copy()) if bool(ck["has_best"]) else None
            self.hist = [tuple(h) for h in ck["hist"]]
            if "stall_its" in ck:
                stall_hist = [(int(i), float(v), np.asarray(gg, float)) for i, v, gg in
                              zip(ck["stall_its"], ck["stall_vol"], ck["stall_g"])]
            if str(ck.get("stopped", "")):
                self.stopped = str(ck["stopped"]); it0 = max(it0, iters)       # it had already stopped: no more its
            log(f"resumed from {resume} at it {it0} (beta {betas[beta_i]})")
        if n == 0: log("no free design domain: nothing to optimise")
        for it in range(it0 + 1, (iters if n else 0) + 1):
            t0 = time.time()
            if it > 1 and (it - 1) % beta_every == 0 and beta_i < len(betas) - 1:
                beta_i += 1; low = np.zeros((n, 1)); upp = np.ones((n, 1))      # restart asymptotes after a beta step
            beta = betas[beta_i]
            mv = move if beta_i < len(betas) - 1 else 0.4 * move          # settle at the final beta
            vol, g, dvol, dg, info, rp = self.evaluate(rho, beta)
            # adaptive p-norm correction (Le et al. 2010): c = max / pnorm, damped
            k = 0
            for i, lc in enumerate(pb.cases):
                if lc.cls is None: continue
                ratio = self.maxf[i] / self.pn_now[i]
                self.cmax[i] = ratio if it == 1 else 0.5 * ratio + 0.5 * self.cmax[i]
                k += 1
            dvol = sym(dvol)[dz][:, None]; dg = np.stack([sym(r)[dz] for r in dg]) if m else np.zeros((0, n))
            # mmapy limits each step to move * (xmax - xmin), so the bounds must be the true 0..1 box
            xmin = np.zeros((n, 1)); xmax = np.ones((n, 1))
            xnew, *_, low, upp = mmapy.mmasub(m, n, it, x, xmin, xmax, xold1, xold2, vol, dvol, g[:, None], dg,
                                              low, upp, a0, a, cc, d, move=mv, asyinit=0.2)
            xold2, xold1 = xold1, x; x = np.clip(xnew, 0, 1)
            rho = rho.copy(); rho[dz] = x[:, 0]; rho = sym(rho); x = rho[dz][:, None]
            worst = max((r for r in info if "g" in r), key=lambda r: r["g"]) if m else None
            self.hist.append((it, beta, vol, g.max() if m else 0))
            log(f"it {it:3d} beta {beta:2d} vol {vol:.4f} max g {g.max() if m else 0:+.3f} ({worst['case'] if worst else ''})"
                f" change {np.abs(x - xold1).max():.3f} {time.time()-t0:4.1f}s")
            if beta == betas[-1] and m and g.max() <= 0.01 and (best is None or vol < best[0]):
                best = (vol, rho.copy())
            if checkpoint is not None and (it % checkpoint_every == 0 or it == iters):
                if stall and (not stall_hist or it - stall_hist[-1][0] >= beta_every):
                    stall_hist.append((it, float(vol), np.asarray(g, float).copy()))
                    last = stall_hist[-stall:]
                    v0, g0 = last[-1][1], last[-1][2]
                    if len(last) >= stall and all(abs(v - v0) <= 0.005 * max(abs(v0), 1e-12) and
                                                  (np.abs(gg - g0).max() <= 0.01 if gg.size else True)
                                                  for _, v, gg in last):
                        self.stopped = (f"no progress over checkpoints at it {', '.join(str(i) for i, *_ in last)} "
                                        f"(vol {v0:.4f}, max g {g0.max() if g0.size else 0:+.3f})")
                self._save_checkpoint(checkpoint, rho=rho, x=x, xold1=xold1, xold2=xold2, low=low, upp=upp,
                                      it=it, beta_i=beta_i, beta=beta, cmax=self.cmax, has_best=best is not None,
                                      best_vol=best[0] if best else np.nan,
                                      best_rho=best[1] if best else np.zeros(0),
                                      hist=np.array(self.hist, float).reshape(-1, 4), nel=self.nel, n=n, m=m,
                                      design=dz, betas=np.array(betas, float), beta_every=beta_every, move=move,
                                      iters=iters, stopped=self.stopped or "", vol=vol, gmax=g.max() if m else 0.0,
                                      stall_its=np.array([s[0] for s in stall_hist], int),
                                      stall_vol=np.array([s[1] for s in stall_hist], float),
                                      stall_g=np.array([s[2] for s in stall_hist], float).reshape(len(stall_hist), m))
                if self.stopped:
                    log(f"stopped at it {it}: {self.stopped} (P0 §5 progress rule)")
                    break
        note = ""
        if best is None:
            # "all preserve": the free domain is empty or smaller than one filter ball (cannot hold a single feature),
            # so the result is the preserve set. Say so instead of "no feasible design", but only if that design
            # meets every constraint (re-evaluated at the final beta); otherwise the infeasibility warning stays.
            if dz.sum() < (self.ker > 0).sum():
                _, g_end, *_ = self.evaluate(rho, betas[-1], grads=False)
                gmax = g_end.max() if m else 0.0
                note = ("  design is all preserve (no free domain)" if gmax <= 0.01 else
                        f"  WARNING: no feasible design: design is all preserve (no free domain) and violates its"
                        f" constraints (max g {gmax:+.3f})")
            else:
                note = "  WARNING: no feasible design at the final beta"
        log(f"done {time.time()-t_start:.0f}s" + note)
        self.rho = rho
        return rho if best is None else best[1]

    def report(self, rho, beta=16, title=""):
        """printable summary of a design (any density, e.g. an existing part voxelised): per case in-layer and
        across-layer failure index (1.0 = at the allowable), where, and unsupported passive volume."""
        vol, g, _, _, info, rp = self.evaluate(rho, beta, grads=False)
        h3 = self.pb.h ** 3; out = [f"{title}  build {self.pb.build}, {self.pb.material.name}, overhang filter "
                                    f"{'on' if self.am else 'off'}; solid {(rp > 0.5).sum()*h3/1000:.2f} cm3"]
        for r in info:
            if "inlayer" in r:
                A = r["A"]; w1, w2 = r["where_inlayer"], r["where_across"]
                out.append(f"  {r['case']:28s} [{r['cls']:9s} A {A[0]:.1f}/{A[1]:.1f}/{A[2]:.1f}] in-layer {r['inlayer']:5.2f} "
                           f"at ({w1[0]:.0f},{w1[1]:.0f},{w1[2]:.0f})  across-layer {r['across']:5.2f} at ({w2[0]:.0f},{w2[1]:.0f},{w2[2]:.0f})"
                           + (f"  body force {r['body_N']:.3g} N" if "body_N" in r else ""))
            elif "compliance" in r:
                out.append(f"  {r['case']:28s} stiffness: compliance {r['compliance']:.3g} = {r['compliance']/r['c_lim']*self.pb.comp_factor:.1f}x the solid domain"
                           f" (limit {self.pb.comp_factor:g}x)")
            elif "disp" in r:
                out.append(f"  {r['case']:28s} displacement {r['disp']:+.3f} mm (limit {r['limit']:.3f})")
        if self.am:
            unsup = self.pb.solid & (self.am.forward(np.where(self.pb.solid, 1.0, (rp > 0.5).astype(float))) < 0.5)
            out.append(f"  passive solid not supported at 45 deg in this orientation: {unsup.sum()*h3:.0f} mm3")
        return "\n".join(out)
