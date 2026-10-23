"""noxopt v0: connected-by-construction layout optimiser for Nox generated parts (6 Oct 2026). Project copy."""
import os
import io
import contextlib
import time
import json
import copy
import numpy as np
from scipy import ndimage as ndi
from scipy.ndimage import correlate
import pymoto as pym
import printopt as PO

if "PYPARDISO_MKL_RT" not in os.environ:                  # cloud box: pypardiso does not find pip's MKL by itself
    import glob
    _mkl = glob.glob("/usr/local/lib/libmkl_rt.so*") + glob.glob(os.path.join(os.path.dirname(os.__file__), "..", "libmkl_rt.so*"))
    if _mkl: os.environ["PYPARDISO_MKL_RT"] = _mkl[0]

HEAVI = "(tanh(inp1*{0}) + tanh(inp1*(inp0-{0}))) / (tanh(inp1*{0}) + tanh(inp1*(1-{0})))"
S26 = np.ones((3, 3, 3))


# --------------------------------------------------------------------------------------------------- own modules
class MaskedFilter(pym.Module):
    """Cone density filter that treats passive void as outside the domain: y = W(x m) / W(m), m = not void."""

    def __init__(self, shape, R, void, masked=True):
        self.shape = tuple(shape)
        k = int(np.ceil(R)); ii = np.arange(-k, k + 1)
        I, J, K = np.meshgrid(ii, ii, ii, indexing="ij")
        self.ker = np.maximum(0.0, R - np.sqrt(I * I + J * J + K * K))
        # masked=False (16 Oct 2026, option, default unchanged): plain cone filter; material flush against a keep-out
        # or the grid edge is eroded, so a member must sit inside the design band at full width (no 1-cell sheets)
        self.m = (~void).astype(float).reshape(self.shape) if masked else np.ones(self.shape)
        self.Hs = np.maximum(correlate(np.ones(self.shape), self.ker, mode="constant"), 1e-12) if not masked else \
            np.maximum(correlate(self.m, self.ker, mode="constant"), 1e-12)

    def __call__(self, x):
        return (correlate(x.reshape(self.shape) * self.m, self.ker, mode="constant") / self.Hs).ravel()

    def _sensitivity(self, dy):
        return (correlate(dy.reshape(self.shape) / self.Hs, self.ker, mode="constant") * self.m).ravel()


class Compliance(pym.Module):
    """compliance of every load case, c_i = F_i(x) . u_i, on printopt's FE (one factorisation per support group)."""

    def __init__(self, po):
        self.po = po
        self.groups = {}
        for i, lc in enumerate(po.pb.cases):
            self.groups.setdefault(lc.support, []).append(i)

    def __call__(self, x):
        return self.calc(x)

    def calc(self, x):
        po = self.po; nc = len(po.pb.cases)
        self.x = x
        Eel = po.EMIN + x ** po.PEN * (1 - po.EMIN)
        K = po.assemble(Eel)
        Fs = [po.force(i, x) for i in range(nc)]
        c = np.zeros(nc); self.U = [None] * nc
        for sup, idx in self.groups.items():
            sol = po.solve(Eel, {sup: np.stack([Fs[i] for i in idx], 1)}, K=K)[sup]
            try:
                for col, i in enumerate(idx):
                    self.U[i] = sol[0][:, col].copy(); c[i] = Fs[i] @ self.U[i]
            finally:
                sol[1].free_memory(everything=True); del sol
        return c

    def _sensitivity(self, dc):
        po = self.po; x = self.x; a = po.act
        dE = po.PEN * x[a] ** (po.PEN - 1) * (1 - po.EMIN)
        out = np.zeros(po.nel); acc = np.zeros(a.sum()); edof = po.edof[a]
        for i, w in enumerate(dc):
            if w == 0.0: continue
            ue = self.U[i][edof]
            d = -dE * np.einsum("ij,jk,ik->i", ue, po.KE, ue)
            if po.fe[i] is not None:
                d = d + 2 * ue @ po.fe[i]
            acc += w * d
        out[a] = acc
        return out


class PMean(pym.Module):
    """weighted p-mean of c / c_ref: every case counts, the worst ones dominate."""

    def __init__(self, c_ref, w, P=6.0, scale=1.0):
        self.c_ref, self.w, self.P, self.scale = c_ref, w, P, scale

    def __call__(self, c):
        self.r = self.w * c / self.c_ref
        self.pm = (np.sum(self.r ** self.P) / len(self.r)) ** (1 / self.P)
        return self.scale * self.pm

    def _sensitivity(self, d):
        return d * self.scale * self.pm ** (1 - self.P) * self.r ** (self.P - 1) / len(self.r) * self.w / self.c_ref


class Symmetrise(pym.Module):
    def __init__(self, sym): self.sym = sym
    def __call__(self, x): return self.sym(x)
    def _sensitivity(self, d): return self.sym(d)


class VolFrac(pym.Module):
    def __init__(self, design, limit):
        self.design, self.limit, self.n = design, limit, int(design.sum())

    def __call__(self, x):
        self.v = float(x[self.design].sum() / self.n)
        return self.v / self.limit - 1.0

    def _sensitivity(self, d):
        return d * self.design / (self.n * self.limit)


# ------------------------------------------------------------------------------------------------------ helpers
def interface_bodies(pb, min_cells=4):
    lab, n = ndi.label(pb.solid.reshape(pb.shape), S26)
    out = [(k, np.where(lab.ravel() == k)[0]) for k in range(1, n + 1)]
    out.sort(key=lambda t: -len(t[1]))
    return [t for t in out if len(t[1]) >= min_cells], [t for t in out if len(t[1]) < min_cells]


def add_tie_cases(pb, min_cells=4, log=print):
    po0 = PO.PrintOpt(_no_comp(pb))
    count = {}
    for lc in pb.cases: count[lc.support] = count.get(lc.support, 0) + 1
    main = max(count, key=count.get)
    fixed_nodes = np.zeros(po0.nn, bool); fixed_nodes[np.unique(pb.supports[main] // 3)] = True
    bodies, small = interface_bodies(pb, min_cells)
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


def _no_comp(pb):
    p = copy.copy(pb); p.comp_factor = None; return p


def connectivity(dens, pb, thr=0.5):
    A = (np.where(pb.solid, 1.0, np.where(pb.void, 0.0, dens)) > thr).reshape(pb.shape)
    lab, n = ndi.label(A)
    L = lab.ravel()
    parent = np.arange(n + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    bodies, _ = interface_bodies(pb, 1)
    for k, el in bodies:
        ls = np.unique(L[el]); r = find(ls[0])
        for q in ls[1:]: parent[find(q)] = r
    root = np.array([find(a) for a in range(n + 1)]); G = root[L]
    sz = np.bincount(G, minlength=n + 1); sz[0] = 0
    main = int(np.argmax(sz)); groups = [g for g in np.unique(G) if g != 0]
    ijk = np.stack(np.unravel_index(np.arange(L.size), pb.shape), 1)
    cen = lambda el: (ijk[el].mean(0) * pb.h + np.asarray(pb.origin) + pb.h / 2).round(1).tolist()
    det = [dict(cells=int(len(el)), at=cen(el)) for k, el in bodies if G[el[0]] != main]
    loose = [dict(cells=int(sz[g]), at=cen(np.where(G == g)[0])) for g in groups
             if g != main and not pb.solid[G == g].any()]
    return dict(ok=(len(groups) == 1), bodies=len(groups), main_share=float(sz[main] / sz.sum()),
                detached_interfaces=det, loose_generated=loose)


def auto_symmetry(pb, near_tol=0.08):
    shape = tuple(pb.shape); so = pb.solid.reshape(shape); vo = pb.void.reshape(shape)
    axes, near = [], []
    n_pass = max(int(so.sum() + vo.sum()), 1)
    for a in range(3):
        miss = (np.logical_xor(so, np.flip(so, a)).sum() + np.logical_xor(vo, np.flip(vo, a)).sum()) / (2.0 * n_pass)
        if miss == 0: axes.append(a)
        elif miss < near_tol: near.append((a, float(miss)))
    if not axes:
        return None, [], near

    def sym(v):
        x = np.asarray(v, float).reshape(shape)
        for a in axes: x = 0.5 * (x + np.flip(x, a))
        return x.ravel()
    return sym, axes, near


def greyness(x, design):
    v = x[design]
    return float(((v > 0.1) & (v < 0.9)).mean())


# ---------------------------------------------------------------------------------------------------------- run
class Layout:
    BETAS = (1, 2, 4, 8, 16, 32)

    def __init__(self, pb, volfrac, R=None, eta=(0.7, 0.5, 0.3), P=4.0, tie_weight=0.25, tie_min_cells=4, log=print,
                 auto_sym=True, masked=True):
        if auto_sym and getattr(pb, "sym", None) is None:
            fn_sym, axes, near = auto_symmetry(pb)
            if axes:
                pb = copy.copy(pb); pb.sym = fn_sym
                log(f"noxopt: drawn features and keep-outs mirror about the mid-plane of {', '.join('xyz'[a] for a in axes)}: "
                    f"generated material held symmetric there (pass auto_sym=False for a one-sided design)")
            for a, miss in near:
                log(f"noxopt: WARNING drawn features are nearly but not exactly mirrored in {'xyz'[a]} ({miss:.1%} of the "
                    f"passive cells differ): no symmetry imposed there.")
        self.log = log; self.pb0 = pb; self.volfrac = volfrac
        self.pb, self.n_real, self.tie_info = add_tie_cases(pb, tie_min_cells, log)
        pb = self.pb
        self.po = PO.PrintOpt(_no_comp(pb))
        po = self.po
        self.design = po.design; self.nel = po.nel
        R = pb.R if R is None else R; self.R = R
        nx, ny, nz = pb.shape
        self.dom = pym.VoxelDomain(nz, ny, nx)
        solid = np.where(pb.solid)[0]; void = np.where(pb.void)[0]
        ax, sgn = PO.AXES[pb.build]
        direction = [0.0, 0.0, 0.0]; direction[2 - ax] = float(sgn)
        comp = Compliance(po)
        self.c_ref = comp.calc(np.where(pb.void, 0.0, 1.0)).copy()
        bad = [lc.name for lc, c in zip(pb.cases, self.c_ref) if not c > 1e-12 * self.c_ref.max()]
        if bad:
            raise ValueError(f"noxopt: zero compliance in the solid domain for {bad}: the load sits on supported dofs")
        w = np.ones(len(pb.cases)); w[self.n_real:] = tie_weight
        self.sx = pym.Signal("x", state=np.full(self.nel, volfrac))
        self.sbeta = pym.Signal("beta", state=1.0)
        self.eta = eta
        with pym.Network() as fn:
            sx_ = Symmetrise(pb.sym)(self.sx) if getattr(pb, "sym", None) is not None else self.sx
            s0 = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(sx_))
            sf = MaskedFilter(pb.shape, R, pb.void, masked=masked)(s0)
            self.masked = masked
            fields = []
            for e in eta:
                s = pym.MathExpression(HEAVI.format(e))(sf, self.sbeta)
                s = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(s))
                if pb.overhang:
                    with contextlib.redirect_stdout(io.StringIO()):
                        s = pym.OverhangFilter(self.dom, direction=direction)(s)
                    s = pym.SetValue(void, 0.0)(pym.SetValue(solid, 1.0)(s))
                fields.append(s)
            self.s_ero, self.s_nom, self.s_dil = fields
            self.comp = comp
            self.s_c = self.comp(self.s_ero)
            self.pmean = PMean(self.c_ref, w, P)
            self.s_obj = self.pmean(self.s_c)
            self.vol = VolFrac(self.design, volfrac)
            self.s_vol = self.vol(self.s_dil)
        self.fn = fn
        self.hist = []

    def _save(self, path, mma, it, bi, since, extra):
        tmp = path + ".tmp.npz"
        np.savez_compressed(tmp, x=self.sx.state, xold1=mma.xold1 if mma.xold1 is not None else np.zeros(0),
                            xold2=mma.xold2 if mma.xold2 is not None else np.zeros(0), offset=mma.offset,
                            it=it, bi=bi, since=since, limit=self.vol.limit, scale=self.pmean.scale,
                            hist=np.array(self.hist, float), extra=json.dumps(extra))
        os.replace(tmp, path)

    def run(self, maxit=220, min_stage=12, max_stage=30, grey_stop=0.02, move=0.1, checkpoint=None, resume=None,
            checkpoint_every=10, time_limit=None, beta0=0, vol_gain=1.0):
        # vol_gain (16 Oct 2026, option, default unchanged): share of the every-5-iteration dilated-volume limit
        # correction applied (in log terms). 1.0 snaps the limit onto volfrac * dil / nom at once; at low volfrac and
        # high beta that over-corrects and limit-cycles (receiver v2: nominal volume 2 <-> 4.5 %, load paths cut every
        # cycle). 0.3 damps it.
        log = self.log; t0 = time.time()
        mma = pym.MMA([self.sx], [self.s_obj, self.s_vol], function=self.fn, move=move, verbosity=0,
                      xmin=0.0, xmax=1.0)
        it = 0; bi = int(beta0); since = 0
        if resume and os.path.exists(resume):
            ck = np.load(resume, allow_pickle=True)
            self.sx.state = ck["x"].copy(); it = int(ck["it"]); bi = int(ck["bi"]); since = int(ck["since"])
            mma.xold1 = ck["xold1"].copy() if ck["xold1"].size else None
            mma.xold2 = ck["xold2"].copy() if ck["xold2"].size else None
            mma.offset = ck["offset"].copy(); mma.iter = it
            self.vol.limit = float(ck["limit"]); self.pmean.scale = float(ck["scale"])
            self.hist = [tuple(r) for r in ck["hist"]]
            log(f"resumed from {resume} at it {it} (beta {self.BETAS[bi]})")
        self.stopped = "maxit"
        while it < maxit:
            it += 1; since += 1
            self.sbeta.state = float(self.BETAS[bi])
            x = mma.x
            if it == 1:
                self.fn.response(); self.pmean.scale = 10.0 / float(self.s_obj.state)
            mma.response_is_uptodate = False
            xnew, g, dg = mma.step(x=x)
            vn = float(self.s_nom.state[self.design].mean()); vd = self.vol.v
            grey = greyness(self.s_nom.state, self.design)
            ch = float(np.abs(xnew - x)[self.design].max())
            r = self.pmean.r; worst = int(np.argmax(r))
            self.hist.append((it, self.BETAS[bi], float(g[0]), float(g[1]), vn, grey, ch))
            log(f"it {it:3d} beta {self.BETAS[bi]:2d} obj {float(g[0]):8.4f} vol {vn:.4f} (dil {vd:.4f} g {float(g[1]):+.3f}) "
                f"grey {grey:.1%} change {ch:.3f} worst {self.pb.cases[worst].name[:34]} {r[worst]:.3g}x  {time.time()-t0:.0f}s")
            self.sx.state = xnew
            if it % 5 == 0 and vn > 0:
                new = self.volfrac * vd / vn
                if vol_gain != 1.0: new = self.vol.limit * (new / self.vol.limit) ** vol_gain
                self.vol.limit = float(np.clip(new, self.volfrac, 3 * self.volfrac))
            last = bi == len(self.BETAS) - 1
            objs = [h[2] for h in self.hist[-4:]]
            settled = since >= min_stage and len(objs) == 4 and abs(objs[-1] - objs[0]) <= 0.01 * abs(objs[-1]) \
                and abs(float(g[1])) < 0.02
            if not last and (settled or since >= max_stage):
                bi += 1; since = 0; mma.xold1 = mma.xold2 = None; mma.offset[:] = mma.asyinit
            elif last and ((grey <= grey_stop and settled) or since >= max_stage):
                self.stopped = "grey" if grey <= grey_stop else "max_stage"; break
            if checkpoint and it % checkpoint_every == 0:
                self._save(checkpoint, mma, it, bi, since, {})
            if time_limit and time.time() - t0 > time_limit:
                self.stopped = "time"; break
        if checkpoint: self._save(checkpoint, mma, it, bi, since, dict(stopped=self.stopped))
        self.sbeta.state = float(self.BETAS[bi]); self.fn.response()
        log(f"stopped: {self.stopped} at it {it}, beta {self.BETAS[bi]}, grey {greyness(self.s_nom.state, self.design):.1%}")
        return self.s_nom.state.copy()

    def save(self, path, dens):
        pb = self.pb
        np.savez_compressed(path, rho=self.sx.state, rho_phys=dens, solid=pb.solid, void=pb.void,
                            shape=np.array(pb.shape), h=pb.h, origin=np.array(pb.origin))


def strength_report(pb, dens, thr=0.5, title="noxopt"):
    binary = np.where(pb.solid, 1.0, np.where(pb.void, 0.0, (dens > thr).astype(float)))
    pb1 = copy.copy(pb); pb1.overhang = False; pb1.R = 0.5
    po = PO.PrintOpt(pb1)
    vol, g, _, _, info, rp = po.evaluate(binary, 64.0, grads=False)
    rows = [dict(case=r["case"], cls=r["cls"], inlayer=float(r["inlayer"]), across=float(r["across"]))
            for r in info if r and "inlayer" in r]
    return rows, po.report(binary, 64.0, title)


# ------------------------------------------------------------------------------------------- stage 3: smooth build
def _voxelise(mesh, xs, ys, zs, chunk=20000):
    """inside test by ray parity along Z; rays go in chunks (all 425k rays of a 164 x 160 part at 0.25 at once took > 14 GB, P)."""
    XX, YY = np.meshgrid(xs, ys, indexing="ij")
    org = np.stack([XX.ravel(), YY.ravel(), np.full(XX.size, zs[0] - 5.0)], 1)
    A = np.zeros((len(xs) * len(ys), len(zs)), bool)
    for c0 in range(0, len(org), chunk):
        o = org[c0:c0 + chunk]
        loc, ray, _ = mesh.ray.intersects_location(o, np.tile([0, 0, 1.0], (len(o), 1)), multiple_hits=True)
        if not len(ray): continue                                     # a chunk of rays that all miss the part
        order = np.argsort(ray, kind="stable"); ray = ray[order] + c0; z = loc[order][:, 2]
        st = np.r_[0, np.where(np.diff(ray))[0] + 1, len(ray)]
        for a, b in zip(st[:-1], st[1:]):
            zz = np.sort(z[a:b]); zz = zz[np.r_[True, np.diff(zz) > 1e-6]]
            for p, q in zip(zz[0::2], zz[1::2]):
                A[ray[a], (zs > p) & (zs < q)] = True
    return A.reshape(len(xs), len(ys), len(zs))


def _smin(a, b, k):
    t = np.clip(0.5 + 0.5 * (b - a) / k, 0, 1)
    return b * (1 - t) + a * t - k * t * (1 - t)


def _coarse_links(rho, sol, dom, h, s, fine_shape):
    from skimage.morphology import skeletonize
    B = ((rho > 0.5) | sol) & dom
    S = skeletonize(B).astype(bool); idx = np.argwhere(S)
    lut = -np.ones(B.shape, np.int64); lut[tuple(idx.T)] = np.arange(len(idx))
    L = np.zeros(fine_shape, bool)
    to_f = lambda c: (2 * h + c * h) / s
    for dv in np.argwhere(np.ones((3, 3, 3))) - 1:
        if not dv.any() or tuple(dv) < (0, 0, 0): continue
        q = idx + dv; ok = np.all((q >= 0) & (q < B.shape), 1); j = lut[tuple(q[ok].T)]; a = idx[ok][j >= 0]; b = q[ok][j >= 0]
        for t in np.linspace(0, 1, int(np.ceil(1.8 * h / s)) + 1):
            f = np.round(to_f(a + t * (b - a))).astype(int)
            f = f[np.all((f >= 0) & (f < fine_shape), 1)]
            L[tuple(f.T)] = True
    return L


def smooth_surface(rho, solid, void, shape, h, origin, keep_mesh=None, up=4, sigma_mm=0.6, close_edges=True, log=print,
                   reinforce=None, reinforce_r=1.2, clip_mesh=None, clip_margin=1.0, clip_near=None,
                   fair_mm="auto", band_mm="auto", floor_mm=2.0, polish=True, polish_T=None, polish_rho=None,
                   polish_hole=None, polish_soft=None, polish_mesh_iters=10):
    import trimesh
    from skimage import measure
    shape = tuple(int(s) for s in shape); o = np.asarray(origin, float)
    rho = np.asarray(rho, float).reshape(shape); dom = ~np.asarray(void).reshape(shape); sol = np.asarray(solid).reshape(shape)
    design = dom & ~sol
    if close_edges:
        full = (rho > 0.5) | sol
        closed = ndi.binary_closing(full, structure=ndi.generate_binary_structure(3, 2)) & design & ~full
        rho = np.where(closed, 1.0, rho)
        log(f"  edge-joint closing: +{closed.sum() * h ** 3 / 1000:.2f} cm3")
    target = float(rho[design].sum()) * h ** 3
    ext = ndi.grey_dilation(np.where(dom, rho, 0.0), size=(3, 3, 3))
    f = np.where(dom, rho, ext)
    f = np.where(sol, 0.0, f)
    pad = lambda a, v=0.0: np.pad(np.pad(a, 1, mode="edge"), 1, constant_values=v)
    rz = ndi.zoom(pad(f), up, order=1)
    n_c = np.array(shape) + 4
    sp = h * (n_c - 1) / (np.array(rz.shape) - 1)
    ax = [o[k] + h / 2 - 2 * h + np.arange(rz.shape[k]) * sp[k] for k in range(3)]
    cell = float(np.prod(sp))
    zc = lambda a: ndi.zoom(np.pad(a.astype(float), 2), up, order=0) > 0.5
    dzf, solf = zc(design), zc(sol)
    lo, hi = 0.2, 0.8
    for _ in range(30):
        iso = 0.5 * (lo + hi); v = float((rz[dzf] > iso).sum()) * cell
        lo, hi = (iso, hi) if v > target else (lo, iso)
    G = rz > iso
    if keep_mesh is not None:
        K = _voxelise(keep_mesh, *ax)
        near = ndi.distance_transform_edt(~K) * sp[0] <= 0.5 * h
        M = G | K | (solf & near)
    else:
        K = solf; M = G | solf
    if clip_mesh is not None:
        A = _voxelise(clip_mesh, *ax)
        near = ndi.distance_transform_edt(~K) * sp[0] <= (2 * clip_margin + sigma_mm if clip_near is None else clip_near)
        ok = (ndi.distance_transform_edt(A) * sp[0] > clip_margin) | near
        cut = G & ~ok
        G = G & ok; M = M & (ok | K)
        log(f"  clip: generated material held {clip_margin} mm inside the keep-outs: {cut[dzf].sum() * cell / 1000:.2f} cm3 pulled back "
            f"(volume target unchanged)")
    if reinforce is not None and len(reinforce):
        seed = np.zeros(M.shape, bool)
        idx = [np.clip(np.round((np.asarray(reinforce)[:, k] - ax[k][0]) / sp[k]).astype(int), 0, M.shape[k] - 1) for k in range(3)]
        seed[idx[0], idx[1], idx[2]] = True
        ball = (ndi.distance_transform_edt(~seed) * sp[0] <= reinforce_r) & (dzf | solf)
        add = ball & ~M & dzf
        M = M | ball; target += float(add.sum()) * cell
        log(f"  reinforcement: {len(reinforce)} hot cells, balls r {reinforce_r} mm, +{add.sum() * cell:.1f} mm3")
    d = (ndi.distance_transform_edt(~M) - ndi.distance_transform_edt(M)) * sp[0]
    ds = ndi.gaussian_filter(d, sigma=sigma_mm / sp[0])
    if fair_mm == "auto": fair_mm = 1.0 * h if h >= 1.0 else 0.0
    if band_mm == "auto": band_mm = 0.5 * h
    if polish:
        polish_T = 2.0 * h * h if polish_T is None else polish_T
        polish_rho = 2.5 * h if polish_rho is None else polish_rho
        polish_hole = 1.0 * h if polish_hole is None else polish_hole
        polish_soft = 0.25 * h if polish_soft is None else polish_soft
        Kb = K if keep_mesh is not None else solf
        ds = _polish(ndi.gaussian_filter(d, 0.5 / sp[0]), Kb, sp[0], floor_mm,
                     links=_coarse_links(rho, sol, dom, h, sp[0], rz.shape), T_mm2=polish_T, rho_mm=polish_rho,
                     hole_mm=polish_hole, soft_mm=polish_soft, core_src=G, log=log)
        if clip_mesh is not None:
            okp = ndi.distance_transform_edt(A) * sp[0] > clip_margin + 0.25
            dok = (ndi.distance_transform_edt(~okp) - ndi.distance_transform_edt(okp)) * sp[0]
            ds = -_smin(-ds, -ndi.gaussian_filter(dok, 0.35 / sp[0]), max(0.75, polish_soft))
        ds[[0, -1], :, :] = ds[:, [0, -1], :] = ds[:, :, [0, -1]] = 1e3
    lo, hi = -1.5, 1.5
    for _ in range(40):
        c = 0.5 * (lo + hi); v = float((ds[dzf] < c).sum()) * cell
        lo, hi = (c, hi) if v < target else (lo, c)
    c = 0.5 * (lo + hi)
    gen0 = float(G[dzf].sum()) * cell; gen1 = float((ds[dzf] < c).sum()) * cell
    moved = float(((ds < c) ^ M)[dzf].sum()) * cell
    log(f"  smooth surface: up {up} ({sp[0]:.2f} mm), sigma {sigma_mm} mm, iso {iso:.3f}, level {c:+.3f} mm; generated "
        f"{gen0/1000:.2f} -> {gen1/1000:.2f} cm3 (target {target/1000:.2f}); {moved/1000:.2f} cm3 moved from edges into fillets")
    v, fc, _, _ = measure.marching_cubes(ds, c, spacing=tuple(sp))
    if polish:
        Kb = K if keep_mesh is not None else solf
        DK = ndi.distance_transform_edt(~Kb) * sp[0] if Kb.any() else np.full(Kb.shape, 1e3)
        free = ndi.map_coordinates(DK, (v / sp).T, order=1, mode="nearest") > 0.75
        v = _taubin_mesh(v, fc, free, iters=polish_mesh_iters)
    v += np.array([ax[0][0], ax[1][0], ax[2][0]]) + np.array([0.0123, 0.0087, 0.0071])
    t = trimesh.Trimesh(v, fc, process=True)
    if t.volume < 0: t.invert()
    return t


def wrinkle(mesh, pitch=0.4, span_mm=2.0, skip=None, drawn=None, allowed=None, skip_mm=1.5):
    if drawn is not None or allowed is not None:
        hh = 0.5; ax2 = [np.arange(mesh.bounds[0][k] - 3.0, mesh.bounds[1][k] + 3.0, hh) for k in range(3)]
        o2 = np.array([a[0] for a in ax2]); dmin = np.full(tuple(len(a) for a in ax2), np.inf, np.float32)
        if drawn is not None:
            dmin = np.minimum(dmin, ndi.distance_transform_edt(~_voxelise(drawn, *ax2)).astype(np.float32) * hh)
        if allowed is not None:
            dmin = np.minimum(dmin, ndi.distance_transform_edt(_voxelise(allowed, *ax2)).astype(np.float32) * hh)
        skip0 = skip
        skip = lambda p: (ndi.map_coordinates(dmin, ((p - o2) / hh).T, order=1, mode="nearest") < skip_mm) | \
            (skip0(p) if skip0 is not None else False)
    ax = [np.arange(mesh.bounds[0][k] - 4 * pitch + pitch / 2, mesh.bounds[1][k] + 4 * pitch, pitch) for k in range(3)]
    M = _voxelise(mesh, *ax); org = np.array([a[0] for a in ax])
    d = ((ndi.distance_transform_edt(~M) - ndi.distance_transform_edt(M)) * pitch).astype(np.float32)
    d = ndi.gaussian_filter(d, 0.35 / pitch)
    g = np.gradient(d, pitch); n = np.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2) + 1e-9
    H = 0.5 * sum(np.gradient(g[k] / n, pitch, axis=k) for k in range(3)); del g, n
    band = (np.abs(d) < 0.5 * pitch).astype(np.float32); s = span_mm / pitch
    Hl = ndi.gaussian_filter(H * band, s) / (ndi.gaussian_filter(band, s) + 1e-9)
    w = np.abs(H - Hl)[band > 0]
    if skip is not None:
        w = w[~skip(np.argwhere(band > 0) * pitch + org)]
    return float(np.percentile(w, 95)), float(np.percentile(w, 99))


def _redistance(phi, s, band_mm):
    try:
        import skfmm
        d = skfmm.distance(phi, dx=[float(s)] * 3, narrow=float(band_mm))
        if np.ma.isMaskedArray(d):
            d = np.where(d.mask, np.sign(phi) * band_mm, d.filled(0.0))
        return d.astype(np.float32)
    except ImportError:
        X = phi < 0
        return ((ndi.distance_transform_edt(~X) - ndi.distance_transform_edt(X)) * s).astype(np.float32)


def _polish(phi0, drawn, s, floor_mm=2.0, links=None, T_mm2=4.0, rho_mm=4.0, hole_mm=2.0, spur_mm=6.0, soft_mm=0.0,
            core_src=None, log=print):
    from skimage.morphology import skeletonize
    phi0 = phi0.astype(np.float32)
    sdf = lambda X: ((ndi.distance_transform_edt(~X) - ndi.distance_transform_edt(X)) * s).astype(np.float32)
    phiK = ndi.gaussian_filter(sdf(drawn), 0.35 / s) if drawn is not None and drawn.any() else None
    gen = (phi0 < 0) & (~drawn if drawn is not None else True)
    if core_src is not None: gen = gen & core_src
    sk = skeletonize(gen).astype(bool)
    if links is not None: sk |= links
    sk = _prune_skeleton(sk, drawn, s, spur_mm)
    phic = ndi.gaussian_filter(sdf(ndi.distance_transform_edt(~sk) * s <= floor_mm / 2), 0.35 / s) + 0.1 \
        if sk.any() else None
    lo_hole = phi0 - hole_mm
    phi = _redistance(phi0, s, 6 * s + hole_mm)
    dt = np.float32(0.15 * s * s); n = int(np.ceil(T_mm2 / dt)); sig = rho_mm / s
    kb = None
    for k in range(n):
        lap = np.clip(ndi.laplace(phi) / np.float32(s * s), -1.0 / s, 1.0 / s)
        if k % 5 == 0:
            w = np.clip(1.0 - np.abs(phi) / s, 0.0, 1.0).astype(np.float32)
            kb = _local_mean(lap * w, w, sig)
        nb = np.abs(phi) < 3 * s
        phi = np.where(nb, phi + dt * (lap - kb), phi)
        if phiK is not None: phi = np.minimum(phi, phiK)
        if phic is not None: phi = _smin(phi, phic, soft_mm) if soft_mm else np.minimum(phi, phic)
        phi = -_smin(-phi, -lo_hole, soft_mm) if soft_mm else np.maximum(phi, lo_hole)
        if k % 5 == 4 and k < n - 1:
            phi = _redistance(phi, s, 6 * s + hole_mm)
    moved = np.abs(phi - phi0)[np.abs(phi0) < s]
    log(f"  polish: flow {T_mm2} mm2 ({n} steps of {dt:.3f}), curvature averaged over {rho_mm} mm, floor {floor_mm} mm, "
        f"holes kept {hole_mm} mm; surface moved p50 {np.median(moved):.2f} p99 {np.percentile(moved, 99):.2f} mm")
    return phi


def _prune_skeleton(S, keep, s, spur_mm=6.0):
    S = S.copy(); ker = np.ones((3, 3, 3), np.float32)
    near = ndi.binary_dilation(keep, iterations=2) if keep is not None and keep.any() else np.zeros_like(S)
    for _ in range(int(np.ceil(spur_mm / s))):
        cnt = ndi.convolve(S.astype(np.float32), ker, mode="constant") - 1
        ends = S & (cnt <= 1.5) & ~near
        if not ends.any(): break
        S &= ~ends
    return S


def _local_mean(a, w, sig, f=4):
    shp = a.shape; pad = [(0, (-n) % f) for n in shp]
    A = np.pad(a, pad); Wt = np.pad(w, pad)
    cs = tuple(n // f for n in A.shape)
    A = A.reshape(cs[0], f, cs[1], f, cs[2], f).sum((1, 3, 5)); Wt = Wt.reshape(cs[0], f, cs[1], f, cs[2], f).sum((1, 3, 5))
    m = ndi.gaussian_filter(A, sig / f) / (ndi.gaussian_filter(Wt, sig / f) + 1e-6)
    up = ndi.zoom(m, f, order=1)[:shp[0], :shp[1], :shp[2]]
    if up.shape != shp:
        up = np.pad(up, [(0, shp[k] - up.shape[k]) for k in range(3)], mode="edge")
    return up.astype(np.float32)


def _taubin_mesh(v, f, free, iters=10, lam=0.5, mu=-0.53):
    import scipy.sparse as sps
    n = len(v); e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    A = sps.coo_matrix((np.ones(2 * len(e)), (np.r_[e[:, 0], e[:, 1]], np.r_[e[:, 1], e[:, 0]])), shape=(n, n)).tocsr()
    A.data[:] = 1.0
    W = sps.diags(1.0 / np.maximum(np.asarray(A.sum(1)).ravel(), 1)) @ A
    m = free.astype(float)[:, None]; x = v.astype(float).copy()
    for _ in range(iters):
        x = x + lam * m * (W @ x - x); x = x + mu * m * (W @ x - x)
    return x


def function_check(part, drawn, cuts=None, allowed=None, log=print):
    import manifold3d as mf
    B = mf.Manifold(mf.Mesh(vert_properties=np.asarray(part.vertices, np.float32), tri_verts=np.asarray(part.faces, np.uint32)))
    fin = drawn - cuts if cuts is not None else drawn
    missing = (fin - B).volume()
    in_cuts = (B ^ cuts).volume() if cuts is not None else 0.0
    in_keep = (B - (allowed + drawn)).volume() if allowed is not None else 0.0
    log(f"function check: drawn missing {missing:.2f} mm3 | material in cuts {in_cuts:.2f} mm3 | in keep-outs {in_keep:.2f} mm3"
        + ("  OK" if max(missing, in_cuts, in_keep) < 0.5 else "  FAIL"))
    return missing, in_cuts, in_keep


# ------------------------------------------------------------------------------------------ thin members (12 Oct 2026)
def thin_members(dens, pb, floor_mm=2.0, margin_mm=0.75, up=3, min_len_mm=None, thr=0.5):
    from skimage.morphology import skeletonize
    shape = tuple(pb.shape); h = pb.h; s = h / up
    rep = lambda a: a.reshape(shape).repeat(up, 0).repeat(up, 1).repeat(up, 2)
    sol = rep(pb.solid); vd = rep(pb.void)
    B = (rep(np.asarray(dens) > thr) | sol) & ~vd
    if vd.any() and margin_mm > 0:
        B &= (ndi.distance_transform_edt(~vd) * s > margin_mm) | sol
    E = ndi.distance_transform_edt(np.pad(B, 1))[1:-1, 1:-1, 1:-1] * s
    sk = skeletonize(B).astype(bool)
    far = ndi.distance_transform_edt(~sol) * s > floor_mm if sol.any() else np.ones(B.shape, bool)
    thin = sk & (E < 0.45 * floor_mm) & far
    lab, n = ndi.label(thin, S26)
    min_len = 2.0 * floor_mm if min_len_mm is None else min_len_mm
    keep = np.zeros(B.shape, bool); info = []
    for k in range(1, n + 1):
        m = lab == k; idx = np.argwhere(m)
        length = float(np.linalg.norm(idx.max(0) - idx.min(0)) * s)
        if length < min_len: continue
        keep |= m
        info.append(dict(len_mm=round(length, 1), radius_mm=round(float(np.median(E[m])), 2),
                         at=((idx.mean(0) + 0.5) * s + np.asarray(pb.origin)).round(1).tolist()))
    if not keep.any():
        return np.zeros(int(np.prod(shape)), bool), info
    near = (ndi.distance_transform_edt(~keep) * s <= 0.5 * floor_mm + s) & B & ~sol
    cells = near.reshape(shape[0], up, shape[1], up, shape[2], up).any((1, 3, 5)).ravel()
    if getattr(pb, "sym", None) is not None:
        cells = pb.sym(cells.astype(float)) > 0
    return cells & ~pb.solid & ~pb.void, info


def prune_thin(L, dens, floor_mm=2.0, margin_mm=0.75, rounds=3, checkpoint=None, log=print, built=None, **run_kw):
    removed = []
    for r in range(rounds):
        cells, info = thin_members(dens, L.pb0, floor_mm, margin_mm)
        if r == 0 and built:
            pb = L.pb0; ijk = np.stack(np.unravel_index(np.arange(cells.size), pb.shape), 1)
            cen = ijk * pb.h + np.asarray(pb.origin) + pb.h / 2
            for b in built:
                cells |= (np.linalg.norm(cen - np.asarray(b["at"]), axis=1) <= b["len_mm"] / 2 + 0.5 * floor_mm) \
                    & (np.asarray(dens) > 0.5) & ~pb.solid & ~pb.void
            if getattr(pb, "sym", None) is not None: cells = pb.sym(cells.astype(float)) > 0
            info = info + [dict(b, built=True) for b in built]
        if not info: break
        log(f"prune {r + 1}: {len(info)} thin member(s) under {floor_mm} mm as built, {int(cells.sum())} cells removed: {info}")
        removed += info
        pb = copy.copy(L.pb0); pb.void = L.pb0.void | cells
        x = L.sx.state.copy(); x[cells] = 0.0
        # 16 Oct 2026: carry the filter choice into the re-settle (a masked=False run was pruned with the masked filter)
        L2 = Layout(pb, L.volfrac, R=L.R, eta=L.eta, log=log, masked=getattr(L, "masked", True)); L2.sx.state = x
        dens = L2.run(beta0=3, checkpoint=checkpoint, **run_kw); L = L2
    else:
        cells, info = thin_members(dens, L.pb0, floor_mm, margin_mm)
        if info: log(f"prune: {len(info)} thin member(s) left after {rounds} passes: {info}")
    return L, dens, removed


THIN_OFFSETS = ((0.5, 0.5, 0.5), (0.25, 0.5, 0.75))   # 17 Oct 2026: shifted sample grids that must confirm a member


def _thin_on_grid(part, drawn, floor_mm, pitch, min_len, off=(0.0, 0.0, 0.0)):
    """thin members of a built part sampled on a pitch grid shifted by `off` (fractions of the pitch)."""
    from skimage.morphology import skeletonize
    ax = [np.arange(part.bounds[0][k] - pitch / 2 + off[k] * pitch, part.bounds[1][k] + pitch, pitch) for k in range(3)]
    M = _voxelise(part, *ax); E = ndi.distance_transform_edt(np.pad(M, 1))[1:-1, 1:-1, 1:-1] * pitch
    far = np.ones(M.shape, bool)
    if drawn is not None:
        far = ndi.distance_transform_edt(~_voxelise(drawn, *ax)) * pitch > floor_mm
    thin = skeletonize(M).astype(bool) & (E < 0.45 * floor_mm) & far
    lab, n = ndi.label(thin, S26); out = []
    o = np.array([a[0] for a in ax])
    for k in range(1, n + 1):
        idx = np.argwhere(lab == k); length = float(np.linalg.norm(idx.max(0) - idx.min(0)) * pitch)
        if length >= min_len:
            out.append(dict(len_mm=round(length, 1), radius_mm=round(float(np.median(E[lab == k])), 2),
                            at=(idx.mean(0) * pitch + o).round(1).tolist(), pts=idx * pitch + o))
    return out


def thin_check(part, drawn=None, floor_mm=2.0, pitch=0.4, min_len_mm=None, log=print, offsets=THIN_OFFSETS):
    """members of the built part under floor_mm. 17 Oct 2026: a member found on the base grid counts only if every
    grid shifted by `offsets` (fractions of the pitch) also finds a thin member within one floor of its skeleton. A face
    that lies exactly on a sample plane makes one-voxel slabs on that grid alone (hub v5: 25 of them at Y 44 / 50,
    0 on every shifted grid), which forced needless re-prunes. offsets=() restores the single-grid check."""
    min_len = 2.0 * floor_mm if min_len_mm is None else min_len_mm
    base = _thin_on_grid(part, drawn, floor_mm, pitch, min_len)
    out = base
    if base and offsets:
        shifted = [_thin_on_grid(part, drawn, floor_mm, pitch, min_len, off) for off in offsets]
        def seen(m, others):
            if not others: return False
            P = np.vstack([o["pts"] for o in others])
            d = np.sqrt(((m["pts"][:, None, :] - P[None, :, :]) ** 2).sum(-1)).min(1) if len(P) * len(m["pts"]) < 4e7 else \
                np.array([np.sqrt(((P - p) ** 2).sum(1)).min() for p in m["pts"]])
            return bool((d <= floor_mm).mean() >= 0.5)        # at least half of its skeleton is confirmed
        out = [m for m in base if all(seen(m, s) for s in shifted)]
        if len(out) < len(base):
            log(f"thin check: {len(base) - len(out)} of {len(base)} base-grid member(s) not seen on the shifted grids "
                f"{list(offsets)}: grid artefacts, ignored")
    out = [{k: v for k, v in m.items() if k != "pts"} for m in out]
    log(f"thin check: {len(out)} member(s) under {floor_mm} mm" + ("  OK" if not out else f"  FAIL {out}"))
    return out
