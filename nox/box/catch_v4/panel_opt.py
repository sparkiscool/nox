"""Funnel face rib optimizer (Mindlin plate FE, variable rib field), 16 Oct 2026.
One full face (A + B) in face coordinates: u across (0 = centre seam), v up the slope (0 = bottom edge).
Design: skin ts everywhere + rib field rho (0..1) of height h behind the skin, symmetric about u = 0.
Supports: bottom edge (foot on the box top) and both valleys (fold lines) w = 0; top edge free.
Passive: the glued centre-seam flange pair (4 wide x 12 deep) along u = 0.
Load cases (v3 panel cases): 20 N patch presses (r 5) on a grid, 20 N on the top edge, 20 N uniform.
Constraints (DEFAULT): functional presses (where wheels / contacts land) and uniform: mean deflection <= 1.0 mm;
misuse (20 N on the free top edge and anywhere off the wheel track): <= 3.0 mm, and every case's plate stress <= 19.5 MPa (PETG XY 39 / SF 2, checked).
Objective: minimum mass.
Sweeps ts x h, keeps the lightest feasible design, writes rib_mask.json + panel_opt.json (16 Oct run: ts 1.6, h 9, 123 g / face)."""
import math, json, time, sys
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl

E, NU, RHO_G = 1460.0, 0.38, 1.27e-3            # PETG Basic card (MPa), g/mm3
KAPPA, G = 5 / 6, 1460.0 / (2 * 1.38)
S2 = math.sqrt(2); SLANT = (77.2 - 1.3) * S2      # 107.35
HW = lambda v: 84.0 + v / S2                     # face half-width at slant v
H = 2.0                                          # element size, mm
FL_W, FL_HALF = 12.0, 2.0                        # passive seam flange: depth, half-width modelled
DEFL = 1.0
nx, ny = int(round(320 / H)), int(math.ceil(SLANT / H))
xs = -160 + H * (np.arange(nx) + 0.5); ys = H * (np.arange(ny) + 0.5)
XC, YC = np.meshgrid(xs, ys, indexing="ij")
active = np.abs(XC) <= HW(YC)
passive_grid = active & (np.abs(XC) < FL_HALF)
el_ij = np.argwhere(active); ne = len(el_ij)
# nodes
nid_full = -np.ones((nx + 1, ny + 1), int)
for i, j in el_ij:
    for a in (0, 1):
        for b in (0, 1):
            nid_full[i + a, j + b] = 0
nodes = np.argwhere(nid_full == 0); nn = len(nodes)
nid_full[nodes[:, 0], nodes[:, 1]] = np.arange(nn)
NX = -160 + H * nodes[:, 0]; NY = H * nodes[:, 1]
conn = np.array([[nid_full[i, j], nid_full[i + 1, j], nid_full[i + 1, j + 1], nid_full[i, j + 1]] for i, j in el_ij])
ndof = 3 * nn
edof = np.stack([3 * conn[:, k] + d for k in range(4) for d in range(3)], 1)   # (ne, 12): node-major w, bx, by
ec = np.stack([XC[active], YC[active]], 1)
passive = passive_grid[active]
# supports: bottom edge v = 0, valleys |u| >= HW(v) - 0.5 H  (w only)
sup = np.where((NY < 1e-6) | (np.abs(NX) >= HW(NY) - 0.5 * H))[0]
fixed = 3 * sup
free = np.setdiff1d(np.arange(ndof), fixed)

def q4_unit():
    """Kb for D = 1 (bending, 2x2 Gauss), Ks for kappa*G*t = 1 (shear, 1 point), square element H."""
    a = H / 2
    def shape(x, e):
        N = 0.25 * np.array([(1 - x) * (1 - e), (1 + x) * (1 - e), (1 + x) * (1 + e), (1 - x) * (1 + e)])
        dx = 0.25 * np.array([-(1 - e), (1 - e), (1 + e), -(1 + e)]) / a
        de = 0.25 * np.array([-(1 - x), -(1 + x), (1 + x), (1 - x)]) / a
        return N, dx, de
    Db = np.array([[1, NU, 0], [NU, 1, 0], [0, 0, (1 - NU) / 2]])
    Kb = np.zeros((12, 12)); Ks = np.zeros((12, 12))
    g = 1 / math.sqrt(3)
    for x in (-g, g):
        for e in (-g, g):
            N, dx, de = shape(x, e); B = np.zeros((3, 12))
            for k in range(4):
                B[0, 3 * k + 1] = dx[k]; B[1, 3 * k + 2] = de[k]; B[2, 3 * k + 1] = de[k]; B[2, 3 * k + 2] = dx[k]
            Kb += B.T @ Db @ B * a * a
    N, dx, de = shape(0, 0); B = np.zeros((2, 12))
    for k in range(4):
        B[0, 3 * k] = dx[k]; B[0, 3 * k + 1] = N[k]; B[1, 3 * k] = de[k]; B[1, 3 * k + 2] = N[k]
    Ks = B.T @ B * 4 * a * a
    return Kb, Ks
KB, KS = q4_unit()
Dof = lambda t: E * t ** 3 / (12 * (1 - NU ** 2))
iK = np.repeat(edof, 12, 1).ravel(); jK = np.tile(edof, (1, 12)).ravel()

# load cases (on the u >= 0 half; the design is symmetric so mirrored loads mirror the response)
def patch(u, v, P, r=5.0):
    sel = np.where((NX - u) ** 2 + (NY - v) ** 2 <= r * r)[0]
    f = np.zeros(ndof); f[3 * sel] = P / len(sel); return f
cases, names = [], []
for v in (15, 40, 65, 90):
    for u in (0, 30, 60, 90, 120, 150):
        if u <= HW(v) - 8:
            cases.append(patch(u, v, 20.0)); names.append(f"press u{u} v{v}")
topv = SLANT - 2.5
for u in (0, 40, 80, 120, 150):
    if u <= HW(topv) - 6:
        cases.append(patch(u, topv, 20.0, r=3.0)); names.append(f"top edge u{u}")
fu = np.zeros(ndof); inside = np.where(np.abs(NX) < HW(NY) - 0.5 * H)[0]; fu[3 * inside] = 20.0 / len(inside)
cases.append(fu); names.append("uniform 20 N")
F = np.stack(cases, 1); Pm = np.array([f.sum() for f in cases])
WHEEL_U = (60.0, 140.0)                         # wheel track off the centre line (box-funnel.md); Overseer contacts in it (DEFAULT)
def functional(n):
    if n.startswith("uniform"): return True
    if n.startswith("press"):
        u = float(n.split()[1][1:]); return WHEEL_U[0] <= u <= WHEEL_U[1]
    return False
LIM = np.array([DEFL if functional(n) else 3.0 for n in names]); CLIM = Pm * LIM
SIG_ALLOW = 19.5
F[fixed, :] = 0

# density filter (radius R) + symmetry
R = 2.6
from scipy.spatial import cKDTree
tree = cKDTree(ec); pairs = tree.sparse_distance_matrix(tree, R, output_type="coo_matrix")
W = sp.csr_matrix((np.maximum(0, R - pairs.data), (pairs.row, pairs.col)), shape=(ne, ne))
W = W + sp.identity(ne) * R
Wn = sp.diags(1 / np.asarray(W.sum(1)).ravel()) @ W
mirror = cKDTree(ec).query(np.stack([-ec[:, 0], ec[:, 1]], 1))[1]
def sym(x): return 0.5 * (x + x[mirror])
def project(x, beta, eta=0.5):
    if beta <= 0: return x, np.ones_like(x)
    th = math.tanh(beta * eta); d = th + math.tanh(beta * (1 - eta))
    y = (th + np.tanh(beta * (x - eta))) / d
    return y, beta * (1 - np.tanh(beta * (x - eta)) ** 2) / d

def solve(rho, ts, h):
    t = np.where(passive, FL_W, ts + rho * h)
    Db = np.where(passive, Dof(FL_W), Dof(ts) + rho ** 3 * (Dof(ts + h) - Dof(ts)))
    Sh = KAPPA * G * t
    vals = (Db[:, None, None] * KB[None] + Sh[:, None, None] * KS[None]).ravel()
    K = sp.coo_matrix((vals, (iK, jK)), shape=(ndof, ndof)).tocsc()
    Kf = K[free][:, free]
    U = np.zeros((ndof, F.shape[1]))
    U[free] = spl.splu(Kf.tocsc()).solve(F[free])
    c = (F * U).sum(0)
    dD = np.where(passive, 0.0, 3 * rho ** 2 * (Dof(ts + h) - Dof(ts)))
    dS = np.where(passive, 0.0, KAPPA * G * h)
    Ue = U[edof]                                    # (ne, 12, ncase)
    eb = np.einsum("eic,ij,ejc->ec", Ue, KB, Ue); es = np.einsum("eic,ij,ejc->ec", Ue, KS, Ue)
    dc = -(dD[:, None] * eb + dS[:, None] * es)      # (ne, ncase)
    return c, dc, U

def optimize(ts, h, vf, x0=None, iters=60, log=False):
    """min KS(c_i / clim_i) s.t. rib fraction <= vf; OC, filter + Heaviside continuation."""
    x = np.full(ne, vf) if x0 is None else x0.copy()
    x[passive] = 0
    kks = 12.0
    for it in range(iters):
        beta = 0 if it < 15 else min(8.0, 1.0 * 2 ** ((it - 15) // 8))
        xf = Wn @ sym(x); rho, dproj = project(xf, beta)
        c, dc, _ = solve(rho, ts, h)
        g = c / CLIM; m = g.max(); w = np.exp(kks * (g - m)); ks = m + math.log(w.sum()) / kks
        dks = (dc * (w / w.sum() / CLIM)[None, :]).sum(1)
        dx = sym(Wn.T @ (dks * dproj)); dvol = sym(Wn.T @ dproj)
        dx[passive] = 0; dx = np.minimum(dx, -1e-12)
        l1, l2, move = 1e-9, 1e9, 0.15
        act = ~passive
        while (l2 - l1) / (l1 + l2) > 1e-4:
            lm = 0.5 * (l1 + l2)
            xn = np.clip(x * np.sqrt(-dx / (lm * np.maximum(dvol, 1e-12))), np.maximum(0, x - move), np.minimum(1, x + move))
            xn[passive] = 0
            r, _ = project(Wn @ sym(xn), beta)
            if r[act].mean() > vf: l1 = lm
            else: l2 = lm
        x = xn
        if log and it % 10 == 0: print(f"  it {it} beta {beta} ks {ks:.3f} max g {m:.3f} vf {rho[act].mean():.3f}", flush=True)
    xf = Wn @ sym(x); rho, _ = project(xf, 8.0)
    return x, rho

def _bc():
    a = H / 2; B = np.zeros((3, 12)); dx = 0.25 * np.array([-1, 1, 1, -1]) / a; de = 0.25 * np.array([-1, -1, 1, 1]) / a
    for k in range(4):
        B[0, 3 * k + 1] = dx[k]; B[1, 3 * k + 2] = de[k]; B[2, 3 * k + 1] = de[k]; B[2, 3 * k + 2] = dx[k]
    return B
BC = _bc()

def evaluate(mask, ts, h, stress=False):
    c, _, U = solve(mask.astype(float), ts, h)
    t = np.where(passive, FL_W, ts + mask * h)
    vol = (t * H * H).sum()
    if not stress: return c / Pm, vol * RHO_G      # mean deflection per case, face mass (skin + ribs + flange strip)
    k = np.einsum("ij,ejc->eic", BC, U[edof])      # curvatures (ne, 3, ncase)
    D = E / (1 - NU ** 2) * t ** 3 / 12
    Mx = D[:, None] * (k[:, 0] + NU * k[:, 1]); My = D[:, None] * (k[:, 1] + NU * k[:, 0]); Mxy = D[:, None] * (1 - NU) / 2 * k[:, 2]
    s = 6 / t[:, None] ** 2
    sx, sy, txy = s * Mx, s * My, s * Mxy
    vm = np.sqrt(sx ** 2 - sx * sy + sy ** 2 + 3 * txy ** 2)
    return c / Pm, vol * RHO_G, vm.max(0)

if __name__ == "__main__":
    t0 = time.time(); results = []
    print(f"mesh {ne} el, {ndof} dof, {len(names)} cases", flush=True)
    for ts in (1.2, 1.6, 2.0):
        # skin-only check first (no ribs)
        d0, m0, s0 = evaluate(np.zeros(ne, bool), ts, 0.0, stress=True); d0 = d0 / LIM
        for h in (6.0, 9.0):
            lo, hi, best, x0 = 0.0, 0.8, None, None
            for k in range(6):
                vf = 0.5 * (lo + hi)
                x, rho = optimize(ts, h, vf, x0, iters=55)
                mask = (rho > 0.5) & ~passive
                d, m, sg = evaluate(mask, ts, h, stress=True)
                ok = (d <= LIM + 1e-9).all() and sg.max() <= SIG_ALLOW
                d = d / LIM * DEFL                         # report as utilisation x 1 mm
                print(f"ts {ts} h {h} vf {vf:.3f}: util {d.max():.3f} stress {sg.max():.1f} MPa ({names[int(d.argmax())]}), face {m:.1f} g  {'OK' if ok else '--'}  [{time.time()-t0:.0f}s]", flush=True)
                if ok: hi, best, x0 = vf, (m, d.max(), mask.copy(), names[int(d.argmax())]), x
                else: lo = vf; x0 = x
            if best: results.append(dict(ts=ts, h=h, face_g=round(best[0], 1), max_defl=round(best[1], 3), worst=best[3], mask=best[2]))
        results.append(dict(ts=ts, h=0.0, face_g=round(m0, 1), max_defl=round(d0.max(), 3), worst=names[int(d0.argmax())], mask=None, skin_only=True))
    feas = [r for r in results if r["max_defl"] <= DEFL and (not r.get("skin_only") or s0.max() <= SIG_ALLOW)]
    win = min(feas, key=lambda r: r["face_g"])
    mk = win["mask"] if win["mask"] is not None else np.zeros(ne, bool)
    json.dump(dict(ts=win["ts"], h=win["h"], H=H, ec=np.round(ec, 3).tolist(), mask=np.where(mk)[0].tolist()), open("rib_mask.json", "w"))
    out = dict(sweep=[{k: v for k, v in r.items() if k != "mask"} for r in results],
               chosen={k: v for k, v in win.items() if k != "mask"}, cases=names, elements=ne)
    d, m, sg = evaluate(win["mask"] if win["mask"] is not None else np.zeros(ne, bool), win["ts"], win["h"], stress=True)
    out["chosen_defl_per_case"] = {n: [round(float(x), 3), round(float(y), 1)] for n, x, y in zip(names, d, sg)}
    # baseline: the 21:30 panel (skin 2.8 + band 6 x 8.4) in the same model
    base = (ec[:, 1] >= SLANT - 6.0) & ~passive
    db, mb, sb = evaluate(base, 2.8, 8.4 - 2.8, stress=True)
    out["baseline_2p8_band"] = dict(face_g=round(mb, 1), util=round(float((db / LIM).max()), 3), worst=names[int((db / LIM).argmax())], stress=round(float(sb.max()), 1))
    json.dump(out, open("panel_opt.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "chosen_defl_per_case"}, indent=1))
