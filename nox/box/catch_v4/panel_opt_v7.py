"""Funnel face rib optimizer v7 (24 Oct 2026): the seated faces with the hook latch. Same Mindlin plate FE, design field,
supports, wheel / top-edge / uniform cases and limits as panel_opt.py (v6, 16 Oct); added:
  - skin 2.0 (the 2.0 floor: the v6 1.6 skin reads thin walls in solid_check), rib height 9 kept;
  - per panel half, the hook latch skin window 9.0 x 13.0 (C) as a hole in the plate, centred at u +-100, v 54.1;
  - over the housing footprint + 1.0 (hook_latch.py result: u_L -25.5..34.0, v_L -10.2..41.6) ribs are low: 6.7 behind
    the skin, 0.3 under the housing flange's panel face (w -9.0); none over the guide block + 1.0 (u_L +-13.2,
    v_L -10.2..11.2), which reaches up to 0.3 under the skin (DEFAULT; the spec's rib-free window left the bosses and
    the strut under the wheel track as islands on the skin, which solid_check fails);
  - the 4 bosses o 12 x 7 per latch as passive thick plate (skin + boss = 9.0, C), gussets left out (conservative);
  - latch cases: the housing is a rigid body on its 4 bosses; a load F along the slope (uphill) at the highest held
    wheel centre (H_LOAD 9.0 off the face, C) is a couple F * (H_LOAD + 9.0) about the lateral axis at the flange face,
    taken by normal boss forces in proportion to their slope offset (conservative: the boss shear's own counter moment
    is left out). functional 30 N on both latches of the face (limit 1.0, as the wheel cases), misuse 100 N on one
    (limit 3.0). The in-plane part F goes into the skin as membrane load (hand calc in funnel_panel_v7.py).
  - extra wheel presses beside the window on the wheel track: u 100 at v 40 and v 65 (functional).
Face coordinates: u across (0 = centre seam), v up the slope (0 = bottom edge). Unit frame of the latch on the u > 0
half: v_FE = U0 + u_L, u_FE = 100 - v_L (v_L toward the centre line).
Fixed ts 2.0, h 9 (one panel family with the v6 flanges and keys); bisection on the rib fraction for the lightest feasible.
python3 panel_opt_v7.py -> rib_mask_v7.json (here) + runs/funnel_panel/v7/panel_opt_v7.json"""
import math, json, time, os, sys
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v7"); os.makedirs(RUN, exist_ok=True)
sys.path[:0] = [os.path.dirname(HERE)]
import hook_latch as HL                                         # latch geometry and loads (C / E), not edited here

SKIN7 = 2.0                                      # panel v7 skin (the 2.0 floor; v6 1.6 reads thin walls in solid_check)
def latch_v7():
    """hook_latch with the panel v7 skin: its frame constants follow the skin (hook_latch.py derives them from SKIN at
    import): guide top 0.3 under the 2.0 skin's back (-2.3), bosses 7 from the skin's back, flange face -9.0 (C).
    Used for every fit check here; hook_latch.py itself is the hook latch part's (SKIN 1.6 -> 2.0 there, PR note)."""
    HL.SKIN = SKIN7; HL.GUIDE_TOP = -SKIN7 - 0.3; HL.W_FL0 = -SKIN7 - HL.BOSS_H; HL.W_FL1 = HL.W_FL0 - HL.FL_T
    return HL
latch_v7()

E, NU, RHO_G = 1460.0, 0.38, 1.27e-3            # PETG Basic card (MPa), g/mm3
KAPPA, G = 5 / 6, 1460.0 / (2 * 1.38)
S2 = math.sqrt(2); SLANT = (77.2 - 1.3) * S2      # 107.35
HW = lambda v: 84.0 + v / S2                     # face half-width at slant v
H = 2.0                                          # element size, mm
FL_W, FL_HALF = 12.0, 2.0                        # passive seam flange: depth, half-width modelled
DEFL = 1.0
TS, RIB_H = SKIN7, 9.0                            # skin (2.0 floor, DEFAULT) and rib height (v6 value)
# ---- hook latch (hook_latch.py, C)
X_L, U0 = HL.X_WHEEL, HL.U0                      # 100, 54.1
WIN_U, WIN_V = HL.PL_U + 2 * HL.CL_SKIN, HL.PL_V + 2 * HL.CL_SKIN   # skin window 9.0 along the slope x 13.0 along the edge
FREE_U, FREE_V = (-25.5, 34.0), (-10.2, 41.6)    # rib-free window, unit frame (housing flange -24.5..33.02 / -9.2..40.6 + 1.0)
BOSS_R, BOSS_T = HL.BOSS_R, SKIN7 + HL.BOSS_H   # boss o 12 x 7 behind the 2.0 skin: 9.0 of plate (C)
W_FACE = HL.W_FL0                                # housing flange panel-side face, -9.0 (C)
_lay = HL.servo_layout(13.0, 1, -1)              # the v1 pose (result.json best)
BOLTS = HL.build(_lay)["bolts"]                  # (u_L, v_L) of the 4 bolts (C)
def to_fe(uL, vL, side=1): return (side * (X_L - vL), U0 + uL)

nx, ny = int(round(320 / H)), int(math.ceil(SLANT / H))
xs = -160 + H * (np.arange(nx) + 0.5); ys = H * (np.arange(ny) + 0.5)
XC, YC = np.meshgrid(xs, ys, indexing="ij")
uL_g = YC - U0; vL_g = X_L - np.abs(XC)                         # unit coordinates of every element (both halves)
hole_g = (np.abs(uL_g) < WIN_U / 2) & (np.abs(vL_g) < WIN_V / 2)
active = (np.abs(XC) <= HW(YC)) & ~hole_g
passive_grid = active & (np.abs(XC) < FL_HALF)
ribfree_grid = active & (uL_g > FREE_U[0]) & (uL_g < FREE_U[1]) & (vL_g > FREE_V[0]) & (vL_g < FREE_V[1])   # low ribs
GUIDE_U = HL.PL_U / 2 + HL.CL_GUIDE + HL.WALL_U + 1.0                      # guide block + 1.0, unit frame (C)
GUIDE_V = (-(HL.PL_V / 2 + HL.CL_GUIDE + HL.WALL_VN) - 1.0, HL.PL_V / 2 + HL.CL_GUIDE + HL.WALL_VP + 1.0)
guide_grid = active & (np.abs(uL_g) < GUIDE_U) & (vL_g > GUIDE_V[0]) & (vL_g < GUIDE_V[1])
H_LOW = -HL.W_FL0 - SKIN7 - 0.3                                              # 6.7 (C)
boss_grid = np.zeros_like(active)
for (bu, bv) in BOLTS:
    boss_grid |= active & ((uL_g - bu) ** 2 + (vL_g - bv) ** 2 <= BOSS_R ** 2)
el_ij = np.argwhere(active); ne = len(el_ij)
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
edof = np.stack([3 * conn[:, k] + d for k in range(4) for d in range(3)], 1)
ec = np.stack([XC[active], YC[active]], 1)
passive = passive_grid[active]; ribfree = ribfree_grid[active]; boss = boss_grid[active]; guide = guide_grid[active]
norib = passive | guide | boss                                 # rho held at 0 (boss and flange thickness set directly)
T_PASS = np.where(passive, FL_W, np.where(boss, BOSS_T, 0.0))   # fixed thickness where > 0
fixedT = T_PASS > 0
sup = np.where((NY < 1e-6) | (np.abs(NX) >= HW(NY) - 0.5 * H))[0]
fixed = 3 * sup
free = np.setdiff1d(np.arange(ndof), fixed)

def q4_unit():
    a = H / 2
    def shape(x, e):
        N = 0.25 * np.array([(1 - x) * (1 - e), (1 + x) * (1 - e), (1 + x) * (1 + e), (1 - x) * (1 + e)])
        dx = 0.25 * np.array([-(1 - e), (1 - e), (1 + e), -(1 + e)]) / a
        de = 0.25 * np.array([-(1 - x), -(1 + x), (1 + x), (1 - x)]) / a
        return N, dx, de
    Db = np.array([[1, NU, 0], [NU, 1, 0], [0, 0, (1 - NU) / 2]])
    Kb = np.zeros((12, 12))
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

def patch(u, v, P, r=5.0):
    sel = np.where((NX - u) ** 2 + (NY - v) ** 2 <= r * r)[0]
    f = np.zeros(ndof); f[3 * sel] = P / len(sel); return f

def latch_forces(F, sides):
    """normal boss forces of a rigid housing under F along the slope at H_LOAD off the face (couple about the flange face)."""
    M = F * (HL.H_LOAD - W_FACE)
    bu = np.array([b[0] for b in BOLTS]); du = bu - bu.mean()
    N = M * du / (du ** 2).sum()
    f = np.zeros(ndof)
    for s in sides:
        for (b, n) in zip(BOLTS, N):
            u, v = to_fe(b[0], b[1], s)
            f += patch(u, v, n, r=3.0)
    return f, N

cases, names, LIMS = [], [], []
for v in (15, 40, 65, 90):
    for u in (0, 30, 60, 90, 120, 150):
        if u <= HW(v) - 8:
            cases.append(patch(u, v, 20.0)); names.append(f"press u{u} v{v}")
for v in (40, 65):
    cases.append(patch(100.0, v, 20.0)); names.append(f"press u100 v{v}")
topv = SLANT - 2.5
for u in (0, 40, 80, 120, 150):
    if u <= HW(topv) - 6:
        cases.append(patch(u, topv, 20.0, r=3.0)); names.append(f"top edge u{u}")
fu = np.zeros(ndof); inside = np.where(np.abs(NX) < HW(NY) - 0.5 * H)[0]; fu[3 * inside] = 20.0 / len(inside)
cases.append(fu); names.append("uniform 20 N")
f_fun, N_fun = latch_forces(HL.F_FUNC, (1, -1)); cases.append(f_fun); names.append("latch functional 30 N x 2")
f_mis, N_mis = latch_forces(HL.F_MIS, (1,)); cases.append(f_mis); names.append("latch misuse 100 N")
F = np.stack(cases, 1); Pm = np.array([np.abs(f).sum() for f in cases])
WHEEL_U = (60.0, 140.0)
def functional(n):
    if n.startswith("uniform") or n.startswith("latch functional"): return True
    if n.startswith("press"):
        u = float(n.split()[1][1:]); return WHEEL_U[0] <= u <= WHEEL_U[1]
    return False
LIM = np.array([DEFL if functional(n) else 3.0 for n in names]); CLIM = Pm * LIM
SIG_ALLOW = 19.5
F[fixed, :] = 0

R = 2.6
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

def thick(rho, ts, h):
    return np.where(fixedT, T_PASS, ts + rho * np.where(ribfree, H_LOW, h))

def solve(rho, ts, h):
    rho = np.where(norib, 0.0, rho)
    h = np.where(ribfree, H_LOW, h)                              # low ribs under the housing flange
    t = thick(rho, ts, h)
    Db = np.where(fixedT, Dof(T_PASS), Dof(ts) + rho ** 3 * (Dof(ts + h) - Dof(ts)))
    Sh = KAPPA * G * t
    vals = (Db[:, None, None] * KB[None] + Sh[:, None, None] * KS[None]).ravel()
    K = sp.coo_matrix((vals, (iK, jK)), shape=(ndof, ndof)).tocsc()
    U = np.zeros((ndof, F.shape[1]))
    U[free] = spl.splu(K[free][:, free].tocsc()).solve(F[free])
    c = (F * U).sum(0)
    dD = np.where(norib, 0.0, 3 * rho ** 2 * (Dof(ts + h) - Dof(ts)))
    dS = np.where(norib, 0.0, KAPPA * G * h)
    Ue = U[edof]
    eb = np.einsum("eic,ij,ejc->ec", Ue, KB, Ue); es = np.einsum("eic,ij,ejc->ec", Ue, KS, Ue)
    return c, -(dD[:, None] * eb + dS[:, None] * es), U

def optimize(ts, h, vf, x0=None, iters=55):
    x = np.full(ne, vf) if x0 is None else x0.copy()
    x[norib] = 0
    kks = 12.0; act = ~norib
    for it in range(iters):
        beta = 0 if it < 15 else min(8.0, 1.0 * 2 ** ((it - 15) // 8))
        rho, dproj = project(Wn @ sym(x), beta)
        c, dc, _ = solve(rho, ts, h)
        g = c / CLIM; m = g.max(); w = np.exp(kks * (g - m))
        dks = (dc * (w / w.sum() / CLIM)[None, :]).sum(1)
        dx = sym(Wn.T @ (dks * dproj)); dvol = sym(Wn.T @ dproj)
        dx[norib] = 0; dx = np.minimum(dx, -1e-12)
        l1, l2, move = 1e-9, 1e9, 0.15
        while (l2 - l1) / (l1 + l2) > 1e-4:
            lm = 0.5 * (l1 + l2)
            xn = np.clip(x * np.sqrt(-dx / (lm * np.maximum(dvol, 1e-12))), np.maximum(0, x - move), np.minimum(1, x + move))
            xn[norib] = 0
            r, _ = project(Wn @ sym(xn), beta)
            if r[act].mean() > vf: l1 = lm
            else: l2 = lm
        x = xn
    rho, _ = project(Wn @ sym(x), 8.0)
    return x, rho

def _bc():
    a = H / 2; B = np.zeros((3, 12)); dx = 0.25 * np.array([-1, 1, 1, -1]) / a; de = 0.25 * np.array([-1, -1, 1, 1]) / a
    for k in range(4):
        B[0, 3 * k + 1] = dx[k]; B[1, 3 * k + 2] = de[k]; B[2, 3 * k + 1] = de[k]; B[2, 3 * k + 2] = dx[k]
    return B
BC = _bc()

def evaluate(mask, ts=TS, h=RIB_H):
    """mean deflection per case (mm), face mass (g, plate model), max von Mises per case (MPa)."""
    mask = mask & ~norib
    c, _, U = solve(mask.astype(float), ts, h)
    t = thick(mask.astype(float), ts, h)
    m = (t * H * H).sum() * RHO_G
    k = np.einsum("ij,ejc->eic", BC, U[edof])
    D = E / (1 - NU ** 2) * t ** 3 / 12
    Mx = D[:, None] * (k[:, 0] + NU * k[:, 1]); My = D[:, None] * (k[:, 1] + NU * k[:, 0]); Mxy = D[:, None] * (1 - NU) / 2 * k[:, 2]
    s = 6 / t[:, None] ** 2
    sx, sy, txy = s * Mx, s * My, s * Mxy
    vm = np.sqrt(sx ** 2 - sx * sy + sy ** 2 + 3 * txy ** 2)
    return c / Pm, m, vm.max(0), U

def boss_defl(U):
    """max normal deflection at the 8 boss centres per case (mm)."""
    ids = []
    for s in (1, -1):
        for b in BOLTS:
            u, v = to_fe(b[0], b[1], s); ids.append(int(np.argmin((NX - u) ** 2 + (NY - v) ** 2)))
    return np.abs(U[3 * np.array(ids)]).max(0)

def report(mask):
    d, m, sg, U = evaluate(mask)
    util = d / LIM; bd = boss_defl(U)
    i = int(util.argmax()); j = int(sg.argmax())
    return dict(face_g=round(m, 1), util=round(float(util.max()), 3), worst=names[i], stress_MPa=round(float(sg.max()), 1),
                stress_case=names[j], ok=bool((util <= 1 + 1e-9).all() and sg.max() <= SIG_ALLOW),
                per_case={n: [round(float(a), 3), round(float(b), 1)] for n, a, b in zip(names, d, sg)},
                latch_boss_defl_mm={names[k]: round(float(bd[k]), 3) for k in (len(names) - 2, len(names) - 1)})

if __name__ == "__main__":
    t0 = time.time()
    print(f"mesh {ne} el ({int(hole_g.sum())} window hole, {int(ribfree.sum())} low-rib, {int(guide.sum())} guide, {int(boss.sum())} boss), {ndof} dof, {len(names)} cases", flush=True)
    print("boss forces N (functional, misuse):", np.round(N_fun, 2).tolist(), np.round(N_mis, 2).tolist(), flush=True)
    base = report(np.zeros(ne, bool)); print("skin only:", {k: base[k] for k in ("face_g", "util", "worst", "stress_MPa")}, flush=True)
    lo, hi, best, x0, sweep = 0.0, 0.8, None, None, []
    for k in range(7):
        vf = 0.5 * (lo + hi)
        x, rho = optimize(TS, RIB_H, vf, x0)
        mask = (rho > 0.5) & ~norib
        r = report(mask)
        sweep.append(dict(vf=round(vf, 4), **{k2: r[k2] for k2 in ("face_g", "util", "worst", "stress_MPa", "ok")}))
        print(f"vf {vf:.3f}: util {r['util']:.3f} ({r['worst']}), stress {r['stress_MPa']} MPa, face {r['face_g']} g  {'OK' if r['ok'] else '--'}  [{time.time()-t0:.0f}s]", flush=True)
        if r["ok"]: hi, best, x0 = vf, (mask.copy(), r, vf), x
        else: lo, x0 = vf, x
    mask, r, vf = best
    json.dump(dict(ts=TS, h=RIB_H, H=H, ec=np.round(ec, 3).tolist(), mask=np.where(mask)[0].tolist(), vf=vf,
                   window=dict(centre_u=X_L, centre_v=U0, along_slope=WIN_U, along_edge=WIN_V),
                   lowrib_unit=dict(u=FREE_U, v=FREE_V, h=H_LOW), guide_keep_unit=dict(u=GUIDE_U, v=GUIDE_V)), open(os.path.join(HERE, "rib_mask_v7.json"), "w"))
    json.dump(dict(elements=ne, cases=names, boss_forces_N=dict(functional=np.round(N_fun, 2).tolist(), misuse=np.round(N_mis, 2).tolist()),
                   skin_only={k: base[k] for k in ("face_g", "util", "worst", "stress_MPa")}, sweep=sweep, chosen=r,
                   wall_s=round(time.time() - t0)), open(os.path.join(RUN, "panel_opt_v7.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in r.items() if k != "per_case"}, indent=1))
