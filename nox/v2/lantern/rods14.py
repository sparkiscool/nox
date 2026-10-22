"""Cage rod-arrangement frame model (22 Oct 2026, research chat; Spark 13:41 "the arrangement of the rods ... make it better").
Beam model of the lantern: every 3.0 carbon rod and the printed columns / hub as 3D frame elements, cage held at the
6 core attachments (2 struts, 4 rail sleeves; grommets taken as rigid, conservative). Load cases:
  wall  : flat rigid wall from every direction above the feet; contact = the hull vertices within 1 mm of the extreme;
          energy capacity at 1500 MPa rod stress (E) -> v_wall = sqrt(2 E / 0.52 kg).
  edge  : plank / pipe edge at mid-span of every exterior rod span (between nodes), normal to the rod, inward;
          energy capacity (J) at 1500 MPa, and the rod bow at that energy.
  props : static hull clearance to the swept prop disks in every direction above z -0.3 (needs >= 12),
          probe test: a ball (38 mm = a 2x4 edge) pushed straight in from any direction must touch a rod before it
          gets within 2 mm of a swept disk; count of probe lines that reach a disk first.
Mass: rods 10.96 g/m (3.0 solid, 1.55 g/cc, E); printed parts calibrated to v13's 100 g (M): 1.18 g per rod-end socket, 0.59 per through-sleeve junction,
0.074 g/mm of printed column (8 x 8 PA12-CF), hub ring 0.04 g/mm (all E).
Tags: everything here is F (frame model) or E unless noted. Coordinates: drone mm, x nose, y left, z up."""
import math, json, sys, numpy as np
import scipy.sparse as sp, scipy.sparse.linalg as spl

E_C, G_C, RHO_ROD = 130000.0, 5000.0, 10.96e-3          # MPa, MPa, g/mm (E)
E_P, G_P = 6000.0, 2200.0                                # PA12-CF printed members (E)
SIG = 1500.0                                             # rod strength (E), D21 replaces
M_ALL = 0.52                                             # kg, all-up (E)
D = 3.0; A_R = math.pi*D*D/4; I_R = math.pi*D**4/64
G_CG = np.array([0.57, 0.0, 2.79])
MOT = [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]; R_PROP, Z_PROP, SWEEP = 44.45, 41.2, 4.0
SOCKET_G, THRU_G, COL_G_MM, HUB_G_MM = 1.18, 0.59, 0.074, 0.04

def sec_rect(b):
    A = b*b; I = b**4/12; return A, I, 0.141*b**4

def frame_k(p1, p2, E, G, A, I, J):
    L = np.linalg.norm(p2-p1); ex = (p2-p1)/L
    ref = np.array([0, 0, 1.]) if abs(ex[2]) < 0.95 else np.array([1., 0, 0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    R = np.vstack([ex, ey, ez]); T = np.zeros((12, 12))
    for i in range(4): T[3*i:3*i+3, 3*i:3*i+3] = R
    k = np.zeros((12, 12)); a = E*A/L; t = G*J/L
    b1 = 12*E*I/L**3; b2 = 6*E*I/L**2; b3 = 4*E*I/L; b4 = 2*E*I/L
    k[0, 0] = k[6, 6] = a; k[0, 6] = k[6, 0] = -a
    k[3, 3] = k[9, 9] = t; k[3, 9] = k[9, 3] = -t
    for (v, r, s) in ((1, 5, 1), (2, 4, -1)):
        k[v, v] = k[v+6, v+6] = b1; k[v, v+6] = k[v+6, v] = -b1
        k[v, r] = k[r, v] = s*b2; k[v, r+6] = k[r+6, v] = s*b2
        k[v+6, r] = k[r, v+6] = -s*b2; k[v+6, r+6] = k[r+6, v+6] = -s*b2
        k[r, r] = k[r+6, r+6] = b3; k[r, r+6] = k[r+6, r] = b4
    return T.T@k@T, T, k

class Cage:
    """rods: list of (polyline points, tag); a polyline is one continuous rod through its interior points (through-sockets).
    printed: list of (a, b, size, tag) printed members. supports: points held fixed. hub_len: printed hub ring length (mass)."""
    def __init__(s, rods, printed, supports, hub_len=0.0, seg=10.0, thru=0):
        s.X = []; s.idx = {}; s.el = []; s.rods = rods; s.printed = printed
        for poly, tag in rods:
            for a, b in zip(poly[:-1], poly[1:]):
                s._line(a, b, 'C', tag, seg)
        for a, b, size, tag in printed:
            s._line(a, b, ('P', size), tag, 6.0)
        s.X = np.array(s.X); s.N = len(s.X)
        s.fix = [s._node(p) for p in supports]
        s.hub_len = hub_len; s.thru = thru
        s._assemble()
    def find(s, p):
        return int(np.argmin(np.linalg.norm(s.X-np.asarray(p, float), axis=1)))
    def _node(s, p):
        if isinstance(s.X, np.ndarray): return s.find(p)
        key = tuple(np.round(np.asarray(p, float), 3))
        if key not in s.idx: s.idx[key] = len(s.X); s.X.append(np.array(key))
        return s.idx[key]
    def _line(s, a, b, mat, tag, seg):
        a = np.asarray(a, float); b = np.asarray(b, float); n = max(2, int(math.ceil(np.linalg.norm(b-a)/seg))); n += n % 2
        ids = [s._node(a+(b-a)*i/n) for i in range(n+1)]
        for i in range(n): s.el.append(dict(n=(ids[i], ids[i+1]), mat=mat, tag=tag))
    def _assemble(s):
        rows, cols, vals = [], [], []
        for e in s.el:
            p1, p2 = s.X[e['n'][0]], s.X[e['n'][1]]
            if e['mat'] == 'C': E, G, A, I, J = E_C, G_C, A_R, I_R, 2*I_R
            else: A, I, J = sec_rect(e['mat'][1]); E, G = E_P, G_P
            kg, T, kl = frame_k(p1, p2, E, G, A, I, J); e['T'] = T; e['kl'] = kl
            dof = np.r_[6*e['n'][0]:6*e['n'][0]+6, 6*e['n'][1]:6*e['n'][1]+6]
            r, c = np.meshgrid(dof, dof, indexing='ij'); rows.append(r.ravel()); cols.append(c.ravel()); vals.append(kg.ravel())
        n = 6*s.N
        K = sp.csc_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n))
        fx = np.zeros(n, bool)
        for i in s.fix: fx[6*i:6*i+6] = True
        s.free = np.where(~fx)[0]
        s.lu = spl.splu(K[s.free][:, s.free].tocsc() + sp.identity(len(s.free))*1e-9)
        s.rod_el = np.array([e['mat'] == 'C' for e in s.el])
    def solve(s, F):
        u = np.zeros(6*s.N); u[s.free] = s.lu.solve(F[s.free]); return u
    def _prep_stress(s):
        s.KT = np.array([e['kl']@e['T'] for e in s.el])
        s.DOF = np.array([np.r_[6*e['n'][0]:6*e['n'][0]+6, 6*e['n'][1]:6*e['n'][1]+6] for e in s.el])
        A = np.empty(len(s.el)); I = np.empty(len(s.el)); c = np.empty(len(s.el)); sc = np.empty(len(s.el))
        for k, e in enumerate(s.el):
            if e['mat'] == 'C': A[k], I[k], c[k], sc[k] = A_R, I_R, D/2, 1.0
            else: b = e['mat'][1]; A[k], I[k], _ = sec_rect(b); c[k] = b/2*1.41; sc[k] = SIG/50.0
        s.sA, s.sI, s.sc, s.sscale = A, I, c, sc
    def rod_stress(s, u):
        """max combined stress per element (MPa); printed members scaled so 50 MPa (E) reads as SIG"""
        if not hasattr(s, 'KT'): s._prep_stress()
        f = np.einsum('kij,kj->ki', s.KT, u[s.DOF])
        s1 = np.abs(f[:, 0])/s.sA + np.hypot(f[:, 4], f[:, 5])*s.sc/s.sI
        s2 = np.abs(f[:, 6])/s.sA + np.hypot(f[:, 10], f[:, 11])*s.sc/s.sI
        return np.maximum(s1, s2)*s.sscale
    def point_capacity(s, ids, d):
        """rigid flat indenter along d at contact nodes ids (all pushed the same distance); returns energy capacity J,
        force N and displacement mm when the worst element reaches SIG"""
        cols = []
        for i in ids:
            F = np.zeros(6*s.N); F[6*i:6*i+3] = d; cols.append(s.solve(F))
        C = np.array([[np.dot(cols[j][6*i:6*i+3], d) for j in range(len(ids))] for i in ids])
        act = list(range(len(ids)))
        while True:
            f = np.linalg.solve(C[np.ix_(act, act)], np.ones(len(act)))
            if (f >= -1e-9).all() or len(act) == 1: break
            act = [a for a, v in zip(act, f) if v > 1e-9]
        f = np.clip(f, 0, None); k = f.sum()
        u = sum(fq*cols[a] for fq, a in zip(f, act))            # displacement field for unit indent (1 mm)
        sig = s.rod_stress(u).max()                              # stress per mm of indent
        delta = SIG/sig; Ecap = 0.5*k*delta**2/1000.0             # J
        return Ecap, k*delta, delta, u*delta
    def mass(s):
        rod_len = sum(sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(p[:-1], p[1:])) for p, t in s.rods)
        sockets = 2*len(s.rods)
        col_len = sum(np.linalg.norm(np.subtract(b, a)) for a, b, sz, t in s.printed if t == 'column')
        return dict(rod_g=round(rod_len*RHO_ROD, 1), printed_g=round(sockets*SOCKET_G + s.thru*THRU_G + col_len*COL_G_MM + s.hub_len*HUB_G_MM, 1),
                    sockets=sockets, rods=len(s.rods), rod_m=round(rod_len/1000, 2))

def prop_points(n=48):
    pts = []
    for mx, my in MOT:
        for k in range(n):
            a = 2*math.pi*k/n
            for dz in (-SWEEP, SWEEP): pts.append((mx+R_PROP*math.cos(a), my+R_PROP*math.sin(a), Z_PROP+dz))
    return np.array(pts)

def fib_dirs(n, zmin=-0.3):
    i = np.arange(n)+0.5; z = 1-2*i/n; r = np.sqrt(1-z*z); th = math.pi*(1+5**0.5)*i
    U = np.c_[r*np.cos(th), r*np.sin(th), z]; return U[U[:, 2] >= zmin]

def seg_dist(p, a, b):
    ab = b-a; t = np.clip(((p-a)@ab)/(ab@ab), 0, 1); return np.linalg.norm(p-(a+np.outer(t, ab) if p.ndim > 1 else a+t*ab), axis=-1)

def evaluate(cage, nodes_hull, ext_spans, probe_r=15.0, n_dirs=160, verbose=False):
    """nodes_hull: points that form the outside (rod ends / through points); ext_spans: (a, b, tag) exterior rod spans"""
    H = np.array(nodes_hull, float); res = {}
    # wall
    worst = (1e9, None); U = fib_dirs(n_dirs*2)
    for u in U:
        h = H@u; ext = np.where(h >= h.max()-1.0)[0]
        ids = [cage._node(H[i]) for i in ext]
        E, F, dl, _ = cage.point_capacity(ids, -u)
        if E < worst[0]: worst = (E, tuple(np.round(u, 2)), F, dl)
    res['wall_J'] = round(worst[0], 2); res['wall_v'] = round(math.sqrt(2*worst[0]/M_ALL), 2); res['wall_dir'] = worst[1]
    # edge hits
    ew = []; ctr = np.array([0, 0, 15.0])
    for a, b, tag in ext_spans:
        a = np.asarray(a, float); b = np.asarray(b, float); m = (a+b)/2; ax = (b-a)/np.linalg.norm(b-a)
        o = m-ctr; o = o-(o@ax)*ax; o /= np.linalg.norm(o)
        i = cage._node(m)
        E, F, dl, uf = cage.point_capacity([i], -o)
        ew.append((round(E, 3), tag, tuple(np.round(m, 1)), round(dl, 1)))
    ew.sort(); res['edge_min_J'] = ew[0][0]; res['edge_worst'] = ew[0][1:]; res['edge_list'] = ew[:6]
    # static prop clearance and probe test
    P = prop_points(); Ud = fib_dirs(400, zmin=-0.3)
    res['prop_hull_clear'] = round(min((H@u).max()-(P@u).max() for u in Ud), 1)
    from scipy.spatial import cKDTree
    S = []
    for poly, t in cage.rods:
        for a, b in zip(poly[:-1], poly[1:]):
            a = np.asarray(a, float); b = np.asarray(b, float); n = max(2, int(np.linalg.norm(b-a)/3))
            S += [a+(b-a)*q for q in np.linspace(0, 1, n)]
    for a, b, sz, t in cage.printed:
        a = np.asarray(a, float); b = np.asarray(b, float); n = max(2, int(np.linalg.norm(b-a)/3))
        S += [a+(b-a)*q for q in np.linspace(0, 1, n)]
    S = np.array(S); fails = 0; tested = 0; g = np.arange(-170, 171, 6.0); G1, G2 = np.meshgrid(g, g); Q = np.c_[G1.ravel(), G2.ravel()]
    for u in fib_dirs(160, zmin=-0.2):
        e1 = np.cross(u, [0, 0, 1.0] if abs(u[2]) < 0.9 else [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
        tr = cKDTree(np.c_[S@e1, S@e2]); dr = S@u; tp = cKDTree(np.c_[P@e1, P@e2]); dpp = P@u
        rl = tr.query_ball_point(Q, probe_r+1.5); pl = tp.query_ball_point(Q, probe_r+2.0)
        for r_, p_ in zip(rl, pl):
            if not p_: continue
            tested += 1
            if not r_ or dr[r_].max() < dpp[p_].max(): fails += 1
    res['probe_fail'] = fails; res['probe_tested'] = tested
    res.update(cage.mass())
    return res

# ---------------------------------------------------------------- lantern builder (v13 = defaults)
CORN = [(106, 106), (-106, 106), (-106, -106), (106, -106)]
TEES = [(112, 0), (0, 112), (-112, 0), (0, -112)]
def flats(z):   # 4 through rods, one per flat, through the tee at its middle
    return [[(100, 112, z), (0, 112, z), (-100, 112, z)], [(-112, 100, z), (-112, 0, z), (-112, -100, z)],
            [(-100, -112, z), (0, -112, z), (100, -112, z)], [(112, -100, z), (112, 0, z), (112, 100, z)]]
def chamfer_links(z):
    out = []
    for (cx, cy) in CORN:
        sx, sy = np.sign(cx), np.sign(cy)
        out += [((cx, cy, z), (sx*100, sy*112, z), 8.0, 'link'), ((cx, cy, z), (sx*112, sy*100, z), 8.0, 'link')]
    return out

def lower_half():
    """fixed by the funnel, sled, swap head and core attachments: skid ring, glove, sled, struts, nose legs, supports"""
    rods = [(f, 'skid ring') for f in flats(33.2)]
    printed = chamfer_links(33.2)
    ZB, YB, XN, XT = -47, 32, 37.5, -52
    for s in (1, -1):
        rods.append(([(XN, s*YB, ZB), (15, s*YB, ZB), (-5, s*YB, ZB), (-25, s*YB, ZB), (XT, s*YB, ZB)], 'sled rail'))
        rods.append(([(106, s*106, 33.2), (XN, s*YB, -7)], 'glove nose'))
        rods.append(([(-106, s*106, 33.2), (-66, s*52, -0.8)], 'glove tail'))
        rods.append(([(-66, s*52, -0.8), (XT, s*YB, ZB)], 'glove tail low'))
        rods.append(([(0, s*112, 33.2), (-5, s*YB, ZB)], 'glove side'))
        rods.append(([(0, s*112, 33.2), (2, s*50, 24)], 'strut'))
        printed.append(((XN, s*YB, -7), (XN, s*YB, ZB), 8.0, 'leg'))
    rods.append(([(XT, YB, ZB), (XT, -YB, ZB)], 'sled cross'))
    supports = [(2, 50, 24), (2, -50, 24), (15, 32, -47), (15, -32, -47), (-25, 32, -47), (-25, -32, -47)]
    return rods, printed, supports

def v13_upper(zs=60.0, zh=85.0, rh=40.0):
    rods = [(f, 'shoulder ring') for f in flats(zs)]
    printed = chamfer_links(zs)
    for x, y in CORN + TEES: printed.append(((x, y, 33.2), (x, y, zs), 8.0, 'column'))
    hub = [(rh*math.cos(k*math.pi/4), rh*math.sin(k*math.pi/4), zh) for k in range(8)]
    for x, y in TEES + CORN:
        a = math.atan2(y, x); k = int(round(a/(math.pi/4))) % 8
        rods.append(([(x, y, zs), hub[k]], 'spoke'))
    for k in range(8): printed.append((hub[k], hub[(k+1) % 8], 6.0, 'hub'))
    hub_len = 8*2*rh*math.sin(math.pi/8)
    return rods, printed, hub_len

def build(upper=None):
    r, p, sup = lower_half()
    ur, up, hl = (upper or v13_upper)()
    rods = r+ur; printed = p+up
    cage = Cage(rods, printed, sup, hub_len=hl)
    hull = sorted({tuple(map(float, q)) for poly, t in rods if t != 'strut' for q in poly})
    spans = [(a, b, t) for poly, t in rods if t != 'strut' for a, b in zip(poly[:-1], poly[1:])]
    return cage, hull, spans

if __name__ == '__main__':
    import time; t0 = time.time()
    cage, hull, spans = build()
    r = evaluate(cage, hull, spans)
    print(json.dumps({k: v for k, v in r.items()}, indent=0, default=str)); print('t', round(time.time()-t0, 1), 's, nodes', cage.N)
