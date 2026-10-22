"""Rod-arrangement search for the lantern's upper half (drum + crown), 22 Oct 2026 (research chat).
The lower half (skid ring, glove, sled, struts, nose legs) is fixed by the funnel, the sled, the swap head and the core
attachments. Every upper-half layout in the grid is built, checked against the keep-outs and scored with rods14.evaluate.
Run: python3 rods14_search.py [workers]  -> rods14_search.json (all rows) + the best rows printed.
Gates (all relative to v13 in the same model, F): rods 12 off the swept disks, printed 6; hull 12 off the disks;
nothing above z 85; plank-edge probe (38 mm) fails <= v13; wall and edge capacities >= v13. Objective: mass."""
import sys, math, json, itertools, numpy as np, multiprocessing as mp
import rods14 as R

def disk_clear(pts):
    """min distance from points to the swept prop disks (solid cylinders r 44.45, z 41.2 +/- 4)"""
    pts = np.asarray(pts, float); best = 1e9
    for mx, my in R.MOT:
        dr = np.maximum(np.hypot(pts[:, 0]-mx, pts[:, 1]-my)-R.R_PROP, 0)
        dz = np.maximum(np.abs(pts[:, 2]-R.Z_PROP)-R.SWEEP, 0)
        best = min(best, np.hypot(dr, dz).min())
    return best

def samples(a, b, step=2.0):
    a = np.asarray(a, float); b = np.asarray(b, float); n = max(2, int(np.linalg.norm(b-a)/step))
    return [a+(b-a)*q for q in np.linspace(0, 1, n)]

def upper(zs=60.0, cols='8', spokes='8', rh=40.0, zh=85.0, mid=None, diag=False):
    rods, printed = [], []
    zlev = zs if zs else 33.2
    if zs:
        rods += [(f, 'shoulder ring') for f in R.flats(zs)]
        printed += R.chamfer_links(zs)
        colset = {'8': R.CORN+R.TEES, 'corners': R.CORN, 'tees': R.TEES}[cols]
        for x, y in colset: printed.append(((x, y, 33.2), (x, y, zs), 8.0, 'column'))
        if diag:   # drum diagonals: each skid tee up to the two neighbouring shoulder corners
            for x, y in R.TEES:
                for cx, cy in R.CORN:
                    if math.hypot(cx-x, cy-y) < 160: rods.append(([(x, y, 33.2), (cx, cy, zs)], 'drum diag'))
    spk = {'8': R.TEES+R.CORN, 'corners': R.CORN, 'tees': R.TEES}[spokes]
    hub = {}; spoke_polys = []
    for x, y in spk:
        a = math.atan2(y, x); hp = (round(rh*math.cos(a), 3), round(rh*math.sin(a), 3), zh); hub[round(a, 4)] = hp
        p0 = (x, y, zlev)
        if mid:
            pm = tuple(np.round(np.add(p0, np.multiply(np.subtract(hp, p0), mid)), 3))
            spoke_polys.append(([p0, pm, hp], 'spoke'))
        else: spoke_polys.append(([p0, hp], 'spoke'))
    rods += spoke_polys
    if mid:
        ms = sorted([p[0][1] for p in spoke_polys], key=lambda q: math.atan2(q[1], q[0]))
        for i in range(len(ms)): rods.append(([ms[i], ms[(i+1) % len(ms)]], 'crown ring'))
    hp = [hub[k] for k in sorted(hub)]
    hub_len = 0.0
    if len(hp) > 1:
        for i in range(len(hp)):
            printed.append((hp[i], hp[(i+1) % len(hp)], 6.0, 'hub')); hub_len += math.dist(hp[i], hp[(i+1) % len(hp)])
    return rods, printed, hub_len

def build(**kw):
    r, p, sup = R.lower_half()
    ur, up, hl = upper(**kw)
    rods, printed = r+ur, p+up
    ends = {tuple(map(float, poly[0])) for poly, t in rods} | {tuple(map(float, poly[-1])) for poly, t in rods}
    pe = {tuple(map(float, a)) for a, b, sz, t in printed} | {tuple(map(float, b)) for a, b, sz, t in printed}
    thru = 0
    for poly, t in rods:
        for q in poly[1:-1]:
            q = tuple(map(float, q))
            if q in ends or q in pe: thru += 1
    cage = R.Cage(rods, printed, sup, hub_len=hl, thru=thru)
    hull = sorted({tuple(map(float, q)) for poly, t in rods if t != 'strut' for q in poly})
    inter = {((15.0, s*32.0, -47.0), (-5.0, s*32.0, -47.0)) for s in (1, -1)} | {((-5.0, s*32.0, -47.0), (-25.0, s*32.0, -47.0)) for s in (1, -1)}
    spans = [(a, b, t) for poly, t in rods if t != 'strut' for a, b in zip(poly[:-1], poly[1:])
             if t != 'sled rail']
    rod_pts = [q for poly, t in ur for a, b in zip(poly[:-1], poly[1:]) for q in samples(a, b)]
    pr_pts = [q for a, b, sz, t in up for q in samples(a, b)]
    return cage, hull, spans, disk_clear(rod_pts), (disk_clear(pr_pts) if pr_pts else 99)

def score(kw):
    try:
        cage, hull, spans, rc, pc = build(**kw)
        r = R.evaluate(cage, hull, spans, probe_r=19.0, n_dirs=160)
        r.pop('edge_list', None)
        r.update(kw=kw, rod_clear=round(rc, 1), printed_clear=round(pc, 1), mass_g=round(r['rod_g']+r['printed_g'], 1))
        return r
    except Exception as e:
        return dict(kw=kw, error=str(e)[:200])

GRID = dict(zs=[None, 58.0, 62.0, 66.0], cols=['8', 'corners', 'tees'], spokes=['8', 'corners', 'tees'],
            rh=[12.0, 24.0, 40.0], zh=[70.0, 78.0, 85.0], mid=[None, 0.5], diag=[False, True])

def grid():
    out = []
    for vals in itertools.product(*GRID.values()):
        kw = dict(zip(GRID, vals))
        if kw['zs'] is None and (kw['cols'] != '8' or kw['diag']): continue   # no shoulder ring: no columns, no diagonals
        out.append(kw)
    return out

if __name__ == '__main__':
    W = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    base = score(dict(zs=60.0, cols='8', spokes='8', rh=40.0, zh=85.0, mid=None, diag=False))
    print('v13', json.dumps(base, default=str), flush=True)
    G = grid(); print(len(G), 'layouts', flush=True)
    rows = []
    with mp.Pool(W) as p:
        for i, r in enumerate(p.imap_unordered(score, G, chunksize=4)):
            rows.append(r)
            if i % 50 == 0: print(i, flush=True); json.dump(dict(v13=base, rows=rows), open('rods14_search.json', 'w'), default=str)
    json.dump(dict(v13=base, rows=rows), open('rods14_search.json', 'w'), default=str)
    print('done', len(rows))
