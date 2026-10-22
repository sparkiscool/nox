"""Scout cage v6: structured dome cage. Builds rods/nodes, checks props, funnel, rests (facet method), mass.
Drone coords mm. Tags: C unless noted. Variant A = passive self-righting (tall), B = compact + 2-motor blip."""
import math, json, sys, numpy as np
from scipy.spatial import ConvexHull
G = np.array([0.57, 0.0, 2.79]); M_KG = 0.45                        # CG E, all-up E
MOT = [(54.1, 54.1), (-54.1, 54.1), (54.1, -54.1), (-54.1, -54.1)]
FEET = [(41, 30, -44.5), (41, -30, -44.5), (-44, 30, -44.5), (-44, -30, -44.5)]   # receiver v4 feet, DEFAULT
PACK = [(x, y, -44.5) for x in (-41.5, 37.5) for y in (-19, 19)]

def ring_pts(z, r_axis, r_diag):   # octagon: 4 on the axes, 4 on the diagonals
    d = r_diag / math.sqrt(2)
    return [(r_axis, 0, z), (d, d, z), (0, r_axis, z), (-d, d, z), (-r_axis, 0, z), (-d, -d, z), (0, -r_axis, z), (d, -d, z)]
R0 = [(112, 0, 33.2), (106, 106, 33.2), (0, 112, 33.2), (-106, 106, 33.2), (-112, 0, 33.2), (-106, -106, 33.2), (0, -112, 33.2), (106, -106, 33.2)]
R0_RING = [((100, 112, 33.2), (-100, 112, 33.2)), ((-112, 100, 33.2), (-112, -100, 33.2)), ((-100, -112, 33.2), (100, -112, 33.2)), ((112, -100, 33.2), (112, 100, 33.2))]
WHEELS = [(112, 100, 33.2), (112, -100, 33.2), (-112, 100, 33.2), (-112, -100, 33.2), (100, 112, 33.2), (-100, 112, 33.2), (100, -112, 33.2), (-100, -112, 33.2)]

VARIANTS = {
 'A passive': dict(levels=[(60, 110, 121), (95, 85, 99), (122, 45, 57)], apex=134),
 'B compact': dict(levels=[(60, 110, 121), (80, 80, 99), (95, 30, 42)], apex=None),
}

def build(v):
    rods, nodes = [], []
    levels = [R0] + [ring_pts(z, ra, rd) for z, ra, rd in v['levels']]
    for a, b in R0_RING: rods.append((a, b, 'wheel ring'))
    for k, L in enumerate(levels):
        nodes += L
        if k > 0:
            for i in range(8): rods.append((L[i], L[(i+1) % 8], f'ring {k}'))
        if k + 1 < len(levels):
            U = levels[k+1]
            for i in range(8):
                rods.append((L[i], U[i], f'spoke {k}-{k+1}'))
                if k > 0: rods.append((L[i], U[(i+1) % 8], f'diag {k}-{k+1}'))   # triangulate above the shoulder
    if v['apex']:
        ap = (0, 0, v['apex']); nodes.append(ap)
        for p in levels[-1]: rods.append((p, ap, 'apex spoke'))
    # nose ladder, skirt, struts (from v5)
    for y in (14, -14): rods.append(((112, y, 24.0), (112, y, 60.0), 'nose ladder'))
    for r in [(44, 28, -5), (-44, 28, -5), (-44, -28, -5), (44, -28, -5)]:
        rods.append(((0, 112 if r[1] > 0 else -112, 33.2), r, 'skirt'))
    for s in (1, -1): rods.append(((0, s*112, 33.2), (2, s*50, 24), 'side strut'))
    return rods, nodes

def prop_clear(rods):
    w, wt = 99, ''
    for a, b, t in rods:
        for i in range(101):
            u = i/100; p = [a[k] + (b[k]-a[k])*u for k in range(3)]
            for m in MOT:
                d = math.hypot(p[0]-m[0], p[1]-m[1]) - 44.45
                dz = 0 if 37.2 <= p[2] <= 45.2 else min(abs(p[2]-37.2), abs(p[2]-45.2))
                c = (dz if d < 0 else math.hypot(d, dz)) - 1.5
                if c < w: w, wt = c, t
    return round(w, 1), wt

def funnel_clear(nodes):
    worst = 99
    for x, y, z in nodes:
        if z > -0.8: worst = min(worst, (84 + z + 0.8) - max(abs(x), abs(y)))
    return round(worst, 1)

def rests(pts):
    P = np.array(pts, float); hull = ConvexHull(P); groups = {}
    for eq, s in zip(hull.equations, hull.simplices):
        groups.setdefault(tuple(np.round(eq, 2)), set()).update(s)
    out = []
    for key, verts in groups.items():
        n = np.array(key[:3]); d = key[3]; dist = -(n @ G + d); foot = G + n*dist
        V = P[sorted(verts)]
        basis = np.linalg.svd(V - V.mean(0))[2][:2]
        V2 = (V - foot) @ basis.T
        try: h2 = ConvexHull(V2)
        except Exception: continue
        # inside if the origin (foot) is strictly inside V2's hull; margin = min edge distance
        margin = min(-(e[2]) / np.hypot(e[0], e[1]) for e in h2.equations)   # distance from origin to each edge (positive inside)
        if margin > 0.3:
            out.append(dict(down=tuple(np.round(n, 2)), pe=round(dist, 1), tip_mm=round(margin, 1), tip_Nm=round(M_KG*9.81*margin/1000, 3)))
    return sorted(out, key=lambda r: r['pe'])

def roll_path(pts, axis='x'):   # PE while tilting from inverted about the y axis toward +x
    P = np.array(pts, float) - G; out = []
    for deg in range(0, 181, 5):
        a = math.radians(deg); u = np.array([math.sin(a), 0, math.cos(a)])   # down direction in body coords: 0 = inverted (+z down), 180 = upright
        out.append((deg, round(float((P @ u).max()), 1)))
    return out

if __name__ == '__main__':
    report = {}
    for name, v in VARIANTS.items():
        rods, nodes = build(v)
        L = [math.dist(a, b) for a, b, t in rods]
        pts = PACK + FEET + nodes + WHEELS + [(112, 14, 24), (112, -14, 24)]
        rs = rests(pts); path = roll_path(pts)
        pe = [p for _, p in path]; hump = max(pe[:-1]) ; mono = all(pe[i] >= pe[i+1] - 0.01 for i in range(len(pe)-1))
        cut = {}
        for (a, b, t), l in zip(rods, L): cut.setdefault(t, []).append(round(l, 1))
        n_nodes = len(nodes) + 2 + 4   # + ladder sleeves + receiver roots not counted; nodes list covers rings
        rod_g = sum(L)/1000*11.0; node_g = 4*4.3 + (len(nodes)-8)*1.4 + 4*2.0 + 2*0.7 + 2*1.86 + 4*0.5   # corner v4, dome nodes 1.4 E, R0 tees, ladder sleeves, hubs, cam clips
        print(f'== {name}: apex {v["apex"] or v["levels"][-1][0]}, height {(v["apex"] or v["levels"][-1][0]) + 44.5:.0f}; rods {len(rods)} = {sum(L)/1000:.2f} m {rod_g:.0f} g; nodes {len(nodes)+8} ~{node_g:.0f} g; cage ~{rod_g+node_g+3:.0f} g E')
        print('   longest span', round(max(L), 1), 'props', prop_clear(rods), 'funnel', funnel_clear(nodes))
        print('   stable rests (down vector in body, PE = CG height, tip margin):')
        for r in rs: print('     ', r)
        print('   roll inverted->+x: ', path[::4], ' monotone:', mono, ' PE inverted', pe[0], 'hump', hump)
        for k, vv in cut.items(): print(f'     {k:12s} x{len(vv)} {sorted(set(vv))}')
        report[name] = dict(rods=rods, nodes=nodes, rests=rs, path=path, cut=cut, rod_g=round(rod_g, 1), node_g=round(node_g, 1))
    json.dump(report, open('s21_cage6.json', 'w'), indent=1)
