"""Export v13 and the v14 rod layout (geometry + frame-model scores) for the comparison page. Run from nox/v2/lantern."""
import json, math, numpy as np, rods14_search as S, rods14 as R, s21_cage6 as c
V13 = dict(zs=60.0, cols='8', spokes='8', rh=40.0, zh=85.0, mid=None, diag=False)
V14 = dict(zs=60.0, cols='corners+nose', spokes='corners', rh=40.0, zh=85.0, mid=0.4, diag=False)   # DEFAULT (r3: shoulder ring held at z 60 by the 90 deg wedge check; crown z 85 keeps v13's recovery climb)
FEET = [(50, 39, -52), (50, -39, -52), (-50, 39, -52), (-50, -39, -52)]
def climb(kw):
    cage, h, sp, rc, pc = S.build(**kw); P = np.vstack([np.array(h), np.array(c.PACK+FEET, float)])-c.G
    top = (P@np.array([0, 0, 1.0])).max()
    return min(max((P@np.array([u[0]*math.sin(math.radians(d)), u[1]*math.sin(math.radians(d)), math.cos(math.radians(d))])).max()
                   for d in range(0, 181, 2))-top for u in ((1, 0), (0, 1)))
out = {}
for name, kw in (('v13', V13), ('v14', V14)):
    r = S.score(kw); cage, h, sp, rc, pc = S.build(**kw)
    rods = [[list(map(float, a)), list(map(float, b)), t] for poly, t in cage.rods for a, b in zip(poly[:-1], poly[1:])]
    pr = [[list(map(float, a)), list(map(float, b)), sz, t] for a, b, sz, t in cage.printed]
    cl = climb(kw)
    out[name] = dict(kw=kw, rods=rods, printed=pr, score={k: r[k] for k in ('bow_clear', 'wedge_tip_r', 'mass_g', 'rod_g', 'printed_g', 'wall_J', 'wall_v', 'edge_min_J', 'edge_worst', 'probe_fail', 'probe_tested', 'prop_hull_clear', 'rod_clear', 'rods', 'rod_m', 'sockets')},
                     climb_mm=round(cl, 1), climb_J=round(0.52*9.81*cl/1000, 2), top_z=max(p[2] for a in rods for p in a[:2]))
json.dump(out, open('rods14_export.json', 'w'), default=float, indent=1)
for k, v in out.items(): print(k, v['score'], v['climb_mm'], v['climb_J'], v['top_z'])
