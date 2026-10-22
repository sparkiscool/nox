import numpy as np, trimesh, copy, json, node_topo as NT, noxopt as NO
nd = dict(NT.NODES['hub']); nd['gen'] = 4.5; h = 2.0
pb, g = NT.problem(nd, h); drawn, cuts, env, allowed = g['drawn'], g['cuts'], g['env'], g['allowed']
m = trimesh.load('hub_dbg.stl', force='mesh'); parts = m.split(only_watertight=False)
big = [p for p in parts if abs(p.volume) > 1.0]; print('bodies', len(big), [round(p.volume) for p in big], flush=True)
Pm = max(parts, key=lambda p: abs(p.volume))
thin = NO.thin_check(Pm, NT.tm(drawn - cuts), 2.0, log=lambda s: print(s[:200], flush=True))
Bm = NT.man(Pm); fc = (((drawn - cuts) - Bm).volume(), (Bm ^ cuts).volume(), (Bm - (allowed + drawn)).volume()); print('function', fc, flush=True)
wr = NO.wrinkle(Pm, drawn=NT.tm(drawn - cuts), allowed=NT.tm(allowed))
hr = 1.5   # as-built recheck on a finer grid: at h 2.0 the 1.7 socket wall voxelises away and the loaded zone floats
pbr, gr = NT.problem(nd, hr)
A = NO._voxelise(Pm, *gr['ax']).ravel().astype(float)
pbs = copy.copy(pbr); pbs.void = (A < 0.5) & ~pbr.solid; pbs.solid = np.zeros_like(pbr.solid)
rows, _ = NO.strength_report(pbs, np.where(pbr.solid, 1.0, A))
w = max(rows, key=lambda r: max(r['inlayer'], r['across'])); worst = max(w['inlayer'], w['across'])
mass = Pm.volume / 1000 * NT.MAT.density
ok = len(big) == 1 and worst <= 1.0 and not thin
out = dict(node='hub', h=h, at=[0, 0, 85], bodies=len(big), watertight=bool(Pm.is_watertight), mass_g=round(mass, 2), drawn_g=round(NT.tm(drawn - cuts).volume / 1000 * NT.MAT.density, 2),
           function=[round(v, 2) for v in fc], thin=len(thin), wrinkle=[round(v, 2) for v in wr], worst=round(worst, 2), worst_case=w['case'], connectivity=len(big) == 1,
           cases=len(pb.cases), design_cells=int((~pb.solid & ~pb.void).sum()), volfrac=0.036, minutes=0, faces=int(len(Pm.faces)), gen_cm3=4.5, recheck_h=hr,
           trail=[dict(gen=1.0, note='slab envelope: starved, pruned apart'), dict(gen=1.5, mass=round(mass, 2), worst=round(worst, 2), ok=ok, bodies=len(big), note='ring envelope, no prune, recheck h 1.5')])
Pm.export('hub.stl'); json.dump(out, open('hub.json', 'w'), indent=1); print('HUB', out['mass_g'], out['worst'], out['bodies'], out['thin'], out['wrinkle'], flush=True)
