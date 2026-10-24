"""Belly post pad search on the Scout v3 system (24 Oct): every pad spot on the belly vs the funnel planes, the post r >= 101,
the post landing cones vs the latch receiver / cartridge / Radxa / flow / mount legs and vs the keel the cage needs.
python3 nox/scout/v3/pad_search.py    (result: 0 spots, layout.md "Belly post")"""
import sys, numpy as np, trimesh, itertools
sys.path.insert(0,__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..', '..'))
src=open(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'layout_v2.py')).read()
src=src[:src.index('# ================================================================ run:')]
g={'__file__':__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'layout_v2.py'),'__name__':'x'}
exec(compile(src,'l2','exec'),g)
pts, box, face_margin, cone_gap, union = g['pts'], g['box'], g['face_margin'], g['cone_gap'], g['union']
S2=2**0.5
base = np.vstack([pts(g['IFACE'],6000), pts(g['CART'],4000), pts(g['RADXA'],600), pts(g['FLOW'],400)] + [pts(l,300) for l in g['LEGS']])
keel_nodes = [np.array([x,y,-47.5]) for x in (-28,0,28) for y in (-28.5,28.5)]
rails = []
for y in (-28.5, 28.5):
    for x0,x1 in ((-28,0),(0,28)):
        t=np.linspace(0,1,30)[:,None]; rails.append(np.array([x0,y,-47.5])*(1-t)+np.array([x1,y,-47.5])*t)
keel = np.vstack(rails + [n + 4*np.array(d) for n in keel_nodes for d in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1))])
def pads_side(xp, yp, zp, s):
    # mirror of layout_v2.pads to either side; carrier from |y| 26 out
    ya = abs(yp)
    top = zp + 4.0 + (ya + 7.0 - 26.0)
    sec = [(26.0, zp), (ya + 3.0, zp), (ya + 7.0, zp + 4.0), (26.0, top)]
    carrier = trimesh.Trimesh(np.array([[xp + x, s*y, z] for (y, z) in sec for x in (-18.0, 18.0)])).convex_hull
    pad = [box(xp + sx*12 - 5, xp + sx*12 + 5, s*ya - 3 if s>0 else -ya-3, s*ya + 3 if s>0 else -ya+3, zp + 1.0 - 1.6, zp + 1.0) for sx in (-1, 1)]
    zf = zp - 0.6
    body = box(xp - 21.0, xp + 21.0, s*ya - 5.0, s*ya + 5.0, zf - 2.0 - 10.0, zf - 2.0)
    tips = np.array([(xp + sx*12 + dx, s*ya, zf + 8.0) for sx in (-1, 1) for dx in (-2.0, 2.0)] + [(xp, s*ya, zf + 3.0)])
    return pad, carrier, body, tips
ok=[]; cnt={'fm':0,'r':0,'cb':0,'ck':0}; best={}
for s in (1,-1):
  for xp in range(-40, 55, 5):
    for ya in np.arange(26.0, 46.0, 1.0):
      for zp in np.arange(-45.0, -19.0, 1.0):
        pad, carrier, body, tips = pads_side(xp, s*ya, zp, s)
        fm = float(face_margin(np.vstack([carrier.vertices] + [p.vertices for p in pad])).min())
        if fm < 1.0: cnt['fm']+=1; continue
        rmin = float(np.hypot(body.vertices[:,1], body.vertices[:,2]+144.5).min())
        if rmin < 101.0: cnt['r']+=1; continue
        pp = body.vertices
        cb = min(cone_gap(tips, base), cone_gap(pp, base))
        if cb < 0: cnt['cb']+=1; continue
        ck = min(cone_gap(tips, keel), cone_gap(pp, keel))
        if ck < 1.5/S2 + 0.3:
            cnt['ck']+=1
            if ck > best.get('ck',(-99,))[0]: best['ck']=(ck, s, xp, ya, zp)
            continue
        # carrier and pads clear of receiver / cartridge / Radxa / flow / legs (simple: box distance of carrier verts)
        ok.append((s, xp, ya, zp, round(fm,2), round(rmin,1), round(cb,2), round(ck,2)))
print(len(ok), cnt, best); 
for o in ok[:40]: print(o)
