"""Rest probe (22 Oct): does a rounded hub cap or a taller dome remove the upside-down rest on cage v13? Run from nox/v2/lantern.
Answer (E, rod-node hull, CG (0.57, 0, 2.79) E): no for the cap; only a dome to apex z ~210 (cage ~262 tall, ~57 top nodes) clears every rest with >=5 mm tip margin."""
import sys, math, numpy as np
sys.path.insert(0,'.')
import s21_lantern12 as L, s21_cage6 as c
from scipy.spatial import cKDTree
G=c.G
base=[p for k,v in L.nodes.items() if k!='hub' for p in v]
feet=[(50,39,-52),(50,-39,-52),(-50,39,-52),(-50,-39,-52)]
def dirs(n=40000):
    i=np.arange(n)+0.5; phi=np.arccos(1-2*i/n); th=math.pi*(1+5**0.5)*i
    return np.c_[np.cos(th)*np.sin(phi),np.sin(th)*np.sin(phi),np.cos(phi)]
U=dirs(); T=cKDTree(U); NB=T.query(U,9)[1]
def rests(P):
    P=np.asarray(P,float)-G; h=(U@P.T).max(1)
    mins=[i for i in range(len(U)) if h[i]<=h[NB[i,1:]].min()+1e-9]
    # merge minima closer than 3 deg
    out=[]
    for i in sorted(mins,key=lambda i:h[i]):
        if all(np.degrees(np.arccos(np.clip(U[i]@U[j],-1,1)))>3 for j,_ in out): out.append((i,h[i]))
    return [(np.round(U[i],2),round(float(v),1)) for i,v in out]
def dome(zc,R,levels=(0.45,0.75)):
    # points on sphere centre (0,0,zc) radius R above the shoulder ring; rings of 8 at polar angles, plus apex
    pts=[(0,0,zc+R)]
    for f in levels:
        a=f*math.radians(90); r=R*math.sin(a) ; z=zc+R*math.cos(a)
        pts+=[(r*math.cos(k*math.pi/4),r*math.sin(k*math.pi/4),z) for k in range(8)]
    return pts
hub=[tuple(p) for p in L.hub]
print('v13 (flat hub):'); [print('  ',r) for r in rests(c.PACK+base+hub+feet)]
cap=[(0,0,85+6)]+[(30*math.cos(k*math.pi/4),30*math.sin(k*math.pi/4),85) for k in range(8)]
print('v13 + R40 cap on hub:'); [print('  ',r) for r in rests(c.PACK+base+cap+feet)]
print('sphere tops (centre above CG, enclosing the drum corners r150 z60):')
for zc in (0,10,20,30):
    R=math.hypot(150,60-zc)+2
    pts=c.PACK+base+feet+dome(zc,R,(0.3,0.6))
    rs=rests(pts); print(f'  zc {zc} R {R:.0f} apex z {zc+R:.0f}: {len(rs)} rests', rs[:4])
