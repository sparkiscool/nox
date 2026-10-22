import sys, math
sys.path.insert(0,'.')
import s21_lantern12 as L, s21_cage6 as c
from rest_probe14 import base, feet
hub=[tuple(p) for p in L.hub]
def report(name,pts):
    rs=[r for r in c.rests(pts) if r['down'][2]>0.05]
    real=[r for r in rs if r['tip_mm']>=5]
    print(f"{name}: upside-down/side-top rests {len(rs)}, with tip margin >=5 mm: {len(real)}", [(r['down'],r['pe'],r['tip_mm']) for r in real[:5]])
report('v13 flat hub', c.PACK+base+hub+feet)
for n in (8,16,24):
  for zc,apz in ((40,193),(60,210)):
    R=apz-zc; pts=[(0,0,apz)]
    for a1 in range(10,90,12):
        a=math.radians(a1); r=R*math.sin(a); z=zc+R*math.cos(a)
        if z<62: break
        pts+=[(r*math.cos(k*2*math.pi/n),r*math.sin(k*2*math.pi/n),z) for k in range(n)]
    report(f'dome {n}-fold apex z {apz} ({len(pts)} top nodes)', c.PACK+base+feet+pts)
