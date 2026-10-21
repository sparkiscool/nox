"""Cage v10 'Lantern': structure first. Four elements built from one rod size and four node types.
Sled (landing ring, z -47) - Glove (8 skirt rods on the funnel's 45 deg) - Drum (skid ring z 33.2 + shoulder ring z 60 + 8 posts)
- Crown (8 spokes to one hub, z 85). Drone coords mm."""
import math, json, numpy as np, s21_cage6 as c
H,RT=85,40
hub=c.ring_pts(H,RT,RT)
CORN=[(106,106),(-106,106),(-106,-106),(106,-106)]      # corner nodes (chamfer midpoints)
TEES=[(112,0),(0,112),(-112,0),(0,-112)]
def clipped(z): return [((100,112,z),(-100,112,z)),((-112,100,z),(-112,-100,z)),((-100,-112,z),(100,-112,z)),((112,-100,z),(112,100,z))]
rods=[]; nodes={}
def N(name,p): nodes.setdefault(name,[]).append(p); return p
for a,b in clipped(33.2): rods.append((a,b,'skid ring'))
for a,b in clipped(60.0): rods.append((a,b,'shoulder ring'))
for x,y in CORN: N('corner',(x,y,33.2)); N('elbow',(x,y,60)); rods.append(((x,y,33.2),(x,y,60),'post'))
for x,y in TEES: N('tee',(x,y,33.2)); N('elbow',(x,y,60)); rods.append(((x,y,33.2),(x,y,60),'post'))
# crown: 8 spokes from the shoulder nodes to the hub (corner -> hub diagonal, tee -> hub axis)
for (x,y),i in zip(CORN,[1,3,5,7]): rods.append(((x,y,60),hub[i],'spoke'))
for (x,y),i in zip(TEES,[0,2,4,6]): rods.append(((x,y,60),hub[i],'spoke'))
N('hub',(0,0,H))
# sled
ZB,YB,XN,XT=-47,32,40,-50
for s in (1,-1):
    nb=N('sled corner',(XN,s*YB,ZB)); tb=N('sled corner',(XT,s*YB,ZB)); mb=N('rail sleeve',(-5,s*YB,ZB))
    rods.append((nb,tb,'sled rail'))
rods.append(((XT,YB,ZB),(XT,-YB,ZB),'sled cross'))
# glove: corners -> knees, tees (sides) -> rail sleeves
for s in (1,-1):
    nk=N('knee',(XN,s*YB,-7)); tk=N('knee',(-75,s*64,-0.8))
    rods.append(((106,s*106,33.2),nk,'glove nose')); rods.append((nk,(XN,s*YB,ZB),'nose leg'))
    rods.append(((-106,s*106,33.2),tk,'glove tail')); rods.append((tk,(XT,s*YB,ZB),'glove tail low'))
    rods.append(((0,s*112,33.2),(-5,s*YB,ZB),'glove side'))
# camera rails on the nose tee (bridge) and two struts to the plate's side bolts
for y in (14,-14): rods.append(((106,y,24),(106,y,60),'camera rail'))
for s in (1,-1): rods.append(((0,s*112,33.2),(2,s*50,24),'strut'))
if __name__=='__main__':
    L=[math.dist(a,b) for a,b,t in rods]
    cut={}
    for (a,b,t),l in zip(rods,L): cut.setdefault(t,[]).append(round(l,1))
    ng={'corner':5.0,'tee':3.0,'elbow':1.4,'hub':7.0,'sled corner':1.2,'rail sleeve':0.8,'knee':1.2}
    node_g=sum(len(v)*ng[k] for k,v in nodes.items())
    allpts=[p for v in nodes.values() for p in v]
    print(f'rods {len(rods)} {sum(L)/1000:.2f} m {sum(L)/1000*11:.0f} g; nodes {sum(len(v) for v in nodes.values())} ({ {k:len(v) for k,v in nodes.items()} }) ~{node_g:.0f} g; + 2 side hubs 3.7 + 4 camera clips 2 + epoxy 2 -> ~{sum(L)/1000*11+node_g+7.7:.0f} g E')
    print('props',c.prop_clear(rods))
    c5,s5=math.cos(math.radians(5)),math.sin(math.radians(5)); wo=99; wf=99
    for a,b,t in rods:
        for i in range(101):
            u=i/100; x,y,z=[a[k]+(b[k]-a[k])*u for k in range(3)]
            if z<=-0.8:
                for sg in (1,-1): wo=min(wo,84-max(abs(x*c5-sg*y*s5),abs(sg*x*s5+y*c5))-1.5)
            else: wf=min(wf,(84+z+0.8)-max(abs(x),abs(y))-1.5)
    print('collar face',round(wf,1),'yawed opening',round(wo,1))
    for k,v in cut.items(): print(f'  {k:16s} x{len(v)} {sorted(set(v))}')
    pts=c.PACK+allpts+[(106,14,24),(106,-14,24)]
    rs=[r for r in c.rests(pts) if r['tip_mm']>=8]
    for r in rs: print('  rest',tuple(float(x) for x in r['down']),'PE',float(r['pe']),'tip',float(r['tip_mm']),float(r['tip_Nm']))
    P=np.array(pts,float)-c.G; paths={}
    for ax,u in (('nose',(1,0)),('side',(0,1)),('diagonal',(0.7071,0.7071))):
        paths[ax]=[round(float((P@np.array([u[0]*math.sin(math.radians(d)),u[1]*math.sin(math.radians(d)),math.cos(math.radians(d))])).max()),1) for d in range(0,181,5)]
    json.dump(dict(rods=[(list(a),list(b),t) for a,b,t in rods],nodes={k:[list(p) for p in v] for k,v in nodes.items()},cut=cut,paths=paths),open('s21_lantern.json','w'))
    open('rods.js','w').write('const RODS='+json.dumps([[list(a),list(b)] for a,b,t in rods],separators=(',',':'))+';\nconst HUB='+json.dumps([[list(hub[i]),list(hub[(i+1)%8])] for i in range(8)],separators=(',',':'))+';\nconst NODES='+json.dumps([list(p) for v in nodes.values() for p in v],separators=(',',':'))+';\nconst PATHS='+json.dumps(paths,separators=(',',':'))+';\n')
