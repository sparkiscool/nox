"""Box stand v4 (wood): geometry in W (X right, Y along the axle away from the board, Z up), checks, JSON for the layout page."""
import numpy as np, json, trimesh
c45=np.sqrt(.5)
T=18.0            # board thickness (E)
BASE_TOP=-152.0   # base top Z: bar sweep r 136.5 + 15.5 clear
SWEEP=136.5; HEADS=152.5   # bar sweep, heads + back plate (C / E)
def prism2d(poly, y0, y1):  # poly in XZ, extruded along Y
    p=np.array(poly,float); n=len(p)
    v=np.vstack([np.c_[p[:,0],np.full(n,y0),p[:,1]], np.c_[p[:,0],np.full(n,y1),p[:,1]]])
    return v
def rot(th): return np.array([np.cos(th),np.sin(th)])
parts={}
parts['upright']=dict(v=prism2d([(-250,BASE_TOP),(250,BASE_TOP),(250,250),(-250,250)],-T,0),kind='wood',label='Upright 500 x 402')
parts['base']=dict(v=prism2d([(-250,BASE_TOP-T),(250,BASE_TOP-T),(250,BASE_TOP),(-250,BASE_TOP)],-T,300-T),kind='wood',label='Base 500 x 300')
# funnel blocks: 2x4 (38 x 89), 89 face on the funnel side panel outer surface (through the collar seat), 45 deg
s=np.array([98.14,151.46]); u=np.array([c45,c45]); n=np.array([c45,-c45])
for sgn,nm in [(1,'funnel_block_R'),(-1,'funnel_block_L')]:
    S=np.array([1,1])*[sgn,1]; uu=u*[sgn,1]; nn=n*[sgn,1]; ss=s*[sgn,1]
    poly=[ss-10*uu, ss+79*uu, ss+79*uu+38*nn, ss-10*uu+38*nn]
    parts[nm]=dict(v=prism2d(poly,0,230),kind='wood',label='Funnel block 2x4 x 230')
# bay block +15: face on the +15 deg plane, 38 thick toward +angle, r 155..244
d=rot(np.radians(15)); m=rot(np.radians(105))
poly=[d*155+m*0.3, d*244+m*0.3, d*244+m*38.3, d*155+m*38.3]
parts['bay_block_p15']=dict(v=prism2d(poly,0,230),kind='wood',label='Bay block +15, 2x4 x 230')
# bay wedge -45: 45 deg face on the -45 plane r 158..200, down to the base
mm=rot(np.radians(-135)); a=rot(np.radians(-45))*158+mm*0.3; b=rot(np.radians(-45))*200+mm*0.3
poly=[a,b,(b[0],BASE_TOP),(a[0],BASE_TOP)]
parts['bay_wedge_m45']=dict(v=prism2d(poly,135,220),kind='wood',label='Bay wedge -45, 85 long')
# checks
def rmin(v): return np.hypot(v[:,0],v[:,2]).min()
def edge_samples(v):
    n=len(v)//2; pts=[]
    for k in (0,n):
        q=v[k:k+n]
        for i in range(n):
            A,B=q[i],q[(i+1)%n]
            for t in np.linspace(0,1,40): pts.append(A+(B-A)*t)
    return np.array(pts)
def ang(p): return np.degrees(np.arctan2(p[:,2],p[:,0]))
print("piece            rmin   minX(|X|) above Z102  in bay sectors?")
for k,p in parts.items():
    if k in('upright','base'): continue
    e=edge_samples(p['v']); r=np.hypot(e[:,0],e[:,2]); A=ang(e)
    d15=e[:,0]*np.sin(np.radians(15))*-1+e[:,2]*np.cos(np.radians(15))   # + = past the +15 plane
    d45=e[:,0]*np.sin(np.radians(45))+e[:,2]*np.cos(np.radians(45))      # - = past the -45 plane
    inbay=((d15<0.29)&(d45>-0.29)&(r<180)&(r>99)).any()
    hi=e[e[:,2]>102]; ox=np.abs(hi[:,0]).min() if len(hi) else None
    print(f"{k:16s} {r.min():6.1f}  {ox if ox is None else round(ox,1)}  {inbay}")
print("base top", BASE_TOP, "clear to sweep", -SWEEP-BASE_TOP)
# funnel interior check: frustum, inner faces through (84, 143.6) at 45 deg, mouth Z 219.6, centre Y 177
for k in ('funnel_block_R','funnel_block_L'):
    e=edge_samples(parts[k]['v']); 
    w=84+(e[:,2]-143.6); inside=(np.abs(e[:,0])<w+4.4-0.5)&(np.abs(e[:,1]-177)<w)&(e[:,2]>143.6)&(e[:,2]<219.6)
    print(k,"points inside funnel panel outer surface:",int(inside.sum()))
json.dump({k:dict(v=np.round(p['v'],2).tolist(),kind=p['kind'],label=p['label']) for k,p in parts.items()},open('stand_v4_parts.json','w'))
