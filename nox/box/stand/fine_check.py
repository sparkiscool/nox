import itertools, json, sys, numpy as np
from stand_opt import layout_feet
from stand_landing import build, drop
def fine(feet, offs=(-30,-20,-10,0,10,20,30), hs=(50,100), mus=(0.05,0.4)):
    out=[]
    for mu in mus:
        m=build(feet,mu_face=mu)[0]
        for dx,dy in itertools.product(offs,offs):
            n=np.hypot(dx,dy); dr=(0,0) if n==0 else (0.15*dx/n,0.15*dy/n)
            for h in hs:
                l,t,s,se=drop(m,dx=dx,dy=dy,h=h,drift=dr,yaw=5)
                out.append(dict(mu=mu,dx=dx,dy=dy,h=h,rock=t,walk=s,lift=l,seated=se))
    return out
if __name__=="__main__":
    f=json.loads(sys.argv[1]); r=fine(layout_feet(*f))
    w=max(r,key=lambda x:x["rock"]); v=max(r,key=lambda x:x["walk"])
    print(f,"max rock",round(w["rock"],2),w,"\n   max walk",round(v["walk"],2),v,"\n   unseated (mu .05):",sum(1 for x in r if x["mu"]<.1 and not x["seated"]))
    json.dump(r,open("fine_%s.json"%"_".join(map(str,f[1:])),"w"))
