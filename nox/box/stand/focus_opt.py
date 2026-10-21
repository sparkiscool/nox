import itertools, json, numpy as np
from multiprocessing import Pool
from stand_model import configs, static_report
from stand_opt import layout_feet, cost
from fine_check import fine
m,s,_,_=configs()
cands=[]
for xb,yb,xf,yf in itertools.product((120,140,160),(-64,),(120,140,160),(200,225,250,275,286)):
    f=layout_feet("quad",xb,yb,xf,yf); r=static_report(np.array(f,float),m,s)
    cands.append(dict(p=(xb,yb,xf,yf),feet=f,cost=cost(f),tilt=float(r["tilt"].min()),push=float(r["push"].min())))
cands.sort(key=lambda c:c["cost"])
def run(c):
    r=fine(c["feet"],hs=(100,),mus=(0.05,))
    c["rock"]=max(x["rock"] for x in r); c["walk"]=max(x["walk"] for x in r); return c
with Pool(2) as P:
    res=[]
    for c in P.imap(run,cands):
        res.append(c); print(c["p"],round(c["cost"],1),"g rock",round(c["rock"],2),"walk",round(c["walk"],2),flush=True)
json.dump(res,open("focus.json","w"),default=float)
