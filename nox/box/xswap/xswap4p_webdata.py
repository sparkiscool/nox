"""Meshes + numbers for the pincer-options page -> runs/xswap/v4p/web/data.js"""
import base64, json, math, os
import numpy as np
import xswap4 as X, xswap4p as P
OUT = os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "web"); os.makedirs(OUT, exist_ok=True)
def pm(m):
    t = X.tm(m); v = np.asarray(t.vertices, np.float32); f = np.asarray(t.faces, np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(), nf=len(f))
crop = X.box(-60, 60, -100, 110, X.SZ0 - 1, -50.0)
res = json.load(open(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "result.json")))
data = dict(common=dict(barA=pm(X.bar(0.0, "A")), barB=pm(X.bar(0.0, "B"))), variants={},
            p=dict(YP=X.Y_P, PIV_A=X.PIV_Y, PIV_B=2 * X.Q_Y - X.PIV_Y, FIN_A=X.FIN_A, CLR=X.SLOT_CLR, STROKE=X.STROKE, R_P=X.R_P, TH_REL=X.TH_REL),
            results=res)
for k, c in P.VARIANTS.items():
    p0 = P.pincer(k, 0.0)
    d = {n: pm(m) for n, m in p0.items()}
    d["shell"] = pm(P.shell_body(k) ^ crop)
    data["variants"][k] = dict(name=c["name"], cfg=c, meshes=d)
    print(k, {n: m["nf"] for n, m in d.items()})
s = "window.PX=" + json.dumps(data, separators=(",", ":")) + ";"
open(os.path.join(OUT, "data.js"), "w").write(s); print("kB", len(s) // 1024)
