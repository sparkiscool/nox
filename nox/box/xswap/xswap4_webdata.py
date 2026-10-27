"""Mesh + section data for the v4 explainer page -> runs/xswap/v4/web/data.js"""
import base64, json, math, os
import numpy as np
import xswap4 as X

OUT = os.path.join(X.OUT, "web"); os.makedirs(OUT, exist_ok=True)

def pack_mesh(m):
    t = X.tm(m)
    v = np.asarray(t.vertices, np.float32); f = np.asarray(t.faces, np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(), nf=len(f))

def plan(m, z):
    return [[[round(x, 2), round(y, 2)] for x, y in p] for p in m.slice(z).to_polygons()]

th_r = X.theta_from_u(X.STROKE, 0.0)
meshes = dict(holder=X.holder(), cap=X.xt60_cap(), male=X.xt60_male(), shell=X.shell_body(), bar=X.bar_local(),
              keeper=X.xt60_keeper(), female=X.xt60_female(), pack=X.pack(), jaw=X.jaw_body(), finger=X.finger_A(0.0),
              pinion=X.pinion(), servo=X.servo_dummy())
data = dict(meshes={k: pack_mesh(m) for k, m in meshes.items()},
            p=dict(YP=X.Y_P, ZAX=X.Z_AX, D1=80.0, D2=80.0, PIV_Y=X.PIV_Y, FIN_A=X.FIN_A, CLR=X.SLOT_CLR, STROKE=X.STROKE,
                   R_P=X.R_P, TH_REL=X.TH_REL),
            sec={})
H, body = meshes["holder"], meshes["shell"]
for st, th, u in (("latched", 0.0, 0.0), ("released", th_r, X.STROKE)):
    data["sec"][st] = dict(holder=plan(H, -63.0), shell=plan(body, -63.0), bar=plan(X.bar(th, "A"), -63.0),
                           finger=plan(X.finger_A(u), -63.0), th=round(th, 2), swing=round(X.L_TIP * math.sin(math.radians(th)), 1),
                           spring=[[X.SPR_BOSS_X1 - X.SPR_POCKET, X.SPR_Y],
                                   [round(-X.BODY_W / 2 * math.cos(math.radians(th)) - X.SPR_D * math.sin(math.radians(th)), 2),
                                    round(X.PIV_Y - X.BODY_W / 2 * math.sin(math.radians(th)) + X.SPR_D * math.cos(math.radians(th)), 2)]])
s = "window.XSWAP=" + json.dumps(data, separators=(",", ":")) + ";"
open(os.path.join(OUT, "data.js"), "w").write(s)
print("data.js %.0f kB" % (len(s) / 1024), {k: v["nf"] for k, v in data["meshes"].items()})
