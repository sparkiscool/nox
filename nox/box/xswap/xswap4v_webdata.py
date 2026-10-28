"""Full swap-station page data for one pincer version (P3 / P4) -> runs/xswap/v4p/<key>/data.js"""
import base64, json, math, os, sys
import numpy as np
import xswap4 as X, xswap4p as P
key = sys.argv[1]
OUT = os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", key); os.makedirs(OUT, exist_ok=True)
def pm(m):
    t = X.tm(m); v = np.asarray(t.vertices, np.float32); f = np.asarray(t.faces, np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(), nf=len(f))
def plan(m, z): return [[[round(a, 2), round(b, 2)] for a, b in p] for p in m.slice(z).to_polygons()]
def sect_x(m, x0):
    r = m.rotate([0, -90, 0]); return [[[round(py, 2), round(-px, 2)] for px, py in p] for p in r.slice(x0).to_polygons()]
g = P.G(P.VARIANTS[key]); flip = json.load(open(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "flip.json")))[key]
res = json.load(open(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "result.json")))
p0 = P.pincer(key, 0.0); sh = P.shell_body(key)
meshes = dict(holder=X.holder(), cap=X.xt60_cap(), male=X.xt60_male(), shell=sh, bar=X.bar_local(), keeper=X.xt60_keeper(),
              female=X.xt60_female(), pack=X.pack(), base=p0["base"], lid=p0["lid"], finger=P.finger_A(key, 0.0),
              steel=P.steel_pins_A(key, 0.0), pinion=P.pinion(key), servo=P.servo(key))
th_r = X.theta_from_u(X.STROKE, 0.0)
sec = {}
for st, th, u in (("latched", 0.0, 0.0), ("released", th_r, X.STROKE)):
    sec[st] = dict(holder=plan(meshes["holder"], -63.0), shell=plan(sh, -63.0), bar=plan(X.bar(th, "A"), -63.0),
                   finger=plan(P.finger_A(key, u), -63.0), swing=round(X.L_TIP * math.sin(math.radians(th)), 1),
                   spring=[[X.SPR_BOSS_X1 - X.SPR_POCKET, X.SPR_Y],
                           [round(-X.BODY_W / 2 * math.cos(math.radians(th)) - X.SPR_D * math.sin(math.radians(th)), 2),
                            round(X.PIV_Y - X.BODY_W / 2 * math.sin(math.radians(th)) + X.SPR_D * math.cos(math.radians(th)), 2)]])
post = dict(shell=sect_x(sh, 0.0), bar=sect_x(X.bar(0.0, "A"), 0.0), finger=sect_x(meshes["finger"], 0.0),
            steel=sect_x(meshes["steel"], 0.0), lid=sect_x(p0["lid"], 0.0), base=sect_x(p0["base"], 0.0),
            z_arm0=round(g.z_arm0, 2), yc=X.FIN_Y, lidz=round(g.z_lid0, 2))
data = dict(key=key, name=P.VARIANTS[key]["name"], meshes={k: pm(m) for k, m in meshes.items()},
            p=dict(YP=X.Y_P, ZAX=g.z_ax, D1=flip["rail before flip"], D2=flip["rail into bay"], PIV_Y=X.PIV_Y, FIN_A=X.FIN_A,
                   CLR=X.SLOT_CLR, STROKE=X.STROKE, R_P=X.R_P, TH_REL=X.TH_REL, TWIN=P.VARIANTS[key]["twin"]),
            sec=sec, post=post, safety=res[key]["safety"], mass=res[key]["mass_g"], printed=res[key]["pincer_printed_g"],
            size=res[key]["size_mm"], flip=flip, v4=res["v4"]["safety"])
s = "window.XSWAP=" + json.dumps(data, separators=(",", ":")) + ";"
open(os.path.join(OUT, "data.js"), "w").write(s); print(key, "kB", len(s) // 1024)
