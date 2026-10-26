"""Mesh + section data for the xswap explainer page -> runs/xswap/v3/web/data.js"""
import base64, json, os
import numpy as np
import xswap as X

YC = X.Y_C
OUT = os.path.join(X.OUT, "web"); os.makedirs(OUT, exist_ok=True)

def pack_mesh(m, dec=None):
    t = X.tm(m)
    if dec and len(t.faces) > dec:
        try: t = t.simplify_quadric_decimation(face_count=dec)
        except Exception: pass
    v = np.asarray(t.vertices, np.float32); f = np.asarray(t.faces, np.uint32)
    return dict(v=base64.b64encode(v.tobytes()).decode(), f=base64.b64encode(f.tobytes()).decode(), nv=len(v), nf=len(f))

def sect(m, y):
    r = m.rotate([90, 0, 0]); cs = r.slice(y)
    return [[[round(px, 2), round(-py, 2)] for px, py in poly] for poly in cs.to_polygons()]

pin_o = X.pincer(YC, X.FACE_OPEN)
meshes = dict(
    holder=X.holder(YC), cap=X.xt60_cap(), male=X.xt60_male(),
    shell=X.shell_body(YC), rocker_p=X.rocker(YC, 0, 1), rocker_m=X.rocker(YC, 0, -1), keeper=X.xt60_keeper(), female=X.xt60_female(), pack=X.pack(),
    jaw=pin_o["jaw"], finger_p=pin_o["finger_p"], finger_m=pin_o["finger_m"], pinion=pin_o["pinion"], servo=pin_o["servo"])
data = dict(meshes={k: pack_mesh(m) for k, m in meshes.items()},
            p=dict(YC=YC, ZA=X.Z_AX, D1=85.0, D2=75.0, PX=X.PX, PZ=X.PZ, TH=X.TH_OPEN, SLOT_LX=X.SLOT_LX, ZN=X.Z_N,
                   NECK=X.NECK, FACE_OPEN=X.FACE_OPEN),
            sec={})
pin_s = X.pincer(YC, X.FACE_SHUT)
for st, sh, p in (("latched", X.shell(YC, 0.0), pin_o), ("pinched", X.shell(YC, X.TH_OPEN), pin_s)):
    data["sec"][st] = dict(shell=sect(sh, YC), holder=sect(meshes["holder"], YC), pack=sect(meshes["pack"], YC),
                           finger=sect(p["finger_p"], YC) + sect(p["finger_m"], YC))
s = "window.XSWAP=" + json.dumps(data, separators=(",", ":")) + ";"
open(os.path.join(OUT, "data.js"), "w").write(s)
print("data.js %.0f kB" % (len(s) / 1024), {k: v["nf"] for k, v in data["meshes"].items()})
