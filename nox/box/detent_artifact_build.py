"""Builds the detent artifact's data (17 Oct 2026): decimated meshes (arm frame / W), the solved kinematics from
detent_kin.py and a per-angle clearance series, written as one JSON blob the page inlines.
python3 detent_artifact_build.py -> out/detent_artifact_data.json (then the page template inlines it)."""
import json, math, base64
import numpy as np, trimesh, fast_simplification as fsimp
import manifold3d as mf
import arm_layout as L, arm_cad as A, detent as DT, detent_kin as K

def dec(m, target):
    m = m.copy(); m.merge_vertices()
    if len(m.faces) > target:
        v, f = fsimp.simplify(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.uint32), target_count=target)
        m = trimesh.Trimesh(v, f, process=True)
    return m

def pack(m, crease=35.0):
    """crease-split smooth normals; positions float32, normals int8, indices uint32 (base64)."""
    s = trimesh.graph.smooth_shade(m, angle=math.radians(crease))
    v = np.asarray(s.vertices, np.float32); n = np.round(np.asarray(s.vertex_normals) * 127).astype(np.int8)
    f = np.asarray(s.faces, np.uint32).ravel()
    b = lambda a: base64.b64encode(a.tobytes()).decode()
    return dict(p=b(v), n=b(n), i=b(f), nv=len(v), nf=len(s.faces))

tm = A.tm
c = DT.CHOSEN; D = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); S = DT.arm_solids(D, np.array(c["Q"]))
M = L.MOTORS; parts = {}
# ---- fixed (W)
parts["ring"] = dict(g="fixed", m=tm(DT.ring(c["th"])))
SB = A.axle_sun_bodies()
parts["p80"] = dict(g="fixed", m=tm(SB["p80"]))
parts["post"] = dict(g="fixed", m=tm(A.sun_post() ^ A.box(-40, 40, 44, 70, -40, 40)))
# ---- arm frame (= W at phi 90)
parts["hub"] = dict(g="arm", m=dec(trimesh.load("out/box_hub_v5.stl", force="mesh"), 36000))
mot = None
for nm, Mi in M.items():
    mm = A.rsquare(Mi, L.NEMA / 2, 2.0, L.Y_PLATE0)
    mot = mm if mot is None else mot + mm
parts["motors"] = dict(g="arm", m=tm(mot))
ybelt = {"A": L.Y_HBELT, "B": L.Y_HBELT_B, "axle": L.Y_AXBELT}
pul = None
for nm, Mi in M.items():
    y0, y1 = ybelt[nm][0] - 2, ybelt[nm][1] + 2
    pp = A.cylY(Mi["xc"], Mi["zc"], y0, y1, L.P20_FL / 2) + A.cylY(Mi["xc"], Mi["zc"], L.Y_PLATE0, y1, 2.5)
    pul = pp if pul is None else pul + pp
parts["pulleys"] = dict(g="arm", m=tm(pul))
parts["loop"] = dict(g="arm", m=tm(SB["loop"]))
r = L.P20_OD / 2; belts = None
for h, xa, zm, zi, yb, rot in (("A", L.X_HBELT_C, L.Z_MOTOR_A, L.Z_IDLER, L.Y_HBELT, 0), ("B", L.X_HBELT_C_B, L.Z_MOTOR_B, L.Z_IDLER_B, L.Y_HBELT_B, -90)):
    o = (A.cylY(xa, zm, yb[0], yb[1], r + 1.2) + A.cylY(xa, zi, yb[0], yb[1], r + 1.2)).hull()
    i = (A.cylY(xa, zm, yb[0] - 1, yb[1] + 1, r) + A.cylY(xa, zi, yb[0] - 1, yb[1] + 1, r)).hull()
    b = (o - i).rotate([0, rot, 0]); belts = b if belts is None else belts + b
parts["belts"] = dict(g="arm", m=tm(belts))
parts["bars"] = dict(g="arm", m=tm(A.bar_2040(*L.BAR_A) + A.bar_2040(*L.BAR_B).rotate([0, -90, 0])))
parts["anchor"] = dict(g="arm", m=tm(S["anchor"]))
# ---- roller arm, swings about P (arm frame)
parts["rarm"] = dict(g="pivot", m=tm(S["arm"] + S["spacer"] + S["hook"]))
parts["roller"] = dict(g="pivot", m=tm(S["roller"] + S["nut"]))
out = dict(parts={k: dict(g=v["g"], **pack(v["m"], 30.0 if k != "hub" else 40.0)) for k, v in parts.items()})
# ---- kinematics + clearance series (1 deg)
kin = json.load(open("out/detent_kin.json"))
phis = np.array(kin["phi"]); psis = np.radians(np.array(kin["psi_deg"]))
import arm_kinematics as KM
PARTS = KM.build_parts("axle")
fixed = {nm: A.man(PARTS[nm].mesh) for nm in ("detent_ring", "p80", "post")}
armfix = {"hub": A.man(PARTS["hub"].mesh), "motors": mot, "pulleys": pul, "loop": SB["loop"]}
ser = []
for p_ in np.arange(-278, 144, 1.0):
    s = float(np.interp(p_, phis, psis)); mv = K.moving(s); g = 9.0; who = ""
    for a_, ma in mv.items():
        for b_, mb in armfix.items():
            if a_ == "springs" and b_ == "hub": pass
            gg = ma.min_gap(mb, 9.0)
            if gg < g: g, who = gg, f"{a_} / {b_}"
        if a_ != "springs":
            mw = K.to_w(ma, p_)
            for b_, mb in fixed.items():
                if a_ == "roller" and b_ == "detent_ring": continue
                gg = mw.min_gap(mb, 9.0)
                if gg < g: g, who = gg, f"{a_} / {b_}"
    ser.append([float(p_), round(g, 2), who])
nice = {"arm": "roller arm", "roller": "roller", "springs": "springs", "hub": "hub v5", "detent_ring": "detent ring",
        "p80": "80T sun", "post": "sun post", "motors": "motors", "pulleys": "pulleys", "loop": "axle loop"}
out["kin"] = dict(phi=kin["phi"], psi=kin["psi_deg"], lift=kin["lift"], spring=kin["spring"], torque=kin["torque"],
                  notches=kin["notches"], stops=kin["stops"], P=kin["P"], H=kin["H"], A=kin["A"], C=kin["C"],
                  clear=[[a, b, " / ".join(nice.get(x, x) for x in w.split(" / "))] for a, b, w in ser],
                  tower_fixed=kin["tower_fixed"], roller_ring=kin["roller_ring"], closest=kin["closest"],
                  spring_y=list(DT.SPRING_Y), spring_free=DT.SPRING_FREE, spring_lmax=DT.SPRING_LMAX,
                  dist=DT.disturbance())
json.dump(out, open("out/detent_artifact_data.json", "w"), separators=(",", ":"))
print({k: (v["nv"], v["nf"]) for k, v in out["parts"].items()}, "clear min", min(x[1] for x in ser))
