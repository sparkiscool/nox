"""Whole box assembly (24 Oct 2026, lead session of the XT60 / hook latch round): every box part in W, the arm through one
swap cycle, the drone landing / latched / leaving, as one motion artifact (`nox/motion.py`).

W coords (box-arm.md): Y along the axle, Z up at the drone position, X tangential. Seat frame (funnel_panel.py, panel
convention: bottom edge z 1.3) -> W: X = drone y, Y = 177 - drone x, Z = 142.3 + z (collar on Y 177, box-case.md
Problem 5; seat z 0 = W Z 144.5 in the dimensions convention, bottom edge -0.8 -> 143.7 = 142.3 + 1.3 + 0.1, E).

Sources, each the part's own script or print file (nothing redrawn here):
  stand (mast, cradle_keel, frames mid / ends, kb, toe, bar halves, posts, brackets): stand/stand_v6_parts.py,
    stand/stand_v7_parts.py BUILD (drawn shapes; bar_half v8 / bracket_jA v8 are topology prints of the same envelope)
  bay modules: models/box_baymod_v4i_W.obj until bay module v5 is on main (then its _W mesh)
  funnel collar: catch_v4/funnel_panel_v7.py when on main (v7 latched faces + v7 plain), else funnel_panel.collar_world (v6)
  hook latch: hook_latch.py (VER), placed by its unit_frame()
  arm: arm_kinematics.py (build_parts + transforms) as the arm session leaves it on main
  electronics: box_electronics.py result.json board rectangles on the rear sheet
python3 nox/box/box_assembly.py [--video]  -> runs/box_assembly/v1/motion.html (+ mp4 / gif / png)"""
import json, math, os, sys, importlib
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(ROOT, "nox"), HERE, os.path.join(HERE, "stand"), os.path.join(HERE, "catch_v4")]
OUT = os.path.join(ROOT, "runs", "box_assembly", "v1")
Y_COLLAR, Z_SEAT = 177.0, 142.3                    # collar centre on the axle line, seat (panel frame) z 0 in W (C / E)


def tm(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


S2W = np.array([[0, 1, 0, 0], [-1, 0, 0, Y_COLLAR], [0, 0, 1, Z_SEAT], [0, 0, 0, 1.0]])   # seat -> W (proper rotation)


def stand_parts():
    import stand_v6_parts as S, stand_v7_parts as V
    P = {}
    P["mast v6"] = S.BUILD["mast"](); P["cradle_keel v6"] = S.BUILD["cradle_keel"]()
    P["kb v7b"] = V.BUILD["kb"](); P["toe v7b"] = V.BUILD["toe"]()
    ba = V.BUILD["bar_half"](); P["bar_half A"] = ba; P["bar_half B"] = ba.rotate([0, 0, 180]).translate([0, 2 * 54.0, 0])
    fm = S.BUILD["frame_mid"]()
    for p in S.POS["frame_mid"]: P[f"frame_mid @{p}"] = S.place(fm, p)
    P["frame_endR @30"] = S.place(V.BUILD["frame_endR"](), 30); P["frame_endL @-210"] = S.place(V.BUILD["frame_endL"](), -210)
    ps = V.BUILD["post_seg"]()
    for c in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        for k in range(3):
            P[f"post {c} {k}"] = V.post_place(ps, c).translate([0, 0, S.FLOOR + k * V.SEG])
    for nm in ("bracket_jA", "bracket_jB", "bracket_e"):
        m = V.BUILD[nm](); P[f"{nm} R"] = m; P[f"{nm} L"] = V.bracket_place(nm, m, "L")
    sheets = {"sheet side +X": S.box(V.X_IN, V.X_IN + 3, V.Y_BACK + 3, V.Y_FRONT - 3, S.FLOOR, V.Z_TOP),
              "sheet side -X": S.box(-V.X_IN - 3, -V.X_IN, V.Y_BACK + 3, V.Y_FRONT - 3, S.FLOOR, V.Z_TOP),
              "sheet front": S.box(-V.X_IN + 3, V.X_IN - 3, V.Y_FRONT, V.Y_FRONT + 3, S.FLOOR, V.Z_TOP),
              "sheet rear": S.box(-V.X_IN + 3, V.X_IN - 3, V.Y_BACK - 3, V.Y_BACK, S.FLOOR, V.Z_TOP)}
    return P, sheets, S.BAYS


def bay_modules(bays):
    p = os.path.join(HERE, "models", "box_baymod_v5_W.stl")
    src = p if os.path.exists(p) else os.path.join(HERE, "models", "box_baymod_v4i_W.obj")
    m = trimesh.load(src, force="mesh")
    out = {}
    for a in bays:
        t = m.copy(); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90 - a), [0, 1, 0]))
        out[f"bay module @{a} ({os.path.basename(src)})"] = t
    return out


def collar():
    """the collar in the seat frame: the funnel panel v7 run's collar mesh (runs/funnel_panel/v7/collar_v7_seat_frame.stl,
    or COLLAR_STL), else funnel_panel_v7.collar_world, else the v6 collar."""
    for pth in (os.environ.get("COLLAR_STL"), os.path.join(ROOT, "runs", "funnel_panel", "v7", "collar_v7_seat_frame.stl")):
        if pth and os.path.exists(pth):
            return man(trimesh.load(pth, force="mesh")), "collar v7 (4 latched + 4 plain panels)"
    try:
        F = importlib.import_module("funnel_panel_v7")
        m = F.collar_world() if hasattr(F, "collar_world") else None
        if m is not None: return m, "collar v7"
    except ImportError:
        pass
    import funnel_panel as F6
    return F6.collar_world(), "collar v6"


def latches():
    import hook_latch as H
    best, _ = H.search()
    lay = H.servo_layout(best["r_h"], best["side"], best["body_dir"])
    P = H.build(lay)
    out = {}
    for name, fs, xs in H.UNITS:
        T, _ = H.unit_frame(fs, xs)
        TW = S2W @ T
        out[f"latch housing {name}"] = (H.to_tm(P["housing"], TW), "#cc6644", True)
        out[f"latch plunger {name}"] = (H.to_tm(P["plunger"], TW), "#e8a33c", True)
        out[f"latch servo {name}"] = (H.to_tm(P["servo"], TW), "#3a6fd8", False)
    return out, H, lay, P


def arm_v5():
    """the v5 arm as merged (PR 36): hub v5 print in W, rails A / B v1, carriage proxies (heads v6 not drawn yet), pinions,
    motors; groups: fixed (stub, sun post), arm (turns), headA / headB (turn + stroke). Cartridges from arm_kinematics."""
    import arm_cad as A, arm_layout as L, arm_v5_layout as V, arm_v5_cad as C5, sweep_box as SB
    tmp = os.path.join(OUT, "_hub_v5_W.stl")
    hub = SB.hub_from_print(os.path.join(ROOT, "print", "box_hub_v5.stl"), tmp); os.remove(tmp)
    parts = {"stub v3": (tm(A.stub()), "fixed", "#566573", True), "sun post v1": (tm(A.sun_post()), "fixed", "#566573", True),
             "hub v5": (hub, "arm", "#7f8c8d", True),
             "rail A v1": (tm(C5.rail("A")), "arm", "#a569bd", True), "rail B v1": (tm(C5.rail("B")), "armB", "#a569bd", True)}
    for h, M in V.MOTORS_V5.items():
        parts[f"NEMA17 {h}"] = (tm(A.rsquare(M, V.NEMA / 2, 2.0, 50.0)), "arm", "#2c3e50", False)
    parts["NEMA17 axle"] = (tm(A.rsquare(dict(xc=53.9, zc=-43.7, rot=-39.0), V.NEMA / 2, 2.0, 50.0)), "arm", "#2c3e50", False)
    for h, g in (("A", "arm"), ("B", "armB")):
        parts[f"pinion16 {h}"] = (tm(C5.pinion16()[0].translate([V.PIN_X, 0, V.PIN_Z])), g, "#f5b041", True)
        c = C5.carriage("A")
        parts[f"carriage {h} (head v6 proxy)"] = (tm(c["printed"] + c["bush"]), f"head{h}", "#d35400", True)
        parts[f"V-wheels {h}"] = (tm(c["wheels"]), f"head{h}", "#c39bd3", True)
    return parts


def electronics():
    p = os.path.join(ROOT, "runs", "box_electronics", "v1", "result.json")
    out = {}
    if not os.path.exists(p): return out
    R = json.load(open(p))
    for k, b in R.get("placement", {}).items():
        x0, x1, z0, z1 = b["rect"]
        out[f"board {k}"] = trimesh.creation.box(extents=[x1 - x0, 18.0, z1 - z0]).apply_translation([(x0 + x1) / 2, -66 + 6 + 9, (z0 + z1) / 2])
    return out


def main(video=False):
    from motion import Scene
    os.makedirs(OUT, exist_ok=True)
    sc = Scene("Nox box: whole assembly", "stand, ring, bays, collar + hook latch, arm v5, electronics; one swap cycle")
    P, sheets, bays = stand_parts()
    for k, m in P.items(): sc.part(k, tm(m), group="stand", color="#8d6e63", printed=True, faces=6000)
    for k, m in sheets.items():                                  # cut-away: only the rear sheet (the electronics board) is drawn
        if k == "sheet rear": sc.part(k, tm(m), group="enclosure", color="#dfe9f2", printed=False, faces=200)
    for k, t in bay_modules(bays).items(): sc.part(k, t, group="bays", color="#5b8ac9", printed=True, faces=15000)
    cm, cname = collar(); ct = tm(cm); ct.apply_transform(S2W)
    sc.part(cname, ct, group="collar", color="#b9c2cc", printed=True, faces=30000)
    L, H, lay, HP = latches()
    for k, (t, c, pr) in L.items(): sc.part(k, t, group="hook latch", color=c, printed=pr, faces=4000)
    for k, t in electronics().items(): sc.part(k, t, group="electronics", color="#2e7d32", printed=False, faces=50)
    # arm v5 through the swap cycle (arm_kinematics KEYS with the v5 strokes), the 2 packs from its pack logic,
    # the drone landing at T_LAND, latched, lifted onto the plungers by the insert, leaving after the latch opens
    import arm_kinematics as K
    zmap = {K.ZP: 32.0, K.ZBAY: 86.1, K.ZDR: 95.6, K.ZW: 81.0}                       # v5 strokes (arm_v5_layout Z0S, C)
    K.KEYS = [(t, ph, zmap.get(a, a), zmap.get(b, b), fa, fb) for (t, ph, a, b, fa, fb) in K.KEYS]
    AP = arm_v5()
    for k, (t, g, c, pr) in AP.items(): sc.part(k, t, group="arm v5", color=c, printed=pr, faces=12000)
    cart = tm(K.cartridge_local()); sc.part("pack charged", cart, group="packs", color="#44aa99", printed=False)
    sc.part("pack dead", cart, group="packs", color="#9955bb", printed=False)
    D = H.drone_parts()
    col = {"wheels": "#44aa66", "cage": "#333333", "plate": "#556677", "props": "#9fb8d0", "receiver": "#778899"}
    for k, m in D.items():
        t = H.to_tm(m, S2W); sc.part(f"drone {k}", t, group="drone", color=col[k], printed=False, faces=8000)
    lift = H.GAP * math.sqrt(2)
    def ease(a): a = min(max(a, 0.0), 1.0); return a * a * (3 - 2 * a)
    def dz(t):                                       # drone height over its seat (mm)
        if t < K.T_LAND: return 220.0 * (1 - ease((t - 3.5) / (K.T_LAND - 3.5))) if t > 3.5 else 220.0
        if 11.0 <= t < 12.0: return lift * ease((t - 11.0) / 0.4)
        if 12.0 <= t < 12.5: return lift * (1 - ease((t - 12.0) / 0.5))
        if t >= 14.0: return 260.0 * ease((t - 14.0) / 2.5)
        return 0.0
    def rise(t):                                     # latch out after the seat, in after the head clears
        if t < K.T_LAND + 0.1: return 0.0
        if t < K.T_LAND + 0.5: return ease((t - K.T_LAND - 0.1) / 0.4)
        if t < 13.5: return 1.0
        if t < 13.9: return 1 - ease((t - 13.5) / 0.4)
        return 0.0
    UT = {name: S2W @ H.unit_frame(fs, xs)[0] for name, fs, xs in H.UNITS}
    # a pack that is nowhere yet is shrunk to a point inside the stand (frame-0 bounds set the floor grid: never park
    # anything outside the box)
    hide = trimesh.transformations.translation_matrix([0, 0, -150]) @ np.diag([1e-4, 1e-4, 1e-4, 1.0]) @ trimesh.transformations.translation_matrix([0, -175, -24])
    def poses(t):
        Tk, carts, st = K.transforms(t)
        Tarm = K.rotY(90.0 - st["phi"]); RB = K.RB
        TA = Tarm @ K.trans(0, 0, st["zA"]); TB = Tarm @ RB @ K.trans(0, 0, st["zB"])
        out = {}
        for k, (_, g, _, _) in AP.items():
            out[k] = {"fixed": np.eye(4), "arm": Tarm, "armB": Tarm @ RB, "headA": TA, "headB": TB}[g]
        for nm in ("pack charged", "pack dead"):
            out[nm] = carts[nm][0] if nm in carts else hide
        if t >= 14.0 and "pack charged" in carts and carts["pack charged"][2].startswith("drone"):
            out["pack charged"] = K.trans(0, 0, dz(t)) @ carts["pack charged"][0]
        for k in D: out[f"drone {k}"] = (0, 0, dz(t), 0, 0, 0)
        for name, T in UT.items():
            out[f"latch plunger {name}"] = T @ trimesh.transformations.translation_matrix([0, 0, H.RISE * rise(t)]) @ np.linalg.inv(T)
        return out
    sc.timeline(poses, duration_s=K.T_END, fps=30)
    ts = np.arange(int(K.T_END * 30)) / 30.0
    sc.series("arm angle", [K.state(t)["phi"] for t in ts], unit="deg")
    sc.series("plunger out", [H.RISE * rise(t) for t in ts], unit="mm")
    sc.series("drone height over seat", [dz(t) for t in ts], unit="mm")
    sc.view("iso", eye=(640, 820, 460), target=(0, 135, 10))          # from the open front corner, the rear sheet behind
    sc.view("front", eye=(700, 800, 250), target=(0, 140, 40))
    sc.view("collar top", eye=(0, 177, 700), target=(0, 177, 150))
    sc.view("latch", eye=(40, 20, 300), target=(122, 77, 182))
    sc.view("arm", eye=(380, 520, 40), target=(0, 120, 0))
    sc.write(os.path.join(OUT, "motion.html"))
    if video: sc.video(os.path.join(OUT, "motion.mp4"))


if __name__ == "__main__":
    main(video="--video" in sys.argv)
