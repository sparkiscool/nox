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
    sc = Scene("Nox box: whole assembly", "stand, ring, bays, collar with the hook latch, electronics; swap cycle")
    P, sheets, bays = stand_parts()
    for k, m in P.items(): sc.part(k, tm(m), group="stand", color="#8d6e63", printed=True, faces=6000)
    for k, m in sheets.items(): sc.part(k, tm(m), group="enclosure", color="#dfe9f2", printed=False, faces=200)
    for k, t in bay_modules(bays).items(): sc.part(k, t, group="bays", color="#5b8ac9", printed=True, faces=15000)
    cm, cname = collar(); ct = tm(cm); ct.apply_transform(S2W)
    sc.part(cname, ct, group="collar", color="#b9c2cc", printed=True, faces=30000)
    L, H, lay, HP = latches()
    for k, (t, c, pr) in L.items(): sc.part(k, t, group="hook latch", color=c, printed=pr, faces=4000)
    for k, t in electronics().items(): sc.part(k, t, group="electronics", color="#2e7d32", printed=False, faces=50)
    sc.view("front", eye=(0, 900, 150), target=(0, 140, 60))
    sc.view("collar top", eye=(0, 177, 700), target=(0, 177, 150))
    sc.timeline(lambda t: {}, duration_s=2.0, fps=10)
    sc.write(os.path.join(OUT, "motion.html"))
    if video: sc.video(os.path.join(OUT, "motion.mp4"))


if __name__ == "__main__":
    main(video="--video" in sys.argv)
