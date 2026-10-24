"""Box collar funnel panel A / B v7 (24 Oct 2026): the 4 panels of the two seated faces (drone +-y, over the 45 / 135 deg
bays) carry the hook latch (Spark 23 Oct: "a hooked latch, not a steel belly, no magnets"; nox/box/box-hook-latch.md).
v7 = v6 (funnel_panel.py: faces, flanges, keys, seat block, M3 countersunk from above, unchanged) plus, per panel:
  - skin 2.0 (the 2.0 floor; the v6 1.6 skin reads thin walls in solid_check), ribs 9 behind it, re-optimized
    (panel_opt_v7.py -> rib_mask_v7.json) with the latch window as a hole and the latch loads as cases;
  - skin window 9.0 (along the slope) x 13.0 (C: plunger 8 x 12 + 0.5 a side), centred at its wheel (lateral 100), 54.1 up
    the slope from the bottom edge (hook_latch.py U0);
  - 4 bosses o 12 x 7 behind the skin with 2 gussets each, M3 x 5.7 heat-set (hole o 4.0), exactly hook_latch.bosses_local()
    at the latch bolt points;
  - low ribs (6.7 behind the skin, 0.3 under the housing flange) over the housing footprint + 1.0 (unit frame
    u -25.5..34.0, v -10.2..41.6), none over the guide block + 1.0 (DEFAULT; the spec's rib-free window left the bosses
    and the strut under the wheel track as islands on the skin: solid_check neck fails);
  - drawn tie ribs 4 wide: boss to boss, bosses to the rim beam and the strut, seat block to the strut (one rib network:
    solid_check fails ribs or blocks that join the rest only through the skin; the v6 seat block sat on skin only).
The other 4 panels (the unlatched faces) stay v6. Frames: world = seat frame of funnel_panel.py; panel A = +x face, y >= 0
half; in the collar the seated faces are +-y (k = 1, 3 of funnel_panel.assembly). The latch on panel A is hook_latch unit
"+y_x-" (housing B) turned by -90 deg about z.
v7 plain (the 4 panels of the unlatched faces; the v6 file fails solid_check): the same skin 2.0, rib height, flanges,
keys, seat block and seat tie, ribs from panel_opt_v7.py run plain (rib_mask_v7_plain.json), no window, bosses or low ribs.
python3 funnel_panel_v7.py [--motion] -> runs/funnel_panel/v7/ (STLs, result.json: latched FE), print/box_funnel_panel_*_v7*.stl
python3 funnel_panel_v7.py --plain     -> the same files + result_plain.json (plain FE)"""
import math, json, os, sys, shutil
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v7"); os.makedirs(RUN, exist_ok=True)
sys.path[:0] = [HERE, os.path.dirname(HERE), os.path.join(ROOT, "nox")]
import funnel_panel as F                                         # v6, not edited: its globals are switched for v7 below
import hook_latch as HL
S2 = math.sqrt(2)

# hook latch v1 as merged (SKIN 1.6), kept for the report; then the panel v7 skin (panel_opt_v7.latch_v7 patches HL)
_LAY = HL.servo_layout(13.0, 1, -1)
P_V1 = HL.build(_LAY)
if "--plain" in sys.argv: os.environ["NOX_FUNNEL_PLAIN"] = "1"   # the FE checks run on the plain plate model
import panel_opt_v7 as PO                                        # patches HL to the 2.0 skin on import
P7 = HL.build(HL.servo_layout(13.0, 1, -1))
SKIN = PO.SKIN7
FREE_U, FREE_V = PO.FREE_U, PO.FREE_V
WIN_U, WIN_V = PO.WIN_U, PO.WIN_V
RIB_GROW_V7 = 1.25                                               # least outline grow whose as-built FE passes (sweep 1.0 .. 2.5, C)
RIB_GROW_PLAIN = 1.25                                            # the same for v7 plain (C)
MODE = dict(plain=False)                                         # which panel the build functions draw

RZ = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])   # seat (+y face) -> panel A world (+x face)
T_A = RZ @ HL.unit_frame(1, -1)[0]                               # unit frame -> panel A world (left-handed: housing B)

def to_mf(m, T): return m.transform(np.asarray(T)[:3, :4].tolist())

def v7_globals():
    J = json.load(open(os.path.join(HERE, "rib_mask_v7_plain.json" if MODE["plain"] else "rib_mask_v7.json")))
    F._OPT = dict(ts=J["ts"], h=J["h"], H=J["H"], ec=np.asarray(J["ec"]), mask=np.isin(np.arange(len(J["ec"])), J["mask"]))
    F.T, F.RIB_H = float(J["ts"]), float(J["h"])
    F.RIB_GROW = RIB_GROW_PLAIN if MODE["plain"] else RIB_GROW_V7
    return J

def unit_rect(U, V):
    """unit-frame rectangle u in U, v in V -> face coordinates (lateral 100 - v, slope 54.1 + u) of panel A."""
    u0, u1 = PO.X_L - V[1], PO.X_L - V[0]
    v0, v1 = PO.U0 + U[0], PO.U0 + U[1]
    return mf.CrossSection([[(u0, v0), (u1, v0), (u1, v1), (u0, v1)]])

GUIDE_UV = ((-PO.GUIDE_U, PO.GUIDE_U), PO.GUIDE_V)

TIE_W = 4.4                                                      # drawn tie ribs: width (survives every solid_check erosion; not 4.0: the boss gussets sit at +-2..4)
TIES_UNIT = [((-18.5, 34.6), (21.67, 34.6)),                    # boss to boss along the slope (valley-far row)
             ((-18.5, 0.0), (-18.5, 34.6)), ((18.5, 0.0), (21.67, 34.6)),   # boss to boss across, either side of the guide
             ((-18.5, 34.6), (-30.0, 34.6)),                    # downhill: into the strut under the wheel track
             ((18.5, 0.0), (48.0, 0.0)), ((21.67, 34.6), (48.0, 34.6))]     # uphill: into the rim beam
TIE_SEAT = ((F.SEAT_Y, F.SEAT_V), (78.0, F.SEAT_V))             # seat block to the strut (face coordinates)
TIES_FACE = [((142.0, 101.0), (139.0, 89.0))]                   # rim corner piece to the valley diagonal (a 2.4 neck in v6 / v7)
TIE_SEAT_PLAIN = TIE_SEAT                                        # v7 plain: seat block to its nearest rib (set from its layout)
TIES_FACE_PLAIN = [((65.5, 66.0), (63.0, 82.0)),              # v7 plain: wheel-track strut up into the rim stub
                   ((139.0, 101.0), (139.0, 87.0))]             # rim corner piece to the valley diagonal

def ties_cs(seat=True):
    """drawn tie ribs (face coordinates): the 4 latch bosses, the strut under the wheel track, the rim beam and the seat block
    in one rib network (the optimizer joins them through the thick bosses, the print needs ribs there)."""
    def seg(a, b):
        c = lambda p: mf.CrossSection.circle(TIE_W / 2, 24).translate(p)
        return mf.CrossSection.batch_hull([c(a), c(b)])
    f = lambda p: (PO.X_L - p[1], PO.U0 + p[0])
    pl = MODE["plain"]
    out = seg(*(TIE_SEAT_PLAIN if pl else TIE_SEAT)) if seat else mf.CrossSection()
    for a, b in ([] if pl else TIES_UNIT): out = out + seg(f(a), f(b))
    for a, b in (TIES_FACE_PLAIN if pl else TIES_FACE): out = out + seg(a, b)
    dom = mf.CrossSection([[(0.0, 0.0), (84.0, 0.0), (84.0 + F.SLANT / S2, F.SLANT), (0.0, F.SLANT)]])
    return out ^ dom

def rib_section_full(seat=True):
    cs = F.rib_section()
    return ties_cs(seat) if cs is None else cs + ties_cs(seat)

def extrude_face(cs):
    """face-coordinate section -> rib solid behind the skin (full height), panel A world."""
    p0 = np.array(F.to_print_pt((84.0, 0.0, F.ZB))); p1 = np.array(F.to_print_pt((84.0 + 10 / S2, 0.0, F.ZB + 10 / S2)))
    dv = (p1 - p0) / 10
    polys = [[(p0[0] + v * dv[0], u) for (u, v) in poly] for poly in cs.to_polygons()]
    rb = mf.Manifold.extrude(mf.CrossSection(polys, mf.FillRule.EvenOdd), F.RIB_H + 0.01).translate([0, 0, p0[2] + F.T - 0.01])
    return rb.rotate([0, -F.ROT, 0]).trim_by_plane([0.0, 0.0, 1.0], F.ZP)

def seat_tie():
    """the seat block's tie rib: through the block, kept 0.1 short of the landing plane inside the post room's radius."""
    room = F.at_seat(mf.Manifold.cylinder(40, F.KEEP_R, F.KEEP_R, 64).translate([0, 0, F.CLAMP - 0.1]))
    ts = TIE_SEAT_PLAIN if MODE["plain"] else TIE_SEAT
    return extrude_face(mf.CrossSection.batch_hull([mf.CrossSection.circle(TIE_W / 2, 24).translate(p) for p in ts])) \
        - room - F.screw_hole()

def rib_section_v7():
    """the rib outline (optimizer mask, smoothed, grown, + ties) without the guide block keep-out."""
    return rib_section_full() if MODE["plain"] else rib_section_full() - unit_rect(*GUIDE_UV)

def ribs_world_v7():
    """full ribs (9) outside the housing footprint + 1.0, low ribs (6.7, 0.3 under the housing flange) inside it, none
    over the guide block + 1.0."""
    rb = extrude_face(rib_section_full(seat=False)) - F.post_room() - F.screw_hole() + seat_tie()
    if MODE["plain"]: return rb
    low = to_mf(HL.box(FREE_U[0], FREE_U[1], FREE_V[0], FREE_V[1], -60.0, -SKIN - PO.H_LOW), T_A)
    guide = to_mf(HL.box(GUIDE_UV[0][0], GUIDE_UV[0][1], GUIDE_UV[1][0], GUIDE_UV[1][1], -60.0, -SKIN), T_A)
    return rb - low - guide

def latch_features():
    """bosses + gussets (hook_latch.bosses_local, 2.0 skin), skin window and rib-free box, in panel A world."""
    bos = to_mf(HL.bosses_local(P7), T_A)
    # the downhill valley-side boss's gussets run 1.3 past the valley plane (into the next face's panel): trimmed there, so
    # they end in this panel's valley flange (the bosses themselves and everything the housing touches are unchanged)
    bos = F.plane_keep(bos, (1, -1, 0), 0.0)
    win = to_mf(HL.box(-WIN_U / 2, WIN_U / 2, -WIN_V / 2, WIN_V / 2, -40.0, 1.0), T_A)
    return bos, win

def panel_A_world():
    J = v7_globals()
    base = F.panel_world()                                        # v6 body at the 2.0 skin: skin, flanges, seat block
    if MODE["plain"]: return base + ribs_world_v7(), J
    bos, win = latch_features()
    return (base + ribs_world_v7() + bos) - win, J

def build(plain=False):
    MODE["plain"] = plain
    w, J = panel_A_world()
    pa = F.to_print(w); pb = F.to_print(w.mirror([0, 1, 0]))
    for p in F.key_points():
        q = F.to_print_pt(p)
        pa = pa + F.teardrop(F.PEG_D, F.PEG_L).translate([q[0], 0.0, q[2]])
        pb = pb - F.teardrop(F.PEG_D + 2 * F.CLR, F.FL_T + 2).translate([q[0], 1.0, q[2]])
    return pa, pb, w, J

def tm(m):
    """manifold -> trimesh as is (a merge pass at trimesh's tolerance joins 0.3-apart rib / boss corners and opens edges)."""
    return F.tm(m)

# ---------------------------------------------------------------- checks
def asbuilt_mask():
    """the built rib outline (smoothed, grown, rib-free window cut) back on the FE elements (centre inside)."""
    from matplotlib.path import Path
    cs = rib_section_v7()
    ec = PO.ec; u = np.abs(ec[:, 0]); v = ec[:, 1]
    inside = np.zeros(len(ec), int)
    for poly in cs.to_polygons():
        inside += Path(np.asarray(poly)).contains_points(np.stack([u, v], 1)).astype(int)
    m = (inside % 2 == 1)
    m &= ~(((u - F.SEAT_Y) ** 2 + (v - F.SEAT_V) ** 2) <= F.KEEP_R ** 2)      # post room behind the seat block
    return m

def collar(pa7, pb7):
    """seat frame collar: v7 plain on the unlatched faces (k 0, 2; v6 until the plain mask exists), v7 on the seated faces
    (+-y, k 1, 3)."""
    pa6, pb6 = F_U
    out = []
    for k in range(4):
        a, b = (pa7, pb7) if k in (1, 3) else (pa6, pb6)
        for m in (a, b):
            out.append(m.rotate([0, -F.ROT, 0]).rotate([0, 0, 90 * k]))
    return out

def latch_fit(parts, col_union):
    res = {}
    for name, fs, xs in HL.UNITS:
        T, _ = HL.unit_frame(fs, xs)
        hm = to_mf(parts["housing"], T)
        r = dict(housing_mm3=round(HL.vol(hm ^ col_union), 3),
                 plunger_in_mm3=round(HL.vol(to_mf(parts["plunger"], T) ^ col_union), 3),
                 plunger_out_mm3=round(HL.vol(to_mf(parts["plunger"].translate([0, 0, HL.RISE]), T) ^ col_union), 3))
        res[name] = r
    return res

def window_fit(col_union):
    """plunger section grown by c a side over the skin depth vs the collar: 0 below 0.5, > 0 above (window 0.5 a side)."""
    out = {}
    for c in (0.45, 0.55):
        v = 0.0
        for name, fs, xs in HL.UNITS:
            T, _ = HL.unit_frame(fs, xs)
            v += HL.vol(to_mf(HL.box(-HL.PL_U / 2 - c, HL.PL_U / 2 + c, -HL.PL_V / 2 - c, HL.PL_V / 2 + c, -SKIN + 0.05, -0.05), T) ^ col_union)
        out[f"grown_{c}_mm3"] = round(v, 3)
    T, _ = HL.unit_frame(1, 1)
    out["window_centre_slope_lateral"] = [round(PO.U0, 2), round(PO.X_L, 2)]
    out["plunger_axis_slope_lateral"] = [round(float(HL.face_t(T[:3, 3])), 2), round(float(T[0, 3]), 2)]
    return out

def wheel_path_steps(col_tm):
    """rays square to each face from inside the mouth on the wheel track (lateral 50..150 each side, slope 0.5 .. rim - 0.5,
    2.0 apart across, 0.5 down): depth of the first hit below the face plane; a step = |change| between neighbours
    down the slope. The window +-1.0 (the plunger fills it) is reported apart."""
    import trimesh.ray.ray_triangle as RT
    it = RT.RayMeshIntersector(col_tm)
    res = {}
    for k in range(4):
        th = math.radians(90 * k); Rz = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
        n_in = Rz @ np.array([-1.0, 0, 1.0]) / S2                  # out of the +x face into the mouth (rotated)
        tt = np.arange(0.5, F.SLANT - 0.5, 0.5); lat = np.concatenate([np.arange(-150, -49.9, 2.0), np.arange(50, 150.1, 2.0)])
        L, Tt = np.meshgrid(lat, tt, indexing="ij")
        p = np.stack([84.0 + Tt / S2, L, F.ZB + Tt / S2], -1).reshape(-1, 3)
        okf = np.abs(p[:, 1]) <= p[:, 0] - 0.6                      # inside the face's own width (off the valley)
        p = p @ Rz.T
        o = p + 3.0 * n_in; d = np.tile(-n_in, (len(p), 1))
        loc, ri, _ = it.intersects_location(o, d, multiple_hits=False)
        dep = np.full(len(p), np.nan); dep[ri] = np.linalg.norm(loc - o[ri], axis=1) - 3.0
        dep[~okf] = np.nan
        dep = dep.reshape(L.shape)
        step = np.abs(np.diff(dep, axis=1)); step = np.nan_to_num(step, nan=0.0)
        uL = (Tt[:, 1:] + Tt[:, :-1]) / 2 - PO.U0; vL = 100.0 - np.abs(L[:, 1:])
        inwin = (k in (1, 3)) & (np.abs(uL) <= WIN_U / 2 + 1.0) & (np.abs(vL) <= WIN_V / 2 + 1.0)
        res[f"face k{k}" + (" (seated)" if k in (1, 3) else "")] = dict(rays=int(okf.sum()), hits=int(np.isfinite(dep).sum()),
            max_step_off_window=round(float(step[~inwin].max()), 3), steps_over_0p2_off_window=int((step[~inwin] > 0.2).sum()),
            max_step_at_window=round(float(step[inwin].max()) if inwin.any() else 0.0, 2),
            face_depth_range_off_window=[round(float(np.nanmin(dep)), 3), round(float(np.nanmax(np.where(np.isnan(dep), 0, dep))), 3)])
    return res

def boss_hand_calc():
    """in-plane share of the latch load on the panel (the plate FE has the normal boss forces): the bosses carry F / 4 each
    as shear at the flange face, bending at the boss root 7 below; boss axis = print z, so across layers (C geometry, E load)."""
    out = []
    D, d = 2 * HL.BOSS_R, 4.0
    Z = math.pi * (D ** 4 - d ** 4) / (32 * D); A = math.pi * (D ** 2 - d ** 2) / 4
    for name, Fl, cls, N in (("functional 30 N", HL.F_FUNC, "flight", np.abs(PO.N_fun).max()), ("misuse 100 N", HL.F_MIS, "misuse", np.abs(PO.N_mis).max())):
        s = Fl / 4 * HL.BOSS_H / Z + N / A
        allow = HL.ZT_ / HL.SF[cls]
        out.append(dict(case=name, boss_shear_N=round(Fl / 4, 1), boss_normal_N=round(float(N), 1), boss_root_MPa=round(s, 2),
                        index_across_layers=round(s / allow, 3), skin_membrane_MPa=round(Fl / (SKIN * 2 * (HL.PL_V + 2 * HL.BOSS_R)), 2)))
    return out

def to_seat(m): return m.rotate([0, -F.ROT, 0])

def make_motion(pa, pb):
    """assembly render: the collar (v7 on the seated faces, v6 on the others) with the 4 hook latches (housings at the 2.0
    skin, plungers, servos), the plungers going out and back in. Static part: this is the fit view, not a mechanism."""
    from motion import Scene
    sc = Scene("Funnel panels v7 + hook latch", "seated faces (drone +-y): v7 panels with skin windows, bosses, low ribs, 4 latches; other faces v7 plain")
    pa6, pb6 = F_U
    for k in range(4):
        for nm, m in (("A", pa if k in (1, 3) else pa6), ("B", pb if k in (1, 3) else pb6)):
            v = "v7" if k in (1, 3) else U_NAME
            sc.part(f"panel {nm} {v} k{k}", tm(m.rotate([0, -F.ROT, 0]).rotate([0, 0, 90 * k])), group=f"panels {v}",
                    color="#b9c2cc" if v != "v7" else "#7fa7d9", printed=True, faces=40000)
    Ts = {}
    for name, fs, xs in HL.UNITS:
        T, _ = HL.unit_frame(fs, xs); Ts[name] = T
        sc.part(f"housing {name}", HL.to_tm(P7["housing"], T), group="latch", color="#cc6644", printed=False)
        sc.part(f"plunger {name}", HL.to_tm(P7["plunger"], T), group="latch", color="#e8a33c", printed=False)
        sc.part(f"servo {name}", HL.to_tm(P7["servo"], T), group="latch", color="#3a6fd8", printed=False)
    DUR = 6.0
    def r(t):
        a = min(max((t - 1.0) / 1.0, 0.0), 1.0) if t < 3.5 else 1 - min(max((t - 3.5) / 1.0, 0.0), 1.0)
        return a * a * (3 - 2 * a)
    def poses(t):
        return {f"plunger {n}": Ts[n] @ trimesh.transformations.translation_matrix([0, 0, HL.RISE * r(t)]) @ np.linalg.inv(Ts[n])
                for n, _, _ in HL.UNITS}
    sc.timeline(poses, duration_s=DUR, fps=30)
    sc.series("plunger out", [HL.RISE * r(t) for t in np.arange(int(DUR * 30)) / 30.0], unit="mm")
    sc.view("seated face from the mouth", eye=(150, 60, 120), target=(100, 120, 40))
    sc.view("behind the seated face", eye=(170, 240, -40), target=(95, 140, 30))
    sc.view("whole collar", eye=(0, -420, 330), target=(0, 0, 0))
    sc.write(os.path.join(RUN, "motion.html"))
    sc.video(os.path.join(RUN, "motion.mp4"), view="behind the seated face")

if __name__ == "__main__":
    F_v6 = F.build()                                             # v6 panels (1.6 skin, v6 ribs) before the v7 switch
    have_plain = os.path.exists(os.path.join(HERE, "rib_mask_v7_plain.json"))
    out = {}
    if have_plain:
        pua, pub, _, Jp = build(plain=True); F_U, U_NAME = (pua, pub), "v7 plain"
        out.update({"A_v7_plain": pua, "B_v7_plain": pub})
    else:
        F_U, U_NAME = F_v6, "v6"
    pa, pb, wA, J = build(plain=False)
    out.update({"A_v7": pa, "B_v7": pb})
    res = dict(part="funnel_panel", version="v7", skin=SKIN, rib_h=float(J["h"]), rib_vf=J.get("vf"),
               rib_vf_plain=Jp.get("vf") if have_plain else None, unlatched_faces=U_NAME)
    names = {}
    for name, m in out.items():
        t = tm(m)
        t.apply_translation(-np.array([t.bounds[:, 0].mean(), t.bounds[:, 1].mean(), t.bounds[0, 2]]))
        p = os.path.join(RUN, f"box_funnel_panel_{name}.stl"); t.export(p); names[name] = p
        res[name] = F.info_of(t)
        res[name]["watertight_after_stl_reload"] = bool(trimesh.load(p, force="mesh").is_watertight)
    parts = collar(pa, pb)
    ov = sum(HL.vol(parts[i] ^ parts[j]) for i in range(8) for j in range(i + 1, 8))
    full = parts[0]
    for q in parts[1:]: full = full + q
    ft = tm(full)
    res["assembly"] = dict(unlatched=U_NAME, overlap_mm3=round(ov, 3), bodies=sum(abs(b.volume) > 1.0 for b in ft.split(only_watertight=False)),
                           g_PETG=round(ft.volume / 1000 * 1.27, 1), bbox=[round(v, 1) for v in ft.extents])
    res["latch_fit_v7_skin"] = latch_fit(P7, full)                # housing / plunger with the guide top under the 2.0 skin
    res["latch_fit_v1_as_merged"] = latch_fit(P_V1, full)         # hook_latch v1 (SKIN 1.6): guide top 0.1 into the skin
    res["window"] = window_fit(full)
    res["bosses_vs_housing_mm3"] = round(HL.vol(HL.bosses_local(P7) ^ P7["housing"]), 3)
    res["wheel_path"] = wheel_path_steps(ft)
    # FE: the as-built rib outline on the plate model of this run's mode (latched, or plain with --plain), every case
    MODE["plain"] = PO.PLAIN; v7_globals()
    res["fe_mode"] = "v7 plain" if PO.PLAIN else "v7 latched"
    res["fe_asbuilt"] = {k: v for k, v in PO.report(asbuilt_mask()).items()}
    if not PO.PLAIN: res["boss_hand_calc"] = boss_hand_calc()
    json.dump(res, open(os.path.join(RUN, "result_plain.json" if PO.PLAIN else "result.json"), "w"), indent=1,
              default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    ft.export(os.path.join(RUN, "collar_v7_seat_frame.stl"))
    print(json.dumps({k: v for k, v in res.items() if k not in ("fe_asbuilt",)}, indent=1, default=str))
    print("fe as-built:", res["fe_mode"], {k: v for k, v in res["fe_asbuilt"].items() if k != "per_case"})
    for name, p in names.items(): shutil.copy(p, os.path.join(ROOT, "print", os.path.basename(p)))
    if "--motion" in sys.argv: make_motion(pa, pb)
