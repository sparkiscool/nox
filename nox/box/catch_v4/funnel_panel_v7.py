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
python3 funnel_panel_v7.py --plain     -> the same files + result_plain.json (plain FE)
v8 plain (24 Oct, box catch DEFAULT): v7 plain with its bottom edge at seat z 12.8 (opening 191 along drone x; the +-x faces
only), 16.26 up the slope from the v7 edge: skin, ribs and flanges end square on the new edge, the seat block and its M3
stay 14 up the slope from it (v 30.26), its tie redrawn; ribs from panel_opt_v7.py with NOX_FUNNEL_ZBOT=12.8
(rib_mask_v8_plain.json). Mouth, top edge, face plane, keys and every v7 file unchanged.
python3 funnel_panel_v7.py --v8        -> runs/funnel_panel/v8_plain/ (STLs, result.json: plain FE + system wheel loads), print/box_funnel_panel_*_v8_plain.stl"""
import math, json, os, sys, shutil
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
if "--v8" in sys.argv:                                           # v8 plain: the plain panel with the raised bottom edge
    os.environ["NOX_FUNNEL_PLAIN"] = "1"; os.environ.setdefault("NOX_FUNNEL_ZBOT", "12.8")
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v8_plain" if "--v8" in sys.argv else "v7"); os.makedirs(RUN, exist_ok=True)
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
V8, V_BOT = PO.V8, PO.V_BOT                                      # v8 plain: the plain build ends at v V_BOT (16.26, C)
RIB_GROW_V8 = 0.5                                                # v8 plain (C): as-built FE sweep 0 .. 1.25 reads 1.023 / 0.991 / 0.949 / 0.897 / 0.879 / 0.859; 0.5 keeps 5 % margin (DEFAULT; 0.25 passes at 0.991)
SEAT_V7, P_FACE7 = F.SEAT_V, F.P_FACE.copy()                     # v6 / v7 seat block: 14 up the slope from the z 1.3 edge
SEAT_V8 = F.SEAT_V + V_BOT                                       # v8 plain: 14 up the slope from its own edge (30.26, C)
MODE = dict(plain=False)                                         # which panel the build functions draw

RZ = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])   # seat (+y face) -> panel A world (+x face)
T_A = RZ @ HL.unit_frame(1, -1)[0]                               # unit frame -> panel A world (left-handed: housing B)

def to_mf(m, T): return m.transform(np.asarray(T)[:3, :4].tolist())

def v8():
    return MODE["plain"] and V8                                  # the plain build is v8 plain in a --v8 run

def v_bot():
    return V_BOT if v8() else 0.0

def v7_globals():
    fn = ("rib_mask_v8_plain.json" if V8 else "rib_mask_v7_plain.json") if MODE["plain"] else "rib_mask_v7.json"
    J = json.load(open(os.path.join(HERE, fn)))
    F._OPT = dict(ts=J["ts"], h=J["h"], H=J["H"], v0=J.get("v0", 0.0), ec=np.asarray(J["ec"]),
                  mask=np.isin(np.arange(len(J["ec"])), J["mask"]))
    F.T, F.RIB_H = float(J["ts"]), float(J["h"])
    F.RIB_GROW = (RIB_GROW_V8 if v8() else RIB_GROW_PLAIN) if MODE["plain"] else RIB_GROW_V7
    F.SEAT_V = SEAT_V8 if v8() else SEAT_V7                       # the seat block (at_seat reads P_FACE at call time)
    F.P_FACE = P_FACE7 + (V_BOT if v8() else 0.0) * F.E_V
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
TIE_SEAT_V8 = ((F.SEAT_Y, SEAT_V8), (84.0, SEAT_V8))            # v8 plain: seat block into the strut (its edge at u ~80, v 30)
TIES_FACE_V8 = [((139.0, 101.0), (139.0, 87.0)),               # v8 plain: rim corner piece to the valley diagonal
                ((98.0, 103.5), (126.0, 103.0))]                # rim beam into the rim corner piece (it joined the rest only
                                                                 # through the 2.4 valley flange: solid_check neck). The strut
                                                                 # now runs whole from the rim to the edge: no strut tie

def ties_cs(seat=True):
    """drawn tie ribs (face coordinates): the 4 latch bosses, the strut under the wheel track, the rim beam and the seat block
    in one rib network (the optimizer joins them through the thick bosses, the print needs ribs there)."""
    def seg(a, b):
        c = lambda p: mf.CrossSection.circle(TIE_W / 2, 24).translate(p)
        return mf.CrossSection.batch_hull([c(a), c(b)])
    f = lambda p: (PO.X_L - p[1], PO.U0 + p[0])
    pl = MODE["plain"]
    out = seg(*(TIE_SEAT_V8 if v8() else TIE_SEAT_PLAIN if pl else TIE_SEAT)) if seat else mf.CrossSection()
    for a, b in ([] if pl else TIES_UNIT): out = out + seg(f(a), f(b))
    for a, b in (TIES_FACE_V8 if v8() else TIES_FACE_PLAIN if pl else TIES_FACE): out = out + seg(a, b)
    dom = mf.CrossSection([[(0.0, 0.0), (84.0, 0.0), (84.0 + F.SLANT / S2, F.SLANT), (0.0, F.SLANT)]])
    return out ^ dom

def rib_section_full(seat=True):
    cs = F.rib_section()
    return ties_cs(seat) if cs is None else cs + ties_cs(seat)

def extrude_face(cs, h=None):
    """face-coordinate section -> rib solid behind the skin (full height, or h), panel A world."""
    h = F.RIB_H if h is None else h
    p0 = np.array(F.to_print_pt((84.0, 0.0, F.ZB))); p1 = np.array(F.to_print_pt((84.0 + 10 / S2, 0.0, F.ZB + 10 / S2)))
    dv = (p1 - p0) / 10
    polys = [[(p0[0] + v * dv[0], u) for (u, v) in poly] for poly in cs.to_polygons()]
    rb = mf.Manifold.extrude(mf.CrossSection(polys, mf.FillRule.EvenOdd), h + 0.01).translate([0, 0, p0[2] + F.T - 0.01])
    return rb.rotate([0, -F.ROT, 0]).trim_by_plane([0.0, 0.0, 1.0], F.ZP)

OPEN_R = 1.0                                                     # 2D opening of every rib section: nothing under 2.0 wide

def opened(cs):
    return cs.offset(-OPEN_R, mf.JoinType.Round).offset(OPEN_R, mf.JoinType.Round)

def rib_sections(seat=True):
    """(full-height section, low section) in face coordinates. Full: the rib outline outside the housing footprint + 1.0;
    low: inside it, less the guide block + 1.0. Each opened by 1.0, so the cut at the footprint edge leaves no lip under
    2.0 (review 1: 1.1 wide full-height lips where the outline ended just past the edge)."""
    vb = v_bot() + 0.3
    cs = rib_section_full(seat) ^ mf.CrossSection([[(-1.0, vb), (400.0, vb), (400.0, 200.0), (-1.0, 200.0)]])
    # ribs stop 0.3 above the skin's square bottom end (flush faces there gave non-manifold edges in the STL)
    if MODE["plain"]: return opened(cs), mf.CrossSection()
    box = unit_rect(FREE_U, FREE_V)
    return opened(cs - box), opened((cs ^ box) - unit_rect(*GUIDE_UV))

def seat_tie():
    """the seat block's tie rib: through the block, kept 0.1 short of the landing plane inside the post room's radius."""
    room = F.at_seat(mf.Manifold.cylinder(40, F.KEEP_R, F.KEEP_R, 64).translate([0, 0, F.CLAMP - 0.1]))
    ts = TIE_SEAT_V8 if v8() else TIE_SEAT_PLAIN if MODE["plain"] else TIE_SEAT
    return extrude_face(mf.CrossSection.batch_hull([mf.CrossSection.circle(TIE_W / 2, 24).translate(p) for p in ts])) \
        - room - F.screw_hole()

def rib_section_v7():
    """the built rib outline (full + low sections) in face coordinates, for the as-built FE."""
    full, low = rib_sections()
    return full + low

def ribs_world_v7():
    """full ribs (9) outside the housing footprint + 1.0, low ribs (6.7, 0.3 under the housing flange) inside it, none
    over the guide block + 1.0."""
    full, low = rib_sections(seat=False)
    rb = extrude_face(full)
    if not MODE["plain"]: rb = rb + extrude_face(low, PO.H_LOW)
    return rb - F.post_room() - F.screw_hole() + seat_tie()

def latch_features():
    """bosses + gussets (hook_latch.bosses_local, 2.0 skin), skin window and rib-free box, in panel A world."""
    bos = to_mf(HL.bosses_local(P7), T_A)
    # the downhill valley-side boss's gussets run 1.3 past the valley plane (into the next face's panel): trimmed there, so
    # they end in this panel's valley flange (the bosses themselves and everything the housing touches are unchanged)
    bos = F.plane_keep(bos, (1, -1, 0), 0.0)
    win = to_mf(HL.box(-WIN_U / 2, WIN_U / 2, -WIN_V / 2, WIN_V / 2, -40.0, 1.0), T_A)
    # the insert holes, cut after every union (review 1: the low ribs and the boss ties refilled them): o 4.0 from the boss
    # face (w -9.0) to 0.5 under the skin's back, as bosses_local()
    for (bu, bv) in P7["bolts"]:
        win = win + to_mf(HL.cyl_w(2.0, bu, bv, HL.W_FL0 - 1, -SKIN - 0.5), T_A)
    return bos, win

def edge_cuts(m):
    """square ends (review 1): the skin ends square to the face at the bottom edge (v >= 0; v6 ran a 2.5 lip under it,
    cut horizontal and by the drafted opening wall: a 48 deg feather), and the valley flange ends square to the valley
    line at the rim corner (the rim plane met it obliquely: a 0.7 wedge)."""
    m = F.plane_keep(m, F.E_V, float(F.E_V @ np.array([84.0, 0.0, F.ZB])) + v_bot())   # v8 plain: on its raised edge
    n = np.array([1.0, 1.0, 1.0]) / math.sqrt(3); pc = np.array([F.K0 + F.ZT, F.K0 + F.ZT, F.ZT])
    return F.plane_keep(m, -n, float(-n @ pc))

def panel_A_world():
    J = v7_globals()
    base = F.panel_world()                                        # v6 body at the 2.0 skin: skin, flanges, seat block
    if MODE["plain"]: return edge_cuts(base + ribs_world_v7()), J
    bos, win = latch_features()
    return edge_cuts((base + ribs_world_v7() + bos) - win), J

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

# catch system pass (lead, nox/box/catch_system.py on part/hook_latch_v2, C): per-plunger joint reaction along the slope
# at the worst plunger, insert 28.3 N and hand lift 85.8 N (the panel's own cases were 30 / 100 N E)
SYS_INSERT, SYS_LIFT = 28.3, 85.8

def boss_hand_calc():
    """in-plane share of the latch load on the panel (the plate FE has the normal boss forces): the bosses carry F / 4 each
    as shear at the flange face, bending at the boss root 7 below; boss axis = print z, so across layers (C geometry, E load)."""
    out = []
    D, d = 2 * HL.BOSS_R, 4.0
    Z = math.pi * (D ** 4 - d ** 4) / (32 * D); A = math.pi * (D ** 2 - d ** 2) / 4
    nf, nm = np.abs(PO.N_fun).max(), np.abs(PO.N_mis).max()
    for name, Fl, cls, N in (("functional 30 N (panel, E)", HL.F_FUNC, "flight", nf), ("misuse 100 N (panel, E)", HL.F_MIS, "misuse", nm),
                             ("insert 28.3 N (system, C)", SYS_INSERT, "flight", nf * SYS_INSERT / HL.F_FUNC),
                             ("hand lift 85.8 N (system, C)", SYS_LIFT, "misuse", nm * SYS_LIFT / HL.F_MIS)):
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
    if V8:
        pua, pub, _, Jp = build(plain=True); F_U, U_NAME = (pua, pub), "v8 plain"
        out.update({"A_v8_plain": pua, "B_v8_plain": pub})
    elif have_plain:
        pua, pub, _, Jp = build(plain=True); F_U, U_NAME = (pua, pub), "v7 plain"
        out.update({"A_v7_plain": pua, "B_v7_plain": pub})
    else:
        F_U, U_NAME = F_v6, "v6"
    pa, pb, wA, J = build(plain=False)
    if not V8: out.update({"A_v7": pa, "B_v7": pb})               # a --v8 run builds v7 for the collar only, files untouched
    PK = "A_v8_plain" if V8 else "A_v7_plain" if PO.PLAIN else "A_v7"
    res = dict(part="funnel_panel", version="v8_plain" if V8 else "v7", skin=SKIN, rib_h=float(J["h"]), rib_vf=J.get("vf"),
               rib_vf_plain=Jp.get("vf") if (have_plain or V8) else None, unlatched_faces=U_NAME)
    if V8:
        MODE["plain"] = True; v7_globals()
        res["v8_geometry"] = dict(z_bottom=PO.Z_BOT, v_bottom_from_v7_edge=V_BOT, opening_half_x=round(F.K0 + PO.Z_BOT, 2),
                                  seat_v_from_new_edge=round(F.SEAT_V - V_BOT, 2), seat_v_from_v7_edge=round(F.SEAT_V, 2),
                                  screw_entry_seat=[round(v, 2) for v in F.P_FACE],
                                  landing_seat=[round(v, 2) for v in F.P_FACE + F.CLAMP * F.N_IN],
                                  post_rise_z=round(float(V_BOT / S2), 2),
                                  key_v_from_new_edge=[round(v - V_BOT, 2) for v in F.PEG_V],
                                  lower_key_wall_to_new_edge=round(F.PEG_V[0] - V_BOT - F.PEG_D / 2 - F.CLR, 2),
                                  wheel_contact_v_from_new_edge=round(PO.V_WHEEL - V_BOT, 2))
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
    if V8:
        # the opening: nothing of the collar inside |x| < 95.5, |y| < 84 below seat z 12.8 (the latch v1 cartridge set
        # 190.7 x ... at +-5 deg yaw passes there); the v7 collar for comparison
        hx, hy = F.K0 + PO.Z_BOT, F.K0 + F.ZB
        prism = F.box(-hx + 0.01, hx - 0.01, -hy + 0.01, hy - 0.01, -20.0, PO.Z_BOT - 0.01)
        v7c = [m.rotate([0, -F.ROT, 0]).rotate([0, 0, 90 * k]) for k in range(4) for m in (pa, pb)]   # v7 edge on all 4
        res["opening"] = dict(rect_mm=[round(2 * hx, 1), round(2 * hy, 1)], below_z=PO.Z_BOT,
                              collar_v8_in_rect_mm3=round(HL.vol(full ^ prism), 3),
                              collar_all_v7_in_rect_mm3=round(sum(HL.vol(m ^ prism) for m in v7c), 1))
    # FE: the as-built rib outline on the plate model of this run's mode (latched, or plain with --plain), every case
    MODE["plain"] = PO.PLAIN; v7_globals()
    res["fe_mode"] = "v8 plain" if V8 else "v7 plain" if PO.PLAIN else "v7 latched"
    res["fe_asbuilt"] = {k: v for k, v in PO.report(asbuilt_mask()).items()}
    if V8:
        # face-wheel loads of the catch system (hook latch v2 system.json, C) at the seated wheel spot of a +-x face
        # (u 100, v 43.6: the 20 N press there, linear plate FE): Scout per face wheel (max), Overseer on the plain faces
        sj = json.load(open(os.path.join(ROOT, "runs", "hook_latch", "v2", "system.json")))
        d0, s0 = res["fe_asbuilt"]["per_case"][f"press u100 v{PO.V_WHEEL}"]
        rows = [("Scout " + k, v["face_wheel_max_N"]) for k, v in sj["cases"].items()] + \
               [("Overseer " + k, v.get("face_wheel_plain_N", v.get("face_wheel_v7_plain_N"))) for k, v in sj["overseer"]["cases"].items()]
        ws = {nm: dict(load_N=Fw, stress_MPa=round(s0 * Fw / 20.0, 2), index=round(s0 * Fw / 20.0 / PO.SIG_ALLOW, 3),
                       defl_mm=round(d0 * Fw / 20.0, 3)) for nm, Fw in rows}
        wk = max(ws, key=lambda k: ws[k]["index"])
        res["wheel_spot_system"] = dict(case=f"press u100 v{PO.V_WHEEL} (20 N)", per_20N=[d0, s0], allow_MPa=PO.SIG_ALLOW,
                                        rows=ws, worst=[wk, ws[wk]["index"]])
    if not PO.PLAIN:
        res["boss_hand_calc"] = boss_hand_calc()
        # the system loads on the same as-built model (linear: the latch cases scale with the load)
        pc = res["fe_asbuilt"]["per_case"]; bd = res["fe_asbuilt"]["latch_boss_defl_mm"]
        sc = {"latch functional 30 N x 2": ("insert 28.3 N x 2 (system)", SYS_INSERT / HL.F_FUNC, 1.0),
              "latch misuse 100 N": ("hand lift 85.8 N (system)", SYS_LIFT / HL.F_MIS, 3.0)}
        res["fe_asbuilt_system_loads"] = {nm: dict(mean_defl_mm=round(pc[k][0] * f, 3), util=round(pc[k][0] * f / lim, 3),
                                                   stress_MPa=round(pc[k][1] * f, 2), boss_defl_mm=round(bd[k] * f, 3))
                                          for k, (nm, f, lim) in sc.items()}
        res["fe_asbuilt_panel_loads"] = {k: dict(mean_defl_mm=pc[k][0], util=round(pc[k][0] / sc[k][2], 3), stress_MPa=pc[k][1],
                                                 boss_defl_mm=bd[k]) for k in sc}
    # insert holes clear (review 1): panel material inside the 4 hole cylinders, panel A world
    if not PO.PLAIN:
        MODE["plain"] = False; v7_globals()
        holes = mf.Manifold()
        for (bu, bv) in P7["bolts"]: holes = holes + to_mf(HL.cyl_w(2.0 - 0.05, bu, bv, HL.W_FL0 - 0.05, -SKIN - 0.55), T_A)
        res["insert_holes_material_mm3"] = round(HL.vol(wA ^ holes), 3)
    # solid gate on the exported files, and the index row (runs_index.py)
    from solid_check import solid_check
    sol = {}
    for name, p in names.items():
        ok, info = solid_check(trimesh.load(p, force="mesh"))
        sol[name] = dict(ok=bool(ok), bodies_eroded=info.get("bodies_eroded"), necks=len(info.get("necks", [])),
                         thin_walls=len(info.get("thin_walls", [])))
    res["solid_check"] = sol
    fe = res["fe_asbuilt"]
    res.update(date=__import__("datetime").date.today().isoformat(), status="DONE",
               mass_g=res[PK]["g_PETG"], mass_tag="M",
               worst_index=fe["util"], worst_case=fe["worst"], solid_ok=all(v["ok"] for v in sol.values()),
               bed_face="contact face", bed_contact_mm2=res[PK]["bed_mm2"])
    json.dump(res, open(os.path.join(RUN, "result_plain.json" if PO.PLAIN and not V8 else "result.json"), "w"), indent=1,
              default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    ft.export(os.path.join(RUN, "collar_v8_seat_frame.stl" if V8 else "collar_v7_seat_frame.stl"))
    print(json.dumps({k: v for k, v in res.items() if k not in ("fe_asbuilt",)}, indent=1, default=str))
    print("fe as-built:", res["fe_mode"], {k: v for k, v in res["fe_asbuilt"].items() if k != "per_case"})
    for name, p in names.items(): shutil.copy(p, os.path.join(ROOT, "print", os.path.basename(p)))
    if "--motion" in sys.argv: make_motion(pa, pb)
