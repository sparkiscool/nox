"""Box collar funnel panel v9 plain (24 Oct 2026): the 4 panels of the unlatched faces (drone +-x), v8 plain + two changes.
v8 plain (funnel_panel_v7.py --v8: bottom edge at seat z 12.8, opening 191 x 168, seat landing clamped on its post, the
wheel-track strut) is kept as it is; added:
  A. keep-alive spring probes (Spark 24 Oct: keep-alive stays, face probes; concept runs/keepalive_contacts/concept/): in
     panel A of each +-x face, 2 probes at panel u 12 and 22 from the centre seam, tips on the contact face at seat
     (+-115.7, +-12 / +-22, 33), axis along the face normal. Each: o3.0 brass pin in a o3.2 guide bore (skin + 4 of boss),
     a brass collar o4.0 x 3.0 soldered on it stops on the floor of a o5.0 spring bore (free tip 8.0 off the face), a
     compression spring between the collar and a printed back cap (fully pushed tip 0.5 off the face: stroke 7.5), the
     silicone lead soldered to the pin tail behind the cap, cable-tied in a drawn groove round the boss head. One drawn boss
     per panel A (both bores, the cap screw lobe), wall >= 3 round every bore, gussets into the seam flange and the
     seat strut; one cap per panel A (M2 x 6 self-tapping into a o1.6 pilot).
  B. arm notch (box structure, hub DEFAULT 24 Oct): the -x face panels (W -Y side: seat y -> W -X, seat x -> W Y) keep
     2.0 (DEFAULT, E) off the drone insert lane: carriage X -35..47.5, Y <= 73.75, r <= 159 and the head / grabber
     X +-47.5, Y 73.75..217, r 111.5..159.6 about the axle (W Y axis through X 0, Z 0): r >= 161 over X -37..49.5,
     Y <= 75.75 (a cylinder about the axle, square-ended at the X planes). v8 plain already clears both by 0.61 (C on M):
     the notch only trims the centre seam flange's heel (a few mm3 a panel).
Files: A px (probes), B px (= v8 plain B, renamed), A nx (probes + notch), B nx (notch), probe cap (x2).
python3 funnel_panel_v9.py -> runs/funnel_panel/v9_plain/ (STLs, result.json, sections), print/ (READY files),
                               runs/box_catch_assembly/v2/ (seat meshes) and nox/box/box_catch.assembly.json"""
import math, json, os, sys, shutil, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
os.environ["NOX_FUNNEL_PLAIN"] = "1"; os.environ["NOX_FUNNEL_ZBOT"] = "12.8"
sys.path[:0] = [HERE, os.path.dirname(HERE), os.path.join(ROOT, "nox")]
import numpy as np, trimesh, manifold3d as mf
import funnel_panel_v7 as FV
F, PO = FV.F, FV.PO
assert FV.V8 and PO.SEAT_SUP
RUN = os.path.join(ROOT, "runs", "funnel_panel", "v9_plain"); os.makedirs(RUN, exist_ok=True)
VER = "v9_plain"
S2 = math.sqrt(2)
E_U, E_V, N_IN = F.E_U, F.E_V, F.N_IN

# ---------------------------------------------------------------- A. probe (C geometry, E bought parts)
PROBE_Z = 33.0                                   # seat z of the tips on the contact face (keep-alive concept, C)
PROBE_V = (PROBE_Z - F.ZB) * S2                  # 44.83 up the slope from the v7 edge (28.57 from the v8 edge, C)
PROBE_U = (12.0, 22.0)                           # from the centre seam (C)
TIP_FREE, TIP_SOLID = 8.0, 0.5                   # free tip / fully pushed tip off the face (C)
STROKE = TIP_FREE - TIP_SOLID                    # 7.5
PIN_D, PIN_L, BORE_PIN = 3.0, 36.5, 3.2          # brass rod o3.0 (E, buy), guide bore o3.2 (C)
GUIDE_W = 6.0                                    # guide bore depth from the face (skin 2.0 + 4.0 of boss) = the collar stop (C)
COLLAR_D, COLLAR_ID, COLLAR_L = 4.0, 3.0, 3.0    # brass tube 4 / 3 x 3 long, soldered 14.0 from the tip (E)
BORE_SPR = 5.0                                   # spring bore (C)
# spring (E, buy; music wire): OD 4.0, wire 0.25, free length 25.0, ~16.3 active coils, rate 0.045 N/mm, solid ~4.6
SPR_OD, SPR_WIRE, SPR_L0, SPR_K, SPR_SOLID = 4.0, 0.25, 25.0, 0.045, 4.6
L_INST = 13.0                                    # spring length at the free tip (preload 12.0 x 0.045 = 0.54 N, C)
W_BACK = GUIDE_W + COLLAR_L + L_INST             # 22.0: boss back face = cap seat (C)
CAP_T, CAP_HOLE = 2.4, 3.3                       # cap plate (PETG), tail hole o3.3 (drill after print)
SCREW_HOLE, PILOT_D, PILOT_DEPTH = 2.2, 1.6, 6.0 # M2 x 6 pan head self-tapping (E, buy) through the cap into a o1.6 pilot
BOSS_R = 6.5                                     # boss round each bore: wall 4.0 (>= 3 after the 0.8 tie groove: 3.2)
LOBE_UV, LOBE_R = (17.0, PROBE_V + 6.0), 3.8     # cap screw lobe: pilot wall 3.0, 4.5 to each spring bore
GROOVE_W, GROOVE_D = (12.8, 15.4, 16.2), 0.8     # cable-tie groove round the boss head: floor 13.0, flat to 15.4, 45 deg roof to 16.2
GUSSET_W = 4.4                                   # drawn gussets (the panel's tie rib width): seam flange and seat strut
GUSSETS = [((0.0, PROBE_V), (8.0, PROBE_V)), ((26.0, PROBE_V), (34.0, PROBE_V))]
LEAD = "20 AWG silicone, one per pin, joined to the panel's 3.5 mm bullet lead (box electronics v1)"
PAD_OFF = (1.0, 2.0, 7.0)                        # drone pad off the face: most compressed, seated, least (C, keep-alive concept)


def spring_force(s):
    """spring force (N) with the pin pushed s from its free position (C on the E spring)."""
    return SPR_K * (SPR_L0 - L_INST + s)


def p_face(u, v):
    return np.array([84.0 + v / S2, u, F.ZB + v / S2])


M_FACE = np.column_stack([E_U, E_V, N_IN, p_face(0.0, 0.0)])     # local (u, v, w into the panel) -> panel A world


def to_world(m):
    return m.transform(M_FACE.tolist())


def prism(cs, w0, w1):
    return to_world(mf.Manifold.extrude(cs, w1 - w0).translate([0.0, 0.0, w0]))


def circ(r, c, n=64):
    return mf.CrossSection.circle(r, n).translate(c)


def boss_cs():
    return mf.CrossSection.batch_hull([circ(BOSS_R, (u, PROBE_V)) for u in PROBE_U] + [circ(LOBE_R, LOBE_UV)])


def cyl_local(r, w0, w1, u, v, n=48):
    return mf.Manifold.cylinder(w1 - w0, r, r, n).translate([u, v, w0])


def probe_boss():
    """the drawn boss (both bores, lobe), its gussets, less the tie groove, in panel A world (no bores yet)."""
    cs = boss_cs()
    w0, w1, w2 = GROOVE_W
    # foot (the outline up to the groove floor), core through the groove, 45 deg roof back to the outline, head to the back
    b = prism(cs, 0.5, w0 + 0.2) + mf.Manifold.batch_hull([prism(cs.offset(-GROOVE_D, mf.JoinType.Round), w0, w1),
                                                         prism(cs, w2, W_BACK)])
    g = mf.Manifold()
    for a, c in GUSSETS:
        g = g + prism(mf.CrossSection.batch_hull([circ(GUSSET_W / 2, a, 24), circ(GUSSET_W / 2, c, 24)]), 0.5, F.T + F.RIB_H)
    out = b + g
    out = F.plane_keep(out, (0, 1, 0), 0.0)                          # nothing past the centre seam (panel A is y >= 0)
    return out - F.post_room() - F.screw_hole()


def probe_bores():
    m = mf.Manifold()
    for u in PROBE_U:
        m = m + cyl_local(BORE_PIN / 2, -1.5, GUIDE_W, u, PROBE_V) + cyl_local(BORE_SPR / 2, GUIDE_W, W_BACK + 1.0, u, PROBE_V)
    m = m + cyl_local(PILOT_D / 2, W_BACK - PILOT_DEPTH, W_BACK + 1.0, *LOBE_UV)
    return to_world(m)


def cap_local():
    """probe cap, local (u, v, w): plate w W_BACK .. + CAP_T on the boss outline, 2 tail holes, the screw hole."""
    p = mf.Manifold.extrude(boss_cs(), CAP_T).translate([0, 0, W_BACK])
    for u in PROBE_U: p = p - cyl_local(CAP_HOLE / 2, W_BACK - 1, W_BACK + CAP_T + 1, u, PROBE_V)
    return p - cyl_local(SCREW_HOLE / 2, W_BACK - 1, W_BACK + CAP_T + 1, *LOBE_UV)


def pin_local(s, u):
    """brass pin + soldered collar, pushed s from free (tip at w -TIP_FREE + s)."""
    w_tip = -TIP_FREE + s
    r = PIN_D / 2
    rod = mf.Manifold.batch_hull([mf.Manifold.sphere(r, 32).translate([u, PROBE_V, w_tip + r]),
                                  mf.Manifold.cylinder(PIN_L - r, r, r, 32).translate([u, PROBE_V, w_tip + r])])
    col = cyl_local(COLLAR_D / 2, GUIDE_W + s, GUIDE_W + COLLAR_L + s, u, PROBE_V)
    return rod + col


def spring_local(s, u):
    w0, w1 = GUIDE_W + COLLAR_L + s, W_BACK
    return cyl_local(SPR_OD / 2, w0, w1, u, PROBE_V) - cyl_local(SPR_OD / 2 - SPR_WIRE, w0 - 1, w1 + 1, u, PROBE_V)


def lead_keepout_local(u):
    """pin tail travel + solder joint + lead bend behind the cap (E): r 3.0 from the cap back to w 40."""
    return cyl_local(3.0, W_BACK + CAP_T, 40.0, u, PROBE_V)


# ---------------------------------------------------------------- B. arm notch (seat frame of the -x face via each panel's own world)
CLR = 2.0                                        # margin over the arm's swept keep-outs (DEFAULT, E; hub: >= 2)
LANE = dict(r=159.0, x=(-35.0, 47.5), y1=73.75)  # carriage (dimensions L v2 layout, box_layout_v2.LANE, C)
GRAB = dict(r0=111.5, r=159.6, x=47.5, y=(73.75, 217.0))
Y_C, Z_SEAT = 165.0, 155.3   # the notch as drawn and printed (v9 plain); the catch now sits at Y_C 167.1 (catch_system.py) and the -Y panel still clears the arm lane by 2.0 in r (box structure, C on M)


def notch_cut():
    """in a panel's own world (+x face frame) placed on the -x face: W X = y, W Y = Y_C - x, W Z = z + Z_SEAT; removes
    r < 159 + CLR about the axle over X -37..49.5, W Y <= 75.75 (x >= 89.25), square-ended at the X planes."""
    R = LANE["r"] + CLR
    cyl = mf.Manifold.cylinder(400.0, R, R, 512).rotate([0, 90, 0]).translate([0.0, 0.0, -Z_SEAT])
    return cyl ^ F.box(Y_C - LANE["y1"] - CLR, 400.0, LANE["x"][0] - CLR, LANE["x"][1] + CLR, -400.0, 50.0)


def grab_zone():
    R0, R1 = GRAB["r0"] - CLR, GRAB["r"] + CLR
    ring = mf.Manifold.cylinder(400.0, R1, R1, 512) - mf.Manifold.cylinder(402.0, R0, R0, 512).translate([0, 0, -1])
    ring = ring.rotate([0, 90, 0]).translate([0.0, 0.0, -Z_SEAT])
    return ring ^ F.box(Y_C - GRAB["y"][1], Y_C - GRAB["y"][0] + CLR, -GRAB["x"] - CLR, GRAB["x"] + CLR, -400.0, 50.0)


# ---------------------------------------------------------------- build
def panel_worlds():
    """(plain A world, A world with probes): v8 plain A as funnel_panel_v7 builds it, then the probe boss and bores."""
    FV.MODE["plain"] = True
    w, _ = FV.panel_A_world()
    boss = probe_boss()
    wp = (w + boss) - probe_bores()
    return w, wp, boss


def to_print_pair(wA, wB):
    pa = F.to_print(wA); pb = F.to_print(wB)
    for p in F.key_points():
        q = F.to_print_pt(p)
        pa = pa + F.teardrop(F.PEG_D, F.PEG_L).translate([q[0], 0.0, q[2]])
        pb = pb - F.teardrop(F.PEG_D + 2 * F.CLR, F.FL_T + 2).translate([q[0], 1.0, q[2]])
    return pa, pb


def build():
    w, wp, boss = panel_worlds()
    nc = notch_cut()
    wB = w.mirror([0, 1, 0])
    worlds = dict(A_px=wp, B_px=wB, A_nx=wp - nc, B_nx=wB - nc)
    pa_px, pb_px = to_print_pair(worlds["A_px"], worlds["B_px"])
    pa_nx, pb_nx = to_print_pair(worlds["A_nx"], worlds["B_nx"])
    return dict(A_px=pa_px, B_px=pb_px, A_nx=pa_nx, B_nx=pb_nx), worlds, w, boss


def seat_pose(face):
    """seat frame pose of a panel world: +x face k 0, -x face k 2 (Rz 180)."""
    k = {"+x": 0, "+y": 1, "-x": 2, "-y": 3}[face]
    c, s = [round(v) for v in (math.cos(math.radians(90 * k)), math.sin(math.radians(90 * k)))]
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def xf(m, T):
    return m.transform(np.asarray(T)[:3, :4].tolist())


def vol(m):
    return float(m.volume())


def zeroed_tm(m):
    t = FV.tm(m)
    off = -np.array([t.bounds[:, 0].mean(), t.bounds[:, 1].mean(), t.bounds[0, 2]])
    t.apply_translation(off)
    return t, off


# ---------------------------------------------------------------- FE (panel_opt_v7 plate model, seat-landing supports)
def fe_face(face, notch_mask=True, probes=True):
    """as-built plate FE of one face (A + B as one plate, PO's model with NOX_FUNNEL_SUPPORT=seat, as v8 plain), with the
    v9 changes: the probe boss as full rib height (skin + 9 = 11.0, conservative: it stands 22 deep) over its outline on
    the A half, the two o5.0 bores as holes (0.5), gussets left out (conservative); the -x face adds the notch (each
    element's thickness capped at the material left above the cut). Cases: v8 plain's, mirrored onto the B half, plus
    the probes (2 x 1.0 N along the normal on the A half, into the panel) and the seated wheel spot on both halves.
    Reports per half (A u >= 0, B u <= 0): worst deflection index (the panels' design index, C) and stress."""
    from matplotlib.path import Path
    FV.MODE["plain"] = True; FV.v7_globals()
    mask = FV.asbuilt_mask()
    keep = {k: getattr(PO, k) for k in ("F", "Pm", "LIM", "names", "norib", "fixedT", "T_PASS")}
    try:
        ec = PO.ec; u, v = ec[:, 0], ec[:, 1]
        T_PASS, fixedT, norib = PO.T_PASS.copy(), PO.fixedT.copy(), PO.norib.copy()
        inb = np.zeros(len(ec), bool)
        for poly in boss_cs().to_polygons():
            inb |= Path(np.asarray(poly)).contains_points(ec)
        inb &= (u > 0) & probes
        bore = np.zeros(len(ec), bool)
        for pu in PROBE_U: bore |= (np.hypot(u - pu, v - PROBE_V) <= BORE_SPR / 2) & probes
        T_PASS = np.where(inb, F.T + F.RIB_H, T_PASS); T_PASS = np.where(bore, 0.5, T_PASS)
        fixedT = fixedT | inb | bore; norib = norib | inb | bore
        notch_el = np.zeros(len(ec), bool)
        if face == "nx" and notch_mask:
            R = LANE["r"] + CLR
            inx = (u >= LANE["x"][0] - CLR) & (u <= LANE["x"][1] + CLR)
            zc = np.sqrt(np.maximum(R ** 2 - u ** 2, 0.0)) - Z_SEAT        # cut below this seat z
            wmax = v - (zc - F.ZB) * S2                                     # material depth left above the cut
            t_now = np.where(fixedT, T_PASS, np.where(mask, PO.TS + PO.RIB_H, PO.TS))
            notch_el = inx & (wmax < t_now)
            T_PASS = np.where(notch_el, np.maximum(wmax, 0.5), T_PASS)
            fixedT = fixedT | notch_el; norib = norib | notch_el
        cols, names = [PO.F[:, i].copy() for i in range(PO.F.shape[1])], list(PO.names)
        topv = PO.topv
        for n in list(names):
            if n.startswith("press u"):
                uu, vv = n.split()[1:3]; uu = float(uu[1:]); vv = float(vv[1:])
                if uu > 0: cols.append(PO.patch(-uu, vv, 20.0)); names.append(f"press u{-uu:g} v{vv:g}")
            elif n.startswith("top edge u"):
                uu = float(n.split()[2][1:])
                if uu > 0: cols.append(PO.patch(-uu, topv, 20.0, r=3.0)); names.append(f"top edge u{-uu:g}")
        pr = sum(PO.patch(pu, PROBE_V, 1.0, r=BOSS_R) for pu in PROBE_U)
        cols.append(pr); names.append("probes 2 x 1.0 N")
        Fm = np.stack(cols, 1); Fm[PO.fixed, :] = 0
        Pm = np.array([np.abs(Fm[:, i]).sum() for i in range(Fm.shape[1])])

        def lim(n):
            if n.startswith("uniform") or n.startswith("probes"): return PO.DEFL
            if n.startswith("press"):
                uu = abs(float(n.split()[1][1:])); return PO.DEFL if PO.WHEEL_U[0] <= uu <= PO.WHEEL_U[1] else 3.0
            return 3.0
        LIM = np.array([lim(n) for n in names])
        PO.F, PO.Pm, PO.LIM, PO.names = Fm, Pm, LIM, names
        PO.norib, PO.fixedT, PO.T_PASS = norib, fixedT, T_PASS
        c, _, U = PO.solve(mask.astype(float), PO.TS, PO.RIB_H)
        t = PO.thick(mask.astype(float) * (~norib), PO.TS, PO.RIB_H)
        d = c / Pm
        k = np.einsum("ij,ejc->eic", PO.BC, U[PO.edof])
        D = PO.E / (1 - PO.NU ** 2) * t ** 3 / 12
        Mx = D[:, None] * (k[:, 0] + PO.NU * k[:, 1]); My = D[:, None] * (k[:, 1] + PO.NU * k[:, 0]); Mxy = D[:, None] * (1 - PO.NU) / 2 * k[:, 2]
        s_ = 6 / t[:, None] ** 2
        sx, sy, txy = s_ * Mx, s_ * My, s_ * Mxy
        vm = np.sqrt(sx ** 2 - sx * sy + sy ** 2 + 3 * txy ** 2)
        mass_g = float((t * PO.H * PO.H).sum() * PO.RHO_G)

        def case_u(n):
            if n.startswith("press u") or n.startswith("top edge u"):
                return float(n.split()[1 if n.startswith("press") else 2][1:])
            return None
        out = dict(face=face, model="as-built plate FE, NOX_FUNNEL_SUPPORT=seat (landings clamped r 8, valleys, bottom edge free)",
                   plate_face_g=round(mass_g, 1), boss_elements=int(inb.sum()), bore_elements=int(bore.sum()),
                   notch_elements=int(notch_el.sum()), halves={})
        for half, sel_u in (("A", u >= -1e-6), ("B", u <= 1e-6)):
            ci = [i for i, n in enumerate(names) if case_u(n) is None or (case_u(n) >= 0 if half == "A" else case_u(n) <= 0)]
            util = d[ci] / LIM[ci]
            st = vm[sel_u][:, ci].max(0)
            i = int(np.argmax(util)); j = int(np.argmax(st))
            per = {names[q]: [round(float(d[q]), 3), round(float(st[ii]), 2)] for ii, q in enumerate(ci)}
            wn = "press u100 v43.6" if half == "A" else "press u-100 v43.6"
            out["halves"][half] = dict(util=round(float(util.max()), 3), worst=names[ci[i]],
                                       stress_MPa=round(float(st.max()), 1), stress_case=names[ci[j]],
                                       stress_index=round(float(st.max()) / PO.SIG_ALLOW, 3),
                                       ok=bool(util.max() <= 1 + 1e-9 and st.max() <= PO.SIG_ALLOW),
                                       wheel_spot_per_20N=per[wn], probes_case=per["probes 2 x 1.0 N"], per_case=per)
        return out
    finally:
        for kk, vv in keep.items(): setattr(PO, kk, vv)


# ---------------------------------------------------------------- main
def section_png(wp, path):
    """section of one probe boss through its axis (plane u = PROBE_U[0]), drawn as (w across, v up), free and seated."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    Minv = np.linalg.inv(np.vstack([M_FACE, [0, 0, 0, 1]]))
    loc = xf(wp, Minv)                                               # panel -> local (u, v, w)
    Ts = [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, -PROBE_U[0]]]       # (u, v, w) -> (v, w, u - u0): slice at 0

    def draw(ax, m, fc, z):
        for poly in m.transform(Ts).slice(0.0).to_polygons():
            p = np.asarray(poly)
            ax.add_patch(Polygon(np.c_[p[:, 1], p[:, 0]], closed=True, fc=fc, ec="k", lw=0.6, zorder=z))
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.2))
    for ax, s, title in ((axs[0], 0.0, f"free: tip {TIP_FREE:g} off the face, {spring_force(0):.2f} N preload"),
                         (axs[1], 6.0, f"seated: pad 2.0 off, pin pushed 6.0, {spring_force(6.0):.2f} N")):
        draw(ax, loc, "#b9c2cc", 1)
        draw(ax, cap_local(), "#7fa7d9", 2)
        draw(ax, spring_local(s, PROBE_U[0]), "#555555", 3)
        draw(ax, pin_local(s, PROBE_U[0]), "#d4a017", 4)
        ax.axvline(0.0, color="k", lw=1.2); ax.text(0.4, PROBE_V + 16.5, "contact face", fontsize=8)
        ax.axvline(-TIP_FREE, color="g", ls=":", lw=0.8); ax.axvline(-TIP_SOLID, color="r", ls=":", lw=0.8)
        ax.text(-TIP_FREE + 0.2, PROBE_V - 15, "free tip", fontsize=7, color="g"); ax.text(-TIP_SOLID + 0.2, PROBE_V - 15, "solid", fontsize=7, color="r")
        if s > 0:
            ax.add_patch(Polygon([[-3.6, PROBE_V - 8], [-2.0, PROBE_V - 8], [-2.0, PROBE_V + 4], [-3.6, PROBE_V + 4]],
                                 closed=True, fc="#c87533", ec="k", lw=0.6, zorder=5))
            ax.text(-11.5, PROBE_V + 5, "drone pad\n(copper 1.6, 12 along v)", fontsize=8)
        ax.text(W_BACK + CAP_T + 0.5, PROBE_V + 2, "cap 2.4 (M2 x 6)\nlead soldered behind,\ntie in the groove", fontsize=7)
        ax.set_xlim(-12, 42); ax.set_ylim(PROBE_V - 16, PROBE_V + 18); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_xlabel("w: depth behind the contact face along the probe axis (mm)"); ax.set_ylabel("v: up the slope from the v7 edge (mm)")
        ax.set_title(title, fontsize=10)
    fig.suptitle(f"Funnel panel A v9 plain: keep-alive probe at u {PROBE_U[0]:g}, section through its axis (C geometry; pin, collar, "
                 f"spring E). Bores o{BORE_PIN} x {GUIDE_W:g} / o{BORE_SPR}, boss wall {BOSS_R - BORE_SPR / 2:.1f} (3.2 at the tie groove), "
                 f"cap {CAP_T}; spring OD {SPR_OD} x {SPR_WIRE} wire x {SPR_L0:g} free, k {SPR_K} N/mm", fontsize=9)
    plt.tight_layout(); plt.savefig(path, dpi=110); plt.close(fig)


def main():
    t0 = time.time()
    files, worlds, w_plain, boss = build()
    res = dict(part="funnel_panel", version=VER, date=__import__("datetime").date.today().isoformat())
    # ---- export (print frame, zeroed as v8) + seat meshes for the assembly
    names = {}
    for k, m in files.items():
        t, off = zeroed_tm(m)
        p = os.path.join(RUN, f"box_funnel_panel_{k[0]}_{VER}_{k[2:]}.stl"); t.export(p); names[k] = p
        res[k] = F.info_of(t); res[k]["watertight_after_stl_reload"] = bool(trimesh.load(p, force="mesh").is_watertight)
    capl = cap_local()
    capt = FV.tm(capl.translate([0, 0, -W_BACK]))
    capt.apply_translation(-np.array([capt.bounds[:, 0].mean(), capt.bounds[:, 1].mean(), capt.bounds[0, 2]]))
    pc = os.path.join(RUN, f"box_funnel_probe_cap_{VER}.stl"); capt.export(pc); names["cap"] = pc
    res["cap"] = F.info_of(capt)
    # ---- probe geometry and fits (local frame of panel A, C)
    Minv = np.linalg.inv(np.vstack([M_FACE, [0, 0, 0, 1]]))
    wl = xf(worlds["A_px"], Minv)
    fit = {}
    for s in (0.0, 6.0, STROKE):
        fit[f"pins_s{s:g}_mm3"] = round(sum(vol(wl ^ pin_local(s, u)) for u in PROBE_U), 4)
        fit[f"springs_s{s:g}_mm3"] = round(sum(vol(wl ^ spring_local(s, u)) for u in PROBE_U), 4)
        fit[f"pins_vs_cap_s{s:g}_mm3"] = round(sum(vol(capl ^ pin_local(s, u)) for u in PROBE_U), 4)
    fit["cap_vs_panel_mm3"] = round(vol(wl ^ capl), 4)
    fit["boss_in_front_of_face_mm3"] = round(vol(xf(boss, Minv) ^ F.box(-60, 60, 0, 120, -40, 0.0)), 4)
    fit["spring_space_free_mm"] = W_BACK - GUIDE_W - COLLAR_L
    fit["spring_space_solid_tip_mm"] = W_BACK - GUIDE_W - COLLAR_L - STROKE
    fit["tail_behind_cap_free_mm"] = round(-TIP_FREE + PIN_L - (W_BACK + CAP_T), 2)
    fit["tail_end_w_max_mm"] = round(-TIP_SOLID + PIN_L, 2)
    # keep-outs inside the panel: seat block, post room, screw hole, keys, valley, wheel track
    FV.MODE["plain"] = True; FV.v7_globals()
    keys = mf.Manifold()
    for p in F.key_points():
        q = F.to_print_pt(p)
        keys = keys + F.teardrop(F.PEG_D + 2 * F.CLR, F.FL_T + 2).translate([q[0], 1.0, q[2]]).rotate([0, -F.ROT, 0])
    boss_all = boss + to_world(capl)
    near = dict(seat_block_mm3=round(vol(boss_all ^ F.seat_block()), 3), post_room_mm3=round(vol(boss_all ^ F.post_room()), 3),
                screw_hole_mm3=round(vol(boss_all ^ F.screw_hole()), 3), key_holes_mm3=round(vol(boss_all ^ keys), 3))
    pts = np.asarray(FV.tm(boss_all).vertices)
    near["boss_min_gap_to_valley_plane_mm"] = round(float(((pts[:, 0] - pts[:, 1]) / S2).min()), 2)
    near["boss_u_range"] = [round(float(pts[:, 1].min()), 2), round(float(pts[:, 1].max()), 2)]
    near["boss_gap_to_seat_flare_mm"] = round(boss_all.min_gap(F.seat_block(), 30.0), 2)
    near["boss_gap_to_post_room_mm"] = round(boss_all.min_gap(F.post_room(), 30.0), 2)
    near["boss_gap_to_wheel_track_u50_mm"] = round(50.0 - (BOSS_R + max(PROBE_U)), 2)
    near["gusset_into_ribs_mm3"] = round(vol(boss ^ w_plain), 2)                   # the boss's overlap with v8 plain (seam flange, strut)
    res["probe"] = dict(tips_seat=[[round(float(v), 2) for v in p_face(u, PROBE_V)] for u in PROBE_U],
                        axis=[round(float(v), 4) for v in -N_IN], v_from_v8_edge=round(PROBE_V - FV.V_BOT, 2),
                        bores=dict(pin=BORE_PIN, guide_depth=GUIDE_W, spring=BORE_SPR, back_face_w=W_BACK, wall=BOSS_R - BORE_SPR / 2,
                                   wall_at_groove=round(BOSS_R - BORE_SPR / 2 - GROOVE_D, 2)),
                        pin=dict(d=PIN_D, L=PIN_L, collar=[COLLAR_D, COLLAR_ID, COLLAR_L], collar_from_tip=TIP_FREE + GUIDE_W),
                        spring=dict(od=SPR_OD, wire=SPR_WIRE, L0=SPR_L0, k_N_per_mm=SPR_K, solid=SPR_SOLID, installed_free=L_INST,
                                    at_full_stroke=L_INST - STROKE),
                        travel=dict(free=TIP_FREE, solid=TIP_SOLID, stroke=STROKE),
                        force_N={f"pad {p:g} off": round(spring_force(TIP_FREE - p), 3) for p in PAD_OFF} |
                                {"preload (free)": round(spring_force(0.0), 3), "solid tip": round(spring_force(STROKE), 3)},
                        fits=fit, keepouts_inside_panel=near)
    # ---- notch
    nc = notch_cut()
    ncA = vol(worlds["A_px"] ^ nc); ncB = vol(worlds["B_px"] ^ nc)
    wheel = mf.Manifold()
    for su in (1, -1):
        wheel = wheel + prism(mf.CrossSection([[(su * 50.0, 20.0), (su * 150.0, 20.0), (su * 150.0, 70.0), (su * 50.0, 70.0)]]), -0.5, F.T + 0.5)
    res["notch"] = dict(clearance=CLR, r_min=LANE["r"] + CLR, x_range_W=[LANE["x"][0] - CLR, LANE["x"][1] + CLR], y_max_W=LANE["y1"] + CLR,
                        removed_mm3=dict(A=round(ncA, 2), B=round(ncB, 2)),
                        vs_seat_block_mm3=round(vol(nc ^ F.seat_block()) + vol(nc ^ F.seat_block().mirror([0, 1, 0])), 3),
                        vs_probe_boss_mm3=round(vol(nc ^ boss), 3), vs_wheel_zone_mm3=round(vol(nc ^ wheel), 3),
                        grab_zone_vs_panels_mm3=round(vol(worlds["A_nx"] ^ grab_zone()) + vol(worlds["B_nx"] ^ grab_zone()), 3))
    # ---- arm keep-out meshes (box_structure_v2.assembly.json, W) vs the -x face panels and both faces' probe hardware
    sys.path.insert(0, os.path.dirname(HERE))
    import assembly as AS, catch_system as CS
    W = CS.SEAT_TO_W
    st = json.load(open(os.path.join(os.path.dirname(HERE), "box_structure_v2.assembly.json")))

    def man(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
    ko = {p["name"]: man(trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")) for p in st["parts"]
          if p.get("mesh") and not p["name"].startswith("catch")}
    lane = {k: v for k, v in ko.items() if "insert lane" in k}
    rows = {}
    for k in ("A_nx", "B_nx", "A_px", "B_px"):
        face = "-x" if k.endswith("nx") else "+x"
        mW = xf(worlds[k], W @ seat_pose(face))
        rows[k] = {ln: dict(overlap_mm3=round(vol(mW ^ m), 3), min_gap_mm=round(mW.min_gap(m, 30.0), 2)) for ln, m in lane.items()}
    v8 = {}
    for k, wv in (("A_v8", w_plain), ("B_v8", w_plain.mirror([0, 1, 0]))):
        mW = xf(wv, W @ seat_pose("-x"))
        v8[k] = {ln: dict(overlap_mm3=round(vol(mW ^ m), 3), min_gap_mm=round(mW.min_gap(m, 30.0), 2)) for ln, m in lane.items()}
    hw = {}
    for face in ("+x", "-x"):
        T = W @ seat_pose(face)
        hwm = to_world(capl)
        for u in PROBE_U: hwm = hwm + to_world(pin_local(STROKE, u)) + to_world(lead_keepout_local(u))
        hwm = xf(hwm + boss, T)
        hits = {n: round(vol(hwm ^ m), 3) for n, m in ko.items()}
        hw[face] = dict(overlap_total_mm3=round(sum(hits.values()), 3), worst=max(hits, key=hits.get),
                        min_gap_mm=round(min(hwm.min_gap(m, 30.0) for m in ko.values()), 2))
    res["arm_keepouts"] = dict(source="nox/box/box_structure_v2.assembly.json (drone insert lane meshes, M; arm PR 79 swept, C)",
                               v9=rows, v8_plain_for_reference=v8,
                               probe_hardware_vs_box_structure=hw)
    # ---- collar: v7 on +-y, v9 plain px on +x, nx on -x
    pa7, pb7, _, _ = FV.build(plain=False)
    FV.MODE["plain"] = True; FV.v7_globals()
    parts = []
    for kq in range(4):
        a, b = {0: (files["A_px"], files["B_px"]), 1: (pa7, pb7), 2: (files["A_nx"], files["B_nx"]), 3: (pa7, pb7)}[kq]
        for m in (a, b): parts.append(m.rotate([0, -F.ROT, 0]).rotate([0, 0, 90 * kq]))
    ov = sum(vol(parts[i] ^ parts[j]) for i in range(8) for j in range(i + 1, 8))
    full = parts[0]
    for q in parts[1:]: full = full + q
    ft = FV.tm(full)
    res["collar"] = dict(overlap_mm3=round(ov, 3), bodies=sum(abs(b.volume) > 1.0 for b in ft.split(only_watertight=False)),
                         g_PETG=round(ft.volume / 1000 * 1.27, 1))
    hx, hy = F.K0 + PO.Z_BOT, F.K0 + F.ZB
    prism_open = F.box(-hx + 0.01, hx - 0.01, -hy + 0.01, hy - 0.01, -20.0, PO.Z_BOT - 0.01)
    res["opening_prism_191x168_mm3"] = round(vol(full ^ prism_open), 3)
    res["latch_fit_v7_skin"] = FV.latch_fit(FV.P7, full)
    res["wheel_path"] = FV.wheel_path_steps(ft)
    ft.export(os.path.join(RUN, "collar_v9_seat_frame.stl"))
    # ---- FE per face
    res["fe"] = {f: fe_face(f) for f in ("px", "nx")}
    # ---- solid gate on the exact files
    from solid_check import solid_check
    sol = {}
    for k, p in names.items():
        ok, info = solid_check(trimesh.load(p, force="mesh"))
        sol[k] = dict(ok=bool(ok), bodies=info.get("bodies"), bodies_eroded=info.get("bodies_eroded"), necks=len(info.get("necks", [])),
                      thin_walls=len(info.get("thin_walls", [])), drawn_not_in_main=info.get("drawn_not_in_main"))
        print("solid_check", os.path.basename(p), "PASS" if ok else "FAIL", sol[k], flush=True)
    res["solid_check"] = sol
    res["solid_ok"] = all(v["ok"] for v in sol.values())
    worst = max(((f"{fc} {h}", r["util"]) for fc, v in res["fe"].items() for h, r in v["halves"].items()), key=lambda t: t[1])
    res.update(status="DONE", worst_index=worst[1], worst_case=worst[0], mass_g=res["A_nx"]["g_PETG"], mass_tag="M",
               bed_face="contact face", bed_contact_mm2=res["A_px"]["bed_mm2"])
    # ---- section image of one probe boss
    section_png(worlds["A_px"], os.path.join(RUN, "probe_section.png"))
    # ---- seat meshes for the assembly (panels, caps, pins at seated) -> runs/box_catch_assembly/v2/
    od = os.path.join(ROOT, "runs", "box_catch_assembly", "v2"); os.makedirs(od, exist_ok=True)
    seat = {}
    for k, m in files.items():
        nm = f"funnel_panel_{k[0]}_{VER}_{k[2:]}_seat.stl"; FV.tm(FV.to_seat(m)).export(os.path.join(od, nm)); seat[k] = f"runs/box_catch_assembly/v2/{nm}"
    FV.tm(to_world(capl)).export(os.path.join(od, "probe_cap_seat.stl")); seat["cap"] = "runs/box_catch_assembly/v2/probe_cap_seat.stl"
    for u in PROBE_U:
        nm = f"probe_pin_u{u:g}_free_seat.stl"
        FV.tm(to_world(pin_local(0.0, u))).export(os.path.join(od, nm)); seat[f"pin{u:g}"] = f"runs/box_catch_assembly/v2/{nm}"
    res["seat_meshes"] = seat
    json.dump(res, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    if res["solid_ok"]:
        for k, p in names.items(): shutil.copy(p, os.path.join(ROOT, "print", os.path.basename(p)))
    print(json.dumps({k: v for k, v in res.items() if k not in ("fe", "wheel_path", "latch_fit_v7_skin")}, indent=1, default=str))
    for f, v in res["fe"].items():
        for h, r in v["halves"].items(): print("FE", f, h, {kk: vv for kk, vv in r.items() if kk != "per_case"})
    print(f"[{time.time() - t0:.0f} s]")


if __name__ == "__main__":
    main()
