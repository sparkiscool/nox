"""Scout v3 underside system (24 Oct 2026; CLAUDE.md "System first, then cut", docs/generative.md The rule 0).

One model of the assembled Scout underside: carbon plate and its 3 bolts, FC, receiver v4 + XT60 lid + TPU strips, the
XT60E1-M, cartridge v4 with the pack, its XT60 plug and both sliders, the Radxa, the flow sensor, the pogo pads, the
cage's 8 catch wheels at the shared points (dimensions G, the v3 cage is item 2: wheels and rings as context), the
collar faces and the hook latch plungers (box), the belly post and the swap head (box). Every part sits in its assembled
pose from scout_bottom.py (the same model the fit checks and the motion artifact use).

The pieces (receiver, lid, strips, tray, sliders) are cut from this system at its joints: the plate bolts (heat-set
inserts), the latch (bolt on pocket floor, compression only), the XT60 (plug in the tray, XT60E1-M in the float pocket
under the lid), the TPU stop strips (cartridge top, compression only). System cases (E loads, C geometry) act where they
act on the real machine: pack and part inertia as body forces, the swap head on the cartridge's flat bottom and its
fingers on the slider channels, the pogo pins on the pad carrier, the hook latch at the 4 +-y wheels (outside the pieces:
reported as the plate-bolt reactions the cage must carry). One voxel FE per contact state (printopt, PETG, layer-aware,
build axis z_c = both big pieces' print axis), the joints closed the way that case loads them; worst index per case for
the assembled system and for each piece inside it; mass and CG of the Scout; first modes of the hanging cartridge.

python3 nox/scout/v3/scout_system.py            -> runs/scout_bottom/v1/system.json (+ the summary lines)
Heavy (~3 GB, ~10 min): run under flock /tmp/nox-heavy.lock.
"""
import json, math, os, sys, time
import numpy as np, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scout_bottom as S

OUT = S.OUT
G = 9.81
M_PACK = 0.165                      # kg (P, dimensions A)
H_FE = 1.25                         # mm voxel (walls 2.0 = 1.6 cells; the strength index is conservative at thin walls)

# ======================================================================== mass and CG of the Scout (d coords)
def mass_table(L):
    """(name, g, centroid xyz, tag). Printed: M from the meshes; bought: P / spec / E (dimensions A, C, M)."""
    D = S.T4(S.T_DC)
    def cen(m, T=None): t = S.tm(m, T); return t.center_mass if t.is_volume else t.centroid
    R = S.receiver(L); T = S.tray(); lid = S.lid()
    lp = S.layout_parts(L)
    rows = [("receiver v4", S.vol(R) * S.RHO_PETG, cen(R, D), "M"), ("XT60 lid", S.vol(lid) * S.RHO_PETG, cen(lid, D), "M"),
            ("TPU strips x2", 2 * S.vol(S.tpu_strip()) * S.RHO_TPU, np.array([-0.5, 0.0, 5.0]), "M"),
            ("cartridge tray v4", S.vol(T) * S.RHO_PETG, cen(T, D), "M"),
            ("sliders x2", 2 * S.vol(S.slider()) * S.RHO_PETG, np.array([0.0, 0.0, -37.5]), "M"),
            ("pack 4S 1500", 165.0, S.c2d([0.0, S.Y_FL1 + 16.0, 37.5])[0], "P"),
            ("XT60 pair + leads", 12.0, S.c2d([0.0, 46.0, S.Z_X])[0], "E"),
            ("FC MicoAir743 AIO", 10.0, np.array([1.9, 0.0, 13.5]), "spec"),
            ("Radxa Zero 3W", 12.0, S.tm(lp["radxa"]).centroid, "E"),
            ("MTF-02P", 1.5, S.tm(lp["flow_board"]).centroid, "spec"),
            ("plate Bee35 (cut)", 28.0, np.array([2.5, 0.0, 20.25]), "E"),
            ("motors 4 x EMAX 2004", 4 * 17.2, np.array([0.0, 0.0, 30.0]), "spec"),
            ("props 4 x Gemfan 3520", 4 * 1.8, np.array([0.0, 0.0, 41.2]), "listing"),
            ("cage v3 (item 2, C3 estimate)", 70.0, np.array([0.0, 0.0, 30.0]), "E"),
            ("camera + mount (item 3)", 20.0, np.array([62.0, 0.0, 8.0]), "E"),
            ("XR2 + BEC + wiring + hardware", 35.0, np.array([-10.0, 0.0, 12.0]), "E")]
    tot = sum(r[1] for r in rows)
    cg = sum(r[1] * np.asarray(r[2]) for r in rows) / tot
    bottom = [r for r in rows if r[0] in ("receiver v4", "XT60 lid", "TPU strips x2", "cartridge tray v4", "sliders x2", "pack 4S 1500",
                                          "XT60 pair + leads", "Radxa Zero 3W", "MTF-02P")]
    bt = sum(r[1] for r in bottom); bcg = sum(r[1] * np.asarray(r[2]) for r in bottom) / bt
    return dict(rows=[dict(part=r[0], g=round(r[1], 1), at=np.round(r[2], 1).tolist(), tag=r[3]) for r in rows],
                total_g=round(tot, 1), cg=np.round(cg, 1).tolist(), underside_g=round(bt, 1), underside_cg=np.round(bcg, 1).tolist())

# ======================================================================== system FE
def pieces_at(state):
    """c-coords manifolds of every piece in the contact state of a case: the joint that case loads is closed."""
    R, lid, T = S.receiver(LAYOUT), S.lid(), S.tray()
    s_rest = S.U(S.slider(True, 1), S.slider(True, -1)); s_in = S.U(S.slider(False, 1), S.slider(False, -1))
    strips = S.U(*[S.box(sg * S.STRIP_X0 if sg > 0 else -S.STRIP_X1, S.STRIP_X1 if sg > 0 else -S.STRIP_X0,
                         S.Y_DK0 - S.STRIP_T + S.STRIP_REC, S.Y_DK0 + S.STRIP_REC, S.STRIP_Z0, S.STRIP_Z1) for sg in (-1, 1)])
    fem = S.box(-S.XF_W / 2, S.XF_W / 2, S.Y_XF_REAR, S.Y_XF_FACE, S.Z_X - S.XF_D / 2, S.Z_X + S.XF_D / 2)
    male = S.xt60_male()
    P = dict(receiver=R, lid=lid)
    if state == "pull-out":                       # cartridge hangs on the bolts: bolt undersides on the pocket floors
        d = (0, -0.27, 0); P.update(tray=T.translate(d), sliders=s_rest.translate(d))
    elif state == "push-in":                      # cartridge top on the TPU strips (strips as PETG: stiffer, E)
        d = (0, 0.1, 0); P.update(tray=T.translate(d), sliders=s_rest.translate(d), strips=strips)
    elif state == "side":                         # rail on the +x locating face
        d = (0.35, 0, 0); P.update(tray=T.translate(d), sliders=s_rest.translate(d))
    elif state == "nose":                         # closed end on the closed-end plate
        d = (0, 0, -0.35); P.update(tray=T.translate(d), sliders=s_rest.translate(d))
    elif state == "insert":                       # head push: top on the strips, XT60 pair bonded, flange on the lid keys
        d = (0, 0.1, 0); P.update(tray=T.translate(d), sliders=s_in.translate(d), strips=strips,
                                   xt60=(fem.translate(d) + male.translate((0, S.AX_PLAY + 0.05, 0))
                                         + S.box(-6.65, 6.65, S.Y_XF_FACE - S.XM_IL, S.Y_XM + 0.6, S.Z_X - 2.95, S.Z_X + 2.95)))
    elif state == "swap-pull":                    # head pull: sliders in, flange down on the lips, plug held by the set screws
        P.update(tray=T, sliders=s_in, xt60=(fem + male.translate((0, -0.1, 0))
                                              + S.box(-6.65, 6.65, S.Y_XF_FACE - S.XM_IL, S.Y_XM + 0.6, S.Z_X - 2.95, S.Z_X + 2.95)))
    return P

def system_fe(state, cases_def, log=print):
    import printopt as PO, noxopt as NO
    P = pieces_at(state)
    allm = S.U(*P.values())
    h = H_FE; bb = np.array(allm.bounding_box()); lo = np.floor(bb[:3] - h); hi = np.ceil(bb[3:] + h)
    shape = tuple(int(np.ceil((hi[k] - lo[k]) / h)) for k in range(3))
    xs, ys, zs = [lo[k] + (np.arange(shape[k]) + 0.5) * h for k in range(3)]
    lab = np.zeros(int(np.prod(shape)), np.int8); names = list(P)
    for i, k in enumerate(names):
        v = NO._voxelise(S.tm(P[k]), xs, ys, zs).ravel().astype(bool); lab[v & (lab == 0)] = i + 1
    A = lab > 0
    I3 = np.stack(np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), np.arange(shape[2]), indexing="ij"), -1).reshape(-1, 3)
    X = I3 * h + lo + h / 2
    ndof = 3 * (shape[0] + 1) * (shape[1] + 1) * (shape[2] + 1)
    nid = lambda i, j, k: (i * (shape[1] + 1) + j) * (shape[2] + 1) + k
    def cells(b0, b1, piece=None):
        m = A & np.all((X >= np.array(b0, float)) & (X <= np.array(b1, float)), axis=1)
        return m & (lab == names.index(piece) + 1) if piece else m
    def nodes(mask):
        el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
    def load(spec):
        F = np.zeros(ndof)
        for (b0, b1, piece, vec) in spec:
            nd = nodes(cells(b0, b1, piece))
            if len(nd) == 0: raise ValueError(f"empty load region {b0} {b1} {piece}")
            for a in range(3): F[3 * nd + a] += vec[a] / len(nd)
        return F
    sup_cells = cells((-1e3, S.Y_PLATE - 1.5, -1e3), (1e3, 1e3, 1e3), "receiver")
    sups = {"plate bolts": (nodes(sup_cells)[:, None] * 3 + np.arange(3)).ravel()}
    cases = [PO.LoadCase(n, load(spec), cls=c, support="plate bolts", accel=None if acc is None else np.array(acc, float))
             for (n, spec, acc, c) in cases_def]
    pb = PO.Problem(shape=shape, h=h, origin=tuple(lo), solid=A.copy(), void=~A, supports=sups, cases=cases,
                    material=PO.MATERIALS["PETG"], build="+z", overhang=False, R=0.5, comp_factor=None)
    po = PO.PrintOpt(pb); rp = po.physical(A.astype(float), 64.0)
    Eel = po.EMIN + rp ** po.PEN * (1 - po.EMIN); K = po.assemble(Eel)
    log(f"system FE [{state}]: grid {shape} h {h}, {int(A.sum()):,} cells, pieces {names}")
    Fs = [po.force(i, rp) for i in range(len(cases))]
    sol = po.solve(Eel, {"plate bolts": np.stack(Fs, 1)}, K=K)["plate bolts"]; Uu = sol[0]
    rows = []
    elab = lab[po.smask] if po.smask.shape == lab.shape else None
    for j, lc in enumerate(cases):
        ue = Uu[:, j][po.edof]; st = po.stresses(ue, rp); Al = PO.allowables(pb.material, lc.cls)
        f1, _, f2, _ = po.failure(st, Al); sm = po.smask
        fm = np.maximum(f1, f2)
        per = {}
        for i, k in enumerate(names):
            m = sm & (lab == i + 1)
            if m.any():
                w = np.argmax(np.where(m, fm, -1)); per[k] = dict(index=round(float(fm[w]), 2), at=np.round(po.ecent[w], 0).tolist())
        w = int(np.argmax(np.where(sm, fm, -1)))
        r = dict(case=lc.name, state=state, cls=lc.cls, worst=round(float(fm[w]), 2), at=np.round(po.ecent[w], 0).tolist(),
                 piece=names[lab[w] - 1] if lab[w] > 0 else "-", inlayer=round(float(f1[sm].max()), 2), across=round(float(f2[sm].max()), 2),
                 max_disp_mm=round(float(np.abs(Uu[:, j]).max()), 4), pieces=per)
        rows.append(r)
        log(f"  {r['case']:52s} [{r['cls']:9s}] worst {r['worst']:.2f} in {r['piece']} at {r['at']} (in-layer {r['inlayer']:.2f}, "
            f"across {r['across']:.2f}), u {r['max_disp_mm']} | " + ", ".join(f"{k} {v['index']:.2f}" for k, v in per.items()))
    try: sol[1].free_memory(everything=True)
    except Exception: pass
    return rows

def cases_by_state():
    L = LAYOUT; big = 1e3
    xp, yp, zp = L["pads"]; x0, zt, s = L["radxa"]; xl, yl, zl = L["flow"]
    car = S.d2c([[xp - S.CAR_HALF, S.Y_IN, zp], [xp + S.CAR_HALF, yp + 7.5, zp + 1.3]]); c0, c1 = car.min(0), car.max(0)
    rb = S.d2c([[x0 - S.RX_L, -(S.SP1 + s) - 0.1, zt - S.RX_W], [x0, -(S.SP1 + s) + 1.3, zt]]); r0, r1 = rb.min(0), rb.max(0)
    fb = S.d2c([[xl - 18.3, S.Y_IN, zl + S.FL_T + 0.5], [xl + 8.3, yl + 12.3, zl + S.FL_T + 2.0]]); f0, f1 = fb.min(0), fb.max(0)
    floor = ((-S.X_IN, S.Y_FL1 - 1.4, S.Z_P0), (S.X_IN, S.Y_FL1 + 0.1, S.Z_P1))
    lips = ((-S.X_IN - 0.1, S.Y_LIP - 1.5, 6.0), (S.X_IN + 0.1, S.Y_TOP + 0.2, 69.0))
    pk = lambda a: M_PACK * G * a
    rim = ((-big, -0.5, -big), (big, 1.3, big))
    chan = ((-S.X_OUT - 1, S.CH_DROP_Y - 1.5, S.Z_TAB - S.CH_HW), (S.X_OUT + 1, S.CH_Y0 + 0.05, S.Z_TAB + S.CH_HW))
    return {
        "pull-out": [
            ("hover + flight 4 g (pack on the floor, all parts)", [(*floor, "tray", (0, -pk(4), 0))], (0, -4, 0), "flight"),
            ("floor landing 6 g", [(*floor, "tray", (0, -pk(6), 0))], (0, -6, 0), "flight"),
            ("crash drop upright 35 g", [(*floor, "tray", (0, -pk(35), 0))], (0, -35, 0), "crash"),
            ("pogo 4 pins + KW11 12 N (carrier, up)", [(tuple(c0), tuple(c1), "receiver", (0, 12.0, 0))], None, "sustained"),
            ("Radxa 15 g at 35 g + 5 N off the plate", [(tuple(r0), tuple(r1), "receiver", (10.0, -5.0, 0))], None, "crash"),
            ("flow seat 10 N down + 10 N out", [(tuple(f0), tuple(f1), "receiver", (-10.0, -10.0, 0))], None, "flight"),
        ],
        "push-in": [
            ("crash inverted 35 g (pack on the lips)", [(*lips, "tray", (0, pk(35), 0))], (0, 35, 0), "crash"),
            ("head stall 75 N at the seat (flat bottom up)", [(*rim, "tray", (0, 75.0, 0))], None, "misuse"),
        ],
        "side": [("crash side 35 g (pack on the +x wall)", [((S.X_IN - 1.4, S.Y_FL1, S.Z_P0), (S.X_IN + 0.1, 46.0, S.Z_P1), "tray", (pk(35), 0, 0))], (35, 0, 0), "crash")],
        "nose": [("crash nose-on 35 g (pack on the closed end)", [((-S.X_IN, S.Y_FL1, S.Z_P0 - 1.4), (S.X_IN, 46.0, S.Z_P0 + 0.1), "tray", (0, 0, -pk(35)))], (0, 0, -35), "crash")],
        "insert": [("swap insert 60 N (head on the flat bottom)", [(*rim, "tray", (0, 60.0, 0))], None, "flight")],
        "swap-pull": [("swap pull-out 50 N (fingers on the channels)", [(*chan, "tray", (0, -50.0, 0))], None, "flight")],
    }

def bolt_reactions():
    """statics (E): plate-bolt reactions for the swap insert, the push the hook latch then holds at the 4 +-y wheels."""
    # 60 N up: XT60 share up to 44 N at the plug (x -48.3), the rest on the strips (centroid x 1.0); bolts sides x 3.9, nose 45
    xs = dict(xt60=37.5 - S.Z_X, strips=37.5 - (S.STRIP_Z0 + S.STRIP_Z1) / 2)
    Fx, Fs = 44.0, 16.0
    M = Fx * xs["xt60"] + Fs * xs["strips"]
    Rn = (M - 60.0 * 3.9) / (45.0 - 3.9); Rs = 60.0 - Rn
    return dict(insert_60N=dict(side_bolts_pair_N=round(Rs, 1), nose_bolt_N=round(Rn, 1), note="negative = pulls on the bolt (E)"),
                hook_latch="the plate passes the 60 N push into the cage; the plungers hold it at the 4 +-y wheels, 30 N along the slope each "
                           "functional / 100 N misuse (dimensions G (7)); the cage (item 2) carries it, not these pieces")

def modes(rows):
    """first modes of the hanging cartridge on the receiver, from the static deflection at 4 g (vertical) and 35 g side (E)."""
    out = {}
    for r in rows:
        if r["case"].startswith("hover"): out["vertical_Hz"] = round(1 / (2 * math.pi) * math.sqrt(4 * 9810 / max(r["max_disp_mm"], 1e-6)), 0)
        if r["case"].startswith("crash side"): out["lateral_Hz"] = round(1 / (2 * math.pi) * math.sqrt(35 * 9810 / max(r["max_disp_mm"], 1e-6)), 0)
    return out

LAYOUT = None
def main():
    global LAYOUT
    t0 = time.time(); log = lambda s: print(s, flush=True)
    res = json.load(open(os.path.join(OUT, "result.json")))
    LAYOUT = {k: tuple(v) for k, v in res["layout"].items()}
    out = dict(layout=LAYOUT, mass=mass_table(LAYOUT))
    log(f"mass {out['mass']['total_g']} g, CG {out['mass']['cg']}; underside {out['mass']['underside_g']} g at {out['mass']['underside_cg']}")
    out["fit"] = dict(insertion_sweep_worst_mm3=res["checks"]["insertion_sweep_worst_mm3"],
                      static=dict(res["checks"]["receiver_vs"], **{"lid " + k: v for k, v in res["checks"]["lid_vs"].items()}),
                      float_extremes=res["checks"]["float_extremes_mm3"], funnel_plane_margin=res["checks"]["funnel_plane_margin"],
                      landing_sweep=res["checks"]["landing_sweep"])
    rows = []
    for state, cases in cases_by_state().items():
        rows += system_fe(state, cases, log)
    out["cases"] = rows
    w = max(rows, key=lambda r: r["worst"])
    nm = [r for r in rows if r["cls"] != "misuse"]; wn = max(nm, key=lambda r: r["worst"])
    out["worst"] = dict(index=w["worst"], case=w["case"], piece=w["piece"], at=w["at"])
    out["worst_non_misuse"] = dict(index=wn["worst"], case=wn["case"], piece=wn["piece"], at=wn["at"])
    per = {}
    for r in rows:
        for k, v in r["pieces"].items():
            if k not in per or v["index"] > per[k]["index"]: per[k] = dict(index=v["index"], case=r["case"], at=v["at"])
    out["worst_per_piece"] = per
    out["modes"] = modes(rows)
    out["bolt_reactions"] = bolt_reactions()
    out["wall_s"] = round(time.time() - t0, 1)
    json.dump(out, open(os.path.join(OUT, "system.json"), "w"), indent=1, default=float)
    fit = sum(v for v in out["fit"]["static"].values()) + out["fit"]["insertion_sweep_worst_mm3"]
    log(f"System: Scout underside (scout_system.py), assembled worst index {w['worst']} in {w['case']} ({w['piece']}), "
        f"fit {fit:.1f} mm3 (M), mass {out['mass']['total_g']} g (M / E), CG {out['mass']['cg']}")
    log(f"per piece {per}")
    log(f"modes {out['modes']}  done {out['wall_s']} s")

if __name__ == "__main__":
    main()
