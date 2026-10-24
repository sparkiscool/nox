"""Box structure v2, topology round 2: sheets are cladding (not in the model), the only support is the floor plane Z -236.

Differences from round 1 (box_system_topo_v2.py, whose helpers are reused):
 - no sheets drawn or held; supports: floor layer (all cases), +X face layer (knocked over: the box lies on that face)
 - final layout (front Y 335, funnel v8 plain in catch_W.stl as it is, no +-11.5 copies), arm envelopes and both drone
   insert lanes from the system file's meshes, electronics boards + PSU + charger + fixed items as keep-outs on the rear
 - receiver loads at the M3 mount bolts (drawn bosses Ø12, wall >= 3), 74 N head stalls, centring 7.6 N / 1.05 N.m
 - optional datum ring (drawn passive solid, --ring); 8 electronics mounting bosses outside the PSU / charger footprints
Output: runs/box_system/v2/topo_r2/<tag>/.   cd nox && python3 box/box_system_topo_v2r2.py --gen 3000 [--ring] --out ...
Tags: C geometry, M mesh, E estimate, F this model."""
import os, sys, json, math, time, argparse
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path: sys.path.insert(0, HERE)
import box_system_topo_v2 as T
from box_system_topo_v2 import (L2, B, J, META, EN, R, Y_C, Y_FUN, G, MASS, ARM_G, FUN_G, ELEC_G, DRONES, GROW, LAY, ROOT,
                                cylY, box, union, bay_pose, disc, seat_posts, service_sweeps, _drop_islands, add_case, peak_gb)
from topo import Settings
from topo.part import Part, tm, man
from topo.layout import Layout

OPT = dict(ring=False)
ELEC = json.load(open(os.path.join(ROOT, "runs", "box_electronics", "v1", "result.json")))
import box_electronics as BE
BOLTS = {"scout": [(sx * 25.6, z) for sx in (-1, 1) for z in (0.85, 85.8)],
         "overseer": [(sx * 26.8, z) for sx in (-1, 1) for z in (0.85, 85.5, 163.8)]}


def load_grown_path(path):
    f = os.path.join(T.CACHE, f"grown_{os.path.basename(path)}_{GROW}_{os.path.getsize(path)}.stl")
    if os.path.exists(f): return man(trimesh.load(f, force="mesh"))
    G_ = man(trimesh.load(path, force="mesh")).minkowski_sum(mf.Manifold.sphere(GROW, 4))
    tm(G_).export(f); return G_


def bosses(h):
    """per bay: Ø12 insert bosses at the M3 mount bolts, from the mount plane Y_M up through the 8 pad and 6 beyond."""
    out = {}
    for ang, pk in L2.BAYS.items():
        g = META[pk]
        out[ang] = L2.place(union([cylY(6.0, g["Y_M"], g["Y_M"] + max(3.0, h) + 6.0).translate([x, 0, zc + g["Z_T"]]) for x, zc in BOLTS[pk]]),
                            g["Z_T"], ang, R)
    return out


def elec_keepouts(Y0):
    """boards: Y -66+10 standoff+depth+6; fixed items: rect, depth from the sheet face (result.json)."""
    out = []
    for n, d in ELEC["placement"].items():
        x0, x1, z0, z1 = d["rect"]; dep = BE.BOARDS[n][2]
        out.append(box(x0 - GROW, x1 + GROW, Y0, Y0 + BE.STANDOFF + dep + 6.0 + GROW, z0 - GROW, z1 + GROW))
    for n, d in ELEC["fixed"].items():
        x0, x1, z0, z1 = d["rect"]
        out.append(box(x0 - GROW, x1 + GROW, Y0, Y0 + d["depth"] + GROW, z0 - GROW, z1 + GROW))
    return out


def elec_bosses(Y0, T_):
    """8 Ø14 mounting bosses on the rear face, just outside the PSU (X 42..157, Z -170..45) and charger (X -162..-44, Z -200..-29)."""
    pts = [(33, -179), (166, -179), (33, 54), (166, 54), (-171, -209), (-35, -209), (-171, -20), (-35, -20)]
    pts = [(x, max(z, EN["FLOOR"] + 9)) for x, z in pts]
    return [cylY(7.0, Y0, Y0 + T_).translate([x, 0, z]) for x, z in pts]


def datum_ring():
    r0, r1, y0, y1 = 115.0, 200.0, 74.0, 98.0
    a0, a1 = -210.0, 30.0
    pts = [(0.0, 0.0)] + [(1.3 * r1 * math.cos(math.radians(a)), 1.3 * r1 * math.sin(math.radians(a))) for a in np.linspace(a0, a1, 49)]
    w = mf.Manifold.extrude(mf.CrossSection([pts]), y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])
    return (cylY(r1, y0, y1) - cylY(r0, y0 - 1, y1 + 1)) ^ w


def setup_factory(log, info):
    geom = {}

    def setup(h):
        t0 = time.time()
        Tn = max(3.0, h)
        X_IN, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
        Y1 = Y1 - Tn                                                    # front sheet = service door, not drawn
        env = box(-X_IN, X_IN, Y0, Y1, Z0, Z1)
        pd = T.pads(Tn)
        bs = bosses(h)
        stub = cylY(45.0, -Tn, 0.0)
        eb = elec_bosses(Y0, Tn)
        cuts = [T.load_grown(n) for n in ("receivers", "cartridge_paths", "drone", "catch")]    # catch: as it is, no copies
        gy = getattr(L2, "FUNNEL_V8_GROW_Y", 0.0)
        hubm, motm, parkm, lane, grab = [load_grown_path(os.path.join(LAY, "parts", f)) for f in
                                         ("hub_27.stl", "motors_28.stl", "parked_29.stl", "drone_30.stl", "drone_31.stl")]
        zt = Z1 + 5; a0 = 84.0; b0 = 84.0 + gy; dz = zt - T.Z_EDGE
        sq = mf.CrossSection([[(-a0, -b0), (a0, -b0), (a0, b0), (-a0, b0)]])
        fun = mf.Manifold.extrude(sq, dz, 0, 0.0, ((a0 + dz) / a0, (b0 + dz) / b0)).translate([0, Y_FUN, T.Z_EDGE])
        rail = cylY(L2.RAIL_DISC["r"] + GROW, L2.RAIL_DISC["y"][0] - GROW, L2.RAIL_DISC["y"][1] + GROW)
        cuts += [hubm, motm, parkm, lane, grab, fun, rail] + elec_keepouts(Y0)
        cuts += service_sweeps(Y1)
        posts, side = seat_posts(6.0 if h <= 6 else h, cuts[3] + fun)
        extra = []
        if OPT["ring"]:
            ring = datum_ring() - union(cuts)
            extra.append(ring); info["ring_cm3"] = round(ring.volume() / 1e3, 1)
            log(f"datum ring (drawn, minus keep-outs) {info['ring_cm3']} cm3")
        drawn_all = union(list(pd.values()) + list(bs.values()) + posts + [stub] + eb + extra)
        cutm = union(cuts) - drawn_all
        info.update(posts_side=side, T=Tn); log(f"seat posts side/overlap mm3 {side}")
        geom.update(pads=pd, bosses=bs, posts=posts, stub=stub, drawn=drawn_all, cuts=cutm, env=env, fun=fun, eboss=eb, ring=extra)
        p = Part("box_system_topo_v2r2", h, drawn_all, cutm, env, build="+z", log=log)
        _cases(p, h, Tn, pd, bs, posts, stub, eb, log)
        _drop_islands(p, log)
        log(f"setup h {h}: {time.time() - t0:.0f} s, {len(p.cases)} cases")
        return p
    return setup, geom


def _cases(p, h, Tn, pd, bs, posts, stub, eb, log):
    X_IN, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
    Y1 = Y1 - Tn
    p.support("default", box(-X_IN - 1, X_IN + 1, Y0 - 1, Y1 + 1, Z0 - 1, Z0 + h))          # the floor plane, anywhere
    p.support("side", box(X_IN - h, X_IN + 1, Y0 - 1, Y1 + 1, Z0 - 1, Z1 + 1))              # lying on its +X face
    posts_m = union(posts); eb_m = union(eb)
    pad_cg = {ang: (bay_pose(ang)(META[pk]["SWAP_CG"]), MASS[pk]) for ang, pk in L2.BAYS.items()}
    cgs = {"arm": (0.0, 60.0, 0.0), "fun": (0.0, Y_FUN, 180.0), "dr": (0.0, Y_FUN, 184.5), "el": (100.0, -50.0, -60.0)}

    def lumps(acc, drone):
        a = np.asarray(acc, float)
        out = [(bs[ang], m * G * a, None, cg) for ang, (cg, m) in pad_cg.items()]
        out.append((stub, ARM_G * G * a, None, cgs["arm"]))
        out.append((posts_m, FUN_G * G * a, None, cgs["fun"]))
        if drone: out.append((posts_m, DRONES[drone] * G * a, None, cgs["dr"]))
        out.append((eb_m, ELEC_G * G * a, None, cgs["el"]))
        return out

    def Ecatch(m_g, v=0.15, drop=0.030):
        m = m_g * 1e-3; return (0.5 * m * v * v + m * 9.81 * drop) * 1000 / 3.0

    down = (0, 0, -1); up = np.array([0, 0, 1.0])
    PL = (60.0, Y_FUN + 30.0, 175.0); PLm = (-60.0, Y_FUN - 30.0, 175.0)
    Fc = Ecatch(DRONES["Overseer"]); Fm = Ecatch(DRONES["Overseer"], 0.15, 0.040)
    add_case(p, "standing, Overseer seated", "sustained", "default", down, lumps(down, "Overseer"))
    add_case(p, "Overseer catch +X+Y diag", "flight", "default", down,
             lumps(down, "Overseer") + [(posts_m, (-0.25 * Fc, -0.25 * Fc, -Fc), None, PL)])
    add_case(p, "Overseer catch -X-Y diag", "flight", "default", down,
             lumps(down, "Overseer") + [(posts_m, (0.25 * Fc, 0.25 * Fc, -Fc), None, PLm)])
    add_case(p, "misuse: Overseer motors cut", "misuse", "default", down,
             lumps(down, "Overseer") + [(posts_m, (-0.25 * Fm, -0.25 * Fm, -Fm), None, PL)])
    tip = lambda ang, r=140.0: np.array([r * math.cos(math.radians(ang)), Y_C, r * math.sin(math.radians(ang))])
    P0 = np.array([0.0, Y_C, 0.0])
    add_case(p, "XT60 insert at the drone 60 N", "flight", "default", down,
             lumps(down, "Scout") + [(posts_m, 60 * up, None, tip(90, 144.5)), (stub, -60 * up, None, tip(90))])
    add_case(p, "misuse: head stall at the drone 74 N", "misuse", "default", down,
             lumps(down, "Scout") + [(posts_m, 74 * up, None, tip(90, 144.5)), (stub, -74 * up, None, tip(90))])
    for ang in (-90, -120):
        d = tip(ang, 1.0) - np.array([0, Y_C, 0])
        xp = trimesh.load(os.path.join(L2.CTX, f"{L2.BAYS[ang]}_xt60e1m_c.stl"), force="mesh").centroid
        P_ = bay_pose(ang)(xp)
        add_case(p, f"XT60 insert at the {ang} bay 60 N", "flight", "default", down,
                 lumps(down, "Scout") + [(bs[ang], 60 * d, None, P_), (stub, -60 * d, None, tip(ang))])
        if ang == -90:
            add_case(p, "XT60 pull at the -90 bay 50 N", "flight", "default", down,
                     lumps(down, "Scout") + [(bs[ang], -50 * d, None, P_), (stub, 50 * d, None, tip(ang))])
            add_case(p, "misuse: head stall at the -90 bay 74 N", "misuse", "default", down,
                     lumps(down, "Scout") + [(bs[ang], 74 * d, None, P_), (stub, -74 * d, None, tip(ang))])
        # centring 7.6 N across + 1.05 N.m about the axle at this bay's receiver, reacted at the stub
        tdir = np.array([-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))])
        F_ = 7.6 * tdir; M_ = np.array([0, 1050.0, 0]); Mr = -(M_ + np.cross(P_ - P0, F_))
        add_case(p, f"centring at the {ang} bay 7.6 N + 1.05 N.m", "flight", "default", down,
                 lumps(down, "Scout") + [(bs[ang], F_, M_, P_), (stub, -F_, Mr, P0)])
    add_case(p, "stub tilt: XT60 at the drone 57 N + 5.9 N.m", "flight", "default", down,
             lumps(down, "Scout") + [(stub, (0, 0, -57.0), (5900.0, 0, 0), (0, Y_C, 0)),
                                     (posts_m, (0, 0, 57.0), (-5900.0, 0, 0), (0, Y_C, 144.5))])
    for nm, mode in (("all bays: 60 N insert", "rad"), ("all bays: 43.5 N across", "tan")):
        loads = []; Ft = np.zeros(3); Mt = np.zeros(3)
        for ang in L2.BAYS:
            cg = pad_cg[ang][0]; r_ = np.array([math.cos(math.radians(ang)), 0, math.sin(math.radians(ang))])
            tdir = np.array([-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))])
            F_ = (60.0 * r_) if mode == "rad" else (43.5 * tdir)
            Pp = np.array([cg[0], Y_C, cg[2]])
            loads.append((bs[ang], F_, None, Pp)); Ft += F_; Mt += np.cross(Pp - P0, F_)
        loads.append((stub, -Ft, -Mt, P0))
        add_case(p, nm, "flight", "default", down, lumps(down, "Scout") + loads)
    a2 = (0, 0, -2); a5 = (0, 0, -5); al = (5, 0, 0)
    add_case(p, "lifted from below 2 g (sheets no longer carry)", "flight", "default", a2, lumps(a2, None))
    add_case(p, "misuse: set down hard 5 g", "misuse", "default", a5, lumps(a5, None))
    add_case(p, "misuse: knocked over onto +X 5 g", "misuse", "side", al, lumps(al, None))


def rigid_fit(part, U, q, region, what):
    el = part.elems(region, part.solid | part.design, what=what)
    nd = part.nodes_of(el); X = np.asarray(part.origin) + np.stack(np.unravel_index(nd, part.nshape), 1) * part.h
    c0 = X.mean(0); r = X - c0
    A = np.zeros((3 * len(nd), 6)); A[:, :3] = np.tile(np.eye(3), (len(nd), 1))
    for i in range(3):
        e = np.zeros(3); e[i] = 1.0; A[:, 3 + i] = np.cross(e[None], r).ravel()
    u = np.stack([U[3 * nd + k, q] for k in range(3)], 1)
    x = np.linalg.lstsq(A, u.ravel(), rcond=None)[0]
    return x[:3], x[3:] * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--h", type=float, default=8.0); ap.add_argument("--gen", type=float, default=3000.0)
    ap.add_argument("--min-member", type=float, default=18.0); ap.add_argument("--ring", action="store_true")
    ap.add_argument("--out", required=True); ap.add_argument("--git", action="store_true")
    ap.add_argument("--time-limit", type=float, default=5.5 * 3600); ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    OPT["ring"] = a.ring
    from dataclasses import replace
    from topo.run import Tee, git_checkpoint
    os.makedirs(a.out, exist_ok=True)
    log = Tee(os.path.join(a.out, "log.txt")); t0 = time.time(); info = {}
    setup, geom = setup_factory(log, info)
    s = Settings(min_member_mm=a.min_member, gate_member=False, finish="drawn", reach_check="warn",
                 tie_min_cells=10 ** 9, time_limit_s=a.time_limit)
    log(f"=== box_system_topo_v2r2: h {a.h}, gen {a.gen} cm3, ring {a.ring}, min member {a.min_member}; sheets = cladding, floor support only")
    hook = git_checkpoint(a.out, log) if a.git else None
    ck = os.path.join(a.out, "ck.npz"); ckc = os.path.join(a.out, "coarse_ck.npz")
    xh_c = part_c = None
    if not os.path.exists(ck):
        part_c = setup(2 * a.h)
        if a.dry: return
        Lc = Layout(part_c, gen_cm3=a.gen, settings=replace(s, reach_check="off", max_stage=25), log=log)
        Lc.run(maxit=400, checkpoint=ckc, resume=ckc, beta_stop=2, on_checkpoint=hook)
        xh_c = Lc.x_hat(); del Lc
        log(f"coarse stage done: {time.time() - t0:.0f} s, peak {peak_gb():.2f} GB")
    part = setup(a.h)
    if a.dry: return
    Ly = Layout(part, gen_cm3=a.gen, settings=s, log=log)
    if xh_c is not None: Ly.init_from(xh_c, part_c)
    Ly.run(maxit=400, checkpoint=ck, resume=ck, beta_start=2, on_checkpoint=hook)
    Ly.save(os.path.join(a.out, "density.npz"))
    comp, _ = Ly._compliance(Ly.result["rho_n"], need_grad=False)
    stub_rot = {}; bay_rot = {}
    try:
        _, (U, E_) = Ly._compliance(Ly.result["rho_n"], need_grad=True)
        for q, c in enumerate(Ly.cases):
            tr, rot = rigid_fit(part, U, q, geom["stub"], "stub")
            stub_rot[c.name] = dict(trans_mm=tr.round(4).tolist(), rot_mrad=rot.round(4).tolist())
            if c.name.startswith("all bays") or c.name.startswith("standing"):
                bay_rot[c.name] = {}
                for ang in L2.BAYS:
                    t2, r2 = rigid_fit(part, U, q, geom["bosses"][ang], f"bosses {ang}")
                    bay_rot[c.name][str(ang)] = dict(rot_mrad=r2.round(4).tolist(), rel_to_stub_Y_mrad=round(float(r2[1] - rot[1]), 4))
    except Exception as ex:
        import traceback; log(f"rotation fit failed: {ex}\n{traceback.format_exc()}")
    res = dict(stub_rotation=stub_rot, bay_rotation=bay_rot, ring=a.ring, h=a.h, grid=list(part.shape), cells=part.nel,
               design_cells=int(part.design.sum()), min_member_mm=a.min_member, gen_cm3=a.gen,
               generated_cm3=float(Ly.result["rho_n"][Ly.design].sum() * a.h ** 3 / 1000.0),
               drawn_cm3=float(part.solid.sum() * a.h ** 3 / 1000.0),
               compliance_nominal={c.name: float(v) for c, v in zip(Ly.cases, comp)},
               compliance_ref_solid={c.name: float(v) for c, v in zip(Ly.cases, Ly.c_ref)},
               ratio_to_full_solid={c.name: float(v / r) for c, v, r in zip(Ly.cases, comp, Ly.c_ref)},
               peak_gb=peak_gb(), wall_s=round(time.time() - t0), stopped=Ly.stopped, iterations=len(Ly.hist), setup=info)
    res["mass_g_PETG"] = (res["generated_cm3"] + res["drawn_cm3"]) * 1.27
    json.dump(res, open(os.path.join(a.out, "result.json"), "w"), indent=1, default=str)
    log(json.dumps(res["ratio_to_full_solid"], indent=1))
    if hook: hook("final")


if __name__ == "__main__":
    main()
