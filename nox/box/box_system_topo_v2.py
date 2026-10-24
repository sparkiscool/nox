"""Box structure v2: coarse SYSTEM topology run (docs/generative.md The rule 0). A load-path study, not a part.

Joined envelope = the enclosure interior (layout.json), keep-outs = receivers, cartridge paths, drone receivers, catch,
the arm's turning envelopes, the funnel inside, the electronics zone; drawn = bay mount pads, 8 funnel seat posts,
stub plate, 4 sheets; supports = floor layer (main), hands, side; loads = the v1 cases translated to v2.
Output: runs/box_system/v2/topo/ (density.npz, log.txt, result.json, views). Nothing goes to print/.

cd nox && python3 box/box_system_topo_v2.py [--h 6] [--gen 1500] [--min-member 18] [--out ../runs/box_system/v2/topo]
Everything is in W (layout.json frame). Tags: C geometry, M mesh, E estimate, F this model."""
import os, sys, json, math, time, argparse
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
NOX = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(NOX, ".."))
for p in (NOX, HERE):
    if p not in sys.path: sys.path.insert(0, p)
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import box_layout_v2 as L2
import box_system as B
from topo import Settings
from topo.part import Part, LoadCase, tm, man
from topo.layout import Layout

LAY = os.path.join(ROOT, "runs", "box_system", "v2", "layout")
J = json.load(open(os.path.join(LAY, "layout.json")))
META = json.load(open(os.path.join(ROOT, "runs", "box_system", "v2", "latch_ctx", "meta.json")))
EN = J["enclosure"]
R = J["R"]; Y_C = L2.Y_C; Y_FUN = L2.Y_FUN
G = 9.81e-3                                     # N per g (mass in g)
MASS = {"scout": META["scout"]["SWAP_M"], "overseer": META["overseer"]["SWAP_M"]}
ARM_G, FUN_G, ELEC_G = 2900.0, 640.0, 2000.0
DRONES = {"Scout": 520.0, "Overseer": 2330.0}
Z_EDGE = 143.6 + 13.0
LANE_Y = getattr(L2, "LANE_Y", (74.0, 260.0))
GROW = 1.0                                      # keep-outs grown by 1 mm
CACHE = os.environ.get("TOPO_CACHE", "/tmp/claude-0/-home-user-nox/48926873-2ab8-5adf-8e03-a2e97c4fb8f1/scratchpad")


def cylY(r, y0, y1, n=96):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([0, y0, 0])


def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def load_grown(name):
    src = os.path.join(LAY, name + "_W.stl")
    f = os.path.join(CACHE, f"grown_{name}_{GROW}_{os.path.getsize(src)}.stl")
    if os.path.exists(f):
        return man(trimesh.load(f, force="mesh"))
    G_ = man(trimesh.load(src, force="mesh")).minkowski_sum(mf.Manifold.sphere(GROW, 4))
    tm(G_).export(f)
    return G_


def union(ms):
    out = None
    for m in ms: out = m if out is None else out + m
    return out


def bay_pose(ang):
    """cartridge coords -> W (box_layout_v2.place) on a point."""
    zT = META[L2.BAYS[ang]]["Z_T"]
    th = math.radians(90 - ang); c, s = math.cos(th), math.sin(th)

    def f(p):
        x, y, z = p
        X, Y, Z = x, Y_C - (z - zT), R + y
        return np.array([X * c + Z * s, Y, -X * s + Z * c])
    return f


def pads(T):
    out = {}
    for ang, pk in L2.BAYS.items():
        g = META[pk]; bx = g["RF"] + 5.3 + 5.0
        pad = mf.Manifold.cube([2 * bx, T, g["L_C"] + 8.6]).translate([-bx, g["Y_M"], -4.3])
        out[ang] = L2.place(pad, g["Z_T"], ang, R)
    return out


def disc(p0, n, r, th):
    """cylinder r x th from p0 along unit n."""
    n = np.asarray(n, float); n /= np.linalg.norm(n)
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, a); u /= np.linalg.norm(u); v = np.cross(n, u)
    M = np.stack([u, v, n, np.asarray(p0, float)], 1)                   # 3 x 4
    return mf.Manifold.cylinder(th, r, r, 48).transform(M.tolist())


def seat_posts(th, forbid):
    """8 Ø16 x 6 discs at the landings: p..p+th n (outward, behind the landing) or p-th n..p, whichever overlaps the
    funnel inside / panels less."""
    out = []; side = []
    for ld in J["landings"]:
        p = np.array(ld["p"]); n = np.array(ld["n"])
        c = [(+1, disc(p, n, 8.0, th)), (-1, disc(p - th * n, n, 8.0, th))]
        c = [(sg, d, (d ^ forbid).volume()) for sg, d in c]
        sg, d, ov = min(c, key=lambda t: t[2])
        out.append(d); side.append((sg, round(ov, 1)))
    return out, side


def setup_factory(log, info):
    geom = {}

    def setup(h):
        t0 = time.time()
        T = max(3.0, h)                                              # drawn thickness: at least one cell layer
        X_IN, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
        env = box(-X_IN, X_IN, Y0, Y1, Z0, Z1)
        sheets = {"+X": box(X_IN - T, X_IN, Y0, Y1, Z0, Z1), "-X": box(-X_IN, -X_IN + T, Y0, Y1, Z0, Z1),
                  "rear": box(-X_IN, X_IN, Y0, Y0 + T, Z0, Z1), "front": box(-X_IN, X_IN, Y1 - T, Y1, Z0, Z1)}
        pd = pads(max(3.0, h))
        stub = cylY(45.0, -T, 0.0)
        cuts = [load_grown(n) for n in ("receivers", "cartridge_paths", "drone", "catch")]
        hub = cylY(88.0 + GROW, 7.0 - GROW, 74.0 + GROW)
        motor = cylY(106.5 + GROW, 2.0 - GROW, 50.0 + GROW)
        park = cylY(R - L2.ARM_PARK_R, 74.0, 300.0)
        pts = [(0.0, 0.0)] + [(1.2 * 159 * math.cos(math.radians(90 + a)), 1.2 * 159 * math.sin(math.radians(90 + a)))
                              for a in np.linspace(-25, 25, 21)]
        lane = mf.Manifold.extrude(mf.CrossSection([pts]), LANE_Y[1] - LANE_Y[0]).rotate([90, 0, 0]).translate([0, LANE_Y[1], 0]) \
            ^ cylY(159.0, LANE_Y[0], LANE_Y[1])
        zt = Z1 + 5
        a0 = 84.0; a1 = a0 + (zt - Z_EDGE)
        sq = mf.CrossSection([[(-a0, -a0), (a0, -a0), (a0, a0), (-a0, a0)]])
        fun = mf.Manifold.extrude(sq, zt - Z_EDGE, 0, 0.0, (a1 / a0, a1 / a0)).translate([0, Y_FUN, Z_EDGE])
        elec = box(42 - GROW, 157 + GROW, Y0 + T, -36 + GROW, -170 - GROW, 45 + GROW)
        cuts += [hub, motor, park, lane, fun, elec]
        posts, side = seat_posts(6.0 if h <= 6 else h, cuts[3] + fun)
        drawn_all = union(list(sheets.values()) + list(pd.values()) + posts + [stub])
        cutm = union(cuts) - drawn_all                                  # drawn wins where it touches a cut
        info.update(posts_side=side, T=T)
        geom.update(pads=pd, posts=posts, stub=stub, sheets=sheets, drawn=drawn_all, cuts=cutm, env=env, fun=fun)
        p = Part("box_system_topo_v2", h, drawn_all, cutm, env, build="+z", log=log)
        _cases(p, h, T, pd, posts, stub, sheets, log)
        log(f"setup h {h}: {time.time() - t0:.0f} s, {len(p.cases)} cases")
        return p
    return setup, geom


# ---------------------------------------------------------------------------------------------------------- loads
def add_case(p, name, cls, support, acc, loads):
    """loads: [(region manifold, F (3), M (3) or None, P (3))] -> nodal forces by rigid distribution (wrench)."""
    F = np.zeros(p.ndof); allnodes = []
    for region, f, m, P in loads:
        el = p.elems(region, p.solid | p.design, what=f"load {name}")
        if len(el) == 0: raise ValueError(f"load {name}: empty region")
        nd = p.nodes_of(el)
        X = np.asarray(p.origin) + np.stack(np.unravel_index(nd, p.nshape), 1) * p.h
        w = B.wrench(X, np.asarray(f, float), np.zeros(3) if m is None else m, P)
        for a in range(3): np.add.at(F, 3 * nd + a, w[:, a])
        allnodes.append(nd)
    p.cases.append(LoadCase(name, F, cls=cls, support=support, accel=np.asarray(acc, float),
                            nodes=np.unique(np.concatenate(allnodes))))


def _cases(p, h, T, pd, posts, stub, sheets, log):
    X_IN, Y0, Y1, Z0, Z1 = EN["X_IN"], EN["Y_BACK"], EN["Y_FRONT"], EN["FLOOR"], EN["Z_TOP"]
    p.support("default", box(-X_IN - 1, X_IN + 1, Y0 - 1, Y1 + 1, Z0 - 1, Z0 + h))
    p.support("hands", union([box(X_IN - T - .1, X_IN + .1, 100, 250, 0, 150), box(-X_IN - .1, -X_IN + T + .1, 100, 250, 0, 150)]))
    p.support("side", sheets["+X"])
    posts_m = union(posts)
    pad_cg = {ang: (bay_pose(ang)(META[pk]["SWAP_CG"]), MASS[pk]) for ang, pk in L2.BAYS.items()}
    cgs = {"arm": (0.0, 60.0, 0.0), "fun": (0.0, Y_FUN, 180.0), "dr": (0.0, Y_FUN, 184.5), "el": (100.0, -50.0, -60.0)}
    elec_reg = box(42, 157, Y0, Y0 + T, -170, 45)

    def lumps(acc, drone):
        a = np.asarray(acc, float)
        out = [(pd[ang], m * G * a, None, cg) for ang, (cg, m) in pad_cg.items()]
        out.append((stub, ARM_G * G * a, None, cgs["arm"]))
        out.append((posts_m, FUN_G * G * a, None, cgs["fun"]))
        if drone: out.append((posts_m, DRONES[drone] * G * a, None, cgs["dr"]))
        out.append((elec_reg, ELEC_G * G * a, None, cgs["el"]))
        return out

    def Ecatch(m_g, v=0.15, drop=0.030):
        m = m_g * 1e-3; return (0.5 * m * v * v + m * 9.81 * drop) * 1000 / 3.0

    down = (0, 0, -1)
    PL = (60.0, Y_FUN + 30.0, 175.0); PLm = (-60.0, Y_FUN - 30.0, 175.0)
    Fc = Ecatch(DRONES["Overseer"]); Fm = Ecatch(DRONES["Overseer"], 0.15, 0.040)
    add_case(p, "standing, Overseer seated", "sustained", "default", down, lumps(down, "Overseer"))
    add_case(p, "Overseer catch +X+Y diag", "flight", "default", down,
             lumps(down, "Overseer") + [(posts_m, (-0.25 * Fc, -0.25 * Fc, -Fc), None, PL)])
    add_case(p, "Overseer catch -X-Y diag", "flight", "default", down,
             lumps(down, "Overseer") + [(posts_m, (0.25 * Fc, 0.25 * Fc, -Fc), None, PLm)])
    add_case(p, "misuse: Overseer motors cut", "misuse", "default", down,
             lumps(down, "Overseer") + [(posts_m, (-0.25 * Fm, -0.25 * Fm, -Fm), None, PL)])
    up = np.array([0, 0, 1.0])
    tip = lambda ang, r=140.0: np.array([r * math.cos(math.radians(ang)), Y_C, r * math.sin(math.radians(ang))])
    add_case(p, "XT60 insert at the drone 60 N", "flight", "default", down,
             lumps(down, "Scout") + [(posts_m, 60 * up, None, tip(90, 144.5)), (stub, -60 * up, None, tip(90))])
    for ang in (-90, -120):
        d = tip(ang, 1.0) - np.array([0, Y_C, 0])
        xp = trimesh.load(os.path.join(L2.CTX, f"{L2.BAYS[ang]}_xt60e1m_c.stl"), force="mesh").centroid
        P_ = bay_pose(ang)(xp)
        add_case(p, f"XT60 insert at the {ang} bay 60 N", "flight", "default", down,
                 lumps(down, "Scout") + [(pd[ang], 60 * d, None, P_), (stub, -60 * d, None, tip(ang))])
        if ang == -90:
            add_case(p, "XT60 pull at the -90 bay 50 N", "flight", "default", down,
                     lumps(down, "Scout") + [(pd[ang], -50 * d, None, P_), (stub, 50 * d, None, tip(ang))])
    a2 = (0, 0, -2); a5 = (0, 0, -5); al = (5, 0, 0)
    add_case(p, "lifted by the side sheets 2 g", "flight", "hands", a2, lumps(a2, None))
    add_case(p, "misuse: set down hard 5 g", "misuse", "default", a5, lumps(a5, None))
    add_case(p, "misuse: knocked over onto +X 5 g", "misuse", "side", al, lumps(al, None))


# --------------------------------------------------------------------------------------------------------- driver
def peak_gb():
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--h", type=float, default=6.0); ap.add_argument("--gen", type=float, default=1500.0)
    ap.add_argument("--min-member", type=float, default=18.0)
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "box_system", "v2", "topo"))
    ap.add_argument("--git", action="store_true"); ap.add_argument("--time-limit", type=float, default=5.5 * 3600)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    from dataclasses import replace
    from topo.run import Tee, git_checkpoint
    os.makedirs(a.out, exist_ok=True)
    log = Tee(os.path.join(a.out, "log.txt"))
    t0 = time.time()
    info = {}
    setup, geom = setup_factory(log, info)
    s = Settings(min_member_mm=a.min_member, gate_member=False, finish="drawn", reach_check="warn",
                 tie_min_cells=10 ** 9, time_limit_s=a.time_limit)
    log(f"=== box_system_topo_v2: h {a.h}, gen {a.gen} cm3, min member {a.min_member}; body ties off (every drawn body carries a real load)")
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
    res = dict(h=a.h, grid=list(part.shape), cells=part.nel, design_cells=int(part.design.sum()),
               min_member_mm=a.min_member, gen_cm3=a.gen,
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
