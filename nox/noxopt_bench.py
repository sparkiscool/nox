"""Benchmark: a Scout-style holder part (P3/v2-style load set) at any cell size, run by the current noxopt or by
noxopt_local.  python noxopt_bench.py ENGINE H [maxit] [out.json]   ENGINE = noxopt | pardiso | gmg"""
import sys, json, time
try:
    import resource
except ImportError:          # Windows
    resource = None
import numpy as np
import printopt as PO


def holder(h=1.0, size=(60.0, 40.0, 30.0)):
    """60 x 40 x 30 mm holder: 4 bolt bosses (bottom corners), pack pad (top), 2 hook roots (sides), camera pad
    (front), battery channel keep-out. 3 support groups, 28 cases (14 misuse directions, flight, landing, hand and
    retention loads, catcher grab)."""
    nx, ny, nz = (int(round(s / h)) for s in size); shape = (nx, ny, nz); nel = nx * ny * nz
    c = np.stack(np.meshgrid(*(np.arange(n) * h + h / 2 for n in shape), indexing="ij"), -1).reshape(-1, 3)
    X, Y, Z = c.T
    box = lambda x0, x1, y0, y1, z0, z1: (X > x0) & (X < x1) & (Y > y0) & (Y < y1) & (Z > z0) & (Z < z1)
    bolts = [box(x - 3, x + 3, y - 3, y + 3, 0, 6) for x in (5, 55) for y in (5, 35)]
    pad = box(22, 38, 14, 26, 26, 30)
    hookL = box(26, 34, 0, 3, 14, 24); hookR = box(26, 34, 37, 40, 14, 24)
    cam = box(56, 60, 15, 25, 8, 18)
    solid = np.zeros(nel, bool)
    for m in bolts + [pad, hookL, hookR, cam]: solid |= m
    void = box(10, 50, 8, 32, 6, 22)                         # battery channel
    void |= (Z > 26) & ~pad & box(8, 52, 6, 34, 26, 30)      # lift path above the channel, except the pad
    solid &= ~void
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
    I = np.arange(nel).reshape(shape)

    def nodes_of(mask):
        el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez + cc) for a in (0, 1) for b in (0, 1) for cc in (0, 1)]))

    def bottom_nodes(mask):                                   # nodes on the underside of a block (bolt seats)
        el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
        m = ez == ez.min(); ex, ey, ez = ex[m], ey[m], ez[m]
        return np.unique(np.concatenate([nid(ex + a, ey + b, ez) for a in (0, 1) for b in (0, 1)]))

    dofs = lambda nd: (nd[:, None] * 3 + np.arange(3)).ravel()
    sup = dict(bolts=dofs(np.concatenate([bottom_nodes(b) for b in bolts])),
               grab=dofs(np.concatenate([nodes_of(hookL), nodes_of(hookR)])),
               pad=dofs(nodes_of(pad)))

    def load(mask, vec):
        nd = nodes_of(mask); F = np.zeros(ndof)
        for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
        return F
    z0 = np.zeros(ndof); cases = []
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / np.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs):                               # 25 g inside the cage, pack 165 g on the pad
        cases.append(PO.LoadCase(f"misuse {k}", load(pad, 0.165 * 9.81 * 25 * d), cls="misuse", support="bolts", accel=25 * d))
    cases.append(PO.LoadCase("flight", load(pad, 0.165 * 9.81 * np.array([0, 2.0, -4.0])), cls="flight", support="bolts",
                             accel=np.array([0, 2.0, -4.0])))
    cases.append(PO.LoadCase("landing 6g", load(pad, 0.165 * 9.81 * np.array([0, 0, -6.0])), cls="misuse", support="bolts",
                             accel=np.array([0, 0, -6.0])))
    for nm, m, v in [("pinch L", hookL, (0, 20, 0)), ("pinch R", hookR, (0, -20, 0)), ("retain L", hookL, (0, 0, 20)),
                     ("retain R", hookR, (0, 0, 20)), ("poke cam x", cam, (-5, 0, 0)), ("poke cam z", cam, (0, 0, -5)),
                     ("poke cam y", cam, (0, 5, 0)), ("preload", pad, (0, 0, 12))]:
        cases.append(PO.LoadCase(nm, load(m, v), cls="misuse", support="bolts"))
    for nm, v in [("grab lift", (0, 0, -0.165 * 9.81 * 6)), ("grab side", (0, 0.165 * 9.81 * 3, 0))]:
        cases.append(PO.LoadCase(nm, load(pad, v), cls="misuse", support="grab", accel=np.array(v) / (0.165 * 9.81)))
    for nm, v in [("pad push", (0, 0, -15)), ("pad shear", (10, 0, 0))]:
        cases.append(PO.LoadCase(nm, load(bolts[0] | bolts[3], v), cls="misuse", support="pad"))
    R = max(2.0, 2.0 / h)
    return PO.Problem(shape=shape, h=h, origin=(0, 0, 0), solid=solid, void=void, supports=sup, cases=cases,
                      material=PO.MATERIALS["PETG"], build="+z", overhang=False, R=R, comp_factor=None)


def peak_gb():
    """peak memory of this process, GB (Linux ru_maxrss; Windows peak working set via psutil)."""
    if resource:
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
    try:
        import psutil
        return psutil.Process().memory_info().peak_wset / 1e9
    except Exception:
        return None


if __name__ == "__main__":
    eng, h = sys.argv[1], float(sys.argv[2])
    maxit = int(sys.argv[3]) if len(sys.argv) > 3 else 220
    out = sys.argv[4] if len(sys.argv) > 4 else None
    pb = holder(h)
    print(f"holder h {h}: {pb.shape}, {pb.shape[0]*pb.shape[1]*pb.shape[2]:,} cells, design "
          f"{int((~pb.solid & ~pb.void).sum()):,}, cases {len(pb.cases)}", flush=True)
    t0 = time.time(); times = []
    log = lambda s: (print(s, flush=True), times.append(time.time()))
    if eng == "noxopt":
        import noxopt as NO; L = NO.Layout(pb, 0.12, log=log)
    else:
        import noxopt_local as NL; L = NL.Layout(pb, 0.12, log=log, backend=eng)
    t_setup = time.time() - t0
    d = L.run(maxit=maxit, checkpoint=f"bench_{eng}_{h}.npz")
    t_run = time.time() - t0 - t_setup
    import noxopt as NO
    con = NO.connectivity(d, pb)
    res = dict(engine=eng, h=h, cells=int(np.prod(pb.shape)), its=len(L.hist), stopped=L.stopped,
               obj=L.hist[-1][2], vol=L.hist[-1][4], grey=L.hist[-1][5], bodies=con["bodies"], setup_s=t_setup,
               run_s=t_run, s_per_it=t_run / max(len(L.hist), 1),
               peak_gb=peak_gb())
    if eng != "noxopt" and getattr(L.comp, "stats", None):
        its = [np.mean(s[1]) for s in L.comp.stats if s[1]]
        res["cg_its_mean"] = float(np.mean(its)) if its else 0.0
    np.save(f"dens_{eng}_{h}.npy", d)
    print(json.dumps(res), flush=True)
    if out: json.dump(res, open(out, "w"))
