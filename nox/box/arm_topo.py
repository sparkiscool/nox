"""Box swap arm v1: generic noxopt pipeline (20 Oct: every build ends in solid_check.solid_check: one body after a 0.9 mm
erosion, every drawn feature in it, no wall under 1.8; solid_ok False = not ready to print, whatever else passes) for a part given as manifold3d bodies (drawn, cuts, env) plus
supports and load cases defined on regions. Follows noxopt_demo.py step for step (layout -> prune thin ->
connectivity -> polished build -> function / thin / wrinkle / as-built strength -> STL).
17 Oct: prune_thin runs one round per call (prune_loop: at most two layouts alive; three passed 7 GB on head A v5),
every round checkpoints and resumes, a layout that already stopped is not iterated again on a rerun, and the failed
build's meshes are freed before a re-prune."""
import sys, os, copy, math, time, json, gc
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO, printopt as PO
from arm_cad import tm, man, box

FLOOR, CLIP = 2.0, 0.5
log = lambda s: print(str(s)[:400], flush=True)


class PartProblem:
    """voxelises the bodies on a grid of cell h over the env's bounding box (+1 cell margin), builds masks, supports and
    load cases, returns a printopt.Problem. Regions (supports / loads) are Manifolds: the elements whose centre lies inside."""

    def __init__(self, name, h, drawn, cuts, env, build="+z", overhang=False, material="PETG", pad=1):
        self.name, self.h = name, h
        bb = env.bounding_box()
        lo = np.floor(np.array(bb[:3]) / h) * h - pad * h
        hi = np.ceil(np.array(bb[3:]) / h) * h + pad * h
        self.shape = tuple(int(round((hi[i] - lo[i]) / h)) for i in range(3)); self.origin = tuple(lo)
        self.axes = [lo[k] + (np.arange(self.shape[k]) + 0.5) * h for k in range(3)]
        self.nel = int(np.prod(self.shape))
        self.env_t, self.drawn_t, self.cuts_t = tm(env), tm(drawn), tm(cuts)
        inside_env = self.vox(self.env_t); inside_cut = self.vox(self.cuts_t); inside_drawn = self.vox(self.drawn_t)
        self.solid = (inside_drawn & ~inside_cut).ravel()
        self.void = (~inside_env | inside_cut).ravel() & ~self.solid
        self.design = ~self.solid & ~self.void
        nx, ny, nz = self.shape
        self.nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
        self.ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
        self.supports, self.cases = {}, []
        self.build, self.overhang, self.material = build, overhang, material
        log(f"{name}: grid {self.shape} h {h} = {self.nel:,} cells; design {int(self.design.sum()):,} "
            f"({self.design.sum() * h ** 3 / 1000:.0f} cm3), solid {int(self.solid.sum()):,}, void {int(self.void.sum()):,}")

    def vox(self, t):
        return NO._voxelise(t, *self.axes)

    def elems(self, region, within_solid=None):
        """element indices whose centre is inside `region` (Manifold)"""
        m = self.vox(tm(region)).ravel()
        if within_solid is not None: m &= within_solid
        return np.where(m)[0]

    def nodes_of(self, el):
        ex, ey, ez = np.unravel_index(el, self.shape)
        return np.unique(np.concatenate([self.nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))

    def support(self, name, region):
        el = self.elems(region, self.solid | self.design)
        nd = self.nodes_of(el); self.supports[name] = (nd[:, None] * 3 + np.arange(3)).ravel()
        log(f"  support {name}: {len(el)} elements, {len(nd)} nodes")

    def load(self, name, region, vec, cls="misuse", support="default", accel=None):
        el = self.elems(region, self.solid | self.design)
        if len(el) == 0: raise ValueError(f"load {name}: empty region")
        nd = self.nodes_of(el); F = np.zeros(self.ndof)
        for a in range(3): F[3 * nd + a] = vec[a] / len(nd)
        self.cases.append(PO.LoadCase(name, F, cls=cls, support=support, accel=None if accel is None else np.asarray(accel, float)))

    def accel_case(self, name, g, cls="misuse", support="default"):
        self.cases.append(PO.LoadCase(name, np.zeros(self.ndof), cls=cls, support=support, accel=np.asarray(g, float)))

    def problem(self, R=None):
        R = max(2.0, 2.0 / self.h) if R is None else R
        return PO.Problem(shape=self.shape, h=self.h, origin=self.origin, solid=self.solid.copy(), void=self.void.copy(),
                          supports=self.supports, cases=self.cases, material=PO.MATERIALS[self.material],
                          build=self.build, overhang=self.overhang, R=R, comp_factor=None)


def prune_loop(L, dens, name, stage, built=None, rounds=3):
    """17 Oct: noxopt.prune_thin one round per call. prune_thin(rounds=3) keeps every round's Layout alive at once (the
    caller's, the last round's and the new one): three layouts of a 266k-cell part passed 7 GB (head A v5, OOM). Here at
    most two are alive. Each round checkpoints to <name>_prune<stage>_<round>_ck.npz and resumes from it (the pruned
    cells are recomputed from the same density, so the resumed state matches its problem)."""
    removed = []
    for r in range(1, rounds + 1):
        ck = f"{name}_prune{stage}_{r}_ck.npz"
        L, dens, rem = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, rounds=1, maxit=60,
                                     built=built if r == 1 else None, checkpoint=ck, resume=ck)
        gc.collect()
        removed += rem
        if not rem: break
    L.save(f"{name}_prune{stage}_density.npz", dens)
    return L, dens, removed


def run_part(pp, drawn, cuts, env, gen_cm3, out_stl, maxit=200, mirror=None, time_limit=5.5 * 3600, extra_check=None):
    """the demo pipeline on a PartProblem. gen_cm3 = generated volume target. Returns dict of results."""
    H = pp.h; pb = pp.problem(); pb0 = copy.copy(pb)
    allowed = env - cuts
    vf = gen_cm3 * 1000 / (pp.design.sum() * H ** 3)
    log(f"{pp.name}: mass target {gen_cm3} cm3 generated = volfrac {vf:.3f} of the design cells; {len(pb.cases)} cases")
    t0 = time.time()
    L = NO.Layout(pb, vf, log=log)
    ck = f"{pp.name}_ck.npz"; mi = maxit
    if os.path.exists(ck):                                           # 17 Oct: a layout that already stopped is not iterated again
        c = np.load(ck, allow_pickle=True)
        if "stopped" in str(c["extra"]): mi = int(c["it"])
    dens = L.run(maxit=mi, checkpoint=ck, resume=ck, time_limit=time_limit)
    L.save(f"{pp.name}_density.npz", dens)
    L, dens, removed = prune_loop(L, dens, pp.name, 0)
    log(f"layout done {time.time() - t0:.0f} s; pruned {len(removed)}")

    def build(d):
        surf = NO.smooth_surface(d, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn),
                                 clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
        S = man(surf)
        return ((S + drawn) ^ env) - cuts

    thin = []
    for attempt in range(3):
        con = NO.connectivity(dens, L.pb0); log(f"connectivity: {con}")
        part = build(dens); P = tm(part)
        thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
        if not thin or attempt == 2: break
        del part, P; gc.collect()                                            # 17 Oct: free the built meshes before a re-prune
        L, dens, _ = prune_loop(L, dens, pp.name, attempt + 1, built=thin)
    bodies = part.decompose(); nb = len(bodies)
    if nb > 1:                                                       # report loose specks; never "keep the largest" silently
        vols = sorted([b.volume() for b in bodies], reverse=True)
        log(f"WARNING: {nb} bodies, volumes {[round(v) for v in vols[:6]]} mm3")
    fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
    A = NO._voxelise(P, *pp.axes).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A), title=pp.name)
    log(rep)
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    from solid_check import solid_check                               # 20 Oct (Spark 01:08): no disconnected structures, ever
    solid_ok, solid = solid_check(P, drawn=[tm(drawn - cuts)])
    log(f"solid check: {'PASS' if solid_ok else 'FAIL (not ready to print)'} {solid}")
    P.export(out_stl)
    res = dict(name=pp.name, solid_ok=solid_ok, solid=solid, bodies=nb, watertight=bool(P.is_watertight), mass_g=P.volume / 1000 * 1.27,
               function_check=[round(float(x), 2) for x in fc], thin=len(thin), wrinkle=[round(wr[0], 3), round(wr[1], 3)],
               worst=round(max(w["inlayer"], w["across"]), 2), worst_case=w["case"], gen_cm3=gen_cm3,
               drawn_g=(drawn - cuts).volume() / 1000 * 1.27, time_s=round(time.time() - t0), stl=out_stl,
               strength_rows=[(r["case"], round(r["inlayer"], 2), round(r["across"], 2)) for r in rows])
    log("RESULT " + json.dumps(res))
    json.dump(res, open(f"{pp.name}_result.json", "w"), indent=1)
    return res, P
