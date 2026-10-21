"""noxopt self-test: sensitivities by finite differences, overhang direction, a small end-to-end layout run, the
smooth build without and with the polish (11 Oct 2026), automatic symmetry and the thin-member check (12 Oct 2026),
the as-built thin check confirmed on shifted grids (17 Oct 2026)."""
import sys, time
import numpy as np
from scipy import ndimage as ndi
import pymoto as pym
import printopt as PO
import noxopt as NO


def small_problem(shape=(20, 12, 10), h=2.0, overhang=False, accel=True):
    nx, ny, nz = shape; nel = nx * ny * nz
    I = np.arange(nel).reshape(shape)
    solid = np.zeros(nel, bool); void = np.zeros(nel, bool)
    blocks = dict(mount=(slice(0, 2), slice(3, 9), slice(0, 4)), A=(slice(18, 20), slice(4, 8), slice(7, 10)),
                  B=(slice(9, 11), slice(0, 2), slice(0, 2)), C=(slice(13, 15), slice(10, 12), slice(4, 6)))
    for b in blocks.values(): solid[I[b].ravel()] = True
    void[I[6:9, 5:8, 3:7].ravel()] = True                      # a keep-out in the middle
    nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
    fixed = np.array([nid(0, j, k) for j in range(3, 10) for k in range(0, 5)])
    sup = dict(default=(fixed[:, None] * 3 + np.arange(3)).ravel())
    ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
    def load(b, ax, mag):
        sl = blocks[b]; el = I[sl].ravel(); F = np.zeros(ndof)
        ex, ey, ez = np.unravel_index(el, shape)
        nd = np.unique([nid(ex + a, ey + bb, ez + c) for a in (0, 1) for bb in (0, 1) for c in (0, 1)])
        F[3 * nd + ax] = mag / len(nd); return F
    cases = [PO.LoadCase("A down", load("A", 2, -20.0), cls="misuse"), PO.LoadCase("A side", load("A", 1, 10.0), cls="misuse"),
             PO.LoadCase("B up", load("B", 2, 10.0), cls="flight")]
    if accel:
        cases.append(PO.LoadCase("25 g -z", np.zeros(ndof), cls="misuse", accel=np.array([0, 0, -25.0])))
    return PO.Problem(shape=shape, h=h, origin=(0, 0, 0), solid=solid, void=void, supports=sup, cases=cases,
                      material=PO.MATERIALS["PETG"], build="+z", overhang=overhang, R=1.6, comp_factor=None)


EPS = 1e-4


def fd(overhang, masked=True):
    pb = small_problem(overhang=overhang)
    L = NO.Layout(pb, 0.3, log=lambda s: None, masked=masked)
    rng = np.random.default_rng(0)
    L.sx.state = np.clip(0.3 + 0.25 * rng.standard_normal(L.nel), 0.05, 0.95); L.sbeta.state = 4.0
    L.fn.response(); L.pmean.scale = 10.0 / float(L.s_obj.state); L.fn.response()
    worst = 0.0
    for sig in (L.s_obj, L.s_vol):
        L.fn.reset(); sig.sensitivity = 1.0; L.fn.sensitivity(); dx = L.sx.sensitivity.copy(); L.fn.reset()
        f0 = float(sig.state); x0 = L.sx.state.copy()
        idx = rng.choice(np.where(L.design)[0], 12, replace=False)
        for i in idx:
            e = EPS; xp = x0.copy(); xp[i] += e; L.sx.state = xp; L.fn.response(); fp = float(sig.state)
            xm = x0.copy(); xm[i] -= e; L.sx.state = xm; L.fn.response(); fm = float(sig.state)
            num = (fp - fm) / (2 * e); err = abs(num - dx[i]) / max(abs(num), 1e-9)
            if abs(num) > 1e-7: worst = max(worst, err)
        L.sx.state = x0; L.fn.response()
    return worst


def overhang_dir():
    """a block hanging in mid air must be removed by the filter in the build direction and kept if it sits on the bed."""
    pb = small_problem()
    for build in ("+z", "-z", "+x"):
        ax, sgn = PO.AXES[build]; nx, ny, nz = pb.shape
        dom = pym.VoxelDomain(nz, ny, nx); d = [0.0, 0.0, 0.0]; d[2 - ax] = float(sgn)
        x = np.zeros(pb.shape); sl = [slice(4, 8)] * 3
        n = pb.shape[ax]; sl[ax] = slice(n - 4, n - 1) if sgn > 0 else slice(1, 4)     # far from the bed
        x[tuple(sl)] = 1.0
        y = pym.OverhangFilter(dom, direction=d)(pym.Signal("x", x.ravel())).state
        sl[ax] = slice(0, 3) if sgn > 0 else slice(n - 3, n)                              # on the bed
        x2 = np.zeros(pb.shape); x2[tuple(sl)] = 1.0
        y2 = pym.OverhangFilter(dom, direction=d)(pym.Signal("x", x2.ravel())).state
        assert y.max() < 0.2 and y2.sum() > 0.9 * x2.sum(), (build, y.max(), y2.sum(), x2.sum())
    return True


def end_to_end():
    pb = small_problem(overhang=False)
    L = NO.Layout(pb, 0.15, log=print)
    t = time.time(); d = L.run(maxit=150, checkpoint="/tmp/noxopt_selftest.npz")
    con = NO.connectivity(d, pb); rows, rep = NO.strength_report(pb, d)
    print(rep); print(con, f"{time.time()-t:.0f}s")
    assert con["ok"], con
    smooth_build(pb, d)
    return con


def smooth_build(pb, d):
    """smooth build: the 7 Oct build (no fairing, no polish) vs the default polished build (11 Oct): one body each,
    generated volume kept within 6 %, wrinkle (p95) of the generated surface inside the box lower with the polish
    (p99 reported: on this small coarse part it moves little)."""
    out = {}
    for name, kw in (("plain", dict(fair_mm=0.0, polish=False)), ("polish", dict())):
        m = NO.smooth_surface(d, pb.solid, pb.void, pb.shape, pb.h, pb.origin, sigma_mm=0.6 * pb.h, floor_mm=3.5,
                              log=lambda s: None, **kw)
        parts = [p for p in m.split(only_watertight=False) if abs(p.volume) > 1.0]
        assert len(parts) == 1, (name, len(parts))
        # generated volume inside the design cells (what the build keeps), counted on a 4x finer grid
        # (noxopt's own voxeliser: trimesh contains() falls back to an rtree ray search that needs gigabytes here)
        f = 4; hh = pb.h / f
        axes = [np.asarray(pb.origin)[k] + (np.arange(n * f) + 0.5) * hh for k, n in enumerate(pb.shape)]
        des = (~pb.solid & ~pb.void).reshape(pb.shape).repeat(f, 0).repeat(f, 1).repeat(f, 2).ravel()
        inside = NO._voxelise(parts[0], *axes).ravel()[des]
        # smoothness of the generated surface only: the drawn blocks keep their sharp edges in the polish (as they must)
        S4 = pb.solid.reshape(pb.shape).repeat(f, 0).repeat(f, 1).repeat(f, 2)
        dS = ndi.distance_transform_edt(~S4) * hh; o4 = np.asarray(pb.origin) + hh / 2
        box = np.array(pb.shape) * pb.h                      # and only inside the design box: a real build trims at the
        skip = lambda p: (ndi.map_coordinates(dS, ((p - o4) / hh).T, order=1, mode="nearest") < 1.5) | \
            np.any((p - np.asarray(pb.origin) < 1.0) | (np.asarray(pb.origin) + box - p < 1.0), 1)     # envelope
        out[name] = (inside.sum() * hh ** 3, NO.wrinkle(parts[0], skip=skip))
    (v0, w0), (v1, w1) = out["plain"], out["polish"]
    print(f"smooth build: generated volume {v0:.0f} -> {v1:.0f} mm3 with the polish; wrinkle p95 {w0[0]:.3f} -> {w1[0]:.3f}, "
          f"p99 {w0[1]:.3f} -> {w1[1]:.3f} 1/mm (generated surface inside the box)")
    assert abs(v1 - v0) / v0 < 0.06, (v0, v1)
    assert w1[0] <= w0[0], (w0, w1)


def sym_and_thin():
    """auto symmetry: mirrored masks give a mirror callable for that axis only. thin members: a 1-cell rod is found, a
    3-cell member is not, and a 2-cell member flush on a keep-out wall is found (the build shaves it under the floor)."""
    shape = (20, 12, 10); h = 1.0; nel = int(np.prod(shape)); I = np.arange(nel).reshape(shape)
    solid = np.zeros(nel, bool); void = np.zeros(nel, bool)
    solid[I[0:2, :, :].ravel()] = True; solid[I[18:20, 4:8, :].ravel()] = True
    void[I[8:12, 0:3, :].ravel()] = True; void[I[8:12, 9:12, :].ravel()] = True
    pb = PO.Problem(shape=shape, h=h, origin=(0, 0, 0), solid=solid, void=void, supports={}, cases=[],
                    material=PO.MATERIALS["PETG"], overhang=False, R=2.0, comp_factor=None)
    fn, axes, near = NO.auto_symmetry(pb)
    assert axes == [1, 2], axes                              # y and z mirror, x does not (two different end blocks)
    v = np.random.default_rng(1).random(nel); w = fn(v).reshape(shape)
    assert np.allclose(w, w[:, ::-1, :]) and np.allclose(w, w[:, :, ::-1]) and abs(w.sum() - v.sum()) < 1e-9
    d = np.zeros(shape); d[2:18, 5:8, 4:7] = 1.0             # 3 x 3 member: fine
    assert not NO.thin_members(d.ravel(), pb, 2.0)[1]
    d2 = d.copy(); d2[2:18, 10, 1] = 1.0; d2[2:8, 10, 1:5] = 1.0   # + a 1-cell rod
    cells, info = NO.thin_members(d2.ravel(), pb, 2.0)
    assert info and cells.reshape(shape)[12:16, 10, 1].all() and not cells.reshape(shape)[10, 6, 5], info
    d3 = np.zeros(shape); d3[2:18, 3:5, 4:8] = 1.0           # 2 cells wide, flush on the keep-out at y < 3
    assert NO.thin_members(d3.ravel(), pb, 2.0, margin_mm=0.75)[1] and not NO.thin_members(d3.ravel(), pb, 2.0, margin_mm=0.0)[1]
    return True


def thin_built():
    """as-built thin check (17 Oct 2026): a real 0.8 rod on a block is found on the base grid and confirmed on every
    shifted grid; a plain block whose faces sit exactly on the base grid's sample planes passes; offsets=() keeps the
    old single-grid behaviour."""
    import trimesh
    blk = trimesh.creation.box(extents=[12.0, 12.0, 12.0]); blk.apply_translation([6.0, 6.0, 6.0])
    rod = trimesh.creation.box(extents=[20.0, 0.8, 0.8]); rod.apply_translation([22.0, 6.0, 6.0])
    part = trimesh.boolean.union([blk, rod], engine="manifold")
    q = lambda s: None
    found = NO.thin_check(part, None, 2.0, log=q)
    assert len(found) == 1 and abs(found[0]["at"][0] - 22.0) < 3.0, found
    assert len(NO.thin_check(part, None, 2.0, log=q, offsets=())) == 1
    face = trimesh.creation.box(extents=[20.0, 8.0, 8.0]); face.apply_translation([10.0 - 0.2, 4.0 - 0.2, 4.0 - 0.2])
    assert not NO.thin_check(face, None, 2.0, log=q)
    return True


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("all", "fd"):
        for oh, mk in ((False, True), (True, True), (False, False)):        # masked=False: plain filter (16 Oct)
            w = fd(oh, mk); print(f"finite-difference check, overhang {oh}, masked {mk}: worst relative error {w:.2e}"); assert w < 5e-3, w
    if what in ("all", "oh"):
        print("overhang direction mapping ok:", overhang_dir())
    if what in ("all", "run"):
        end_to_end()
    if what in ("all", "sym"):
        print("auto symmetry and thin-member check ok:", sym_and_thin())
    if what in ("all", "thin"):
        print("as-built thin check on shifted grids ok:", thin_built())
