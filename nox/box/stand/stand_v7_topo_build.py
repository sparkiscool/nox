"""Rebuild a stand v7 topology part from its saved (pruned) density without rerunning the layout: the same polished build,
checks and as-built strength as arm_topo.run_part, plus the print-pose STL, preview and fit numbers.
python3 stand_v7_topo_build.py <part> [h] [density.npz]   (default <part>_prune0_density.npz)"""
import sys, os, copy, json
import numpy as np, trimesh, manifold3d as mf
os.environ["NOX_SETUP_ONLY"] = "1"
import noxopt as NO, gc
if os.environ.get("NOX_BACKEND"):                                   # 20 Oct kb v8: as-built recheck on the multigrid solver (pardiso OOM at 6 GB)
    import noxopt_local as NL
    NO.strength_report = lambda pb, dens, thr=0.5, title="": NL.strength_report(pb, dens, thr, backend=os.environ["NOX_BACKEND"])
from arm_cad import tm, man
import arm_topo as T
import stand_v7_topo as X
import stand_v7_parts as V

part = sys.argv[1]; h = float(sys.argv[2]) if len(sys.argv) > 2 else X.H_DEFAULT[part]
dfile = sys.argv[3] if len(sys.argv) > 3 else f"{part}_prune0_density.npz"
pp, drawn, cuts, env, gen = X.SETUP[part](h); env = env + drawn
pb0 = pp.problem(); allowed = env - cuts
d = np.load(dfile, allow_pickle=True); dens = d["rho_phys"]
assert tuple(d["shape"]) == tuple(pb0.shape), (d["shape"], pb0.shape)
solid, void = d["solid"], d["void"]                                # the pruned problem's masks (prune adds void cells)
surf = NO.smooth_surface(dens, solid, void, pb0.shape, h, pb0.origin, keep_mesh=tm(drawn), clip_mesh=tm(allowed),
                         clip_margin=T.CLIP, sigma_mm=0.6 * h, floor_mm=T.FLOOR, log=T.log)
m = ((man(surf) + drawn) ^ env) - cuts
# 20 Oct (bar_half v8): the generated web stops at the density's cell edge + the 0.5 clip margin, ~0.6 above a flat bed
# face (7400 mm2 floating, M). BED_SKIN = {part: (bed plane, section plane)}: the part's own outline at the section plane,
# extruded flat to the bed plane, joins the drawn bodies (rebuilt from the same density, no new layout).
BED_SKIN = dict(bar_half=(46.0, 48.0))
if part in BED_SKIN and not os.environ.get("NOX_NO_SKIN"):
    yb, ys = BED_SKIN[part]
    sec = tm(m).section(plane_origin=[0, ys, 0], plane_normal=[0, 1, 0])
    T2 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, -ys], [0, 0, 0, 1]], float)       # (X, Z) in the plane
    p2, _ = sec.to_2D(to_2D=T2)
    from stand_v7_parts import prism_xz
    skin = None
    for poly in p2.polygons_full:
        poly = poly.buffer(0)
        for g in getattr(poly, "geoms", [poly]):
            if g.area < 4.0: continue
            s = prism_xz(list(g.exterior.coords)[:-1], yb, ys)
            for hole in g.interiors:
                if abs(type(g)(hole).area) > 1.0: s -= prism_xz(list(hole.coords)[:-1], yb - 1, ys + 1)
            skin = s if skin is None else skin + s
    T.log(f"bed skin: Y {yb}..{ys}, {skin.volume() / 1000:.2f} cm3 from the outline at Y {ys}")
    drawn = drawn + skin
    m = ((man(surf) + drawn) ^ env) - cuts
def drop_zero_volume(m, eps=0.01):
    """20 Oct kb v8: a coplanar face (generated surface on the spigot bore floor, Z -198) left a 2-triangle, 0.0 mm3 sheet.
    Only components under eps mm3 (no material) go; any real loose piece (>= eps) stays and fails the checks."""
    parts = m.decompose(); keep = [b for b in parts if abs(b.volume()) >= eps]
    if len(keep) < len(parts): print(f"dropped {len(parts) - len(keep)} zero-volume sheet(s): {[round(b.volume(), 4) for b in parts if abs(b.volume()) < eps]} mm3", flush=True)
    return mf.Manifold.batch_boolean(keep, mf.OpType.Add) if len(keep) > 1 else keep[0]
m = drop_zero_volume(m)
P = tm(m); nb = len(m.decompose())
thin = NO.thin_check(P, tm(drawn - cuts), T.FLOOR, log=T.log)
fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=T.log)
os.makedirs("out", exist_ok=True); P.export(f"out/box_{part}_v8_W.stl")           # saved before the heavy steps (20 Oct kb v8: OOM lost the mesh twice)
# the solid check at pitch 0.25 needs the RAM to itself (kb v8: killed in-process at 6.1 GB, PASS standalone): run it as its own process
import subprocess; r = subprocess.run([sys.executable, "solid_check.py", f"out/box_{part}_v8_W.stl"], capture_output=True, text=True)
solid_ok = r.returncode == 0; solid = r.stdout.strip()[-1500:]; print("solid check", "PASS" if solid_ok else "FAIL (not ready to print)", solid, flush=True)
wr = NO.wrinkle(P, pitch=float(os.environ.get("NOX_WRINKLE_PITCH", 0.4)), drawn=tm(drawn - cuts), allowed=tm(allowed)); gc.collect()
A = NO._voxelise(P, *pp.axes).ravel().astype(float)
pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A), title=part); print(rep)
w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
tp = V.print_pose(part, m); tp.export(f"out/box_{part}_v8.stl")
res = dict(name=part, solid_ok=solid_ok, solid=solid, bodies=nb, watertight=bool(P.is_watertight), mass_g=round(P.volume / 1000 * 1.27, 1), function_check=[round(float(x), 2) for x in fc],
           thin=len(thin), wrinkle=[round(wr[0], 3), round(wr[1], 3)], worst=round(max(w["inlayer"], w["across"]), 2), worst_case=w["case"],
           gen_cm3=gen, drawn_g=round((drawn - cuts).volume() / 1000 * 1.27, 1), print_bbox=np.round(tp.extents, 1).tolist(),
           strength_rows=[(r["case"], round(r["inlayer"], 2), round(r["across"], 2)) for r in rows])
print("RESULT " + json.dumps(res)); json.dump(res, open(f"{part}_result.json", "w"), indent=1)
