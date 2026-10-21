"""After arm_run_baymod_nb.py (17 Oct 2026): the drone-neighbour bay modules as print files, and their checks.
    python post_nb.py   -> out/box_baymod_nb30_v1.3mf, out/box_baymod_nb150_v1.3mf (print pose, far end down), *_M.ply
                           (module frame), out/post_nb.json
30 deg: the run's body, bed cut flat at Y 249 (DEFAULT: the polish rounded the far end to 6 mm2 at Y 253).
150 deg (v4c attempt, fails one_body: generate it in its own run instead): the 30 deg body mirrored in X with its tongue joint removed, then built with the run's own formula against the
150 deg problem: ((body + drawn) ^ env) - cuts, so the groove joint, cuts and keep-outs are exact (the heads' clamps are
not mirror images, so a plain mirror enters 2 to 3 mm3 of keep-out). 3MF keeps the exact topology (STL float merge breaks
coincident faces into slivers).
Checks: function by exact booleans, thin, wrinkle, as-built strength (150), collar clearance (conservative collar: every
face + 12 behind it, flanges included), the 0 / 180 deg neighbour sectors, arm sweep (bars r < 136.5 for Y < 95.5,
parked heads r < 98.5), bed / overhang areas."""
import math, json, copy
import numpy as np, trimesh, manifold3d as mf
import arm_layout as L, arm_cad as A
from arm_cad import box, cylY, man
import noxopt as NO
import arm_run_baymod_nb as NB

OUT = "out"; S2 = math.sqrt(2); Y_BED = 249.0

def tm(m):                                    # exact topology, no float merge
    me = m.to_mesh(); idx = np.arange(len(me.vert_properties))
    if len(me.merge_from_vert): idx[np.asarray(me.merge_from_vert)] = np.asarray(me.merge_to_vert)
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], idx[np.asarray(me.tri_verts)], process=False)

def print_pose(m):                            # far end (Y max) down = build -y
    t = tm(m); t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [1, 0, 0]))
    t.apply_translation(-t.bounds[0]); return t

def to_W(m, angle): return m.rotate([0, 90.0 - angle, 0])

def collar_W():
    Z0, K0, ZP, ZT = NB.SEAT_Z0, NB.K0, -1.2, 77.2
    def fr(off, z0, z1):
        a0, a1 = K0 + off + z0, K0 + off + z1
        return mf.Manifold.batch_hull([mf.Manifold.cube([2 * a0, 2 * a0, 0.001]).translate([-a0, -a0, z0]),
                                       mf.Manifold.cube([2 * a1, 2 * a1, 0.001]).translate([-a1, -a1, z1 - 0.001])])
    c = (fr(12.0 * S2, ZP, ZT) - fr(0.0, ZP - 5, ZT + 5)) - box(-84.0, 84.0, -84.0, 84.0, ZP - 2, 1.3)
    return c.translate([0.0, L.Y_C, Z0])

def one_body(m, tag):
    """no disconnected structures (Spark 17 Oct 09:19): nothing is ever deleted; a second body stops the script."""
    d = m.decompose()
    if len(d) != 1:
        raise SystemExit(f"{tag}: {len(d)} bodies (volumes {sorted(round(x.volume(), 3) for x in d)[-5:]} mm3): rerun, never delete")
    return m

def checks(m, side, strength=False):
    P = NB.parts(side); drawn, cuts, env = P["drawn"], P["cuts"], P["env"]; allowed = env - cuts
    M = tm(m)
    d = dict(g=round(m.volume() * 1.27e-3, 1), bodies=len(m.decompose()), watertight=bool(M.is_watertight),
             function_exact_mm3=[round(((drawn - cuts) - m).volume(), 2), round((m ^ cuts).volume(), 2), round((m - env).volume(), 2)],
             thin=len(NO.thin_check(M, tm(drawn - cuts), 2.0, log=lambda s: None)))
    wr = NO.wrinkle(M, drawn=tm(drawn - cuts), allowed=tm(allowed)); d["wrinkle"] = [round(wr[0], 3), round(wr[1], 3)]
    T = print_pose(m); n, c, a = T.face_normals, T.triangles_center, T.area_faces; z0 = T.bounds[0, 2]
    d.update(print_mm=T.extents.round(1).tolist(), bed_mm2=round(float(a[(n[:, 2] < -0.999) & (c[:, 2] < z0 + 0.05)].sum())),
             overhang_mm2=round(float(a[(n[:, 2] < -math.cos(math.radians(45.5))) & (c[:, 2] > z0 + 0.3)].sum())))
    if strength:
        pp, _ = NB.setup(side, h=2.5); pb0 = pp.problem()
        Av = NO._voxelise(M, *pp.axes).ravel().astype(float)
        pbs = copy.copy(pb0); pbs.void = (Av < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
        rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, Av), title="nb150")
        w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
        d.update(worst=round(max(w["inlayer"], w["across"]), 2), worst_case=w["case"])
    return d, T

if __name__ == "__main__":
    t = trimesh.load(f"{OUT}/box_baymod_nb_v1.stl", force="mesh"); t.merge_vertices()
    run = man(t).decompose(); b30 = sorted(run, key=lambda x: -x.volume())[0] - box(-300, 300, Y_BED, 400, -10, 400)
    P = NB.parts(-1); jb, jc = P["joint"]
    S = b30.mirror([1, 0, 0]) - jb.mirror([1, 0, 0])
    b30 = one_body(b30, "nb30")
    b150 = one_body((((S + P["drawn"]) ^ P["env"]) - P["cuts"]) - box(-300, 300, Y_BED, 400, -10, 400), "nb150")   # 17 Oct: 24 slivers -> generate the 150 deg module in its own run
    info = {}
    for tag, m, side in (("nb30", b30, +1), ("nb150", b150, -1)):
        d, T = checks(m, side, strength=(side < 0))
        T.export(f"{OUT}/box_baymod_{tag}_v1.3mf"); tm(m).export(f"{OUT}/box_baymod_{tag}_v1_M.ply")
        info[tag] = d; print(tag, d, flush=True)
    col = tm(collar_W()); pq = trimesh.proximity.ProximityQuery(col)
    room = None
    for p in NB.SEATS_W:
        for sx in (1, -1):
            q = p.copy(); q[0] *= sx; n = NB.N_W.copy(); n[0] *= sx
            c = NB.along(q, n, NB.KEEP_R + 0.5, -8.0, NB.KEEP_T + 2.0); room = c if room is None else room + c
    for tag, m, ang in (("nb30", b30, 30.0), ("nb150", b150, 150.0)):
        info[f"collar_clear_{tag}"] = round(float(-pq.signed_distance(tm(to_W(m, ang) - room).sample(60000)).max()), 2)
        v = np.asarray(tm(m).vertices); r = np.hypot(v[:, 0], v[:, 2]); lo = v[:, 1] < 95.5
        info[f"arm_{tag}"] = dict(bar_clear=round(float(r[lo].min() - 136.5), 2), head_clear=round(float(r.min() - 98.5), 2))
    sect = (cylY(0, 0, L.MOD_Y[0], L.MOD_Y[1], L.MOD_R[1], 180) - cylY(0, 0, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1, L.MOD_R[0], 180)) ^ \
        A.wedge(L.HALF, L.JOINT_GAP, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1)
    info["collar_clear_neighbour_sector"] = round(float(-pq.signed_distance(tm(to_W(sect, 0.0)).sample(60000)).max()), 2)
    json.dump(info, open(f"{OUT}/post_nb.json", "w"), indent=1)
    print(json.dumps(info, indent=1))
