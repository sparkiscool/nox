"""Hub v5 print set, drawn parts (18 Oct 2026; stub v3 18 Oct, arm chat): builds every drawn part of the arm's hub set from arm_cad / detent, picks
the bed face by the docs/printing.md rule (print_orient: largest bed contact among the 6 axis faces, no first-layer island under
20 mm2 lasting more than 10 mm), saves each STL in that pose and checks one watertight body inside the 180 cube.
python3 export_hub_set.py -> out/<name>.stl + out/hub_set.json   (hub v5 itself: arm_run_hub.py, then post_hub_v5.py)"""
import json, math
import numpy as np, trimesh
import arm_cad as A, detent as DT, print_orient as PO

OUT = "out"
D = 1.27                                                                    # PETG g/cm3 (E)


def parts():
    c = DT.CHOSEN; Dc = DT.design(c["th"], c["s"], c["a"], c["b"], c["phi_d"], c["psi"]); S = DT.arm_solids(Dc, np.array(c["Q"]))
    out = {"box_stub_v3": (A.stub(), 1), "box_sun_post_v1": (A.sun_post(), 1),
           "box_detent_ring_v1": (DT.ring(c["th"]), 1), "box_detent_arm_v1": (S["arm"] + S["spacer"], 1),
           "box_idler_end_A_v3": (A.idler_end("A"), 1), "box_idler_end_B_v3": (A.idler_end("B"), 1),
           "box_pinion_v4": (A.pinion_u(), 2)}
    for v in ("A", "B"):                                                    # head v's two fingers (B mirrored, box-arm Problem 16)
        F = A.fingers_and_racks(v)
        for k in ("A", "B"): out[f"box_finger_{k}_for_head_{v}_v4"] = (F[k], 1)
    return out


def best_pose(t):
    rows = []
    for ax in range(3):
        for s in (1, -1):
            m = t.copy(); m.apply_transform(PO.rot_to_down(ax, s)); m.apply_translation(-m.bounds[0])
            n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]
            bed = float(a[(n[:, 2] < -0.99) & (zc < 0.3)].sum())
            over = float(a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < 0.3))].sum())
            first, run = PO.island_check(m)
            rows.append(dict(face=f"{'+' if s > 0 else '-'}{'XYZ'[ax]} down (W)", bed=round(bed), over=round(over), run=round(run), mesh=m))
    ok = [r for r in rows if r["run"] <= 10] or rows
    best = max(ok, key=lambda r: (r["bed"], -r["over"]))
    return best, rows


if __name__ == "__main__":
    import sys
    only = sys.argv[1:]                                                     # optional part names: rebuild just those
    info = {}
    for name, (m, qty) in parts().items():
        if only and name not in only: continue
        t = A.tm(m); t.merge_vertices()
        best, rows = best_pose(t)
        P = best["mesh"]; P.export(f"{OUT}/{name}.stl")
        info[name] = dict(qty=qty, g=round(m.volume() / 1000 * D, 2), bodies=len(m.decompose()), watertight=bool(P.is_watertight),
                          extents=P.extents.round(1).tolist(), fits_180=bool((P.extents <= 180).all()), bed_face=best["face"],
                          bed_mm2=best["bed"], overhang_mm2=best["over"], thin_island_run=best["run"],
                          faces=[(r["face"], r["bed"], r["over"], r["run"]) for r in rows])
        print(name, {k: v for k, v in info[name].items() if k != "faces"}, flush=True)
    json.dump(info, open(f"{OUT}/hub_set.json", "w"), indent=1)
