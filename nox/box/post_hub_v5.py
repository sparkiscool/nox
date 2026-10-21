"""Hub v5 after the run (18 Oct 2026): one body check, print pose (sleeve end, Y 7, on the bed = the run's +y build), the
README orientation check on all 6 faces, the off-grid thin recheck and a preview. The Ø80.2 bearing bore is drawn exact in
hub_parts() since 18 Oct (256-gon, BRG_OD/2 + 0.10), so no post-cut is needed any more.
python3 post_hub_v5.py -> out/box_hub_v5_W.stl (wheel coords), out/box_hub_v5.stl (print pose), out/hub_v5_post.json"""
import json, math
import numpy as np, trimesh
import arm_cad as A, print_orient as PO, thin_offset as TO

W = trimesh.load("out/box_hub_v5.stl", force="mesh"); W.merge_vertices()
M = A.man(W); bodies = M.decompose()
H = A.hub_parts()
fc_drawn = round(((H["drawn"] - H["cuts"]) - M).volume(), 2)                 # drawn material missing from the part
fc_cuts = round((M ^ H["cuts"]).volume(), 3)                                  # part material inside a keep-out
W.export("out/box_hub_v5_W.stl")
P = W.copy(); P.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0])); P.apply_translation(-P.bounds[0])
P.export("out/box_hub_v5.stl")                                               # sleeve end (low Y) down
rows = []
for ax in range(3):
    for s in (1, -1):
        m = W.copy(); m.apply_transform(PO.rot_to_down(ax, s)); m.apply_translation(-m.bounds[0])
        n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]
        bed = float(a[(n[:, 2] < -0.99) & (zc < 0.3)].sum()); over = float(a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < 0.3))].sum())
        first, run = PO.island_check(m)
        rows.append((f"{'+' if s > 0 else '-'}{'XYZ'[ax]} down", round(bed), round(over), round(run), first[0]))
# bore check: radius of the part's surface over the bearing seat
v = W.vertices; sel = (v[:, 1] > 8) & (v[:, 1] < 39) & (np.hypot(v[:, 0], v[:, 2]) < 41)
bore_min = round(2 * float(np.hypot(v[sel, 0], v[sel, 2]).min()), 3) if sel.any() else None
thin = {str(o): len(TO.thin_at(W, A.tm(H["drawn"] - H["cuts"]), o)) for o in ((0, 0, 0), (0.5, 0.5, 0.5), (0.25, 0.5, 0.75), (0.75, 0.25, 0.5))}
out = dict(g=round(W.volume / 1000 * 1.27, 1), bodies=len(bodies), watertight=bool(W.is_watertight), drawn_missing_mm3=fc_drawn,
           in_cuts_mm3=fc_cuts, print_extents=P.extents.round(1).tolist(), bearing_bore_min_d=bore_min, thin_offsets=thin, faces=rows)
print(json.dumps(out, indent=1)); json.dump(out, open("out/hub_v5_post.json", "w"), indent=1)
