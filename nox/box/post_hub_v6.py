"""Hub v6 after the run (24 Oct 2026): print pose (sleeve end, Y 7, on the bed = the run's +y build, as hub v5), solid_check on the
exact print file, function check against the drawn bodies and keep-outs, the 6-face orientation table, the bearing bore, the
off-grid thin recheck, every gap to the v5 parts over the stroke (hub_v6.checks on the part) and a preview.
python3 post_hub_v6.py <topo run dir with box_hub_v6.stl and result.json>
-> print/box_hub_v6.stl, runs/hub/v6/{result.json, preview.png, box_hub_v6_W.stl}"""
import sys, os, json, math, subprocess
import numpy as np, trimesh
import arm_cad as A, hub_v6 as HV, print_orient as PO, thin_offset as TO
from solid_check import solid_check

RUN = sys.argv[1] if len(sys.argv) > 1 else "."
OUTR = os.path.join(HV.REPO, "runs", "hub", "v6"); os.makedirs(OUTR, exist_ok=True)
W = trimesh.load(os.path.join(RUN, "box_hub_v6.stl"), force="mesh"); W.merge_vertices()
M = A.man(W)
H = HV.hub_v6_parts()
fc_drawn = round(((H["drawn"] - H["cuts"]) - M).volume(), 2)
fc_cuts = round((M ^ H["cuts"]).volume(), 3)
W.export(os.path.join(OUTR, "box_hub_v6_W.stl"))
P = W.copy(); P.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0])); P.apply_translation(-P.bounds[0])
fn = os.path.join(HV.REPO, "print", "box_hub_v6.stl"); P.export(fn)
Pt = trimesh.load(fn, force="mesh")
ok, info = solid_check(Pt)
rows = []
for ax in range(3):
    for s in (1, -1):
        m = W.copy(); m.apply_transform(PO.rot_to_down(ax, s)); m.apply_translation(-m.bounds[0])
        n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]
        bed = float(a[(n[:, 2] < -0.99) & (zc < 0.3)].sum()); over = float(a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < 0.3))].sum())
        first, run = PO.island_check(m)
        rows.append((f"{'+' if s > 0 else '-'}{'XYZ'[ax]} down", round(bed), round(over), round(run), first[0]))
v = W.vertices; sel = (v[:, 1] > 8) & (v[:, 1] < 39) & (np.hypot(v[:, 0], v[:, 2]) < 41)
bore = round(2 * float(np.hypot(v[sel, 0], v[sel, 2]).min()), 3) if sel.any() else None
thin = {str(o): len(TO.thin_at(W, A.tm(H["drawn"] - H["cuts"]), o)) for o in ((0, 0, 0), (0.5, 0.5, 0.5), (0.25, 0.5, 0.75))}
gaps = HV.checks(dict(H, drawn=M, cuts=H["cuts"]), verbose=True)
out = dict(run=json.load(open(os.path.join(RUN, "result.json"))), g=round(Pt.volume * 1.27e-3, 1), watertight=bool(Pt.is_watertight),
           solid_ok=bool(ok), solid=info, drawn_missing_mm3=fc_drawn, in_cuts_mm3=fc_cuts, print_extents=Pt.extents.round(1).tolist(),
           bearing_bore_min_d=bore, thin_offsets=thin, faces=rows, detent=HV.DET6,
           gaps=sorted([[a, b, round(g, 2)] for (a, b), g in gaps.items()], key=lambda r: r[2])[:20], file="print/box_hub_v6.stl")
print(json.dumps({k: v for k, v in out.items() if k not in ("run", "gaps")}, indent=1, default=str))
json.dump(out, open(os.path.join(OUTR, "result.json"), "w"), indent=1, default=str)
subprocess.run([sys.executable, os.path.join(HV.REPO, "nox", "render_preview.py"), fn, "--mat", "PETG", "--prev", "214.2",
                "--out", os.path.join(OUTR, "preview.png")], check=False)
print("->", OUTR)
