"""Heads v6 after the run (24 Oct 2026): head A from the run (out/box_head_A_v6.stl, head frame), head B = head A mirrored in X;
print pose = the rack web's bottom face (Y 64) on the bed (rotate +90 about X, as the v5 heads' glide legs); solid_check on the
exact print files, the print-orientation table, the 3D gaps to the v5 parts over every stroke state (head_v6.checks), and the
as-built strength rows of the run. Writes print/box_head_{A,B}_v6.stl, runs/head/v6/{result.json, preview*.png} and
runs/head/v6/box_head_{A,B}_v6_W.stl (head frame).
python3 post_head_v6.py <topo run dir with head_A_v6.stl and result.json>"""
import sys, os, json, math, subprocess
import numpy as np, trimesh
import head_v6 as HD, print_orient as PO
from arm_cad import man, tm, mirror_x
from solid_check import solid_check

RUN = sys.argv[1] if len(sys.argv) > 1 else "."
REPO = HD.HV.REPO
OUTR = os.path.join(REPO, "runs", "head", "v6"); os.makedirs(OUTR, exist_ok=True)
PR = os.path.join(REPO, "print")

def pose(W):
    P = W.copy(); P.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [1, 0, 0])); P.apply_translation(-P.bounds[0])
    return P

def faces(W):
    rows = []
    for ax in range(3):
        for s in (1, -1):
            m = W.copy(); m.apply_transform(PO.rot_to_down(ax, s)); m.apply_translation(-m.bounds[0])
            n = m.face_normals; a = m.area_faces; zc = m.triangles_center[:, 2]
            bed = float(a[(n[:, 2] < -0.99) & (zc < 0.3)].sum()); over = float(a[(n[:, 2] < -0.5) & ~((n[:, 2] < -0.99) & (zc < 0.3))].sum())
            first, run = PO.island_check(m)
            rows.append((f"{'+' if s > 0 else '-'}{'XYZ'[ax]} down", round(bed), round(over), round(run), first[0]))
    return rows

res = json.load(open(os.path.join(RUN, "result.json")))
WA = trimesh.load(os.path.join(RUN, "head_A_v6.stl"), force="mesh"); WA.merge_vertices()
WB = WA.copy(); WB.apply_transform(np.diag([-1.0, 1, 1, 1]))
if WB.volume < 0: WB.invert()
out = dict(run=res, heads={})
for h, W in (("A", WA), ("B", WB)):
    W.export(os.path.join(OUTR, f"box_head_{h}_v6_W.stl"))
    P = pose(W); fn = os.path.join(PR, f"box_head_{h}_v6.stl"); P.export(fn)
    Pt = trimesh.load(fn, force="mesh")
    ok, info = solid_check(Pt)
    out["heads"][h] = dict(g=round(Pt.volume * 1.27e-3, 1), watertight=bool(Pt.is_watertight), solid_ok=bool(ok), solid=info,
                          print_extents=Pt.extents.round(1).tolist(), faces=faces(W), file=f"print/box_head_{h}_v6.stl")
    print(h, json.dumps({k: v for k, v in out["heads"][h].items() if k != "faces"}), flush=True)
    for r in out["heads"][h]["faces"]: print("   ", r)
gaps = HD.checks(man(WA))
out["gaps"] = sorted([[a, b, round(g, 2)] for (a, b), g in gaps.items()], key=lambda r: r[2])[:20]
json.dump(out, open(os.path.join(OUTR, "result.json"), "w"), indent=1)
for h in ("A", "B"):
    subprocess.run([sys.executable, os.path.join(REPO, "nox", "render_preview.py"), os.path.join(PR, f"box_head_{h}_v6.stl"), "--mat", "PETG",
                    "--prev", "56.1" if h == "A" else "55.5", "--out", os.path.join(OUTR, "preview.png" if h == "A" else "preview_head_B.png")], check=False)
for f in ("log.txt", "density.npz"):                                                  # the run's own record beside the summary
    if os.path.exists(os.path.join(RUN, f)) and os.path.abspath(RUN) != os.path.abspath(OUTR):
        import shutil; shutil.copy(os.path.join(RUN, f), os.path.join(OUTR, f))
json.dump(res, open(os.path.join(OUTR, "topo_result.json"), "w"), indent=1, default=str)
print("->", OUTR)
