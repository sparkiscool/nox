"""Print-ready meshes: each part turned to its bed face (cage_layout.json info.build), z min 0, centred on the plate.
Mirrored parts (the -y side hub, 3 of the 4 feet) are written as their own files so the slicer gets exactly what goes
on the drone. Run after cage_cad.py: python3 export_print.py <out_dir>"""
import json, os, sys, numpy as np, trimesh
import cage_cad as cc
from print_check import to_bed
OUT = cc.OUT; D = sys.argv[1] if len(sys.argv) > 1 else f"{OUT}/print"; os.makedirs(D, exist_ok=True)
info = json.load(open(f"{OUT}/cage_layout.json"))["info"]
def put(m, build, name):
    m2, _ = to_bed(m, build); b = m2.bounds; m2.apply_translation([-(b[0, 0] + b[1, 0]) / 2, -(b[0, 1] + b[1, 1]) / 2, 0])
    m2.export(f"{D}/{name}.stl"); return m2
rows = {}
for part in ("corner_node", "tee_node", "tee_node_tail", "side_hub", "tail_hub", "foot_node", "wheel"):
    m = trimesh.load(f"{OUT}/s2_cage_v2_{part}.stl"); b = info[part]["build"]
    rows[part] = put(m, b, f"s2_cage_v2_{part}")
    if part == "side_hub":
        mm = m.copy(); mm.apply_transform(np.diag([1, -1, 1, 1.0]))          # trimesh fixes the winding of a mirror itself
        rows["side_hub_R"] = put(mm, b, "s2_cage_v2_side_hub_R")     # -y hub; mirror keeps the same bed face (x)
for k, m in rows.items(): print(k, [round(float(x), 1) for x in m.extents], round(abs(m.volume), 1))

# mirrored foot for the (+x,-y) and (-x,+y) corners; (-x,-y) is the (+x,+y) foot turned 180 deg
fm = trimesh.load(f"{OUT}/s2_cage_v2_foot_node.stl"); fm.apply_transform(np.diag([1, -1, 1, 1.0]))
rows["foot_node_M"] = put(fm, info["foot_node"]["build"], "s2_cage_v2_foot_node_M")
# plates: one PETG set and one TPU set per drone, shelf-packed on the A1 mini bed (180, 8 mm margin, 6 mm gaps)
SETS = {"PETG": [("corner_node", 4), ("side_hub", 1), ("side_hub_R", 1), ("tail_hub", 1), ("wheel", 8)],
        "TPU": [("tee_node", 2), ("tee_node_tail", 1), ("foot_node", 2), ("foot_node_M", 2)]}
for mat, items in SETS.items():
    sc = trimesh.Scene(); x = y = 8.0; row_h = 0.0; n = 0
    for name, cnt in sorted(items, key=lambda it: -rows[it[0]].extents[1]):
        for i in range(cnt):
            m = rows[name].copy(); e = m.extents
            if x + e[0] > 172: x = 8.0; y += row_h + 6.0; row_h = 0.0
            b = m.bounds; m.apply_translation([x - b[0, 0], y - b[0, 1], 0]); sc.add_geometry(m, node_name=f"{name}_{i + 1}")
            x += e[0] + 6.0; row_h = max(row_h, e[1]); n += 1
    assert y + row_h <= 172, mat
    lo, hi = sc.bounds; sc.apply_translation([90 - (lo[0] + hi[0]) / 2, 90 - (lo[1] + hi[1]) / 2, 0])   # centred on the bed
    sc.export(f"{D}/s2_cage_v2_{mat}_plate.3mf"); print(mat, "plate:", n, "parts, used", round(y + row_h, 1), "mm of 180")
