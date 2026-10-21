"""After the v3 runs: main bodies, W and print-orientation STLs, ring segments. python post_v3.py [hub|heads|ring|all]"""
import sys, os, math, json
import numpy as np, trimesh
import arm_cad as A, arm_layout as L
from arm_cad import man, tm

OUT = "out"
what = sys.argv[1] if len(sys.argv) > 1 else "all"

def main_body(src):
    m = trimesh.load(src, force="mesh", process=False); m.merge_vertices()
    bodies = sorted(man(m).decompose(), key=lambda b: -b.volume())
    print(src, "bodies", [round(b.volume(), 1) for b in bodies[:4]])
    return bodies[0]

def orient(mesh, axis, deg):
    o = mesh.copy(); o.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis)); o.apply_translation(-o.bounds[0]); return o

if what in ("hub", "all") and os.path.exists(f"{OUT}/box_hub_v3.stl"):
    b = main_body(f"{OUT}/box_hub_v3.stl"); W = tm(b); W.export(f"{OUT}/box_hub_v3_W.stl")
    P = orient(W, [1, 0, 0], 90); P.export(f"{OUT}/box_hub_v3.stl")            # sleeve end down, plate up
    print("hub g", round(b.volume() * 1.27e-3, 1), "print", P.extents.round(1).tolist())
if what in ("heads", "all"):
    for v in ("A", "B"):
        src = f"{OUT}/box_head_{v}_v3_W.stl" if not os.path.exists(f"{OUT}/box_head_{v}_v3.stl") else f"{OUT}/box_head_{v}_v3.stl"
        if not os.path.exists(src): continue
        b = main_body(src); W = tm(b); W.export(f"{OUT}/box_head_{v}_v3_W.stl")
        P = orient(W, [1, 0, 0], -90); P.export(f"{OUT}/box_head_{v}_v3.stl")  # plate end (Y 164) down
        print(v, "head g", round(b.volume() * 1.27e-3, 1), "print", P.extents.round(1).tolist())
    for v in ("A", "B"):                                                           # the matching fingers (A mirrored)
        F = A.fingers_and_racks(v)
        for k in ("A", "B"):
            tm(F[k]).export(f"{OUT}/box_finger_{k}_for_head_{v}_v3.stl")
if what in ("ring", "all"):
    src = f"{OUT}/box_bay_ring_v3.stl" if os.path.exists(f"{OUT}/box_bay_ring_v3.stl") else f"{OUT}/box_bay_ring_v3_W.stl"
    if os.path.exists(src):
        b = main_body(src); W = tm(b); W.export(f"{OUT}/box_bay_ring_v3_W.stl")
        info = []
        for k, (a, sg) in enumerate(zip(L.BAY_ANGLES, [max(x.decompose(), key=lambda q: q.volume()) for x in A.ring_segments(b)])):
            S = tm(sg)
            # print: far-wall face down. Turn the segment to the top position first (its bay radial = +Z), then Y 174 to the bed
            S.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-(90.0 - a)), [0, 1, 0]))
            S = orient(S, [1, 0, 0], -90)
            S.export(f"{OUT}/box_bay_seg{k}_v3.stl")
            info.append(dict(seg=k, angle=a, g=round(sg.volume() * 1.27e-3, 1), bodies=len(sg.decompose()), extents=S.extents.round(1).tolist()))
            print(info[-1])
        json.dump(info, open(f"{OUT}/ring_segments_v3.json", "w"), indent=1)
        print("ring g", round(b.volume() * 1.27e-3, 1))
