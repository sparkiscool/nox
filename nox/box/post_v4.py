"""After the v4 runs: main bodies, W and print-orientation STLs, drawn small parts. python post_v4.py [heads|baymod|small|all]
17 Oct: heads use the newest version whose mesh exists (v5 = belt clamp v1, else v4)."""
import sys, os, math, json
import numpy as np, trimesh
import arm_cad as A, arm_layout as L
from arm_cad import man, tm
from manifold3d import Manifold

OUT = "out"
what = sys.argv[1] if len(sys.argv) > 1 else "all"

def main_body(src):
    m = trimesh.load(src, force="mesh", process=False); m.merge_vertices()
    bodies = sorted(man(m).decompose(), key=lambda b: -b.volume())
    print(src, "bodies", [round(b.volume(), 1) for b in bodies[:4]])
    # 17 Oct: keep enclosed cavities (negative-volume shells inside the main body); drop only loose positive specks.
    # Was bodies[0] alone, which silently filled a 15 cm3 generated cavity in head A v5.
    out = bodies[0]
    for b in bodies[1:]:
        if b.volume() < 0: out = out - man(_flip(tm(b)))        # carve the cavity back out
    return out

def _flip(t):
    t = t.copy(); t.invert(); return t

def src_of(stem):
    """the run writes W coords to <stem>.stl; post writes <stem>_W.stl (W) and overwrites <stem>.stl (print pose).
    Read <stem>.stl only if it is newer than <stem>_W.stl (a fresh run), else <stem>_W.stl (17 Oct: a rerun read a
    print-posed file as W)."""
    raw, w = f"{OUT}/{stem}.stl", f"{OUT}/{stem}_W.stl"
    if os.path.exists(raw) and (not os.path.exists(w) or os.path.getmtime(raw) > os.path.getmtime(w) + 1): return raw
    return w if os.path.exists(w) else raw

def head_ver(v):
    for ver in ("v5", "v4"):
        if os.path.exists(f"{OUT}/box_head_{v}_{ver}.stl") or os.path.exists(f"{OUT}/box_head_{v}_{ver}_W.stl"): return ver
    return "v5"

def orient(mesh, axis, deg):
    o = mesh.copy(); o.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis)); o.apply_translation(-o.bounds[0]); return o

info = {}
if what in ("heads", "all"):
    for v in ("A", "B"):
        ver = head_ver(v)
        src = src_of(f"box_head_{v}_{ver}")
        if not os.path.exists(src): continue
        b = main_body(src); W = tm(b); W.export(f"{OUT}/box_head_{v}_{ver}_W.stl")
        P = orient(W, [1, 0, 0], 90); P.export(f"{OUT}/box_head_{v}_{ver}.stl")         # glide legs (Y 74) down
        info[f"head_{v}"] = dict(ver=ver, g=round(b.volume() * 1.27e-3, 1), bodies=len(b.decompose()), print=P.extents.round(1).tolist())
        F = A.fingers_and_racks(v)
        for k in ("A", "B"):
            f = tm(F[k]); f.export(f"{OUT}/box_finger_{k}_for_head_{v}_v4_W.stl")
            orient(f, [0, 1, 0], 90 if (k == "A") == (v == "A") else -90).export(f"{OUT}/box_finger_{k}_for_head_{v}_v4.stl")   # blade side down
if what in ("baymod", "all"):
    src = src_of("box_baymod_v4")
    if os.path.exists(src):
        b = main_body(src)
        B = A.bay_module_parts()
        b = sorted((b - B["cuts"]).decompose(), key=lambda q: -q.volume())[0]   # smoothing can push a sliver into the dovetail groove (17 Oct: 27 / 70 mm3)
        # v4i (17 Oct 09:30, Spark: spikes): two dead-end horns ran 4 mm past the near end (Y 141) of the drawn blocks at
        # X +-22, Z 119, toward the bed. Trim generated material flush with that end; drawn parts are never cut.
        horn = Manifold.cube((12.0, 140.9 - 128.0, 15.0)).translate((19.0, 128.0, 112.0))
        horn = (horn + horn.mirror((1, 0, 0))) - B["drawn"]
        v0 = b.volume(); b = sorted((b - horn).decompose(), key=lambda q: -q.volume())[0]
        print("horn trim mm3", round(v0 - b.volume(), 1))
        W = tm(b); W.export(f"{OUT}/box_baymod_v4_W.stl")
        P = orient(W, [1, 0, 0], 90); P.export(f"{OUT}/box_baymod_v4.stl")           # near end (Y 133) down = the run's build direction; tree supports ~21 g (E)
        info["baymod"] = dict(g=round(b.volume() * 1.27e-3, 1), bodies=len(b.decompose()), print=P.extents.round(1).tolist())
if what in ("small", "all"):
    for nm, m, ax, deg in (("box_bay_strip_v1", A.bay_strip(), [1, 0, 0], 0),
                           ("box_pinion_v4", A.pinion_u(), [1, 0, 0], 0)):
        t = orient(tm(m), ax, deg); t.export(f"{OUT}/{nm}.stl")
        info[nm] = dict(g=round(m.volume() * 1.27e-3, 2), print=t.extents.round(1).tolist())
allinfo = json.load(open(f"{OUT}/post_v4.json")) if os.path.exists(f"{OUT}/post_v4.json") else {}
allinfo.update(info)                                                              # keep the other parts' rows
print(json.dumps(info, indent=1)); json.dump(allinfo, open(f"{OUT}/post_v4.json", "w"), indent=1)
