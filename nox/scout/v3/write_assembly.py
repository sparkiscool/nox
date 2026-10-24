"""Write nox/scout/v3/scout.assembly.json from the Scout v3 system model (24 Oct 2026; nox/ASSEMBLY.md).

Geometry and numbers come from layout_v2.py (its definitions are executed, not retyped); the cage is the system pass's
chosen rods in runs/scout_layout/v2/result.json (pass 1, commit 1d9279a: 116 rods, E cases). Part meshes the file points
at are written to runs/scout_layout/v2/assembly/.

python3 nox/scout/v3/write_assembly.py && python3 nox/assembly.py nox/scout/v3/scout.assembly.json --check
"""
import json, os, sys
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
SRC = os.path.join(HERE, "layout_v2.py")
src = open(SRC).read(); src = src[:src.index("# ================================================================ run:")]
L = {"__file__": SRC, "__name__": "layout_v2_defs"}
exec(compile(src, SRC, "exec"), L)
from assembly import load

OUT = os.path.join(ROOT, "runs", "scout_layout", "v2", "assembly"); os.makedirs(OUT, exist_ok=True)
R = json.load(open(os.path.join(ROOT, "runs", "scout_layout", "v2", "result.json")))
rel = lambda p: os.path.relpath(p, ROOT)


def mesh_part(name, m, **kw):
    path = os.path.join(OUT, name.replace(" ", "_").replace("/", "_") + ".stl"); m.export(path)
    return dict(name=name, mesh=rel(path), **kw)


SRC_REL = "nox/scout/v3/layout_v2.py"
own = dict(owner="scout v3", version="v2 pass 1", source=SRC_REL)
parts = []
parts.append(mesh_part("frame plate", L["PLATE"], role="structure", mass_g=28, tag="E", note="Bee35 cut down, 3.5 carbon (C), tail XT60 cut-out", **own))
for i, (b, (x, y)) in enumerate(zip(L["BELLS"], L["MOTORS_XY"])):
    parts.append(dict(name=f"motor {i + 1}", role="bought", protect=True, cylinder=dict(r=L["BELL_R"], p0=[x, y, L["PLATE_TOP"]], p1=[x, y, L["PLATE_TOP"] + L["BELL_H"]]),
                      mass_g=18, tag="E", **own))
    parts.append(dict(name=f"prop {i + 1}", role="bought", protect=True, cylinder=dict(r=L["PROP_R"], p0=[x, y, L["PROP_Z"] - L["SWEEP"]], p1=[x, y, L["PROP_Z"] + L["SWEEP"]]),
                      mass_g=1.8, tag="P", note="swept disk +-4", **own))
parts.append(mesh_part("FC", L["FC"], role="bought", protect=True, mass_g=10, tag="P", **own))
parts.append(mesh_part("receiver v4 interface", L["IFACE"], role="structure", protect=True, mass_g=79.9, tag="M",
                       note="PR 44 (control session's latch standard, DEFAULT)", **own))
parts.append(mesh_part("cartridge v4", L["CART"], role="protected", protect=True, swap=True, mass_g=42.4, tag="M", note="PR 44", **own))
parts.append(mesh_part("pack 4S 1500", L["box"](-35, 35, -16, 16, -44.5 + 14.4, -44.5 + 14.4 + 33), role="protected", protect=True, swap=True, mass_g=165, tag="P", **own))
for nm, key, g, t in (("Radxa Zero 3W", "RADXA", 10, "E"), ("flow MTF-02P", "FLOW", 1.5, "P"), ("UBEC", "UBEC", 15, "E"), ("XR2", "XR2", 1, "P"),
                      ("XT60E1-M + lid", "XT_M", 8, "E"), ("camera", "CAM", 10, "E")):
    parts.append(mesh_part(nm, L[key], role="bought", protect=True, mass_g=g, tag=t, **own))
pad, carrier, body, tips = L["pads"](R["pads"]["y"], R["pads"]["z"])
parts.append(mesh_part("pogo pads + carrier", L["union"](pad + [carrier]), role="structure", protect=True, mass_g=6, tag="E", note="PR 44 position", **own))
rods = [L["rod"](np.array(a), np.array(b)) for a, b in R["rods"]]
ends = {tuple(np.round(p, 2)) for ab in R["rods"] for p in ab}
nodes = [trimesh.creation.icosphere(1, 4.0).apply_translation(p) for p in ends]
cage = trimesh.util.concatenate(rods + nodes)
parts.append(mesh_part("cage (rods + nodes)", cage, role="cage", mass_g=round(R["cage_mass"], 1), tag="E", material="carbon rod",
                       note="system pass v2 pass 1: 116 x 3.0 rods, printed nodes; F frame model", **own))
WP = L["wheel_pts"]()
parts.append(mesh_part("catch wheels", trimesh.Trimesh(WP).convex_hull if False else trimesh.util.concatenate(
    [trimesh.creation.icosphere(1, 5.0).apply_translation(c) for c in WP.reshape(-1, 72, 3).mean(1)]), role="cage", mass_g=1.6, tag="M", **own))
a = {"name": "Scout v3", "machine": "scout",
     "note": ("Scout v3 system pass v2 (pass 1 cage, E cases; layout.md). Seat frame = drone frame (PR 44). Bottom from PR 44 "
              "(control session's latch standard, DEFAULT). Left out: wires (20 g E, in no part), camera mount, plate hubs, rods' "
              "core sockets. Known FAIL: cage first at the tail-bottom (receiver needs >= 7.0 inside the funnel planes)."),
     "cg_ref": [0.0, 0.0, L["PROP_Z"]], "thrust_axis": [0, 0, 1], "cg_tol_mm": 3.0, "cage_margin_mm": 3.0,
     "openings": [{"part": "cartridge v4", "axis": [0, 0, -1], "half_angle_deg": 20.0, "min_mm": 0.0},
                  {"part": "pack 4S 1500", "axis": [0, 0, -1], "half_angle_deg": 20.0, "min_mm": 0.0}],
     "contact_pairs": [[f"motor {i}", f"prop {i}", 1e9] for i in range(1, 5)] + [[f"motor {i}", "frame plate", 1e9] for i in range(1, 5)] +
                      [["cartridge v4", "pack 4S 1500", 1e9], ["cartridge v4", "receiver v4 interface", 1e9], ["XT60E1-M + lid", "receiver v4 interface", 1e9],
                       ["XT60E1-M + lid", "cartridge v4", 1e9], ["XT60E1-M + lid", "frame plate", 1e9], ["pogo pads + carrier", "receiver v4 interface", 1e9],
                       ["cage (rods + nodes)", "frame plate", 1e9], ["cage (rods + nodes)", "receiver v4 interface", 1e9],
                       ["cage (rods + nodes)", "catch wheels", 1e9], ["FC", "frame plate", 1e9], ["Radxa Zero 3W", "receiver v4 interface", 1e9],
                       ["flow MTF-02P", "receiver v4 interface", 1e9]],
     "parts": parts}
dst = os.path.join(HERE, "scout.assembly.json")
json.dump(a, open(dst, "w"), indent=1)
load(dst).save()                                              # one part per line, the assembly.py format
print("wrote", rel(dst))
