"""Write nox/scout/v3/scout.assembly.json from the Scout v3 system model (24 Oct 2026; nox/ASSEMBLY.md).

Geometry and numbers come from layout_v2.py (its definitions are executed, not retyped); the cage is the system pass's
chosen members in runs/scout_layout/v3/result.json (v3: carbon rods + printed short members, rod reduction). Part meshes
the file points at are written to runs/scout_layout/v3/assembly/.

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

OUT = os.path.join(ROOT, "runs", "scout_layout", "v3", "assembly"); os.makedirs(OUT, exist_ok=True)
R = json.load(open(os.path.join(ROOT, "runs", "scout_layout", "v3", "result.json")))
rel = lambda p: os.path.relpath(p, ROOT)
for p_, q_ in R.get("webs", []): L["WEB"].add(L["_mkey"](np.array(p_), np.array(q_)))   # printed webs (short, by a keep-out)
for p_, q_ in R.get("short_carbon_clearance", []): L["FORCED_CARBON"].add(L["_mkey"](np.array(p_), np.array(q_)))   # short carbon kept for clearance


def mesh_part(name, m, **kw):
    path = os.path.join(OUT, name.replace(" ", "_").replace("/", "_") + ".stl"); m.export(path)
    return dict(name=name, mesh=rel(path), **kw)


SRC_REL = "nox/scout/v3/layout_v2.py"
own = dict(owner="scout v3", version=R.get("version", "v3"), source=SRC_REL)
parts = []
parts.append(mesh_part("frame plate", L["PLATE"], role="structure", mass_g=28, tag="E", note="Bee35 cut down, 3.5 carbon (C), tail XT60 cut-out", **own))
for i, (b, (x, y)) in enumerate(zip(L["BELLS"], L["MOTORS_XY"])):
    parts.append(dict(name=f"motor {i + 1}", role="bought", protect=True, cylinder=dict(r=L["BELL_R"], p0=[x, y, L["PLATE_TOP"]], p1=[x, y, L["PLATE_TOP"] + L["BELL_H"]]),
                      mass_g=18, tag="E", **own))
    parts.append(dict(name=f"prop {i + 1}", role="bought", protect=True, cylinder=dict(r=L["PROP_R"], p0=[x, y, L["PROP_Z"] - L["SWEEP"]], p1=[x, y, L["PROP_Z"] + L["SWEEP"]]),
                      mass_g=1.8, tag="P", note="swept disk +-4", **own))
parts.append(mesh_part("FC", L["FC"], role="bought", protect=True, mass_g=10, tag="P", **own))
Lm = L["_L"]                                                  # battery latch standard meshes (control session, 258becf), drone frame
latch_parts = [("receiver", "receiver v1 (latch)", "protected", 41.2, False, "M"), ("receiver_lid", "receiver lid (latch)", "protected", 5.3, False, "M"),
               ("receiver_strips", "receiver TPU strips (latch)", "structure", 2.0, False, "M"), ("xt60e1m", "XT60E1-M", "bought", 6.0, False, "E"),
               ("cartridge", "cartridge v5 (latch)", "protected", 35.5, True, "M"), ("slider_px", "slider +y (latch)", "protected", 1.95, True, "M"),
               ("slider_nx", "slider -y (latch)", "protected", 1.95, True, "M"), ("pack", "pack 4S 1500", "protected", 155.0, True, "P"),
               ("xt60_plug", "pack XT60 plug", "bought", 10.0, True, "E"),
               ("cartridge_skid_closed", "cartridge skid closed end (latch, TPU)", "cage", 1.4, True, "M"),
               ("cartridge_skid_lead", "cartridge skid lead end (latch, TPU)", "cage", 3.1, True, "M")]
for key, nm, role, g, swap, t in latch_parts:
    d = dict(role=role, protect=(role != "cage"), mass_g=g, tag=t, note="battery latch standard, part/battery-latch 258becf (control session)", **own)
    if swap: d["swap"] = True
    parts.append(mesh_part(nm, Lm[key], **d))
parts.append(mesh_part("mount + nose legs (Scout)", L["union"]([L["MOUNT"]] + L["LEGS"]), role="structure", mass_g=25, tag="E",
                       note="the Scout's receiver mount: bosses z 9.6 to the plate, Radxa plate, flow seat, suspension joints; envelope", **own))
for nm, key, g, t in (("Radxa Zero 3W", "RADXA", 10, "E"), ("flow MTF-02P", "FLOW", 1.5, "P"), ("UBEC", "UBEC", 15, "E"), ("XR2", "XR2", 1, "P"),
                      ("camera", "CAM", 10, "E")):
    parts.append(mesh_part(nm, L[key], role="bought", protect=True, mass_g=g, tag=t, **own))
if R.get("pads"):
    pad, carrier, body, tips = L["pads"](R["pads"]["y"], R["pads"]["z"])
    parts.append(mesh_part("pogo pads + carrier", L["union"](pad + [carrier]), role="structure", protect=True, mass_g=6, tag="E", note="belly post pads", **own))
rods = [L["member_mesh"](np.array(a), np.array(b)) for a, b in R["rods"]]   # carbon rods and printed struts (short members)
ends = {tuple(np.round(p, 2)) for ab in R["rods"] for p in ab}
def _at_core(p):                                              # core joints are the mount's / plate's hubs, not printed cage nodes
    for m_ in [L["MOUNT"], L["PLATE"]] + L["LEGS"]:
        lo, hi = m_.bounds
        if np.all(np.array(p) >= lo - 2.5) and np.all(np.array(p) <= hi + 2.5): return True
    return False
nodes = [trimesh.creation.icosphere(1, 4.0).apply_translation(p) for p in ends if not _at_core(p)]
cage = trimesh.util.concatenate(rods + nodes)
parts.append(mesh_part("cage (rods + nodes)", cage, role="cage", mass_g=round(R["cage_mass"], 1), tag="E", material="carbon rod",
                       note=f"{R.get('carbon_members', len(R['rods']))} x 3.0 carbon rods (>= {R.get('l_carbon', 0):.0f} mm) + {R.get('printed_struts', 0)} printed 6 x 6 PETG struts, on a preloaded TPU suspension; F frame model", **own))
WP = L["wheel_pts"]()
parts.append(mesh_part("catch wheels", trimesh.Trimesh(WP).convex_hull if False else trimesh.util.concatenate(
    [trimesh.creation.icosphere(1, 5.0).apply_translation(c) for c in WP.reshape(-1, 72, 3).mean(1)]), role="cage", mass_g=1.6, tag="M", **own))
parts.append(mesh_part("keep-alive pads", trimesh.util.concatenate(L["KA_PADS"]), role="cage", mass_g=2.0, tag="E",
                       note="B+ on the +x lower ring, B- on -x; the box catch's face probes touch them (G (8))", **own))
a = {"name": "Scout v3", "machine": "scout",
     "note": ("Scout v3 system pass v3: printed short members (carbon >= 33 mm, Spark), fewest pieces (layout.md). Seat frame = drone frame. Battery set = the control session's "
              "latch standard (part/battery-latch 258becf, interim). No belly post: keep-alive by probes in the +-x funnel faces onto pads on the lower ring (box catch DEFAULT, G (8)). "
              "Left out: wires (20 g E), camera mount, plate hubs, the TPU suspension pads, wheel contact pads."),
     "cg_ref": [0.0, 0.0, L["PROP_Z"]], "thrust_axis": [0, 0, 1], "cg_tol_mm": 3.0, "cage_margin_mm": 3.0,
     "openings": [{"part": n, "axis": [0, 0, -1], "half_angle_deg": L["SWAP_CONE"], "min_mm": 0.0, "skip_probes": True}   # the swap path:
                  # the box drops the cartridge out through the keel; recessed (margin >= 0); edges inside 35 deg of straight up
                  # may reach the cartridge's chamfered ends (no keel rail fits under them inside the funnel planes); it takes
                  # landings (J) (DEFAULT, Scout refit; alt: a cartridge end skid, control session)
                  for n in ("cartridge v5 (latch)", "pack 4S 1500", "slider +y (latch)", "slider -y (latch)", "pack XT60 plug",
                            "receiver v1 (latch)")],
     "contact_pairs": [[f"motor {i}", f"prop {i}", 1e9] for i in range(1, 5)] + [[f"motor {i}", "frame plate", 1e9] for i in range(1, 5)] +
                      [["cartridge v5 (latch)", "receiver TPU strips (latch)", 5.0], ["pack 4S 1500", "cartridge v5 (latch)", 1.0],
                       ["pack XT60 plug", "cartridge v5 (latch)", 5.0], ["XT60E1-M", "receiver v1 (latch)", 5.0], ["XT60E1-M", "receiver lid (latch)", 5.0],
                       ["receiver TPU strips (latch)", "receiver v1 (latch)", 5.0], ["pack XT60 plug", "XT60E1-M", 1.0],
                       ["mount + nose legs (Scout)", "receiver v1 (latch)", 1e9], ["mount + nose legs (Scout)", "receiver lid (latch)", 1e9],
                       ["mount + nose legs (Scout)", "frame plate", 1e9], ["mount + nose legs (Scout)", "XT60E1-M", 1e9], ["mount + nose legs (Scout)", "FC", 1e9], ["mount + nose legs (Scout)", "UBEC", 1e9],
                       ["cage (rods + nodes)", "frame plate", 1e9], ["cage (rods + nodes)", "mount + nose legs (Scout)", 1e9],
                       ["cage (rods + nodes)", "catch wheels", 1e9], ["cage (rods + nodes)", "keep-alive pads", 1e9], ["FC", "frame plate", 1e9],
                       ["Radxa Zero 3W", "mount + nose legs (Scout)", 1e9], ["flow MTF-02P", "mount + nose legs (Scout)", 1e9]],
     "parts": parts}
dst = os.path.join(HERE, "scout.assembly.json")
json.dump(a, open(dst, "w"), indent=1)
load(dst).save()                                              # one part per line, the assembly.py format
print("wrote", rel(dst))
