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

RES = os.environ.get("SCOUT_RESULT") or os.path.join(ROOT, "runs", "scout_layout", "v4", "result.json")   # a candidate run's result
OUT = os.path.join(os.path.dirname(os.path.abspath(RES)), "assembly"); os.makedirs(OUT, exist_ok=True)
R = json.load(open(RES))
rel = lambda p: os.path.relpath(p, ROOT)


def mesh_part(name, m, **kw):
    path = os.path.join(OUT, name.replace(" ", "_").replace("/", "_") + ".stl"); m.export(path)
    return dict(name=name, mesh=rel(path), **kw)


SRC_REL = "nox/scout/v3/layout_v2.py"
own = dict(owner="scout v3", version=R.get("version", "v4"), source=SRC_REL)
parts = []
parts.append(mesh_part("frame plate", L["PLATE"], role="structure", mass_g=28, tag="E", note="Bee35 cut down, 3.5 carbon (C), tail XT60 cut-out", **own))
for i, (b, (x, y)) in enumerate(zip(L["BELLS"], L["MOTORS_XY"])):
    parts.append(dict(name=f"motor {i + 1}", role="bought", protect=True, cylinder=dict(r=L["BELL_R"], p0=[x, y, L["PLATE_TOP"]], p1=[x, y, L["PLATE_TOP"] + L["BELL_H"]]),
                      mass_g=18, tag="E", **own))
    parts.append(dict(name=f"prop {i + 1}", role="bought", protect=True, cylinder=dict(r=L["PROP_R"], p0=[x, y, L["PROP_Z"] - L["SWEEP"]], p1=[x, y, L["PROP_Z"] + L["SWEEP"]]),
                      mass_g=1.8, tag="P", note="swept disk +-4", **own))
parts.append(mesh_part("FC", L["FC"], role="bought", protect=True, mass_g=10, tag="P", **own))
Lm = L["_L"]                                                  # battery latch standard meshes (control session, 258becf), drone frame
V6 = L["LATCH_V"] == "v6"
RN, CN = ("receiver v2 (latch)", "cartridge v6 (latch)") if V6 else ("receiver v1 (latch)", "cartridge v5 (latch)")
latch_parts = [("receiver", RN, "protected", 85.0 if V6 else 41.2, False, "E" if V6 else "M"), ("receiver_lid", "receiver lid (latch)", "protected", 9.0 if V6 else 5.3, False, "E" if V6 else "M"),
               ("receiver_strips", "receiver TPU strips (latch)", "structure", 2.0, False, "M"), ("xt60e1m", "XT60E1-M", "bought", 6.0, False, "E"),
               ("cartridge", CN, "protected", 35.0 if V6 else 35.5, True, "E" if V6 else "M"), ("slider_px", "slider +y (latch)", "protected", 2.6 if V6 else 1.95, True, "M"),
               ("slider_nx", "slider -y (latch)", "protected", 2.6 if V6 else 1.95, True, "M"), ("pack", "pack 4S 1500", "protected", 155.0, True, "P"),
               ("xt60_plug", "pack XT60 plug", "bought", 10.0, True, "E")] + ([] if V6 else
              [("cartridge_skid_closed", "cartridge skid closed end (latch, TPU)", "cage", 1.4, True, "M"),
               ("cartridge_skid_lead", "cartridge skid lead end (latch, TPU)", "cage", 3.1, True, "M")])
if V6:                                                        # cartridge v6 end skids (latch 4d3b167)
    for k_, nm_, g_ in (("cartridge_skid_closed", "cartridge skid closed end (latch, TPU)", 1.4), ("cartridge_skid_lead", "cartridge skid lead end (latch, TPU)", 3.1)):
        parts.append(mesh_part(nm_, L["SKIDS"][0 if "closed" in k_ else 1], role="cage", protect=False, mass_g=g_, tag="E", swap=True,
                               note="cartridge v6 end skid (latch 4d3b167)", **own))
for key, nm, role, g, swap, t in latch_parts:
    d = dict(role=role, protect=(role != "cage"), mass_g=g, tag=t, note=("battery latch v6 / receiver v2, part/battery-latch-v6 b0b0321 (latch PR 109)" if V6 else "battery latch standard, part/battery-latch 258becf"), **own)
    if swap: d["swap"] = True
    parts.append(mesh_part(nm, Lm[key], **d))
parts.append(mesh_part("receiver v2 skids (latch, E)", L["REC_SKIDS"], role="cage", mass_g=3.5, tag="E", material="TPU",
                       note=("the latch's drawn receiver v2 skids (PR 109): nose plate, side bars + bottom strips" if V6 else "this pass's skid requirement (E shapes)"), **own))
parts.append(mesh_part("cartridge v6 skids (latch, E)", L["CART_SKIDS"], role="cage", mass_g=2.0, tag="E", material="TPU", swap=True,
                       note=("the latch's cartridge v6 strips (PR 109)" if V6 else "requirement: strips under the bottom long edges"), **own))
parts.append(mesh_part("mount + nose legs (Scout)", L["union"]([L["MOUNT"]] + L["LEGS"]), role="structure", mass_g=25, tag="E",
                       note="the Scout's receiver mount: bosses z 9.6 to the plate, Radxa plate, flow seat, suspension joints; envelope", **own))
for nm, key, g, t in (("Radxa Zero 3W", "RADXA", 10, "E"), ("flow MTF-02P", "FLOW", 1.5, "P"), ("UBEC", "UBEC", 15, "E"), ("XR2", "XR2", 1, "P"),
                      ("camera", "CAM", 10, "E")):
    parts.append(mesh_part(nm, L[key], role="bought", protect=True, mass_g=g, tag=t, **own))
if R.get("pads"):
    pad, carrier, body, tips = L["pads"](R["pads"]["y"], R["pads"]["z"])
    parts.append(mesh_part("pogo pads + carrier", L["union"](pad + [carrier]), role="structure", protect=True, mass_g=6, tag="E", note="belly post pads", **own))
# left-out items now in (review PR 94): wires as a mass point, camera mount, plate hubs at the core joints (E)
parts.append(dict(name="wires", role="structure", sphere=dict(r=3.0, center=[0.0, 0.0, 13.5]), mass_g=20, tag="E",
                  note="harness as one mass point at the FC (E 20 g)", **own))
parts.append(mesh_part("camera mount", L["box"](84.0, 90.0, -10.0, 10.0, 22.0, 60.0), role="structure", mass_g=6, tag="E",
                       note="envelope; the piece needs >= 100 Hz with the 10 g camera (k >= 4 N/mm, dimensions)", **own))
HUBS = [c["at"] for c in R.get("core_reactions", []) if c["at"][2] > 18.0]       # plate hubs (the mount carries the rest)
if HUBS:
    parts.append(mesh_part("plate hubs + TPU suspension", trimesh.util.concatenate([L["box"](x - 5, x + 5, y - 5, y + 5, z - 4, z + 4) for x, y, z in HUBS]),
                           role="structure", mass_g=2.0 * len(HUBS), tag="E", note="cage joints on the plate, preloaded TPU inside (E)", **own))
rods = [L["member_mesh"](np.array(a), np.array(b), t) for (a, b), t in zip(R["rods"], R["types"])]   # carbon rods and printed struts
ends = {tuple(np.round(p, 2)) for ab in R["rods"] for p in ab}
def _at_core(p):                                              # core joints are the mount's / plate's hubs, not printed cage nodes
    for m_ in [L["MOUNT"], L["PLATE"]] + L["LEGS"]:
        lo, hi = m_.bounds
        if np.all(np.array(p) >= lo - 2.5) and np.all(np.array(p) <= hi + 2.5): return True
    return False
nodes = [trimesh.creation.icosphere(1, 4.0).apply_translation(p) for p in ends if not _at_core(p)]
cage = trimesh.util.concatenate(rods + nodes)
parts.append(mesh_part("cage (rods + nodes)", cage, role="cage", mass_g=round(R["cage_mass"], 1), tag="E", material="carbon rod",
                       note=f"{R['pieces']} x 3.0 carbon rods (shortest {R['shortest_carbon']} mm, >= {R['l_carbon']:.0f}) + {R['printed_struts']} printed 6 x 6 PETG struts in {R['printed_nodes']} printed parts, on a preloaded TPU suspension; F frame model", **own))
WP = L["wheel_pts"]()
parts.append(mesh_part("catch wheels", trimesh.Trimesh(WP).convex_hull if False else trimesh.util.concatenate(
    [trimesh.creation.icosphere(1, 5.0).apply_translation(c) for c in WP.reshape(-1, 72, 3).mean(1)]), role="cage", mass_g=1.6, tag="M", **own))
parts.append(mesh_part("keep-alive pads", trimesh.util.concatenate(L["KA_PADS"]), role="cage", mass_g=2.0, tag="E",
                       note="B+ on the +x lower ring, B- on -x; the box catch's face probes touch them (G (8))", **own))
SWAPSET = [CN, "cartridge v6 skids (latch, E)", "pack 4S 1500", "slider +y (latch)", "slider -y (latch)", "pack XT60 plug", "cartridge skid closed end (latch, TPU)", "cartridge skid lead end (latch, TPU)", "receiver TPU strips (latch)"]
a = {"name": "Scout v3", "machine": "scout",
     "note": ("Scout v3 system pass v4: layout change after review PR 94 (Radxa up 8, shields, access), carbon rods >= 33 mm (layout.md). Seat frame = drone frame. Battery set = the control session's "
              "latch standard (part/battery-latch 258becf, interim). No belly post: keep-alive by probes in the +-x funnel faces onto pads on the lower ring (box catch DEFAULT, G (8)). "
              "Wires, camera mount and plate hubs are E envelopes / mass points. Left out: wheel contact pads."),
     "cg_ref": [0.0, 0.0, L["PROP_Z"]], "thrust_axis": [0, 0, 1], "cg_tol_mm": 3.0, "cage_margin_mm": 3.0,
     "openings": [{"part": n, "axis": [0, 0, -1], "half_angle_deg": L["SWAP_CONE"], "min_mm": 0.0, "skip_probes": True}   # the swap path:
                  # the box drops the cartridge out through the keel; recessed (margin >= 0); edges inside 35 deg of straight up
                  # may reach the cartridge's chamfered ends (no keel rail fits under them inside the funnel planes); it takes
                  # landings (J) (DEFAULT, Scout refit; alt: a cartridge end skid, control session)
                  for n in (CN, "pack 4S 1500", "slider +y (latch)", "slider -y (latch)", "pack XT60 plug",
                            RN, "receiver lid (latch)")],
     "mass_budget_g": 500,
     "contact_pairs": [["receiver v2 skids (latch, E)", n_, 1e9] for n_ in (RN, "receiver lid (latch)", "receiver TPU strips (latch)", "mount + nose legs (Scout)", "cartridge v6 skids (latch, E)")] +
                      [["cartridge v6 skids (latch, E)", n_, 1e9] for n_ in (CN, "slider +y (latch)", "slider -y (latch)", "pack 4S 1500")] + [[f"motor {i}", f"prop {i}", 1e9] for i in range(1, 5)] + [[f"motor {i}", "frame plate", 1e9] for i in range(1, 5)] +
                      [[CN, "receiver TPU strips (latch)", 5.0], ["pack 4S 1500", CN, 1.0],
                       ["pack XT60 plug", CN, 5.0], ["XT60E1-M", RN, 5.0], ["XT60E1-M", "receiver lid (latch)", 5.0],
                       ["receiver TPU strips (latch)", RN, 5.0], ["pack XT60 plug", "XT60E1-M", 1.0],
                       ["mount + nose legs (Scout)", RN, 1e9], ["mount + nose legs (Scout)", "receiver lid (latch)", 1e9],
                       ["mount + nose legs (Scout)", "frame plate", 1e9], ["mount + nose legs (Scout)", "XT60E1-M", 1e9], ["mount + nose legs (Scout)", "FC", 1e9], ["mount + nose legs (Scout)", "UBEC", 1e9],
                       ["cage (rods + nodes)", "frame plate", 1e9], ["cage (rods + nodes)", "mount + nose legs (Scout)", 1e9],
                       ["cage (rods + nodes)", "catch wheels", 1e9], ["cage (rods + nodes)", "keep-alive pads", 1e9], ["FC", "frame plate", 1e9],
                       ["Radxa Zero 3W", "mount + nose legs (Scout)", 1e9], ["flow MTF-02P", "mount + nose legs (Scout)", 1e9],
                       ["wires", "FC", 1e9], ["wires", "mount + nose legs (Scout)", 1e9], ["camera mount", "camera", 1e9], ["camera mount", "frame plate", 1e9],
                       ["camera mount", "cage (rods + nodes)", 1e9], ["plate hubs + TPU suspension", "frame plate", 1e9],
                       ["plate hubs + TPU suspension", "cage (rods + nodes)", 1e9], ["plate hubs + TPU suspension", "mount + nose legs (Scout)", 1e9]],
     # access (review PR 94; docs/generative.md 0): the swap drops the set out through the keel; USB and the receiver bolts
     # are reached with the pack out; the mount envelope is a solid box here, so its channels are requirements (dimensions);
     # the Radxa is reached over WiFi (SSH), no USB path (DEFAULT; alt: a 12 x 7 path down from its bottom edge)
     "access": [{"name": f"swap: {n} down", "part": n, "dir": [0, 0, -1], "length_mm": 90, "tool": "part", "ignore": SWAPSET}
                for n in (CN, "pack 4S 1500", "pack XT60 plug")] +
               [{"name": f"M3 key, receiver bolt ({x:+.1f}, {y:+.1f})", "part": RN, "from": [x, y, 9.6], "dir": [0, 0, -1],
                 "length_mm": 90, "tool": {"type": "cylinder", "d": 4}, "ignore": SWAPSET + ["receiver lid (latch)", "mount + nose legs (Scout)", "XT60E1-M"]}
                for x in L["BOLTS_X"] for y in (-L["BOLTS_Y"], L["BOLTS_Y"])] +
               [{"name": "FC USB-C (edge to the tail-left, E)", "part": "FC", "from": [-10.8, -12.7, 13.5], "dir": [-0.7071, -0.7071, 0], "length_mm": 70,
                 "tool": {"type": "box", "w": 12, "h": 7, "w_axis": [0.7071, -0.7071, 0]}, "ignore": ["mount + nose legs (Scout)", "wires"]}],

     "parts": parts}
BUDGET = {"cage (rods + nodes)": 300, "mount + nose legs (Scout)": 20, "camera mount": 5, "plate hubs + TPU suspension": 8,
          "catch wheels": 1.6, "keep-alive pads": 2.0}          # soft targets (dimensions L Mass budget; the 500 g whole leaves 62 g after
for p_ in a["parts"]:                                           # the 438 g fixed: the full cage is over it, reported, Spark 24 Oct)
    if p_["name"] in BUDGET: p_["budget_g"] = BUDGET[p_["name"]]
dst = os.environ.get("SCOUT_MANIFEST") or os.path.join(HERE, "scout.assembly.json")
json.dump(a, open(dst, "w"), indent=1)
load(dst).save()                                              # one part per line, the assembly.py format
print("wrote", rel(dst))
