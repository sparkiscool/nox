"""Overseer front arm v6: modal what-if of the diagonals on a built arm (27 Oct). Takes a built front arm (the v5d result),
adds the v6 diagonals as carbon inserts (their exact setup geometry, per-rod E A) and reads the first modes on the 4
inserts (h 1.5, as the v5d modes). Two cases: rods only (no new PETG) and rods + the drawn v6 sleeves, node blocks and
shoulder wedge (NOX_ARM_* overrides any v6 setting, e.g. NOX_ARM_SHWEB=tri10).

python3 nox/overseer/arm_diag_whatif_v6.py MESH [--h 1.5] [--out runs/overseer_arm_front/v6]
"""
import argparse, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for k, v in (("NOX_ARM_VERSION", "v6"), ("NOX_ARM_CAP", "1"), ("NOX_ARM_SLEEVE", "1"), ("NOX_ARM_SHFLANGE", "1"),
             ("NOX_ARM_BRACES", "0"), ("NOX_ARM_LINKS", "1"), ("NOX_ARM_ROOT_U1", "84"), ("NOX_ARM_ROOT_SOLID", "1"),
             ("NOX_ARM_FLANGE_X", "40"), ("NOX_ARM_CHORD", "tube6"), ("NOX_ARM_ROD_Z", "38.5,6.5"),
             ("NOX_ARM_JOINT", os.path.join(ROOT, "runs", "overseer_arms", "v5", "arm_loads.json")),
             ("NOX_ARM_LAYOUT", os.path.join(ROOT, "runs", "overseer_layout", "v5")),
             ("NOX_ARM_CORE", os.path.join(ROOT, "runs", "overseer_core", "v1", "parts"))):
    os.environ.setdefault(k, v)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "nox"))
import arm_topo_v6  # noqa: F401  (the v6 settings, env defaults)
import arm_topo_v1 as A

ap = argparse.ArgumentParser()
ap.add_argument("mesh")
ap.add_argument("--h", type=float, default=1.5)
ap.add_argument("--out", default=os.path.join(ROOT, "runs", "overseer_arm_front", "v6"))
ap.add_argument("--support", default="inserts")
ap.add_argument("--nodes-only", action="store_true", help="skip the rods-only case")
a = ap.parse_args()
B, F = A.bodies("front")
res = {"mesh": os.path.relpath(os.path.abspath(a.mesh), ROOT), "h": a.h, "support": a.support, "pattern": A.DIAG_PAT,
       "stations": A.DIAG_T, "offset_mm": A.DIAG_OFF, "gap_mm": A.DIAG_GAP, "kind": A.DIAG_KIND,
       "shoulder_web": os.environ.get("NOX_ARM_SHWEB"), "shoulder_web_cm3": round(B["shweb"].volume() / 1000, 2),
       "diag_drawn_cm3": round((B.get("diag_drawn", A.mf.Manifold()) - B["cuts"]).volume() / 1000, 2)}
res["rods_only"] = None if a.nodes_only else A.modes("front", a.mesh, h=a.h, support=a.support)
res["rods_and_nodes"] = A.modes("front", a.mesh, h=a.h, support=a.support, extra=B.get("diag_drawn", A.mf.Manifold()) + B["shweb"] - B["cuts"])
os.makedirs(a.out, exist_ok=True)
json.dump(res, open(os.path.join(a.out, "whatif_modes.json"), "w"), indent=1)
print(json.dumps({k: [m["Hz"] for m in res[k]["modes"]] for k in ("rods_only", "rods_and_nodes") if res[k]}))
