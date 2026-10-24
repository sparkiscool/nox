"""Collect the round 2 sweep into runs/box_system/v2/topo_r2/: result.json (chosen run + sweep + ring comparison; every connectivity
sentence is read from the runs' connectivity.json, none typed by hand), analysis.txt, copies of the chosen run's files.
python3 box_system_topo_v2r2_collect.py [chosen=g3000]"""
import os, sys, json, shutil
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
D = os.path.join(ROOT, "runs", "box_system", "v2", "topo_r2")
CH = sys.argv[1] if len(sys.argv) > 1 else "g3000"
RUNS = ["g2000", "g3000", "g3000_ring", "g4000"]
STUB = "stub tilt: XT60 at the drone 57 N + 5.9 N.m"
sweep = {}
for t in RUNS:
    r = json.load(open(os.path.join(D, t, "result.json"))); c = json.load(open(os.path.join(D, t, "connectivity.json")))["summary"]
    br = r["bay_rotation"]
    rel = {k: max(abs(v["rel_to_stub_Y_mrad"]) for v in br[k].values()) for k in br if k.startswith("all bays")}
    sweep[t] = dict(gen_cm3=r["gen_cm3"], generated_cm3_M=round(r["generated_cm3"], 1), generated_mass_g_E=round(r["generated_cm3"] * 1.27),
                    drawn_cm3_M=round(r["drawn_cm3"], 1), total_mass_g_E=round(r["mass_g_PETG"]), ring=r["ring"],
                    connectivity=c["text"], bays_joined=f"{c['bays_joined']}/{c['bays_total']}", posts_joined=f"{c['posts_joined']}/{c['posts_total']}",
                    floor_contact_cm2_M=c["floor_contact_cm2_M"], side_x_contact_cm2_M=c["side_x_contact_cm2_M"],
                    stub_tilt_mrad_C=round(max(abs(x) for x in r["stub_rotation"][STUB]["rot_mrad"]), 3),
                    ratio_to_full_solid_M={k: round(v, 1) for k, v in r["ratio_to_full_solid"].items()},
                    worst_ratio_M=round(max(r["ratio_to_full_solid"].values()), 1),
                    bay_rot_about_axle_rel_stub_max_mrad_C=rel, peak_gb_M=round(r["peak_gb"], 2), wall_s=r["wall_s"], iterations=r["iterations"])
res = json.load(open(os.path.join(D, CH, "result.json")))
res["chosen"] = CH
res["sweep"] = sweep
res["note"] = f"{CH}: " + sweep[CH]["connectivity"]
json.dump(res, open(os.path.join(D, "result.json"), "w"), indent=1)
with open(os.path.join(D, "analysis.txt"), "w") as f:
    for t in RUNS:
        f.write(f"######## {t}\n")
        for n in ("box_system_topo_v2r2_analyse.txt", "cuts.txt", "box_system_topo_v2r2_newlanes.txt"):
            f.write(f"--- {n}\n" + "".join(l for l in open(os.path.join(D, t, n)) if "Warning" not in l and "inverse_denominator" not in l and "barycentric" not in l) + "\n")
for n in ("log.txt", "density.npz", "iso05.stl", "view_iso.png", "view_front.png", "view_side.png", "view_top.png", "view_back_iso.png", "cuts.json", "connectivity.json"):
    shutil.copy(os.path.join(D, CH, n), os.path.join(D, n))
shutil.copy(os.path.join(D, "view_iso.png"), os.path.join(D, "preview.png"))
for t in RUNS:
    for n in ("ck.npz", "coarse_ck.npz", "pid", "stdout.txt"):
        p = os.path.join(D, t, n)
        if os.path.exists(p): os.remove(p)
print(json.dumps({t: (s["connectivity"], s["worst_ratio_M"], s["stub_tilt_mrad_C"]) for t, s in sweep.items()}, indent=1))
