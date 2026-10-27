"""Updates to nox/scout/v3/scout.assembly.json from other sessions' parts, applied after layout_v3.py writes it (and runnable alone,
idempotent): the saved camera mount (PR 146) in place of the layout's cradle, and the electronics session's tail bay round
(its mount and cable masses at their CGs, the Radxa's post keep-outs, then the belly mount v1's system model). Then system_check writes the run's system_check_mount.json.

python3 nox/scout/v3/scout_assembly_updates.py
"""
import json, os, subprocess, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from assembly import Assembly

ASM = os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")
RUN = os.path.join(ROOT, "runs", "scout_layout", "v12")
CAM_SAVED = os.path.join(ROOT, "runs", "scout_camera_mount", "v1", "scout_with_mount.assembly.json")
ELEC_R = os.path.join(RUN, "electronics_envelope_r5.json")      # the electronics session's round 5 (SOURCE.txt), drawn at the v12 catch
BELLY = os.path.join(ROOT, "runs", "scout_belly_mount", "v1")    # the electronics session's belly mount v1 (PR 150, concept): its system model
CAMPS = ("camera cradle (printed)", "camera mount (boom + seat)")   # the layout's camera mount part, by its CAM_MOUNT variant
# the Radxa's 4 M2.5 holes, board coords (X along 65 from the SD end, Y along 30 from the ports edge; H, STEP, from the electronics
# session's electronics_system.py RADXA_HOLES); its posts land on the PCB back (y -1.6) with Ø8.4 keep-outs (their round 5)
RADXA_HOLES = [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]
POST_R, PCB_BACK_Y = 4.2, -1.6


def camera_mount(a):
    CAMP = next((p["name"] for p in a["parts"] if p["name"] in CAMPS), None)
    if not os.path.exists(CAM_SAVED) or CAMP is None: return []
    sm = json.load(open(CAM_SAVED)); old = {CAMP, "camera cradle arms (printed)", "camera cradle clips (printed)"}
    mine = [p for p in sm["parts"] if p.get("owner") == "scout camera mount"]; mn = {p["name"] for p in mine}
    a["parts"] = [p for p in a["parts"] if p["name"] not in old] + mine
    a["contact_pairs"] = [[("camera mount (printed)" if v == CAMP else v) for v in c[:2]] + c[2:] for c in a["contact_pairs"]
                          if not ({c[0], c[1]} & (old - {CAMP}))] + [c for c in sm["contact_pairs"] if c[0] in mn - {"camera mount (printed)"}
                                                                     or (c[0] == "camera mount (printed)" and c[1] == "cage shell tubes")]
    a["note"] = sm["note"]
    return ["camera mount v1: " + ", ".join(f"{p['name']} {p['mass_g']} g" for p in mine)]


def electronics(a):
    if not os.path.exists(ELEC_R): return []
    e = json.load(open(ELEC_R)); P = {p["name"]: p for p in a["parts"]}; out = []
    blk = e["mount"]["belly-mount tail block (PETG)"]; tot = e["mount_total"]
    rest_g = tot["mass_g"] - blk["mass_g"]; rest_c = (np.array(tot["cg"]) * tot["mass_g"] - np.array(blk["cg"]) * blk["mass_g"]) / rest_g
    for n, g, c in (("tail bay mount block (Scout electronics, E)", blk["mass_g"], blk["cg"]),
                    ("tail bay carrier (Scout electronics, E)", rest_g, rest_c),
                    ("tail bay cables (Scout electronics, E)", e["cables"]["mass_g"], e["cables"]["cg"])):
        if n not in P: continue                                         # drawn by the belly mount's system model (below)
        P[n]["mass_g"] = round(float(g), 2); P[n]["cg"] = np.round(np.asarray(c, float), 2).tolist()
        P[n]["version"] = f"electronics round {e['round']} (concept)"; out.append(f"{n} {g:.1f} g")
    rx = P["Radxa Zero 3W"]; src = os.path.join(ROOT, rx["mesh"])
    if not src.endswith("_keepouts.stl"):
        m = trimesh.load(src); (x0, y0, z0), (x1, y1, z1) = m.bounds
        assert abs(x1 - x0 - 65.0) < 0.1 and abs(z1 - z0 - 30.0) < 0.1, "the Radxa on edge (65 x 30) expected"
        cuts = []
        for hx, hy in RADXA_HOLES:                                        # board X from the SD end (-x), Y down from the ports edge (top)
            c = trimesh.creation.cylinder(POST_R, PCB_BACK_Y - (y0 - 0.5), sections=32)
            c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
            c.apply_translation([x0 + hx, 0.5 * (PCB_BACK_Y + y0 - 0.5), z1 - hy]); cuts.append(c)
        m = m.difference(trimesh.util.concatenate(cuts)) if hasattr(m, "difference") else m
        dst = src[:-4] + "_keepouts.stl"; m.export(dst); rx["mesh"] = os.path.relpath(dst, ROOT)
        rx["note"] = "the 4 post keep-outs (Ø8.4, PCB back side to y -1.6) cut, positions H (STEP), electronics round 5"
        out.append(f"Radxa: 4 post keep-outs cut ({os.path.basename(dst)}, volume {m.volume:.0f} mm³)")
    return out


def belly(a):
    """the belly mount v1 system model (runs/scout_belly_mount/v1/manifest.json: main's assembly with the belly mount in place of the
    25 g placeholder, the tail block and carrier as drawn, the real FC board, the flow re-placed): its parts, contacts, clearances, access"""
    man = os.path.join(BELLY, "manifest.json")
    if not os.path.exists(man) or not any(p["name"] == "mount + nose legs (Scout)" for p in a["parts"]): return []
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import belly_mount as bm
    m = json.load(open(man)); r = json.load(open(os.path.join(BELLY, "result.json"))); d = os.path.join(RUN, "belly_v1"); os.makedirs(d, exist_ok=True)
    meshes = {"FC": bm.fc_mesh(r["fc"]["z"][0]), "flow MTF-02P": bm.flow_mesh(r["flow"]),
              "belly mount (PETG)": trimesh.load(os.path.join(BELLY, "belly_mount__petg_.stl"))}
    SWAP = {"belly mount (PETG)", "electronics tail block (PETG)", "electronics tail carrier (PETG)", "FC", "flow MTF-02P", "wires"}
    GONE = {"mount + nose legs (Scout)", "tail bay mount block (Scout electronics, E)", "tail bay carrier (Scout electronics, E)"}
    new = []
    for p in m["parts"]:
        if p["name"] not in SWAP: continue                               # only their pieces; every other part stays the layout's
        if p.get("trimesh"):
            f = os.path.join(d, p["name"].split(" (")[0].replace(" ", "_").lower() + ".stl")
            if p["name"] != "belly mount (PETG)": meshes[p["name"]].export(f)
            else: f = os.path.join(BELLY, "belly_mount__petg_.stl")
            del p["trimesh"]; p["mesh"] = os.path.relpath(f, ROOT)
        elif "mesh" in p and os.path.isabs(p["mesh"]): p["mesh"] = os.path.relpath(p["mesh"], ROOT)
        new.append(p)
    a["parts"] = [p for p in a["parts"] if p["name"] not in SWAP | GONE] + new
    touched = SWAP | GONE
    a["contact_pairs"] = [c for c in a["contact_pairs"] if not ({c[0], c[1]} & touched)] + [c for c in m["contact_pairs"] if {c[0], c[1]} & SWAP]
    a["contact_pairs"] = [c for c in a["contact_pairs"] if not ("tail bay cables (Scout electronics, E)" in c[:2] and c[2] >= 1e9)]   # the cables:
                                                                     # a mass point, no blanket pairs (a future cable clash shows)
    for k in ("clearance_pairs", "access"):
        if k in m: a[k] = m[k]
    a["note"] = "belly mount v1 (concept, runs/scout_belly_mount/v1) system model: " + m["note"] + " " + a["note"]
    bp = next(p for p in m["parts"] if p["name"] == "belly mount (PETG)")
    return [f"belly mount v1 (concept) {bp['mass_g']} g E, FC 36 x 36 on the AIO holes, flow at ({r['flow']['c'][0]}, {r['flow']['c'][1]}, {r['flow']['zb']})"]


def main():
    a = json.load(open(ASM)); done = camera_mount(a) + electronics(a) + belly(a)
    Assembly(a, ASM).save(ASM)
    subprocess.run([sys.executable, os.path.join(ROOT, "nox", "system_check.py"), ASM, "--report", "--out", os.path.join(RUN, "system_check_mount.json")],
                   capture_output=True, cwd=ROOT)                 # the view's and summary's numbers with these in
    return done


if __name__ == "__main__":
    for s in main(): print(s)
