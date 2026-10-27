"""Scout cage frame model (F): the v12 race-car cage (every rod 3.0 solid carbon, or 6/4 tube on the sized list), the 8 saddle ties into the core and
the camera mount v1 on its 4 snap clips, under the system cases of dimensions.md L. Checks every member (buckling, stress, handling) and the camera
mount's first mode, and writes the forces each node socket really carries, for the node topology runs (nox/scout/v3/cage_node.py).

Model (E properties: carbon E 100 GPa, allowables 1000 / 600 MPa):
- cage nodes rigid joints (PETG, epoxied 3.2 sockets); shell rods pinned both ends for buckling (k 1.0, conservative);
- the saddle ties' saddle ends fixed (the saddle is bolted to the plate, the core held), k 0.7;
- the camera + mount one rigid body on its 4 clips, which split their rods (the clips grip, requirement).
Cases: crash 35 g through the most extreme node, 14 directions (the drone's mass, the core held); flight 4 g on the cage and camera
masses; handling 50 N at mid-span of each rod (closed form). Modes: the lowest 3 with lumped masses.

python3 nox/scout/v3/cage_frame.py [out]  -> runs/scout_frame/v3/ (summary.md, members.json, node_loads.json)
"""
import os, sys, json, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from frame_model import Frame, tube

RUN = os.environ.get("SCOUT_FRAME_RUN", os.path.join(ROOT, "runs", "scout_layout", "v12"))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_frame", "v3")
CG = json.load(open(os.path.join(RUN, "cage_shell.json"))); RES = json.load(open(os.path.join(RUN, "result.json")))
G = 9.81
MASS = sum(p["mass_g"] for p in json.load(open(os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")))["parts"]) / 1000.0   # kg, the drone (E, the assembly)
CRASH, FLIGHT, HANDLE = 35.0, 4.0, 50.0            # g, g, N (dimensions.md L; handling E)
SIG_T, SIG_C = 1000.0, 600.0                       # MPa allowables, pultruded carbon (E)
ROD3 = tube(3.0, 0.0); TUBE6 = tube(6.0, 4.0); RIGID = dict(E=100e6, G=40e6, A=1e3, I=1e5, J=2e5, c=5.0)
ROD_GPM = 7.07 * 1.55e-3                           # g/mm, 3.0 solid carbon (E)
TUBE_GPM = 15.71 * 1.55e-3                         # g/mm, 6/4 carbon tube (E)
NODE_G, RING_NODE_G = 1.5, 4.5                     # g (E, layout_v3.py)
CLIPS = [(110.5, 18.0, 61.1), (122.6, 18.0, 11.5)]  # the camera mount v1's snap clips (+y; mirrored) on rods 7-8 and 14-15 (v12, camera_mount.py)
CAM_C, CAM_G = (136.0, 0.0, 41.7), 31.8             # camera + mount v1 + 2 collars, g at their CG (M mount, E camera; runs/scout_camera_mount/v1)
K_TIE = 0.7                                        # a saddle tie: epoxied into the PETG saddle (~ fixed), pinned at its cage node
N = np.array(CG["nodes"], float); nR = len(CG["open"][0]["ring"])
SHELL_SEC = TUBE6 if os.environ.get("SCOUT_FRAME_SHELL") == "6" else ROD3      # variants: 6/4 tube shell, 6/4 tube ties (Spark rule: 3.0)
TIE_SEC = TUBE6 if os.environ.get("SCOUT_FRAME_TIES") == "6" else ROD3
_U6 = os.environ.get("SCOUT_FRAME_UP6")                                          # a json list: members sized up to 6/4 tube (the sizing loop)
UP6 = set(json.load(open(_U6))) if _U6 and os.path.exists(_U6) else set()


def seg_d(c, p, q):
    t = np.clip(np.dot(c - p, q - p) / np.dot(q - p, q - p), 0, 1); return np.linalg.norm(c - (p + t * (q - p)))


def build():
    f = Frame(); nid = [f.node(p) for p in N]; rods = [tuple(r) for r in CG["rods"]]
    clips = [np.array(c) * [1, s, 1] for c in CLIPS for s in (1, -1)]
    split = {}
    for c in clips:                                                      # the rod each clip sits on
        best = min(rods, key=lambda r: seg_d(c, N[r[0]], N[r[1]]))     # distance to the segment, not its line
        split.setdefault(best, []).append(c)
    for a, b in rods:
        pts = [N[a]] + sorted(split.get((a, b), []), key=lambda c: np.linalg.norm(c - N[a])) + [N[b]]
        sec_ = TUBE6 if f"rod {a}-{b}" in UP6 else SHELL_SEC
        for p_, q_ in zip(pts[:-1], pts[1:]): f.member(f.node(p_), f.node(q_), sec_, f"rod {a}-{b}")
    H = f.node(CAM_C)
    for c in clips: f.member(H, f.node(c), RIGID, "camera clip")
    for k, (p, q) in RES["links"].items():
        p, q = np.array(p), np.array(q); C_ = f.node(p); f.member(C_, f.node(q), TUBE6 if k in UP6 else TIE_SEC, k, k=K_TIE); f.fix(C_)
    return f, nid, H


def masses(f, nid, H):
    m = {}
    for a, b in CG["rods"]:
        w = np.linalg.norm(N[a] - N[b]) * (TUBE_GPM if f"rod {a}-{b}" in UP6 or SHELL_SEC is TUBE6 else ROD_GPM); m[nid[a]] = m.get(nid[a], 0) + w / 2; m[nid[b]] = m.get(nid[b], 0) + w / 2
    for i, j in enumerate(nid): m[j] = m.get(j, 0) + (RING_NODE_G if i < nR else NODE_G)
    m[H] = m.get(H, 0) + CAM_G
    return m                                                             # g


def main():
    os.makedirs(OUT, exist_ok=True)
    f, nid, H = build(); m = masses(f, nid, H)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    F_crash = MASS * G * CRASH; cases = {}
    for k, d in enumerate(dirs):
        i = int(np.argmax(N @ d)); cases[f"crash {k} {np.round(d, 2).tolist()} via node {i}"] = {nid[i]: -d * F_crash}
    cases["flight 4 g down"] = {j: np.array([0, 0, -g_ / 1000 * G * FLIGHT]) for j, g_ in m.items()}
    res, worst = {}, {}
    for cn, loads in cases.items():
        u = f.solve(loads); mf_ = f.member_forces(u); res[cn] = mf_
        for r in mf_:
            if r["name"] == "camera clip": continue
            w = worst.setdefault(r["name"], dict(L=0.0, buckle=0.0, sigma=0.0, N_max=0.0, N_min=0.0, case_b="", case_s=""))
            w["L"] = max(w["L"], r["L"])
            if r["buckle"] > w["buckle"]: w["buckle"], w["case_b"] = r["buckle"], cn
            if r["sigma"] > w["sigma"]: w["sigma"], w["case_s"] = r["sigma"], cn
            w["N_max"] = max(w["N_max"], r["N"]); w["N_min"] = min(w["N_min"], r["N"])
    for nm, w in worst.items():                                          # handling 50 N mid-span, pinned-pinned, the full rod
        if nm.startswith("rod "):
            a, b = map(int, nm.split()[1].split("-")); L = float(np.linalg.norm(N[a] - N[b]))
        else:
            L = w["L"]
        w["L_full"] = L; sc_ = TUBE6 if nm in UP6 else SHELL_SEC
        w["defl_handling"] = HANDLE * L ** 3 / (48 * sc_["E"] * sc_["I"]); w["sig_handling"] = HANDLE * L / 4 * sc_["c"] / sc_["I"]
    nl = {}                                                              # node socket loads (largest axial / side per member)
    for cn, mf_ in res.items():
        for r in mf_:
            if r["name"] == "camera clip": continue
            for end, fe in ((r["a"], r["f_end_a"]), (r["b"], r["f_end_b"])):
                if end not in nid: continue
                i = nid.index(end); e = nl.setdefault(str(i), {}).setdefault(r["name"], dict(axial=0.0, side=0.0))
                e["axial"] = max(e["axial"], abs(float(fe[0]))); e["side"] = max(e["side"], float(np.hypot(fe[1], fe[2])))
    json.dump(worst, open(os.path.join(OUT, "members.json"), "w"), indent=1)
    json.dump(nl, open(os.path.join(OUT, "node_loads.json"), "w"), indent=1)
    hz, U = f.modes(m, 3)
    hd = [float(np.linalg.norm(U[6 * H:6 * H + 3, k]) / max(1e-12, np.abs(U[:, k]).max())) for k in range(len(hz))]
    rods = {k: v for k, v in worst.items() if k.startswith("rod ")}; ties = {k: v for k, v in worst.items() if not k.startswith("rod ")}
    bad_b = sorted([(v["buckle"], k, v["L"]) for k, v in worst.items() if v["buckle"] >= 1.0], reverse=True)
    bad_s = sorted([(v["sigma"], k) for k, v in worst.items() if v["sigma"] > SIG_C], reverse=True)
    bad_h = sorted([(v["sig_handling"], k, v["L_full"]) for k, v in rods.items() if v["sig_handling"] > SIG_C], reverse=True)
    soft = sorted([(v["defl_handling"], k, v["L_full"]) for k, v in rods.items() if v["defl_handling"] > 10.0], reverse=True)
    L_ = []
    say = lambda s: (print(s), L_.append(s))
    say(f"# Scout cage frame model (F on E properties: carbon E 100 GPa, allowables {SIG_T:.0f} / {SIG_C:.0f} MPa; buckling k 1.0 shell rods, {K_TIE} saddle ties)\n")
    say(f"Cage `{os.path.relpath(RUN, ROOT)}/cage_shell.json`: {len(N)} nodes, {len(CG['rods'])} shell rods + {len(RES['links'])} saddle ties, " + (f"{len(UP6)} members 6/4 tube (the sized list), the rest 3.0 solid; " if UP6 else "every rod 3.0 solid; ") +
        f"the ties' saddle ends fixed (the core held); the camera + mount ({CAM_G:.1f} g, M mount + E camera) rigid on its 4 clips. Drone {MASS * 1000:.0f} g (E): crash "
        f"{F_crash:.0f} N through the most extreme node, 14 directions; flight 4 g; handling {HANDLE:.0f} N at mid-span (closed form).\n")
    say("Lowest modes (rigid joints, lumped masses): " + ", ".join(f"{h_:.0f} Hz" + (" (the camera moves)" if hd[k] > 0.3 else "") for k, h_ in enumerate(hz)) + "\n")
    say(f"## Buckling: {len(bad_b)} members at or over 1.0 x their Euler load")
    for b_, k, L in bad_b[:25]: say(f"- {k}: {b_:.2f} x Pcr (segment L {L:.0f}; {worst[k]['case_b']})")
    say(f"\n## Crash stress over {SIG_C:.0f} MPa: {len(bad_s)} members")
    for s_, k in bad_s[:20]: say(f"- {k}: {s_:.0f} MPa ({worst[k]['case_s']})")
    say(f"\n## Handling {HANDLE:.0f} N at mid-span: {len(bad_h)} rods over {SIG_C:.0f} MPa, {len(soft)} bending over 10 mm")
    for s_, k, L in bad_h[:20]: say(f"- {k}: {s_:.0f} MPa, {worst[k]['defl_handling']:.0f} mm (L {L:.0f})")
    say(f"\nWorst shell rod buckling {max(v['buckle'] for v in rods.values()):.2f}, worst tie {max([v['buckle'] for v in ties.values()] or [0]):.2f}; "
        f"node socket loads (largest axial / side per member and node over all cases) in node_loads.json.")
    open(os.path.join(OUT, "summary.md"), "w").write("\n".join(L_) + "\n")


if __name__ == "__main__":
    main()
