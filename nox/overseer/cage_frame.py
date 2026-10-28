"""Overseer cage frame model (F): the cage v5 tubes, the links and the sensor head on the core, under the system cases of
dimensions.md L. Checks every member (pinned-end buckling, stress) and writes the forces each node socket really carries, for the
node and head-mount topology runs (in place of their E loads).

Model: cage nodes as rigid joints (PETG nodes, Spark 27 Oct; a 15-deep epoxied socket is not fully fixed, so the buckling check keeps pinned ends for cage tubes, conservative); the 6/4 carbon tubes and the 3.0
carbon links as beams; the core (plates, arms, belly) fixed: every link's core end is a support; the sensor head mount one rigid
body (H) on its keels and braces, holding the cage's front centre node and the wing struts.
Cases: crash 35 g through the cage member that touches first, 14 directions (the drone's mass, the core held); a nose hit on the
head guard (the same force into H); flight 4 g on the cage and head masses; handling 50 N at mid-span of each tube (closed form).

python3 nox/overseer/cage_frame.py  -> runs/overseer_frame/v1/ (summary.md, members.json, node_loads.json)
"""
import os, sys, json, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
from frame_model import Frame, tube
HEAD_STRUTS = [((401.0, 83.0, 151.0), (354.0, 223.0, 128.7)), ((383.0, 83.0, 73.0), (336.5, 219.0, -29.3))]   # = head_mount.HEAD_STRUTS

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_frame", "v1")
CG = json.load(open(os.path.join(HERE, "cage_v5_shell.json")))
RES = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "result.json")))
HS = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "head_system.json")))
G = 9.81
MASS = RES["mass"] / 1000.0                       # kg, the drone (E)
CRASH, FLIGHT = 35.0, 4.0                         # g (dimensions.md L)
HANDLE = 50.0                                     # N at mid-span (E)
SIG_T, SIG_C = 1000.0, 600.0                      # MPa allowables, pultruded carbon (E)
TUBE6 = tube(6.0, 4.0); ROD3 = tube(3.0, 0.0)
RIGID = dict(E=100e6, G=40e6, A=1e3, I=1e5, J=2e5, c=5.0)
K_CORE = 0.7                                      # a link's core end epoxied 15 deep in a printed PETG socket ~ fixed, its cage end taken
                                                  # as pinned: effective length 0.7 L (E); cage tubes pinned both ends, 1.0 (conservative on PETG nodes)
N = np.array(CG["nodes"]); B = np.array(HS["l2"]["base"]); A = np.array(HS["l2"]["axis"])
_k = [i for i in range(len(N)) if abs(N[i, 1]) < 0.5 and N[i, 0] > 300]
HEAD_NODE = max(_k, key=lambda i: N[i, 0])
M_HEAD = (230 + 30 + float(HS["mass_budget_g"])) / 1000.0   # L2 + cameras + the printed head (head_system.json, E)
LS = os.path.join(HERE, "link_sizes.json")        # links sized up to 6/4 tube (the only other stock, Spark 27 Oct), written here
LINK6 = set(json.load(open(LS))["tube6"]) if os.path.exists(LS) else set()
sec_of = lambda k: TUBE6 if k in LINK6 else ROD3


def build():
    f = Frame(); nid = [f.node(p) for p in N]
    for a, b in CG["rods"]: f.member(nid[a], nid[b], TUBE6, f"tube {a}-{b}")
    H = f.node(B - A * 20.0)                                            # the head mount, rigid
    if not HS.get("rods"): f.member(H, nid[HEAD_NODE], RIGID, "head-centre node")   # (the 3-rod head holds no cage node)
    for k, (p, q) in RES["links"].items():
        if k.startswith("head strut"): continue                        # built with the head below
        if k.startswith("head rod"):                                   # the 3-rod head (Spark 27 Oct): head socket rigid on H, the rod to its cage node
            p, q = np.array(p), np.array(q); i = int(np.argmin(np.linalg.norm(N - q, axis=1)))
            Wn = f.node(p); f.member(H, Wn, RIGID, f"head-{k}"); f.member(Wn, nid[i], sec_of(k), k); continue
        p, q = np.array(p), np.array(q)
        on = [i for i in range(len(N)) if np.linalg.norm(N[i] - p) < 0.6 or np.linalg.norm(N[i] - q) < 0.6]
        if k.startswith(("keel", "L2 brace")):                          # head lug to the core
            lug = p if (p - B) @ A > -40 else q; core = q if lug is p else p
            L_ = f.node(lug); C_ = f.node(core); f.member(H, L_, RIGID, f"head-{k}"); f.member(L_, C_, sec_of(k), k, k=0.5); f.fix(C_)   # PETG both ends
        elif on:                                                        # a link from a cage node to the core
            i = on[0]; other = q if np.linalg.norm(N[i] - p) < 0.6 else p; C_ = f.node(other); f.member(nid[i], C_, sec_of(k), k, k=K_CORE); f.fix(C_)
    struts = []                                                         # the one-piece head's struts to the side nodes (head_mount.py)
    if os.environ.get("V5_HEAD_ON", "core") != "cage" and not HS.get("rods"):
        for w, q in HEAD_STRUTS:
            for sy in (1, -1):
                w_, i = np.array(w) * [1, sy, 1], int(np.argmin(np.linalg.norm(N - np.array(q) * [1, sy, 1], axis=1)))
                Wn = f.node(w_); f.member(H, Wn, RIGID, "head-strut"); f.member(Wn, nid[i], sec_of(f"head strut {i}"), f"head strut {i}"); struts.append(i)
    W0 = np.array([368.0, 112.0, 70.0])                                 # the old wing struts (two-piece head), off
    for i, q in enumerate(N):
        if abs(q[1]) > 150 and q[0] > 300 and os.environ.get("V5_HEAD_ON", "core") == "wings":
            w = W0 * [1, np.sign(q[1]), 1]; d = (w - q) / np.linalg.norm(w - q)
            tub = [N[b if a == i else a] - q for a, b in CG["rods"] if i in (a, b)]
            if all(math.degrees(math.acos(np.clip(d @ (t / np.linalg.norm(t)), -1, 1))) >= 20 for t in tub):
                Wn = f.node(w); f.member(H, Wn, RIGID, "head-wing"); f.member(Wn, nid[i], sec_of(f"wing strut {i}"), f"wing strut {i}"); struts.append(i)
    return f, nid, H, struts


def node_mass(f, nid):
    m = np.zeros(len(f.X))
    for a, b in CG["rods"]:
        L = np.linalg.norm(N[a] - N[b]); w = L * 24.3e-6; m[nid[a]] += w / 2; m[nid[b]] += w / 2   # kg (6/4 tube 24.3 g/m)
    for i in nid: m[i] += 4.5e-3
    return m


def head_forces(f, u, load_h=None):
    """the force (and moment) the rest of the frame puts on the rigid head at each attachment, global frame (N, N mm): the end
    force of each rigid head member at its far end (the keel and brace lugs, the strut sockets, the cage's front centre node)"""
    out = []
    for m, (T, kl, L) in zip(f.M, f._cache):
        if not m["name"].startswith("head-"): continue
        ue = np.concatenate([u[6 * m["a"]:6 * m["a"] + 6], u[6 * m["b"]:6 * m["b"] + 6]]); fl = kl @ (T @ ue); R = T[:3, :3]
        out.append(dict(name=m["name"], at=np.round(np.asarray(f.X[m["b"]], float), 2).tolist(),
                        F=np.round(R.T @ fl[6:9], 2).tolist(), M=np.round(R.T @ fl[9:12], 1).tolist()))   # the end force on the rigid link = on the head
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    f, nid, H, struts = build()
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    guard = np.array(HS["guard_nodes"])
    F_crash = MASS * G * CRASH
    cases = {}
    for k, d in enumerate(dirs):                                         # the obstacle from d meets the most extreme member first
        sup_c = N @ d; i = int(np.argmax(sup_c)); sup_h = (guard @ d).max() if len(guard) else -1e9   # no guard ring (Spark 27 Oct): cage nodes only
        if sup_h > sup_c[i] + 3.0: cases[f"crash {k} {np.round(d, 2).tolist()} via head guard"] = {H: -d * F_crash}
        else: cases[f"crash {k} {np.round(d, 2).tolist()} via node {i}"] = {nid[i]: -d * F_crash}
    for k, d in enumerate(dirs):                                         # the head's own 35 g (the L2, cameras and mount decelerating) on its supports
        cases[f"head inertia {k} {np.round(d, 2).tolist()}"] = {H: d * M_HEAD * G * CRASH}
    m = node_mass(f, nid); fl = {i: np.array([0, 0, -m[i] * G * FLIGHT]) for i in range(len(m)) if m[i] > 0}
    fl[H] = fl.get(H, 0) + np.array([0, 0, -M_HEAD * G * FLIGHT]); cases["flight 4 g down"] = fl
    res = {}; worst = {}; head = {}
    for cn, loads in cases.items():
        u = f.solve(loads); mf_ = f.member_forces(u); res[cn] = mf_
        head[cn] = head_forces(f, u, loads.get(H))
        for r in mf_:
            w = worst.setdefault(r["name"], dict(L=r["L"], buckle=0.0, sigma=0.0, N_max=0.0, N_min=0.0, case_b="", case_s=""))
            if r["buckle"] > w["buckle"]: w["buckle"], w["case_b"] = r["buckle"], cn
            if r["sigma"] > w["sigma"]: w["sigma"], w["case_s"] = r["sigma"], cn
            w["N_max"] = max(w["N_max"], r["N"]); w["N_min"] = min(w["N_min"], r["N"])
    # handling: 50 N at mid-span, pinned-pinned (closed form)
    for nm, w in worst.items():
        sec = TUBE6 if nm.startswith("tube") else sec_of(nm)
        if nm.startswith("head-") or nm == "head-centre node": continue
        L = w["L"]; w["defl_handling"] = HANDLE * L ** 3 / (48 * sec["E"] * sec["I"]); w["sig_handling"] = HANDLE * L / 4 * sec["c"] / sec["I"]
    # node socket loads: per cage node and member, the largest axial and side force over the cases
    nl = {}
    for cn, mf_ in res.items():
        for r in mf_:
            if not (r["name"].startswith("tube") or r["name"] in RES["links"] or r["name"].startswith(("wing strut", "head strut"))): continue
            for end, fe in ((r["a"], r["f_end_a"]), (r["b"], r["f_end_b"])):
                if end not in nid: continue
                i = nid.index(end); e = nl.setdefault(str(i), {}).setdefault(r["name"], dict(axial=0.0, side=0.0))
                e["axial"] = max(e["axial"], abs(float(fe[0]))); e["side"] = max(e["side"], float(np.hypot(fe[1], fe[2])))
    json.dump({k: {kk: {kkk: round(vvv, 3) if isinstance(vvv, float) else vvv for kkk, vvv in vv.items()} for kk, vv in v.items()}
               for k, v in [("members", worst)]}, open(os.path.join(OUT, "members.json"), "w"), indent=1)
    json.dump(nl, open(os.path.join(OUT, "node_loads.json"), "w"), indent=1)
    json.dump(dict(note="forces (N) and moments (N mm) the frame puts on the rigid sensor head at each attachment, global drone frame, "
                        "per case of cage_frame.py (crash 35 g x 14 through the first-touching cage member, core fixed; flight 4 g); "
                        "head_mount.py loads its topology with these (F on E)", cases=head),
              open(os.path.join(OUT, "head_loads.json"), "w"), indent=1)
    # the arm groups (overseer arm session): the summed link force on each arm's spoke node and on each front beam's brace roots,
    # per case (N, vector on the arm; the frame holds the core rigid)
    grp = {f"arm {a} spoke node": [k for k in RES["links"] if k.startswith(f"spoke {a}.")] for a in (1, 2, 3, 4)}
    grp.update({f"L2 braces {sy:+d}": [k for k in RES["links"] if k.startswith("L2 brace") and k.endswith(f"{sy:+d}")] for sy in (1, -1)})
    arm = {}
    for cn, mf_ in res.items():
        byn = {r["name"]: r for r in mf_}
        for gname, ks in grp.items():
            F = np.zeros(3)
            for k in ks:
                p_, q_ = (np.array(v) for v in RES["links"][k]); core = q_ if gname.startswith("L2") else p_
                far = p_ if core is q_ else q_; F += byn[k]["N"] * (far - core) / np.linalg.norm(far - core)   # tension pulls the arm toward the far end
            arm.setdefault(gname, {})[cn] = np.round(F, 1).tolist()
    core_root = {}                                                      # the core session's hub deck tower: keels + L2 braces per case
    for cn, mf_ in res.items():
        core_root[cn] = {r["name"]: round(float(r["N"]), 1) for r in mf_ if r["name"].startswith(("keel", "L2 brace"))}
    json.dump(dict(note="axial force N (+ tension), no SF, per case of cage_frame.py (crash 35 g x 14 through the first-touching member, flight 4 g); "
                        "core = the tube end in the hub deck tower, lug = the head end (F on E)",
                   points={k: dict(core=(v[1] if k.startswith("L2") else v[0]), lug=(v[0] if k.startswith("L2") else v[1]))
                           for k, v in RES["links"].items() if k.startswith(("keel", "L2 brace"))},
                   cases=core_root), open(os.path.join(OUT, "core_root_forces.json"), "w"), indent=1)
    arm_worst = {g_: max(((float(np.linalg.norm(v)), c_) for c_, v in d.items())) for g_, d in arm.items()}
    json.dump(dict(cases=arm, worst={g_: [round(v[0]), v[1]] for g_, v in arm_worst.items()}), open(os.path.join(OUT, "arm_loads.json"), "w"), indent=1)
    m_g = {i: v * 1000 for i, v in enumerate(m) if v > 0}; m_g[H] = m_g.get(H, 0) + M_HEAD * 1000   # the frame's lowest modes, rigid joints
    hz, U = f.modes(m_g, 3); hd = [float(np.linalg.norm(U[6 * H:6 * H + 3, k]) / max(1e-12, np.abs(U[:, k]).max())) for k in range(len(hz))]
    cage_g = sum(np.linalg.norm(N[a] - N[b]) * 24.3e-3 for a, b in CG["rods"]) + 4.5 * len(N)
    tubes = {k: v for k, v in worst.items() if k.startswith("tube")}; links = {k: v for k, v in worst.items() if not k.startswith(("tube", "head-"))}
    bad_b = sorted([(v["buckle"], k, v["L"]) for k, v in {**tubes, **links}.items() if v["buckle"] >= 1.0], reverse=True)
    bad_s = sorted([(v["sigma"], k) for k, v in {**tubes, **links}.items() if v["sigma"] > SIG_C], reverse=True)
    soft = sorted([(v["defl_handling"], k, v["L"]) for k, v in tubes.items() if v.get("defl_handling", 0) > 10.0], reverse=True)
    L_ = []
    say = lambda s: (print(s), L_.append(s))
    say(f"# Overseer cage frame model v1 (F, on E properties: carbon E 100 GPa, allowables {SIG_T:.0f} / {SIG_C:.0f} MPa; buckling k 1.0 cage tubes, {K_CORE} links with a core end, 0.5 keels / braces (PETG both ends), E)\n")
    say(f"Cage `nox/overseer/cage_v5_shell.json` ({len(N)} nodes, {len(CG['rods'])} tubes 6/4, {cage_g:.0f} g E) + {len(links)} links on the core (fixed; "
        f"{sum(k in LINK6 for k in links)} tube 6/4, the rest rod 3.0); "
        f"sensor head rigid on {sum(k.startswith('keel') for k in RES['links'])} keels + {sum(k.startswith('L2 brace') for k in RES['links'])} braces, holding node {HEAD_NODE} and struts to nodes {struts}. Drone {MASS * 1000:.0f} g (E): crash force "
        f"{F_crash:.0f} N through the first-touching member, 14 directions; flight 4 g; handling {HANDLE:.0f} N at mid-span (closed form).\n")
    say(f"Lowest modes (rigid joints, lumped masses, the head {M_HEAD * 1000:.0f} g (E) at its mount): " + ", ".join(f"{h_:.0f} Hz" + (" (the head moves)" if hd[k] > 0.3 else "")
        for k, h_ in enumerate(hz)) + "; TPU joints and the head's own mode: runs/overseer_head_cage/concept/.\n")
    say(f"## Buckling (k L): {len(bad_b)} members at or over 1.0 x their Euler load")
    for b_, k, L in bad_b[:20]: say(f"- {k}: {b_:.2f} x Pcr (L {L:.0f}; {worst[k]['case_b']})")
    say(f"\n## Stress over {SIG_C:.0f} MPa: {len(bad_s)} members")
    for s_, k in bad_s[:15]: say(f"- {k}: {s_:.0f} MPa ({worst[k]['case_s']})")
    say(f"\n## Handling {HANDLE:.0f} N at mid-span: tubes bending over 10 mm: {len(soft)}")
    for d_, k, L in soft[:15]: say(f"- {k}: {d_:.0f} mm (L {L:.0f})")
    say("\n## Load into the arms (summed link force per group, core rigid; `arm_loads.json`)")
    for g_, (v, c_) in sorted(arm_worst.items()): say(f"- {g_}: {v:.0f} N ({c_})")
    say(f"\nWorst tube buckling {max(v['buckle'] for v in tubes.values()):.2f}, worst link {max([v['buckle'] for v in links.values()] or [0]):.2f}; "
        f"node socket loads (largest axial / side per member and node over all cases) in node_loads.json.")
    up = sorted({k for k, v in links.items() if v["buckle"] >= 1.0 and k not in LINK6})   # a 3.0 link that buckles becomes a 6/4 tube
    if up:
        json.dump(dict(note="links the frame model sized up to 6/4 carbon tube (nox/overseer/cage_frame.py); every other link 3.0 solid",
                       tube6=sorted(LINK6 | set(up))), open(LS, "w"), indent=1)
        say(f"\nsized up to 6/4 tube this pass: {up} (rerun layout_v5.py, then this, until none)")
    open(os.path.join(OUT, "summary.md"), "w").write("\n".join(L_) + "\n")


if __name__ == "__main__":
    main()
