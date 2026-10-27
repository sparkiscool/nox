"""Overseer sensor head on the cage (Spark 27 Oct: "lets try to mount the lidar on the cage of the drone so we dont need the rods
coming from the body"): concept check of three head supports on the frame model (F on E properties), cage v5 and links as built.

  A  as v5: the head on 2 keels + 4 L2 braces to the core, plus the cage's front centre node and the wing struts
  B  cage only: no keels, no braces; the head holds the front centre node (17) and the wing struts to the side nodes (6/4 tube)
  C  B + top struts (6/4 tube) from the guard ring's top to the cage's top nodes behind it (the centre one and the two sides)
  D  B + the centre top strut only
  Z  cage only, no struts at all: the head holds the front centre node (17) and its 4 tubes, nothing else
  G  A without the 4 L2 braces (the 2 keels only);  H  A without the 2 keels (the 4 braces only)
  F  B + struts to every node in reach: the centre top, the two top-side front nodes, the base loop's front node

Cases (core fixed): a nose hit (the drone's mass at 35 g into the head, the directions where the head guard leads), the head's own
inertia at 35 g in 14 directions at its CG (L2 230 + cameras 30 + mount 100 g), crash 35 g through the first-touching cage node,
flight 4 g. First modes with the head masses at their CGs, two joint bounds: rigid nodes, and TPU nodes (each tube end 6 mm short of
the node centre, joined by a TPU 95A stub Ø11.3, E 26 MPa: E). Needs: buckle < 1, stress < 600 MPa, head first mode >= 100 Hz.

python3 nox/overseer/head_cage_concept.py -> runs/overseer_head_cage/concept/ (summary.md, result.json, concept.png)
"""
import os, sys, json, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
import cage_frame as cf
from frame_model import Frame, tube

OUT = os.path.join(ROOT, "runs", "overseer_head_cage", "concept")
N, CG, RES, HS = cf.N, cf.CG, cf.RES, cf.HS
B, A = cf.B, cf.A; UPB = np.array([-A[2], 0, A[0]])
G, CRASH, FLIGHT = cf.G, cf.CRASH, cf.FLIGHT
TUBE6, ROD3 = cf.TUBE6, cf.ROD3
RIGID = dict(E=100e3, G=40e3, A=1e3, I=1e5, J=2e5, c=5.0)                # 'rigid' head links: ~2000 x a tube's EI (cf's 1e6 x leaves K
                                                                  # near singular, condition 8e13, and the modes wrong)
TPU = dict(tube(11.3, 0.0, E=26.0), G=26.0 / 3.0)               # TPU 95A node core stub (E)
SET = 6.0                                                         # tube end short of the node centre (E)
M_L2, M_CAM, M_MOUNT = 230.0, 15.0, 100.0                         # g (P, E, E system model)
M_DRONE = RES["mass"] / 1000.0
L2CG = np.array(HS["l2"]["cg"]); CAMS = [np.array(c["cg"]) for c in HS["cameras"]]
TOP = [i for i in range(len(N)) if N[i, 0] < B[0] - 60 and N[i, 2] > 100 and N[i, 0] > 150]   # cage top nodes behind the head


def build(var, soft):
    f = Frame(); nid = [f.node(p) for p in N]; mass = {}
    petg = {cf.HEAD_NODE}                                          # the head's own node is printed in the head (PETG)
    add_m = lambda i, g: mass.__setitem__(i, mass.get(i, 0.0) + g)
    def end(i, other):                                             # where a tube into cage node i really starts
        if not soft or i in petg: return nid[i]
        d = other - N[i]; p = N[i] + d / np.linalg.norm(d) * SET
        j = f.node(p, tol=1e-3); f.member(nid[i], j, TPU, f"tpu {i}"); return j
    for a, b in CG["rods"]:
        L = np.linalg.norm(N[a] - N[b]); f.member(end(a, N[b]), end(b, N[a]), TUBE6, f"tube {a}-{b}")
        add_m(nid[a], L * 24.3e-3 / 2); add_m(nid[b], L * 24.3e-3 / 2)
    for i in range(len(N)): add_m(nid[i], 4.5)
    H = f.node(B - A * 20.0); add_m(H, M_MOUNT)
    Lc = f.node(L2CG); f.member(H, Lc, RIGID, "head-L2"); add_m(Lc, M_L2)
    for k, c in enumerate(CAMS): Cn = f.node(c); f.member(H, Cn, RIGID, f"head-cam {k}"); add_m(Cn, M_CAM)
    f.member(H, nid[cf.HEAD_NODE], RIGID, "head-centre node")
    carbon = 0.0
    for k, (p, q) in RES["links"].items():
        if k.startswith("head strut"): continue
        p, q = np.array(p), np.array(q); sec = cf.sec_of(k); gpm = 24.3 if sec is TUBE6 else 10.5
        if k.startswith(("keel", "L2 brace")):
            if var not in ("A", "G", "H") or (var == "G" and k.startswith("L2 brace")) or (var == "H" and k.startswith("keel")): continue
            lug = p if (p - B) @ A > -40 else q; core = q if lug is p else p
            L_ = f.node(lug); C_ = f.node(core); f.member(H, L_, RIGID, f"head-{k}"); f.member(L_, C_, sec, k); f.fix(C_)
            carbon += np.linalg.norm(p - q) * gpm / 1000; add_m(L_, np.linalg.norm(p - q) * gpm / 2000)
            continue
        on = [i for i in range(len(N)) if np.linalg.norm(N[i] - p) < 0.6 or np.linalg.norm(N[i] - q) < 0.6]
        if on:
            i = on[0]; other = q if np.linalg.norm(N[i] - p) < 0.6 else p; C_ = f.node(other)
            f.member(end(i, other), C_, sec, k); f.fix(C_); add_m(nid[i], np.linalg.norm(p - q) * gpm / 2000)
    W0 = np.array([float(v) for v in os.environ.get("V5_W0", "368,112,70").split(",")]); struts = []
    for i, q in enumerate(N):                                      # wing struts (head_mount.py: 20 deg+ from the node's tubes)
        if abs(q[1]) > 150 and q[0] > 300 and var != "Z":
            w = W0 * [1, np.sign(q[1]), 1]; d = (w - q) / np.linalg.norm(w - q)
            tub = [N[b if a == i else a] - q for a, b in CG["rods"] if i in (a, b)]
            if all(math.degrees(math.acos(np.clip(d @ (t / np.linalg.norm(t)), -1, 1))) >= 20 for t in tub):
                sec = cf.sec_of(f"wing strut {i}") if var == "A" else TUBE6
                Wn = f.node(w); f.member(H, Wn, RIGID, "head-wing"); f.member(Wn, end(i, w), sec, f"wing strut {i}")
                L = np.linalg.norm(w - q); carbon += L * (24.3 if sec is TUBE6 else 10.5) / 1000; add_m(nid[i], L * 24.3e-3 / 2); struts.append(i)
    tops = []
    if var == "A" and os.environ.get("V5_WS"):                         # the one-piece head's struts (cage_frame.HEAD_STRUTS)
        for w, q in cf.HEAD_STRUTS:
            for sy in (1, -1):
                w_, i = np.array(w) * [1, sy, 1], int(np.argmin(np.linalg.norm(N - np.array(q) * [1, sy, 1], axis=1)))
                Wn = f.node(w_); f.member(H, Wn, RIGID, "head-strut"); f.member(Wn, end(i, w_), TUBE6, f"head strut {i}")
                L = np.linalg.norm(w_ - N[i]); carbon += L * 24.3 / 1000; add_m(nid[i], L * 24.3e-3 / 2); struts.append(i)
    if var in ("C", "D"):                                          # guard ring top (8 behind the plane, r 62) to the cage top nodes
        for i in (TOP if var == "C" else [i for i in TOP if abs(N[i, 1]) < 1]):
            r0 = B - A * 8.0 + UPB * 62.0 + np.array([0, 25.0 * np.sign(N[i, 1]) if abs(N[i, 1]) > 1 else 0, 0])
            Rn = f.node(r0); f.member(H, Rn, RIGID, "head-ring"); f.member(Rn, end(i, r0), TUBE6, f"top strut {i}")
            L = np.linalg.norm(r0 - N[i]); carbon += L * 24.3 / 1000; add_m(nid[i], L * 24.3e-3 / 2); tops.append((i, round(float(L))))
    if var == "F":                                                 # every reachable node: centre top, the two top-side front nodes
        hub = {7: B - A * 8.0 + UPB * 62.0, 2: B - A * 8.0 - UPB * 62.0}   # (from the guard ring's sides), the loop's front node (from its bottom)
        for i in (15, 16): hub[i] = B - A * 8.0 + UPB * 30.0 + np.array([0, 55.0 * np.sign(N[i, 1]), 0])
        for i, r0 in hub.items():
            Rn = f.node(r0); f.member(H, Rn, RIGID, "head-ring"); f.member(Rn, end(i, r0), TUBE6, f"top strut {i}")
            L = np.linalg.norm(r0 - N[i]); carbon += L * 24.3 / 1000; add_m(nid[i], L * 24.3e-3 / 2); tops.append((i, round(float(L))))
    return f, nid, H, Lc, mass, dict(carbon_g=carbon, struts=struts, tops=tops)


def cases(f, nid, H, Lc, mass):
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    guard = np.array(HS["guard_nodes"]); Fc = M_DRONE * G * CRASH; Fh = (M_L2 + 2 * M_CAM + M_MOUNT) / 1000 * G * CRASH
    cs = {}
    for k, d in enumerate(dirs):
        sup = N @ d; i = int(np.argmax(sup))
        if (guard @ d).max() > sup[i] + 3.0: cs[f"nose hit {k}"] = {H: -d * Fc}
        else: cs[f"crash {k} via node {i}"] = {nid[i]: -d * Fc}
        cs[f"head inertia {k}"] = {Lc: d * Fh}
    cs["flight 4 g"] = {i: np.array([0, 0, -m * 1e-3 * G * FLIGHT]) for i, m in mass.items()}
    return cs


def head_modes(f, H):
    """the head as a rigid body (L2 230 g at its CG with its own 75-cube inertia, cameras, mount at H) on the frame's 6 x 6
    stiffness at H (static condensation: unit forces and moments at H); returns its 6 frequencies (Hz)"""
    C = np.zeros((6, 6))
    for k in range(6):
        e = np.zeros(6); e[k] = 1.0; u = f.solve({H: e}); C[:, k] = u[6 * H:6 * H + 6]
    K = np.linalg.inv((C + C.T) / 2); Mm = np.zeros((6, 6)); h0 = f.X[H]
    sk = lambda r: np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    for p, g in [(L2CG, M_L2)] + [(c, M_CAM) for c in CAMS] + [(h0, M_MOUNT)]:
        m = g * 1e-6; r = np.asarray(p) - h0; S = sk(r)
        Mm[:3, :3] += m * np.eye(3); Mm[:3, 3:] += -m * S; Mm[3:, :3] += m * S; Mm[3:, 3:] += -m * S @ S
    Mm[3:, 3:] += np.eye(3) * M_L2 * 1e-6 * 2 * 75.0 ** 2 / 12
    from scipy.linalg import eigh
    w2 = eigh(K, Mm, eigvals_only=True)
    return [round(float(x), 1) for x in np.sqrt(np.maximum(w2, 0)) / (2 * np.pi)]


def run(var, soft):
    f, nid, H, Lc, mass, info = build(var, soft)
    worst = dict(buckle=(0, "", ""), sigma=(0, "", "")); over = {}
    defl = {}
    for cn, loads in cases(f, nid, H, Lc, mass).items():
        u = f.solve(loads)
        defl[cn] = float(np.linalg.norm(u[6 * Lc:6 * Lc + 3]))
        for r in f.member_forces(u):
            if r["name"].startswith(("head-", "tpu")): continue
            if r["buckle"] > worst["buckle"][0]: worst["buckle"] = (r["buckle"], r["name"], cn)
            if r["buckle"] >= 1.0: over[r["name"]] = max(over.get(r["name"], 0), round(float(r["buckle"]), 2))
            if r["sigma"] > worst["sigma"][0]: worst["sigma"] = (r["sigma"], r["name"], cn)
    head_mode = head_modes(f, H)[0]
    hz, U = f.modes(mass, 3)                                       # the frame's own lowest modes (cage nodes rattling on soft joints)
    dn = {k: v for k, v in defl.items() if k.startswith("nose")}; di = {k: v for k, v in defl.items() if k.startswith("head inertia")}
    return dict(var=var, soft=soft, buckle=[round(worst["buckle"][0], 2), worst["buckle"][1], worst["buckle"][2]],
                sigma=[round(worst["sigma"][0]), worst["sigma"][1], worst["sigma"][2]],
                nose_defl=round(max(dn.values()), 2) if dn else None, inertia_defl=round(max(di.values()), 2),
                frame_modes_hz=[round(float(h), 1) for h in hz], head_mode_hz=head_mode, over=over, **info)


TITLE = {"A": "keels + braces (v5)", "B": "cage only", "C": "cage only + 3 top struts", "D": "cage only + centre top strut", "F": "cage only + 4 struts", "G": "2 keels only", "H": "4 braces only", "Z": "front node only"}


def picture(res, path):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 4, figsize=(22, 10)); axs = axs.ravel()
    for ax, var in zip(axs, "ABCDFGHZ"):
        f, nid, H, Lc, mass, info = build(var, False)
        for m in f.M:
            a, b = f.X[m["a"]], f.X[m["b"]]; nm = m["name"]
            if nm.startswith("head-"): c, lw = "#888", 0.8
            elif nm.startswith("tube"): c, lw = "#4a6fa5", 1.0
            elif nm.startswith(("keel", "L2 brace")): c, lw = "#d1495b", 2.4
            elif nm.startswith(("wing strut", "top strut")): c, lw = "#2a9d8f", 2.4
            else: c, lw = "#aaa", 0.8
            ax.plot([a[0], b[0]], [a[2], b[2]], color=c, lw=lw)
        ax.add_patch(plt.Rectangle((B[0], B[2] - 37), 75, 75, angle=-15, color="#333", alpha=0.6))
        for i in f.fixed: ax.plot(f.X[i][0], f.X[i][2], "k^", ms=4)
        r = [x for x in res if x["var"] == var]
        rr, rs = [x for x in r if not x["soft"]][0], [x for x in r if x["soft"]][0]
        ax.set_title(f"{var}: {TITLE[var]}\n"
                     f"buckle {rr['buckle'][0]} / {rs['buckle'][0]}, head mode {rr['head_mode_hz']} / {rs['head_mode_hz']} Hz (rigid / TPU)",
                     fontsize=9)
        ax.set_aspect("equal"); ax.set_xlim(-100, 480); ax.set_ylim(-120, 200); ax.set_xlabel("x (side view)")
    axs[0].plot([], [], color="#d1495b", lw=2.4, label="keels, L2 braces (to the core)")
    axs[0].plot([], [], color="#2a9d8f", lw=2.4, label="head to cage struts")
    axs[0].plot([], [], color="#4a6fa5", label="cage tubes"); axs[0].plot([], [], "k^", label="core (fixed)")
    axs[0].legend(loc="lower left", fontsize=7)
    fig.suptitle("Overseer: L2 head held from the core vs by the cage (frame model F on E, cage v5)")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    res = [run(v, s) for v in "ABCDFGHZ" for s in (False, True)]
    json.dump(res, open(os.path.join(OUT, "result.json"), "w"), indent=1)
    picture(res, os.path.join(OUT, "concept.png"))
    for r in res: print(r)


if __name__ == "__main__":
    main()
