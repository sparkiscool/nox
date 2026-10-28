"""Overseer sensor head on at most 3 carbon rods (Spark 27 Oct: "i want to reduce the number of rods going to the lidar to a max of 3").

System pass for the head's supports (F on E, nox/frame_model.py): the cage v5 shell (PETG node stubs) and its links on the core, the
head a rigid body (the L2 at its CG with its inertia, both cameras, the mount) that holds no cage node (node 17 stays a cage node,
so its 4 tubes are not head rods). Every mirror-symmetric 3-rod set: one rod in the centre plane and a mirrored pair, each from a
point on the head (behind the L2 base plane, in the print box) to the core (the hub deck tower) or a cage node. Each rod is a
6/4 carbon tube epoxied at both ends (rigid ends; the PETG stub version for the chosen one). A rod must stay behind the L2 base
plane, out of both camera views, 12 off the prop disks and clear of the cage tubes it does not end on.

Ranked by the head's lowest mode (target >= 100 Hz, dimensions.md L); also the L2 deflection and the worst rod buckling under the
head's own 35 g in 14 directions. The cage elsewhere is condensed onto the candidate cage nodes once (Schur complement), so each
set solves in a few small matrices.

python3 nox/overseer/head_rods.py -> runs/overseer_head_rods/v1/ (result.json, summary.md, options.png)
"""
import os, sys, json, math, itertools
import numpy as np
from scipy.sparse.linalg import splu
from scipy.linalg import eigh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
from frame_model import Frame, tube
import cage_frame as cf

OUT = os.path.join(ROOT, "runs", "overseer_head_rods", "v1")
CG = json.load(open(os.path.join(HERE, "cage_v5_shell.json"))); N = np.array(CG["nodes"])
RES = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "result.json")))
HS = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "head_system.json")))
B = np.array(HS["l2"]["base"]); A = np.array(HS["l2"]["axis"]); UPB = np.array([-A[2], 0, A[0]])
L2CG = np.array(HS["l2"]["cg"]); CAMS = [np.array(c["cg"]) for c in HS["cameras"]]
TUBE6 = cf.TUBE6
JOINT = dict(tube(11.3, 0.0, E=2000.0), G=2000.0 / 3.0)            # PETG node stub (E), as head_cage_concept with V5_JOINT_E=2000
SET = 6.0
M_L2, M_CAM, M_MOUNT = 230.0, 15.0, float(os.environ.get("V5_HEAD_MOUNT_G", HS["mass_budget_g"]))     # g (P, E, E)
G, CRASH = 9.81, 35.0
PROP_GAP, CAM_EXTRA = 12.0, 2.0
_KO = None
BOSS_R = 6.2 / 2 + 2.5                                            # the head's socket boss (head_mount.socket, 6/4 tube)
HEAD_DEPTH = float(os.environ.get("V5_HEAD_DEPTH", "12"))     # head rod socket depth: rod forces <= 126 N (F), epoxy shear ~0.6 MPa at 12 (E)
H0 = B - A * 20.0                                                  # the head's reference point (as cage_frame)

# candidate rod ends on the head: behind the L2 base plane by 6, (u along UPB, y) (C)
HEAD_PTS = {f"c u{u:+.0f}": (u, 0.0) for u in (-40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 85.0)}
HEAD_PTS.update({f"c u{u:+.0f} b{b:.0f}": (u, 0.0, b) for u in (-40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 85.0) for b in (9.0, 12.0, 18.0, 24.0)})
HEAD_PTS.update({f"s u{u:+.0f} y{y:.0f}": (u, y) for u in (-30.0, -10.0, 10.0, 30.0, 50.0, 70.0, 90.0, 100.0, 110.0) for y in (30.0, 50.0, 70.0, 85.0)})
HEAD_PTS.update({f"s u{u:+.0f} y{y:.0f} b{b:.0f}": (u, y, b) for u in (60.0, 80.0, 90.0, 100.0, 110.0) for y in (50.0, 60.0, 70.0, 80.0) for b in (9.0, 12.0, 18.0, 24.0)})   # deeper behind the plane (the sockets' bosses stay behind it)
MIN_ROD = 33.0                                                     # carbon break-even (CLAUDE.md): a shorter rod is printed, so its node joins the head
CORE_PTS = {"tower front": (114.0, 0.0, 20.0), "tower top": (97.0, 0.0, 24.0), "brace lo": (97.0, 26.0, 12.0), "brace hi": (97.0, 14.0, 18.0)}
CAGE_C = [i for i in range(len(N)) if N[i, 0] > 150 and (abs(N[i, 1]) < 0.5 or N[i, 1] > 0)]   # front cage nodes (y >= 0 designs)


def hp(name, sy=1):
    v = HEAD_PTS[name]; u, y = v[0], v[1]; back = v[2] if len(v) > 2 else 6.0
    return B - A * back + UPB * u + np.array([0, y * sy, 0])


def cage_frame_k():
    """the cage + links on the core (no head): stiffness condensed onto the candidate cage nodes and their mirrors"""
    f = Frame(); nid = [f.node(p) for p in N]
    def end(i, other):
        d = other - N[i]; p = N[i] + d / np.linalg.norm(d) * SET
        j = f.node(p, tol=1e-3); f.member(nid[i], j, JOINT, f"joint {i}"); return j
    for a, b in CG["rods"]: f.member(end(a, N[b]), end(b, N[a]), TUBE6, f"tube {a}-{b}")
    for k, (p, q) in RES["links"].items():
        if k.startswith(("L2 brace", "keel", "head strut")): continue
        p, q = np.array(p), np.array(q)
        on = [i for i in range(len(N)) if np.linalg.norm(N[i] - p) < 0.6 or np.linalg.norm(N[i] - q) < 0.6]
        if on:
            i = on[0]; other = q if np.linalg.norm(N[i] - p) < 0.6 else p; C_ = f.node(other)
            f.member(end(i, other), C_, cf.sec_of(k), k); f.fix(C_)
    keep = sorted({i for i in CAGE_C} | {int(np.argmin(np.linalg.norm(N - N[i] * [1, -1, 1], axis=1))) for i in CAGE_C})
    K = f.stiffness().tocsc(); n = len(f.X)
    free = np.array([d for d in range(6 * n) if d // 6 not in f.fixed])
    kd = np.array([6 * nid[i] + c for i in keep for c in range(6)]); ks = set(kd.tolist())
    idd = np.array([d for d in free if d not in ks])
    Kff = K[idd][:, idd] + 1e-10 * np.median(K.diagonal()) * __import__("scipy.sparse", fromlist=["eye"]).eye(len(idd))
    Kic = K[idd][:, kd].toarray(); Kcc = K[kd][:, kd].toarray()
    X = splu(Kff.tocsc()).solve(Kic)
    Kc = Kcc - Kic.T @ X
    return (Kc + Kc.T) / 2, keep


def beam_k(pa, pb, sec):
    f = Frame(); a = f.node(pa); b = f.node(pb); f.member(a, b, sec, "r"); Kg, *_ = f._k(f.M[0]); return Kg


def rigid_T(p):
    """6 dof at point p from the head's 6 at H0 (small rotations)"""
    r = np.asarray(p) - H0; T = np.eye(6)
    T[:3, 3:] = -np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    return T


def head_mass():
    sk = lambda r: np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]]); M = np.zeros((6, 6))
    for p, g in [(L2CG, M_L2)] + [(c, M_CAM) for c in CAMS] + [(H0, M_MOUNT)]:
        m = g * 1e-6; S = sk(np.asarray(p) - H0)
        M[:3, :3] += m * np.eye(3); M[:3, 3:] += -m * S; M[3:, :3] += m * S; M[3:, 3:] += -m * S @ S
    M[3:, 3:] += np.eye(3) * M_L2 * 1e-6 * 2 * 75.0 ** 2 / 12
    return M


# ---------------------------------------------------------------- the rod checks (geometry, C)
def l2_cable_hit(pts, margin=1.0):
    """head_mount.centre_bodies' L2 cable keep-out: box (-50..-32, -8..8, -40..2) in the L2 frame (T_l2, TILT 105)"""
    import head_mount as hm
    R = hm.T_l2()[:3, :3]; loc = (np.asarray(pts) - B) @ R           # drone -> L2 frame (R orthonormal; the L2's roll included)
    lo, hi = np.array([-50, -8, -40.0]) - margin, np.array([-32, 8, 2.0]) + margin
    return ((loc >= lo) & (loc <= hi)).all(1)


def cam_frustum_hit(pts):
    hit = np.zeros(len(pts), bool)
    for c in HS["cameras"]:
        lens, f, u, r = (np.array(c[k], float) for k in ("lens", "f", "u", "r"))
        ha, hv = math.radians(c["half_across"] + CAM_EXTRA), math.radians(c["half_up"] + CAM_EXTRA)
        d = pts - lens; z = d @ f
        hit |= (z > -2.5) & (np.abs(d @ r) <= z * math.tan(ha) + 4) & (np.abs(d @ u) <= z * math.tan(hv) + 4)
    return hit


def props():
    import trimesh
    out = []
    for k in range(1, 5):
        m = trimesh.load(os.path.join(ROOT, "runs", "overseer_layout", "v5", "parts", f"prop_{k}.stl"), force="mesh")
        lo, hi = m.bounds; out.append(((lo + hi) / 2, (hi[0] - lo[0]) / 2, (hi[2] - lo[2]) / 2))
    return out


PROPS = props()


def rod_ok(p, q, far_node=None):
    if np.linalg.norm(q - p) < MIN_ROD: return False, "under the 33 break-even"
    s = p[None] + np.linspace(0, 1, 80)[:, None] * (q - p)[None]
    if ((s - B) @ A > -(3.0 + 1.0)).any(): return False, "L2 field"
    if cam_frustum_hit(s).any(): return False, "camera view"
    d_ = (q - p) / np.linalg.norm(q - p); x_ = np.cross(d_, [0, 0, 1.0]); x_ = x_ / np.linalg.norm(x_) if np.linalg.norm(x_) > 1e-6 else np.array([1.0, 0, 0]); y_ = np.cross(d_, x_)
    boss = np.array([p + d_ * t + (x_ * np.cos(a) + y_ * np.sin(a)) * (BOSS_R + 1.0) for t in np.linspace(-(HEAD_DEPTH + 3.0 + 1.0), 1.0, 9) for a in np.linspace(0, 2 * np.pi, 16, endpoint=False)])
    if cam_frustum_hit(boss).any(): return False, "socket boss in a camera view"
    if ((boss - B) @ A > -1.0).any(): return False, "socket boss in the L2 field"     # (v5 sweep: the L2 plane cut the side bosses to a 1.2 wall)
    if l2_cable_hit(np.vstack([s, boss])).any(): return False, "the L2 cable exit"     # (28 Oct: the centre boss sat in the cable keep-out)
    import head_mount as hm                                            # the head's own keep-outs (exact): the boss must lose nothing to them
    global _KO
    if _KO is None: _KO = hm.head_keepouts()
    bo, cut_, keep_ = hm.socket(p, q, hm.TUBE_BORE, HEAD_DEPTH, wall=2.5)
    if (bo ^ _KO).volume() > 1.0: return False, "socket boss clipped by a head keep-out"
    for c, R, hz in PROPS:
        dr = np.hypot(s[:, 0] - c[0], s[:, 1] - c[1]) - R; dz = np.abs(s[:, 2] - c[2]) - hz
        if (np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) < PROP_GAP).any(): return False, "prop"
    for a, b in CG["rods"]:                                          # cage tubes it does not end on
        if far_node is not None and far_node in (a, b): continue
        pa, pb = N[a], N[b]; d = pb - pa; t = np.clip(((s - pa) @ d) / (d @ d), 0, 1)
        if (np.linalg.norm(s - (pa + t[:, None] * d), axis=1) < 3.0 + 3.0 + 1.0).any(): return False, f"tube {a}-{b}"
    return True, ""


def rods_all():
    """(name, head point, far point, far cage node or None, side) for every candidate rod, y >= 0 side and centre"""
    out = []
    for hn, v_ in HEAD_PTS.items():
        u, y = v_[0], v_[1]; p = hp(hn)
        for cn, c in CORE_PTS.items():
            q = np.array(c)
            if (y == 0) == (c[1] == 0): out.append((f"{hn} -> core {cn}", p, q, None, "centre" if y == 0 else "side"))
        for i in CAGE_C:
            if (y == 0) == (abs(N[i, 1]) < 0.5): out.append((f"{hn} -> node {i}", p, N[i], i, "centre" if y == 0 else "side"))
    return out


def evaluate(rods, Kc, keep, M, soft=False, merged=()):
    """rods: [(name, p, q, cage node or None)] full set (mirrors included): head modes (Hz), L2 deflection and rod forces at 35 g.
    merged: cage nodes printed into the head (their 6 dof follow the head rigidly; Spark 28 Oct: node 17 in the head print)"""
    cn = sorted({r[3] for r in rods if r[3] is not None} | set(merged)); kidx = [keep.index(i) for i in cn]
    sel = np.array([6 * j + c for j in kidx for c in range(6)], int); oth = np.array([d for d in range(len(Kc)) if d not in set(sel.tolist())], int)
    Ks = Kc[np.ix_(sel, sel)] - (Kc[np.ix_(sel, oth)] @ np.linalg.solve(Kc[np.ix_(oth, oth)], Kc[np.ix_(oth, sel)]) if len(oth) else 0)
    n = 6 + len(sel); K = np.zeros((n, n)); K[6:, 6:] = Ks
    mats = []
    for nm, p, q, i in rods:
        if soft:                                                     # PETG stubs at both ends: rod L - 2 SET + 2 stubs (E)
            d = (q - p) / np.linalg.norm(q - p); Kr = chain_k(p, q, d)
        else:
            Kr = beam_k(p, q, TUBE6)
        Ta = rigid_T(p); Tm = np.zeros((12, n)); Tm[:6, :6] = Ta
        if i is not None: j = cn.index(i); Tm[6:, 6 + 6 * j:12 + 6 * j] = np.eye(6)
        K += Tm.T @ Kr @ Tm; mats.append((nm, Tm, p, q))
    if merged:                                                         # slave each merged node to the head: u_node = T(node) u_H
        free = [j for j, i in enumerate(cn) if i not in merged]; nr = 6 + 6 * len(free); Ts = np.zeros((n, nr)); Ts[:6, :6] = np.eye(6)
        for j, i in enumerate(cn):
            if i in merged: Ts[6 + 6 * j:12 + 6 * j, :6] = rigid_T(N[i])
            else: f_ = free.index(j); Ts[6 + 6 * j:12 + 6 * j, 6 + 6 * f_:12 + 6 * f_] = np.eye(6)
        K = Ts.T @ K @ Ts; mats = [(nm, Tm @ Ts, p, q) for nm, Tm, p, q in mats]; n = nr
    # the head's 6 dof: condense the cage nodes out
    if n > 6: KH = K[:6, :6] - K[:6, 6:] @ np.linalg.solve(K[6:, 6:], K[6:, :6])
    else: KH = K
    KH = (KH + KH.T) / 2
    w2 = eigh(KH, M, eigvals_only=True); hz = np.sqrt(np.maximum(w2, 0)) / (2 * np.pi)
    # head inertia 35 g in 14 directions: L2 CG deflection and the largest rod axial force (rigid-end rods only)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((a, b, c), float) / math.sqrt(3) for a in (1, -1) for b in (1, -1) for c in (1, -1)]
    mt = (M_L2 + 2 * M_CAM + M_MOUNT) / 1000
    cgm = (L2CG * M_L2 + sum(c * M_CAM for c in CAMS) + H0 * M_MOUNT) / (M_L2 + 2 * M_CAM + M_MOUNT)
    Tcg = rigid_T(cgm); dmax = 0.0; nmax = 0.0
    for d in dirs:
        F6 = Tcg.T @ np.concatenate([d * mt * G * CRASH, np.zeros(3)])
        F = np.zeros(n); F[:6] = F6
        u = np.linalg.solve(K + 1e-9 * np.eye(n) * np.median(np.diag(K)), F)
        dmax = max(dmax, float(np.linalg.norm((rigid_T(L2CG) @ u[:6])[:3])))
        if not soft:
            for nm, Tm, p, q in mats:
                ue = Tm @ u; ax = (q - p) / np.linalg.norm(q - p)
                nmax = max(nmax, abs(float(TUBE6["E"] * TUBE6["A"] / np.linalg.norm(q - p) * ((ue[6:9] - ue[:3]) @ ax))))
    return [round(float(h), 1) for h in hz], round(dmax, 3), round(nmax, 1)


def chain_k(p, q, d):
    """a rod with a PETG stub at each end, condensed to its two end points (12 x 12)"""
    f = Frame(); a = f.node(p); a2 = f.node(p + d * SET); b2 = f.node(q - d * SET); b = f.node(q)
    f.member(a, a2, JOINT, "j"); f.member(a2, b2, TUBE6, "r"); f.member(b2, b, JOINT, "j")
    K = f.stiffness().toarray(); keep = list(range(6)) + list(range(18, 24)); mid = list(range(6, 18))
    return K[np.ix_(keep, keep)] - K[np.ix_(keep, mid)] @ np.linalg.solve(K[np.ix_(mid, mid)], K[np.ix_(mid, keep)])


# ---------------------------------------------------------------- cage first for the L2 and both cameras (outside their field cones)
def _fib(n=2006):
    i = np.arange(n) + 0.5; ph = np.arccos(1 - 2 * i / n); th = np.pi * (1 + 5 ** 0.5) * i
    return np.c_[np.cos(th) * np.sin(ph), np.sin(th) * np.sin(ph), np.cos(ph)]


DIRS = _fib()


def guard_setup():
    """support functions (max d.x) of the base cage (shell tubes and nodes, links other than the head's) and of each guarded part,
    over the directions outside its field cone (system_check's openings): margin = cage - part, needed >= 3 (C)"""
    import trimesh
    P = lambda f: np.asarray(trimesh.load(os.path.join(ROOT, "runs", "overseer_layout", "v5", "parts", f), force="mesh").vertices)
    base = np.vstack([P("cage_shell_tubes.stl"), P("cage_shell_nodes.stl")])
    for k, (p, q) in RES["links"].items():
        if k.startswith(("head rod", "head strut", "L2 brace", "keel")): continue
        p, q = np.array(p), np.array(q); base = np.vstack([base, p + np.linspace(0, 1, 40)[:, None] * (q - p)])
    hb = (base @ DIRS.T).max(0) + 1.5                                   # + a 3.0 rod's radius at least (C, conservative for 6 mm)
    asm = json.load(open(os.path.join(HERE, "overseer.assembly.json"))); op = {o["part"]: o for o in asm["openings"]}
    parts = {}
    for nm, f in (("Unitree L2", "unitree_l2.stl"), ("camera left", "camera_left.stl"), ("camera right", "camera_right.stl")):
        o = op[nm]; ax = np.array(o["axis"], float); ax /= np.linalg.norm(ax)
        out = np.degrees(np.arccos(np.clip(DIRS @ ax, -1, 1))) > o["half_angle_deg"]
        parts[nm] = ((P(f) @ DIRS.T).max(0), out)
    return hb, parts


def guard_margins(rods, G_):
    hb, parts = G_; h = hb.copy()
    for nm, p, q, i in rods:
        s_ = p[None] + np.linspace(0, 1, 40)[:, None] * (q - p)[None]; h = np.maximum(h, (s_ @ DIRS.T).max(0) + 3.0)
    return {nm: round(float((h - hp_)[out].min()), 2) for nm, (hp_, out) in parts.items()}


def mirror(r):
    nm, p, q, i, s = r
    mi = None if i is None else int(np.argmin(np.linalg.norm(N - N[i] * [1, -1, 1], axis=1)))
    return (nm + " m", p * [1, -1, 1], q * [1, -1, 1], mi)


def main():
    os.makedirs(OUT, exist_ok=True)
    Kc, keep = cage_frame_k(); M = head_mass()
    cand = []; rej = {}
    for r in rods_all():
        ok, why = rod_ok(r[1], r[2], r[3])
        if ok: cand.append(r)
        else: rej[r[0]] = why
    cen = [r for r in cand if r[4] == "centre"]; sid = [r for r in cand if r[4] == "side"]
    res = []; G_ = guard_setup()
    for c in cen:
        for s in sid:
            rods = [c[:4], s[:4], mirror(s)]
            gm = guard_margins(rods, G_)
            hz, dl2, nmax = evaluate(rods, Kc, keep, M)
            L = sum(np.linalg.norm(r[2] - r[1]) for r in rods)
            res.append(dict(centre=c[0], pair=s[0], hz=hz, l2_defl_mm=dl2, rod_N_max=nmax, carbon_mm=round(float(L), 0), guard_mm=gm,
                            guarded=bool(min(gm.values()) >= 3.0)))
    res.sort(key=lambda r: (not r["guarded"], -r["hz"][0]))
    top = res[:5]
    for r in top:                                                    # the PETG-stub (soft joint) number for the leaders
        rods = [next(c[:4] for c in cen if c[0] == r["centre"]), next(s[:4] for s in sid if s[0] == r["pair"])]
        rods.append(mirror(next(s for s in sid if s[0] == r["pair"])))
        r["hz_petg_joints"], r["l2_defl_petg_mm"], _ = evaluate(rods, Kc, keep, M, soft=True)
    # the present 12-rod head for reference: 4 braces, 4 struts, node 17's 4 tubes (cage_frame's head; concept A + struts: 208 / 187 Hz)
    json.dump(dict(note="mirror-symmetric 3-rod sets (one centre rod + a mirrored pair), rigid ends; hz_petg_joints with PETG stubs (F on E)",
                   candidates=len(cand), rejected=rej, sets=res), open(os.path.join(OUT, "result.json"), "w"), indent=1)
    for r in res[:12]:
        print(r["hz"][0], r.get("hz_petg_joints", [None])[0], r["l2_defl_mm"], r["rod_N_max"], r["carbon_mm"], r["guard_mm"], "|", r["centre"], "+", r["pair"])
    print("guarded sets:", sum(r["guarded"] for r in res))
    print(len(cand), "candidate rods,", len(res), "sets")
    return res, cen, sid


def main_node17():
    """Spark 28 Oct: "i want the back of the camera mount to just be connected to that node since they are so close anyways and get rid
    of that tiny rod on the mount and just make all that a printed structure": node 17 printed into the head (its 4 cage tubes end
    in the head), no centre rod; the side pair searched as before, and node 17 alone for reference"""
    Kc, keep = cage_frame_k(); M = head_mass(); G_ = guard_setup(); out = []
    sid = []
    for r in rods_all():
        if r[4] != "side": continue
        ok, why = rod_ok(r[1], r[2], r[3])
        if ok: sid.append(r)
    hz, dl2, _ = evaluate([], Kc, keep, M, merged=(17,)); hzs, dls, _ = evaluate([], Kc, keep, M, soft=True, merged=(17,))
    out.append(dict(pair="none (node 17 only)", hz=hz, hz_petg_joints=hzs, l2_defl_mm=dl2, guard_mm=guard_margins([], G_)))
    for s_ in sid:
        rods = [s_[:4], mirror(s_)]
        hz, dl2, nmax = evaluate(rods, Kc, keep, M, merged=(17,)); gm = guard_margins(rods, G_)
        out.append(dict(pair=s_[0], hz=hz, l2_defl_mm=dl2, rod_N_max=nmax, guard_mm=gm, guarded=bool(min(gm.values()) >= 3.0),
                        carbon_mm=round(float(2 * np.linalg.norm(s_[2] - s_[1])), 0)))
    out[1:] = sorted(out[1:], key=lambda r: (not r["guarded"], -r["hz"][0]))
    for r in out[1:4]:
        s_ = next(x for x in sid if x[0] == r["pair"])
        r["hz_petg_joints"], r["l2_defl_petg_mm"], _ = evaluate([s_[:4], mirror(s_)], Kc, keep, M, soft=True, merged=(17,))
    json.dump(dict(note="node 17 printed into the head (Spark 28 Oct), no centre rod; mirror side pairs; rigid / PETG joints (F on E)", sets=out),
              open(os.path.join(OUT, "result_node17.json"), "w"), indent=1)
    for r in out[:8]:
        print(r["hz"][0], r.get("hz_petg_joints", [None])[0], r["l2_defl_mm"], r.get("guard_mm"), r["pair"], r.get("carbon_mm"))


if __name__ == "__main__":
    main_node17() if "node17" in sys.argv[1:] else main()
