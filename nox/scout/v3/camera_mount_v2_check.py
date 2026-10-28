"""Scout camera mount v2, after the topology run: the checks, the frame model with the as-built mount, the print file and the
system recheck. Numbers to <run>/checks.json (and merged into <run>/result.json under "checks").

  view    nothing of the mount in the camera's field (point test, + 2 deg) and with the lens as a disk of r 6 (E); hood lead
  fit     clearance to every cage rod (all 3.0) and node, the props' swept disks and wash prisms, the camera envelope
  stroke  the push-on stroke (110 along -x): every member clear but rod 14-15, and that one only by the snap at the clips
  snap    the slit-fork clips (camera_mount_v2.fork_snap)
  frame   the v12 cage (every rod 3.0) + the mount as a super-element: its stiffness on 9 interface points (4 camera seats, 2
          clips on rod 14-15, hood centre, hood corner, mid-tie) from the as-built voxel FE (nox/topo fem, the foot clamped:
          bolt pads fixed, contact plate z), flexibility method; the camera + the mount's front one rigid body with real
          inertia (v1's rigid_body_points), the tie's mass at mid-tie. Modes, hood hits, 35 g crash, flight, breaking order.
  print   print pose (+y face down: the clip prongs bend in the layers), solid_check on the exact print file
  system  the Scout manifest with v2 in place of v1 (scout_with_mount.assembly.json), system_check

python3 nox/scout/v3/camera_mount_v2_check.py [run_dir]
"""
import os, sys, json, math, subprocess, copy
import numpy as np, trimesh, manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, HERE)
import camera_mount_v2 as cm
import cage_frame as cf
from frame_model import Frame
from scipy.sparse import csr_matrix, coo_matrix

RUN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_camera_mount", "v2")
NAME = "scout_camera_mount"
RHO = 1.27e-3
N = cm.N
PRINT = os.path.join(ROOT, "print", "scout_camera_mount_v2.stl")
MU, PRELOAD, INSERT_PULL = 0.3, 300.0, 600.0            # PETG on carbon, M3 preload into a heat-set insert, insert pull-out (E; dimensions Sled row 600 N)
PETG_FAIL = 51.0 * 0.85                                 # in-layer strength x knockdown (nox/topo material, E)
SIG_T, SIG_C = 1000.0, 600.0                            # carbon allowables (cage_frame, E)


def tmesh(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)


def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def seg_dist(P, a, b):
    ab = b - a; L2 = float(ab @ ab)
    t = np.clip(((P - a) @ ab) / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(P))
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def members():
    """(name, a, b, r) every cage rod (3.0), saddle tie, and node (a = b, r node radius)"""
    out = [(f"rod {a}-{b}", N[a], N[b], cm.R_T) for a, b in cm.CG["rods"]]
    out += [(k, np.array(p, float), np.array(q, float), cm.CG.get("link_r_mm", 1.5)) for k, (p, q) in cm.RES["links"].items()]
    out += [(f"node {i}", N[i], N[i], 10.0 if i < cm.NR else cm.CG["node_r_mm"]) for i in range(len(N))]
    return out


# ---------------------------------------------------------------- view / fit / stroke / props
def in_view(P, ap=0.0):
    d = P - cm.LENS; dx = d[:, 0]
    dy = np.maximum(np.abs(d[:, 1]) - ap, 0); dz = np.maximum(np.abs(d[:, 2]) - ap, 0)
    el = np.degrees(np.arctan2(dz, np.maximum(dx, 1e-6))); az = np.degrees(np.arctan2(dy, np.maximum(dx, 1e-6)))
    return (dx > 0) & (el < cm.VFOV / 2 + cm.VIEW_MARGIN) & (az < cm.HFOV / 2 + cm.VIEW_MARGIN), el, az


def view_report(P, mesh):
    s0, el, az = in_view(P); s1, el2, az2 = in_view(P, cm.APERTURE_R); fr = P[:, 0] > cm.LENS[0]
    m0 = float(np.min(np.maximum(el[fr] - cm.VFOV / 2, az[fr] - cm.HFOV / 2))) if fr.any() else 90.0
    m1 = float(np.min(np.maximum(el2[fr] - cm.VFOV / 2, az2[fr] - cm.HFOV / 2))) if fr.any() else 90.0
    return dict(in_view_points=int(s0.sum()), in_view_aperture=int(s1.sum()), cone_margin_deg_point=round(m0, 1), cone_margin_deg_aperture=round(m1, 1),
                hood_lead_mm=round(float(mesh.bounds[1][0] - cm.LENS[0]), 2))


def fit_report(P, mesh):
    out = {}
    for nm, a, b, r in members():
        if nm == "rod 14-15": continue
        out[nm] = float(seg_dist(P, a, b).min() - r)
    w = sorted(out.items(), key=lambda kv: kv[1])[:5]
    d1415 = seg_dist(P, N[14], N[15]) - cm.R_T
    lo, hi = np.array([c[0] for c in cm.CAM_BOX]), np.array([c[1] for c in cm.CAM_BOX])
    inside_cam = int(((P > lo + 0.01) & (P < hi - 0.01)).all(1).sum())
    wash = 0
    for (mx, my) in cm.MOTORS:
        wash += int(((np.hypot(P[:, 0] - mx, P[:, 1] - my) < cm.PROP_R)).sum())
    wash_pts = np.vstack([P[np.hypot(P[:, 0] - mx, P[:, 1] - my) < cm.PROP_R] for (mx, my) in cm.MOTORS])
    props = {}
    for k in range(1, 5):
        q = trimesh.load(os.path.join(cm.LAY, "parts", f"prop_{k}.stl"), force="mesh")
        props[f"prop_{k}"] = round(float(trimesh.proximity.closest_point(q, P[::9])[1].min()), 1)
    return dict(member_clearance_min_mm=round(w[0][1], 2), member_clearance_worst=[(k, round(v, 2)) for k, v in w],
                rod_14_15_min_mm=round(float(d1415.min()), 3), points_in_camera_box=inside_cam, props_mm=props,
                wash_points=wash, wash_max_z=round(float(wash_pts[:, 2].max()), 2) if len(wash_pts) else None,
                wash_note="points inside a swept disk's vertical prism (the foot pads' outer edge on the plate top only, E)")


def stroke_report(P):
    """the mount from +STROKE along x to its seat (it moves -x); every member's clearance, rod 14-15 by the snap only"""
    other, clip = 1e9, 0.0; worst = ""
    mem = members()
    for s in np.arange(0.0, cm.STROKE + 0.01, 1.0):
        Q = P + [s, 0, 0]
        for nm, a, b, r in mem:
            d = seg_dist(Q, a, b) - r
            if nm == "rod 14-15":
                if s > 0.3: clip = max(clip, float(-d.min()))
            elif d.min() < other:
                other = float(d.min()); worst = f"{nm} at {s:.0f} out"
    return dict(stroke_mm=cm.STROKE, other_members_min_clear_mm=round(other, 2), worst=worst, clip_rod_max_interference_mm=round(clip, 3),
                allowed_mm=round(cm.SNAP + cm.GRIP + 0.05, 2))


# ---------------------------------------------------------------- super-element of the as-built mount
def superelement(mesh, h=1.0):
    from topo import Part
    from topo.material import MATERIALS, stiffness
    from topo.fem import Solver
    b = cm.bodies(); R = b["R"]; M = man(mesh)
    p = Part("mount_as_built", h, M, mf.Manifold(), M, build="+y", material="PETG", log=lambda s: None)
    p.support("foot", R["pads"])
    nd = p.nodes_of(p.elems(R["foot_bottom"], p.solid, what="foot")); p.supports["foot"][nd, 2] = True
    yo, zo0, zo1 = cm.HOOD_OUT
    tie_mid = cm.box(80.0, 90.0, -40, 40, 0, 40)
    regs = [("seat " + str(i), r) for i, r in enumerate(R["seats"])] + [("clip +y", R["clip_rings"][0]), ("clip -y", R["clip_rings"][1]),
            ("hood centre", R["hood_front"]), ("hood corner +y top", R["hood_corners"]["+y top"]), ("mid-tie", tie_mid)]
    nodes, cents = [], []
    X = p.cell_centres()
    for nm, r in regs:
        el = p.elems(r, p.solid, what=nm); nodes.append(p.nodes_of(el)); cents.append(X[el].mean(0))
    C = stiffness(MATERIALS["PETG"], 1)
    sol = Solver(p.shape, h, C, p.supports, active=p.solid, backend="direct", log=lambda s: None)
    E = np.where(p.solid, 1.0, 1e-9); nr = len(regs)
    F = np.zeros((p.ndof, 3 * nr))
    for i, nd_ in enumerate(nodes):
        for d in range(3): F[3 * nd_ + d, 3 * i + d] = 1.0 / len(nd_)
    U = sol.solve(E, F, ["foot"] * (3 * nr))
    Cf = np.zeros((3 * nr, 3 * nr))
    for j, nd_ in enumerate(nodes):
        for e in range(3): Cf[3 * j + e, :] = U[3 * nd_ + e, :].mean(0)
    Cf = 0.5 * (Cf + Cf.T); K = np.linalg.inv(Cf)
    # stresses for the breaking order: unit loads at the hood centre (x, y, z), clips free: von Mises max away from the loaded / fixed nodes
    g = sol.grid; edof = g.edof(); en = edof[:, ::3] // 3
    hot = np.zeros(p.nn, bool); hot[nodes[6]] = True; hot[p.supports["foot"].any(1)] = True
    keep = p.solid & ~hot[en].any(1)
    sig = {}
    for k, ax in enumerate("xyz"):
        u = U[:, 3 * 6 + k]; ue = u[edof]                                   # (nel, 24)
        S = (C @ (sol.B0 @ ue[keep].T)).T                                    # Voigt stress at element centres, MPa per N
        vm = np.sqrt(0.5 * ((S[:, 0] - S[:, 1]) ** 2 + (S[:, 1] - S[:, 2]) ** 2 + (S[:, 2] - S[:, 0]) ** 2) + 3 * (S[:, 3] ** 2 + S[:, 4] ** 2 + S[:, 5] ** 2))
        sig[ax] = float(np.percentile(vm, 99.9))
    # mass split: front body (x >= 112: cradle, hood, clips) and the tie (x 52..112) at mid-tie; the foot (x < 52) is clamped
    cells = X[p.solid]; m_c = h ** 3 * RHO
    front = cells[cells[:, 0] >= 112.0]; tie = cells[(cells[:, 0] < 112.0) & (cells[:, 0] >= 52.0)]
    return dict(K=K, Cf=Cf, names=[r[0] for r in regs], cents=cents, sig_per_N=sig, front_cells=front, cell_g=m_c, tie_g=len(tie) * m_c,
                n_solid=int(p.solid.sum()))


class SuperFrame(Frame):
    def __init__(self):
        super().__init__(); self.supers = []; self.springs = []           # springs: (node i, node j, dofs, k N/mm)

    def stiffness(self):
        K = super().stiffness()
        if not self.supers and not self.springs: return K
        rows, cols, vals = [], [], []
        for ids, Kr in self.supers:
            dofs = np.array([6 * i + d for i in ids for d in range(3)])
            I, J = np.meshgrid(dofs, dofs, indexing="ij"); rows.append(I.ravel()); cols.append(J.ravel()); vals.append(Kr.ravel())
            rd = np.array([6 * i + d for i in ids for d in range(3, 6)])            # the points carry no moment: a token rotational spring
            rows.append(rd); cols.append(rd); vals.append(np.full(len(rd), 1e-3))     # keeps a point with no member attached regular
        for i, j, ds, k in self.springs:
            for d in ds:
                a_, b_ = 6 * i + d, 6 * j + d
                rows += [np.array([a_, b_, a_, b_])]; cols += [np.array([a_, b_, b_, a_])]; vals += [np.array([k, k, -k, -k])]
        A = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=K.shape).tocsr()
        return csr_matrix(K + A)


def inertia_points(parts):
    """mass, CG and 6 lumped points with the bodies' inertia (v1 camera_mount.rigid_body_points): parts [(points (n,3), g each) or box]"""
    Ms, Cs, Is = [], [], []
    for item, m in parts:
        if isinstance(item, tuple):
            (x0, x1), (y0, y1), (z0, z1) = item; c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); a, b, cc = x1 - x0, y1 - y0, z1 - z0
            I = m / 12 * np.diag([b * b + cc * cc, a * a + cc * cc, a * a + b * b])
        else:
            P = np.asarray(item); c = P.mean(0); d = P - c; I = m / len(P) * ((d * d).sum() * np.eye(3) - d.T @ d)
        Ms.append(m); Cs.append(c); Is.append(I)
    mt = sum(Ms); cg = sum(m * c for m, c in zip(Ms, Cs)) / mt; It = np.zeros((3, 3))
    for m, c, I in zip(Ms, Cs, Is):
        d = c - cg; It += I + m * ((d @ d) * np.eye(3) - np.outer(d, d))
    w, V = np.linalg.eigh(It); a2 = 3 / (2 * mt) * np.array([w[1] + w[2] - w[0], w[0] + w[2] - w[1], w[0] + w[1] - w[2]])
    return mt, cg, [cg + s * math.sqrt(max(a2[k], 1e-6)) * V[:, k] for k in range(3) for s in (1, -1)]


def frame_model(se, snap, with_mount=True):
    f = SuperFrame(); nid = [f.node(p) for p in N]; rods = [tuple(r) for r in cm.CG["rods"]]
    clips = [np.array([N[14][0], s * cm.CLIP_Y, N[14][2]]) for s in (1, -1)]
    for a, b in rods:
        pts = [N[a]] + (sorted(clips, key=lambda c: np.linalg.norm(c - N[a])) if (a, b) == (14, 15) and with_mount else []) + [N[b]]
        for p_, q_ in zip(pts[:-1], pts[1:]): f.member(f.node(p_), f.node(q_), cf.ROD3, f"rod {a}-{b}")
    for k, (p, q) in cm.RES["links"].items():
        p, q = np.array(p), np.array(q); c_ = f.node(p); f.member(c_, f.node(q), cf.ROD3, k, k=cf.K_TIE); f.fix(c_)
    mass = cf.masses(f, nid, f.node(N[0])); mass[f.node(N[0])] -= cf.CAM_G          # the cage alone (cf.masses adds v1's camera at H)
    out = dict(nodes={})
    if not with_mount:
        hz, U = f.modes(mass, 4); return dict(modes_hz=[round(float(x), 1) for x in hz])
    ids = []
    for nm, c in zip(se["names"], se["cents"]):
        if nm.startswith("clip"):                                      # the clip holds the rod in x and z; in y it slides (grip ~1.5 N, E): the foot takes y
            c = clips[0] if "+y" in nm else clips[1]; r_ = f.node(c); m_ = f.node(c + np.array([0, 1.0 if c[1] > 0 else -1.0, 0]))
            f.springs.append((r_, m_, (0, 2), 1e6)); ids.append(m_); out["nodes"][nm] = np.round(c, 1).tolist(); continue
        ids.append(f.node(c)); out["nodes"][nm] = np.round(c, 1).tolist()
    f.supers.append((ids, se["K"]))
    mt, cg, pts = inertia_points([(cm.CAM_BOX, cm.CAM_G), (se["front_cells"], len(se["front_cells"]) * se["cell_g"])])
    H = f.node(cg)
    for i in ids[:4]: f.member(H, i, cf.RIGID, "body")
    for q in pts:
        j = f.node(q); f.member(H, j, cf.RIGID, "body"); mass[j] = mass.get(j, 0.0) + mt / 6
    mid = ids[8]; mass[mid] = mass.get(mid, 0.0) + se["tie_g"]
    hz, U = f.modes(mass, 8); body = [f.node(q) for q in pts]
    share, ratio = [], []
    for k in range(len(hz)):
        eb = sum(mass[i] * np.sum(U[6 * i:6 * i + 3, k] ** 2) for i in body); ea = sum(m_ * np.sum(U[6 * i:6 * i + 3, k] ** 2) for i, m_ in mass.items())
        share.append(eb / ea)
        ratio.append(max(np.linalg.norm(U[6 * i:6 * i + 3, k]) for i in body) / max(np.linalg.norm(U[6 * i:6 * i + 3, k]) for i in mass))
    out.update(modes_hz=[round(float(x), 1) for x in hz], camera_share=[round(float(s), 2) for s in share], camera_disp_ratio=[round(float(r), 2) for r in ratio],
               first_mode_hz=round(float(hz[0]), 1), first_camera_mode_hz=round(float(next((h for h, s in zip(hz, share) if s > 0.2), hz[-1])), 1),
               body_g=round(mt, 1), body_cg=np.round(cg, 1).tolist(), tie_g=round(se["tie_g"], 1))
    big = sorted(range(len(f.X)), key=lambda i: -np.linalg.norm(U[6 * i:6 * i + 3, 0]))[:4]
    out["mode1_largest_motion_at"] = [np.round(f.X[i], 1).tolist() for i in big]
    Kr = se["K"]; dof = lambda i: [6 * i + d for d in range(3)]

    def run(loads):
        u = f.solve(loads); ui = np.concatenate([u[dof(i)] for i in ids]); fi = Kr @ ui     # forces applied ON the mount at its interface points
        clip = {nm: fi[3 * k:3 * k + 3] for k, nm in enumerate(se["names"]) if nm.startswith("clip")}
        mf_ = f.member_forces(u); carb = {}
        for r in mf_:
            if r["name"] in ("body",): continue
            c = carb.setdefault(r["name"], 0.0); carb[r["name"]] = max(c, r["buckle"], r["sigma"] / SIG_C)
        return u, clip, carb

    def total(loads):
        return sum(np.asarray(v, float) for v in loads.values())

    cases = {}

    def summarise(name, loads):
        u, clip, carb = run(loads)
        Fa = total(loads); fc = sum(clip.values()); R = -(Fa + fc)                       # the foot's force on the mount + camera
        wc = max(carb.items(), key=lambda kv: kv[1])
        cases[name] = dict(clip_force_on_mount_N={k: np.round(v, 1).tolist() for k, v in clip.items()}, foot_force_N=np.round(R, 1).tolist(),
                           carbon_worst=wc[0], carbon_ratio=round(wc[1], 3),
                           lens_disp_mm=round(float(np.linalg.norm(u[dof(H)])), 3))
    hood, corner = ids[6], ids[7]
    summarise("hood hit 250 N centre -x", {hood: np.array([-cm.HIT, 0, 0])})
    summarise("hood hit 250 N top corner -x", {corner: np.array([-cm.HIT, 0, 0])})
    W = (mt + se["tie_g"]) / 1000 * cm.G
    summarise("flight 4 g down", {H: np.array([0, 0, -W * cm.FLIGHT])})
    for nm, d in (("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+y", (0, 1, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1))):
        summarise(f"crash 35 g {nm} (camera + mount)", {H: np.array(d, float) * W * cm.CRASH})
    out["cases"] = cases
    # breaking order: a load at the hood centre per direction; what lets go first (E strengths)
    order = {}
    F_pop, F_slide = snap["pop_N"], snap.get("slide_N", 10.0)
    for nm, d, ax in (("pull forward +x", (1, 0, 0), "x"), ("push back -x (frontal)", (-1, 0, 0), "x"), ("side +y", (0, 1, 0), "y"),
                      ("up +z", (0, 0, 1), "z"), ("down -z", (0, 0, -1), "z")):
        u, clip, carb = run({hood: np.array(d, float)})
        R = -(np.array(d, float) + sum(clip.values())); cand = []
        pull = max(max(0.0, -v[0]) for v in clip.values())
        if pull > 1e-9: cand.append((F_pop / pull, "a clip pops (the fuse)"))
        sl = max(abs(v[1]) for v in clip.values())
        if sl > 1.0: cand.append((F_slide / sl, "a clip slides on the rod (grip)"))
        sh = float(np.hypot(R[0], R[1]))
        if sh > 1e-9: cand.append((MU * 2 * PRELOAD / sh, f"the foot slips on the plate (friction {MU} x 2 x {PRELOAD:.0f} N preload, E; slot +-{cm.SLOT_HALF} then the bolts bear)"))
        if R[2] < -1e-9: cand.append((2 * INSERT_PULL / -R[2], "the foot's 2 plate inserts pull out (belly mount, 600 N each, E)"))
        cand.append((PETG_FAIL / se["sig_per_N"][ax], "the mount's PETG (99.9th pct von Mises, foot clamped, clips free; E)"))
        for k, v in carb.items():
            if v > 1e-12 and k.startswith(("rod", "saddle")): cand.append((1.0 / v, f"carbon {k}"))
        cand.sort(); order[nm] = [(round(a, 0), b) for a, b in cand[:4]]
    out["breaking_order"] = order
    return out


# ---------------------------------------------------------------- print, preview, system
def print_pose(m):
    """+y face down (the cheek plane: the clips' prongs bend within the layers), centred on the bed"""
    q = m.copy(); q.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
    q.apply_translation([-q.bounds[0][0] - q.extents[0] / 2, -q.bounds[0][1] - q.extents[1] / 2, -q.bounds[0][2]]); return q


def system_recheck(mesh_rel, g_mount):
    a = json.load(open(os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")))
    old = {p["name"] for p in a["parts"] if p.get("owner") == "scout camera mount"} | {"camera cradle (printed)", "camera cradle arms (printed)", "camera cradle clips (printed)"}
    rel = os.path.relpath(RUN, ROOT)
    parts = [p for p in a["parts"] if p["name"] not in old]
    parts.append(dict(name="camera mount (printed)", role="cage", mesh=mesh_rel, material="PETG", mass_g=round(g_mount, 2), tag="M",
                      owner="scout camera mount", version="v2", source="nox/scout/v3/camera_mount_v2.py"))
    cp = [c for c in a["contact_pairs"] if not ({c[0], c[1]} & old)]
    cp += [["camera mount (printed)", "frame plate (Bee35 carbon)", 1.0],      # the foot sits on the plate top
           ["camera mount (printed)", "cage shell tubes", 5.0]]                # the clips' grip ribs, 0.05 into rod 14-15 (2.9 mm3, C)
    acc = [x for x in a.get("access", []) if not x["name"].startswith("camera mount")]
    for (x, y) in cm.FOOT_BOLTS:
        acc.append({"name": f"camera mount foot bolt key ({x:.0f}, {y:+.0f})", "part": "camera mount (printed)", "from": [x, y, cm.PLATE_TOP + cm.PAD[2] + 3.5],
                    "dir": [0, 0, 1], "length_mm": 60.0, "tool": {"type": "cylinder", "d": 6.0}})
    acc.append({"name": "camera mount push-on (-x, 110; rods: stroke check)", "part": "camera mount (printed)", "dir": [1, 0, 0], "length_mm": cm.STROKE,
                "tool": "part", "ignore": ["cage shell tubes", "frame plate (Bee35 carbon)", "camera"]})
    a["parts"], a["contact_pairs"], a["access"] = parts, cp, acc
    a["note"] = "v12 Scout manifest with the camera mount v2 (runs/scout_camera_mount/v2, topology) in place of v1. " + a["note"].split("v12 Scout manifest with the camera mount v1 (runs/scout_camera_mount/v1) in place of the layout's cradle, arms and clips. ")[-1]
    out = os.path.join(RUN, "scout_with_mount.assembly.json"); json.dump(a, open(out, "w"), indent=1)
    r = subprocess.run([sys.executable, os.path.join(ROOT, "nox", "system_check.py"), out, "--report", "--out", os.path.join(RUN, "system_check.json")],
                       capture_output=True, text=True, cwd=ROOT)
    open(os.path.join(RUN, "system_check.txt"), "w").write(r.stdout + r.stderr)
    print(r.stdout[-2500:])
    return dict(manifest=os.path.relpath(out, ROOT), report=f"{rel}/system_check.txt", returncode=r.returncode)


def main():
    from solid_check import solid_check
    mesh = trimesh.load(os.path.join(RUN, f"{NAME}.stl"), force="mesh")
    g = float(mesh.volume * RHO)
    P = np.vstack([mesh.vertices[::3], mesh.sample(60000)])
    snap = cm.fork_snap()
    # grip slide (E): v1's rib model on the prong hooks, scaled by the prongs' stiffness: slide ~ mu x (2 rib loads + seat)
    snap["slide_N"] = round(cm.GRIP * snap["k_prong_N_mm"] * 3 * 0.25, 1)
    res = dict(mass_g=round(g, 2), bounds=np.round(mesh.bounds, 1).tolist(), snap=snap)
    res["view"] = view_report(P, mesh); print("view", res["view"])
    res["fit"] = fit_report(P, mesh); print("fit", json.dumps(res["fit"]))
    res["stroke"] = stroke_report(np.vstack([mesh.vertices[::5], mesh.sample(20000)])); print("stroke", res["stroke"])
    se = superelement(mesh)
    print("super-element: solid cells", se["n_solid"], "tie", round(se["tie_g"], 1), "g; hood stress per N", se["sig_per_N"])
    res["frame"] = frame_model(se, snap)
    res["frame"]["cage_alone_modes_hz"] = frame_model(se, snap, with_mount=False)["modes_hz"]
    print("frame", json.dumps({k: res["frame"][k] for k in ("modes_hz", "camera_share", "first_mode_hz", "first_camera_mode_hz", "body_g", "cage_alone_modes_hz")}))
    for k, v in res["frame"]["cases"].items(): print("  ", k, v)
    for k, v in res["frame"]["breaking_order"].items(): print("  break", k, v)
    # print file + gate
    pm = print_pose(mesh); os.makedirs(os.path.dirname(PRINT), exist_ok=True); pm.export(PRINT)
    ok, info = solid_check(trimesh.load(PRINT, force="mesh"))
    res["print"] = dict(file=os.path.relpath(PRINT, ROOT), extents=np.round(pm.extents, 1).tolist(), pose="+y face down", solid_check="PASS" if ok else "FAIL",
                        solid_info={k: v for k, v in info.items() if isinstance(v, (int, float, str, bool))} if isinstance(info, dict) else str(info))
    print("solid_check", res["print"]["solid_check"])
    mesh.export(os.path.join(RUN, "camera_mount_assembled.stl"))
    res["system"] = system_recheck(os.path.relpath(os.path.join(RUN, "camera_mount_assembled.stl"), ROOT), g)
    json.dump(res, open(os.path.join(RUN, "checks.json"), "w"), indent=1, default=float)
    rj = os.path.join(RUN, "result.json")
    if os.path.exists(rj):
        r = json.load(open(rj)); r["checks"] = res; json.dump(r, open(rj, "w"), indent=1, default=float)
    return res


if __name__ == "__main__":
    main()
