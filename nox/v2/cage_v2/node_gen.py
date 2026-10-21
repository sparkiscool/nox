"""S2 cage v3 (layout C2) printed nodes, each through the whole noxopt pipeline (Spark 17 Oct 03:01: all nodes super
smooth by the final optimization run; 04:01: all the cage CAD in the cage chat).

Per part: drawn = rod sockets (sleeves, teardrop keel toward the bed so a level sleeve prints without support), wheel
cheeks, seat pads; cuts = bores with lead-ins, wheel slots, pin holes, keep-outs (collar faces 1.0, Radxa, hex-key
lanes); loads = the socket end forces and contact forces of the frame model (layout_v3.py, layout C2) at the design
speed v_intact (the speed where the rods reach index 1, so a node is never the weak link), hits as 'crash', swap push and
upside-down landing as 'misuse'; the part's support = its ring sleeve (tees), its +x lower socket (corner node) or its
seat pads (crown). Mirror-image instances share one part: their loads are mirrored into the part's frame.
Then layout (overhang filter on, build direction per part), prune_thin, connectivity, polished smooth build,
thin / function / wrinkle checks, as-built strength, 45 deg print closure (print_close.py: fills only what the polish left
flatter than 45 deg, smoothed), print_check, STL in drone coordinates.
Run: python3 node_gen.py <part> [h] [time limit s]     parts: corner vtee snode crown  (CAGE_LAYOUT env: layout json)
"""
import sys, os, json, copy, math, time
import numpy as np, trimesh, manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__))
ENG = os.environ.get("NOX_ENGINE", os.path.expanduser("~/nox"))
for p_ in (ENG, os.path.dirname(HERE), HERE):
    if p_ not in sys.path: sys.path.insert(0, p_)
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
import noxopt as NO, printopt as PO
import layout_v3 as L, s2_guard_frame as sg

OUT = os.environ.get("NODE_OUT", "/home/claude/cage/nodes"); os.makedirs(OUT, exist_ok=True)
OPT = json.load(open(os.path.join(HERE, os.environ.get("CAGE_LAYOUT", "layout_C3.json"))))   # C3: box-safe skirt, no feet (17 Oct 09:20)
GAP, LEAD = 0.3, 0.6
REIN_PASSES, R_REIN = int(os.environ.get("REIN_PASSES", 3)), float(os.environ.get("R_REIN", 1.5))   # README Strength: local reinforcement passes
REIN_LIM = float(os.environ.get("REIN_LIM", 1.0))   # index a case must stay under; v4 runs at 0.5 (Problem 46: socket members <= 0.5)
TPU_AMS = PO.Material("TPU_AMS", E=1190.0, Ez=600.0, nu=0.45, Xt=22.4, Zt=11.2, S=11.2, density=1.26)   # Bambu TDS
MAT = {"PETG": (PO.MATERIALS["PETG"], 2.0), "TPU": (TPU_AMS, 2.4)}       # material, wall = floor (nox-generative)
F_, C_ = 112.0, 100.0
ZL, ZU = L.PZ - 8.0, L.PZ + 8.0
ROD = dict(lower=OPT.get("lower", 2.5), upper=OPT["upper"], spoke=OPT.get("spoke", 2.5), skirt=OPT["skirt_d"], pin=OPT.get("pin", 2.0))   # v4: all 3.0 (Spark 17 Oct 21:33 / 21:45)
WR = 5.0 if ROD["pin"] >= 3.0 else 4.0        # catch wheel radius: R 5 on a 3.0 pin (v4), R 4 on 2.0 (v3)
PIN_HOLE = 3.3 if ROD["pin"] >= 3.0 else 2.2  # cheek hole: pin epoxied in both cheeks
log = lambda s: print(str(s)[:400], flush=True)

# ------------------------------------------------------------------------------------------------ manifold helpers
def mbox(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float); return mf.Manifold.cube(list(hi - lo)).translate(list(lo))
def frame_to(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; Ln = np.linalg.norm(d); z = d / Ln
    x = np.cross([0, 0, 1], z) if abs(z[2]) < 0.99 else np.cross([1, 0, 0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    M = np.zeros((3, 4)); M[:, 0], M[:, 1], M[:, 2], M[:, 3] = x, y, z, a; return M, Ln
def cyl(a, b, r, r2=None, seg=48):
    M, Ln = frame_to(a, b); return mf.Manifold.cylinder(Ln, r, r if r2 is None else r2, seg).transform(M)
def tm(m_):
    me = m_.to_mesh(); t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
    t.merge_vertices(digits_vertex=5); t.update_faces(t.nondegenerate_faces()); t.remove_unreferenced_vertices(); return t
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
def U(lst):
    out = None
    for m_ in lst:
        out = m_ if out is None else out + m_
    return out if out is not None else mf.Manifold()
def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)

def sleeve(bottom, mouth, rod_d, wall, keel=None, through=False, cap=1.0):
    """socket sleeve (drawn): outer r = rod / 2 + GAP / 2 + wall, closed end cap 1.0 (through: open both ends); keel =
    unit vector toward the bed: teardrop with a 45 deg ridge (a level sleeve prints without support)"""
    a, b = np.asarray(bottom, float), np.asarray(mouth, float); u = unit(b - a)
    ro = rod_d / 2 + GAP / 2 + wall; a0 = a if through else a - u * cap
    body = cyl(a0, b, ro)
    if keel is not None:
        k = np.asarray(keel, float) - (np.asarray(keel, float) @ u) * u
        if np.linalg.norm(k) > 0.3:
            k = unit(k) * ro * math.sqrt(2.0)
            body = mf.Manifold.hull(body + cyl(a0 + k, b + k, 0.05, seg=8))
    return body
def bore(bottom, mouth, rod_d, through=False, ext=8.0):
    a, b = np.asarray(bottom, float), np.asarray(mouth, float); u = unit(b - a); r = rod_d / 2 + GAP / 2
    c = cyl(a - (u * ext if through else 0), b + u * ext, r) + cyl(b - u * LEAD, b + u * 0.01, r, r + LEAD + 0.01)
    if through: c = c + cyl(a + u * LEAD, a - u * 0.01, r, r + LEAD + 0.01)
    return c + cyl(b, b + u * ext, r + LEAD + 0.01)                        # nothing in front of the mouth
def halfspace(n, d):
    """everything with n.p > d"""
    B = 2000.0; b = mf.Manifold.cube([B, B, B], center=True).translate([0, 0, B / 2])
    z = unit(n); x = np.cross([0, 0, 1], z) if abs(z[2]) < 0.99 else np.cross([1, 0, 0], z); x = unit(x); y = np.cross(z, x)
    T = np.zeros((3, 4)); T[:, 0], T[:, 1], T[:, 2] = x, y, z; T[:, 3] = z * d; return b.transform(T)
def collar_keepout(margin=1.0):
    out = []
    for k in range(4):
        c, s = math.cos(k * math.pi / 2), math.sin(k * math.pi / 2); S45 = math.sin(math.radians(45))
        out.append(halfspace((c * S45, s * S45, -S45), L.D_FACE - margin))
    return U(out)

# ------------------------------------------------------------------------------------------------ frame model loads
_FR = {}
def frame():
    if _FR: return _FR["g"], _FR["cases"], _FR["v"]
    P = sg.params_v2(PZ_=L.PZ, lower=("solid", ROD["lower"]), upper=("solid", OPT["upper"])); P.update(opt=OPT, spoke=("solid", ROD["spoke"]), skirt=("solid", OPT["skirt_d"]))
    g = L.Guard3(P); g.carbon_only = True
    r, _ = L.evaluate(OPT); v = r["v_intact"]
    hits = sg.hit_cases_v2(g)
    v = float(os.environ.get("NODE_V", v))                # node design speed (17 Oct 12:00: corner at the 0.5 m/s touch rule)
    U1, F1 = g.indent_batch(hits, sg.energy(v))
    cases = [(f"hit {h[0]} {np.round(np.asarray(h[1], float), 1).tolist() if not isinstance(h[1], float) else h[1]}", "crash", U1[:, j]) for j, h in enumerate(hits)]
    W = sg.MIN_KG * sg.G0
    def apply(loads):
        Fv = np.zeros(6 * g.N)
        for i, f in loads: Fv[6 * i:6 * i + 3] += f
        return g.solve(Fv)
    for sgn in (1, -1):                                   # swap push on the two wheels of the +x and the -x face
        wp = [(p, n) for p, n in g.wheel_pts if n[0] * sgn > 0.5]
        cases.append((f"swap push 15N face {'+' if sgn > 0 else '-'}x", "misuse", apply([(g.node_id(p), -n * 7.5) for p, n in wp])))
    for sgn in (1, -1):
        wp = [(p, n) for p, n in g.wheel_pts if n[1] * sgn > 0.5]
        cases.append((f"swap push 15N face {'+' if sgn > 0 else '-'}y", "misuse", apply([(g.node_id(p), -n * 7.5) for p, n in wp])))
    cases.append(("flip 6g on the crown", "misuse", apply([(i, np.array([0, 0, -6 * W / 4])) for i in g.crown])))
    top = [i for i in range(g.N) if g.X[i, 2] > g.X[:, 2].max() - 0.05]
    Uc, _ = g.indent_batch([("ceiling", "top", top, [np.array([0, 0, 1.0])] * len(top))], sg.energy(v))
    cases.append(("ceiling hit", "crash", Uc[:, 0]))
    _FR.update(g=g, cases=cases, v=v); log(f"frame model C2: {g.N} nodes, {len(cases)} cases at v_intact {v:.2f} m/s")
    return g, cases, v

def region_items(g, inside, u):
    """forces on the printed part: rods crossing into the region (force of the rod on the part) + contacts on region nodes"""
    reg = {i for i in range(g.N) if inside(g.X[i])}; items = []
    for e in g.elems:
        a, b = e["n"]
        if (a in reg) == (b in reg): continue
        k = 0 if a in reg else 1; dof = np.r_[6 * a:6 * a + 6, 6 * b:6 * b + 6]
        fg = e["T"].T @ (e["kl"] @ (e["T"] @ u[dof]))
        other = g.X[e["n"][1 - k]]; here = g.X[e["n"][k]]
        items.append(dict(kind=e["tag"], p=here.copy(), d=unit(other - here), F=-fg[6 * k:6 * k + 3], M=-fg[6 * k + 3:6 * k + 6]))
    ext = g.K @ u
    for i in reg:
        if g.free[6 * i]:
            f = ext[6 * i:6 * i + 3]
            if np.linalg.norm(f) > 1e-3: items.append(dict(kind="contact", p=g.X[i].copy(), d=None, F=f, M=ext[6 * i + 3:6 * i + 6]))
    return items

def xform_items(items, R):
    """rigid or mirrored transform of loads into the part frame (moments are pseudo-vectors)"""
    det = np.linalg.det(R); out = []
    for it in items:
        q = dict(it); q["p"] = R @ it["p"]; q["F"] = R @ it["F"]; q["M"] = det * (R @ it["M"])
        q["d"] = None if it["d"] is None else R @ it["d"]; out.append(q)
    return out

# ------------------------------------------------------------------------------------------------ parts
def wheel_center(px, py, k):
    n = np.array([math.cos(k * math.pi / 2) * math.sin(math.radians(45)), math.sin(k * math.pi / 2) * math.sin(math.radians(45)), -math.cos(math.radians(45))])
    return np.array([px, py, ZL]) + n * (ROD["lower"] / 2 + 2.5 - WR)     # rim 2.5 proud of the lower rod surface
SLOT_R, SLOT_F = WR + 0.4, 1.2     # wheel slot: radius wheel + 0.4 (5.4 at R 5), edge fillet 1.2 (C; wheel 3.0 wide, 0.5 edge chamfers)
def _rrect(hx, hy, f):
    return mf.CrossSection.hull(mf.CrossSection.batch_hull([mf.CrossSection.circle(f, 32).translate([sx * (hx - f), sy * (hy - f)])
                                                            for sx in (1, -1) for sy in (1, -1)]))
def wheel_slot(cw, axis, outw, half, length=20.0):
    """pocket for the wheel + the opening it reaches the collar face through, every edge filleted (17 Oct 12:10: the box slot's
    sharp top corner was the corner node's stress peak, index 19.7 F at (103,114,38) in every crash case)"""
    cw, axis, outw = (np.asarray(v, float) for v in (cw, axis, outw))
    sec = mf.CrossSection.hull(mf.CrossSection.batch_hull([mf.CrossSection.circle(SLOT_F, 32).translate([SLOT_R - SLOT_F, sy * (half - SLOT_F)]) for sy in (1, -1)]
                                                          + [mf.CrossSection.square([0.01, 2 * half]).translate([0, -half])]))
    M, _ = frame_to(cw, cw + axis); pocket = mf.Manifold.revolve(sec, 96).transform(M)
    ez = np.array([0, 0, 1.0]); a = axis if np.linalg.det(np.c_[axis, ez, outw]) > 0 else -axis
    T = np.zeros((3, 4)); T[:, 0], T[:, 1], T[:, 2], T[:, 3] = a, ez, outw, cw
    return pocket + mf.Manifold.extrude(_rrect(half, SLOT_R, SLOT_F), length).transform(T)

def part_corner():
    mat = "PETG"; wall = MAT[mat][1]; down = (0, 0, -1)
    feat, anchor, cuts = {}, {}, []
    for ax, nm in ((1, "x"), (0, "y")):                    # ax = the axis the flat runs along
        def Q(along, z):
            q = np.array([F_, F_, z], float); q[ax] = along; return q
        feat[f"lo_{nm}"] = sleeve(Q(95, ZL), Q(83, ZL), ROD["lower"], wall, down); cuts.append(bore(Q(95, ZL), Q(83, ZL), ROD["lower"]))
        feat[f"up_{nm}"] = sleeve(Q(95, ZU), Q(85, ZU), ROD["upper"], wall, down); cuts.append(bore(Q(95, ZU), Q(85, ZU), ROD["upper"]))
        anchor[f"lo_{nm}"] = Q(86, ZL); anchor[f"up_{nm}"] = Q(86, ZU)
        px, py = (F_, C_) if ax == 1 else (C_, F_); k = 0 if ax == 1 else 1
        cw = wheel_center(px, py, k); axis = np.zeros(3); axis[ax] = 1.0; outw = np.zeros(3); outw[1 - ax] = 1.0
        half = 1.5 + 0.4
        for s in (1, -1):                                   # pin cheeks
            feat[f"cheek_{nm}{'+' if s > 0 else '-'}"] = sleeve(cw + s * axis * half, cw + s * axis * (half + 2.1), ROD["pin"], wall, down, through=True)
            anchor[f"cheek_{nm}{'+' if s > 0 else '-'}"] = np.array([px, py, ZL]) + s * axis * 0.01
        cuts += [wheel_slot(cw, axis, outw, half), cyl(cw - axis * (half + 2.2), cw + axis * (half + 2.2), PIN_HOLE / 2)]
    keep = collar_keepout(1.0)
    ro_lo = ROD["lower"] / 2 + GAP / 2 + wall
    env = mbox((80.0, 80.0, ZL - ro_lo), (119.0, 119.0, ZU + 4.0))
    inside = lambda p: p[0] >= 85.9 and p[1] >= 85.9 and ZL - 1 <= p[2] <= ZU + 1
    def loads():
        g, cases, v = frame(); out = []
        for name, cls, u in cases:
            for k, R in enumerate((np.eye(3), np.diag([-1, 1, 1.]), np.diag([1, -1, 1.]), np.diag([-1, -1, 1.]))):   # 4 corners -> +x+y
                out.append((f"{name} @{k}", cls, xform_items(region_items(g, lambda p, R=R: inside(R @ p), u), R)))
        return out
    return dict(name="corner_node", mat=mat, build="+z", feat=feat, anchor=anchor, support="lo_x", cuts=U(cuts), keep=keep,
                env=env, loads=loads, gen_cm3=0.8)

def tee_part(name, R0, ring_axis, spokes, rod_ring, mat="TPU", build="+z", half=9.0, gen_cm3=0.4, instances=None, kind_ring="upper"):
    """through sleeve on the ring at R0 along ring_axis; spokes = [(socket point on the ring, far end of the rod, rod d, tag)]"""
    wall = MAT[mat][1]; bed = (0, 0, -1) if build == "+z" else (0, 0, 1)
    ra = unit(ring_axis); feat = {"ring": sleeve(R0 - ra * half, R0 + ra * half, rod_ring, wall, bed, through=True)}
    anchor = {"ring": np.asarray(R0, float)}; cuts = [bore(R0 - ra * half, R0 + ra * half, rod_ring, through=True)]
    for j, (Pr, far, d, tag) in enumerate(spokes):
        u = unit(np.asarray(far, float) - Pr); b0 = Pr + u * 3.5; m0 = b0 + u * 12.0
        feat[f"s{j}"] = sleeve(b0, m0, d, wall, bed); cuts.append(bore(b0, m0, d)); anchor[f"s{j}"] = Pr + u * 8.5
    ro = rod_ring / 2 + GAP / 2 + wall
    allp = np.array([np.asarray(R0) - ra * half, np.asarray(R0) + ra * half] + [Pr + unit(np.asarray(f) - Pr) * 15.5 for Pr, f, _, _ in spokes])
    lo = allp.min(0) - 6.0; hi = allp.max(0) + 6.0
    if build == "+z": lo[2] = R0[2] - ro
    else: hi[2] = R0[2] + ro
    env = mbox(lo, hi)
    keep = collar_keepout(1.0)
    inside = lambda p: np.linalg.norm(p - R0) < 9.0
    def loads():
        g, cases, v = frame(); out = []
        for name_, cls, u in cases:
            for k, (c0, R) in enumerate(instances):
                its = xform_items(region_items(g, lambda p, c0=c0: np.linalg.norm(p - c0) < 9.0, u), R)
                out.append((f"{name_} @{k}", cls, its))
        return out
    return dict(name=name, mat=mat, build=build, feat=feat, anchor=anchor, support="ring", cuts=U(cuts), keep=keep, env=env,
                loads=loads, gen_cm3=gen_cm3)

def part_vtee():
    """nose and tail upper tees (V of two spokes); the tail tee's loads are turned 180 deg into the nose frame. The two
    spoke ends sit 3.2 either side of the axis so the sockets keep a wall between them (frame model: both at the axis)."""
    R0 = np.array([F_, 0.0, ZU]); st = L.SPOKES["nose_V"]["starts"]
    spokes = [(R0 + np.array([0, s * 3.2, 0]), np.array(p, float), ROD["spoke"], "spoke") for s, p in zip((1, -1), sorted(st, key=lambda q: -q[1]))]
    Rz = np.diag([-1, -1, 1.0])
    return tee_part("vtee", R0, (0, 1, 0), spokes, ROD["upper"], instances=[(R0, np.eye(3)), (np.array([-F_, 0, ZU]), Rz)], gen_cm3=0.3)
def part_stee():
    R0 = np.array([0.0, F_, ZU]); st = L.SPOKES["side_single"]["starts"][0]
    return tee_part("stee", R0, (1, 0, 0), [(R0, np.array(st, float), ROD["spoke"], "spoke")], ROD["upper"],
                    instances=[(R0, np.eye(3)), (np.array([0, -F_, ZU]), np.diag([1, -1, 1.0]))], gen_cm3=0.2)
def part_ktee():
    """skirt tees on the lower ring at x +-20 of the side flats; the 4 instances mirrored into the +x+y one"""
    q = OPT["bottom"]["quad"]; R0 = L.skirt_target(tuple(q["targets"][0]), 1, 1, ZL); root = np.array([q["x"], L.RCV_Y, q["z"]])
    inst = [(R0 * [sx, sy, 1], np.diag([sx, sy, 1.0])) for sx in (1, -1) for sy in (1, -1)]
    return tee_part("ktee", R0, (1, 0, 0), [(R0, root, ROD["skirt"], "skirt")], ROD["lower"], build="-z", instances=inst,
                    gen_cm3=0.2, kind_ring="lower")
def part_crown():
    """crown on the 4 Radxa boss tops (epoxied, DEFAULT), 6 spoke sockets at the spoke starts (spoke_route.py)"""
    mat = "PETG"; wall = MAT[mat][1]; down = (0, 0, -1)
    feat, anchor, cuts = {}, {}, []
    for bx in (L.BX, -L.BX):
        for by in (L.BY, -L.BY):
            nm = f"pad{'+' if bx > 0 else '-'}{'+' if by > 0 else '-'}"
            feat[nm] = cyl((bx, by, L.BOSS_TOP), (bx, by, L.BOSS_TOP + 1.6), 3.5); anchor[nm] = np.array([bx, by, L.ZC])
    j = 0
    for axis, a, Rp, ring in L.spokes_of(OPT):
        u = unit(Rp - a); b0 = a; m0 = a + u * 12.0
        feat[f"s{j}"] = sleeve(b0, m0, ROD["spoke"], wall, down); cuts.append(bore(b0, m0, ROD["spoke"])); anchor[f"s{j}"] = a.copy(); j += 1
    radxa = mbox((-16.5, -34.5, 0.0), (16.5, 34.5, 65.3 + L.CASE_DZ + 0.8))   # board + back parts (top case geom_top), 0.8 off
    keys = U([cyl((kx, ky, 20.0), (kx, ky, 90.0), kr + 0.5) for (kx, ky), kr in zip(L.KEY_LANES, L.KEY_R)])
    sd = mbox(*L.SD_LANE)
    keep = radxa + keys + sd
    env = mbox((-30.0, -46.0, L.BOSS_TOP), (30.0, 46.0, L.ZS + L.R_SOCK + 2.5))      # pads on the bed, sockets above them (spoke starts z 76)
    support = [k for k in feat if k.startswith("pad")]
    def loads():
        g, cases, v = frame(); out = []
        for name_, cls, u in cases:
            out.append((name_, cls, region_items(g, lambda p: p[2] > 68.5 and abs(p[0]) <= 13.6 and abs(p[1]) <= 31.1, u)))
        return out
    return dict(name="crown", mat=mat, build="+z", feat=feat, anchor=anchor, support=support, cuts=U(cuts), keep=keep,
                env=env, loads=loads, gen_cm3=1.2)
def part_snode():
    """side node (17 Oct, layout C3): one TPU part on both rings at the +y side flat's centre: the upper ring sleeve with
    the crown spoke's socket, the lower ring sleeve with the two skirt sockets (front and back receiver roots); the two
    sleeves joined by what the layout grows. Prints on its outer side (build -y): both sleeves flat on the bed, every
    socket rising inward. The -y node is this part mirrored (loads mirrored in)."""
    mat = "TPU"; wall = MAT[mat][1]; bed = (0, 1, 0)
    RU = np.array([0.0, F_, ZU]); RL = np.array([0.0, F_, ZL]); ra = np.array([1.0, 0, 0]); half = 9.0
    feat = {"ringU": sleeve(RU - ra * half, RU + ra * half, ROD["upper"], wall, bed, through=True),
            "ringL": sleeve(RL - ra * half, RL + ra * half, ROD["lower"], wall, bed, through=True)}
    anchor = {"ringU": RU, "ringL": RL}
    cuts = [bore(RU - ra * half, RU + ra * half, ROD["upper"], through=True), bore(RL - ra * half, RL + ra * half, ROD["lower"], through=True)]
    st = np.array(L.SPOKES["side_single"]["starts"][0], float)
    q = OPT["bottom"]["quad"]; roots = [np.array([sx * q["x"], L.RCV_Y, q["z"]]) for sx in (1, -1)]
    socks = [("s0", RU, st, ROD["spoke"])] + [(f"k{j}", RL + ra * (3.2 * (1 if r[0] > 0 else -1)), r, ROD["skirt"]) for j, r in enumerate(roots)]
    pts = [RU - ra * half, RU + ra * half, RL - ra * half, RL + ra * half]
    for nm, Pr, far, d in socks:
        u = unit(far - Pr); b0 = Pr + u * 3.5; m0 = b0 + u * 12.0      # ring-end sockets 12 deep (the receiver end is 10)
        feat[nm] = sleeve(b0, m0, d, wall, bed); cuts.append(bore(b0, m0, d)); anchor[nm] = (RU if nm == "s0" else RL) + u * 8.5
        pts.append(m0)
    ro = ROD["lower"] / 2 + GAP / 2 + wall
    P_ = np.array(pts); lo = P_.min(0) - 6.0; hi = P_.max(0) + 6.0; hi[1] = F_ + ro
    env = mbox(lo, hi); keep = collar_keepout(1.0)
    inside = lambda p: np.linalg.norm(p - RU) < 9.0 or np.linalg.norm(p - RL) < 9.0
    def loads():
        g, cases, v = frame(); out = []
        for name_, cls, u in cases:
            for k, R in enumerate((np.eye(3), np.diag([1, -1, 1.0]))):
                out.append((f"{name_} @{k}", cls, xform_items(region_items(g, lambda p, R=R: inside(R @ p), u), R)))
        return out
    return dict(name="side_node", mat=mat, build="-y", feat=feat, anchor=anchor, support=["ringL"], cuts=U(cuts),   # one ring held (17 Oct 16:50: both held grew two bodies)
                keep=keep, env=env, loads=loads, gen_cm3=0.35)
PARTS = dict(corner=part_corner, vtee=part_vtee, snode=part_snode, crown=part_crown)

# ------------------------------------------------------------------------------------------------ problem + run
def run(part, h, tlim):
    S_ = PARTS[part](); name = S_["name"]; S_["mat"] = os.environ.get("MAT", S_["mat"])   # cage all PETG (Spark 17 Oct 10:39)
    mat, floor = MAT[S_["mat"]]; t0 = time.time()
    env = S_["env"]; drawn = U(list(S_["feat"].values())) ^ env; cuts = S_["cuts"]; keep = S_["keep"]   # keels trimmed flat at the bed
    allcut = cuts + keep
    b = env.bounding_box(); lo, hi = np.array(b[:3]), np.array(b[3:])
    shape = tuple(int(math.ceil((hi[i] - lo[i]) / h - 1e-6)) for i in range(3)); origin = tuple(lo)
    xs, ys, zs = (lo[i] + (np.arange(shape[i]) + 0.5) * h for i in range(3))
    I3 = np.stack(np.meshgrid(*[np.arange(n) for n in shape], indexing="ij"), -1).reshape(-1, 3); Xc = I3 * h + lo + h / 2
    vox = lambda m_: NO._voxelise(tm(m_), xs, ys, zs).ravel() if not m_.is_empty() else np.zeros(int(np.prod(shape)), bool)
    allcut_v = allcut ^ mbox(lo - np.array([10, 10, 1.0]), hi + 10.0)   # bounded above the voxel rays (z0 - 5): the keep-out half-spaces flipped the ray parity
    solid = vox(drawn - allcut); void = (vox(allcut_v) | ~vox(env)) & ~solid   # (17 Oct 14:55: pockets + collar keep-out were not void)
    ax, sg_ = PO.AXES[S_["build"]]                                             # empty skin, except the bed face
    for a in range(3):
        for side in (0, 1):
            if a == ax and ((side == 0) == (sg_ > 0)): continue
            edge = (Xc[:, a] - lo[a] < 2.0) if side == 0 else (hi[a] - Xc[:, a] < 2.0)
            void |= edge & ~solid
    nx, ny, nz = shape; nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k; ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
    def nodes_of(mask):
        el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
        return np.unique(np.concatenate([nid(ex + a, ey + b_, ez + c) for a in (0, 1) for b_ in (0, 1) for c in (0, 1)]))
    def ncoord(nd):
        k = nd % (nz + 1); j = (nd // (nz + 1)) % (ny + 1); i = nd // ((nz + 1) * (ny + 1)); return np.stack([i, j, k], 1) * h + lo
    fnodes = {}
    for k, m_ in S_["feat"].items():
        mk = vox(m_) & solid
        if mk.sum() == 0: raise RuntimeError(f"feature {k} has no cells at h {h}")
        fnodes[k] = nodes_of(mk)
    sup_keys = S_["support"] if isinstance(S_["support"], list) else [S_["support"]]
    sdofs = np.concatenate([(fnodes[k][:, None] * 3 + np.arange(3)).ravel() for k in sup_keys])
    anchors = {k: np.asarray(v, float) for k, v in S_["anchor"].items() if k not in sup_keys}
    def apply(F, M, p, key):
        nd = fnodes[key]; X = ncoord(nd); c = X.mean(0); r = X - c
        Mc = M + np.cross(p - c, F)
        J = (r ** 2).sum() * np.eye(3) - r.T @ r; w = np.linalg.lstsq(J, Mc, rcond=None)[0]
        f = F / len(nd) + np.cross(w, r); out = np.zeros(ndof)
        for a in range(3): out[3 * nd + a] = f[:, a]
        return out
    raw = S_["loads"](); cases = []
    scored = []                                            # score every case first (cheap), build force vectors for the kept ones only
    for name_, cls, items in raw:
        use = []; tot = 0.0
        for it in items:
            if it["kind"] in ("case", "rcv"): continue
            if it["kind"] in ("upper", "lower") and ("ring" in sup_keys or {"upper": "ringU", "lower": "ringL"}[it["kind"]] in sup_keys): continue   # the held ring
            key = min(anchors, key=lambda k: np.linalg.norm(anchors[k] - it["p"]))
            if np.linalg.norm(anchors[key] - it["p"]) > 12.0: continue
            use.append((it, key)); tot += np.linalg.norm(it["F"])
        if tot > 0.2: scored.append((tot, name_, cls, use))
    hits = sorted([s for s in scored if s[2] == "crash"], key=lambda s: -s[0])[:int(os.environ.get("NCRASH", 12))]
    stat = sorted([s for s in scored if s[2] != "crash"], key=lambda s: -s[0])[:int(os.environ.get("NMIS", 6))]
    for tot, name_, cls, use in hits + stat:
        Fv = np.zeros(ndof)
        for it, key in use: Fv += apply(it["F"], it["M"], it["p"], key)
        cases.append(PO.LoadCase(name_[:60], Fv, cls=cls, support="main"))
    nraw = len(raw); del raw, scored
    log(f"{name}: {nraw} frame cases -> {len(cases)} kept; largest socket load sums: " + ", ".join(f"{t:.0f} N" for t, *_ in (hits + stat)[:6]))
    sups = dict(main=sdofs)
    if len(sup_keys) > 1:                                  # several held pads (one rigid part under them): tie them together so
        k0 = sup_keys[0]; sups["t0"] = (fnodes[k0][:, None] * 3 + np.arange(3)).ravel()   # the layout grows one body (17 Oct 16:50)
        for k in sup_keys[1:]:
            nd = fnodes[k]
            for a in range(3):
                Fv = np.zeros(ndof); Fv[3 * nd + a] = 1.0 / len(nd); cases.append(PO.LoadCase(f"tie {k} {'xyz'[a]}", Fv, cls=None, support="t0"))
        log(f"{name}: {len(sup_keys) - 1} held pads tied to {k0} (3 cases each)")
    pb = PO.Problem(shape=shape, h=h, origin=origin, solid=solid, void=void, supports=sups, cases=cases,
                    material=mat, build=S_["build"], overhang=True, R=max(2.0, floor / h), comp_factor=None)
    pb0 = copy.copy(pb); design = ~solid & ~void
    S_["gen_cm3"] = float(os.environ.get("GEN_CM3", S_["gen_cm3"]))
    VF = S_["gen_cm3"] * 1000 / (design.sum() * h ** 3)
    log(f"{name}: grid h {h} {shape} = {int(np.prod(shape)):,} cells, design {int(design.sum()):,}, drawn {solid.sum() * h ** 3 / 1000:.2f} cm3, "
        f"generated target {S_['gen_cm3']} cm3 (volfrac {VF:.3f}), {S_['mat']}, build {S_['build']}")
    tag = f"{OUT}/{name}" + os.environ.get("TAG_SUFFIX", "")
    if os.environ.get("RECHECK"):                         # as-built strength of an existing mesh in this material, nothing else
        Pm = trimesh.load(os.environ["RECHECK"]); A = NO._voxelise(Pm, xs, ys, zs).ravel().astype(float)
        pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
        rows, rep = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A)); print(rep)
        w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
        res = dict(part=name, mat=S_["mat"], mesh=os.environ["RECHECK"], mass_g=round(Pm.volume / 1000 * mat.density, 2),
                   worst=round(max(w["inlayer"], w["across"]), 2), worst_case=w["case"])
        json.dump(res, open(f"{tag}_recheck_{S_['mat']}.json", "w"), indent=1); log("RECHECK " + json.dumps(res)); return res
    big = int(np.prod(shape)) > int(os.environ.get("BIG_CELLS", 150000))   # crown 146k OOMed on the direct solve (17 Oct 17:42)
    if big:
        import noxopt_local as NL; Lay = NL.Layout(pb, VF, log=log, backend="auto")
    else:
        Lay = NO.Layout(pb, VF, log=log)
    if os.environ.get("REBUILD") and os.path.exists(f"{tag}_pruned.npz"):   # build + strength passes only, from the saved pruned layout
        Z = np.load(f"{tag}_pruned.npz"); dens = Z["dens"]; pb0.void = Z["void"] if "void" in Z.files else pb0.void; removed = []
        class _L: pass
        Lay = _L(); Lay.pb0 = pb0; log(f"rebuild from {tag}_pruned.npz")
    else:
        dens = Lay.run(checkpoint=f"{tag}_ck.npz", resume=f"{tag}_ck.npz" if os.path.exists(f"{tag}_ck.npz") else None, time_limit=tlim)
        Lay.save(f"{tag}_density.npz", dens)
    if not os.environ.get("REBUILD") and (big or int(np.prod(shape)) > 60000):                # prune re-settles on multigrid too (a second direct solve OOMed, 17 Oct 10:36)
        import noxopt_local as NL
        class _GL(NL.Layout):
            def __init__(self, pb, volfrac, R=None, eta=(0.7, 0.5, 0.3), log=print, masked=True, **kw):
                super().__init__(pb, volfrac, R=R, eta=eta, log=log, backend="gmg")
        NO.Layout = _GL
    if not os.environ.get("REBUILD"):
        Lay, dens, removed = NO.prune_thin(Lay, dens, floor_mm=floor, margin_mm=0.75, log=log, rounds=2, time_limit=float(os.environ.get("PRUNE_T", 600)))   # re-settle capped (17 Oct 10:40)
    allowed = env - allcut
    rr = [R_REIN]
    def build(dn, hot=None):
        surf = NO.smooth_surface(dn, pb0.solid, pb0.void, pb0.shape, h, pb0.origin, keep_mesh=tm(drawn - allcut),
                                 clip_mesh=tm(allowed), clip_margin=0.5, sigma_mm=0.6 * h, floor_mm=floor, log=log,
                                 reinforce=None if not hot else np.array(hot), reinforce_r=rr[0])
        return ((man(surf) + drawn) ^ env) - allcut
    for attempt in range(3):
        con = NO.connectivity(dens, Lay.pb0); log(f"connectivity: {con}")
        part_m = build(dens); Pm = tm(part_m)
        thin = NO.thin_check(Pm, tm(drawn - allcut), floor, log=log)
        if not thin or attempt == 2 or os.environ.get("REBUILD"): break
        Lay, dens, _ = NO.prune_thin(Lay, dens, floor_mm=floor, margin_mm=0.75, log=log, built=thin, time_limit=float(os.environ.get("PRUNE_T2", 400)))
    np.savez_compressed(f"{tag}_pruned.npz", dens=dens, void=pb0.void)
    def strength(Pm):                                     # as-built recheck, every kept case (README Strength)
        A = NO._voxelise(Pm, xs, ys, zs).ravel().astype(float)
        pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
        binary = np.where(pb0.solid, 1.0, A)
        pb1 = copy.copy(pbs); pb1.overhang = False; pb1.R = 0.5; po = PO.PrintOpt(pb1)
        fail, over = po.failure, set()
        def rec(s_, A_):                                  # every element over 1.0 in any case = the hot cells (not just the peak)
            f1, d1, f2, d2 = fail(s_, A_); sm = po.smask
            for q in po.ecent[sm][(f1[sm] > REIN_LIM) | (f2[sm] > REIN_LIM)]: over.add(tuple(np.round(q, 2)))
            return f1, d1, f2, d2
        po.failure = rec
        _, _, _, _, info, _ = po.evaluate(binary, 64.0, grads=False)
        rows = [dict(case=r["case"], cls=r["cls"], inlayer=float(r["inlayer"]), across=float(r["across"]),
                     wi=list(r["where_inlayer"]), wa=list(r["where_across"])) for r in info if r and "inlayer" in r]
        po.failure = fail; strength.hot = sorted(over)
        return rows, po.report(binary, 64.0, "noxopt")
    hot = []
    for rp_ in range(REIN_PASSES + 1):                    # README Strength: a must-hold case over 1.0 gets local reinforcement, at most 3 passes
        rows, rep = strength(Pm)
        w = max(rows, key=lambda r: max(r["inlayer"], r["across"])) if rows else dict(inlayer=0, across=0, case="-")
        log(f"strength pass {rp_}: worst {max(w['inlayer'], w['across']):.2f} {w['case']}, {Pm.volume / 1000 * mat.density:.2f} g")
        over = [r for r in rows if max(r["inlayer"], r["across"]) > REIN_LIM]
        if not over or rp_ == REIN_PASSES: break
        new = [list(q) for q in strength.hot if list(q) not in hot]; hot += new
        if not new: rr[0] *= 1.5                          # same cells still over: the balls were clipped by a cut, grow them
        log(f"  {len(new)} new cells over {REIN_LIM} near {np.round(np.mean(new or hot, 0), 1).tolist()} -> reinforcement balls r {rr[0]:.1f}")
        part_m = build(dens, hot); Pm = tm(part_m)
        thin = NO.thin_check(Pm, tm(drawn - allcut), floor, log=log)
    bodies = len(part_m.decompose())
    fc = NO.function_check(Pm, drawn - allcut, allcut, allowed, log=log)
    wr = NO.wrinkle(Pm, drawn=tm(drawn - allcut), allowed=tm(allowed))
    import print_check as PC, print_close as PCL
    Pm, closed_mm3 = PCL.close(Pm, tm(allcut_v), S_["build"])          # 45 deg print closure of the polished part
    import mesh_seal; Pm, _sealed = mesh_seal.seal(Pm)      # zero-area holes from degenerate-face cleanup (17 Oct 17:45)
    Pq = Pm.copy(); Pq.merge_vertices(); bodies = len(Pq.split(only_watertight=False))   # face-connected pieces (manifold of a broken mesh read 1)
    pc, _ = PC.check(Pm, S_["build"])
    Pm.export(f"{tag}.stl")
    res = dict(part=name, mat=S_["mat"], build=S_["build"], h=h, mass_g=round(Pm.volume / 1000 * mat.density, 2), bodies=bodies,
               watertight=bool(Pm.is_watertight), function=[round(float(x), 2) for x in fc], thin=len(thin),
               wrinkle=[round(float(x), 2) for x in wr], reinforced=len(hot), worst=round(max(w["inlayer"], w["across"]), 2), worst_case=w["case"],
               support_mm2=round(pc["support_mm2"], 1), bridge_mm2=round(pc["bridge_mm2"], 1), height=round(pc["height"], 1),
               removed_thin=len(removed), closure_mm3=round(closed_mm3, 1), minutes=round((time.time() - t0) / 60, 1))
    json.dump(res, open(f"{tag}_result.json", "w"), indent=1); print(rep); log("RESULT " + json.dumps(res))
    return res

if __name__ == "__main__":
    part = sys.argv[1]; h = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6; tlim = float(sys.argv[3]) if len(sys.argv) > 3 else 2400
    if "--check" in sys.argv:
        S_ = PARTS[part](); d = U(list(S_["feat"].values()))
        log(f"{part}: drawn {d.volume():.0f} mm3, pieces {len(d.decompose())}, drawn in keep-outs {(d ^ S_['keep']).volume():.2f} mm3, "
            f"drawn outside env {(d - S_['env']).volume():.2f} mm3")
        its = S_["loads"](); n = sum(1 for _ in its); log(f"{n} frame cases")
        sys.exit(0)
    run(part, h, tlim)
