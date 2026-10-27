"""Scout v3 belly mount, concept v1 (27 Oct 2026). `python3 nox/scout/v3/belly_mount.py` -> runs/scout_belly_mount/v1/ (~3 min).

The printed piece under the Bee35 carbon plate, between the plate underside (z 18.5) and the receiver's mount plane (z 9.6):
the receiver's partner (its 4 M3 bolts into heat-set inserts here), the frame round the FC, the flow sensor's cradle, and the
face the electronics tail block's tongue bears on (x -31.0). System pass on the assembled Scout v3 (main's assembly, layout v12,
catch 5.0 forward), every part in pose:
  1. FC placement by the system: the plate's own AIO holes (as built, P) vs a seat on the belly mount; the stack in the 8.9 gap.
  2. Flow sensor placement by a grid search (1 mm, seeded, deterministic): body + cradle 2.0 clear of every part, both windows'
     23 deg cones (42 FOV + 2) clear of everything down to the ground, out of the funnel band unless inside the swap column, out of
     the prop wash, the shortest clear strut to the belly mount (objective), centre plane preferred.
  3. FC USB-C exit search (12 x 7 plug + cable, 250 long) from each FC edge.
  4. The belly mount drawn (frame, receiver inserts, plate inserts, tail face, ears, windows, flow strut + cradle), solid_check.
  5. Loads (dimensions.md L: crash 35 g x 14, flight, landing, hover; the tail block's reactions), bolt groups, inserts, sections
     cut from the mesh, the flow strut; FC on grommets: modes; breaking order.
  6. Fit table (every pair within 10 of our pieces: >= 2.0 or a declared face contact <= 0.05 mm3), system_check (cage first,
     shell, fit, probes, access incl. the new keys and the USB path, balance), mass and CG for the cage session, PX4 offsets.
  7. Two alternatives weighed with numbers; preview.png.
Drone frame: x nose, y left, z up, plate underside 18.5, thrust centre x 0. Numbers: nox/dimensions.md System Scout v3, H, M, L.
Tags: C computed on the v12 geometry, E estimate, P measured, M mesh.
"""
import os, sys, json, math, io, time
import numpy as np, trimesh
from scipy import ndimage as ndi

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "nox"))
RUN = os.path.join(ROOT, "runs", "scout_belly_mount", "v1")
ASM = os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")
V12 = os.path.join(ROOT, "runs", "scout_layout", "v12")
ELEC = os.path.join(ROOT, "runs", "scout_electronics", "v2")
LOG = io.StringIO()
RNG = np.random.default_rng(0)              # seeded: a rerun reproduces byte for byte


def say(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); LOG.write(s + "\n")


# ---------------------------------------------------------------- numbers (C from the v12 meshes unless tagged)
PLATE_Z, PLATE_TOP = 18.5, 22.0          # plate underside / top (C, 3.5 carbon)
REC_Z = 9.6                              # receiver mount plane (C, receiver v3 top)
GAP = 2.0                                # every pair (DEFAULT, CLAUDE.md)
REC_BOLTS = [(-37.65, 26.55), (-37.65, -26.55), (48.28, 26.55), (48.28, -26.55)]   # receiver v3 M3 up into the partner (C, mesh holes)
INSERT = (4.0, 6.5)                      # M3 x 5.7 heat-set insert: hole Ø4.0, hole depth 6.5 (>= 6, receiver spec; E)
BOSS_R = 5.0                             # Ø10 boss: 3.0 wall round the Ø4.0 insert hole (attachment rule)
PLATE_BOLTS = [(-25.0, 13.0), (-25.0, -13.0), (30.0, 13.0), (30.0, -13.0)]   # 4 new M3 down through the plate into inserts (DEFAULT):
                                         # straight keys clear of the swept disks (|x| or |y| small), off the FC, the leads channel,
                                         # the tail block's foot (|y| <= 8) and its plate bolts (-42, +-16)
TAIL_X = -31.0                           # the tail block tongue's front face = our tail face (electronics v2 requirement, C)
EAR_Y = 21.0                             # the tail block's widest point aft of x -31 (its plate-bolt bosses, C mesh): our ears bear on it
EAR_X0 = -44.0                           # the ears run aft to here (the rear receiver bosses' back + skin)
FC_C, FC_W, FC_H = (0.0, 0.0), 36.0, 8.0  # FC board centre on the plate's AIO holes (as built, P; CHECK on the plate), 36 x 36 x 8 (M)
FC_Z0 = 9.5                              # FC lowest point 9 below the plate underside (H, Spark P: as built)
FC_HOLES = 12.75                         # 25.5 x 25.5 (M)
FC_WIN = 20.0                            # frame window half width: the board + 2.0
XT60_KO = (40.2, 56.3, 15.6)             # XT60E1-M + its rear leads + 2.0: x0, x1, |y| (C)
LEADS_Y = 6.0                            # battery-lead channel |y| <= 6 from the XT60 to the FC's nose edge (2 x 14 AWG, E)
RAIL_Y = 31.6                            # frame outer half width at the bosses and ears (<= 35 beside the tie saddle; brief)
RAIL_IN = 26.5                           # the side rail between the bosses: |y| 20..26.5 (on the receiver's wall top |y| 20.5..25.5, C)
BRIDGE_T = 3.5                           # the front bridge over the battery-lead channel, under the plate (z 15.0..18.5; the channel 5.4 tall)
FRONT_X = 53.3                           # front receiver bosses' nose edge (48.28 + 5)
USB_W, USB_H, USB_L = 12.0, 7.0, 250.0   # FC USB-C 90 deg plug + cable (E, layout v12)
USB_Z = 13.5                             # its centre height (layout v12, E)
FLOW_L, FLOW_W, FLOW_T = 21.6, 16.0, 8.0  # MTF-02P 21.6 x 16 (M), 8 tall kept (H, P)
FLOW_HALF = 23.0                         # view half angle: 42 FOV / 2 + 2 (H)
FLOW_WIN = 5.4                           # lens and ToF window centres +- along the long side (E)
FLOW_PLUG = 6.0                          # SH1.0-4P plug + lead keep-out past the plug end (E)
CRADLE = 2.0                             # cradle wall round the flow (rule: >= 2.0)
STRUT_R = 3.0                            # flow strut 6 x 6 (post rule), PETG
DISKS = [(54.1, 54.1), (54.1, -54.1), (-54.1, 54.1), (-54.1, -54.1)]
DISK_R, WASH = 44.5, 3.0                 # swept disk (v12 props, C), wash margin (rule 8, DEFAULT)
PROP_Z = 41.2                            # prop plane (C)
WAKE = os.environ.get("BELLY_WAKE", "contracted")   # the flow's clear-air test: "column" = plan outside every swept disk + 3 (the layout's and
                                         # the electronics session's measure); "contracted" = outside the slipstream at the body's depth
                                         # (momentum theory, E): no pose with a clear view passes the column test (search, run.log)


def wake_r(dz):
    """slipstream radius dz below the disk (momentum theory: v(z) = v_i (1 + z / sqrt(z^2 + R^2)), area x velocity constant; E)"""
    dz = max(dz, 0.0); return DISK_R / math.sqrt(1.0 + dz / math.sqrt(dz * dz + DISK_R ** 2))
PLANE = -49.62                           # bottom ring plane (v12, C)
BAND = 30.0                              # funnel band above the plane (G, design center 27 Oct)
SWAP_COL = (-43.8 - 17.6, 61.8 + 26.5, 32.8 + 30.0)   # swap column: receiver + 17.6 tail, + 26.5 nose, + 30 sides (v12, C)
G = 9.81
PETG_E, PETG_RHO, SIG_ALLOW = 2000.0, 1.27e-3, 25.0   # MPa, g/mm3, MPa (L / T, E)
INFILL, WALL = 0.20, 2.4                 # as printed: 2.4 walls + 20 % infill (the electronics session's basis, E)
INSERT_PULL, INSERT_SHEAR = 1000.0, 1000.0   # M3 x 5.7 heat-set in PETG: pull-out and bearing (N, E, published coupon range 0.9..1.5 kN)
PLATE_BEAR = 300.0                       # carbon plate bolt bearing (MPa, E)

# the drone's own parts that are not ours: the layout placeholder for this piece and its FC / wires boxes are replaced
PLACEHOLDER = "mount + nose legs (Scout)"
TAILBAY_BOX = {"tail bay mount block (Scout electronics, E)": ("belly_mount_tail_block__petg_.stl", "electronics tail block (PETG)"),
               "tail bay carrier (Scout electronics, E)": ("tail_carrier__petg_.stl", "electronics tail carrier (PETG)")}
HUNG = ["receiver v3 (latch)", "receiver lid (latch)", "receiver TPU strips (latch)", "receiver TPU end pieces (latch, E)", "XT60E1-M",
        "cartridge v6 (latch)", "slider +y (latch)", "slider -y (latch)", "cartridge v6 strips (latch, TPU)",
        "cartridge skid closed end (latch, TPU)", "cartridge skid lead end (latch, TPU)", "pack 4S 1500", "pack XT60 plug"]


def box(lo, hi): return trimesh.creation.box(bounds=[np.asarray(lo, float), np.asarray(hi, float)])


def cyl(p0, p1, r, sections=48): return trimesh.creation.cylinder(r, segment=np.vstack([p0, p1]), sections=sections)


def union(ms): return trimesh.boolean.union([m for m in ms if m is not None], engine="manifold")


def diff(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")


def centroid(m):
    if m.is_watertight and m.volume > 1.0: return np.asarray(m.center_mass, float)
    w = m.area_faces; return (m.triangles_center * w[:, None]).sum(0) / w.sum()


# ---------------------------------------------------------------- the assembled model (main's assembly, v12 poses)
def load_model():
    A = json.load(open(ASM)); P = {}
    for p in A["parts"]:
        n = p["name"]
        if n == PLACEHOLDER: continue
        if n in TAILBAY_BOX:                                            # the merged tail-bay pieces as drawn (PR 130), not the mass points
            f, nn = TAILBAY_BOX[n]; m = trimesh.load(os.path.join(ELEC, f), force="mesh")
            P[nn] = dict(mesh=m, g=float(p["mass_g"]), c=np.array(p["cg"], float), role="structure", tag="E", spec=dict(p, name=nn, mesh=None))
            continue
        if n == "wires":                                                # 20 g E at the FC (the layout's FC box): a mass point here
            P[n] = dict(mesh=box([-0.5, -0.5, 13.0], [0.5, 0.5, 14.0]), g=20.0, c=np.array([0.0, 0.0, 13.5]), role="structure", tag="E", spec=p)
            continue
        if n == "FC":                                                   # the board itself (36 x 36 x 8) on the plate's AIO holes
            P[n] = dict(mesh=fc_mesh(), g=10.0, c=np.array([FC_C[0], FC_C[1], FC_Z0 + FC_H / 2]), role="bought", tag="P", spec=p)
            continue
        if "mesh" in p: m = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
        else: x0, x1, y0, y1, z0, z1 = p["box"]; m = box([x0, y0, z0], [x1, y1, z1])
        P[n] = dict(mesh=m, g=float(p["mass_g"]), c=np.array(p["cg"], float) if "cg" in p else centroid(m), role=p["role"],
                    tag=p.get("tag", "E"), spec=p)
    sh = json.load(open(os.path.join(V12, "cage_shell.json")))
    return dict(A=A, P=P, nodes=np.array(sh["nodes"], float), rods=[tuple(e) for e in sh["rods"]], shell=sh,
                links={k: (np.array(a, float), np.array(b, float)) for k, (a, b) in sh["links"].items()} if isinstance(sh.get("links"), dict)
                else {k: (np.array(a, float), np.array(b, float)) for k, (a, b) in json.load(open(os.path.join(V12, "result.json")))["links"].items()})


def fc_mesh(z0=FC_Z0):
    h = FC_W / 2; return box([FC_C[0] - h, FC_C[1] - h, z0], [FC_C[0] + h, FC_C[1] + h, z0 + FC_H])


# ---------------------------------------------------------------- voxel occupancy of everything (1 mm), for the searches
VLO, VHI, VP = np.array([-130.0, -90.0, -62.0]), np.array([150.0, 90.0, 30.0]), 1.0


def occupancy(M, skip=(), env=False, rods=True):
    from solid_check import voxels
    shape = tuple(int(round((VHI[k] - VLO[k]) / VP)) for k in range(3))
    O = np.zeros(shape, bool)
    for n, p in M["P"].items():
        if n in skip or n in ("cage shell tubes", "spokes, bay ties, core ties, camera guard"): continue
        m = p["mesh"]; lo = np.maximum(np.floor((m.bounds[0] - VLO) / VP).astype(int) - 1, 0)
        hi = np.minimum(np.ceil((m.bounds[1] - VLO) / VP).astype(int) + 1, shape)
        if (hi <= lo).any(): continue
        sub = voxels(m, VP, VLO + lo * VP, tuple(hi - lo)) if m.is_watertight else None
        if sub is None:                                                 # not watertight: its surface, filled
            pts = m.sample(20000, seed=0) if hasattr(m, "sample") else m.vertices
            ij = np.floor((pts - VLO) / VP).astype(int); ok = ((ij >= 0) & (ij < shape)).all(1); ij = ij[ok]
            O[ij[:, 0], ij[:, 1], ij[:, 2]] = True; continue
        O[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] |= sub
    if env:                                                             # the belly mount's own envelope (the flow cannot sit in it)
        a = vidx([EAR_X0, -RAIL_Y, REC_Z]); b = vidx([FRONT_X, RAIL_Y, PLATE_Z]); O[a[0]:b[0] + 1, a[1]:b[1] + 1, a[2]:b[2] + 1] = True
    for a, b, r in (rods_all(M) if rods else []):                       # the cage tubes and links as capsules
        L = np.linalg.norm(b - a); n_ = max(int(L / 0.5), 2)
        q = a + np.linspace(0, 1, n_)[:, None] * (b - a)
        rr = int(np.ceil(r / VP))
        for dx in range(-rr, rr + 1):
            for dy in range(-rr, rr + 1):
                for dz in range(-rr, rr + 1):
                    if dx * dx + dy * dy + dz * dz > (r / VP + 0.5) ** 2: continue
                    ij = np.floor((q - VLO) / VP).astype(int) + [dx, dy, dz]; ok = ((ij >= 0) & (ij < shape)).all(1); ij = ij[ok]
                    O[ij[:, 0], ij[:, 1], ij[:, 2]] = True
    return O


def rods_all(M):
    sh = M["shell"]; r = float(sh.get("rod_r_mm", 1.5)); lr = float(sh.get("link_r_mm", 1.5))
    out = [(M["nodes"][i], M["nodes"][j], r) for i, j in M["rods"]]
    out += [(a, b, lr) for a, b in M["links"].values()]
    return out


def vidx(p): return np.floor((np.asarray(p, float) - VLO) / VP).astype(int)


def integral3(O):
    I = np.zeros(tuple(s + 1 for s in O.shape), np.int32); I[1:, 1:, 1:] = O.astype(np.int32).cumsum(0).cumsum(1).cumsum(2); return I


def box_any(I, lo, hi):
    """any occupied voxel in the world box [lo, hi]"""
    a = np.clip(vidx(lo), 0, np.array(I.shape) - 1); b = np.clip(vidx(hi) + 1, 0, np.array(I.shape) - 1)
    if (b <= a).any(): return False
    s = (I[b[0], b[1], b[2]] - I[a[0], b[1], b[2]] - I[b[0], a[1], b[2]] - I[b[0], b[1], a[2]]
         + I[a[0], a[1], b[2]] + I[a[0], b[1], a[2]] + I[b[0], a[1], a[2]] - I[a[0], a[1], a[2]])
    return s > 0


# ---------------------------------------------------------------- 1. the flow sensor: placement search
SWAP_PLAN = (-37.5 - GAP, 58.5 + GAP, 23.2 + GAP)   # the swap set's plan (cartridge, strips, skids, pack, plug; C) + 2: nothing under it


def under_swap(x0, x1, y0, y1):
    """a body in plan under the swap set: the cartridge moving straight down would hit it"""
    return x1 > SWAP_PLAN[0] and x0 < SWAP_PLAN[1] and y1 > -SWAP_PLAN[2] and y0 < SWAP_PLAN[2]


def wash_ok(x, y, r=0.0):
    return all(math.hypot(x - mx, y - my) - r >= DISK_R + WASH for mx, my in DISKS) or (abs(x) <= 51 and abs(y) <= 30)   # under the plate: its shadow


FEAS = []


def flow_search(M, O, Oc):
    """grid search: lens-down MTF-02P, long side along x (lens to the nose, plug out the tail end, as built), body bottom z_b;
    feasible = body + cradle + 2 clear, both windows' cones clear of everything below to the ground, wash, funnel band (or inside the
    swap column), inside the cage hull by 3 + the rod radius; objective = the shortest clear Ø7 strut to the belly-mount envelope,
    then |y| (the centre plane), then the lowest lens (flow / ToF minimum range)"""
    t0 = time.time()
    I = integral3(O)
    # lowest occupied z per column (the cone test: a column whose lowest point is below the lens blocks a disc of radius (z_l - h) tan)
    zs = VLO[2] + (np.arange(O.shape[2]) + 0.5) * VP
    anyc = O.any(2); first = np.argmax(O, axis=2); H = np.where(anyc, zs[first], np.inf)
    tn = math.tan(math.radians(FLOW_HALF)); R0 = 1.5                    # window radius (E)
    Rmax = int(math.ceil((40.0 - PLANE + 12) * tn)) + 3
    Mrho = {}
    for rho in range(0, Rmax + 1):
        yy, xx = np.mgrid[-rho:rho + 1, -rho:rho + 1]; fp = (xx * xx + yy * yy) <= rho * rho
        Mrho[rho] = ndi.minimum_filter(H, footprint=fp, mode="constant", cval=np.inf)

    def cone_clear(px, py, zl):
        ij = vidx([px, py, zl])[:2]
        if (ij < 0).any() or ij[0] >= H.shape[0] or ij[1] >= H.shape[1]: return False
        for rho in range(0, Rmax + 1):                                  # blocked if a column within rho (+1 pixel) has its lowest point below
            lim = zl - max(rho - R0 - 1.0, 0.0) / tn                    # zl - (rho - R0) / tan
            if Mrho[rho][ij[0], ij[1]] < lim: return False
        return True

    # the cage hull (convex, node centres) inset by node r + 3: the flow + cradle must stay inside (cage first)
    from scipy.spatial import ConvexHull
    hull = ConvexHull(M["nodes"]); Aeq = hull.equations
    def in_hull(pts, inset):
        return ((pts @ Aeq[:, :3].T + Aeq[:, 3]) <= -inset).all()
    # belly-mount anchor points (the frame's outer side and nose faces, z 10.6..17.5)
    anc = []
    for x in np.arange(-40.0, 52.0, 1.0):
        for z in np.arange(11.0, 17.01, 0.5):
            anc += [(x, RAIL_Y, z), (x, -RAIL_Y, z)]
    for y in np.arange(-RAIL_Y, RAIL_Y + 0.1, 2.0):
        for z in np.arange(11.0, 17.01, 0.5): anc.append((FRONT_X, y, z))
    for x in np.arange(-40.0, 52.0, 1.0):                              # the frame's bottom face outboard of the receiver's top (|y| > 25.9)
        for y in np.arange(26.5, RAIL_Y - 2.9, 1.0): anc += [(x, y, REC_Z), (x, -y, REC_Z)]
    anc = np.array(anc)
    st_ = np.ones((3, 3, 3), bool)
    Os = O.copy(); a_ = vidx([EAR_X0, -RAIL_Y, REC_Z]); b_ = vidx([FRONT_X, RAIL_Y, PLATE_Z])
    Os[a_[0]:b_[0] + 1, a_[1]:b_[1] + 1, a_[2]:b_[2] + 1] = False     # the strut ends on the belly mount's envelope (our own piece)
    Od = ndi.binary_dilation(Os, structure=st_, iterations=int(STRUT_R + GAP))   # strut path: 6 x 6 + 2 clear (box dilation)

    shp = np.array(O.shape)

    def seg_ok(p, a, skip_p=True, skip_a=True):
        L = float(np.linalg.norm(a - p)); n_ = max(int(L), 2); q = p + np.linspace(0, 1, n_)[:, None] * (a - p)
        keep = np.ones(len(q), bool)
        if skip_p: keep &= np.linalg.norm(q - p, axis=1) > 4.0          # the joins at the cradle and the frame
        if skip_a: keep &= np.linalg.norm(q - a, axis=1) > 4.0
        q = q[keep]
        if not len(q): return True
        ij = vidx(q)
        if not ((ij >= 0) & (ij < shp)).all(): return False
        return (not Od[ij[:, 0], ij[:, 1], ij[:, 2]].any()) and AIR[ij[:, 0], ij[:, 1], ij[:, 2]].all()

    def strut(starts):
        """the shortest 6 x 6 strut, straight or one bend (vertical then horizontal, or the reverse), from any of the cradle's
        start points to the belly mount, 2.0 clear of everything and in clear air"""
        best = None
        for p in starts:
            d1 = np.abs(anc - p).sum(1); order = np.argsort(d1)            # an L is never shorter than the straight line: rank by L1
            for k in order[:400]:
                a = anc[k]
                if best is not None and np.linalg.norm(a - p) >= best[0]: break
                L0 = float(np.linalg.norm(a - p))
                if seg_ok(p, a): best = (L0, a, p, None); continue
                for c in (np.array([p[0], p[1], a[2]]), np.array([a[0], a[1], p[2]])):
                    L = float(np.linalg.norm(c - p) + np.linalg.norm(a - c))
                    if best is not None and L >= best[0]: continue
                    if seg_ok(p, c, True, False) and seg_ok(c, a, False, True): best = (L, a, p, c)
        return best

    # wash (rule 8): a body is in clear air if it is outside every swept disk + 3, or in plan inside the shadow of the drone's core
    # above it (plate, receiver set, tie saddle, belly mount, tail bay: a column with core material above its top adds no blockage)
    zs_ = VLO[2] + (np.arange(Oc.shape[2]) + 0.5) * VP
    TOP = np.where(Oc.any(2), zs_[Oc.shape[2] - 1 - np.argmax(Oc[:, :, ::-1], axis=2)], -np.inf)
    def clear_air(x0, x1, y0, y1, ztop, column=False):
        rw = wake_r(PROP_Z - ztop) + WASH if (WAKE == "contracted" and not column) else DISK_R + WASH
        a = vidx([x0, y0, 0]); b = vidx([x1, y1, 0])
        for i in range(a[0], b[0] + 1):
            for j in range(a[1], b[1] + 1):
                if TOP[i, j] > ztop: continue
                xw, yw = VLO[0] + (i + 0.5) * VP, VLO[1] + (j + 0.5) * VP
                if any(math.hypot(xw - mx, yw - my) < rw for mx, my in DISKS): return False
        return True
    # the strut's clear air as a voxel mask (its 6 x 6 footprint: the 2D mask eroded by 3)
    xs_ = VLO[0] + (np.arange(Oc.shape[0]) + 0.5) * VP; ys_ = VLO[1] + (np.arange(Oc.shape[1]) + 0.5) * VP
    XX, YY = np.meshgrid(xs_, ys_, indexing="ij")
    AIR = np.zeros(Oc.shape, bool)
    for k, z in enumerate(zs_):
        rw = wake_r(PROP_Z - z - STRUT_R) + WASH if WAKE == "contracted" else DISK_R + WASH
        out = np.ones(XX.shape, bool)
        for mx, my in DISKS: out &= np.hypot(XX - mx, YY - my) >= rw
        AIR[:, :, k] = ndi.binary_erosion((TOP > z + STRUT_R) | out, structure=np.ones((7, 7), bool), border_value=1)
    feas = []
    n_tried = 0; kill = dict(fit=0, swap=0, band=0, cage=0, cone=0, wash=0, column_ok=0)
    for ori in ("x-", "x+", "y-", "y+"):                                # long side along x (as built) or y, the plug end toward -/+ (SENS_FLOW_ROT)
        hx, hy = (FLOW_L / 2, FLOW_W / 2) if ori[0] == "x" else (FLOW_W / 2, FLOW_L / 2)
        pl = FLOW_PLUG if ori[1] == "+" else -FLOW_PLUG
        px_, py_ = (pl, 0.0) if ori[0] == "x" else (0.0, pl)
        wx, wy = (FLOW_WIN, 0.0) if ori == "x" else (0.0, FLOW_WIN)
        for x in np.arange(-70.0, 101.0, 1.0):
            for y in np.arange(0.0, 70.1, 1.0):                         # the +y half (the -y half is its mirror but for the cage ties' sides)
                for zb in np.arange(-44.0, 9.1, 1.0):
                    n_tried += 1
                    c = CRADLE + GAP
                    bx0, bx1 = x - hx + min(px_, 0), x + hx + max(px_, 0); by0, by1 = y - hy + min(py_, 0), y + hy + max(py_, 0)
                    lo = (bx0 - c, by0 - c, zb - GAP); hi = (bx1 + c, by1 + c, zb + FLOW_T + c)
                    if box_any(I, lo, hi): kill["fit"] += 1; continue
                    if under_swap(bx0 - c, bx1 + c, by0 - c, by1 + c): kill["swap"] += 1; continue
                    in_col = (SWAP_COL[0] <= bx0 and bx1 <= SWAP_COL[1] and max(abs(by0), abs(by1)) <= SWAP_COL[2])
                    if zb < PLANE + BAND and not in_col: kill["band"] += 1; continue
                    corners = np.array([[x + sx * (hx + CRADLE), y + sy * (hy + CRADLE), zb + sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (0, FLOW_T + CRADLE)])
                    if not in_hull(corners, 6.0 + 3.0): kill["cage"] += 1; continue
                    if not (cone_clear(x + wx, y + wy, zb) and cone_clear(x - wx, y - wy, zb)): kill["cone"] += 1; continue
                    if not clear_air(x - hx - CRADLE, x + hx + CRADLE, y - hy - CRADLE, y + hy + CRADLE, zb + FLOW_T + CRADLE): kill["wash"] += 1; continue
                    if clear_air(x - hx - CRADLE, x + hx + CRADLE, y - hy - CRADLE, y + hy + CRADLE, zb + FLOW_T + CRADLE, column=True): kill["column_ok"] += 1
                    feas.append((x, y, zb, in_col, ori))
    say(f"flow search: rejected by fit {kill['fit']}, under the swap set {kill['swap']}, funnel band {kill['band']}, cage hull {kill['cage']}, cone {kill['cone']}, wash {kill['wash']}; of the feasible, {kill['column_ok']} also pass the column measure")
    say(f"flow search: {n_tried} poses, {len(feas)} feasible before the strut ({time.time() - t0:.0f} s)")
    if feas:
        F_ = np.array([f[:3] for f in feas])
        for o_ in ("x-", "x+", "y-", "y+"):
            k_ = np.array([f[4] == o_ for f in feas])
            if k_.any(): say(f"   {o_}: {k_.sum()} poses, x {F_[k_, 0].min():.0f}..{F_[k_, 0].max():.0f}, y {F_[k_, 1].min():.0f}..{F_[k_, 1].max():.0f}, z_b {F_[k_, 2].min():.0f}..{F_[k_, 2].max():.0f}")
    FEAS[:] = feas
    # objective: strut length (+ 10 inside the swap column below the band, DEFAULT: the box head's side), |y| / 10, lens height / 100
    scored = []
    for x, y, zb, in_col, ori in feas:
        if int(x) % 2 or int(y) % 2 or int(zb) % 2: continue            # the strut on a 2 mm sub-grid (time; the poses stay on 1 mm)
        hx, hy = (FLOW_L / 2, FLOW_W / 2) if ori[0] == "x" else (FLOW_W / 2, FLOW_L / 2)
        zt = zb + FLOW_T + CRADLE; zm = zb + FLOW_T / 2; ex, ey = hx + CRADLE, hy + CRADLE
        starts = [np.array(v, float) for v in ((x, y, zt), (x + ex, y, zt), (x - ex, y, zt), (x, y + ey, zt), (x, y - ey, zt),
                                                 (x + ex, y, zm), (x - ex, y, zm), (x, y + ey, zm), (x, y - ey, zm))]
        s = strut(starts)
        if s is None: continue
        pen = 10.0 if (in_col and zb < PLANE + BAND) else 0.0
        scored.append((round(s[0] + pen + abs(y) / 10.0 + (zb - PLANE) / 100.0 + abs(x) / 1000.0 + (0.0 if ori == 'x-' else 0.0001), 5), x, y, zb, s[0], tuple(np.round(s[1], 2)), tuple(np.round(s[2], 2)), None if s[3] is None else tuple(np.round(s[3], 2)), in_col, ori))
    scored.sort()
    say(f"flow search: {len(scored)} with a clear strut ({time.time() - t0:.0f} s)")
    return scored


# ---------------------------------------------------------------- 2. the FC USB-C exit (the FC's yaw follows it)
def usb_search(M, O):
    """from a port on the FC's side edge (|y| 18) at x_c, the 12 x 7 plug + cable swept 250 along an azimuth within 60 deg of
    the outward normal and an elevation of -20..+20 deg: every sample of its section clear of the occupancy (parts and cage, 1 mm)
    except the FC and our own frame; the receiver's 1.0 relief (x 0..13.5 at |y| 20.5..25, C) is where it crosses the receiver's
    wall. Objective: the angle off the outward normal (a straight exit), then the side (+y first: tie-break)."""
    Os = O.copy(); a_ = vidx([EAR_X0, -RAIL_Y, REC_Z]); b_ = vidx([FRONT_X, RAIL_Y, PLATE_Z])
    Os[a_[0]:b_[0] + 1, a_[1]:b_[1] + 1, a_[2]:b_[2] + 1] = False
    for n in ("FC", "wires"):                                           # re-add nothing of the FC: the plug leaves it
        pass
    # the parts inside the envelope must stay (receiver wall top, XT60, FC excluded): add them back but the FC
    from solid_check import voxels
    for n in ("receiver v3 (latch)", "receiver lid (latch)", "XT60E1-M", "frame plate (Bee35 carbon)", "electronics tail block (PETG)"):
        m = M["P"][n]["mesh"]; lo = np.maximum(np.floor((m.bounds[0] - VLO) / VP).astype(int) - 1, 0)
        hi = np.minimum(np.ceil((m.bounds[1] - VLO) / VP).astype(int) + 1, Os.shape)
        Os[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] |= voxels(m, VP, VLO + lo * VP, tuple(hi - lo))
    out = []
    uu, vv = np.meshgrid(np.linspace(-USB_W / 2, USB_W / 2, 7), np.linspace(-USB_H / 2, USB_H / 2, 5)); uu = uu.ravel(); vv = vv.ravel()
    for side in (1, -1):
        for xc in np.arange(5.0, 8.01, 0.5):
            p0 = np.array([xc, side * FC_W / 2, USB_Z])
            for az in np.arange(-60, 61, 2.5):
                for el in np.arange(-20, 21, 2.5):
                    a, e = math.radians(az), math.radians(el)
                    d = np.array([math.sin(a) * math.cos(e), side * math.cos(a) * math.cos(e), math.sin(e)])
                    w = np.cross([0, 0, 1.0], d); w /= np.linalg.norm(w); h = np.cross(d, w)
                    t = np.arange(0.0, USB_L + 0.1, 1.0)
                    q = p0[None, None] + t[:, None, None] * d + uu[None, :, None] * w + vv[None, :, None] * h
                    q = q.reshape(-1, 3); ij = vidx(q); ok = ((ij >= 0) & (ij < np.array(O.shape))).all(1)
                    if Os[ij[ok, 0], ij[ok, 1], ij[ok, 2]].any(): continue
                    out.append((abs(az) + abs(el) + (0 if side > 0 else 0.5), side, float(xc), float(az), float(el), d))
    out.sort(key=lambda r: r[:5])
    say(f"USB-C search: {len(out)} clear (side, x, azimuth, elevation) of {2 * 7 * 49 * 17}; best: side {'+y' if out and out[0][1] > 0 else '-y'}, "
        + (f"port x {out[0][2]:.1f}, azimuth {out[0][3]:+.1f}, elevation {out[0][4]:+.1f} deg" if out else "none"))
    return out


# ---------------------------------------------------------------- 3. the belly mount, drawn
def usb_window(side, xu, az, y_out=RAIL_Y + 8.0, clr=1.0):
    """the plug's swept footprint through the frame (12 + 2 wide, full height), along its azimuth"""
    import shapely.geometry as sg
    a = math.radians(az); d = np.array([math.sin(a), side * math.cos(a)]); w = np.array([d[1], -d[0]])
    p0 = np.array([xu, side * (FC_W / 2)]); L = (y_out - FC_W / 2) / max(math.cos(a), 0.3)
    hw = USB_W / 2 + clr
    pts = [p0 + w * hw - d * 3, p0 - w * hw - d * 3, p0 - w * hw + d * L, p0 + w * hw + d * L]
    m = trimesh.creation.extrude_polygon(sg.Polygon(pts).convex_hull, PLATE_Z - REC_Z + 2.0); m.apply_translation([0, 0, REC_Z - 1.0])
    return m


def belly_mount(side, xu, az, flow):
    """side: the USB / flow / cable side (+1 = +y); xu, az: the USB port x and azimuth; flow: dict(c=(x, y), zb, ori, post_x, post_y)"""
    S = side
    zz = (REC_Z, PLATE_Z)
    B = []
    B.append(box([TAIL_X, -EAR_Y, zz[0]], [-FC_WIN, EAR_Y, zz[1]]))                      # rear cross-member: the tongue's bearing face
    for sy in (1, -1):
        y0, y1 = sorted((sy * EAR_Y, sy * RAIL_Y))
        B.append(box([EAR_X0, y0, zz[0]], [TAIL_X, y1, zz[1]]))                          # ear: inner face on the tail block (|y| 21)
        y0, y1 = sorted((sy * FC_WIN, sy * RAIL_IN))
        B.append(box([TAIL_X - 0.01, y0, zz[0]], [REC_BOLTS[2][0], y1, zz[1]]))           # side rail (on the receiver's wall top)
    for x_, y_ in REC_BOLTS + PLATE_BOLTS: B.append(cyl([x_, y_, zz[0]], [x_, y_, zz[1]], BOSS_R))
    for sy in (1, -1):                                                                    # front blocks to the front plate bolts
        y0, y1 = sorted((sy * LEADS_Y, sy * FC_WIN))
        B.append(box([24.0, y0, zz[0]], [36.0, y1, zz[1]]))
    B.append(box([24.0, -LEADS_Y - 0.01, zz[1] - BRIDGE_T], [36.0, LEADS_Y + 0.01, zz[1]]))   # the bridge over the lead channel
    # the flow post (6 x 6) from the rail's underside down to the cradle, a pad on the rail, 2 gussets
    fx, fy = flow["c"]; zb = flow["zb"]; hx, hy = (FLOW_L / 2, FLOW_W / 2) if flow["ori"][0] == "x" else (FLOW_W / 2, FLOW_L / 2)
    px = flow["post_x"]; py = flow["post_y"]; ztop_c = zb + FLOW_T + CRADLE
    B.append(box([px - STRUT_R, py - STRUT_R, ztop_c - 0.01], [px + STRUT_R, py + STRUT_R, zz[0] + 0.01]))
    FS = 1 if py > 0 else -1
    yp0, yp1 = sorted((FS * (FC_WIN + 1.0), py + FS * STRUT_R))
    B.append(box([px - STRUT_R, yp0, zz[0]], [px + STRUT_R, yp1, zz[1]]))               # the post's pad: the rail widened over it
    for gx in (-1, 1):                                                                     # gussets fore and aft of the post into the rail pad
        x0 = px + gx * STRUT_R; x1 = px + gx * (STRUT_R + 6.0)
        tri = np.array([[x0, zz[0] - 8.0], [x0, zz[0] + 0.01], [x1, zz[0] + 0.01]])
        B.append(extrude_xz(tri, py - 1.5, py + 1.5))
    # the cradle: 2.0 walls round the flow, a 2.0 top, 2 bottom lips 2.0 x 2.0 along the long sides, the plug end open
    c0 = np.array([fx - hx - CRADLE, fy - hy - CRADLE, zb - CRADLE]); c1 = np.array([fx + hx + CRADLE, fy + hy + CRADLE, ztop_c])
    cr = box(c0, c1)
    cuts_c = [box([fx - hx, fy - hy, zb], [fx + hx, fy + hy, zb + FLOW_T])]              # the pocket (the flow's faces: declared contact)
    if flow["ori"][0] == "x":
        lip = 2.0; cuts_c.append(box([fx - hx, fy - hy + lip, zb - CRADLE - 1], [fx + hx, fy + hy - lip, zb + 0.01]))   # windows open below
        pe = fx - hx if flow["ori"][1] == "-" else fx + hx                                 # the plug end wall open
        cuts_c.append(box([pe - CRADLE - 1 if flow["ori"][1] == "-" else pe - 0.01, fy - hy + lip, zb - CRADLE - 1],
                          [pe + 0.01 if flow["ori"][1] == "-" else pe + CRADLE + 1, fy + hy - lip, zb + FLOW_T]))
    else:
        lip = 2.0; cuts_c.append(box([fx - hx + lip, fy - hy, zb - CRADLE - 1], [fx + hx - lip, fy + hy, zb + 0.01]))
        pe = fy - hy if flow["ori"][1] == "-" else fy + hy
        cuts_c.append(box([fx - hx + lip, pe - CRADLE - 1 if flow["ori"][1] == "-" else pe - 0.01, zb - CRADLE - 1],
                          [fx + hx - lip, pe + 0.01 if flow["ori"][1] == "-" else pe + CRADLE + 1, zb + FLOW_T]))
    B.append(diff(cr, cuts_c))
    body = union(B)
    cuts = []
    cuts.append(box([-FC_WIN, -FC_WIN, zz[0] - 1], [FC_WIN, FC_WIN, zz[1] + 1]))        # FC window (board + 2.0)
    cuts.append(box([FC_WIN - 0.01, -LEADS_Y, zz[0] - 1], [XT60_KO[0] + 0.5, LEADS_Y, zz[1] - BRIDGE_T]))   # battery-lead channel to the XT60 (under the bridge)
    gx = flow["post_x"] - STRUT_R - 2.0; fsy = 1 if flow["post_y"] > 0 else -1     # the flow lead's groove under the rail (2.5 x 4, laid before the receiver)
    y0, y1 = sorted((fsy * (FC_WIN - 1.0), fsy * (RAIL_IN + 1.0)))
    cuts.append(box([gx - 4.0, y0, zz[0] - 1], [gx, y1, zz[0] + 2.5]))
    cuts.append(box([FC_WIN - 0.01, -FC_WIN, zz[0] - 1], [24.0, FC_WIN, zz[1] + 1]))      # the FC's nose-edge pads
    cuts.append(box([XT60_KO[0], -XT60_KO[2], zz[0] - 1], [XT60_KO[1], XT60_KO[2], zz[1] + 1]))   # XT60E1-M + its rear leads
    cuts.append(usb_window(S, xu, az))                                                   # the USB-C + cable window (full height)
    for x_, y_ in REC_BOLTS:                                                              # receiver inserts from below
        cuts.append(cyl([x_, y_, zz[0] - 1], [x_, y_, zz[0] + INSERT[1]], INSERT[0] / 2, 32))
    for x_, y_ in PLATE_BOLTS:                                                            # plate-bolt inserts from above
        cuts.append(cyl([x_, y_, zz[1] - INSERT[1]], [x_, y_, zz[1] + 1], INSERT[0] / 2, 32))
    return diff(body, cuts)


def extrude_xz(poly, y0, y1):
    """a prism: polygon in (x, z), extruded along y from y0 to y1"""
    import shapely.geometry as sg
    m = trimesh.creation.extrude_polygon(sg.Polygon(poly), y1 - y0)       # polygon in (x, y) plane, extruded along +z
    T = np.array([[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0], [0, 0, 0, 1.0]])   # (x, y', z') -> (x, z', y'): the polygon's y is z
    m.apply_transform(T)
    if m.volume < 0: m.invert()
    return m


def flow_mesh(flow):
    fx, fy = flow["c"]; zb = flow["zb"]; hx, hy = (FLOW_L / 2, FLOW_W / 2) if flow["ori"][0] == "x" else (FLOW_W / 2, FLOW_L / 2)
    return box([fx - hx, fy - hy, zb], [fx + hx, fy + hy, zb + FLOW_T])


def printed_g(m, pitch=0.4):
    """as printed: 2.4 walls solid + 20 % infill inside (E); the eroded interior by a distance transform on a 0.4 grid"""
    from solid_check import voxels
    lo = m.bounds[0] - 2 * pitch; hi = m.bounds[1] + 2 * pitch
    shape = tuple(int(np.ceil((hi[k] - lo[k]) / pitch)) for k in range(3))
    V = voxels(m, pitch, lo, shape); d = ndi.distance_transform_edt(V) * pitch
    vin = (d > WALL).sum() * pitch ** 3; vall = V.sum() * pitch ** 3
    return (vall - vin + INFILL * vin) * PETG_RHO, vall * PETG_RHO, (V, lo, pitch, d)


# ---------------------------------------------------------------- 4. fit: every pair within 10 of our pieces, >= 2.0 or a declared contact
OURS = ("belly mount (PETG)", "flow MTF-02P", "FC")
CONTACT = {frozenset(p) for p in (("belly mount (PETG)", "frame plate (Bee35 carbon)"), ("belly mount (PETG)", "receiver v3 (latch)"),
                                  ("belly mount (PETG)", "receiver lid (latch)"), ("belly mount (PETG)", "electronics tail block (PETG)"),
                                  ("belly mount (PETG)", "flow MTF-02P"))}


def overlap(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold"); return float(abs(i.volume)) if len(i.faces) else 0.0
    except Exception: return float("nan")


def min_gap(a, b, n=6000):
    pa = np.vstack([a.vertices, a.sample(n, seed=1)]); pb = np.vstack([b.vertices, b.sample(n, seed=2)])
    da = trimesh.proximity.ProximityQuery(b).signed_distance(pa); db = trimesh.proximity.ProximityQuery(a).signed_distance(pb)
    if (da > 1e-3).any() or (db > 1e-3).any(): return 0.0
    return float(min((-da).min(), (-db).min()))


def fit_table(P, rods):
    rows = []
    names = list(P)
    for a in OURS:
        for b in names:
            if b == a or b == "wires" or (b in OURS and OURS.index(b) < OURS.index(a)): continue
            ma, mb = P[a]["mesh"], P[b]["mesh"]
            if (ma.bounds[0] - 10 > mb.bounds[1]).any() or (mb.bounds[0] - 10 > ma.bounds[1]).any(): continue
            g = min_gap(ma, mb)
            if g > 10.0: continue
            ov = overlap(ma, mb) if g < 0.5 else 0.0
            dec = frozenset((a, b)) in CONTACT
            ok = (ov <= 0.05) if dec else (g >= GAP - 1e-6)
            rows.append(dict(a=a, b=b, overlap_mm3=round(ov, 3), gap_mm=round(g, 2), kind="declared contact" if dec else ">= 2", pass_=bool(ok)))
    lower = trimesh.boolean.intersection([P["belly mount (PETG)"]["mesh"], box([-100, -100, -80], [100, 100, REC_Z - 0.6])], engine="manifold")
    for b in ("receiver v3 (latch)", "receiver TPU end pieces (latch, E)", "tie saddle (printed)"):   # the post, gussets and cradle alone
        g = min_gap(lower, P[b]["mesh"])
        rows.append(dict(a="belly mount below z 9.0 (post, gussets, cradle)", b=b, overlap_mm3=0.0, gap_mm=round(g, 2), kind=">= 2", pass_=bool(g >= GAP)))
    for a in OURS:                                                      # the cage tubes and links (capsules)
        m = P[a]["mesh"]; pts = np.vstack([m.vertices, m.sample(6000, seed=3)]); best = (np.inf, None)
        for p, q, r in rods:
            u = q - p; t = np.clip(((pts - p) @ u) / (u @ u), 0, 1); d = np.linalg.norm(pts - (p + t[:, None] * u), axis=1).min() - r
            if d < best[0]: best = (d, (np.round(p, 1).tolist(), np.round(q, 1).tolist()))
        rows.append(dict(a=a, b="cage rods / links" + ("" if best[0] <= 10.0 else " (nearest)"), overlap_mm3=0.0, gap_mm=round(float(best[0]), 2),
                         kind=">= 2", pass_=bool(best[0] >= GAP)))
    return rows


# ---------------------------------------------------------------- 5. loads (dimensions.md L), bolt groups, sections, the flow post
def cases():
    out = [("hover 1 g", np.array([0, 0, -1.0]), "flight", 0.5), ("flight 4 g thrust", np.array([0, 0, -4.0]), "flight", 0.5)]
    for d, l in (((1, 0, 0), "x"), ((-1, 0, 0), "-x"), ((0, 1, 0), "y"), ((0, -1, 0), "-y")):
        out.append((f"flight 2 g {l}", 2.0 * np.array(d, float), "flight", 0.5))
    out.append(("landing 6 g", np.array([0, 0, -6.0]), "landing", 0.5))
    dirs = [((1, 0, 0), "x"), ((-1, 0, 0), "-x"), ((0, 1, 0), "y"), ((0, -1, 0), "-y"), ((0, 0, 1), "z"), ((0, 0, -1), "-z")]
    dirs += [((sx, sy, sz), f"{'+' if sx > 0 else '-'}x{'+' if sy > 0 else '-'}y{'+' if sz > 0 else '-'}z") for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for d, l in dirs:
        v = np.array(d, float); out.append((f"crash 35 g {l}", 35.0 * v / np.linalg.norm(v), "crash", 1.0))
    return out


def bolt_group(F, c, bolts, z0):
    """elastic bolt group in the plane z0: inertia F (N) at c; per bolt (tension N, shear N); compression goes into bearing"""
    B_ = np.array([[x, y] for x, y in bolts], float); g0 = np.r_[B_.mean(0), z0]
    r = np.asarray(c, float) - g0; Mo = np.cross(r, F)
    xi, yi = B_[:, 0] - g0[0], B_[:, 1] - g0[1]
    f = -F[2] / len(B_) - Mo[0] * yi / (yi ** 2).sum() + Mo[1] * xi / (xi ** 2).sum()
    T = np.maximum(f, 0.0)
    J = (xi ** 2 + yi ** 2).sum()
    sx = -F[0] / len(B_) + Mo[2] * yi / J; sy = -F[1] / len(B_) - Mo[2] * xi / J
    return T, np.hypot(sx, sy)


def section_props(mesh, origin, normal, hollow=True):
    """a cut of the mesh: area, I about its horizontal centroidal axis (bending under a vertical load), c, Z; hollow = the 2.4 walls only"""
    s = mesh.section(plane_origin=origin, plane_normal=normal)
    if s is None: return None
    P2, T = s.to_2D(normal=normal)
    from shapely.ops import unary_union, triangulate
    polys = list(P2.polygons_full)
    if not polys: return None
    G_ = unary_union(polys)
    if hollow: G_ = G_.difference(G_.buffer(-WALL))
    up2 = np.array([T[2, 0], T[2, 1]]); up2 = up2 / max(np.linalg.norm(up2), 1e-9)   # T maps the 2D frame to the world: world z in 2D
    tri = []
    for g in getattr(G_, "geoms", [G_]):
        for t in triangulate(g):
            if t.representative_point().within(g.buffer(1e-6)): tri.append(np.array(t.exterior.coords)[:3])
    if not tri: return None
    tri = np.array(tri); v = tri @ up2
    a = 0.5 * np.abs((tri[:, 1, 0] - tri[:, 0, 0]) * (tri[:, 2, 1] - tri[:, 0, 1]) - (tri[:, 2, 0] - tri[:, 0, 0]) * (tri[:, 1, 1] - tri[:, 0, 1]))
    A = a.sum(); vc = (a * v.mean(1)).sum() / A
    I = (a * (v[:, 0] ** 2 + v[:, 1] ** 2 + v[:, 2] ** 2 + v[:, 0] * v[:, 1] + v[:, 1] * v[:, 2] + v[:, 0] * v[:, 2]) / 6.0).sum() - A * vc ** 2
    cm = max(v.max() - vc, vc - v.min())
    return dict(A=round(float(A), 1), I=round(float(I), 1), c=round(float(cm), 2), Z=round(float(I / cm), 1) if cm > 0 else 0.0)


def loads(P, bm, bm_g, bm_c, flow, cradle_g):
    """L cases on the hung set (receiver set, cartridge set, pack), the belly mount's own mass, the flow + cradle, the tail tongue"""
    hung = [(P[n]["g"], P[n]["c"]) for n in HUNG]
    mh = sum(g for g, _ in hung); ch = sum(g * c for g, c in hung) / mh
    fl_c = np.array([flow["c"][0], flow["c"][1], flow["zb"] + FLOW_T / 2]); fl_g = 1.5 + cradle_g
    bay_g = sum(P[n]["g"] for n in ("electronics tail block (PETG)", "electronics tail carrier (PETG)", "Radxa Zero 3W", "UBEC", "XR2")) + 0.5 * 6.27
    tongue_c = np.array([TAIL_X, 3.0, 14.05])                          # the tongue's face centre (y -10..16, z 9.6..18.5, C)
    tongue_A = 26.0 * (PLATE_Z - REC_Z)
    secs = {}
    frame_only = trimesh.boolean.intersection([bm, box([-100, -100, REC_Z - 0.01], [100, 100, 30])], engine="manifold")   # the frame (no post)
    for nm, (o_, n_) in {"rear cross-member + ears at x -31 (behind: the rear receiver inserts)": ([-31.3, 0, 14.0], [1, 0, 0]),
                         "side rail at x 0 (the FC window; the USB side is cut)": ([0.0, 0, 14.0], [1, 0, 0]),
                         "front blocks + bridge at x 30 (the front plate inserts)": ([30.0, 0, 14.0], [1, 0, 0]),
                         "front rails at x 42": ([42.0, 0, 14.0], [1, 0, 0])}.items():
        secs[nm] = section_props(frame_only, o_, n_)
    s_ = bm.section(plane_origin=[flow["post_x"], flow["post_y"], REC_Z - 12.0], plane_normal=[0, 0, 1])   # the post below the gussets
    import shapely.geometry as sg
    pa = max((sg.Polygon(e[:, :2]).area for e in s_.discrete if abs(e[:, 0].mean() - flow["post_x"]) < 5 and abs(e[:, 1].mean() - flow["post_y"]) < 5), default=0.0) if s_ is not None else 0.0
    post_root = dict(A=round(pa, 1), Z=round(pa ** 1.5 / 6.0, 1))       # a square post: Z = b^3 / 6 (solid; 6 wide is all wall)
    lever = [min(np.hypot(bx - px_, by - py_) for px_, py_ in PLATE_BOLTS) for bx, by in REC_BOLTS]
    rows = []
    for name, a, kind, lim in cases():
        acc = a * G / 1000.0                                            # N per g of mass
        Fh = mh * acc
        T1, S1 = bolt_group(Fh, ch, REC_BOLTS, REC_Z)
        rec_idx = float(np.max(np.sqrt((T1 / INSERT_PULL) ** 2 + (S1 / INSERT_SHEAR) ** 2)))
        Ff = fl_g * acc
        Ft = np.array([max(a[0], 0.0) * bay_g * G / 1000.0, 0, 0]) if kind == "crash" else np.zeros(3)   # the tongue bears on our face (+x only)
        Fb = bm_g * acc
        Fall = Fh + Ff + Fb
        call = (mh * ch + fl_g * fl_c + bm_g * np.asarray(bm_c)) / (mh + fl_g + bm_g)
        T2, S2 = bolt_group(Fall, call, PLATE_BOLTS, PLATE_Z)
        if Ft[0] > 0:
            T2t, S2t = bolt_group(Ft, tongue_c, PLATE_BOLTS, PLATE_Z); T2 = T2 + T2t; S2 = S2 + S2t
        pl_idx = float(np.max(np.sqrt((T2 / INSERT_PULL) ** 2 + (S2 / INSERT_SHEAR) ** 2)))
        carbon_idx = float(np.max(S2) / (3.0 * 3.5 * PLATE_BEAR))      # M3 bearing in the 3.5 carbon plate
        Mb = float(max(T1[i] * lever[i] for i in range(4)))            # a receiver insert's pull carried to the nearest plate insert (cantilever)
        zmin = min(v["Z"] for v in secs.values() if v and v["Z"] > 0)
        frame_idx = Mb / zmin / SIG_ALLOW
        Lp = REC_Z - (flow["zb"] + FLOW_T / 2)
        Mp = float(np.hypot(Ff[0], Ff[1]) * Lp); post_idx = (Mp / post_root["Z"] + abs(Ff[2]) / post_root["A"]) / SIG_ALLOW if post_root and post_root["Z"] > 0 else float("nan")
        tongue_idx = Ft[0] / tongue_A / SIG_ALLOW
        rows.append(dict(case=name, kind=kind, limit=lim, rec_T=round(float(T1.max()), 1), rec_S=round(float(S1.max()), 1), rec_idx=round(rec_idx, 4),
                         plate_T=round(float(T2.max()), 1), plate_S=round(float(S2.max()), 1), plate_idx=round(pl_idx, 4),
                         frame_M=round(Mb, 0), frame_idx=round(frame_idx, 4), post_idx=round(post_idx, 4), tongue_N=round(float(Ft[0]), 1),
                         tongue_idx=round(tongue_idx, 4), carbon_idx=round(carbon_idx, 4)))
    return dict(rows=rows, hung_g=round(mh, 1), hung_cg=np.round(ch, 2).tolist(), flow_g=round(fl_g, 2), bay_g=round(bay_g, 1),
                sections=secs, post_root=post_root, post_L=round(REC_Z - (flow["zb"] + FLOW_T / 2), 1), levers=np.round(lever, 1).tolist())


# ---------------------------------------------------------------- 6. FC on grommets and the flow post: first modes (E)
def fc_modes(E_mpa):
    """the FC (10 g, 36 x 36 x 8) on 4 soft grommets at +-12.75; M2 grommet in the board's Ø3 hole (E): flanges Ø5.5 x 1.0 each side,
    neck Ø3.0 / bore Ø2.0 through the 1.6 PCB, both flanges preloaded (in parallel); compression modulus with the shape factor
    (E_c = E (1 + 2 k S^2), k 0.6), shear G = E / 3; the screws, spacers and plate rigid against them"""
    A = math.pi / 4 * (5.5 ** 2 - 2.0 ** 2); t = 1.0; S_ = A / (math.pi * (5.5 + 2.0) * t); Ec = E_mpa * (1 + 2 * 0.6 * S_ ** 2)
    kv = 2 * Ec * A / t                                                 # N/mm per grommet, vertical
    kr = Ec * 2.5 * 1.6 / 0.5 + 2 * (E_mpa / 3.0) * A / t              # lateral: the neck in radial compression + the 2 flanges in shear
    m = 0.010; Ixx = m * (FC_W ** 2 + FC_H ** 2) / 12 * 1e-6
    f = lambda k, mm: math.sqrt(k / mm) / (2 * math.pi)
    fv = f(4 * kv * 1e3, m); fl = f(4 * kr * 1e3, m)
    frock = f(4 * kv * 1e3 * (FC_HOLES * 1e-3) ** 2, Ixx)
    return dict(E_MPa=E_mpa, kv_N_mm=round(kv, 1), kr_N_mm=round(kr, 1), vertical_Hz=round(fv), lateral_Hz=round(fl), rocking_Hz=round(frock),
                first_Hz=round(min(fv, fl, frock)))


def fc_rigid_mode():
    """the FC hard on its 4 screws (no grommet): each M2 steel screw a cantilever over the 4.2 spacer gap, lateral (E, lower bound)"""
    E_s = 200e3; d = 2.0; I = math.pi * d ** 4 / 64; L = 4.2
    k = 3 * E_s * I / L ** 3
    return round(math.sqrt(4 * k * 1e3 / 0.010) / (2 * math.pi))


def post_mode(L, m_g):
    """the flow + cradle on the 6 x 6 PETG post (cantilever with a tip mass, E)"""
    I = 6.0 ** 4 / 12.0; k = 3 * PETG_E * I / L ** 3
    return round(math.sqrt(k * 1e3 / (m_g / 1000.0)) / (2 * math.pi))


# ---------------------------------------------------------------- 7. the system model with our pieces: system_check
def manifest(M, P, usb, flow, bm, bm_g, bm_c):
    A = json.loads(json.dumps(M["A"]))
    keep = {n for n in P}
    parts = []
    for p in A["parts"]:
        n = p["name"]
        if n == PLACEHOLDER or n in TAILBAY_BOX or n in ("FC", "wires", "flow MTF-02P"): continue
        q = dict(p)
        if "mesh" in q: q["mesh"] = os.path.join(ROOT, q["mesh"])
        if "cg" not in q and "mesh" in q: q["cg"] = np.round(P[n]["c"], 3).tolist()
        parts.append(q)
    for n, src in (("electronics tail block (PETG)", "belly_mount_tail_block__petg_.stl"), ("electronics tail carrier (PETG)", "tail_carrier__petg_.stl")):
        parts.append(dict(name=n, role="structure", mesh=os.path.join(ELEC, src), material="PETG", mass_g=P[n]["g"], cg=np.round(P[n]["c"], 2).tolist(),
                          tag="E", owner="scout electronics", version="v2 (PR 130)"))
    parts.append(dict(name="FC", role="bought", protect=True, trimesh=P["FC"]["mesh"], mass_g=10.0, tag="P", geom_tag="C",
                      note="MicoAir743v2-AIO 36 x 36 x 8 on the plate's AIO holes (as built), z 9.5..17.5"))
    parts.append(dict(name="wires", role="structure", box=[-0.5, 0.5, -0.5, 0.5, 13.0, 14.0], mass_g=20.0, tag="E", note="the layout's 20 g, at the FC"))
    parts.append(dict(name="flow MTF-02P", role="bought", protect=True, trimesh=P["flow MTF-02P"]["mesh"], mass_g=1.5, tag="P", geom_tag="C"))
    parts.append(dict(name="belly mount (PETG)", role="structure", trimesh=bm, material="PETG", mass_g=round(bm_g, 2), cg=np.round(bm_c, 2).tolist(),
                      tag="E", geom_tag="C", owner="scout electronics", version="belly mount v1 (concept)", source="nox/scout/v3/belly_mount.py"))
    A["parts"] = parts
    names = {p["name"] for p in parts}
    cp = [c for c in A["contact_pairs"] if set(c[:2]) <= names and not ({"FC", "wires", "flow MTF-02P"} & set(c[:2]))]
    cp += [[a, b, 0.05] for a, b in (tuple(x) for x in CONTACT)]
    cp += [[a, b, 0.05] for a, b in (("electronics tail block (PETG)", "frame plate (Bee35 carbon)"), ("electronics tail block (PETG)", "receiver v3 (latch)"),
                                      ("electronics tail carrier (PETG)", "electronics tail block (PETG)"), ("electronics tail carrier (PETG)", "Radxa Zero 3W"),
                                      ("electronics tail carrier (PETG)", "UBEC"), ("electronics tail carrier (PETG)", "XR2"))]
    cp += [["wires", n, 1e9] for n in names if n != "wires"]
    A["contact_pairs"] = cp
    A["clearance_pairs"] = [["belly mount (PETG)", n, GAP] for n in ("XT60E1-M", "FC", "tie saddle (printed)", "receiver TPU end pieces (latch, E)")] + \
                           [["flow MTF-02P", n, GAP] for n in ("receiver v3 (latch)", "tie saddle (printed)")] + [["FC", n, GAP] for n in ("frame plate (Bee35 carbon)", "receiver v3 (latch)")]
    acc = []
    for a in A["access"]:
        a = dict(a)
        a["ignore"] = [("belly mount (PETG)" if i == PLACEHOLDER else i) for i in a.get("ignore", []) if i != "wires"] + ["wires"]
        if a["name"].startswith("FC USB-C"):
            a["from"] = np.round(usb["p0"], 2).tolist(); a["dir"] = np.round(usb["d"], 4).tolist()
            w = np.cross([0, 0, 1.0], usb["d"]); w /= np.linalg.norm(w)
            a["tool"] = {"type": "box", "w": USB_W, "h": USB_H, "w_axis": np.round(w, 4).tolist()}
            a["name"] = f"FC USB-C cable (12 x 7, from the FC's {'left' if usb['side'] > 0 else 'right'} edge, E)"
            a["ignore"] = ["wires"]
        acc.append(a)
    for x_, y_ in PLATE_BOLTS:
        acc.append(dict(name=f"M3 key, belly-mount plate bolt ({x_:.0f}, {y_:+.0f})", part="belly mount (PETG)", **{"from": [x_, y_, PLATE_TOP + 0.5]},
                        dir=[0, 0, 1], length_mm=150.0, tool={"type": "cylinder", "d": 4}, ignore=["wires"]))
    for sx in (1, -1):
        for sy in (1, -1):
            acc.append(dict(name=f"M2 key, FC grommet screw ({sx * FC_HOLES:+.2f}, {sy * FC_HOLES:+.2f})", part="FC",
                            **{"from": [FC_C[0] + sx * FC_HOLES, FC_C[1] + sy * FC_HOLES, PLATE_TOP + 0.5]}, dir=[0, 0, 1], length_mm=150.0,
                            tool={"type": "cylinder", "d": 4}, ignore=["wires"]))
    fx, fy = flow["c"]; hx = FLOW_L / 2 if flow["ori"][0] == "x" else FLOW_W / 2
    pe = (fx - hx if flow["ori"][1] == "-" else fx + hx)
    acc.append(dict(name="flow SH1.0 plug out of its end (6 x 4, E)", part="flow MTF-02P", **{"from": [pe, fy, flow["zb"] + 3.25]},
                    dir=[-1.0 if flow["ori"][1] == "-" else 1.0, 0, 0], length_mm=25.0, tool={"type": "box", "w": 6.0, "h": 4.0}, ignore=["wires"]))
    env = json.load(open(os.path.join(ELEC, "envelope.json")))          # the tail bay's own keys and tool lines (PR 130), kept clear of us
    for k, v in env["keep_clear"]["keys"].items():
        acc.append(dict(name=k, part=("electronics tail carrier (PETG)" if "yoke" in k else "electronics tail block (PETG)"), **{"from": v["start"]},
                        dir=v["dir"], length_mm=150.0, tool={"type": "cylinder", "d": v["d"]}, ignore=["wires"]))
    for k, v in env["keep_clear"]["tool_lines"].items():
        acc.append(dict(name=f"Radxa {k} (Ø{v['d']:g})", part="Radxa Zero 3W", **{"from": v["start"]}, dir=v["dir"], length_mm=150.0,
                        tool={"type": "cylinder", "d": v["d"]}, ignore=["wires"]))
    A["access"] = acc
    A["name"] = "Scout v3 with the belly mount v1 (concept)"
    A["note"] = "main's Scout v3 assembly (v12, camera mount v1) with the belly mount v1, the FC as drawn, the flow re-placed, the tail block and carrier as drawn (PR 130)"
    return A


def run_system_check(man):
    from system_check import run, report
    r = run(man); return r, report(r)


# ---------------------------------------------------------------- 8. picture
COLS = {"belly mount (PETG)": "#2f7fd8", "FC": "#2e7d32", "flow MTF-02P": "#1b5e20", "frame plate (Bee35 carbon)": "#3a3a3a",
        "electronics tail block (PETG)": "#8e6bbf", "electronics tail carrier (PETG)": "#b49ad6", "tie saddle (printed)": "#c77d2e",
        "XT60E1-M": "#c9a400", "receiver v3 (latch)": "#8899aa", "receiver lid (latch)": "#8899aa"}


def preview(P, M, bm, flow, usb, path, info):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle, Polygon as MPoly
    fig = plt.figure(figsize=(20, 13.5))
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1.05])
    views = [("side (from +y): x, z", (0, 2), 1), ("top: x, y", (0, 1), 1), ("rear (from -x): y, z", (1, 2), -1)]
    lims = {(0, 2): ((-75, 105), (-58, 48)), (0, 1): ((-75, 105), (-70, 70)), (1, 2): ((-75, 75), (-58, 48))}
    rng = np.random.default_rng(0)
    for k, (vt, (i, j), flip) in enumerate(views):
        ax = fig.add_subplot(gs[0, k]); ax.set_aspect("equal")
        ax.set_title(f"belly mount v1 (concept) in the Scout v3, {vt}", fontsize=10)
        def poly(m, fc, ec="none", a=0.3, z=1, lw=0.15):
            V = m.vertices[:, [i, j]] * [flip, 1]; Fc = m.faces
            if len(Fc) > 7000: Fc = Fc[rng.choice(len(Fc), 7000, replace=False)]
            ax.add_collection(PolyCollection(V[Fc], facecolor=fc, edgecolor=ec, alpha=a, linewidths=lw, zorder=z))
        for n, p in P.items():
            if n in ("cage shell tubes", "spokes, bay ties, core ties, camera guard", "wires"): continue
            if n.startswith("prop"): continue
            c = COLS.get(n, "#4f6fcf" if p["spec"].get("swap") else "#a8b3bf")
            if n == "belly mount (PETG)": poly(p["mesh"], c, "#174a85", 0.75, 6, 0.2)
            elif n in ("FC", "flow MTF-02P"): poly(p["mesh"], c, "k", 0.8, 7, 0.3)
            else: poly(p["mesh"], c, a=0.16 if n not in COLS else 0.3)
        for a_, b_, r_ in rods_all(M):
            ax.plot([flip * a_[i], flip * b_[i]], [a_[j], b_[j]], color="#222", lw=0.8, zorder=3)
        if (i, j) == (0, 1):
            for mx, my in DISKS: ax.add_patch(plt.Circle((mx, my), DISK_R, color="#e8c547", alpha=0.10, zorder=0))
            for x_, y_ in REC_BOLTS: ax.plot(x_, y_, "o", ms=4, mfc="w", mec="#174a85", zorder=9)
            for x_, y_ in PLATE_BOLTS: ax.plot(x_, y_, "s", ms=4, mfc="#c62828", mec="k", zorder=9)
            for sx in (1, -1):
                for sy in (1, -1): ax.plot(sx * FC_HOLES, sy * FC_HOLES, "x", color="w", ms=5, zorder=9)
        # the USB-C path and the flow's view cone
        q = usb["p0"] + 90 * usb["d"]
        ax.annotate("", xy=(flip * q[i], q[j]), xytext=(flip * usb["p0"][i], usb["p0"][j]), arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.6), zorder=10)
        ax.text(flip * q[i], q[j], " FC USB-C", color="#c62828", fontsize=8, zorder=10, clip_on=True)
        if j == 2:
            fx, fy = flow["c"]; zl = flow["zb"]; d_ = zl - PLANE + 10; t_ = math.tan(math.radians(FLOW_HALF))
            cx = fx if i == 0 else fy * flip
            ax.add_patch(MPoly([[cx - FLOW_WIN * (i == 0), zl], [cx - FLOW_WIN * (i == 0) - d_ * t_, zl - d_], [cx + FLOW_WIN * (i == 0) + d_ * t_, zl - d_],
                                [cx + FLOW_WIN * (i == 0), zl]], closed=True, color="#43a047", alpha=0.18, zorder=2))
            ax.axhline(PLANE + BAND, color="#c62828", lw=0.7, ls="--", zorder=2); ax.text(lims[(i, j)][0][0] + 2, PLANE + BAND + 1, "funnel band top", color="#c62828", fontsize=7)
        (x0, x1), (y0, y1) = lims[(i, j)]
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up 1: a slice at z 14 (the frame, the FC window, the tongue, XT60, the USB window)
    ax = fig.add_subplot(gs[1, 0]); ax.set_aspect("equal"); ax.set_title("close-up: slice at z 14 (mid frame), top view", fontsize=10)
    for n in ("belly mount (PETG)", "FC", "electronics tail block (PETG)", "XT60E1-M", "receiver v3 (latch)", "tie saddle (printed)", "receiver lid (latch)"):
        s = P[n]["mesh"].section(plane_origin=[0, 0, 14.0 if n not in ("receiver v3 (latch)", "receiver lid (latch)") else 9.0], plane_normal=[0, 0, 1])
        if s is None: continue
        P2, T = s.to_2D()
        for pg in P2.polygons_full:
            pts = np.c_[np.array(pg.exterior.coords), np.zeros(len(pg.exterior.coords)), np.ones(len(pg.exterior.coords))] @ T.T
            ax.add_patch(MPoly(pts[:, :2], closed=True, facecolor=COLS.get(n, "#a8b3bf"), edgecolor="k", lw=0.4,
                               alpha=0.85 if n in ("belly mount (PETG)", "FC") else 0.35, zorder=5 if n == "belly mount (PETG)" else 3))
            for h in pg.interiors:
                hp = np.c_[np.array(h.coords), np.zeros(len(h.coords)), np.ones(len(h.coords))] @ T.T
                ax.add_patch(MPoly(hp[:, :2], closed=True, facecolor="w", edgecolor="k", lw=0.3, zorder=6))
    for x_, y_ in REC_BOLTS: ax.text(x_, y_ - 7.5, "rcv M3", fontsize=6.5, ha="center", zorder=9)
    for x_, y_ in PLATE_BOLTS: ax.text(x_, y_ + 5.5, "plate M3", fontsize=6.5, ha="center", color="#c62828", zorder=9)
    for sx in (1, -1):
        for sy in (1, -1): ax.plot(sx * FC_HOLES, sy * FC_HOLES, "o", ms=4, mfc="w", mec="k", zorder=9)
    ax.annotate("", xy=tuple((usb["p0"] + 30 * usb["d"])[:2]), xytext=tuple(usb["p0"][:2]), arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.6), zorder=10)
    ax.text(-28, 0, "tail\nblock\ntongue", fontsize=7, ha="center", zorder=9); ax.text(0, 0, "FC 36 x 36\n(on the plate's\nAIO holes)", fontsize=7.5, ha="center", va="center", zorder=9, color="w")
    ax.text(48, 0, "XT60", fontsize=7, ha="center", zorder=9); ax.text(31, 0, "leads", fontsize=6.5, ha="center", zorder=9)
    ax.set_xlim(-50, 60); ax.set_ylim(-40, 55); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up 2: a slice at y = the post (x, z): the post, the cradle, the flow, its cone, the receiver wall, the saddle rail
    ax = fig.add_subplot(gs[1, 1]); ax.set_aspect("equal"); py = flow["post_y"]
    ax.set_title(f"close-up: slice at y {py:+.1f} through the flow post (x, z) and the flow's view (23 deg)", fontsize=10)
    for n in ("belly mount (PETG)", "flow MTF-02P", "receiver v3 (latch)", "tie saddle (printed)", "frame plate (Bee35 carbon)", "cartridge v6 (latch)", "pack 4S 1500"):
        for yy in (py, flow["c"][1]):
            s = P[n]["mesh"].section(plane_origin=[0, yy, 0], plane_normal=[0, 1, 0])
            if s is None: continue
            for e in s.discrete:
                ax.fill(e[:, 0], e[:, 2], facecolor=COLS.get(n, "#a8b3bf"), edgecolor="k", lw=0.4, alpha=0.6 if n in OURS else 0.25, zorder=5 if n in OURS else 3)
    fx = flow["c"][0]; zl = flow["zb"]; t_ = math.tan(math.radians(FLOW_HALF))
    for wx in (fx - FLOW_WIN, fx + FLOW_WIN):
        ax.plot([wx, wx - (zl - PLANE + 8) * t_], [zl, PLANE - 8], color="#43a047", lw=0.9); ax.plot([wx, wx + (zl - PLANE + 8) * t_], [zl, PLANE - 8], color="#43a047", lw=0.9)
    ax.axhline(PLANE, color="#555", lw=0.6, ls=":"); ax.text(-48, PLANE + 1, "bottom ring plane", fontsize=7)
    ax.axhline(PLANE + BAND, color="#c62828", lw=0.7, ls="--"); ax.text(-48, PLANE + BAND + 1, "funnel band top", fontsize=7, color="#c62828")
    ax.set_xlim(-50, 60); ax.set_ylim(-62, 28); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up 3: the FC stack (section through two grommet screws, x, z), drawn from the stack numbers (E)
    ax = fig.add_subplot(gs[1, 2]); ax.set_aspect("equal"); ax.set_title("close-up: the FC stack at y +12.75 (x, z), numbers E (CHECK on the board)", fontsize=10)
    st = info["stack"]
    ax.add_patch(Rectangle((-40, PLATE_Z), 80, PLATE_TOP - PLATE_Z, color="#3a3a3a")); ax.text(30, 20.2, "carbon plate 3.5", color="w", fontsize=7)
    ax.add_patch(Rectangle((-18, st["pcb"][0]), 36, st["pcb"][1] - st["pcb"][0], color="#2e7d32")); ax.text(-3, st["pcb"][0] + 0.2, "PCB", color="w", fontsize=7)
    ax.add_patch(Rectangle((-18, FC_Z0), 36, FC_H, fill=False, ec="#2e7d32", ls="--", lw=0.8)); ax.text(-17.5, FC_Z0 + FC_H + 0.2, "FC envelope 36 x 8", fontsize=6.5, color="#2e7d32")
    for sx in (-1, 1):
        x_ = sx * FC_HOLES
        ax.add_patch(Rectangle((x_ - 1.0, st["nut"][0] - 0.5), 2.0, PLATE_TOP - st["nut"][0] + 0.5, color="#777"))          # M2 screw
        ax.add_patch(Rectangle((x_ - 1.75, PLATE_TOP), 3.5, 1.3, color="#555"))                                           # head
        ax.add_patch(Rectangle((x_ - 2.0, st["spacer"][0]), 4.0, st["spacer"][1] - st["spacer"][0], color="#bbb", ec="k", lw=0.3))   # spacer
        for z0_, z1_ in (st["flange_up"], st["flange_lo"]): ax.add_patch(Rectangle((x_ - 2.75, z0_), 5.5, z1_ - z0_, color="#d84315"))   # grommet flanges
        ax.add_patch(Rectangle((x_ - 2.0, st["nut"][0]), 4.0, st["nut"][1] - st["nut"][0], color="#444"))                   # M2 nut
    rec = P["receiver v3 (latch)"]["mesh"].section(plane_origin=[0, 12.75, 0], plane_normal=[0, 1, 0])
    if rec is not None:
        for e in rec.discrete: ax.fill(e[:, 0], e[:, 2], facecolor="#8899aa", edgecolor="k", lw=0.4, alpha=0.4)
    bms = P["belly mount (PETG)"]["mesh"].section(plane_origin=[0, 12.75, 0], plane_normal=[0, 1, 0])
    if bms is not None:
        for e in bms.discrete: ax.fill(e[:, 0], e[:, 2], facecolor="#2f7fd8", edgecolor="k", lw=0.4, alpha=0.8)
    ax.text(-38, 3, "receiver tray (floor 5.6), rib 8.6 at x 3..6.5", fontsize=7)
    ax.text(0, 25.5, f"FC: plate {info['fc_gap_plate']:.1f} above, receiver rib {info['fc_gap_rec']:.1f} below (rule 2.0)", fontsize=7.5, ha="center", color="#c62828")
    ax.text(0, -1.5, "M2 x 12 from the top, Ø3 / 3.2 spacer, M2 grommet (E), M2 nut", fontsize=7, ha="center")
    ax.set_xlim(-42, 42); ax.set_ylim(-4, 28); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    fig.text(0.01, 0.004, "blue: belly mount v1 (one PETG print: frame between the plate and the receiver, 4 receiver inserts, 4 plate inserts (red squares), "
             "ears on the tail block, the flow post + cradle); green: FC and flow; violet: the electronics tail block and carrier (PR 130); orange: tie saddle; "
             "grey: receiver set; dark blue: swap set; black: cage rods; yellow discs: swept props; green fan: the flow's 23 deg view; red arrow: FC USB-C path",
             fontsize=8)
    fig.tight_layout(rect=(0, 0.015, 1, 1)); fig.savefig(path, dpi=105); plt.close(fig)


# ---------------------------------------------------------------- 9. the pass
def pick_flow(scored, side):
    """the best-scoring pose beside the receiver (|y| >= 30, its post in the receiver's straight wall run x -24..34, away from the bolt
    columns), mirrored to the side opposite the USB-C window (the window breaks that rail; the flow's post hangs from the whole one)"""
    fs = -side
    for r in scored:
        _, x, y, zb, L, anc, start, corner, in_col, ori = r
        hx, hy = (FLOW_L / 2, FLOW_W / 2) if ori[0] == "x" else (FLOW_W / 2, FLOW_L / 2)
        if abs(y) < 30.0 or not (-24.0 <= x <= 34.0): continue
        bx0 = x - hx + min(0.0, FLOW_PLUG if ori[1] == "+" else -FLOW_PLUG) * (ori[0] == "x")
        bx1 = x + hx + max(0.0, FLOW_PLUG if ori[1] == "+" else -FLOW_PLUG) * (ori[0] == "x")
        if under_swap(bx0 - 4, bx1 + 4, y - hy - 4, y + hy + 4): continue
        py = fs * (25.3 + GAP + STRUT_R + 0.2)                          # the post: 2.2 off the receiver's wall (|y| 25.3, C), 2.8 off the saddle rail
        if abs(y) - hy - CRADLE > abs(py) + STRUT_R - 1.0: continue     # the post must land on the cradle
        return dict(c=(float(x), float(fs * y)), zb=float(zb), ori=ori, post_x=float(x), post_y=float(py), score=r[0], strut_L=float(L),
                    in_swap_column=bool(in_col), side=fs)
    return None


def run():
    t0 = time.time(); os.makedirs(RUN, exist_ok=True)
    say("Scout v3 belly mount, concept v1: system pass on main's assembly (layout v12, catch 5.0 forward, camera mount v1, tail bay PR 130)")
    M = load_model()
    O = occupancy(M, skip=("flow MTF-02P",), env=True)
    Oc = occupancy(M, skip=("flow MTF-02P", "cage shell nodes", "camera", "camera mount (printed)", "camera mount stop collar +y",
                            "camera mount stop collar -y"), env=True, rods=False)
    # --- 1. the FC: on the plate's own AIO holes (as built), the stack in the 8.9 gap (E)
    pcb = (FC_Z0 + 3.2, FC_Z0 + 3.2 + 1.6)
    stack = dict(pcb=pcb, flange_up=(pcb[1], pcb[1] + 1.0), flange_lo=(pcb[0] - 1.0, pcb[0]), spacer=(pcb[1] + 1.0, PLATE_Z),
                 nut=(pcb[0] - 1.0 - 1.6, pcb[0] - 1.0))
    fcm = P_fc = M["P"]["FC"]["mesh"]
    gap_plate = PLATE_Z - (FC_Z0 + FC_H); gap_rec = min_gap(fcm, M["P"]["receiver v3 (latch)"]["mesh"])
    say(f"FC: on the plate's 25.5 AIO holes at x 0 (as built, P; CHECK the hole positions on the plate), z {FC_Z0}..{FC_Z0 + FC_H} (H: lowest point 9 "
        f"below the plate). Gaps: plate {gap_plate:.1f}, receiver {gap_rec:.2f} (its rib at x 3..6.5, top 8.6) against the 2.0 rule: both short (C on E)")
    say(f"FC stack (E): M2 x 12 from the plate top, spacer {stack['spacer'][0]:.1f}..{stack['spacer'][1]:.1f}, grommet flanges {stack['flange_up']} / "
        f"{stack['flange_lo']}, PCB {pcb[0]:.1f}..{pcb[1]:.1f}, M2 nut {stack['nut'][0]:.1f}..{stack['nut'][1]:.1f} in the receiver's tray (floor 5.6)")
    # --- 2. the flow and the USB-C
    say(f"flow clear-air test: {WAKE} (E)")
    cache = os.environ.get("BELLY_CACHE")                              # development only: reuse a saved search (the run itself never sets it)
    if cache and os.path.exists(cache):
        d_ = json.load(open(cache)); scored = [tuple(r_) for r_ in d_["scored"]]; FEAS[:] = [None] * d_["n_feas"]
    else:
        scored = flow_search(M, O, Oc)
        if cache: json.dump(dict(scored=[[x if not isinstance(x, np.generic) else x.item() for x in r_] for r_ in scored], n_feas=len(FEAS)), open(cache, "w"), default=lambda x: x.item() if hasattr(x, "item") else x)
    usbs = usb_search(M, O)
    _, side, xu, az, el, d = usbs[0]
    usb = dict(side=side, x=xu, az=az, el=el, d=d, p0=np.array([xu, side * FC_W / 2, USB_Z]))
    flow = pick_flow(scored, side)
    say(f"flow picked: centre ({flow['c'][0]:.0f}, {flow['c'][1]:+.0f}), z {flow['zb']:.0f}..{flow['zb'] + FLOW_T:.0f}, long side along {flow['ori'][0]}, "
        f"plug end {flow['ori'][1]}{flow['ori'][0]}, strut {flow['strut_L']:.1f} (search), post at ({flow['post_x']:.1f}, {flow['post_y']:+.1f}), "
        f"{'inside the swap column below the funnel band' if flow['in_swap_column'] else 'above the funnel band'}")
    top = scored[:5]
    say("flow search, best 5 (score, x, y, z_b, strut, anchor, start, bend, in column, orientation): " + "; ".join(str((r[0],) + r[1:5] + r[8:]) for r in top))
    # --- 3. the belly mount
    bm = belly_mount(side, xu, az, flow)
    from solid_check import solid_check
    ok, sc_info = solid_check(bm)
    bm_g, bm_solid, _ = printed_g(bm)
    bm_c = np.asarray(bm.center_mass, float)
    hx, hy = (FLOW_L / 2, FLOW_W / 2) if flow["ori"][0] == "x" else (FLOW_W / 2, FLOW_L / 2)
    cr_box = box([flow["c"][0] - hx - CRADLE, flow["c"][1] - hy - CRADLE, flow["zb"] - CRADLE], [flow["c"][0] + hx + CRADLE, flow["c"][1] + hy + CRADLE, flow["zb"] + FLOW_T + CRADLE])
    cradle = trimesh.boolean.intersection([bm, cr_box], engine="manifold"); cradle_g = printed_g(cradle)[0]
    bm.export(os.path.join(RUN, "belly_mount__petg_.stl"))
    say(f"belly mount: {bm_g:.1f} g as printed (2.4 walls + 20 %, E; solid {bm_solid:.1f} g), CG ({bm_c[0]:.1f}, {bm_c[1]:+.1f}, {bm_c[2]:.1f}), "
        f"bounds {np.round(bm.bounds[0], 1).tolist()} .. {np.round(bm.bounds[1], 1).tolist()}, solid_check {'PASS' if ok else 'FAIL'} {sc_info}")
    P = dict(M["P"])
    P["flow MTF-02P"] = dict(mesh=flow_mesh(flow), g=1.5, c=np.array([flow["c"][0], flow["c"][1], flow["zb"] + FLOW_T / 2]), role="bought", tag="P", spec={})
    P["belly mount (PETG)"] = dict(mesh=bm, g=bm_g, c=bm_c, role="structure", tag="E", spec={})
    # --- 4. fit
    rods = rods_all(M)
    ft = fit_table(P, rods)
    say("fit table (every pair within 10 of the belly mount, the flow and the FC; >= 2.0 or a declared face contact <= 0.05 mm3):")
    for r in ft: say(f"   {r['a']} / {r['b']}: overlap {r['overlap_mm3']:.3f} mm3, gap {r['gap_mm']:.2f}, {r['kind']}: {'PASS' if r['pass_'] else 'FAIL'}")
    # the flow's view on the final assembly: every part's points inside either window's cone
    seen = []
    for n, p in P.items():
        if n in ("flow MTF-02P", "wires"): continue
        pts = np.vstack([p["mesh"].vertices, p["mesh"].sample(3000, seed=4)])
        for wx, wy in ((FLOW_WIN, 0.0), (-FLOW_WIN, 0.0)) if flow["ori"][0] == "x" else ((0.0, FLOW_WIN), (0.0, -FLOW_WIN)):
            ap = np.array([flow["c"][0] + wx, flow["c"][1] + wy, flow["zb"]]); dd = pts - ap; zz_ = -dd[:, 2]
            ins = (zz_ > 0) & (np.hypot(dd[:, 0], dd[:, 1]) < zz_ * math.tan(math.radians(FLOW_HALF)) + 1.5)
            if ins.any(): seen.append(f"{n} ({ins.sum()} pts)")
    for a_, b_, r_ in rods:
        q = a_ + np.linspace(0, 1, 200)[:, None] * (b_ - a_)
        for wx, wy in ((FLOW_WIN, 0.0), (-FLOW_WIN, 0.0)) if flow["ori"][0] == "x" else ((0.0, FLOW_WIN), (0.0, -FLOW_WIN)):
            ap = np.array([flow["c"][0] + wx, flow["c"][1] + wy, flow["zb"]]); dd = q - ap; zz_ = -dd[:, 2]
            if ((zz_ > 0) & (np.hypot(dd[:, 0], dd[:, 1]) < zz_ * math.tan(math.radians(FLOW_HALF)) + 1.5 + r_)).any(): seen.append("a cage rod"); break
    say("flow view (both windows, 23 deg + the window radius, to the ground): " + ("clear of every part and rod (C)" if not seen else "IN VIEW: " + ", ".join(sorted(set(seen)))))
    # the flow body's plan under the swept disks (the column measure of rule 8) and against the contracted slipstream
    fb = P["flow MTF-02P"]["mesh"].bounds; xs_ = np.linspace(fb[0][0], fb[1][0], 23); ys_ = np.linspace(fb[0][1], fb[1][1], 17)
    XX, YY = np.meshgrid(xs_, ys_); inside = np.zeros(XX.shape, bool)
    for mx, my in DISKS: inside |= np.hypot(XX - mx, YY - my) < DISK_R
    dmin = min(math.hypot(x_ - mx, y_ - my) for x_ in (fb[0][0], fb[1][0]) for y_ in (fb[0][1], fb[1][1]) for mx, my in DISKS)
    rw = wake_r(PROP_Z - (flow["zb"] + FLOW_T))
    say(f"flow wash: {100 * inside.mean():.0f} % of its plan under a swept disk (column measure: FAIL, rule 8 wants >= 3 outside); nearest corner "
        f"{dmin:.1f} from a motor axis, the slipstream there {rw:.1f} (E, momentum theory, {PROP_Z - flow['zb'] - FLOW_T:.0f} below the props): "
        f"{dmin - rw:+.1f} outside it")
    # --- 5. system_check
    man = manifest(M, P, usb, flow, bm, bm_g, bm_c)
    r, txt = run_system_check(man)
    open(os.path.join(RUN, "system_check.txt"), "w").write(txt + "\n")
    say(r["summary"])
    acc_rows = [(p["name"], p["pass_"], round(p.get("overlap_mm3", 0.0), 2), [b["part"] for b in p.get("blocked_by", [])]) for p in r["access"]["paths"]]
    for a_ in acc_rows: say(f"   access {a_[0]}: {'clear' if a_[1] else 'BLOCKED ' + str(a_[3]) + ' ' + str(a_[2]) + ' mm3'}")
    mc = r["mass_cg"]; bal = r["balance"]
    cin = np.array(bal["cases"]["all"]["cg_offset"]) if bal.get("applies") else None
    # --- 6. loads, modes
    L = loads(P, bm, bm_g, bm_c, flow, cradle_g)
    by = {}
    for row in L["rows"]:
        k = row["kind"]; b = by.setdefault(k, dict(limit=row["limit"]))
        for f_ in ("rec_idx", "plate_idx", "frame_idx", "post_idx", "tongue_idx", "carbon_idx"):
            if row[f_] > b.get(f_, (-1, ""))[0]: b[f_] = (row[f_], row["case"])
    say(f"loads (L, E): hung set {L['hung_g']} g at {L['hung_cg']}, flow + cradle {L['flow_g']} g, tail bay {L['bay_g']} g on the tongue (+x crash)")
    for k, b in by.items():
        say(f"   {k} (limit {b['limit']}): receiver inserts {b['rec_idx'][0]:.3f} ({b['rec_idx'][1]}), plate inserts {b['plate_idx'][0]:.3f} ({b['plate_idx'][1]}), "
            f"frame {b['frame_idx'][0]:.3f} ({b['frame_idx'][1]}), flow post {b['post_idx'][0]:.3f}, tongue face {b['tongue_idx'][0]:.4f}, carbon bearing {b['carbon_idx'][0]:.4f}")
    say("   sections (hollow 2.4 walls): " + "; ".join(f"{k}: A {v['A']}, I {v['I']}, Z {v['Z']}" for k, v in L["sections"].items() if v)
        + f"; post root (solid) A {L['post_root']['A']}, Z {L['post_root']['Z']}")
    modes = {E_: fc_modes(E_) for E_ in (1.0, 1.5, 2.5)}
    rigid = fc_rigid_mode(); pm = post_mode(L["post_L"], L["flow_g"])
    say(f"FC modes on 4 M2 grommets (E, silicone E 1.0 / 1.5 / 2.5 MPa): first " + " / ".join(f"{m['first_Hz']}" for m in modes.values())
        + f" Hz (lateral), vertical {modes[1.5]['vertical_Hz']}, rocking {modes[1.5]['rocking_Hz']} at 1.5 MPa; hard-mounted on the screws {rigid} Hz: the grommet sets it")
    say(f"flow post: {L['post_L']} long 6 x 6 PETG with {L['flow_g']} g at the tip: {pm} Hz (E)")
    # --- 7. PX4 offsets (FRD from the CG, m)
    cg = np.array(mc["cg"], float)
    imu = np.array([FC_C[0], FC_C[1], (pcb[0] + pcb[1]) / 2]); lens = np.array([flow["c"][0] + (FLOW_WIN if flow["ori"][0] == "x" else 0), flow["c"][1], flow["zb"]])
    tof = np.array([flow["c"][0] - (FLOW_WIN if flow["ori"][0] == "x" else 0), flow["c"][1], flow["zb"]])
    frd = lambda p: np.round([(p[0] - cg[0]) / 1000, -(p[1] - cg[1]) / 1000, -(p[2] - cg[2]) / 1000], 4).tolist()
    px4 = dict(cg=np.round(cg, 2).tolist(), EKF2_IMU_POS=frd(imu), EKF2_OF_POS=frd(lens), EKF2_RNG_POS=frd(tof),
               note="IMU at the board centre (E: the chip's place on the board is not published); lens at the nose window, ToF at the tail window (E)")
    say(f"PX4 (FRD from the CG {px4['cg']}, m): EKF2_IMU_POS {px4['EKF2_IMU_POS']}, EKF2_OF_POS {px4['EKF2_OF_POS']}, EKF2_RNG_POS {px4['EKF2_RNG_POS']}")
    # --- 8. balance against the placeholder (25 g E at its centroid)
    ph = trimesh.load(os.path.join(V12, "parts", "mount_p_nose_legs__scout.stl")); ph_c = centroid(ph); ph_g = 25.0
    dm = bm_g + 1.5 - ph_g - 1.5
    say(f"vs the layout's placeholder ({ph_g} g E at {np.round(ph_c, 1).tolist()}): {dm:+.1f} g; ours at {np.round(bm_c, 1).tolist()}")
    info = dict(stack=stack, fc_gap_plate=gap_plate, fc_gap_rec=gap_rec)
    preview(P, M, bm, flow, usb, os.path.join(RUN, "preview.png"), info)
    res = dict(version="v1 (concept)", layout="scout_layout v12 + main's assembly (camera mount v1, PR 146) + the tail bay as drawn (PR 130)",
               wake_test=WAKE,
               belly_mount=dict(mass_g=round(bm_g, 2), mass_solid_g=round(bm_solid, 2), cg=np.round(bm_c, 2).tolist(), bounds=np.round(bm.bounds, 2).tolist(),
                                solid_check=dict(pass_=bool(ok), **{k: v for k, v in sc_info.items() if k != "ok"}), cradle_g=round(cradle_g, 2),
                                receiver_inserts=REC_BOLTS, plate_bolts=PLATE_BOLTS, tail_face_x=TAIL_X, ear_y=EAR_Y),
               fc=dict(centre=list(FC_C), z=[FC_Z0, FC_Z0 + FC_H], gap_plate=round(gap_plate, 2), gap_receiver=round(gap_rec, 2), stack=stack,
                       modes=modes, hard_mounted_Hz=rigid),
               flow=dict(flow, c=list(flow["c"]), post_mode_Hz=pm, view_clear=not seen, seen=sorted(set(seen)), plan_under_disk_pct=round(100 * inside.mean(), 1),
                         corner_from_axis=round(dmin, 1), slipstream_r=round(rw, 1)),
               flow_search=dict(top=[[float(v) if isinstance(v, (int, float, np.floating)) else (str(v) if not isinstance(v, tuple) else [float(x) for x in v]) for v in r_] for r_ in scored[:20]],
                                n_scored=len(scored), n_feasible=len(FEAS)),
               usb=dict(side=int(side), x=xu, azimuth=az, elevation=el, dir=np.round(d, 4).tolist(), p0=usb["p0"].tolist(), n_clear=len(usbs)),
               fit=ft, loads=dict(by_kind={k: {kk: (list(vv) if isinstance(vv, tuple) else vv) for kk, vv in b.items()} for k, b in by.items()},
                                  **{k: v for k, v in L.items() if k != "rows"}, cases=L["rows"]),
               system_check=dict(summary=r["summary"], pass_=bool(r["pass"]), cg_offset_in=bal["cases"]["all"]["horizontal_mm"] if bal.get("applies") else None,
                                 cg_offset_out=bal["cases"].get("no_swap", {}).get("horizontal_mm") if bal.get("applies") else None,
                                 mass_g=mc["mass_g"], cg=mc["cg"], access=acc_rows),
               px4=px4, vs_placeholder=dict(dm_g=round(dm, 2), placeholder_cg=np.round(ph_c, 2).tolist()))
    json.dump(res, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else (bool(x) if isinstance(x, np.bool_) else str(x)))
    json.dump(man, open(os.path.join(RUN, "manifest.json"), "w"), indent=1, default=lambda x: "<trimesh: see the STL>" if isinstance(x, trimesh.Trimesh) else (x.tolist() if hasattr(x, "tolist") else str(x)))
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())
    return res


if __name__ == "__main__":
    run()
