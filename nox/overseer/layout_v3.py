"""Overseer layout v3 (rough design, 26 Oct): the geodesic sphere cage (Spark 26 Oct, with a photo of the Flyability Elios
3: "for all cages in the future i want it to be in this format ... do this to the overseer scrapping the previous cage
design"; then "ignore how the funnel is right now, we will create a new funnel for this design, try to make the cage
closer to this picture like a sphere and just make sure the opening at the bottom is large enough with room to spare
for the battery and the swapping").

Cage v1: a class-I icosahedral geodesic sphere of 3.0 carbon rods and printed nodes round the whole drone (props,
motors, arms, plates, L2, cameras, pack and latch set inside), one round opening at the bottom pole: its clear circle
holds the receiver footprint + SPARE a side, so the pack and the swap head pass with room to spare. The pack bay (latch
set + down camera + flow) sits just above the opening, on a triangulated hanger from the arm roots tied to the opening
ring, so the swap reach stays short. Wheel ring, corner nodes, guard rings and the v2 belly sled are gone; the funnel,
hook latch and keep-alive probes are redesigned round this sphere by the box catch session (Spark 26 Oct).

Placement by optimisation: for each L2 height zB the motor square is moved until the CG sits on the thrust centre; the
L2 base is the smallest x that keeps every non-cage point behind its field; the sphere is the smallest that holds every
point with its margin (props 12 + rod, the rest 3) inside the geodesic's inner face planes; the geodesic is turned
about z for the best prop clearance. The zB with the smallest sphere wins.

python3 nox/overseer/layout_v3.py [out_dir]     (~10 min)
Drone frame: x forward, y left, z up; z 0 = bottom plate top; plate centre x 0. Numbers: `nox/dimensions.md` O / M / G / L.
"""
import math, os, sys, io, contextlib, json
import numpy as np, trimesh
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
import assembly_seed as S

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_layout", "v3")
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


L = S._layout_v2()                       # v2 definitions: plates, hub, arms, L2, camera, helpers (v2 itself is not edited)
box, cyl, rod, pts, union, overlap = L["box"], L["cyl"], L["rod"], L["pts"], L["union"], L["overlap"]
A, HALF, PROP_R, PROP_Z, BLADE, ROOT_H = L["A"], L["HALF"], L["PROP_R"], L["PROP_Z"], L["BLADE"], L["ROOT_H"]
TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX = (L[k] for k in ("TOP", "BOTTOM", "FC", "DECK", "PI", "PDB", "UBEC", "BUCK", "RX"))

# ---------------------------------------------------------------- numbers
NU = int(os.environ.get("V3_NU", 2))     # geodesic frequency (Spark 26 Oct: "go with the larger cells that save weight"; NU 3 alt)
ROD_R, ROD_GPM = 1.5, 11.0e-3            # links (spokes, mast, hanger, ties): 3.0 solid carbon (G Carbon rods), 11.0 g/m (C)
SHELL_R = float(os.environ.get("V3_TUBE", 6.0)) / 2   # shell: 6 mm carbon tube (Spark 26 Oct "we will use 6mm diameter tubes")
SHELL_GPM = 1.55e-3 * math.pi / 4 * ((2 * SHELL_R) ** 2 - (2 * SHELL_R - 2.0) ** 2)   # 6/4 tube (ID 4 DEFAULT): 24.3 g/m (E)
NODE_R = SHELL_R + 0.15 + 2.0 + 5.0      # printed hub node round its 6.3 sockets: bore + 2.0 wall + the socket fan (E): Ø20 at 6 mm
NODE_G, RING_G = 4.5, 6.0                # PETG node, 5 to 6 sockets 15 deep (E); opening ring node with its TPU foot (E)
PROP_GAP, CAGE_GAP = 12.0, 3.0           # guard rods 12 off the swept disks (G Guard outline); cage first 3 (L)
SPARE = 30.0                             # room to spare a side round the receiver at the opening (DEFAULT, Spark 26 Oct)
REC = (179.6, 76.2)                      # Overseer receiver v2 footprint (M, battery latch PR 69)
CART = (174.0, 55.0)                     # cartridge v6 (M)
CART_H, REC_H = 49.0, 24.0               # cartridge height (pack 45 + 2 + 2, E), receiver deck above it (E)
PACK = (148.0, 48.0, 45.0)               # 4S 6000 (P)
LATCH_G = 338.5                          # Overseer latch set (M, battery latch v4)
ZONE = (REC[0] / 2 + SPARE, REC[1] / 2 + SPARE)       # swap zone half sizes: receiver + SPARE a side
R_CLEAR = math.hypot(*ZONE)                            # 137.8: clear radius the opening must give (C)
CB_UP = 3.0                              # cartridge bottom above the opening ring axis plane (DEFAULT; feet 12 below it)
FOOT_DOWN = 12.0                         # TPU foot bottom under the ring axis plane (E)
LENS_UP = 12.0                           # down camera and flow lenses above the cartridge bottom (DEFAULT: the probe check, 26 Oct)
BREAK_EVEN = 33.0                        # carbon vs printed strut (generative.md rule 0)
K = float(os.environ.get("V3_K", 1.0))   # vertical / horizontal semi-axis (1.0 sphere; the rule allows height / width >= 0.85)
BAY = os.environ.get("V3_BAY", "low")    # low: pack bay at the opening (DEFAULT, short swap reach); high: receiver under the plates


# ---------------------------------------------------------------- geodesic (class I, icosahedral, frequency NU)
def geodesic(nu):
    p = (1 + 5 ** 0.5) / 2
    V = np.array([(-1, p, 0), (1, p, 0), (-1, -p, 0), (1, -p, 0), (0, -1, p), (0, 1, p), (0, -1, -p), (0, 1, -p),
                  (p, 0, -1), (p, 0, 1), (-p, 0, -1), (-p, 0, 1)], float)
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    V /= np.linalg.norm(V, axis=1)[:, None]
    z = V[5] / np.linalg.norm(V[5])                               # turn vertex 5 to the top pole
    ax = np.cross(z, [0, 0, 1.0]); s = np.linalg.norm(ax); c = z[2]
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]) / s
    Rm = np.eye(3) + s * K + (1 - c) * K @ K
    V = V @ Rm.T
    pts_, keys = [], {}
    for a, b, c_ in F:
        for i in range(nu + 1):
            for j in range(nu + 1 - i):
                k = nu - i - j
                q = (i * V[a] + j * V[b] + k * V[c_]) / nu; q /= np.linalg.norm(q)
                key = tuple(np.round(q, 6))
                if key not in keys: keys[key] = len(pts_); pts_.append(q)
    U = np.array(pts_)
    hull = trimesh.convex.convex_hull(U)
    rho = float(np.min(np.abs(np.einsum("ij,ij->i", hull.face_normals, U[hull.faces[:, 0]]))))   # inner face plane / R
    # the hull's vertex order is not U's: keep U, faces from the hull mapped back by position
    return U, rho


U_GEO, RHO = geodesic(NU)


def ring_r(R, delta):
    """opening ring: node count and radius so the node surfaces and the chords between them stay outside the clear circle
    (swap zone diagonal + the bay's offset from the sphere axis)"""
    r_clear = R_CLEAR + abs(delta); mean_edge = R * 1.1 / NU      # ~ class-I edge (E), for the ring spacing
    n = max(6, int(round(2 * math.pi * (r_clear + NODE_R) / (0.8 * mean_edge))))
    return n, max(r_clear + NODE_R, (r_clear + SHELL_R) / math.cos(math.pi / n)), mean_edge


def shell(c, R, rot_deg, delta):
    """sphere nodes (geodesic turned rot_deg about z), the opening ring, triangulated; returns nodes, rods, tris, z_o, ring ids"""
    t = math.radians(rot_deg); Rz = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
    P = c + R * (U_GEO @ Rz.T) * [1, 1, K]
    n_ring, R_RING, mean_edge = ring_r(R, delta)
    dz_o = K * math.sqrt(R * R - R_RING ** 2); z_o = c[2] - dz_o
    keep = P[:, 2] > z_o + 0.45 * mean_edge
    ang = np.linspace(0, 2 * math.pi, n_ring, endpoint=False) + math.pi / n_ring
    ring = np.stack([c[0] + R_RING * np.cos(ang), c[1] + R_RING * np.sin(ang), np.full(n_ring, z_o)], 1)
    N = np.vstack([P[keep], ring]); is_ring = np.r_[np.zeros(keep.sum(), bool), np.ones(n_ring, bool)]
    h = trimesh.convex.convex_hull(N)
    # convex_hull may drop / reorder vertices: map its vertices back to N
    idx = np.array([int(np.argmin(np.linalg.norm(N - v, axis=1))) for v in h.vertices])
    tris = idx[h.faces]
    tris = tris[~is_ring[tris].all(1)]                             # the opening: no rods across it
    E = set()
    for a, b, c_ in tris:
        for u, v in ((a, b), (b, c_), (c_, a)):
            E.add((min(u, v), max(u, v)))
    used = sorted(set(tris.ravel()))
    remap = {o: n for n, o in enumerate(used)}
    nodes = N[used]; rods = [(remap[a], remap[b]) for a, b in sorted(E)]
    tris = np.array([[remap[a] for a in t_] for t_ in tris])
    ring_ids = [remap[i] for i in np.where(is_ring)[0] if i in remap]
    return nodes, rods, tris, z_o, ring_ids


def seg_band_dist(p0, p1, motors_xy):
    """smallest distance from a segment to the prop swept bands (disk r PROP_R, PROP_Z ± BLADE)"""
    t = np.linspace(0, 1, 120)[:, None]; q = p0 + t * (p1 - p0); best = np.inf
    for mx, my in motors_xy:
        dr = np.maximum(np.hypot(q[:, 0] - mx, q[:, 1] - my) - PROP_R, 0); dz = np.maximum(np.abs(q[:, 2] - PROP_Z) - BLADE, 0)
        best = min(best, float(np.hypot(dr, dz).min()))
    return best


# ---------------------------------------------------------------- the drone round the sphere
def prop_ring_pts(motors_xy):
    a = np.linspace(0, 2 * math.pi, 96, endpoint=False)
    out = []
    for mx, my in motors_xy:
        for z in (PROP_Z - BLADE, PROP_Z + BLADE):
            out.append(np.stack([mx + PROP_R * np.cos(a), my + PROP_R * np.sin(a), np.full_like(a, z)], 1))
    return np.vstack(out)


def bay(x0, z_o):
    """pack bay on the opening axis: cartridge, pack, receiver deck, down camera + flow pods (outside the swap zone)"""
    zc = z_o + CB_UP if BAY == "low" else -2.0 - CART_H - REC_H     # high: receiver deck under the bottom plate
    b = {}
    b["cart"] = box(x0 - CART[0] / 2, x0 + CART[0] / 2, -CART[1] / 2, CART[1] / 2, zc, zc + CART_H)
    b["pack"] = box(x0 - PACK[0] / 2, x0 + PACK[0] / 2, -PACK[1] / 2, PACK[1] / 2, zc + 2, zc + 2 + PACK[2])
    b["rec"] = box(x0 - REC[0] / 2, x0 + REC[0] / 2, -REC[1] / 2, REC[1] / 2, zc + CART_H, zc + CART_H + REC_H)
    y_in = ZONE[1] + 2.0                                            # pods start 2 outside the swap zone
    b["cam_pod"] = box(x0 - 23, x0 + 23, y_in, y_in + 44, zc + 4, zc + CART_H + REC_H)
    b["flow_pod"] = box(x0 - 15, x0 + 15, -(y_in + 30), -y_in, zc + 4, zc + CART_H + REC_H)
    yc = y_in + 22
    zl = zc + LENS_UP
    b["cam_pod"] = box(x0 - 23, x0 + 23, y_in, y_in + 44, zl + 10, zc + CART_H + REC_H)
    b["flow_pod"] = box(x0 - 15, x0 + 15, -(y_in + 30), -y_in, zl + 6.5, zc + CART_H + REC_H)
    b["down_cam"] = trimesh.util.concatenate([box(x0 - 19, x0 + 19, yc - 19, yc + 19, zl + 8.4, zl + 10), cyl(6, zl, zl + 8.4, x0, yc)])
    b["down_lens"] = np.array([x0, yc, zl])
    b["flow"] = box(x0 - 10.8, x0 + 10.8, -(y_in + 23), -(y_in + 7), zl, zl + 6.5)
    b["flow_lens"] = np.array([x0, -(y_in + 15), zl])
    b["z_top"] = zc + CART_H + REC_H
    return b


L2_MODE = os.environ.get("V3_L2", "top")  # top: axis up on a mast over the plates, as the reference photo (DEFAULT 26 Oct);
                                          # nose: axis 15 deg below +x at the nose (Spark 24 Oct), the alternative
L2_NEG_TOP = 6.0                          # top: the 96 deg negative-angle mode kept clear (as layout v0)
CAM_UP_TOP = 30.0                         # top: front camera 30 deg up (Spark 23 Oct choice for the up-facing L2)


def l2_up(xc, zb):
    body = box(xc - 37.5, xc + 37.5, -37.5, 37.5, zb, zb + 39)
    dome = trimesh.creation.icosphere(3, 35.5); dome.apply_scale([1, 1, 26 / 35.5]); dome.apply_translation([xc, 0, zb + 39])
    dome = dome.slice_plane([0, 0, zb + 39], [0, 0, 1], cap=True)
    return trimesh.util.concatenate([body, dome])


def cam_mesh(C, up_deg):
    fwd = np.array([math.cos(math.radians(up_deg)), 0, math.sin(math.radians(up_deg))]); cup = np.array([-fwd[2], 0, fwd[0]])
    Rc = np.eye(4); Rc[:3, 0] = fwd; Rc[:3, 1] = [0, 1, 0]; Rc[:3, 2] = cup; Rc[:3, 3] = C
    board = box(-1, 1, -19, 19, -19, 19); lens = trimesh.creation.cylinder(6, 8, sections=32)
    lens.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); lens.apply_translation([5, 0, 0])
    cam = trimesh.util.concatenate([board, lens]); cam.apply_transform(Rc)
    plate = box(-4.5, -1.5, -20, 20, -20, 20); plate.apply_transform(Rc)
    return cam, plate, C + fwd * 9, fwd, cup


def top_l2(g, xc):
    """L2 axis up over the motor-square centre: lowest base with every non-cage point under the -6 deg cone + 1; seat and
    front camera on it; a triangulated 8-rod mast from the 4 arm-top screw pads (r 63 on the diagonals, top plate top)"""
    P = np.vstack([pts(m, 800) for grp in (g["arms"], g["motors"], [TOP, PI]) for m in grp] + [prop_ring_pts(g["motors_xy"])])
    lo, hi = 50.0, 300.0
    for _ in range(30):
        zb = (lo + hi) / 2; d = P - [xc, 0, zb]
        if ((-(d[:, 2]) - math.sin(math.radians(L2_NEG_TOP)) * np.linalg.norm(d, axis=1)) >= 1.0).all(): hi = zb
        else: lo = zb
    zb0 = hi
    # front camera on a plate hung from the seat's front edge, 30 deg up (Spark 23 Oct), nearest spot ahead (review 26 Oct: no
    # props in its view, CLAUDE.md 8): the L2 and its camera rise together from the lowest base until the L2 and the prop disks
    # (± blade band) are out of the camera view + 2 deg, with the camera itself under the L2's -6 deg cone
    disks = np.vstack([pts(cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, mx_, my_, 96), 3000) for mx_, my_ in g["motors_xy"]])
    found = False; up = CAM_UP_TOP
    for zb in np.arange(zb0, zb0 + 120.0, 2.0):
        l2 = l2_up(xc, zb); B_ = np.array([xc, 0, zb])
        for dx in np.arange(40.0, 90.0, 2.0):
            for dzc in (-22.0, -30.0):
                cam, plate, lens, fwd, cup = cam_mesh(np.array([xc + dx, 0, zb + dzc]), up)
                cp = pts(cam, 400) - B_
                if (-cp[:, 2] - math.sin(math.radians(L2_NEG_TOP)) * np.linalg.norm(cp, axis=1) < 1.0).any(): continue
                bad = False
                for tgt in (pts(l2, 3000), disks):
                    d = tgt - lens; zz = d @ fwd
                    el = np.degrees(np.arctan2(d @ cup, np.maximum(zz, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
                    if ((zz > 0) & (np.abs(el) < L["CAM_VFOV"] / 2 + 2.0) & (np.abs(az) < L["CAM_HFOV"] / 2 + 2.0)).any(): bad = True; break
                if not bad: found = True; break
            if found: break
        if found: break
    if not found: raise RuntimeError("no L2 height clears the props from the front camera view")
    seat = cyl(37.5, zb - 5, zb, xc, 0, 48)
    zcam = zb + dzc
    arm_bracket = box(xc + 30, xc + dx - 3, -12, 12, zb - 5, zb - 2)   # printed strap from the seat to the camera plate (E)
    if zcam < zb - 25: arm_bracket = trimesh.util.concatenate([arm_bracket, box(xc + dx - 9, xc + dx - 3, -12, 12, zcam + 19, zb - 2)])
    mount = trimesh.util.concatenate([seat, plate, arm_bracket])
    # mast feet on the top plate between the arms (0 / 90 / 180 / 270 deg, r 52 on the ±60 plate, pad Ø12; drilled M3, DEFAULT): on the diagonals the legs
    # pass the prop tips under 12 (review 26 Oct)
    ang4 = {(1, 1): 0.0, (-1, 1): 90.0, (-1, -1): 180.0, (1, -1): 270.0}
    feet = {k: np.array([52.0 * math.cos(math.radians(a_)), 52.0 * math.sin(math.radians(a_)), ROOT_H + 1.6]) for k, a_ in ang4.items()}
    tops = {k: np.array([xc + 30.0 * math.cos(math.radians(a_)), 30.0 * math.sin(math.radians(a_)), zb - 5]) for k, a_ in ang4.items()}
    mast = {}
    order = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    for i, k in enumerate(order):
        mast[f"mast leg {k[0]:+d}{k[1]:+d}"] = (feet[k], tops[k])
        mast[f"mast diag {k[0]:+d}{k[1]:+d}"] = (feet[k], tops[order[(i + 1) % 4]])
    return dict(zb0=zb0, l2=l2, seat=seat, cam=cam, cam_plate=plate, cam_lens=lens, cam_fwd=fwd, cam_up_v=cup, mount=mount, mast=mast, zB=zb, cam_dx=dx, cam_up=up)


def layout(dm, zB, cam_h=None, delta=0.0):
    """everything for motor square dm ahead of the plate centre and L2 base height zB"""
    g = L["build"](dm, 30.0, 330.0, zB)                           # v2 arms, motors, props (dc / xB placeholders)
    if L2_MODE == "top":
        return layout_top(g, dm, delta)
    mxy = g["motors_xy"]
    fixed = [g["arms"], g["motors"], [TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX]]
    P0 = np.vstack([pts(m, 800) for grp in fixed for m in grp] + [prop_ring_pts(mxy)])
    xB = (float((P0 @ A).max()) + 1.0 + (-A[2]) * zB) / A[0]       # base plane: every non-cage point 1.0 behind it (C)
    xB = max(xB, 0.0)
    g = L["build"](dm, 30.0, xB, zB)
    # front camera: lowest height on the L2 back plane with the L2 out of its view + 2 deg (v2's search)
    for h in ([cam_h] if cam_h else np.arange(30, 160, 2.0)):
        gg = L["build"](dm, 30.0, xB, zB, 0.0, h)
        if cam_h: g = gg; g["cam_h"] = h; break
        d = pts(gg["l2"], 3000) - gg["cam_lens"]; zz = d @ gg["cam_fwd"]
        el = np.degrees(np.arctan2(d @ gg["cam_up_v"], np.maximum(zz, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
        if not ((zz > 0) & (el > -L["CAM_VFOV"] / 2 - 2.0) & (np.abs(az) < L["CAM_HFOV"] / 2 + 2.0)).any():
            g = gg; g["cam_h"] = h; break
    l2mount = trimesh.util.concatenate([g["apex"], g["cam_bracket"]])
    keels = {k: v for k, v in g["rods"].items() if k.startswith("keel")}
    g["keels"] = keels
    # smallest sphere holding every point with its margin inside the inner face planes (rho R)
    body = np.vstack([pts(m, 800) for grp in fixed for m in grp] + [pts(g["l2"], 2000), pts(g["cam"], 300), pts(l2mount, 1500)])
    props = prop_ring_pts(mxy)
    Q = np.vstack([props, body]); M = np.r_[np.full(len(props), PROP_GAP + SHELL_R), np.full(len(body), CAGE_GAP + SHELL_R)]
    f = lambda c: float(((np.linalg.norm(Q - [c[0], 0, c[1]], axis=1) + M) / RHO).max())
    best = None
    for x_s in (dm - 20, dm, dm + 20, dm + 40):
        for z_s in (40.0, 80.0, 120.0):
            r = minimize(f, [x_s, z_s], method="Nelder-Mead", options=dict(xatol=0.05, fatol=0.01, maxiter=600))
            if best is None or r.fun < best.fun: best = r
    c = np.array([best.x[0], 0.0, best.x[1]]); R = best.fun
    # bay at the bottom: it must sit inside too (check after the shell) and its masses enter the balance
    z_o = c[2] - K * math.sqrt(R * R - ring_r(R, delta)[1] ** 2)
    b = bay(c[0] + delta, z_o)
    g.update(c=c, R=R, z_o=z_o, bay=b, mxy=mxy, xB=xB, zB=zB, dm=dm, l2mount=l2mount, delta=delta, xp=c[0] + delta)
    return g


def layout_top(g, dm, delta):
    mxy = g["motors_xy"]; t = top_l2(g, dm)
    g.update(l2=t["l2"], cam=t["cam"], cam_plate=t["cam_plate"], cam_lens=t["cam_lens"], cam_fwd=t["cam_fwd"], cam_up_v=t["cam_up_v"],
             l2mount=t["mount"], keels=t["mast"], cam_h=t["cam_dx"], B=np.array([dm, 0, t["zB"]]), xB=dm, apex=t["mount"],
             cam_bracket=t["mount"], cam_up=t["cam_up"], zb0=t["zb0"])
    fixed = [g["arms"], g["motors"], [TOP, BOTTOM, FC, DECK, PI, PDB, UBEC, BUCK, RX]]
    body = np.vstack([pts(m, 800) for grp in fixed for m in grp] + [pts(g["l2"], 2000), pts(g["cam"], 300), pts(t["mount"], 800)])
    props = prop_ring_pts(mxy)
    Q = np.vstack([props, body]); M = np.r_[np.full(len(props), PROP_GAP + SHELL_R), np.full(len(body), CAGE_GAP + SHELL_R)]
    S_ = np.array([1, 1, 1 / K])                                     # ellipsoid: a sphere in z-scaled space, margins / K (conservative)
    f = lambda c: float(((np.linalg.norm((Q - [c[0], 0, c[1]]) * S_, axis=1) + M / K) / RHO).max())
    best = None
    for z_s in (60.0, 90.0, 120.0):
        r = minimize(f, [dm, z_s], method="Nelder-Mead", options=dict(xatol=0.05, fatol=0.01, maxiter=600))
        if best is None or r.fun < best.fun: best = r
    c = np.array([best.x[0], 0.0, best.x[1]]); R = best.fun
    z_o = c[2] - K * math.sqrt(R * R - ring_r(R, delta)[1] ** 2)
    g.update(c=c, R=R, z_o=z_o, bay=bay(c[0] + delta, z_o), mxy=mxy, zB=t["zB"], dm=dm, delta=delta, xp=c[0] + delta)
    return g


def rebay(g, delta):
    """move the bay (and the opening that follows it) without rebuilding the drone"""
    R, c = g["R"], g["c"]
    z_o = c[2] - K * math.sqrt(R * R - ring_r(R, delta)[1] ** 2)
    g.update(z_o=z_o, bay=bay(c[0] + delta, z_o), delta=delta, xp=c[0] + delta)
    return g


def hanger(g):
    """triangulated hanger: each receiver top corner to its own arm root and the next one along x (8 rods)"""
    b = g["bay"]; x0 = g["xp"]; zt = b["z_top"]; rods = {}
    roots = {(sx, sy): np.array([66.5 * sx / math.sqrt(2), 66.5 * sy / math.sqrt(2), 0.0]) for sx in (1, -1) for sy in (1, -1)}
    for sx in (1, -1):
        for sy in (1, -1):
            cn = np.array([x0 + sx * REC[0] / 2, sy * REC[1] / 2, zt])
            rods[f"hanger {sx:+d}{sy:+d}"] = (roots[(sx, sy)], cn)
            rods[f"hanger diag {sx:+d}{sy:+d}"] = (roots[(-sx, sy)], cn)
    return rods


def attach(g, nodes, rods_idx, ring_ids):
    """load paths into the core: 3 spokes per arm tip, 2 ties per L2 lug, 2 ties per receiver corner to the opening ring"""
    mxy = g["mxy"]; extra = {}
    for i, ((mx, my), (r0, m1, ht)) in enumerate(zip(mxy, g["axes"])):
        tip = np.array([mx, my, ROOT_H - ht])
        cand = sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - tip))
        n_ok = 0
        for k in cand:
            if seg_band_dist(tip, nodes[k], mxy) - ROD_R >= PROP_GAP:
                extra[f"spoke {i + 1}.{n_ok + 1}"] = (tip, nodes[k]); n_ok += 1
                if n_ok == 3: break
    B = g["B"]; up_b = np.array([-A[2], 0, A[0]])
    for sy in ((1, -1) if L2_MODE == "nose" else ()):
        sock = B - A * 5 + up_b * 16 + np.array([0, 46.0 * sy, 0])
        cand = [k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - sock))
                if (nodes[k] - B) @ A < -1.0 - NODE_R][:2]
        for n_, k in enumerate(cand):
            extra[f"L2 tie {sy:+d}.{n_ + 1}"] = (sock, nodes[k])
    # core ties (review 26 Oct): the plates straight into the shell, so a hit does not bend the motor arms: from the 4 mid-edges
    # of the bottom plate underside to the 2 nearest shell nodes under the props that keep the disks 12 off
    bb = BOTTOM.bounds
    for nm_, q0 in (("front", (bb[1][0], 0)), ("rear", (bb[0][0], 0)), ("left", (0, bb[1][1])), ("right", (0, bb[0][1]))):
        q = np.array([q0[0], q0[1], -1.0]); n_ok = 0
        for k in sorted([k for k in range(len(nodes)) if k not in ring_ids], key=lambda k: np.linalg.norm(nodes[k] - q)):
            if seg_band_dist(q, nodes[k], mxy) - ROD_R >= PROP_GAP:
                extra[f"core tie {nm_}.{n_ok + 1}"] = (q, nodes[k]); n_ok += 1
                if n_ok == 2: break
    b = g["bay"]; x0 = g["xp"]
    for sx in (1, -1):                                             # receiver top corners up and out to the 2 nearest shell nodes
        for sy in (1, -1):                                         # above it (never down through the swap zone)
            cn = np.array([x0 + sx * REC[0] / 2, sy * REC[1] / 2, b["z_top"]])
            near = sorted([k for k in range(len(nodes)) if nodes[k][2] > b["z_top"] + 10 and k not in ring_ids],
                          key=lambda k: np.linalg.norm(nodes[k] - cn))[:2]
            for n_, k in enumerate(near):
                extra[f"bay tie {sx:+d}{sy:+d}.{n_ + 1}"] = (cn, nodes[k])
    return extra


def masses(g, nodes, rods, extra, ring_ids):
    """(name, g, centre) list: bought parts as cg_of v2, printed with v2's fill factors, the cage from rods and nodes"""
    b = g["bay"]; it = []
    add = lambda n, m, c: it.append((n, float(m), np.asarray(c, float)))
    cen = lambda m: m.bounding_box.centroid
    add("L2", 230, cen(g["l2"])); add("front camera", 15, cen(g["cam"])); add("down camera", 15, cen(b["down_cam"]))
    for n, gm, m in (("Pi 5", 55, PI), ("FC", 10, FC), ("PDB", 21, PDB), ("UBEC", 15, UBEC), ("buck", 6, BUCK), ("RX", 2, RX),
                     ("top plate", 23, TOP), ("bottom plate", 61, BOTTOM), ("wires", 80, FC)):
        add(n, gm, cen(m))
    add("pack", 650, cen(b["pack"])); add("flow", 2, cen(b["flow"]))
    add("latch set (receiver + cartridge)", LATCH_G, (cen(b["rec"]) * 0.5 + cen(b["cart"]) * 0.5))
    for i, m in enumerate(g["motors"]): add(f"motor + prop {i + 1}", 91, cen(m))
    for i, a in enumerate(g["arms"]): add(f"arm {i + 1}", abs(a.volume) * 1.27e-3 * 0.45 + 25, a.centroid)   # + ESC 25 (E)
    add("L2 mount + cam bracket", abs(g["apex"].volume) * 1.27e-3 * 0.45 + abs(g["cam_plate"].volume) * 1.27e-3 * 0.5, cen(g["apex"]))
    if L2_MODE == "top": it[-1] = ("L2 seat + cam plate", abs(g["l2mount"].volume) * 1.27e-3 * 0.5, cen(g["l2mount"]))
    add("Pi deck", abs(DECK.volume) * 1.27e-3 * 0.5, DECK.centroid)
    add("pods", (abs(b["cam_pod"].volume) + abs(b["flow_pod"].volume)) * 1.27e-3 * 0.3, (cen(b["cam_pod"]) + cen(b["flow_pod"])) / 2)
    for k, (p, q) in list(extra.items()) + [(f"shell {i}", (nodes[a], nodes[bb])) for i, (a, bb) in enumerate(rods)]:
        Lr = float(np.linalg.norm(np.subtract(q, p))); add(k, Lr * (SHELL_GPM if k.startswith("shell") else ROD_GPM), (np.asarray(p) + np.asarray(q)) / 2)
    for i, n in enumerate(nodes):
        add(f"node {i}", RING_G if i in ring_ids else NODE_G, n)
    return it


def cg(it):
    m = sum(x for _, x, _ in it); return m, sum(x * c for _, x, c in it) / m


BAY_G = 650 + LATCH_G + 15 + 2                                     # pack, latch set, down camera, flow (+ pods, small)


def build_all(g, rot):
    nodes, rods, tris, z_o, ring_ids = shell(g["c"], g["R"], rot, g["delta"])
    extra = attach(g, nodes, rods, ring_ids); extra.update(hanger(g)); extra.update(g["keels"])
    return nodes, rods, tris, z_o, ring_ids, extra


def solve(dm, zB, cam_h=74.0, rot=0.0):
    """balance at a fixed motor square: the bay slides along x until the CG sits on the thrust centre (nose); top: the bay
    stays on the sphere axis (pack in and out balanced) and the motor square moves onto the CG"""
    if L2_MODE == "top":
        for _ in range(12):
            g = layout(dm, zB, None, 0.0)
            nodes, rods, tris, z_o, ring_ids, extra = build_all(g, rot)
            it = masses(g, nodes, rods, extra, ring_ids); tot, c = cg(it)
            if abs(c[0] - dm) < 0.2: break
            dm = c[0]
        return g, tot, c, it
    g = layout(dm, zB, cam_h, 0.0); delta = 0.0
    for _ in range(12):
        nodes, rods, tris, z_o, ring_ids, extra = build_all(g, rot)
        it = masses(g, nodes, rods, extra, ring_ids); tot, c = cg(it)
        if abs(c[0] - dm) < 0.2: break
        delta += (dm - c[0]) * tot / BAY_G
        rebay(g, delta)
    return g, tot, c, it


# ---------------------------------------------------------------- search: motor square position (arm lengths vs opening)
os.makedirs(OUT, exist_ok=True)
ZB = 51.0                                # L2 base height as v2 (the sphere is set by the props, not the L2: inside margin > 3)
ARM_MAX = 237.0                          # front arm r: v2's accepted front arms (~197 Hz E, over 1P); longer ones fall into 1P
DMS = [float(x) for x in os.environ.get("V3_DM", "0,26,52" if L2_MODE == "nose" else "0").split(",")]
res = []
for dm in DMS:
    g, tot, c, it = solve(dm, ZB)
    fr = math.hypot(*g["mxy"][0]); n_, rr, _ = ring_r(g["R"], g["delta"])
    res.append((tot, g["dm"], fr)); dm = g["dm"]
    say(f"motors {dm:5.1f} ahead: front arms r {fr:4.0f}, rear r {math.hypot(*g['mxy'][1]):4.0f}; bay {g['delta']:+6.1f} off the sphere axis, "
        f"opening clear Ø{2 * (R_CLEAR + abs(g['delta'])):4.0f} at z {g['z_o']:6.1f}; sphere R {g['R']:5.1f}; {tot:5.0f} g, CG ({c[0]:5.1f}, {c[1]:4.1f}, {c[2]:5.1f})")
ok = [r for r in res if r[2] <= ARM_MAX + 0.5]
tot, dm, _ = min(ok)                                               # lightest with the front arms no longer than v2's
if L2_MODE == "top": dm = res[0][1] if False else dm
g, tot, c, it = solve(dm, ZB, cam_h=None)
dm = g["dm"]
best_rot = None
for rot in np.arange(0, 72, 3.0):                                  # the class-I sphere repeats every 72 deg about the pole
    nodes, rods, tris, z_o, ring_ids = shell(g["c"], g["R"], rot, g["delta"])
    worst = min(seg_band_dist(nodes[a], nodes[b], g["mxy"]) for a, b in rods) - SHELL_R
    if best_rot is None or worst > best_rot[1]: best_rot = (rot, worst)
ROT = best_rot[0]
g, tot, cgv, it = solve(dm, ZB, cam_h=g["cam_h"], rot=ROT); dm = g["dm"]
nodes, rods, tris, z_o, ring_ids, extra = build_all(g, ROT)
b = g["bay"]; zB = g["zB"]

# ---------------------------------------------------------------- checks on the chosen layout
say(f"\nchosen: motor square {dm:.1f} ahead of the plate centre (lightest of {len(DMS)} with front arms ≤ r {ARM_MAX:.0f}), bay {g['delta']:+.1f} "
    f"off the sphere axis (x {g['xp']:.1f}); geodesic NU {NU}, turned {ROT:.0f} deg about z")
say(f"sphere: R {g['R']:.1f} (C), centre ({g['c'][0]:.1f}, 0, {g['c'][2]:.1f}); inner face planes at {RHO:.4f} R (C)")
mx = g["mxy"]
say(f"motors at ({mx[0][0]:.1f}, ±{HALF}) and ({mx[1][0]:.1f}, ±{HALF}); front arms r {math.hypot(*mx[0]):.0f}, rear r {math.hypot(*mx[1]):.0f}")
if L2_MODE == "nose":
    say(f"L2: base centre ({g['xB']:.1f}, 0, {zB:.0f}), axis 15 deg below +x (Spark 24 Oct); front camera {g['cam_h']:.0f} up its back plane")
else:
    say(f"L2: axis up, base centre ({g['xB']:.1f}, 0, {zB:.1f}): every non-cage point 1.0 under its -{L2_NEG_TOP:.0f} deg cone (96 deg mode); "
        f"{zB - g['zb0']:.0f} over the lowest base so the props leave the front camera view; seat on an 8-rod mast from 4 top plate pads between the arms; "
        f"front camera {g['cam_up']:.0f} deg up, portrait, {g['cam_h']:.0f} ahead of the L2 axis, "
        f"lens ({g['cam_lens'][0]:.0f}, 0, {g['cam_lens'][2]:.0f})")
rl = {f"shell {i}": float(np.linalg.norm(nodes[a] - nodes[bb])) for i, (a, bb) in enumerate(rods)}
rl.update({k: float(np.linalg.norm(np.subtract(q, p))) for k, (p, q) in extra.items()})
carbon = {k: v for k, v in rl.items() if v >= BREAK_EVEN and not k.startswith("shell")}; printed = {k: v for k, v in rl.items() if v < BREAK_EVEN}
sh = [rl[f"shell {i}"] for i in range(len(rods))]
say(f"shell: {len(nodes)} nodes ({len(ring_ids)} on the opening ring), {len(rods)} rods, {len(tris)} triangles; rods {min(sh):.0f} .. {max(sh):.0f} "
    f"(mean {np.mean(sh):.0f}), {sum(sh) / 1000:.2f} m (C)")
circ = []
for t_ in tris:
    a_, b_, c_ = nodes[t_]; la, lb, lc = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_)
    s_ = (la + lb + lc) / 2; area = math.sqrt(max(s_ * (s_ - la) * (s_ - lb) * (s_ - lc), 1e-9)); circ.append(2 * area / s_)
say(f"largest cell: inscribed circle Ø{max(circ) - 2 * SHELL_R:.0f} between tube surfaces (C); smallest Ø{min(circ) - 2 * SHELL_R:.0f}")
say(f"load paths: {sum(k.startswith('spoke') for k in extra)} spokes (3 per arm tip), " + (f"{sum(k.startswith('L2 tie') for k in extra)} L2 ties + 2 keels, " if L2_MODE == "nose" else "8 mast rods (L2 on the core, not on the cage), ") +
    f"8 hanger rods (arm roots to the receiver corners), {sum(k.startswith('bay tie') for k in extra)} bay ties up into the shell, "
    f"{sum(k.startswith('core tie') for k in extra)} core ties (bottom plate to the shell)")
say(f"carbon: shell {len(sh)} tubes 6/4 (Spark 26 Oct), {sum(sh) / 1000:.2f} m, shortest {min(sh):.0f}, longest {max(sh):.0f}; links {len(carbon)} rods of 3.0 solid, {sum(carbon.values()) / 1000:.2f} m, shortest {min(carbon.values()):.0f} ({min(carbon, key=carbon.get)}), "
    f"longest {max(carbon.values()):.0f} ({max(carbon, key=carbon.get)}); printed struts (< {BREAK_EVEN:.0f}): "
    + (", ".join(f"{k} {v:.0f}" for k, v in printed.items()) or "none") + "; links stay 3.0 until the frame model sizes them")
pg = min(seg_band_dist(nodes[a], nodes[bb], mx) for a, bb in rods) - SHELL_R
pe_all = {k: seg_band_dist(np.asarray(p, float), np.asarray(q, float), mx) - ROD_R for k, (p, q) in extra.items()}
pe = min(pe_all.values()); pe_k = min(pe_all, key=pe_all.get)
pn = min(seg_band_dist(n, n, mx) for n in nodes) - NODE_R
say(f"props: shell rods {pg:.1f} off the swept disks (≥ {PROP_GAP:.0f}, 3D), spokes / ties / hanger / keels {pe:.1f} ({pe_k}), nodes {pn:.1f} (C)")
# opening
ring = nodes[ring_ids]
rc = np.hypot(ring[:, 0] - g["c"][0], ring[:, 1]).min() - NODE_R; rc_bay = rc - abs(g["delta"])
say(f"opening: ring of {len(ring_ids)} nodes at z {z_o:.1f}, clear circle Ø{2 * rc:.0f} (node surfaces, C), Ø{2 * rc_bay:.0f} round the bay axis, for the swap zone "
    f"{2 * ZONE[0]:.0f} x {2 * ZONE[1]:.0f} (receiver {REC[0]} x {REC[1]} M + {SPARE:.0f} a side, diagonal Ø{2 * R_CLEAR:.0f}); "
    f"cartridge bottom {CB_UP:.0f} above the ring plane, TPU feet {FOOT_DOWN:.0f} below it (lowest point z {z_o - FOOT_DOWN:.1f})")
zone = box(g["xp"] - ZONE[0], g["xp"] + ZONE[0], -ZONE[1], ZONE[1], z_o - 200, b["z_top"] - REC_H)
blk = []
cage_meshes = {f"shell {i}": rod(nodes[a], nodes[bb], SHELL_R) for i, (a, bb) in enumerate(rods)}
cage_meshes.update({k: rod(p, q) for k, (p, q) in extra.items()})
for k, m in cage_meshes.items():
    if k.startswith("hanger") or k.startswith("bay tie"):
        continue
    v = overlap(zone, m)
    if v > 0.5: blk.append((k, v))
for k in [k for k in cage_meshes if k.startswith(("hanger", "bay tie"))]:
    v = overlap(zone, cage_meshes[k])
    if v > 0.5: blk.append((k, v))
say("swap zone (receiver + spare, from the receiver down through the opening): " + ("clear (0 mm³, M)" if not blk else "; ".join(f"{k} {v:.0f} mm³" for k, v in blk)))
# everything inside: distance to the shell's inner planes
fn = np.cross(nodes[tris[:, 1]] - nodes[tris[:, 0]], nodes[tris[:, 2]] - nodes[tris[:, 0]]); fn /= np.linalg.norm(fn, axis=1)[:, None]
fn *= np.sign(np.einsum("ij,ij->i", fn, nodes[tris[:, 0]] - g["c"]))[:, None]
off = np.einsum("ij,ij->i", fn, nodes[tris[:, 0]])
inside = []
for nm, m in (("L2", g["l2"]), ("front camera", g["cam"]), ("cam bracket", g["cam_bracket"]), ("pack bay", trimesh.util.concatenate([b["cart"], b["rec"], b["cam_pod"], b["flow_pod"]])),
              ("arms", trimesh.util.concatenate(g["arms"])), ("motors", trimesh.util.concatenate(g["motors"])), ("plates + hub", trimesh.util.concatenate([TOP, BOTTOM, PI, DECK]))):
    p = pts(m, 3000); sd = (off[None, :] - p @ fn.T).min(1) - SHELL_R
    inside.append((nm, float(sd.min())))
say("inside the shell (closest point to a face plane, > 0 inside, C): " + ", ".join(f"{n} {v:.1f}" for n, v in inside))
# L2 field: share of its hemisphere blocked by rods and nodes (thin rods are allowed in sensor views, CLAUDE.md 8)
AX = A if L2_MODE == "nose" else np.array([0, 0, 1.0])
Bc = g["B"] + AX * 40.0                                            # optical centre ~40 up the axis (E)
rng = np.random.default_rng(1); D = rng.normal(size=(20000, 3)); D /= np.linalg.norm(D, axis=1)[:, None]; D = D[D @ AX > 0]
segs = [(nodes[a], nodes[bb], SHELL_R) for a, bb in rods] + [(np.asarray(p, float), np.asarray(q, float), ROD_R) for p, q in extra.values()]
hit = np.zeros(len(D), bool)
for p0, p1, rr_ in segs:
    u = p1 - p0; w0 = Bc - p0; bq = D @ u; cq = float(u @ u); dq = D @ w0; eq = float(u @ w0)
    den = np.maximum(cq - bq * bq, 1e-9)
    sq = np.clip((eq - bq * dq) / den, 0, 1); tq = -dq + sq * bq
    dist = np.linalg.norm(Bc + tq[:, None] * D - (p0 + sq[:, None] * u), axis=1)
    hit |= (dist < rr_) & (tq > 0)
for n in nodes:
    v = n - Bc; t = D @ v; hit |= (np.linalg.norm(v - t[:, None] * D, axis=1) < NODE_R) & (t > 0)
say(f"L2 field: {100 * hit.mean():.1f} % of its {'upper ' if L2_MODE == 'top' else ''}hemisphere blocked by rods and nodes (E, sampled; thin rods allowed, CLAUDE.md 8); "
    f"every non-cage point 1.0 behind the base plane by construction")
for lens, hh, hv, nm in ((b["down_lens"], 44.5, 36.5, "down camera"), (b["flow_lens"], 23.0, 23.0, "flow")):
    tgt = [("cartridge", b["cart"]), ("feet ring", trimesh.util.concatenate([rod(nodes[a], nodes[bb], SHELL_R) for a, bb in rods if a in ring_ids and bb in ring_ids]))]
    for k, m in tgt:
        d = pts(m, 3000) - lens; zz = -d[:, 2]
        n_ = ((zz > 0) & (np.abs(np.degrees(np.arctan2(d[:, 0], np.maximum(zz, 1e-6)))) < hh) & (np.abs(np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))) < hv)).sum()
        if n_: say(f"   {k} in the {nm} view ({n_} points)")
for nm, m in (("props", trimesh.util.concatenate([cyl(PROP_R, PROP_Z - 0.5, PROP_Z + 0.5, x_, y_, 96) for x_, y_ in mx])),
              ("arms", trimesh.util.concatenate(g["arms"])), ("L2", g["l2"])):
    d = pts(nm == "props" and m or m, 6000) - g["cam_lens"]; zz = d @ g["cam_fwd"]
    el = np.degrees(np.arctan2(d @ g["cam_up_v"], np.maximum(zz, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
    n_ = ((zz > 0) & (np.abs(el) < L["CAM_VFOV"] / 2) & (np.abs(az) < L["CAM_HFOV"] / 2)).mean()
    if n_: say(f"   {nm} in the front camera view: {100 * n_:.0f} % of its surface points (E)")
say("down camera and flow: views checked (lines above list anything in view)")

say(f"size: {2 * g['R']:.0f} across (sphere), {nodes[:, 2].max() + NODE_R - (z_o - FOOT_DOWN):.0f} tall (feet to the top node), "
    f"sphere top z {g['c'][2] + g['R']:.0f}; v2 was 615 x 533 x 237")
shell_g = sum(sh) * SHELL_GPM + NODE_G * (len(nodes) - len(ring_ids)) + RING_G * len(ring_ids)
att_g = sum(v for k, v in rl.items() if not k.startswith("shell")) * ROD_GPM
say(f"cage: shell {shell_g:.0f} g (6/4 tubes {sum(sh) * SHELL_GPM:.0f} E + nodes E), spokes / ties / hanger / keels {att_g:.0f} g (C); v0 cage ~96 g without the apex (E)")
it2 = [x for x in it if x[0] != "pack"] + [("cartridge out", -LATCH_G * 0.45, cg(it)[1] * 0 + (b["cart"].bounding_box.centroid))]; t2, c2 = cg(it2)
say(f"mass ~{tot:.0f} g (E; latch set {LATCH_G} M now counted, v2 had the cartridge at 4 % fill), CG ({cgv[0]:.1f}, {cgv[1]:.1f}, {cgv[2]:.1f}) "
    f"vs thrust centre ({dm:.1f}, 0): {math.hypot(cgv[0] - dm, cgv[1]):.1f} off; pack + cartridge out: {math.hypot(c2[0] - dm, c2[1]):.1f} off ({t2:.0f} g)")
Iyy = sum(m_ * ((c_[0] - cgv[0]) ** 2 + (c_[2] - cgv[2]) ** 2) for _, m_, c_ in it) * 1e-9      # point masses (E), kg m2
say(f"T/W {6600 / tot:.2f} (C on E mass, 1650 gf per motor); CG {PROP_Z - cgv[2]:.0f} under the prop plane; pitch inertia {Iyy * 1000:.1f} g m² (E, point masses); "
    f"swap reach: cartridge bottom {b['cart'].bounds[0][2] - z_o:.0f} above the opening plane; h/w {(nodes[:, 2].max() - nodes[:, 2].min()) / (np.ptp(nodes[:, 0])):.2f}")
SUMMARY = dict(K=K, bay=BAY, Iyy_gm2=Iyy * 1000, cam_up=g.get("cam_up", 0), motors_xy=[list(map(float, m_)) for m_ in g["mxy"]], delta=g["delta"], xp=g["xp"], NU=NU, R=g["R"], centre=g["c"].tolist(), rot=ROT, zB=zB, xB=g["xB"], dm=dm, z_o=z_o, mass=tot, cg=cgv.tolist(),
               cg_pack_out=c2.tolist(), nodes=len(nodes), rods=len(rods), ring=len(ring_ids), shell_m=sum(sh) / 1000, shell_g=shell_g,
               attach_g=att_g, clear_d=2 * rc, prop_gap=pg, l2_blocked=float(hit.mean()), cam_h=g["cam_h"], spare=SPARE)
json.dump(SUMMARY, open(os.path.join(OUT, "result.json"), "w"), indent=1)
np.savez(os.path.join(OUT, "shell.npz"), nodes=nodes, rods=np.array(rods), tris=tris, ring=np.array(ring_ids))
G = dict(g=g, nodes=nodes, rods=rods, tris=tris, ring_ids=ring_ids, extra=extra, it=it, z_o=z_o, dm=dm, zB=zB)
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())


# ---------------------------------------------------------------- export: part meshes, the assembly file, the motion artifact
from assembly import new as new_assembly
from motion import Scene

PD = os.path.join(OUT, "parts"); os.makedirs(PD, exist_ok=True)
rel = lambda p_: os.path.relpath(p_, ROOT)


def node_mesh(c_, r_=NODE_R):
    m_ = trimesh.creation.icosphere(1, r_); m_.apply_translation(c_); return m_


shell_rods = trimesh.util.concatenate([rod(nodes[a], nodes[bb], SHELL_R) for a, bb in rods])
shell_nodes = trimesh.util.concatenate([node_mesh(n) for i, n in enumerate(nodes) if i not in ring_ids])
ring_nodes = trimesh.util.concatenate([node_mesh(nodes[i]) for i in ring_ids])
feet = trimesh.util.concatenate([cyl(6.0, z_o - FOOT_DOWN, z_o - 2.0, nodes[i][0], nodes[i][1], 24) for i in ring_ids])
links = trimesh.util.concatenate([rod(p, q) for p, q in extra.values()])
hub = trimesh.util.concatenate([FC, PI, DECK, PDB, UBEC, BUCK, RX])
grp = {}
for n_, gm, _c in it:
    k = ("shell rods" if n_.startswith("shell") else "nodes" if n_.startswith("node") else "links" if n_.split()[0] in ("spoke", "hanger", "bay", "mast", "L2", "keel", "core")
         and n_ not in ("L2",) and not n_.startswith("L2 seat") else n_)
    grp[k] = grp.get(k, 0.0) + gm
ring_g = RING_G * len(ring_ids); grp["ring nodes + feet"] = ring_g; grp["nodes"] -= ring_g
PARTS = [  # name, mesh, role, protect, mass g, tag, material, group, colour, printed, swap
    ("cage shell tubes", shell_rods, "cage", False, grp["shell rods"], "E", "carbon", "cage", "#2b2b2b", False, False),
    ("cage shell nodes", shell_nodes, "cage", False, grp["nodes"], "E", "PETG", "cage", "#7d8a99", True, False),
    ("opening ring nodes + TPU feet", trimesh.util.concatenate([ring_nodes, feet]), "cage", False, grp["ring nodes + feet"], "E", "PETG + TPU", "cage", "#555555", True, False),
    ("spokes, mast, hanger, ties", links, "structure", False, grp["links"], "C", "carbon", "cage", "#444444", False, False),
    ("L2 seat + cam plate", g["l2mount"], "structure", False, grp.get("L2 seat + cam plate", grp.get("L2 mount + cam bracket", 0)), "E", "PETG", "printed", "#aab4c0", True, False),
    ("Unitree L2", g["l2"], "bought", True, 230, "P", None, "bought", "#222222", False, False),
    ("front camera", g["cam"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False, False),
    ("top plate", TOP, "bought", False, 23, "E", None, "frame", "#d8d8d8", False, False),
    ("bottom plate", BOTTOM, "bought", False, 61, "E", None, "frame", "#cfcfcf", False, False),
    ("hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", hub, "bought", True, 55 + 10 + 21 + 15 + 6 + 2 + grp["Pi deck"], "E", None, "bought", "#2e7d32", False, False),
    ("wires", FC, "bought", False, 80, "E", None, "bought", "#1b5e20", False, False),
    ("pack", b["pack"], "bought", True, 650, "E", None, "bought", "#2f4fbf", False, True),
    ("cartridge", b["cart"], "protected", True, LATCH_G * 0.45, "E", "PETG", "printed", "#99a6b3", True, True),
    ("receiver", b["rec"], "protected", True, LATCH_G * 0.55, "E", "PETG", "printed", "#8899aa", True, False),
    ("pods", trimesh.util.concatenate([b["cam_pod"], b["flow_pod"]]), "structure", False, grp["pods"], "E", "PETG", "printed", "#99a6b3", True, False),
    ("down camera", b["down_cam"], "bought", True, 15, "E", None, "bought", "#2d5a2d", False, False),
    ("flow MTF-02P", b["flow"], "bought", True, 2, "E", None, "bought", "#2d5a2d", False, False),
]
for i in range(4):
    PARTS.append((f"arm {i + 1}", g["arms"][i], "structure", False, grp[f"arm {i + 1}"], "E", "PETG", "printed", "#8899aa", True, False))
    PARTS.append((f"motor {i + 1}", g["motors"][i], "bought", True, 81, "E", None, "bought", "#b0b0b0", False, False))
    PARTS.append((f"prop {i + 1}", cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, *g["mxy"][i], 96), "bought", True, 10, "E", None, "bought", "#d9a400", False, False))
fname = lambda n_: "".join(ch if ch.isalnum() else "_" for ch in n_.lower()).strip("_")[:40] + ".stl"
for n_, m_, *_ in PARTS:
    m_.export(os.path.join(PD, fname(n_)))
json.dump([dict(name=n_, file=fname(n_), role=r_, protect=pr_, mass_g=round(float(gm_), 2), tag=t_, group=gr_, color=co_, printed=pt_, swap=sw_)
           for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, pt_, sw_ in PARTS], open(os.path.join(PD, "parts.json"), "w"), indent=1)
say(f"assembly mass {sum(p[4] for p in PARTS):.0f} g (E) vs the layout model {tot:.0f} g")

# the system's assembly file (owner: Overseer session); only the chosen layout writes it
SHELL_SPEC = dict(nodes=np.round(nodes, 3).tolist(), rods=[list(map(int, r_)) for r_ in rods],
                  open=[dict(name="bottom opening (pack + swap)", axis=[0, 0, -1], half_angle_deg=round(math.degrees(math.atan2(rc + NODE_R, g["c"][2] - z_o)) + 2, 1))],
                  ratio_min=0.85, node_r_mm=NODE_R, rod_r_mm=SHELL_R)
if L2_MODE == "top" and "V3_NU" not in os.environ and "V3_TUBE" not in os.environ and os.path.abspath(OUT) == os.path.join(ROOT, "runs", "overseer_layout", "v3"):
    asm_parts = []
    for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, pt_, sw_ in PARTS:
        p_ = dict(name=n_, role=r_, mesh=rel(os.path.join(PD, fname(n_))), mass_g=round(float(gm_), 2), tag=t_, owner="overseer",
                  version="layout v3 / cage v1", source="nox/overseer/layout_v3.py")
        if pr_: p_["protect"] = True
        if mat_: p_["material"] = mat_
        if sw_: p_["swap"] = True
        asm_parts.append(p_)
    contact = [["wires", "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)"], ["pack", "cartridge"], ["cartridge", "receiver"],
               ["cage shell tubes", "cage shell nodes"], ["cage shell tubes", "opening ring nodes + TPU feet"],
               ["spokes, mast, hanger, ties", "cage shell nodes"], ["spokes, mast, hanger, ties", "cage shell tubes"], ["spokes, mast, hanger, ties", "opening ring nodes + TPU feet"],
               ["spokes, mast, hanger, ties", "receiver"], ["spokes, mast, hanger, ties", "top plate"], ["spokes, mast, hanger, ties", "L2 seat + cam plate"],
               ["spokes, mast, hanger, ties", "bottom plate"], ["Unitree L2", "L2 seat + cam plate"], ["front camera", "L2 seat + cam plate"],
               ["pods", "receiver"], ["pods", "down camera"], ["pods", "flow MTF-02P"], ["hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", "bottom plate"],
               ["hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", "top plate"]]
    for i in range(1, 5):
        contact += [[f"arm {i}", "top plate"], [f"arm {i}", "bottom plate"], [f"motor {i}", f"arm {i}"], ["spokes, mast, hanger, ties", f"arm {i}"],
                    [f"arm {i}", "hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)"]]
    asm = new_assembly("Overseer", "overseer", asm_parts,
                       note=("Layout v3 / cage v1 (26 Oct, Spark: geodesic sphere cage as the Elios photo, bottom opening for the pack and the swap with "
                             "room to spare, a new funnel to follow) from nox/overseer/layout_v3.py; supersedes the cage v0 seed (runs/overseer_assembly/v0). "
                             "Rough layout: C geometry on E sizes, masses the layout's model. Drone frame: x fwd, y left, z up, z 0 bottom plate top. "
                             "Left out: ESC bodies (25 g each in the arm masses), wheel / catch features (the new funnel's), scan stick mount, XT60 and leads."),
                       cg_ref=[round(dm, 3), 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0,
                       openings=[dict(part=p_, axis=[0, 0, -1], half_angle_deg=40.0, skip_probes=True)
                                 for p_ in ("pack", "cartridge", "receiver", "down camera", "flow MTF-02P")],
                       contact_pairs=contact, guard_offsets={"shell rods to prop disks, 3D (G)": round(pg, 1)}, mass_budget_g=2200,
                       cage_shell=SHELL_SPEC)
    asm.save(os.path.join(ROOT, "nox", "overseer", "overseer.assembly.json"))
    say("wrote nox/overseer/overseer.assembly.json")
json.dump(SHELL_SPEC, open(os.path.join(OUT, "cage_shell.json"), "w"))
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
