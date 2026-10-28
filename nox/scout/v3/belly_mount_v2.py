"""Scout v3 belly mount v2 + the FC on top of the plate (27 Oct 2026). `python3 nox/scout/v3/belly_mount_v2.py` -> runs/scout_belly_mount/v2/.

Spark 27 Oct (through the cage session): "the fc can live anywhere lets just do on top". Every Scout rod stays 3.0.
System pass on main's assembled Scout v3 (layout v12, catch 5.0 forward, camera mount v1, the tail bay as drawn, PR 130), every
part in pose. v1 (FC under the plate, `belly_mount.py`, runs/scout_belly_mount/v1/) still reproduces and is kept as history; this
script imports its helpers and never changes them.
  1. FC on the plate top, placed by a search (x, yaw) against the swept prop disks (column and contracted-slipstream measures),
     the tail block's foot (as drawn and with its tip trimmed), the cage rods and the roof; the plate's own AIO holes weighed.
  2. The soft mount: 4 bonded silicone columns (male-female dampers) between the plate and the PCB, the PCB clamped to the column
     tops (nothing rigid beside the rubber); stiffness of each column (Timoshenko, guided) and the 6-DOF rigid-body modes of the
     board on them, swept over durometer and height; pick: lateral >= 110 Hz (gate 100 + 10 % for E), then the lowest vertical.
  3. FC USB-C exit search from the board's edges (the battery pads keep the nose edge, toward the lead notch).
  4. The flow sensor re-placed by v1's search (unchanged method, seeded), the plug end now toward the nose (lead to the notch).
  5. The belly mount without the FC: a closed ladder (rails on the receiver's wall tops, 4 rungs, ears, tail face, the dampers'
     4 inserts), plate-bolt variants weighed (the Bee35's own side holes vs new holes); loads (L) on each: receiver bolt group,
     prying on the ring's footprint, a moment diagram cut from the mesh every 1 mm (x) and at the rungs (y); the flow bipod as a
     3D frame (the twist from the flow's offset), its modes.
  6. Fit: our pieces against every part (full pairwise table: >= 2.0, or a declared face contact <= 0.05 mm3, no allowance);
     system_check on the assembled model (cage first, shell, fit, probes, access, balance); masses and CGs; PX4 offsets.
  7. Pictures: preview.png (side, top, rear, the FC on top close-up, the ladder slice, the bipod), preview_alternatives.png.
Drone frame: x nose, y left, z up, plate underside 18.5, top 22.0, thrust centre x 0. Numbers: nox/dimensions.md System Scout v3,
H, M, L. Tags: C computed on the v12 geometry, E estimate, P measured, M mesh, F frame model.
"""
import os, sys, json, math, io, time
import numpy as np, trimesh
from scipy import ndimage as ndi

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import belly_mount as v1                                                  # v1's helpers (imports nox/ onto the path too)
from belly_mount import (ROOT, box, cyl, union, diff, centroid, rods_all, printed_g, min_gap, overlap, section_props, bolt_group,
                         cases, prying, strut_box, extrude_xz, flow_mesh, wake_r, REC_Z, PLATE_Z, PLATE_TOP, GAP, REC_BOLTS, INSERT,
                         BOSS_R, TAIL_X, EAR_Y, EAR_X0, XT60_KO, RAIL_Y, FRONT_X, FLOW_L, FLOW_W, FLOW_T, FLOW_HALF, FLOW_WIN, CRADLE,
                         STRUT_R, STRUT_X, DISKS, DISK_R, WASH, PROP_Z, PLANE, BAND, G, PETG_E, PETG_RHO, SIG_ALLOW, WALL,
                         INSERT_PULL, INSERT_SHEAR, PLATE_BEAR, HUNG)

RUN = os.path.join(ROOT, "runs", "scout_belly_mount", "v2")
REL = lambda p: os.path.relpath(p, ROOT)                                  # repo-relative paths in everything written (review 27 Oct)
ASM = os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")
V12 = os.path.join(ROOT, "runs", "scout_layout", "v12")
LOG = io.StringIO(); v1.LOG = LOG                                         # v1's searches log into this run's log


def say(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); LOG.write(s + "\n")


# ---------------------------------------------------------------- numbers (C from the v12 meshes unless tagged)
FC_W, FC_HOLES = 36.0, 12.75              # MicoAir743v2-AIO 36 x 36, holes 25.5 square, Ø3 (M)
COMP = 3.2                                # components above and below the 1.6 PCB in the 8.0 envelope (v1 stack, E; CHECK on the board)
PCB_T = 1.6
HOLE_KO = 5.1                             # no component under the PCB within r 5.1 of a mounting hole (E, CHECK): the damper (r 3) + 2.1
D_DAMP, END_T = 6.0, 0.5                  # silicone column Ø6 (solid, bonded), metal end discs 0.5 each (E)
FOOT_X = -14.0                            # the tail block foot's nose tip on the plate top as drawn (PR 130, C mesh)
FOOT_TRIM = -20.0                         # the tip trimmed to here (requirement on the tail block, picked option)
SIDE_HOLES = [(3.9, 31.0), (3.9, -31.0)]  # the Bee35's own side holes (dimensions.md C, Plate bolts, P)
NOSE_HOLE = (45.0, 0.0)                   # the Bee35's nose hole (P): inside the battery-lead notch below
NOTCH = (44.0, 7.5)                       # battery-lead notch in the plate's nose edge: x 44.0 to the edge (~50.3), |y| <= 7.5 (DEFAULT)
XT60_CUPS = [(48.25, 3.6), (48.25, -3.6)]  # XT60E1-M solder cups, tops z 14.8 (C mesh): the leads go straight up
LEAD_OD = 3.2                             # 14 AWG silicone (E)
RAIL_IN, RAIL_OUT = 20.0, 26.5            # the ladder's rails on the receiver's wall tops (|y| 20.5..25.5, C)
RUNG_W = 6.0                              # rungs 6 wide (the post rule), full height 9.6..18.5
FRONT_RUNG = (33.0, 39.0)                 # behind the XT60 window (x 40.2), over the receiver top and lid
USB_W, USB_H, USB_L = 12.0, 7.0, 250.0    # FC USB-C 90 deg plug + cable (E, as v1)
GENT = lambda sh: 0.0981 * (56 + 7.62336 * sh) / (0.137505 * (254 - 2.54 * sh))   # Young's modulus (MPa) from Shore A (Gent, E)
STAND_RANGE = (4.2, 8.0)                  # rubber heights searched: >= 4.2 keeps the FC's lower components 2.0 off the plate


def fc_stack(h):
    """the FC stack on columns of rubber height h: plate top, end disc, rubber, end disc, PCB; envelope 3.2 below and above"""
    pcb0 = PLATE_TOP + END_T + h + END_T
    return dict(rubber=[PLATE_TOP + END_T, PLATE_TOP + END_T + h], pcb=[pcb0, pcb0 + PCB_T], env=[pcb0 - COMP, pcb0 + PCB_T + COMP])


# ---------------------------------------------------------------- the assembled model (main's assembly) without v1's FC / belly / flow
REPLACED = ("belly mount (PETG)", "FC", "wires", "flow MTF-02P")


def load_model():
    A = json.load(open(ASM)); P = {}
    for p in A["parts"]:
        n = p["name"]
        if n in REPLACED: continue
        if "mesh" in p: m = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
        else: x0, x1, y0, y1, z0, z1 = p["box"]; m = box([x0, y0, z0], [x1, y1, z1])
        P[n] = dict(mesh=m, g=float(p["mass_g"]), c=np.array(p["cg"], float) if "cg" in p else centroid(m), role=p["role"],
                    tag=p.get("tag", "E"), spec=p)
    sh = json.load(open(os.path.join(V12, "cage_shell.json")))
    links = json.load(open(os.path.join(V12, "result.json")))["links"]
    return dict(A=A, P=P, nodes=np.array(sh["nodes"], float), rods=[tuple(e) for e in sh["rods"]], shell=sh,
                links={k: (np.array(a, float), np.array(b, float)) for k, (a, b) in links.items()})


def fc_mesh(x, y, yaw_deg, st):
    """the board: PCB, the component layer above (full), the layer below less r HOLE_KO round each hole (E)"""
    z0, z1 = st["env"]; p0, p1 = st["pcb"]; h = FC_W / 2
    lo = box([-h, -h, z0], [h, h, p0]); lo = diff(lo, [cyl([sx * FC_HOLES, sy * FC_HOLES, z0 - 1], [sx * FC_HOLES, sy * FC_HOLES, p0 + 0.001], HOLE_KO)
                                                    for sx in (-1, 1) for sy in (-1, 1)])
    m = union([lo, box([-h, -h, p0], [h, h, z1])])
    T = trimesh.transformations.rotation_matrix(math.radians(yaw_deg), [0, 0, 1]); T[:3, 3] = [x, y, 0]; m.apply_transform(T); return m


def holes_xy(x, y, yaw_deg):
    a = math.radians(yaw_deg); R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return [tuple(np.round(np.array([x, y]) + R @ np.array([sx * FC_HOLES, sy * FC_HOLES]), 3)) for sx in (-1, 1) for sy in (-1, 1)]


def dampers_mesh(holes, st):
    return union([cyl([hx, hy, PLATE_TOP], [hx, hy, st["pcb"][0]], D_DAMP / 2, 48) for hx, hy in holes])


def trimmed_tail_block(tb):
    """the requirement on the tail block: its foot tip on the plate top cut back from x -14 to FOOT_TRIM (nothing else of it is at
    x > -20 above z 18)"""
    return diff(tb, [box([FOOT_TRIM, -40, PLATE_Z - 0.5], [20, 40, 60])])


# ---------------------------------------------------------------- 1. FC pose search on the plate top
def box_dist(pts, c, half, yaw_deg):
    """distance from points to an oriented box (centre c, half extents, yaw about z); 0 inside"""
    a = math.radians(yaw_deg); R = np.array([[math.cos(a), math.sin(a), 0], [-math.sin(a), math.cos(a), 0], [0, 0, 1]])
    q = (pts - c) @ R.T; d = np.maximum(np.abs(q) - half, 0.0); return np.linalg.norm(d, axis=1)


def plan_margin(x, y, yaw_deg, ztop):
    """the board's plan against each swept disk: column (outside the disk, rule 8 + 3) and contracted slipstream at the board top"""
    import shapely.geometry as sg, shapely.affinity as sa
    sq = sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), yaw_deg, origin=(0, 0)); sq = sa.translate(sq, x, y)
    d = min(sq.distance(sg.Point(mx, my)) for mx, my in DISKS)
    return d - DISK_R, d - wake_r(PROP_Z - ztop)


def fc_pose_search(M, st, tb, tb_trim):
    """x along the centre line (y 0: the disks are symmetric, any y offset loses margin), yaw 0 and 45 deg; constraints: 2.0 to the
    tail block (as drawn / trimmed), 2.0 to the cage rods, the board top under the prop sweep; objective: the column plan margin"""
    rods = rods_all(M)
    rp = np.vstack([a + np.linspace(0, 1, max(int(np.linalg.norm(b - a) / 1.0), 2))[:, None] * (b - a) for a, b, r in rods])
    rr = np.concatenate([np.full(max(int(np.linalg.norm(b - a) / 1.0), 2), r) for a, b, r in rods])
    def pts(m):
        p = np.vstack([m.vertices, m.sample(40000, seed=5)]); return p[p[:, 2] > PLATE_TOP - 0.01]
    tp, tq = pts(tb), pts(tb_trim)
    zc = (st["env"][0] + st["env"][1]) / 2; half = np.array([FC_W / 2, FC_W / 2, (st["env"][1] - st["env"][0]) / 2])
    rows = []
    for yaw in (0.0, 45.0):
        for x in np.arange(-6.0, 20.01, 0.5):
            c = np.array([x, 0.0, zc])
            col, con = plan_margin(x, 0.0, yaw, st["env"][1])
            rows.append(dict(x=float(x), yaw=yaw, col=round(col, 2), contracted=round(con, 2),
                             foot=round(float(box_dist(tp, c, half, yaw).min()), 2), foot_trim=round(float(box_dist(tq, c, half, yaw).min()), 2),
                             rods=round(float((box_dist(rp, c, half, yaw) - rr).min()), 2), aio=bool(x == 0.0 and yaw == 0.0)))
    def best(key, yaw=None):
        ok = [r for r in rows if r[key] >= GAP and r["rods"] >= GAP and (yaw is None or r["yaw"] == yaw)]
        return max(ok, key=lambda r: (r["col"], -abs(r["x"]))) if ok else None
    out = dict(rows=rows, as_drawn=best("foot"), as_drawn_yaw0=best("foot", 0.0), as_drawn_yaw45=best("foot", 45.0), trimmed=best("foot_trim"))
    for k in ("as_drawn_yaw0", "as_drawn_yaw45", "trimmed"):
        r = out[k]; say(f"FC pose, {k.replace('_', ' ')}: x {r['x']:+.1f}, yaw {r['yaw']:.0f}: plan {r['col']:.2f} outside the swept disks (column; "
                        f"{r['contracted']:.2f} outside the contracted slipstream), tail block {r['foot'] if 'drawn' in k else r['foot_trim']:.2f}, rods {r['rods']:.1f}")
    return out


def roof_limit(M, x, y, yaw):
    """the lowest cage member over the board's plan + 3 (cage first): the highest the board top may go"""
    import shapely.geometry as sg, shapely.affinity as sa
    sq = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), yaw, origin=(0, 0)), x, y).buffer(3.0)
    lo = np.inf; who = None
    for a, b, r in rods_all(M):
        q = a + np.linspace(0, 1, 400)[:, None] * (b - a)
        for p in q:
            if p[2] > PLATE_TOP and sq.contains(sg.Point(p[0], p[1])) and p[2] - r < lo: lo = p[2] - r; who = (np.round(a, 1).tolist(), np.round(b, 1).tolist())
    return lo, who


# ---------------------------------------------------------------- 2. the soft mount: column stiffness and the board's 6 modes
def column_k(G, h, d=D_DAMP):
    """one bonded silicone column, guided at the PCB (the board does not let its top rotate): lateral = shear + bending (Timoshenko),
    vertical = compression with the shape factor (E_c = E (1 + 2 k S^2), k 0.6), torsion G J / h; N/mm, N mm / rad"""
    E = 3.0 * G; A = math.pi * d * d / 4; I = math.pi * d ** 4 / 64; J = 2 * I
    k_lat = 1.0 / (h / (0.9 * G * A) + h ** 3 / (12 * E * I))
    S = d / (4 * h); k_v = E * (1 + 2 * 0.6 * S * S) * A / h
    return dict(lat=k_lat, v=k_v, tor=G * J / h)


def board_modes(G, h, holes_rel, m_g, cg_dz):
    """6-DOF rigid board on 4 columns (springs at the column tops, the PCB underside; the CG cg_dz above it): eigenfrequencies (Hz)
    with the dominant motion of each"""
    k = column_k(G, h); m = m_g / 1000.0
    Ixx = m * (FC_W ** 2 + 8.0 ** 2) / 12 * 1e-6; Izz = m * (2 * FC_W ** 2) / 12 * 1e-6
    Mm = np.diag([m, m, m, Ixx, Ixx, Izz])
    K = np.zeros((6, 6))
    for hx, hy in holes_rel:
        r = np.array([hx, hy, -cg_dz]) * 1e-3                          # spring point from the CG (m)
        T = np.hstack([np.eye(3), -np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])])   # d = u + theta x r
        Ks = np.diag([k["lat"], k["lat"], k["v"]]) * 1e3
        K += T.T @ Ks @ T
        K[5, 5] += k["tor"] * 1e-3                                       # N m / rad
    w2, V = np.linalg.eig(np.linalg.solve(Mm, K)); o = np.argsort(w2.real)
    names = ["x", "y", "z", "roll", "pitch", "yaw"]
    out = []
    for i in o:
        v = np.abs(V[:, i].real) * np.sqrt(np.diag(Mm)); out.append((round(math.sqrt(max(w2[i].real, 0)) / (2 * math.pi), 1), names[int(np.argmax(v))]))
    return out


def damper_sweep(m_g, cg_dz):
    rows = []
    for sh in (30, 40, 50):
        Gm = GENT(sh) / 3.0
        for h in np.round(np.arange(STAND_RANGE[0], STAND_RANGE[1] + 0.01, 0.2), 1):
            md = board_modes(Gm, h, [(sx * FC_HOLES, sy * FC_HOLES) for sx in (-1, 1) for sy in (-1, 1)], m_g, cg_dz)
            lat = min(f for f, n in md if n in ("x", "y")); vert = [f for f, n in md if n == "z"][0]
            rows.append(dict(shore=sh, G=round(Gm, 3), h=float(h), modes=md, lateral=lat, vertical=vert, first=md[0][0], k=column_k(Gm, h)))
    ok = [r for r in rows if r["lateral"] >= 110.0]
    pick = min(ok, key=lambda r: (r["vertical"], r["h"]))
    return rows, pick


# ---------------------------------------------------------------- 3. occupancy on a grid that reaches over the plate, the USB-C search
def occ(M, lo, hi, pitch=1.0, skip=(), rods=True, extra=()):
    from solid_check import voxels
    lo = np.asarray(lo, float); hi = np.asarray(hi, float); shape = tuple(int(round((hi[k] - lo[k]) / pitch)) for k in range(3))
    O = np.zeros(shape, bool)
    ms = [(n, p["mesh"]) for n, p in M["P"].items() if n not in skip and n not in ("cage shell tubes", "spokes, bay ties, core ties, camera guard")] + list(extra)
    for n, m in ms:
        a = np.maximum(np.floor((m.bounds[0] - lo) / pitch).astype(int) - 1, 0); b = np.minimum(np.ceil((m.bounds[1] - lo) / pitch).astype(int) + 1, shape)
        if (b <= a).any(): continue
        if m.is_watertight: O[a[0]:b[0], a[1]:b[1], a[2]:b[2]] |= voxels(m, pitch, lo + a * pitch, tuple(b - a))
        else:
            q = np.vstack([m.vertices, m.sample(20000, seed=0)]); ij = np.floor((q - lo) / pitch).astype(int); k_ = ((ij >= 0) & (ij < shape)).all(1); ij = ij[k_]
            O[ij[:, 0], ij[:, 1], ij[:, 2]] = True
    if rods:
        for a, b, r in rods_all(M):
            L = np.linalg.norm(b - a); q = a + np.linspace(0, 1, max(int(L / 0.5), 2))[:, None] * (b - a); rr = int(np.ceil(r / pitch))
            for dx in range(-rr, rr + 1):
                for dy in range(-rr, rr + 1):
                    for dz in range(-rr, rr + 1):
                        if dx * dx + dy * dy + dz * dz > (r / pitch + 0.5) ** 2: continue
                        ij = np.floor((q - lo) / pitch).astype(int) + [dx, dy, dz]; k_ = ((ij >= 0) & (ij < shape)).all(1); ij = ij[k_]
                        O[ij[:, 0], ij[:, 1], ij[:, 2]] = True
    return O, lo, pitch


def usb_search2(M, fc_xy, st):
    """from each board edge (the nose edge keeps the battery pads, DEFAULT), port centres every 2 along the edge, z = PCB top + 1.6;
    the 12 x 7 plug + cable swept 250 at azimuth +-60 and elevation -10..+30 deg (5 deg steps): every sample clear of every part and
    rod (1 mm grid) but the FC and its dampers. Objective: the straightest exit (|az| + |el|), then a side edge, then +y"""
    O, lo, p = occ(M, [-150, -120, -20], [170, 120, 100], 1.0, skip=("FC", "FC dampers (4, silicone)", "wires", "tail bay cables (Scout electronics, E)"))
    shp = np.array(O.shape); zc = st["pcb"][1] + 1.6
    uu, vv = np.meshgrid(np.linspace(-USB_W / 2, USB_W / 2, 7), np.linspace(-USB_H / 2, USB_H / 2, 5)); uu = uu.ravel(); vv = vv.ravel()
    t = np.arange(1.0, USB_L + 0.1, 1.0)
    out = []; n = 0
    edges = {"+y": (np.array([0, 1.0]), np.array([1.0, 0])), "-y": (np.array([0, -1.0]), np.array([1.0, 0])), "-x": (np.array([-1.0, 0]), np.array([0, 1.0])),
             "+x": (np.array([1.0, 0]), np.array([0, 1.0]))}
    for e, (nrm, tan) in edges.items():
        for s in np.arange(-12.0, 12.01, 2.0):
            p0 = np.r_[np.array(fc_xy) + nrm * FC_W / 2 + tan * s, zc]
            for az in np.arange(-60, 61, 5):
                for el in np.arange(-10, 31, 5):
                    n += 1
                    a_, e_ = math.radians(az), math.radians(el)
                    dh = nrm * math.cos(a_) + tan * math.sin(a_); d = np.r_[dh * math.cos(e_), math.sin(e_)]
                    w = np.cross([0, 0, 1.0], d); w /= np.linalg.norm(w); hv = np.cross(d, w)
                    q = (p0[None, None] + t[:, None, None] * d + uu[None, :, None] * w + vv[None, :, None] * hv).reshape(-1, 3)
                    ij = np.floor((q - lo) / p).astype(int); k_ = ((ij >= 0) & (ij < shp)).all(1)
                    if O[ij[k_, 0], ij[k_, 1], ij[k_, 2]].any(): continue
                    out.append((abs(az) + abs(el) + (0.0 if e in ("+y", "-y") else 0.5) + (0.0 if e == "+y" else 0.1), e, float(s), float(az), float(el), d, p0))
    out.sort(key=lambda r: (r[0], abs(r[2]), r[1], r[3], r[4]))
    say(f"USB-C search: {len(out)} clear of {n} (edge, offset, azimuth, elevation); by edge: "
        + ", ".join(f"{e} {sum(1 for r in out if r[1] == e)}" for e in edges))
    return [r for r in out if r[1] != "+x"], out


# ---------------------------------------------------------------- 5. the belly mount v2, drawn
VARIANTS = {
    "VA": dict(bolts=SIDE_HOLES, new=[], label="the Bee35's 2 side holes (3.9, +-31), 0 new holes"),
    "VB": dict(bolts=SIDE_HOLES + [(-25.0, 13.0), (-25.0, -13.0)], new=[(-25.0, 13.0), (-25.0, -13.0)], label="side holes + 2 new at (-25, +-13)"),
    "VC": dict(bolts=[(-25.0, 13.0), (-25.0, -13.0), (30.0, 13.0), (30.0, -13.0)], new=[(-25.0, 13.0), (-25.0, -13.0), (30.0, 13.0), (30.0, -13.0)],
               label="v1's 4 new holes (-25, +-13), (30, +-13)"),
}


def bipod_cradle(flow):
    """v1's bipod and cradle (pads on the -y rail at x -+24, 2 struts 6 x 6, gussets, the tie bar, the 2.0 cradle, windows open)"""
    zz = (REC_Z, PLATE_Z); B = []
    fx, fy = flow["c"]; zb = flow["zb"]; hx, hy = (FLOW_L / 2, FLOW_W / 2) if flow["ori"][0] == "x" else (FLOW_W / 2, FLOW_L / 2)
    py = flow["post_y"]; ztop_c = zb + FLOW_T + CRADLE; FS = 1 if py > 0 else -1
    for sx in (-1, 1):
        yp0, yp1 = sorted((FS * (RAIL_IN + 1.0), py + FS * STRUT_R))
        x0, x1 = sorted((sx * (STRUT_X - STRUT_R - 3.0), sx * (STRUT_X + STRUT_R + 3.0)))
        B.append(box([x0, yp0, zz[0]], [x1, yp1, zz[1]]))
    st = v1.flow_struts(flow)
    for p0, p1 in st:
        B.append(strut_box(p0, p1))
        sx = 1 if p0[0] < 0 else -1; u = (p1 - p0) / np.linalg.norm(p1 - p0); a = p0 + u * 9.0
        tri = np.array([[p0[0] + sx * STRUT_R, zz[0] + 0.01], [p0[0] + sx * (STRUT_R + 7.0), zz[0] + 0.01], [a[0] + sx * STRUT_R * 0.5, a[2]]])
        B.append(extrude_xz(tri, py - 1.5, py + 1.5))
    B.append(box([st[0][1][0] - STRUT_R, py - STRUT_R, ztop_c - 1.0], [st[1][1][0] + STRUT_R, py + STRUT_R, ztop_c + 2 * STRUT_R - 1.0]))
    c0 = np.array([fx - hx - CRADLE, fy - hy - CRADLE, zb - CRADLE]); c1 = np.array([fx + hx + CRADLE, fy + hy + CRADLE, ztop_c])
    cuts_c = [box([fx - hx, fy - hy, zb], [fx + hx, fy + hy, zb + FLOW_T])]
    lip = 2.0
    if flow["ori"][0] == "x":
        cuts_c.append(box([fx - hx, fy - hy + lip, zb - CRADLE - 1], [fx + hx, fy + hy - lip, zb + 0.01]))
        pe = fx - hx if flow["ori"][1] == "-" else fx + hx
        cuts_c.append(box([pe - CRADLE - 1 if flow["ori"][1] == "-" else pe - 0.01, fy - hy + lip, zb - CRADLE - 1],
                          [pe + 0.01 if flow["ori"][1] == "-" else pe + CRADLE + 1, fy + hy - lip, zb + FLOW_T]))
    else:
        cuts_c.append(box([fx - hx + lip, fy - hy, zb - CRADLE - 1], [fx + hx - lip, fy + hy, zb + 0.01]))
        pe = fy - hy if flow["ori"][1] == "-" else fy + hy
        cuts_c.append(box([fx - hx + lip, pe - CRADLE - 1 if flow["ori"][1] == "-" else pe - 0.01, zb - CRADLE - 1],
                          [fx + hx - lip, pe + 0.01 if flow["ori"][1] == "-" else pe + CRADLE + 1, zb + FLOW_T]))
    B.append(diff(box(c0, c1), cuts_c))
    return B


def belly_v2(flow, variant, damper_holes, front_rung=False, damper="none", rail_out=RAIL_OUT):
    """the U-frame: rails |y| 20..rail_out on the receiver's wall tops from the tail face to the front receiver bosses, the rear
    rung (the tail face, x -31) and the ears; a Ø10 boss under each receiver insert and plate bolt; the FC's dampers screw into M3
    nuts bonded under the plate (damper="none", DEFAULT: the FC needs nothing of this piece), or into inserts here: each a pod on a
    6-wide link to its rail (damper="pod") or the 2 of a side on one rung (damper="rung"); a front rung behind the XT60 only if
    asked (the receiver, bolted at its 4 corners, closes the box at the nose); VC: 2 yokes on v1's 4 new holes, no rail beside the
    middle (x -18..18), joined through the bipod and cradle as v1; the bipod and cradle. All between the plate (z 18.5) and the
    receiver (z 9.6)."""
    V = VARIANTS[variant]; zz = (REC_Z, PLATE_Z); B = []
    rear_x1 = -20.0 if any(abs(bx + 25.0) < 0.1 for bx, by in V["bolts"]) else TAIL_X + RUNG_W
    split = variant == "VC" and damper == "none"
    B.append(box([TAIL_X, -EAR_Y, zz[0]], [rear_x1, EAR_Y, zz[1]]))                     # rear rung: the tongue's bearing face
    for sy in (1, -1):
        y0, y1 = sorted((sy * EAR_Y, sy * RAIL_Y)); B.append(box([EAR_X0, y0, zz[0]], [TAIL_X, y1, zz[1]]))   # ear on the tail block (|y| 21)
        y0, y1 = sorted((sy * RAIL_IN, sy * rail_out))
        for xa, xb in (((TAIL_X - 0.01, -18.0), (18.0, REC_BOLTS[2][0])) if split else ((TAIL_X - 0.01, REC_BOLTS[2][0]),)):
            B.append(box([xa, y0, zz[0]], [xb, y1, zz[1]]))                              # rail (VC: 2 stubs)
    if damper == "none": damper_holes = []
    if damper == "rung":
        for xd in sorted({round(hx, 3) for hx, hy in damper_holes}):
            B.append(box([xd - RUNG_W / 2, -RAIL_IN - 0.01, zz[0]], [xd + RUNG_W / 2, RAIL_IN + 0.01, zz[1]]))
    else:
        for hx, hy in damper_holes:                                                       # pod: boss + a 6-wide link into the rail
            sy = 1 if hy > 0 else -1; y0, y1 = sorted((hy, sy * (RAIL_IN + 1.0)))
            B.append(box([hx - RUNG_W / 2, y0, zz[0]], [hx + RUNG_W / 2, y1, zz[1]]))
    if front_rung or any(abs(bx - 30.0) < 0.1 for bx, by in V["bolts"]):
        fr = (25.0, 35.0) if any(abs(bx - 30.0) < 0.1 for bx, by in V["bolts"]) else FRONT_RUNG
        B.append(box([fr[0], -RAIL_IN - 0.01, zz[0]], [fr[1], RAIL_IN + 0.01, zz[1]]))     # front rung
    for x_, y_ in REC_BOLTS + list(V["bolts"]) + list(damper_holes): B.append(cyl([x_, y_, zz[0]], [x_, y_, zz[1]], BOSS_R))
    B += bipod_cradle(flow)
    body = union(B)
    cuts = [box([XT60_KO[0], -XT60_KO[2], zz[0] - 1], [XT60_KO[1], XT60_KO[2], zz[1] + 1])]   # XT60E1-M + 2.0 (the leads go straight up)
    for x_, y_ in REC_BOLTS: cuts.append(cyl([x_, y_, zz[0] - 1], [x_, y_, zz[0] + INSERT[1]], INSERT[0] / 2, 32))
    for x_, y_ in list(V["bolts"]) + list(damper_holes): cuts.append(cyl([x_, y_, zz[1] - INSERT[1]], [x_, y_, zz[1] + 1], INSERT[0] / 2, 32))
    return diff(body, cuts)


# ---------------------------------------------------------------- loads on the ladder (L): bolt groups, prying, the moment diagram
def frame_sections(bm, xs, ys):
    fo = trimesh.boolean.intersection([bm, box([-100, -100, REC_Z - 0.01], [100, 100, 30])], engine="manifold")
    sx = {}; sy = {}
    for x in xs:
        s = section_props(fo, [x, 0, 14.0], [1, 0, 0]); sx[round(float(x), 2)] = s
    for y in ys:
        s = section_props(fo, [0, y, 14.0], [0, 1, 0]); sy[round(float(y), 2)] = s
    return sx, sy


def ring_hull(bm):
    import shapely.geometry as sg
    from shapely.ops import unary_union
    s = bm.section(plane_origin=[0, 0, PLATE_Z - 0.3], plane_normal=[0, 0, 1])
    return unary_union([sg.Polygon(e[:, :2]) for e in s.discrete]).convex_hull


def ladder_loads(P, bm, bm_g, flow, flow_g, bolts, secs_x, secs_y):
    """every L case: the hung set into the 4 receiver inserts (elastic bolt group), the ladder prying about the worst edge of its
    footprint on the plate (the plate bolts take T_b = k d_b), shear shared; the moment about y at every x cut (both rails and the
    rungs together) and about x at the rung cuts, from the insert pulls, the bolt tensions and the bearing at the pivot edge; the
    flow and the ladder's own weight at the pads / CG. Index = M / Z / 25 MPa (2.4 walls)."""
    hung = [(P[n]["g"], P[n]["c"]) for n in HUNG]; mh = sum(g for g, _ in hung); ch = sum(g * c for g, c in hung) / mh
    bay_g = sum(P[n]["g"] for n in ("electronics tail block (PETG)", "electronics tail carrier (PETG)", "Radxa Zero 3W", "UBEC", "XR2")) + 0.5 * 6.27
    hull = ring_hull(bm); hc = np.array(hull.exterior.coords)[:-1]
    pads = [(-STRUT_X, flow["post_y"]), (STRUT_X, flow["post_y"])]
    Zx = {k: v["Z"] for k, v in secs_x.items() if v and v["Z"] > 0}; Zy = {k: v["Z"] for k, v in secs_y.items() if v and v["Z"] > 0}
    rows = []
    for name, a, kind, lim in cases():
        acc = a * G / 1000.0
        T1, S1 = bolt_group(mh * acc, ch, REC_BOLTS, REC_Z)
        rec_idx = float(np.max(np.sqrt((T1 / INSERT_PULL) ** 2 + (S1 / INSERT_SHEAR) ** 2)))
        Ff = flow_g * acc; Fo = bm_g * acc; Ft = max(a[0], 0.0) * bay_g * G / 1000.0 if kind == "crash" else 0.0
        pulls = [(REC_BOLTS[i], float(T1[i])) for i in range(4) if T1[i] > 0]
        down = [((px, py), max(-Ff[2], 0.0) / 2) for px, py in pads] + [((2.0, -10.0), max(-Fo[2], 0.0))]   # flow at the pads, own weight
        Tb, piv = prying(hull, pulls + [l for l in down if l[1] > 0] or [((0, 0), 0.0)], bolts)
        Tb = np.asarray(Tb, float)
        S = (float(np.hypot(*(mh * acc[:2]))) + float(np.hypot(*Ff[:2])) + float(np.hypot(*Fo[:2])) + Ft) / len(bolts)   # shear shared
        pl_idx = float(np.max(np.sqrt((Tb / INSERT_PULL) ** 2 + (S / INSERT_SHEAR) ** 2)))
        carbon_idx = float(S / (3.0 * 3.5 * PLATE_BEAR))
        # forces on the ladder (up +): pulls down, bolts up, bearing at the pivot edge (equilibrium)
        F = [(p[0], p[1], -T) for p, T in pulls + down if T > 0] + [(bx, by, float(t)) for (bx, by), t in zip(bolts, Tb)]
        R = -sum(f[2] for f in F)                                          # the plate's bearing on the ladder, placed where both
        if abs(R) > 1e-6:                                                  # moments close (the prying edge's own position is not known
            xr = -sum(f[2] * f[0] for f in F) / R; yr = -sum(f[2] * f[1] for f in F) / R   # along it): the diagrams end at 0
            xr = float(np.clip(xr, hc[:, 0].min(), hc[:, 0].max())); yr = float(np.clip(yr, hc[:, 1].min(), hc[:, 1].max()))
            F.append((xr, yr, R))
        def My(c): return sum(fz * (c - fx) for fx, fy, fz in F if fx < c)
        def Mx(c): return sum(fz * (c - fy) for fx, fy, fz in F if fy < c)
        fx_ = max(((abs(My(c)) / z_ / SIG_ALLOW, c) for c, z_ in Zx.items()), default=(0.0, None))
        fy_ = max(((abs(Mx(c)) / z_ / SIG_ALLOW, c) for c, z_ in Zy.items()), default=(0.0, None))
        rows.append(dict(case=name, kind=kind, limit=lim, rec_T=round(float(T1.max()), 1), rec_idx=round(rec_idx, 4),
                         plate_T=round(float(Tb.max()), 1), plate_S=round(float(S), 1), plate_idx=round(pl_idx, 4),
                         pry=round(float(Tb.max() / max(T1.max(), 1e-9)), 2), frame_x_idx=round(fx_[0], 4), frame_x_at=fx_[1],
                         frame_y_idx=round(fy_[0], 4), frame_y_at=fy_[1], carbon_idx=round(carbon_idx, 4),
                         tongue_N=round(float(Ft), 1), tongue_idx=round(float(Ft / (26.0 * (PLATE_Z - REC_Z)) / SIG_ALLOW), 4)))
    by = {}
    for r in rows:
        b = by.setdefault(r["kind"], dict(limit=r["limit"]))
        for f_ in ("rec_idx", "plate_idx", "frame_x_idx", "frame_y_idx", "carbon_idx", "tongue_idx", "pry"):
            if r[f_] > b.get(f_, (-1, ""))[0]: b[f_] = (r[f_], r["case"], r.get("frame_x_at") if f_ == "frame_x_idx" else (r.get("frame_y_at") if f_ == "frame_y_idx" else None))
    return dict(rows=rows, by=by, hung_g=round(mh, 1), hung_cg=np.round(ch, 2).tolist(), hull=np.round(hc, 1).tolist())


# ---------------------------------------------------------------- the flow bipod as a 3D frame (the twist, review 27 Oct)
def frame3d(nodes, elems, fixed, loads, masses=None):
    """linear 3D frame (Euler-Bernoulli, 12 dof elements): nodes (n, 3) mm, elems [(i, j, E, G, A, Iy, Iz, J)], fixed node ids,
    loads {node: (Fx, Fy, Fz, Mx, My, Mz)} N / N mm; returns displacements and each element's end forces in its local frame.
    masses {node: (m_kg, Ixx, Iyy, Izz) (kg m^2)} -> also the first modes (Hz)."""
    n = len(nodes); K = np.zeros((6 * n, 6 * n)); Ts = []
    for i, j, E, Gs, A, Iy, Iz, J in elems:
        d = nodes[j] - nodes[i]; L = float(np.linalg.norm(d)); ex = d / L
        ref = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([0, 1.0, 0])
        ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey); R = np.vstack([ex, ey, ez])
        T = np.kron(np.eye(4), R)
        k = np.zeros((12, 12)); a = E * A / L; t = Gs * J / L
        k[0, 0] = k[6, 6] = a; k[0, 6] = k[6, 0] = -a; k[3, 3] = k[9, 9] = t; k[3, 9] = k[9, 3] = -t
        for (I, v, th, s) in ((Iz, 1, 5, 1), (Iy, 2, 4, -1)):
            c1 = 12 * E * I / L ** 3; c2 = 6 * E * I / L ** 2; c3 = 4 * E * I / L; c4 = 2 * E * I / L
            idx = [v, th, 6 + v, 6 + th]
            kk = np.array([[c1, s * c2, -c1, s * c2], [s * c2, c3, -s * c2, c4], [-c1, -s * c2, c1, -s * c2], [s * c2, c4, -s * c2, c3]])
            for p in range(4):
                for q in range(4): k[idx[p], idx[q]] += kk[p, q]
        kg = T.T @ k @ T; dof = list(range(6 * i, 6 * i + 6)) + list(range(6 * j, 6 * j + 6))
        K[np.ix_(dof, dof)] += kg; Ts.append((T, k, dof, L))
    free = [d for d in range(6 * n) if d // 6 not in fixed]
    F = np.zeros(6 * n)
    for nd, f in loads.items(): F[6 * nd:6 * nd + 6] += f
    U = np.zeros(6 * n); U[free] = np.linalg.solve(K[np.ix_(free, free)], F[free])
    ef = [k @ (T @ U[dof]) for T, k, dof, L in Ts]
    modes = None
    if masses:
        Mm = np.zeros((6 * n, 6 * n))
        for nd, (m, ix, iy, iz) in masses.items():
            Mm[6 * nd:6 * nd + 3, 6 * nd:6 * nd + 3] = np.eye(3) * m * 1e-3          # N/mm stiffness with kg mass: scale 1e-3
            Mm[6 * nd + 3:6 * nd + 6, 6 * nd + 3:6 * nd + 6] = np.diag([ix, iy, iz]) * 1e3   # N mm/rad with kg m^2 -> kg mm^2 * 1e-3
        Kf = K[np.ix_(free, free)]; Mf = Mm[np.ix_(free, free)] + np.eye(len(free)) * 1e-12
        w2 = np.sort(np.linalg.eigvals(np.linalg.solve(Mf, Kf)).real)
        modes = [round(math.sqrt(max(w, 0)) / (2 * math.pi), 0) for w in w2[:4]]
    return U, ef, modes


def bipod_frame(flow, flow_g):
    """the 2 struts (6 x 6 PETG) from their pads, the tie bar between their feet, and the cradle as a rigid arm from the tie bar's
    middle to the flow + cradle CG (y -40: 9.5 off the strut plane, the twist); all 14 crash directions, flight, landing"""
    st = v1.flow_struts(flow); fc = np.array([flow["c"][0], flow["c"][1], flow["zb"] + FLOW_T / 2])
    tm = (st[0][1] + st[1][1]) / 2
    nodes = np.array([st[0][0], st[1][0], st[0][1], st[1][1], tm, fc], float)
    A = 36.0; I = 6.0 ** 4 / 12; J = 0.1406 * 6.0 ** 4; Gp = PETG_E / 2.6; Er = PETG_E * 100
    elems = [(0, 2, PETG_E, Gp, A, I, I, J), (1, 3, PETG_E, Gp, A, I, I, J), (2, 4, PETG_E, Gp, A, I, I, J), (4, 3, PETG_E, Gp, A, I, I, J),
             (4, 5, Er, Er / 2.6, 400.0, 1e5, 1e5, 2e5)]                     # the cradle: rigid
    m = flow_g / 1000.0; Ib = m * (FLOW_L ** 2 + FLOW_W ** 2) / 12 * 1e-6
    out = []; worst = (0.0, None)
    for name, a, kind, lim in cases():
        Fv = flow_g * a * G / 1000.0
        U, ef, _ = frame3d(nodes, elems, {0, 1}, {5: np.r_[Fv, 0, 0, 0]})
        idx = 0.0
        for e in ef[:4]:                                                  # each strut / tie end: N / A + |My| / Z + |Mz| / Z + T / Zt
            for end in (e[:6], e[6:]):
                s = abs(end[0]) / A + (abs(end[4]) + abs(end[5])) / (6.0 ** 3 / 6) + abs(end[3]) / (0.208 * 6.0 ** 3)
                idx = max(idx, s / SIG_ALLOW)
        out.append(dict(case=name, kind=kind, idx=round(idx, 4), defl=round(float(np.linalg.norm(U[30:33])), 4)))
        if idx > worst[0]: worst = (idx, name)
    _, _, modes = frame3d(nodes, elems, {0, 1}, {}, masses={5: (m, Ib, Ib, Ib)})
    # the same without the offset (the flow's CG in the strut plane): what the twist costs
    n2 = nodes.copy(); n2[5, 1] = n2[4, 1]
    _, _, modes0 = frame3d(n2, elems, {0, 1}, {}, masses={5: (m, Ib, Ib, Ib)})
    w0 = 0.0
    for name, a, kind, lim in cases():
        Fv = flow_g * a * G / 1000.0; U, ef, _ = frame3d(n2, elems, {0, 1}, {5: np.r_[Fv, 0, 0, 0]})
        for e in ef[:4]:
            for end in (e[:6], e[6:]):
                w0 = max(w0, (abs(end[0]) / A + (abs(end[4]) + abs(end[5])) / (6.0 ** 3 / 6) + abs(end[3]) / (0.208 * 6.0 ** 3)) / SIG_ALLOW)
    by = {}
    for r in out:
        b = by.setdefault(r["kind"], (0.0, ""))
        if r["idx"] > b[0]: by[r["kind"]] = (r["idx"], r["case"])
    return dict(by_kind=by, worst=worst, modes_Hz=modes, modes_no_offset_Hz=modes0, worst_no_offset=round(w0, 4),
                offset_y=round(float(fc[1] - tm[1]), 1), strut_L=round(float(np.linalg.norm(st[0][1] - st[0][0])), 1))


# ---------------------------------------------------------------- 6. fit: our pieces against every part (full table)
BM, FCN, DMP, FLW, TBN = "belly mount (PETG)", "FC", "FC dampers (4, silicone)", "flow MTF-02P", "electronics tail block (PETG)"
OURS = (BM, FCN, DMP, FLW, TBN)
CONTACT = {frozenset(p) for p in ((BM, "frame plate (Bee35 carbon)"), (BM, "receiver v3 (latch)"), (BM, "receiver lid (latch)"), (BM, TBN), (BM, FLW),
                                  (DMP, "frame plate (Bee35 carbon)"), (DMP, FCN), (TBN, "frame plate (Bee35 carbon)"), (TBN, "receiver v3 (latch)"),
                                  (TBN, "electronics tail carrier (PETG)"))}


def bbox_gap(a, b):
    d = np.maximum(np.maximum(a.bounds[0] - b.bounds[1], b.bounds[0] - a.bounds[1]), 0.0); return float(np.linalg.norm(d))


def fit_full(P, rods):
    """every pair of ours with every part: min gap (sampled, 6000 + vertices each side) when the boxes are within 25, else the box gap
    as a lower bound; overlap volume when closer than 0.5. Declared face contacts pass at <= 0.05 mm3, every other pair needs 2.0."""
    rows = []; names = list(P)
    for a in OURS:
        for b in names:
            if b == a or (b in OURS and OURS.index(b) < OURS.index(a)): continue
            ma, mb = P[a]["mesh"], P[b]["mesh"]; bg = bbox_gap(ma, mb); dec = frozenset((a, b)) in CONTACT
            if bg > 25.0:
                rows.append(dict(a=a, b=b, overlap_mm3=0.0, gap_mm=round(bg, 1), lower_bound=True, kind="declared contact" if dec else ">= 2", pass_=True)); continue
            g = min_gap(ma, mb); ov = overlap(ma, mb) if g < 0.5 else 0.0
            ok = (ov <= 0.05) if dec else (g >= GAP - 1e-6)
            rows.append(dict(a=a, b=b, overlap_mm3=round(ov, 3), gap_mm=round(g, 2), lower_bound=False, kind="declared contact" if dec else ">= 2", pass_=bool(ok)))
    # inside the declared contacts, the faces that must not touch: the belly mount below its contact plane (bipod, gussets, cradle)
    # against the receiver set, and the columns' sides against the FC's component layer under the PCB
    lower = trimesh.boolean.intersection([P[BM]["mesh"], box([-100, -100, -80], [100, 100, REC_Z - 0.6])], engine="manifold")
    for b in ("receiver v3 (latch)", "receiver lid (latch)", "receiver TPU end pieces (latch, E)"):
        g = min_gap(lower, P[b]["mesh"])
        rows.append(dict(a="belly mount below z 9.0 (bipod, gussets, cradle)", b=b, overlap_mm3=0.0, gap_mm=round(g, 2), lower_bound=False, kind=">= 2", pass_=bool(g >= GAP)))
    fcm = P[FCN]["mesh"]; zp = float(fcm.bounds[0][2]) + COMP
    comp = trimesh.boolean.intersection([fcm, box([-100, -100, 0], [100, 100, zp - 0.01])], engine="manifold")
    g = min_gap(P[DMP]["mesh"], comp)
    rows.append(dict(a=DMP, b="FC component layer under the PCB", overlap_mm3=round(overlap(P[DMP]["mesh"], comp), 3), gap_mm=round(g, 2), lower_bound=False,
                     kind=">= 2", pass_=bool(g >= GAP - 1e-6)))
    for a in OURS:
        m = P[a]["mesh"]; pts = np.vstack([m.vertices, m.sample(6000, seed=3)]); best = (np.inf, None)
        for p, q, r in rods:
            u = q - p; t = np.clip(((pts - p) @ u) / (u @ u), 0, 1); d = np.linalg.norm(pts - (p + t[:, None] * u), axis=1).min() - r
            if d < best[0]: best = (d, (np.round(p, 1).tolist(), np.round(q, 1).tolist()))
        rows.append(dict(a=a, b="cage rods / links (nearest)", overlap_mm3=0.0, gap_mm=round(float(best[0]), 2), lower_bound=False, kind=">= 2",
                         pass_=bool(best[0] >= GAP), rod=best[1]))
    return rows


# ---------------------------------------------------------------- 7. the system model with our pieces: system_check (repo-relative)
def write_stl(m, name):
    p = os.path.join(RUN, name); m.export(p); return REL(p)


def manifest(M, P, files, usb, flow, holes, bolts, st, wires):
    A = json.loads(json.dumps(M["A"]))
    parts = []
    for p in A["parts"]:
        n = p["name"]
        if n in REPLACED: continue
        q = dict(p)
        if n == TBN:
            q.update(mesh=files[TBN], mass_g=round(P[TBN]["g"], 2), cg=np.round(P[TBN]["c"], 2).tolist(),
                     note="PR 130's block with the requirement applied: its foot tip on the plate top cut back from x -14.0 to -20.0 (belly mount v2)")
        if n == "tail bay cables (Scout electronics, E)":
            q.pop("mesh", None); q["box"] = list(CABLE_BOX)
            q["note"] = "6.27 g at its CG (-8.3, 2.3, 31.6) as main; for the fit a 0.4 box over the FC on top (main's box sat where the FC now is)"
        parts.append(q)
    parts.append(dict(name=FCN, role="bought", protect=True, mesh=files[FCN], mass_g=10.0, cg=np.round(P[FCN]["c"], 2).tolist(), tag="P", geom_tag="C",
                      note=f"MicoAir743v2-AIO 36 x 36 x 8 on the plate top, on the AIO holes, z {st['env'][0]}..{st['env'][1]} (belly mount v2)"))
    parts.append(dict(name=DMP, role="bought", mesh=files[DMP], mass_g=DMP_G, cg=np.round(P[DMP]["c"], 2).tolist(), tag="E", geom_tag="C",
                      note="4 M3 male-female bonded silicone columns Ø6, rubber %.1f, 30A (E), studs into 4 M3 nuts epoxied under the plate (0.8 g of the %.1f g)" % (st["rubber"][1] - st["rubber"][0], DMP_G)))
    b_ = wires["box"]
    parts.append(dict(name="wires", role="structure", box=[b_[0][0], b_[1][0], b_[0][1], b_[1][1], b_[0][2], b_[1][2]], cg=wires["cg"], mass_g=20.0, tag="E",
                      note=wires["note"]))
    parts.append(dict(name=FLW, role="bought", protect=True, mesh=files[FLW], mass_g=1.5, tag="P", geom_tag="C"))
    parts.append(dict(name=BM, role="structure", mesh=files[BM], material="PETG", mass_g=round(P[BM]["g"], 2), cg=np.round(P[BM]["c"], 2).tolist(),
                      tag="E", geom_tag="C", owner="scout electronics", version="belly mount v2 (concept)", source="nox/scout/v3/belly_mount_v2.py"))
    A["parts"] = parts
    names = {p["name"] for p in parts}
    cp = [c for c in A["contact_pairs"] if set(c[:2]) <= names and not (set(REPLACED) & set(c[:2])) and frozenset(c[:2]) not in CONTACT]
    cp += [[a, b, 0.05] for a, b in sorted(tuple(sorted(x)) for x in CONTACT)]
    A["contact_pairs"] = cp
    A["clearance_pairs"] = ([[BM, n, GAP] for n in ("XT60E1-M", "tie saddle (printed)", "receiver TPU end pieces (latch, E)", "cartridge v6 (latch)")]
                            + [[FLW, n, GAP] for n in ("receiver v3 (latch)", "tie saddle (printed)")]
                            + [[FCN, n, GAP] for n in ("frame plate (Bee35 carbon)", TBN, "tail bay cables (Scout electronics, E)", "wires")]
                            + [[DMP, TBN, GAP]])
    acc = []
    for a in A["access"]:
        if a["name"].startswith(("FC USB-C", "M2 key, FC grommet", "M3 key, belly-mount plate bolt", "flow SH1.0")): continue
        a = dict(a); a["ignore"] = [i for i in a.get("ignore", []) if i != "wires"] + ["wires"]; acc.append(a)
    w = np.cross([0, 0, 1.0], usb["d"]); w /= np.linalg.norm(w)
    acc.append({"name": f"FC USB-C cable (12 x 7, from the FC's {usb['edge']} edge on the plate top, E)", "part": FCN, "from": np.round(usb["p0"], 2).tolist(),
                "dir": np.round(usb["d"], 4).tolist(), "length_mm": USB_L, "tool": {"type": "box", "w": USB_W, "h": USB_H, "w_axis": np.round(w, 4).tolist()},
                "ignore": ["wires", DMP]})
    for hx, hy in holes:
        acc.append({"name": f"M3 key, FC damper screw ({hx:+.2f}, {hy:+.2f}) from the top", "part": FCN, "from": [hx, hy, st["pcb"][1] + 0.5],
                    "dir": [0, 0, 1], "length_mm": 150.0, "tool": {"type": "cylinder", "d": 4}, "ignore": ["wires"]})
    for bx, by in bolts:
        acc.append({"name": f"M3 key, belly-mount plate bolt ({bx:g}, {by:+g}) from the top", "part": BM, "from": [bx, by, PLATE_TOP + 0.5],
                    "dir": [0, 0, 1], "length_mm": 150.0, "tool": {"type": "cylinder", "d": 4}, "ignore": ["wires"]})
    fx, fy = flow["c"]; hx_ = FLOW_L / 2; pe = fx - hx_ if flow["ori"][1] == "-" else fx + hx_
    acc.append({"name": "flow SH1.0 plug out of its end (6 x 4, E)", "part": FLW, "from": [pe, fy, flow["zb"] + 3.25],
                "dir": [-1.0 if flow["ori"][1] == "-" else 1.0, 0, 0], "length_mm": 25.0, "tool": {"type": "box", "w": 6.0, "h": 4.0}, "ignore": ["wires"]})
    A["access"] = acc
    A["name"] = "Scout v3 with the FC on top and the belly mount v2 (concept)"
    A["note"] = ("main's Scout v3 assembly (v12, camera mount v1, tail bay PR 130) with the FC on the plate top on 4 silicone columns, the belly mount v2, "
                 "the flow re-placed, the tail block's foot tip trimmed to x -20 (requirement)")
    return A


CABLE_BOX = (-8.5, -8.1, 2.1, 2.5, 35.8, 36.2)   # the tail bay cables' 0.4 fit box, moved over the FC (main's at (-4, 1.4, 32) is inside the FC now)


def run_system_check(man):
    from system_check import run, report
    r = run(man); return r, report(r)


# ---------------------------------------------------------------- 8. pictures
COLS = {BM: "#2f7fd8", FCN: "#2e7d32", DMP: "#d84315", FLW: "#1b5e20", "frame plate (Bee35 carbon)": "#3a3a3a", TBN: "#8e6bbf",
        "electronics tail carrier (PETG)": "#b49ad6", "tie saddle (printed)": "#c77d2e", "XT60E1-M": "#c9a400", "receiver v3 (latch)": "#8899aa",
        "receiver lid (latch)": "#8899aa"}
SKIP_DRAW = ("cage shell tubes", "spokes, bay ties, core ties, camera guard", "wires", "tail bay cables (Scout electronics, E)")


def _poly(ax, m, i, j, flip, fc, ec="none", a=0.3, z=1, lw=0.15, rng=None):
    from matplotlib.collections import PolyCollection
    V = m.vertices[:, [i, j]] * [flip, 1]; Fc = m.faces
    if len(Fc) > 7000: Fc = Fc[rng.choice(len(Fc), 7000, replace=False)]
    ax.add_collection(PolyCollection(V[Fc], facecolor=fc, edgecolor=ec, alpha=a, linewidths=lw, zorder=z))


def _sect(ax, m, origin, normal, ij, fc, a=0.5, ec="k", z=3, lw=0.4, ls="-"):
    s = m.section(plane_origin=origin, plane_normal=normal)
    if s is None: return
    for e in s.discrete: ax.fill(e[:, ij[0]], e[:, ij[1]], facecolor=fc, edgecolor=ec, lw=lw, alpha=a, zorder=z, ls=ls)


def plate_outline(P):
    import shapely.geometry as sg
    s = P["frame plate (Bee35 carbon)"]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
    pg = sg.MultiPolygon([sg.Polygon(e[:, :2]) for e in s.discrete]).buffer(0)
    notch = sg.box(NOTCH[0], -NOTCH[1], 80.0, NOTCH[1]).buffer(0)
    return pg, pg.difference(notch)


def preview(P, M, info, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon as MPoly, Circle, FancyArrowPatch
    rng = np.random.default_rng(0)
    fig = plt.figure(figsize=(21, 19.5)); gs = fig.add_gridspec(3, 3, height_ratios=[1, 1.05, 0.95])
    views = [("side (from +y): x, z", (0, 2), 1), ("top: x, y", (0, 1), 1), ("rear (from -x): y, z", (1, 2), -1)]
    lims = {(0, 2): ((-80, 110), (-58, 72)), (0, 1): ((-80, 110), (-72, 72)), (1, 2): ((-75, 75), (-58, 72))}
    flow = info["flow"]; usb = info["usb"]; st = info["stack"]
    for k, (vt, (i, j), flip) in enumerate(views):
        ax = fig.add_subplot(gs[0, k]); ax.set_aspect("equal"); ax.set_title(f"FC on top + belly mount v2 in the Scout v3, {vt}", fontsize=10)
        for n, p in P.items():
            if n in SKIP_DRAW or n.startswith("prop"): continue
            c = COLS.get(n, "#4f6fcf" if p["spec"].get("swap") else "#a8b3bf")
            if n == BM: _poly(ax, p["mesh"], i, j, flip, c, "#174a85", 0.75, 6, 0.2, rng)
            elif n in (FCN, FLW, DMP): _poly(ax, p["mesh"], i, j, flip, c, "k", 0.85, 7, 0.3, rng)
            else: _poly(ax, p["mesh"], i, j, flip, c, a=0.16 if n not in COLS else 0.3, rng=rng)
        for a_, b_, r_ in rods_all(M): ax.plot([flip * a_[i], flip * b_[i]], [a_[j], b_[j]], color="#222", lw=0.8, zorder=3)
        if (i, j) == (0, 1):
            for mx, my in DISKS: ax.add_patch(Circle((mx, my), DISK_R, color="#e8c547", alpha=0.12, zorder=0))
        else:
            ax.axhspan(PROP_Z - 4, PROP_Z + 4, color="#e8c547", alpha=0.15, zorder=0); ax.text(lims[(i, j)][0][0] + 2, PROP_Z + 5, "prop sweep", fontsize=7, color="#a06800")
        q = usb["p0"] + 70 * usb["d"]
        ax.annotate("", xy=(flip * q[i], q[j]), xytext=(flip * usb["p0"][i], usb["p0"][j]), arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.6), zorder=10)
        ax.text(flip * q[i], q[j], " FC USB-C", color="#c62828", fontsize=8, zorder=10, clip_on=True)
        (x0, x1), (y0, y1) = lims[(i, j)]; ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up: section at y +12.75 through 2 dampers (x, z)
    ax = fig.add_subplot(gs[1, 0]); ax.set_aspect("equal"); yh = FC_HOLES
    ax.set_title(f"close-up: the FC on top, section at y {yh:+.2f} through 2 dampers (x, z)", fontsize=10)
    _sect(ax, info["tb_orig"], [0, 1.0, 0], [0, 1, 0], (0, 2), "none", 1.0, "#8e6bbf", 4, 0.9, "--")
    for n, c, a in (("frame plate (Bee35 carbon)", "#3a3a3a", 0.8), (TBN, "#8e6bbf", 0.5), ("receiver v3 (latch)", "#8899aa", 0.35), (BM, "#2f7fd8", 0.75),
                    (DMP, "#d84315", 0.9), (FCN, "#2e7d32", 0.8), ("XT60E1-M", "#c9a400", 0.6), ("receiver lid (latch)", "#8899aa", 0.35)):
        _sect(ax, P[n]["mesh"], [0, yh if n not in (TBN,) else 1.0, 0], [0, 1, 0], (0, 2), c, a)
    ax.add_patch(Rectangle((-FC_W / 2, st["pcb"][0]), FC_W, PCB_T, color="#1b5e20", zorder=8))
    ax.axhspan(PROP_Z - 4, PROP_Z + 4, color="#e8c547", alpha=0.2, zorder=0); ax.text(-58, PROP_Z + 1, "prop sweep (6.6 outside in plan)", fontsize=7, color="#a06800")
    md = info["damper"]
    ax.text(-58, 64, f"4 silicone columns Ø6 x {st['rubber'][1] - st['rubber'][0]:.1f}, {md['shore']}A (E), bonded M3 male-female", fontsize=7.5)
    ax.text(-58, 60, f"PCB clamped on the column tops; nothing rigid beside the rubber", fontsize=7.5)
    ax.text(-58, 56, "modes (E): " + ", ".join(f"{n} {f:.0f}" for f, n in md["modes"]) + " Hz", fontsize=7.5)
    ax.text(-58, 52, f"FC z {st['env'][0]:.1f}..{st['env'][1]:.1f}: plate {st['env'][0] - PLATE_TOP:.1f}, foot {info['fc_foot']:.1f} (dashed: the foot as drawn)", fontsize=7.5)
    ax.text(-58, 3.5, "the dampers' M3 studs into nuts bonded under the plate; the belly mount carries nothing of the FC", fontsize=7.5, color="#174a85")
    for hx in (-FC_HOLES, FC_HOLES): ax.add_patch(Rectangle((hx - 2.75, PLATE_Z - 2.4), 5.5, 2.4, color="#555", zorder=7))
    ax.set_xlim(-60, 60); ax.set_ylim(0, 68); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up: the plate top (x, y): FC, dampers, foot, notch, routes
    ax = fig.add_subplot(gs[1, 1]); ax.set_aspect("equal"); ax.set_title("close-up: the plate top, wiring routes (drawn, E)", fontsize=10)
    pg, pgn = plate_outline(P)
    for g in getattr(pgn, "geoms", [pgn]):
        ax.add_patch(MPoly(np.array(g.exterior.coords), closed=True, facecolor="#3a3a3a", alpha=0.25, edgecolor="#3a3a3a", zorder=1))
    for mx, my in DISKS: ax.add_patch(Circle((mx, my), DISK_R, color="#e8c547", alpha=0.12, zorder=0))
    _sect(ax, P[TBN]["mesh"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "#8e6bbf", 0.6)
    _sect(ax, info["tb_orig"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "none", 1.0, "#8e6bbf", 4, 0.9, "--")
    ax.add_patch(Rectangle((-FC_W / 2, -FC_W / 2), FC_W, FC_W, color="#2e7d32", alpha=0.75, zorder=5))
    for hx, hy in info["holes"]: ax.add_patch(Circle((hx, hy), D_DAMP / 2, color="#d84315", zorder=6))
    for bx, by in info["bolts"]: ax.plot(bx, by, "s", ms=6, mfc="#c62828", mec="k", zorder=9)
    for mx, my in DISKS: ax.add_patch(Circle((mx, my), 12.4, color="#777", alpha=0.6, zorder=2))
    for (sx, sy) in ((1, 1), (1, -1), (-1, 1), (-1, -1)):                  # motor leads: FC corner to the motor along the arm top
        ax.plot([sx * 16, sx * 42.0], [sy * 16, sy * 42.0], color="#6d4c41", lw=2.2, zorder=7)
    for cx, cy in XT60_CUPS: ax.plot([cx, cx, 22.0], [cy, cy, cy], color="#b71c1c" if cy > 0 else "#212121", lw=3.0, zorder=7)
    ax.plot([24, 46, 46, 30, 18], [-30.5, -30.5, -6.5, -6.5, -6.5], color="#1b5e20", lw=1.4, ls="--", zorder=7)   # flow lead (under the plate dashed)
    for yy in (-11.0, 11.0): ax.plot([-18.0, -40.0, -64.0], [yy, yy, yy * 1.0], color="#5e35b1", lw=1.6, zorder=7)
    q = usb["p0"] + 40 * usb["d"]
    ax.annotate("", xy=tuple(q[:2]), xytext=tuple(usb["p0"][:2]), arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.8), zorder=10)
    ax.text(q[0] + 1, q[1], "USB-C", color="#c62828", fontsize=7.5, zorder=10)
    ax.text(47.5, 9.5, "lead\nnotch", fontsize=7, ha="center", zorder=10); ax.text(0, 0, "FC\n(on the\nAIO holes)", color="w", fontsize=8, ha="center", va="center", zorder=10)
    ax.text(-42, 14, "Radxa UART, XR2,\nUBEC in (to the bay)", fontsize=7, color="#5e35b1", zorder=10)
    ax.text(26, -28, "flow lead (under the plate,\nup through the notch)", fontsize=6.5, color="#1b5e20", zorder=10)
    ax.text(-30, 3, "foot\n(tip to x -20)", fontsize=7, ha="center", color="#5e35b1", zorder=10)
    for bx, by in info["bolts"]: ax.text(bx, by + (4 if by > 0 else -6.5), "side bolt", fontsize=6.5, ha="center", color="#c62828", zorder=10)
    ax.set_xlim(-70, 70); ax.set_ylim(-60, 60); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up: the belly mount slice at z 14 (top view)
    ax = fig.add_subplot(gs[1, 2]); ax.set_aspect("equal"); ax.set_title("close-up: belly mount v2, slice at z 14 (top view)", fontsize=10)
    _sect(ax, P["receiver v3 (latch)"]["mesh"], [0, 0, 9.0], [0, 0, 1], (0, 1), "#8899aa", 0.25, "#667", 1)
    for n, zc_ in ((TBN, 14.0), ("XT60E1-M", 14.0), ("tie saddle (printed)", 14.0), (BM, 14.0)):
        _sect(ax, P[n]["mesh"], [0, 0, zc_], [0, 0, 1], (0, 1), COLS.get(n, "#a8b3bf"), 0.85 if n == BM else 0.45, "k", 5 if n == BM else 3)
    for x_, y_ in REC_BOLTS: ax.text(x_, y_ - 8.0 if y_ < 0 else y_ + 6.0, "rcv M3", fontsize=6.5, ha="center", zorder=9)
    for bx, by in info["bolts"]: ax.text(bx, by + (6.5 if by > 0 else -8.5), "plate M3\n(side hole)", fontsize=6.5, ha="center", color="#c62828", zorder=9)
    for hx, hy in info["holes"]: ax.add_patch(Circle((hx, hy), 2.75, color="#555", zorder=6)); ax.text(hx, hy - 5.5, "bonded nut", fontsize=6, ha="center", zorder=9)
    ax.text(-28, 0, "tail\nface", fontsize=7, ha="center", zorder=9); ax.text(48, 0, "XT60", fontsize=7, ha="center", zorder=9)
    ax.set_xlim(-50, 60); ax.set_ylim(-55, 45); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # bottom row: the bipod slice, the damper chart, the numbers
    ax = fig.add_subplot(gs[2, 0]); ax.set_aspect("equal"); py = flow["post_y"]
    ax.set_title(f"close-up: the flow bipod, slice at y {py:+.1f} and at the flow (x, z)", fontsize=10)
    for n in (BM, FLW, "receiver v3 (latch)", "tie saddle (printed)", "frame plate (Bee35 carbon)", "cartridge v6 (latch)"):
        for yy in (py, flow["c"][1]): _sect(ax, P[n]["mesh"], [0, yy, 0], [0, 1, 0], (0, 2), COLS.get(n, "#a8b3bf"), 0.6 if n in OURS else 0.25, "k", 5 if n in OURS else 3)
    t_ = math.tan(math.radians(FLOW_HALF)); zl = flow["zb"]; fx = flow["c"][0]
    for wx in (fx - FLOW_WIN, fx + FLOW_WIN):
        for s_ in (-1, 1): ax.plot([wx, wx + s_ * (zl - PLANE + 8) * t_], [zl, PLANE - 8], color="#43a047", lw=0.9)
    ax.axhline(PLANE, color="#555", lw=0.6, ls=":"); ax.axhline(PLANE + BAND, color="#c62828", lw=0.7, ls="--"); ax.text(-48, PLANE + BAND + 1, "funnel band top", fontsize=7, color="#c62828")
    bp = info["bipod"]
    ax.text(-48, 24, f"bipod 6 x 6 PETG, {bp['strut_L']} long; flow CG {abs(bp['offset_y']):.1f} off the strut plane (the twist)", fontsize=7)
    ax.set_xlim(-50, 60); ax.set_ylim(-62, 30); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    ax = fig.add_subplot(gs[2, 1]); ax.set_title("the soft mount: first lateral (solid) and vertical (dashed) mode vs rubber height (E)", fontsize=10)
    for sh, c in ((30, "#2e7d32"), (40, "#1565c0"), (50, "#6a1b9a")):
        rr = [r for r in info["damper_rows"] if r["shore"] == sh]
        ax.plot([r["h"] for r in rr], [r["lateral"] for r in rr], color=c, lw=1.6, label=f"{sh}A lateral")
        ax.plot([r["h"] for r in rr], [r["vertical"] for r in rr], color=c, lw=1.2, ls="--", label=f"{sh}A vertical")
    ax.axhspan(100, 175, color="#43a047", alpha=0.08); ax.axhline(100, color="#c62828", lw=0.8); ax.text(4.25, 102, "gate 100 Hz", fontsize=7, color="#c62828")
    ax.axhspan(500, 670, color="#e8c547", alpha=0.25); ax.text(4.25, 585, "blade pass 500..670 (E)", fontsize=7)
    ax.axhspan(250, 335, color="#ff8a65", alpha=0.12); ax.text(4.25, 290, "rotor 1P ~250..335 (E)", fontsize=7)
    ax.plot([md["h"]], [md["lateral"]], "o", color="k", ms=6); ax.text(md["h"] + 0.1, md["lateral"] - 18, f"pick {md['shore']}A, {md['h']}", fontsize=7.5)
    ax.set_xlabel("rubber height (mm)", fontsize=8); ax.set_ylabel("Hz", fontsize=8); ax.set_ylim(60, 700); ax.legend(fontsize=7, ncol=3, loc="upper right")
    ax.grid(alpha=0.2); ax.tick_params(labelsize=7)
    ax = fig.add_subplot(gs[2, 2]); ax.axis("off")
    ax.text(0.0, 1.0, "\n".join(info["notes"]), va="top", ha="left", fontsize=8.2, family="monospace", transform=ax.transAxes)
    fig.text(0.01, 0.004, "blue: belly mount v2 (one PETG print); green: FC (on the plate top) and flow; orange-red: the 4 silicone columns; violet: the tail block "
             "(its foot tip trimmed to x -20, dashed: as drawn) and carrier; orange: tie saddle; grey: receiver set; dark blue: swap set; black: cage rods; "
             "yellow: swept props; red arrow: FC USB-C path", fontsize=8)
    fig.tight_layout(rect=(0, 0.012, 1, 1)); fig.savefig(path, dpi=100); plt.close(fig)


def preview_alternatives(P, M, info, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, Polygon as MPoly
    import shapely.geometry as sg, shapely.affinity as sa
    fig, axs = plt.subplots(2, 4, figsize=(23, 13.5))
    pg, pgn = plate_outline(P)
    def base(ax, tb):
        for g in getattr(pgn, "geoms", [pgn]):
            ax.add_patch(MPoly(np.array(g.exterior.coords), closed=True, facecolor="#3a3a3a", alpha=0.18, edgecolor="#3a3a3a", zorder=1))
        for mx, my in DISKS: ax.add_patch(Circle((mx, my), DISK_R, color="#e8c547", alpha=0.15, zorder=0))
        _sect(ax, tb, [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "#8e6bbf", 0.6)
        ax.set_xlim(-65, 70); ax.set_ylim(-62, 62); ax.set_aspect("equal"); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    def note(ax, lines, y0=-0.08):
        ax.text(0.0, y0, "\n".join(lines), transform=ax.transAxes, fontsize=7.8, va="top", ha="left",
                bbox=dict(facecolor="white", alpha=0.9, edgecolor="#999", lw=0.5), zorder=20)
    for k, (key, title) in enumerate(info["fc_options"].items()):
        o = title; ax = axs[0, k]
        base(ax, P[TBN]["mesh"] if o["trim"] else info["tb_orig"])
        sq = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), o["yaw"], origin=(0, 0)), o["x"], 0)
        ax.add_patch(MPoly(np.array(sq.exterior.coords), closed=True, color="#2e7d32", alpha=0.8, zorder=5))
        for hx, hy in holes_xy(o["x"], 0, o["yaw"]): ax.plot(hx, hy, "o", ms=5, mfc="w" if not o["aio"] else "#d84315", mec="k", zorder=9)
        if not o["aio"]:
            for sx in (-1, 1):
                for sy in (-1, 1): ax.plot(sx * FC_HOLES, sy * FC_HOLES, "x", color="#c62828", ms=5, zorder=9)
        ax.set_title(f"{key}{' (picked)' if o['picked'] else ''}: FC at x {o['x']:+.1f}, yaw {o['yaw']:.0f}\n{o['what']}", fontsize=10)
        note(ax, o["lines"])
    for k, (v, d) in enumerate(info["bolt_variants"].items()):
        ax = axs[1, k]
        _sect(ax, P["receiver v3 (latch)"]["mesh"], [0, 0, 9.0], [0, 0, 1], (0, 1), "#8899aa", 0.25, "#667", 1)
        _sect(ax, d["mesh"], [0, 0, 14.0], [0, 0, 1], (0, 1), "#2f7fd8", 0.8, "k", 4)
        for bx, by in VARIANTS[v]["bolts"]: ax.plot(bx, by, "s", ms=6, mfc="#c62828" if (bx, by) in VARIANTS[v]["new"] else "#ffb300", mec="k", zorder=9)
        ax.set_xlim(-50, 58); ax.set_ylim(-55, 42); ax.set_aspect("equal"); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
        ax.set_title(f"belly {v}{' (picked)' if d['picked'] else ''}: {VARIANTS[v]['label']}", fontsize=9.5)
        note(ax, d["lines"])
    ax = axs[1, 3]; ax.axis("off")
    note(ax, info["alt_text"], 1.0)
    fig.suptitle("Scout v3 FC on top + belly mount v2: the options weighed, from the same system model (C on E). Top: where the FC sits "
                 "(red x: the AIO holes, o: its 4 holes); bottom: how the belly mount bolts to the plate (yellow: the Bee35's own holes, red: new)", fontsize=11)
    fig.subplots_adjust(left=0.03, right=0.99, top=0.92, bottom=0.12, wspace=0.16, hspace=0.42); fig.savefig(path, dpi=95); plt.close(fig)


# ---------------------------------------------------------------- 9. the pass
M_FC = 10.0 + 4 * 0.3 + 4 * 0.15          # board 10 (P) + 4 M3 x 5 button screws + 4 column top discs (E)
CG_DZ = PCB_T / 2                         # the board's CG at the PCB middle, over the column tops (E: components both sides)
DMP_G = 4 * 0.4 + 4 * 0.2                 # 4 silicone columns with their studs and discs + 4 M3 nuts bonded under the plate (E)
CRASH_TARGET = 1.0 / 1.5                  # DEFAULT: crash worst <= 1 / 1.5 (the L breaking order against an index-1.0 node; node indices
                                          # are not computed, and the 3.0 rods fail at 35 g: rods are the fuse, Spark 27 Oct)
WIRES = dict(box=[[35.8, -0.2, 24.0], [36.2, 0.2, 24.4]], cg=[5.0, 0.0, 25.0],
             note="20 g E: motor leads on the arm tops, battery leads from the notch to the FC's nose edge, signal leads; CG (5, 0, 25) E; "
                  "for the fit a 0.4 box in the battery-lead lane on the plate top, no allowance")


def ser(x):
    if isinstance(x, (np.floating, np.integer)): return x.item()
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, (np.bool_,)): return bool(x)
    if isinstance(x, trimesh.Trimesh): return "<mesh>"
    return str(x)


def run():
    t0 = time.time(); os.makedirs(RUN, exist_ok=True)
    say("Scout v3 FC on top + belly mount v2: system pass on main's assembly (layout v12, catch 5.0 forward, camera mount v1, tail bay PR 130). "
        "Spark 27 Oct: the FC on top of the plate; every rod 3.0")
    M = load_model(); P = M["P"]
    tb = P[TBN]["mesh"]; tbt = trimmed_tail_block(tb)
    cut = trimesh.boolean.intersection([tb, box([FOOT_TRIM, -40, PLATE_Z - 0.5], [20, 40, 60])], engine="manifold")
    tb_g = P[TBN]["g"] * tbt.volume / tb.volume; dg = P[TBN]["g"] - tb_g
    tb_c = (P[TBN]["g"] * P[TBN]["c"] - dg * np.asarray(cut.center_mass)) / tb_g
    say(f"tail block requirement (its owner's piece pass): foot tip x {FOOT_X:+.1f} -> {FOOT_TRIM:+.1f}: {cut.volume:.0f} mm3 off, "
        f"{P[TBN]['g']:.2f} -> {tb_g:.2f} g (E, pro rata), CG {np.round(tb_c, 2).tolist()}")
    # --- 1. the soft mount (sets the stack)
    drows, dp = damper_sweep(M_FC, CG_DZ); st = fc_stack(dp["h"])
    for sh in (30, 40, 50):
        rr = [r for r in drows if r["shore"] == sh and r["lateral"] >= 110.0]
        if rr: r = rr[-1]; say(f"soft mount {sh}A (G {r['G']} MPa, Gent E): lateral >= 110 up to h {r['h']}: lateral {r['lateral']}, vertical {r['vertical']} Hz")
    say(f"soft mount pick ({dp['shore']}A, G {dp['G']} MPa, Ø{D_DAMP:g} x {dp['h']} rubber, ends {END_T} each; board {M_FC:.1f} g, CG {CG_DZ} over the column tops): "
        f"k lateral {dp['k']['lat']:.2f}, vertical {dp['k']['v']:.2f} N/mm per column; modes " + ", ".join(f"{n} {f:.0f}" for f, n in dp["modes"]) + " Hz (E)")
    say(f"FC stack: plate top {PLATE_TOP}, rubber {st['rubber']}, PCB {st['pcb']}, envelope {st['env']} (C on E)")
    crash_dx = 35 * G * M_FC / 1000 / 4 / dp["k"]["lat"]; crash_dz = 35 * G * M_FC / 1000 / 4 / dp["k"]["v"]
    say(f"soft mount in a 35 g crash: {crash_dx:.2f} sideways, {crash_dz:.2f} vertical (E); shear stress {35 * G * M_FC / 1000 / 4 / (math.pi * 9):.3f} MPa")
    # --- 2. the FC pose
    ps = fc_pose_search(M, st, tb, tbt)
    r0 = next(r for r in ps["rows"] if r["x"] == 0.0 and r["yaw"] == 0.0)
    say(f"FC pose picked (B): x 0, yaw 0 on the plate's AIO holes: plan {r0['col']:.2f} outside the swept disks (column), {r0['contracted']:.2f} (contracted), "
        f"tail block {r0['foot_trim']:.2f} with the tip trimmed ({r0['foot']:.2f} as drawn), rods {r0['rods']:.1f}")
    roof = roof_limit(M, 0.0, 0.0, 0.0)
    say(f"roof over the FC's plan (+3): " + ("no cage member" if not np.isfinite(roof[0]) else f"lowest member at z {roof[0]:.1f} {roof[1]}")
        + f"; the board top {st['env'][1]:.1f} is {PROP_Z - 4 - st['env'][1]:.1f} under the prop sweep and {r0['col']:.1f} outside it in plan")
    holes = holes_xy(0.0, 0.0, 0.0)
    P[TBN] = dict(P[TBN], mesh=tbt, g=tb_g, c=tb_c)
    P[FCN] = dict(mesh=fc_mesh(0.0, 0.0, 0.0, st), g=10.0, c=np.array([0.0, 0.0, (st["pcb"][0] + st["pcb"][1]) / 2]), role="bought", tag="P", spec={})
    P[DMP] = dict(mesh=dampers_mesh(holes, st), g=DMP_G, c=np.array([0.0, 0.0, (PLATE_TOP + st["pcb"][0]) / 2]), role="bought", tag="E", spec={})
    P["wires"] = dict(mesh=box(*WIRES["box"]), g=20.0, c=np.array(WIRES["cg"]), role="structure", tag="E", spec={})
    cb = CABLE_BOX; P["tail bay cables (Scout electronics, E)"] = dict(P["tail bay cables (Scout electronics, E)"], mesh=box([cb[0], cb[2], cb[4]], [cb[1], cb[3], cb[5]]))
    # --- 3. USB-C
    usb_ok, usb_all = usb_search2(M, (0.0, 0.0), st); u = usb_ok[0]
    usb = dict(edge=u[1], s=u[2], az=u[3], el=u[4], d=u[5], p0=u[6], n_clear=len(usb_all), n_clear_sides=len(usb_ok))
    say(f"USB-C picked: the {usb['edge']} edge at {usb['s']:+.0f} along it, azimuth {usb['az']:+.0f}, elevation {usb['el']:+.0f}: 250 clear (C); "
        f"the battery pads keep the nose edge (DEFAULT, toward the notch)")
    # --- 4. the flow (v1's search, seeded)
    O = v1.occupancy(M, skip=(FLW,), env=True)
    Oc = v1.occupancy(M, skip=(FLW, "cage shell nodes", "camera", "camera mount (printed)", "camera mount stop collar +y", "camera mount stop collar -y",
                               FCN, DMP, "wires"), env=True, rods=False)
    cache = os.environ.get("BELLY_CACHE")                              # development only: a saved search (the run itself never sets it)
    if cache and os.path.exists(cache):
        d_ = json.load(open(cache)); scored = [tuple(r_) for r_ in d_["scored"]]; v1.FEAS[:] = [None] * d_["n_feas"]
    else:
        scored = v1.flow_search(M, O, Oc)
        if cache: json.dump(dict(scored=[[ser(x) if not isinstance(x, (tuple, list, str, bool, float, int)) else x for x in r_] for r_ in scored],
                                 n_feas=len(v1.FEAS)), open(cache, "w"), default=ser)
    pref = [r for r in scored if r[-1] == "x+"] + [r for r in scored if r[-1] != "x+"]   # plug to the nose: its lead runs forward to the notch
    flow = v1.pick_flow(pref, 1)
    say(f"flow picked: centre ({flow['c'][0]:.0f}, {flow['c'][1]:+.0f}), z {flow['zb']:.0f}..{flow['zb'] + FLOW_T:.0f}, long side along x, plug end {flow['ori']} "
        f"(toward the nose: the lead goes up the front strut and the notch), strut search {flow['strut_L']:.1f}, "
        f"{'inside the swap column below the funnel band' if flow['in_swap_column'] else 'above the funnel band'}; {len(scored)} poses with a clear strut")
    P[FLW] = dict(mesh=flow_mesh(flow), g=1.5, c=np.array([flow["c"][0], flow["c"][1], flow["zb"] + FLOW_T / 2]), role="bought", tag="P", spec={})
    # --- 5. the belly mount: the variants, built and loaded
    from solid_check import solid_check
    xs = np.round(np.arange(-43.5, 53.01, 1.0), 2); ys = [-15.0, -8.0, 0.0, 8.0, 15.0]
    hx_, hy_ = FLOW_L / 2, FLOW_W / 2
    cr_box = box([flow["c"][0] - hx_ - CRADLE, flow["c"][1] - hy_ - CRADLE, flow["zb"] - CRADLE], [flow["c"][0] + hx_ + CRADLE, flow["c"][1] + hy_ + CRADLE, flow["zb"] + FLOW_T + CRADLE])
    VR = {}
    for key, (v, kw) in {"VA": ("VA", {}), "VA, damper pods": ("VA", dict(damper="pod")), "VA, damper rungs": ("VA", dict(damper="rung")),
                         "VA + front rung": ("VA", dict(front_rung=True)), "VA, rails 5.5": ("VA", dict(rail_out=25.5)),
                         "VB": ("VB", {}), "VC": ("VC", {})}.items():
        bm = belly_v2(flow, v, holes, **kw); ok, sc = solid_check(bm); g_, gs_, _ = printed_g(bm)
        cradle_g = printed_g(trimesh.boolean.intersection([bm, cr_box], engine="manifold"))[0]
        sx_, sy_ = frame_sections(bm, xs, ys)
        L = ladder_loads(P, bm, g_, flow, 1.5 + cradle_g, VARIANTS[v]["bolts"], sx_, sy_)
        cw = max(L["by"]["crash"][f][0] for f in ("rec_idx", "plate_idx", "frame_x_idx", "frame_y_idx"))
        fw = max(max(L["by"][k][f][0] for f in ("rec_idx", "plate_idx", "frame_x_idx", "frame_y_idx")) for k in ("flight", "landing"))
        zmin = min((v_["Z"], k) for k, v_ in sx_.items() if v_ and v_["Z"] > 0)
        VR[key] = dict(variant=v, kw=kw, mesh=bm, ok=bool(ok), sc=sc, g=round(g_, 2), g_solid=round(gs_, 2), cradle_g=round(cradle_g, 2), L=L, crash=round(cw, 3),
                       fl=round(fw, 3), new=len(VARIANTS[v]["new"]), zmin=zmin, secs_x=sx_, secs_y=sy_)
        b = L["by"]["crash"]
        say(f"belly {key}: {g_:.1f} g as printed (solid {gs_:.1f}), solid_check {'PASS' if ok else 'FAIL'} (eroded bodies {sc.get('bodies_eroded')}, necks {len(sc.get('necks', []))}); "
            f"new plate holes {len(VARIANTS[v]['new'])}; crash worst {cw:.3f}: receiver inserts {b['rec_idx'][0]:.3f}, plate inserts {b['plate_idx'][0]:.3f} "
            f"(prying {b['pry'][0]:.2f} x), frame about y {b['frame_x_idx'][0]:.3f} at x {b['frame_x_idx'][2]} ({b['frame_x_idx'][1]}), about x {b['frame_y_idx'][0]:.3f} "
            f"at y {b['frame_y_idx'][2]}; flight / landing worst {fw:.3f}; weakest x cut Z {zmin[0]:.0f} mm3 at x {zmin[1]}")
    okv = [k for k, r in VR.items() if r["ok"] and r["crash"] <= CRASH_TARGET and r["fl"] <= 0.5]
    pick = min(okv, key=lambda k: (VR[k]["new"], VR[k]["g"]))
    say(f"belly pick (crash <= {CRASH_TARGET:.2f} DEFAULT, flight / landing <= 0.5, solid_check PASS; then the fewest new plate holes, then the lightest): {pick}")
    R = VR[pick]; bm = R["mesh"]; bm_g = R["g"]; bm_c = np.asarray(bm.center_mass, float); bolts = VARIANTS[R["variant"]]["bolts"]
    ok, sc = R["ok"], R["sc"]
    P[BM] = dict(mesh=bm, g=bm_g, c=bm_c, role="structure", tag="E", spec={})
    flow_g = 1.5 + R["cradle_g"]
    # --- the flow bipod as a frame
    bp = bipod_frame(flow, flow_g)
    say(f"flow bipod (3D frame, E): struts {bp['strut_L']} long, the flow + cradle ({flow_g:.2f} g) {abs(bp['offset_y']):.1f} off the strut plane: crash worst "
        f"{bp['by_kind']['crash'][0]:.4f} ({bp['by_kind']['crash'][1]}), flight {bp['by_kind']['flight'][0]:.4f}, landing {bp['by_kind']['landing'][0]:.4f}; "
        f"modes {bp['modes_Hz']} Hz; with the CG in the strut plane {bp['worst_no_offset']:.4f}, {bp['modes_no_offset_Hz']} Hz: what the twist costs")
    # --- 6. fit and system_check
    rods = rods_all(M)
    ft = fit_full(P, rods)
    say("fit, full pairwise table (ours against every part; >= 2.0 or a declared face contact <= 0.05 mm3; '>' = box gap, a lower bound):")
    for r in ft: say(f"   {r['a']} / {r['b']}: overlap {r['overlap_mm3']:.3f} mm3, gap {'>' if r['lower_bound'] else ''}{r['gap_mm']:.2f}, {r['kind']}: {'PASS' if r['pass_'] else 'FAIL'}")
    files = {BM: write_stl(bm, "belly_mount_v2__petg_.stl"), FCN: write_stl(P[FCN]["mesh"], "fc_on_top.stl"), DMP: write_stl(P[DMP]["mesh"], "fc_dampers.stl"),
             FLW: write_stl(P[FLW]["mesh"], "flow_mtf-02p.stl"), TBN: write_stl(tbt, "tail_block_foot_trimmed__requirement.stl")}
    man = manifest(M, P, files, usb, flow, holes, bolts, st, WIRES)
    json.dump(man, open(os.path.join(RUN, "manifest.json"), "w"), indent=1, default=ser)
    r, txt = run_system_check(man)
    open(os.path.join(RUN, "system_check.txt"), "w").write(txt + "\n")
    say(r["summary"])
    acc_rows = [(p["name"], p["pass_"], round(p.get("overlap_mm3", 0.0), 2), [b["part"] for b in p.get("blocked_by", [])]) for p in r["access"]["paths"]]
    for a_ in acc_rows: say(f"   access {a_[0]}: {'clear' if a_[1] else 'BLOCKED ' + str(a_[3]) + ' ' + str(a_[2]) + ' mm3'}")
    mc = r["mass_cg"]; bal = r["balance"]
    # --- the ESC in free air: the board's 4 sides and top, 3 mm out, against every part and rod
    Og, lo_, pt_ = occ(M, [-60, -60, 15], [60, 60, 60], 0.5, skip=(FCN, DMP, "wires"))
    def free(pts):
        ij = np.floor((pts - lo_) / pt_).astype(int); return float(1.0 - Og[ij[:, 0], ij[:, 1], ij[:, 2]].mean())
    s_ = np.linspace(-FC_W / 2, FC_W / 2, 73); zz_ = np.linspace(PLATE_TOP + 0.25, st["env"][1], 23); S_, Z_ = np.meshgrid(s_, zz_)
    sides = np.vstack([np.c_[S_.ravel(), np.full(S_.size, sy * (FC_W / 2 + 3)), Z_.ravel()] for sy in (-1, 1)]
                      + [np.c_[np.full(S_.size, sx * (FC_W / 2 + 3)), S_.ravel(), Z_.ravel()] for sx in (-1, 1)])
    XX, YY = np.meshgrid(s_, s_); topp = np.c_[XX.ravel(), YY.ravel(), np.full(XX.size, st["env"][1] + 3)]
    side_open, top_open = free(sides), free(topp)
    side_A = 4 * FC_W * (st["env"][1] - PLATE_TOP); top_A = FC_W * FC_W
    say(f"ESC (the AIO) in free air: its 4 sides {100 * side_open:.0f} % open 3 mm out ({side_open * side_A:.0f} of {side_A:.0f} mm2, plate top to the board top), "
        f"its top {100 * top_open:.0f} % open 3 mm up ({top_open * top_A:.0f} mm2); under it {st['env'][0] - PLATE_TOP:.1f} of air over the plate; "
        f"{r0['col']:.1f} outside the swept disks in plan (v1: 641 mm2 of side vents, the plate 1.0 above it and the receiver below)")
    # --- plate holes
    import shapely.geometry as sg
    pg, pgn = plate_outline(P)
    edge_x = max(x for x, y in pg.exterior.coords if abs(y) < 8) if hasattr(pg, "exterior") else 50.3
    say(f"plate: the FC's 4 dampers on the Bee35's AIO holes (±12.75, ±12.75) (P, as built; M3 studs: CHECK the hole size, open to Ø3.4 if smaller); "
        f"the belly mount on the 2 side holes {SIDE_HOLES} ({pick}); a battery-lead notch x {NOTCH[0]}..{edge_x:.1f} (the nose edge), |y| <= {NOTCH[1]} over the XT60's "
        f"cups (it takes the empty nose hole (45, 0)); v1's 4 new holes (-25, ±13), (30, ±13): " + ("not needed" if not VARIANTS[R['variant']]["new"] else f"{VARIANTS[R['variant']]['new']}"))
    # --- PX4 offsets (FRD from the CG, m)
    cg = np.array(mc["cg"], float)
    imu = np.array([0.0, 0.0, (st["pcb"][0] + st["pcb"][1]) / 2])
    sgn = 1.0 if flow["ori"][1] == "-" else -1.0                      # x-: the lens at the nose window (as built); x+: the board turned 180, lens aft
    lens = np.array([flow["c"][0] + sgn * FLOW_WIN, flow["c"][1], flow["zb"]]); tof = np.array([flow["c"][0] - sgn * FLOW_WIN, flow["c"][1], flow["zb"]])
    frd = lambda p: np.round([(p[0] - cg[0]) / 1000, -(p[1] - cg[1]) / 1000, -(p[2] - cg[2]) / 1000], 4).tolist()
    px4 = dict(cg=np.round(cg, 2).tolist(), EKF2_IMU_POS=frd(imu), EKF2_OF_POS=frd(lens), EKF2_RNG_POS=frd(tof),
               SENS_FLOW_ROT="4 (yaw 180: plug to the nose, the lens aft; v1 as built 0), CHECK on the bench",
               SENS_BOARD_ROT=f"from the USB-C edge ({usb['edge']}) and the battery pads at the nose; set at the bench",
               note="IMU at the board centre, PCB middle (E: the chip's place is not published)")
    say(f"PX4 (FRD from the CG {px4['cg']}, m): EKF2_IMU_POS {px4['EKF2_IMU_POS']}, EKF2_OF_POS {px4['EKF2_OF_POS']}, EKF2_RNG_POS {px4['EKF2_RNG_POS']}; "
        f"SENS_FLOW_ROT {px4['SENS_FLOW_ROT']}")
    masses = {n: dict(g=round(float(P[n]["g"]), 2), cg=np.round(P[n]["c"], 2).tolist()) for n in (BM, FCN, DMP, "wires", FLW, TBN)}
    say("masses and CGs for the cage session (E): " + "; ".join(f"{n} {v['g']} g at {v['cg']}" for n, v in masses.items()))
    cin = bal["cases"]["all"]["horizontal_mm"] if bal.get("applies") else None
    cout = bal["cases"].get("no_swap", {}).get("horizontal_mm") if bal.get("applies") else None
    # --- the alternatives' numbers
    rows = {(rw["x"], rw["yaw"]): rw for rw in ps["rows"]}
    def opt(x, yaw, trim, aio, picked, what):
        rw = rows[(x, yaw)]
        return dict(x=x, yaw=yaw, trim=trim, aio=aio, picked=picked, what=what, col=rw["col"], con=rw["contracted"], gap=rw["foot_trim"] if trim else rw["foot"])
    A_, C_, D_ = ps["as_drawn_yaw0"], ps["as_drawn_yaw45"], ps["trimmed"]
    fco = {"A": opt(A_["x"], 0.0, False, False, False, "the foot as drawn, 4 new holes"),
           "B": opt(0.0, 0.0, True, True, True, "on the AIO holes, the foot tip to x -20"),
           "C": opt(C_["x"], 45.0, False, False, False, "turned 45, the foot as drawn, 4 new holes"),
           "D": opt(D_["x"], D_["yaw"], True, False, False, "turned 45, the foot trimmed, 4 new holes")}
    for k, o in fco.items():
        o["lines"] = [f"plan {o['col']:.2f} outside the swept disks (column, rule 8 >= 3: {'PASS' if o['col'] >= WASH else 'FAIL'});",
                      f"{o['con']:.2f} outside the contracted slipstream; tail block {o['gap']:.2f}",
                      f"IMU {o['x']:+.1f} in x from the thrust centre; holes: {'the AIO ones (none new)' if o['aio'] else '4 new M3 in the carbon'}",
                      ("tail block: foot tip x -14 -> -20 (its piece pass)" if o["trim"] else "tail block unchanged")]
    fco["A"]["lines"].append("nearest new hole 6.0 from an AIO hole (2.6 of carbon between)")
    fco["C"]["lines"].append("damper bosses on the centre line (x -4.5, 31.5): a spine in the belly mount")
    bvar = {}
    for k in ("VA", "VB", "VC"):
        q = VR[k]; b = q["L"]["by"]
        bvar[k] = dict(mesh=q["mesh"], picked=(k == pick), lines=[
            f"{q['g']:.1f} g as printed (E), new plate holes {q['new']}, solid_check {'PASS' if q['ok'] else 'FAIL'}",
            f"crash worst {q['crash']:.3f} (target <= {CRASH_TARGET:.2f}): inserts rcv {b['crash']['rec_idx'][0]:.3f} / plate {b['crash']['plate_idx'][0]:.3f},",
            f"frame {b['crash']['frame_x_idx'][0]:.3f} at x {b['crash']['frame_x_idx'][2]}; prying up to {b['crash']['pry'][0]:.2f} x",
            f"flight / landing worst {q['fl']:.3f} (<= 0.5)"])
    alt_text = ["Also weighed (same model):",
                f"- the dampers into inserts here instead of nuts bonded under the plate:",
                f"  pods on the rails {VR['VA, damper pods']['g']:.1f} g (crash {VR['VA, damper pods']['crash']:.3f}), rungs {VR['VA, damper rungs']['g']:.1f} g",
                f"  (crash {VR['VA, damper rungs']['crash']:.3f}) vs {VR['VA']['g']:.1f} g: the FC would hang on this piece",
                f"- rails 5.5 wide (|y| 20..25.5): {VR['VA, rails 5.5']['g']:.1f} g, crash {VR['VA, rails 5.5']['crash']:.3f}",
                f"- VA + a front rung behind the XT60: {VR['VA + front rung']['g']:.1f} g, crash {VR['VA + front rung']['crash']:.3f}",
                "  (the receiver, bolted at its 4 corners, already closes the box)",
                "- nylocs under the plate instead of bonded nuts: a damper change",
                "  then needs the receiver and the belly mount off",
                f"- a printed FC seat on the plate: +3..4 g E, a part in series with the rubber, no gain",
                f"- softer columns (30A x 6.0): lateral {next(r['lateral'] for r in drows if r['shore'] == 30 and r['h'] == 6.0)} Hz < the 100 gate",
                f"- stiffer (40A x 5.0): lateral {next(r['lateral'] for r in drows if r['shore'] == 40 and r['h'] == 5.0)}, vertical {next(r['vertical'] for r in drows if r['shore'] == 40 and r['h'] == 5.0)} Hz",
                f"- flow CG in the strut plane (no twist): {bp['worst_no_offset']:.4f} vs {bp['worst'][0]:.4f}; no pose: the receiver",
                "  wall and the saddle rail leave a 2.8 corridor"]
    notes = [f"FC on top (Spark 27 Oct): x 0 on the AIO holes, z {st['env'][0]:.1f}..{st['env'][1]:.1f}",
             f"  plan {r0['col']:.2f} outside the swept disks; roof: {'no member over it' if not np.isfinite(roof[0]) else f'z {roof[0]:.1f}'}",
             f"  4 columns {dp['shore']}A Ø6 x {dp['h']}: lateral {dp['lateral']:.0f}, vertical {dp['vertical']:.0f} Hz",
             f"  USB-C: {usb['edge']} edge, {usb['n_clear']} clear exits",
             f"  ESC sides {100 * side_open:.0f} % / top {100 * top_open:.0f} % open",
             f"belly mount v2 ({pick}): {bm_g:.1f} g E (v1 25.0)",
             f"  plate bolts: the 2 side holes, 0 new; crash {R['crash']:.3f}",
             f"  flight / landing {R['fl']:.3f}; solid_check {'PASS' if ok else 'FAIL'}",
             f"flow ({flow['c'][0]:.0f}, {flow['c'][1]:+.0f}), bipod {bp['modes_Hz'][0]:.0f} Hz, crash {bp['by_kind']['crash'][0]:.3f}",
             f"system_check {'PASS' if r['pass'] else 'FAIL'}: CG {cin} / {cout} (pack in / out)",
             f"  access {sum(1 for a in acc_rows if a[1])} / {len(acc_rows)}; mass {mc['mass_g']:.0f} g E",
             f"fit: {sum(1 for f in ft if f['pass_'])} / {len(ft)} pairs pass, no allowance"]
    info = dict(flow=flow, usb=usb, stack=st, damper=dp, damper_rows=drows, holes=holes, bolts=bolts, tb_orig=tb, fc_foot=r0["foot_trim"], bipod=bp,
                notes=notes, fc_options=fco, bolt_variants=bvar, alt_text=alt_text)
    preview(P, M, info, os.path.join(RUN, "preview.png"))
    preview_alternatives(P, M, info, os.path.join(RUN, "preview_alternatives.png"))
    res = dict(version="v2 (concept): the FC on top (Spark 27 Oct), belly mount without the FC",
               layout="scout_layout v12 + main's assembly (camera mount v1, PR 146) + the tail bay as drawn (PR 130), the tail block's foot tip trimmed (requirement)",
               fc=dict(pose=dict(x=0.0, y=0.0, yaw=0.0, holes=holes, on="the Bee35's AIO holes"), stack=st, plan_margin_column=r0["col"], plan_margin_contracted=r0["contracted"],
                       gap_tail_block=r0["foot_trim"], gap_tail_block_as_drawn=r0["foot"], roof=dict(lowest=None if not np.isfinite(roof[0]) else roof[0], member=roof[1]),
                       prop_sweep_gap=round(PROP_Z - 4 - st["env"][1], 2), esc=dict(side_open=round(side_open, 3), top_open=round(top_open, 3),
                       side_area_mm2=round(side_open * side_A), top_area_mm2=round(top_open * top_A))),
               soft_mount=dict(pick={k: v for k, v in dp.items()}, crash_travel_mm=dict(lateral=round(crash_dx, 2), vertical=round(crash_dz, 2)),
                               table=[{k: v for k, v in rw.items() if k != "k"} for rw in drows]),
               fc_pose_search=dict(as_drawn_yaw0=A_, as_drawn_yaw45=C_, trimmed=D_, picked=r0),
               usb=dict(edge=usb["edge"], offset=usb["s"], azimuth=usb["az"], elevation=usb["el"], dir=np.round(usb["d"], 4).tolist(), p0=np.round(usb["p0"], 2).tolist(),
                        n_clear=usb["n_clear"], n_clear_sides=usb["n_clear_sides"]),
               flow=dict(flow, c=list(flow["c"]), bipod=bp, n_scored=len(scored), n_feasible=len(v1.FEAS)),
               belly_mount=dict(pick=pick, mass_g=bm_g, mass_solid_g=R["g_solid"], cg=np.round(bm_c, 2).tolist(), bounds=np.round(bm.bounds, 2).tolist(),
                                solid_check=dict(pass_=bool(ok), **{k: v for k, v in sc.items() if k != "ok"}), cradle_g=R["cradle_g"], plate_bolts=bolts,
                                receiver_inserts=REC_BOLTS, damper_inserts=holes, tail_face_x=TAIL_X, ear_y=EAR_Y,
                                variants={k: dict(g=q["g"], g_solid=q["g_solid"], crash=q["crash"], flight_landing=q["fl"], new_holes=q["new"], solid=q["ok"],
                                                  zmin=q["zmin"], by={kk: {f: list(vv) if isinstance(vv, tuple) else vv for f, vv in bb.items()} for kk, bb in q["L"]["by"].items()})
                                          for k, q in VR.items()},
                                sections_x={str(k): v for k, v in R["secs_x"].items()}, sections_y={str(k): v for k, v in R["secs_y"].items()},
                                cases=R["L"]["rows"], hung_g=R["L"]["hung_g"], hung_cg=R["L"]["hung_cg"]),
               tail_block_requirement=dict(foot_tip_x=[FOOT_X, FOOT_TRIM], removed_mm3=round(float(cut.volume), 1), g=[P[TBN]["spec"]["mass_g"], round(tb_g, 2)]),
               plate=dict(aio_holes="the FC's dampers", side_holes=SIDE_HOLES, nose_hole="inside the notch (was empty)", notch=dict(x=[NOTCH[0], round(edge_x, 1)], y=NOTCH[1]),
                          new_holes=VARIANTS[R["variant"]]["new"], v1_new_holes_dropped=[(-25, 13), (-25, -13), (30, 13), (30, -13)]),
               fit=ft, system_check=dict(summary=r["summary"], pass_=bool(r["pass"]), cg_offset_in=cin, cg_offset_out=cout, mass_g=mc["mass_g"], cg=mc["cg"], access=acc_rows),
               px4=px4, masses=masses, wires=WIRES, cables_box=CABLE_BOX, alternatives=dict(fc={k: {kk: vv for kk, vv in o.items()} for k, o in fco.items()}))
    json.dump(res, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=ser)
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())
    return res


if __name__ == "__main__":
    run()
