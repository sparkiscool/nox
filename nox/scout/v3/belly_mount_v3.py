"""Scout v3 belly mount v3 + the FC on top of the plate + the system check of the whole electronics mount (28 Oct 2026).
`python3 nox/scout/v3/belly_mount_v3.py` -> runs/scout_belly_mount/v3/ (run nox/scout/v3/electronics_v3.py first: the tail mount v3).

Why v3: scout_layout v13 (PR 169) re-trimmed the catch to 2.5 aft of the thrust centre (v12: 5.0 forward), so everything on the catch
(the receiver, this belly mount, the tail block and carrier, the XT60) moved 7.5 aft against the plate, the frame and the props. The FC
on top is plate-fixed (Spark 27 Oct). The belly mount and the tail mount meet at the plate and the catch, so they are redone together:
the tail mount v3 by electronics_v3.py, this pass on its result. v2 (runs/scout_belly_mount/v2/, round 2) stays as history; round 3's
FC decision (review of round 2) carries over: the FC hard on 3 standoffs (H-high) is the DEFAULT, the isolator B the alternative,
built only if the first-hover vibration log fails.
System pass on main's assembled Scout v3 (layout v13, camera mount v1) with the tail mount v3, every part in pose. v1's helpers
(`belly_mount.py`) are imported and never changed; its catch-riding numbers are moved 7.5 aft here at import (see DX).
  0. The rotor: 1P from the hover thrust at the model's mass (C_T bracket, E) over the flight thrust range; blade pass (3 blades).
  1. The FC: H-high, the board hard on 3 M3 standoffs over the front and side AIO holes (the shortest stock length clear of the tail
     block as drawn), nuts bonded under the plate, button screws on top; rigid and PCB modes (E); the PX4 filtering DEFAULT. The
     isolator B (4 inclined rubber mounts, a layout sketch) and the poses A and H-low weighed.
  2. FC USB-C from the board's tail corner (P), the exit searched.
  3. The flow sensor re-placed by v1's search (seeded), the plug end toward the nose.
  4. The belly mount: a U-frame (rails on the receiver's wall tops, the rear rung and tail face, ears), plate-bolt variants (the
     Bee35's side holes, + 2 new, 4 new; rails 6.5 or 11.6), the FC's nuts cleared; loads (L), sections every 1 mm, the hung set's
     first modes (a 3D frame clamped at the bolts, E); the flow bipod as a 3D frame.
  5. Fit (full pairwise table, no allowance), wash (CLAUDE.md 8: >= 3 outside the swept disks unless in the plate's shadow),
     system_check on the assembled model (cage first, shell, fit, probes, access, balance); masses and CGs; PX4 offsets; the plate's
     holes reconciled into one list.
  6. Pictures: preview.png, preview_alternatives.png.
Drone frame: x nose, y left, z up, plate underside 18.5, top 22.0, thrust centre x 0. Numbers: nox/dimensions.md System Scout v3,
A, H, M, L. Tags: C computed on the v13 geometry, E estimate, P measured, M mesh, F frame model.
"""
import os, sys, json, math, io, time
import numpy as np, trimesh
from scipy import ndimage as ndi

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import belly_mount as v1                                                  # v1's helpers (imports nox/ onto the path too)
LAYOUT = os.path.join(v1.ROOT, "runs", "scout_layout", "v13")            # scout_layout v13 (PR 169, on main)
DX = round(json.load(open(os.path.join(LAYOUT, "result.json")))["tab_x"]
           - json.load(open(os.path.join(v1.ROOT, "runs", "scout_layout", "v12", "result.json")))["tab_x"], 2)   # -7.5: the catch moved aft (C)
v1.REC_BOLTS = [(round(x + DX, 2), y) for x, y in v1.REC_BOLTS]          # v1's numbers that ride the catch, moved with it (C)
v1.EAR_X0 += DX; v1.FRONT_X += DX
v1.XT60_KO = (v1.XT60_KO[0] + DX, v1.XT60_KO[1] + DX, v1.XT60_KO[2])
v1.SWAP_COL = (v1.SWAP_COL[0] + DX, v1.SWAP_COL[1] + DX, v1.SWAP_COL[2])
v1.SWAP_PLAN = (v1.SWAP_PLAN[0] + DX, v1.SWAP_PLAN[1] + DX, v1.SWAP_PLAN[2])
v1.V12 = LAYOUT
from belly_mount import (ROOT, box, cyl, union, diff, centroid, rods_all, printed_g, min_gap, overlap, section_props, bolt_group,
                         cases, prying, strut_box, extrude_xz, flow_mesh, wake_r, REC_Z, PLATE_Z, PLATE_TOP, GAP, REC_BOLTS, INSERT,
                         BOSS_R, TAIL_X, EAR_Y, EAR_X0, XT60_KO, RAIL_Y, FRONT_X, FLOW_L, FLOW_W, FLOW_T, FLOW_HALF, FLOW_WIN, CRADLE,
                         STRUT_R, STRUT_X, DISKS, DISK_R, WASH, PROP_Z, PLANE, BAND, G, PETG_E, PETG_RHO, SIG_ALLOW, WALL,
                         INSERT_PULL, INSERT_SHEAR, PLATE_BEAR, HUNG)

RUN = os.path.join(ROOT, "runs", "scout_belly_mount", "v3")
REL = lambda p: os.path.relpath(p, ROOT)                                  # repo-relative paths in everything written (review 27 Oct)
ASM = os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")
ELEC = os.path.join(ROOT, "runs", "scout_electronics", "v3")              # the tail mount v3 (nox/scout/v3/electronics_v3.py)
LOG = io.StringIO(); v1.LOG = LOG                                         # v1's searches log into this run's log


def say(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); LOG.write(s + "\n")


# ---------------------------------------------------------------- numbers (C from the v13 meshes unless tagged)
FC_W, FC_HOLES = 36.0, 12.75              # MicoAir743v2-AIO 36 x 36, holes 25.5 square, Ø3 (M)
FC_C, FC_YAW = (1.9, 0.0), 45.0           # P (dimensions.md A, FC row): "36 x 36 diamond at (1.9, 0)", USB-C at the tail corner, slightly
                                          # right: the plate's AIO holes are the 25.5 square turned 45 about (1.9, 0) (review 27 Oct)
COMP = 3.2                                # components above and below the 1.6 PCB in the 8.0 envelope (v1 stack, E; CHECK on the board)
PCB_T = 1.6
BOSS_D = 7.4                              # tray boss under each FC hole: M3 through (Ø3.4) + 2.0 walls; the FC's M3 x 16 into a nyloc under it
HOLE_KO = BOSS_D / 2 + GAP + 0.1          # no component under the PCB within r 5.8 of a hole (E, CHECK): the boss + 2.0 (+0.1 numeric)
HEAD_D, HEAD_H = 5.7, 1.65                # M3 button head (ISO 7380, E)
NUT_AF, NUT_H = 5.5, 2.4                  # M3 hex nut (ISO 4032, E)
HEAD_KO = HEAD_D / 2 + GAP + 0.1          # no component above the PCB within r 4.95 of a hole (E, CHECK): the head + 2.0
NUT_KO = NUT_AF / math.sqrt(3) + GAP + 0.1   # the belly mount clears each FC nut under the plate by 2.0 (r 5.28)
BOSS_H = COMP + GAP + 0.1                 # the tray under the board's lower components by 2.0 (+0.1 numeric)
TRAY_T, PED_T, LINK_T = 3.5, 5.5, 3.5     # the tray plate, the base under a mount's low rim, the base's links and bosses (PETG; the links
                                          # 2.0 under every mount; >= 3 so the wedges and pads join without a neck, solid_check)
FOOT_X = -14.0                            # the tail block foot's nose tip on the plate top (tail mount v3, plate-fixed, C mesh)
FOOT_LEVER0 = 42.0                        # the tail block's plate bolts at x -42 (plate-fixed): the tail-up couple's lever is tip + 2 - (-42)
PLATE_TAIL = -50.0                        # the plate's tail edge (C, plate mesh): aft of it nothing is in the plate's shadow
SIDE_HOLES = [(3.9, 31.0), (3.9, -31.0)]  # the Bee35's own side holes (dimensions.md C, Plate bolts, P)
NOSE_HOLE, NOSE_HOLE_D = (45.0, 0.0), 3.4  # the Bee35's nose hole (P), M3 clearance
XT60_CUPS = [(48.25 + DX, 3.6), (48.25 + DX, -3.6)]  # XT60E1-M solder cups (x 40.75 on v13), tops z 14.8 (C mesh): the leads go straight up
LEAD_OD, FLOW_LEAD_D = 3.2, 2.5           # 14 AWG silicone; the flow's 4-wire SH lead (E)
NOTCH_X0 = min(NOSE_HOLE[0] - NOSE_HOLE_D / 2 - 0.5, XT60_CUPS[0][0] - LEAD_OD / 2 - 1.0)   # x 38.15 on v13: the leads straight up
                                          # from the cups + 1.0 (the cups moved 7.5 aft with the catch); the nose hole (45, 0) inside it
NOTCH_Y = XT60_CUPS[0][1] + LEAD_OD / 2 + 1.0          # |y| 6.2: the leads at the cups' y + 1.0; the flow lead (Ø2.5) runs between them
NOTCH_R = 1.5                             # corner radii (DEFAULT)
RAIL_IN, RAIL_OUT = 20.0, 26.5            # the U-frame's rails on the receiver's wall tops (|y| 20.5..25.5, C)
RUNG_W = 6.0                              # rungs 6 wide (the post rule), full height 9.6..18.5
FRONT_RUNG = (33.0 + DX, 39.0 + DX)       # behind the XT60 window (x 32.7 on v13), over the receiver top and lid
USB_W, USB_H, USB_L = 12.0, 7.0, 250.0    # FC USB-C 90 deg plug + cable (E, as v1)
GENT = lambda sh: 0.0981 * (56 + 7.62336 * sh) / (0.137505 * (254 - 2.54 * sh))   # Young's modulus (MPa) from Shore A (Gent, E)
DAMPERS = {"D8xH8": dict(d=8.0, h=8.0, end=1.0, g=1.6), "D10xH10": dict(d=10.0, h=10.0, end=1.2, g=2.8)}
                                          # M3 male-female rubber anti-vibration mounts, VE type (review: an M3 female needs Ø8+); rubber
                                          # Ø x height, metal ends; 30..50A listed (E: common listings, CHECK the one bought)
SHORES = (30, 40, 50)
M_FC, SCREW_G, NUT_G = 10.0, 0.4, 0.2     # the board (P, dimensions.md), M3 x 16 screw, M3 nyloc (E)
PROP_D_IN, BLADES = 3.5, 3                # Gemfan 3520, 3 blades (dimensions.md A, P)
PROP_CT = (0.10, 0.18)                    # static thrust coefficient C_T = T / (rho n^2 D^4) of a 3.5 in 3-blade prop (E: 0.10..0.18 bracket)
FLIGHT_T = (0.5, 4.0)                     # thrust in flight, x hover: 0.5 (a descent, E) .. 4 g (dimensions.md L flight); motor KV not recorded
LOOP_BAT, LOOP_MOT, LOOP_SIG = 30.0, 20.0, 25.0   # required free loop lengths of the leads to the board (DEFAULT): stiffer leads bridge the rubber
ISO_MARGIN, F_FLOOR = 1.4, 110.0          # isolation: every mode <= the lowest 1P / 1.4 (review); floor: the 100 Hz gate + 10 % for E


def rotor_bands(m_g):
    """rotor 1P from hover thrust (T = m g / 4 = C_T rho n^2 D^4, E: C_T bracket, motor KV unknown) and its flight range; blade pass"""
    T = m_g * G / 1000.0 / 4; D = PROP_D_IN * 0.0254
    n = {ct: math.sqrt(T / (ct * 1.225 * D ** 4)) for ct in PROP_CT}
    hov = (min(n.values()), max(n.values())); fl = (hov[0] * math.sqrt(FLIGHT_T[0]), hov[1] * math.sqrt(FLIGHT_T[1]))
    return dict(thrust_N=round(T, 3), hover_1P=[round(v) for v in hov], flight_1P=[round(v) for v in fl], blade_pass_hover=[round(BLADES * v) for v in hov],
                blade_pass_flight=[round(BLADES * v) for v in fl], iso_ceiling=round(fl[0] / ISO_MARGIN, 1), over_floor=round(BLADES * fl[1] * ISO_MARGIN))


def fc_stack_pcb(p0):
    return dict(pcb=[p0, p0 + PCB_T], env=[p0 - COMP, p0 + PCB_T + COMP])


# ---------------------------------------------------------------- the assembled model (main's assembly) without v1's FC / belly / flow
REPLACED = ("belly mount (PETG)", "mount + nose legs (Scout)", "FC", "wires", "flow MTF-02P", "tail bay mount block (Scout electronics, E)",
            "tail bay carrier (Scout electronics, E)", "tail bay cables (Scout electronics, E)", "Radxa Zero 3W", "UBEC", "XR2")
CAR = "electronics tail carrier (PETG)"
CABLES = "tail bay cables (Scout electronics, E)"
ELEC_FILES = {"electronics tail block (PETG)": ("belly-mount tail block (PETG)", "structure"), CAR: ("tail carrier (PETG)", "structure"),
              "Radxa Zero 3W": ("Radxa Zero 3W", "bought"), "UBEC": ("UBEC", "bought"), "XR2": ("XR2", "bought")}


def load_model():
    A = json.load(open(ASM)); P = {}
    for p in A["parts"]:
        n = p["name"]
        if n in REPLACED: continue
        if "mesh" in p: m = trimesh.load(os.path.join(ROOT, p["mesh"]), force="mesh")
        else: x0, x1, y0, y1, z0, z1 = p["box"]; m = box([x0, y0, z0], [x1, y1, z1])
        P[n] = dict(mesh=m, g=float(p["mass_g"]), c=np.array(p["cg"], float) if "cg" in p else centroid(m), role=p["role"],
                    tag=p.get("tag", "E"), spec=p)
    env = json.load(open(os.path.join(ELEC, "envelope.json")))             # the tail mount v3 as drawn, its items as fitted (Radxa: the
    for n, (en, role) in ELEC_FILES.items():                                # layout's box less the post keep-outs, + its plugs' keep-clear)
        f = env["files"][en]; m = trimesh.load(os.path.join(ROOT, f), force="mesh")
        d = env["mount"].get(en) or env["items"].get(en)
        P[n] = dict(mesh=m, g=float(d["mass_g"]), c=np.array(d["cg"], float), role=role, tag=d.get("tag", "E"), spec={}, file=f)
    cb = CABLE_BOX
    P[CABLES] = dict(mesh=box([cb[0], cb[2], cb[4]], [cb[1], cb[3], cb[5]]), g=float(env["cables"]["mass_g"]), c=np.array(env["cables"]["cg"], float),
                     role="structure", tag="E", spec={})
    sh = json.load(open(os.path.join(LAYOUT, "cage_shell.json")))
    links = json.load(open(os.path.join(LAYOUT, "result.json")))["links"]
    return dict(A=A, P=P, nodes=np.array(sh["nodes"], float), rods=[tuple(e) for e in sh["rods"]], shell=sh,
                links={k: (np.array(a, float), np.array(b, float)) for k, (a, b) in links.items()})


def _place(m, x, y, yaw_deg):
    T = trimesh.transformations.rotation_matrix(math.radians(yaw_deg), [0, 0, 1]); T[:3, 3] = [x, y, 0]; m.apply_transform(T); return m


def fc_mesh(x, y, yaw_deg, st):
    """the board: the PCB, the component layer above less r HEAD_KO round each hole (the screw head's pad), the layer below less
    r HOLE_KO round each hole (E, CHECK on the board)"""
    z0, z1 = st["env"]; p0, p1 = st["pcb"]; h = FC_W / 2
    lo = box([-h, -h, z0], [h, h, p0]); lo = diff(lo, [cyl([sx * FC_HOLES, sy * FC_HOLES, z0 - 1], [sx * FC_HOLES, sy * FC_HOLES, p0 + 0.001], HOLE_KO)
                                                    for sx in (-1, 1) for sy in (-1, 1)])
    hi = diff(box([-h, -h, p1], [h, h, z1]), [cyl([sx * FC_HOLES, sy * FC_HOLES, p1 - 0.001], [sx * FC_HOLES, sy * FC_HOLES, z1 + 1], HEAD_KO)
                                               for sx in (-1, 1) for sy in (-1, 1)])
    return _place(union([lo, box([-h, -h, p0], [h, h, p1]), hi]), x, y, yaw_deg)


def fc_parts(x, y, yaw_deg, st):
    """the PCB alone and the component layer under it (for the inside-contact checks)"""
    p0 = st["pcb"][0]; h = FC_W / 2
    lo = diff(box([-h, -h, st["env"][0]], [h, h, p0]), [cyl([sx * FC_HOLES, sy * FC_HOLES, st["env"][0] - 1], [sx * FC_HOLES, sy * FC_HOLES, p0 + 0.001], HOLE_KO)
                                                        for sx in (-1, 1) for sy in (-1, 1)])
    return _place(lo, x, y, yaw_deg)


def holes_xy(x, y, yaw_deg):
    a = math.radians(yaw_deg); R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return [tuple(np.round(np.array([x, y]) + R @ np.array([sx * FC_HOLES, sy * FC_HOLES]), 3)) for sx in (-1, 1) for sy in (-1, 1)]


def trimmed_tail_block(tb, x_trim):
    """the tail block with its foot tip on the plate top cut back to x_trim (nothing else of it lies ahead of x -31 above z 18)"""
    if x_trim >= FOOT_X: return tb
    return diff(tb, [box([x_trim, -40, PLATE_Z - 0.5], [40, 40, 80])])


# ---------------------------------------------------------------- 1. the FC isolator: 4 inclined rubber mounts between pedestals and a tray
def iso_layout(c, yaw, dmp, th_deg, r_mid):
    """4 mounts on the board's edge-midpoint directions (between its holes: on the plate diagonals for the diamond, clear of the
    tail block's centre line and of the side holes), each in its radial plane, inclined th from vertical with the top inward;
    midpoint at radius r_mid. A PETG base on the plate (bolted through the AIO holes but the one under the tail block's foot)
    carries the bottom ends on wedges, a PETG tray the top ends; the board bolts hard onto the tray's 4 bosses."""
    D = DAMPERS[dmp]; t = math.radians(th_deg); L = D["h"] + 2 * D["end"]; d = D["d"]
    c = np.array([c[0], c[1]], float); a0 = math.radians(yaw)
    out = []
    for k in range(4):
        ang = a0 + k * math.pi / 2                                       # the edge normals of the board (its holes sit at +45 deg)
        u2 = np.array([math.cos(ang), math.sin(ang)]); u = np.r_[u2, 0.0]; z = np.array([0, 0, 1.0])
        dv = -u * math.sin(t) + z * math.cos(t)
        rb = r_mid + L / 2 * math.sin(t); zb = PLATE_TOP + PED_T + (d / 2 + 1.0) * math.sin(t)
        pb = np.r_[c + u2 * rb, zb]; pt = pb + L * dv
        out.append(dict(u=u, d=dv, pb=pb, pt=pt, mid=(pb + pt) / 2, rb=rb))
    z_tr = max(o["pt"][2] + (d / 2 + 1.0) * math.sin(t) for o in out)
    p0 = z_tr + TRAY_T + BOSS_H
    return dict(c=tuple(c), yaw=yaw, dmp=dmp, th=th_deg, r_mid=r_mid, L=L, d=d, mounts=out, tray=[z_tr, z_tr + TRAY_T], **fc_stack_pcb(p0))


def _disk_pts(p, n, r, k=24):
    a = np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.99 else np.array([1.0, 0, 0]); a /= np.linalg.norm(a); b_ = np.cross(n, a)
    ang = np.linspace(0, 2 * math.pi, k, endpoint=False)
    return p + r * (np.cos(ang)[:, None] * a + np.sin(ang)[:, None] * b_)


def base_bolts(lay):
    """the AIO holes the base bolts through: all but the one under the tail block's foot (the rearmost, on the centre line)"""
    hs = holes_xy(*lay["c"], lay["yaw"]); rear = min(hs, key=lambda h: h[0])
    return [h for h in hs if h != rear]


def iso_meshes(lay):
    """the mounts (cylinders); the base (one PETG print): under each mount a wedge, the hull of a Ø(d + 2) disc on the mount's bottom
    face and its drop to the plate (all of it under that face: a full face contact, 3.0 round the stud's insert); Ø10 x 3.5 bosses on
    3 AIO holes (M3 down into nuts bonded under the plate); 6-wide x 3.5 links joining them round the front and sides (open at the
    tail, over the foot); the tray (one PETG print): the board's footprint plate, 4 Ø7.4 bosses up to the PCB, a pad on each mount's
    top face (the same hull, upward)"""
    import shapely.geometry as sg, shapely.affinity as sa
    d = lay["d"]; rr = d / 2 + 1.0; dm, we, pads = [], [], []
    for o in lay["mounts"]:
        dm.append(cyl(o["pb"], o["pt"], d / 2, 48))
        bot = _disk_pts(o["pb"], o["d"], rr); bot = np.vstack([bot, bot - 2.5 * o["d"]])          # a 2.5 band square to the face: no knife edge
        we.append(trimesh.PointCloud(np.vstack([bot, bot * [1, 1, 0] + [0, 0, PLATE_TOP]])).convex_hull)
        top = _disk_pts(o["pt"], o["d"], rr); top = np.vstack([top, top + 2.5 * o["d"]])
        pads.append(trimesh.PointCloud(np.vstack([top, top * [1, 1, 0] + [0, 0, lay["tray"][1]]])).convex_hull)
    bb = base_bolts(lay); zb1 = PLATE_TOP + LINK_T
    bosses = [cyl([hx, hy, PLATE_TOP], [hx, hy, zb1], BOSS_R, 48) for hx, hy in bb]
    ring = []                                                           # front and sides: boss, wedge, boss ... (no link across the tail)
    feet = [tuple(o["pb"][:2] - o["u"][:2] * 0.0) for o in lay["mounts"]]
    pts = sorted([("b", h) for h in bb] + [("w", f) for f in feet], key=lambda q: math.atan2(q[1][1] - lay["c"][1], q[1][0] - lay["c"][0]))
    for i in range(len(pts)):
        p_, q_ = pts[i][1], pts[(i + 1) % len(pts)][1]
        mid = ((p_[0] + q_[0]) / 2, (p_[1] + q_[1]) / 2)
        if mid[0] < lay["c"][0] - 8 and abs(mid[1] - lay["c"][1]) < 12: continue                      # the tail gap
        ring.append(sg.LineString([p_, q_]).buffer(3.0, cap_style=2))
    links = []
    for g in ring:
        m_ = trimesh.creation.extrude_polygon(g, zb1 - PLATE_TOP); m_.apply_translation([0, 0, PLATE_TOP]); links.append(m_)
    base = diff(union(we + bosses + links), [cyl([hx, hy, PLATE_TOP - 1], [hx, hy, zb1 + 1], 1.7, 32) for hx, hy in bb])   # layout sketch: the mounts' stud bores and the tray's screw holes are not drawn (alternative B)
    sq = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), lay["yaw"], origin=(0, 0)), *lay["c"])
    plate = trimesh.creation.extrude_polygon(sq, TRAY_T); plate.apply_translation([0, 0, lay["tray"][0]])
    tb_ = [cyl([hx, hy, lay["tray"][1] - 0.5], [hx, hy, lay["pcb"][0]], BOSS_D / 2, 48) for hx, hy in holes_xy(*lay["c"], lay["yaw"])]
    tray = diff(union([plate] + tb_ + pads), [cyl([hx, hy, lay["tray"][0] - 1], [hx, hy, lay["pcb"][0] + 1], 1.7, 32) for hx, hy in holes_xy(*lay["c"], lay["yaw"])])
    links_only = diff(union(bosses + links), we)                         # for the inside-contact check: the links stay 2.0 off the mounts
    return dict(dampers=union(dm), pedestals=base, tray=tray, links=links_only)


def kcol(Gm, h, d):
    """one rubber column, guided at both ends: axial (compression, shape factor E_c = E (1 + 2 k S^2), k 0.6), transverse (shear +
    bending, Timoshenko), torsion G J / h; N/mm, N mm / rad (E)"""
    E = 3.0 * Gm; A = math.pi * d * d / 4; I = math.pi * d ** 4 / 64
    ks = 1.0 / (h / (0.9 * Gm * A) + h ** 3 / (12 * E * I)); S = d / (4 * h); ka = E * (1 + 1.2 * S * S) * A / h
    return dict(a=ka, s=ks, t=Gm * 2 * I / h)


def rigid_modes(springs, m_kg, cg, Icg):
    """6-DOF rigid body on oriented springs [(point mm, axis, k_axial, k_transverse, k_torsion)] (N/mm): Hz with the dominant motion"""
    K = np.zeros((6, 6)); Mm = np.zeros((6, 6)); Mm[:3, :3] = np.eye(3) * m_kg; Mm[3:, 3:] = Icg
    for p, dv, ka, ks, kt in springs:
        r = (np.asarray(p, float) - cg) * 1e-3; dv = np.asarray(dv, float) / np.linalg.norm(dv)
        Ks = (ka * np.outer(dv, dv) + ks * (np.eye(3) - np.outer(dv, dv))) * 1e3
        T = np.hstack([np.eye(3), -np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])])
        K += T.T @ Ks @ T; K[3:, 3:] += kt * 1e-3 * np.outer(dv, dv)
    w2, V = np.linalg.eig(np.linalg.solve(Mm, K)); o = np.argsort(w2.real); names = ["x", "y", "z", "roll", "pitch", "yaw"]
    return [(round(math.sqrt(max(w2[i].real, 0)) / (2 * math.pi), 1), names[int(np.argmax(np.abs(V[:, i].real) * np.sqrt(np.diag(Mm))))]) for i in o]


def moving_set(lay, tray_g=None, tray_mesh=None):
    """what rides on the rubber: the board 10 g (P) at its envelope centre, the tray, 4 M3 x 16 + 4 nylocs at the holes, half of each
    mount (its top end and half the rubber) at its top: mass (kg), CG (mm), inertia about the CG (kg m^2) (E)"""
    D = DAMPERS[lay["dmp"]]; c3 = np.r_[np.array(lay["c"]), 0.0]
    items = [(M_FC, c3 + [0, 0, (lay["env"][0] + lay["env"][1]) / 2], (FC_W, FC_W, lay["env"][1] - lay["env"][0]))]
    if tray_mesh is not None:
        tg = tray_g; items.append((tg, np.asarray(tray_mesh.center_mass), tuple(tray_mesh.extents)))
    else:
        tg = PETG_RHO * (FC_W ** 2 * TRAY_T + 4 * math.pi / 4 * BOSS_D ** 2 * BOSS_H + 4 * D["d"] ** 2 * 4) * 0.95   # estimate in the search
        items.append((tg, c3 + [0, 0, lay["tray"][0] + 1.5], (FC_W * 1.41, FC_W * 1.41, TRAY_T + BOSS_H)))
    for hx, hy in holes_xy(*lay["c"], lay["yaw"]): items.append((SCREW_G + NUT_G, np.array([hx, hy, (lay["tray"][0] + lay["pcb"][1]) / 2]), (3, 3, 16)))
    for o in lay["mounts"]: items.append((D["g"] / 2, o["pt"], (D["d"], D["d"], 3)))
    m = sum(i[0] for i in items); cg = sum(i[0] * i[1] for i in items) / m
    I = np.zeros((3, 3))
    for mi, p, ext in items:
        a, b_, c_ = ext; Ib = mi / 12 * np.diag([b_ ** 2 + c_ ** 2, a ** 2 + c_ ** 2, a ** 2 + b_ ** 2]); r = p - cg
        I += Ib + mi * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return m / 1000.0, cg, I * 1e-9, round(m, 2)


def iso_modes(lay, shore, tray_g=None, tray_mesh=None, extra=()):
    D = DAMPERS[lay["dmp"]]; k = kcol(GENT(shore) / 3.0, D["h"], D["d"])
    m, cg, I, mg = moving_set(lay, tray_g, tray_mesh)
    sp = [(o["mid"], o["d"], k["a"], k["s"], k["t"]) for o in lay["mounts"]] + list(extra)
    return rigid_modes(sp, m, cg, I), k, mg, cg


def plan_margin_poly(poly):
    import shapely.geometry as sg
    d = min(poly.distance(sg.Point(mx, my)) for mx, my in DISKS); return d - DISK_R


def iso_search(bands):
    """every mount size and Shore, inclination 0..70 deg, midpoint radius 12..24, with the leads as springs (their loops as required):
    all 6 modes inside [110, the lowest flight 1P / 1.4] and the pedestals 3 off the swept disks; pick: the largest margin to both
    bounds (to 0.01), then the smallest pedestal reach (the tail block's foot), then the lowest board"""
    import shapely.geometry as sg
    rows = []
    for dmp in DAMPERS:
        D = DAMPERS[dmp]
        for sh in SHORES:
            for th in range(0, 71, 5):
                for r in range(12, 25):
                    lay = iso_layout(FC_C, FC_YAW, dmp, th, float(r))
                    md, k, mg, cg = iso_modes(lay, sh, extra=lead_springs(lay))
                    f = [x[0] for x in md]
                    r_out = max(o["rb"] for o in lay["mounts"]) + (D["d"] / 2 + 1.0) * math.cos(math.radians(th)) + 2.5 * math.sin(math.radians(th))
                    ped = sg.MultiPoint([tuple(np.array(lay["c"]) + o["u"][:2] * (r_out - D["d"] / 2 - 1.0)) for o in lay["mounts"]]).buffer(D["d"] / 2 + 1.0)
                    pm = plan_margin_poly(ped)
                    marg = min(f[0] / F_FLOOR, bands["iso_ceiling"] / f[-1])
                    rows.append(dict(dmp=dmp, shore=sh, th=th, r=r, modes=md, f_lo=f[0], f_hi=f[-1], margin=round(marg, 3), pcb=round(float(lay["pcb"][0]), 2),
                                     plan=round(pm, 2), r_out=round(r_out, 1), ok=bool(f[0] >= F_FLOOR and f[-1] <= bands["iso_ceiling"] and pm >= WASH)))
    ok = [r for r in rows if r["ok"]]
    pick = max(ok, key=lambda r: (round(r["margin"], 2), -r["r_out"], -r["pcb"])) if ok else None
    vert = [r for r in rows if r["th"] == 0]                            # vertical columns (round 1's layout): the best of them
    vbest = max(vert, key=lambda r: r["margin"])
    return rows, pick, vbest


def find_trim(tb, meshes, step=0.5):
    """the least foot-tip trim (x) with every mesh 2.0 off the tail block; the tail-up couple's lever shrinks with it"""
    u = union(meshes)
    for xt in np.arange(FOOT_X, -45.0, -step):
        t = trimmed_tail_block(tb, float(xt))
        if bbox_gap(t, u) >= GAP or min_gap(t, u, 3000) >= GAP: return float(round(xt, 2))
    return None


def roof_limit(M, poly):
    """the lowest cage member over a plan polygon + 3 (cage first)"""
    import shapely.geometry as sg
    sq = poly.buffer(3.0); lo = np.inf; who = None
    for a, b, r in rods_all(M):
        q = a + np.linspace(0, 1, 400)[:, None] * (b - a)
        for p in q:
            if p[2] > PLATE_TOP and sq.contains(sg.Point(p[0], p[1])) and p[2] - r < lo: lo = p[2] - r; who = (np.round(a, 1).tolist(), np.round(b, 1).tolist())
    return lo, who


def lead_springs(lay):
    """the leads to the board as springs at its corners (each a stranded cantilever loop, E): 2 x 14 AWG battery leads, 12 x 20 AWG
    motor leads (3 a corner), 20 x 26 AWG signal wires, each with its required loop length; stranded wire bends at ~5 % of solid
    copper (E)"""
    Ecu = 117e3 * 0.05; out = []
    def k(dw, L): return 3 * Ecu * math.pi * dw ** 4 / 64 / L ** 3
    c = np.r_[np.array(lay["c"]), lay["pcb"][1]]; R = FC_W / 2 * math.sqrt(2)          # the diamond's corners lie on the plate axes
    for n_, dw, L, u in [(2, 1.63, LOOP_BAT, (1, 0))] + [(3, 0.81, LOOP_MOT, uu) for uu in ((1, 0), (0, 1), (-1, 0), (0, -1))] + [(20, 0.40, LOOP_SIG, (-1, 0))]:
        kk = n_ * k(dw, L); p = c + np.r_[np.array(u, float) * R * 0.9, 0]
        out.append((p, (0, 0, 1.0), kk, kk, 0.0))
    return out


# ---------------------------------------------------------------- 1b. the DEFAULT: the FC hard on 3 standoffs, high enough to clear the tail block as drawn
STO_AF, STO_BORE = 5.5, 2.5               # M3 aluminium hex standoff, male-female, 5.5 across flats (E: the common M3 part)
STO_D = STO_AF / math.cos(math.radians(30))   # 6.35 across corners
AL_E, JOINT = 70e3, 0.1                   # aluminium; the threaded joints keep about 1/10 of a standoff's beam stiffness (E)
STO_G = (0.9, 0.4, 0.2)                   # a standoff ~12 long, the M3 x 6 button screw through the PCB, the M3 nut bonded under the plate (E)
STO_STOCK = (8.0, 10.0, 12.0, 15.0, 20.0, 25.0)   # M3 hex standoff stock lengths (E): the shortest that clears is bought
FR4_E, FR4_NU = 20e3, 0.15                # the PCB (E)
FILTERING = ("DEFAULT (review round 2): the board hard-mounted, the rotor tones handled in PX4: bidirectional DShot (DSHOT_BIDIR_EN 1; the AIO's ESC "
             "runs Bluejay, which reports RPM; nox/scout/scout.md notes the Bluejay telemetry-bit bug fixed upstream, PX4 PR #28826: fly a build with it), "
             "the ESC-RPM dynamic notch (IMU_GYRO_DNF_EN, ESC RPM bit), the gyro FFT notch (IMU_GYRO_FFT_EN 1). Gate: the first-hover vibration log "
             "(accel never clipping; vibration metrics in PX4's flight-review guidance). Isolator B is built only if that log fails, with its window "
             "re-cut on the measured 1P and a measured mount stiffness")


def hard_layout(tb):
    """H-high: the board hard on 3 M3 male-female aluminium standoffs over the front and side AIO holes (the rear one sits under the
    tail block's foot), each stud (6 long: the plate 3.5 + the nut 2.4) down through the plate into an M3 nut bonded under it, an
    M3 x 6 button screw down through the PCB into its female top; the shortest stock standoff that keeps the board's lower
    components 2.0 over the plate and the board and standoffs 2.0 off the tail block as drawn. Drawn: the hex bodies, the nuts
    and the heads (the stud and the screw shank sit inside the plate's and the PCB's holes)"""
    holes = [(round(float(hx), 3), round(float(hy), 3)) for hx, hy in base_bolts(dict(c=FC_C, yaw=FC_YAW))]
    for L_ in STO_STOCK:
        p0 = PLATE_TOP + L_
        if p0 < PLATE_TOP + GAP + COMP: continue
        st = fc_stack_pcb(float(p0)); fcm = fc_mesh(*FC_C, FC_YAW, st)
        sto = union([cyl([hx, hy, PLATE_TOP], [hx, hy, float(p0)], STO_D / 2, 6) for hx, hy in holes]
                    + [cyl([hx, hy, PLATE_Z - NUT_H], [hx, hy, PLATE_Z], NUT_AF / math.sqrt(3), 6) for hx, hy in holes]
                    + [cyl([hx, hy, p0 + PCB_T], [hx, hy, p0 + PCB_T + HEAD_H], HEAD_D / 2, 32) for hx, hy in holes])
        u = union([fcm, sto]); g = bbox_gap(tb, u)
        g = g if g >= GAP else min_gap(tb, u, 3000)
        if g >= GAP: return dict(c=FC_C, yaw=FC_YAW, holes=holes, L=L_, fc=fcm, sto=sto, foot_gap=round(g, 2), **st)
    return None


def hard_modes(hl):
    """the board as a rigid body on the 3 standoffs (each guided by the clamped PCB: axial E A / L, transverse 12 E I / L^3, both at
    1/10 for the threads, E), and the PCB's own first mode at its free tail corner: the part behind the side standoffs' line as a
    uniform cantilever strip (f = 1.875^2 / (2 pi L^2) sqrt(D / rho h), FR4, the board's 10 g spread over it; E, the triangle's taper
    raises it)"""
    L = hl["L"]; A = math.sqrt(3) / 2 * STO_AF ** 2 - math.pi / 4 * STO_BORE ** 2
    s_ = STO_AF / math.sqrt(3); I = 5 * math.sqrt(3) / 16 * s_ ** 4 - math.pi * STO_BORE ** 4 / 64
    ka = JOINT * AL_E * A / L; ks = JOINT * 12 * AL_E * I / L ** 3
    m = M_FC + 3 * STO_G[1]; c3 = np.r_[np.array(hl["c"]), (hl["env"][0] + hl["env"][1]) / 2]
    Ib = m / 12 * np.diag([FC_W ** 2 + 8.0 ** 2, FC_W ** 2 + 8.0 ** 2, 2 * FC_W ** 2]) * 1e-9
    sp = [(np.array([hx, hy, (PLATE_TOP + hl["pcb"][0]) / 2]), (0, 0, 1.0), ka, ks, 0.0) for hx, hy in hl["holes"]]
    rigid = rigid_modes(sp, m / 1000.0, c3, Ib)
    Lc = FC_W / math.sqrt(2) * 1e-3; D = FR4_E * 1e6 * (PCB_T * 1e-3) ** 3 / (12 * (1 - FR4_NU ** 2)); rho_h = M_FC * 1e-3 / (FC_W * 1e-3) ** 2
    f_plate = 1.875 ** 2 / (2 * math.pi * Lc ** 2) * math.sqrt(D / rho_h)
    return dict(rigid=rigid, plate_Hz=round(f_plate), k_axial=round(ka), k_trans=round(ks), mass_g=m, L=L)


# ---------------------------------------------------------------- 3. occupancy on a grid that reaches over the plate, the USB-C search
def occ(M, lo, hi, pitch=1.0, skip=(), rods=True, extra=(), rod_meshes=False):
    from solid_check import voxels
    lo = np.asarray(lo, float); hi = np.asarray(hi, float); shape = tuple(int(round((hi[k] - lo[k]) / pitch)) for k in range(3))
    O = np.zeros(shape, bool)
    ms = [(n, p["mesh"]) for n, p in M["P"].items() if n not in skip and (rod_meshes or n not in ("cage shell tubes", "spokes, bay ties, core ties, camera guard"))] + list(extra)
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


def usb_search2(M, lay, skip_extra=()):
    """P (dimensions.md A): the USB-C sits at the board's tail corner, slightly right. On the turned board the 2 edges that meet at
    the tail corner: port centres 2..14 from the corner along each, z = PCB top + 1.6; the 12 x 7 plug + cable swept 250 at azimuth
    +-60 and elevation -10..+30 deg (5 deg steps) from the edge normal: every sample clear of every part and rod (1 mm grid) but the
    FC. Objective: the straightest exit, then the right-hand edge, then nearest the corner"""
    O, lo, p = occ(M, [-280, -200, -80], [200, 200, 140], 1.0, skip=("FC", "wires", "tail bay cables (Scout electronics, E)") + tuple(skip_extra), rod_meshes=True)
    O = ndi.binary_dilation(O, iterations=1)                            # 1 mm more round everything: the grid's sampling error (conservative)
    shp = np.array(O.shape); zc = lay["pcb"][1] + 1.6; c = np.array(lay["c"], float); a0 = math.radians(lay["yaw"])
    uu, vv = np.meshgrid(np.linspace(-USB_W / 2, USB_W / 2, 7), np.linspace(-USB_H / 2, USB_H / 2, 5)); uu = uu.ravel(); vv = vv.ravel()
    t = np.arange(1.0, USB_L + 0.1, 1.0)
    R = np.array([[math.cos(a0), -math.sin(a0)], [math.sin(a0), math.cos(a0)]])
    corners = [c + R @ np.array([sx * FC_W / 2, sy * FC_W / 2]) for sx in (-1, 1) for sy in (-1, 1)]
    tail = min(corners, key=lambda q: q[0])
    out = []; n = 0
    for k, (sx, sy) in enumerate(((-1, 1), (1, -1))):                   # the 2 edges at the tail corner
        other = [q for q in corners if np.linalg.norm(q - tail) > 1 and abs(np.linalg.norm(q - tail) - FC_W) < 0.5]
        for q in other:
            tan = (q - tail) / np.linalg.norm(q - tail); nrm = np.array([tan[1], -tan[0]])
            if np.dot(nrm, tail + tan * FC_W / 2 - c) < 0: nrm = -nrm
            side = "right" if (tail + tan * FC_W / 2)[1] < c[1] else "left"
            for s_ in np.arange(2.0, 14.01, 2.0):
                p0 = np.r_[tail + tan * s_, zc]
                for az in np.arange(-60, 61, 5):
                    for el in np.arange(-10, 31, 5):
                        n += 1; a_, e_ = math.radians(az), math.radians(el)
                        dh = nrm * math.cos(a_) + tan * math.sin(a_); d = np.r_[dh * math.cos(e_), math.sin(e_)]
                        w = np.cross([0, 0, 1.0], d); w /= np.linalg.norm(w); hv = np.cross(d, w)
                        qq = (p0[None, None] + t[:, None, None] * d + uu[None, :, None] * w + vv[None, :, None] * hv).reshape(-1, 3)
                        ij = np.floor((qq - lo) / p).astype(int); k_ = ((ij >= 0) & (ij < shp)).all(1)
                        if O[ij[k_, 0], ij[k_, 1], ij[k_, 2]].any(): continue
                        out.append((abs(az) + abs(el) + (0.0 if side == "right" else 0.1) + s_ / 1000, side, float(s_), float(az), float(el), d, p0))
        break
    out.sort(key=lambda r: r[0])
    say(f"USB-C search (tail corner, P): {len(out)} clear of {n}; right edge {sum(1 for r in out if r[1] == 'right')}, left {sum(1 for r in out if r[1] == 'left')}")
    return out


# ---------------------------------------------------------------- 5. the belly mount v3, drawn
VARIANTS = {
    "VA": dict(bolts=SIDE_HOLES, new=[], label="the Bee35's 2 side holes (3.9, +-31), 0 new holes"),
    "VB": dict(bolts=SIDE_HOLES + [(-25.0, 13.0), (-25.0, -13.0)], new=[(-25.0, 13.0), (-25.0, -13.0)], label="side holes + 2 new at (-25, +-13)"),
    "VC": dict(bolts=[(-25.0, 13.0), (-25.0, -13.0), (27.0, 13.0), (27.0, -13.0)], new=[(-25.0, 13.0), (-25.0, -13.0), (27.0, 13.0), (27.0, -13.0)],
               label="4 new holes (-25, +-13), (27, +-13) (v1's front pair 3 aft: the XT60 window is at x 32.7 on v13)"),
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
        y0, y1 = sorted((sy * EAR_Y, sy * RAIL_Y)); B.append(box([max(EAR_X0, PLATE_TAIL), y0, zz[0]], [TAIL_X, y1, zz[1]]))   # ear on the tail
                                                                                          # block (|y| 21), no further aft than the plate's edge
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
    if front_rung or any(abs(bx - 27.0) < 0.1 for bx, by in V["bolts"]):
        fr = (22.0, 32.0) if any(abs(bx - 27.0) < 0.1 for bx, by in V["bolts"]) else FRONT_RUNG
        B.append(box([fr[0], -RAIL_IN - 0.01, zz[0]], [fr[1], RAIL_IN + 0.01, zz[1]]))     # front rung
    for x_, y_ in REC_BOLTS + list(V["bolts"]) + list(damper_holes): B.append(cyl([x_, y_, zz[0]], [x_, y_, zz[1]], BOSS_R))
    B += bipod_cradle(flow)
    body = union(B)
    cuts = [box([XT60_KO[0], -XT60_KO[2], zz[0] - 1], [XT60_KO[1], XT60_KO[2], zz[1] + 1])]   # XT60E1-M + 2.0 (the leads go straight up)
    for x_, y_ in REC_BOLTS: cuts.append(cyl([x_, y_, zz[0] - 1], [x_, y_, zz[0] + INSERT[1]], INSERT[0] / 2, 32))
    for x_, y_ in list(V["bolts"]) + list(damper_holes): cuts.append(cyl([x_, y_, zz[1] - INSERT[1]], [x_, y_, zz[1] + 1], INSERT[0] / 2, 32))
    for x_, y_ in base_bolts(dict(c=FC_C, yaw=FC_YAW)):                                    # the FC standoffs' nuts under the plate + 2.0
        cuts.append(cyl([x_, y_, zz[1] - NUT_H - GAP - 0.1], [x_, y_, zz[1] + 1], NUT_KO, 48))
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
    bay_g = sum(P[n]["g"] for n in ("electronics tail block (PETG)", "electronics tail carrier (PETG)", "Radxa Zero 3W", "UBEC", "XR2")) + 0.5 * P[CABLES]["g"]
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
        modes = [round(math.sqrt(max(w, 0)) / (2 * math.pi), 0) for w in w2[:6]]
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


# ---------------------------------------------------------------- the hung set on the frame: its first modes (review 27 Oct)
def hung_set(P):
    """the receiver set + swap set hanging from the 4 receiver inserts: mass, CG, inertia (each part a uniform box of its mesh's
    bounds at its CG, E)"""
    items = [(P[n]["g"], np.asarray(P[n]["c"], float), P[n]["mesh"].extents) for n in HUNG]
    m = sum(i[0] for i in items); cg = sum(i[0] * i[1] for i in items) / m; I = np.zeros((3, 3))
    for mi, c, ext in items:
        a, b_, c_ = ext; r = c - cg
        I += mi / 12 * np.diag([b_ ** 2 + c_ ** 2, a ** 2 + c_ ** 2, a ** 2 + b_ ** 2]) + mi * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return m, cg, I * 1e-9


def rect_sec(bw, h, t=WALL):
    """a printed rectangle (2.4 walls, the infill ignored: conservative): A, I about the horizontal axis, I about the vertical, J"""
    bi, hi = bw - 2 * t, h - 2 * t
    A = bw * h - max(bi, 0) * max(hi, 0)
    Iy = (bw * h ** 3 - max(bi, 0) * max(hi, 0) ** 3) / 12; Iz = (h * bw ** 3 - max(hi, 0) * max(bi, 0) ** 3) / 12
    if bi > 0 and hi > 0: J = 2 * t * (bw - t) ** 2 * (h - t) ** 2 / (bw + h - 2 * t)
    else: a_, b_ = max(bw, h), min(bw, h); J = a_ * b_ ** 3 / 3 * (1 - 0.63 * b_ / a_)
    return A, Iy, Iz, J


def hung_modes(variant, P, rail_out=RAIL_OUT):
    """the frame as 3D beams (rails, rungs, ears, bosses; PETG with 2.4 walls) clamped at its plate bolts, the hung set a rigid body
    tied to the 4 receiver inserts; the plate contact and friction left out (a lower bound: a rail clamped by one bolt a side bends
    as a cantilever, review 27 Oct). Returns the first 6 modes (Hz, F on E)"""
    E = PETG_E; Gs = E / 2.6; zc = (REC_Z + PLATE_Z) / 2; yr = (RAIL_IN + rail_out) / 2
    nodes = []; idx = {}; el = []; fixed = set()
    def N(key, p):
        if key not in idx: idx[key] = len(nodes); nodes.append(np.array(p, float))
        return idx[key]
    def beam(a, c, sec): el.append((a, c, E, Gs) + tuple(sec))
    rail = rect_sec(rail_out - RAIL_IN, PLATE_Z - REC_Z)
    xr = -28.0 if variant == "VA" else -25.5; wr = RUNG_W if variant == "VA" else 11.0
    for s in (1, -1):
        segs = ([[TAIL_X, xr, -24.0, -18.0], [18.0, 24.0, 27.0, REC_BOLTS[2][0]]] if variant == "VC" else
                [[TAIL_X, xr, -24.0, SIDE_HOLES[0][0], 24.0, REC_BOLTS[2][0]]])
        for seg in segs:
            for x0, x1 in zip(seg[:-1], seg[1:]): beam(N(("r", s, x0), (x0, s * yr, zc)), N(("r", s, x1), (x1, s * yr, zc)), rail)
        R = N(("R", s), (REC_BOLTS[0][0], s * REC_BOLTS[0][1], zc)); beam(R, N(("r", s, TAIL_X), (TAIL_X, s * yr, zc)), rect_sec(13.0, PLATE_Z - REC_Z))
        F = N(("F", s), (REC_BOLTS[2][0], s * REC_BOLTS[2][1], zc)); beam(F, N(("r", s, REC_BOLTS[2][0]), (REC_BOLTS[2][0], s * yr, zc)), rect_sec(2 * BOSS_R, PLATE_Z - REC_Z))
        if variant in ("VA", "VB"):
            B = N(("B", s), (SIDE_HOLES[0][0], s * SIDE_HOLES[0][1], zc)); beam(N(("r", s, SIDE_HOLES[0][0]), (SIDE_HOLES[0][0], s * yr, zc)), B, rect_sec(2 * BOSS_R, PLATE_Z - REC_Z)); fixed.add(B)
    def rung(x, w, ys, tag):
        for y0, y1 in zip(ys[:-1], ys[1:]):
            a = N(("r", -1, x) if y0 == -yr else (tag, y0), (x, y0, zc)); c = N(("r", 1, x) if y1 == yr else (tag, y1), (x, y1, zc)); beam(a, c, rect_sec(w, PLATE_Z - REC_Z))
    if variant == "VA": rung(xr, wr, [-yr, 0.0, yr], "q")
    else:
        rung(xr, wr, [-yr, -13.0, 13.0, yr], "q"); fixed |= {idx[("q", -13.0)], idx[("q", 13.0)]}
    if variant == "VC": rung(27.0, 10.0, [-yr, -13.0, 13.0, yr], "f"); fixed |= {idx[("f", -13.0)], idx[("f", 13.0)]}
    m, cg, I = hung_set(P)
    C = N("cg", cg)
    for k in (("R", 1), ("R", -1), ("F", 1), ("F", -1)): el.append((C, idx[k], E * 1e3, Gs * 1e3, 1e4, 1e7, 1e7, 1e7))   # the receiver: rigid
    nodes = np.array(nodes)
    masses = {i: (0.0005, 1e-9, 1e-9, 1e-9) for i in range(len(nodes)) if i not in fixed}
    masses[C] = (m / 1000.0, I[0, 0], I[1, 1], I[2, 2])
    _, _, md = frame3d(nodes, el, fixed, {}, masses=masses)
    return md


# ---------------------------------------------------------------- 6. fit: our pieces against every part (full table)
BM, FCN, FLW, TBN = "belly mount (PETG)", "FC", "flow MTF-02P", "electronics tail block (PETG)"
TRAY, PEDS, DMP = "FC isolator tray (PETG)", "FC isolator base (PETG)", "FC mounts (4 rubber, M3 male-female)"   # alternative B only
STO = "FC standoffs (3, M3 aluminium)"
OURS = (BM, FCN, STO, FLW, TBN, "electronics tail carrier (PETG)")
PLATE = "frame plate (Bee35 carbon)"
CONTACT = {frozenset(p) for p in ((BM, PLATE), (BM, "receiver v3 (latch)"), (BM, "receiver lid (latch)"), (BM, TBN), (BM, FLW),
                                  (STO, FCN), (STO, PLATE), (TBN, PLATE), (TBN, "receiver v3 (latch)"),
                                  (TBN, "electronics tail carrier (PETG)"), ("electronics tail carrier (PETG)", "Radxa Zero 3W"),
                                  ("electronics tail carrier (PETG)", "UBEC"), ("electronics tail carrier (PETG)", "XR2"))}


def bbox_gap(a, b):
    d = np.maximum(np.maximum(a.bounds[0] - b.bounds[1], b.bounds[0] - a.bounds[1]), 0.0); return float(np.linalg.norm(d))


def fit_full(P, rods, fc_low):
    """every pair of ours with every part: min gap (sampled, 6000 + vertices each side) when the boxes are within 25, else the box gap
    as a lower bound; overlap volume when closer than 0.5. Declared face contacts pass at <= 0.05 mm3, every other pair needs 2.0.
    Inside the declared contacts, the faces that must not touch are checked apart."""
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
    lower = trimesh.boolean.intersection([P[BM]["mesh"], box([-100, -100, -80], [100, 100, REC_Z - 0.6])], engine="manifold")
    for b in ("receiver v3 (latch)", "receiver lid (latch)", "receiver TPU end pieces (latch, E)"):
        g = min_gap(lower, P[b]["mesh"])
        rows.append(dict(a="belly mount below z 9.0 (bipod, gussets, cradle)", b=b, overlap_mm3=0.0, gap_mm=round(g, 2), lower_bound=False, kind=">= 2", pass_=bool(g >= GAP)))
    g = min_gap(P[STO]["mesh"], fc_low)
    rows.append(dict(a=STO, b="FC component layer under the PCB", overlap_mm3=round(overlap(P[STO]["mesh"], fc_low), 3), gap_mm=round(g, 2), lower_bound=False,
                     kind=">= 2", pass_=bool(g >= GAP - 1e-6)))
    for a in OURS:
        m = P[a]["mesh"]; pts = np.vstack([m.vertices, m.sample(6000, seed=3)]); best = (np.inf, None)
        for p, q, r in rods:
            u = q - p; t = np.clip(((pts - p) @ u) / (u @ u), 0, 1); d = np.linalg.norm(pts - (p + t[:, None] * u), axis=1).min() - r
            if d < best[0]: best = (d, (np.round(p, 1).tolist(), np.round(q, 1).tolist()))
        rows.append(dict(a=a, b="cage rods / links (nearest)", overlap_mm3=0.0, gap_mm=round(float(best[0]), 2), lower_bound=False, kind=">= 2",
                         pass_=bool(best[0] >= GAP), rod=best[1]))
    return rows


def wash_report(P, names):
    """CLAUDE.md 8: each piece's least plan distance outside the swept disks (need >= 3), points in the plate's shadow (under the
    carbon, inside its outline + 0.05) left out; where it is"""
    import shapely, shapely.geometry as sg, shapely.ops as so
    sec = P[PLATE]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
    plate = so.unary_union([sg.Polygon(e[:, :2]) for e in sec.discrete]).buffer(0.05)
    out = {}
    for n in names:
        m = P[n]["mesh"]; Q = np.vstack([m.vertices, m.sample(8000, seed=5)])
        wd = np.min([np.hypot(Q[:, 0] - mx, Q[:, 1] - my) for mx, my in DISKS], axis=0) - DISK_R
        wd[shapely.contains_xy(plate, Q[:, 0], Q[:, 1]) & (Q[:, 2] <= PLATE_Z + 0.1)] = np.inf
        i = int(np.argmin(wd)); bad = Q[wd < WASH]
        out[n] = dict(wash=round(float(wd[i]), 1) if np.isfinite(wd[i]) else None, at=np.round(Q[i], 1).tolist(),
                      bad_box=np.round([bad.min(0), bad.max(0)], 1).tolist() if len(bad) else None)
    return out


# ---------------------------------------------------------------- 7. the system model with our pieces: system_check (repo-relative)
CABLE_BOX = (-34.2, -33.8, 11.8, 12.2, 39.8, 40.2)   # the tail bay cables' 0.4 fit box, moved beside the fin (main's at (-4, 1.4, 32) is inside the FC tower)
WIRES = dict(box=[[35.8, -0.2, 24.0], [36.2, 0.2, 24.4]], cg=[3.0, 0.0, 29.0],
             note="20 g E: motor leads from the board's corners down to the arm tops, battery leads up the notch to the board, signal leads; "
                  "CG (3, 0, 29) E; for the fit a 0.4 box in the battery-lead lane on the plate top, no allowance")


def write_stl(m, name):
    p = os.path.join(RUN, name); m.export(p); return REL(p)


def manifest(M, P, files, usb, flow, lay, bolts):
    A = json.loads(json.dumps(M["A"]))
    parts = []
    for p in A["parts"]:
        n = p["name"]
        if n in REPLACED: continue
        parts.append(dict(p))
    for n in (TBN, CAR):
        parts.append(dict(name=n, role="structure", mesh=P[n]["file"], material="PETG", mass_g=round(P[n]["g"], 2), cg=np.round(P[n]["c"], 2).tolist(), tag="E",
                          geom_tag="C", owner="scout electronics", version="tail mount v3 (concept)", source="nox/scout/v3/electronics_v3.py"))
    for n in ("Radxa Zero 3W", "UBEC", "XR2"):
        parts.append(dict(name=n, role="bought", protect=True, mesh=P[n]["file"], mass_g=P[n]["g"], cg=np.round(P[n]["c"], 2).tolist(), tag=P[n]["tag"], geom_tag="C",
                          note="as fitted by the tail mount v3 (the Radxa: the layout's box less the 4 post keep-outs, + its plugs' keep-clear)"))
    cb = CABLE_BOX
    parts.append(dict(name=CABLES, role="structure", box=list(cb), mass_g=round(P[CABLES]["g"], 2), cg=np.round(P[CABLES]["c"], 2).tolist(), tag="E",
                      note="the tail mount v3's 5 cable runs at their CG; for the fit a 0.4 box beside the fin (their CG sits inside the FC's air)"))
    parts.append(dict(name=FCN, role="bought", protect=True, mesh=files[FCN], mass_g=M_FC, cg=np.round(P[FCN]["c"], 2).tolist(), tag="P", geom_tag="C",
                      note=f"MicoAir743v2-AIO 36 x 36 x 8, the diamond at (1.9, 0) (P), hard on 3 standoffs, z {lay['env'][0]:.1f}..{lay['env'][1]:.1f}"))
    parts.append(dict(name=STO, role="bought", mesh=files[STO], mass_g=round(P[STO]["g"], 2), cg=np.round(P[STO]["c"], 2).tolist(), tag="E", geom_tag="C",
                      note=P[STO]["note"]))
    b_ = WIRES["box"]
    parts.append(dict(name="wires", role="structure", box=[b_[0][0], b_[1][0], b_[0][1], b_[1][1], b_[0][2], b_[1][2]], cg=WIRES["cg"], mass_g=20.0, tag="E",
                      note=WIRES["note"]))
    parts.append(dict(name=FLW, role="bought", protect=True, mesh=files[FLW], mass_g=1.5, tag="P", geom_tag="C"))
    parts.append(dict(name=BM, role="structure", mesh=files[BM], material="PETG", mass_g=round(P[BM]["g"], 2), cg=np.round(P[BM]["c"], 2).tolist(),
                      tag="E", geom_tag="C", owner="scout electronics", version="belly mount v3 (concept)", source="nox/scout/v3/belly_mount_v3.py"))
    A["parts"] = parts
    names = {p["name"] for p in parts}
    cp = [c for c in A["contact_pairs"] if set(c[:2]) <= names and not (set(REPLACED) & set(c[:2])) and frozenset(c[:2]) not in CONTACT]
    cp += [[a, b, 0.05] for a, b in sorted(tuple(sorted(x)) for x in CONTACT)]
    A["contact_pairs"] = cp
    A["clearance_pairs"] = ([[BM, n, GAP] for n in ("XT60E1-M", "tie saddle (printed)", "receiver TPU end pieces (latch, E)", "cartridge v6 (latch)")]
                            + [[FLW, n, GAP] for n in ("receiver v3 (latch)", "tie saddle (printed)")]
                            + [[FCN, n, GAP] for n in (PLATE, TBN, CABLES, "wires")]
                            + [[STO, TBN, GAP]])
    acc = []
    for a in A["access"]:
        if a["name"].startswith(("FC USB-C", "M2 key, FC grommet", "M3 key, belly-mount plate bolt", "flow SH1.0")): continue
        a = dict(a); a["ignore"] = [i for i in a.get("ignore", []) if i != "wires"] + ["wires"]; acc.append(a)
    w = np.cross([0, 0, 1.0], usb["d"]); w /= np.linalg.norm(w)
    acc.append({"name": f"FC USB-C cable (12 x 7, the tail corner's {usb['side']} edge, P, E)", "part": FCN, "from": np.round(usb["p0"], 2).tolist(),
                "dir": np.round(usb["d"], 4).tolist(), "length_mm": USB_L, "tool": {"type": "box", "w": USB_W, "h": USB_H, "w_axis": np.round(w, 4).tolist()},
                "ignore": ["wires"]})
    for hx, hy in lay.get("holes", holes_xy(*lay["c"], lay["yaw"])):
        acc.append({"name": f"M3 key (2.0 hex), FC button screw ({hx:+.2f}, {hy:+.2f}) from the top", "part": FCN, "from": [hx, hy, lay["pcb"][1] + HEAD_H + 0.5],
                    "dir": [0, 0, 1], "length_mm": 150.0, "tool": {"type": "cylinder", "d": 4}, "ignore": ["wires"]})
    for bx, by in bolts:
        acc.append({"name": f"M3 key, belly-mount plate bolt ({bx:g}, {by:+g}) from the top", "part": BM, "from": [bx, by, PLATE_TOP + 0.5],
                    "dir": [0, 0, 1], "length_mm": 150.0, "tool": {"type": "cylinder", "d": 4}, "ignore": ["wires"]})
    env = json.load(open(os.path.join(ELEC, "envelope.json")))          # the tail mount v3's own keys and tool lines
    swap = [n for n, p_ in P.items() if p_.get("spec", {}).get("swap")]
    for nm, k in env["keep_clear"]["keys"].items():
        acc.append({"name": nm, "part": TBN if "plate bolt" in nm else CAR, "from": k["start"], "dir": k["dir"], "length_mm": 150.0,
                    "tool": {"type": "cylinder", "d": k["d"]}, "ignore": ["wires", CABLES] + (swap if k["dir"][2] < 0 else [])})
    for nm, k in env["keep_clear"]["tool_lines"].items():
        acc.append({"name": f"Radxa {nm} (Ø{k['d']:.0f} tool)", "part": "Radxa Zero 3W", "from": k["start"], "dir": k["dir"], "length_mm": 150.0,
                    "tool": {"type": "cylinder", "d": k["d"]}, "ignore": ["wires", CABLES]})
    fx, fy = flow["c"]; hx_ = FLOW_L / 2; pe = fx - hx_ if flow["ori"][1] == "-" else fx + hx_
    acc.append({"name": "flow SH1.0 plug out of its end (6 x 4, E)", "part": FLW, "from": [pe, fy, flow["zb"] + 3.25],
                "dir": [-1.0 if flow["ori"][1] == "-" else 1.0, 0, 0], "length_mm": 25.0, "tool": {"type": "box", "w": 6.0, "h": 4.0}, "ignore": ["wires"]})
    A["access"] = acc
    A["name"] = "Scout v3 (layout v13) with the FC on top (hard on 3 standoffs), the belly mount v3 and the tail mount v3 (concept)"
    A["note"] = ("main's Scout v3 assembly (layout v13, catch 2.5 aft, camera mount v1) with the FC on the plate top hard on 3 M3 standoffs over the front and "
                 "side AIO holes, the belly mount v3, the flow re-placed, the tail mount v3 (tail block + carrier, nox/scout/v3/electronics_v3.py) and its items as fitted")
    return A


def run_system_check(man):
    from system_check import run, report
    r = run(man); return r, report(r)


# ---------------------------------------------------------------- 8. pictures
COLS = {BM: "#2f7fd8", FCN: "#2e7d32", STO: "#00838f", TRAY: "#0097a7", PEDS: "#00838f", DMP: "#d84315", FLW: "#1b5e20", PLATE: "#3a3a3a", TBN: "#8e6bbf",
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
    """the plate's outline (C mesh) and the same with the battery-lead notch cut (inner edge x 42.8, |y| 6.2, corners r 1.5)"""
    import shapely.geometry as sg
    s = P[PLATE]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
    pg = sg.MultiPolygon([sg.Polygon(e[:, :2]) for e in s.discrete]).buffer(0)
    notch = sg.box(NOTCH_X0 + NOTCH_R, -NOTCH_Y + NOTCH_R, 80.0, NOTCH_Y - NOTCH_R).buffer(NOTCH_R)
    return pg, pg.difference(notch)


def _plate_patch(ax, pgn, a=0.25, z=1):
    from matplotlib.patches import Polygon as MPoly
    for g in getattr(pgn, "geoms", [pgn]):
        ax.add_patch(MPoly(np.array(g.exterior.coords), closed=True, facecolor="#3a3a3a", alpha=a, edgecolor="#3a3a3a", zorder=z))


def preview(P, M, info, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, Arc
    rng = np.random.default_rng(0)
    fig = plt.figure(figsize=(21, 19.5)); gs = fig.add_gridspec(3, 3, height_ratios=[1, 1.05, 0.95])
    views = [("side (from +y): x, z", (0, 2), 1), ("top: x, y", (0, 1), 1), ("rear (from -x): y, z", (1, 2), -1)]
    lims = {(0, 2): ((-80, 110), (-58, 72)), (0, 1): ((-80, 110), (-72, 72)), (1, 2): ((-75, 75), (-58, 72))}
    flow = info["flow"]; usb = info["usb"]; lay = info["lay"]
    pg, pgn = plate_outline(P)
    for k, (vt, (i, j), flip) in enumerate(views):
        ax = fig.add_subplot(gs[0, k]); ax.set_aspect("equal"); ax.set_title(f"FC on top + belly mount v3 in the Scout v3, {vt}", fontsize=10)
        for n, p in P.items():
            if n in SKIP_DRAW or n.startswith("prop"): continue
            if (i, j) == (0, 1) and n == PLATE: _plate_patch(ax, pgn, 0.3, 1); continue
            c = COLS.get(n, "#4f6fcf" if p["spec"].get("swap") else "#a8b3bf")
            if n == BM: _poly(ax, p["mesh"], i, j, flip, c, "#174a85", 0.75, 6, 0.2, rng)
            elif n in (FCN, FLW, DMP, TRAY, PEDS): _poly(ax, p["mesh"], i, j, flip, c, "k", 0.85, 7, 0.3, rng)
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
    # close-up: section at y 0 (x, z): the board hard on its front standoff, over the tail block's foot as drawn
    ax = fig.add_subplot(gs[1, 0]); ax.set_aspect("equal"); ax.set_title("close-up: the FC hard on 3 standoffs, section at y 0 (x, z)", fontsize=10)
    for n, a in ((PLATE, 0.8), (TBN, 0.5), ("receiver v3 (latch)", 0.35), (BM, 0.75), (STO, 0.9), (FCN, 0.8), ("XT60E1-M", 0.6), ("receiver lid (latch)", 0.35)):
        _sect(ax, P[n]["mesh"], [0, 0.01, 0], [0, 1, 0], (0, 2), COLS.get(n, "#a8b3bf"), a)
    for hx, hy in lay["holes"]:
        if abs(hy) > 1: ax.add_patch(Rectangle((hx - STO_D / 2, PLATE_TOP), STO_D, lay["pcb"][0] - PLATE_TOP, fill=False, ec="#00838f", ls="--", lw=1.0, zorder=6))
    ax.add_patch(Rectangle((lay["c"][0] - FC_W / 2 * math.sqrt(2), lay["pcb"][0]), FC_W * math.sqrt(2), PCB_T, color="#1b5e20", zorder=8))
    ax.axhspan(PROP_Z - 4, PROP_Z + 4, color="#e8c547", alpha=0.2, zorder=0); ax.text(-58, PROP_Z + 1.0, f"prop sweep (the board {info['fc_plan']:.1f} outside the disks in plan)", fontsize=7, color="#a06800")
    hf = info["hard"]
    ax.text(-58, 76, f"3 M3 male-female aluminium standoffs 5.5 AF x {lay['L']:.1f} (E) on the front and side AIO holes (dashed: the side pair)", fontsize=7.3)
    ax.text(-58, 72, "studs into M3 nuts bonded under the plate; M3 screws down through the PCB; the rear hole stays empty (under the foot)", fontsize=7.3)
    ax.text(-58, 68, f"rigid modes {hf['rigid'][0][0]:.0f}..{hf['rigid'][-1][0]:.0f} Hz (E); the PCB's free tail corner ~{hf['plate_Hz']:.0f} Hz (E), the IMU on its hinge line", fontsize=7.3)
    ax.text(-58, 64, f"the board's lower components {hf['foot_gap']:.1f} over the foot as drawn; filtering: bidirectional DShot + RPM notch + FFT", fontsize=7.3)
    ax.set_xlim(-60, 60); ax.set_ylim(0, 80); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up: the plate top (x, y): the notch, FC, isolator, foot, routes with their loops
    ax = fig.add_subplot(gs[1, 1]); ax.set_aspect("equal"); ax.set_title("close-up: the plate top (notch drawn), wiring routes and ties (E)", fontsize=10)
    _plate_patch(ax, pgn, 0.25, 1)
    for mx, my in DISKS: ax.add_patch(Circle((mx, my), DISK_R, color="#e8c547", alpha=0.12, zorder=0)); ax.add_patch(Circle((mx, my), 12.4, color="#777", alpha=0.6, zorder=2))
    _sect(ax, P[TBN]["mesh"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "#8e6bbf", 0.6)
    _sect(ax, info["tb_orig"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "none", 1.0, "#8e6bbf", 4, 0.9, "--")
    _sect(ax, P[STO]["mesh"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), COLS[STO], 0.9, "k", 5)
    _sect(ax, P[FCN]["mesh"], [0, 0, lay["pcb"][0] + 0.5], [0, 0, 1], (0, 1), "#2e7d32", 0.55, "k", 6)
    for hx, hy in holes_xy(*lay["c"], lay["yaw"]): ax.plot(hx, hy, "o", ms=4, mfc="w", mec="k", zorder=9)
    for bx, by in info["bolts"]: ax.plot(bx, by, "s", ms=6, mfc="#ffb300", mec="k", zorder=9)
    c = np.array(lay["c"])
    for u in ((1, 1), (1, -1), (-1, 1), (-1, -1)):                      # motor leads: board corner to the motor, down onto the arm top, a loop
        a0 = c + np.array([0.0 if u[0] * u[1] else 0, 0]); p_ = np.array([54.1 * u[0], 54.1 * u[1]]) * 0.62
        ax.plot([c[0] + 12 * u[0], p_[0]], [12 * u[1], p_[1]], color="#6d4c41", lw=2.0, zorder=7)
        ax.add_patch(Circle((c[0] + 15 * u[0], 15 * u[1]), 3.2, fill=False, ec="#6d4c41", lw=1.2, zorder=8))
    for cx, cy in XT60_CUPS: ax.plot([cx, cx - 14, c[0] + 26], [cy, cy, cy * 0.4], color="#b71c1c" if cy > 0 else "#212121", lw=3.0, zorder=7)
    ax.add_patch(Arc((c[0] + 33, 0), 10, 14, theta1=90, theta2=270, color="#b71c1c", lw=1.5, zorder=8))
    ax.plot([24, 46, 46], [-30.5, -30.5, -1.0], color="#1b5e20", lw=1.4, ls="--", zorder=7)
    for yy in (-12.0, 12.0): ax.plot([c[0] - 22, -40.0, -64.0], [yy * 0.6, yy, yy], color="#5e35b1", lw=1.6, zorder=7)
    q = usb["p0"] + 40 * usb["d"]
    ax.annotate("", xy=tuple(q[:2]), xytext=tuple(usb["p0"][:2]), arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.8), zorder=10)
    ax.text(q[0] - 2, q[1] - 4, "USB-C", color="#c62828", fontsize=7.5, zorder=10)
    ax.text(47.5, 8.5, "lead\nnotch", fontsize=7, ha="center", zorder=10); ax.text(c[0], 0, "FC\n(diamond, on\n3 standoffs)", color="w", fontsize=8, ha="center", va="center", zorder=10)
    ax.text(-44, 15, "Radxa UART, XR2,\nUBEC in (to the bay)", fontsize=7, color="#5e35b1", zorder=10)
    ax.text(26, -28, "flow lead (under the plate,\nup the notch between\nthe battery leads)", fontsize=6.5, color="#1b5e20", zorder=10)
    ax.text(-33, 3, "foot (as drawn)" if info["trim"] == FOOT_X else f"foot\n(tip to x {info['trim']:.0f})", fontsize=7, ha="center", color="#5e35b1", zorder=10)
    ax.text(-68, -56, "circles / arcs: tie points and strain relief (the hard mount needs no isolation loops)", fontsize=7, zorder=10)
    ax.set_xlim(-70, 70); ax.set_ylim(-60, 60); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # close-up: the belly mount slice at z 14 (top view)
    ax = fig.add_subplot(gs[1, 2]); ax.set_aspect("equal"); ax.set_title(f"close-up: belly mount v3 ({info['pick']}), slice at z 14 (top view)", fontsize=10)
    _sect(ax, P["receiver v3 (latch)"]["mesh"], [0, 0, 9.0], [0, 0, 1], (0, 1), "#8899aa", 0.25, "#667", 1)
    for n in (TBN, "XT60E1-M", "tie saddle (printed)", BM):
        _sect(ax, P[n]["mesh"], [0, 0, 14.0], [0, 0, 1], (0, 1), COLS.get(n, "#a8b3bf"), 0.85 if n == BM else 0.45, "k", 5 if n == BM else 3)
    for x_, y_ in REC_BOLTS: ax.text(x_, y_ - 8.0 if y_ < 0 else y_ + 6.0, "rcv M3", fontsize=6.5, ha="center", zorder=9)
    for bx, by in info["bolts"]: ax.text(bx, by + (6.5 if by > 0 else -8.5), "plate M3", fontsize=6.5, ha="center", color="#c62828", zorder=9)
    ax.text(-28, 0, "tail\nface", fontsize=7, ha="center", zorder=9); ax.text(48, 0, "XT60", fontsize=7, ha="center", zorder=9)
    ax.set_xlim(-50, 60); ax.set_ylim(-55, 45); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    # bottom row: the bipod slice, the isolator chart, the numbers
    ax = fig.add_subplot(gs[2, 0]); ax.set_aspect("equal"); py = flow["post_y"]
    ax.set_title(f"close-up: the flow bipod, slice at y {py:+.1f} and at the flow (x, z)", fontsize=10)
    for n in (BM, FLW, "receiver v3 (latch)", "tie saddle (printed)", PLATE, "cartridge v6 (latch)"):
        for yy in (py, flow["c"][1]): _sect(ax, P[n]["mesh"], [0, yy, 0], [0, 1, 0], (0, 2), COLS.get(n, "#a8b3bf"), 0.6 if n in OURS else 0.25, "k", 5 if n in OURS else 3)
    t_ = math.tan(math.radians(FLOW_HALF)); zl = flow["zb"]; fx = flow["c"][0]
    for wx in (fx - FLOW_WIN, fx + FLOW_WIN):
        for s_ in (-1, 1): ax.plot([wx, wx + s_ * (zl - PLANE + 8) * t_], [zl, PLANE - 8], color="#43a047", lw=0.9)
    ax.axhline(PLANE, color="#555", lw=0.6, ls=":"); ax.axhline(PLANE + BAND, color="#c62828", lw=0.7, ls="--"); ax.text(-48, PLANE + BAND + 1, "funnel band top", fontsize=7, color="#c62828")
    bp = info["bipod"]
    ax.text(-48, 24, f"bipod 6 x 6 PETG, {bp['strut_L']} long; flow CG {abs(bp['offset_y']):.1f} off the strut plane (the twist)", fontsize=7)
    ax.set_xlim(-50, 60); ax.set_ylim(-62, 30); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    ax = fig.add_subplot(gs[2, 1]); b = info["bands"]; md = info["iso"]; hf = info["hard"]
    ax.set_title("FC mount modes against the rotor (E): H-high (picked, lines), isolator B (alternative, band)", fontsize=10)
    rr = [r for r in info["iso_rows"] if r["dmp"] == md["dmp"] and r["shore"] == md["shore"] and r["r"] == md["r"]]
    ax.fill_between([r["th"] for r in rr], [r["f_lo"] for r in rr], [r["f_hi"] for r in rr], color="#0097a7", alpha=0.35, label=f"B: {md['dmp']} {md['shore']}A, r {md['r']}, by inclination")
    ax.axhspan(b["flight_1P"][0], b["flight_1P"][1], color="#ff8a65", alpha=0.15, label=f"rotor 1P in flight {b['flight_1P'][0]}..{b['flight_1P'][1]} (E)")
    ax.axhspan(b["blade_pass_flight"][0], b["blade_pass_flight"][1], color="#e8c547", alpha=0.2, label=f"blade pass {b['blade_pass_flight'][0]}..{b['blade_pass_flight'][1]} (E)")
    ax.axhline(100, color="#c62828", lw=0.8); ax.text(1, 104, "gate 100", fontsize=7, color="#c62828")
    ax.axhline(b["iso_ceiling"], color="#c62828", lw=0.8, ls="--"); ax.text(1, b["iso_ceiling"] * 1.04, f"B's ceiling {b['iso_ceiling']:.0f}", fontsize=7, color="#c62828")
    ax.axhline(hf["plate_Hz"], color="#2e7d32", lw=1.6); ax.text(42, hf["plate_Hz"] * 1.05, f"H-high: the PCB's tail corner ~{hf['plate_Hz']:.0f}", fontsize=7.5, color="#2e7d32")
    ax.axhline(hf["rigid"][0][0], color="#2e7d32", lw=1.6, ls=":"); ax.text(42, hf["rigid"][0][0] * 1.05, f"H-high: rigid on the standoffs {hf['rigid'][0][0]:.0f}+", fontsize=7.5, color="#2e7d32")
    ax.axvline(md["th"], color="k", lw=0.8, ls=":")
    ax.set_yscale("log"); ax.set_xlabel("B's mount inclination from vertical (deg)", fontsize=8); ax.set_ylabel("Hz", fontsize=8); ax.set_ylim(60, 20000)
    ax.legend(fontsize=7, loc="upper left"); ax.grid(alpha=0.2); ax.tick_params(labelsize=7)
    ax = fig.add_subplot(gs[2, 2]); ax.axis("off")
    ax.text(0.0, 1.0, "\n".join(info["notes"]), va="top", ha="left", fontsize=8.0, family="monospace", transform=ax.transAxes)
    fig.text(0.01, 0.004, "blue: belly mount v3 (one PETG print); green: FC (on the plate top) and flow; teal: the FC's 3 aluminium standoffs; "
             "violet: the tail block (as drawn) and carrier; orange: tie saddle; grey: receiver set; dark blue: "
             "swap set; black: cage rods; yellow: swept props; red arrow: FC USB-C path; the plate drawn with its lead notch", fontsize=8)
    fig.tight_layout(rect=(0, 0.012, 1, 1)); fig.savefig(path, dpi=100); plt.close(fig)


def preview_alternatives(P, M, info, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MPoly
    import shapely.geometry as sg, shapely.affinity as sa
    fig, axs = plt.subplots(2, 4, figsize=(23, 13.5))
    pg, pgn = plate_outline(P)
    def note(ax, lines, y0=-0.08):
        ax.text(0.0, y0, "\n".join(lines), transform=ax.transAxes, fontsize=7.8, va="top", ha="left",
                bbox=dict(facecolor="white", alpha=0.9, edgecolor="#999", lw=0.5), zorder=20)
    for k, (key, o) in enumerate(info["fc_options"].items()):
        ax = axs[0, k]; _plate_patch(ax, pgn, 0.18, 1)
        for mx, my in DISKS: ax.add_patch(Circle((mx, my), DISK_R, color="#e8c547", alpha=0.15, zorder=0))
        _sect(ax, o["tb"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "#8e6bbf", 0.6)
        sq = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), o["yaw"], origin=(0, 0)), o["x"], 0)
        ax.add_patch(MPoly(np.array(sq.exterior.coords), closed=True, color="#2e7d32", alpha=0.6, zorder=5))
        if o.get("peds") is not None: _sect(ax, o["peds"], [0, 0, PLATE_TOP + 1.0], [0, 0, 1], (0, 1), "#00838f", 0.9, "k", 6)
        for hx, hy in holes_xy(o["x"], 0, o["yaw"]): ax.plot(hx, hy, "o", ms=5, mfc="#d84315" if o["aio"] else "w", mec="k", zorder=9)
        for hx, hy in holes_xy(*FC_C, FC_YAW): ax.plot(hx, hy, "x", color="#c62828", ms=5, zorder=9)
        ax.set_xlim(-65, 70); ax.set_ylim(-62, 62); ax.set_aspect("equal"); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
        ax.set_title(f"{key}{' (picked)' if o['picked'] else ''}: {o['what']}", fontsize=9.5); note(ax, o["lines"])
    for k, (v, d) in enumerate(info["bolt_variants"].items()):
        ax = axs[1, k]
        _sect(ax, P["receiver v3 (latch)"]["mesh"], [0, 0, 9.0], [0, 0, 1], (0, 1), "#8899aa", 0.25, "#667", 1)
        _sect(ax, d["mesh"], [0, 0, 14.0], [0, 0, 1], (0, 1), "#2f7fd8", 0.8, "k", 4)
        for bx, by in VARIANTS[d["variant"]]["bolts"]: ax.plot(bx, by, "s", ms=6, mfc="#c62828" if (bx, by) in VARIANTS[d["variant"]]["new"] else "#ffb300", mec="k", zorder=9)
        ax.set_xlim(-50, 58); ax.set_ylim(-55, 42); ax.set_aspect("equal"); ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
        ax.set_title(f"belly {v}{' (picked)' if d['picked'] else ''}", fontsize=9.5); note(ax, d["lines"])
    ax = axs[1, 3]; ax.axis("off"); note(ax, info["alt_text"], 1.0)
    fig.suptitle("Scout v3 FC on top + belly mount v3 round 3: the options from the same system model (C on E). Top: where the FC sits, H-high picked, "
                 "B the alternative (red x: the AIO holes, the diamond at (1.9, 0), P; teal: standoffs or isolator feet); bottom: how the belly mount bolts on (yellow: the Bee35's own holes, red: new)", fontsize=11)
    fig.subplots_adjust(left=0.03, right=0.99, top=0.92, bottom=0.13, wspace=0.16, hspace=0.45); fig.savefig(path, dpi=95); plt.close(fig)


# ---------------------------------------------------------------- 9. the pass
CRASH_TARGET = 1.0 / 1.5                  # DEFAULT: crash worst <= 1 / 1.5 (the L breaking order against an index-1.0 node; node indices
                                          # are not computed, and the 3.0 rods fail at 35 g: rods are the fuse, Spark 27 Oct)
G_PER_W = (4.0, 5.0)                      # hover efficiency of a 3.5 in quad, g of thrust per W (E); 4S at 14.8 V
AIO_LOSS_W = (2.0, 4.0)                   # the AIO's heat at hover current: 4-in-1 ESC switching + FC + regulators (E)
H_CONV = dict(still=10.0, hover_inflow=30.0, forward_8ms=58.0)   # W/m2K (E: still air; ~3 m/s inflow near the disk edge; 8 m/s, flat plate)


def ser(x):
    if isinstance(x, (np.floating, np.integer)): return x.item()
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, (np.bool_,)): return bool(x)
    if isinstance(x, trimesh.Trimesh): return "<mesh>"
    return str(x)


def tail_up_couple():
    """the tail block's peak tail-up couple from its own run (runs/scout_electronics/v3/result.json, concept A, the L cases): the foot
    tip bears on the plate top at tip + 2 against the web under the tab at x -42"""
    r = json.load(open(os.path.join(ELEC, "result.json")))
    cs = r["concepts"]["A"]["loads"]["cases"]
    best = max(((v["plate bolts (2 M3)"]["tail_up_couple_N"], k, v.get("foot", {}).get("index", 0.0)) for k, v in cs.items()
                if k.startswith(("crash", "flight", "landing"))), key=lambda t: t[0])   # the L cases only
    return dict(F=best[0], case=best[1], foot_index=best[2])


def run():
    t0 = time.time(); os.makedirs(RUN, exist_ok=True)
    say("Scout v3 belly mount v3 + the FC on top + the tail mount v3 (28 Oct 2026): system pass on main's assembly (layout v13, catch 2.5 aft, camera mount "
        f"v1) with the tail mount v3 (runs/scout_electronics/v3/); everything on the catch moved {DX:+.1f} in x against v12, the plate, the frame, the props and "
        "the FC on top fixed. Spark 27 Oct: the FC on top of the plate; every rod 3.0. Round 3's FC pick (review of round 2) carries over")
    M = load_model(); P = M["P"]
    tb = P[TBN]["mesh"]
    # --- 0. the rotor
    M_ALL = float(json.load(open(os.path.join(LAYOUT, "result.json")))["mass"])        # v13's all-up mass (E)
    bands = rotor_bands(M_ALL)
    say(f"rotor (E: C_T {PROP_CT[0]}..{PROP_CT[1]}, motor KV not recorded): hover thrust {bands['thrust_N']} N a motor at {M_ALL:.0f} g (v13, E); 1P hover {bands['hover_1P']} Hz, "
        f"in flight ({FLIGHT_T[0]}..{FLIGHT_T[1]} x hover thrust) {bands['flight_1P']}; blade pass ({BLADES} blades) hover {bands['blade_pass_hover']}, flight {bands['blade_pass_flight']}")
    # --- 1. the DEFAULT: H-high, the board hard on 3 standoffs over the front and side AIO holes
    hl = hard_layout(tb); hf = hard_modes(hl); hf["foot_gap"] = hl["foot_gap"]
    say(f"FC (H-high, DEFAULT): the diamond at (1.9, 0) (P) hard on 3 M3 male-female aluminium standoffs 5.5 AF x {hl['L']:.2f} (E) over the AIO holes "
        f"{hl['holes']} (the rear one, under the tail block's foot, stays empty): the shortest that keeps the board {hl['foot_gap']:.2f} off the tail block as drawn; "
        f"PCB z {hl['pcb'][0]:.2f}..{hl['pcb'][1]:.2f}, board z {hl['env'][0]:.1f}..{hl['env'][1]:.1f}")
    say(f"FC modes (E): rigid on the standoffs " + ", ".join(f"{n} {f:.0f}" for f, n in hf["rigid"]) + f" Hz (k axial {hf['k_axial']}, transverse {hf['k_trans']} N/mm a "
        f"standoff, threads at 1/10); the PCB's free tail corner about {hf['plate_Hz']} Hz (E, a uniform strip: the taper raises it; the IMU at the board centre sits on "
        f"its hinge line): every mode >= 100 (gate PASS); the hard mount isolates nothing, the frame's and the rotor's tones reach the IMU: {FILTERING}")
    say("the hard mount's numbers in round 2 (a standoffs-only lateral mode of 14.5 kHz) are dropped: the PCB's own mode is the lower one")
    # --- 2. the alternative B: the isolator (built only if the first-hover vibration log fails)
    rows_i, ip, vbest = iso_search(bands)
    lay = iso_layout(FC_C, FC_YAW, ip["dmp"], ip["th"], float(ip["r"])); ms = iso_meshes(lay)
    tray_g = printed_g(ms["tray"])[0]; peds_g = printed_g(ms["pedestals"])[0]
    md0, kk, mg, mcg = iso_modes(lay, ip["shore"], tray_g, ms["tray"])
    mdl, _, _, _ = iso_modes(lay, ip["shore"], tray_g, ms["tray"], extra=lead_springs(lay))
    sens = {}
    for f_ in (0.8, 1.2):
        Dd = DAMPERS[ip["dmp"]]; kk_ = kcol(GENT(ip["shore"]) / 3.0 * f_, Dd["h"], Dd["d"]); m_, cg_, I_, _ = moving_set(lay, tray_g, ms["tray"])
        sens[f_] = rigid_modes([(o["mid"], o["d"], kk_["a"], kk_["s"], kk_["t"]) for o in lay["mounts"]] + lead_springs(lay), m_, cg_, I_)
    D = DAMPERS[ip["dmp"]]; b_extra = tray_g + peds_g + 4 * D["g"] + 8 * (SCREW_G + NUT_G)
    k_horiz = 2 * (kk["a"] * math.sin(math.radians(ip["th"])) ** 2 + kk["s"] * math.cos(math.radians(ip["th"])) ** 2) + 2 * kk["s"]
    lead_k = sum(s_[2] for s_ in lead_springs(lay))
    say(f"alternative B (isolator, a layout sketch: the mounts' stud bores and the tray's screw holes are not drawn): {ip['dmp']} {ip['shore']}A at {ip['th']} deg, "
        f"radius {ip['r']}; modes with the leads " + ", ".join(f"{n} {f:.0f}" for f, n in mdl) + f" Hz, margin {ip['margin']:.2f} to both bounds of "
        f"[{F_FLOOR:.0f}, {bands['iso_ceiling']}]; rubber -20 / +20 %: {sens[0.8][0][0]:.0f}..{sens[0.8][-1][0]:.0f} / {sens[1.2][0][0]:.0f}..{sens[1.2][-1][0]:.0f} Hz; "
        f"horizontal stiffness {k_horiz:.1f} N/mm along a board axis; the leads with their loops {lead_k:.2f} N/mm; +{b_extra:.0f} g against H-high; the board at "
        f"z {lay['env'][0]:.0f}..{lay['env'][1]:.0f}; vertical columns (round 1) at best {vbest['f_lo']:.0f}..{vbest['f_hi']:.0f} Hz: no fit")
    # --- 3. the pose options, the plan margins, the roof
    import shapely.geometry as sg, shapely.affinity as sa
    fc_poly = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), FC_YAW, origin=(0, 0)), *FC_C)
    fc_plan = plan_margin_poly(fc_poly); roof = roof_limit(M, fc_poly.buffer(6))
    sto_plan = plan_margin_poly(sg.MultiPoint([tuple(h) for h in hl["holes"]]).buffer(STO_D / 2))
    say(f"FC plan: the board {fc_plan:.2f} outside the swept disks (rule 8 >= 3 PASS), the standoffs {sto_plan:.2f}; roof over it (+3): "
        + ("no cage member" if not np.isfinite(roof[0]) else f"z {roof[0]:.1f} {roof[1]}") + f"; the board top {hl['env'][1]:.1f} vs the prop plane {PROP_Z}")
    def option_iso(x, yaw, aio, what):
        l_ = iso_layout((x, 0.0), yaw, ip["dmp"], ip["th"], float(ip["r"])); m_ = iso_meshes(l_); f_ = fc_mesh(x, 0.0, yaw, dict(pcb=l_["pcb"], env=l_["env"]))
        tr = find_trim(tb, [f_, m_["dampers"], m_["pedestals"], m_["tray"]]); poly = sa.translate(sa.rotate(sg.box(-FC_W / 2, -FC_W / 2, FC_W / 2, FC_W / 2), yaw, origin=(0, 0)), x, 0)
        pp = plan_margin_poly(sg.MultiPolygon([sg.Polygon(e[:, :2]) for e in m_["pedestals"].section(plane_origin=[0, 0, PLATE_TOP + 1], plane_normal=[0, 0, 1]).discrete]).buffer(0))
        return dict(x=x, yaw=yaw, aio=aio, what=what, trim=tr, plan_fc=round(plan_margin_poly(poly), 2), plan_peds=round(pp, 2), peds=m_["pedestals"],
                    tb=trimmed_tail_block(tb, tr), picked=False, env=l_["env"])
    options = {}
    options["H"] = dict(x=FC_C[0], yaw=FC_YAW, aio=True, what="H-high (picked): the diamond hard on 3 standoffs\nover the front and side AIO holes",
                        trim=FOOT_X, plan_fc=round(fc_plan, 2), plan_peds=round(sto_plan, 2), peds=hl["sto"], tb=tb, picked=True, env=hl["env"])
    options["B"] = option_iso(FC_C[0], FC_YAW, True, "B (alternative, if the vibration log fails):\nthe inclined isolator, a layout sketch")
    options["A"] = option_iso(FC_C[0], 0.0, False, "A: the board square (yaw 0), isolator,\n4 new holes: the mounts on the plate axes")
    low = fc_mesh(*FC_C, FC_YAW, fc_stack_pcb(PLATE_TOP + GAP + COMP + 0.1)); tr_low = find_trim(tb, [low])
    options["L"] = dict(x=FC_C[0], yaw=FC_YAW, aio=True, what="H-low (round 2's H): the diamond on short\nstandoffs, its tail corner on the foot",
                        trim=tr_low, plan_fc=round(fc_plan, 2), plan_peds=None, peds=None, tb=trimmed_tail_block(tb, tr_low), picked=False,
                        env=fc_stack_pcb(PLATE_TOP + GAP + COMP + 0.1)["env"])
    lev0 = FOOT_X + 2 + FOOT_LEVER0
    for k, o in options.items():
        lv = o["trim"] + 2 + FOOT_LEVER0
        o["lines"] = [f"board plan {o['plan_fc']:.2f} outside the swept disks" + (f"; feet {o['plan_peds']:.2f}" if o["plan_peds"] is not None else ""),
                      f"tail block foot tip x {o['trim']:.1f}" + (" (as drawn)" if o["trim"] == FOOT_X else f" (trim; couple x {lev0 / lv:.2f})"),
                      f"holes: {'the AIO ones (none new)' if o['aio'] else '4 new M3 in the carbon'}; board z {o['env'][0]:.0f}..{o['env'][1]:.0f}"]
    options["H"]["lines"].append(f"modes: rigid {hf['rigid'][0][0]:.0f}+ Hz, PCB tail corner ~{hf['plate_Hz']} (E); filtering in PX4")
    options["B"]["lines"].append(f"modes {mdl[0][0]:.0f}..{mdl[-1][0]:.0f} Hz in [{F_FLOOR:.0f}, {bands['iso_ceiling']:.0f}], margin {ip['margin']:.2f}; +{b_extra:.0f} g")
    options["A"]["lines"].append("modes as B")
    options["L"]["lines"].append("modes as H-high; needs the foot trimmed")
    for k, o in options.items(): say(f"FC option {k}: {o['what']}: " + "; ".join(l for l in o["lines"] if l).replace(chr(10), " "))
    tu = tail_up_couple()
    say(f"tail block foot (the tail mount v3, its tip plate-fixed at x {FOOT_X}): its tail-up couple ({tu['case']}) {tu['F']:.1f} N (foot index {tu['foot_index']}); "
        f"the rear AIO hole (-16.13, 0) lies under the foot: a 4th FC post or a foot bolt there is weighed in the summary, not drawn")
    # --- 4. the model with the FC on top
    P[FCN] = dict(mesh=hl["fc"], g=M_FC, c=np.array([FC_C[0], FC_C[1], (hl["pcb"][0] + hl["pcb"][1]) / 2]), role="bought", tag="P", spec={})
    sto_g = 3 * sum(STO_G)
    P[STO] = dict(mesh=hl["sto"], g=sto_g, c=np.asarray(hl["sto"].center_mass), role="bought", tag="E", spec={},
                  note=f"3 M3 male-female aluminium standoffs 5.5 AF x {hl['L']:.1f} (E), studs into 3 M3 nuts bonded under the plate, 3 M3 screws through the PCB ({sto_g:.1f} g)")
    P["wires"] = dict(mesh=box(*WIRES["box"]), g=20.0, c=np.array(WIRES["cg"]), role="structure", tag="E", spec={})
    cb = CABLE_BOX; P["tail bay cables (Scout electronics, E)"] = dict(P["tail bay cables (Scout electronics, E)"], mesh=box([cb[0], cb[2], cb[4]], [cb[1], cb[3], cb[5]]))
    fc_low = fc_parts(*FC_C, FC_YAW, dict(pcb=hl["pcb"], env=hl["env"]))
    # --- 5. USB-C (P: the tail corner)
    usbs = usb_search2(M, hl, skip_extra=(STO,)); u = usbs[0]
    usb = dict(side=u[1], s=u[2], az=u[3], el=u[4], d=u[5], p0=u[6], n_clear=len(usbs))
    say(f"USB-C picked: the tail corner's {usb['side']} edge, {usb['s']:.0f} from the corner, azimuth {usb['az']:+.0f}, elevation {usb['el']:+.0f}: 250 clear (C)")
    # --- 6. the flow (v1's search, seeded)
    O = v1.occupancy(M, skip=(FLW,), env=True)
    Oc = v1.occupancy(M, skip=(FLW, "cage shell nodes", "camera", "camera mount (printed)", "camera mount stop collar +y", "camera mount stop collar -y",
                               FCN, STO, "wires"), env=True, rods=False)
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
        f"(toward the nose: the lead goes up the front strut and the notch; 180 deg from as built, SENS_FLOW_ROT 4 -> 0), strut search {flow['strut_L']:.1f}, "
        f"{'inside the swap column below the funnel band' if flow['in_swap_column'] else 'above the funnel band'}; {len(scored)} poses with a clear strut")
    P[FLW] = dict(mesh=flow_mesh(flow), g=1.5, c=np.array([flow["c"][0], flow["c"][1], flow["zb"] + FLOW_T / 2]), role="bought", tag="P", spec={})
    # --- 7. the belly mount: the variants, built, loaded and their hung-set modes
    from solid_check import solid_check
    xs = np.round(np.arange(-43.5, 53.01, 1.0), 2); ys = [-15.0, -8.0, 0.0, 8.0, 15.0]
    hx_, hy_ = FLOW_L / 2, FLOW_W / 2
    cr_box = box([flow["c"][0] - hx_ - CRADLE, flow["c"][1] - hy_ - CRADLE, flow["zb"] - CRADLE], [flow["c"][0] + hx_ + CRADLE, flow["c"][1] + hy_ + CRADLE, flow["zb"] + FLOW_T + CRADLE])
    hung_lo = 100.0
    VR = {}
    for key, (v, kw) in {"VA": ("VA", {}), "VA, rails 11.6": ("VA", dict(rail_out=RAIL_Y)), "VA + front rung": ("VA", dict(front_rung=True)),
                         "VB": ("VB", {}), "VB, rails 11.6": ("VB", dict(rail_out=RAIL_Y)), "VC": ("VC", {})}.items():
        bm = belly_v2(flow, v, [], **kw); ok, sc = solid_check(bm); g_, gs_, _ = printed_g(bm)
        cradle_g = printed_g(trimesh.boolean.intersection([bm, cr_box], engine="manifold"))[0]
        sx_, sy_ = frame_sections(bm, xs, ys)
        L = ladder_loads(P, bm, g_, flow, 1.5 + cradle_g, VARIANTS[v]["bolts"], sx_, sy_)
        cw = max(L["by"]["crash"][f][0] for f in ("rec_idx", "plate_idx", "frame_x_idx", "frame_y_idx"))
        fw = max(max(L["by"][k][f][0] for f in ("rec_idx", "plate_idx", "frame_x_idx", "frame_y_idx")) for k in ("flight", "landing"))
        hm = hung_modes(v, P, kw.get("rail_out", RAIL_OUT))
        zmin = min((v_["Z"], k) for k, v_ in sx_.items() if v_ and v_["Z"] > 0)
        VR[key] = dict(variant=v, kw=kw, mesh=bm, ok=bool(ok), sc=sc, g=round(g_, 2), g_solid=round(gs_, 2), cradle_g=round(cradle_g, 2), L=L, crash=round(cw, 3),
                       fl=round(fw, 3), new=len(VARIANTS[v]["new"]), zmin=zmin, secs_x=sx_, secs_y=sy_, hung=hm, hung_ok=bool(hm[0] >= hung_lo))
        b = L["by"]["crash"]
        say(f"belly {key}: {g_:.1f} g as printed (solid {gs_:.1f}), solid_check {'PASS' if ok else 'FAIL'} (eroded bodies {sc.get('bodies_eroded')}, necks {len(sc.get('necks', []))}); "
            f"new plate holes {len(VARIANTS[v]['new'])}; hung set modes {hm[:3]} Hz (E; first {'>=' if VR[key]['hung_ok'] else '<'} {hung_lo:.0f}); "
            f"crash worst {cw:.3f}: receiver inserts {b['rec_idx'][0]:.3f}, plate inserts {b['plate_idx'][0]:.3f} (prying {b['pry'][0]:.2f} x), frame about y "
            f"{b['frame_x_idx'][0]:.3f} at x {b['frame_x_idx'][2]}, about x {b['frame_y_idx'][0]:.3f} at y {b['frame_y_idx'][2]}; flight / landing worst {fw:.3f}")
    okv = [k for k, r in VR.items() if r["ok"] and r["crash"] <= CRASH_TARGET and r["fl"] <= 0.5 and r["hung_ok"]]
    pick = min(okv, key=lambda k: (VR[k]["new"], VR[k]["g"])) if okv else max(VR, key=lambda k: VR[k]["hung"][0])
    say(f"belly pick (solid_check PASS, crash <= {CRASH_TARGET:.2f} DEFAULT, flight / landing <= 0.5, the hung set's first mode >= {hung_lo:.0f} Hz (DEFAULT: the flow "
        f"sensor rides on this frame; the frame model's bolt clamps are rigid, so E); then the fewest new plate holes, then the lightest): {pick}; its higher modes and "
        f"VC's first sit in the flight 1P band (reported; with the FC hard-mounted no isolator couples to them)")
    R = VR[pick]; bm = R["mesh"]; bm_g = R["g"]; bm_c = np.asarray(bm.center_mass, float); bolts = VARIANTS[R["variant"]]["bolts"]
    ok, sc = R["ok"], R["sc"]
    P[BM] = dict(mesh=bm, g=bm_g, c=bm_c, role="structure", tag="E", spec={})
    flow_g = 1.5 + R["cradle_g"]
    bp = bipod_frame(flow, flow_g)
    say(f"flow bipod (3D frame, E): struts {bp['strut_L']} long, the flow + cradle ({flow_g:.2f} g) {abs(bp['offset_y']):.1f} off the strut plane: crash worst "
        f"{bp['by_kind']['crash'][0]:.4f} ({bp['by_kind']['crash'][1]}), flight {bp['by_kind']['flight'][0]:.4f}, landing {bp['by_kind']['landing'][0]:.4f}; "
        f"modes {bp['modes_Hz'][:3]} Hz on the frame held rigid; with the CG in the strut plane {bp['worst_no_offset']:.4f}, {bp['modes_no_offset_Hz'][:3]} Hz")
    # --- 8. fit and system_check
    rods = rods_all(M)
    ft = fit_full(P, rods, fc_low)
    say("fit, full pairwise table (ours against every part; >= 2.0 or a declared face contact <= 0.05 mm3; '>' = box gap, a lower bound):")
    for r in ft: say(f"   {r['a']} / {r['b']}: overlap {r['overlap_mm3']:.3f} mm3, gap {'>' if r['lower_bound'] else ''}{r['gap_mm']:.2f}, {r['kind']}: {'PASS' if r['pass_'] else 'FAIL'}")
    wr = wash_report(P, [BM, FCN, STO, FLW, TBN, CAR, "Radxa Zero 3W", "UBEC", "XR2", "receiver v3 (latch)"])
    say("wash (CLAUDE.md 8: plan distance outside the swept disks, the plate's shadow left out; need >= 3; the receiver is the layout's, for reference): "
        + "; ".join(f"{n} {v['wash']}" + (f" (under 3 within {v['bad_box']})" if v["bad_box"] else "") for n, v in wr.items()))
    files = {BM: write_stl(bm, "belly_mount_v2__petg_.stl"), FCN: write_stl(hl["fc"], "fc_on_top.stl"), STO: write_stl(hl["sto"], "fc_standoffs.stl"),
             FLW: write_stl(P[FLW]["mesh"], "flow_mtf-02p.stl"), TBN: None}
    for f_ in ("fc_dampers.stl", "fc_isolator_pedestals__petg_.stl", "tail_block_foot_trimmed__requirement.stl", "fc_isolator_tray__petg_.stl",
               "fc_isolator_base__petg_.stl", "fc_mounts.stl"):          # earlier rounds' files: B is a layout sketch now, drawn in the previews only
        if os.path.exists(os.path.join(RUN, f_)): os.remove(os.path.join(RUN, f_))
    man = manifest(M, P, files, usb, flow, hl, bolts)
    json.dump(man, open(os.path.join(RUN, "manifest.json"), "w"), indent=1, default=ser)
    r, txt = run_system_check(man)
    open(os.path.join(RUN, "system_check.txt"), "w").write(txt + "\n")
    say(r["summary"])
    acc_rows = [(p["name"], p["pass_"], round(p.get("overlap_mm3", 0.0), 2), [b["part"] for b in p.get("blocked_by", [])]) for p in r["access"]["paths"]]
    for a_ in acc_rows: say(f"   access {a_[0]}: {'clear' if a_[1] else 'BLOCKED ' + str(a_[3]) + ' ' + str(a_[2]) + ' mm3'}")
    mc = r["mass_cg"]; bal = r["balance"]
    # --- 9. the ESC in free air at its height, its cooling case
    Og, lo_, pt_ = occ(M, [-60, -60, 15], [60, 60, 70], 0.5, skip=(FCN, "wires"))
    def free(pts):
        ij = np.floor((pts - lo_) / pt_).astype(int); return float(1.0 - Og[ij[:, 0], ij[:, 1], ij[:, 2]].mean())
    a0 = math.radians(FC_YAW); Rm = np.array([[math.cos(a0), -math.sin(a0)], [math.sin(a0), math.cos(a0)]])
    s_ = np.linspace(-FC_W / 2, FC_W / 2, 73); zz_ = np.linspace(hl["env"][0], hl["env"][1], 17)
    side_pts = []
    for nrm in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nv = np.array(nrm, float); tv = np.array([-nv[1], nv[0]])
        for ss in s_:
            for zz in zz_: side_pts.append(np.r_[np.array(FC_C) + Rm @ (nv * (FC_W / 2 + 3) + tv * ss), zz])
    side_open = free(np.array(side_pts))
    XX, YY = np.meshgrid(s_, s_); topp = np.array([np.r_[np.array(FC_C) + Rm @ np.array([xx, yy]), hl["env"][1] + 3] for xx, yy in zip(XX.ravel(), YY.ravel())])
    top_open = free(topp)
    i_hov = [M_ALL / g_w / 14.8 for g_w in G_PER_W[::-1]]
    area = 2 * FC_W ** 2 * 1e-6
    dT = {k: [round(w / (h * area)) for w in AIO_LOSS_W] for k, h in H_CONV.items()}
    say(f"ESC (the AIO) in free air: its 4 sides {100 * side_open:.0f} % open 3 mm out, its top {100 * top_open:.0f} %, {hl['env'][0] - PLATE_TOP:.1f} of air under it; "
        f"{fc_plan:.1f} outside the swept disks, board z {hl['env'][0]:.0f}..{hl['env'][1]:.0f} just under the prop plane ({PROP_Z}). Cooling (E): hover current "
        f"{i_hov[0]:.1f}..{i_hov[1]:.1f} A at {M_ALL:.0f} g ({G_PER_W[0]:.0f}..{G_PER_W[1]:.0f} g/W, 4S), the AIO's heat {AIO_LOSS_W[0]:.0f}..{AIO_LOSS_W[1]:.0f} W over "
        f"{area * 1e6:.0f} mm2 (both faces): rise " + ", ".join(f"{k} {v[0]}..{v[1]} K" for k, v in dT.items())
        + ": rule 8 (heat sources in airflow) holds in forward flight and, through the disks' inflow, in hover (E); still air on the bench overheats it: log the temperature")
    # --- plate holes
    edge_x = max(x for x, y in pg_exterior(P) if abs(y) < 8)
    say(f"plate: the FC's 3 standoffs on the Bee35's front and side AIO holes (the diamond at (1.9, 0), P; M3 nuts bonded under the plate; CHECK the hole size, "
        f"open to Ø3.4), the rear one empty; the belly mount on {bolts} ({pick}: new {VARIANTS[R['variant']]['new']}); the battery-lead notch x {NOTCH_X0:.1f}..{edge_x:.1f} "
        f"(the nose edge), |y| <= {NOTCH_Y:.1f}, corners r {NOTCH_R} (it takes the empty nose hole (45, 0) whole, 0.5 to spare): the leads at the cups' y ± {LEAD_OD / 2} "
        f"+ 1.0, the flow lead (Ø{FLOW_LEAD_D}) between them; a silicone edge strip on the notch's carbon edges; the cups' tops sit {PLATE_Z - 14.8:.1f} under the plate, "
        f"so the slack goes above it; the leads tied to the plate and the standoffs (strain relief only: no isolation loops)")
    newb = VARIANTS[R["variant"]]["new"]
    holes = [dict(what="AIO: the FC's 3 standoffs", at=[tuple(h) for h in hl["holes"]], state="existing (P: the diamond at (1.9, 0)); M3 nuts bonded under"),
             dict(what="AIO: rear", at=[(-16.13, 0.0)], state="existing, left empty (under the tail block's foot)"),
             dict(what="side: the belly mount", at=[b for b in bolts if b in SIDE_HOLES], state="existing (P)"),
             dict(what="the belly mount", at=newb, state="new, Ø3.4" if newb else "none"),
             dict(what="the tail block (tail mount v3)", at=[(-42.0, 16.0), (-42.0, -16.0)], state="new, Ø3.4 (round 5's, kept plate-fixed)"),
             dict(what="the tie saddle's 4 tabs (layout v13, the cage session's)", at=[(-20.0, 39.3), (-20.0, -39.3), (20.0, 39.3), (20.0, -39.3)], state="new (layout)"),
             dict(what="nose", at=[NOSE_HOLE], state="existing, inside the battery-lead notch"),
             dict(what="battery-lead notch", at=[(round(NOTCH_X0, 2), round(edge_x, 1)), (NOTCH_Y, NOTCH_R)], state="new cut: x 38.15..edge, |y| <= 6.2, r 1.5"),
             dict(what="tail notch (the Bee35's XT60 cut-out)", at=[(-50.0, -40.0), (9.0, 0.0)], state="existing; the tail block's web, fin and foot pass it")]
    say("plate holes, one list: " + "; ".join(f"{h['what']} {h['at']}: {h['state']}" for h in holes)
        + f" = {sum(len(h['at']) for h in holes if h['state'].startswith('new') and 'notch' not in h['what'])} new holes (4 of them the layout's), "
        f"{sum(len(h['at']) for h in holes if h['state'].startswith('existing') and 'notch' not in h['what'])} existing ones used or kept")
    # --- PX4 offsets (FRD from the CG, m)
    cg = np.array(mc["cg"], float)
    imu = np.array([FC_C[0], FC_C[1], (hl["pcb"][0] + hl["pcb"][1]) / 2])
    sgn = 1.0 if flow["ori"][1] == "-" else -1.0
    lens = np.array([flow["c"][0] + sgn * FLOW_WIN, flow["c"][1], flow["zb"]]); tof = np.array([flow["c"][0] - sgn * FLOW_WIN, flow["c"][1], flow["zb"]])
    frd = lambda p: np.round([(p[0] - cg[0]) / 1000, -(p[1] - cg[1]) / 1000, -(p[2] - cg[2]) / 1000], 4).tolist()
    px4 = dict(cg=np.round(cg, 2).tolist(), EKF2_IMU_POS=frd(imu), EKF2_OF_POS=frd(lens), EKF2_RNG_POS=frd(tof),
               SENS_FLOW_ROT="0 (as built 4, dimensions.md A; the board turned 180: plug to the nose), CHECK on the bench",
               SENS_BOARD_ROT="as built it hung upside down under the plate (nox/scout/scout.md); on top, MCU side up: the roll 180 goes, the yaw stays (set at the bench)",
               filtering=FILTERING, note="IMU at the board centre, PCB middle (E)")
    say(f"PX4 (FRD from the CG {px4['cg']}, m): EKF2_IMU_POS {px4['EKF2_IMU_POS']}, EKF2_OF_POS {px4['EKF2_OF_POS']}, EKF2_RNG_POS {px4['EKF2_RNG_POS']}; "
        f"SENS_FLOW_ROT {px4['SENS_FLOW_ROT']}; SENS_BOARD_ROT {px4['SENS_BOARD_ROT']}")
    masses = {n: dict(g=round(float(P[n]["g"]), 2), cg=np.round(P[n]["c"], 2).tolist()) for n in (BM, FCN, STO, "wires", FLW, TBN, CAR, CABLES)}
    say("masses and CGs for the cage session (E): " + "; ".join(f"{n} {v['g']} g at {v['cg']}" for n, v in masses.items()))
    cin = bal["cases"]["all"]["horizontal_mm"] if bal.get("applies") else None
    cout = bal["cases"].get("no_swap", {}).get("horizontal_mm") if bal.get("applies") else None
    bench = ("Bench (Spark): with the FC off, measure the plate's 4 AIO holes: the pattern (25.5 square turned 45 about (1.9, 0)?), the hole size, "
             "and which corner the USB-C faces")
    say(bench)
    # --- pictures and results
    bvar = {}
    for k in ("VA", "VB", "VC"):
        q = VR[k]; b = q["L"]["by"]
        bvar[k] = dict(mesh=q["mesh"], variant=q["variant"], picked=(k == pick), lines=[
            f"{VARIANTS[q['variant']]['label']}", f"{q['g']:.1f} g as printed (E), new plate holes {q['new']}, solid_check {'PASS' if q['ok'] else 'FAIL'}",
            f"hung set first modes {q['hung'][0]:.0f} / {q['hung'][1]:.0f} Hz (E; first {'>=' if q['hung_ok'] else '<'} {hung_lo:.0f})",
            f"crash worst {q['crash']:.3f} (<= {CRASH_TARGET:.2f}); prying up to {b['crash']['pry'][0]:.2f} x", f"flight / landing worst {q['fl']:.3f} (<= 0.5)"])
    alt_text = ["Also weighed (same model):"] + [f"- belly {k}: {q['g']:.1f} g, hung {q['hung'][0]:.0f} Hz, crash {q['crash']:.3f}, holes {q['new']}" for k, q in VR.items() if k not in ("VA", "VB", "VC")] + [
        f"- FC vertical columns (round 1): best {vbest['f_lo']:.0f}..{vbest['f_hi']:.0f} Hz: no fit",
        f"- isolator B, rubber -20 / +20 %: {sens[0.8][0][0]:.0f}..{sens[0.8][-1][0]:.0f} / {sens[1.2][0][0]:.0f}..{sens[1.2][-1][0]:.0f} Hz",
        f"- flow CG in the strut plane: {bp['worst_no_offset']:.4f} vs {bp['worst'][0]:.4f}",
        "- FC filtering (DEFAULT): bidirectional DShot (Bluejay), RPM notch, FFT;", "  gate: the first-hover vibration log"]
    notes = [f"FC on top (Spark 27 Oct): the diamond at (1.9, 0), P", f"  H-high: hard on 3 standoffs x {hl['L']:.1f}, board z {hl['env'][0]:.0f}..{hl['env'][1]:.0f}",
             f"  modes: rigid {hf['rigid'][0][0]:.0f}+, PCB corner ~{hf['plate_Hz']} Hz (E)", f"  {hl['foot_gap']:.1f} over the tail block as drawn; plan {fc_plan:.1f}",
             "  filtering in PX4 (DShot RPM notch + FFT)", f"  B (alt.): isolator {mdl[0][0]:.0f}..{mdl[-1][0]:.0f} Hz, +{b_extra:.0f} g",
             f"belly ({pick}): {bm_g:.1f} g E; hung {R['hung'][0]:.0f} Hz (E)", f"  crash {R['crash']:.3f}, flight / landing {R['fl']:.3f}", f"  new plate holes {R['new']}",
             f"system_check {'PASS' if r['pass'] else 'FAIL'}: CG {cin} / {cout}", f"  access {sum(1 for a in acc_rows if a[1])} / {len(acc_rows)}; mass {mc['mass_g']:.0f} g E",
             f"fit: {sum(1 for f in ft if f['pass_'])} / {len(ft)} pairs pass, no allowance"]
    info = dict(flow=flow, usb=usb, lay=hl, hard=hf, iso=dict(ip, modes_leads=mdl), iso_rows=rows_i, bands=bands, bolts=bolts, tb_orig=tb, trim=FOOT_X, fc_plan=fc_plan,
                bipod=bp, notes=notes, fc_options=options, bolt_variants=bvar, alt_text=alt_text, pick=pick)
    preview(P, M, info, os.path.join(RUN, "preview.png"))
    preview_alternatives(P, M, info, os.path.join(RUN, "preview_alternatives.png"))
    res = dict(version="v3 (concept): the FC on top hard on 3 standoffs (H-high, DEFAULT), the isolator B the alternative; the belly mount and the tail mount v3 on layout v13",
               layout="scout_layout v13 (PR 169) + main's assembly (camera mount v1) + the tail mount v3 (runs/scout_electronics/v3/)", catch_dx=DX, wash=wr, plate_holes=holes,
               rotor=bands,
               fc=dict(pose=dict(x=FC_C[0], y=FC_C[1], yaw=FC_YAW, holes=hl["holes"], on="3 of the Bee35's AIO holes (P: the diamond at (1.9, 0)); the rear one empty"),
                       standoffs=dict(L=hl["L"], af=STO_AF, g=round(sto_g, 2)), stack=dict(pcb=hl["pcb"], env=hl["env"]), foot_gap=hl["foot_gap"], modes=hf,
                       plan_margin=round(fc_plan, 2), standoffs_plan=round(sto_plan, 2), roof=dict(lowest=None if not np.isfinite(roof[0]) else roof[0], member=roof[1]),
                       esc=dict(side_open=round(side_open, 3), top_open=round(top_open, 3), hover_A=[round(i, 1) for i in i_hov], loss_W=AIO_LOSS_W, rise_K=dT),
                       filtering=FILTERING, bench_check=bench),
               alternative_B=dict(status="layout sketch; built only if the first-hover vibration log fails", pick={k: v for k, v in ip.items()},
                                  modes=md0, modes_with_leads=mdl, sensitivity={"-20 %": sens[0.8], "+20 %": sens[1.2]}, k=kk, k_horizontal_N_mm=round(k_horiz, 1),
                                  lead_k_N_mm=round(lead_k, 2), moving_g=mg, extra_g=round(b_extra, 1), tray_g=round(tray_g, 2), base_g=round(peds_g, 2),
                                  env=lay["env"], vertical_columns_best=vbest, n_ok=sum(rr_["ok"] for rr_ in rows_i)),
               fc_options={k: {kk_: vv for kk_, vv in o.items() if kk_ not in ("peds", "tb")} for k, o in options.items()},
               usb=dict(side=usb["side"], from_corner=usb["s"], azimuth=usb["az"], elevation=usb["el"], dir=np.round(usb["d"], 4).tolist(), p0=np.round(usb["p0"], 2).tolist(), n_clear=usb["n_clear"]),
               flow=dict(flow, c=list(flow["c"]), bipod=bp, n_scored=len(scored), n_feasible=len(v1.FEAS)),
               belly_mount=dict(pick=pick, mass_g=bm_g, mass_solid_g=R["g_solid"], cg=np.round(bm_c, 2).tolist(), bounds=np.round(bm.bounds, 2).tolist(),
                                solid_check=dict(pass_=bool(ok), **{k: v for k, v in sc.items() if k != "ok"}), cradle_g=R["cradle_g"], plate_bolts=bolts,
                                receiver_inserts=REC_BOLTS, tail_face_x=TAIL_X, ear_y=EAR_Y, hung_rule=">= 100 Hz first mode (E)",
                                variants={k: dict(g=q["g"], g_solid=q["g_solid"], crash=q["crash"], flight_landing=q["fl"], new_holes=q["new"], solid=q["ok"], hung_modes=q["hung"],
                                                  hung_ok=q["hung_ok"], zmin=q["zmin"], by={kk_: {f: list(vv) if isinstance(vv, tuple) else vv for f, vv in bb.items()} for kk_, bb in q["L"]["by"].items()})
                                          for k, q in VR.items()},
                                sections_x={str(k): v for k, v in R["secs_x"].items()}, sections_y={str(k): v for k, v in R["secs_y"].items()},
                                cases=R["L"]["rows"], hung_g=R["L"]["hung_g"], hung_cg=R["L"]["hung_cg"]),
               tail_block=dict(owner="this session: the tail mount v3 (nox/scout/v3/electronics_v3.py)", change="refit to v13 (wash cut, plate interfaces kept)",
                               couple_peak_N=tu["F"], couple_case=tu["case"],
                               note="the rear AIO hole under the foot: a 4th FC post or a foot bolt, weighed in the summary"),
               plate=dict(aio_holes="3 FC standoffs (M3 into bonded nuts); the rear one, under the tail block's foot, stays empty", side_holes=SIDE_HOLES,
                          notch=dict(x=[round(NOTCH_X0, 1), round(edge_x, 1)], y=round(NOTCH_Y, 1), r=NOTCH_R), nose_hole="inside the notch (was empty)",
                          new_holes=VARIANTS[R["variant"]]["new"], v1_new_holes_dropped=[(30, 13), (30, -13)]),
               fit=ft, system_check=dict(summary=r["summary"], pass_=bool(r["pass"]), cg_offset_in=cin, cg_offset_out=cout, mass_g=mc["mass_g"], cg=mc["cg"], access=acc_rows),
               px4=px4, masses=masses, wires=WIRES, cables_box=CABLE_BOX)
    json.dump(res, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=ser)
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())
    return res


def pg_exterior(P):
    pg, _ = plate_outline(P)
    return pg.exterior.coords if hasattr(pg, "exterior") else max(pg.geoms, key=lambda g: g.area).exterior.coords


if __name__ == "__main__":
    run()
