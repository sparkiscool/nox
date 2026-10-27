"""Overseer sensor head mount (printed PETG, nox/topo): the L2 and the two cameras on printed parts instead of short rods
(Spark 27 Oct: "for the mounts on the lidar and the camera, replace the bunch of little rods with a topology printed mount";
"make the cameras and lidar be mounted on petg not rods, make those parts first").

Requirements from the system pass: runs/overseer_layout/v5/head_system.json (layout_v5.py), the cage nox/overseer/cage_v5_shell.json.
Two pieces (the head is 230 wide, the bed 180): the centre piece (|y| <= 72: the L2 seat, back block and lugs with the keel and
brace sockets, the guard ring, the cage's front centre node with its tube sockets, joint pads for the wings) and a wing (the left
one; the right is its mirror: the camera pad, the camera guard bar, joint pads, sockets for 3.0 carbon struts to the side cage
nodes). Drone frame, mm, N.

One piece (Spark 27 Oct: "make the camera mounted on the same print as the lidar mount"): `head` = the centre piece's bodies
(no wing joint pads) + both camera plates and guard bars + strut sockets to the side cage nodes, in one print (the layout placed
the cameras so the head's box fits the A1 mini, layout_v5.py HEAD_ONE). The L2 loads at its CG (head_system.json l2.cg, 28
ahead of its base) through its 4 bolts, force and moment (27 Oct review); each crash case carries the L2 and both cameras.

python3 nox/overseer/head_mount.py head|centre|wing [out_dir] [--h 2.0] [--git]
"""
import os, sys, json, math, argparse
import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))
HS = json.load(open(os.path.join(ROOT, "runs", "overseer_layout", "v5", "head_system.json")))
CG = json.load(open(os.path.join(HERE, "cage_v5_shell.json")))
B = np.array(HS["l2"]["base"]); A = np.array(HS["l2"]["axis"]); UPB = np.array([-A[2], 0, A[0]])
TILT = 105.0
G = 9.81
M_L2, M_CAM = 0.230, 0.015                     # kg (P, E)
M_HEAD = 0.36                                  # kg: L2 230 + cameras 30 + mount 102 (system model), what a guard member stops (E)
CRASH, FLIGHT_UP, FLIGHT_LAT = 35.0, 4.0, 2.0  # g (dimensions.md L)
ROD_BORE, TUBE_BORE = 3.2, 6.2                 # socket bores (Spark 27 Oct, coupons v2: the largest fits, standing and sideways)
NODE_R, TUBE_R = CG["node_r_mm"], CG["rod_r_mm"]
GUARD_R = 3.4                                  # guard ring / bar: printed ~6 x 6 (Ø6.8 round)
GUARD_NODE_R = 5.0
_LS = os.path.join(HERE, "link_sizes.json")
LINK6 = set(json.load(open(_LS))["tube6"]) if os.path.exists(_LS) else set()   # keels / braces the frame model made 6/4 tube
WING_Y = 72.0                                  # the cut between the centre piece and the wings (DEFAULT, bed 180)
_CN = np.array(CG["nodes"]); _k = [i for i in range(len(_CN)) if abs(_CN[i, 1]) < 0.5 and _CN[i, 0] > 300]
CENTRE_NODE = _CN[max(_k, key=lambda i: _CN[i, 0])] if _k else np.array([352.88, 0.0, 47.0])   # the cage's front centre node: part of the centre piece


# ---------------------------------------------------------------- geometry helpers
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def xf(m, T):
    """apply a 4x4 rigid transform"""
    return m.transform(np.asarray(T, float)[:3, :4])


def T_l2():
    c, s = math.cos(math.radians(TILT)), math.sin(math.radians(TILT))
    T = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]]); T[:3, 3] = B
    return T


def seg(a, b, r, n=24):
    """a round bar from a to b"""
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; L = float(np.linalg.norm(d))
    z = d / L; x = np.cross(z, [0, 0, 1.0]) if abs(z[2]) < 0.9 else np.cross(z, [1.0, 0, 0]); x /= np.linalg.norm(x); y = np.cross(z, x)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, a
    return xf(mf.Manifold.cylinder(L, r, r, n), T)


def ball(c, r):
    return mf.Manifold.sphere(r, 32).translate(list(map(float, c)))


def socket(end, toward, bore, depth, wall=3.0):
    """a drawn boss round a rod / tube socket at `end`, the rod leaving toward `toward`: (boss, bore cut, rod keep-out)"""
    end, toward = np.asarray(end, float), np.asarray(toward, float); d = (toward - end) / np.linalg.norm(toward - end)
    boss = seg(end - d * depth, end + d * 1.0, bore / 2 + wall)
    cut = seg(end - d * depth, end + d * 400.0, bore / 2)
    keep = seg(end + d * 2.0, end + d * 400.0, bore / 2 + 1.0)       # 1 past the boss end: no coplanar face at the mouth (v3: voxel
                                                                       # specks at every socket rim failed the solid gate, 27 Oct)
    return boss, cut, keep


def half_space(n, p0, off, size=900.0):
    """everything on the +n side of the plane through p0 + n*off"""
    n = np.asarray(n, float) / np.linalg.norm(n); z = n; x = np.cross(z, [0, 1.0, 0]) if abs(z[1]) < 0.9 else np.cross(z, [1.0, 0, 0])
    x /= np.linalg.norm(x); y = np.cross(z, x); T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z; T[:3, 3] = np.asarray(p0, float) + n * off
    return xf(box(-size, size, -size, size, 0, size), T)


def frustum(c, L=320.0, extra=2.0):
    lens, f, u, r = (np.array(c[k], float) for k in ("lens", "f", "u", "r"))
    ha, hv = math.radians(c["half_across"] + extra), math.radians(c["half_up"] + extra)
    pts = [lens - f * 2.0] + [lens + L * (f + math.tan(ha) * sa * r + math.tan(hv) * sb * u) for sa in (1, -1) for sb in (1, -1)]
    return mf.Manifold.hull_points(np.array(pts))


def cam_frame(c):
    f, u, r = (np.array(c[k], float) for k in ("f", "u", "r")); T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = f, r, u; T[:3, 3] = np.array(c["lens"]) - 9.0 * f
    return T


def l2_body():
    body = box(-37.5, 37.5, -37.5, 37.5, 0, 39) + mf.Manifold.sphere(35.5, 48).scale([1, 1, 26 / 35.5]).translate([0, 0, 39])
    return xf(body, T_l2())


def cage_tubes(skip_node=None, r=TUBE_R + 1.0):
    N = np.array(CG["nodes"]); out = None
    for a, b in CG["rods"]:
        if skip_node is not None and (np.allclose(N[a], skip_node, atol=0.5) or np.allclose(N[b], skip_node, atol=0.5)): continue
        s = seg(N[a], N[b], r, 16); out = s if out is None else out + s
    for i, n in enumerate(N):
        if skip_node is not None and np.allclose(n, skip_node, atol=0.5): continue
        out = out + ball(n, NODE_R + 1.0)
    return out


def tubes_at(node):
    N = np.array(CG["nodes"]); out = []
    for a, b in CG["rods"]:
        if np.allclose(N[a], node, atol=0.5): out.append(N[b])
        elif np.allclose(N[b], node, atol=0.5): out.append(N[a])
    return out


def bolt_pad(c, axis, r=6.0, t=6.0, hole=3.4):
    """an M3 joint pad Ø12 x 6 at c, its bolt along axis: (pad, hole cut)"""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return seg(c - axis * t / 2, c + axis * t / 2, r), seg(c - axis * 60, c + axis * 60, hole / 2)


JOINT = [np.array([372.0, WING_Y, 62.0]), np.array([380.0, WING_Y, 112.0])]   # wing joint pads (left side), bolts along y


# ---------------------------------------------------------------- the centre piece
def centre_bodies(skip_pads=False):
    T = T_l2()
    seat = xf(mf.Manifold.cylinder(5.0, 37.5, 37.5, 64).translate([0, 0, -5.0]), T)
    back = xf(box(-22, 22, -16, 16, -11, -5), T)
    lugs = [xf(box(-22, 4, 36, 50, -8, -2), T), xf(box(-22, 4, -50, -36, -8, -2), T)]
    drawn = seat + back + lugs[0] + lugs[1]
    cuts = None
    def addcut(m):
        nonlocal cuts; cuts = m if cuts is None else cuts + m
    # L2 bolts: 4 x M3 on Ø51, 22.5 deg off the cable axis (local -x), through the seat, heads and driver from behind
    for k in range(4):
        a = math.radians(22.5 + 90 * k); p = np.array([25.5 * math.cos(a), 25.5 * math.sin(a)])
        addcut(xf(mf.Manifold.cylinder(12.0, 1.7, 1.7, 24).translate([p[0], p[1], -11.0]), T))
        addcut(xf(mf.Manifold.cylinder(60.0, 4.5, 4.5, 24).translate([p[0], p[1], -65.0]), T))     # head + driver Ø9
    addcut(xf(box(-50, -32, -8, 8, -40, 2), T))                                                        # cable at the base, -x
    # keel and brace sockets on the lugs
    sockets = []
    for k, (p, q) in HS["carbon_kept"].items():
        end = np.array(q if k.startswith("keel") else p, float); far = np.array(p if k.startswith("keel") else q, float)
        six = k in LINK6; boss, cut, keep = socket(end, far, TUBE_BORE if six else ROD_BORE, 15.0 if six else 12.0, wall=2.5 if six else 3.0)
        drawn += boss; addcut(cut + keep); sockets.append(boss)
    # guard ring (a cage member, drawn): its nodes and bars
    gr = [np.array(v, float) for v in HS["guard_nodes"][:8]]
    for i in range(8):
        drawn += ball(gr[i], GUARD_NODE_R) + seg(gr[i], gr[(i + 1) % 8], GUARD_R)
    # the cage's front centre node is part of the piece: its tube sockets
    node = ball(CENTRE_NODE, NODE_R); drawn += node; frame = []
    for far in tubes_at(CENTRE_NODE):
        boss, cut, keep = socket(CENTRE_NODE + (np.array(far) - CENTRE_NODE) / np.linalg.norm(np.array(far) - CENTRE_NODE) * NODE_R,
                                 far, TUBE_BORE, 14.0, wall=2.5)
        drawn += boss; addcut(cut + keep); frame.append(boss)
    # wing joint pads (both sides), bolts along y
    pads = []
    for sy in (() if skip_pads else (1, -1)):
        for j in JOINT:
            c = j * [1, sy, 1] - np.array([0, sy * 3.0, 0]); pad, hole = bolt_pad(c, [0, 1, 0]); drawn += pad; addcut(hole); pads.append(pad)
    # keep-outs: the L2 and its field (1 behind its base plane, the seat excepted), both cameras and their views, the cage
    addcut(l2_body() + half_space(A, B, 0.0))
    for c in HS["cameras"]:                                            # (one piece: the plates are drawn, the camera body only)
        addcut(frustum(c) + xf(box(-1.4 if skip_pads else -14, 10, -21, 21, -21, 21), cam_frame(c)))
    addcut(cage_tubes(skip_node=CENTRE_NODE))
    # envelope: behind the L2 base plane, |y| <= WING_Y, round the head
    env = box(334.0, 405.0, -WING_Y, WING_Y, -14.0, 122.0) - half_space(A, B, -1.0)
    return dict(drawn=drawn, cuts=cuts, env=env, seat=seat, sockets=sockets, frame=frame, pads=pads, guard=gr)


def setup_centre(h):
    from topo import Part
    b = centre_bodies()
    p = Part("head_centre", h, b["drawn"], b["cuts"], b["env"], build="+z", material="PETG")
    sup = b["frame"][0]
    for s in b["frame"][1:] + b["sockets"]: sup = sup + s
    p.support("frame", sup)                                            # the keels, braces and the cage tubes hold it
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs):                                       # the L2 at its CG through the seat: crash 35 g, 14 directions
        p.load(f"crash L2 {k}", b["seat"], M_L2 * G * CRASH * d, cls="misuse", support="frame", accel=CRASH * d)
    p.load("flight", b["seat"], M_L2 * G * np.array([FLIGHT_LAT, 0, -FLIGHT_UP]), cls="flight", support="frame",
           accel=np.array([FLIGHT_LAT, 0, -FLIGHT_UP]))
    wing_pads = b["pads"][0] + b["pads"][1]
    for k, d in enumerate(dirs[:6]):                                   # a wing (camera + wing ~ 35 g) through its pads
        p.load(f"crash wing {k}", wing_pads, 0.035 * G * CRASH * d, cls="misuse", support="frame")
    for i, q in enumerate(b["guard"]):                                 # a guard ring node takes the head's crash, inward and back
        inward = (B - A * 8 - q); inward[0] = 0; inward = inward / max(np.linalg.norm(inward), 1e-6) - A * 0.5
        p.load(f"guard hit {i}", ball(q, GUARD_NODE_R), M_HEAD * G * CRASH * inward / np.linalg.norm(inward), cls="misuse", support="frame")
    return p


# ---------------------------------------------------------------- the wing (left; the right is its mirror)
def wing_bodies():
    c = HS["cameras"][0] if HS["cameras"][0]["side"] > 0 else HS["cameras"][1]
    Tc = cam_frame(c)
    plate = xf(box(-9.5, -1.5, -20, 20, -20, 20), Tc)                   # the board's plate, 8 thick (board at -1..1 in front of it)
    drawn = plate; cuts = None
    def addcut(m):
        nonlocal cuts; cuts = m if cuts is None else cuts + m
    for a_ in (-14, 14):                                               # M2.5 heat-set inserts on 28 x 28, from the board face
        for b_ in (-14, 14):
            addcut(xf(seg([-7.5, a_, b_], [0.5, a_, b_], 1.6), Tc))     # blind, 6 deep: a 2.0 floor behind (v1's 6.1 in a 6 plate left 0.1 rims)
    gb = [np.array(v, float) for v in HS["guards"]["cam guard bar"]]
    guard = ball(gb[0], GUARD_NODE_R) + ball(gb[1], GUARD_NODE_R) + seg(gb[0], gb[1], GUARD_R); drawn += guard
    pads = []
    for j in JOINT:
        pad, hole = bolt_pad(j + np.array([0, 3.0, 0]), [0, 1, 0]); drawn += pad; addcut(hole); pads.append(pad)
    struts = []                                                        # 3.0 carbon struts to the side cage nodes (y > 150)
    side = []
    CN = np.array(CG["nodes"]); W0 = np.array([368.0, 112.0, 70.0])
    for k_, q in enumerate(CN):                                        # side nodes by the head, only where the strut is 20 deg+ from their tubes
        if q[1] > 150 and q[0] > 300:
            d = (W0 - q) / np.linalg.norm(W0 - q)
            tub = [CN[b_ if a_ == k_ else a_] - q for a_, b_ in CG["rods"] if k_ in (a_, b_)]
            if all(math.degrees(math.acos(np.clip(d @ (t / np.linalg.norm(t)), -1, 1))) >= 20.0 for t in tub): side.append(q)
    for q in side:
        end = np.array([368.0, 112.0, 70.0]) + (q - np.array([368.0, 112.0, 70.0])) / np.linalg.norm(q - np.array([368.0, 112.0, 70.0])) * 6.0
        boss, cut, keep = socket(end, q, ROD_BORE, 12.0); drawn += boss; addcut(cut + keep); struts.append(boss)
    addcut(l2_body() + half_space(A, B, 0.0))
    for cc in HS["cameras"]:
        addcut(frustum(cc) + xf(box(-1.4, 10, -21, 21, -21, 21), cam_frame(cc)))
    addcut(cage_tubes())
    env = box(340.0, 405.0, WING_Y, 125.0, 40.0, 140.0) - half_space(A, B, -1.0)
    return dict(drawn=drawn, cuts=cuts, env=env, plate=plate, guard=guard, pads=pads, struts=struts, cam=c)


def setup_wing(h):
    from topo import Part
    b = wing_bodies()
    p = Part("head_wing", h, b["drawn"], b["cuts"], b["env"], build="+z", material="PETG")
    sup = b["pads"][0] + b["pads"][1]
    for s in b["struts"]: sup = sup + s
    p.support("frame", sup)
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    for k, d in enumerate(dirs):
        p.load(f"crash camera {k}", b["plate"], M_CAM * G * CRASH * d, cls="misuse", support="frame", accel=CRASH * d)
    p.load("flight", b["plate"], M_CAM * G * np.array([FLIGHT_LAT, 0, -FLIGHT_UP]), cls="flight", support="frame",
           accel=np.array([FLIGHT_LAT, 0, -FLIGHT_UP]))
    for k, d in enumerate([np.array([0, 0, -1.0]), np.array([0, -1.0, 0]), np.array([-1.0, 0, 0]), np.array([0, -0.6, -0.8])]):
        p.load(f"guard hit {k}", b["guard"], M_HEAD * G * CRASH * 0.5 * d, cls="misuse", support="frame")   # half the head on a bar (E)
    return p


# ---------------------------------------------------------------- one piece: the head with both cameras
# struts from the one-piece head to the side cage nodes (left; the right mirrored): the shortest socket point inside the print box,
# behind the L2 base plane by 4, the strut out of both camera views (+4 deg) and 20 deg+ from the node's tubes (searched 27 Oct, C)
HEAD_STRUTS = [((401.0, 83.0, 151.0), (354.0, 223.0, 128.7)), ((383.0, 83.0, 73.0), (336.5, 219.0, -29.3))]


def head_struts():
    """[(socket point, cage node)] both sides"""
    return [(np.array(w) * [1, sy, 1], np.array(q) * [1, sy, 1]) for w, q in HEAD_STRUTS for sy in (1, -1)]


def l2_bolts():
    """the 4 L2 bolt points on the seat face (drone frame) and the seat centre"""
    T = T_l2(); out = []
    for k in range(4):
        a = math.radians(22.5 + 90 * k); out.append((T @ np.array([25.5 * math.cos(a), 25.5 * math.sin(a), 0.0, 1.0]))[:3])
    return out, (T @ np.array([0, 0, 0, 1.0]))[:3]


def couple(F, at, pts):
    """least-norm forces at pts that carry force F acting at `at` (its moment about the bolts included)"""
    c = np.mean(pts, 0); Mm = np.cross(at - c, F); rows = []
    for p in pts:
        S = np.array([[0, -(p - c)[2], (p - c)[1]], [(p - c)[2], 0, -(p - c)[0]], [-(p - c)[1], (p - c)[0], 0]])
        rows.append(np.vstack([np.eye(3), S]))
    Aeq = np.hstack(rows); x = np.linalg.pinv(Aeq) @ np.concatenate([F, Mm])
    return [x[3 * k:3 * k + 3] for k in range(len(pts))]


def head_bodies():
    b = centre_bodies(skip_pads=True)
    drawn, cuts = b["drawn"], b["cuts"]; plates, cguards, struts = [], [], []
    CN = np.array(CG["nodes"])
    for c in HS["cameras"]:
        Tc = cam_frame(c); pl = xf(box(-9.5, -1.5, -20, 20, -20, 20), Tc); drawn += pl; plates.append(pl)
        for a_ in (-14, 14):
            for b_ in (-14, 14): cuts += xf(seg([-7.5, a_, b_], [0.5, a_, b_], 1.6), Tc)
    for k_ in ("cam guard bar", "cam guard bar m"):
        gb = [np.array(v, float) for v in HS["guards"][k_]]
        g_ = ball(gb[0], GUARD_NODE_R) + ball(gb[1], GUARD_NODE_R) + seg(gb[0], gb[1], GUARD_R); drawn += g_; cguards.append(g_)
    for W, q in head_struts():                                          # struts to the side cage nodes
        k_ = int(np.argmin(np.linalg.norm(CN - q, axis=1))); six = f"head strut {k_}" in LINK6
        boss, cut, keep = socket(W, q, TUBE_BORE if six else ROD_BORE, 15.0 if six else 12.0, wall=2.5 if six else 3.0)
        drawn += boss; cuts += cut + keep; struts.append(boss)
    for c in HS["cameras"]:                                             # the camera bodies and views stay clear (plates excepted)
        cuts += frustum(c) + xf(box(-1.4, 10, -21, 21, -21, 21), cam_frame(c))
    pts = np.vstack([np.asarray(mf_mesh_pts(drawn))])
    lo, hi = pts.min(0) - 6.0, pts.max(0) + 6.0
    for ax in range(3):                                                 # the print box: no axis over 178 (A1 mini 180, C)
        if hi[ax] - lo[ax] > 178.0: c_ = (hi[ax] + lo[ax]) / 2; lo[ax], hi[ax] = c_ - 89.0, c_ + 89.0
    env = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]) - half_space(A, B, -1.0)
    return dict(drawn=drawn, cuts=cuts, env=env, seat=b["seat"], frame=b["frame"], sockets=b["sockets"] + struts, guard=b["guard"],
                cguards=cguards, plates=plates)


def mf_mesh_pts(m):
    return np.asarray(m.to_mesh().vert_properties)[:, :3]


def setup_head(h):
    from topo import Part
    b = head_bodies()
    p = Part("head", h, b["drawn"], b["cuts"], b["env"], build="+z", material="PETG")
    sup = b["frame"][0]
    for s_ in b["frame"][1:] + b["sockets"]: sup = sup + s_
    p.support("frame", sup)                                            # keels, braces, struts and the cage tubes at the centre node
    bolts, sc = l2_bolts(); L2CG = np.array(HS["l2"]["cg"]); breg = [ball(q, 6.0) for q in bolts]
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
    dirs += [np.array((sx, sy, sz), float) / math.sqrt(3) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
    def case(acc):                                                      # the L2 at its CG through its bolts + both cameras at their plates
        F = M_L2 * G * acc; parts = list(zip(breg, couple(F, L2CG, bolts)))
        return parts + [(pl, M_CAM * G * acc) for pl in b["plates"]]
    for k, d in enumerate(dirs):
        p.load_multi(f"crash {k}", case(CRASH * d), cls="misuse", support="frame", accel=CRASH * d)
    fl = np.array([FLIGHT_LAT, 0, -FLIGHT_UP]); p.load_multi("flight", case(fl), cls="flight", support="frame", accel=fl)
    for i, q in enumerate(b["guard"]):                                 # a guard ring node takes the head's crash, inward and back
        inward = (B - A * 8 - q); inward[0] = 0; inward = inward / max(np.linalg.norm(inward), 1e-6) - A * 0.5
        p.load(f"guard hit {i}", ball(q, GUARD_NODE_R), M_HEAD * G * CRASH * inward / np.linalg.norm(inward), cls="misuse", support="frame")
    for i, g_ in enumerate(b["cguards"]):
        for k, d in enumerate([np.array([0, 0, -1.0]), np.array([-1.0, 0, 0])]):
            p.load(f"camera guard hit {i}.{k}", g_, M_HEAD * G * CRASH * 0.5 * d, cls="misuse", support="frame")   # half the head on a bar (E)
    return p


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("piece", choices=["head", "centre", "wing"]); ap.add_argument("out", nargs="?")
    ap.add_argument("--h", type=float, default=2.0); ap.add_argument("--gen", type=float, default=None); ap.add_argument("--git", action="store_true")
    ap.add_argument("--smoke", action="store_true", help="build the Part once and report, no optimisation")
    a = ap.parse_args()
    setup = {"head": setup_head, "centre": setup_centre, "wing": setup_wing}[a.piece]
    if a.smoke:
        p = setup(a.h); print(a.piece, "grid", p.shape, "ok"); sys.exit(0)
    from topo import Settings, optimise
    out = a.out or os.path.join(ROOT, "runs", f"head_{a.piece}" if a.piece != "head" else "head_mount", "v1")
    gen = a.gen if a.gen is not None else {"head": 26.0, "centre": 16.0, "wing": 5.0}[a.piece]   # cm3 generated (the head ~100 g, E)
    optimise(setup, h=a.h, gen_cm3=gen, out_dir=out, name="head_mount" if a.piece == "head" else f"head_{a.piece}", settings=Settings(), git=a.git)
