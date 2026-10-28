"""Scout v3 layout v3 / cage v2 (27 Oct 2026): the Scout v3 drone (PR 94's latest layout: Bee35 carbon plate, props on top,
receiver v3 + cartridge v6 under it, FC / UBEC / Radxa / flow on the belly mount) in a triangulated shell cage, the Overseer cage v3
method (`nox/overseer/layout_v5.py`) with the Scout's own rod rules.

Spark 27 Oct: "using the same design idea as the latest scout design make sure the bottom of the cage is the exact same as the
overseer just that loop of rods directly under the battery so funnel is interchangeable, the scout still has one camera and we
will keep the carbon base for the scout" (cage v1, `runs/scout_layout/v6/`); then "subtract the number of rods, these are all 3mm
as well and also just divide the number of rods by half" and "dont worry about same rod legnth anymore" (cage v2).

- Bottom ring: the Overseer's 8 opening ring nodes and 8 ring rods, read from `runs/overseer_layout/v5/cage_shell.json` at run
  time, on the same datum (ring centre on the catch / grip line, plane 3 under the cartridge flat bottom); the ring nodes stay
  Ø20 (the funnel contact), the rods are 3.0 like every Scout rod.
- The shell (cage v2, pattern "free"): no geodesic spacing and any rod length; its nodes are corners of the drone's margin hull
  grown by an offset (scanned), taken farthest-outside first in mirror pairs until the hull holds the whole drone; within the
  rod cap (about half of cage v1's 123 rods) the most compact shell that balances on its own wins. Pattern "geodesic" (cage v1's
  frequency 2, optionally thinned to SCOUT_NFREE nodes) is kept for the alternatives.
- One camera (front, horizon, landscape) outside the shell with nothing of the drone in its view, on a braced boom off the
  plate nose, a guard bar over its top behind the lens.
- The carbon base stays; links (lean set, SCOUT_LEAN=1): 2 spokes per arm tip, tail bay ties, a tail core tie, camera braces.
- Balance: the catch set slides aft of the thrust centre (scanned), the shell and camera rebuilt each time.

python3 nox/scout/v3/layout_v3.py [out_dir]          (~13 min)       env: SCOUT_PATTERN free|geodesic, SCOUT_NFREE, SCOUT_LEAN,
SCOUT_OFFSETS, SCOUT_SHELL_CAP, SCOUT_RING=own (a ring round the Scout's own receiver), SCOUT_CAM_Z, SCOUT_DCS, SCOUT_OUT.
Drone frame (PR 94): x nose, y left, z up, plate underside z 18.5, thrust centre x 0. Numbers: `nox/dimensions.md` "System Scout v3".
"""
import math, os, sys, io, json
import numpy as np, trimesh
from scipy.spatial import ConvexHull, cKDTree

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))

RING_MODE = os.environ.get("SCOUT_RING", "v5side")         # v5side: the 4 side nodes of the Overseer cage v5 base loop (Spark 27 Oct, via the design
                                                           # center: "the rectangle part" of its current hexagon); rect: the v3 octagon's middle rectangle;
                                                           # overseer: the whole v3 octagon; own
OV5_SIDE = [(85.8, 116.1), (-68.9, 119.7), (-68.9, -119.7), (85.8, -116.1)]   # about the catch (C, Overseer cage v5, PR 122 nox/overseer/cage_v5_shell.json)
OV_ROD_R = 3.0                                             # the Overseer's ring tube radius (6 mm tubes)
SEAT_DZ = float(os.environ.get("SCOUT_SEAT_DZ", (OV_ROD_R - 1.5) * math.sqrt(2)))   # the Scout plane this much lower under its cartridge: a 3.0 rod
                                                           # sits r·√2 above a 45 deg ramp face, 2.12 lower than the 6 mm tube, so the cartridges
                                                           # stop at the same height (DEFAULT 27 Oct, design center; alt: 6 mm ring tubes)
USB_FIRST = RING_MODE != "v5side"                          # v5side: the links first (the rear spokes need the tail-left side), then the USB path
SADDLE_X = (-45.0, -15.0, 15.0, 38.0)                     # tie saddle (printed PETG, DEFAULT 27 Oct): a rail a side outside the belly mount and the
SADDLE_Y = (36.3, 42.3)                                    # receiver's TPU end pieces (y 36.3..42.3, x -50..42, z -15..6), 2 tabs a side up to the
SADDLE_Z = (-15.0, 6.0)                                    # plate underside between the arms (x ±14..26, 4 M3 into the plate); tie roots on its face
FUNNEL_BAND = 30.0                                         # nothing but the ring rods within this height above the plane, outside the swap column (funnel ramps)
OUT = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SCOUT_OUT") or os.path.join(ROOT, "runs", "scout_layout", "v13", "" if RING_MODE == "v5side" else "alt_" + RING_MODE + "_ring")
DR = os.path.join(ROOT, "runs", "scout_layout", "v6", "drone")          # drone meshes: PR 94 run v5 assembly (24 Oct), as drawn
OV = os.path.join(ROOT, "runs", "overseer_layout", "v5")                # the Overseer cage v3 (ring source)
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


# ---------------------------------------------------------------- numbers (C unless tagged; dimensions.md A / C / L / System Scout v3)
MOTOR, PROP_R, PROP_Z, BLADE = 54.1, 44.45, 41.2, 4.0      # Bee35 hubs (P), Gemfan 3520 (P), prop plane (E), sweep ±4
SHELL_R = 1.5                                              # 3.0 solid carbon, every Scout rod (Spark 27 Oct: "these are all 3mm")
SHELL_GPM = 11.0e-3                                        # 11.0 g/m (C, G Carbon rods)
ROD_R, ROD_GPM = 1.5, 11.0e-3                              # links 3.0 solid, 11.0 g/m (C)
NODE_R, NODE_G = 6.0, 1.5                                  # printed shell node round 3.2 epoxy sockets (Spark coupon pick), Ø12 (pad rule), 1.5 g (E)
RING_NODE_R, RING_NODE_G = 10.15, 4.5                      # the shared ring nodes stay the Overseer's Ø20 (the funnel contact), 4.5 g (E)
GUARD_NODE_R, GUARD_NODE_G = 5.0, 1.5                      # camera guard bar nodes (E)
NODE_MODEL = os.environ.get("SCOUT_NODE_MODEL", "fit")      # fit: node mass from its socket count, fitted to the saved nodes v2 (M); flat: NODE_G
NODE_FIT = (2.34, 3.49, -0.57, -3.14)                      # g per shell socket, per saddle-tie socket, ring node, constant (fit on runs/scout_nodes/v2,
                                                           # 18 nodes, 188.8 g M, max residual 1.9 g, E)
CAP_G = 0.86                                               # TPU foot cap under each ring node (box catch F, runs/scout_nodes/v2, M)
CAM_MOUNT_G = float(os.environ.get("SCOUT_CAM_MOUNT_G", 22.3))   # the camera mount as saved (v1 21.8 + 2 collars 0.5, M), camera excluded


def node_g(sd, td=0, ring=False):
    if NODE_MODEL != "fit": return RING_NODE_G if ring else NODE_G
    return max(1.5, NODE_FIT[0] * sd + NODE_FIT[1] * td + NODE_FIT[2] * ring + NODE_FIT[3]) + (CAP_G if ring else 0.0)


def node_masses(nodes, E, extra=None):
    """each node's mass from its sockets: shell rods, and saddle ties ending on it"""
    sd = np.zeros(len(nodes), int); td = np.zeros(len(nodes), int)
    for a, b in E: sd[a] += 1; sd[b] += 1
    for p, q in (extra or {}).values():
        for e in (p, q):
            d = np.linalg.norm(nodes - np.asarray(e, float), axis=1)
            if d.min() < 1.0: td[d.argmin()] += 1
    return np.array([node_g(sd[i], td[i], i < nR) for i in range(len(nodes))])
PROP_GAP, CAGE_GAP = 12.0, 3.0
M_PROP, M_BODY = PROP_GAP + SHELL_R + 1.0, CAGE_GAP + SHELL_R + 1.0
FREQ = int(os.environ.get("SCOUT_FREQ", 2))                # geodesic frequency
PATTERN = os.environ.get("SCOUT_PATTERN", "free")
RACE = os.environ.get("SCOUT_RACE", "1") == "1"            # free pattern with the race-car silhouette (Spark 27 Oct)
RACE_NOSE = [float(x) for x in os.environ.get("SCOUT_RACE_NOSE", "0,0").split(",")]   # nose point ahead of the target, above the bottom (0: none; Spark 27 Oct: "get rid of that sharp section under the camera")
RACE_TAIL = [float(x) for x in os.environ.get("SCOUT_RACE_TAIL", "55,18").split(",")]   # tail point: behind the target, under its top           # free: nodes on the drone's own corners, any rod length (Spark 27 Oct); geodesic
NDIR = int(os.environ.get("SCOUT_NDIR", 6))
# race pattern anchors (fractions of the target's half length about its centre, of its half width, of its height above the bottom):
# nose point, shoulders, mid sides, haunches, roof front, roof rear, tail point (Spark 27 Oct, the race-car look)
RACE_ANCHORS = [(1.08, 0.0, 0.38), (0.72, 1.0, 0.62), (0.0, 1.06, 0.72), (-0.72, 1.0, 0.66), (0.42, 0.62, 1.0), (-0.42, 0.62, 1.0), (-1.1, 0.0, 0.8)]
SHELL_CAP = int(os.environ.get("SCOUT_SHELL_CAP", 52))     # shell rods allowed: 62 minus about 10 links (the cage ties; the camera is on printed arms), about half of cage v1's 123 (Spark 27 Oct)
OFFSETS = [float(x) for x in os.environ.get("SCOUT_OFFSETS", "16,20,24,28,32").split(",")]   # free pattern corner offsets scanned
NFREE = int(os.environ.get("SCOUT_NFREE", 9))              # free shell nodes kept (Spark 27 Oct: half the rods; 0 = all of the geodesic)
TIES = os.environ.get("SCOUT_TIES", "saddle")             # saddle: the cage ties root on a printed PETG tie saddle beside the receiver, bolted to the
                                                           # plate (review 27 Oct: the replaceable part, not the latch); core: on the plate underside edges
ARM_SPOKES = os.environ.get("SCOUT_ARM_SPOKES", "0") == "1"   # spokes from under the motors: off (Spark 27 Oct: the arms are just carbon fibre; the
                                                           # cage ties to the receiver on the underside instead)
LEAN = os.environ.get("SCOUT_LEAN", "1") == "1"            # links: the essential set only (Spark 27 Oct: half the rods)
BREAK_EVEN = 33.0
CAM_HFOV, CAM_VFOV = 89.0, 73.0                            # DECXIN 2633V2 landscape (the Overseer's 73 x 89 turned, E)
CAM_CONE = 55.0                                            # cage first gated outside this cone (diagonal half-angle 51 + 4)
CAM_W, CAM_H, CAM_D = 38.0, 38.0, 14.0                     # board 38 x 38 (P) + lens, 14 deep (E, PR 94)
LIM = NODE_R + 3.0                                         # shell node centres this far behind the camera's view planes
SLACK = float(os.environ.get("SCOUT_SLACK", 5.0))
CAM_Z = [float(x) for x in os.environ.get("SCOUT_CAM_Z", "30,50").split(",")]   # lens height range scanned (Spark 27 Oct: "you can definitly lower the camera": low in the nose)
PHIS = [float(x) for x in os.environ.get("SCOUT_PHIS", "0,20,35,50").split(",")]   # camera back-plane tilts scanned
MIN_ROD = float(os.environ.get("SCOUT_MIN_ROD", 45.0))      # shortest shell rod (a sliver triangle below, Spark 27 Oct "small triangles", E)
SMALL_TRI = float(os.environ.get("SCOUT_SMALL_TRI", 70.0))  # a hull triangle with every edge under this collapses to one node (Spark 27 Oct, E)
CAM_MOUNT = os.environ.get("SCOUT_CAM_MOUNT", "printed")    # printed: the camera in a PETG cradle on printed arms into the cage nodes, no rods (Spark 27 Oct:
                                                           # "mount to the cage", "not on rods but on printed petg or tpu"); cage: on carbon struts; boom: off the plate nose
CAMP = "camera cradle (printed)" if CAM_MOUNT in ("cage", "printed") else "camera mount (boom + seat)"
LIP = 6.0                                                  # the cradle frame stands this far round the camera body (above, below; cage first over it)
ARM = 9.0                                                  # printed cradle arm, round Ø9 PETG (64 mm², as an 8 x 8 post; posts >= 6 x 6, E)
CLAMP_L, CLAMP_R = 12.0, 4.65                              # printed snap clip on a shell rod (Spark 27 Oct: "clip into the rods ... put it on
CLIP_BORE, CLIP_MOUTH = 3.1, 2.6                           # after cage is done"): 12 long, OD 9.3, bore 3.1 on the 3.0 rod, mouth 2.6 (snaps over), E
TAIL_GAP = float(os.environ.get("SCOUT_TAIL_GAP", 3.0))   # tail bay: the Radxa / UBEC front ends this far behind the receiver (balance trim, DEFAULT)
RADXA_POSE = os.environ.get("SCOUT_RADXA", "stand")       # tail bay (Spark 27 Oct: "move the tail bay out of the prop wash but also make the cg correct in
                                                           # the middle of the props"): stand = the Radxa on edge on the centre plane (65 x 6.31 x 30,
                                                           # dimensions H), in the clear strip between the rear disks (|y| < their inner edge - 3),
                                                           # ports edge up (plugs 14 above), SD end aft; the UBEC flat behind the rear disks; flat = v11
RADXA_Z = float(os.environ.get("SCOUT_RADXA_Z", 8.0))      # stand: the board's lower edge (the funnel: no low tail node)
BAY_FIXED = os.environ.get("SCOUT_BAY_FIXED", "0") == "1"   # alternative: the tail bay fixed in the frame at its v11 place (only the catch moves)
UBEC_DY = float(os.environ.get("SCOUT_UBEC_DY", 7.0))     # the electronics session: the UBEC 7 further out (8 of air over the Radxa's SoC)
UBEC_Z = float(os.environ.get("SCOUT_UBEC_Z", 16.0))      # stand: the UBEC flat behind the rear disks, its bottom here (the funnel hull line)
RADXA_T = 6.31                                             # board thickness over the parts (dimensions H, C from the STEP; 7.62 over the SD slot)
WASH_X = -(MOTOR + PROP_R + 3.0)                           # behind this x nothing is under a rear disk (disk + 3)
TB = 0.0                                                   # tail bay shift aft (scanned with the catch for the balance)
ELEC = os.environ.get("SCOUT_ELEC", os.path.join(ROOT, "runs", "scout_layout", "v13", "electronics_envelope.json"))   # the electronics
ELEC = json.load(open(ELEC)) if ELEC and os.path.exists(ELEC) else None    # session's mount, XR2, cables and keep-clears (drawn at catch ELEC["catch"])
LEN_W = float(os.environ.get("SCOUT_LEN_W", 0.01))         # balance-scan length term: 0.01 mm of CG per mm of cage length (DEFAULT: 30 mm buys 0.3)
TAIL_BAY = os.environ.get("SCOUT_TAIL_BAY", "1") == "1"   # Radxa + UBEC in a tail bay behind the receiver (balance, race-car tail, Spark 27 Oct)
PLATE_NOSE = np.array([60.0, 0.0, 20.25])                  # camera boom root on the plate nose (E, plate edge x ~65)
BOOM = 10.0                                                # printed boom section 10 x 10 (E; braced to the arms, balance)

ld = lambda f: trimesh.load(os.path.join(DR, f), force="mesh")


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b


def cyl(r, z0, z1, x=0, y=0, sections=48):
    c = trimesh.creation.cylinder(r, z1 - z0, sections=sections); c.apply_translation([x, y, (z0 + z1) / 2]); return c


def rod(p0, p1, r=ROD_R):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return trimesh.creation.cylinder(r, np.linalg.norm(p1 - p0), sections=12, segment=np.vstack([p0, p1]))


def pts(m, n=1500, seed=1):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=seed)[0]])


def fib(n):
    i = np.arange(n) + 0.5; z = 1 - 2 * i / n; r = np.sqrt(1 - z * z); a = np.pi * (3 - 5 ** 0.5) * i
    return np.stack([r * np.cos(a), r * np.sin(a), z], 1)


DIRS = fib(64)
MXY = [(MOTOR, MOTOR), (MOTOR, -MOTOR), (-MOTOR, MOTOR), (-MOTOR, -MOTOR)]   # motor 1..4 as PR 94


def seg_band_dist(p0, p1, mxy=MXY):
    t = np.linspace(0, 1, 120)[:, None]; q = np.asarray(p0) + t * (np.asarray(p1) - np.asarray(p0)); best = np.inf
    for mx, my in mxy:
        dr = np.maximum(np.hypot(q[:, 0] - mx, q[:, 1] - my) - PROP_R, 0); dz = np.maximum(np.abs(q[:, 2] - PROP_Z) - BLADE, 0)
        best = min(best, float(np.hypot(dr, dz).min()))
    return best


def geodesic(nu):
    """class-I icosahedral geodesic, unit sphere, a vertex at +z and the xz plane a mirror plane (as the Overseer's)"""
    p = (1 + 5 ** 0.5) / 2
    V = np.array([(-1, p, 0), (1, p, 0), (-1, -p, 0), (1, -p, 0), (0, -1, p), (0, 1, p), (0, -1, -p), (0, 1, -p),
                  (p, 0, -1), (p, 0, 1), (-p, 0, -1), (-p, 0, 1)], float)
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    V /= np.linalg.norm(V, axis=1)[:, None]
    z = V[5]; ax = np.cross(z, [0, 0, 1.0]); sn = np.linalg.norm(ax); cs = z[2]
    Kx = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]) / sn
    V = V @ (np.eye(3) + sn * Kx + (1 - cs) * Kx @ Kx).T
    up = [i for i in range(12) if 0.3 < V[i, 2] < 0.6]
    t = -math.atan2(V[up[0], 1], V[up[0], 0]); Rz = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
    V = V @ Rz.T
    out, seen = [], set()
    for a, b, c in F:
        for i in range(nu + 1):
            for j in range(nu + 1 - i):
                q = (i * V[a] + j * V[b] + (nu - i - j) * V[c]) / nu; q /= np.linalg.norm(q)
                k = tuple(np.round(q, 6))
                if k not in seen: seen.add(k); out.append(q)
    return np.array(out)


# ---------------------------------------------------------------- the drone (PR 94 run v5, everything but its cage)
SW_IGN = ["cartridge v6 (latch)", "cartridge v6 strips (latch, TPU)", "pack 4S 1500", "slider +y (latch)", "slider -y (latch)",
          "pack XT60 plug", "cartridge skid closed end (latch, TPU)", "cartridge skid lead end (latch, TPU)", "receiver TPU strips (latch)"]
CATCH = tuple(dict.fromkeys(("mount + nose legs (Scout)", "FC", "UBEC", "Radxa Zero 3W", "flow MTF-02P", "receiver v3 (latch)", "receiver lid (latch)",
                              "receiver TPU strips (latch)", "receiver TPU end pieces (latch, E)", "XT60E1-M") + tuple(SW_IGN)))
# the catch set slides with the catch (DC aft of the thrust centre); the FC and the UBEC ride in the belly mount over the receiver
# (PR 94: the mount takes z 9.6..18.5 round the FC), so they move with it and keep their clearances to the receiver


def drone(DC):
    """the drone parts, the catch set DC aft of the thrust centre (the plate centre, x 0)"""
    D = {}   # name -> (mesh, role, protect, g, tag, material, group, colour, swap)
    D["frame plate (Bee35 carbon)"] = (ld("frame_plate.stl"), "structure", False, 28.0, "E", "carbon", "frame", "#3a3a3a", False)
    for i, (mx, my) in enumerate(MXY):
        D[f"motor {i + 1}"] = (cyl(12.4, 22.0, 37.0, mx, my), "bought", True, 18.0, "E", None, "bought", "#b0b0b0", False)
        D[f"prop {i + 1}"] = (cyl(PROP_R, PROP_Z - BLADE, PROP_Z + BLADE, mx, my, 96), "bought", True, 1.8, "P", None, "bought", "#d9a400", False)
    D["FC"] = (ld("FC.stl"), "bought", True, 10.0, "P", None, "bought", "#2e7d32", False)
    D["UBEC"] = (ld("UBEC.stl"), "bought", True, 15.0, "E", None, "bought", "#2e7d32", False)
    D["XR2"] = (ld("XR2.stl"), "bought", True, 1.0, "P", None, "bought", "#2e7d32", False)
    D["Radxa Zero 3W"] = (ld("Radxa_Zero_3W.stl"), "bought", True, 10.0, "E", None, "bought", "#2e7d32", False)
    D["flow MTF-02P"] = (ld("flow_MTF-02P.stl"), "bought", True, 1.5, "P", None, "bought", "#2d5a2d", False)
    D["mount + nose legs (Scout)"] = (ld("mount_+_nose_legs_(Scout).stl"), "structure", False, 25.0, "E", "PETG", "printed", "#99a6b3", False)
    D["receiver v3 (latch)"] = (ld("receiver_v3_(latch).stl"), "protected", True, 94.4, "M", "PETG", "printed", "#8899aa", False)
    D["receiver lid (latch)"] = (ld("receiver_lid_(latch).stl"), "protected", True, 10.3, "M", "PETG", "printed", "#8899aa", False)
    D["receiver TPU strips (latch)"] = (ld("receiver_TPU_strips_(latch).stl"), "structure", True, 2.0, "M", "TPU", "printed", "#555555", False)
    D["receiver TPU end pieces (latch, E)"] = (ld("receiver_v2_skids_(latch,_E).stl"), "cage", False, 8.2, "M", "TPU", "printed", "#555555", False)
    D["XT60E1-M"] = (ld("XT60E1-M.stl"), "bought", True, 6.0, "E", None, "bought", "#c9a400", False)
    D["cartridge v6 (latch)"] = (ld("cartridge_v6_(latch).stl"), "protected", True, 37.9, "M", "PETG", "printed", "#99a6b3", True)
    D["slider +y (latch)"] = (ld("slider_+y_(latch).stl"), "protected", True, 2.9, "M", "PETG", "printed", "#7d8a99", True)
    D["slider -y (latch)"] = (ld("slider_-y_(latch).stl"), "protected", True, 2.9, "M", "PETG", "printed", "#7d8a99", True)
    D["cartridge v6 strips (latch, TPU)"] = (ld("cartridge_v6_skids_(latch,_E).stl"), "cage", False, 2.2, "M", "TPU", "printed", "#555555", True)
    D["cartridge skid closed end (latch, TPU)"] = (ld("cartridge_skid_closed_end_(latch,_TPU).stl"), "cage", False, 1.4, "M", "TPU", "printed", "#555555", True)
    D["cartridge skid lead end (latch, TPU)"] = (ld("cartridge_skid_lead_end_(latch,_TPU).stl"), "cage", False, 3.2, "M", "TPU", "printed", "#555555", True)
    D["pack 4S 1500"] = (ld("pack_4S_1500.stl"), "protected", True, 155.0, "P", None, "bought", "#2f4fbf", True)
    D["pack XT60 plug"] = (ld("pack_XT60_plug.stl"), "bought", True, 10.0, "E", None, "bought", "#c9a400", True)
    for n in CATCH:
        m = D[n][0].copy(); m.apply_translation([-DC, 0, 0]); D[n] = (m,) + D[n][1:]
    if TAIL_BAY:
        # tail bay (Spark 27 Oct: "a race car look", "lower the camera"): the Radxa and the UBEC ride behind the receiver's tail on the
        # belly mount, their front ends TAIL_GAP behind it (the low nose camera's moment balanced by them; the cage gets a long low tail)
        rt = float(D["receiver v3 (latch)"][0].bounds[0][0]) - TAIL_GAP
        for n, y0 in (("Radxa Zero 3W", -1.6), ("UBEC", None)):
            m = D[n][0].copy()
            if n == "Radxa Zero 3W" and RADXA_POSE == "stand" and ELEC and BAY_FIXED:   # alternative: the v11 pose fixed in the frame
                m = box(-145.4, -80.3, -RADXA_T / 2, RADXA_T / 2, RADXA_Z, RADXA_Z + 30.0)
            elif n == "Radxa Zero 3W" and RADXA_POSE == "stand":   # on edge on the centre plane, its front end TAIL_GAP + TB behind the receiver
                m = box(rt - TB - 65.0, rt - TB, -RADXA_T / 2, RADXA_T / 2, RADXA_Z, RADXA_Z + 30.0)
            elif n == "UBEC" and RADXA_POSE == "stand" and ELEC:   # the electronics session's worst-size envelope, 7 further out in y (8 of air
                (x0, y0, z0), (x1, y1, z1) = ELEC["items"]["UBEC"]["box"]; dx_ = 0.0 if BAY_FIXED else -(DC - ELEC["catch"])   # over the Radxa's SoC), riding with the catch
                m = box(x0 + dx_, x1 + dx_, y0 + UBEC_DY, y1 + UBEC_DY, z0, z1)
            elif n == "UBEC" and RADXA_POSE == "stand":       # flat behind the rear disks (out of the wash), beside the Radxa's aft half
                b_ = m.bounds; m.apply_translation([WASH_X - TB - b_[1][0], RADXA_T / 2 + 1.0 - b_[0][1], UBEC_Z - b_[0][2]])
            elif n == "Radxa Zero 3W":                        # v11: laid flat under the UBEC (in the rear wash: superseded)
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
                b_ = m.bounds; m.apply_translation([rt - b_[1][0], -0.5 * (b_[0][1] + b_[1][1]), 4.8 - b_[0][2]])
            else:
                b_ = m.bounds; m.apply_translation([rt - b_[1][0], 0.0 if y0 is None else y0 - b_[0][1], 0.0])
            D[n] = (m,) + D[n][1:]
    if TAIL_BAY and RADXA_POSE == "stand" and ELEC:            # the electronics session's XR2 shelf pose and the mount + cables as masses at
        dx_ = -(DC - ELEC["catch"])                            # their CGs; the bay rides with the catch (the belly mount), the XR2 held at its
        (x0, y0, z0), (x1, y1, z1) = ELEC["items"]["XR2"]["box"]   # v11 place in the frame (riding forward put its antenna in the rear prop
        D["XR2"] = (box(x0, x1, y0, y1, z0, z1),) + D["XR2"][1:]   # disk: its shelf grows aft by the catch shift, requirement for the mount)
        blk = ELEC["mount"]["belly-mount tail block (PETG)"]
        rest_g = ELEC["mount_total"]["mass_g"] - blk["mass_g"]
        rest_c = (np.array(ELEC["mount_total"]["cg"]) * ELEC["mount_total"]["mass_g"] - np.array(blk["cg"]) * blk["mass_g"]) / rest_g
        for nm_, g_, c_ in (("tail bay mount block (Scout electronics, E)", blk["mass_g"], np.array(blk["cg"]) + [dx_, 0, 0]),
                            ("tail bay carrier (Scout electronics, E)", rest_g, rest_c + [0.0 if BAY_FIXED else dx_, 0, 0]),
                            ("tail bay cables (Scout electronics, E)", ELEC["cables"]["mass_g"], np.array(ELEC["cables"]["cg"]) + [0.0 if BAY_FIXED else dx_, 0, 0])):
            wires_ = "cables" in nm_
            D[nm_] = (box(*(np.array([c_ - 0.2, c_ + 0.2]).T.ravel())), "structure", False, g_, "E", None if wires_ else "PETG", "wires" if wires_ else "printed", "#99a6b3", False)
    return D


def build(DC):
    """every drone-side global for a catch offset DC: parts, datum, ring, shell target, camera"""
    global D, TAB_X, CART_BOT, REC_B, Z_O, RING, nR, ZONE, BODY, props, body, K, CAM, GRIDS, CDC, SPARE_X, SPARE_ENDS
    D = drone(DC); CDC = DC
    TAB_X = float(np.mean([D["slider +y (latch)"][0].bounds[:, 0].mean(), D["slider -y (latch)"][0].bounds[:, 0].mean()]))
    CART_BOT = float(D["cartridge v6 (latch)"][0].bounds[0][2])
    REC_B = D["receiver v3 (latch)"][0].bounds
    Z_O = CART_BOT + RING_DZ - (SEAT_DZ if RING_MODE == "v5side" else 0.0)   # the Scout ring plane on the same datum (− the seat offset)
    RING = ring_nodes(); nR = len(RING)
    sx_ = 30.0 if RING_MODE not in ("rect", "v5side") else min(30.0, float(RING[:, 0].max()) - REC_B[1][0] - SHELL_R - 1.0, REC_B[0][0] - float(RING[:, 0].min()) - SHELL_R - 1.0)
    SPARE_X = sx_
    SPARE_ENDS = (float(RING[:, 0].max()) - REC_B[1][0] - SHELL_R - 1.0, REC_B[0][0] - float(RING[:, 0].min()) - SHELL_R - 1.0)
    ZONE = (REC_B[0][0] - sx_, REC_B[1][0] + sx_, REC_B[0][1] - 30.0, REC_B[1][1] + 30.0)   # swap zone: receiver + 30 a side (rect: + what the
                                                                                          # rectangle leaves along x, Spark 27 Oct)
    BODY = [n for n in D if not n.startswith("prop")]
    props = np.vstack([np.stack([mx + PROP_R * np.cos(a), my + PROP_R * np.sin(a), np.full_like(a, PROP_Z + s_ * BLADE)], 1)
                       for mx, my in MXY for s_ in (1, -1) for a in [np.linspace(0, 2 * np.pi, 72, endpoint=False)]])
    body = hullpts([D[n][0] for n in BODY])
    if RADXA_POSE == "stand" and TAIL_BAY and ELEC:          # the electronics session's mount boxes, item boxes and plug keep-clear (envelopes)
        dx_ = 0.0 if BAY_FIXED else -(DC - ELEC["catch"]); bx_ = [v["box"] for k_, v in ELEC["items"].items() if k_ not in ("UBEC", "XR2")] + [ELEC["keep_clear"]["plugs"]]
        x2_ = ELEC["items"]["XR2"]["box"]; body = np.vstack([body, box(x2_[0][0], x2_[1][0], x2_[0][1], x2_[1][1], x2_[0][2], x2_[1][2]).vertices])
        # (the mount's own boxes are bounding boxes: their lower aft corners reach where the real spine does not; the electronics
        # session keeps the mount ≥ 3 inside the shell and over the funnel line on its real geometry, envelope.json funnel_margin / inset)
        body = np.vstack([body] + [box(b_[0][0] + dx_, b_[1][0] + dx_, b_[0][1], b_[1][1], b_[0][2], b_[1][2]).vertices for b_ in bx_])
    if RADXA_POSE == "stand" and TAIL_BAY:                    # the Radxa's USB-C plugs: 14 above its ports edge (board X 37..58.5 from the SD end), kept clear
        rb_ = D["Radxa Zero 3W"][0].bounds; xa = rb_[0][0]
        body = np.vstack([body, box(xa + 36.0, xa + 60.0, -5.0, 5.0, rb_[1][2], rb_[1][2] + 14.0).vertices])
    K = np.vstack([(props[:, None, :] + M_PROP * DIRS[None]).reshape(-1, 3), (body[:, None, :] + M_BODY * DIRS[None]).reshape(-1, 3)])
    K[:, 2] = np.maximum(K[:, 2], Z_O)
    K = np.vstack([K, K * [1, -1, 1]]); K = K[ConvexHull(K).vertices]
    CAM = place_camera()
    D["camera"] = (cam_box(CAM["Lx"], CAM["Lz"]), "bought", True, 10.0, "E", None, "bought", "#2d5a2d", False)
    GRIDS = prop_grids()


# ---------------------------------------------------------------- the bottom ring
def overseer_ring():
    """the Overseer cage v3 opening ring (nodes and tubes) and its datum: centre = the catch (its cartridge envelope's centre,
    the grip line), plane offset from its cartridge flat bottom"""
    s = json.load(open(os.path.join(OV, "cage_shell.json")))
    N = np.array(s["nodes"]); ring = s["open"][0]["ring"]; R = N[ring]
    cart = trimesh.load(os.path.join(OV, "parts", "cartridge.stl"))
    cx = float(cart.bounds[:, 0].mean()); dz = float(R[:, 2].mean() - cart.bounds[0][2])
    rs = set(ring); tubes = [(ring.index(a), ring.index(b)) for a, b in s["rods"] if a in rs and b in rs]
    return R - [cx, 0, 0], dz, tubes, cx, float(cart.bounds[0][2])


R_OV, RING_DZ, RING_TUBES, OV_CX, OV_CB = overseer_ring()


def ring_nodes():
    if RING_MODE == "v5side":                                  # the Overseer cage v5 base loop's 4 side nodes (its nose and tail nodes dropped)
        return np.array([[TAB_X + x_, y_, Z_O] for x_, y_ in OV5_SIDE])
    if RING_MODE in ("overseer", "rect"):
        R = R_OV + [TAB_X, 0, 0] + [0, 0, Z_O - R_OV[:, 2].mean()]
        # rect (Spark 27 Oct: "get rid of the two trapezoids so the bottom is just a rectangle"): the octagon's 4 side nodes, the
        # nose and tail trapezoids dropped; the same funnel takes it on fewer ramps
        return R if RING_MODE == "overseer" else R[np.abs(R[:, 0] - TAB_X) < 0.75 * np.abs(R[:, 0] - TAB_X).max()]
    # alternative: the Overseer's formula round the Scout's own receiver (+30 a side), 8 nodes, grown until chords clear
    zx, zy = (REC_B[1][0] - REC_B[0][0]) / 2 + 30.0, (REC_B[1][1] - REC_B[0][1]) / 2 + 30.0; cxr = REC_B[:, 0].mean(); k = 1.0
    while True:
        a, b = (math.sqrt(2) * zx + RING_NODE_R) * k, (math.sqrt(2) * zy + RING_NODE_R) * k
        t = np.linspace(0, 2 * math.pi, 8, endpoint=False) + math.pi / 8
        R = np.stack([cxr + a * np.cos(t), b * np.sin(t), np.full(8, Z_O)], 1); ok = True
        for i in range(8):
            q = R[i] + np.linspace(0, 1, 50)[:, None] * (R[(i + 1) % 8] - R[i])
            if ((np.abs(q[:, 0] - cxr) < zx + SHELL_R + 2) & (np.abs(q[:, 1]) < zy + SHELL_R + 2)).any(): ok = False; break
        if ok: return R
        k *= 1.03




# ---------------------------------------------------------------- the shell target and the camera outside it
def hullpts(meshes, n=1500):
    P = np.vstack([pts(m, n) for m in meshes]); return P[ConvexHull(P).vertices]




def cam_box(Lx, Lz):
    return box(Lx - CAM_D, Lx, -CAM_W / 2, CAM_W / 2, Lz - CAM_H / 2, Lz + CAM_H / 2)


def lower_plane(Lx, Lz):
    e = math.radians(CAM_VFOV / 2 + 2.0); n = np.array([math.sin(e), 0, math.cos(e)]); return n, np.array([Lx, 0, Lz])


def place_camera():
    """the camera on the nose outside the shell: for each lens height and head-plane tilt, the least forward lens x that keeps
    the shell target LIM + SLACK under the camera's lower view plane (+2 deg) and behind the head plane (the camera's back,
    tilted phi from +x toward +z), and the ring nodes under the view plane; compact first (the least growth of the drone's
    box: shell estimate + camera + its guard bar), then the shortest boom from the plate nose"""
    best = None
    eqKR = ConvexHull(np.vstack([K, RING])).equations                # the camera body also outside the ring-to-target hull
    env = np.vstack([K, RING - RING_NODE_R, RING + RING_NODE_R])
    vol = lambda P: float(np.prod(P.max(0) - P.min(0))) / 1e6
    for Lz in np.arange(CAM_Z[0], CAM_Z[1] + 0.1, 2.0):
        for phi in PHIS:
            n3 = np.array([math.cos(math.radians(phi)), 0, math.sin(math.radians(phi))])
            for Lx in np.arange(70.0, 260.0, 1.0):
                n2, lens = lower_plane(Lx, Lz)
                if ((K - lens) @ n2).max() > -(LIM + SLACK) or ((RING - lens) @ n2).max() > -(RING_NODE_R + 3.0): continue
                cb = cam_box(Lx, Lz); d_b = float((cb.vertices @ n3).min())
                if (K @ n3).max() > d_b - LIM - SLACK: continue
                if ((cb.vertices @ eqKR[:, :3].T + eqKR[:, 3]).max(1) < LIM + SLACK).any(): continue
                back = np.array([Lx - CAM_D, 0, Lz])
                boom = float(np.linalg.norm(back - PLATE_NOSE))
                if CAM_MOUNT == "printed":                    # as close to the drone as its clear view allows (Spark 27 Oct: "let the
                    if Lz + CAM_H / 2 + LIP > K[:, 2].max() + 8.0: continue   # camera close to the drone, as compact as possible"), its cradle
                    cost = (float(Lx), float(Lz))             # lip no higher than the cage roof
                else:
                    cost = (round(vol(np.vstack([env, cb.vertices, cb.vertices + [0, 0, 3.0 + 2 * GUARD_NODE_R]])), 2), boom)
                if best is None or cost < best[0]: best = (cost, Lx, Lz, phi, n3, d_b)
                break
    (_, cost), Lx, Lz, phi, n3, d_b = best
    n2, lens = lower_plane(Lx, Lz)
    return dict(Lx=Lx, Lz=Lz, phi=phi, lens=lens, fwd=np.array([1.0, 0, 0]), up=np.array([0, 0, 1.0]), boom=cost,
                lims=[(n2, lens, LIM), (n3, n3 * d_b, LIM)])




def in_cam_view(P, extra_deg=2.0):
    d = P - CAM["lens"]; zz = d[:, 0]
    el = np.degrees(np.arctan2(d[:, 2], np.maximum(zz, 1e-6))); az = np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))
    return (zz > 0) & (np.abs(el) < CAM_VFOV / 2 + extra_deg) & (np.abs(az) < CAM_HFOV / 2 + extra_deg)


def views_clear(p0, p1, rad):
    q = np.asarray(p0)[None] + np.linspace(0, 1, 60)[:, None] * (np.asarray(p1) - np.asarray(p0))[None]
    return not in_cam_view(q).any()


# ---------------------------------------------------------------- the shell (the Overseer cage v3 method, ring fixed)
mir = np.array([1.0, -1.0, 1.0])


def partners(N):
    d, j = cKDTree(N).query(N * mir)
    assert d.max() < 1.0, f"shell nodes are not mirror pairs ({d.max():.2f})"
    return j


def sym(N, P):
    M = 0.5 * (N + N[P] * mir); M[P == np.arange(len(N)), 1] = 0.0; return M


def behind(N):
    N = N.copy()
    for _ in range(3):
        for n_, p0, lim_ in CAM["lims"]:
            over = (N - p0) @ n_ + lim_
            mv = over > 0; mv[:nR] = False
            N[mv] -= over[mv][:, None] * n_[None]
        N[nR:, 2] = np.maximum(N[nR:, 2], Z_O + 1.0)
    return N


def convex(N, U):
    viol = np.array([np.inf]); stuck = np.zeros(len(N), int); P = partners(N)
    for it in range(2000):
        h = ConvexHull(N)
        lost = np.setdiff1d(np.arange(len(N)), h.vertices); lost = np.union1d(lost, P[lost])
        if len(lost):
            lf = lost[lost >= nR]; stuck[lf] += 1
            drop = lf[stuck[lf] > (0 if PATTERN == "race" else 15)]     # race: an anchor inside the hull is not a corner there: drop it
            if len(drop):
                drop = np.union1d(drop, P[drop])
                keep = np.setdiff1d(np.arange(len(N)), drop); N, U, stuck = N[keep], U[keep], stuck[keep]; P = partners(N); continue
            if len(lf) == 0: return N, np.array([np.inf]), U
            N[lf] += 3.0 * U[lf]; N = sym(behind(N), P); continue
        viol = (K @ h.equations[:, :3].T + h.equations[:, 3]).max(0)
        if viol.max() <= 0.05: break
        step = np.zeros(len(N))
        for f, v in zip(h.simplices, viol):
            if v > 0.05:
                for a in f: step[a] = max(step[a], min(v, 15.0))
        step = np.maximum(step, step[P]); step[:nR] = 0
        N = sym(behind(N + (0.7 * step)[:, None] * U), P)
    return N, viol, U


def shell(START, DOME, SNAP=False):
    c0 = 0.5 * (K.min(0) + K.max(0)); c0[2] = PROP_Z
    hx, hy = np.ptp(K[:, 0]) / 2, np.ptp(K[:, 1]) / 2; ct, cb = max(K[:, 2].max() - c0[2], DOME), c0[2] - Z_O
    Ug = geodesic(FREQ)
    if START > 0:                                          # on an ellipsoid of the target's half extents x START
        Pg = c0 + START * np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1)
    else:                                                  # 0: each direction cast from the centre onto the target hull itself
        Dd = np.stack([hx * Ug[:, 0], hy * Ug[:, 1], np.where(Ug[:, 2] > 0, ct, cb) * Ug[:, 2]], 1); Dd /= np.linalg.norm(Dd, axis=1)[:, None]
        eqK = ConvexHull(K).equations; den = Dd @ eqK[:, :3].T; num = -(c0 @ eqK[:, :3].T + eqK[:, 3])
        Pg = c0 + np.where(den > 1e-9, num / np.where(den > 1e-9, den, 1), np.inf).min(1)[:, None] * Dd
    ex, ey = np.ptp(RING[:, 0]) / 2, np.ptp(RING[:, 1]) / 2; rcx = RING[:, 0].mean()
    in_open = (((Pg[:, 0] - rcx) / (1.25 * ex)) ** 2 + (Pg[:, 1] / (1.25 * ey)) ** 2 < 1.0) & (Ug[:, 2] < -1e-6)
    Pf, Uf = Pg[~in_open], Ug[~in_open]
    if PATTERN == "race":
        # race-car silhouette (Spark 27 Oct: "i want a race car look the overseer looks cooler"): 12 nodes on fixed fractions of
        # the target's own extents (a low nose point under the camera, shoulders at the front prop tips, mid sides, haunches at the
        # rear prop tips, a flat roof of 4 over the props, a tail point), grown START % about the centre; the camera's view limits
        # rake the nose, and the convex pass below pushes a node out only as far as the hull needs to hold every part
        lo_, hi_ = K.min(0), K.max(0); cx_ = 0.5 * (lo_[0] + hi_[0]); hx_ = 0.5 * (hi_[0] - lo_[0]); hy_ = hi_[1]; zt_ = hi_[2]
        g_ = 1.0 + START / 100.0
        sel = []
        for fx, fy, fz in RACE_ANCHORS:
            q = np.array([cx_ + g_ * fx * hx_, g_ * fy * hy_, Z_O + min(1.0, g_ * fz) * (zt_ - Z_O) + (g_ - 1.0) * max(0.0, fz - 0.9) * (zt_ - Z_O)])
            sel += [q] if fy == 0 else [q, q * mir]
        Pf = behind(np.vstack([RING, np.array(sel)]))[nR:]
    elif PATTERN == "free":
        # free pattern (Spark 27 Oct: "dont worry about same rod length anymore"): no geodesic spacing, any rod length; the nodes
        # are corners of the target grown by START mm (scanned), taken one at a time (a mirror pair, or one on the centre plane),
        # the one farthest outside the hull so far first, until the hull of the ring and the nodes holds the whole target: a
        # larger offset needs fewer nodes (fewer rods) for a slightly larger cage
        Kd = np.vstack([K + START * d_ for d_ in fib(NDIR)])
        if RACE:                                              # the race-car look (Spark 27 Oct): a flat roof (the grown target clipped at
            Kd[:, 2] = np.minimum(Kd[:, 2], K[:, 2].max() + 2.0)   # the target's own top), a low nose point under the camera and a raised
            lo_, hi_ = K.min(0), K.max(0)                     # tail point, both on the centre plane
            Kd = np.vstack([Kd, [[lo_[0] - RACE_TAIL[0], 0.0, hi_[2] - RACE_TAIL[1]]]]
                           + ([[[hi_[0] + RACE_NOSE[0], 0.0, Z_O + RACE_NOSE[1]]]] if RACE_NOSE[0] > 0 else []))
        Kd = Kd[ConvexHull(Kd).vertices]
        C = behind(np.vstack([RING, Kd[Kd[:, 2] > Z_O + 1.0]]))[nR:]
        C[np.abs(C[:, 1]) < 3.0, 1] = 0.0
        sel = [C[int(np.argmax(C[:, 2] - 0.01 * np.abs(C[:, 1])))] * [1, 0, 1]]           # the top, on the centre plane
        if not ARM_SPOKES and TIES == "saddle":               # a waist pair at mid-side, over the catch, just above the funnel band: the
            w_ = np.array([TAB_X, float(np.abs(RING[:, 1]).max()), Z_O + FUNNEL_BAND + 10.0])   # saddle ties land there (the arms carry none)
            sel += [w_, w_ * mir]
        for _ in range(60):
            h_ = ConvexHull(np.vstack([RING, sel]), qhull_options="QJ")
            if (K @ h_.equations[:, :3].T + h_.equations[:, 3]).max() <= 0.05: break
            v_ = (C @ h_.equations[:, :3].T + h_.equations[:, 3]).max(1)
            c_ = C[int(np.argmax(v_))]
            sel += [c_] if abs(c_[1]) < 3.0 else [c_, c_ * mir]
        Pf = np.array(sel)
    elif NFREE and len(Pf) > NFREE:
        # half the rods (Spark 27 Oct): keep NFREE of the free geodesic nodes, mirror pairs together, spread by farthest-point
        # sampling on the direction sphere from the top node (the pattern stays geodesic-spread, the cells grow)
        units, seen_ = [], set()
        for i in range(len(Uf)):
            if i in seen_: continue
            j = int(np.argmin(np.linalg.norm(Uf - Uf[i] * mir, axis=1)))
            units.append([i] if j == i else [i, j]); seen_.update([i, j])
        pick_ = [max(range(len(units)), key=lambda u: Uf[units[u][0], 2])]
        while sum(len(units[u]) for u in pick_) < NFREE:
            dmin = [min(np.linalg.norm(Uf[units[u][0]] - Uf[units[v][0]]) for v in pick_) if u not in pick_ else -1 for u in range(len(units))]
            pick_.append(int(np.argmax(dmin)))
        keep_ = [i for u in pick_ for i in units[u]]; Pf = Pf[keep_]
    N = np.vstack([RING, Pf]); N[nR:, 2] = np.maximum(N[nR:, 2], Z_O + 1.0)
    dirs = lambda P: (P - c0) / np.linalg.norm(P - c0, axis=1)[:, None]
    N = sym(N, partners(N)); U = dirs(N)
    N, viol, U = convex(behind(N), U)
    if not np.isfinite(viol.max()): return None
    # small triangles to one node (Spark 27 Oct: "reduce small triangles to one node"): a hull triangle whose every edge is under
    # SMALL_TRI collapses to one node (its free nodes merged at their centroid; a triangle and its mirror together, a self-mirror
    # one onto the centre plane; a free node in one with a ring node is dropped into the ring node); the shell is pushed again
    skip = set()                                             # collapses whose push did not close (reverted, review 27 Oct)
    hkey = lambda fr_: frozenset(tuple(np.round(N[v], 1)) for v in fr_)
    ztop = K[:, 2].max() + 2.0 * START + 40.0
    for _ in range(20):
        h_ = ConvexHull(N); P = partners(N); hit = None
        for f in h_.simplices:
            e_ = max(np.linalg.norm(N[f[i]] - N[f[(i + 1) % 3]]) for i in range(3))
            fr_ = [int(v) for v in f if v >= nR]
            if e_ < SMALL_TRI and len(fr_) >= 2 - (len(fr_) < len(f)) and hkey(fr_) not in skip: hit = (f, fr_); break
        if hit is None:                                      # a mirror pair closer than SMALL_TRI: one node on the centre plane
            near_ = [i for i in range(nR, len(N)) if P[i] > i and np.linalg.norm(N[i] - N[P[i]]) < SMALL_TRI and hkey([i, int(P[i])]) not in skip]
            if near_:
                i = near_[0]; hit = ([i, int(P[i])], [i, int(P[i])])
        if hit is None: break
        N0, U0, v0 = N.copy(), U.copy(), viol
        f, fr_ = hit
        if len(fr_) == 2 and len(f) == 2:                    # the mirror pair
            c_ = N[fr_].mean(0); c_[1] = 0.0; drop = set(fr_); newN = [c_]
        elif len(fr_) < 3:                                   # a ring node in it: the free ones go into the ring node
            drop = set(fr_) | {int(P[v]) for v in fr_}; newN = []
        else:
            mirr = {int(P[v]) for v in fr_}
            if mirr == set(fr_):                              # self-mirror triangle: one node on the centre plane
                c_ = N[fr_].mean(0); c_[1] = 0.0; drop = set(fr_); newN = [c_]
            else:                                             # the triangle and its mirror: one node each
                c_ = N[fr_].mean(0); drop = set(fr_) | mirr; newN = [c_, c_ * mir]
        keep = [i for i in range(len(N)) if i not in drop]
        N = np.vstack([N[keep]] + ([np.array(newN)] if newN else []))
        U = dirs(N); N, viol, U = convex(behind(sym(N, partners(N))), U)
        if not np.isfinite(viol.max()) or viol.max() > 0.05 or N[:, 2].max() > ztop:   # the push ran away (a node up the
            skip.add(frozenset(tuple(np.round(N0[v], 1)) for v in fr_)); N, U, viol = N0, U0, v0   # camera's limit plane): keep
                                                                                       # the triangle, try the next one
    if SNAP:
        # prop snap (the Overseer's, a mirror pair at a time): a free node over or under a prop disk tries the motor axis (the
        # hub, low thrust; one per prop and side) or out past the tip (R + 15) at its height, its partner the mirror move, both
        # then only vertically in a convex pass; kept only if the shell still converges, keeps every node, the props 12 off,
        # and its prop shadow drops
        def stats(Nn):
            h_ = ConvexHull(Nn); F = [f for f in h_.simplices if not (f < nR).all()]
            Ee = sorted({tuple(sorted((int(f[i]), int(f[(i + 1) % 3])))) for f in F for i in range(3)})
            pg_ = min(seg_band_dist(Nn[a_], Nn[b_]) for a_, b_ in Ee) - SHELL_R
            sh_ = sum(sum(seg_shadow(Nn[a_], Nn[b_], SHELL_R)) for a_, b_ in Ee)
            return pg_, sh_, len(h_.vertices) == len(Nn)
        _, sh0, _ = stats(N); Ud = U.copy(); used_axis = set()
        cand = [i for i in range(nR, len(N)) if N[i, 1] >= 0 and min(math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in MXY) < PROP_R + 15.0]
        for i in sorted(cand, key=lambda i: min(math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in MXY)):
            P = partners(N); j = P[i]
            k = int(np.argmin([math.hypot(N[i, 0] - mx, N[i, 1] - my) for mx, my in MXY])); mx, my = MXY[k]
            r = math.hypot(N[i, 0] - mx, N[i, 1] - my); side = 1 if N[i, 2] > PROP_Z else -1
            T_ = N.copy(); Ut = Ud.copy()
            if r < 0.6 * PROP_R and (k, side) not in used_axis: T_[i, :2] = (mx, my)
            else: T_[i, :2] = np.array([mx, my]) + (N[i, :2] - (mx, my)) / max(r, 1e-6) * (PROP_R + 15.0)
            if j != i: T_[j] = T_[i] * mir
            elif abs(T_[i, 1]) > 0.5: continue
            if np.min(np.linalg.norm(np.delete(T_, [i, j], 0) - T_[i], axis=1)) < 40.0: continue
            Ut[i] = Ut[j] = (0, 0, side)
            T_, v_, Ut = convex(T_, Ut)
            if len(T_) != len(N) or not np.isfinite(v_.max()) or v_.max() > 0.05: continue
            pg_, sh_, allv = stats(T_)
            if allv and pg_ >= PROP_GAP and sh_ < sh0 - 0.05:
                N, Ud, viol, sh0 = T_, Ut, v_, sh_
                if r < 0.6 * PROP_R:
                    used_axis.add((k, side)); used_axis.add((int(np.argmin([math.hypot(T_[j, 0] - a_, T_[j, 1] - b_) for a_, b_ in MXY])), side))
    # symmetric triangulation (as the Overseer's): coplanar hull faces merged into facets, a facet and its mirror share one
    # triangulation, a centre-plane facet keeps a self-mirror triangulation or is fanned from an on-plane vertex / centre node
    h = ConvexHull(N); P = partners(N); S = h.simplices; nS = len(S); par = list(range(nS))
    def root(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    for fi in range(nS):
        for fj in h.neighbors[fi]:
            opp = [v for v in S[fj] if v not in S[fi]]
            if opp and abs(N[opp[0]] @ h.equations[fi, :3] + h.equations[fi, 3]) < 0.3: par[root(fi)] = root(fj)
    facets = {}
    for fi in range(nS): facets.setdefault(root(fi), []).append(fi)
    facets = [fs for fs in facets.values() if not (np.unique(S[fs]) < nR).all()]          # the ring's own facet is the opening
    key = {frozenset(np.unique(S[fs]).tolist()): k for k, fs in enumerate(facets)}
    tris = []; extraN = []
    for k, fs in enumerate(facets):
        vs = np.unique(S[fs]); km = key.get(frozenset(P[vs].tolist()))
        if km is None: return None
        T_ = [tuple(sorted(int(v) for v in S[fi])) for fi in fs]
        if km != k:
            if N[vs, 1].mean() > 0: tris += T_ + [tuple(sorted(int(P[v]) for v in t)) for t in T_]
            continue
        if {tuple(sorted(int(P[v]) for v in t)) for t in T_} == set(T_): tris += T_; continue
        cnt = {}
        for t in T_:
            for i in range(3): e2 = tuple(sorted((t[i], t[(i + 1) % 3]))); cnt[e2] = cnt.get(e2, 0) + 1
        c = N[vs].mean(0); c[1] = 0.0
        onp = [int(v) for v in vs if abs(N[v, 1]) < 0.5 and np.linalg.norm(N[v] - c) < 60.0]
        if onp: ci = min(onp, key=lambda v: np.linalg.norm(N[v] - c))
        else: ci = len(N) + len(extraN); extraN.append(c)
        tris += [tuple(sorted((ci, a_, b_))) for (a_, b_), n_ in cnt.items() if n_ == 1 and ci not in (a_, b_)]
    if extraN: N = np.vstack([N, extraN])
    tris = np.array(sorted(set(tris)))
    used = sorted(set(tris.ravel()) | set(range(nR))); remap = {o: i for i, o in enumerate(used)}
    nodes = N[used]; tris = np.array([[remap[a] for a in f] for f in tris])
    E = sorted({tuple(sorted((int(f[i]), int(f[(i + 1) % 3])))) for f in tris for i in range(3)})
    E = sorted(set(E) | {tuple(sorted((remap[i], remap[(i + 1) % nR]))) for i in range(nR)})   # the ring rods (consecutive nodes)
    return nodes, E, tris, float(viol.max())


# ---------------------------------------------------------------- prop blockage (secondary to structure)
def prop_grids():
    from matplotlib.path import Path
    rr = np.linspace(0.2, 1.0, 24) * PROP_R; th = np.linspace(0, 2 * np.pi, 96, endpoint=False)
    Rg, Tg = np.meshgrid(rr, th); w = (Rg ** 2).ravel()
    pl = D["frame plate (Bee35 carbon)"][0]; plp = Path(pl.vertices[ConvexHull(pl.vertices[:, :2]).vertices, :2])
    out = []
    for mx, my in MXY:
        P = np.stack([mx + (Rg * np.cos(Tg)).ravel(), my + (Rg * np.sin(Tg)).ravel()], 1)
        shade = plp.contains_points(P) & (np.hypot(P[:, 0] - mx, P[:, 1] - my) < 14.0)   # the arm tip and motor only (E)
        out.append((P, w, shade))
    return out




def seg_shadow(p0, p1, rad):
    top = bot = 0.0; d = p1[:2] - p0[:2]; L2 = float(d @ d)
    for P, w, shade in GRIDS:
        t = np.clip(((P - p0[:2]) @ d) / L2, 0, 1) if L2 > 1e-9 else np.zeros(len(P))
        hit = (np.linalg.norm(P - (p0[:2] + t[:, None] * d), axis=1) <= rad) & ~shade
        z = p0[2] + t * (p1[2] - p0[2])
        top += (w * (hit & (z > PROP_Z))).sum() / w.sum(); bot += (w * (hit & (z < PROP_Z))).sum() / w.sum()
    return 100 * top / len(GRIDS), 100 * bot / len(GRIDS)


def blockage(nodes, E, extra):
    t = b = 0.0
    for a_, b_ in E:
        x, y = seg_shadow(nodes[a_], nodes[b_], SHELL_R); t += x; b += y
    for p, q in extra.values():
        x, y = seg_shadow(np.asarray(p, float), np.asarray(q, float), ROD_R); t += x; b += y
    return t, b


# ---------------------------------------------------------------- links: spokes, bay ties, camera guard
def place_cradle():
    """the camera cradle (Spark 27 Oct: "just let the camera mount to the cage, its light and dosent need the support from the
    drone body"): a printed back plate and two side cheeks round the camera, all behind the lens (out of its view), from the chin
    lip up to the guard bar nodes; tied to the shell by the guard bar struts and the cradle struts (attach), no boom to the plate"""
    global BOOM_AX
    BOOM_AX = []
    cb = D["camera"][0].bounds; xs = CAM["Lx"] - CAM_D - 4.0; xf = cb[1][0] - 2.0
    w = CAM_W / 2
    if CAM_MOUNT == "printed":                                # a closed PETG frame round the camera, 0.5 clear of its body (fit 0),
        z0, z1 = cb[0][2] - LIP, cb[1][2] + LIP                # LIP above and below it (the cage-first members over and under it), behind
        wo = w + 0.5 + 3.0                                     # the lens; the camera board screws to the back plate (TPU washers, DEFAULT)
        parts_ = [box(xs, xs + 4.0, -wo, wo, z0, z1)]
        parts_ += [box(xs, xf, sy * (w + 0.5) - (0.0 if sy > 0 else 3.0), sy * (w + 0.5) + (3.0 if sy > 0 else 0.0), z0, z1) for sy in (1, -1)]
        parts_ += [box(xs, xf, -wo, wo, z1 - 3.0, z1), box(xs, xf, -wo, wo, z0, z0 + 3.0)]
    else:
        z0, z1 = cb[0][2] - 3.0 - 4.0, cb[1][2] + 3.0 + GUARD_NODE_R
        parts_ = [box(xs, xs + 4.0, -w - 4.0, w + 4.0, z0, z1)] + [box(xs, xf, sy * (w + 1.0) - (3.0 if sy > 0 else 0.0), sy * (w + 1.0) + (0.0 if sy > 0 else 3.0), z0, z1) for sy in (1, -1)]
    cr = trimesh.util.concatenate(parts_)
    D[CAMP] = (cr, "cage" if CAM_MOUNT == "printed" else "structure", False, sum(abs(p_.volume) for p_ in parts_) * 1.27e-3 * 0.5, "E", "PETG", "printed", "#aab4c0", False)
    CAM.update(boom=0.0, boom_gap=float("nan"), boom_w=0.0, boom_sec=0.0, seat_z=(z0, z1), cradle=(xs, xf, z0, z1))


def place_boom(nodes, E):
    """the camera boom (PETG) from the plate nose to the camera seat: one 12 x 12 bar on the centre plane, or a fork of two
    10 x 10 bars at y ±w (mirror pair); root height, end height and w picked so the bars keep the most room to the shell
    tubes (needs half the bar diagonal + the tube radius + 1)"""
    global BOOM_AX
    lo, hi = CAM["Lz"] - CAM_H / 2 - BOOM / 2, CAM["Lz"] + CAM_H / 2 - BOOM / 2   # the bar may meet the seat under the camera
    T = [(nodes[a], nodes[b]) for a, b in E]
    def gap(a, b, sec):
        q = a + np.linspace(0, 1, 80)[:, None] * (b - a)
        return min(float(np.linalg.norm(q - (p0 + np.clip(((q - p0) @ (p1 - p0)) / ((p1 - p0) @ (p1 - p0)), 0, 1)[:, None] * (p1 - p0)), axis=1).min())
                   for p0, p1 in T) - sec / math.sqrt(2) - SHELL_R
    best = None
    for w in (0.0, 8.0, 12.0, 15.0, 18.0):
        sec = BOOM if w == 0 else 10.0
        for zr in (8.0, 14.0, 20.25, 26.0, 32.0, 38.0):
            for ze in np.arange(lo, hi + 0.1, 2.0):
                a = np.array([PLATE_NOSE[0], w, zr]); b = np.array([CAM["Lx"] - CAM_D - 4.0, w, ze])
                g_ = gap(a, b, sec)
                key = (g_ >= 1.0, -w, g_)                            # clear first, then a single bar, then the most room
                if best is None or key > best[0]: best = (key, a, b, w, sec, g_)
    _, a, b, w, sec, g_ = best
    bars = [(a, b)] if w == 0 else [(a, b), (a * mir, b * mir)]
    BOOM_AX = bars; meshes = []; bl = float(np.linalg.norm(b - a))
    for a_, b_ in bars:
        bm = trimesh.creation.box([bl, sec, sec]); bm.apply_transform(trimesh.geometry.align_vectors([1, 0, 0], (b_ - a_) / bl))
        bm.apply_translation((a_ + b_) / 2); meshes.append(bm)
    seat = box(CAM["Lx"] - CAM_D - 4.0, CAM["Lx"] - CAM_D, -CAM_W / 2, CAM_W / 2, min(CAM["Lz"] - CAM_H / 2, b[2] - sec / 2), CAM["Lz"] + CAM_H / 2)
    D[CAMP] = (trimesh.util.concatenate(meshes + [seat]), "structure", False,
                                       (len(bars) * bl * sec * sec * 0.5 + 4.0 * CAM_W * CAM_H * 0.6) * 1.27e-3, "E", "PETG", "printed", "#aab4c0", False)
    CAM.update(boom=bl, boom_gap=g_, boom_w=w, boom_sec=sec, seat_z=(float(seat.bounds[0][2]), float(seat.bounds[1][2])))
    return g_


def in_band(q, r=0.0):
    """the funnel band (design center 27 Oct): within FUNNEL_BAND above the plane, inside the bottom opening or up to 40 past its
    nose and tail ends (the ramps); only the ring rods (and the shell rods leaving the 4 bottom nodes outward) may be there"""
    q = np.atleast_2d(q)
    x0, x1 = float(RING[:, 0].min()), float(RING[:, 0].max())
    yw = np.interp(q[:, 0], [x0, x1], [np.abs(RING[RING[:, 0] < x0 + 1, 1]).max(), np.abs(RING[RING[:, 0] > x1 - 1, 1]).max()])
    inside = (q[:, 0] > x0 - 40.0) & (q[:, 0] < x1 + 40.0) & (np.abs(q[:, 1]) < yw + r)   # the 40 past the ends on the rod axis
    return inside & (q[:, 2] < Z_O + FUNNEL_BAND + r)


def usb_line(nodes, E, segs=()):
    """FC USB-C cable path (12 x 7 plug + cable, 250 long, E): from a point on the FC's tail or left edge, the direction nearest
    the tail-left diagonal (within 60 deg) that clears every part (but the mount and the wires) by 1 and the shell tubes and
    nodes by their radius + 7 + 1 (as the Overseer's access search)"""
    global USB
    fb = D["FC"][0].bounds; zc = 13.5
    starts = [np.array([fb[0][0] + 14.0, -12.7, zc]), np.array([fb[0][0] + 6.0, 0.0, zc]), np.array([fb[0][0] + 14.0, 12.7, zc])]
    others = [D[n][0] for n in D if n not in ("FC", "mount + nose legs (Scout)", CAMP) and not n.startswith("prop")]
    pq = [(m, trimesh.proximity.ProximityQuery(m)) for m in others]
    best = None
    for p0 in starts:
        for dv in fib(600):
            if abs(dv[2]) > 0.5: continue
            ang = math.degrees(math.acos(np.clip(dv @ np.array([-0.7071, -0.7071, 0.0]), -1, 1)))
            if ang > 60.0 or (best is not None and ang >= best[0]): continue
            q = p0 + np.linspace(0.0, 250.0, 250)[:, None] * dv
            ok = True
            for a_, b_ in [(nodes[a], nodes[b]) for a, b in E] + [(np.asarray(p_, float), np.asarray(q_, float)) for p_, q_ in segs]:
                u = b_ - a_; t = np.clip(((q - a_) @ u) / (u @ u), 0, 1)
                if (np.linalg.norm(q - (a_ + t[:, None] * u), axis=1) < SHELL_R + 7.0 + 2.0).any(): ok = False; break
            if not ok or (np.linalg.norm(q[:, None, :] - nodes[None], axis=2) < NODE_R + 7.0 + 1.0).any(): continue
            for m, pq_ in pq:
                near = ((q > m.bounds[0] - 9.0) & (q < m.bounds[1] + 9.0)).all(1)
                if near.any() and (pq_.signed_distance(q[near]) > -4.0).any(): ok = False; break           # the 7 tall plug, 0.5 clear (system_check sweeps the true 12 x 7)
            if ok: best = (ang, p0, dv)
    USB = (best[1], best[2]) if best else (np.array([-10.8 - CDC, -12.7, zc]), np.array([-0.7071, -0.7071, 0.0]))
    return best is not None


def cradle_arms(nodes, E, clear, add, extra=None):
    """printed PETG arms (round, Ø ARM) from the cradle's back corners into the cage: to the nearest shell node whose rods leave
    clear of the arm (its joint: the node sphere + 2, review 27 Oct), else to a printed snap clip on the nearest shell rod (a C
    round the rod, its mouth across the arm and up, pushed on after the cage is built, Spark 27 Oct); 3+ non-collinear supports (no pivot); behind the camera's view,
    props 12 off, clear of every other rod; the arm roots at the cage are the printed fuse (they let go before the camera)"""
    xs, xf, z0, z1 = CAM["cradle"]; wo = CAM_W / 2 + 3.5
    T = [(nodes[a], nodes[b]) for a, b in E] + [(np.asarray(a_, float), np.asarray(b_, float)) for a_, b_ in (extra or {}).values()]
    def rod_clear(p0, p1, nk, jr):                            # the round arm clear of every shell rod and link by 0.3 outside its
        L_ = float(np.linalg.norm(p1 - p0)); ax = (p1 - p0) / L_                # joint (within jr of nk)
        h = ARM / 2 + SHELL_R + 0.3
        for a_, b_ in T:
            q = a_ + np.linspace(0, 1, max(2, int(np.linalg.norm(b_ - a_))))[:, None] * (b_ - a_)
            q = q[np.linalg.norm(q - nk, axis=1) > jr] - p0
            t = q @ ax
            if ((t >= 0) & (t <= L_) & (np.linalg.norm(q - t[:, None] * ax, axis=1) < h)).any(): return False
        return True
    def free(c_, k):                                          # an arm from c_ into node k, its end face inside the node sphere
        u = (nodes[k] - c_) / np.linalg.norm(nodes[k] - c_); e_ = nodes[k] - u * (math.sqrt(NODE_R ** 2 - (ARM / 2) ** 2) - 0.1)
        ok = seg_band_dist(c_, nodes[k]) - ARM / 2 >= PROP_GAP and views_clear(c_, nodes[k], ARM / 2) and rod_clear(c_, e_, nodes[k], NODE_R + 2.0)
        return e_ if ok else None
    sup, ends, clamps = [], [], []                            # support points, arms (root, end), clamps (centre, rod direction)
    for zc in (z1 - ARM / 2, z0 + ARM / 2):
        c_ = np.array([xs - 0.5, wo - ARM / 2, zc])                 # from the back face of the cradle's back plate (clear of the camera)
        opts = []                                             # every clear way in: the shortest arm wins
        for k in [k for k in range(nR, len(nodes)) if nodes[k][1] > -1.0 and nodes[k][0] < xs and np.linalg.norm(nodes[k] - c_) < 90.0]:
            if any(np.linalg.norm(nodes[k] - s_) < 1.0 for s_ in sup): continue
            if sup and abs(nodes[k][1]) < 1.0 and abs(sup[0][1]) < 1.0: continue   # not 2 supports on the centre plane (a pivot)
            e_ = free(c_, k)
            if e_ is not None: opts.append((float(np.linalg.norm(e_ - c_)), nodes[k], e_, None))
        for a, b in E:                                        # a snap clip on a shell rod (off its nodes, off the centre plane)
            pa, pb = nodes[a], nodes[b]; p_ = _closest(c_, pa, pb); u = (pb - pa) / np.linalg.norm(pb - pa)
            if np.linalg.norm(p_ - c_) > 90.0 or min(np.linalg.norm(p_ - pa), np.linalg.norm(p_ - pb)) < NODE_R + CLAMP_L / 2 + 4.0 \
                    or abs(p_[1]) < CLAMP_L or p_[0] >= xs or any(np.linalg.norm(p_ - s_) < CLAMP_L for s_ in sup): continue
            d_ = (c_ - p_) / np.linalg.norm(c_ - p_); e_ = p_ + d_ * (CLAMP_R - 1.0)
            if not (views_clear(c_, p_, ARM / 2) and seg_band_dist(c_, p_) - ARM / 2 >= PROP_GAP and rod_clear(c_, e_, p_, CLAMP_L / 2 + 1.0)): continue
            if not rod_clear(p_ - u * CLAMP_L / 2, p_ + u * CLAMP_L / 2, p_, CLAMP_L / 2 + 1.0): continue   # (the clamp: other rods clear)
            opts.append((float(np.linalg.norm(e_ - c_)), p_, e_, u))
        got = min(opts, key=lambda o: o[0])[1:] if opts else None
        if got is None: raise SystemExit("camera cradle: no clear printed arm from a back corner into the cage")
        sup.append(got[0]); ends.append((c_, got[1], got[0], got[2] is not None))
        if got[2] is not None: clamps.append((got[0], got[2], (c_ - got[0]) / np.linalg.norm(c_ - got[0])))
    meshes = []
    for c_, n_ in [(e[0], e[1]) for e in ends] + [(e[0] * mir, e[1] * mir) for e in ends]:
        L_ = float(np.linalg.norm(n_ - c_)); bm = trimesh.creation.cylinder(radius=ARM / 2, height=L_, sections=24)
        bm.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (n_ - c_) / L_)); bm.apply_translation((c_ + n_) / 2); meshes.append(bm)
    D["camera cradle arms (printed)"] = (trimesh.util.concatenate(meshes), "cage", False, sum(abs(m_.volume) for m_ in meshes) * 1.27e-3 * 0.6, "E", "PETG", "cage", "#7d8a99", False)
    cm = []
    for p_, u, d_ in clamps + [(p_ * mir, u * mir, d_ * mir) for p_, u, d_ in clamps]:
        m_ = np.cross(u, d_); m_ /= np.linalg.norm(m_); m_ = m_ if m_[2] >= 0 else -m_    # the mouth: across the arm, up (the weight
        w_ = np.cross(u, m_)                                                              # bears on the clip's closed side)
        F = np.eye(4); F[:3, 0], F[:3, 1], F[:3, 2], F[:3, 3] = w_, m_, u, p_          # local frame: x across, y the mouth, z the rod
        body = trimesh.creation.cylinder(radius=CLAMP_R, height=CLAMP_L, sections=48)
        bore = trimesh.creation.cylinder(radius=CLIP_BORE / 2, height=CLAMP_L + 2.0, sections=48)
        slot = trimesh.creation.box([CLIP_MOUTH, CLAMP_R + 1.0, CLAMP_L + 2.0]); slot.apply_translation([0, (CLAMP_R + 1.0) / 2, 0])
        bm = trimesh.boolean.difference([body, bore, slot], engine="manifold"); bm.apply_transform(F); cm.append(bm)
    if cm:
        D["camera cradle clips (printed)"] = (trimesh.util.concatenate(cm), "cage", False, sum(abs(m_.volume) for m_ in cm) * 1.27e-3 * 0.8, "E", "PETG", "cage", "#7d8a99", False)
    CAM["arms"] = [(np.round(c_, 1).tolist(), np.round(p_, 1).tolist(), round(float(np.linalg.norm(e_ - c_)), 1), cl) for c_, e_, p_, cl in ends]
    CAM["clamps"] = len(cm)


def _closest(q, a, b):
    u = b - a; return a + np.clip(((q - a) @ u) / (u @ u), 0, 1) * u


def tie_saddle():
    ms = []
    for sy in (1, -1):
        y0, y1 = (SADDLE_Y if sy > 0 else (-SADDLE_Y[1], -SADDLE_Y[0]))
        ms.append(box(-50.0, 42.0, y0, y1, SADDLE_Z[0], SADDLE_Z[1]))
        for xa, xb in ((-26.0, -14.0), (14.0, 26.0)):
            ms.append(box(xa, xb, y0, y1, SADDLE_Z[1], 18.5))
    m = trimesh.util.concatenate(ms)
    D["tie saddle (printed)"] = (m, "cage", False, sum(abs(m_.volume) for m_ in ms) * 1.27e-3 * 0.5, "E", "PETG", "printed", "#7d8a99", False)


def attach(nodes, ring_ids, E=()):
    place_cradle() if CAM_MOUNT in ("cage", "printed") else place_boom(nodes, E)
    if not ARM_SPOKES and TIES == "saddle": tie_saddle()
    else: D.pop("tie saddle (printed)", None)
    usb_ok = usb_line(nodes, E)
    extra = {}
    keep_out = [D[n][0].bounds + [[-ROD_R - 1] * 3, [ROD_R + 1] * 3] for n in
                ("receiver v3 (latch)", "receiver lid (latch)", "FC", "UBEC", "XR2", "Radxa Zero 3W", "flow MTF-02P", "cartridge v6 (latch)")]
    keys_ = [np.array(k) - [CDC, 0] for k in ((-42.65, 26.55), (-42.65, -26.55), (43.25, 26.55), (43.25, -26.55))]   # receiver bolt keys (d 4)

    usb0, usb_d = USB                                        # FC USB-C cable line (12 x 7, 250 long)

    def seg_d(q, a, b):
        u = b - a; t = np.clip(((q - a) @ u) / (u @ u), 0, 1); return np.linalg.norm(q - (a + t[:, None] * u), axis=1)

    acc = []                                                 # accepted links (and their mirrors): no two rods through each other

    def link_gap(q, a_, b_):
        r_ = np.asarray(a_) + np.linspace(0.08, 0.92, 40)[:, None] * (np.asarray(b_) - np.asarray(a_))
        return float(np.linalg.norm(q[:, None, :] - r_[None], axis=2).min())

    def clear(p0, p1):                                       # a link and its mirror (mirror-exact links)
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        if not (clear1(p0, p1) and clear1(p0 * mir, p1 * mir)): return False
        self_m = np.linalg.norm(p0 * mir - p1) + np.linalg.norm(p1 * mir - p0) < 1.0   # a rod across the centre plane is its own mirror
        if not self_m and np.linalg.norm(p0 - p0 * mir) + np.linalg.norm(p1 - p1 * mir) > 1.0:   # a link and its own mirror apart
            q = p0 + np.linspace(0.08, 0.92, 40)[:, None] * (p1 - p0)
            if link_gap(q, p0 * mir, p1 * mir) < 2 * ROD_R + 1.0: return False
        return True

    def clear1(p0, p1):
        q = np.asarray(p0)[None] + np.linspace(0.04, 0.96, 60)[:, None] * (np.asarray(p1) - np.asarray(p0))[None]
        if USB_FIRST and (seg_d(q, usb0, usb0 + 250.0 * usb_d) < 7.0 + ROD_R + 1.0).any(): return False
        if any((seg_d(q, a_, b_) < CAM['boom_sec'] / math.sqrt(2) + ROD_R + 1.0).any() for a_, b_ in BOOM_AX): return False
        if any(((q > b_[0]) & (q < b_[1])).all(1).any() for b_ in keep_out): return False
        if any(((np.linalg.norm(q[:, :2] - k, axis=1) < 2.0 + 1.0 + ROD_R) & (q[:, 2] < -20.0)).any() for k in keys_): return False
        if ((q[:, 0] > ZONE[0]) & (q[:, 0] < ZONE[1]) & (q[:, 1] > ZONE[2]) & (q[:, 1] < ZONE[3]) & (q[:, 2] < CART_BOT)).any(): return False
        if (in_band(q)).any(): return False                                             # the funnel band (the ramps inside the opening), rod axis
        qi = q[(np.linspace(0.04, 0.96, 60) > 0.08) & (np.linspace(0.04, 0.96, 60) < 0.92)]
        if any(link_gap(qi, a_, b_) < 2 * ROD_R + 1.0 for a_, b_ in acc): return False
        return views_clear(p0, p1, ROD_R)

    def add(name, a_, b_):
        extra[name] = (a_, b_); acc.append((np.asarray(a_, float), np.asarray(b_, float)))
        acc.append((np.asarray(a_, float) * mir, np.asarray(b_, float) * mir))

    def pick(p0, cands, n, spread_deg, name):
        """n links from p0 among the 10 nearest valid candidates: least added prop shadow (secondary to structure; the links
        spread at least spread_deg apart), length as the tie-break; each taken link is checked against the ones before it"""
        cands = sorted(cands, key=lambda k: np.linalg.norm(nodes[k] - p0))[:14]
        cost = {k: sum(seg_shadow(p0, nodes[k], ROD_R)) + 0.002 * np.linalg.norm(nodes[k] - p0) for k in cands}
        out = []
        for k in sorted(cands, key=cost.get):
            u = (nodes[k] - p0) / np.linalg.norm(nodes[k] - p0)
            if all(np.degrees(np.arccos(np.clip(u @ v, -1, 1))) >= spread_deg for v in [(nodes[j] - p0) / np.linalg.norm(nodes[j] - p0) for j in out]) \
                    and clear(p0, nodes[k]):
                out.append(k); add(f"{name}.{len(out)}", p0, nodes[k])
            if len(out) == n: break
        return out
    side = lambda P, sx, sy: (sx * P[0] > -1.0) and (sy * P[1] > -1.0)
    for i, (mx, my) in enumerate(MXY if ARM_SPOKES else []):   # 3 spokes per arm tip (under the motor, the carbon arm) to shell nodes
        tip = np.array([mx, my, 18.5 - ROD_R])                # on its own side (docs/generative.md 0), props 12 off
        if my < 0: continue                                   # the -y arms are the mirrors
        ok = [k for k in range(len(nodes)) if side(nodes[k], np.sign(mx), np.sign(my)) and seg_band_dist(tip, nodes[k]) - ROD_R >= PROP_GAP]
        ns = 2 if LEAN else 3
        got = pick(tip, ok, ns, 30.0, f"spoke {i + 1}")
        if len(got) < ns: pick(tip, [k for k in ok if k not in got], ns - len(got), 20.0, f"spoke {i + 1}x")
    if not ARM_SPOKES and TIES == "saddle":                  # saddle ties (Spark 27 Oct: "the rods to the arms need to be on the cartridge on the
        for i, xr in enumerate(SADDLE_X):                     # underside because the arms are just carbon fiber"): from the tie saddle's outer
            r0 = np.array([xr, SADDLE_Y[1] + ROD_R + 0.5, SADDLE_Z[0] + 8.0])          # face beside the receiver (the replaceable piece), above
            ok = [k for k in range(nR, len(nodes)) if nodes[k][1] > -1.0 and seg_band_dist(r0, nodes[k]) - ROD_R >= PROP_GAP]   # the funnel band,
            pick(r0, ok, 1, 0.0, f"saddle tie {i + 1}")                                             # to the nearest clear shell node on its side
    rb = REC_B                                               # bay ties: the receiver's side corners, above the funnel band, to the nearest shell
    for sx in ((1, -1) if ARM_SPOKES else ()):            # (with the arm spokes only: their roots sit in the TPU end pieces, review 27 Oct)                                       # nodes on their side (design center 27 Oct: nothing but the ring rods in the band);
        for zc_ in np.arange(Z_O + FUNNEL_BAND + ROD_R + 1.0, rb[1][2] - 2.0, 6.0):   # 1 a corner when lean, nose and tail; the lowest start
            cn = np.array([rb[1][0] if sx > 0 else rb[0][0], rb[1][1], zc_])            # on the receiver's side that finds a clear node
            ok = [k for k in range(nR, len(nodes)) if side(nodes[k], sx, 1) and nodes[k][2] >= cn[2] and seg_band_dist(cn, nodes[k]) - ROD_R >= PROP_GAP]
            if pick(cn, ok, 1 if LEAN else 2, 15.0, f"bay tie {sx:+d}+1"): break
    pb = D["frame plate (Bee35 carbon)"][0].bounds           # core ties: 2 from each plate edge (underside) into the shell
    for nm, q0, sx, sy in (("nose", (pb[1][0] - 6.0, 0.0), 1, 0), ("tail", (pb[0][0] + 6.0, 0.0), -1, 0), ("left", (0.0, 44.0), 0, 1))[3 if (LEAN and TIES != "core") else 0:3]:   # lean: none (the spokes, bay ties and camera braces tie the core; Spark 27 Oct: half the rods)
        q = np.array([q0[0], q0[1], 18.5 - ROD_R])
        ok = [k for k in range(len(nodes)) if (sx == 0 or sx * nodes[k][0] > 0) and (sy == 0 or sy * nodes[k][1] > 0)
              and seg_band_dist(q, nodes[k]) - ROD_R >= PROP_GAP]
        pick(q, ok, 1 if (LEAN or TIES == "core") else 2, 25.0, f"core tie {nm}")
    # camera: a guard bar of 2 printed nodes over its top behind the lens, 2 struts each into the shell; the seat braced from its
    # back corners into the shell and from its lower corners to the front arm beams 30 out of the plate (a braced pyramid on the
    # core with the boom, not a stalk); the boom root lug at the plate nose is the printed fuse (requirement)
    cb = D["camera"][0].bounds; x = cb[1][0] - GUARD_NODE_R - 3.0
    T = [np.array([x, sy * (CAM_W / 2 + 4.0), cb[1][2] + 3.0 + GUARD_NODE_R]) for sy in (1, -1)]
    guard = []
    if CAM_MOUNT != "printed" and views_clear(T[0], T[1], ROD_R):
        add("cam guard bar", T[0], T[1]); guard = T
        ok = [k for k in range(len(nodes)) if seg_band_dist(T[0], nodes[k]) - ROD_R >= PROP_GAP]
        pick(T[0], ok, 1 if LEAN else 2, 20.0, "cam guard strut 0")
    # chin guard (27 Oct: the low camera with no nose point under it, Spark "get rid of that sharp section under the camera"): a
    # printed lip on the camera seat under the camera, behind its lens (out of its view), 3 under the camera body; it links printed
    # parts close together, so it is printed, not carbon (CLAUDE.md carbon rule); the cage-first member for a hit from below-forward
    cl = box(CAM["Lx"] - CAM_D - 4.0, cb[1][0] - 2.0, -CAM_W / 2 - 6.0, CAM_W / 2 + 6.0, cb[0][2] - 3.0 - 4.0, cb[0][2] - 3.0)
    CAM["chin"] = cl
    sz = CAM["seat_z"]; xs = CAM["Lx"] - CAM_D - 4.0
    if CAM_MOUNT == "cage":                                   # the cradle's low back corners into the shell (the top is tied by the guard
        c_ = np.array([xs, CAM_W / 2 + 4.0, sz[0] + ROD_R])     # bar struts): the camera hangs from the cage only, triangulated, not a stalk
        ok = [k for k in range(len(nodes)) if nodes[k][1] > -1.0 and seg_band_dist(c_, nodes[k]) - ROD_R >= PROP_GAP]
        if not pick(c_, ok, 1, 0.0, "cam cradle strut lo"):
            raise SystemExit("camera cradle: no clear strut from its low corners into the shell")
    for lvl, zc in ((("lo", sz[0] + ROD_R), ("hi", sz[1] - ROD_R)) if not LEAN else ()):   # lean: the boom + the arm braces carry the seat
        c_ = np.array([xs, CAM_W / 2 - ROD_R, zc])
        ok = [k for k in range(len(nodes)) if nodes[k][1] > -1.0 and seg_band_dist(c_, nodes[k]) - ROD_R >= PROP_GAP]
        pick(c_, ok, 1, 0.0, f"cam brace {lvl}")
    c_ = np.array([xs, CAM_W / 2 - ROD_R, sz[0] + ROD_R]); u_ = np.array([1.0, 1.0, 0]) / math.sqrt(2)
    for out_ in ((30.0, 40.0, 50.0, 60.0) if CAM_MOUNT == "boom" else ()):
        root = np.array([*(out_ * u_[:2]), 18.5 - ROD_R])
        if seg_band_dist(c_, root) - ROD_R >= PROP_GAP and clear(c_, root):
            add("cam brace arm.1", c_, root); break
    else:                                                     # no clear root on the arm beam: brace the seat into the shell instead
        if CAM_MOUNT == "boom":
            ok = [k for k in range(len(nodes)) if nodes[k][1] > -1.0 and seg_band_dist(c_, nodes[k]) - ROD_R >= PROP_GAP]
            pick(c_, ok, 1, 0.0, "cam brace shell")
    if CAM_MOUNT == "boom" and not any(k.startswith("cam brace") for k in extra):
        raise SystemExit("camera seat unbraced (a stalk): no clear brace to the arms or the shell")
    if CAM_MOUNT != "printed":
        D["camera chin guard (printed)"] = (CAM["chin"], "cage", False, abs(CAM["chin"].volume) * 1.27e-3 * 0.5, "E", "PETG", "cage", "#7d8a99", False)
    else:
        cradle_arms(nodes, E, clear, add, extra)
    have = [(np.asarray(a_), np.asarray(b_)) for a_, b_ in extra.values()]   # mirror-exact links
    for k_, (a_, b_) in list(extra.items()):
        a_, b_ = np.asarray(a_) * mir, np.asarray(b_) * mir
        if not any(min(np.linalg.norm(a_ - p_) + np.linalg.norm(b_ - q_), np.linalg.norm(a_ - q_) + np.linalg.norm(b_ - p_)) < 1.0 for p_, q_ in have):
            extra[k_ + " m"] = (a_, b_); have.append((a_, b_))
    if not USB_FIRST:                                        # the links first, then the FC USB-C cable path round them (and the shell)
        usb_line(nodes, E, [v for v in extra.values()] + [(np.asarray(a_, float) * mir, np.asarray(b_, float) * mir) for a_, b_ in extra.values()])
    return extra, guard


def masses(nodes, E, extra, guard):
    it = [(n, D[n][3], ctr_(D[n][0])) for n in D]                    # true centroids (review 27 Oct: box centres put the receiver 4.85 off)
    it.append(("wires", 20.0, D["FC"][0].bounding_box.centroid))
    for i, (a, b) in enumerate(E): it.append((f"shell {i}", np.linalg.norm(nodes[a] - nodes[b]) * SHELL_GPM, (nodes[a] + nodes[b]) / 2))
    for k, (p, q) in extra.items(): it.append((k, np.linalg.norm(np.subtract(q, p)) * ROD_GPM, (np.asarray(p) + np.asarray(q)) / 2))
    for i, (n, g) in enumerate(zip(nodes, node_masses(nodes, E, extra))): it.append((f"node {i}", g, n))
    if CAMP in D and CAM_MOUNT == "printed":                 # the camera mount at its saved mass (the layout's cradle, arms and clips are rough)
        mod = sum(D[k][3] for k in (CAMP, "camera cradle arms (printed)", "camera cradle clips (printed)") if k in D)
        it.append(("camera mount (saved mass delta)", CAM_MOUNT_G - mod, ctr_(D[CAMP][0])))
    for i, n in enumerate(guard): it.append((f"guard node {i}", GUARD_NODE_G, n))
    return it


def ctr_(m):
    try:
        if m.is_watertight and m.volume > 1e-6: return np.asarray(m.center_mass)
    except Exception: pass
    return np.asarray(m.bounding_box.centroid)


def cg(it):
    m = sum(x for _, x, _ in it); return m, sum(x * np.asarray(c, float) for _, x, c in it) / m


# ---------------------------------------------------------------- scan: structure first, then the least prop shadow; balance
def run(DC):
    """the drone with the catch DC aft of the thrust centre, its shell scan (start x dome: the most compact valid shell sets
    the bar, the least prop shadow within 10 % volume / 3 % mass of it wins), links and masses"""
    build(DC); lines = []; cands = []
    combos = [(s_, d_, n_) for s_ in (0.0, 0.7, 0.85, 1.0) for d_ in (20.0, 40.0, 60.0) for n_ in SNAPS] if PATTERN not in ("free", "race") else \
        [(o_, 20.0, False) for o_ in OFFSETS]                 # free: START is the corner offset (mm), no dome, no prop snap
    for START, DOME, SNAP in combos:
        if START == 0.0 and DOME > 20.0: continue                 # cast onto the target: the dome does not apply
        r = shell(START, DOME, SNAP)
        if r is None: lines.append(f"start {START} dome {DOME:.0f} snap {int(SNAP)}: invalid (no closed mirror shell)"); continue
        nodes, E, tris, viol = r
        pg = min(seg_band_dist(nodes[a], nodes[b]) for a, b in E) - SHELL_R
        vol = ConvexHull(nodes).volume / 1e6; sh = sum(np.linalg.norm(nodes[a] - nodes[b]) for a, b in E)
        mass = sh * SHELL_GPM + node_masses(nodes, E).sum()
        ok = viol <= 0.05 and pg >= PROP_GAP
        bt, bb = blockage(nodes, E, {}) if ok else (np.nan, np.nan)
        lines.append(f"{'offset' if PATTERN in ('free', 'race') else 'start'} {START} dome {DOME:.0f} snap {int(SNAP)}: {'valid' if ok else 'invalid'}: {len(nodes)} nodes, {len(E)} tubes, {vol:.1f} L, "
                     f"{np.ptp(nodes[:, 2]):.0f} tall, shell {mass:.0f} g, props {pg:.1f}, shell shadow {bt:.1f} / {bb:.1f} %")
        if ok: cands.append(dict(start=START, dome=DOME, snap=SNAP, vol=vol, mass=mass, block=bt + bb, r=r))
    if PATTERN in ("free", "race"):                           # half the rods (Spark 27 Oct): shells within the shell rod cap, the
        cap_ = [c for c in cands if len(c["r"][1]) <= SHELL_CAP] or [min(cands, key=lambda c: len(c["r"][1]))]   # most compact,
        for c in cap_:                                        # among the shells balanced on their own (x moment about the thrust
            nd_, E_ = c["r"][0], c["r"][1]                    # centre under 400 g mm, ~0.6 mm of CG: a lopsided shell leaves the
            c["mom"] = sum(np.linalg.norm(nd_[a] - nd_[b]) * SHELL_GPM * (nd_[a][0] + nd_[b][0]) / 2 for a, b in E_) + (node_masses(nd_, E_) * nd_[:, 0]).sum()
        def small_(c):                                        # triangles with every edge under SMALL_TRI left in the final shell
            nd_, _, tr_, _ = c["r"]
            return sum(max(np.linalg.norm(nd_[t[i]] - nd_[t[(i + 1) % 3]]) for i in range(3)) < SMALL_TRI for t in tr_)
        short_ = lambda c: min(np.linalg.norm(c["r"][0][a] - c["r"][0][b]) for a, b in c["r"][1])
        cap_ = [c for c in cap_ if small_(c) == 0 and short_(c) >= MIN_ROD] or cap_   # no small or sliver triangles (Spark 27 Oct) first
        bal_ = [c for c in cap_ if abs(c["mom"]) <= 1500.0] or cap_   # the catch scan balances the rest; a lopsided shell beyond this is out
        ref = pick = min(bal_, key=lambda c: c["vol"])
    else:
        ref = min(cands, key=lambda c: c["vol"])
        fit = [c for c in cands if c["vol"] <= 1.1 * ref["vol"] and c["mass"] <= 1.03 * ref["mass"]]
        pick = min(fit, key=lambda c: c["block"])
    nodes, E, tris, viol = pick["r"]
    lines.append(f"structure bar: start {ref['start']} dome {ref['dome']:.0f} ({ref['vol']:.1f} L, {ref['mass']:.0f} g); picked start "
                 f"{pick['start']} dome {pick['dome']:.0f} snap {int(pick['snap'])}")
    extra, guard = attach(nodes, list(range(nR)), E)
    it = masses(nodes, E, extra, guard); tot, cgv = cg(it)
    t2, c2 = cg([x for x in it if not D.get(x[0], (0,) * 9)[8]])
    return dict(DC=DC, lines=lines, pick=pick, nodes=nodes, E=E, tris=tris, viol=viol, extra=extra, guard=guard, it=it, tot=tot, cgv=cgv,
                t2=t2, c2=c2, bal=max(math.hypot(cgv[0], cgv[1]), math.hypot(c2[0], c2[1])) + feas_(nodes, E, extra)
                + LEN_W * max(0.0, float(np.ptp(nodes[:, 0])) - 320.0))      # the CG off the thrust axis (x and y), plus length (compact)


def feas_(nodes, E, extra):
    """balance-scan penalty for a layout that breaks a gate the scan cannot see: over the 62-rod rule (Scout clause) or a tail node in
    the Overseer's tail-tip ramp (under 45 above the plane past x -130 about the catch, design center 27 Oct)"""
    pen = 100.0 * max(0, len(E) + len(extra) - 62)
    tl_ = [n[2] - Z_O for n in nodes[nR:] if n[0] - TAB_X < -130.0]
    return pen + (50.0 + 45.0 - min(tl_) if tl_ and min(tl_) < 45.0 else 0.0)


os.makedirs(OUT, exist_ok=True)
# balance (CLAUDE.md 4, L Balance: pack in and pack out within 3 mm): the catch set (receiver, cartridge, pack, belly mount,
# Radxa, flow) and the ring slide aft of the thrust centre by DC; the shell and the camera are rebuilt round it each time; the DC
# with the least worse of the two CG offsets wins
SNAPS = [bool(int(x)) for x in os.environ.get("SCOUT_SNAPS", "0,1").split(",")]
DCS = [float(x) for x in os.environ.get("SCOUT_DCS", "-16,-12,-8,-4,0,4").split(",")]
runs = {}
TBS = [float(x) for x in os.environ.get("SCOUT_TBS", ("25" if ELEC else "0,5,10,15,20,25,30,35,40") if RADXA_POSE == "stand" else "0").split(",")]
best_tb = None                                                         # Spark 27 Oct: the CG in the middle of the props, pack in and out:
for TB in TBS:                                                         # the tail bay shift and the catch together (two knobs, two CGs)
    runs = {}
    for DC in DCS:
        runs[DC] = run(DC); r_ = runs[DC]
        say(f"   tail bay +{TB:4.1f}, catch {DC:4.1f} aft: CG pack in {r_['cgv'][0]:+.1f}, pack out {r_['c2'][0]:+.1f} (x, mm), {r_['tot']:.0f} g, {len(r_['E']) + len(r_['extra'])} rods, score {r_['bal']:.1f}")
    for _ in range(5):                                                 # refine round the best
        b0 = min(runs, key=lambda d: runs[d]["bal"]); step = min([abs(d - b0) for d in runs if d != b0] + [4.0]) / 2
        if step < 0.25: break
        for DC in (b0 - step, b0 + step):
            if DC >= -30 and DC not in runs:
                runs[DC] = run(DC); r_ = runs[DC]
                say(f"   tail bay +{TB:4.1f}, catch {DC:4.1f} aft: CG pack in {r_['cgv'][0]:+.1f}, pack out {r_['c2'][0]:+.1f} (x, mm), {r_['tot']:.0f} g, {len(r_['E']) + len(r_['extra'])} rods, score {r_['bal']:.1f}")
    b0 = min(runs, key=lambda d: runs[d]["bal"])
    if best_tb is None or runs[b0]["bal"] < best_tb[2]: best_tb = (TB, b0, runs[b0]["bal"])
TB, BEST, _ = best_tb
R_ = run(BEST)                                                         # rebuild the globals for the pick
DC = BEST; nodes, E, tris, viol, extra, guard, it, tot, cgv = (R_[k] for k in ("nodes", "E", "tris", "viol", "extra", "guard", "it", "tot", "cgv"))
pick = R_["pick"]; ring_ids = list(range(nR))
say(f"Scout v3 layout v3 / cage v2 (pattern {PATTERN}, every rod 3 mm); ring {RING_MODE}: {nR} nodes; balance: tail bay +{TB:.1f}, catch {DC:.1f} aft of the thrust centre; tab line x {TAB_X:.2f}, "
    f"cartridge flat bottom z {CART_BOT:.2f}, ring plane z {Z_O:.2f} (Overseer: catch x {OV_CX:.1f}, cartridge bottom {OV_CB:.1f}, ring {RING_DZ:+.2f} under it)")
say(f"camera: lens ({CAM['Lx']:.0f}, 0, {CAM['Lz']:.0f}), head plane tilt {CAM['phi']:.0f} deg, " + ("in a printed cradle on the cage (no boom) " if CAM_MOUNT in ("cage", "printed") else f"boom {CAM['boom']:.0f} from the plate nose ")
    + f"(lens z {CAM_Z[0]:.0f}..{CAM_Z[1]:.0f} x tilt 0..90: the shell target {LIM + SLACK:.1f} under its lower view plane and behind its back; "
    + ("closest to the drone with its cradle under the cage roof)" if CAM_MOUNT == "printed" else "least growth of the drone's box, then the shortest boom)"))
for l_ in R_["lines"]: say(l_)

# ---------------------------------------------------------------- checks
sh = np.array([np.linalg.norm(nodes[a] - nodes[b]) for a, b in E]); ext = nodes.max(0) - nodes.min(0)
say(f"shell: {len(nodes)} nodes ({nR} on the ring), {len(E)} tubes, {len(tris)} triangles; tubes {sh.min():.0f} .. {sh.max():.0f}, "
    f"{sh.sum() / 1000:.2f} m; node extents {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}; hull residual {viol:.2f} (C)")
ringE = [e for e in E if e[0] < nR and e[1] < nR]
if RING_MODE in ("overseer", "rect"):
    Rd = R_OV - [0, 0, R_OV[:, 2].mean()]
    dR = max(np.linalg.norm(Rd - (q - [TAB_X, 0, Z_O]), axis=1).min() for q in nodes[:nR])
    say(f"ring: {nR} of the Overseer's {len(R_OV)} ring nodes " + ("(the whole octagon)" if RING_MODE == "overseer" else "(its middle rectangle, the nose and tail trapezoids dropped)")
        + f", each within {dR:.3f} mm of its node on the same datum, {len(ringE)} ring rods, extents {np.ptp(nodes[:nR, 0]):.1f} x {np.ptp(nodes[:nR, 1]):.1f} (C)")
if RING_MODE == "v5side":
    say(f"ring: the 4 side nodes of the Overseer cage v5 base loop (its nose and tail nodes dropped; Spark 27 Oct), about the catch "
        + ", ".join(f"({x_:+.1f}, {y_:+.1f})" for x_, y_ in OV5_SIDE) + f", {np.ptp(nodes[:nR, 0]):.1f} long, {2 * OV5_SIDE[1][1]:.1f} wide at the tail, "
        + f"{2 * OV5_SIDE[0][1]:.1f} at the nose; plane z {Z_O:.2f} = cartridge flat bottom {RING_DZ:+.2f} − seat {SEAT_DZ:.2f} (the 3.0 rods sit 2.12 lower on "
        + f"the 45 deg ramps than the Overseer's 6 mm tubes, so both cartridges stop at the same height); {len(ringE)} ring rods (C)")
    smp = lambda a_, b_: np.asarray(a_) + np.linspace(0.02, 0.98, 80)[:, None] * (np.asarray(b_) - np.asarray(a_))
    nl_ = sum(int(in_band(smp(p_, q_)).any()) for p_, q_ in extra.values()); nl1_ = sum(int(in_band(smp(p_, q_), ROD_R).any()) for p_, q_ in extra.values())
    ns_ = [(a, b) for a, b in E if min(a, b) >= nR and in_band(smp(nodes[a], nodes[b]), SHELL_R).any()]
    nr_ = [(a, b) for a, b in E if (a < nR) != (b < nR) and in_band(smp(nodes[a], nodes[b])[8:], SHELL_R).any()]
    say(f"funnel band (within {FUNNEL_BAND:.0f} above the plane, inside the opening or 40 past its ends): links in it {nl_} (rod axis; {nl1_} graze its edge by up to the rod radius), shell rods between free "
        f"nodes {len(ns_)}, shell rods leaving the 4 bottom nodes that stay in it past their first 10 % {len(nr_)} "
        + (f"({', '.join(f'{a}-{b}' for a, b in nr_)})" if nr_ else "") + " (C)")
if TAIL_BAY:                                                           # the rear prop wash (rule 8; Spark 27 Oct: the tail bay out of it): the
    wash_ = []                                                         # tail bay parts' plan points against every disk + 3 (under the props)
    rb_ = D["Radxa Zero 3W"][0].bounds
    plug_ = box(rb_[0][0] + 36.0, rb_[0][0] + 60.0, -5.0, 5.0, rb_[1][2], rb_[1][2] + 14.0)
    extra_w = []
    if ELEC:                                                           # the mount's spine section (y -13.05..-4.6, its summary) and tail block,
        dxw = 0.0 if BAY_FIXED else -(DC - ELEC["catch"])              # bounding boxes shifted with the bay (their real geometry is the
        sp_ = (ELEC["mount"].get("tail carrier: yoke + spine (PETG)") or ELEC["mount"]["tail carrier (PETG)"])["box"]; bk_ = ELEC["mount"]["belly-mount tail block (PETG)"]["box"]   # electronics session's)
        extra_w = [("mount spine (envelope)", box(sp_[0][0] + dxw, sp_[1][0] + dxw, -13.05, -4.6, sp_[0][2], sp_[1][2])),
                   ("mount tail block (envelope)", box(bk_[0][0] - (DC - ELEC["catch"]), bk_[1][0] - (DC - ELEC["catch"]), bk_[0][1], bk_[1][1], bk_[0][2], bk_[1][2])),
                   ("XR2", D["XR2"][0])]
    for n_, m_ in [("Radxa Zero 3W", D["Radxa Zero 3W"][0]), ("UBEC", D["UBEC"][0]), ("Radxa plugs (keep clear)", plug_)] + extra_w:
        v_ = m_.vertices; v_ = np.vstack([v_, m_.sample(400)])
        d_ = min(float((np.hypot(v_[:, 0] - mx, v_[:, 1] - my) - PROP_R).min()) for mx, my in MXY)
        wash_.append(f"{n_} {d_:+.1f}")
    say("tail bay out of the prop wash (plan distance outside every disk, need ≥ 3): " + ", ".join(wash_) + " (C)")
    if ELEC:                                                           # the electronics session's tool lines: clear of every rod (Ø10 tool)
        dx_ = 0.0 if BAY_FIXED else -(DC - ELEC["catch"]); segs_ = [(nodes[a], nodes[b]) for a, b in E] + [(np.asarray(p_, float), np.asarray(q_, float)) for p_, q_ in extra.values()]
        out_ = []
        for nm_, t_ in ELEC["keep_clear"]["tool_lines"].items():
            q0 = np.array(t_["start"]) + [dx_, 0, 0]; dv = np.array(t_["dir"]) / np.linalg.norm(t_["dir"])
            qq = q0 + np.linspace(0, 80.0, 161)[:, None] * dv
            dmin = min(float(np.min(np.linalg.norm(qq - (a_ + np.clip(((qq - a_) @ (b_ - a_)) / ((b_ - a_) @ (b_ - a_)), 0, 1)[:, None] * (b_ - a_)), axis=1))) for a_, b_ in segs_)
            out_.append(f"{nm_} {dmin - t_['d'] / 2 - SHELL_R:+.1f}")
        say("electronics tool lines (Ø10, 80 out) clear of every rod by: " + ", ".join(out_) + " (C; need ≥ 0)")
    tl_ = [n for n in nodes[nR:] if n[0] - TAB_X < -130.0]
    say(f"tail nodes past x -130 about the catch: lowest {min([n[2] - Z_O for n in tl_] + [999]):.1f} above the plane (the Overseer tail-tip ramp, need ≥ 45) (C)")
circ = []
for f in tris:
    a_, b_, c_ = nodes[f]; la, lb, lc = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_); s_ = (la + lb + lc) / 2
    circ.append(2 * math.sqrt(max(s_ * (s_ - la) * (s_ - lb) * (s_ - lc), 1e-9)) / s_)
say(f"largest cell: inscribed circle Ø{max(circ) - 2 * SHELL_R:.0f} between tube surfaces (C)")
rl = {k: float(np.linalg.norm(np.subtract(q, p))) for k, (p, q) in extra.items()}
short = [l for l in sh if l < BREAK_EVEN]
kinds = {}
for k_ in extra: kinds[" ".join(k_.split()[:2]).split(".")[0]] = kinds.get(" ".join(k_.split()[:2]).split(".")[0], 0) + 1
say("links by kind: " + ", ".join(f"{k} {v}" for k, v in sorted(kinds.items())))
say(f"carbon: shell {len(sh)} rods 3.0 solid, shortest {sh.min():.0f}" + (f" ({len(short)} under {BREAK_EVEN:.0f}: printed into the node)" if short else "")
    + f"; links {len(rl)} rods 3.0; total {len(sh) + len(rl)} rods (cage v1: 123), {sum(rl.values()) / 1000:.2f} m, shortest {min(rl.values()):.0f} ({min(rl, key=rl.get)})"
    + (f"; printed (< {BREAK_EVEN:.0f}): " + ", ".join(f"{k} {v:.0f}" for k, v in rl.items() if v < BREAK_EVEN) if any(v < BREAK_EVEN for v in rl.values()) else ""))
LL = list(extra.items()); lgap = (np.inf, "")
for i_ in range(len(LL)):                                # link to link: no two rods through each other (interior 8..92 %)
    for j_ in range(i_ + 1, len(LL)):
        (ka, (a0, a1)), (kb, (b0, b1)) = LL[i_], LL[j_]
        qa = np.asarray(a0) + np.linspace(0.08, 0.92, 40)[:, None] * (np.asarray(a1) - np.asarray(a0))
        qb = np.asarray(b0) + np.linspace(0.08, 0.92, 40)[:, None] * (np.asarray(b1) - np.asarray(b0))
        d_ = float(np.linalg.norm(qa[:, None] - qb[None], axis=2).min()) - 2 * ROD_R
        if d_ < lgap[0]: lgap = (d_, f"{ka} / {kb}")
say(f"links apart: closest pair {lgap[0]:.1f} between rod surfaces ({lgap[1]}) (C, need ≥ 1.0)")
pg = min(seg_band_dist(nodes[a], nodes[b]) for a, b in E) - SHELL_R
pn = min(seg_band_dist(n, n) for n in nodes) - NODE_R
pe = {k: seg_band_dist(np.asarray(p, float), np.asarray(q, float)) - ROD_R for k, (p, q) in extra.items()}
say(f"props: shell tubes {pg:.1f} off the swept disks (≥ 12, 3D), nodes {pn:.1f}, links {min(pe.values()):.1f} ({min(pe, key=pe.get)}) (C)")
eqs = ConvexHull(nodes).equations
for nm in ("props", "body"):
    P_ = props if nm == "props" else body
    P_ = P_[P_[:, 2] > Z_O]
    say(f"   inside: {nm} {float((-(P_ @ eqs[:, :3].T + eqs[:, 3])).min(1).min()):.1f} from the nearest face plane (C)")
zone = box(ZONE[0], ZONE[1], ZONE[2], ZONE[3], Z_O - 200, CART_BOT)
from topo.part import man
blk = [i for i, (a, b) in enumerate(E) if (man(zone) ^ man(rod(nodes[a], nodes[b], SHELL_R))).volume() > 0.5]
blk += [k for k, (p, q) in extra.items() if (man(zone) ^ man(rod(p, q))).volume() > 0.5]
say(f"swap zone (receiver {REC_B[1][0] - REC_B[0][0]:.1f} x {REC_B[1][1] - REC_B[0][1]:.1f} + {SPARE_X:.1f} at the ends (nose {SPARE_ENDS[0]:.1f}, tail {SPARE_ENDS[1]:.1f} to the rectangle rods), 30 at the sides, cartridge bottom down): "
    + ("clear (0 mm³)" if not blk else "BLOCKED by " + ", ".join(map(str, blk))))
seen = []
memb = [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), ROD_R) for p, q in extra.values()]
for k, P_ in (("shell tubes", np.vstack([rod(nodes[a], nodes[b], SHELL_R).vertices for a, b in E])), ("shell nodes", np.vstack([n + NODE_R * DIRS for n in nodes])),
              ("links + guard", np.vstack([pts(rod(p0, p1, rr), 200) for p0, p1, rr in memb[len(E):]])), ("props", props),
              ("drone", np.vstack([pts(D[n][0], 1500) for n in D if n not in ("camera", CAMP)])),
              ("camera mount", pts(D[CAMP][0], 1500))):
    n_ = int(in_cam_view(P_, 0.0).sum())
    if n_: seen.append(f"{k} {n_} points")
say(f"front camera (horizon, landscape, {CAM_HFOV:.0f} x {CAM_VFOV:.0f} deg): " + ("nothing of the drone in its view (C)" if not seen else "in view: " + "; ".join(seen)))
fl = D["flow MTF-02P"][0].bounds; flens = np.array([fl[:, 0].mean(), fl[:, 1].mean(), fl[0][2]])
for k, m in (("shell", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E])), ("links", trimesh.util.concatenate([rod(p, q) for p, q in extra.values()])),
             ("cartridge set", trimesh.util.concatenate([D[n][0] for n in D if D[n][8]])), ("receiver", D["receiver v3 (latch)"][0])):
    d = pts(m, 4000) - flens; zz = -d[:, 2]
    n_ = ((zz > 0) & (np.abs(np.degrees(np.arctan2(d[:, 0], np.maximum(zz, 1e-6)))) < 23.0) & (np.abs(np.degrees(np.arctan2(d[:, 1], np.maximum(zz, 1e-6)))) < 23.0)).mean()
    if n_: say(f"   {k} in the flow view ({100 * n_:.1f} % of its points)")
say("flow (23 deg cone, straight down): checked (lines above list anything in view)")
BT, BB = blockage(nodes, E, extra)
say(f"prop blockage (secondary to structure): {BT:.1f} % of the disks' thrust shadowed from above, {BB:.1f} % from below (plan shadow of tubes and links, "
    f"annulus 0.2..1 R weighted by r, E)")
nodes_g = sum(x for k, x, _ in it if k.startswith("node ")); shell_g = sh.sum() * SHELL_GPM + nodes_g; link_g = sum(rl.values()) * ROD_GPM
t2, c2 = cg([x for x in it if not D.get(x[0], (0,) * 9)[8]])
say(f"cage: shell {shell_g:.0f} g (tubes {sh.sum() * SHELL_GPM:.0f} + nodes {nodes_g:.0f}, E: {NODE_MODEL} model), links {link_g:.0f} g, guard nodes "
    f"{GUARD_NODE_G * len(guard):.0f} g (C on E)")
say(f"mass ~{tot:.0f} g (E), CG ({cgv[0]:.1f}, {cgv[1]:.1f}, {cgv[2]:.1f}) vs thrust centre (0, 0): {math.hypot(*cgv[:2]):.1f} off; pack + cartridge out: "
    f"{math.hypot(*c2[:2]):.1f} off ({t2:.0f} g); CG {PROP_Z - cgv[2]:.0f} under the prop plane")
say(f"compact: shell hull {ConvexHull(nodes).volume / 1e6:.1f} L vs the parts-with-margins hull {ConvexHull(K).volume / 1e6:.1f} L (C)")
# funnel interchangeability: the Scout's shell inside the Overseer's shell in the band just above the shared ring
if RING_MODE in ("overseer", "rect"):
    so = json.load(open(os.path.join(OV, "cage_shell.json"))); No = np.array(so["nodes"])
    No = No - [OV_CX, 0, R_OV[:, 2].mean()] + [TAB_X, 0, Z_O]                     # the Overseer on the same datum
    eo = ConvexHull(No).equations; band = {}
    # the funnel bears on the outside of the Overseer's lower shell: a Scout tube inside the Overseer's tube-axis hull cannot reach
    # it; the margin per 20 mm band above the ring (the low band holds the tubes leaving the shared ring nodes)
    q = np.vstack([nodes[a] + np.linspace(0, 1, 80)[:, None] * (nodes[b] - nodes[a]) for a, b in E if not (a < nR and b < nR)])
    q = q[np.linalg.norm(q[:, None, :] - nodes[None, :nR], axis=2).min(1) > RING_NODE_R]          # off the shared ring nodes
    dq = (-(q @ eo[:, :3].T + eo[:, 3])).min(1)
    for H in (20.0, 40.0, 60.0, 80.0):
        m_ = (q[:, 2] > Z_O + H - 20.0) & (q[:, 2] <= Z_O + H)
        band[H] = float(dq[m_].min()) if m_.any() else float("nan")
    say("funnel: Scout shell tube axes inside the Overseer's tube-axis hull (same datum, outside the shared ring nodes) by "
        + ", ".join(f"{v:.1f} at {H - 20:.0f}..{H:.0f} above the ring" for H, v in band.items())
        + " (C; positive = inside; the Overseer's rods are 6 mm, the Scout's 3.0: a funnel that takes the Overseer's ring and lower shell takes the Scout's)")
else:
    band = {}

SUMMARY = dict(ring_mode=RING_MODE, seat_dz=SEAT_DZ if RING_MODE == "v5side" else 0.0, tab_x=TAB_X, cart_bottom=CART_BOT, z_o=Z_O, ring=np.round(nodes[:nR], 3).tolist(), cam=dict(lens=CAM["lens"].tolist(),
               phi=CAM["phi"], boom=CAM["boom"], hfov=CAM_HFOV, vfov=CAM_VFOV), start=pick["start"], dome=pick["dome"], nodes=len(nodes), tubes=len(E),
               shell_m=float(sh.sum() / 1000), shell_g=float(shell_g), link_g=float(link_g), mass=float(tot), cg=cgv.tolist(), cg_pack_out=c2.tolist(),
               extents=ext.tolist(), prop_gap=float(pg), block_top=BT, block_bottom=BB, largest_cell=max(circ) - 2 * SHELL_R, funnel_band=band,
               links={k: [np.round(np.asarray(a_), 2).tolist(), np.round(np.asarray(b_), 2).tolist()] for k, (a_, b_) in extra.items()})
json.dump(SUMMARY, open(os.path.join(OUT, "result.json"), "w"), indent=1)
json.dump([dict(name=n_, g=round(float(m_), 3), c=np.round(c_, 2).tolist()) for n_, m_, c_ in it], open(os.path.join(OUT, "masses.json"), "w"))
SHELL_SPEC = dict(nodes=np.round(nodes, 3).tolist(), rods=[list(map(int, e)) for e in E],
                  links=len(extra), link_r_mm=ROD_R, total_rods=len(E) + len(extra),
                  open=[dict(name={"v5side": "bottom opening (pack + swap): the Overseer cage v5 base loop's 4 side nodes", "overseer": "bottom opening (pack + swap): the Overseer's ring", "rect": "bottom opening (pack + swap): the Overseer ring's middle rectangle"}.get(RING_MODE, "bottom opening (pack + swap)"),
                             ring=list(range(nR)))], node_r_mm=NODE_R, rod_r_mm=SHELL_R)
json.dump(SHELL_SPEC, open(os.path.join(OUT, "cage_shell.json"), "w"))

# ---------------------------------------------------------------- export: parts, assembly file
grp = {}
for n_, gm, _c in it:
    k = "shell" if n_.startswith("shell") else "nodes" if n_.startswith("node") else "guard nodes" if n_.startswith("guard node") else \
        "links" if n_.split()[0] in ("spoke", "bay", "core", "cam", "saddle") else n_
    grp[k] = grp.get(k, 0.0) + gm
PARTS = [("cage shell tubes", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E]), "cage", False, grp["shell"], "E", "carbon", "cage", "#2b2b2b", False),
         ("cage shell nodes", trimesh.util.concatenate([trimesh.creation.icosphere(1, RING_NODE_R if i < nR else NODE_R).apply_translation(n) for i, n in enumerate(nodes)]), "cage", False, grp["nodes"], "E", "PETG", "cage", "#7d8a99", False),
         ("spokes, bay ties, core ties, camera guard", trimesh.util.concatenate([rod(p, q) for p, q in extra.values()]), "cage", False, grp["links"], "C", "carbon", "cage", "#444444", False)]
if guard:
    PARTS.append(("camera guard nodes", trimesh.util.concatenate([trimesh.creation.icosphere(1, GUARD_NODE_R).apply_translation(g_) for g_ in guard]), "cage", False,
                  grp["guard nodes"], "E", "PETG", "cage", "#7d8a99", False))
for n_, (m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_) in D.items():
    PARTS.append((n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_))
PD = os.path.join(OUT, "parts"); os.makedirs(PD, exist_ok=True)
fname = lambda n_: "".join(ch if ch.isalnum() else "_" for ch in n_.lower().replace("+", "p").replace("-y", "my")).strip("_")[:40] + ".stl"
for p_ in PARTS: p_[1].export(os.path.join(PD, fname(p_[0])))
json.dump([dict(name=n_, file=fname(n_), role=r_, protect=pr_, mass_g=round(float(gm_), 2), tag=t_, group=gr_, color=co_, swap=sw_)
           for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS], open(os.path.join(PD, "parts.json"), "w"), indent=1)
rel = lambda p_: os.path.relpath(p_, ROOT)
LINKS = "spokes, bay ties, core ties, camera guard"            # (the part name kept; it holds the saddle ties too)
ACCESS = [{"name": f"swap: {p} down", "part": p, "dir": [0, 0, -1], "length_mm": 150.0, "tool": "part", "ignore": SW_IGN}
          for p in ("cartridge v6 (latch)", "pack 4S 1500", "pack XT60 plug")]
for x_, y_ in ((-42.65, -26.55), (-42.65, 26.55), (43.25, -26.55), (43.25, 26.55)):
    ACCESS.append({"name": f"M3 key, receiver bolt ({x_ - DC:+.1f}, {y_:+.1f})", "part": "receiver v3 (latch)", "from": [x_ - DC, y_, 9.6], "dir": [0, 0, -1],
                   "length_mm": 150.0, "tool": {"type": "cylinder", "d": 4},
                   "ignore": SW_IGN + ["receiver lid (latch)", "mount + nose legs (Scout)", "XT60E1-M", "receiver TPU end pieces (latch, E)"]})
ACCESS.append({"name": "FC USB-C cable (12 x 7, searched path, E)", "part": "FC", "from": np.round(USB[0], 2).tolist(), "dir": np.round(USB[1], 4).tolist(),
               "length_mm": 250.0, "tool": {"type": "box", "w": 12, "h": 7, "w_axis": np.round(np.cross([0, 0, 1.0], USB[1]) / max(np.linalg.norm(np.cross([0, 0, 1.0], USB[1])), 1e-9), 4).tolist()},
               "ignore": ["mount + nose legs (Scout)", "wires"]})
say(f"FC USB-C cable: from ({USB[0][0]:.1f}, {USB[0][1]:.1f}, {USB[0][2]:.1f}) along ({USB[1][0]:.2f}, {USB[1][1]:.2f}, {USB[1][2]:.2f}), "
    f"{math.degrees(math.acos(np.clip(USB[1] @ np.array([-0.7071, -0.7071, 0.0]), -1, 1))):.0f} deg off the tail-left diagonal, out through a cell (C)")
if CAM_MOUNT == "printed":
    say("camera mount: a printed PETG cradle round the camera (0.5 clear, lips 6 over and under, behind the lens) on 4 printed Ø9 round arms into "
        + ", ".join(f"{'a snap clip on the shell rod at' if cl_ else 'node'} {tuple(n_)} ({l_:.0f} long)" for _, n_, l_, cl_ in CAM["arms"]) + " and their mirrors: no rods, no boom (Spark 27 Oct)")
if CAM_MOUNT != "printed": say("camera ties: " + ", ".join(k for k in extra if k.startswith(("cam brace", "cam cradle", "cam guard strut"))) + (" (the cradle hangs from the cage only, Spark 27 Oct)" if CAM_MOUNT == "cage" else " (the seat on the boom + these: not a stalk)"))
if CAM_MOUNT == "boom": say(f"camera boom: {'one bar' if CAM['boom_w'] == 0 else 'a fork of 2 bars at y ±%.0f' % CAM['boom_w']}, {CAM['boom']:.0f} long, "
    f"{CAM['boom_sec']:.0f} x {CAM['boom_sec']:.0f} PETG from the plate nose to the camera seat, {CAM['boom_gap']:.1f} clear of the shell tubes (C)")
if OUT == os.path.join(ROOT, "runs", "scout_layout", "v13", ""):
    from assembly import new as new_assembly
    asm_parts = []
    for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS:
        p_ = dict(name=n_, role=r_, mesh=rel(os.path.join(PD, fname(n_))), mass_g=round(float(gm_), 2), tag=t_, owner="scout v3",
                  version="layout v3 / cage v2", source="nox/scout/v3/layout_v3.py")
        if n_ in ("cage shell tubes", "cage shell nodes", LINKS):
            k = {"cage shell tubes": "shell", "cage shell nodes": "nodes", LINKS: "links"}[n_]
            pref = {"shell": "shell", "nodes": "node "}[k] if k != "links" else None
            sel = [(g_, c_) for nn, g_, c_ in it if (nn.startswith(pref) if pref else nn.split()[0] in ("spoke", "bay", "core", "cam", "saddle"))]
            p_["cg"] = np.round(sum(g_ * np.asarray(c_) for g_, c_ in sel) / sum(g_ for g_, _ in sel), 2).tolist()
        if pr_: p_["protect"] = True
        if mat_: p_["material"] = mat_
        if sw_: p_["swap"] = True
        asm_parts.append(p_)
    asm_parts.append(dict(name="wires", role="structure", mass_g=20.0, tag="E", owner="scout v3", version="layout v3", source="nox/scout/v3/layout_v3.py",
                          box=np.round(np.array(D["FC"][0].bounds).T.ravel(), 2).tolist(), note="envelope = the FC's (E)"))
    contact = [["motor %d" % i, "prop %d" % i] for i in range(1, 5)] + [["motor %d" % i, "frame plate (Bee35 carbon)"] for i in range(1, 5)]
    contact += [["cage shell tubes", "cage shell nodes"], [LINKS, "cage shell nodes"], [LINKS, "cage shell tubes"], [LINKS, "frame plate (Bee35 carbon)"],
                [LINKS, "receiver v3 (latch)"], [LINKS, "mount + nose legs (Scout)"], [LINKS, "receiver TPU end pieces (latch, E)"], [LINKS, CAMP],
                [CAMP, "camera"], [CAMP, "frame plate (Bee35 carbon)"], ["camera chin guard (printed)", CAMP], [CAMP, "mount + nose legs (Scout)"],
                ["receiver TPU end pieces (latch, E)", "receiver v3 (latch)"], ["receiver TPU end pieces (latch, E)", "receiver lid (latch)"],
                ["receiver TPU end pieces (latch, E)", "receiver TPU strips (latch)"], ["receiver TPU end pieces (latch, E)", "mount + nose legs (Scout)"],
                ["receiver TPU end pieces (latch, E)", "cartridge v6 strips (latch, TPU)"], ["cartridge v6 strips (latch, TPU)", "cartridge v6 (latch)"],
                ["cartridge v6 strips (latch, TPU)", "slider +y (latch)"], ["cartridge v6 strips (latch, TPU)", "slider -y (latch)"], ["cartridge v6 strips (latch, TPU)", "pack 4S 1500"],
                ["cartridge v6 (latch)", "receiver TPU strips (latch)", 5.0], ["pack 4S 1500", "cartridge v6 (latch)", 1.0], ["pack XT60 plug", "cartridge v6 (latch)", 5.0],
                ["XT60E1-M", "receiver v3 (latch)", 5.0], ["XT60E1-M", "receiver lid (latch)", 5.0], ["receiver TPU strips (latch)", "receiver v3 (latch)", 5.0],
                ["pack XT60 plug", "XT60E1-M", 1.0], ["mount + nose legs (Scout)", "receiver v3 (latch)"], ["mount + nose legs (Scout)", "receiver lid (latch)"],
                ["mount + nose legs (Scout)", "frame plate (Bee35 carbon)"], ["mount + nose legs (Scout)", "XT60E1-M"], ["mount + nose legs (Scout)", "FC"],
                ["mount + nose legs (Scout)", "UBEC"], ["FC", "frame plate (Bee35 carbon)"], ["Radxa Zero 3W", "mount + nose legs (Scout)"],
                ["flow MTF-02P", "mount + nose legs (Scout)"], ["UBEC", "frame plate (Bee35 carbon)"], ["XR2", "frame plate (Bee35 carbon)"],
                ["wires", "FC"], ["wires", "mount + nose legs (Scout)"], ["wires", "frame plate (Bee35 carbon)"], ["wires", "UBEC"], ["wires", "receiver v3 (latch)"]]
    if guard: contact += [[LINKS, "camera guard nodes"], ["camera guard nodes", CAMP]]
    contact += [[LINKS, "camera chin guard (printed)"]]          # the cradle struts root in the chin lip
    contact += [[LINKS, "tie saddle (printed)"], ["tie saddle (printed)", "frame plate (Bee35 carbon)"]]   # the ties root in the saddle; it bolts to the plate
    if CAM_MOUNT == "printed":
        contact += [["camera cradle arms (printed)", "cage shell nodes"], ["camera cradle arms (printed)", CAMP]]   # printed into the nodes and
        if CAM.get("clamps"):                                 # the snap clips sit round their rods (bore 3.1 on 3.0: no overlap); the arms end in them
            contact += [["camera cradle clips (printed)", "camera cradle arms (printed)"]]
                                                                                  # the cradle; clear of every rod (no contact, review 27 Oct)
    names_ = {p_[0] for p_ in PARTS} | {a_["name"] for a_ in asm_parts}
    contact += [[m_, o_] for m_ in names_ if "(Scout electronics, E)" in m_ for o_ in names_ if o_ != m_]   # mass markers, not geometry
    contact = [c_ if len(c_) == 3 else c_ + [1e9] for c_ in contact if c_[0] in names_ and c_[1] in names_]   # pairs of drawn parts only
    openings = [dict(part=p_, axis=[0, 0, -1], half_angle_deg=40.0, min_mm=0.0, skip_probes=True)
                for p_ in SW_IGN[:1] + ["pack 4S 1500", "slider +y (latch)", "slider -y (latch)", "pack XT60 plug", "receiver v3 (latch)", "receiver lid (latch)", "flow MTF-02P"]]
    openings.append(dict(part="camera", axis=[1.0, 0.0, 0.0], half_angle_deg=CAM_CONE, field=True,
                         note="Spark 26 Oct rule: the front camera outside the cage, nothing of the drone in its view; cage first gated outside this cone"))
    asm = new_assembly("Scout v3", "scout", asm_parts,
                       note=("Layout v3 / cage v2 (27 Oct, Spark: the latest Scout layout (PR 94), a triangulated shell cage, the bottom ring exactly the "
                             "Overseer's so the funnel is interchangeable, one camera, the carbon base kept; then every rod 3 mm, half the rods, any rod length) from nox/scout/v3/layout_v3.py. "
                             "Rough layout: C geometry on E sizes. Drone frame: x nose, y left, z up, plate underside 18.5, thrust centre and tab line x 0. "
                             "Left out: keep-alive contacts (the new funnel's), catch features, ESC (AIO)."),
                       cg_ref=[0.0, 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0, openings=openings, contact_pairs=contact,
                       guard_offsets={"shell tubes to prop disks, 3D (G)": round(pg, 1)}, mass_budget_g=500, cage_shell=SHELL_SPEC, access=ACCESS)
    asm.save(os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json"))
    say("wrote nox/scout/v3/scout.assembly.json")
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import scout_assembly_updates                             # the saved camera mount (PR 146) and the electronics session's round
    for s_ in scout_assembly_updates.main(): say(s_)          # (masses, Radxa post keep-outs) into the assembly; system_check_mount.json
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
