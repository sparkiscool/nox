"""Overseer core v3 (27 Oct): v2 plus the L2 head's 4 braces on the hub deck's front hub (a tower beside the keels; cage + arm
sessions moved the braces off the front arm beams; core concept: tower B over a top-plate block A, which put 765..788 N on each of
4 fasteners in the 1.6 top plate). Everything else as v2. Overseer core v2: system pass for everything inside the cage except the arms and the nodes (Spark 27 Oct: "design everything
else left on the newest overseer design inside the cage, excluding the arms ... ignore the actual mechanism of the battery
cartridge and pocket for now"; "also ignore all nodes on the body for now"; round 6, 27 Oct night: "the body needs to also include
the part that the nodes from the lidar goes to, this should be everything except the arms and the nodes"; "check if a run is
already using the camera mount" (it is: the cage session's head_mount v1, L2 + both cameras on one print); "we didnt need to
generate the part thats on the battery, the drone side just have an empty rectangle slot for now").

v2 (from v1 round 5, PR 123):
  FC stack   unchanged: MicoAir743-Lite on a 4-in-1 ESC straight on the top plate, turned 45° into the plate's slots, M3 studs.
  hub deck   PETG on the bottom plate, now one piece with the keel front hub: 6 feet, M3 x 8 up from under the plate into Ø4.0
             heat-set inserts in the feet ((-24, ±36.5), (65, ±22.5) Ø3.1 and (96, ±17) Ø3.5, every one a 360° hole in the STEP, every head outside the slot);
             the Pi 5 on 4 posts, the UBEC + buck bays on the front wing, and 2 keel sockets (6.2 x 15, Spark 27 Oct coupons) on the
             head's keel roots (96, ±12, 8) aimed at its lugs, so the L2 head's keels land on the core (layout v5 links)
  flow clip  PETG under the front wing: the MTF-02P pocket (lens down, ahead of the battery slot so its 42° view clears it),
             clamped under its 3.0 flange by the 2 front feet's screws (no screws of its own)
  battery    the drone side is an empty rectangular slot (the receiver v2 envelope in pose, the latch set's masses kept for
             balance and loads); no sled, nothing generated there until the latch is designed
Every screw goes straight in with a screwdriver in assembly order. Not this session's: arms, cage nodes and tubes (the keels'
far ends, the braces on the front arm beams), the sensor head and its cameras (head_mount v1 run), the latch mechanism.

Context (not edited here): layout v5 / cage v5 (`nox/overseer/overseer.assembly.json`).

python3 nox/overseer/core_v2.py [out_dir]     (~1 min)
Drone frame: x forward, y left, z up; z 0 = bottom plate top; plate centre x 0. Numbers: `nox/dimensions.md` O, M, L,
"System Overseer core".
"""
import os, sys, json, math, io, contextlib
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.join(ROOT, "nox", "box"))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_core", "v3")
PARTS = os.path.join(OUT, "parts"); os.makedirs(PARTS, exist_ok=True)
V5 = os.path.join(HERE, "overseer.assembly.json")
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    m = trimesh.creation.box([x1 - x0, y1 - y0, z1 - z0]); m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return m


def cyl(r, z0, z1, x, y, n=24):
    m = trimesh.creation.cylinder(r, z1 - z0, sections=n); m.apply_translation([x, y, (z0 + z1) / 2]); return m


def tube(path, r, n=10):
    segs = []
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        if np.linalg.norm(b - a) > 1e-6:
            segs.append(trimesh.creation.cylinder(r, np.linalg.norm(b - a), sections=n, segment=np.vstack([a, b])))
        segs.append(trimesh.creation.icosphere(1, r).apply_translation(b))
    return trimesh.util.concatenate(segs)


def cat(ms):
    return trimesh.util.concatenate(ms)


def uni(ms):
    """one closed body (manifold3d union) from overlapping blocks"""
    from topo.part import man
    import manifold3d as mf_
    u = mf_.Manifold.batch_boolean([man(m) for m in ms], mf_.OpType.Add); mm = u.to_mesh()
    return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts))


def bar(p, q, w, z0, z1):
    """a w wide bar from p to q in plan, z0..z1"""
    p, q = np.asarray(p, float), np.asarray(q, float); d = q - p; L_ = np.linalg.norm(d)
    m = box(-L_ / 2, L_ / 2, -w / 2, w / 2, z0, z1)
    T = np.eye(4); c, s = d / L_; T[:2, :2] = [[c, -s], [s, c]]; T[:2, 3] = (p + q) / 2
    m.apply_transform(T); return m


# ---------------------------------------------------------------- numbers (tags in dimensions.md "System Overseer core")
DC = 30.0                          # catch / latch centre 30 aft of the plate centre (v2, C)
Z_PLATE_BOT = -2.1                 # bottom plate underside (S500 STEP, derived)
Z_TOP_UNDER = 45.0                 # top plate underside (arm root 45, DEFAULT)
SLED_T = 6.0                       # the slot's top 6 under the plate is kept for the receiver's own mount (latch later; v1's sled, DEFAULT)
Z_M = Z_PLATE_BOT - SLED_T         # receiver mount plane (its deck top) -8.1: the latch set stays where v5 and the funnel have it
Y_M = 68.8                         # receiver v2 mount plane above the cartridge bottom (M, battery_latch r["Y_M"])
Z_CART = Z_M - Y_M                 # cartridge bottom -76.9
Z_T = 85.55                        # tab line (swap CG) in cartridge z (M); sits on the catch centre
BX = 27.8                          # receiver v2 bolt line x (M)
MOUNT_ZC = (-1.3, 85.55, 163.8)    # receiver v2 bolts along the cartridge (M)
# FC stack: MicoAir743-Lite 36 x 36 x 8, 10 g (spec) on a MicoAir 55A AM32 4-in-1 44 x 43.5 x 5.2, 14 g (listing; the stack's
# 17.5 / 24 g on the MicoAir drawing = FC 10 + this ESC 14, E: model CHECK), 30.5 x 30.5 Ø4 holes, M3 in grommets (usual, E).
# On the top plate (Spark 27 Oct: "the fc mounts to the top plates lines so just move that there"; "the fc does not need a mount it
# already goes to the carbon plate"): bolted straight through the plate's own 30.5 holes (Spark's plate, P; not in the Holybro STEP), centred.
STACK_P = 30.5 / 2
STACK_R = STACK_P * 2 ** 0.5                      # 21.57: the stack turned 45°, holes on the plate axes in its slots (C)
STACK_HOLES = [(STACK_R, 0.0), (-STACK_R, 0.0), (0.0, STACK_R), (0.0, -STACK_R)]
ESC_L, ESC_W, ESC_T = 44.0, 43.5, 5.2
FC_HALF, FC_T, STACK_H = 18.0, 8.0, 17.5
Z_TP = 46.6                        # top plate top (S500 STEP, derived; 1.6 plate on the 45 gap)
FLOOR_T = 4.0                      # 6 x 4 bars and Ø12 pads (the solid gate's 3.0 neck test)
POST_H, POST_D = 5.0, 11.0         # stack posts (wall 3.5 round the insert): M3 short heat-set insert (Ruthex M3 x 4.0, hole Ø4.0 x 5, Q17)
GROM = 1.0                         # grommet under the ESC (E)
STANDOFF = 3.0                     # the stack's own bottom grommet / nylon standoff on the plate (E)
Z_ESC = Z_TP + STANDOFF            # 49.6
Z_STACK_TOP = Z_ESC + STACK_H      # 70.1
CAP = (10.0, 20.0)                 # low-ESR cap on the ESC battery pads, 35 V 470 µF Ø10 x 20, lying on the top plate behind the ESC (E)
RX_TP = (-30.0, -18.0, 24.0, 40.0)  # RadioMaster XR2 16 x 12 x 6 (spec) on the top plate beside the stack, taped / zip-tied through the plate
FEET_XY = [(-24.0, 36.5), (-24.0, -36.5), (65.0, 22.5), (65.0, -22.5), (96.0, 17.0), (96.0, -17.0), (60.0, 17.0), (60.0, -17.0)]   # v3: + (60, ±17) Ø3.5   # bottom plate Ø3.1 / Ø3.5 (front) holes, 360° in the STEP
# v2 (concept review 27 Oct): every foot outside the battery slot (heads clear its reserved top 6: |y| 36.5 - 2.75 > 33.4, x 65 - 2.75 > 61.9);
# v3: except (60, ±17), whose heads sit in reserved pockets Ø7 x 3.5 in the slot top (asked of the latch session);
# each keel root closes a triangle into the plate: (96, ±17), (65, ±22.5) and the cross bar at x 96. M3 up from under the plate into inserts
INSERT = (4.0, 5.7)                # Ruthex M3 x 5.7 heat-set insert, hole Ø4.0 (Q17), pressed into each foot from the plate side
FOOT_H = 6.5                       # feet Ø12 x 6.5 (the deck's floor height), the insert hole Ø4.0 through (no thin cap over it)
CLIP_FLANGE = 3.0                  # the flow clip's flange under the 2 front screw heads (review: only 3 of PETG in the keel joint's clamp)
# (Spark 27 Oct: "do we really need that many": 4 carry 193 N a bolt in the 35 g crash vs 137 N with 6, E)
# (review 27 Oct: (-72.5, ±31.75) are R3 outline fillets, not holes; (-65, ±36.5) sit under the rear arm roots)
# Pi 5: board 85 x 56 x 1.6, M2.5 holes Ø2.7 on 58 x 49, 3.5 from the far edge and 23.5 from the USB-A / RJ45 edge (spec);
# Active Cooler 63.5 x 42.5 footprint, 13.7 tall over the board, blower intake on top (Raspberry Pi product brief, Jan 2026);
# USB-A / RJ45 16 over the board (spec); parts under the board 1.5 (microSD, E); 46 + 9 g ~ 55 g (E)
PI_CX = float(os.environ.get("CORE_PI_CX", -11.0))   # Pi centre between the plates (scanned: 0 mm³ with the arm roots, plugs aft through the gap)
PI_X0, PI_X1, PI_W = PI_CX - 42.5, PI_CX + 42.5, 56.0
PI_HOLES = [(PI_X0 + 23.5 + dx, sy * 24.5) for dx in (0.0, 58.0) for sy in (1, -1)]
PI_POST = 4.0                      # posts on the floor bars, M2.5 self-tapping Ø2.2 x 4 (DEFAULT)
# the Pi boots from a USB stick in an aft port (DEFAULT): its microSD end faces the UBEC bay 2.5 away, not reachable
Z_PI = FLOOR_T + PI_POST           # board underside 8.0
PI_BOARD_T, PI_UP, PI_COOL = 1.6, 16.0, 13.7
COOL_X = (PI_X1 - 63.5 + 2.0, PI_X1 + 2.0)     # cooler over the SoC end of the board (E place)
RAIL_Y = 22.0                      # floor rails at |y| 22: clear of the arm roots' inner corners (C)
BAY_Y = 17.0                       # front wing bay rails at |y| 17: bars inside |y| 20, 1 off the front arm envelopes (arm session 27 Oct)
# front wing bays (zip ties, no screws): UBEC and buck between the front arm roots, ahead of the Pi, under the top plate's front
DECK_TOP = 6.5                     # the hub deck's floor top (its topology envelope, the gate's 6.25 member; hub deck v1 run)
UBEC_BAY = (34.0, 82.0, -19.0, 2.0, DECK_TOP, DECK_TOP + 10.0)   # any 5 V 5 A UBEC of 43-50 x 17-25 x 7-10 (Q4 range), on the deck
BUCK_BAY = (34.0, 58.0, 4.0, 22.0, DECK_TOP, DECK_TOP + 10.0)    # 12 V buck (model open: 24 x 18 x 10 bay, E), on the deck
# keels of the L2 head (layout v5 links, runs/overseer_layout/v5/head_system.json carbon_kept): core root -> head lug, 6/4 tube
KEELS = {sy: (np.array([96.0, 12.0 * sy, 8.0]), np.array([379.7, 46.0 * sy, 54.23])) for sy in (1, -1)}
KEEL_N = (274.2, 387.3)            # max tension / compression per keel (frame model runs/overseer_frame/v1 members.json, F)
SOCK_D, SOCK_L, SOCK_OFF, SOCK_OD = 6.2, 15.0, 5.0, 13.0   # bore (Spark 27 Oct coupons), depth, the tube end 5 past the root, boss (wall 3.4)
# L2 braces (v3): tube ends on the front hub tower, each aimed at its head lug (cage session's tower roots, 27 Oct; their frame model:
# 712 N a side summed along the braces in the top hit, F on E, no SF; head 166 Hz), split lo / hi by the v1 frame maxima 551 : 353 (E)
BRACES = {(lvl, sy): (np.array([97.0, yy * sy, zz]), np.array([lx, 46.0 * sy, lz])) for lvl, yy, zz, lx, lz in (("lo", 26.0, 22.0, 379.7, 54.23), ("hi", 14.0, 32.0, 383.32, 67.75)) for sy in (1, -1)}
BRACE_N = {"lo": 712.0 * 551 / 904, "hi": 712.0 * 353 / 904}
BR_BORE, BR_L, BR_LEAD, BR_WALL = 6.2, 15.0, 0.6, 2.5      # cage ask: 6.2 x 15 (+0.6 lead-in), 2.5 walls; the tube end at the root
# flow clip under the front wing (MTF-02P 21.6 x 16 x 6.5, 1.5 g, spec; no holes: pocket), long side along x between the front feet
FLOW_X, FLOW_Y = 94.0, 0.0
FLOW_T = 6.5
LENS_Z = Z_PLATE_BOT - FLOW_T      # lens on the sensor's underside, the clip's window below it
CLIP_FLOOR = 3.0                   # floor and walls 3.0: the solid gate fails necks under 3.0 (2.0 failed on the clip, 27 Oct)
ARM_SCREWS = [(sx * a, sy * b) for sx in (1, -1) for sy in (1, -1) for a, b in ((52.14, 38.0), (38.0, 52.14))]   # bottom plate (derived)

# ---------------------------------------------------------------- context: v5 manifest
v5 = json.load(open(V5))
REPLACED = {"hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", "wires", "pack", "cartridge", "receiver", "belly sled + pods",
            "flow MTF-02P", "TPU feet"}
KEELS_NM = "spokes, ties, keels, braces, head struts"
context = [dict(p) for p in v5["parts"] if p["name"] not in REPLACED]
ctx_mesh = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in context}
# Spark 27 Oct: the keels' front hub is the body's (v5 drew it as a block x 86..99 inside the head mount part): the hub deck takes it
HEAD_NM = next(p["name"] for p in context if p["name"].startswith("sensor head mount"))
_hb = [b for b in ctx_mesh[HEAD_NM].split(only_watertight=False) if b.bounds[0][0] > 120.0]
ctx_mesh[HEAD_NM] = cat(_hb)
for p in context:
    if p["name"] == HEAD_NM:
        p["mesh"] = os.path.relpath(os.path.join(PARTS, "sensor_head_mount_without_front_hub.stl"), ROOT); ctx_mesh[HEAD_NM].export(os.path.join(ROOT, p["mesh"]))
        p["note"] = "v5 head mount without its front hub block (x 86..99 on the bottom plate): the keel sockets are the hub deck's now (Spark 27 Oct)"
ARMS = [ctx_mesh[f"arm {i}"] for i in (1, 2, 3, 4)]

# ---------------------------------------------------------------- latch set (battery latch v4, Overseer) in pose
with contextlib.redirect_stdout(io.StringIO()):
    import battery_latch as BL
    LA = BL.latch_assembly("overseer", "latched")
T_LATCH = np.array([[0, 0, -1, -DC + Z_T], [-1, 0, 0, 0], [0, 1, 0, Z_M - Y_M], [0, 0, 0, 1.0]])   # zc -> -x (XT60 end aft)


def m2t(m):
    mm = m.to_mesh(); t = trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False)
    t.apply_transform(T_LATCH); return t


receiver = cat([m2t(LA[k]) for k in ("receiver", "receiver_lid", "receiver_strips", "xt60_carrier")])
cartridge = cat([m2t(LA[k]) for k in ("cartridge", "slider_px", "slider_nx")])
pack = m2t(LA["pack"]); xt60 = cat([m2t(LA["xt60_plug"]), m2t(LA["xt60e1m"])])
RB = receiver.bounds
say(f"receiver in pose x {RB[0][0]:.1f}..{RB[1][0]:.1f}, y ±{RB[1][1]:.1f}, z {RB[0][2]:.1f}..{RB[1][2]:.1f} (M); "
    f"pack top z {pack.bounds[1][2]:.1f}, cartridge bottom z {cartridge.bounds[0][2]:.1f}; XT60 male top z {xt60.bounds[1][2]:.1f}")
rec_bolts = [(-DC + Z_T - zc, sy * BX) for zc in MOUNT_ZC for sy in (1, -1)]

# ---------------------------------------------------------------- bought parts (their real envelopes)
esc = box(-ESC_L / 2, ESC_L / 2, -ESC_W / 2, ESC_W / 2, Z_ESC, Z_ESC + ESC_T)
fc = box(-FC_HALF, FC_HALF, -FC_HALF, FC_HALF, Z_STACK_TOP - FC_T, Z_STACK_TOP)
stack = cat([esc, fc] + [cyl(2.5, Z_ESC + ESC_T, Z_STACK_TOP - FC_T, sx * STACK_P, sy * STACK_P, 12) for sx in (1, -1) for sy in (1, -1)])
stack.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 4, [0, 0, 1]))       # turned 45°: holes on the plate's slots (PX4 SENS_BOARD_ROT yaw 45)
cap = box(-62.0, -62.0 + CAP[1], -CAP[0] / 2, CAP[0] / 2, Z_TP, Z_TP + CAP[0])   # on the battery leads where they come over the plate's rear edge
rx = box(RX_TP[0], RX_TP[1], RX_TP[2], RX_TP[3], Z_TP, Z_TP + 6)
pi = cat([box(PI_X0, PI_X1, -PI_W / 2, PI_W / 2, Z_PI, Z_PI + PI_BOARD_T),
          box(PI_X0 - 2, PI_X0 + 21, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_UP),                  # USB-A x4 + RJ45, aft
          box(COOL_X[0], COOL_X[1], -21.25, 21.25, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_COOL)])                            # Active Cooler
pi_plugs = box(PI_X0 - 2 - 18.0, PI_X0 - 2, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_UP)         # plug bodies (keep-out, E)
fan_air = box(COOL_X[0] + 10, COOL_X[1] - 25, -15, 15, Z_PI + PI_BOARD_T + PI_COOL, Z_PI + PI_BOARD_T + PI_COOL + 8)           # intake room (keep-out, E)
ubec = box(UBEC_BAY[0], UBEC_BAY[0] + 48, -18, 2, UBEC_BAY[4], UBEC_BAY[4] + 9)              # a 48 x 20 x 9 UBEC (Q4 middle, E)
buck = box(BUCK_BAY[0] + 1, BUCK_BAY[0] + 21, 5, 21, BUCK_BAY[4], BUCK_BAY[4] + 8)           # 20 x 16 x 8 (E)
flow = box(FLOW_X - 10.8, FLOW_X + 10.8, FLOW_Y - 8, FLOW_Y + 8, LENS_Z, LENS_Z + FLOW_T)     # MTF-02P (spec), long side along x
slot = box(RB[0][0], RB[1][0], RB[0][1], RB[1][1], Z_CART, Z_PLATE_BOT)                         # the empty battery slot (receiver v2 envelope)


# ---------------------------------------------------------------- the core pieces (concept bodies for the piece runs; the coupons are their mounting features)
def keel_frame(sy):
    a, b = KEELS[sy]; d = (b - a) / np.linalg.norm(b - a); return a, d


def keel_socket(sy, r, l0, l1):
    a, d = keel_frame(sy)
    return trimesh.creation.cylinder(r, l1 - l0, sections=32, segment=np.vstack([a + d * l0, a + d * l1]))


def hub_deck():
    """on the bottom plate: 6 feet (inserts, M3 up from under the plate), the Pi's 4 posts, the UBEC + buck bays, the keel front hub"""
    feet = [cyl(7, 0, FOOT_H, x, y) for x, y in FEET_XY]                                                          # Ø14 (hub deck v4: the gate's 2.0 round Ø4.0)
    px0, px1 = PI_HOLES[0][0], PI_HOLES[2][0]
    pip = [cyl(3.5, 0, Z_PI, x, y) for x, y in PI_HOLES]
    bars = [bar((-24.0, sy * 36.5), (px0, sy * 24.5), 6, 0, FLOOR_T) for sy in (1, -1)]                         # rear feet to the rear posts
    bars += [bar((px0, 24.5), (px0, -24.5), 6, 0, FLOOR_T), bar((px1, 24.5), (px1, -24.5), 6, 0, FLOOR_T)]
    bars += [bar((px1, sy * RAIL_Y), (65.0, sy * 22.5), 6, 0, FLOOR_T) for sy in (1, -1)]                       # front posts to the mid feet
    bars += [bar((65.0, sy * 22.5), (96.0, sy * 17.0), 6, 0, FLOOR_T) for sy in (1, -1)]                        # the keel triangles' sides
    bars += [bar((px1, sy * BAY_Y), (96, sy * BAY_Y), 6, 0, FLOOR_T) for sy in (1, -1)]                          # bay rails to the front feet
    bars += [bar((px0, sy * RAIL_Y), (px1, sy * RAIL_Y), 6, 0, FLOOR_T) for sy in (1, -1)]                       # under the Pi (board z 8)
    bars += [bar((96, BAY_Y), (96, -BAY_Y), 6, 0, FLOOR_T), bar((57, BAY_Y), (57, -BAY_Y), 6, 0, FLOOR_T)]
    hub = [keel_socket(sy, SOCK_OD / 2, -2.0, SOCK_OFF + SOCK_L) for sy in (1, -1)]                               # keel bosses
    hub += [box(88.0, 100.0, sy * 6.0, sy * 18.0, 0, 8.0) for sy in (1, -1)]                                       # their seats on the front feet
    return uni(feet + pip + bars + hub)


def flow_clip():
    """under the front wing: the MTF-02P pocket (window below the lens) between the 2 front feet, clamped by their screws"""
    pads = [cyl(6, Z_PLATE_BOT - CLIP_FLANGE, Z_PLATE_BOT, 96.0, sy * 17.0) for sy in (1, -1)]               # 3 flange under the heads
    cup = box(FLOW_X - 13.8, FLOW_X + 13.8, -11.2, 11.2, LENS_Z - CLIP_FLOOR, Z_PLATE_BOT)
    arms_ = [box(90.0, 102.0, sy * 9.0, sy * 17.0, Z_PLATE_BOT - CLIP_FLANGE, Z_PLATE_BOT) for sy in (1, -1)]
    return uni(pads + [cup] + arms_)


def brace_socket(key, r, l0, l1):
    a, b = BRACES[key]; d = (b - a) / np.linalg.norm(b - a)
    return trimesh.creation.cylinder(r, l1 - l0, sections=32, segment=np.vstack([a + d * l0, a + d * l1]))


# second chord (concept review 27 Oct, rounds 1-2): the braces tip the tower forward (top hit) or back (front hit), ~20 / 14 N m a side
# at SF 1.1. Round 3: deep side webs from the tower back to the mid feet beside the bays (|y| 20..34, x 58..92, z 0..36; the UBEC /
# buck columns stay open for placing them; the arm-free wing is |y| <= x - 24), and 2 more feet (60, ±17): the tower, web and the 4
# mid / front feet close a triangle both ways (round 2's pads under the top plate bore only one way and bent the 1.6 plate)
TOE_X, MID_X, FRONT_X = 104.0, 62.5, 96.0
def bar3(p, q, w):
    p, q = np.asarray(p, float), np.asarray(q, float)
    return trimesh.creation.box([np.linalg.norm(q - p), w, w], transform=trimesh.geometry.align_vectors([1, 0, 0], q - p) @ np.eye(4)
                                ).apply_translation((p + q) / 2)


def tower():
    """the front hub tower: the 4 brace bosses, a web down to the keel bosses and the front feet, and the deep side webs back to the mid feet"""
    bosses = [brace_socket(k, BR_BORE / 2 + BR_WALL, -BR_WALL, BR_L + BR_LEAD) for k in BRACES]
    webs = [box(86.0, 104.0, sy * 9.0, sy * 31.0, 0, 36.0) for sy in (1, -1)]
    sides = [box(60.0, 92.0, sy * 20.0, sy * 28.0, 0, 36.0) for sy in (1, -1)]   # concept: the deep side webs to the mid feet (the run shapes them)
    return uni(bosses + webs + sides)


DECK, CLIP = hub_deck(), flow_clip()
DECK = uni([DECK, tower()])
HUB_RUN = os.path.join(ROOT, "runs", "overseer_hub_deck", "v6", "overseer_hub_deck.stl")   # v6: keels + braces (v4/v5 keels only)   # the saved piece (topology run v4), drone frame
if os.path.exists(HUB_RUN) and not os.environ.get("CORE_CONCEPT"):
    DECK = trimesh.load(HUB_RUN); say(f"hub deck: the v4 run's mesh ({DECK.volume * 1.27e-3:.1f} g PETG, M) in the assembly (CORE_CONCEPT=1: the concept body)")

# ---------------------------------------------------------------- wiring: drawn routes, each ending in a plug (clean wiring, L 8)
W = {
    "battery 12 AWG pair: receiver XT60 -> up the rear wing past the top plate's rear edge -> ESC battery pads (rear edge, on top), cap there": (
        [(-108.3, 0, xt60.bounds[1][2]), (-100, 0, 4), (-68, 0, 30), (-64, 0, Z_TP + 3), (-24, 0, Z_ESC + 2)], 4.0),
    "UBEC + buck in: ESC battery pads -> over the top plate's front edge -> front bays": ([(-20, -8, Z_ESC + 2), (40, -8, Z_TP + 2), (64, -8, 30), (60, -8, 10)], 1.5),
    "UBEC 5 V -> Pi GPIO 5 V pins (+y edge)": ([(40, 0, 9), (32, 20, 12), (PI_CX, 24, Z_PI + 4)], 1.5),
    "buck 12 V -> nose (the L2 owner takes it there)": ([(56, 12, 8), (90, 0, 6)], 1.5),
    "FC <-> Pi (USB), out past the top plate's -y edge": ([(0, -18, Z_STACK_TOP - 3), (0, -58, Z_TP + 2), (0, -62, 30), (PI_X0 - 8, -15, Z_PI + 8)], 1.5),
    "FC <-> RX": ([(-18, 10, Z_STACK_TOP - 3), (-32, 26, Z_TP + 4)], 1.5),
    "FC UART -> flow (over the top plate's front edge, down the front wing beside the UBEC, round the plate's nose to the clip)": (
        [(10, -10, Z_STACK_TOP - 4), (50, -4, Z_TP + 2), (64, -2, 30), (84, -2, 9), (102, -2, 3), (103, -2, Z_PLATE_BOT - 1), (FLOW_X + 9, 0, LENS_Z + 3)], 1.5),
}
# motor leads: from each ESC corner straight out along the top of its arm (on top of the plate now: no root to pass)
for k, ang in enumerate((45, 135, 225, 315)):   # the stack is turned 45°: its corners (motor pads, E) sit on the plate axes; each lead
    d = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])   # leaves the nearer corner and turns onto its arm's top
    c0 = np.array([math.cos(math.radians(ang - 45)), math.sin(math.radians(ang - 45))]) * 30.0
    W[f"motor lead {k + 1}: ESC corner (on the plate axis) -> along the top of arm {k + 1}"] = ([(*c0, Z_ESC + 3), (*(45 * d), Z_TP + 3), (*(110 * d), Z_TP + 4)], 2.0)
wires = cat([tube(p, r) for p, r in W.values()])

# ---------------------------------------------------------------- manifest
def P(name, m, role, mass=None, material="PETG", protect=False, tag="E", swap=False, owner="overseer core", budget=None, version="core v2"):
    fn = name.lower().replace(" ", "_").replace("/", "_").replace("+", "").replace("(", "").replace(")", "").replace(",", "")[:40] + ".stl"
    m.export(os.path.join(PARTS, fn))
    d = {"name": name, "role": role, "mesh": os.path.relpath(os.path.join(PARTS, fn), ROOT), "material": material, "tag": tag,
         "owner": owner, "version": version, "source": "nox/overseer/core_v1.py"}
    if mass is not None: d["mass_g"] = round(float(mass), 2)
    if protect: d["protect"] = True
    if swap: d["swap"] = True
    if budget: d["budget_g"] = budget
    return d


BUDGET = {"hub deck": 55, "flow clip": 5}   # g, soft slots (DEFAULT; v1 deck 24 + the keel hub 8)
MASS = {"hub deck": round(float(DECK.volume) * 1.27e-3, 1), "flow clip": 5.0}   # M once saved
# floor contact: TPU feet on 4 opening ring nodes (cage session; placeholders at v5's ring nodes, contact 6 under the lenses)
RING = [(146.0, 43.2), (146.0, -43.2), (-102.9, 104.3), (-102.9, -104.3)]
Z_FOOT = Z_CART - 6.0                # contact 6 under the cartridge bottom (v1)
feet = cat([cyl(7, Z_FOOT, Z_FOOT + 8, x, y) for x, y in RING])
STACK_NM, PI_NM = "FC stack (MicoAir743-Lite on 55A 4-in-1)", "Pi 5 + Active Cooler"
core = [
    P(STACK_NM, stack, "bought", 24, protect=True),
    P("ESC cap", cap, "bought", 5, protect=True),
    P(PI_NM, cat([pi, pi_plugs, fan_air]), "bought", 55, protect=True),
    P("UBEC 5 V", ubec, "bought", 15, protect=True),
    P("buck 12 V", buck, "bought", 6, protect=True),
    P("RX XR2", rx, "bought", 2, protect=True),
    P("flow MTF-02P", flow, "bought", 1.5, protect=True),
    P("wires", wires, "bought", 80),
    P("pack", pack, "bought", 650, protect=True, swap=True),
    P("cartridge", cartridge, "protected", 105.6, protect=True, swap=True, tag="M", owner="box swap (battery latch, later: the slot's contents)", version="cartridge v6 + sliders v6"),
    P("receiver", receiver, "protected", 229.3, protect=True, tag="M", owner="box swap (battery latch, later: the slot's contents)", version="receiver v2 + carrier v1 + cap + strips"),
    P("XT60 pair", xt60, "bought", 15, protect=True),
    P("hub deck", DECK, "structure", MASS["hub deck"], budget=BUDGET["hub deck"], tag="M" if os.path.exists(HUB_RUN) else "E"),
    P("flow clip", CLIP, "structure", BUDGET["flow clip"], budget=BUDGET["flow clip"]),
    P("TPU feet (cage session, placeholders)", feet, "cage", 6.4, material="TPU", owner="overseer cage", version="placeholder"),
]
CORE_NAMES = {p["name"] for p in core}
contact = [list(c) for c in v5["contact_pairs"] if not (set(c) & REPLACED)]
contact += [[STACK_NM, "top plate"], ["ESC cap", "top plate"], ["RX XR2", "top plate"],
            ["hub deck", "bottom plate"], ["hub deck", PI_NM], ["hub deck", "UBEC 5 V"], ["hub deck", "buck 12 V"],
            ["ESC cap", STACK_NM], ["flow clip", "bottom plate"], ["flow clip", "flow MTF-02P"], ["flow MTF-02P", "bottom plate"], ["hub deck", KEELS_NM],
            ["receiver", "bottom plate"],
            ["pack", "cartridge"], ["pack", "receiver"], ["cartridge", "receiver"], ["receiver", "XT60 pair"], ["cartridge", "XT60 pair"], ["pack", "XT60 pair"]]
contact += [["wires", n] for n in sorted(CORE_NAMES | {"bottom plate", "top plate", "arm 1", "arm 2", "arm 3", "arm 4", KEELS_NM}) if n != "wires"]
openings = [o for o in v5["openings"] if o.get("part") not in ("pack", "cartridge", "receiver", "flow MTF-02P")]
openings += [{"part": p_, "axis": [0, 0, -1], "half_angle_deg": 40.0, "skip_probes": True} for p_ in ("pack", "cartridge", "receiver", "XT60 pair", "flow MTF-02P")]

# access (CLAUDE.md 2; Spark 27 Oct: "make sure bolts are accessible through a screwdriver"): every core screw is driven straight
# down with a screwdriver (shaft Ø6, 150 long) at its assembly step, so each path ignores what goes on later (top plate, Pi, stack)
DRV = {"type": "cylinder", "d": 6}
LATER = ["top plate", "prop 1", "prop 2", "prop 3", "prop 4", "cage shell tubes", "cage shell nodes", "spokes, ties, keels, L2 guard ring", "wires",
         "RX XR2", "ESC cap", "FC stack (MicoAir743-Lite on 55A 4-in-1)"]   # all of these go on after the hub deck and the Pi
acc = [a for a in v5["access"] if a["name"].startswith("M3")]
for a in acc:
    a["ignore"] = [i for i in a["ignore"] if i in ("arm 1", "arm 2", "arm 3", "arm 4", "prop 1", "prop 2", "prop 3", "prop 4")]
acc += [{"name": "pack swap: cartridge + pack down through the opening", "part": "cartridge", "dir": [0, 0, -1], "length_mm": 150.0, "tool": "part", "ignore": ["pack", "receiver", "XT60 pair"]},
        {"name": "FC USB-C (an edge of the turned FC: plug straight in 30 along the arm, then the cable bends up)", "part": STACK_NM,
         "from": [12.7, -12.7, Z_STACK_TOP - 4], "dir": [0.7071, -0.7071, 0.0], "length_mm": 30.0, "tool": {"type": "box", "w": 12, "h": 7}, "ignore": ["wires"]},
        {"name": "Pi USB-A / RJ45 plugs (aft, in the open)", "part": PI_NM, "from": [PI_X0 - 20, 0, Z_PI + 9], "dir": [-1, 0, 0], "length_mm": 40.0,
         "tool": {"type": "box", "w": 40, "h": 14, "w_axis": [0, 1, 0]}, "ignore": ["wires"]}]
SLOT_LATER = ["pack", "cartridge", "receiver", "XT60 pair", "cage shell tubes", "cage shell nodes", KEELS_NM]   # the latch set and the cage go on after the core
for k, (x, y) in enumerate(FEET_XY):
    front = x > 90
    acc.append({"name": f"step {2 if front else 1}: M3 x {10 if front else 8} up from under the plate{' through the flow clip flange' if front else ''} into the hub deck insert, foot {k} "
                f"(screwdriver from below, the battery slot empty)", "part": "flow clip" if front else "bottom plate",
                "from": [x, y, (Z_PLATE_BOT - CLIP_FLANGE if front else Z_PLATE_BOT) - 0.5], "dir": [0, 0, -1], "length_mm": 150.0, "tool": DRV,
                "ignore": ["pack", "cartridge", "receiver", "XT60 pair", "cage shell tubes", "cage shell nodes", "TPU feet (cage session, placeholders)"]})
for k, (x, y) in enumerate(PI_HOLES):
    acc.append({"name": f"step 2: M2.5 Pi screw {k} (screwdriver, top plate not on yet)", "part": PI_NM, "from": [x, y, Z_PI + PI_BOARD_T + 0.5], "dir": [0, 0, 1],
                "length_mm": 150.0, "tool": {"type": "cylinder", "d": 5}, "ignore": LATER})
for k, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
    acc.append({"name": f"step 6: M3 nylon nut {k} on its stud over the FC (nut driver from above, before the cage shell; v5's top tubes cross the x-axis nut lines)", "part": STACK_NM,
                "from": [STACK_HOLES[k][0], STACK_HOLES[k][1], Z_STACK_TOP + 0.5], "dir": [0, 0, 1], "length_mm": 150.0, "tool": DRV,
                "ignore": ["prop 1", "prop 2", "prop 3", "prop 4", "wires", "cage shell tubes", "cage shell nodes"]})

man = {k: v for k, v in v5.items() if k not in ("parts", "contact_pairs", "openings", "access", "note", "name")}
man.update({"name": "Overseer core v2", "machine": "overseer",
            "note": ("Overseer core v2 (27 Oct, Spark: everything inside the cage except the arms and the nodes, the keel roots of the L2 head included; "
                     "the battery side an empty rectangular slot; latch mechanism, cameras and the head not this session's; screwdriver access, fewest pieces): "
                     "v5 / cage v5 context parts unchanged (nox/overseer/overseer.assembly.json); the core, the latch set in pose (masses only: the slot's "
                     "contents, latch later) and the feet placeholders from nox/overseer/core_v2.py."),
            "parts": context + core, "contact_pairs": contact, "openings": openings, "access": acc})
MAN = os.path.join(HERE, "overseer_core.assembly.json")
with open(MAN, "w") as f:
    f.write("{\n" + ",\n".join(f' {json.dumps(k)}: ' + (("[\n" + ",\n".join("  " + json.dumps(p) for p in v) + "\n ]") if k == "parts" else json.dumps(v))
                               for k, v in man.items()) + "\n}\n")

# ---------------------------------------------------------------- checks of this pass (C on E sizes)
from topo.part import man as MAN_OF


def ov(a, b):
    return (MAN_OF(a) ^ MAN_OF(b)).volume()


top_plate = ctx_mesh["top plate"]
props = cat([ctx_mesh[f"prop {i}"] for i in (1, 2, 3, 4)])
say(f"stack straight on the top plate: ESC z {Z_ESC:.1f}..{Z_ESC + ESC_T:.1f} on its own {STANDOFF:.0f} standoffs / grommets, FC top z {Z_STACK_TOP:.1f}; leads to ~{Z_STACK_TOP + 3:.1f}; "
    f"x props {ov(cat([stack, cap, rx]), props):.1f} mm³; nearest blade band z {82 - 5:.0f} (C on E)")
cool_top = Z_PI + PI_BOARD_T + PI_COOL
say(f"Pi right side up between the plates: board x {PI_X0:.1f}..{PI_X1:.1f}, z {Z_PI}..{Z_PI + PI_BOARD_T}; Active Cooler top z {cool_top:.1f} "
    f"({Z_TOP_UNDER - cool_top:.1f} under the top plate for the blower intake); ports aft, top z {Z_PI + PI_BOARD_T + PI_UP:.1f} (C on E)")
fits = [f"{nm} {sum(ov(m, arm) for arm in ARMS):.1f}" for nm, m in (("hub deck", DECK), ("Pi + plugs", cat([pi, pi_plugs])), ("UBEC", ubec), ("buck", buck),
                                                                    ("stack", stack), ("cap", cap), ("RX", rx))]
say("   x arms (v5 meshes), mm³: " + ", ".join(fits))
say(f"   deck x Pi {ov(DECK, pi):.1f} (posts on its holes), deck x UBEC {ov(DECK, ubec):.1f}, deck x buck {ov(DECK, buck):.1f}, Pi + intake x top plate {ov(cat([pi, fan_air]), top_plate):.1f}, "
    f"Pi x UBEC {ov(cat([pi, pi_plugs]), ubec):.1f}, cap x stack {ov(cap, stack):.1f}, RX x stack {ov(rx, stack):.1f} mm³")
say(f"   battery slot (empty, x {RB[0][0]:.1f}..{RB[1][0]:.1f}, y ±{RB[1][1]:.1f}, z {Z_CART:.1f}..{Z_PLATE_BOT}): x deck {ov(slot, DECK):.1f}, x flow clip {ov(slot, CLIP):.1f}, "
    f"x flow {ov(slot, flow):.1f}, x wires {ov(slot, wires):.1f} mm³")
heads = cat([cyl(2.75, Z_PLATE_BOT - (CLIP_FLANGE if x > 90 else 0) - 3.0, Z_PLATE_BOT - (CLIP_FLANGE if x > 90 else 0), x, y) for x, y in FEET_XY])
say(f"deck screw heads (M3 Ø5.5 x 3) x the battery slot {ov(heads, slot):.1f} mm³: the 2 heads at (60, ±17) sit in its top 6 (reserved pockets Ø7 x 3.5 "
    f"for the receiver mount to clear, v3 DEFAULT); every other head 0")
keel_tubes = [trimesh.creation.cylinder(3.4, np.linalg.norm(b - a), sections=24, segment=np.vstack([a, b])) for a, b in KEELS.values()]   # v5 draws them from the root
bores = cat([keel_socket(sy, SOCK_D / 2, SOCK_OFF, SOCK_OFF + SOCK_L + 1) for sy in (1, -1)])
say(f"keel sockets: bore Ø{SOCK_D} x {SOCK_L} from {SOCK_OFF} past each root, boss Ø{SOCK_OD}; deck x keel tubes {ov(DECK, cat(keel_tubes)):.1f} mm³ "
    f"(inside the bores and the boss root, cut in the run); deck x arms {sum(ov(DECK, arm) for arm in ARMS):.1f}; deck x the other v5 links and nodes "
    f"{(MAN_OF(DECK) ^ (MAN_OF(ctx_mesh[KEELS_NM]) - MAN_OF(cat(keel_tubes)))).volume():.1f}, x cage nodes {ov(DECK, ctx_mesh['cage shell nodes']):.1f} mm³")
say(f"flow clip x bottom plate {ov(CLIP, ctx_mesh['bottom plate']):.1f}, x the v5 links and nodes {ov(CLIP, ctx_mesh[KEELS_NM]):.1f} / {ov(CLIP, ctx_mesh['cage shell nodes']):.1f}, "
    f"flow x clip {ov(flow, CLIP):.1f} (pocket cut in the piece), deck x clip {ov(DECK, CLIP):.1f} mm³")
blk = []
for x, y in ARM_SCREWS:
    key = cyl(4.0, Z_FOOT - 30, Z_PLATE_BOT - 0.5, x, y, 16)
    for nm, m in (("flow clip", CLIP), ("flow", flow)):
        v = ov(key, m)
        if v > 0.5: blk.append(f"({x:.0f},{y:.0f}) {nm} {v:.0f}")
say("arm bottom screw keys (Ø8 below the plate): " + (", ".join(blk) if blk else "all 8 clear of the core (C); the slot's contents come later"))


def view_clear(lens, half_x, half_y, meshes):
    hits = []
    for nm, m in meshes:
        p = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 4000, seed=1)[0]])
        d = p - lens; below = d[:, 2] < -0.5
        ax = np.degrees(np.arctan2(np.abs(d[:, 0]), -d[:, 2])); ay = np.degrees(np.arctan2(np.abs(d[:, 1]), -d[:, 2]))
        n = int((below & (ax < half_x) & (ay < half_y)).sum())
        if n: hits.append(f"{nm} {n}")
    return hits


h2 = view_clear(np.array([FLOW_X, FLOW_Y, LENS_Z - CLIP_FLOOR]), 23, 23, [("battery slot", slot), ("cartridge", cartridge), ("feet", feet),
               ("cage tubes", ctx_mesh["cage shell tubes"]), ("v5 links", ctx_mesh[KEELS_NM]), ("cage nodes", ctx_mesh["cage shell nodes"])])
say("flow view (42°, +2°, from the clip's window): " + ("clear (C)" if not h2 else "blocked: " + ", ".join(h2)))
col = box(-DC - 179.6 / 2 - 30, -DC + 179.6 / 2 + 30, -76.2 / 2 - 30, 76.2 / 2 + 30, Z_FOOT - 50, Z_CART)
say(f"swap column (receiver + 30 a side, below the cartridge bottom): flow clip {ov(col, CLIP):.1f}, flow {ov(col, flow):.1f}, feet {ov(col, feet):.1f} mm³")
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union
disks = unary_union([Point(ctx_mesh[f"motor {i}"].centroid[0], ctx_mesh[f"motor {i}"].centroid[1]).buffer(127.0, 64) for i in (1, 2, 3, 4)])
plan = {nm: MultiPoint(m.vertices[:, :2]).convex_hull for nm, m in (("hub deck + Pi + bays", cat([DECK, pi, ubec, buck])), ("flow clip", CLIP), ("battery slot", slot))}
wash = {nm: round(float(g.intersection(disks).area / disks.area * 100), 2) for nm, g in plan.items()}
say("prop wash (plan area under the disks, % of the disk area): " + ", ".join(f"{k} {v}" for k, v in wash.items()))
br_tubes = cat([trimesh.creation.cylinder(3.4, np.linalg.norm(b - a), sections=24, segment=np.vstack([a, b])) for a, b in BRACES.values()])
TW = tower()
say(f"tower: 4 brace sockets Ø{BR_BORE} x {BR_L} (+{BR_LEAD}), walls {BR_WALL}, ends lo (97, ±26, 22), hi (97, ±14, 32); x arms {sum(ov(TW, a_) for a_ in ARMS):.1f}, "
    f"x top plate {ov(TW, top_plate):.1f}, x bottom plate {ov(TW, ctx_mesh['bottom plate']):.1f}, x UBEC {ov(TW, ubec):.1f}, x buck {ov(TW, buck):.1f}, x wires {ov(TW, wires):.1f}, "
    f"x props {ov(TW, props):.1f}, x cage nodes {ov(TW, ctx_mesh['cage shell nodes']):.1f}, x cage tubes {ov(TW, ctx_mesh['cage shell tubes']):.1f}, "
    f"x the v5 links {ov(TW, ctx_mesh[KEELS_NM]):.1f} (all of it the keels and braces at their roots: links minus those tubes "
    f"{(MAN_OF(TW) ^ (MAN_OF(ctx_mesh[KEELS_NM]) - MAN_OF(cat([br_tubes] + [trimesh.creation.cylinder(3.4, np.linalg.norm(b - a), sections=24, segment=np.vstack([a, b])) for a, b in KEELS.values()])))).volume():.1f}), "
    f"x the brace tubes {ov(TW, br_tubes):.1f} mm³ (inside the bores; C); x the buck {ov(TW, buck):.1f}, x the Pi {ov(TW, cat([pi, pi_plugs])):.1f}")

# ---------------------------------------------------------------- system check, mass, CG
from system_check import run as SC_RUN
res = SC_RUN(MAN)
say(res["summary"])
json.dump(res, open(os.path.join(OUT, "system_check.json"), "w"), indent=1, default=str)

# ---------------------------------------------------------------- joint reactions per piece (rigid body, least-norm bolt forces, E)
G = 9.81e-3                                         # N per g of mass per g of acceleration
DIRS14 = [np.array(v, float) / np.linalg.norm(v) for v in
          [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] + [(a, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)]]
M_TOT = float(res["mass_cg"]["mass_g"])
mass_c = {"stack": (24 + 5, stack.centroid), "Pi": (55, pi.centroid), "UBEC": (15, ubec.centroid), "buck": (6, buck.centroid), "RX": (2, rx.centroid),
          "deck wires": (20, np.array([0, 0, 8.0])), "swap set": (650 + 105.6 + 7.5, pack.centroid), "receiver set": (229.3 + 7.5, receiver.centroid),
          "flow": (1.5, flow.centroid), "hub deck": (MASS["hub deck"], DECK.centroid), "flow clip": (MASS["flow clip"], CLIP.centroid)}


def bolt_forces(pts_, F, Mc, c):
    """least-norm forces f_i at the bolt points: sum f = F, sum (p_i - c) x f_i = Mc; the support set must hold every moment"""
    P_ = np.asarray(pts_, float) - c; n = len(P_); A_ = np.zeros((6, 3 * n))
    for i, r in enumerate(P_):
        A_[0:3, 3 * i:3 * i + 3] = np.eye(3)
        A_[3:6, 3 * i:3 * i + 3] = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    assert np.linalg.matrix_rank(A_) == 6, "supports collinear: rank < 6"
    f = np.linalg.pinv(A_) @ np.concatenate([F, Mc]); return f.reshape(n, 3)


def piece_case(supports, loads):
    c = np.mean(np.asarray(supports, float), 0)
    F = sum(l[0] for l in loads); Mc = sum(np.cross(np.asarray(l[1]) - c, l[0]) for l in loads)
    return bolt_forces(supports, -F, -Mc, c)


def inertial(names, acc):
    return [(-mass_c[n][0] * G * acc, mass_c[n][1]) for n in names]


SUP = {"hub deck": [(x, y, 0.0) for x, y in FEET_XY], "hub deck, front 4 feet alone (compliant deck bound)": [(x, y, 0.0) for x, y in FEET_XY[2:]],
       "flow clip": [(96.0, sy * 17.0, Z_PLATE_BOT) for sy in (1, -1)] + [(FLOW_X + 12.8, 0.0, Z_PLATE_BOT)],
       "Pi screws": [(x, y, Z_PI) for x, y in PI_HOLES], "stack screws": [(x, y, Z_TP) for x, y in STACK_HOLES]}
CARRY = {"hub deck": ["Pi", "UBEC", "buck", "deck wires", "hub deck"], "hub deck, front 4 feet alone (compliant deck bound)": ["UBEC", "buck"],
         "flow clip": ["flow", "flow clip"], "Pi screws": ["Pi"], "stack screws": ["stack"]}
KEEL_CASES = {"keels push (both 387 N, F)": (-1, -1), "keels pull (both 274 N, F)": (1, 1), "keel +y push, -y pull (F)": (-1, 1)}
BRACE_CASES = {"top hit: keels push 387 + braces pull 712 a side (F)": ((-1, -1), 1), "braces pull alone (E)": (None, 1), "braces push alone (E)": (None, -1)}


def keel_loads(signs):
    out_ = []
    for (sy, (a, b)), sg in zip(KEELS.items(), signs):
        d = (b - a) / np.linalg.norm(b - a)
        out_.append((d * (KEEL_N[0] if sg > 0 else KEEL_N[1]) * sg, a))   # the tube pulls the socket toward the head (+) or pushes it in (-)
    return out_
cases = {f"crash 35 g {k}": 35 * d for k, d in enumerate(DIRS14)}
cases.update({"flight 4 g": np.array([0, 0, 4.0]), "flight 2 g x": np.array([2.0, 0, 1]), "flight 2 g y": np.array([0, 2.0, 1]), "landing 6 g": np.array([0, 0, 6.0])})
REQ = {k: {"worst_case": None, "max_bolt_N": 0.0, "max_shear_N": 0.0, "max_axial_N": 0.0} for k in SUP}


def rec(nm, case, f, axis=np.array([0, 0, 1.0])):
    ax = np.abs(f @ axis); sh = np.linalg.norm(f - np.outer(f @ axis, axis), axis=1); tot = np.linalg.norm(f, axis=1)
    r_ = REQ[nm]
    if tot.max() > r_["max_bolt_N"]: r_.update(worst_case=case, max_bolt_N=round(float(tot.max()), 1))
    r_["max_shear_N"] = round(max(r_["max_shear_N"], float(sh.max())), 1); r_["max_axial_N"] = round(max(r_["max_axial_N"], float(ax.max())), 1)


for case, a in cases.items():
    for nm in SUP:
        rec(nm, case, piece_case(SUP[nm], inertial(CARRY[nm], a)))
for case, sg in KEEL_CASES.items():          # the head's keel forces on the hub (frame model maxima, F), with the deck's own 35 g down
    for nm in ("hub deck", "hub deck, front 4 feet alone (compliant deck bound)"):
        rec(nm, case, piece_case(SUP[nm], keel_loads(sg) + inertial(CARRY[nm], np.array([0, 0, -35.0]))))
def brace_loads(sg):
    return [((b - a) / np.linalg.norm(b - a) * BRACE_N[lvl] * sg, a) for (lvl, sy), (a, b) in BRACES.items()]
RF = json.load(open(os.path.join(OUT, "core_root_forces.json")))      # the cage frame model's per-case root forces (F on E, no SF)
RP = {k: (np.array(v["core"]), np.array(v["lug"])) for k, v in RF["points"].items()}
PAD_N = {"fwd_N_m": 0.0, "fwd_case": None, "back_N_m": 0.0, "back_case": None}
for case, nn_ in RF["cases"].items():
    for sy in (1, -1):                                    # one side's 3 tubes (keel + 2 braces), SF 1.1: moment about the toe line (y axis)
        My = 0.0
        for k, n in nn_.items():
            if not k.endswith("+1" if sy > 0 else "-1"): continue
            a, b = RP[k]; F = (b - a) / np.linalg.norm(b - a) * n * 1.1; r = a - np.array([TOE_X, a[1], 0.0])
            My += (r[2] * F[0] - r[0] * F[2])                 # + = tips the tower forward (its back rises)
        key = ("fwd_N_m", "fwd_case") if My > 0 else ("back_N_m", "back_case")
        if abs(My) / 1000 > PAD_N[key[0]]: PAD_N.update({key[0]: round(abs(My) / 1000, 1), key[1]: case})
PAD_N["mid_insert_pull_N"] = round(PAD_N["fwd_N_m"] * 1000 / (TOE_X - MID_X) / 2, 0)        # forward tip: the 2 mid feet a side pull
PAD_N["front_insert_pull_N"] = round(PAD_N["back_N_m"] * 1000 / (FRONT_X - MID_X) / 2, 0)   # back tip about the mid feet: the front... (1 a side)
PAD_N["front_insert_pull_N"] *= 2
for case, (ks, bs) in BRACE_CASES.items():
    for nm in ("hub deck", "hub deck, front 4 feet alone (compliant deck bound)"):
        rec(nm, case, piece_case(SUP[nm], (keel_loads(ks) if ks else []) + brace_loads(bs) + inertial(CARRY[nm], np.array([0, 0, -35.0]))))
say("joint reactions (rigid body, least-norm, E masses; floor loads go to the cage feet, review alternative B):")
say(f"   tower tipping (SF 1.1, the cage's per-case root forces, F, the deep webs carrying it to the feet): forward {PAD_N['fwd_N_m']} N m a side ({PAD_N['fwd_case']}) "
    f"-> {PAD_N['mid_insert_pull_N']:.0f} N pull on each of the 2 mid inserts a side over {TOE_X - MID_X:.0f} toe -> mid feet; back {PAD_N['back_N_m']} N m ({PAD_N['back_case']}) "
    f"-> {PAD_N['front_insert_pull_N']:.0f} N on the front insert a side over {FRONT_X - MID_X:.1f} (E: insert pull-out ~600, coupon P to come)")
for k, v in REQ.items():
    say(f"   {k}: max {v['max_bolt_N']} N per bolt ({v['worst_case']}), shear {v['max_shear_N']}, axial {v['max_axial_N']}")
say("   the 2 front deck screws clamp the flow clip's 3 flange and the plate (M3 x 10, inserts in the deck); the front 4 bound puts every keel load on (96, ±17) + (65, ±22.5)")
f1 = None
cg = np.asarray(res["mass_cg"]["cg"], float)
tw = 4 * 1650 / M_TOT
say(f"mass {M_TOT:.0f} g (E), T/W {tw:.2f} at 1650 gf a motor; CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f})")
result = {"mass_g": M_TOT, "tw": round(tw, 2), "cg": cg.tolist(), "requirements": REQ, "wash_pct": wash,
          "keels": {str(sy): [a.tolist(), b.tolist()] for sy, (a, b) in KEELS.items()}, "keel_n": KEEL_N,
          "braces": {f"{l} {sy}": [a.tolist(), b.tolist()] for (l, sy), (a, b) in BRACES.items()}, "brace_n": BRACE_N,
          "brace_socket": {"d": BR_BORE, "l": BR_L, "lead": BR_LEAD, "wall": BR_WALL}, "tipping": PAD_N,
          "socket": {"d": SOCK_D, "l": SOCK_L, "off": SOCK_OFF, "od": SOCK_OD}, "insert": INSERT, "foot_h": FOOT_H,
          "slot": {"x": [float(RB[0][0]), float(RB[1][0])], "y": float(RB[1][1]), "z": [Z_CART, Z_PLATE_BOT]},
          "z": {"mount_plane": Z_M, "pack_top": float(pack.bounds[1][2]), "cartridge_bottom": float(cartridge.bounds[0][2]), "lens": LENS_Z,
                "esc": Z_ESC, "stack_top": Z_STACK_TOP, "pi_board": Z_PI, "cooler_top": cool_top, "foot": Z_FOOT, "top_plate_top": Z_TP},
          "receiver_bolts": rec_bolts, "feet_xy": FEET_XY, "pi_holes": PI_HOLES, "arm_screws_driver_holes": ARM_SCREWS,
          "budget_g": BUDGET, "system": res["summary"],
          "coupon": {"floor_t": FLOOR_T, "post_h": POST_H, "post_d": POST_D, "stack_p": STACK_P, "z_pi": Z_PI, "pi_x": [PI_X0, PI_X1], "rail_y": RAIL_Y, "bay_y": BAY_Y,
                     "ubec_bay": UBEC_BAY, "buck_bay": BUCK_BAY, "rx_tp": RX_TP, "flow": [FLOW_X, FLOW_Y], "lens_z": LENS_Z, "clip_floor": CLIP_FLOOR,
                     "z_plate_bot": Z_PLATE_BOT}}
json.dump(result, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
