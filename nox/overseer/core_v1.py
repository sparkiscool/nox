"""Overseer core v1: system pass for everything inside the cage except the arms (Spark 27 Oct: "design everything else left on
the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual
mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now"; "no powerboard the stack
for the fc has a esc under it, the camera and the lidar isnt your job ... make sure you get the right dimensions and give the
appropriate space for the parts, and then design coupons for the least amount of materials for me to confirm mounting").

Round 3 (27 Oct, after the concept review FAIL and Spark's corrections; Spark: "make sure bolts are accessible through a
screwdriver, the whole thing need to be in as little pieces as possible and easy to work with"): no PDB; the FC stack
(MicoAir743-Lite on a MicoAir 55A AM32 4-in-1 ESC, 17.5 tall, 24 g) at the plate centre; the Pi 5 right side up on the rear wing
beside it (its Active Cooler is 13.7 tall with the blower intake on top: hung over the stack it would breathe through a 2 mm gap),
cooler under the top plate's rear edge and in the open past it, ports aft; UBEC and 12 V buck under the Pi deck; RX ahead of the
stack. Two printed pieces:
  hub deck   PETG on the bottom plate: 6 feet on the plate's Ø3.1 / Ø6 holes (±24, ±36.5), (-72.5, ±31.75), M3 from above
             through the plate into the sled's inserts; 4 stack posts on 30.5 (M3 short inserts); the Pi deck (4 posts on its
             58 x 49, M2.5 self-tapping); UBEC, buck, RX bays (zip ties); battery lead + cap channel
  sled       PETG under the bottom plate: the receiver's mount face, the 6 bolts' inserts, the flow pod (MTF-02P pocket, lens on
             the cartridge bottom plane) and a +y rib for the down camera owner's pod; Ø9 driver holes under the 8 arm screws
Every screw is driven straight down with a screwdriver in assembly order: hub deck + sled (6), stack (4), Pi (4), then arms and
the top plate. Cameras, lidar and their mounts are not this session's. Floor contact: TPU feet on the cage's opening ring nodes
(cage session, review alternative B; placeholders here).

Context (not edited here): layout v5 / cage v3 (`nox/overseer/overseer.assembly.json`): plates, arms, motors, props, cage,
sensor head. The latch set is battery latch v4's Overseer meshes in pose (mechanism not designed here).

python3 nox/overseer/core_v1.py [out_dir]     (~1 min)
Drone frame: x forward, y left, z up; z 0 = bottom plate top; plate centre x 0. Numbers: `nox/dimensions.md` O, M, L,
"System Overseer core".
"""
import os, sys, json, math, io, contextlib
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox")); sys.path.insert(0, os.path.join(ROOT, "nox", "box"))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "overseer_core", "v1")
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
SLED_T = 6.0                       # sled under the bottom plate (DEFAULT: receiver v2 asks a hole >= 6 deep under each M3 x 5.7 insert)
Z_M = Z_PLATE_BOT - SLED_T         # receiver mount plane (its deck top) -8.1
Y_M = 68.8                         # receiver v2 mount plane above the cartridge bottom (M, battery_latch r["Y_M"])
Z_CART = Z_M - Y_M                 # cartridge bottom -76.9
Z_T = 85.55                        # tab line (swap CG) in cartridge z (M); sits on the catch centre
BX = 27.8                          # receiver v2 bolt line x (M)
MOUNT_ZC = (-1.3, 85.55, 163.8)    # receiver v2 bolts along the cartridge (M)
# FC stack: MicoAir743-Lite 36 x 36 x 8, 10 g (spec) on a MicoAir 55A AM32 4-in-1 44 x 43.5 x 5.2, 14 g (listing; the stack's
# 17.5 / 24 g on the MicoAir drawing = FC 10 + this ESC 14, E: model CHECK), 30.5 x 30.5 Ø4 holes, M3 in grommets (usual, E).
# On the top plate (Spark 27 Oct: "the fc mounts to the top plates lines so just move that there"), centred on the plate.
STACK_P = 30.5 / 2
ESC_L, ESC_W, ESC_T = 44.0, 43.5, 5.2
FC_HALF, FC_T, STACK_H = 18.0, 8.0, 17.5
Z_TP = 46.6                        # top plate top (S500 STEP, derived; 1.6 plate on the 45 gap)
TP_HOLES = [(35.5, 0.0), (-35.5, 0.0), (0.0, 35.5), (0.0, -35.5)]       # top plate Ø3.0 (derived, STEP): the stack mount's 4 M3
FLOOR_T = 4.0                      # 6 x 4 bars and Ø12 pads (the solid gate's 3.0 neck test)
POST_H, POST_D = 5.0, 11.0         # stack posts (wall 3.5 round the insert): M3 short heat-set insert (Ruthex M3 x 4.0, hole Ø4.0 x 5, Q17)
GROM = 1.0                         # grommet under the ESC (E)
Z_ESC = Z_TP + POST_H + GROM       # 52.6
Z_STACK_TOP = Z_ESC + STACK_H      # 70.1
CAP = (10.0, 20.0)                 # low-ESR cap on the ESC battery pads, 35 V 470 µF Ø10 x 20, lying on the top plate behind the ESC (E)
RX_TP = (-30.0, -18.0, 24.0, 40.0)  # RadioMaster XR2 16 x 12 x 6 (spec) on the top plate beside the stack, zip-tied to the mount's tab
FEET_XY = [(24.0, 36.5), (24.0, -36.5), (-24.0, 36.5), (-24.0, -36.5), (-65.0, 22.5), (-65.0, -22.5)]   # bottom plate Ø3.1 holes, 360° faces in the STEP
# (review 27 Oct: (-72.5, ±31.75) are R3 outline fillets, not holes; (-65, ±36.5) sit under the rear arm roots)
# Pi 5: board 85 x 56 x 1.6, M2.5 holes Ø2.7 on 58 x 49, 3.5 from the far edge and 23.5 from the USB-A / RJ45 edge (spec);
# Active Cooler 63.5 x 42.5 footprint, 13.7 tall over the board, blower intake on top (Raspberry Pi product brief, Jan 2026);
# USB-A / RJ45 16 over the board (spec); parts under the board 1.5 (microSD, E); 46 + 9 g ~ 55 g (E)
PI_CX = float(os.environ.get("CORE_PI_CX", -11.0))   # Pi centre between the plates (scanned: 0 mm³ with the arm roots, plugs aft through the gap)
PI_X0, PI_X1, PI_W = PI_CX - 42.5, PI_CX + 42.5, 56.0
PI_HOLES = [(PI_X0 + 23.5 + dx, sy * 24.5) for dx in (0.0, 58.0) for sy in (1, -1)]
PI_POST = 4.0                      # posts on the floor bars, M2.5 self-tapping Ø2.2 x 4 (DEFAULT)
Z_PI = FLOOR_T + PI_POST           # board underside 8.0
PI_BOARD_T, PI_UP, PI_COOL = 1.6, 16.0, 13.7
COOL_X = (PI_X1 - 63.5 + 2.0, PI_X1 + 2.0)     # cooler over the SoC end of the board (E place)
RAIL_Y = 22.0                      # floor rails at |y| 22: clear of the arm roots' inner corners (C)
# front wing bays (zip ties, no screws): UBEC and buck between the front arm roots, ahead of the Pi, under the top plate's front
UBEC_BAY = (34.0, 82.0, -19.0, 2.0, FLOOR_T, FLOOR_T + 10.0)     # any 5 V 5 A UBEC of 43-50 x 17-25 x 7-10 (Q4 range)
BUCK_BAY = (34.0, 58.0, 4.0, 22.0, FLOOR_T, FLOOR_T + 10.0)      # 12 V buck (model open: 24 x 18 x 10 bay, E)
# flow pod in the sled (MTF-02P 21.6 x 16 x 6.5, 1.5 g, spec; no holes: pocket)
POD_X, FLOW_Y = -10.0, -55.0
LENS_Z = Z_CART                    # lens on the cartridge bottom plane (C)
RIB_X = (-32.0, 12.0)              # sled ribs (pods), off every arm screw key line (C)
ARM_SCREWS = [(sx * a, sy * b) for sx in (1, -1) for sy in (1, -1) for a, b in ((52.14, 38.0), (38.0, 52.14))]   # bottom plate (derived)

# ---------------------------------------------------------------- context: v5 manifest
v5 = json.load(open(V5))
REPLACED = {"hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", "wires", "pack", "cartridge", "receiver", "belly sled + pods",
            "flow MTF-02P", "TPU feet"}
context = [dict(p) for p in v5["parts"] if p["name"] not in REPLACED]
ctx_mesh = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in context}
ARMS = [ctx_mesh[f"arm {i}"] for i in (1, 2, 3, 4)]
for p in context:
    if p["name"] == "down camera":
        p["note"] = "v5 pose; its pod is the camera owner's (Spark 27 Oct), hung from the sled's +y rib"

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
cap = box(-62.0, -62.0 + CAP[1], -CAP[0] / 2, CAP[0] / 2, Z_TP, Z_TP + CAP[0])   # on the battery leads where they come over the plate's rear edge
rx = box(RX_TP[0], RX_TP[1], RX_TP[2], RX_TP[3], Z_TP, Z_TP + 6)
pi = cat([box(PI_X0, PI_X1, -PI_W / 2, PI_W / 2, Z_PI, Z_PI + PI_BOARD_T),
          box(PI_X0 - 2, PI_X0 + 21, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_UP),                  # USB-A x4 + RJ45, aft
          box(COOL_X[0], COOL_X[1], -21.25, 21.25, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_COOL)])                            # Active Cooler
pi_plugs = box(PI_X0 - 2 - 18.0, PI_X0 - 2, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI + PI_BOARD_T, Z_PI + PI_BOARD_T + PI_UP)         # plug bodies (keep-out, E)
fan_air = box(COOL_X[0] + 10, COOL_X[1] - 25, -15, 15, Z_PI + PI_BOARD_T + PI_COOL, Z_PI + PI_BOARD_T + PI_COOL + 8)           # intake room (keep-out, E)
ubec = box(UBEC_BAY[0], UBEC_BAY[0] + 48, -18, 2, UBEC_BAY[4], UBEC_BAY[4] + 9)              # a 48 x 20 x 9 UBEC (Q4 middle, E)
buck = box(BUCK_BAY[0] + 1, BUCK_BAY[0] + 21, 5, 21, BUCK_BAY[4], BUCK_BAY[4] + 8)           # 20 x 16 x 8 (E)
flow = box(POD_X - 10.8, POD_X + 10.8, FLOW_Y - 8, FLOW_Y + 8, LENS_Z, LENS_Z + 6.5)          # MTF-02P (spec)


# ---------------------------------------------------------------- the three core pieces (concept bodies for the piece runs; the coupons are their mounting features)
def stack_mount():
    """on the top plate: 4 pads on its Ø3.0 holes (M3 up from under the plate into short inserts, on the bench), 4 posts for the stack"""
    pads = [cyl(6, Z_TP, Z_TP + POST_H, x, y) for x, y in TP_HOLES]
    posts = [cyl(POST_D / 2, Z_TP, Z_TP + POST_H, sx * STACK_P, sy * STACK_P) for sx in (1, -1) for sy in (1, -1)]
    bars = [bar(p_, (np.sign(p_[0] or 1) * STACK_P if p_[0] else s2 * STACK_P, np.sign(p_[1] or 1) * STACK_P if p_[1] else s2 * STACK_P), 6, Z_TP, Z_TP + FLOOR_T)
            for p_ in TP_HOLES for s2 in (1, -1)]
    tab = [bar((-35.5, 0), (-33, 21), 6, Z_TP, Z_TP + FLOOR_T)]                                   # RX tab (zip tie)
    return uni(pads + posts + bars + tab)


def hub_deck():
    """on the bottom plate: 6 feet (M3 down through the plate into the sled), the Pi's 4 posts, the UBEC + buck bays ahead of it"""
    feet = [cyl(6, 0, FLOOR_T, x, y) for x, y in FEET_XY]
    px0, px1 = PI_HOLES[0][0], PI_HOLES[2][0]
    pip = [cyl(3.5, 0, Z_PI, x, y) for x, y in PI_HOLES]
    bars = [bar((24, sy * 36.5), (px1, sy * 24.5), 6, 0, FLOOR_T) for sy in (1, -1)]
    bars += [bar((-24, sy * 36.5), (px0, sy * 24.5), 6, 0, FLOOR_T) for sy in (1, -1)]
    bars += [bar((px0, 24.5), (px0, -24.5), 6, 0, FLOOR_T), bar((px1, 24.5), (px1, -24.5), 6, 0, FLOOR_T)]
    bars += [bar((-65.0, sy * 22.5), (px0, sy * RAIL_Y), 6, 0, FLOOR_T) for sy in (1, -1)]
    bars += [bar((px1, sy * RAIL_Y), (80, sy * RAIL_Y), 6, 0, FLOOR_T) for sy in (1, -1)]                        # bay rails
    bars += [bar((px0, sy * RAIL_Y), (px1, sy * RAIL_Y), 6, 0, FLOOR_T) for sy in (1, -1)]                       # under the Pi (board z 8)
    bars += [bar((80, RAIL_Y), (80, -RAIL_Y), 6, 0, FLOOR_T), bar((57, RAIL_Y), (57, -RAIL_Y), 6, 0, FLOOR_T)]
    return uni(feet + pip + bars)


def sled():
    plate = box(-115.0, 58.0, -56.0, 56.0, Z_M, Z_PLATE_BOT)
    ribs = [box(RIB_X[0], RIB_X[1], sy * 36, sy * 56, Z_M - 14, Z_M) for sy in (1, -1)]
    pod = box(POD_X - 15, POD_X + 15, FLOW_Y - 11.5, FLOW_Y + 11.5, LENS_Z, Z_M - 14)
    return uni([plate, pod] + ribs)


SMOUNT, DECK, SLED = stack_mount(), hub_deck(), sled()

# ---------------------------------------------------------------- wiring: drawn routes, each ending in a plug (clean wiring, L 8)
W = {
    "battery 12 AWG pair: receiver XT60 -> up the rear wing past the top plate's rear edge -> ESC battery pads (rear edge, on top), cap there": (
        [(-108.3, 0, xt60.bounds[1][2]), (-100, 0, 4), (-68, 0, 30), (-64, 0, Z_TP + 3), (-24, 0, Z_ESC + 2)], 4.0),
    "UBEC + buck in: ESC battery pads -> over the top plate's front edge -> front bays": ([(-20, -8, Z_ESC + 2), (40, -8, Z_TP + 2), (64, -8, 30), (60, -8, 10)], 1.5),
    "UBEC 5 V -> Pi GPIO 5 V pins (+y edge)": ([(40, 0, 9), (32, 20, 12), (PI_CX, 24, Z_PI + 4)], 1.5),
    "buck 12 V -> nose (the L2 owner takes it there)": ([(56, 12, 8), (90, 0, 6)], 1.5),
    "FC <-> Pi (USB), out past the top plate's -y edge": ([(0, -18, Z_STACK_TOP - 3), (0, -58, Z_TP + 2), (0, -62, 30), (PI_X0 - 8, -15, Z_PI + 8)], 1.5),
    "FC <-> RX": ([(-18, 10, Z_STACK_TOP - 3), (-32, 26, Z_TP + 4)], 1.5),
    "FC UART -> flow (out at -y, down past both plate edges to the pod)": ([(10, -18, Z_STACK_TOP - 4), (8, -58, Z_TP + 2), (6, -62, 20), (0, -61, 2), (-4, -61, Z_M - 16), (POD_X, FLOW_Y - 4, LENS_Z + 8)], 1.5),
}
# motor leads: from each ESC corner straight out along the top of its arm (on top of the plate now: no root to pass)
for k, ang in enumerate((45, 135, 225, 315)):
    d = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
    W[f"motor lead {k + 1}: ESC corner -> along the top of arm {k + 1}"] = ([(*(28 * d), Z_ESC + 3), (*(70 * d), Z_TP + 3), (*(110 * d), Z_TP + 4)], 2.0)
wires = cat([tube(p, r) for p, r in W.values()])

# ---------------------------------------------------------------- manifest
def P(name, m, role, mass=None, material="PETG", protect=False, tag="E", swap=False, owner="overseer core", budget=None, version="core v1"):
    fn = name.lower().replace(" ", "_").replace("/", "_").replace("+", "").replace("(", "").replace(")", "").replace(",", "")[:40] + ".stl"
    m.export(os.path.join(PARTS, fn))
    d = {"name": name, "role": role, "mesh": os.path.relpath(os.path.join(PARTS, fn), ROOT), "material": material, "tag": tag,
         "owner": owner, "version": version, "source": "nox/overseer/core_v1.py"}
    if mass is not None: d["mass_g"] = round(float(mass), 2)
    if protect: d["protect"] = True
    if swap: d["swap"] = True
    if budget: d["budget_g"] = budget
    return d


BUDGET = {"stack mount": 8, "hub deck": 24, "sled": 75}   # g, soft slots (DEFAULT)
# floor contact: TPU feet on 4 opening ring nodes (cage session; placeholders at v5's ring nodes, contact 6 under the lenses)
RING = [(146.0, 43.2), (146.0, -43.2), (-102.9, 104.3), (-102.9, -104.3)]
Z_FOOT = LENS_Z - 6.0
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
    P("cartridge", cartridge, "protected", 105.6, protect=True, swap=True, tag="M", owner="box swap (battery latch)", version="cartridge v6 + sliders v6"),
    P("receiver", receiver, "protected", 229.3, protect=True, tag="M", owner="box swap (battery latch)", version="receiver v2 + carrier v1 + cap + strips"),
    P("XT60 pair", xt60, "bought", 15, protect=True),
    P("stack mount", SMOUNT, "structure", BUDGET["stack mount"], budget=BUDGET["stack mount"]),
    P("hub deck", DECK, "structure", BUDGET["hub deck"], budget=BUDGET["hub deck"]),
    P("sled", SLED, "structure", BUDGET["sled"], budget=BUDGET["sled"]),
    P("TPU feet (cage session, placeholders)", feet, "cage", 6.4, material="TPU", owner="overseer cage", version="placeholder"),
]
CORE_NAMES = {p["name"] for p in core}
contact = [list(c) for c in v5["contact_pairs"] if not (set(c) & REPLACED)]
contact += [["stack mount", "top plate"], ["stack mount", STACK_NM], ["stack mount", "RX XR2"], ["ESC cap", "top plate"], ["RX XR2", "top plate"],
            ["hub deck", "bottom plate"], ["hub deck", PI_NM], ["hub deck", "UBEC 5 V"], ["hub deck", "buck 12 V"],
            ["ESC cap", STACK_NM], ["sled", "bottom plate"], ["sled", "receiver"], ["sled", "XT60 pair"], ["sled", "flow MTF-02P"],
            ["pack", "cartridge"], ["pack", "receiver"], ["cartridge", "receiver"], ["receiver", "XT60 pair"], ["cartridge", "XT60 pair"], ["pack", "XT60 pair"]]
contact += [["wires", n] for n in sorted(CORE_NAMES | {"bottom plate", "top plate", "arm 1", "arm 2", "arm 3", "arm 4", "apex + cam bracket"}) if n != "wires"]
openings = [o for o in v5["openings"] if o.get("part") not in ("pack", "cartridge", "receiver", "flow MTF-02P")]
openings += [{"part": p_, "axis": [0, 0, -1], "half_angle_deg": 40.0, "skip_probes": True} for p_ in ("pack", "cartridge", "receiver", "XT60 pair", "flow MTF-02P")]

# access (CLAUDE.md 2; Spark 27 Oct: "make sure bolts are accessible through a screwdriver"): every core screw is driven straight
# down with a screwdriver (shaft Ø6, 150 long) at its assembly step, so each path ignores what goes on later (top plate, Pi, stack)
DRV = {"type": "cylinder", "d": 6}
LATER = ["top plate", "prop 1", "prop 2", "prop 3", "prop 4", "cage shell tubes", "cage shell nodes", "spokes, ties, keels, L2 guard ring", "wires"]
acc = [a for a in v5["access"] if a["name"].startswith("M3")]
for a in acc:
    a["ignore"] = [i for i in a["ignore"] if i in ("arm 1", "arm 2", "arm 3", "arm 4", "prop 1", "prop 2", "prop 3", "prop 4", "down camera")] + (["sled"] if a["part"] == "bottom plate" else [])
acc += [{"name": "pack swap: cartridge + pack down through the opening", "part": "cartridge", "dir": [0, 0, -1], "length_mm": 150.0, "tool": "part", "ignore": ["pack", "receiver", "XT60 pair"]},
        {"name": "FC USB-C (-y edge, on the top plate)", "part": STACK_NM, "from": [0, -FC_HALF, Z_STACK_TOP - 4],
         "dir": [0.0, -0.98, 0.2], "length_mm": 300.0, "tool": {"type": "box", "w": 12, "h": 7}, "ignore": ["wires"]},
        {"name": "Pi USB-A / RJ45 plugs (aft, in the open)", "part": PI_NM, "from": [PI_X0 - 20, 0, Z_PI + 9], "dir": [-1, 0, 0], "length_mm": 40.0,
         "tool": {"type": "box", "w": 40, "h": 14}, "ignore": ["wires"]}]
for k, (x, y) in enumerate(FEET_XY):
    acc.append({"name": f"step 1: M3 x 16 hub deck + plate into the sled insert, foot {k} (screwdriver, top plate not on yet)", "part": "hub deck",
                "from": [x, y, FLOOR_T + 0.5], "dir": [0, 0, 1], "length_mm": 150.0, "tool": DRV, "ignore": LATER + [PI_NM]})
for k, (x, y) in enumerate(PI_HOLES):
    acc.append({"name": f"step 2: M2.5 Pi screw {k} (screwdriver, top plate not on yet)", "part": PI_NM, "from": [x, y, Z_PI + PI_BOARD_T + 0.5], "dir": [0, 0, 1],
                "length_mm": 150.0, "tool": {"type": "cylinder", "d": 5}, "ignore": LATER})
for k, (x, y) in enumerate(TP_HOLES):             # on the bench: the top plate upside down, M3 up into the mount's inserts
    acc.append({"name": f"step 3 (bench): M3 stack mount screw {k} up through the top plate", "part": "stack mount", "from": [x, y, Z_TP - 1.7 - 0.5], "dir": [0, 0, -1],
                "length_mm": 150.0, "tool": DRV, "ignore": ["top plate", "stack mount", "wires", PI_NM, "hub deck", "bottom plate", "sled", "receiver", "cartridge", "pack", "XT60 pair",
                                                          "flow MTF-02P", "UBEC 5 V", "buck 12 V", "down camera", "arm 1", "arm 2", "arm 3", "arm 4", "TPU feet (cage session, placeholders)",
                                                          "cage shell tubes", "cage shell nodes", "spokes, ties, keels, L2 guard ring"]})
for k, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
    acc.append({"name": f"step 5: M3 stack screw {k} into the mount's post insert (screwdriver, everything assembled)", "part": STACK_NM,
                "from": [sx * STACK_P, sy * STACK_P, Z_STACK_TOP + 0.5], "dir": [0, 0, 1], "length_mm": 150.0, "tool": DRV,
                "ignore": ["prop 1", "prop 2", "prop 3", "prop 4", "wires"]})

man = {k: v for k, v in v5.items() if k not in ("parts", "contact_pairs", "openings", "access", "note", "name")}
man.update({"name": "Overseer core v1", "machine": "overseer",
            "note": ("Overseer core v1 round 4 (27 Oct, Spark: everything inside the cage except the arms; latch mechanism, body nodes, cameras and "
                     "lidar not this session's; screwdriver access, fewest pieces; the FC stack on the top plate): v5 / cage v3 context parts unchanged (nox/overseer/overseer.assembly.json); "
                     "the core, the real latch set (battery latch v4 Overseer meshes) and the feet placeholders on the cage's opening ring from nox/overseer/core_v1.py."),
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
say(f"stack on the top plate: ESC z {Z_ESC:.1f}..{Z_ESC + ESC_T:.1f} on {POST_H:.0f} posts + grommets, FC top z {Z_STACK_TOP:.1f}; leads to ~{Z_STACK_TOP + 3:.1f}; "
    f"x props {ov(cat([stack, cap, rx]), props):.1f} mm³; nearest blade band z {82 - 5:.0f} (C on E)")
cool_top = Z_PI + PI_BOARD_T + PI_COOL
say(f"Pi right side up between the plates: board x {PI_X0:.1f}..{PI_X1:.1f}, z {Z_PI}..{Z_PI + PI_BOARD_T}; Active Cooler top z {cool_top:.1f} "
    f"({Z_TOP_UNDER - cool_top:.1f} under the top plate for the blower intake); ports aft, top z {Z_PI + PI_BOARD_T + PI_UP:.1f} (C on E)")
fits = [f"{nm} {sum(ov(m, arm) for arm in ARMS):.1f}" for nm, m in (("hub deck", DECK), ("Pi + plugs", cat([pi, pi_plugs])), ("UBEC", ubec), ("buck", buck),
                                                                    ("stack mount", SMOUNT), ("stack", stack), ("cap", cap), ("RX", rx))]
say("   x arms (v5 meshes), mm³: " + ", ".join(fits))
say(f"   deck x Pi {ov(DECK, pi):.1f} (posts on its holes), deck x UBEC {ov(DECK, ubec):.1f}, deck x buck {ov(DECK, buck):.1f}, Pi + intake x top plate {ov(cat([pi, fan_air]), top_plate):.1f}, "
    f"Pi x UBEC {ov(cat([pi, pi_plugs]), ubec):.1f}, mount x stack {ov(SMOUNT, stack):.1f}, mount x cap {ov(SMOUNT, cap):.1f}, mount x RX {ov(SMOUNT, rx):.1f}, cap x stack {ov(cap, stack):.1f} mm³")
say(f"   sled x receiver {ov(SLED, receiver):.1f}, deck x sled {ov(DECK, SLED):.1f} mm³")
blk = []
for x, y in ARM_SCREWS:
    key = cyl(4.0, Z_FOOT - 30, Z_M - 0.5, x, y, 16)
    for nm, m in (("sled pod", SLED.slice_plane([0, 0, Z_M - 0.6], [0, 0, -1], cap=True)), ("receiver", receiver), ("flow", flow), ("down camera (v5)", ctx_mesh["down camera"])):
        v = ov(key, m)
        if v > 0.5: blk.append(f"({x:.0f},{y:.0f}) {nm} {v:.0f}")
say("arm bottom screw keys (Ø8 below the sled): " + (", ".join(blk) if blk else "all 8 clear (C); the sled gets a Ø9 hole at each"))


def view_clear(lens, half_x, half_y, meshes):
    hits = []
    for nm, m in meshes:
        p = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 4000, seed=1)[0]])
        d = p - lens; below = d[:, 2] < -0.5
        ax = np.degrees(np.arctan2(np.abs(d[:, 0]), -d[:, 2])); ay = np.degrees(np.arctan2(np.abs(d[:, 1]), -d[:, 2]))
        n = int((below & (ax < half_x) & (ay < half_y)).sum())
        if n: hits.append(f"{nm} {n}")
    return hits


h2 = view_clear(np.array([POD_X, FLOW_Y, LENS_Z]), 23, 23, [("receiver", receiver), ("cartridge", cartridge), ("feet", feet)])
say("flow view (42°, +2°): " + ("clear (C)" if not h2 else "blocked: " + ", ".join(h2)))
col = box(-DC - 179.6 / 2 - 30, -DC + 179.6 / 2 + 30, -76.2 / 2 - 30, 76.2 / 2 + 30, Z_FOOT - 50, Z_CART)
say(f"swap column (receiver + 30 a side, below the cartridge bottom): sled {ov(col, SLED):.1f}, flow {ov(col, flow):.1f}, feet {ov(col, feet):.1f} mm³")
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union
disks = unary_union([Point(ctx_mesh[f"motor {i}"].centroid[0], ctx_mesh[f"motor {i}"].centroid[1]).buffer(127.0, 64) for i in (1, 2, 3, 4)])
plan = {nm: MultiPoint(m.vertices[:, :2]).convex_hull for nm, m in (("hub deck + Pi + bays", cat([DECK, pi, ubec, buck])), ("sled + pod", SLED), ("receiver", receiver))}
wash = {nm: round(float(g.intersection(disks).area / disks.area * 100), 2) for nm, g in plan.items()}
say("prop wash (plan area under the disks, % of the disk area): " + ", ".join(f"{k} {v}" for k, v in wash.items()))

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
          "deck wires": (20, np.array([0, 0, 8.0])), "stack mount": (BUDGET["stack mount"], SMOUNT.centroid), "swap set": (650 + 105.6 + 7.5, pack.centroid), "receiver set": (229.3 + 7.5, receiver.centroid),
          "flow": (1.5, flow.centroid), "down camera + its pod (E)": (15 + 12, np.array([POD_X, 55.0, -50.0])),
          "hub deck": (BUDGET["hub deck"], DECK.centroid), "sled": (BUDGET["sled"], SLED.centroid)}


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


SUP = {"stack mount": [(x, y, Z_TP) for x, y in TP_HOLES], "hub deck": [(x, y, 0.0) for x, y in FEET_XY], "sled to plate": [(x, y, Z_PLATE_BOT) for x, y in FEET_XY],
       "receiver bolts": [(x, y, Z_M) for x, y in rec_bolts], "Pi screws": [(x, y, Z_PI) for x, y in PI_HOLES],
       "stack screws": [(sx * STACK_P, sy * STACK_P, Z_TP + POST_H) for sx in (1, -1) for sy in (1, -1)]}
CARRY = {"stack mount": ["stack", "RX", "stack mount"], "hub deck": ["Pi", "UBEC", "buck", "deck wires", "hub deck"], "receiver bolts": ["swap set", "receiver set"],
         "Pi screws": ["Pi"], "stack screws": ["stack"],
         "sled to plate": ["swap set", "receiver set", "sled", "flow", "down camera + its pod (E)"]}
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
say("joint reactions (rigid body, least-norm, E masses; floor loads go to the cage feet, review alternative B):")
for k, v in REQ.items():
    say(f"   {k}: max {v['max_bolt_N']} N per bolt ({v['worst_case']}), shear {v['max_shear_N']}, axial {v['max_axial_N']}")
say("   the 6 sled bolts pass through the hub deck feet and the plate: hub deck and sled loads add on the same M3 x 16 (inserts in the sled)")
Lp = (Z_M - 14) - LENS_Z
I_ = (30 * 23 ** 3 - 25 * 18 ** 3) / 12; mt = 1.5 + 0.24 * 2 * (30 + 23) * 2.5 * Lp * 1.27e-3
f1 = 1 / (2 * math.pi) * math.sqrt(3 * 2000 * I_ / Lp ** 3 / (mt * 1e-3) * 1e3)
say(f"flow pod: cantilever {Lp:.1f} under the sled rib, first mode ~{f1:.0f} Hz (E; target >= 100, L)")
cg = np.asarray(res["mass_cg"]["cg"], float)
tw = 4 * 1650 / M_TOT
say(f"mass {M_TOT:.0f} g (E), T/W {tw:.2f} at 1650 gf a motor; CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f})")
result = {"mass_g": M_TOT, "tw": round(tw, 2), "cg": cg.tolist(), "requirements": REQ, "pod_mode_hz": round(f1), "wash_pct": wash,
          "z": {"mount_plane": Z_M, "pack_top": float(pack.bounds[1][2]), "cartridge_bottom": float(cartridge.bounds[0][2]), "lens": LENS_Z,
                "esc": Z_ESC, "stack_top": Z_STACK_TOP, "pi_board": Z_PI, "cooler_top": cool_top, "foot": Z_FOOT, "top_plate_top": Z_TP},
          "receiver_bolts": rec_bolts, "feet_xy": FEET_XY, "pi_holes": PI_HOLES, "tp_holes": TP_HOLES, "arm_screws_driver_holes": ARM_SCREWS,
          "budget_g": BUDGET, "system": res["summary"],
          "coupon": {"floor_t": FLOOR_T, "post_h": POST_H, "post_d": POST_D, "stack_p": STACK_P, "z_pi": Z_PI, "pi_x": [PI_X0, PI_X1], "rail_y": RAIL_Y,
                     "ubec_bay": UBEC_BAY, "buck_bay": BUCK_BAY, "rx_tp": RX_TP, "aft_legs": [], "sled_t": SLED_T, "sled_x": [-115.0, 58.0], "sled_y": 56.0,
                     "flow": [POD_X, FLOW_Y], "z_plate_bot": Z_PLATE_BOT}}
json.dump(result, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
