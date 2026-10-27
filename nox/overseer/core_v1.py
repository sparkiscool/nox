"""Overseer core v1: system pass for everything inside the cage except the arms (Spark 27 Oct: "design everything else left on
the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual
mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now"; "no powerboard the stack
for the fc has a esc under it, the camera and the lidar isnt your job ... make sure you get the right dimensions and give the
appropriate space for the parts, and then design coupons for the least amount of materials for me to confirm mounting").

Round 2 (27 Oct, after the concept review FAIL and Spark's corrections): no PDB, the FC stack (MicoAir743-Lite on a MicoAir
55A AM32 4-in-1 ESC, 17.5 tall, 24 g) on the bottom plate; the Pi 5 hangs component side down under the top plate (a Pi over the
stack does not fit the 45 gap); UBEC + 12 V buck + RX in a tower on the rear wing past the top plate; the sled; the flow pod.
Cameras, lidar and their mounts are not this session's (the down camera's pod hangs from the sled's +y rib, its owner's piece).
Floor contact: TPU feet on the cage's opening ring nodes (cage session, alternative B of the review; placeholders here).

Context (not edited here): layout v5 / cage v3 (`nox/overseer/overseer.assembly.json`): plates, arms, motors, props, cage,
sensor head. The latch set is battery latch v4's Overseer meshes in pose (mechanism not designed here).

  stack mount  PETG on the bottom plate: 4 feet on the plate's Ø3.1 holes (±24, ±36.5), 4 grommet posts on 30.5 for the stack
  Pi hanger    PETG under the top plate: 4 pads on the top plate's Ø3.1 holes (±24.51, ±24.51), 4 M2.5 posts on the Pi's 58 x 49
  rear tray    PETG on the rear wing: 4 feet on the plate's Ø6 holes (-72.5, ±31.75), (-92.5, ±25.75); battery lead channel,
               UBEC bay over it, buck + RX bays on top; all inside |y| 13.5 (the strip between the rear prop disks)
  sled         PETG under the bottom plate: the receiver's mount face, the 8 hub bolts' inserts, ribs for the two pods
  flow pod     PETG, hangs from the sled's -y rib, MTF-02P lens on the cartridge bottom plane

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
# 17.5 / 24 g on the MicoAir drawing = FC 10 + this ESC 14, E: model CHECK), 30.5 x 30.5 Ø4 holes, M3 in grommets (usual, E)
STACK_P = 30.5 / 2
ESC_L, ESC_W, ESC_T = 44.0, 43.5, 5.2
FC_HALF, FC_T, STACK_H = 18.0, 8.0, 17.5
PAD_T, GROM = 3.0, 1.0             # stack mount pad, grommet under the ESC (E)
Z_ESC = PAD_T + GROM               # 4.0
Z_STACK_TOP = Z_ESC + STACK_H      # 21.5
PLUG = 6.0                         # SH1.0 plugs + lead bends over the FC (E)
CAP = (14.0, 10.0, 20.0)           # low-ESR cap on the ESC battery pads, 35 V 470 µF Ø10 x 20 lying (E)
MOUNT_HOLES = [(sx * 24.0, sy * 36.5) for sx in (1, -1) for sy in (1, -1)]       # bottom plate Ø3.1 (derived, STEP)
# Pi 5: board 85 x 56 x 1.6, M2.5 holes Ø2.7 on 58 x 49, 3.5 from the edges (spec); tallest above the board 16 (USB-A / RJ45),
# active cooler 10 (E: "lower than the USB ports"), under the board 1.5 (microSD, E); 46 g + cooler 9 g ~ 55 g (E)
PI_C = (-9.0, 0.0)                 # board centre; hung component side down; USB-A / RJ45 end at -x (clear of the stack)
PI_L, PI_W, PI_HX, PI_HY = 85.0, 56.0, 58.0, 49.0
PI_BOARD_T, PI_UP, PI_COOL, PI_UNDER = 1.6, 16.0, 10.0, 1.5
HANGER_T, PI_POST = 3.0, 3.0       # hanger pads under the top plate; posts so the Pi's underside parts clear it
TOP_HOLES = [(sx * 24.51, sy * 24.51) for sx in (1, -1) for sy in (1, -1)]       # top plate Ø3.1 (derived, STEP)
Z_PI_BACK = Z_TOP_UNDER - PI_POST                                                # board's back face (up)
Z_PI_COMP = Z_PI_BACK - PI_BOARD_T                                               # component face (down)
# rear tower (past the top plate's rear edge x -60.2, inside |y| 13.5: the strip between the rear prop disks, y > 12.5)
TRAY_HOLES = [(-72.5, 31.75), (-72.5, -31.75), (-92.5, 25.75), (-92.5, -25.75)]  # bottom plate Ø6 (derived), M3 + washer
UBEC_BAY = (-106.0, -58.0, -12.5, 12.5, 10.0, 20.0)   # 5 V 5 A UBEC, any of 43-50 x 17-25 x 7-10 (Q4 range), open ended along x
BUCK_BAY = (-106.0, -82.0, -12.5, 12.5, 23.0, 32.0)  # 12 V buck (model open: 24 x 25 x 9 bay, E)
RX_BAY = (-81.0, -74.0, -7.0, 7.0, 23.0, 40.0)        # RadioMaster XR2 16 x 12 x 6 (spec) standing on its end, antenna up
PI_PLUGS = 18.0                    # USB-A / RJ45 plug bodies past the Pi's -x ports before the cables bend (E)
# flow pod (MTF-02P 21.6 x 16 x 6.5, 1.5 g, spec; no holes: pocket)
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

# ---------------------------------------------------------------- bought parts in the hub (their real envelopes)
esc = box(-ESC_L / 2, ESC_L / 2, -ESC_W / 2, ESC_W / 2, Z_ESC, Z_ESC + ESC_T)
fc = box(-FC_HALF, FC_HALF, -FC_HALF, FC_HALF, Z_STACK_TOP - FC_T, Z_STACK_TOP)
stack = cat([esc, fc] + [cyl(2.5, Z_ESC + ESC_T, Z_STACK_TOP - FC_T, sx * STACK_P, sy * STACK_P, 12) for sx in (1, -1) for sy in (1, -1)])
cap = box(-ESC_L / 2 - CAP[2], -ESC_L / 2, -14, -4, Z_ESC, Z_ESC + CAP[1])
pi = cat([box(PI_C[0] - PI_L / 2, PI_C[0] + PI_L / 2, -PI_W / 2, PI_W / 2, Z_PI_COMP, Z_PI_BACK),
          box(PI_C[0] - PI_L / 2 - 2, PI_C[0] - PI_L / 2 + 21, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI_COMP - PI_UP, Z_PI_COMP),           # USB-A x4 + RJ45 at -x
          box(PI_C[0] - 8, PI_C[0] + 32, -21, 19, Z_PI_COMP - PI_COOL, Z_PI_COMP),                                                 # active cooler (E place)
          box(PI_C[0] - 26, PI_C[0] + 26, PI_W / 2 - 6, PI_W / 2 - 1, Z_PI_COMP - 8.5, Z_PI_COMP)])                               # GPIO header
ubec = box(UBEC_BAY[0] + 2, UBEC_BAY[0] + 50, -11, 11, UBEC_BAY[4], UBEC_BAY[4] + 9)        # a 48 x 22 x 9 UBEC (Q4 middle, E)
buck = box(-104, -84, -10, 10, BUCK_BAY[4], BUCK_BAY[4] + 8)                                 # 20 x 20 x 8 (E)
rx = box(-80.5, -74.5, -6, 6, RX_BAY[4], RX_BAY[4] + 16)                                     # XR2 16 x 12 x 6 (spec) on end
pi_plugs = box(PI_C[0] - PI_L / 2 - 2 - PI_PLUGS, PI_C[0] - PI_L / 2 - 2, -PI_W / 2 + 2, PI_W / 2 - 2, Z_PI_COMP - PI_UP, Z_PI_COMP)   # keep-out
flow = box(POD_X - 10.8, POD_X + 10.8, FLOW_Y - 8, FLOW_Y + 8, LENS_Z, LENS_Z + 6.5)          # MTF-02P (spec)


# ---------------------------------------------------------------- the core pieces (concept bodies for the piece runs and coupons)
def stack_mount():
    feet = [cyl(6, 0, PAD_T, x, y) for x, y in MOUNT_HOLES]
    posts = [cyl(4.5, 0, PAD_T, sx * STACK_P, sy * STACK_P) for sx in (1, -1) for sy in (1, -1)]
    bars = [bar((sx * 24, sy * 36.5), (sx * STACK_P, sy * STACK_P), 6, 0, PAD_T) for sx in (1, -1) for sy in (1, -1)]
    bars += [bar((sx * 24, 36.5), (sx * 24, -36.5), 6, 0, PAD_T) for sx in (1, -1)]
    return uni(feet + posts + bars)


def pi_hanger():
    pads = [cyl(6, Z_TOP_UNDER - HANGER_T, Z_TOP_UNDER, x, y) for x, y in TOP_HOLES]
    posts = [cyl(3.5, Z_PI_BACK, Z_TOP_UNDER, PI_C[0] + sx * PI_HX / 2, sy * PI_HY / 2) for sx in (1, -1) for sy in (1, -1)]
    bars = [bar((PI_C[0] - PI_HX / 2, sy * 24.5), (24.51, sy * 24.51), 6, Z_TOP_UNDER - HANGER_T, Z_TOP_UNDER) for sy in (1, -1)]
    bars += [bar((sx * 24.51, 24.51), (sx * 24.51, -24.51), 6, Z_TOP_UNDER - HANGER_T, Z_TOP_UNDER) for sx in (1, -1)]
    return uni(pads + posts + bars)


def rear_tray():
    feet = [cyl(6, 0, 3, x, y) for x, y in TRAY_HOLES]
    floor = box(-100, -62, -13.5, 13.5, 0, 3)
    legs = [box(x0, x0 + 6, sy * 13.5 - (6 if sy > 0 else 0), sy * 13.5 + (0 if sy > 0 else 6), 0, h) for x0, h in ((-100, BUCK_BAY[4]), (-68, UBEC_BAY[4])) for sy in (1, -1)]
    shelf1 = box(-100, -62, -13.5, 13.5, UBEC_BAY[4] - 2, UBEC_BAY[4])
    shelf2 = box(-106, -74, -13.5, 13.5, BUCK_BAY[4] - 2, BUCK_BAY[4])                    # ends at x -74: the Pi's plugs reach x -71.5
    arms_ = [bar((x, y), (-81, 0), 6, 0, 3) for x, y in TRAY_HOLES]
    return uni(feet + [floor, shelf1, shelf2] + legs + arms_)


def sled():
    plate = box(-115.0, 58.0, -56.0, 56.0, Z_M, Z_PLATE_BOT)
    ribs = [box(RIB_X[0], RIB_X[1], sy * 36, sy * 56, Z_M - 14, Z_M) for sy in (1, -1)]
    return uni([plate] + ribs)


def flow_pod():
    return uni([box(POD_X - 15, POD_X + 15, FLOW_Y - 11.5, FLOW_Y + 11.5, LENS_Z, Z_M - 14),
                box(RIB_X[0], RIB_X[1], -56, -44, Z_M - 18, Z_M - 14)])


MOUNT, HANGER, TRAY, SLED, POD = stack_mount(), pi_hanger(), rear_tray(), sled(), flow_pod()

# ---------------------------------------------------------------- wiring: drawn routes, each ending in a plug (clean wiring, L 8)
W = {
    "battery 12 AWG pair: receiver XT60 -> ESC battery pads (rear edge), cap on the pads": ([(-108.3, 0, xt60.bounds[1][2]), (-108.3, 0, 2), (-100, 0, 6), (-24, 0, 6)], 4.0),
    "motor leads x4: ESC corners -> along each arm to its motor (arm session routes them on the arm)": ([(20, 20, 7), (45, 45, 10)], 2.0),
    "UBEC in: ESC battery pads -> UBEC (rear tray)": ([(-24, 4, 8), (-60, 4, 12), (-70, 4, 14)], 1.5),
    "UBEC 5 V -> Pi GPIO 5 V pins (+y edge)": ([(-60, 8, 18), (-40, 20, 30), (PI_C[0] + 20, 24, Z_PI_COMP - 4)], 1.5),
    "buck in (ESC pads) and 12 V out -> nose (the L2's owner takes it there)": ([(-24, -4, 8), (-70, -8, 26), (-30, -10, 26), (30, -10, 26), (95, 0, 8)], 1.5),
    "FC <-> Pi UART (USB on the Pi), FC <-> RX (CRSF)": ([(0, -14, Z_STACK_TOP + 2), (-30, -16, 24), (-66, -6, 26)], 1.5),
    "FC UART -> flow (out at -y, down past the plate edge to the pod)": ([(10, -18, Z_STACK_TOP - 4), (5, -45, 10), (0, -61, 2), (-4, -61, Z_M - 16), (POD_X, FLOW_Y - 4, LENS_Z + 8)], 1.5),
}
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


BUDGET = {"stack mount": 8, "Pi hanger": 10, "rear tray": 20, "sled": 70, "flow pod": 8}   # g, soft slots (DEFAULT)
# floor contact: TPU feet on 4 opening ring nodes (cage session; placeholders at v5's ring nodes, contact 6 under the lenses)
RING = [(146.0, 43.2), (146.0, -43.2), (-102.9, 104.3), (-102.9, -104.3)]
Z_FOOT = LENS_Z - 6.0
feet = cat([cyl(7, Z_FOOT, Z_FOOT + 8, x, y) for x, y in RING])
STACK_NM = "FC stack (MicoAir743-Lite on 55A 4-in-1)"
core = [
    P(STACK_NM, stack, "bought", 24, protect=True),
    P("ESC cap", cap, "bought", 5, protect=True),
    P("Pi 5 + active cooler", cat([pi, pi_plugs]), "bought", 55, protect=True),
    P("UBEC 5 V", ubec, "bought", 15, protect=True),
    P("buck 12 V", buck, "bought", 6, protect=True),
    P("RX XR2", rx, "bought", 2, protect=True),
    P("flow MTF-02P", flow, "bought", 1.5, protect=True),
    P("wires", wires, "bought", 80),
    P("pack", pack, "bought", 650, protect=True, swap=True),
    P("cartridge", cartridge, "protected", 105.6, protect=True, swap=True, tag="M", owner="box swap (battery latch)", version="cartridge v6 + sliders v6"),
    P("receiver", receiver, "protected", 229.3, protect=True, tag="M", owner="box swap (battery latch)", version="receiver v2 + carrier v1 + cap + strips"),
    P("XT60 pair", xt60, "bought", 15, protect=True),
    P("stack mount", MOUNT, "structure", BUDGET["stack mount"], budget=BUDGET["stack mount"]),
    P("Pi hanger", HANGER, "structure", BUDGET["Pi hanger"], budget=BUDGET["Pi hanger"]),
    P("rear tray", TRAY, "structure", BUDGET["rear tray"], budget=BUDGET["rear tray"]),
    P("sled", SLED, "structure", BUDGET["sled"], budget=BUDGET["sled"]),
    P("flow pod", POD, "structure", BUDGET["flow pod"], budget=BUDGET["flow pod"]),
    P("TPU feet (cage session, placeholders)", feet, "cage", 6.4, material="TPU", owner="overseer cage", version="placeholder"),
]
CORE_NAMES = {p["name"] for p in core}
contact = [list(c) for c in v5["contact_pairs"] if not (set(c) & REPLACED)]
contact += [["stack mount", "bottom plate"], ["stack mount", STACK_NM], ["Pi hanger", "top plate"], ["Pi hanger", "Pi 5 + active cooler"],
            ["rear tray", "bottom plate"], ["rear tray", "UBEC 5 V"], ["rear tray", "buck 12 V"], ["rear tray", "RX XR2"], ["ESC cap", STACK_NM],
            ["sled", "bottom plate"], ["sled", "receiver"], ["sled", "XT60 pair"], ["flow pod", "sled"], ["flow pod", "flow MTF-02P"],
            ["pack", "cartridge"], ["pack", "receiver"], ["cartridge", "receiver"], ["receiver", "XT60 pair"], ["cartridge", "XT60 pair"], ["pack", "XT60 pair"]]
contact += [["wires", n] for n in sorted(CORE_NAMES | {"bottom plate", "top plate", "arm 1", "arm 2", "arm 3", "arm 4", "apex + cam bracket"}) if n != "wires"]
openings = [o for o in v5["openings"] if o.get("part") not in ("pack", "cartridge", "receiver", "flow MTF-02P")]
openings += [{"part": p_, "axis": [0, 0, -1], "half_angle_deg": 40.0, "skip_probes": True} for p_ in ("pack", "cartridge", "receiver", "XT60 pair", "flow MTF-02P")]

# access (CLAUDE.md 2): v5's arm screw keys (bottom keys through the sled's Ø9 holes), the stack's USB-C, the stack mount feet
# (L-key from the side), the Pi hanger screws (from above through the top plate), the tray bolts (from above, past the top plate), the pack swap
L_KEY = {"type": "cylinder", "d": 4}
acc = [a for a in v5["access"] if a["name"].startswith("M3")]
for a in acc:
    a["ignore"] = [i for i in a["ignore"] if i in ("arm 1", "arm 2", "arm 3", "arm 4", "prop 1", "prop 2", "prop 3", "prop 4", "down camera")] + (["sled"] if a["part"] == "bottom plate" else [])
acc += [{"name": "pack swap: cartridge + pack down through the opening", "part": "cartridge", "dir": [0, 0, -1], "length_mm": 150.0, "tool": "part", "ignore": ["pack", "receiver", "XT60 pair"]},
        {"name": "FC USB-C (-y edge, between arm 3 and arm 4 roots)", "part": STACK_NM, "from": [0, -FC_HALF, Z_STACK_TOP - 4],
         "dir": [0.091, -0.9947, 0.0475], "length_mm": 450.0, "tool": {"type": "box", "w": 12, "h": 7}, "ignore": ["wires"]}]
for k, (x, y) in enumerate(MOUNT_HOLES):          # stack mount feet: L-key, short leg down beside the stack, long leg out at ±y
    sy = 1 if y > 0 else -1
    acc.append({"name": f"M3 L-key short leg, stack mount foot {k}", "part": "stack mount", "from": [x, y, PAD_T + 0.5], "dir": [0, 0, 1], "length_mm": 18.0,
                "tool": L_KEY, "ignore": ["wires"]})
    acc.append({"name": f"M3 L-key long leg, stack mount foot {k}", "part": "stack mount", "from": [x, y, PAD_T + 18.5], "dir": [0, sy, 0], "length_mm": 400.0,
                "tool": L_KEY, "ignore": ["wires", "prop 1", "prop 2", "prop 3", "prop 4"]})
for k, (x, y) in enumerate(TOP_HOLES):            # Pi hanger screws from above through the top plate
    acc.append({"name": f"M3 key, Pi hanger screw {k} (from above)", "part": "Pi hanger", "from": [x, y, Z_TOP_UNDER + 1.7], "dir": [0, 0, 1], "length_mm": 400.0,
                "tool": {"type": "cylinder", "d": 6}, "ignore": ["top plate", "prop 1", "prop 2", "prop 3", "prop 4"]})
for k, (x, y) in enumerate(TRAY_HOLES):           # rear tray bolts from above (past the top plate)
    acc.append({"name": f"M3 key, rear tray bolt {k} (from above)", "part": "rear tray", "from": [x, y, 3.5], "dir": [0, 0, 1], "length_mm": 400.0,
                "tool": {"type": "cylinder", "d": 6}, "ignore": ["prop 1", "prop 2", "prop 3", "prop 4", "wires"]})

man = {k: v for k, v in v5.items() if k not in ("parts", "contact_pairs", "openings", "access", "note", "name")}
man.update({"name": "Overseer core v1", "machine": "overseer",
            "note": ("Overseer core v1 round 2 (27 Oct, Spark: everything inside the cage except the arms; latch mechanism, body nodes, cameras and "
                     "lidar not this session's): v5 / cage v3 context parts unchanged (nox/overseer/overseer.assembly.json); the core, the real latch "
                     "set (battery latch v4 Overseer meshes) and the feet placeholders on the cage's opening ring from nox/overseer/core_v1.py."),
            "parts": context + core, "contact_pairs": contact, "openings": openings, "access": acc})
MAN = os.path.join(HERE, "overseer_core.assembly.json")
with open(MAN, "w") as f:
    f.write("{\n" + ",\n".join(f' {json.dumps(k)}: ' + (("[\n" + ",\n".join("  " + json.dumps(p) for p in v) + "\n ]") if k == "parts" else json.dumps(v))
                               for k, v in man.items()) + "\n}\n")

# ---------------------------------------------------------------- checks of this pass (C on E sizes)
from topo.part import man as MAN_OF


def ov(a, b):
    return (MAN_OF(a) ^ MAN_OF(b)).volume()


say(f"stack: ESC z {Z_ESC}..{Z_ESC + ESC_T}, FC top z {Z_STACK_TOP}, plugs to {Z_STACK_TOP + PLUG} (17.5 stack, E model)")
low_cool = Z_PI_COMP - PI_COOL; low_port = Z_PI_COMP - PI_UP
say(f"Pi hung component side down: board z {Z_PI_COMP:.1f}..{Z_PI_BACK:.1f}; cooler down to {low_cool:.1f} ({low_cool - Z_STACK_TOP - PLUG:.1f} over the FC plugs); "
    f"USB-A / RJ45 down to {low_port:.1f} at x {PI_C[0] - PI_L / 2 - 2:.1f}..{PI_C[0] - PI_L / 2 + 21:.1f} (the stack ends at x {-ESC_L / 2:.1f}; "
    f"the cap at z <= {Z_ESC + CAP[1]:.1f}: {low_port - Z_ESC - CAP[1]:.1f} clear) (C on E)")
say(f"Pi -x plugs (keep-out {PI_PLUGS:.0f} past the ports) reach x {pi_plugs.bounds[0][0]:.1f}: x arms {sum(ov(pi_plugs, a) for a in ARMS):.1f}, x rear tray + bays {ov(pi_plugs, cat([TRAY, ubec, buck, rx])):.1f} mm³ (C on E)")
fits = []
for nm, m in (("stack mount", MOUNT), ("Pi hanger", HANGER), ("rear tray", TRAY), ("Pi 5", pi), ("stack", stack), ("cap", cap), ("UBEC", ubec), ("buck", buck), ("RX", rx)):
    fits.append(f"{nm} {sum(ov(m, arm) for arm in ARMS):.1f}")
say("   x arms (v5 meshes), mm³: " + ", ".join(fits))
say(f"   Pi x stack {ov(pi, stack):.1f}, Pi x cap {ov(pi, cap):.1f}, hanger x Pi {ov(HANGER, pi):.1f} (posts on the board's holes, C), stack x mount {ov(stack, MOUNT):.1f} mm³")
say(f"   sled x receiver {ov(SLED, receiver):.1f}, flow pod x receiver {ov(POD, receiver):.1f} mm³")
blk = []
for x, y in ARM_SCREWS:
    key = cyl(4.0, Z_FOOT - 30, Z_M - 0.5, x, y, 16)
    for nm, m in (("flow pod", POD), ("receiver", receiver), ("flow", flow), ("down camera (v5)", ctx_mesh["down camera"])):
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
say(f"swap column (receiver + 30 a side, below the cartridge bottom): flow pod {ov(col, POD):.1f}, flow {ov(col, flow):.1f}, feet {ov(col, feet):.1f} mm³")
# prop wash: plan area of the core's lower parts under the four disks (reported, L 8)
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union
disks = unary_union([Point(ctx_mesh[f"motor {i}"].centroid[0], ctx_mesh[f"motor {i}"].centroid[1]).buffer(127.0, 64) for i in (1, 2, 3, 4)])
plan = {nm: MultiPoint(m.vertices[:, :2]).convex_hull for nm, m in (("rear tray + bays", cat([TRAY, ubec, buck, rx])), ("sled", SLED), ("flow pod", POD), ("receiver", receiver))}
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
          "tray wires": (20, np.array([-80, 0, 12.0])), "swap set": (650 + 105.6 + 7.5, pack.centroid), "receiver set": (229.3 + 7.5, receiver.centroid),
          "flow": (1.5, flow.centroid), "down camera + its pod (E)": (15 + 12, np.array([POD_X, 55.0, -50.0]))}
for k_, m_ in (("stack mount", MOUNT), ("Pi hanger", HANGER), ("rear tray", TRAY), ("sled", SLED), ("flow pod", POD)):
    mass_c[k_] = (BUDGET[k_], m_.centroid)


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


SUP = {"stack mount": [(x, y, 0.0) for x, y in MOUNT_HOLES], "Pi hanger": [(x, y, Z_TOP_UNDER) for x, y in TOP_HOLES],
       "rear tray": [(x, y, 0.0) for x, y in TRAY_HOLES], "sled to plate": [(x, y, Z_PLATE_BOT) for x, y in MOUNT_HOLES + TRAY_HOLES],
       "receiver bolts": [(x, y, Z_M) for x, y in rec_bolts],
       "flow pod": [(-26.0, -40.0, Z_M - 14), (6.0, -40.0, Z_M - 14), (-10.0, -52.0, Z_M - 14)]}
CARRY = {"stack mount": ["stack", "stack mount"], "Pi hanger": ["Pi", "Pi hanger"], "rear tray": ["UBEC", "buck", "RX", "tray wires", "rear tray"],
         "receiver bolts": ["swap set", "receiver set"], "flow pod": ["flow", "flow pod"]}
cases = {f"crash 35 g {k}": 35 * d for k, d in enumerate(DIRS14)}
cases.update({"flight 4 g": np.array([0, 0, 4.0]), "flight 2 g x": np.array([2.0, 0, 1]), "flight 2 g y": np.array([0, 2.0, 1]), "landing 6 g": np.array([0, 0, 6.0])})
REQ = {k: {"worst_case": None, "max_bolt_N": 0.0, "max_shear_N": 0.0, "max_axial_N": 0.0} for k in SUP}


def rec(nm, case, f, axis=np.array([0, 0, 1.0])):
    ax = np.abs(f @ axis); sh = np.linalg.norm(f - np.outer(f @ axis, axis), axis=1); tot = np.linalg.norm(f, axis=1)
    r_ = REQ[nm]
    if tot.max() > r_["max_bolt_N"]: r_.update(worst_case=case, max_bolt_N=round(float(tot.max()), 1))
    r_["max_shear_N"] = round(max(r_["max_shear_N"], float(sh.max())), 1); r_["max_axial_N"] = round(max(r_["max_axial_N"], float(ax.max())), 1)


for case, a in cases.items():
    for nm in ("stack mount", "Pi hanger", "rear tray", "receiver bolts", "flow pod"):
        rec(nm, case, piece_case(SUP[nm], inertial(CARRY[nm], a)))
    rec("sled to plate", case, piece_case(SUP["sled to plate"], inertial(CARRY["receiver bolts"] + ["sled", "flow pod", "flow", "down camera + its pod (E)"], a)))
PRELOAD = 500.0                                     # M3 into a PETG insert, hand tight (E)
say("joint reactions (rigid body, least-norm, E masses; floor loads go to the cage feet, alternative B):")
for k, v in REQ.items():
    say(f"   {k}: max {v['max_bolt_N']} N per bolt ({v['worst_case']}), shear {v['max_shear_N']}, axial {v['max_axial_N']}")
say(f"   the sled's 8 bolts clamp the stack mount and rear tray feet: each foot carries {PRELOAD:.0f} N preload (E) + up to {REQ['sled to plate']['max_axial_N']} N")
Lp = (Z_M - 14) - LENS_Z
I_ = (30 * 23 ** 3 - 25 * 18 ** 3) / 12; mt = 1.5 + 0.24 * 2 * (30 + 23) * 2.5 * Lp * 1.27e-3
f1 = 1 / (2 * math.pi) * math.sqrt(3 * 2000 * I_ / Lp ** 3 / (mt * 1e-3) * 1e3)
say(f"flow pod: cantilever {Lp:.1f} under the sled rib, first mode ~{f1:.0f} Hz (E; target >= 100, L)")
cg = np.asarray(res["mass_cg"]["cg"], float)
tw = 4 * 1650 / M_TOT
say(f"mass {M_TOT:.0f} g (E), T/W {tw:.2f} at 1650 gf a motor; CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f})")
result = {"mass_g": M_TOT, "tw": round(tw, 2), "cg": cg.tolist(), "requirements": REQ, "pod_mode_hz": round(f1), "wash_pct": wash,
          "z": {"mount_plane": Z_M, "pack_top": float(pack.bounds[1][2]), "cartridge_bottom": float(cartridge.bounds[0][2]), "lens": LENS_Z,
                "esc": Z_ESC, "stack_top": Z_STACK_TOP, "pi_component_face": Z_PI_COMP, "pi_lowest": low_port, "foot": Z_FOOT},
          "receiver_bolts": rec_bolts, "mount_holes": MOUNT_HOLES, "tray_holes": TRAY_HOLES, "top_holes": TOP_HOLES,
          "arm_screws_driver_holes": ARM_SCREWS, "budget_g": BUDGET, "system": res["summary"]}
json.dump(result, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
