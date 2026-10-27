"""Overseer core v1: system pass for everything inside the cage except the arms (Spark 27 Oct: "design everything else left on
the newest overseer design inside the cage, excluding the arms, it needs to hold all components but ignore the actual
mechanism of the battery cartridge and pocket for now"; "also ignore all nodes on the body for now").

Context (not edited here): layout v5 / cage v3 (`nox/overseer/overseer.assembly.json`, `layout_v5.py`): S500 plates, arms, motors,
props, cage shell, sensor head (L2 + front camera + their mount). This script replaces v5's stand-ins for the hub
electronics, the belly sled, pods, feet and the latch set with the real parts in pose and the core pieces as envelopes:

  hub frame    PETG, between the plates: 4 columns at the bottom plate's Ø3.1 holes (±24, ±22.5) and the top plate's (±24.51,
               ±24.51), FC pads (30.5, grommets) at the plate, Pi 5 deck over the FC
  power tray   PETG, rear wing: FCHUB-12S on 30.5, UBEC + 12 V buck + RX bays over it
  sled         PETG, under the bottom plate: the receiver's mount plane (6 M3 inserts at the receiver v2 MOUNT points),
               clamped to the plate by the hub frame and power tray bolts, Ø9 driver holes under the 8 arm bottom screws
  skid L / R   PETG, from the sled edges: the down camera pod (L, +y) and the flow pod (R, -y) beside the receiver, lenses at
               the cartridge bottom, a TPU foot at each end, feet spanning the CG
  feet x4      TPU
The receiver, cartridge, pack and XT60 are battery latch v4's Overseer meshes in pose (mechanism not designed here).
No cage node or tie is placed on these pieces (Spark 27 Oct).

python3 nox/overseer/core_v1.py [out_dir]     (~2 min)
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
    """one closed body (manifold3d union) from overlapping envelope blocks"""
    from topo.part import man
    import manifold3d as mf_
    u = mf_.Manifold.batch_boolean([man(m) for m in ms], mf_.OpType.Add); mm = u.to_mesh()
    return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts))


# ---------------------------------------------------------------- numbers (tags in dimensions.md "System Overseer core")
DC = 30.0                          # catch / latch centre 30 aft of the plate centre (v2, C)
Z_PLATE_BOT = -2.1                 # bottom plate underside (S500 STEP, derived)
Z_TOP_UNDER = 45.0                 # top plate underside (arm root 45, DEFAULT)
SLED_T = 6.0                       # sled plate under the bottom plate (DEFAULT: receiver v2 asks a hole >= 6 deep under each M3 x 5.7 insert)
Z_M = Z_PLATE_BOT - SLED_T         # receiver mount plane (its deck top) -8.1
Y_M = 68.8                         # receiver v2 mount plane above the cartridge bottom (M, battery_latch r["Y_M"])
Z_CART = Z_M - Y_M                 # cartridge bottom -76.9
Z_T = 85.55                        # tab line (swap CG) in cartridge z (M); sits on the catch centre
BX = 27.8                          # receiver v2 bolt line x (M)
MOUNT_ZC = (-1.3, 85.55, 163.8)    # receiver v2 bolts along the cartridge (M)
# hub
PLATE_HOLES = [(sx * 24.0, sy * 22.5) for sx in (1, -1) for sy in (1, -1)]       # bottom plate Ø3.1 (derived, STEP)
TOP_HOLES = [(sx * 24.51, sy * 24.51) for sx in (1, -1) for sy in (1, -1)]       # top plate Ø3.1 (derived, STEP)
TRAY_HOLES = [(-65.0, 22.5), (-65.0, -22.5)]                                     # bottom plate Ø3.1, rear wing (derived)
FC_C, FC_HALF, FC_P = (0.0, 0.0), 18.0, 30.5 / 2                                 # MicoAir743-Lite 36 x 36 x 8, 30.5 (spec)
FC_Z0, FC_T, FC_PLUG = 4.0, 8.0, 7.0          # on 3.0 pads + 1.0 grommet (E); plugs and lead bends above it (E)
PI_C = (-9.0, 0.0)                 # Pi 5 centre: 9 aft so the USB-A / Ethernet plugs at +x clear the front roots (C); powered on its
                                   # GPIO 5 V pins from the UBEC (DEFAULT: its USB-C edge faces arm 3's root; alt right-angle USB-C)
PI_L, PI_W, PI_HOLES = 85.0, 56.0, (58.0, 49.0)                                  # Pi 5 board and M2.5 pattern (spec)
DECK_Z0, DECK_T, PI_POST = 20.0, 3.0, 2.5     # deck over the FC plugs; M2.5 posts (DEFAULT)
PI_BOARD_T, PI_UP, PI_COOL = 1.6, 16.0, 9.0   # board, tallest port above it (USB-A), active cooler (E)
# power tray (rear wing)
PDB_C, PDB_L, PDB_W = (-74.0, 0.0), 50.0, 50.0   # FCHUB-12S 55 x 50 x 6 (spec), 55 along y here
TRAY_Z0, PDB_Z0 = 0.0, 7.0                     # tray floor 3 on the plate, PDB on 4 mm nylon standoffs (DEFAULT)
# belly
POD_X = -10.0                      # pods 20 ahead of v2's place: every arm bottom screw key line clear (DEFAULT; alt v2's -30 + unplug a board)
CAM_Y, FLOW_Y = 55.0, -55.0        # down camera board 38 x 38 (P), flow 21.6 x 16 (spec), beside the receiver (±33.4, M)
LENS_Z = Z_CART                    # lenses on the cartridge bottom plane: the receiver (0.2 above) stays out of both views, pods out of the swap column (C)
SKID_X = (-48.0, 112.0)            # skid ends = feet x: the CG (thrust centre x 60.8) inside the footprint (C)
FOOT_Y, FOOT_R, FOOT_H = 76.0, 7.0, 8.0        # feet outside the receiver + 30 column (|y| > 68.1 + 0.9) (C); TPU domes Ø14 (DEFAULT)
Z_FOOT = Z_CART - 9.0              # floor contact 9 under the cartridge bottom (DEFAULT: lenses 8.4 off the floor)
ARM_SCREWS = [(sx * a, sy * b) for sx in (1, -1) for sy in (1, -1) for a, b in ((52.14, 38.0), (38.0, 52.14))]   # bottom plate (derived)

# ---------------------------------------------------------------- context: v5 manifest
v5 = json.load(open(V5))
REPLACED = {"hub electronics (FC, Pi 5 + deck, PDB, UBEC, buck, RX)", "wires", "pack", "cartridge", "receiver", "belly sled + pods",
            "down camera", "flow MTF-02P", "TPU feet"}
context = [dict(p) for p in v5["parts"] if p["name"] not in REPLACED]
for p in context:
    p["mesh"] = os.path.relpath(os.path.join(ROOT, p["mesh"]), ROOT)
ctx_mesh = {p["name"]: trimesh.load(os.path.join(ROOT, p["mesh"])) for p in context}
ARMS = [ctx_mesh[f"arm {i}"] for i in (1, 2, 3, 4)]

# ---------------------------------------------------------------- latch set (battery latch v4, Overseer) in pose
import manifold3d as mf
with contextlib.redirect_stdout(io.StringIO()):
    import battery_latch as BL
    LA = BL.latch_assembly("overseer", "latched")
T_LATCH = np.array([[0, 0, -1, DC * -1 + Z_T], [-1, 0, 0, 0], [0, 1, 0, Z_M - Y_M], [0, 0, 0, 1.0]])   # zc -> -x (XT60 end aft)


def m2t(m):
    mm = m.to_mesh(); t = trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False)
    t.apply_transform(T_LATCH); return t


receiver = cat([m2t(LA[k]) for k in ("receiver", "receiver_lid", "receiver_strips", "xt60_carrier")])
cartridge = cat([m2t(LA[k]) for k in ("cartridge", "slider_px", "slider_nx")])
pack = m2t(LA["pack"]); xt60 = cat([m2t(LA["xt60_plug"]), m2t(LA["xt60e1m"])])
RB = receiver.bounds; say(f"receiver in pose x {RB[0][0]:.1f}..{RB[1][0]:.1f}, y ±{RB[1][1]:.1f}, z {RB[0][2]:.1f}..{RB[1][2]:.1f} (M); "
                          f"pack top z {pack.bounds[1][2]:.1f}, cartridge bottom z {cartridge.bounds[0][2]:.1f}; XT60 male top z {xt60.bounds[1][2]:.1f}")
rec_bolts = [(DC * -1 + Z_T - zc, sy * BX) for zc in MOUNT_ZC for sy in (1, -1)]

# ---------------------------------------------------------------- bought parts in the hub
fc = box(-FC_HALF, FC_HALF, -FC_HALF, FC_HALF, FC_Z0, FC_Z0 + FC_T)
fc_plugs = box(-FC_HALF, FC_HALF, -FC_HALF, FC_HALF, FC_Z0 + FC_T, FC_Z0 + FC_T + FC_PLUG)      # keep-out, not a part
pi_z0 = DECK_Z0 + DECK_T + PI_POST
pi = cat([box(PI_C[0] - PI_L / 2, PI_C[0] + PI_L / 2, -PI_W / 2, PI_W / 2, pi_z0, pi_z0 + PI_BOARD_T),
          box(PI_C[0] + PI_L / 2 - 21, PI_C[0] + PI_L / 2 + 2, -PI_W / 2 + 2, PI_W / 2 - 2, pi_z0 + PI_BOARD_T, pi_z0 + PI_BOARD_T + PI_UP),  # USB-A x4 + Ethernet at +x
          box(PI_C[0] - 22, PI_C[0] + 18, -20, 20, pi_z0 + PI_BOARD_T, pi_z0 + PI_BOARD_T + PI_COOL)])                                   # active cooler
pi_top = pi.bounds[1][2]
pdb = box(PDB_C[0] - 27.5, PDB_C[0] + 27.5, -25, 25, PDB_Z0, PDB_Z0 + 6)
ubec = box(-98, -52, 2, 22, PDB_Z0 + 8, PDB_Z0 + 17)           # 5 V 5 A UBEC 46 x 20 x 9 (Q4 envelope, E)
buck = box(-82, -64, -22, -4, PDB_Z0 + 8, PDB_Z0 + 14)         # 12 V buck for the L2 (E)
rx = box(-98, -86, -20, -8, PDB_Z0 + 8, PDB_Z0 + 14)           # RadioMaster XR2 16 x 12 x 6 (spec) on its side bay
cam_dn = cat([box(POD_X - 19, POD_X + 19, CAM_Y - 19, CAM_Y + 19, LENS_Z + 9, LENS_Z + 11),                   # board 38 x 38 (P), thickness E
              cyl(7, LENS_Z, LENS_Z + 9, POD_X, CAM_Y)])                                                      # lens barrel Ø14 x 9 (Q3, E)
flow = box(POD_X - 10.8, POD_X + 10.8, FLOW_Y - 8, FLOW_Y + 8, LENS_Z, LENS_Z + 6.5)                         # MTF-02P 21.6 x 16 x 6.5 (spec)

# ---------------------------------------------------------------- the core pieces (envelopes of the piece runs)
def hub_frame():
    cols = [cat([cyl(6, 0, 3, x, y), cyl(4, 0, Z_TOP_UNDER, x + (tx - x) * 0.5, y + (ty - y) * 0.5), cyl(6, Z_TOP_UNDER - 3, Z_TOP_UNDER, tx, ty)])
            for (x, y), (tx, ty) in zip(PLATE_HOLES, TOP_HOLES)]
    pads = [cyl(4, 0, FC_Z0 - 1, FC_C[0] + sx * FC_P, FC_C[1] + sy * FC_P) for sx in (1, -1) for sy in (1, -1)]
    ring = [box(sx * 24 - 3, sx * 24 + 3, -24, 24, 0, 3) for sx in (1, -1)] + [box(sx * 9, sx * 24, sy * 15.25 - 3, sy * 15.25 + 3, 0, 3) for sx in (1, -1) for sy in (1, -1)]   # x side bars only: the FC USB-C and leads leave at ±y
    deck = box(PI_C[0] - 31, PI_C[0] + 31, -27.5, 27.5, DECK_Z0, DECK_Z0 + DECK_T)
    posts = [cyl(3, DECK_Z0 + DECK_T, pi_z0, PI_C[0] + sx * PI_HOLES[0] / 2, sy * PI_HOLES[1] / 2) for sx in (1, -1) for sy in (1, -1)]
    return uni(cols + pads + ring + [deck] + posts)


def power_tray():
    floor = box(-97, -52, -22, 22, TRAY_Z0, TRAY_Z0 + 3)
    pads = [cyl(4, 3, PDB_Z0, PDB_C[0] + sx * 15.25, PDB_C[1] + sy * 15.25) for sx in (1, -1) for sy in (1, -1)]
    cols = [cyl(4, 0, PDB_Z0 + 8, x, y) for x, y in ((-95, 20), (-95, -20), (-55, 20), (-55, -20))]
    shelf = box(-99, -52, -23, 23, PDB_Z0 + 6.5, PDB_Z0 + 8)
    return uni([floor, shelf] + pads + cols)


SKID_JOINTS = ((-32.0, 12.0), (-110.0, -62.0))   # x spans of the sled-skid joints: the pod and the rear end, off every arm screw key line (C)


def sled():
    plate = box(-115.0, 58.0, -56.0, 56.0, Z_M, Z_PLATE_BOT)
    ribs = [box(x0, x1, sy * 36, sy * 56, Z_M - 14, Z_M) for x0, x1 in SKID_JOINTS for sy in (1, -1)]   # side ribs beside the receiver at the skid joints
    return uni([plate] + ribs)


def skid(side):
    """mirror pair: the pod envelope is the same on both sides (camera 38 x 38 on L, flow 21.6 x 16 on R) (DEFAULT, symmetry)"""
    sy = 1 if side > 0 else -1
    yc = CAM_Y * sy
    rail = box(SKID_X[0] - 7, SKID_X[1] + 7, sy * 64, sy * (FOOT_Y + 7), Z_CART, Z_M - 14)        # above the cartridge bottom: out of the swap column
    legs = [box(x - 7, x + 7, sy * (FOOT_Y - 7), sy * (FOOT_Y + 7), Z_FOOT + FOOT_H, Z_M - 14) for x in SKID_X]
    pod = box(POD_X - 21.5, POD_X + 21.5, yc - 21.5, yc + 21.5, LENS_Z, Z_M - 14)
    joints = [box(x0, x1, sy * 36, sy * 66, Z_M - 20, Z_M - 14) for x0, x1 in SKID_JOINTS]
    return uni([rail, pod] + legs + joints)


def feet():
    return [cyl(FOOT_R, Z_FOOT, Z_FOOT + FOOT_H, x, sy * FOOT_Y) for x in SKID_X for sy in (1, -1)]


HUB, TRAY, SLED = hub_frame(), power_tray(), sled()
SKID_L, SKID_R, FEET = skid(1), skid(-1), feet()

# ---------------------------------------------------------------- wiring: drawn routes, each with its connector (clean wiring, L 8)
W = {
    "main power 12 AWG pair: receiver XT60 -> PDB battery tab": ([(-108.3, 0, xt60.bounds[1][2]), (-108.3, 0, 4), (-99, 0, 10), (-92, 0, PDB_Z0 + 3)], 4.0, 0.0),
    "ESC leads x4: PDB corners -> arm roots (ESCs on the arms, arm session)": ([(-50, 22, PDB_Z0 + 3), (-40, 34, 6)], 2.5, 0.0),
    "FC <-> Pi UART + FC <-> RX + FC <-> ESC signal": ([(0, -14, FC_Z0 + FC_T + 2), (-30, -14, 14), (-86, -14, PDB_Z0 + 11)], 1.5, 0.0),
    "UBEC 5 V -> Pi GPIO 5 V pins (+y edge)": ([(-60, 12, PDB_Z0 + 12), (-45, 20, 24), (PI_C[0] - 20, PI_W / 2 - 3, pi_z0 + 4)], 1.5, 0.0),
    "buck 12 V + Ethernet -> L2 (nose, sensor head)": ([(-70, -14, PDB_Z0 + 11), (-30, 0, 16), (40, 0, 16), (95, 0, 8)], 2.5, 0.0),
    "Pi USB -> front camera (nose)": ([(PI_C[0] + 43, 10, pi_z0 + 8), (60, 10, 30), (95, 10, 10)], 2.0, 0.0),
    "Pi USB -> down camera (out at +y, down the skid L to its pod)": ([(PI_C[0] + 43, 20, pi_z0 + 8), (10, 50, 30), (0, 62, 2), (0, 62, Z_M - 16), (POD_X + 6, CAM_Y + 6, LENS_Z + 13)], 2.0, 0.0),
    "FC UART -> flow (out at -y, down the skid R to its pod)": ([(10, -18, FC_Z0 + 4), (5, -45, 10), (0, -62, 2), (0, -62, Z_M - 16), (POD_X, FLOW_Y - 4, LENS_Z + 8)], 1.5, 0.0),
}
wires = cat([tube(p, r) for p, r, _ in W.values()])

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


BUDGET = {"hub frame": 30, "power tray": 18, "sled": 70, "skid L (down camera pod)": 45, "skid R (flow pod)": 45}   # g, soft slots (DEFAULT)


def g_of(m, fill):
    return m.volume / 1000 * 1.27 * fill


# printed piece masses: E at the envelope's fill until the piece runs (budget_g = the slot)
core = [
    P("FC MicoAir743-Lite", fc, "bought", 10, protect=True, tag="P"),
    P("Pi 5 + active cooler", pi, "bought", 55, protect=True),
    P("PDB FCHUB-12S", pdb, "bought", 21, protect=True, tag="P"),
    P("UBEC 5 V", ubec, "bought", 15, protect=True),
    P("buck 12 V", buck, "bought", 6, protect=True),
    P("RX XR2", rx, "bought", 2, protect=True),
    P("down camera", cam_dn, "bought", 15, protect=True),
    P("flow MTF-02P", flow, "bought", 2, protect=True, tag="P"),
    P("wires", wires, "bought", 80),
    P("pack", pack, "bought", 650, protect=True, swap=True),
    P("cartridge", cartridge, "protected", 97.6 + 8, protect=True, swap=True, tag="M", owner="box swap (battery latch)", version="cartridge v6 + sliders v6"),
    P("receiver", receiver, "protected", 216.2 + 13.1, protect=True, tag="M", owner="box swap (battery latch)", version="receiver v2 + carrier v1 + cap + strips"),
    P("XT60 pair", xt60, "bought", 15, protect=True),
    P("hub frame", HUB, "structure", BUDGET["hub frame"], budget=BUDGET["hub frame"]),
    P("power tray", TRAY, "structure", BUDGET["power tray"], budget=BUDGET["power tray"]),
    P("sled", SLED, "structure", BUDGET["sled"], budget=BUDGET["sled"]),
    P("skid L (down camera pod)", SKID_L, "structure", BUDGET["skid L (down camera pod)"], budget=BUDGET["skid L (down camera pod)"]),
    P("skid R (flow pod)", SKID_R, "structure", BUDGET["skid R (flow pod)"], budget=BUDGET["skid R (flow pod)"]),
    P("TPU feet", cat(FEET), "cage", 4 * 1.6, material="TPU"),
]
contact = [list(c) for c in v5["contact_pairs"] if not (set(c) & REPLACED)]
contact += [["hub frame", "bottom plate"], ["hub frame", "top plate"], ["hub frame", "FC MicoAir743-Lite"], ["hub frame", "Pi 5 + active cooler"],
            ["power tray", "bottom plate"], ["power tray", "PDB FCHUB-12S"], ["power tray", "UBEC 5 V"], ["power tray", "buck 12 V"], ["power tray", "RX XR2"],
            ["sled", "bottom plate"], ["sled", "receiver"], ["sled", "XT60 pair"], ["skid L (down camera pod)", "sled"], ["skid R (flow pod)", "sled"],
            ["skid L (down camera pod)", "down camera"], ["skid R (flow pod)", "flow MTF-02P"], ["TPU feet", "skid L (down camera pod)"], ["TPU feet", "skid R (flow pod)"],
            ["pack", "cartridge"], ["pack", "receiver"], ["cartridge", "receiver"], ["receiver", "XT60 pair"], ["cartridge", "XT60 pair"], ["pack", "XT60 pair"]]
contact += [["wires", n] for n in ("FC MicoAir743-Lite", "Pi 5 + active cooler", "PDB FCHUB-12S", "UBEC 5 V", "buck 12 V", "RX XR2", "down camera", "flow MTF-02P",
                                   "XT60 pair", "hub frame", "power tray", "sled", "skid L (down camera pod)", "skid R (flow pod)", "bottom plate", "receiver",
                                   "arm 1", "arm 2", "arm 3", "arm 4", "apex + cam bracket", "top plate")]
openings = [o for o in v5["openings"] if o.get("part") not in ("pack", "cartridge", "receiver", "down camera", "flow MTF-02P")]
openings += [{"part": p_, "axis": [0, 0, -1], "half_angle_deg": 40.0, "skip_probes": True} for p_ in ("pack", "cartridge", "receiver", "XT60 pair", "down camera", "flow MTF-02P")]

# access: v5's arm screw keys (bottom keys now through the sled's Ø9 holes), FC / Pi USB-C, the pack swap, the XT60 lead
acc = [a for a in v5["access"] if a["name"].startswith("M3")]
for a in acc:
    a["ignore"] = [i for i in a["ignore"] if i in ("arm 1", "arm 2", "arm 3", "arm 4", "prop 1", "prop 2", "prop 3", "prop 4")] + (["sled"] if a["part"] == "bottom plate" else [])
acc += [{"name": "pack swap: cartridge + pack down through the opening", "part": "cartridge", "dir": [0, 0, -1], "length_mm": 150.0, "tool": "part", "ignore": ["pack", "receiver", "XT60 pair"]},
        {"name": "FC USB-C (-y, between arm 3 and arm 4 roots)", "part": "FC MicoAir743-Lite", "from": [0, -18.0, FC_Z0 + 4], "dir": [0.091, -0.9947, 0.0475], "length_mm": 450.0,
         "tool": {"type": "box", "w": 12, "h": 7}, "ignore": ["wires"]},
        {"name": "Pi 5 microSD (-x)", "part": "Pi 5 + active cooler", "from": [PI_C[0] - PI_L / 2, 0, pi_z0 - 1.5], "dir": [-0.96, 0, 0.28], "length_mm": 60.0,
         "tool": {"type": "box", "w": 14, "h": 3}, "ignore": ["wires", "hub frame"]}]
# sled driver holes (Ø9) are cut in the piece; in this envelope model the bottom keys ignore the sled and must clear the skids, pods and receiver

man = {k: v for k, v in v5.items() if k not in ("parts", "contact_pairs", "openings", "access", "note", "name")}
man.update({"name": "Overseer core v1", "machine": "overseer",
            "note": ("Overseer core v1 (27 Oct, Spark: everything inside the cage except the arms; latch mechanism and body nodes ignored): "
                     "v5 / cage v3 context parts unchanged (nox/overseer/overseer.assembly.json); hub electronics, belly and the real latch set "
                     "(battery latch v4 Overseer meshes) in pose from nox/overseer/core_v1.py; core pieces as envelopes until their runs."),
            "parts": context + core, "contact_pairs": contact, "openings": openings, "access": acc})
MAN = os.path.join(HERE, "overseer_core.assembly.json")
with open(MAN, "w") as f:
    f.write("{\n" + ",\n".join(f' {json.dumps(k)}: ' + (("[\n" + ",\n".join("  " + json.dumps(p) for p in v) + "\n ]") if k == "parts" else json.dumps(v))
                               for k, v in man.items()) + "\n}\n")

# ---------------------------------------------------------------- checks of this pass (C on E sizes)
from topo.part import man as MAN_OF


def ov(a, b):
    return (MAN_OF(a) ^ MAN_OF(b)).volume()


say(f"hub: FC z {FC_Z0}..{FC_Z0 + FC_T} on 30.5 grommet pads, plugs to {FC_Z0 + FC_T + FC_PLUG}; deck z {DECK_Z0}..{DECK_Z0 + DECK_T}; "
    f"Pi board z {pi_z0:.1f}, top {pi_top:.1f}, {Z_TOP_UNDER - pi_top:.1f} under the top plate, cooler fan {Z_TOP_UNDER - (pi_z0 + PI_BOARD_T + PI_COOL):.1f} under it (C on E)")
pl = PI_C[0] + PI_L / 2 + 2 + 18                       # +x port face + a straight plug 18 (E)
say(f"Pi +x plugs reach x {pl:.1f}: half sizes {pl:.1f} + {PI_W / 2:.0f} = {pl + PI_W / 2:.1f} (roots allow 82, dimensions O Arms and hub)")
for nm, m in (("hub frame", HUB), ("power tray", TRAY), ("Pi 5", pi), ("PDB", pdb), ("UBEC", ubec), ("buck", buck), ("RX", rx), ("FC", fc)):
    a = sum(ov(m, arm) for arm in ARMS)
    say(f"   {nm} x arms (v5 meshes): {a:.1f} mm³")
for nm, m in (("sled", SLED), ("skid L", SKID_L), ("skid R", SKID_R)):
    say(f"   {nm} x receiver: {ov(m, receiver):.1f} mm³; x cartridge {ov(m, cartridge):.1f}")
# key lines of the 8 arm bottom screws (Ø8 driver, down from the plate) against the belly below the sled
blk = []
for x, y in ARM_SCREWS:
    key = cyl(4.0, Z_FOOT - 30, Z_M - 0.5, x, y, 16)
    for nm, m in (("skid L", SKID_L), ("skid R", SKID_R), ("receiver", receiver), ("down camera", cam_dn), ("flow", flow)):
        v = ov(key, m)
        if v > 0.5: blk.append(f"({x:.0f},{y:.0f}) {nm} {v:.0f}")
say("arm bottom screw keys (Ø8 below the sled): " + (", ".join(blk) if blk else "all 8 clear (C); the sled gets a Ø9 hole at each"))
# views: down camera and flow straight down, nothing of the drone inside (102° diagonal: 44.5 x 36.5 half, +2; flow 42° FOV)
def view_clear(lens, half_x, half_y, meshes):
    hits = []
    for nm, m in meshes:
        p = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 4000, seed=1)[0]])
        d = p - lens; below = d[:, 2] < -0.5
        ax = np.degrees(np.arctan2(np.abs(d[:, 0]), -d[:, 2])); ay = np.degrees(np.arctan2(np.abs(d[:, 1]), -d[:, 2]))
        n = int((below & (ax < half_x) & (ay < half_y)).sum())
        if n: hits.append(f"{nm} {n}")
    return hits


others = [("receiver", receiver), ("cartridge", cartridge), ("skid L", SKID_L), ("skid R", SKID_R), ("feet", cat(FEET))]
h1 = view_clear(np.array([POD_X, CAM_Y, LENS_Z]), 36.5 + 2, 44.5 + 2, others)
h2 = view_clear(np.array([POD_X, FLOW_Y, LENS_Z]), 21 + 2, 21 + 2, others)
say("down camera view (portrait 73 x 89°, long side across, +2°): " + ("clear (C)" if not h1 else "blocked: " + ", ".join(h1)))
say("flow view (42°, +2°): " + ("clear (C)" if not h2 else "blocked: " + ", ".join(h2)))
col = box(-DC - 179.6 / 2 - 30, -DC + 179.6 / 2 + 30, -76.2 / 2 - 30, 76.2 / 2 + 30, Z_FOOT - 50, Z_CART)
say("swap column (receiver + 30 a side, below the cartridge bottom): " +
    ", ".join(f"{nm} {ov(col, m):.1f} mm³" for nm, m in (("skid L", SKID_L), ("skid R", SKID_R), ("feet", cat(FEET)), ("down camera", cam_dn), ("flow", flow))))

# ---------------------------------------------------------------- mass, CG, tip-over on the feet
from system_check import run as SC_RUN
res = SC_RUN(MAN)
say(res["summary"])
rows = {r_["name"]: r_ for r_ in res.get("mass", {}).get("rows", [])} if isinstance(res.get("mass"), dict) else {}
json.dump(res, open(os.path.join(OUT, "system_check.json"), "w"), indent=1, default=str)

# ---------------------------------------------------------------- joint reactions per piece (rigid body, least-norm bolt forces, E)
G = 9.81e-3                                         # N per g of mass per g of acceleration
DIRS14 = [np.array(v, float) / np.linalg.norm(v) for v in
          [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] + [(a, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)]]
M_TOT = float(res["mass_cg"]["mass_g"])
mass_c = {"Pi": (55, pi.centroid), "FC": (10, fc.centroid), "PDB": (21, pdb.centroid), "UBEC": (15, ubec.centroid), "buck": (6, buck.centroid),
          "RX": (2, rx.centroid), "tray wires": (30, np.array([-75, 0, 10.0])),
          "swap set (pack + cartridge + XT60 plug)": (650 + 105.6 + 7.5, pack.centroid), "receiver set": (229.3 + 7.5, receiver.centroid),
          "down camera": (15, cam_dn.centroid), "flow": (2, flow.centroid)}
for k_, m_ in (("hub frame", HUB), ("power tray", TRAY), ("sled", SLED), ("skid L", SKID_L), ("skid R", SKID_R)):
    mass_c[k_] = (BUDGET[[b for b in BUDGET if b.startswith(k_)][0]], m_.centroid)


def bolt_forces(pts_, F, Mc, c):
    """least-norm forces f_i at the bolt points: sum f = F, sum (p_i - c) x f_i = Mc (moments about c)"""
    P_ = np.asarray(pts_, float) - c; n = len(P_); A_ = np.zeros((6, 3 * n))
    for i, r in enumerate(P_):
        A_[0:3, 3 * i:3 * i + 3] = np.eye(3)
        A_[3:6, 3 * i:3 * i + 3] = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    f = np.linalg.pinv(A_) @ np.concatenate([F, Mc]); return f.reshape(n, 3)


def piece_case(supports, loads):
    """loads: [(force N, point)] -> bolt forces holding the piece (reaction on the piece = -sum)"""
    c = np.mean(np.asarray(supports, float), 0)
    F = sum(l[0] for l in loads); Mc = sum(np.cross(np.asarray(l[1]) - c, l[0]) for l in loads)
    return bolt_forces(supports, -F, -Mc, c)


def inertial(names, acc):
    """inertial loads (N) of the named masses for a body acceleration acc (g): -m a"""
    return [(-mass_c[n][0] * G * acc, mass_c[n][1]) for n in names]


SUP = {
    "hub frame": [(x, y, 0.0) for x, y in PLATE_HOLES] + [(x, y, Z_TOP_UNDER) for x, y in TOP_HOLES],
    "power tray": [(x, y, 0.0) for x, y in TRAY_HOLES] + [(-92.5, 25.75, 0.0), (-92.5, -25.75, 0.0)],
    "sled": [(x, y, Z_PLATE_BOT) for x, y in PLATE_HOLES + TRAY_HOLES] + [(-92.5, 25.75, Z_PLATE_BOT), (-92.5, -25.75, Z_PLATE_BOT)],
    "skid": [(x, s_ * 46.0, Z_M - 17) for x in (-28.0, 8.0, -106.0, -66.0) for s_ in (1,)],
}
REC_B = [(x, y, Z_M) for x, y in rec_bolts]
CARRY = {"hub frame": ["Pi", "FC", "hub frame"], "power tray": ["PDB", "UBEC", "buck", "RX", "tray wires", "power tray"],
         "receiver": ["swap set (pack + cartridge + XT60 plug)", "receiver set"], "skid L": ["down camera", "skid L"]}
cases = {}
for k, d in enumerate(DIRS14):
    cases[f"crash 35 g {k}"] = 35 * d
cases.update({"flight 4 g": np.array([0, 0, 4.0]), "flight 2 g x": np.array([2.0, 0, 1]), "flight 2 g y": np.array([0, 2.0, 1]),
              "landing 6 g": np.array([0, 0, 6.0]), "hover": np.array([0, 0, 1.0])})
# body acceleration a (g): the drone accelerates along a; every carried mass loads its piece with -m a (gravity folded in as 1 g up in hover)
feet_pts = [(x, s_ * FOOT_Y, Z_FOOT) for x in SKID_X for s_ in (1, -1)]
REQ = {}
for nm in ("hub frame", "power tray", "receiver bolts (into the sled)", "sled to plate", "skid joints (one skid)"):
    REQ[nm] = {"worst_case": None, "max_bolt_N": 0.0, "max_shear_N": 0.0, "max_axial_N": 0.0}


def rec(nm, case, f, axis=np.array([0, 0, 1.0])):
    ax = np.abs(f @ axis); sh = np.linalg.norm(f - np.outer(f @ axis, axis), axis=1); tot = np.linalg.norm(f, axis=1)
    r_ = REQ[nm]
    if tot.max() > r_["max_bolt_N"]: r_.update(worst_case=case, max_bolt_N=round(float(tot.max()), 1))
    r_["max_shear_N"] = round(max(r_["max_shear_N"], float(sh.max())), 1); r_["max_axial_N"] = round(max(r_["max_axial_N"], float(ax.max())), 1)


for case, a in cases.items():
    rec("hub frame", case, piece_case(SUP["hub frame"], inertial(CARRY["hub frame"], a)))
    rec("power tray", case, piece_case(SUP["power tray"], inertial(CARRY["power tray"], a)))
    rl = inertial(CARRY["receiver"], a)
    rec("receiver bolts (into the sled)", case, piece_case(REC_B, rl))
    ext = []
    if case in ("landing 6 g",) or (case.startswith("crash") and a[2] > 0 and abs(a[0]) < 1e-6 and abs(a[1]) < 1e-6):
        # floor contact through the feet: the whole drone's deceleration enters at the 4 feet, worst on 2 (one side) (E)
        for p_ in feet_pts[:2]:
            ext.append((np.array([0, 0, M_TOT * G * a[2] / 2]), np.array(p_)))
    sk = inertial(CARRY["skid L"], a) + [e for e in ext if e[1][1] > 0]
    sup_sk = [(x, 46.0, Z_M - 17) for x in (-28.0, 8.0, -106.0, -66.0)]
    rec("skid joints (one skid)", case, piece_case(sup_sk, sk))
    sled_loads = rl + inertial(["sled", "skid L", "skid R", "down camera", "flow"], a) + ext
    rec("sled to plate", case, piece_case(SUP["sled"], sled_loads))
say("joint reactions (rigid body, least-norm, E masses; landing / -Z crash through 2 feet on one side):")
for k, v in REQ.items():
    say(f"   {k}: max {v['max_bolt_N']} N per bolt ({v['worst_case']}), shear {v['max_shear_N']}, axial {v['max_axial_N']}")
# pod first mode (cantilever from the sled ribs: pod wall box 43 x 43, wall 2.5, length to the lens, PETG 2.0 GPa, camera at the tip) (E)
Lp = (Z_M - 14) - LENS_Z
I_ = (43 ** 4 - 38 ** 4) / 12; mt = 15 + 0.24 * 43 * 4 * 2.5 * Lp * 1.27e-3
f1 = 1 / (2 * math.pi) * math.sqrt(3 * 2000 * I_ / Lp ** 3 / (mt * 1e-3) * 1e3)
say(f"down camera pod: cantilever {Lp:.1f} under the sled ribs, first mode ~{f1:.0f} Hz (E; target >= 100, L)")
cg = np.asarray(res["mass_cg"]["cg"], float)
fx = (SKID_X[1] - cg[0], cg[0] - SKID_X[0]); h_cg = cg[2] - Z_FOOT
tip = {"forward": math.degrees(math.atan2(fx[0], h_cg)), "back": math.degrees(math.atan2(fx[1], h_cg)), "side": math.degrees(math.atan2(FOOT_Y - abs(cg[1]), h_cg))}
say(f"on the floor: CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.1f}) {h_cg:.0f} over the feet; feet x {SKID_X[0]:.0f}..{SKID_X[1]:.0f}, y ±{FOOT_Y:.0f}: "
    f"tip-over forward {tip['forward']:.0f}°, back {tip['back']:.0f}°, side {tip['side']:.0f}° (C on E)")
ext = res["compact"]
result = {"mass_g": M_TOT, "cg": cg.tolist(), "requirements": REQ, "tip_deg": tip, "pod_mode_hz": round(f1),
          "z": {"mount_plane": Z_M, "pack_top": float(pack.bounds[1][2]), "cartridge_bottom": float(cartridge.bounds[0][2]), "lens": LENS_Z, "floor": Z_FOOT},
          "receiver_bolts": rec_bolts, "plate_bolts": PLATE_HOLES + TRAY_HOLES + [(-92.5, 25.75), (-92.5, -25.75)], "top_bolts": TOP_HOLES,
          "arm_screws_driver_holes": ARM_SCREWS, "budget_g": BUDGET, "system": res["summary"]}
json.dump(result, open(os.path.join(OUT, "result.json"), "w"), indent=1, default=float)
open(os.path.join(OUT, "checks.txt"), "w").write(LOG.getvalue())
