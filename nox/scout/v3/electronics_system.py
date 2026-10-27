"""Scout v3 electronics mount v2 (27 Oct 2026).

ROUND 4 (current, `python3 nox/scout/v3/electronics_system.py`): the mount on scout_layout v12's tail-bay poses (PR 141: the catch 5.0
forward; the Radxa, UBEC, mount and cables ride the belly mount with it). `run_v12()`: concept A = 2 printed pieces. The belly-mount
tail block (the root): block, gusset bearing on the receiver's tail face, a web under the plate out to 2 plate bolts in the plate's
tail tab, the fin through the tail notch and its foot on the plate top. The tail carrier: a yoke on 4 M3 (the fuse in x, y and yaw;
z, pitch and roll go in bearing on the block), its 2.0 side wall = the yoke's back and the spine's inner wall, a box spine behind the
wash line with the Radxa's 4 posts, the UBEC's torsion box and tray, the XR2 shelf. Sections and modes from mesh slices (hollow, 2.4
walls), every printed piece checked as one body, the layout's own FC USB-C path, wash / funnel line (v12's ring and tail point) / cage
first / fit / access (system_check), CG with the mount and the balancing slide, the UBEC pose region; concept B (a strut to the tail
point) weighed. Outputs runs/scout_electronics/v2/. Round 3 (on v11) is commit 2ef0b72b.

Rounds 1-2 (superseded, `--keel`): the poses optimised on v10 with a box keel, the text below. Places the Radxa Zero 3W, the 5 V UBEC and the RadioMaster XR2 by
optimisation on the race-car cage (scout_layout v10, PRs 127 + 131), jointly with balance (the catch-set slide), draws the mount concept
from the picked poses and checks the assembled result.

Why v2: concept v1 (`runs/scout_electronics/v1/`, one PETG fin) failed review three times (root joint did not fit, CG from
bounding-box centres, hand-picked poses), so it comes back to the system pass (three strikes, docs/generative.md 0).
Spark 27 Oct: "fit the stuff there" (the pointed tail). The FC and the flow sensor stay; the battery side is not touched.

Model: the Scout v3 system model `nox/scout/v3/scout.assembly.json` (every part in pose, catch set 8.0 aft of the thrust centre)
with its meshes in `runs/scout_layout/v10/parts/`, the shell in `runs/scout_layout/v10/cage_shell.json`. Centroids: the manifest's
`cg` where given, else the volume centroid of a watertight mesh, else the area-weighted surface centroid (v1's bounding-box
centres were ~1.5 mm off). The catch set (receiver set, cartridge set, pack, belly mount with FC and flow, wires) slides by DC.

Stage 1, placement (`search`): a 1 mm grid of the assembled model. Every axis-aligned orientation of each item (the discrete set:
the Radxa and the UBEC on edge against the carrier's side walls, the XR2 antenna vertical on a shelf) at every lattice position is
tested against: inside the shell inset by rod r + 3 (cage first), >= 3 off the swept disk columns (wash), >= 2 to every other
part, the swap column, the FC USB-C path, the flow view, the funnel band (30 over the bottom plane, the opening + 40), the belly-mount tail block and the
carrier box kept free; the Radxa's SD card and USB-C port swept with a Ø10 tool to the outside of the shell, its component side
8 mm free for air. The triple (and the slide DC, scanned) minimises
    J = max(worst CG, 0.3) / 0.25 mm + bay volume / 50 cm3 + added cable mass / 5 g          (DEFAULT weights)
worst CG = max(horizontal CG offset pack in, pack out), with the mount and the five cable runs in it.
Stage 2, mount (`mount`): the carrier drawn round the picked poses (belly-mount tail block, yoke with 4 M3 into inserts,
box keel, standoffs, XR2 shelf), every boss modelled; checked for fit, wash, cage first (system_check), access, CG.
Stage 3, recheck (`--rebuild`): the shell rebuilt by `nox/scout/v3/layout_v3.py` at the picked slide with these parts in the
drone (a patched copy in a temp dir: NOX_ROOT, SCOUT_ELEC, SCOUT_FIX_DC hooks only), every check rerun on the rebuilt model.

python3 nox/scout/v3/electronics_system.py               stages 1 + 2 (~6 min): runs/scout_electronics/v2/
python3 nox/scout/v3/electronics_system.py --rebuild     stage 3 (~2 min): runs/scout_electronics/v2/rebuilt/
Drone frame (PR 94): x nose, y left, z up, plate underside z 18.5, thrust centre x 0. Numbers: nox/dimensions.md System Scout v3.
"""
import os, sys, json, math, io, itertools, time, subprocess, tempfile, shutil
import numpy as np, trimesh
from scipy.spatial import ConvexHull
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "nox"))
RUN = os.path.join(ROOT, "runs", "scout_electronics", "v2")
V9 = os.path.join(ROOT, "runs", "scout_layout", "v10")      # the race-car layout as merged (PR 131, 23a7814; v9 + tie saddle)
ASM = os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json")
LOG = io.StringIO()


def say(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); LOG.write(s + "\n")


# ---------------------------------------------------------------- numbers (dimensions.md H, M, L; E where open)
DC_REF = 8.0                      # v10: catch set 8.0 aft of the thrust centre (C, PR 131)
MOTORS = [(54.1, 54.1), (54.1, -54.1), (-54.1, 54.1), (-54.1, -54.1)]      # C (P 153 diagonals)
DISK_R = 44.5                     # swept prop radius (v10 props, C)
WASH = 3.0                        # off the swept disk columns (CLAUDE.md 8, DEFAULT)
CAGE_FIRST = 3.0                  # L, DEFAULT
ROD_R, NODE_R, RING_NODE_R = 1.5, 6.0, 10.0
GAP = 2.0                         # to every other part (DEFAULT)
AIR = 8.0                         # free air over the Radxa's component side (DEFAULT)
TOOL = 10.0                       # Ø10 tool / fingers to outside the shell (v1 method, E)
FUNNEL_OFF = (2.1, 1.7, 3.0)      # funnel line (cage session 27 Oct, hard, E): the chord from the tail bottom rod (ring tail node x - 2.1,
                                  # plane z) to the tail point (tail node x - 1.7, its z), + 3; v10 at catch 8: (-79, -49.6) -> (-197, 41.5).
                                  # It rides the catch (the ring sits about it): derived from the current shell and slide every time
FUNNEL_BAND, FUNNEL_PAST = 30.0, 40.0   # nothing within 30 above the bottom plane over the opening or 40 past its ends (funnel ramps, v10)
SWAP_NOSE, SWAP_TAIL, SWAP_SIDE = 26.5, 17.6, 30.0   # swap column: receiver + 26.5 nose, + 17.6 tail, + 30 sides (v10, C)
CART_BOT = -44.5                  # cartridge flat bottom (C)
W_CG, W_V, W_C, CG_FLOOR = 0.25, 50.0, 5.0, 0.3   # objective: mm, cm3, g; a CG under 0.3 counts as 0.3 (model accuracy, DEFAULT)
NEAR = 20.0                       # one carrier: every pair of items within 20 (DEFAULT)
MASS = {"Radxa Zero 3W": (10.0, "E"), "UBEC": (15.0, "E"), "XR2": (1.0, "P")}  # as v9 (M: XR2 0.8 spec, UBEC 11..21 Q4)
CABLE_GPM = {"camera USB-C": 15.0, "Radxa UART": 2.5, "UBEC 5 V out": 6.6, "UBEC in (FC pads)": 6.6, "XR2 UART": 2.0}  # g/m (E)
ROUTE = 1.25                      # cable length / straight distance (E)
CAMERA_BACK = np.array([135.0, 0.0, 42.0])   # camera rear face centre (v9, C): the USB-C cable leaves here (the camera is cage-hung)
# carrier (DEFAULT, drawn): belly-mount tail block behind the receiver's TPU end pieces, the carrier's yoke clamps it top and bottom
BLK_GAP, BLK_LEN, BLK_HALF, BLK_Z = 2.0, 22.0, 6.5, (-8.0, 18.5)   # block x from 2 behind the TPU end pieces, 21 long, |y| <= 6.5
TONGUE = 5.0                      # yoke tongue: 3.0 counterbore for the M3 head + 2.0 web under it
BOLT_DX = (6.5, 16.5)             # M3 positions from the block's front face (C): the Ø12 pads stay 2.5 off the TPU end pieces
ROOT_WALL = 3.0                   # the root fin's walls (stiffer than the tongues: the fuse goes first, C)
FIN_TOP = 45.0                    # the block's root fin rises through the plate's tail notch (|y| <= 6.5) to here (C, DEFAULT)
NOTCH_X = -39.1                   # the plate's tail notch front (|y| < 9, C from the plate mesh); the fin stops 0.6 short
PLATE_BOLTS = [(-34.0, 9.0), (-34.0, -1.0)]   # 2 new M3 down through the carbon plate into inserts in the belly mount's tail (DEFAULT:
                                  # drill 2 x Ø3.4 in the plate; heads on the plate top, between the rear props, ahead of the notch)
ZCAP = None                       # the bay's top (items + keel) in the tail, set by the rebuild loop (compact cage, review 27 Oct)
SO_D, SO_PILOT = 8.0, 2.1         # Radxa posts Ø8, screw-only: M2.5 x 8 thread-forming into a Ø2.1 pilot through post + wall (DEFAULT)
INSERT = (4.0, 6.7, 10.0)         # M3 heat-set insert hole Ø, depth (5.7 + 1), boss Ø (hole + 2 x 3.0 wall) (M Ruthex)
PAD_D, CBORE_D = 12.0, 6.0        # pad Ø12 round the head, counterbore Ø6 (M3 SHCS head 5.5)
KEEL_OUT, KEEL_WALL = float(os.environ.get("ES_KEEL", 4.0)), 2.4    # box keel 8 wide (|y| <= 4.0, DEFAULT; alt 13 wide), 2.4 side walls = the items' carrier faces
FILL = 0.6                        # lightened walls before topology (E)
PETG = 1.27e-3                    # g/mm3
CATCH = ("mount + nose legs (Scout)", "FC", "flow MTF-02P", "receiver v3 (latch)", "receiver lid (latch)", "receiver TPU strips (latch)",
         "receiver TPU end pieces (latch, E)", "XT60E1-M", "cartridge v6 (latch)", "slider +y (latch)", "slider -y (latch)",
         "cartridge v6 strips (latch, TPU)", "cartridge skid closed end (latch, TPU)", "cartridge skid lead end (latch, TPU)",
         "pack 4S 1500", "pack XT60 plug", "wires")          # v9's catch set (moves with DC), less the three items placed here
MOVED = ("Radxa Zero 3W", "UBEC", "XR2")
CAGE_NAMES = ("cage shell tubes", "cage shell nodes", "spokes, bay ties, core ties, camera guard")
PLACEHOLDERS = ("tail bay mount block (Scout electronics, E)", "tail bay carrier (Scout electronics, E)", "tail bay cables (Scout electronics, E)")   # v12's boxes for our mount


# ---------------------------------------------------------------- the assembled model
def centroid(m):
    if m.is_watertight and m.volume > 1.0: return np.asarray(m.center_mass, float), "volume"
    w = m.area_faces; return (m.triangles_center * w[:, None]).sum(0) / w.sum(), "area"


def load_model(asm_path=ASM, v9=V9):
    A = json.load(open(asm_path)); P = {}
    for p in A["parts"]:
        if p["name"] in PLACEHOLDERS: continue          # the layout's placeholder boxes for this mount (v12)
        if "mesh" in p:
            m = trimesh.load(os.path.join(ROOT, p["mesh"]) if not os.path.isabs(p["mesh"]) else p["mesh"], force="mesh")
        else:
            x0, x1, y0, y1, z0, z1 = p["box"]; m = trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
        c, how = (np.array(p["cg"], float), "manifest") if "cg" in p else centroid(m)
        P[p["name"]] = dict(mesh=m, g=float(p["mass_g"]), tag=p.get("tag", "E"), swap=bool(p.get("swap")), role=p["role"],
                            protect=bool(p.get("protect")), c=c, how=how, catch=p["name"] in CATCH, spec=p)
    sh = json.load(open(os.path.join(v9, "cage_shell.json")))
    links = json.load(open(os.path.join(v9, "result.json")))["links"]
    return dict(A=A, P=P, nodes=np.array(sh["nodes"], float), rods=[tuple(e) for e in sh["rods"]], shell=sh,
                links={k: (np.array(a, float), np.array(b, float)) for k, (a, b) in links.items()}, nring=len(sh["open"][0]["ring"]),
                dc_ref=DC_REF)


MODEL = load_model()


def at_dc(name, dc, M=None):
    M = M or MODEL; P = M["P"][name]; m = P["mesh"]
    if P["catch"] and dc != M["dc_ref"]:
        m = m.copy(); m.apply_translation([-(dc - M["dc_ref"]), 0, 0])
    return m


def base_moments(dc, M=None):
    """mass and first moments (g, g mm) of everything but the three items: pack in, and of the swap parts"""
    M = M or MODEL; sh = np.array([-(dc - M["dc_ref"]), 0, 0])
    Min = np.zeros(3); m_in = 0.0; Msw = np.zeros(3); m_sw = 0.0
    for n, P in M["P"].items():
        if n in MOVED: continue
        c = P["c"] + (sh if P["catch"] else 0)
        Min += P["g"] * c; m_in += P["g"]
        if P["swap"]: Msw += P["g"] * c; m_sw += P["g"]
    return m_in, Min, m_sw, Msw


def anchors(dc, M=None):
    """the drawn carrier's reference faces, from the parts in pose at this slide"""
    tpu = at_dc("receiver TPU end pieces (latch, E)", dc, M).bounds; rec = at_dc("receiver v3 (latch)", dc, M).bounds
    bm = at_dc("mount + nose legs (Scout)", dc, M).bounds
    xb0 = tpu[0][0] - BLK_GAP; xb1 = xb0 - BLK_LEN
    return dict(xb0=xb0, xb1=xb1, rec_tail=rec[0][0], belly_tail=bm[0][0], tpu_top=tpu[1][2], rec_top=rec[1][2], fc=fc_point(dc))


def fc_point(dc): return np.array([-6.1 - (dc - DC_REF), 0.0, 13.5])      # FC / wires box centre (C)


# ---------------------------------------------------------------- the grid (1 mm)
PITCH = 1.0
LO = np.array([-206.0, -146.0, -62.0]); NX, NY, NZ = 368, 292, 136
XC = LO[0] + (np.arange(NX) + 0.5) * PITCH; YC = LO[1] + (np.arange(NY) + 0.5) * PITCH; ZC = LO[2] + (np.arange(NZ) + 0.5) * PITCH


def vox_mesh(m, G):
    v = trimesh.voxel.creation.voxelize(m, pitch=PITCH)
    try: v = v.fill()
    except Exception: pass
    ijk = np.floor((v.points - LO) / PITCH).astype(int)
    ok = ((ijk >= 0) & (ijk < [NX, NY, NZ])).all(1); ijk = ijk[ok]
    G[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True


def vox_capsule(G, a, b, r):
    lo = np.floor((np.minimum(a, b) - r - LO) / PITCH).astype(int).clip(0, [NX, NY, NZ]); hi = (np.ceil((np.maximum(a, b) + r - LO) / PITCH).astype(int) + 1).clip(0, [NX, NY, NZ])
    if (hi <= lo).any(): return
    X, Y, Z = np.meshgrid(XC[lo[0]:hi[0]], YC[lo[1]:hi[1]], ZC[lo[2]:hi[2]], indexing="ij")
    P = np.stack([X, Y, Z], -1); u = b - a; t = np.clip(((P - a) @ u) / max(u @ u, 1e-9), 0, 1)
    G[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] |= np.linalg.norm(P - (a + t[..., None] * u), axis=-1) <= r


def integral(G):
    I = np.zeros((NX + 1, NY + 1, NZ + 1), np.int32)
    I[1:, 1:, 1:] = G.astype(np.int32).cumsum(0).cumsum(1).cumsum(2)
    return I


def box_sum(I, lo, hi, clip=False):
    """occupied voxels whose centres lie in the boxes [lo, hi] (N x 3, mm); a box leaving the grid reads blocked unless clip"""
    i0 = np.ceil((lo - LO) / PITCH - 0.5).astype(int); i1 = np.floor((hi - LO) / PITCH - 0.5).astype(int) + 1
    out = ((i0 < 0) | (i1 > [NX, NY, NZ])).any(1)
    i0 = i0.clip(0, [NX, NY, NZ]); i1 = np.maximum(i1.clip(0, [NX, NY, NZ]), i0)
    s = (I[i1[:, 0], i1[:, 1], i1[:, 2]] - I[i0[:, 0], i1[:, 1], i1[:, 2]] - I[i1[:, 0], i0[:, 1], i1[:, 2]] - I[i1[:, 0], i1[:, 1], i0[:, 2]]
         + I[i0[:, 0], i0[:, 1], i1[:, 2]] + I[i0[:, 0], i1[:, 1], i0[:, 2]] + I[i1[:, 0], i0[:, 1], i0[:, 2]] - I[i0[:, 0], i0[:, 1], i0[:, 2]])
    return np.where(out & (not clip), 10 ** 9, s)


def funnel_line(dc, M=None):
    """(x0, z0, x1, z1): the funnel chord at this slide, from the model's ring and tail point moved with the catch, + 3"""
    M = M or MODEL; sh = -(dc - M["dc_ref"]); ring = M["nodes"][:M["nring"]]; free = M["nodes"][M["nring"]:]
    tp = free[np.argmin(free[:, 0])]
    return (ring[:, 0].min() + sh - FUNNEL_OFF[0], ring[:, 2].mean() + FUNNEL_OFF[2], tp[0] + sh - FUNNEL_OFF[1], tp[2] + FUNNEL_OFF[2])


def funnel_z(x, dc=DC_REF, M=None):
    """the funnel line's z at x (nothing of the tail bay below it); -inf ahead of the ring's tail (the funnel band rules there)"""
    x0, z0, x1, z1 = funnel_line(dc, M); x = np.asarray(x, float)
    return np.where(x < x0, z0 + (x0 - x) * (z1 - z0) / (x0 - x1), -np.inf)


def wash_dist(x, y):
    """horizontal distance outside the nearest swept disk column (negative inside)"""
    return np.min([np.hypot(np.asarray(x) - mx, np.asarray(y) - my) for mx, my in MOTORS], axis=0) - DISK_R


def usb_path(dc, M=None):
    """the system model's FC USB-C cable path (12 x 7 plug + cable, 250 long, E), moved with the catch"""
    M = M or MODEL
    a = [p for p in M["A"]["access"] if p["name"].startswith("FC USB-C")][0]
    u = np.array(a["dir"], float); u /= np.linalg.norm(u)
    return np.array(a["from"], float) - [dc - M["dc_ref"], 0, 0], u


def grids(dc, M=None):
    """O: every solid but the three items; F: where an item body may not be (O + GAP, wash, outside the shell inset, swap column,
    USB path, flow view, low over the opening, the tail block and the carrier box interior); A = O for air and tool sweeps"""
    M = M or MODEL; O = np.zeros((NX, NY, NZ), bool)
    for n in M["P"]:
        if n in MOVED or n.startswith("prop") or n in CAGE_NAMES: continue
        vox_mesh(at_dc(n, dc, M), O)
    for a, b in M["rods"]: vox_capsule(O, M["nodes"][a], M["nodes"][b], ROD_R)
    for a, b in M["links"].values(): vox_capsule(O, a, b, ROD_R)
    for i, q in enumerate(M["nodes"]): vox_capsule(O, q, q, RING_NODE_R if i < M["nring"] else NODE_R)
    ball = np.linalg.norm(np.mgrid[-2:3, -2:3, -2:3], axis=0) <= GAP + 0.01
    F = ndimage.binary_dilation(O, structure=ball)
    X, Y = np.meshgrid(XC, YC, indexing="ij")
    F |= (wash_dist(X, Y) < WASH + 0.5)[:, :, None]                 # + half a voxel: the boxes are read at voxel centres
    X3, Y3, Z3 = (XC[:, None, None].astype(np.float32), YC[None, :, None].astype(np.float32), ZC[None, None, :].astype(np.float32))
    for e in ConvexHull(M["nodes"]).equations:
        F |= (e[0] * X3 + e[1] * Y3 + e[2] * Z3 + e[3]) > -(ROD_R + CAGE_FIRST + 0.5)
    rb = at_dc("receiver v3 (latch)", dc, M).bounds
    sx = (XC >= rb[0][0] - SWAP_TAIL) & (XC <= rb[1][0] + SWAP_NOSE); sy = (YC >= rb[0][1] - SWAP_SIDE) & (YC <= rb[1][1] + SWAP_SIDE)
    F |= sx[:, None, None] & sy[None, :, None] & (ZC < CART_BOT)[None, None, :]
    ring = M["nodes"][:M["nring"]]
    ox = (XC >= ring[:, 0].min() - FUNNEL_PAST) & (XC <= ring[:, 0].max() + FUNNEL_PAST); oy = (YC >= ring[:, 1].min()) & (YC <= ring[:, 1].max())
    F |= ox[:, None, None] & oy[None, :, None] & (ZC < ring[:, 2].mean() + FUNNEL_BAND + 0.5)[None, None, :]
    s, u = usb_path(dc, M); w = np.cross([0, 0, 1.0], u); w /= np.linalg.norm(w); h = np.cross(u, w)
    du = (X3 - s[0]) * u[0] + (Y3 - s[1]) * u[1] + (Z3 - s[2]) * u[2]
    dw = (X3 - s[0]) * w[0] + (Y3 - s[1]) * w[1] + (Z3 - s[2]) * w[2]
    dh = (X3 - s[0]) * h[0] + (Y3 - s[1]) * h[1] + (Z3 - s[2]) * h[2]
    F |= (du >= -1) & (du <= 251) & (np.abs(dw) <= 7.0) & (np.abs(dh) <= 4.5)
    an = anchors(dc, M)          # the tail block + bridge + yoke tongues (+ GAP) and the box keel (|y| < 6.5 behind the block)
    bx = (XC >= an["xb1"] - GAP) & (XC <= an["belly_tail"] + GAP)
    F |= bx[:, None, None] & (np.abs(YC) <= BLK_HALF + GAP)[None, :, None] & ((ZC >= BLK_Z[0] - TONGUE - GAP) & (ZC <= FIN_TOP + GAP))[None, None, :]
    F |= Z3 < funnel_z(XC, dc, M)[:, None, None] + 0.5
    fb = at_dc("flow MTF-02P", dc, M).bounds; lens = np.array([fb[:, 0].mean(), fb[:, 1].mean(), fb[0][2]])
    zz = lens[2] - Z3; tn = math.tan(math.radians(23))
    F |= (zz > 0) & (np.abs(X3 - lens[0]) < zz * tn) & (np.abs(Y3 - lens[1]) < zz * tn)
    F2 = F.copy()                     # F2: without the keel's interior (a flat Radxa on a tray over the keel)
    if not FREE: F |= (XC <= an["xb1"])[:, None, None] & (np.abs(YC) < KEEL_OUT)[None, :, None]
    return O, (F, F2)


# ---------------------------------------------------------------- the items (local frames; H, M)
def B(x0, x1, y0, y1, z0, z1): return np.array([[x0, y0, z0], [x1, y1, z1]], float)


INF = 1000.0
RADXA_HOLES = [(3.55, 3.60), (3.60, 26.45), (61.40, 3.60), (61.40, 26.50)]   # M2.5, board coords (H, STEP)
ITEMS = {
    # Radxa: board X along 65, Y along 30, Z 0 = PCB top, component side +Z (H, STEP); back on 3.0 standoffs, mount face Z -4.6
    # (clears the SD slot, 2.86 under the PCB). Body: SD card stub past X 0, ports' 1.5 overhang past Y 0; plug: the 90 deg
    # USB-C camera plug on USB-C 2 (E). Sweeps: the SD card out along -X and a Ø10 tool; USB-C 1 (OTG, 5 V) along -Y.
    "Radxa Zero 3W": dict(body=[B(-2, 65, -1.5, 30, -4.6, 3.2), B(47, 61, -15, -1.5, -3, 7)], air=[B(0, 65, 0, 30, 3.2, 3.2 + AIR)],
                          sweep=[B(-INF, -2, 10, 20, -8, 2), B(34, 46.5, -INF, -1.5, -4, 8)],
                          cg=np.array([32.5, 15.0, -0.8]), pts={"usb2": np.array([54.0, 0.0, 1.6]), "pads": np.array([32.0, 27.0, -1.6]),
                                                                "sd": np.array([-2.0, 15.0, -3.0]), "usb1": np.array([41.5, -1.5, 1.6])},
                          face=np.array([0, 0, -1.0]), keel=True),
    # UBEC: worst envelope 50 x 25 x 10 (M Q4), leads out both short ends (8 for the bend, E); zip-tied flat on a carrier wall
    "UBEC": dict(body=[B(0, 50, 0, 25, 0, 10), B(-8, 0, 5, 20, 0, 10), B(50, 58, 5, 20, 0, 10)], air=[], sweep=[],
                 cg=np.array([25.0, 12.5, 5.0]), pts={"end0": np.array([-4.0, 12.5, 5.0]), "end1": np.array([54.0, 12.5, 5.0])},
                 face=np.array([0, 0, -1.0]), keel=True),
    # XR2: 16 x 12 x 6 (M), tower antenna 14 over the board (E), vertical (DEFAULT), in a pocket with a 2.0 floor on a shelf
    "XR2": dict(body=[B(0, 16, 0, 12, -2.0, 6), B(6, 10, 4, 8, 6, 20)], air=[], sweep=[],
                cg=np.array([8.0, 6.0, 3.0]), pts={"pads": np.array([8.0, 6.0, 0.0])}, face=np.array([0, 0, -1.0]), keel=False),
}
SHELF = 12.0                      # XR2 within 12 of the carrier's side wall (a 6 x 6 post, DEFAULT)
TRAY_T = 2.4                      # a flat Radxa sits on a 2.4 tray on the keel's top (or hangs under it)
TRAYS = {"Radxa Zero 3W": B(0, 65, -1.5, 30, -4.6 - TRAY_T, -4.6),   # under the posts, short of the SD end (its sweep runs off X 0)
         "UBEC": B(0, 50, 0, 25, -TRAY_T, 0)}                         # the UBEC flat on a tray (round 2: a low bay for a compact cage)


def rotations():
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, j in enumerate(perm): R[j, i] = sg[i]
            if np.linalg.det(R) > 0.5: out.append(R)
    return out


ROTS = rotations()


def orient(item, R):
    it = ITEMS[item]
    def rb(b):
        c = np.array([[b[i][0], b[j][1], b[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)]) @ R.T
        return np.array([c.min(0), c.max(0)])
    body = [rb(b) for b in it["body"]]; o = body[0][0].copy()
    sub = lambda L: [b - o for b in L]
    return dict(item=item, R=R, o=o, body=sub(body), air=sub([rb(b) for b in it["air"]]), sweep=sub([rb(b) for b in it["sweep"]]),
                cg=R @ it["cg"] - o, pts={k: R @ v - o for k, v in it["pts"].items()}, face=R @ it["face"])


RADXA_MODE = "any"                # any | flat | edge: the Radxa's orientation family (alternatives)
FREE = False                      # True: no carrier walls (any orientation, any y): the "forward / split" alternative only


def orient_set(item):
    """the discrete set: Radxa and UBEC with their mount face on a carrier side wall (face normal +-y), any in-plane turn;
    XR2 antenna up or down"""
    seen = set(); out = []
    for R in ROTS:
        o = orient(item, R)
        flat_ok = item in TRAYS and abs(o["face"][2]) > 0.9                      # flat on a tray over the keel (cage session 27 Oct)
        if item == "Radxa Zero 3W" and not FREE and RADXA_MODE != "any" and (RADXA_MODE == "flat") != flat_ok: continue
        if ITEMS[item]["keel"] and not FREE and abs(o["face"][1]) < 0.9 and not flat_ok: continue
        if item == "XR2" and abs((R @ [0, 0, 1.0])[2]) < 0.9: continue
        key = (item, tuple(np.round(np.concatenate([b.ravel() for b in o["body"]] + [o["face"]]), 2))) if item == "UBEC" else id(R)
        if key in seen: continue
        seen.add(key); out.append(o)
    return out


def tray_box(o):
    T_ = TRAYS[o["item"]]; R = o["R"]; c = np.array([[T_[i][0], T_[j][1], T_[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)]) @ R.T
    return np.array([c.min(0), c.max(0)]) - o["o"]


def candidates(item, IF, IA, x_lim=None, IF2=None):
    """every feasible pose: body clear of F, air and sweeps clear of A (sweeps run to the grid edge, out of the shell)"""
    ors = orient_set(item); out = []
    xs = np.arange(-200.0, 140.0, 3.0); zs = np.arange(-45.0, 62.0, 3.0); ys = np.arange(-120.0, 120.0, 2.0)
    for k, o in enumerate(ors):
        flat = item in TRAYS and abs(o["face"][2]) > 0.9 and not FREE
        if ITEMS[item]["keel"] and not FREE and not flat:  # mount face on the wall: y fixed by the face
            ty = [KEEL_OUT] if o["face"][1] < 0 else [-KEEL_OUT - o["body"][0][1][1]]
        else:
            ty = ys
        T = np.stack(np.meshgrid(xs, np.array(ty, float), zs, indexing="ij"), -1).reshape(-1, 3)
        ok = np.ones(len(T), bool)
        if item == "XR2" and not FREE:                    # on a shelf within SHELF of a side wall
            ylo, yhi = T[:, 1] + o["body"][0][0][1], T[:, 1] + o["body"][0][1][1]
            ok &= ((ylo >= KEEL_OUT + GAP) & (ylo <= KEEL_OUT + SHELF)) | ((yhi <= -KEEL_OUT - GAP) & (yhi >= -KEEL_OUT - SHELF))
        if x_lim is not None: ok &= (T[:, 0] + o["cg"][0] < x_lim) if x_lim < 0 else (T[:, 0] + o["cg"][0] > -x_lim)
        if flat:                                          # on a tray over the keel: the board spans the keel, the tray clear too
            ok &= (T[:, 1] + o["body"][0][0][1] <= -KEEL_OUT) & (T[:, 1] + o["body"][0][1][1] >= KEEL_OUT)
            tb = tray_box(o); ok[ok] &= box_sum(IF2, T[ok] + tb[0], T[ok] + tb[1]) == 0
        for b in o["body"]: ok[ok] &= box_sum(IF2 if flat else IF, T[ok] + b[0], T[ok] + b[1]) == 0
        for b in o["air"]: ok[ok] &= box_sum(IA, T[ok] + b[0], T[ok] + b[1]) == 0
        for b in o["sweep"]: ok[ok] &= box_sum(IA, T[ok] + np.maximum(b[0], -INF), T[ok] + np.minimum(b[1], INF), clip=True) == 0
        out += [(k, t) for t in T[ok]]
    return ors, out


def pose_arrays(ors, cands):
    k = np.array([c[0] for c in cands]); t = np.array([c[1] for c in cands])
    def stack(name):
        n_ = len(ors[0][name])
        if not n_: return np.zeros((len(k), 0, 2, 3))
        return np.stack([np.array([ors[i][name][j] for i in k]) + t[:, None, :] for j in range(n_)], 1)
    body = stack("body")
    if ors[0].get("item") in TRAYS:                           # the tray counts as the item's in the clash checks
        body = np.concatenate([body, np.stack([np.array([tray_box(ors[i]) for i in k]) + t[:, None, :]], 1)[:, 0][:, None]], 1)
    return dict(k=k, t=t, body=body, air=stack("air"), sweep=stack("sweep"), cg=np.array([ors[i]["cg"] for i in k]) + t,
                pts={n: np.array([ors[i]["pts"][n] for i in k]) + t for n in ors[0]["pts"]}, ulo=body[:, :, 0].min(1), uhi=body[:, :, 1].max(1))


def boxes_overlap(a, b, pad=0.0):
    return ((a[..., 0, :] < b[..., 1, :] + pad) & (b[..., 0, :] < a[..., 1, :] + pad)).all(-1)


def clash(P, i, Q, j):
    bad = np.zeros(np.broadcast(i, j).shape, bool)
    for a in range(P["body"].shape[1]):
        for b in range(Q["body"].shape[1]): bad |= boxes_overlap(P["body"][i, a], Q["body"][j, b], GAP)
        for b in range(Q["air"].shape[1]): bad |= boxes_overlap(P["body"][i, a], Q["air"][j, b])
        for b in range(Q["sweep"].shape[1]): bad |= boxes_overlap(P["body"][i, a], Q["sweep"][j, b])
    for b in range(Q["body"].shape[1]):
        for a in range(P["air"].shape[1]): bad |= boxes_overlap(Q["body"][j, b], P["air"][i, a])
        for a in range(P["sweep"].shape[1]): bad |= boxes_overlap(Q["body"][j, b], P["sweep"][i, a])
    return bad


def gap(P, i, Q, j):
    d = np.maximum(0, np.maximum(P["ulo"][i] - Q["uhi"][j], Q["ulo"][j] - P["uhi"][i]))
    return np.linalg.norm(d, axis=-1)


# ---------------------------------------------------------------- objective
def cables(dc, r, u, x):
    """the five runs (E): (name, g, midpoint, length mm). UBEC: input end toward the FC pads, output toward the Radxa pads"""
    fc = fc_point(dc); e0, e1 = u["end0"], u["end1"]
    l_a = np.linalg.norm(e0 - fc, axis=-1) + np.linalg.norm(e1 - r["pads"], axis=-1)
    l_b = np.linalg.norm(e1 - fc, axis=-1) + np.linalg.norm(e0 - r["pads"], axis=-1)
    sw = (l_b < l_a)[..., None]; ein = np.where(sw, e1, e0); eout = np.where(sw, e0, e1)
    runs = [("camera USB-C", CAMERA_BACK + 0 * r["usb2"], r["usb2"]), ("Radxa UART", r["pads"], fc + 0 * r["pads"]),
            ("UBEC 5 V out", eout, r["pads"]), ("UBEC in (FC pads)", fc + 0 * ein, ein), ("XR2 UART", x["pads"], fc + 0 * x["pads"])]
    out = []
    for n, a, b in runs:
        L = ROUTE * np.linalg.norm(a - b, axis=-1)
        out.append((n, CABLE_GPM[n] * L / 1000.0, (a + b) / 2, L))
    return out


def mount_est(lo, hi, dc):
    """the carrier before it is drawn (E): tail block 5 g, yoke 4 g, box keel walls over the bay's side face"""
    an = anchors(dc); dx = np.maximum(an["xb1"], hi[..., 0]) - lo[..., 0]; dz = hi[..., 2] - lo[..., 2]
    keel = (2 * dx * dz * KEEL_WALL * FILL + 2 * (dx + dz) * (2 * KEEL_OUT - 2 * KEEL_WALL) * 2.0) * PETG      # as drawn in mount()
    jc = np.array([(an["xb0"] + an["xb1"]) / 2, 0.0, (BLK_Z[0] + BLK_Z[1]) / 2])
    return [(8.0, jc + 0 * lo), (2 * (BLK_LEN + 0.5) * 2 * BLK_HALF * TONGUE * PETG, jc + 0 * lo),
            (keel, np.stack([(np.minimum(an["xb1"], hi[..., 0]) + lo[..., 0]) / 2, 0 * lo[..., 0], (lo[..., 2] + hi[..., 2]) / 2], -1))]


def evaluate(dc, R, i, U, j, X, k, mount=None):
    m_in, Min, m_sw, Msw = base_moments(dc)
    items = [(MASS["Radxa Zero 3W"][0], R["cg"][i]), (MASS["UBEC"][0], U["cg"][j]), (MASS["XR2"][0], X["cg"][k])]
    lo = np.minimum(np.minimum(R["ulo"][i], U["ulo"][j]), X["ulo"][k]); hi = np.maximum(np.maximum(R["uhi"][i], U["uhi"][j]), X["uhi"][k])
    cab = cables(dc, {n: v[i] for n, v in R["pts"].items()}, {n: v[j] for n, v in U["pts"].items()}, {n: v[k] for n, v in X["pts"].items()})
    mo = mount_est(lo, hi, dc) if mount is None else mount
    add = items + [(g, c) for _, g, c, _ in cab] + list(mo)
    gm = sum(np.asarray(g, float) for g, _ in add); Mm = sum(np.asarray(g, float)[..., None] * c for g, c in add)
    cin = (Min + Mm) / (m_in + gm)[..., None]; cout = (Min - Msw + Mm) / (m_in - m_sw + gm)[..., None]
    worst = np.maximum(np.linalg.norm(cin[..., :2], axis=-1), np.linalg.norm(cout[..., :2], axis=-1))
    V = np.prod(hi - lo, axis=-1) / 1000.0; cg_ = sum(g for _, g, _, _ in cab)
    J = np.maximum(worst, CG_FLOOR) / W_CG + V / W_V + cg_ / W_C
    return dict(J=J, worst=worst, cin=cin, cout=cout, V=V, cable_g=cg_, lo=lo, hi=hi, add_g=gm, mass=m_in + gm, cab=cab)


def search(dc, F, A, x_lim=None, log=True, label="tail"):
    """the best (Radxa, UBEC, XR2) for one catch slide"""
    t0 = time.time(); F, F2 = F if isinstance(F, tuple) else (F, F)
    if ZCAP is not None:                                 # the bay's top in the tail (items; the keel follows them)
        cap = (XC <= anchors(dc)["xb1"])[:, None, None] & (ZC > ZCAP - 0.5)[None, None, :]; F = F | cap; F2 = F2 | cap
    IF, IA, IF2 = integral(F), integral(A), integral(F2)
    oR, cR = candidates("Radxa Zero 3W", IF, IA, x_lim, IF2); oU, cU = candidates("UBEC", IF, IA); oX, cX = candidates("XR2", IF, IA)
    if log: say(f"   DC {dc:+5.2f} {label}: feasible poses Radxa {len(cR)}, UBEC {len(cU)}, XR2 {len(cX)}")
    if not cR or not cU or not cX: return None
    Rf, Uf, Xf = pose_arrays(oR, cR), pose_arrays(oU, cU), pose_arrays(oX, cX)
    def coarse(P):
        g = np.round((P["t"] - [-200.0, -120.0, -45.0]) / [3.0, 2.0, 3.0]).astype(int)
        return np.where((g[:, [0, 2]] % 2 == 0).all(1) & ((g[:, 1] % 3 == 0) | (np.abs(np.abs(P["t"][:, 1]) - KEEL_OUT) < 30)))[0]
    iR, iU, iX = coarse(Rf), coarse(Uf), coarse(Xf)
    xr_mid = iX[np.argmin(np.abs(Xf["cg"][iX][:, 0] - np.median(Xf["cg"][iX][:, 0])))]
    best_pairs = []
    for c0 in range(0, len(iR), 300):
        a = iR[c0:c0 + 300][:, None]; b = iU[None, :]
        ok = ~clash(Rf, a, Uf, b) & ((gap(Rf, a, Uf, b) <= NEAR) | FREE)
        if not ok.any(): continue
        aa, bb = np.broadcast_arrays(a, b); aa, bb = aa[ok], bb[ok]
        ev = evaluate(dc, Rf, aa, Uf, bb, Xf, np.full(len(aa), xr_mid))
        o = np.argsort(ev["J"])[:200]
        best_pairs += [(float(ev["J"][q]), int(aa[q]), int(bb[q])) for q in o]
    best_pairs = sorted(best_pairs)[:400]
    best = None
    for _, a, b in best_pairs:
        k = iX; fa, fb = np.full(len(k), a), np.full(len(k), b)
        ok = ~clash(Xf, k, Rf, fa) & ~clash(Xf, k, Uf, fb) & ((np.maximum(gap(Xf, k, Rf, fa), gap(Xf, k, Uf, fb)) <= NEAR) | (FREE & (gap(Xf, k, Uf, fb) <= NEAR)))
        if not ok.any(): continue
        k = k[ok]; ev = evaluate(dc, Rf, np.full(len(k), a), Uf, np.full(len(k), b), Xf, k); q = int(np.argmin(ev["J"]))
        if best is None or ev["J"][q] < best[0]: best = (float(ev["J"][q]), a, b, int(k[q]))
    if best is None: return None
    _, a, b, c = best
    for _ in range(6):                                    # coordinate descent on the full lattice, each item within 9 of its pose
        moved = False
        for which in "RUX":
            P = {"R": Rf, "U": Uf, "X": Xf}[which]; cur = {"R": a, "U": b, "X": c}[which]
            near = np.where(np.abs(P["cg"] - P["cg"][cur]).max(1) <= 9.0)[0]
            ia = near if which == "R" else np.full(len(near), a); ib = near if which == "U" else np.full(len(near), b); ic = near if which == "X" else np.full(len(near), c)
            ok = ~clash(Rf, ia, Uf, ib) & ~clash(Xf, ic, Rf, ia) & ~clash(Xf, ic, Uf, ib)
            ok &= ((gap(Rf, ia, Uf, ib) <= NEAR) & (gap(Xf, ic, Rf, ia) <= NEAR) | FREE) & (gap(Xf, ic, Uf, ib) <= NEAR)
            if not ok.any(): continue
            ev = evaluate(dc, Rf, ia[ok], Uf, ib[ok], Xf, ic[ok]); q = int(np.argmin(ev["J"]))
            if ev["J"][q] < best[0] - 1e-9:
                best = (float(ev["J"][q]), int(ia[ok][q]), int(ib[ok][q]), int(ic[ok][q])); a, b, c = best[1:]; moved = True
        if not moved: break
    ev = evaluate(dc, Rf, np.array([a]), Uf, np.array([b]), Xf, np.array([c]))
    res = dict(dc=dc, label=label, J=float(ev["J"][0]), worst=float(ev["worst"][0]), cin=ev["cin"][0], cout=ev["cout"][0], V=float(ev["V"][0]),
               cable_g=float(ev["cable_g"][0]), lo=ev["lo"][0], hi=ev["hi"][0], mass=float(ev["mass"][0]),
               poses={"Radxa Zero 3W": (oR[Rf["k"][a]], Rf["t"][a]), "UBEC": (oU[Uf["k"][b]], Uf["t"][b]), "XR2": (oX[Xf["k"][c]], Xf["t"][c])},
               cab=[(n, float(np.asarray(g).ravel()[0]), np.asarray(m).reshape(-1, 3)[0], float(np.asarray(L).ravel()[0])) for n, g, m, L in ev["cab"]],
               n_feasible=(len(cR), len(cU), len(cX)), secs=time.time() - t0)
    if log: say(f"      J {res['J']:.2f}: CG in {np.hypot(*res['cin'][:2]):.2f} / out {np.hypot(*res['cout'][:2]):.2f} mm (E), bay {res['V']:.0f} cm3, "
                f"cables {res['cable_g']:.1f} g, bay x {res['lo'][0]:.0f}..{res['hi'][0]:.0f} ({res['secs']:.0f} s)")
    return res


def describe(n, o, t):
    R = o["R"]; ax = lambda v: "+-"[int((R @ v) @ [1, 1, 1] < 0)] + "xyz"[int(np.argmax(np.abs(R @ v)))]
    lo = o["body"][0][0] + t; hi = o["body"][0][1] + t; side = "left (+y)" if lo[1] > 0 else "right (-y)"
    if n == "Radxa Zero 3W":
        return (f"Radxa on the {side} wall: long side {ax(np.array([1.0, 0, 0]))[1:]}, component side {ax(np.array([0, 0, 1.0]))} (out), "
                f"ports edge {ax(np.array([0, -1.0, 0]))}, SD end {ax(np.array([-1.0, 0, 0]))}; board x {lo[0]:.1f}..{hi[0]:.1f}, y {lo[1]:.1f}..{hi[1]:.1f}, z {lo[2]:.1f}..{hi[2]:.1f}")
    if n == "UBEC":
        return f"UBEC on the {side} wall: long side {ax(np.array([1.0, 0, 0]))[1:]}; x {lo[0]:.1f}..{hi[0]:.1f}, y {lo[1]:.1f}..{hi[1]:.1f}, z {lo[2]:.1f}..{hi[2]:.1f}"
    return f"XR2 on a shelf, {side}: antenna {ax(np.array([0, 0, 1.0]))}; x {lo[0]:.1f}..{hi[0]:.1f}, y {lo[1]:.1f}..{hi[1]:.1f}, z {lo[2]:.1f}..{hi[2]:.1f}"


# ---------------------------------------------------------------- stage 2: the carrier, drawn
def box(lo, hi): return trimesh.creation.box(bounds=[np.asarray(lo, float), np.asarray(hi, float)])


def cyl(p0, p1, r, sections=32): return trimesh.creation.cylinder(r, segment=np.vstack([p0, p1]), sections=sections)


def union(ms):
    ms = [m for m in ms if m is not None]
    try: return trimesh.boolean.union(ms, engine="manifold")
    except Exception: return trimesh.util.concatenate(ms)


def diff(a, bs):
    try: return trimesh.boolean.difference([a] + list(bs), engine="manifold")
    except Exception: return a


def item_meshes(res):
    out = {}
    for n, (o, t) in res["poses"].items():
        out[n] = union([box(b[0] + t, b[1] + t) for b in o["body"]])
    return out


def mount(res, dc):
    """the carrier round the picked poses (C geometry, E mass): returns {name: (mesh, g, cg, note)}"""
    an = anchors(dc); xb0, xb1 = an["xb0"], an["xb1"]; z0, z1 = BLK_Z; H = BLK_HALF
    P = res["poses"]; parts = {}
    bolts = [xb0 - d for d in BOLT_DX]
    # 1. the belly mount's tail (one piece with the belly mount, drawn here): the block behind the receiver's TPU end pieces, a root fin
    #    |y| <= 6.5 from the block up through the plate's tail notch (z to FIN_TOP) and forward to the notch front, sitting on the
    #    belly mount's tail (it bears on the receiver's top there, as the belly mount does, by design), a gusset down behind the
    #    receiver; 4 M3 heat-set inserts in the block (2 top, 2 bottom) and 2 more where 2 new M3 come down through the carbon plate
    #    at PLATE_BOLTS (the root's moment goes into the plate right ahead of the notch), every insert in a Ø10 boss (3.0 wall)
    fin = box([xb0, -H, an["rec_top"]], [NOTCH_X - 0.6, H, FIN_TOP])
    blk = union([box([xb1, -H, z0], [xb0, H, z1]), fin, box([xb0, -H, an["tpu_top"] + 2.0], [an["rec_tail"] - 2.0, H, an["rec_top"]]),
                 ]
                + [cyl([px, py, an["rec_top"]], [px, py, 18.5], INSERT[2] / 2) for px, py in PLATE_BOLTS])
    holes = []
    for x in bolts:
        holes += [cyl([x, 0, z1 - INSERT[1]], [x, 0, z1 + 0.1], INSERT[0] / 2), cyl([x, 0, z0 - 0.1], [x, 0, z0 + INSERT[1]], INSERT[0] / 2)]
    holes += [cyl([px, py, 18.5 - INSERT[1]], [px, py, 18.6], INSERT[0] / 2) for px, py in PLATE_BOLTS]
    blk = diff(blk, holes)
    shell_v = lambda dx, dy, dz, w=2.4: (dx * dy * dz - max(dx - 2 * w, 0) * max(dy - 2 * w, 0) * max(dz - 2 * w, 0))   # hollow, walls w
    g_blk = (shell_v(BLK_LEN, 2 * H, z1 - z0) + shell_v(NOTCH_X - 0.6 - xb0, 2 * H, FIN_TOP - an["rec_top"], ROOT_WALL)
             + 2.2 * 2 * H * (an["rec_top"] - an["tpu_top"] - 2.0) + 6 * math.pi * (INSERT[2] / 2) ** 2 * INSERT[1]) * PETG
    parts["belly-mount tail block (PETG)"] = (blk, round(g_blk, 2), "the belly mount's tail: block + root fin through the plate notch + gusset, "
                                              "hollow 2.4 walls, 6 insert bosses (4 yoke, 2 plate bolts)")
    # 2. carrier: yoke tongues over and under the block (Ø12 pads, Ø6 x 3 counterbores, 2.0 web), box keel |y| <= 6.5 behind the
    #    block whose side walls carry the Radxa (standoffs) and the UBEC (zip ties), XR2 shelf on a 6 x 6 post
    tongue_lo = xb1; tongue_hi = xb0
    pts = [[xb1, z0 - TONGUE], [xb1, z1 + TONGUE]]
    flats = {n: tray_box(P[n][0]) + P[n][1] for n in TRAYS if abs(P[n][0]["face"][2]) > 0.9}
    for n in ("Radxa Zero 3W", "UBEC", "XR2"):
        o, t = P[n]; b = flats[n] if n in flats else o["body"][0] + t
        pts += [[b[0][0], b[0][2]], [b[1][0], b[0][2]], [b[0][0], b[1][2]], [b[1][0], b[1][2]]]
    pts = np.array(pts); pts[:, 0] = np.minimum(pts[:, 0], xb1)
    hull = ConvexHull(pts); poly = pts[hull.vertices]
    import shapely.geometry as sg
    pg = sg.Polygon(poly)
    o, t = P["Radxa Zero 3W"]; R = o["R"]                              # keep the Radxa's tool sweeps clear of the keel (C)
    for nm, d in (("sd", R @ [-1.0, 0, 0]), ("usb1", R @ [0, -1.0, 0])):
        p0 = o["pts"][nm] + t
        if abs(p0[1]) - TOOL / 2 - 1.0 < KEEL_OUT and abs(d[1]) < 0.5:
            q = p0 + 400 * d; w = TOOL / 2 + 1.0; n_ = np.array([-d[2], d[0]])
            pg = pg.difference(sg.Polygon([p0[[0, 2]] + w * n_, q[[0, 2]] + w * n_, q[[0, 2]] - w * n_, p0[[0, 2]] - w * n_]))
    for n, trb in flats.items():                                       # the keel stops at a tray: the item (and its air) above / below it
        o_, t_ = P[n]; up = o_["face"][2] < 0; bb = np.array([np.min([b[0] for b in o_["body"]], 0), np.max([b[1] for b in o_["body"]], 0)]) + t_
        ex = AIR if n == "Radxa Zero 3W" else 0.0
        z_a, z_b = (trb[1][2], bb[1][2] + ex + GAP) if up else (bb[0][2] - ex - GAP, trb[0][2])
        pg = pg.difference(sg.box(min(trb[0][0], bb[0][0]) - GAP, z_a, max(trb[1][0], bb[1][0]) + GAP, z_b))
    if pg.geom_type != "Polygon": pg = max(pg.geoms, key=lambda g_: g_.area)
    keel = trimesh.creation.extrude_polygon(pg, 2 * KEEL_OUT)          # extrudes along +z: map (x, z) plane -> (x, z), y thickness
    T = np.array([[1, 0, 0, 0], [0, 0, 1, -KEEL_OUT], [0, 1, 0, 0], [0, 0, 0, 1.0]]); keel.apply_transform(T)
    if keel.volume < 0: keel.invert()
    tongues = [box([tongue_lo, -H, z1], [tongue_hi, H, z1 + TONGUE]), box([tongue_lo, -H, z0 - TONGUE], [tongue_hi, H, z0])]
    pads = [cyl([x, 0, z1], [x, 0, z1 + TONGUE], PAD_D / 2) for x in bolts] + [cyl([x, 0, z0 - TONGUE], [x, 0, z0], PAD_D / 2) for x in bolts]
    car = union([keel] + tongues + pads)
    cb = []
    for x in bolts:
        cb += [cyl([x, 0, z1 - 0.1], [x, 0, z1 + TONGUE + 0.1], 1.7), cyl([x, 0, z1 + TONGUE - 3.0], [x, 0, z1 + TONGUE + 0.1], CBORE_D / 2),
               cyl([x, 0, z0 - TONGUE - 0.1], [x, 0, z0 + 0.1], 1.7), cyl([x, 0, z0 - TONGUE - 0.1], [x, 0, z0 - TONGUE + 3.0], CBORE_D / 2)]
    car = diff(car, cb)
    A_xz = pg.area; perim = pg.length
    g_keel = (2 * A_xz * KEEL_WALL * FILL + perim * (2 * KEEL_OUT - 2 * KEEL_WALL) * 2.0) * PETG
    g_yoke = 2 * (tongue_hi - tongue_lo) * 2 * H * TONGUE * PETG
    parts["tail carrier: yoke + box keel (PETG)"] = (car, round(g_keel + g_yoke, 2), f"keel side face {A_xz / 100:.0f} cm2, walls 2.4 at {FILL:.0%} (E)")
    # 3. Radxa posts: 4 x Ø8, 3.0 from the wall to the board back (clear of the SD slot), screw-only: M2.5 x 8 thread-forming into a
    #    Ø2.1 pilot 5.4 deep through post + wall (no insert: an M2.5 insert would leave < 3 wall in a post this size)
    o, t = P["Radxa Zero 3W"]; R = o["R"]
    so = []
    for hx, hy in RADXA_HOLES:
        a = R @ np.array([hx, hy, -4.6]) - o["o"] + t; b = R @ np.array([hx, hy, -1.6]) - o["o"] + t
        so.append(diff(cyl(a + (a - b) / 3.0 * 0.2, b, SO_D / 2, 24), [cyl(a + (a - b) / 3.0 * 2.6, b + (b - a) * 0.02, SO_PILOT / 2, 12)]))
    parts["Radxa standoffs (PETG)"] = (union(so), round(4 * math.pi * ((SO_D / 2) ** 2 - (SO_PILOT / 2) ** 2) * 3.0 * PETG, 2),
                                       "4 x Ø8 x 3 posts, M2.5 thread-forming into Ø2.1 pilots through post + wall")
    for n, trb in flats.items():
        tr = box(trb[0], trb[1]); parts[f"{n.split()[0]} tray (PETG)"] = (tr, round(tr.volume * FILL * PETG, 2), f"2.4 tray on the keel, {FILL:.0%} (E)")
    # 4. XR2 shelf: pocket floor 2.0 under it and a 6 x 6 post into the keel wall
    o, t = P["XR2"]; b = o["body"][0] + t; fl = o["face"]
    zf = (b[0][2], b[0][2] + 2.0) if fl[2] < 0 else (b[1][2] - 2.0, b[1][2])
    ys = (b[0][1], b[1][1]); yin = KEEL_OUT if ys[0] > 0 else -KEEL_OUT
    xm = (b[0][0] + b[1][0]) / 2; zm = (zf[0] + zf[1]) / 2
    post = box([xm - 3, min(yin, ys[0] if ys[0] > 0 else ys[1]), zm - 3], [xm + 3, max(yin, ys[0] if ys[0] > 0 else ys[1]), zm + 3])
    floor = box([b[0][0], ys[0], zf[0]], [b[1][0], ys[1], zf[1]])
    sh = union([floor, post])
    parts["XR2 shelf (PETG)"] = (sh, round(sh.volume * PETG, 2), "pocket floor 2.0 + 6 x 6 post")
    for k, (m, g, note) in parts.items():
        parts[k] = (m, g, centroid(m)[0], note)
    return parts


# ---------------------------------------------------------------- checks
def surf(m, n=4000):
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=2)[0]])


def under_plate(Q, M=None):
    """points under the carbon plate (z <= its underside) and inside its outline in plan: in the plate's own shadow (no added wash)"""
    M = M or MODEL
    if "_plate_poly" not in M:
        import shapely.geometry as sg, shapely.ops as so
        sec = M["P"]["frame plate (Bee35 carbon)"]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
        pl, T_ = sec.to_planar(); polys = [pg.buffer(0) for pg in pl.polygons_full]
        poly = so.unary_union(polys); off = T_[:2, 3]
        M["_plate_poly"] = (poly, T_)
    poly, T_ = M["_plate_poly"]
    import shapely
    Qh = np.c_[Q[:, :2], np.zeros(len(Q)), np.ones(len(Q))] @ np.linalg.inv(T_).T
    inside = shapely.contains_xy(poly, Qh[:, 0], Qh[:, 1])
    return inside & (Q[:, 2] <= 18.6)


def checks(res, dc, mparts, M=None, label=""):
    """fit (min gaps), wash, shell inset, sweeps; returns a dict of numbers"""
    M = M or MODEL; it = item_meshes(res); out = {}
    others = {n: at_dc(n, dc, M) for n in M["P"] if n not in MOVED and not n.startswith("prop") and n not in CAGE_NAMES}
    rods = [(M["nodes"][a], M["nodes"][b]) for a, b in M["rods"]] + list(M["links"].values())
    def rod_gap(Q):
        d = np.inf
        for a, b in rods:
            u = b - a; t = np.clip(((Q - a) @ u) / (u @ u), 0, 1); d = min(d, float(np.linalg.norm(Q - (a + t[:, None] * u), axis=1).min()))
        nd = min(float(np.linalg.norm(Q[:, None] - M["nodes"][None, M["nring"]:], axis=2).min()) - NODE_R,
                 float(np.linalg.norm(Q[:, None] - M["nodes"][None, :M["nring"]], axis=2).min()) - RING_NODE_R)
        return min(d - ROD_R, nd)
    eq = ConvexHull(M["nodes"]).equations
    allp = {**{n: m for n, m in it.items()}, **{k: v[0] for k, v in mparts.items()}}
    for n, m in allp.items():
        Q = surf(m)
        g = {}
        for k, om in others.items():
            if np.any(m.bounds[0] > om.bounds[1] + 30) or np.any(om.bounds[0] > m.bounds[1] + 30): continue
            pq = trimesh.proximity.ProximityQuery(om); sd = pq.signed_distance(Q)
            g[k] = float(-sd.max())
        for n2, m2 in allp.items():
            if n2 == n: continue
            pq = trimesh.proximity.ProximityQuery(m2); g[n2] = float(-pq.signed_distance(Q).max())
        wd = wash_dist(Q[:, 0], Q[:, 1]); wd[under_plate(Q, M)] = np.inf          # under the carbon plate the wash is the plate's
        w = float(wd.min())
        inset = float((-(Q @ eq[:, :3].T + eq[:, 3])).min(1).min()) - ROD_R
        fz = Q[:, 2] - funnel_z(Q[:, 0], dc, M); fl = float(fz[np.isfinite(fz)].min()) if np.isfinite(fz).any() else float("nan")
        out[n] = dict(gaps=g, wash=w, rods=rod_gap(Q), inset=inset, funnel=fl)
    return out


def sweep_out(res, M=None):
    """Radxa SD card and USB-C 1: a Ø10 tool along the pull direction from the slot to the outside of the shell; the least
    distance from its axis to a rod or node surface past the shell face (>= 5 = the tool passes through a cell)"""
    M = M or MODEL; o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]; out = {}
    eq = ConvexHull(M["nodes"]).equations
    for nm, p, d in (("SD card", o["pts"]["sd"] + t, R @ np.array([-1.0, 0, 0])),
                     ("USB-C 1", o["pts"]["usb1"] + t, R @ np.array([0, -1.0, 0]) if USB1_DIR is None else np.asarray(USB1_DIR, float))):
        s = np.linspace(0, 300, 601); Q = p + s[:, None] * d
        inside = (Q @ eq[:, :3].T + eq[:, 3]).max(1) <= 0
        exit_s = float(s[np.argmin(inside)]) if not inside.all() else np.inf
        dmin = np.inf
        for a, b in [(M["nodes"][a], M["nodes"][b]) for a, b in M["rods"]] + list(M["links"].values()):
            u = b - a; tt = np.clip(((Q - a) @ u) / (u @ u), 0, 1); dmin = min(dmin, float(np.linalg.norm(Q - (a + tt[:, None] * u), axis=1).min()) - ROD_R)
        dn = float(np.linalg.norm(Q[:, None] - M["nodes"][None], axis=2).min()) - NODE_R
        out[nm] = dict(start=np.round(p, 1).tolist(), dir=np.round(d, 2).tolist(), exits_shell_at_mm=round(exit_s, 1), least_clear_mm=round(min(dmin, dn), 1))
    return out


def manifest(res, dc, mparts, M=None, extra_access=True):
    """the Scout v3 system model with the catch slide, the three items as placed and the carrier pieces"""
    M = M or MODEL; A = json.loads(json.dumps(M["A"])); sh = -(dc - M["dc_ref"])
    parts = []
    for p in A["parts"]:
        if p["name"] in MOVED: continue
        q = dict(p)
        if "mesh" in q and not os.path.isabs(q["mesh"]): q["mesh"] = os.path.join(ROOT, q["mesh"])
        if p["name"] in CATCH and sh != 0:
            if "box" in q: x0, x1, y0, y1, z0, z1 = q["box"]; q["box"] = [x0 + sh, x1 + sh, y0, y1, z0, z1]
            else: q["pose"] = {"t": [sh, 0, 0]}
            if "cg" in q: q["cg"] = list(q["cg"])
        if "cg" not in q and "mesh" in q and p["name"] in M["P"]:
            q["cg"] = np.round(M["P"][p["name"]]["c"], 3).tolist()   # true centroid (system_check would take a bbox centre off a non-watertight mesh)
        parts.append(q)
    for n, m in item_meshes(res).items():
        parts.append(dict(name=n, role="bought", protect=True, trimesh=m, mass_g=MASS[n][0], tag=MASS[n][1], geom_tag="C"))
    for n, (m, g, c, note) in mparts.items():
        parts.append(dict(name=n, role="structure", trimesh=m, mass_g=g, tag="E", material="PETG", geom_tag="C"))
    cab = res["cab"]
    parts.append(dict(name="tail cables (5 runs)", role="structure", box=list(np.repeat(np.average([c for _, _, c, _ in cab], axis=0, weights=[g for _, g, _, _ in cab]), 2) + [-0.5, 0.5] * 3),
                      mass_g=round(sum(g for _, g, _, _ in cab), 2), tag="E"))
    A["parts"] = parts
    acc = []
    for a in A["access"]:
        a = dict(a)
        if "from" in a and a["part"] in CATCH: a["from"] = (np.array(a["from"]) + [sh, 0, 0]).tolist()
        if a["name"].startswith("FC USB-C") and FC_USB_FROM is not None:
            a["from"] = list(FC_USB_FROM); a["name"] = "FC USB-C cable (12 x 7, from the FC's tail-right port, E)"
        a["ignore"] = list(a.get("ignore", [])) + ["tail cables (5 runs)"]
        acc.append(a)
    if extra_access:
        o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]
        acc.append(dict(name="Radxa microSD out + Ø10 fingers", part="Radxa Zero 3W", **{"from": (o["pts"]["sd"] + t).tolist()}, dir=(R @ [-1.0, 0, 0]).tolist(),
                        length_mm=150.0, tool={"type": "cylinder", "d": TOOL}, ignore=["tail cables (5 runs)"]))
        acc.append(dict(name="Radxa USB-C 1 plug + Ø10", part="Radxa Zero 3W", **{"from": (o["pts"]["usb1"] + t).tolist()},
                        dir=(R @ [0, -1.0, 0]).tolist() if USB1_DIR is None else list(USB1_DIR),
                        length_mm=150.0, tool={"type": "cylinder", "d": TOOL}, ignore=["tail cables (5 runs)"]))
        an = anchors(dc, M); z0, z1 = BLK_Z; CARRIER = [k for k in mparts if k.startswith("tail carrier")][0]
        for x in [an["xb0"] - d for d in BOLT_DX]:
            acc.append(dict(name=f"M3 key, carrier top bolt (x {x:.1f}), ball-end 15 deg to +y", part=CARRIER, **{"from": [x, 0.0, z1 + TONGUE]},
                            dir=list(TOP_TILT), length_mm=150.0, tool={"type": "cylinder", "d": 4}, ignore=["tail cables (5 runs)"]))
            rear = x < an["xb0"] - BOLT_DX[0] - 1.0       # the rear one: a ball-end key 15 deg aft, past the ring's tail rod under it (C)
            acc.append(dict(name=f"M3 key, carrier bottom bolt (x {x:.1f})" + (", ball-end 15 deg aft" if rear else ""), part=CARRIER,
                            **{"from": [x, 0.0, z0 - TONGUE]}, dir=([-0.259, 0, -0.966] if rear else [0, 0, -1]), length_mm=150.0, tool={"type": "cylinder", "d": 4},
                            ignore=["tail cables (5 runs)"] + [n for n in CATCH if M["P"].get(n, {}).get("swap")]))
        for px, py in PLATE_BOLTS:
            acc.append(dict(name=f"M3 key, plate bolt ({px:.0f}, {py:+.1f})" + ("" if py > 3 else ", ball-end 15 deg to -y"), part="belly-mount tail block (PETG)", **{"from": [px, py, 22.5]},
                            dir=([0, 0, 1] if py > 3 else [0, -0.259, 0.966]), length_mm=150.0, tool={"type": "cylinder", "d": 4}, ignore=["tail cables (5 runs)"]))
    A["access"] = acc
    car = list(mparts)
    cp = [c for c in A["contact_pairs"] if not (set(c[:2]) & set(MOVED))]
    cp += [[a, b, 1e9] for a in car for b in car if a < b]
    cp += [[n, k, 1e9] for n in MOVED for k in car] + [["Radxa Zero 3W", "Radxa standoffs (PETG)", 1e9], ["XR2", "XR2 shelf (PETG)", 1e9]]
    cp += [["belly-mount tail block (PETG)", n, 1e9] for n in ("mount + nose legs (Scout)", "receiver v3 (latch)", "frame plate (Bee35 carbon)")]
    cp += [[n, f"{n.split()[0]} tray (PETG)", 1e9] for n in MOVED if f"{n.split()[0]} tray (PETG)" in mparts]
    cp += [["tail cables (5 runs)", n, 1e9] for n in [p["name"] for p in parts] if n != "tail cables (5 runs)"]
    A["contact_pairs"] = cp
    A["name"] = "Scout v3 with the tail electronics (system pass v2)"
    return A


def run_system_check(man, out_txt=None):
    from system_check import run, report
    r = run(man)
    txt = report(r)
    if out_txt: open(out_txt, "w").write(txt + "\n")
    return r, txt


def cg_final(res, dc, mparts, M=None):
    """mass and CG pack in / out with the drawn carrier (true centroids)"""
    M = M or MODEL; m_in, Min, m_sw, Msw = base_moments(dc, M)
    rows = [(n, MASS[n][0], MASS[n][1], o["cg"] + t) for n, (o, t) in res["poses"].items()]
    rows += [(k, g, "E", c) for k, (m, g, c, _) in mparts.items()]
    rows += [(f"cable: {n}", g, "E", c) for n, g, c, _ in res["cab"]]
    g = sum(r[1] for r in rows); Mm = sum(r[1] * r[3] for r in rows)
    cin = (Min + Mm) / (m_in + g); cout = (Min - Msw + Mm) / (m_in - m_sw + g)
    return dict(rows=rows, add_g=g, mass=m_in + g, mass_out=m_in - m_sw + g, cin=cin, cout=cout,
                hin=float(np.hypot(*cin[:2])), hout=float(np.hypot(*cout[:2])), bay_cg=Mm / g)


def slide_scan(res, dc, mparts, M=None):
    """the carrier rides the belly mount: CG pack in / out against the slide with everything of ours moving with it"""
    M = M or MODEL; out = []
    for d in np.arange(dc - 3.0, dc + 3.01, 0.25):
        sh = np.array([-(d - dc), 0, 0]); m_in, Min, m_sw, Msw = base_moments(d, M)
        rows = [(MASS[n][0], o["cg"] + t + sh) for n, (o, t) in res["poses"].items()] + [(g, c + sh) for _, (m, g, c, _) in mparts.items()]
        rows += [(g, c + sh / 2) for _, g, c, _ in res["cab"]]
        g = sum(r[0] for r in rows); Mm = sum(r[0] * r[1] for r in rows)
        cin = (Min + Mm) / (m_in + g); cout = (Min - Msw + Mm) / (m_in - m_sw + g)
        out.append((round(float(d), 2), float(np.hypot(*cin[:2])), float(np.hypot(*cout[:2])), float(cin[0]), float(cout[0])))
    return out


SIG_ALLOW, TAU_PULL = 25.0, 15.0   # PETG bending allowable, head pull-through shear through a 2.0 web (MPa, E; L / T)
PRELOAD = 300.0                   # M3 into a heat-set insert in PETG, working preload (N, E)


def box_section(b, h, t):
    """hollow rectangle b (lateral, y) x h (vertical, z), wall t: Z about y (vertical bending), Z about z (lateral), Bredt factor"""
    Iy = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12; Iz = (h * b ** 3 - (h - 2 * t) * (b - 2 * t) ** 3) / 12
    return Iy / (h / 2), Iz / (b / 2), 2 * (b - t) * (h - t) * t


def loads(res, dc, mparts):
    """hand estimate (E). Two sections: the yoke joint (the fuse: 4 M3 through Ø12 pads with a 2.0 web under the heads) and the
    root of the belly mount's tail (the fin, a 13 x (FIN_TOP - receiver top) hollow box at the receiver's tail, the weakest section
    between the bay and the carbon plate; behind it the 2 plate bolts). Cases: 35 g on the carried mass along x, y, z at its CG;
    50 N on the bay's far top corner along y and z. Index = stress / allowable; the fuse must reach 1.5 x the root's first."""
    an = anchors(dc); z0, z1 = BLK_Z
    rows = [(MASS[n][0], o["cg"] + t) for n, (o, t) in res["poses"].items()]
    rows += [(g, c) for k, (m, g, c, _) in mparts.items() if not k.startswith("belly")]
    rows += [(g * 0.5, c) for _, g, c, _ in res["cab"]]            # half of each run hangs on the carrier (E)
    m = sum(r[0] for r in rows); c = sum(r[0] * r[1] for r in rows) / m
    blk = mparts["belly-mount tail block (PETG)"]; mb, cb = blk[1], blk[2]
    jy = np.array([an["xb0"] - (BOLT_DX[0] + BOLT_DX[1]) / 2, 0.0, (z0 + z1) / 2])          # yoke bolt group centre
    xr = an["rec_tail"] - 1.0; hr = FIN_TOP - an["rec_top"]; jr = np.array([xr, 0.0, an["rec_top"] + hr / 2])   # root section centre
    Zv, Zl, Bt = box_section(2 * BLK_HALF, hr, ROOT_WALL)
    zarm = (z1 + TONGUE / 2) - (z0 - TONGUE / 2); xarm = BOLT_DX[1] - BOLT_DX[0]
    far = np.array([res["lo"][0], 0.0, res["hi"][2]])
    cases = {}
    def joint(F, p, Fr=None, pr=None):
        """forces F at p on the carried part (and Fr at pr on the block too, for the root): bolt tension / shear, root stresses"""
        a = p - jy; M = np.cross(a, F)                             # moments about the yoke centre
        T = abs(M[1]) / zarm + abs(F[2]) / 4 + abs(M[0]) / (2 * BLK_HALF) / 2   # pitch couple, direct, roll prying
        S = abs(M[2]) / xarm / 2 + np.hypot(F[0], F[1]) / 4                     # yaw couple, direct shear
        fuse = max(T / (math.pi * 5.5 * 2.0) / TAU_PULL, S / (3.0 * TONGUE) / SIG_ALLOW)
        Ft = F + (Fr if Fr is not None else 0); Mr = np.cross(p - jr, F) + (np.cross(pr - jr, Fr) if Fr is not None else 0)
        sig = abs(Mr[1]) / Zv + abs(Mr[2]) / Zl + abs(Ft[0]) / (2 * ROOT_WALL * (2 * BLK_HALF + hr)); tau = abs(Mr[0]) / Bt
        root = math.sqrt(sig ** 2 + 3 * tau ** 2) / SIG_ALLOW
        pb = (abs(Mr[1]) + abs(Ft[2]) * abs(xr - PLATE_BOLTS[0][0])) / 15.0 / 2      # plate bolts: the root moment over a 15 prying arm (E)
        return dict(bolt_T=round(T, 1), bolt_S=round(S, 1), fuse=round(fuse, 3), root_sigma=round(sig, 2), root_tau=round(tau, 2), root=round(root, 3),
                    plate_bolt_T=round(pb, 1), ratio=round(fuse / max(root, 1e-9), 2))
    g35 = 35 * 9.81 / 1000
    for ax, v in (("x", [1, 0, 0]), ("y", [0, 1, 0]), ("z", [0, 0, 1])):
        cases[f"35 g {ax}"] = joint(m * g35 * np.array(v, float), c, mb * g35 * np.array(v, float), cb)
    for ax, v in (("y", [0, 1, 0]), ("z", [0, 0, 1])):
        cases[f"50 N {ax} at the far corner"] = joint(50.0 * np.array(v, float), far)
    worst_root = max(v["root"] for v in cases.values()); worst_fuse = max(v["fuse"] for v in cases.values())
    order_ok = all(v["fuse"] >= 1.5 * v["root"] for v in cases.values())
    L = abs(c[0] - xr); Iz = Zl * BLK_HALF; k = 3 * 2000 * Iz / L ** 3
    return dict(carried_g=round(m, 1), carried_cg=np.round(c, 1).tolist(), yoke_centre=np.round(jy, 1).tolist(), root_section=dict(
                    x=round(xr, 1), b=2 * BLK_HALF, h=round(hr, 1), wall=ROOT_WALL, Zv=round(Zv, 0), Zl=round(Zl, 0)),
                far_corner=np.round(far, 1).tolist(), cases=cases, worst_root=worst_root, worst_fuse=worst_fuse, fuse_first=order_ok,
                mode_lateral_Hz=round(math.sqrt(k * 1000 / (m / 1000)) / (2 * math.pi), 0))


# ---------------------------------------------------------------- reference: v9's own DEFAULT tail bay, in this model
def v9_bay_eval(dc=DC_REF):
    """the system model's own tail bay (Radxa, UBEC 13 behind the receiver, XR2 on the plate) with true centroids and the wash"""
    m_in, Min, m_sw, Msw = base_moments(dc); rows = []; wash = {}
    for n in MOVED:
        mm = MODEL["P"][n]["mesh"]; rows.append((MODEL["P"][n]["g"], centroid(mm)[0])); wash[n] = float(wash_dist(*surf(mm, 500)[:, :2].T).min())
    g = sum(r[0] for r in rows); Mm = sum(r[0] * r[1] for r in rows)
    cin = (Min + Mm) / (m_in + g); cout = (Min - Msw + Mm) / (m_in - m_sw + g)
    return dict(hin=float(np.hypot(*cin[:2])), hout=float(np.hypot(*cout[:2])), cin=cin.tolist(), cout=cout.tolist(), wash=wash)


# ---------------------------------------------------------------- picture: side, top, rear of the pick and the alternative
COL = {"Radxa Zero 3W": "#2e7d32", "UBEC": "#c77700", "XR2": "#7b1fa2"}


def preview(rows, path, M=None):
    """rows: [(title, res, dc, mparts)]; three views each, every part in pose, wash columns, tool sweeps"""
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    fig, axs = plt.subplots(len(rows), 3, figsize=(19, 6.2 * len(rows)), gridspec_kw=dict(width_ratios=[1.35, 1.35, 1.0]))
    axs = np.atleast_2d(axs)
    views = [("side (from +y): x, z", (0, 2)), ("top: x, y", (0, 1)), ("rear (from -x): y, z", (1, 2))]
    for r_, row in enumerate(rows):
        title, res, dc, mparts = row[:4]; M = row[4] if len(row) > 4 else MODEL
        for c_, (vt, (i, j)) in enumerate(views):
            ax = axs[r_, c_]; ax.set_aspect("equal"); ax.set_title(f"{title}\n{vt}", fontsize=10)
            flip = -1 if c_ == 2 else 1               # rear view: looking forward from behind, +y on the left
            def poly(m, fc, ec="none", a=0.35, z=1):
                V = m.vertices[:, [i, j]] * [flip if c_ == 2 else 1, 1]; Fc = m.faces
                if len(Fc) > 6000: Fc = Fc[np.random.default_rng(0).choice(len(Fc), 6000, replace=False)]
                ax.add_collection(PolyCollection(V[Fc], facecolor=fc, edgecolor=ec, alpha=a, linewidths=0.2, zorder=z))
            for n in M["P"]:
                if n in MOVED: continue
                m = at_dc(n, dc, M)
                if n in CAGE_NAMES: continue
                colr = "#9aa7b4" if not M["P"][n]["swap"] else "#4f6fcf"
                if n.startswith("prop"): colr = "#e8c547"
                if n.startswith("frame plate"): colr = "#404040"
                poly(m, colr, a=0.18 if not n.startswith("prop") else 0.12)
            for a_, b_ in M["rods"]:
                p, q = M["nodes"][a_], M["nodes"][b_]
                ax.plot([flip * p[i] if c_ == 2 else p[i], flip * q[i] if c_ == 2 else q[i]], [p[j], q[j]], color="#222", lw=0.9, zorder=3)
            for p, q in M["links"].values():
                ax.plot([flip * p[i] if c_ == 2 else p[i], flip * q[i] if c_ == 2 else q[i]], [p[j], q[j]], color="#666", lw=0.6, zorder=3)
            for k_, (m, g, cc, note) in mparts.items():
                poly(m, "#4a90d9", "#1f4f8a", 0.55, 4)
            for n, m in item_meshes(res).items():
                poly(m, COL[n], "k", 0.75, 5)
            if c_ == 0:                                 # the funnel line at this catch
                fx0, fz0, fx1, fz1 = funnel_line(dc, M); ax.plot([fx0, fx1], [fz0, fz1], color="#c62828", lw=1.0, ls="--", zorder=6)
            if c_ == 1:                                 # wash columns + 3 margin
                for mx, my in MOTORS:
                    ax.add_patch(plt.Circle((mx, my), DISK_R + WASH, color="#e8c547", alpha=0.10, zorder=0))
            o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]
            for nm, p, d in (("SD", o["pts"]["sd"] + t, R @ [-1.0, 0, 0]), ("USB-C 1", o["pts"]["usb1"] + t, R @ [0, -1.0, 0])):
                q = p + 35 * d
                if abs(d[i]) + abs(d[j]) > 0.5:
                    ax.annotate("", xy=(flip * q[i] if c_ == 2 else q[i], q[j]), xytext=(flip * p[i] if c_ == 2 else p[i], p[j]),
                                arrowprops=dict(arrowstyle="->", color="#c62828", lw=1.4), zorder=6, annotation_clip=True)
                    ax.text(flip * q[i] if c_ == 2 else q[i], q[j], " " + nm, color="#c62828", fontsize=7, zorder=6, clip_on=True)
            lim = {0: (-215, 160), 1: (-150, 150), 2: (-62, 72)}
            ax.set_xlim(*(lim[i] if c_ != 2 else (-150, 150))); ax.set_ylim(*lim[j])
            ax.axvline(0 if c_ != 2 else 0, color="#999", lw=0.5, ls=":")
            ax.grid(alpha=0.15); ax.tick_params(labelsize=7)
    fig.text(0.01, 0.003, "green Radxa Zero 3W (with its low 90 deg plugs), orange UBEC, purple XR2, blue the mount (belly-mount tail block, yoke, spine / keel, trays, "
             "standoffs, XR2 shelf); grey drone parts, dark blue the swap set, black the cage rods; red arrows the SD and USB-C tool sweeps; "
             "yellow rings = prop columns + 3 (wash)", fontsize=8)
    fig.tight_layout(rect=(0, 0.015, 1, 1)); fig.savefig(path, dpi=110); plt.close(fig)


# ---------------------------------------------------------------- stage 3: the shell rebuilt round these parts at the picked slide
def patched_layout(dst):
    src_path = os.path.join(ROOT, "nox", "scout", "v3", "layout_v3.py")
    src = open(src_path).read()
    a = 'HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))'
    b = "    return D\n\n\ndef build(DC):"
    c = "for _ in range(4):                                                     # refine round the best"
    for s_ in (a, b, c): assert s_ in src, f"layout_v3.py changed: {s_[:40]}"
    src = src.replace(a, 'HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.environ["NOX_ROOT"]')
    src = src.replace(b, '''    EJ = os.environ.get("SCOUT_ELEC")
    if EJ:                                   # electronics system pass v2: the parts in the drone frame at dc_ref, riding the catch set
        ej = json.load(open(EJ)); sh_ = float(ej["dc_ref"]) - DC
        for n, v in ej["parts"].items():
            m = trimesh.load(v["mesh"], force="mesh"); m.apply_translation([sh_, 0.0, 0.0])
            D[n] = (m, v.get("role", "bought"), bool(v.get("protect", True)), float(v["mass_g"]), v.get("tag", "E"), v.get("material"),
                    v.get("group", "bought"), v.get("color", "#2e7d32"), False)
''' + b)
    src = src.replace(c, "for _ in range(0 if os.environ.get('SCOUT_FIX_DC') else 4):")
    open(dst, "w").write(src)


def run_layout(out_dir, dc, elec_json=None):
    """PR 127 / 131's layout script (patched copy in a temp dir) at a fixed slide; with elec_json the tail bay is ours, without it
    the script's own placeholder tail bay (the reference)"""
    tmp = tempfile.mkdtemp(); lay = os.path.join(tmp, "layout_v3_e.py"); patched_layout(lay)
    env = dict(os.environ, NOX_ROOT=ROOT, SCOUT_DCS=str(dc), SCOUT_FIX_DC="1")
    if elec_json: env["SCOUT_ELEC"] = elec_json
    else: env.pop("SCOUT_ELEC", None)
    r = subprocess.run([sys.executable, lay, out_dir], env=env, capture_output=True, text=True)
    shutil.rmtree(tmp, ignore_errors=True)
    if r.returncode: raise RuntimeError(r.stderr[-3000:])
    return r.stdout


def shell_stats(out_dir):
    """the rebuilt shell: nodes, rods, offset picked, node extents, hull volume, longest rod, bbox of the drone (layout log)"""
    sj = json.load(open(os.path.join(out_dir, "cage_shell.json"))); nd = np.array(sj["nodes"]); E_ = sj["rods"]
    log = open(os.path.join(out_dir, "checks.txt")).read()
    off = [l for l in log.splitlines() if l.startswith("structure bar")]
    off = float(off[0].split("picked start")[1].split()[0]) if off else float("nan")
    ext = nd.max(0) - nd.min(0)
    return dict(nodes=len(nd), rods=len(E_), offset=off, extents=np.round(ext, 1).tolist(), width=round(float(ext[1]), 1),
                hull_L=round(ConvexHull(nd).volume / 1e6, 2), longest=round(max(np.linalg.norm(nd[a] - nd[b]) for a, b in E_), 1))


def rebuild(res, dc, mparts, out_dir):
    os.makedirs(os.path.join(out_dir, "inputs"), exist_ok=True)
    ej = dict(dc_ref=dc, parts={})
    for n, m in item_meshes(res).items():
        f = os.path.join(out_dir, "inputs", n.split()[0].lower() + ".stl"); m.export(f)
        ej["parts"][n] = dict(mesh=f, mass_g=MASS[n][0], tag=MASS[n][1], role="bought", protect=True)
    for k, (m, g, c, note) in mparts.items():
        f = os.path.join(out_dir, "inputs", "".join(ch if ch.isalnum() else "_" for ch in k.lower())[:40] + ".stl"); m.export(f)
        ej["parts"][k] = dict(mesh=f, mass_g=g, tag="E", role="structure", protect=False, material="PETG", group="printed", color="#4a90d9")
    jf = os.path.join(out_dir, "inputs", "electronics.json"); json.dump(ej, open(jf, "w"), indent=1)
    return run_layout(out_dir, dc, jf)


def rebuilt_model(out_dir, dc):
    """the system model on the rebuilt shell: the main manifest's parts, meshes from the rebuilt run (catch set at dc)"""
    A = json.load(open(ASM)); pj = {p["name"]: p for p in json.load(open(os.path.join(out_dir, "parts", "parts.json")))}
    for p in A["parts"]:
        if p["name"] in pj and "mesh" in p:
            p["mesh"] = os.path.relpath(os.path.join(out_dir, "parts", pj[p["name"]]["file"]), ROOT); p["mass_g"] = pj[p["name"]]["mass_g"]
            p.pop("cg", None)
        if "box" in p and p["name"] == "wires":
            x0, x1, y0, y1, z0, z1 = p["box"]; sh = -(dc - DC_REF); p["box"] = [x0 + sh, x1 + sh, y0, y1, z0, z1]
    sj = json.load(open(os.path.join(out_dir, "cage_shell.json")))
    A["cage_shell"] = sj
    tmpA = os.path.join(out_dir, "system_model_base.json"); json.dump(A, open(tmpA, "w"))
    M = load_model(tmpA, out_dir); M["dc_ref"] = dc
    for p in A["parts"]:
        if p["name"] in ("cage shell tubes", "cage shell nodes", "spokes, bay ties, core ties, camera guard"):
            m = M["P"][p["name"]]["mesh"]; M["P"][p["name"]]["c"] = centroid(m)[0]
    acc = []
    for a in A["access"]:
        a = dict(a)
        if "from" in a and a["part"] in CATCH: a["from"] = (np.array(a["from"]) - [dc - DC_REF, 0, 0]).tolist()
        acc.append(a)
    M["A"]["access"] = acc
    os.remove(tmpA)
    return M


# ---------------------------------------------------------------- reporting helpers
def pose_json(res):
    out = {}
    for n, (o, t) in res["poses"].items():
        b = o["body"][0] + t
        out[n] = dict(R=o["R"].astype(int).tolist(), t=np.round(t, 2).tolist(), body=[np.round(x, 2).tolist() for x in b],
                      boxes=[[np.round(x + t, 2).tolist() for x in bb] for bb in o["body"]], cg=np.round(o["cg"] + t, 2).tolist(),
                      mass_g=MASS[n][0], tag=MASS[n][1], text=describe(n, o, t))
    return out


def res_from_json(d):
    poses = {}
    for n, v in d["poses"].items():
        o = orient(n, np.array(v["R"], float)); poses[n] = (o, np.array(v["t"], float))
    cab = [(c["name"], c["g"], np.array(c["mid"]), c["L"]) for c in d["cables"]]
    return dict(poses=poses, cab=cab, lo=np.array(d["bay"]["lo"]), hi=np.array(d["bay"]["hi"]), dc=d["dc"])


def fmt_checks(ck):
    lines = []
    for n, v in ck.items():
        g = sorted(v["gaps"].items(), key=lambda kv: kv[1])[:3]
        lines.append(f"   {n}: wash {v['wash']:+.1f} (need >= {WASH:.0f}), above the funnel line {v['funnel']:+.1f} (need >= 0), shell inset {v['inset']:.1f} (need >= {CAGE_FIRST:.0f}), rods/nodes {v['rods']:.1f}, "
                     f"nearest: " + ", ".join(f"{k} {d:.1f}" for k, d in g))
    return lines


SHELL_CAP_REF = 52                # layout_v3.py SHELL_CAP: 62 rods in all (Spark 27 Oct "half the rods", review_precheck 13) minus ~10 links
CAPS = [None, 40.0, 32.0, 26.0, 20.0]   # the bay's top scanned (none = free), each pick rebuilt by the layout script
W_SLACK, L_SLACK = 10.0, 0.3      # compact = rods <= cap, node width <= the placeholder bay's + 10, hull <= its + 0.3 L (DEFAULT)


def run_all():
    """the whole loop: reference shells; placement x slide x bay cap with every pick's shell rebuilt (the compact shell is a
    constraint, its CG the objective); the placement again on the chosen rebuilt shell; the final rebuild and every check"""
    global MODEL, ZCAP, RADXA_MODE
    t0 = time.time(); V10 = MODEL
    say("Scout v3 electronics, system pass v2 round 2 (27 Oct 2026): nox/scout/v3/electronics_system.py on scout_layout v10 (main), "
        "the shell rebuilt by nox/scout/v3/layout_v3.py inside the loop")
    say(f"model: {len(MODEL['P'])} parts in pose, catch set {DC_REF} aft (v10); shell {len(MODEL['nodes'])} nodes / {len(MODEL['rods'])} rods + {len(MODEL['links'])} links")
    ref = v9_bay_eval()
    say(f"reference, v10's own tail bay in this model (catch {DC_REF}): CG pack in {ref['hin']:.2f}, out {ref['hout']:.2f} mm (E); wash "
        + ", ".join(f"{n} {w:+.1f}" for n, w in ref["wash"].items()) + " (negative = inside a disk column)")
    refs = {}
    for dc in (0.0, DC_REF):
        d_ = tempfile.mkdtemp(); run_layout(d_, dc); refs[dc] = shell_stats(d_); shutil.rmtree(d_, ignore_errors=True)
        say(f"reference shell (the layout's placeholder bay) at catch {dc:+.1f}: " + json.dumps(refs[dc]))
    W_REF, L_REF = refs[0.0]["width"], refs[0.0]["hull_L"]
    say("1. placement x slide x bay top, each pick's shell rebuilt (compact = rods <= 52, width <= %.0f, hull <= %.1f L):" % (W_REF + W_SLACK, L_REF + L_SLACK))
    DCS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0, 6.0, 8.0)
    G = {dc: grids(dc) for dc in DCS}
    cand = []
    for cap in CAPS:
        ZCAP = cap
        runs = {dc: search(dc, G[dc][1], G[dc][0], log=False) for dc in DCS}
        ok = {k: r for k, r in runs.items() if r}
        if not ok: say(f"   top {cap}: no feasible layout"); continue
        pdc = min(ok, key=lambda k: ok[k]["J"]); P_ = ok[pdc]; mp = mount(P_, pdc)
        d_ = tempfile.mkdtemp(); rebuild(P_, pdc, mp, d_); st = shell_stats(d_); M2 = rebuilt_model(d_, pdc)
        cg2 = cg_final(P_, pdc, mp, M2); sc = slide_scan(P_, pdc, mp, M2); bs = min(sc, key=lambda r: max(r[1], r[2]))
        comp = st["rods"] <= SHELL_CAP_REF and st["width"] <= W_REF + W_SLACK and st["hull_L"] <= L_REF + L_SLACK
        say(f"   top {cap}: DC {pdc:+.1f}, J {P_['J']:.2f}, CG (v10) {np.hypot(*P_['cin'][:2]):.2f} / {np.hypot(*P_['cout'][:2]):.2f}; bay x {P_['lo'][0]:.0f}..{P_['hi'][0]:.0f} "
            f"z {P_['lo'][2]:.0f}..{P_['hi'][2]:.0f}; rebuilt shell: offset {st['offset']:.0f}, {st['nodes']} nodes / {st['rods']} rods, width {st['width']:.0f}, "
            f"{st['hull_L']:.2f} L, longest {st['longest']:.0f}; CG there {cg2['hin']:.2f} / {cg2['hout']:.2f}, trim DC {bs[0]:+.2f}: {bs[1]:.2f} / {bs[2]:.2f}; "
            + ("COMPACT" if comp else "not compact"))
        cand.append(dict(cap=cap, dc=pdc, J=P_["J"], res=P_, mp=mp, dir=d_, st=st, cg=(cg2["hin"], cg2["hout"]), trim=bs, compact=comp))
    pool = [c for c in cand if c["compact"]]
    if pool:
        ch = min(pool, key=lambda c: (max(c["trim"][1], c["trim"][2], CG_FLOOR), c["J"]))
    else:
        ch = min(cand, key=lambda c: c["st"]["width"]); say("   NO bay top keeps the rebuilt shell compact: the narrowest is kept")
    say(f"   chosen: top {ch['cap']}, DC {ch['dc']:+.1f}")
    for c in cand:
        if c is not ch: shutil.rmtree(c["dir"], ignore_errors=True)
    # 2. the placement again on the chosen rebuilt shell, the final rebuild
    ZCAP = ch["cap"]; MODEL = rebuilt_model(ch["dir"], ch["dc"])
    say(f"2. placement x slide again on the chosen rebuilt shell ({len(MODEL['nodes'])} nodes, {len(MODEL['rods'])} rods), top {ZCAP}:")
    runs = {}
    for dc in sorted(set(ch["dc"] + k for k in (-1.0, 0.0, 1.0, 2.0))):
        O, F = grids(dc); runs[dc] = search(dc, F, O)
    ok = {k: r for k, r in runs.items() if r}; pick_dc = min(ok, key=lambda k: ok[k]["J"]); P_ = ok[pick_dc]
    MODEL = V10
    mp = mount(P_, pick_dc)
    out_dir = os.path.join(RUN, "rebuilt"); shutil.rmtree(out_dir, ignore_errors=True)
    rebuild(P_, pick_dc, mp, out_dir); stf = shell_stats(out_dir); M3 = rebuilt_model(out_dir, pick_dc)
    shutil.rmtree(ch["dir"], ignore_errors=True)
    say(f"   pick: DC {pick_dc:+.1f} (slide {DC_REF - pick_dc:.1f} forward of v10's {DC_REF}), J {P_['J']:.2f}; final shell: " + json.dumps(stf))
    for n, (o, t) in P_["poses"].items(): say("      " + describe(n, o, t))
    # 3. checks on the final rebuilt shell
    say("3. the carrier and the checks on the final rebuilt shell:")
    MODEL = M3
    for k, (m, g, c, note) in mp.items():
        say(f"   {k}: {g:.1f} g (E), CG ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), x {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, y {m.bounds[0][1]:.1f}..{m.bounds[1][1]:.1f}, "
            f"z {m.bounds[0][2]:.1f}..{m.bounds[1][2]:.1f}; {note}")
    say("   funnel line at this slide (C): from (%.1f, %.1f) to (%.1f, %.1f)" % funnel_line(pick_dc, M3))
    ck = checks(P_, pick_dc, mp, M3)
    for l in fmt_checks(ck): say(l)
    sw = sweep_out(P_, M3)
    for k, v in sw.items(): say(f"   tool Ø10, {k}: from {v['start']} along {v['dir']}, out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
    cg = cg_final(P_, pick_dc, mp, M3)
    say(f"   CG (true centroids, drawn carrier, final shell): pack in ({cg['cin'][0]:+.2f}, {cg['cin'][1]:+.2f}) = {cg['hin']:.2f} mm, pack out ({cg['cout'][0]:+.2f}, "
        f"{cg['cout'][1]:+.2f}) = {cg['hout']:.2f} mm (E, gate 3); {cg['mass']:.0f} g, {cg['mass_out']:.0f} g pack out; added {cg['add_g']:.1f} g")
    mom = sum(r[1] * r[3][0] for r in cg["rows"])
    v10m = sum(V10["P"][n]["g"] * centroid(V10["P"][n]["mesh"])[0][0] for n in MOVED)
    say(f"   tail bay x moment {mom:.0f} g mm ({cg['add_g']:.1f} g at x {mom / cg['add_g']:.1f}) vs v10's placeholder {v10m:.0f}")
    sc = slide_scan(P_, pick_dc, mp, M3); bs = min(sc, key=lambda r: max(r[1], r[2]))
    say(f"   slide trim: best DC {bs[0]:+.2f}: {bs[1]:.2f} / {bs[2]:.2f} mm")
    ld = loads(P_, pick_dc, mp)
    say(f"   loads (E): carried {ld['carried_g']} g at {ld['carried_cg']}; root section {ld['root_section']}; worst root index {ld['worst_root']:.2f}, "
        f"worst fuse (yoke tongues) index {ld['worst_fuse']:.2f}, fuse first in every case: {ld['fuse_first']}")
    for k, v in ld["cases"].items(): say(f"      {k}: " + json.dumps(v))
    r_sc, txt = run_system_check(manifest(P_, pick_dc, mp, M3), os.path.join(out_dir, "system_check.txt"))
    shutil.copy(os.path.join(out_dir, "system_check.txt"), os.path.join(RUN, "system_check.txt"))
    say("   " + txt.splitlines()[0]); say("   " + txt.splitlines()[-1][:600])
    # 4. alternatives (on v10's shell)
    MODEL = V10
    say("4. alternatives (v10 shell):")
    O, F = grids(DC_REF); Bres = search(DC_REF, F, O, label=f"B no slide, same carrier family, top {ZCAP}")
    mB = mount(Bres, DC_REF) if Bres else None; cgB = cg_final(Bres, DC_REF, mB) if Bres else None
    txtB = None
    if Bres:
        _, txtB = run_system_check(manifest(Bres, DC_REF, mB), os.path.join(RUN, "system_check_alt_B.txt"))
        say(f"   B drawn: CG {cgB['hin']:.2f} / {cgB['hout']:.2f} mm; " + txtB.splitlines()[0])
    o_, _ = P_["poses"]["Radxa Zero 3W"]; other = "edge" if abs(o_["face"][2]) > 0.9 else "flat"
    RADXA_MODE = other; dcE = min(DCS, key=lambda d: abs(d - pick_dc)); Eres = search(dcE, G[dcE][1], G[dcE][0], label=f"E Radxa {other} only, top {ZCAP}")
    RADXA_MODE = "any"
    ZCAP_used = ZCAP; ZCAP = None
    preview([(f"A (picked): box keel, top {ZCAP_used}, catch {pick_dc:+.1f} (v10 {DC_REF:+.1f}), shell rebuilt round it; CG {cg['hin']:.1f} / {cg['hout']:.1f} mm", P_, pick_dc, mp, M3)]
            + ([(f"B: no slide (catch {DC_REF:+.1f}, v10 shell), layout re-optimised at catch {DC_REF:.0f}, drawn carrier; CG {cgB['hin']:.1f} / {cgB['hout']:.1f} mm (gate 3)",
                 Bres, DC_REF, mB, V10)] if Bres else []), os.path.join(RUN, "preview.png"))
    for f in os.listdir(RUN):
        if f.endswith(".stl"): os.remove(os.path.join(RUN, f))
    for k, (m, g, c, note) in mp.items(): m.export(os.path.join(RUN, "".join(ch_ if ch_.isalnum() else "_" for ch_ in k.lower())[:40] + ".stl"))
    # 5. envelope and result
    env = dict(dc=pick_dc, bay_top=ZCAP_used, funnel_line=np.round(funnel_line(pick_dc, M3), 1).tolist(),
               note="drone frame (x nose, y left, z up, thrust centre x 0), catch set at x dc aft; boxes [[x0, y0, z0], [x1, y1, z1]]; the items and the "
                    "carrier ride the belly mount (move with the catch); masses E unless tagged; funnel = lowest point above the funnel line at this slide",
               parts={})
    for n, (o, t) in P_["poses"].items():
        env["parts"][n] = dict(box=np.round(np.array([np.min([b[0] for b in o["body"]], 0), np.max([b[1] for b in o["body"]], 0)]) + t, 1).tolist(),
                               board_box=np.round(o["body"][0] + t, 1).tolist(), mass_g=MASS[n][0], tag=MASS[n][1], cg=np.round(o["cg"] + t, 1).tolist(),
                               funnel_margin=round(ck[n]["funnel"], 1))
    for k, (m, g, c, note) in mp.items():
        env["parts"][k] = dict(box=np.round(m.bounds, 1).tolist(), mass_g=g, tag="E", cg=np.round(c, 1).tolist(), note=note,
                               funnel_margin=None if not np.isfinite(ck[k]["funnel"]) else round(ck[k]["funnel"], 1))
    cb = [(n, g, c) for n, g, c, _ in P_["cab"]]
    env["parts"]["tail cables (5 runs, at their midpoints)"] = dict(mass_g=round(sum(g for _, g, _ in cb), 2), tag="E",
                                                                    cg=np.round(sum(g * c for _, g, c in cb) / sum(g for _, g, _ in cb), 1).tolist())
    gs = sum(v["mass_g"] for v in env["parts"].values()); env["total_g"] = round(gs, 1)
    env["total_cg"] = np.round(sum(v["mass_g"] * np.array(v["cg"]) for v in env["parts"].values()) / gs, 1).tolist()
    env["x_moment_gmm"] = round(sum(v["mass_g"] * v["cg"][0] for v in env["parts"].values()), 0)
    json.dump(env, open(os.path.join(RUN, "envelope.json"), "w"), indent=1)
    def rj(r):
        if not r: return None
        return dict(dc=r["dc"], J=round(r["J"], 3), cg_in=np.round(r["cin"], 3).tolist(), cg_out=np.round(r["cout"], 3).tolist(), bay_cm3=round(r["V"], 1),
                    cable_g=round(r["cable_g"], 2), bay=dict(lo=np.round(r["lo"], 2).tolist(), hi=np.round(r["hi"], 2).tolist()), n_feasible=r["n_feasible"])
    out = dict(round=2, reference_shells=refs, ref_v10_bay=ref,
               cap_scan=[dict(cap=c["cap"], dc=c["dc"], J=round(c["J"], 3), shell=c["st"], cg_rebuilt=np.round(c["cg"], 3).tolist(),
                              trim=np.round(c["trim"], 3).tolist(), compact=c["compact"], bay=rj(c["res"])["bay"]) for c in cand],
               chosen_cap=ZCAP_used,
               pick=dict(**rj(P_), poses=pose_json(P_), cables=[dict(name=n, g=round(g, 2), mid=np.round(m, 2).tolist(), L=round(L, 0)) for n, g, m, L in P_["cab"]],
                         carrier={k: dict(g=g, cg=np.round(c, 2).tolist(), bounds=np.round(m.bounds, 2).tolist(), note=note) for k, (m, g, c, note) in mp.items()},
                         final_shell=stf, funnel_line=env["funnel_line"],
                         cg_final=dict(pack_in=np.round(cg["cin"], 3).tolist(), pack_out=np.round(cg["cout"], 3).tolist(), h_in=round(cg["hin"], 3), h_out=round(cg["hout"], 3),
                                       mass=round(cg["mass"], 1), mass_out=round(cg["mass_out"], 1), added_g=round(cg["add_g"], 2), bay_cg=np.round(cg["bay_cg"], 2).tolist(),
                                       rows=[dict(name=n, g=round(g, 2), tag=t, cg=np.round(c, 2).tolist()) for n, g, t, c in cg["rows"]]),
                         bay_moment=round(mom, 0), v10_bay_moment=round(v10m, 0), slide_scan=sc, checks=ck, sweeps=sw, loads=ld,
                         anchors={k: (np.round(v, 2).tolist() if isinstance(v, np.ndarray) else round(v, 2)) for k, v in anchors(pick_dc).items()},
                         system_check=txt.splitlines()[-1][:800]),
               alt_B_no_slide=dict(**rj(Bres), poses=pose_json(Bres), drawn_cg=dict(h_in=round(cgB["hin"], 3), h_out=round(cgB["hout"], 3)),
                                   system_check=txtB.splitlines()[-1][:600]) if Bres else None,
               alt_E_other_radxa=rj(Eres), weights=dict(W_CG=W_CG, W_V=W_V, W_C=W_C, CG_FLOOR=CG_FLOOR, NEAR=NEAR), dc_ref=DC_REF)
    json.dump(out, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())


# ================================================================ round 3 (27 Oct): the mount on the v11 poses
# Spark asked the Scout v3 design session to move the tail bay out of the prop wash and centre the CG; its layout v11 (PR 133) now
# sets the Radxa and UBEC poses as the system's requirement. This pass draws the mount for them on v11's cage (no pose optimisation:
# the round-1/2 search above stays for the record), places the XR2, and checks everything on v11's own model.
# ================================================================ round 4 (27 Oct 2026): the mount on scout_layout v12
# v12 (PR 141, cage session): the catch 5.0 forward of the thrust centre; the Radxa, the UBEC, the mount and the cables ride the belly
# mount with the catch (8.5 forward of v11, nothing changes relative to the receiver); the XR2 is ours to place (v12 held it at v11's
# frame place). Review FAIL of round 3 (2ef0b72b) fixed here: a real yoke back, a one-body root bridged to the plate bolts, the plate
# bolts off the FC USB-C path (the layout's own path), sections and modes from mesh slices, every printed piece one body.
V12 = os.path.join(ROOT, "runs", "scout_layout", "v12")   # scout_layout v12 (PR 141, on main); system model nox/scout/v3/scout.assembly.json
DC_V12 = -json.load(open(os.path.join(V12, "result.json")))["tab_x"]   # -5.0: the catch 5.0 forward (C)
TOP_TILT = (0.0, 0.259, 0.966)                  # a ball-end key 15 deg toward +y (kept for the key search)
PLUG_TOP = 45.0                                 # the bay's height cap at the centre strip (the cage session's request, 27 Oct): low 90 deg
                                                # USB-C plugs along the board's top edge, body 39.5..45 (E, unmeasured), cable forward
RADXA_ZTOP = 38.0                               # the Radxa's top edge in the layout (C; checked against the mesh)
RADXA_POSE = {}                                 # from the layout's Radxa mesh: board X0 (SD end), ports edge (Y0) up, PCB top plane
USB1_DIR = None                                 # the USB-C 1 tool line, set by run_v12
FC_USB_FROM = None                              # round 3's override of the FC USB-C path start: removed (review 27 Oct), the layout's own path rules
# --- the joint and the root (round 4)
BLK_LEN = 21.4                                  # the block 21.4 long (was 22): its tail 2.0 ahead of the Radxa's front posts (review 27 Oct)
BOLT_DX = (6.5, 15.5)                           # yoke M3 from the block's front face (C): pads 2.5 off the TPU end pieces, 9 apart (cbore wall 3)
BLOCK_Y = (-4.55, 6.5)                          # the block beside the yoke's side wall (C)
YB = 0.5                                        # yoke bolt line y (C): insert walls 3.05 / 4.0 in the block, pads inside the tongue
SIDE_Y = (-6.55, -4.55)                         # the yoke's back = its side wall = the spine's inner wall, 2.0 (C): wash +3.05 at x -54.1
SPINE_Y = (-13.05, -4.55)                       # the spine's box behind the wash line (C)
TRAY_Z = (13.6, 16.0)                           # the UBEC tray and the cross plate behind the Radxa: under the UBEC (z 16, C)
RIB_Z0 = 2.0                                    # the UBEC rib's bottom (C): over the funnel line by 17 there
LOW_AFT = float(os.environ.get("ES_LOWAFT", 20.0))   # the lower back runs 20 aft of the wash line under the Radxa, into the spine tube (C)
LOW_TOP = 6.0                                   # the yoke's lower back under the Radxa (board bottom z 8 - 2)
PLATE_BOLTS = [(-42.0, 16.0), (-42.0, -16.0)]   # 2 M3 down through the plate tab into inserts in the block's web (DEFAULT): 8.0 from the
                                                # tab's tail edge (x -50), 7.0 from its notch edge (y 9), 11.5 from the FC USB-C path axis (need 11);
                                                # the key (ball-end 25 deg toward the centreline) clears the swept prop disk by 0.2
PLATE_TAIL = -50.0                              # the plate tab's tail edge (C, plate mesh)
WEB_X0 = -49.5                                  # the web under the plate tab (C)
WEB_Y = 16.0                                    # the arms' half width (to the boss axes; the bosses round them), C: clear of the FC USB-C path
WEB_X1 = -31.0                                  # the web runs forward under the plate to here (C): crosses |y| 6.6..9 in the plate's shadow;
                                                # requirement: the belly mount's tail face at x -31.0 over |y| <= 22 (10.85 ahead of the layout's box)
FOOT_X1, FOOT_T, FOOT_Y = -14.0, 4.0, 8.0      # the fin's foot on the plate top, a wedge from FOOT_TOP at the fin to 4 tall at x -14,
                                                # |y| <= 8.0 (the wash corridor there >= 8.08) (C): the tail-up bearing
FOOT_TOP = float(os.environ.get("ES_FOOT", 45.0))    # the foot's height at the fin (<= the 45 cap)
SIDE_AFT = (-7.15, -5.15)                       # beside the Radxa the side wall steps out: 2.0 off the layout's Radxa box (y -3.15), C
POST_FACE = SIDE_AFT[1]                         # the posts run from the wall (y -5.15) to the PCB's back (y -1.6): 3.55 long
NOTCH_Y = 9.0                                   # the plate's tail notch half width (C, plate mesh)
SPINE_SKIN = os.environ.get("ES_SKIN", "1") == "1"   # the spine behind the wash line: a closed tube (1) or flanges only (0)
BLOCK_SOLID = os.environ.get("ES_SOLID", "1") == "1" # the tail block printed solid (1) or walls 2.4 + 20 % infill (0)
WALL = 2.4                                      # printed walls (E): sections are taken hollow (infill ignored), masses as printed
INFILL = 0.2                                    # mass: walls 2.4 + 20 % infill (E), the interior found by eroding the mesh 2.4
E_PETG, G_PETG = 2000.0, 770.0                  # MPa (E)
PULL_INS = TAU_PULL * math.pi * INSERT[0] * (INSERT[1] - 1.0)     # M3 insert pull-out of PETG, 15 MPa over Ø4 x 5.7 = 1074 N (E)
SHEAR_INS = SIG_ALLOW * INSERT[0] * (INSERT[1] - 1.0)             # M3 insert lateral bearing in PETG, 25 MPa x 4 x 5.7 = 570 N (E)
PULL_HEAD = TAU_PULL * math.pi * 5.5 * 2.0                        # M3 head through the 2.0 web: 518 N (E)
BEAR_TONGUE = SIG_ALLOW * 3.0 * TONGUE                            # M3 bearing in the 5.0 tongue: 375 N (E)


SPINE_TUBE = float(os.environ.get("ES_TUBE", 20.0))  # the spine's closed box: a tube 20 tall along its lower edge (DEFAULT; 14: -3 g, the carrier
                                                     # then within 1.5 x of the fuse at 50 N y; 0 = full height, +8 g)
UBOX = float(os.environ.get("ES_UBOX", 99.0))        # the UBEC torsion box's depth under the tray (default: down to the funnel / inset floor)
PIECES = {}                                 # the carrier's pieces before the union (for the mass breakdown)


def printed_g(m, infill=INFILL, pitch=0.4):
    """mass of a printed piece as printed (E): the mesh's volume within 2.4 of its surface solid, the interior (the mesh eroded 2.4 on a
    0.4 grid) at the infill; infill=1.0 = printed solid"""
    V = abs(m.volume)
    if infill >= 1.0: return round(V * PETG, 2)
    vx = m.voxelized(pitch=pitch).fill(); A = vx.matrix
    k = int(round(WALL / pitch)); ball = np.linalg.norm(np.mgrid[-k:k + 1, -k:k + 1, -k:k + 1], axis=0) <= k + 1e-6
    inner = ndimage.binary_erosion(np.pad(A, 1), structure=ball).sum() * pitch ** 3
    inner = min(inner, V)
    return round(((V - inner) + infill * inner) * PETG, 2)


def z_floor(x, y, dc, M=None):
    """the lowest z at (x, y) that is >= 1.5 over the funnel line and >= 3.5 inside the shell (rod r + 3 + 0.5)"""
    M = M or MODEL
    if "_eq" not in M: M["_eq"] = ConvexHull(M["nodes"]).equations
    eq = M["_eq"]
    for z in np.arange(-50.0, 45.0, 0.5):
        p = np.array([x, y, z])
        if z < funnel_z(np.array([x]), dc, M)[0] + 1.5: continue
        if (-(eq[:, :3] @ p + eq[:, 3])).min() - ROD_R >= 3.5: return float(z)
    return 45.0


def notch_front(M=None):
    """the plate's tail notch front (x of the plate outline where |y| < 8, aft of x -30) and the tab tail edge"""
    M = M or MODEL
    if "_notch" not in M:
        s = M["P"]["frame plate (Bee35 carbon)"]["mesh"].section(plane_origin=[0, 0, 20.0], plane_normal=[0, 0, 1])
        V = np.vstack([np.asarray(d) for d in s.discrete]); sel = V[(np.abs(V[:, 1]) < 8.0) & (V[:, 0] < -30) & (V[:, 0] > -60)]
        M["_notch"] = float(sel[:, 0].max())
    return M["_notch"]


def corridor(x):
    """the least |y| at x where a point is >= WASH outside the rear disks (the wash corridor's half width; inf where no disk)"""
    dx = abs(x + 54.1); r = DISK_R + WASH
    return 54.1 - math.sqrt(r * r - dx * dx) if dx < r else np.inf


def bay_poses():
    """the Radxa and the UBEC in the layout's poses as item orientations (true sizes: Radxa 65 x 30 x 6.31 body; the UBEC body 45 x 15.4
    x 8 inside the layout's 61 box, its leads 8 out each end)"""
    br = MODEL["P"]["Radxa Zero 3W"]["mesh"].bounds; bu_ = MODEL["P"]["UBEC"]["mesh"].bounds
    RADXA_POSE.update(x0=float(br[0][0]), ztop=float(br[1][2]), ypcb=float(br[1][1]) - 3.16)
    assert abs(RADXA_POSE["ztop"] - RADXA_ZTOP) < 0.5
    Rr = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)     # board X -> +x (SD end aft), Y -> -z (ports edge up), Z -> +y
    rb = ITEMS["Radxa Zero 3W"]["body"]
    ITEMS["Radxa Zero 3W"] = dict(ITEMS["Radxa Zero 3W"], body=[rb[0], B(37.1, 68.5, -(PLUG_TOP - RADXA_ZTOP), -1.5, -4.0, 6.0)])
    o = orient("Radxa Zero 3W", Rr); t = np.array([RADXA_POSE["x0"], RADXA_POSE["ypcb"], RADXA_POSE["ztop"]]) + o["o"]
    L_, W_, H_ = (bu_[1] - bu_[0]).tolist(); lead = 8.0 if L_ > 55 else 0.0; L_ -= 2 * lead
    ITEMS["UBEC"] = dict(ITEMS["UBEC"], body=[B(0, L_, 0, W_, 0, H_), B(-8, 0, 2, W_ - 2, 0, H_), B(L_, L_ + 8, 2, W_ - 2, 0, H_)],
                         cg=np.array([L_ / 2, W_ / 2, H_ / 2]), pts={"end0": np.array([-4.0, W_ / 2, H_ / 2]), "end1": np.array([L_ + 4.0, W_ / 2, H_ / 2])})
    ou = orient("UBEC", np.eye(3)); bu = ou["body"][0]
    tu = np.array([bu_[1][0] - lead - bu[1][0], bu_[0][1], bu_[0][2]])
    return {"Radxa Zero 3W": (o, t), "UBEC": (ou, tu)}


def keepclear_plug(o, t):
    """the permanent camera plug on USB-C 2 and the service plug on USB-C 1: low 90 deg plugs along the top edge, over the ports'
    overhang (z 39.5) to the 45 cap (E, not measured)"""
    lo = o["R"] @ np.array([37.1, 0, 0]) - o["o"] + t; hi = o["R"] @ np.array([58.5 + 10, 0, 0]) - o["o"] + t
    return np.array([[min(lo[0], hi[0]), -4.0, RADXA_POSE["ztop"] + 1.5], [max(lo[0], hi[0]), 6.0, PLUG_TOP]])


def radxa_posts(o, t):
    """the 4 Ø8 posts (screw-only, Ø2.1 pilots) from the spine's side wall (0.45 into it) to the PCB's back (board Z -1.6)"""
    R = o["R"]; out = []
    for hx, hy in RADXA_HOLES:
        a = R @ np.array([hx, hy, POST_FACE - 0.45 - RADXA_POSE["ypcb"]]) - o["o"] + t; b = R @ np.array([hx, hy, -1.6]) - o["o"] + t
        out.append((a, b))
    return out


def radxa_envelope(o, t, plugs=True):
    """the Radxa for the fit table: the layout's box (65 x 6.31 x 30: PCB top at y 0, components to +3.15, back side to -3.15, H) with
    the ports' overhang to z 39.5, less the 4 post keep-outs (Ø8.4, back side only: the posts land on the PCB's back at y -1.6), and
    (plugs=True) the low plugs' keep-clear"""
    br = MODEL["P"]["Radxa Zero 3W"]["mesh"].bounds; kp = keepclear_plug(o, t)
    env = union([box(br[0], br[1]), box([kp[0][0], br[0][1], br[1][2] - 0.1], [min(kp[1][0], br[1][0]), br[1][1], br[1][2] + 1.5])])
    R = o["R"]; cuts = []
    for hx, hy in RADXA_HOLES:
        c = R @ np.array([hx, hy, 0.0]) - o["o"] + t
        cuts.append(cyl([c[0], br[0][1] - 0.5, c[2]], [c[0], -1.6, c[2]], 4.2, 24))
    env = diff(env, cuts)
    return union([env, box(kp[0], kp[1])]) if plugs else env


def mount_v12(P, dc, xr2=None, strut=False):
    """concept A: two printed pieces.
    belly-mount tail block (root): the block between the yoke's tongues, a gusset bearing on the receiver's tail face, a web under the
    plate tab out to 2 plate-bolt bosses (M3 down through the plate), the fin up through the plate's tail notch and its foot on the
    plate top (the tail-up bearing).
    tail carrier: the yoke (2 tongues on 4 M3 = the fuse, the side wall = the yoke's back, the lower back under the Radxa), the spine
    (2.0 inner wall + a box behind the wash line) with the Radxa's 4 posts, the cross plate behind the Radxa into the UBEC tray, the XR2
    shelf. strut=True (concept B): a printed strut from the spine's top to the cage's tail point (z 41.5, under the cap), clipped (fuse)."""
    import shapely.geometry as sg
    an = anchors(dc); xb0, xb1 = an["xb0"], an["xb1"]; z0, z1 = BLK_Z; T = TONGUE; nf = notch_front()
    bolts = [xb0 - d for d in BOLT_DX]; parts = {}
    # ---- the root: belly-mount tail block
    rt, bt = an["rec_top"], an["belly_tail"]
    corr = min(corridor(xb0 + GAP), corridor(an["rec_tail"])) - 0.05
    fpg = sg.Polygon([(bt - 0.5, 22.0), (bt - 0.5, FOOT_TOP), (FOOT_X1, 22.0 + FOOT_T), (FOOT_X1, 22.0)])
    foot = trimesh.creation.extrude_polygon(fpg, 2 * FOOT_Y)                         # the foot, a wedge on the plate top (x, z) to the cap, |y| <= 8.5
    foot.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, -FOOT_Y], [0, 1, 0, 0], [0, 0, 0, 1.0]]))
    if foot.volume < 0: foot.invert()
    blk = union([box([xb1, BLOCK_Y[0], z0], [xb0, BLOCK_Y[1], z1]),
                 box([xb0 - 2.0, BLOCK_Y[0], an["tpu_top"] + GAP], [an["rec_tail"], BLOCK_Y[1], rt]),          # gusset: bears on the receiver's tail
                 box([xb0 - 2.0, BLOCK_Y[0], rt], [WEB_X1, BLOCK_Y[1], 18.5]),                                 # web core (the block's y band), on under the plate
                 box([xb0 + GAP, -BLK_HALF, rt], [WEB_X1, BLOCK_Y[0] + 1.0, 18.5]),
                 box([nf + 0.3, -NOTCH_Y - 1.0, rt], [WEB_X1, NOTCH_Y + 1.0, 18.5]),                            # web across under the plate, ahead of the notch
                 box([xb0 + GAP, NOTCH_Y + 0.3, rt], [WEB_X1, WEB_Y, 18.5]),                                     # arms under the tab to the bosses
                 box([xb0 + GAP, NOTCH_Y + 0.3, an["tpu_top"] + GAP], [an["rec_tail"], WEB_Y, rt]),               # deepened behind the receiver
                 box([xb0 + GAP, -WEB_Y, an["tpu_top"] + GAP], [an["rec_tail"], -NOTCH_Y - 0.3, rt]),
                 box([xb0 + GAP, -WEB_Y, rt], [-37.0, -NOTCH_Y - 0.3, 18.5]),                                     # (-y: 1.7 off the FC USB-C path box)
                 box([xb0 + GAP, -corr, an["tpu_top"] + GAP], [an["rec_tail"], corr, rt]),                      # gusset's outer bearing on the receiver
                 box([xb0 + GAP, BLOCK_Y[0], 18.0], [bt, BLOCK_Y[1], 22.0 + FOOT_T]),                            # fin up through the notch (|y| <= 6.5), 2.0 ahead of the tongue
                 box([xb0 + GAP, -BLK_HALF, 18.0], [bt, BLOCK_Y[0] + 1.0, 22.0 + FOOT_T])]
                + [foot]
                + [cyl([px, py, rt], [px, py, 18.5], INSERT[2] / 2) for px, py in PLATE_BOLTS])
    holes = []
    for x in bolts:
        holes += [cyl([x, YB, z1 - INSERT[1]], [x, YB, z1 + 0.1], INSERT[0] / 2), cyl([x, YB, z0 - 0.1], [x, YB, z0 + INSERT[1]], INSERT[0] / 2)]
    holes += [cyl([px, py, 18.5 - INSERT[1]], [px, py, 18.6], INSERT[0] / 2) for px, py in PLATE_BOLTS]
    blk = diff(blk, holes)
    parts["belly-mount tail block (PETG)"] = (blk, printed_g(blk, 1.0 if BLOCK_SOLID else INFILL), "block 21.4 x 11.05 between the tongues, gusset on the receiver's tail face, web under the "
                                              f"plate tab to 2 plate-bolt bosses at {PLATE_BOLTS}, fin through the notch, foot on the plate top to x {FOOT_X1}; 6 inserts")
    # ---- the carrier
    o, t = P["Radxa Zero 3W"]; bR = o["body"][0] + t
    ou, tu = P["UBEC"]; bU = ou["body"][0] + tu
    xa = bR[0][0]; ztop = RADXA_POSE["ztop"] - 0.5                                # 2.0 under the plugs' keep-clear (z 39.5)
    x_step = xb1                                                                        # the side wall steps out at the block's tail (2.0 off the Radxa)
    xw = -54.1 - math.sqrt((DISK_R + WASH) ** 2 - (54.1 - abs(SPINE_Y[0])) ** 2) - 0.5   # aft of this the box spine is out of the wash
    pts = np.array([[xb0, z0 - T], [xb0, z1 + T], [bR[1][0], ztop], [xa, ztop], [xa, bR[0][2]], [bU[0][0], TRAY_Z[0]], [bU[0][0], TRAY_Z[1]]])
    pg = sg.Polygon(pts[ConvexHull(pts).vertices])
    R = o["R"]
    for nm, d in (("sd", R @ [-1.0, 0, 0]), ("usb1", R @ [0, -1.0, 0])):
        p0 = o["pts"][nm] + t
        if abs(d[1]) < 0.5 and p0[1] - TOOL / 2 - 1.0 < SIDE_AFT[1]:
            q = p0 + 400 * d; w = TOOL / 2 + 1.0; n_ = np.array([-d[2], d[0]])
            pg = pg.difference(sg.Polygon([p0[[0, 2]] + w * n_, q[[0, 2]] + w * n_, q[[0, 2]] - w * n_, p0[[0, 2]] - w * n_]))
    if pg.geom_type != "Polygon": pg = max(pg.geoms, key=lambda g_: g_.area)
    def extrude_xz(poly, y0, y1):
        m = trimesh.creation.extrude_polygon(poly, y1 - y0)
        m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0], [0, 0, 0, 1.0]]))
        if m.volume < 0: m.invert()
        return m
    pf = pg.intersection(sg.box(x_step, -1000, 1000, 1000)); pa_ = pg.intersection(sg.box(-1000, -1000, x_step, 1000))
    side = union([extrude_xz(pf, SIDE_Y[0], SIDE_Y[1]), extrude_xz(pa_, SIDE_AFT[0], SIDE_AFT[1])])   # the yoke's back / the spine's side wall
    pgo = pg.intersection(sg.box(-1000, -1000, xw, 1000))
    if pgo.geom_type != "Polygon": pgo = max(pgo.geoms, key=lambda g_: g_.area)
    if SPINE_SKIN:                                                                      # the box spine behind the wash line: a closed tube
        import shapely.affinity as sa                                                   # along the spine's lower edge, SPINE_TUBE tall
        band = pgo.difference(sa.translate(pgo, yoff=SPINE_TUBE)) if SPINE_TUBE else pgo
        band = max(band.geoms, key=lambda g_: g_.area) if band.geom_type != "Polygon" else band
        outer = extrude_xz(band, SPINE_Y[0], SIDE_AFT[0] + 0.5)
    else:                                                                               # its flanges: the outline's 2.4 rim, 6.5 out (an open C)
        rim = pgo.difference(pgo.buffer(-2.4))
        rim = max(rim.geoms, key=lambda g_: g_.area) if rim.geom_type != "Polygon" else rim
        outer = extrude_xz(rim, SPINE_Y[0], SIDE_AFT[0] + 0.5)
    tong_y = (SIDE_Y[0], BLOCK_Y[1])
    x_lb = xw - LOW_AFT
    yoke = [box([xb1, tong_y[0], z1], [xb0, tong_y[1], z1 + T]),                          # upper tongue
            box([x_lb, tong_y[0], z0 - T], [xb0, tong_y[1], z0]),                         # lower tongue, on aft under the lower back
            box([x_lb, tong_y[0], z0 - T], [xb1, tong_y[1], LOW_TOP])]                    # lower back under the Radxa
    posts = [cyl(a, b, SO_D / 2, 24).intersection(box([-1000, -1000, -1000], [1000, 1000, ztop])) for a, b in radxa_posts(o, t)]
    bUf = bU[1][0] - 2.0                                                              # the UBEC rib under the Radxa, near the UBEC's front
    rib = [box([bUf - 4.0, SPINE_Y[0], RIB_Z0], [bUf, bU[1][1], LOW_TOP]), box([bUf - 4.0, bU[0][1], RIB_Z0], [bUf, bU[1][1], TRAY_Z[0] + 0.5])]
    cross = box([bU[0][0], SPINE_Y[0], TRAY_Z[0]], [xa - GAP, bU[1][1], TRAY_Z[1]])
    tray = box([bU[0][0], bU[0][1], TRAY_Z[0]], [bU[1][0], bU[1][1], TRAY_Z[1]])
    xr_ = np.linspace(bU[0][0], bUf, 24)
    zb = np.array([max(RIB_Z0, TRAY_Z[0] - UBOX, z_floor(x_, bU[1][1], dc), z_floor(x_, bU[0][1], dc)) for x_ in xr_])   # over the funnel line + 1.5, inset >= 3.5
    rpg = sg.Polygon(list(zip(xr_, zb)) + [(bUf, TRAY_Z[0] + 0.5), (bU[0][0], TRAY_Z[0] + 0.5)])
    tbox = [extrude_xz(rpg, bU[0][1], bU[1][1])]      # the UBEC's torsion box under its tray: a solid prism printed with 2.4 walls + infill
    aft = box([bU[0][0], SPINE_Y[0], float(zb[0])], [bU[0][0] + 2.4, bU[1][1], TRAY_Z[0] + 0.5])     # and its aft wall across to the spine
    pieces = [side, outer] + yoke + posts + [cross, tray, aft] + tbox + rib
    PIECES.clear(); PIECES.update(side=side, spine_box=outer, yoke=union(yoke), posts=union(posts), cross=cross, tray=tray, aft=aft, ubec_box=tbox[0], rib=union(rib))
    if xr2 is not None:
        ox, tx = xr2; b = ox["body"][0] + tx
        zf = (b[0][2] - 2.0, b[0][2])                                                    # the floor under the XR2 (z 21..23)
        floor = box([b[0][0], b[0][1], zf[0]], [b[1][0], b[1][1], zf[1]])
        xm = (b[0][0] + b[1][0]) / 2
        pieces += [floor, box([xm - 3, b[1][1] - 1.0, zf[0] - 12.0], [xm + 3, SIDE_AFT[0] + 0.5, zf[1]])]   # 6 x 14 post down into the spine tube
    pieces.append(box([xb1, SIDE_Y[1] - 0.5, z1 + T], [xb0, -1.0, z1 + T + 6.5]))        # fillet: the upper tongue onto the side wall over 3.5 x 6.5
    xc_ = np.linspace(xb0, xw, 40); yo_ = [max(SPINE_Y[0], -(corridor(x_) - 0.05)) for x_ in xc_]
    chk = trimesh.creation.extrude_polygon(sg.Polygon(list(zip(xc_, yo_)) + [(xw, SIDE_AFT[1]), (x_step, SIDE_AFT[1]), (x_step, SIDE_Y[1]),
                                                             (xb0, SIDE_Y[1])]), (z1 + T + 6.5) - (z0 - T))
    chk.apply_translation([0, 0, z0 - T]); pieces.append(chk)                           # the cheek: the side wall thickened out to the wash line
    car = union(pieces)
    cb = [cyl([x, YB, z1 + T - 0.1], [x, YB, z1 + T + 7.0], 3.2) for x in bolts]          # Ø6.4 head / key clearance through the fillet
    for x in bolts:
        cb += [cyl([x, YB, z1 - 0.1], [x, YB, z1 + T + 0.1], 1.7), cyl([x, YB, z1 + T - 3.0], [x, YB, z1 + T + 0.1], CBORE_D / 2),
               cyl([x, YB, z0 - T - 0.1], [x, YB, z0 + 0.1], 1.7), cyl([x, YB, z0 - T - 0.1], [x, YB, z0 - T + 3.0], CBORE_D / 2)]
    for a, b in radxa_posts(o, t):
        cb.append(cyl(a + (a - b) * 0.9, b + (b - a) * 0.02, SO_PILOT / 2, 12))
    car = diff(car, cb)
    parts["tail carrier (PETG)"] = (car, printed_g(car), "yoke (2 tongues, 4 M3, 2.0 side wall, lower back under the Radxa) + spine (2.0 inner wall, "
                                    f"box y {SPINE_Y[0]}..{SIDE_Y[0]} aft of x {xw:.1f}) + 4 Ø8 posts + cross plate + UBEC tray"
                                    + (" + XR2 shelf" if xr2 is not None else "") + "; one print")
    if strut:
        free = MODEL["nodes"][MODEL["nring"]:]; node = free[np.argmin(free[:, 0])]          # the tail point (lowest node aft, under the cap)
        top = np.array([xa + 6.0, (SPINE_Y[0] + SIDE_Y[0]) / 2, ztop - 3.0])
        end = node - (node - top) / np.linalg.norm(node - top) * (NODE_R + 1.0)
        st = union([cyl(top, end, 3.0, 16), cyl(end - [0, 0, 4.0], end + [0, 0, 4.0], 4.0, 16)])
        parts["strut to the tail point (PETG, B)"] = (st, round(printed_g(st) + 1.0, 2), f"Ø6 printed strut to the tail point {np.round(node, 1).tolist()}, "
                                                      "snap clip on its rod beside the node (3.2 bore) = the fuse (+1 g clip, E)")
    for k, (m, g, note) in parts.items():
        parts[k] = (m, g, centroid(m)[0], note)
    return parts


def place_xr2(P, dc, mparts):
    """the XR2 (antenna up) on a shelf on the spine's outer wall: every lattice pose clear of everything (+2), out of the wash, inside
    the shell inset, above the funnel line; the one with the smallest x moment (most forward), then the lowest"""
    O, (F, F2) = grids(dc)
    X = np.zeros_like(O)
    for n, m in {**item_meshes(dict(poses=P)), **{k: v[0] for k, v in mparts.items()}}.items(): vox_mesh(m, X)
    o, t = P["Radxa Zero 3W"]; kp = keepclear_plug(o, t); kb = box(kp[0], kp[1]); vox_mesh(kb, X)
    ball = np.linalg.norm(np.mgrid[-2:3, -2:3, -2:3], axis=0) <= GAP + 0.01
    IF = integral(F2 | ndimage.binary_dilation(X, structure=ball))
    best = None
    for R in ROTS:
        if (R @ [0, 0, 1.0])[2] < 0.9: continue                          # antenna up
        ox = orient("XR2", R); bb = ox["body"]
        for yhi in np.arange(SPINE_Y[0] - GAP, SPINE_Y[0] - GAP - 12.01, -1.0):
            xs = np.arange(-185.0, -60.0, 1.0); zs = np.arange(-20.0, 46.0, 1.0)
            T = np.stack(np.meshgrid(xs, [0.0], zs, indexing="ij"), -1).reshape(-1, 3); T[:, 1] = yhi - bb[0][1][1]
            ok = np.ones(len(T), bool)
            for b in bb: ok &= box_sum(IF, T + b[0], T + b[1]) == 0
            ok &= (T[:, 2] + bb[1][1][2] <= PLUG_TOP)
            if not ok.any(): continue
            Tk = T[ok]; b0 = bb[0]
            wl = np.min([wash_dist(Tk[:, 0] + b0[i][0], Tk[:, 1] + b0[j][1]) for i in (0, 1) for j in (0, 1)], axis=0)
            Tk = Tk[wl >= WASH + 0.5]
            if not len(Tk): continue
            c = Tk + ox["cg"]; key = np.lexsort((-c[:, 2], -c[:, 0])); q = key[0]
            cand = (-c[q][0], -c[q][2], ox, Tk[q])
            if best is None or cand[:2] < best[:2]: best = cand
    return None if best is None else (best[2], best[3])


# ---------------------------------------------------------------- sections from the meshes
def _ring(c):
    x, y = c[:-1, 0], c[:-1, 1]; x1, y1 = c[1:, 0], c[1:, 1]; cr = x * y1 - x1 * y
    return cr.sum() / 2, ((x + x1) * cr).sum() / 6, ((y + y1) * cr).sum() / 6, ((y * y + y * y1 + y1 * y1) * cr).sum() / 12, ((x * x + x * x1 + x1 * x1) * cr).sum() / 12


def poly_props(geoms):
    """A, centroid (u, v), Iuu = int v^2, Ivv = int u^2 about the centroid, c_u, c_v (extreme fibres), J (Bredt for hollow pieces,
    A^4 / (4 pi^2 Ip) for solid ones), t (wall / thickness for the torsion stress)"""
    import shapely.geometry.polygon as sp
    A = Su = Sv = Iu = Iv = 0.0; pts = []
    for g in geoms:
        g = sp.orient(g, 1.0)
        for ring_ in [g.exterior] + list(g.interiors):
            a, sx, sy, ixx, iyy = _ring(np.asarray(ring_.coords)); A += a; Su += sx; Sv += sy; Iu += ixx; Iv += iyy
        pts.append(np.asarray(g.exterior.coords))
    if A <= 1e-6: return None
    cu, cv = Su / A, Sv / A; Iuu = Iu - A * cv * cv; Ivv = Iv - A * cu * cu
    P_ = np.vstack(pts); c_u = float(np.abs(P_[:, 0] - cu).max()); c_v = float(np.abs(P_[:, 1] - cv).max())
    return dict(A=A, cu=cu, cv=cv, Iuu=Iuu, Ivv=Ivv, c_u=c_u, c_v=c_v)


def section(meshes, origin, n, u, hollow_=True):
    """the hollow (2.4 walls; hollow_=False: solid, the block) section of the meshes cut by the plane (origin, n), axes u, v = n x u"""
    import shapely.geometry as sg, shapely.ops as so
    n = np.asarray(n, float); u = np.asarray(u, float); v = np.cross(n, u)
    solids = []
    for m in meshes:
        s = m.section(plane_origin=origin, plane_normal=n)
        if s is None: continue
        loops = [np.c_[(np.asarray(d) - origin) @ u, (np.asarray(d) - origin) @ v] for d in s.discrete]
        polys = [sg.Polygon(l_).buffer(0) for l_ in loops if len(l_) >= 4]
        if not polys: continue
        acc = None                                      # even-odd: a loop inside another is a hole
        for p_ in sorted(polys, key=lambda p_: -p_.area):
            acc = p_ if acc is None else acc.symmetric_difference(p_)
        solids.append(acc)
    if not solids: return None
    full = so.unary_union(solids)
    geoms = [full] if full.geom_type == "Polygon" else list(full.geoms)
    hollow = []; J = 0.0; tmax = 0.0
    for g in geoms:
        if g.area < 0.5: continue
        inner = g.buffer(-WALL) if hollow_ else sg.Polygon()
        if (inner.is_empty or inner.area < 1.0) and len(g.interiors):        # a closed thin-walled cell: Bredt on its midline
            hollow.append(g); ext = sg.Polygon(g.exterior).area; hol = max(sg.Polygon(i).area for i in g.interiors)
            sm = (g.exterior.length + max(i.length for i in g.interiors)) / 2; t_ = g.area / sm
            J += 4 * ((ext + hol) / 2) ** 2 * t_ / sm; tmax = max(tmax, t_)
        elif inner.is_empty or inner.area < 1.0:
            hollow.append(g); pr = poly_props([g]); Ip = pr["Iuu"] + pr["Ivv"]
            rr = np.asarray(g.minimum_rotated_rectangle.exterior.coords); t_ = float(np.linalg.norm(np.diff(rr, axis=0), axis=1).min())
            J += pr["A"] ** 4 / (4 * math.pi ** 2 * Ip); tmax = max(tmax, t_)
        else:
            h = g.difference(inner); hollow += [h] if h.geom_type == "Polygon" else list(h.geoms)
            mid = g.buffer(-WALL / 2); J += 4 * mid.area ** 2 * WALL / mid.length; tmax = max(tmax, WALL)
    if not hollow: return None
    pr = poly_props(hollow); pr.update(J=J, t=max(tmax, WALL), u=u, v=v, n=n, origin=np.asarray(origin, float),
                                       c3=np.asarray(origin, float) + pr["cu"] * u + pr["cv"] * v, solid_A=full.area)
    return pr


def sec_index(pr, F, Mo):
    """von Mises index at a section: F, Mo the load on the free side about the section's centroid (global axes)"""
    N = F @ pr["n"]; V = np.linalg.norm(F - N * pr["n"])
    Mt = Mo @ pr["n"]; Mu = Mo @ pr["u"]; Mv = Mo @ pr["v"]
    sig = abs(Mu) * pr["c_v"] / pr["Iuu"] + abs(Mv) * pr["c_u"] / pr["Ivv"] + abs(N) / pr["A"]
    tau = abs(Mt) * pr["t"] / pr["J"] + 1.5 * V / pr["A"]
    return math.sqrt(sig ** 2 + 3 * tau ** 2) / SIG_ALLOW, sig, tau


def mass_points(res, mparts, which, half_cables=True):
    """(g, point) rows of the mass the named mount pieces carry: items, cables (half), and each mount piece's volume as 400 points"""
    rows = [(MASS[n][0], o["cg"] + t) for n, (o, t) in res["poses"].items()]
    rows += [(g * (0.5 if half_cables else 1.0), c) for _, g, c, _ in res["cab"]]
    for k, (m, g, c, _) in mparts.items():
        if k not in which: continue
        np.random.seed(7)                                                            # seeded: one run gives one set of numbers
        p = trimesh.sample.volume_mesh(m, 4000)[:400] if m.is_watertight else np.zeros((0, 3))
        if len(p) < 50: p = trimesh.sample.sample_surface(m, 400, seed=3)[0]
        rows += [(g / len(p), q) for q in p]
    return rows


def _l_cases():
    """dimensions.md L: crash 35 g along the 6 face and 8 corner directions (limit 1.0), flight 4 g along the thrust axis and 2 g
    lateral (0.5), landing 6 g (0.5), each at the CG of every mass it moves; plus (reported, not an L case) 50 N at the bay's far item
    corner, a hand on the bay through a cage cell (E)"""
    out = []
    for nm, d in (("x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("y", (0, 1, 0)), ("-y", (0, -1, 0)), ("z", (0, 0, 1)), ("-z", (0, 0, -1))):
        out.append((f"crash 35 g {nm}", np.array(d, float), 35.0, "crash"))
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                out.append((f"crash 35 g corner ({'+' if sx > 0 else '-'}x {'+' if sy > 0 else '-'}y {'+' if sz > 0 else '-'}z)",
                            np.array([sx, sy, sz], float) / math.sqrt(3), 35.0, "crash"))
    out += [("flight 4 g (thrust axis)", np.array([0, 0, -1.0]), 4.0, "flight")]
    out += [(f"flight 2 g {nm}", np.array(d, float), 2.0, "flight") for nm, d in (("x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("y", (0, 1, 0)), ("-y", (0, -1, 0)))]
    out += [("landing 6 g", np.array([0, 0, -1.0]), 6.0, "landing")]
    out += [(f"E: 50 N {nm} at the far item corner", np.array(d, float), 50.0, "E") for nm, d in (("y", (0, 1, 0)), ("z", (0, 0, 1)), ("-z", (0, 0, -1)))]
    return out


LOAD_CASES = _l_cases()
LIMIT = {"crash": 1.0, "flight": 0.5, "landing": 0.5, "E": 1.0}


def case_loads(rows, far, case, pick):
    """the load (list of (f, point)) of a case on the rows that pick(point) keeps"""
    name, d, mag, kind = case
    if kind != "E":
        return [(g * mag * 9.81 / 1000 * d, p) for g, p in rows if pick(p)]
    return [(mag * d, far)] if pick(far) else []


def resultant(fl, about):
    F = sum((f for f, _ in fl), np.zeros(3)); Mo = sum((np.cross(p - about, f) for f, p in fl), np.zeros(3))
    return F, Mo


def loads_mesh(res, dc, mparts):
    """sections from the meshes along the load path (x planes through the carrier, the yoke, the block, the fin and web; y planes
    through the web's arms), the yoke's 4 M3 (the fuse) and the 2 plate bolts (the root's fasteners), every case. Index = stress /
    allowable (E); the fuse must reach 1.5 x every other index first."""
    an = anchors(dc); z0, z1 = BLK_Z
    CAR = [k for k in mparts if k.startswith("tail carrier")][0]; BLK = "belly-mount tail block (PETG)"
    car, blk = mparts[CAR][0], mparts[BLK][0]
    rows_car = mass_points(res, mparts, [CAR]); rows_all = mass_points(res, mparts, [CAR, BLK])
    lo, hi = res["lo"], res["hi"]
    bolts = [an["xb0"] - d for d in BOLT_DX]; jc = np.array([np.mean(bolts), YB, (z0 + z1) / 2])
    cor = np.array([[b[i][0], b[j][1], b[k][2]] for o, t in res["poses"].values() for b in [o["body"][0] + t] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    far = cor[np.argmax(np.linalg.norm(cor - jc, axis=1))]          # the item body corner farthest from the yoke (the UBEC's aft outer top)
    xs_car = np.arange(an["xb1"] - 1.0, res["lo"][0] + 8.0, -3.0)
    xs_blk = np.arange(an["xb0"] - 0.5, PLATE_BOLTS[0][0] - 0.4, 1.0)              # aft of the plate bolts (the support)
    secs = {}
    for x in xs_car: secs[("carrier", round(float(x), 1))] = section([car], np.array([x, 0, 0.0]), [-1.0, 0, 0], [0, 1.0, 0])
    for x in np.arange(an["xb1"] + 1.0, an["xb0"] - 0.5, 2.0):
        secs[("yoke + block (clamped)", round(float(x), 1))] = section([car, blk], np.array([x, 0, 0.0]), [-1.0, 0, 0], [0, 1.0, 0])
    for x in xs_blk: secs[("block / web / fin", round(float(x), 1))] = section([blk], np.array([x, 0, 0.0]), [-1.0, 0, 0], [0, 1.0, 0], not BLOCK_SOLID)
    arms = {}
    for px, py in PLATE_BOLTS:
        for yy in np.arange(BLK_HALF + 0.5, abs(py) - INSERT[2] / 2 + 0.01, 1.0):
            s_ = section([blk], np.array([0, math.copysign(yy, py), 0.0]), [0, -math.copysign(1.0, py), 0], [1.0, 0, 0], not BLOCK_SOLID)
            if s_ is not None: arms[(py, round(float(yy), 1))] = s_
    feet = {}
    for xf in np.arange(an["belly_tail"] + 0.5, FOOT_X1 - 3.0, 1.5):
        feet[round(float(xf), 1)] = section([blk], np.array([xf, 0, 0.0]), [1.0, 0, 0], [0, 1.0, 0], not BLOCK_SOLID)
    cases = {}
    for case in LOAD_CASES:
        name = case[0]; out = dict(kind=case[3], limit=LIMIT[case[3]])
        worst = []
        for (grp, x), pr in secs.items():
            if pr is None: continue
            fl = case_loads(rows_car if grp == "carrier" else rows_all, far, case, lambda p, x=x: p[0] < x)
            F, Mo = resultant(fl, pr["c3"]); ix, sg_, ta = sec_index(pr, F, Mo)
            worst.append((ix, grp, x, sg_, ta, pr["A"]))
        for grp in ("carrier", "yoke + block (clamped)", "block / web / fin"):
            w = [r for r in worst if r[1] == grp]
            if w:
                r = max(w); out[grp] = dict(index=round(r[0], 3), x=r[2], sigma=round(r[3], 1), tau=round(r[4], 1), A=round(r[5], 0))
        # the yoke's 4 M3 (the fuse): everything the carrier carries, about the bolt group's centre
        F, Mo = resultant(case_loads(rows_car, far, case, lambda p: True), jc)
        # the tongues sit on the block's top and bottom faces: F z, pitch and roll go in bearing (tongue on block, 21.4 x 11 faces);
        # the 4 bolts carry x, y and yaw in shear, bearing in the tongues (the fuse; the inserts, 570 N, hold past 375 N)
        xarm = BOLT_DX[1] - BOLT_DX[0]; Lb = BLK_LEN; Wb = BLOCK_Y[1] - BLOCK_Y[0]
        Sb = abs(Mo[2]) / xarm / 2 + np.hypot(F[0], F[1]) / 4
        fuse = Sb / BEAR_TONGUE
        Rb = abs(Mo[1]) / (Lb * 2 / 3) + abs(F[2]) + abs(Mo[0]) / (Wb * 2 / 3)          # the largest bearing resultant on one tongue face
        pb = Rb / (Lb * Wb / 4) / SIG_ALLOW                                               # over a quarter of the face (E)
        out["fuse (yoke 4 M3)"] = dict(S=round(Sb, 1), index=round(fuse, 3), mode="tongue bearing at the bolts", face_bearing_index=round(pb, 3))
        # the 2 plate bolts: everything (carrier + block) about the bolt line's centre at the plate underside
        pc = np.array([PLATE_BOLTS[0][0], 0.0, 18.5]); F, Mo = resultant(case_loads(rows_all, far, case, lambda p: True), pc)
        span = abs(PLATE_BOLTS[0][1] - PLATE_BOLTS[1][1]); zr = an["rec_top"] / 2 + (an["tpu_top"] + GAP) / 2      # receiver bearing centroid z
        arm_down = 20.25 - zr                                          # tail-down: bolt shear at the plate vs the receiver bearing
        Tp = (max(-F[2], 0.0)) / 2 + abs(Mo[0]) / span
        Sp = np.hypot(min(F[0], 0.0), F[1]) / 2 + abs(Mo[2]) / span + (max(-Mo[1], 0.0) / arm_down / 2)
        root_b = max(Tp / PULL_INS, Sp / SHEAR_INS)
        rb = max(-Mo[1], 0.0) / arm_down / ((BLOCK_Y[1] - BLOCK_Y[0]) * (an["rec_top"] - an["tpu_top"] - GAP)) / SIG_ALLOW
        tail_up = max(Mo[1], 0.0) / (FOOT_X1 + 2.0 - PLATE_BOLTS[0][0])      # foot on the plate top vs the web under the tab (plate in bending)
        out["plate bolts (2 M3)"] = dict(T=round(Tp, 1), S=round(Sp, 1), index=round(root_b, 3), receiver_bearing_index=round(rb, 3),
                                          tail_up_couple_N=round(tail_up, 1))
        # the foot: the tail-up bearing on the plate top at its tip, back to the fin (x planes)
        if tail_up > 0:
            fo = []
            for xf in np.arange(an["belly_tail"] + 0.5, FOOT_X1 - 3.0, 1.5):
                pr = feet.get(round(float(xf), 1))
                if pr is None: continue
                Ff = np.array([0, 0, tail_up]); pf = np.array([FOOT_X1 + 2.0, 0, 22.0])
                ix, sg_, ta = sec_index(pr, Ff, np.cross(pf - pr["c3"], Ff)); fo.append((ix, xf))
            if fo: r = max(fo); out["foot"] = dict(index=round(r[0], 3), x=round(r[1], 1), F=round(tail_up, 1))
        # the web's arms: each bolt's reaction back to the fin (y planes)
        wa = []
        for (py, yy), pr in arms.items():
            Fb = np.array([np.sign(-F[0]) * Sp, 0, Tp]); pb = np.array([PLATE_BOLTS[0][0], py, 18.5])
            ix, sg_, ta = sec_index(pr, Fb, np.cross(pb - pr["c3"], Fb)); wa.append((ix, py, yy))
        if wa:
            r = max(wa); out["web arms"] = dict(index=round(r[0], 3), y=round(math.copysign(r[2], r[1]), 1))
        root_ix = max(v["index"] for k, v in out.items() if isinstance(v, dict) and not k.startswith("fuse") and k != "carrier")
        car_ix = out.get("carrier", {}).get("index", 0.0)
        cand = {"fuse (yoke bolts)": out["fuse (yoke 4 M3)"]["index"], "tail block (root)": round(root_ix, 3), "carrier": car_ix}
        first = max(cand, key=cand.get)
        out["order"] = dict(fuse=cand["fuse (yoke bolts)"], root=cand["tail block (root)"], carrier=car_ix, first=first,
                            fuse_first=bool(first != "carrier" and cand[first] >= 1.5 * car_ix))   # a replaceable piece goes first, 1.5 x before the carrier
        cases[name] = out
    # least-I sections per group: vertical bending (Iy = int z^2) and lateral bending (Iz = int y^2) listed apart
    weakest = {}
    for grp in ("carrier", "yoke + block (clamped)", "block / web / fin"):
        cand = [(k[1], pr) for k, pr in secs.items() if k[0] == grp and pr is not None]
        if cand:
            for lab, key in (("vertical (least Iy)", "Iuu"), ("lateral (least Iz)", "Ivv")):
                x, pr = min(cand, key=lambda r: r[1][key])
                weakest[f"{grp}, {lab}"] = dict(x=x, A=round(pr["A"], 0), Iy=round(pr["Iuu"], 0), Iz=round(pr["Ivv"], 0), J=round(pr["J"], 0))
    summ = {}
    for kind in ("crash", "flight", "landing", "E"):
        cs = {k: v for k, v in cases.items() if v["kind"] == kind}
        summ[kind] = dict(fuse=max(v["order"]["fuse"] for v in cs.values()), root=max(v["order"]["root"] for v in cs.values()),
                          carrier=max(v["order"]["carrier"] for v in cs.values()), limit=LIMIT[kind],
                          worst=max(max(v["order"]["fuse"], v["order"]["root"], v["order"]["carrier"]) for v in cs.values()),
                          order_ok=all(v["order"]["fuse_first"] for v in cs.values()),
                          order_fails=[k for k, v in cs.items() if not v["order"]["fuse_first"]])
        summ[kind]["pass"] = bool(summ[kind]["worst"] <= LIMIT[kind])
    return dict(far_corner=np.round(far, 1).tolist(), yoke_centre=np.round(jc, 1).tolist(), cases=cases, weakest_sections=weakest, by_kind=summ,
                worst_root=summ["crash"]["root"], worst_fuse=summ["crash"]["fuse"], worst_carrier=summ["crash"]["carrier"],
                fuse_first=summ["crash"]["order_ok"], l_pass=all(summ[k]["pass"] for k in ("crash", "flight", "landing")), secs=secs)


def modes_mesh(res, dc, mparts, ld, strut=None):
    """first lateral and vertical modes (E): the bay's carried mass at its CG on the mount as a beam from the plate bolts aft, EI and GJ
    from the x-plane sections (hollow, 2.4 walls, PETG E 2000, G 770); B: the strut as a spring at its spine end (axial EA / L along its
    line, in series with the tail point's own 100 N/mm, E), the propped beam solved by flexibilities"""
    an = anchors(dc); CAR = [k for k in mparts if k.startswith("tail carrier")][0]; BLK = "belly-mount tail block (PETG)"
    rows = mass_points(res, mparts, [CAR]); mc = sum(g for g, _ in rows); cc = sum(g * p for g, p in rows) / mc
    x_end = min(cc[0], strut[0][0]) if strut is not None else cc[0]
    xs = []; EIv = []; EIl = []; GJ = []; zc = []
    for x in np.arange(PLATE_BOLTS[0][0], x_end - 1.0, -1.0):
        ms = [mparts[BLK][0]] if x > an["xb0"] else ([mparts[CAR][0], mparts[BLK][0]] if x > an["xb1"] else [mparts[CAR][0]])
        pr = section(ms, np.array([x, 0, 0.0]), [-1.0, 0, 0], [0, 1.0, 0], x <= an["xb0"] or not BLOCK_SOLID)
        if pr is None: continue
        xs.append(x); EIv.append(E_PETG * pr["Iuu"]); EIl.append(E_PETG * pr["Ivv"]); GJ.append(G_PETG * pr["J"]); zc.append(pr["c3"][2])
    xs = np.array(xs); EIv = np.array(EIv); EIl = np.array(EIl); GJ = np.array(GJ); zc = np.array(zc)
    def flex(EI, a, b):                                            # deflection at a under a unit load at b (cantilever from the root)
        k_ = xs > max(a, b); return float(np.sum((xs[k_] - a) * (xs[k_] - b) / EI[k_]))
    out = {}
    for nm, EI, ax in (("lateral", EIl, 1), ("vertical", EIv, 2)):
        f_cc = flex(EI, cc[0], cc[0])
        if ax == 1: f_cc += float(np.sum(((cc[2] - zc) ** 2 / GJ)[xs > cc[0]]))
        if strut is not None:
            p, u_, EA_L = strut
            kp = 1.0 / (1.0 / max(EA_L * u_[ax] ** 2, 1e-6) + 1.0 / 100.0)
            f_cc = f_cc - flex(EI, cc[0], p[0]) ** 2 / (flex(EI, p[0], p[0]) + 1.0 / kp)
        out[nm] = round(math.sqrt(1000.0 / f_cc / (mc / 1000)) / (2 * math.pi), 0)
    out.update(carried_g=round(mc, 1), carried_cg=np.round(cc, 1).tolist(), root_x=PLATE_BOLTS[0][0])
    return out


def one_body(m):
    return len(m.split(only_watertight=False))


def key_dir(p, cands, M=None, L=150.0, r=2.0, max_tilt=25.0):
    """the key direction from p (a Ø4 hex key, a ball-end up to 25 deg) that clears every rod and node by 0.5 and every prop mesh
    (the layout's swept disks, the meshes system_check sweeps against) by 0: the listed directions first, then the cone round the
    first one, least tilt first; else the best found. Returns (dir, clearance to rods / nodes, clearance to the props)"""
    M = M or MODEL; rods = [(M["nodes"][a], M["nodes"][b]) for a, b in M["rods"]] + list(M["links"].values())
    if "_propq" not in M:
        M["_propq"] = [trimesh.proximity.ProximityQuery(M["P"][f"prop {i}"]["mesh"]) for i in range(1, 5) if f"prop {i}" in M["P"]]
    def clear(d):
        s_ = np.linspace(0, L, 301); Q = p + s_[:, None] * d; cl = np.inf
        for a, b in rods:
            u = b - a; tt = np.clip(((Q - a) @ u) / (u @ u), 0, 1); cl = min(cl, float(np.linalg.norm(Q - (a + tt[:, None] * u), axis=1).min()) - ROD_R - r)
        cl = min(cl, float(np.linalg.norm(Q[:, None] - M["nodes"][None], axis=2).min()) - NODE_R - r)
        Qp = Q[(Q[:, 2] > 25.0) & (Q[:, 2] < 60.0)]
        cp = min((float((-q.signed_distance(Qp)).min()) - r for q in M["_propq"]), default=np.inf) if len(Qp) else np.inf
        return cl, cp
    best = None; d0 = np.asarray(cands[0], float); d0 /= np.linalg.norm(d0)
    e1 = np.cross(d0, [1.0, 0, 0] if abs(d0[0]) < 0.9 else [0, 1.0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(d0, e1)
    cone = [d0 * math.cos(math.radians(tl)) + math.sin(math.radians(tl)) * (math.cos(math.radians(az)) * e1 + math.sin(math.radians(az)) * e2)
            for tl in np.arange(5.0, max_tilt + 0.1, 5.0) for az in range(0, 360, 15)]
    for d in [np.asarray(c, float) / np.linalg.norm(c) for c in cands] + cone:
        cl, cp = clear(d)
        if cl >= 0.5 and cp >= 0.0: return d, cl, cp
        if best is None or min(cl - 0.5, cp) > min(best[1] - 0.5, best[2]): best = (d, cl, cp)
    return best


def manifest_v12(res, dc, mparts, M=None):
    """the Scout v3 system model at this catch with the three items as placed and the mount's printed pieces (the layout's placeholder
    boxes for the mount dropped), the layout's FC USB-C path as drawn, the Radxa's SD / USB-C sweeps, every M3 key"""
    M = M or MODEL; A = json.loads(json.dumps(M["A"])); sh = -(dc - M["dc_ref"])
    parts = []
    for p in A["parts"]:
        if p["name"] in MOVED or p["name"] in PLACEHOLDERS: continue
        q = dict(p)
        if "mesh" in q and not os.path.isabs(q["mesh"]): q["mesh"] = os.path.join(ROOT, q["mesh"])
        if p["name"] in CATCH and sh != 0:
            if "box" in q: x0, x1, y0, y1, z0, z1 = q["box"]; q["box"] = [x0 + sh, x1 + sh, y0, y1, z0, z1]
            else: q["pose"] = {"t": [sh, 0, 0]}
        if "cg" not in q and "mesh" in q and p["name"] in M["P"]:
            q["cg"] = np.round(M["P"][p["name"]]["c"], 3).tolist()
        parts.append(q)
    for n, m in item_meshes(res).items():
        if n == "Radxa Zero 3W": m = radxa_envelope(*res["poses"][n])                    # the layout's box less the post keep-outs, + the plugs
        parts.append(dict(name=n, role="bought", protect=True, trimesh=m, mass_g=MASS[n][0], tag=MASS[n][1], geom_tag="C"))
    for n, (m, g, c, note) in mparts.items():
        parts.append(dict(name=n, role="structure", trimesh=m, mass_g=g, tag="E", material="PETG", geom_tag="C", cg=np.round(c, 3).tolist()))
    for q in parts:                                                                     # the belly mount cut to its requirement (x -31.0)
        if q["name"] == "mount + nose legs (Scout)": q.pop("mesh", None); q["trimesh"] = M["P"][q["name"]]["mesh"]
    cab = res["cab"]
    parts.append(dict(name="tail cables (5 runs)", role="structure", box=list(np.repeat(np.average([c for _, _, c, _ in cab], axis=0, weights=[g for _, g, _, _ in cab]), 2) + [-0.5, 0.5] * 3),
                      mass_g=round(sum(g for _, g, _, _ in cab), 2), tag="E"))
    A["parts"] = parts
    acc = []
    for a in A["access"]:
        a = dict(a)
        if "from" in a and a["part"] in CATCH: a["from"] = (np.array(a["from"]) + [sh, 0, 0]).tolist()
        a["ignore"] = list(a.get("ignore", [])) + ["tail cables (5 runs)"]
        acc.append(a)
    o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]
    acc.append(dict(name="Radxa microSD out + Ø10 fingers", part="Radxa Zero 3W", **{"from": (o["pts"]["sd"] + t).tolist()}, dir=(R @ [-1.0, 0, 0]).tolist(),
                    length_mm=150.0, tool={"type": "cylinder", "d": TOOL}, ignore=["tail cables (5 runs)"]))
    acc.append(dict(name="Radxa USB-C 1 plug + Ø10", part="Radxa Zero 3W", **{"from": (o["pts"]["usb1"] + t).tolist()},
                    dir=(R @ [0, -1.0, 0]).tolist() if USB1_DIR is None else list(USB1_DIR), length_mm=150.0, tool={"type": "cylinder", "d": TOOL},
                    ignore=["tail cables (5 runs)"]))
    an = anchors(dc); z0, z1 = BLK_Z; CAR = [k for k in mparts if k.startswith("tail carrier")][0]
    swap = [n for n in CATCH if M["P"].get(n, {}).get("swap")]
    keys = []
    for x in [an["xb0"] - d for d in BOLT_DX]:
        p = np.array([x, YB, z1 + TONGUE]); d, cl, cp = key_dir(p, [[0, 0, 1], TOP_TILT, [0, -0.259, 0.966], [-0.259, 0, 0.966]], M)
        keys.append((f"M3 key, yoke top bolt (x {x:.1f})", CAR, p, d, min(cl, cp)))
        p = np.array([x, YB, z0 - TONGUE]); d, cl, cp = key_dir(p, [[0, 0, -1], [-0.259, 0, -0.966], [-0.423, 0, -0.906], [0, 0.259, -0.966]], M)
        keys.append((f"M3 key, yoke bottom bolt (x {x:.1f})", CAR, p, d, min(cl, cp)))
    for px, py in PLATE_BOLTS:
        p = np.array([px, py, 22.5]); d, cl, cp = key_dir(p, [[0, 0, 1], [0, -np.sign(py) * 0.423, 0.906], [0.259, 0, 0.966]], M)
        keys.append((f"M3 key, plate bolt ({px:.1f}, {py:+.1f})", "belly-mount tail block (PETG)", p, d, min(cl, cp)))
    for nm, part, p, d, cl in keys:
        tilt = math.degrees(math.acos(min(1.0, abs(d[2]))))
        acc.append(dict(name=nm + (f", ball-end {tilt:.0f} deg" if tilt > 1 else ""), part=part, **{"from": p.tolist()}, dir=np.round(d, 3).tolist(),
                        length_mm=150.0, tool={"type": "cylinder", "d": 4}, ignore=["tail cables (5 runs)"] + (swap if d[2] < 0 else [])))
    A["access"] = acc
    cp = [c for c in A["contact_pairs"] if not (set(c[:2]) & (set(MOVED) | set(PLACEHOLDERS)))]
    cp += [[a_, b_, CONTACT_MM3] for a_, b_ in declared_contacts(mparts)]                 # face-to-face bearing only: overlap <= 0.05 mm3
    cp += [["tail cables (5 runs)", n, 1.0] for n in [p["name"] for p in parts] if n != "tail cables (5 runs)"]   # a 1 mm3 mass marker
    A["contact_pairs"] = cp
    A["clearance_pairs"] = [[k, n, GAP] for k in mparts for n in [p["name"] for p in parts]
                            if n != k and n != "tail cables (5 runs)" and frozenset((k, n)) not in {frozenset(c) for c in declared_contacts(mparts)}
                            and near(mparts[k][0], part_mesh(n, parts, M))]
    A["name"] = "Scout v3 with the tail electronics mount (system pass v2, round 5, layout v12)"
    return A, keys


CAGE_CRASH_WORST = 0.59                         # the cage's worst crash index (rod buckling, frame model v2 alt_sized: the 6/4 DEFAULT cage, F on E)
CONTACT_MM3 = 0.05                              # a declared bearing contact: face to face, overlap <= 0.05 mm3 (review 27 Oct)


def declared_contacts(mparts):
    """the mount's bearing faces: gusset on the receiver's tail and web on its top, block under and foot on the plate, web against the
    belly mount's (cut) tail face, the tongues and side wall on the block, the posts on the Radxa's PCB back, UBEC and XR2 on their floors"""
    BLK = "belly-mount tail block (PETG)"; CAR = [k for k in mparts if k.startswith("tail carrier")][0]
    out = [(BLK, "receiver v3 (latch)"), (BLK, "frame plate (Bee35 carbon)"), (BLK, "mount + nose legs (Scout)"), (CAR, BLK),
           (CAR, "Radxa Zero 3W"), (CAR, "UBEC"), (CAR, "XR2")]
    out += [(k, CAR) for k in mparts if k.startswith("strut")]
    return out


def near(a, b, d=10.0):
    return not (np.any(a.bounds[0] > b.bounds[1] + d) or np.any(b.bounds[0] > a.bounds[1] + d))


def part_mesh(n, parts, M=None):
    M = M or MODEL
    for q in parts:
        if q["name"] != n: continue
        if "trimesh" in q: return q["trimesh"]
        if n in M["P"]: return M["P"][n]["mesh"]
        if "box" in q: x0, x1, y0, y1, z0, z1 = q["box"]; return box([x0, y0, z0], [x1, y1, z1])
    return box([1e4] * 3, [1e4 + 1] * 3)


def overlap_mm3(a, b):
    try:
        v = trimesh.boolean.intersection([a, b], engine="manifold")
        return float(abs(v.volume)) if v is not None and len(v.faces) else 0.0
    except Exception:
        return float("nan")


def fit_table(res, mparts, dc, M=None):
    """the full pairwise fit table of the mount pieces against every part within 10 (items as drawn, the Radxa as the layout's box with
    the post keep-outs, the plugs' keep-clear apart): overlap volume, least gap, and the verdict: a declared bearing contact passes at
    overlap <= 0.05 mm3, every other pair needs >= 2.0. The carrier against the Radxa is also listed less its 4 posts (>= 2.0)."""
    M = M or MODEL; out = []
    o, t = res["poses"]["Radxa Zero 3W"]; kp = keepclear_plug(o, t)
    items = item_meshes(res); items["Radxa Zero 3W"] = radxa_envelope(o, t, plugs=False); items["Radxa plug keep-clear (E)"] = box(kp[0], kp[1])
    others = {n: at_dc(n, dc, M) for n in M["P"] if n not in MOVED and n not in PLACEHOLDERS}
    decl = {frozenset(c) for c in declared_contacts(mparts)}
    pool = {**others, **items, **{k: v[0] for k, v in mparts.items()}}
    done = set()
    for k, (m, g, c, _) in mparts.items():
        Q = surf(m, 8000)
        for n, om in pool.items():
            if n == k or frozenset((k, n)) in done or not near(m, om): continue
            done.add(frozenset((k, n)))
            v = overlap_mm3(m, om) if om.is_watertight and m.is_watertight else float("nan")
            Qo = surf(om, 8000)
            gap_ = min(float(-trimesh.proximity.ProximityQuery(om).signed_distance(Q).max()),
                       float(-trimesh.proximity.ProximityQuery(m).signed_distance(Qo).max()))
            dc_ = frozenset((k, n if n != "Radxa plug keep-clear (E)" else "-")) in decl
            ok = (v <= CONTACT_MM3 if np.isfinite(v) else gap_ >= -0.05) if dc_ else (gap_ >= GAP - 0.01)
            out.append(dict(a=k, b=n, overlap_mm3=round(v, 3) if np.isfinite(v) else None, gap_mm=round(gap_, 2),
                            kind="declared bearing contact" if dc_ else ">= 2.0", ok=bool(ok)))
    CAR = [k for k in mparts if k.startswith("tail carrier")][0]
    Q = surf(mparts[CAR][0], 20000); ax_ = [(a_, b_) for a_, b_ in radxa_posts(o, t)]
    keep = np.ones(len(Q), bool)
    for a_, b_ in ax_:
        u = (b_ - a_) / np.linalg.norm(b_ - a_); d_ = Q - a_; s_ = d_ @ u
        keep &= ~((np.linalg.norm(d_ - s_[:, None] * u, axis=1) < SO_D / 2 + 0.3) & (s_ > -0.5) & (s_ < np.linalg.norm(b_ - a_) + 0.5))
    g2 = float(-trimesh.proximity.ProximityQuery(items["Radxa Zero 3W"]).signed_distance(Q[keep]).max())
    out.append(dict(a=CAR + " less its 4 posts", b="Radxa Zero 3W", overlap_mm3=None, gap_mm=round(g2, 2), kind=">= 2.0", ok=bool(g2 >= GAP - 0.01)))
    return sorted(out, key=lambda r: (r["ok"], r["gap_mm"]))


def fit_pairs(res, mparts, dc, M=None):
    """the old short list (kept for --keel callers): the least gaps"""
    return [(r["a"], r["b"], r["gap_mm"]) for r in fit_table(res, mparts, dc, M)]


def slide_wide(res, dc, mparts, ride=True, M=None):
    """CG pack in / out against the catch slide over -14..+6 of the layout's: everything of ours rides the belly mount with the catch"""
    M = M or MODEL; out = []
    for d in np.arange(dc - 14.0, dc + 6.01, 0.25):
        sh = np.array([-(d - dc), 0, 0]); m_in, Min, m_sw, Msw = base_moments(d, M)
        rows = [(MASS[n][0], o["cg"] + t + (sh if ride else 0)) for n, (o, t) in res["poses"].items()]
        rows += [(g, c + (sh if (ride or k.startswith("belly")) else 0)) for k, (m, g, c, _) in mparts.items()]
        rows += [(g, c + sh / 2 * ride) for _, g, c, _ in res["cab"]]
        g = sum(r[0] for r in rows); Mm = sum(r[0] * r[1] for r in rows)
        cin = (Min + Mm) / (m_in + g); cout = (Min - Msw + Mm) / (m_in - m_sw + g)
        out.append((round(float(d), 2), float(np.hypot(*cin[:2])), float(np.hypot(*cout[:2])), float(cin[0]), float(cout[0])))
    return out


def cg_riding(res, dc, mparts, d, M=None):
    """CG pack in / out with the catch at d and everything of ours riding the belly mount with it (as slide_wide)"""
    M = M or MODEL; sh = np.array([-(d - dc), 0, 0]); m_in, Min, m_sw, Msw = base_moments(d, M)
    rows = [(MASS[n][0], o["cg"] + t + sh) for n, (o, t) in res["poses"].items()] + [(g, c + sh) for k, (m, g, c, _) in mparts.items()]
    rows += [(g, c + sh / 2) for _, g, c, _ in res["cab"]]
    g = sum(r[0] for r in rows); Mm = sum(r[0] * r[1] for r in rows)
    return dict(cin=(Min + Mm) / (m_in + g), cout=(Min - Msw + Mm) / (m_in - m_sw + g), mass=m_in + g, mass_out=m_in - m_sw + g)


def ubec_region(res, dc, mparts, d=None):
    """the UBEC poses that hold balance (<= 3 pack in and out), the funnel line (>= 0 under the body and leads) and the wash (>= 3)
    together, everything else fixed, with the catch at d (default the layout's) and the bay riding it: x / y / z shift windows"""
    d = dc if d is None else d; cg = cg_riding(res, dc, mparts, d); s = -(d - dc)
    ou, tu = res["poses"]["UBEC"]; bb = np.array([np.min([b[0] for b in ou["body"]], 0), np.max([b[1] for b in ou["body"]], 0)]) + tu
    mt = cg["mass"]; mto = cg["mass_out"]; g = MASS["UBEC"][0]; ok = []
    for dx in np.arange(-40.0, 40.01, 1.0):
        for dz in np.arange(-12.0, 20.01, 1.0):
            for dy in np.arange(0.0, 20.01, 2.0):
                ci = cg["cin"] + np.array([g * dx / mt, g * dy / mt, 0]); co = cg["cout"] + np.array([g * dx / mto, g * dy / mto, 0])
                if max(np.hypot(*ci[:2]), np.hypot(*co[:2])) > 3.0: continue
                b0 = bb + [dx, dy, dz]
                fz = b0[0][2] - funnel_z(np.array([b0[0][0], b0[1][0]]), dc)       # the funnel line rides with the bay
                if np.isfinite(fz).any() and fz[np.isfinite(fz)].min() < 0: continue
                w = min(wash_dist(x_ + s, y_) for x_ in (b0[0][0], b0[1][0]) for y_ in (b0[0][1], b0[1][1]))
                if w < WASH: continue
                ok.append((dx, dy, dz))
    out = dict(catch=round(float(d), 2), cg_now=[round(float(np.hypot(*cg["cin"][:2])), 2), round(float(np.hypot(*cg["cout"][:2])), 2)],
               box_now=np.round(bb + [[s, 0, 0], [s, 0, 0]], 1).tolist(), n=len(ok))
    if not ok: return out
    A_ = np.array(ok)
    out.update(shift_x=[float(A_[:, 0].min()), float(A_[:, 0].max())], shift_y=[float(A_[:, 1].min()), float(A_[:, 1].max())],
               shift_z_min_at_dx0=float(A_[A_[:, 0] == 0][:, 2].min()) if (A_[:, 0] == 0).any() else None)
    return out


def run_v12():
    global MODEL, USB1_DIR
    t0 = time.time()
    np.random.seed(0)
    MODEL = load_model(ASM, V12); MODEL["dc_ref"] = DC_V12
    bm = MODEL["P"]["mount + nose legs (Scout)"]                                          # the belly mount placeholder cut to the requirement:
    bm["mesh"] = diff(bm["mesh"], [box([-300.0, -22.0, 0.0], [WEB_X1, 22.0, 30.0])])     # its tail face at x -31.0 over |y| <= 22
    dc = DC_V12
    fl = funnel_line(dc)
    say("Scout v3 electronics mount, round 5 (27 Oct 2026): nox/scout/v3/electronics_system.py on scout_layout v12 (PR 141, on main), "
        "the Radxa and UBEC in v12's poses (riding the catch); the review FAIL of round 4 fixed; the belly mount placeholder cut to x -31.0 over |y| <= 22 "
        "(the requirement)")
    say(f"model: {len(MODEL['P'])} parts in pose (the layout's 3 placeholder boxes for our mount dropped), catch {dc:+.1f} (5.0 forward); shell "
        f"{len(MODEL['nodes'])} nodes / {len(MODEL['rods'])} rods + {len(MODEL['links'])} links; funnel line from v12's ring tail node "
        f"(x {MODEL['nodes'][:MODEL['nring']][:, 0].min():.1f}, z {MODEL['nodes'][:MODEL['nring']][:, 2].mean():.1f}) and tail point, both - offsets + 3: "
        "(%.1f, %.1f) -> (%.1f, %.1f)" % fl + f"; plate notch front x {notch_front():.1f}")
    P = bay_poses()
    for n, (o, t) in P.items(): say("   " + describe(n, o, t))
    o, t = P["Radxa Zero 3W"]; up = o["R"] @ np.array([0, -1.0, 0]); tilts = []
    for ang in range(0, 41, 5):
        for sd in (1, -1):
            d = np.array([0.0, sd * math.sin(math.radians(ang)), math.cos(math.radians(ang))]) if ang else up
            USB1_DIR = d; v = sweep_out(dict(poses=P))["USB-C 1"]; tilts.append((ang, sd, v["least_clear_mm"]))
            if v["least_clear_mm"] >= TOOL / 2: break
        else: continue
        break
    say("   USB-C 1 tool line (Ø10): " + ", ".join(f"{a} deg {'+y' if s_ > 0 else '-y'}: {c:.1f}" for a, s_, c in tilts) + f" -> dir {np.round(USB1_DIR, 3).tolist()}")
    mp0 = mount_v12(P, dc)
    xr = place_xr2(P, dc, mp0)
    P["XR2"] = xr
    say("   " + describe("XR2", *xr))
    cab = cables(dc, {n: v + P["Radxa Zero 3W"][1] for n, v in P["Radxa Zero 3W"][0]["pts"].items()},
                 {n: v + P["UBEC"][1] for n, v in P["UBEC"][0]["pts"].items()}, {n: v + xr[1] for n, v in xr[0]["pts"].items()})
    lo = np.min([np.min([b[0] for b in o["body"]], 0) + t for o, t in P.values()], 0); hi = np.max([np.max([b[1] for b in o["body"]], 0) + t for o, t in P.values()], 0)
    res = dict(poses=P, cab=[(n, float(g), np.asarray(m), float(L)) for n, g, m, L in cab], lo=lo, hi=hi, dc=dc)
    # the variants, sized on dimensions.md L (not on the 50 N bay-corner case, reported as E): the lightest that passes is A
    base = dict(SPINE_SKIN=True, SPINE_TUBE=20.0, UBOX=99.0, BLOCK_SOLID=True, FOOT_TOP=45.0, LOW_AFT=20.0)
    VARS = [("V0 round 4", {}), ("V1 tube 14", dict(SPINE_TUBE=14.0)), ("V2 tube 14 + UBEC box 6", dict(SPINE_TUBE=14.0, UBOX=6.0)),
            ("V3 flanges only + UBEC box 6", dict(SPINE_SKIN=False, UBOX=6.0)),
            ("V4 V2 + block walls 2.4 / 20 %", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False)),
            ("V5 V4 + foot to z 34", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0)),
            ("V6 V5 + lower back 6 aft", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=6.0)),
            ("V7 V3 + block 2.4 / 20 % + foot 34 + lower back 6", dict(SPINE_SKIN=False, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=6.0)),
            ("V8 V5 + lower back 14 aft", dict(SPINE_TUBE=14.0, UBOX=6.0, BLOCK_SOLID=False, FOOT_TOP=34.0, LOW_AFT=14.0))]
    say("variants, sized on dimensions.md L (crash 35 g, 6 faces + 8 corners, <= 1.0; flight 4 g / 2 g and landing 6 g, <= 0.5; break order in the "
        "crash cases: the yoke bolts or the tail block >= 1.5 x before the carrier); the 50 N bay-corner case (E) reported:")
    vrows = []
    for vn, cfg in VARS:
        c_ = dict(base, **cfg); globals().update(c_)
        mpv = mount_v12(P, dc, xr); ldv = loads_mesh(res, dc, mpv); cgv = cg_final(res, dc, mpv); bk = ldv["by_kind"]
        r_ = dict(name=vn, cfg=c_, mass=round(sum(v[1] for v in mpv.values()), 1), pieces={k: v[1] for k, v in mpv.items()},
                  cage_first_order=bool(bk["crash"]["worst"] <= CAGE_CRASH_WORST / 1.5),
                  cg=[round(cgv["hin"], 2), round(cgv["hout"], 2)], crash=bk["crash"]["worst"], flight=bk["flight"]["worst"], landing=bk["landing"]["worst"],
                  E=bk["E"]["worst"], order_ok=bk["crash"]["order_ok"], order_fails=bk["crash"]["order_fails"], l_pass=ldv["l_pass"],
                  one_body=all(one_body(v[0]) == 1 for v in mpv.values()))
        vrows.append(r_)
        say(f"   {vn}: mount {r_['mass']} g ({', '.join(f'{k} {g:.1f}' for k, g in r_['pieces'].items())}); CG {r_['cg'][0]:.2f} / {r_['cg'][1]:.2f} at {dc:+.1f}; "
            f"worst crash {r_['crash']:.2f}, flight {r_['flight']:.2f}, landing {r_['landing']:.2f} (L pass {r_['l_pass']}); L breaking order (mount <= cage "
            f"{CAGE_CRASH_WORST} / 1.5) {r_['cage_first_order']}; yoke / block before the carrier {r_['order_ok']} {r_['order_fails']}; "
            f"50 N bay corner (E) {r_['E']:.2f}; one body {r_['one_body']}")
    ok_ = [r for r in vrows if r["l_pass"] and r["cage_first_order"] and r["one_body"] and max(r["cg"]) <= 3.0]
    pick = min(ok_, key=lambda r: (len(r["order_fails"]), r["mass"])) if ok_ else vrows[0]   # fewest yoke / block order misses, then the lightest
    globals().update(pick["cfg"])
    say(f"   picked (passes L, its breaking order and the balance at {dc:+.1f}; then the fewest cases where the yoke or block is not 1.5 x before the carrier; then the lightest): {pick['name']}")
    out = {}
    for label, strut in (("A", False), ("B", True)):
        mp = mount_v12(P, dc, xr, strut=strut)
        say(f"concept {label} ({'A + a strut to the tail point (z 41.5, under the cap), clip fuse' if strut else 'yoke + spine on the belly-mount tail block'}):")
        bodies = {}
        for k, (m, g, c, note) in mp.items():
            bodies[k] = one_body(m)
            say(f"   {k}: {g:.1f} g (E), CG ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), x {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, y {m.bounds[0][1]:.1f}..{m.bounds[1][1]:.1f}, "
                f"z {m.bounds[0][2]:.1f}..{m.bounds[1][2]:.1f}; bodies {bodies[k]}, watertight {m.is_watertight}; {note}")
        ck = checks(res, dc, mp)
        for l in fmt_checks(ck): say(l)
        fp = fit_table(res, mp, dc)
        say(f"   fit table ({len(fp)} pairs within 10; a declared bearing contact passes at overlap <= {CONTACT_MM3} mm3, every other pair needs >= 2.0): "
            f"{sum(r['ok'] for r in fp)} / {len(fp)} pass")
        for r in fp: say(f"      {'ok  ' if r['ok'] else 'FAIL'} {r['a']} / {r['b']}: overlap {r['overlap_mm3']} mm3, gap {r['gap_mm']:.2f} ({r['kind']})")
        sw = sweep_out(res)
        cg = cg_final(res, dc, mp)
        sc = slide_wide(res, dc, mp, True); bs = min(sc, key=lambda r: max(r[1], r[2]))
        ld = loads_mesh(res, dc, mp)
        st = None
        if strut:
            free = MODEL["nodes"][MODEL["nring"]:]; node = free[np.argmin(free[:, 0])]
            o_, t_ = P["Radxa Zero 3W"]; bR_ = o_["body"][0] + t_
            top = np.array([bR_[0][0] + 6.0, (SPINE_Y[0] + SIDE_Y[0]) / 2, RADXA_POSE["ztop"] - 3.5])
            u_ = (node - top) / np.linalg.norm(node - top); st = (top, np.abs(u_), E_PETG * math.pi * 9.0 / np.linalg.norm(node - top))
        md = modes_mesh(res, dc, mp, ld, st)
        g_mount = sum(v[1] for v in mp.values())
        man, keys = manifest_v12(res, dc, mp)
        r_sc, txt = run_system_check(man, os.path.join(RUN, f"system_check{'' if label == 'A' else '_alt_B'}.txt"))
        say(f"   mass: mount {g_mount:.1f} g, cables {sum(g for _, g, _, _ in res['cab']):.1f} g (E); CG pack in {cg['hin']:.2f}, out {cg['hout']:.2f} mm at catch {dc:+.1f} "
            f"({cg['mass']:.0f} g); best catch with the bay riding it {bs[0]:+.2f}: {bs[1]:.2f} / {bs[2]:.2f}")
        say(f"   modes (E, from the sections): lateral {md['lateral']:.0f} Hz, vertical {md['vertical']:.0f} Hz ({md['carried_g']} g carried at {md['carried_cg']}, from the plate bolts x {md['root_x']})")
        for kind, v in ld["by_kind"].items():
            say(f"   loads, {kind} (limit {v['limit']}): worst {v['worst']:.2f} (fuse {v['fuse']:.2f}, tail block {v['root']:.2f}, carrier {v['carrier']:.2f}), "
                f"pass {v['pass']}; a replaceable piece 1.5 x before the carrier in every case {v['order_ok']} {v['order_fails']}")
        for k, v in ld["cases"].items(): say(f"      {k}: " + json.dumps(v))
        say("   weakest sections (least I): " + json.dumps(ld["weakest_sections"]))
        for nm, part, p, d, cl in keys: say(f"   key {nm}: from {np.round(p, 1).tolist()} along {np.round(d, 3).tolist()}, clear of rods / nodes (0.5) and prop disks (0) {cl:.2f}")
        for k, v in sw.items(): say(f"   tool Ø10, {k}: from {v['start']} along {v['dir']}, out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
        say("   " + txt.splitlines()[0]); say("   " + txt.splitlines()[-1][:900])
        out[label] = dict(mp=mp, ck=ck, fp=fp, sw=sw, sc_txt=txt, cg=cg, sc=sc, bs=bs, ld=ld, md=md, g=g_mount, sc_line=txt.splitlines()[-1], r=r_sc, bodies=bodies,
                          keys=[(nm, np.round(p, 1).tolist(), np.round(d, 3).tolist(), round(cl, 1)) for nm, part, p, d, cl in keys])
    A = out["A"]; mp = A["mp"]
    # funnel margins (every tail part) at the layout's catch and at the best riding catch (the bay rides: the same margins), wash there
    o, t = P["Radxa Zero 3W"]; ou, tu = P["UBEC"]
    bR = o["body"][0] + t; bU = ou["body"][0] + tu
    fm = {n: round(v["funnel"], 1) for n, v in A["ck"].items()}
    say(f"funnel line at catch {dc:+.1f}: (%.1f, %.1f) -> (%.1f, %.1f); margins (part's lowest point over the line): " % fl + json.dumps(fm))
    ub = [b + tu for b in ou["body"]]; Qu = np.vstack([surf(box(b[0], b[1]), 3000) for b in ub]); fz = Qu[:, 2] - funnel_z(Qu[:, 0], dc)
    i_ = int(np.nanargmin(np.where(np.isfinite(fz), fz, np.nan)))
    say(f"   the UBEC (body + leads): lowest margin {fz[i_]:+.1f} at x {Qu[i_, 0]:.1f}, z {Qu[i_, 2]:.1f} (line z there {funnel_z(np.array([Qu[i_, 0]]), dc)[0]:.1f})")
    dd = A["bs"][0] - dc
    shifted = item_meshes(res)
    wb = {}
    for n, m in list(shifted.items()) + [(k, v[0].copy()) for k, v in mp.items()]:
        Q = surf(m, 3000) - [dd, 0, 0]; w_ = wash_dist(Q[:, 0], Q[:, 1]); w_[under_plate(Q)] = np.inf; wb[n] = round(float(w_.min()), 1)
    say(f"   at the best riding catch {A['bs'][0]:+.2f} (everything of ours {-dd:+.2f} in x with the catch and the funnel line): funnel margins unchanged; wash " + json.dumps(wb))
    ur = dict(at_layout_catch=ubec_region(res, dc, mp), at_best_catch=ubec_region(res, dc, mp, A["bs"][0]))
    say("UBEC pose region (balance <= 3 pack in / out at this catch, funnel >= 0, wash >= 3; everything else fixed): " + json.dumps(ur))
    gapRU = bU[0][1] - bR[1][1]; kp = keepclear_plug(o, t)
    say(f"pose check: UBEC to Radxa component face {gapRU:.1f} mm (need >= {GAP:.0f} and {AIR:.0f} of air over the SoC)")
    say(f"keep-clear: plugs x {kp[0][0]:.1f}..{kp[1][0]:.1f}, y {kp[0][1]}..{kp[1][1]}, z {kp[0][2]}..{kp[1][2]} (low 90 deg plugs, E unmeasured); tool lines: "
        + "; ".join(f"{k} from {v['start']} along {v['dir']}" for k, v in A["sw"].items()))
    allm = trimesh.util.concatenate([at_dc(n, dc) for n in MODEL["P"] if n not in MOVED and n not in PLACEHOLDERS] + list(item_meshes(res).values()) + [v[0] for v in mp.values()])
    ext = allm.bounds[1] - allm.bounds[0]
    say(f"drone bounding box with the mount: {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} mm, {np.prod(ext) / 1e6:.1f} L, largest extent {ext.max():.0f}")
    preview([(f"A (picked): yoke + spine on the belly-mount tail block, v12 poses, catch {dc:+.1f}; CG {A['cg']['hin']:.1f} / {A['cg']['hout']:.1f} mm", res, dc, mp, MODEL),
             (f"B: A + strut to the tail point (clip fuse); CG {out['B']['cg']['hin']:.1f} / {out['B']['cg']['hout']:.1f} mm", res, dc, out["B"]["mp"], MODEL)],
            os.path.join(RUN, "preview.png"))
    for f in os.listdir(RUN):
        if f.endswith(".stl"): os.remove(os.path.join(RUN, f))
    for k, (m, g, c, note) in mp.items(): m.export(os.path.join(RUN, "".join(ch_ if ch_.isalnum() else "_" for ch_ in k.lower())[:40] + ".stl"))
    env = dict(round=4, layout="scout_layout v12 (PR 141, on main)", catch=dc, funnel_line=np.round(fl, 1).tolist(),
               note="drone frame (x nose, y left, z up, thrust centre x 0), catch 5.0 forward (v12); boxes [[x0, y0, z0], [x1, y1, z1]]; masses E unless tagged",
               items={}, mount={}, keep_clear=dict(plugs=np.round(kp, 1).tolist(), tool_lines={k: dict(start=v["start"], dir=v["dir"], d=TOOL) for k, v in A["sw"].items()},
                                                   keys={nm: dict(start=p, dir=d, d=4) for nm, p, d, cl in A["keys"]}))
    for n, (o_, t_) in P.items():
        env["items"][n] = dict(box=np.round(np.array([np.min([b[0] for b in o_["body"]], 0), np.max([b[1] for b in o_["body"]], 0)]) + t_, 1).tolist(),
                               mass_g=MASS[n][0], tag=MASS[n][1], cg=np.round(o_["cg"] + t_, 1).tolist(), funnel_margin=round(A["ck"][n]["funnel"], 1),
                               rods=round(A["ck"][n]["rods"], 1), inset=round(A["ck"][n]["inset"], 1), wash=round(A["ck"][n]["wash"], 1))
    for k, (m, g, c, note) in mp.items():
        fm_ = A["ck"][k]["funnel"]
        env["mount"][k] = dict(box=np.round(m.bounds, 1).tolist(), mass_g=g, tag="E", cg=np.round(c, 1).tolist(), note=note,
                               funnel_margin=None if not np.isfinite(fm_) else round(fm_, 1), rods=round(A["ck"][k]["rods"], 1), inset=round(A["ck"][k]["inset"], 1))
    cb = [(n, g, c) for n, g, c, _ in res["cab"]]
    env["cables"] = dict(mass_g=round(sum(g for _, g, _ in cb), 2), tag="E", cg=np.round(sum(g * c for _, g, c in cb) / sum(g for _, g, _ in cb), 1).tolist(),
                         runs={n: dict(g=round(g, 2), mid=np.round(c, 1).tolist(), L=round(L, 0)) for n, g, c, L in res["cab"]})
    gm = sum(v["mass_g"] for v in env["mount"].values())
    env["mount_total"] = dict(mass_g=round(gm, 1), cg=np.round(sum(v["mass_g"] * np.array(v["cg"]) for v in env["mount"].values()) / gm, 1).tolist())
    env["cg"] = dict(pack_in=np.round(A["cg"]["cin"], 2).tolist(), pack_out=np.round(A["cg"]["cout"], 2).tolist(), h_in=round(A["cg"]["hin"], 2),
                     h_out=round(A["cg"]["hout"], 2), mass=round(A["cg"]["mass"], 1),
                     best_catch_bay_riding=dict(catch=A["bs"][0], h_in=round(A["bs"][1], 2), h_out=round(A["bs"][2], 2)))
    env["ubec_region"] = ur
    env["requirements"] = dict(belly_mount="its tail face at x -31.0 over |y| <= 22, z 9.6..18.5 (10.85 ahead of the layout's box): the tail block's web and the 2 "
                                            "plate-bolt bosses sit there under the plate; the block's gusset bears on the receiver's tail face (z -4..9.6, |y| <= 7.2)",
                               plate="2 x Ø3.4 holes at (-42.0, +-16.0) in the tail tab (8.0 from its tail edge, 7.0 from the notch edge)")
    json.dump(env, open(os.path.join(RUN, "envelope.json"), "w"), indent=1)
    res_out = dict(round=4, catch=dc, poses=pose_json(res), cables=env["cables"], concepts={
        k: dict(mount={kk: dict(g=g, cg=np.round(c, 2).tolist(), bounds=np.round(m.bounds, 2).tolist(), note=note, bodies=v["bodies"][kk]) for kk, (m, g, c, note) in v["mp"].items()},
                mount_g=round(v["g"], 1), checks=v["ck"], fit=v["fp"], sweeps=v["sw"], loads={kk: vv for kk, vv in v["ld"].items() if kk != "secs"}, modes=v["md"],
                cg=dict(pack_in=np.round(v["cg"]["cin"], 3).tolist(), pack_out=np.round(v["cg"]["cout"], 3).tolist(),
                h_in=round(v["cg"]["hin"], 3), h_out=round(v["cg"]["hout"], 3), mass=round(v["cg"]["mass"], 1)), slide_riding=v["sc"], best_riding=v["bs"],
                keys=v["keys"], system_check=v["sc_line"][:900], by_kind=v["ld"]["by_kind"]) for k, v in out.items()},
        funnel=dict(line=np.round(fl, 2).tolist(), margins=fm, wash_at_best_catch=wb), ubec_region=ur, variants=vrows, picked=pick["name"],
        pose_findings=dict(ubec_radxa_gap=round(float(gapRU), 2), plug_keepclear=np.round(kp, 1).tolist()), usb1_tilts=tilts,
        usb1_dir=np.round(USB1_DIR, 3).tolist(), bbox=dict(extents=np.round(ext, 0).tolist(), L=round(float(np.prod(ext) / 1e6), 1)))
    json.dump(res_out, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())


if __name__ == "__main__":
    if "--keel" in sys.argv: run_all()          # rounds 1-2 (superseded; needs the v10 assembly from git history)
    else: run_v12()                             # round 4: the mount on v12's poses
