"""Scout v3 electronics: system pass v2 (27 Oct 2026). Places the Radxa Zero 3W, the 5 V UBEC and the RadioMaster XR2 by
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
FUNNEL_LINE = ([-197.0, -165.0, -150.0, -130.0, -100.0, -79.0], [44.5, 20.0, 8.0, -7.0, -30.0, -46.6])   # tail bay lowest point above
                                  # this (x, z) line: the hull from the tail bottom rod (-79, -49.6) to the tail point (-197, 41.5) + 3,
                                  # so the Overseer's tail-tip ramp stays clear (cage session 27 Oct, hard, E)
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
BLK_GAP, BLK_LEN, BLK_HALF, BLK_Z = 2.0, 21.0, 6.5, (-8.0, 18.5)   # block x from 2 behind the TPU end pieces, 21 long, |y| <= 6.5
TONGUE = 5.0                      # yoke tongue: 3.0 counterbore for the M3 head + 2.0 web under it
BOLT_DX = (5.5, 15.5)             # M3 positions from the block's front face (C)
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


# ---------------------------------------------------------------- the assembled model
def centroid(m):
    if m.is_watertight and m.volume > 1.0: return np.asarray(m.center_mass, float), "volume"
    w = m.area_faces; return (m.triangles_center * w[:, None]).sum(0) / w.sum(), "area"


def load_model(asm_path=ASM, v9=V9):
    A = json.load(open(asm_path)); P = {}
    for p in A["parts"]:
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


def funnel_z(x):
    """the funnel line's z at x (nothing of the tail bay below it); -inf ahead of x -79 (the funnel band rules there)"""
    x = np.asarray(x, float); return np.where(x < FUNNEL_LINE[0][-1], np.interp(x, *FUNNEL_LINE), -np.inf)


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
    F |= bx[:, None, None] & (np.abs(YC) <= BLK_HALF + GAP)[None, :, None] & ((ZC >= BLK_Z[0] - TONGUE - GAP) & (ZC <= BLK_Z[1] + TONGUE + GAP))[None, None, :]
    F |= Z3 < funnel_z(XC)[:, None, None] + 0.5
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
RADXA_TRAY = B(-2, 65, -1.5, 30, -4.6 - TRAY_T, -4.6)


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
        flat_ok = item == "Radxa Zero 3W" and abs(o["face"][2]) > 0.9            # the Radxa flat on a tray (cage session 27 Oct)
        if item == "Radxa Zero 3W" and not FREE and RADXA_MODE != "any" and (RADXA_MODE == "flat") != flat_ok: continue
        if ITEMS[item]["keel"] and not FREE and abs(o["face"][1]) < 0.9 and not flat_ok: continue
        if item == "XR2" and abs((R @ [0, 0, 1.0])[2]) < 0.9: continue
        key = (item, tuple(np.round(np.concatenate([b.ravel() for b in o["body"]] + [o["face"]]), 2))) if item == "UBEC" else id(R)
        if key in seen: continue
        seen.add(key); out.append(o)
    return out


def tray_box(o):
    R = o["R"]; c = np.array([[RADXA_TRAY[i][0], RADXA_TRAY[j][1], RADXA_TRAY[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)]) @ R.T
    return np.array([c.min(0), c.max(0)]) - o["o"]


def candidates(item, IF, IA, x_lim=None, IF2=None):
    """every feasible pose: body clear of F, air and sweeps clear of A (sweeps run to the grid edge, out of the shell)"""
    ors = orient_set(item); out = []
    xs = np.arange(-200.0, 140.0, 3.0); zs = np.arange(-45.0, 62.0, 3.0); ys = np.arange(-120.0, 120.0, 2.0)
    for k, o in enumerate(ors):
        flat = item == "Radxa Zero 3W" and abs(o["face"][2]) > 0.9 and not FREE
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
    if "Radxa" in str(ors[0].get("item")):                    # the tray counts as the Radxa's in the clash checks
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
    t0 = time.time(); F, F2 = F if isinstance(F, tuple) else (F, F); IF, IA, IF2 = integral(F), integral(A), integral(F2)
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
    # 1. tail block on the belly mount (the belly mount's own piece; drawn here as its tail): block, bridge over the receiver's tail,
    #    gusset down behind it; 4 M3 heat-set inserts, 2 in the top face and 2 in the bottom, each in a Ø10 boss (3.0 wall)
    blk = union([box([xb1, -H, z0], [xb0, H, z1]), box([xb0, -H, an["rec_top"]], [an["belly_tail"] + 4.0, H, z1]),
                 box([xb0, -H, an["tpu_top"] + 2.0], [an["rec_tail"] - 2.0, H, an["rec_top"]])])
    holes = []
    for x in bolts:
        holes += [cyl([x, 0, z1 - INSERT[1]], [x, 0, z1 + 0.1], INSERT[0] / 2), cyl([x, 0, z0 - 0.1], [x, 0, z0 + INSERT[1]], INSERT[0] / 2)]
    blk = diff(blk, holes)
    vb = blk.volume
    parts["belly-mount tail block (PETG)"] = (blk, round((BLK_LEN * 2 * H * (z1 - z0) - (BLK_LEN - 4.8) * (2 * H - 4.8) * (z1 - z0 - 4.8)) * PETG
                                                  + 4 * math.pi * (INSERT[2] / 2) ** 2 * INSERT[1] * PETG + (vb - BLK_LEN * 2 * H * (z1 - z0)) * PETG, 2),
                                              "hollow 2.4 walls + 4 insert bosses")
    # 2. carrier: yoke tongues over and under the block (Ø12 pads, Ø6 x 3 counterbores, 2.0 web), box keel |y| <= 6.5 behind the
    #    block whose side walls carry the Radxa (standoffs) and the UBEC (zip ties), XR2 shelf on a 6 x 6 post
    tongue_lo = xb1; tongue_hi = xb0 + 0.5
    pts = [[xb1, z0 - TONGUE], [xb1, z1 + TONGUE]]
    o, t = P["Radxa Zero 3W"]; flat = abs(o["face"][2]) > 0.9; trb = tray_box(o) + t
    for n in ("Radxa Zero 3W", "UBEC", "XR2"):
        o, t = P[n]; b = (trb if (n == "Radxa Zero 3W" and flat) else o["body"][0] + t)
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
    if flat:                                                          # the keel stops at the tray (the board above / below it)
        up = o["face"][2] < 0; zc_ = trb[0][2] if up else trb[1][2]
        pg = pg.difference(sg.box(trb[0][0] - 0.01, zc_ if up else -1000, trb[1][0] + 0.01, 1000 if up else zc_))
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
    # 3. Radxa standoffs (4 x Ø6 posts, M2.5 into inserts, 3.0 long from the wall to the board back)
    o, t = P["Radxa Zero 3W"]; R = o["R"]
    so = []
    for hx, hy in RADXA_HOLES:
        a = R @ np.array([hx, hy, -4.6]) - o["o"] + t; b = R @ np.array([hx, hy, -1.6]) - o["o"] + t
        so.append(cyl(a + (a - b) / 3.0 * 0.2, b, 3.0, 16))
    parts["Radxa standoffs (PETG)"] = (union(so), round(4 * math.pi * 9 * 3.0 * PETG, 2), "4 x Ø6 x 3, M2.5")
    if flat:
        tr = box(trb[0], trb[1]); parts["Radxa tray (PETG)"] = (tr, round(tr.volume * FILL * PETG, 2), f"2.4 tray on the keel, {FILL:.0%} (E)")
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
        w = float(wash_dist(Q[:, 0], Q[:, 1]).min())
        inset = float((-(Q @ eq[:, :3].T + eq[:, 3])).min(1).min()) - ROD_R
        fz = Q[:, 2] - funnel_z(Q[:, 0]); fl = float(fz[np.isfinite(fz)].min()) if np.isfinite(fz).any() else float("nan")
        out[n] = dict(gaps=g, wash=w, rods=rod_gap(Q), inset=inset, funnel=fl)
    return out


def sweep_out(res, M=None):
    """Radxa SD card and USB-C 1: a Ø10 tool along the pull direction from the slot to the outside of the shell; the least
    distance from its axis to a rod or node surface past the shell face (>= 5 = the tool passes through a cell)"""
    M = M or MODEL; o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]; out = {}
    eq = ConvexHull(M["nodes"]).equations
    for nm, p, d in (("SD card", o["pts"]["sd"] + t, R @ np.array([-1.0, 0, 0])), ("USB-C 1", o["pts"]["usb1"] + t, R @ np.array([0, -1.0, 0]))):
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
        a["ignore"] = list(a.get("ignore", [])) + ["tail cables (5 runs)"]
        acc.append(a)
    if extra_access:
        o, t = res["poses"]["Radxa Zero 3W"]; R = o["R"]
        acc.append(dict(name="Radxa microSD out + Ø10 fingers", part="Radxa Zero 3W", **{"from": (o["pts"]["sd"] + t).tolist()}, dir=(R @ [-1.0, 0, 0]).tolist(),
                        length_mm=150.0, tool={"type": "cylinder", "d": TOOL}, ignore=["tail cables (5 runs)"]))
        acc.append(dict(name="Radxa USB-C 1 plug + Ø10", part="Radxa Zero 3W", **{"from": (o["pts"]["usb1"] + t).tolist()}, dir=(R @ [0, -1.0, 0]).tolist(),
                        length_mm=150.0, tool={"type": "cylinder", "d": TOOL}, ignore=["tail cables (5 runs)"]))
        an = anchors(dc, M); z0, z1 = BLK_Z
        for x in [an["xb0"] - d for d in BOLT_DX]:
            acc.append(dict(name=f"M3 key, carrier top bolt (x {x:.1f})", part="tail carrier: yoke + box keel (PETG)", **{"from": [x, 0.0, z1 + TONGUE]},
                            dir=[0, 0, 1], length_mm=150.0, tool={"type": "cylinder", "d": 4}, ignore=["tail cables (5 runs)"]))
            acc.append(dict(name=f"M3 key, carrier bottom bolt (x {x:.1f})", part="tail carrier: yoke + box keel (PETG)", **{"from": [x, 0.0, z0 - TONGUE]},
                            dir=[0, 0, -1], length_mm=150.0, tool={"type": "cylinder", "d": 4},
                            ignore=["tail cables (5 runs)"] + [n for n in CATCH if M["P"].get(n, {}).get("swap")]))
    A["access"] = acc
    car = list(mparts)
    cp = [c for c in A["contact_pairs"] if not (set(c[:2]) & set(MOVED))]
    cp += [[a, b, 1e9] for a in car for b in car if a < b]
    cp += [[n, "tail carrier: yoke + box keel (PETG)", 1e9] for n in MOVED] + [["Radxa Zero 3W", "Radxa standoffs (PETG)", 1e9], ["XR2", "XR2 shelf (PETG)", 1e9]]
    cp += [["belly-mount tail block (PETG)", n, 1e9] for n in ("mount + nose legs (Scout)", "receiver v3 (latch)")]
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


def loads(res, dc, mparts):
    """hand estimate (E): the carried mass at its CG on the yoke joint, 35 g crash in each axis, 50 N on the bay's far corner"""
    an = anchors(dc); z0, z1 = BLK_Z
    rows = [(MASS[n][0], o["cg"] + t) for n, (o, t) in res["poses"].items()]
    rows += [(g, c) for k, (m, g, c, _) in mparts.items() if not k.startswith("belly")]
    rows += [(g * 0.5, c) for _, g, c, _ in res["cab"]]            # half of each run hangs on the carrier (E)
    m = sum(r[0] for r in rows); c = sum(r[0] * r[1] for r in rows) / m
    j = np.array([(an["xb0"] + an["xb1"]) / 2, 0.0, (z0 + z1) / 2])
    F35 = m / 1000 * 9.81 * 35; arm = c - j
    zarm = (z1 + TONGUE / 2) - (z0 - TONGUE / 2); xarm = BOLT_DX[1] - BOLT_DX[0]
    pre = 300.0                                                   # M3 into a heat-set insert in PETG, working preload (E)
    Mp = F35 * abs(arm[0]); Mr = F35 * abs(arm[2]); My = F35 * abs(arm[0])
    out = dict(carried_g=round(m, 1), carried_cg=np.round(c, 1).tolist(), joint=np.round(j, 1).tolist(), F35_N=round(F35, 1),
               pitch_Nmm=round(Mp, 0), roll_Nmm=round(Mr, 0), yaw_Nmm=round(My, 0),
               bolt_tension_pitch_N=round(Mp / zarm, 1), bolt_shear_yaw_N=round(My / xarm / 2, 1),
               roll_capacity_Nmm=round(4 * pre * BLK_HALF, 0))
    far = np.array([res["lo"][0], 0, res["hi"][2]]) - j
    M50 = 50.0 * np.hypot(far[0], far[2])
    out.update(h50_corner=np.round(far + j, 1).tolist(), h50_moment_Nmm=round(M50, 0), h50_bolt_tension_N=round(50 * abs(far[0]) / zarm + 50 / 4, 1),
               h50_bolt_shear_N=round(50 * abs(far[0]) / xarm / 2, 1))
    # keel root section (box 13 x root height, 2.4 walls): bending stress for the 50 N lateral push at the far corner (E)
    hroot = (z1 + TONGUE) - (z0 - TONGUE); b_ = 2 * KEEL_OUT; t_ = KEEL_WALL
    Iz = (hroot * b_ ** 3 - (hroot - 2 * t_) * (b_ - 2 * t_) ** 3) / 12; Iy = (b_ * hroot ** 3 - (b_ - 2 * t_) * (hroot - 2 * t_) ** 3) / 12
    s_lat = 50.0 * abs(far[0]) * KEEL_OUT / Iz; s_vert35 = F35 * abs(arm[0]) * (hroot / 2) / Iy
    out.update(keel_root_Iz=round(Iz, 0), keel_root_Iy=round(Iy, 0), sigma_h50_lateral_MPa=round(s_lat, 2), sigma_35g_vertical_MPa=round(s_vert35, 2),
               index_h50=round(s_lat / 25.0, 2), index_35g=round(max(s_vert35, F35 * abs(arm[0]) * KEEL_OUT / Iz) / 25.0, 2))
    # insert pull-out and bearing
    out.update(bearing_MPa_h50=round(out["h50_bolt_shear_N"] / (3.0 * TONGUE), 2))
    # first lateral mode (E): the carried mass on the keel root box as a cantilever, PETG E 2000
    L = abs(arm[0]); k = 3 * 2000 * Iz / L ** 3
    out.update(mode_lateral_Hz=round(math.sqrt(k * 1000 / (m / 1000)) / (2 * math.pi), 0))
    return out


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
    fig.text(0.01, 0.003, "green Radxa Zero 3W, orange UBEC (50 x 25 x 10 worst), purple XR2, blue the carrier (belly-mount tail block, yoke, box keel, "
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
    tmp = tempfile.mkdtemp(); lay = os.path.join(tmp, "layout_v3_e.py"); patched_layout(lay)
    env = dict(os.environ, NOX_ROOT=ROOT, SCOUT_ELEC=jf, SCOUT_DCS=str(dc), SCOUT_FIX_DC="1")
    r = subprocess.run([sys.executable, lay, out_dir], env=env, capture_output=True, text=True)
    shutil.rmtree(tmp, ignore_errors=True)
    if r.returncode: raise RuntimeError(r.stderr[-3000:])
    return r.stdout


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


def run_all():
    global KEEL_OUT, FREE, RADXA_MODE
    t0 = time.time()
    say("Scout v3 electronics, system pass v2 (27 Oct 2026): nox/scout/v3/electronics_system.py on scout_layout v10 (PRs 127 + 131, main 23a7814)")
    say(f"model: {len(MODEL['P'])} parts in pose, catch set {DC_REF} aft (v10); shell {len(MODEL['nodes'])} nodes / {len(MODEL['rods'])} rods + {len(MODEL['links'])} links")
    say("centroids: " + ", ".join(f"{n} ({P['how']})" for n, P in MODEL["P"].items() if P["how"] != "volume"))
    ref = v9_bay_eval()
    say(f"reference, v10's own tail bay in this model (DC {DC_REF}): CG pack in {ref['hin']:.2f}, out {ref['hout']:.2f} mm (E); wash "
        + ", ".join(f"{n} {w:+.1f}" for n, w in ref["wash"].items()) + " (need >= 3: negative = inside a disk column)")
    # 1. joint optimisation with the slide (picked family: the box keel)
    say("1. placement x slide (box keel 8 wide, items on its side walls):")
    runs = {}
    for dc in (-4.0, -3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 1.0, 2.0, 4.0, 6.0, DC_REF, 10.0):
        O, F = grids(dc); runs[dc] = search(dc, F, O)
    ok = {d: r for d, r in runs.items() if r}
    pick_dc = min(ok, key=lambda d: ok[d]["J"]); P_ = ok[pick_dc]
    say(f"   pick: DC {pick_dc:+.1f} (slide {DC_REF - pick_dc:.1f} forward of v10's {DC_REF}), J {P_['J']:.2f}")
    for n, (o, t) in P_["poses"].items(): say("      " + describe(n, o, t))
    B_ = ok.get(DC_REF)
    # 2. alternatives
    say("2. alternatives:")
    say(f"   B no slide (catch stays {DC_REF} aft): " + (f"CG in {np.hypot(*B_['cin'][:2]):.2f} / out {np.hypot(*B_['cout'][:2]):.2f} mm" if B_ else "no feasible layout"))
    KEEL_OUT = 6.5
    C_ = {}
    for dc in (-4.0, -3.0, -2.0):
        O, F = grids(dc); C_[dc] = search(dc, F, O, label="keel 13")
    C_ = {d: r for d, r in C_.items() if r}; cdc = min(C_, key=lambda d: C_[d]["J"]) if C_ else None
    KEEL_OUT = 4.0
    o_, t_ = P_["poses"]["Radxa Zero 3W"]; other = "edge" if abs(o_["face"][2]) > 0.9 else "flat"
    RADXA_MODE = other; E_ = {}
    for dc in (pick_dc - 1.0, pick_dc, pick_dc + 1.0):
        O, F = grids(dc); E_[dc] = search(dc, F, O, label=f"Radxa {other} only")
    RADXA_MODE = "any"
    E_ = {d: r for d, r in E_.items() if r}
    FREE = True
    D_ = {}
    for dc in (pick_dc, DC_REF):
        O, F = grids(dc); D_[dc] = search(dc, F, O, x_lim=40.0, label="forward Radxa (x > -40), free")
    FREE = False
    # 3. the carrier, drawn, and the checks
    say("3. the carrier (drawn round the pick) and the checks:")
    mp = mount(P_, pick_dc)
    for k, (m, g, c, note) in mp.items():
        say(f"   {k}: {g:.1f} g (E), CG ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), x {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, y {m.bounds[0][1]:.1f}..{m.bounds[1][1]:.1f}, "
            f"z {m.bounds[0][2]:.1f}..{m.bounds[1][2]:.1f}; {note}; watertight {m.is_watertight}")
    ck = checks(P_, pick_dc, mp)
    for l in fmt_checks(ck): say(l)
    sw = sweep_out(P_)
    for k, v in sw.items(): say(f"   tool Ø10, {k}: from {v['start']} along {v['dir']}, out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
    cg = cg_final(P_, pick_dc, mp)
    say(f"   CG (true centroids, drawn carrier): pack in ({cg['cin'][0]:+.2f}, {cg['cin'][1]:+.2f}) = {cg['hin']:.2f} mm, pack out ({cg['cout'][0]:+.2f}, {cg['cout'][1]:+.2f}) = "
        f"{cg['hout']:.2f} mm (E, gate 3); {cg['mass']:.0f} g all-up, {cg['mass_out']:.0f} g pack out; added {cg['add_g']:.1f} g")
    v10 = sum(MODEL["P"][n]["g"] * centroid(MODEL["P"][n]["mesh"])[0][0] for n in ("Radxa Zero 3W", "UBEC"))
    v10x = sum(MODEL["P"][n]["g"] * centroid(MODEL["P"][n]["mesh"])[0][0] for n in MOVED)
    mom = sum(r[1] * r[3][0] for r in cg["rows"])
    say(f"   tail bay x moment about the thrust centre: {mom:.0f} g mm ({cg['add_g']:.1f} g at x {mom / cg['add_g']:.1f}: items, carrier, cables) vs the v10 "
        f"placeholder Radxa + UBEC {v10:.0f} (25 g; with its XR2 {v10x:.0f}); {mom - v10x:+.0f} g mm more aft = {(mom - v10x) / cg['mass']:+.1f} mm of CG at the same slide")
    sc = slide_scan(P_, pick_dc, mp); best_sl = min(sc, key=lambda r: max(r[1], r[2]))
    say(f"   slide trim (our parts ride the belly mount): best DC {best_sl[0]:+.2f}: {best_sl[1]:.2f} / {best_sl[2]:.2f} mm")
    ld = loads(P_, pick_dc, mp)
    say("   loads (E): " + json.dumps(ld))
    man = manifest(P_, pick_dc, mp)
    r_sc, txt = run_system_check(man, os.path.join(RUN, "system_check.txt"))
    say("   " + txt.splitlines()[0])
    # alternative B drawn for the picture and its own checks
    mB = mount(B_, DC_REF) if B_ else None
    cgB = cg_final(B_, DC_REF, mB) if B_ else None
    txtB = None
    if B_:
        r_B, txtB = run_system_check(manifest(B_, DC_REF, mB), os.path.join(RUN, "system_check_alt_B.txt"))
        say(f"   alt B (no slide), drawn: CG in {cgB['hin']:.2f} / out {cgB['hout']:.2f} mm; " + txtB.splitlines()[0])
    preview([(f"A (picked): box keel, catch slide to {pick_dc:+.1f} (v10 {DC_REF:+.1f}); CG {cg['hin']:.1f} / {cg['hout']:.1f} mm", P_, pick_dc, mp)]
            + ([(f"B: same carrier, no slide (catch {DC_REF:+.1f}); CG {cgB['hin']:.1f} / {cgB['hout']:.1f} mm (gate 3)", B_, DC_REF, mB)] if B_ else []),
            os.path.join(RUN, "preview.png"))
    for k, (m, g, c, note) in mp.items(): m.export(os.path.join(RUN, "".join(ch if ch.isalnum() else "_" for ch in k.lower())[:40] + ".stl"))
    # 4. result
    def rj(r):
        if not r: return None
        return dict(dc=r["dc"], J=round(r["J"], 3), cg_in=np.round(r["cin"], 3).tolist(), cg_out=np.round(r["cout"], 3).tolist(), bay_cm3=round(r["V"], 1),
                    cable_g=round(r["cable_g"], 2), bay=dict(lo=np.round(r["lo"], 2).tolist(), hi=np.round(r["hi"], 2).tolist()), n_feasible=r["n_feasible"])
    out = dict(pick=dict(**rj(P_), poses=pose_json(P_), cables=[dict(name=n, g=round(g, 2), mid=np.round(m, 2).tolist(), L=round(L, 0)) for n, g, m, L in P_["cab"]],
                         carrier={k: dict(g=g, cg=np.round(c, 2).tolist(), bounds=np.round(m.bounds, 2).tolist(), note=note) for k, (m, g, c, note) in mp.items()},
                         cg_final=dict(pack_in=np.round(cg["cin"], 3).tolist(), pack_out=np.round(cg["cout"], 3).tolist(), h_in=round(cg["hin"], 3), h_out=round(cg["hout"], 3),
                                       mass=round(cg["mass"], 1), mass_out=round(cg["mass_out"], 1), added_g=round(cg["add_g"], 2), bay_cg=np.round(cg["bay_cg"], 2).tolist(),
                                       rows=[dict(name=n, g=round(g, 2), tag=t, cg=np.round(c, 2).tolist()) for n, g, t, c in cg["rows"]]),
                         slide_scan=sc, checks=ck, sweeps=sw, loads=ld, anchors={k: (np.round(v, 2).tolist() if isinstance(v, np.ndarray) else round(v, 2)) for k, v in anchors(pick_dc).items()},
                         system_check=txt.splitlines()[0]),
               scan={str(d): rj(r) for d, r in runs.items()},
               alt_B_no_slide=dict(**rj(B_), poses=pose_json(B_), cg_final=dict(h_in=round(cgB["hin"], 3), h_out=round(cgB["hout"], 3)),
                                   system_check=txtB.splitlines()[0]) if B_ else None,
               alt_C_keel13={str(d): rj(r) for d, r in C_.items()}, alt_C_pick=cdc, alt_E_other_radxa={str(d): rj(r) for d, r in E_.items()},
               bay_moment=dict(ours=round(mom, 0), v10_radxa_ubec=round(v10, 0), v10_with_xr2=round(v10x, 0)),
               alt_D_forward={str(d): (rj(r) if r else None) for d, r in D_.items()},
               ref_v9_bay=ref, weights=dict(W_CG=W_CG, W_V=W_V, W_C=W_C, CG_FLOOR=CG_FLOOR, NEAR=NEAR), dc_ref=DC_REF)
    json.dump(out, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    say(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(RUN, "run.log"), "w").write(LOG.getvalue())


def run_rebuild():
    d = json.load(open(os.path.join(RUN, "result.json"))); P_ = res_from_json(d["pick"]); dc = d["pick"]["dc"]
    mp = mount(P_, dc); out_dir = os.path.join(RUN, "rebuilt")
    say(f"stage 3: shell rebuilt by nox/scout/v3/layout_v3.py at DC {dc:+.2f} with the electronics and the carrier in the drone")
    log = rebuild(P_, dc, mp, out_dir)
    for l in log.splitlines():
        if any(k in l for k in ("shell:", "swap zone", "FC USB-C", "props:", "front camera", "mass ~", "largest cell", "compact:")): say("   layout: " + l.strip())
    M2 = rebuilt_model(out_dir, dc)
    ck = checks(P_, dc, mp, M2)
    for l in fmt_checks(ck): say(l)
    sw = sweep_out(P_, M2)
    for k, v in sw.items(): say(f"   tool Ø10, {k}: out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
    cg = cg_final(P_, dc, mp, M2)
    say(f"   CG on the rebuilt shell: pack in {cg['hin']:.2f}, pack out {cg['hout']:.2f} mm (E); {cg['mass']:.0f} g")
    sc = slide_scan(P_, dc, mp, M2); b = min(sc, key=lambda r: max(r[1], r[2]))
    say(f"   slide trim on the rebuilt shell: best DC {b[0]:+.2f}: {b[1]:.2f} / {b[2]:.2f} mm")
    man = manifest(P_, dc, mp, M2)
    r_sc, txt = run_system_check(man, os.path.join(out_dir, "system_check.txt"))
    say("   " + txt.splitlines()[0])
    sj = json.load(open(os.path.join(out_dir, "cage_shell.json"))); nd = np.array(sj["nodes"])
    json.dump(dict(dc=dc, nodes=len(nd), rods=len(sj["rods"]), extents=np.round(nd.max(0) - nd.min(0), 1).tolist(), checks=ck, sweeps=sw,
                   cg=dict(h_in=cg["hin"], h_out=cg["hout"], pack_in=cg["cin"].tolist(), pack_out=cg["cout"].tolist(), mass=cg["mass"]),
                   slide_scan=sc, system_check=txt.splitlines()[0]), open(os.path.join(out_dir, "recheck.json"), "w"), indent=1)
    for f in os.listdir(os.path.join(out_dir, "parts")):
        pass
    open(os.path.join(out_dir, "recheck.log"), "w").write(LOG.getvalue())


def run_iterate():
    """stage 4: the placement x slide again on the shell rebuilt round stage 1's pick (the cage follows the tail bay), then the
    carrier, the checks and the picture on that shell; `--rebuild` after it closes the loop"""
    global MODEL
    V10 = MODEL; d = json.load(open(os.path.join(RUN, "result.json"))); dc0 = d["pick"]["dc"]
    rb = os.path.join(RUN, "rebuilt"); MODEL = rebuilt_model(rb, dc0)
    say(f"stage 4: placement x slide again on the shell rebuilt at DC {dc0:+.1f} ({len(MODEL['nodes'])} nodes, {len(MODEL['rods'])} rods)")
    runs = {}
    for dc in [dc0 + k for k in (-1.0, 0.0, 1.0, 2.0, 3.0, 4.0)]:
        O, F = grids(dc); runs[dc] = search(dc, F, O)
    ok = {k: r for k, r in runs.items() if r}; pick_dc = min(ok, key=lambda k: ok[k]["J"]); P_ = ok[pick_dc]
    say(f"   pick: DC {pick_dc:+.1f} (slide {DC_REF - pick_dc:.1f} forward of v10's {DC_REF}), J {P_['J']:.2f}")
    for n, (o, t) in P_["poses"].items(): say("      " + describe(n, o, t))
    mp = mount(P_, pick_dc)
    for k, (m, g, c, note) in mp.items():
        say(f"   {k}: {g:.1f} g (E), CG ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), x {m.bounds[0][0]:.1f}..{m.bounds[1][0]:.1f}, y {m.bounds[0][1]:.1f}..{m.bounds[1][1]:.1f}, "
            f"z {m.bounds[0][2]:.1f}..{m.bounds[1][2]:.1f}; {note}")
    ck = checks(P_, pick_dc, mp)
    for l in fmt_checks(ck): say(l)
    sw = sweep_out(P_)
    for k, v in sw.items(): say(f"   tool Ø10, {k}: out of the shell at {v['exits_shell_at_mm']} mm, least clear of a rod / node {v['least_clear_mm']} (need >= 5)")
    cg = cg_final(P_, pick_dc, mp)
    say(f"   CG (true centroids, drawn carrier, iteration-1 shell): pack in ({cg['cin'][0]:+.2f}, {cg['cin'][1]:+.2f}) = {cg['hin']:.2f} mm, pack out "
        f"({cg['cout'][0]:+.2f}, {cg['cout'][1]:+.2f}) = {cg['hout']:.2f} mm (E); {cg['mass']:.0f} g, added {cg['add_g']:.1f} g")
    mom = sum(r[1] * r[3][0] for r in cg["rows"])
    sc = slide_scan(P_, pick_dc, mp); bs = min(sc, key=lambda r: max(r[1], r[2]))
    say(f"   tail bay x moment {mom:.0f} g mm; slide trim best DC {bs[0]:+.2f}: {bs[1]:.2f} / {bs[2]:.2f} mm")
    ld = loads(P_, pick_dc, mp); say("   loads (E): " + json.dumps(ld))
    r_sc, txt = run_system_check(manifest(P_, pick_dc, mp), os.path.join(RUN, "system_check.txt")); say("   " + txt.splitlines()[0])
    MODEL_IT = MODEL; MODEL = V10
    O, F = grids(DC_REF); Bres = search(DC_REF, F, O, label="alt B, no slide (v10 shell)")
    mB = mount(Bres, DC_REF); cgB = cg_final(Bres, DC_REF, mB)
    preview([(f"A (picked): box keel, catch slide to {pick_dc:+.1f} (v10 {DC_REF:+.1f}), cage rebuilt round it; CG {cg['hin']:.1f} / {cg['hout']:.1f} mm", P_, pick_dc, mp, MODEL_IT),
             (f"B: same carrier, no slide (catch {DC_REF:+.1f}, v10 cage); CG {cgB['hin']:.1f} / {cgB['hout']:.1f} mm (gate 3)", Bres, DC_REF, mB, V10)],
            os.path.join(RUN, "preview.png"))
    MODEL = MODEL_IT
    for k, (m, g, c, note) in mp.items(): m.export(os.path.join(RUN, "".join(ch if ch.isalnum() else "_" for ch in k.lower())[:40] + ".stl"))
    d["pick_stage1_v10"] = {k: d["pick"][k] for k in ("dc", "J", "cg_in", "cg_out", "bay", "cg_final", "system_check") if k in d["pick"]}
    d["pick"] = dict(dc=pick_dc, J=round(P_["J"], 3), base="rebuilt iteration 1", poses=pose_json(P_),
                     cables=[dict(name=n, g=round(g, 2), mid=np.round(m, 2).tolist(), L=round(L, 0)) for n, g, m, L in P_["cab"]],
                     bay=dict(lo=np.round(P_["lo"], 2).tolist(), hi=np.round(P_["hi"], 2).tolist()),
                     carrier={k: dict(g=g, cg=np.round(c, 2).tolist(), bounds=np.round(m.bounds, 2).tolist(), note=note) for k, (m, g, c, note) in mp.items()},
                     cg_final=dict(pack_in=np.round(cg["cin"], 3).tolist(), pack_out=np.round(cg["cout"], 3).tolist(), h_in=round(cg["hin"], 3), h_out=round(cg["hout"], 3),
                                   mass=round(cg["mass"], 1), mass_out=round(cg["mass_out"], 1), added_g=round(cg["add_g"], 2), bay_cg=np.round(cg["bay_cg"], 2).tolist(),
                                   rows=[dict(name=n, g=round(g, 2), tag=t, cg=np.round(c, 2).tolist()) for n, g, t, c in cg["rows"]]),
                     bay_moment=round(mom, 0), slide_scan=sc, checks=ck, sweeps=sw, loads=ld,
                     anchors={k: (np.round(v, 2).tolist() if isinstance(v, np.ndarray) else round(v, 2)) for k, v in anchors(pick_dc).items()},
                     system_check=txt.splitlines()[0], scan={str(k): (round(r["J"], 3), round(r["worst"], 3)) for k, r in ok.items()})
    if d.get("alt_B_no_slide"): d["alt_B_no_slide"]["drawn_cg"] = dict(h_in=round(cgB["hin"], 3), h_out=round(cgB["hout"], 3))
    json.dump(d, open(os.path.join(RUN, "result.json"), "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    open(os.path.join(RUN, "run.log"), "a").write("\n" + LOG.getvalue())


if __name__ == "__main__":
    if "--rebuild" in sys.argv: run_rebuild()
    elif "--iterate" in sys.argv: run_iterate()
    else: run_all()
