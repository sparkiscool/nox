"""Scout v3 layout v3 / cage v1 geodesic (27 Oct 2026): the Scout v3 drone (PR 94's latest layout: Bee35 carbon plate,
props on top, receiver v3 + cartridge v6 under it, FC under the plate, Radxa and flow on the belly mount) with a geodesic
shell cage formed to it, the Overseer cage v3 method (`nox/overseer/layout_v5.py`).

Spark 27 Oct: "using the same design idea as the latest scout design make sure the bottom of the cage is the exact same as
the overseer just that loop of rods directly under the battery so funnel is interchangeable, the scout still has one camera
and we will keep the carbon base for the scout".

- Bottom ring: the Overseer's 8 opening ring nodes and 8 tubes, copied from `runs/overseer_layout/v5/cage_shell.json` at run
  time, placed on the same datum: ring centre on the catch (the grip / tab line), ring plane 3 under the cartridge flat bottom
  (the flat bottom sits at R 113 in every bay and at the drone for both packs, `dimensions.md` System box swap: latch). Same
  nodes, same tubes, same height under the swap datum, so the funnel built round the Overseer's ring takes the Scout too.
- The rest of the shell: a frequency 2 class-I geodesic formed to the Scout (props 12 off, the rest 3 off), mirror-exact.
- One camera (front, horizon, landscape) outside the shell with nothing of the drone in its view, held from behind its view
  on a printed boom off the plate nose, a guard bar over its top behind the lens (Overseer cage v3 rule, Spark 26 Oct).
- The carbon base stays: the Bee35 plate carries the motors; 3 spokes per arm tip into the shell, bay ties from the receiver
  to the ring.

python3 nox/scout/v3/layout_v3.py [out_dir]          (~3 min)       SCOUT_RING=own: the alternative ring round the Scout's own
receiver (+30 a side, the Overseer's formula), not interchangeable; written to runs/scout_layout/v6/alt_own_ring/.
Drone frame (PR 94): x nose, y left, z up, plate underside z 18.5, thrust centre x 0, tab line x 0. Numbers: `nox/dimensions.md`
"System Scout v3".
"""
import math, os, sys, io, json
import numpy as np, trimesh
from scipy.spatial import ConvexHull, cKDTree

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nox"))

RING_MODE = os.environ.get("SCOUT_RING", "overseer")
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "runs", "scout_layout", "v6", "" if RING_MODE == "overseer" else "alt_own_ring")
DR = os.path.join(ROOT, "runs", "scout_layout", "v6", "drone")          # drone meshes: PR 94 run v5 assembly (24 Oct), as drawn
OV = os.path.join(ROOT, "runs", "overseer_layout", "v5")                # the Overseer cage v3 (ring source)
LOG = io.StringIO()


def say(*a):
    print(*a); print(*a, file=LOG)


# ---------------------------------------------------------------- numbers (C unless tagged; dimensions.md A / C / L / System Scout v3)
MOTOR, PROP_R, PROP_Z, BLADE = 54.1, 44.45, 41.2, 4.0      # Bee35 hubs (P), Gemfan 3520 (P), prop plane (E), sweep ±4
SHELL_R = 3.0                                              # 6 mm carbon tube (rule, Spark 26 / 27 Oct), 6/4
SHELL_GPM = 1.55e-3 * math.pi / 4 * (6.0 ** 2 - 4.0 ** 2)  # 24.3 g/m (E)
ROD_R, ROD_GPM = 1.5, 11.0e-3                              # links 3.0 solid, 11.0 g/m (C)
NODE_R, NODE_G = 10.15, 4.5                                # printed shell node round 6.3 sockets, Ø20 (E), 4.5 g (E), as the Overseer
GUARD_NODE_R, GUARD_NODE_G = 5.0, 1.5                      # camera guard bar nodes (E)
PROP_GAP, CAGE_GAP = 12.0, 3.0
M_PROP, M_BODY = PROP_GAP + SHELL_R + 1.0, CAGE_GAP + SHELL_R + 1.0
FREQ = 2
BREAK_EVEN = 33.0
CAM_HFOV, CAM_VFOV = 89.0, 73.0                            # DECXIN 2633V2 landscape (the Overseer's 73 x 89 turned, E)
CAM_CONE = 55.0                                            # cage first gated outside this cone (diagonal half-angle 51 + 4)
CAM_W, CAM_H, CAM_D = 38.0, 38.0, 14.0                     # board 38 x 38 (P) + lens, 14 deep (E, PR 94)
LIM = NODE_R + 3.0                                         # shell node centres this far behind the camera's view planes
SLACK = float(os.environ.get("SCOUT_SLACK", 5.0))
CAM_Z = [float(x) for x in os.environ.get("SCOUT_CAM_Z", "30,110").split(",")]   # lens height range scanned
PLATE_NOSE = np.array([60.0, 0.0, 20.25])                  # camera boom root on the plate nose (E, plate edge x ~65)
BOOM = 12.0                                                # printed boom section 12 x 12 (E)

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
    return D


def build(DC):
    """every drone-side global for a catch offset DC: parts, datum, ring, shell target, camera"""
    global D, TAB_X, CART_BOT, REC_B, Z_O, RING, nR, ZONE, BODY, props, body, K, CAM, GRIDS, CDC
    D = drone(DC); CDC = DC
    TAB_X = float(np.mean([D["slider +y (latch)"][0].bounds[:, 0].mean(), D["slider -y (latch)"][0].bounds[:, 0].mean()]))
    CART_BOT = float(D["cartridge v6 (latch)"][0].bounds[0][2])
    REC_B = D["receiver v3 (latch)"][0].bounds
    Z_O = CART_BOT + RING_DZ                                   # the Scout ring plane on the same datum
    RING = ring_nodes(); nR = len(RING)
    ZONE = (REC_B[0][0] - 30.0, REC_B[1][0] + 30.0, REC_B[0][1] - 30.0, REC_B[1][1] + 30.0)   # swap zone: receiver + 30 a side
    BODY = [n for n in D if not n.startswith("prop")]
    props = np.vstack([np.stack([mx + PROP_R * np.cos(a), my + PROP_R * np.sin(a), np.full_like(a, PROP_Z + s_ * BLADE)], 1)
                       for mx, my in MXY for s_ in (1, -1) for a in [np.linspace(0, 2 * np.pi, 72, endpoint=False)]])
    body = hullpts([D[n][0] for n in BODY])
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
    if RING_MODE == "overseer":
        return R_OV + [TAB_X, 0, 0] + [0, 0, Z_O - R_OV[:, 2].mean()]
    # alternative: the Overseer's formula round the Scout's own receiver (+30 a side), 8 nodes, grown until chords clear
    zx, zy = (REC_B[1][0] - REC_B[0][0]) / 2 + 30.0, (REC_B[1][1] - REC_B[0][1]) / 2 + 30.0; cxr = REC_B[:, 0].mean(); k = 1.0
    while True:
        a, b = (math.sqrt(2) * zx + NODE_R) * k, (math.sqrt(2) * zy + NODE_R) * k
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
    env = np.vstack([K, RING - NODE_R, RING + NODE_R])
    vol = lambda P: float(np.prod(P.max(0) - P.min(0))) / 1e6
    for Lz in np.arange(CAM_Z[0], CAM_Z[1] + 0.1, 2.0):
        for phi in (0.0, 20.0, 35.0, 50.0, 65.0, 80.0, 90.0):
            n3 = np.array([math.cos(math.radians(phi)), 0, math.sin(math.radians(phi))])
            for Lx in np.arange(70.0, 260.0, 1.0):
                n2, lens = lower_plane(Lx, Lz)
                if ((K - lens) @ n2).max() > -(LIM + SLACK) or ((RING - lens) @ n2).max() > -(NODE_R + 3.0): continue
                cb = cam_box(Lx, Lz); d_b = float((cb.vertices @ n3).min())
                if (K @ n3).max() > d_b - LIM - SLACK: continue
                if ((cb.vertices @ eqKR[:, :3].T + eqKR[:, 3]).max(1) < LIM + SLACK).any(): continue
                back = np.array([Lx - CAM_D, 0, Lz])
                boom = float(np.linalg.norm(back - PLATE_NOSE))
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
            drop = lf[stuck[lf] > 15]
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
    N = np.vstack([RING, Pg[~in_open]]); N[nR:, 2] = np.maximum(N[nR:, 2], Z_O + 1.0)
    dirs = lambda P: (P - c0) / np.linalg.norm(P - c0, axis=1)[:, None]
    N = sym(N, partners(N)); U = dirs(N)
    N, viol, U = convex(behind(N), U)
    if not np.isfinite(viol.max()): return None
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
    E = sorted(set(E) | {tuple(sorted((remap[a], remap[b]))) for a, b in RING_TUBES if RING_MODE == "overseer"}
               | ({tuple(sorted((i, (i + 1) % 8))) for i in range(8)} if RING_MODE != "overseer" else set()))
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
    for w in (0.0, 8.0, 12.0, 15.0):
        sec = BOOM if w == 0 else 10.0
        for zr in (8.0, 14.0, 20.25, 26.0):
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
    D["camera mount (boom + seat)"] = (trimesh.util.concatenate(meshes + [seat]), "structure", False,
                                       (len(bars) * bl * sec * sec * 0.5 + 4.0 * CAM_W * CAM_H * 0.6) * 1.27e-3, "E", "PETG", "printed", "#aab4c0", False)
    CAM.update(boom=bl, boom_gap=g_, boom_w=w, boom_sec=sec)
    return g_


def usb_line(nodes, E):
    """FC USB-C cable path (12 x 7 plug + cable, 250 long, E): from a point on the FC's tail or left edge, the direction nearest
    the tail-left diagonal (within 60 deg) that clears every part (but the mount and the wires) by 1 and the shell tubes and
    nodes by their radius + 7 + 1 (as the Overseer's access search)"""
    global USB
    fb = D["FC"][0].bounds; zc = 13.5
    starts = [np.array([fb[0][0] + 14.0, -12.7, zc]), np.array([fb[0][0] + 6.0, 0.0, zc]), np.array([fb[0][0] + 14.0, 12.7, zc])]
    others = [D[n][0] for n in D if n not in ("FC", "mount + nose legs (Scout)", "camera mount (boom + seat)") and not n.startswith("prop")]
    pq = [(m, trimesh.proximity.ProximityQuery(m)) for m in others]
    best = None
    for p0 in starts:
        for dv in fib(600):
            if abs(dv[2]) > 0.5: continue
            ang = math.degrees(math.acos(np.clip(dv @ np.array([-0.7071, -0.7071, 0.0]), -1, 1)))
            if ang > 60.0 or (best is not None and ang >= best[0]): continue
            q = p0 + np.linspace(0.0, 250.0, 250)[:, None] * dv
            ok = True
            for a, b in E:
                u = nodes[b] - nodes[a]; t = np.clip(((q - nodes[a]) @ u) / (u @ u), 0, 1)
                if (np.linalg.norm(q - (nodes[a] + t[:, None] * u), axis=1) < SHELL_R + 7.0 + 2.0).any(): ok = False; break
            if not ok or (np.linalg.norm(q[:, None, :] - nodes[None], axis=2) < NODE_R + 7.0 + 1.0).any(): continue
            for m, pq_ in pq:
                near = ((q > m.bounds[0] - 9.0) & (q < m.bounds[1] + 9.0)).all(1)
                if near.any() and (pq_.signed_distance(q[near]) > -4.0).any(): ok = False; break           # the 7 tall plug, 0.5 clear (system_check sweeps the true 12 x 7)
            if ok: best = (ang, p0, dv)
    USB = (best[1], best[2]) if best else (np.array([-10.8 - CDC, -12.7, zc]), np.array([-0.7071, -0.7071, 0.0]))
    return best is not None


def attach(nodes, ring_ids, E=()):
    place_boom(nodes, E)
    usb_ok = usb_line(nodes, E)
    extra = {}
    keep_out = [D[n][0].bounds + [[-ROD_R - 1] * 3, [ROD_R + 1] * 3] for n in
                ("receiver v3 (latch)", "receiver lid (latch)", "FC", "UBEC", "XR2", "Radxa Zero 3W", "flow MTF-02P", "cartridge v6 (latch)")]
    keys_ = [np.array(k) - [CDC, 0] for k in ((-42.65, 26.55), (-42.65, -26.55), (43.25, 26.55), (43.25, -26.55))]   # receiver bolt keys (d 4)

    usb0, usb_d = USB                                        # FC USB-C cable line (12 x 7, 250 long)

    def seg_d(q, a, b):
        u = b - a; t = np.clip(((q - a) @ u) / (u @ u), 0, 1); return np.linalg.norm(q - (a + t[:, None] * u), axis=1)

    def clear(p0, p1):                                       # a link and its mirror (mirror-exact links)
        return clear1(p0, p1) and clear1(np.asarray(p0) * mir, np.asarray(p1) * mir)

    def clear1(p0, p1):
        q = np.asarray(p0)[None] + np.linspace(0.04, 0.96, 60)[:, None] * (np.asarray(p1) - np.asarray(p0))[None]
        if (seg_d(q, usb0, usb0 + 250.0 * usb_d) < 7.0 + ROD_R + 1.0).any(): return False
        if any((seg_d(q, a_, b_) < CAM['boom_sec'] / math.sqrt(2) + ROD_R + 1.0).any() for a_, b_ in BOOM_AX): return False
        if any(((q > b_[0]) & (q < b_[1])).all(1).any() for b_ in keep_out): return False
        if any(((np.linalg.norm(q[:, :2] - k, axis=1) < 2.0 + 1.0 + ROD_R) & (q[:, 2] < -20.0)).any() for k in keys_): return False
        if (q[:, 0] > ZONE[0]) .any() and ((q[:, 0] > ZONE[0]) & (q[:, 0] < ZONE[1]) & (q[:, 1] > ZONE[2]) & (q[:, 1] < ZONE[3]) & (q[:, 2] < CART_BOT)).any():
            return False
        return views_clear(p0, p1, ROD_R)

    def pick(p0, cands, n, spread_deg):
        cands = sorted(cands, key=lambda k: np.linalg.norm(nodes[k] - p0))[:10]
        cost = {k: sum(seg_shadow(p0, nodes[k], ROD_R)) + 0.002 * np.linalg.norm(nodes[k] - p0) for k in cands}
        out = []
        for k in sorted(cands, key=cost.get):
            u = (nodes[k] - p0) / np.linalg.norm(nodes[k] - p0)
            if all(np.degrees(np.arccos(np.clip(u @ v, -1, 1))) >= spread_deg for v in [(nodes[j] - p0) / np.linalg.norm(nodes[j] - p0) for j in out]):
                out.append(k)
            if len(out) == n: break
        return out
    for i, (mx, my) in enumerate(MXY):                      # 3 spokes per arm tip (under the motor, the carbon arm), props 12 off
        tip = np.array([mx, my, 18.5 - ROD_R])
        ok = [k for k in range(len(nodes)) if seg_band_dist(tip, nodes[k]) - ROD_R >= PROP_GAP and clear(tip, nodes[k])]
        for n_, k in enumerate(pick(tip, ok, 3, 30.0)):
            extra[f"spoke {i + 1}.{n_ + 1}"] = (tip, nodes[k])
    rb = REC_B                                               # bay ties: receiver bottom corners to the 2 nearest ring nodes
    for sx in (1, -1):
        for sy in (1, -1):
            cn = np.array([rb[1][0] if sx > 0 else rb[0][0], sy * rb[1][1], -34.0])
            ok = [k for k in sorted(ring_ids, key=lambda k: np.linalg.norm(nodes[k] - cn)) if clear(cn, nodes[k])]
            for n_, k in enumerate(ok[:2]):
                extra[f"bay tie {sx:+d}{sy:+d}.{n_ + 1}"] = (cn, nodes[k])
    pb = D["frame plate (Bee35 carbon)"][0].bounds           # core ties: plate nose and tail to the shell top
    for nm, q0 in (("nose", (pb[1][0] - 8.0, 0.0)), ("tail", (pb[0][0] + 8.0, 0.0))):
        q = np.array([q0[0], q0[1], 22.0 + ROD_R])
        ok = [k for k in range(len(nodes)) if k not in ring_ids and seg_band_dist(q, nodes[k]) - ROD_R >= PROP_GAP and clear(q, nodes[k])]
        for n_, k in enumerate(pick(q, ok, 2, 25.0)):
            extra[f"core tie {nm}.{n_ + 1}"] = (q, nodes[k])
    # camera guard bar: 2 printed nodes over the camera top, behind its lens (out of its view), 2 struts each into the shell
    cb = D["camera"][0].bounds; x = cb[1][0] - GUARD_NODE_R - 3.0
    T = [np.array([x, sy * (CAM_W / 2 + 4.0), cb[1][2] + 3.0 + GUARD_NODE_R]) for sy in (1, -1)]
    guard = []
    if views_clear(T[0], T[1], ROD_R):
        extra["cam guard bar"] = (T[0], T[1]); guard = T
        for i, t_ in enumerate(T):
            ok = [k for k in sorted(range(len(nodes)), key=lambda k: np.linalg.norm(nodes[k] - t_))
                  if seg_band_dist(t_, nodes[k]) - ROD_R >= PROP_GAP and views_clear(t_, nodes[k], ROD_R)
                  and all((seg_d(t_[None] + np.linspace(0.05, 0.95, 40)[:, None] * (nodes[k] - t_)[None], a_, b_) > CAM['boom_sec'] / math.sqrt(2) + ROD_R + 1.0).all() for a_, b_ in BOOM_AX)]
            for n_, k in enumerate(ok[:2]):
                extra[f"cam guard strut {i}.{n_ + 1}"] = (t_, nodes[k])
    have = [(np.asarray(a_), np.asarray(b_)) for a_, b_ in extra.values()]   # mirror-exact links
    for k_, (a_, b_) in list(extra.items()):
        a_, b_ = np.asarray(a_) * mir, np.asarray(b_) * mir
        if not any(min(np.linalg.norm(a_ - p_) + np.linalg.norm(b_ - q_), np.linalg.norm(a_ - q_) + np.linalg.norm(b_ - p_)) < 1.0 for p_, q_ in have):
            extra[k_ + " m"] = (a_, b_); have.append((a_, b_))
    return extra, guard


def masses(nodes, E, extra, guard):
    it = [(n, D[n][3], D[n][0].bounding_box.centroid) for n in D]
    it.append(("wires", 20.0, D["FC"][0].bounding_box.centroid))
    for i, (a, b) in enumerate(E): it.append((f"shell {i}", np.linalg.norm(nodes[a] - nodes[b]) * SHELL_GPM, (nodes[a] + nodes[b]) / 2))
    for k, (p, q) in extra.items(): it.append((k, np.linalg.norm(np.subtract(q, p)) * ROD_GPM, (np.asarray(p) + np.asarray(q)) / 2))
    for i, n in enumerate(nodes): it.append((f"node {i}", NODE_G, n))
    for i, n in enumerate(guard): it.append((f"guard node {i}", GUARD_NODE_G, n))
    return it


def cg(it):
    m = sum(x for _, x, _ in it); return m, sum(x * np.asarray(c, float) for _, x, c in it) / m


# ---------------------------------------------------------------- scan: structure first, then the least prop shadow; balance
def run(DC):
    """the drone with the catch DC aft of the thrust centre, its shell scan (start x dome: the most compact valid shell sets
    the bar, the least prop shadow within 10 % volume / 3 % mass of it wins), links and masses"""
    build(DC); lines = []; cands = []
    for START, DOME, SNAP in [(s_, d_, n_) for s_ in (0.0, 0.7, 0.85, 1.0) for d_ in (20.0, 40.0, 60.0) for n_ in SNAPS]:
        if START == 0.0 and DOME > 20.0: continue                 # cast onto the target: the dome does not apply
        r = shell(START, DOME, SNAP)
        if r is None: lines.append(f"start {START} dome {DOME:.0f} snap {int(SNAP)}: invalid (no closed mirror shell)"); continue
        nodes, E, tris, viol = r
        pg = min(seg_band_dist(nodes[a], nodes[b]) for a, b in E) - SHELL_R
        vol = ConvexHull(nodes).volume / 1e6; sh = sum(np.linalg.norm(nodes[a] - nodes[b]) for a, b in E)
        mass = sh * SHELL_GPM + NODE_G * len(nodes)
        ok = viol <= 0.05 and pg >= PROP_GAP
        bt, bb = blockage(nodes, E, {}) if ok else (np.nan, np.nan)
        lines.append(f"start {START} dome {DOME:.0f} snap {int(SNAP)}: {'valid' if ok else 'invalid'}: {len(nodes)} nodes, {len(E)} tubes, {vol:.1f} L, "
                     f"{np.ptp(nodes[:, 2]):.0f} tall, shell {mass:.0f} g, props {pg:.1f}, shell shadow {bt:.1f} / {bb:.1f} %")
        if ok: cands.append(dict(start=START, dome=DOME, snap=SNAP, vol=vol, mass=mass, block=bt + bb, r=r))
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
                t2=t2, c2=c2, bal=max(abs(cgv[0]), abs(c2[0])))


os.makedirs(OUT, exist_ok=True)
# balance (CLAUDE.md 4, L Balance: pack in and pack out within 3 mm): the catch set (receiver, cartridge, pack, belly mount,
# Radxa, flow) and the ring slide aft of the thrust centre by DC; the shell and the camera are rebuilt round it each time; the DC
# with the least worse of the two CG offsets wins
SNAPS = [bool(int(x)) for x in os.environ.get("SCOUT_SNAPS", "0,1").split(",")]
DCS = [float(x) for x in os.environ.get("SCOUT_DCS", "0,4,8,12,16,20,24").split(",")]
runs = {}
for DC in DCS:
    runs[DC] = run(DC); r_ = runs[DC]
    say(f"   catch {DC:4.1f} aft: CG pack in {r_['cgv'][0]:+.1f}, pack out {r_['c2'][0]:+.1f} (x, mm), {r_['tot']:.0f} g, lens ({CAM['Lx']:.0f}, {CAM['Lz']:.0f})")
for _ in range(4):                                                     # refine round the best
    b0 = min(runs, key=lambda d: runs[d]["bal"]); step = min([abs(d - b0) for d in runs if d != b0] + [4.0]) / 2
    if step < 0.5: break
    for DC in (b0 - step, b0 + step):
        if DC >= 0 and DC not in runs:
            runs[DC] = run(DC); r_ = runs[DC]
            say(f"   catch {DC:4.1f} aft: CG pack in {r_['cgv'][0]:+.1f}, pack out {r_['c2'][0]:+.1f} (x, mm), {r_['tot']:.0f} g, lens ({CAM['Lx']:.0f}, {CAM['Lz']:.0f})")
BEST = min(runs, key=lambda d: runs[d]["bal"])
R_ = run(BEST)                                                         # rebuild the globals for the pick
DC = BEST; nodes, E, tris, viol, extra, guard, it, tot, cgv = (R_[k] for k in ("nodes", "E", "tris", "viol", "extra", "guard", "it", "tot", "cgv"))
pick = R_["pick"]; ring_ids = list(range(nR))
say(f"Scout v3 layout v3 / cage v1 geodesic; ring {RING_MODE}: {nR} nodes; balance: catch {DC:.1f} aft of the thrust centre; tab line x {TAB_X:.2f}, "
    f"cartridge flat bottom z {CART_BOT:.2f}, ring plane z {Z_O:.2f} (Overseer: catch x {OV_CX:.1f}, cartridge bottom {OV_CB:.1f}, ring {RING_DZ:+.2f} under it)")
say(f"camera: lens ({CAM['Lx']:.0f}, 0, {CAM['Lz']:.0f}), head plane tilt {CAM['phi']:.0f} deg, boom {CAM['boom']:.0f} from the plate nose "
    f"(lens z {CAM_Z[0]:.0f}..{CAM_Z[1]:.0f} x tilt 0..90: the shell target {LIM + SLACK:.1f} under its lower view plane and behind its back; "
    f"least growth of the drone's box, then the shortest boom)")
for l_ in R_["lines"]: say(l_)

# ---------------------------------------------------------------- checks
sh = np.array([np.linalg.norm(nodes[a] - nodes[b]) for a, b in E]); ext = nodes.max(0) - nodes.min(0)
say(f"shell: {len(nodes)} nodes ({nR} on the ring), {len(E)} tubes, {len(tris)} triangles; tubes {sh.min():.0f} .. {sh.max():.0f}, "
    f"{sh.sum() / 1000:.2f} m; node extents {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}; hull residual {viol:.2f} (C)")
ringE = [e for e in E if e[0] < nR and e[1] < nR]
if RING_MODE == "overseer":
    dR = np.abs((nodes[:nR] - [TAB_X, 0, Z_O]) - (R_OV - [0, 0, R_OV[:, 2].mean()])).max()
    say(f"ring = the Overseer's: {nR} nodes within {dR:.3f} mm of its nodes on the same datum, {len(ringE)} ring tubes "
        f"(Overseer {len(RING_TUBES)}), extents {np.ptp(nodes[:nR, 0]):.1f} x {np.ptp(nodes[:nR, 1]):.1f} (C)")
circ = []
for f in tris:
    a_, b_, c_ = nodes[f]; la, lb, lc = np.linalg.norm(b_ - c_), np.linalg.norm(a_ - c_), np.linalg.norm(a_ - b_); s_ = (la + lb + lc) / 2
    circ.append(2 * math.sqrt(max(s_ * (s_ - la) * (s_ - lb) * (s_ - lc), 1e-9)) / s_)
say(f"largest cell: inscribed circle Ø{max(circ) - 2 * SHELL_R:.0f} between tube surfaces (C)")
rl = {k: float(np.linalg.norm(np.subtract(q, p))) for k, (p, q) in extra.items()}
short = [l for l in sh if l < BREAK_EVEN]
say(f"carbon: shell {len(sh)} tubes 6/4, shortest {sh.min():.0f}" + (f" ({len(short)} under {BREAK_EVEN:.0f}: printed into the node)" if short else "")
    + f"; links {len(rl)} rods 3.0, {sum(rl.values()) / 1000:.2f} m, shortest {min(rl.values()):.0f} ({min(rl, key=rl.get)})"
    + (f"; printed (< {BREAK_EVEN:.0f}): " + ", ".join(f"{k} {v:.0f}" for k, v in rl.items() if v < BREAK_EVEN) if any(v < BREAK_EVEN for v in rl.values()) else ""))
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
say(f"swap zone (receiver {REC_B[1][0] - REC_B[0][0]:.1f} x {REC_B[1][1] - REC_B[0][1]:.1f} + 30 a side, cartridge bottom down): "
    + ("clear (0 mm³)" if not blk else "BLOCKED by " + ", ".join(map(str, blk))))
seen = []
memb = [(nodes[a], nodes[b], SHELL_R) for a, b in E] + [(np.asarray(p, float), np.asarray(q, float), ROD_R) for p, q in extra.values()]
for k, P_ in (("shell tubes", np.vstack([rod(nodes[a], nodes[b], SHELL_R).vertices for a, b in E])), ("shell nodes", np.vstack([n + NODE_R * DIRS for n in nodes])),
              ("links + guard", np.vstack([pts(rod(p0, p1, rr), 200) for p0, p1, rr in memb[len(E):]])), ("props", props),
              ("drone", np.vstack([pts(D[n][0], 1500) for n in D if n not in ("camera", "camera mount (boom + seat)")])),
              ("camera mount", pts(D["camera mount (boom + seat)"][0], 1500))):
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
shell_g = sh.sum() * SHELL_GPM + NODE_G * len(nodes); link_g = sum(rl.values()) * ROD_GPM
t2, c2 = cg([x for x in it if not D.get(x[0], (0,) * 9)[8]])
say(f"cage: shell {shell_g:.0f} g (tubes {sh.sum() * SHELL_GPM:.0f} + nodes {NODE_G * len(nodes):.0f}, E), links {link_g:.0f} g, guard nodes "
    f"{GUARD_NODE_G * len(guard):.0f} g (C on E)")
say(f"mass ~{tot:.0f} g (E), CG ({cgv[0]:.1f}, {cgv[1]:.1f}, {cgv[2]:.1f}) vs thrust centre (0, 0): {math.hypot(*cgv[:2]):.1f} off; pack + cartridge out: "
    f"{math.hypot(*c2[:2]):.1f} off ({t2:.0f} g); CG {PROP_Z - cgv[2]:.0f} under the prop plane")
say(f"compact: shell hull {ConvexHull(nodes).volume / 1e6:.1f} L vs the parts-with-margins hull {ConvexHull(K).volume / 1e6:.1f} L (C)")
# funnel interchangeability: the Scout's shell inside the Overseer's shell in the band just above the shared ring
if RING_MODE == "overseer":
    so = json.load(open(os.path.join(OV, "cage_shell.json"))); No = np.array(so["nodes"])
    No = No - [OV_CX, 0, R_OV[:, 2].mean()] + [TAB_X, 0, Z_O]                     # the Overseer on the same datum
    eo = ConvexHull(No).equations; band = {}
    for H in (20.0, 40.0, 60.0, 80.0):
        q = np.vstack([nodes[a] + np.linspace(0, 1, 40)[:, None] * (nodes[b] - nodes[a]) for a, b in E if not (a < nR and b < nR)])
        q = q[(q[:, 2] <= Z_O + H) & (np.linalg.norm(q[:, None, :] - nodes[None, :nR], axis=2).min(1) > NODE_R)]   # off the shared ring nodes
        band[H] = float((-(q @ eo[:, :3].T + eo[:, 3])).min(1).min()) if len(q) else float("nan")
    say("funnel: Scout shell tube axes inside the Overseer's tube-axis hull (same datum, outside the shared ring nodes) by "
        + ", ".join(f"{v:.1f} up to {H:.0f} above the ring" for H, v in band.items())
        + " (C; positive = inside, same 6 mm tubes: a funnel that takes the Overseer's ring and lower shell takes the Scout's)")
else:
    band = {}

SUMMARY = dict(ring_mode=RING_MODE, tab_x=TAB_X, cart_bottom=CART_BOT, z_o=Z_O, ring=np.round(nodes[:nR], 3).tolist(), cam=dict(lens=CAM["lens"].tolist(),
               phi=CAM["phi"], boom=CAM["boom"], hfov=CAM_HFOV, vfov=CAM_VFOV), start=pick["start"], dome=pick["dome"], nodes=len(nodes), tubes=len(E),
               shell_m=float(sh.sum() / 1000), shell_g=float(shell_g), link_g=float(link_g), mass=float(tot), cg=cgv.tolist(), cg_pack_out=c2.tolist(),
               extents=ext.tolist(), prop_gap=float(pg), block_top=BT, block_bottom=BB, largest_cell=max(circ) - 2 * SHELL_R, funnel_band=band,
               links={k: [np.round(np.asarray(a_), 2).tolist(), np.round(np.asarray(b_), 2).tolist()] for k, (a_, b_) in extra.items()})
json.dump(SUMMARY, open(os.path.join(OUT, "result.json"), "w"), indent=1)
json.dump([dict(name=n_, g=round(float(m_), 3), c=np.round(c_, 2).tolist()) for n_, m_, c_ in it], open(os.path.join(OUT, "masses.json"), "w"))
SHELL_SPEC = dict(nodes=np.round(nodes, 3).tolist(), rods=[list(map(int, e)) for e in E],
                  open=[dict(name="bottom opening (pack + swap): the Overseer's ring" if RING_MODE == "overseer" else "bottom opening (pack + swap)",
                             ring=list(range(nR)))], node_r_mm=NODE_R, rod_r_mm=SHELL_R)
json.dump(SHELL_SPEC, open(os.path.join(OUT, "cage_shell.json"), "w"))

# ---------------------------------------------------------------- export: parts, assembly file
grp = {}
for n_, gm, _c in it:
    k = "shell" if n_.startswith("shell") else "nodes" if n_.startswith("node") else "guard nodes" if n_.startswith("guard node") else \
        "links" if n_.split()[0] in ("spoke", "bay", "core", "cam") else n_
    grp[k] = grp.get(k, 0.0) + gm
PARTS = [("cage shell tubes", trimesh.util.concatenate([rod(nodes[a], nodes[b], SHELL_R) for a, b in E]), "cage", False, grp["shell"], "E", "carbon", "cage", "#2b2b2b", False),
         ("cage shell nodes", trimesh.util.concatenate([trimesh.creation.icosphere(1, NODE_R).apply_translation(n) for n in nodes]), "cage", False, grp["nodes"], "E", "PETG", "cage", "#7d8a99", False),
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
LINKS = "spokes, bay ties, core ties, camera guard"
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
say(f"camera boom: {'one bar' if CAM['boom_w'] == 0 else 'a fork of 2 bars at y ±%.0f' % CAM['boom_w']}, {CAM['boom']:.0f} long, "
    f"{CAM['boom_sec']:.0f} x {CAM['boom_sec']:.0f} PETG from the plate nose to the camera seat, {CAM['boom_gap']:.1f} clear of the shell tubes (C)")
if OUT == os.path.join(ROOT, "runs", "scout_layout", "v6", ""):
    from assembly import new as new_assembly
    asm_parts = []
    for n_, m_, r_, pr_, gm_, t_, mat_, gr_, co_, sw_ in PARTS:
        p_ = dict(name=n_, role=r_, mesh=rel(os.path.join(PD, fname(n_))), mass_g=round(float(gm_), 2), tag=t_, owner="scout v3",
                  version="layout v3 / cage v1 geodesic", source="nox/scout/v3/layout_v3.py")
        if n_ in ("cage shell tubes", "cage shell nodes", LINKS):
            k = {"cage shell tubes": "shell", "cage shell nodes": "nodes", LINKS: "links"}[n_]
            pref = {"shell": "shell", "nodes": "node "}[k] if k != "links" else None
            sel = [(g_, c_) for nn, g_, c_ in it if (nn.startswith(pref) if pref else nn.split()[0] in ("spoke", "bay", "core", "cam"))]
            p_["cg"] = np.round(sum(g_ * np.asarray(c_) for g_, c_ in sel) / sum(g_ for g_, _ in sel), 2).tolist()
        if pr_: p_["protect"] = True
        if mat_: p_["material"] = mat_
        if sw_: p_["swap"] = True
        asm_parts.append(p_)
    asm_parts.append(dict(name="wires", role="structure", mass_g=20.0, tag="E", owner="scout v3", version="layout v3", source="nox/scout/v3/layout_v3.py",
                          box=np.round(np.array(D["FC"][0].bounds).T.ravel(), 2).tolist(), note="envelope = the FC's (E)"))
    contact = [["motor %d" % i, "prop %d" % i] for i in range(1, 5)] + [["motor %d" % i, "frame plate (Bee35 carbon)"] for i in range(1, 5)]
    contact += [["cage shell tubes", "cage shell nodes"], [LINKS, "cage shell nodes"], [LINKS, "cage shell tubes"], [LINKS, "frame plate (Bee35 carbon)"],
                [LINKS, "receiver v3 (latch)"], [LINKS, "mount + nose legs (Scout)"], [LINKS, "receiver TPU end pieces (latch, E)"],
                ["camera mount (boom + seat)", "camera"], ["camera mount (boom + seat)", "frame plate (Bee35 carbon)"], ["camera mount (boom + seat)", "mount + nose legs (Scout)"],
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
    if guard: contact += [[LINKS, "camera guard nodes"]]
    contact = [c_ if len(c_) == 3 else c_ + [1e9] for c_ in contact]
    openings = [dict(part=p_, axis=[0, 0, -1], half_angle_deg=40.0, min_mm=0.0, skip_probes=True)
                for p_ in SW_IGN[:1] + ["pack 4S 1500", "slider +y (latch)", "slider -y (latch)", "pack XT60 plug", "receiver v3 (latch)", "receiver lid (latch)", "flow MTF-02P"]]
    openings.append(dict(part="camera", axis=[1.0, 0.0, 0.0], half_angle_deg=CAM_CONE, field=True,
                         note="Spark 26 Oct rule: the front camera outside the cage, nothing of the drone in its view; cage first gated outside this cone"))
    asm = new_assembly("Scout v3", "scout", asm_parts,
                       note=("Layout v3 / cage v1 geodesic (27 Oct, Spark: the latest Scout layout (PR 94) with the Overseer's geodesic shell idea, the bottom "
                             "ring exactly the Overseer's so the funnel is interchangeable, one camera, the carbon base kept) from nox/scout/v3/layout_v3.py. "
                             "Rough layout: C geometry on E sizes. Drone frame: x nose, y left, z up, plate underside 18.5, thrust centre and tab line x 0. "
                             "Left out: keep-alive contacts (the new funnel's), catch features, ESC (AIO)."),
                       cg_ref=[0.0, 0.0, PROP_Z], thrust_axis=[0, 0, 1], cg_tol_mm=3.0, cage_margin_mm=3.0, openings=openings, contact_pairs=contact,
                       guard_offsets={"shell tubes to prop disks, 3D (G)": round(pg, 1)}, mass_budget_g=500, cage_shell=SHELL_SPEC, access=ACCESS)
    asm.save(os.path.join(ROOT, "nox", "scout", "v3", "scout.assembly.json"))
    say("wrote nox/scout/v3/scout.assembly.json")
with open(os.path.join(OUT, "checks.txt"), "w") as f:
    f.write(LOG.getvalue())
print("wrote", OUT)
