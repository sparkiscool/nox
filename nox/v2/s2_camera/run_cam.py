"""S2 camera mount v2 (Scout v2), 17 Oct 2026. Copy of noxopt_demo.py with the part's own problem, drawn bodies, cuts and
envelope; every pipeline step in order.   python run_cam.py [h, default 1.0] [generated target cm3, default 1.15]
Needs noxopt.py, printopt.py beside it or on PYTHONPATH (~/nox). Pose from cam_place.py (cam_place.json, C).
v2 (decision 17, Spark 17 Oct 15:50): tilt 4 deg (v1 13); prop plane 41.2 (+0.7 z: prop keep-outs and the collar face
move with it); receiver v3.3 features kept out: its flow seat (land top z -14, board 1.0 above it), the tail half of its
nose cup (tube r 5.5..7.5 round the bolt, x <= 45) and the flow-sensor lead chase (|y| 6..10, x 42.2..46.2); the pad
shrinks to r 5.25 so it sits in the receiver's r 5.5 nut tube with 0.25 clear (hex wall 2.02, C). Upper boss tails past
6.0 (insert 5 + 1 floor) are trimmed by the plate keep-out (DEFAULT v2: the 8 bosses (E) otherwise put the lens 1.4 behind
the collar limit).

Part: one PETG piece, no supports. Camera DECXIN 2633V2 (board 38 x 38, M2 on 28 x 28, P) on 4 bosses with M2 x 4
heat-set inserts, screws from the front; hangs from the BOTTOM of the nose plate-bolt stack (45, 0): pad top z 16 against
the receiver pad (E), M3 nyloc captive in a hex pocket in the pad (bolt +2.0).
Why the nose bolt (constrained search over the 6 candidates, C): on top of a top case pad (nose or side) the pad + head
sit 2.6 / 2.2 inside the 12 prop keep-out; under the side bolts is the pack + receiver (Problem 31). Only the nose bolt
from below is feasible, and it is the closest point to the camera.
Frames. Drone: x fwd, y left, z up. Board: n = board normal (4 deg up), b = board up, seat plane = board back face,
centre c (cam_place.json). Grid frame: X = b, Y = y, Z = -n (Z 0 = seat plane). Build +X: the bed is the plane under
the two lower bosses (board 'down' side, flats drawn on them), layers stack along b (4 deg off drone up), so pillars
between the bosses print vertical and the struts to the pad rise at ~45 deg; the mirror plane Y 0 contains the build
axis. The STL is exported in the print frame (x = y, y = -n, z = b - bed).
Camera loads go onto the boss seat faces as least-norm force sets carrying the force and its moment.
Mass E: camera 6 g, CG 5 in front of the seat plane. Lens 20 long from the board front, plug room 16 x 16 x 8 behind,
2 back clearance under the board, bosses 8 tall (E, CHECK on the camera)."""
import sys, os, copy, json
import numpy as np, trimesh, manifold3d as mf
import noxopt as NO, printopt as PO

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
GEN_CM3 = float(sys.argv[2]) if len(sys.argv) > 2 else 1.15
VER = os.environ.get("VER", "v2")
FLOOR, CLIP = 2.0, 0.5
log = lambda s: print(s[:300], flush=True)
HERE = os.path.dirname(os.path.abspath(__file__))

# ---- pose (C, cam_place.py) and frames
P0 = json.load(open(os.path.join(HERE, "cam_place.json")))
T = np.radians(P0["tilt"]); C0 = np.array([P0["cx"], 0.0, P0["cz"]])
Nn = np.array([np.cos(T), 0, np.sin(T)]); Bb = np.array([-np.sin(T), 0, np.cos(T)]); Yy = np.array([0.0, 1, 0])
R = np.stack([Bb, Yy, -Nn])                              # drone vector -> print vector
to_print = lambda p: (np.asarray(p, float) - C0) @ R.T
M34 = np.hstack([R, (-R @ C0)[:, None]])                 # manifold transform drone -> print

# ---- camera and mount numbers
HOLE, BOSS_R, BOSS_H, INS_R, INS_D = 14.0, 3.6, 8.0, 1.6, 5.0     # M2 on 28 x 28 (P); insert hole 3.2 (E), 5 deep
CAM_M, CAM_CG, LENS_W = 0.006, 5.0, 21.6                          # kg, mm in front of the seat plane, lens front (E)
PLUG = (8.0, 8.0, 8.0); BACK = 2.0                             # plug room half u, half v, depth; back clearance (E)
PAD = dict(x=45.0, z_top=16.0, r=5.25, h=6.2, web=2.0, hex_af=5.6, hole_r=1.65)   # nose bolt stack bottom (E / C)
PROP_Z = (37.2, 45.2)                                            # prop plane 41.2 +-4 (C)
COLLAR = 81.27                                                   # x - z limit, 2.0 off the collar face (C)
FS = (46.7, 70.5, 9.5, -14.0); FS_CLR = 1.0                      # receiver flow seat x0, x1, |y|, land top; gap (C / DEFAULT)

# ---- grid in the print frame, centred on the mirror plane Y = 0
U0, U1, ZP1 = -17.6, 21.6, 25.6                      # U0 = bottom of the lower bosses = the bed
nx, ny, nz = int(round((U1 - U0) / H)), 2 * int(round(19.2 / H)), int(round(ZP1 / H))
shape = (nx, ny, nz); origin = (U0, -ny * H / 2, 0.0)
xs, ys, zs = [np.arange(n) * H + o + H / 2 for n, o in zip(shape, origin)]

# ---- exact bodies: drawn (print frame), cuts and keep-outs (drone frame -> print frame), envelope
box = lambda x0, x1, y0, y1, z0, z1: mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])
cyl = lambda x, y, z0, z1, r, n=48: mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])
def tm(m):
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=True)
def man(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
tp = lambda m: m.transform(M34)                          # drone-frame body -> print frame

bosses = [(su * HOLE, sv * HOLE) for su in (-1, 1) for sv in (-1, 1)]
z_bot = PAD["z_top"] - PAD["h"]
hexa = mf.CrossSection.circle(PAD["hex_af"] / np.sqrt(3), 6)        # vertex on +x: gothic roof in print (up = -x)
drawn = tp(cyl(PAD["x"], 0, z_bot, PAD["z_top"], PAD["r"], 64))
def boss(u, v, z0, z1):
    """boss column (grid frame, axis along Z): lower bosses get a flat to the bed, upper ones a 45 deg teardrop roof
    under them (build +X), so neither needs support."""
    if u < 0: return cyl(u, v, z0, z1, BOSS_R) + box(U0, u, v - BOSS_R, v + BOSS_R, z0, z1)
    d = BOSS_R * np.sqrt(2)
    dia = mf.Manifold.cube([2 * BOSS_R, 2 * BOSS_R, z1 - z0], center=True).rotate([0, 0, 45]).translate([u, v, (z0 + z1) / 2])
    return cyl(u, v, z0, z1, BOSS_R) + (dia ^ box(u - d, u, v - BOSS_R, v + BOSS_R, z0, z1))
for u, v in bosses: drawn += boss(u, v, 0, BOSS_H)
cuts = tp(cyl(PAD["x"], 0, z_bot - 1, PAD["z_top"] + 1, PAD["hole_r"], 32)
          + mf.Manifold.extrude(hexa, PAD["h"] - PAD["web"] + 1).translate([PAD["x"], 0, z_bot - 1])
          + cyl(PAD["x"], 0, z_bot - 6.0, z_bot + 0.3, 3.5))             # nut + bolt-tail lane below the pad (v2: 0.3 into
                                                                          # the pad, a lane top flush with the pad bottom left a
                                                                          # non-manifold edge at the hex corners)
for u, v in bosses: cuts += cyl(u, v, -1, INS_D, INS_R, 32)
keep = box(-PLUG[0], PLUG[0], -PLUG[1], PLUG[1], -1, PLUG[2])            # plug room (print frame)
slab = box(-30, 30, -30, 30, -1, BACK)
for u, v in bosses: slab -= boss(u, v, -2, BACK + 1)
keep += slab                                                              # back clearance under the board
ko = box(-200, 52.0, -60, 60, PAD["z_top"], 120) + box(-200, 39.0, -60, 60, -100, 120)   # plate / stack / receiver
for px, py in [(54.1, 54.1), (54.1, -54.1)]:                              # front props: sweep cylinder offset 12
    tor = mf.Manifold.revolve(mf.CrossSection.circle(12.0, 32).translate([44.45, 0]), 96)
    ko += (cyl(0, 0, PROP_Z[0], PROP_Z[1], 56.45, 96) + cyl(0, 0, PROP_Z[0] - 12, PROP_Z[1] + 12, 44.45, 96)
           + tor.translate([0, 0, PROP_Z[0]]) + tor.translate([0, 0, PROP_Z[1]])).translate([px, py, 0])
ko += mf.Manifold.cube([400, 400, 400]).translate([0, -200, -200]).rotate([0, 45, 0]).translate(
    list(COLLAR / np.sqrt(2) * np.array([1, 0, -1]) / np.sqrt(2)))       # collar face, 2.0 off (x - z > 81.27)
# receiver v3.3 (C, receiver_v33.py): flow seat + gap, tail half of the nose cup tube, flow-sensor lead chase
ko += box(FS[0] - FS_CLR, FS[1] + FS_CLR, -FS[2] - FS_CLR, FS[2] + FS_CLR, -100, FS[3] + FS_CLR)
ko += (cyl(PAD["x"], 0, 4.0, 18.5, 8.0, 64) - cyl(PAD["x"], 0, 3.0, 19.5, 5.5, 64)) ^ box(-200, PAD["x"], -60, 60, -100, 120)
for s in (-1, 1):
    ko += box(41.7, 46.7, min(s * 5.5, s * 10.5), max(s * 5.5, s * 10.5), -100, 18.5)
env = box(U0, U1, origin[1], -origin[1], 0, ZP1); MIRROR_Y = 0.0
grid_box = box(U0 - 3, U1 + 3, origin[1] - 3, 3 - origin[1], -3, ZP1 + 3)   # cuts end near the grid: the ray voxeliser
cuts = (cuts + keep + tp(ko)) ^ grid_box                                  # needs ray origins (Z -5) outside every body
allowed = env - cuts

# ---- problem
cen = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), -1).reshape(-1, 3)
vox = lambda m: NO._voxelise(tm(m), xs, ys, zs).ravel()
solid = vox(drawn - cuts); void = vox(cuts) | ~vox(env)
so = solid.reshape(shape); vo = void.reshape(shape)
solid = (so & so[:, ::-1, :]).ravel(); void = (vo | vo[:, ::-1, :]).ravel() & ~solid
nel = solid.size; ndof = 3 * (nx + 1) * (ny + 1) * (nz + 1)
nid = lambda i, j, k: (i * (ny + 1) + j) * (nz + 1) + k
def nodes_of(mask):
    el = np.where(mask)[0]; ex, ey, ez = np.unravel_index(el, shape)
    return np.unique(np.concatenate([nid(ex + a, ey + b, ez + c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]))
dofs = lambda nd: (nd[:, None] * 3 + np.arange(3)).ravel()
cd = C0[None, :] + cen @ R                                 # cell centres in drone coords
pad_cells = solid & (np.hypot(cd[:, 0] - PAD["x"], cd[:, 1]) < PAD["r"] + H) & (cd[:, 2] > z_bot - H)
sup_nodes = nodes_of(pad_cells & (cd[:, 2] > PAD["z_top"] - 1.5 * H))       # pad top face, clamped by the bolt
seat = [nodes_of(solid & (np.hypot(cen[:, 0] - u, cen[:, 1] - v) < BOSS_R + H) & (cen[:, 2] < 2 * H)) for u, v in bosses]
rb = np.array([[u, v, 0.0] for u, v in bosses])

def cam_load(Fp, at):
    """force Fp (print frame, N) acting at point `at` (print frame) -> least-norm boss forces with the same force and
    moment, spread over each boss's seat nodes."""
    A = np.zeros((6, 12)); rhs = np.concatenate([Fp, np.cross(at, Fp)])
    for i, r in enumerate(rb):
        A[:3, 3 * i:3 * i + 3] = np.eye(3)
        A[3:, 3 * i:3 * i + 3] = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    f = np.linalg.lstsq(A, rhs, rcond=None)[0].reshape(4, 3)
    F = np.zeros(ndof)
    for nd, fi in zip(seat, f):
        for a in range(3): F[3 * nd + a] += fi[a] / len(nd)
    return F
CG = np.array([0, 0, -CAM_CG]); LENS = np.array([0, 0, -LENS_W]); MID = np.zeros(3)
g_case = lambda nm, a, cls: PO.LoadCase(nm, cam_load(R @ (CAM_M * 9.81 * np.asarray(a, float)), CG), cls=cls,
                                        support="pad", accel=R @ np.asarray(a, float))
cases = [g_case("flight 4g+2g", (0, 2, -4), "flight"), g_case("landing 6g", (0, 0, -6), "misuse")]
for nm, a in [("crash +x", (25, 0, 0)), ("crash -x", (-25, 0, 0)), ("crash +y", (0, 25, 0)), ("crash +z", (0, 0, 25)),
              ("crash -z", (0, 0, -25))]:
    cases.append(g_case(nm + " 25g", a, "misuse"))
for nm, Fp, at in [("lens push back 10N", R @ (-10 * Nn), LENS), ("lens side 5N", R @ (5 * Yy), LENS),
                   ("lens up 5N", R @ (5 * Bb), LENS), ("lens down 5N", R @ (-5 * Bb), LENS),
                   ("plug push 10N", R @ (10 * Nn), MID), ("plug pull 10N", R @ (-10 * Nn), MID)]:
    cases.append(PO.LoadCase(nm, cam_load(Fp, at), cls="misuse", support="pad"))
pb = PO.Problem(shape=shape, h=H, origin=origin, solid=solid, void=void, supports=dict(pad=dofs(sup_nodes)), cases=cases,
                material=PO.MATERIALS["PETG"], build="+x", overhang=True, R=max(2.0, 2.4 / H), comp_factor=None)
pb0 = copy.copy(pb)
ndes = int((~solid & ~void).sum()); VF = GEN_CM3 * 1000 / (ndes * H ** 3)
log(f"grid {shape} h {H}: {nel:,} cells, design {ndes:,}, drawn {solid.sum() * H**3 / 1000:.2f} cm3 (voxel), "
    f"{len(cases)} cases, volfrac {VF:.4f} for {GEN_CM3} cm3 generated")

def build(dens):
    surf = NO.smooth_surface(dens, pb0.solid, pb0.void, pb0.shape, H, pb0.origin, keep_mesh=tm(drawn),
                             clip_mesh=tm(allowed), clip_margin=CLIP, sigma_mm=0.6 * H, floor_mm=FLOOR, log=log)
    S = man(surf)
    if MIRROR_Y is not None:
        half = S ^ box(-1e3, 1e3, -1e3, MIRROR_Y, -1e3, 1e3)
        S = half + half.mirror([0, 1, 0]).translate([0, 2 * MIRROR_Y, 0])
    return ((S + drawn) ^ env) - cuts

if __name__ == "__main__":
    # ---- 1 layout, 2 prune
    L = NO.Layout(pb, VF, log=log, masked=False)     # unmasked filter: no thin skins along the keep-outs
    dens = L.run(checkpoint="cam_ck.npz", resume="cam_ck.npz", vol_gain=0.3)
    L, dens, removed = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, vol_gain=0.3)
    np.save("cam_dens.npy", dens)
    # ---- 3 build, 4 thin check (hits back into the prune, at most 2 times)
    for attempt in range(3):
        con = NO.connectivity(dens, L.pb0); assert con["ok"], con
        part = build(dens); P = tm(part)
        thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
        if not thin or attempt == 2: break
        L, dens, _ = NO.prune_thin(L, dens, floor_mm=FLOOR, margin_mm=CLIP + 0.25, log=log, built=thin, vol_gain=0.3)
    np.save("cam_dens_final.npy", dens)
    # ---- 5 checks
    bodies = len(part.decompose())
    fc = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
    wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
    A = NO._voxelise(P, xs, ys, zs).ravel().astype(float)
    pbs = copy.copy(pb0); pbs.void = (A < 0.5) & ~pb0.solid; pbs.solid = np.zeros_like(pb0.solid)
    rows, _ = NO.strength_report(pbs, np.where(pb0.solid, 1.0, A))
    w = max(rows, key=lambda r: max(r["inlayer"], r["across"]))
    P.export(f"s2_camera_mount_{VER}_grid.stl")                                                     # grid frame (checks)
    Q = P.copy(); Q.vertices = np.stack([Q.vertices[:, 1], Q.vertices[:, 2], Q.vertices[:, 0] - U0], 1)  # print frame
    Q.export(f"s2_camera_mount_{VER}.stl")
    json.dump(dict(rows=rows, fc=list(fc), wrinkle=list(wr), thin=len(thin), bodies=bodies, removed=removed,
                   mass_g=P.volume / 1000 * 1.27), open("cam_result.json", "w"), indent=1)
    print(f"RESULT bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.2f} g | function check "
          f"{fc[0]:.1f}/{fc[1]:.1f}/{fc[2]:.1f} | thin {len(thin)} | wrinkle {wr[0]:.2f}/{wr[1]:.2f} | worst "
          f"{max(w['inlayer'], w['across']):.2f} in {w['case']}")
    ok = bodies == 1 and P.is_watertight and max(fc) < 0.5 and not thin
    print("PASS" if ok else "FAIL: not ready to print"); sys.exit(0 if ok else 1)
