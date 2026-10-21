"""S2 camera mount: camera pose search (v2, 17 Oct 2026).   python cam_place.py  ->  cam_place.json   (TILT env, default 4)
v2 (Spark 17 Oct 15:50, decision 17): tilt 4 deg up (was 13); prop plane z 41.2 (D12 19.2, was 40.5: everything tied to the
props and the cage moves +0.7 z, current-parts Problem 33); receiver v3.3 flow seat kept out (under the camera board).
Maximise the lens front x (drone x, on the axis) over the board's seat-plane centre (cx, cz); y 0 (mirror).
Hard constraints, every sampled point of the camera, its 4 bosses and its plug room:
  collar   2.0 off the 45 deg face (x - z <= 81.27, seat frame: the cage's lower ring moved +0.7, so the seated drone sits
           0.7 lower in the collar, collar bottom edge now drone z -0.1, Problem 33; v1 used 81.97)
  props    12 (3D) from each prop sweep cylinder (r 44.45, z 37.2..45.2, hubs (+-54.1, +-54.1))
  guard    3 off the nose-flat ring rods (x 112: lower z 33.2 r 1.25, upper z 49.2 r 1.0, cage v2 rings, E for x)
  nose     1 off the nose bolt stack (receiver pad z 16..18.5, plate, top case pad z 22..25, head z 25..28, all
           Ø9.5 / Ø5.5 at (45, 0)) and off the plate (nose edge x 51 E, z 18.5..22)
  flow     the receiver v3.3 flow seat (C, receiver_v33.py): land top z -14 over x 46.7..70.5, |y| <= 9.5; board and
           bosses 1.0 above it (DEFAULT v2: the receiver's 1.2 left the 38 board 0.03 short of the props band)
  bosses   the nose / plate check skips each boss past 6.0 (insert 5 + 1 floor): run_cam.py's plate keep-out trims
           that tail (DEFAULT v2; reported as boss_trim)
Reported: how far the camera and mount leave the receiver v3.3 camera keep-out (in_keep_depth: hull of its nose pad
front half r 6.5 z 6..16 + box x 54..70, |y| <= 21, z -13..26). The receiver chat redraws that zone from cam_place.json.
Reported, not a constraint: the lower nose ring in the image (top of frame = tilt + 33.2 deg, E).
Camera (A: board 38 x 38, M2 on 28 x 28, P); lens holder 16 x 16 x 8.4, barrel Ø14 to 20 in front of the board,
board 1.6, plug room 16 x 16 x 8 behind the board centre, bosses 8 (all E, CHECK; same as run_cam.py)."""
import json, os, numpy as np
from scipy.spatial import ConvexHull

T = np.radians(float(os.environ.get("TILT", 4.0)))
N = np.array([np.cos(T), 0, np.sin(T)])            # board normal (forward, up)
B = np.array([-np.sin(T), 0, np.cos(T)])           # board 'up'
Y = np.array([0.0, 1, 0])
BOARD, HOLE, BOSS_R, BOSS_H = 38.0, 28.0, 3.6, 8.0
BOSS_KEEP = 6.0      # insert 5 deep + 1 floor (E): the boss tail past this may be trimmed by the plate keep-out (DEFAULT v2)
LENS_BOX, LENS_HOLDER, LENS_R, LENS_L, PCB = 16.0, 8.4, 7.0, 20.0, 1.6
PLUG = (8.0, 8.0, 8.0)                             # half u, half v, depth behind the board (E)
PROPS = [(54.1, 54.1), (54.1, -54.1), (-54.1, 54.1), (-54.1, -54.1)]
PROP_Z = (37.2, 45.2)                              # prop plane 41.2 +-4 (C, dimensions C)
RING = ((33.2, 1.25), (49.2, 1.0))                 # cage v2 nose rods at x 112 (z C, x E)
COLLAR = 81.27                                     # x - z limit, 2.0 off the face (C)
FOV_HALF_V = 33.2                                  # vertical half FOV (E)

# receiver v3.3 camera keep-out (drone coords, C from receiver_v33.py)
_pad = [(45 + 6.5 * np.cos(t), 6.5 * np.sin(t), z) for t in np.linspace(-np.pi / 2, np.pi / 2, 13) for z in (6.0, 16.0)]
_box = [(x, y, z) for x in (54.0, 70.0) for y in (-21.0, 21.0) for z in (-13.0, 16.0)]
_box2 = [(x, y, z) for x in (54.0, 70.0) for y in (-21.0, 21.0) for z in (-13.0, 26.0)]
HULLS = [ConvexHull(np.array(_pad + _box)).equations, ConvexHull(np.array(_box2)).equations]
RBOX = ((-48.5, 71.5), (-39.0, 39.0), (-45.0, 18.5))
FS, FS_CLR = (46.7, 70.5, 9.5, -14.0), 1.0        # flow seat x0, x1, |y|, land top z (C); board-to-land gap 1.0 (DEFAULT v2: 1.2 left 0.03 short of the props band)


def in_keep_depth(P):
    """depth inside the receiver's camera keep-out (>= 0 inside), per point."""
    return np.max([-(P @ E[:, :3].T + E[:, 3]).max(1) for E in HULLS], 0)


def in_rbox(P):
    return np.all([(P[:, i] > lo) & (P[:, i] < hi) for i, (lo, hi) in enumerate(RBOX)], 0)


def grid(a0, a1, b0, b1, c0, c1, s=1.0):
    g = [np.linspace(lo, hi, max(2, int(np.ceil((hi - lo) / s)) + 1)) for lo, hi in ((a0, a1), (b0, b1), (c0, c1))]
    A, Bb, C = np.meshgrid(*g, indexing="ij"); return np.stack([A.ravel(), Bb.ravel(), C.ravel()], 1)


def cam_points(s=1.0):
    """points in board coords (u along B, v along Y, w along N; w 0 = board back = seat plane), and a barrel mask."""
    P = [grid(-19, 19, -19, 19, 0, PCB, s),
         grid(-LENS_BOX / 2, LENS_BOX / 2, -LENS_BOX / 2, LENS_BOX / 2, PCB, PCB + LENS_HOLDER, s)]
    a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    bar = []
    for w in np.linspace(PCB + LENS_HOLDER, PCB + LENS_L, 8):
        bar.append(np.stack([LENS_R * np.cos(a), LENS_R * np.sin(a), np.full_like(a, w)], 1))
    P.append(grid(-PLUG[0], PLUG[0], -PLUG[1], PLUG[1], -PLUG[2], 0, s))
    for su in (-1, 1):
        for sv in (-1, 1):
            for w in np.linspace(-BOSS_H, 0, 17):
                P.append(np.stack([su * HOLE / 2 + BOSS_R * np.cos(a), sv * HOLE / 2 + BOSS_R * np.sin(a), np.full_like(a, w)], 1))
    P, bar = np.concatenate(P), np.concatenate(bar)
    lab = np.r_[np.where(P[:, 2] < -BOSS_KEEP - 1e-6, 2, 0), np.ones(len(bar), int)]
    return np.concatenate([P, bar]), lab


def to_drone(Pb, c):
    return c[None, :] + Pb[:, :1] * B + Pb[:, 1:2] * Y + Pb[:, 2:3] * N


def margins(Pd, lab):
    x, y, z = Pd.T
    m = {}
    m["collar"] = ((z + COLLAR + 2.0 * np.sqrt(2) - x) / np.sqrt(2) - 2.0).min()
    d = []
    for px, py in PROPS:
        rp = np.hypot(x - px, y - py) - 44.45
        dz = np.maximum(0, np.maximum(PROP_Z[0] - z, z - PROP_Z[1]))
        d.append(np.hypot(np.maximum(rp, 0), dz) - np.where((rp < 0) & (dz == 0), 1e3, 0))
    m["props"] = np.min(d, 0).min() - 12.0
    m["guard"] = min((np.hypot(x - 112, z - zr) - rr - 3.0).min() for zr, rr in RING)
    rr = np.hypot(x - 45, y)
    ds = [np.hypot(np.maximum(rr - r0, 0), np.maximum(0, np.maximum(z0 - z, z - z1))) - 1.0
          for r0, z0, z1 in [(9.5 / 2, 16, 25), (5.5 / 2, 25, 28)]]
    ds.append(np.where((z > 16.0) & (z < 23), x - 52.0, 1e3))   # plate nose edge x 51 (E) + 1; z 16 = run_cam keep-out
    m["nose"] = np.min(ds, 0)[lab != 2].min()
    m["boss_trim"] = -max(0.0, -np.min(ds, 0)[lab == 2].min())    # reported: how deep the plate trims a boss tail

    # receiver v3.3 flow seat (C, receiver_v33.py): land top z -14 over x 46.7..70.5, |y| <= 9.5 (rim incl.), 1.2 clear
    fs = (x > FS[0] - FS_CLR) & (x < FS[1] + FS_CLR) & (np.abs(y) < FS[2] + FS_CLR)
    m["flow_seat"] = np.where(fs, z - (FS[3] + FS_CLR), 1e3).min()
    return m


if __name__ == "__main__":
    Pb, lab = cam_points(1.0)
    ok = lambda c: min(v for k, v in margins(to_drone(Pb, c), lab).items() if k != "boss_trim") >= 0
    best = None; rows = []
    for cz in np.arange(-10, 20.01, 0.25):
        lo = None                                              # feasible cx need not be one interval: scan down
        for cx in np.arange(130.0, 20.0, -0.25):
            if ok(np.array([cx, 0, cz])): lo = cx; break
        if lo is None: continue
        hi = lo + 0.25
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if ok(np.array([mid, 0, cz])) else (lo, mid)
        c = np.array([lo, 0, cz]); lens = c + (PCB + LENS_L) * N
        m = margins(to_drone(Pb, c), lab); m_ = {k: v for k, v in m.items() if k != "boss_trim"}; act = min(m_, key=m_.get)
        rows.append((cz, lo, lens[0], act))
        if best is None or lens[0] > best[2] + 1e-6: best = (cz, lo, lens[0], m)
    cz, cx, lx, m = best
    c = np.array([cx, 0, cz]); lens = c + (PCB + LENS_L) * N
    Pd = to_drone(Pb, c)
    ring = np.degrees(np.arctan2(RING[0][0] - RING[0][1] - lens[2], 112 - lens[0]))
    top = np.degrees(T) + FOV_HALF_V
    env_ = Pd[lab != 1]
    out = dict(version="v2", cx=round(cx, 2), cz=round(cz, 2), tilt=float(np.degrees(T)),
               lens_front=[round(v, 2) for v in lens], board_centre=[round(v, 2) for v in c + PCB / 2 * N],
               board_bottom_z=round(float((c + PCB / 2 * N - 19 * B)[2]), 2),
               margins={k: round(float(v), 2) for k, v in m.items()},
               ring_elev_deg=round(ring, 1), fov_top_deg=round(top, 1), fov_ring_clear_deg=round(ring - top, 1),
               camera_envelope=dict(x=[round(float(env_[:, 0].min()), 1), round(float(env_[:, 0].max()), 1)],
                                    y=round(float(np.abs(env_[:, 1]).max()), 1),
                                    z=[round(float(env_[:, 2].min()), 1), round(float(env_[:, 2].max()), 1)]),
               lens_barrel=dict(x=[round(float(Pd[lab == 1, 0].min()), 1), round(float(Pd[lab == 1, 0].max()), 1)],
                                z=[round(float(Pd[lab == 1, 2].min()), 1), round(float(Pd[lab == 1, 2].max()), 1)]))
    for cz_, cx_, lx_, a in rows[::10]: print(f"cz {cz_:6.1f}  cx {cx_:6.2f}  lens x {lx_:6.2f}  binding {a}")
    print(json.dumps(out, indent=1))
    json.dump(out, open("cam_place.json", "w"), indent=1)
