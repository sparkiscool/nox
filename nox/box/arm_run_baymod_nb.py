"""Drone-neighbour bay module (v4c, 17 Oct 2026; Spark 03:0x: the 30 deg and 150 deg modules, the two either side of the
drone opening). The universal v4c module (Scout receiver, back plate, lug + dovetail joint) plus 2 funnel posts up to the
collar's 45 deg seats and 1 spider-tie boss on the 45 deg diagonal. Generated for the 30 deg slot (side +1); the 150 deg module runs on its own (side -1: parts(-1) / setup(-1)):
a mirror with the joint swapped left 24 detached slivers (17 Oct), and no structure may be disconnected or deleted.
    python arm_run_baymod_nb.py [h] [gen_cm3]      (NOX_SETUP_ONLY=1: build the problem, report sizes, stop)
    17 Oct run: NOX_LOCAL=1 NOX_BACKEND=pardiso python arm_run_baymod_nb.py 2.5 80   (h 1.5 / 2.0 exceed the 7 GB box;
    multigrid ran ~10 min per iteration), then python post_nb.py. Needs arm_cad / arm_layout v4c and v2/s2_cartridge.py beside it.
Frames: W (box-arm.md). The module is drawn at the top position (module frame = W turned by -(90 - 30) about Y).
Numbers (C unless tagged):
  collar seat z 0 = W Z 144.5 (E, lowered 17 Oct 02:28), centred on W Y = Y_C 175 (C)
  funnel v6 seat (funnel_panel.py): screw entry (84 + 14/sqrt2, +-36, 1.3 + 14/sqrt2), landing 6 further along n
  n = (0.7071, 0, -0.7071) on the W +X face; post seat flat disc o16, M3 x 5.7 insert hole o4.0 x 8 along n
  post inside r 9.5 of the axis for 12 behind the landing (rib-free room), nothing in the 168 opening column (|X| < 85)
  spider-tie boss: r 146 on the 45 deg diagonal, axis Y, tie face W Y 85.5 (box legs v1), M3 x 5.7 insert o4.0 x 8
Removed drawn geometry vs the universal module: the joint on the opening side (no neighbour there)."""
import sys, os, math, json, copy
import numpy as np, manifold3d as mf
import arm_layout as L, arm_cad as A
from arm_cad import box, cylY, hull, tm, man
from arm_topo import PartProblem, run_part, log
import printopt as PO

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
GEN = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
A_MOD = 30.0
TH = -(90.0 - A_MOD)                      # W -> module frame (manifold rotate about Y, deg)
S2 = math.sqrt(2.0)

def rotv(v, deg):
    t = math.radians(deg); x, y, z = v
    return np.array([x * math.cos(t) + z * math.sin(t), y, -x * math.sin(t) + z * math.cos(t)])

def toM(m): return m.rotate([0, TH, 0])
def vM(v): return rotv(v, TH)

# ---------------------------------------------------------------- collar seats (W)
SEAT_Z0 = 144.5                           # E: seat frame z 0 in W (lowered 17 Oct 02:28)
Y_COL = L.Y_C                             # collar centred on the tab line (C)
ZB, K0 = 1.3, 84.0 - 1.3                  # funnel_panel.py
P_FACE = np.array([84.0 + 14.0 / S2, 36.0, ZB + 14.0 / S2])
N_W = np.array([1.0, 0.0, -1.0]) / S2     # screw axis, out and down (+X face)
LAND = P_FACE + 6.0 * N_W                 # (98.14, 36, 6.96) seat frame
SEATS_W = [np.array([LAND[0], Y_COL + s * LAND[1], SEAT_Z0 + LAND[2]]) for s in (-1, 1)]
SEAT_R, INS_R, INS_D, KEEP_R, KEEP_T = 8.0, 2.0, 8.0, 9.5, 12.0
POST_DRAWN_T = 10.0                       # drawn seat disc + insert boss: o16 x 10 along n (insert 8 + 2 floor)
COLLAR_BACK = K0 + (12.0 * S2) + 1.0      # x - z behind the deepest collar feature (flanges 12 behind the face) + 1 clear

def half(m, n, d):
    """keep n . p >= d (n need not be unit)"""
    n = np.asarray(n, float); k = float(np.linalg.norm(n))
    return m.trim_by_plane((n / k).tolist(), float(d) / k)

def axis_frame(p, n):
    n = n / np.linalg.norm(n); e2 = np.array([0.0, 1.0, 0.0]); e1 = np.cross(e2, n)
    return np.column_stack([e1, e2, n, p])

def along(p, n, r, t0, t1, seg=64):
    """cylinder of radius r along n from t0 to t1 measured from p"""
    c = mf.Manifold.cylinder(t1 - t0, r, r, seg).translate([0, 0, t0])
    return c.transform(axis_frame(p, n).tolist())

def seat_post(p):
    body = along(p, N_W, SEAT_R, 0.0, POST_DRAWN_T)
    hole = along(p, N_W, INS_R, -1.0, INS_D)
    return body, hole

def collar_keep():
    """everything the collar may occupy (W), conservative: the face slab from the contact face to 1 behind the deepest
    flange, z from 1 under the lip up, all Y; less the rib-free room round each post axis."""
    big = 600.0
    # seat frame x - z in [K0 - 1, COLLAR_BACK], z >= -1.2 - 1 (W Z >= SEAT_Z0 - 2.2)
    slab = box(-big, big, -big, big, SEAT_Z0 - 2.2, SEAT_Z0 + 90.0)
    slab = half(slab, [1.0, 0.0, -1.0], K0 - 1.0 - SEAT_Z0)                 # X - Z >= K0 - 1 - Z0
    slab = half(slab, [-1.0, 0.0, 1.0], -(COLLAR_BACK - SEAT_Z0))           # X - Z <= COLLAR_BACK - Z0
    room = None
    for p in SEATS_W:
        c = along(p, N_W, KEEP_R, 0.0, 40.0)
        room = c if room is None else room + c
    return slab - room

def opening_keep():
    return box(-85.0, 85.0, 0.0, 400.0, 99.0, 400.0)                       # the 168 opening column + 1, drone feet, plate

def post_env():
    """where the posts may grow (W): behind the collar, out of the opening column, near the 45 deg edge (40..62 deg),
    r 115..192; plus the r 9.5 room for 12 behind each landing."""
    reg = box(85.0, 200.0, L.MOD_Y[0], L.MOD_Y[1], 60.0, 200.0)
    reg = half(reg, [1.0, 0.0, -1.0], COLLAR_BACK - SEAT_Z0)               # X - Z >= back - Z0 (behind the collar)
    a0, a1 = math.radians(40.0), math.radians(62.0)
    reg = reg.trim_by_plane([-math.sin(a0), 0.0, math.cos(a0)], 0.0)      # angle >= 40
    reg = reg.trim_by_plane([math.sin(a1), 0.0, -math.cos(a1)], 0.0)      # angle <= 62
    ring = cylY(0, 0, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1, 192.0, 180) - cylY(0, 0, L.MOD_Y[0] - 2, L.MOD_Y[1] + 2, 115.0, 180)
    reg = reg ^ ring
    for p in SEATS_W:
        reg += along(p, N_W, KEEP_R, 0.0, KEEP_T)
    return reg

# ---------------------------------------------------------------- spider-tie boss (W)
TIE_R, TIE_ANG, TIE_Y, TIE_BOSS_R = 146.0, 45.0, 85.5, 5.0
TIE_ROOM_R = 9.5                          # generated stub round the boss, r 136.5..155.5: clear of the bars' sweep (r < 136.5, Y < 95.5)
TIE_C = (TIE_R * math.cos(math.radians(TIE_ANG)), TIE_R * math.sin(math.radians(TIE_ANG)))

def tie_boss():
    b = cylY(TIE_C[0], TIE_C[1], TIE_Y, L.MOD_Y[0] + 6.0, TIE_BOSS_R)       # 6 into the design space: 2+ cells at h 2.5 (17 Oct: at +2 it came out detached)
    h = cylY(TIE_C[0], TIE_C[1], TIE_Y - 1.0, TIE_Y + INS_D, INS_R)
    face = cylY(TIE_C[0], TIE_C[1], TIE_Y, TIE_Y + 1.6, TIE_BOSS_R)       # support region: the tie face layer
    return b, h, face


def joint_side(side):
    """the universal module's drawn joint on one side, exactly as arm_cad.bay_module_parts draws it: side +1 = tongue +
    clearance lugs, side -1 = groove block + insert lugs. Returns (bodies, cuts)."""
    g = L.JOINT_GAP; bodies = None; cuts = None
    for y in L.LUG_Y:
        rc = (L.LUG_RHO[0] + L.LUG_RHO[1]) / 2
        if side > 0:
            b = A._plane_box(+1, -6.0, -0.05, y - 5.0, y + 5.0, *L.LUG_RHO)
            b += A._dovetail(+1, y, -4.0, 0.0) + A._plane_box(+1, -6.0, -g + 0.01, y - L.DT_W1 / 2 - 3, y + L.DT_W1 / 2 + 3, *L.DT_RHO)
            c = A._plane_cyl_t(+1, -7.0, 1.0, y, rc, 1.7)
        else:
            b = A._plane_box(-1, -8.0, -0.05, y - 5.0, y + 5.0, *L.LUG_RHO)
            b += A._plane_box(-1, -7.0, -g, y - L.DT_W1 / 2 - 3.0, y + L.DT_W1 / 2 + 3.0, *L.DT_RHO)
            c = A._plane_cyl_t(-1, -9.0, 1.0, y, rc, 1.7) + A._plane_cyl_t(-1, -5.8, 0.5, y, rc, 2.0) + A._dovetail(-1, y, g + 0.5, 0.2)
        bodies = b if bodies is None else bodies + b
        cuts = c if cuts is None else cuts + c
    return bodies, cuts


def parts(side=+1):
    """drawn, cuts, env, regions in the module frame for the 30 deg module (side +1) or the 150 deg module (side -1:
    everything mirrored in X, joint swapped so it mates the 180 deg module's tongue)."""
    yc = L.Y_C; y_lo, y_hi = yc - 41.8, yc + 45.8
    rec = A._receiver_W()
    back = box(-24.3, 24.3, y_lo, y_hi, 149.5, 152.5)
    jb, jc = joint_side(+1)                         # the neighbour (0 deg module) joins on +X: tongue + clearance lugs
    posts = None; holes = None; seat_faces = []
    for p in SEATS_W:
        b, h = seat_post(p)
        posts = b if posts is None else posts + b; holes = h if holes is None else holes + h
        seat_faces.append(toM(along(p, N_W, SEAT_R - 0.6, 0.0, 1.8)))
    tb, th, tface = tie_boss()
    drawn = rec + back + jb + toM(posts) + toM(tb)
    # envelope: the universal module's sector + the posts' region; keep-outs as the universal module + the collar,
    # the opening column, the heads at the drone and at the 0 deg neighbour, the drone's pack path
    ann = cylY(0, 0, L.MOD_Y[0], L.MOD_Y[1], L.MOD_R[1], 180) - cylY(0, 0, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1, L.MOD_R[0], 180)
    env = (ann ^ A.wedge(L.HALF, L.JOINT_GAP, L.MOD_Y[0] - 1, L.MOD_Y[1] + 1)) + toM(post_env())
    env += toM(cylY(TIE_C[0], TIE_C[1], TIE_Y, L.MOD_Y[0] + 1.0, TIE_ROOM_R))   # reinforcement 1 (17 Oct): stub room round the tie boss
    keep = cylY(0, 0, 70.0, 95.5, 136.5)
    sw_bay = A.swept_head_env("A", L.Z0_A_HOME, L.Z0_BAY + 1.5) + A.swept_head_env("B", L.Z0_A_HOME, L.Z0_BAY + 1.5)
    sw_dr = A.swept_head_env("A", L.Z0_A_HOME, L.Z0_DRONE + 0.5) + A.swept_head_env("B", L.Z0_A_HOME, L.Z0_DRONE + 0.5)
    keep += sw_bay + A.pack_path_v4(L.R)
    keep += (sw_bay + A.pack_path_v4(L.R)).rotate([0, +30.0, 0])           # 0 deg neighbour (module frame +30 = W 0)
    keep += (sw_dr + A.pack_path_v4(L.R + 5.0)).rotate([0, -60.0, 0])      # drone (W 90 = module frame 150)
    keep += toM(collar_keep() + opening_keep())
    cuts = jc + keep + toM(holes) + toM(th)
    env = env - keep + drawn
    reg = dict(
        back=box(-18.0, 18.0, yc - 36.0, yc + 40.0, 149.5, 152.5),
        pockets=A._cart_to_W(20.3, 29.6, 4.2, 15.3, 29.7, 44.3) + A._cart_to_W(-29.6, -20.3, 4.2, 15.3, 29.7, 44.3),
        shoes_px=A._cart_to_W(-32.92, -20.8, 35.2, 52.0, 47.3, 68.0),
        shoes_mx=A._cart_to_W(20.8, 32.92, 35.2, 52.0, 47.3, 68.0),
        posts=rec ^ box(-30, 30, y_lo - 1, y_hi + 1, L.R + 2, L.R + 47),
        seats=seat_faces, links=jb, tie=toM(tface))
    if side < 0:                                     # 150 deg: mirror, then swap the drawn joint
        mir = lambda m: m.mirror([1, 0, 0])
        jbm, jcm = joint_side(-1)
        drawn = (mir(drawn) - mir(jb)) + jbm
        cuts = (mir(cuts) - mir(jc)) + jcm
        env = (mir(env) - mir(jb)) + jbm
        reg = {k: ([mir(x) for x in v] if isinstance(v, list) else mir(v)) for k, v in reg.items()}
        reg["links"] = jbm
    return dict(drawn=drawn, cuts=cuts, env=env, regions=reg, joint=(jb, jc))


def vec_side(v, side):
    v = np.asarray(v, float).copy()
    if side < 0: v[0] = -v[0]
    return v


def setup(side=+1, h=H, build="-y"):
    P = parts(side)
    drawn, cuts, env, R = P["drawn"], P["cuts"], P["env"], P["regions"]
    pp = PartProblem("baymod_nb" if side > 0 else "baymod_nb150", h, drawn, cuts, env, build=build, overhang=True, pad=0)
    pp.support("default", R["links"] + R["tie"])
    V = lambda v: vec_side(v, side)
    # the universal module's 11 cases (module frame, as arm_run_baymod.py)
    pp.load("insert push 20 N (TPU on the back plate)", R["back"], V((0, 0, 20)))
    pp.load("extract pull 20 N (bolts in the pockets)", R["pockets"], V((0, 0, -20)))
    pp.load("contact shoe spring +X 10 N", R["shoes_px"], V((10, 0, 0))); pp.load("contact shoe spring -X 10 N", R["shoes_mx"], V((-10, 0, 0)))
    for v, nm in (((0, 0, -20), "-z"), ((20, 0, 0), "+x"), ((0, 20, 0), "+y"), ((0, -20, 0), "-y")):
        pp.load(f"hand on the back plate 20 N {nm}", R["back"], V(v))
    for v, nm in (((6, 0, 0), "+x"), ((0, 6, 0), "+y"), ((0, 0, -6), "-z")):
        pp.load(f"pack 3 g {nm}", R["posts"], V(v))

    def multi(name, items, accel=None, cls="misuse"):
        F = np.zeros(pp.ndof)
        for region, vec in items:
            el = pp.elems(region, pp.solid | pp.design)
            if len(el) == 0: raise ValueError(f"{name}: empty region")
            nd = pp.nodes_of(el)
            for a in range(3): F[3 * nd + a] += vec[a] / len(nd)
        pp.cases.append(PO.LoadCase(name, F, cls=cls, support="default", accel=None if accel is None else np.asarray(accel, float)))

    seats = R["seats"]
    # Overseer landing (F, DEFAULT split): 1 kN on the face over these posts -> 500 N per seat along the seat normal;
    # 1 kN on any other face -> 250 N per seat (4 seats share it through the glued collar), along that face's normal
    for nm, d, per in (("+X face (over these posts)", (S2 / 2, 0, -S2 / 2), 500.0), ("-X face", (-S2 / 2, 0, -S2 / 2), 250.0),
                       ("+Y face", (0, S2 / 2, -S2 / 2), 250.0), ("-Y face", (0, -S2 / 2, -S2 / 2), 250.0)):
        v = V(vM(np.array(d) * per))
        multi(f"Overseer landing 1 kN on the {nm}", [(s, v) for s in seats])
    # 13 N rim push in 8 directions (collar rigid on 4 seats, rim 70.2 above the seats: shear F/4 + the tipping couple)
    lever = 77.2 - LAND[2]
    pts = [np.array([sx * LAND[0], sy * LAND[1]]) for sx in (-1, 1) for sy in (-1, 1)]
    own = [np.array([LAND[0], -LAND[1]]), np.array([LAND[0], LAND[1]])]          # these seats (W, before mirroring)
    for k in range(8):
        a = math.radians(45.0 * k); d = np.array([math.cos(a), math.sin(a)])
        M = 13.0 * lever; den = sum(float(d @ p) ** 2 for p in pts)
        items = []
        for s, p in zip(seats, own):
            fz = -M * float(d @ p) / den
            items.append((s, V(vM(np.array([13.0 / 4 * d[0], 13.0 / 4 * d[1], fz])))))
        multi(f"rim push 13 N toward {45 * k} deg", items)
    # box weight 1 g: own material (accel), collar 543 g on 4 seats (C / E), Scout pack + cartridge 0.19 kg in the bay (E)
    g = V(vM(np.array([0.0, 0.0, -1.0])))
    multi("box weight 1 g (own mass, collar share, pack)", [(s, 0.543 * 9.81 / 4 * g) for s in seats] +
          [(R["posts"], 0.19 * 9.81 * g)], accel=g)
    return pp, P


if __name__ == "__main__":
    SIDE = int(os.environ.get("NOX_SIDE", "1"))        # 1: 30 deg module, -1: 150 deg module (own run)
    pp, P = setup(SIDE)
    pb = pp.problem()
    log(f"cases {len(pb.cases)}; active cells {int((~pp.void).sum()):,}")
    drawn, cuts, env = P["drawn"], P["cuts"], P["env"]
    bb = env.bounding_box(); log(f"env bbox {[round(v, 1) for v in bb]}")
    os.makedirs("out", exist_ok=True)
    tm(drawn - cuts).export("out/nb_drawn.stl"); tm(env - cuts).export("out/nb_allowed.stl")
    if os.environ.get("NOX_SETUP_ONLY"):
        sys.exit(0)
    if os.environ.get("NOX_LOCAL"):                    # big grid: multigrid backend, same formulation
        import noxopt as NO, noxopt_local as NL
        import functools
        NO.Layout = functools.partial(NL.Layout, backend=os.environ.get("NOX_BACKEND", "pardiso"))   # no auto calibration: it holds both solvers at once (OOM at h 2.0, 17 Oct)
    res, mesh = run_part(pp, drawn, cuts, env, GEN, "out/box_baymod_nb_v1.stl" if SIDE > 0 else "out/box_baymod_nb150_run.stl")
    if res["bodies"] != 1:
        raise SystemExit(f"{res['bodies']} bodies: detached structure, rerun (never delete)")
