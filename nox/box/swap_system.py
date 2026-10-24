"""Box swap arm: system model v6 (24 Oct 2026, swap arm system worker; docs/generative.md The rule 0, system first).
One model of the whole swap arm in its assembled pose: stub, sun post, 6010s, hub, 3 NEMA17s, 80T + loop + 20T, rails A / B, carriages
(plate, wheels, axle stacks, eccentric bushes, rack webs), 16T pinions, heads (requirement envelopes until heads v6), the detent
(ring, arm, roller, springs, tower), the carried cartridge (envelope from dimensions E: cartridge v4 of PR 44; nox/box/battery_latch.py
latch_assembly() replaces it when it lands), the bay modules and the drone seat as context; posed through the swap cycle by
arm_kinematics (KEYS_V5). On it:
  stack     the axial-stack optimisation (arm_v6_layout.stack_search: rails from the posts to the plate) and the assembled bounding
            volume / largest extent of the arm (v5 posts vs v6 compact) at the demo Y_C 175 and at the compact modules' Y_C;
  frame     a 3D frame model (beams + contact springs, numpy) of the assembled arm: stub -> 6010s -> hub -> rail pads -> rails ->
            V-wheel contacts -> carriage -> head -> cartridge, motors on their seats, rack on pinion, the axle loop; the system cases
            act where they act (XT60 at the cartridge's lead end, masses at their CGs); out come the joint reactions per case, the
            pack deflection, the piece indices (hand-section stresses, E) and the stiffness each joint must keep;
  overseer  can the v6 stroke and stack carry the Overseer pack (148 x 48 x 45, P) in a bay of its size;
  manifest  nox/system_check.py manifests per pose (fit, compact, mass / CG).
Coordinates: the arm frame (W at phi 90: X tangential, Y axial, Z along head A's rail). Tags: C geometry, E loads / stiffness / masses
unless stated, M meshes. python3 swap_system.py [stack|frame|overseer|check|all] -> runs/arm/v6/system.json (+ system_check_*.json)"""
import os, sys, json, math, copy
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import arm_v6_layout as S6
S6.use("v6")
import arm_layout as L, arm_v5_layout as V, detent as DT, arm_cad as A_

REPO = os.path.normpath(os.path.join(HERE, "..", "..")); OUT = os.path.join(REPO, "runs", "arm", "v6"); os.makedirs(OUT, exist_ok=True)
G = 9.81e-3                               # N per g (mass in g)

# ================================================================== cartridges (envelopes, cartridge coords x_c, y_c up, z_c along the pack)
CART = {
    # dimensions E (PR 44, cartridge v4): box x +-19 + rails to +-20, y 0..48, z 0..93; tab line z 37.5 = Y_C = drone x 0 (pack CG on the
    # thrust axis); XT60 axis (0, z 85.8) face y 46 at seat; set-screw boss 3.0 proud of the end face; 38.5 g (M) + pack 165 g (P)
    "scout": dict(x=20.0, h=48.0, l=93.0, l_boss=3.0, tab=37.5, xt60_z=85.8, xt60_y=46.0, cg=(0.0, 31.0, 37.5), mass=203.5, tag="M / P"),
    # Overseer (P pack 148 x 48 x 45, Spark 16 Oct; the latch worker's one-standard cartridge not drawn yet): E 153 x 52 x 57 (box-arm
    # Problem 15 + rails), tab line at its middle (E, DEFAULT: CG on the thrust axis), XT60 at the lead end like the Scout's; pack 650 g (E,
    # 4S 6000 class) + cartridge 70 g (E)
    "overseer": dict(x=26.0, h=57.0, l=153.0, l_boss=3.0, tab=76.5, xt60_z=145.8, xt60_y=55.0, cg=(0.0, 35.0, 76.5), mass=720.0, tag="E"),
}
BOT_DZ = L.CART_BOT_DZ                    # 15.4: carried cartridge bottom above z0 (gripper datum, unchanged)
LATCH_BRANCH = "origin/part/battery-latch"
LATCH = dict(source="dimensions E (PR 44, cartridge v4)")


def latch_module():
    """nox/box/battery_latch.py (the latch child's; never edited here): on the path when it is on main, else staged read-only from its
    branch into .cache/box/ (gitignored). None when neither exists."""
    try:
        import battery_latch as BL
        LATCH["source"] = "nox/box/battery_latch.py"
        return BL
    except ImportError:
        pass
    import subprocess, importlib.util
    cache = os.path.join(REPO, ".cache", "box"); os.makedirs(cache, exist_ok=True); path = os.path.join(cache, "battery_latch.py")
    try:
        src = subprocess.run(["git", "-C", REPO, "show", f"{LATCH_BRANCH}:nox/box/battery_latch.py"], capture_output=True, text=True, check=True).stdout
        sha = subprocess.run(["git", "-C", REPO, "rev-parse", "--short", LATCH_BRANCH], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None
    open(path, "w").write(src)
    sp = importlib.util.spec_from_file_location("battery_latch", path); BL = importlib.util.module_from_spec(sp); sys.modules["battery_latch"] = BL
    sp.loader.exec_module(BL)
    LATCH["source"] = f"{LATCH_BRANCH} @ {sha} (battery_latch.latch_assembly, staged read-only)"
    return BL


def cart_from_latch(BL, pack):
    """the carried cartridge from the latch standard (cartridge coords: x across, y up from the flat bottom, z along, 0 = closed end)."""
    b = BL.build(pack); g = b["g"]
    M, c, _ = BL.swap_cg(g)
    return dict(x=g["X_RAIL"], x_tip=g["X_TIP"], h=g["Y_TOP"], l=g["L_C"], l_boss=0.0, tab=g["Z_T"], xt60_z=g["Z_X"], xt60_y=g["Y_XF_FACE"],
                cg=(0.0, round(float(c[1]), 2), round(float(c[2]), 2)), mass=round(float(M), 1), tag="C geometry / M + P mass (latch v1)")


_BL = latch_module()
if _BL is not None:
    for _p in ("scout", "overseer"):
        CART[_p] = cart_from_latch(_BL, _p)

# ================================================================== masses (g) and their CGs (arm frame / head frame)
MASS = dict(hub=210.0, motor=400.0, rail=50.0, carriage=60.0, head=120.0, stub=47.7, sun_post=30.4, ring=5.0, det_arm=2.3)
MASS_TAG = dict(hub="E (hub v5 214.2 M; v6 from its run)", motor="E (17HS19 48-long class, datasheet ~0.4 kg)", rail="M (rail v1 49.7 / 50.0)",
                carriage="E (3 wheels 8.4 M + 6 x 625ZZ ~30 + M5 stacks ~20)", head="E (head v5 56 M + servo 60 E)", stub="C", sun_post="C")

# ================================================================== stiffness and strength data (E unless stated)
E_PETG, G_PETG = 2000.0, 750.0            # MPa (E)
E_STEEL, G_STEEL = 200000.0, 77000.0
S_IN, S_ACROSS = 45.0, 25.0               # PETG in-layer / across-layer strength, MPa (E, as the topo engine and arm_v5_cad)
K_BRG_R, K_BRG_A = 2.0e4, 1.0e4           # 6010 in a printed seat, radial / axial, N/mm (E)
K_WHEEL = 2.0e3                           # V-wheel on the PETG ridge, per direction, N/mm (E, Hertz line contact at ~100 N)
K_TOOTH_T = 470.0                         # rack on pinion along the rack: NEMA17 holding stiffness ~30 N.m/rad at r 8 (E)
K_TOOTH_N = 2.0e3                         # separating direction, N/mm (E)
K_BELT = 200.0                            # axle loop (6 mm GT2, ~140 mm runs), tangential at the 20T, N/mm (E)
K_DETENT_ROT = 9.0e3                      # detent restoring stiffness 0.55 N.m over ~3.5 deg, N.mm/rad (C/E, detent_kin)
F_XT60, F_PRELOAD, F_LEADIN = 45.0, 12.0, 3.0      # dimensions I XT60 mate: insert 60 = 45 XT60 + 12 preload + 3 lead-ins (E)
F_PULL = 50.0                             # pull-out 50 (E): 45 XT60 + 5 latch wedge
F_STALL = 74.0                            # head motor stall at 2 A on the 16T (E)
T_AXLE_STALL = 0.59 * 4.0                 # N.m at the arm: 17HS19 0.59 N.m holding (E) x 4:1
DROP_H = 50.0                             # a cartridge falls 50 back onto head A's seat at the drone (E, DEFAULT misuse): F = sqrt(2 m g h k_seat)
F_ECC_PRE = 30.0                          # eccentric wheel preload across the rail (E, arm_v5_cad)
G_HANDLE = (3.0, 2.0)                     # box handling: 3 g vertical + 2 g sideways on every arm mass (E, DEFAULT; set down hard, carried)
F_HAND = 50.0                             # a hand on a rail tip, any direction (misuse, as hub v5's per-post pushes)
TARGET = dict(pack_x=0.5, tilt_deg=0.5)   # pack shift at the XT60 across the insert axis and tilt under the 60 N insert (DEFAULT: half the
                                          # receiver XT60 float carrier's +-1.0 / +-1.06 deg)
INDEX_TARGET = dict(swap=0.5, sustained=0.5, misuse=1.0, handling=1.0)
K_RECV_T, K_RECV_R = 2.0e3, 2.0e6         # the receiver's rails on a cartridge in them: across N/mm, tilt N.mm/rad (E, the drone on its plungers
                                          # in series); free along the insert
GAP_T = 0.3                               # the latch receiver's locating faces 0.3 off the cartridge (battery_latch RF, C)
GAP_R = 2 * GAP_T / 87.0                  # tilt before both ends touch: 0.6 over the ~87 engaged length at the XT60 mate (C): 0.0069 rad = 0.40 deg
LAG_MAX = S6.Z0_INSERT_MAX - (L.Z0_BAY + 5.0 + V.DRONE_PUSH)   # 2.0: the commanded drone insert's over-travel (arm_v6_layout)
M_HEAD_FALL = 180.0                       # head + carriage that fall to the park stop on a power cut at the drone insert (g, E)
BUMPER = dict(x=(-9.5, 9.5), u=(0.0, 5.0), t=5.0, contact_mm2=18.0 * 3.0, E_MPa=26.0)   # head v6: TPU 95A pad in a notch of the carriage plate's
                                          # inner edge (X +-10 x u 0..5, no axle there), glued; meets the rail's park stop (X +-9, Y 73..77) over Y 74..77 (C)
K_STOP = dict(petg=5.0e3, tpu=round(BUMPER["E_MPa"] * BUMPER["contact_mm2"] / BUMPER["t"], 0))   # N/mm: PETG on PETG (E) / the bumper (E 26 MPa)
KT_HEAD, R_PHASE = 0.21, 1.4             # 17HS19 head motor: torque constant N.m/A, phase resistance ohm (E, datasheet class)
STOP_SHEAR_N = 0.5 * S_ACROSS * 18.0 * 4.0                # the stop block on the rail top, across layers (C area, E strength): 900 N


# ================================================================== frame model
def rect(b, h):
    """section b along local y, h along local z"""
    return dict(A=b * h, Iy=b * h ** 3 / 12, Iz=h * b ** 3 / 12, J=b * h * (b * b + h * h) / 12 * 0.6, cy=b / 2, cz=h / 2, kind=f"rect {b:g} x {h:g}")


def tube(ro, ri):
    I = math.pi * (ro ** 4 - ri ** 4) / 4
    return dict(A=math.pi * (ro * ro - ri * ri), Iy=I, Iz=I, J=2 * I, cy=ro, cz=ro, kind=f"tube {2 * ro:g}/{2 * ri:g}")


def rail_section():
    """rail v1/v2 cross-section (C): body 18 x 19 + two ridges (mean width 1.75, Y 85..91 in v5 = 11..17 above the bottom)"""
    b, h = 2 * V.RAIL_BODY_X, V.RAIL_Y[1] - V.RAIL_Y[0]
    parts = [(b * h, 0.0, h / 2, b ** 3 * h / 12, b * h ** 3 / 12)]
    yr = V.RIDGE_Y - V.RAIL_Y[0]
    for sx in (-1, 1):
        w, hh = 1.75, 2 * V.RIDGE_HALF_H
        parts.append((w * hh, sx * (V.RAIL_BODY_X + w / 2), yr, w ** 3 * hh / 12, w * hh ** 3 / 12))
    A_ = sum(p[0] for p in parts); yc = sum(p[0] * p[2] for p in parts) / A_
    Ix = sum(p[4] + p[0] * (p[2] - yc) ** 2 for p in parts)            # about X (bending in the Y-r plane: the carriage couple)
    Iyy = sum(p[3] + p[0] * p[1] ** 2 for p in parts)                  # about Y (bending in the X-r plane: side loads)
    return dict(A=A_, Iy=Ix, Iz=Iyy, J=0.25 * b * h ** 3 * 0.6, cy=b / 2 + 2.5, cz=max(yc, h - yc), kind="rail 18 x 19 + ridges", yc=V.RAIL_Y[0] + yc)


SEC = dict(stub=tube(24.95, 20.0), hub_axis=tube(43.0, 40.0), hub_arm=rect(36.0, 30.0), pad=rect(32.0, 20.0), rail=rail_section(),
           m5=tube(2.5, 0.0), shaft=tube(2.5, 0.0), plate=rect(40.0, 8.0), web=rect(30.0, 6.0), head=rect(30.0, 30.0), rigid=rect(60.0, 60.0),
           motor=rect(42.3, 42.3), post=tube(21.75, 4.25))


class Frame:
    def __init__(self):
        self.P, self.idx, self.beams, self.springs, self.fixed, self.mass = [], {}, [], [], set(), {}

    def node(self, nm, p):
        if nm not in self.idx:
            self.idx[nm] = len(self.P); self.P.append(np.asarray(p, float))
        return nm

    def beam(self, nm, a, b, sec, E=E_PETG, G_=G_PETG, up=(0.0, 1.0, 0.0), piece=None, allow=S_IN, scale=1.0):
        self.beams.append(dict(nm=nm, a=a, b=b, sec=sec, E=E * scale, G=G_ * scale, up=np.asarray(up, float), piece=piece, allow=allow))

    def spring(self, nm, a, b, k6, piece=None):
        self.springs.append(dict(nm=nm, a=a, b=b, k=np.asarray(k6, float), piece=piece))

    def _T(self, e):
        pa, pb = self.P[self.idx[e["a"]]], self.P[self.idx[e["b"]]]
        L_ = np.linalg.norm(pb - pa); ex = (pb - pa) / L_
        up = e["up"] if abs(np.dot(e["up"], ex)) < 0.95 else np.array([1.0, 0, 0]) if abs(ex[0]) < 0.9 else np.array([0, 0, 1.0])
        ez = up - np.dot(up, ex) * ex; ez /= np.linalg.norm(ez); ey = np.cross(ez, ex)
        R = np.vstack([ex, ey, ez]); T = np.zeros((12, 12))
        for k in range(4): T[3 * k:3 * k + 3, 3 * k:3 * k + 3] = R
        return L_, T

    def _kl(self, e, L_):
        s = e["sec"]; E, G_ = e["E"], e["G"]; A_, Iy, Iz, J = s["A"], s["Iy"], s["Iz"], s["J"]
        k = np.zeros((12, 12))
        def put(i, j, v): k[i, j] += v
        a = E * A_ / L_; t = G_ * J / L_
        for i, j, v in ((0, 0, a), (0, 6, -a), (6, 0, -a), (6, 6, a), (3, 3, t), (3, 9, -t), (9, 3, -t), (9, 9, t)): put(i, j, v)
        for (I, d, r, sg) in ((Iz, 1, 5, 1.0), (Iy, 2, 4, -1.0)):           # bending in local xy (about z) and xz (about y)
            c1, c2, c3, c4 = 12 * E * I / L_ ** 3, 6 * E * I / L_ ** 2, 4 * E * I / L_, 2 * E * I / L_
            dd, rr = (d, d + 6), (r, r + 6)
            put(d, d, c1); put(d + 6, d + 6, c1); put(d, d + 6, -c1); put(d + 6, d, -c1)
            put(d, r, sg * c2); put(r, d, sg * c2); put(d, r + 6, sg * c2); put(r + 6, d, sg * c2)
            put(d + 6, r, -sg * c2); put(r, d + 6, -sg * c2); put(d + 6, r + 6, -sg * c2); put(r + 6, d + 6, -sg * c2)
            put(r, r, c3); put(r + 6, r + 6, c3); put(r, r + 6, c4); put(r + 6, r, c4)
        return k

    def solve(self, loads):
        """loads: {node: (fx, fy, fz, mx, my, mz)}. Returns displacements (n, 6), beam end forces (local), spring forces (global)."""
        n = len(self.P); K = np.zeros((6 * n, 6 * n)); F = np.zeros(6 * n)
        cache = []
        for e in self.beams:
            L_, T = self._T(e); kl = self._kl(e, L_); kg = T.T @ kl @ T
            ia, ib = self.idx[e["a"]], self.idx[e["b"]]; dofs = np.r_[6 * ia:6 * ia + 6, 6 * ib:6 * ib + 6]
            K[np.ix_(dofs, dofs)] += kg; cache.append((L_, T, kl, dofs))
        for s in self.springs:
            ia, ib = self.idx[s["a"]], self.idx[s["b"]]
            for d in range(6):
                if s["k"][d] == 0: continue
                i, j = 6 * ia + d, 6 * ib + d
                K[i, i] += s["k"][d]; K[j, j] += s["k"][d]; K[i, j] -= s["k"][d]; K[j, i] -= s["k"][d]
        for nm, f in loads.items():
            i = self.idx[nm]; F[6 * i:6 * i + 6] += np.asarray(f, float)
        free = np.ones(6 * n, bool)
        for nm in self.fixed:
            i = self.idx[nm]; free[6 * i:6 * i + 6] = False
        u = np.zeros(6 * n)
        Kf = K[np.ix_(free, free)] + np.eye(free.sum()) * 1e-9
        u[free] = np.linalg.solve(Kf, F[free])
        R = K @ u - F                                                       # reactions at the fixed nodes
        ef = []
        for e, (L_, T, kl, dofs) in zip(self.beams, cache):
            fl = kl @ (T @ u[dofs]); ef.append(fl)
        sf = []
        for s in self.springs:
            ia, ib = self.idx[s["a"]], self.idx[s["b"]]
            sf.append(s["k"] * (u[6 * ib:6 * ib + 6] - u[6 * ia:6 * ia + 6]))     # force on a from b (positive = b moved away along +)
        reac = {nm: R[6 * self.idx[nm]:6 * self.idx[nm] + 6] for nm in self.fixed}
        return u.reshape(n, 6), ef, sf, reac


def stress(e, fl):
    """max |sigma| over the two ends: |N|/A + |My|/Wy + |Mz|/Wz (MPa, hand section), N in N, M in N.mm"""
    s = e["sec"]; out = 0.0
    for k in (0, 6):
        N, My, Mz = fl[k], fl[k + 4], fl[k + 5]
        out = max(out, abs(N) / s["A"] + abs(My) * s["cz"] / s["Iy"] + abs(Mz) * s["cy"] / s["Iz"])
    return out


# ------------------------------------------------------------------ the assembled arm
OPTS = dict(rail_sec=None, edge_r=None, cart={}, tie=None, guided=None)   # guided: the head whose cartridge sits in a receiver's rails    # build options: the alternatives (rail section, plate-edge bearing, cartridge XT60
                                                              # place, a carbon tie under each rail); rail_sec None = rail_section() of the stack in use
TIE = dict(r_rail=S6.TIE["rail"][1], y_rail=S6.TIE["rail"][0], y_hub=S6.TIE["hub"][0], r_hub=S6.TIE["hub"][1], n=2, x=S6.TIE["x"])   # arm_v6_layout.TIE
YC_RAIL = SEC["rail"]["yc"]               # rail section centroid Y
Y_WHEEL = 0.5 * (V.WHEEL_Y[0] + V.WHEEL_Y[1])
Y_PLATE = 0.5 * (V.PLATE_Y[0] + V.PLATE_Y[1])
Y_RACK = 0.5 * (V.RACK_TEETH_Y[0] + V.RACK_TEETH_Y[1])


def head_pt(h, x, y, u, z0):
    """a point of head h's own geometry (A convention; B mirrored in X) in the arm frame at stroke z0"""
    if h == "A": return (x, y, u + z0)
    return (-(u + z0), y, -x)             # B: arm X = -(u + z0), arm Z = X_B = -x


def rail_pt(h, x, y, r):
    return (x, y, r) if h == "A" else (-r, y, -x)


def build(zA, zB, Y_C, cart="scout", carry=("A",), axle="belt", phi=90.0, opts=None):
    """the frame at the stroke states (zA, zB), carried cartridges on the heads in `carry`. axle: 'belt' (driver on) or 'detent'
    (driver off: the insert strokes, box-arm Problem 17). Returns (Frame, info)."""
    o = dict(OPTS); o.update(opts or {})
    if o["tie"] == "TIE": o["tie"] = TIE                  # carbon ties: rejected alternative (guided: -3 % receiver moment for 4 rods + 8 sockets)
    F = Frame(); c = dict(CART[cart]); c.update(o.get("cart", {}))
    rsec = o["rail_sec"] or rail_section(); YC_RAIL = rsec["yc"]
    Y_WHEEL = 0.5 * (V.WHEEL_Y[0] + V.WHEEL_Y[1]); Y_PLATE = 0.5 * (V.PLATE_Y[0] + V.PLATE_Y[1]); Y_RACK = 0.5 * (V.RACK_TEETH_Y[0] + V.RACK_TEETH_Y[1])
    posts = bool(V.POSTS_A)
    F.node("ground", (0, 0, 0)); F.fixed.add("ground")
    # stub on the post face, bearings at their centres, hub axis
    for y in (16.0, 32.0, 40.0):
        F.node(f"S{y:g}", (0, y, 0))
    F.beam("stub 0-16", "ground", "S16", SEC["stub"], piece="stub", allow=S_ACROSS)
    F.beam("stub 16-32", "S16", "S32", SEC["stub"], piece="stub", allow=S_ACROSS)
    F.beam("stub 32-40", "S32", "S40", SEC["stub"], piece="stub", allow=S_ACROSS)
    for y in (16.0, 32.0, 52.0):
        F.node(f"H{y:g}", (0, y, 0))
    F.spring("bearing 1", "S16", "H16", (K_BRG_R, K_BRG_A, K_BRG_R, 0, 0, 0), piece="6010")
    F.spring("bearing 2", "S32", "H32", (K_BRG_R, K_BRG_A, K_BRG_R, 0, 0, 0), piece="6010")
    F.beam("hub sleeve 16-32", "H16", "H32", SEC["hub_axis"], piece="hub")
    F.beam("hub web 32-52", "H32", "H52", SEC["hub_axis"], piece="hub")
    # sun post (fixed) carries the 80T: the loop's reaction goes to ground through it
    F.node("P80", (0, 59.0, 0)); F.beam("sun post", "S40", "P80", SEC["post"], piece="sun post", allow=S_ACROSS)
    # motors on their seats (the body hangs Y 2..50 from the face, CG at Y 26)
    M6 = dict(A=V.MOTORS_V5["A"], B=V.MOTORS_V5["B"], axle=dict(xc=53.93, zc=-43.67, rot=-39.0))
    for nm, M in M6.items():
        F.node(f"seat {nm}", (M["xc"], 52.0, M["zc"])); F.node(f"motor {nm}", (M["xc"], 26.0, M["zc"]))
        F.beam(f"hub arm to seat {nm}", "H52", f"seat {nm}", SEC["hub_arm"], piece="hub")
        F.beam(f"motor body {nm}", f"seat {nm}", f"motor {nm}", SEC["motor"], E=E_STEEL * 0.05, G_=G_STEEL * 0.05, piece="motor")
        F.mass[f"motor {nm}"] = MASS["motor"]
    # axle drive: the 20T on the axle motor's shaft, the loop to the fixed 80T (tangential spring) or the detent (rotational)
    ax = M6["axle"]; r_ax = math.hypot(ax["xc"], ax["zc"]); t_ax = np.array([-ax["zc"], 0, ax["xc"]]) / r_ax      # +phi tangent at the 20T
    F.node("ax20T", (ax["xc"], 59.0, ax["zc"])); F.beam("axle shaft", "seat axle", "ax20T", SEC["shaft"], E=E_STEEL, G_=G_STEEL, piece="shaft", allow=400.0)
    F.node("ax20T g", (ax["xc"], 59.0, ax["zc"])); F.beam("80T to loop (rigid)", "P80", "ax20T g", SEC["rigid"], E=E_STEEL, G_=G_STEEL, piece="fixed")
    if axle != "none":                                                        # 'none': the stall case, the loop force is applied as a load
        kb = K_BELT if axle == "belt" else K_DETENT_ROT / r_ax ** 2
        kv = np.abs(t_ax) * kb
        F.spring("axle loop" if axle == "belt" else "detent (driver off)", "ax20T", "ax20T g", (kv[0], 0, kv[2], 0, 0, 0), piece="loop")
    # detent tower on the hub (pivot / anchor loads)
    Q = DT.CHOSEN_V6["Q"]; F.node("tower", (Q[0], 70.0, Q[1])); F.beam("tower", "H52", "tower", SEC["hub_arm"], piece="hub")
    F.mass["H32"] = MASS["hub"] + MASS["ring"] * 0 + MASS["det_arm"]
    info = dict(zA=zA, zB=zB, Y_C=Y_C, cart=cart, carry=carry, axle=axle, pads={}, wheels={}, rack={}, head_root={}, xt60={}, pack_seat={})
    for h, z0 in (("A", zA), ("B", zB)):
        # rail pads on the hub plate (Y 52) and the rail centroid line
        rb = V.RAIL_BOLTS["r_" + h]
        rs = sorted(set([V.RAIL_R[0] + 2.0, *rb, V.RAIL_R[1]] + [z0 + u for _, u in V.WHEELS] + [V.PIN_Z] + ([o["edge_r"]] if o["edge_r"] else [])
                        + ([o["tie"]["r_rail"]] if o["tie"] else [])))
        for r in rs:
            F.node(f"rail{h} {r:.1f}", rail_pt(h, 0, YC_RAIL, r))
        for a, b in zip(rs[:-1], rs[1:]):
            F.beam(f"rail {h} {a:.0f}-{b:.0f}", f"rail{h} {a:.1f}", f"rail{h} {b:.1f}", rsec, piece=f"rail {h}", allow=S_IN,
                   up=(0, 1, 0))
        for r in rb:
            F.node(f"pad{h} {r:g}", rail_pt(h, 0, 52.0, r))
            F.beam(f"hub arm to pad {h} {r:g}", "H52", f"pad{h} {r:g}", SEC["hub_arm"], piece="hub")
            F.beam(f"pad joint {h} {r:g}", f"pad{h} {r:g}", f"rail{h} {r:.1f}", rect(32.0, 12.0) if posts else SEC["pad"], piece=f"pad joint {h}",
                   up=(1.0, 0, 0) if h == "A" else (0, 0, 1.0))                  # v5: the 32 x 12 post (C) + foot; v6: the foot on its pad
            info["pads"][f"{h} r{r:g}"] = f"pad joint {h} {r:g}"
        F.beam(f"hub plate pads {h}", f"pad{h} {rb[0]:g}", f"pad{h} {rb[1]:g}", SEC["hub_arm"], piece="hub")
        if o["edge_r"]:                                                       # the rail bears on the plate's outer pad edge (contact in Y only)
            F.node(f"edge{h}", rail_pt(h, 0, L.Y_PLATE1, o["edge_r"])); F.beam(f"hub to pad edge {h}", f"pad{h} {rb[1]:g}", f"edge{h}", SEC["hub_arm"], piece="hub")
            F.node(f"edge{h} r", rail_pt(h, 0, L.Y_PLATE1, o["edge_r"]))
            F.beam(f"rail at the edge {h}", f"rail{h} {o['edge_r']:.1f}", f"edge{h} r", SEC["rigid"], piece=f"rail {h}")
            F.spring(f"pad edge contact {h}", f"edge{h}", f"edge{h} r", (0, 5.0e3, 0, 0, 0, 0), piece="pad joint")
            info.setdefault("edges", {})[h] = f"pad edge contact {h}"
        F.mass[f"rail{h} {rb[1]:.1f}"] = MASS["rail"]
        if o["tie"]:                                                          # carbon tie: rail bottom near the tip -> hub lower rim
            t = o["tie"]; rn = min(rs, key=lambda q: abs(q - t["r_rail"]))
            F.node(f"tie rail {h}", rail_pt(h, 0, t.get("y_rail", V.RAIL_Y[0]), rn)); F.beam(f"rail tie lug {h}", f"rail{h} {rn:.1f}", f"tie rail {h}", SEC["rigid"], piece=f"rail {h}")
            F.node(f"tie hub {h}", rail_pt(h, 0, t["y_hub"], t["r_hub"]))
            F.beam(f"hub to tie {h}", "H16", f"tie hub {h}", SEC["hub_arm"], piece="hub"); F.beam(f"hub to tie 2 {h}", f"pad{h} {rb[1]:g}", f"tie hub {h}", SEC["hub_arm"], piece="hub")
            ts = dict(tube(1.5, 0.0)); ts.update(A=ts["A"] * t["n"], Iy=ts["Iy"] * t["n"], Iz=ts["Iz"] * t["n"], J=ts["J"] * t["n"])
            F.beam(f"carbon tie {h}", f"tie hub {h}", f"tie rail {h}", ts, E=120000.0, G_=5000.0, piece="carbon tie", allow=1000.0)
            info.setdefault("ties", {})[h] = f"carbon tie {h}"
        # carriage: wheels on the ridges, axles to the plate, the plate star, the rack web, the head body, the cartridge
        cx = head_pt(h, 9.0, Y_PLATE, 18.0, z0); F.node(f"carriage {h}", cx); F.mass[f"carriage {h}"] = MASS["carriage"]
        for k, (x, u) in enumerate(V.WHEELS):
            sx = 1.0 if x > 0 else -1.0
            r = z0 + u
            pc = head_pt(h, sx * V.RIDGE_FLAT_X, Y_WHEEL, u, z0)
            F.node(f"ridge {h}{k}", pc); F.node(f"wheel {h}{k}", pc)
            F.beam(f"rail to ridge {h}{k}", f"rail{h} {r:.1f}", f"ridge {h}{k}", SEC["rigid"], piece=f"rail {h}")
            F.node(f"axle {h}{k}", head_pt(h, x, Y_WHEEL, u, z0)); F.node(f"axle top {h}{k}", head_pt(h, x, Y_PLATE, u, z0))
            F.beam(f"wheel {h}{k}", f"axle {h}{k}", f"wheel {h}{k}", SEC["rigid"], piece="wheel")
            F.beam(f"M5 axle {h}{k}", f"axle {h}{k}", f"axle top {h}{k}", SEC["m5"], E=E_STEEL, G_=G_STEEL, piece="M5 axle", allow=640.0)
            F.beam(f"plate {h}{k}", f"axle top {h}{k}", f"carriage {h}", SEC["plate"], piece=f"carriage {h}")
            kx = np.abs(np.array(head_pt(h, 1, 0, 0, 0)) - np.array(head_pt(h, 0, 0, 0, 0))) * K_WHEEL      # contact across the rail (head X)
            F.spring(f"wheel contact {h}{k}", f"ridge {h}{k}", f"wheel {h}{k}", (kx[0], K_WHEEL, kx[2], 0, 0, 0), piece="V-wheel")
            info["wheels"][f"{h}{k}"] = (f"wheel contact {h}{k}", f"M5 axle {h}{k}")
        # rack web from the plate down to the teeth at the pinion line; the pinion on the motor shaft
        F.node(f"web {h}", head_pt(h, 43.5, Y_PLATE, V.PIN_Z - z0, z0)); F.node(f"rack {h}", head_pt(h, V.RACK_PITCH_X, Y_RACK, V.PIN_Z - z0, z0))
        F.beam(f"plate to web {h}", f"carriage {h}", f"web {h}", SEC["plate"], piece=f"carriage {h}")
        F.beam(f"rack web {h}", f"web {h}", f"rack {h}", SEC["web"], piece=f"carriage {h}", up=head_dir(h, (1, 0, 0)))
        F.node(f"pinion {h}", head_pt(h, V.RACK_PITCH_X, Y_RACK, V.PIN_Z - z0, z0)); F.node(f"shaft {h}", head_pt(h, V.PIN_X, Y_RACK, V.PIN_Z - z0, z0))
        F.beam(f"motor shaft {h}", f"seat {h}", f"shaft {h}", SEC["shaft"], E=E_STEEL, G_=G_STEEL, piece="shaft", allow=400.0)
        F.beam(f"pinion body {h}", f"shaft {h}", f"pinion {h}", SEC["rigid"], piece="pinion")
        ut = np.abs(np.array(head_pt(h, 0, 0, 1, 0)) - np.array(head_pt(h, 0, 0, 0, 0)))        # along the rail
        ux = np.abs(np.array(head_pt(h, 1, 0, 0, 0)) - np.array(head_pt(h, 0, 0, 0, 0)))
        kk = ut * K_TOOTH_T + ux * K_TOOTH_N
        F.spring(f"rack on pinion {h}", f"pinion {h}", f"rack {h}", (kk[0], 0, kk[2], 0, 0, 0), piece="pinion")
        info["rack"][h] = f"rack on pinion {h}"
        # head body: carriage -> the cartridge seat (the head's generated body + gripper, E section), the cartridge rigid
        F.node(f"cseat {h}", head_pt(h, 0.0, Y_C, BOT_DZ, z0))
        F.beam(f"head body {h}", f"carriage {h}", f"cseat {h}", SEC["head"], piece=f"head {h}", allow=S_ACROSS)
        info["head_root"][h] = f"head body {h}"
        F.mass[f"cseat {h}"] = MASS["head"]
        if h in carry:
            F.node(f"xt60 {h}", head_pt(h, 0.0, Y_C - c["tab"] + c["xt60_z"], BOT_DZ + c["xt60_y"], z0))
            F.node(f"pack top {h}", head_pt(h, 0.0, Y_C, BOT_DZ + c["h"], z0))
            F.node(f"pack cg {h}", head_pt(h, 0.0, Y_C - c["tab"] + c["cg"][2], BOT_DZ + c["cg"][1], z0))
            for nn in (f"xt60 {h}", f"pack top {h}", f"pack cg {h}"):
                F.beam(f"cartridge {h} {nn}", f"cseat {h}", nn, SEC["rigid"], piece="cartridge")
            F.mass[f"pack cg {h}"] = c["mass"]
            info["xt60"][h] = f"xt60 {h}"
            if o["guided"] == h:                                              # the receiver's rails: across and tilt held, free along the insert
                F.node(f"receiver {h}", F.P[F.idx[f"xt60 {h}"]]); F.fixed.add(f"receiver {h}")
                ax_ = np.abs(head_dir(h, (1, 0, 0))) + np.array([0, 1.0, 0])
                F.spring(f"receiver rails {h}", f"xt60 {h}", f"receiver {h}", (*(ax_ * K_RECV_T), K_RECV_R, K_RECV_R, K_RECV_R), piece="receiver")
                info["receiver"] = f"receiver rails {h}"
    return F, info


def gravity_loads(F, phi, gx=0.0, gz=-1.0, gy=0.0):
    """mass x g at each mass node. (gx, gy, gz) in world g units (W: Z up at the drone); the arm frame at phi is W turned by rotY(90 - phi)."""
    a = math.radians(90.0 - phi); c, s = math.cos(a), math.sin(a)
    gw = np.array([gx, gy, gz])
    g_arm = np.array([c * gw[0] - s * gw[2], gw[1], s * gw[0] + c * gw[2]])     # inverse of rotY(90 - phi) applied to the world vector
    return {nm: np.r_[m * G * g_arm, 0, 0, 0] for nm, m in F.mass.items()}


def add(loads, nm, f):
    loads[nm] = np.asarray(loads.get(nm, np.zeros(6)), float) + np.r_[np.asarray(f, float), np.zeros(6 - len(f))]


def head_dir(h, v):
    """head-frame direction (X, Y, u) -> arm frame"""
    return np.array(head_pt(h, *v, 0.0)) - np.array(head_pt(h, 0, 0, 0, 0.0))


# ------------------------------------------------------------------ system cases
def cases(Y_C=L.Y_C, cart="scout"):
    """[(name, class, (zA, zB), carry, axle, phi, loads builder, note)]: every case acts where it acts on the real machine (C positions, E forces)."""
    z = dict(park=L.Z0_A_HOME, wait=L.Z0_B_WAIT, bay=L.Z0_BAY + 1.5, drone=L.Z0_BAY + 5.0, insert=L.Z0_BAY + 5.0 + V.DRONE_PUSH)
    def swap(F_main, F_seat, sign, lead=True, h="A"):
        def f(Fr, info):
            ld = {}
            add(ld, info["xt60"][h], sign * F_main * head_dir(h, (0, 0, 1)))
            add(ld, f"pack top {h}", sign * F_seat * head_dir(h, (0, 0, 1)))
            if lead: add(ld, f"pack top {h}", F_LEADIN * head_dir(h, (1, 0, 0)))
            return ld
        f.h = h
        return f
    out = [
        ("insert at the drone 60 N", "swap", (z["insert"], z["park"]), ("A", "B"), "detent", 90.0, swap(F_XT60, F_PRELOAD, -1.0),
         "head A at the drone insert z0 95.6, B parked carrying; 45 N at the XT60 (cartridge lead end) + 12 N seat preload + 3 N lead-in, axle driver off"),
        ("insert at a bay 60 N", "swap", (z["bay"], z["park"]), ("A",), "detent", -90.0, swap(F_XT60, F_PRELOAD, -1.0),
         "head A into the -90 bay z0 86.1 (arm horizontal, gravity along the rail), axle driver off"),
        ("pull at the drone 50 N", "swap", (z["drone"], z["wait"]), ("A",), "belt", 90.0, swap(F_XT60, F_PULL - F_XT60, 1.0, lead=False),
         "head B's pick (here on A's geometry) at z0 89.6: 45 N XT60 + 5 N latch wedge outward"),
        ("pull at a bay 50 N", "swap", (z["bay"], z["park"]), ("A",), "belt", -90.0, swap(F_XT60, F_PULL - F_XT60, 1.0, lead=False),
         "head A picks the charged pack from the -90 bay"),
        ("B insert at a bay 60 N", "swap", (z["park"], z["bay"]), ("A", "B"), "detent", -180.0, swap(F_XT60, F_PRELOAD, -1.0, h="B"),
         "head B returns the dead pack into the -90 bay (phi -180), A parked carrying"),
        ("B pull at the drone 50 N", "swap", (z["park"], z["drone"]), ("A", "B"), "belt", 0.0, swap(F_XT60, F_PULL - F_XT60, 1.0, lead=False, h="B"),
         "head B picks the dead pack at the drone (phi 0), A parked carrying the charged one"),
        ("head stall into a jammed pack 74 N", "misuse", (z["insert"], z["park"]), ("A", "B"), "detent", 90.0, swap(F_STALL, 0.0, -1.0),
         "the 16T at 2 A stall drives the cartridge's XT60 into a jam at the drone insert"),
        ("axle stall, head in a bay", "misuse", (z["bay"], z["park"]), ("A",), "none", -90.0, None,
         "the axle motor stalls (0.59 N.m x 4 = 2.36 N.m) with head A held in the -90 bay: the bay holds the cartridge tangentially"),
        ("dropped pack", "misuse", (z["insert"], z["park"]), ("A",), "belt", 90.0, None,
         "a cartridge falls 50 back onto head A's seat at the drone: F = sqrt(2 m g h k) with the assembled seat stiffness (E)"),
        ("box handling 3 g down + 2 g side", "handling", (z["park"], z["park"]), ("A", "B"), "belt", 0.0, None,
         "the box set down hard / carried, both heads parked and carrying, arm at phi 0 (A horizontal)"),
        ("power cut carrying: head A falls to the park stop", "misuse", (z["park"], z["park"]), ("A",), "detent", 90.0, None,
         "power cut with head A up carrying a cartridge at the drone insert: head + carriage + cartridge outweigh the cogging and fall 63.6 onto the "
         "TPU bumper; the bumper force decelerates them and the free rotor (through the 16T); static equivalent at the stop"),
        ("hand on rail A tip 50 N", "misuse", (z["park"], z["park"]), (), "belt", 90.0, None, "50 N in -X (tangential), worst of +-X / +-Y"),
        ("gravity, A horizontal carrying", "sustained", (z["park"], z["park"]), ("A", "B"), "belt", 0.0, None, "self weight, the worst stop for the axle"),
    ]
    return out


def solve_gap(Fr, info, ld):
    """Fr.solve with the receiver as gap + spring per DOF (GAP_T across, GAP_R in tilt): the arm alone carries the push until a gap
    closes; an active set over the six DOFs. Returns (u, ef, sf, reac, receiver force 6-vector on the cartridge or None)."""
    if "receiver" not in info:
        u, ef, sf, reac = Fr.solve(ld); return u, ef, sf, reac, None
    k = [s_["nm"] for s_ in Fr.springs].index(info["receiver"]); sp = Fr.springs[k]; k0 = sp["k"].copy()
    i = Fr.idx[sp["a"]]; gap = np.array([GAP_T] * 3 + [GAP_R] * 3)
    act = np.zeros(6, bool); sgn = np.zeros(6)
    for _ in range(12):
        sp["k"] = np.where(act, k0, 0.0)
        l2 = {n: np.asarray(v, float).copy() for n, v in ld.items()}
        add(l2, sp["a"], np.where(act, k0 * gap * sgn, 0.0))
        u, ef, sf, reac = Fr.solve(l2)
        d = u[i]
        new = (np.abs(d) > gap) & (k0 > 0)
        if np.array_equal(new, act) and np.array_equal(np.sign(d) * new, sgn * act): break
        act, sgn = new, np.where(new, np.sign(d), 0.0)
    f = -np.where(act, k0 * (d - gap * sgn), 0.0)
    sp["k"] = k0
    return u, ef, sf, reac, f


def run_frame(Y_C=L.Y_C, cart="scout", verbose=True, guided=True):
    res = []
    rows = []
    for c in cases(Y_C, cart):                                              # every receiver-guided case also runs free (no receiver): the arm
        rows.append(c + ("gap",))                                           # pieces take the worse of the two (concept review, DEFAULT)
        if getattr(c[6], "h", None): rows.append((c[0] + " (free, no receiver)", "misuse") + c[2:] + ("free",))   # the bound for a missing or
                                                                            # misaligned receiver: misuse class (DEFAULT)
    for nm, cls, (zA, zB), carry, axle, phi, fn, note, mode in rows:
        gh = getattr(fn, "h", None)
        Fr, info = build(zA, zB, Y_C, cart, carry, axle, phi, dict(guided=gh) if guided and gh and mode == "gap" else None)
        ld = gravity_loads(Fr, phi)
        if fn is not None:
            for k, v in fn(Fr, info).items(): add(ld, k, v)
        if nm.startswith("axle stall"):
            # the bay holds the cartridge: a stiff tangential spring from A's pack top to ground; the 20T pulls with the stall torque
            Fr.node("bay hold", Fr.P[Fr.idx["pack top A"]]); Fr.fixed.add("bay hold")
            tA = head_dir("A", (1, 0, 0)); Fr.spring("bay holds the cartridge", "pack top A", "bay hold", (abs(tA[0]) * 5e3, 0, abs(tA[2]) * 5e3, 0, 0, 0))
            # the loop's tight run pulls the 20T toward the 80T along the common tangent (25.5 off the axis); the motor torque is internal
            ax = np.array(Fr.P[Fr.idx["ax20T"]]); r = math.hypot(ax[0], ax[2]); ec = np.array([ax[0], 0, ax[2]]) / r
            a = math.asin((25.47 - 6.37) / r); ca, sa = math.cos(a), math.sin(a)
            d = -np.array([ca * ec[0] - sa * ec[2], 0, sa * ec[0] + ca * ec[2]])
            fb = 1e3 * T_AXLE_STALL / 25.47 * d
            m_pt = ax[2] * fb[0] - ax[0] * fb[2]
            m_need = math.copysign(1e3 * T_AXLE_STALL, m_pt if abs(m_pt) > 1e-9 else 1.0)
            add(ld, "ax20T", np.r_[fb, 0, m_need - m_pt, 0])
        if nm.startswith("dropped"):
            u1, _, _, _ = Fr.solve({"cseat A": np.r_[head_dir("A", (0, 0, -1)), 0, 0, 0]})
            k_seat = 1.0 / max(float(np.dot(u1[Fr.idx["cseat A"]][:3], head_dir("A", (0, 0, -1)))), 1e-6)     # N/mm along the drop
            F_drop = math.sqrt(2 * CART[cart]["mass"] * 1e-3 * 9.81 * DROP_H * 1e-3 * k_seat * 1e3)
            # the head motor (powered) holds the rack only to its stall force: past it the rotor skips and the head gives way, so the
            # pinion and everything behind it see at most F_STALL; the rest of the energy goes into the slip (E)
            F_hold = F_STALL + (MASS["head"] + MASS["carriage"]) * G
            slip_mm = CART[cart]["mass"] * 1e-3 * 9.81 * DROP_H / max(F_hold - CART[cart]["mass"] * G, 1e-6) if F_drop > F_hold else 0.0
            F_drop_raw = F_drop; F_drop = min(F_drop, F_hold)
            add(ld, "cseat A", F_drop * head_dir("A", (0, 0, -1)))
        if nm.startswith("box handling"):
            ld = gravity_loads(Fr, phi, gx=G_HANDLE[1], gz=-G_HANDLE[0])
        if nm.startswith("hand on rail"):
            add(ld, f"railA {V.RAIL_R[1]:.1f}", (-F_HAND, 0, 0))
        if nm.startswith("power cut"):
            pc = power_cut_loads(Fr, info, ld, cart)
        u, ef, sf, reac, frec = solve_gap(Fr, info, ld)
        # separating force of the rack (tan 20 x the tangential tooth force): one more pass with it applied
        sep = {}
        for h in info["rack"]:
            k = [s["nm"] for s in Fr.springs].index(info["rack"][h])
            ft = float(np.dot(sf[k][:3], head_dir(h, (0, 0, 1))))
            fs = abs(ft) * math.tan(math.radians(20.0)); sep[h] = (ft, fs)
            add(ld, f"rack {h}", -fs * head_dir(h, (1, 0, 0))); add(ld, f"pinion {h}", fs * head_dir(h, (1, 0, 0)))
        u, ef, sf, reac, frec = solve_gap(Fr, info, ld)
        extra = {}
        if nm.startswith("dropped"): extra = dict(F_drop_rigid_N=round(F_drop_raw, 1), F_drop_N=round(F_drop, 1), k_seat_N_mm=round(k_seat, 1), slip_mm=round(slip_mm, 2))
        if frec is not None:
            extra["receiver"] = dict(F_across_N=round(float(np.linalg.norm(frec[:3])), 1), M_Nm=round(float(np.linalg.norm(frec[3:])) / 1e3, 2),
                                     note="what the receiver's rails hold after their 0.3 gaps close (latch / drone / bay requirement)")
        if nm.startswith("power cut"): extra["power_cut"] = pc
        extra["hub"] = hub_interfaces(Fr, u, ef, sf)
        extra["hub_mass"] = {nm: dict(p=Fr.P[Fr.idx[nm]].round(2).tolist(), g=m) for nm, m in Fr.mass.items() if nm in ("H32",)}
        res.append(dict(case=nm, cls=cls, note=note, mode=mode, **extra, phi=phi, zA=zA, zB=zB, carry=list(carry), axle=axle, **summarise(Fr, info, u, ef, sf, reac, sep)))
        if verbose:
            r = res[-1]
            if "receiver" in r: print(f"{'':38s} lag along the insert {r['pack']['du_mm']:6.2f} (max {LAG_MAX:.1f}), receiver {r['receiver']}")
            print(f"{nm:38s} pack dX {r['pack']['dx_mm']:6.3f} tilt {r['pack']['tilt_deg']:6.3f} | worst {r['worst']['piece']} {r['worst']['index']:.2f} ({r['worst']['elem']})")
    return res


HUB_IFACE = ("seat A", "seat B", "seat axle", "tower")    # + pads and tie sockets per head (build adds them)


def hub_interfaces(Fr, u, ef, sf):
    """loads (F N, M N.mm, arm frame, about the node) that the neighbours put on the hub at each interface node; the 6010 springs are
    the supports. The hub = every element with piece 'hub' and the nodes they touch."""
    hub_nodes = set()
    for e in Fr.beams:
        if e["piece"] == "hub": hub_nodes |= {e["a"], e["b"]}
    out = {}
    for e, fl in zip(Fr.beams, ef):
        if e["piece"] == "hub": continue
        L_, T = Fr._T(e); fg = T.T @ fl
        for end, nm in ((0, e["a"]), (6, e["b"])):
            if nm in hub_nodes:
                out.setdefault(nm, np.zeros(6)); out[nm] -= fg[end:end + 6]       # the element pushes the node with -(its end force)
    for s_, f in zip(Fr.springs, sf):
        if s_["piece"] == "6010": continue
        for nm, sg in ((s_["a"], 1.0), (s_["b"], -1.0)):
            if nm in hub_nodes:
                out.setdefault(nm, np.zeros(6)); out[nm] += sg * f
    return {nm: dict(p=Fr.P[Fr.idx[nm]].round(2).tolist(), F=v[:3].round(2).tolist(), M=v[3:].round(1).tolist()) for nm, v in out.items()
            if np.abs(v).max() > 1e-6}


def summarise(Fr, info, u, ef, sf, reac, sep):
    out = dict(joints={}, pieces={}, pack={})
    snames = [s["nm"] for s in Fr.springs]; bnames = [b["nm"] for b in Fr.beams]
    def bf(nm):
        e = Fr.beams[bnames.index(nm)]; f = ef[bnames.index(nm)]
        return e, f
    # stub base (post face): the 4 M5s at (X +-10, Z +-30)
    R = reac["ground"]
    out["joints"]["stub to post (4 x M5)"] = dict(F=np.round(R[:3], 1).tolist(), M_Nm=np.round(R[3:] / 1e3, 2).tolist(),
                                                bolt_N=round(abs(R[1]) / 4 + abs(R[3]) / (2 * 60.0) + abs(R[5]) / (2 * 20.0), 0))
    for b in ("bearing 1", "bearing 2"):
        f = sf[snames.index(b)]; out["joints"][b] = dict(radial_N=round(math.hypot(f[0], f[2]), 1), axial_N=round(abs(f[1]), 1))
    for k, nm in info["pads"].items():
        e, f = bf(nm)
        out["joints"][f"rail pad {k}"] = dict(bolt_N=round(f[0], 1), shear_N=round(math.hypot(f[1], f[2]), 1), M_Nm=round(math.hypot(f[4], f[5]) / 1e3, 2))
    for k, (cn, an) in info["wheels"].items():
        f = sf[snames.index(cn)]; h = k[0]
        out["joints"][f"wheel {k}"] = dict(across_N=round(float(np.dot(f[:3], head_dir(h, (1, 0, 0)))), 1), axial_N=round(f[1], 1))
    for h, nm in info["rack"].items():
        ft, fs = sep.get(h, (0.0, 0.0))
        out["joints"][f"rack on pinion {h}"] = dict(tangential_N=round(ft, 1), separating_N=round(fs, 1))
    for h, nm in info["head_root"].items():
        e, f = bf(nm)
        out["joints"][f"carriage to head {h}"] = dict(F_N=round(float(np.linalg.norm(f[:3])), 1), M_Nm=round(float(np.linalg.norm(f[3:6])) / 1e3, 2))
    for nm in ("seat A", "seat B", "seat axle"):
        k = f"motor body {nm.split()[1]}"
        e, f = bf(k); out["joints"][f"motor seat {nm.split()[1]}"] = dict(F_N=round(float(np.linalg.norm(f[:3])), 1), M_Nm=round(float(np.linalg.norm(f[3:6])) / 1e3, 3))
    for nm in ("axle loop", "detent (driver off)"):
        if nm in snames:
            f = sf[snames.index(nm)]; out["joints"][nm] = dict(F_N=round(float(np.linalg.norm(f[:3])), 1), torque_Nm=round(float(np.linalg.norm(f[:3])) * 69.4 / 1e3, 3))
    # pack deflection at the XT60 (head A when carried)
    ih = Fr.idx["H52"]; uh = u[ih]; ph = Fr.P[ih]
    def rel(i):                                                           # relative to the hub's rigid motion (the arm's float on the detent is not deflection)
        return np.r_[u[i][:3] - uh[:3] - np.cross(uh[3:], Fr.P[i] - ph), u[i][3:] - uh[3:]]
    out["hub_turn_deg"] = round(math.degrees(float(uh[4])), 3)
    if "A" in info["xt60"]:
        i = Fr.idx[info["xt60"]["A"]]; d = rel(i)
        tx = head_dir("A", (1, 0, 0)); tu = head_dir("A", (0, 0, 1))
        rot = d[3:]
        out["pack"] = dict(dx_mm=round(float(np.dot(d[:3], tx)), 3), du_mm=round(float(np.dot(d[:3], tu)), 3), dy_mm=round(float(d[1]), 3),
                           tilt_deg=round(math.degrees(float(np.linalg.norm(rot))), 3))
    else:
        i = Fr.idx[f"rail{'A'} {V.RAIL_R[1]:.1f}"]; d = rel(i)
        out["pack"] = dict(dx_mm=round(float(d[0]), 3), du_mm=round(float(d[2]), 3), dy_mm=round(float(d[1]), 3), tilt_deg=round(math.degrees(float(np.linalg.norm(d[3:]))), 3),
                           at="rail A tip")
    # piece indices (hand sections, E)
    for e, f in zip(Fr.beams, ef):
        if e["piece"] in (None, "fixed", "wheel", "pinion", "cartridge", "motor"): continue
        sg = stress(e, f); idx = sg / e["allow"]
        p = out["pieces"].setdefault(e["piece"], dict(index=0.0, elem=None, sigma=0.0))
        if idx > p["index"]: p.update(index=round(idx, 3), elem=e["nm"], sigma=round(sg, 2))
    # joint-specific indices: V-wheel flank Hertz, 625ZZ axial, pinion Lewis, insert pull-out
    p0 = 1.67 * 55.0
    worst_w = 0.0; worst_brg = 0.0
    for k in info["wheels"]:
        j = out["joints"][f"wheel {k}"]; Fy = abs(j["axial_N"]); Fx = abs(j["across_N"])
        Fn = (Fx + F_ECC_PRE + Fy) / math.sqrt(2.0)                            # 90 deg V: the worse flank takes (Fx + Fy) / sqrt 2, + the eccentric preload
        band = (V.WHEEL_OD / 2 - (22.0 - V.RIDGE_FLAT_X)) * math.sqrt(2); Rw = (V.WHEEL_OD / 2 + 10.5) / 2 * math.sqrt(2)
        Es = E_PETG / (2 * (1 - 0.4 ** 2)); a = math.sqrt(4 * max(Fn, 1e-6) * Rw / (math.pi * band * Es)); pm = 2 * Fn / (math.pi * a * band)
        worst_w = max(worst_w, pm / p0); worst_brg = max(worst_brg, Fy / 670.0)
    out["pieces"]["V-wheel (flank Hertz)"] = dict(index=round(worst_w, 3), elem="worst wheel", sigma=None)
    out["pieces"]["625ZZ (axial / C0 670 N)"] = dict(index=round(worst_brg, 3), elem="worst wheel", sigma=None)
    for h in info["rack"]:
        ft = abs(out["joints"][f"rack on pinion {h}"]["tangential_N"])
        lew = ft / ((V.PIN_TEETH_Y[1] - V.PIN_TEETH_Y[0]) * V.PIN_M * 0.295)
        q = out["pieces"].setdefault("pinion 16T (Lewis, in-layer)", dict(index=0.0, elem=None, sigma=0.0))
        if lew / S_IN > q["index"]: q.update(index=round(lew / S_IN, 3), elem=f"pinion {h}", sigma=round(lew, 1))
    ins = max(abs(out["joints"][k]["bolt_N"]) for k in out["joints"] if k.startswith("rail pad"))
    out["pieces"]["rail pad insert (pull-out 1000 N E)"] = dict(index=round(ins / 1000.0, 3), elem="worst pad", sigma=None)
    w = max(out["pieces"].items(), key=lambda kv: kv[1]["index"])
    out["worst"] = dict(piece=w[0], index=w[1]["index"], elem=w[1]["elem"])
    return out


def pitch(stack="v6", Y_C=L.Y_C, opts=None, F=(F_XT60, F_PRELOAD)):
    """cartridge pitch (deg) and XT60 lag along the push (mm) under the insert at the drone, head A at the drone insert (E)."""
    S6.use(stack)
    try:
        Fr, info = build(max(V.Z0S), L.Z0_A_HOME, Y_C, "scout", ("A",), "belt", 90.0, opts)
        ld = {}; add(ld, "xt60 A", (0, 0, -F[0])); add(ld, "pack top A", (0, 0, -F[1]))
        u, ef, sf, reac = Fr.solve(ld); i = Fr.idx["xt60 A"]
        free = (round(math.degrees(float(np.linalg.norm(u[i][3:]))), 2), round(float(-u[i][2]), 2))
        og = dict(opts or {}); og["guided"] = "A"
        Fg, ig = build(max(V.Z0S), L.Z0_A_HOME, Y_C, "scout", ("A",), "belt", 90.0, og)
        ug, efg, sfg, _, frec = solve_gap(Fg, ig, ld)                       # the receiver with its 0.3 gaps
        return free + (round(float(-ug[Fg.idx["xt60 A"]][2]), 2), round(float(np.linalg.norm(frec[3:])) / 1e3, 2))
    finally:
        S6.use("v6")


def alternatives():
    """the concept check's alternatives with numbers (E loads, C geometry): the stack (posts / plate), the carbon tie, the XT60 place,
    at the demo Y_C 175 and at the lowest Y_C each stack's carriage hardware allows."""
    rows = []
    yc = {r["dy"]: r["yc_min"] for r in S6.stack_search(CART["scout"]["tab"])}
    for stack, ycm in (("v5", yc[0.0]), ("v6", yc[-20.0])):
        for nm, o in (("as drawn", dict(tie=None)), ("+ carbon ties", dict(tie=TIE)), ("+ ties + XT60 on the tab line", dict(tie=TIE, cart=dict(xt60_z=CART["scout"]["tab"])))):
            if stack == "v5" and nm != "as drawn": continue
            for Y in (L.Y_C, ycm):
                t, d, dg, mr = pitch(stack, Y, o); ex = arm_extent(stack, Y)
                rows.append(dict(stack=stack, variant=nm, Y_C=Y, pitch_free_deg=t, lag_free_mm=d, lag_guided_mm=dg, receiver_M_Nm=mr,
                                 length_y=ex["length_y"], bbox_L=ex["bbox_L"], largest_mm=ex["largest_mm"]))
    return rows


def power_cut_loads(Fr, info, ld, cart="scout", k=None):
    """the power-cut-carrying case on the frame (A up, at the park stop): bumper F = sqrt(2 E k) for head + carriage + cartridge falling
    from the drone insert, deceleration a = F / (falling + rotor); inertial loads m a toward the axle at the masses' nodes, the free rotor's
    share as a tooth-force pair (rack -, pinion +, the rack spring's tangential holding stiffness off: unpowered), the stop as a spring."""
    k = k or K_STOP["tpu"]
    m_f = (MASS["head"] + MASS["carriage"] + CART[cart]["mass"]) * 1e-3; m_r = 82e-7 / (V.PIN_R * 1e-3) ** 2
    h = (L.Z0_BAY + 5.0 + V.DRONE_PUSH - V.RAIL_STOP["z0_home"]) * 1e-3
    F = math.sqrt(2 * m_f * 9.81 * h * k * 1e3); a = F / (m_f + m_r)
    u_ = head_dir("A", (0, 0, 1))
    for node, g in (("cseat A", MASS["head"]), ("carriage A", MASS["carriage"]), ("pack cg A", CART[cart]["mass"])):
        add(ld, node, -g * 1e-3 * a * u_)
    Ft = m_r * a
    add(ld, "rack A", -Ft * u_); add(ld, "pinion A", Ft * u_)
    ks = [s_["nm"] for s_ in Fr.springs].index(info["rack"]["A"]); Fr.springs[ks]["k"] = np.r_[np.abs(head_dir("A", (1, 0, 0))) * K_TOOTH_N, 0, 0, 0]
    Fr.node("park stop A", Fr.P[Fr.idx["carriage A"]]); Fr.fixed.add("park stop A")
    Fr.spring("bumper on the park stop A", "carriage A", "park stop A", (*(np.abs(u_) * k), 0, 0, 0), piece="bumper")
    lew = Ft / ((V.PIN_TEETH_Y[1] - V.PIN_TEETH_Y[0]) * V.PIN_M * 0.295) / S_IN
    return dict(falling_g=round(m_f * 1e3, 0), F_bumper_N=round(F, 0), decel_g=round(a / 9.81, 1), F_tooth_N=round(Ft, 0), pinion_index=round(lew, 2),
                stop_index=round(F / STOP_SHEAR_N, 2), cart_inertia_N=round(CART[cart]["mass"] * 1e-3 * a, 0))


def power_cut():
    """misuse: power cut with head A at the drone insert (arm vertical), then a bump. Unpowered, the stepper's cogging (0.022 N.m E,
    detent.disturbance) holds the head at r 8 against its weight; a 3 g bump beats it and head + carriage fall 64.6 to the park stop,
    driving the rotor (17HS19 rotor 82 g.cm2, E: 0.128 kg at r 8). F at the stop = sqrt(2 E k); the rotor's share goes through the 16T."""
    m_h = M_HEAD_FALL * 1e-3; m_r = 82e-7 / (V.PIN_R * 1e-3) ** 2
    hold_N = DT.disturbance()["cogging"] / 4.0 / (V.PIN_R * 1e-3)
    w = m_h * 9.81; h = (L.Z0_BAY + 5.0 + V.DRONE_PUSH - V.RAIL_STOP["z0_home"]) * 1e-3
    E_ = w * h
    lew = lambda F: F / ((V.PIN_TEETH_Y[1] - V.PIN_TEETH_Y[0]) * V.PIN_M * 0.295) / S_IN
    out = dict(hold_N=round(hold_N, 2), weight_N=round(w, 2), holds_at_1g=bool(hold_N > w), holds_at_3g=bool(hold_N > 3 * w), fall_mm=round(h * 1e3, 1),
               energy_J=round(E_, 3), rotor_kg=round(m_r, 3))
    for nm, k in K_STOP.items():
        F = math.sqrt(2 * E_ * k * 1e3); Ft = F * m_r / (m_h + m_r)
        out[nm] = dict(k_N_mm=k, F_stop_N=round(F, 0), stop_index=round(F / STOP_SHEAR_N, 2), F_tooth_N=round(Ft, 0), pinion_index=round(lew(Ft), 2))
    # carrying (head + carriage + cartridge 391 g): falls at 1 g with no bump (3.8 N > the 2.75 N cogging); the frame case "power cut carrying"
    m_c = m_h + CART["scout"]["mass"] * 1e-3; w_c = m_c * 9.81
    out["carrying"] = dict(falling_g=round(m_c * 1e3, 0), weight_N=round(w_c, 2), holds_at_1g=bool(hold_N > w_c),
                           F_bumper_N=round(math.sqrt(2 * w_c * h * K_STOP["tpu"] * 1e3), 0), note="frame case: carriage to head 24 N.m, 625ZZ 1.23 without a brake")
    # the brake (box electronics): both coils of each head motor shorted by a normally closed relay contact while the 24 V is off; back-EMF
    # torque Kt^2 w / R (17HS19: Kt ~0.21 N.m/A, R 1.4 ohm per phase, E) against the weight at r 8: the head creeps down instead of falling
    w_br = w_c * V.PIN_R * 1e-3 / (KT_HEAD ** 2 / R_PHASE); v = w_br * V.PIN_R
    E_br = 0.5 * (m_c + m_r) * (v * 1e-3) ** 2
    out["brake"] = dict(creep_mm_s=round(v, 1), impact_J=round(E_br, 6), F_bumper_N=round(math.sqrt(2 * E_br * K_STOP["tpu"] * 1e3), 1))
    return out


def overseer_radius():
    """the ring radius R that would let the arm carry the Overseer cartridge (latch v1) with the Scout's park rule (corners 2.5 inside
    the seated faces) at the current park z0, and the sector it needs at that R (C)."""
    c = CART["overseer"]; z = V.RAIL_STOP["z0_home"] + 1.0
    top = z + BOT_DZ + c["h"]; xw = c["x"]                           # gripped: the bolts are in (tips 4 inside), the rails set the width
    R_park = math.hypot(xw, top) + 2.5
    sec = math.degrees(2 * math.asin((xw + 0.3) / R_park))
    return dict(park_z0=z, top_r=round(top, 1), R_needed=round(R_park, 1), R_now=L.R, sector_deg_at_R=round(sec, 1), pitch_deg=L.PITCH,
                note="every bay, the stand lane and the funnel scale with R; the Scout swap would pay for it")


# ================================================================== stack and bounding volume
def arm_extent(stack="v6", Y_C=L.Y_C, cart="scout"):
    """assembled arm (both heads, carried cartridges, motors, hub, stub) axis-aligned extents in the arm frame over the stroke, from the
    layout numbers (C): X / Z = the turning envelope (2 x the reach at the drone insert), Y = stub face 0 .. cartridge far end."""
    S6.use(stack)
    c = CART[cart]
    zmax = max(V.Z0S)
    reach = max(math.hypot(V.WHEEL_X, zmax + V.WHEELS[1][1] + V.WHEEL_OD / 2), math.hypot(V.PLATE_X[1], zmax + V.PLATE_U[1]),
                math.hypot(c["x"], zmax + BOT_DZ + c["h"]), math.hypot(V.RIDGE_X, V.RAIL_R[1]))
    y1 = Y_C - c["tab"] + c["l"] + c["l_boss"]
    y0 = 0.0                                                     # stub spider face on the post (motor bodies from Y 2)
    S6.use("v6")
    turn_d = 2 * reach
    return dict(stack=stack, Y_C=Y_C, reach_r=round(reach, 1), y=(y0, round(y1, 1)), length_y=round(y1 - y0, 1),
                swept_cyl_L=round(math.pi * reach ** 2 * (y1 - y0) / 1e6, 2), bbox_L=round(turn_d ** 2 * (y1 - y0) / 1e6, 2),
                largest_mm=round(max(turn_d, y1 - y0), 1))


def stack_report():
    rows = S6.stack_search(CART["scout"]["tab"])
    v5 = next(r for r in rows if r["dy"] == 0.0); v6 = next(r for r in rows if r["dy"] == -20.0)
    out = dict(search=rows, chosen="v6 (dy -20: rails on the motor plate)")
    out["demo"] = dict(v5=arm_extent("v5", L.Y_C), v6=arm_extent("v6", L.Y_C))
    out["compact_modules"] = dict(v5=arm_extent("v5", v5["yc_min"]), v6=arm_extent("v6", v6["yc_min"]))
    return out


# ================================================================== Overseer pack
def overseer():
    """can the v6 stroke and axial stack carry the Overseer pack (dimensions O: 148 x 48 x 45 P; cartridge E 153 x 52 x 57)?"""
    c = CART["overseer"]; s = CART["scout"]
    z = dict(park=L.Z0_A_HOME, bay=L.Z0_BAY + 1.5, drone=L.Z0_BAY + 5.0, insert=max(V.Z0S), home=V.RAIL_STOP["z0_home"])
    top_park = z["park"] + BOT_DZ + c["h"]; corner_park = math.hypot(c["x"], top_park)
    seat_face = L.R - 2.5                                         # carried corners stay 2.5 inside the seated packs' faces (park rule)
    z_park_need = math.sqrt(seat_face ** 2 - c["x"] ** 2) - BOT_DZ - c["h"]
    rows = dict(cartridge=c)
    rows["stroke"] = dict(bay_z0=z["bay"], drone_z0=z["drone"], insert_z0=z["insert"], note="bottom-referenced like the Scout (one catch interface): the same bay / drone strokes",
                          ok=True)
    rows["park"] = dict(top_r=round(top_park, 1), corner_r=round(corner_park, 1), limit_r=seat_face, z0_needed=round(z_park_need, 1), z0_min_reachable=z["home"],
                        short_mm=round(z["home"] - z_park_need, 1), ok=bool(z_park_need >= z["home"]),
                        note="carried at park its top corner crosses the seated packs' faces (R 100): the arm cannot turn with it")
    # axial: closed end vs the carriage hardware (v6 top 85.5 inside the cartridge footprint) and the bay module length
    S6.use("v6")
    hw_top = max(V.AXLE_KEEP["y"][1] - 3.0, V.ECC["flange_y"][1] + 5.0)
    end0 = L.Y_C - c["tab"]; end1 = end0 + c["l"] + c["l_boss"]
    rows["axial"] = dict(Y_C=L.Y_C, cart_y=(end0, end1), hw_top_v6=hw_top, clear_v6=round(end0 - hw_top, 1),
                         hw_top_v5=105.5, clear_v5=round(end0 - 105.5, 1), module_y=(94.0, 255.0), fits_module=bool(end1 <= 255.0 and end0 >= 94.0))
    # sector: the cartridge's width at r 100 against the 30 deg pitch
    chord = 2 * L.R * math.sin(math.radians(L.PITCH / 2))
    rows["sector"] = dict(width=2 * c["x"], chord_30deg_at_R=round(chord, 1), ok=bool(2 * c["x"] + 2 * 0.3 <= chord), note="needs two sectors (60 deg) or a narrower cartridge")
    # detent with the Overseer pack: gravity torque carried at park, arm horizontal
    r_cg = z["park"] + BOT_DZ + c["cg"][1]
    t_ov = c["mass"] * G * r_cg / 1e3; t_sc = s["mass"] * G * (z["park"] + BOT_DZ + s["cg"][1]) / 1e3
    dist = DT.disturbance_v5()
    hold = 2 * 0.390
    rows["detent"] = dict(pack_torque_Nm=round(t_ov, 3), scout_torque_Nm=round(t_sc, 3), disturbance_scout_Nm=dist["total"],
                          disturbance_overseer_Nm=round(dist["total"] - t_sc + t_ov, 3), holding_2_springs_Nm=hold,
                          margin=round(hold / (dist["total"] - t_sc + t_ov), 2))
    # loads: the 60 N insert is the same XT60; the mass case grows
    rows["mass_ratio_vs_scout"] = round(c["mass"] / s["mass"], 2)
    return rows


# ================================================================== the assembly file and its system_check per pose
ASM = os.path.join(HERE, "box_swap_arm.assembly.json")
ASM_DIR = os.path.join(OUT, "asm")
POSES = dict(park=0.0, bay_pick=3.0, drone_insert=11.5, bay_return=18.5, turn_carrying=9.5)     # arm_kinematics v6 cycle times (s)
BOUGHT_G = dict(motor=400.0, brg=110.0, stub_bolts=8.0, p20_axle=8.0, p80=30.0, belt_axle=8.0, wheels=8.4, axles=20.0, servo=60.0, detent_roller=1.0,
                detent_springs=1.0, detent_tower=0.5, shaft=0.0)


def _bought_g(nm):
    for k, g in BOUGHT_G.items():
        if nm.startswith(k): return g
    return 5.0


def pose_at(s, part):
    """assembly 'moves' function: the part's world pose at swap-cycle time s (s, arm_kinematics v6 keys)."""
    import arm_kinematics as K
    if not K.PARTS_CACHE: K.KEYS = K._keys_v6(); K.build_parts("v6")
    T, carts, _ = K.transforms(float(s))
    nm = part["name"]
    if nm in T: return T[nm]
    if nm in carts: return carts[nm][0]
    if nm.startswith("pack"): return K.trans(0, 0, L.R + 5.0)               # the dead pack before the drone lands: in the drone at the seat
    return np.eye(4)


def write_assembly(t=0.0):
    """nox/box/box_swap_arm.assembly.json (this session owns it): every arm part in its pose at cycle time t (park), meshes exported
    once to runs/arm/v6/asm/ (W, decimated), the bay modules as one mesh posed 9 times, the carried cartridges (latch v1) as bought."""
    import arm_kinematics as K, assembly as AS, trimesh
    K.KEYS = K._keys_v6(); P = K.build_parts("v6")
    os.makedirs(ASM_DIR, exist_ok=True)
    T, carts, st = K.transforms(t)
    parts = []
    for nm, p in P.items():
        if nm.startswith("bay") and nm != f"bay{K.demo_idx()}":
            k = int(nm[3:]); a = L.SCOUT_BAYS[k]
            parts.append(dict(name=nm, role="structure", mesh="models/box_baymod_v4i_W.obj", pose=K.rotY(90.0 - a).tolist(), material="PETG", tag="M",
                              owner="box bay modules", version="v4i", note="context (fixed)")); continue
        path = os.path.join(ASM_DIR, f"{nm}.stl"); m = p.mesh
        if len(m.faces) > 40000:
            import fast_simplification as fsimp, trimesh
            v_, f_ = fsimp.simplify(np.asarray(m.vertices, np.float32), np.asarray(m.faces), 1.0 - 40000.0 / len(m.faces)); m = trimesh.Trimesh(v_, f_)
        m.export(path)
        role = "bought" if not p.printed else "structure"
        d = dict(name=nm, role=role, mesh=os.path.relpath(path, HERE), pose=np.asarray(T.get(nm, np.eye(4))).tolist(), material="TPU" if nm.startswith("strip") else "PETG",
                 tag="M" if p.printed else "C", owner="swap arm system" if not nm.startswith(("bay", "strip")) else "box bay modules",
                 source="nox/box/arm_kinematics.py build_parts_v6", moves="nox/box/swap_system.py:pose_at", note=p.name)
        if not p.printed: d["mass_g"] = _bought_g(nm)
        parts.append(d)
    for cn in ("pack charged", "pack dead"):                               # both always (the dead one sits in the drone until it lands)
        path = os.path.join(ASM_DIR, f"{cn.replace(' ', '_')}.stl"); K.tm(K.CART_LOCAL(4.0)).export(path)   # gripped: bolts in (the sweep has every squeeze)
        own = carts[cn][2] if cn in carts else "drone (before landing)"
        parts.append(dict(name=cn, role="bought", swap=True, mesh=os.path.relpath(path, HERE), pose=np.asarray(pose_at(t, dict(name=cn))).tolist(),
                          mass_g=CART["scout"]["mass"], tag="E", owner="battery latch (latch_assembly)", source=LATCH["source"],
                          moves="nox/box/swap_system.py:pose_at", note=own))
    carts = {"pack charged": None, "pack dead": None}
    contact = [[a, b, 1e9] for a, b in K.ALLOWED if a in P and b in P]
    for cn in carts:
        for h in "AB": contact += [[cn, f"fingA_{h}", 1e9], [cn, f"fingB_{h}", 1e9]]
        contact += [[cn, f"bay{K.demo_idx()}", 0.5]]
    a = AS.new("box swap arm v6", "box", parts, note="the swap arm at cycle time t (park at t 0, phi -90); meshes runs/arm/v6/asm (W). Context: the 9 bay "
               "modules (v4i, the demo bay as the latch receiver). Not in: stand, funnel, drone (their owners' files). Pose any state with a.at(**{name: t}) "
               "(swap_system.pose_at). No cage (a ground station's arm): cage first and probes do not apply. The CG line includes the fixed parts (report only, "
               "cg_tol 1000): the arm's balance is the turning parts' unbalance about the axle, swap_system.run_checks.",
               cg_ref=[0.0, 0.0, 0.0], thrust_axis=[0.0, 1.0, 0.0], cg_tol_mm=1000.0, fit_tol_mm3=0.5, contact_pairs=contact, probes=False)
    a.save(ASM)
    return a


DET_HOLD = 2 * 0.390                       # N.m, two springs (detent CHOSEN_V5 / V6, E)
DET_OTHER = 0.06 + 0.088                   # seals + the unpowered axle motor's cogging through the 4:1 (detent.disturbance, E)


def arm_only(a, t):
    """compact and balance of the turning arm alone (no bays, stub, post, bearings, 80T, ring): AABB, largest extent, hull volume, and the
    unbalance about the axle (g.m) of the turning parts + carried cartridges, vs the 10.1 g.m the detent was sized for (E)."""
    import arm_kinematics as K, trimesh
    fixed = {nm for nm, p in K.PARTS_CACHE.items() if p.group == "fixed"} | {"stub_bolts"}
    _, carts, _ = K.transforms(t)
    held = {cn for cn, (_, _, own) in carts.items() if own.startswith("head")}
    ms, mx, mz, M = [], 0.0, 0.0, 0.0
    for p in a.parts:
        nm = p["name"]
        if nm in fixed or (nm.startswith("pack") and nm not in held): continue
        m = a.mesh(nm); ms.append(m)
        g = p.get("mass_g", m.volume / 1000 * 1.27 if m.is_volume else 0.0)
        c = m.center_mass if m.is_volume else m.centroid
        mx += g * c[0]; mz += g * c[2]; M += g
    allm = trimesh.util.concatenate(ms); lo, hi = allm.bounds; ex = hi - lo
    return dict(aabb_L=round(float(np.prod(ex)) / 1e6, 2), extents_mm=np.round(ex, 1).tolist(), largest_mm=round(float(ex.max()), 1),
                hull_L=round(float(allm.convex_hull.volume) / 1e6, 2), turning_g=round(M, 0), unbalance_gm=round(math.hypot(mx, mz) / 1e3, 2),
                cg_off_axis_mm=round(math.hypot(mx, mz) / M, 2), detent_sized_gm=10.1, gravity_Nm=round(9.81e-6 * abs(mx), 3),
                detent_margin_2_springs=round(DET_HOLD / (9.81e-6 * abs(mx) + DET_OTHER), 2))


def access_paths(t, heads=("A", "B")):
    """the key lines that must be free at cycle time t (rule 2): each rail's two M5 socket heads (Ø6 tool from the counterbore floor,
    100 along +Y) and each motor's 4 M3 screws on the plate top (Ø6 from Y 55); world frame from arm_kinematics.transforms (C)."""
    import arm_kinematics as K
    T, _, _ = K.transforms(t); Ta = T["hub"]
    w = lambda p: (Ta @ np.r_[p, 1.0])[:3].round(3).tolist()
    out = []
    for h in heads:
        for r in V.RAIL_BOLTS["r_" + h]:
            p = (0.0, V.RAIL_BOLTS["cbore_y"] + 0.5, r) if h == "A" else (-r, V.RAIL_BOLTS["cbore_y"] + 0.5, 0.0)
            out.append(dict(name=f"M5 key, rail {h} r {r:g}", part=f"rail{h}", **{"from": w(p)}, dir=[0, 1, 0], length_mm=100.0,
                            tool=dict(type="cylinder", d=6.0), ignore=["hub"]))
    import hub_v6 as HV
    for nm, M in HV.MOTORS6.items():
        if nm in ("A", "B") and nm not in heads: continue
        for sx in (-1, 1):
            for sz in (-1, 1):
                if (nm, sx, sz) in HV.SEAT_SKIP: continue
                hx, hz = A_.rot_pt(M, sx * L.NEMA_HOLES / 2, sz * L.NEMA_HOLES / 2)
                out.append(dict(name=f"M3 key, motor {nm} ({sx:+d}, {sz:+d})", part="hub", **{"from": w((hx, L.Y_PLATE1 + 1.0, hz))}, dir=[0, 1, 0],
                                length_mm=100.0, tool=dict(type="cylinder", d=6.0), ignore=[f"motor_{nm}"]))
    return out


ACCESS_AT = dict(bay_pick=("A",), bay_return=("B",))   # head A is out at the bay pick (z0 84.6), head B at the bay return: their keys are free there


def run_checks():
    """system_check on the assembly file at every pose (fit, compact, mass / CG; the CG offset from the axle is the arm's unbalance)."""
    import assembly as AS
    a = write_assembly(0.0)
    out = {}
    for pn, t in POSES.items():
        b = a.at(**{nm: t for nm in a.names if a.part(nm).get("moves")}) if t else a
        if pn in ACCESS_AT: b.data["access"] = access_paths(t, ACCESS_AT[pn])
        res = b.check(n_dirs=200)
        if pn in ACCESS_AT:
            print(" ", pn, "access", {p["name"]: (p.get("pass_"), p.get("blocked_by")) for p in res["access"]["paths"]} if res["access"].get("paths") else res["access"])
        open(os.path.join(OUT, f"system_check_{pn}.txt"), "w").write(__import__("system_check").report(res))
        keep = {k: res[k] for k in ("name", "pass", "summary", "compact")}
        keep["fit_fail"] = [p for p in res["fit"]["pairs"] if not p["pass_"]]
        keep["mass_g"] = res["mass_cg"]["mass_g"]; keep["cg"] = res["mass_cg"]["cg"]
        keep["arm_only"] = arm_only(b, t)
        json.dump(keep, open(os.path.join(OUT, f"system_check_{pn}.json"), "w"), indent=1, default=float)
        out[pn] = keep
        print(pn, res["summary"], "PASS" if res["pass"] else "FAIL", keep["arm_only"])
    return out


def main(what="all"):
    res = {}
    path = os.path.join(OUT, "system.json")
    if os.path.exists(path): res = json.load(open(path))
    if what in ("stack", "all"):
        res["stack"] = stack_report()
        for k in ("demo", "compact_modules"):
            for st, r in res["stack"][k].items(): print(k, st, r)
    if what in ("alt", "all"):
        res["alternatives"] = alternatives()
        for r in res["alternatives"]: print(r)
    if what in ("frame", "all"):
        res["frame"] = run_frame()
        res["frame_overseer"] = run_frame(cart="overseer", verbose=False)
    if what in ("overseer", "all"):
        res["overseer"] = overseer(); res["overseer"]["radius"] = overseer_radius(); print(json.dumps(res["overseer"], indent=1, default=str))
    if what in ("frame", "all"):
        res["power_cut"] = power_cut(); print("power cut", res["power_cut"])
        res["latch_source"] = LATCH["source"]; res["cart"] = CART
    if what in ("check", "all"):
        res["system_check"] = run_checks()
    res["constants"] = dict(mass=MASS, mass_tag=MASS_TAG, targets=TARGET, index_targets=INDEX_TARGET, K=dict(brg_r=K_BRG_R, brg_a=K_BRG_A, wheel=K_WHEEL,
                            tooth_t=K_TOOTH_T, tooth_n=K_TOOTH_N, belt=K_BELT, detent_rot=K_DETENT_ROT),
                            sections={k: v["kind"] for k, v in SEC.items()}, forces=dict(xt60=F_XT60, preload=F_PRELOAD, leadin=F_LEADIN, pull=F_PULL,
                            stall=F_STALL, axle_stall_Nm=T_AXLE_STALL, drop_h=DROP_H, handle_g=G_HANDLE, hand=F_HAND))
    json.dump(res, open(path, "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print("->", path)
    return res


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "all")
