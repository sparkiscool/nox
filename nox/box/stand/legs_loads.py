"""Box legs v1: load cases for one front leg (the +X leg; the -X leg is its mirror), 17 Oct 2026.

Rigid-body statics of the free-standing box (nox/box-stand.md): the bolted bay modules + legs = the ring body; the stub
spider (axle, hub, motors, arm, PSU, boards) = the spider body, tied to the ring by 4 ties (r 146 on the 45 deg
diagonals, along Y, E). 4 feet: back +-140 / Y -64 (spider), front +-160 / Y 225 (legs), floor Z -160 (F).
Foot reactions: non-negative least squares on the 3 support equations (vertical force, 2 tipping moments) = rigid box on
equal pads; a foot that would pull lifts off; what the feet cannot balance (tipping, sliding) is box inertia (E).
Horizontal reactions: shared in proportion to the vertical ones (Coulomb-like), capped at mu 0.8 (DEFAULT).
Tie forces: the spider body's balance shared by the 4 ties at least norm (rigid spider, E).
Leg load = floor reaction on its pad + the spider's force on its tie boss; the leg is held by its 8 screws.
Masses: stand_model.py (mostly E).   python legs_loads.py -> legs_loads.json
"""
import json, itertools
import numpy as np
from scipy.optimize import nnls
import stand_model as SM

G = 9.81
FEET = np.array([[140.0, -64.0], [-140.0, -64.0], [160.0, 225.0], [-160.0, 225.0]])   # F; index 2 = this leg
ZF = SM.ZF
TIE_Y = 85.5                                                # tie boss face (DEFAULT, 8.5 short of the module near face)
TIES = np.array([[sx * 146 / np.sqrt(2), TIE_Y, sz * 146 / np.sqrt(2)] for sz in (-1, 1) for sx in (1, -1)])  # 0 = this leg's
MU = 0.8                                                    # floor friction cap (DEFAULT)
COLLAR_Y, SEAT_Z = 138.0, 159.2                             # C / E (box-funnel.md v6)
FACE_MID = 122.0, SEAT_Z + 38.0                             # 45 deg face centroid: 84 + 38 out, 38 up (C)
RIM = SEAT_Z + 76.0                                         # mouth rim Z (C)
SPIDER = {"stub spider", "stub axle+flange", "2x 6010-2RS", "slip ring", "hub v3", "NEMA17 A", "NEMA17 B", "NEMA17 axle",
          "80T, belts, idlers", "PSU LRS-350-24", "SKR, chargers"}


def states():
    """every stand_model load state split into ring-borne and spider-borne mass (g) and first moment (g mm)."""
    ms = np.zeros(1); ss = np.zeros((1, 3)); mr0 = 0.0; sr0 = np.zeros(3)
    for name, g, p, _, _ in SM.FIXED:
        p = np.array(SM.ring_cg() if p is None else p, float)
        if name in SPIDER: ms[0] += g; ss[0] += g * p
        elif name == "wiring": ms[0] += g / 2; ss[0] += g / 2 * p; mr0 += g / 2; sr0 += g / 2 * p
        else: mr0 += g; sr0 += g * p
    occ = np.array([o for o in itertools.product((0, 1, 2), repeat=7) if o.count(2) <= 3 and o.count(1) <= 4 and o.count(0) >= 1])
    bay = np.array([[SM.R_BAY * np.cos(np.radians(a)), SM.Y_BAY, SM.R_BAY * np.sin(np.radians(a))] for a in SM.BAY_ANG])
    pm = np.vectorize(SM.PACK.get)(occ).astype(float); bm = pm.sum(1); bs = pm @ bay
    out = []
    for d in SM.DRONE:
        dm, dp = SM.DRONE[d]; ds = np.zeros(3) if dp is None else dm * np.array(dp)
        for phi in np.arange(0, 360, 15):
            uA = np.array([np.cos(np.radians(phi)), 0, np.sin(np.radians(phi))])
            uB = np.array([np.cos(np.radians(phi + 90)), 0, np.sin(np.radians(phi + 90))])
            for carry in ("none", "A_park", "A_ext", "B_park", "B_ext"):
                for pk in ((0,) if carry == "none" else (1, 2)):
                    rA = SM.R_HEAD_EXT if carry == "A_ext" else SM.R_HEAD_PARK
                    rB = SM.R_HEAD_EXT if carry == "B_ext" else SM.R_HEAD_PARK
                    am = 2 * SM.BAR + SM.HEAD_A + SM.HEAD_B
                    as_ = (SM.BAR * 78 * uA + SM.BAR * 78 * uB + SM.HEAD_A * rA * uA + SM.HEAD_B * rB * uB
                           + np.array([0, 84 * 2 * SM.BAR + 125 * (SM.HEAD_A + SM.HEAD_B), 0]))
                    if pk:
                        head = uA if carry[0] == "A" else uB
                        rp = SM.R_PACK_EXT if carry.endswith("ext") else SM.R_PACK_PARK
                        as_ = as_ + SM.PACK[pk] * (rp * head + np.array([0, 126, 0])); am += SM.PACK[pk]
                    ok = ((occ == 2).sum(1) + (pk == 2) <= 3) & ((occ == 1).sum(1) + (pk == 1) <= 4)
                    n = ok.sum()
                    out.append((np.full(n, mr0 + dm) + bm[ok], sr0 + ds + bs[ok], np.full(n, ms[0] + am), np.tile(ss[0] + as_, (n, 1))))
    mr, sr, msp, ssp = [np.concatenate(x) for x in zip(*out)]
    return mr, sr, msp, ssp


def feet_vertical(Fz, M, feet=FEET, active=None):
    """vertical foot reactions (N, up) for a net downward load Fz (N) with moments M = (Mx, My) about the origin
    (N mm, from the applied loads incl. horizontal ones at height). NNLS: feet push only."""
    act = np.ones(len(feet), bool) if active is None else np.asarray(active, bool).copy()
    b = np.array([Fz, M[0] / 150.0, M[1] / 150.0])
    while True:                                   # least-norm (equal pads), lifting the most negative foot until none pulls
        x, y = feet[act, 0], feet[act, 1]
        A = np.vstack([np.ones_like(x), y / 150.0, x / 150.0])
        if act.sum() >= 3:
            r = np.linalg.pinv(A) @ b
        else:                                     # 2 feet left: the box rocks on that line: vertical force exact (row x 1e3),
            w = np.array([1e3, 1.0, 1.0])         # the moment along the line in least squares, the rest is rocking inertia
            r = nnls(A * w[:, None], b * w)[0]
        if (r >= -1e-9).all() or act.sum() <= 1: break
        k = np.where(act)[0][np.argmin(r)]; act[k] = False
    R = np.zeros(len(feet)); R[act] = np.maximum(r, 0)
    return R, None


def solve(loads, mr, sr, msp, ssp, active=None, mu=MU):
    """loads: list of (point (3,), force (3,) N). Box weight from the state. Returns foot forces (4,3), tie forces on
    the ring (4,3), residual (inertia) force."""
    Wr = mr / 1000 * G; Ws = msp / 1000 * G
    cr = sr / mr; cs = ssp / msp
    pts = [cr, cs] + [np.asarray(p, float) for p, _ in loads]
    frc = [np.array([0, 0, -Wr]), np.array([0, 0, -Ws])] + [np.asarray(f, float) for _, f in loads]
    Ftot = sum(frc)
    # vertical reaction must give: sum R = -Fz ; sum R*y = -(sum moments of applied vertical about X-ish) ...
    # moment balance about the floor point (0,0,ZF): applied moment + reactions moment = 0
    Mapp = sum(np.cross(p - np.array([0, 0, ZF]), f) for p, f in zip(pts, frc))
    # reactions R_i (up) at (x_i, y_i, ZF): moment = (y_i R_i, -x_i R_i, 0); horizontal reactions at floor level add no Mx/My
    # need sum y R = -Mapp_x ; sum x R = Mapp_y
    R, res = feet_vertical(-Ftot[2], (-Mapp[0], Mapp[1]), active=active)
    H = -Ftot[:2]                                                     # horizontal the floor must give
    Rs = R.sum()
    cap = mu * Rs
    Hn = np.linalg.norm(H)
    if Hn > cap and Hn > 0: H = H * cap / Hn
    feet = np.zeros((4, 3)); feet[:, 2] = R
    if Rs > 0: feet[:, :2] = np.outer(R / Rs, H)
    resid = Ftot + feet.sum(0)                                        # unbalanced -> box accelerates (E, translation only)
    Mtot = mr + msp
    acc_force_sp = -resid * (msp / Mtot)                               # d'Alembert share on the spider body
    # spider body: its weight + back feet + inertia + ties = 0  (ties = force of ring on spider)
    fsp = [np.array([0, 0, -Ws]), feet[0], feet[1], acc_force_sp]
    psp = [cs, np.array([FEET[0, 0], FEET[0, 1], ZF]), np.array([FEET[1, 0], FEET[1, 1], ZF]), cs]
    Fs = sum(fsp); Ms = sum(np.cross(p, f) for p, f in zip(psp, fsp))
    # unknown tie forces t_i (ring on spider): sum t = -Fs ; sum p_i x t_i = -Ms   -> least norm
    A = np.zeros((6, 12))
    for i, p in enumerate(TIES):
        A[0:3, 3 * i:3 * i + 3] = np.eye(3)
        A[3:6, 3 * i:3 * i + 3] = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    t = np.linalg.pinv(A) @ np.concatenate([-Fs, -Ms])
    ties_on_ring = -t.reshape(4, 3)
    return feet, ties_on_ring, resid


def main():
    mr, sr, msp, ssp = states()
    N = len(mr); print("states", N, "mass kg", round((mr + msp).min() / 1000, 2), "..", round((mr + msp).max() / 1000, 2))
    # vectorised front-right vertical reaction over every state (4 feet, all down)
    Wr = (mr + msp) / 1000 * G; cg = (sr + ssp) / (mr + msp)[:, None]
    A = np.vstack([np.ones(4), FEET[:, 1] / 150, FEET[:, 0] / 150]); P = np.linalg.pinv(A)
    R4 = (P @ np.vstack([Wr, Wr * cg[:, 1] / 150, Wr * cg[:, 0] / 150])).T            # (N,4)
    A3 = np.vstack([np.ones(3), FEET[:3, 1] / 150, FEET[:3, 0] / 150]); R3 = (np.linalg.solve(A3, np.vstack([Wr, Wr * cg[:, 1] / 150, Wr * cg[:, 0] / 150]))).T
    i_max4 = int(np.argmax(R4[:, 2])); i_max3 = int(np.argmax(R3[:, 2]))
    i_light = int(np.argmin(mr + msp)); i_heavy = int(np.argmax(mr + msp))
    print("front-right max (4 feet)", R4[i_max4].round(1), "max with front-left lifted", R3[i_max3].round(1))
    cases = []

    def add(name, cls, i, loads=(), active=None):
        f, t, r = solve(list(loads), mr[i], sr[i], msp[i], ssp[i], active=active)
        cases.append(dict(name=name, cls=cls, foot=f[2].round(2).tolist(), tie=t[0].round(2).tolist(),
                          feet_all=f.round(1).tolist(), ties_all=t.round(1).tolist(), resid=r.round(1).tolist(),
                          mass_kg=round((mr[i] + msp[i]) / 1000, 2)))

    add("box weight, 4 feet (heaviest on this foot)", "sustained", i_max4)
    add("box weight, other front foot off the floor", "sustained", i_max3, active=[1, 1, 1, 0])
    # this foot off the floor: worst tie load over the heavy and the max state
    for i, nm in ((i_heavy, "heavy"), (i_max4, "front-heavy")):
        add(f"box weight, this front foot off the floor ({nm})", "sustained", i, active=[1, 1, 0, 1])
    for k in range(8):
        u = np.array([np.cos(np.radians(45 * k)), np.sin(np.radians(45 * k)), 0.0])
        p = np.array([0, COLLAR_Y, RIM]) - 160 * u * np.array([1, 1, 0]) / max(abs(u[0]), abs(u[1]))
        for i, nm in ((i_light, "light"), (i_heavy, "heavy")):
            add(f"rim push 13 N {45 * k} deg ({nm})", "misuse", i, [(p, 13 * u)])
    s = 1 / np.sqrt(2)
    for nm, (p, f) in {"+X": ((FACE_MID[0], COLLAR_Y, FACE_MID[1]), (s, 0, -s)), "-X": ((-FACE_MID[0], COLLAR_Y, FACE_MID[1]), (-s, 0, -s)),
                       "+Y": ((0, COLLAR_Y + FACE_MID[0], FACE_MID[1]), (0, s, -s)), "-Y": ((0, COLLAR_Y - FACE_MID[0], FACE_MID[1]), (0, -s, -s))}.items():
        for i, mn in ((i_light, "light"), (i_heavy, "heavy")):
            add(f"Overseer landing 1 kN on the {nm} face ({mn})", "misuse", i, [(np.array(p), 1000 * np.array(f))])
    json.dump(dict(states=N, tie_point=TIES[0].tolist(), foot=[160.0, 225.0, ZF], mu=MU, cases=cases), open("legs_loads.json", "w"), indent=1)
    w = max(cases, key=lambda c: np.linalg.norm(c["foot"]))
    print(f"{len(cases)} cases; worst foot {w['name']}: {w['foot']} N; worst tie",
          max(cases, key=lambda c: np.linalg.norm(c['tie']))["name"], max(np.linalg.norm(c['tie']) for c in cases).round(1), "N")
    for c in cases: print(f"  {c['name']:52s} foot {c['foot']}  tie {c['tie']}  resid {c['resid']}")


if __name__ == "__main__":
    main()
