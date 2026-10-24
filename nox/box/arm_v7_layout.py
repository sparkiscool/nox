"""Box swap arm v7 (24 Oct 2026, swap arm system worker; hub decisions after the v6 concept check, DEFAULT): the v6 stack (rails on the
motor plate) carrying both packs, Scout and Overseer, to 9 bays at 30 deg (Spark 24 Oct): one ring radius R 113 for every seated face (drone
seat and bays; a parked Overseer cartridge passes them), strokes +13, rails to r 151, rack tail to u -63, carbon ties under the rails back
in, the XT60 at each cartridge's closed (arm-side) end (the latch's change; the only option inside the float carrier for both packs), the
Overseer gripped on its drone's thrust axis (30 off its swap CG, body toward +Y), detent springs x1.18.
`use()` applies arm_v6_layout's v6 values and then patches arm_v5_layout (rails, rack, strokes) and arm_layout (R and the strokes derived
from it) in place, so every script that reads them builds v7. Import and call it before arm_kinematics (its stroke constants are read at
import). Numbers C unless tagged."""
import math, copy
import arm_layout as L
import arm_v5_layout as V
import arm_v6_layout as S6

R_V5 = 100.0
R7 = 113.0                              # every seated face (hub decision, DEFAULT; alt R 100 Scout / R 113 Overseer with two strokes)
DR = R7 - R_V5                          # 13: every stroke that ends on a seated face moves out by it
RAIL_END = 151.0                        # the outer wheel's contact at the commanded drone insert (u 32 at z0 111.3) + 6.4 of ridge + 1.0 (C)
WEB_U7 = (-63.0, 30.0)                  # head A's rack tail: the pinion (u = 55 - z0) stays 6 inside it at the commanded drone insert z0 111.3
WEB_U7_B = (-54.0, 30.0)                # head B never inserts at the drone (it pulls there at 102.6 and inserts at bays, 101.8 commanded): 6.4 inside;
                                        # its shorter tail clears the detent's springs and tower web (the full -63 tail met them, sweep C)
Y_C7 = 138.0                            # the common grip for both packs (DEFAULT): the Overseer gripped 30 toward its closed end needs Y_C >= 136.8,
                                        # the Scout >= 129.1; 138 brings the free mate pitch to 0.64 / 0.56 deg (175: 1.10 / 1.01) and the box 37 shorter
DETENT_SPRING_SCALE = 1.18              # the Overseer's 0.59 N.m at a horizontal stop with two springs at 1.25 margin (swap_system, E)
XT60_Z = 7.0                            # the cartridge's XT60 axis 7 from its closed end (DEFAULT, asked of the latch; plug face at its y as before)
OV_GRIP_OFFSET = 30.0                   # the Overseer's tab line 30 toward the closed end from its swap CG = its drone's thrust axis (Spark B)
BAYS = dict(overseer=(-60.0, -90.0, -120.0, -150.0), scout=(30.0, 0.0, -30.0, -180.0, -210.0), drone=90.0, pitch=30.0)
PHI_WINDOW = (-300.0, 90.0)             # head A at any bay or the drone, head B at any bay or the drone: one axle window (390 of the loop's 422)
TIE7 = dict(S6.TIE)                     # the tie rods as v6's alternative (rail end r 128: the stand's arm lane at low Y keeps them inside r 135)
_DONE = False


def strokes():
    z = dict(park=L.Z0_A_HOME, wait=81.0 + DR, bay=84.6 + 1.5 + DR, drone=84.6 + 5.0 + DR, insert=84.6 + 5.0 + V.DRONE_PUSH + DR)
    z["bay_cmd"] = z["bay"] + S6.OVERTRAVEL; z["insert_cmd"] = z["insert"] + S6.OVERTRAVEL
    return z


def _patch_v5():
    z = strokes()
    V.RAIL_R = (V.RAIL_R[0], RAIL_END)
    V.WEB_U = WEB_U7; V.WEB_U_B = WEB_U7_B
    V.Z0S = (z["park"], z["wait"], z["bay"], z["drone"], z["insert"], z["bay_cmd"], z["insert_cmd"])
    S6.Z0_INSERT_MAX = z["insert_cmd"]; S6.Z0_BAY_INSERT = z["bay_cmd"]


def use():
    """v6 values, then the v7 patches (idempotent); registers itself so a later S6.use("v6") (any v6 script's import) keeps v7."""
    global _DONE
    if _patch_v5 not in S6.POST_V6: S6.POST_V6.append(_patch_v5)
    S6.use("v6")
    if not _DONE:
        L.R = R7
        L.R_BAY_TOP = L.R + L.CART_H; L.R_BAY_BACK = L.R_BAY_TOP + 6.0
        L.Z0_BAY = 84.6 + DR; L.Z0_DRONE = L.Z0_BAY + 5.0; L.Z0_B_WAIT = 81.0 + DR
        L.Y_C = Y_C7
        _DONE = True
    _patch_v5()
    return strokes()


if __name__ == "__main__":
    z = use()
    print("strokes", z)
    w = V.clearance_sweep((53.93, -43.67)); g = min(w.values())
    print("2D clearance min", round(g, 2), min(w, key=w.get))
    print("reach: wheel rim at the commanded insert r", round(math.hypot(V.WHEEL_X, z["insert_cmd"] + V.WHEELS[1][1] + V.WHEEL_OD / 2), 1),
          "plate corner", round(math.hypot(V.PLATE_X[1], z["insert_cmd"] + V.PLATE_U[1]), 1), "pinion in the rack tail by",
          round((V.PIN_Z - z["insert_cmd"]) - V.WEB_U[0], 1))
