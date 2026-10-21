"""Box swap arm layout numbers (v1 15 Oct 2026; v3 and v4 16 Oct: see the v3 note below and the v4 block at the end, which
overrides the cartridge interface, Y_C, the far wall, park heights, the bay list and the top opening). Every number here is C (CAD) unless tagged E (estimate) or
CHECK (hardware not measured). The layout came out of the radial-stack optimisation in layout_opt(): the ring
radius R is the smallest that clears the three NEMA17s standing beside the Ø90 hub (the two 6010 bearings set
the hub), with the carried pack and the parked empty head inside it.

Wheel coordinates (W): origin on the axle at the inner face of the stub-side casing wall.
  Y = axial (along the axle, + away from the wall) = the drone's left / right (B+ / B- side of the cartridge).
  Z = radial up at the drone position (the top of the ring).  X = tangential at the top (the drone's nose axis).
Arm v3 (16 Oct 2026): two flat 2040 bars at 90 deg (Spark: one pitch apart; 45 cannot hold two packs inside R 100).
Head A's bar points along phi (the arm angle), head B's along phi + 90 (counter-clockwise, on A's -X side). In the
arm frame (A up): B's axis is -X, B's local +X is the arm's +Z.  Angles are in the XZ plane from +X.
Carried packs stay inside r 89.4 so a turning pack's corners (r 98.5) clear the seated packs' faces at r 100: the
glide rides UNDER the pack (pack bottom 15.4 above the glide's inner end), parked at r 26..70, so the two glides
never meet at the axle (v2 had them crossing and parked B's pack at r 100: corners at r 108 through the bays).
"""
import math

# ---------------- bought hardware (A1 mini plates 180 cube) ----------------
BRG_ID, BRG_OD, BRG_W = 50.0, 80.0, 16.0          # 6010-2RS, 50 x 80 x 16 (P marking + OD / ID, Spark 17 Oct)
SLIP_D, SLIP_L = 22.0, 18.0                        # capsule slip ring body (E, CHECK): pocket in the stub
SLIP_FL_W, SLIP_FL_T = 44.0, 1.5                   # its flange plate (E, CHECK), 2 x M3 at 32 pitch? CHECK
NEMA, NEMA_L, NEMA_BOSS, NEMA_SHAFT, NEMA_HOLES = 42.3, 48.0, 22.0, 24.0, 31.0   # StepperOnline 17HS19-2004S1 (label, P 17 Oct): width 42.4 P; length, boss, shaft, holes per its datasheet (E)
P80_OD, P80_FL, P80_W, P80_BOSS, P80_BOSS_L = 51.6, 55.0, 10.0, 30.0, 8.0        # GT2 80T 8 bore: flange 55, collar 30, total 18 (P, Spark 17 Oct 12:41); W / BOSS_L split E, CHECK
P20_OD, P20_FL, P20_W = 12.7, 16.0, 14.0           # GT2 20T 5 bore, incl. boss: real flange 15.7, length 15.7 (P 17 Oct); model kept 16 / 14, set on the shaft to the belt line
IDLER_OD, IDLER_BORE, IDLER_W = 16.0, 5.0, 10.0    # kit idler, 20T toothless (E, CHECK: 3 or 5 bore)
BELT_W, BELT_T = 6.0, 1.4                          # GT2 6 mm open belt
SERVO_L, SERVO_W, SERVO_H = 40.0, 20.0, 40.7       # DS3218 body (listing 40 x 20 x 40.5, P listing); case top at the shaft end 40.7 above the base (P photo +-1, 17 Oct)
SERVO_EAR_L, SERVO_EAR_T, SERVO_EAR_Z = 54.5, 2.9, 28.2   # ears: span (P photo, listing 54), thickness, underside above the base (P photo +-1)
SERVO_HOLES = (49.5, 10.0)                         # 4 holes (standard size; photo ~49, P photo +-1); MG995 the same (E, CHECK on arrival)
SERVO_SHAFT_X = 8.5                                # output axis from the near end of the body (P photo +-1.5; the cut covers 7..10.5)
SERVO_BOSS_D, SERVO_BOSS_H = 13.0, 4.0             # boss above the top flat (E)
EXTR = 20.0                                        # 2040 bar flat: 20 axial (Y) x 40 tangential (X)
EXTR_W = 40.0
SLOT_W, SLOT_D = 6.0, 6.0                          # V-slot opening and depth (E)

# ---------------- cartridge interface (v2/dimensions.md E / J, s2_cartridge.py) ----------------
CART_W = 40.0        # over the guide rails (x, tangential since v4); cartridge v3 (16 Oct 19:15)
CART_X_OUT = 19.0    # tray outer half width (x)
CART_H = 48.0        # y: flat bottom (0) to the top
CART_L = 79.0        # z: along the pack (along the axle since v4, Spark 16 Oct 17:34)
CART_RAIL_Y0 = 38.0  # guide rail x +-19..+-20 over y 38..48, full length
TAB_Y0, TAB_IN, TAB_OUT, TAB_W = 0.0, 4.0, 10.0, 14.0     # v2: tabs sunken in a 29 x 14 x 7.65 recess open to the flat bottom; faces x 4 / 10 at rest; 14 wide (z)
TAB_SQZ = 4.0                                      # travel to the stop (faces at 4 / 10)
POCK_YC, POCK_H, POCK_W, POCK_D = 4.0, 4.0, 8.0, 3.0      # grip pocket centred y 4 (v2)
Z_TABC = 37.5                                      # tab centre along the pack (bolt plane, v3)
Y_PIN, Z_PINC, PIN_PITCH, PIN_N, PIN_BORE = 41.5, 66.5, 5.4, 4, 3.4   # v3
X_PIN_TIP, PIN_LEN = 18.56, 15.21
RAIL_FACE = 20.3                                   # receiver locating face (x), 0.3 off the guide rail, v3
PINBLK_X1 = X_PIN_TIP + PIN_LEN + 1.0              # 33.87 pin block outer face (x)

# ---------------- axial stack (Y) ----------------
Y_WALL = 0.0
Y_FLANGE1 = 6.0            # stub flange Y 0..6
Y_STUB1 = 42.0             # stub Ø50 to Y 42
Y_HUB0, Y_HUB1 = 7.0, 50.0 # hub body (bearings Y 8..24, 24..40; shoulder 40..44; web 44..48; plate 50..54 below)
Y_BRG = (8.0, 24.0)        # bearing 1 and 2 start
Y_PLATE0, Y_PLATE1 = 50.0, 54.0   # motor plate: motor faces at Y 50 (bodies Y 2..50), shafts to Y 74
Y_P80 = (54.0, 54.0 + P80_W)  # 80T body (belt 56..62), collar to 72 (total 18, P); top face moves with P80_W (CHECK)
Y_AXBELT = (56.0, 62.0)
Y_HBELT = (66.0, 72.0)     # head A belt (on A's +X side)
Y_HBELT_B = (56.0, 62.0)   # head B belt: in the axle belt's plane (the two head belts would cross at Y 66..72)
Y_BAR = (74.0, 94.0)       # 2040 flat, 20 axial; Y-face slots at X +-10, X-face slots at Y 84
Y_GLIDE1 = Y_BAR[1] + 3.3  # 97.3: glide top skin (0.3 clearance, 3.0 skin)
Y_C = 175.0                # v4c (Spark 17 Oct 02:02): tab centre = middle of the universal bay (Overseer cartridge E 153 long from Y 98.5); Scout pack Y 137.5..216.5 (was 136)
Y_SERVO = (Y_C - 10.5, Y_C - 10.5 + 40.5)   # servo lies along Y, turned round: shaft end inboard at 115.5, body to 156, ears 108.25..162.75
Y_FAR_WALL = 255.0         # v4c: far end of the universal bay module (was 184); v4: far end of the wheel's envelope (no casing since 16 Oct 21:09; was the far wall), the pack's 79 runs along the axle
Y_ELEC = -60.0             # electronics bay behind the stub wall (PSU 30, SKR, chargers) (E)

# ---------------- radial layout ----------------
R_HUB = 44.0               # hub body outer radius (Ø88 round the Ø80 bore, 4 wall)
# motors standing beside the hub in the band Y 2..50, seen along the axle as rotated squares (xc, zc, rot deg).
# Poses from the pose search (15 Oct): nearest point to the axis >= 46.5 (hub 44), farthest <= 101 (R 104),
# head-motor bodies clear of their own belt runs, hub part inside 170 in X and Z.
# Poses from arm_pose.py (16 Oct, 90-deg V): bodies' edges toward the axle, nearest point >= 46.5, bodies >= 3 apart,
# seats inside |x|,|z| <= 88 (180 plate), belt bands clear of posts and of each other; residual unbalance 4.4 g.m (E).
MOTORS = {"A": dict(xc=34.0, zc=-59.0, rot=25.0),        # head A belt at X 34 (runs 27.65 / 40.35), pulley Z -59 (Y 66..72)
          "B": dict(xc=61.0, zc=55.0, rot=0.0),          # head B belt line at arm Z 55 (B's X 55: runs 48.65 / 61.35), pulley at X 61 (Y 56..62)
          "axle": dict(xc=69.4 * math.cos(math.radians(-120.0)), zc=69.4 * math.sin(math.radians(-120.0)), rot=50.0)}   # 80T sun, C 69.4: spliced 122T loop from the 6 mm open belt (axle drive v4 block at the end)
R_MOTOR = math.hypot(34.0, 59.0)   # 68.1 head A motor's shaft radius
ANG_A = math.degrees(math.atan2(-59.0, 34.0)); ANG_B = math.degrees(math.atan2(55.0, 61.0)); ANG_AX = -120.0
B_ANGLE = 90.0             # head B's bar at phi + 90 (counter-clockwise from A)
R = 100.0                  # ring radius = flat bottom face of a seated cartridge (bays and drone alike); fingers reach 7 into its recess
R_PIN = R + Y_PIN                    # 142.5 bay pins (and strips)
R_BAY_TOP = R + CART_H               # 148 cartridge top in a bay
R_BAY_BACK = R_BAY_TOP + 6.0         # 154 cradle back (= casing inner half width)
BAR_HALF = 134.0           # bars end at r 134 (corner r 135.5: the bays' -Y pin blocks start at r 136.8, Y >= 91.3)
BAR_A = (22.0, 134.0)      # A's bar r 22..134 (112 long); the other bar starts at r 22 too: the two never meet (|x| 20 < 22)
BAR_B = (22.0, 134.0)      # B's bar (B's own axis). Two 112 bars from the 230+ 2040 (Spark 16 Oct 02:50)
POSTS_A = (32.0, 76.0)     # bar posts on the hub (centres along each bar, 12 long, X +-16, 2 x M5 into the -Y face slots at X +-10);
POSTS_B = (32.0, 76.0)     # the gap 38..70 between them lets head B's belt (arm z 48.65..61.35, Y 56..62) pass A's posts
N_POS = 8                  # ring positions at 45 deg, the top one = drone; 7 bays (3 Overseer + 4 Scout, Spark 15 Oct)
BAY_ANGLES = [90.0 - 45.0 * k for k in range(1, N_POS)]   # bays clockwise from the drone (deg)
RING_FEET = (45.0, -45.0, -135.0, 135.0)   # the bay ring's 4 far-wall feet (one structure: the ring carries the 0, 180 and -90 bays round to them)

# ---------------- carriage / gripper (head A at the top; z0 = inner end of the glide along Z) ----------------
GLIDE_L = 44.0             # glide length along the bar
GLIDE_X = 24.0             # glide half width (bar +-20, 0.3 clearance, 3.7 skins), as v1
GRIP_DU = -22.6            # v3: the gripper sits 22.6 further in than v1/v2 relative to the glide (the glide rides under the pack)
RIB_DZ = 42.0 + GRIP_DU    # 19.4: rib centre above the glide's inner end (Lrib)
FINGER_END_DZ = 45.5 + GRIP_DU   # 22.9: finger tip above the glide's inner end (7.5 into the 7.65 recess)
CART_BOT_DZ = RIB_DZ - POCK_YC            # 38: carried cartridge's flat bottom above z0
CART_TOP_DZ = CART_BOT_DZ + CART_H        # 86: carried cartridge top above z0
Z0_A_HOME = 32.0           # head A parked / carrying: glide r 26..70, pack r 41.4..89.4 (corners r 98.5)
Z0_B_HOME = 32.0           # head B the same: B's glide at arm x <= -26 passes A's glide (x >= -24) by 2
Z0_B_WAIT = 81.0           # B waits under the drone seat with its blade tips at r 103.9 (the seated pack's bottom is r 105): it never lands onto raised fingers
Z0_BAY = R + POCK_YC - RIB_DZ               # 62: rib on a bay cartridge's pocket (bottom face at R)
Z0_DRONE = Z0_BAY + 5.0                     # 67: 5 over-travel at the drone (DEFAULT)
ON_BAR_MIN = BAR_HALF - Z0_DRONE            # 27 of glide left on the bar at full extension
CLAMP_DZ = (-18.6, 21.4)   # head A belt clamp zone 40 (clamp v2, belt_clamp_v2_head.py, 17 Oct 19:10: two loop-and-pin ends back to back, 39.7 used);
                           # top pushed to the full-stroke limit (idler flange + 1 at the drone) = most overlap with the glide (C)
CLAMP_DZ_B = (-15.6, 24.4) # head B the same; its joint to the glide is generated only at u >= -4 (at park 4 past A's glide skin, arm x <= -28)
X_HBELT_C = MOTORS['A']['xc']                         # 34: head A belt runs at X 34 +- 6.35 (inner run 27.65)
X_HBELT_C_B = MOTORS['B']['zc']                       # 55: head B belt (in B's own frame) runs at X 55 +- 6.35 (inner run 48.65)
Z_MOTOR_A = MOTORS['A']['zc']                         # -59
Z_MOTOR_B = -MOTORS['B']['xc']                        # -61 in B's own frame (arm X 61 = B's -Z)
Z_IDLER = 120.0                                       # head A idler centre: A's clamp top reaches u 111 at the drone (+ rim 8 + 1 clear)
Z_IDLER_B = 124.0                                     # head B idler: B's clamp top reaches u 114 at the drone (+ cheek r 9 + 1); rim r 143.6 at Y 54..64 (bays start at Y 88)
X_SERVO = (-45.5, -9.0)    # servo body in X for head B (shaft along +X through the plate at X -18..-12; boss to -5, pinion -4..2); head A mirrored (+X)
Z_SERVO_DZ = (8.0 + GRIP_DU, 28.0 + GRIP_DU)   # -14.6 .. 5.4: servo body radial band (its 20 width), lying along Y (Y_SERVO)
Z_PINION_DZ = 18.0 + GRIP_DU                   # -4.6: pinion axis (servo shaft), at Y = Y_SERVO[0] + SERVO_SHAFT_X = Y_C
X_RACK = (-4.0, 4.0)       # racks and finger roots in X (the tab is 14 wide: finger blades X -7..7)
RACK_A_DZ = (24.3 + GRIP_DU, 30.3 + GRIP_DU)   # 1.7 .. 7.7: upper rack (finger on the Y_C - side)
RACK_B_DZ = (5.7 + GRIP_DU, 11.7 + GRIP_DU)    # -16.9 .. -10.9: lower rack (finger on the Y_C + side)
FINGER_T = 3.0             # finger blade thickness (Y), v2: blades reach 7 into the bottom recess
FINGER_OPEN = 1.0          # open: inner faces at Y_C +- 11 (closed on the tabs +- 10, squeezed +- 6)


def part_angle_pos(r, ang):
    a = math.radians(ang); return r * math.cos(a), r * math.sin(a)


def layout_opt():
    """why R = 96: the smallest ring that clears everything inside it. Returns the binding constraints (mm)."""
    def square_edge_pts(m, half):
        c, sn = math.cos(math.radians(m["rot"])), math.sin(math.radians(m["rot"]))
        corners = [(sx * half, sz * half) for sx, sz in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        pts = []
        for i in range(4):
            (ax, az), (bx, bz) = corners[i], corners[(i + 1) % 4]
            for k in range(21):
                t = k / 20; ex, ez = ax + t * (bx - ax), az + t * (bz - az)
                pts.append((m["xc"] + ex * c - ez * sn, m["zc"] + ex * sn + ez * c))
        return pts
    rs = [math.hypot(x, z) for m in MOTORS.values() for x, z in square_edge_pts(m, NEMA / 2)]
    diamond_out, motor_in = max(rs), min(rs)
    bar_corner = math.hypot(BAR_HALF, EXTR_W / 2)
    carried_top = Z0_B_HOME + CART_TOP_DZ
    b_fingers = Z0_B_HOME + FINGER_END_DZ
    return dict(motor_corner=(diamond_out, R - diamond_out), bar_corner=(bar_corner, R - bar_corner),
                carried_top=(carried_top, R - carried_top), B_fingers=(b_fingers, R - b_fingers),
                motor_to_hub=motor_in - R_HUB, on_bar_at_drone=ON_BAR_MIN,
                bay_pitch_at_pins=2 * math.pi * R_PIN / N_POS)


if __name__ == "__main__":
    for k, v in layout_opt().items(): print(f"{k:18s} {v}")
    print("R_PIN", R_PIN, "R_BAY_TOP", R_BAY_TOP, "R_BAY_BACK", R_BAY_BACK, "Z0 bay/drone", Z0_BAY, Z0_DRONE)

# ---------------- casing (printed tiles, glued; DEFAULT) ----------------
CASE_IN = 154.0            # inner half width in X and Z (bay backs at r 154)
CASE_T = 2.0               # skin thickness (wall-class minimum; ribs carry the stiffness)
TILE = 158.0               # face tiles 158 square (2 x 2 per face), rim tiles 158 x 119
Y_BACK_IN, Y_BACK_OUT = -56.0, -60.0        # electronics bay Y -56..-4 (PSU 30 + boards); back skin -60..-56
Y_STUB_WALL = (-4.0, 0.0)                   # partition = stub wall, flange bolts at 45 deg (one per tile)
Y_FAR_OUT = Y_FAR_WALL + CASE_T             # 176
TOP_OPENING = (-42.0, 46.0, Y_C - 23.0, Y_C + 23.0)   # X0, X1, Y0, Y1: cartridge (Y_C +- 20) + 3 through the top wall


# ================================================================== v4 (16 Oct 20:30): cartridge long side along the axle (Spark 17:34),
# squeeze tangential; Scout bays as bolted modules (Spark 19:28). Head frame: X tangential, Y axial, u radial from the glide's
# inner end. Cartridge -> W at a position: X = -x, Y = Y_C + (z - Z_TABC), Z = r_bottom + y (a proper rotation).
SCOUT_BAYS = (0.0, -45.0, -90.0, -135.0)     # one contiguous arc (bay at 0: A picks while B waits under the drone)
OVERSEER_BAYS = (45.0, 135.0, 180.0)         # reserved, drawn after D10 (same link pattern)
BAY_ANGLES = list(SCOUT_BAYS)
# gripper v4 (head A; head B = mirrored in X): servo standing radially under the pack, shaft +u; pinion axis along u;
# racks side by side in s (= Y - Y_C), moving along X. Placement from the envelope rules (arm_layout_v4_opt below):
# park bisector (A's parts at X + Z >= 1.5 in the arm frame, Z = z0 + u) and r <= 98.5 at the drone (z0 89.6) except
# inside the own mouth |X| <= 19.
P_SH = 33.5                  # pinion / servo shaft X
U_PIN = (-10.0, -4.0)        # pinion and racks, u
U_BOSS = (-11.0, -10.3)      # v4d (17 Oct, servo from photos): spline boss / horn seat 0.7 over the case top; pinion on the spline from -10
U_SERVO = (-51.7, -11.0)     # v4d: body 40.7 to its case top at the shaft end (drawn flat to that height: the real top slopes 3 lower); 1.0 under the pinion
U_EARS = (-23.5, -20.6)      # v4d: underside 28.2 above the base, 2.9 thick
S_SERVO = (-8.5, 31.5)       # v4d: body along s (shaft 8.5 from its near end); ears -15.5..38.5; the body cut covers shaft offsets 7..10.5
U_PLATE = (-20.6, -14.6)     # servo plate on the ears (window for the body); M3 through slots +-1.5 along s + nuts on top (DEFAULT)
RACK_S = (6.0, 12.0)         # rack A at s +6..+12 (pitch line s 6, teeth on its -s face); rack B mirrored in s
RACK_A_X = (-15.0, P_SH + 7.0)      # carries the -X blade; moves +X on squeeze (4), -X when opened (1)
RACK_B_X = (10.0, P_SH + 10.0)      # carries the +X blade; moves -X on squeeze
TEETH_A = (P_SH - 9.0, P_SH + 6.0)
TEETH_B = (P_SH - 6.0, P_SH + 9.0)
GUIDE_X = (17.0, 44.0); GUIDE_U = (-12.0, -2.0); GUIDE_S = 15.0
BLADE_IN, BLADE_T, BLADE_THICK = 10.0, 3.0, 5.0   # inner faces X -+10 on the tabs; 3 thick in the recess, 5 below u 14
BLADE_S = 6.5                # blades 13 wide along s in the 14 long recess
U_HEAD_TOP = 14.0            # nothing of the head above u 14 under the pack (pack bottom u 15.4)
ENV_R = 98.5                 # head points outside |X| <= 19 stay inside r 98.5 at every position (drone z0 the worst)
BISECT = 1.5                 # A's parts at X + z0 + u >= 1.5 when parked (B mirrored): 3 mm between the heads
# bay modules
LINK_R = (125.0, 146.0)      # link bosses (M3 heat-set) on the -Y face, 6 inside each joint plane
FOOT_R = (122.0, 147.0)      # retired: far-wall feet (the casing is retired, Spark 16 Oct 21:09; feet come from nox/box-stand.md)
JOINT_GAP = 0.3              # each module stops 0.3 short of its 22.5 deg plane
TOP_OPENING = (-23.0, 23.0, Y_C - 40.0, Y_C + 44.0)   # carried cartridge (X +-20, Y 98.5..177.5) + 3 / 2.5
Y_FAR_OUT = Y_FAR_WALL + CASE_T

# balance-lead charging (Spark 16 Oct 20:39): 5 pads on the cartridge's -x wall (bay +X), AA spring contacts in each bay
BAL_Z = (8.0, 18.0, 28.0, 38.0, 48.0)   # pad centres along the pack (cartridge z), pitch 10 (E, Spark), C
BAL_Y = 24.0                            # pad centre height (cartridge y), C
BAL_PAD_X = 18.5                        # pad face |x| (1.5 under the band's lip at 20.0), C
BAL_X_BASE = 22.35                      # contact's mounting-leg front |X| in the bay: AA negative leaf contact (MPD BK-209 type, on hand,
                                        # P listing: 6.35 deep free, 7.92 wide, 11.40 tall, leg 1.57), seated 3.85 = 2.5 deflection (E)
BAL_SPRING_N = 1.5                      # per contact at 2.5 deflection (E, CHECK): 5 x 1.5 = 7.5 N sideways, 15 N design
BAL_LEAF = (7.92, 11.40, 1.57)          # width (along the row, z: 2.1 between contacts at 10 pitch), height (radial), leg thickness (P listing)
BAL_WIN = (9.0, 7.0)                    # lip window per pad, along the row x radial: passes the 7.92 leaf; 1.0 lip bars between pads (C)


# ================================================================== v4c (Spark 17 Oct 02:00..02:02): universal bay modules
# Every bay is one module sized for the Overseer cartridge (estimated from the pack 148 x 48 x 45, P; the Overseer receiver is
# not drawn), the Scout receiver centred on the gripper's tab line Y_C. Modules bolt long side to long side round the ring with
# one bay-sized opening (the drone). Pitch 30 deg: the widest that packs the modules and divides the heads' 90 deg.
PITCH = 30.0
N_POS = 12                                    # 12 positions at 30 deg; the top three (60, 90, 120) are the opening
TOP_OPEN = 90.0                               # opening 90 wide, centred on the drone (Spark 17 Oct 02:27: as much as the funnel allows):
                                              # a module at 60 / 120 would reach into the collar's 168 opening column (|X| <= 84) and the
                                              # drone's feet (X +-30.75, Z 102) and plate; with 45 deg edges every module point above Z 102
                                              # has |X| >= Z > 84 (C)
BAY_ANGLES = [0.0] + [-PITCH * k for k in range(1, 8)] + [30.0]   # 9 bays; bay 0 first (A picks there while B waits under the drone)
SCOUT_BAYS = tuple(BAY_ANGLES)                # every bay takes a Scout module now (Spark 17 Oct 02:02: Scout only)
OVERSEER_BAYS = ()
HALF = PITCH / 2.0                            # each module's sector half angle
OV_L, OV_W, OV_H = 153.0, 51.0, 57.0          # Overseer cartridge envelope (E): pack 148 + 2.5 walls along Y, 45 + walls + rails, 45 radial + latch recess + top
MOD_Y = (94.0, Y_FAR_WALL)                    # module along the axle (C): bars' sweep ends at 95.5 (r < 136.5)
MOD_R = (99.5, 163.0)                         # module radially (C): Overseer top at R + 57 + strip 1.5 + back 3 + 1.5
LUG_Y = (145.0, 209.0)                        # 2 M3 joints per side, right over the Scout receiver and back plate (Spark 17 Oct 03:25: the
                                              # joints at the Overseer-length ends made long arms); an Overseer module uses the same pattern
LUG_RHO = (165.0, 179.0)                      # outer lugs along the joint plane, above the dovetail; bolt at rho 172: head and nut reached from outside
LUG_W = 14.0                                  # lug width along Y (M5: Ø5.5 hole, ~4 walls; Spark 17 Oct 04:28 / 04:29: M5 bolts and nuts)
M5_CLR_R, M5_NUT_AF, M5_NUT_T = 2.75, 8.0, 4.3   # M5 clearance; hex nut 8 AF in a pocket 4.3 deep on the insert-side lug's far face (DEFAULT)
LUG_T = (7.0, 9.0)                            # lug thickness: head side (+X) 7, nut side (-X) 9
DT_RHO_C = 157.0                              # one dovetail per side running the full module length along Y (Y-axis profile: prints
                                              # straight up the build axis), above the back plate and the shoe modules (r <= 152.5) and
                                              # inside a future Overseer path; holds the joint shut and lets any module slide out along +Y
                                              # once its 4 bolts are out (a radial slide is blocked by any shear key: 17 Oct 03:25)
DT_RHO = (147.5, 168.0)                       # the tongue's base pad / the groove block, along rho (groove walls >= 4: 17 Oct 04:24 the 1.5 walls vanished at h 2)
DT_SEG = ((135.0, 155.0), (199.0, 219.0))     # tongue and groove-block segments along Y at the lugs (the groove itself is cut the full length)
DT_W0, DT_W1, DT_D = 6.0, 12.0, 3.0           # neck / tip width (along rho), depth (45 deg flanks)


# ================================================================== axle drive v4 (17 Oct 2026; Spark 01:37: no closed 244 belt, only
# 6 mm open GT2 on hand, use as little as possible). The axle motor rides on the arm and walks round a SUN that must be
# FIXED to the stub (planetary). Hub v3 carries the 80T's Ø8 stub and its 3 M3 bosses, so as drawn the 80T turns with the
# motor: the sun needs a fixed post through the hub (axle_drive()['blocking'], hub chat). Belt pitch numbers: GT2 2 mm.
# Option 1, printed spur gears m1 (28T motor : 111T sun, C 69.5): REJECTED, arm_kinematics.py 1.0 gears: the 111T
# (tip r 56.5) crosses head B's belt (arm z >= 48.65, Y 56..62), both inner bar posts (r 26..41, Y 52..74) and head A's
# belt and clamp if moved to Y 66..72; no axial slot in Y 52..74 is free of them (C).
# Option 2 (DEFAULT): the 80T sun + a loop cut from the 6 mm open belt, lap-spliced, current C 69.4, axle motor unmoved.
AX_SUN_T, AX_PIN_T, AX_PITCH = 80, 20, 2.0
AX_LOOP_T = 122                     # 244 pitch length at C 69.4 (C)
AX_SPLICE_T = 15                    # lap 15 teeth (30 long): outer end's teeth glued on the inner end's back (DEFAULT)
AX_SPLICE_END_CLR = 5.0             # the lap stays >= 5 off the 20T's wrap at both ends of the arm's range (DEFAULT)
PHI_RANGE = (-225.0, 90.0)          # arm angles one swap cycle uses (Spark 17 Oct, E): 315 deg, bounded (no net winding)
GEAR_REJ = dict(m=1.0, z_sun=111, z_pin=28, C=69.5, r_tip_sun=56.5, r_tip_pin=15.0, width=6.0, Y=(56.0, 62.0))
HEAD_BELT_PD = 2.0 * 20 / math.pi   # 12.73: 20T pitch dia; the toothless idler taken as the same (E, CHECK)
IDLER_FLANGE_R = {"A": IDLER_OD / 2, "B": 9.0}   # what the clamp must pass under: A's idler flange (cheek below the belt); B's cheek r 9 (above it)
CLAMP_TOP_CLR = 1.0                 # clamp top to that radius at full stroke (DEFAULT)
CLAMP_LAND = 8.0                    # v1 land (retired); axle_drive()'s head cut numbers are v1: for clamp v2 cuts see nox/box-arm.md (A ~445, B ~457)


def _loop(C, d1, d2):
    """open-belt loop over two pulleys (pitch dias d1 >= d2) at centre distance C: length, spans, wraps (rad)."""
    a = math.asin((d1 - d2) / 2 / C)
    span = math.sqrt(C * C - ((d1 - d2) / 2) ** 2)
    w1, w2 = math.pi + 2 * a, math.pi - 2 * a
    return dict(L=2 * span + w1 * d1 / 2 + w2 * d2 / 2, span=span, wrap1=w1, wrap2=w2)


def axle_drive():
    """axle drive v4 numbers (C unless noted): the spliced loop's length and cut, how far the arm may turn before the lap
    reaches the 20T, and the head belts' loop and cut lengths (current idlers, and the shortest the full stroke allows)."""
    pd80, pd20 = AX_SUN_T * AX_PITCH / math.pi, AX_PIN_T * AX_PITCH / math.pi
    C = math.hypot(MOTORS["axle"]["xc"], MOTORS["axle"]["zc"])
    lp = _loop(C, pd80, pd20)
    lap = AX_SPLICE_T * AX_PITCH
    free = lp["L"] - lp["wrap2"] * pd20 / 2 - 2 * AX_SPLICE_END_CLR          # loop outside the 20T wrap, less the end margins
    belt_per_rad = pd80 / 2                                                   # planetary: the belt runs round the arm by r_sun per radian
    rng = math.degrees((free - lap) / belt_per_rad)
    out = dict(C=round(C, 2), loop_exact=round(lp["L"], 2), loop_T=AX_LOOP_T, lap=lap, cut=AX_LOOP_T * AX_PITCH + lap,
               wrap_sun_deg=round(math.degrees(lp["wrap1"]), 1), wrap_20T_deg=round(math.degrees(lp["wrap2"]), 1),
               span=round(lp["span"], 2), lap_range_deg=round(rng, 0), needed_deg=PHI_RANGE[1] - PHI_RANGE[0],
               margin_deg=round(rng - (PHI_RANGE[1] - PHI_RANGE[0]), 0),
               ratio=AX_SUN_T / AX_PIN_T)
    for h, zm, zi, cz in (("A", Z_MOTOR_A, Z_IDLER, CLAMP_DZ), ("B", Z_MOTOR_B, Z_IDLER_B, CLAMP_DZ_B)):
        L = 2 * (zi - zm) + math.pi * HEAD_BELT_PD                            # equal pulleys: loop pitch length
        zmin = Z0_BAY + 5.0 + cz[1] + IDLER_FLANGE_R[h] + CLAMP_TOP_CLR      # lowest idler the full stroke (drone) allows
        Lmin = 2 * (zmin - zm) + math.pi * HEAD_BELT_PD
        cut = AX_PITCH * round((L - CLAMP_LAND) / AX_PITCH)                    # open belt = loop less the clamp's centre land
        out[f"head{h}"] = dict(loop=round(L, 1), cut=cut, idler_min=round(zmin, 1), loop_min=round(Lmin, 1),
                               cut_min=AX_PITCH * round((Lmin - CLAMP_LAND) / AX_PITCH))
    out["sun_mount"] = ("hub v4 (17 Oct): the 80T sits on the fixed sun post (arm_cad.sun_post) through a Ø45.5 hole in the hub; "
                        "inner bar posts at r 35 (A) / 38.75 (B), >= 1 off the 80T and the loop (was blocking with hub v3)")
    return out


if __name__ == "__main__":
    for k, v in axle_drive().items(): print(f"axle {k:16s} {v}")


# ================================================================== hub v4 (17 Oct 2026): FIXED sun post (planetary axle drive)
# The 80T sun is bolted to a printed post that is bolted to the stub end (2 x M3 at Z +-21, the old bearing cap's inserts) and
# passes through a hole in the hub web and motor plate. Hub v4 drops v3's Ø8 pulley stub and its 3 M3 bosses. The post
# also clamps bearing 2's inner race (the stub now ends flush with it, Y 40: the old cap at Y 42..44 never touched the race).
# Leads: a 19-wire bundle twists along the axle inside the post's bore (bounded travel, DEFAULT; the slip ring is retired, it
# can still sit on the post tip later). Numbers below are from arm_cad.sun_post_search() (python3 arm_cad.py post), C.
Y_STUB_END = 40.0                   # stub shaft Y 6..40, end flush with bearing 2's inner race (was Y_STUB1 42), C
CLR_POST = 1.0                      # radial and axial clearance, every fixed (post, 80T, loop) to rotating (hub) gap, DEFAULT
INSERT_R, INSERT_WALL = 2.0, 1.6    # M3 heat-set hole r and the wall round it (DEFAULT)
SUN_BC_R = 18.0                     # 80T web bolt circle Ø36 (Problem 4, DEFAULT, CHECK)
SUN_BOLT_ANG = (0.0, 120.0, 240.0)  # 3 x M3 on Ø36, turned 30 deg off v3's 90/210/330 so they miss the stub bolts at +-90
POST_BOLT_Z = (-21.0, 21.0)         # 2 x M3 into the stub's inserts (were the bearing cap's), C
POST_FL_R, POST_FL_Y = 28.5, (40.0, 44.0)    # flange: bears on bearing 2's inner race (r 25..29.1, 6010 d1 58.3 E, CHECK)
POST_R = 21.75                      # column r: SUN_BC_R + INSERT_R + INSERT_WALL (21.6) up to 0.25, C (search)
POST_HOLE_R = 22.75                 # hub web + motor plate hole r = POST_R + CLR_POST (Ø45.5), C (search)
POST_Y1 = 54.0                      # column top = the 80T's bottom face (axle belt plane unchanged, Y 56..62)
POST_SPIGOT_R, POST_SPIGOT_Y1 = 5.95, 62.0   # Ø11.9 locating spigot 8 into the 80T bore, drilled to Ø12 (Problem 4)
POST_BORE_R = 4.25                  # Ø8.5 lead bore through the post: 19-wire bundle Ø7.3 (arm_wiring.py, 1.2 clear)
POST_FLUTE_R = 3.25                 # Ø6.5 access flutes down the column to the 2 stub bolts (M3 SHCS head Ø5.5)
HUB_RECESS_R = P80_FL / 2 + CLR_POST          # 29: hub stays at Y <= 53 inside r 29, 1 under the 80T's flange
HUB_RECESS_Y = Y_P80[0] - CLR_POST            # 53
POSTS_A = (35.0, 76.0)               # A's inner post r 32 -> 35 (sun_post_search: 1.0 off the 80T flange, 2.0 off the loop), C
POSTS_B = (38.75, 76.0)             # B's inner post r 32 -> 38.75 (sun_post_search: 1.04 off the loop, 4.75 off the 80T), C
AX_SLOT = 1.0                       # axle motor seat: 4 x M3, boss hole and body keep-out slotted +-1 along motor-to-axle (C 68.4..70.4)
WIRE_TWIST = dict(L=90.0, n_wires=19, bundle_d=7.3, travel_deg=421.0)   # stub grommet (Y ~ -10) to the end clip's on-axis saddle (Y ~ 82);
                                        # arm_wiring.py / nox/box-wiring.md (C; the old 12 / 15 counts missed the head servos)

# ================================================================== arm auto-centering detent (17 Oct 2026, Spark 04:16; next arm version)
# One detent at the pivot for both heads: ring fixed on the 80T, roller arm + springs on a tower drawn into hub v5. Geometry and the
# placement search live in detent.py (CHOSEN); HUB_DETENT puts the tower into hub_parts() for the hub v5 run.
HUB_DETENT = True

# ================================================================== arm harness v1 (17 Oct 2026, wiring chat; doc nox/box-wiring.md,
# numbers arm_wiring.py). One 19-wire bundle through the post bore; per-head servo ribbons folded inside the bars' TOP-face
# T-slots at X -10 (the bottom slots hold the bar posts' M5 slot nuts); stepper + hall branches drop between the bars and lace
# to the hub struts. Parts: wire_grommet (stub bore back), end_clip x2 (M5 into each bar's Ø4.2 core hole; B's carries the
# on-axis saddle), slot_clip x6.
WIRE_SADDLE_Y = (78.0, 86.0)         # end clip's on-axis bundle saddle: 6 over the fixed 80T collar top (Y 72), C
WIRE_DROP = dict(ang=135.0, r=32.0)  # drop lane between the bars: clears ring r 23.5, roller arm (-105), loop sector (-120), belts (Y >= 56), C
WIRE_SLOT = dict(face="top", x=-10.0, rib_t=1.3, rib_w=3.9, anchor_r=23.0, entry_du=8.0, Lw=80.6)   # fold-in-slot per head, C; glide
                                        # entry channel Ø4.5 at glide inner end + 8 = head v5 drawn feature

# ================================================================== stub v3 + demo bay (18 Oct 2026, arm chat; stand v5e, nox/box-stand.md)
# The stub bolts to ONE 2040 post (40 face toward +Y, slots at X +-10, face at Y 0): 4 x M5 at (X +-10, Z +-30), C (stand v5e).
# The holes sit at r 31.6, inside bearing 1 (Y 8..24, r 25..40): button heads (ISO 7380 M5, Ø9.5 x 2.75, E) sink flush with
# the plate top (Y 6), 2.0 below the bearing face. Placement and sizes from stub_v3_search() in arm_cad (min mass, C).
STUB_HOLES = ((10.0, 30.0), (-10.0, 30.0), (-10.0, -30.0), (10.0, -30.0))   # (X, Z) on the post's two slots, C
STUB_PLATE_T = 6.0                  # spider plate Y 0..6 (= Y_FLANGE1)
STUB_PAD_D = 14.4                   # pad Ø: counterbore Ø10.2 + 2 x 2.1 wall; >= Ø12 + 2.0 web (Problem 46), C
STUB_CB = (10.2, 2.75)              # counterbore Ø x depth for the button head: floor Y 3.25 (grip 3.25), head top flush at Y 6
STUB_HOLE_R = 2.75                  # M5 clearance Ø5.5
STUB_BORE_R = 20.0                  # shaft hollowed to Ø40 under both bearing seats (Y 6..40): wall 4.95 under the races
STUB_RIB_R = INSERT_R + INSERT_WALL # 3.6: the two sun-post insert ribs, full height inside the bore at Z +-21
STUB_SHOULDER_R = 26.4              # bearing 1 abutment Y 6..8 (was 27: the button heads at r >= 26.85 pass it by 0.45)
STUB_FILLET = 3.0                   # lobe-to-hub fillet in the plate outline (DEFAULT)
STUB_LEAD = dict(x_dir=-1, w=10.0)  # lead channel in the plate's back face toward -X, 10 wide (Z +-5) x 6 deep: the bundle (Ø7.3)
                                    # leaves the bore between the plate and the post face and drops beside the post (box-stand Problem 7)
WIRE_TWIST = dict(L=69.0, n_wires=19, bundle_d=7.3, travel_deg=421.0, neutral_phi=-45.0)   # stub v3: channel (Y ~3) to the saddle
                                    # (Y ~72 exit): L 90 -> 69; lay the bundle untwisted at phi -45 (cycle centre), C

DEMO_BAY = -90.0                    # the mounted bay (stand v5e cradle): head A picks the charged pack there, B returns the dead one.
                                    # BAY_ANGLES / SCOUT_BAYS keep all 9 module angles (the chained ring); arm_kinematics uses DEMO_BAY.
HOME_PHI = 0.0                      # homing stays at phi 0 (the magnet in detent ring v1, unchanged); cycle start = phi DEMO_BAY
CYCLE_PHI = (-180.0, 90.0)          # phi range the −90 cycle uses (arm_kinematics KEYS), inside the soft limits -278 / +143
