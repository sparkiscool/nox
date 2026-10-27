# X-axis flip swap v4: latch under the pack (27 Oct 2026)

Spark's review of v3 (via Alex, 26 Oct): deeper finger tip and slot in the latch; bigger latch; small finger stroke, big
latch travel; tolerance on every contact; pincer narrower than the battery, latch under the pack, not beside it;
endurance over thousands of uses; simplest solution. Scope: latching and unlatching only. Research first:
`claude/research/2026-10-27-xswap-latch-v4.md`. Model `xswap4.py`, checks `xswap4_check.py`, prints and coupons
`xswap4_export.py`, figures `xswap4_render.py`, page `xswap4_webdata.py` + `page4_template.html`. Results `runs/xswap/v4/`.
v3 (`xswap.py`, `README.md`) is unchanged and still builds.

## The latch: a gate bar at each end
- A tray under the pack floor holds two **gate bars** (one print, used twice). Each bar is a rigid PETG lever on a vertical
  **Ø4 x 16 steel dowel** (press fit in the shell bottom, blind hole above). A **steel spring** (6 OD, ~21 free, ~0.8 N/mm,
  E) holds it latched.
- The bar's **tip** (10 wide, 6.7 tall) comes out of the shell's end face and sits **4.2 mm deep** in a pocket in a
  **holder end leg**, on a **12 mm ledge**. The tip's load face is flat on the ledge: a pull on the pack presses it down
  harder, and friction under that load (~150 N at 150 g) is ~50x the spring, so a loaded latch can't be opened at all.
- **Lever ratio 4.8:** the finger post pushes the bar 12 mm from the pivot; the tip is 58 mm out. The tip must swing
  10.7 mm to clear the ledge; the finger moves 3.8 mm in all (0.8 clearance + 0.5 allowed pincer error + 2.2 drive),
  which swings the tip 11.9 to 16.8 mm depending on the pincer's x error.
- The two bars release in opposite directions (point symmetry about the shell's middle), so a side hit that flicks
  one open pushes the other shut.
- **Load path in a crash:** pack -> floor -> shell side walls -> 16 mm end band over the tip -> tip -> ledge -> leg ->
  deck -> 2 M3 inserts over each leg -> drone.

## The pincer
- Jaw 44 wide (battery 47), under the pack. Servo, pinion and racks as before, turned so the fingers move across the pack.
- Each finger is a **round Ø7 post** that goes up through a keyhole in the shell bottom and **7.7 mm into a slot in the
  bar** (0.8 clearance a side, 45 deg chamfered mouth). A flange plate on the post slides under the keyhole's lips as
  it moves, so the pack hangs on the pincer (XT60 pull, flip) by geometry, not friction.
- The flip shafts go in blind bores in the jaw's sides (axis z -110); the carriage fork sits outside them.

## Contacts (never cubes on cubes)
Round posts in obround slots; 45 deg chamfers on every slot and keyhole mouth; tip end chamfered 1.0; 30 deg x 6 lead-in
on the legs' inner bottom edges; 3 mm flared mouths on the release channels; bullet guide posts lengthened to lead the
tips (touch 37 to 78 mm before seat).

## Checks (`runs/xswap/v4/check.txt`, all C unless E)
- 0 mm³: seat; bars over their whole swing; insert / pull sweep with the bars released (least and largest swing); pincer
  approach; posts in the bar slots over the stroke at pincer x error -0.5 / 0 / +0.5; pinion and racks; pincer vs
  holder; flip vs drone and bay holders; bay insert and seat. Both bars pass release at either pincer error.
- Latched shell pulled down 0.6 mm collides (ledges). Shut fingers pulled down catch the lips (21 mm³); open fingers don't.
- Capture: 13 of 13 offset / tilt cases (±3, ±5 mm, diagonals, 2 deg) touch a slope before any flat face.
- Connectivity PASS on all 8 print files and both coupons (`xswap_solid.py`).
- Crash, 150 g on the 694 g swap mass = 1022 N, 511 N per tip (E): tip shear 7.6 / bending 16.4 MPa, tip bearing 12.2,
  shell end band 18.1, holder ledge 10.6 (real section with the lead-in), leg tension 2.0 across layers, 256 N per insert.
  Printed bending is in-layer (PETG ~45 MPa). The spring is steel; no printed part flexes.
- Servo: 15 N per finger at full swing, 0.41 N·m.

## Prints (PETG, A1 mini, tree supports)
| File | Count | Size (mm) | g solid |
|---|---|---|---|
| xswap_shell_v4 | 1 per pack, tail end down | 99 x 70 x 165 | 179 |
| xswap_bar_v4 | 2 per pack, flat | 15 x 66 x 12 | 6.5 |
| xswap_holder_v4 | 1 per drone + 1 per bay, mount face down | 99 x 179 x 100 | 212 |
| xswap_finger_v4 | 2, rack bottom down | 30 x 34 x 28 | 5.4 |
| xswap_jaw_v4 | 1 | 44 x 90 x 58 | 173 |
| xswap_pinion_v4, xswap_xt60_cap_v4, xswap_xt60_keeper_v4 | as v3 | | |
| coupon_shell_end_v4, coupon_holder_leg_v4 | strength coupons: one latch end + its leg | | 22 / 57 |

Coupon test (next week): print both coupons + one bar + dowel + spring; latch it, hang weights from the shell coupon
(target 511 N per tip at 150 g), cycle the bar 1000+ times with a finger, measure finger force and tip swing.

## Bought per pack / holder / pincer
Per pack: 2 x Ø4 x 16 steel dowel, 2 x springs (6 OD, ~21 free, ~0.8 N/mm), plus v3's tail bolts and XT60 keeper.
Per holder: v3's + 4 M3 inserts over the legs (8 in all with the 4 corners). Pincer: servo, 2 x 8 mm shafts, set screws.

## What moved (cost)
Shell +14.8 mm taller (70.3); holder legs reach 90 below the deck; holder same length (178.6). Flip axis now in the jaw:
sweep radius 138 (v3 95), rail travel 80 + 80 (v3 85 + 75). Out of scope here: moving the flip axis back up.
