# Box hook latch v1 (23 / 24 Oct 2026)

**Spark 23 Oct: "a hooked latch, not a steel belly, no magnets". 24 Oct: "a bit more error tolerance".** Holds the seated drone down against the XT60 insert push (`nox/dimensions.md` I "XT60 mate", 60 N E) and lets it go for takeoff. Replaces the electromagnet and the steel belly plate (retired). Numbers: `nox/dimensions.md` G "Hook latch". Script: `nox/box/hook_latch.py` (placement search, parts, checks, tolerance sweep, motion artifact). Run: `runs/hook_latch/v1/` (`result.json` holds every number below). Coordinates: seat frame as in `nox/box/catch_v4/funnel_panel.py`; unit frame u up the slope from the plunger axis, v along the face toward its centre line, w out of the face into the mouth.

## What it is
Four units, one behind each catch wheel on the two seated collar faces (the faces over the 45 / 135° bays = drone ±y; wheels at (±100, ±112) C, `dimensions.md` G Catch interface). Each is a **PETG plunger 8 (along the slope) x 12 (along the face edge) x 31.55 (C)** that slides out of the face, square to it, its downhill face **1.0 (C, DEFAULT) uphill of the seated R 5 wheel** (2.0 for R 4, C). Out, its top stands **11 (C, DEFAULT)** proud of the face. The XT60 push lifts the drone 1.41 (C, = 1.0 x √2) until each wheel meets a plunger's downhill face; a vertical lift also moves the wheel off the face, so the held wheel centre is 6.0 (C) off the face.
- **Self-locking:** the wheel loads the plunger across its travel, so no load can push it back in. The latch holds with the servos unpowered; the servo only moves the plunger.
- **Drive:** one MG90S-class micro servo per unit; an M2 screw in the horn hole at **r 11.5 (C, DEFAULT)** runs in a pocket in the plunger (scotch yoke): ±28.6° (C) of horn = 11 of stroke.
- **Nothing new on either drone:** the wheels are the interface both drones already share. The drone keeps the plunger zone clear (Catch interface (7)).
- **Why not a swinging hook:** a hook pivoted at the rim or the face has to lift off the face to open; its tip sits 4.5 (C) outside the guard outline between the two cage rings, so it swings into the upper ring. A belly hook needs a drone feature and a post in the swap wheel's top opening. The plunger opens straight into the face.

## Error tolerance (24 Oct; `tolerance_sweep()` in the script, C)
The wheel may sit off its interface point by e along the slope (+ = uphill, toward the plunger: cage build, panel print and glue, seat scatter) and d sideways. Swept with the real plunger and an R 5 wheel, the plunger rising in 0.25 steps:
| Wheel error | What happens |
|---|---|
| e −2.0 .. +1.0 | plunger rises clear; the drone lifts 0 .. 4.24 (C) onto it; held wheel centre 5.0 .. 8.0 off the face, ≥ 1.0 under the top of the plunger's flat face (9.0) |
| e +1.0 .. +3.5 | first contact on the 2.0 (C, DEFAULT) 45° lead-in on the plunger's downhill top edge: it pushes the wheel down the slope. Works for a drone sitting off-centre; a cage built oversize at the wheels on both faces cannot move, so the servo stalls and the box aborts (Problems 1) |
| e < −2.0 | held too high on the plunger (−3.0: centre level with the flat face's top) |
| d ±4.5 | the whole 3 wide rim stays on the 12 wide plunger (C); partial contact to ±7.5 |
v1 as first drawn (8 x 8, out 7, no lead-in) held only e 0 .. +1 and the review found its held wheel centre (6.0) 1.0 under the plunger top.

## Placement (search, `hook_latch.py search()`)
Plunger axis fixed by the wheel: slope position from the face's bottom edge = wheel contact 43.6 (C) + R 5 + gap 1.0 + half the plunger 4.0 = **53.6 (C)**, at x ±100 (C). Servo pose searched: horn hole radius 10 / 11.5 / 13 / 14.5 / 16, servo shaft uphill or downhill of the plunger, servo body toward +u or −u (20 cases). Rules: swing ≤ 30° (DEFAULT) each way, housing inside its panel half (valley plane + its 2.4 flange + 1.0, C), ≥ 4 below the rim, clear of the seat block. Objective: housing + plunger mass. r 10 swings over 30°; a shaft downhill crosses the valley. Lightest: shaft uphill with the body running back over the guide, 26.82 g a unit (M) at r 11.5, rising 0.4 g per horn step. **r 11.5 taken (lightest; holes near the hub are on every stock horn, DEFAULT).** Table: `result.json` `search`.

## Parts (PETG; `print/`; C unless tagged)
| Part | Qty | g (M) | What | Bed (M) |
|---|---|---|---|---|
| `box_hook_latch_housing_A_v1` | 2 | 23.1 | flange 53.25 x 49.8 x 4 on 4 panel bosses (M3); guide block 24.4 x 19.4 round an 8.4 x 12.4 bore (walls 8 across the load, 3 / 4 along it) from w −1.9 to −20.55; pin window in the servo-side wall; two ear posts with M2 slots open toward the servo body, gussets into the flange, a tie bar under the servo | valley-side face (guide + flange edge), 570 mm², the largest of the 6 (M); guide walls bend in-layer |
| `box_hook_latch_housing_B_v1` | 2 | 23.1 | A mirrored (the other half of each face) | the same face |
| `box_hook_latch_plunger_v1` | 4 | 3.8 | 8 x 12 x 31.55; yoke pocket 4.0 x 5.5 deep x 2.3 from the servo side (walls ≥ 2.0, floor 6.5); 45° lead-in 2.0 on the downhill top edge, 0.8 chamfer on the uphill one | downhill (contact) face, 354 mm² (M), smooth; the uphill face is 368 mm² (M) but would put the rough top side on the contact |
Bought per unit: MG90S (E dims, D18); M2 x 12 as the yoke pin, threaded into the horn hole at r 11.5, CA; 2 x M2 x 12 + nuts through the ear posts (head on the post's far face, nut on the ear: the guide block blocks a driver from the ear side); 4 x M3 x 10 into the panel's heat-set inserts. Bolts at (u, v) (−15.7, −3.2), (15.7, −3.2), (−15.7, 34.6), (20.2, 34.6) (C), each 3.5 (C) from every wall below the flange: head and hex key reach all 4 (0 mm³, C). Set of 4: 107.3 g printed (M).

## Funnel panel v7 (funnel chat; the panels are not edited here)
On the 4 panels of the two seated faces (the ones over the 45 / 135° bays):
- **Skin window** 9.0 (along the slope) x 13.0 (C: plunger + 0.5 a side), centred at x ±100 (each panel's own wheel), 53.6 up the slope from the bottom edge.
- **4 bosses** Ø12 x 7 (C; pad rule) behind the skin (w −1.6..−8.6), M3 x 5.7 heat-set, hole Ø4.0 x 7 (wall 4.0), each with 2 gussets into the skin, at the bolt points above.
- **Rib-free window** behind the skin over the housing footprint + 1.0: u −21.7..31.55 (slope 31.9..85.1 from the bottom edge), v −9.2..40.6 (lateral 59.4..109.2) (C); the v6 ribs there are 879 mm³ inside the housing (M). Reroute the strut under the wheel track round it; rerun `panel_opt.py` with it as a hole.
- The 3 panels already printed are v6: the latch goes on v7 panels only.

## Checks (C unless tagged; `result.json`)
- **solid_check PASS** on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls).
- **Moving parts, 0 mm³ overlap:** plunger vs housing at both ends of the stroke; horn + pin vs housing and vs the plunger every 2° of swing; servo body, dome and ears vs housing; bolt head + key cylinders vs housing and servo.
- **Drone fit, 0 mm³:** extended plungers vs the R 5 wheels, Ø8 cheeks and both rings of the catch-interface drone (its corner posts lean 8 inward over their 16 rise, E; a post straight up from a wheel node crosses the plunger zone, Catch interface (7)); housings vs the drone.
- **Placement:** valley margin 3.44 (rule 3.4), highest point 85.1 up the slope (rim 107.3), seat block 29.5 away.
- **Strength (hand calc, E loads):** functional = 30 N along the slope on one plunger (55 N net up, 67.5 % on the near pair with the XT60 push 35 off centre, rounded up), class flight (SF 4); misuse = 100 N on one plunger (a hand lifting the latched drone), SF 2.5. Load at the highest held wheel centre in the band, 8.0 off the face (C).

| Case | Plunger (in-layer) | Guide top reaction | Guide wall (in-layer as printed) | (if printed flange down: across layers) |
|---|---|---|---|---|
| functional 30 N | 2.3 MPa, index 0.21 | 46 N | 1.5 MPa, 0.14 | 0.27 |
| misuse 100 N | 7.7 MPa, index 0.45 | 153 N | 5.0 MPa, 0.29 | 0.57 |
No voxel FE (drawn part, hand calc only).

## Sequence (box)
Drone seated (KW11) → 4 servos out (0.1 s E) → swap: the XT60 push lifts the drone 1.4 (up to 4.2 at the band's edge, C) onto the plungers, so the drone's insert stroke ends up to 5 further (E, `dimensions.md` I) → head clear → 4 servos in → spin-up, take off. During the push the belly rises off the post: the pogo pads need ≥ 5 of travel (E) to stay made, and the box ignores the KW11 between latch-out and latch-in. A servo that fails out keeps the drone held (it cannot leave). A servo that fails in, or stalls on the lead-in, leaves that wheel free: the swap aborts unless all 4 reach "out" (DEFAULT: commanded position only; a switch per plunger is the alternative).

## Problems
1. **Wheel position is the interface's (rod centre on the outline, 4.0 off the face, C), not a measured seat.** First fit: seat the real drone in the printed collar and push up by hand. The drone must lift ≤ 4 before the plungers hold it, and every plunger must rise without touching a wheel. A cage oversize at the wheels by more than 1.0 stalls the plungers: move the panel v7 window uphill by the excess (one number, `GAP`).
2. **MG90S size and horn holes are E (D18).** No hole at r 11.5 on the horn: take the next one (13: +0.4 g, swing 25°), change `servo_layout` r_h, rebuild.
3. **Load share is E:** 4 plungers under a 35 (E) off-centre push. If the drone tilts on the push, add the XT60 offset as a placement case.
4. **Cage keep-out:** the v3 cage and the Overseer belly frame keep 1.0 off each extended plunger (Catch interface (7)); no corner post straight up from a ±y-face wheel node.
5. **Servo wiring:** 4 PWM + 5 V from the box controller (DEFAULT: a PCA9685 on the box Pi's I2C, CHECK the controller's spare pins first); not on the buy list until the box electronics are drawn.
6. **Pogo travel and KW11 during the lift:** the pogo pins (section M Q20) must take the 1.4 .. 4.2 lift with preload; CHECK when the pogo pins are chosen.
