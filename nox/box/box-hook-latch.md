# Box hook latch v1 (23 Oct 2026)

**Spark 23 Oct: "a hooked latch, not a steel belly, no magnets".** Holds the seated drone down against the XT60 insert push (`nox/dimensions.md` I "XT60 mate", 60 N E) and lets it go for takeoff. Replaces the electromagnet and the steel belly plate (retired). Numbers: `nox/dimensions.md` G "Hook latch". Script: `nox/box/hook_latch.py` (placement search, parts, checks, motion artifact). Run: `runs/hook_latch/v1/`. Coordinates: seat frame as in `nox/box/catch_v4/funnel_panel.py`.

## What it is
Four identical units, one behind each catch wheel on the two seated collar faces (the faces over the 45 / 135° bays = drone ±y; wheels at (±100, ±112), `dimensions.md` G Catch interface). Each is a **square PETG plunger** (8 x 8 x 27.6) that slides out of the face, square to it, **1.0 uphill of the seated wheel** (R 5; 2.0 for R 4). Out, its top stands 7 proud of the face. The XT60 push lifts the drone 1.4 (E), each wheel rolls up-slope into a plunger's downhill face and stops there.
- **Self-locking:** the wheel loads the plunger across its travel, so no load can push it back in. The latch holds with the servos unpowered; the servo only moves the plunger.
- **Drive:** one MG90S-class micro servo per unit. A pin (M2 screw in the horn hole at r 10) runs in a pocket in the plunger (scotch yoke): ±20.5° of horn = 7 of stroke.
- **Nothing new on either drone:** the wheels are the interface both drones already share. No belly plate, no magnet power, no drone part changes. The drone keeps the plunger zone clear (Catch interface (7)).
- **Why not a swinging hook:** a hook pivoted at the rim or the face has to lift off the face to open, and its tip sits 4.5 outside the guard outline between the two cage rings, so it swings into the upper ring (worked in the 23 Oct box chat). A belly hook needs a drone feature and a post in the swap wheel's top opening. The plunger opens straight into the face.

## Placement (search, `hook_latch.py search()`)
Plunger axis fixed by the wheel: slope position from the face's bottom edge = wheel contact 43.6 + R 5 + gap 1.0 + half the plunger 4.0 = **53.6 (C)**, at x ±100. The servo pose was searched: horn hole radius 7 / 8.5 / 10 / 11.5 / 13, servo shaft uphill or downhill of the plunger, servo body toward +u or −u. Rules: swing ≤ 30° each way, the housing inside its panel half (valley plane + its 2.4 flange + 1.0), 4 or more below the rim, clear of the seat block. Objective: housing mass. 20 combinations: r 7 swings 30° or more; a shaft downhill with the body uphill crosses the valley. The lightest housings (18.38 g a unit with the plunger, M) tie for r 8.5 / 10 / 11.5 with the shaft uphill and the body running back over the guide. **r 10 taken (DEFAULT: the hole most stock horns have; 20.5° swing, 0.63 pin drift).** Table: `runs/hook_latch/v1/result.json` `search`.

## Parts (PETG; `print/`)
| Part | Qty | g (M) | What | Bed |
|---|---|---|---|---|
| `box_hook_latch_housing_A_v1` | 2 | 17.3 | flange 48.5 x 37.3 x 4 on 4 panel bosses (M3), guide block 24.4 x 16.4 round an 8.4 square bore (walls 8 across the load, 4 along it) from w −1.9 to −20.55, pin window in the servo-side wall, two ear posts + a tie bar under the servo | guide-side face (511 mm², the largest of the 6, M) |
| `box_hook_latch_housing_B_v1` | 2 | 17.3 | A mirrored (the other half of each face) | the same face |
| `box_hook_latch_plunger_v1` | 4 | 2.2 | 8 x 8 x 27.55, yoke pocket 3.4 x 2.3 x 6.5 from the servo side, 0.8 chamfer on the uphill top edge | downhill (contact) face: smooth (220 mm², M; brim 5) |
Bought per unit: MG90S (E dims, D18), M2 x 12 as the yoke pin (threaded into the horn hole, CA), 2 x M2 x 12 + nuts through the ear posts and ears, 4 x M3 x 10 into the panel's heat-set inserts. Set of 4: 77.9 g printed (M).

## Funnel panel v7 (funnel chat; the panels are not edited here)
On the 4 panels of the two seated faces (the ones over the 45 / 135° bays):
- **Skin window** 9.0 x 9.0 (plunger 8.0 + 0.5 a side), centred at x ±100 (each panel's own wheel), 53.6 up the slope from the bottom edge (C).
- **4 bosses** Ø10 x 7 behind the skin (w −1.6..−8.6), M3 x 5.7 heat-set, hole Ø4.0 x 7, at the housing's bolt points (unit frame u / v: (−13.75, −2.2), (−13.75, 23.1), (22.75, −2.2), (22.75, 23.1), C; u up the slope from the plunger axis, v toward the face centre line).
- **Rib-free window** behind the skin over the housing footprint + 1.0: unit frame u −19.75..28.75 (slope 33.8..82.4 from the bottom edge), v −8.2..29.1 (lateral 70.9..108.2), the v6 ribs there are 433 mm³ inside the housing (M). Reroute the strut under the wheel track round it; rerun `panel_opt.py` with it as a hole.
- The 3 panels already printed are v6: the latch goes on v7 panels only.

## Checks (C unless tagged; `result.json`)
- **solid_check PASS** on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls).
- **Moving parts, 0 mm³ overlap:** plunger vs housing at both ends of the stroke; horn + pin vs housing and vs the plunger every 2° of swing; servo body, dome and ears vs housing.
- **Drone fit, 0 mm³:** extended plungers vs the R 5 wheels, cheeks and both rings of the catch-interface drone; housings vs the drone. Gap to the wheel 1.0 (R 5) / 2.0 (R 4); the drone rises 1.4 / 2.8 before it is held (E).
- **Placement:** valley margin 6.4, highest point 82.3 up the slope (rim 107.3), seat block 42 away (C).
- **Strength (hand calc, E loads):** functional = 30 N along the slope on one plunger (55 N net up, 67.5 % on the near pair with the XT60 push 35 off centre, rounded up), class flight (SF 4); misuse = 100 N on one plunger (a hand lifting the latched drone), SF 2.5. Load on the wheel's equator, 5 off the face.

| Case | Plunger (in-layer) | Guide top reaction | Guide wall (in-layer as printed) | (if printed flange down: across layers) |
|---|---|---|---|---|
| functional 30 N | 2.4 MPa, index 0.22 | 41 N | 1.6 MPa, 0.15 | 0.29 |
| misuse 100 N | 8.1 MPa, index 0.47 | 137 N | 5.3 MPa, 0.30 | 0.61 |
No voxel FE (drawn part, hand calc only).

## Sequence (box)
Drone seated (KW11) → 4 servos out (0.1 s E) → swap (the XT60 push lifts the drone 1.4 onto the plungers; the head's last 12 run slow, `dimensions.md` I) → head clear → 4 servos in → spin-up, take off. A servo that fails out keeps the drone held (it cannot leave). The box checks with the KW11 that the drone is still seated before it retracts. A servo that fails in leaves the drone unheld: the swap aborts if the 4 servo feedbacks are not out (DEFAULT: the box reads each servo's commanded position only; a switch per plunger is the alternative).

## Problems
1. **Wheel position is the interface's (rod centre on the outline, 4.0 off the face, C), not a measured seat.** First fit: seat the real drone in the printed collar, push up by hand: the wheel must meet the plunger within 1 to 2 (E). If the gap is wrong, the panel v7 window moves (one number, `GAP`).
2. **MG90S size and horn holes are E (D18).** A horn without a hole at r 10: use r 8.5 or 11.5, change `servo_layout` r_h and rebuild (same mass).
3. **Load share is E:** 4 plungers under a 35 off-centre push. If the drone tilts on the push, add the XT60 offset as a placement case (the push near the lead end).
4. **Cage keep-out:** the v3 cage and the Overseer belly frame keep 1.0 off each extended plunger (Catch interface (7)); the catch-interface drone here is the 3.0 rod rings + R 5 wheels + Ø8 cheeks (E).
5. **Servo wiring:** 4 PWM + 5 V from the box controller (DEFAULT: a PCA9685 on the box Pi's I2C, CHECK the controller's spare pins first); not on the buy list until the box electronics are drawn.
