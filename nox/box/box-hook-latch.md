# Box hook latch v2 (24 Oct 2026; v1 23 / 24 Oct)

**Spark 23 Oct: "a hooked latch, not a steel belly, no magnets". 24 Oct: "a bit more error tolerance".** **v2 (24 Oct): built on the funnel panel v7 skin 2.0 (C; v1 was on the v6 skin 1.6 and overlaps a v7 panel by 242 mm³, M): guide top w −2.3, flange w −9.0..−13.0, plunger 32.95 long (C); everything else as v1. The v1 files stay in `print/` (do not print).** Holds the seated drone down against the XT60 insert push (`nox/dimensions.md` I "XT60 mate", 60 N E) and lets it go for takeoff. Replaces the electromagnet and the steel belly plate (retired). Numbers: `nox/dimensions.md` G "Hook latch". Script: `nox/box/hook_latch.py` (placement search, parts, checks, tolerance sweep, motion artifact). Run: `runs/hook_latch/v2/` (`result.json` holds every number below; `system.json` the catch system pass). Coordinates: seat frame as in `nox/box/catch_v4/funnel_panel.py`; unit frame u up the slope from the plunger axis, v along the face toward its centre line, w out of the face into the mouth.

## What it is
Four units, one behind each catch wheel on the two seated collar faces (the faces over the 45 / 135° bays = drone ±y; wheels at (±100, ±112) C, `dimensions.md` G Catch interface). Each is a **PETG plunger 8 (along the slope) x 12 (along the face edge) x 32.95 (C)** that slides out of the face, square to it, its downhill face **1.5 (C, DEFAULT) uphill of the seated R 5 wheel** (2.5 for R 4, C). Out, its top stands **12 (C, DEFAULT)** proud of the face. The XT60 push lifts the drone 2.12 (C, = 1.5 x √2) until each wheel meets a plunger's downhill face; a vertical lift also moves the wheel off the face, so the held wheel centre is 6.5 (C) off the face.
- **Self-locking:** the wheel loads the plunger across its travel, so no load can push it back in. The latch holds with the servos unpowered; the servo only moves the plunger.
- **Drive:** one MG90S-class micro servo per unit; an M2 screw in the horn hole at **r 13 (C, DEFAULT)** runs in a pocket in the plunger (scotch yoke): ±27.5° (C) of horn = 12 of stroke.
- **Out sensing:** a Ø3 x 2 magnet (E) in each plunger's valley-side face faces an A3144 hall switch (E) in a 1.0 (C) recess in the guide's valley-side face only when the plunger is fully out (2.2 C of PETG between them). The swap starts only when all 4 read out; a stalled plunger reads off.
- **Nothing new on either drone:** the wheels are the interface both drones already share. The drone keeps the plunger zone clear (Catch interface (7)).
- **Why not a swinging hook:** a hook pivoted at the rim or the face has to lift off the face to open; its tip sits 4.5 (C) outside the guard outline between the two cage rings, so it swings into the upper ring. A belly hook needs a drone feature and a post in the swap wheel's top opening. The plunger opens straight into the face.

## Error tolerance (24 Oct; `tolerance_sweep()` in the script, C)
The wheel may sit off its interface point by e along the slope (+ = uphill, toward the plunger: cage build, panel print and glue, seat scatter) and d sideways. Swept with the real plunger and an R 5 wheel, the plunger rising in 0.25 steps:
| Wheel error (C) | What happens (C) |
|---|---|
| e −2.5 .. +1.0 | plunger rises with ≥ 0.5 clearance; the drone lifts 0.71 .. 5.66 onto it; held wheel centre 5.5 .. 9.0 off the face, ≥ 1.0 under the top of the plunger's flat face (10.0) |
| e +1.0 .. +1.5 | rises touching or with under 0.5 clearance (held, no margin) |
| e > +1.5 | the plunger meets the wheel: a seated drone cannot move down the slope (the opposite face holds it), so the plunger stalls, its hall stays off, the swap aborts. The 2.0 lead-in on its downhill top edge only helps a drone resting above its seat |
| e < −2.5 | held too high on the plunger (−3.0: centre 0.5 under the flat face's top) |
| d ±4.5 | the whole 3 wide rim stays on the 12 wide plunger; partial contact to ±7.5 |
**Band −2.5 .. +1.0 (3.5 wide, C)** with 0.5 rise clearance; v1 as first drawn (8 x 8, out 7, gap 1.0) took e 0 .. +1 only, and review round 2 found the lead-in claim (to +3.5) wrong for a seated drone.

## Placement (search, `hook_latch.py search()`)
Plunger axis fixed by the wheel: slope position from the face's bottom edge = wheel contact 43.6 (C) + R 5 + gap 1.5 + half the plunger 4.0 = **54.1 (C)**, at x ±100 (C). Servo pose searched: horn hole radius 11.5 / 13 / 14.5 / 16 / 17.5 (C), servo shaft uphill or downhill of the plunger, servo body toward +u or −u (20 cases). Rules (C, DEFAULT): swing ≤ 30° each way; housing inside its panel half (valley plane + its 2.4 flange + 1.0); ≥ 4 below the rim; clear of the seat block; ≥ 3 off the ring (rho 183.6) and the side sheets (X ±186). Objective: housing + plunger mass. r 11.5 swings over 30°; a shaft downhill crosses the valley or the ring. Lightest: shaft uphill with the body running back over the guide, 27.52 g a unit (M) at r 13, rising 0.4 g per horn step. **r 13 taken.** Table: `result.json` `search`.

## Parts (PETG; `print/`; C unless tagged)
| Part | Qty | g (M) | What | Bed (M) |
|---|---|---|---|---|
| `box_hook_latch_housing_A_v2` | 2 | 23.7 | flange 57.5 x 49.8 x 4 on 4 panel bosses (M3), full width over the guide, from v −6 beside it; guide block 24.4 x 19.4 round an 8.4 x 12.4 bore (walls 8 across the load, 3 / 4 along it) from w −1.9 to −20.55; pin window in the servo-side wall; hall recess 4.4 x 3.4 x 1.0 in the valley-side face, centred 4.0 over the guide bottom (2.3 under it); two ear posts with M2 slots open toward the servo body, gussets into the flange, a tie bar under the servo | valley-side face (guide + flange edge), 440 mm², the largest of the 6; guide walls bend in-layer |
| `box_hook_latch_housing_B_v2` | 2 | 23.7 | A mirrored (the other half of each face) | the same face |
| `box_hook_latch_plunger_A_v2` (x2, A units) / `box_hook_latch_plunger_B_v2` (x2, B units, A mirrored: the pockets and top chamfers make it chiral) | 2 + 2 | 3.9 | 8 x 12 x 32.95; yoke pocket 3.97 x 5.5 deep x 2.3 from the servo side (walls 2.02); magnet pocket Ø3.1 x 2.2 in the valley-side face, centred 4.0 over the bottom (2.45 under it); 45° lead-in 2.0 on the downhill top edge, 0.8 chamfer on the uphill one | downhill (contact) face, 370 mm², smooth; the uphill face is larger but would put the rough top side on the contact |
Bought per unit: MG90S (E dims, D18); M2 x 12 as the yoke pin, threaded into the horn hole at r 13, CA; 2 x M2 x 12 + nuts through the ear posts (head on the post's far face, nut on the ear: the guide block blocks a driver from the ear side); 4 x M3 x 10 into the panel's heat-set inserts; Ø3 x 2 magnet (E) + A3144 (E), glued. Bolts at (u, v) (−18.5, 0), (18.5, 0), (−18.5, 34.6), (21.67, 34.6) (C): the valley row 18.5 off the axis so the Ø12 panel bosses clear the guide block (0.3), every axis 3.5 from any wall below the flange so the head and hex key reach all 4 (0 mm³). Set of 4: 110.3 g printed (M).

## Funnel panel v7 (as built by the funnel panel v7 worker, PR 37: skin 2.0, bosses w −2.0..−9.0)
On the 4 panels of the two seated faces (the ones over the 45 / 135° bays):
- **Skin window** 9.0 (along the slope) x 13.0 (C: plunger + 0.5 a side), centred at x ±100 (each panel's own wheel), 54.1 up the slope from the bottom edge.
- **4 bosses** Ø12 x 7 (C; pad rule) behind the skin (w −2.0..−9.0), M3 x 5.7 heat-set, hole Ø4.0 x 7 (wall 4.0), each with 2 gussets 2.0 thick (C) at v ±3 from its axis, running from the boss 6 along u away from the guide block (full boss height at the boss, 0.5 at the skin 6 out), at the bolt points above. Housing and plunger vs bosses + gussets: 0 mm³ (C, `bosses_local()` in the script).
- **Rib-free window** behind the skin over the housing footprint + 1.0: u −24.5..33.0 (slope 29.6..87.1 from the bottom edge), v −9.2..40.6 (lateral 59.4..109.2) (C); the v6 ribs there are 962 mm³ inside the housing (M). Reroute the strut under the wheel track round it; rerun `panel_opt.py` with it as a hole.
- The 3 panels already printed are v6: they go on the two unlatched faces.

## Checks (C unless tagged; `result.json`)
- **solid_check PASS** on all 3 files (1 body, 1 eroded body, 0 necks, 0 thin walls).
- **Moving parts, 0 mm³ overlap:** plunger vs housing at both ends of the stroke; horn + pin vs housing and vs the plunger every 2° of swing; servo body, dome and ears vs housing; bolt head + key cylinders vs housing and servo; housing and plunger vs the panel bosses and gussets.
- **Drone fit, 0 mm³:** extended plungers vs the R 5 wheels, Ø8 cheeks and both rings of the catch-interface drone (its corner posts lean 8 inward over their 16 rise, E; a post straight up from a wheel node crosses the plunger zone, Catch interface (7)); housings vs the drone.
- **Placement:** valley margin 5.3 (rule 3.4), highest point 87.1 up the slope (rim 107.3), seat block 28.1 away, ring 9.2 clear, side sheets 23.1 clear.
- **Fit with the v7 collar (C):** `nox/box/box_catch.assembly.json` (panels v7 on the seated faces, v7 plain on the others, latch v2) checks 0.0 mm³ for every pair with the plungers in and out (`assembly.py --check`); the PR 37 review found the same against `collar_v7_seat_frame.stl`.
- **Strength (hand calc, E loads):** functional = 30 N along the slope on one plunger (55 N net up, 67.5 % on the near pair with the XT60 push 35 off centre, rounded up), class flight (SF 4); misuse = 100 N on one plunger (a hand lifting the latched drone), SF 2.5. Load at the highest held wheel centre in the band, 9.0 off the face (C).

| Case | Plunger (in-layer) | Guide top reaction | Guide wall (in-layer as printed) | (if printed flange down: across layers) |
|---|---|---|---|---|
| functional 30 N | 2.65 MPa, index 0.24 | 48 N | 1.56 MPa, 0.14 | 0.29 |
| misuse 100 N | 8.83 MPa, index 0.51 | 161 N | 5.2 MPa, 0.30 | 0.60 |
No voxel FE (drawn part, hand calc only).


## System pass: the catch (`nox/box/catch_system.py`, CLAUDE.md "System first, then cut"; C unless tagged)
The seated drone (rigid, 4.9 N E, its 8 catch wheels on the faces), the collar, the 4 latches and the cartridge push as one assembly; supports unilateral (8 face normals, 4 plungers along the slope), equal stiffness (DEFAULT), least-norm non-negative reactions. The latch pieces take these joint reactions:
| System case (E loads) | Worst plunger (along the slope) | Worst face wheel | Plunger / guide index (class) |
|---|---|---|---|
| XT60 insert 60 N up, anywhere on the cartridge footprint | 28.3 N | 7.2 N | 0.23 / 0.14 (flight) |
| pull-out 50 N down | 1.7 N | 17.0 N | 0.01 / 0.01 (flight) |
| hand lift 100 N at a cage corner | 85.8 N | 46.3 N | 0.44 / 0.26 (misuse) |
| catch, 6 g floor landing on the faces | 0 | 6.1 N | 0 |
**Assembled worst 0.44 (hand lift; max over plunger, guide, panel FE and bosses)**, under the piece's own 100 N misuse check (0.51). Bolts into the panel bosses: ≤ 21.5 N shear each (hand lift). The panel takes the plunger reactions through its 4 bosses (funnel panel v7 as-built FE under these system reactions: util 0.10 / 0.08, boss deflection 0.31 / 0.63 mm in insert / hand lift, boss hand calc 0.07 / 0.13, C from `runs/funnel_panel/v7/result.json`).
- **Compact:** one latch unit (housing + servo + plunger out) 57.5 x 49.8 x 44.4 (C), 127.2 cm³ box, 28.7 cm³ solid; nested in the panel's rib field behind the face, nothing stacked.
- **Cage first:** the collar and the plungers meet only cage members (the catch wheels and their nodes). The catch-interface drone keeps its node cheeks 1.0 off the faces (G (1)) and everything else ≥ 19 off them (plate 30.6, props 19.0, receiver 25.2, C); the 3 mm rule for the cheeks is a shared-number question for the Scout v3 / Overseer chats.

## Concept check (docs/generative.md The rule, 0; 24 Oct)
Rule checks on the layout: `system_check` on `box_catch.assembly.json` passes latch out and in (fit 0.0 mm³ M, compact 8.89 L, extents 337 x 337 x 78 E); access: all 16 housing bolts reachable with the plunger in (C); cage first: only cage members (wheels, nodes) touch the collar and plungers.
- **Chosen: 4 face plungers behind the seated wheels.** Wins: nothing on the drone, self-locking under load with the servos off, fits the panel rib field (127 cm³ per unit); loses: 4 servos and 4 halls (110 g set M).
- **Alt 1, belly hook on a post:** 1 actuator instead of 4 (−3 servos, about −80 g E); loses: a keeper on both drones (about 5 g E each, Scout v3 and Overseer bellies) and a post standing in the swap opening the head must pass. Rejected.
- **Alt 2, rim swing hook:** a hook on the collar rim swinging over the upper cage ring; its tip arc r 75.6 about the pivot (166, 95.3) passes 5.8 from the upper-ring rod (112, 51), under rod r 1.5 + hook half-thickness 4 + clearance 0.5 = 6.0 (C): it clashes with the cage. Rejected.
- **Node cheeks 1.0 off the faces (G (1)): DEFAULT compliant**, they are cage members, so the 3 mm cage-first margin does not apply to them; alt: a 3 mm rule for cheeks via G (1), owned by the Scout v3 / Overseer chats.
Review agent in concept mode on this layout (PR 66 round 2); layout artifact: the v2 motion artifact.

## Sequence (box)
Drone seated (KW11) → 4 servos out (0.1 s E) → all 4 halls out, else retract and abort → swap: the XT60 push lifts the drone 2.1 (up to 5.7 at the band's edge, C) onto the plungers, so the drone's insert stroke ends up to 6 further (E, `dimensions.md` I) → head clear → 4 servos in → spin-up, take off. During the push the belly rises off the post: the pogo pads need ≥ 6 of travel (E) to stay made, and the box ignores the KW11 between latch-out and latch-in. A servo that fails out keeps the drone held (it cannot leave).

## Problems
1. **Wheel position is the interface's (rod centre on the outline, 4.0 off the face, C), not a measured seat.** First fit: seat the real drone in the printed collar, latch out (all 4 halls on), push up by hand: the drone lifts ≤ 6 before it is held. A cage oversize at the wheels by more than 1.0 stalls the plungers: move the panel v7 window uphill by the excess (one number, `GAP`).
2. **MG90S size and horn holes are E (D18).** No hole at r 13 on the horn: take the next one out (14.5: +0.4 g, swing 24°), change `servo_layout` r_h, rebuild.
3. **Load share is E:** 4 plungers under a 35 (E) off-centre push. If the drone tilts on the push, add the XT60 offset as a placement case.
4. **Cage keep-out:** the v3 cage and the Overseer belly frame keep 1.0 off each extended plunger (Catch interface (7)); no corner post straight up from a ±y-face wheel node.
5. **Servo and hall wiring:** 4 PWM + 4 hall inputs + 5 V from the box controller (box electronics worker).
6. **Pogo travel and KW11 during the lift:** the pogo pins (section M Q20) must take the 2.1 .. 5.7 lift with preload (≥ 6 travel, E); CHECK when the pogo pins are chosen.
