# Box collar funnel v4 / panels v7 + v7 plain (catcher), shared by the Scout and the Overseer

The box's catcher: a **static all-PETG collar** of 4 flat 45° faces on top of the box with an **open bottom** (Spark 16 Oct 19:02 / 19:33). The drone lands into it under its own control; the 8 print-in-place PETG wheels in the guard's corner nodes roll down the bare faces and centre it; the pack hangs through the open bottom to the swap wheel's top head; on the two seated faces the hook latch (`nox/box/box-hook-latch.md`) pushes 4 plungers out through the panels to hold the drone for the swap. Nothing else on the funnel moves. Numbers: `nox/dimensions.md` G (the only source; this page explains them). Guard side: `archive/scout-v2/s2-guard.md`. Box system: `nox/box/box.md`. Scripts: `nox/box/catch_v4/` (`funnel_panel.py` v6, `funnel_panel_v7.py` v7, `panel_opt.py` / `panel_opt_v7.py` rib optimizers, `rib_mask.json` / `rib_mask_v7.json`); the catch sims are in `archive/scout-v2/catch_v4/`. Seat frame = drone coordinates of a seated Scout. C / M / E tags; DEFAULT unless marked Spark.

## Brief (Spark)
- One funnel for both drones; high error tolerance; nothing can stick; printed on the A1 mini, glued.
- 16 Oct: compact; open bottom so both drones' packs pass; Overseer on its own contacts on the same faces.
- 16 Oct 18:37: it must always centre. 19:02: contacts in PETG built into the nodes. 19:33: optimize the nodes, tweak the funnel a reasonable amount, everything PETG.
- 16 Oct 21:04: make it on the A1 minis, glued together like a puzzle; Claude picks the split.
- 16 Oct 21:19: **flat prints only** (support and tall prints cost too much time); Spark sands the surfaces / seams flush.
- 16 Oct 22:06: optimize the panel.
- 16 Oct 22:15: the collar sits on the bay ring, not a box top. 17 Oct 00:16: **mount it straight to the top of the two charging bays either side of the drone position, with M3 screws**. 00:55: **screw it from above** (from below was blind).

## Why this works
1. **Only the 8 node wheels touch:** 2 per face, 200 apart; rods and nodes stay ≥ 1.0 off.
2. **Rolling beats friction.** Fixed contacts jam once μ > 0.20 (two faces carry the weight and hold the slide in the other direction); bare PETG is ~0.3 to 0.4 (E). Wheels rolling down the slope centre 100 % at μ 0.4 at 45° (MuJoCo, 204 drops: ±40 offset, ±5° yaw, 2° tilt, 0.15 m/s drift in and out; also 100 % with the pin at μ 0.7).
3. **45° is the reasonable tweak:** same mouth 320 and opening 168, 76 tall instead of 53. 35° and 40° miss only the 40-straight-off-with-outward-drift drops (92 / 95 % at μ 0.4); 45° catches all of ±40.
4. **Full flight physics** (`catch_v4/s2_catch_real.py`, viewer artifact "Scout Collar Catch"): with real inertia, a flown approach, prop spin-down and bouncy contacts, every drop inside a ±30 gate centres (300 of 300). At 40 off drifting outward the drone reaches the face 46 to 61 off, past the mouth, and hangs; a taller mouth only reaches 97 %. Best fix: the landing mode commits only inside ±30 (open, Spark).
5. **Faces reach down to a 168 opening:** at 40 off the far side still lands on a face.
6. **Compact:** 76 tall, 337 square outside, 543 g (C) for all 8 panels.

## Geometry (seat frame, C)
- Faces: 45°, bottom edge z 1.3 (168 square), top z 77.2 (320 square).
- Below the bottom edge: open; 2 pogo pads and the KW11 sit on a post under the belly (box chat, DEFAULT). Hold: the hook latch on the two seated faces (Spark 23 Oct; the electromagnet is retired).
- Camera keep-out: the face reaches x 98 at z 15.3.
- **Mount (v6, v7 the same):** seat frame to W per `dimensions.md` G "Funnel": collar centred on the tab line W Y = Y_C 175 this round (lead DEFAULT 24 Oct), seat z 0 = W Z 144.5 (E); W X tangential. The 2 faces over the 45° and 135° bay modules carry the 4 screws; the other 4 holes are spares (any 90° rotation fits). 3D view: artifact "Nox Collar Mount".

## Split for the A1 mini (16 Oct 21:30, `catch_v4/funnel_panel.py`)
Flat-only rules it: a whole face (320 wide) and any corner piece (78 tall, upright) are out, so each face is two flat half-face panels, **8 panels = 4 x A + 4 x B** (B = A mirrored). Each prints **contact face down**: the face the wheels roll on comes off the plate smooth.
- **Centre seam** (face centre line, plane y = 0, out of the wheel path at 60 to 140): back flanges glued back to back; A carries 2 teardrop key pegs (Ø4 x 2.2, slant 30 and 80), B the through-holes (Ø4.3). Glue each face's A + B face-down on a flat table: the table makes the face flush, the keys lock it in plane.
- **Valley seam** (plane x = y): both skins end on the valley plane and their back flanges glue back to back. Jig: stand the 4 faces mouth-down on the table, so all top edges sit in one plane while the glue cures. The valley is in the wheel path: sand it flush (Spark).

## Rib optimization (16 Oct 22:40, `catch_v4/panel_opt.py`, result `catch_v4/rib_mask.json`)
Mindlin plate FE of one full face (2 mm quads, 6 596 el). Supports: bottom edge (foot) and both valleys (fold lines), top edge free; the glued seam flange pair is passive. Design = skin ts + a symmetric rib field of height h; minimum mass.
- **Loads (DEFAULT):** functional = 20 N presses (r 5) on the wheel track (u 60 to 140; Overseer contacts assumed in it) + 20 N uniform: ≤ 1.0 mm. Misuse = 20 N presses off the track and on the free top edge: ≤ 3.0 mm. Every case: plate stress ≤ 19.5 MPa (PETG XY 39 / SF 2; printed flat, so bending stress is in-layer).
- **Why it was needed:** the 21:30 panel (skin 2.8 + top band) fails this spec at 3.6x: 3.6 mm at u 60 v 90. The free top edge spans the full 320 between valleys, not 160 as the hand calc assumed.
- **Sweep (g per face, FE):** ts 1.2: h 6 168, h 9 176 (stress-limited); ts 1.6: h 6 159, **h 9 123**; ts 2.0: h 6 155, h 9 132. Skin alone fails at every ts (16 to 60 mm).
- **Layout:** a deep rim beam across the mouth, one strut per panel from the rim to the foot under the wheel track, diagonal braces to the valleys.
- **As built:** rib field smoothed (Gaussian 0.9 el, 0.8 opening, crumbs < 60 mm² dropped) then grown 1.0 mm, the least grow that passes on the smoothed outline: 0.98 utilisation at u 60 v 90, misuse ≤ 2.3 mm, 10.8 MPa max (FE).

## Part: funnel panel A / B v6 (PETG, 4 + 4)
| Feature | Value | Source |
|---|---|---|
| Skin | 1.6 | rib optimizer |
| Ribs | 9 tall behind the skin (10.6 total), smoothed outline from the optimizer, flat tops | rib optimizer |
| Seam + valley flanges | 2.4 thick, 12 behind the face, full height | v3 values |
| Seat block (v6, replaces the v5 pad) | one per panel behind the skin at **36 from the face centre line, 14 up the slope** (screw entry (93.9, ±36, 11.2) seat frame): flat landing disc Ø16 at 6 behind the face (parallel to it), flaring to Ø26 at the skin's back; no ribs within r 9.5 behind the landing (post room). Skin keeps the 2.5 lip under the bottom edge (z -1.2) | C, DEFAULT |
| Mount (v6) | **M3 x 12 countersunk (ISO 10642) from above, square to the face**, straight 2.0 hex from inside the funnel: Ø3.4 through the block, 90° countersink Ø6.6 (head sits 0.2 under the face), 6 of PETG clamped, 6 of thread into an **M3 x 5.7 heat-set insert in the bay post** (hole Ø4.0 x 8 along the screw axis, post top cut at 45° to seat on the landing). The heads are 36 from the centre line; the wheels run 50..150 from it (±100, ±40 offset, yaw), so no wheel crosses a head. Screw axis = print z: the countersink is a 45° cone over the bed, the landing a flat top | C, DEFAULT |
| Mass | A 68.0 g, B 67.8 g, all 8: 543 g | C |
| Bed | 108.8 x 170.6 x 12.0, contact face down (13 052 mm² on the plate), no supports (14 mm² over 45°: key undersides; the countersink cone is at 45°). Sliced 0.28 Extra Draft, 3 walls, 100 % rectilinear: A 62.7 g, 1 h 38 m | C / slicer |
| Checks | each panel watertight, 1 body; 8 panels assembled: 0.00 mm³ overlap, 1 body | C |

Build: print 4 A + 4 B. Scuff the flanges. Glue each A + B pair face-down on a flat table (4 faces). Stand the 4 faces mouth-down, glue the valley flanges, clip, cure. Sand the valley seams flush. Press an M3 heat-set insert into each of the 4 bay posts' 45° seats. Set the collar on the posts (landings on the seats), 4 x M3 x 12 countersunk from inside the funnel.

## Part: funnel panel A / B v7 (PETG, 2 + 2; the two seated faces, 24 Oct)
The 4 panels of the faces over the 45 / 135° bays (drone ±y) carry the hook latch; the other 4 are **v7 plain** (below). Script `catch_v4/funnel_panel_v7.py` (builds on `funnel_panel.py`, which stays the v6 source), rib run `catch_v4/panel_opt_v7.py`, results `runs/funnel_panel/v7/` (`result.json` holds every number below). C unless tagged.
| Feature | Value | Source |
|---|---|---|
| Unchanged from v6 | faces, 45°, 168 / 320, centre seam + valley flanges (2.4 x 12), keys, seat block (36 from the centre line, 14 up the slope), M3 x 12 countersunk from above into the bay post insert | v6 |
| Skin | **2.0** (v6 1.6 reads thin walls in solid_check: the 2.0 floor) | DEFAULT |
| Skin window | 9.0 along the slope x 13.0 along the edge, centred at its wheel, lateral 100, 54.1 up the slope: plunger 8 x 12 + 0.5 a side (checked: the plunger section grown 0.45 a side clears, 0.55 touches) | hook latch spec |
| Latch bosses | 4 per panel, Ø12 x 7 behind the skin (w −2.0 .. −9.0), M3 x 5.7 heat-set, hole Ø4.0 x 6.5 (2.5 floor), 2 gussets 2.0 thick each: exactly `hook_latch.bosses_local()` at the 2.0 skin. The downhill valley-side boss's gussets ran 1.3 past the valley plane: trimmed there, they end in the valley flange. Housing flange face lands at w −9.0 | hook latch spec |
| Ribs | 9 behind the skin; **low ribs 6.7 over the housing footprint + 1.0** (unit frame u −25.5 .. 34.0, v −10.2 .. 41.6), 0.3 under the housing flange; **none over the guide block + 1.0** (u ±13.2, v −10.2 .. 11.2). The spec's rib-free window left the bosses and the strut under the wheel track as islands on the skin (solid_check necks) | DEFAULT |
| Rib optimization | `panel_opt_v7.py`: v6 plate model, cases and limits + the window as a hole, bosses as 9.0 plate, 2 presses beside the window (u 100, v 40 / 65), latch functional 30 N on both plungers (≤ 1.0) and misuse 100 N on one (≤ 3.0): the housing as a rigid body, F x (9.0 + 9.0) as normal boss forces (7.6 N / 25.2 N max, conservative). ts 2.0, h 9 fixed; lightest feasible rib fraction 0.20, 134.5 g a face (plate model) | C (FE) |
| Drawn tie ribs | 4.4 wide (survive every solid_check erosion): boss to boss along the slope and on both sides of the guide, the uphill bosses up into the rim beam, the downhill one into the strut under the wheel track, the seat block into that strut (it sat on skin only in v6), the rim corner piece into the valley diagonal | DEFAULT |
| As built (FE) | outline smoothed as v6, grown 1.25 (the least that passes): worst 0.893 of the 1.0 limit at press u 60 v 90, 6.6 MPa max (limit 19.5), boss deflection 0.32 (functional) / 0.72 (misuse) | C (FE) |
| Bosses in plane (hand calc) | misuse: 25 N shear per boss at the flange face + 25.2 N normal: 1.29 MPa at the boss root, across layers (boss axis = print z), index 0.15 (misuse SF 2.5); functional 0.07; skin membrane 1.0 MPa | C / E |
| Mass | A 80.8 g, B 80.7 g (M, solid); collar 4 x v7 + 4 x v7 plain: 624.3 g | M |
| Bed | 109.0 x 170.6 x 12.0, contact face down (12 935 mm² on the plate), 14 mm² over 45° (key undersides; countersink cone at 45°), no supports | M |
| Checks | solid_check PASS on A and B; collar of 4 v7 + 4 v7 plain: 0.000 mm³ between panels, 1 body; wheel track (lateral 50 .. 150, every face): 0 steps over 0.2, face depth 0.000 off the windows | C |

**Hook latch fit:** with the guide top 0.3 under the 2.0 skin (−2.3) and the flange on the 7 bosses (−9.0): housings, plungers in and out vs the collar 0 mm³ at all 4 units. **The merged hook latch v1 (`SKIN = 1.6`) overlaps the v7 panel 242 mm³ per unit** (guide top 0.1 into the skin, flange 0.4 into the bosses): `hook_latch.py` must take `SKIN = 2.0` (its frame constants follow; a housing v2) before the housings are printed.

Build: as v6, plus press 4 M3 x 5.7 heat-set inserts into each v7 panel's bosses before gluing. The v7 pair goes on the seated faces (over the 45 / 135° bays, drone ±y), each panel's window toward its own valley.

## Part: funnel panel A / B v7 plain (PETG, 2 + 2; the two unlatched faces, 24 Oct)
v7 without the latch: the same skin 2.0, ribs 9, flanges, keys, seat block and its tie rib; no window, bosses or low ribs. Ribs from `panel_opt_v7.py` run plain (`NOX_FUNNEL_PLAIN=1`, v6 cases and limits + the 2 presses beside the wheel point, no latch cases; `rib_mask_v7_plain.json`): lightest feasible rib fraction 0.20, 132.6 g a face (plate model). Drawn tie ribs 4.4 wide: the wheel-track strut up into the rim stub, the rim corner piece into the valley diagonal, the seat block into the strut. Build: `funnel_panel_v7.py --plain` (C unless tagged).
| Item | Value |
|---|---|
| As built (FE) | grown 1.25 (the least that passes): worst 0.91 of the 1.0 limit at press u 60 v 90, 6.9 MPa max |
| Mass | A 75.4 g, B 75.3 g (M, solid) |
| Bed | 109.0 x 170.6 x 12.0, contact face down, 13 052 mm² (M), 14 mm² over 45°, no supports |
| Checks | solid_check PASS on A and B; in the collar with the 4 v7: 0.000 mm³, 1 body; wheel track 0 steps over 0.2 |
**The 3 printed v6 panels:** the v6 file fails solid_check (1.6 skin thin walls, seat block and a rim piece on skin-only necks), so they are reprinted as v7 plain (DEFAULT; alternative: keep them on the unlatched faces as printed, their 1.6 skin is under the 2.0 floor). They can serve as fit and glue coupons.

## Sequence
1. Drone descends inside ±40, ±5° yaw, ≤ 0.15 m/s.
2. First face touch: motors kill (Spark's landing mode).
3. Wheels roll to centre; KW11 made = seated; the 4 hook latch plungers go out through the v7 skin windows (all 4 halls read out, else abort).
4. Swap (`nox/box/box.md`): the XT60 push lifts the drone 2.1 .. 5.7 (C) onto the plungers. Leaving: plungers in, spin-up, take off. Miss (no switch within 1.0 s): climb, retry.

## Problems
0. **The v6 file fails solid_check** (24 Oct, M): 1.5 thin walls (the 1.6 skin), the seat block on skin only, a rim piece on a 2.4 neck. Fixed by v7 plain on the unlatched faces; the 3 printed v6 panels are reprinted (DEFAULT).
1. **Wheel coupon gate** (dimensions D14): a corner node's wheel turns free after printing and slides sideways on a 45° PETG face at μ ≤ 0.4. Test on a panel's bed-side face.
2. **Valley seams are in the wheel path:** sand flush, step ≤ 0.2, and round the glue line so a wheel can't catch.
3. **The 2 bays the collar bolts to (45° and 135°) aren't drawn** (Overseer bays, wait on D10). Their posts (45° seat + insert) are the bay ring chat's job (spec in `nox/box/box-case.md` "To add" 2). Best fix: draw those two modules now as bay envelopes with the posts, so the collar has something to bolt to; Overseer receivers later.
4. **Glue joints are the structure:** glue coupon (current-parts 9) before gluing the panels.
5. **The FE is a plate model (E):** first article check: glue one face (A + B), press 2 kg at u 60 v 90 (on the wheel track, 90 up the slope), ≤ 1.0 mm against a straightedge. If it fails, rerun `panel_opt.py` with a stiffer limit; it's one number.
6. **Overseer contact positions are assumed on the wheel track** (DEFAULT). If its contacts land elsewhere on the faces, add them as functional cases and rerun.
7. **Strength on the seats (E):** v5 pads (screw from below) checked on the unribbed collar (`funnel_seat_fe.py`: 2 kg Overseer, 50 mm free fall onto one face, flight SF 4, critical distance 1.5): worst layer-aware index ~2.4 at the top-band corners, pad 2.3, so ~1.6 real margin on PETG, not 4. Glue lines read 5 to 7 against a 2 MPa design bond: the glue coupon (Problem 4) decides it. The ribbed collar hangs the mesher. **v6 seat blocks are not FE-checked:** the landing load is now normal to the face, straight into the post in compression (better than the v5 pads' bending), but the screw-through-the-face region is new. First article: 2 kg press on the face next to a screw. If the coupon is weak: the ±30 landing gate already proposed cuts the hit.

## Status
Layout and centring settled 16 Oct 19:50 (sim, all PETG). Flat panel split 16 Oct 21:30. Ribs optimized 16 Oct 22:40. v5 17 Oct 00:35 (pads, screw from below; A v5 sent 00:39, cancelled by Spark). **v6 17 Oct 01:05: seat blocks, M3 countersunk from above into inserts in the bay posts.** Files `funnel_panel_A_v6.stl` / `B_v6.stl` in `dubhacks\`; A v6 sliced, waiting on a clear bed. Next: 1 A + 1 B as the first article (fit, keys, stiffness, countersink), then the other 6; wheel and glue coupons. **v7 24 Oct: the seated faces' 2 A + 2 B with the hook latch windows, bosses and re-optimized ribs (`print/box_funnel_panel_A_v7.stl`, `B_v7.stl`); the unlatched faces' 2 A + 2 B as v7 plain (`print/box_funnel_panel_A_v7_plain.stl`, `B_v7_plain.stl`); the 3 printed v6 are reprinted (DEFAULT).**
