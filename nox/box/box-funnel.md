# Box collar funnel v4 / panels v7 + v8 plain (catcher), shared by the Scout and the Overseer

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
- Faces: 45°, bottom edge z 1.3 (168 square), top z 77.2 (320 square). **The ±x faces (v8 plain) end at z 12.8: opening 191 along drone x by 168 along drone y (24 Oct, DEFAULT).**
- Below the bottom edge: open; 2 pogo pads and the KW11 sit on a post under the belly (box chat, DEFAULT). Hold: the hook latch on the two seated faces (Spark 23 Oct; the electromagnet is retired).
- Camera keep-out: the face reaches x 98 at z 15.3.
- **Mount (v6, v7, v8 plain the same, box structure v2):** seat frame to W by `catch_system.py` SEAT_TO_W (dimensions G System Box catch, L v2 layout): collar centred on the grip line W Y = Y_C 165 (arm PR 79 DEFAULT, hub 24 Oct), seat z 0 = W Z 155.3 at ring R 113 (C); seat y → W −X, seat x → W Y. `box_catch.assembly.json` is posed in that W. The 2 faces over the 45° and 135° bay modules carry the 4 screws; the other 4 holes are spares (any 90° rotation fits). 3D view: artifact "Nox Collar Mount".

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

**Wider opening (24 Oct, v8 plain, box catch DEFAULT).** The Overseer latch v1 set (receiver + cartridge, M) is 190.7 fore-aft at ±5° yaw incl. 3 a side when centred on the catch centre (hub DEFAULT + arm PR 79: each drone's catch centre moves onto its set's centre; E until the Overseer chat moves its wheel nodes); it does not pass the 168 square opening. Chosen: a **rectangular opening 191 along drone x x 168 along drone y**: only the ±x faces (the unlatched ones) end higher, at seat z 12.8 (the face is 82.7 + z, C), 16.26 up the slope; the ±y faces (v7 + hook latch v2) stay as they are. Checked on the real meshes (`catch_v4/opening_sets.py`): Overseer set centred, ±5° yaw, lift 0..5.7, cartridge swept down: 0 mm³, 3.15 / 3.27 to the +x / −x panels; Scout ≥ 40 (C). Motion: `runs/funnel_panel/v8_plain/motion.html` (`catch_v4/opening_motion.py`). Centring does not change: the wheels never run below their seat contact, 27.3 up the slope from the new edge (dimensions G "Opening limit"). Alt: 181 square by cutting all 8 panels at z 7.83, which moves the latched panels and all 4 posts.

Build: print 4 A + 4 B. Scuff the flanges. Glue each A + B pair face-down on a flat table (4 faces). Stand the 4 faces mouth-down, glue the valley flanges, clip, cure. Sand the valley seams flush. Press an M3 heat-set insert into each of the 4 bay posts' 45° seats. Set the collar on the posts (landings on the seats), 4 x M3 x 12 countersunk from inside the funnel.

## Part: funnel panel A / B v7 (PETG, 2 + 2; the two seated faces, 24 Oct)
The 4 panels of the faces over the 45 / 135° bays (drone ±y) carry the hook latch; the other 4 are **v7 plain** (below). Script `catch_v4/funnel_panel_v7.py` (builds on `funnel_panel.py`, which stays the v6 source), rib run `catch_v4/panel_opt_v7.py`, results `runs/funnel_panel/v7/` (`result.json` holds every number below).
| Feature | Value | Tag |
|---|---|---|
| Unchanged from v6 | faces at 45°, opening 168 / mouth 320, centre seam + valley flanges 2.4 x 12, keys, seat block (36 from the centre line, 14 up the slope), M3 x 12 countersunk from above into the bay post insert | C |
| Skin | **2.0** (v6: 1.6, which reads 1.5 thin walls in solid_check, `v6_gate/`) | C, DEFAULT |
| Square ends (review 1) | the skin ends square to the face at the bottom edge (v6 ran a 2.5 lip under it, cut horizontal and by the drafted opening: a 48° feather); the valley flange ends square to the valley line at the rim corner (the rim plane met it obliquely: a 0.7 wedge). Lowest point of the collar stays z −1.2 (rib ends) | C, DEFAULT |
| Skin window | 9.0 along the slope x 13.0 along the edge, centred at its wheel, lateral 100, 54.1 up the slope: plunger 8 x 12 + 0.5 a side (the plunger section grown 0.45 a side clears, 0.55 touches) | C (hook latch spec) |
| Latch bosses | 4 per panel, Ø12 x 7 behind the skin (w −2.0 .. −9.0), M3 x 5.7 heat-set, hole Ø4.0 x 6.5 (2.5 floor), cut after every union (0.000 mm³ of panel inside the 4 holes), 2 gussets 2.0 thick each: `hook_latch.bosses_local()` at the 2.0 skin. The downhill valley-side boss's gussets ran 1.3 past the valley plane: trimmed there, they end in the valley flange. Housing flange face at w −9.0 | C (hook latch spec) |
| Ribs | 9 behind the skin; **low ribs 6.7 over the housing footprint + 1.0** (unit frame u −25.5 .. 34.0, v −10.2 .. 41.6), 0.3 under the housing flange; **none over the guide block + 1.0** (u ±13.2, v −10.2 .. 11.2). Each section opened by 1.0 so no piece under 2.0 wide is left at the footprint edge; ribs stop 0.3 above the skin's bottom end. The spec's rib-free window left the bosses and the wheel-track strut as islands on the skin (solid_check necks) | C, DEFAULT |
| Rib optimization | `panel_opt_v7.py`: the v6 plate model, cases and limits, plus the window as a hole, bosses as 9.0 plate, 2 presses beside the window (u 100, v 40 / 65) and the latch: functional 30 N (E) on both plungers (limit 1.0) and misuse 100 N (E) on one (limit 3.0), the housing a rigid body, F x (9.0 + 9.0) as normal boss forces (7.6 / 25.2 N max, C, conservative). ts 2.0, h 9 fixed; lightest feasible rib fraction 0.20, 134.5 g a face (plate model) | C |
| Drawn tie ribs | 4.4 wide (survive every solid_check erosion): boss to boss along the slope and on both sides of the guide, the uphill bosses into the rim beam, the downhill one into the wheel-track strut, the seat block into that strut (it sat on skin only in v6), the rim corner piece into the valley diagonal | C, DEFAULT |
| As built (plate FE) | outline smoothed as v6, grown 1.25 (the least that passes): worst 0.913 of the 1.0 limit at press u 60 v 90; 6.6 MPa max (limit 19.5) | C |
| Latch loads, panel vs system | panel cases (E): 30 N x 2: mean 0.107 mm, boss 0.33 mm, 1.5 MPa; 100 N: 0.261 mm (0.087 of 3.0), boss 0.73 mm, 4.1 MPa. **Catch system pass** (lead, `nox/box/catch_system.py`, worst plunger, C): insert 28.3 N x 2: 0.101 mm, boss 0.31 mm, 1.4 MPa; hand lift 85.8 N: 0.224 mm (0.075 of 3.0), boss 0.63 mm, 3.5 MPa (C, linear plate FE) | E / C |
| Bosses in plane (hand calc) | shear F / 4 at the flange face + normal boss force, boss axis = print z (across layers): misuse 100 N 1.29 MPa, index 0.149 (SF 2.5); hand lift 85.8 N 1.11 MPa, 0.128; functional 30 N 0.072; insert 28.3 N 0.068; skin membrane ≤ 1.04 MPa | C (geometry) / E, C (loads) |
| Mass | A 80.0 g, B 79.9 g (M, solid); collar 4 x v7 + 4 x v7 plain 620.0 g (M) | M |
| Bed | 107.3 x 167.8 x 12.0 (M), contact face down, 12 935 mm² on the plate (M). print_orient overhang (faces over 30° from vertical) 336 / 340 mm² A / B (M): rib ends cut at 45° by the lowest-point plane, flange ends, the countersink cone; faces under 30° from horizontal 9 / 11 mm² (M, key undersides): tree supports at Bambu's 30° threshold ~0 g (E) | M / E |
| Checks | solid_check PASS on A and B; collar of 4 v7 + 4 v7 plain: 0.000 mm³ between panels, 1 body; wheel track (lateral 50 .. 150, every face): 0 steps over 0.2, face depth 0.000 off the windows (C) | C |

**Hook latch fit:** with the guide top 0.3 under the 2.0 skin (−2.3) and the flange on the 7 bosses (−9.0): housings, plungers in and out vs the collar 0.000 mm³ at all 4 units (C). **The merged hook latch v1 (`SKIN = 1.6`) overlaps the v7 panel 242 mm³ per unit (C)** (guide top 0.1 into the skin, flange 0.4 into the bosses): `hook_latch.py` takes `SKIN = 2.0` (hook latch v2, lead) before the housings are printed.

Build: as v6, plus press 4 M3 x 5.7 heat-set inserts into each v7 panel's bosses before gluing. The v7 pair goes on the seated faces (over the 45 / 135° bays, drone ±y), each panel's window toward its own valley.

## Part: funnel panel A / B v7 plain (PETG, 2 + 2; the two unlatched faces, 24 Oct)
v7 without the latch: the same skin 2.0, ribs 9, flanges, keys, square ends, seat block and its tie rib; no window, bosses or low ribs. Ribs from `panel_opt_v7.py` run plain (`NOX_FUNNEL_PLAIN=1`, v6 cases and limits + the 2 presses beside the wheel point, no latch cases; `rib_mask_v7_plain.json`): lightest feasible rib fraction 0.20, 132.6 g a face (plate model, C). Drawn tie ribs 4.4 wide: the wheel-track strut into the rim stub, the rim corner piece into the valley diagonal, the seat block into the strut. Build: `funnel_panel_v7.py --plain`.
| Item | Value | Tag |
|---|---|---|
| As built (plate FE) | grown 1.25 (the least that passes): worst 0.933 of the 1.0 limit at press u 60 v 90, 6.9 MPa max (limit 19.5) | C |
| Mass | A 75.1 g, B 75.0 g (solid) | M |
| Bed | 107.3 x 167.8 x 12.0, contact face down, 13 052 mm² on the plate; print_orient overhang 709 / 712 mm² A / B (45° rib ends and flange ends), under 30° from horizontal 9 / 11 mm²: tree supports ~0 g (E) | M / E |
| Checks | solid_check PASS on A and B; in the collar with the 4 v7: 0.000 mm³, 1 body; wheel track 0 steps over 0.2 | C |
**The 3 printed v6 panels:** the v6 file fails solid_check (`runs/funnel_panel/v7/v6_gate/`: 1.5 thin walls from the 1.6 skin, the seat block on a 0 mm² joint at (38.9, −47.1), a rim piece on a 2.4 neck, M), so they are reprinted as v7 plain (DEFAULT; alternative: keep them on the unlatched faces as printed, their 1.6 skin is under the 2.0 floor). They can serve as fit and glue coupons.

**Neither v7 plain nor v8 plain is printed (24 Oct, DEFAULT): v8 plain is superseded by v9 plain (keep-alive probe bosses + an arm notch, next PR); the 3 printed v6 panels are reprinted as v9 plain.**

## Part: funnel panel A / B v8 plain (PETG, 2 + 2; the two unlatched faces, drone ±x, 24 Oct; DESIGN, do not print)
v7 plain with its bottom edge at **seat z 12.8** (opening half 95.5, 16.26 up the slope from the v7 edge, C), so the collar opening is 191 along drone x by 168 along drone y. Build: `catch_v4/funnel_panel_v7.py --v8` (the bottom edge is a parameter, `NOX_FUNNEL_ZBOT`; every v7 file unchanged); ribs `NOX_FUNNEL_PLAIN=1 NOX_FUNNEL_ZBOT=12.8 python3 panel_opt_v7.py` (`rib_mask_v8_plain.json`); results `runs/funnel_panel/v8_plain/` (`result.json`, `opening_sets.json`, `summary.md`).
| Item | Value | Tag |
|---|---|---|
| Unchanged from v7 plain | mouth 320, top edge z 77.2, face plane, skin 2.0, ribs 9, centre seam keys (the lower one now 13.74 up from the edge, 11.6 of wall under its hole), valley flange 2.4 x 12 (now from z 12.8; the ±y panels' flanges run free below it) | C |
| Bottom edge | seat z 12.8: skin, ribs and flanges end square to the face on it (review 1 rule); ribs stop 0.3 above it | C, DEFAULT |
| Seat block + M3 | 14 up the slope from the new edge (v 30.26 from the old), 36 from the centre line, same landing Ø16, flare Ø26, post room r 9.5; screw entry (105.4, ±36, 22.7), landing (109.6, ±36, 18.5) seat frame. **The bay post under it rises 11.5 (box structure session's part) and must clamp the landing square: the panel hangs on it** | C |
| FE supports | bottom edge free (it rests on nothing at z 12.8); the two seat landings fixed (disc r 8), both valley glue lines (all above z 12.8); the centre seam glue line is the passive A + B flange strip; the seat block passive (skin + 6.0 inside r 8, 0.5 at r 13). Sensitivity: a landing that only pins reads 1.10 (r 3) / 4.0 (a point) | C |
| Rib optimization | v7 plain's cases and limits less the v 15 press row (below the edge), plus a 20 N press at the seated wheel (u 100, v 43.6, i.e. 27.3 up from the new edge); lightest feasible rib fraction 0.062, 86.2 g a face (plate model): one strut from each seat block up to the rim beam | C |
| Drawn tie ribs | 4.4 wide: seat block up into the strut, strut top into the rim beam (the opened outline left a gap), rim beam out into the valley flange, **a wheel-track strut under the seated wheel (u 100, v 28 .. 98) into the rim beam** (Overseer 6 g wheel 0.52 → 0.17 against the 0.5 flight target, +3.2 g a panel; alt a stub v 30..60, 0.40, +1.6 g) | C, DEFAULT |
| As built (plate FE) | outline grown 1.0 (DEFAULT; alt 0.75 at 0.970): worst 0.911 at the top edge over the seam (2.73 of the 3.0 misuse limit), functional 0.81 at press u 60 v 90, 12.8 MPa max (limit 19.5) | C |
| System wheel loads | at the seated wheel spot (20 N case, linear): Scout 7.2 / 17.0 / 46.3 / 6.1 N per face wheel (insert / pull / hand lift / 6 g): index 0.035 / 0.083 / 0.225 / 0.030; Overseer 0.2 / 17.9 / 51.2 / 35.6 N: 0.001 / 0.087 / 0.249 / 0.174 (19.5 MPa), ≤ 0.31 mm | C |
| Opening | Overseer latch v1 set centred on its set centre, ±5° yaw, lift 0..5.7, cartridge swept down: 0 mm³, 3.15 / 3.27 to the +x / −x panels (at the bottom edge, yaw 5°), ≥ 40 to ±y; Scout ≥ 40 every side; motion `runs/funnel_panel/v8_plain/motion.html` | C (M meshes) |
| Mass | A 53.8 g, B 53.7 g (solid), was 75.1 / 75.0; collar 2 x v7 A + B + 2 x v8 plain A + B 534.8 g, was 620.0 | M |
| Bed | 91.1 x 167.8 x 12.0, contact face down, 11 592 mm² on the plate; print_orient overhang 799 / 802 mm² A / B, over 45° in the render 14 / 16 mm²: tree supports ~0 g (E) | M / E |
| Checks | solid_check PASS on A and B; collar (v7 on ±y, v8 plain on ±x): 0.000 mm³ between panels, 1 body; hook latch v2 in and out 0.000 mm³; nothing inside 191 x 168 below z 12.8; wheel track 0 steps over 0.2 | C |
| Panel v7 recheck | on the real supports too (nothing of box structure v2 is under the z 1.3 foot): bottom edge free, landings clamped (r 8, seat block modelled), valleys only above z 12.8: 0.827 (was 0.913 on the edge support), boss 0.31 / 0.73 mm, Overseer seated wheel 0.42 (6 g) / 0.94 (hand lift); pinned landing 0.854, point 1.09: passes, v7 unchanged (`v7_valley_recheck.json`) | C |

## Cage first (Spark 24 Oct rule, CLAUDE.md)
The collar may meet only cage members: the catch wheels in the corner nodes (Catch interface (1)); everything else of either drone stays ≥ 3 off the faces (the cage-first rule's 3, DEFAULT). **Today's shared number is 1.0** (dimensions G Catch interface (1): "nothing else of the drone within 1.0 of a face", and (7) 1.0 off the extended plungers): **a shared-number question for the Scout v3 and Overseer chats** (raise to 3 so the cage touches first by the rule's margin); this part does not change it. The collar itself has nothing inside the mouth above the faces but the 4 plungers, which only meet the wheels (hook latch fit).

## Sequence
1. Drone descends inside ±40, ±5° yaw, ≤ 0.15 m/s.
2. First face touch: motors kill (Spark's landing mode).
3. Wheels roll to centre; KW11 made = seated; the 4 hook latch plungers go out through the v7 skin windows (all 4 halls read out, else abort).
4. Swap (`nox/box/box.md`): the XT60 push lifts the drone 2.1 .. 5.7 (C) onto the plungers. Leaving: plungers in, spin-up, take off. Miss (no switch within 1.0 s): climb, retry.

## Problems
0. **The v6 file fails solid_check** (24 Oct, M, `runs/funnel_panel/v7/v6_gate/`): 1.5 thin walls (the 1.6 skin), the seat block on skin only, a rim piece on a 2.4 neck. Fixed by v7 plain on the unlatched faces; the 3 printed v6 panels are reprinted (DEFAULT).
1. **Wheel coupon gate** (dimensions D14): a corner node's wheel turns free after printing and slides sideways on a 45° PETG face at μ ≤ 0.4. Test on a panel's bed-side face.
2. **Valley seams are in the wheel path:** sand flush, step ≤ 0.2, and round the glue line so a wheel can't catch.
3. **The 2 bays the collar bolts to (45° and 135°) aren't drawn** (Overseer bays, wait on D10). Their posts (45° seat + insert) are the bay ring chat's job (spec in `nox/box/box-case.md` "To add" 2). Best fix: draw those two modules now as bay envelopes with the posts, so the collar has something to bolt to; Overseer receivers later.
4. **Glue joints are the structure:** glue coupon (current-parts 9) before gluing the panels.
5. **The FE is a plate model (E):** first article check: glue one face (A + B), press 2 kg at u 60 v 90 (on the wheel track, 90 up the slope), ≤ 1.0 mm against a straightedge. If it fails, rerun `panel_opt.py` with a stiffer limit; it's one number.
6. **Overseer contact positions are assumed on the wheel track** (DEFAULT). If its contacts land elsewhere on the faces, add them as functional cases and rerun.
7. **Strength on the seats (E):** v5 pads (screw from below) checked on the unribbed collar (`funnel_seat_fe.py`: 2 kg Overseer, 50 mm free fall onto one face, flight SF 4, critical distance 1.5): worst layer-aware index ~2.4 at the top-band corners, pad 2.3, so ~1.6 real margin on PETG, not 4. Glue lines read 5 to 7 against a 2 MPa design bond: the glue coupon (Problem 4) decides it. The ribbed collar hangs the mesher. **v6 seat blocks are not FE-checked:** the landing load is now normal to the face, straight into the post in compression (better than the v5 pads' bending), but the screw-through-the-face region is new. First article: 2 kg press on the face next to a screw. If the coupon is weak: the ±30 landing gate already proposed cuts the hit.

## Status
Layout and centring settled 16 Oct 19:50 (sim, all PETG). Flat panel split 16 Oct 21:30. Ribs optimized 16 Oct 22:40. v5 17 Oct 00:35 (pads, screw from below; A v5 sent 00:39, cancelled by Spark). **v6 17 Oct 01:05: seat blocks, M3 countersunk from above into inserts in the bay posts.** Files `funnel_panel_A_v6.stl` / `B_v6.stl` in `dubhacks\`; A v6 sliced, waiting on a clear bed. Next: 1 A + 1 B as the first article (fit, keys, stiffness, countersink), then the other 6; wheel and glue coupons. **v7 24 Oct: the seated faces' 2 A + 2 B with the hook latch windows, bosses and re-optimized ribs (`print/box_funnel_panel_A_v7.stl`, `B_v7.stl`). v8 plain 24 Oct: the unlatched faces' 2 A + 2 B, bottom edge at seat z 12.8 (opening 191 x 168), DESIGN, do not print: v9 plain (probe bosses, arm notch) is next; v7 plain is not printed either; the 3 printed v6 are reprinted as v9 plain (DEFAULT).**
