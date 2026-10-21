# S2 cage (Scout v2 prop guard), layout C3, all PETG. **Print version: v3.2 = the v3 nodes with every hole for 3.0 rods (17 Oct 22:36). v4 node set (regenerated at 3.0, 18 Oct 06:30) saved as the optional upgrade, section below.**

Carbon rod cage round the props on top: two clipped-square rings, 4 PETG corner nodes carrying the 8 catch wheels, a crown epoxied on the 4 Radxa boss tops of the S2 top case with 6 spokes down to the upper ring, and a 4-rod skirt from the side nodes down to the drone-side receiver. No feet: the drone lands on the cartridge's flat bottom (Spark 17 Oct 09:20, box-safe). State: `v2/current-parts.md`. Numbers: `v2/dimensions.md` C and G. Scripts (Nox Source): `v2/cage_v2/` (layout `layout_v3.py` + `layout_C3.json`, nodes `node_gen.py`, print set `export_c3.py`, receiver interface `receiver_skirt.py` + `.json`), frame model `v2/s2_guard_frame.py`; v3.2 re-cut `recut_v32.py` (cage chat scratch, not saved: re-cut only). Drone coordinates (x nose, y left, z up, mm).

**Rods (Spark 17 Oct 21:33 / 21:45): every cage rod and wheel pin is 3.0 solid carbon.** v3.2 (Spark 22:32 "just need the holes to fit 3mm") keeps the v3 / v3.1 node shapes and axes and only re-cuts the holes: every rod socket 3.3 + 0.6 lead-in, corner cheek holes 3.3, wheels R 4 with a 3.4 bore. **v4** (regenerated nodes, rerouted spokes, R 5 wheels, index ≤ 0.5): the nodes were regenerated on the v3 axes by the status chat (section "v4 node set"); the cage chat's rerouted-spoke inputs are in Nox Source (`layout_C3.json` rods 3.0, spoke starts nose (13.5, ±8) and side (10, ±31), nose bolt lane r 3.5, `node_gen.py REIN_LIM`) for a later vtee / crown rerun. Note: Nox Source's layout files describe that reroute, not the printed v3.2 axes (v3 spoke starts below).

## Why it looks like this
- Spark 17 Oct 03:01: nodes up to the electronics above the props, layout from an optimization; 03:05 crown on the top case; 03:52 the bottom rods mount to the receiver and tie into the guard; 09:20 box-safe (skirt sockets at deck height, nothing in the swap wheel's top slot, no feet); 10:39 all PETG; 21:33 / 21:45 all rods and pins 3.0; 22:32 holes only, print fast; 21:54 "make all of those new cads" → v4 node set.
- Layout C3 won the frame-model search (`layout_v3.py`): nose and tail V spokes, single side spokes, no plate struts, skirt quad to the receiver. Every printed node was generated (noxopt) for the socket and contact loads the frame model handed it at 2.5 rods (v3) or 3.0 rods (v4 set).

## Geometry (C unless tagged; v3.2 = v3 axes, 3.0 rods)
| Item | Value |
|---|---|
| Prop plane | z 41.2 (plate top 22 + D12 19.2, E); swept band z 37.2..45.2, disks r 44.45 at (±54.1, ±54.1) |
| Rings | flats ±112, chamfers (±112, ±100) to (±100, ±112). Lower rod at z 33.2, upper at z 49.2 (prop plane ∓ 8). One rod per flat per ring, through the tee / side node sleeves. 3.0 solid |
| Corner nodes (4) | span the chamfer; lower and upper ring sockets 3.3 + 0.6 lead-ins (v3.2), 2 wheel pockets (r 4.4, every edge filleted 1.2), pin cheeks with 3.3 holes. v4 set: pockets r 5.4 |
| Wheels (8) | **v3.2: R 4.0, 3.0 wide, 0.5 edge chamfers, bore 3.4 on a 3.0 carbon pin epoxied in both cheeks; rim wall 2.3; centres as v3, so the rim stands 2.25 proud of the 3.0 rod surface (was 2.5 of the 2.5 rod).** v4 set: R 5, pocket r 5.4, rim 2.5 proud of the 3.0 rod surface |
| Nose / tail tee (vtee, 2, same part) | through sleeve on the upper ring at (±112, 0, 49.2), 2 spoke sockets 3.2 either side of the axis |
| Side nodes (L + R mirror) | both rings at (0, ±112): upper sleeve with the side spoke socket, lower sleeve with the 2 skirt sockets (3.2 either side of x 0) |
| Crown | 4 pads on the Radxa boss tops (±11.45, ±28.9), designed at z 68.81 (E); the printed top case v1 has 68.11 (C), so the crown sits 0.7 lower (fits; spoke lengths change < 0.3). 6 spoke sockets at the spoke starts, z 76: nose (13.5, ±5), tail (-13.5, ±4), sides (9, ±31) (v3). Keeps clear of the Radxa back parts, the 3 hex-key lanes and the microSD lane |
| Spokes (6) | nose 2 x 98.1, tail 2 x 98.1, side 2 x 81.8 (cut lengths); 12 off the swept props. 3.0. **At 3.0 the nose spokes pass 0.15 off the nose bolt key lane (r 2.0): the nose bolt goes in before the nose spokes are epoxied** (top case chat flag 22:07; a rerouted v4 vtee / crown would fix it) |
| Skirt (4) | side node lower sockets to the receiver roots (±30.5, ±26.5, 5) (receiver v3.3 sockets 10 deep, cut for 2.5: **drill to 3.3**; v3.4 from `receiver_skirt.json`, rod 3.0, aimed at (±3.2, ±112, 33.2)); 90.1 each. 3.0 |
| Top case side spacers (2) | Ø9.5 x 4.0, bore 3.3, under the case side pads (z 22..26): top case v1 was drawn on the old side hubs, C3 has none |

## v3.2 holes (M, measured on the re-cut meshes in drone coords, 17 Oct 22:36)
| Part | g | Socket walls (min / most) | Notes |
|---|---|---|---|
| corner node (4) | 4.23 | 1.11 lower ring / 1.74 upper ring; cheek holes 3.3, wall 1.36 | 1.11 = collar-face side, set by the 1.0 face clearance |
| vtee (2) | 2.63 | 1.86 / 2.14 | |
| side node L + R | 4.51 each | 1.10 lower ring / 1.75 to 2.14 | 1.10 = collar-face side |
| crown | 7.67 | 0.95 at one spot on each side-spoke socket (5th percentile 1.74) / 1.77 | |
| wheel (8) | 0.15 | rim wall 2.3 | R 4, bore 3.4 |
Every part one watertight body, no breakouts, every bore 3.30 (M). **Not rechecked for strength:** v3 already read corner 1.13 and crown 0.81 (F) against Problem 46's 0.5, and v3.2 adds thinner socket walls and 3.0-rod loads. Fly gently, inspect after any hit.

## v4 node set (status chat, 18 Oct 06:30): regenerated nodes at 3.0 rods, same C3 axes, optional upgrade of v3.2
Generated with the pre-22:20 scripts (C3 axes = the printed v3.2 axes; spoke starts nose ±5, side (9, ±31)) and `layout_C4.json` (every rod and pin 3.0, wheels R 5); loads from the frame model at 3.0 rods (v_intact 0.89 m/s). Scripts in this chat: `node_gen.py` (`ROD` / `WR` from the layout file, `NOX_OVERHANG=0`, `SNODE_TIE`), `layout_C4.json`, `export_c4.py`; not yet in Nox Source (which now holds the cage chat's rerouted v4 inputs). Files: Nox Print Files `s2_cage_v4_<part>.stl.zip.b64.txt` (corner_node, vtee, side_node, side_node_M, wheel), print pose. Crown: v3.2 (its v4 run came out as 9 bodies and died in the print check; v3.2 already has 3.3 bores).
| Part | g (M) | Worst index (M, as built, 3.0-rod loads) | Notes |
|---|---|---|---|
| corner node (4) | 6.10 | **0.99** hit edge-lo (v3 1.13) | 1.8 cm³ generated, 3 reinforcement passes (3.21 → 1.35 → 1.24 → 0.99), 204 hot cells; wheel pockets r 5.4 for the R 5 wheels, cheek holes 3.3; 21 mm² support, 164 bridge; bed face drone-bottom down, 118 mm² M, thin-island run 8 mm |
| vtee (2) | 2.99 | 0.06 | 0.3 cm³, no reinforcement; 152 mm² M on the bed |
| side node L + R | 4.97 | 0.38 hit wall | 0.8 cm³; needed the overhang filter OFF: with it on (-y build) the generated volume started at 2 % and never bridged the two ring sleeves (three failed runs, PETG and TPU, tied and untied); 101 mm² M on its outer side |
| wheel (8) | 0.25 | | R 5, 3 wide, 0.5 chamfers, bore 3.4 on the 3.0 pin |
All one watertight body, function 0/0/0, thin 0. Printed set 50.6 g (M, with the v3.2 crown 7.67 and spacers) + rods 27.1 (C) = 77.7 (v3.2: 69.4). Interchangeable with v3.2 on the same rods and crown, except the wheels (v4 corner pockets take R 5 only). Catch sim (`v2/catch_v4/wheel_r5_check.py`, settle model, 204 drops at 45°): R 4 / Ø2 pin 100 % at μ 0.4, R 5 / Ø3 95 % (axle drag ∝ pin radius); both 84 % at μ 0.5; inside the ±30 gate both 100 % (E). Frame model C4 (3.0 rods, R 5 wheel offset): v_intact 0.89, worst 1 m/s hit 1.12, swap deflection 3.16, clearance 0.74, peak 72 g, rods 26.3 g (C).
Open: crown v4 (regen with the pads tied and `NOX_OVERHANG=0`, or keep v3.2); the cage chat's rerouted spokes (nose ±8, lane r 3.5) are not in this set.

## Frame model (F, `layout_v3.py` evaluate on `layout_C3.json`; 0.40 kg into a rigid wall, pole or edge at every ring point; carbon 622 MPa, PETG crash class 39.4 in-layer / 19.7 across)
| Case | 2.5 rods (v3) | **3.0 rods (22:20)** |
|---|---|---|
| Rods at index 1 (cage design speed) | 0.71 m/s | **0.89 m/s** |
| Worst 1 m/s hit | 1.40 (edge hit, upper ring) | **1.13** (same) |
| 0.5 m/s touch | 0.70 | **0.56** |
| Prop disk clearance under the 1 m/s hit | 0.54 | **0.50** |
| Peak deceleration | 57 g | **73 g** |
| Swap push 15 N, wheel deflection | 4.6 | **3.2** |
| Ceiling hit (crown) | 0.23; case load 428 N | **0.20; 452 N** |
| Peak skirt root force on the receiver | 132 N | **183 N** (`receiver_skirt.json`) |
The 3.0 column uses the v4 spoke starts (nose ±8, side x 10); the v3 starts change it by < 0.01 (the hits that set these numbers are on the rings). Catch sim (`v2/catch_v4/s2_catch_v4.py`, full physics, 204 drops, ±40 spec): R 5 wheels 86.8 %, R 4 86.3 %.

## Print (DubHacks rules: tree supports, largest face down)
**v3.2: one plate `latest\s2_cage_plate.3mf` (Nox Print Files `s2_cage_PETG_plate_v3.2.3mf.b64.txt`, sha a06ab1709693), 19 parts, 40.7 g (M); sliced `latest\s2_cage_plate_v32_supports.gcode.3mf` 37.09 g, 2 h 46 min, 0.20 Strength, 6 walls, tree(auto) on, 0 g generated; on the SD card 22:50.** Do not print the v3 / v3.1 files (2.8 bores). v4 node set: singles only (the 19-part plate is over the 16 MB file limit; the print chat lays one out from the singles, same bed faces).
| Part | Qty | g each (M, v3.2) | Bed face | Bed contact (M) | Height |
|---|---|---|---|---|---|
| corner node | 4 | 4.23 | drone bottom down | 124 | 22.8 |
| vtee | 2 | 2.63 | drone bottom down | 130 | 12.5 |
| side node L / R | 1 + 1 | 4.51 | outer side down | 103 | 19.7 |
| crown | 1 | 7.67 | pads down | 515 | 10.9 |
| wheel | 8 | 0.15 | flat | 33 | 3.0 |
| side spacer | 2 | 0.32 | flat | 62 | 4.0 |
Bed faces as v3 (`print_orient.py` pass on v3: the saved face is the best for every part; a hole re-cut does not change it).

## Carbon cut list (C, socket bottom to socket bottom less 0.5 for epoxy; v3.2 = v3 axes; the v4 node set uses the same list)
| Rod | Pieces | Total |
|---|---|---|
| 3.0 solid | 8 x 188.5 rings, 4 x 98.1 nose / tail spokes, 2 x 81.8 side spokes, 4 x 90.1 skirt | 2424 |
| 3.0 solid | 8 x 7.5 wheel pins (through both cheeks) | 60 |
Rods 27.2 g (C, 2.48 m x 7.07 mm² x 1.55 g/cm³). **Cage total ~69.4 g (40.7 M + 27.2 C + ~1.5 epoxy E) vs the 55 budget in dimensions G: +14, Spark's call (current-parts item 26).**

## Assembly
1. Wheels: into the pocket, pin through both cheeks, epoxy the cheek holes only; spin check.
2. Side spacers under the top case side pads (side bolts already +4); **nose bolt in first**; crown epoxied on the 4 boss tops, square to the plate.
3. Rings: each flat rod through its vtee / side node sleeve into the corner sockets, lower then upper.
4. Spokes crown to vtees and side nodes; skirt rods side nodes to the receiver sockets (receiver v3.3: drill the 4 sockets to 3.3 first).
5. Epoxy every rod end (never press fit); square the rings on a flat table before it sets.

## Problems
1. **Corner node index 1.13, crown 0.81** (F, v3 at 2.5 rods; Problem 46 wants ≤ 0.5 on socket members). v3.2 is weaker still (thinner socket walls, 3.0-rod loads). **v4 node set: corner 0.99, side 0.38, vtee 0.06 (M, 3.0-rod loads); crown still v3.2.** Remaining fix: crown regen; corner to ≤ 0.5 would need more volume or a thicker socket wall at the collar face (DEFAULT: accept 0.99, the index over-rates small sections, Problem 46).
2. **Lower-ring socket walls 1.1 on the collar-face side, crown side-spoke sockets 0.95 at one spot** (M, v3.2). v4 set: sleeve walls drawn for 3.0 (outer r 3.65, trimmed to the 1.0 face clearance like v3); crown unchanged.
3. **Nose spokes 0.15 off the nose bolt key lane** (C, 3.0 rods on v3 axes). Fix now: bolt before epoxy. Later: nose lane r 3.5, spokes at ±8 (cage chat inputs in Nox Source; needs a vtee + crown rerun).
4. Prop clearance under a 1 m/s hit is 0.50 (F): the rings never reach a disk, with little margin.
5. Corner node instances are 4 identical prints placed by rotation; the loads were designed with the 4 corners mirrored into one (DEFAULT).
6. **Mass over budget with 3.0 rods** (~69.4 g vs 55; v4 set 77.7): the rods alone are 27.2; v4 does not get it back. Spark's call.
7. dimensions G "Catch wheels" / "Catch interface": printed cage (v3.2) is R 4 on a Ø3 pin, the v4 node set is R 5. Fixed in dimensions G 18 Oct 06:40 (both stated; the shared interface stays the wheel points and face planes, not the radius).
8. **Crown v4 regen failed** (18 Oct 06:11): 2.2 cm³ at h 1.0 came out as 9 bodies after prune (tail spoke sockets and pads detached), worst 1.10, then an empty mesh in the print check. Fix if wanted: rerun with `NOX_OVERHANG=0` and the pads tied (`support` list), ~1 h; otherwise v3.2's crown stays (3.3 bores, 0.81 F at 2.5-rod loads, not rechecked at 3.0).
