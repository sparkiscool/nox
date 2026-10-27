# Scout v3 layout v3 / cage v1 geodesic (27 Oct 2026, concept)

Spark 27 Oct: "using the same design idea as the latest scout design make sure the bottom of the cage is the exact same as the overseer just that loop of rods directly under the battery so funnel is interchangeable, the scout still has one camera and we will keep the carbon base for the scout".

Read as: the latest Scout layout (PR 94, run v5: Bee35 carbon plate, props on top, receiver v3 + cartridge v6 under it, FC / UBEC / Radxa / flow on the belly mount) with the latest cage idea (the Overseer cage v3 geodesic shell, the rule since 26 Oct), its bottom loop = the Overseer's opening ring. Supersedes the PR 94 ground-structure cage (pre-geodesic rule).

Script `nox/scout/v3/layout_v3.py` (~13 min: balance x start / dome / prop-snap scan), stills + artifact `layout_v3_view.py`. Drone meshes `drone/` (PR 94 run v5 as drawn, `drone/SOURCE.txt`). Numbers `nox/dimensions.md` "System Scout v3".

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (source `motion.html`, still `preview.png`; orange = the shared ring, light blue = the Overseer cage ring on ring, green = camera view)

System: Scout v3, cage first 7.4 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 2.46 mm), shell 91 rods closed, h/w 0.37, fit 0.0 mm³ (E), bbox 16.1 L (E), extents 372 x 290 x 149 mm (E), mass 928 of 500 g (186 %) (E) over budget (total), CG 2.1 mm off thrust axis (E), 1.2 mm without swap parts, probes 8.5 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## The shared bottom ring (the funnel interface)
- The Overseer cage v3's 8 opening ring nodes and 8 tubes, read from `runs/overseer_layout/v5/cage_shell.json` at run time: nodes (±176.0, ±43.2) and (±72.9, ±104.3) about the catch centre, 352.0 x 208.7 between node centres, 6 mm tubes on Ø20 nodes; on the Scout within 0.000 mm (C).
- Datum: ring centre on the catch (the grip / tab line), ring plane 3.0 under the cartridge flat bottom, as on the Overseer (its cartridge bottom −54, ring −57). The flat bottom sits at R 113 in every bay and at the drone for both packs, so both rings sit at the same height over the swap head. Scout: tab line x −12.5, ring plane z −47.5.
- Above the ring the Scout's shell stays inside the Overseer's: tube axes 6.0 inside the Overseer's tube-axis hull up to 80 above the ring (C). A funnel built round the Overseer's ring and lower shell takes the Scout with nothing of it proud.

## Layout (C geometry on E sizes; `checks.txt`)
- Carbon base kept: the Bee35 plate carries the 4 motors, props on top (prop plane z 41.2). 3 spokes (3.0 carbon) per arm tip under the motor into the shell, 2 core ties at the plate nose and tail, 8 bay ties from the receiver bottom corners to the ring.
- **Balance:** the nose camera pulls the CG forward (pack in +5.9, pack out +7.8 with the catch on the thrust axis). The catch set (receiver, cartridge, pack, belly mount with the FC, UBEC, Radxa and flow) and the ring slide 12.5 aft of the thrust centre (plate centre), searched 0..24; CG 1.8 mm pack in, 1.6 pack out (layout model), system_check 2.1 / 1.2 (gate 3). The FC and UBEC ride in the belly mount (PR 94: the mount takes z 9.6..18.5 round the FC), so they move with it and keep their clearance to the receiver; the FC's PX4 IMU offset changes (x −12.5).
- **One camera, outside the cage:** DECXIN 2633V2 landscape (89 x 73, E), horizon, lens (173, 0, 30). Placement searched over lens height 30..110 and back-plane tilt 0..90: the shell target 18.1 under its lower view plane and behind its back; compact first (no growth of the drone's box: it sits inside the ring's length), then the shortest boom. Nothing of the drone in its view (C). Held from behind its view on a 12 x 12 PETG boom, 95 long, from the plate nose to a seat on the camera back (4.1 clear of the shell tubes), a guard bar of 2 printed nodes over its top behind the lens, 2 struts each into the shell. Cage first 7.4 everywhere outside its 55° field cone; it stands 2.5 out along its axis. Alternative weighed in the same search: the camera on top (lens (82, 0, 102)), boom 81, but its guard bar tops out ~134 against the shell's 91, so the drone grows ~43 taller (rejected: under-table work wants height; the nose camera adds no length inside the ring).
- **Shell:** frequency 2 class-I geodesic formed to the drone (props 12 off, the rest 3 off the tube surface), mirror-exact (system_check: nodes within 0.0, 0 rods without a mirror). Scan start 0 / 0.7 / 0.85 / 1.0 x dome 20 / 40 / 60 x prop snap: the most compact valid shell sets the bar (start 1.0 dome 40: 7.2 L, 346 g), the least prop shadow within 10 % / 3 % wins: start 1.0, dome 40, snap. 34 nodes, 91 tubes 45..155 (8.0 m), node extents 352 x 270 x 129, largest cell Ø65; tubes 13.9 off the swept disks.
- Props shadow 19.7 % above, 10.1 % below (plan shadow weighted by r, E; the Overseer 8.0 / 14.7): small 3.5 in disks under 6 mm tubes. The thrust loss is less than the shadow (tubes 15+ off the disk), but it is the main cost of the 6 mm shell on this drone; a flight current check on the first hover tells.
- Swap zone (receiver + 30 a side, down from the cartridge bottom): 0 mm³. Flow view straight down: clear. Access 8 / 8: cartridge, pack and plug down, 4 receiver bolt keys from below, FC USB-C cable out the tail through a cell (searched path, 49° off the tail-left diagonal).
- Feet: none drawn; the 8 ring nodes are the lowest points (13 under the cartridge flat bottom) and land first (TPU caps on the ring nodes, DEFAULT, with the funnel session's contact).

## Two alternatives weighed (concept rule)
| | A: the Overseer's ring (Spark, picked) | B: a ring round the Scout's own receiver + 30 |
|---|---|---|
| Ring | 352.0 x 208.7, identical to the Overseer's | 249.4 x 193.9 |
| Shell | 34 nodes, 91 tubes, 8.0 m, 352 x 270 x 129 | 36 nodes, 97 tubes, 7.7 m, 274 x 268 x 139 |
| Mass (E) | 928 g | 924 g |
| Props shadow above / below | 19.7 / 10.1 % | 18.2 / 18.5 % |
| CG pack in / out | 1.8 / 1.6 | 1.8 / 1.8 |
| Funnel | the Overseer's funnel takes it | its own funnel |
B saves 4 g and 78 mm of length, but shades the props more from below, and loses the shared funnel; A is Spark's ask and costs almost nothing. `alt_own_ring/` (`SCOUT_RING=own`).

## Carbon (6 mm tube, 3 mm rod)
- Shell: 91 tubes 6/4, 8.0 m, shortest 45 (all over the 33 break-even).
- Links: 26 rods 3.0, 2.58 m: 12 spokes, 8 bay ties, 2 core ties, the camera guard bar and 4 struts. Shortest 38 (cam guard strut).

## Mass (E)
928 g (system_check): shell 348 (tubes 195 + 34 nodes 153), links 28, camera guard nodes 3; bought, latch set and wires ~505. PR 94's cage was ~474 g (1007 g all-up). Over the 500 g soft budget: the grams buy a closed shell that touches first everywhere, the shared funnel, and 6 mm tubes (rule). T/W not checked: the motor KV and thrust are not recorded (open).

## Next
Frame model on the shell (crash 35 g in 14 directions through the first node, landing on the ring, handling 50 N), then the pieces from its requirements: shell nodes, camera boom + seat + guard, belly mount (FC, UBEC, Radxa, flow round the receiver, receiver 12.5 aft of the plate centre), spoke clamps at the arm tips. Keep-alive contacts with the new funnel (box catch).
