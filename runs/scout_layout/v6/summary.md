# Scout v3 layout v3 / cage v1 geodesic (27 Oct 2026, concept)

**Superseded 27 Oct by cage v2 (`runs/scout_layout/v7/summary.md`: every rod 3 mm, 65 rods, Spark). Kept: summary, results, stills; meshes in git history; `drone/` stays (the input of v7).**

Spark 27 Oct: "using the same design idea as the latest scout design make sure the bottom of the cage is the exact same as the overseer just that loop of rods directly under the battery so funnel is interchangeable, the scout still has one camera and we will keep the carbon base for the scout".

Read as: the latest Scout layout (PR 94, run v5: Bee35 carbon plate, props on top, receiver v3 + cartridge v6 under it, FC / UBEC / Radxa / flow on the belly mount) with the latest cage idea (the Overseer cage v3 geodesic shell, the rule since 26 Oct), its bottom loop = the Overseer's opening ring. Supersedes the PR 94 ground-structure cage (pre-geodesic rule).

Script `nox/scout/v3/layout_v3.py` (~13 min: balance x start / dome / prop-snap scan), stills + artifact `layout_v3_view.py`. Drone meshes `drone/` (PR 94 run v5 as drawn, `drone/SOURCE.txt`). Numbers `nox/dimensions.md` "System Scout v3".

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (source `motion.html`: the picked drone, the shared ring in orange, the Overseer cage ring on ring as a ghost, the camera view; alternatives B and C beside it). Stills `preview.png` (picked, 4 views), `options.png` (A / B / C top and side).

System: Scout v3, cage first 7.4 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 2.46 mm), shell 91 rods closed, h/w 0.37, fit 0.0 mm³ (E), bbox 16.1 L (E), extents 373 x 290 x 149 mm (E), mass 933 of 500 g (187 %) (E) over budget (total), CG 2.2 mm off thrust axis (E), 1.3 mm without swap parts, probes 8.5 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## The shared bottom ring (the funnel interface)
- The Overseer cage v3's 8 opening ring nodes and 8 tubes, read from `runs/overseer_layout/v5/cage_shell.json` at run time: nodes (±176.0, ±43.2) and (±72.9, ±104.3) about the catch centre, 352.0 x 208.7 between node centres, 6 mm tubes on Ø20 nodes; copied exactly (0.000 mm on the shared datum, C).
- Datum (the rule, `dimensions.md` G Shared bottom ring): ring centre on the catch (the grip / tab line), ring plane 3.0 under the cartridge flat bottom (R 113 in every bay and at the drone, both packs). Scout: tab line x −13.5 (the latch sliders' centre), cartridge v6 flat bottom z −44.5, ring plane z −47.5.
- **Open (Overseer side):** the Overseer layout v5 still draws its latch set as v2's cartridge envelope (157 x 54 x 51.9; its centre x −30 = its catch 30 aft, its bottom −54 = the ring's datum). When the Overseer refits to the latch v4 set (171.0 x 55.0 x 58.6, tab line at its centre) its ring stays on the same rule (catch centre, flat bottom − 3), and this script reads it again. Until then the match is the same ring on the same rule, not yet both on the real latch meshes.
- Above the ring the Scout's shell stays inside the Overseer's: tube axes inside the Overseer's tube-axis hull by 5.9 at 0..20 above the ring (the tubes leaving the shared nodes), 17.4 at 20..40, 34.8 at 40..60, 52.2 at 60..80 (C). The funnel bears on the outside of the Overseer's lower shell, so nothing of the Scout above the ring can reach it: the Scout rises steeper.
- Not modelled yet: the new funnel's hook latch zone and keep-alive pads (the box catch session draws the funnel round the Overseer's shell). Every link that lands on a ring node comes from inside and above (bay ties from the receiver corners at z −34, spokes from the arm tips at z 17), so the ring's outer lower half, the funnel contact, stays free; the pads go where the funnel session puts them.

## Layout (C geometry on E sizes; `checks.txt`)
- Carbon base kept: the Bee35 plate carries the 4 motors, props on top (prop plane z 41.2).
- **Balance:** the nose camera pulls the CG forward (pack in +5.9, pack out +7.8 with the catch on the thrust axis). The catch set (receiver, cartridge, pack, belly mount with the FC, UBEC, Radxa and flow) and the ring slide 13.5 aft of the thrust centre (plate centre), searched 0..24 then refined; CG 1.9 mm pack in, 1.7 pack out (layout model), system_check 2.2 / 1.3 (gate 3). The FC and UBEC ride in the belly mount (PR 94: the mount takes z 9.6..18.5 round the FC) and keep their clearance to the receiver; the FC's PX4 IMU offset becomes x −13.5.
- **One camera, outside the cage:** DECXIN 2633V2 landscape (89 x 73, E), horizon, lens (173, 0, 30). Placement searched over lens height 30..110 and back-plane tilt 0..90: the shell target 18.1 under its lower view plane and behind its back; compact first (no growth of the drone's box: it sits inside the ring's length), then the shortest boom. Nothing of the drone in its view (C). Cage first 7.4 outside its 55° field cone; it stands 2.5 out along its axis.
- **Camera mount, braced (review 27 Oct: no stalk):** a 12 x 12 PETG boom, 95 long, from the plate nose to a seat on the camera back (4.1 clear of the shell tubes); the seat is braced from its low and high back corners into the shell (2 + 2 rods, 52 / 79) and from its low corners to the front arm beams 30 out of the plate (2 rods, 135): with the boom a braced pyramid on the core. A guard bar of 2 printed nodes over the camera top behind the lens, 2 struts each into the shell. Break order (requirement for the frame model case "nose hit on the camera"): the boom's root lug at the plate nose is the printed fuse, it lets go before the plate or the camera.
- **Links (3.0 carbon), 32 rods, 3.04 m, no two through each other (closest pair 1.4 between rod surfaces, C):** 2 spokes per arm tip from under the motor to the 2 ring nodes on its own side (82..150; every other shell node on its side sits over a prop disk, so a 3rd spoke would cross the disk band: the frame model sizes these long 3.0 rods); 8 bay ties from the receiver bottom corners to the ring; core ties from the plate underside: tail 1, left 2, right 2 (the nose edge carries the camera boom and its arm braces instead; a nose tie has no clear node there); camera braces 6, guard bar + 4 struts.
- **Shell:** frequency 2 class-I geodesic formed to the drone (props 12 off, the rest 3 off the tube surface), mirror-exact (system_check: nodes within 0.0, 0 rods without a mirror). Scan start 0 / 0.7 / 0.85 / 1.0 x dome 20 / 40 / 60 x prop snap: the most compact valid shell sets the bar (start 1.0 dome 40: 7.2 L, 346 g), the least prop shadow within 10 % / 3 % wins: start 1.0, dome 40, snap. 34 nodes, 91 tubes 45..156 (8.0 m), node extents 352 x 270 x 129, largest cell Ø65; tubes 13.9 off the swept disks.
- Props shadow 19.7 % above, 11.5 % below (plan shadow weighted by r, E; the Overseer 8.0 / 14.7): small 3.5 in disks under 6 mm tubes. The thrust loss is less than the shadow (tubes 13+ off the disk), but it is the main cost of the 6 mm shell on this drone; a flight current check on the first hover tells.
- Swap zone (receiver + 30 a side, down from the cartridge bottom): 0 mm³. Flow view straight down: clear. Access 8 / 8: cartridge, pack and plug down, 4 receiver bolt keys from below, FC USB-C cable out the tail through a cell (searched path, 49° off the tail-left diagonal).
- Feet: none drawn; the 8 ring nodes are the lowest points (13 under the cartridge flat bottom) and land first (TPU caps on the ring nodes, DEFAULT, with the funnel session's contact).

## Three options weighed (concept rule; all drawn: `options.png`, the artifact)
| | A: the Overseer's ring, nose camera (picked) | B: a ring round the Scout's own receiver + 30 | C: the Overseer's ring, camera on top |
|---|---|---|---|
| Ring | 352.0 x 208.7, identical to the Overseer's | 249.4 x 193.9 | as A |
| Whole drone (all parts) | 373 x 290 x 149 | 320 x 288 x 160 | 372 x 289 x 180 |
| Mass (E) | 933 g | 935 g | 937 g |
| Props shadow above / below | 19.7 / 11.5 % | 18.2 / 21.0 % | 19.6 / 9.4 % |
| CG pack in / out (layout model) | 1.9 / 1.7 | 2.1 / 2.2 | 1.5 / 1.8 |
| Funnel | the Overseer's funnel takes it | its own funnel | the Overseer's funnel |
B is 53 shorter but loses the shared funnel and shades the props twice as much from below; C is 31 taller (bad under tables). A is Spark's ask at no mass cost. Runs: `alt_own_ring/` (`SCOUT_RING=own`), `alt_top_camera/` (`SCOUT_CAM_Z=90,110`).

## Carbon (6 mm tube, 3 mm rod)
- Shell: 91 tubes 6/4, 8.0 m, shortest 45 (all over the 33 break-even).
- Links: 32 rods 3.0, 3.04 m, shortest 38 (a guard strut); the spokes 82..150 and the arm braces 135 are the long ones the frame model must size (3.0 by default, thicker only where it says).

## Mass (E)
933 g (system_check): shell 348 (tubes 195 + 34 nodes 153), links 33, camera guard nodes 3; bought, latch set and wires ~505. PR 94's cage was ~474 g (1007 g all-up). Over the 500 g soft budget: the grams buy a closed shell that touches first everywhere, the shared funnel and the 6 mm tubes (rule). T/W not checked: the motor KV and thrust are not recorded (open).

## Next
Frame model on the shell (crash 35 g in 14 directions through the first node incl. a nose hit on the camera, landing on the ring, handling 50 N), then the pieces from its requirements: shell nodes, camera boom + seat + braces + guard, belly mount (FC, UBEC, Radxa, flow round the receiver, receiver 13.5 aft of the plate centre), spoke clamps at the arm tips. Keep-alive contacts and latch zone with the new funnel (box catch). The Overseer's ring on its real latch set (Overseer session).
