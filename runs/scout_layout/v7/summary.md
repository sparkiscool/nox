# Scout v3 layout v3 / cage v2: every rod 3 mm, about half the rods, any rod length (27 Oct 2026, concept)

**Superseded 27 Oct by the rectangle-bottom design (`runs/scout_layout/v8/summary.md`, Spark: "get rid of the two trapezoids", "just choose one design"). Kept: summary, results, stills.**

Spark 27 Oct, on cage v1 (`runs/scout_layout/v6/`, PR 124: 91 x 6 mm shell tubes + 32 x 3 mm links = 123 rods): "subtract the number of rods, these are all 3mm as well and also just divide the number of rods by half", then "dont worry about same rod legnth anymore". Unchanged from cage v1: the PR 94 drone, the bottom ring = the Overseer's (funnel shared), one nose camera outside the cage, the Bee35 carbon base.

Read as: every Scout rod 3.0 solid carbon (shell and links), about half of cage v1's 123 rods, and the shell no longer has to keep the geodesic pattern's near-equal rod lengths. Rule lines: CLAUDE.md (Cages are geodesic shells: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Script `nox/scout/v3/layout_v3.py` (pattern "free", ~13 min), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/` (PR 94 run v5). Numbers `nox/dimensions.md` "System Scout v3".

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 3; source `motion.html`; picked drone, the shared ring in orange, the Overseer cage ghosted ring on ring, the camera view, options B and C beside it). Stills `preview.png`, `options.png`.

System: Scout v3, cage first 7.4 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 1.59 mm), shell 49 rods closed, h/w 0.37, fit 0.0 mm³ (E), bbox 15.6 L (E), extents 372 x 285 x 146 mm (E), mass 690 of 500 g (138 %) (E) over budget (total), CG 2.8 mm off thrust axis (E), 1.9 mm without swap parts, probes 7.1 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## Rods: 65, was 123 (−47 %), all 3.0 solid carbon
- Shell 49 rods (6.22 m, 63..232 long), 20 nodes: the 8 shared ring nodes (Ø20, the Overseer's, the funnel contact) + 12 printed nodes Ø12 for 3.3 sockets (pad rule).
- Links 16 rods (1.49 m): 2 spokes per arm tip from under the motor (82..112), 2 bay ties from the receiver's tail corners to the ring (128), 1 core tie from the plate tail into the shell (89), the camera seat braced to the front arm beams (2 x 125), the guard bar over the camera (46) with 1 strut a side (58). No two rods closer than 3.6 between surfaces.
- Exactly half (61) was drawn too: the 46-rod shell (offset 24) has its CG 22 mm forward on its own, so no catch position balances it pack in and pack out together (3.6 / 2.4 mm against the 3.0 gate). The picked 49-rod shell balances on its own; 3 more rods buy the balance.

## The shell: free pattern (no equal rod lengths)
- The drone's margin hull (props 12 off, the rest 3 off the rod surface) is grown by an offset along 6 directions; nodes go on its corners, the one farthest outside the hull so far first (a mirror pair, or one on the centre plane), until the hull of the ring and the nodes holds the whole drone. A larger offset needs fewer nodes for a slightly bigger cage. Scan 16..36: 55 / 46 / 49 rods; within the rod cap (49) the most compact shell balanced on its own wins: offset 28, 7.5 L, node extents 352 x 273 x 130, largest cell Ø122 (cage v1 Ø65), rods 15.9 off the swept disks.
- Mirror-exact (system_check: nodes within 0.0, 0 rods without a mirror). Closed outside the bottom opening.
- Props shadow 6.8 % above, 5.6 % below (cage v1 19.7 / 11.5): thin rods and fewer of them.
- The cells are large (Ø122): cage first and the crash probes still pass (7.4 / 7.1 mm); the frame model decides whether 3.0 rods of up to 232 hold the crash and handling cases.

## Unchanged from cage v1 (`runs/scout_layout/v6/summary.md`)
- Bottom ring = the Overseer's 8 nodes and 8 ring rods on the same datum (catch centre, 3 under the cartridge flat bottom; Scout tab line x −13.0, ring z −47.5); ring rods now 3.0 like the rest, ring nodes kept Ø20. Above the ring the Scout's rods stay inside the Overseer's shell: 3.8 at 0..20, 15.5 at 20..40, 30.9 at 40..60, 46.4 at 60..80 above the ring. Open: the Overseer side still uses v2's latch envelope.
- Camera: lens (164, 0, 30), nothing of the drone in its view, 12 x 12 PETG boom 87 from the plate nose (10.0 clear of the shell rods), seat braced to the front arm beams, guard bar behind the lens; boom root lug = the printed fuse.
- Balance: catch set 13.0 aft of the thrust centre: CG 2.4 / 2.6 (layout model), 2.8 / 1.9 (system_check, gate 3).
- Access 8 / 8 (swap down, 4 receiver bolt keys, FC USB-C out the tail-left through a cell).

## Three options (all drawn: `options.png`, the artifact)
| | A: free pattern (picked) | B: cage v1's geodesic, every rod 3 mm | C: the geodesic thinned to ~half |
|---|---|---|---|
| Rods (shell + links) | 65 (49 + 16) | 130 (94 + 36) | 65 (49 + 16) |
| Whole drone | 372 x 285 x 146 | 372 x 287 x 135 | 372 x 361 x 205 |
| Mass (E) | 690 g | 762 g | 694 g |
| Props shadow above / below | 6.8 / 5.6 % | 7.5 / 13.3 % | 3.2 / 5.5 % |
| CG pack in / out (layout) | 2.4 / 2.6 | 2.6 / 2.4 | 1.8 / 1.9 |
| Largest cell | Ø122 | Ø68 | Ø134 |
B keeps twice the rods; C has the same count but its sparse geodesic balloons 76 wider and 59 taller. A is half the rods at B's size. Runs `alt_geodesic_full/` (`SCOUT_PATTERN=geodesic SCOUT_NFREE=0 SCOUT_LEAN=0`), `alt_geodesic_half/` (`SCOUT_PATTERN=geodesic SCOUT_NFREE=9`).

## Carbon
Every rod 3.0 solid (Spark 27 Oct): shell 49 (63..232), links 16 (46..128), all over the 33 break-even. Thicker only if the frame model asks (CLAUDE.md rod rule).

## Mass (E)
690 g (system_check): shell 122 (rods 68 + nodes 54), links 16, guard nodes 3; bought, latch set and wires ~505. Cage v1 933 g. T/W not checked (motor KV not recorded).

## Next
Frame model on this cage (crash 35 g in 14 directions incl. a nose hit on the camera, landing on the ring, handling 50 N on any rod or node): long 3.0 rods (to 232) and the large cells are the risk; it may ask for thicker rods or one more node. Then the pieces: nodes, spoke clamps, camera boom + seat + braces + guard, belly mount (receiver 13.0 aft of the plate centre).
