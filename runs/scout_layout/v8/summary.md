# Scout v3 layout v3 / cage v2 (rectangle bottom): the Scout design (27 Oct 2026, concept)

Spark 27 Oct, in order:
- On cage v1 (`runs/scout_layout/v6/`, 123 rods): "subtract the number of rods, these are all 3mm as well and also just divide the number of rods by half"
- "dont worry about same rod legnth anymore" (`v7/`: octagon ring, 65 rods)
- "get rid of the two trapezoids so the bottom is just a rectangle and update this as the new scout ... a reverse funnel that works for both and the scout can just use less ramps"
- "just choose one design"

This is the one design: no options are shown. Unchanged throughout: the PR 94 drone, one camera, the Bee35 carbon base.

Script `nox/scout/v3/layout_v3.py` (defaults: ring rect, pattern free, lean links), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/`. Numbers `nox/dimensions.md` "System Scout v3". Rule lines: CLAUDE.md (Cages: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 4; source `motion.html`; orange = the bottom rectangle, light blue = the Overseer cage on the same datum, green = the camera view). Still `preview.png`.

System: Scout v3, cage first 7.4 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 4.4 mm), shell 47 rods closed, h/w 0.48, fit 0.0 mm³ (E), bbox 12.4 L (E), extents 282 x 276 x 160 mm (E), mass 663 of 500 g (133 %) (E) over budget (total), CG 2.9 mm off thrust axis (E), 1.6 mm without swap parts, probes 5.7 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## Bottom: the Overseer ring's middle rectangle
- The Overseer cage v3 ring is an octagon: a middle rectangle plus a trapezoid at the nose and one at the tail. The Scout keeps its 4 side nodes, (±72.9, ±104.3) about the catch centre (145.8 x 208.7), on the same datum: catch centre on the grip line, plane 3.0 under the cartridge flat bottom. The two trapezoids are dropped. Scout: tab line x −12.0, rectangle plane z −47.5, nodes Ø20 (the funnel contact), rods 3.0.
- Funnel: the reverse funnel serves both drones. The Overseer bears on all of its ring; the Scout bears only on the ramps under its rectangle's 4 nodes and 4 sides (fewer ramps; message sent to the design center session). Above the bottom, the Scout's rods stay inside the Overseer's shell: 3.5 at 0..20, 17.3 at 20..40, 34.7 at 40..60, 52.0 at 60..80 above it (C).
- Swap column: along x the rectangle leaves the receiver + 13.7 at each end (was + 30, DEFAULT), and + 30 at the sides; the column is clear (0 mm³). The cartridge (96 long) drops through a 145.8 opening. Box swap arm and catch sessions: check the head and grabber against that 13.7.
- Open (Overseer side): its layout still draws v2's latch envelope (bottom −54). The datum rule holds when it refits to the latch v4 set.

## Cage: 61 rods, all 3.0 solid carbon (cage v1: 123)
- **Shell, 47 rods** (59..211, 5.61 m) on 18 nodes: the 4 bottom nodes Ø20 and 14 printed Ø12 nodes for 3.3 sockets. Free pattern, so rod lengths vary: nodes sit on the corners of the drone's margin hull (props 12 off, the rest 3 off the rod surface), grown 28 mm along 6 directions, added farthest-outside first in mirror pairs until the hull holds the drone.
  - Within the rod cap, the most compact shell that balances on its own wins: offset 28 of 16..36.
  - Mirror-exact, closed outside the bottom opening.
  - Node extents 263 x 264 x 127, 5.8 L; largest cell Ø110; rods 18.7 off the swept disks.
- **Links, 14 rods** (46..87):
  - spokes from under the motors to own-side nodes: 2 per front arm, 1 per rear arm (no other own-side node is clear of the disks);
  - 4 bay ties (receiver nose and tail corners to the bottom nodes, review 27 Oct: both ends tied);
  - 1 tail core tie;
  - the camera guard bar over the camera, behind the lens, with 1 strut a side.
  - Closest pair 1.5 between rod surfaces.
- Props shadow 6.4 % above, 10.7 % below (E).
- **Strength is not yet checked: the frame model decides.**
  - Hand estimate (E, pinned ends, 3.0 rod): a 50 N handling load mid-span on the longest rod (211) reaches ~1.0 GPa against 0.75 allowable. Euler buckling there is ~100 N.
  - So the frame model will likely ask for a few long rods to be thicker (CLAUDE.md rod rule: thicker only where the frame model needs it), or one more node pair.
  - The cage-first margin (7.4) is geometric only.

## Camera (one), over the nose, outside the cage
DECXIN 2633V2 landscape (89 x 73, E), horizon, lens (144, 0, 70), nothing of the drone in its view (C).
- **Why up there:** placement is searched over lens height 70..90 and back-plane tilt 35 / 50, with the cage 14 under its lower view plane and behind its tilted back. Low on the nose (lens z 30..46), the camera group's forward moment left pack in and pack out 5.8 mm apart, more than the 3 mm gate allows from the catch shift alone. At z 70 it sits close enough to balance.
- **Mount:**
  - 12 x 12 PETG boom, 78 long, from the plate nose to the camera seat, 3.7 clear of the shell rods;
  - guard bar over its top, behind the lens;
  - the boom root lug is the printed fuse (requirement for the frame model).
- **Cage first:** 7.4 outside its 55° field cone. It stands 4.4 out along its axis.

## Balance
The catch set slides 12.0 aft of the thrust centre (plate centre), searched 0..24. The catch set is the receiver, cartridge, pack and belly mount with the FC, Radxa and flow. The UBEC sits 30 further aft on the mount's tail (trim, DEFAULT).
- CG 2.4 / 2.4 mm pack in / out (layout model), 2.9 / 1.6 (system_check, gate 3): tight.
- FC IMU offset x −12.0 (PX4).

## Mass (E)
663 g: cage 115 (shell rods 62, nodes 39, links 11, guard nodes 3); bought, latch set and wires ~505. Cage v1: 933 g. T/W not checked (motor KV not recorded).

## Access
8 / 8:
- swap down: cartridge, pack, plug;
- 4 receiver bolt keys from below;
- FC USB-C cable out the tail-left through a cell.

## Next
Frame model on this cage: crash 35 g in 14 directions incl. a camera hit, landing on the 4 bottom nodes, handling 50 N. It sizes the long rods. Then the pieces: nodes, spoke clamps, camera boom + seat + guard, belly mount (receiver 12.0 aft, UBEC at the tail). The reverse funnel comes from the design center / funnel sessions.
