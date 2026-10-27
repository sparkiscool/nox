# Scout v3 layout v3 / cage v2, race-car shape: the Scout design (27 Oct 2026, concept)

Spark 27 Oct, in order:
- On cage v1 (`v6/`, 123 rods): "these are all 3mm ... divide the number of rods by half"
- "dont worry about same rod legnth anymore" (`v7/`)
- "get rid of the two trapezoids so the bottom is just a rectangle ... a reverse funnel that works for both and the scout can just use less ramps"; "just choose one design" (`v8/`)
- On v8: "looks too dumb, i want a race car look the overseer looks cooler, and also you can definitly lower the camera"
- "reduce small triangles to one node"
- "get rid of that shap section under the camera"
- "just let the camera mount to the cage, its light and dosent need the support from the drone body"

This is the one design. Unchanged: the PR 94 drone, one camera, the Bee35 carbon base, the rectangle bottom.

Script `nox/scout/v3/layout_v3.py` (defaults: ring rect, pattern free with the race silhouette, lean links, camera low, tail bay), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/`. Numbers `nox/dimensions.md` "System Scout v3". Rule: CLAUDE.md (Cages: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 7; source `motion.html`; orange = the bottom rectangle, light blue = the Overseer cage on the same datum, green = the camera view). Still `preview.png`.

System: Scout v3, cage first 4.9 mm (E) at (+0.57, -0.05, -0.82) (camera) (field cones open: camera 55 deg, out 2.55 mm), shell 47 rods closed, h/w 0.34, fit 0.0 mm³ (E), bbox 13.1 L (E), extents 376 x 278 x 125 mm (E), mass 666 of 500 g (133 %) (E) over budget (total), CG 2.8 mm off thrust axis (E), 1.3 mm without swap parts, probes 7.1 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## Shape: race car
- A low flat roof just over the props: the drone's margin hull, grown, is clipped at the drone's own top. The cage stands 109 between node centres, 125 overall (v8: 160 with the camera on top).
- A clean raked front face behind the camera. The low nose point is gone (Spark: no sharp section under the camera).
- A raised tail point, 55 behind the hull and 18 under its top (DEFAULT, the look). It costs 54 of length: the tail node sits at x −198, the next-most-aft node at −144. Without it the cage would be ~54 shorter; the frame model may drop it if the tail path is weak.
- In plan the cage tapers to the tail; 375 x 278 overall.
- **How the nodes are placed:**
  - corners of the margin hull grown 24 mm along 6 directions, plus the tail point;
  - added farthest-outside first in mirror pairs until the hull holds the drone;
  - scan of offsets 16..32: 16 stands 117 tall; 24..32 give 18 nodes / 47 rods at 109;
  - the most compact shell with no small or sliver triangle (every rod ≥ 45), within the rod cap, wins: offset 24. Offset 20 is more compact (5.7 L) but has 53 shell rods, over the cap (49).
- **Small triangles to one node (Spark):** during the build, a hull triangle with every edge under 70, or a mirror pair closer than 70, collapses to one node (a mirror pair onto the centre plane). Shell rods are 63..209; no triangle has all edges under 70.

## Rods: 62, all 3.0 solid carbon (cage v1: 123)
- **Shell, 47 rods** (5.92 m) on 18 nodes: the 4 bottom nodes Ø20, and 14 printed Ø12 nodes for 3.3 sockets.
- **Links, 15 rods** (46..87):
  - spokes from under the motors: 2 per front arm, 1 per rear arm (69..87);
  - 4 bay ties, receiver nose and tail corners to the bottom nodes (75, 77);
  - the camera cradle's 2 low struts into the shell (51);
  - the guard bar over the camera, behind the lens (46), with 1 strut a side into the shell (73).
- No core tie (lean set, to hold half the rods): the plate ties to the cage through the 6 spokes and the 4 bay ties. The frame model checks that path, and the tail point's path to the core.
- Closest pair 1.2 between rod surfaces.
- Mirror-exact, closed outside the bottom. Largest cell Ø95. Rods 14.5 off the swept disks.
- Props shadow 3.0 % above, 9.5 % below (E).
- **Strength is not yet checked: the frame model decides.** Hand estimate (E, pinned, 3.0 rod): 50 N mid-span on the longest rod (209) reaches ~1.0 GPa against 0.75 allowable; Euler ~100 N. Some long rods will likely go thicker (CLAUDE.md rod rule), or the frame model adds a node.

## Camera (one), low in the nose, outside the cage, hung from the cage
DECXIN 2633V2 landscape (89 x 73, E), horizon, lens (172, 0, 30). Nothing of the drone is in its view (C).
- **Placement:** searched over lens z 30..46 and back-plane tilt 35 / 50, with the cage 14 under its lower view plane and behind its back.
- **Mount (Spark: "just let the camera mount to the cage"):** a printed cradle, all behind the lens and out of view. No boom to the plate, no arm braces.
  - The cradle is a back plate and two side cheeks round the camera (x 154..170, z 4..57).
  - A 4 mm chin lip sits 3 under the camera.
  - The guard bar's 2 printed nodes sit on its top.
  - It is tied to the shell by the 2 guard struts (top) and 2 cradle struts (low corners): triangulated on the cage, not a stalk.
- **Cage first:** worst 4.9, from below-forward on the chin lip, outside the 55° field cone. The camera stands 2.6 out along its axis.

## Bottom: the Overseer ring's middle rectangle
- 4 of the Overseer ring's 8 nodes, (±72.9, ±104.3) about the catch centre, 145.8 x 208.7, on the same datum: catch centre on the grip line, 3.0 under the cartridge flat bottom.
- Scout: tab line x −10.5, plane z −47.5, nodes Ø20, rods 3.0.
- One reverse funnel serves both drones; the Scout bears on fewer ramps (design center told, 27 Oct).
- Above the bottom, the Scout's rods stay inside the Overseer's shell: 0.3 at 0..20, 17.8 at 20..40, 34.6 at 40..60, 34.4 at 60..80 (C).
- **Swap column:** the rectangle leaves the receiver + 13.7 at the nose end and + 21.6 at the tail (`checks.txt` swap zone; the + 30 DEFAULT gives way to Spark's rectangle), + 30 at the sides; clear (0 mm³). Box arm and catch sessions: check the head and grabber against these.

## Balance (tail bay)
- **Tail bay (DEFAULT 27 Oct, for the low nose camera):** the Radxa and the UBEC ride behind the receiver's tail on the belly mount, their front ends 13 behind it (Radxa x −137..−72, UBEC −117..−72). The alternative is the camera over the nose (v8), which Spark asked to lower.
- **Catch set:** 10.5 aft of the thrust centre (plate centre), searched 0..24 then refined. It is the receiver, cartridge, pack and belly mount with the FC and flow.
- **CG:** 2.0 / 2.4 mm pack in / out (layout model), 2.8 / 1.3 (system_check, gate 3).
- FC IMU offset x −10.5 (PX4).

## Mass (E)
666 g: cage ~125 (shell rods 65, nodes 39, links 12, guard nodes 3, cradle + chin ~6); bought, latch set and wires ~505. Cage v1: 933 g. T/W not checked (motor KV not recorded).

## Access
8 / 8:
- swap down: cartridge, pack, plug;
- 4 receiver bolt keys from below;
- FC USB-C cable out the tail-left through a cell.

## Next
- Frame model on this cage: crash 35 g in 14 directions, incl. a nose-camera hit; landing on the 4 bottom nodes; handling 50 N. It sizes the long rods and checks the core path without a core tie.
- Then the pieces: nodes, spoke clamps, the camera cradle (with chin and guard nodes), belly mount with the tail bay (receiver 10.5 aft of the plate centre).
- The reverse funnel: design center / funnel sessions.
