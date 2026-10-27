# Scout v3 layout v3 / cage v2, race-car shape: the Scout design (27 Oct 2026, concept)

Spark 27 Oct, in order:
- On cage v1 (`v6/`, 123 rods): "these are all 3mm ... divide the number of rods by half"
- "dont worry about same rod legnth anymore" (`v7/`)
- "get rid of the two trapezoids so the bottom is just a rectangle ... a reverse funnel that works for both and the scout can just use less ramps"; "just choose one design" (`v8/`)
- On v8: "looks too dumb, i want a race car look the overseer looks cooler, and also you can definitly lower the camera"
- "reduce small triangles to one node"
- "get rid of that shap section under the camera"
- "just let the camera mount to the cage, its light and dosent need the support from the drone body"
- "it should be mounted not on rods but on printed petg or tpu if thats better"; "also let the camera close to the drone, making it as compact as possible"
- "make the camera mount clip into the rods and not the rods have to go through it so i can put it on after cage is done"

This is the one design. Unchanged: the PR 94 drone, one camera, the Bee35 carbon base, the rectangle bottom.

Script `nox/scout/v3/layout_v3.py` (defaults: ring rect, pattern free with the race silhouette, lean links, camera low, tail bay), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/`. Numbers `nox/dimensions.md` "System Scout v3". Rule: CLAUDE.md (Cages: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Visual: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 10; source `motion.html`; orange = the bottom rectangle, light blue = the Overseer cage on the same datum, green = the camera view). Still `preview.png`. Camera clips: https://claude.ai/artifact/3nsfgsnE6tmctThfu9HZqs (source `camera_clips.html`, still `camera_clips.png`).

System: Scout v3, cage first 4.2 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 2 mm), shell 44 rods closed, h/w 0.34, fit 0.0 mm³ (E), bbox 12.5 L (E), extents 350 x 285 x 125 mm (E), mass 662 of 500 g (132 %) (E) over budget (total), CG 2.3 mm off thrust axis (E), 1.0 mm without swap parts, probes 5.7 mm (E) edge (cartridge v6), access 8/8 clear. PASS.

## Shape: race car
- A low flat roof just over the props: the drone's margin hull, grown, is clipped at the drone's own top. The cage stands 109 between node centres, 125 overall (v8: 160 with the camera on top).
- A clean raked front face behind the camera. The low nose point is gone (Spark: no sharp section under the camera).
- A raised tail point, 55 behind the hull and 18 under its top (DEFAULT, the look). It costs 54 of length: the tail node sits at x −197, the next-most-aft node at −143. Without it the cage would be ~54 shorter; the frame model may drop it if the tail path is weak.
- In plan the cage tapers to the tail; 352 x 285 overall (camera included).
- **How the nodes are placed:**
  - corners of the margin hull grown 24 mm along 6 directions, plus the tail point;
  - added farthest-outside first in mirror pairs until the hull holds the drone;
  - scan of offsets 16..32, all 5 valid (review 27 Oct: a merge whose push runs up the camera's limit plane is reverted and that triangle kept):
    - 16: 44 rods but a 30 mm sliver rod (under the 45 minimum);
    - 20 and 24: 53 and 62 shell rods (over the 52 cap) and 2 small triangles each;
    - 28 and 32: 17 nodes / 44 rods, 5.9 and 6.1 L;
  - the most compact shell with no small or sliver triangle (every rod ≥ 45), within the rod cap (62 − 10 links = 52), wins: offset 28.
- **Small triangles to one node (Spark):** during the build, a hull triangle with every edge under 70, or a mirror pair closer than 70, collapses to one node (a mirror pair onto the centre plane). Shell rods are 66..264; no triangle has all edges under 70.

## Rods: 54, all 3.0 solid carbon (cage v1: 123)
- **Shell, 44 rods** (5.77 m) on 17 nodes: the 4 bottom nodes Ø20, and 13 printed Ø12 nodes for 3.3 sockets.
- **Links, 10 rods** (71..87): spokes from under the motors, 2 per front arm and 1 per rear arm; 4 bay ties, receiver nose and tail corners to the bottom nodes (75, 77). No carbon to the camera (printed arms, below). No core tie: the plate ties to the cage through the 6 spokes and the 4 bay ties; the frame model checks that path and the tail point's path to the core.
- Closest pair 1.4 between rod surfaces. Mirror-exact, closed outside the bottom. Largest cell Ø135. Rods 13.3 off the swept disks.
- Props shadow 3.7 % above, 9.3 % below (E).
- **Strength is not yet checked: the frame model decides.** Hand estimate (E, pinned, 3.0 rod): 50 N mid-span on the longest rod (264) reaches ~1.2 GPa against 0.75 allowable; Euler ~65 N. Some long rods will likely go thicker (CLAUDE.md rod rule), or the frame model adds a node.

## Camera (one), close in at the nose, on printed PETG
DECXIN 2633V2 landscape (89 x 73, E), horizon, lens (149, 0, 42). Nothing of the drone is in its view (C).
- **Placement (Spark: "as compact as possible"):** as close to the drone as its clear view allows, with its cradle no higher than the cage roof. Searched over lens z 30..50 and back-plane tilt 0..50, with the cage 14 under its lower view plane and behind its back. The lens is 23 closer than v9's first pass (172), and the drone 24 shorter (352).
- **Mount (Spark: "not on rods but on printed petg or tpu"):** all printed PETG, no carbon, nothing to the drone body.
  - A cradle frame round the camera, 0.5 clear of its body (fit 0): back plate, two cheeks, a top lip and a chin lip 6 over and under it, all behind the lens (x 131..147, z 17..67, 11.5 g E).
  - 4 printed round Ø9 arms (64 mm², as an 8 x 8 post) from the cradle's back corners into the cage, 2.5 g E. Each corner takes the shortest clear way in:
    - into a node, if every rod there stays clear of the arm outside the node + 2;
    - else into a printed snap clip on a shell rod (Spark: "clip into the rods ... so i can put it on after cage is done"): a PETG C, 12 long, OD 9.3, bore 3.1 on the 3.0 rod, mouth 2.6 that snaps over it, 2.5 g E for all 4.
  - **Picked: 4 snap clips.** No node near the cradle has its rods leaving at a clear angle. The old bottom arms ran 15° off two rods and through them (review 27 Oct).
    - The top corners go to the top front cross rod at (109.4, ±18.0, 61.5), 18 long.
    - The bottom corners go to the front rods at (119.4, ±21.2, 25.9), 9 long.
    - That gives 4 non-collinear supports, so the cradle cannot pivot.
  - **Fitting:** build the whole cage first, then push the 4 clips onto their rods. No rod goes through the mount.
  - **Mouth:** it faces up and across the arm, so the camera's weight and the arm's push and pull bear on the clip's closed side.
  - **Clearance:** every arm is clear of every rod and link (0 mm³), and every clip is clear of its rod (bore 3.1 on 3.0, 0 mm³). The snap fingers' section and the snap force are set in the piece (PETG, E). Requirement for the piece (review 27 Oct): the clip grips the rod so it cannot rattle, turn or slide (fingers about 2.9 to 3.0, or a TPU liner, plus an end stop); the frame model's mode run includes that fit.
  - **Why PETG and not TPU:** the camera mount must stay ≥ 100 Hz (CLAUDE.md design list 6) and hold the view still; TPU arms would let it swing. TPU goes only as washers under the camera board screws, for vibration (DEFAULT).
  - **Fuse:** the clips are the fuse. In a hard hit they pop off their rods before the camera breaks, and they snap back on (requirement for the piece).
  - **First mode, not computed yet (frame model): ~70 to 140 Hz (E), across the 100 Hz gate.**
    - The arms alone would give ~360 Hz. The rods' own give sets the mode: the top clips sit near the middle of the 210 long top front cross rod (t 0.41 / 0.59), and the cradle pitches about the bottom clips on that rod's bending (review hand estimate).
    - If the frame model reads under 100 Hz, the cheapest fix is to move the top clips toward that rod's end nodes or onto a shorter rod.
- **Cage first:** worst 4.2, over the camera on the cradle's top lip, outside the 55° field cone. The camera stands 2.0 out along its axis.

## Bottom: the Overseer ring's middle rectangle
- 4 of the Overseer ring's 8 nodes, (±72.9, ±104.3) about the catch centre, 145.8 x 208.7, on the same datum: catch centre on the grip line, 3.0 under the cartridge flat bottom.
- Scout: tab line x −8.0, plane z −47.5, nodes Ø20, rods 3.0.
- One reverse funnel serves both drones; the Scout bears on fewer ramps (design center told, 27 Oct).
- Above the bottom, the Scout's rods stay inside the Overseer's shell: 0.1 at 0..20, 16.6 at 20..40, 33.2 at 40..60 (C).
- **Open input:** the Overseer cage PR 122 (not merged) turns its base loop into a 6-node hexagon. Once it lands, the Scout bottom refits to that loop's 4 side nodes.
- **Swap column:** the rectangle leaves the receiver + 13.7 at the nose end and + 21.6 at the tail (`checks.txt` swap zone; the + 30 DEFAULT gives way to Spark's rectangle), + 30 at the sides; clear (0 mm³). Box arm and catch sessions: check the head and grabber against these.

## Balance (tail bay)
- **Tail bay (DEFAULT 27 Oct, for the low nose camera):** the Radxa and the UBEC ride behind the receiver's tail on the belly mount, their front ends 13 behind it (Radxa x −136..−71, UBEC −116..−71). The alternative is the camera over the nose (v8), which Spark asked to lower.
- **Catch set:** 8.0 aft of the thrust centre (plate centre), searched 0..24 then refined. It is the receiver, cartridge, pack and belly mount with the FC and flow.
- **CG:** 1.6 / 1.9 mm pack in / out (layout model), 2.3 / 1.0 (system_check, gate 3).
- FC IMU offset x −8.0 (PX4).

## Mass (E)
662 g: cage ~126 (shell rods 63, nodes 38, links 8, camera cradle + arms + clips 17); bought, latch set and wires ~505. Cage v1: 933 g. T/W not checked (motor KV not recorded).

## Access
8 / 8:
- swap down: cartridge, pack, plug;
- 4 receiver bolt keys from below;
- FC USB-C cable out the tail-left through a cell.

## Next
- Frame model on this cage: crash 35 g in 14 directions, incl. a nose-camera hit; landing on the 4 bottom nodes; handling 50 N. It sizes the long rods and checks the core path without a core tie.
- Then the pieces: nodes, spoke clamps, the camera cradle + arms + snap clips (with their mode and snap force), belly mount with the tail bay (receiver 8.0 aft of the plate centre).
- The reverse funnel: design center / funnel sessions.
