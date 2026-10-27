# Scout v3 layout v3 / cage v2, race-car shape, bottom on the Overseer v5 loop (27 Oct 2026, concept)

What changed from `v9/` (PR 127):
- **Bottom:** Spark 27 Oct, relayed by the design center session: the Scout's bottom matches "the rectangle part" of the current Overseer base loop. That is the 4 side nodes of the Overseer cage v5 hexagon (PR 122, not merged), refitted now.
- **Funnel band:** design center check 27 Oct. Nothing but the ring rods may sit within 30 above the bottom plane, inside the opening or up to 40 past its nose and tail ends, where the ramps reach. So the bay ties came off the ring nodes.
- **Seat height:** the Scout's plane sits 2.12 further under its cartridge, so both cartridges stop at the same height on the shared ramps (DEFAULT; sign from the design center).

Everything else is as in v9 (`v9/summary.md`, Spark's asks in order): the race-car shape, every rod 3 mm, no small triangles, the camera close in at the nose on a PETG cradle snap-clipped onto the cage rods, the tail bay, the Bee35 base, one camera.

Script `nox/scout/v3/layout_v3.py` (defaults: ring v5side, pattern free with the race silhouette, lean links, the funnel band, camera clips, tail bay), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/`. The Overseer cage v5 is copied in as `overseer_cage_v5_shell.json` (ghost only, `SOURCE.txt`). Numbers `nox/dimensions.md` "System Scout v3". Rule: CLAUDE.md (Cages: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Visuals:
- Whole drone: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 12; source `motion.html`; orange = the bottom loop, light blue = the Overseer cage v5 on the shared datum, green = the camera view). Still `preview.png`.
- Camera clips: https://claude.ai/artifact/3nsfgsnE6tmctThfu9HZqs (version 2; source `camera_clips.html`, still `camera_clips.png`).

System: Scout v3, cage first 4.2 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 2 mm), shell 38 rods closed, fit 0.0 mm³ (E), bbox 11.6 L (E), extents 350 x 260 x 127 mm (E), mass 653 of 500 g (131 %) (E) over budget (total), CG 2.3 mm off thrust axis (E), 0.9 mm without swap parts, probes 9.9 mm (E) edge (receiver v3), access 8/8 clear. PASS.

## Bottom: the Overseer cage v5 loop's 4 side nodes
- **The 4 nodes:** (+85.8, ±116.1) and (−68.9, ±119.7) about the catch (C, from the Overseer cage v5 `nox/overseer/cage_v5_shell.json`, PR 122 @ 93e39539). The loop is 154.7 long, 232.2 wide at the nose and 239.4 at the tail; the Overseer's nose and tail nodes are dropped.
- **Datum:** the same rule as the Overseer: catch centre on the grip line, plane 3.0 under the cartridge flat bottom.
- **Seat (DEFAULT):** the Scout plane sits (3.0 − 1.5)·√2 = 2.12 further under its cartridge. On a 45° ramp a rod's centre sits r·√2 above the face, so the 3.0 ring rides 2.12 lower than the Overseer's 6 mm tube; this lands both cartridges at the same height. (A first push had the sign reversed; the design center caught it.)
  - The alternative is 6 mm ring tubes, which the Scout rule (every rod 3 mm) does not allow.
  - Scout: tab line x −7.5, cartridge flat bottom z −44.5, plane z −49.6; nodes Ø20, rods 3.0.
- **Funnel band:** within 30 above the plane, inside the opening or up to 40 past its ends.
  - No link is in it.
  - The 14 shell rods leaving the 4 bottom nodes rise through it from the plane. 2 shell rods between free nodes reach into it: the nose and tail low nodes, (102.2, 0, −22.4) and (−155.4, 0, −35.2), the ones the design center found clear when seated.
  - The design center's ramp check decides whether any of these must move.
- **Swap column:** receiver + 26.5 at the nose end, + 17.6 at the tail, + 30 at the sides; clear (0 mm³).

## Shape: race car
- Flat roof just over the props, a raked front behind the camera, a raised tail point (DEFAULT, the look), tapering to the tail.
- Node extents 321 x 239 x 111; whole drone 350 x 260 x 127 (camera included). v9 was 350 x 285 x 125: the loop's own width now sets the cage's sides.
- **Offset scan,** all 5 valid:

  | offset | nodes | shell rods | L |
  |---|---|---|---|
  | 16 | 15 | 38 | 6.6 |
  | 20 | 19 | 50 | 6.1 |
  | 24 | 18 | 47 | 6.2 |
  | 28 | 17 | 44 | 6.4 |
  | 32 | 15 | 38 | 6.2 |

  The most compact shell with no small or sliver triangle (every rod ≥ 45), within the rod cap and balanced on its own, wins: offset 32.
- **Small triangles merged to one node (Spark):** shell rods 62..269.

## Rods: 44, all 3.0 solid carbon (v9: 54, cage v1: 123)
- **Shell, 38 rods** (5.44 m) on 15 nodes: the 4 bottom nodes Ø20 and 11 printed Ø12 nodes for 3.3 sockets. Largest cell Ø140. Rods 13.6 off the swept disks.
- **Links, 6 rods.** Every one stays out of the funnel band, so none lands on a bottom node now. A rod's carbon is justified by its length (all over the 33 break-even).
  - Front spokes: from under each front motor to the side node (126.0, ±64.8, 4.9), 1 per arm.
  - Rear spokes: from under each rear motor to the tail low node (−155.4, 0, −35.2), 1 per arm.
  - Nose bay ties: from the receiver's nose corners, 32 above the plane, up to the front-top node (117.7, 0, 33.2).
  - **No tail bay tie:** every node it could reach crosses the tail bay or comes within 12 of a prop disk. The rear spokes into the tail node carry the tail end, and the frame model checks that path.
  - No core tie, no carbon to the camera.
  - Closest pair 2.2 between rod surfaces. Mirror-exact, closed outside the bottom.
- **Strength is not checked yet; the frame model decides.** The longest rod is 269: 50 N mid-span ~1.2 GPa (E, pinned) against 0.75 allowable. With fewer links the core path is thinner than in v9; the frame model sizes it.
- Props shadow 3.8 % above, 8.0 % below (E).

## Camera (one), close in at the nose, snap-clipped onto the cage
- DECXIN 2633V2 landscape, horizon, lens (149, 0, 42). Nothing of the drone is in its view (C).
- PETG cradle 0.5 clear round it, lips 6 over and under (11.5 g E).
- 4 Ø9 printed arms into 4 PETG snap clips. Each clip is 12 long, OD 9.3, bore 3.1, with a 2.6 mouth facing up, pushed onto its rod after the cage is built.
  - Top pair on the top front cross rod (220 long, t 0.58) at (109.3, ±18.0, 61.5), arms 18.
  - Bottom pair on the front rods (71 long, t 0.68) at (120.3, ±20.5, 24.2), arms 7.
  - That gives 4 non-collinear supports. Arms and clips are 0 mm³ on every rod.
- The clips are the fuse: they pop off before the camera breaks and snap back on. TPU washers go under the board (DEFAULT).
- **Requirement for the piece:** the clip grips the rod so it cannot rattle, turn or slide: fingers about 2.9 to 3.0, or a TPU liner, plus an end stop.
- **First mode:** ~70 to 140 Hz (E). The top clips sit near the middle of a 220 rod, so the frame model decides. If it reads under 100 Hz, move the top clips toward that rod's ends.
- Cage first 4.2 over the cradle's top lip, outside the 55° field cone.

## Balance (tail bay)
- **Tail bay (DEFAULT):** the Radxa and the UBEC sit behind the receiver's tail.
- **Catch set:** 7.5 aft of the thrust centre. The FC IMU offset is x −7.5 (PX4).
- **CG:** 1.6 / 1.8 mm pack in / out (layout model), 2.3 / 0.9 (system_check, gate 3).

## Mass (E)
653 g: cage ~117 (shell rods 60, nodes 34, links 6, camera cradle + arms + clips 17); bought, latch set and wires ~505.

## Access
8 / 8:
- swap down: cartridge, pack, plug;
- 4 receiver bolt keys from below;
- FC USB-C cable out the tail-left through a cell (17° off the diagonal; the links were drawn first and the cable routed round them).

## Open
- **Funnel:** the design center reruns its ramp check on this cage. The 2 nose / tail low nodes are what to look at (the tail one near the Overseer's tail-tip ramp).
- **Overseer side:** PR 122 must merge. Until then the 4 nodes are copied from its branch; if the Overseer loop moves, this refits.
- **Frame model:** crash 35 g in 14 directions incl. a nose-camera hit, landing on the 4 bottom nodes, handling 50 N. It sizes the long rods, the thinner core path (6 links) and the camera mount's mode.
