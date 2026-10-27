# Scout v3 layout v3 / cage v2, race-car shape, bottom on the Overseer v5 loop, flat tail bay (27 Oct 2026, concept)

**What changed from `v10/` (PR 131):** the design center's ramp check (27 Oct) found the Scout lands ±25 sideways and ±20 on the diagonals but only ±5 fore-aft.
- The limit was the cage's tail low node (−156.4, 0, −35.2): landing 10 toward the tail, it rode into the Overseer's tail-tip V ramp.
- That node was there only because the Radxa stood on edge in the tail bay, hanging to z −27.
- **Fix (DEFAULT):** the Radxa lies flat (65 x 30 x 3.2) under the UBEC, x −136.9..−71.8, z 4.8..8.0. The cage needs no low tail node now.
  - Every node past x −130 about the catch sits ≥ 91 above the bottom plane, against the ≥ 45 the ramp check asked for.
  - The tail bay contents and their mount belong to the electronics session; this is the layout's placement for them.
- **Alternative weighed:** the Radxa standing, raised ≥ 32 so its bottom clears z −4.6. Its top would then reach z 35, 2 under the rear prop sweep (the disks reach y ±9.6 of the centre plane), against the 12 the props need: out.
- **Cost:** 3 fewer shell rods (41), 6 g lighter, and the balance is tighter: CG 2.7 / 1.3 (system_check, gate 3), catch 10.0 aft.

What changed from `v9/` (PR 127), carried over from v10:
- **Bottom:** Spark 27 Oct, relayed by the design center session: the Scout's bottom matches "the rectangle part" of the current Overseer base loop. That is the 4 side nodes of the Overseer cage v5 hexagon (PR 122, not merged), refitted now.
- **Funnel band:** design center check 27 Oct. No link may sit within 30 above the bottom plane, inside the opening or up to 40 past its nose and tail ends, where the ramps reach; shell nodes and rods only where the ramp check clears them. So the bay ties came off the ring nodes.
- **Seat height:** the Scout's plane sits 2.12 further under its cartridge, so both cartridges stop at the same height on the shared ramps (DEFAULT; sign from the design center).
- **No rods on the arms (Spark 27 Oct):** "the rods to the arms need to be on the cartridge on the underside because the arms are just carbon fiber". The spokes from under the motors are gone, and the cage ties to the receiver instead.
  - Read as the underside round the receiver (DEFAULT). The cartridge itself drops out at every swap, so a rod on it would block the swap.
  - The ties root on a separate printed **tie saddle** beside the receiver, bolted to the plate (review 27 Oct): the latch parts stay out of the load path, and the saddle can be replaced.
  - A waist pair of shell nodes is added at mid-side for those ties.

Everything else is as in v9 (`v9/summary.md`, Spark's asks in order): the race-car shape, every rod 3 mm, no small triangles, the camera close in at the nose on a PETG cradle snap-clipped onto the cage rods, the tail bay, the Bee35 base, one camera.

Script `nox/scout/v3/layout_v3.py` (defaults: ring v5side, pattern free with the race silhouette, saddle ties, the funnel band, camera clips, tail bay with the Radxa flat), view `layout_v3_view.py`. Drone meshes `runs/scout_layout/v6/drone/`. The Overseer cage v5 is copied in as `overseer_cage_v5_shell.json` (ghost only, `SOURCE.txt`). Numbers `nox/dimensions.md` "System Scout v3". Rule: CLAUDE.md (Cages: the Scout clause), `docs/generative.md` 0 (Shared ring and the Scout), check `tools/review_precheck.py` 13.

Visuals:
- Whole drone: https://claude.ai/artifact/WHdjYkzKPifXuHT1dWeqoR (version 15; source `motion.html`; orange = the bottom loop, light blue = the Overseer cage v5 on the shared datum, green = the camera view). Still `preview.png`.
- Camera clips: https://claude.ai/artifact/3nsfgsnE6tmctThfu9HZqs (version 2; source `camera_clips.html`, still `camera_clips.png`; the clips are unchanged from v10).

System: Scout v3, cage first 4.2 mm (E) at (+0.55, -0.07, +0.83) (camera) (field cones open: camera 55 deg, out 2 mm), shell 41 rods closed, fit 0.0 mm³ (E), bbox 11.6 L (E), extents 352 x 260 x 127 mm (E), mass 673 of 500 g (135 %) (E) over budget (total), CG 2.7 mm off thrust axis (E), 1.3 mm without swap parts, probes 8.5 mm (E) edge (Radxa), access 8/8 clear. PASS.

## Bottom: the Overseer cage v5 loop's 4 side nodes
- **The 4 nodes:** (+85.8, ±116.1) and (−68.9, ±119.7) about the catch (C, from the Overseer cage v5 `nox/overseer/cage_v5_shell.json`, PR 122 @ 93e39539). The loop is 154.7 long, 232.2 wide at the nose and 239.4 at the tail; the Overseer's nose and tail nodes are dropped.
- **Datum:** the same rule as the Overseer: catch centre on the grip line, plane 3.0 under the cartridge flat bottom.
- **Seat (DEFAULT):** the Scout plane sits (3.0 − 1.5)·√2 = 2.12 further under its cartridge.
  - On a 45° ramp a rod's centre sits r·√2 above the face, so the 3.0 ring rides 2.12 lower than the Overseer's 6 mm tube; this lands both cartridges at the same height. (A first push had the sign reversed; the design center caught it.)
  - Confirmed by the design center: the ring rods bear, and every ramp stops 20 short of the Ø20 nodes. (Had the nodes borne, the offset would be about 0.2.)
  - The alternative is 6 mm ring tubes, which the Scout rule (every rod 3 mm) does not allow.
  - Scout: tab line x −10.0, cartridge flat bottom z −44.5, plane z −49.6; nodes Ø20, rods 3.0.
- **Funnel band:** within 30 above the plane, inside the opening or up to 40 past its ends.
  - No link is in it (the rule enforced in the code, `in_band`).
  - Shell nodes and rods may enter it only where the funnel's ramp check clears them (`docs/generative.md`); this run's are listed below for that check.
  - The 14 shell rods leaving the 4 bottom nodes rise through it from the plane. 2 shell rods between free nodes reach into it, at the nose low node (100.9, 0, −22.4): 27 above the plane, 24 past the nose end, clear of the nose V (design center). There is no tail low node now.
  - The design center's ramp check decides whether any of these must move.
- **Swap column:** receiver + 26.5 at the nose end, + 17.6 at the tail, + 30 at the sides; clear (0 mm³).

## Shape: race car
- Flat roof just over the props, a raked front behind the camera, a raised tail point (DEFAULT, the look), tapering to the tail.
- Node extents 323 x 239 x 111; whole drone 352 x 260 x 127 (camera included). v9 was 350 x 285 x 125: the loop's own width now sets the cage's sides.
- **Offset scan,** all 5 valid:

  | offset | nodes | shell rods | L |
  |---|---|---|---|
  | 16 | 16 | 41 | 6.4 |
  | 20 | 20 | 53 | 6.0 |
  | 24 | 18 | 47 | 6.1 |
  | 28 | 18 | 47 | 6.2 |
  | 32 | 16 | 41 | 6.0 |

  The most compact shell with no small or sliver triangle (every rod ≥ 45), within the rod cap and balanced on its own, wins: offset 32.
- **Waist pair:** (−10.0, ±119.7, −9.6), over the catch on the cage's sides, 40 above the bottom plane, so 10 above the funnel band. The saddle ties land there.
- **Small triangles merged to one node (Spark):** shell rods 65..268.

## Rods: 47, all 3.0 solid carbon (v10: 50, v9: 54, cage v1: 123)
- **Shell, 41 rods** (5.51 m) on 16 nodes: the 4 bottom nodes Ø20 and 12 printed Ø12 nodes for 3.3 sockets. Largest cell Ø139. Rods 13.6 off the swept disks.
- **Links, 6 rods, all from the tie saddle** (no rod on the carbon arms, Spark). All stay out of the funnel band and over the 33 break-even.
  - Each runs from the saddle's outer face (y ±44.3, z −7, 23 above the band) to a shell node on its side:
    - tail station (x −45) to the waist node, 83 long;
    - middle (x −15) to the front side node (126.0, ±64.8, 4.9), 143 long;
    - nose (x 38) to the waist node, 89 long.
    - The 4th station (x 15) found no clear rod.
  - No tie roots on the receiver, its TPU end pieces or the belly mount.
  - Closest pair 1.4 between rod surfaces. Mirror-exact, closed outside the bottom.
- **Tie saddle (printed PETG, DEFAULT 27 Oct; review: the replaceable part, not the latch), 17 g E:**
  - Shape: a rail a side, x −50..42, y ±36.3..42.3, z −15..6, outside the belly mount (y ±35) and the receiver's TPU end pieces (y ±35.8). Two tabs a side run up to the plate underside between the arms (x ±14..26).
  - Fixing: 4 M3 into the plate. This needs 4 holes in the Bee35 plate at the tabs; the frame model checks them against the arms.
  - The 6 tie roots sit on its outer face. It is the fuse, ahead of the latch.
  - Alternative: clamp it under the receiver's 4 bolts, which needs the latch stack to change (latch / electronics sessions).
- **Alternative weighed (v10 run): core ties on the plate underside edges** (`../v10/alt_core_ties/`, `SCOUT_TIES=core`; no waist pair, no saddle): 42 rods (38 + 4), 651 g, CG 1.8 / 1.8 (layout).
  - It is lighter by 28 g and 8 rods.
  - But its roots sit on bare carbon plate edges, which Spark ruled out for the arms ("just carbon fiber").
  - The saddle buys a printed root that can be replaced and sized.
- **Strength is not checked yet; the frame model decides.**
  - The longest rod is 268: 50 N mid-span ~1.2 GPa (E, pinned) against 0.75 allowable.
  - Load path: the cage reaches the plate through the 6 saddle ties, the saddle and its 4 plate bolts. The frame model sizes the saddle and the bolts (crash 35 g x ~680 g ≈ 230 N, E).
- Props shadow 3.8 % above, 8.8 % below (E).

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
- **Tail bay (DEFAULT):** the Radxa (flat, z 4.8..8.0) and the UBEC (z 10..18) sit behind the receiver's tail, x −137..−72.
- **Catch set:** 10.0 aft of the thrust centre. The FC IMU offset is x −10.0 (PX4).
- **CG:** 2.0 / 2.2 mm pack in / out (layout model), 2.7 / 1.3 (system_check, gate 3). This is tighter than v10 (2.4 / 0.9): the Radxa moved up, and the catch moved 2 further aft to balance it.

## Mass (E)
673 g: cage ~138 (shell rods 61, nodes 36, links 7, tie saddle 17, camera cradle + arms + clips 17); bought, latch set and wires ~505.

## Access
8 / 8:
- swap down: cartridge, pack, plug;
- 4 receiver bolt keys from below;
- FC USB-C cable out the tail-left through a cell (4° off the diagonal; the links were drawn first and the cable routed round them).

## Open
- **Funnel:** the design center reruns its ramp check: the fore-aft tolerance should rise from ±5 to about ±20 (its estimate).
- **Overseer side:** PR 122 must merge. Until then the 4 nodes are copied from its branch; if the Overseer loop moves, this refits.
- **Frame model:** crash 35 g in 14 directions incl. a nose-camera hit, landing on the 4 bottom nodes, handling 50 N. It sizes the long rods, the saddle and its plate bolts, and the camera mount's mode.
