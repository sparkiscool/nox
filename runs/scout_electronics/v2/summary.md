# Scout v3 electronics mount v2: the mount on the v12 tail-bay poses (27 Oct 2026, concept, round 5)

**Round 5 in one line:**
- **Fit:** the full pairwise fit table passes, 33 / 33, with no contact allowances.
- **Balance passes at v12's catch −5.0: CG 0.56 / 1.49 pack in / out** (`system_check` 0.6 / 1.4, PASS). This is with main's assembly (the camera mount, PR 146) and a 49.2 g mount sized on the L cases.

**What v2 is.** The Scout v3 design session (cage owner) sets the tail-bay poses. Layout v12 (PR 141, on main) puts the catch 5.0 forward of the thrust centre; the Radxa, the UBEC, this mount and the cables ride the belly mount with it. This round fixes round 4's review FAIL (2c00fd72):
- the two pieces overlapped;
- the fit allowances are removed;
- the XR2 floor was inside the XR2;
- the mount was sized on a case that is not L, so balance had no route.

**Superseded.** Rounds 1 to 4 are history: round 3 is 2ef0b72b and round 4 is 2c00fd72. v1 (the fin) is in `../v1/`.

**Files**
- Script `nox/scout/v3/electronics_system.py`. A plain run does this pass in about 15 min (9 variants, then A and B); it is seeded, so one run gives one set of numbers.
- Geometry: `runs/scout_layout/v12/` and `nox/scout/v3/scout.assembly.json` (main, with the camera mount).
  - The layout's 3 placeholder boxes for this mount are dropped.
  - The belly-mount placeholder is cut to its requirement (tail face x −31.0 over |y| ≤ 22).
- Outputs:
  - `result.json` (both concepts, every case, the variants, the fit table), `envelope.json` (for the cage session), `run.log`;
  - `system_check.txt` (A, PASS), `system_check_alt_B.txt`;
  - `preview.png`;
  - the 2 print STLs, `belly_mount_tail_block__petg_.stl` and `tail_carrier__petg_.stl`, both `solid_check` PASS (1 body each).
- Visual: https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU (from `preview.png`).

**Tags:** C = computed on v12's geometry; E = estimate; M = bought-part data; P = measured. CGs are true centroids.

**Mass basis (E):** as printed, 2.4 walls + 20 % infill, for both pieces.
- The interior is found by eroding each mesh 2.4 on a 0.4 grid. The interior volume counts at 20 %; the rest counts solid.
- Thin parts (≤ 4.8) come out solid, thick ones lighter. Solid prints would weigh 19.7 g (block) and 40.5 g (carrier).
- Sections use the same 2.4 walls, with the infill ignored.
- `system_check` takes these masses (`mass_g`).

## Fit table (A; every pair within 10, from `fit_table()`, C)
- **Rule:** a declared bearing contact passes only face to face, with overlap ≤ 0.05 mm³. Every other pair needs ≥ 2.0. Contact allowances are removed: `system_check` gets the declared pairs at 0.05 mm³ and the rest as clearance pairs at 2.0.
- **The fin overlap is fixed:** the fin starts at xb0 + 2.0 (x −46.05), 2.0 ahead of the tongue and fillet ends. The bolt pads are unchanged, 2.5 off the TPU end pieces.

| Pair | Overlap mm³ | Gap | Kind |
|---|---|---|---|
| tail block / plate (web under it, foot on it) | 0.0 | 0.00 | declared contact |
| tail block / belly mount (cut tail face x −31.0) | 0.0 | 0.00 | declared contact |
| tail block / receiver (gusset on its tail face, web on its top) | 0.0 | 0.00 | declared contact |
| tail carrier / tail block (tongues and side wall on its faces) | 0.0 | 0.00 | declared contact |
| tail carrier / Radxa (4 posts on the PCB's back) | 0.0 | 0.00 | declared contact |
| tail carrier / UBEC (tray) | 0.0 | 0.00 | declared contact |
| tail carrier / XR2 (floor, now z 21..23) | 0.0 | 0.00 | declared contact |
| tail block / TPU end pieces | 0.0 | 2.00 | ≥ 2 |
| tail carrier / TPU end pieces | 0.0 | 2.00 | ≥ 2 |
| tail carrier / Radxa plug keep-clear (E) | 0.0 | 2.00 | ≥ 2 |
| tail carrier less its 4 posts / Radxa | n/a | 2.00 | ≥ 2 |
| tail carrier / plate | 0.0 | 2.06 | ≥ 2 |
| tail block / Radxa | 0.0 | 2.40 | ≥ 2 |
| tail carrier / receiver | 0.0 | 4.20 | ≥ 2 |
| tail block / props 3, 4 | 0.0 | 4.48 | ≥ 2 |
| tail block / FC, wires | 0.0 | 4.50 | ≥ 2 |
| tail carrier / prop 4, prop 3 | 0.0 | 5.08, 12.9 | ≥ 2 |
| tail block / TPU strips, cartridge, pack | 0.0 | 6.1, 6.3, 8.8 | ≥ 2 |
| tail block / tie saddle, carrier / tie saddle | 0.0 | 15.7, 29.4 | ≥ 2 |
| tail carrier / belly mount | 0.0 | 16.3 | ≥ 2 |
| tail carrier / cage tubes, nodes | n/a, 0.0 | 16.6, 25.9 | ≥ 2 |
| tail block / plug keep-clear, spokes, cage tubes, nodes | 0.0 | 21.0, 27.7, 40.1, 93.4 | ≥ 2 |
| tail carrier / spokes | 0.0 | 36.8 | ≥ 2 |

**The Radxa's y, reconciled (H STEP, not measured):**
- The layout's box is ±3.15 (6.31). The PCB top is at y 0 and the PCB is 1.6 thick (y −1.6..0).
- Components reach +3.15 on the +y side. On the back, parts (the microSD socket) reach −2.86, inside the box's −3.15.
- The Ø8 posts run from the side wall (y −5.15, now stepped out 0.6 beside the Radxa so it stays 2.0 off the box) to the PCB's back at y −1.6. They sit in the box's back-side envelope only at the 4 holes. The fit models that as Ø8.4 keep-outs, assuming nothing on the back within 4.2 of a hole centre.
- Round 4's y −4.6..3.2 was the body plus the post space.
- **Measure first:** if the microSD socket is 14 wide, it is only about 0.4 from the Ø8 posts at the SD-end holes.

## Poses (v12, the requirement; not changed here)
| Item | Pose (C) |
|---|---|
| Radxa Zero 3W | On edge on the centre plane: x −138.9..−71.9, y −3.15..3.15, z 8..38 (ports overhang to 39.5). −138.9 includes the 2 mm microSD stub; the board is −136.9..−71.9. Component side +y, SD end aft. Wash +5.7 |
| UBEC | Body x −163.1..−118.1, leads to −171.1 / −110.1; y 11.2..26.6, z 16..24 (v12's 61 x 15.4 x 8 box, E, not measured). 8.0 of air over the Radxa's SoC. Wash +18.8 |
| XR2 (placed here) | x −110..−94, y −27..−15, z 23..31, antenna to 45. The shelf sits 8.5 further aft along the spine than in round 3: v12's frame place, the most forward pose clear of the wash (+3.7), so the lightest shelf. Floor z 21..23. UART to the FC GPS port |

## Concept A (picked: variant V8): 2 printed pieces
1. **Belly-mount tail block (the root), 15.9 g (E), CG (−45.3, 0.7, 12.9).**
   - **Block:** x −69.4..−48.05, y −4.55..6.5, z −8..18.5, 2.0 behind the TPU end pieces.
   - **Gusset:** bears on the receiver's tail face (z −4..9.6, |y| ≤ 7.2) and takes the tail-down moment.
   - **Web:** runs under the plate to x −31.0. It crosses |y| 6.6..9 only in the plate's shadow (the notch front is x −40.0).
   - **Plate bolts:** 2 M3 go down through the plate's tail tab at **(−42.0, ±16.0)**.
     - Edge distances: 8.0 to the tab's tail edge, 7.0 to the notch edge.
     - 11.5 from the layout's own FC USB-C path axis (need 11).
     - The keys are ball-end, 25° toward the centreline.
   - **Fin and foot:** the fin (|y| ≤ 6.5) passes through the notch from x −46.05. Its foot is a wedge on the plate top, z 34 at the fin down to 4 tall at x −14, |y| ≤ 8.0. This is the tail-up bearing.
   - 6 inserts, each in a Ø10 boss with 3.0 walls.
2. **Tail carrier, 33.2 g (E), CG (−102.6, −0.7, 9.6), one print.**
   - **Yoke:** tongues 13.05 x 5 over and under the block, 4 M3 (Ø6 x 3 counterbores, 2.0 web).
     - The yoke's back is its side wall: 2.0 along the block's −y face, plus a cheek out to the wash line and a fillet onto the upper tongue.
     - The side wall steps out to y −7.15..−5.15 behind the block, beside the Radxa.
     - A lower back (|y| ≤ 6.5, z −13..6) runs under the Radxa to x −92.5.
   - **Spine:** the side wall carries the 4 Ø8 posts. Aft of x −78.5 it adds a closed tube 6.5 deep and 14 tall along its lower edge.
   - **UBEC:** a tray on a 6-deep torsion box, a cross plate and a rib.
   - **XR2 shelf:** a 2.0 floor at z 21..23 on a 6 x 14 post down into the spine tube.

**Mount total:** 49.2 g (E), CG (−84.0, −0.2, 10.7). **Cables (5 runs):** 6.3 g (E), CG (−8.3, 2.3, 31.6). **Carbon: none.**

## How the block comes off
- Take out the 4 yoke bolts: top keys straight, bottom keys straight or ball-end 15° aft.
- Unplug the bay's cables and slide the carrier aft off the block. The Radxa, UBEC and XR2 come with it.
- Take out the 2 plate bolts. The ball-end 25° keys clear the swept prop disks by 0.18 / 0.06 on the prop meshes `system_check` uses, so no prop comes off.
- The block then slides aft out of the open notch.
- Round 4's "replaced without touching the carrier" was wrong.

## Sized on dimensions.md L (`loads_mesh()`, E)
- **Cases:**
  - crash, 35 g along the 6 face and 8 corner directions (all 14: the bay is not symmetric), limit 1.0;
  - flight, 4 g along the thrust axis and 2 g ±x / ±y, limit 0.5;
  - landing, 6 g, limit 0.5.
  - Each acts at the CG of every mass it moves (items, half the cables, each piece's volume as point masses).
- **Not an L case:** the 50 N "far item corner" load (a hand on the UBEC through a cell, E). It is reported, not sized on. Keeping it as a design case costs about 10 g: V0 needed 0.80 there.
- **Sections:** cut from the meshes along the path, with 2.4 walls:
  - carrier x planes every 3 mm;
  - yoke + block clamped;
  - block / web / fin aft of the plate bolts;
  - web-arm y planes;
  - foot x planes.
- **The yoke's load paths:** the tongues take z, pitch and roll in bearing on the block. The 4 bolts take x, y and yaw; their tongue bearing (375 N per bolt) is the fuse.

**The variants (all on L, CG at −5.0 with main's assembly, the same run):**

| Variant | Mount g | CG in / out | Crash worst | Flight / landing | L breaking order (≤ 0.59 / 1.5) | Yoke or block 1.5 x before the carrier | 50 N corner (E) |
|---|---|---|---|---|---|---|---|
| V0 round 4 (tube 20, UBEC box full, block solid, foot 45, lower back 20) | 59.4 | 0.72 / 2.73 | 0.44 | 0.01 / 0.02 | no (0.44) | all but +x | 0.80 |
| V1 tube 14 | 58.3 | 0.62 / 2.51 | 0.43 | 0.01 / 0.02 | no | all but +x | 0.80 |
| V2 V1 + UBEC box 6 | 57.0 | 0.43 / 2.10 | 0.41 | 0.01 / 0.02 | no | all but +x | 0.80 |
| V3 flanges only + box 6 | 56.5 | 0.44 / 1.98 | 0.71 | 0.08 / 0.12 | no | fails 14 / 14 | 5.92 |
| V4 V2 + block 2.4 walls / 20 % | 51.8 | 0.46 / 1.72 | 0.27 | 0.01 / 0.02 | yes | all but ±x | 0.80 |
| V5 V4 + foot to z 34 | 50.1 | 0.49 / 1.64 | 0.27 | 0.01 / 0.02 | yes | all but ±x | 0.83 |
| V6 V5 + lower back 6 | 47.9 | 0.66 / 1.32 | 0.26 | 0.01 / 0.02 | yes | fails 7 / 14 | 0.94 |
| V7 V3 + V6's lightening | 47.2 | 0.78 / 1.19 | 0.73 | 0.08 / 0.12 | no | fails all | 5.92 |
| **V8 V5 + lower back 14 (picked)** | **49.2** | **0.56 / 1.49** | **0.27** | **0.01 / 0.02** | **yes** | **all but ±x** | 0.83 |

- **Pick rule:** pass L, its breaking order and balance; then the fewest cases where the yoke or block is not 1.5 times ahead of the carrier; then the lightest. That gives V8.
- **Crash detail (V8):**
  - Worst 0.27: the web arm at y −9, 35 g corner (−x −y −z).
  - 35 g y: the fuse first (0.17) before the block (0.14) and the carrier (0.07).
  - 35 g −z: the block (0.10), carrier 0.05.
- **L breaking order (cage first):** the mount's crash worst is 0.27 against the cage's 0.59 (the sized 6/4 cage's worst rod, frame model v2 `alt_sized`, F on E). 0.27 ≤ 0.59 / 1.5 = 0.39, so it holds.
  - The cage nodes' index isn't computed yet, so this uses the worst member.
- **Our own order (yoke or block before the carrier):** it holds in 12 of 14 crash cases.
  - It misses **pure ±x**: 35 g x is carrier 0.054 (spine at x −112) vs fuse 0.025 and block 0.022; 35 g −x is block 0.063 vs carrier 0.054.
  - All of these are ≤ 0.07, at least 15 times short of failure. Reported, not fixed.
- **Least-I sections:**

  | Section | x | A | Iy | Iz | J |
  |---|---|---|---|---|---|
  | Carrier, vertical | −160.4 | 157 | 832 | n/a | 1006 |
  | Carrier, lateral (the spine between the lower back and the UBEC rib) | −115.4 | 124 | 11577 | **807** | 1133 |
  | Yoke + block | −50.4 | 245 | 44143 | 5897 | n/a |
  | Block, vertical | −42.5 | n/a | 5942 | n/a | n/a |
  | Block, lateral | −47.5 | n/a | 7601 | 2170 | n/a |

- **First modes (E):** lateral 235 Hz, vertical 476 Hz. They come from the section EI and GJ from the plate bolts to the carried CG (62.4 g at x −106.8), PETG E 2000, rigid joints.

## Checks (A, v12 at catch −5.0, C)
| Check | Result |
|---|---|
| `system_check` | **PASS.** Cage first 6.2 (the camera, theirs). Shell 47 rods, closed. Fit 0 with 0.05 mm³ declared contacts. Probes 7.2 (UBEC). Access 16 / 16. **CG 0.6 / 1.4**. Bbox 13.8 L (390 x 278 x 127), largest extent 391 |
| Wash (≥ 3, plan, rear disks R 44.5 + 3) | Radxa +5.7, UBEC +18.8, XR2 +3.7, tail block +3.0, carrier +3.0. Material in |y| 6.6..9 sits only under the plate |
| Funnel line, from v12's ring tail node (−63.9, z −49.6) − 2.1 and its tail point (−231.5, z 41.5) − 1.7, both + 3: **(−66.0, −46.6) → (−233.2, 44.5)** | Radxa +14.9. **UBEC +5.4**, lowest at x −171.1, z 16.0 (the line is at 10.6 there): not under the line. XR2 +45.7 with the floor now at z 21. Tail block +36.7, carrier +3.7 |
| Rods / shell inset | Rods: block 27.0, carrier 16.6, Radxa 20.3, UBEC 13.8, XR2 14.9. Inset: block 25.8, carrier 3.5, UBEC 5.0, XR2 14.9 |
| Height cap 45 | Carrier top 37.5, foot top 34.0. Plugs keep-clear z 39.5..45 (E, not measured). XR2 antenna to 45 |

| Access line (`system_check` clear) | How |
|---|---|
| Radxa microSD | Ø10 aft; out of the shell after 59, 11.4 off any rod |
| Radxa USB-C 1 | Ø10 straight up, 18.4 clear |
| FC USB-C | The layout's own path from (−12.56, 0, 13.5) |
| Yoke keys | Top straight up (7.2 / 8.0 clear); bottom front straight down (5.9); bottom rear ball-end 15° aft (5.7) |
| Plate keys (−42.0, ±16.0) | Ball-end 25° toward the centreline. 0.18 (+y) and 0.06 (−y) clear of the prop meshes `system_check` sweeps. This script now measures against the same meshes, so the two agree |

## Balance (main's assembly, with the camera mount)
- **At v12's catch −5.0: CG 0.56 / 1.49 pack in / out (E), 746 g: PASS.** `system_check` reads 0.6 / 1.4.
- The best riding catch would be −6.0 (1.09 / 1.06), but nothing needs it.
- **UBEC pose region (balance, funnel and wash together, everything else fixed):** at −5.0 the UBEC may shift x −40..+17, y 0..+20, and z down to −5 (at x 0).

## Requirements (`envelope.json`)
1. **Cage session:** the mount is 49.2 g at (−84.0, −0.2, 10.7) and the cables are 6.3 g at (−8.3, 2.3, 31.6). Keep these clear:
   - rods ≥ 3 off the mount (now ≥ 16.6);
   - plugs: x −99.8..−68.4, y −4..6, z 39.5..45 (E);
   - microSD: Ø10 line from (−138.9, −3, 23) along −x;
   - USB-C 1: Ø10 line from (−95.4, 1.6, 39.5) straight up;
   - the 6 Ø4 key lines.
2. **Belly mount** (a placeholder box today; this model cuts it so): its tail face at **x −31.0 over |y| ≤ 22**, z 9.6..18.5 (DEFAULT; the alternative is inserts in the belly mount itself).
3. **Plate:** 2 x Ø3.4 holes at (−42.0, ±16.0) in the tail tab.
4. **Receiver:** its tail face takes the gusset's bearing (z −4..9.6, |y| ≤ 7.2).
5. **Scout layout, Radxa entry:** the mount's 4 posts land on the PCB back at y −1.6. The layout's Radxa box (±3.15) must take the 4 Ø8.4 back-side keep-outs this model cuts (`radxa_envelope()`, from the H STEP data, not measured), or the posts read 1.55 deep (269 mm³) in main's assembly.

## What the extra mass buys
The mount is 49.2 g against the layout's 38.6 g placeholder (+10.6 g, E). Crash worst is 0.27 of 1.0, so strength isn't what sets it:
- the spine tube (14 tall) and the UBEC box keep the internal break order (fuse or block first) in 12 of 14 crash directions; the lighter flanges-only spine (V3 / V7) reaches 0.71 / 0.73 and loses the order;
- the tail block is a printed root sized for the bolts' bosses (Ø10, wall 3.0) and the gusset bearing on the receiver;
- the piece passes (topology on the same L loads) are where the grams come out, with the balance (0.56 / 1.49) and the order as constraints.

## Alternative B: A + a Ø6 strut to the tail point (−231.5, 0, 41.5), clip fuse
- +4.3 g; modes 244 / 479 Hz against A's 235 / 476: almost no stiffness gained.
- CG 0.79 / 2.91.
- Its fit fails: it overlaps the cage node 71 mm³ and the carrier 18 mm³, and dips 5.0 under the funnel line.
- **A picked.**

## Still open
1. **Our own order in pure ±x:** the carrier's spine at x −112 is within 1.2 times of the block (indices ≤ 0.07). L orders the mount only against the cage, and the cage goes first by about 8 times, so this is a piece-pass constraint (DEFAULT): the carrier's piece pass keeps the spine section at x −112.4 (A 125, C) ≥ 1.5 times the block in ±x.
2. **Requirements:** the belly mount's tail face and the 2 plate holes are open.
3. **Measure first:** the UBEC, the plugs, and the microSD socket's width at the posts.
4. **L breaking order is checked against the worst cage rod, not a node.** L asks the mount to stay ≤ 1/1.5 of the worst cage node or foot index. Node indices aren't computed yet, so the 0.59 used is the sized 6/4 cage's worst rod (F on E, a cage Spark has not decided). It needs a node or foot index ≥ 0.41 (= 0.27 x 1.5) when the frame model reports nodes.
5. **Plate keys have no practical margin:** the binding measure is the prop meshes `system_check` sweeps, 0.18 / 0.06 (C); the reviewer's swept-disk measure reads 0.21 / 0.09. The keys are also only 0.77 / 1.42 off the block's own fin and foot at the ball-end's 25° limit (C, reviewer's measure). Any hole tolerance uses that up. In practice turn the props off the line, or use a stubby key.

## Visual
`preview.png`, also at https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU. Top row A, bottom row B. Side, top and rear views on v12 at catch −5.0, with the wash rings, the funnel line (red dashed) and the SD and USB-C tool lines.

## Next
1. Concept review.
2. Piece passes: topology on these L loads for the tail block and the carrier.
3. Measure the UBEC, the plugs and the microSD socket.
