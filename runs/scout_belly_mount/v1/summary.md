# Scout v3 belly mount, concept v1 (27 Oct 2026, system pass)

**In one line:** one PETG print, 25.2 g (E), clamped between the Bee35 plate (z 18.5) and the receiver's mount plane (z 9.6). It takes the receiver's 4 M3 bolts in inserts, bolts to the plate with 4 new M3, frames the FC, and holds the flow sensor on a post beside the receiver. The tail block's tongue bears on its tail face. `solid_check` PASS (1 body). `system_check` PASS (fit 0, access 25/25, CG 0.2 / 2.2). **Two pair-rule misses, not this part's:** FC to plate 1.0 and FC to the receiver's rib 0.9 (rule 2.0); see Open.

**Files**
- Script: `nox/scout/v3/belly_mount.py`. One run takes about 8 min. It is seeded, so a rerun reproduces the STL, `result.json`, `system_check.txt` and `preview.png` byte for byte (checked: two full runs).
- Outputs: `belly_mount__petg_.stl`, `result.json`, `run.log`, `system_check.txt`, `manifest.json` (the system model as checked), `preview.png`.
- Visual: https://claude.ai/artifact/HRkD9JSKWhZFe1MChcSceV (source `concept.html`) (built from `preview.png`).

**Tags:** C = computed on the v12 geometry; E = estimate; P = measured; M = bought-part data. Mass basis is the electronics session's: 2.4 walls + 20 % infill (solid 28.7 g).

## System model
- Main's `scout.assembly.json`: layout v12, catch 5.0 forward, camera mount v1.
- Changes to it:
  - the tail block and carrier as drawn (PR 130 STLs) in place of their mass points;
  - the FC as the real board (36 x 36 x 8) in place of the layout's 51 x 51 envelope;
  - the 20 g wires as a mass point at the FC;
  - the flow sensor re-placed (below);
  - the placeholder "mount + nose legs" replaced by this piece.

## 1. FC: on the plate's own AIO holes (picked) and the stack
- **Where:** the FC stays where it is built, on the Bee35's 25.5 AIO holes at the plate centre (x 0; P, CHECK the hole positions on the plate). It hangs on 4 grommet screws through the carbon, so its load path goes straight into the plate, not through PETG.
- **Height:** z 9.5..17.5 (H: lowest point 9 below the plate).
- **Stack (E, CHECK on the board):** M2 x 12 button screws from the plate top, a Ø4 x 3.2 spacer, M2 grommet flanges 1.0 either side of the 1.6 PCB, M2 nut at z 10.1..11.7 in the receiver's tray (floor 5.6).
- **M2, not M3 (DEFAULT):** the board's holes are Ø3 (M), and a common M3 grommet needs a Ø4+ hole. M2 also keeps the screw heads 3.0 off the tail block's foot; an M3 head would be 1.9 off it. Alternative: M3 if the holes measure Ø4.
- **Belly mount round it:** a window of board + 2.0 (gap 2.00, C). Nothing of the belly mount carries the FC.
- **FC yaw, from the USB-C search:**
  - USB-C on the +y (left) edge at x 5, out at azimuth −10° (the receiver's 1.0 wall relief is there), 250 mm clear (890 of 11662 directions clear, C);
  - battery pads on the nose edge, toward the XT60 (leads 24 mm, in a channel under the front bridge);
  - DJI SH on the right edge.
  - Set PX4 `SENS_BOARD_ROT` to match at the bench.
- **IMU offset:** x +0.2 mm from the CG, not the layout's +5.0. The FC no longer rides with the catch; moving its 10 g back 5 shifts the CG 0.07 mm.

**First mode on the grommets (E):**
- Lateral (the first mode): **543 / 665 / 858 Hz** for silicone E 1.0 / 1.5 / 2.5 MPa.
- At 1.5 MPa: vertical 1097 Hz, rocking 1313 Hz.
- Hard-mounted on the same screws it would be about 8 kHz, so **the grommet sets the number**. Gate ≥ 100 Hz: PASS.

What the grommet does to vibration:
- These are short, thin grommets, so they are stiff. Isolation starts only above √2 x f_n, about 770..1200 Hz.
- The rotor band at hover (1P ~250..330 Hz E, blade pass ~500..670 Hz E) sits below that, near the lateral mode.
- So they barely isolate, and could amplify at blade pass. For real isolation the mode would have to sit at 100..175 Hz: a softer, taller grommet.
- DEFAULT: the as-built grommets; the first PX4 vibration log (accel FFT) decides.

## 2. Flow sensor: placed by search
- **The search:** 2.6 M poses (1 mm grid, 4 orientations), seeded. Each pose must have:
  - body + 2.0 cradle + 2.0 clear of every part;
  - not under the swap set;
  - out of the funnel band (30 over the ring plane) unless inside the swap column;
  - inside the cage hull by node r + 3;
  - both windows' 23° cones (42 FOV + 2) clear of everything down to the ground;
  - clear air (below);
  - then the shortest clear 6 x 6 strut to the belly mount, straight or with one bend.
- **Result:** 8349 feasible, 518 with a strut (C). Best: **body at (0, −40), z −30..−22, long side along x, lens to the nose, plug out the tail end (as built)**. It hangs on a **6 x 6 post, 35.6 long**, from the right rail at (0, −30.5), with gussets; the 2.0 cradle has bottom lips and leaves the windows open.
- Post gaps: 2.2 to the receiver's wall, 2.8 to the tie saddle rail (C).
- **View:** both windows clear of every part and rod to the ground (C).
- **Other zones:**
  - nose and tail: the bottom ring's front (x 90.8) and rear (x −63.9) rods enter the cone within the 27 / 18 mm between them and the receiver;
  - outboard above the saddle rail: the saddle ties are in the cone, and the strut has no clear path.
- **Funnel band:** the flow sits inside the swap column, in the band (like the layout's pose). That is legal only if the box's ramps stay out of the swap column (+30 a side). It was the only zone with a clear view. The design center confirms.
- **Clear air:**
  - no pose with a clear view passes the column measure (rule 8: plan ≥ 3 outside every swept disk); of the 8349, 0 do;
  - this pose has 2.6 % of its plan under the front-right disk;
  - picked on the slipstream at its depth (momentum theory, E): 63 below the props the wake radius is 33.0, and the body's nearest corner is 43.7 from the motor axis (+10.7 outside);
  - **a rule-8 method question for Spark** (alt: the column measure, then no flow pose exists under this cage).
- **Post mode:** 274 Hz (E: cantilever, 4.8 g flow + cradle).
- **PX4 (FRD from the CG (−0.19, 0.02, 2.47), m):**
  - `EKF2_IMU_POS` (0.000, 0.000, −0.011);
  - `EKF2_OF_POS` (0.006, 0.040, 0.033), the lens at the nose window;
  - `EKF2_RNG_POS` (−0.005, 0.040, 0.033), the ToF at the tail window;
  - flow yaw as built (`SENS_FLOW_ROT` unchanged).

## 3. The belly mount (drawn, concept)
**Frame z 9.6..18.5 (8.9, the gap):**
- rear cross-member x −31..−20, |y| ≤ 21: its tail face takes the tongue;
- 2 ears x −44..−31, |y| 21..31.6, carrying the rear receiver inserts. Their inner faces bear on the tail block's plate-bolt bosses at |y| 21.0 (C);
- side rails |y| 20..26.5 on the receiver's wall top;
- front blocks x 24..36 with a 3.5 bridge over the battery-lead channel (|y| ≤ 6);
- front receiver bosses to x 53.3 (2.3 past the plate's nose edge, on the lid);
- the XT60 window: x 40.2..56.3, |y| ≤ 15.6 (+2.0).

**Attachments:**
- **Receiver:** its 4 M3 x 35 up into M3 x 5.7 heat-set inserts at (−37.65, ±26.55) and (48.28, ±26.55) (C), hole Ø4.0 x 6.5, Ø10 bosses, 3.0 walls. The receiver's own interface spec, unchanged.
- **Plate:** **4 new M3 down through the plate into inserts at (−25, ±13) and (30, ±13)** (DEFAULT).
  - Straight keys clear of the swept disks; 12.25 off the FC screws; 2.15 head-to-foot clearance.
  - Holes Ø3.4, ≥ 8 from the plate edges (C). CHECK they miss the plate's existing holes.

**Cuts:**
- the FC window;
- the USB-C window: full height, along the plug's path, on the left rail;
- a 2.5 x 4 groove under the right rail for the flow's lead (laid before the receiver goes on).

**Bearing faces (declared contacts, all 0.000 mm³):**
- top on the plate;
- bottom on the receiver's wall tops and the lid;
- tail face and ears on the tail block;
- the cradle on the flow.

**Mass and gate:** 25.2 g (E; solid 28.7), CG (1.8, −7.5, 8.7), the cradle 3.3 g. `solid_check` PASS: 1 body, eroded 1, no necks.

## 4. Load path and strength (L cases, E)
**Load path:**
- **Hung set:** receiver set + swap set, 336.4 g at (6.4, 0, −15.2). It goes into the 4 receiver inserts (elastic bolt group; compression in bearing on the receiver's wall tops), then the frame, then the 4 plate inserts and the frame's top bearing face, into the carbon.
- **FC:** grommets, screws, plate.
- **Flow:** post, rail.
- **Tail bay (78.3 g E):** its tongue bears on our tail face only when pushed forward (+x crash). Its 2 plate bolts and the receiver gusset take the rest (PR 130).

| Kind (limit) | Receiver inserts | Plate inserts | Frame (bending, hollow 2.4) | Flow post | Tail face bearing | Carbon bearing |
|---|---|---|---|---|---|---|
| Crash 35 g, 14 directions (1.0) | 0.049 (−x+y−z) | 0.092 (−x+y−z) | **0.242** (−x+y−z) | 0.066 (x) | 0.005 (x, 26.9 N) | 0.012 |
| Flight 4 g / 2 g (0.5) | 0.003 | 0.005 | 0.019 | 0.004 | 0 | 0.001 |
| Landing 6 g (0.5) | 0.005 | 0.007 | 0.029 | 0.000 | 0 | 0 |

- **Capacities (E):** inserts 1 kN pull / 1 kN bearing (published coupon range 0.9..1.5 kN); PETG 25 MPa; carbon 300 MPa bearing.
- **Frame index:** conservative. Each receiver insert's pull is taken as a cantilever to its nearest plate insert (18.5 / 22.8), over the weakest cut section, the front rails at x 42 (Z 167 mm³, C).
- **Sections:**
  - x −31: A 141, Z 265;
  - x 0, the right rail alone (the USB window cuts the left): A 90, Z 159;
  - x 30: A 299, Z 522;
  - x 42: A 102, Z 167.
- **Breaking order (L):**
  - the mount's crash worst 0.24 ≤ the cage's 0.59 / 1.5 = 0.39 (worst rod, F on E; node index still open). Holds.
  - Inside the system the cheap piece goes first: the belly mount's frame and inserts (0.24 / 0.09) are ahead of the carbon (0.012) and the FC (on grommets).
  - A broken belly mount is replaced with 4 plate bolts and 4 receiver bolts (pack out first). The plate and FC survive.

## 5. Fit (every pair within 10, no allowances; C)
| Pair | Overlap mm³ | Gap | Kind |
|---|---|---|---|
| belly mount / plate | 0.000 | 0 | declared contact |
| belly mount / receiver v3 | 0.000 | 0 | declared contact |
| belly mount / receiver lid | 0.000 | 0 | declared contact |
| belly mount / tail block (tail face, ears) | 0.000 | 0 | declared contact |
| belly mount / flow (cradle) | 0.000 | 0 | declared contact |
| belly mount / FC | 0 | 2.00 | ≥ 2 PASS |
| belly mount / tie saddle | 0 | 2.80 | ≥ 2 PASS |
| belly mount / TPU strips, cartridge, slider −y, XT60 | 0 | 6.10, 7.50, 7.66, 8.20 | ≥ 2 PASS |
| belly mount below z 9 (post, gussets, cradle) / receiver, saddle, end pieces | 0 | 2.20, 2.80, 17.4 | ≥ 2 PASS |
| belly mount / cage rods, links | 0 | 11.2 | ≥ 2 PASS |
| flow / receiver, saddle, rods | 0 | 6.10, 7.00, 13.7 | ≥ 2 PASS |
| **FC / plate** | 0 | **1.00** | ≥ 2 **FAIL** (as built) |
| **FC / receiver v3 (its rib x 3..6.5, top 8.6)** | 0 | **0.90** | ≥ 2 **FAIL** (as built) |
| FC / TPU strips, cartridge, pack, tail block, rods | 0 | 6.0, 8.4, 10.0, 4.5, 29.6 | ≥ 2 PASS |

`system_check` fit 0.0 mm³ (contacts at 0.05).

## 6. Checks (`system_check.txt`)
- Overall: `system_check` **PASS**.
- Cage first 6.2 (the camera, as main). Shell 47 rods closed. Probes 7.2.
- Bbox 13.8 L (390 x 278 x 127).
- Mass 746 g E (soft over 500).
- **CG 0.2 pack in / 2.2 pack out** (gate 3.0; main read 0.6 / 1.5).
- Access 25 / 25 clear:
  - swap ×3;
  - 4 receiver keys from below;
  - FC USB-C;
  - 4 plate-bolt keys from the top;
  - 4 FC grommet keys from the top;
  - the flow plug;
  - the tail bay's 6 keys and its SD / USB-C tool lines.

## 7. Access and cables
- **From the top, nothing off:** the 4 plate bolts and the 4 FC screws. The FC's nuts sit under it, in the receiver's tray: removing the FC means the receiver comes off (pack out, 4 bolts from below). The belly mount stays on the plate.
- **FC USB-C:** out the left window, 250 clear.
- **FC SH plugs:** face plugs (if top-entry) need the FC out, as built. Edge plugs on the left are reached through the USB window. CHECK the connector entry on the board.
- **Flow plug:** clear 25 out of its tail end. The lead goes up the post and under the right rail in its groove, to the FC.
- **Cable routes to the tail bay** (Radxa UART, XR2 to the FC GPS port, UBEC power from the FC pads): out the left window beside the USB, up round the plate's edge at x −5 (clear of the saddle tabs at x ±14..26), aft over the plate top beside the fin and foot (|y| 9..12), down into the bay. Tie points: the flow post and the window's edges.
- **Camera USB:** from the nose over the plate top (PR 130's run), clear of this piece.
- **Battery leads:** XT60 to the FC's nose-edge pads, 24 mm in the channel under the bridge.
- **Heat:** the ESC is in the closed sandwich (rule 8, heat sources in airflow, **not met**; same as built). Only the left window and the lead channel vent it.

## 8. Alternatives weighed
**FC placement:**
1. **(Picked) FC on the plate's AIO holes (x 0, as built):**
   - Wins: no new holes beside the AIO pattern; the shortest, stiffest path (carbon); harness as built; first mode 543..858 Hz.
   - Loses: FC 1.0 / 0.9 off the plate and the receiver; nuts under the FC; ESC enclosed.
2. **FC on the belly mount at x +5 (the layout's):**
   - Needs a seat over each FC hole: insert 5.7 + cap 2.4 + Ø10 boss, so **8.1 of height above the FC where 1.0 exists**.
   - Only possible by dropping the FC 4.7 into the receiver's tray (a receiver change). The mode then falls by the PETG seat in series.
   - Rejected.
3. **FC on the plate top (for Spark):**
   - Wins: ≥ 2 on every side; USB, plugs and screws reachable from above; the ESC in free air; the sandwich frame loses its FC and USB windows (both rails whole).
   - Costs: Spark's 12 Oct "the FC cannot move up"; 6 forward on 4 new holes (or the tail block's foot cut back 6) to clear the foot at x −14; cables over the top.

**The tail block's root:**
1. **(Picked) bearing face at x −31 + ears:** no fasteners. Peak 26.9 N (35 g +x on the 78.3 g bay), 0.12 MPa, index 0.005. The tail block slides off aft after its own 2 plate bolts.
2. **2 M3 inserts in our tail face:** +2 inserts + 2 bolts (~1.0 g E). Would also carry the −x pull (26.9 N, ~0.03), which the block's own plate bolts already take at 0.028. But their keys run horizontally through the tail block and carrier: no straight key line, and the block could not come off without the belly mount.

## Carbon
None in this piece (no rods). The cage rods are the layout's (`runs/scout_layout/v12/summary.md`).

## Requirements on other parts
1. **Plate:** 4 x Ø3.4 at (−25, ±13), (30, ±13). The FC on the existing 25.5 AIO holes (M2, Ø2.2+ CHECK).
2. **Receiver (battery side, not changed here):**
   - the 2.0 FC rule asks its rib under the FC (x 3..6.5, top 8.6) to come down ≥ 1.1 (to ≤ 7.5), over the FC's footprint |y| ≤ 18;
   - with that, the FC could drop 1.1 and clear the plate by 2.1 and the rib by 2.0. Until then the as-built 1.0 / 0.9 stand.
   - Its 4 inserts and bearing faces are as its interface spec.
3. **Tail block (this session's, piece pass):**
   - keep its tongue's front at x −31.0;
   - keep its plate-bolt bosses at |y| ≤ 21.0 between x −44 and −37.5, where our ears bear (a flat at |y| 21 gives a face contact instead of a line).
4. **Cage session:**
   - the belly mount 25.2 g E at (1.8, −7.5, 8.7), replacing the 25 g placeholder at (10.2, 0, 13.1);
   - the FC 10 g now at (0, 0, 13.5) (was the box centre (6.9, 0, 13.5));
   - the flow 1.5 g at (0, −40, −26).
   - Keep the saddle rail and ties ≥ 2 off the flow cradle (body + cradle y −50..−30, x −12.8..12.8, z −32..−20 under the right saddle rail; now 2.8 / 11.2).
   - The cradle reaches y −50, outside the brief's ±35 envelope, but only under the saddle rail, 5.0 below it.
5. **Design center:** the flow sits in the funnel band inside the swap column (like the layout's). The ramps must leave the swap column (+30 a side) free.
6. **Spark (method):** rule 8's clear-air test for a body 60+ below the props (column vs contracted slipstream).

## Open
- **Measure:** the FC's stack as built (grommet type and size, spacer, nut, PCB height in the 8.0 envelope, connector entry); the plate's AIO hole positions and size; the M2 / M3 question.
- **Vibration:** the vibration log, then the grommet choice (100..175 Hz for isolation).
- **ESC heat** in the sandwich (bench temperature at hover current).
- **Next:** concept review; then the piece pass (topology on these loads; the frame is light on index, so the grams come out there).
