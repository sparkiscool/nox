# Scout v3 belly mount, concept v1 (27 Oct 2026, system pass, round 2)

**In one line:** one PETG print, **25.0 g (E)**, under the Bee35 plate. It is two yokes clamped between the plate (z 18.5) and the receiver's mount plane (z 9.6), joined only through the flow sensor's bipod and cradle:
- a rear yoke: tail face, ears, rear receiver inserts, 2 plate inserts;
- a front yoke: front receiver inserts, 2 plate inserts, bridge.

Nothing of it stands beside the FC over x −18..18, so the ESC has **641 mm² of side vents**. Gates: `solid_check` PASS (1 body). `system_check` PASS (fit 0, access 25/25, CG 0.2 / 2.2). **Two pair-rule misses are the FC's as-built height, not this piece:** FC to plate 1.0, FC to the receiver's rib 0.9 (rule 2.0). The soft-mount requirement below fixes both.

**Round 2 (concept review 27 Oct, FAIL on round 1, 5350beb6):**
- The full-height rails beside the FC are gone, so the ESC is no longer boxed in by this piece.
- The flow hangs on a bipod from the two yokes instead of a post from the rail.
- New `preview_alternatives.png`: every option drawn from the same model.
- Inserts now at 600 N (dimensions Sled row), with prying on each yoke.
- The existing Bee35 plate holes are checked.
- Grommet requirement written for isolation.
- The wires mass point has no fit allowance; contact pairs are sorted.

**Files**
- Script: `nox/scout/v3/belly_mount.py`. One run takes about 8 min. It is seeded: two full runs gave byte-identical STL, `result.json`, `manifest.json`, `system_check.txt` and both PNGs.
- Outputs: `belly_mount__petg_.stl`, `result.json`, `run.log`, `system_check.txt`, `manifest.json` (the system model as checked), `preview.png` (the pick), `preview_alternatives.png` (the options), `concept.html`.
- Visual: https://claude.ai/artifact/HRkD9JSKWhZFe1MChcSceV (source `concept.html`, both PNGs).

**Tags:** C = computed on the v12 geometry; E = estimate; P = measured; M = bought-part data. Mass basis is the electronics session's: 2.4 walls + 20 % infill (solid 28.5 g).

## System model
- Main's `scout.assembly.json`: layout v12, catch 5.0 forward, camera mount v1.
- Changes to it:
  - the tail block and carrier as drawn (PR 130 STLs) in place of their mass points;
  - the FC as the real board (36 x 36 x 8) in place of the layout's 51 x 51 envelope;
  - the flow sensor re-placed (below);
  - the placeholder "mount + nose legs" replaced by this piece.
- **Wires (20 g E):** the CG is declared at the FC (0, 0, 13.5). For the fit they are a 0.4 box in free air in the right vent (no contact allowance; fit 0).

## 1. FC: on the plate's own AIO holes (picked) and the stack
- **Where:** the FC stays where it is built, on the Bee35's 25.5 AIO holes at the plate centre (x 0; P). Its load path is grommets, screws, then the carbon plate. The belly mount only frames it, with a 2.0 window.
- **Height:** z 9.5..17.5 (H: lowest point 9 below the plate).
- **Stack (E, CHECK on the board):** M2 x 12 button screws from the plate top, a Ø4 x 3.2 spacer, M2 grommet flanges 1.0 either side of the 1.6 PCB, M2 nut at z 10.1..11.7 in the receiver's tray (floor 5.6).
- **M2, not M3 (DEFAULT):** the board's holes are Ø3 (M), and a common M3 grommet needs a Ø4+ hole. M2 also keeps the screw heads 3.0 off the tail block's foot (M3: 1.9). Alternative: M3 if the holes measure Ø4.
- **FC yaw, from the USB-C search:**
  - USB-C on the +y (left) edge at x 5, out at azimuth −10° through the left vent, 250 mm clear (890 of 11662 directions clear, C);
  - battery pads on the nose edge, toward the XT60 (24 mm of leads, in the channel under the front bridge).
  - Set PX4 `SENS_BOARD_ROT` to match at the bench.
- **IMU offset:** x +0.2 mm from the CG, not the layout's +5.0.

**First mode on the as-built grommets (E):**
- Lateral (the first mode): 543 / 665 / 858 Hz for silicone E 1.0 / 1.5 / 2.5 MPa.
- At 1.5 MPa: vertical 1097 Hz, rocking 1313 Hz.
- Hard-mounted on the same screws: about 8 kHz. So the grommet sets the number. Gate ≥ 100 Hz: PASS.

What the grommet does to vibration:
- These thin grommets are stiff. They isolate only above √2 x f_n, about 770..1200 Hz.
- The lateral mode, 543 Hz at E 1.0, **sits inside blade pass** (500..670 Hz at hover, E), so they can amplify there. Hence the requirement below.

### Grommet requirement (for Spark, with the FC decision)
- **Target:** the FC's soft-mount mode at 100..175 Hz, so that √2 x f_n sits under rotor 1P (~250 Hz E) and isolates from there up.
- **Columns:** Ø6 / Ø2.2 silicone 30A columns (G 0.3 MPa E) between the plate and the PCB, working in shear. The window is 2.4..7.4 tall.
- **Height:** with the FC 2.0 under the plate the column is **5.2 tall** (2.0 + the 3.2 of upper components). That gives **120 Hz sideways and 211 Hz vertical (E)**, with 0.61 of travel at 35 g.
- **What it takes:** the FC drops to **z 8.5..16.5**, 2.0 to the plate and 2.9 to the receiver's tray floor. **The receiver's rib under the FC (x 3..6.5, top 8.6) must come down to ≤ 6.5, a cut of 2.1.**
- **Result:** one change (columns + rib cut) clears both FC pair misses and isolates the FC. The belly mount needs nothing: its window is already 2.0 round the board at every height.

## 2. Flow sensor: placed by search
- **The search:** 2.6 M poses (1 mm grid, 4 orientations), seeded. Each pose must have:
  - body + 2.0 cradle + 2.0 clear of every part;
  - not under the swap set;
  - out of the funnel band (30 over the ring plane) unless inside the swap column;
  - inside the cage hull by node r + 3;
  - both windows' 23° cones (42 FOV + 2) clear of everything down to the ground;
  - clear air (below);
  - then the shortest clear 6 x 6 strut to the belly mount.
- **Result:** 8349 feasible, 518 with a strut (C). Best: **body at (0, −40), z −30..−22, long side along x, lens to the nose, plug out the tail end (as built)**.
- **Support (round 2): a bipod.**
  - Two 6 x 6 struts, 34.6 long, from pads on the rail stubs at x ∓24 (y −30.5, outside the vents) down to the cradle top.
  - A 6 x 6 tie bar joins their feet over the cradle: a closed triangle.
  - Gussets at the roots. A 2.0 cradle with bottom lips leaves both windows open.
  - Gaps 2.2 to the receiver wall, 2.8 to the tie saddle rail (C).
  - Modes 405 Hz sideways and 3304 Hz along x / z (E). Crash index 0.032.
- **View:** both windows clear of every part and rod to the ground (C).
- **Other zones:**
  - nose and tail: the bottom ring's front (x 90.8) and rear (x −63.9) rods enter the cone within the 27 / 18 mm between them and the receiver;
  - outboard above the saddle rail: the saddle ties are in the cone, and the strut has no clear path.
- **Funnel band:** the flow sits inside the swap column, in the band (like the layout's pose). Box catch concept F (main, 27 Oct: a flat deck, end pads 3 tall, side bars at the ring tubes) would replace the ramps. Its pads and bars stay about 19 under the cradle and about 66 outboard of the flow.
- **Clear air, DEFAULT: the contracted-slipstream test.**
  - The body is outside the slipstream at its depth (momentum theory, E). 63 below the props the wake radius is 33.0, and the nearest corner is 43.7 from the motor axis: 10.7 outside.
  - **Alternative: the column test** (plan ≥ 3 outside every swept disk, the layout's and PR 130's measure). This pose has **2.6 % of its plan under the front-right disk**. **No pose with a clear view passes it** (0 of 8349), so under that test there is no flow pose under this cage. Spark rules (rule 8 method).
- **PX4 (FRD from the CG (−0.18, −0.08, 2.40), m):**
  - `EKF2_IMU_POS` (0.000, 0.000, −0.011);
  - `EKF2_OF_POS` (0.006, 0.040, 0.032), the lens at the nose window;
  - `EKF2_RNG_POS` (−0.005, 0.040, 0.032), the ToF at the tail window;
  - `SENS_FLOW_ROT` unchanged.

## 3. The belly mount (drawn, concept)
All at z 9.6..18.5 (the 8.9 gap) except the bipod.

**Rear yoke:**
- cross-member x −31..−20, |y| ≤ 21: its tail face takes the tongue;
- ears x −44..−31, |y| 21..31.6, carrying the rear receiver inserts. Their inner faces bear on the tail block's plate-bolt bosses at |y| 21.0 (C);
- rail stubs x −31..−18 on the receiver's wall top;
- the bipod pad at x −30..−18.

**Front yoke:**
- rails x 18..48 on the receiver's wall top;
- front blocks x 24..36;
- a 4.5 bridge over the battery-lead channel (|y| ≤ 6, channel 4.4 tall). The bridge is the link between the front halves;
- front receiver bosses to x 53.3 (2.3 past the plate's nose edge, on the lid);
- the XT60 window: x 40.2..56.3, |y| ≤ 15.6 (+2.0);
- the bipod pad at x 18..30.

**Vents (round 2):** nothing of ours beside the FC over x −18..18, z 9.6..18.5, both sides: **2 x 36 x 8.9 = 641 mm²** (0.2 mm³ of our material touches the edge planes, C).
- In hover and in any side wind, air runs straight across the board, in one vent and out the other (36 wide).
- In forward flight it enters by the nose channel (53 mm² under the bridge, into the FC's nose slot) and the windward vent edges.
- **Rule 8 (heat sources in airflow) is now met by this piece.** What still encloses the ESC is the plate 1.0 above and the receiver below, the system's sandwich.
- Mass 25.2 → 25.0 g: what the rails beside the FC weighed, the bipod, its tie bar and pads took back (net −0.2 g E).

**Attachments:**
- **Receiver:** its 4 M3 x 35 up into M3 x 5.7 heat-set inserts at (−37.65, ±26.55) and (48.28, ±26.55) (C), hole Ø4.0 x 6.5, Ø10 bosses, 3.0 walls (the receiver's interface spec, unchanged).
- **Plate:** **4 new M3 down through the plate into inserts at (−25, ±13) and (30, ±13)** (DEFAULT).
  - Straight keys clear of the swept disks; 12.25 off the FC screws; 2.15 head to the tail block's foot. Ø3.4 holes, ≥ 8 from the plate edges.
- **Existing Bee35 holes** (dimensions C, Plate bolts, P: sides (3.9, ±31), nose (45, 0)): our top face is 13.1 / 12.4 / 7.3 from their edges (C).
  - **They stay empty** (DEFAULT). A side bolt's tail and nyloc would hang to z ~13: into the USB path (its axis passes 1.17 from (3.9, +31)) and into the vent. A nut on the nose hole would land on the XT60, 3.7 under the plate.
  - **Using them, weighed:** a side bolt as a belly-mount attachment needs a Ø10 boss at |y| 31. That is 0.3 from the tie saddle rail (36.3), sits in the vent, and is 22 from the nearest bipod pad, so it would not stiffen the bipod root. The 4 new holes stand: (−25, ±13) and (30, ±13) are where the prying is lowest and the keys are straight.

**Bearing faces (declared contacts, all 0.000 mm³):**
- top on the plate;
- bottom on the receiver's wall tops and the lid;
- tail face and ears on the tail block;
- the cradle on the flow.

**Mass and gate:** 25.0 g (E; solid 28.5), CG (2.1, −10.7, 6.8), the cradle 3.3 g. `solid_check` PASS: 1 body, eroded 1, no necks.

## 4. Load path and strength (L cases, E)
**Load path:**
- **Hung set:** receiver set + swap set, 336.4 g at (6.4, 0, −15.2). It goes into the 4 receiver inserts (elastic bolt group; compression into bearing on the receiver's wall tops).
- **Each yoke** carries its 2 receiver pulls to its 2 plate inserts by **prying** about the worst edge of its bearing footprint on the plate. The plate inserts see **up to 3.32 x** the receiver pull (35 g x).
- **Shear** goes to the plate inserts and the carbon.
- **The flow** goes through the bipod, half to each yoke.
- **The tail bay (78.3 g E)** pushes on the rear yoke's tail face in a +x crash only (26.9 N, z 14.05). Its own 2 plate bolts and the receiver gusset take the rest (PR 130).

| Kind (limit) | Receiver inserts | Plate inserts (with prying) | Frame (bending, hollow 2.4) | Flow bipod | Tail face bearing | Carbon bearing |
|---|---|---|---|---|---|---|
| Crash 35 g, 14 directions (1.0) | 0.081 (−x+y−z) | **0.143** (+x+y−z) | **0.231** (−x+y−z) | 0.032 | 0.005 (x, 26.9 N) | 0.014 |
| Flight 4 g / 2 g (0.5) | 0.006 | 0.015 | 0.018 | 0.002 | 0 | 0.001 |
| Landing 6 g (0.5) | 0.009 | 0.023 | 0.028 | 0.000 | 0 | 0 |

- **Capacities (E):** an M3 heat-set insert in PETG ~600 N pull (dimensions Sled row, E), 600 N bearing taken the same; PETG 25 MPa; carbon 300 MPa bearing.
- **Frame index (conservative):** each receiver insert's pull over its lever to the nearest plate insert (18.5 / 22.8), on the weakest cut section, **the front rails at x 42 (Z 167.2 mm³, C)**.
- **Sections:**
  - x −31: A 141, Z 265;
  - x −22: A 364, Z 508;
  - x 30: A 310, Z 521;
  - x 42: A 102, Z 167.
- **Breaking order (L):**
  - the mount's crash worst 0.23 ≤ the cage's 0.59 / 1.5 = 0.39 (worst rod, F on E; node index open). Holds.
  - Inside the system the cheap piece goes first: the belly mount's frame and inserts (0.23 / 0.14) are ahead of the carbon (0.014) and the FC (on grommets).
  - A broken belly mount is replaced with 4 plate bolts and 4 receiver bolts (pack out first). Plate and FC survive.

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
| belly mount below z 9 (bipod, tie bar, cradle) / receiver, saddle, end pieces | 0 | 2.20, 2.80, 17.4 | ≥ 2 PASS |
| belly mount / cage rods, links | 0 | 9.3 | ≥ 2 PASS |
| flow / receiver, saddle, rods | 0 | 6.10, 7.00, 13.7 | ≥ 2 PASS |
| **FC / plate** | 0 | **1.00** | ≥ 2 **FAIL** (as built; fixed by the grommet requirement) |
| **FC / receiver v3 (its rib x 3..6.5, top 8.6)** | 0 | **0.90** | ≥ 2 **FAIL** (as built; fixed by the grommet requirement) |
| FC / TPU strips, cartridge, pack, tail block, rods | 0 | 6.0, 8.4, 10.0, 4.5, 29.6 | ≥ 2 PASS |

`system_check` fit 0.0 mm³ (contacts at 0.05; no other allowance of ours).

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
- **From the top, nothing off:** the 4 plate bolts and the 4 FC screws.
- **FC removal:** its nuts sit under it, in the receiver's tray. **The FC comes out only with the receiver off** (pack out, 4 bolts from below). The belly mount stays on the plate.
- **FC USB-C:** out the left vent, 250 clear.
- **FC SH plugs:** edge plugs are reached through the vents. Face plugs (if top-entry) need the FC out, as built. CHECK the connector entry.
- **Flow plug:** clear 25 out of its tail end. The lead goes up the rear strut and into the right vent, to the FC.
- **Cable routes to the tail bay** (Radxa UART, XR2 to the FC GPS port, UBEC power from the FC pads): out the left vent beside the USB, up round the plate's edge at x −5 (clear of the saddle tabs at x ±14..26; the side holes stay empty), aft over the plate top beside the fin and foot (|y| 9..12), down into the bay. Tie points: the bipod struts and the vent's yoke edges.
- **Camera USB:** over the plate top (PR 130's run).
- **Battery leads:** in the channel under the bridge.

## 8. Alternatives weighed (drawn: `preview_alternatives.png`)
**FC placement:**
1. **A (picked): FC under the plate on its AIO holes (x 0, as built).**
   - Wins: carbon load path; no new holes beside the AIO pattern; harness as built; ESC now vented both sides (641 mm²); first mode 543..858 Hz.
   - Loses: FC 1.0 / 0.9 to plate and receiver rib until the grommet requirement; the nuts under the FC, so **FC out = receiver off**.
2. **B (Spark's call): FC on the plate top at x +6.0** (clear of the tail block's foot by 2.0), on 3.0 standoffs, z 25..33.
   - Wins: plate 3.0, props 4.2, foot 2.0, plan 2.5 outside the swept disks; ESC in free air; USB, plugs and its 4 screws from above, and **the FC comes out with no other part off**; the belly mount loses its FC window.
   - Costs: Spark's 12 Oct "the FC cannot move up"; 4 new M2 holes 6.0 from the AIO ones; cables over the plate; the IMU 15.5 higher.
3. **C (rejected): FC seated on the belly mount at x +5 (the layout's).**
   - An insert seat over each FC hole needs 5.7 + 2.4 = **8.1 above the FC where 1.0 exists**.
   - Or the FC drops 7.1 (z 2.4..10.4), which cuts the receiver's tray 5.2 below its floor (a receiver change).
   - The PETG seat would also sit in series with the grommets.

**The tail block's root:**
1. **(Picked) bearing face at x −31 + ears:**
   - no fasteners;
   - 26.9 N peak, 0.12 MPa, index 0.005;
   - the block slides off aft after its own 2 plate bolts.
2. **(Rejected) 2 M3 inserts in our tail face** (at (−27.75, −4 / +10, 14.05)), +1.0 g:
   - they would also take the −x pull (26.9 N), which the block's own plate bolts already carry at 0.028;
   - their key lines, along the bolt axis aft, hit the tail block 31.8 / 8.4 mm behind the tongue. No straight key, and the block could not come off without the belly mount.

## Carbon
None in this piece (no rods). The cage rods are the layout's (`runs/scout_layout/v12/summary.md`).

## Requirements on other parts
1. **Plate:**
   - 4 x Ø3.4 at (−25, ±13), (30, ±13);
   - the FC on the existing 25.5 AIO holes (M2, Ø2.2+);
   - the existing side holes (3.9, ±31) and nose hole (45, 0) stay empty.
2. **Receiver (battery side, not changed here):** with the grommet requirement, its rib under the FC comes down to ≤ 6.5 (cut 2.1) over the FC's footprint |y| ≤ 18. Its 4 inserts and bearing faces are as its interface spec.
3. **Tail block (this session's, piece pass):** keep the tongue front at x −31.0 and the plate-bolt bosses at |y| ≤ 21.0 between x −44 and −37.5 (a flat at |y| 21 makes the ear contact a face).
4. **Cage session:**
   - belly mount 25.0 g E at (2.1, −10.7, 6.8), replacing the 25 g placeholder at (10.2, 0, 13.1);
   - FC 10 g at (0, 0, 13.5) (was (6.9, 0, 13.5));
   - flow 1.5 g at (0, −40, −26).
   - Keep the saddle rail and ties ≥ 2 off the bipod and cradle (y −50..−27.5, x −30..30, z −32..18.5 under the right saddle rail; now 2.8 / 9.3).
   - The cradle reaches y −50, outside the brief's ±35 envelope, but only under the saddle rail.
5. **Design center:** the flow sits in the funnel band inside the swap column. The ramps leave the swap column free, or concept F settles it.
6. **Spark:**
   - the FC placement (A as built with the grommet requirement, or B on the plate top);
   - the rule-8 clear-air method (column vs contracted slipstream).

## Open
- **Measure:** the FC's stack as built (grommet type and size, spacer, nut, PCB height in the 8.0 envelope, connector entry); the plate's AIO hole size.
- **Vibration:** the vibration log.
- **ESC heat:** the temperature at hover current with the vents.
- **Next:** concept re-review; then the piece pass (topology on these loads; the yokes are light on index).
