# Scout v3 belly mount v2 + the FC on top (27 Oct 2026, concept, system pass)

**Spark 27 Oct (through the cage session): "the fc can live anywhere lets just do on top".** Every Scout rod stays 3.0.

**In one line:** the FC sits on the plate top at x 0 on the Bee35's own AIO holes, on 4 bonded silicone columns (Ø6 x 5.2, 30A: lateral 111 Hz, vertical 242 Hz, E). The belly mount loses everything of the FC. It becomes a U-frame, 25.5 g (E), bolted to the plate through the Bee35's 2 existing side holes: **0 new plate holes for it** (v1 needed 4).
- Gates: `solid_check` PASS (1 body); `system_check` PASS (cage first 6.2, CG 0.2 / 2.1, access 23 / 23, fit 0).
- Fit: every pair of ours passes with no allowance (declared face contacts 0.000 mm³, every other pair ≥ 2.0; full table in `run.log`).
- One requirement on another part: **the tail block's foot tip is trimmed from x −14 to −20.** The alternative that changes no other part (FC turned 45° at x +13.5) needs 4 new holes and puts the IMU 13.5 forward.
- Visual: https://claude.ai/artifact/RZLTp1w4wCQgNe4zNUpYqE (source `concept.html`, both PNGs).

**Files**
- Script `nox/scout/v3/belly_mount_v2.py`. It imports v1's helpers; v1 (`belly_mount.py`, `runs/scout_belly_mount/v1/`) still reproduces and stays as history.
- One run takes about 17 min (the flow search 8, the variants 5). No topology run, so no `estimate.json`.
- Seeded: two full runs gave byte-identical outputs (sha256 of every file in this folder; `run.log` differs only in its 3 timing figures).
- Outputs: `belly_mount_v2__petg_.stl` (the piece), `fc_on_top.stl`, `fc_dampers.stl`, `flow_mtf-02p.stl`, `tail_block_foot_trimmed__requirement.stl` (the requirement drawn), `manifest.json`, `system_check.txt`, `result.json`, `run.log`, `preview.png`, `preview_alternatives.png`, `concept.html`.
- `manifest.json` uses repo-relative paths only: `python3 nox/system_check.py runs/scout_belly_mount/v2/manifest.json` reruns the check.

**Tags:** C computed on layout v12; E estimate; P measured; M bought-part data; F frame model. Printed mass basis as v1: 2.4 walls + 20 % infill.

## 1. The FC on the plate top
**Placed by a search** over x (−6..20, step 0.5) and yaw 0 / 45°. Each pose is checked for:
- the plan margin to every swept disk, by the column measure (rule 8 needs ≥ 3) and by the contracted slipstream;
- 2.0 to the tail block (as drawn and trimmed);
- 2.0 to the cage rods.

| Option | FC pose | Plan margin, column / contracted | Tail block | Holes | IMU x |
|---|---|---|---|---|---|
| A | x +6.0, yaw 0, foot as drawn | **2.50 (FAIL 3.0)** / 6.06 | 2.00, unchanged | 4 new, 6.0 from the AIO ones (2.6 of carbon between) | +6.0 |
| **B (picked)** | **x 0, yaw 0, on the AIO holes** | **6.55 / 10.11** | **foot tip x −14 → −20** (465 mm³, −0.5 g) | **none new** | 0 |
| C | x +13.5, yaw 45, foot as drawn | 4.46 / 8.02 | 2.05, unchanged | 4 new; damper bosses on the centre line | +13.5 |
| D | x +7.5, yaw 45, foot trimmed | 8.71 / 12.26 | foot trimmed | 4 new | +7.5 |

- **Why B:** no new holes; the IMU sits on the thrust centre; the margin is ample; the dampers reuse the holes the FC was built on.
- **What B costs:** the 6 mm wedge tip of the tail block's foot (4..6 tall there). That block is still a concept and unprinted.
- **C** is the pick if the tail block must not change.

**The stack (C on E):**
- plate top 22.0;
- column end disc 0.5, rubber 22.5..27.7, end disc 0.5;
- PCB 28.2..29.8;
- envelope z 25.0..33.0 (3.2 of components each side, E).

Gaps:
- plate 3.0;
- prop sweep (z 37.2) 4.2 above the board, and the board is 6.55 outside the disks in plan;
- props 7.8 (3D).

**Roof:** no cage member over the board's plan (+3). The flat roof's cell is open over the centre, so the roof sets no cap over the FC; the prop sweep does. The cage-first margin is set by `system_check`, below.

**ESC (the AIO) in free air:**
- its 4 sides are 94 % open 3 mm out (1493 of 1584 mm²);
- its top is 100 % open (1296 mm²);
- 3.0 of air under it.

v1 had 641 mm² of side vents, with the plate 1.0 above the ESC and the receiver below.

**USB-C:** clear 250 straight out of either side edge at every offset along it (512 clear on each side edge; 1268 of 11700 overall). Picked: the +y edge, azimuth 0, elevation 0. The battery pads keep the nose edge, toward the lead notch (DEFAULT). Set `SENS_BOARD_ROT` at the bench.

## 2. The soft mount (the v1 review's point)
**Design:** 4 bonded silicone columns (male-female dampers): M3 stud down, M3 female top.
- The PCB is clamped hard onto the column tops with M3 x 5 button screws from above.
- The studs go through the AIO holes into M3 nuts epoxied under the plate.
- Nothing rigid runs beside the rubber in any direction, so the column is the only path: shear sideways, compression vertically. No nut clamps the PCB from below.
- Remove the FC with 4 screws from the top; a damper unscrews from the top.

**Stiffness per column (E):** Timoshenko, guided at the PCB, on Gent's E from the Shore value.
- lateral 1.43 N/mm, vertical 6.83 N/mm;
- the board 11.8 g (E), CG 0.8 over the column tops.

**Modes from the 6-DOF model (E):**

| Mode | Hz |
|---|---|
| x, y | 111 |
| yaw | 137 |
| z | 242 |
| pitch, roll | 290 |

- **Gate ≥ 100: PASS.** The pick rule was lateral ≥ 110 (gate + 10 % for E), then the lowest vertical.
- Isolation at blade pass 500..670 (E): lateral transmissibility 0.05..0.03, vertical 0.31..0.15.
- **Open:** any single column puts the vertical about 2.2 x the lateral. The z mode at 242 Hz sits at the bottom of rotor 1P (~250..335 Hz, E).
  - A softer column (30A x 6.0) gives 99.5 Hz lateral, under the gate.
  - 45° inclined dampers would equalise the two, at the cost of a printed seat.
  - PX4's rotor-RPM notch covers 1P; the vibration log decides.
- **In a 35 g crash:** 0.71 sideways, 0.15 vertical, shear stress 0.036 MPa (E).
- **Wiring:** each lead to the FC needs a 10 mm service loop. A taut 14 AWG lead would bridge the rubber.
- **Bought (E, CHECK the listing):** 4 x M3 male-female silicone damper, Ø6 x 5.2 rubber, 30A; 4 x M3 nut.
- **CHECK:**
  - the plate's AIO hole size (open to Ø3.4 for M3);
  - M3 through the board's Ø3 holes (M2 dampers if not; the rubber sets the stiffness);
  - no part under the PCB within r 5.1 of a hole.

## 3. Wiring (drawn on `preview.png`, plate-top close-up)
- **Motor leads:** from the board corners straight along the arm tops to the motors (on top now; v1 had to bring them under the plate).
- **Battery:**
  - The XT60E1-M's cups point up at (48.25, ±3.6, 14.8). The leads go straight up through **a notch in the plate's nose edge: x 44.0 to the edge (51.0), |y| ≤ 7.5**. The notch takes the empty nose hole (45, 0).
  - They run aft on the plate top to the FC's nose-edge pads (about 30 mm), with a service loop.
  - They are tied to the plate's nose web beside the notch. The XT60 floats ±1 in its carrier, so the leads keep slack below the plate.
- **Receiver (XR2), Radxa UART, UBEC in:** from the FC's tail edge along the plate top beside the fin (|y| 9..14), aft to the bay. About 40 mm shorter than v1's route round the plate edge. Tied to the fin's side faces.
- **Flow:**
  - Its plug is now at the nose end (`SENS_FLOW_ROT` 4, yaw 180 vs as built; CHECK at the bench).
  - The lead runs up the front strut, along the −y rail's outer face to the XT60 window, up through the notch beside the battery leads, then to the FC.
  - About 120 mm; plug-end out 25 clear (access).
- **USB-C:** the +y edge, out sideways under the props (cable z 27.7..34.7).
- **The tail bay cables:** their 0.4 fit box sat inside the FC's new place. It is moved to z 36 over the FC; the 6.27 g CG is unchanged.

## 4. The belly mount without the FC
**Concept:** a U-frame between the plate (z 18.5) and the receiver (z 9.6). It carries nothing of the FC.
- rails |y| 20..26.5 on the receiver's wall tops, x −31 to the front receiver bosses;
- the rear rung, whose tail face at x −31 takes the tongue;
- the ears, x −44..−31, bearing on the tail block's bosses at |y| 21;
- Ø10 bosses (3.0 walls) round:
  - the 4 receiver inserts at (−37.65, ±26.55) and (48.28, ±26.55), unchanged;
  - the 2 plate-bolt inserts at the Bee35's side holes (3.9, ±31): M3 down from the plate top;
- the XT60 window, x 40.2..56.3, |y| ≤ 15.6;
- the flow bipod and cradle, as v1.

**Front:** no front rung. The receiver, bolted at its 4 corners, closes the box. A front rung adds 2.4 g and changes the crash worst by 0.000.

**v1's receiver rib requirement is dropped.** With the FC off the underside nothing needs the 2.1 rib cut or the FC gaps. The receiver is unchanged.

**Mass and gate:** 25.5 g (E; solid 28.8), CG (−0.9, −10.6, 6.8). `solid_check` PASS: 1 body, eroded 1, no necks.

**Variants, built and loaded on the same model (crash target ≤ 0.67 DEFAULT = 1 / 1.5 of an index-1.0 node):**

| Variant | g (E) | New holes | Crash worst | Flight / landing | solid_check |
|---|---|---|---|---|---|
| **VA (picked): side holes only, dampers into bonded nuts** | **25.5** | **0** | **0.523** (frame at x 9.5, −z) | 0.090 | PASS |
| VA, damper pods (inserts in this piece) | 29.1 | 0 | 0.491 | 0.084 | PASS |
| VA, damper rungs | 30.9 | 0 | 0.491 | 0.084 | PASS |
| VA + front rung | 27.9 | 0 | 0.523 | 0.090 | PASS |
| VA, rails 5.5 wide | 25.1 | 0 | 0.602 | 0.103 | **FAIL** (necks) |
| VB: side holes + 2 new (−25, ±13) | 26.9 | 2 | 0.571 | 0.098 | PASS |
| VC: v1's 4 new holes, 2 yokes | 24.6 | 4 | 0.173 | 0.021 | PASS |

- **Pick rule:** inside the targets, the fewest new holes, then the lightest. That gives VA.
- **What VA costs:** 0.9 g over VC and a crash index of 0.52 against 0.17. It is a double cantilever from the side bolts to the receiver corners (44.4 / 41.5 long).
- **Against v1 (25.0 g):** +0.5 g buys 4 fewer holes drilled in the carbon and a closed frame.
- **The dampers' studs** go into bonded nuts rather than this piece, so the FC system stands on the plate alone (3.6 g less than pods). A damper change is still top-only.

## 5. Loads (L, E) on VA
**Model:**
- The hung set (receiver set + swap set, 336.4 g at (6.4, 0, −15.2)) goes into the 4 receiver inserts as an elastic bolt group.
- The U-frame then pries about the worst edge of its footprint on the plate: the bolts take T = k d.
- The bearing sits where both moments close, so the moment diagrams end at 0. That fixes a first-pass artefact at the boss tips.
- Moment about y at an x cut every 1 mm, and about x at y cuts −15..15, each on the cut section with 2.4 walls.

| Kind (limit) | Receiver inserts | Plate inserts (prying) | Frame about y | Frame about x |
|---|---|---|---|---|
| Crash 35 g, 14 directions (1.0) | 0.081 | 0.154 (2.5 x) | **0.523** (x 9.5, −z) | 0.393 (y 15, the rear rung) |
| Flight 4 g / 2 g, landing 6 g (0.5) | ≤ 0.09 overall | | | |

- **Sections:** the weakest x cut is a boss tip (Z 139 mm³ at x 52.5), where the moment is 0. The worst index sits where both rails pass the side bolts. Full table in `result.json` `sections_x`.
- **Front bridge / links (review):** v2 has no bridge. Each rail runs continuous from the tail face to the front receiver boss. The rear rung carries the roll moment at 0.393. The receiver closes the nose.
- **Breaking order (L):**
  - every 3.0 rod already fails at 35 g (`runs/scout_frame/v3`: 25 of 55 buckle, up to 6.9 x Pcr; Spark 27 Oct, rods are the fuse);
  - the belly mount at 0.52 stays under both 1.0 and the 0.67 DEFAULT;
  - a broken belly mount comes out with 2 plate bolts and 4 receiver bolts (pack out first); FC and plate untouched.
- **In flight:** the preloaded joint keeps the rails on the plate. The worst flight / landing pull moment is about 145 N·mm a rail, against roughly 1500 N·mm of preload couple (E).

**Flow bipod (review: the twist):** a 3D frame (struts and tie 6 x 6 PETG, the cradle rigid to the flow CG at y −40, 9.5 off the strut plane).
- crash worst 0.0385 (35 g y), flight 0.0022, landing 0.0026;
- modes 296 / 774 / 1018 Hz (v1's cantilever estimate 405);
- with the CG moved into the strut plane: 0.0385 and 319 Hz. **The twist costs 7 % of the first mode and nothing in strength.**
- A tripod or a strut plane through the CG has no room: the receiver wall and the saddle rail leave a 2.8 corridor.

**FC on the columns:** 11.8 g x 35 g = 4.1 N over 4 columns, 0.036 MPa in the rubber. The bonded nuts see about 1 N each (E).

## 6. Fit (no allowances; full table in `run.log`)
Every pair of ours (belly mount, FC, columns, flow, the trimmed tail block) is checked against every part.

**Declared face contacts (0.000 mm³):**
- belly mount / plate, receiver, lid, tail block, flow cradle;
- columns / plate, FC (the PCB underside);
- tail block / plate, receiver, carrier.

**Every other pair ≥ 2.0.** The closest:

| Pair | Gap |
|---|---|
| FC / tail block (trimmed foot) | 2.00 |
| tail block / TPU end pieces | 2.00 (as PR 130) |
| columns / FC component layer | 2.1 |
| belly mount below z 9 / receiver | 2.20 |
| tail block / Radxa | 2.40 (as PR 130) |
| belly / tie saddle | 2.80 |
| FC / cables box | 2.80 |
| FC / plate | 3.00 |
| FC / props | 7.83 |
| belly / cage links | 9.3 |

`system_check` fit 0.0 mm³. Main's own contact pairs between parts not ours are unchanged.

## 7. Checks (`system_check.txt`)
- **Overall: PASS.**
- Cage first 6.2 (the camera, as main). Shell 47 rods closed. Probes 7.2.
- Bbox 13.8 L (390 x 278 x 127).
- Mass 748 g E (soft over 500).
- **CG 0.2 pack in / 2.1 pack out (gate 3.0; v1 0.2 / 2.2).** The FC rose 15.5 mm but sits on the axis.
- **Access 23 / 23 clear:**
  - the swap ×3;
  - 4 receiver keys from below;
  - the FC USB-C (+y);
  - 4 FC column screws from the top;
  - 2 belly-mount bolts at the side holes from the top;
  - the flow plug;
  - the tail bay's 6 keys and its SD / USB-C tool lines.

## 8. PX4 (FRD from the CG (−0.14, −0.08, 2.98), m)
- `EKF2_IMU_POS` (0.0001, −0.0001, −0.026): the IMU 26 mm over the CG, at the board centre (E: the chip's place is not published).
- `EKF2_OF_POS` (−0.0053, 0.0399, 0.033): the lens, now at the tail-side window.
- `EKF2_RNG_POS` (0.0055, 0.0399, 0.033).
- `SENS_FLOW_ROT` 4 (CHECK).

## 9. Masses and CGs for the cage session (E)

| Piece | g | CG |
|---|---|---|
| Belly mount v2 | 25.5 | (−0.9, −10.6, 6.8) (was 25.0 at (2.1, −10.7, 6.8)) |
| FC | 10.0 | (0, 0, 29.0) (was z 13.5) |
| 4 columns + 4 bonded nuts | 2.4 | (0, 0, 25.1) |
| Wires | 20.0 | (5, 0, 25) (was (0, 0, 13.5)) |
| Flow | 1.5 | (0, −40, −26) |
| Tail block, foot trimmed | 15.44 | (−46.2, 0.7, 12.5) |

## Carbon
None in this piece. The cage rods are the layout's, all 3.0 (Spark 27 Oct).

## Requirements on other parts
1. **Plate (Bee35 carbon):**
   - **no new round holes for the belly mount** (v1's 4 at (−25, ±13), (30, ±13) are dropped);
   - the 2 side holes (3.9, ±31) take the belly mount's M3;
   - the 4 AIO holes take the FC's columns (open to Ø3.4 if smaller; CHECK);
   - 4 M3 nuts epoxied under them;
   - **one notch in the nose edge, x 44.0..edge, |y| ≤ 7.5** for the battery and flow leads. It takes the empty nose hole.
2. **Tail block (electronics session, its piece pass):** **the foot tip trimmed from x −14.0 to −20.0** (the wedge's 465 mm³; its tail-up bearing loses that tip). Rerun its foot x planes. Alternative: FC option C, with no change here.
3. **Receiver:** **v1's request (rib under the FC to ≤ 6.5) is dropped.** No change.
4. **Electronics session (cables):**
   - the 3 FC runs (Radxa UART, XR2, UBEC in) now end at the FC's tail edge on the plate top;
   - the cables' fit box moves to (−8.3, 2.3, 36.0);
   - each FC lead keeps a 10 mm service loop.
5. **Cage session:** the masses and CGs above. The FC on top has no cage member over it (the roof cell is open), and the cage-first margin holds (system_check PASS, worst 6.2 at the camera).

## Carried from the v1 review
- **Front bridge / link sections:** no bridge in v2. The x and y cuts are on the mesh, every 1 mm (section 5).
- **Bipod twist:** 3D frame, section 5.
- **Stale wording:** v1's script is left as merged (history). Every v2 label matches the geometry: no FC window, no rails beside the FC, no USB window.
- **Repo-relative manifest:** done.
- **Seeded and reproducible:** two runs, byte-identical but the timings in `run.log`.
- **No fit allowances, full pairwise table:** done (section 6).

## Open
- The z mode at 242 Hz near rotor 1P (above); the vibration log decides.
- Measure:
  - the AIO hole size;
  - the board's hole size and component keep-out;
  - the damper listing (rubber height, Shore);
  - `SENS_FLOW_ROT` and `SENS_BOARD_ROT`.
- **Next:** concept review; then the piece pass (topology on these loads; the rails carry the 0.52).
