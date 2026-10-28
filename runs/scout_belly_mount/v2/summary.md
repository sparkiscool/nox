# Scout v3 belly mount v2 + the FC on top (round 2, 28 Oct 2026, concept, system pass)

**Spark 27 Oct (through the cage session): "the fc can live anywhere lets just do on top".** Every Scout rod stays 3.0.

**Round 2 answers the concept review of 27 Oct (FAIL, 28208720).** It had 3 blocking findings:
1. the FC's hole pattern;
2. the soft mount's modes against the rotor;
3. the hung set's first mode.

**In one line:** the FC sits on the plate top on its measured holes: the diamond at (1.9, 0) (P), the plate's AIO pattern. It rides on a printed isolator: a base on the plate, 4 inclined rubber mounts (D8xH8 40A, 45°) on the diagonals, and a tray the board bolts to. All 6 modes sit at 114..177 Hz (E), inside the window [110, 184]: every rotor tone in flight is at least 1.4 x above them.
- The belly mount is a U-frame on the Bee35's 2 side holes plus 2 new holes at (−25, ±13) (VB). The hung set's first mode is 115 Hz. Side holes alone (VA) give 65 Hz, under the 100 floor.
- The tail block's foot stays as drawn.
- Gates: `solid_check` PASS on the belly mount, the tray and the base (1 body each).
- `system_check`: see section 7.
- Fit: no allowances.
- Visual: https://claude.ai/artifact/RZLTp1w4wCQgNe4zNUpYqE (source `concept.html`, both PNGs).

**What round 2 costs, honestly**
- **Mass:** the FC system gains about 21 g (E): tray 7.4, base 5.2, mounts and hardware 11.2 g, against 2.4 g in round 1. All-up is 772 g (E).
  - The hard mount (option H) would save almost all of it, with no isolation at all.
  - **What the 21 g buys:** every rotor tone from the lowest flight 1P up is attenuated before the IMU.
- **Height:** the board sits at z 47..55, a 33 mm tower on the plate. That is level with and above the props (plane 41.2), but 12.7 outside the swept disks in plan, with no cage member over it.
  - The camera USB-C (main's route at z ~40 over the centre line) must pass beside the tower.
- **Thin margins:** with the leads, the modes are 114..177 Hz against 110..184. A ±20 % rubber modulus moves them to 104..159 / 124..193: the soft end stays over the 100 gate, the hard end crosses the ceiling. **Buy 40A and check the listing.**

**Files**
- Script `nox/scout/v3/belly_mount_v2.py`. It imports v1's helpers; v1 (`belly_mount.py`, `runs/scout_belly_mount/v1/`) still reproduces and stays as history.
- One run takes about 15 min; no topology run.
- Seeded: two full runs compared by sha256 (see Checks).
- Outputs:
  - `belly_mount_v2__petg_.stl` (the belly mount);
  - `fc_isolator_tray__petg_.stl`, `fc_isolator_base__petg_.stl` (the isolator's tray and base, printed pieces of this system);
  - `fc_mounts.stl`, `fc_on_top.stl`, `flow_mtf-02p.stl`;
  - `manifest.json` (repo-relative paths), `system_check.txt`, `result.json`, `run.log`;
  - `preview.png`, `preview_alternatives.png`, `concept.html`.

**Tags:** C computed on layout v12; E estimate; P measured; M bought-part data; F frame model. Printed mass basis as v1: 2.4 walls + 20 % infill.

## 0. The rotor (review finding 2: derived, 3 blades)
**1P is derived from the hover thrust (E):**
- 748 g gives 1.834 N a motor: T = C_T ρ n² D⁴, Gemfan 3520 (3.5 in, 3 blades, P).
- C_T is bracketed 0.10..0.18 (E). The motor KV is not recorded.

| Quantity | Hz (E) |
|---|---|
| 1P, hover | 365..490 |
| 1P, flight (0.5..4 x hover thrust) | 258..979 |
| Blade pass, hover | 1095..1469 |
| Blade pass, flight | 774..2938 |

Round 1's 500..670 blade pass and its 250..335 1P were wrong (3 blades, and undrived).

**The FC mount's window:** every mode at or under 258 / 1.4 = **184 Hz** (isolation from the lowest flight 1P up). The floor is 110 (the 100 gate + 10 % for E). The other way out, over 2938 x 1.4 = 4113 Hz, is only a hard mount.

## 1. The FC on the plate top (review finding 1: the measured pattern)
- **The truth (P, `dimensions.md` A):** the FC is a 36 x 36 diamond at (1.9, 0), its USB-C at the tail corner. The plate's AIO holes are the 25.5 square turned 45° about (1.9, 0): (19.93, 0), (−16.13, 0), (1.9, ±18.03).
  - Round 1 used a square at (±12.75, ±12.75) wrongly tagged P; that is corrected.
- **Bench check (one line, Spark):** with the FC off, measure the plate's 4 AIO holes: the pattern (25.5 square turned 45° about (1.9, 0)?), the hole size, and which corner the USB-C faces.
- **Placed:** the diamond on its own holes, x 1.9, board z 47.2..55.2.
  - 12.67 outside the swept disks in plan; the base 3.85 (rule 8 ≥ 3 PASS).
  - No cage member over it (+3).
  - IMU at x 1.9.

**Options from the same model (drawn in `preview_alternatives.png`):**

| Option | What | Margin to disks (board / base) | Tail block foot | Holes | Modes |
|---|---|---|---|---|---|
| **B (picked)** | the diamond on the AIO holes; isolator mounts on the diagonals | 12.67 / 3.85 | **as drawn** | **none new** (3 of the 4 AIO holes) | 114..177 (in window) |
| A | the board square (yaw 0) at x 1.9 | 5.23 / 7.65 | its mounts fall on the plate axes, so the rear one sits on the foot's line: tip to **x −27** (couple x 1.76) | 4 new | as B |
| V | round 1: 4 vertical columns on the AIO holes | 12.67 | as drawn | none new | best column 91..225: **no column fits** (a column is ~5 x stiffer along than across, so its modes spread 2.5 x; the vertical lands in 1P) |
| H | hard mount: the diamond on 4 aluminium standoffs | 12.67 | tip to **x −26** (its rear corner sits on the foot) | none new | ~14.5 kHz (E): over blade pass x 1.4, **no isolation** |

**Why B:**
- the only option that isolates with no new hole;
- it leaves the tail block as drawn;
- the diagonals keep the mounts off both the foot's centre line and the side holes.

## 2. The FC isolator (review finding 2: real parts, every mode in the window)
**Parts (E, CHECK the listing):** 4 M3 male-female rubber anti-vibration mounts, VE type, **Ø8 x 8, 40A**. An M3 female thread needs Ø8+ (review), so round 1's Ø6 column is dropped.
- Each mount sits in its radial plane on a board diagonal, inclined 45° from vertical, top inward; midpoint radius 18.
- **Base (PETG, one print, 5.2 g E):**
  - a wedge under each mount, its face square to the mount: a full face contact, with a 2.5 band so no knife edge;
  - Ø10 x 3.5 bosses on 3 AIO holes (front, sides): M3 down into nuts bonded under the plate;
  - 6 x 3.5 links round the front and sides;
  - open at the tail, over the foot; the rear AIO hole sits under the foot and stays empty;
  - the links stay 2.0 under every mount.
- **Tray (PETG, one print, 7.4 g E):**
  - the board's footprint, 3.5 thick;
  - 4 Ø7.4 bosses up to the PCB: the board's M3 x 16 through them into nylocs under the tray;
  - a pad on each mount's top face.
- Nothing rigid runs beside the rubber in any direction.

**Search:** 1170 layouts.
- Mounts D8xH8 / D10xH10; 30 / 40 / 50A; inclined 0..70°; radius 12..24.
- The leads are springs in the model (below).
- 6 layouts fit the window with the base 3 off the disks. Pick: the largest margin to both bounds, then the smallest reach.

**Per mount (E):** axial 11.41 N/mm, transverse 2.27 N/mm. Moving set 23.0 g (board 10, tray, 4 screws and nylocs, the mounts' top halves), CG z 46.3.

| Case | Modes (Hz, E) |
|---|---|
| As drawn | yaw 106, roll 124, pitch 124, y 154, x 154, z 174 |
| **With the leads** | **yaw 114, roll 130, x 133, y 156, x 158, z 177** |
| Rubber −20 % | 104..159 |
| Rubber +20 % | 124..193 |

- **Isolation (E):**
  - at the lowest flight 1P (258) the worst transmissibility is 1 / ((258 / 177)² − 1) = 0.89;
  - at hover 1P (365) it is 0.31;
  - at blade pass (≥ 774) it is ≤ 0.055.
- **The leads (review item 7):** the model treats them as stranded cantilever loops (5 % of solid copper, E): 2 x 14 AWG battery, 12 x 20 AWG motor (3 a corner), 20 x 26 AWG signal.
  - With **loops of 30 / 20 / 25 mm (requirement, DEFAULT)** they add 1.03 N/mm against the mounts' 9.1 N/mm transverse: modes +3..8 %.
  - With round 1's 20 / 15 / 20 loops the battery leads alone would add 3.5 N/mm.
  - The loops are drawn on the plate-top close-up.
- **Crash:** the 23 g set on 4 mounts. 35 g gives about 8 N, under 1 mm of travel (E); the mounts' bonded ends carry it in shear and compression.

**Board mass in the model (review item 9):** 10.0 g is the board (P). The modal mass adds the tray (7.4), 4 M3 x 16 + nylocs (2.4) and the mounts' top halves (3.2): 23.0 g, not round 1's 11.8.

## 3. Wiring (drawn on `preview.png`, plate-top close-up, with the loops)
- **Motor leads:** from the board's corners down to the arm tops, with a 20 mm loop each, then along the arms.
- **Battery:**
  - The XT60E1-M's cups point up at (48.25, ±3.6). Their tops sit at z 14.8, 3.7 under the plate, so the slack goes above the plate.
  - The leads go straight up through **a notch in the plate's nose edge: inner edge x 42.8, |y| ≤ 6.2, corners r 1.5**. The width is the leads at the cups' y ± 1.6 + 1.0; the inner edge takes the empty nose hole (45, 0; it spans x 43.3..46.7) whole, with 0.5 to spare.
  - A silicone edge strip guards the carbon edges.
  - The leads climb to the board's nose corner with a 30 mm loop, tied to the plate's nose web.
- **Flow:**
  - The plug is now at the nose end. `SENS_FLOW_ROT` goes from 4 (as built, `dimensions.md` A) to 0; check at the bench.
  - The lead goes up the front strut, along the −y rail to the XT60 window, up the notch between the two battery leads, then to the board.
- **Receiver (XR2), Radxa UART, UBEC in:** from the board's tail corner (z ~47) down beside the fin (|y| 9..14), aft to the bay. 25 mm loop.
- **USB-C (P: the tail corner, slightly right):** the right edge of the tail corner, 2 from it. The cable is clear 250. The search now covers the whole cage: round 1's grid stopped at x −150.
- **The tail bay cables:** their 0.4 fit box sat inside the FC tower. It is moved to (−34, 12, 40); the 6.27 g CG is unchanged.

## 4. The belly mount without the FC (review finding 3: the hung set)
**Concept:** a U-frame between the plate (z 18.5) and the receiver (z 9.6), carrying nothing of the FC:
- rails |y| 20..26.5 on the receiver's wall tops;
- the rear rung, whose tail face at x −31 takes the tongue;
- the ears on the tail block's bosses at |y| 21;
- the receiver's 4 inserts, unchanged;
- the flow bipod and cradle, as v1;
- no front rung: the receiver closes the box.

**The hung set's modes.** The receiver set and swap set, 336.4 g at (6.4, 0, −15.2), are a rigid body on the 4 inserts. The frame is 3D beams (2.4-wall sections), clamped at its plate bolts.
- **Lower bound:** no plate contact or friction. A rail clamped by one bolt a side bends as a cantilever (review).
- **Target window: first mode 100..184 Hz (DEFAULT).**
  - ≥ 100: the flow sensor rides on this frame.
  - ≤ the lowest flight 1P / 1.4: the pack then isolates the rotor tones instead of riding on them.

| Variant | g (E) | New holes | Hung modes (Hz, F on E) | Crash worst | Flight / landing | Pick rule |
|---|---|---|---|---|---|---|
| VA: side holes only | 25.5 | 0 | **65** / 102 / 180 | 0.523 | 0.090 | fails 100 |
| VA, rails 11.6 | 28.7 | 0 | **81** / 167 / 236 | 0.398 | 0.068 | fails 100 |
| VA + front rung | 27.9 | 0 | 65 / 102 / 180 | 0.523 | 0.090 | fails 100 |
| **VB: side holes + (−25, ±13)** | **26.9** | **2** | **115** / 300 / 635 | **0.571** | **0.098** | **picked** |
| VB, rails 11.6 | 30.2 | 2 | 151 / 324 / 617 | 0.376 | 0.064 | heavier |
| VC: v1's 4 new holes | 24.6 | 4 | **287** / 323 / 621 | 0.173 | 0.021 | inside 1P (258..979) |

- **Pick rule:** `solid_check` PASS; crash ≤ 0.67 (DEFAULT, 1 / 1.5); flight and landing ≤ 0.5; the first hung mode in the window; then the fewest new holes, then the lightest. **That gives VB.**
- **Round 1's VA loses** on the hung set at 65 Hz: its rails cantilever 44 mm off the side bolts.
- VB's second mode (300) and VC's first (287) sit inside the flight 1P band. They are reported, not gated.
- **Against v1 (25.0 g):** +1.9 g. v1 had 4 new holes; VB has 2.

**Loads (L, E), VB:**
- receiver inserts 0.081;
- plate inserts 0.095 (prying 1.9 x);
- frame about y 0.571 at x −1.5;
- roll 0.164.
- **Sections:** cut every 1 mm (`result.json` `sections_x`).
- **Front bridge / links (review):** no bridge in v2; the rails run continuous.
- **Breaking order:** the 3.0 rods fail first at 35 g (Spark 27 Oct, the rods are the fuse).

**Flow bipod (3D frame):**
- crash 0.0385, flight 0.0022, landing 0.0026;
- 296 Hz on the frame held rigid;
- with the CG moved into the strut plane: 319 Hz and the same index. The twist costs 7 % of the mode.

## 5. The tail block (review item 4)
**It stays as drawn.** The isolator's base is open at the tail and the mounts sit on the diagonals, so every part of the FC tower stays ≥ 2.0 off the foot (tip x −14). The board's tail corner at x −23.6 rides at z 47, over the foot's top (~29).
- Its tail-up couple (35 g z) stays 46.7 N, foot index 0.193 (`runs/scout_electronics/v2`).
- **Owner, reconciled:** this session owns the tail block (PR 130). No change is needed this round.
- The trims the other options would need are computed and listed: A −27 (couple x 1.76); H −26 (x 1.67). A trim would land in our next tail-block piece pass, with the couple's increase as stated.

## 6. Fit (no allowances; full table in `run.log`)
Every pair of ours (belly mount, FC, tray, base, mounts, flow, tail block) is checked against every part.

**Declared face contacts (≤ 0.05 mm³):**
- belly / plate, receiver, lid, tail block, flow cradle;
- tray / FC (the PCB on the bosses);
- tray / mounts;
- base / mounts;
- base / plate;
- tail block / plate, receiver, carrier.

**Inside them, checked apart, ≥ 2.0:**
- the base's links and bosses / the mounts;
- the tray / the FC's component layer;
- the belly below z 9 / the receiver set.

Every other pair is ≥ 2.0.

## 7. Checks (`system_check.txt`)
- **Overall: PASS.**
- Cage first 6.2 (the camera, as main). Shell 47 rods closed.
- Probes 7.0: the FC tower's top is now the +Z probe's nearest part (PASS).
- Fit 0.0 mm³. Bbox 13.8 L (390 x 278 x 127).
- Mass 772 g E (soft over 500).
- **CG 0.2 pack in / 2.0 pack out** (gate 3.0).
- **Access 25 / 25 clear:**
  - the swap ×3;
  - 4 receiver keys from below;
  - the FC USB-C (tail corner, right edge);
  - 4 FC screws from the top;
  - 4 belly-mount bolts from the top (side holes + (−25, ±13));
  - the flow plug;
  - the tail bay's 6 keys and its SD / USB-C tool lines.
- **The fit table:** 285 of 285 rows pass, no allowance. Closest pairs:

| Pair | Gap |
|---|---|
| tail block / TPU end pieces | 2.00 (as PR 130) |
| tray / FC components | 2.10 |
| belly below z 9 / receiver | 2.20 |
| tail block / Radxa | 2.40 |
| base links / mounts | 2.80 |
| belly / tie saddle | 2.80 |
| base / tail block | 2.92 |

- **Reproducible:** two full runs of this code gave byte-identical files (sha256), but `run.log`, which differs only in its timing figures.

## 8. ESC cooling (review item 8, E)
- **Air around it:** the AIO's 4 sides and its top are 100 % open. It is 12.7 outside the swept disks: out of the slipstream (round 1's contracted margin 10.1 holds).
- **Hover current at 748 g:** 10.1..12.6 A, from 4..5 g/W on 4S, E. `dimensions.md`'s 6..10 A predates this build.
- **Heat:** the AIO dissipates 2..4 W (E) over 2592 mm² (both faces).

| Air | h (W/m²K, E) | Rise |
|---|---|---|
| Still | 10 | 77..154 K |
| Hover inflow (~3 m/s) | 30 | 26..51 K |
| Forward flight (8 m/s) | 58 | 13..27 K |

- **Rule 8 (heat sources in airflow):** met in forward flight. In hover it holds only through the disks' inflow (E). On the bench in still air it overheats.
- **Log the temperature.** If hover runs hot, move the board into the inflow (lower the tower).

## 9. PX4 (FRD from the CG (−0.15, −0.08, 4.52), m)
- `EKF2_IMU_POS` (0.002, −0.0001, −0.0467): the IMU 47 mm over the CG.
- `EKF2_OF_POS` (−0.0052, 0.0399, 0.0345).
- `EKF2_RNG_POS` (0.0056, 0.0399, 0.0345).
- `SENS_FLOW_ROT` 0 (as built 4).
- `SENS_BOARD_ROT` unchanged (the diamond, as built).

## 10. Masses and CGs for the cage session (E)

| Piece | g | CG |
|---|---|---|
| Belly mount v2 (VB) | 26.95 | (−2.3, −9.9, 7.3) |
| FC | 10.0 | (1.9, 0, 51.2) |
| Isolator tray | 7.38 | (1.9, 0, 43.1) |
| Isolator base | 5.23 | (4.0, 0, 25.8) |
| 4 mounts + 8 M3 sets | 11.2 | (1.9, 0, 34.6) |
| Wires | 20 | (3, 0, 36) |
| Flow | 1.5 | (0, −40, −26) |
| Tail block | 15.92 | unchanged |

## Carbon
None in these pieces. The cage rods are the layout's, all 3.0 (Spark 27 Oct).

## Requirements on other parts
1. **Plate (Bee35 carbon):**
   - 2 new Ø3.4 holes at (−25, ±13) for the belly mount (VB);
   - the 2 side holes (3.9, ±31) take its other M3;
   - 3 AIO holes (front, sides) take the isolator base's M3, with 3 M3 nuts epoxied under them. The rear AIO hole stays empty, under the foot;
   - the lead notch (x 42.8..edge, |y| ≤ 6.2, r 1.5) with an edge strip;
   - v1's holes at (30, ±13) are dropped.
2. **Tail block:** no change.
3. **Receiver:** v1's rib request is dropped. No change.
4. **Electronics session (cables):**
   - the camera USB-C passes beside the FC tower, not over the centre line at z 40;
   - the bay's 3 FC runs end at the board's tail corner with 25 mm loops;
   - the cables' fit box is at (−34, 12, 40).
5. **Cage session:**
   - the masses above;
   - the FC tower's top at z 55 is now the +Z probe's nearest part (7.0, PASS); no cage member is over it.

## Carried from the v1 and round-1 reviews
- **Stale wording:** fixed. The docstring now describes the U-frame and the isolator; `result.json` no longer lists damper inserts in the belly mount.
- **Repo-relative manifest; seeded runs; no allowances; full pairwise table:** done.

## Open
- The rubber's real modulus (the ceiling is crossed at +20 %): measure one mount (load / deflection) before the print.
- The bench check above.
- ESC temperature in hover.
- **Next:** concept re-review; then piece passes for the belly mount, the tray and the base.
