# Scout v3 electronics in the tail: system pass v2 (27 Oct 2026, concept)

**Why v2:** concept v1 (`runs/scout_electronics/v1/`) failed review 3 times, so it went back to the system pass (three strikes rule). Spark, 27 Oct: the Scout has a pointed tail, "fit the stuff there".

**Script:** `nox/scout/v3/electronics_system.py`.
- Stages 1 and 2 by default: optimise the poses, then build the carrier.
- `--rebuild`: stage 3, the cage shell rebuilt round the bay with main's `layout_v3.py` (a patched temp copy).
- `--iterate`: stage 4, re-optimise on the rebuilt shell.

**Geometry:** `runs/scout_layout/v10` on main. Layout v11 (PR 133, held) is designed for:
- the funnel line is a hard constraint;
- the cage rebuilds round this envelope;
- a flat-Radxa pose is in the candidate set.

**Outputs:** `result.json`, `envelope.json`, `run.log`, `system_check.txt`, `system_check_alt_B.txt`, `preview.png`, carrier STLs, and `rebuilt/` (the rebuilt shell with its `system_check.txt` and `recheck.json`).

**Visual:** https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU (v2 section; source `preview.png`). Top row: A on the rebuilt shell. Bottom row: B on v10. Side, top and rear views, with wash rings and tool sweeps.

**Tags:** C = computed on v10 / the rebuilt shell; E = estimate; M = bought-part data; P = measured.

## Method
- **Model:** `nox/scout/v3/scout.assembly.json`, every part in pose, the catch set sliding (DC).
- **CGs:** true centroids: the manifest `cg`, else the volume centroid, else the area-weighted surface centroid. Not bounding boxes; that was v1's error.
- **Search:** 1 mm grid, discrete poses:
  - Radxa: on edge on a carrier wall, or flat on a tray;
  - UBEC (worst 50 x 25 x 10, M): flat on a wall;
  - XR2: antenna vertical, on a shelf.
- **Hard constraints:**
  - shell inset by rod radius + 3 (cage first);
  - wash ≥ 3 (rear disks R 44.5);
  - ≥ 2 to every part;
  - swap column and funnel band;
  - the cage session's funnel line (z −30 at x −100 .. z +20 at x −165, E);
  - FC USB-C path and flow view kept;
  - Radxa microSD and USB-C 1 swept with a Ø10 tool to outside the shell;
  - 8 mm of air over the Radxa's component side.
- **Objective:** worst CG (pack in, pack out), then bay volume, then cable mass. The drawn carrier and 5 cable runs are counted. Joint scan over DC −4..10.
- **Loop:**
  1. Stage 1 on v10: DC −3.
  2. Shell rebuilt round that pick.
  3. Re-optimised: DC 0.0.
  4. Rebuilt and rechecked; the trim moves 0.5, so it has converged.

## Picked: A, box-keel tail bay, catch set at x 0.0
**Radxa Zero 3W:** on edge on the right wall, board vertical (C).
- Board x −110..−78.5, y −11.8..−4, z −15..52.
- Component side out (−y) with 8 of air.
- USB-C 1 reached out of the tail; the microSD comes out up through a roof cell.

**UBEC:** left wall, x −113..−88, y 4..14, z −6..44 (C).

**XR2:** shelf on the right, antenna up, x −128..−112, y −18..−6, z 18..26 (C).

**Carrier:** printed PETG, 27.1 g (E), every boss modelled.
- **Belly-mount tail block**, 9.8 g (E): drawn as the belly mount's own tail, x −74.1..−53.1, |y| ≤ 6.5, z −8..18.5 (C).
  - A bridge over the receiver's tail to x −42.8, with a gusset.
  - 4 M3 heat-set inserts (2 top, 2 bottom), each Ø4 x 6.7 in a Ø10 boss, wall 3.0 (C).
- **Tail carrier**, 16.3 g (E):
  - yoke tongues 5 thick over and under the block, with Ø12 pads, Ø6 x 3 counterbores and a 2.0 web;
  - a box keel 8 wide (|y| ≤ 4) with 2.4 walls, x −128..−52.5, z −15..52;
  - Radxa standoffs 4 x Ø6 (M2.5);
  - XR2 shelf with a 2.0 floor and a 6 x 6 post.
- No cage rod is held. **Carbon: none.**

**Joint:** a 4-bolt clamp, tongues 31.5 apart and bolt pairs 10 apart, so it carries all moments (no stalk).
- All bosses sit inside |y| 6.5, wash +3.1, 2 clear of the receiver and its TPU end pieces, and clear of the plate and the tie saddle (C).
- Keys:
  - top bolts: straight down between the rear props;
  - front bottom: straight up;
  - rear bottom: a ball-end hex key 15° aft. Straight up it grazes the ring's tail rod by about 1 mm³ (C).

**Fuse:** the carrier's yoke tongues; the fix is a reprint with 4 bolts. The block, belly mount and plate stay. The cage is ≥ 7 ahead of every tail part (C).

**Hand loads (E):**

| Case | Result |
|---|---|
| 35 g crash on 46.5 g carried | 16 N; bolt tension 14 N, shear 22 N; keel index 0.05 |
| 50 N on the bay's far corner | bolt tension 115 N, shear 161 N per bolt; bearing 10.7 MPa (index 0.43); keel root 8.8 MPa (index 0.35) |
| First lateral mode | about 480 Hz (rigid root) |

## Checks (A on the final rebuilt shell, C)
| Check | Result |
|---|---|
| `system_check` | PASS (the stage-4 shell also PASS) |
| Probes | 8.5 |
| Access | 14 / 14 |
| Cage first | worst 4.2 (the camera); tail parts ≥ 18 |
| Shell inset | Radxa 7.2, UBEC 8.4, XR2 18.5, keel 8.4 |
| Wash | Radxa +4.3, UBEC +8.0, XR2 +23.7, carrier +3.1 |
| Funnel line | Radxa +1.0, UBEC +9.9, keel +7.3 |
| Tool sweeps | microSD exits after 10.5, 13.0 off any rod; USB-C 1 exits after 45.5, 8.7 off |
| FC USB-C path, swap column, camera and flow views | clear |
| Bay | 106 cm³, x −128..−78 |
| Mass | 715 g all-up (E; v10 679): items 26, carrier 27.1, cables 6.4 = 59.5 g |

## CG and the slide
- **With the catch set at x 0.0:** CG 0.15 / 0.34 pack in / out (C on E masses).
- **At v10's catch (+8.0):** 6.7 / 6.1, which fails the 3 gate.
- **Compared with v10's placeholder:**
  - Its x moment is −2450 g·mm; ours is −4850 (59.5 g at x −81.5).
  - The placeholder had no mount and no cables, and its UBEC and XR2 sat inside the prop disk columns.
  - **The cage session must move the catch set to x 0.0:** 8.0 forward of v10, 10.0 forward of v11.

## Alternatives weighed (C)
- **B, same carrier, no slide:** 6.7 / 6.1, fails. No feasible tail layout balances at catch 8.
- **C, keel 13 wide:** best 0.6 / 2.5 at DC −4; bay 194 cm³.
- **E, Radxa flat on a tray:** best 0.1 / 1.4 at DC −4; bay 194 cm³, board 30 wide over the keel.
- **D, Radxa forward, free poses:** 2.6 / 2.7 at catch 8 with no slide. But the Radxa lands on a cage side wing (y −50..−112), the bay sprawls to 357 cm³, and there is no braced carrier. Rejected.

## Requirements on the cage (Scout v3 design session)
1. Catch set at x 0.0, FC IMU offset x 0.0. Rebuild the shell round `envelope.json`.
2. Use true centroids in `layout_v3.py` `masses()`. Bounding-box centres put the receiver 4.85 off, about 0.7 of CG.
3. Keep these tool lines clear of rods:
   - Ø10 up from (−93.5, −5.6, 52) (microSD);
   - Ø10 aft from (−110, −10.2, 8.5) (USB-C 1);
   - Ø4 down onto (−58.5, 0, 23.5) and (−68.5, 0, 23.5);
   - Ø4 up from (−58.5, 0, −13), and from (−68.5, 0, −13) at 15° aft.
4. Rods ≥ 3 outside the envelope boxes.
5. **Open:** the rebuilt shell at catch 0 came out 310 wide between nodes (v10 242; bbox 14.7 L vs 11.6, C). Only offset 16 fits their 52-rod cap; offsets 20..32 give 56 rods. That is their pick rule, not the bay's width (|y| ≤ 18). The cage session should keep the shell width when it reruns.

## Next
1. Concept review (`review` agent, concept mode).
2. The cage session rebuilds at catch 0.0.
3. Then the piece passes:
   - the belly mount with its tail block;
   - the tail carrier, with its tongues as the fuse.
4. Weigh the Radxa and the XR2, and measure the UBEC (P).
