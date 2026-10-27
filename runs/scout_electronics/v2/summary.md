# Scout v3 electronics mount v2: the mount on the v11 tail-bay poses (27 Oct 2026, concept)

**What v2 is now.** Spark asked the Scout v3 design session (cage owner) to move the tail bay out of the prop wash and to centre the CG. Its layout v11 (PR 133, on main) sets the Radxa and UBEC poses as the system's requirement for this mount. This pass:
- draws the mount for those poses on v11's cage;
- places the XR2;
- checks everything on v11's own system model;
- reports the mass and CG of the mount and cables so the cage session can rerun balance.

**Superseded (kept for the record).** Rounds 1 and 2 optimised the poses themselves on v10 (a box-keel tail bay with the catch at x 0.0). Round 2 failed concept review, and Spark redirected the bay to v11, so their results are withdrawn. Their code is still in the script behind `--keel`; v1 (the fin) is in `../v1/`.

**Files**
- Script `nox/scout/v3/electronics_system.py`; a plain run does this pass (about 2 min).
- Geometry: `runs/scout_layout/v11/` and `nox/scout/v3/scout.assembly.json` (main, dea0b51); catch 3.5 aft.
- Outputs:
  - `result.json` (both concepts), `envelope.json` (for the cage session), `run.log`;
  - `system_check.txt` (A at catch 3.5), `system_check_at_balance.txt` (A at the balancing slide), `system_check_alt_B.txt`;
  - `preview.png`, and the carrier STLs (concept A).

**Tags:** C = computed on v11's geometry; E = estimate; M = bought-part data; P = measured. CGs come from true centroids (manifest `cg`, volume centroid, else area-weighted surface centroid).

## Poses (v11, the requirement; not changed here)
| Item | Pose (C) |
|---|---|
| Radxa Zero 3W | On edge on the centre plane: board x −147.4..−80.4, y −4.6..3.2, z 8..38. Ports edge up (z 38), SD end aft, component side +y (forced by ports up and SD aft). Wash +5.8 |
| UBEC | Flat at x −171.6..−126.6, y 4.2..19.6, z 16..24, as v11 draws it (45 x 15.4 x 8, E; M Q4 worst 50 x 25 x 10, not measured) |
| XR2 (placed here) | On a shelf on the spine's outer wall, x −110..−94, y −27..−15, z 23..31, antenna up to z 45. The most forward pose clear of the wash (+3.7): the smallest moment. UART to the FC GPS port |

## Concept A (picked): a spine on the belly mount's tail
1. **The belly mount's tail block** (drawn as part of the belly mount, which this part owns), 17.2 g (E).
   - Block: x −78.6..−56.6, |y| ≤ 6.5, z −8..18.5, 2 behind the receiver's TPU end pieces.
   - **Root fin:** 13 x 35.4, 3.0 walls, from the block up through the plate's tail notch to z 45 and forward to x −39.7. It sits on the belly mount's tail; over the receiver's tail it bears on the receiver's top **on purpose**, as the belly mount itself does.
   - Gusset down behind the receiver.
   - **2 new M3 through the carbon plate** at (−34, +9) and (−34, −1), into inserts in the belly mount's tail. They take the root moment into the plate just ahead of the notch.
     - The plate gets 2 drilled Ø3.4 holes (DEFAULT).
     - Clear of the FC (7), the tie saddle (y ±36.3..42.3; 20+ away) and the receiver bolts.
   - 4 yoke inserts. Every insert is in a Ø10 boss with 3.0 walls.
2. **Tail carrier, 18.0 g (E).**
   - **Yoke tongues** over and under the block: Ø12 pads, Ø6 x 3 counterbores, 2.0 web; the pads are 2.5 off the TPU end pieces. 4 M3 into the inserts.
   - **Box spine** at y −13.05..−4.6 (2.4 walls, 60 % lightened, E). Its inner wall carries the Radxa's posts. It runs x −78.5..−171.6, z −13..39.5, cut back round the microSD tool line.
   - A narrow neck (|y| ≤ 6.5) carries it forward wherever the wash is narrower than the spine.
3. **UBEC tray and a cross plate** behind the Radxa, from the spine to the tray, 2.4 thick at z 13.6..16, 2.0 g. The UBEC is held with zip ties.
4. **Radxa posts:** 4 x Ø8 x 3, screw-only. M2.5 x 8 thread-forming screws go into Ø2.1 pilots 5.4 deep through the post and the wall. There is no insert, because an M2.5 insert would leave less than a 3 wall. 0.7 g.
5. **XR2 shelf:** a 2.0 pocket floor and a 6 x 6 post into the spine, 0.6 g.

**Mount total:** 38.6 g (E), CG (−87.7, −3.7, 16.6). **Cables (5 runs):** 6.4 g (E), CG (−12.5, 1.4, 32.0); the camera USB-C runs along the Radxa's top edge. **Carbon: none.**

## The joint, its weakest section, the crash order (hand estimate, E, `loads()`)
- **Carried:** 47.3 g at x −124.4.
- **Root section:** the fin over the receiver's tail at x −53.3: 13 x 35.4 with 3.0 walls (Zv 1878, Zl 868 mm³). It is the weakest section between the bay and the plate bolts.
- **Fuse:** the yoke tongues, by head pull-through of the 2.0 web or bearing.

| Case | Yoke bolt T / S (N) | Fuse index | Root index | Plate bolt T (N) |
|---|---|---|---|---|
| 35 g along x | 8 / 4 | 0.02 | 0.01 | 6 |
| 35 g along y | 10 / 47 | 0.13 | 0.05 | 0 |
| 35 g along z | 32 / 0 | 0.06 | 0.02 | 53 |
| 50 N along y at the far corner (−179.6, 0, 45) | 92 / 291 | **0.78** | 0.29 | 0 |
| 50 N along z at the far corner | 190 / 0 | 0.37 | 0.13 | 243 |

- **The fuse goes first in every case:** its index is 2.2 to 2.7 times the root's (the rule asks for 1.5). The tongues are the cheap, reprinted part; the belly mount (with the FC) and the plate stay.
- **Plate bolts:** 243 N worst, against several hundred N of insert pull-out (E).
- The 50 N cases are on the bay (L's handling case is on cage members), so they are extra.
- **First modes (E):** lateral 238 Hz, vertical 903 Hz. Spine root box, PETG E 2000, rigid root (the root's compliance lowers them). That clears the 100 Hz sensor-mount target. It is not gated here, because this is not a sensor mount. The mount needs no more stiffness.

## Checks (concept A, v11 at catch 3.5, C)
| Check | Result |
|---|---|
| `system_check` | Cage first: worst 4.2 is the camera (theirs); Radxa 14.5, UBEC 11.6, XR2 22.5. Shell 44 rods, closed. Fit 0. Probes 5.0 (UBEC). **Access 16 / 16. Balance FAIL: 5.6 / 6.3** (below) |
| Wash (≥ 3) | Radxa +5.8, XR2 +3.7, spine +3.2, block +3.1. Under the carbon plate the plate's own shadow counts |
| Funnel line (v11's ring and tail point at catch 3.5: (−74.5, −46.6) → (−233.7, 44.5), + 3) | Radxa +12.9, UBEC +2.5, spine and tray +4.7, block +36 |
| Rods to the mount (v11 as drawn) | Spine 16.8, tray 19.1, block 15.0, posts 17.1, XR2 shelf 33.3. Shell inset ≥ 4.3 for every mount part. The mount sits inside v11's shell, so the cage does not grow |
| Fit (≥ 2) | ≥ 2 except UBEC to Radxa: **1.0 (their poses)**. The other contacts are by design: the mount pieces with each other, the belly mount, the plate and the receiver top |
| Height cap 45 (centre strip) | Mount top 39.5. Plugs are low 90° USB-C lying along the top edge, z 38..45, cable bent 10 forward (E). XR2 antenna to 45 |
| Access | All clear at catch 3.5; the lines are in the next table |

| Access line | How it clears |
|---|---|
| Radxa microSD | Ø10 aft (−x); leaves the shell after 52.5, 11.4 off any rod |
| Radxa USB-C 1 | Ø10 **tilted 15° to +y** (5.7 clear). Straight up passes 0.1 from the roof's centre rod |
| FC USB-C | From the FC's tail-right edge (−13.1, −12.7, 13.5) in v11's direction (the centre start meets the plate-bolt bosses) |
| Yoke keys | Top 2: ball-end 15° to +y. Bottom front: straight. Bottom rear: ball-end 15° aft, past the ring's tail rod |
| Plate bolts | (−34, +9) straight; (−34, −1) ball-end 15° to −y |

## Balance with the mount in
- **At v11's catch 3.5:** the CG is **5.6 / 6.3 pack in / out (E), 725 g: FAIL**.
  - Without our mount it was 0.8 / 0.9.
  - The mount (38.6 g at x −87.7) and the cables add about −3.5e3 g·mm.
- **Slide that fixes it:** with the bay held at v11's poses, the catch moves to **x −9.0 (12.5 forward)** and the carrier is redrawn there (a longer neck, mount 33.6 g). CG **1.8 / 1.2** (`system_check_at_balance.txt`), carrier wash +3.1.
  - On v11's unchanged cage, the rear bottom yoke key then meets a shell tube (9 mm³). The cage rebuilt at that catch moves its ring with it.
  - With everything of ours riding the catch, the best is −7.25: 1.25 / 1.17. But that pushes the bay forward into the wash (1.2 mm³ into prop 4): out.
- **Or a lighter mount:** the tail block (17.2 g) and the spine (18.0 g) are E sizes before the piece pass. Each 10 g taken out at x −90 recovers about 1.2 mm.

## Pose findings (the cage session's; nothing changed here)
1. **The UBEC is 1.0 from the Radxa's component face.**
   - The rule asks for ≥ 2, plus 8 of air over the SoC; the UBEC overlaps the Radxa's air box over x −145.4..−126.6.
   - The component side must face +y, because the ports are up and the SD end is aft.
   - **Smallest fix:** UBEC y +7 (to y 11.2..26.6). It stays behind the disks (wash +29), and the CG moves 0.15 in y.
2. **UBEC to the rod-hull plane: 2.4** on this script's measure (rod axis hull minus rod radius, against 3). `system_check` cage first gives it 11.6 and probes 5.0 (PASS). Reported only.
3. **The UBEC's worst size** (50 x 25 x 10, M Q4, not measured), grown aft at the tail, crosses the funnel line and the shell inset. If the real one is bigger, grow it forward or outward.
4. **The plug keep-clear stays under 45 without moving the Radxa:** low 90° plugs along the edge (E), z 38..45. Service USB-C 1 is reached on a line tilted 15° to +y.

## Alternative B: spine plus a printed strut to the roof node (−142.2, 0, 61.5), snap-clipped
- The clip goes round the roof rod beside the node: 3.2 bore (Spark 27 Oct coupons). The clip is the fuse.
- B: mount 40.2 g, lateral mode 652 Hz (E), balance slide −9.75: 1.6 / 1.7.
- It rises to z 56 (over the 45 cap), passes 0.9 from the Radxa, and ties cage deflection into the bay.
- A already has 238 Hz without it, so A is picked.

## Visual
`preview.png`: top row A, bottom row B. Side, top and rear views on v11 at catch 3.5, with wash rings and the SD and USB-C tool lines.

## Requirements on the cage (Scout v3 design session; `envelope.json`)
1. Rerun balance with the mount and cables in. That puts the catch at about x −9.0 (FC IMU x +9.0) with the bay at v11's poses, or lighter mount pieces.
2. UBEC y +7 (to y 11.2..26.6).
3. **Keep-clear:**
   - plugs x −108.3..−76.9, y −4..6, z 38..45;
   - Ø10 microSD line from (−147.4, −3, 23) along −x;
   - Ø10 USB-C 1 line from (−103.9, 1.6, 39.5) along (0, 0.26, 0.97);
   - Ø4 yoke keys and plate keys as in the table above.
4. Rods ≥ 3 outside the mount boxes (now 15..33).
5. `nox/scout/v3/README.md:38` (catch 8.0 aft, the old Radxa x) is theirs to update.

## Next
1. Concept review.
2. Then the piece passes:
   - the belly mount with its tail (topology on these reactions);
   - the carrier, with the tongues as the fuse.
3. Measure the UBEC.
