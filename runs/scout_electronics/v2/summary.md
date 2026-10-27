# Scout v3 electronics mount v2: the mount on the v12 tail-bay poses (27 Oct 2026, concept, round 4)

**What v2 is now.** The Scout v3 design session (cage owner) sets the tail-bay poses. Its layout v12 (PR 141, on main) moved the catch 5.0 forward of the thrust centre; the Radxa, the UBEC, this mount and the cables ride the belly mount with it. This round draws the mount for v12's poses on v12's cage and fixes round 3's review FAIL (2ef0b72b):
- a real yoke back;
- a one-body root bridged to the plate bolts;
- the plate bolts off the layout's own FC USB-C path;
- sections and modes from the meshes;
- every printed piece checked as one body.

**Superseded.** Rounds 1 and 2 (own pose search on v10, `--keel`) and round 3 (v11, commit 2ef0b72b) are history; v1 (the fin) is in `../v1/`.

**Files**
- Script `nox/scout/v3/electronics_system.py`; a plain run does this pass (about 2 min).
- Geometry: `runs/scout_layout/v12/` and `nox/scout/v3/scout.assembly.json` (main). The layout's 3 placeholder boxes for this mount are dropped from the model.
- Outputs:
  - `result.json` (both concepts, every load case), `envelope.json` (for the cage session), `run.log`;
  - `system_check.txt` (A), `system_check_alt_B.txt`;
  - `preview.png`;
  - the 2 print STLs, `belly_mount_tail_block__petg_.stl` and `tail_carrier__petg_.stl`, both `solid_check` PASS.
- Visual: https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU (from `preview.png`).

**Tags:** C = computed on v12's geometry; E = estimate; M = bought-part data; P = measured.
- CGs are true centroids.
- Printed masses come from the meshes: walls 2.4 + 20 % infill; the block is solid (E).

## Poses (v12, the requirement; not changed here)
| Item | Pose (C) |
|---|---|
| Radxa Zero 3W | On edge on the centre plane: x −138.9..−71.9, y −4.6..3.2, z 8..38 (ports overhang to 39.5). −138.9 includes the 2 mm microSD stub; the board is −136.9..−71.9. Component side +y, SD end aft. Wash +5.7 |
| UBEC | Body x −163.1..−118.1, leads to −171.1 / −110.1; y 11.2..26.6, z 16..24 (v12's 61 x 15.4 x 8 box, E, not measured). 8.0 of air over the Radxa's SoC. Wash +18.8 |
| XR2 (placed here) | On a shelf off the spine at v12's frame place: x −110..−94, y −27..−15, z 23..31, antenna to 45. The shelf sits 8.5 further aft along the spine than in round 3. It is the most forward pose clear of the wash (+3.7), so the lightest shelf. UART to the FC GPS port |

## Concept A (picked): 2 printed pieces
1. **Belly-mount tail block (the root), 23.4 g (E), printed solid.**
   - **Block:** x −69.4..−48.05, y −4.55..6.5, z −8..18.5, 2.0 behind the receiver's TPU end pieces.
   - **Gusset:** bears on the receiver's tail face (x −43.85, z −4..9.6, |y| ≤ 7.2) by design; it takes the tail-down moment.
   - **Web:** runs under the plate forward to x −31.0. It crosses |y| 6.6..9 only in the plate's shadow (ahead of the notch front, x −40.0).
   - **Arms:** out to 2 Ø10 insert bosses at **(−43.2, ±16.0)**. **2 M3 down through the plate's tail tab** (2 drilled Ø3.4 holes, DEFAULT; drilling is fine at these edge distances):
     - edge distance 6.8 to the tab's tail edge (x −50), 7.0 to the notch edge (y 9), 14 to the tab side: 2D or more everywhere;
     - 12.4 from the layout's own FC USB-C path axis (need 11). The path starts as drawn at (−12.56, 0, 13.5); round 3's override is removed.
   - **Fin and foot:**
     - The fin (|y| ≤ 6.5) rises through the tail notch.
     - Its foot is a wedge on the plate top, z 45 at the fin down to 4 tall at x −14, |y| ≤ 8.0.
     - The foot is the tail-up bearing: it presses the plate top while the web presses the plate underside, over a 23 lever.
   - 6 inserts, every one in a Ø10 boss with 3.0 walls.
2. **Tail carrier, 49.3 g (E), one print.**
   - **Yoke:** tongues 13.05 x 5 over and under the block, 4 M3 (Ø6 x 3 counterbores, 2.0 web) into the inserts; the pads are 2.5 off the TPU end pieces.
     - **The yoke's back is its side wall:** 2.0 at y −6.55..−4.55 along the block's −y face, joining both tongues over the block's length.
     - The side wall is thickened outward to the wash line (a cheek, 2.0 → 4.1), with a 3.5 x 6.5 fillet onto the upper tongue.
     - A lower back (|y| ≤ 6.5, z −13..6) runs under the Radxa to x −98.5, into the spine.
     - Round 3's back was a 1.9 strip. A 13-wide back behind the block cannot fit: the Radxa's front is 2.45 behind it.
   - **Spine:** the side wall carries the Radxa's 4 Ø8 posts (screw-only, M2.5 into Ø2.1 pilots).
     - Aft of x −78.5 (where a box first clears the wash), a closed tube 6.5 deep and 20 tall runs along its lower edge.
     - It is cut back round the microSD tool line.
   - **UBEC:** a 2.4 tray on a closed torsion box down to the funnel line + 1.5 (shell inset ≥ 3.5). A cross plate behind the Radxa and a rib under it at x −124 tie it to the spine.
   - **XR2 shelf:** a 2.0 floor and a 6 x 8 post down into the spine tube.
   - **Posts to the block: 2.0** (round 3: 1.4). The block is now 21.4 long, with the yoke bolts 9 apart.

**Mount total:** 72.7 g (E), CG (−85.4, 0.3, 11.6). **Cables (5 runs):** 6.3 g (E), CG (−8.3, 2.3, 31.6). **Carbon: none.**
- Round 3's 38.6 g came from hand formulas and was low.
- The extra grams buy the solid root (foot, web, arms), the spine's closed tube and the UBEC box. They carry the 50 N corner cases.

## Sections, joint and break order (`loads_mesh()`, E)
- **Sections** are cut from the STL meshes along the load path, hollow with 2.4 walls (the block solid):
  - x planes through the carrier every 3 mm;
  - the yoke and block clamped together;
  - block, web and fin aft of the plate bolts;
  - y planes through the web arms;
  - x planes through the foot.
- **Loads:** each section carries the mass aft of it (items, half the cables, the pieces' own volume as point masses). The 50 N cases act at the far item corner (−163.1, 26.6, 24.0), the UBEC's aft outer top.
- **The yoke:** z, pitch and roll go in bearing of the tongues on the block's faces (index ≤ 0.40). The 4 bolts carry x, y and yaw in shear.
  - The fuse is tongue bearing at the bolts: 375 N per bolt.
  - The inserts (570 N lateral, 1074 N pull-out, E) hold past it.
- **Plate bolts:** tension from z and roll; shear from yaw, side load and tail-down (against the receiver bearing, 17.5 lever).

| Case | Fuse (yoke bolts) | Tail block (worst of block, web arms, foot, plate bolts) | Carrier (worst x) | First to go |
|---|---|---|---|---|
| 35 g x | 0.03 | 0.05 (web arms) | 0.045 (−115) | block, only 1.1 x before the carrier |
| 35 g −x | 0.03 | 0.14 (web arms) | 0.045 | block |
| 35 g y | 0.21 | 0.13 (plate bolts) | 0.07 | fuse |
| 35 g ±z | 0 | 0.13 / 0.15 (foot / web arms) | 0.03 | block |
| 50 N y at the far corner | **0.80** | 0.37 (plate bolts 212 N shear) | 0.41 (−112) | **fuse, 1.96 x before the carrier** |
| 50 N +z at the far corner | 0 | 0.44 (foot) | 0.23 | block, 1.96 x |
| 50 N −z at the far corner | 0 | **0.46** (web arm at y −9) | 0.23 | block, 2.05 x |

- **Nothing reaches 1.** The worst is 0.80: the yoke bolts, with 50 N sideways at the UBEC's far corner.
- **Break order:**
  - Sideways and in yaw, the yoke's tongue bearing goes first.
  - In z and pitch, the tail block's web arm or foot goes first.
  - Both are replaced without touching the cage, plate, receiver or carrier.
  - The carrier stays at least 1.5 x behind except at **35 g along +x, where the block leads it by only 1.1 x** (both about 20 x short of failure). Reported, not fixed.
- **Least-I sections:**
  - carrier at x −160.4: A 158, Iy 838, Iz 23477, J 1010;
  - yoke + block at −50.4: A 245;
  - block at −48.5: A 376, Iy 36192, Iz 3823.
  - The loaded weak spots are the carrier at −112 (0.41) and the web arm at y −9 (0.46).
- **First modes (E):** lateral **235 Hz**, vertical **506 Hz**.
  - Method: section EI and GJ along x from the plate bolts to the carried CG (78.5 g at x −108.2), PETG E 2000, rigid joints.
  - Round 3's 238 / 903 Hz came from an assumed box and are withdrawn.

## Checks (concept A, v12 at catch −5.0, C)
| Check | Result |
|---|---|
| `system_check` | Cage first 4.2 (the camera, theirs). Shell 47 rods, closed. Fit 0. Probes 7.2 (UBEC). **Access 16 / 16. Balance FAIL: 3.4 / 6.6** (below). Bbox 13.7 L (387 x 278 x 127), largest extent 387 |
| Wash (≥ 3, plan, rear disks R 44.5 + 3) | Radxa +5.7, UBEC +18.8, XR2 +3.7, **tail block +3.0, carrier +3.0**. The cage session's −3.4 / −4.3 were the placeholder boxes. The real fin (|y| ≤ 6.5), foot (|y| ≤ 8.0) and yoke side wall (y −6.55) follow the strip between the disks; the only material in |y| 6.6..9 is under the plate (its shadow) |
| Funnel line, from v12's ring tail node (−63.9, z −49.6) − 2.1 and its tail point (−231.5, z 41.5) − 1.7, both + 3: **(−66.0, −46.6) → (−233.2, 44.5)** | Radxa +14.9. **UBEC +5.4: not under the line.** Its lowest point is at x −171.1, z 16.0; the line there is z 10.6. XR2 +45.7, tail block +36.7, carrier +3.7 (the UBEC box floor, drawn at line + 1.5) |
| Rods / shell inset (v12 as drawn) | Rods: block 27.0, carrier 16.6, Radxa 20.3, UBEC 13.8, XR2 14.9. Inset: block 14.8, **carrier 3.5** (the UBEC box floor, drawn at ≥ 3.5), UBEC 5.0. The mount sits inside v12's shell: nothing for the cage to grow. No 6/4 member lands on the mount |
| One body | Both pieces 1 body, watertight; `solid_check` PASS |
| Fit list (≥ 2) | Posts to the block 2.0, block to the TPU end pieces 2.0, yoke to the TPU end pieces 2.0, carrier to the plate 2.2, block to the Radxa 2.4, carrier to the receiver 4.2, block to the FC 4.5. **Posts to the plug keep-clear 0.0:** clipped at the board's top edge (z 38), where the plug envelope (E) starts. Contacts by design: carrier on the block (the clamp); block on the belly mount, receiver tail and plate; items on the carrier |
| Height cap 45 (centre strip; the cage session's request, 27 Oct) | Carrier top 39.0, foot top 45.0. The plugs are low 90° USB-C along the top edge, z 38..45 (E, not measured). XR2 antenna to 45 |

| Access line (`system_check`: all clear) | How |
|---|---|
| Radxa microSD | Ø10 aft (−x); out of the shell after 59, 11.4 off any rod |
| Radxa USB-C 1 | Ø10 straight up, 18.4 clear (v12 has no roof rod over it) |
| FC USB-C | The layout's own path from (−12.56, 0, 13.5), unchanged |
| Yoke keys | Top 2 straight up. Bottom front straight down; bottom rear ball-end 15° aft, past the ring's tail rod |
| Plate keys (−43.2, ±16.0) | **Ball-end 25° toward the centreline.** On this script's swept-disk measure the Ø4 key passes 0.7 inside the prop circle at the prop plane: turn that blade off the line, or take the prop off (assembly-time bolts) |

## Balance (the catch rides, everything of ours with it)
- **At v12's catch −5.0:** CG **3.4 / 6.6 pack in / out (E), 763 g: FAIL.** v12 read 0.9 / 0.8 with round 3's 38.6 g placeholder.
- **Best riding catch −13.5:** 2.17 / 2.31, but out as it stands.
  - The XR2 and the spine's box run 3.1 into the rear disks there.
  - The tail block no longer meets the plate's notch and tab, so the root would be redrawn.
- **What fixes it at −5.0:** about **21 g less at x ≈ −100**, or the same moment moved forward. Pack-out binds, at about 1950 g·mm.
  - The piece passes (topology on these loads) are where it comes from.
  - Each 10 g out of the carrier at x −105 recovers about 1.7 mm pack out.
- **UBEC pose region where balance, funnel and wash all hold:**
  - At −5.0: none. The UBEC alone moves at most ~1 mm of CG.
  - At the riding catch −13.5: the UBEC may shift x −25..+8, y 0..+20, z down to −5 (at x 0).

## Requirements (`envelope.json`)
1. **Cage session:** rerun balance with the mount (72.7 g, CG (−85.4, 0.3, 11.6)) and the cables (6.3 g, (−8.3, 2.3, 31.6)). Keep the rods ≥ 3 off the mount (now ≥ 16.6), and keep clear:
   - the plugs, x −99.8..−68.4, y −4..6, z 38..45 (E);
   - the microSD line, Ø10 from (−138.9, −3, 23) along −x;
   - the USB-C 1 line, Ø10 from (−95.4, 1.6, 39.5) straight up;
   - the 6 Ø4 key lines in `envelope.json`.
2. **Belly mount** (not drawn yet; a placeholder box in the model): its tail face at **x −31.0 over |y| ≤ 22**, z 9.6..18.5. That is 10.85 ahead of the layout's box, where the tail block's web and plate-bolt bosses sit under the plate. The receiver bolts at |y| 26.55 are untouched.
3. **Plate:** 2 x Ø3.4 holes at (−43.2, ±16.0) in the tail tab.
4. **Receiver:** its tail face takes the gusset's bearing (z −4..9.6, |y| ≤ 7.2).

## Alternatives weighed
- **B: A + a Ø6 printed strut to the tail point (−231.5, 0, 41.5), under the 45 cap; its snap clip is the fuse.**
  - +4.6 g. Modes 239 / 501 Hz against A's 235 / 506: the strut lies nearly along x and adds no lateral stiffness.
  - It dips 4.9 under the funnel line, passes 1.6 from the Radxa and ties cage deflection into the bay. CG 4.5 / 8.0.
  - **A picked.** Round 3's B (to the roof node, z 56) broke the cap and is withdrawn.
- **Spine as flanges only, no closed tube:** −16 g, but the 50 N corner case reaches index 5.8 (torsion). Out.
- **Spine tube 14 tall:** −3 g, but the carrier comes within 1.46 x of the fuse at 50 N y. Out.
- **UBEC box 6 deep:** −4 g, but the carrier comes within 1.42 x of the block at 50 N z. Out.

## Still failing / open
1. **Balance at v12's catch: 3.4 / 6.6, FAIL.** It needs about 21 g out at x −100 (piece passes) or a cage-session change; the riding slide puts the XR2 and the spine into the wash.
2. **35 g along +x:** the replaceable block leads the carrier by only 1.1 x (indices about 0.05).
3. **Plate keys:** 0.7 inside the swept prop circle on this script's measure (`system_check` reads them clear).
4. The belly-mount and plate requirements above are open; the UBEC and plug sizes are not measured.

## Visual
`preview.png`, also at https://claude.ai/artifact/EiZ5Z7WktXkSQePPHYEPiU. Top row A, bottom row B. Side, top and rear views on v12 at catch −5.0, with the wash rings, the funnel line (red dashed, side view) and the SD and USB-C tool lines.

## Next
1. Concept review.
2. The cage session reruns balance with these masses.
3. Piece passes: the tail block and the carrier (the tongues stay the lateral fuse), both lightened by topology on these loads.
4. Measure the UBEC and the plugs.
