# S2 top case (Scout v2)

One printed case on top of the plate for every electronic part except the flow sensor (Spark 16 Oct 19:46). Scripts: `v2/s2_top_case/`. Numbers below are this part's own; shared numbers live in `v2/dimensions.md`.

## State (17 Oct 16:50): v1 ready to print, preview sent
```
S2 top case v1: ready to print
Weight: 12.85 g (M), new part (build f 12.6 g, not printed)
Strength: worst index 0.84 in camera plug push / pull 20 N (misuse); weakest spot the right side post at (3.1, -26.4, 36.4); real cases <= 0.54
Print: right side (y min) on the bed, tree supports + brim (946 mm² over 45°, bed contact 8 mm²), PETG Basic; file s2_top_case_v1_print.stl
```
One watertight body, function check 0.07 / 0 / 0 (mm³; boss rim sliver), thin 0, wrinkle 0.21 / 0.28 (M); 66 x 80 x 46 (C). Rebuild: `run_top.py` (run h warm from f), `prune_cut.py h`, `build_top.py h 3 1.25`, `trim_keep.py h`, then the tangency slot (Problem 6).

## What it holds (drone coordinates)
| Part | Where | How | Tag |
|---|---|---|---|
| FC MicoAir743 AIO | diamond at (1.9, 0) on the plate's own holes; stack 10 tall (Spark 17 Oct 11:35), plug room to z 37 | its own grommets; the case never touches it (1 play kept) | P / E |
| UBEC | body 44 x 20 (Spark 17 Oct: 55 with pins) x 10 (E) on edge along y: x -7..3, y ±22, z 37..57 | pins clipped, leads soldered (DEFAULT); VHB on the tape land, 2 zip ties | P / E |
| XR2 | x 5..10, y ±11, z 40.5..53.5 | VHB on the land's other face, inside the same 2 ties | E |
| Radxa Zero 3W | component side down, long side along y, ports to the nose, SD to the left; USB-C faces z 59.0, board 62.16..63.76 | 4 x M2.5 self-tapping from below into the boss columns | C |
| Tape land + ears | land x 3..5, y ±20, z 40..54; ears out to y ±26 at z 44.5..50 (between the tie bands) | drawn | DEFAULT |
| Boss columns | Ø7 at (±11.45, ±28.9), z 63.76..68.11; tops = top contact when upside down | drawn | C |
| Side pads + posts | pad Ø9.5 x 3 at (3.9, ±31), z 26..29 on the cage v2 side hub pads (bolt 4 longer, cage chat); posts 3.8 x 3.0 x z 26..40 in the FC-corner corridor | drawn | C |
| Nose pad + post | pad on the plate at (45, 0); strip y ±2 back to a 3.7 x 4 post x 28.6..32.3 to z 40 | drawn | C |

## Placement (`s2t_place.py`)
Exhaustive pose search: prop keep-out 12 (3D, prop plane 41.2), nothing inside a disk in plan, FC zone, lanes, gaps. A flat Radxa clears the disks by 0.22 in plan wherever it sits, so the keep-out puts it above the props. Top z vs keep-out (C, old UBEC): 12 → 68, 9.5 → 64, 6 → 60. With the 55 UBEC incl. pins nothing fits under the Radxa at 12: pins clipped instead.

## Run history (what failed and why)
- a, b, c: narrow corridors (FC corners, nose, UBEC land inside the tie bands) came out under 2.0, were pruned, and cut the load paths. Fix: drawn side posts (b), nose post (c), land ears (d).
- b, d: the nose pad as a support was never tied in. Fix: only the side pads are supports; the nose pad is tied like any drawn feature (strength checks a little conservative).
- f: built 12.6 g, 0.93; superseded by the cage hub stack and prop plane 41.2.
- The prune re-convergence (`run_top.py` after the layout) dropped volume and cut load paths every time: `prune_cut.py` cuts thin members pass by pass without re-converging.
- h = v1.

## Problems
1. **Needs supports.** Every orientation has ~900 to 1,000 mm² over 45° (the +y overhang filter did not hold as built); right side down is the only one where the posts pass (0.84; plate face down 1.26 across layers). Fix in v2: find why the filter missed (drawn features exempt + polish), or draw 45° chamfers under the arms.
2. **Side bolt stacks sit ~10.2 from the prop sweep** (pad top z 29 on the cage hub, C); everything else 12.01. Set by the shared bolt + cage hub; accepted like the 9.5 nose post exception (DEFAULT, Spark can veto).
3. FC moves from under the plate (upside down) to on top: `SENS_BOARD_ROT 0` (was 8) if right side up, then level calibration.
4. SoC faces the UBEC across 2.0 (heat). Bench the demo load and read the SoC temperature.
5. UBEC thickness E 10 (CHECK). Thicker than ~11 hits the XR2 / land layout.
6. Boss (-11.5, -28.9) touched the generated rim tangentially at x -15 (non-manifold edge for slicers): a 0.3 x 3 x 2.4 slot cut there (0.38 mm³).
7. The 2.5 layer over the board back made a thin member in both builds; pruned. v2: raise the boss tops 1 (top z 69.1) so members there can reach 2.0.
