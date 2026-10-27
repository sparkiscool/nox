# overseer_arm_front v3: run result (FAILED: solid_check FAIL)

Waited for: front v2's result (27 Oct). Run: `arm_topo_v3f.py front --gen 45`, backend direct, gen 45.0 cm3, h 2.0, 82 iterations (stopped: grey), wall 1.33 h (E: 27 Oct 11:30Z to 12:50Z), peak 10.9 GB (M). Mesh not patched.

## Solid
```
runs/overseer_arm_front/v3/overseer_arm_front.stl FAIL {'bodies_plain': 2, 'bodies_eroded': 1, 'eroded_volumes_mm3': [95853.8], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [{'kind': 'wall', 'thick_mm': 1.5, 'core_mm3': 28.52, 'at': [139.2, 16.4, 37.5], 'span': [6.3, 1.3, 0.6]}, {'kind': 'wall', 'thick_mm': 1.12, 'core_mm3': 11.42, 'at': [87.8, 26.7, 5.6], 'span': [5.1, 1.5, 0.9]}, {'kind': 'wall', 'thick_mm': 1.41, 'core_mm3': 5.77, 'at': [127.4, -16.5, 6.1], 'span': [2.0, 0.8, 0.2]}, {'kind': 'wall', 'thick_mm': 1.12, 'core_mm3': 5.28, 'at': [46.6, 33.6, 39.7], 'span': [1.6, 0.9, 0.2]}, {'kind': 'wall', 'thick_mm': 1.12, 'core_mm3': 4.84, 'at': [91.8, 25.8, 38.8], 'span': [3.4, 0.8, 0.4]}], 'acute_edges': 11, 'ok': False}
```
Result: FAILED. Eroded 2.0 gives 1 body, but 5 thin walls (1.12 to 1.5 mm) and 11 acute edges.

## Print orient (print_orient --all)
```
runs/overseer_arm_front/v3/overseer_arm_front.stl extents [215.3  77.9  46. ] g PETG 165.4 watertight True
   saved: bed   2394 mm2  overhang  10569 mm2  height  46.0  first layer 1 islands [2424.9] (total 2424.9)  thin-island run 0 mm
 +x down: bed      5 mm2  overhang   5224 mm2  height 215.3  first layer 1 islands [13.7] (total 13.7)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   5476 mm2  height 215.3  first layer 1 islands [60.9] (total 60.9)  thin-island run 0 mm
 +y down: bed      4 mm2  overhang  10437 mm2  height  77.9  first layer 3 islands [0.1, 6.2, 8.7] (total 15.0)  thin-island run 1 mm
 -y down: bed      4 mm2  overhang  10223 mm2  height  77.9  first layer 3 islands [0.1, 6.2, 8.7] (total 15.0)  thin-island run 1 mm
 +z down: bed      0 mm2  overhang  12969 mm2  height  46.0  first layer 1 islands [7.5] (total 7.5)  thin-island run 0 mm
```
Fits A1 mini 180: NO (extents 215.3 x 77.9 x 46.0 mm).

## Mass
Arm 165.4 g (M, PETG, 130.23 cm3) + 4 x 3.0 carbon rods 145.7 mm (~1.65 g each, E) = ~172 g (E; modes grid says 171.5 g).

## Worst index (result.json; strength_band arm in brackets)
| class | worst | case | at (mm) | target |
|---|---|---|---|---|
| flight/landing | 0.234 [0.248 arm] | +y side 2 g + max thrust | 156.1, -19.0, 31.2 | <= 0.25 OK |
| crash/blow/cage | 1.937 [2.089 arm] | cage crash 4 | 98.1, 25.0, 35.2 | <= 0.5 MISS |
| handling (misuse) | 0.654 [0.701 arm, inserts only +y] | handling 100 N +z | 156.1, -19.0, 31.2 | <= 0.5 MISS |
Other cage crash misses: 6 (1.31), 7 (1.40), 11 (1.04). Joint split: strength_band printed only arm lines (no separate joint rows).

## Modes (faces / inserts)
103.9 / 102.3 Hz lateral, 122.9 / 119.9 vertical, 267.4 / 264.4 vertical, 322.1 / 319.2 lateral.

## Rod check
Rods 0 to 3: 145.7 long, bore blocked 0.0 mm3, socket grip 0.99 to 1.0, bonded 75 / 59 / 43 / 73 mm: all OK.

Previews: preview.png. Function check ok (skin missing 0). Full checks in checks.txt.
