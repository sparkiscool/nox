# overseer_arm_rear v3: run result

Waited for: rear v2's result (27 Oct)

Run: `python3 nox/overseer/arm_topo_v3.py rear --gen 45`, backend direct, gen 45 cm3, 77 cases (53 load + ties), 27 Oct 09:58 to 10:56 UTC, wall 0.96 h, peak 5.0 GB (M).

## Solid check (the exact file)
```
runs/overseer_arm_rear/v3/overseer_arm_rear.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [46766.2], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
```
Note: the script's own in-run check printed FAIL (drawn_not_in_main [0.0, 0.997, erode 2.0], 1 drawn body at 99.7 %), and result.json has solid_ok false. The gating check `nox/solid_check.py` on the saved STL reads PASS. Mesh not patched.

## Print orientation (print_orient --all)
```
extents [131.7  48.6  45. ] g PETG 80.86 watertight True
   saved: bed    617 mm2  overhang   5595 mm2  height  45.0  first layer 1 islands [629.4] (total 629.4)  thin-island run 0 mm
 +x down: bed      6 mm2  overhang   2844 mm2  height 131.7  first layer 1 islands [12.6] (total 12.6)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   2881 mm2  height 131.7  first layer 2 islands [6.3, 6.3] (total 12.7)  thin-island run 0 mm
 +y down: bed     13 mm2  overhang   5812 mm2  height  48.6  first layer 1 islands [23.8] (total 23.8)  thin-island run 0 mm
 -y down: bed     13 mm2  overhang   5452 mm2  height  48.6  first layer 2 islands [12.6, 15.0] (total 27.6)  thin-island run 1 mm
 +z down: bed   1431 mm2  overhang   4525 mm2  height  45.0  first layer 2 islands [507.7, 959.7] (total 1467.5)  thin-island run 0 mm
```

## Mass (M)
80.86 g PETG, 63.67 cm3, bbox 131.7 x 48.6 x 45.0 mm, 1 body, watertight, fits A1 mini.

## Worst index per class (result.json)
| class | limit | worst | case | at (mm) |
|---|---|---|---|---|
| flight / landing | 0.25 | 0.095 | max thrust + 2 g +y side + gyro | [9.5, -18.4, 0.8] |
| crash / cage | 0.5 | **1.276 FAIL** | cage crash 11 | [12.5, -13.9, 44.2] |
| handling / blow | 0.5 | 0.268 | handling 100 N +y | [11.0, -18.4, 44.2] |

## Modes (FE grid h 1.0, arm + rods 80.3 g)
- faces: 284.6 lateral, 288.5 vertical, 662.0 vertical, 776.6 lateral Hz
- inserts: 268.4 lateral, 274.1 vertical, 623.1 vertical, 755.4 lateral Hz

Function check ok. Preview: preview.png.
