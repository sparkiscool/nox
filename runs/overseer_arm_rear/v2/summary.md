# Overseer rear arm v2 (cage sockets): run result

Waited for: the cage sockets (Spark 27 Oct). Run `arm_topo_v2.py rear`, backend direct, 77 cases (53 load cases), grid 96x40x32 h 1.5, resumed after a 30 min tool timeout (restart from the same command; layout 2445 s, wall 0.75 h, peak 4.6 GB M). Not READY: strength targets missed (below). No PR.

## Solid check (exact file `overseer_arm_rear.stl`, M)
```
runs/overseer_arm_rear/v2/overseer_arm_rear.stl PASS {'bodies_plain': 1, 'bodies_eroded': 1, 'eroded_volumes_mm3': [29944.0], 'necks': [], 'drawn_not_in_main': [], 'thin_walls': [], 'acute_edges': 0, 'ok': True}
```
Note: the in-run check inside the script (`result.json` solid_ok false, drawn_not_in_main 0.0/0.1 at erode 2.0) read FAIL on the pre-export body; the exported file reads PASS. Reported as is; mesh not patched.

## Print orientation (`print_orient.py --all`)
```
runs/overseer_arm_rear/v2/overseer_arm_rear.stl extents [131.7  51.7  45. ] g PETG 56.83 watertight True
   saved: bed    582 mm2  overhang   4605 mm2  height  45.0  first layer 1 islands [591.7] (total 591.7)  thin-island run 0 mm
 +x down: bed      6 mm2  overhang   2939 mm2  height 131.7  first layer 1 islands [13.1] (total 13.1)  thin-island run 0 mm
 -x down: bed      0 mm2  overhang   2835 mm2  height 131.7  first layer 2 islands [6.4, 6.4] (total 12.8)  thin-island run 0 mm
 +y down: bed      3 mm2  overhang   5180 mm2  height  51.7  first layer 1 islands [10.9] (total 10.9)  thin-island run 0 mm
 -y down: bed      0 mm2  overhang   4804 mm2  height  51.7  first layer 1 islands [2.5] (total 2.5)  thin-island run 1 mm
 +z down: bed   1400 mm2  overhang   3635 mm2  height  45.0  first layer 2 islands [490.7, 929.0] (total 1419.7)  thin-island run 0 mm
```
Saved orientation: bed 582 mm2, height 45.0; fits A1 mini.

## Mass and strength
Mass 56.83 g (M; budget 55), volume 44.75 cm3, drawn 25.0 g, rods none (rear).
Worst index per class (result.json, F/E, targets: flight/landing <= 0.25, crash/handling/blow/cage <= 0.5):
- flight: 0.217 ('max thrust + 2 g -x radial + gyro') PASS
- misuse (handling 100 N): 0.578 ('handling 100 N +z') over 0.5, FAIL
- crash: **2.442 'cage crash 11'** at (12.4, -13.1, 44.2), FAIL; also cage crash 10 1.48, 12 1.17, 1 0.87, rest < 0.7. The cage joint loads (rear tip up to 1811 N) exceed what this arm holds; the peaks sit at the root top plate region (z 44), the insert bores.

## Modes (modes.json, FE h 1.0, arm 56.6 g; target >= 190 Hz)
- on faces: 169.1 lateral, 187.1 vertical, 450.5, 514.0
- on inserts alone: **157.8 lateral, 183.5 vertical**, 438.5, 500.9
Both first modes under 190 Hz: FAIL.

Preview: `preview.png`. Not ready to print; no slicing prompt.
