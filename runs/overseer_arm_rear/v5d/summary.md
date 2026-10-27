# Overseer arm rear v5d (arm 2, mirror arm 3), 27 Oct

Script `nox/overseer/arm_topo_v5d.py rear --gen 50` (wrapper chain v5d > v5c > v5b > v5 > `arm_topo_v1.py`; nox/topo, organic, h 1.5). System: cage v3 / layout v5, loads from the cage frame `runs/overseer_arms/v5/arm_loads.json` (rear spoke node 2354 N worst, F on E). L2 braces root on the top-plate block (cage session), not on the arms.

## What it is
- One printed PETG part, between the S500 plates (z 0..45), 2 + 2 M3 heat-set inserts (top u 63.17 v ±15, bottom u 63.74 v ±10).
- 4 x 6/4 carbon tube chords, 63.1 each (C): top z 38.5, bottom z 6.5, from the shoulders (arm x 20, y ±28) to y ±12 at the motor; bore 6.2, 20 mm sockets, full-length 2.5 sleeve, 2.5 blind tip caps. Length: each tube spans root shoulder to motor pad, 63 mm, past the 33 mm break-even (C); the tubes carry the cage crash moment (2354 N at the spoke node, ~300 N·m at the root, E).
- Solid root block, top / bottom flanges from the root to both shoulder sockets, the cage spoke node socket (3 x 6.2 bores) beside the motor, motor pad Ø36 x 6 with M3 slots either hand, Ø8.6 shaft bore, Ø27 screw pocket.

## Checks (M, on the exact files)
- solid_check PASS: `print/overseer_arm_rear_v5d.stl`, `print/overseer_arm_rear_v5d_mirror.stl` (and the run mesh).
- Strength (worst index, target: flight ≤ 0.25, crash / handling / blow ≤ 0.5): thrust + yaw + 2 g 0.042, landing 0.017, own crash 35 g 0.041, handling 100 N 0.156, blow 150 N 0.114, inserts-only reruns ≤ 0.23; **cage crash 11 (corner, 2354 N) 0.941**, at the top plate's corner (3.0..3.4 mm from it: the root joint Spark said to assume, `strength_band.json`); beyond 4.5 mm of the plates the arm's worst is 0.771 at the shoulder socket (`strength_band_4.5.json`). Below 1 everywhere, above the generous 0.5 in that one cage case (cases 10, 1 at 0.6 / 0.4).
- Modes (h 1.5, motor rigid, carbon area-corrected): first 435 Hz on the inserts alone, 481 Hz on the plate faces (target ≥ 190).
- Rod check: all 4 tubes slide in straight from the shoulder, bores clear, both sockets grip 100 %.
- Print: top face down (motor face + root top on the bed), bed contact 3376 mm² (M), 2 first-layer islands 960 / 2454 mm², 45 tall, 132 x 67 x 45, fits the A1 mini; largest flat face.
- Mass 187.4 g PETG (M) + 4 tubes ~18 g (E); was 96 g (v4, 3 mm-free PETG, cage crash 1.09). The extra 91 g buys the cage crash at 2354 N (v4 failed at 1811 N) and the sleeves / caps the gate needed.

## History
v1 (no cage nodes), v2b, v3, v4 (3 strikes: cage crash 1.22 / 1.09) > system pass v5 (6/4 tube chords), v5b (solid FAIL collars, crash 1.47 at the shoulder), v5c (solid FAIL tip caps, crash 1.10), v5d PASS.

Previews: `preview_overseer_arm_rear_v5d.png`, `preview_overseer_arm_rear_v5d_mirror.png`.
