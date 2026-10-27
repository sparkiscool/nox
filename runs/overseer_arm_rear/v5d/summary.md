# Overseer arm rear v5d (arm 2, mirror arm 3), 27 Oct

Script `nox/overseer/arm_topo_v5d.py rear --gen 50` (chain v5d > v5c > v5b > v5 > `arm_topo_v1.py`; nox/topo, organic, h 1.5 (C)). Requirements: `dimensions.md` System Overseer: arms (v5 system pass). Loads: the cage frame `runs/overseer_arms/v5/arm_loads.json` (rear spoke node 2354 N worst, F on E). The L2 braces root on the cage's top-plate block, not on the arms.

Page (27 Oct): https://claude.ai/artifact/JS9dzFCg1PsPeGfaZYprRc (source `nox/overseer/arm_page_v5d.py` -> `runs/overseer_arms/v5d/arms.html`).
Cage artifact: the cage session republishes with this arm (asked 27 Oct); its URL goes here when it lands.

System: Overseer cage v3 / layout v5, assembled in `runs/overseer_arms/v5d/overseer_arms_v5d.assembly.json` (the layout manifest with the v5d arms placed, front v5d as the current front), `system_check` (`runs/overseer_arms/v5d/system_check.json`): worst index 0.94 in cage crash 11 (M; was 1.47, v5b), cage first 7.7 mm (E), fit 0.0 mm³ (E), bbox 116 L, extents 771 x 675 x 223 mm (E), access 21/21 clear (E), mass 3555 g (E; was 3212 with the layout's arm meshes), CG 5.8 mm aft of the thrust axis, [-5.8, -0.2, -62.5] (E): **balance FAIL (tol 3; was 0.2)**, pack in; 20.6 mm without the swap parts (reported, L exception). The pack position is the layout's (cage session told 27 Oct); the front v6 changes the front mass again.

## What it is
- One printed PETG part between the S500 plates, z 0..45 (C), 2 + 2 M3 heat-set inserts (top u 63.17 v ±15, bottom u 63.74 v ±10, O).
- 4 x 6/4 carbon tube chords, 63.1 each (C): top z 38.5, bottom z 6.5 (C), from the shoulders (arm x 20, y ±28, C) to y ±12 at the motor (C); bore 6.2 (P, Spark's coupons), 20 mm sockets (C), full-length 2.5 sleeve (C), 2.5 blind tip caps (C). Carbon list: each tube spans root shoulder to motor pad, 63.1 (C), past the 33 mm break-even (DEFAULT); the tubes carry the cage crash moment (2354 N at the spoke node, F; ~300 N·m at the root, E).
- Solid root block, top / bottom flanges 6 thick (C) from the root to both shoulder sockets, the cage spoke node socket (3 x 6.2 x 15, C) at SPOKE_NODE rear (-53.3, ±119.7, 25) (C, the cage's), motor pad Ø36 x 6 (C) with M3 slots either hand, Ø8.6 shaft bore (C, CHECK P), Ø27 screw pocket (C).

## Checks (M, on the exact files)
- solid_check PASS: `print/overseer_arm_rear_v5d.stl`, `print/overseer_arm_rear_v5d_mirror.stl` (and the run mesh).
- Strength, worst index (targets `dimensions.md`: flight ≤ 0.25, crash / handling / blow ≤ 0.5): thrust + yaw + 2 g 0.042, landing 0.017, own crash 35 g 0.041, handling 100 N 0.156, blow 150 N 0.114, inserts-only reruns ≤ 0.23; cage crash 11 (corner, 2354 N) 0.941 at the top plate's corner, 3.0..3.4 mm from it (`strength_band.json`): the root joint, which Spark said on 27 Oct to assume good. **In the arm itself (beyond 4.5 mm of the plates) the worst is 0.771 at the shoulder socket (`strength_band_4.5.json`): over the 0.5 crash target, under 1. Spark's decision does not cover it. Reported, not a gate; DEFAULT keep (alt: more volume or a lighter cage load on the rear node).** Cage crash cases 10 and 1 read 0.61 / 0.43.
- Modes (h 1.5, C; motor rigid, carbon area-corrected): first 435 Hz on the inserts alone, 481 Hz on the plate faces (target ≥ 190).
- Rod check: all 4 tubes slide in straight from the shoulder, bores clear, both sockets grip 100 %.
- Print: top face down (motor face + root top on the bed), bed contact 3376 mm², 2 first-layer islands 960 / 2454 mm², 132.1 x 67.4 x 45.0, fits the A1 mini; largest flat face.
- Weight: 187.4 g PETG (M) + 4 tubes 6.1 g (E: 0.0243 g/mm x 252); budget slot 55 g (E, dimensions), so 3.5 x the slot; was 96 g (v4, no tubes, cage crash 1.09 at 1811 N). The extra grams buy the cage crash at 2354 N and the sleeves / caps the gate needed.

## History
v1 (no cage nodes), v2b, v3, v4 (3 strikes: cage crash 1.22 / 1.09) > system pass v5 (6/4 tube chords), v5b (solid FAIL collars, crash 1.47 at the shoulder), v5c (solid FAIL tip caps, crash 1.10), v5d solid PASS.

Previews: `preview_overseer_arm_rear_v5d.png`, `preview_overseer_arm_rear_v5d_mirror.png`.
