scan_stick_head v2: ready to print
Weight: 61.9 g (M), was 86.8 g (-24.9 g); drawn features 36.0 g (M), generated 20.0 cm3 target (DEFAULT, E)
Strength: worst index 0.03 (F, as-built voxel recheck) in lidar 25 g 11 (was none: v1 had no FE); weakest spot the lidar pad under the 25 g (E) knock cases (all 34 cases ≤ 0.03, F)
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls) on print/scan_stick_head_v2.stl
Print: lidar seat face down (4 pads), bed contact 1037 mm² (M), the largest of the 6 faces (boss end 671 mm², sides ≤ 11 mm², M); tree supports ~6 g (E) under the generated body and the camera block; file print/scan_stick_head_v2.stl

Run: head_topo_run.py h 1.75 (C), 20 cm3 (E), 70,304 cells / 48,242 design (M), 34 cases (C), layout stopped on grey 0.4 % (M) at it 88 (1622 s, M), pruned 0, polished build, function check 0.0/0.0/0.0, thin 0, wrinkle p95/p99 0.183/0.236 (floor 0.18/0.21), peak 3.51 GB (M, ru_maxrss), wall 2108 s on 4 cores.
Next: the part is far over-strong (worst 0.03, F): step the generated target down (10 cm3, then 6; E) while the worst index stays ≤ 1.0; measure the pole tube (P) for the cradle.
Previews: preview.png (as printed, from below, front, top); run-frame mesh scan_stick_head_v2_runframe.stl (lidar seat at z = +6).
