scan_stick_head v9: ready to print
Weight: 40.0 g (M), was 40.8 g (v8, M; -0.8 g for the cable hole) / 61.9 g (v2, M)
Strength: worst index 0.36 (F, as-built voxel recheck) in lidar impact 50 g 2 at a pad gusset root [-5.2, -18.8, 10.2] mm (M); all 47 cases (E loads: 25 g in 14 dirs, 50 g impact in 6, 100 N side hits, 150 N drop on the lidar, 20 N hand grab, camera 25 / 50 g, 5 N pokes) ≤ 0.36: sturdy
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls) on print/scan_stick_head_v9.stl
Print: flange face down, bed contact 1119 mm² (M), the largest of the 6 faces (pad tops 698); overhang 1895 mm² (M) under the four spider arms and the camera block, tree supports ~5 g (E); file print/scan_stick_head_v9.stl (sha256 b9c1607a9aa3)

Run: rebuild of the v8 layout (`head_topo_v9.py`, topo.run.rebuild, 125 s, M) with a Ø10 cable hole through the flange centre as an exact cut (the base's bore carries the lidar and camera cables up); v8 layout: nox/topo, finish drawn, h 1.5 (C), 12 cm3 generated (E), 131,072 cells / 53,989 design (M), peak 4.3 GB (M).
Layout: 4 arms from the Ø40 bolt flange to the L2 pads (drawn Ø12 pads with 6 x 10 gussets), the camera block tied in; M4 x 12 from the top into the base's inserts, M3 x 12 up into the L2 with driver clearance cut under each pad.
History: v3 stopped (layout change), v4 organic FAIL + 0.87, v5 keep-out sheet, v6 hung (coarse grid), v7 insert-hole floor speck, v8 PASS 40.8 g (no cable hole), v9 PASS.
Previews: preview.png (engine), preview_print.png.
