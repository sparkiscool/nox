scan_stick_head v10: ready to print
Weight: 40.0 g (M), was 40.0 g (v9, M) / 40.8 g (v8, M) / 61.9 g (v2 noxopt, M)
Strength: worst index 0.36 (F, as-built voxel recheck) in lidar impact 50 g 2 at a pad gusset root [-5.2, -18.8, 10.2] mm (M); all 47 cases (E loads: 25 g in 14 dirs, 50 g impact in 6, 100 N side hits, 150 N drop on the lidar, 20 N hand grab, camera 25 / 50 g, 5 N pokes) ≤ 0.36 (F): sturdy
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls) on print/scan_stick_head_v10.stl
Print: flange face down, bed contact 1119 mm² (M), the largest of the 6 faces (pad tops 698 mm², M); overhang 1896 mm² (M) under the four spider arms and the camera block, tree supports ~5 g (E); 65.6 x 65.7 x 34 (M); file print/scan_stick_head_v10.stl (sha256 453d14339999)

Run: rebuild of the v8 layout (`head_topo_v10.py`, topo.run.rebuild, 126 s, M) with the v9 Ø10 (C) cable hole through the flange centre and the L2 pad counterbores cut Ø6.0 (C; v9 Ø6.2 left a 2.9 mm pad wall, review of PR 22; M3 socket head Ø5.5 spec) for a 3.0 mm (C) wall; v8 layout: nox/topo, finish drawn, h 1.5 (C), 12 cm3 generated (E), 131,072 cells / 53,989 design (M), peak 4.3 GB (M).
Layout: 4 arms from the Ø40 (C) bolt flange to the L2 pads (drawn Ø12 pads with 6 x 10 gussets, C), the camera block tied in; M4 x 12 (C) from the top into the base's inserts, M3 x 12 (C) up into the L2 with Ø9 (C) driver clearance cut under each pad. Bolted to the base turned +90° (C): camera over the Pi tray side.
History: v3 stopped (layout change), v4 organic FAIL + 0.87 (F), v5 keep-out sheet, v6 hung (coarse grid), v7 insert-hole floor speck, v8 PASS 40.8 g (M, no cable hole), v9 PASS 40.0 g (M, cable hole, 2.9 mm pad wall), v10 PASS.
Previews: preview.png (engine), preview_print.png.
