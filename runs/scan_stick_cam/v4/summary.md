scan_stick_cam v4: ready to print
Weight: 12.5 g (M), was 10.0 g (drawn v1, M; +2.5 g for the impact cases and Ø12 posts)
Strength: worst index 0.28 (F, as-built voxel recheck) in impact 50 g 4 (was none on v1; v3 read 0.33 (F) before its gate FAIL); weakest spot at [-10.1, 45.4, 0.6] mm (M); all 39 cases (E loads: 25 g, 50 g impact, 10 N pokes, 20 N cable pull) ≤ 0.28 (F)
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls) on print/scan_stick_cam_v4.stl
Print: plate bottom down (the 2 bolt pads + 4 post pads + gussets), bed contact 1169 mm² (M), the largest of the 6 faces (post tops 563 mm², M); overhang 357 mm² (M) under the generated web, tree supports ~1 g (E); file print/scan_stick_cam_v4.stl (sha256 016d187f2b81)

Run: cam_topo_v4.py, nox/topo (finish drawn, min member 5.0 mm (C) after the gate calibration), h 0.75 (C), 5 cm3 generated (E), 90,112 cells / 38,000 design (M), layout 1082 s, build + checks to 1102 s, peak 3.05 GB (M).
History: v2 (organic, 2.5 cm3 E) FAIL necks + worst 0.96 (F); v3 (drawn, Ø8 posts C) FAIL posts at the drawn erosion, 0.33 (F); v4 Ø12 posts (C) PASS.
Previews: preview.png (engine), preview_print.png (render_preview).
