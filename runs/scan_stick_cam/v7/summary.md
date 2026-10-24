scan_stick_cam v7: ready to print
Weight: 13.35 g (M), was 12.5 g (v4, M; +0.85 g for the 6 mm plate layer and the 10 mm posts)
Strength: worst index 0.32 (F, as-built voxel recheck) in impact 50 g 4 (was 0.28 on v4, F); weakest spot at [11.6, 45.4, 0.1] mm (M), the plate bottom beside the +x bolt pad; all 29 cases (E loads: 25 g, 50 g impact, 10 N pokes, 20 N cable pull) ≤ 0.32 (F), cable pull 0.014 (F)
Solid: solid_check PASS (1 body, 1 eroded body, 0 necks, 0 thin walls) on print/scan_stick_cam_v7.stl
Print: plate bottom down (2 bolt pads + 4 post pads + gussets, 4 islands of 164 / 278 mm², M), bed contact 840 mm² (M), the largest of the 6 faces (post tops 222 mm², M); overhang 699 mm² (M) under the generated web, tree supports ~1.5 g (E); no brim (over 400 mm²); file print/scan_stick_cam_v7.stl (sha256 cf092d1aba74)

Run: cam_topo_v7.py 0.75 4 --post-d 9, nox/topo (finish drawn, min member 5.0 mm after the gate calibration), h 0.75 (C), 4 cm3 generated (E), 135,168 cells / 24,672 design (M, volfrac 0.38), layout 964 s, 985 s total, peak 3.14 GB (M).
Brief (Spark 23 Oct): camera on 10 mm standoffs (C) for the plug and parts on the board's back, M2.5 inserts. On the mesh the 4 posts are Ø9 (C) with the Ø3.2 (E) hole from the plate top (z 6) to the post tops (z 16): 55.4 mm² annulus, r 4.50 (M) at z 6.5, 8, 11, 15.5; at z 8 only the 4 posts cut the section (M), so nothing generated sits under the board.
Change from v6: the plate layer is 6 thick (C, was 4). v6's 4 mm layer was under the engine's 4.53 mm robust plate, so the eroded design split (6 bodies, main 21 %, M) and v6 stopped 75 % grey. Bracket screws M3 x 12 (C, was M3 x 10): 6 pad + 6 in the head's 7 deep insert holes. The board sits 2 mm further out (world x 39.5..41.1, C).
The layout's eroded realization still read the 20 N cable pull as 5.6e4 (it loads 1 mm strips round the USB notch that the erosion removes); the as-built recheck reads 0.014 (F) for that case.
Fit: assembly_view.py, every pair 0 mm3 overlap (M), incl. head x bracket, base x bracket and bracket x camera board.
History: v2 / v3 gate FAILs, v4 4 mm posts (Spark: 10 mm), v5 web wrapped the posts, v6 stopped (4 mm layer). Previews: preview.png (engine), preview_print.png (render_preview).
