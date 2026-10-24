scan_stick_unit v1: FAILED (strength and gate), not saved
Weight: 455.1 g (M) for the whole unit; split: base 356.7 g, head 98.3 g (M, PETG 1.27 E)
Strength: worst index 2.00 (F, as-built voxel recheck) in unit 50 g 3 (-y knock) at [-7.4, 31.0, 179.0] mm (M): the head just above the joint, beside the -x/+y L-key keyway
Solid: base_v19 FAIL (677 plain bodies, 1 wall 1.12 mm, M); head_v14 FAIL (2428 plain bodies, 44 eroded bodies, 12 walls 1.58-1.66 mm along the 15° tilted keep-outs, M); function check FAIL (0.52 mm3 drawn missing, M)
Print: not saved

Run: unit_topo_v1.py, nox/topo (finish drawn, min member 5.0 mm C after the gate calibration), h 2.0 (C), 55 cm3 generated (E), 491,520 cells / 103,958 design (M, volfrac 0.066), layout 2133 s, peak 9.9 GB (M). The first build was killed for memory (14 GB) in the whole-unit solid check (0.25 mm voxels over 302 mm, E); rebuilt with `--rebuild` (no whole-unit gate; the gate per print file).
Causes: (1) the L-key keyways above the joint (12 x 39 x 24, C) opened the head's ±y sides, where lateral bending under the cantilevered L2 needs material; (2) keep-outs tilted 15° (camera board, view frustum, L2 driver bores, the FOV clip of the envelope) trim the 2 mm voxel field into 1.6 mm slivers and specks; (3) a 0.5 mm sliver over the pack (x -20.5). Fixed in v2 (`unit_topo_v2.py`).
