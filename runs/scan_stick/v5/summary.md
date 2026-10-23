scan_stick_head v5: FAILED (solid_check FAIL, not ready to print)
Weight: 40.1 g (M), was 30.5 g (v4 FAILED, M) / 61.9 g (v2, M)
Strength: worst index 0.43 (F) at 12 cm3 generated (E) with the impact cases (50 g, 100 N side hits, 150 N drop): sturdy
Solid: solid_check FAIL on one 1.1 mm sheet at a front lidar pad (M): the camera-bracket keep-out box reached z 33 and cut through the pads (setup error, not the layout)
Print: not saved
Fix in the design (v6, `head_topo_v6.py`): keep-out top at z 25 (the bracket tab ends at 24, pads start at 28); same loads and target.
