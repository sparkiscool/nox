scan_stick_base v13: FAILED (function check: 4.9 cm3 of drawn material clipped by the layout envelope)
Weight: 328.1 g (M), would have been +28 g over v12 (300.1 g, M) for the Pi tray plate, the restored -Y rail and back plate
Strength: worst index 0.27 (F) in pack impact 50 g 3 at [-8.5, 15.0, 35.0] mm (M); the tray halves v12's 0.56 (F) at the tube / Pi-pad web
Solid: engine check FAIL (4 plain bodies on its own grid; not gated on the exact file: the function check failed first)
Print: not saved

Run: rebuild of the v7 layout (`base_topo_v13.py`, 414 s class, M) with the review fixes of PR 22: 5 mm (C) Pi tray plate x 15..20, y ±32, z 8..99 with Ø8 (C) standoffs, UBEC keep-out narrowed to x ±14 (C), M4 insert holes 6 deep from z 162 (C). The build intersects (field + drawn) with the envelope (`topo/build.py`), and the v7 envelope is y -23..30 (C): the tray beyond y -23 / 30 and the -Y standoffs below y -23 were clipped (3993 + 910 mm3, M), which is also why v12's lower -Y standoff was a sliver. v14 swaps the exact envelope (y ±32) in after the Part is built.
