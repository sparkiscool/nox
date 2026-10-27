# head_mount v4 (Overseer sensor head mount): solid_check FAIL

`python3 nox/solid_check.py runs/head_mount/v4/head_mount.stl` = **FAIL**:
bodies_plain 9, bodies_eroded 1 (79017 mm3), acute_edges 4, necks [], thin_walls [], drawn_not_in_main: drawn items 0.5 and 0.6 partly outside the main body after erode 2.0 (0.932, 0.805), 0.7 to 0.10 and later 0.0 (fully lost).

- Worst index (M): 1.056 (guard hit 3 at 378.3, 55.1, 45.4).
- Mass (M): 143.66 g (drawn 109.44 g), volume 113118 mm3, watertight, function_check ok.
- Run: resumed once after a container restart from ck.npz; wall 1.04 h on the second leg, layout 3151 s, peak 12.0 GB (M).
- No print file saved, mesh not patched. FAIL goes back to the design (drawn attachments 0.5 to 0.10 do not stay joined to the main body at erode 2.0).
- Files: head_mount.stl, head_mount_view.stl, preview.png (topology preview), result.json.
