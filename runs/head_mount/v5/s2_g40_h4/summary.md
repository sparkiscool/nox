# head_mount v5 s2_g40_h4 (sweep 2, 3-rod head)

- `python3 nox/solid_check.py runs/head_mount/v5/s2_g40_h4/head_mount.stl` = PASS (1 body, no thin walls, no acute edges, no necks); output in solid.txt.
- Mass 74.0 g (M), worst index 0.983 (head inertia 13 [-0.58, -0.58, -0.58]) at [389.5, 6.0, 83.5].
- result.json solid_ok is False from the in-run gate: 6 drawn_not_in_main entries (erode 2.0); the CLI check on the saved file reads PASS. Reported, not patched.
- No print file saved, mesh not patched. Files: result.json, preview.png, log.txt, solid.txt.
