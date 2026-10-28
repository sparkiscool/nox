# head_mount v5 s2_g60_h4 (sweep 2, 3-rod head)

- `python3 nox/solid_check.py runs/head_mount/v5/s2_g60_h4/head_mount.stl` = PASS (1 body, no thin walls, no acute edges, no necks); output in solid.txt.
- Mass 99.76 g (M), worst index 0.72 (head inertia 13 [-0.58, -0.58, -0.58]) at [369.5, 14.0, 55.5].
- result.json solid_ok is False from the in-run gate: 5 drawn_not_in_main entries (erode 2.0); the CLI check on the saved file reads PASS. Reported, not patched.
- No print file saved, mesh not patched. Files: result.json, preview.png, log.txt, solid.txt.
