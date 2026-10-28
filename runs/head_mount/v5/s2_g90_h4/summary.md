# head_mount v5 s2_g90_h4 (sweep 2, 3-rod head)

- `python3 nox/solid_check.py runs/head_mount/v5/s2_g90_h4/head_mount.stl` = PASS (1 body, no thin walls, no acute edges, no necks); output in solid.txt.
- Mass 136.45 g (M), worst index 0.342 (head inertia 6 [0.58, 0.58, 0.58]) at [373.5, -18.0, 63.5].
- result.json solid_ok is False from the in-run gate: 3 drawn_not_in_main entries (erode 2.0); the CLI check on the saved file reads PASS. Reported, not patched.
- No print file saved, mesh not patched. Files: result.json, preview.png, log.txt, solid.txt.
