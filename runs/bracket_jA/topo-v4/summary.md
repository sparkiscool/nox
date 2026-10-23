bracket_jA topo-v4: solid_check PASS (demo run, not saved to print/)

Engine nox/topo round 2 final (masked eroded filter, gate ties on their own realization, voxeliser fix), run with
`--set finish=drawn` so it compares with topo-v3. Setup unchanged, generated 12 cm3.

| Item | Value |
|---|---|
| Mass | 36.02 g (M), drawn 21.4 g; v8 38.37 g (-2.35 g) |
| Worst index, same grid (topo.compare) | 0.124 in 'ring -Z 30 N' at (176.5, 211.5, -58.0); v8 0.133 |
| Solid check | PASS: 1 body, 1 eroded body (19,262 mm3), 0 necks, 0 thin walls, drawn_not_in_main none |
| Function check | ok |
| Iterations | 124 (37 coarse h 2.0 + 87 fine h 1.0), stopped at max_stage, beta 32 |
| Wall time | 615 s (layout 589 s), peak 2.48 GB, Pardiso |
| Cases | 6 load + 24 tie + 18 gate ties (gate ties solved on the gate realization, a second solve group) |
