bracket_jA topo-v3: solid_check PASS (demo run, not saved to print/)

Engine nox/topo @ round 2 (masked eroded filter, gate ties, voxeliser fix), branch topo/new-engine. Setup unchanged.

| Item | Value |
|---|---|
| Mass | 36.04 g (M), drawn 21.4 g; v8 38.37 g (-2.33 g) |
| Worst index, same grid (topo.compare) | 0.125 in 'ring -Z 30 N' at (176.5, 211.5, -58.0); v8 0.133 in 'ring up + fore 30 N' |
| Solid check | PASS: 1 body, 1 eroded body (19,257 mm3), 0 necks, 0 thin walls, drawn_not_in_main none |
| Function check | ok (0 drawn missing, 0 in cuts, 0 outside envelope) |
| Iterations | 119 (37 coarse h 2.0 + 82 fine h 1.0), stopped at max_stage, beta 32, grey 5.0 % |
| Wall time | 727 s (layout 699 s), peak 2.37 GB, Pardiso (141,150 active dofs) |
| Cases | 6 load cases + 24 tie cases + 18 gate ties (flange counts by its cover, 3 cheeks tied to cheek 1) |

Preview (preview.png): the four cheek bosses joined by one thick generated beam along the flange, smooth, no specks.
