bracket_jA topo-v2: solid_check FAIL, not READY

Engine nox/topo @ round 2 (masked eroded filter, voxeliser fix), branch topo/new-engine. Setup unchanged.

| Item | Value |
|---|---|
| Mass | 35.87 g (M), drawn 21.4 g |
| Worst index | 0.119 in 'ring up + fore 30 N' (same grid as v8: 0.133) |
| Solid check | FAIL: 1 body, 1 eroded body, 0 necks, 0 thin walls; drawn 0.2 / 0.3 share 0.0, flange 0.4 share 0.501 (erode 2.0) |
| Iterations | 107 (37 coarse + 70 fine), stopped at max_stage, beta 32, grey 4.0 % |
| Wall time | 348 s, peak 1.96 GB, Pardiso (141,150 active dofs) |

Why: at the gate's 2.0 mm erosion the part splits into two halves (3903 / 3890 mm3), one per arm. They join only
through the 4 mm drawn flange, uncovered over Y 160..195. The frozen flange carried that link in the eroded
design. Fixed in the engine by gate ties (topo-v3).
