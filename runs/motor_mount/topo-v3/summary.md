motor_mount topo-v3: solid_check PASS (demo part, assumed dimensions E; not a Nox part, nothing to print/)

Geometry v3: pads Ø13 x 6 (PAD_T 6.0). Engine nox/topo round 2, finish "drawn" (started before the organic merge).

| Item | Value |
|---|---|
| Mass | 17.21 g (M), drawn 8.35 g; v2 16.46 g |
| Worst index | 0.117 in 'crash push 40 N +Y on the motor' at (8.4, -10.0, 9.8); v2 0.075 |
| Solid check | PASS: 1 body, 1 eroded body (8,439 mm3), 0 necks, 0 thin walls, drawn_not_in_main none |
| Function check | ok |
| Iterations | 98 fine (stopped at max_stage, beta 32) after the coarse stage |
| Wall time | 1761 s (layout 1741 s), peak 6.00 GB, Pardiso |

log.txt stops at fine iteration 61: a git merge at 03:10 replaced the file while the run held the old one open.
result.json, density.npz and the checkpoints are complete.
