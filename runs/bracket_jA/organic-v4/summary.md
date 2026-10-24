bracket_jA organic-v4: rerun on the final engine (demo, not saved to print/)

| Item | Value |
|---|---|
| Engine | nox/topo after the holder round: held ties, marching-cubes zero fix, gate member 6.25 mm (2 x 2.0 + 2.25), case_norm solid |
| Settings | defaults, gen 12 cm3, min member 6.25 mm (design space routes it to every body) |
| Mass | 36.82 g (M); old v8 38.37 g |
| Worst index, same grid (topo.compare) | 0.113 in 'ring -Z 30 N' (old v8 0.133 in 'ring up + fore 30 N'; organic-v3 0.108) |
| Solid check | **PASS** (`nox/solid_check.py` on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls |
| Function check | ok |
| Wall time | 662 s, peak 2.49 GB |

Look: the same smooth box beam as organic-v3 between the ring keep-out and the wall flange, filling most of the thin
crescent design space; no members with open space between them. Spark moved the look test to holder for this reason.
