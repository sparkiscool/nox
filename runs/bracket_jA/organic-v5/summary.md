bracket_jA organic-v5: rerun on the final engine (demo, not saved to print/)

| Item | Value |
|---|---|
| Engine | final: held ties, MC zero fix, gate member 6.25 mm, case_norm solid, held bodies whole in the organic finish |
| Settings | defaults, gen 12 cm3 |
| Mass | 36.85 g (M); old v8 38.37 g |
| Worst index, same grid (topo.compare) | 0.127 in 'ring -Z 30 N' (old v8 0.133) |
| Solid check | **PASS** (`nox/solid_check.py` on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls |
| Function check | ok |
| Wall time | 666 s, peak 2.49 GB |

Look: the wall flange now stays whole (it is the body the wall support holds), with the smooth beam between it and
the cheeks as in organic-v3 / v4. Still a beam, not members: the design space is a thin crescent between the ring
keep-out and the wall.
