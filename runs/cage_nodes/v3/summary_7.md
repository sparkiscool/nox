# cage nodes 7 10 11 13 15 (v3 rerun on main 730510cd), stopped

Stopped on the controller's order (nodes saved as drawn CAD by the cage session). Nothing here is saved or ready: no node has a solid-check PASS.

| node | members 6 / 3 mm | mass (M) | worst index | solid check | files |
|---|---|---|---|---|---|
| 7 | 8 / 0 | 15.18 g | tube to 15 pull 0.011 | FAIL: 3 bodies at erode 2.0 (4636 + 362 + 326 mm3), joins under 2.0 mm | node_7/cage_node_7.stl (mesh unpatched) |
| 10 | 6 / 0 | none | none | not run, stopped at it 95 | node_10/ck.npz |
| 11 | 9 / 0 | none | none | not run, stopped early | node_11/ |
| 13 | 7 / 0 | none | none | never started | none |
| 15 | 6 / 0 | none | none | never started | none |

Notes: 3 runs at once ran out of memory (exit -9); mg backend hung or ran 356 s per iteration, direct about 4 s; the first arm+hub design left node 7 with a thin join, reproduced on the rerun.
