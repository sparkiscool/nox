holder organic-v9: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.4 --min-member 6.6` (plates 6.0 mm, to fill the 7 mm layer), case_norm frame |
| Setup | as organic-v7 |
| Mass | 11.74 g (M) |
| Worst index (as built) | 1.083 (two bodies) |
| Solid check | FAIL: 2 bodies (right bosses + camera pad apart, as organic-v5) |
| Function check | ok |
| Wall time | about 4300 s |

Look: wrong: two frames.

Why: at this mass, 6.6 mm members are too few to hold every body; the optimiser dropped the link to the right half
(the gate tie on the camera pad read 22 x its frame value at the end). 6.25 mm is the size that both passes the gate
and fits about 11.7 g.
