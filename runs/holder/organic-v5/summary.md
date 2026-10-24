holder organic-v5: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.4`, min member 6.25 mm, held ties |
| Setup | bosses 12 mm round cut by the envelope; keep-out under the pack from z 28 (6 mm layer) |
| Mass | 11.73 g (M) |
| Worst index (as built) | 1.157 (two bodies: the right half carries nothing to the pad) |
| Solid check | FAIL: 2 bodies (right bosses + camera pad apart from the rest) |
| Function check | ok |
| Wall time | about 3700 s, 99 iterations |

Look: wrong: two separate frames, two bulbs reaching for each other beside the battery channel.

Why: with 6.25 mm members the robust filter keeps plates only from 5.7 mm. The 6 mm layer under the pad (the only
way into it) barely survives the optimiser's eroded design, so every case through the pad read 1e4 to 2e5 x its
solid compliance (pad push 2.2e5 at the end), swamped the objective, and the optimiser lost the link to the right
bosses at beta 8. Fix (setup, DEFAULT): keep-out under the pack from z 29 (7 mm layer, 1 mm pack clearance); the
battery channel is unchanged.
