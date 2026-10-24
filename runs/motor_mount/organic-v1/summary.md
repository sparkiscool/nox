motor_mount organic-v1: rerun with the organic finish (demo, not saved to print/)

| Item | Value |
|---|---|
| Engine | nox/topo after the holder round (held ties, MC zero fix, gate member 6.25 mm), before the held-body rule |
| Mass | 17.94 g (M); topo-v3 (drawn finish) 17.21 g |
| Worst index (as built) | 0.143 in 'yaw torque 0.35 Nm' region cases (topo-v3 0.117) |
| Solid check | FAIL: 1 body, 1 eroded body at 0.9 mm, but each bolt pad keeps only 0.75 to 0.78 of its core in the main body at 2.0 mm erosion |
| Function check | ok |
| Wall time | about 4000 s (log.txt restored from the run's stdout: a rebase replaced it under the run) |

What is wrong: the organic skin kept each pad's envelope side (the pad reaches 0.5 mm past the envelope) and opened the
rest to design; that kept side has its own core at 2.0 mm erosion (about 1200 voxels at the outer corner), apart from
the legs. Fix (engine): bodies the main support holds stay whole in the organic finish.
