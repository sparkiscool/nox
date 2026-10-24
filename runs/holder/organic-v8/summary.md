holder organic-v8: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.4`, min member 6.25 mm, held ties, case_norm frame (new) |
| Setup | as organic-v7 |
| Mass | 11.72 g (M) |
| Worst index (as built) | 0.596 in 'misuse 10' at (28.5, 7.5, 22.5) (misuse 12 mirrored 0.593); every other case <= 0.446 |
| Solid check | **PASS** (1 body, 1 eroded body, 0 necks, 0 thin walls) |
| Function check | ok |
| Wall time | 4471 s, 127 iterations (a transient tie spike at beta 32 recovered) |

Look: right, same frame as v7.

What is wrong: the worst index did not move (v7 0.569). It sits where the member from each hook column bends 90 deg over
the battery channel's top edge: 8 mm column beside the channel, 5 mm arm over it, inner corner on the keep-out edge.
Tried on this layout by rebuild (no new layout), all worse: surface filter normalised by the non-void share
(12.1 g, 1.04), closing air gaps under 3 mm against cuts (0.711), 1.0 mm hold-off from cuts (0.895). Index vs mass is
flat (v6 11.26 g 0.582, v7 11.73 g 0.569). Next: min member 6.6 mm, so plates are 6.0 mm and the arm fills the 7 mm
layer (6 mm built) instead of 5 mm.
