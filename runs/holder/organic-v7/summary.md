holder organic-v7: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.4`, min member 6.25 mm, held ties, case_norm solid (the engine then) |
| Setup | bosses 11.5 mm round centred 0.75 mm toward the part; keep-out under the pack from z 29 (7 mm layer) |
| Mass | 11.73 g (M), limit 11.8 |
| Worst index (as built) | 0.569 in 'misuse 12' at (28.5, 33.5, 21.5) (misuse 10 mirrored 0.568); every other case <= 0.437 |
| Solid check | **PASS** (`nox/solid_check.py` on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls |
| Function check | ok |
| Wall time | 4260 s, 116 iterations |

Look: right. One frame: four round legs from the bosses to the pad, a ring round the pad, rails along the battery
channel to the hooks, a fork to the camera pad; open space between members, smooth fillets into every drawn body; only
the bolt holes and bores, the pad, hook and camera contact faces and the envelope-cut boss faces are geometric.

What is wrong: worst index 0.569 > 0.50, on the rails at the channel's top edge, across layers. At the end each misuse
case was 1e-4 of the objective: pad push (0.50), gate ties and held ties (ratios 5e3 to 8e3 against the full solid)
set the whole design. Fix (engine): every case renormalised on the first frame (case_norm frame).
