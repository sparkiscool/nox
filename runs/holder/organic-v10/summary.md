holder organic-v10: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 6.0`, min member 6.25 mm, held ties, case_norm solid (final engine) |
| Setup | as organic-v7 |
| Mass | 12.50 g (M): +21 % on topo-v1's 10.3 g (target about 15 %) |
| Worst index (as built) | 0.504 in 'misuse 12' at (28.5, 32.5, 22.5) (misuse 10 mirrored 0.497); every other case <= 0.391 |
| Solid check | **PASS** (`nox/solid_check.py` on the exact STL): 1 body, 1 eroded body, 0 necks, 0 thin walls |
| Function check | ok |
| Wall time | 4148 s, 121 iterations, peak 3.97 GB |

Look: right, the v7 frame with fuller members: round legs and rails, open space, smooth fillets; only holes, bores,
contact faces and envelope-cut boss faces geometric.

What it shows: the worst index falls with mass only slowly (11.26 g 0.582, 11.73 g 0.569, 12.50 g 0.504) and always
sits on the same cell: the 90 deg bend over the battery channel's top long edge behind each hook (topo-v1 peaked
there too, 0.498). Worst <= 0.50 and mass within 15 % do not hold together on this setup.
