# Scout v3 system pass v5 (WIP): second review fixes, receiver v3 refit (24 Oct)

Candidates on the latch's receiver v3 + skids (part/battery-latch-v6 @ 90cf7f3; bolt pattern moving to (-42.65 / 43.25, +-26.55), refit pending), keel kept (y +-29.6), every rule on. Picture: compare.html (https://claude.ai/artifact/Dm8HQfFbyDoDSPbs6wmzNe).

| | v3 (seen) | warm (from v4) | cold (DEFAULT) |
|---|---|---|---|
| carbon rods / printed parts | 62 / 23 | 97 / 33 | 127 / 52 |
| cage g (E) | 318 | 352 | 443 |
| cage first (system_check) | 4.5 (latch v1) | 3.9 PASS, rods -0.97 in the swap cone | 4.2 PASS |
| probes (500 axes) | 12 rows (v1) | 6 rows, worst -29.1 | 6 rows, worst -15.5 |

Cold open rows: receiver nose top above the nose plate (56.8, 25.3, -0.1) -15.5; flow -4.2; receiver lid corner -4.1; camera -4.1. Fit 0, access PASS, every case <= 1.00 (F).
`cold/`, `warm/`: result.json, checks.txt. DEFAULT cold (the rules win, weight soft); alt warm (lighter, 1 mm short in the swap cone).
