holder organic-v4: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.2`, min member 6.25 mm, held ties |
| Setup | bosses 10 mm round; keep-out under the pack from z 28 (6 mm layer under the pad) |
| Mass | 10.83 g (M) |
| Worst index (as built) | 0.587 in 'pad push' at (21.5, 6.5, 21.5), the battery channel's top edge (misuse 2 / 3 / 10 / 12: 0.52 to 0.53, same spot) |
| Solid check | FAIL by single voxels: 1 plain body, 1 eroded body; the 4 boss cores read 0.996 to 0.999 in the main body at 2.0 mm erosion (needs 0.999) |
| Function check | ok |
| Wall time | about 4300 s, 125 iterations |

Look: right (round struts, open frame, fillets).

What is wrong: (1) 1 to 3 isolated voxels per boss where the 3.4 mm boss wall meets a fused leg (about 4.0 mm
there). Fix (setup): 12 mm bosses cut flat by the envelope (4.4 mm walls toward the part). (2) Worst index 0.587 at
the channel's top edge. Next run gets gen 5.4 (v3 at 12.5 g read 0.399 there).
