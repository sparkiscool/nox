holder organic-v3: demo run, not saved to print/

| Item | Value |
|---|---|
| Settings | organic finish, `--gen 5.6`, min member 6.25 mm, held ties, marching-cubes zero fix |
| Setup | bosses 10 mm round; keep-out under the pack from z 27 (5 mm layer under the pad) |
| Mass | 12.52 g (M): over the 11.8 g limit; the volume constraint ran 0.14 over (the optimiser fought an unmeetable gate tie) |
| Worst index (as built) | 0.399 in the misuse set |
| Solid check | FAIL: 1 plain body, 1 eroded body at 0.9 mm; at 2.0 mm the part splits at the pad, so every other body reads as loose |
| Function check | ok |
| Wall time | about 3400 s, 105 iterations |

Look: right. Round legs of 6.3 to 6.5 mm (sections), arch and fork round the pad, fillets into the bosses, hooks and
camera pad, no slabs, no steps.

What is wrong: the struts keep a 3.25 mm deep core, but the only way into the pad is the 5 mm layer under it, built
4 mm (0.5 mm off each keep-out), which the 2.0 mm erosion removes. The robust filter's thinnest plate at 6.25 mm
members is 5.7 mm, so the camera pad's gate tie (to the pad) could not close. Fix (setup, DEFAULT): keep-out under
the pack from z 28 (6 mm layer, 5 mm built).

Correction to the organic-v2 note: the 4.0 mm "member" median there was this layer, not the struts. The member rule
is now stated as a margin (2 x 2.0 + 2.25 = 6.25 mm) with the v2 / v3 strut evidence; the size is unchanged.
