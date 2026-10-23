holder organic finish: test part for the generative look (demo, not saved to print/)

Spark (23 Oct, via the Nox control conversation): test the organic look on a part with thin supports; holder
replaces bracket_jA as the test part. Targets: (a) thin round struts and branches with open space, smooth fillets into
the bosses, pad, hooks and camera pad, only holes, bores and flat contact faces geometric, no slabs, no voxel steps;
(b) solid_check PASS; (c) worst index <= topo-v1's 0.50; (d) mass within about 15 % of topo-v1's 10.3 g (8.8 to 11.8 g).

## Setup changes (nox/topo/demos/holder.py, written in its docstring)

1. Bolt bosses 11.5 mm round, centred 0.75 mm toward the part, cut flat by the envelope's outer faces (were 6 x 6
   squares with 1.4 mm walls, which the gate cut off; 10 mm round (v1 to v4) and 12 mm round (v5, v6) left stray
   boss cores at 2.0 mm erosion).
2. Keep-out under the pack round the pad from z 29 (was 26; DEFAULT, 1 mm pack clearance instead of 4): the pad sat on
   a 4 mm layer above the battery channel, built 3 mm, too thin for anything that survives the gate's 2.0 mm erosion
   or the optimiser's own eroded design (5.7 mm plates at 6.25 mm members). Battery channel unchanged.

## Engine changes (nox/topo)

1. Held ties: two or more bodies held by the main support (bolt bosses on their seats) are tied to one hub body, so
   none is left loose (the self-test smoke run never joined one corner boss).
2. Build: samples within 1e-4 of the level are nudged outside before marching cubes (exact zeros gave degenerate
   faces, Taubin spikes and voxel slivers on keep-out faces).
3. Gate member margin 2.25 mm (5.0 -> 6.25 mm members): v2's 5.0 mm struts broke at 2.0 mm erosion, v3's 6.25 mm
   struts kept a core.
4. `case_norm="frame"` (option, off): renormalises every case on the first frame. Against the full solid, pad push
   and the tie cases take the whole objective (misuse cases 1e-4 each, organic-v7), but frame did not lower the worst
   index (organic-v8).

## Runs

| Run | Settings | Mass g (M) | Worst (as built) | Solid | Function | Look | Wall time |
|---|---|---|---|---|---|---|---|
| topo-v1 (drawn finish, before) | gen 6.0, 3.2 mm | 10.26 | 0.498 | FAIL (boss walls) | ok | struts, box bosses / pad | 2530 s |
| organic-v1 | gen 4.7, 4.44 mm, held ties | 10.15 | 0.65 | FAIL (2.0 mm erosion splits) | ok | right: struts, arch, fillets | 3502 s |
| organic-v2 | gen 5.3, 5.0 mm, 5 mm layer | 10.93 | 0.529 | FAIL (30 pieces at 2.0 mm; slivers) | ok | right; spikes under the pad | 3333 s |
| organic-v3 | gen 5.6, 6.25 mm, 5 mm layer | 12.52 | 0.399 | FAIL (split at the 4 mm built layer) | ok | right | ~3400 s |
| organic-v4 | gen 5.2, 6.25 mm, 6 mm layer | 10.83 | 0.587 | FAIL (single boss voxels, share 0.996) | ok | right | ~4300 s |
| organic-v5 | gen 5.4, 6.25 mm, 6 mm layer, 12 mm bosses | 11.73 | 1.157 | FAIL (2 bodies) | ok | wrong: two frames | ~3700 s |
| organic-v6 | gen 5.0, 6.25 mm, 7 mm layer, 12 mm bosses | 11.26 | 0.582 | FAIL (boss corner lobes, share 0.96) | ok | right: one frame | ~4200 s |
| organic-v7 | gen 5.4, 6.25 mm, 7 mm layer, offset 11.5 mm bosses | 11.73 | 0.569 | **PASS** | ok | right: one frame | 4260 s |
| organic-v8 | as v7, case_norm frame | 11.72 | 0.596 | **PASS** | ok | right: one frame | 4471 s |
| organic-v9 | as v8, min member 6.6 mm | 11.74 | 1.083 | FAIL (2 bodies) | ok | wrong: two frames | ~4300 s |
