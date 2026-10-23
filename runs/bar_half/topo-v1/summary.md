# bar_half topo-v1 (nox/topo, branch run/topo-bar_half)

Cloud box: 4 cores, 16 GB. Engine unchanged from `topo/new-engine` @ 90b5afe. Run ended clean (RESULT line), no crash.

| Item | Value |
|---|---|
| Mass | 71.6 g (M), drawn bodies 16.68 g, volume 56,381 mm3, watertight |
| Worst strength index | 0.93 (in-layer) in 'box on one tip 50 N up' at (21.5, 52.5, -181.5) mm, the tip pad; 'tip 50 N down' is the same 0.93 there. Across-layer worst 0.56 |
| Solid check | **PASS**: 1 body, 1 eroded body (43,140 mm3), 0 necks, 0 thin walls, drawn_not_in_main none, 0 acute edges |
| Function check | ok: drawn missing 0, in cuts 0, outside envelope 0 mm3 |
| Iterations | 156 (53 coarse, stopped on beta_stop at beta 4; 103 fine, stopped at max_stage at beta 32, grey 3.6 %) |
| Wall time | 1314 s total (layout 1274 s, build 41 s) |
| Peak memory | 4.59 GB (coarse 0.63 GB) |
| Solver | Pardiso direct on both grids (fine: 322,539 active dofs, estimate 3.8 GB of 11.8 GB budget) |
| Min member | 5.0 mm rods (asked 3.2, raised by the gate-member rule to 2 x 2.0 + 1 mm), plates 4.54 mm, filter R 5.15 mm |
| Grid | h 1.0 mm, (208, 24, 40), 82,896 design cells, generated 45 cm3 target |
| Baseline (result.json) | noxopt v8 (shipped 20 Oct): 77.6 g, worst 0.68, solid PASS then |

Against the baseline: 6.0 g lighter (-7.7 %), worst index 0.93 vs 0.68 (higher, still under 1.0). The peak sits at the tip pad
in both tip cases, so the extra load share goes through the pad root, not the generated beam.

Preview (`preview.png`): one smooth beam, both end blocks fused, no specks, holes or slivers.

Not READY or saved to `print/`: this was a run-only task (no print_orient, no print/ copy, no current-parts.md edit).
