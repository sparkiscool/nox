bar_half organic-v2: solid PASS, lighter but weaker than topo-v3

Engine: topo/new-engine @ 611f4e8, default organic finish (2.5 mm skins, fillet 2.5 mm, rims 1.0 mm, pitch 0.333 mm).

| | organic-v2 | topo-v3 (drawn) | old v8 |
|---|---|---|---|
| Mass | 74.94 g (M) | 71.23 g (M) | 77.58 g (M) |
| Worst index, as built | 0.833 (F) | | |
| Worst index, same grid h 1.0 | 0.833 (F) | 0.804 (F) | 0.929 (F) |
| Solid check | PASS | PASS | |

- Worst case: 'box on one tip 50 N up' at [21.5, 53.5, -181.5].
- Solid check: PASS (1 body, 1 eroded body, 0 necks, 0 thin walls, 0 acute edges).
- Function check: ok (skin 8737.6 mm3, 0 missing, 0 in cuts, 0 outside envelope).
- Same-grid fit: 0 drawn missing, 0 in cuts, 0 outside envelope.
- Versus v8: -2.64 g, worst 0.929 -> 0.833 (-10%).
- Versus topo-v3: +3.71 g heavier and worst 0.833 vs 0.804 (+4%, slightly worse). topo-v3 stays the better result on both numbers.
- Wall time: 836 s (P) (layout 749 s, build 24 s). Peak memory: 4.63 GB (P).
- Layout stopped at max_stage, it 75, beta 32, grey 4.6%.

Look (preview.png): the middle span is smooth and organic, with filleted lightening holes and a flowing top edge; no voxel steps or slivers seen there. The two end blocks (drawn bodies) read boxy, with faceted triangle shading on their faces; no slivers visible at preview resolution. Verdict: organic in the span, not yet at the ends.

Context: organic-v1 (branch run/bar_half-organic-v1) failed with build slivers before the sliver fix; v2 builds clean.

Note: the engine's --git checkpoint pushes after ckpt 50 were rejected (other sessions pushing to this branch); they are pushed with this commit.
