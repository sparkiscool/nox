# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs: Spark 24 Oct, 9 bays (4 Overseer at -60..-150, 5 Scout); swap arm v6 (PR 71), R 113, Overseer modules 172 long, lane >= 159; battery latch v1 (PR 69), receivers in `runs/box_system/v2/latch_ctx/` (M).
Layout artifact: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

## Concept check (docs/generative.md rule 0)
- System line (`concept.assembly.json`): System: Box structure v2 layout, cage first 26.0 mm (M) at -X (receivers + lids (latch v1, 9 bays)), fit 4984.3 mm³ (M) (catch (panels v7, hook latch v2) at the R seat/drone receivers + paths (keep-out)) FAIL, bbox 80 L (M), extents 402 x 419 x 475 mm (M), mass 13608 g (M), CG [-8.6, 178.7, -44.7] (M, no cg_ref)
- Fit FAIL: the Overseer drone receiver and cartridge at 90 deg against the panels v7 (the latch v1 Overseer cartridge, 171 long, vs the 168 opening; catch interface G (2)). Sent to the latch (PR 69). The Scout set clears.
- Neighbouring receivers: unclipped at R 113, the Overseer pairs overlap 1020.2 mm3 (M). Clipped to their sectors less 0.3 (DEFAULT), 0 mm3.
- Alternative A (DEFAULT): R 113 + clipped receivers: outer 402 x 419 x 475 mm, 80.0 L, floor Z -241, top Z 234.
- Alternative B: R 128, no clip: 430 x 419 x 505 mm, 91.0 L (+11.0 L). The arm strokes move again.
- Concept review: the `review` agent on this PR.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker running). It sets the load paths and the cuts into pieces.

## Layout numbers
See `layout.json` (C / M). Mirrored in `nox/dimensions.md` System box structure, v2 layout.
