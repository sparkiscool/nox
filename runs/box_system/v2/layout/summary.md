# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent, Y_C 138.
- Swap arm v6 (PR 71): R 113, lane ≥ 159.
- Battery latch v1 (PR 69): receivers in `runs/box_system/v2/latch_ctx/` (M).

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A, mirror-symmetric).
Layout artifact: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

## Concept check (docs/generative.md rule 0)
- System line (`concept.assembly.json`): System: Box structure v2 layout, cage first 27.9 mm (M) at +X (receivers + lids (latch v1, 9 bays)), fit 4984.3 mm³ (M) (catch (panels v7, hook latch v2) at the R seat/drone receivers + paths (keep-out)) FAIL, bbox 74.4 L (M), extents 416 x 382 x 468 mm (M), mass 13501 g (M), CG [-0.0, 145.6, -29.3] (M, no cg_ref)
- **Fit FAIL:** the Overseer drone receiver and cartridge at 90° run into the panels v7. The latch v1 Overseer cartridge is 171 long against the 168 opening (catch interface G (2)). This was sent to the latch (PR 69). The Scout set clears.
- **A (DEFAULT), alternating bays at R 113:** 416 x 382 x 468 mm, 74.4 L. Receivers are not clipped, and neighbours overlap 0 mm³.
- **B, Overseer at the 4 bottom slots, receivers clipped to their sectors less 0.3:** 402 x 382 x 475, 72.9 L (-1.5 L). Unclipped, neighbouring Overseer receivers overlap 1020 mm³, so B needs the latch clip.
- **C, bottom slots at R 128:** 430 x 382 x 505, 83.0 L. The arm strokes would move.
- Concept review: the `review` agent on PR 74.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker). It sets the load paths and the cuts.

Numbers: `layout.json` (C / M), mirrored in `nox/dimensions.md` System box structure, v2 layout.
