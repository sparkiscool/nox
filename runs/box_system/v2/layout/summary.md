# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent; Y_C 138 (C).
- Swap arm v6 (PR 71): R 113, lane ≥ 159 (C).
- Battery latch v1 (PR 69): receivers in `runs/box_system/v2/latch_ctx/` (M).

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A).
Funnel centre: Y 140 = the Scout's catch centre at Y_C + 2 (DEFAULT; alt 177 with Y_C 175).
Layout artifact: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

## Concept check (docs/generative.md rule 0)
- System line (`concept.assembly.json`): System: Box structure v2 layout, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 4984.3 mm³ (E) (catch (panels v7, hook latch v2) at the R seat/drone receivers + paths (keep-out)) FAIL, bbox 74.4 L (E), extents 416 x 382 x 468 mm (E), mass 18321 g (E), CG [4.1, 137.2, -35.5] (E, no cg_ref), access 18/18 clear
  - Its mass and CG count the bay envelopes as solid PETG: an envelope figure, not a design mass. Its bbox is the sheets' (C).
- **Arm:** the hub, motors, parked heads and drone-insert lane envelopes clear every bay envelope, the catch, the v1 mast and the PSU (0 mm³, M).
- **Access 18 / 18:**
  - Every cartridge comes out to park along its install axis.
  - Every bay body comes out along +Y with the front sheet off (the service door, DEFAULT).
- **Fit FAIL (Overseer drone set vs the funnel):** the cartridge spans Y 74.8..248.8 against the opening's 56..224 (M).
  - Its tab line sits ~30 from its catch centre, against the Scout's 2: latch v1 puts it on the thrust axis, which is 29.7 ahead of the catch centre (System Box catch).
  - Its 171 length is also more than 168.
  - Requirement sent to the latch (PR 69): the same tab-to-catch offset for both drones, and the Overseer set within ± 82 of its catch centre.
- **Option A (DEFAULT):** alternating bays at R 113: 416 x 382 x 468 mm, 74.4 L (C), 0 mm³ between receivers.
- **Option B:** Overseer at the bottom 4, receivers clipped: 402 x 382 x 475, 72.9 L (-1.5 L, C).
  - Unclipped, neighbouring Overseer receivers overlap 1020 mm³ (M, `layout_bottom/`); B needs the latch clip.
- **Option C:** bottom slots at R 128: 430 x 382 x 505, 83.0 L (C); the arm strokes move.
- Concept review: the `review` agent on PR 74.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker).

Numbers: `layout.json` (C / M), mirrored in `nox/dimensions.md` System box structure, v2 layout.
