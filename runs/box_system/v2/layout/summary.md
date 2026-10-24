# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent; Y_C 138 (C).
- Swap arm v6 (PR 71): R 113, lane ≥ 159 (C).
- Battery latch v1 (PR 69): receivers in `runs/box_system/v2/latch_ctx/` (M).

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A).
Funnel centre: Y 140 (C) = both drones' catch centre at Y_C + 2 (C; hub: the Overseer moves its catch centre onto its thrust axis; DEFAULT, alt 177 with Y_C 175). Funnel v8 (catch): the opening must pass the lopsided Overseer set (−66.5 / +113.1 from the catch centre, M): the +Y face out 32.5 with 3 a side (C), the −Y face stays; a symmetric ~191 is 17.6 short at +Y (C). The enclosure keeps the 32.5.
Layout artifact: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

## Concept check (docs/generative.md rule 0)
- System line (`concept.assembly.json`): System: Box structure v2 layout, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 4984.3 mm³ (E) (catch (panels v7, hook latch v2) at the R seat/drone receivers + paths (keep-out)) FAIL, bbox 80.6 L (E), extents 416 x 414 x 468 mm (E), mass 18428 g (E), CG [4.0, 139.5, -35.3] (E, no cg_ref), access 18/18 clear
  - Its mass and CG count the bay envelopes as solid PETG: an envelope figure, not a design mass. Its bbox is the sheets' (C).
- **Arm:** the hub, motors, parked heads and drone-insert lane envelopes clear every bay envelope, the catch, the v1 mast and the PSU (0 mm³, M).
- **Access 18 / 18:**
  - Every cartridge comes out to park along its install axis.
  - Every bay body comes out along +Y with the front sheet off (the service door, DEFAULT).
- **Fit FAIL (Overseer drone set vs the funnel):** the cartridge spans Y 74.8..248.8 against the opening's 56..224 (M).
  - Its tab line sits ~30 from its catch centre, against the Scout's 2: latch v1 puts it on the thrust axis, which is 29.7 ahead of the catch centre (System Box catch).
  - Its 171 length is also more than 168.
  - Hub arbitration (24 Oct): the Overseer moves its catch centre onto its thrust axis (C, within 2); funnel v8 must follow the set (+Y face out 32.5, C); recheck when v8 lands.
- **Option A (DEFAULT):** alternating bays at R 113: 416 x 414 x 468 mm, 80.6 L (C), 0 mm³ between receivers (M).
- **Option B:** Overseer at the bottom 4, receivers clipped: 402 x 414 x 475, 79.1 L (-1.5 L, C).
  - Unclipped, neighbouring Overseer receivers overlap 1020 mm³ (M, `layout_bottom/`); B needs the latch clip.
- **Option C:** bottom slots at R 128: 430 x 414 x 505, 89.9 L (C); the arm strokes move.
- Concept review: the `review` agent on PR 74.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker).

Numbers: `layout.json` (C / M), mirrored in `nox/dimensions.md` System box structure, v2 layout.
