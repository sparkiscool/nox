# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent; common grip line **Y_C 165** (C; alt 138 or 175, `nox/log.md`).
- Swap arm v6 / v7: R 113, lane ≥ 159, rail disc r 151 at Y 38..73, drone-insert keep-outs from the v7 head (C).
- Battery latch v1 (PR 69): receivers in `runs/box_system/v2/latch_ctx/` (M); receiver bolts up from below into inserts in the bay bodies.

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A). Funnel centre = the grip line, Y 165.

Visuals:
- Four layouts side by side, sections across and along the axle from the concept meshes: https://claude.ai/artifact/Mn5JGDkjEGSQXyPhjqrpxb (source `runs/box_system/v2/layout_views.html`, `nox/box/box_layout_v2_views.py`).
- 3D scene of A: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

System file: `nox/box/box_structure_v2.assembly.json` (= `concept.assembly.json` of A), with both drone-insert keep-outs as parts so the catch and arm can check against them.

## Concept check (docs/generative.md rule 0)
- System line: `concept_check.json` (box 416 x 418 x 468 mm, 81.4 L, access 18/18 clear).
- **Arm:** hub, motors, parked heads, rail disc and both insert lanes clear every bay envelope, the v1 mast and the PSU (0 mm³, M).
- **Access 18 / 18:** every cartridge comes out to park along its install axis; every bay body comes out along +Y with the front sheet off (the service door, DEFAULT).
- **Fit FAILs left, both in the catch's scope:**
  - Overseer drone set vs the funnel panels v7, 772.6 mm³ (M): funnel v8 (opening 191 x 168) clears it.
  - −Y plain panel bottom vs the insert lanes: carriage 524.9 + grabber 434.0 mm³ (M, panels v7). With v8 the grabber clears and the carriage band still meets the bottom lip, 2 x 248.7 mm³ (E, no v8 file yet). Hub decision: the catch notches the lip in panel v9 plain (X ±35, Y 54..73, to r 159). Re-measure when v8 / v9 land.
- **Option A (DEFAULT):** alternating bays at R 113: 81.4 L.
- **Option B:** Overseer at the bottom 4, receivers clipped: 402 x 418 x 475, 79.8 L (−1.6 L); unclipped (B0) neighbouring receivers overlap 1020 mm³, so B needs the clip.
- **Option C:** bottom slots at R 128: 430 x 418 x 505, 90.8 L; the arm strokes change.
- A wins: B saves 1.6 L (2 %) and lowers the CG (Z −52.7 vs −35.2, E), but needs a new receiver clip on the latch and runs every Overseer swap at the bottom; C costs 9.4 L.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker), then the cut requirements.
