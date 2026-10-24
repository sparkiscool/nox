# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent; common grip line **Y_C 165** (C; alt 138 or 175, `nox/log.md`).
- Swap arm v6 / v7: R 113, lane ≥ 159, rail disc r 151 at Y 38..73, drone-insert keep-outs from the v7 head (C).
- Battery latch v1 (PR 69): receivers in `runs/box_system/v2/latch_ctx/` (M); receiver bolts up from below into inserts in the bay bodies.

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A). Funnel centre = the grip line, Y 165 (C).

Visuals:
- Four layouts side by side, sections across and along the axle from the concept meshes: https://claude.ai/artifact/Mn5JGDkjEGSQXyPhjqrpxb (source `runs/box_system/v2/layout_views.html`, `nox/box/box_layout_v2_views.py`).
- 3D scene of A: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

System file: `nox/box/box_structure_v2.assembly.json` (= `concept.assembly.json` of A), with both drone-insert keep-outs as parts so the catch and arm can check against them.

## Concept check (docs/generative.md rule 0)
- System line (`concept_check.json`): System: Box structure v2 layout, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 772.6 mm³ (E) (catch (panels v7, hook latch v2) at the R seat/drone receivers + paths (keep-out)) FAIL, bbox 81.4 L (E), extents 416 x 418 x 468 mm (E), mass 18406 g (E), CG [4.0, 149.9, -35.2] (E, no cg_ref), access 18/18 clear
- **Arm:** hub, motors, parked heads, rail disc and both insert lanes clear every bay envelope, the v1 mast and the PSU (0 mm³, M).
- **Access 18 / 18:** every cartridge comes out to park along its install axis; every bay body comes out along +Y with the front sheet off (the service door, DEFAULT).
- **Fit FAILs left, both in the catch's scope:**
  - Overseer drone set vs the funnel panels v7, 772.6 mm³ (E): funnel v8 (opening 191 x 168, C) clears it.
  - −Y plain panel bottom vs the insert lanes: carriage 123.5 + grabber 512.3 mm³ (E, panels v7; lanes from arm PR 79 70c3c33: carriage X −35..47.5, Y 53.75..73.75, r ≤ 159; head + grabber X ±47.5, Y 73.75..217, r 111.5..159.6, C). With v8 the grabber clears and the carriage still meets the bottom lip, 492.6 mm³ (E, no v8 file yet). Hub decision: the catch notches the lip in panel v9 plain (X ±35, Y 54..73, to r 159, C; it must start at ≤ Y 53.75, the lane's start, or 0.2 of clash stays, E). Re-measure when v8 / v9 land.
- **Option A (DEFAULT):** alternating bays at R 113 (C): 81.4 L (E).
- **Option B:** Overseer at the bottom 4, receivers clipped: 402 x 418 x 475 mm, 79.8 L (−1.6 L) (E); unclipped (B0) neighbouring receivers overlap 1020 mm³ (E), so B needs the clip.
- **Option C:** bottom slots at R 128 (C): 430 x 418 x 505 mm, 90.8 L (E); the arm strokes change. C has no lane clash: its bottom slots move the −Y lip clear without the v9 notch.
- A wins: B saves 1.6 L (2 %, E) and lowers the CG (Z −52.7 vs −35.2, E), but needs a new receiver clip on the latch and runs every Overseer swap at the bottom; C avoids the panel notch but costs 9.4 L (E) and new arm strokes, where the notch is one panel edit (hub).
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker), then the cut requirements.
