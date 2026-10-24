# Box structure v2 layout (concept, `nox/box/box_layout_v2.py`)

Inputs:
- Spark 24 Oct: 9 bays, 4 Overseer + 5 Scout.
- Hub DEFAULT (arm v7): no two Overseer bays adjacent; common grip line **Y_C 165** (C; alt 138 or 175, `nox/log.md`).
- Swap arm v6 / v7: R 113, lane ≥ 159, rail disc r 151 at Y 38..73, drone-insert keep-outs from the v7 head (C).
- Battery latch v1 (PR 69 258becf): receivers in `runs/box_system/v2/latch_ctx/` (M, `latch_ctx_export.py`), each placed by its tab line on the grip line (hub DEFAULT: the arm grips there; the Scout set's mid sits 5.4 off the funnel centre); receiver bolts up from below into inserts in the bay bodies.

Bays: Overseer 0, −60, −120, −180; Scout 30, −30, −90, −150, −210 (DEFAULT, set A). Funnel centre = the grip line, Y 165 (C).

Visuals:
- Four layouts side by side, sections across and along the axle from the concept meshes: https://claude.ai/artifact/Mn5JGDkjEGSQXyPhjqrpxb (source `runs/box_system/v2/layout_views.html`, `nox/box/box_layout_v2_views.py`).
- 3D scene of A: https://claude.ai/artifact/AioyNWKzpTH96znsmfrAXp (`layout.html`).

System file: `nox/box/box_structure_v2.assembly.json` (= `concept.assembly.json` of A), with both drone-insert keep-outs as parts so the catch and arm can check against them.

## Concept check (docs/generative.md rule 0)
- System line (`concept_check.json`): System: Box structure v2 layout, cage first 3.0 mm (E) at -Y (PSU LRS-350-24), fit 0.0 mm³ (E) (bay -30 body envelope/bay -60 receiver), bbox 80 L (E), extents 418 x 407 x 470 mm (E), mass 18472 g (E), CG [4.0, 148.4, -36.9] (E, no cg_ref), access 18/18 clear
- **Arm:** hub, motors, parked heads, rail disc and both insert lanes clear every bay envelope, the v1 mast and the PSU (0 mm³, M).
- **Access 18 / 18:** every cartridge comes out to park along its install axis; every bay body comes out along +Y with the front sheet off (the service door, DEFAULT).
- **Fit PASS with funnel v8 plain** (catch PR 81, the catch file now in W, loaded as is): the Overseer drone set passes the 191 x 168 opening and both drone-insert lanes (arm PR 79 70c3c33: carriage X −35..47.5, Y 53.75..73.75, r ≤ 159; head + grabber X ±47.5, Y 73.75..217, r 111.5..159.6, C) clear the −Y plain panel: its raised bottom edge is at r ≥ 159.61 over the carriage band (0.61 over the lane, C on M). `v8_lane_check.json` reads 0 mm³ (M). The panel v9 notch is no longer needed for fit. With panels v7 it read 980.6 (drone set) and 123.5 / 512.3 mm³ (lanes) (E).
- **Option A (DEFAULT):** alternating bays at R 113 (C): 418 x 407 x 470 mm, 80.0 L (E); v8 needs no extra Y room (its opening grows by raising the panel bottom).
- **Option B:** Overseer at the bottom 4, receivers clipped: 406 x 407 x 476 mm, 78.7 L (−1.3 L) (E); unclipped (B0) neighbouring receivers overlap 981.7 mm³ (E), so B needs the clip.
- **Option C:** bottom slots at R 128 (C): 434 x 407 x 506 mm, 89.4 L (E); the arm strokes change.
- A wins: B saves 1.3 L (2 %, E) and lowers the CG (Z −54.8 vs −36.9, E), but needs a new receiver clip on the latch and runs every Overseer swap at the bottom; C costs 9.4 L (E) and new arm strokes for nothing now that v8 clears the lanes.
- Next: the coarse system topology over the joined envelope (`runs/box_system/v2/topo/`, worker), then the cut requirements.
