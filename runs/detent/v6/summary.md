# Detent v6: ring v4, arm v3, 623ZZ roller (swap arm v7 piece pass, 24 Oct 2026, swap arm system worker)

**DESIGN.** Nothing goes to `print/` until the v7 assembled recheck with hub v6. Script: `nox/box/detent_v6.py` (search, export). The parts live in `detent.py` (`V6_PARTS`, `use_v6`, `CHOSEN_V8`), and `arm_v7_layout.use()` switches them on. Requirements: `nox/dimensions.md` "System box swap: arm".

**Why v6:** the round 3d recheck at springs x1.3 (C / E) failed two parts.
- Ring v3's 2.1 rim under the 683ZZ roller (Ø7 x 3) read about 54 MPa Hertz, index about 0.8. That contact repeats every swap (Spark: the latch must hold under repetition).
- Arm v2 read 0.68 at its pivot.

**v6:**
- **Roller:** 623ZZ (Ø10 x 4, bought, E) on the 3.0 carbon axle (bore 3), as v2.
- **Ring v4:** r 12.25..26.5, Y 64..69.9 (C). Rim Y 65.9..69.9 = the roller's width (C). Notch apex r 20.0 (C), cut through where the floor would be under 2.0. Walls to the 80T's bolt pockets 2.24 / 2.34 (C).
- **Arm v3:** plate Y 70.9..74.4 (3.5, C), bosses r 6.0 (C).
- **Tower:** webs from Y 65.5 (C), 1.5 over the 80T (C).
- **Placement CHOSEN_V8 (C):** th −105, a 13, b 31.2, spring toward −75, hook ψ 315. Tower Q (21.92, −28.56) (C), found by `detent_v6.search_free` on the v7 obstacles. Arm parts and tower clear by 1.0 (C).
- **Hold:** two springs x1.3 give **1.065 N·m** (E). That is the DET_HOLD in `swap_system`. Index past a notch at the slow pass, the parked carry and the head-out case are in the frame results.

**Stress** (two springs on a land, hook 10.6 N, roller 24.7 N, E):
- rim Hertz 30.6 MPa, index **0.45** (of 1.5 x 45);
- arm at its pivot 7.4 MPa, index **0.16**.

**Meshes** (PETG, print poses):

| Part | Mass (M) | Extents | Bed | Overhang | Height | solid_check |
|---|---|---|---|---|---|---|
| `box_detent_ring_v4.stl` | 8.7 g | 53 x 53 x 5.9 | 1102 mm² | 96 mm² | 5.9 | **PASS** |
| `box_detent_arm_v3.stl` | 3.3 g | 51.6 x 27.6 x 3.5 | 741 mm² | 0 | 3.5 | **PASS** |

Previews: `preview_box_detent_ring_v4.png`, `preview_box_detent_arm_v3.png`.

**Mass vs previous:** ring v4 8.7 g (M) vs ring v3 6.7 g (M): +2.0 g buys the rim the roller's width and r 26.5 (rim index 0.8 → 0.45); arm v3 3.3 g (M) vs arm v2 2.3 g (M): +1.0 g buys the 3.5 plate and r 6 bosses (0.68 → 0.16). No budget_g slots for the detent in "System box swap: arm" (soft; v3 / v2 the reference).

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (the 16T; the detent's own worst 0.45, the rim; was ring v3 / arm v2 ~0.8 / 0.68 at x1.3, E), fit 0.0 mm³ at 10 poses (M), mass 8.7 + 3.3 g (M), CG of the turning arm on the axle within 11.8 mm (unbalance 19.1 g·m at park, E). Hub v6's drawn tower moves to the new Q; its topology run restarts on it; the assembled sweeps with these parts read 0 overlaps (C).
