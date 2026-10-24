# Pinion16 v3 (swap arm v7 round 3d, 24 Oct 2026, swap arm system worker)

**DESIGN**, not in `print/` until the v7 piece pass's assembled recheck. Script: `nox/box/arm_v7_cad.py pinion` (`pinion_v3`). Requirements: `nox/dimensions.md` "System box swap: arm", row Pinion16 v3.

- 16T module 1, the v2 involute. Teeth Y 59.75..73.25 (face 13.5, C), fully engaged on the rack, which is grown down to Y 59.25..73.25 (concept review r3). On the rack tail (the drone insert, from its step at Y 63) 10.25 engages.
- D-bore on the NEMA 17HS19's flat (Y 59..74) carries the torque. Boss Ø14, Y 55..59.25, runs into the teeth. M3 grub from −X at Y 57.1 on the round shaft, for axial hold only.
- Lewis (frame, E): 0.45 / 0.49 (Scout / Overseer) at the 60 N drone insert (swap ≤ 0.5); 0.57 / 0.61 at the 74 N stall; 0.66 dropped Overseer (misuse ≤ 1.0).
- 3.7 g (M), extents 17.8 x 17.8 x 18.2, printed with the boss face down: bed 143 mm², overhang 44 mm² (print_orient, M).
- solid_check (`python3 nox/solid_check.py runs/arm_pinion16/v3/box_arm_pinion16_v3.stl`): **PASS** (bodies 1, no necks, no thin walls).
- Preview: `preview_box_arm_pinion16_v3.png`.
- System: box swap arm v7 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`). Sweeps 0 overlaps with the v3 tooth band on both cycles; fit 0.0 mm³ at 10 poses (M).
