# Rails A / B v2 (box swap arm v6, 24 Oct 2026, swap arm system worker)
Rail v1 on the v6 stack (`nox/box/arm_v6_cad.py rail_v2`): Y 54..73 straight on the hub v6 motor plate (no posts), r 29..134 (r 27 above Y 65 over the 80T flange), park stop X ±9 x Y 73..77 x r 27..31, one M5 x 16 per pad at r 40 / 78 from the top into the hub's heat-set inserts, ribbon channel X −2..2. A and B differ only in the dimples. No carbon tie sockets (rejected alternative). Loads from the system pass (C geometry, E loads).
```
arm_rail_A_v2 / arm_rail_B_v2: DESIGN (v6 concept check FAIL x2; READY in v7 after its concept check)
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`), assembled worst index 0.28 in head stall free (rail A r 78..100, hand section E; was 0.23 in v5), fit 0.0 mm³ at 5 poses (M), mass 3232 g (E, the assembly with its context), CG 7.0 mm off the axle (E; the turning parts' unbalance 22.5 g·m at park, M / E)
Weight: 49.4 g each (M), was 49.7 / 50.0
Strength: worst index 0.28 in head stall free (misuse ≤ 1.0); weakest spot rail A r 78..100 (outer pad to the wheels); wheel axial 420 N, pad 233 N + 6.6 N·m (E)
Solid: solid_check PASS (runs/arm_v6_parts/arm_rail_A_v2_W.stl, arm_rail_B_v2_W.stl (meshes; out of print/))
Print: bottom (Y 54) down, bed contact 2403 mm² (M), no supports (the stop bridges the 3.2 channel mouth); files runs/arm_v6_parts/arm_rail_A_v2_W.stl, arm_rail_B_v2_W.stl (meshes; out of print/)
```
print_orient: saved bed 2403 mm², largest of the 6 faces (+x 240, −y 338), first layer 1 island, thin-island run 0 mm.

Previews (rendered from the files this PR first saved in print/, now removed until v7): `preview_box_arm_rail_A_v2.png`, `preview_box_arm_rail_B_v2.png`
