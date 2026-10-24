# Rails A / B v2 (box swap arm v6, 24 Oct 2026, swap arm system worker)
Rail v1 on the v6 stack (`nox/box/arm_v6_cad.py rail_v2`): Y 54..73 straight on the hub v6 motor plate (no posts), r 29..134 (r 27 above Y 65 over the 80T flange), park stop X ±9 x Y 73..77 x r 27..31, one M5 x 16 per pad at r 40 / 78 from the top into the hub's heat-set inserts, ribbon channel X −2..2. A and B differ only in the dimples. No carbon tie sockets (rejected alternative). Loads from the system pass (C geometry, E loads).
```
arm_rail_A_v2 / arm_rail_B_v2: ready to print
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`, dimensions System box swap: arm): fit 0.0 mm³ at 5 poses (M), sweep 298 states 0 overlaps (C), arm alone 10.4 L / 220 at park (M); rail worst index 0.28 in the 74 N stall without the receiver (hand section, E; v1 on posts 0.23 at 60 N)
Weight: 49.4 g each (M), was 49.7 / 50.0
Strength: worst index 0.28 in head stall free (misuse ≤ 1.0); weakest spot rail A r 78..100 (outer pad to the wheels); wheel axial 420 N, pad 233 N + 6.6 N·m (E)
Solid: solid_check PASS (print/box_arm_rail_A_v2.stl, print/box_arm_rail_B_v2.stl)
Print: bottom (Y 54) down, bed contact 2403 mm² (M), no supports (the stop bridges the 3.2 channel mouth); files print/box_arm_rail_A_v2.stl, print/box_arm_rail_B_v2.stl
```
print_orient: saved bed 2403 mm², largest of the 6 faces (+x 240, −y 338), first layer 1 island, thin-island run 0 mm.

Previews: `runs/arm_rail/v2/preview_box_arm_rail_A_v2.png` (from `print/box_arm_rail_A_v2.stl`), `runs/arm_rail/v2/preview_box_arm_rail_B_v2.png` (from `print/box_arm_rail_B_v2.stl`)
