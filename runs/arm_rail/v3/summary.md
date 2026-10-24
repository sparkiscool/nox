# Rails A / B v3 (box swap arm v7, 24 Oct 2026, swap arm system worker)
**Superseded by rails v4 (round 3, `runs/arm_rail/v4/`): the low carriage stack. These meshes left `print/`.**
Rail v2 on the v7 stack (`nox/box/arm_v7_cad.py rail_v3`): Y 54..73 on the hub v6 motor plate, **r 29..151** (r 27 above Y 65 over the 80T; the outer wheel at the commanded drone insert z0 111.3 has 6.4 of ridge past it, C), park stop X ±9 x Y 73..77 x r 27..31, one M5 x 16 per pad at r 40 / 78 into the hub's heat-set inserts, ribbon channel X −2..2, **two Ø3.3 x 10 carbon tie sockets** in the bottom face at r 128 (rods 2 x 3.0 x 62.5 per rail to the hub's lower rim at Y 22.5 / r 78, epoxied). The tie sockets sit 0.0 mm³ in the rods. Loads from the v7 system pass (C geometry, E loads).
```
arm_rail_A_v3 / arm_rail_B_v3
System: box swap arm v7 (`nox/box/swap_system.py` NOX_ARM v7, `nox/box/box_swap_arm.assembly.json`, dimensions System box swap: arm): fit 0.0 mm³ at 5 poses (M), sweeps 0 arm overlaps (Scout and Overseer cycles, C), arm alone 9.9 L / 236 at park (M)
Weight: 56.7 g each (M), was 49.4 (v2: r 134)
Strength: rail worst index 0.11 (dropped Overseer, hand section E); wheel axial 553 N and pad bolt 350 N in the Scout power cut carrying (the fallback until the head brake is tested)
Solid: solid_check PASS (print/box_arm_rail_A_v3.stl, print/box_arm_rail_B_v3.stl)
Print: bottom (Y 54) down, bed contact 2681 mm² (M, the largest of the 6 faces), no supports (the stop bridges the 3.2 channel mouth; the Ø3.3 sockets rise into the bed face); files print/box_arm_rail_A_v3.stl, print/box_arm_rail_B_v3.stl
```
Previews: `runs/arm_rail/v3/preview_box_arm_rail_A_v3.png` (from `print/box_arm_rail_A_v3.stl`), `runs/arm_rail/v3/preview_box_arm_rail_B_v3.png` (from `print/box_arm_rail_B_v3.stl`)
