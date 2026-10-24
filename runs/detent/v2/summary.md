# Detent arm v2 (24 Oct 2026; arm-mech worker drew it for CHOSEN_V5, rechecked by the swap arm system worker on v6)
Tags: numbers C geometry (drawn), M from the meshes, E loads, stiffnesses and strengths, unless tagged in place.
The v1 lever with the hook along the tangent (`detent.arm_solids`), 58.6 x 19.6 x 2.5 (M), 2.3 g (M), flat, 2 x DIN 988 3 x 5 x 0.5 shims instead of v1's printed spacer. On v6 it runs at `detent.CHOSEN_V6`: the same roller (−135°, r 25.5), levers 15 / 39 and hook as CHOSEN_V5, the tower in hub v6 at (21.2, −30.3) (was (18.5, −28.4)), with detent ring v3 (was ring v1). Gaps arm parts 1.0, tower 1.0 (C).
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`), assembled worst index 0.22 in the detent's spring load (8.4 N at the hook, in-plane bending at the pivot, in-layer, E; was 0.22), fit 0.0 mm³ at 5 poses (M), mass 3232 g (E: printed parts from their meshes, M; bought parts from their data, E), CG 7.0 mm off the axle (E; the turning parts' unbalance 22.5 g·m at park). State DESIGN with the v6 pieces (concept check FAIL x2); READY again in v7.
Weight: 2.3 g (M), was 0.6 g (C, v1 lever): the 1.7 g are the longer hook arm along the tangent (b 39) and the 2.5 thick lever the 8.4 N spring bends (index 0.22)
Solid: solid_check PASS (the print-pose mesh, now `runs/arm_v6_parts/detent_arm_v2_print_pose.stl`, out of print/ until v7)
Print: arm top face (Y 74.1) down, bed contact 714 mm² (M), no supports
Previews: `runs/detent/v2/preview.png` (rendered from that print-pose mesh)
