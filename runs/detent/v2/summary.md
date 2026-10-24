detent arm v2: ready to print
Weight: 2.3 g (M), was 2.4 g with the printed spacer (M, v1)
Strength: worst index ~0.22 (E, hand calc) in the 8.4 N (E) spring pull at the hook, in-plane bending at the pivot ~10 MPa (E) vs 45 in-layer (E); weakest spot the arm beside the pivot hole   [box part: fits (C): `python3 detent.py v5` arm parts 1.14, tower 1.0; sweep `arm_kinematics.py 1.0 v5` 0 overlaps, detent spacer over the notch 0.40 (designed); no supports]
Solid: solid_check PASS
Print: arm top face (Y 74.1, C) down, bed contact 714 mm2 (M), no supports, brim not needed; 100 % infill; file print/box_detent_arm_v2.stl

Placement `detent.CHOSEN_V5`: roller at -135 deg (C), pivot side +1, levers 15 / 39 (C), hook along the tangent (psi 315); ring v1 unchanged; holding 0.390 N.m per spring (E) vs 0.422 (E, v5 unbalance): two springs, margin 1.85.
DEFAULT: 2 x DIN 988 3 x 5 x 0.5 shims under the arm (v1's printed spacer was a 0.65 wall). Tower for hub v6: runs/detent/v2/detent_tower_v5_W.stl.
