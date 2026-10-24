# Pinion 16T v2 (box swap arm v6, x2, 24 Oct 2026)
16T m1 involute 20° (v1's profile, tip relief), face 10.5 (C) (Y 62..72.5, C; v1 8), boss Ø14 Y 56.5..60.5 (C), round bore below Y 59.5 (C) (the NEMA's D-flat is only the shaft's last 15: v1's D-bore from 55.5 met the round part), M3 grub onto the flat at Y 60 (`arm_v6_cad.pinion_v2`).
```
arm_pinion16_v2: DESIGN (v6 concept check FAIL x2; READY in v7 after its concept check)
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`), assembled worst index 0.56 in head stall into a jammed pack 74 N (Lewis in-layer; 0.76 a bumped head alone; was 0.57 at 60 N on v1), fit 0.0 mm³ at 5 poses (M), mass 3232 g (E, the assembly with its context), CG 7.0 mm off the axle (E; the turning parts' unbalance 22.5 g·m at park, M / E)
Weight: 3.1 g (M), was 3.0 (M): the 0.1 g is the 2.5 longer face (10.5 vs 8) that takes the 60 N insert's Lewis index from 0.57 to 0.44
Strength: worst index 0.76 (power cut, bumped empty head, misuse) ; on the rack tail (face 9.5) 0.84; weakest spot the tooth root (m1 teeth under the 2.0 rule by nature, as v1)
Solid: solid_check PASS (the print-pose mesh that was print/box_arm_pinion16_v2.stl; W mesh `runs/arm_v6_parts/arm_pinion16_v2_W.stl`)
Print: gear face down, bed contact 143 mm² (M), brim 5 mm, 100 % infill; mesh `runs/arm_v6_parts/arm_pinion16_v2_W.stl` (out of print/ until v7)
```
Roll on the drawn rack: 0 mm³, closest 0.063 (C). print_orient: saved 143 mm² is the largest face.

Previews (rendered from the files this PR first saved in print/, now removed until v7): `preview_box_arm_pinion16_v2.png`
