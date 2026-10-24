# Pinion 16T v2 (box swap arm v6, x2, 24 Oct 2026)
16T m1 involute 20° (v1's profile, tip relief), face 10.5 (Y 62..72.5, v1 8), boss Ø14 Y 56.5..60.5, round bore below Y 59.5 (the NEMA's D-flat is only the shaft's last 15: v1's D-bore from 55.5 met the round part), M3 grub onto the flat at Y 60 (`arm_v6_cad.pinion_v2`).
```
arm_pinion16_v2: ready to print
System: box swap arm v6 (`nox/box/swap_system.py`, `nox/box/box_swap_arm.assembly.json`, dimensions System box swap: arm): fit 0.0 mm³ at 5 poses (M), sweep 298 states 0 overlaps (C), arm alone 10.4 L / 220 at park (M); Lewis 0.44 at the 60 N insert (was 0.57 on v1's 8 face), 0.56 at the 74 N stall, 0.76 a bumped head alone (E)
Weight: 3.1 g (M), was 3.0
Strength: worst index 0.76 (power cut, bumped empty head, misuse) ; on the rack tail (face 9.5) 0.84; weakest spot the tooth root (m1 teeth under the 2.0 rule by nature, as v1)
Solid: solid_check PASS (print/box_arm_pinion16_v2.stl)
Print: gear face down, bed contact 143 mm² (M), brim 5 mm, 100 % infill; file print/box_arm_pinion16_v2.stl
```
Roll on the drawn rack: 0 mm³, closest 0.063 (C). print_orient: saved 143 mm² is the largest face.

Previews: `runs/arm_pinion16/v2/preview_box_arm_pinion16_v2.png` (from `print/box_arm_pinion16_v2.stl`)
