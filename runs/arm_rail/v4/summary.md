# Rails A / B v4 (swap arm v7 round 3, 24 Oct 2026, swap arm system worker)

DESIGN, not in `print/` until the v7 concept check passes. Script `nox/box/arm_v7_cad.py` (arm_v7_layout LOW stack).

- Rail v2's body on the low carriage stack: top Y 66.75 (C), ridge at Y 62.75, no feet (the M5 x 12 heads sit in the body, counterbore at Y 61).
- Keel X ±9 x Y 38..54 from r 89 to r 151 (C): the free XT60 tilt goes from 0.83° (low stack without it) to 0.51° (Scout, E; `swap_system.mate_pitch`).
- Tie sockets: two Ø3.3 x 10 bores rising into the keel's bottom face at r 128 (rod end Y 41.6). The rods touch 2.3 mm³ at the mouths (C).
- 58.1 g each (M), extents 23 x 124 x 34. solid_check PASS on both (`checks.json`).
- Loads: `nox/dimensions.md` "System box swap: arm" (the round 3 frame, `runs/arm/v7/system.json`).
