# Rails A / B v4 (swap arm v7 round 3, 24 Oct 2026, swap arm system worker)

DESIGN, not in `print/` until hub v6's topology and the assembled recheck (the v7 concept check passed, review round 4). Script `nox/box/arm_v7_cad.py` (arm_v7_layout LOW stack).

- Rail v2's body on the low carriage stack: top Y 66.75 (C), ridge at Y 62.75 (C), no feet (the M5 x 12 heads sit in the body, counterbore at Y 61, C).
- Keel X ±9 x Y 38..54 from r 89 to r 151 (C): the free XT60 tilt goes from 0.83° (low stack without it) to 0.51° (Scout, E; `swap_system.mate_pitch`).
- Tie sockets: two Ø3.3 x 10 bores rising into the keel's bottom face at r 128 (rod end Y 41.6). The rods touch 2.3 mm³ at the mouths (C).
- 58.0 g each (M), extents 23 x 123.5 x 34 (M). solid_check PASS on both (`checks.json`).
- Inner step and park stop from r 27.5 (round 3d piece pass: 1.0 off detent ring v4, r 26.5). Mass 58.0 g each (M) vs rails v3 56.7 g: +1.3 g buys the keel (the XT60's free tilt from 0.83° to 0.51°, Scout, low stack). No `budget_g` slot is set for the rails in "System box swap: arm" (soft; v3's 56.7 is the reference).
- Loads: `nox/dimensions.md` "System box swap: arm" (the round 3d frame, `runs/arm/v7/system.json`).

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (the 16T; the rail body 0.12 in the dropped Overseer pack, the pad inserts 0.59 / 1.24 only in the unbraked power cut, the brake-failed bound the brake gate removes, E), fit 0.0 mm³ at 10 poses (M), mass 58.0 g each (M), CG n/a (a fixed rail; the turning arm's unbalance 19.1 g·m at park, E).
