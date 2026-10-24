# Rails A / B v5 (swap arm v7, 24 Oct 2026, swap arm system worker)

DESIGN, not in `print/` until hub v6's topology and the assembled recheck. Script `nox/box/arm_v7_cad.py` (arm_v7_layout LOW stack, `CHAN_X`, `CHAN_MOUTH`, `CBORE_Y`). Supersedes v4 (`runs/arm_rail/v4/`, never printed).

- **Why:** box electronics PR 103 widens each head ribbon to 5 conductors (V, G, S, summed hall, 3.3 V), ~6.1 x 1.3 (E). The v4 channel was 4.0 at the floor with a 3.2 mouth, and the rail bolt heads (Y 61..66) stood 3.25 up into it at r 40 / 78, so the folded ribbon had only 0.75 over them (C).
- **Channel:** X −3.25..3.25 at the floor Y 62.75, flanks to a 5.0 mouth at the top Y 66.75 (C); 0.75 lip a side as in v4. Depth 4.0 takes the ribbon folded on itself (2 x 1.3 + the fold, E). Wall to the body face 5.75 (C).
- **Bolts:** M5 x 10 ISO 4762, head Y 57.75..62.75 under the channel floor (counterbore Ø9 from Y 57.75, C), 3.75 of rail under the head (C), tip Y 47.75 through the full 5.8 heat-set in the hub boss Y 46..54 (C); counterbore wall 4.5 ≥ 3 (C). Access 9/9 clear (system_check, M).
- **Install:** the ribbon is wider than the 5.0 mouth: slide it in from the channel's outer end, or flex it past the lips.
- 56.5 g each (M; v4 58.0), extents 23 x 123.5 x 34 (M). solid_check PASS on both (`checks.json`). Preview `preview_A.png`.
- Keel, tie sockets, inner step and park stop as v4.

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (the 16T; unchanged by the rail), fit 0.0 mm³ at 10 poses (M), mass 56.5 g each (M; assembly 5589 g at park, E), CG n/a (a fixed rail; the turning arm's unbalance 18.96 g·m at park, E).
