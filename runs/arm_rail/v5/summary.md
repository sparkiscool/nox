# Rails A / B v5 (swap arm v7, 24 Oct 2026, swap arm system worker)

DESIGN, not in `print/` until hub v6's topology and the assembled recheck. Script `nox/box/arm_v7_cad.py` (arm_v7_layout LOW stack, `CHAN_X`, `CHAN_MOUTH`, `CBORE_Y`). Supersedes v4 (`runs/arm_rail/v4/`, never printed). Section v4 vs v5: https://claude.ai/artifact/B6W6b75NU1q4LCnm4BZDPp (source `section.py`, `section.html`).

- **Loads:** `nox/dimensions.md` "System box swap: arm" (model `nox/box/swap_system.py`, the round 3d frame, `runs/arm/v7/system.json`).
- **Why:** box electronics PR 103 widens each head ribbon to 5 conductors (V, G, S, summed hall, 3.3 V), ~6.1 x 1.3 (E). The v4 channel was 4.0 at the floor with a 3.2 mouth (C), and the rail bolt heads (Y 61..66, C) stood 3.25 up into it at r 40 / 78, so the folded ribbon topped at Y 68.60, 1.85 past the rail top (C).
- **Channel:** X −3.25..3.25 at the floor Y 62.75, flanks to a 5.0 mouth at the top Y 66.75 (C); 0.75 lip a side (C), as in v4. Folded ribbon tops at Y 65.35, 1.40 under the rail top (C). Wall to the body face 5.75 (C).
- **Bolts:** M5 x 10 ISO 4762, head Y 57.75..62.75 under the channel floor (counterbore Ø9 from Y 57.75, C), 3.75 of rail under the head (C; v4 7.0), tip Y 47.75 through the full 5.8 heat-set, 0.75 above the floor of hub v6's insert hole Y 47..54 (boss Y 44..54, C); counterbore wall 4.5 ≥ 3 (C).
- **Punch-through under the head:** shear ring π x 8.5 x 3.75 = 100 mm² (C) x 20 MPa (PETG across layers, E) = 2000 N (E). Worst bolt pull 1226 N in the unbraked Overseer power cut (F, the brake-failed bound; the insert pull-out reads 1.226 there first, F): index 0.61 (E). Swap bolt loads ≤ 46 N (F): 0.02.
- **Rail body:** worst index 0.485 rail A in the unbraked Overseer power cut, 0.24 Scout (F; unchanged, the frame has no channel).
- **Access:** the bolts sit under the ribbon: lift the ribbon out of the channel to reach them (system_check access 9/9 does not model the ribbon).
- **Install:** the ribbon is wider than the 5.0 mouth: slide it in from the channel's outer end, or flex it past the lips.
- 56.5 g each (M; v4 58.0: −1.5 g, the bigger channel), extents 23 x 123.5 x 34 (M). solid_check PASS on both (`checks.json`). Preview `preview_A.png`. No `budget_g` slot is set for the rails in "System box swap: arm" (soft; v3's 56.7 is the reference).
- Keel, tie sockets, inner step and park stop as v4.

System: box swap arm v7, assembled worst index 0.49 in the Overseer insert at the drone 60 N (the 16T; the rail body 0.485 only in the unbraked power cut, F), fit 0.0 mm³ at 10 poses (M), mass 56.5 g each (M; assembly 5589 g at park, E), CG on the axle within 11.7 mm (E; the rails turn with the hub, turning parts 1620 g, unbalance 18.96 g·m at park, E).
