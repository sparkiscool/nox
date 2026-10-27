# Overseer front arm v6: concept (27 Oct, the system pass after front v5c / v5d)

Page (27 Oct): https://claude.ai/artifact/MB3PnJE9y3nJdXq1EtGi4Z (source `nox/overseer/arm_page_v6.py` -> `runs/overseer_arms/v6/arms.html`).

Why: front v5d read 131 Hz vertical / 162 Hz lateral (M) against 190 (dimensions System Overseer: arms); the 4 x 6/4 chords bend almost alone. Third front miss, so a system pass.

Concept (C unless tagged): 12 x 3.0 carbon diagonals, Warren, 3 panels on the side, top and bottom faces, 7 off each face; rods 50.2..71.0 (all past the 33 break-even); each blind at its motor-side end, slid in from the root-side end (`--rodcheck-setup` PASS, 16/16, `rodcheck_setup.json`); drawn 2.65 sleeves and node blocks (65.3 cm³), a solid shoulder wedge (91.4 cm³) where the vertical shear crosses into the root. Loads, spoke node, root and chords as v5d.

Options (modal what-if on the v5d arm, 4 inserts, h 1.5, E; `whatif_modes.json`, `tri10/whatif_modes.json`):
| Option | Vertical / lateral Hz | Grid mass g |
|---|---|---|
| v6 chosen (diagonals + nodes + solid wedge) | 197.8 / 233.6 | 326.9 |
| tri10 (diagonals + nodes + 10 mm triangle walls) | 192.3 / 225.9 | 318.1 |
| v5d as is | 131.2 / 162.3 | 249.6 |
DEFAULT: the solid wedge, 9 g for 5.5 Hz more margin; tri10 is not run (its margin, 2.3 Hz on a what-if, is inside the what-if's own error: staircase rods read stiff, E).

Mass: about +77 g a front arm on the grid (M on the what-if), so ~305 g + tubes against the 60 g soft slot, ~+150 g for the pair. It buys the 190 Hz first mode (1P full is 158 Hz: v5d's 131 Hz vertical sat inside the 1P band, a resonance in flight). The topology rerun moves some of it.

Strength: v5d read 0.62 (cage crash 6) against the 0.5 target; v6 adds carbon in shear, so E ≤ 0.62; the run reports it.

System: +150 g at the front moves the assembled CG forward (the rear v5d left it 5.8 mm aft, M): the cage session rebalances once with the final front mass (its DEFAULT, 27 Oct); system_check with the built v6 follows the run.

Run: `arm_topo_v6.py front --gen 45`, started 19:15 UTC, smoke 0.39..0.97 h (E). On its mesh: modes at h 1.5 and 1.0 on faces and inserts; under 190 is a fourth miss and goes back to the system pass, not a v6b.
