# Overseer cage v0 (layout v2, rough, 24 Oct)

Script `nox/overseer/layout_v2.py`: the full search (~30 min, P on this box) or `V2_CASES="30,-30" python3 nox/overseer/layout_v2.py` for the chosen case (~8 min, P).
Numbers: `nox/dimensions.md` O Cage v0. Status: DESIGN, rough shapes (rods as cylinders, nodes as boxes), nothing saved or printed.

- `checks.txt`: the catch-offset search (0 / 10 / 20 / 30 aft) and every check of the chosen layout (L2 field, guard 12, funnel, plungers, camera views, overlaps, cage rods and mass, CG, PX4 offsets). The L2 field label was corrected by hand after the run (the run used the 90° normal field, L2_NEG 0; the old label said 96°).
- `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`.

Open: frame model on the rod layout (E stiffness only so far); node designs; pogo pad positions; cartridge seat height vs the Scout's (CHECK).
