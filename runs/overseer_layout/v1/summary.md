# Overseer layout v1 (rough, 24 Oct)

Script `nox/overseer/layout_v1.py`; rerun the chosen case with `V1_CASES="30,-34" python3 nox/overseer/layout_v1.py` (~8 min), the full search without it (~25 min).
Numbers: `nox/dimensions.md` O Layout v1. Status: DESIGN, rough shapes, nothing saved or printed.

- `checks.txt`: every check of the chosen layout (L2 field, camera views, overlaps, funnel, mass, CG, PX4 offsets).
- `search_before_arm_fix.txt`: the catch-offset search (0 / 10 / 20 / 30 aft) with the first arm sizes (front 22 deep) and the misfacing L2 back block; it set the choice (30 aft: shortest front arms). The chosen case was rerun with the fixed geometry (front 30 deep, 32 wide).
- `motion.html` (artifact), `motion.mp4` / `.gif` / `.png`.

Open: front arm first mode ~161 Hz (E) sits between 1P full and 2P hover, frame model when drawn; cartridge seat height vs the Scout's (CHECK); camera bracket 74 up the L2 back plane; wheel legs up to 81 in plan.
