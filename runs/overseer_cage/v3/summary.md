# Overseer cage v3 (layout A seed search, 24 Oct)

`CAGE_CRASH_G=35 python3 nox/overseer/cage_opt2.py runs/overseer_cage/v3/params_seed.json runs/overseer_cage/v3/cage_opt2.json 100`: all 1024 topologies of `cage_opt2.py` (fully stressed sizing per rod run, symmetric left / right) on the layout A seed geometry (`params_seed.json`), crash 35 g x 3000 g (E), L2 mode ≥ 100 Hz. 96 feasible, best 827 g (C).

Superseded as the final cage: the seed geometry is not the final one, and the constraints grew after this run (funnel and motor clearance, belly frame and skid, sensor modes, 12 to 20 mm tubes, two front cameras). The system model now re-runs the same search on its own final geometry (`nox/overseer/overseer_system.py`, `SYS_SEARCH=1`) and re-sizes the cage until it settles: the final cage is in `runs/overseer_system/v2A` (`cage_final.json`). Kept as the seed record and as the system model's starting table.
