# Overseer assembly v0 (seed, 24 Oct)

`python3 nox/assembly_seed.py overseer` (~8 min): `nox/overseer/layout_v2.py` definitions, the chosen case of `runs/overseer_layout/v2` re-solved (motors 52.10 ahead, L2 base x 336.36, camera 74 up: matches the run), meshes here, file `nox/overseer/overseer.assembly.json`. Layout mass model reproduced: 2332.3 g (E) both ways.

`check.txt` = `python3 nox/assembly.py nox/overseer/overseer.assembly.json --check --list`:
System: Overseer, cage first -62.4 mm (E) at (+0.95, +0.02, -0.32) (Unitree L2) FAIL, fit 19375.2 mm³ (E) (belly sled + pods/cartridge) FAIL, bbox 85.7 L (E), extents 634 x 570 x 237 mm (E), mass 2332 g (E), CG 1.1 mm off thrust axis (E) [-0.1, +1.1, -75.1], 33.9 mm without swap parts FAIL (tol 3)
