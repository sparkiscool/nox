arm v5-layout: layout check, no mesh (pure python, C)
Motors: A (53.5, 55.0), B (-55.0, -53.5), axle (53.93, -43.67) at -39 deg (C); unbalance 10.1 g.m (E)
Clearance: min 1.00 over 5x5 stroke states (wheels to the rail body, designed, C); bisector 4.0 (C)
Detent: detent.CHOSEN_V5 = roller at -135 deg, pivot side +1, levers 15 / 39, hook along the tangent (psi 315), tower at (18.5, -28.4) (C); detent.py search(v5=True) / `python3 detent.py v5`: arm parts 1.14, tower 1.0 (C); ring v1 unchanged; the 2D lever-only check above uses v1's radial hook
Compaction: 403 / 403 / 366 / 346 long (C; v4 / v5 / +C1 / compact)
Solid: n/a (no part built here; rails / wheels / pinion / eccentric bush: nox/box/arm_v5_cad.py, runs/arm_rail/v1/)
