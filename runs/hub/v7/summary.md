# hub v7 (topo, after the hub's three-strikes system pass; arm v7 round 3d frame, detent v6 tower): solid PASS

Script `nox/box/hub_v6_topo.py 2.05 62 runs/hub/v7` (`NOX_ARM=v7`), drawn bodies `nox/box/hub_v6.py` (hub v7: the 14 x 14 detent column inside the axle loop on a 3.0 plate pad, the generated region 1.0 into the plate; `runs/hub/v7-concept/`, concept review PASS). Branch `run/hub-v6r2`, PR 111. DESIGN until the assembled recheck.

- **solid_check PASS** on the exact file (`python3 nox/solid_check.py runs/hub/v7/box_hub_v7.stl`): 1 plain body, 1 eroded, no necks, no thin walls, drawn_not_in_main [], 1 acute edge (M). The engine's in-run check reads drawn body 0.0 at 0.997 after a 2.0 erosion (as runs v6r2 / v6r3); the gate is the file check.
- **Mass 213.3 g** (M; budget_g 215 DEFAULT; v6 run 3 199.3 g, hub v5 214 g). The extra 14 g over run 3 buys the tower column (sustained 1.08 → 0.60) and swap headroom elsewhere.
- **Strength (F, class max of in-layer / across, 69 cases + the detent springs):**
  - sustained 0.596, detent springs, at (18.9, 64.0, −25.1) (target ≤ 1.0: met; run 3 1.075).
  - misuse 0.468, hand on rail A tip 50 N, at (49.7, 51.7, −18.9) (target ≤ 1.0: met).
  - swap 0.524, pull at bay −60 50 N [Overseer], at (37.4, 47.6, 22.1) (target ≤ 0.5: 5 % over). The same point and value as run 3 (0.523 at 52 cm³), unmoved by +10 cm³; the h 3.0 test of the same layout read 0.32 there. A value that grows with grid refinement and ignores added material at a re-entrant corner of the drawn roof (r 43, Y 47.5) is the voxel model's sharp corner, where the printed part carries the organic finish's 2.5 fillet (E). Swap elsewhere ≤ 0.5.
- **Print:** sleeve end down (−y): bed 743 mm², height 66.9, one first-layer island 756 mm², footprint 170.6 x 169.3 (A1 mini 180: fits), overhang 15 493 mm² (tree supports) (M, `print_orient --all`). The W file is in the arm frame; the print file is posed −y down.
- h 2.05, grid 96 x 40 x 96; peak 9.56 GB (engine layout), 10.5 GB RSS sum (poll); wall 2862 s.

System: box swap arm v7, assembled worst index 0.60 in the detent springs, sustained (was 1.08 hub v6 run 3), fit function check ok (skins complete, nothing in cuts or outside the envelope), mass 213.3 g (M), CG n/a (the assembled recheck follows).
